// ================= ADC.cpp =================
#include "ADC.h"
#include <Wire.h>
#include <math.h>

// ===== DEVICE ADDRESSES =====
#ifndef ADS1_ADDR
#define ADS1_ADDR 0x49   // output bridge
#endif
#ifndef ADS2_ADDR
#define ADS2_ADDR 0x48   // input bridge
#endif

// ===== ADC CONFIG =====
#ifndef MAX_SAMPLES
#define MAX_SAMPLES 32
#endif
#ifndef ADS_DR_860SPS
#define ADS_DR_860SPS 0x00E0
#endif

#define ADS_REG_CONVERSION      0x00
#define ADS_REG_CONFIG          0x01
#define ADC_CHANNELS            4
#define ADC_CONVERSION_WAIT_MS  2UL    // at 860 SPS one conversion ≈ 1.16ms
#define ADC_RESPONSE_TIMEOUT_MS 20UL   // only checked while WAITING, never idle

// ===== AD8310 CALIBRATION TABLES =====
// Formula: power_W = exp((mV - offset) / slope)
// Slopes are in mV/Np (neper) — calibrated empirically using exp() formula.
// Forward channels: fixed slope, per-band offset.
// Reflected channels: per-band slope AND offset.
// Index 0 = fallback (band unknown).  1–5 = bands 3.5/7/14/24/29 MHz.

#define NUM_BANDS 6

static const float ADC1_FWD_SLOPE = 105.51104f;
static const float ADC1_FWD_OFFSET[NUM_BANDS] = {
  1805.3561f,   // [0] fallback
  1810.8410f,   // [1] 3.5 MHz
  1805.5594f,   // [2] 7 MHz
  1804.5779f,   // [3] 14 MHz
  1803.8650f,   // [4] 24 MHz
  1801.9371f,   // [5] 29 MHz
};

static const float ADC2_FWD_SLOPE = 104.12791f;
static const float ADC2_FWD_OFFSET[NUM_BANDS] = {
  1825.9539f,   // [0] fallback
  1832.3248f,   // [1] 3.5 MHz
  1825.7759f,   // [2] 7 MHz
  1825.4426f,   // [3] 14 MHz
  1823.4430f,   // [4] 24 MHz
  1822.7833f,   // [5] 29 MHz
};

static const float ADC1_REF_OFFSET[NUM_BANDS] = {
  1838.2400f,   // [0] fallback (average of bands)
  1896.3457f,   // [1] 3.5 MHz
  1878.5945f,   // [2] 7 MHz
  1854.5676f,   // [3] 14 MHz
  1799.7759f,   // [4] 24 MHz
  1761.9060f,   // [5] 29 MHz
};
static const float ADC1_REF_SLOPE[NUM_BANDS] = {
   99.16f,      // [0] fallback
   95.6308f,    // [1]
   98.4622f,    // [2]
   97.9788f,    // [3]
  100.1477f,    // [4]
  103.5750f,    // [5]
};

static const float ADC2_REF_OFFSET[NUM_BANDS] = {
  1910.9000f,   // [0] fallback (average of bands)
  1902.9500f,   // [1] 3.5 MHz
  1893.2996f,   // [2] 7 MHz
  1901.7076f,   // [3] 14 MHz
  1923.8122f,   // [4] 24 MHz
  1932.7205f,   // [5] 29 MHz
};
static const float ADC2_REF_SLOPE[NUM_BANDS] = {
   98.82f,      // [0] fallback
   94.0665f,    // [1]
   94.9633f,    // [2]
   98.2110f,    // [3]
  103.1205f,    // [4]
  103.7531f,    // [5]
};

// ===== INTERNAL TYPES =====
enum ADCState { ADC_IDLE, ADC_WAITING };

struct ADCDevice {
  uint8_t  addr;
  ADCState state;
  uint8_t  currentChannel;
  uint8_t  scheduleIndex;

  int16_t  buffers[ADC_CHANNELS][MAX_SAMPLES];
  uint8_t  index[ADC_CHANNELS];

  float    filteredLog[ADC_CHANNELS];
  float    filteredPower[ADC_CHANNELS];
  float    peakPower[ADC_CHANNELS];
  bool     initialized[ADC_CHANNELS];

  int16_t       lastRaw[ADC_CHANNELS];
  unsigned long conversionStarted;

  // Active calibration — ch0=reflected, ch1=forward
  // Set by ADC_setBand(), consumed by processSampleLog()
  float calOffset[2];
  float calSlope[2];
};

static ADCDevice adc1;
static ADCDevice adc2;

// Weighted schedule: FORWARD (ch1) sampled 4×, reflected (ch0) 2×,
// slow channels (ch2, ch3) once each per cycle.
static const uint8_t schedule[]     = {0, 0, 1, 0, 2, 1, 0, 3};
static const uint8_t scheduleLength = sizeof(schedule) / sizeof(schedule[0]);

static bool useADC1      = true;
static bool adcFaultFlag = false;

// ===== ADC FAULT RATE LIMITING =====
// Single transient I²C errors or response timeouts (e.g. caused by occasional
// slow main-loop iterations during verbose Serial output) shouldn't flip the
// system into a hard fault. adcFaultFlag is now only set when N timeouts
// occur within T ms — the failure must be sustained, not a one-off.
// Tune the two constants below as needed.

#define ADC_FAULT_WINDOW_COUNT 5
#define ADC_FAULT_WINDOW_MS    1000UL

static unsigned long faultTimes[ADC_FAULT_WINDOW_COUNT] = {0};
static uint8_t       faultIdx = 0;

static void recordADCFault() {
  unsigned long now = millis();
  faultTimes[faultIdx] = now;
  faultIdx = (faultIdx + 1) % ADC_FAULT_WINDOW_COUNT;

  // After writing, faultIdx points to the OLDEST entry in the ring.
  // If that oldest is non-zero (ring full) and within the window, we've
  // had ADC_FAULT_WINDOW_COUNT faults in <= ADC_FAULT_WINDOW_MS — real fault.
  unsigned long oldest = faultTimes[faultIdx];
  if (oldest != 0 && (now - oldest) < ADC_FAULT_WINDOW_MS) {
    adcFaultFlag = true;
  }
}

// ===== CALIBRATION =====
void ADC_setBand(uint8_t band) {
  if (band >= NUM_BANDS) band = 0;

  // ADC1 output bridge: ch0=reflected, ch1=forward
  adc1.calOffset[0] = ADC1_REF_OFFSET[band];
  adc1.calSlope[0]  = ADC1_REF_SLOPE[band];
  adc1.calOffset[1] = ADC1_FWD_OFFSET[band];
  adc1.calSlope[1]  = ADC1_FWD_SLOPE;

  // ADC2 input bridge: ch0=FORWARD, ch1=REFLECTED — opposite to ADC1
  adc2.calOffset[0] = ADC2_FWD_OFFSET[band];   // ch0 = FORWARD
  adc2.calSlope[0]  = ADC2_FWD_SLOPE;
  adc2.calOffset[1] = ADC2_REF_OFFSET[band];   // ch1 = REFLECTED
  adc2.calSlope[1]  = ADC2_REF_SLOPE[band];
}

// ===== INIT =====
static void initADCDevice(ADCDevice &adc, uint8_t addr) {
  adc.addr             = addr;
  adc.state            = ADC_IDLE;
  adc.currentChannel   = 0;
  adc.scheduleIndex    = 0;
  adc.conversionStarted = 0;

  for (int i = 0; i < ADC_CHANNELS; i++) {
    adc.index[i]         = 0;
    adc.filteredLog[i]   = 0;
    adc.filteredPower[i] = 0;
    adc.peakPower[i]     = 0;
    adc.lastRaw[i]       = 0;
    adc.initialized[i]   = false;
    for (int j = 0; j < MAX_SAMPLES; j++) adc.buffers[i][j] = 0;
  }

  adc.calOffset[0] = 0; adc.calSlope[0] = 100.0f;
  adc.calOffset[1] = 0; adc.calSlope[1] = 100.0f;
}

void ADC_init() {
  initADCDevice(adc1, ADS1_ADDR);
  initADCDevice(adc2, ADS2_ADDR);
  ADC_setBand(0);   // load fallback calibration until band is known
}

// ===== LOW LEVEL I2C =====
static void writeRegister(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    recordADCFault();
  }
}

static uint16_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    recordADCFault();
    return 0;
  }
  Wire.requestFrom(addr, (uint8_t)2);
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    recordADCFault();
    return 0;
  }
  if (Wire.available() < 2) return 0;
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

static int16_t readConversion(uint8_t addr) {
  return (int16_t)readRegister(addr, ADS_REG_CONVERSION);
}

static bool isReady(uint8_t addr) {
  return readRegister(addr, ADS_REG_CONFIG) & 0x8000;
}

static uint16_t buildConfig(uint8_t ch) {
  uint16_t mux;
  switch (ch) {
    case 0:  mux = 0x4000; break;
    case 1:  mux = 0x5000; break;
    case 2:  mux = 0x6000; break;
    case 3:  mux = 0x7000; break;
    default: mux = 0x4000; break;
  }
  // Single-shot | ±4.096V FSR | 860 SPS
  return 0x8000 | mux | 0x0200 | 0x0100 | ADS_DR_860SPS;
}

// ===== SAMPLE PROCESSING =====

// Log-detector channels (ch0 reflected, ch1 forward).
// Formula: power_W = exp((mV - offset) / slope)
// Slopes are in mV/Np — must use expf(), NOT powf(10, x/10).
static void processSampleLog(ADCDevice &adc, uint8_t ch, int16_t raw) {
  float mV  = raw * (4096.0f / 32768.0f);
  float arg = (mV - adc.calOffset[ch]) / adc.calSlope[ch];

  if (!adc.initialized[ch]) {
    adc.filteredLog[ch]   = arg;
    adc.filteredPower[ch] = expf(arg);
    adc.peakPower[ch]     = adc.filteredPower[ch];
    adc.initialized[ch]   = true;
    return;
  }

  float diff = fabsf(arg - adc.filteredLog[ch]);

  // Spike > 0.34: snap immediately for fast transient tracking.
  // Otherwise: exponential moving average (τ ≈ 5 samples).
  if (diff > 0.34f) {
    adc.filteredLog[ch] = arg;
  } else {
    adc.filteredLog[ch] = adc.filteredLog[ch] * 0.8f + arg * 0.2f;
  }

  adc.filteredPower[ch] = expf(adc.filteredLog[ch]);
  adc.peakPower[ch]     = fmaxf(adc.peakPower[ch] * 0.95f, adc.filteredPower[ch]);
}

// Slow/voltage channels (ch2, ch3) — stored in circular buffer.
static void processSampleBuffer(ADCDevice &adc, uint8_t ch, int16_t raw) {
  uint8_t i          = adc.index[ch];
  adc.buffers[ch][i] = raw;
  adc.index[ch]      = (i + 1) % MAX_SAMPLES;
}

// ===== DEVICE UPDATE =====
static void updateADC(ADCDevice &adc) {
  switch (adc.state) {

    case ADC_IDLE: {
      uint8_t ch         = schedule[adc.scheduleIndex];
      adc.currentChannel = ch;
      writeRegister(adc.addr, ADS_REG_CONFIG, buildConfig(ch));
      adc.conversionStarted = millis();
      adc.state             = ADC_WAITING;
      break;
    }

    case ADC_WAITING: {
      unsigned long now = millis();

      // Don't poll before conversion can possibly be done
      if (now - adc.conversionStarted < ADC_CONVERSION_WAIT_MS) break;

      // Timeout — only checked here in WAITING, never in IDLE.
      // A device in IDLE between conversions is not unresponsive.
      // Goes through rate limiter: a single late poll (slow loop iteration)
      // won't latch the fault; only repeated misses inside the window do.
      if (now - adc.conversionStarted > ADC_RESPONSE_TIMEOUT_MS) {
        recordADCFault();
        adc.conversionStarted = now;   // reset so we don't re-fire every call
        adc.state             = ADC_IDLE;
        break;
      }

      if (!isReady(adc.addr)) break;

      uint8_t ch  = adc.currentChannel;
      int16_t val = readConversion(adc.addr);

      adc.lastRaw[ch] = val;

      if (ch <= 1) processSampleLog(adc, ch, val);
      else         processSampleBuffer(adc, ch, val);

      adc.scheduleIndex = (adc.scheduleIndex + 1) % scheduleLength;
      adc.state         = ADC_IDLE;
      break;
    }
  }
}

// ===== PUBLIC UPDATE =====
void ADC_update() {
  if (useADC1) updateADC(adc1);
  else         updateADC(adc2);
  useADC1 = !useADC1;
}

// ===== ADC1 PUBLIC API — OUTPUT BRIDGE =====

float   ADC1_getPower(uint8_t ch)  { return adc1.filteredPower[ch]; }
float   ADC1_getPeak(uint8_t ch)   { return adc1.peakPower[ch]; }
int16_t ADC1_getRaw(uint8_t ch)    { return adc1.lastRaw[ch]; }

float ADC1_getAvg(uint8_t ch, uint8_t samples) {
  if (ch < 2) return ADC1_getPower(ch);
  if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    int idx = (int)adc1.index[ch] - 1 - i;
    if (idx < 0) idx += MAX_SAMPLES;
    sum += adc1.buffers[ch][idx];
  }
  return (sum / (float)samples) * (4.096f / 32768.0f);
}

// ===== ADC2 PUBLIC API — INPUT BRIDGE =====

float   ADC2_getPower(uint8_t ch)  { return adc2.filteredPower[ch]; }
float   ADC2_getPeak(uint8_t ch)   { return adc2.peakPower[ch]; }
int16_t ADC2_getRaw(uint8_t ch)    { return adc2.lastRaw[ch]; }

float ADC2_getAvg(uint8_t ch, uint8_t samples) {
  if (ch < 2) return ADC2_getPower(ch);
  if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    int idx = (int)adc2.index[ch] - 1 - i;
    if (idx < 0) idx += MAX_SAMPLES;
    sum += adc2.buffers[ch][idx];
  }
  return (sum / (float)samples) * (4.096f / 32768.0f);
}

// ===== CONTROL =====

void ADC_resetPeaks() {
  for (int i = 0; i < ADC_CHANNELS; i++) {
    adc1.peakPower[i] = 0;
    adc2.peakPower[i] = 0;
  }
}

// ===== STATUS =====

bool ADC_fault()      { return adcFaultFlag; }

void ADC_clearFault() {
  adcFaultFlag = false;
  for (uint8_t i = 0; i < ADC_FAULT_WINDOW_COUNT; i++) faultTimes[i] = 0;
  faultIdx = 0;
}
