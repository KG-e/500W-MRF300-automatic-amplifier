// ================= AMP MAIN =================
#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "UART.h"
#include "ADC.h"
#include <avr/pgmspace.h>

// ================= FORWARD DECLARATIONS =================
// Predeclare all functions in this file so definition order doesn't matter.

typedef enum {
  INPUT_PWR_LOW  = 0,   // ~1.5W — AMP_LOW_TRANSMIT
  INPUT_PWR_MED  = 1,   // ~3.0W — high-gain bands
  INPUT_PWR_HIGH = 2    // ~5.0W — lower-gain bands
} InputPowerTarget_t;

InputPowerTarget_t getBandPowerTarget(uint8_t band);

// Safe state / hardware reset
void setSafeState();
void tunerDefaultOffState();
void attenuatorReset();
void LPFreset();
void AMP_pin_init();

// RF detection
void rfDetectorUpdate();
bool rfPresent();
bool rfOnDelayReached();
bool rfSignalDelayReached();
bool rfSignalValid();
bool rfSignalStable();
bool rfSignalNotValidTimeout();
bool rfSignalOutOfRangeCheck();

// Frequency / band
void freqDetector();
int  getFrequencyBand();
bool bandStable(uint8_t band);
bool setBand(uint8_t band);

// Protection
bool checkProtectionFast();
bool checkProtectionSlow();
void resetProtectionSlow();
bool checkSRLatch();
bool checkOutputSWR(uint8_t &sustainedCount);
bool checkInputSWR(uint8_t &sustainedCount);
void checkAllFaults();

// Performance
float getAmpCurrent();
float computeGain();
float computeDCInputPower();
float computeEfficiency();
bool checkAmpPerformance(bool &hardFault, InputPowerTarget_t target = INPUT_PWR_LOW);

// SWR
float SWR(float Pf, float Pr);
float SWR_input();
float SWR_output();

// Attenuator / tuning
float   maskToDb(uint8_t mask);
uint8_t getAttMaskFromTable(float inputPower, InputPowerTarget_t target);
bool    powerOutOfRange(float measured, float expected);
float   expectedOutputPower(float Pin, float att_db);
void    applyAttenuatorMask(uint8_t mask);
bool    getAveragedInputPowerNB(float &result);
bool    inputPowerStable(float p);
void    resetInputPowerStable();
bool    inputPowerTooHigh(float p, InputPowerTarget_t target);
bool    inputPowerTooHighWait(float p, InputPowerTarget_t target);
bool    runInputTuning(InputPowerTarget_t target);

// State machine
void updateAmpState();

// UART / telemetry
void sendTelemetry();
void myUARTHandler(uint8_t type, uint16_t value);
void IVcheck();
void emergencyInputPowerShutdown();

// ================= END FORWARD DECLARATIONS =================

// ================= AMP STATE MACHINE =================

typedef enum {
  AMP_OFF = 0,
  AMP_INIT,
  AMP_IDLE,
  AMP_FIRST_STARTUP,
  AMP_FIRST_ADJUST_INPUT,
  AMP_LOW_TRANSMIT,
  AMP_TUNE,
  AMP_TRANSMIT_TRANSITION,
  AMP_TRANSMIT_HIGH,
  AMP_READY,
  AMP_SSB_VERIFY,
  AMP_SOFT_FAULT,
  AMP_FAULT
} AmpState_t;

/*
  AMP_OFF               — wait state (startup grace cycles)
  AMP_INIT              — first init, hardware reset to safe state
  AMP_IDLE              — amp idle, waiting for RF
  AMP_FIRST_STARTUP     — RF detected, begin transmit procedure
  AMP_FIRST_ADJUST_INPUT— adjusting and confirming input power via attenuator
  AMP_LOW_TRANSMIT      — transmitting at low output power
  AMP_TUNE              — tuning output impedance matching network (placeholder)
  AMP_TRANSMIT_TRANSITION — changing input power for high output
  AMP_TRANSMIT_HIGH     — transmitting at high output power
  AMP_READY             — configuration set, waiting for next TX
  AMP_SOFT_FAULT        — latched fault, cleared only by UI MCU reset command
  AMP_FAULT             — hard fault, requires power cycle
*/

// Named fault reasons — sent to UI MCU via UART on every fault
typedef enum {
  FAULT_NONE                = 0,
  FAULT_AMPLIFIER           = 1,   // hardware hard fault (generic)
  FAULT_RF_SIGNAL_INVALID   = 2,   // signal must be valid for signalNotValidCountdown
  FAULT_BAND_INVALID        = 3,   // freq. is out of specified band
  FAULT_BAND_UNSTABLE       = 4,   // freq. band unstable during firststartup
  FAULT_BAND_SET_FAILED     = 5,   // band request out of range 0-5
  FAULT_BAND_CHANGE_LIVE    = 6,   // band change attempted during 50V
  FAULT_UART_COMM           = 7,   // UART comm/heartbeat/sync latched
  FAULT_ADC_HARDWARE        = 8,   // ADS1115 / I²C bus fault (rate-limited)
  FAULT_CURRENT_SENSOR      = 9,   // Iamp reads near-zero with meaningful Pout — sensor failed
  FAULT_BIAS_STARTUP        = 10,  // bias current didn't rise within timeout
  FAULT_TUNE_UNSTABLE       = 20,  // freq. unstable during tune (ssb)
  FAULT_INPUT_TOO_HIGH_HW   = 21,  // input too high (over 120W)
  FAULT_ATT_VERIFY          = 22,  // power was too high after att. switch
  FAULT_ATT_NO_REDUCTION    = 23,  // attenuators did not reduce power
  FAULT_RF_LOST_DURING_TUNE = 24,  // !rfPresent() during tune
  FAULT_INPUT_TOO_HIGH_SOFT = 30,  // checks if input power is too high during verify
  FAULT_INPUT_TOO_HIGH_WAIT = 33,  // checks if input power is too high after wait for att. switch
  FAULT_SWR_HIGH            = 40,  // sustained output SWR >3 — soft fault
  FAULT_GAIN_LOW            = 41,  // gain <13dB — soft fault
  FAULT_EFFICIENCY_LOW      = 42,  // efficiency <40% — hard fault
  FAULT_LEAKAGE_CURRENT     = 43,  // >650mA idle with 50V on — hard fault
  FAULT_OVERCURRENT_HW      = 44,  // SR latch tripped by comparator — soft fault
  FAULT_HIGH_REFLECTED      = 45,  // instantaneous high reflected from checkProtectionFast
  FAULT_TEMP_HIGH           = 50,  // heatsink ≥70°C / inner ≥55°C / air ≥50°C — soft fault
  FAULT_TEMP_CRITICAL       = 51,  // heatsink ≥80°C — hard fault (MOSFET case temp)
  FAULT_LOW_VOLTAGE         = 60,  // 18V or 50V rail out of range
  FAULT_HIGH_CURRENT_AVR    = 61,  // overcurrent caught by AVR ADC (SR latch backup)
  FAULT_GATE_DRIFT          = 62,  // checkProtectionSlow tripped — thermal runaway suspect
  FAULT_PA_SUPPLY_LOW       = 63,  // 50V requested, not achieved
  FAULT_PA_SUPPLY_STUCK     = 64,  // 50V not requested, still present
  FAULT_INPUT_SWR_HIGH      = 65,  // input SWR too high
  FAULT_STATE_CORRUPTED     = 98,  // switch hit default branch
  FAULT_EMERGENCY_SHUTDOWN  = 99
} FaultReason_t;

volatile AmpState_t AmpState     = AMP_OFF;
static AmpState_t   PrevAmpState = AMP_OFF;

// Proper entry-action flag — set true once entry actions fire, reset whenever
// the state machine detects we've changed states (see updateAmpState()).
static bool stateEntryDone = false;

// ================= TUNE SUB-STATE =================
static InputPowerTarget_t currentInputTarget = INPUT_PWR_LOW;

// Tune this table against your actual output measurements
InputPowerTarget_t getBandPowerTarget(uint8_t band) {
  switch (band) {
    case 2: return INPUT_PWR_MED;   // 7MHz  — ~650W at 5W, reduce to 3W
    case 3: return INPUT_PWR_MED;   // 14MHz — ~650W at 5W, reduce to 3W
    case 4: return INPUT_PWR_MED;   // 21MHz — ~650W at 5W, reduce to 3W
    default: return INPUT_PWR_HIGH; // 3.5MHz, 28MHz — 5W OK
  }
}

typedef enum {
  TUNE_IDLE,
  TUNE_MEASURE_INPUT,
  TUNE_SET_ATT,
  TUNE_WAIT,
  TUNE_VERIFY
} TuneState_t;

static TuneState_t   tuneState  = TUNE_IDLE;
static unsigned long tuneTimer  = 0;

// TUNE_VERIFY monitoring window
#define TUNE_VERIFY_MS  200UL   // must hold stable for this long before success

static float   Pin_avg        = 0;
static uint8_t currentMask    = 0;

// ================= AMP GLOBALS =================

float U18V = 0;
float U50V = 0;
float I18V = 0;
float I50V = 0;
unsigned long lastIVcheck = 0;

bool lowVoltageFault     = false;
bool paSupplyStuckFault  = false;
bool paSupplyFault       = false; 
bool highCurrentFault    = false;
bool amplifierFault      = false;
bool uartLatchedFault    = false;
bool powerDetectionFault = false;
bool adcFaultLatch       = false;
bool softFaultResetRequest = false;
bool faultReportSent       = false;
bool tempFault             = false;  // latched when any temperature exceeds soft limit

// Temperature values received from secondary MCU via UART
// Updated in myUARTHandler, checked in checkAllFaults() every loop
float tempHeatsink = 0.0f;
float tempInner    = 0.0f;
float tempAir      = 0.0f;

// Stored from last successful transmit — used for READY fast-path check
//static float   lastPeakPower      = -1.0f; unused

static uint8_t lastTxBand         = 0;
static bool    transitionFromReady = false;  // true when TRANSMIT_TRANSITION entered via READY fast-path

uint8_t startupcycle = 0;
uint8_t startupgrace = 5;

FaultReason_t FaultReason = FAULT_NONE;

bool rfSignalInvalid    = false;
uint8_t lastTransmittedBand = 5;

#define DEBUG_SERIAL
// ================= TIMING CONFIG =================

#define IVCheckInterval 30

// ================= PROTECTION THRESHOLDS =================

#define low18Vlimit   15.0f
#define low50Vlimit   40.0f
#define high18Ilimit   4.0f
#define high50Ilimit  24.0f

// Gate voltage validity window — tune against your bias circuit //currently not active, any floating voltage accepted to avoid error
#define GATE_MIN_V           0.0f
#define GATE_MAX_V           3.0f
#define GATE_MATCH_TOLERANCE 3.0f

// Temperature thresholds — heatsink is LM35DT bolted to MRF300 TO-247 case
#define TEMP_HEATSINK_SOFT   70.0f   // °C — fans at 100%, soft fault
#define TEMP_HEATSINK_HARD   80.0f   // °C — hard fault, MOSFET destruction risk
#define TEMP_HEATSINK_RESET  60.0f   // °C — must be below this to allow soft fault reset
#define TEMP_INNER_SOFT      55.0f   // °C — enclosure inner board soft fault
#define TEMP_AIR_SOFT        50.0f   // °C — ambient air soft fault

#define TYPE_RESET           0x72    // Protocol type (bidirectional)

#define SSB_VERIFY_MS   500UL    // window to catch representative SSB peaks
#define TYPE_SSB_MODE   0x74     // HMI SSB button — value carries no info

#define SSB_MIN_PEAK_W  2.5f   // minimum peak seen to confirm real SSB signal on bridge

bool ssbModeRequested = false;
static bool ssbTransmitActive = false;

// Amplifier performance thresholds
#define AMP_MIN_GAIN_DB      10.0f   // below this — soft fault
#define AMP_MIN_EFFICIENCY_LOW_TX   0.20f   // below this — hard fault (20%)
#define AMP_MIN_EFFICIENCY_MED_TX   0.30f   // 30% efficiency floor for 3W input
#define AMP_MIN_EFFICIENCY_HIGH_TX  0.30f   // below this - hard fault (40%)

#define AMP_LEAKAGE_LIMIT_A  0.65f   // 650mA idle leakage limit — hard fault
#define AMP_LEAKAGE_LIMIT_LOW 2.0f   // 2A start idle leakage limit — hard fault
#define AMP_IDLE_CURRENT_MAX 0.65f   // same, alias for clarity in READY

// SWR limits
#define SWR_SUSTAINED_LIMIT  3.0f    // sustained above this — soft fault
#define SWR_TRANSIENT_LIMIT  5.0f    // single spike above this — soft fault

#define FAST_PROT_COUNT  20     // ~10ms sustained — rides through ADC cycle transients
#define FAST_PROT_COUNT_SSB 70
#define SWR_INPUT_TRANSIENT_LIMIT  5.0f   // immediate — catastrophic mismatch
#define SWR_INPUT_SUSTAINED_LIMIT  3.0f   // soft fault after N consecutive readings

// Transmit state timing
#define LOW_TX_CONFIRM_MS    500UL   // time at low power before going to TRANSITION
#define HIGH_TX_CONFIRM_MS   1000UL   // time at high power before arming READY transition
#define HIGH_TX_RF_OFF_MS   1000UL   // SSB hold time before going to READY
#define READY_IDLE_TIMEOUT  120000UL // 2 minutes no RF in READY before going to IDLE

// READY fast-path: max allowed peak power change between overs (20%)
#define READY_POWER_TOLERANCE 0.20f

// Current sense scaling: 0.132V/A on ADS1115 ADC1 CH2
#define ISENSE_V_PER_A  0.138f

// Frequency detector hardware prescaler — RF input goes through a /2^14
// divider chain before reaching pin 2, so the measured pin frequency must
// be multiplied back up to get the true RF frequency.
#define RF_DIVIDER  16384.0f

#define BIAS_STARTUP_TIMEOUT_MS    1000UL   // max time to wait for quiescent current
#define MIN_LOW_TX_OUTPUT_W         40.0f  // minimum expected output in LOW_TRANSMIT
#define LOW_TX_POWER_TIMEOUT_MS    3000UL   // max time to reach minimum output

// ================= PIN DEFINITIONS =================

// Attenuator board OUTPUT (D30–D37 = PORTC on ATmega2560)
#define att1  30
#define att2  31
#define att3  32
#define att4  33
#define att5  34
#define att6  35
#define att7  36
#define att8  37
#define rfout A7

// Amplifier board OUTPUT
#define biasenable A6

// LPF board OUTPUT
#define LPF1 A8
#define LPF2 A9
#define LPF3 A10
#define LPF4 A11
#define LPF5 A12

// Tuner board OUTPUT
#define TBL1   A13   // CTR1 on schematic
#define TBL2   A14
#define TBL3   A15
#define TBL4   29    // inductor bypass
#define TBL5   28
#define TBL6   27
#define TBL7   26    // CTR7 on schematic

#define TBC1   25    // CTR8 on schematic
#define TBC2   24
#define TBC3   23
#define TBC4   22    // capacitor adding
#define TBC5   10
#define TBC6   11
#define TBC7   12    // CTR14 on schematic

#define TBLCsw 13    // CTR15 — capacitance location

// Misc
#define highvsupplyon A5  // OUTPUT — 50V supply on
#define rfswitch      38  // OUTPUT — switch relays RX→TX
#define buzzer        49  // OUTPUT — alarm buzzer
#define QofSR         16  // INPUT  — SR latch output (50V overcurrent hardware trip)
#define RofSR         17  // OUTPUT — SR latch reset (only after fault acknowledged)
#define rfdetection   A4  // INPUT  — diode RF detector
#define rffrequency    2  // INPUT  — frequency detector (INT4, 0–2kHz pin freq * 16384 = actual RF Hz)

#define IsenseEighteenV A3  // INPUT — 18V supply current (MAX40010)
#define IsenseFiftyV    A2  // INPUT — 50V supply current (MAX40010)
#define VsenseEighteenV A1  // INPUT — 18V supply voltage
#define VsenseFiftyV    A0  // INPUT — 50V supply voltage

// ================= ATT LOOKUP TABLES =================

const uint8_t attTable_5W[121] PROGMEM = {
  0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,
  0b11011111,0b01011111,0b11101111,0b01101111,0b11111101,0b01111101,
  0b11011101,0b11011101,0b01011101,0b11110101,0b11110101,0b01110101,
  0b01110101,0b11111110,0b11111110,0b11111110,0b01111110,0b01111110,
  0b11011110,0b11011110,0b11011110,0b01011110,0b01011110,0b01011110,
  0b11101110,0b11101110,0b11101110,0b11101110,0b01101110,0b01101110,
  0b01101110,0b01101110,0b11111100,0b11111100,0b11111100,0b11111100,
  0b11111100,0b01111100,0b01111100,0b01111100,0b01111100,0b01111100,
  0b11011100,0b11011100,0b11011100,0b11011100,0b11011100,0b01011100,
  0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
  0b11110100,0b11110100,0b11110100,0b11110100,0b11110100,0b11110100,
  0b11110100,0b01110100,0b01110100,0b01110100,0b01110100,0b01110100,
  0b01110100,0b01110100,0b01110100,0b11111000,0b11111000,0b11111000,
  0b11111000,0b11111000,0b11111000,0b11111000,0b11111000,0b11111000,
  0b01111000,0b01111000,0b01111000,0b01111000,0b01111000,0b01111000,
  0b01111000,0b01111000,0b01111000,0b01111000,0b01111000,0b11011000,
  0b11011000,0b11011000,0b11011000,0b11011000,0b11011000,0b11011000,
  0b11011000,0b11011000,0b11011000,0b11011000,0b01011000,0b01011000,
  0b01011000,0b01011000,0b01011000,0b01011000,0b01011000,0b01011000,
  0b01011000,0b01011000,0b01011000,0b01011000,0b01011000,0b11110000,
  0b11110000
};

const uint8_t attTable_1W5[121] PROGMEM = {
  0b11111111,0b11111111,0b01011111,0b11111101,0b01011101,0b11110011,
  0b11111110,0b01111110,0b01011110,0b11110110,0b11110110,0b01110110,
  0b11111100,0b01111100,0b01111100,0b11011100,0b01011100,0b01011100,
  0b11110100,0b11110100,0b01110100,0b01110100,0b01110100,0b11111000,
  0b11111000,0b11111000,0b01111000,0b01111000,0b01111000,0b11011000,
  0b11011000,0b11011000,0b01011000,0b01011000,0b01011000,0b01011000,
  0b11110000,0b11110000,0b11110000,0b11110000,0b01110000,0b01110000,
  0b01110000,0b01110000,0b01110000,0b11010000,0b11010000,0b11010000,
  0b11010000,0b11010000,0b11010000,0b01010000,0b01010000,0b01010000,
  0b01010000,0b01010000,0b01010000,0b11100000,0b11100000,0b11100000,
  0b11100000,0b11100000,0b11100000,0b11100000,0b01100000,0b01100000,
  0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b11000000,
  0b11000000,0b11000000,0b11000000,0b11000000,0b11000000,0b11000000,
  0b11000000,0b11000000,0b01000000,0b01000000,0b01000000,0b01000000,
  0b01000000,0b01000000,0b01000000,0b01000000,0b01000000,0b01000000,
  0b10000000,0b10000000,0b10000000,0b10000000,0b10000000,0b10000000,
  0b10000000,0b10000000,0b10000000,0b10000000,0b10000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
  0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
  0b00000000
};

const uint8_t attTable_3W[121] PROGMEM = {
  0b11111111,0b11111111,0b11111111,0b11111111,0b11011111,0b11110111,
  0b11111101,0b01111101,0b01011101,0b11110101,0b11110101,0b01110101,
  0b11111110,0b01111110,0b01111110,0b11011110,0b01011110,0b01011110,
  0b11110110,0b11110110,0b11110110,0b01110110,0b01110110,0b11111100,
  0b11111100,0b11111100,0b01111100,0b01111100,0b01111100,0b11011100,
  0b11011100,0b11011100,0b01011100,0b01011100,0b01011100,0b01011100,
  0b11110100,0b11110100,0b11110100,0b11110100,0b11110100,0b01110100,
  0b01110100,0b01110100,0b01110100,0b11111000,0b11111000,0b11111000,
  0b11111000,0b11111000,0b11111000,0b01111000,0b01111000,0b01111000,
  0b01111000,0b01111000,0b01111000,0b11011000,0b11011000,0b11011000,
  0b11011000,0b11011000,0b11011000,0b11011000,0b01011000,0b01011000,
  0b01011000,0b01011000,0b01011000,0b01011000,0b01011000,0b01011000,
  0b11110000,0b11110000,0b11110000,0b11110000,0b11110000,0b11110000,
  0b11110000,0b11110000,0b01110000,0b01110000,0b01110000,0b01110000,
  0b01110000,0b01110000,0b01110000,0b01110000,0b01110000,0b01110000,
  0b11010000,0b11010000,0b11010000,0b11010000,0b11010000,0b11010000,
  0b11010000,0b11010000,0b11010000,0b11010000,0b11010000,0b01010000,
  0b01010000,0b01010000,0b01010000,0b01010000,0b01010000,0b01010000,
  0b01010000,0b01010000,0b01010000,0b01010000,0b01010000,0b11100000,
  0b11100000,0b11100000,0b11100000,0b11100000,0b11100000,0b11100000,
  0b11100000
};

// ================= ADS1115 CONFIG =================

#define ADS1_ADDR         0x48
#define ADS2_ADDR         0x49
#define ADS_REG_CONVERSION 0x00
#define ADS_REG_CONFIG     0x01
#define MAX_SAMPLES        32
#define ADS_DR_860SPS      0x00E0

// AD8310 calibration constants are now per-band in ADC.cpp.
// Call ADC_setBand(band) whenever lastTransmittedBand changes.

// SWR protection thresholds
#define SWR_TRIP      4.0f
#define SWR_MIN_POWER 0.05f

// ================= SWR =================

float SWR(float Pf, float Pr) {
  if (Pf < SWR_MIN_POWER) return 1.0f;
  float ratio = Pr / Pf;
  if (ratio >= 0.999f) return 99.9f;
  float gamma = sqrtf(ratio);
  return (1.0f + gamma) / (1.0f - gamma);
}

// ADC1=output, ADC2=input
float SWR_input() {
  return SWR(ADC2_getPower(ADC2_FORWARD), ADC2_getPower(ADC2_REFLECTED));
}

float SWR_output() {
  return SWR(ADC1_getPower(ADC1_FORWARD), ADC1_getPower(ADC1_REFLECTED));
}

// ================= PROTECTION =================

bool checkProtectionFast() {
  static uint8_t fastCount = 0;

  // Suppress during amp current ramp-up — ADC channels not yet in sync
  if (getAmpCurrent() < 2.0f) {
    fastCount = 0;
    return false;
  }

  float swr = SWR_output();
  float Pf  = ADC1_getPeak(ADC1_FORWARD);
  float Pr  = ADC1_getPeak(ADC1_REFLECTED);

  bool trip = (swr > SWR_TRIP) || (Pf > SWR_MIN_POWER && Pr > Pf * 0.5f);

  if (trip && !ssbTransmitActive) {
    if (++fastCount >= FAST_PROT_COUNT) {
      fastCount = 0;
      Serial.print("swr:"); Serial.println(swr);
      return true;
    }
  } else if (trip && ssbTransmitActive) {
    if (++fastCount >= FAST_PROT_COUNT_SSB) {
      fastCount = 0;
      Serial.print("swr:"); Serial.println(swr);
      return true;
    }
  } else {
    fastCount = 0;
  }


  return false;
}

// Reset flag — set when leaving transmit states so stale gate voltage
// references from a previous session don't carry over into the next
static bool protectionSlowResetPending = false;

void resetProtectionSlow() {
  protectionSlowResetPending = true;
}

// Reads gate voltages (ADC2 ch2/ch3). Monitors for drift indicating
// thermal runaway on either MRF300.
bool checkProtectionSlow() {
  // Only meaningful when 50V supply is active and bias current is flowing
  if (!digitalRead(highvsupplyon)) return false;

  float V1 = ADC2_getAvg(ADC2_GATE1, 10);
  float V2 = ADC2_getAvg(ADC2_GATE2, 10);

  static bool     initialized = false;
  static float    V1_ref, V2_ref;

  // Reset reference whenever we re-enter a transmit state
  if (protectionSlowResetPending) {
    initialized              = false;
    protectionSlowResetPending = false;
  }

  if (!initialized) {
    V1_ref      = V1;
    V2_ref      = V2;
    initialized = true;
    return false;
  }

  // Trip if either gate drifts more than 40% from its reference
  if (fabsf(V1 - V1_ref) > V1_ref * 0.4f) return true;
  if (fabsf(V2 - V2_ref) > V2_ref * 0.4f) return true;

  // Slow EWMA tracks gradual temperature drift without alarming
  V1_ref = V1_ref * 0.99f + V1 * 0.01f;
  V2_ref = V2_ref * 0.99f + V2 * 0.01f;

  return false;
}

// ================= UART CONFIG =================

// TYPE_FAULT_REASON: sent to UI MCU whenever a fault is latched.
// Value carries the FaultReason_t code (fits in 8 bits, sent as uint16_t).
#define TYPE_FAULT_REASON  0x10

// Telemetry packet types — MCU1 → MCU2, cycled every 50ms.
// Encoding noted per type.
#define TYPE_AMP_STATE     0x20   // AmpState_t enum value, sent on every cycle
#define TYPE_OUT_POWER     0x21   // output forward power W×10 (0.1W res, max 6553W)
#define TYPE_OUT_SWR       0x22   // output SWR ×100 (0.01 res, max 655)
#define TYPE_IN_POWER      0x23   // input forward power W×10
#define TYPE_IN_SWR        0x24   // input SWR ×100
#define TYPE_AMP_CURRENT   0x25   // amp supply current in mA (0–20000 fits uint16)
#define TYPE_GAIN          0x26   // gain dB×10 offset +500 (0=-50dB, 500=0dB, 1000=+50dB)
#define TYPE_EFFICIENCY    0x27   // efficiency as integer percent 0–100
#define TYPE_BAND          0x28   // currently set band 1–5, 0=none

// ================= TELEMETRY =================

// Telemetry cycles through 5 slots on a 50ms rotation.
// Slot 0: AmpState + output power + output SWR
// Slot 1: input power + input SWR
// Slot 2: amp current + efficiency
// Slot 3: gain + band
// Slot 4: AmpState again (keeps UI display fresh at 200ms worst-case lag)

void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  static uint8_t       slot          = 0;

  unsigned long now = millis();
  if (now - lastTelemetry < 50) return;
  lastTelemetry = now;

  switch (slot) {

    case 0:
      UART_send(TYPE_AMP_STATE,  (uint16_t)AmpState);
      UART_send(TYPE_OUT_POWER,  (uint16_t)(ADC1_getPower(ADC1_FORWARD) * 10.0f));
      UART_send(TYPE_OUT_SWR,    (uint16_t)(SWR_output() * 100.0f));
      break;

    case 1:
      UART_send(TYPE_IN_POWER,   (uint16_t)(ADC2_getPower(ADC2_FORWARD) * 10.0f));
      UART_send(TYPE_IN_SWR,     (uint16_t)(SWR_input() * 100.0f));
      break;

    case 2: {
      float Iamp   = getAmpCurrent();
      float IampMa = Iamp * 1000.0f;
      if (IampMa < 0.0f)     IampMa = 0.0f;
      if (IampMa > 20000.0f) IampMa = 20000.0f;

      float eff = computeEfficiency() * 100.0f;
      if (eff  < 0.0f)   eff   = 0.0f;
      if (eff  > 100.0f) eff   = 100.0f;

      UART_send(TYPE_AMP_CURRENT, (uint16_t)IampMa);
      UART_send(TYPE_EFFICIENCY,  (uint16_t)eff);
      break;
    }

    case 3: {
      float gain    = computeGain();
      int16_t gainEncoded = (int16_t)(gain * 10.0f) + 500;
      if (gainEncoded < 0)    gainEncoded = 0;
      if (gainEncoded > 9999) gainEncoded = 9999;
      UART_send(TYPE_GAIN, (uint16_t)gainEncoded);
      UART_send(TYPE_BAND, (uint16_t)lastTransmittedBand);
      break;
    }

    case 4:
      UART_send(TYPE_AMP_STATE, (uint16_t)AmpState);
      UART_send(TYPE_FAULT_REASON, (uint16_t)FaultReason);
      UART_send(TYPE_SSB_MODE,     (uint16_t)ssbModeRequested);
      break;
  }

  slot = (slot + 1) % 5;
}

// Secondary MCU inbound packet types (0x80–0x83)
#define TYPE_TEMP_HEATSINK   0x80   // heatsink case temp, integer °C
#define TYPE_TEMP_INNER      0x81   // inner board temp, integer °C
#define TYPE_TEMP_AIR        0x82   // ambient air temp, integer °C
#define TYPE_VOLTAGE_FAULT   0x83   // 1 = at least one rail out of range

Range myRanges[] = {
  {TYPE_SSB_MODE,      0,    0},
  {TYPE_RESET,         0,    0},   // operator reset request from HMI — value carries no info
  {TYPE_TEMP_HEATSINK, 0,  120},   // °C, 0–120 reasonable for LM35DT range
  {TYPE_TEMP_INNER,    0,  120},
  {TYPE_TEMP_AIR,      0,  120},
  {TYPE_VOLTAGE_FAULT, 0,    1}
};

void myUARTHandler(uint8_t type, uint16_t value) {
  if (UART_latchedFault()) return;

  switch (type) {

    case 0x03:
      // Reserved for future use. Currently not sent by the secondary MCU.
      // If wired up later, ensure 0x03 is also added to myRanges[].
      break;

    case TYPE_RESET:
      // Temperature soft fault: only allow reset if heatsink has cooled
      if (FaultReason == FAULT_TEMP_HIGH || FaultReason == FAULT_TEMP_CRITICAL) {
        if (tempHeatsink < TEMP_HEATSINK_RESET) {
          softFaultResetRequest = true;
        }
        // If too hot, ignore reset — secondary MCU will show "TOO HOT" on display
      } else {
        softFaultResetRequest = true;
      }
      break;
    case TYPE_TEMP_HEATSINK:
      tempHeatsink = (float)value;
      break;
    case TYPE_TEMP_INNER:
      tempInner = (float)value;
      break;
    case TYPE_TEMP_AIR:
      tempAir = (float)value;
      break;
    case TYPE_VOLTAGE_FAULT:
      if (value == 1) {
        setSafeState();
        amplifierFault = true;
        FaultReason    = FAULT_LOW_VOLTAGE;
      }
      break;
    case TYPE_SSB_MODE:
      ssbModeRequested = !ssbModeRequested;
    break;
  }
}

// Send fault reason to UI MCU once per fault event.
// Called from AMP_SOFT_FAULT and AMP_FAULT entry actions.
static void reportFaultToUI(FaultReason_t reason) {
  if (!faultReportSent) {
    UART_send(TYPE_FAULT_REASON, (uint16_t)reason);
    faultReportSent = true;
  }
}

// ================= VOLTAGE AND CURRENT CHECK =================

// Timing constants for 50V supply relay
#define PA_SUPPLY_SETTLE_MS    30UL    // relay close time before voltage is valid
#define PA_SUPPLY_DRAIN_MS   1000UL    // max time for 50V to drain after relay opens

void IVcheck() {
  unsigned long now = millis();
  if (now - lastIVcheck < IVCheckInterval) return;
  lastIVcheck = now;

  // Track transitions on highvsupplyon to gate voltage checks correctly
  static bool          lastSupplyState   = false;
  static unsigned long supplyStateChange = 0;
  static bool          ivInitialized     = false;

  if (!ivInitialized) {
    supplyStateChange = now;   // anchor the timer to first real call, not millis()=0
    ivInitialized     = true;
  }

  bool supplyOn = digitalRead(highvsupplyon);
  if (supplyOn != lastSupplyState) {
    lastSupplyState   = supplyOn;
    supplyStateChange = now;
  }

  U18V = analogRead(VsenseEighteenV) * 0.0294f;
  U50V = analogRead(VsenseFiftyV)    * 0.079f;
  I18V = analogRead(IsenseEighteenV) * 0.008f;
  I50V = getAmpCurrent();

  // 18V driver supply — always on, always check immediately
  if (U18V < low18Vlimit) lowVoltageFault = true;

  if (supplyOn) {
    // Supply commanded ON — wait for relay contacts to settle before trusting voltage.
    // Checking immediately would always false-fault since the cap takes time to charge.
    if ((now - supplyStateChange) >= PA_SUPPLY_SETTLE_MS) {
      if (U50V < low50Vlimit){
        paSupplyFault = true;
        Serial.print(U50V);
      } 
    }

    if (I50V > high50Ilimit) highCurrentFault = true;

  } else {
    // Supply commanded OFF — if 50V is still above threshold after drain timeout,
    // the relay is stuck closed or the bleed circuit has failed.
    if ((now - supplyStateChange) >= PA_SUPPLY_DRAIN_MS) {
      if (U50V > low50Vlimit) paSupplyStuckFault = true;
    }
  }

  if (I18V > high18Ilimit) highCurrentFault = true;
}

// ================= FREQUENCY DETECTOR =================

#define TIMEOUT_US          20000
#define FILTER_ALPHA        0.1f
#define STABLE_THRESHOLD    0.02f
#define LOCK_TIME_MS        30
#define RF_SIGNAL_DELAY_MS  100
#define signalNotValidCountdown 1000

volatile uint16_t periodTicks  = 0;
volatile uint16_t lastCapture  = 0;
volatile uint32_t lastEdgeTime = 0;

static float filteredPeriod = 0;
static float lastPeriod     = 0;
static bool  initializedFD  = false;

static bool signalPresent   = false;
static bool signalStable    = false;
bool        signalLocked    = false;   // promoted to file scope — read by debug output

static unsigned long stableStartTime = 0;

float rfFrequency = 0;
float rfError     = 0;
float rfJitter    = 0;

void freqDetector() {
  noInterrupts();
  uint16_t p        = periodTicks;
  uint32_t lastEdge = lastEdgeTime;
  interrupts();

  if ((micros() - lastEdge) > TIMEOUT_US || p == 0) {
    signalPresent   = false;
    signalStable    = false;
    signalLocked    = false;
    initializedFD   = false;
    return;
  }

  signalPresent = true;

  float period_us = p * 0.5f;
  // Pin frequency is RF / 2^14 (hardware divider). Multiply back up to get
  // actual RF frequency, which is what getFrequencyBand() compares against.
  rfFrequency    = (1000000.0f / period_us) * RF_DIVIDER;

  if (!initializedFD) {
    filteredPeriod  = period_us;
    lastPeriod      = period_us;
    initializedFD   = true;
    stableStartTime = millis();
  }

  filteredPeriod = (1.0f - FILTER_ALPHA) * filteredPeriod + FILTER_ALPHA * period_us;

  if (!isfinite(filteredPeriod) || filteredPeriod <= 0.0f) {
    initializedFD = false;
    signalStable  = false;
    signalLocked  = false;
    return;
  }

  rfError  = fabsf(period_us - filteredPeriod) / filteredPeriod;
  rfJitter = fabsf(period_us - lastPeriod);
  lastPeriod = period_us;

  signalStable = (rfError < STABLE_THRESHOLD);

  if (signalStable) {
    if ((millis() - stableStartTime) >= LOCK_TIME_MS) {
      signalLocked = true;
    }
  } else {
    stableStartTime = millis();
    signalLocked    = false;
  }
}

bool rfSignalValid() {
  return signalPresent && signalLocked;
}

// rfSignalStable: signal is present and currently measuring stably,
// but does not require the full lock timer to have elapsed.
// Used for band checks during transmit where SSB causes frequent
// lock/unlock transitions — stable is sufficient to trust a band reading,
// whereas requiring full lock would cause constant false misses mid-speech.
bool rfSignalStable() {
  return signalPresent && signalStable;
}

static unsigned long signalNotValidStart = 0;
static bool          SignalStartState    = false;

bool rfSignalNotValidTimeout() {
  unsigned long now = millis();

  if (rfPresent() && !signalLocked) {
    if (!SignalStartState) {
      signalNotValidStart = now;
      SignalStartState    = true;
    }
    if ((now - signalNotValidStart) >= signalNotValidCountdown) return true;
  } else {
    SignalStartState = false;
  }

  return false;
}

int getFrequencyBand() {
  if (!rfSignalStable()) return 0;

  if (rfFrequency >  3000000 && rfFrequency <  4200000) return 1;
  if (rfFrequency >  6900000 && rfFrequency <  8000000) return 2;
  if (rfFrequency >  9900000 && rfFrequency < 15000000) return 3;
  if (rfFrequency > 17500000 && rfFrequency < 25400000) return 4;
  if (rfFrequency > 26900000 && rfFrequency < 29900000) return 5;

  return 6; // valid signal but outside all LPF bands
}

static uint8_t       lastBand        = 0;
static unsigned long bandTimer       = 0;
static bool          bandTimerActive = false;

bool bandStable(uint8_t band) {
  unsigned long now = millis();

  if (band != lastBand) {
    lastBand        = band;
    bandTimer       = now;
    bandTimerActive = true;
    return false;
  }

  if (bandTimerActive && (now - bandTimer >= 50)) return true;

  return false;
}

bool rfSignalOutOfRangeCheck() {
  return (getFrequencyBand() == 6);
}

// ================= BAND / LPF =================

bool setBand(uint8_t band) {
  if (digitalRead(highvsupplyon)) {
    setSafeState();
    amplifierFault = true;
    FaultReason    = FAULT_BAND_CHANGE_LIVE;
    return false;
  }

  // Lookup table: which LPF pin to set HIGH (others LOW)
  static const uint8_t lpfPins[5] = { LPF1, LPF2, LPF3, LPF4, LPF5 };

  if (band < 1 || band > 5) return false;

  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(lpfPins[i], (i == band - 1) ? HIGH : LOW);
  }

  return true;
}

// ================= RF DETECTOR =================

#define RF_ON_DELAY_MS    100
#define RF_ON_THRESHOLD     30
#define RF_OFF_THRESHOLD    25
#define RF_HOLD_TIME      500
#define RF_FILTER_ALPHA   0.2f

static float rfFiltered    = 0;
static bool  rfState       = false;
static unsigned long rfLastOnTime = 0;

int rfRaw = 0;

void rfDetectorUpdate() {
  rfRaw      = analogRead(rfdetection);
  rfFiltered = (1.0f - RF_FILTER_ALPHA) * rfFiltered + RF_FILTER_ALPHA * rfRaw;

  if (!rfState) {
    if (rfFiltered >= RF_ON_THRESHOLD) {
      rfState     = true;
      rfLastOnTime = millis();
    }
  } else {
    if (rfFiltered >= RF_OFF_THRESHOLD) rfLastOnTime = millis();
    if ((millis() - rfLastOnTime) > RF_HOLD_TIME)  rfState = false;
  }
}

static unsigned long rfOnStartTime  = 0;
static bool          rfTimerActive  = false;

bool rfOnDelayReached() {
  unsigned long now = millis();

  if (rfState) {
    if (!rfTimerActive) { rfOnStartTime = now; rfTimerActive = true; }
    if ((now - rfOnStartTime) >= RF_ON_DELAY_MS) return true;
  } else {
    rfTimerActive = false;
  }

  return false;
}

static unsigned long rfSignalValidtime   = 0;
static bool          rfSignalTimerActive = false;

bool rfSignalDelayReached() {
  unsigned long now = millis();

  if (rfSignalStable()) {
    if (!rfSignalTimerActive) { rfSignalValidtime = now; rfSignalTimerActive = true; }
    if ((now - rfSignalValidtime) >= RF_SIGNAL_DELAY_MS) return true;
  } else {
    rfSignalTimerActive = false;
  }

  return false;
}

bool rfPresent() { return rfState; }

// ================= FAULT CHECKING =================

void checkAllFaults() {
  if (!powerDetectionFault) {
    if (checkProtectionFast()) {
      setSafeState();
      powerDetectionFault = true;
      amplifierFault      = true;
      Serial.print("errorhr");
      FaultReason         = FAULT_HIGH_REFLECTED;
      digitalWrite(buzzer, HIGH);
    } /*else if (checkProtectionSlow()) {
      setSafeState();
      powerDetectionFault = true;
      amplifierFault      = true;
      FaultReason         = FAULT_GATE_DRIFT;
      digitalWrite(buzzer, HIGH);
    }*/
  }

   if (paSupplyStuckFault && !amplifierFault) {
    setSafeState();
    amplifierFault     = true;
    FaultReason        = FAULT_PA_SUPPLY_STUCK;
    digitalWrite(buzzer, HIGH);
   } 
  

  if (!uartLatchedFault) {
    if (UART_latchedFault()) {
      setSafeState();
      uartLatchedFault = true;
      amplifierFault   = true;
      FaultReason      = FAULT_UART_COMM;
      digitalWrite(buzzer, HIGH);
    }
  }

  if (!adcFaultLatch) {
    if (ADC_fault()) {
      setSafeState();
      adcFaultLatch  = true;
      amplifierFault = true;
      FaultReason    = FAULT_ADC_HARDWARE;
      digitalWrite(buzzer, HIGH);
    }
  }

  // Supply rail faults from main MCU's own ADC (IVcheck).
  // Routed through the soft-fault path because brownouts and transient
  // overcurrents are recoverable. The hardware SR latch is the primary
  // fast protection on the 50V rail; this is a software backup.
if (!tempFault) {
  if (lowVoltageFault) {
    tempFault   = true;
    FaultReason = FAULT_LOW_VOLTAGE;      // 18V driver supply
    digitalWrite(buzzer, HIGH);
  } else if (paSupplyFault) {
    tempFault   = true;
    FaultReason = FAULT_PA_SUPPLY_LOW;    // 50V PA supply missing when requested
    digitalWrite(buzzer, HIGH);
  } else if (highCurrentFault) {
    tempFault   = true;
    FaultReason = FAULT_HIGH_CURRENT_AVR;
    digitalWrite(buzzer, HIGH);
  }
}

  // Temperature checks — values updated every loop from secondary MCU UART packets.
  if (tempHeatsink >= TEMP_HEATSINK_HARD) {
    setSafeState();
    amplifierFault = true;
    FaultReason    = FAULT_TEMP_CRITICAL;
    digitalWrite(buzzer, HIGH);
  } else if (!tempFault) {
    if (tempHeatsink >= TEMP_HEATSINK_SOFT) {
      tempFault   = true;
      FaultReason = FAULT_TEMP_HIGH;
      digitalWrite(buzzer, HIGH);
    } else if (tempInner >= TEMP_INNER_SOFT) {
      tempFault   = true;
      FaultReason = FAULT_TEMP_HIGH;
      digitalWrite(buzzer, HIGH);
    } else if (tempAir >= TEMP_AIR_SOFT) {
      tempFault   = true;
      FaultReason = FAULT_TEMP_HIGH;
      digitalWrite(buzzer, HIGH);
    }
  }
}

// ================= BIAS STARTUP VERIFICATION =================
// NOTE: Gate bias voltage is derived from the 50V supply.
// With the supply off in AMP_INIT, gate voltages read 0V — a startup
// voltage check is therefore not possible here.
// Bias is confirmed functionally in AMP_LOW_TRANSMIT: the machine waits
// for supply current to appear (>50mA) before routing RF to the amp,
// and faults if current exceeds the leakage limit (>650mA) before RF arrives.

// ================= PIN INIT =================

void AMP_pin_init() {
  pinMode(att1, OUTPUT);  digitalWrite(att1, LOW);
  pinMode(att2, OUTPUT);  digitalWrite(att2, LOW);
  pinMode(att3, OUTPUT);  digitalWrite(att3, LOW);
  pinMode(att4, OUTPUT);  digitalWrite(att4, LOW);
  pinMode(att5, OUTPUT);  digitalWrite(att5, LOW);
  pinMode(att6, OUTPUT);  digitalWrite(att6, LOW);
  pinMode(att7, OUTPUT);  digitalWrite(att7, LOW);
  pinMode(att8, OUTPUT);  digitalWrite(att8, LOW);

  pinMode(rfout, OUTPUT); digitalWrite(rfout, LOW);

  pinMode(TBL1, OUTPUT);  digitalWrite(TBL1, HIGH);
  pinMode(TBL2, OUTPUT);  digitalWrite(TBL2, HIGH);
  pinMode(TBL3, OUTPUT);  digitalWrite(TBL3, HIGH);
  pinMode(TBL4, OUTPUT);  digitalWrite(TBL4, HIGH);
  pinMode(TBL5, OUTPUT);  digitalWrite(TBL5, HIGH);
  pinMode(TBL6, OUTPUT);  digitalWrite(TBL6, HIGH);
  pinMode(TBL7, OUTPUT);  digitalWrite(TBL7, HIGH);

  pinMode(TBC1, OUTPUT);  digitalWrite(TBC1, LOW);
  pinMode(TBC2, OUTPUT);  digitalWrite(TBC2, LOW);
  pinMode(TBC3, OUTPUT);  digitalWrite(TBC3, LOW);
  pinMode(TBC4, OUTPUT);  digitalWrite(TBC4, LOW);
  pinMode(TBC5, OUTPUT);  digitalWrite(TBC5, LOW);
  pinMode(TBC6, OUTPUT);  digitalWrite(TBC6, LOW);
  pinMode(TBC7, OUTPUT);  digitalWrite(TBC7, LOW);

  pinMode(TBLCsw, OUTPUT); digitalWrite(TBLCsw, LOW);

  pinMode(LPF1, OUTPUT);  digitalWrite(LPF1, LOW);
  pinMode(LPF2, OUTPUT);  digitalWrite(LPF2, LOW);
  pinMode(LPF3, OUTPUT);  digitalWrite(LPF3, LOW);
  pinMode(LPF4, OUTPUT);  digitalWrite(LPF4, LOW);
  pinMode(LPF5, OUTPUT);  digitalWrite(LPF5, LOW);

  pinMode(biasenable, OUTPUT); digitalWrite(biasenable, LOW);
  pinMode(highvsupplyon, OUTPUT); digitalWrite(highvsupplyon, LOW);
  pinMode(rfswitch,      OUTPUT); digitalWrite(rfswitch,      LOW);
  pinMode(buzzer,        OUTPUT); digitalWrite(buzzer,        LOW);

  pinMode(QofSR, INPUT);
  pinMode(RofSR, OUTPUT); digitalWrite(RofSR, LOW);

  pinMode(rfdetection,     INPUT);
  pinMode(rffrequency,     INPUT);
  pinMode(IsenseEighteenV, INPUT);
  pinMode(IsenseFiftyV,    INPUT);
  pinMode(VsenseEighteenV, INPUT);
  pinMode(VsenseFiftyV,    INPUT);
}

// ================= SAFE STATE / RESET HELPERS =================

void setSafeState() {
  digitalWrite(highvsupplyon, LOW);
  digitalWrite(biasenable,    LOW);
  digitalWrite(rfswitch,      LOW);
  digitalWrite(rfout,         LOW);  // RF to dummy load — always safe state
}

void tunerDefaultOffState() {
  digitalWrite(TBL1, HIGH); digitalWrite(TBL2, HIGH); digitalWrite(TBL3, HIGH);
  digitalWrite(TBL4, HIGH); digitalWrite(TBL5, HIGH); digitalWrite(TBL6, HIGH);
  digitalWrite(TBL7, HIGH);

  digitalWrite(TBC1, LOW);  digitalWrite(TBC2, LOW);  digitalWrite(TBC3, LOW);
  digitalWrite(TBC4, LOW);  digitalWrite(TBC5, LOW);  digitalWrite(TBC6, LOW);
  digitalWrite(TBC7, LOW);

  digitalWrite(TBLCsw, LOW);
}

void attenuatorReset() {
  // 0x00 = all attenuator stages engaged = minimum power to amp = safe state.
  // (0xFF would bypass all stages = maximum power — never use as a reset.)
  PORTC = 0x00;
  // Note: rfout (dummy load relay) is managed by setSafeState(), not here.
}

void LPFreset() {
  digitalWrite(LPF1, LOW);
  digitalWrite(LPF2, LOW);
  digitalWrite(LPF3, LOW);
  digitalWrite(LPF4, LOW);
  digitalWrite(LPF5, HIGH); // highest freq LPF — safest default if LPF not set
}

// ================= ATT FUNCTIONS =================

// Generous limits for TUNE_WAIT — used immediately after relay settle.
// The lookup table may not hit the target exactly for all input levels,
// and the ADC is still freshly snapped after the mask change. These catch
// complete attenuator failures, not table granularity.
#define MAX_ATT_WAIT_LOW    10.0f   // for low power mode  (target 1.5W, must be < Pin_avg)
#define MAX_ATT_WAIT_HIGH   15.0f   // for high power mode (target 5W,   must be < Pin_avg)

// Tight limits for TUNE_VERIFY — what the amp should sustainably receive.
// Unchanged — these stay as 5W and 7W.
#define MAX_ATT_OUTPUT_LOW   5.0f
#define MAX_ATT_OUTPUT_HIGH  7.0f

#define MAX_ATT_WAIT_MED      12.0f   // generous post-relay-settle limit for 3W mode
#define MAX_ATT_OUTPUT_MED     4.5f   // verify window limit for 3W mode
#define AMP_MIN_EFFICIENCY_MED_TX  0.30f   // 30% efficiency floor for 3W input

#define MAX_INPUT_POWER_EMERGENCY 17.0f

#define ATT_PORT PORTC  // att1–att8 = D30–D37 = PORTC on ATmega2560
                        // 0x00 = all stages engaged (min power) — safe state
                        // 0xFF = all stages bypassed (max power) — never use as default

// Initial attenuator mask during TUNE_MEASURE_INPUT, applied before the first
// power measurement. 0x3F = 0b00111111: bits 7,6 engaged (6+3 = 9 dB), all
// other stages bypassed. 9 dB chosen as a compromise: enough headroom at
// the dummy load for the expected drive levels (up to ~20W), while keeping
// the AD8310 detector in a usable dynamic range. Max attenuation (0x00,
// 18.5 dB) would drop a 5W drive to 70 mW at the bridge — too low for the
// detector to resolve well.
#define INITIAL_ATT_MASK 0xFC

bool powerOutOfRange(float measured, float expected) {
  return (fabsf(measured - expected) / expected) > 0.2f;
}

float expectedOutputPower(float Pin, float att_db) {
  return Pin * powf(10.0f, -att_db / 10.0f);
}

float maskToDb(uint8_t mask) {
  float db = 0;
  if (!(mask & (1 << 0))) db += 6.0f;   // bit 0 → D37 → 6dB stage
  if (!(mask & (1 << 1))) db += 3.0f;   // bit 1 → D36 → 3dB stage
  if (!(mask & (1 << 2))) db += 3.0f;   // bit 2 → D35 → 3dB stage
  if (!(mask & (1 << 3))) db += 2.0f;   // bit 3 → D34 → 2dB stage
  if (!(mask & (1 << 4))) db += 2.0f;   // bit 4 → D33 → 2dB stage
  if (!(mask & (1 << 5))) db += 1.0f;   // bit 5 → D32 → 1dB stage
  if (!(mask & (1 << 6))) db += 1.0f;   // bit 6 → D31 → 1dB stage
  if (!(mask & (1 << 7))) db += 0.5f;   // bit 7 → D30 → 0.5dB stage
  return db;
}

uint8_t getAttMaskFromTable(float inputPower, InputPowerTarget_t target) {
  int index = (int)(inputPower + 0.5f);
  if (index < 0)   index = 0;
  if (index > 120) index = 120;
  switch (target) {
    case INPUT_PWR_HIGH: return pgm_read_byte(&attTable_5W[index]);
    case INPUT_PWR_MED:  return pgm_read_byte(&attTable_3W[index]);
    default:             return pgm_read_byte(&attTable_1W5[index]);
  }
}

// Reset flag — set by resetInputPowerStable(), consumed inside inputPowerStable()
static bool inputPowerStableResetPending = false;
static bool avgInputPowerResetPending    = false;

void resetInputPowerStable() {
  inputPowerStableResetPending = true;
  avgInputPowerResetPending    = true;  // always reset both together
}

bool inputPowerStable(float p) {
  static float avg  = 0;
  static bool  firstinit = false;

  // External reset request — treat next call as fresh initialisation
  if (inputPowerStableResetPending) {
    firstinit = false;
    inputPowerStableResetPending = false;
  }

  if (!firstinit) { avg = p; firstinit = true; return true; }

  float diff = fabsf(p - avg);
  avg = avg * 0.8f + p * 0.2f;
  return (diff < avg * 0.3f);
}

inline void applyAttenuatorMask(uint8_t mask) {
  ATT_PORT = mask;
}

bool getAveragedInputPowerNB(float &result) {
  static float         sum   = 0;
  static uint16_t      count = 0;
  static unsigned long start = 0;

  // External reset — discard any partial accumulation from a failed tune attempt
  if (avgInputPowerResetPending) {
    sum   = 0;
    count = 0;
    avgInputPowerResetPending = false;
  }

  if (count == 0) start = millis();

  sum += ADC2_getPower(ADC2_FORWARD);
  count++;

  if (millis() - start >= 100) {
    result = sum / count;
    sum    = 0;
    count  = 0;
    return true;
  }

  return false;
}

bool inputPowerTooHigh(float p, InputPowerTarget_t target) {
  switch (target) {
    case INPUT_PWR_HIGH: return p > MAX_ATT_OUTPUT_HIGH;
    case INPUT_PWR_MED:  return p > MAX_ATT_OUTPUT_MED;
    default:             return p > MAX_ATT_OUTPUT_LOW;
  }
}

bool inputPowerTooHighWait(float p, InputPowerTarget_t target) {
  switch (target) {
    case INPUT_PWR_HIGH: return p > MAX_ATT_WAIT_HIGH;
    case INPUT_PWR_MED:  return p > MAX_ATT_WAIT_MED;
    default:             return p > MAX_ATT_WAIT_LOW;
  }
}

void emergencyInputPowerShutdown() {
  float Pin = ADC2_getPower(ADC2_FORWARD);
  float limit;
  if (digitalRead(rfout)) {
    // RF going to amp — use the current target's verify limit
    switch (currentInputTarget) {
      case INPUT_PWR_HIGH: limit = MAX_ATT_OUTPUT_HIGH; break;
      case INPUT_PWR_MED:  limit = MAX_ATT_OUTPUT_MED;  break;
      default:             limit = MAX_ATT_OUTPUT_LOW;   break;
    }
  } else {
    limit = MAX_INPUT_POWER_EMERGENCY;   // dummy load, generous limit
  }

  if (Pin > limit) {
    setSafeState();
    attenuatorReset();
    tunerDefaultOffState();
    LPFreset();
    amplifierFault = true;
    FaultReason    = FAULT_EMERGENCY_SHUTDOWN;
    digitalWrite(buzzer, HIGH);
  }
}

// ================= PERFORMANCE MONITORING =================

// Returns amp current in amps from ADS1115 ADC1 CH2.
// 0.1V/A scaling, ADS1115 getAvg returns voltage already.
float getAmpCurrent() {
  return ADC1_getAvg(ADC1_AMP_ISENSE, 8) / ISENSE_V_PER_A;
}

// Gain in dB: ratio of output forward power to input forward power.
float computeGain() {
  float Pin  = ADC2_getPower(ADC2_FORWARD);
  float Pout = ADC1_getPower(ADC1_FORWARD);
  if (Pin < 0.001f || Pout < 0.001f) return 0.0f;
  float gain = 10.0f * log10f(Pout / Pin);
  if (!isfinite(gain)) return 0.0f;
  return gain;
}

// DC input power from 50V supply (W).
float computeDCInputPower() {
  return U50V * getAmpCurrent();
}

// Efficiency: RF output / DC input power.
// Returns 0 if DC input is negligible to avoid divide-by-zero.
float computeEfficiency() {
  float Pdc = computeDCInputPower();
  if (Pdc < 1.0f) return 0.0f;
  return ADC1_getPower(ADC1_FORWARD) / Pdc;
}

// Check amplifier performance — called every loop while 50V is on and transmitting.
// Returns false and sets FaultReason if something trips. Caller handles state transition.
bool checkAmpPerformance(bool &hardFault, InputPowerTarget_t target) {
  hardFault = false;
  static uint8_t sensorFaultCount = 0;
  static uint8_t effLowCount      = 0;
  static uint8_t gainLowCount     = 0;

  float minEff;
  switch (target) {
    case INPUT_PWR_HIGH: minEff = AMP_MIN_EFFICIENCY_HIGH_TX; break;
    case INPUT_PWR_MED:  minEff = AMP_MIN_EFFICIENCY_MED_TX;  break;
    default:             minEff = AMP_MIN_EFFICIENCY_LOW_TX;  break;
  }
  float Pout = ADC1_getPower(ADC1_FORWARD);
  if (Pout < 1.0f) {
    // Power too low to check — reset all counters so they don't
    // carry stale counts into the next transmission
    sensorFaultCount = 0;
    effLowCount      = 0;
    gainLowCount     = 0;
    return true;
  }

  float gain = computeGain();
  float eff  = computeEfficiency();
  float Iamp = getAmpCurrent();

  // Impossible efficiency — current sensor failed or disconnected
  // Efficiency below minimum — skip during SSB: inter-syllable dropouts cause
// Pout to drop instantly while Iamp drains slowly, giving false low readings
  if (Pout > 10.0f && Iamp > 1.0f && eff > 1.0f) {
    if (++sensorFaultCount >= 200) {
      sensorFaultCount = 0;
      FaultReason = FAULT_CURRENT_SENSOR;
      hardFault   = true;
      return false;
    }
  } else {
    sensorFaultCount = 0;
  }

  // Efficiency below minimum — sustained to ride through dropout transient
  // where Iamp buffer drains slowly after RF stops
  if (!ssbTransmitActive && U50V > 40.0f && Iamp > 1.0f && eff < minEff) {
    if (++effLowCount >= 30) {   // ~30ms
      effLowCount = 0;
      Serial.print(computeEfficiency());
      Serial.print(computeDCInputPower());
      Serial.print(ADC1_getPower(ADC1_FORWARD), 2);
      FaultReason = FAULT_EFFICIENCY_LOW;
      hardFault   = true;
      return false;
    }
  } else {
    effLowCount = 0;
  }

  // Gain below minimum — sustained to ride through input/output ADC
  // disagreement as radio fades
  if (gain < AMP_MIN_GAIN_DB) {
    if (++gainLowCount >= 30) {  // ~30ms
      gainLowCount = 0;
      FaultReason = FAULT_GAIN_LOW;
      return false;
    }
  } else {
    gainLowCount = 0;
  }

  return true;
}

// Check SR latch (hardware overcurrent trip from comparator).
// QofSR HIGH = tripped. Only call when 50V supply is active.
bool checkSRLatch() {
  return digitalRead(QofSR) == HIGH;
}

// SWR monitoring with transient tolerance.
// Returns true if a fault condition is detected.
// sustainedCount: caller-owned counter, incremented each loop, reset on SWR OK.
bool checkOutputSWR(uint8_t &sustainedCount) {
  float swr = SWR_output();

  // Sustained above sustained limit — soft fault after ~3 consecutive readings
  if (swr > SWR_SUSTAINED_LIMIT && !ssbTransmitActive) {
    sustainedCount++;
    if (sustainedCount >= 5) {
      FaultReason = FAULT_SWR_HIGH;
      Serial.print("swr6");
      return true;
    }
  } else if (swr > SWR_SUSTAINED_LIMIT && ssbTransmitActive) {
    sustainedCount++;
    if (sustainedCount >= 40) {
      FaultReason = FAULT_SWR_HIGH;
      Serial.print("swr8");
      return true;
    }
  } else {
    sustainedCount = 0;
  }
  return false;
}

bool checkInputSWR(uint8_t &sustainedCount) {
  float swr = SWR_input();

  if (getAmpCurrent() < 1.0f) return false;

  if (swr > SWR_INPUT_SUSTAINED_LIMIT && !ssbTransmitActive) {
    sustainedCount++;
    if (sustainedCount >= 5) {
      Serial.print("error6");
      FaultReason = FAULT_INPUT_SWR_HIGH;
      return true;
    }
  } else if (swr > SWR_INPUT_SUSTAINED_LIMIT && ssbTransmitActive) {
    sustainedCount++;
    if (sustainedCount >= 30) {
      Serial.print("error11");
      FaultReason = FAULT_INPUT_SWR_HIGH;
      return true;
    }
  } else {
    sustainedCount = 0;
  }

  return false;
}

void setup() {
  Serial.begin(115200);   // debug
  Serial1.begin(115200);  // MCU-to-MCU

  AMP_pin_init();

  // Fresh bootID every power-up — lets the secondary MCU detect a restart of
  // this MCU via the syncFault mechanism. Previously this was persisted to
  // EEPROM, which made the ID stable across reboots and defeated the purpose
  // of restart detection.
  uint32_t bootID = analogRead(A1) ^ analogRead(A0) ^ micros();

  // UART
  UART_setCallback(myUARTHandler);
  UART_setConfig(0xAA, 10, 10, 200, 5000, 100);
  UART_setRangeTable(myRanges, sizeof(myRanges) / sizeof(myRanges[0]));
  UART_init(Serial1, bootID);

  // Hold SR latch reset asserted at boot — released cleanly by AMP_INIT
  // which does a HIGH→delay→LOW pulse to ensure the latch is actually cleared.
  digitalWrite(RofSR, HIGH);

  // ADC
  Wire.begin();
  Wire.setWireTimeout(25000, true);   // 25ms timeout, auto-reset Wire on bus hang
  ADC_init();

  // Timer1 — free-running counter used as a time reference for the INT4-based
  // frequency detector. NOT input capture (ICP1 would be PB0/pin 48); the ISR
  // samples TCNT1 manually. /8 prescaler → 0.5us per tick.
  TCCR1A = 0;
  TCCR1B = (1 << CS11);   // prescaler /8, normal mode (no WGM bits set)

  // INT4 rising edge (PE4 = pin 2 on ATmega2560)
  EICRB |= (1 << ISC41) | (1 << ISC40);
  EIMSK |= (1 << INT4);

  sei();
}

// ================= LOOP =================

void loop() {
  static unsigned long loopStart    = 0;
  static unsigned long loopTimeAccum = 0;
  static uint16_t      loopCount    = 0;
  static float         avgLoopUs    = 0;

  unsigned long now = micros();
  if (loopStart != 0) {
    loopTimeAccum += (now - loopStart);
    if (++loopCount >= 500) {
      avgLoopUs     = loopTimeAccum / 500.0f;
      loopTimeAccum = 0;
      loopCount     = 0;
    }
  }
   
  UART_readSerial();
  UART_parse();
  UART_sendHeartbeat();
  UART_checkHeartbeat();
  UART_sendBootID();
  UART_updateFaults();

  ADC_update();

  emergencyInputPowerShutdown();
  IVcheck();

  checkAllFaults();

  if (amplifierFault) {
    // Don't bypass the state machine — let AMP_FAULT entry actions handle
    // safe state, buzzer, and UART fault report
    AmpState = AMP_FAULT;
    // Still call updateAmpState so entry actions fire this loop iteration
    updateAmpState();
    //return;
  }

  // Temperature soft fault — checked every loop since temps arrive via UART.
  // Does not require power cycle; waits for operator reset + cooling to 60°C.
  if (tempFault && AmpState != AMP_SOFT_FAULT && AmpState != AMP_FAULT) {
    setSafeState();
    tunerDefaultOffState();
    attenuatorReset();
    LPFreset();
    AmpState = AMP_SOFT_FAULT;
  }

  rfDetectorUpdate();
  freqDetector();

  rfSignalInvalid = rfSignalNotValidTimeout();

  updateAmpState();

  checkAllFaults();

  sendTelemetry();

  // ===== DEBUG OUTPUT =====
#ifdef DEBUG_SERIAL
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();

    // ----- UART health -----
    Serial.print("UART errs: ");    Serial.print(UART_errorCount());
    Serial.print(" | CommFault: "); Serial.print(UART_commFault());
    Serial.print(" | HB Fault: ");  Serial.println(UART_heartbeatFault());

    // ----- State -----
    Serial.print("AmpState: ");     Serial.print(AmpState);
    Serial.print(" | FaultReason: "); Serial.println((int)FaultReason);

    // ----- RF power -----
    Serial.print("IN  Pf: ");  Serial.print(ADC2_getPower(ADC2_FORWARD),  2);
    Serial.print("W  SWR: ");  Serial.print(SWR_input(),  2);
    Serial.print(" | OUT Pf: "); Serial.print(ADC1_getPower(ADC1_FORWARD), 2);
    Serial.print("W  SWR: ");   Serial.print(SWR_output(), 2);
    Serial.print(" | Fault: "); Serial.println(amplifierFault ? "YES" : "NO");

    // ----- Amp performance -----
    Serial.print("Gain: ");    Serial.print(computeGain(), 1);
    Serial.print("dB | Eff: "); Serial.print(computeEfficiency() * 100.0f, 1);
    Serial.print("% | Iamp: "); Serial.print(getAmpCurrent(), 3);
    Serial.print("A | U50V: "); Serial.print(U50V, 1);
    Serial.println("V");

    // ----- Gate voltages -----
    Serial.print("Gate1: "); Serial.print(ADC2_getAvg(ADC2_GATE1, 8), 3);
    Serial.print("V | Gate2: "); Serial.println(ADC2_getAvg(ADC2_GATE2, 8), 3);

    // ----- RF detector -----
    Serial.print("RF raw: ");      Serial.print(rfRaw);
    Serial.print(" | filtered: "); Serial.print(rfFiltered, 2);
    Serial.print(" | present: ");  Serial.print(rfPresent()      ? "Y" : "N");
    Serial.print(" | valid: ");    Serial.print(rfSignalValid()   ? "Y" : "N");
    Serial.print(" | stable: ");   Serial.println(rfSignalStable() ? "Y" : "N");

    // ----- Frequency detector -----
    Serial.print("Freq: ");    Serial.print(rfFrequency, 0);
    Serial.print("Hz | err: "); Serial.print(rfError, 4);
    Serial.print(" | jitter: "); Serial.print(rfJitter, 1);
    Serial.print("us | locked: "); Serial.print(signalLocked ? "Y" : "N");
    Serial.print(" | band: ");  Serial.println(getFrequencyBand());

    // ----- SR latch -----
    Serial.print("SR latch tripped: "); Serial.println(checkSRLatch() ? "YES" : "no");

    // ----- Temperatures (from secondary MCU) -----
    Serial.print("Heatsink: "); Serial.print(tempHeatsink, 1);
    Serial.print("C | Inner: "); Serial.print(tempInner, 1);
    Serial.print("C | Air: ");   Serial.print(tempAir, 1);
    Serial.print("C | TempFault: "); Serial.println(tempFault ? "YES" : "no");
    Serial.print("lastTxBand: ");    Serial.print(lastTxBand);
    Serial.print(" | lastTransBand: "); Serial.println(lastTransmittedBand);
    Serial.print("avg lt");
    Serial.println(avgLoopUs);
  }
#endif
  loopStart = now;
}

static inline bool rfActuallyPresent() {
  return analogRead(rfdetection) >= RF_ON_THRESHOLD;
}

// ================= STATE MACHINE =================

void updateAmpState() {
  static AmpState_t lastStateAtEntry = AMP_OFF;

  // Detect any state change since the last call to this function —
  // whether it happened inside the previous switch, or in loop() (the
  // amplifierFault → AMP_FAULT path). PrevAmpState is preserved as the
  // state we just came FROM (used by entry-action blocks like
  // AMP_FIRST_ADJUST_INPUT's comingFromReady check).
  if (AmpState != lastStateAtEntry) {
    PrevAmpState     = lastStateAtEntry;
    stateEntryDone   = false;
    lastStateAtEntry = AmpState;
  }

  switch (AmpState) {

    case AMP_OFF:
      startupcycle++;
      if (startupcycle >= startupgrace) AmpState = AMP_INIT;
      break;

    case AMP_INIT:
      // Clear transient faults — impossible to have fired before init
      adcFaultLatch        = false;
      lowVoltageFault      = false;
      paSupplyFault        = false;
      highCurrentFault     = false;
      powerDetectionFault  = false;
      tempFault            = false;
      ADC_clearFault();

      // Pulse the SR latch reset line — required to actually clear a
      // previously-tripped latch. Just driving LOW (the old code) only
      // released a reset that was never asserted on a soft-fault recovery,
      // leaving the latch stuck in its tripped state from the original fault.
      digitalWrite(RofSR, HIGH);
      delayMicroseconds(10);
      digitalWrite(RofSR, LOW);

      digitalWrite(buzzer, LOW);   // silence the alarm on reset

      setSafeState();
      tunerDefaultOffState();
      attenuatorReset();
      LPFreset();
      ADC_setBand(0);   // reset to fallback calibration until band is confirmed
      // Bias verification not possible here — gate voltage is derived from
      // the 50V supply which is off. Bias is confirmed in AMP_LOW_TRANSMIT.
      AmpState = AMP_IDLE;
      break;

    case AMP_IDLE:
      if (amplifierFault) { AmpState = AMP_FAULT; break; }

      setSafeState();
      tunerDefaultOffState();
      //attenuatorReset();
      applyAttenuatorMask(INITIAL_ATT_MASK);
      LPFreset();
      if(ssbTransmitActive) ssbTransmitActive = false;
      if (rfOnDelayReached()) AmpState = AMP_FIRST_STARTUP;      
      break;                                                       

    case AMP_FIRST_STARTUP: {
      if (amplifierFault) { AmpState = AMP_FAULT; FaultReason = FAULT_AMPLIFIER; break; }
      if (rfSignalInvalid) { AmpState = AMP_SOFT_FAULT; FaultReason = FAULT_RF_SIGNAL_INVALID; break; }

      static unsigned long bandFailStart  = 0;
      static bool          bandFailActive = false;

      if (!stateEntryDone) {
        digitalWrite(highvsupplyon, LOW);
        digitalWrite(biasenable,    LOW);
        digitalWrite(rfswitch,      HIGH);
        digitalWrite(rfout,         LOW);
        tunerDefaultOffState();
        attenuatorReset();
        bandFailActive = false;   // ← reset stale state from previous visit
        stateEntryDone = true;
        ssbTransmitActive = false;
      }

      if (!rfState) { AmpState = AMP_IDLE; break; }
      if (!rfSignalDelayReached()) break;

      // Only read band when signal is actively stable — on SSB, lock can drop
      // momentarily between syllables. If not stable right now, wait rather than
      // acting on a potentially noisy frequency reading.
      if (!rfSignalStable()) break;

      uint8_t band = getFrequencyBand();
        if (band == 6) { AmpState = AMP_SOFT_FAULT; FaultReason = FAULT_BAND_INVALID; break; }
        if (band == 0) break;   // signal not stable yet — wait for EWMA to converge

      if (!bandStable(band)) {
        if (!bandFailActive) { bandFailStart = millis(); bandFailActive = true; }
        if (millis() - bandFailStart > 500) { AmpState = AMP_SOFT_FAULT; FaultReason = FAULT_BAND_UNSTABLE; break; }
        break;
      }
      bandFailActive = false;

      if (!setBand(band)) { AmpState = AMP_SOFT_FAULT; FaultReason = FAULT_BAND_SET_FAILED; break; }

      lastTransmittedBand = band;
      ADC_setBand(band);   // select per-band AD8310 calibration constants
      AmpState = AMP_FIRST_ADJUST_INPUT;
      break;
    }

    case AMP_FIRST_ADJUST_INPUT: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }
      if (rfSignalInvalid) { AmpState = AMP_SOFT_FAULT; FaultReason = FAULT_RF_SIGNAL_INVALID; break; }

      static bool tuneInProgress = false;
      // Entry actions
      // Supply stays ON if coming from READY (operator between SSB overs, band changed)
      // Supply OFF in all other cases
      if (!stateEntryDone) {
        bool comingFromReady = (PrevAmpState == AMP_READY);
        if (!comingFromReady) {
          digitalWrite(highvsupplyon, LOW);
          digitalWrite(biasenable,    LOW);
        } else {
          digitalWrite(biasenable, LOW);
        }
        digitalWrite(rfswitch, HIGH);
        digitalWrite(rfout,    LOW);
        tunerDefaultOffState();
        tuneState      = TUNE_IDLE;
        tuneInProgress = false;   // ← reset stale state from previous visit
        ssbTransmitActive = false; 
        resetInputPowerStable();  // ← also reset avg — old power level from previous band is irrelevant
        stateEntryDone = true;
      }

      if (!rfPresent()) { AmpState = AMP_IDLE; break; }

      if (runInputTuning(INPUT_PWR_LOW)) {
        tuneInProgress = false;
        AmpState = AMP_LOW_TRANSMIT;
        break;
      }

      if (tuneState != TUNE_IDLE) {
        tuneInProgress = true;
      } else if (tuneInProgress) {
        tuneInProgress = false;
        AmpState = AMP_SOFT_FAULT;
      }
      break;
    }

    case AMP_LOW_TRANSMIT: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }
      static uint8_t swrInputCount  = 0; 
      static uint8_t       swrCount        = 0;
      static bool          loadSwitched    = false;
      static unsigned long loadSwitchTime  = 0;   // relay settle after load switch
      static unsigned long confirmStart    = 0;
      static bool          confirmStarted  = false;  
      static unsigned long biasWaitStart = 0;
      static unsigned long lowPowerStart = 0;
      // Entry actions — fresh per-session initialisation lives here,
      // runs exactly once per entry into this state.
      if (!stateEntryDone) {
        digitalWrite(highvsupplyon, HIGH);
        digitalWrite(biasenable,    HIGH);
        digitalWrite(rfswitch,      HIGH);
        digitalWrite(rfout,         LOW);    // RF to dummy load until current confirmed
        tunerDefaultOffState();
        resetProtectionSlow();               // fresh gate voltage reference for this session
        ADC_resetPeaks(); 
        swrInputCount  = 0;
        swrCount       = 0;
        loadSwitched   = false;
        confirmStarted = false;
        ssbTransmitActive = false; 
        biasWaitStart  = 0;
        lowPowerStart  = 0;
        stateEntryDone = true;
      }

      if (!rfPresent()) { AmpState = AMP_IDLE; break; }

      // SR latch check — hardware overcurrent trip
      if (checkSRLatch()) {
        setSafeState();
        FaultReason = FAULT_OVERCURRENT_HW;
        AmpState    = AMP_SOFT_FAULT;
        break;
      }

      if (!loadSwitched) {
        float Iamp = getAmpCurrent();

        if (Iamp < 0.05f) {
          // Bias not up yet — start timeout on first loop
          if (biasWaitStart == 0) biasWaitStart = millis();

          if (millis() - biasWaitStart >= BIAS_STARTUP_TIMEOUT_MS) {
            // Gate voltage may be wrong, bias circuit failed, or amp not connected
            setSafeState();
            Serial.print("error8");
            FaultReason = FAULT_BIAS_STARTUP;
            AmpState    = AMP_SOFT_FAULT;
          }
          break;
        }

        biasWaitStart = 0;   // current rose — cancel timeout

        if (Iamp > AMP_LEAKAGE_LIMIT_LOW) {
          setSafeState();
          Serial.print("error4");
          FaultReason    = FAULT_LEAKAGE_CURRENT;
          amplifierFault = true;
          break;
        }

        digitalWrite(rfout, HIGH);
        loadSwitched   = true;
        loadSwitchTime = millis();
        break;
      }

      // Wait for rfout relay to settle before trusting output measurements.
      // Same rationale as TUNE_WAIT's 50ms guard.
      if (millis() - loadSwitchTime < 50) break;

      // SR latch re-check after load switch — RF now reaching amp
      if (checkSRLatch()) {
        setSafeState();
        FaultReason = FAULT_OVERCURRENT_HW;
        AmpState    = AMP_SOFT_FAULT;
        break;
      }

      // Output power gate — amp must produce minimum power before we start
      // the confirmation window. Catches no-input, disconnected output, or
      // a completely inactive PA stage.
      float Pout = ADC1_getPower(ADC1_FORWARD);
      if (Pout < MIN_LOW_TX_OUTPUT_W) {
        if (lowPowerStart == 0) lowPowerStart = millis();

        // Only check overcurrent after Iamp buffer has had time to settle.
        // Avoids false fault during RF-off transient where Pout drops
        // instantly but Iamp drains over ~256ms.
        if ((millis() - lowPowerStart >= 400UL) && getAmpCurrent() > AMP_LEAKAGE_LIMIT_A * 3.0f) {
          setSafeState();
          Serial.print(getAmpCurrent(), 3);
          Serial.print(ADC1_getPower(ADC1_FORWARD), 2);
          Serial.print("error1");
          FaultReason    = FAULT_LEAKAGE_CURRENT;
          amplifierFault = true;
          break;
        }

        if (millis() - lowPowerStart >= LOW_TX_POWER_TIMEOUT_MS) {
          setSafeState();
          Serial.print("error9");
          FaultReason = FAULT_GAIN_LOW;
          AmpState    = AMP_SOFT_FAULT;
          break;
        }

        confirmStarted = false;
        break;
      }
      lowPowerStart = 0;
      
      // Output SWR monitoring.
      // Reset confirmation window if SWR is elevated (even if not yet faulting)
      // so we guarantee 200ms of fully clean operation, not 200ms since first clean reading.
      bool swrElevated = (SWR_output() > SWR_SUSTAINED_LIMIT);
      if (checkOutputSWR(swrCount)) {
        setSafeState();
        Serial.print("error3");
        AmpState = AMP_SOFT_FAULT;
        break;
      }
      if (swrElevated) confirmStarted = false;

      // replace the existing if (SWR_input() > SWR_SUSTAINED_LIMIT) block with:
      if (checkInputSWR(swrInputCount)) {
        setSafeState();
        Serial.print("error2");
        AmpState = AMP_SOFT_FAULT;
        break;
      }

      // Performance monitoring
      bool hardFault = false;
      if (!checkAmpPerformance(hardFault, INPUT_PWR_LOW)) {
        Serial.print("error10");
        setSafeState();
        AmpState       = hardFault ? AMP_FAULT : AMP_SOFT_FAULT;
        amplifierFault = hardFault;
        break;
      }

      // 200ms confirmation — must be continuously clean
      if (!confirmStarted) {
        confirmStart   = millis();
        confirmStarted = true;
      }

      if (millis() - confirmStart >= LOW_TX_CONFIRM_MS) {
        AmpState = AMP_TRANSMIT_TRANSITION;
      }

      break;
    }

    case AMP_TUNE:
      // PLACEHOLDER — intentionally unreachable in current firmware.
      // The state and its enum value are kept because:
      //   (1) the secondary MCU's getStateString()/UART range table reference its index;
      //   (2) the impedance tuner sub-state machine will live here when implemented.
      // No state currently transitions to AMP_TUNE. If you wire it in, remember
      // to add the supply/bias/relay guard appropriate to whichever state precedes it.
      if (amplifierFault) { AmpState = AMP_FAULT; break; }
      AmpState = AMP_TRANSMIT_TRANSITION;
      break;

    case AMP_TRANSMIT_TRANSITION: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }

      // Entry: dummy load ON during attenuator reconfiguration, rfswitch ON explicitly
      if (!stateEntryDone) {
        digitalWrite(rfswitch, HIGH);   // explicit — READY entry turned this off
        digitalWrite(rfout,    LOW);    // RF to dummy load while switching attenuator
        tuneState           = TUNE_IDLE;
        stateEntryDone      = true;
        ssbTransmitActive = false; 
        currentInputTarget = getBandPowerTarget(lastTransmittedBand);
      }

      // RF loss during transition:
      // — from READY fast-path: go back to READY (supply still warm)
      // — from normal path: go to IDLE
      if (!rfPresent()) {
        AmpState = transitionFromReady ? AMP_READY : AMP_IDLE;
        transitionFromReady = false;
        break;
      }

      // runInputTuning(true) handles the full sequence including
      // the 200ms TUNE_VERIFY monitoring window (input power, SWR, band check)
      static bool transitionTuneInProgress = false;

      if (runInputTuning(currentInputTarget)) {
        transitionTuneInProgress = false;
        transitionFromReady      = false;
        // Do NOT switch rfout here — RF relay switches exclusively in
        // AMP_TRANSMIT_HIGH entry actions, keeping state boundaries clean.
        AmpState = AMP_TRANSMIT_HIGH;
        break;
      }

      if (tuneState != TUNE_IDLE) {
        transitionTuneInProgress = true;
      } else if (transitionTuneInProgress) {
        transitionTuneInProgress = false;
        AmpState = AMP_SOFT_FAULT;
        // FaultReason already set inside runInputTuning()
      }

      break;
    }

    case AMP_TRANSMIT_HIGH: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }
      static uint8_t       swrInputCountHigh = 0;
      static uint8_t       swrCountHigh      = 0;
      static bool          confirmed         = false;
      static unsigned long confirmStartHi    = 0;
      static bool          rfLostActive      = false;
      static unsigned long rfLostStart       = 0;

      if (!stateEntryDone) {
        digitalWrite(rfout, HIGH);
        ADC_resetPeaks();
        swrCountHigh      = 0;
        swrInputCountHigh = 0;
        confirmed         = false;
        confirmStartHi    = 0;
        rfLostActive      = false;
        stateEntryDone    = true;
      }

      if (checkSRLatch()) {
        digitalWrite(rfout, LOW);
        setSafeState();
        FaultReason = FAULT_OVERCURRENT_HW;
        AmpState    = AMP_SOFT_FAULT;
        break;
      }

      if (rfSignalStable() && !ssbTransmitActive) {
        uint8_t currentBandHi = getFrequencyBand();
        if (currentBandHi != 0 && currentBandHi != lastTransmittedBand) {
          digitalWrite(rfout, LOW);
          setSafeState();
          FaultReason = FAULT_BAND_INVALID;
          AmpState    = AMP_SOFT_FAULT;
          break;
        }
      }
      if (rfSignalValid() && ssbTransmitActive) {
        uint8_t currentBandHi = getFrequencyBand();
        if (currentBandHi != 0 && currentBandHi != lastTransmittedBand) {
          digitalWrite(rfout, LOW);
          setSafeState();
          FaultReason = FAULT_BAND_INVALID;
          AmpState    = AMP_SOFT_FAULT;
          break;
        }
      }

      if (!rfPresent()) {
        if (!rfLostActive) {
          rfLostStart  = millis();
          rfLostActive = true;
        }

        if (confirmed && millis() - rfLostStart >= HIGH_TX_RF_OFF_MS) {
          lastTxBand = lastTransmittedBand;
          ADC_resetPeaks();
          AmpState = AMP_READY;
        } else if (!confirmed && millis() - rfLostStart >= HIGH_TX_RF_OFF_MS) {
          if (ssbModeRequested) {
            // SSB mode: short over is normal — return to READY so the
            // next over can re-enter via SSB_VERIFY without full re-init
            lastTxBand = lastTransmittedBand;
            ADC_resetPeaks();
            AmpState = AMP_READY;
          } else {
            lastTxBand = 0;
            AmpState   = AMP_IDLE;
          }
        }
        break;
      } else {
        rfLostActive = false;   // fix 1 — reset when RF returns
      }

      bool swrElevatedHi = (SWR_output() > SWR_SUSTAINED_LIMIT);
      if (checkOutputSWR(swrCountHigh)) {
        digitalWrite(rfout, LOW);
        setSafeState();
        AmpState = AMP_SOFT_FAULT;
        break;
      }
      if (swrElevatedHi) { confirmed = false; confirmStartHi = 0; }

      if (checkInputSWR(swrInputCountHigh)) {
        digitalWrite(rfout, LOW);
        setSafeState();
        AmpState = AMP_SOFT_FAULT;
        break;
      }

      bool hardFaultHi = false;
      if (!checkAmpPerformance(hardFaultHi, currentInputTarget)) {
        digitalWrite(rfout, LOW);
        setSafeState();
        AmpState       = hardFaultHi ? AMP_FAULT : AMP_SOFT_FAULT;
        amplifierFault = hardFaultHi;
        break;
      }

      if (!confirmed) {
        if (confirmStartHi == 0) {
          confirmStartHi = millis();
        } else if (millis() - confirmStartHi >= HIGH_TX_CONFIRM_MS) {
          confirmed         = true;
         // ssbTransmitActive = false;   // fix 2 — re-enable efficiency check after 1s clean TX
        }
      }

      break;
    }

    case AMP_READY: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }

      static unsigned long readyEntryTime = 0;
      static unsigned long readyRFTimer   = 0;
      static bool          readyTimerSet  = false;

      if (!stateEntryDone) {
        digitalWrite(biasenable, LOW);
        digitalWrite(rfswitch,   LOW);   // disconnected until RF confirmed
        digitalWrite(rfout,      LOW);
        resetProtectionSlow();
        readyEntryTime = millis();
        readyTimerSet  = false;
        stateEntryDone = true;
        ssbTransmitActive = false; 
      }

      if (checkSRLatch()) {
        setSafeState();
        FaultReason    = FAULT_OVERCURRENT_HW;
        amplifierFault = true;
        break;
      }

      if (millis() - readyEntryTime >= 100) {
        float Iidle = getAmpCurrent();
        if (Iidle > AMP_IDLE_CURRENT_MAX) {
          setSafeState();
          FaultReason    = FAULT_LEAKAGE_CURRENT;
          amplifierFault = true;
          break;
        }
      }

      if (!rfPresent()) {
        // RF gone — make sure rfswitch is back LOW (may have been raised for freq detection)
        digitalWrite(rfswitch, LOW);

        if (!readyTimerSet) { readyRFTimer = millis(); readyTimerSet = true; }
        if (millis() - readyRFTimer >= READY_IDLE_TIMEOUT) {
          digitalWrite(highvsupplyon, LOW);
          readyTimerSet = false;
          AmpState      = AMP_IDLE;
        }
        break;
      }
      readyTimerSet = false;

      // RF present — raise rfswitch so the divider chain gets signal
      // rfout=LOW and biasenable=LOW so no RF reaches the PA or antenna
      if (!rfOnDelayReached()) break;
      digitalWrite(rfswitch, HIGH);

      if (!rfSignalValid()) break;   // wait for frequency lock with rfswitch now live

      uint8_t currentBand = getFrequencyBand();
      float   currentPeak = ADC2_getPeak(ADC2_FORWARD);

      bool bandMatch  = (lastTxBand > 0) && (currentBand == lastTxBand);

      if (bandMatch) {
        if (ssbModeRequested) {
          transitionFromReady = true;
          AmpState = AMP_SSB_VERIFY;   // biasenable goes HIGH in SSB_VERIFY entry
        } else {
          digitalWrite(biasenable, HIGH);
          transitionFromReady = true;
          AmpState = AMP_TRANSMIT_TRANSITION;
        }
      } else{
        transitionFromReady = false;
        AmpState = AMP_FIRST_STARTUP;
      }

      break;
    }

    case AMP_SSB_VERIFY: {
      if (amplifierFault) { AmpState = AMP_FAULT; break; }

      static bool          verifyStarted = false;
      static unsigned long verifyStart   = 0;
      static float         peakSeen      = 0.0f;

      if (!stateEntryDone) {
        // rfswitch already HIGH from READY
        // rfout stays LOW — don't route to amp until verified
        // Keep current attenuator mask — tuned from FM, don't touch it
        digitalWrite(biasenable, HIGH);   // supply already on
        verifyStarted  = false;
        peakSeen       = 0.0f;
        stateEntryDone = true;
      }

      // RF gone — go back to READY to wait, keep supply warm
      if (!rfPresent()) {
        digitalWrite(biasenable, LOW);
        transitionFromReady = false;
        AmpState = AMP_READY;
        break;
      }

      // Band change check — only when stable to avoid SSB gap false triggers
      if (rfSignalStable()) {
        uint8_t band = getFrequencyBand();
        if (band != 0 && band != lastTransmittedBand) {
          setSafeState();
          transitionFromReady = false;
          Serial.print("bandbadssb");
          AmpState = AMP_FIRST_STARTUP;
          break;
        }
      }

      // Only measure during actual RF presence — ignore SSB gaps
      if (rfActuallyPresent()) {
        if (!verifyStarted) {
          verifyStart   = millis();
          verifyStarted = true;
        }

        float Pin = ADC2_getPower(ADC2_FORWARD);
        if (Pin > peakSeen) peakSeen = Pin;

        // Peak already exceeds target limit — existing mask is too loose
        // Fall through to full re-tune with supply still warm
        if (inputPowerTooHigh(peakSeen, currentInputTarget)) {
          transitionFromReady = true;
          Serial.print("toohighpower");
          AmpState = AMP_TRANSMIT_TRANSITION;
          break;
        }
      }

      if (!verifyStarted) break;
        if (millis() - verifyStart < SSB_VERIFY_MS) break;

        // Verify a meaningful peak was actually seen — rfPresent hold timer could
        // keep verifyStarted true for 500ms with no actual signal on the bridge
        if (peakSeen < SSB_MIN_PEAK_W) {
          digitalWrite(biasenable, LOW);
          transitionFromReady = false;
          Serial.print("toolowpower");
          AmpState = AMP_READY;
          break;
        }

        // All checks passed — go directly to high power TX
          ssbTransmitActive = true;   
          transitionFromReady = false;
          AmpState = AMP_TRANSMIT_HIGH;
        break;
    }
    

    case AMP_SOFT_FAULT:
      if (!stateEntryDone) {
        setSafeState();            // supply off, bias off, rfswitch off, rfout off
        tunerDefaultOffState();
        attenuatorReset();
        LPFreset();
        reportFaultToUI(FaultReason);
        faultReportSent = true;
        stateEntryDone  = true;
      }

      if (softFaultResetRequest) {
        softFaultResetRequest = false;
        faultReportSent       = false;
        FaultReason           = FAULT_NONE;
        tempFault             = false;
        lowVoltageFault       = false;
        highCurrentFault      = false;
        paSupplyFault         = false;    
        AmpState              = AMP_INIT;
      }
      break;

    case AMP_FAULT:
      if (!stateEntryDone) {
        setSafeState();            // supply off, bias off, rfswitch off, rfout off
        tunerDefaultOffState();
        attenuatorReset();
        LPFreset();
        digitalWrite(buzzer, HIGH);
        reportFaultToUI(FaultReason);
        faultReportSent = true;
        stateEntryDone  = true;
      }
      // Hard fault — no software recovery, requires power cycle
      break;

    default:
      // Defensive: AmpState somehow took a value outside the enum (memory
      // corruption, EMI, etc). Force into hard fault with safe outputs.
      setSafeState();
      tunerDefaultOffState();
      attenuatorReset();
      LPFreset();
      digitalWrite(buzzer, HIGH);
      FaultReason    = FAULT_STATE_CORRUPTED;
      amplifierFault = true;
      AmpState       = AMP_FAULT;
      break;
  }

  // Deliberately do NOT update lastStateAtEntry or PrevAmpState here.
  // Any state change made inside the switch is detected on the next call,
  // at which point lastStateAtEntry still holds the value it had when this
  // call began — exactly what we need to detect the transition.
}

// ================= INPUT TUNING SUB-STATE MACHINE =================

// Raw RF presence check — bypasses the hold timer in rfPresent().
// Used inside tuning to confirm actual RF energy is present right now,
// not just that the hold timer from a recent dropout is still running.
// Prevents measuring input power during SSB gaps misread as "RF present".

bool runInputTuning(InputPowerTarget_t target){
  if (!rfPresent()) {
    FaultReason = FAULT_RF_LOST_DURING_TUNE;
    resetInputPowerStable();
    tuneState   = TUNE_IDLE;
    return false;
  }

  switch (tuneState) {

    case TUNE_IDLE:
    applyAttenuatorMask(INITIAL_ATT_MASK);
    tuneTimer = millis();
      tuneState = TUNE_MEASURE_INPUT;
      break;

    case TUNE_MEASURE_INPUT:
      // Set initial attenuation during measurement for a known starting point.
      // INITIAL_ATT_MASK = 0x3F = 9 dB (see definition for rationale).
      applyAttenuatorMask(INITIAL_ATT_MASK);

      if (millis() - tuneTimer < 80) break;

      // Raw diode check — don't average power if there's no actual RF right now.
      if (!rfActuallyPresent()) break;

      {
        float Pin_measured;
        if (!getAveragedInputPowerNB(Pin_measured)) break;
        
        // Pin_measured is post-attenuation. Convert back to actual
        // pre-attenuation power so the table lookup and safety check
        // operate on the real input power at the RF connector.
        float initialAttDb = maskToDb(INITIAL_ATT_MASK);   // = 9.0 dB
        float Pin_actual   = Pin_measured * powf(10.0f, initialAttDb / 10.0f);
        

        if (!inputPowerStable(Pin_actual)) {
          FaultReason = FAULT_TUNE_UNSTABLE;
          resetInputPowerStable();
          tuneState   = TUNE_IDLE;
          return false;
        }

        // Safety check against ACTUAL input power — protects PCB traces
        // and the first attenuator stage which always sees full power.
        if (Pin_actual > 120.0f) {
          FaultReason = FAULT_INPUT_TOO_HIGH_HW;
          resetInputPowerStable();
          tuneState   = TUNE_IDLE;
          return false;
        }

        // Store actual pre-attenuation power — used in TUNE_WAIT and TUNE_VERIFY
        // for computing expected post-attenuation power and verifying reduction.
        Pin_avg = Pin_actual;

        // Table expects actual input watts, returns mask to attenuate down to target
        currentMask = getAttMaskFromTable(Pin_actual, target);
        tuneState   = TUNE_SET_ATT;
      }
      break;

    case TUNE_SET_ATT:
      applyAttenuatorMask(currentMask);
      tuneTimer = millis();
      tuneState = TUNE_WAIT;
      break;

    case TUNE_WAIT: {
      if (!rfPresent()) {
        FaultReason = FAULT_RF_LOST_DURING_TUNE;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Wait for relay to settle
      if (millis() - tuneTimer < 50) break;

      // Raw diode check before snapshot — a dropout right at relay settle time
      // would cause a large apparent spike in input power reading.
      if (!rfActuallyPresent()) break;

      float Pin_now = ADC2_getPower(ADC2_FORWARD);

      if (inputPowerTooHighWait(Pin_now, target)) {
        FaultReason = FAULT_INPUT_TOO_HIGH_WAIT;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Attenuator reduction check: Pin_now is post-new-attenuation,
      // Pin_avg is actual pre-attenuation power. Any working attenuator
      // must reduce power, so Pin_now must be less than Pin_avg.
      if (Pin_now >= Pin_avg) {
        FaultReason = FAULT_ATT_NO_REDUCTION;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Verify the measured post-attenuation power matches what the table predicts.
      // expected = Pin_actual * 10^(-att_db/10) = what SHOULD come out.
      float att_db   = maskToDb(currentMask);
      float expected = expectedOutputPower(Pin_avg, att_db);

      if (powerOutOfRange(Pin_now, expected)) {
        FaultReason = FAULT_ATT_VERIFY;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Snapshot passed — arm the 200ms monitoring window
      tuneTimer = millis();
      tuneState = TUNE_VERIFY;
      break;
    }

    case TUNE_VERIFY: {
      // Active 200ms monitoring window. Every loop iteration checks:
      //   — RF actually present (raw diode, not hold timer)
      //   — Input power still in range
      //   — Band has not changed
      //   — Output SWR within limits
      // Any failure resets to TUNE_IDLE. Returns true only after full window.

      if (!rfPresent()) {
        FaultReason = FAULT_RF_LOST_DURING_TUNE;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Raw diode check every iteration — if RF drops during the window,
      // abort immediately rather than averaging through a dropout
      if (!rfActuallyPresent()) {
        // Don't fault — RF just momentarily low (SSB gap).
        // Reset window so we only count uninterrupted RF.
        tuneTimer = millis();
        break;
      }

      // Input power must stay in range throughout the window
      float Pin_now = ADC2_getPower(ADC2_FORWARD);
      if (inputPowerTooHigh(Pin_now, target)) {
        FaultReason = FAULT_INPUT_TOO_HIGH_SOFT;
        attenuatorReset();
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Band must not change
      uint8_t band = getFrequencyBand();
      if (band != 0 && band != lastTransmittedBand) {
        FaultReason = FAULT_BAND_INVALID;
        resetInputPowerStable();
        tuneState   = TUNE_IDLE;
        return false;
      }

      // Output SWR
      static uint8_t verifySWRCount = 0;
      float swr = SWR_output();

      if (swr > SWR_TRANSIENT_LIMIT) {
        FaultReason    = FAULT_SWR_HIGH;
        verifySWRCount = 0;
        resetInputPowerStable();
        tuneState      = TUNE_IDLE;
        return false;
      }

      if (swr > SWR_SUSTAINED_LIMIT) {
        if (++verifySWRCount >= 3) {
          FaultReason    = FAULT_SWR_HIGH;
          verifySWRCount = 0;
          resetInputPowerStable();
          tuneState      = TUNE_IDLE;
          return false;
        }
      } else {
        verifySWRCount = 0;
      }

      // Full 200ms window elapsed with no faults — success
      if (millis() - tuneTimer >= TUNE_VERIFY_MS) {
        verifySWRCount = 0;
        tuneState      = TUNE_IDLE;
        return true;
      }

      break;
    }
  }

  return false;
}

// ================= ISR =================

ISR(INT4_vect) {
  uint16_t timer1now = TCNT1;
  uint16_t delta     = timer1now - lastCapture;
  lastCapture        = timer1now;
  periodTicks        = delta;
  lastEdgeTime       = micros();
}
