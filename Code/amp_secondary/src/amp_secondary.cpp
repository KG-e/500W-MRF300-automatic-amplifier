// ================= AMP SECONDARY MCU =================
// Responsibilities:
//   - Read all temperatures (LM35DT on A10, A11, A12)
//   - Read all voltages (5V and 12V rails for 3 boards, 18V, 50V)
//   - Read current on 18V and 50V rails
//   - Report temps and voltage fault flag to main MCU via UART
//   - Drive Nextion HMI display (Serial2)
//   - Drive 6x PWM fans based on heatsink temperature
//   - Forward operator reset from HMI to main MCU, with temperature gate
//   - INIT grace period: skip fault reporting for first INIT_GRACE_MS

// =====================================================================
// SERIAL1 RX BUFFER SIZE REQUIREMENT
// =====================================================================
// The Nextion HMI library blocks for ~15ms per setText/setValue call while
// waiting for an ACK at 9600 baud. During that window, the main MCU is
// streaming telemetry on Serial1 at 115200 baud (~11.5 kB/s). The default
// 64-byte hardware buffer will overflow (~173 bytes arrive in 15ms),
// causing UART CRC errors and eventually commFault latching.
//
// To prevent this, the Serial1 RX buffer must be bumped to 256 bytes.
//
//   PlatformIO:  add to platformio.ini:
//                  build_flags = -DSERIAL_RX_BUFFER_SIZE=256
//
//   Arduino IDE: easiest is to add this line to platform.local.txt:
//                  compiler.cpp.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256
//                (or edit HardwareSerial.h directly)
//
// The line below MAY work if the Arduino core hasn't already included
// HardwareSerial.h before reaching this point — but the build flag above
// is the reliable method. Keep both for safety.
// #define SERIAL_RX_BUFFER_SIZE 256
// =====================================================================
#include "Arduino.h"
#include "UART.h"
#include "Nextion.h"

#include <Arduino.h>
#include "UART.h"
#include "Nextion.h"

// ================= FORWARD DECLARATIONS =================

// Fan PWM
void     fan_pwm_init();
void     setFanPWM(uint8_t duty);
void     updateFans(float heatsinkTemp);

// Sensors
void readTemperatures();
void readVoltages();
bool checkVoltageFaults();
void reportSensors();

// Display
void updateNextionDisplay();
const char* getStateString(uint8_t state);
const char* getFaultString(uint8_t code);
const char* getBandString(uint8_t band);

// UART
void secondaryUARTHandler(uint8_t type, uint16_t value);

// Nextion callbacks
void p1b0PopCallback(void *ptr);
void p1b1PopCallback(void *ptr);

// ================= END FORWARD DECLARATIONS =================

#define DEBUG_SERIAL
// ================= TEMP ENABLE FLAGS =================
#define ENABLE_TEMP_HEATSINK      // A11 — LM35DT on MRF300 TO-247 case
//#define ENABLE_TEMP_INNER       // A12 — LM35DT inner board (currently NC)
//#define ENABLE_TEMP_AIR         // A10 — LM35DT ambient air (currently NC)

// ================= PIN DEFINITIONS =================

#define PinTempInner          A12
#define PinTempHeatsink       A10
#define PinTempAir            A11

#define IsenseEighteenV       A9
#define IsenseFiftyV          A8

#define VsenseFiveVboardTwo   A7
#define VsenseTwelveVboardTwo A6
#define VsenseFiveVboardThree A5
#define VsenseTwelveVboardThree A4
#define VsenseFiveVboardOne   A3
#define VsenseTwelveVboardOne A2
#define VsenseEighteenV       A1
#define VsenseFiftyV          A0

#define rffrequency  2
#define PinFan1     44
#define PinFan2     45
#define PinFan3     46
#define PinFan4     12
#define PinFan5     11
#define PinFan6     10
#define buzzer      49

// ================= UART PACKET TYPES =================
// Types 0x00–0x7F: main MCU sends, secondary receives
// Types 0x80–0xFF: secondary MCU sends, main receives

// Inbound from main MCU (secondary receives these)
#define TYPE_FAULT_REASON    0x10
#define TYPE_AMP_STATE       0x20
#define TYPE_OUT_POWER       0x21   // output forward power W×10
#define TYPE_OUT_SWR         0x22   // output SWR ×100
#define TYPE_IN_POWER        0x23   // input forward power W×10
#define TYPE_IN_SWR          0x24   // input SWR ×100
#define TYPE_AMP_CURRENT     0x25   // amp supply current in mA
#define TYPE_GAIN            0x26   // gain dB×10 offset +500
#define TYPE_EFFICIENCY      0x27   // efficiency integer percent 0–100
#define TYPE_BAND            0x28   // set band 1–5, 0=none
#define TYPE_SSB_MODE        0x74   // HMI SSB button — bidirectional toggle/state

// Outbound to main MCU (secondary sends these)
#define TYPE_TEMP_HEATSINK   0x80
#define TYPE_TEMP_INNER      0x81
#define TYPE_TEMP_AIR        0x82
#define TYPE_VOLTAGE_FAULT   0x83

// Protocol types (bidirectional)
#define TYPE_RESET           0x72

// ================= FAULT REASON CODES =================
// Mirror of main MCU FaultReason_t — used to decide temperature reset gate
#define FAULT_NONE           0
#define FAULT_TEMP_HIGH      50
#define FAULT_TEMP_CRITICAL  51

// ================= VOLTAGE THRESHOLDS =================

#define VOLT_5V_LOW    4.5f
#define VOLT_5V_HIGH   8.5f
#define VOLT_12V_LOW  10.8f
#define VOLT_12V_HIGH 13.2f
#define VOLT_18V_LOW  15.0f
#define VOLT_18V_HIGH 20.0f
#define VOLT_50V_LOW  44.0f
#define VOLT_50V_HIGH 56.0f

// ================= ADC SCALING =================

#define ADC_VREF            5.0f
#define ADC_STEPS        1024.0f
#define ADC_LSB          (ADC_VREF / ADC_STEPS)
#define SCALE_DIVIDED_RAIL (ADC_LSB * 4.4f)   // ÷4 divider on 5V/12V rails
#define SCALE_DIVIDED_RAIL_FIVE (ADC_LSB * 2.0f)
#define SCALE_18V          0.0294f              
#define SCALE_50V          0.079f              
#define SCALE_I18V         0.008f               
#define SCALE_I50V         0.037f               

// LM35DT: 10mV/°C, Vref=5V → 0.4883°C per LSB
#define SCALE_LM35  (ADC_VREF * 1000.0f / ADC_STEPS / 10.0f)

// ================= TEMPERATURE THRESHOLDS =================

#define TEMP_HEATSINK_RESET  60.0f   // must be below this to permit reset after FAULT_TEMP_HIGH

// ================= TEMPERATURE SLEW RATE =================
// Limits how fast the heatsink reading can rise, to reject RF-induced
// voltage spikes on the LM35DT sense line (e.g. sudden false 100°C reads).
// Fall rate is unrestricted — real cooling must be tracked immediately.
// Rate: 1°C per 500ms. Interval is SENSOR_INTERVAL_MS (50ms).
// → Max rise per read = 0.1°C
#define TEMP_MAX_RISE_PER_READ  (1.0f * SENSOR_INTERVAL_MS / 500.0f)

static float tempHeatsinkFiltered = 0.0f;
static bool  tempHeatsinkInitDone = false;

// ================= FAN CONTROL =================

// Fan curve — heatsink case temperature drives all 6 fans together.
// Below FAN_TEMP_ON: fans off.
// At FAN_TEMP_ON: hard jump to FAN_PWM_MIN (30%) to guarantee start.
// FAN_TEMP_ON to FAN_TEMP_MAX: linear ramp FAN_PWM_MIN → 255.
// Above FAN_TEMP_MAX: locked at 255 (100%).

#define FAN_TEMP_ON          35.0f   // °C — turn fans on (rising edge)
#define FAN_TEMP_HYSTERESIS   3.0f   // °C — turn-off threshold is FAN_TEMP_ON - this
#define FAN_TEMP_OFF         (FAN_TEMP_ON - FAN_TEMP_HYSTERESIS)  // °C — turn fans off (falling edge)
#define FAN_TEMP_MAX         70.0f   // °C — fans at 100% (also soft fault threshold on main MCU)
#define FAN_PWM_MIN             77   // 30% of 255 — minimum to guarantee spin-up

static uint8_t fanPWM = 0;    // current fan duty cycle 0–255, tracked for display


#define FAN_PWM_TOP_16BIT 799   // Timer1/Timer5 TOP for 20kHz at 16MHz

void fan_pwm_init() {
  // --- Timer1: D11 (Fan5, OC1A), D12 (Fan4, OC1B) ---
  // COM1A1=1, COM1B1=1: non-inverting (clear on match, set at BOTTOM)
  // WGM13:0 = 1110 → mode 14 (Fast PWM, TOP=ICR1)
  // CS12:0 = 001 → prescaler 1
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS10);
  ICR1   = FAN_PWM_TOP_16BIT;
  OCR1A  = 0;  // Fan5 off
  OCR1B  = 0;  // Fan4 off

  // --- Timer5: D44 (Fan1, OC5C), D45 (Fan2, OC5B), D46 (Fan3, OC5A) ---
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM51);
  TCCR5B = (1 << WGM53)  | (1 << WGM52)  | (1 << CS50);
  ICR5   = FAN_PWM_TOP_16BIT;
  OCR5A  = 0;  // Fan3 off
  OCR5B  = 0;  // Fan2 off
  OCR5C  = 0;  // Fan1 off

  // --- Timer2: D10 (Fan6, OC2A) ---
  // COM2A1=1: non-inverting
  // WGM22:0 = 001 → mode 1 (Phase-correct PWM, TOP=0xFF)
  // CS22:0 = 001 → prescaler 1
  TCCR2A = (1 << COM2A1) | (1 << WGM20);
  TCCR2B = (1 << CS20);
  OCR2A  = 0;  // Fan6 off
}

// Apply 0..255 duty cycle to all fans. Internally scales the value to
// the appropriate range for each timer. Intentionally keeps the same
// 0..255 API as the original analogWrite()-based code so updateFans()
// doesn't need to change its internal math.

void setFanPWM(uint8_t duty) {
  // Timer1 / Timer5 use 0..(TOP+1) for 0..100% duty.
  // Scale 0..255 → 0..800. At duty=0 → 0 (always low),
  // at duty=255 → 800 (always high, since 800 > TOP).
  uint16_t duty16 = (uint16_t)(((uint32_t)duty * (FAN_PWM_TOP_16BIT + 1)) / 255UL);

  OCR1A = duty16;  // Fan5
  OCR1B = duty16;  // Fan4
  OCR5A = duty16;  // Fan3
  OCR5B = duty16;  // Fan2
  OCR5C = duty16;  // Fan1

  OCR2A = duty;    // Fan6 — native 8-bit range, no scaling
}

void updateFans(float heatsinkTemp) {
  uint8_t pwm;

  // Use current fan state to decide which threshold applies.
  // fansOn==true  → use lower threshold FAN_TEMP_OFF before shutting down
  // fansOn==false → require full FAN_TEMP_ON before starting up
  bool fansOn = (fanPWM > 0);
  float lowerThreshold = fansOn ? FAN_TEMP_OFF : FAN_TEMP_ON;

  if (heatsinkTemp < lowerThreshold) {
    // Below the active threshold — fans off (or stay off).
    pwm = 0;
  } else if (heatsinkTemp >= FAN_TEMP_MAX) {
    // Above max — full speed.
    pwm = 255;
  } else if (heatsinkTemp < FAN_TEMP_ON) {
    // In the hysteresis band AND fans are already on
    // (else we'd have taken the first branch). Hold at minimum to
    // keep them reliably spinning without dropping below stall PWM.
    pwm = FAN_PWM_MIN;
  } else {
    // Normal linear ramp from FAN_PWM_MIN at FAN_TEMP_ON up to 255 at FAN_TEMP_MAX.
    float fraction = (heatsinkTemp - FAN_TEMP_ON) / (FAN_TEMP_MAX - FAN_TEMP_ON);
    pwm = (uint8_t)(FAN_PWM_MIN + fraction * (255 - FAN_PWM_MIN));
  }

  if (pwm != fanPWM) {
    fanPWM = pwm;
    setFanPWM(pwm);
  }
}


// ================= TIMING =================

#define INIT_GRACE_MS      1500UL
#define SENSOR_INTERVAL_MS   50UL
#define REPORT_INTERVAL_MS  100UL

// ================= UART RANGE TABLE =================
// Inbound from main MCU — all types that secondary expects to receive

Range secondaryRanges[] = {
  {TYPE_FAULT_REASON, 0,  99},
  {TYPE_AMP_STATE,    0,  12},   // was 11 — now includes AMP_SSB_VERIFY
  {TYPE_SSB_MODE,     0,   1},   // new — main MCU echoes 0/1 state back
  {TYPE_OUT_POWER,    0, 9999},
  {TYPE_OUT_SWR,      0, 9999},
  {TYPE_IN_POWER,     0, 9999},
  {TYPE_IN_SWR,       0, 9999},
  {TYPE_AMP_CURRENT,  0, 25000},
  {TYPE_GAIN,         0, 9999},
  {TYPE_EFFICIENCY,   0,  100},
  {TYPE_BAND,         0,    5}
};

// ================= DISPLAY RECEIVED VALUES =================
// Stored here as received from main MCU, decoded for display

static uint8_t  dispAmpState   = 0;
static uint16_t dispOutPower   = 0;    // W×10
static uint16_t dispOutSWR     = 100;  // SWR×100 (1.00 default)
static uint16_t dispInPower    = 0;
static uint16_t dispInSWR      = 100;
static uint16_t dispAmpCurrent = 0;    // mA
static uint16_t dispGain       = 500;  // dB×10 offset +500 (0dB default)
static uint8_t  dispEfficiency = 0;
static uint8_t  dispBand       = 0;
static uint8_t  dispFaultReason = FAULT_NONE;

// ================= SENSOR VALUES =================

int tempHeatsink = 0;
int tempInner    = 0;
int tempAir      = 0;

float volt5V_board1  = 0.0f;
float volt12V_board1 = 0.0f;
float volt5V_board2  = 0.0f;
float volt12V_board2 = 0.0f;
float volt5V_board3  = 0.0f;
float volt12V_board3 = 0.0f;
float volt18V        = 0.0f;
float volt50V        = 0.0f;
float curr18V        = 0.0f;
float curr50V        = 0.0f;

bool voltageFault = false;

// Grace period flag — voltage faults suppressed until supply rails have settled
static bool          gracePeriodDone = false;
static unsigned long initStartTime   = 0;

// ================= NEXTION HMI =================

NexProgressBar p1j0 = NexProgressBar(1,  9, "j0");  // output power bar 0–550W
NexProgressBar p1j1 = NexProgressBar(1, 10, "j1");  // output SWR bar 1.00–3.00

NexButton      p1b0 = NexButton(1, 11, "b0");       // RESET SOFT FAULT
NexButton      p1b1 = NexButton(1, 27, "b1");       // SSB MODE toggle button


NexText        p1t0  = NexText(1, 12, "t0");         // fault description string
NexText        p1t9  = NexText(1, 21, "t9");         // amp state string
NexText        p1t14 = NexText(1, 26, "t14");        // band string
NexText        p1t15 = NexText(1,  28, "t15");       // SSB mode status indicator

NexNumber      p1n0 = NexNumber(1, 1, "n0");         // output forward power (W)
NexNumber      p1n1 = NexNumber(1, 2, "n1");         // output SWR ×100
NexNumber      p1n2 = NexNumber(1, 3, "n2");         // input forward power (W)
NexNumber      p1n3 = NexNumber(1, 4, "n3");         // input SWR ×100
NexNumber      p1n4 = NexNumber(1, 5, "n4");         // amp supply current (mA)
NexNumber      p1n5 = NexNumber(1, 6, "n5");         // heatsink temperature (integer °C)
NexNumber      p1n6 = NexNumber(1, 7, "n6");         // gain (dB, decoded)
NexNumber      p1n7 = NexNumber(1, 8, "n7");         // power loss (100 - efficiency %)

NexTouch *nex_listen_list[] = {
  &p1b0,
  &p1b1,   
  NULL
};

static bool hmiResetButton = false;
static bool dispSSBMode  = false;
static bool hmiSSBButton = false;

void p1b0PopCallback(void *ptr) {
  hmiResetButton = true;
}

void p1b1PopCallback(void *ptr) {
  hmiSSBButton = true;
}

// ================= AMP STATE NAME LOOKUP =================

const char* getStateString(uint8_t state) {
  switch (state) {
    case  0: return "IZKLOPLJEN";
    case  1: return "INIT";
    case  2: return "MIROVANJE";
    case  3: return "ZAGON";
    case  4: return "NASTAV. VHODA";
    case  5: return "NIZKA ODDAJA";
    case  6: return "UGLASITEV";
    case  7: return "PREHOD";
    case  8: return "ODDAJANJE";
    case  9: return "PRIPRAVLJEN";
    case 10: return "SSB VER";
    case 11: return "MEHKA NAPAKA";
    case 12: return "NAPAKA";
    default: return "NEZNANO";
  }
}
/*
    case  0: return "IZKLOPLJEN";
    case  1: return "INIT";
    case  2: return "MIROVANJE";
    case  3: return "ZAGON";
    case  4: return "NASTAV. VHODA";
    case  5: return "NIZKA ODDAJA";
    case  6: return "UGLASITEV";
    case  7: return "PREHOD";
    case  8: return "ODDAJANJE";
    case  9: return "PRIPRAVLJEN";
    case 10: return "SSB VER";
    case 11: return "MEHKA NAPAKA";
    case 12: return "NAPAKA";
    default: return "NEZNANO";
*/
/*
    case  0: return "OFF";
    case  1: return "INIT";
    case  2: return "IDLE";
    case  3: return "STARTUP";
    case  4: return "ADJUST INPUT";
    case  5: return "LOW TRANSMIT";
    case  6: return "TUNE";
    case  7: return "TRANSITION";
    case  8: return "TRANSMIT";
    case  9: return "READY";
    case 10: return "SSB VERIFY";   
    case 11: return "SOFT FAULT";   
    case 12: return "FAULT";        
    default: return "UNKNOWN";
*/



// ================= FAULT REASON NAME LOOKUP =================

const char* getFaultString(uint8_t code) {
  switch (code) {
    case  0: return "BREZ NAPAKE";
    case  1: return "NAPAKA OJ.";
    case  2: return "RF NEVELJAVEN";
    case  3: return "BAND NEVELJAVEN";
    case  4: return "BAND NESTABILEN";
    case  5: return "NASTAV. BAND";
    case  6: return "SPREMEMBA BAND";
    case  7: return "UART KOMM";
    case  8: return "ADC HRDWR";
    case  9: return "SENZOR TOKA";
    case 10: return "ZAGON BIAS";
    case 20: return "TUNE. NESTAB.";
    case 21: return "VHOD PREVISOK";
    case 22: return "ATT PREVERJANJE";
    case 23: return "ATT BREZ ZMANJ.";
    case 24: return "RF IZGUBLJEN";
    case 30: return "VHOD PREOBREMEN";
    case 33: return "VHOD VISOK CAKAJ";
    case 40: return "VISOK SWR";
    case 41: return "NIZKO OJACENJE";
    case 42: return "NIZ. IZKORISTEK";
    case 43: return "MIROVALNI TOK";
    case 44: return "SR LATCH";
    case 45: return "VISOK SWR2";
    case 50: return "PREVISOKA TEMP.";
    case 51: return "KRITICNA TEMP.";
    case 60: return "NIZKA NAPETOST";
    case 61: return "VISOK TOK";
    case 62: return "GATE DRIFT";
    case 63: return "50V NI PRISOTEN";
    case 64: return "50V OSTAJA";
    case 65: return "VHODNI SWR VIS.";
    case 98: return "POKVARJ. STANJE";
    case 99: return "NUJNI IZKLOP";
    default: return "NEZNANA NAPAKA";
  }
}
/*
    case  0: return "BREZ NAPAKE";
    case  1: return "NAPAKA OJ.";
    case  2: return "RF NEVELJAVEN";
    case  3: return "BAND NEVELJAVEN";
    case  4: return "BAND NESTABILEN";
    case  5: return "NASTAV. BAND";
    case  6: return "SPREMEMBA BAND";
    case  7: return "UART KOMM";
    case  8: return "ADC HRDWR";
    case  9: return "SENZOR TOKA";
    case 10: return "ZAGON BIAS";
    case 20: return "TUNE. NESTAB.";
    case 21: return "VHOD PREVISOK";
    case 22: return "ATT PREVERJANJE";
    case 23: return "ATT BREZ ZMANJ.";
    case 24: return "RF IZGUBLJEN";
    case 30: return "VHOD PREOBREMEN";
    case 33: return "VHOD VISOK CAKAJ";
    case 40: return "VISOK SWR";
    case 41: return "NIZKO OJACENJE";
    case 42: return "NIZ. IZKORISTEK";
    case 43: return "MIROVALNI TOK";
    case 44: return "SR LATCH";
    case 45: return "VISOKO ODBITO";
    case 50: return "PREVISOKA TEMP.";
    case 51: return "KRITICNA TEMP.";
    case 60: return "NIZKA NAPETOST";
    case 61: return "VISOK TOK";
    case 62: return "GATE DRIFT";
    case 63: return "50V NI PRISOTNO";
    case 64: return "50V OSTAJA";
    case 65: return "VHODNI SWR VIS.";
    case 98: return "POKVARJ. STANJE";
    case 99: return "NUJNI IZKLOP";
    default: return "NEZNANA NAPAKA";
*/
/*
    case  0: return "NO FAULT";
    case  1: return "AMP FAULT";
    case  2: return "RF INVALID";
    case  3: return "BAND INVALID";
    case  4: return "BAND UNSTABLE";
    case  5: return "BAND SET FAIL";
    case  6: return "BAND CHG LIVE";
    case  7: return "UART COMM";         
    case  8: return "ADC HW";             
    case  9: return "CURRENT SENSOR";
    case 10: return "BIAS STARTUP";
    case 20: return "TUNE UNSTABLE";
    case 21: return "INPUT TOO HIGH";
    case 22: return "ATT VERIFY";
    case 23: return "ATT NO REDUCE";
    case 24: return "RF LOST TUNE";
    case 30: return "INPUT OVERLOAD";
    case 33: return "INPUT HIGH WAIT";
    case 40: return "HIGH SWR";
    case 41: return "LOW GAIN";
    case 42: return "LOW EFFICIENCY";
    case 43: return "LEAKAGE CURRENT";
    case 44: return "HW OVERCURRENT";
    case 45: return "HIGH REFLECTED";    
    case 50: return "OVERTEMPERATURE";
    case 51: return "TEMP CRITICAL";
    case 60: return "LOW VOLTAGE";        
    case 61: return "HIGH CURRENT";       
    case 62: return "GATE DRIFT";         
    case 63: return "50V NOT PRESENT";
    case 64: return "50V SUPPLY STUCK";  
    case 65: return "INPUT SWR HIGH";
    case 98: return "STATE CORRUPT";      
    case 99: return "EMERGENCY STOP";
    default: return "UNKNOWN FAULT";
*/
// ================= BAND NAME LOOKUP =================

const char* getBandString(uint8_t band) {
  switch (band) {
    case 1: return "80m";
    case 2: return "40m";
    case 3: return "30/20m";
    case 4: return "15m";
    case 5: return "10m";
    default: return "---";
  }
}

// ================= NEXTION UPDATE =================

static bool nexDirty = false;

// Rate-limited Nextion update — sends ONE command per call to avoid
// blocking the UART loop. The Nextion library's setText/setValue blocks
// waiting for ACK (~15ms each at 9600 baud). Sending all 10+ commands
// at once would block for 150+ms, overflowing the Serial1 RX buffer
// and causing UART comm faults.
//
// Rotates through all display elements. When nexDirty is set, one full
// rotation pushes all values to the display over ~13 loop iterations.

static uint8_t nexSlot = 0;

void updateNextionDisplay() {
  // Only start a NEW rotation when dirty. Once a rotation is in progress
  // (nexSlot > 0), always complete it so all values are pushed.
  if (nexSlot == 0 && !nexDirty) return;

  switch (nexSlot) {
    case 0:  p1n0.setValue(dispOutPower / 10);  break;
    case 1:  p1j0.setValue((uint32_t)((dispOutPower / 10) * 100 / 550));  break; 
    case 2:  p1n1.setValue(dispOutSWR);  break;
    case 3: {
      uint32_t swr_bar = (dispOutSWR > 100) ? (dispOutSWR - 100) : 0;
      if (swr_bar > 200) swr_bar = 200;
      p1j1.setValue(swr_bar / 2);
      break;
    }
    case 4:  p1n2.setValue(dispInPower / 10);  break;
    case 5:  p1n3.setValue(dispInSWR);  break;
    case 6:  p1n4.setValue(dispAmpCurrent);  break;
    case 7:  p1n5.setValue((uint32_t)tempHeatsink);  break;
    case 8:  p1n6.setValue(((int16_t)dispGain - 500) / 10);  break;
    case 9:  p1n7.setValue((dispEfficiency <= 100) ? (100 - dispEfficiency) : 0);  break;
    case 10: p1t9.setText(getStateString(dispAmpState));  break;
    case 11: p1t0.setText(getFaultString(dispFaultReason));  break;
    case 12: p1t14.setText(getBandString(dispBand));  break;
    case 13: p1t15.setText(dispSSBMode ? "SBON" : "SBOFF"); break;
  }

  nexSlot++;
  if (nexSlot > 13) {
    nexSlot  = 0;
    nexDirty = false;  // full rotation complete — all values pushed
  }
}

// ================= UART CALLBACK =================

void secondaryUARTHandler(uint8_t type, uint16_t value) {
  //if (UART_latchedFault()) return;

  switch (type) {

    case TYPE_FAULT_REASON:
      if (value != dispFaultReason) {
        dispFaultReason = (uint8_t)value;
        nexDirty        = true;
      }
      break;

    case TYPE_AMP_STATE:
      if (value != dispAmpState) {
        dispAmpState = (uint8_t)value;
        nexDirty     = true;
      }
      break;

    case TYPE_OUT_POWER:
      if (value != dispOutPower) {
        dispOutPower = value;
        nexDirty     = true;
      }
      break;

    case TYPE_OUT_SWR:
      if (value != dispOutSWR) {
        dispOutSWR = value;
        nexDirty   = true;
      }
      break;

    case TYPE_IN_POWER:
      if (value != dispInPower) {
        dispInPower = value;
        nexDirty    = true;
      }
      break;

    case TYPE_IN_SWR:
      if (value != dispInSWR) {
        dispInSWR = value;
        nexDirty  = true;
      }
      break;

    case TYPE_AMP_CURRENT:
      if (value != dispAmpCurrent) {
        dispAmpCurrent = value;
        nexDirty       = true;
      }
      break;

    case TYPE_GAIN:
      if (value != dispGain) {
        dispGain = value;
        nexDirty = true;
      }
      break;

    case TYPE_EFFICIENCY:
      if ((uint8_t)value != dispEfficiency) {
        dispEfficiency = (uint8_t)value;
        nexDirty       = true;
      }
      break;

    case TYPE_BAND:
      if ((uint8_t)value != dispBand) {
        dispBand = (uint8_t)value;
        nexDirty = true;
      }
      break;

    case TYPE_SSB_MODE:
      if ((bool)value != dispSSBMode) {
        dispSSBMode = (bool)value;
        nexDirty    = true;
      }
    break;

    default:
      break;
  }
}

// ================= SENSOR READING =================

void readTemperatures() {
    #ifdef ENABLE_TEMP_HEATSINK
    float rawTemp = analogRead(PinTempHeatsink) * SCALE_LM35;

    if (!tempHeatsinkInitDone) {
      // Seed the filter with the first real reading so startup isn't
      // artificially slewed from 0°C up to ambient over several seconds.
      tempHeatsinkFiltered = rawTemp;
      tempHeatsinkInitDone = true;
    } else if (rawTemp > tempHeatsinkFiltered + TEMP_MAX_RISE_PER_READ) {
      // Raw reading jumped more than the allowed slew rate — clamp the rise.
      tempHeatsinkFiltered += TEMP_MAX_RISE_PER_READ;
    } else {
      // Normal update: either a gradual rise, or any fall (unrestricted).
      tempHeatsinkFiltered = rawTemp;
    }

    int newTemp = (int)(tempHeatsinkFiltered + 0.5f);
    if (newTemp != tempHeatsink) { tempHeatsink = newTemp; nexDirty = true; }
  #endif
  #ifdef ENABLE_TEMP_INNER
    int newInner = (int)(analogRead(PinTempInner) * SCALE_LM35 + 0.5f);
    if (newInner != tempInner) { tempInner = newInner; nexDirty = true; }
  #endif
  #ifdef ENABLE_TEMP_AIR
    int newAir = (int)(analogRead(PinTempAir) * SCALE_LM35 + 0.5f);
    if (newAir != tempAir) { tempAir = newAir; nexDirty = true; }
  #endif
}

void readVoltages() {
  volt5V_board1   = analogRead(VsenseFiveVboardOne)       * SCALE_DIVIDED_RAIL_FIVE;
  volt12V_board1  = analogRead(VsenseTwelveVboardOne)     * SCALE_DIVIDED_RAIL;
  volt5V_board2   = analogRead(VsenseFiveVboardTwo)       * SCALE_DIVIDED_RAIL_FIVE;
  volt12V_board2  = analogRead(VsenseTwelveVboardTwo)     * SCALE_DIVIDED_RAIL;
  volt5V_board3   = analogRead(VsenseFiveVboardThree)     * SCALE_DIVIDED_RAIL_FIVE;
  volt12V_board3  = analogRead(VsenseTwelveVboardThree)   * SCALE_DIVIDED_RAIL;
  volt18V         = analogRead(VsenseEighteenV)            * SCALE_18V;
  volt50V         = analogRead(VsenseFiftyV)               * SCALE_50V;
  curr18V         = analogRead(IsenseEighteenV)            * SCALE_I18V;
  curr50V         = analogRead(IsenseFiftyV)               * SCALE_I50V;
}

bool checkVoltageFaults() {
  // 5V and 12V logic supply rails — always present when system is powered
  if (volt5V_board1  < VOLT_5V_LOW  || volt5V_board1  > VOLT_5V_HIGH)  return true;
  if (volt5V_board2  < VOLT_5V_LOW  || volt5V_board2  > VOLT_5V_HIGH)  return true;
  if (volt5V_board3  < VOLT_5V_LOW  || volt5V_board3  > VOLT_5V_HIGH)  return true;
  if (volt12V_board1 < VOLT_12V_LOW || volt12V_board1 > VOLT_12V_HIGH) return true;
  if (volt12V_board2 < VOLT_12V_LOW || volt12V_board2 > VOLT_12V_HIGH) return true;
  if (volt12V_board3 < VOLT_12V_LOW || volt12V_board3 > VOLT_12V_HIGH) return true;

  // 18V driver supply — always on when system is powered
  if (volt18V < VOLT_18V_LOW || volt18V > VOLT_18V_HIGH) return true;

  // 50V PA supply intentionally excluded — controlled by main MCU via highvsupplyon.
  // When idle the rail is off and reads ~0V, which would always trigger a false fault.
  // The main MCU monitors 50V directly via its own ADC in IVcheck().

  return false;
}

// ================= UART REPORTING =================

void reportSensors() {
#ifdef ENABLE_TEMP_HEATSINK
  UART_send(TYPE_TEMP_HEATSINK, (uint16_t)tempHeatsink);
#endif
#ifdef ENABLE_TEMP_INNER
  UART_send(TYPE_TEMP_INNER,    (uint16_t)tempInner);
#endif
#ifdef ENABLE_TEMP_AIR
  UART_send(TYPE_TEMP_AIR,      (uint16_t)tempAir);
#endif
  UART_send(TYPE_VOLTAGE_FAULT, voltageFault ? 1 : 0);
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);   // debug
  Serial1.begin(115200);  // MCU-to-MCU UART
  Serial2.begin(9600);    // Nextion HMI

// Fan pins — direction only; PWM signal driven by hardware timers.
    pinMode(PinFan1, OUTPUT);
    pinMode(PinFan2, OUTPUT);
    pinMode(PinFan3, OUTPUT);
    pinMode(PinFan4, OUTPUT);
    pinMode(PinFan5, OUTPUT);
    pinMode(PinFan6, OUTPUT);
    fan_pwm_init();   // 20kHz (Timer1/5) and 31kHz (Timer2)

  pinMode(buzzer, OUTPUT); digitalWrite(buzzer, LOW);

  // Fresh bootID every power-up — lets the main MCU detect a restart of
  // this MCU via the syncFault mechanism. Previously this was persisted
  // to EEPROM, which made the ID stable across reboots and defeated the
  // purpose of restart detection.
  uint32_t bootID = analogRead(A1) ^ analogRead(A0) ^ micros();

  // UART
  UART_setCallback(secondaryUARTHandler);
  UART_setConfig(
    0xAA,   // start byte — must match main MCU
    10,     // max errors before comm fault
    10,     // heartbeat interval (ms)
    200,     // heartbeat timeout (ms)
    5000,    // sync grace period (ms)
    100     // boot ID send interval (ms)
  );
  UART_setRangeTable(secondaryRanges, sizeof(secondaryRanges) / sizeof(secondaryRanges[0]));
  UART_init(Serial1, bootID);

  delay(800);  //for nextion boot freezing
  nexInit(); // Nextion
  delay(100);

  p1b0.attachPop(p1b0PopCallback);
  p1b1.attachPop(p1b1PopCallback);

  initStartTime = millis();
}

// ================= LOOP =================

void loop() {
  // --- UART housekeeping ---
  UART_readSerial();
  UART_parse();
  UART_sendHeartbeat();
  UART_checkHeartbeat();
  UART_sendBootID();
  UART_updateFaults();

  // --- Nextion event polling ---
  nexLoop(nex_listen_list);

  // --- Reset button handling ---
  // Temperature gate: silently ignore reset if heatsink is above cooling threshold.
  // The fault display already shows OVERTEMPERATURE — the operator knows why.
  if (hmiResetButton) {
    hmiResetButton = false;

    bool isTempFault = (dispFaultReason == FAULT_TEMP_HIGH ||
                        dispFaultReason == FAULT_TEMP_CRITICAL);

    if (!isTempFault || tempHeatsink < TEMP_HEATSINK_RESET) {
      UART_send(TYPE_RESET, 0);
    }
  }
  
  if (hmiSSBButton) {
    hmiSSBButton = false;
    UART_send(TYPE_SSB_MODE, 0);   // value ignored — main MCU owns toggle state
  }

  // --- Timed sensor read ---
  static unsigned long lastSensorRead = 0;
  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readTemperatures();
    readVoltages();
    updateFans(tempHeatsink);
  }

  // --- Grace period / voltage fault ---
  // Suppress voltage faults for the first INIT_GRACE_MS after boot
  // (supply rails may be low during capacitor inrush at startup)
  if (!gracePeriodDone && (now - initStartTime >= INIT_GRACE_MS)) {
    gracePeriodDone = true;
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.print("page page1");
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    delay(5);
  }
  voltageFault = gracePeriodDone && checkVoltageFaults();

  // --- Timed UART report ---
  static unsigned long lastReport = 0;
  if (now - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = now;
    reportSensors();
  }

  // --- Nextion display update ---
  // Only writes to display when something has changed (nexDirty flag)
  if(gracePeriodDone) updateNextionDisplay();

  // --- Debug output ---
#ifdef DEBUG_SERIAL
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 500) {
    lastDebug = now;

    Serial.print("Grace done: "); Serial.println(gracePeriodDone ? "YES" : "no");

    Serial.print("Heatsink: "); Serial.print(tempHeatsink);
    Serial.print("C | Inner: "); Serial.print(tempInner);
    Serial.print("C | Air: ");   Serial.print(tempAir); Serial.println("C");

    Serial.print("Fan PWM: "); Serial.print(fanPWM);
    Serial.print(" ("); Serial.print((fanPWM * 100) / 255); Serial.println("%)");

    Serial.print("5V  B1: ");  Serial.print(volt5V_board1,  2);
    Serial.print("  B2: ");    Serial.print(volt5V_board2,  2);
    Serial.print("  B3: ");    Serial.println(volt5V_board3, 2);

    Serial.print("12V B1: ");  Serial.print(volt12V_board1, 2);
    Serial.print("  B2: ");    Serial.print(volt12V_board2, 2);
    Serial.print("  B3: ");    Serial.println(volt12V_board3, 2);

    Serial.print("18V: ");     Serial.print(volt18V, 2);
    Serial.print("V  50V: ");  Serial.print(volt50V, 2); Serial.println("V");

    Serial.print("I18V: ");    Serial.print(curr18V, 2);
    Serial.print("A  I50V: "); Serial.print(curr50V, 2); Serial.println("A");

    Serial.print("VoltageFault: "); Serial.println(voltageFault ? "YES" : "no");

    Serial.print("AmpState: ");     Serial.print(dispAmpState);
    Serial.print(" | FaultReason: "); Serial.println(dispFaultReason);

    Serial.print("OutPwr: ");  Serial.print(dispOutPower  / 10);
    Serial.print("W  OutSWR: "); Serial.print(dispOutSWR, DEC);
    Serial.print("  InPwr: ");   Serial.print(dispInPower / 10);
    Serial.print("W  InSWR: "); Serial.println(dispInSWR, DEC);

    Serial.print("Gain: ");    Serial.print(((int16_t)dispGain - 500) / 10);
    Serial.print("dB  Eff: "); Serial.print(dispEfficiency);
    Serial.print("%  Loss: "); Serial.print(100 - dispEfficiency);
    Serial.print("%  Iamp: "); Serial.print(dispAmpCurrent); Serial.println("mA");

    Serial.print("Band: ");    Serial.println(dispBand);

    Serial.print("UART Errors: "); Serial.print(UART_errorCount());
    Serial.print("  CommFault: "); Serial.print(UART_commFault());
    Serial.print("  HBFault: ");   Serial.println(UART_heartbeatFault());

    Serial.print("SSB Mode: "); Serial.println(dispSSBMode ? "ON" : "off");

    Serial.println("---");
  }
#endif
}
