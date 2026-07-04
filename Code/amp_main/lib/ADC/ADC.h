// ================= ADC.h =================
#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

// ===== CHANNEL MAPS =====
// ADC1 channels (output bridge)
#define ADC1_REFLECTED   0   // output reflected power (AD8310)
#define ADC1_FORWARD     1   // output forward power (AD8310)
#define ADC1_AMP_ISENSE  2   // 50V rail current sense (0-20A)

// ADC2 channels (input bridge)
#define ADC2_REFLECTED   1   // input reflected power (AD8310)
#define ADC2_FORWARD     0   // input forward power (AD8310)
#define ADC2_GATE1       2   // MRF300 gate 1 buffered voltage
#define ADC2_GATE2       3   // MRF300 gate 2 buffered voltage

// ===== INIT =====
void ADC_init();

// ===== CALIBRATION =====
// Call when band changes. Selects per-band AD8310 calibration constants
// for all four log-detector channels. Band 0 = unknown, uses fallback.
// Band 1–5 = HF bands (3.5, 7, 14, 24, 29 MHz).
void ADC_setBand(uint8_t band);

// ===== MAIN UPDATE — call every loop() =====
void ADC_update();

// ===== ADC1 — OUTPUT POWER BRIDGE + AMP CURRENT =====
float   ADC1_getPower(uint8_t ch);
float   ADC1_getPeak(uint8_t ch);
int16_t ADC1_getRaw(uint8_t ch);
float   ADC1_getAvg(uint8_t ch, uint8_t samples);

// ===== ADC2 — INPUT POWER BRIDGE + GATE VOLTAGES =====
float   ADC2_getPower(uint8_t ch);
float   ADC2_getPeak(uint8_t ch);
int16_t ADC2_getRaw(uint8_t ch);
float   ADC2_getAvg(uint8_t ch, uint8_t samples);

// ===== CONTROL =====
void ADC_resetPeaks();

// ===== STATUS =====
bool ADC_fault();
void ADC_clearFault();

#endif

/*
Copyright <2026> <KG>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
