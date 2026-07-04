// ================= UART.h =================
#ifndef UART_H
#define UART_H

#include <Arduino.h>

#define TYPE_HEARTBEAT 0x7F
#define TYPE_BOOT_ID   0x70

struct Range {
  uint8_t type;
  uint16_t min;
  uint16_t max;
};

// ===== Callback =====
typedef void (*UART_Callback)(uint8_t type, uint16_t value);

void UART_setCallback(UART_Callback cb);

// ================= USER CONFIG =================
void UART_setConfig(
  uint8_t startByte,
  uint8_t maxErrors,
  uint16_t heartbeatInterval,
  uint16_t heartbeatTimeout,
  uint16_t syncGrace,
  uint16_t bootInterval
);

void UART_setRangeTable(const Range* table, uint8_t size);

// ================= INIT =================
void UART_init(HardwareSerial &serialPort, uint32_t bootID);

// ================= CORE =================
void UART_readSerial();
void UART_parse();
void UART_sendHeartbeat();
void UART_checkHeartbeat();
void UART_sendBootID();
void UART_updateFaults();

// send function (NEW)
void UART_send(uint8_t type, uint16_t value);

// ================= STATUS =================
bool UART_commFault();
bool UART_syncFault();
bool UART_heartbeatFault();
bool UART_latchedFault();

uint8_t UART_errorCount();

#endif

/*
Copyright <2026> <KG>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/