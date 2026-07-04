// ================= UART.cpp =================
#include "UART.h"

static HardwareSerial* uart = nullptr;

static UART_Callback userCallback = nullptr;

static uint8_t START_BYTE;
static uint8_t MAX_ERRORS;

static uint16_t HEARTBEAT_INTERVAL;
static uint16_t HEARTBEAT_TIMEOUT;
static uint16_t SYNC_GRACE_PERIOD;
static uint16_t BOOT_ID_INTERVAL;

static const Range* rangeTable = nullptr;
static uint8_t rangeSize = 0;

#define RX_BUFFER_SIZE 128

static uint8_t rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxHead = 0;
static uint8_t rxTail = 0;

static uint8_t errorCount = 0;

static bool commFault = false;
static bool heartbeatFault = false;
static bool syncFault = false;
static bool latchedFault = false;

static unsigned long lastHeartbeatSent = 0;
static unsigned long lastHeartbeatReceived = 0;
static unsigned long lastBootSend = 0;

static uint32_t bootID;
static uint32_t lastRemoteBootID = 0;

static uint8_t state = 0;
static uint8_t type;
static uint16_t data;
static uint8_t crc;
static uint8_t buffer[3];

// ================= CALLBACK =================
void UART_setCallback(UART_Callback cb) {
  userCallback = cb;
}

// ================= CRC =================
static uint8_t crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

// ================= CONFIG =================
void UART_setConfig(uint8_t startByte,uint8_t maxErrors,uint16_t heartbeatInterval,uint16_t heartbeatTimeout,uint16_t syncGrace,uint16_t bootInterval){
  START_BYTE = startByte;
  MAX_ERRORS = maxErrors;
  HEARTBEAT_INTERVAL = heartbeatInterval;
  HEARTBEAT_TIMEOUT = heartbeatTimeout;
  SYNC_GRACE_PERIOD = syncGrace;
  BOOT_ID_INTERVAL = bootInterval;
}

void UART_setRangeTable(const Range* table, uint8_t size){
  rangeTable = table;
  rangeSize = size;
}

// ================= INIT =================
void UART_init(HardwareSerial &serialPort, uint32_t id){
  uart = &serialPort;
  bootID = id;
  lastHeartbeatReceived = millis();
}

// ================= BUFFER =================
static void rxPush(uint8_t data) {
  uint8_t next = (rxHead + 1) % RX_BUFFER_SIZE;
  if (next != rxTail) {
    rxBuffer[rxHead] = data;
    rxHead = next;
  } else {
    errorCount++;
  }
}

void UART_readSerial() {
  if (!uart) return;
  while (uart->available()) {
    rxPush(uart->read());
  }
}

// ================= VALIDATION =================
static bool validate(uint8_t t, uint16_t value) {
  if (t == TYPE_HEARTBEAT || t == TYPE_BOOT_ID) return true;

  if (!rangeTable) return false;

  for (uint8_t i = 0; i < rangeSize; i++) {
    if (rangeTable[i].type == t) {
      return (value >= rangeTable[i].min && value <= rangeTable[i].max);
    }
  }
  return false;
}

// ================= SEND =================
static void sendPacket(uint8_t t, uint16_t value) {
  if (!uart) return;

  uint8_t packet[5];
  packet[0] = START_BYTE;
  packet[1] = t;
  packet[2] = value >> 8;
  packet[3] = value & 0xFF;
  packet[4] = crc8(&packet[1], 3);

  uart->write(packet, 5);
}

void UART_send(uint8_t type, uint16_t value) {
  sendPacket(type, value);
}

// ================= HEARTBEAT =================
void UART_sendHeartbeat() {
  static uint16_t hbCounter = 0;
  unsigned long now = millis();

  if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
    lastHeartbeatSent = now;
    sendPacket(TYPE_HEARTBEAT, hbCounter++);
  }
}

void UART_checkHeartbeat() {
  
    if (millis() < SYNC_GRACE_PERIOD) {
    lastHeartbeatReceived = millis();
    return;
  }

  heartbeatFault = (millis() - lastHeartbeatReceived > HEARTBEAT_TIMEOUT);
}

void UART_sendBootID() {
  unsigned long now = millis();

  if (now - lastBootSend >= BOOT_ID_INTERVAL) {
    lastBootSend = now;
    sendPacket(TYPE_BOOT_ID, (uint16_t)(bootID & 0xFFFF));
  }
}

// ================= PARSER =================
static void handlePacket(uint8_t t, uint16_t value) {

  if (t == TYPE_HEARTBEAT) {
    lastHeartbeatReceived = millis();
    return;
  }

  if (t == TYPE_BOOT_ID) {
    if (millis() > SYNC_GRACE_PERIOD) {
      if (lastRemoteBootID != 0 && value != lastRemoteBootID) {
        syncFault = true;
      }
    }
    lastRemoteBootID = value;
    return;
  }

  if (!validate(t, value)) {
    errorCount++;
    return;
  }

  // ===== USER CALLBACK =====
  if (userCallback) {
    userCallback(t, value);
  }
}

void UART_parse() {
  while (rxTail != rxHead) {

    uint8_t byte = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

    switch (state) {
      case 0: if (byte == START_BYTE) state = 1; break;
      case 1: type = byte; buffer[0]=byte; state = 2; break;
      case 2: data = byte<<8; buffer[1]=byte; state = 3; break;
      case 3: data |= byte; buffer[2]=byte; state = 4; break;
      case 4:
        crc = byte;
        if (crc8(buffer,3)==crc) handlePacket(type,data);
        else errorCount++;
        state=0;
      break;
    }
  }

  if (errorCount > MAX_ERRORS) commFault = true;
}

// ================= FAULTS =================
void UART_updateFaults() {
  if (heartbeatFault || commFault || syncFault) {
    latchedFault = true;
  }
}

// ================= GETTERS =================
bool UART_commFault(){return commFault;}
bool UART_syncFault(){return syncFault;}
bool UART_heartbeatFault(){return heartbeatFault;}
bool UART_latchedFault(){return latchedFault;}
uint8_t UART_errorCount(){return errorCount;}

/*
Copyright <2026> <KG>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/