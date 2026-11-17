#include "vfd.h"
#include "config.h"

uint16_t modbusCrc(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void Vfd::begin() {
  pinMode(PIN_RS485_DE, OUTPUT);         // pinMode before digitalWrite (ESP32 core 3.x)
  digitalWrite(PIN_RS485_DE, LOW);       // receive
  Serial2.begin(VFD_BAUD, VFD_SERIAL_CONFIG, PIN_RS485_RX, PIN_RS485_TX);
}

bool Vfd::transact(const uint8_t* req, size_t reqLen, uint8_t* resp, size_t respLen) {
  exception_ = false;
  while (Serial2.available()) Serial2.read();          // drop stale bytes
  digitalWrite(PIN_RS485_DE, HIGH);
  delayMicroseconds(10);
  Serial2.write(req, reqLen);
  Serial2.flush();                                     // wait until the last bit has left
  delayMicroseconds(10);
  digitalWrite(PIN_RS485_DE, LOW);
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < respLen && millis() - t0 < VFD_TIMEOUT_MS) {
    if (Serial2.available()) {
      resp[got++] = (uint8_t)Serial2.read();
      if (got == 5 && resp[0] == req[0] && resp[1] == (req[1] | 0x80)) {   // exception reply
        uint16_t crc = modbusCrc(resp, 3);
        exception_ = resp[3] == (crc & 0xFF) && resp[4] == (crc >> 8);
        break;
      }
    } else {
      vTaskDelay(1);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(5));                        // inter-frame gap (> 3.5 characters)
  if (exception_ || got < respLen) return false;
  uint16_t crc = modbusCrc(resp, respLen - 2);
  return resp[respLen - 2] == (crc & 0xFF) && resp[respLen - 1] == (crc >> 8) && resp[0] == req[0] && resp[1] == req[1];
}

bool Vfd::writeRegister(uint16_t reg, uint16_t value) {
  uint8_t req[8] = { VFD_ADDRESS, 0x06, (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(value >> 8), (uint8_t)value, 0, 0 };
  uint16_t crc = modbusCrc(req, 6);
  req[6] = crc & 0xFF; req[7] = crc >> 8;
  uint8_t resp[8];
  return transact(req, 8, resp, 8) && memcmp(req, resp, 8) == 0;   // the drive echoes a write
}

bool Vfd::readRegister(uint16_t reg, uint16_t& value) {
  uint8_t req[8] = { VFD_ADDRESS, 0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x01, 0, 0 };
  uint16_t crc = modbusCrc(req, 6);
  req[6] = crc & 0xFF; req[7] = crc >> 8;
  uint8_t resp[7];
  if (!transact(req, 8, resp, 7) || resp[2] != 2) return false;
  value = (uint16_t)((resp[3] << 8) | resp[4]);
  return true;
}
