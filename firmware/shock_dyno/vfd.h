// Minimal Modbus RTU master for the VFD (functions 0x03 and 0x06) over the MAX485 module.
#pragma once
#include <Arduino.h>

class Vfd {
 public:
  void begin();
  bool writeRegister(uint16_t reg, uint16_t value);
  bool readRegister(uint16_t reg, uint16_t& value);
  // True when the last request was answered with a Modbus exception (the drive is
  // there and talking, but rejected that register or value).
  bool lastWasException() const { return exception_; }
 private:
  bool exception_ = false;
  bool transact(const uint8_t* req, size_t reqLen, uint8_t* resp, size_t respLen);
};

uint16_t modbusCrc(const uint8_t* data, size_t len);
