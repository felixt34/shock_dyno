// Minimal ADS1256 driver: one differential channel, read on DRDY with RDATA.
#pragma once
#include <Arduino.h>
#include <SPI.h>

class Ads1256 {
 public:
  struct Pins { int8_t sck, miso, mosi, cs, drdy, sync, rst; };

  Ads1256(SPIClass& spi, const Pins& pins) : spi_(spi), p_(pins) {}

  // Sets pin modes with CS / SYNC / RST driven high. Call for every ADC before any SPI traffic.
  void beginPins();
  // Resets and configures the chip, then self-calibrates. False if the chip does not answer.
  bool configure(uint8_t status, uint8_t mux, uint8_t adcon, uint8_t drate);
  // Reads the latest conversion (call after DRDY has gone low). Signed 24-bit result.
  int32_t readData();
  bool waitDrdyLow(uint32_t timeoutMs);
  uint8_t readRegister(uint8_t reg);
  const Pins& pins() const { return p_; }

 private:
  void command(uint8_t cmd);
  void writeRegister(uint8_t reg, uint8_t value);
  SPIClass& spi_;
  Pins p_;
};
