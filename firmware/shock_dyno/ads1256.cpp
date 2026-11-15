#include "ads1256.h"
#include "config.h"

namespace {
const uint8_t CMD_RDATA = 0x01, CMD_SDATAC = 0x0F, CMD_RREG = 0x10, CMD_WREG = 0x50, CMD_SELFCAL = 0xF0;
const uint8_t REG_STATUS = 0x00, REG_MUX = 0x01, REG_ADCON = 0x02, REG_DRATE = 0x03;
const SPISettings kSpi(ADC_SPI_HZ, MSBFIRST, SPI_MODE1);
const uint32_t T6_US = 10;   // >= 50 master-clock periods (6.5 us at 7.68 MHz) between command and data
}

void Ads1256::beginPins() {
  digitalWrite(p_.cs, HIGH);   pinMode(p_.cs, OUTPUT);
  digitalWrite(p_.sync, HIGH); pinMode(p_.sync, OUTPUT);
  digitalWrite(p_.rst, HIGH);  pinMode(p_.rst, OUTPUT);
  pinMode(p_.drdy, INPUT);
  spi_.begin(p_.sck, p_.miso, p_.mosi, -1);
}

bool Ads1256::waitDrdyLow(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(p_.drdy) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;
    delayMicroseconds(50);
  }
  return true;
}

void Ads1256::command(uint8_t cmd) {
  spi_.beginTransaction(kSpi);
  digitalWrite(p_.cs, LOW);
  spi_.transfer(cmd);
  delayMicroseconds(T6_US);
  digitalWrite(p_.cs, HIGH);
  spi_.endTransaction();
}

void Ads1256::writeRegister(uint8_t reg, uint8_t value) {
  spi_.beginTransaction(kSpi);
  digitalWrite(p_.cs, LOW);
  spi_.transfer(CMD_WREG | reg);
  spi_.transfer(0x00);          // one register
  spi_.transfer(value);
  delayMicroseconds(T6_US);
  digitalWrite(p_.cs, HIGH);
  spi_.endTransaction();
}

uint8_t Ads1256::readRegister(uint8_t reg) {
  spi_.beginTransaction(kSpi);
  digitalWrite(p_.cs, LOW);
  spi_.transfer(CMD_RREG | reg);
  spi_.transfer(0x00);
  delayMicroseconds(T6_US);
  uint8_t v = spi_.transfer(0xFF);
  delayMicroseconds(T6_US);
  digitalWrite(p_.cs, HIGH);
  spi_.endTransaction();
  return v;
}

bool Ads1256::configure(uint8_t status, uint8_t mux, uint8_t adcon, uint8_t drate) {
  digitalWrite(p_.rst, LOW);
  delay(2);
  digitalWrite(p_.rst, HIGH);
  delay(5);
  if (!waitDrdyLow(1000)) return false;
  command(CMD_SDATAC);
  delay(1);
  writeRegister(REG_STATUS, status);
  writeRegister(REG_MUX, mux);
  writeRegister(REG_ADCON, adcon);
  writeRegister(REG_DRATE, drate);
  delay(1);
  // Read back: the chip ID is the high nibble of STATUS (3 for an ADS1256).
  if ((readRegister(REG_STATUS) >> 4) != 0x3) return false;
  if (readRegister(REG_DRATE) != drate) return false;
  if ((readRegister(REG_ADCON) & 0x07) != (adcon & 0x07)) return false;
  command(CMD_SELFCAL);
  delay(5);
  return waitDrdyLow(2000);
}

int32_t Ads1256::readData() {
  spi_.beginTransaction(kSpi);
  digitalWrite(p_.cs, LOW);
  spi_.transfer(CMD_RDATA);
  delayMicroseconds(T6_US);
  uint32_t v = (uint32_t)spi_.transfer(0xFF) << 16;
  v |= (uint32_t)spi_.transfer(0xFF) << 8;
  v |= spi_.transfer(0xFF);
  digitalWrite(p_.cs, HIGH);
  spi_.endTransaction();
  if (v & 0x800000) v |= 0xFF000000;   // sign-extend 24 -> 32 bits
  return (int32_t)v;
}
