// ADS1256 bring-up test for the DynamiX DAQ board.
// Upload, open the Serial Monitor at 115200 baud, press EN on the ESP32.
// Checks each ADS1256 step by step and prints what it finds, then streams readings.
// Same pins as firmware/shock_dyno/config.h. Re-upload shock_dyno afterwards.
#include <Arduino.h>
#include <SPI.h>

// Input buffer of the force ADC (STATUS register). 0x02 = buffer off (as in the main firmware),
// 0x06 = buffer on. With the buffer on, both inputs must stay below AVDD - 2 V, which clipped
// the load cell signals on these boards.
const uint8_t FORCE_STATUS = 0x02;

struct Adc {
  const char* name;
  SPIClass* spi;
  int sck, miso, mosi, cs, drdy, sync, rst;
  uint8_t adcon;      // PGA setting used by the main firmware
  uint8_t status;     // input buffer on (0x06) or off (0x02)
  bool ok;
};

SPIClass spiA(VSPI), spiB(HSPI);
Adc adcs[2] = {
  { "ADS #1 (force)",    &spiA, 18, 19, 23, 25,  5, 27, 26, 0x06, FORCE_STATUS, false },
  { "ADS #2 (position)", &spiB, 14, 34, 13, 33, 32, 22, 21, 0x00, 0x02, false },
};

const SPISettings kSpi(1000000, MSBFIRST, SPI_MODE1);

void cmd(Adc& a, uint8_t c) {
  a.spi->beginTransaction(kSpi);
  digitalWrite(a.cs, LOW);
  a.spi->transfer(c);
  delayMicroseconds(10);
  digitalWrite(a.cs, HIGH);
  a.spi->endTransaction();
}

uint8_t rreg(Adc& a, uint8_t reg) {
  a.spi->beginTransaction(kSpi);
  digitalWrite(a.cs, LOW);
  a.spi->transfer(0x10 | reg);
  a.spi->transfer(0x00);
  delayMicroseconds(10);
  uint8_t v = a.spi->transfer(0xFF);
  delayMicroseconds(10);
  digitalWrite(a.cs, HIGH);
  a.spi->endTransaction();
  return v;
}

void wreg(Adc& a, uint8_t reg, uint8_t v) {
  a.spi->beginTransaction(kSpi);
  digitalWrite(a.cs, LOW);
  a.spi->transfer(0x50 | reg);
  a.spi->transfer(0x00);
  a.spi->transfer(v);
  delayMicroseconds(10);
  digitalWrite(a.cs, HIGH);
  a.spi->endTransaction();
}

long waitDrdy(Adc& a, uint32_t timeoutMs) {   // ms until DRDY low, -1 on timeout
  uint32_t t0 = millis();
  while (digitalRead(a.drdy) == HIGH) {
    if (millis() - t0 > timeoutMs) return -1;
    delayMicroseconds(50);
  }
  return (long)(millis() - t0);
}

int32_t readData(Adc& a) {
  a.spi->beginTransaction(kSpi);
  digitalWrite(a.cs, LOW);
  a.spi->transfer(0x01);
  delayMicroseconds(10);
  uint32_t v = (uint32_t)a.spi->transfer(0xFF) << 16;
  v |= (uint32_t)a.spi->transfer(0xFF) << 8;
  v |= a.spi->transfer(0xFF);
  digitalWrite(a.cs, HIGH);
  a.spi->endTransaction();
  if (v & 0x800000) v |= 0xFF000000;
  return (int32_t)v;
}

void test(Adc& a) {
  Serial.printf("\n===== %s =====\n", a.name);
  Serial.printf("pins: SCK %d  DOUT(MISO) %d  DIN(MOSI) %d  CS %d  DRDY %d  SYNC %d  RST %d\n",
                a.sck, a.miso, a.mosi, a.cs, a.drdy, a.sync, a.rst);
  a.ok = false;

  Serial.printf("1. DRDY level before reset: %s\n", digitalRead(a.drdy) ? "HIGH" : "LOW");
  digitalWrite(a.rst, LOW); delay(10); digitalWrite(a.rst, HIGH); delay(100);
  long t = waitDrdy(a, 1000);
  if (t < 0) {
    Serial.println("2. DRDY never went LOW after reset  -> FAIL");
    Serial.println("   The chip is not running or DRDY does not reach the ESP32.");
    Serial.println("   Check: 5 V and GND at this ADS board, the ribbon (mirrored?), DRY -> GPIO, RST and SYNC reach the board (both must be HIGH).");
    Serial.printf("   RST pin reads %s, SYNC pin reads %s at the ESP32.\n", digitalRead(a.rst) ? "HIGH" : "LOW", digitalRead(a.sync) ? "HIGH" : "LOW");
    return;
  }
  Serial.printf("2. DRDY went LOW %ld ms after reset  -> OK (chip is converting)\n", t);

  cmd(a, 0x0F);  // SDATAC
  delay(1);
  uint8_t r[5];
  for (int i = 0; i < 5; i++) r[i] = rreg(a, i);
  Serial.printf("3. Registers after reset: STATUS %02X  MUX %02X  ADCON %02X  DRATE %02X  IO %02X\n", r[0], r[1], r[2], r[3], r[4]);
  Serial.println("   Expected:              STATUS 3x  MUX 01  ADCON 20  DRATE F0  IO Ex");
  bool allFF = true, all00 = true;
  for (int i = 0; i < 5; i++) { allFF &= r[i] == 0xFF; all00 &= r[i] == 0x00; }
  if (allFF) { Serial.println("   All FF: DOUT does not reach the ESP32, or CS does not reach the board -> FAIL"); return; }
  if (all00) { Serial.println("   All 00: DOUT stuck LOW, or DIN/SCK do not reach the board -> FAIL"); return; }
  if ((r[0] >> 4) != 3) { Serial.printf("   Chip ID %d instead of 3: wrong chip or bits shifted (SCK/DIN wiring, SPI mode) -> FAIL\n", r[0] >> 4); return; }
  Serial.println("   Chip ID 3 (ADS1256)  -> OK");

  const uint8_t vals[4] = { a.status, 0x01, a.adcon, 0x92 };
  for (int i = 0; i < 4; i++) {
    if (waitDrdy(a, 1000) < 0) { Serial.printf("4. DRDY stuck HIGH before writing register %d -> FAIL\n", i); return; }
    wreg(a, i, vals[i]);
    delay(5);
  }
  waitDrdy(a, 1000);
  uint8_t s = rreg(a, 0), m = rreg(a, 1), c = rreg(a, 2), d = rreg(a, 3);
  Serial.printf("4. Written STATUS %02X MUX 01 ADCON %02X DRATE 92, read back %02X %02X %02X %02X", a.status, a.adcon, s, m, c, d);
  if ((s & 0x0E) != (a.status & 0x0E) || m != 0x01 || (c & 0x07) != (a.adcon & 0x07) || d != 0x92) {
    Serial.println("  -> FAIL (DIN or SCK wiring)");
    return;
  }
  Serial.println("  -> OK");

  waitDrdy(a, 1000);
  cmd(a, 0xF0);  // SELFCAL
  delay(20);
  t = waitDrdy(a, 2000);
  if (t < 0) { Serial.println("5. Self-calibration never finished -> FAIL"); return; }
  Serial.println("5. Self-calibration done -> OK");
  a.ok = true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\nDynamiX ADS1256 test");
  for (Adc& a : adcs) {   // all chip selects / resets / syncs high before any SPI traffic
    pinMode(a.cs, OUTPUT);   digitalWrite(a.cs, HIGH);    // pinMode first (ESP32 core 3.x ignores
    pinMode(a.sync, OUTPUT); digitalWrite(a.sync, HIGH);  // writes to unconfigured pins)
    pinMode(a.rst, OUTPUT);  digitalWrite(a.rst, HIGH);
    pinMode(a.drdy, INPUT_PULLUP);
    a.spi->begin(a.sck, a.miso, a.mosi, -1);
  }
  for (Adc& a : adcs) test(a);
  Serial.printf("\nRESULT: ADS #1 %s, ADS #2 %s\n", adcs[0].ok ? "OK" : "FAIL", adcs[1].ok ? "OK" : "FAIL");
  Serial.println("Streaming raw readings of the boards that passed (counts and volts at the input), 4 per second:");
}

void loop() {
  for (Adc& a : adcs) {
    if (!a.ok) { Serial.print("            -            |"); continue; }
    if (waitDrdy(a, 100) < 0) { Serial.print("   DRDY timeout          |"); continue; }
    int32_t v = readData(a);
    double gain = (a.adcon & 0x07) == 6 ? 64.0 : 1.0;
    double volts = v * (2.0 * 2.5 / gain) / 8388608.0;
    Serial.printf(" %9ld  %+11.7f V |", (long)v, volts);
  }
  Serial.println();
  delay(250);
}
