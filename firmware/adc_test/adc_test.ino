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

// Approximate scales for the printout (from the app's sensor calibration; adjust if yours differ)
const double FORCE_N_PER_COUNT = 0.0046;       // load cell, gain 64
const double LVIT_MM_PER_COUNT = 0.000052827;  // LVIT through the 33k/10k divider, gain 1 (about 19,000 counts per mm)

// Crosstalk tests between the two boards (change, upload, compare the p-p columns):
//   READ_...  false: the board keeps converting but is not read (no SPI traffic to it)
//   STOP_...  true : the board is put in power-down after its start-up check (SYNC/PDWN held low:
//                    its clock and converter stop), so it cannot disturb the other one
const bool READ_FORCE = true,  READ_POSITION = true;
const bool STOP_FORCE = false, STOP_POSITION = false;

// SINGLE-BOARD MODE: load cell AND LVIT on ADS #1 (ADS #2 unplugged).
//   load cell  on chip AIN0 (+) / AIN1 (-)  = board terminals AIN1 / AIN2  (unchanged)
//   LVIT pair  on chip AIN2 (+) / AIN3 (-)  = board terminals AIN3 / AIN4
// ADS #1 alternates: force conversion at gain 64, position conversion at gain 1. Auto-calibration
// is off (it would recalibrate at every gain change); one self-calibration at gain 64 at start-up.
const bool    SINGLE_BOARD = true;
const uint8_t SINGLE_DRATE = 0xA1;   // 1000 samples/s per conversion (each one settled after the switch)
const uint8_t MUX_FORCE = 0x01, MUX_POSITION = 0x23, ADCON_FORCE = 0x06, ADCON_POSITION = 0x00;

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

void singleSetup();
void singleLoop();

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
  if (SINGLE_BOARD) { singleSetup(); return; }
  for (Adc& a : adcs) test(a);
  if (STOP_FORCE && adcs[0].ok)    { digitalWrite(adcs[0].sync, LOW); Serial.println("ADS #1 (force) put in power-down"); }
  if (STOP_POSITION && adcs[1].ok) { digitalWrite(adcs[1].sync, LOW); Serial.println("ADS #2 (position) put in power-down"); }
  Serial.printf("\nRESULT: ADS #1 %s, ADS #2 %s\n", adcs[0].ok ? "OK" : "FAIL", adcs[1].ok ? "OK" : "FAIL");
  Serial.println("Noise check: both ADCs read at full speed (500 samples/s); every 0.25 s, the mean and the");
  Serial.println("peak-to-peak spread of each, in counts and (approximately) in N or mm. No USB data stream, no RS-485 traffic.");
  Serial.println("   ADS #1 force: mean counts, p-p counts (~N)   |   ADS #2 LVIT: mean counts (~mm from 0 V), p-p counts (~mm)");
}

uint32_t rateStart = 0, rateCount[2] = { 0, 0 }, rateLines = 0;

// ------------------------------------------------------------------ single-board mode
void singleSetup() {
  adcs[0].status = 0x00;                         // MSB first, auto-calibration OFF, buffer off
  test(adcs[0]);                                 // reset, registers, read-back, self-cal at gain 64 on the load cell
  adcs[1].ok = false;
  if (adcs[0].ok) { waitDrdy(adcs[0], 1000); wreg(adcs[0], 3, SINGLE_DRATE); delay(2); }
  Serial.printf("\nSINGLE-BOARD MODE: ADS #1 %s (ADS #2 not used)\n", adcs[0].ok ? "OK" : "FAIL");
  Serial.println("Alternating force (AIN0/AIN1, gain 64) and position (AIN2/AIN3, gain 1). Every 0.25 s:");
  Serial.println("   force: mean counts, p-p counts (~N)   |   LVIT: mean counts (~mm from 0 V), p-p counts (~mm), pairs");
}

// Selects the input pair and gain, restarts the converter, waits for the settled result.
bool convert(Adc& a, uint8_t mux, uint8_t adcon, int32_t& value) {
  a.spi->beginTransaction(kSpi);
  digitalWrite(a.cs, LOW);
  a.spi->transfer(0x51); a.spi->transfer(0x01);  // WREG from MUX, 2 registers
  a.spi->transfer(mux);  a.spi->transfer(adcon);
  delayMicroseconds(5);
  a.spi->transfer(0xFC);                         // SYNC
  delayMicroseconds(5);
  a.spi->transfer(0x00);                         // WAKEUP: conversion starts
  digitalWrite(a.cs, HIGH);
  a.spi->endTransaction();
  delayMicroseconds(50);                         // DRDY goes high while converting
  if (waitDrdy(a, 10) < 0) return false;
  value = readData(a);
  return true;
}

void singleLoop() {
  Adc& a = adcs[0];
  if (!a.ok) { delay(1000); return; }
  long long sum[2] = { 0, 0 };
  int32_t lo[2] = { INT32_MAX, INT32_MAX }, hi[2] = { INT32_MIN, INT32_MIN };
  int n = 0, fails = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 250) {
    int32_t f, x;
    if (!convert(a, MUX_FORCE, ADCON_FORCE, f) || !convert(a, MUX_POSITION, ADCON_POSITION, x)) { fails++; continue; }
    sum[0] += f; sum[1] += x; n++;
    if (f < lo[0]) { lo[0] = f; }
    if (f > hi[0]) { hi[0] = f; }
    if (x < lo[1]) { lo[1] = x; }
    if (x > hi[1]) { hi[1] = x; }
  }
  if (n == 0) { Serial.printf("no conversions (%d timeouts)\n", fails); return; }
  long mf = (long)(sum[0] / n), mx = (long)(sum[1] / n), pf = (long)(hi[0] - lo[0]), px = (long)(hi[1] - lo[1]);
  Serial.printf(" %9ld  p-p %6ld (~%5.1f N) | %9ld (~%7.2f mm)  p-p %6ld (~%6.3f mm)  %3d pairs (%d/s)%s\n",
                mf, pf, pf * FORCE_N_PER_COUNT, mx, mx * LVIT_MM_PER_COUNT, px, px * LVIT_MM_PER_COUNT, n, n * 4,
                fails ? "  TIMEOUTS" : "");
}

void loop() {
  if (SINGLE_BOARD) { singleLoop(); return; }
  if (rateStart == 0) rateStart = millis();
  long long sum[2] = { 0, 0 };
  int32_t lo[2] = { INT32_MAX, INT32_MAX }, hi[2] = { INT32_MIN, INT32_MIN };
  int n[2] = { 0, 0 };
  uint32_t t0 = millis();
  while (millis() - t0 < 250) {                       // read each board as soon as it has a new sample
    for (int k = 0; k < 2; k++) {
      Adc& a = adcs[k];
      bool read = k == 0 ? (READ_FORCE && !STOP_FORCE) : (READ_POSITION && !STOP_POSITION);
      if (!a.ok || !read || digitalRead(a.drdy) == HIGH) continue;
      int32_t v = readData(a);
      sum[k] += v; n[k]++; rateCount[k]++;
      if (v < lo[k]) lo[k] = v;
      if (v > hi[k]) hi[k] = v;
    }
  }
  for (int k = 0; k < 2; k++) {
    if (!adcs[k].ok || n[k] == 0)
    {
      bool stopped = k == 0 ? STOP_FORCE : STOP_POSITION, read = k == 0 ? READ_FORCE : READ_POSITION;
      Serial.print(!adcs[k].ok ? "        no data              |" : stopped ? "     powered down            |" : !read ? "     running, not read       |" : "        no data              |");
      continue;
    }
    long pp = (long)(hi[k] - lo[k]);
    long mean = (long)(sum[k] / n[k]);
    if (k == 0) Serial.printf(" %9ld  p-p %6ld (~%5.1f N) |", mean, pp, pp * FORCE_N_PER_COUNT);
    else        Serial.printf(" %9ld (~%7.2f mm)  p-p %6ld (~%6.3f mm)  (%3d samples)", mean, mean * LVIT_MM_PER_COUNT, pp, pp * LVIT_MM_PER_COUNT, n[k]);
  }
  Serial.println();
  // every 10 s: the real sample rate of each board read, and their difference
  if (++rateLines >= 40) {
    double secs = (millis() - rateStart) / 1000.0;
    double r0 = rateCount[0] / secs, r1 = rateCount[1] / secs;
    Serial.printf("== sample rates over %.1f s: ADS #1 %.2f /s, ADS #2 %.2f /s", secs, r0, r1);
    if (rateCount[0] > 0 && rateCount[1] > 0) Serial.printf(", difference %.2f /s (beat every %.2f s)", fabs(r0 - r1), fabs(r0 - r1) > 0.01 ? 1 / fabs(r0 - r1) : 0.0);
    Serial.println(" ==");
    rateStart = millis(); rateCount[0] = rateCount[1] = 0; rateLines = 0;
  }
}
