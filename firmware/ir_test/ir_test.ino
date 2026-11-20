// MLX90614 (GY-906) IR thermometer test, on its own on any ESP32.
// Upload, open the Serial Monitor at 115200 baud, press EN.
//
// Wiring (GY-906 -> ESP32):
//   VIN -> 3.3 V      GND -> GND      SCL -> GPIO 0      SDA -> GPIO 15
// GPIO 0 and 15 are the pins planned on the dyno board: both may idle high at power-up.
// (GPIO 34-39 cannot be used: they are inputs only, and I2C must be able to pull SDA low.)
#include <Arduino.h>
#include <Wire.h>

const int      PIN_SDA = 15;
const int      PIN_SCL = 0;
const uint32_t I2C_HZ  = 50000;          // the MLX90614 allows up to 100 kHz; 50 kHz leaves margin for long wires
const uint8_t  ADDR    = 0x5A;           // factory address
const uint8_t  REG_AMBIENT = 0x06, REG_OBJECT = 0x07, REG_EMISSIVITY = 0x24;   // RAM, RAM, EEPROM

// Reads one 16-bit register (the sensor also sends a checksum byte, ignored here).
bool readWord(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start, as the MLX90614 needs
  if (Wire.requestFrom((int)ADDR, 3) != 3) return false;
  value = Wire.read();
  value |= (uint16_t)Wire.read() << 8;
  Wire.read();
  return true;
}

// Temperature register to degrees C. False if no answer or the sensor flags an error.
bool readTemp(uint8_t reg, float& c) {
  uint16_t raw;
  if (!readWord(reg, raw) || (raw & 0x8000)) return false;
  c = raw * 0.02f - 273.15f;                            // 0.02 K per count
  return true;
}

void scanBus() {
  Serial.print("I2C devices found:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " none");
  if (n == 0) Serial.println("  -> check VIN (3.3 V), GND, SDA on GPIO 15, SCL on GPIO 0, and that the board has its pull-ups");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\nMLX90614 (GY-906) test");
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  scanBus();
  uint16_t e;
  if (readWord(REG_EMISSIVITY, e))
    Serial.printf("Emissivity setting: %.3f (factory 1.000; shiny metal reads low unless you use matte tape)\n", e / 65535.0);
  else
    Serial.println("No answer at 0x5A");
  Serial.println("Every 0.5 s: object (what the sensor points at) and body (the sensor itself), in C");
}

void loop() {
  float obj, body;
  bool okO = readTemp(REG_OBJECT, obj), okB = readTemp(REG_AMBIENT, body);
  if (okO && okB) Serial.printf("object %7.2f C   body %7.2f C\n", obj, body);
  else Serial.printf("read failed (object %s, body %s)\n", okO ? "ok" : "FAIL", okB ? "ok" : "FAIL");
  delay(500);
}
