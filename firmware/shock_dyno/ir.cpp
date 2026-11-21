// MLX90614 (GY-906) IR thermometer on I2C. Read once per request from the PC, never on its own,
// so there is no I2C traffic during a measurement except the one reading at the start of each speed.
#include "shared.h"
#include <Wire.h>

#if USE_IR

namespace {

TaskHandle_t irTask = nullptr;
bool wireOk = false;
const uint8_t REG_AMBIENT = 0x06, REG_OBJECT = 0x07;

// One 16-bit RAM register (the sensor also sends a checksum byte, ignored).
bool readWord(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start, as the MLX90614 needs
  if (Wire.requestFrom((int)IR_ADDRESS, 3) != 3) return false;
  value = Wire.read();
  value |= (uint16_t)Wire.read() << 8;
  Wire.read();
  return true;
}

// Register to 0.01 C. False if no answer or the sensor flags an error (bit 15).
bool readTemp(uint8_t reg, int16_t& centi) {
  uint16_t raw;
  if (!readWord(reg, raw) || (raw & 0x8000)) return false;
  centi = (int16_t)((int32_t)raw * 2 - 27315);            // 0.02 K per count
  return true;
}

void taskMain(void*) {
  wireOk = Wire.begin(PIN_IR_SDA, PIN_IR_SCL, IR_I2C_HZ);
  Wire.setTimeOut(20);                                    // a missing sensor must not block the task
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int16_t obj = 0, body = 0;
    bool ok = wireOk && readTemp(REG_OBJECT, obj) && readTemp(REG_AMBIENT, body);
    sh.irObject = obj; sh.irBody = body; sh.irOk = ok;
    sh.irDone = true;                                     // comm sends it
  }
}

}  // namespace

void irBegin() {
  xTaskCreatePinnedToCore(taskMain, "ir", 3072, nullptr, 2, &irTask, 0);
}

void irRequest() {
  if (irTask) xTaskNotifyGive(irTask);
}

#else

void irBegin() {}
void irRequest() { sh.irOk = false; sh.irDone = true; }  // answer "no sensor" so the PC does not wait

#endif
