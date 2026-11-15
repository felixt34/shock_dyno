// DynamiX shock dyno - ESP32 firmware (Arduino core for ESP32)
//
// Tasks:
//   acq   (core 1) reads both ADS1256, aligns force/position in time, enforces the force limit
//   comm  (core 0) owns the USB serial link to the PC application
//   motor (core 0) supervisor + Modbus RTU link to the VFD
//
// Pins and tuning are in config.h. The packet protocol is described in docs/protocol.md.
#include "shared.h"

void setup() {
  commBegin();
  controlBegin();
  acquisitionBegin();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
