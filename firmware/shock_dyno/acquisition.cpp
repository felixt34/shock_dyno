// Reads both ADS1256 on their DRDY interrupts, timestamps every conversion and
// produces time-aligned (force, position) samples at the force ADC's sample times.
#include "shared.h"
#include "ads1256.h"
#include "soc/gpio_struct.h"
#include "esp_timer.h"

namespace {

SPIClass spi1(VSPI), spi2(HSPI);
Ads1256 adcForce(spi1, { PIN_ADC1_SCK, PIN_ADC1_MISO, PIN_ADC1_MOSI, PIN_ADC1_CS, PIN_ADC1_DRDY, PIN_ADC1_SYNC, PIN_ADC1_RST });
Ads1256 adcPos(spi2,   { PIN_ADC2_SCK, PIN_ADC2_MISO, PIN_ADC2_MOSI, PIN_ADC2_CS, PIN_ADC2_DRDY, PIN_ADC2_SYNC, PIN_ADC2_RST });

TaskHandle_t acqTask = nullptr;
volatile uint32_t isrForceUs = 0, isrPosUs = 0;
const uint32_t BIT_FORCE = 1, BIT_POS = 2;

void IRAM_ATTR onForceDrdy() {
  isrForceUs = (uint32_t)esp_timer_get_time();
  BaseType_t woken = pdFALSE;
  xTaskNotifyFromISR(acqTask, BIT_FORCE, eSetBits, &woken);
  if (woken) portYIELD_FROM_ISR();
}

void IRAM_ATTR onPosDrdy() {
  isrPosUs = (uint32_t)esp_timer_get_time();
  BaseType_t woken = pdFALSE;
  xTaskNotifyFromISR(acqTask, BIT_POS, eSetBits, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// Restart both converters in the same CPU cycle (both SYNC pins are below GPIO32).
void syncBoth() {
  const uint32_t mask = (1UL << PIN_ADC1_SYNC) | (1UL << PIN_ADC2_SYNC);
  GPIO.out_w1tc = mask;
  delayMicroseconds(5);          // > 4 master-clock periods, far less than 20 DRDY periods
  GPIO.out_w1ts = mask;
}

// --- alignment: position is interpolated at each force timestamp ---
struct Stamp { uint32_t t; int32_t v; };
const int MAX_PENDING = 8;
Stamp pending[MAX_PENDING];
int nPending = 0;
Stamp pos0 = {0, 0}, pos1 = {0, 0};
int nPos = 0;
int overLimit = 0;

void emit(uint32_t t, int32_t force, int32_t position) {
  sh.lastSampleMs = millis();
  if (sh.state == proto::ST_RUNNING && sh.limitsSet) {
    if (force < sh.limitLo || force > sh.limitHi) {
      if (++overLimit >= FORCE_LIMIT_SAMPLES) tripFault(proto::FAULT_FORCE_LIMIT);
    } else {
      overLimit = 0;
    }
  }
  if (!sh.acqOn) return;
  Sample s = { t, force, position };
  if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) sh.dropped = sh.dropped + 1;
}

void resolvePending() {
  if (nPos < 2) return;
  int kept = 0;
  for (int i = 0; i < nPending; i++) {
    int32_t afterNew = (int32_t)(pending[i].t - pos1.t);
    if (afterNew > 0) { pending[kept++] = pending[i]; continue; }   // wait for the next position sample
    int32_t span = (int32_t)(pos1.t - pos0.t);
    int32_t dt = (int32_t)(pending[i].t - pos0.t);
    int32_t p = pos0.v;
    if (span > 0 && dt > 0) p = pos0.v + (int32_t)(((int64_t)(pos1.v - pos0.v) * dt) / span);
    emit(pending[i].t, pending[i].v, p);
  }
  nPending = kept;
}

void onForce(uint32_t t, int32_t v) {
  if (nPending >= MAX_PENDING) {            // position ADC is not keeping up
    for (int i = 1; i < nPending; i++) pending[i - 1] = pending[i];
    nPending--;
    sh.dropped = sh.dropped + 1;
  }
  pending[nPending++] = { t, v };
}

void onPosition(uint32_t t, int32_t v) {
  pos0 = pos1;
  pos1 = { t, v };
  if (nPos < 2) nPos++;
  resolvePending();
}

bool initAdcs() {
  bool ok1 = adcForce.configure(ADC_STATUS, ADC_MUX, ADC1_ADCON, ADC_DRATE);
  bool ok2 = adcPos.configure(ADC_STATUS, ADC_MUX, ADC2_ADCON, ADC_DRATE);
  if (!ok1 || !ok2) return false;
  nPending = 0; nPos = 0; overLimit = 0;
  syncBoth();
  return true;
}

void taskMain(void*) {
  uint32_t lastForceMs = 0, lastPosMs = 0;
  for (;;) {
    if (!sh.adcOk) {
      if (initAdcs()) {
        lastForceMs = lastPosMs = millis();
        xTaskNotifyStateClear(nullptr);
        sh.adcOk = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      continue;
    }
    uint32_t bits = 0;
    xTaskNotifyWait(0, 0xFFFFFFFF, &bits, pdMS_TO_TICKS(ADC_TIMEOUT_MS));
    uint32_t now = millis();
    if (sh.resyncRequest) {
      sh.resyncRequest = false;
      nPending = 0; nPos = 0;
      syncBoth();
      lastForceMs = lastPosMs = now;
      continue;
    }
    if (bits & BIT_FORCE) { uint32_t t = isrForceUs; onForce(t, adcForce.readData()); lastForceMs = now; }
    if (bits & BIT_POS)   { uint32_t t = isrPosUs;   onPosition(t, adcPos.readData()); lastPosMs = now; }
    if ((int32_t)(now - lastForceMs) > ADC_TIMEOUT_MS || (int32_t)(now - lastPosMs) > ADC_TIMEOUT_MS) {
      sh.adcOk = false;                       // one converter stopped answering: re-initialise both
    }
  }
}

}  // namespace

void acquisitionBegin() {
  adcForce.beginPins();
  adcPos.beginPins();
  xTaskCreatePinnedToCore(taskMain, "acq", 4096, nullptr, 20, &acqTask, 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ADC1_DRDY), onForceDrdy, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ADC2_DRDY), onPosDrdy, FALLING);
}
