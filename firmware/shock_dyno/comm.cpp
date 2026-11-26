// PC link: the only task that touches Serial. Parses commands, streams samples and status.
#include "shared.h"

QueueHandle_t sampleQueue = nullptr;

const int TX_BUFFER_SIZE = 4096;

namespace {

proto::Decoder decoder;
uint8_t txBuf[proto::MAX_PAYLOAD + 8];

void sendEvent(const uint8_t* payload, uint8_t len) {
  size_t n = proto::encode(proto::TYPE_EVENT, payload, len, txBuf);
  Serial.write(txBuf, n);
}

void sendAck(uint8_t id, uint8_t result) {
  uint8_t p[3] = { proto::EV_ACK, id, result };
  sendEvent(p, 3);
}

void sendStatus() {
  uint8_t p[1 + proto::STATUS_SIZE];
  p[0] = proto::EV_STATUS;
  p[1] = sh.state;
  p[2] = sh.fault;
  p[3] = (safetyOk() ? proto::FLAG_SAFETY_OK : 0) | (sh.acqOn ? proto::FLAG_ACQ_ON : 0) |
         (sh.limitsSet ? proto::FLAG_LIMITS_SET : 0) | (sh.vfdOk ? proto::FLAG_VFD_OK : 0) |
         (sh.adcOk ? proto::FLAG_ADC_OK : 0);
  proto::putU16(p + 4, sh.vfdFreq);
  proto::putU16(p + 6, sh.vfdFault);
  proto::putU16(p + 8, sh.dropped);
  p[10] = (uint8_t)sh.vfdStatus;
  proto::putU16(p + 11, (uint16_t)uxQueueMessagesWaiting(sampleQueue));
  int txFree = Serial.availableForWrite();
  proto::putU16(p + 13, (uint16_t)(txFree >= 0 && txFree < TX_BUFFER_SIZE ? TX_BUFFER_SIZE - txFree : 0));
  sendEvent(p, sizeof(p));
}

void handleEvent(const uint8_t* p, uint8_t len) {
  if (len < 1) return;
  uint8_t id = p[0];
  const uint8_t* a = p + 1;
  uint8_t n = len - 1;
  switch (id) {
    case proto::EV_HEARTBEAT: return;                         // lastPcMs already refreshed
    case proto::EV_ACQ_START:
      xQueueReset(sampleQueue);
      sh.dropped = 0;
      sh.resyncRequest = true;
      sh.acqOn = true;
      sendAck(id, proto::RES_OK);
      return;
    case proto::EV_ACQ_STOP:
      if (sh.state == proto::ST_RUNNING) cmdMotorStop();      // never run blind
      sh.acqOn = false;
      sendAck(id, proto::RES_OK);
      return;
    case proto::EV_SET_FORCE_LIMITS: {
      if (n < 8) { sendAck(id, proto::RES_BAD_ARGS); return; }
      int32_t lo = proto::getI32(a), hi = proto::getI32(a + 4);
      if (lo >= hi) { sendAck(id, proto::RES_BAD_ARGS); return; }
      sh.limitsSet = false;
      sh.limitLo = lo; sh.limitHi = hi;
      sh.limitsSet = true;
      sendAck(id, proto::RES_OK);
      return;
    }
    case proto::EV_MOTOR_RUN:
      sendAck(id, n < 2 ? (uint8_t)proto::RES_BAD_ARGS : cmdMotorRun(proto::getU16(a)));
      return;
    case proto::EV_MOTOR_SET_FREQ:
      sendAck(id, n < 2 ? (uint8_t)proto::RES_BAD_ARGS : cmdMotorSetFreq(proto::getU16(a)));
      return;
    case proto::EV_MOTOR_STOP:  sendAck(id, cmdMotorStop()); return;
    case proto::EV_CLEAR_FAULT: sendAck(id, cmdClearFault()); return;
    case proto::EV_READ_TEMP:   irRequest(); return;          // answered with EV_TEMP
    default: sendAck(id, proto::RES_UNKNOWN); return;
  }
}

void taskMain(void*) {
  uint8_t data[POINTS_PER_PACKET * proto::POINT_SIZE];
  int nPoints = 0;
  uint32_t lastFlush = millis(), lastStatus = 0;
  for (;;) {
    while (Serial.available()) {
      if (decoder.feed((uint8_t)Serial.read())) {
        sh.lastPcMs = millis();
        if (decoder.type() == proto::TYPE_EVENT) handleEvent(decoder.payload(), decoder.length());
      }
    }
    // Send everything waiting (several packets if the queue has built up), so the queue never grows.
    uint32_t now = millis();
    for (int k = 0; k < 16; k++) {
      Sample s;
      while (nPoints < POINTS_PER_PACKET && xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
        uint8_t* p = data + nPoints * proto::POINT_SIZE;
        proto::putU32(p, s.t_us);
        proto::putI24(p + 4, s.force);
        proto::putI24(p + 7, s.position);
        nPoints++;
      }
      now = millis();
      if (nPoints >= POINTS_PER_PACKET || (nPoints > 0 && now - lastFlush >= PACKET_FLUSH_MS)) {
        size_t n = proto::encode(proto::TYPE_DATA, data, (uint8_t)(nPoints * proto::POINT_SIZE), txBuf);
        Serial.write(txBuf, n);
        nPoints = 0;
        lastFlush = now;
      }
      if (nPoints > 0 || uxQueueMessagesWaiting(sampleQueue) < POINTS_PER_PACKET) break;   // nothing more to send now
    }
    if (sh.irDone) {
      sh.irDone = false;
      uint8_t p[6] = { proto::EV_TEMP, (uint8_t)(sh.irOk ? 1 : 0) };
      proto::putI16(p + 2, sh.irObject);
      proto::putI16(p + 4, sh.irBody);
      sendEvent(p, sizeof(p));
    }
    if (now - lastStatus >= STATUS_PERIOD_MS) {
      lastStatus = now;
      sendStatus();
    }
    vTaskDelay(1);
  }
}

}  // namespace

void commBegin() {
  sampleQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(Sample));
  Serial.setTxBufferSize(TX_BUFFER_SIZE);
  Serial.setRxBufferSize(1024);
  Serial.begin(PC_BAUD);
  xTaskCreatePinnedToCore(taskMain, "comm", 4096, nullptr, 8, nullptr, 0);
}
