// Supervisor state machine and motor task. The motor only runs while every
// condition holds; any violation latches a fault and sends a stop to the drive.
// This is an addition to the hard-wired safety circuit, never a replacement for it.
#include "shared.h"
#include "vfd.h"

Shared sh;

namespace {

Vfd vfd;
TaskHandle_t motorTask = nullptr;

int safetyOpenCount = 0;

void supervise() {
  // Debounced: contact bounce or a spike on the cable must not trip a test. The hard-wired
  // safety circuit removes motor power by itself, so this short delay does not affect safety.
  safetyOpenCount = safetyOk() ? 0 : safetyOpenCount + 1;
  if (sh.state != proto::ST_RUNNING) return;
  uint32_t now = millis();
  if (safetyOpenCount >= SAFETY_DEBOUNCE)                     tripFault(proto::FAULT_SAFETY_OPEN);
  else if ((int32_t)(now - sh.lastPcMs) > HEARTBEAT_TIMEOUT_MS) tripFault(proto::FAULT_HEARTBEAT);
  else if (!sh.adcOk || (int32_t)(now - sh.lastSampleMs) > ADC_TIMEOUT_MS) tripFault(proto::FAULT_ADC_TIMEOUT);
  else if (sh.vfdOk && sh.vfdStatus == VFD_STATUS_FAULT)      tripFault(proto::FAULT_VFD_FAULT);
}

void taskMain(void*) {
  bool sentRun = false;
  uint16_t sentFreq = 0xFFFF;
  int fails = 0;
  uint32_t lastPoll = 0;
  auto note = [&](bool ok) {
    if (ok) { fails = 0; sh.vfdOk = true; return; }
    if (++fails >= VFD_COMM_FAIL_LIMIT) {
      sh.vfdOk = false;
      if (sh.state == proto::ST_RUNNING) tripFault(proto::FAULT_VFD_COMM);
    }
  };
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));     // woken early by tripFault() or a command
    supervise();

    bool wantRun = sh.runRequested && sh.state == proto::ST_RUNNING;
    if (!wantRun) {
      if (sentRun || sh.stopPending) {               // retried every pass until the drive confirms
        bool ok = vfd.writeRegister(VFD_REG_CONTROL, VFD_CMD_STOP);
        if (ok) { sentRun = false; sentFreq = 0xFFFF; sh.stopPending = false; }
        note(ok);
      } else if (sh.vfdResetPending) {
        sh.vfdResetPending = false;
        note(vfd.writeRegister(VFD_REG_CONTROL, VFD_CMD_FAULT_RESET));
      }
    } else {
      uint16_t f = sh.freqCmd;
      if (f != sentFreq) {
        bool ok = vfd.writeRegister(VFD_REG_FREQ_SET, f);
        if (ok) sentFreq = f;
        note(ok);
      }
      if (!sentRun && sentFreq == f) {
        bool ok = vfd.writeRegister(VFD_REG_CONTROL, VFD_CMD_RUN_FWD);
        if (ok) sentRun = true;
        note(ok);
      }
    }

    if (millis() - lastPoll >= 200) {
      lastPoll = millis();
      uint16_t v;
      bool ok = vfd.readRegister(VFD_REG_STATUS, v);
      if (ok) sh.vfdStatus = v;
      if (ok && (ok = vfd.readRegister(VFD_REG_FREQ_OUT, v))) sh.vfdFreq = v;
      if (ok && (ok = vfd.readRegister(VFD_REG_FAULT, v))) sh.vfdFault = v;
      note(ok);
    }
  }
}

}  // namespace

void tripFault(uint8_t reason) {
  if (sh.state != proto::ST_FAULT) {
    sh.fault = reason;
    sh.state = proto::ST_FAULT;
  }
  sh.runRequested = false;
  sh.stopPending = true;
  if (motorTask) xTaskNotifyGive(motorTask);
}

uint8_t cmdMotorRun(uint16_t freq) {
  if (sh.state == proto::ST_FAULT) return proto::RES_IN_FAULT;
  if (freq == 0 || freq > VFD_MAX_FREQ_CHZ) return proto::RES_BAD_ARGS;
  if (!safetyOk()) return proto::RES_SAFETY_OPEN;
  if (!sh.limitsSet) return proto::RES_NO_LIMITS;
  if (!sh.acqOn) return proto::RES_ACQ_OFF;
  if (!sh.adcOk) return proto::RES_ADC_NOT_READY;
  sh.freqCmd = freq;
  sh.runRequested = true;
  sh.state = proto::ST_RUNNING;
  xTaskNotifyGive(motorTask);
  return proto::RES_OK;
}

uint8_t cmdMotorSetFreq(uint16_t freq) {
  if (sh.state != proto::ST_RUNNING) return proto::RES_NOT_RUNNING;
  if (freq == 0 || freq > VFD_MAX_FREQ_CHZ) return proto::RES_BAD_ARGS;
  sh.freqCmd = freq;
  xTaskNotifyGive(motorTask);
  return proto::RES_OK;
}

uint8_t cmdMotorStop() {
  sh.runRequested = false;
  sh.stopPending = true;
  if (sh.state == proto::ST_RUNNING) sh.state = proto::ST_IDLE;
  xTaskNotifyGive(motorTask);
  return proto::RES_OK;
}

uint8_t cmdClearFault() {
  if (sh.state == proto::ST_FAULT) {
    sh.fault = proto::FAULT_NONE;
    sh.state = proto::ST_IDLE;
  }
  sh.vfdResetPending = true;
  xTaskNotifyGive(motorTask);
  return proto::RES_OK;
}

void controlBegin() {
  pinMode(PIN_SAFETY, INPUT);
  vfd.begin();
  xTaskCreatePinnedToCore(taskMain, "motor", 4096, nullptr, 6, &motorTask, 0);
}
