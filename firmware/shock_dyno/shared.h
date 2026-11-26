// State shared between the acquisition, communication and motor tasks.
#pragma once
#include <Arduino.h>
#include "config.h"
#include "protocol.h"

struct Sample { uint32_t t_us; int32_t force; int32_t position; };

struct Shared {
  // supervisor
  volatile uint8_t  state = proto::ST_IDLE;
  volatile uint8_t  fault = proto::FAULT_NONE;
  volatile bool     runRequested = false;
  volatile bool     stopPending = true;       // send a stop to the drive at boot
  volatile bool     vfdResetPending = false;
  volatile uint16_t freqCmd = 0;              // 0.01 Hz
  volatile uint32_t lastPcMs = 0;             // last valid packet from the PC
  volatile bool     drivePaused = false;      // PC lost: no Modbus traffic, so the drive's comm timeout trips it
  // force limits (raw counts)
  volatile bool     limitsSet = false;
  volatile int32_t  limitLo = 0, limitHi = 0;
  // acquisition
  volatile bool     acqOn = false;
  volatile bool     adcOk = false;
  volatile bool     resyncRequest = false;
  volatile uint32_t lastSampleMs = 0;
  volatile uint16_t dropped = 0;
  // drive
  volatile bool     vfdOk = false;
  volatile uint16_t vfdFreq = 0, vfdFault = 0, vfdStatus = 0;
  // IR thermometer: irDone is set by the ir task, cleared by comm when it sends the reading
  volatile bool     irDone = false, irOk = false;
  volatile int16_t  irObject = 0, irBody = 0;   // 0.01 C
};

extern Shared sh;
extern QueueHandle_t sampleQueue;

inline bool safetyOk() { return digitalRead(PIN_SAFETY) == SAFETY_OK_LEVEL; }

// control.cpp
void controlBegin();
void tripFault(uint8_t reason);                    // safe to call from any task
uint8_t cmdMotorRun(uint16_t freq);
uint8_t cmdMotorSetFreq(uint16_t freq);
uint8_t cmdMotorStop();
uint8_t cmdClearFault();

// acquisition.cpp
void acquisitionBegin();

// comm.cpp
void commBegin();

// ir.cpp
void irBegin();
void irRequest();                                  // read once; the result appears in sh.irDone/irOk/irObject/irBody
