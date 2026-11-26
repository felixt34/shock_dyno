// Packet protocol shared with the PC application (see docs/protocol.md)
//   FF 00 TYPE LEN PAYLOAD[LEN] CRC8      CRC8 (poly 0x07) over TYPE, LEN, PAYLOAD
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace proto {

const uint8_t HDR0 = 0xFF, HDR1 = 0x00;
const uint8_t TYPE_EVENT = 0x55, TYPE_DATA = 0xAA;
const size_t  MAX_PAYLOAD = 255;
const size_t  POINT_SIZE = 10;          // u32 t_us, i24 force, i24 position (little endian)

enum Event : uint8_t {
  // PC -> ESP32
  EV_HEARTBEAT        = 0x01,
  EV_ACQ_START        = 0x02,
  EV_ACQ_STOP         = 0x03,
  EV_SET_FORCE_LIMITS = 0x04,           // i32 lo, i32 hi (raw ADC counts)
  EV_MOTOR_RUN        = 0x05,           // u16 frequency, 0.01 Hz
  EV_MOTOR_SET_FREQ   = 0x06,           // u16 frequency, 0.01 Hz
  EV_MOTOR_STOP       = 0x07,
  EV_CLEAR_FAULT      = 0x08,
  EV_READ_TEMP        = 0x09,           // read the IR thermometer once; answered with EV_TEMP
  // ESP32 -> PC
  EV_STATUS           = 0x81,           // see StatusPayload
  EV_ACK              = 0x82,           // u8 event id, u8 result
  EV_TEMP             = 0x83,           // u8 ok, i16 object (damper) 0.01 C, i16 sensor body 0.01 C
};

enum State : uint8_t { ST_IDLE = 0, ST_RUNNING = 1, ST_FAULT = 2 };

enum Fault : uint8_t {
  FAULT_NONE = 0, FAULT_HEARTBEAT = 1, FAULT_FORCE_LIMIT = 2, FAULT_SAFETY_OPEN = 3,
  FAULT_VFD_COMM = 4, FAULT_VFD_FAULT = 5, FAULT_ADC_TIMEOUT = 6,
};

enum Result : uint8_t {
  RES_OK = 0, RES_BAD_ARGS = 1, RES_IN_FAULT = 2, RES_SAFETY_OPEN = 3, RES_NO_LIMITS = 4,
  RES_ACQ_OFF = 5, RES_ADC_NOT_READY = 6, RES_NOT_RUNNING = 7, RES_UNKNOWN = 8,
};

const uint8_t FLAG_SAFETY_OK = 0x01, FLAG_ACQ_ON = 0x02, FLAG_LIMITS_SET = 0x04,
              FLAG_VFD_OK = 0x08, FLAG_ADC_OK = 0x10;

// STATUS payload after the event id (little endian): 10 bytes
//   u8 state, u8 fault, u8 flags, u16 vfdFreq(0.01 Hz), u16 vfdFault, u16 dropped, u8 vfdStatus,
//   u16 queued points, u16 serial TX bytes pending (both: how far the data sent lags the measurement)
const size_t STATUS_SIZE = 16;   // + u16 missed DRDY edges   // + u16 samples waiting in the queue, u16 bytes waiting in the serial transmit buffer

uint8_t crc8(const uint8_t* data, size_t len, uint8_t crc = 0);

// Builds a packet into out (needs len + 5 bytes). Returns total size.
size_t encode(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out);

// Byte-wise decoder (finite state machine). feed() returns true when a full valid packet is ready.
class Decoder {
 public:
  bool feed(uint8_t b);
  uint8_t type() const { return type_; }
  uint8_t length() const { return len_; }
  const uint8_t* payload() const { return buf_; }
 private:
  enum { S_HDR0, S_HDR1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC } st_ = S_HDR0;
  uint8_t type_ = 0, len_ = 0, pos_ = 0;
  uint8_t buf_[MAX_PAYLOAD];
};

inline void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
inline void putU32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
inline void putI16(uint8_t* p, int16_t v)  { p[0] = v & 0xFF; p[1] = ((uint16_t)v) >> 8; }
inline void putI24(uint8_t* p, int32_t v)  { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; }
inline uint16_t getU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline int32_t  getI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

}  // namespace proto
