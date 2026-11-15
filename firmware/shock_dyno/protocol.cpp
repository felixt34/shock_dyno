#include "protocol.h"
#include <string.h>

namespace proto {

uint8_t crc8(const uint8_t* data, size_t len, uint8_t crc) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

size_t encode(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out) {
  out[0] = HDR0; out[1] = HDR1; out[2] = type; out[3] = len;
  if (len) memcpy(out + 4, payload, len);
  out[4 + len] = crc8(out + 2, (size_t)len + 2);
  return (size_t)len + 5;
}

bool Decoder::feed(uint8_t b) {
  switch (st_) {
    case S_HDR0: if (b == HDR0) st_ = S_HDR1; break;
    case S_HDR1: st_ = (b == HDR1) ? S_TYPE : (b == HDR0 ? S_HDR1 : S_HDR0); break;
    case S_TYPE:
      if (b == TYPE_EVENT || b == TYPE_DATA) { type_ = b; st_ = S_LEN; }
      else st_ = (b == HDR0) ? S_HDR1 : S_HDR0;
      break;
    case S_LEN: len_ = b; pos_ = 0; st_ = len_ ? S_PAYLOAD : S_CRC; break;
    case S_PAYLOAD: buf_[pos_++] = b; if (pos_ >= len_) st_ = S_CRC; break;
    case S_CRC: {
      uint8_t hdr[2] = { type_, len_ };
      uint8_t c = crc8(buf_, len_, crc8(hdr, 2));
      st_ = S_HDR0;
      return c == b;
    }
  }
  return false;
}

}  // namespace proto
