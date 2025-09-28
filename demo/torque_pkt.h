#pragma once
#include <cstdint>
#pragma pack(push,1)
struct TorquePkt {
  uint8_t  magic[3];          // 'T','S','1'
  uint8_t  version;           // 1
  uint32_t seq;               // LE
  uint64_t t_ns;              // CLOCK_MONOTONIC ns
  uint16_t slave;             // 1..N
  int32_t  torque_mN_m;       // raw from PDO
  int16_t  ratio_tenths_pct;  // 123 = 12.3%
  uint8_t  reserved[8];       // zero
};
#pragma pack(pop)
static_assert(sizeof(TorquePkt)==32, "TorquePkt must be 32 bytes");
