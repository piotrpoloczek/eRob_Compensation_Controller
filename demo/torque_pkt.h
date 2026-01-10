#pragma once
#include <cstdint>

#pragma pack(push,1)

// ----- V1 (TS1, 32 bytes) -----
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
static_assert(sizeof(TorquePkt) == 32, "TorquePkt must be 32 bytes");

// ----- V2 (TS2, 28 bytes) -----
struct TorquePktV2 {
    char     magic[3];            // 'T''S''2'
    uint8_t  ver;                 // 2
    uint32_t seq;                 // LE
    uint64_t t_ns;                // CLOCK_MONOTONIC ns
    uint16_t slave;               // 1..N
    int32_t  torque_raw_mNm;      // raw (mN·m)
    int32_t  torque_smooth_mNm;   // smoothed (mN·m)
    int16_t  ratio_tenths_pct;    // e.g. 161 = 16.1%
};
static_assert(sizeof(TorquePktV2) == 28, "TorquePktV2 must be 28 bytes");

#pragma pack(pop)
