#pragma once
#include "torque_pkt.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <atomic>
#include <cstdint>

struct UdpPub {
    int fd{-1};
    sockaddr_in addr{};
    std::atomic<uint32_t> seq{0};

    bool init(const char* host = "127.0.0.1", uint16_t port = 9999) {
        fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd < 0) return false;

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        ::inet_pton(AF_INET, host, &addr.sin_addr);
        return true;
    }

    static inline uint64_t now_ns() {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    }

    // -------- V1 sender (TS1) --------
    inline void send_sample(uint16_t slave, int32_t torque_mN_m, int16_t ratio_tenths_pct) {
        if (fd < 0) return;
        TorquePkt p{};
        p.magic[0] = 'T'; p.magic[1] = 'S'; p.magic[2] = '1';
        p.version = 1;
        p.seq = ++seq;
        p.t_ns = now_ns();
        p.slave = slave;
        p.torque_mN_m = torque_mN_m;
        p.ratio_tenths_pct = ratio_tenths_pct;
        (void)::sendto(fd, &p, sizeof(p), MSG_DONTWAIT, (sockaddr*)&addr, sizeof(addr));
    }

    // -------- V2 sender (TS2) --------
    inline void send_sample_v2(uint16_t slave,
                               int32_t torque_raw_mNm,
                               int32_t torque_smooth_mNm,
                               int16_t ratio_tenths_pct,
                               uint64_t t_ns = 0) {
        if (fd < 0) return;
        TorquePktV2 p{};
        p.magic[0] = 'T'; p.magic[1] = 'S'; p.magic[2] = '2';
        p.ver = 2;
        p.seq = ++seq;
        p.t_ns = (t_ns != 0) ? t_ns : now_ns();
        p.slave = slave;
        p.torque_raw_mNm = torque_raw_mNm;
        p.torque_smooth_mNm = torque_smooth_mNm;
        p.ratio_tenths_pct = ratio_tenths_pct;
        (void)::sendto(fd, &p, sizeof(p), MSG_DONTWAIT, (sockaddr*)&addr, sizeof(addr));
    }

    void close_fd() { if (fd >= 0) ::close(fd); fd = -1; }
};
