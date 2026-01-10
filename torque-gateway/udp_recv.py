import socket, struct, time

FMT = "<3sB I Q H i h"  # magic, ver, seq, t_ns, slave, torque_mN_m, ratio_tenths_pct
SIZE = struct.calcsize(FMT)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 9999))
print("listening 127.0.0.1:9999, pkt_size =", SIZE)

while True:
    data, addr = s.recvfrom(2048)
    if len(data) < SIZE:
        print("short packet", len(data), "bytes from", addr)
        continue

    magic, ver, seq, t_ns, slave, torque_mN_m, ratio_tenths = struct.unpack_from(FMT, data, 0)
    if magic != b"TS1":
        print("bad magic", magic, "len", len(data))
        continue

    print(f"seq={seq} slave={slave} torque={torque_mN_m} mNm ({torque_mN_m/1000:.3f} Nm) ratio={ratio_tenths/10:.1f}%")
