import asyncio, socket, struct, json, time, csv, os
import websockets
from websockets.exceptions import ConnectionClosed

# ---------- CONFIG ----------
UDP_IP   = "127.0.0.1"
UDP_PORT = 9999

WS_HOST  = "127.0.0.1"   # use "0.0.0.0" only if you really need LAN access
WS_PORT  = 8765

CSV_PATH = "torque_log.csv"
FLUSH_EVERY_N = 200       # less frequent flush -> less blocking

WS_HZ = 30                # send to browser at 30 Hz (prevents lag)
# ---------------------------

# TS1: magic, ver, seq, t_ns, slave, torque_mN_m, ratio_tenths_pct
FMT1  = "<3sB I Q H i h"
SIZE1 = struct.calcsize(FMT1)

# TS2: magic, ver, seq, t_ns, slave, torque_raw_mNm, torque_smooth_mNm, ratio_tenths_pct
FMT2  = "<3sB I Q H i i h"
SIZE2 = struct.calcsize(FMT2)  # 28

clients = set()

def open_udp():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    s.bind((UDP_IP, UDP_PORT))
    s.setblocking(False)
    return s

def open_csv(path):
    is_new = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if is_new:
        w.writerow([
            "ts_unix","t_ns","t_rel_s","seq","version","slave",
            "torque_raw_mNm","torque_raw_Nm",
            "torque_smooth_mNm","torque_smooth_Nm",
            "ratio_pct"
        ])
        f.flush()
    return f, w

# ---- smoothing (EMA per slave; used only for TS1) ----
class EMA:
    def __init__(self, tau_s=0.5):
        self.tau_s = float(tau_s)
        self.y = None
        self.t_prev = None

    def step(self, x, t_s):
        if self.y is None:
            self.y = float(x)
            self.t_prev = float(t_s)
            return self.y
        dt = max(1e-6, float(t_s) - self.t_prev)
        self.t_prev = float(t_s)
        # classic EMA
        a = 1.0 - pow(2.718281828, -dt / self.tau_s)
        self.y = self.y + a * (float(x) - self.y)
        return self.y

ema_by_slave = {}

latest_msg = None
latest_lock = asyncio.Lock()

async def udp_reader(sock, csv_file, csv_writer):
    """Reads UDP as fast as it comes, logs CSV, updates latest_msg (no backlog)."""
    global latest_msg
    loop = asyncio.get_running_loop()

    first_t_ns = None
    count = 0

    while True:
        data, _ = await loop.sock_recvfrom(sock, 2048)
        if len(data) < 4:
            continue

        magic = data[0:3]

        if magic == b"TS2":
            if len(data) < SIZE2:
                continue
            magic, ver, seq, t_ns, slave, torque_raw_mNm, torque_smooth_mNm, ratio_tenths = struct.unpack_from(FMT2, data, 0)

            if first_t_ns is None:
                first_t_ns = int(t_ns)
            t_rel_s = (int(t_ns) - first_t_ns) * 1e-9

            ts_unix = time.time()
            torque_raw_Nm = float(torque_raw_mNm) / 1000.0
            torque_smooth_Nm = float(torque_smooth_mNm) / 1000.0
            ratio_pct = float(ratio_tenths) / 10.0

            msg = {
                "seq": int(seq),
                "t_ns": int(t_ns),
                "t_rel_s": float(t_rel_s),
                "slave": int(slave),
                "version": int(ver),
                "ts": float(ts_unix),

                # unified names used by the browser
                "torque_mNm": int(torque_raw_mNm),
                "torque_Nm": float(torque_raw_Nm),
                "torque_mNm_smooth": int(torque_smooth_mNm),
                "torque_Nm_smooth": float(torque_smooth_Nm),

                "ratio_pct": float(ratio_pct),
            }

            csv_writer.writerow([
                ts_unix, int(t_ns), t_rel_s, int(seq), int(ver), int(slave),
                int(torque_raw_mNm), torque_raw_Nm,
                int(torque_smooth_mNm), torque_smooth_Nm,
                ratio_pct
            ])

        elif magic == b"TS1":
            if len(data) < SIZE1:
                continue
            magic, ver, seq, t_ns, slave, torque_mN_m, ratio_tenths = struct.unpack_from(FMT1, data, 0)

            if first_t_ns is None:
                first_t_ns = int(t_ns)
            t_rel_s = (int(t_ns) - first_t_ns) * 1e-9

            ts_unix = time.time()
            torque_raw_Nm = float(torque_mN_m) / 1000.0
            ratio_pct = float(ratio_tenths) / 10.0

            ema = ema_by_slave.get(slave)
            if ema is None:
                ema = EMA(tau_s=0.5)
                ema_by_slave[slave] = ema

            torque_smooth_Nm = float(ema.step(torque_raw_Nm, t_rel_s))
            torque_smooth_mNm = int(round(torque_smooth_Nm * 1000.0))

            msg = {
                "seq": int(seq),
                "t_ns": int(t_ns),
                "t_rel_s": float(t_rel_s),
                "slave": int(slave),
                "version": int(ver),
                "ts": float(ts_unix),

                "torque_mNm": int(torque_mN_m),
                "torque_Nm": float(torque_raw_Nm),
                "torque_mNm_smooth": int(torque_smooth_mNm),
                "torque_Nm_smooth": float(torque_smooth_Nm),

                "ratio_pct": float(ratio_pct),
            }

            csv_writer.writerow([
                ts_unix, int(t_ns), t_rel_s, int(seq), int(ver), int(slave),
                int(torque_mN_m), torque_raw_Nm,
                int(torque_smooth_mNm), torque_smooth_Nm,
                ratio_pct
            ])
        else:
            continue

        count += 1
        if (count % FLUSH_EVERY_N) == 0:
            csv_file.flush()

        async with latest_lock:
            latest_msg = msg

async def broadcaster():
    """Sends latest_msg to all WS clients at fixed rate (WS_HZ)."""
    global latest_msg
    period = 1.0 / float(WS_HZ)

    while True:
        await asyncio.sleep(period)

        async with latest_lock:
            msg = latest_msg

        if not msg or not clients:
            continue

        payload = json.dumps(msg)
        dead = []

        for ws in list(clients):
            try:
                await asyncio.wait_for(ws.send(payload), timeout=0.05)
            except Exception:
                dead.append(ws)

        for ws in dead:
            clients.discard(ws)

async def ws_handler(ws):
    clients.add(ws)
    try:
        await ws.send(json.dumps({"hello": "connected"}))
        try:
            async for _ in ws:
                pass
        except ConnectionClosed:
            pass
    finally:
        clients.discard(ws)

async def main():
    udp = open_udp()
    csv_file, csv_writer = open_csv(CSV_PATH)

    try:
        async with websockets.serve(
            ws_handler,
            WS_HOST,
            WS_PORT,
            ping_interval=20,
            ping_timeout=20,
            max_queue=1,
            compression=None
        ):
            print(f"WebSocket server: ws://{WS_HOST}:{WS_PORT}")
            print(f"Listening UDP: {UDP_IP}:{UDP_PORT}")
            print(f"Logging CSV: {os.path.abspath(CSV_PATH)}")
            print(f"WS throttle: {WS_HZ} Hz")
            print(f"TS1 size={SIZE1} bytes | TS2 size={SIZE2} bytes")

            await asyncio.gather(
                udp_reader(udp, csv_file, csv_writer),
                broadcaster(),
            )
    finally:
        try:
            csv_file.flush()
            csv_file.close()
        except Exception:
            pass

if __name__ == "__main__":
    asyncio.run(main())
