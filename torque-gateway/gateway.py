import asyncio, socket, struct, json, time, csv, os
import websockets

# ---------- CONFIG ----------
UDP_IP   = "127.0.0.1"
UDP_PORT = 9999

WS_HOST  = "127.0.0.1"   # use "0.0.0.0" only if you really need LAN access
WS_PORT  = 8765

CSV_PATH = "torque_log.csv"
FLUSH_EVERY_N = 200        # less frequent flush -> less blocking
WS_HZ = 30                 # send to browser at 30 Hz (prevents lag)
# ---------------------------

FMT  = "<3sB I Q H i h"
SIZE = struct.calcsize(FMT)

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
        w.writerow(["ts_unix", "t_ns", "t_rel_s", "seq", "version", "slave",
                    "torque_mNm", "torque_Nm", "ratio_pct",
                    "torque_Nm_smooth"])
        f.flush()
    return f, w

# ---- smoothing (simple EMA) ----
class EMA:
    def __init__(self, tau_s=0.2):
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
        a = 1.0 - pow(2.718281828, -dt / self.tau_s)
        self.y = self.y + a * (float(x) - self.y)
        return self.y

ema = EMA(tau_s=0.5)

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
        if len(data) < SIZE:
            continue

        magic, ver, seq, t_ns, slave, torque_mN_m, ratio_tenths = struct.unpack_from(FMT, data, 0)
        if magic != b"TS1":
            continue

        if first_t_ns is None:
            first_t_ns = int(t_ns)

        # monotonic time for chart
        t_rel_s = (int(t_ns) - first_t_ns) * 1e-9

        ts_unix = time.time()
        torque_Nm = float(torque_mN_m) / 1000.0
        ratio_pct = float(ratio_tenths) / 10.0

        torque_smooth = ema.step(torque_Nm, t_rel_s)

        msg = {
            "seq": int(seq),
            "t_ns": int(t_ns),
            "t_rel_s": float(t_rel_s),
            "slave": int(slave),
            "torque_mNm": int(torque_mN_m),
            "torque_Nm": float(torque_Nm),
            "torque_Nm_smooth": float(torque_smooth),
            "ratio_pct": float(ratio_pct),
            "version": int(ver),
            "ts": float(ts_unix),
        }

        # CSV (can be a tiny bit blocking; keep flush rare)
        csv_writer.writerow([ts_unix, int(t_ns), t_rel_s, int(seq), int(ver), int(slave),
                             int(torque_mN_m), torque_Nm, ratio_pct, torque_smooth])
        count += 1
        if (count % FLUSH_EVERY_N) == 0:
            csv_file.flush()

        # update latest (overwrite; never queue)
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

        # send with timeout; drop slow/broken clients to avoid lag buildup
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
        async for _ in ws:
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
            max_queue=1,        # don't buffer tons of outgoing msgs per client
            compression=None
        ):
            print(f"WebSocket server: ws://{WS_HOST}:{WS_PORT}")
            print(f"Listening UDP: {UDP_IP}:{UDP_PORT}")
            print(f"Logging CSV: {os.path.abspath(CSV_PATH)}")
            print(f"WS throttle: {WS_HZ} Hz")

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
