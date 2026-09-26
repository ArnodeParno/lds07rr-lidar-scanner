"""LDS07RR (Roborock) lidar toolbox: directly via a USB-serial adapter, or via the ESP32 firmware.

Directly (USB-serial adapter on the lidar TX):
  py lds.py ports                          # list serial ports
  py lds.py listen --port COM5             # live bytes/s: which wire is transmitting?
  py lds.py sniff --port COM5              # find baud rate + packet format
  py lds.py raw   --port COM5 --baud 115200 --seconds 10   # raw dump to file
  py lds.py view  --port COM5 --baud 115200 --proto neato  # live 360-degree radar

Via the ESP32 (firmware in lds_probe/, lidar TX on GPIO32, motor PWM on GPIO26):
  py lds.py esp   --port COM6 scan         # which wire is transmitting, at what baud?
  py lds.py view  --port COM6 --esp 32 --motor 26 --rpm 300   # radar with motor speed control
  py lds.py view  --host 192.168.x.y       # same over Wi-Fi
"""
import argparse
import collections
import sys
import threading
import time

import serial
import serial.tools.list_ports

BAUDS = [115200, 230400, 256000, 460800, 921600, 128000, 153600, 512000, 1000000, 1500000, 57600, 38400, 19200]

# Known headers of cheap 2D lidars: (name, bytes)
KNOWN_HEADERS = [
    ("neato (LDS02RR/LDS08RR/XV11)", None),  # 0xFA + index 0xA0..0xF9, checked separately
    ("ld06/ld19/ld14 (0x54 0x2C)", b"\x54\x2c"),
    ("delta-2x (0xAA)", b"\xaa\x00"),
    ("rplidar (0xA5 0x5A)", b"\xa5\x5a"),
    ("ydlidar (0xAA 0x55)", b"\xaa\x55"),
    ("camsense (0x55 0xAA)", b"\x55\xaa\x03\x08"),
]


# ---------------------------------------------------------------- helpers
ESP_BAUD = 921600


class TcpPort:
    """The ESP32's lidar data stream over wifi (TCP port 2323), with the same read() as pyserial."""

    def __init__(self, host, port=2323, timeout=0.2):
        import socket
        self._timeout = socket.timeout
        self.s = socket.create_connection((host, port), timeout=5)
        self.s.settimeout(timeout)

    def read(self, n=4096):
        try:
            data = self.s.recv(n)
        except self._timeout:
            return b""
        if not data:  # connection lost (e.g. out of wifi range)
            raise ConnectionError("wifi connection to the ESP32 lost")
        return data

    def reset_input_buffer(self):
        pass

    def close(self):
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def esp_open(port, reset=False):
    """Open the port to the ESP32 without triggering the auto-reset (DTR/RTS low before open):
    that way a running motor keeps running. reset=True only to get out of a tx bridge."""
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, ESP_BAUD, 0.2
    s.dtr = s.rts = False
    s.open()
    if reset:
        return esp_reset(s)
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def esp_reset(s):
    """Hard-reset the ESP32 via RTS (= EN), so every session starts clean (also out of bridge mode)."""
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    time.sleep(1.2)  # boot up; let the boot noise drain
    s.reset_input_buffer()
    return s


def open_port(port, baud, esp=None, tx=None, od=False, motor=None, rpm=300, host=None):
    """Direct UART, or with esp=<gpio> via the ESP32 probe (bridge on that pin, optional tx pin).
    With motor=<gpio> the ESP32 also controls the speed (PWM on that pin) towards `rpm`.
    With host=<ip or lds.local>: via wifi (the ESP32 then measures and controls on its own)."""
    if host:
        return TcpPort(host)
    if esp is None:
        return serial.Serial(port, baud, timeout=0.2)
    s = esp_open(port, reset=tx is not None)  # only a tx bridge needs a reset
    if motor is not None:
        s.write(f"\nlidar {esp} {baud} {motor} {rpm}\n".encode())
    else:
        s.write(f"\nbridge {esp} {baud}{'' if tx is None else f' {tx} {int(od)}'}\n".encode())
    confirm, end = b"", time.time() + 2
    while b"bridge rx=" not in confirm and time.time() < end:
        confirm += s.read(64)
    if b"bridge rx=" not in confirm:
        s.close()
        raise SystemExit(f"ESP32 probe is not responding ({confirm!r}). Firmware flashed? Correct port?")
    return s


def capture(port, baud, seconds, esp=None):
    with open_port(port, baud, esp) as s:
        s.reset_input_buffer()
        buf = bytearray()
        end = time.time() + seconds
        while time.time() < end:
            buf += s.read(4096)
    return bytes(buf)


def neato_hits(data):
    return sum(1 for i in range(len(data) - 22)
               if data[i] == 0xFA and 0xA0 <= data[i + 1] <= 0xF9 and data[i + 22] == 0xFA)


def autocorr(data, lo=6, hi=160):
    """Best period (frame length) + how strong it is (0..1)."""
    if len(data) < hi * 4:
        return None, 0.0
    n = min(len(data) - hi, 20000)
    best, score = None, 0.0
    for lag in range(lo, hi):
        same = sum(1 for i in range(n) if data[i] == data[i + lag])
        r = same / n
        if r > score:
            best, score = lag, r
    return best, score


def hexdump(data, width=32, lines=12):
    for off in range(0, min(len(data), width * lines), width):
        print(f"  {off:05x}  {data[off:off + width].hex(' ')}")


# ---------------------------------------------------------------- commands
def cmd_ports(_):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No COM ports found. Is the FTDI cable plugged in?")
    for p in ports:
        print(f"{p.device:8} {p.description}  [{p.hwid}]")


def cmd_sniff(a):
    results = []
    for baud in ([a.baud] if a.baud else BAUDS):
        data = capture(a.port, baud, a.seconds, a.esp)
        if not data:
            print(f"{baud:>8}: no data (TX wire correct? module powered? motor spinning?)")
            continue
        lag, r = autocorr(data)
        found = []
        nh = neato_hits(data)
        if nh > 5:
            found.append(f"neato x{nh}")
        for name, hdr in KNOWN_HEADERS[1:]:
            c = data.count(hdr)
            if c > 5:
                found.append(f"{name} x{c}")
        rate = len(data) / a.seconds
        print(f"{baud:>8}: {rate:7.0f} B/s  period={lag}  structure={r:.2f}  {', '.join(found)}")
        results.append((r + (0.5 if found else 0), baud, data))

    if not results:
        return
    results.sort(reverse=True)
    _, baud, data = results[0]
    print(f"\nBest candidate: {baud} baud. First bytes:")
    hexdump(data)
    fn = f"dump_{baud}.bin"
    with open(fn, "wb") as f:
        f.write(data)
    print(f"\nSaved to {fn} (send this on for analysis).")


def cmd_listen(a):
    """Show live bytes/s: move the RX wire along the pins until something comes in."""
    print(f"Listening on {a.port} @ {a.baud} (Ctrl+C = stop)")
    with open_port(a.port, a.baud, a.esp) as s:
        s.reset_input_buffer()
        try:
            while True:
                time.sleep(1)
                data = s.read(s.in_waiting)
                bar = "#" * min(60, len(data) // 200)
                print(f"\r{len(data):7d} B/s  {data[:12].hex(' '):36} {bar:60}", end="", flush=True)
        except KeyboardInterrupt:
            print()


def cmd_esp(a):
    """Send a command to the ESP32 probe and show the reply (up to 'ok')."""
    with esp_open(a.port, reset=a.reset) as s:
        # multiple commands in one session (opening resets the ESP32): "set 25 1 ; scan 3000"
        for cmd in " ".join(a.command).split(";"):
            print(f"> {cmd.strip()}")
            s.write((cmd.strip() + "\n").encode())
            end = time.time() + a.timeout
            while time.time() < end:
                ln = s.readline().decode(errors="replace").rstrip()
                if ln and ln.isprintable():  # skip lidar data arriving in between
                    print(ln)
                if ln == "ok" or ln.startswith("?"):
                    break


POKES = [
    ("wake-edge", b"\x00"),
    ("LDS-006 '$'", b"$"),
    ("0x55 sync", b"\x55" * 8),
    ("0xAA", b"\xaa"),
    ("0xFF", b"\xff"),
    ("CR LF", b"\r\n"),
    ("rplidar scan", b"\xa5\x20"),
    ("ydlidar scan", b"\xa5\x60"),
    ("ascii start", b"start\r\n"),
    ("ascii b", b"b"),
]


def cmd_poke(a):
    """Send known start commands to the module RX and listen on the module TX."""
    bauds = [a.baud] if a.baud else [115200, 230400, 256000, 921600, 57600, 9600]
    for baud in bauds:
        for name, payload in POKES:
            with open_port(a.port, baud, a.rx, a.tx, a.od) as s:
                s.reset_input_buffer()
                s.write(payload)
                end, got = time.time() + a.wait, b""
                while time.time() < end:
                    got += s.read(4096)
            flag = "  <<< RESPONSE" if got else ""
            print(f"{baud:>7} {name:14} -> {len(got):5d} bytes {got[:24].hex(' ')}{flag}", flush=True)
            if got:
                fn = f"poke_{baud}_{name.split()[0].strip(chr(39))}.bin"
                with open(fn, "wb") as f:
                    f.write(got)


def cmd_raw(a):
    with open_port(a.port, a.baud, a.esp, motor=a.motor, rpm=a.rpm, host=a.host) as s:
        s.reset_input_buffer()
        data, end = bytearray(), time.time() + a.seconds
        while time.time() < end:
            data += s.read(4096)
    data = bytes(data)
    fn = a.out or f"dump_{a.baud}.bin"
    with open(fn, "wb") as f:
        f.write(data)
    print(f"{len(data)} bytes -> {fn}")
    hexdump(data)


# ---------------------------------------------------------------- decoders
def neato_checksum(pkt):
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (pkt[i] | pkt[i + 1] << 8)
    chk = (chk & 0x7FFF) + (chk >> 15)
    return chk & 0x7FFF


def decode_neato(buf, out):
    """22-byte packets: FA idx speedL speedH [4x dist(14b)+flags, quality] chkL chkH."""
    rpm = None
    while True:
        i = buf.find(0xFA)
        if i < 0:
            buf.clear()
            return rpm
        del buf[:i]
        if len(buf) < 22:
            return rpm
        pkt = bytes(buf[:22])
        if not (0xA0 <= pkt[1] <= 0xF9) or neato_checksum(pkt) != (pkt[20] | pkt[21] << 8):
            del buf[:1]
            continue
        del buf[:22]
        rpm = (pkt[2] | pkt[3] << 8) / 64
        base = (pkt[1] - 0xA0) * 4
        for k in range(4):
            o = 4 + k * 4
            invalid = pkt[o + 1] & 0x80
            dist = pkt[o] | (pkt[o + 1] & 0x3F) << 8
            out[(base + k) % 360] = 0 if invalid else dist


_CRC8 = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c << 1) ^ 0x4D) & 0xFF if _c & 0x80 else (_c << 1) & 0xFF
    _CRC8.append(_c)


def crc8(data):
    c = 0
    for b in data:
        c = _CRC8[(c ^ b) & 0xFF]
    return c


def decode_ld06(buf, out):
    """47-byte packets: 54 2C speed start 12x(dist,int) end ts crc."""
    rpm = None
    while True:
        i = buf.find(b"\x54\x2c")
        if i < 0:
            del buf[:-1]
            return rpm
        del buf[:i]
        if len(buf) < 47:
            return rpm
        pkt = bytes(buf[:47])
        if crc8(pkt[:46]) != pkt[46]:
            del buf[:1]
            continue
        del buf[:47]
        rpm = (pkt[2] | pkt[3] << 8) / 6  # degrees/s -> rpm
        start = (pkt[4] | pkt[5] << 8) / 100
        end = (pkt[42] | pkt[43] << 8) / 100
        span = (end - start) % 360
        for k in range(12):
            o = 6 + k * 3
            dist = pkt[o] | pkt[o + 1] << 8
            ang = int(round(start + span * k / 11)) % 360
            out[ang] = dist


DECODERS = {"neato": decode_neato, "ld06": decode_ld06}


def cmd_view(a):
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FuncAnimation

    class Stamped(list):
        """Distances per degree, with the time of the last measurement (old points fade)."""
        def __init__(self):
            super().__init__([0] * 360)
            self.t = [0.0] * 360

        def __setitem__(self, i, v):
            super().__setitem__(i, v)
            self.t[i] = time.time()

    dist = Stamped()
    state = {"rpm": None, "bytes": 0, "stop": False}
    decode = DECODERS[a.proto]

    def reader():
        with open_port(a.port, a.baud, a.esp, motor=a.motor, rpm=a.rpm, host=a.host) as s:
            buf = bytearray()
            while not state["stop"]:
                chunk = s.read(2048)
                state["bytes"] += len(chunk)
                buf += chunk
                rpm = decode(buf, dist)
                if rpm is not None:
                    state["rpm"] = rpm

    threading.Thread(target=reader, daemon=True).start()

    fig = plt.figure(figsize=(8, 8), facecolor="#0b0f14")
    ax = fig.add_subplot(projection="polar", facecolor="#0b0f14")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.tick_params(colors="#7a8a99")
    ax.grid(color="#1f2a36")
    theta = np.radians(np.arange(360))
    sc = ax.scatter(theta, np.full(360, np.nan), s=14, c=np.zeros(360), cmap="turbo", vmin=0, vmax=a.range)
    ax.set_autoscale_on(False)
    ax.set_ylim(0, a.range)  # after scatter(), otherwise matplotlib rescales the axis back to ±0.04
    ax.set_rlabel_position(22.5)
    ax.yaxis.set_major_formatter(lambda v, _: f"{v / 1000:.1f} m")
    title = ax.set_title("", color="#cfd8e3")

    def update(_):
        r = np.array(dist, dtype=float)
        r[(r == 0) | (time.time() - np.array(dist.t) > 1.0)] = np.nan
        sc.set_offsets(np.c_[theta, r])
        sc.set_array(np.nan_to_num(r))
        rpm = f"{state['rpm']:.0f} rpm" if state["rpm"] else "no valid packets"
        title.set_text(f"LDS07RR  {a.baud} baud  {a.proto}  |  {rpm}  |  {np.count_nonzero(~np.isnan(r))}/360 points")
        return sc, title

    if a.snapshot:  # without a window: save a PNG after a few seconds
        time.sleep(3)
        update(None)
        fig.savefig(a.snapshot, facecolor=fig.get_facecolor())
        state["stop"] = True
        print(f"snapshot -> {a.snapshot}")
        return
    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        state["stop"] = True


# ---------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description="LDS07RR lidar scanner")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports")
    s = sub.add_parser("sniff")
    s.add_argument("--port", required=True)
    s.add_argument("--baud", type=int)
    s.add_argument("--seconds", type=float, default=1.5)
    li = sub.add_parser("listen")
    li.add_argument("--port", required=True)
    li.add_argument("--baud", type=int, default=115200)
    r = sub.add_parser("raw")
    r.add_argument("--port")
    r.add_argument("--baud", type=int, required=True)
    r.add_argument("--seconds", type=float, default=10)
    r.add_argument("--out")
    v = sub.add_parser("view")
    v.add_argument("--port")
    v.add_argument("--baud", type=int, default=115200)
    v.add_argument("--proto", choices=DECODERS, default="neato")
    v.add_argument("--range", type=float, default=4000, help="max distance in mm")
    for sp in (r, v):
        sp.add_argument("--motor", type=int, metavar="GPIO", help="ESP32 controls the motor PWM on this pin")
        sp.add_argument("--rpm", type=int, default=300, help="target speed (default 300)")
        sp.add_argument("--host", help="via wifi: IP address or lds.local of the ESP32")
    v.add_argument("--snapshot", metavar="PNG", help="no window: save an image after 3 s")
    for sp in (s, li, r, v):
        sp.add_argument("--esp", type=int, metavar="GPIO",
                        help="via the ESP32 probe: read the lidar TX on this GPIO")
    pk = sub.add_parser("poke", help="try start commands via the ESP32 probe")
    pk.add_argument("--port", required=True)
    pk.add_argument("--rx", type=int, default=32, help="GPIO the module TX is connected to")
    pk.add_argument("--tx", type=int, default=25, help="GPIO to the module RX")
    pk.add_argument("--od", action="store_true", help="tx open-drain (only pull low)")
    pk.add_argument("--baud", type=int)
    pk.add_argument("--wait", type=float, default=1.0)
    e = sub.add_parser("esp", help="command to the ESP32 probe, e.g. 'scan' or 'set 25 1'")
    e.add_argument("--port", required=True)
    e.add_argument("--timeout", type=float, default=6)
    e.add_argument("--reset", action="store_true", help="reset the ESP32 first (also stops the motor)")
    e.add_argument("command", nargs="+")
    a = p.parse_args()
    {"ports": cmd_ports, "sniff": cmd_sniff, "listen": cmd_listen, "raw": cmd_raw,
     "view": cmd_view, "esp": cmd_esp, "poke": cmd_poke}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
