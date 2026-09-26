"""Build a map with the LDS07RR: scan-matching SLAM without odometry.

Each revolution becomes a scan; it is fitted onto the map so far with ICP
(= new position + heading) and then drawn into an occupancy grid (5 cm/cell).

Usage:
  py slam.py --port COM6                       # live map via USB, raw data is recorded right away
  py slam.py --host lds.local                  # same via wifi (ESP32 on a battery pack)
  py slam.py --replay walk.bin                 # replay a recording (with window)
  py slam.py --replay walk.bin --fast --out map   # without window, as fast as possible

When the window is closed: <out>.png (map) and <out>.npz (grid + route).
Tips: keep the lidar level, move calmly (walking pace), don't spin quickly on the spot.
"""
import argparse
import math
import threading
import time

import numpy as np
from scipy.spatial import cKDTree

import lds

RES = 0.05            # m per grid cell
SIZE = 40.0           # map is SIZE x SIZE metres, start in the centre
MIN_R, MAX_R = 0.12, 6.0
L_OCC, L_FREE, L_MAX = 0.85, -0.4, 4.0
KEY_DIST, KEY_ANGLE = 0.15, math.radians(8)   # new keyframe after this much movement
MAX_REF_POINTS = 30000


# ---------------------------------------------------------------- scans from the data stream
class ScanAssembler:
    """Neato packets (FA idx ...) -> one scan per revolution as Nx2 (x, y) in metres."""

    def __init__(self):
        self.buf = bytearray()
        self.cur = {}
        self.last_idx = None
        self.rpm = 0.0
        self.first = True   # the first revolution is almost always incomplete

    def feed(self, chunk):
        self.buf += chunk
        scans = []
        b = self.buf
        while True:
            i = b.find(0xFA)
            if i < 0:
                b.clear()
                break
            del b[:i]
            if len(b) < 22:
                break
            p = bytes(b[:22])
            if not (0xA0 <= p[1] <= 0xF9) or lds.neato_checksum(p) != (p[20] | p[21] << 8):
                del b[:1]
                continue
            del b[:22]
            idx = p[1] - 0xA0
            if self.last_idx is not None and idx < self.last_idx and len(self.cur) > 30:
                scan = self._emit()
                if self.first:
                    self.first = False
                else:
                    scans.append(scan)
            self.last_idx = idx
            self.rpm = (p[2] | p[3] << 8) / 64
            for k in range(4):
                o = 4 + k * 4
                if p[o + 1] & 0x80:
                    continue
                self.cur[idx * 4 + k] = (p[o] | (p[o + 1] & 0x3F) << 8) / 1000.0
        return scans

    def _emit(self):
        ang = np.radians(np.fromiter(self.cur.keys(), float))
        r = np.fromiter(self.cur.values(), float)
        self.cur = {}
        keep = (r > MIN_R) & (r < MAX_R)
        ang, r = ang[keep], r[keep]
        # the lidar measures clockwise; minus sign -> standard mathematical (counter-clockwise) frame
        return np.c_[r * np.cos(-ang), r * np.sin(-ang)]


# ---------------------------------------------------------------- geometry
def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def transform(pts, pose):
    x, y, th = pose
    return pts @ rot(th).T + (x, y)


def icp(src, tree, ref, pose, iters=30):
    """Point-to-point ICP with decreasing match distance. Returns (pose, rms, inlier fraction)."""
    x, y, th = pose
    t = np.array([x, y])
    thr = 0.5
    rms, frac = 1e9, 0.0
    for _ in range(iters):
        P = src @ rot(th).T + t
        d, j = tree.query(P, distance_upper_bound=thr)
        m = np.isfinite(d)
        if m.sum() < 25:
            return None, 1e9, 0.0
        # drop the worst 20% of matches (more robust against moving things)
        cut = np.quantile(d[m], 0.8)
        m &= d <= cut
        A, B = P[m], ref[j[m]]
        ma, mb = A.mean(0), B.mean(0)
        H = (A - ma).T @ (B - mb)
        dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        Rd = rot(dth)
        dt = mb - Rd @ ma
        th += dth
        t = Rd @ t + dt
        rms = float(np.sqrt(np.mean(d[m] ** 2)))
        frac = m.sum() / len(src)
        thr = max(0.08, thr * 0.75)
        if abs(dth) < 1e-4 and np.hypot(*dt) < 1e-4:
            break
    return (t[0], t[1], th), rms, frac


# ---------------------------------------------------------------- map
class Mapper:
    def __init__(self):
        n = int(SIZE / RES)
        self.n = n
        self.grid = np.zeros((n, n), np.float32)   # log-odds, [row=y, column=x]
        self.pose = (0.0, 0.0, 0.0)
        self.key_pose = None
        self.ref = np.empty((0, 2))
        self.tree = None
        self.path = [(0.0, 0.0)]
        self.scan_world = np.empty((0, 2))
        self.stats = {"scans": 0, "ok": 0, "lost": 0, "rms": 0.0, "resets": 0}
        self.lost_run = 0
        self.lock = threading.Lock()

    def cell(self, xy):
        return np.floor(xy / RES).astype(int) + self.n // 2

    def add(self, scan):
        self.stats["scans"] += 1
        if len(scan) < 40:
            return
        if self.tree is None:
            if len(scan) >= 100:          # only start with a full scan as reference
                self._integrate(scan, self.pose, key=True)
            return
        pose, rms, frac = icp(scan, self.tree, self.ref, self.pose)
        if pose is None or rms > 0.06 or frac * len(scan) < 60:
            self.stats["lost"] += 1       # scan doesn't fit: skip, don't update position
            self.lost_run += 1
            if self.lost_run >= 15:       # lost too long: re-anchor at the last position
                self.stats["resets"] += 1
                self.lost_run = 0
                self._integrate(scan, self.pose, key=True)
            return
        self.lost_run = 0
        self.stats["ok"] += 1
        self.stats["rms"] = rms
        kx, ky, kth = self.key_pose
        moved = math.hypot(pose[0] - kx, pose[1] - ky) > KEY_DIST or \
            abs((pose[2] - kth + math.pi) % (2 * math.pi) - math.pi) > KEY_ANGLE
        self._integrate(scan, pose, key=moved)

    def _integrate(self, scan, pose, key):
        world = transform(scan, pose)
        with self.lock:
            self.pose = pose
            self.path.append(pose[:2])
            self.scan_world = world
            self._raytrace(np.array(pose[:2]), world)
            if key:
                self.key_pose = pose
                ref = np.vstack([self.ref, world])
                # voxel filter (2.5 cm) so the reference cloud doesn't explode
                _, keep = np.unique(np.floor(ref / (RES / 2)).astype(np.int64), axis=0, return_index=True)
                self.ref = ref[np.sort(keep)][-MAX_REF_POINTS:]
                self.tree = cKDTree(self.ref)

    def _raytrace(self, origin, hits):
        vec = hits - origin
        dist = np.hypot(vec[:, 0], vec[:, 1])
        steps = np.maximum((dist / RES).astype(int) - 1, 0)
        ray = np.repeat(np.arange(len(hits)), steps)
        frac = (np.arange(steps.sum()) - np.repeat(np.cumsum(steps) - steps, steps)) / np.repeat(dist / RES, steps)
        free = origin + vec[ray] * frac[:, None]
        for cells, val in ((self.cell(free), L_FREE), (self.cell(hits), L_OCC)):
            ok = (cells >= 0).all(1) & (cells < self.n).all(1)
            cells = np.unique(cells[ok], axis=0)
            np.add.at(self.grid, (cells[:, 1], cells[:, 0]), val)
        np.clip(self.grid, -L_MAX, L_MAX, out=self.grid)

    def image(self):
        """Greyscale image: white = free, black = obstacle, grey = unknown; cropped."""
        g = self.grid
        known = np.argwhere(g != 0)
        if not len(known):
            return np.full((10, 10), 0.5), (0, 1, 0, 1)
        (r0, c0), (r1, c1) = known.min(0) - 20, known.max(0) + 20
        r0, c0 = max(r0, 0), max(c0, 0)
        img = 1 - 1 / (1 + np.exp(-g[r0:r1, c0:c1]))
        ext = ((c0 - self.n // 2) * RES, (c1 - self.n // 2) * RES,
               (r0 - self.n // 2) * RES, (r1 - self.n // 2) * RES)
        return img, ext

    def save(self, out):
        import matplotlib
        matplotlib.use("Agg", force=False)
        import matplotlib.pyplot as plt
        img, ext = self.image()
        path = np.array(self.path)
        w, h = ext[1] - ext[0], ext[3] - ext[2]
        fig, ax = plt.subplots(figsize=(max(4, min(14, w * 1.2)), max(4, min(14, h * 1.2))), dpi=120)
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=ext, interpolation="nearest")
        ax.plot(path[:, 0], path[:, 1], "-", color="#ff5a36", lw=1.5, label="route")
        ax.plot(*path[0], "o", color="#2ecc71", ms=7, label="start")
        ax.plot(*path[-1], "s", color="#ff5a36", ms=6, label="end")
        ax.set_xlabel("metres")
        ax.set_ylabel("metres")
        ax.set_aspect("equal")
        ax.grid(color="#4a90d9", alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{out}.png")
        plt.close(fig)
        np.savez_compressed(f"{out}.npz", grid=self.grid, res=RES, path=np.array(self.path))
        print(f"map -> {out}.png, {out}.npz  ({img.shape[1] * RES:.1f} x {img.shape[0] * RES:.1f} m)")


# ---------------------------------------------------------------- sources
def live_source(a, sink, stop):
    rec = open(a.record, "wb") if a.record else None
    while not stop.is_set():
        try:
            with lds.open_port(a.port, 115200, esp=a.esp, motor=a.motor, rpm=a.rpm, host=a.host) as s:
                print("connected to the lidar")
                while not stop.is_set():
                    chunk = s.read(4096)
                    if chunk:
                        if rec:
                            rec.write(chunk)
                        sink(chunk)
        except (OSError, ConnectionError) as e:  # wifi briefly gone: keep trying
            print(f"connection lost ({e}), retrying...")
            time.sleep(1)
    if rec:
        rec.close()


def replay_source(a, sink, stop):
    data = open(a.replay, "rb").read()
    step = 1024
    for i in range(0, len(data), step):
        if stop.is_set():
            break
        sink(data[i:i + step])
        if not a.fast:
            time.sleep(step / 10000)   # ~10 kB/s, the speed of the real lidar
    stop.set()


def main():
    p = argparse.ArgumentParser(description="Build a map with the LDS07RR")
    p.add_argument("--port", help="USB (COMx)")
    p.add_argument("--host", help="via wifi: IP address or lds.local of the ESP32")
    p.add_argument("--esp", type=int, default=32)
    p.add_argument("--motor", type=int, default=26)
    p.add_argument("--rpm", type=int, default=300)
    p.add_argument("--record", help="save raw data (default: recording_<time>.bin)")
    p.add_argument("--replay", help="replay a recorded .bin instead of live")
    p.add_argument("--fast", action="store_true", help="replay without window, as fast as possible")
    p.add_argument("--out", default="map", help="file name (without extension) for the map")
    a = p.parse_args()
    if not (a.port or a.host or a.replay):
        p.error("give --port (USB), --host (wifi) or --replay (recording)")
    if (a.port or a.host) and not a.record:
        a.record = time.strftime("recording_%Y%m%d_%H%M%S.bin")

    mapper, asm, stop = Mapper(), ScanAssembler(), threading.Event()

    def sink(chunk):
        for scan in asm.feed(chunk):
            mapper.add(scan)

    src = replay_source if a.replay else live_source
    if a.fast:
        src(a, sink, stop)
        st = mapper.stats
        print(f"scans {st['scans']}, matched {st['ok']}, lost {st['lost']}, resets {st['resets']}, "
              f"final position x={mapper.pose[0]:.2f} y={mapper.pose[1]:.2f} m, "
              f"heading {math.degrees(mapper.pose[2]):.0f} degrees")
        mapper.save(a.out)
        return

    threading.Thread(target=src, args=(a, sink, stop), daemon=True).start()

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    fig, ax = plt.subplots(figsize=(9, 9), facecolor="#0b0f14")
    ax.set_facecolor("#808080")
    ax.tick_params(colors="#7a8a99")
    im = ax.imshow(np.full((10, 10), 0.5), cmap="gray", vmin=0, vmax=1, origin="lower",
                   extent=(0, 1, 0, 1), interpolation="nearest")
    route, = ax.plot([], [], "-", color="#ff5a36", lw=1.5)
    scan_pts = ax.scatter([], [], s=4, color="#27c4ff")
    me, = ax.plot([], [], "o", color="#ff5a36", ms=8)
    title = ax.set_title("", color="#cfd8e3")
    ax.set_aspect("equal")

    def update(_):
        with mapper.lock:
            img, ext = mapper.image()
            path = np.array(mapper.path)
            sw = mapper.scan_world.copy()
            x, y, th = mapper.pose
        im.set_data(img)
        im.set_extent(ext)
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        route.set_data(path[:, 0], path[:, 1])
        scan_pts.set_offsets(sw if len(sw) else np.empty((0, 2)))
        me.set_data([x], [y])
        st = mapper.stats
        title.set_text(f"{asm.rpm:.0f} rpm | scans {st['scans']}  matched {st['ok']}  lost {st['lost']}"
                       f"  resets {st['resets']} | "
                       f"x {x:.2f}  y {y:.2f} m  {math.degrees(th):.0f}°")
        return im, route, scan_pts, me, title

    _anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        time.sleep(0.3)
        mapper.save(a.out)
        if a.record:
            print(f"raw data -> {a.record} (replay: py slam.py --replay {a.record})")


if __name__ == "__main__":
    main()
