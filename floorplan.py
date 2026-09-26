"""Make a floor plan from a SLAM map (.npz from slam.py).

Steps: find the main direction of the walls and straighten the map, detect walls as straight
segments (horizontal/vertical), clean up the floor, add dimensions.

Usage:
  py floorplan.py map_indoor.npz --out floorplan_indoor
    -> floorplan_indoor.png (with dimensions) and floorplan_indoor.dxf (walls, mm, for CAD)

Dimensions are indicative (lidar ~1-3 cm, plus drift from walking): expect +/- 5-10 cm.
"""
import argparse
import math

import numpy as np
from scipy import ndimage

OCC_T, FREE_T = 1.0, -1.0       # log-odds thresholds: certain obstacle / certain free
MIN_WALL = 0.40                 # shortest wall segment (m)
GAP = 5                         # bridge gaps of up to this many cells in a wall
MERGE = 4                       # parallel pieces within this many cells = the same wall
SNAP = 0.25                     # join wall ends within this many metres (corners)


def dominant_angle(pts):
    """Rotation angle (0..90 degrees) at which the walls align most sharply with rows/columns."""
    best, best_a = -1, 0.0
    # grid cells already lie at 0 degrees: without jitter 0 degrees always wins artificially
    pts = pts + np.random.default_rng(0).uniform(-0.025, 0.025, pts.shape)
    for a in np.arange(0, 90, 0.5):
        r = np.radians(a)
        c, s = math.cos(r), math.sin(r)
        x = pts[:, 0] * c + pts[:, 1] * s
        y = -pts[:, 0] * s + pts[:, 1] * c
        score = sum((np.bincount(np.floor((v - v.min()) / 0.05).astype(int)) ** 2).sum() for v in (x, y))
        if score > best:
            best, best_a = score, a
    return best_a


def raster(pts, res, origin, shape):
    g = np.zeros(shape, bool)
    ij = np.floor((pts - origin) / res).astype(int)
    ok = (ij >= 0).all(1) & (ij[:, 0] < shape[1]) & (ij[:, 1] < shape[0])
    g[ij[ok, 1], ij[ok, 0]] = True
    return g


def runs(line):
    """Contiguous runs of True (with small gaps bridged) -> [(start, end)]."""
    idx = np.flatnonzero(line)
    if not len(idx):
        return []
    out, s, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > GAP + 1:
            out.append((s, prev))
            s = i
        prev = i
    out.append((s, prev))
    return out


def walls_1d(occ, res, horizontal):
    """Wall segments along rows (horizontal) or columns (vertical), merged."""
    g = occ if horizontal else occ.T
    segs = []
    for k in range(g.shape[0]):
        for a, b in runs(g[k]):
            if (b - a + 1) * res >= MIN_WALL:
                segs.append([k, a, b, b - a + 1])  # [row, start, end, weight]
    segs.sort()
    merged = []
    for k, a, b, w in segs:
        for m in merged:
            if abs(m[0] - k) <= MERGE and a <= m[2] + GAP and b >= m[1] - GAP:
                m[0] = (m[0] * m[3] + k * w) / (m[3] + w)
                m[1], m[2], m[3] = min(m[1], a), max(m[2], b), m[3] + w
                break
        else:
            merged.append([k, a, b, w])
    return [(k, a, b) for k, a, b, _ in merged]


def to_lines(hs, vs, res, origin):
    """Grid segments -> lines in metres [(x0, y0, x1, y1)], corners joined."""
    H = [[origin[0] + a * res, origin[1] + (k + .5) * res, origin[0] + (b + 1) * res] for k, a, b in hs]
    V = [[origin[0] + (k + .5) * res, origin[1] + a * res, origin[1] + (b + 1) * res] for k, a, b in vs]
    for h in H:           # h = [x0, y, x1]; v = [x, y0, y1]
        for v in V:
            if v[1] - SNAP <= h[1] <= v[2] + SNAP:
                for end in (0, 2):
                    if abs(h[end] - v[0]) < SNAP:
                        h[end] = v[0]
                for end in (1, 2):
                    if abs(v[end] - h[1]) < SNAP and h[0] - SNAP <= v[0] <= h[2] + SNAP:
                        v[end] = h[1]
    return [(x0, y, x1, y) for x0, y, x1 in H] + [(x, y0, x, y1) for x, y0, y1 in V]


def ransac_lines(pts, res, min_len, dist=0.05, min_pts=15, gap=0.30, seed=0):
    """Walls at any angle: repeatedly find the line with the most points (RANSAC),
    fit it precisely (PCA), split it at gaps and remove those points."""
    rng = np.random.default_rng(seed)
    pts = pts.copy()
    lines = []
    while len(pts) >= min_pts:
        best = None
        tree_pairs = rng.integers(0, len(pts), (400, 2))
        for i, j in tree_pairs:
            d = pts[j] - pts[i]
            L = np.hypot(*d)
            if L < 0.2 or L > 3.0:
                continue
            n = np.array([-d[1], d[0]]) / L
            inl = np.abs((pts - pts[i]) @ n) < dist
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < min_pts:
            break
        P = pts[best]
        c = P.mean(0)
        u = np.linalg.svd(P - c)[2][0]              # main direction of the inliers
        t = np.sort((P - c) @ u)
        splits = np.flatnonzero(np.diff(t) > gap)
        for a, b in zip(np.r_[0, splits + 1], np.r_[splits, len(t) - 1]):
            if t[b] - t[a] >= min_len and b - a + 1 >= 8:
                p0, p1 = c + t[a] * u, c + t[b] * u
                lines.append((p0[0], p0[1], p1[0], p1[1]))
        pts = pts[~best]
    return lines


def floor_mask(free, occ):
    """Floor: free cells, thin 'rays' through doors/windows removed, small holes filled."""
    f = ndimage.binary_closing(free | occ, iterations=2)
    f = ndimage.binary_opening(f, iterations=3)          # remove narrow protrusions
    lab, n = ndimage.label(f)
    if n:
        sizes = ndimage.sum(f, lab, range(1, n + 1))
        f = np.isin(lab, 1 + np.flatnonzero(sizes >= 0.1 * sizes.max()))
    return ndimage.binary_fill_holes(f)


def write_dxf(lines, fn):
    """Minimal ASCII DXF (R12): walls as LINE on layer WALLS, units mm."""
    out = ["0", "SECTION", "2", "ENTITIES"]
    for x0, y0, x1, y1 in lines:
        out += ["0", "LINE", "8", "WALLS",
                "10", f"{x0 * 1000:.0f}", "20", f"{y0 * 1000:.0f}", "30", "0",
                "11", f"{x1 * 1000:.0f}", "21", f"{y1 * 1000:.0f}", "31", "0"]
    out += ["0", "ENDSEC", "0", "EOF"]
    with open(fn, "w") as f:
        f.write("\n".join(out) + "\n")


def main():
    global MIN_WALL
    p = argparse.ArgumentParser(description="Floor plan from a SLAM map")
    p.add_argument("npz")
    p.add_argument("--out", default="floorplan")
    p.add_argument("--title", default="Floor plan")
    p.add_argument("--occ", type=float, default=OCC_T, help="wall threshold (log-odds, lower = more wall)")
    p.add_argument("--free", type=float, default=FREE_T, help="floor threshold (log-odds, lower = stricter)")
    p.add_argument("--minwall", type=float, default=MIN_WALL, help="shortest wall (m)")
    p.add_argument("--manhattan", action="store_true",
                   help="only right-angled walls (straighten); default: walls at any angle")
    a = p.parse_args()
    MIN_WALL = a.minwall

    z = np.load(a.npz)
    grid, res = z["grid"], float(z["res"])
    n = grid.shape[0]
    cells = lambda m: (np.argwhere(m)[:, ::-1] - n // 2 + 0.5) * res   # (x, y) in metres
    occ_pts, free_pts = cells(grid > a.occ), cells(grid < a.free)

    ang = dominant_angle(occ_pts) if a.manhattan else 0.0
    r = np.radians(ang)
    R = np.array([[math.cos(r), math.sin(r)], [-math.sin(r), math.cos(r)]])
    occ_pts, free_pts = occ_pts @ R.T, free_pts @ R.T
    path = z["path"] @ R.T if "path" in z else None

    lo = np.minimum(occ_pts.min(0), free_pts.min(0)) - 0.5
    hi = np.maximum(occ_pts.max(0), free_pts.max(0)) + 0.5
    shape = (int((hi[1] - lo[1]) / res) + 1, int((hi[0] - lo[0]) / res) + 1)
    occ = raster(occ_pts, res, lo, shape)
    free = raster(free_pts, res, lo, shape) & ~occ
    occ_c = ndimage.binary_dilation(occ, iterations=1)  # let thick, wavy walls join up

    if a.manhattan:
        lines = to_lines(walls_1d(occ_c, res, True), walls_1d(occ_c, res, False), res, lo)
    else:
        lines = ransac_lines(occ_pts, res, MIN_WALL)
    floor = floor_mask(free, occ)

    # loose obstacles (furniture etc.): occupied cells that are not on a wall
    wall_img = np.zeros(shape, bool)
    for x0, y0, x1, y1 in lines:
        for t in np.linspace(0, 1, int(max(abs(x1 - x0), abs(y1 - y0)) / res) + 2):
            i = int((y0 + t * (y1 - y0) - lo[1]) / res)
            j = int((x0 + t * (x1 - x0) - lo[0]) / res)
            if 0 <= i < shape[0] and 0 <= j < shape[1]:
                wall_img[i, j] = True
    loose = occ & ~ndimage.binary_dilation(wall_img, iterations=3)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ext = (lo[0], lo[0] + shape[1] * res, lo[1], lo[1] + shape[0] * res)
    w, h = ext[1] - ext[0], ext[3] - ext[2]
    fig, ax = plt.subplots(figsize=(min(16, 2 + w * 0.9), min(16, 2 + h * 0.9)), dpi=150)
    ax.imshow(np.where(floor, 1.0, np.nan), extent=ext, origin="lower", cmap="Blues",
              vmin=0, vmax=8, interpolation="nearest")
    ly, lx = np.nonzero(loose)
    ax.scatter(lo[0] + (lx + .5) * res, lo[1] + (ly + .5) * res, s=1.5, color="#9aa4b1", lw=0)
    for x0, y0, x1, y1 in lines:
        ax.plot([x0, x1], [y0, y1], color="#1c2430", lw=3.2, solid_capstyle="projecting")
        L = math.hypot(x1 - x0, y1 - y0)
        if L >= 1.0:
            horiz = abs(y1 - y0) <= abs(x1 - x0)
            ax.annotate(f"{L:.2f} m", ((x0 + x1) / 2, (y0 + y1) / 2), xytext=(0, 7) if horiz else (7, 0),
                        textcoords="offset points", fontsize=7, color="#b03a2e",
                        ha="center" if horiz else "left", va="bottom" if horiz else "center",
                        rotation=0 if horiz else 90) if a.manhattan else ax.text(
                (x0 + x1) / 2, (y0 + y1) / 2, f" {L:.2f} m", fontsize=7, color="#b03a2e",
                rotation=math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180 - (180 if math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180 > 90 else 0),
                rotation_mode="anchor", ha="center", va="bottom")
    if path is not None:
        ax.plot(path[:, 0], path[:, 1], "--", color="#e67e22", lw=1, alpha=.7, label="walked route")
        ax.legend(loc="lower right", fontsize=7, frameon=False)
    # scale bar 1 m
    sx, sy = ext[0] + 0.3, ext[2] + 0.3
    ax.plot([sx, sx + 1], [sy, sy], color="#1c2430", lw=3)
    ax.text(sx + 0.5, sy + 0.12, "1 m", ha="center", fontsize=8)
    ax.set_title(f"{a.title}  ({'walls straightened, rotated %.1f°' % ang if a.manhattan else 'walls at any angle'}; "
                 f"dimensions ±5-10 cm)", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(f"{a.out}.png", facecolor="white")
    write_dxf(lines, f"{a.out}.dxf")
    tot = sum(math.hypot(x1 - x0, y1 - y0) for x0, y0, x1, y1 in lines)
    print(f"{len(lines)} wall segments ({tot:.1f} m), floor {floor.sum() * res * res:.1f} m2 "
          f"-> {a.out}.png, {a.out}.dxf")


if __name__ == "__main__":
    main()
