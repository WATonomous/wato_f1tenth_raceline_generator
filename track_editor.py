"""Draw the race track on a map that is not a clean closed loop, and write a raceline-ready map.

map_converter.ipynb needs one closed ring of free space: white (254) inside the track, black (0)
on its two edges, grey (205) everywhere else. A map straight from slam_toolbox is rarely that, so
this tool lets you draw the outer and inner edge of the track on the original map and writes:

    maps/<map>_track.pgm          the ring: same size, resolution and origin as the original
    maps/<map>_track.yaml         copy of the original yaml, pointing at the new image
    maps/<map>_track_lines.json   the two edges in metres (reopen it to keep editing)

Build the raceline from <map>_track; keep the original map for localization (same frame).

Usage (in the wato_raceline conda env, from the repo root):

    python track_editor.py maps/<map>                 # draw (reopens <map>_track_lines.json if it exists)
    python track_editor.py maps/<map> --no-gui        # rewrite the map from the saved lines
    python track_editor.py --check maps/<map>         # check any map is one clean ring

Editor keys: click add a point (shift+click inserts it into the nearest segment), drag a point to
move it, right-click delete a point, o/i edit the outer/inner edge, u undo, c clear the current
edge, m smooth/straight edges, k keep/ignore obstacles inside the track, p preview the result,
s save, h help, q quit. Ctrl+plus / ctrl+minus or the scroll wheel zoom in and out at the mouse,
ctrl+0 shows the whole map. The toolbar's zoom and pan also work (editing pauses while they are on).
"""

import argparse
import json
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

FREE, UNKNOWN, OCCUPIED = 254, 205, 0
FREE_CUTOFF = 210            # map_converter.ipynb: pixels above this are free, the rest walls
LINES_VERSION = 1


# --------------------------------------------------------------------------------------------
# Map loading, coordinates

class MapInfo:
    """A map image (as stored in the file: row 0 at the top) and its yaml metadata."""

    def __init__(self, name, directory, image, meta):
        self.name = name
        self.directory = directory
        self.image = image                      # uint8, shape (rows, cols)
        self.meta = meta
        self.resolution = float(meta["resolution"])
        self.origin = [float(v) for v in meta["origin"][:2]]
        self.rows, self.cols = image.shape

    def to_metres(self, col, row):
        """Image pixel (file orientation) to map-frame metres, as map_converter.ipynb does."""
        x = np.asarray(col) * self.resolution + self.origin[0]
        y = (self.rows - 1 - np.asarray(row)) * self.resolution + self.origin[1]
        return x, y

    def to_pixels(self, x, y):
        """Map-frame metres to image pixel coordinates (file orientation), floats."""
        col = (np.asarray(x) - self.origin[0]) / self.resolution
        row = self.rows - 1 - (np.asarray(y) - self.origin[1]) / self.resolution
        return col, row

    def extent(self):
        """imshow extent (metres) for the image flipped to origin='lower'."""
        x0, y0 = self.origin
        return [x0, x0 + self.cols * self.resolution, y0, y0 + self.rows * self.resolution]

    def occupied_mask(self):
        """Cells the map server would call occupied (yaml occupied_thresh and negate)."""
        img = self.image.astype(np.float64)
        p = img / 255.0 if int(self.meta.get("negate", 0)) else (255.0 - img) / 255.0
        return p > float(self.meta.get("occupied_thresh", 0.65))


def split_map_path(path):
    """'maps/foo', 'maps/foo.pgm' or 'maps/foo.yaml' -> ('maps', 'foo')."""
    directory, base = os.path.split(os.path.abspath(path))
    name, ext = os.path.splitext(base)
    if ext.lower() not in (".pgm", ".png", ".yaml", ".yml", ""):
        name = base
    return directory, name


def load_map(path):
    directory, name = split_map_path(path)
    yaml_path = os.path.join(directory, name + ".yaml")
    if not os.path.exists(yaml_path):
        raise FileNotFoundError("no map yaml: " + yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    # Like map_converter.ipynb: <name>.png or <name>.pgm next to the yaml, then the yaml's image.
    candidates = [os.path.join(directory, name + ext) for ext in (".png", ".pgm")]
    candidates.append(os.path.join(directory, str(meta.get("image", ""))))
    image_path = next((c for c in candidates if os.path.isfile(c)), None)
    if image_path is None:
        raise FileNotFoundError("no map image for " + yaml_path)
    image = np.array(Image.open(image_path).convert("L"), dtype=np.uint8)
    return MapInfo(name, directory, image, meta)


# --------------------------------------------------------------------------------------------
# Edge curves

def dedupe(points):
    pts = [tuple(p) for p in points]
    out = []
    for p in pts:
        if not out or np.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-9:
            out.append(p)
    if len(out) > 1 and np.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= 1e-9:
        out.pop()
    return np.array(out, dtype=np.float64).reshape(-1, 2)


def closed_curve(points, smooth=True, spacing=0.02):
    """Closed curve through the points: centripetal Catmull-Rom (no overshoot or loops between
    points) when smooth, else the straight polygon. Sampled about every `spacing` metres."""
    pts = dedupe(points)
    n = len(pts)
    if n < 3:
        return pts
    if not smooth or n < 4:
        segs = []
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            k = max(2, int(np.ceil(np.hypot(*(b - a)) / spacing)))
            t = np.linspace(0.0, 1.0, k, endpoint=False)[:, None]
            segs.append(a + t * (b - a))
        return np.vstack(segs)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = pts[(i - 1) % n], pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        t0 = 0.0
        t1 = t0 + max(np.hypot(*(p1 - p0)), 1e-9) ** 0.5
        t2 = t1 + max(np.hypot(*(p2 - p1)), 1e-9) ** 0.5
        t3 = t2 + max(np.hypot(*(p3 - p2)), 1e-9) ** 0.5
        k = max(4, int(np.ceil(np.hypot(*(p2 - p1)) / spacing)))
        t = np.linspace(t1, t2, k, endpoint=False)[:, None]
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.vstack(out)


def _segments_cross(a, b, c, d):
    """Vectorised: do segments a-b and c-d properly cross? a, b: (N, 1, 2); c, d: (1, M, 2)."""
    def orient(p, q, r):
        return (q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1]) - \
               (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0])
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) & (o3 * o4 < 0)


def curve_problems(outer, inner, smooth=True):
    """Geometry problems with the two edges, as readable strings."""
    from matplotlib.path import Path
    problems = []
    curves = {}
    for label, pts in (("outer", outer), ("inner", inner)):
        if len(dedupe(pts)) < 3:
            problems.append("the %s edge needs at least 3 points" % label)
        else:
            curves[label] = closed_curve(pts, smooth, spacing=0.1)
    for label, c in curves.items():
        a, b = c, np.roll(c, -1, axis=0)
        cross = _segments_cross(a[:, None], b[:, None], a[None], b[None])
        n = len(c)
        idx = np.arange(n)
        near = np.abs(idx[:, None] - idx[None]) <= 1
        near |= (idx[:, None] == 0) & (idx[None] == n - 1)
        near |= (idx[:, None] == n - 1) & (idx[None] == 0)
        if np.any(cross & ~near):
            problems.append("the %s edge crosses itself" % label)
    if len(curves) == 2:
        o, i = curves["outer"], curves["inner"]
        cross = _segments_cross(o[:, None], np.roll(o, -1, axis=0)[:, None],
                                i[None], np.roll(i, -1, axis=0)[None])
        if np.any(cross):
            problems.append("the outer and inner edges cross")
        elif not np.all(Path(o).contains_points(i)):
            problems.append("the inner edge is not inside the outer edge")
    return problems


# --------------------------------------------------------------------------------------------
# The track map

def make_track_image(info, outer, inner, smooth=True, wall_px=3, keep_obstacles=True):
    """Ring between the two edges -> FREE, the edges -> OCCUPIED, the rest -> UNKNOWN.
    keep_obstacles keeps cells that are occupied in the original map (pillars) inside the ring."""
    spacing = info.resolution / 2.0
    polys = []
    for pts in (outer, inner):
        x, y = closed_curve(pts, smooth, spacing).T
        col, row = info.to_pixels(x, y)
        polys.append([(float(c), float(r)) for c, r in zip(col, row)])

    ring = Image.new("L", (info.cols, info.rows), 0)
    draw = ImageDraw.Draw(ring)
    draw.polygon(polys[0], fill=1)
    draw.polygon(polys[1], fill=0)
    ring = np.array(ring, dtype=bool)

    out = np.full(info.image.shape, UNKNOWN, dtype=np.uint8)
    out[ring] = FREE
    if keep_obstacles:
        out[ring & info.occupied_mask()] = OCCUPIED

    img = Image.fromarray(out, mode="L")
    draw = ImageDraw.Draw(img)
    for poly in polys:
        draw.line(poly + poly[:1], fill=OCCUPIED, width=max(2, int(wall_px)), joint="curve")
    out = np.array(img, dtype=np.uint8)
    # The walls can cut a few free pixels off the ring at sharp bends: grey them out.
    free = out > FREE_CUTOFF
    labels, n = ndimage.label(free, structure=np.ones((3, 3)))
    if n > 1:
        sizes = ndimage.sum(free, labels, index=np.arange(1, n + 1))
        out[free & (labels != int(np.argmax(sizes)) + 1)] = UNKNOWN
    return out


def output_paths(info, out_name=None, out_dir=None):
    out_name = out_name or info.name + "_track"
    out_dir = out_dir or info.directory
    base = os.path.join(out_dir, out_name)
    return {"name": out_name, "pgm": base + ".pgm", "yaml": base + ".yaml",
            "lines": os.path.join(out_dir, info.name + "_track_lines.json")}


def save_track(info, image, paths):
    Image.fromarray(image, mode="L").save(paths["pgm"])
    meta = dict(info.meta)
    meta["image"] = os.path.basename(paths["pgm"])
    meta["mode"] = meta.get("mode", "trinary")
    with open(paths["yaml"], "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)


def save_lines(path, info, lines):
    data = {"version": LINES_VERSION, "source_map": info.name,
            "resolution": info.resolution, "origin": info.origin}
    data.update(lines)
    data["outer"] = [[round(float(x), 4), round(float(y), 4)] for x, y in lines["outer"]]
    data["inner"] = [[round(float(x), 4), round(float(y), 4)] for x, y in lines["inner"]]
    text = json.dumps(data, indent=2)
    # one [x, y] point per line, so the file stays short and diffs read well
    text = re.sub(r"\[\s+(-?[\d.eE+-]+),\s+(-?[\d.eE+-]+)\s+\]", r"[\1, \2]", text)
    with open(path, "w") as f:
        f.write(text + "\n")


def default_lines():
    return {"outer": [], "inner": [], "smooth": True, "wall_px": 3, "keep_obstacles": True,
            "min_width_m": 0.6}


def load_lines(path):
    lines = default_lines()
    if path and os.path.exists(path):
        with open(path) as f:
            lines.update(json.load(f))
    return lines


# --------------------------------------------------------------------------------------------
# Checks

class CheckResult:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.info = []
        self.min_width_m = None

    @property
    def ok(self):
        return not self.errors

    def __str__(self):
        lines = ["map check: " + ("OK" if self.ok else "PROBLEMS")]
        lines += ["  error:   " + e for e in self.errors]
        lines += ["  warning: " + w for w in self.warnings]
        lines += ["  " + i for i in self.info]
        return "\n".join(lines)


def prune_spurs(skel, max_iter=5000):
    """Strip dead-end branches from a skeleton, leaving only its loops."""
    skel = skel.copy()
    kernel = np.ones((3, 3), dtype=np.int32)
    for _ in range(max_iter):
        neighbours = ndimage.convolve(skel.astype(np.int32), kernel, mode="constant") - skel
        ends = skel & (neighbours <= 1)
        if not ends.any():
            break
        skel &= ~ends
    return skel


def check_track_image(info, image, min_width_m=0.6, max_extra_hole_m2=0.25):
    """Is the free space (pixels > 210, as map_converter.ipynb reads it) one clean ring?"""
    from skimage.morphology import skeletonize
    res = CheckResult()
    px_area = info.resolution ** 2
    free = image > FREE_CUTOFF
    labels, n = ndimage.label(free, structure=np.ones((3, 3)))
    if n == 0:
        res.errors.append("no free space (nothing brighter than %d)" % FREE_CUTOFF)
        return res
    sizes = ndimage.sum(free, labels, index=np.arange(1, n + 1))
    main = int(np.argmax(sizes)) + 1
    ring = labels == main
    others = np.delete(sizes, main - 1)
    big = others[others >= 0.01 * sizes.max()]
    if len(big):
        res.warnings.append("%d other large free area(s) outside the track (%.1f m^2): the "
                            "centerline step may pick them up" % (len(big), big.sum() * px_area))
    if len(others) > len(big):
        res.info.append("%d tiny free speck(s) outside the track (ignored by the converter)"
                        % (len(others) - len(big)))
    edge = np.zeros_like(ring)
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
    if np.any(ring & edge):
        res.errors.append("the track's free space reaches the image border: its edge is not closed")

    walls, nw = ndimage.label(~ring)          # 4-connected, the dual of the 8-connected ring
    outside = set(np.unique(walls[edge & ~ring])) - {0}
    holes = [k for k in range(1, nw + 1) if k not in outside]
    if not holes:
        res.errors.append("the free space has no hole: it is a blob, not a ring around an infield")
    elif len(holes) > 1:
        hole_area = np.array(ndimage.sum(np.ones_like(ring), walls, index=holes)) * px_area
        order = np.argsort(hole_area)[::-1]
        extra = [(holes[k], hole_area[k]) for k in order[1:]]
        spots = []
        for h, area in extra[:5]:
            r, c = ndimage.center_of_mass(walls == h)
            x, y = info.to_metres(c, r)
            spots.append("(%.2f, %.2f) %.2f m^2" % (x, y, area))
        msg = ("%d obstacle(s) inside the track besides the infield, at %s: the centerline can "
               "split around them" % (len(extra), "; ".join(spots)))
        if len(extra) > 10 or extra[0][1] > max_extra_hole_m2:
            res.errors.append(msg + ". This is not one clean ring: draw the track edges "
                              "(TRACK_TYPE = \"open\")")
        else:
            res.warnings.append(msg + ". Move the edges so they are outside the track, or set "
                                "keep_obstacles off")

    dist = ndimage.distance_transform_edt(ring)
    loop = prune_spurs(skeletonize(ring))
    if np.any(loop):
        widths = 2.0 * dist[loop] * info.resolution
        k = int(np.argmin(widths))
        r, c = np.argwhere(loop)[k]
        x, y = info.to_metres(c, r)
        res.min_width_m = float(widths[k])
        msg = "narrowest track width %.2f m at (%.2f, %.2f)" % (widths[k], x, y)
        if widths[k] < min_width_m:
            res.warnings.append(msg + ", below the %.2f m minimum" % min_width_m)
        else:
            res.info.append(msg)
        res.info.append("track area %.1f m^2, median width %.2f m"
                        % (ring.sum() * px_area, float(np.median(widths))))
    return res


def check_map(path, min_width_m=0.6):
    """Check a map file on disk; returns a CheckResult (print it)."""
    info = load_map(path)
    return check_track_image(info, info.image, min_width_m)


def build_from_lines(info, lines, paths):
    """Headless: lines -> image -> files. Returns the CheckResult."""
    problems = curve_problems(lines["outer"], lines["inner"], lines["smooth"])
    if any("needs at least" in p for p in problems):
        raise ValueError("; ".join(problems))
    image = make_track_image(info, lines["outer"], lines["inner"], lines["smooth"],
                             lines["wall_px"], lines["keep_obstacles"])
    save_track(info, image, paths)
    result = check_track_image(info, image, lines["min_width_m"])
    result.errors = problems + result.errors
    return result


# --------------------------------------------------------------------------------------------
# Editor window

HELP = ("click add | shift+click insert | drag move | right-click delete | o/i outer/inner | "
        "u undo | c clear | m smooth | k obstacles | p preview | s save | q quit\n"
        "ctrl+plus / ctrl+minus or scroll: zoom at the mouse | ctrl+0: whole map")
ZOOM_STEP = 1.5   # each zoom key press or scroll notch


class Editor:
    COLORS = {"outer": "tab:red", "inner": "tab:blue"}

    def __init__(self, info, lines, paths):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.info, self.lines, self.paths = info, lines, paths
        self.active = "outer" if len(lines["outer"]) < 3 else "inner"
        self.history = []
        self.drag = None
        self.dirty = False
        self.previewing = False

        for key in ("keymap.save", "keymap.zoom", "keymap.pan", "keymap.back", "keymap.forward",
                    "keymap.home", "keymap.xscale", "keymap.yscale", "keymap.grid",
                    "keymap.grid_minor", "keymap.quit", "keymap.fullscreen"):
            if key in plt.rcParams:
                plt.rcParams[key] = []

        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        self.fig.canvas.manager.set_window_title("track editor: " + info.name)
        flipped = info.image[::-1]
        self.base = self.ax.imshow(flipped, cmap="gray", vmin=0, vmax=255, origin="lower",
                                   extent=info.extent(), interpolation="nearest")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.curve_art, self.point_art = {}, {}
        for label in ("outer", "inner"):
            self.curve_art[label], = self.ax.plot([], [], "-", color=self.COLORS[label], lw=1.5)
            self.point_art[label], = self.ax.plot([], [], "o", color=self.COLORS[label], ms=5)
        self.status = self.fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
        self.fig.text(0.01, 0.96, HELP, fontsize=8, family="monospace")

        cid = self.fig.canvas.mpl_connect
        cid("button_press_event", self.on_press)
        cid("motion_notify_event", self.on_motion)
        cid("button_release_event", self.on_release)
        cid("key_press_event", self.on_key)
        cid("scroll_event", self.on_scroll)
        cid("close_event", self.on_close)
        self.redraw("ready")

    # -- helpers
    def toolbar_busy(self):
        tb = self.fig.canvas.toolbar
        mode = getattr(tb, "mode", "") if tb is not None else ""
        return bool(getattr(mode, "value", mode))

    def push(self):
        self.history.append(json.dumps({k: self.lines[k] for k in ("outer", "inner")}))
        self.history = self.history[-200:]
        self.dirty = True

    def nearest_point(self, event, label, radius_px=8):
        pts = self.lines[label]
        if not pts:
            return None
        disp = self.ax.transData.transform(np.array(pts))
        d = np.hypot(disp[:, 0] - event.x, disp[:, 1] - event.y)
        k = int(np.argmin(d))
        return k if d[k] <= radius_px else None

    def insert_index(self, pts, x, y):
        p = np.array(pts)
        a, b = p, np.roll(p, -1, axis=0)
        ab = b - a
        t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) /
                    np.maximum((ab ** 2).sum(axis=1), 1e-12), 0, 1)
        proj = a + t[:, None] * ab
        return int(np.argmin(np.hypot(proj[:, 0] - x, proj[:, 1] - y))) + 1

    def redraw(self, message=""):
        for label in ("outer", "inner"):
            pts = self.lines[label]
            curve = closed_curve(pts, self.lines["smooth"], self.info.resolution) if len(pts) >= 3 else np.array(pts)
            if len(curve) >= 3:
                curve = np.vstack([curve, curve[:1]])
            xy = np.array(pts).reshape(-1, 2)
            self.curve_art[label].set_data(*(curve.T if len(curve) else ([], [])))
            self.point_art[label].set_data(*(xy.T if len(xy) else ([], [])))
            active = label == self.active
            self.curve_art[label].set_linewidth(2.5 if active else 1.2)
            self.point_art[label].set_markersize(6 if active else 3)
        problems = curve_problems(self.lines["outer"], self.lines["inner"], self.lines["smooth"])
        state = ("editing %s | outer %d pts, inner %d pts | %s | obstacles %s | %s%s"
                 % (self.active.upper(), len(self.lines["outer"]), len(self.lines["inner"]),
                    "smooth" if self.lines["smooth"] else "straight",
                    "kept" if self.lines["keep_obstacles"] else "ignored",
                    "PREVIEW | " if self.previewing else "", message))
        if problems:
            state += "\n" + "; ".join(problems)
        self.status.set_text(state)
        self.status.set_color("darkred" if problems else "black")
        self.fig.canvas.draw_idle()

    # -- events
    def on_press(self, event):
        if event.inaxes is not self.ax or self.toolbar_busy() or event.xdata is None:
            return
        pts = self.lines[self.active]
        if event.button == 1:
            k = self.nearest_point(event, self.active)
            if k is not None:
                self.push()
                self.drag = k
                return
            self.push()
            if event.key == "shift" and len(pts) >= 2:
                pts.insert(self.insert_index(pts, event.xdata, event.ydata), [event.xdata, event.ydata])
            else:
                pts.append([event.xdata, event.ydata])
            self.redraw("point added")
        elif event.button == 3:
            k = self.nearest_point(event, self.active, radius_px=12)
            if k is not None:
                self.push()
                pts.pop(k)
                self.redraw("point deleted")

    def on_motion(self, event):
        if self.drag is None or event.inaxes is not self.ax or event.xdata is None:
            return
        self.lines[self.active][self.drag] = [event.xdata, event.ydata]
        self.redraw("moving")

    def on_release(self, event):
        if self.drag is not None:
            self.drag = None
            self.redraw("point moved")

    def zoom(self, factor, x=None, y=None):
        """Zoom by `factor` (<1 in, >1 out) keeping the point (x, y) under the mouse in place."""
        (x0, x1), (y0, y1) = self.ax.get_xlim(), self.ax.get_ylim()
        if x is None or y is None:
            x, y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        full = self.info.extent()
        nx0, nx1 = x - (x - x0) * factor, x + (x1 - x) * factor
        ny0, ny1 = y - (y - y0) * factor, y + (y1 - y) * factor
        if (nx1 - nx0) >= (full[1] - full[0]) and (ny1 - ny0) >= (full[3] - full[2]):
            nx0, nx1, ny0, ny1 = full          # zoomed out past the whole map: show the whole map
        self.ax.set_xlim(nx0, nx1)
        self.ax.set_ylim(ny0, ny1)
        self.fig.canvas.draw_idle()

    def on_scroll(self, event):
        if event.inaxes is not self.ax:
            return
        self.zoom(1.0 / ZOOM_STEP if event.button == "up" else ZOOM_STEP, event.xdata, event.ydata)

    def on_key(self, event):
        key = event.key
        x = getattr(event, "xdata", None) if getattr(event, "inaxes", None) is self.ax else None
        y = getattr(event, "ydata", None) if x is not None else None
        # "+" needs shift on most keyboards, so Tk may report ctrl++, ctrl+= or ctrl+plus
        if key in ("ctrl++", "ctrl+=", "ctrl+plus", "ctrl+shift+=", "ctrl+shift++"):
            self.zoom(1.0 / ZOOM_STEP, x, y)
            return
        if key in ("ctrl+-", "ctrl+minus", "ctrl+_", "ctrl+shift+-", "ctrl+shift+_"):
            self.zoom(ZOOM_STEP, x, y)
            return
        if key in ("ctrl+0", "ctrl+)"):
            full = self.info.extent()
            self.ax.set_xlim(full[0], full[1])
            self.ax.set_ylim(full[2], full[3])
            self.fig.canvas.draw_idle()
            return
        if key in ("o", "i"):
            self.active = "outer" if key == "o" else "inner"
            self.redraw()
        elif key == "u":
            if self.history:
                prev = json.loads(self.history.pop())
                self.lines["outer"], self.lines["inner"] = prev["outer"], prev["inner"]
                self.redraw("undone")
        elif key == "c":
            self.push()
            self.lines[self.active] = []
            self.redraw("cleared " + self.active)
        elif key == "m":
            self.lines["smooth"] = not self.lines["smooth"]
            self.dirty = True
            self.redraw()
        elif key == "k":
            self.lines["keep_obstacles"] = not self.lines["keep_obstacles"]
            self.dirty = True
            self.redraw()
        elif key == "p":
            self.toggle_preview()
        elif key == "s":
            self.save()
        elif key == "h":
            print(__doc__)
            self.redraw("help printed in the terminal")
        elif key == "q":
            self.plt.close(self.fig)

    def toggle_preview(self):
        if self.previewing:
            self.base.set_data(self.info.image[::-1])
            self.previewing = False
            self.redraw("showing the original map")
            return
        if curve_problems(self.lines["outer"], self.lines["inner"], self.lines["smooth"]):
            self.redraw("fix the edges before previewing")
            return
        img = make_track_image(self.info, self.lines["outer"], self.lines["inner"],
                               self.lines["smooth"], self.lines["wall_px"], self.lines["keep_obstacles"])
        self.base.set_data(img[::-1])
        self.previewing = True
        result = check_track_image(self.info, img, self.lines["min_width_m"])
        print(result)
        self.redraw("preview: " + ("ok" if result.ok and not result.warnings else
                                   "see the terminal for warnings"))

    def save(self):
        try:
            result = build_from_lines(self.info, self.lines, self.paths)
        except ValueError as e:
            self.redraw("not saved: %s" % e)
            return
        save_lines(self.paths["lines"], self.info, self.lines)
        self.dirty = False
        print("saved %s, %s and %s" % (self.paths["pgm"], self.paths["yaml"], self.paths["lines"]))
        print(result)
        self.redraw("saved %s (%s)" % (self.paths["name"], "ok" if result.ok else "has problems"))

    def on_close(self, event):
        if self.dirty:
            print("closed with unsaved changes (press s to save next time)")

    def run(self):
        self.plt.show()


# --------------------------------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("map", help="map in maps/: name, .pgm/.png or .yaml path")
    p.add_argument("--check", action="store_true", help="only check that the map is one clean ring")
    p.add_argument("--no-gui", action="store_true", help="rewrite the track map from the saved lines")
    p.add_argument("--lines", help="lines json to load (default maps/<map>_track_lines.json)")
    p.add_argument("--out-name", help="output map name (default <map>_track)")
    p.add_argument("--out-dir", help="output directory (default the map's directory)")
    p.add_argument("--min-width", type=float, help="warn below this track width in m (default 0.6)")
    args = p.parse_args(argv)

    if args.check:
        result = check_map(args.map, args.min_width or 0.6)
        print(result)
        return 0 if result.ok else 1

    info = load_map(args.map)
    paths = output_paths(info, args.out_name, args.out_dir)
    lines = load_lines(args.lines or paths["lines"])
    if args.min_width:
        lines["min_width_m"] = args.min_width

    if args.no_gui:
        if len(lines["outer"]) < 3 or len(lines["inner"]) < 3:
            print("no saved edges in %s: draw them first (run without --no-gui)" % (args.lines or paths["lines"]))
            return 1
        result = build_from_lines(info, lines, paths)
        print("wrote %s and %s" % (paths["pgm"], paths["yaml"]))
        print(result)
        return 0 if result.ok else 1

    import matplotlib
    if "inline" in matplotlib.get_backend().lower() or not os.environ.get("DISPLAY", "x"):
        print("no display: the editor needs a desktop session")
        return 1
    try:
        matplotlib.use("TkAgg")
    except Exception:  # noqa: BLE001 (fall back to whatever GUI backend exists)
        pass
    Editor(info, lines, paths).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
