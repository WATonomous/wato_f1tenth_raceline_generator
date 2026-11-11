"""Centerline and track widths from a map with one clean ring of free space.

The algorithm map_converter.ipynb walks through step by step, as one function, so the notebook and
make_raceline.py run the same code:

  1. free space (pixels > 210) -> distance to the nearest wall (Euclidean distance transform)
  2. keep the middle of the track (distance > THRESHOLD * max) and skeletonize it
  3. strip dead-end branches (spurs), so only the loop is left
  4. walk the loop from its leftmost point; drop stray points visited out of order
  5. start the loop on its straightest stretch (the optimizer joins the loop at its first point)
  6. pixels -> metres; smooth with a periodic spline, resample evenly, clamp widths at hairpins
  7. orient the loop counter-clockwise, so the output direction never depends on the map's shape

    python centerline.py maps/<map>                   # writes inputs/tracks/<map>.csv
"""

import argparse
import os
import sys

import numpy as np
from scipy import interpolate, ndimage

import track_editor

# walk order (x, y): down first, then left, up, right, then diagonals (as the notebook's DFS)
DIRECTIONS = [(0, -1), (-1, 0), (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]

DEFAULTS = {
    "threshold": 0.17,           # keep distance-transform values above this fraction of the max
    "track_width_margin": 0.0,   # [m] taken off both half-widths (extra safety margin)
    "smoothing": 1.0,            # splprep smoothing factor; higher = smoother
    "resample_step": 0.1,        # [m] spacing of the output points
    "width_curv_frac": 0.9,      # cap half-width at this fraction of the local turn radius (None = off)
    "prune_spurs": True,
    "rotate_to_straight": True,
    "left_start_y": None,        # None: start at the leftmost loop point; or a pixel row (from the bottom)
}


def _walk(mask, start):
    """Depth-first walk over the skeleton in DIRECTIONS order (iterative, no recursion limit)."""
    rows, cols = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    visited[start[1], start[0]] = True
    order = [start]
    stack = [[start, 0]]
    while stack:
        (px, py), k = stack[-1]
        if k >= len(DIRECTIONS):
            stack.pop()
            continue
        stack[-1][1] = k + 1
        qx, qy = px + DIRECTIONS[k][0], py + DIRECTIONS[k][1]
        if 0 <= qx < cols and 0 <= qy < rows and mask[qy, qx] and not visited[qy, qx]:
            visited[qy, qx] = True
            order.append((qx, qy))
            stack.append([(qx, qy), 0])
    return order


def _longest_run(points, max_step=3.0):
    """Keep the longest run of consecutive points less than max_step px apart (closed loop)."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    step = np.hypot(*np.diff(np.vstack([pts, pts[:1]]), axis=0).T)
    breaks = np.flatnonzero(step > max_step)
    if len(breaks) <= 1:
        return np.arange(n)
    runs = []
    for a, b in zip(breaks, np.roll(breaks, -1)):
        first, last = (a + 1) % n, b
        runs.append(np.arange(first, last + 1) if last >= first
                    else np.r_[np.arange(first, n), np.arange(0, last + 1)])
    return max(runs, key=len)


def _straightest_start(points, k=15):
    """Index of the point in the middle of the straightest stretch of the loop."""
    pts = np.asarray(points, dtype=float)
    before, after = pts - np.roll(pts, k, axis=0), np.roll(pts, -k, axis=0) - pts
    turn = np.abs(np.arctan2(before[:, 0] * after[:, 1] - before[:, 1] * after[:, 0],
                             (before * after).sum(axis=1)))
    turn = np.convolve(np.r_[turn[-k:], turn, turn[:k]], np.ones(2 * k + 1), "valid")
    return int(np.argmin(turn))


def extract(map_path, **options):
    """Run the whole conversion. Returns a dict with the result ('xyw': [x, y, w_right, w_left],
    counter-clockwise) and every intermediate step (for plots and debugging)."""
    from skimage.morphology import skeletonize
    opt = dict(DEFAULTS)
    opt.update({k: v for k, v in options.items() if v is not None or k in ("width_curv_frac", "left_start_y")})
    info = track_editor.load_map(map_path)
    out = {"info": info, "options": opt, "messages": []}
    say = out["messages"].append

    image = info.image[::-1].astype(np.float64)            # row 0 at the bottom, as the notebook
    binary = (image > track_editor.FREE_CUTOFF).astype(np.float64)
    dist = ndimage.distance_transform_edt(binary)
    centers = dist > opt["threshold"] * dist.max()
    skeleton = skeletonize(centers)
    if opt["prune_spurs"]:
        skeleton = track_editor.prune_spurs(skeleton)
    out.update(image=image, binary=binary, dist=dist, centers=centers, skeleton=skeleton)
    if not skeleton.any():
        raise ValueError("no centerline loop found: is the map one clean ring? "
                         "(python track_editor.py --check %s)" % map_path)

    # start: leftmost point of the largest piece of skeleton (never a stray speck)
    if opt["left_start_y"] is None:
        labels, n = ndimage.label(skeleton, structure=np.ones((3, 3)))
        sizes = ndimage.sum(skeleton, labels, index=np.arange(1, n + 1))
        ys, xs = np.nonzero(labels == int(np.argmax(sizes)) + 1)
        k = int(np.argmin(xs))
        start = (int(xs[k]), int(ys[k]))
    else:
        y = int(opt["left_start_y"])
        xs = np.flatnonzero(skeleton[y])
        if not len(xs):
            raise ValueError("no centerline on row %d (left_start_y)" % y)
        start = (int(xs[0]), y)
    order = _walk(skeleton, start)
    keep = _longest_run(order)
    if len(keep) < len(order):
        say("dropped %d stray point(s) visited out of order" % (len(order) - len(keep)))
    order = [order[i] for i in keep]
    if opt["rotate_to_straight"] and len(order) > 60:
        i0 = _straightest_start(order)
        order = order[i0:] + order[:i0]
    out.update(start_px=start, order_px=np.array(order))

    # pixels -> metres (x = col * res + origin_x, y = row * res + origin_y), widths from the distance transform
    px = np.array(order, dtype=float)
    w = dist[px[:, 1].astype(int), px[:, 0].astype(int)] * info.resolution - opt["track_width_margin"]
    raw = np.column_stack([px[:, 0] * info.resolution + info.origin[0],
                           px[:, 1] * info.resolution + info.origin[1], w, w])
    out["raw"] = raw

    # periodic smoothing spline, uniform arc-length resampling, curvature width clamp
    tck, u = interpolate.splprep([raw[:, 0], raw[:, 1]], per=1, k=3, s=opt["smoothing"])
    dense = np.array(interpolate.splev(np.linspace(0.0, 1.0, 2000), tck)).T
    length = float(np.sum(np.hypot(*np.diff(dense, axis=0).T)))
    n_points = max(int(round(length / opt["resample_step"])), 10)
    u_new = np.linspace(0.0, 1.0, n_points, endpoint=False)
    x_new, y_new = interpolate.splev(u_new, tck)
    wr = np.interp(u_new, u, raw[:, 2], period=1.0)
    wl = np.interp(u_new, u, raw[:, 3], period=1.0)
    out["min_radius_m"] = None
    if opt["width_curv_frac"] is not None:
        dx, dy = interpolate.splev(u_new, tck, der=1)
        ddx, ddy = interpolate.splev(u_new, tck, der=2)
        cross = np.abs(dx * ddy - dy * ddx)
        radius = np.where(cross > 1e-9, np.power(dx * dx + dy * dy, 1.5) / np.maximum(cross, 1e-12), np.inf)
        cap = opt["width_curv_frac"] * radius
        n_clamped = int(np.sum((wr > cap) | (wl > cap)))
        wr, wl = np.minimum(wr, cap), np.minimum(wl, cap)
        out["min_radius_m"] = float(np.min(radius))
        say("width clamp: %d of %d points reduced (tightest radius %.2f m)"
            % (n_clamped, n_points, np.min(radius)))
    xyw = np.column_stack([x_new, y_new, wr, wl])

    # always counter-clockwise: reverse the order (keeping the start point) and swap right/left
    walked = "ccw" if _signed_area(xyw[:, :2]) > 0 else "cw"
    if walked == "cw":
        xyw = np.vstack([xyw[:1], xyw[:0:-1]])[:, [0, 1, 3, 2]]
    out.update(xyw=xyw, length_m=length, walked_direction=walked)
    say("centerline: %d raw points -> %d points, %.1f m loop, written counter-clockwise (the walk went %s)"
        % (len(raw), len(xyw), length, walked))
    return out


def _signed_area(xy):
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def write_csv(path, xyw):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as fh:
        np.savetxt(fh, xyw, fmt="%0.4f", delimiter=",", header="x_m,y_m,w_tr_right_m,w_tr_left_m")


def main(argv=None):
    p = argparse.ArgumentParser(description="Centerline + track widths from a clean ring map.")
    p.add_argument("map", help="map in maps/ (name or path)")
    p.add_argument("--out", help="output csv (default inputs/tracks/<map>.csv)")
    args = p.parse_args(argv)
    result = extract(args.map)
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "inputs", "tracks",
                                   result["info"].name + ".csv")
    write_csv(out, result["xyw"])
    for m in result["messages"]:
        print(m)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
