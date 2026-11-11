"""Write an optimized raceline straight into the stack's formats (no conversion step in between).

    raceline_<dir>.csv        x_m,y_m,v_mps,dr_m,dl_m        race_tracks raceline.csv (global planner)
    raceline_mppi_<dir>.csv   s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2;w_tr_right_m;w_tr_left_m   (MPPI)
    summary_<dir>.json        lap time, top speed and where, clearance, the car profile used

dr_m / dl_m are the distances from each raceline point to the right / left edge of the track the
centerline was made from (the drawn edges for an open track), measured across the direction of travel.
"""

import datetime
import json
import os
import subprocess

import numpy as np
from scipy.spatial import cKDTree

DIRECTIONS = {"ccw": "counter-clockwise", "cw": "clockwise"}


def signed_area(xy):
    x, y = np.asarray(xy)[:, 0], np.asarray(xy)[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def direction_of(xy):
    """'ccw' or 'cw' in the map frame (x right, y up, as Foxglove and RViz show it)."""
    return "ccw" if signed_area(xy) > 0 else "cw"


def edge_distances(xy, reftrack):
    """Distances to the right and left track edge. reftrack is [x, y, w_right, w_left] in the
    same direction of travel as xy."""
    xy = np.asarray(xy, dtype=float)
    ref = np.asarray(reftrack, dtype=float)
    n = len(ref)
    _, idx = cKDTree(ref[:, :2]).query(xy)
    tangent = ref[(idx + 1) % n, :2] - ref[(idx - 1) % n, :2]
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    left_normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    d = np.sum((xy - ref[idx, :2]) * left_normal, axis=1)     # > 0: left of the centerline
    return ref[idx, 2] + d, ref[idx, 3] - d


def heading_curvature(xy):
    """Heading (rad, 0 along +x, counter-clockwise positive) and curvature (1/m, > 0 turning left)
    of a closed polyline."""
    xy = np.asarray(xy, dtype=float)
    fwd = np.roll(xy, -1, axis=0) - xy
    back = xy - np.roll(xy, 1, axis=0)
    tangent = fwd + back
    psi = np.arctan2(tangent[:, 1], tangent[:, 0])
    dpsi = np.angle(np.exp(1j * (np.roll(psi, -1) - np.roll(psi, 1))))
    ds = np.linalg.norm(fwd, axis=1) + np.linalg.norm(back, axis=1)
    return psi, dpsi / np.maximum(ds, 1e-9)


def git_commit(repo_dir):
    try:
        out = subprocess.run(["git", "-C", repo_dir, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "-C", repo_dir, "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return out.stdout.strip() + ("-dirty" if dirty else "") if out.returncode == 0 else None
    except Exception:  # noqa: BLE001
        return None


def export(out_dir, direction, trajectory, reftrack, laptime, car_summary=None, width_veh=0.296,
           map_name=None, extra=None):
    """trajectory: [s, x, y, psi, kappa, vx, ax] (open, no repeated last point), in the direction of
    travel; reftrack: [x, y, w_right, w_left] in the same direction. Returns the summary dict."""
    os.makedirs(out_dir, exist_ok=True)
    traj = np.asarray(trajectory, dtype=float)
    s, xy, vx = traj[:, 0], traj[:, 1:3], traj[:, 5]
    dr, dl = edge_distances(xy, reftrack)
    psi, kappa_num = heading_curvature(xy)
    # the optimizer's analytic curvature (smooth); point-to-point curvature is too noisy at 0.15 m
    # spacing. Match its sign to "> 0 turning left" using the numeric estimate.
    kappa = traj[:, 4] * (1.0 if np.dot(traj[:, 4], kappa_num) >= 0 else -1.0)
    # acceleration from the speed and the real point spacing, across the closing seam too (the
    # optimizer's own profile divides by a near-zero closing segment at the last point)
    ds = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    ax = (np.roll(vx, -1) ** 2 - vx ** 2) / (2.0 * np.maximum(ds, 1e-6))

    stack_path = os.path.join(out_dir, "raceline_%s.csv" % direction)
    with open(stack_path, "w") as f:
        f.write("x_m,y_m,v_mps,dr_m,dl_m\n")
        for row in np.column_stack([xy, vx, dr, dl]):
            f.write("%.4f,%.4f,%.4f,%.4f,%.4f\n" % tuple(row))

    mppi_path = os.path.join(out_dir, "raceline_mppi_%s.csv" % direction)
    with open(mppi_path, "w") as f:
        f.write("s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2;w_tr_right_m;w_tr_left_m\n")
        for row in np.column_stack([s, xy, psi, kappa, vx, ax, dr, dl]):
            f.write("%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n" % tuple(row))

    i_top, i_slow = int(np.argmax(vx)), int(np.argmin(vx))
    ay = vx ** 2 * kappa
    clearance = np.minimum(dr, dl) - width_veh / 2.0
    i_clear = int(np.argmin(clearance))
    seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)

    # spots closer to the real track edges than the car profile's margin (5 cm tolerance)
    below_margin = []
    margin = (car_summary or {}).get("edge_margin_m")
    if margin is not None:
        low = clearance < margin - 0.05
        if low.any():
            idx = np.flatnonzero(low)
            groups = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == len(xy) - 1:
                groups[0] = np.concatenate([groups.pop(), groups[0]])   # one spot across the seam
            for g in sorted(groups, key=lambda g: clearance[g].min())[:10]:
                k = g[np.argmin(clearance[g])]
                below_margin.append({"at_m": [round(float(xy[k, 0]), 2), round(float(xy[k, 1]), 2)],
                                     "clearance_m": round(float(clearance[k]), 3),
                                     "length_m": round(float(seg[g].sum()), 2)})
    summary = {
        "map": map_name,
        "direction": direction,
        "direction_name": DIRECTIONS[direction],
        "laptime_s": round(float(laptime), 3),
        "length_m": round(float(seg.sum()), 2),
        "top_speed_mps": round(float(vx[i_top]), 3),
        "top_speed_at_m": [round(float(xy[i_top, 0]), 3), round(float(xy[i_top, 1]), 3)],
        "min_speed_mps": round(float(vx[i_slow]), 3),
        "min_speed_at_m": [round(float(xy[i_slow, 0]), 3), round(float(xy[i_slow, 1]), 3)],
        "max_accel_mps2": round(float(ax.max()), 3),
        "max_decel_mps2": round(float(-ax.min()), 3),
        "max_lateral_accel_mps2": round(float(np.abs(ay).max()), 3),
        "min_edge_clearance_m": round(float(clearance[i_clear]), 3),
        "min_edge_clearance_at_m": [round(float(xy[i_clear, 0]), 3), round(float(xy[i_clear, 1]), 3)],
        "below_margin": below_margin,
        "start_m": [round(float(xy[0, 0]), 3), round(float(xy[0, 1]), 3)],
        "car": car_summary,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "generator_commit": git_commit(os.path.dirname(os.path.abspath(__file__))),
        "files": {"raceline": os.path.basename(stack_path), "raceline_mppi": os.path.basename(mppi_path)},
    }
    if extra:
        summary.update(extra)
    with open(os.path.join(out_dir, "summary_%s.json" % direction), "w") as f:
        json.dump(summary, f, indent=2)
    return summary
