"""Raceline report: what the optimized line looks like and whether it is safe, one page per direction.

    python report.py <map>                 # outputs/<map>/report_ccw.png, report_cw.png, report_compare.png

Each page: the raceline on the original map coloured by speed, with direction arrows, start/finish,
the top-speed and slowest points, braking zones and any spot closer to the track edge than the car's
margin; speed along the lap; clearance to both edges; and the accelerations against the car's limits.
"""

import argparse
import json
import os
import sys

import numpy as np

import raceline_export
import track_editor

HERE = os.path.dirname(os.path.abspath(__file__))
ARROW = {"ccw": "↺", "cw": "↻"}


def load_run(map_name, direction, out_root=None):
    out_dir = os.path.join(out_root or os.path.join(HERE, "outputs"), map_name, direction)
    with open(os.path.join(out_dir, "summary_%s.json" % direction)) as f:
        summary = json.load(f)
    data = np.loadtxt(os.path.join(out_dir, "raceline_mppi_%s.csv" % direction), delimiter=";", skiprows=1)
    keys = ["s", "x", "y", "psi", "kappa", "v", "ax", "dr", "dl"]
    run = {k: data[:, i] for i, k in enumerate(keys)}
    run["summary"] = summary
    run["ay"] = run["v"] ** 2 * run["kappa"]
    return run


def background(map_name):
    """The map to draw on: the original (localization) map for a drawn track, else the map itself."""
    for name in ([map_name[:-len("_track")]] if map_name.endswith("_track") else []) + [map_name]:
        try:
            return track_editor.load_map(os.path.join(HERE, "maps", name))
        except FileNotFoundError:
            continue
    return None


def track_edges(map_name):
    """The two track edges in metres: the drawn ones if there are saved lines, else centerline +/- widths."""
    if map_name.endswith("_track"):
        lines_path = os.path.join(HERE, "maps", map_name + "_lines.json")
        if os.path.exists(lines_path):
            lines = track_editor.load_lines(lines_path)
            return [track_editor.closed_curve(lines[k], lines["smooth"], 0.05) for k in ("outer", "inner")]
    csv = os.path.join(HERE, "inputs", "tracks", map_name + ".csv")
    if not os.path.exists(csv):
        return []
    cl = np.loadtxt(csv, delimiter=",", comments="#")
    t = np.roll(cl[:, :2], -1, axis=0) - np.roll(cl[:, :2], 1, axis=0)
    t /= np.linalg.norm(t, axis=1)[:, None]
    left = np.column_stack([-t[:, 1], t[:, 0]])
    return [cl[:, :2] - left * cl[:, 2:3], cl[:, :2] + left * cl[:, 3:4]]


def draw_map(ax, run, info, edges, vmax, title=True, detail=True):
    from matplotlib.collections import LineCollection
    x, y, v, axl = run["x"], run["y"], run["v"], run["ax"]
    s = run["summary"]
    if info is not None:
        ax.imshow(info.image[::-1], cmap="gray", vmin=0, vmax=255, origin="lower", extent=info.extent(),
                  interpolation="nearest", alpha=0.55)
    for e in edges:
        ee = np.vstack([e, e[:1]])
        ax.plot(ee[:, 0], ee[:, 1], color="#c2571a", lw=1.2)
    pts = np.column_stack([x, y])
    segs = np.stack([pts, np.roll(pts, -1, axis=0)], axis=1)
    if detail:
        brake = axl < -1.0
        ax.add_collection(LineCollection(segs[brake], colors="red", linewidths=7, alpha=0.25))
    lc = LineCollection(segs, cmap="plasma", linewidths=2.6)
    lc.set_array(v)
    lc.set_clim(0, vmax)
    ax.add_collection(lc)
    # direction arrows about every 3 m
    step = max(1, int(round(3.0 / max(np.median(np.diff(run["s"])), 1e-3))))
    for i in range(step // 2, len(x) - 1, step):
        ax.annotate("", xy=(x[i + 1], y[i + 1]), xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle="-|>", color="black", lw=0, mutation_scale=14))
    ax.plot(x[0], y[0], "o", color="limegreen", ms=11, mec="black", zorder=5)
    ax.annotate("START", (x[0], y[0]), xytext=(8, 8), textcoords="offset points", fontsize=9, weight="bold",
                color="darkgreen")
    i_top, i_slow = int(np.argmax(v)), int(np.argmin(v))
    ax.plot(x[i_top], y[i_top], "*", color="gold", ms=18, mec="black", zorder=6)
    ax.annotate("top %.2f m/s" % v[i_top], (x[i_top], y[i_top]), xytext=(10, -14), textcoords="offset points",
                fontsize=10, weight="bold", bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.9))
    if detail:
        ax.plot(x[i_slow], y[i_slow], "v", color="deepskyblue", ms=10, mec="black", zorder=6)
        ax.annotate("slowest %.2f m/s" % v[i_slow], (x[i_slow], y[i_slow]), xytext=(10, 6),
                    textcoords="offset points", fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.85))
        for k, spot in enumerate(s.get("below_margin", [])):   # sorted worst first
            ax.plot(*spot["at_m"], "o", ms=16, mfc="none", mec="red", mew=2.5, zorder=7)
            if k < 3:
                ax.annotate("%.2f m from edge" % spot["clearance_m"], spot["at_m"], xytext=(-40, -24 - 12 * k),
                            textcoords="offset points", fontsize=9, color="red", weight="bold",
                            bbox=dict(boxstyle="round", fc="white", ec="red", alpha=0.85))
    pad = 1.0
    ax.set_xlim(x.min() - pad, x.max() + pad)
    ax.set_ylim(y.min() - pad, y.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    if title:
        ax.set_title("%s %s  lap %.2f s, top %.2f m/s" % (ARROW[s["direction"]], s["direction_name"],
                                                          s["laptime_s"], s["top_speed_mps"]), fontsize=12)
    return lc


def make_page(map_name, direction, out_root=None, info=None, edges=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    run = load_run(map_name, direction, out_root)
    s = run["summary"]
    car = s.get("car") or {}
    vmax = car.get("top_speed_mps", float(run["v"].max()))
    info = info if info is not None else background(map_name)
    edges = edges if edges is not None else track_edges(map_name)

    fig = plt.figure(figsize=(18, 11))
    gs = fig.add_gridspec(3, 3, width_ratios=[1.25, 1.25, 1.0], hspace=0.45, wspace=0.28)
    ax_map = fig.add_subplot(gs[:, :2])
    lc = draw_map(ax_map, run, info, edges, vmax, title=False)
    cb = fig.colorbar(lc, ax=ax_map, shrink=0.6, pad=0.01)
    cb.set_label("speed (m/s)")

    # speed along the lap, with accelerating / braking bands
    ax_v = fig.add_subplot(gs[0, 2])
    sd, v, axl = run["s"], run["v"], run["ax"]
    ax_v.plot(sd, v, color="black", lw=1.5)
    ax_v.fill_between(sd, 0, v, where=axl > 0.3, color="tab:green", alpha=0.3, label="speeding up")
    ax_v.fill_between(sd, 0, v, where=axl < -0.3, color="tab:red", alpha=0.3, label="braking")
    if car.get("top_speed_mps"):
        ax_v.axhline(car["top_speed_mps"], ls="--", color="gray", lw=1, label="car top speed")
    i_top = int(np.argmax(v))
    ax_v.plot(sd[i_top], v[i_top], "*", color="gold", ms=14, mec="black")
    ax_v.set_xlabel("distance along the lap (m)")
    ax_v.set_ylabel("speed (m/s)")
    ax_v.set_title("speed (top %.2f m/s at %.1f m)" % (v[i_top], sd[i_top]))
    ax_v.set_ylim(0, max(vmax, v.max()) * 1.1)
    ax_v.legend(fontsize=8, loc="lower right")
    ax_v.grid(alpha=0.3)

    # clearance to the real track edges
    ax_c = fig.add_subplot(gs[1, 2])
    half = 0.5 * (s.get("car") or {}).get("width_opt_m", 0) - (s.get("car") or {}).get("edge_margin_m", 0) \
        if s.get("car") else 0.148
    right, left = run["dr"] - half, run["dl"] - half
    ax_c.plot(sd, right, color="tab:blue", lw=1.2, label="right edge")
    ax_c.plot(sd, left, color="tab:purple", lw=1.2, label="left edge")
    margin = car.get("edge_margin_m")
    if margin is not None:
        ax_c.axhline(margin, ls="--", color="red", lw=1, label="edge margin %.2f m" % margin)
        ax_c.fill_between(sd, np.minimum(right, left), margin, where=np.minimum(right, left) < margin,
                          color="red", alpha=0.3)
    ax_c.axhline(0, color="black", lw=0.8)
    ax_c.set_xlabel("distance along the lap (m)")
    ax_c.set_ylabel("car side to edge (m)")
    ax_c.set_title("clearance to the track edges (min %.2f m)" % s["min_edge_clearance_m"])
    ax_c.legend(fontsize=8, loc="upper right")
    ax_c.grid(alpha=0.3)

    # accelerations against the car's limits (the optimizer's safe_traj ellipses)
    ax_g = fig.add_subplot(gs[2, 2])
    sc = ax_g.scatter(run["ay"], axl, c=v, cmap="plasma", s=6, vmin=0, vmax=vmax)
    a_up, a_dn, a_y = car.get("max_accel_mps2"), car.get("max_decel_mps2"), car.get("max_lateral_accel_mps2")
    if a_up and a_dn and a_y:
        ax_g.add_patch(Ellipse((0, 0), 2 * a_y, 2 * a_up, fill=False, ls="--", color="tab:green"))
        ax_g.add_patch(Ellipse((0, 0), 2 * a_y, 2 * a_dn, fill=False, ls="--", color="tab:red"))
        lim = 1.25 * max(a_up, a_dn, a_y)
        ax_g.set_xlim(-lim, lim)
        ax_g.set_ylim(-lim, lim)
    ax_g.axhline(0, color="black", lw=0.6)
    ax_g.axvline(0, color="black", lw=0.6)
    ax_g.set_xlabel("sideways (m/s^2)")
    ax_g.set_ylabel("forward (m/s^2)")
    ax_g.set_title("accelerations vs the car's limits (dashed)")
    ax_g.set_aspect("equal")
    ax_g.grid(alpha=0.3)

    warn = ("   |   %d spot(s) closer to the edge than the margin" % len(s.get("below_margin", []))
            if s.get("below_margin") else "")
    fig.suptitle("%s   %s %s (%s)   lap %.2f s   top %.2f m/s   length %.1f m%s"
                 % (map_name, ARROW[direction], s["direction_name"], direction, s["laptime_s"],
                    s["top_speed_mps"], s["length_m"], warn),
                 fontsize=15, weight="bold", color="darkred" if warn else "black")
    if car:
        fig.text(0.01, 0.005, "car: " + ", ".join("%s %s" % (k, v_) for k, v_ in car.items()),
                 fontsize=8, family="monospace", color="dimgray")
    path = os.path.join(out_root or os.path.join(HERE, "outputs"), map_name, "report_%s.png" % direction)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return path


def make_compare(map_name, directions, out_root=None, info=None, edges=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    runs = [load_run(map_name, d, out_root) for d in directions]
    info = info if info is not None else background(map_name)
    edges = edges if edges is not None else track_edges(map_name)
    vmax = max((r["summary"].get("car") or {}).get("top_speed_mps", r["v"].max()) for r in runs)
    fig, axes = plt.subplots(1, len(runs), figsize=(9 * len(runs), 8.5), squeeze=False)
    lc = None
    for ax, run in zip(axes[0], runs):
        lc = draw_map(ax, run, info, edges, vmax, title=True, detail=False)
    fig.colorbar(lc, ax=axes[0].tolist(), shrink=0.7, pad=0.01).set_label("speed (m/s)")
    best = min(runs, key=lambda r: r["summary"]["laptime_s"])["summary"]
    fig.suptitle("%s: %s" % (map_name, "   vs   ".join(
        "%s %s %.2f s" % (ARROW[r["summary"]["direction"]], r["summary"]["direction"], r["summary"]["laptime_s"])
        for r in runs)) + ("   (faster: %s)" % best["direction"] if len(runs) > 1 else ""),
        fontsize=15, weight="bold")
    path = os.path.join(out_root or os.path.join(HERE, "outputs"), map_name, "report_compare.png")
    fig.savefig(path, dpi=100, bbox_inches="tight")
    plt.close(fig)
    return path


def make_report(map_name, directions=None, out_root=None):
    root = out_root or os.path.join(HERE, "outputs")
    directions = directions or [d for d in raceline_export.DIRECTIONS
                                if os.path.exists(os.path.join(root, map_name, d, "summary_%s.json" % d))]
    if not directions:
        raise FileNotFoundError("no optimized raceline in %s/%s/<ccw|cw>/" % (root, map_name))
    info, edges = background(map_name), track_edges(map_name)
    paths = [make_page(map_name, d, out_root, info, edges) for d in directions]
    paths.append(make_compare(map_name, directions, out_root, info, edges))
    return paths


def main(argv=None):
    p = argparse.ArgumentParser(description="Raceline report pages (PNG) for an optimized map.")
    p.add_argument("map", help="map name as optimized (e.g. e7_fifth_v3_track)")
    p.add_argument("--directions", nargs="+", choices=list(raceline_export.DIRECTIONS))
    p.add_argument("--out-root", help="outputs folder (default outputs/)")
    args = p.parse_args(argv)
    for path in make_report(args.map, args.directions, args.out_root):
        print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
