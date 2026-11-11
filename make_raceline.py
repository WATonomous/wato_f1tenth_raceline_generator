"""One command from a map to the raceline files the car uses. The only human step is drawing the
track's edges (python track_editor.py maps/<map>) when the map is not already one clean ring.

    python make_raceline.py <map>                         # optimize both directions, write the reports
    python make_raceline.py <map> --export                # ... and write them into the stack's race_tracks
    python make_raceline.py <map> --track <race_tracks name> --export --default-direction ccw

Steps:
  1. map: maps/<map>.yaml, or copied in from the stack's race_tracks/tracks/<track>/map.*
  2. track: rebuilt from maps/<map>_track_lines.json if there are drawn edges, else the map itself if it is
     one clean ring; otherwise stop and ask for the edges to be drawn
  3. centerline: inputs/tracks/<name>.csv (centerline.py, always counter-clockwise)
  4. optimize clockwise and counter-clockwise in parallel with the car profile (params/wato_car.yaml).
     A failed solve is retried with more reference smoothing (s_reg 1 -> 3 -> 10). If the line comes closer to
     the real track edges than the profile's edge margin, it is re-solved with a wider buffer (up to 2 times).
  5. report: outputs/<name>/report_ccw.png, report_cw.png, report_compare.png
  6. --export: race_tracks/tracks/<track>/raceline_{ccw,cw}.csv, raceline_mppi_{ccw,cw}.csv, raceline.csv and
     raceline_mppi.csv (the default direction) and track_info.yaml
"""

import argparse
import concurrent.futures
import datetime
import json
import os
import shutil
import subprocess
import sys

import numpy as np
import yaml

import car_profile
import centerline
import raceline_export
import report
import track_editor

HERE = os.path.dirname(os.path.abspath(__file__))
S_REG_LADDER = [1.0, 3.0, 10.0]
MARGIN_TOLERANCE = 0.05     # [m] clearance this much under the margin is accepted
MAX_MARGIN_RETRIES = 2


def log(msg):
    print(msg, flush=True)


def find_stack_root(explicit=None):
    candidates = [explicit] if explicit else [os.path.join(HERE, "..", ".."), os.path.join(HERE, "..")]
    for c in candidates:
        if c and os.path.isdir(os.path.join(c, "src", "world_modeling", "race_tracks", "tracks")):
            return os.path.abspath(c)
    return None


def import_map(map_name, track, stack_root):
    """Make sure maps/<map_name>.yaml exists, copying the stack's race_tracks map in if needed."""
    if os.path.exists(os.path.join(HERE, "maps", map_name + ".yaml")):
        return
    src = os.path.join(stack_root or "", "src", "world_modeling", "race_tracks", "tracks", track or map_name)
    if not stack_root or not os.path.exists(os.path.join(src, "map.yaml")):
        raise SystemExit("ERROR: no maps/%s.yaml, and no race_tracks/tracks/%s/map.yaml to copy in"
                         % (map_name, track or map_name))
    with open(os.path.join(src, "map.yaml")) as f:
        meta = yaml.safe_load(f)
    image = os.path.join(src, str(meta.get("image", "map.pgm")))
    ext = os.path.splitext(image)[1] or ".pgm"
    shutil.copy2(image, os.path.join(HERE, "maps", map_name + ext))
    meta["image"] = map_name + ext
    with open(os.path.join(HERE, "maps", map_name + ".yaml"), "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)
    log("map: copied %s into maps/%s%s" % (os.path.relpath(src), map_name, ext))


def prepare_track(map_name):
    """Returns the name of the clean ring map to build the raceline from, and the edges file (or None)."""
    lines_path = os.path.join(HERE, "maps", map_name + "_track_lines.json")
    if os.path.exists(lines_path):
        info = track_editor.load_map(os.path.join(HERE, "maps", map_name))
        paths = track_editor.output_paths(info)
        result = track_editor.build_from_lines(info, track_editor.load_lines(lines_path), paths)
        log("track: rebuilt maps/%s from your drawn edges" % paths["name"])
        log("  " + str(result).replace("\n", "\n  "))
        if not result.ok:
            raise SystemExit("ERROR: the drawn track has problems (above): fix them in "
                             "python track_editor.py maps/%s" % map_name)
        return paths["name"], lines_path
    result = track_editor.check_map(os.path.join(HERE, "maps", map_name))
    if result.ok:
        log("track: maps/%s is one clean ring (closed track)" % map_name)
        return map_name, None
    log(str(result))
    log("\nThis map is not one clean ring, so the track's edges need drawing first (a person does this):\n"
        "    python track_editor.py maps/%s\n"
        "Draw the outer edge, press i, draw the inner edge, press s, close the window. Then run this again."
        % map_name)
    raise SystemExit(2)


def oriented(xyw, direction):
    """Centerline [x, y, w_right, w_left] running in `direction` (start point kept, sides swapped)."""
    if raceline_export.direction_of(xyw[:, :2]) == direction:
        return xyw.copy()
    return np.vstack([xyw[:1], xyw[:0:-1]])[:, [0, 1, 3, 2]]


def shrink_where_too_close(track, raceline_csv, width_veh, margin, width_opt, radius=1.0):
    """Narrow the corridor on the side the raceline came too close to, around those spots only.
    track: [x, y, w_right, w_left] in the raceline's direction. Returns (new track, number of spots)."""
    rl = np.loadtxt(raceline_csv, delimiter=",", skiprows=1)
    clearance_r, clearance_l = rl[:, 3] - width_veh / 2, rl[:, 4] - width_veh / 2
    out = track.copy()
    cut = np.zeros((len(track), 2))
    bad = np.flatnonzero(np.minimum(clearance_r, clearance_l) < margin - MARGIN_TOLERANCE)
    for i in bad:
        side = 0 if clearance_r[i] < clearance_l[i] else 1
        deficit = margin - min(clearance_r[i], clearance_l[i]) + 0.03
        near = np.hypot(track[:, 0] - rl[i, 0], track[:, 1] - rl[i, 1]) < radius
        cut[near, side] = np.maximum(cut[near, side], deficit)
    out[:, 2:4] -= cut
    # keep the corridor solvable: each side at least half the solver width, both together the full width
    out[:, 2:4] = np.maximum(out[:, 2:4], width_opt / 2.0 + 0.01)
    return out, len(bad)


def run_optimizer(name, direction, s_reg, start, car_path, out_dir, track_csv, edges_csv):
    os.makedirs(out_dir, exist_ok=True)
    cmd = [sys.executable, os.path.join(HERE, "main_globaltraj_f110.py"), "--map_name", name,
           "--direction", direction, "--car", car_path, "--s_reg", str(s_reg), "--out_dir", out_dir,
           "--map_path", track_csv, "--edges_path", edges_csv]
    if start:
        cmd += ["--start", "%.4f,%.4f" % tuple(start)]
    env = dict(os.environ, MPLBACKEND="Agg")
    with open(os.path.join(out_dir, "optimize.log"), "w") as logf:
        rc = subprocess.run(cmd, cwd=HERE, stdout=logf, stderr=subprocess.STDOUT, env=env).returncode
    summary_path = os.path.join(out_dir, "summary_%s.json" % direction)
    if rc != 0 or not os.path.exists(summary_path):
        with open(os.path.join(out_dir, "optimize.log")) as f:
            tail = [l.strip() for l in f if l.startswith(("EXIT", "ERROR", "OSError", "RuntimeError", "SystemExit"))]
        return None, (tail[-1] if tail else "exit code %d" % rc)
    with open(summary_path) as f:
        return json.load(f), None


def optimize_direction(name, direction, car, car_path, start, centerline_xyw, margin_retries):
    """Solve one direction with the s_reg ladder; then, where the line comes closer to the real track
    edges than the margin, narrow the corridor on that side there and re-solve. Returns (summary, attempts)."""
    margin = car["track"]["edge_margin_m"]
    width_opt = car_profile.width_opt(car)
    width_veh = car["car"]["width_m"]
    base = os.path.join(HERE, "outputs", name, direction)
    shutil.rmtree(base, ignore_errors=True)
    os.makedirs(base)
    real = oriented(centerline_xyw, direction)            # the real track, in this direction
    edges_csv = os.path.join(base, "centerline_%s.csv" % direction)
    centerline.write_csv(edges_csv, real)
    attempts, best = [], None
    for s_reg in S_REG_LADDER:
        track, track_csv = real, edges_csv
        for k in range(1 + (MAX_MARGIN_RETRIES if margin_retries else 0)):
            tag = "try_sreg%g_%d" % (s_reg, k)
            summary, err = run_optimizer(name, direction, s_reg, start, car_path, os.path.join(base, tag),
                                         track_csv, edges_csv)
            attempts.append({"s_reg": s_reg, "corridor_narrowed": k, "ok": summary is not None,
                             "min_edge_clearance_m": summary and summary["min_edge_clearance_m"],
                             "laptime_s": summary and summary["laptime_s"], "error": err, "dir": tag})
            log("  %-3s s_reg %-4g %s: %s" % (direction, s_reg, "first solve " if k == 0 else "narrowed x%d" % k,
                "lap %.2f s, min clearance %.2f m, %d spot(s) under the margin"
                % (summary["laptime_s"], summary["min_edge_clearance_m"], len(summary["below_margin"]))
                if summary else "failed (%s)" % err))
            if summary is None:
                break
            if best is None or len(summary["below_margin"]) <= len(best[0]["below_margin"]):
                best = (summary, tag)
            if not summary["below_margin"] or k == MAX_MARGIN_RETRIES or not margin_retries:
                break
            track, n_bad = shrink_where_too_close(track, os.path.join(base, tag, "raceline_%s.csv" % direction),
                                                  width_veh, margin, width_opt)
            track_csv = os.path.join(base, tag, "corridor_%s.csv" % direction)
            centerline.write_csv(track_csv, track)
        if best is not None:
            break
    if best is None:
        return None, attempts
    summary, tag = best
    for f in os.listdir(os.path.join(base, tag)):
        if f.startswith(("raceline_", "summary_")):
            shutil.copy2(os.path.join(base, tag, f), os.path.join(base, f))
    summary["solver"] = {"chosen": tag, "attempts": attempts}
    with open(os.path.join(base, "summary_%s.json" % direction), "w") as f:
        json.dump(summary, f, indent=2)
    return summary, attempts


def existing_direction(track_dir):
    """The direction the stack currently drives this track, so a regeneration doesn't flip it silently."""
    info_path = os.path.join(track_dir, "track_info.yaml")
    if os.path.exists(info_path):
        with open(info_path) as f:
            d = (yaml.safe_load(f) or {}).get("default_direction")
        if d in raceline_export.DIRECTIONS:
            return d, "track_info.yaml"
    rl = os.path.join(track_dir, "raceline.csv")
    if os.path.exists(rl):
        xy = np.loadtxt(rl, delimiter=",", skiprows=1)[:, :2]
        return raceline_export.direction_of(xy), "the current raceline.csv"
    return None, None


def export(name, track, stack_root, summaries, default_direction, source_map, lines_path):
    track_dir = os.path.join(stack_root, "src", "world_modeling", "race_tracks", "tracks", track)
    if not os.path.isdir(track_dir):
        raise SystemExit("ERROR: no race_tracks/tracks/%s in the stack (add the map there first)" % track)
    if default_direction is None:
        default_direction, why = existing_direction(track_dir)
        default_direction = default_direction or "ccw"
        log("export: default direction %s (%s)" % (default_direction, why or "nothing there yet"))
    if default_direction not in summaries:
        raise SystemExit("ERROR: default direction %s was not optimized" % default_direction)
    for d in summaries:
        out = os.path.join(HERE, "outputs", name, d)
        for f in ("raceline_%s.csv" % d, "raceline_mppi_%s.csv" % d):
            shutil.copy2(os.path.join(out, f), os.path.join(track_dir, f))
    shutil.copy2(os.path.join(track_dir, "raceline_%s.csv" % default_direction), os.path.join(track_dir, "raceline.csv"))
    shutil.copy2(os.path.join(track_dir, "raceline_mppi_%s.csv" % default_direction),
                 os.path.join(track_dir, "raceline_mppi.csv"))
    info = {
        "default_direction": default_direction,
        "note": "raceline.csv and raceline_mppi.csv are copies of the default direction. Pick a direction "
                "with DIRECTION=ccw|cw in watod-config.local.sh. Generated by the raceline generator "
                "(tools/wato_f1tenth_raceline_generator, make_raceline.py); do not edit by hand.",
        "source_map": source_map,
        "track_map": name,
        "drawn_edges": os.path.basename(lines_path) if lines_path else None,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "generator_commit": raceline_export.git_commit(HERE),
        "car": next(iter(summaries.values())).get("car"),
        "directions": {d: {k: s[k] for k in ("direction_name", "laptime_s", "length_m", "top_speed_mps",
                                              "min_speed_mps", "min_edge_clearance_m", "start_m")}
                       for d, s in summaries.items()},
    }
    with open(os.path.join(track_dir, "track_info.yaml"), "w") as f:
        yaml.safe_dump(info, f, sort_keys=False, default_flow_style=None)
    log("export: wrote %s (raceline_{%s}.csv, raceline_mppi_*.csv, raceline.csv = %s, track_info.yaml)"
        % (os.path.relpath(track_dir), ",".join(summaries), default_direction))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("map", help="map name in maps/ (or a race_tracks track name to copy in)")
    p.add_argument("--track", help="race_tracks folder to read the map from / export to (default: the map name)")
    p.add_argument("--export", action="store_true", help="write the results into the stack's race_tracks")
    p.add_argument("--directions", choices=["both", "ccw", "cw"], default="both")
    p.add_argument("--default-direction", choices=list(raceline_export.DIRECTIONS),
                   help="direction raceline.csv gets (default: keep the track's current direction, else ccw)")
    p.add_argument("--car", default=os.path.join("params", "wato_car.yaml"), help="car profile")
    p.add_argument("--stack-root", help="the wato_f1tenth checkout (default: found from tools/)")
    p.add_argument("--no-margin-retry", action="store_true", help="don't re-solve to hold the edge margin")
    args = p.parse_args(argv)

    stack_root = find_stack_root(args.stack_root)
    track = args.track or args.map
    import_map(args.map, track, stack_root)
    name, lines_path = prepare_track(args.map)
    start = None
    if lines_path:
        start = track_editor.load_lines(lines_path).get("start")

    result = centerline.extract(os.path.join(HERE, "maps", name))
    for m in result["messages"]:
        log("centerline: " + m)
    cl_path = os.path.join(HERE, "inputs", "tracks", name + ".csv")
    centerline.write_csv(cl_path, result["xyw"])
    xyw = result["xyw"]
    widths = xyw[:, 2] + xyw[:, 3]
    i = int(np.argmin(widths))

    car_path = args.car if os.path.isabs(args.car) else os.path.join(HERE, args.car)
    car = car_profile.load(car_path)
    log("car: " + json.dumps(car_profile.summary(car)))
    if stack_root:
        for w in car_profile.check_against_stack(car, stack_root):
            log("WARNING (car profile vs the stack): " + w)
    if widths[i] < car_profile.width_opt(car):
        raise SystemExit("ERROR: the track is %.2f m wide at (%.2f, %.2f) but the car needs %.2f m (width + 2 x "
                         "edge_margin_m). Widen it there (python track_editor.py maps/%s) or lower edge_margin_m "
                         "in %s to at most %.2f m." % (widths[i], xyw[i, 0], xyw[i, 1], car_profile.width_opt(car),
                                                       args.map, args.car, max((widths[i] - car["car"]["width_m"]) / 2, 0)))

    directions = ["ccw", "cw"] if args.directions == "both" else [args.directions]
    log("optimizing %s (%s), this takes a few minutes ..." % (name, " and ".join(directions)))
    summaries = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(directions)) as pool:
        futures = {d: pool.submit(optimize_direction, name, d, car, car_path, start, xyw,
                                  not args.no_margin_retry) for d in directions}
        for d, fut in futures.items():
            summary, _ = fut.result()
            if summary is not None:
                summaries[d] = summary
    if not summaries:
        raise SystemExit("ERROR: no direction could be optimized: see outputs/%s/<ccw|cw>/try_*/optimize.log" % name)

    for path in report.make_report(name, list(summaries)):
        log("report: " + os.path.relpath(path, HERE))

    log("\n%-4s %-18s %8s %9s %10s %s" % ("dir", "", "lap (s)", "top m/s", "clear (m)", "edge margin"))
    for d in directions:
        s = summaries.get(d)
        if s is None:
            log("%-4s %-18s %s" % (d, raceline_export.DIRECTIONS[d], "FAILED (see outputs/%s/%s/)" % (name, d)))
            continue
        flag = "ok" if not s["below_margin"] else "%d spot(s) under the margin" % len(s["below_margin"])
        log("%-4s %-18s %8.2f %9.2f %10.2f %s" % (d, s["direction_name"], s["laptime_s"], s["top_speed_mps"],
                                                  s["min_edge_clearance_m"], flag))
    if args.export:
        if not stack_root:
            raise SystemExit("ERROR: --export needs the stack (run from tools/ in wato_f1tenth, or --stack-root)")
        export(name, track, stack_root, summaries, args.default_direction, args.map, lines_path)
    else:
        log("\nNot exported. Look at the reports, then: python make_raceline.py %s%s --export"
            % (args.map, " --track %s" % track if track != args.map else ""))
    log("Before the car drives it: check the raceline in Foxglove (TRACK=%s) lies inside the walls." % track)
    return 0


if __name__ == "__main__":
    sys.exit(main())
