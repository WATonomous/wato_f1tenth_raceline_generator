# Curvy / hairpin map fixes — handoff notes

Context for future sessions. This documents changes made to get a **dense, smooth raceline on
a tight, curvy F1TENTH map with a hairpin** (map: `e7_fifth_big_raceline`), and how to debug if
something breaks again. Read this before touching `map_converter.ipynb`, `params/f110.ini`, or
the stepsize/vehicle parameters.

## The original symptom

On a curvy map with a hairpin, the user had to inflate the `stepsize_opts` in `params/f110.ini`
to get past an `IOError("At least two spline normals are crossed …")`, which produced a **sparse,
crude raceline**. After fixing that, the `mintime` solver then reported
`EXIT: Converged to a point of local infeasibility`.

There were **three independent root causes**, fixed in order:

## Root cause 1 — jagged pixel centerline (map_converter.ipynb)

`map_converter.ipynb` builds the centerline by skeletonizing the occupancy grid (resolution
**0.05 m/px**) and walking it with DFS. The raw path is **pixel-quantized / staircase-jagged**,
~5 cm spacing. When the optimizer smooths it and checks spline normals, the noise makes adjacent
normals cross → the `IOError`. The old workaround was `transformed_data = transformed_data[::4]`
(crude downsample) plus inflating stepsizes — both make the line sparse.

## Root cause 2 — over-large track widths at the hairpin (map_converter.ipynb)

The distance-transform widths at the skeleton apex of the hairpin were **physically impossible**:
~1.21 m half-width (2.4 m full width) at a **0.72 m-radius** turn. When half-width > turn radius,
the inner track boundary **folds over itself**, which is the real trigger for the "normals
crossed" error. No amount of centerline smoothing fixes this (more smoothing actually *tightens*
the apex).

### Fix for causes 1 & 2 — `map_converter.ipynb`

Replaced the `transformed_data[::4]` cell (the cell just before the CSV-writing cell) with a
smoothing + resample + width-clamp step. New tunable constants live in the `MAP_NAME` /
`TRACK_WIDTH_MARGIN` cell near the top:

- `SMOOTHING = 1.0` — `scipy.interpolate.splprep` smoothing factor `s`. **Higher = smoother**
  (further from the raw skeleton). **Raise this if the "normals crossed" error still fires, or if
  the line still looks jagged** — more input points (e.g. the full ~1150-pt raw skeleton) need a
  larger `s`.
- `RESAMPLE_STEP = 0.1` — [m] target arc-length spacing of the resampled centerline.
- `WIDTH_CURV_FRAC = 0.9` — caps each half-width at `WIDTH_CURV_FRAC × local turn radius`, so a
  corner of radius R never claims more than 0.9·R of lateral room (prevents the boundary fold).
  **Lower this (e.g. 0.6) if the "normals crossed" error still fires at a hairpin.** `None`
  disables the clamp.

The cell fits a **periodic** cubic spline (`per=1`), resamples at uniform arc length, re-maps the
widths onto the new sampling, then applies the curvature clamp using the spline's exact
derivatives. It also prints diagnostics and shows a **before/after preview plot**.

**IMPORTANT: you must re-run `map_converter.ipynb` end-to-end to regenerate**
`inputs/tracks/<MAP_NAME>.csv` — it is not regenerated automatically.

## Root cause 3 — wrong vehicle geometry made mintime infeasible (params/f110.ini)

After the centerline was clean, `mintime` still failed with local infeasibility. Reason: the
`vehicle_params_mintime` had **wheelbase 0.55 m** (`wheelbase_front 0.275 + wheelbase_rear 0.275`)
— **double** the real F1TENTH value. The kinematic minimum turning radius is
`R_min = wheelbase / tan(delta_max)`. At 0.55 m / 0.4189 rad that's **1.24 m**, but the hairpin
needs **~0.69 m** → the car literally cannot steer that tight → solver (correctly) reports
infeasible. Note: `curvlim = 3.0` (→ 0.33 m) is *not* the binding limit; `delta_max` is.

### Fix — `params/f110.ini` `vehicle_params_mintime`

Corrected to the real F1TENTH / bayesrace values the ini already cites, plus the user's measured
steering:

- `wheelbase_front`: `0.275 → 0.15875`
- `wheelbase_rear`:  `0.275 → 0.17145`  (total wheelbase now **0.33 m**)
- `delta_max`:       `0.4189 → 0.52` rad  (user's measured max steer)

New `R_min = 0.3302 / tan(0.52) ≈ 0.58 m`, inside the 0.69 m hairpin → **mintime now converges.**

## Also changed — stepsize decoupling (params/f110.ini)

Key insight: **the exported raceline density is set by `stepsize_interp_after_opt` alone**, and is
decoupled from `stepsize_reg` (the optimization can run at ~46 coarse points and still export ~383
dense points). So you never need to inflate the output-facing stepsize.

`stepsize_opts` retuned (was `0.6 / 1.75 / 1.25`):
- `stepsize_prep = 0.3`
- `stepsize_reg = 1.25` — only needs to be small enough for faithful corners **while still
  passing the normal-crossing check** on this hairpin. Smaller values (≤1.0) re-trigger the
  crossing unless `WIDTH_CURV_FRAC` is lowered further.
- `stepsize_interp_after_opt = 0.15` — **this is what makes the exported line dense.**

## Root cause 4 — the centerline walk (map_converter.ipynb), found 2026-11-11

Even with clean widths, "normals crossed" can come from the **order** of the centerline points:

- The skeleton has short **spurs** (dead-end branches, e.g. near the hairpin at about (14.5, 3.2) m on
  E7). The depth-first walk goes out along a spur and back, or leaves the spur (or one stray pixel of a
  thick skeleton corner) for last and then **jumps** to it across the track. The periodic spline then fits
  a curve out and back across the track: the centerline came out 98 m long instead of 61 m.
- The walk starts at the leftmost point, which is always a **corner**. The optimizer joins the loop at
  its first point and copes badly with that seam inside a corner.

Whether a given map hit this was luck (the old `e7_fifth_big_raceline.csv` happened to dodge it).
Fixes, all in `map_converter.ipynb`: the skeleton cell strips spurs (`PRUNE_SPURS`, using
`track_editor.prune_spurs`); the cell after the walk keeps only the longest unbroken run of points
(drops strays visited out of order) and rotates the loop to start on its straightest stretch
(`ROTATE_TO_STRAIGHT`); `LEFT_START_Y = None` picks the start on the main skeleton loop automatically.
With these, `e7_fifth_big_raceline`, `e7_floor5_square` and the drawn `e7_fifth_big_real_track` all
optimize with the default knobs. Also fixed: `main_globaltraj_f110.py` overrode `--map_name` with a
hard-coded `MAP_NAME = "e7_fifth_big_raceline"`, so every run optimized that one map.

## What each knob does (quick reference)

| Symptom | Knob | Direction |
|---|---|---|
| "spline normals are crossed" error | centerline length / jumps: check `PRUNE_SPURS` and `ROTATE_TO_STRAIGHT` are on (converter) | first: a 1.5x-too-long centerline means a bad walk, not widths |
| " " | `WIDTH_CURV_FRAC` (converter) | lower (e.g. 0.6) |
| "local infeasibility" / "Restoration Failed" in one direction only | `s_reg` (ini, or `--s_reg`) | raise: `make_raceline.py` already retries 1 -> 3 -> 10 |
| raceline closer to the drawn edges than `edge_margin_m` | `s_reg` low (1, the default since 2026-11-11); `make_raceline.py` narrows the corridor at those spots | widen the track there if it still shows red circles |
| bound assertion `lb <= ub` from casadi | track narrower than car width + 2 x `edge_margin_m` | now caught before the solve with the location |
| " " (or jagged line) | `SMOOTHING` (converter) | raise |
| " " | `stepsize_reg` (ini) | raise |
| Exported raceline too sparse | `stepsize_interp_after_opt` (ini) | lower |
| mintime "local infeasibility" | `wheelbase_*`, `delta_max` (ini) | match real car; check `R_min = L/tan(delta_max)` vs tightest turn |
| mintime still infeasible | `mue` (0.3 = low grip), `t_delta` (steer-rate), `width_opt` (0.9) | raise `mue` / relax as appropriate |

## Also changed — real-map overlay in visualize_raceline.ipynb

The "OPTIONAL Overlay on the original map" section used to load `maps/{MAP_NAME}_RAW.pgm`
(`e7_fifth_big_raceline_RAW.pgm`), which does not exist. It now overlays the raceline on the
**real, un-massaged map** (the one that is NOT closed into a loop):

- New config constant in the `MAP_NAME` cell: `ORIG_MAP_NAME = "e7_fifth_big_real"`.
- The overlay cells recompute the meters→pixels transform from **that map's own yaml** (its
  `origin` + `resolution`), so the raceline lands correctly even if the original map's frame
  differs from the raceline map's. (Today they match: both `[-24.5, -11, 0]` @ `0.05`.)
- Saves `outputs/{MAP_NAME}_raceline_on_{ORIG_MAP_NAME}.png`.

The raceline CSV columns are `[x_m, y_m, velocity]` (3 cols) — that's why the transform
subtracts `[origin_x, origin_y, 0]`. To point at a different original map, just change
`ORIG_MAP_NAME` (needs `maps/<name>.pgm` + `maps/<name>.yaml`).

## Files changed

- `map_converter.ipynb` — smoothing + resample + curvature width-clamp cell (replaces `[::4]`);
  new constants `SMOOTHING`, `RESAMPLE_STEP`, `WIDTH_CURV_FRAC`; before/after preview.
- `params/f110.ini` — `stepsize_opts` retuned; `vehicle_params_mintime` wheelbase + `delta_max`
  corrected.
- `visualize_raceline.ipynb` — original-map overlay now targets `ORIG_MAP_NAME`
  (`e7_fifth_big_real`) and recomputes the transform from that map's yaml.

## How to run / verify

1. Re-run `map_converter.ipynb` end-to-end → regenerates `inputs/tracks/<MAP_NAME>.csv`.
   Check the before/after plot and the printed width-clamp / point-count diagnostics.
2. `python main_globaltraj_f110.py --map_name e7_fifth_big_raceline`
   - Success prints `INFO: Estimated laptime: …`.
   - Failure prints `EXIT: Converged to a point of local infeasibility` (see mintime rows above)
     or the `IOError` normals-crossed (see converter knobs above).
3. Inspect with `visualize_raceline.ipynb`.

## Notes / not-yet-done

- `veh_params.length` is still `0.568 m` (used only by the `check_traj` collision check, not the
  optimization). If the real car is shorter, lower it so that check isn't overly conservative.
- `opt_type` is `mintime` (`main_globaltraj_f110.py`). If mintime is ever too fragile on a new
  map, `mincurv` / `mincurv_iqp` are more robust geometric alternatives — **but note** the repo's
  `iqp_handler` call signature does **not** match the installed `trajectory_planning_helpers`
  version (it's missing `spline_len, psi, kappa, dkappa`), so `mincurv_iqp` would need a code fix
  before use in this env.
- `MAP_NAME` is currently hardcoded near the top of `main_globaltraj_f110.py` (overrides the
  `--map_name` CLI arg).
