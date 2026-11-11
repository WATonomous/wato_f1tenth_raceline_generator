# AGENTS.md

Notes for AI coding agents working in this repo (the WATonomous F1TENTH raceline generator). Plain
Markdown so any tool can read it. `Readme.md` has the full pipeline; read `HAIRPIN_FIX_NOTES.md` before
touching `centerline.py`, `map_converter.ipynb` or `params/f110.ini`.

**The only step that needs a person is drawing a track's edges** in `track_editor.py` (a window), and
only when the map is not already one clean ring. Agents run everything else and show the person the
reports before anything is exported.

## Environment (set it up yourself)

Everything runs in the conda env `wato_raceline` (Python 3.8; the pinned packages have no newer wheels).

```bash
conda env list | grep -q '^wato_raceline ' || ./setup_env.sh   # once, in the background; safe to re-run
PY=$(conda run -n wato_raceline which python)                   # or: conda activate wato_raceline
```

- `conda run` does not pass stdin through: write scripts to a file, or call the env's `python` directly.
- Run from the repo root (modules import each other: `track_editor`, `centerline`, `car_profile`, ...).
- Never `pip install` into another env or the system Python; change `requirements.txt` and re-run
  `./setup_env.sh`.

## The one command

```bash
$PY make_raceline.py <map>              # optimize both directions, write the reports (minutes: run it in the background)
$PY make_raceline.py <map> --export     # ... and write the files into the stack's race_tracks
```

`<map>` is a name in `maps/`, or a `race_tracks` track (its `map.*` is copied in on the first run, from the
stack checkout this repo sits in under `tools/`). It:

1. rebuilds `maps/<map>_track` from `maps/<map>_track_lines.json` (drawn edges), or uses the map if it is
   one clean ring, or **exits with code 2**: ask the person to run `python track_editor.py maps/<map>`;
2. writes the centerline `inputs/tracks/<name>.csv` (`centerline.py`, always counter-clockwise);
3. optimizes counter-clockwise and clockwise in parallel with the car profile `params/wato_car.yaml`;
   a failed solve is retried with more smoothing (`s_reg` 1, 3, 10); spots closer to the real track edges
   than `edge_margin_m` get the corridor narrowed on that side and are re-solved (up to twice);
4. writes `outputs/<name>/report_ccw.png`, `report_cw.png`, `report_compare.png` (`report.py`);
5. with `--export`: `raceline_{ccw,cw}.csv`, `raceline_mppi_{ccw,cw}.csv`, `raceline.csv` and
   `raceline_mppi.csv` (the default direction: the track's current one unless `--default-direction`)
   and `track_info.yaml` into `race_tracks/tracks/<track>/`.

Read the final table and `outputs/<name>/<dir>/summary_<dir>.json` (lap time, top speed and where,
clearance, `below_margin` spots, `solver.attempts`). Logs: `outputs/<name>/<dir>/try_*/optimize.log`.

## Pieces (when you need one step)

| Script | Does |
| --- | --- |
| `track_editor.py maps/<map>` | the drawing window (people only); `--no-gui` rebuilds from saved edges, `--check` checks a map is one ring |
| `centerline.py maps/<name>` | centerline + widths -> `inputs/tracks/<name>.csv` |
| `main_globaltraj_f110.py --map_name <name> --direction ccw\|cw` | one optimization; writes `outputs/<name>/<dir>/raceline_<dir>.csv`, `raceline_mppi_<dir>.csv`, `summary_<dir>.json`. Use `MPLBACKEND=Agg` |
| `report.py <name>` | the report pages from existing outputs |
| `car_profile.py` | maps `params/wato_car.yaml` onto the optimizer's parameters |
| notebooks | `map_converter.ipynb`, `sanity_check.ipynb`, `visualize_raceline.ipynb`: viewers over the same code |

## Rules

- Never draw, invent or edit track edges (`maps/*_track_lines.json`) yourself; that is the person's call.
- Never hand-edit generated files (`inputs/tracks/*.csv`, `maps/*_track.*`, `outputs/`, exported racelines).
- Change the car in `params/wato_car.yaml`, with a source for each number; `f110.ini` is solver settings.
  Keep `limits` at or below the stack's (`make_raceline.py` warns when they differ from `vesc.yaml`).
- Measured values (grip, mass, drag): `docs/measuring_the_car.md`. Until measured, `mue` stays 0.3.
- When the optimizer fails, use the symptom table in `HAIRPIN_FIX_NOTES.md` rather than guessing.
- Commits use the human's git identity; never credit an agent in commits. Ask before pushing.
- Getting the raceline onto the car: `agents/workflows/raceline.md` in the stack repo.
