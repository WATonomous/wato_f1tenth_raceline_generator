# AGENTS.md

Notes for AI coding agents working in this repo (the WATonomous F1TENTH raceline generator). Plain
Markdown so any tool can read it. Read `Readme.md` for the full pipeline and `HAIRPIN_FIX_NOTES.md`
before touching `map_converter.ipynb` or `params/f110.ini`.

## Environment (set it up yourself)

Everything runs in the conda env `wato_raceline` (Python 3.8; the pinned packages have no newer wheels).

```bash
conda env list | grep -q '^wato_raceline ' || ./setup_env.sh   # create it once; safe to re-run
PY=$(conda run -n wato_raceline which python)                   # or: conda activate wato_raceline
```

- `conda run` does not pass stdin through: write scripts to a file, or call the env's `python` directly.
- Import this repo's modules (`track_editor`, `helper_funcs_glob`) from the repo root, or set `PYTHONPATH`.
- Never `pip install` into another env or the system Python for this repo; change `requirements.txt`
  and re-run `./setup_env.sh` instead.

## Running the pipeline headlessly

```bash
# 1. check a map, or build a drawn track from saved edges (no window)
$PY track_editor.py --check maps/<map>
$PY track_editor.py maps/<map> --no-gui
# 2. centerline: set MAP_NAME / TRACK_TYPE / EDIT_TRACK=False in the step 0 cell first
MPLBACKEND=Agg conda run -n wato_raceline jupyter nbconvert --to notebook --execute map_converter.ipynb \
  --output /tmp/map_converter_run.ipynb --ExecutePreprocessor.kernel_name=wato_raceline
# 3. raceline
MPLBACKEND=Agg $PY main_globaltraj_f110.py --map_name <map>_track   # (or <map> for a closed track)
```

Success prints `EXIT: Optimal Solution Found.` and `INFO: Estimated laptime`; the raceline is
`outputs/<map>/traj_race_cl-<time>.csv` (`x, y, v`, git-ignored). Use `--output` with nbconvert so test
runs don't rewrite the committed notebook; run `--inplace` only to refresh its saved outputs on purpose.

## The track editor

`track_editor.py` draws the track on a map that is not a clean ring. Its window needs a human; agents use
`--no-gui` and `--check`, or edit `maps/<map>_track_lines.json` (two closed lists of `[x, y]` points in
metres, map frame). Output map: free 254 inside the edges, 0 on them, 205 elsewhere; same size,
resolution, origin and thresholds as the original. Keep the original map for localization.

## Rules

- Never hand-edit generated files (`inputs/tracks/*.csv`, `maps/*_track.pgm`, `outputs/`); change the
  inputs or parameters and regenerate.
- When the optimizer reports `spline normals are crossed` or `local infeasibility`, use the symptom
  table in `HAIRPIN_FIX_NOTES.md` rather than guessing.
- The car's numbers in `params/f110.ini` (wheelbase 0.33 m, `delta_max` 0.52 rad, mass 3.74 kg) must match
  the real car. More margin from the track edges: raise `width_opt`.
- Commits use the human's git identity; never credit an agent in commits. Ask before pushing.
- The racelines the car uses live in the main stack repo (`src/world_modeling/race_tracks/`); see
  `agents/workflows/raceline.md` there for getting a raceline onto the car.
