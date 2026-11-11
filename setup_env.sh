#!/usr/bin/env bash
# Create (or update) the conda environment for the raceline generator.
#
#   ./setup_env.sh              env named wato_raceline
#   ./setup_env.sh <name>       another name
#
# Python 3.8: the pinned packages in requirements.txt (numpy 1.18, scipy 1.3, matplotlib 3.3,
# casadi 3.5) only have wheels up to 3.8. Also installs Jupyter (to run the notebooks, including
# headless with nbconvert) and registers a Jupyter kernel with the same name. Safe to re-run.
set -euo pipefail

ENV_NAME="${1:-wato_raceline}"
REPO_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"

command -v conda >/dev/null || { echo "conda not found: install Miniconda first"; exit 1; }

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "== updating conda env '$ENV_NAME'"
else
  echo "== creating conda env '$ENV_NAME' (Python 3.8)"
  # tk gives matplotlib its window backend (track_editor.py)
  conda create -y -n "$ENV_NAME" python=3.8 tk
fi

run() { conda run --no-capture-output -n "$ENV_NAME" "$@"; }

echo "== installing requirements"
run python -m pip install --upgrade "pip<25" wheel
run python -m pip install -r "$REPO_DIR/requirements.txt"
run python -m pip install notebook nbconvert ipykernel
run python -m ipykernel install --user --name "$ENV_NAME" --display-name "Python ($ENV_NAME)"

echo "== checking"
run python -c "import tkinter, numpy, scipy, skimage, matplotlib, yaml, casadi, trajectory_planning_helpers; \
print('ok: python', __import__('sys').version.split()[0], 'numpy', numpy.__version__, 'matplotlib', matplotlib.__version__)"

echo
echo "Done. Use it with: conda activate $ENV_NAME"
echo "In Jupyter, pick the kernel 'Python ($ENV_NAME)'."
