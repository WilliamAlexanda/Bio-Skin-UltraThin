#!/bin/zsh
# Double-click in Finder (or run in a terminal) to start the visualizer.
# Creates .venv on first run. Extra arguments are passed on, e.g. ./run.command --demo
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
    PY=""
    for c in python3.13 python3.12 python3.11 python3.10 python3; do
        if command -v $c >/dev/null && $c -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
            PY=$c
            break
        fi
    done
    if [ -z "$PY" ]; then
        echo "Python >= 3.9 not found. Install it first, e.g.: brew install python@3.12"
        exit 1
    fi
    $PY -m venv .venv && .venv/bin/pip install -r requirements.txt || exit 1
fi
exec .venv/bin/python tactile_viz.py "$@"
