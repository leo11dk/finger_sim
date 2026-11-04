#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$ROOT"
exec "$ROOT/.venv/bin/python" "$ROOT/run.py" "$@"
