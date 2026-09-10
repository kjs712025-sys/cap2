#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

export PYTHONUNBUFFERED=1

if [ ! -d .venv ]; then
  python3 -m venv --system-site-packages .venv
fi

source .venv/bin/activate

# Install dependencies only when requirements.txt has changed since the last
# successful install (tracked by a hash stamp). This keeps boot fast and, more
# importantly, lets the robot start with no internet: a transient PyPI failure
# at boot logs a warning instead of aborting the service into a restart loop.
# Force a re-install with:  rm .venv/.requirements.sha1
if [ "${ROBOT_SKIP_PIP:-0}" != "1" ] && [ -f requirements.txt ]; then
  req_hash="$(sha1sum requirements.txt | cut -d' ' -f1)"
  stamp=".venv/.requirements.sha1"
  if [ "$(cat "$stamp" 2>/dev/null || true)" != "$req_hash" ]; then
    if pip install -r requirements.txt; then
      echo "$req_hash" > "$stamp"
    else
      echo "WARN: pip install failed; starting with the packages already present" >&2
    fi
  fi
fi

exec python main.py
