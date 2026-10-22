#!/usr/bin/env bash
# Detached, inhibited supervisor; exactly one simulator process, no sudo.
set -eu
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON=/home/kean/miniconda3/envs/scg/bin/python
OUTPUT="${2:-$SCRIPT_DIR/independent_evaluation_20_319}"

if [[ "${1:-}" == "--worker" ]]; then
    set +e
    env PYTHONMALLOC=malloc MALLOC_ARENA_MAX=1 PYTHONUNBUFFERED=1 \
        "$PYTHON" "$SCRIPT_DIR/independent_evaluation.py" --mode run --output "$OUTPUT"
    code=$?
    # Catch failures before Python's handler (import failure, OOM kill, etc.).
    "$PYTHON" - "$OUTPUT" "$code" <<'PY'
import json,os,sys,time
from pathlib import Path
output=Path(sys.argv[1]);code=int(sys.argv[2]);path=output/'status.json'
status=json.loads(path.read_text()) if path.exists() else {}
if code or status.get('state')!='SUCCESS':
    status.update(state='FAILURE',supervisor_exit_code=code,
                  error=status.get('error','Worker exited before completing all reports'),
                  updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    temporary=path.with_suffix('.json.tmp')
    with temporary.open('w') as stream:
        json.dump(status,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    os.replace(temporary,path)
    (output/'FAILURE').write_text(str(status)+'\nDurable checkpoints retained; resume with the launcher.\n')
PY
    exit "$code"
fi

OUTPUT="${1:-$SCRIPT_DIR/independent_evaluation_20_319}"
mkdir -p -- "$OUTPUT"
OUTPUT="$(cd -- "$OUTPUT" && pwd)"
exec 9>"$OUTPUT/launch.lock"
flock -n 9
if [[ -f "$OUTPUT/supervisor.pid" ]]; then
    read -r existing < "$OUTPUT/supervisor.pid"
    args="$(ps -p "$existing" -o args= 2>/dev/null || true)"
    if [[ "$args" == *systemd-inhibit* && "$args" == *launch_independent_evaluation.sh* ]]; then
        printf 'Already running: inhibitor/supervisor PID %s\n' "$existing"
        exit 0
    fi
fi
if [[ -f "$OUTPUT/worker.pid" ]]; then
    read -r existing < "$OUTPUT/worker.pid"
    args="$(ps -p "$existing" -o args= 2>/dev/null || true)"
    if [[ "$args" == *independent_evaluation.py* ]]; then
        printf 'Already running: worker PID %s\n' "$existing"
        exit 0
    fi
fi
"$PYTHON" - "$OUTPUT" <<'PY'
import json,sys
from pathlib import Path
output=Path(sys.argv[1])
assert json.loads((output/'smoke_validation.json').read_text())['success'], 'Smoke test must pass before launch'
status=output/'status.json'
if status.exists() and json.loads(status.read_text()).get('state')=='SUCCESS':
    raise SystemExit('Evaluation already complete; no relaunch needed')
PY
# This inhibitor encloses calibration, evaluation, statistics and report generation.
# nohup + a new session detach the entire job from Codex and its terminal.
nohup setsid systemd-inhibit --what=sleep --mode=block --who=StaleGuard-Independent \
    --why='300-scenario independent UAV evaluation, seeds 20-319' \
    /bin/bash "$SCRIPT_DIR/launch_independent_evaluation.sh" --worker "$OUTPUT" \
    </dev/null >>"$OUTPUT/run.log" 2>&1 9>&- &
supervisor=$!
printf '%s\n' "$supervisor" > "$OUTPUT/supervisor.pid"
printf 'Detached inhibitor/supervisor PID: %s\nLog: %s/run.log\n' "$supervisor" "$OUTPUT"
