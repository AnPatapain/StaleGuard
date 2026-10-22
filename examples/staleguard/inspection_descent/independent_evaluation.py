"""Resumable, single-worker independent Experiment B: seeds 20–319.

Scientific implementations are imported unchanged. Calibration calls the official
planner; its physics list is replaced with a streaming dwell accumulator, and
durably checkpointed candidates are replayed to the planner without simulation.
"""
import os
for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
                 "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMBA_NUM_THREADS"):
    os.environ[variable] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ.setdefault("MPLCONFIGDIR", "/tmp/staleguard-independent-matplotlib")

import argparse
from dataclasses import asdict
import ctypes
import fcntl
import gc
import hashlib
import json
from pathlib import Path
import pickle
import signal
import sys
import time
import traceback
from unittest.mock import patch

import numpy as np
import staleguard_benefit as b
import max_dwell_calibration as calibration
from diagnostics.validate_official_calibration import check_memory

ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = ROOT / "independent_evaluation_20_319"
POLICIES = ("No Recovery", "Always-Recover", "StaleGuard", "Use-Time Recheck", "FBS", "Fixed TTL (0.3s)")
SEEDS = tuple(range(20, 320))
DELAY_STEPS = tuple(range(0, 51, 5))
LIBC = ctypes.CDLL(None)


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def journal(path):
    """Recover only complete, newline-committed records after an interruption."""
    if not path.exists():
        return []
    rows, committed = [], 0
    with path.open("rb") as stream:
        for line in stream:
            if not line.endswith(b"\n"):
                break
            rows.append(json.loads(line))
            committed += len(line)
    if path.stat().st_size != committed:
        with path.open("r+b") as stream:
            stream.truncate(committed)
    return rows


def append_record(path, row):
    with path.open("a") as stream:
        stream.write(json.dumps(row, allow_nan=False)+"\n")
        stream.flush(); os.fsync(stream.fileno())


def release():
    gc.collect()
    LIBC.malloc_trim(0)


def status(output, **fields):
    path = output / "status.json"
    prior = json.loads(path.read_text()) if path.exists() else {}
    prior.update(fields, updated_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    atomic_json(path, prior)


def initialize(output):
    output.mkdir(parents=True, exist_ok=True)
    protocol = dict(seeds=list(SEEDS), delays_steps=list(DELAY_STEPS), policies=list(POLICIES),
        fixed_ttl_steps=15, deadline_after_approval_s=4.2, ctrl_hz=50, physics_hz=1000,
        calibration="Unchanged official D=0-only maximum dwell; first candidate on ties; require 180ms",
        fbs=dict(alpha=.1, epsilon_m_per_sample=1e-12, initial_volatility=0, maximum_ttl_steps=210),
        bootstrap=dict(replicates=10000, random_seed=1729, confidence=.95,
                       unit="feasible scenario; all delays and policies resampled together; paired percentile intervals"),
        paired_inference="Exact two-sided scenario-level sign tests; ties excluded; Holm adjustment over 180 aggregate/per-delay comparisons (5 baselines x 3 metrics x 12 strata)",
        feasibility_ci="Wilson binomial 95% interval over exactly 300 attempted scenarios",
        smoke_seeds=[20, 21], smoke_delays_steps=[0, 15, 20],
        reuse="Always-Recover D=0 equals original; positive delays share the nominal fallback. Fixed TTL uses original or nominal fallback; native checks at D=.3/.4 for every feasible scene.",
        test_adaptation="None; smoke scenes remain in final cohort and measured checkpoints are retained")
    path = output / "protocol.json"
    if path.exists():
        assert json.loads(path.read_text()) == protocol, "Locked protocol changed"
    else:
        atomic_json(path, protocol)
    sources = list(ROOT.glob("*.py"))+list((ROOT / "model").glob("*.py"))
    sources += [ROOT / "diagnostics/validate_official_calibration.py", ROOT / "launch_independent_evaluation.sh"]
    development = [ROOT / "calibrated_scenarios.json"]
    development += [p for p in (ROOT / "result").iterdir() if p.is_file()]
    for folder in ("use_time_recheck", "fbs", "fixed_ttl"):
        development += [p for p in (ROOT / "results" / folder).glob("*.json*")]
    manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sources+development}
    path = output / "preserved_files.json"
    if path.exists():
        assert json.loads(path.read_text()) == manifest, "Scientific code or development artifacts changed"
    else:
        atomic_json(path, manifest)
    assert b.CTRL_FREQ_HZ == 50 and b.PYB_FREQ_HZ == 1000 and b.MISSION_DEADLINE_S == 4.2
    assert (b.fbs.ALPHA, b.fbs.EPSILON, b.fbs.MAX_TTL_STEPS) == (.1, 1e-12, 210)


class PhysicsSummary:
    """Constant-memory physics journal sink, using the existing predicates."""
    def __init__(self, scenario):
        self.scenario = scenario
        self.state = np.zeros(12)
        self.scan_x = b.inspection_x(scenario)
        self.entered = None
        self.longest = 0
        self.last_tick = -1
        self.first_unsafe_time_s = None
        self.digest = hashlib.sha256()

    def append(self, row):
        tick, position = row
        assert tick == self.last_tick+1
        self.last_tick = tick
        self.state[[b.STATE_X, b.STATE_Y, b.STATE_Z]] = position
        if b.in_inspection_region(self.state, self.scenario, self.scan_x):
            if self.entered is None:
                self.entered = tick
            self.longest = max(self.longest, tick-self.entered)
        else:
            self.entered = None
        if self.first_unsafe_time_s is None and b.unsafe(self.state, self.scenario):
            self.first_unsafe_time_s = tick/b.PYB_FREQ_HZ
        self.digest.update(np.asarray(tick).tobytes())
        self.digest.update(np.asarray(position).tobytes())
        if tick and tick % b.PYB_FREQ_HZ == 0:
            check_memory()


class ControlSummary:
    """Stream a trace digest and retain only approval/dispatch observations."""
    def __init__(self, scenario, approval, dispatch):
        self.scenario, self.approval, self.dispatch = scenario, approval, dispatch
        self.last_step = -1
        self.approval_state = self.dispatch_state = None
        self.first_descent_step = None
        self.digest = hashlib.sha256()

    def append(self, row):
        step, state, reference = row
        assert step == self.last_step+1
        self.last_step = step
        for value in row:
            self.digest.update(np.asarray(value).tobytes())
        if step == self.approval:
            self.approval_state = state.copy()
        if step == self.dispatch:
            self.dispatch_state = state.copy()
        if step and reference[b.STATE_Z] == self.scenario.low_height and self.first_descent_step is None:
            self.first_descent_step = step-1


class CheckpointPause(Exception):
    """Intentional smoke-test interruption after a durable checkpoint."""


def seed_directory(output, seed):
    assert seed in SEEDS
    path = output / "scenarios" / str(seed)
    path.mkdir(parents=True, exist_ok=True)
    return path


def calibrate(output, seed, pause_after=None):
    directory = seed_directory(output, seed)
    selected_path = directory / "calibration.json"
    if selected_path.exists():
        return json.loads(selected_path.read_text())
    path = directory / "calibration_candidates.jsonl"
    saved = journal(path)
    scenario = b.generate_scenario(seed)
    index, new_count, last_dwell = 0, 0, None
    original_capture, original_actual = calibration.capture_approval, calibration.run_actual

    def capture(candidate):
        if index < len(saved):
            assert asdict(candidate) == saved[index]["command"]
            return None  # The official planner receives its saved outcome below.
        check_memory()
        snapshot = original_capture(candidate)
        release(); check_memory()
        return snapshot

    def actual(candidate, snapshot, **kwargs):
        nonlocal last_dwell
        if index < len(saved):
            row = saved[index]["outcome"]
            last_dwell = row["best_dwell_ticks"]
            return (row["inspection_completed"], row["unsafe_entry"],
                    row["recovery_started"], row["safe_completion"])
        check_memory()
        summary = PhysicsSummary(candidate)
        kwargs["physics_trace"] = summary
        try:
            result = original_actual(candidate, snapshot, **kwargs)
            last_dwell = summary.longest
            assert bool(summary.first_unsafe_time_s is not None) == result[1]
            return result
        finally:
            release()

    def record(rank, candidate, outcome):
        nonlocal index, new_count
        row = dict(rank=rank, command=asdict(candidate), outcome=outcome)
        if index < len(saved):
            assert row == saved[index]
        else:
            append_record(path, row)
            saved.append(row)
            new_count += 1
            status(output, phase="calibration", current_seed=seed, candidate_rank=rank)
            if rank % 20 == 0:
                print(f"Calibration seed={seed}: {rank} candidates checkpointed", flush=True)
        index += 1
        if pause_after is not None and new_count >= pause_after:
            raise CheckpointPause()

    # The official selection loop, tie rule, feasibility test and grid are intact.
    with patch.object(calibration, "capture_approval", capture), \
         patch.object(calibration, "run_actual", actual), \
         patch.object(calibration, "inspection_dwell_ticks", side_effect=lambda trace, candidate: last_dwell):
        command = calibration.plan_max_dwell_command(scenario, on_candidate=record)
    assert index == len(saved)
    safe = [r for r in saved if not r["outcome"]["unsafe_entry"]]
    best = max(safe, key=lambda r: r["outcome"]["best_dwell_ticks"]) if safe else None
    best_dwell = best["outcome"]["best_dwell_ticks"] if best else None
    expected = best["command"] if best is not None and best_dwell >= round(b.SCAN_DWELL_S*b.PYB_FREQ_HZ) else None
    assert (asdict(command) if command is not None else None) == expected
    row = dict(seed=seed, generated_scenario=asdict(scenario), command=expected,
               best_safe_dwell_ticks=best_dwell, candidates=len(saved),
               selected_rank=best["rank"] if command is not None else None)
    atomic_json(selected_path, row)
    print(f"Calibration seed={seed}: {'feasible' if command is not None else 'infeasible'}; "
          f"best safe dwell={best_dwell} ms; candidates={len(saved)}", flush=True)
    return row


def approval_snapshot(directory, scenario):
    path = directory / "approval.pkl"
    if path.exists():
        with path.open("rb") as stream:
            return pickle.load(stream)
    check_memory()
    snapshot = b.capture_approval(scenario)
    temporary = path.with_suffix(".tmp")
    with temporary.open("wb") as stream:
        pickle.dump(snapshot, stream); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)
    release(); check_memory()
    return snapshot


def reception(directory, snapshot, quad, pid):
    path = directory / "reception.json"
    if path.exists():
        return json.loads(path.read_text())
    check_memory()
    prediction, elapsed = b.predict_at_reception(snapshot, quad, pid)
    row = dict(prediction=prediction, computation_s=elapsed)
    atomic_json(path, row)
    release(); check_memory()
    return row


def clean_event(event):
    event = event.copy()
    current = event.pop("snapshot", None)
    if current is not None:
        event["snapshot_time_s"] = current.approval_time
        event["snapshot_state"] = current.state.tolist()
        event["snapshot_horizon_steps"] = current.horizon_steps
        event["snapshot_low_steps"] = current.low_steps
    return {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in event.items()}


def native_flight(seed, scenario, snapshot, steps, policy, received, quad, pid):
    approval = round(snapshot.approval_time*b.CTRL_FREQ_HZ)
    dispatch = approval+steps
    deadline = round((snapshot.approval_time+b.MISSION_DEADLINE_S)*b.CTRL_FREQ_HZ)
    controls, physics = ControlSummary(scenario, approval, dispatch), PhysicsSummary(scenario)
    kwargs, event, expiry_checks = {}, {}, []
    recover, recovery = False, dispatch
    if policy == "Always-Recover":
        recover, recovery = approval < dispatch, approval
    elif policy == "StaleGuard":
        kwargs["ttl_prediction"] = (received["prediction"], received["computation_s"])
    elif policy == "Use-Time Recheck":
        kwargs.update(use_time_check=lambda current: b.predict_immediate(current, b.LightweightPredictiveModel(quad, pid)),
                      recheck_log=event)
    elif policy == "FBS":
        kwargs["fbs_log"] = event
    elif policy == "Fixed TTL (0.3s)":
        kwargs.update(fixed_ttl_steps=15, fixed_ttl_log=event)
    original_expiry = b.ttl_expired_at_dispatch
    def checked_expiry(prediction, waited):
        assert controls.last_step == dispatch and waited == steps
        expiry_checks.append(dispatch)
        return original_expiry(prediction, waited)
    with patch.object(b, "ttl_expired_at_dispatch", side_effect=checked_expiry):
        outcome = b.run_actual(scenario, snapshot, dispatch, recovery, deadline, recover,
            trace=controls, physics_trace=physics, recovery_mode="safety-only", **kwargs)
    reached = controls.last_step > dispatch
    assert outcome[3] == (outcome[0] and not outcome[1])
    assert bool(physics.first_unsafe_time_s is not None) == outcome[1]
    np.testing.assert_array_equal(controls.approval_state, snapshot.state)
    prediction_time, ready, availability_clock = None, None, None
    if policy == "StaleGuard":
        prediction_time = received["computation_s"]
        ready = prediction_time <= steps/b.CTRL_FREQ_HZ if reached else None
        assert expiry_checks == ([dispatch] if reached and ready else [])
        expected = reached and ready and original_expiry(received["prediction"], steps)
        assert outcome[2] == expected
        availability_clock = "scheduled dispatch"
    elif policy == "Use-Time Recheck" and reached:
        prediction_time = event["computation_s"]
        decision = event["decision_step"]
        assert decision == b.first_step_after_check(dispatch, prediction_time)
        current = event["snapshot"]
        np.testing.assert_array_equal(current.state, controls.dispatch_state)
        assert current.approval_time == dispatch/b.CTRL_FREQ_HZ
        assert current.low_steps == snapshot.low_steps
        assert current.horizon_steps == round(b.MEASURE_SECONDS*b.CTRL_FREQ_HZ)-dispatch
        ready = event["decision_applied_step"] is not None
        assert outcome[2] == (ready and event["prediction"] != "SAFE")
        assert controls.first_descent_step == (decision if ready and event["prediction"] == "SAFE" else None)
        availability_clock = "first post-computation control step (command withheld until then)"
    elif policy == "FBS":
        prediction_time = event["computation_s"]
        ready = event.get("ready")
        np.testing.assert_array_equal(event["observation_state"], snapshot.state)
        assert event["observation_steps"] == list(range(approval+1))
        assert outcome[2] == (event.get("recover", False))
        availability_clock = "scheduled dispatch"
    elif policy == "Fixed TTL (0.3s)":
        assert event["approval_step"] == approval and event["ttl_steps"] == 15
        assert outcome[2] == (reached and steps > 15)
    if policy != "Use-Time Recheck":
        assert controls.first_descent_step == (dispatch if reached and not outcome[2] else None)
    return dict(seed=seed, delay_steps=steps, delay_s=steps/b.CTRL_FREQ_HZ, policy=policy,
        outcome=list(outcome), dispatch_step=dispatch, approval_step=approval, deadline_step=deadline,
        dispatch_reached=reached, prediction_computation_s=prediction_time, prediction_ready=ready,
        availability_clock=availability_clock, event=clean_event(event), execution_kind="native",
        control_sha256=controls.digest.hexdigest(), physics_sha256=physics.digest.hexdigest(),
        last_control_step=controls.last_step, last_physics_tick=physics.last_tick,
        first_unsafe_time_s=physics.first_unsafe_time_s)


def reused_flight(seed, snapshot, steps, policy, reference):
    row = reference.copy()
    approval = round(snapshot.approval_time*b.CTRL_FREQ_HZ)
    dispatch = approval+steps
    reached = row["last_control_step"] > dispatch
    if policy == "Always-Recover":
        fallback = steps > 0 and row["last_control_step"] > approval
        event = dict(fallback_step=approval if fallback else None,
                     application_step=dispatch if not fallback and reached else None)
    else:
        fallback = reached and steps > 15
        event = dict(approval_step=approval, ttl_steps=15, expiry_step=approval+15)
        if reached:
            event.update(dispatch_step=dispatch, waited_steps=steps, expired=fallback,
                         fallback_step=dispatch if fallback else None,
                         application_step=None if fallback else dispatch)
    outcome = list(row["outcome"])
    outcome[2] = fallback
    row.update(seed=seed, delay_steps=steps, delay_s=steps/b.CTRL_FREQ_HZ, policy=policy,
               dispatch_step=dispatch, dispatch_reached=reached, outcome=outcome, event=event,
               execution_kind="verified_reference_reuse",
               reused_from=dict(policy=reference["policy"], delay_steps=reference["delay_steps"]))
    return row


def evaluate(output, seed, calibrated, quad, pid, delays=DELAY_STEPS, force_native=False, pause_after=None):
    if calibrated["command"] is None:
        return
    directory = seed_directory(output, seed)
    scenario = b.Scenario(**calibrated["command"])
    snapshot = approval_snapshot(directory, scenario)
    received = reception(directory, snapshot, quad, pid)  # Before enumerating actual D.
    path = directory / "flights.jsonl"
    rows = journal(path)
    completed = {(r["delay_steps"], r["policy"]): r for r in rows}
    assert len(completed) == len(rows)
    added = 0
    for steps in delays:
        for policy in POLICIES:
            key = steps, policy
            if key in completed:
                continue
            check_memory()
            reference = None
            if policy == "Always-Recover":
                reference = completed.get((0, "No Recovery")) if steps == 0 else completed.get((5, policy))
            elif policy == "Fixed TTL (0.3s)":
                reference = completed.get((steps, "No Recovery")) if steps <= 15 else completed.get((5, "Always-Recover"))
            native = force_native or reference is None or (policy == "Fixed TTL (0.3s)" and steps in (15, 20))
            if native:
                row = native_flight(seed, scenario, snapshot, steps, policy, received, quad, pid)
                if reference is not None:
                    expected = reused_flight(seed, snapshot, steps, policy, reference)
                    for field in ("outcome", "control_sha256", "physics_sha256"):
                        assert row[field] == expected[field], (seed, steps, policy, field)
            else:
                row = reused_flight(seed, snapshot, steps, policy, reference)
            append_record(path, row)
            completed[key] = row
            added += 1
            status(output, phase="evaluation", current_seed=seed, delay_s=steps/50,
                   current_policy=policy, seed_flights_completed=len(completed))
            release(); check_memory()
            if pause_after is not None and added >= pause_after:
                raise CheckpointPause()
        print(f"Evaluation seed={seed}, D={steps/50:.1f}s: six policies checkpointed", flush=True)


def warmup():
    check_memory()
    quad, pid = b.load_scg_platform_params()
    current = b.capture_approval(b.generate_scenario(0))
    b.predict(current, b.LightweightPredictiveModel(quad, pid), delays=b.DELAY_GRID)
    b.predict_immediate(current, b.LightweightPredictiveModel(quad, pid))
    del current
    release(); check_memory()
    return quad, pid


def smoke(output, quad, pid):
    directory = seed_directory(output, 20)
    candidates = directory / "calibration_candidates.jsonl"
    if not candidates.exists():
        try:
            calibrate(output, 20, pause_after=1)
        except CheckpointPause:
            pass
    prefix = candidates.read_bytes()
    calibrated = [calibrate(output, seed) for seed in (20, 21)]
    assert candidates.read_bytes().startswith(prefix)
    eligible = [row for row in calibrated if row["command"] is not None]
    assert eligible, "Smoke scenes infeasible; stop without changing the locked cohort or parameters"
    first = eligible[0]
    flights = seed_directory(output, first["seed"]) / "flights.jsonl"
    if not flights.exists():
        try:
            evaluate(output, first["seed"], first, quad, pid, delays=(0, 15, 20),
                     force_native=True, pause_after=1)
        except CheckpointPause:
            pass
    first_record = journal(flights)[0]
    for row in eligible:
        evaluate(output, row["seed"], row, quad, pid, delays=(0, 15, 20), force_native=True)
    assert journal(flights)[0] == first_record
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
              for row in eligible for p in seed_directory(output, row["seed"]).glob("*") if p.is_file()}
    for row in eligible:
        evaluate(output, row["seed"], row, quad, pid, delays=(0, 15, 20), force_native=True)
    assert before == {name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in before}
    initialize(output)
    summary = dict(success=True, seeds=[20, 21], feasible=[r["seed"] for r in eligible],
                   policies=list(POLICIES), delays_steps=[0, 15, 20],
                   calibration_resume_verified=True, flight_resume_verified=True,
                   completed_work_unchanged_on_resume=True, development_preserved=True,
                   native_flights=18*len(eligible), parameters_tuned=False)
    atomic_json(output / "smoke_validation.json", summary)
    status(output, state="SMOKE_PASSED", phase="ready_for_detached_launch")
    print("PASS: new-scene six-policy smoke, calibration/flight resume, and development preservation.", flush=True)


def run(output, quad, pid):
    assert json.loads((output / "smoke_validation.json").read_text())["success"]
    calibrated = []
    # Freeze all 300 D=0 selections before full operational evaluation.
    for seed in SEEDS:
        calibrated.append(calibrate(output, seed))
        status(output, calibration_completed=len(calibrated),
               calibration_feasible=sum(r["command"] is not None for r in calibrated))
    atomic_json(output / "calibrated_scenarios.json", {str(r["seed"]): r for r in calibrated})
    for row in calibrated:
        evaluate(output, row["seed"], row, quad, pid)
    initialize(output)
    release(); check_memory()
    status(output, phase="reporting")
    from independent_report import generate_report
    generate_report(output)
    initialize(output)
    status(output, state="SUCCESS", phase="complete", completed_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    (output / "SUCCESS").write_text("Completed all 300 calibration attempts and all feasible six-policy flights and reports.\n")
    print("SUCCESS: independent evaluation and all final reports complete.", flush=True)


def interrupted(signum, frame):
    raise InterruptedError(f"Interrupted by signal {signum}; durable checkpoints retained")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mode", choices=("smoke", "run", "report"), required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # One global simulation lock, including development validation tools.
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            initialize(output)
            for sig in (signal.SIGTERM, signal.SIGINT):
                signal.signal(sig, interrupted)
            status(output, state="RUNNING", mode=args.mode, pid=os.getpid())
            failure = output / "FAILURE"
            if failure.exists():
                append_record(output / "failure_history.jsonl", dict(previous_failure=failure.read_text()))
                failure.unlink()
            (output / "worker.pid").write_text(str(os.getpid())+"\n")
            if args.mode == "report":
                from independent_report import generate_report
                generate_report(output)
                status(output, state="SUCCESS", phase="complete")
            else:
                quad, pid = warmup()
                {"smoke": smoke, "run": run}[args.mode](output, quad, pid)
        except BaseException as exc:
            status(output, state="FAILURE", error=str(exc), exception_type=type(exc).__name__)
            (output / "FAILURE").write_text(f"{type(exc).__name__}: {exc}\nResume uses durable checkpoints.\n")
            traceback.print_exc()
            raise


if __name__ == "__main__":
    main()
