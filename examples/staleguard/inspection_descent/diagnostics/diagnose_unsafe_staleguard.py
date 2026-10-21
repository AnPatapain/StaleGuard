"""Replay only the three saved unsafe Experiment B StaleGuard flights.

No experiment inputs are changed. Extra RK4 evaluations expose the model path
between its existing integration endpoints, without advancing the model or PID.
Run serially in the scg environment; output must be a new directory.
"""

import argparse
import fcntl
import gc
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]

# Reuse the existing thread limits, memory guard, and complete-trace checksum.
from validate_official_calibration import check_memory, trace_hash
from verify_feasibility import intervals
import numpy as np
import staleguard_benefit as b
from experiment import MODEL_FREQ_HZ
from model.common import STATE_SIZE, STATE_X, STATE_Y, STATE_Z, capture_pid_state
from model.lightweight import rk4
from staleguard import build_nominal_checkpoints


def model_trace(snapshot, delay, quad_params, pid_params):
    """Trace the unchanged 100 Hz model, with diagnostic samples every 1 ms."""
    model = b.LightweightPredictiveModel(quad_params, pid_params)
    ctrl, obs, info, reference = model.start(snapshot)
    dispatch = round(delay * b.CTRL_FREQ_HZ)
    nominal_safe, checkpoints = build_nominal_checkpoints(
        model, ctrl, obs, info, snapshot, b.DELAY_GRID,
    )
    assert nominal_safe
    checkpoint = checkpoints[dispatch]
    ctrl, obs, info, reference = model.start(snapshot)
    model.set_reference(ctrl, b.apply_descent_command(
        reference, dispatch, snapshot.low_steps, snapshot.low_height,
    ))
    states = [obs.copy()]
    p = quad_params
    substep_ticks = round(model.dt * b.PYB_FREQ_HZ)
    for step in range(snapshot.horizon_steps):
        if step == dispatch:
            # The direct replay must match StaleGuard's actual checkpoint path.
            np.testing.assert_array_equal(obs, checkpoint.obs)
            pid = capture_pid_state(ctrl)
            for name in vars(pid):
                np.testing.assert_array_equal(getattr(pid, name), getattr(checkpoint.pid_state, name))
        action = ctrl.select_action(obs, info)
        next_obs, _, next_info = model.step(action, obs, info)
        internal = obs.copy()
        for _ in range(model.substeps):
            # Each sample starts at the original 10 ms integration endpoint.
            # These partial RK4 evaluations never feed back into the rollout.
            for tick in range(1, substep_ticks + 1):
                sample = rk4(
                    internal, action, tick / b.PYB_FREQ_HZ,
                    p.mass, p.inertia, p.inertia_inv, p.arm_length, p.gamma, p.gravity,
                )
                states.append(sample.copy())
            internal = sample
        np.testing.assert_array_equal(next_obs, internal)
        obs, info = next_obs, next_info
    ticks = round(snapshot.approval_time * b.PYB_FREQ_HZ) + np.arange(len(states))
    return ticks, np.asarray(states)


def unsafe_mask(positions, scenario):
    # Use the original predicate, including its closed boundary inequalities.
    state = np.zeros(STATE_SIZE)
    result = []
    for position in positions:
        state[[STATE_X, STATE_Y, STATE_Z]] = position
        result.append(b.unsafe(state, scenario))
    return np.asarray(result)


def diagnose(row, quad_params, pid_params, output):
    check_memory()
    s = b.Scenario(**row["scenario"])
    snapshot = b.capture_approval(s)
    prediction = b.predict(snapshot, b.LightweightPredictiveModel(quad_params, pid_params))
    assert prediction == row["prediction"]
    approval = round(snapshot.approval_time * b.CTRL_FREQ_HZ)
    waited = row["dispatch_step"] - approval
    ready = row["computation_s"] <= waited / b.CTRL_FREQ_HZ
    expired = b.ttl_expired_at_dispatch(prediction, waited)
    assert ready and not expired and row["prediction_ready"]

    trace, physics = [], []
    outcome = b.run_actual(
        s, snapshot, row["dispatch_step"], row["recovery_step"], row["deadline_step"], False,
        ttl_prediction=(prediction, row["computation_s"]), trace=trace, physics_trace=physics,
    )
    assert list(outcome) == row["outcome"]
    digest = trace_hash(trace, physics)
    assert digest == row["trace_sha256"]
    np.testing.assert_array_equal(trace[approval][1], snapshot.state)

    actual_ticks = np.asarray([tick for tick, position in physics])
    actual = np.asarray([position for tick, position in physics])
    np.testing.assert_array_equal(actual_ticks, np.arange(len(actual)))
    actual_unsafe = unsafe_mask(actual, s)
    model_ticks, model_states = model_trace(snapshot, row["delay_s"], quad_params, pid_params)
    model = model_states[:, [STATE_X, STATE_Y, STATE_Z]]
    model_unsafe = unsafe_mask(model, s)
    control_stride = b.PYB_FREQ_HZ // b.CTRL_FREQ_HZ
    integration_stride = b.PYB_FREQ_HZ // MODEL_FREQ_HZ
    assert not model_unsafe[::control_stride].any()
    first = int(actual_ticks[np.flatnonzero(actual_unsafe)[0]])
    model_entry = int(model_ticks[np.flatnonzero(model[:, 0] >= s.box_x_min)[0]])
    lowest = int(np.argmin(model[:, 2]))
    clear = lowest + int(np.flatnonzero(model[lowest:, 2] > s.unsafe_below_z)[0])
    model_clear = int(model_ticks[clear])
    first_index = first - int(model_ticks[0])
    actual_at_entry = actual[first].tolist()
    model_at_entry = model[first_index].tolist()

    samples = []
    for tick in sorted({first - 1, first, first + 1,
                        first // control_stride * control_stride,
                        (first // control_stride + 1) * control_stride,
                        model_entry, model_clear}):
        index = tick - int(model_ticks[0])
        samples.append(dict(
            time_s=tick / b.PYB_FREQ_HZ,
            actual_xyz=actual[tick].tolist() if tick < len(actual) else None,
            model_xyz=model[index].tolist(),
            actual_unsafe=bool(actual_unsafe[tick]) if tick < len(actual) else None,
            model_unsafe=bool(model_unsafe[index]),
        ))
    np.savez_compressed(
        output / f"seed_{row['seed']}.npz",
        actual_ticks=actual_ticks, actual_xyz=actual,
        control_steps=np.asarray([step for step, state, reference in trace]),
        control_states=np.asarray([state for step, state, reference in trace]),
        model_ticks=model_ticks, model_states=model_states,
    )
    return dict(
        seed=row["seed"], delay_s=row["delay_s"], command=row["scenario"], prediction=prediction,
        predicted_safety=next(status for delay, status in zip(b.DELAY_GRID, prediction["results"])
                              if round(float(delay) * b.CTRL_FREQ_HZ) == waited),
        recorded_computation_s=row["computation_s"], prediction_available=bool(ready),
        waited_control_steps=waited, expiry_control_steps=round(prediction["first_bad_delay"] * b.CTRL_FREQ_HZ),
        ttl_expired=bool(expired), outcome=outcome, saved_trace_hash_matches=True,
        actual_first_unsafe_s=first / b.PYB_FREQ_HZ,
        actual_unsafe_intervals=intervals(actual_unsafe, actual_ticks / b.PYB_FREQ_HZ),
        actual_unsafe_at_control_samples=bool(actual_unsafe[::control_stride].any()),
        model_unsafe_at_control_samples=bool(model_unsafe[::control_stride].any()),
        model_unsafe_at_integration_endpoints=bool(model_unsafe[::integration_stride].any()),
        model_dense_unsafe_intervals=intervals(model_unsafe, model_ticks / b.PYB_FREQ_HZ),
        model_structure_entry_s=model_entry / b.PYB_FREQ_HZ,
        model_climb_clear_s=model_clear / b.PYB_FREQ_HZ,
        actual_xyz_at_entry=actual_at_entry, model_xyz_at_actual_entry=model_at_entry,
        model_minus_actual_at_entry_m=(model[first_index] - actual[first]).tolist(),
        near_entry=samples,
    )


def main(output):
    log = ROOT / "results/unknown_dispatch_delay/executions.jsonl"
    rows = [json.loads(line) for line in log.read_text().splitlines()]
    cases = [r for r in rows if r["policy"] == "StaleGuard" and r["outcome"][1]]
    assert [(r["seed"], r["delay_s"]) for r in cases] == [(9, 0.3), (7, 0.4), (8, 0.4)]
    calibrated = json.loads(b.CALIBRATION_FILE.read_text())
    for row in cases:
        assert row["scenario"] == calibrated[str(row["seed"])]["command"]
    # Reuse the existing scientific-input/log preservation list and add B's logs.
    preserved = ROOT / "results/unknown_dispatch_delay/preserved_files.json"
    paths = [ROOT / name for name in json.loads(preserved.read_text())]
    paths += [Path(b.__file__), log, ROOT / "result/baseline_unknown_dispatch.log"]
    hashes = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        output.mkdir(parents=True, exist_ok=False)
        quad_params, pid_params = b.load_scg_platform_params()
        results = []
        for row in cases:
            result = diagnose(row, quad_params, pid_params, output)
            results.append(result)
            gc.collect()
            print(f"Seed {row['seed']}, D={row['delay_s']:.1f}: saved trace matched; "
                  f"first unsafe at {result['actual_first_unsafe_s']:.3f}s", flush=True)
        for path, digest in hashes.items():
            assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, path
        (output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
        print(f"Preserved {len(hashes)} existing files; evidence: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/unsafe_staleguard_diagnostic")
    main(parser.parse_args().output)
