"""Serial before/after validation of swept prediction and physics-step truth."""

import argparse
import ast
import contextlib
from dataclasses import asdict
import fcntl
import gc
import hashlib
import importlib.util
import json
from pathlib import Path
import pickle
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]
from validate_official_calibration import check_memory, trace_hash
import numpy as np
import ground_truth as gt
import mission
import staleguard
import staleguard_benefit as b
from diagnose_unsafe_staleguard import model_trace, unsafe_mask
from experiment import classify_delay_results, unsafe_segment

OUTPUT = ROOT / "results/temporal_safety_correction"


def before_module(name):
    spec = importlib.util.spec_from_file_location("before_temporal_" + name, OUTPUT / "before" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_preserved():
    allowed = {"experiment.py", "staleguard.py", "ground_truth.py", "staleguard_benefit.py",
               "README.md", "result/README.md"}
    hashes = json.loads((OUTPUT / "preserved_files.json").read_text())
    for name, digest in hashes.items():
        if name not in allowed:
            assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    old_ast = ast.parse((OUTPUT / "before/staleguard_benefit.py").read_text())
    new_ast = ast.parse(Path(b.__file__).read_text())
    old = {n.name: ast.dump(n) for n in old_ast.body if isinstance(n, ast.FunctionDef)}
    new = {n.name: ast.dump(n) for n in new_ast.body if isinstance(n, ast.FunctionDef)}
    for name in old.keys() - {"run_actual"}:
        assert old[name] == new[name], name
    old_experiment = ast.parse((OUTPUT / "before/experiment.py").read_text())
    new_experiment = ast.parse((ROOT / "experiment.py").read_text())
    for name in ("generate_scenario", "make_forward_trajectory", "apply_descent_command",
                 "inspection_x", "unsafe", "classify_delay_results"):
        old = next(n for n in old_experiment.body if isinstance(n, ast.FunctionDef) and n.name == name)
        new = next(n for n in new_experiment.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(old) == ast.dump(new), name


def execute(module, s, snapshot, row, **kwargs):
    check_memory()
    trace, physics = [], []
    outcome = module.run_actual(
        s, snapshot, row["dispatch_step"], row["dispatch_step"], row["deadline_step"],
        kwargs.pop("recover", False), trace=trace, physics_trace=physics, **kwargs,
    )
    result = outcome, trace_hash(trace, physics)
    del trace, physics
    gc.collect()
    return result


def ground_truth_trace(module, s, delay):
    """Compare every action, control state, and pre-physics position, not timing labels."""
    digest = hashlib.sha256()
    factory = module.make_scg_env_factory
    def recorded_factory(*args, **kwargs):
        make_env = factory(*args, **kwargs)
        def create(*args, **kwargs):
            env = make_env(*args, **kwargs)
            physics, step = env._physics, env.step
            def recorded_physics(rpm):
                position, _ = b.p.getBasePositionAndOrientation(env.DRONE_ID, physicsClientId=env.PYB_CLIENT)
                digest.update(np.asarray(position).tobytes())
                physics(rpm)
            def recorded_step(action):
                digest.update(np.asarray(action).tobytes())
                result = step(action)
                digest.update(np.asarray(result[0]).tobytes())
                return result
            env._physics, env.step = recorded_physics, recorded_step
            return env
        return create
    check_memory()
    with patch.object(module, "make_scg_env_factory", recorded_factory):
        result = module.run_one(s, delay)
    gc.collect()
    return result, digest.hexdigest()


def small_checks():
    old_guard = before_module("staleguard")
    old_flight = before_module("staleguard_benefit")
    old_truth = before_module("ground_truth")
    rows = [json.loads(line) for line in (ROOT / "results/unknown_dispatch_delay/executions.jsonl").read_text().splitlines()]
    cases = [r for r in rows if r["policy"] == "StaleGuard" and r["outcome"][1]]
    quad, pid = b.load_scg_platform_params()
    # Warm the same dynamics/PID kernels before measuring B reception time.
    warmup = b.capture_approval(b.Scenario(**cases[0]["scenario"]))
    staleguard.predict(warmup, b.LightweightPredictiveModel(quad, pid))
    records = []
    for row in cases:
        s = b.Scenario(**row["scenario"])
        snapshot = b.capture_approval(s)
        old_prediction = old_guard.predict(snapshot, b.LightweightPredictiveModel(quad, pid))
        assert old_prediction == row["prediction"]
        prediction, duration = b.predict_at_reception(snapshot, quad, pid)
        # Replay the original command through the extracted shared monitor.
        original, digest = execute(b, s, snapshot, row, ttl_prediction=(old_prediction, row["computation_s"]))
        assert list(original) == row["outcome"] and digest == row["trace_sha256"]
        outcome, new_digest = execute(b, s, snapshot, row, ttl_prediction=(prediction, duration))
        old_gt, old_hash = ground_truth_trace(old_truth, s, row["delay_s"])
        new_gt, new_hash = ground_truth_trace(gt, s, row["delay_s"])
        assert old_hash == new_hash
        expected = {9: 1.829, 7: 2.254, 8: 2.091}[row["seed"]]
        assert new_gt["hit_after_command"] == expected
        ticks, states = model_trace(snapshot, row["delay_s"], quad, pid)
        coarse = states[::b.PYB_FREQ_HZ // b.CTRL_FREQ_HZ]
        swept = any(unsafe_segment(a, c, s) for a, c in zip(coarse, coarse[1:]))
        dense = bool(unsafe_mask(states[:, [0, 2, 4]], s).any())
        assert swept == dense == (row["seed"] == 9)
        if row["seed"] == 9:
            assert outcome == (False, False, True, False)
            # Shared fallback must still have the historical execution trace.
            reference = execute(old_flight, s, snapshot, row, recover=True)
            assert (outcome, new_digest) == reference
            old_retry = execute(old_flight, s, snapshot, row, recover=True, recovery_mode="mission-retry")
            new_retry = execute(b, s, snapshot, row, recover=True, recovery_mode="mission-retry")
            assert old_retry == new_retry
        else:
            assert outcome == original and new_digest == digest
        record = dict(seed=row["seed"], delay_s=row["delay_s"], old_prediction=old_prediction,
                      new_prediction=prediction, measured_computation_s=duration,
                      prediction_ready=duration <= row["delay_s"], old_outcome=original, new_outcome=outcome,
                      old_ground_truth=old_gt, new_ground_truth=new_gt,
                      ground_truth_dynamics_trace_identical=True,
                      original_operational_trace_identical=True, swept_unsafe=swept, dense_unsafe=dense)
        records.append(record)
        gc.collect()
        print(f"PASS: seed {row['seed']}, D={row['delay_s']}: new outcome={outcome}, compute={1000*duration:.2f} ms", flush=True)
    (OUTPUT / "small_checks.json").write_text(json.dumps(records, indent=2) + "\n")
    check_preserved()


def ground_truth_checks():
    """Save simulator measurements and snapshots without compiling the predictor."""
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    scenarios = {int(seed): b.Scenario(**r["command"]) for seed, r in saved.items() if r["command"] is not None}
    completed_path = OUTPUT / "completed_ground_truth.json"
    completed = json.loads(completed_path.read_text()) if completed_path.exists() else {}
    quad, pid = b.load_scg_platform_params()
    snapshots = {}
    for seed, s in scenarios.items():
        check_memory()
        snapshots[seed] = b.capture_approval(s)
        gc.collect()
    inputs = dict(snapshots=snapshots, warmup=b.capture_approval(b.generate_scenario(0)),
                  quad_params=quad, pid_params=pid,
                  calibration_sha256=hashlib.sha256(b.CALIBRATION_FILE.read_bytes()).hexdigest())
    with (OUTPUT / "prediction_inputs.pkl").open("wb") as stream:
        pickle.dump(inputs, stream)
    records_path = OUTPUT / "ground_truth_remaining.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()] if records_path.exists() else []
    def delay_key(delay):
        return None if delay is None else round(float(delay) * b.CTRL_FREQ_HZ)
    measured = {(r["seed"], delay_key(r["delay_s"])): r for r in records}
    run_one = gt.run_one
    results = {}
    with records_path.open("a") as stream:
        for seed, s in scenarios.items():
            if str(seed) in completed:
                actual = classify_delay_results(b.DELAY_GRID, completed[str(seed)]["results"])
                source = completed[str(seed)]["source"]
            else:
                def recorded_run(s, delay):
                    key = seed, delay_key(delay)
                    if key in measured:
                        assert measured[key]["command"] == asdict(s)
                        return measured[key]["result"]
                    check_memory()
                    result = run_one(s, delay)
                    row = dict(seed=seed, command=asdict(s), delay_s=delay, result=result)
                    stream.write(json.dumps(row) + "\n"); stream.flush()
                    measured[key] = row
                    gc.collect()
                    return result
                with patch.object(gt, "run_one", recorded_run):
                    actual = gt.classify_scenario(s, b.DELAY_GRID)
                source = "ground_truth_remaining.jsonl"
            results[str(seed)] = dict(actual=actual, source=source)
            (OUTPUT / "ground_truth_results.json").write_text(json.dumps(results, indent=2) + "\n")
            print(f"Ground truth complete: seed {seed}; source={source}; RSS={check_memory()[0]:.0f} MiB", flush=True)
    assert len(results) == 19
    check_preserved()


def accuracy_checks():
    # Reuse measured truth across processes so simulator libraries and the JIT
    # compiler do not need to occupy memory at the same time.
    with (OUTPUT / "prediction_inputs.pkl").open("rb") as stream:
        inputs = pickle.load(stream)
    assert inputs["calibration_sha256"] == hashlib.sha256(b.CALIBRATION_FILE.read_bytes()).hexdigest()
    truth = json.loads((OUTPUT / "ground_truth_results.json").read_text())
    assert len(truth) == 19
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    scenarios = {int(seed): b.Scenario(**r["command"]) for seed, r in saved.items() if r["command"] is not None}
    def key(snapshot):
        return snapshot.approval_time, snapshot.low_height, snapshot.low_steps, snapshot.box_x_min
    keys = {(s.approval_time, s.low_height, round(s.low_seconds * b.CTRL_FREQ_HZ), s.box_x_min): seed
            for seed, s in scenarios.items()}
    snapshots, predictions, actuals = {}, {}, {}
    predict = mission.predict
    def recorded_predict(snapshot, predictive_model, delays):
        result = predict(snapshot, predictive_model, delays)
        seed = keys.get(key(snapshot))
        if seed is not None:
            snapshots[seed], predictions[seed] = snapshot, result
        return result
    def recorded_classify(s, delays):
        seed = next(seed for seed, scenario in scenarios.items() if asdict(scenario) == asdict(s))
        assert tuple(delays) == b.DELAY_GRID
        result = truth[str(seed)]["actual"]
        actuals[seed] = result
        return result
    def saved_snapshot(s):
        for seed, scenario in scenarios.items():
            if asdict(scenario) == asdict(s):
                return inputs["snapshots"][seed]
        assert asdict(s) == asdict(b.generate_scenario(0))
        return inputs["warmup"]
    quad, pid = inputs["quad_params"], inputs["pid_params"]
    log = ROOT / "result/ttl_accuracy_swept_physics.log"
    with log.open("x") as stream, contextlib.redirect_stdout(stream), \
         patch.object(mission, "predict", recorded_predict), \
         patch.object(gt, "classify_scenario", recorded_classify), \
         patch.object(mission, "capture_approval", saved_snapshot), \
         patch.object(mission, "load_scg_platform_params", return_value=(quad, pid)), \
         patch.object(sys, "argv", ["mission.py", "--calibrated", "--count", "20"]):
        print("Ground truth: reused 228 measured serial 1,000 Hz PyBullet flights; snapshots unchanged.")
        mission.main()
    assert len(predictions) == len(actuals) == 19

    # Same snapshots and dynamics, alternating old/new order after JIT warmup.
    old = before_module("staleguard")
    timing, comparison, dense_checks = [], [], []
    for seed, snapshot in snapshots.items():
        check_memory()
        before = old.predict(snapshot, b.LightweightPredictiveModel(quad, pid))
        comparison.append(dict(seed=seed, old_prediction=before, new_prediction=predictions[seed], ground_truth=actuals[seed]))
        for repetition in range(5):
            durations = {}
            order = [("old", old.predict), ("new", staleguard.predict)]
            if repetition % 2:
                order.reverse()
            for name, predictor in order:
                started = time.perf_counter()
                result = predictor(snapshot, b.LightweightPredictiveModel(quad, pid))
                durations[name] = time.perf_counter() - started
                assert result == (before if name == "old" else predictions[seed])
            timing.append(dict(seed=seed, repetition=repetition, **durations))
        for index, delay in enumerate(b.DELAY_GRID):
            ticks, states = model_trace(snapshot, delay, quad, pid)
            coarse = states[::b.PYB_FREQ_HZ // b.CTRL_FREQ_HZ]
            swept = any(unsafe_segment(a, c, scenarios[seed]) for a, c in zip(coarse, coarse[1:]))
            mask = unsafe_mask(states[:, [0, 2, 4]], scenarios[seed])
            dense = bool(mask.any())
            assert swept == (predictions[seed]["results"][index] != "SAFE")
            dense_checks.append(dict(seed=seed, delay_s=float(delay), swept_unsafe=swept, dense_unsafe=dense))
            if swept != dense:
                np.savez_compressed(OUTPUT / f"segment_dense_difference_{seed}_{index}.npz", ticks=ticks, states=states)
        print(f"Prediction timing and dense comparison: seed {seed} checked", flush=True)
        gc.collect()
    (OUTPUT / "accuracy_comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    (OUTPUT / "prediction_timing.json").write_text(json.dumps(timing, indent=2) + "\n")
    (OUTPUT / "dense_comparison.json").write_text(json.dumps(dense_checks, indent=2) + "\n")
    check_preserved()
    print(f"PASS: calibrated TTL runner, 228 measured ground-truth flights reused, 95 paired timings, {len(dense_checks)} dense comparisons.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("small", "truth", "accuracy"), default="small")
    args = parser.parse_args()
    check_preserved()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"small": small_checks, "truth": ground_truth_checks, "accuracy": accuracy_checks}[args.stage]()
