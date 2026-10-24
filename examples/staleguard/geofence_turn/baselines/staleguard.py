"""Mission-specific safety adapters around the unchanged lightweight dynamics.

The nominal patrol eventually crosses the fence: unlike Inspection-Descent,
unsafe *unbounded nominal continuation* is not grounds to discard all queued
turns. Only the waiting prefix and the dispatched path are evaluated for each D.
No mission-completion prediction is used to manufacture a favorable TTL.
"""
import time
from model.lightweight import LightweightPredictiveModel
from ..scenario import HZ, apply_turn_command, clearance

# Frozen development grid; actual future delay is never an input at approval.
PREDICTION_DELAYS = tuple(i/10 for i in range(11))



def predict_delays(snapshot, platform, delays=PREDICTION_DELAYS):
    if not hasattr(snapshot.command, 'checkpoint_lower') or not hasattr(snapshot.command, 'checkpoint_upper'):
        raise ValueError('Legacy connector snapshot: regenerate the northbound dataset')
    began = time.perf_counter()
    model = LightweightPredictiveModel(*platform)
    checkpoints = {}
    try:
        ctrl, obs, info, reference = model.start(snapshot)
        wanted = {round(d*HZ) for d in delays}
        prefix = snapshot.prefix_min_clearance_m >= snapshot.config.d_min_m
        prefix_min = snapshot.prefix_min_clearance_m
        # Predict just the required waiting prefix once; unknown D stays unknown.
        for step in range(max(wanted)+1):
            if step in wanted:
                checkpoints[step] = (model.save_checkpoint(ctrl, obs, info), prefix, prefix_min)
            if step == max(wanted):
                break
            action = ctrl.select_action(obs, info)
            prev = obs
            obs, _, info = model.step(action, obs, info)
            # A half-space's signed clearance along a straight segment attains
            # its minimum at an endpoint. Sphere support is orientation invariant.
            current = min(float(clearance(prev[0], snapshot.fence_x)),
                          float(clearance(obs[0], snapshot.fence_x)))
            prefix_min = min(prefix_min, current)
            prefix = prefix and current >= snapshot.config.d_min_m
        results = []
        for delay in delays:
            step = round(delay*HZ)
            checkpoint, safe, minimum = checkpoints[step]
            obs, info = model.restore_checkpoint(ctrl, checkpoint)
            changed = apply_turn_command(reference, step, snapshot.command)
            model.set_reference(ctrl, changed)
            for _ in range(step, snapshot.horizon_steps):
                action = ctrl.select_action(obs, info)
                previous = obs
                obs, _, info = model.step(action, obs, info)
                minimum = min(minimum, float(clearance(previous[0], snapshot.fence_x)),
                              float(clearance(obs[0], snapshot.fence_x)))
                safe = safe and minimum >= snapshot.config.d_min_m
            results.append(dict(delay_s=float(delay), safe=bool(safe), min_clearance_m=minimum))
        # Conservative first-invalid expiry, including any nonmonotonic tail.
        bad = next((r for r in results if not r['safe']), None)
        expiry = None if bad is None else round(bad['delay_s']*HZ)
        return dict(results=results, expiry_steps=expiry,
                    computation_s=time.perf_counter()-began, grid_s=list(delays))
    finally:
        model.close(checkpoints)


def staleguard_status(received, waited_steps):
    ready = received['computation_s'] <= waited_steps/HZ
    expired = received['expiry_steps'] is not None and waited_steps >= received['expiry_steps']
    return ready, ready and expired
