"""Mission prediction adapters, exact pending-command clock and physical replay."""

from types import SimpleNamespace
import importlib, time
from staleguard.underpass_inspection.baselines.policies import make_policy


def prediction(mission, s, snap, platform, **kwargs):
    module = importlib.import_module('staleguard.' + mission + '.baselines.recovery_horizons')
    began = time.perf_counter()
    wall = time.time()
    result = module.predict(s, snap, platform, **kwargs)
    result.update(
        computation_s=time.perf_counter() - began,
        wall_started_unix_s=wall,
        wall_finished_unix_s=time.time(),
    )
    return result


def clock_action(policy, snapshot, delay, forecast=None, horizon_steps=None):
    """Simulator schedules events; the unchanged guard sees only elapsed clock."""
    approval = round(snapshot.approval_time * 50)
    dispatch = approval + round(delay * 50)
    scenario = SimpleNamespace(approval_s=snapshot.approval_time)
    agent = make_policy(policy, scenario, forecast)
    agent.approve()
    selected = None
    horizon = approval + (snapshot.horizon_steps if horizon_steps is None else horizon_steps)
    for step in range(approval, horizon):
        agent.announce_readiness(step)
        if step == dispatch:
            agent.dispatch(step, None, None)
        agent.tick(step)
        action = agent.take_action()
        if action is not None:
            assert selected is None
            selected = ('recovery' if action == 'sideways' else 'command', step)
    assert selected is not None
    return selected, agent.finish(dispatch, horizon)


def physical(job):
    mission, seed, s, snapshot, delay, action, event = job
    if action is None:
        action = ('recovery', round(snapshot.approval_time * 50))
    import gc, ctypes

    gc.collect()
    ctypes.CDLL(None).malloc_trim(0)
    if mission == 'geofence_turn':
        from staleguard.geofence_turn.simulation import execute
        from staleguard.geofence_turn.storage import check_approval_snapshot

        row, actual, _ = execute(s, delay=delay, policy='no_recovery', forced_action=action)
        check_approval_snapshot(actual, snapshot)
    else:
        from staleguard.inspection_descent.simulation import execute

        row, actual, _ = execute(
            s, delay=delay, policy='no_recovery', snapshot=snapshot, seed=seed, forced_action=action
        )
    native = row['event'].copy()
    if event is not None:
        row['event'] = dict(event)
        # Physical early termination cannot be erased by precomputed timer logs.
        for key in ('application_step', 'fallback_step'):
            row['event'][key] = native.get(key)
        end = (
            round(row['horizon_s'] * 50)
            if row['safe']
            else round(row.get('monitored_until_s', row['horizon_s']) * 50)
        )
        row['event']['events'] = [e for e in event.get('events', []) if e['control_step'] <= end]
        row['event']['early_recovery'] = (
            native.get('fallback_step') is not None
            and native['fallback_step'] < native['scheduled_dispatch_step']
        )
        row['native_event'] = native
    a = round(snapshot.approval_time * 50)
    start = (
        native.get('fallback_step')
        if native.get('fallback_step') is not None
        else native.get('application_step')
    )
    row.update(
        mission=mission,
        seed=seed,
        delay_s=round(delay, 2),
        approval_step=a,
        start_s=None if start is None else (start - a) / 50,
        mode='recovery' if native.get('fallback_step') is not None else 'command',
        execution_kind='new_physical_flight',
        approval_snapshot_verified=True,
        observation_horizon_s=4.2 if mission == 'inspection_descent' else s.deadline_s,
    )
    assert row['safe_completion'] == (row['safe'] and row['completed'])
    return row
