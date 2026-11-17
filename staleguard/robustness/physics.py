"""E01 adapters around unchanged Underpass physics, predictor and policy classes."""

from dataclasses import replace, asdict
from pathlib import Path
import os
import time
import resource
import uuid

import numpy as np
import pybullet as p

from staleguard.model.common import capture_pid_state, PIDState
from staleguard.model.scg import load_scg_platform_params
from staleguard.underpass_inspection.scenario import Scenario, POS, VEL
from staleguard.underpass_inspection.simulation import Snapshot, execute
from staleguard.underpass_inspection.baselines.recovery_horizons import predict
from staleguard.underpass_inspection.baselines.use_time_recheck import predict_immediate
from staleguard.underpass_inspection.baselines.policies import make_policy
from .protocol import ROOT, digest, save_record


class Hooks:
    def __init__(self, uncertainty, capture_steps=()):
        self.uncertainty = uncertainty
        self.capture_steps = set(capture_steps)
        self.snapshots = {}
        self.plant_metadata = None

    def after_reset(self, env, ctrl):
        before = p.getDynamicsInfo(env.DRONE_ID, -1, physicsClientId=env.PYB_CLIENT)
        nominal = float(env.MASS)
        target = before[0] * self.uncertainty['mass_factor']
        memory = capture_pid_state(ctrl)
        state = env.state.copy()
        if self.uncertainty['mass_factor'] != 1.0:
            p.changeDynamics(
                env.DRONE_ID,
                -1,
                mass=target,
                localInertiaDiagonal=before[2],
                physicsClientId=env.PYB_CLIENT,
            )
        after = p.getDynamicsInfo(env.DRONE_ID, -1, physicsClientId=env.PYB_CLIENT)
        assert np.isclose(after[0], target, rtol=0, atol=1e-15)
        np.testing.assert_array_equal(after[2], before[2])
        np.testing.assert_array_equal(env.state, state)
        for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
            np.testing.assert_array_equal(
                getattr(memory, name), getattr(capture_pid_state(ctrl), name)
            )
        assert env.MASS == nominal and float(ctrl.model.quad_mass) == nominal
        self.plant_metadata = dict(
            nominal_base_mass_kg=before[0],
            actual_base_mass_kg=after[0],
            mass_factor=self.uncertainty['mass_factor'],
            nominal_controller_mass_kg=nominal,
            inertia_before=list(before[2]),
            inertia_after=list(after[2]),
            control_dt=ctrl.control_timestep,
            pid_gains={
                name: np.asarray(getattr(ctrl, name)).tolist()
                for name in (
                    'P_COEFF_FOR',
                    'I_COEFF_FOR',
                    'D_COEFF_FOR',
                    'P_COEFF_TOR',
                    'I_COEFF_TOR',
                    'D_COEFF_TOR',
                )
            },
        )

    def before_step(self, step, obs, ctrl, reference, total, prefix, scenario):
        if step in self.capture_steps:
            self.snapshots[step] = Snapshot(
                obs.copy(),
                capture_pid_state(ctrl),
                reference[step:].copy(),
                total - step,
                step / 50,
                scenario,
                prefix,
            )

    def observation(self, snapshot):
        state = snapshot.state.copy()
        state[POS] += self.uncertainty['position_bias_m']
        state[VEL] += self.uncertainty['velocity_bias_mps']
        return replace(snapshot, state=state)


def save_snapshots(path, snapshots):
    values = {}
    for step, snap in snapshots.items():
        for field, value in (
            ('state', snap.state),
            ('reference', snap.remaining_reference),
            ('integral_pos_e', snap.pid_state.integral_pos_e),
            ('last_rpy', snap.pid_state.last_rpy),
            ('integral_rpy_e', snap.pid_state.integral_rpy_e),
            ('prefix', snap.prefix_min_clearance_m),
            ('horizon', snap.horizon_steps),
        ):
            values[f'{step}_{field}'] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, **values)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def load_snapshot(path, scenario, step):
    with np.load(path) as archive:
        value = lambda name: archive[f'{step}_{name}'].copy()
        return Snapshot(
            value('state'),
            PIDState(value('integral_pos_e'), value('last_rpy'), value('integral_rpy_e')),
            value('reference'),
            int(value('horizon')),
            step / 50,
            scenario,
            float(value('prefix')),
        )


def physical(scenario, uncertainty, action, capture_steps=()):
    hooks = Hooks(uncertainty, capture_steps)
    mode, step = action
    start = 0.0 if step is None else (step - round(scenario.approval_s * 50)) / 50
    began = time.perf_counter()
    record, arrays = execute(scenario, mode, start, simulation_hooks=hooks)
    record['control_sha256'] = __import__('hashlib').sha256(arrays['control'].tobytes()).hexdigest()
    record['reference_sha256'] = (
        __import__('hashlib').sha256(arrays['reference'].tobytes()).hexdigest()
    )
    record['physical_runtime_s'] = time.perf_counter() - began
    record['worker_peak_rss_mb'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    scope = ROOT / 'verification' if uncertainty.get('audit_scope') == 'verification' else ROOT
    save_record(
        scope / 'attempts' / str(scenario.seed) / (str(uuid.uuid4()) + '.json'),
        dict(
            scenario=asdict(scenario),
            uncertainty=uncertainty,
            action=list(action),
            capture_steps=list(capture_steps),
            record=record,
        ),
    )
    return record, hooks.snapshots


def forecast(snapshot, uncertainty, platform):
    # This API intentionally has no policy dispatch delay or physical truth argument.
    observed = Hooks(uncertainty).observation(snapshot)
    began = time.perf_counter()
    result = predict(observed, platform)
    result['supervisory_observation_sha256'] = digest(observed.state.tolist())
    result['nominal_predictor_mass_kg'] = platform[0].mass
    result['snapshot_adapter_s'] = time.perf_counter() - began - result['computation_s']
    return result


def decide(scenario, delay, policy, received, dispatch_snapshot, uncertainty, platform):
    agent = make_policy(policy, scenario, received if policy.startswith('staleguard') else None)
    approval = round(scenario.approval_s * 50)
    dispatch = approval + round(delay * 50)
    total = round((scenario.approval_s + scenario.observation_s) * 50)
    agent.approve()
    action = ('nominal', None)
    for step in range(approval, total):
        agent.announce_readiness(step)
        if step == dispatch:
            observed = (
                Hooks(uncertainty).observation(dispatch_snapshot)
                if policy == 'use_time_recheck'
                else None
            )
            agent.dispatch(step, observed, platform)
        agent.tick(step)
        chosen = agent.take_action()
        if chosen is not None:
            if action[0] != 'nominal':
                raise ValueError('Policy selected multiple physical actions')
            action = (chosen, step)
    event = agent.finish(dispatch, total)
    return dict(
        seed=scenario.seed,
        condition=uncertainty['condition'],
        policy=policy,
        delay_s=delay,
        action=list(action),
        event=event,
        dispatch_observation_sha256=(
            digest(Hooks(uncertainty).observation(dispatch_snapshot).state.tolist())
            if policy == 'use_time_recheck'
            else None
        ),
    )


def warm(scenario, snapshot, platform):
    predict(snapshot, platform)
    predict_immediate(snapshot, platform)
