"""Observe PyBullet positions without changing physics steps or actions."""

import pybullet as p


def monitor_physics_steps(env, observe):
    """Call observe(position, tick) once per physics tick, starting at zero.

    Observe before each physics step. The caller must also call the returned
    sample() after env.step() to include the control endpoint/final state.
    Repeated observations of the same tick are skipped.
    """
    tick = 0
    last_sampled = -1
    original_physics = env._physics

    def sample():
        nonlocal last_sampled
        if tick != last_sampled:
            position, _ = p.getBasePositionAndOrientation(
                env.DRONE_ID, physicsClientId=env.PYB_CLIENT,
            )
            observe(position, tick)
            last_sampled = tick

    def monitored_physics(rpm):
        nonlocal tick
        sample()
        original_physics(rpm)
        tick += 1

    env._physics = monitored_physics
    return sample
