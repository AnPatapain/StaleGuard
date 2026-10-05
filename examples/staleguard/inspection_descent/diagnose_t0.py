"""
Diagnose T0 approval-state reconstruction mismatch.

Run:
    python diagnose_t0.py --scenario 8 --delay 1.0

Compares:

A) LIVE:
   Run mission normally to approval, then continue in SAME simulator.

B) RESTORED:
   Capture ApprovalSnapshot, create a fresh StaleGuard simulator at approval,
   restore UAV + PID state, then continue.

If A and B diverge, the problem is approval-state reconstruction, not TTL logic.
"""

import argparse
import numpy as np

import ground_truth
import mission
import staleguard


STATE_NAMES = [
    "x", "vx",
    "y", "vy",
    "z", "vz",
    "roll", "pitch", "yaw",
    "p", "q", "r",
]


def pid_snapshot(ctrl):
    return {
        "integral_pos_e": np.asarray(
            ctrl.integral_pos_e
        ).copy(),

        "last_rpy": np.asarray(
            ctrl.last_rpy
        ).copy(),

        "integral_rpy_e": np.asarray(
            ctrl.integral_rpy_e
        ).copy(),
    }


def print_vec(name, x):
    x = np.asarray(x)
    print(
        f"{name:<22}: "
        + " ".join(
            f"{v:+.10e}"
            for v in x
        )
    )


def is_unsafe(state, obstacle):
    return (
        obstacle.x_min <= state[0] <= obstacle.x_max
        and abs(state[2]) <= obstacle.y_limit
        and state[4] <= obstacle.unsafe_below_z
    )


def safety_margin(state, obstacle):
    """
    Vertical clearance only while UAV is horizontally inside obstacle.

    > 0 : vertically safe
    = 0 : boundary
    < 0 : unsafe
    None: UAV is not horizontally over/in obstacle.
    """

    inside_xy = (
        obstacle.x_min
        <= state[0]
        <= obstacle.x_max
        and abs(state[2])
        <= obstacle.y_limit
    )

    if not inside_xy:
        return None

    return (
        state[4]
        - obstacle.unsafe_below_z
    )


# ============================================================
# A. Continue from approval WITHOUT reconstructing simulator.
# ============================================================

def trace_live(s, delay):

    (
        env,
        ctrl,
        obs,
        info,
        traj,
    ) = mission.start_mission_simulator(s)

    approval_step = round(
        s.approval_time
        * mission.FREQ
    )

    total_steps = round(
        mission.MEASURE_SECONDS
        * mission.FREQ
    )

    dispatch_global_step = (
        approval_step
        + round(
            delay
            * mission.FREQ
        )
    )

    low_steps = round(
        s.low_seconds
        * mission.FREQ
    )

    try:

        # ------------------------------------------------
        # Run original mission to approval.
        # ------------------------------------------------

        for _ in range(
            approval_step
        ):

            action = ctrl.select_action(
                obs,
                info,
            )

            (
                obs,
                _,
                done,
                info,
            ) = env.step(action)

            if done:
                raise RuntimeError(
                    "Mission ended before approval"
                )

        approval_obs = (
            np.asarray(obs).copy()
        )

        approval_pid = (
            pid_snapshot(ctrl)
        )

        # ------------------------------------------------
        # Continue in SAME simulator.
        # local_step = 0 == approval.
        # ------------------------------------------------

        states = [
            approval_obs.copy()
        ]

        actions = []

        pids_before = []

        pids_after = []

        refs_z = []

        for local_step in range(
            total_steps
            - approval_step
        ):

            global_step = (
                approval_step
                + local_step
            )

            if (
                global_step
                == dispatch_global_step
            ):

                changed = traj.copy()

                changed[
                    global_step:
                    global_step + low_steps,
                    4,
                ] = s.low_height

                env.X_GOAL = changed

                ctrl.reference = changed

            pids_before.append(
                pid_snapshot(ctrl)
            )

            refs_z.append(
                float(
                    ctrl.reference[
                        global_step,
                        4,
                    ]
                )
            )

            action = ctrl.select_action(
                obs,
                info,
            )

            actions.append(
                np.asarray(action).copy()
            )

            pids_after.append(
                pid_snapshot(ctrl)
            )

            (
                obs,
                _,
                done,
                info,
            ) = env.step(action)

            states.append(
                np.asarray(obs).copy()
            )

            if done:
                break

        return {
            "approval_state":
                approval_obs,

            "approval_pid":
                approval_pid,

            "states":
                np.asarray(states),

            "actions":
                np.asarray(actions),

            "pids_before":
                pids_before,

            "pids_after":
                pids_after,

            "refs_z":
                np.asarray(refs_z),

            "dispatch_local_step":
                round(
                    delay
                    * mission.FREQ
                ),
        }

    finally:

        ctrl.close()
        env.close()


# ============================================================
# B. StaleGuard: reconstruct simulator AT approval.
# ============================================================

def trace_restored(
    snapshot,
    delay,
):

    (
        env,
        ctrl,
        obs,
        info,
        reference,
    ) = staleguard.start_from_approval(
        snapshot
    )

    dispatch_step = round(
        delay
        * staleguard.FREQ
    )

    states = [
        np.asarray(obs).copy()
    ]

    actions = []

    pids_before = []

    pids_after = []

    refs_z = []

    try:

        for local_step in range(
            snapshot.horizon_steps
        ):

            if (
                local_step
                == dispatch_step
            ):

                changed = (
                    reference.copy()
                )

                end = min(
                    local_step
                    + snapshot.command.low_steps,

                    len(changed),
                )

                changed[
                    local_step:end,
                    4,
                ] = (
                    snapshot.command.low_height
                )

                env.X_GOAL = changed

                ctrl.reference = changed

            pids_before.append(
                pid_snapshot(ctrl)
            )

            refs_z.append(
                float(
                    ctrl.reference[
                        local_step,
                        4,
                    ]
                )
            )

            action = ctrl.select_action(
                obs,
                info,
            )

            actions.append(
                np.asarray(action).copy()
            )

            pids_after.append(
                pid_snapshot(ctrl)
            )

            (
                obs,
                _,
                done,
                info,
            ) = env.step(action)

            states.append(
                np.asarray(obs).copy()
            )

            if done:
                break

        return {
            "approval_state":
                states[0].copy(),

            "approval_pid":
                pid_snapshot_at_start(
                    snapshot
                ),

            "states":
                np.asarray(states),

            "actions":
                np.asarray(actions),

            "pids_before":
                pids_before,

            "pids_after":
                pids_after,

            "refs_z":
                np.asarray(refs_z),

            "dispatch_local_step":
                dispatch_step,
        }

    finally:

        ctrl.close()
        env.close()


def pid_snapshot_at_start(
    snapshot,
):
    return {
        "integral_pos_e":
            np.asarray(
                snapshot.pid_state.integral_pos_e
            ).copy(),

        "last_rpy":
            np.asarray(
                snapshot.pid_state.last_rpy
            ).copy(),

        "integral_rpy_e":
            np.asarray(
                snapshot.pid_state.integral_rpy_e
            ).copy(),
    }


# ============================================================
# Diagnostics
# ============================================================

def compare_initial_state(
    live,
    restored,
):

    print()
    print(
        "=" * 80
    )
    print(
        "1. APPROVAL STATE"
    )
    print(
        "=" * 80
    )

    a = live[
        "approval_state"
    ]

    b = restored[
        "approval_state"
    ]

    print_vec(
        "LIVE",
        a,
    )

    print_vec(
        "RESTORED",
        b,
    )

    print_vec(
        "DIFF",
        b - a,
    )

    print(
        "max abs state diff:",
        np.max(
            np.abs(
                b - a
            )
        ),
    )


def compare_initial_pid(
    live,
    restored,
):

    print()
    print(
        "=" * 80
    )
    print(
        "2. PID STATE AT APPROVAL"
    )
    print(
        "=" * 80
    )

    for key in [
        "integral_pos_e",
        "last_rpy",
        "integral_rpy_e",
    ]:

        a = live[
            "approval_pid"
        ][key]

        b = restored[
            "approval_pid"
        ][key]

        print()
        print(key)

        print_vec(
            "LIVE",
            a,
        )

        print_vec(
            "RESTORED",
            b,
        )

        print_vec(
            "DIFF",
            b - a,
        )


def find_first_state_divergence(
    live,
    restored,
    atol=1e-8,
):

    A = live["states"]

    B = restored["states"]

    n = min(
        len(A),
        len(B),
    )

    for i in range(n):

        diff = np.abs(
            A[i] - B[i]
        )

        if np.max(diff) > atol:

            return (
                i,
                diff,
            )

    return (
        None,
        None,
    )


def find_first_action_divergence(
    live,
    restored,
    atol=1e-8,
):

    A = live["actions"]

    B = restored["actions"]

    n = min(
        len(A),
        len(B),
    )

    for i in range(n):

        diff = np.abs(
            A[i] - B[i]
        )

        if np.max(diff) > atol:

            return (
                i,
                diff,
            )

    return (
        None,
        None,
    )


def print_first_divergence(
    live,
    restored,
):

    print()
    print(
        "=" * 80
    )
    print(
        "3. FIRST DIVERGENCE"
    )
    print(
        "=" * 80
    )

    action_step, action_diff = (
        find_first_action_divergence(
            live,
            restored,
        )
    )

    state_step, state_diff = (
        find_first_state_divergence(
            live,
            restored,
        )
    )

    print(
        "first action divergence:",
        action_step,
    )

    if action_step is not None:

        print_vec(
            "LIVE action",
            live["actions"][
                action_step
            ],
        )

        print_vec(
            "REST action",
            restored["actions"][
                action_step
            ],
        )

        print_vec(
            "ACTION diff",
            action_diff,
        )

    print()

    print(
        "first state divergence:",
        state_step,
    )

    if state_step is not None:

        print(
            "time after approval:",
            state_step
            / mission.FREQ,
            "s",
        )

        print_vec(
            "LIVE state",
            live["states"][
                state_step
            ],
        )

        print_vec(
            "REST state",
            restored["states"][
                state_step
            ],
        )

        print_vec(
            "STATE abs diff",
            state_diff,
        )


def print_state_error_summary(
    live,
    restored,
):

    print()
    print(
        "=" * 80
    )
    print(
        "4. MAX STATE ERROR BY VARIABLE"
    )
    print(
        "=" * 80
    )

    A = live["states"]

    B = restored["states"]

    n = min(
        len(A),
        len(B),
    )

    abs_error = np.abs(
        A[:n] - B[:n]
    )

    max_error = np.max(
        abs_error,
        axis=0,
    )

    argmax = np.argmax(
        abs_error,
        axis=0,
    )

    for j, name in enumerate(
        STATE_NAMES
    ):

        print(
            f"{name:<8} "
            f"max_error="
            f"{max_error[j]:.12e} "
            f"at local_step="
            f"{argmax[j]} "
            f"(t="
            f"{argmax[j]/mission.FREQ:.3f}s)"
        )


def print_dispatch_window(
    live,
    restored,
    radius=5,
):

    print()
    print(
        "=" * 80
    )
    print(
        "5. AROUND DISPATCH"
    )
    print(
        "=" * 80
    )

    dispatch = (
        live[
            "dispatch_local_step"
        ]
    )

    start = max(
        0,
        dispatch - radius,
    )

    end = min(
        len(live["states"]),
        len(restored["states"]),
        dispatch + radius + 2,
    )

    print(
        "step   time     "
        "live_x       rest_x       "
        "dx           "
        "live_z       rest_z       "
        "dz           "
        "live_ref_z   rest_ref_z"
    )

    for i in range(
        start,
        end,
    ):

        a = live["states"][i]

        b = restored["states"][i]

        ref_index = min(
            i,
            len(live["refs_z"]) - 1,
            len(restored["refs_z"]) - 1,
        )

        print(
            f"{i:4d} "
            f"{i/mission.FREQ:7.3f} "
            f"{a[0]:+12.8f} "
            f"{b[0]:+12.8f} "
            f"{b[0]-a[0]:+12.3e} "
            f"{a[4]:+12.8f} "
            f"{b[4]:+12.8f} "
            f"{b[4]-a[4]:+12.3e} "
            f"{live['refs_z'][ref_index]:+12.8f} "
            f"{restored['refs_z'][ref_index]:+12.8f}"
        )


def print_safety_analysis(
    live,
    restored,
    obstacle,
):

    print()
    print(
        "=" * 80
    )
    print(
        "6. SAFETY BOUNDARY ANALYSIS"
    )
    print(
        "=" * 80
    )

    for name, trace in [
        ("LIVE", live),
        ("RESTORED", restored),
    ]:

        states = trace["states"]

        unsafe_steps = []

        margins = []

        for i, state in enumerate(
            states
        ):

            margin = safety_margin(
                state,
                obstacle,
            )

            if margin is not None:

                margins.append(
                    (
                        i,
                        margin,
                        state.copy(),
                    )
                )

            if is_unsafe(
                state,
                obstacle,
            ):

                unsafe_steps.append(i)

        print()
        print(name)

        if unsafe_steps:

            first = unsafe_steps[0]

            print(
                "first unsafe local step:",
                first,
            )

            print(
                "first unsafe time:",
                first
                / mission.FREQ,
                "s after approval",
            )

            print_vec(
                "unsafe state",
                states[first],
            )

        else:

            print(
                "never unsafe"
            )

        if margins:

            best = min(
                margins,
                key=lambda x: x[1],
            )

            i, margin, state = best

            print(
                "minimum vertical margin:",
                f"{margin:+.12e} m",
            )

            print(
                "at local step:",
                i,
                f"(t={i/mission.FREQ:.3f}s)",
            )

            print_vec(
                "state",
                state,
            )


def print_near_boundary(
    live,
    restored,
    obstacle,
    radius=4,
):

    candidates = []

    for trace_name, trace in [
        ("LIVE", live),
        ("RESTORED", restored),
    ]:

        for i, state in enumerate(
            trace["states"]
        ):

            margin = safety_margin(
                state,
                obstacle,
            )

            if margin is not None:

                candidates.append(
                    (
                        abs(margin),
                        i,
                    )
                )

    if not candidates:
        return

    _, center = min(
        candidates
    )

    start = max(
        0,
        center - radius,
    )

    end = min(
        len(live["states"]),
        len(restored["states"]),
        center + radius + 1,
    )

    print()
    print(
        "=" * 80
    )
    print(
        "7. CLOSEST SAFETY-BOUNDARY WINDOW"
    )
    print(
        "=" * 80
    )

    print(
        "step  time    "
        "live_x       rest_x       "
        "live_z       rest_z       "
        "live_margin      rest_margin"
    )

    for i in range(
        start,
        end,
    ):

        a = live["states"][i]

        b = restored["states"][i]

        ma = safety_margin(
            a,
            obstacle,
        )

        mb = safety_margin(
            b,
            obstacle,
        )

        ma_s = (
            "      OUTSIDE"
            if ma is None
            else f"{ma:+14.9e}"
        )

        mb_s = (
            "      OUTSIDE"
            if mb is None
            else f"{mb:+14.9e}"
        )

        print(
            f"{i:4d} "
            f"{i/mission.FREQ:6.3f} "
            f"{a[0]:+12.8f} "
            f"{b[0]:+12.8f} "
            f"{a[4]:+12.8f} "
            f"{b[4]:+12.8f} "
            f"{ma_s} "
            f"{mb_s}"
        )


def print_pid_at_first_action_difference(
    live,
    restored,
):

    step, _ = (
        find_first_action_divergence(
            live,
            restored,
        )
    )

    if step is None:
        return

    print()
    print(
        "=" * 80
    )
    print(
        "8. PID MEMORY AT FIRST ACTION DIVERGENCE"
    )
    print(
        "=" * 80
    )

    for when in [
        "pids_before",
        "pids_after",
    ]:

        print()
        print(when)

        for key in [
            "integral_pos_e",
            "last_rpy",
            "integral_rpy_e",
        ]:

            a = live[
                when
            ][step][key]

            b = restored[
                when
            ][step][key]

            print()
            print(key)

            print_vec(
                "LIVE",
                a,
            )

            print_vec(
                "REST",
                b,
            )

            print_vec(
                "DIFF",
                b - a,
            )


def main():

    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(
        "--scenario",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--delay",
        type=float,
        default=1.0,
    )

    args = parser.parse_args()

    s = (
        ground_truth.generate_scenario(
            args.scenario
        )
    )

    print(
        f"Scenario={args.scenario}, "
        f"delay={args.delay}s"
    )

    print(
        f"speed={s.speed:.6f}, "
        f"height={s.usual_height:.6f}"
    )

    print(
        f"box=["
        f"{s.box_x_min:.6f}, "
        f"{s.box_x_max:.6f}], "
        f"unsafe_z<="
        f"{s.unsafe_below_z:.6f}"
    )

    print(
        f"approval="
        f"{s.approval_time:.6f}, "
        f"command_z="
        f"{s.low_height:.6f}, "
        f"duration="
        f"{s.low_seconds:.6f}"
    )

    # ----------------------------------------
    # Same original simulator through approval.
    # ----------------------------------------

    live = trace_live(
        s,
        args.delay,
    )

    # ----------------------------------------
    # Snapshot + fresh restored simulator.
    # ----------------------------------------

    snapshot = (
        mission.capture_approval(s)
    )

    restored = trace_restored(
        snapshot,
        args.delay,
    )

    compare_initial_state(
        live,
        restored,
    )

    compare_initial_pid(
        live,
        restored,
    )

    print_first_divergence(
        live,
        restored,
    )

    print_state_error_summary(
        live,
        restored,
    )

    print_dispatch_window(
        live,
        restored,
    )

    print_safety_analysis(
        live,
        restored,
        snapshot.obstacle,
    )

    print_near_boundary(
        live,
        restored,
        snapshot.obstacle,
    )

    print_pid_at_first_action_difference(
        live,
        restored,
    )


if __name__ == "__main__":
    main()