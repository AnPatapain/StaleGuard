"""Inspection predicates/references for the unchanged three-opportunity policy."""

import numpy as np
from ..scenario import (
    DELAY_GRID,
    MISSION_DEADLINE_S,
    SCAN_DWELL_S,
    inspection_x,
    safety_only_reference,
    apply_descent_command,
    unsafe,
)
from staleguard.model.feasibility_profiles import predict as shared_predict


def adapter(s, snapshot):
    def reference(base, step, state, recovery):
        return (
            safety_only_reference(base, step, s.usual_height)
            if recovery
            else apply_descent_command(base, step, snapshot.low_steps, snapshot.low_height)
        )

    x = inspection_x(s)
    return dict(
        grid=[round(float(d), 2) for d in DELAY_GRID],
        horizon_steps=round(MISSION_DEADLINE_S * 50),
        deadline_s=MISSION_DEADLINE_S,
        kind=1,
        geometry=[
            s.box_x_min,
            s.box_x_max,
            -s.box_y_limit,
            s.box_y_limit,
            -np.inf,
            s.unsafe_below_z,
        ],
        lower=[x - 0.12, -0.20, s.low_height - 0.08],
        upper=[x + 0.12, 0.20, s.low_height + 0.08],
        target=[0.0, 0.0, 0.0],
        tolerance=0.0,
        speed_required=False,
        strict=True,
        dwell_s=SCAN_DWELL_S,
        prefix_safe=not unsafe(snapshot.state, snapshot),
        reference=reference,
    )


def predict(s, snapshot, platform, **kwargs):
    return shared_predict(snapshot, platform, adapter(s, snapshot), **kwargs)
