"""Inspection-specific periodic recoverability prediction, not command prediction.

Monitor every 0.10 s (the original mission's delay grid), with a 10 ms warmed
computation allowance. Strict next-tick availability permits one 20 ms tick;
predict patrol for 6 ticks/120 ms, then the original usual-altitude fallback.
Twenty-five warmed checks on development seeds 0--4 measured 3.35 ms mean and
3.85 ms maximum before evaluation. The allowance is an engineering assumption;
record actual runtimes and overruns, never shorten a measured latency.

Important degeneracy: before dispatch, usual-altitude fallback changes no
nominal reference. For a nominally safe queued descent, this prediction normally
remains safe and PPR does not reject a potentially unsafe future descent. It is
not silently replaced with a queued-command or TTL checker.
"""
import math
from model.lightweight import LightweightPredictiveModel
from ..scenario import CTRL_FREQ_HZ,unsafe,unsafe_segment,safety_only_reference

INTERVAL_STEPS=5
COMPUTATION_BOUND_S=.010
LATENCY_BOUND_STEPS=math.floor(COMPUTATION_BOUND_S*CTRL_FREQ_HZ)+1


def predict_recovery(snapshot,usual_height,platform):
    model=LightweightPredictiveModel(*platform)
    try:
        ctrl,state,info,reference=model.start(snapshot)
        wait=INTERVAL_STEPS+LATENCY_BOUND_STEPS
        safe=not unsafe(state,snapshot)
        simulated=0
        for step in range(snapshot.horizon_steps):
            if not safe:
                break
            if step==wait:
                model.set_reference(ctrl,safety_only_reference(reference,step,usual_height))
            previous=state
            state,_,info=model.step(ctrl.select_action(state,info),state,info)
            safe=not unsafe_segment(previous,state,snapshot);simulated+=1
        return dict(safe=safe,recovery_after_steps=wait if wait<snapshot.horizon_steps else None,
                    horizon_steps=snapshot.horizon_steps,predicted_steps=simulated)
    finally:
        model.close({})
