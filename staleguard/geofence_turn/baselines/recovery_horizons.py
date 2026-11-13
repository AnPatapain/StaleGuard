"""Geofence predicates/references for the unchanged three-opportunity policy."""
from ..scenario import BODY_RADIUS_M,safety_only_reference,apply_turn_command
from ..baseline_benchmark.run import TTL_GRID
from staleguard.model.feasibility_profiles import predict as shared_predict

def adapter(s,snapshot):
    def reference(base,step,state,recovery):
        return safety_only_reference(base,step,state) if recovery else apply_turn_command(base,step,s.command)
    c=snapshot.config
    return dict(grid=list(TTL_GRID),horizon_steps=snapshot.horizon_steps,deadline_s=s.deadline_s,
        kind=0,geometry=[s.fence_x,BODY_RADIUS_M,c.d_min_m,0.,0.,0.],
        lower=list(s.command.checkpoint_lower),upper=list(s.command.checkpoint_upper),
        target=[0.,s.command.speed,0.],tolerance=c.velocity_tolerance_mps,
        speed_required=True,strict=False,dwell_s=c.dwell_s,
        prefix_safe=snapshot.prefix_min_clearance_m>=c.d_min_m,reference=reference)

def predict(s,snapshot,platform,**kwargs):return shared_predict(snapshot,platform,adapter(s,snapshot),**kwargs)
