"""Historical rule: fallback at approval for D>0; execute normally for D=0."""
def decision(approval_step, dispatch_step):
    return approval_step < dispatch_step, approval_step
