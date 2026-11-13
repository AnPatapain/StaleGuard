"""Unconditionally cancel and recover at approval, including simultaneous dispatch.

Archived historical executions retain their documented D=0 exception.
"""
def decision(approval_step, dispatch_step):
    return True, approval_step
