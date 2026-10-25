"""Reject the turn at scheduled dispatch and use the existing safety recovery."""


def should_recover():
    """Unconditional, including D=0; no prediction or computation delay."""
    return True
