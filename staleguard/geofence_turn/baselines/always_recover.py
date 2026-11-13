"""Cancel at approval and use the existing safety recovery, including D=0."""


def should_recover():
    """Unconditional, including D=0; no prediction or computation delay."""
    return True
