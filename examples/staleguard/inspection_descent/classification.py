"""Shared scenario classification logic."""


def classify_delay_results(delays, results, nominal_safe=True):
    """
    Classify safety outcomes across dispatch delays.
    results:
        List of "SAFE", "UNSAFE", or "WAIT_UNSAFE".
        Any status other than "SAFE" is considered unsafe.
    nominal_safe:
        Whether the trajectory without the command is safe.
    """

    if not nominal_safe:
        return {
            "classification": "INVALID_NOMINAL",
            "results": None,
            "first_bad_delay": None,
            "last_safe_delay": None,
        }

    if not delays or len(delays) != len(results):
        raise ValueError(
            "delays and results must have equal, nonzero length"
        )

    # Immediate command execution must be safe.
    if results[0] != "SAFE":
        return {
            "classification": "INVALID_AT_APPROVAL",
            "results": results,
            "first_bad_delay": delays[0],
            "last_safe_delay": None,
        }

    # Find first unsafe delay.
    first_bad_index = next(
        (
            i
            for i, status in enumerate(results)
            if status != "SAFE"
        ),
        None,
    )

    if first_bad_index is None:
        return {
            "classification": "ALWAYS_SAFE",
            "results": results,
            "first_bad_delay": None,
            "last_safe_delay": delays[-1],
        }

    # Determine whether safety is monotonic with delay.
    monotonic = all(
        status != "SAFE"
        for status in results[first_bad_index:]
    )

    return {
        "classification": (
            "FINITE_TTL"
            if monotonic
            else "NON_MONOTONIC"
        ),
        "results": results,
        "first_bad_delay": delays[first_bad_index],
        "last_safe_delay": delays[first_bad_index - 1],
    }