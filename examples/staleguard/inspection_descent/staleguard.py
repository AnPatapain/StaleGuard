
"""StaleGuard T0: predict validity over the delay grid at approval time.

No ground_truth import, future measurements, learned dynamics, or GT TTL.
This is an ORACLE-DYNAMICS sanity check, not a novel TTL estimator yet.
"""

from mission import DELAYS, counterfactual

def predict(snapshot, delays=DELAYS):
    if len(delays) == 0 or delays[0] != 0 or any(b <= a for a, b in zip(delays, delays[1:])):
        raise ValueError("Delay grid must start at 0 and increase")

    # The old mission must remain safe even if the command is never sent.
    if counterfactual(snapshot, None) != "SAFE":
        return {"classification": "INVALID_NOMINAL", "results": None,
                "first_bad_delay": None, "last_safe_delay": None}

    results = [counterfactual(snapshot, d) for d in delays]
    if results[0] != "SAFE":
        return {"classification": "INVALID_AT_APPROVAL", "results": results,
                "first_bad_delay": 0.0, "last_safe_delay": None}

    first_bad = next((i for i, status in enumerate(results)
                      if status != "SAFE"), None)
    if first_bad is None:
        return {"classification": "ALWAYS_SAFE", "results": results,
                "first_bad_delay": None, "last_safe_delay": delays[-1]}

    monotonic = all(status != "SAFE" for status in results[first_bad:])
    return {
        "classification": "FINITE_TTL" if monotonic else "NON_MONOTONIC",
        "results": results,
        "first_bad_delay": delays[first_bad],
        "last_safe_delay": delays[first_bad - 1],
    }