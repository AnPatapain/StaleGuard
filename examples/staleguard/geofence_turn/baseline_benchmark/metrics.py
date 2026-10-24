"""Summarize only complete paired blocks, retaining explicit partial-run counts."""
from collections import Counter, defaultdict
from enum import Enum
import json
from pathlib import Path

POLICIES = ('no_recovery', 'staleguard', 'use_time_recheck', 'fixed_ttl')


# Reported counts for each baseline, using only complete scenario-delay groups.
# The three mission outcomes are mutually exclusive. Fallback can overlap any
# outcome: activating it does not guarantee safety or mission completion.
class Metric(Enum):
    # Number of evaluated flights.
    FLIGHTS = 'flights'
    # Completed the checkpoint requirements before the deadline and stayed safe.
    SAFE_COMPLETION = 'safe_completion'
    # Stayed safe but did not complete the checkpoint requirements before the deadline.
    SAFE_NONCOMPLETION = 'safe_noncompletion'
    # Violated the required UAV-body clearance at any time during the flight.
    UNSAFE = 'unsafe'
    # Activated the safety fallback, regardless of the flight outcome.
    FALLBACK = 'fallback'


def flight_metrics(row):
    """Convert one saved flight into contributions to the counts defined above."""
    values = {metric.value: 0 for metric in Metric}
    values[Metric.FLIGHTS.value] = 1
    values[Metric.FALLBACK.value] = int(row['event']['fallback_step'] is not None)
    if row['safe_completion']:
        values[Metric.SAFE_COMPLETION.value] = 1
    elif row['safe']:
        values[Metric.SAFE_NONCOMPLETION.value] = 1
    else:
        values[Metric.UNSAFE.value] = 1
    return values


def summarize(directory):
    """Group matched flights and sum their metric contributions per baseline."""
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory/'executions.jsonl').read_text().splitlines()]
    blocks = defaultdict(dict)
    for row in rows:
        key = (row['seed'], row['delay_s'])
        policy = row['event']['policy']
        if policy in blocks[key]:
            raise ValueError('Duplicate flight in policy journal')
        blocks[key][policy] = row
    paired = {key:value for key,value in blocks.items() if set(value)==set(POLICIES)}
    counts = {p:Counter({metric.value: 0 for metric in Metric}) for p in POLICIES}
    for block in paired.values():
        for policy, row in block.items():
            counts[policy].update(flight_metrics(row))
    return dict(recorded_flights=len(rows), paired_blocks=len(paired),
                partial_blocks=[dict(seed=k[0], delay_s=k[1], recorded_policies=list(v))
                                for k,v in blocks.items() if k not in paired],
                paired_policy_counts={p:dict(c) for p,c in counts.items()})


if __name__ == '__main__':
    import sys
    print(json.dumps(summarize(sys.argv[1]), indent=2))
