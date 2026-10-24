"""Summarize only complete paired blocks, retaining explicit partial-run counts."""
from collections import Counter, defaultdict
import json
from pathlib import Path

POLICIES = ('no_recovery', 'staleguard', 'use_time_recheck', 'fixed_ttl')


def summarize(directory):
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
    counts = {p:Counter(dict(flights=0, safe_completion=0, safe_noncompletion=0, unsafe=0, fallback=0)) for p in POLICIES}
    for block in paired.values():
        for policy, row in block.items():
            category = 'safe_completion' if row['safe_completion'] else 'safe_noncompletion' if row['safe'] else 'unsafe'
            counts[policy]['flights'] += 1
            counts[policy][category] += 1
            counts[policy]['fallback'] += int(row['event']['fallback_step'] is not None)
    return dict(recorded_flights=len(rows), paired_blocks=len(paired),
                partial_blocks=[dict(seed=k[0], delay_s=k[1], recorded_policies=list(v))
                                for k,v in blocks.items() if k not in paired],
                paired_policy_counts={p:dict(c) for p,c in counts.items()})


if __name__ == '__main__':
    import sys
    print(json.dumps(summarize(sys.argv[1]), indent=2))
