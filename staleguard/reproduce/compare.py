"""Compare a fresh execution journal with the matching published scenario-delay trials."""

import argparse
import json
from pathlib import Path
from .common import Data, MISSIONS, write_csv, write_json


def action(row):
    event = row['event']
    if event.get('fallback_step') is not None:
        return 'recovery', event['fallback_step']
    if event.get('application_step') is not None:
        return 'command', event['application_step']
    return 'none', None


def compare(data_root, mission, run, out):
    data = Data(data_root)
    original, _, _ = data.policy_records(mission)
    index = {(r['seed'], round(r['delay_s'] * 50), r['event']['policy']): r for r in original}
    records = []
    seen = set()
    for line in (run / 'executions.jsonl').open():
        fresh = json.loads(line)
        key = fresh['seed'], round(fresh['delay_s'] * 50), fresh['event']['policy']
        if key in seen or key not in index:
            raise ValueError('Duplicate or unmatched fresh execution: ' + str(key))
        seen.add(key)
        old = index[key]
        same_action = action(fresh) == action(old)
        old_runtime = old['event'].get('computation_s')
        runtime = fresh['event'].get('computation_s')
        records.append(
            dict(
                seed=key[0],
                delay_s=key[1] / 50,
                policy=key[2],
                archived_action=str(action(old)),
                fresh_action=str(action(fresh)),
                action_equal=same_action,
                archived_safe=old['safe'],
                fresh_safe=fresh['safe'],
                archived_safe_completion=old['safe_completion'],
                fresh_safe_completion=fresh['safe_completion'],
                safety_equal=fresh['safe'] == old['safe'],
                completion_equal=fresh['safe_completion'] == old['safe_completion'],
                archived_runtime_s=old_runtime,
                fresh_runtime_s=runtime,
                recorded_runtime_equal=runtime == old_runtime,
                trajectory_hash_equal=(
                    fresh['physics_sha256'] == old['physics_sha256']
                    if 'physics_sha256' in fresh and 'physics_sha256' in old
                    else None
                ),
                interpretation=(
                    'Same action; check physical labels/hash'
                    if same_action
                    else 'Changed action; investigate readiness before attributing to algorithm'
                ),
            )
        )
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / 'comparisons.csv', records)
    summary = dict(
        mission=mission,
        matched_trials=len(records),
        action_differences=sum(not r['action_equal'] for r in records),
        safety_differences=sum(not r['safety_equal'] for r in records),
        completion_differences=sum(not r['completion_equal'] for r in records),
        same_action_hash_differences=sum(
            r['action_equal'] and r['trajectory_hash_equal'] is False for r in records
        ),
        scope='Matched recorded trials only; fresh measured runtimes are not held to archived values',
        input_sha256=data.used,
    )
    write_json(out / 'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path('data'))
    parser.add_argument('--mission', choices=MISSIONS, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.resolve().is_relative_to(args.data_root.resolve()):
        parser.error('--out must be outside immutable data')
    print(json.dumps(compare(args.data_root, args.mission, args.run, args.out), indent=2))


if __name__ == '__main__':
    main()
