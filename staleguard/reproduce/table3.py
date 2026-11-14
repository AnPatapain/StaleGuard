"""Table III: eight-policy outcomes and whole-scenario paired bootstrap."""

from .common import *


def main():
    args, data = arguments(__doc__)
    outcomes = []
    summary = []
    comparisons = []
    reuse = {}
    for mission in MISSIONS:
        raw, seeds, delays = data.policy_records(mission)
        rows = [compact(r) for r in raw]
        if mission != 'underpass_inspection':
            reuse[mission] = data.verify_reuse(mission)
        outcomes.extend(dict(mission=mission, **r) for r in rows)
        for policy in POLICIES:
            q = group_metrics([r for r in rows if r['policy'] == policy])
            summary.append(
                dict(
                    mission=mission,
                    policy=policy,
                    label=LABELS[policy],
                    scenarios=len(seeds),
                    dispatch_delays=len(delays),
                    **{k: v for k, v in q.items() if k != 'source_kind_counts'},
                    unsafe_pct=100 * q['unsafe_rate'],
                    safe_completion_pct=100 * q['safe_completion_rate'],
                )
            )
        for other, q in paired(rows, seeds, delays).items():
            comparisons.append(
                dict(mission=mission, full='staleguard_dual_horizon', comparator=other, **q)
            )
    write_csv(args.out / 'policy_outcomes.csv', outcomes)
    write_csv(args.out / 'policy_summary.csv', summary)
    write_csv(args.out / 'paired_comparisons.csv', comparisons)
    write_csv(args.out / 'table3.csv', summary)
    table_text(args.out, 'table3', summary)
    metadata(
        data,
        args.out,
        'Table III',
        physical_reuse_verified=reuse,
        bootstrap=dict(unit='whole scenario', resamples=2000, seed=2027),
    )


if __name__ == '__main__':
    main()
