"""Figure 3: independent physical feasibility and sampled first-loss ordering."""

from .common import *


def main():
    args, data = arguments(__doc__)
    mission = 'underpass_inspection'
    forecasts, truth, grid, seeds = data.physical(mission)
    profiles = []
    boundaries = []
    categories = Counter()
    for seed in seeds:
        props = {
            p: boundary(
                [
                    truth[seed, 'recovery' if p == 'recovery' else 'command', round(d * 50)][
                        'safe_completion' if p == 'completion' else 'safe'
                    ]
                    for d in grid
                ],
                grid,
            )
            for p in PROPERTIES
        }
        for d in grid:
            profiles.append(
                dict(
                    seed=seed,
                    delay_s=d,
                    command_safe=truth[seed, 'command', round(d * 50)]['safe'],
                    command_complete=truth[seed, 'command', round(d * 50)]['safe_completion'],
                    recovery_safe=truth[seed, 'recovery', round(d * 50)]['safe'],
                )
            )
        c, r = props['command']['first_loss_s'], props['recovery']['first_loss_s']
        category = (
            'neither_observed'
            if c is None and r is None
            else (
                'recovery_only'
                if c is None
                else (
                    'command_only'
                    if r is None
                    else 'command_first' if c < r else 'recovery_first' if r < c else 'same_sample'
                )
            )
        )
        categories[category] += 1
        row = dict(
            seed=seed,
            ordering=category,
            gap_recovery_minus_command_s=None if c is None or r is None else round(r - c, 10),
        )
        for p, q in props.items():
            row.update({p + '_' + k: v for k, v in q.items() if k != 'labels'})
        successful = [d for d, v in zip(grid, props['completion']['labels']) if v]
        row['completion_last_feasible_sample_s'] = max(successful) if successful else None
        boundaries.append(row)
    write_csv(args.out / 'physical_profiles.csv', profiles)
    write_csv(args.out / 'per_scenario_boundaries.csv', boundaries)
    order = [
        'command_first',
        'recovery_first',
        'same_sample',
        'command_only',
        'recovery_only',
        'neither_observed',
    ]
    write_csv(
        args.out / 'boundary_ordering.csv',
        [
            dict(category=k, scenarios=categories[k], percentage=100 * categories[k] / len(seeds))
            for k in order
        ],
    )
    plt = plot_style()
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.1))
    for prop, field in PROPERTIES.items():
        axes[0].plot(
            grid,
            [100 * sum(r[field] for r in profiles if r['delay_s'] == d) / len(seeds) for d in grid],
            marker='o',
            ms=3,
            lw=1,
            label=prop.title(),
        )
    axes[0].set(
        xlabel='Maneuver-start delay (s)',
        ylabel='Physically feasible scenarios (%)',
        ylim=(-2, 102),
    )
    axes[0].legend(frameon=False)
    axes[0].grid(alpha=0.2)
    axes[1].bar(
        range(6),
        [categories[k] for k in order],
        color=['#777777', '#d87b2b', '#459279', '#b1b8bd', '#d6ad7a', '#dedede'],
    )
    axes[1].set_xticks(
        range(6),
        [
            'Command\nfirst',
            'Recovery\nfirst',
            'Same\nsample',
            'Command\nonly',
            'Recovery\nonly',
            'Neither',
        ],
        fontsize=8,
    )
    axes[1].set_ylabel('Scenarios')
    for i, k in enumerate(order):
        axes[1].text(i, categories[k] + 2, str(categories[k]), ha='center')
    fig.tight_layout()
    save_figure(fig, args.out, 'fig3')
    metadata(
        data,
        args.out,
        'Figure 3',
        scenarios=len(seeds),
        delays=grid,
        ordering=dict(categories),
        boundary='first sampled infeasible delay; no continuous-time inference',
    )


if __name__ == '__main__':
    main()
