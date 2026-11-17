"""Figure 4: Underpass safety/completion versus dispatch delay, with early-delay detail."""

from .common import *


def main():
    args, data = arguments(__doc__)
    rows, seeds, delays = data.policy_records('underpass_inspection')
    unsafe = []
    completion = []
    for d in delays:
        a = dict(delay_s=d)
        b = dict(delay_s=d)
        for p in POLICIES:
            group = [r for r in rows if r['delay_s'] == d and r['event']['policy'] == p]
            a[p] = 100 * sum(not r['safe'] for r in group) / len(group)
            b[p] = 100 * sum(r['safe_completion'] for r in group) / len(group)
        unsafe.append(a)
        completion.append(b)
    write_csv(args.out / 'unsafe_by_delay.csv', unsafe)
    write_csv(args.out / 'completion_by_delay.csv', completion)
    plt = plot_style()
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    colors = dict(
        zip(
            POLICIES,
            (
                '#777777',
                '#777777',
                '#d29120',
                '#348abd',
                '#9257a1',
                '#ba4b48',
                '#138656',
                '#138656',
            ),
        )
    )
    shown = [p for p in POLICIES if p not in ('fbs', 'staleguard_recovery_aware')]
    for ax, series, title in zip(
        axes, (unsafe, completion), ('Unsafe flights (%)', 'Safe completion (%)')
    ):
        inset = ax.inset_axes([0.13, 0.48, 0.35, 0.36])
        for p in shown:
            ax.plot(
                delays, [r[p] for r in series], '-o', lw=1, ms=3, color=colors[p], label=LABELS[p]
            )
            early = [r for r in series if r['delay_s'] <= 0.2]
            inset.plot(
                [r['delay_s'] for r in early],
                [r[p] for r in early],
                '-o',
                lw=0.75,
                ms=2,
                color=colors[p],
            )
        ax.axvline(1, color='#999999', ls='--', lw=0.8)
        ax.set(xlabel='Dispatch delay (s)', ylabel=title, xlim=(-0.03, 2.03), ylim=(-3, 103))
        ax.grid(alpha=0.2)
        inset.set_title('0–0.20 s', fontsize=7)
        inset.tick_params(labelsize=6)
        inset.grid(alpha=0.15)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=3, frameon=False, fontsize=7)
    fig.subplots_adjust(bottom=0.27, wspace=0.25)
    save_figure(fig, args.out, 'fig4')
    metadata(
        data,
        args.out,
        'Figure 4',
        scenarios=len(seeds),
        dispatch_delays=delays,
        prediction_horizon_s=1,
        overlaps=['FBS = No Recovery', 'Recovery-aware = Three-opportunity-aware'],
    )


if __name__ == '__main__':
    main()
