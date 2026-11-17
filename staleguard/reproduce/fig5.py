"""Figure 5: Underpass safety–completion trade-off, without artificial data offsets."""

from .common import *


def main():
    args, data = arguments(__doc__)
    rows, seeds, delays = data.policy_records('underpass_inspection')
    summary = []
    for p in POLICIES:
        group = [r for r in rows if r['event']['policy'] == p]
        summary.append(
            dict(
                policy=p,
                label=LABELS[p],
                flights=len(group),
                unsafe=sum(not r['safe'] for r in group),
                safe_completion=sum(r['safe_completion'] for r in group),
                unsafe_pct=100 * sum(not r['safe'] for r in group) / len(group),
                safe_completion_pct=100 * sum(r['safe_completion'] for r in group) / len(group),
            )
        )
    write_csv(args.out / 'policy_tradeoff.csv', summary)
    plt = plot_style()
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    grouped = defaultdict(list)
    for r in summary:
        grouped[r['unsafe_pct'], r['safe_completion_pct']].append(r['label'])
    offsets = [(5, 8), (5, -18), (5, 8), (5, 12), (5, 25), (5, -18)]
    for i, ((x, y), labels) in enumerate(grouped.items()):
        ax.scatter(x, y, s=38)
        ax.annotate(
            "\n".join(labels), (x, y), xytext=offsets[i], textcoords='offset points', fontsize=7
        )
    ax.set(
        xlabel='Unsafe flights (%) ↓', ylabel='Safe completion (%) ↑', xlim=(-1, 53), ylim=(30, 64)
    )
    ax.grid(alpha=0.2)
    save_figure(fig, args.out, 'fig5')
    metadata(
        data,
        args.out,
        'Figure 5',
        scenarios=len(seeds),
        trials_per_policy=len(seeds) * len(delays),
        scientific_points_unshifted=True,
    )


if __name__ == '__main__':
    main()
