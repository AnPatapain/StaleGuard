"""Paper figure: No-Recovery dispatch delay versus unsafe/safe-completion rates.

Run from the repository root:
    python -m staleguard.plot_dispatch_delay_effect

Only the two original six-policy evaluation journals are read. No simulation or
prediction is run. Lines join measured points at their actual delay positions;
there is no smoothing, interpolation, or raw-completion series.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import os
import tempfile

ROOT = Path(__file__).resolve().parent
INSPECTION_DELAYS = tuple(i / 10 for i in range(11))
GEOFENCE_DELAYS = (
    0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20,
    .24, .30, .40, .44, .50, .60, .64, .70, .80, .84, .90, 1.,
)
MISSIONS = (
    dict(name="Inspection-Descent", directory="inspection_descent", scenarios=276,
         delays=INSPECTION_DELAYS, evaluations=3036, unsafe=1841, safe_completions=930),
    dict(name="Geofence Turn", directory="geofence_turn", scenarios=300,
         delays=GEOFENCE_DELAYS, evaluations=6900, unsafe=2549, safe_completions=1692),
)
CSV_FIELDS = (
    "mission", "dispatch_delay_s", "evaluated_scenarios", "unsafe_executions",
    "safe_completions", "unsafe_rate_pct", "safe_completion_rate_pct",
)


def load_rates(mission):
    """Validate the frozen journal and derive counts using authoritative flags."""
    folder = ROOT / mission["directory"] / "results/evaluation/baseline_benchmark"
    run = json.loads((folder / "run.json").read_text())
    metrics = json.loads((folder / "metrics.json").read_text())
    protocol = run["protocol"]
    policies = {"no_recovery", "always_recover", "fixed_ttl", "fbs",
                "use_time_recheck", "staleguard"}
    seeds = {s["seed"] for s in protocol["scenarios"]}
    if (run["status"] != "complete" or set(protocol["policies"]) != policies
            or tuple(protocol["delays"]) != mission["delays"]
            or len(protocol["scenarios"]) != len(seeds)
            or len(seeds) != mission["scenarios"]):
        raise ValueError(f'{folder}: not the expected complete six-policy protocol')

    counts = {d: dict(n=0, unsafe=0, safe_completions=0) for d in mission["delays"]}
    seen = set()
    digest = hashlib.sha256()
    size = 0
    with (folder / "executions.jsonl").open("rb") as stream:
        for line_number, line in enumerate(stream, 1):
            digest.update(line)
            size += len(line)
            record = json.loads(line)
            if record["event"]["policy"] != "no_recovery":
                continue
            seed, delay = record["seed"], record["delay_s"]
            key = seed, delay
            if seed not in seeds or delay not in counts or key in seen:
                raise ValueError(f'{folder}: duplicate/unexpected No-Recovery key {key} at line {line_number}')
            seen.add(key)
            if any(type(record[f]) is not bool for f in ("safe", "completed", "safe_completion")):
                raise ValueError(f'{folder}: non-boolean outcome at {key}')
            # Raw completed is checked for consistency only; it is never plotted.
            if record["safe_completion"] != (record["safe"] and record["completed"]):
                raise ValueError(f'{folder}: inconsistent safe_completion at {key}')
            if record["event"]["fallback_step"] is not None:
                raise ValueError(f'{folder}: recovery found in No-Recovery record {key}')
            if delay == 0 and not (record["safe"] and record["completed"]):
                raise ValueError(f'{folder}: D=0 did not safely complete for seed {seed}')
            counts[delay]["n"] += 1
            counts[delay]["unsafe"] += not record["safe"]
            counts[delay]["safe_completions"] += record["safe_completion"]

    if size != run["journal_bytes"] or digest.hexdigest() != run["journal_sha256"]:
        raise ValueError(f'{folder}: execution journal differs from its saved checksum')
    expected = {(seed, delay) for seed in seeds for delay in mission["delays"]}
    if seen != expected:
        raise ValueError(f'{folder}: missing No-Recovery scenario/delay observations')
    totals = (len(seen), sum(c["unsafe"] for c in counts.values()),
              sum(c["safe_completions"] for c in counts.values()))
    required = (mission["evaluations"], mission["unsafe"], mission["safe_completions"])
    if totals != required:
        raise ValueError(f'{folder}: evaluation/unsafe/safe-completion totals {totals}, expected {required}')

    rows = []
    for delay, count in counts.items():
        if count["n"] != mission["scenarios"]:
            raise ValueError(f'{folder}: D={delay} has {count["n"]} observations, expected {mission["scenarios"]}')
        saved = metrics["by_delay"][str(delay)]["by_policy"]["no_recovery"]
        if (saved["flights"], saved["unsafe"], saved["safe_completion"]) != (
                count["n"], count["unsafe"], count["safe_completions"]):
            raise ValueError(f'{folder}: calculated counts differ from saved metrics at D={delay}')
        rows.append(dict(mission=mission["name"], dispatch_delay_s=delay,
                         evaluated_scenarios=count["n"], unsafe_executions=count["unsafe"],
                         safe_completions=count["safe_completions"],
                         unsafe_rate_pct=100 * count["unsafe"] / count["n"],
                         safe_completion_rate_pct=100 * count["safe_completions"] / count["n"]))
    print(f'{mission["name"]}: validated {len(seeds)} scenarios x {len(rows)} delays; '
          f'{totals[0]:,} evaluations, {totals[1]:,} unsafe, {totals[2]:,} safe completions; D=0 all pass')
    return rows


def draw_figure(groups, output, small_text=False):
    """Two equal axes, measured delay positions, and one shared legend."""
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "staleguard-paper-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FormatStrFormatter, PercentFormatter

    style = {
        "font.family": "DejaVu Sans", "font.size": 8.5,
        "axes.labelsize": 9, "axes.titlesize": 9.5,
        "axes.labelcolor": "#32383F", "text.color": "#252B32",
        "xtick.color": "#56606A", "ytick.color": "#56606A",
        "axes.edgecolor": "#B5BCC3", "axes.linewidth": .55,
        "figure.facecolor": "white", "axes.facecolor": "white",
        "pdf.fonttype": 42, "ps.fonttype": 42, "svg.fonttype": "none",
        "svg.hashsalt": "staleguard-dispatch-delay-effect",
    }
    if small_text:
        # IEEEtran's regular conference mode uses Times Roman (TeX family ptm).
        # Nimbus Roman is its URW Times-compatible face, not Times New Roman.
        from matplotlib.font_manager import FontProperties, findfont
        findfont(FontProperties(family="Nimbus Roman", weight="normal"), fallback_to_default=False)
        style.update({"font.family": "serif", "font.serif": ["Nimbus Roman", "Times"],
                      "font.size": 9, "font.weight": "normal", "font.style": "normal",
                      "axes.labelsize": 9, "axes.titlesize": 10,
                      "axes.labelweight": "normal", "axes.titleweight": "normal",
                      "xtick.labelsize": 8, "ytick.labelsize": 8,
                      "text.usetex": False, "pdf.use14corefonts": True})
    stem = "dispatch_delay_effect_small_text" if small_text else "dispatch_delay_effect"
    with plt.rc_context(style):
        fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.75), sharex=True, sharey=True)
        fig.subplots_adjust(left=.106, right=.98, bottom=.205, top=.755, wspace=.19)
        for panel, (ax, rows) in enumerate(zip(axes, groups)):
            delays = [r["dispatch_delay_s"] for r in rows]
            ax.plot(delays, [r["unsafe_rate_pct"] for r in rows],
                    label="Unsafe rate", color="#C76759", linewidth=1.35,
                    linestyle="-" if small_text else (0, (3.4, 1.8)), marker="o", markersize=3.0,
                    markeredgecolor="white", markeredgewidth=.4, clip_on=False, zorder=3)
            ax.plot(delays, [r["safe_completion_rate_pct"] for r in rows],
                    label="Safe-completion rate", color="#24567A", linewidth=1.35,
                    linestyle="-", marker="D", markersize=2.7,
                    markeredgecolor="white", markeredgewidth=.4, clip_on=False, zorder=4)
            ax.set(xlim=(0, 1), ylim=(0, 100), xlabel="Dispatch delay (s)",
                   xticks=[0, .2, .4, .6, .8, 1], yticks=[0, 20, 40, 60, 80, 100])
            ax.xaxis.set_major_formatter(FormatStrFormatter("%.1f"))
            ax.yaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))
            ax.set_axisbelow(True)
            ax.grid(axis="y", color="#E9EDF0", linewidth=.55)
            ax.tick_params(axis="both", length=2.4, width=.55, pad=3.5)
            ax.spines[["top", "right"]].set_visible(False)
            ax.spines["bottom"].set_zorder(0)
            title = rows[0]["mission"] if small_text else f'({chr(97 + panel)}) {rows[0]["mission"]}'
            ax.set_title(title, loc="left", pad=11)
            ax.annotate(f'{rows[0]["evaluated_scenarios"]} scenarios', xy=(1, 1),
                        xycoords="axes fraction", xytext=(0, 11), textcoords="offset points",
                        ha="right", va="baseline", fontsize=8 if small_text else 7.5, color="#68737D")
            ax.xaxis.labelpad = 7
        axes[0].set_ylabel("Percentage of flights", labelpad=7)
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(.543, .995),
                   ncol=2, frameon=False, fontsize=9, handlelength=2.4,
                   columnspacing=2.4, handletextpad=.7)
        # Keep the canvas at precisely 7.16 inches; tight cropping changes width.
        pdf_metadata = {
            "Title": "Effect of Dispatch Delay on Flight Safety and Safe Mission Completion",
            "Creator": "StaleGuard plot_dispatch_delay_effect.py",
            "CreationDate": None, "ModDate": None}
        if small_text:
            # Matplotlib's direct CFF export mislabels the embedded font on this
            # version. Export genuine Times metrics, then embed the system's URW
            # Times outlines with Ghostscript. Text/lines remain vector objects;
            # no LaTeX installation or font substitution to a sans face is needed.
            import subprocess
            with tempfile.TemporaryDirectory(prefix="staleguard-times-") as temporary:
                source = Path(temporary) / "times.pdf"
                fig.savefig(source, metadata=pdf_metadata)
                subprocess.run([
                    "gs", "-q", "-dBATCH", "-dNOPAUSE", "-sDEVICE=pdfwrite",
                    "-dCompatibilityLevel=1.4", "-dEmbedAllFonts=true", "-dSubsetFonts=true",
                    "-dAutoRotatePages=/None", f"-sOutputFile={(output / f'{stem}.pdf').resolve()}",
                    "-c", "<</NeverEmbed []>> setdistillerparams", "-f", str(source),
                ], check=True)
        else:
            fig.savefig(output / f"{stem}.pdf", metadata=pdf_metadata)
        fig.savefig(output / f"{stem}.svg", metadata={"Date": None})
        fig.savefig(output / f"{stem}.png", dpi=600)
        plt.close(fig)
    return stem


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "output/pdf",
                        help="Figure/CSV destination; canonical experiment inputs are fixed")
    parser.add_argument("--small-text", action="store_true",
                        help="Save IEEE serif typography without panel letters; preserve original outputs (Nimbus Roman and Ghostscript required)")
    args = parser.parse_args()
    try:
        groups = [load_rates(mission) for mission in MISSIONS]
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Data validation failed: {error}\n")
    args.out.mkdir(parents=True, exist_ok=True)
    if not args.small_text:
        with (args.out / "dispatch_delay_effect.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(row for group in groups for row in group)
    stem = draw_figure(groups, args.out, small_text=args.small_text)
    for suffix in (("pdf", "svg", "png") if args.small_text else ("pdf", "svg", "png", "csv")):
        print(args.out / f"{stem}.{suffix}")



if __name__ == "__main__":
    main()
