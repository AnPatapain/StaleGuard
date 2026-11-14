"""Table II: three-property prediction accuracy and recorded computational readiness."""

from .common import *


def main():
    args, data = arguments(__doc__)
    parta = []
    partb = []
    predicted = []
    physical = []
    errors = []
    runtime = []
    for mission in MISSIONS:
        forecasts, truth, grid, seeds = data.physical(mission)
        acc, timing, profiles = profile_metrics(forecasts, truth, grid, seeds)
        delays = data.run(mission + '/benchmark')['protocol']['delays']
        for seed in seeds:
            f = forecasts[str(seed)]
            ready = first_ready_step(f['computation_s']) / 50
            for row in f['results']:
                predicted.append(
                    dict(
                        mission=mission,
                        seed=seed,
                        **{k: row[k] for k in ('delay_s', *PROPERTIES.values())},
                    )
                )
            for d in grid:
                physical.append(
                    dict(
                        mission=mission,
                        seed=seed,
                        delay_s=d,
                        **{
                            field: truth[
                                seed, 'recovery' if prop == 'recovery' else 'command', round(d * 50)
                            ]['safe_completion' if prop == 'completion' else 'safe']
                            for prop, field in PROPERTIES.items()
                        },
                    )
                )
            status = next(r for r in timing['physical_readiness_per_scenario'] if r['seed'] == seed)
            runtime.append(
                dict(
                    mission=mission,
                    computation_s=f['computation_s'],
                    ready_control_step=first_ready_step(f['computation_s']),
                    **status,
                )
            )
        for prop, q in acc.items():
            parta.append(
                dict(
                    mission=mission,
                    property=prop,
                    scenarios=len(seeds),
                    physical_comparisons=q['physical_comparisons'],
                    tp=q['counts']['tp'],
                    tn=q['counts']['tn'],
                    false_feasible_count=q['counts']['fp'],
                    false_reject_count=q['counts']['fn'],
                    accuracy_pct=100 * q['accuracy'],
                    false_feasible_pct=(
                        None if q['false_feasible_rate'] is None else 100 * q['false_feasible_rate']
                    ),
                    false_reject_pct=(
                        None if q['false_reject_rate'] is None else 100 * q['false_reject_rate']
                    ),
                    boundary_mae_ms=(
                        None if q['boundary_mae_s'] is None else 1000 * q['boundary_mae_s']
                    ),
                    boundary_median_ae_ms=(
                        None
                        if q['boundary_median_absolute_error_s'] is None
                        else 1000 * q['boundary_median_absolute_error_s']
                    ),
                    boundary_pairs=q['boundary_pairs'],
                    predicted_initially_infeasible=q['initial_cases'].get('predicted', 0),
                    physical_initially_infeasible=q['initial_cases'].get('physical', 0),
                    predicted_censored=q['right_censored'].get('predicted', 0),
                    physical_censored=q['right_censored'].get('physical', 0),
                    predicted_non_monotonic=q['non_monotonic'].get('predicted', 0),
                    physical_non_monotonic=q['non_monotonic'].get('physical', 0),
                )
            )
            for row in q['per_scenario']:
                errors.append(
                    dict(
                        mission=mission,
                        property=prop,
                        seed=row['seed'],
                        predicted_first_loss_s=row['predicted']['first_loss_s'],
                        physical_first_loss_s=row['physical']['first_loss_s'],
                        signed_error_s=row['sampled_boundary_error_s'],
                        absolute_error_s=(
                            None
                            if row['sampled_boundary_error_s'] is None
                            else abs(row['sampled_boundary_error_s'])
                        ),
                    )
                )
        available = sum(
            forecasts[str(seed)]['computation_s'] <= d for seed in seeds for d in delays
        )
        partb.append(
            dict(
                mission=mission,
                timed_forecasts=len(seeds),
                runtime_median_ms=1000 * timing['runtime_median_s'],
                runtime_p95_ms=1000 * timing['runtime_p95_s'],
                available_by_dispatch=available,
                dispatch_cases=len(seeds) * len(delays),
                ready_by_dispatch_pct=100 * available / (len(seeds) * len(delays)),
                observed_recovery_losses=timing['observed_physical_recovery_boundaries'],
                late_at_recovery_loss=(
                    timing['physical_readiness'].get('late_at_or_after_first_unsafe_sample', 0)
                    if timing['observed_physical_recovery_boundaries']
                    else None
                ),
                late_recovery_loss_pct=(
                    None
                    if timing['late_recovery_loss_rate_over_observed'] is None
                    else 100 * timing['late_recovery_loss_rate_over_observed']
                ),
                recovery_loss_censored=timing['physical_readiness'].get(
                    'recovery_loss_unobserved', 0
                ),
                ready_in_unresolved_bracket=timing['physical_readiness'].get(
                    'ready_inside_unresolved_boundary_bracket', 0
                ),
            )
        )
    for name, rows in [
        ('prediction_labels', predicted),
        ('physical_labels', physical),
        ('boundary_errors', errors),
        ('runtime_readiness', runtime),
        ('table2_part_a', parta),
        ('table2_part_b', partb),
    ]:
        write_csv(args.out / (name + '.csv'), rows)
    table_text(args.out, 'table2_part_a', parta)
    table_text(args.out, 'table2_part_b', partb)
    (args.out / 'table2.md').write_text(
        (args.out / 'table2_part_a.md').read_text()
        + '\n'
        + (args.out / 'table2_part_b.md').read_text()
    )
    metadata(
        data,
        args.out,
        'Table II',
        runtime='original offline measurements, replayed at 50 Hz; no onboard validation',
        false_feasible_denominator='physically infeasible labels',
        false_reject_denominator='physically feasible labels',
    )


if __name__ == '__main__':
    main()
