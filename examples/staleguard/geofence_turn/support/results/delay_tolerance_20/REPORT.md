# Task 1: delay tolerance of the current calibration

Keeping calibration unchanged, I reused five cases, calibrated 15 additional cases, and ran 380 actual No-Recovery flights. The resulting delay tolerance is narrow: 16/20 scenarios complete safely at some positive delay, nine retain a success at or above 0.1 s, and none at any tested delay of 0.2 s or more. In measured seed 1, waiting 0.12 s shifts the northbound flight east of the checkpoint while remaining safe. Checkpoint failure occurs first in 15 cases, safety failure first in three, and both together in two. This supports the hypothesis in this sample, particularly the checkpoint limitation. It does not establish how an alternative calibration would perform. Use the fine delay grid for the next development comparison and decide the desired delay range before broader calibration.

## What was tested

The UAV patrols east, executes its approved left turn, and continues north. A delay shifts that northbound path east without moving the checkpoint or changing the command. Completion requires the correct position and northbound velocity for 0.20 s before the fixed deadline; safety requires 5 cm of body clearance throughout the flight.

Calibration still chooses the latest safe, completing approval on its 0.2 s grid. Consecutive new seeds 5–21 produced two checkpoint-fit geometry rejections (5 and 13); the other 15 calibrated. Each accepted case was tested at 0–0.20 s in 0.02 s increments and 0.30–1.00 s in 0.10 s increments, using actual PyBullet flights observed at 1000 Hz.

## Measured results

| Requested measure | Observed result |
|---|---|
| Calibration acceptance | **20/22 = 90.9%** overall; **15/17 = 88.2%** for new seeds |
| At least one safe completion at D > 0 | **16/20 = 80%** |
| At least one safe completion at D ≥ 0.1 s | **9/20 = 45%** |
| Last safely completable tested delay | **0–0.18 s**, median **0.08 s** |
| First unsafe tested delay | **0.04–1.00 s** in 19 cases; seed 16 stayed safe through 1.00 s |
| First checkpoint-objective loss | **0.02–0.30 s**; all 20 eventually lose completion |
| Predispatch violations | **5/20 scenarios**, occurring in **12/380 flights**, at delays 0.7–1.0 s |
| Lost and later regained safety / completion / safe completion | **0 / 1 / 0 scenarios**, respectively |

Rates count scenarios. Seeds 6, 8, 11 and 20 have no positive-delay safe completion. Individual results are in the [appendix](TECHNICAL_APPENDIX.md#individual-scenario-results) and [CSV](scenario_metrics.csv).

| Dispatch delay | Safe completion | Safe non-completion | Unsafe |
|---|---:|---:|---:|
| 0.00 s | 20/20 | 0/20 | 0/20 |
| 0.02 s | 16/20 | 4/20 | 0/20 |
| 0.04 s | 13/20 | 5/20 | 2/20 |
| 0.10 s | 9/20 | 7/20 | 4/20 |
| 0.18 s | 2/20 | 11/20 | 7/20 |
| 0.20 s | 0/20 | 13/20 | 7/20 |
| 0.50 s | 0/20 | 7/20 | 13/20 |
| 1.00 s | 0/20 | 1/20 | 19/20 |

## Which limit matters?

At the first loss of safe mission completion, **15 cases miss the objective while safe**, **three complete while unsafe**, and **two lose both**. Seed 16 stays safe at a 1 s delay but loses completion at 0.06 s, showing that clearance is not always the limiting factor.

At their first checkpoint failure, 14 cases pass entirely east of the box. Six enter at acceptable velocity but remain inside for only **0.073–0.190 s**, below the required 0.20 s. Seed 1 first misses at 0.12 s, passing at least **2.15 mm east**, but first becomes unsafe at 0.50 s. Seed 3 reverses that order: unsafe at 0.04 s, checkpoint loss at 0.06 s. Predispatch violations occur at larger delays; none explains a case's first unsafe result.

## Is there sufficient positive-delay variation?

**Yes for a fine-grid development comparison below 0.2 s; no for broad completion variation over 0–1 s.** The old 0.2/0.6/1.0 s positive-delay grid yields no No-Recovery safe completions, although safety still varies. Retain the fine grid. A different approval-buffer rule would be a separate scientific decision; none was introduced here.

These 20 feasibility-selected cases do not establish a six-policy ranking. Thresholds are tested grid points. Seed 11 regains completion at 0.04 s after losing it at 0.02 s, but clearance is **49.986 mm**, **0.014 mm below** the safety requirement. Two uninstrumented repeats reproduced this sensitive boundary; safe completion never returns. All 34 focused tests passed; original D=0 results and archives remain unchanged.
