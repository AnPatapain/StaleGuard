# Always Recover: confirmed cross-mission inconsistency

The audit confirms three different policies under the same name. A Geofence UAV keeps moving until dispatch, then brakes; an Underpass UAV abandons inspection and turns East immediately at approval. Inspection recovers at approval only for positive delays and executes the command at zero delay. The recorded cases agree with those code paths: Inspection has 276 command executions and 2,760 recoveries; Geofence has 6,900 dispatch-time recoveries, including 500 unsafe cases; Underpass has 5,200 approval-time recoveries, all safe. This is a baseline-definition inconsistency, not a plotting error. A footnote does not make the row a unified baseline. Use distinct names for the historical policies, or define approval-time unconditional recovery and produce versioned corrected Always Recover results for the affected missions. No policy code, historical artifacts, or physical simulations were changed by this audit.

## Verified behavior and recorded outcomes

| Mission | At D=0 | At positive D | Recorded cases | Recoveries | Unsafe | Safe mission completions |
|---|---|---|---:|---:|---:|---:|
| Inspection Descent | Execute approved descent | Recover at approval | 3,036 | 2,760 | 0 | 276 (9.09%) |
| Geofence Turn | Recover at dispatch, coincident with approval | Continue nominal flight, then recover at dispatch | 6,900 | 6,900 | 500 (7.25%) | 0 |
| Underpass Inspection | Recover at approval; cancel command | Recover at approval; cancel command | 5,200 | 5,200 | 0 | 0 |

All 276 Inspection safe completions occur at D=0, when this policy does not recover. Geofence has 300 approval-time recoveries only because its 300 D=0 cases have approval and dispatch at the same instant; none of its 6,600 positive-delay cases recovers before dispatch. Underpass recovers at approval in all cases, including its 400 zero-delay cases.

Geofence's 500 unsafe recoveries span **124 scenarios**. First violation is before recovery in 192 cases, at the recovery instant in two, and after recovery starts in 306. Its fallback requests hover at the current position: the actual aircraft must brake under PID/PyBullet dynamics. Therefore rejection at dispatch can occur after a violation, or too late to stop before the fence. This is not evidence that its approval-time recovery has a 7.25% unsafe rate: that policy was not the one evaluated.

## Source-level confirmation

- **Inspection:** `inspection_descent/baselines/always_recover.py:1–3` explicitly returns `approval_step < dispatch_step, approval_step`; `inspection_descent/simulation.py:392–393` uses that decision. Its original historical report also explicitly states the D=0 exception (`independent_evaluation_20_319/main_result/REPORT.md`, policy-semantics section).
- **Geofence:** `geofence_turn/baselines/always_recover.py:4–6` unconditionally rejects, but it is called only inside `if step == dispatch` in `geofence_turn/simulation.py:251–269`. Recovery is physically applied at that action step (`simulation.py:286–296`). `scenario.py:154–163` defines the braking/hover reference.
- **Underpass:** `underpass_inspection/baselines/policies.py:57–71` starts sideways recovery directly in `approve()`. `dispatch():92–100` ignores later dispatch when recovery has already started. `simulation.py:142–158` calls approval before same-step dispatch and applies the selected physical maneuver.

The Inspection evaluation directory is a schema conversion of its historical journal. The original, unconverted outcomes were checked as well: D=0 records have `recover=False`, while positive-delay records have `recover=True`. Its original report documents approval-time fallback. Some normalized fallback timestamps are reconstructed from that documented rule, not newly measured event timestamps. The journal contains 282 native records and 2,754 verified-reference-reuse records for this policy; these are recorded evaluation cases, not a claim of 3,036 independently rerun trajectories.

## Verification performed

- Read every Always Recover record across the three primary evaluation journals: **15,136 cases**.
- Verified exact scenario/delay coverage, no duplicate keys, recovery/application steps, safety and completion counts.
- Verified each complete journal's recorded byte count and SHA-256 checksum.
- Checked the original unconverted Inspection records and its documented historical rule.
- Ran five direct, non-physical semantic checks: Inspection's D=0/positive-delay cases; Geofence's unconditional rejection; Underpass's immediate cancellation at zero and positive dispatch delays.
- **No flights, prediction runs, algorithm changes, or historical result edits.**

## Consequence and recommended correction

Table II's single Always Recover row must not be interpreted as the same policy across missions. Its numerical cells reproduce the actual archived experiments, but the common label masks different intervention timing. The existing footnote disclosed the difference; it did not resolve experimental comparability. This should have been raised as a methodological issue before presenting the row as a common baseline.

The consistent definition recommended for a new version is: **at approval, cancel the approved command and immediately start the mission's predefined safety-only recovery, including at D=0, without future dispatch information**. Underpass already implements this. Inspection's D=0 exception and Geofence's dispatch-only initiation would need correction and revalidation in fresh versioned results. There is no need to rerun the other policies merely to correct this baseline. Alternatively, retain the archived results with three explicitly distinct policy labels rather than a unified baseline row.

The Underpass full-versus-command-only and full-versus-fixed comparisons do not use different Always Recover variants and are not invalidated by this finding. Historical Geofence dispatch-time recovery records remain valid as independent physical recovery-feasibility samples; they must retain their actual start times. This focused audit does not certify equivalence of every other cross-mission baseline.
