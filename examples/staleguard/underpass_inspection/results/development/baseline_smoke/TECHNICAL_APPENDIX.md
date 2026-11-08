# Underpass seven-policy physical smoke validation

This actual-PyBullet smoke benchmark executes 189 flights: three frozen scenarios,
nine shared dispatch delays and all seven policies. Seed 1 has a negative measured
sideways gap, seed 6 a longer positive gap and seed 14 a short positive gap. They
were selected to exercise the requested physical cases, not from policy outcomes.

| Policy | Unsafe / 27 | Safe inspection / 27 | Recoveries / 27 | Early recoveries / 27 |
|---|---:|---:|---:|---:|
| No Recovery | 16 | 11 | 0 | 0 |
| Always Recover | 0 | 0 | 27 | 24 |
| FBS | 16 | 11 | 0 | 0 |
| Use-Time Recheck | 11 | 10 | 17 | 0 |
| StaleGuard | 12 | 11 | 12 | 0 |
| StaleGuard EarlyRecovery | 7 | 11 | 12 | 10 |
| Fixed EarlyRecovery | 0 | 3 | 24 | 21 |

These are smoke-validation counts, not the full development comparison. Both
StaleGuard variants use the same forecast and measured readiness duration. The
primary recovery is always the independent Eastward sideways maneuver. No policy
parameters or geometry were adjusted after seeing these outcomes.

The [validation audit](validation.json) checks complete unique coverage, actual
approval states and controller memory, full observation horizons at 1,000 Hz,
continuous native motion, source provenance, cancellation and realized event
ordering. All 27 No Recovery and all 27 Always Recover trajectories exactly match
the archived physical hashes and outcomes. In total, 186 flights have an
applicable archived maneuver/start reference and match it; three recheck starts
fall outside the archived 0–1.00 s grid. Seven representative actual arrays are
independently checked for clearance, first violation, inspection dwell and
relative reference shape. All 110 pre-existing result artifacts remain unchanged.

The [resumption test](resume_test.json) stops after seven committed flights,
appends an unfinished JSON tail to this new journal, resumes, repairs the tail and
finishes all 189 unique records. The original committed prefix is preserved
byte-for-byte. No archived calibration/deadline files were altered for the test.

The 27 focused code/physical tests additionally include two actual stress flights
at D=2.00 s, outside this benchmark's grid. No Recovery retains its waiting-period
violation; Use-Time Recheck rejects after the observed violation and its subsequent
sideways recovery cannot erase the unsafe outcome. Those two tests are not scores
in this table. Policy rules, reused implementations, runtime replay, source
equivalence and reproduction are documented in the
[full technical appendix](../baseline_benchmark/TECHNICAL_APPENDIX.md).
