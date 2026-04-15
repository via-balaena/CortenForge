# Amended re-run digest

This file is the anchor-3 commit in Chapter 51 §2.5's
four-commit reproducibility sequence. Its job is narrow:
capture the dual-metric summary block from the amended re-run
of `sim/L0/opt/tests/d2c_sr_rematch.rs` under the
`TwoMetricOutcome` pipeline, and record the bit-for-bit
determinism check against Chapter 50's per-replicate numbers.
The interpretation of the joint `(best, final)` verdict lives
in Chapter 51 §§3-4; this chapter does not re-do that work.

The full run log at `/tmp/d2c_sr_rematch_dual.log` is
ephemeral — regenerable via the command under "Reproducibility
anchors" below. This digest exists so that `git log --reverse`
over `51-metric-sensitivity-amendment.md` and this file makes
the pre-registration sequence visible without needing to
trust either chapter's own claim about when things were
written.

## Run metadata

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit:** `086c04c8` (`feat(sim-opt): dual-metric
  rematch TwoMetricOutcome amendment`)
- **Wall time:** 13,821.83 seconds (~3h50m) on a MacBook Pro
  under `caffeinate -i`
- **Test result:** `ok. 1 passed; 0 failed; 0 ignored`

Ch 50's original single-metric run took 13,760.92s (~3h49m)
on the same machine; the ~61s delta is normal run-to-run
variation and is not a correctness signal.

## Dual-metric summary block (verbatim from the log)

```
--- Rematch summary: dual-metric (Ch 51 §2) ---
Per Ch 51 §2.3, final_reward is the primary operationalization of "resolves the peak";
best_reward is retained as a complementary signal about exploration reach.  A human
reader applies Ch 51 §2.3's nine-cell framework to the (best, final) pair printed below.

[best_reward — max-across-epochs per replicate]
rep  CEM best_reward  SA best_reward     SA-CEM diff
  0        210.35         55.77         -154.58
  1        212.64        212.32           -0.32
  2        292.98        292.12           -0.86
  3        206.16        198.09           -8.06
  4        202.60        246.66          +44.06
  5        222.49         60.41         -162.08
  6        164.56        158.26           -6.30
  7        220.76        226.69           +5.92
  8        220.89        244.62          +23.72
  9        181.08        203.84          +22.76

mean(CEM) = 213.4526
mean(SA)  = 189.8784
bootstrap CI on mean(SA) - mean(CEM): point=-23.5743, [-76.4708, +24.1681], B=10000
classification: Null

[final_reward — last-epoch mean_reward per replicate]
rep  CEM final_reward  SA final_reward     SA-CEM diff
  0         14.16         55.77          +41.61
  1         26.59        212.32         +185.73
  2         21.97        292.12         +270.15
  3        -31.22        182.36         +213.58
  4         86.13        246.66         +160.53
  5         45.40         55.77          +10.38
  6         16.58        158.26         +141.68
  7         95.34        226.69         +131.35
  8         39.33        237.83         +198.50
  9        -17.54        203.84         +221.38

mean(CEM) = 29.6742
mean(SA)  = 187.1626
bootstrap CI on mean(SA) - mean(CEM): point=+157.4883, [+104.5254, +207.7066], B=10000
classification: Positive

Joint (best, final) = (Null, Positive) — see Ch 51 §2.3 for the reading.
```

## Determinism verification against Ch 50

Chapter 51 §2.5 states that the amended re-run is expected to
reproduce Ch 50's per-replicate peak and final numbers
bit-for-bit, because the `run_rematch_with_runner` driver at
`sim/L0/opt/src/analysis.rs:492-557` consumes the bootstrap
RNG best-first, matching the byte order the pre-amendment
single-metric driver used. That expectation is what this
section verifies.

### CEM per-replicate: log vs Ch 50

Ch 50's per-replicate table lives at
`50-d2c-sr-rematch-writeup.md:193-208`. The amended re-run
reproduces every CEM row exactly:

| rep | log `best_reward` | Ch 50 `CEM peak` | log `final_reward` | Ch 50 `CEM final` |
|----:|------------------:|-----------------:|-------------------:|------------------:|
|   0 |            210.35 |           210.35 |              14.16 |             14.16 |
|   1 |            212.64 |           212.64 |              26.59 |             26.59 |
|   2 |            292.98 |           292.98 |              21.97 |             21.97 |
|   3 |            206.16 |           206.16 |             −31.22 |            −31.22 |
|   4 |            202.60 |           202.60 |              86.13 |             86.13 |
|   5 |            222.49 |           222.49 |              45.40 |             45.40 |
|   6 |            164.56 |           164.56 |              16.58 |             16.58 |
|   7 |            220.76 |           220.76 |              95.34 |             95.34 |
|   8 |            220.89 |           220.89 |              39.33 |             39.33 |
|   9 |            181.08 |           181.08 |             −17.54 |            −17.54 |

All 20 CEM values match bit-for-bit.

### SA per-replicate: peak ≈ final pattern reproduced

Ch 50's `"What the metric measured"` section at
`50-d2c-sr-rematch-writeup.md:233-242` claims that SA's
`peak - final` is near zero across the board, with seven of
ten replicates showing exactly zero and the largest gap
topping out at 15.73. The amended re-run's dual-metric
columns reproduce that pattern exactly:

| rep | SA `best` | SA `final` | gap  |
|----:|----------:|-----------:|-----:|
|   0 |     55.77 |      55.77 | 0.00 |
|   1 |    212.32 |     212.32 | 0.00 |
|   2 |    292.12 |     292.12 | 0.00 |
|   3 |    198.09 |     182.36 | 15.73|
|   4 |    246.66 |     246.66 | 0.00 |
|   5 |     60.41 |      55.77 |  4.64|
|   6 |    158.26 |     158.26 | 0.00 |
|   7 |    226.69 |     226.69 | 0.00 |
|   8 |    244.62 |     237.83 |  6.79|
|   9 |    203.84 |     203.84 | 0.00 |

Seven replicates (`{0, 1, 2, 4, 6, 7, 9}`) show exactly zero,
and the maximum gap is 15.73 at rep 3. Both claims match
Ch 50's prose.

### Bootstrap CI on `best_reward`: reproduces Ch 50 to 2 decimals

Ch 50 reports at `50-d2c-sr-rematch-writeup.md:149-155` that
"the upper bound of the bootstrap CI [is] at +24.17 reward
units." The amended re-run reports the upper bound at
+24.1681, which rounds to +24.17. Mean(CEM) - mean(SA) is
+23.5743, which rounds to +23.57 — matching the +23.57 figure
Ch 50's methodological-finding discussion quotes at
`50-d2c-sr-rematch-writeup.md:179-187` and
`memory/project_metric_vs_menu_plan.md:69-74`.

The bootstrap RNG byte-consumption order held. The best-first
`test_and_classify` call ordering at
`sim/L0/opt/src/analysis.rs:514-517` preserves the exact
`BOOTSTRAP_RNG_SEED` state sequence the pre-amendment
single-metric driver used, which is why the CI bounds
reproduce the pre-amendment numbers to the decimals Ch 50
published.

## What the dual-metric run adds beyond Ch 50

Ch 50 tabled per-replicate CEM peak and CEM final but only
named SA's peak-vs-final pattern in prose, not in a table,
and did not report the `final_reward` bootstrap numerically
because the Ch 50 protocol's bootstrap was single-metric.
The amended re-run adds three pieces of data the original
run did not produce:

1. **Per-replicate SA `final_reward` values** — columns above.
2. **`final_reward` bootstrap CI on `mean(SA) - mean(CEM)`** —
   point = +157.4883, CI = [+104.5254, +207.7066],
   `B = 10_000`, classification `Positive`.
3. **Joint `(best, final)` classification pair** — `(Null,
   Positive)` as it arrives from the `TwoMetricOutcome`
   pipeline, to be read through Ch 51 §2.3's nine-cell
   framework in Ch 51 §§3-4.

## Reproducibility anchors

Re-running this exact experiment under the same branch HEAD
should reproduce every number above to the final decimal.

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit for this digest:** `086c04c8`
- **Fixture:** `sim/L0/opt/tests/d2c_sr_rematch.rs`
- **Command:**
  `caffeinate -i cargo test -p sim-opt --release d2c_sr_rematch -- --ignored --nocapture 2>&1 | tee /tmp/d2c_sr_rematch_dual.log`
- **`REMATCH_MASTER_SEED`:** `20_260_412`
- **`BOOTSTRAP_RNG_SEED`:** `0xB007_0057_00AA_0055`
- **Bootstrap resamples `B`:** `10_000`
- **Initial batch size `N_INITIAL`:** `10` (no expansion fired
  — `either_ambiguous` was false on the initial classify)
- **Budget:** `TrainingBudget::Steps(16_000_000)` =
  `32 * 5_000 * 100`
- **Matched-complexity anchor:** `LinearPolicy(2, 1)`,
  `n_params = 3`, init `[0.0, 0.0, 2.0]`
- **Captured run log:** `/tmp/d2c_sr_rematch_dual.log`
  (ephemeral — regenerate with the command above)
- **Wall time:** 13,821.83 seconds (~3h50m) under
  `caffeinate -i`

## Cross-references

- Ch 50 per-replicate table
  (`50-d2c-sr-rematch-writeup.md:193-208`) — the single-metric
  source this digest reproduces bit-for-bit
- Ch 50 §"What the metric measured"
  (`50-d2c-sr-rematch-writeup.md:167-298`) — the methodological
  finding that motivated Ch 51
- Ch 51 §2.3 — the nine-cell interpretation framework that
  Ch 51 §§3-4 will apply to the joint `(Null, Positive)` pair
  above
- Ch 51 §2.5 — the four-commit reproducibility anchor whose
  third slot is this file's commit
- `sim/L0/opt/src/analysis.rs:492-557` —
  `run_rematch_with_runner`, the dual-metric driver
- `sim/L0/opt/src/analysis.rs:577-606` —
  `extract_replicate_vectors`, the pool-matched helper
- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the fixture
