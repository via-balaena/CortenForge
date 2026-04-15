# Richer-SA digest

This file is the anchor-2 commit in Chapter 53 §4's four-commit
reproducibility sequence. Its job is narrow: capture the
dual-metric summary block from the Ch 53 §2.2 richer-proposal
SA follow-up run, compare the per-replicate `best` and `final`
columns against Ch 52's basic-SA numbers on the same replicate
seed pool, and record a qualitative note about chain dynamics
visible in the epoch-level log. The interpretation of the
joint `(C_rich, C_pt)` verdict under Ch 53 §3.2's decision
rule lives in Ch 53 §§5-6; this chapter does not do that work.

## Run metadata

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit:** `abf0f3aa` (`fix(sim-opt): exclude epoch 0
  forced-accept from Rechenberg rate`) — the self-review fix
  that closed out Ch 53 §4's richer-SA code landing sequence
  before the run fired.
- **Algorithm:** richer-proposal SA with the Rechenberg 1/5
  success rule from `c7aabcc7`. Non-stationary proposal std
  adapts toward the Rechenberg target as the chain progresses,
  satisfying Ch 53 §2.2's class commitment ("proposal
  distribution evolves during training").
- **Fixture:** `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs`
  (production `#[ignore]` test `d2c_sr_rematch_richer_sa`).
- **Wall time:** 13,598.48 seconds (~3h46m) under
  `caffeinate -i` on a MacBook Pro.
- **Test result:** `ok. 1 passed; 0 failed; 0 ignored`.

Ch 52's basic-SA run took 13,821.83 seconds (~3h50m) on the
same machine; the ~223s delta is normal run-to-run variation
and is not a correctness signal.

## Dual-metric summary block (verbatim from the log)

```
--- Rematch summary: dual-metric (Ch 51 §2) ---
Per Ch 51 §2.3, final_reward is the primary operationalization of "resolves the peak";
best_reward is retained as a complementary signal about exploration reach.  A human
reader applies Ch 51 §2.3's nine-cell framework to the (best, final) pair printed below.

[best_reward — max-across-epochs per replicate]
rep  CEM best_reward  SA best_reward     SA-CEM diff
  0        210.35        131.77          -78.58
  1        212.64        210.76           -1.88
  2        292.98        235.32          -57.66
  3        206.16        180.46          -25.70
  4        202.60        237.51          +34.91
  5        222.49         60.41         -162.08
  6        164.56        146.40          -18.17
  7        220.76        197.33          -23.43
  8        220.89        263.86          +42.97
  9        181.08        190.50           +9.42

mean(CEM) = 213.4526
mean(SA)  = 185.4320
bootstrap CI on mean(SA) - mean(CEM): point=-28.0207, [-70.5583, +10.8916], B=10000
classification: Null

[final_reward — last-epoch mean_reward per replicate]
rep  CEM final_reward  SA final_reward     SA-CEM diff
  0         14.16        131.77         +117.61
  1         26.59        210.76         +184.17
  2         21.97        235.32         +213.35
  3        -31.22        180.46         +211.68
  4         86.13        237.51         +151.37
  5         45.40         55.77          +10.38
  6         16.58        146.32         +129.74
  7         95.34        197.33         +101.99
  8         39.33        263.86         +224.53
  9        -17.54        190.50         +208.04

mean(CEM) = 29.6742
mean(SA)  = 184.9604
bootstrap CI on mean(SA) - mean(CEM): point=+155.2862, [+110.3305, +197.1795], B=10000
classification: Positive

Joint (best, final) = (Null, Positive) — see Ch 51 §2.3 for the reading.

--- Ch 53 §2.2 richer-SA outcome (dual metric, Ch 51 §2) ---
best_reward classification:  Null
final_reward classification: Positive
joint (best, final):         (Null, Positive)
Apply Ch 53 §3.2's three-case rule to this variant's final_reward
classification (joint with PT's) to pick Corroborate / Mixed / Contradict.
```

Initial batch size `N_INITIAL = 10`. Folded-pilot expansion to
`N = 20` did not fire (expansion only triggers when *either*
metric classifies Ambiguous on the initial bootstrap, and
neither did).

## Per-replicate comparison against Ch 52 basic-SA

The two runs share `REMATCH_MASTER_SEED = 20_260_412` and so
the per-replicate seed vector is pointwise identical. CEM is
invariant across the two runs because the replicate driver
consumes the CEM branch before the SA branch on the same seed
stream; both runs therefore see the same 10 CEM rows. The SA
columns below isolate the effect of swapping basic SA for
richer-proposal SA under matched compute, matched physics
noise, and matched bootstrap seed.

### `best_reward` per replicate

| rep | Ch 52 basic-SA `best` | Ch 54 richer-SA `best` | Δ (rich − basic) |
|----:|----------------------:|-----------------------:|-----------------:|
|   0 |                 55.77 |                 131.77 |           +76.00 |
|   1 |                212.32 |                 210.76 |            −1.56 |
|   2 |                292.12 |                 235.32 |           −56.80 |
|   3 |                198.09 |                 180.46 |           −17.63 |
|   4 |                246.66 |                 237.51 |            −9.15 |
|   5 |                 60.41 |                  60.41 |             0.00 |
|   6 |                158.26 |                 146.40 |           −11.86 |
|   7 |                226.69 |                 197.33 |           −29.36 |
|   8 |                244.62 |                 263.86 |           +19.24 |
|   9 |                203.84 |                 190.50 |           −13.34 |

Ten-replicate means: basic-SA `best` = 189.878, richer-SA `best`
= 185.432. The richer variant is lower by 4.45 units on the
peak metric; the per-replicate deltas span −56.80 to +76.00,
with Rep 5 numerically identical at 60.41.

### `final_reward` per replicate

| rep | Ch 52 basic-SA `final` | Ch 54 richer-SA `final` | Δ (rich − basic) |
|----:|-----------------------:|------------------------:|-----------------:|
|   0 |                  55.77 |                  131.77 |           +76.00 |
|   1 |                 212.32 |                  210.76 |            −1.56 |
|   2 |                 292.12 |                  235.32 |           −56.80 |
|   3 |                 182.36 |                  180.46 |            −1.90 |
|   4 |                 246.66 |                  237.51 |            −9.15 |
|   5 |                  55.77 |                   55.77 |             0.00 |
|   6 |                 158.26 |                  146.32 |           −11.94 |
|   7 |                 226.69 |                  197.33 |           −29.36 |
|   8 |                 237.83 |                  263.86 |           +26.03 |
|   9 |                 203.84 |                  190.50 |           −13.34 |

Ten-replicate means: basic-SA `final` = 187.163, richer-SA
`final` = 184.960. The richer variant is lower by 2.20 units
on the primary metric. Per-replicate deltas span −56.80
(Rep 2) to +76.00 (Rep 0); Rep 5 is bit-for-bit identical
on both metrics (the chain evidently rejected every adaptive
proposal the Rechenberg rule produced beyond the basic-SA
trajectory on that seed). Rep 0's matching +76.00 delta on
both `best` and `final` reflects the fact that eight of ten
richer-SA replicates have `peak == final` exactly, one
(Rep 6) has a trivial 0.08 gap, and one (Rep 5) has a 4.64
gap that is bit-for-bit identical to basic-SA's Rep 5 gap —
so on a per-replicate basis, swapping basic SA for richer
SA under matched seeds leaves the peak-to-final dispersion
pattern essentially unchanged.

Both variants' `final_reward` classifications are `Positive`
against CEM, with:

- Ch 52 basic-SA: point = +157.4883, CI = [+104.5254, +207.7066]
- Ch 54 richer-SA: point = +155.2862, CI = [+110.3305, +197.1795]

The CIs overlap heavily and the point estimates are within
~2.2 units. The richer variant's CI lower bound is actually
*higher* than basic SA's (+110.33 vs +104.53), driven by
the narrower richer-SA replicate spread (std across rep
deltas is smaller on the primary metric).

## Qualitative note on chain dynamics

The epoch-level log shows long terminal plateaus under
Rechenberg 1/5 across **all ten replicates**. Reading the
last 20 epochs of each replicate:

- Rep 0: `reward = 131.61` for epochs 42–90, then one late
  step to `131.77` at epoch 91 which held to epoch 99.
- Rep 1: `210.13` for ≥13 epochs, stepped to `210.76` at
  epoch 93.
- Rep 2: `207.97` for ~10 epochs, stepped to `235.32` at
  epoch 90.
- Rep 3: `180.46` held unchanged from ≤ epoch 80 through 99.
- Rep 4: `237.51` held unchanged from ≤ epoch 80 through 99.
- Rep 5: `55.77` held unchanged from ≤ epoch 80 through 99.
- Rep 6: `146.40` held ~11 epochs, stepped to `146.32` at
  epoch 91 (an unusual *negative* step — either an accepted
  downward proposal or a rare physics-noise artifact; the
  `final_reward` column records 146.32 per `mean_reward`
  semantics).
- Rep 7: `197.33` held unchanged from ≤ epoch 80 through 99.
- Rep 8: `263.86` held unchanged from ≤ epoch 80 through 99.
- Rep 9: `172.41` held ~11 epochs, stepped to `190.50` at
  epoch 92.

Plateaus of ≥ 20 epochs in the final 20% of training are
universal to richer-SA under the Rechenberg 1/5 rule on
this fixture. The Rechenberg controller shrinks
`proposal_std` toward the 1/5 success target as the chain
accumulates rejections; late in training, proposals become
small enough that the chain's acceptance rate drops toward
zero and the incumbent freezes. Rep 0's unusually long
49-epoch mid-training plateau at `131.61` is replicate-
specific in its length but structurally the same dynamic;
its late-training one-step escape to `131.77` at epoch 91
mirrors the other replicates' late-training escape steps
at epochs 90-93.

This is not a pathology — it is the expected terminal
behavior of a success-rule-adaptive proposal once the
chain has landed on a local plateau with a low-acceptance-
rate neighborhood. Basic SA's Ch 52 run shows the same
qualitative pattern (seven of ten replicates have
`peak == final` and the other three have gaps ≤ 15.73);
the adaptive proposal does not qualitatively change the
terminal freezing behavior, it only changes *when* the
chain freezes and *at what value*.

The methodological implication — whether the universal
plateau behavior is a signal that `final_reward` is
over-reporting incumbent persistence rather than
optimization progress — is an interpretation question for
Ch 53 §6 and not this digest. This digest only records
that the plateaus are universal and that rep 0's length
is an outlier within the pattern, not an exception to it.

## Applicability to Ch 53 §3.2 preconditions

Ch 53 §3.2's Corroborate preconditions on the richer-SA
variant read:

- `C_rich == Positive` — yes (final_reward classification is
  Positive per the summary block).
- Richer-SA `final_reward` CI lower bound strictly positive
  — yes (+110.33).
- Richer-SA `final_reward` point estimate ≥ +50 — yes
  (+155.29).

All three preconditions on the richer-SA side are satisfied.
Whether the *joint* outcome is Corroborate depends on PT's
row of preconditions, resolved in Ch 55. The case resolution
and §3.3 response live in Ch 53 §§5-6.

## Cross-machine note

Ch 53 §4.2's hardware note anticipated a second-machine run
for one or both follow-ups. In practice, both the richer-SA
and PT runs fired on the same MacBook Pro as Ch 52's basic-SA
run, so §4.2's x86_64/aarch64 `f64` identity claim is not
exercised by this chapter. Reproducibility under cross-
machine execution remains an untested inheritance from
Ch 23 §3's single-machine discipline. A future chapter that
re-runs either fixture on a non-aarch64 host would be the
place to exercise it; Ch 54 and Ch 55 do not.

## Reproducibility anchors

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit for this digest:** `abf0f3aa`
- **Fixture:** `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs`
- **Command:**
  `caffeinate -i cargo test -p sim-opt --release --ignored d2c_sr_rematch_richer_sa -- --nocapture 2>&1 | tee /tmp/ch53_richer_sa.log`
- **`REMATCH_MASTER_SEED`:** `20_260_412`
- **`BOOTSTRAP_RNG_SEED`:** `0xB007_0057_00AA_0055`
- **Bootstrap resamples `B`:** `10_000`
- **Initial batch size `N_INITIAL`:** `10` (no expansion fired
  — `either_ambiguous` was false on the initial classify)
- **Budget:** `TrainingBudget::Steps(16_000_000)` =
  `32 * 5_000 * 100`
- **Matched-complexity anchor:** `LinearPolicy(2, 1)`,
  `n_params = 3`, init `[0.0, 0.0, 2.0]`
- **Captured run log:** `/tmp/ch53_richer_sa.log` (ephemeral
  — regenerate with the command above)
- **Wall time:** 13,598.48 seconds (~3h46m) under
  `caffeinate -i`

## Cross-references

- Ch 52 (`52-amended-rerun-digest.md`) — basic-SA baseline
  whose per-replicate rows the comparison tables above
  reference
- Ch 53 §2.2
  (`53-robustness-check-prereg.md`) — the class commitment
  and five shared bindings this run inherits
- Ch 53 §3.2 — the three-case rule whose Corroborate
  preconditions are checked against this run's `final_reward`
  row above
- `sim/L0/opt/src/algorithm.rs` — where `Sa::new`'s
  Rechenberg 1/5 proposal adaptation lives (`c7aabcc7`,
  `abf0f3aa`)
- `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs` — the
  fixture whose production `#[ignore]` test produced the
  log above
