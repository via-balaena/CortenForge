# Parallel Tempering digest

This file is the anchor-3 commit in Chapter 53 §4's four-commit
reproducibility sequence. Its job is narrow: capture the
dual-metric summary block from the Ch 53 §2.3 Parallel
Tempering follow-up run, compare the per-replicate `best` and
`final` columns against Ch 52's basic-SA numbers on the same
replicate seed pool, and record a qualitative note about
chain dynamics and the per-chain compute split. The
interpretation of the joint `(C_rich, C_pt)` verdict under
Ch 53 §3.2's decision rule lives in Ch 53 §§5-6; this chapter
does not do that work.

## Run metadata

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit:** `002d056a` (`docs(ml-chassis-study): Ch 54
  richer-SA digest`) — the anchor-2 commit that landed
  immediately before this digest in Ch 53 §4's four-commit
  sequence.
- **Algorithm:** Parallel Tempering with `K = 4` chains on a
  geometric temperature ladder `T_1 = 0.5 → T_4 = 50.0`,
  Metropolis-criterion swap attempted every epoch between
  adjacent chains. The returned policy is chain 1 (the
  lowest-temperature chain). Implementation from `8f8e006f`.
- **Fixture:** `sim/L0/opt/tests/d2c_sr_rematch_pt.rs`
  (production `#[ignore]` test `d2c_sr_rematch_pt`).
- **Wall time:** 13,342.65 seconds (~3h42m) under
  `caffeinate -i` on a MacBook Pro.
- **Test result:** `ok. 1 passed; 0 failed; 0 ignored`.

## Dual-metric summary block (verbatim from the log)

```
--- Rematch summary: dual-metric (Ch 51 §2) ---
Per Ch 51 §2.3, final_reward is the primary operationalization of "resolves the peak";
best_reward is retained as a complementary signal about exploration reach.  A human
reader applies Ch 51 §2.3's nine-cell framework to the (best, final) pair printed below.

[best_reward — max-across-epochs per replicate]
rep  CEM best_reward  SA best_reward     SA-CEM diff
  0        210.35        125.77          -84.59
  1        212.64        237.67          +25.03
  2        292.98        203.41          -89.57
  3        206.16        226.29          +20.13
  4        202.60        247.57          +44.97
  5        222.49        221.30           -1.19
  6        164.56        198.67          +34.10
  7        220.76        161.87          -58.89
  8        220.89        242.18          +21.29
  9        181.08        236.85          +55.77

mean(CEM) = 213.4526
mean(SA)  = 210.1580
bootstrap CI on mean(SA) - mean(CEM): point=-3.2947, [-35.1893, +26.1591], B=10000
classification: Null

[final_reward — last-epoch mean_reward per replicate]
rep  CEM final_reward  SA final_reward     SA-CEM diff
  0         14.16        125.77         +111.60
  1         26.59        237.67         +211.08
  2         21.97        203.41         +181.44
  3        -31.22        226.29         +257.51
  4         86.13        247.57         +161.44
  5         45.40        221.30         +175.91
  6         16.58        198.67         +182.08
  7         95.34        160.37          +65.02
  8         39.33        242.18         +202.85
  9        -17.54        236.85         +254.39

mean(CEM) = 29.6742
mean(SA)  = 210.0072
bootstrap CI on mean(SA) - mean(CEM): point=+180.3329, [+146.1572, +212.3541], B=10000
classification: Positive

Joint (best, final) = (Null, Positive) — see Ch 51 §2.3 for the reading.

--- Ch 53 §2.3 PT outcome (dual metric, Ch 51 §2) ---
best_reward classification:  Null
final_reward classification: Positive
joint (best, final):         (Null, Positive)
Apply Ch 53 §3.2's three-case rule to PT's final_reward classification (joint with richer-SA's).
```

Initial batch size `N_INITIAL = 10`. Folded-pilot expansion to
`N = 20` did not fire (neither metric classified Ambiguous on
the initial bootstrap).

Note that the PT fixture uses `name() -> "SA"` on its algorithm
wrapper per Ch 53 §2.4's bit-for-bit replicate-extraction
binding, which is why the summary block above prints
`SA best_reward` / `SA final_reward` columns rather than
`PT best_reward` / `PT final_reward`. The values in the "SA"
columns are the PT chain-1 artifacts.

## Per-chain compute parity

Ch 53 §2.3's compute-parity commitment is *total-compute
parity*: all `K = 4` chains together consume the same
`16_000_000` env steps basic SA consumed on its single chain.
Each PT chain therefore receives `16_000_000 / 4 = 4_000_000`
env steps, and at `32 envs × 5_000 steps = 160_000` steps per
epoch, each chain executes **25 epochs** of proposals against
basic SA's 100 epochs. The per-chain proposal budget is
exactly one-quarter of basic SA's per-chain proposal budget.

This structural asymmetry is a load-bearing fact for reading
the PT results. Any per-replicate gap where PT's chain 1
reaches a lower `final_reward` than basic SA on the same seed
is at least partially explainable by PT chain 1 having had
one-quarter as many acceptance opportunities. A converse gap
(PT chain 1 reaching a higher `final_reward`) means PT's
swap-move mechanism was efficient enough to compensate for
the per-chain budget gap *and* add something basic SA did not
reach under its 4x-longer single-chain trajectory.

Wall-time sanity check: PT's 13,342.65s against basic-SA's
13,821.83s on the same machine is within 3.5%, confirming the
total-compute equality held in practice (the CEM branch, which
is identical across both runs, contributes roughly half the
wall time, and the remaining PT-side ~6,600s matches 25
epochs × ~26.5s × 10 replicates within measurement noise).

## Per-replicate comparison against Ch 52 basic-SA

As in Ch 54, the two runs share `REMATCH_MASTER_SEED =
20_260_412` and therefore the per-replicate seed pool. The
CEM columns are pointwise identical across runs; the SA
columns isolate the effect of running PT's chain 1 instead of
basic SA's single chain under the same physics noise and the
same bootstrap seed.

### `best_reward` per replicate

| rep | Ch 52 basic-SA `best` | Ch 55 PT chain-1 `best` | Δ (PT − basic) |
|----:|----------------------:|------------------------:|---------------:|
|   0 |                 55.77 |                  125.77 |         +70.00 |
|   1 |                212.32 |                  237.67 |         +25.35 |
|   2 |                292.12 |                  203.41 |         −88.71 |
|   3 |                198.09 |                  226.29 |         +28.20 |
|   4 |                246.66 |                  247.57 |          +0.91 |
|   5 |                 60.41 |                  221.30 |        +160.89 |
|   6 |                158.26 |                  198.67 |         +40.41 |
|   7 |                226.69 |                  161.87 |         −64.82 |
|   8 |                244.62 |                  242.18 |          −2.44 |
|   9 |                203.84 |                  236.85 |         +33.01 |

Ten-replicate means: basic-SA `best` = 189.878, PT `best`
= 210.158. PT's peak metric is *higher* by 20.28 units on
average — opposite sign from Ch 54's richer-SA delta. PT
improved on basic SA on seven of ten replicates, most
dramatically on Rep 5 where basic SA reached only 60.41
and PT's chain 1 reached 221.30 (a +160.89 per-replicate
delta). Rep 5 is the replicate basic SA found catastrophic in
Ch 50 and Ch 52; PT's tempering swaps evidently pulled
chain 1 off the same local plateau basic SA got stuck on.

The bootstrap CI on `best_reward` is nonetheless Null
(point = −3.29, CI = [−35.19, +26.16]) because the +160.89
Rep 5 improvement is partially cancelled by the −88.71 and
−64.82 regressions on Rep 2 and Rep 7. The per-replicate
variance is high enough that the mean delta is close to zero.

### `final_reward` per replicate

| rep | Ch 52 basic-SA `final` | Ch 55 PT chain-1 `final` | Δ (PT − basic) |
|----:|-----------------------:|-------------------------:|---------------:|
|   0 |                  55.77 |                   125.77 |         +70.00 |
|   1 |                 212.32 |                   237.67 |         +25.35 |
|   2 |                 292.12 |                   203.41 |         −88.71 |
|   3 |                 182.36 |                   226.29 |         +43.93 |
|   4 |                 246.66 |                   247.57 |          +0.91 |
|   5 |                  55.77 |                   221.30 |        +165.53 |
|   6 |                 158.26 |                   198.67 |         +40.41 |
|   7 |                 226.69 |                   160.37 |         −66.32 |
|   8 |                 237.83 |                   242.18 |          +4.35 |
|   9 |                 203.84 |                   236.85 |         +33.01 |

Ten-replicate means: basic-SA `final` = 187.163, PT `final`
= 210.007. PT is *higher* by 22.84 units on the primary
metric; eight of ten replicates are positive deltas, Rep 5's
+165.53 is the largest single-replicate improvement, and the
two regressions (Rep 2 at −88.71, Rep 7 at −66.32) are
partially offset by the seven improvements.

Both variants' `final_reward` classifications are `Positive`
against CEM, with:

- Ch 52 basic-SA: point = +157.4883, CI = [+104.5254, +207.7066]
- Ch 55 PT chain-1: point = +180.3329, CI = [+146.1572, +212.3541]

PT's point estimate is higher by +22.84 units and its CI
lower bound is higher by +41.63 units — the narrower CI
reflects the tighter PT replicate spread on the positive
side. PT's `final_reward` CI is strictly *above* basic SA's
CI lower bound, which is a stronger signal than Ch 54's
richer-SA result where the CIs overlapped closely.

## Qualitative note on chain dynamics

The epoch-level log shows PT chain 1's incumbent trajectories
are qualitatively different from basic SA's under the per-
chain 25-epoch budget. Reading the full 25 epochs of each
replicate:

- Rep 0: `43.47` → `118.76` at epoch 3 → held 19 epochs →
  `125.77` at epoch 23 (final). 3 accepted steps.
- Rep 1: `123.09` → `161.21` at epoch 8 → `237.67` at epoch
  16 (final, held 8 epochs). 3+ accepted steps, final escape
  mid-run.
- Rep 2: `159.27` → `168.61` → `191.54` at epoch 20 →
  `203.41` at epoch 23 (final). 4+ accepted steps, with
  late-run escapes.
- Rep 3: `128.47` → `155.46` → `162.42` → `226.29` at
  epoch 24 (final, last-epoch jump). 4 accepted steps.
- Rep 4: `214.51` held 18 epochs → `247.57` at epoch 23
  (final). 2 accepted steps total.
- Rep 5: `151.19` → `191.10` at epoch 12 → `221.30` at
  epoch 24 (final, last-epoch jump). 3 accepted steps, and
  this is the replicate that rescued basic SA's 55.77
  plateau — tempering swaps found a route around the local
  optimum that trapped basic SA.
- Rep 6: `125.24` → `144.96` → `198.67` at epoch 9 (final,
  held 16 epochs). 3 accepted steps, unusually early
  convergence.
- Rep 7: `120.21` → `146.24` → `161.87` at epoch 15 →
  `160.37` at epoch 16 (accepted downhill step, final).
  The downhill step is the signature of a swap move: a
  higher-T chain state was accepted into chain 1 at a worse
  value than the incumbent, because the Metropolis swap
  criterion ran, not the single-chain accept-only criterion.
  This is PT behaving as intended.
- Rep 8: `242.18` held 20 epochs → unchanged through
  epoch 24 (final). 0 accepted steps after epoch 5.
- Rep 9: shown in log tail — `121.07` → `128.19` →
  `138.26` → `236.85` at epoch 11 (final, held 14 epochs).
  4 accepted steps, large late escape.

Typical PT chain-1 behavior under this compute parity:
2-4 accepted incumbent updates in 25 epochs, with large
jumps at specific epochs (often epoch 23 or 24, coinciding
with the last swap attempt that pulls a hot-chain state
into chain 1). This is very different from basic SA's
~5-7 accepted steps in 100 epochs; PT's per-chain budget is
shorter but each accepted step tends to be a larger
incumbent improvement because the candidate came from a
higher-temperature chain that had been exploring more
freely. Rep 7's accepted *downhill* step at epoch 16
(161.87 → 160.37) is the clearest behavioral signature of
PT vs basic SA in the log — basic SA could not have
accepted that step under its Metropolis criterion because
basic SA is quasi-monotone on this fixture.

The methodological implication for Ch 51 §2.3's framework —
that PT's 25-epoch chain 1 reaches a *higher* mean
`final_reward` than basic SA's 100-epoch chain at the cost
of higher per-replicate variance — is an interpretation
question for Ch 53 §6 and not this digest. This digest only
records that the trajectories are visibly swap-driven, that
Rep 5's rescue of basic SA's local-optimum plateau is the
strongest single-replicate signal of PT's mechanism at
work, and that the two regressions (Rep 2, Rep 7) are not
obviously attributable to the 25-epoch budget shortfall
alone — Rep 2 in particular lost ~89 units against a basic
SA trajectory that had reached 292.12 on that seed, which
is above the empirical peak range Ch 30 expected.

## Applicability to Ch 53 §3.2 preconditions

Ch 53 §3.2's Corroborate preconditions on the PT variant
read:

- `C_pt == Positive` — yes (final_reward classification is
  Positive per the summary block).
- PT `final_reward` CI lower bound strictly positive —
  yes (+146.16).
- PT `final_reward` point estimate ≥ +50 — yes (+180.33).

All three preconditions on the PT side are satisfied. Ch 54
§"Applicability to Ch 53 §3.2 preconditions" established
that the richer-SA side also satisfies all three. The joint
`(C_rich, C_pt)` pair is therefore `(Positive, Positive)`
with both CI lower bounds strictly positive and both point
estimates well above the +50 floor. Under §3.2, this pair
lands in **Case Corroborate**. The case resolution and §3.3
response live in Ch 53 §§5-6.

## Cross-machine note

As noted in Ch 54, both the richer-SA and PT runs fired on
the same MacBook Pro as Ch 52's basic-SA run, so Ch 53
§4.2's x86_64/aarch64 `f64` identity claim is not exercised
by this chapter. Reproducibility under cross-machine
execution remains an untested inheritance from Ch 23 §3's
single-machine discipline, and is the responsibility of a
future chapter that re-runs either fixture on a non-aarch64
host.

## Reproducibility anchors

- **Branch:** `feature/ml-chassis-post-impl`
- **Parent commit for this digest:** `002d056a`
- **Fixture:** `sim/L0/opt/tests/d2c_sr_rematch_pt.rs`
- **Command:**
  `caffeinate -i cargo test -p sim-opt --release --ignored d2c_sr_rematch_pt -- --nocapture 2>&1 | tee /tmp/ch53_pt.log`
- **`REMATCH_MASTER_SEED`:** `20_260_412`
- **`BOOTSTRAP_RNG_SEED`:** `0xB007_0057_00AA_0055`
- **Bootstrap resamples `B`:** `10_000`
- **Initial batch size `N_INITIAL`:** `10` (no expansion fired
  — `either_ambiguous` was false on the initial classify)
- **Total budget:** `TrainingBudget::Steps(16_000_000)` across
  all `K = 4` PT chains (per-chain: `4_000_000` env steps =
  25 epochs × 160K steps/epoch)
- **PT ladder:** `K = 4`, geometric from `T_1 = 0.5` to
  `T_4 = 50.0`, swap attempts every epoch between adjacent
  chains
- **Matched-complexity anchor:** `LinearPolicy(2, 1)`,
  `n_params = 3`, init `[0.0, 0.0, 2.0]`
- **Captured run log:** `/tmp/ch53_pt.log` (ephemeral —
  regenerate with the command above)
- **Wall time:** 13,342.65 seconds (~3h42m) under
  `caffeinate -i`

## Cross-references

- Ch 52 (`52-amended-rerun-digest.md`) — basic-SA baseline
  whose per-replicate rows the comparison tables above
  reference
- Ch 53 §2.3
  (`53-robustness-check-prereg.md`) — the PT class
  commitment, total-compute parity constraint, and five
  shared bindings this run inherits
- Ch 53 §2.4 — the `name() -> "SA"` binding that explains
  why the summary block's columns are labelled `SA` rather
  than `PT`
- Ch 53 §3.2 — the three-case rule whose Corroborate
  preconditions are checked against this run's `final_reward`
  row above
- Ch 54 (`54-richer-sa-digest.md`) — the richer-SA
  anchor-2 digest whose conclusions chain into this file's
  §3.2 applicability paragraph
- `sim/L0/opt/src/algorithm.rs` — where the PT implementation
  lives (`8f8e006f`)
- `sim/L0/opt/tests/d2c_sr_rematch_pt.rs` — the fixture
  whose production `#[ignore]` test produced the log above
