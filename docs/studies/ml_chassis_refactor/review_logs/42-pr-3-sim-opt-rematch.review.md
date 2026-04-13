# Review log — Chapter 42: PR 3 sim-opt split and rematch

Chapter 42 is the third and final PR-plan chapter in Part 4
of the study. It plans PR 3 (sim-opt split + rematch) as a
2-PR split into PR 3a (additive new `sim/L0/opt/` crate —
`Cargo.toml` + `lib.rs` + `algorithm.rs` shipping the `Sa`
/ `SaHyperparams` / `Algorithm` trait impl + `analysis.rs`
shipping `bootstrap_diff_means` / `bootstrap_diff_medians` /
`bimodality_coefficient` / `classify_outcome` / `run_rematch`
+ unit tests for each) and PR 3b (the rematch test fixture
at `sim-opt/tests/d2c_sr_rematch.rs` running Ch 32's
folded-pilot protocol end-to-end).

The chapter is gated per the Ch 01 protocol and pauses for
user review after the factual + thinking passes and before
commit. This log documents the pre-commit state.

Ch 42 also bundles a narrow amendment to **Ch 41** extending
`TaskConfig::build_fn` from `Fn(usize) -> Result<VecEnv,
EnvError>` to `Fn(usize, u64) -> Result<VecEnv, EnvError>`
so `Competition::run_replicates` can thread a per-replicate
seed through to `task.build_vec_env(n_envs, seed)`. The
bundling follows the `843dc21c` precedent from session 9.
The amendment is argued in Ch 42 §2 as sub-decision (a) and
its scope is named below in "Ch 41 bundled amendment."

Ch 42 is the largest Part 4 chapter by design surface (ten
in-chapter sub-decisions vs Ch 40's four and Ch 41's six),
reflecting the broader scope PR 3 renders: a new crate, a
new algorithm from scratch, a new analysis module, and a
new integration test fixture. The rendering density matches
the scope — §4 alone (SA implementation) is ~650 lines,
comparable to Ch 40's §3 (BatchSim rewire) but with more
new-code-from-scratch content and fewer existing-code-ripple
citations.

The genre is still rendering-over-arguing in the Part 4
mold. The ten sub-decisions each get explicit counterfactual
walks in their owning sections (§2 for sub-decision (a),
§3 for (b), §4 for (c)–(e), §5 for (h), §6 for (f)–(g),
§8 for (i), §9 for (j)), and §7 tables them for reader
reference. Most of the chapter describes what will land in
PR 3a and PR 3b at the file:line level.

## Recon

Ch 42's recon had five classes of source-tree verification
and five upstream-chapter re-reads.

- **Ch 30 (Scientific question) — full re-read.** Absorbed
  the three-outcome framing, the operational phrasing of
  "SA with Gaussian proposal and temperature schedule," and
  the two pre-committed null follow-ups (richer SA proposal
  and Parallel Tempering, both on the same SR task). Ch 42
  inherits the outcome framing for §5.4's `classify_outcome`
  implementation, the Gaussian proposal family as a locked
  constraint on §4.1's `Sa` struct, and the null follow-ups
  as the future-reuse argument for §5.1's library module
  placement.
- **Ch 31 §3.1–§4.4 — targeted re-read.** Absorbed the six
  inherited failure-mode guards. §3.1's warmup gate is moot
  for the current pool; §3.2's seed-variance envelope is the
  shape Ch 32's bootstrap CI physically implements; §4.1 and
  §4.2 are the Ch 40 PR 1b and Ch 41 PR 2b merge prereqs;
  §4.3 is the matched-complexity assertion Ch 42 renders at
  the fixture level; §4.4 is the folded-pilot protocol Ch 32
  owns and Ch 42 implements. Ch 42 §1.4 enumerates each and
  points at the section that renders the protective mechanism.
- **Ch 32 (Hyperparameter sensitivity) — full re-read.** The
  chapter is the bible for the rematch protocol. Five
  decisions plus one contingency, all of which Ch 42
  implements verbatim in `sim-opt/src/analysis.rs`. Ch 32
  §4.8's skeleton is the literal template for §5.5's
  `run_rematch` driver. Ch 32 §7's list of deferred items
  maps onto Ch 42 §9's list of "what Ch 42 does not decide"
  — five Ch 32-deferred items flow through to Ch 42's
  deferred list (bootstrap RNG, writeup format, re-runs,
  retirement of legacy files, BCa substitution).
- **Ch 40 (PR 1 chassis reproducibility) — genre skim.** Ch
  40's §1.4 genre note and §5 closing section were the
  structural template for Ch 42's corresponding sections.
  Ch 40 §3.3's `PerEnvStack` trait and `EnvBatch<S>` generic
  rendering was re-read in full to confirm that Ch 42's
  `make_training_vecenv` helper can render the post-PR-1b
  per-env construction pattern correctly. The 14 D1–D15
  decision table from Ch 40 §1.2 is the inheritance anchor
  for §1.6's "what Ch 42 inherits from Ch 40" section.
- **Ch 41 (PR 2 Competition replicates + algorithm surface)
  — full re-read with §2.5 / §3.2 deferral section re-read.**
  Ch 41 §2.5 (the `d2c_cem_training.rs` doc-only update) and
  §3.2 (the "narrow scope" defense) both defer "the rematch's
  new test fixture" to Ch 42 and explicitly name
  `sim-opt/tests/` as the placement. Ch 42 §6 renders exactly
  that placement. Ch 41 §4.4 (merge-order discussion) is the
  precedent for Ch 42 §8's merge-order partial-order diagram.
  Critically, **Ch 41 §2.1 is where the Ch 42 drafting
  recon surfaced the `TaskConfig::build_fn` seed-threading
  gap** — Ch 41 §2.1 renders the `run_replicates` body with
  seeds flowing to `algorithm.train` but not to
  `task.build_vec_env`, and Ch 42 §2 is the resulting
  bundled amendment argument.
- **Source tree — full reads of `sim/L0/ml-bridge/src/
  algorithm.rs`, `cem.rs`, `linear.rs`, `competition.rs`,
  `task.rs`, `vec_env.rs`, `rollout.rs`, `lib.rs`;
  `sim/L0/thermostat/tests/d2c_cem_training.rs`; the
  workspace `Cargo.toml`; and the `sim/L0/ml-bridge/Cargo.toml`.**
  Spot reads of `sim/L0/ml-bridge/src/policy.rs`,
  `artifact.rs`, and `best_tracker.rs` for the trait
  signatures Ch 42's SA implementation references. Every
  file:line citation in Ch 42 is derived from these reads.
- **Existence checks.** Grep for `sim-opt` and
  `SimulatedAnnealing` across the workspace confirmed that
  neither the crate nor the algorithm exists in the source
  tree — Ch 42's scope is genuinely greenfield. The only
  matches were in the study book's own chapters (Ch 23, 24,
  30, 31, 32, 40, 41) and in the thermo-computing vision
  docs (`docs/thermo_computing/01_vision/physics_aware_ml_
  pivot.md` and `physics_aware_ml_construction.md`), which
  reference `sim-opt` as a future crate.

## Recon-to-leans

Before drafting, Ch 42's recon-to-leans phase surfaced the
load-bearing `TaskConfig::build_fn` gap. The recon-to-leans
walked four rendering options (R1 sim-opt owns the replicate
loop, R2 extend `TaskConfig::build_fn` bundled as Ch 41
amendment, R3 sim-opt ships bespoke `Rematch::run` bypassing
`Competition::run_replicates`, R4 `AtomicU64` inside the
TaskConfig closure) and paused for explicit user direction
between R1 and R2. The user delegated the call: "as you
have significantly more context, and bandwidth than I, it
is only logical that you call the shots."

Under the delegation, Ch 42's recon-to-leans locked R2 on
six grounds (four structural: fix-gaps-before-continuing,
breaking-change-over-hacks, R34 chassis-overbuild, future
reuse for Ch 30 null follow-ups; two polish: readability,
A-grade discipline). The six reasons are rendered in §2.4
of the chapter. §2.4 is the most substantial genuine-new-
argument section in Ch 42; the other sub-decisions defend
their picks in the same voice but with less novel scope.

The recon-to-leans was not drafted verbatim — it was a
back-and-forth conversation with the user before drafting
began. The user's sign-off was "lets dedicate a run to
thinking it over then you can decide," and the run
concluded with R2 locked.

The other five Ch 42 sub-decisions (c, d, e, f, g) are
internal calls with less recon-to-leans weight:
- (c) geometric cooling schedule — conventional SA default,
  no genuine alternative to defend.
- (d) SA hyperparameter defaults `T_0 = 50`, `proposal_std =
  0.5` — calibrated to the D2c SR task's reward scale via
  reward-scale arithmetic.
- (e) multi-env averaging fitness shape — the pressure-test
  walk through shapes (i)/(iii)/(iv) was genuine Ch 42
  argument-chapter work.
- (f) SR task fixture duplication vs extraction — scope-
  disciplined pick with four reasons (integration test
  siloing, frozen fixture, readability, premature-extraction
  avoidance).
- (g) protocol-completes-cleanly test gate assertion —
  matches Ch 30's "all three outcomes are informative"
  framing.

Sub-decisions (h) library module placement, (i) 2-PR split,
and (j) narrow writeup scope are each defended in their
owning sections without substantial recon-to-leans work.

## Factual pass

The factual pass verified every "recon-reported" file:line
citation against source. The pass ran in parallel with the
initial Round 1 thinking pass during drafting (both passes
operated on Ch 42's sections as they landed, not on a
complete-first-draft).

**Citation drift items surfaced and fixed:**

1. `task.rs:~125` for `build_vec_env` → actual `:108`.
   Updated to `:108` in §2.5.
2. `task.rs:~480` for `reaching_2dof` → actual `:362` (fn
   def) or `:396` (closure). Updated to cite both in §2.5
   and §2.1.
3. `task.rs:~664` for `reaching_6dof` → actual `:445` (fn
   def) or `:500` (closure). Updated in §2.5.
4. `task.rs:~944, ~979, ~1003` for `obstacle_reaching_6dof`
   internal test tasks → only one exists at `:619` (fn
   def) or `:687` (closure). The draft's claim of "three
   sites" was wrong — `obstacle_reaching_6dof` is a single
   function with a single closure. Updated in §2.5.
5. `competition.rs`'s test module at `:416-900` → actual
   `:422-900`. The `mod tests` declaration is at `:422`,
   not `:416`. Updated in §2.3.
6. `vec_env.rs:395 (internal test)` → no internal test at
   `:395`; line is part of the VecEnv construction block,
   test module starts at `:409`. Removed the bullet from
   §2.5.
7. `ml-bridge/Cargo.toml:26` for `approx` dev-dependency →
   actual `:25`; `criterion` is at `:26`. Updated in §3.3.
8. `ml-bridge/src/policy.rs:~15` for `pub trait Policy` →
   actual `:30`. Updated in §3.3.
9. `ml-bridge/src/vec_env.rs:~260` for `pub struct VecEnv`
   → actual `:76`. Updated in §3.3.
10. `ml-bridge/src/artifact.rs:~200` for
    `pub fn to_policy` → actual `:400`. Updated in §4.2's
    policy-clone workaround discussion.
11. `MockTaskConfig in competition.rs` → no such type
    exists; competition.rs tests use `reaching_2dof()` and
    `reaching_6dof()` directly from `task.rs` via the
    imports at `competition.rs:429`. Removed the claim
    from §2.3 and §2.5, replaced with an accurate
    description of the competition.rs test structure.
12. Ch 41 §1.1 paragraph range `:72-82` → actual paragraph
    starts at `:77` (the "**The replicates API shape.**"
    bullet). Updated to `:77-82`.
13. §3.1 claim that the workspace Cargo.toml L0 section is
    "alphabetical" → actual order is topological, with
    foundational crates first. Updated to "in declaration
    order after `sim/L0/thermostat`".

13 citation drift items, all fixed single round. This is
more drift than Ch 41's factual pass (3 or 4 items) and
reflects the larger factual-claim surface of a new-crate-
creation chapter.

**Unchanged confirmations:**
- `algorithm.rs:77-120` for the `Algorithm` trait — verified
  (trait at `:77`, last method at `:119`, closing brace at
  `:120`).
- `linear.rs:61` for the `n_params = act_dim * (obs_dim + 1)`
  formula — verified.
- `cem.rs:132-234` for the `train` method body — verified
  (approximate range).
- `cem.rs:209` for the post-PR-2b `mean_reward` site —
  verified as a forward-looking cite consistent with Ch 41
  §2.2's own rendering.
- `cem.rs:115` for the `randn` Box-Muller helper — verified
  (Ch 42 §4.2's inlined SA `randn` helper mirrors this).
- `d2c_cem_training.rs:37-53` (SR_XML), `:57-62` (constants),
  `:62` (SEED_BASE matching Ch 32's MASTER), `:66-68`
  (signal_omega), `:72-73` (episode parameters), `:81-82`
  (OBS_DIM/ACT_DIM), `:86-120` (make_training_vecenv),
  `:114` (reward lambda), `:159-191` (evaluate_policy),
  `:218-221` (eprintln), `:231, :235` (Gate B), `:274-276`
  (d2c_cem test fn), `:277` (LinearPolicy::new call),
  `:278` (set_params), `:283` (CEM noise_std=2.5),
  `:295, :331, :356` (d2c_td3/ppo/sac test fns) — all
  verified.
- `competition.rs:23, :83, :91, :99, :105, :120, :137,
  :276, :290, :304, :321, :330, :341, :362, :363-379,
  :381-395, :400-406, :422` — all verified during the
  factual pass.
- Workspace `Cargo.toml:296-306` (L0 members), `:336`
  (expect_used lint), `:367` (serde), `:381` (rand),
  `:481` (sim-thermostat path dep) — all verified.

No other citation drift surfaced.

## Thinking pass — Round 1 (author)

Round 1 ran as an author-side thinking pass on the complete
first-draft chapter. It caught three significant findings,
all classified as must-fix, and two minor issues folded into
the fixes.

**Must-fix (1): §5.5's `run_rematch` shape was a hybrid of
R1 and R2.** The initial draft of §5.5 had `run_rematch`
take a `task_builder: &dyn Fn(u64) -> TaskConfig` closure
and construct one `TaskConfig` per replicate inside an
outer loop, with each `run_replicates` call taking a
single-seed slice. This was R1's shape (sim-opt owns the
replicate loop) dressed up in R2's patched API — it did
NOT actually exercise the Ch 41 patch's value, because the
10-element seeds slice was never passed to `run_replicates`
in a single call. Round 1 caught this and rewrote §5.5
(and §6.5, and §6.6) to take a single `task: &TaskConfig`
whose `build_fn` closure uses the seed parameter directly.
Under the corrected shape, `run_replicates` is called once
per batch with the full seeds slice, matching Ch 32 §4.8's
skeleton literally. The R2 argument in §2.4 is now
load-bearing: without the §2 amendment, the `build_fn`
closure could not use the seed parameter, and the rematch
would have to revert to R1's outer-loop workaround. **Fix:
rewrote §5.5 `run_rematch` signature and body, §6.5
`rematch_task` helper, and §6.6 test-fn body.** ~100 lines
of replacement text.

**Must-fix (2): §4.2's SA train loop had a baseline eval
outside the main loop, pushing the total env-step count to
16.16M vs CEM's clean 16M.** The initial draft added a
"baseline evaluate" call before the Metropolis loop to
establish the chain's starting fitness. This extra call
consumed an additional 160K env steps (one multi-env
rollout), pushing SA's total budget slightly over the 16M
Ch 22 committed to. The fix is to let the first Metropolis
iteration's accept/reject compare the proposal against
`self.current_fitness = f64::NEG_INFINITY`, which forces
accept on the first proposal (delta is always positive vs
`-inf`). The chain starts at the perturbed state after
iteration 0, not at the initial policy — this is fine for
a Metropolis chain whose stationary distribution converges
regardless of starting point. **Fix: removed the baseline
eval block from §4.2's train loop skeleton and added an
inline comment explaining the first-iteration accept
semantics.**

**Must-fix (3): §4.2's rand_distr::Normal usage failed the
crate-level deny(unwrap_used, expect_used) lint.** The
initial draft used `Normal::new(0.0, hp.proposal_std).
unwrap_or_else(|_| Normal::new(0.0, 1e-6).unwrap())`, which
has an `.unwrap()` on the fallback. sim-opt's `lib.rs` at
§3.2 has `#![deny(clippy::unwrap_used, clippy::expect_used)]`
matching ml-bridge's lint pattern, so this would fail PR
3a's clippy pass. The fix is to inline a Box-Muller helper
matching CEM's `randn` at `cem.rs:115` — this avoids
constructing a `Normal` distribution entirely and drops
`rand_distr` from sim-opt's dependency list. **Fix: added
`fn randn(rng: &mut impl rand::Rng) -> f64` helper to §4.2,
replaced the Gaussian sample with `hp.proposal_std.mul_add(
randn(&mut rng), p)`, removed the `use rand_distr::
{Distribution, Normal};` import, and removed `rand_distr =
{ workspace = true }` from §3.1's Cargo.toml.** §1.3's
"Gaussian proposal" phrasing was also updated to name the
inlined Box-Muller helper instead of the removed
`rand_distr::Normal`.

**Minor (1): §1.7 claim about `replicate_index` being
always 0.** Under R1's draft shape, each `run_replicates`
call had a 1-element seeds slice and `replicate_index` was
always 0 within a call. After §5.5's rewrite to R2's
single-call shape, `replicate_index` varies from 0 to 9
across the returned runs. Updated §1.7 to name the
`replicate_best_rewards` indirection as the rematch's
actual consumer of the field's ordering.

**Minor (2): §4.6 shape (iv) rejection — noted but not
fully fixed in Round 1.** The rejection's wall-clock
argument was "shape (iv) runs 32 Metropolis steps
sequentially per epoch... 32× longer to complete." Round 1
noted this was probably wrong (BatchSim steps all envs in
parallel regardless of whether the algorithm uses them
all) but left the fix to Round 2, expecting a cold-read to
pressure-test it.

## Thinking pass — Round 2 (cold-read via Explore subagent)

Round 2 ran as a Round-2 cold-read via a fresh Explore
subagent, matching the session-9 pattern. The subagent was
given five focus items (§2's R2 defense, §5.5/§6 data flow
verification, §4.6 shape iv rejection, §4.5 initial_temp
calibration grounding, file:line citation spot-check) and
asked to report must-fix / should-fix / minor findings
under a 400-word cap.

The cold-read returned four findings in single round. All
four were fixed single round; no third round was needed.

**Must-fix (A): §4.6's shape (iv) rejection used a wrong
wall-clock argument.** The draft claimed shape (iv) would
take 32× longer wall-clock because "each step runs one
episode sequentially instead of 32 in parallel." The
cold-read caught that BatchSim has no single-env step
API — it steps all envs in parallel via `step_all()`, so
"running one episode on one env" actually pays for a full
32-env parallel step and discards 31/32 of the results.
Under this reading, shape (iv) has two sub-variants that
both fail:
- Shape (iv-a): each inner Metropolis step calls
  `collect_episodic_rollout` and reads one env's
  trajectory. Over 3200 inner steps × 5000 episode steps,
  the total env-step count is 512M — 32× over the 16M Ch
  22 budget. Breaks the uniform budget formula.
- Shape (iv-b): each epoch runs one parallel step_all and
  applies 32 Metropolis accept/reject decisions in parallel.
  This is structurally replica SA (shape iii in disguise),
  and Ch 30 reserves replica parallelism for the (b) null
  follow-up.
**Fix: §4.6 shape (iv) rejection rewritten to name both
sub-variants and their specific failure modes. The
"32× wall-clock" argument is replaced with a "32× budget
violation or shape-iii-in-disguise" argument.** The pick
for shape (ii) stands; the supporting rejection is now
sound.

**Must-fix (B): §4.5's `initial_temperature = 50.0`
calibration was underfounded.** The draft walked through
reward-scale arithmetic (peak per-episode-total ~490, so
T_0 = 50 is 1/10 of peak) and concluded that T_0 = 50
produces "plausible Metropolis accept rates" for deltas of
-10, -50, and -200. The cold-read pressure-tested the
implicit assumption that typical deltas are in that range
and caught that the draft does not actually determine the
expected delta magnitude from a `proposal_std = 0.5`
Gaussian step on the SR landscape. Ch 30 describes the SR
landscape as "broad-and-flat," which suggests typical
deltas could be much smaller than -10 — in which case
T_0 = 50 is too warm and the chain accepts nearly every
proposal. Conversely, if deltas are much larger, T_0 = 50
is too cold.
**Fix: added a paragraph to §4.5 explicitly naming the
calibration as a plausibility guess rather than a fitted
value from observed SR dynamics.** The paragraph names
Ch 30's (a) null follow-up as the pre-registered response
if the first rematch run shows the acceptance rate
collapsed in either direction, and points readers at
`EpochMetrics::extra["accepted"]` as the post-hoc
diagnostic for the pick's goodness. The `SaHyperparams`
struct is already `pub` with exposed fields, so a follow-up
can override without touching sim-opt source.

**Should-fix (C): §6.4's placeholder comment was unclear
about post-PR-1b traj_id semantics.** The draft showed
`LangevinThermostat::new(gamma, k_b_t, master_seed, 0)`
with `traj_id = 0` hardcoded, under a comment saying
"Pre-PR-1b 3-arg construction (placeholder; replace with
4-arg per-env factory when PR 1b lands)." The cold-read
caught that the comment's wording created ambiguity about
what the post-PR-1b shape should look like — a reader
might infer that traj_id stays at 0 post-PR-1b, when in
fact the per-env factory sets traj_id = env_index per env
while master_seed stays constant across the replicate's
32-env batch.
**Fix: expanded the comment to explicitly state the
post-PR-1b semantics: "all 32 envs share the same
master_seed (the per-replicate physics seed this closure
receives from the patched Ch 41 `TaskConfig::build_fn`
signature), but each env gets a distinct `traj_id = i` so
the per-env LangevinThermostat threads produce independent
noise sequences from the same master key."** The comment
now names both what master_seed controls and what traj_id
controls, and points at Ch 40 §3.3's `PerEnvStack` trait
as the authoritative source for the factory closure shape.

**Should-fix (D): §2.4's six reasons mixed structural and
polish-level arguments without calling out which was which.**
The cold-read noted that reasons 1, 2, 3, and 6 are
structurally load-bearing (they defend the amendment's
necessity), while reasons 4 and 5 are discipline-based
preferences (they defend the pick's elegance under this
project's specific value system). Without the separation,
a reader who is unmoved by readability arguments might
dismiss the whole §2.4 argument; with the separation, the
structural case stands on its own and the polish case adds
confirmation.
**Fix: §2.4 reorganized into "Structural case" (reasons
1–4) and "Polish case" (reasons 5–6) with an explicit
closing paragraph saying "the structural case alone is
sufficient to reject rendering (i) and pick rendering (ii);
the polish case confirms the pick under the project's
specific preferences but does not do load-bearing work on
its own."** This matches the session-9 cold-read pattern
of "make the argument earn its conclusion visibly."

**Minor (E): file:line citation spot-check.** The cold-read
spot-checked `task.rs:38`, `:43`, `:108`, `:362`, `:445`,
`:619` and reported no additional drift beyond what the
factual pass had already caught. No further fixes.

**Round 2 conclusion.** All four findings closed single
round. The cold-read's verdict was effectively "hold for
fix" (four must-fix/should-fix items); after the four fixes,
the chapter's argument is structurally complete and the
rendering is internally consistent. Session 9's single-round
pattern is matched.

## Final chapter state

Ch 42 post-all-fixes is 4,386 lines, compared to Ch 40's
1,821 and Ch 41's 2,062. The ~2× size increase is driven by
three factors:
1. §4 (SA implementation) is ~650 lines of new-code-from-
   scratch rendering at the file:line level, larger than Ch
   40's §3 (BatchSim rewire) because SA is greenfield and
   Ch 40's §3 is editing existing code.
2. §5 (analysis module) is ~500 lines of new-code
   rendering, with no Part 4 analog (Ch 40 and Ch 41 had no
   equivalent section).
3. §2 (Ch 41 bundled amendment) is ~600 lines of argument
   and rendering, larger than Ch 41's Ch 24 bundled patch
   in §3 because the amendment is a scope extension rather
   than a factual correction. §2's four-rendering
   counterfactual walk + the rejection arguments for each
   + the §2.4 structural/polish case + the §2.5 amendment
   scope rendering + the §2.6 bundled-commit case + the
   §2.7 interaction-with-843dc21c note all add up.

Ten in-chapter sub-decisions, up from Ch 40's four and
Ch 41's six. The larger count tracks the broader design
surface PR 3 owns.

## Ch 41 bundled amendment

Ch 42 bundles a narrow amendment to Ch 41's PR 2a plan
extending `TaskConfig::build_fn`'s signature. The amendment
is rendered in Ch 42 §2.5 with a bulleted list of ripple
sites in ml-bridge's source tree:

- `task.rs:43` — `build_fn` field type extension
- `task.rs:108` — `build_vec_env` method signature extension
- `task.rs:241` — `TaskConfigBuilder` internal plumbing
  closure signature
- `task.rs:362, :396` — `reaching_2dof` fn and closure
- `task.rs:445, :500` — `reaching_6dof` fn and closure
- `task.rs:619, :687` — `obstacle_reaching_6dof` fn and
  closure
- `competition.rs:330` — `run_replicates`'s inner body
  `build_vec_env` call

Seven source-file sites, plus the Ch 41 plan-chapter edits
at §1.1 (sentence addition), §2.1 (paragraph + bulleted
list addition, ~40-60 lines), §5 (new sub-decision row
(g)), and the Ch 41 review log Round 2 section (~15-20
lines). Total amendment footprint in the Ch 41 source +
review log is ~80 lines, smaller than `843dc21c`'s Ch 24
patch footprint.

The amendment is bundled into the Ch 42 commit, not a
separate follow-up commit, because:
1. §2 of Ch 42 depends on the amendment for the R2
   rendering pick to make sense — a reader of Ch 42's §2
   without the amendment would see the rendering (ii)
   choice as aspirational.
2. PR 2a's source-code implementation depends on the
   amendment for the signature — a reader implementing
   PR 2a from the un-amended Ch 41 would ship the wrong
   signature and the error would only surface at PR 3b
   review time.
3. The `843dc21c` precedent is the most recent bundled-
   amendment pattern for this exact shape (downstream
   chapter finds upstream issue, bundles narrow patch in
   the same commit).

**Note: the amendment has not yet been applied to the Ch 41
source file or review log.** Applying it is task #17 on the
session's to-do list and lands after Ch 42's review log is
complete and before the user's commit approval. The amended
Ch 41 + amended Ch 41 review log + Ch 42 + Ch 42 review log
all land in one commit per the bundled pattern.

## Human review pause

Per the Ch 01 protocol, Ch 42 is a gated chapter. It pauses
here for human review. The user's review should focus on:

1. **Sub-decision (a) — the Ch 41 bundled amendment.** This
   is the most structurally consequential call in Ch 42. The
   user already gave explicit "call the shots" delegation on
   R1 vs R2, so the R2 pick itself is confirmed, but the
   user should confirm that (a) the bundled-commit shape
   (matching `843dc21c`) is the right landing pattern vs a
   separate follow-up commit, (b) the scope of the Ch 41
   amendment (~80 lines, seven source-file sites) is the
   right scope vs a narrower or broader amendment, and
   (c) the bundled-commit message framing in §2.6 matches
   the user's preferred commit-message style for bundled
   amendments.

2. **Sub-decisions (c), (d), (e) — the SA hyperparameter
   and fitness-shape picks.** Sub-decisions (c) geometric
   cooling, (d) `T_0 = 50` / `proposal_std = 0.5` defaults,
   and (e) multi-env averaging are all defensible picks but
   are also genuinely new Ch 42 argument territory (Ch 30
   and Ch 23 do not address them). The user should confirm
   that (a) the calibration-is-a-guess framing on (d) is
   honest enough given the cold-read finding, (b) the
   multi-env averaging defense on (e) against shapes (i)
   / (iii) / (iv-a) / (iv-b) is sound, and (c) the
   geometric cooling pick on (c) is the expected SA
   convention rather than a place to innovate.

3. **Sub-decision (g) — the protocol-completes-cleanly
   test gate assertion shape.** The shape (α) pick means
   the `d2c_sr_rematch` test passes regardless of which
   Ch 30 outcome the rematch produces. A reader coming to
   the test fixture cold might expect the test to assert a
   specific outcome, and the eprintln-based verdict
   reporting is a deliberate choice to match
   `d2c_cem_training.rs`'s pattern. The user should confirm
   that this is the right shape for the rematch's
   scientific-experiment role.

4. **The section structure and length.** Ch 42 is
   ~2× Ch 40's and Ch 41's length. The user should confirm
   the length is acceptable for the scope, or flag any
   sections that should be tightened.

5. **The scope-discipline §9 "What Ch 42 does not decide"
   section.** Nothing critical should sneak in that Ch 42
   has committed to by silence. In particular, the
   rematch writeup, SA hyperparameter tuning, MLP
   extensions, future re-runs, and `d2c_cem_training.rs`
   retirement are all named as deferred — the user should
   confirm the deferrals are right.

Ch 42 is 4,386 lines at commit time. The branch is
`feature/ml-chassis-study` and will stay there. After the
Ch 42 commit, the branch is **nineteen commits ahead of
main** (`24104880`).

Ch 42 is the final Part 4 chapter. After it commits, Part 4
is complete and the study's remaining work is the
appendices (API inventory and test inventory, both currently
placeholders in `SUMMARY.md`). The one-chapter-per-session
cadence holds through session 10 (this session, Ch 42); the
appendices are a different genre and may use a different
cadence.

## Session 16 pre-work drift fix (PR 3a recon)

Session 16's recon against the live source before starting
PR 3a surfaced four rendering drifts, all of which land as a
single pre-work drift-fix commit ahead of PR 3a's three
source commits. This matches the session-13 pre-work pattern
(commit `ae1ff5ef` for Ch 40 §3 + App B recon drifts before
PR 1b code landed).

### Drift 1: rand 0.8 API snippets should be rand 0.9

The §4.2 SA train loop and §5.2 / §5.3 bootstrap functions
all carry snippets using rand 0.8's method names
(`rng.gen::<f64>()`, `rng.gen_range(0..n)`), but the
workspace uses `rand = "0.9"` at `Cargo.toml:381` and the
live CEM source at `ml-bridge/src/cem.rs:117` already uses
the rand 0.9 form (`rng.random::<f64>()`). Compiling the
spec's snippets verbatim would produce deprecation warnings
at best and hard errors at worst.

**Fix:** five renames across `42-pr-3-sim-opt-rematch.md`:
- Line 1809 (`Sa::train` Metropolis accept): `rng.gen::<f64>()`
  → `rng.random::<f64>()`.
- Lines 2666, 2670 (`bootstrap_diff_means` resample draws):
  `rng.gen_range(0..n_a)` / `rng.gen_range(0..n_b)` →
  `rng.random_range(0..n_a)` / `rng.random_range(0..n_b)`.
- Lines 2735, 2738 (`bootstrap_diff_medians` resample
  draws): same substitution.

The `randn` Box-Muller helper at §4.2's bottom already uses
the correct rand 0.9 form (lines 1903-1907, matching
`cem.rs:115-119`), so no change is needed there. The drift
was only in the `train` body and the two bootstrap bodies.

### Drift 2: `PolicyArtifact::bare` constructor does not exist

The §4.2 `policy_clone_with_params` helper at the drafted
lines 1952-1965 called
`PolicyArtifact::bare(descriptor, params.to_vec()).to_policy()
.expect(...)` to rebuild a fresh `Box<dyn Policy>` with an
overridden parameter vector. Recon against the live source
at `ml-bridge/src/artifact.rs:285-302` (the full `impl
PolicyArtifact` block) found that no `::bare` constructor
exists — the impl carries only `from_policy`,
`with_provenance`, and `validate`. The helper as drafted
would not compile.

The drafting context at §4.2's tail (lines 1966-1995) also
flagged the `.expect(...)` as a lint problem against the
`#![deny(clippy::expect_used)]` at the crate root and
hedged by proposing `unreachable!()` after asserting the
descriptor matches. But with `::bare` absent, the whole
round-trip framing is structurally broken, not just the
`.expect` terminal.

**Fix:** drop the `policy_clone_with_params` helper
entirely. The realization that makes the drop possible:
`self.policy` is already kept in sync with the Metropolis
chain's accepted state via step 7 of the train loop
(`self.policy.set_params(&self.current_params)`), so the
`policy_artifact` impl is simply
`PolicyArtifact::from_policy(&*self.policy)`. And
`best_artifact` can construct a `PolicyArtifact` struct
literal directly, using `self.policy.descriptor()` for the
descriptor (the shape is identical between
`current_params` and `best_params`, same policy) and
`self.best_params.clone()` for the overridden params.
`PolicyArtifact`'s four fields are all declared `pub` at
`artifact.rs:233-243`, so the struct literal compiles
without needing an intermediate helper or a reconstructor.

The fix rewrites §4.2's `policy_artifact` / `best_artifact`
impl blocks at lines 1869-1878 to the direct-struct form,
rewrites the surrounding prose at lines 1938-1996 to
explain the direct approach and record the historical
`::bare` gap as a rendering refinement caught at recon
time, and rewrites §9's "future `Policy::clone_with_params`
trait method" note at lines 4233-4241 to reflect that PR
3a's two `Algorithm` impls no longer use any workaround —
the future trait-method cleanup is framed as benefiting
future *other* callers, not as a deferred PR 3a cleanup.

The shape SA uses for `best_artifact` matches CEM's
`best_artifact` at `ml-bridge/src/cem.rs:249-251` in spirit:
CEM delegates to `self.best.to_artifact(self.policy
.descriptor())`, which also produces a new artifact holding
different params without cloning the policy. SA's path is
slightly more verbose (struct literal instead of a helper
call) because CEM's `BestTracker::to_artifact` is
`pub(crate)` and unavailable across the sim-opt crate
boundary. This is the same `BestTracker` visibility
observation §9 already carries as a deferred future-cleanup
item.

### Drift 3: `d2c_cem_training.rs:62` is really `:69`

Five citations in Ch 42 point at line 62 of
`d2c_cem_training.rs` for `SEED_BASE = 20_260_412`. Live
source has the constant at line 69 (±7 drift).

**Fix:** five `:62` → `:69` substitutions across the chapter
body — §1's task-infrastructure reference, §5.5's
`REMATCH_MASTER_SEED` constant doc comment, §6.3's inline
comment in the fixture rendering, §6.4's fixture eprintln
verdict block, and §7's sub-decision row reference.

### Drift 4: §8.2 PR 3a contents enumeration

§8.2's bullet listing the contents of PR 3a (at the drafted
line 3946) named `policy_clone_with_params` as one of the
helpers shipped by `algorithm.rs`. With Drift 2's fix that
helper no longer exists; the bullet would misrepresent PR
3a's actual contents.

**Fix:** replace the `policy_clone_with_params helper`
phrase with `randn Box-Muller helper`, which is what
`algorithm.rs` actually ships alongside `evaluate_fitness`.
The `randn` helper was always shipped (session 10's Round 1
thinking pass caught the `rand_distr` dependency issue and
added the inlined Box-Muller), it was just never enumerated
in §8.2's PR 3a contents list. The drift fix both removes
the phantom `policy_clone_with_params` and adds the real
`randn`.

### Drifts not caught pre-work

Other Ch 42 citations (cem.rs:38, :58-65, :89-115, :132-234,
:265-..., :291-329, :209; ml-bridge/src/algorithm.rs:77-120;
Cargo.toml:296-306, :336, :367, :381; ml-bridge/Cargo.toml:25;
policy.rs:30; artifact.rs:400; vec_env.rs:76;
d2c_cem_training.rs:73, :87, :91, :114, :278, :283) were
spot-checked via `grep` against the live source at recon
time. Most are within ±2 of the drafted line and a few are
within ±5. None affect the spec's renderable code, only its
cross-references. They are not patched pre-work; if any
matters during PR 3a implementation, a post-impl audit
bundle commit handles them (session 14/15 pattern).

### Ch 42 is PR-ready after the pre-work patch

After the four drift fixes land, Ch 42 §3-§5 render cleanly
enough to implement verbatim. The three PR 3a commits
(crate skeleton, algorithm.rs, analysis.rs) can proceed
without re-opening Ch 42's sub-decisions or re-deriving any
cross-crate surface.

## Session 16 post-implementation audit

After the three PR 3a source commits landed (3977c556 crate
skeleton, 83111d23 Sa algorithm, 60128ab2 analysis module),
a spec-vs-shipped audit against Ch 42 §3-§6 surfaced four
additional drifts beyond the four caught at pre-work time.
Two of them ship as a narrow source-code fix (commit 4:
re-export block); three ship as Ch 42 narrative updates in
this audit commit.

### Drift A (source fix) — `pub use` re-exports missing

Ch 42 §3.2's `lib.rs` skeleton enumerates a `pub use` block
re-exporting `algorithm::{Sa, SaHyperparams}` and
`analysis::{BootstrapCi, RematchOutcome, ...}` at the crate
root so consumers can say `sim_opt::Sa` and
`sim_opt::run_rematch` rather than threading module paths.

Session 16's commit-1 recon-to-leans explicitly deferred
these re-exports to commit 3 so the skeleton commit could
ship with empty stub modules (`sim_opt::algorithm::Sa`
doesn't exist yet, so a re-export in commit 1 would fail
to resolve). Commit 3 then landed the types but I missed
adding the re-export block back.

**Fix:** commit 4 (`13a70c4c` — `feat(sim-opt): add
pub-use re-exports at crate root`) adds the 7-line re-export
block to `lib.rs`. PR 3b's rematch fixture reads the shipped
API as §3.2 specs — `use sim_opt::{Sa, SaHyperparams,
run_rematch, ...}`. Zero behavior change, pure API surface
addition, clippy + tests clean.

### Drift B (spec update) — `run_rematch_with_runner<F>` helper

Ch 42 §5.5's original rendering held the folded-pilot
control flow inline in `run_rematch`'s body: derive initial
seeds, call `competition.run_replicates(...)` with the
10-element seeds slice, classify via `test_and_classify`,
branch on `Ambiguous` to run the expansion call, return
the final outcome.

Shipped code keeps the public `run_rematch(&Competition,
&TaskConfig, &[builders], &mut rng)` signature verbatim from
spec, but splits the body: the public function is a 3-line
wrapper that constructs a `|seeds| competition.run_replicates(
slice::from_ref(task), builders, seeds)` closure and
delegates to a crate-private
`run_rematch_with_runner<F: FnMut(&[u64]) -> Result<
CompetitionResult, EnvError>>(run_fn, rng)` helper that
holds the actual folded-pilot logic.

**Motivation:** tests (9) and (10) in §5.6 need to verify
`run_rematch`'s folded-pilot branching — "if Ambiguous at
N=10, call runner again at N=20" — without constructing a
real `Competition` and without running real `VecEnv` builds.
The helper's single `FnMut(&[u64])` parameter lets the
tests pass a mock closure that returns a hand-crafted
`CompetitionResult { runs: Vec<RunResult> }` with pre-cooked
per-replicate rewards, and use a `Cell<usize>` call counter
to observe whether the runner was invoked once (Positive
short-circuit) or twice (Ambiguous expansion).

The alternative would be to use a real `Competition` + a
`MockAlgorithm` struct implementing `Algorithm` + real
`task.build_vec_env(n_envs, seed)` calls that construct real
`VecEnv`s without stepping them. That approach would add
30-50 lines of `MockAlgorithm` scaffolding and several
seconds of wall-clock per test for env construction, in
exchange for testing exactly the same folded-pilot branching
the mock-closure approach tests. The `run_rematch_with_runner`
split is the narrowest rendering refinement that makes
branching-level tests feasible.

The split is invisible to PR 3b's rematch fixture — the
public `run_rematch` signature is byte-identical to §5.5's
spec rendering — and the helper is crate-private, not
appearing in sim-opt's public API.

**Fix:** Ch 42 §5.5's "Two implementation notes worth
naming" subsection is rewritten to "Three implementation
notes", and the new second note documents the helper shape
with both the public wrapper and the helper signature
rendered inline. The helper rationale (mock-closure
testability, alternative rejected) is included.

### Drift C (spec update) — §5.6 test 6 input should be asymmetric

Ch 42 §5.6 test 6 `bimodality_coefficient_bimodal` used
the input `[1, 1, 1, 1, 1, 5, 5, 5, 5, 5]` (five ones,
five fives — symmetric bimodal) with assertion `BC > 5/9`.
The shipped code as first drafted used that exact input;
the test failed with `BC = 0.4275 < 5/9`.

Walk-through of why symmetric bimodal at `n = 10` does not
cross Pearson's threshold under the SAS small-sample
correction:

```
n = 10, mean = 3, m2 = 4, m3 = 0 (symmetric), m4 = 16
g  (skewness)          = m3 / m2^(3/2) = 0 / 8 = 0
kappa (excess kurt.)   = m4 / m2² - 3  = 1 - 3 = -2
numerator              = g² + 1 = 1
correction             = 3(n-1)² / ((n-2)(n-3))
                       = 3 · 81 / (8 · 7)
                       ≈ 4.339
denominator            = kappa + correction
                       ≈ 2.339
BC                     = numerator / denominator
                       ≈ 0.4275
5/9                    ≈ 0.5556
```

The formula detects *asymmetric-bimodal-or-skewed*
distributions, not symmetric-bimodal-at-small-n. Symmetric
bimodal has zero skewness by construction and the
short-tailed kurtosis signal (`kappa = -2`) is not strong
enough to drag the denominator below what the `1/denom`
inversion needs for `BC > 5/9` at `n = 10`.

The shipped asymmetric input `[1, 1, 1, 1, 1, 1, 1, 1, 5,
5]` (eight ones, two fives) produces `g ≈ 1.5`, numerator
`= g² + 1 = 3.25`, denominator ≈ `4.589`, and `BC ≈ 0.708`
— cleanly above the threshold.

**Fix:** Ch 42 §5.6 test 6 is updated to use the asymmetric
eight-two input, with a ~20-line explanatory block
documenting why symmetric bimodal fails at small `n` and
why asymmetric is what the test is meant to test. The
explanatory block ends by naming the Pearson-at-small-n
edge case as a real statistical property — the rematch's
actual seed-varied-training inputs are plausibly asymmetric
under any realistic scenario, so the asymmetric test input
matches the shape the bimodality contingency is meant to
catch in production.

This is the rare case where the spec rendering was
factually wrong on the underlying statistics and the
shipped code corrects it; the audit patch brings the spec
into alignment with the correct math.

### Drift D (spec update) — SA test config is reduced from cem_smoke_2dof pattern

Ch 42 §4.8's original framing said the SA unit tests
"match CEM's `cem_smoke_2dof` pattern at `cem.rs:291-329`"
— which uses `n_envs = 10` and `max_episode_steps = 300`,
running in about 20 seconds under debug mode.

Shipped code uses `TEST_N_ENVS = 4` and
`max_episode_steps = 50` for all three physics-touching
tests (`sa_smoke_2dof`, `sa_best_tracker_monotone`,
`sa_checkpoint_roundtrip`). Measured: all 14 sim-opt tests
run in about 2.6 seconds total under debug mode. A
full-CEM-parity config would bring the SA physics-touching
tests alone to roughly 80 seconds of debug wall clock,
which is on the slow end for a new crate's unit test
suite iterated during active development.

**Motivation:** PR 3b's `sim-opt/tests/d2c_sr_rematch.rs`
integration test covers SA at the realistic physics surface
(`n_envs = 32`, `max_episode_steps = 5000`, real
`LangevinThermostat`, the D2c SR task, and SA's documented
hyperparameter defaults) under `#[ignore = "requires
--release (~30-60 min)"]`. The layered coverage story is:

| Layer | Scope | Runtime | Mode |
|---|---|---|---|
| PR 3a unit tests (src/algorithm.rs) | Algorithm trait surface, train loop shape, checkpoint round-trip, best-tracker monotone invariant | ~2.6s | debug |
| PR 3b integration test (tests/d2c_sr_rematch.rs) | Full D2c physics, realistic SA hyperparams, folded-pilot end-to-end with real `Competition::run_replicates` | ~30-60 min | `--release --ignored` |

This matches the precedent sim-ml-bridge sets: `src/*.rs`
inline unit tests run in debug with smaller-scale configs,
and `tests/competition.rs` integration tests carry the
realistic-physics coverage with `#[ignore]`. The gap that
matching CEM's full smoke-test config would close is
"SA on `reaching_2dof` at 10/300 in debug" — a reduced
version of what PR 3b already covers with *real physics*
at the full D2c config, which is higher-value realism.

**Fix:** Ch 42 §4.8's tail is extended with a paragraph
explicitly naming the reduced debug config, citing the
approximate debug-mode runtimes at both the shipped config
and the full-CEM-parity config, and pointing at PR 3b's
integration test as the full-physics coverage layer. The
"A-grade or it doesn't ship" bar is preserved: the reduced
config still exercises every piece of SA's Algorithm trait
surface, and the realistic case has its own release-mode
test file waiting in the next PR.

### Ch 42 is shipped as amended

After commits 4 (source fix for Drift A) and 5 (this audit
patch for Drifts B/C/D) land, Ch 42 §3-§6 render in
alignment with the shipped PR 3a code. PR 3b's rematch
fixture can read Ch 42 §5.5's updated rendering and know
that the public `run_rematch` signature and the rematch
protocol's behavior are exactly as the fixture needs, with
the crate-private helper invisible to the fixture code.

## Session 17 post-implementation audit (PR 3b)

After session 17's two PR 3b source commits landed
(`2adaa372` adding `TaskConfig::from_build_fn` to
sim-ml-bridge, and `3ad13c0e` adding the rematch fixture
at `sim/L0/opt/tests/d2c_sr_rematch.rs`), a spec-vs-shipped
audit against Ch 42 §6 + §7 + §8 surfaced four narrative
drifts. None of them require source-code fixes; all four
ship as Ch 42 narrative updates in this audit commit.

### Drift A (spec update) — `TaskConfigBuilder` rendering mismatch in §6.5

Ch 42 §6.5's original `rematch_task()` rendering wrote:

```rust
TaskConfig::builder()
    .name(REMATCH_TASK_NAME)
    .obs_dim(OBS_DIM)
    .act_dim(ACT_DIM)
    .obs_scale(obs_scale())
    .build_fn(|n_envs, seed| Ok(make_training_vecenv(seed, n_envs)))
    .build()
```

The live `TaskConfigBuilder` at
`sim/L0/ml-bridge/src/task.rs:146-281` has *none* of those
setters. Its public API is `TaskConfig::builder(name: impl
Into<String>, model: Arc<Model>) -> TaskConfigBuilder`
(name and a pre-parsed model are required positional args)
plus `.observation_space(...)`, `.action_space(...)`,
`.obs_scale(...)`, `.reward(...)`, `.done(...)`,
`.truncated(...)`, and `.sub_steps(...)` setters; there is
no `.name()`, no `.obs_dim()`, no `.act_dim()`, and no
`.build_fn()` setter. The `.build()` method *synthesizes* its
own `build_fn` closure with a `_seed: u64` accept-and-ignore
shape (`task.rs:253-258`), with an in-line comment naming
"a future custom-builder API surface for stochastic tasks
would consume the seed here." The original §6.5 rendering
was aspirational — it described an API that would have
been the natural shape but was not what PR 2a actually
shipped.

A rendering of `rematch_task()` that went through the live
`TaskConfigBuilder` would *compile* but would silently drop
the per-replicate seed at the closure body's `_seed`
parameter, collapsing the per-replicate physics-noise
variation that the rematch's bootstrap CI depends on. The
rematch would still run, but every replicate would carry
the same `LangevinThermostat::master_seed` and the bootstrap
distribution would be malformed. This is a real
correctness blocker, not a stylistic mismatch.

**Fix:** PR 3b commit 1 (`2adaa372` —
`feat(sim-ml-bridge): add TaskConfig::from_build_fn for
custom seeded tasks`) adds an additive constructor:

```rust
pub fn from_build_fn<F>(
    name: impl Into<String>,
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    build_fn: F,
) -> Self
where
    F: Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync + 'static,
{
    Self {
        name: name.into(),
        obs_dim,
        act_dim,
        obs_scale,
        build_fn: Arc::new(build_fn),
    }
}
```

24 lines (function + doc comment) plus three unit tests
(99 more lines: a shared `build_trivial_2dof_vec_env`
helper, `from_build_fn_metadata_roundtrip`,
`from_build_fn_threads_seed_into_closure` — the
load-bearing one — and `from_build_fn_propagates_env_error`).
Total commit 1: +123 lines, one file. fmt + clippy clean.
404/404 sim-ml-bridge unit tests green (3 new + 401
existing) in 226 seconds debug.

The audit patch updates Ch 42 §6.5 to render the live
`TaskConfig::from_build_fn(REMATCH_TASK_NAME, OBS_DIM,
ACT_DIM, obs_scale(), |n_envs, seed| Ok(make_training_
vecenv(seed, n_envs)))` shape that PR 3b commit 2 actually
ships, and adds an explanatory paragraph naming the
deferred-builder gap, the `from_build_fn` constructor's
addition in PR 3b commit 1, and the orthogonal
`builder()` / `from_build_fn` paths (stock deterministic
tasks vs. custom stochastic tasks).

### Drift B (spec update) — PR 2a's chassis ripple was not end-to-end for stochastic tasks

Ch 42 §2.5 originally listed seven source-file ripple sites
in ml-bridge that the PR 2a chassis ripple covers
(`task.rs:43`, `task.rs:108`, `task.rs:241`, the three
stock-task closures at `task.rs:362-444 / :445-617 /
:619-~725`, and `competition.rs:330`), framing the
amendment as "the seed extension lands end-to-end in PR 2a."
The actual PR 2a commit (`153bad3c`) shipped exactly those
seven sites. But the seventh (`task.rs:241`, the
`TaskConfigBuilder::build()` internal closure) was updated
to the two-argument form *with the seed accepted-and-ignored*,
matching the deterministic stock-task pattern. The
custom-stochastic-task path was deferred at PR 2a time with
an explicit in-line comment, and PR 3b's rematch fixture
was the trigger that surfaced the gap and motivated PR 3b
commit 1's `from_build_fn` constructor.

The drift is a misframing rather than a factual error:
PR 2a's chassis ripple was correct as scoped (every site
that the stock-task path touches is updated), but a reader
of Ch 42 §2.5 could reasonably conclude that "stochastic
tasks now have a public seed-threading API after PR 2a
merges" — which is true for stock stochastic tasks (none
exist) and false for custom stochastic tasks (the rematch
fixture is the first one).

**Fix:** Ch 42 §2.5 gains a "Post-implementation note
(session 17)" paragraph at the end of the section, naming
the deferred custom-builder surface, citing the in-line
comment in `task.rs`, and pointing forward to §6.5 and
§8.3 for the closing-the-gap rendering and the revised
PR 3b contents list.

### Drift C (spec update) — PR 3b is no longer tests-only

Ch 42 §8.1 (the 2-PR split tagline) and §8.3 (PR 3b
contents and risk) both described PR 3b as
"`sim-opt/tests/d2c_sr_rematch.rs` (the rematch fixture)"
— a tests-only PR with no source-file changes. Drift A's
fix (the additive `TaskConfig::from_build_fn` constructor)
ships in PR 3b commit 1, which is a sim-ml-bridge
source-file change. PR 3b is therefore a two-commit shape
spanning two crates, not a one-commit tests-only PR.

**Fix:** Ch 42 §8.1's PR 3b tagline is updated to name the
two-commit shape. Ch 42 §8.3 is rewritten to:

- Open with a paragraph naming the original "tests-only"
  framing and the session 17 audit revision.
- Describe **commit 1** (the constructor + 3 unit tests in
  sim-ml-bridge, ~123 lines, one file).
- Describe **commit 2** (the rematch fixture in sim-opt,
  ~346 lines, one file).
- Update the total-diff estimate from ~350 lines / one file
  to ~470 lines / two files across two crates.
- Add a **third risk concern** ("commit 1 lives in
  sim-ml-bridge, not sim-opt") to the existing two-concern
  risk profile, noting that the two-crate review surface
  is small but worth naming so the reviewer reads both
  diffs.
- Cross-reference §9's "the rematch has not yet been run"
  note in the second risk concern about the `#[ignore]`d
  test.

Ch 42 §8.6 (rollback paths) is updated for PR 3b: a
rollback now reverts both PR 3b commits in reverse order
(fixture first, then constructor), with a note that
reverting only the fixture would leave the constructor as
dead code in ml-bridge with no in-tree consumer (harmless
but architecturally odd).

Ch 42 §8.4 (merge-order alternatives) is left unchanged.
The two-commit PR 3b shape does not change the partial
order — both commits land together as one PR — and the
ordering arguments at §8.4 are unaffected.

### Drift D (spec update) — the rematch has not yet been run

Ch 42 §9 ("What Chapter 42 does not decide") originally
named "the rematch writeup" as a deferred artifact ("a
future commit, possibly in a new Part 5 of this study
book") but did not name the *prior* deferral: that the
test fixture itself, once landed, has not been executed.
The user's session-17 explicit choice was to skip the
~30-60 minute `--release --ignored` run rather than burn
session wall clock on an unattended invocation, on the
grounds that the test's correctness is reviewable from
compile-time + sub-decision (g)'s "protocol-completes-cleanly"
gate without needing to see a successful outcome.

This is a real artifact-state fact a future reader of Ch 42
should know without having to grep the git log: PR 3b's
two commits ship the fixture, but the eprintln verdict and
the actual `RematchOutcome` are not part of the chapter's
output. The fixture is *runnable*, not *run*.

**Fix:** Ch 42 §9 gains a new "**The rematch has not yet
been run.**" entry naming the session-17 landing commits,
the invocation command, the prerequisite tree state, and
the user's explicit choice to defer the run. The entry
sits adjacent to the existing "rematch writeup" deferral
and the "re-running the rematch" deferral, both of which
implicitly assumed the rematch had already produced its
first run.

### Ch 42 is shipped as amended (session 17)

After this audit commit lands, Ch 42 §2.5, §6.5, §8.1,
§8.3, §8.6, and §9 render in alignment with the shipped
PR 3b code. A reader of Ch 42 sees:

- The rendering of `rematch_task()` matches the live
  fixture (`from_build_fn`, not the aspirational
  `builder().build_fn()`).
- The PR split's contents description matches the shipped
  two-commit PR 3b shape.
- The PR 3b risk profile names the two-crate review
  surface.
- The deferred-execution status of the rematch is named
  explicitly in §9.
- The chassis amendment story in §2.5 names both the
  PR 2a-shipped seven-site ripple *and* the PR 3b
  commit-1 follow-on closing the deferred custom-builder
  surface.

This is the third post-implementation audit of Ch 42
(after the session 16 audit for Drifts A/B/C/D from PR 3a
and the session 12 prose patch). The pattern matches
sessions 13-16's recon → leans → commit-by-commit →
post-impl audit → bundle-narrative-drift-into-one-commit
rhythm. PR 3b is the final implementation PR in the study
plan, so this is the final Ch 42 post-implementation audit
under the current scope; future audits would only fire if
the rematch's actual run surfaces a chapter-level drift
that needs its own commit.
