# Review log: Ch 23 Competition API v2

Chapter type: **argument chapter**. Makes three high-stakes design
decisions — the replicates API shape, PPO pool-membership for the
rematch, and the operational definition of "matched complexity."
Phase 3 **gated chapter** — paused for user review before commit
per the Ch 01 protocol.

Passes run **sequentially** per the Phase 2 process note for
design chapters: factual pass first, discrepancy fix, then
thinking pass. The thinking pass surfaced substantive findings
that triggered a second round of the thinking pass (per Ch 01's
"when a second round is triggered" clause) on the revised
chapter.

## Pre-draft recon

Ch 23 required direct source recon before drafting — the Phase 3
roadmap notes that Ch 23 and Ch 24 each need recon on the current
Competition API and the five algorithms' parameterizations before
drafting. Recon was delegated to Explore agents and returned three
load-bearing findings:

1. **Competition API surface.** `Competition::new(n_envs, budget,
   seed: u64)` at `sim/L0/ml-bridge/src/competition.rs:290` takes
   a single `u64` seed and threads it directly to
   `Algorithm::train(seed)` at `competition.rs:341`.
   `CompetitionResult { runs: Vec<RunResult> }` at `:83-86`;
   `RunResult` fields at `:23-34` (`task_name`, `algorithm_name`,
   `metrics`, `artifact`, `best_artifact`); `best_reward()` at
   `:43-50` is max-over-per-epoch-means. Error channel is
   `Result<CompetitionResult, EnvError>` from
   `task.build_vec_env()` at `:330`; algorithm failures caught
   post-hoc via `assert_finite()` at `:66-76`.
2. **Algorithm parameterization.** CEM at `cem.rs:59` takes
   `Box<dyn Policy>`; REINFORCE at `reinforce.rs:62` and PPO at
   `ppo.rs:71` take `Box<dyn DifferentiablePolicy>`; TD3 at
   `td3.rs:83` takes `Box<dyn DifferentiablePolicy>` with twin Q;
   SAC at `sac.rs:88` takes `Box<dyn StochasticPolicy>` with
   learned `log_std`. **SA is not implemented** — spec-only,
   will live in sim-opt when cut.
3. **PPO's D2c failure characterization.** The D2c study ran
   exactly four algorithms (CEM, TD3, PPO, SAC) at
   `sim/L0/thermostat/tests/d2c_cem_training.rs:274-387` — no
   REINFORCE test. PPO's "D1d-style exploration-noise inflation"
   label in `project_d2_sr_findings.md` traces back to
   `sim/L0/thermostat/tests/d1d_reinforce_comparison.rs:256-260`,
   where the pattern was first named on REINFORCE on a Brownian
   ratchet task — a different task family from SR's broad-and-
   flat landscape. The D2c PPO characterization is an analogy
   from D1d plus an outcome-level memo reading, not an
   independent empirical replication.

A follow-up recon resolved a fork the initial recon surfaced: the
REINFORCE-parallel concern. Initial draft leans on "exclude PPO"
raised the question of whether REINFORCE should be excluded too
by symmetry. Follow-up recon confirmed REINFORCE was never in
D2c, so the rematch pool (which inherits D2c's scope minus Ch 30
exclusions) never had REINFORCE as a candidate. The "symmetry
concern" dissolved. The D2c CEM baseline's concrete policy type
was also verified in the follow-up pass:
`LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale())` at
`d2c_cem_training.rs:277` with `OBS_DIM = 2` and `ACT_DIM = 1` at
`:81-82`, giving `n_params = 3` via `linear.rs:61` formula
`act_dim * (obs_dim + 1)`. This is the load-bearing anchor for
Section 3.

The three design calls were briefed to the user after recon for
a leans-check, and the user explicitly delegated both the PPO
pool call and the seed-API call to the assistant. Section 1.2's
`&[u64]` pick and Section 2's PPO exclusion (pool = `{CEM, SA}`)
are assistant calls under the standing delegation.

## Factual pass

**Status:** Run via Explore agent. **38/38 claims verified.**
One minor line-range drift fixed before the thinking pass.

| # | Claim | Method | Result |
|---|-------|--------|--------|
| 1 | `competition.rs:290` `Competition::new(n_envs, budget, seed: u64)` const fn | Read | Verified |
| 2 | `competition.rs:321` `Competition::run` signature | Read | Verified |
| 3 | `competition.rs:325` `Result<CompetitionResult, EnvError>` return | Read | Verified |
| 4 | `competition.rs:328-329` nested `for task in tasks { for builder in builders }` loops | Read | Verified |
| 5 | `competition.rs:330` `let mut env = task.build_vec_env(self.n_envs)?;` | Read | Verified |
| 6 | `competition.rs:341` `algorithm.train(&mut env, self.budget, self.seed, ...)` | Read | Verified |
| 7 | `competition.rs:83-86` `CompetitionResult { runs: Vec<RunResult> }` | Read | Verified |
| 8 | `competition.rs:23-34` `RunResult` struct shape | Read | Verified |
| 9 | `competition.rs:43-50` `RunResult::best_reward()` method | Read | Verified |
| 10 | `competition.rs:66-76` `RunResult::assert_finite()` | Read | Verified |
| 11 | `competition.rs:91-95` `find(task, algorithm)` | Read | Verified |
| 12 | `competition.rs:99-101` `for_task(task)` | Read | Verified |
| 13 | `competition.rs:105-110` `for_algorithm(algorithm)` | Read | Verified |
| 14 | `competition.rs:520-538` existing test names | Read | Verified |
| 15 | `ppo.rs:41,43,45` `sigma_init/decay/min` fields | Read | Verified |
| 16 | `ppo.rs:81` `sigma: f64` field | Read | Verified |
| 17 | `ppo.rs:414` sigma decay line | Read | Verified |
| 18 | `reinforce.rs:34-38` sigma hyperparameter fields | Read | Verified |
| 19 | `reinforce.rs:69` `sigma: f64` field | Read | Verified |
| 20 | `reinforce.rs:261` sigma decay line | Read | Verified |
| 21 | `d2c_cem_training.rs:81-82` `OBS_DIM = 2, ACT_DIM = 1` | Read | Verified |
| 22 | `d2c_cem_training.rs:274-387` four tests, no `d2c_reinforce` | Read | Verified |
| 23 | `d2c_cem_training.rs:277` `LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale())` | Read | Verified |
| 24 | `d2c_cem_training.rs:278` `policy.set_params(&[0.0, 0.0, 2.0])` three values | Read | Verified |
| 25 | `linear.rs:55` `LinearPolicy::new` constructor | Read | Verified |
| 26 | `linear.rs:61` `n_params = act_dim * (obs_dim + 1)` formula | Read | Verified |
| 27 | `cem.rs:28-39` `CemHyperparams` fields | Read | Verified |
| 28 | `d1d_reinforce_comparison.rs:256-260` D1d finding quote | Read | Verified |
| 29 | `30:88-92` matched complexity passage | Read | Verified (exact wording) |
| 30 | `30:125-127` "drags baseline down" phrase | Read | Verified |
| 31 | `30:130-135` scientific question definition | Read | Verified |
| 32 | `30:164-167` null-case outcome passage | Read | Verified |
| 33 | `30:183-187` move-the-goalposts discipline | Read | Verified |
| 34 | `30:212-216` PPO-deferred-to-Ch-23 passage | Read | Verified |
| 35 | `20:85-89` Colas et al. "How Many Random Seeds?" citation | Read | Verified |
| 36 | `project_d2_sr_findings.md:11` D2c 4-algorithm framing | Read | Verified |
| 37 | Memory lines 16, 18 TD3/SAC kt_mult values | Read | Verified |
| 38 | Memory lines 17, 20, 22 PPO PASS* / false positive / D1d-style language | Read | Verified |

**Verdict:** Factual pass clean. All source-verifiable claims
accurate. One minor drift: the Section 2.1 block-quote citation
was `30:121-127` but the quote starts at line 122; fixed to
`30:122-127` before the thinking pass ran.

**Non-verified items (by kind, intentional):**
- Argument-level claims about "matched complexity," "geometric
  update rule," "expected value of including PPO" — these are
  Ch 23's own reasoning, not source-verifiable.
- Language-level Rust properties (`const fn`, trait object
  semantics, `Box<dyn Policy>` sizing) — standard Rust
  guarantees.
- The Ch 30 internal structure — verified by Ch 30's own factual
  pass; Ch 23's cross-references are verified by quoting passages.

## Thinking pass — Round 1

**Status:** Run via general-purpose agent against the 10-point
brief for argument chapters. **13 findings surfaced.** Verdict:
*"second round needed."*

### Findings applied

**Must-fix:**

1. **#2 — Section 2 / Section 3 internal-consistency tension.**
   The TD3/SAC exclusion argument (linear-$Q$ grounds) and the
   matched-complexity anchor (policy-level `LinearPolicy(2, 1)`)
   had an unacknowledged tension: Ch 30 describes
   matched-complexity as "what controls for the expressiveness
   failure," which on a careless read would undo the TD3/SAC
   exclusion. The fix: added an explicit paragraph in Section 2.1
   ("A subtlety that matters for Section 3") reconciling the two
   — TD3/SAC's linear-$Q$ failure lives in the *value-function
   approximator*, a component CEM and SA do not have at all, so
   policy-level matched-complexity structurally cannot reach it.
   The exclusion stands because the failure lives in a component
   the rule cannot match against. Anchored the paragraph to the
   specific D2c construction
   (`d2c_cem_training.rs:299-302` TD3 constructs `LinearQ`).

2. **#5 — Section 2.2 part (c) "either branch confounding"
   rhetorical foreclosure.** The original argument dismissed the
   PPO-washout-into-clean-pass case as "not answering a geometry
   question," which rhetorically parallelized two different
   outcomes. The fix: rewrote part (c) to enumerate three
   outcomes (persist / clean-pass / clean-fail), explicitly call
   the clean-pass case "genuinely informative," and argue the
   exclusion on a probability-weighted expected-value calculus
   rather than symmetric confounding. The conclusion (exclude
   PPO) is unchanged; the reasoning is now honest about the
   evidence state and does not rely on a too-clever-by-half
   symmetry.

**Should-fix:**

3. **#1 — three-part rule ownership.** Added explicit framing in
   Section 2.1 that the three-part rule is "Ch 23's own work,
   not a direct quote from Ch 30," distilled from Ch 30's
   informal two-clause argument. The chapter now owns the
   formalization rather than treating it as inherited.

4. **#3 — commit to `run` as wrapper over `run_replicates`.**
   Section 1.1 was hedging on whether Part 4 would ship the
   wrapper unification. Rewrote to commit: `run_replicates`
   carries the shared loop body, `run` is one line of forwarding,
   the two methods share a single implementation. Updated the
   chapter intro and the "what Ch 23 does not decide" section
   to match.

5. **#4 — `&[u64]` consistency argument demoted.** Section 1.2's
   three parts were reordered. Physics-seed-domain separation
   is now Part 1 (load-bearing). Closure-complexity is Part 2
   (secondary). The "Competition already uses raw u64
   consistency" argument is now Part 3 explicitly named as
   weaker — "genuinely weaker than Part 1," "not load-bearing
   on its own."

6. **#6 — D1d→D2c analogy softening.** Section 2.2 part (a)
   now explicitly calls the "D1d-style" label "an analogy that
   points back to a different task" and describes the D2 SR
   memo's characterization as "an inference from architectural
   similarity and D2c output behavior, not an independent
   empirical replication of the D1d diagnostic." The chapter
   commits to excluding PPO under this weaker evidential state
   rather than overclaiming the mechanism is closed.

7. **#7 — positive-case "any local random walk" limitation.**
   Section 2.4's "what it costs" paragraph was split. The
   positive-case paragraph now names the "joint bound on SA's
   specific mechanism *and* on the broader local-walk category"
   as a real limitation on interpretive scope — "stating it
   and accepting it, not handwaving it."

8. **#8 — pilot clause narrowed.** Section 3.4's pilot paragraph
   was rewritten to restrict pilot-triggered changes to
   objective structural criteria (acceptance-rate collapse at 0
   or 1) and explicitly rule out "SA's mean reward looks low,"
   "isn't converging fast enough," and "proposal schedule seems
   off" as triggers. The clause grounds the distinction in the
   reward-signal-in-disguise discipline.

9. **#9 — flat-vs-nested as ergonomics call, not principled.**
   Section 1.3 now opens with "neither shape is abstractly
   wrong" and closes by explicitly calling the pick "an
   ergonomics-and-refactor call rather than a principled one."
   Added a paragraph comparing flat vs nested under Ch 24's
   likely aggregation shape.

10. **#10 — three-way convergence is D2c-specific.** Section 3.2
    has a new paragraph explaining that parameter count,
    parameterization family, and representation width collapse
    onto one axis only under `LinearPolicy(2, 1)`, and that a
    future non-D2c rematch with a richer policy class inherits
    an open question Ch 23 is not answering. Example given:
    3-param linear policy vs 3-param 1-layer MLP with tied
    weights.

11. **#11 — null-case asymmetry named.** Section 2.4 has a new
    "what it costs in the null case" paragraph noting the
    exclusion's interpretive cost is actually largest in the
    null outcome, not the positive outcome, and
    acknowledging the chapter did not name this asymmetry in
    its initial pass.

**Minor:**

12. **#12 — recon-reported density.** Added a paragraph in the
    "what Chapter 23 does not decide" section naming the
    load-bearing Section 3 anchor numbers explicitly and
    assigning Part 4 the obligation to re-verify them at
    PR-write time with a hard-fail. The `(recon-reported)` tag
    is kept as a trail-of-verification marker with this
    explicit escalation for the numbers the matched-complexity
    rule is built on.

13. **#13 — sequential replicate wall-clock.** Section 1.4 has
    a new "wall-clock is a real concern" paragraph acknowledging
    that sequential replicates at the 16M-step rematch budget
    push into multi-hour-to-overnight territory, justifying why
    the sequential pick stands anyway (parallelism is out of
    scope for the API itself), and assigning external
    concurrency management to whoever runs the rematch.

### Findings not applied

None. All 13 findings were addressed in the revision. The two
must-fix findings triggered the second round of the thinking
pass.

## Thinking pass — Round 2

**Status:** Run via general-purpose agent against a tailored
brief (re-check each round-1 finding against the revised
chapter). **13/13 findings resolved.** No new issues introduced
by the revision. **Verdict: "ship."**

The round-2 reviewer's summary of the two must-fix resolutions:
the Q-vs-policy structural argument (finding #2) is coherent and
internally consistent with Section 3's rule; the three-outcome
enumeration (finding #5) lands honestly and strengthens the
chapter's falsifiability posture. The other 11 findings (8
should-fix + 3 minor including the two re-categorized as
load-bearing-not-noise) all land.

## Notes on confidence and remaining uncertainty

**Load-bearing claims that need Part 4 re-verification.** The
Section 3 anchor numbers — `OBS_DIM = 2`, `ACT_DIM = 1`,
`LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale())` at
`d2c_cem_training.rs:277`, the `n_params = act_dim * (obs_dim +
1)` formula at `linear.rs:61` — are the matched-complexity
rule's foundation. They are verified as of the factual pass, but
the chapter's "what Ch 23 does not decide" section explicitly
assigns Part 4's execution PR the obligation to re-verify them
at PR-write time with a hard-fail if they have drifted.

**Delegation standing for the PPO and seed-API calls.** Both
were explicitly delegated by the user before drafting ("you can
do all the recon you want to make the right call here" on PPO,
"I genuinely want the wiser option and I feel like you would
know better than I" on the seed API). The recon-to-commitment
cycle surfaced the REINFORCE-never-in-scope clarification and
the D2c-CEM anchor. Both calls are assistant calls under the
standing delegation, flagged in the commit message as such.

**Ch 24 cross-dependency.** Ch 23 leaves Ch 24 free to pick any
`best_reward` aggregation rule over the flat `Vec<RunResult>`
slice filtered by `(task, algorithm)`. Ch 24 will be drafted
next and inherits the return shape this chapter commits to. No
contortion expected.

**What Ch 23 does not resolve and names as Part 4 / Ch 32
follow-ups:**

- Specific number of replicates (Ch 32, depends on pilot variance).
- Statistical test the gate uses (Ch 24 + Ch 32).
- Exact `best_reward` aggregation rule (Ch 24).
- Exact `run_replicates` signature beyond `&[u64]` (Part 4).
- Pilot run's specific design (pre-rematch, out of Ch 23 scope).
- Whether a follow-up study investigates D1d-style inflation at
  higher compute budgets (out of Ch 23 scope).

## Gated review status

**Paused for user review before commit.** Per the Ch 01
protocol's list of gated chapters, Ch 23 (Competition v2 API
shape) is flagged for human review because the API becomes
load-bearing the moment the PR lands and any subsequent shape
change breaks every caller. The chapter has cleared both passes
plus a second round of the thinking pass on the revisions. The
user has seen the recon-to-leans report and the three picks the
chapter commits to; what needs the user's eyes on the final
draft is whether the revised argument structure (especially
Section 2.2(c)'s three-outcome PPO analysis and Section 2.1's
Q-vs-policy reconciliation) reads cleanly and whether anything
in the final chapter conflicts with the user's own prior.
