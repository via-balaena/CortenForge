# Review log — Chapter 20: Single-seed is broken

## Factual pass

**Status:** Run. Two tracks: codebase claims (verified by direct
grep/read of current source) and external-literature claims
(verified by sub-agent with web access against the Henderson paper).

### Codebase claims

| Claim | Method | Result |
|---|---|---|
| `Competition::new(n_envs: usize, budget: TrainingBudget, seed: u64) -> Self` is the current constructor | Grep `pub fn new` in `competition.rs` | **Verified** at `competition.rs:290`. `const fn` variant. |
| `Competition` has a `seed: u64` field | Grep `pub struct Competition` / field listing | **Verified** at `competition.rs:279`. |
| `Competition::run` passes `self.seed` to `algorithm.train` | Read `run` body | **Verified** at `competition.rs:341`: `algorithm.train(&mut env, self.budget, self.seed, &|m| { ... })`. |
| A `new_verbose` variant with the same single-seed shape exists | Grep | **Verified** at `competition.rs:304`, same signature. |
| `CompetitionResult` is shaped as `runs: Vec<RunResult>` with per-run scalar `best_reward` | Read struct definitions | **Verified.** `pub struct CompetitionResult` at `competition.rs:83` with `pub runs: Vec<RunResult>` at `85`. `pub struct RunResult` at `23`, `pub fn best_reward(&self) -> Option<f64>` at `45`. The result shape has no slot for "this is run 3 of 5 for algorithm X." |
| `print_ranked` operates on scalar `best_reward` values | Read body | **Verified** at `competition.rs:153`. Body uses `r.best_reward().unwrap_or(f64::NAN)` at `161` to build the ranked table — a scalar-per-algorithm sort. |
| The D2c rematch test uses `const SEED: u64 = 20_260_412` and `Competition::new_verbose(N_ENVS, TrainingBudget::Epochs(N_EPOCHS), SEED)` | Read spec text | **Verified** at `docs/thermo_computing/01_vision/physics_aware_ml_construction.md:1654` and `:1660`. |
| The rematch gate is the scalar comparison `sa_best >= best_rl` where `best_rl = cem_best.max(ppo_best).max(td3_best)` | Read spec text | **Verified** at `physics_aware_ml_construction.md:1685` and `:1698-1699`. No variance, no CI, no significance test. |

All codebase claims verified. No discrepancies.

### Henderson et al. 2018 — external-literature verification

**Method:** Sub-agent with `WebSearch`/`WebFetch` given the chapter
text and a 4-point verification brief. The sub-agent consulted
arXiv, the AAAI proceedings, dblp, and the ar5iv HTML render of the
paper.

| Claim | Result |
|---|---|
| Authors, title, venue, year | **Verified.** Henderson, Islam, Bachman, Pineau, Precup, Meger; *Deep Reinforcement Learning that Matters*; Thirty-Second AAAI Conference on Artificial Intelligence (AAAI-18), pp. 3207–3214, 2018. DOI 10.1609/aaai.v32i1.11694. arXiv 1709.06560. Chapter's author order matches dblp exactly. |
| "Partition ten seeds of the same algorithm into two groups of five and see learning curves that look like they come from different distributions" | **Verified.** The paper's "Random Seeds and Trials" subsection describes exactly this procedure: "We perform 10 experiment trials, for the same hyperparameter configuration, only varying the random seed across all 10 trials. We then split the trials into two sets of 5 and average these two groupings together." Summary line from the paper: "the variance between runs is enough to create statistically different distributions just from varying random seeds." |
| "The paper suggests five or more" seeds | **OVERSTATED in draft.** The paper explicitly declines to name a fixed number: "there can be no specific number of trials specified as a recommendation... power analysis methods can be used to give a general idea." The five comes from their own demonstration splits, not from a recommendation. Discrepancy flagged and fixed — see below. |
| The paper recommends confidence intervals + significance testing | **Verified.** The paper discusses bootstrap CIs, two-sample t-tests, and Kolmogorov–Smirnov; core recommendation is "proper significance testing to determine if the higher average returns are in fact representative of better performance." |

### Fixes applied

1. **Required:** Removed "the paper suggests five or more" and
   replaced with an accurate characterization — Henderson declines
   to name a number and points at power analysis; added a pointer
   to Colas, Sigaud, Oudeyer (arXiv 1806.08295) as the follow-up
   that actually computes concrete seed counts. This was a real
   factual error and the fix is material.
2. **Minor tightening:** "by margins comparable to the
   between-algorithm margins people report in papers" → "produce
   learning curves that look indistinguishable from the
   between-algorithm gaps people report in papers." The original
   phrasing implied numeric margin comparison; the paper shows
   distribution-level separation, which is the right flavor but
   not literally a margin comparison. Also inlined the paper's
   own "statistically different distributions just from varying
   random seeds" quote to anchor the claim.
3. **Minor:** Added arXiv ID alongside the venue in the first
   mention of the paper, to make the citation easy to follow up
   on.

No second round was triggered by the codebase claims — those were
all clean on the first pass.

## Thinking pass

**Status:** Author self-review. No cold-reader sub-agent run this
round; the chapter's argument is well-trodden in the literature
(single-seed RL comparisons are unsound) and the substantive risk
is factual (does the paper actually say what I'm claiming), which
the factual pass covered. If the user flags concerns in their own
read, a cold-reader pass is easy to run in round 2.

**Self-review notes:**

1. **"There is a published, well-known version of this argument."**
   This line is load-bearing — it tells the reader they are not
   being asked to take my word for a novel claim. I want to make
   sure it's not doing too much work. The literature consensus on
   single-seed comparisons being unsafe is strong and has been for
   at least six years; I don't think I'm overclaiming by calling
   it well-known. Keep.
2. **The "Why the fix is architectural" section.** My argument
   there hinges on "you can't just wrap `Competition::run` in a
   loop in the test file." A skeptic could say: but actually, yes
   you can — run the competition five times, collect five
   `CompetitionResult`s, write a small helper in the test that
   aggregates them, run the gate on the aggregate. That works for
   exactly one test. It does not work for the general case, for
   which the result type has to nest per-seed runs, the ranking
   function has to understand replicates, and the per-algorithm
   `best_reward` semantics have to be re-examined (chapter 24's
   problem). I think the chapter's framing is correct for a
   chassis-level fix but the skeptical reading is worth
   acknowledging if a cold reader surfaces it. Second-round
   candidate.
3. **Chapter 24 forward-reference.** I gesture at chapter 24's
   finding that `best_reward` means subtly different things across
   algorithms, without restating the finding here. This is
   deliberate — chapter 24 is where the argument is made properly,
   and chapter 20's job is to say "there's a further wrinkle."
   Reader who wants detail follows the reference. If a reader
   reads chapter 20 in isolation they may feel the reference is
   unsupported; I think this is acceptable for a book that is
   read in order.
4. **The D2 SR findings reference.** I cite the memory memo
   "`project_d2_sr_findings.md`" directly. That memo exists in the
   user's memory system, not in the committed repo, and a future
   reader browsing the book on disk won't be able to click
   through. On reflection I should either (a) inline the relevant
   finding from the memo, or (b) scope the reference to "the prior
   D2c run's result." I went with option (b) implicitly — the
   text says "the D2 SR findings memo records the prior result"
   which is factually accurate. If the user wants the specific
   finding inlined, easy second-round fix.
5. **Length and density.** ~1200 words. The chapter is shorter
   than chapter 10 despite covering one of the three headline
   findings. Justified because the underlying argument (Henderson)
   is pre-published and doesn't need to be re-derived; this
   chapter is doing the mapping from the published argument onto
   our specific codebase, which is a narrower job.

**Unstated assumptions I am aware of:**

- That the reader accepts the RL literature's standard on this.
  If they don't, nothing in this chapter will convince them —
  they'd need to read Henderson and the follow-ups directly. The
  chapter does not try to re-derive the argument from first
  principles the way chapter 10 does for physics.
- That "multi-seed with significance test" is the right standard
  for *our* experiments specifically, not just for generic RL
  papers. A skeptic could argue that CortenForge's experiments are
  unusual in ways that change the answer — more stochastic, more
  domain-specific, not trying to claim SOTA. I don't think this
  objection survives contact with the rematch specifically (the
  rematch is exactly the kind of cross-algorithm benchmark
  Henderson is about) but it's an objection I have not engaged
  with in-chapter.

**Alternatives considered and left out:**

- Running the rematch single-seed with an explicit caveat "we know
  this is underpowered, treat the result as directional." This is
  a defensible choice for a *first* experiment but the chapter's
  job is to point at the architectural fix, not to defend a
  compromise protocol. Left out.

## Second round

**Triggered:** Partially. The Henderson citation's "five or more"
claim was a factual error requiring a substantive fix (not just a
typo or line-number drift), and the fix was applied in the revision
above. The rest of the factual pass was clean, and the thinking
pass did not surface argument-level problems that require
restructuring. No further rounds pending unless user review finds
new issues.

## Open questions carried forward

- Should the D2 SR findings memo's specific finding be inlined, or
  is the current indirect reference enough? Defer to user.
- Does the "chapter 24 forward-reference" feel unsupported when
  read in isolation? Will be testable once chapter 24 is written
  in Phase 2.

## Status

Drafted, factual pass complete, self-reviewed, one substantive
factual fix applied in a second round. Not yet committed.
