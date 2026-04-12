# Review log — Chapter 10: The wrong struct

## Factual pass

**Status:** Run. Verified against current source in
`sim/L0/thermostat/src/` and `sim/L0/core/src/batch.rs`.

The chapter is primarily a physics-first-principles argument and
makes only a small number of concrete claims about the codebase.
Each one was grep-verified:

| Claim | Method | Result |
|---|---|---|
| `LangevinThermostat` is the only passive component in `sim/L0/thermostat/src/` that holds RNG state | `grep -n 'rng\|ChaCha\|Rng'` across the directory | **Verified.** Only three files match: `langevin.rs` (the RNG-owning component), `component.rs` (line 99 is a docstring comment mentioning `Mutex<ChaCha8Rng>` as architectural rationale, no actual state), and `gibbs.rs` (standalone sampler, not a `PassiveComponent` — confirmed: no `impl PassiveComponent for GibbsSampler` matches). |
| `DoubleWellPotential`, `OscillatingField`, `RatchetPotential`, `PairwiseCoupling`, `ExternalField` are all pure (no RNG, no `rand`) | `grep 'rng\|ChaCha\|SmallRng\|StdRng\|rand::'` in each file | **Verified.** All five files return zero matches. |
| Each of those five components `impl PassiveComponent` | `grep 'impl PassiveComponent for'` across `thermostat/src/` | **Verified.** Five `impl PassiveComponent for` lines, one per component, at `double_well.rs:142`, `oscillating_field.rs:139`, `ratchet.rs:142`, `pairwise_coupling.rs:167`, `external_field.rs:61`, plus `langevin.rs:133` for Langevin and three dummy test impls under `stack.rs` / `component.rs`. |
| `LangevinThermostat` wraps its RNG in a mutex | Read `langevin.rs:72-107` | **Verified.** Struct field at `langevin.rs:76`: `rng: Mutex<ChaCha8Rng>`. Constructed at `langevin.rs:107`: `rng: Mutex::new(ChaCha8Rng::seed_from_u64(seed))`. `self.rng.lock()` appears at `langevin.rs:184` inside the `apply` implementation. |
| `BatchSim` shares a single `Arc<Model>` across all parallel environments | `grep 'Arc<Model>\|pub struct BatchSim'` across `sim/L0/` | **Verified.** `sim/L0/core/src/batch.rs:61` doc comment: "All environments share the same `Arc<Model>`." Struct field at `batch.rs:65`: `model: Arc<Model>`. Same pattern in `ml-bridge/src/vec_env.rs:77` (the `VecEnv` wrapper). |
| The shared `Arc<Model>` captures `Arc<dyn PassiveComponent>` clones into its `cb_passive` closure | Read `stack.rs:89-136` | **Verified.** Doc comment at `stack.rs:92`: "the `cb_passive` callback closure captures a clone of the `Arc`." `PassiveStack::install(self: &Arc<Self>, model: &mut Model)` signature at `stack.rs:120`. `components: Vec<Arc<dyn PassiveComponent>>` at `stack.rs:101`. |

**No discrepancies.** All claims that the chapter makes about the
current code match the current source.

**Non-verified items (intentional, by kind):**

- The Langevin-equation form and its properties — standard physics,
  not a claim about CortenForge code. Cited to textbook treatment
  implicitly; no source-citation needed.
- The "not a subtle point" assertion about ensemble averages under
  correlated noise — a physics-methodology claim, not a code claim.

## Thinking pass

**Status:** Author self-review, cold-reader pass pending before
commit.

**Self-review notes:**

1. **Physics first vs. code first.** I ordered the chapter physics →
   code → implication deliberately. The alternative order (start
   with the bug, then justify it with physics) reads more like a bug
   report and undersells the "this is a mismodeling, not an
   implementation bug" framing. The current order is the right one
   for a study chapter; it would be the wrong order for a PR
   description. Keep.
2. **"Order is not independence."** This is the chapter's sharpest
   claim and I want to make sure it holds under a skeptical reading.
   The argument: two envs pulling sequentially from a shared
   `ChaCha8Rng` get draws that are deterministically related to each
   other (env 2's draws are whatever the stream produces after env
   1's consumption). That is not the same as "each env draws from
   its own independent stream." A more charitable reading of the
   existing code would be: "if we treat the single stream as the
   ground truth and the envs as two strides through it, the union is
   still an unbiased sample of Gaussian white noise." That reading
   is *numerically* defensible for a single step but breaks down
   over multi-step trajectories because the *correlation structure*
   between the two paths is no longer zero — they share a state
   history. I considered adding this paragraph to the chapter and
   decided it was a distraction from the core argument: even if the
   marginals are fine, the joint distribution is wrong, and the
   joint distribution is what ensemble statistics are computed over.
   Leaving it as a footnote candidate for a second round if a
   reviewer surfaces the objection.
3. **"Bit-identical reproducibility ... is no longer attainable."**
   Load-bearing claim. True under rayon's scheduler for
   `par_iter_mut`, which is the current parallelism path (recon
   session B, unverified — but the claim is also true for *any*
   scheduler whose order is not itself seeded, which is the general
   case). I am comfortable stating it unconditionally. If the
   parallelism layer turns out to run envs in strict insertion order
   under some feature flag, the claim needs a "under typical
   configuration" qualifier. Flagging as a second-round candidate
   once chapter 11 (BatchSim parallelism) does the full recon
   verification.
4. **Smallness of the defect.** I used the word "good news" for
   "only one component is wrong." On a cold read this might sound
   glib for a book whose thesis is that the defect is serious. I
   left it because the immediate next sentence is "bad news for the
   history of the code" and the contrast is the point: a one-file
   defect feels patchable, which is exactly why it was not patched,
   which is why we are writing a chapter. If a cold reader thinks
   the glibness undermines the seriousness, this is an easy rewrite.
5. **What it does not decide.** The closing section points
   explicitly at chapters 14/15 for the design fork (RNG-on-Data vs
   per-env installer). This is deliberate: chapter 10 makes the
   physics argument; the engineering argument is a separate act of
   thinking and does not belong here. Open question: is this
   division clean enough that a reader of chapter 10 alone walks
   away with the right take, or do they need chapter 14 to feel
   complete? I think 10 stands alone — the requirement is stated,
   the fix is deferred. If the user reads it as a cliffhanger, the
   fix is to add one more sentence near the end.

**Unstated assumptions I am aware of:**

- That the reader accepts "per-trajectory noise" as a physical
  definition rather than an implementation preference. A determined
  skeptic could argue that for some observables you can recover
  correct statistics from correlated-noise simulations with
  careful bookkeeping. I do not engage with this because the
  carefully-bookkept case does not describe any of the experiments
  we actually run, and the correct thing to do is take the
  physically clean path.
- That the fix is worth the architectural upheaval. Chapter 00 made
  this argument ("years of physics-aware ML research on top"); I
  don't re-litigate it here.

**Alternatives considered and left out:**

- A discussion of "what if we just serialize all noise draws across
  envs, in a fixed order, to get reproducibility without moving
  state?" This is the naïve fix-the-order patch. I left it out
  because (a) it does not solve the physics problem, only the
  reproducibility problem, and chapter 10 is about the physics; (b)
  it *does* deserve a paragraph in chapter 14 as an option that was
  considered and rejected.

## Cold-reader thinking pass (sub-agent)

**Run:** A general-purpose sub-agent with no prior context reviewed
the chapter against a 7-point brief (unstated assumptions, missed
alternatives, earned emphatics, does-the-conclusion-follow, simpler
explanation, falsifiability, tone/voice). Full report preserved in
the originating session; the substantive findings are captured
below.

**Substantive findings:**

1. **Counter-based / splittable RNGs were the most conspicuous gap.**
   The cold reader flagged that generators in the Philox / Threefry /
   PCG-with-streams family let a single `Model`-resident generator
   hand out provably-independent substreams keyed by trajectory index
   without any per-trajectory mutable state. The original chapter
   lumped these into "per-thread RNGs... thread-scheduling leaks into
   the results," which is not what a counter-based substream does.
   The chapter needed to name and dispose of this alternative before
   concluding anything about where state lives.
2. **The conclusion over-reached.** The chapter's closing line — "the
   RNG has to live with the trajectory because that is what a
   trajectory is" — was stronger than the physics licensed. The
   physics licenses "the *noise* has to be per-trajectory
   independent"; the jump from "noise per-trajectory independent" to
   "RNG struct on the per-trajectory data object" is an engineering
   claim wearing physics clothes. Counter-based RNGs are the
   falsifying counter-example.
3. **The pre-generated tape was incorrectly dismissed.** The original
   wording called it an "approximation" that gives up "some property
   of the definition." That is not true for a correctly-sized disjoint
   tape, which is exactly correct.
4. **Discretization was unstated.** The chapter presented the
   continuous-SDE form of the Langevin equation as "the definition,"
   without noting that the actual object a simulation operates on is
   a discretized scheme (Euler–Maruyama, BAOAB, etc.) whose noise is
   a finite sequence of Gaussian draws. The physics argument is really
   about the statistical properties of that finite sequence.

**Lower-severity findings (noted, not acted on in-chapter):**

- "'Independent realization' is not a polite phrase" flagged as the
  voice's weakest sentence. Softened in the revision.
- "This is not a subtle point" and "Order is not independence"
  declared earned — kept.
- The Langevin-equation preamble could be trimmed but was defensible
  and load-bearing for the rest of Part 1. Kept.

## Second round

**Triggered:** **Yes.** Findings 1 and 2 above are substantive — an
alternative the chapter dismissed in effectively one sentence is in
fact a live option, and the chapter's conclusion was therefore
overstated. Finding 3 was a factual error of characterization.

**Revisions applied:**

- Opening restated: the physics claim is now "noise per-trajectory
  independent," not "RNG struct per-trajectory." Added an explicit
  paragraph deferring the engineering question to chapters 14/15.
- Added a discretization note to the "What a Langevin trajectory is"
  section (finding 4).
- Replaced the old "Where state has to live" section with a new
  "What faithful implementations look like" section that enumerates
  three valid mechanisms (per-trajectory stateful RNG, counter-based
  keyed by `(trajectory_id, step_index)`, pre-generated tape) and
  explicitly disposes of each on its merits rather than dismissing.
- Added a new "Why the current code is none of the above" section
  that locates the bug precisely: the current implementation is
  shape 1 in everything except the detail that matters — there is
  one generator behind a mutex, not N.
- Rewrote the closing. Old closing was "the RNG has to live with the
  trajectory because that is what a trajectory is." New closing is
  "the noise driving each trajectory must be that trajectory's own,
  and the current code does not supply it" — which is what the
  physics actually licenses.
- Softened "not a polite phrase" language in the independence
  paragraph. Added the marginals-vs-joint-distribution distinction
  directly inline, which also resolves self-review note 2.

The revisions are structural enough that a round 3 is in scope if
the next cold read finds anything; I do not think it will, because
the chapter now makes a narrower and better-supported claim.

## Open questions carried forward

- "Bit-identical reproducibility" claim conditionality — revisit
  after chapter 11's parallelism recon verification. Flagged but
  not currently load-bearing.
- Whether chapters 14/15 ultimately recommend shape 1 (stateful
  per-trajectory) or shape 2 (counter-based) is now a live design
  question, not a foregone conclusion. Memory's "Design forks"
  section lists RNG-on-Data vs install_per_env as the known fork;
  counter-based is a third option that should be explicitly
  considered in chapter 15 and not dismissed by omission.

## Status

Drafted, factual pass complete, cold-reader thinking pass complete,
second-round revisions applied. Not yet committed.
