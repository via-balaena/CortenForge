# Review log — Chapter 30: The scientific question

## Factual pass

**Status:** Run. Verified against committed source (the d2c test
and the pivot spec) and, for findings that no longer have a
committed doc home, against the D2 SR findings memory memo with
explicit attribution.

### Claims verified against committed source

| Claim | Method | Result |
|---|---|---|
| D2c ran with 100 epochs, 5000-step episodes, 32 parallel environments | Read `sim/L0/thermostat/tests/d2c_cem_training.rs:72-78` | **Verified.** `SUB_STEPS = 100`, `EPISODE_STEPS = 5_000`, `N_ENVS = 32`, `N_EPOCHS = 100`. |
| D2c trained with linear function approximation (LinearPolicy / LinearQ / LinearValue / LinearStochasticPolicy) | Cross-referenced via pivot spec's evidence table | **Verified** via `docs/thermo_computing/01_vision/physics_aware_ml_pivot.md:110-124`. The committed pivot spec asserts linear function approximation on all four algorithms. |
| D2c per-algorithm eval kT results: CEM 0.99 (FAIL), TD3 −0.31 (FAIL), PPO 0.005 (PASS\*, false positive), SAC −0.78 (FAIL) | Read pivot spec's evidence table | **Verified** at `physics_aware_ml_pivot.md:116-124`. Table in the committed pivot spec matches the chapter's numbers exactly. |
| "D1d-style exploration-noise inflation" as the PPO failure mechanism | Cross-ref pivot spec | **Verified.** Pivot spec line 120: "PPO \| 0.005 \| PASS\* \| Exploration-noise inflation (not SR)". Footnote at 123–124 says "False positive — synchrony came from transient exploration noise, not from discovered resonance." |
| The rematch test spec excludes SAC "because its D2c result (Gate A FAIL, `kT=-0.78`) has no reasonable way to be a 'baseline'" | Read `physics_aware_ml_construction.md:1633-1635` | **Verified** verbatim in the committed construction spec. |
| The `kt_mult` convention — the optimizer controls a multiplier on a base kT, not an absolute kT | Read `d2c_cem_training.rs:92` doc comment ("The action is the kT multiplier written to `data.ctrl[0]`") and `d2b_stochastic_resonance_baselines.rs:241` ("30 kT multipliers in [0.1, 5.0]") | **Verified.** The action is a multiplier, not an absolute kT. |
| The d2b temperature sweep uses 30 log-spaced `kt_mult` values in [0.1, 5.0] with 20 episodes each | Read `d2b_stochastic_resonance_baselines.rs:239-245` | **Verified.** Doc comment at 241 states the exact convention. |
| The d2b test computes the SR peak empirically (the peak values are not hard-coded) | Read `d2b_stochastic_resonance_baselines.rs:269-279` | **Verified.** Peak is found by `max_by` over `sweep_means`; the numeric peak is printed, not asserted. |

### Claims traceable to memory memo only

The specific peak characterization — peak synchrony $0.098 \pm
0.022$ at `kt_mult` $\approx 2.55$ and elevated-band `kt_mult` $\in
[1.1, 2.5]$ — comes from the D2 SR findings memo (user memory file
`project_d2_sr_findings.md`), not from any committed doc. Context:

- The original home for these numbers was
  `docs/thermo_computing/03_phases/d2_stochastic_resonance.md` §11,
  referenced in the memo. That phase doc was deleted in the repo
  audit (2026-04-11). The memo is now the authoritative record.
- The d2b test is the *source* of the numbers — running it under
  `--release` reproduces them. This session did not rerun the test;
  doing so would be a 30-second release run that could be added
  later if the numbers become load-bearing.
- The chapter cites the memo explicitly ("the D2 SR findings memo
  records peak synchrony...") rather than smuggling the numbers in
  as if they were from a committed doc. This is the honest
  attribution given that the memo is a real historical record that
  predates the repo audit.
- The numbers are scene-setting — the chapter's argument does not
  depend on their exact values, only on the qualitative shape
  ("broad band, narrow peak with modest absolute magnitude"). If
  the next session reruns the test and the numbers shift slightly,
  the chapter's claims still hold and the specific values get
  updated.

### Fixes applied

- **kT vs kt_mult.** Original draft used "$k_BT \approx 2.55$",
  "$k_BT \in [1.1, 2.5]$", etc., as if these were absolute
  temperatures. The test's `kt_mult` is a *multiplier* on a base
  $k_BT$ — the action space is dimensionless, not temperature.
  Chapter updated throughout to use `kt_mult` language, matching
  both `d2c_cem_training.rs:92` and `d2b_stochastic_resonance_baselines.rs:241`.
- **Attribution.** Added explicit memo attribution for the peak
  numbers and for the D2c eval-kT results, rather than presenting
  them as unattributed background facts.
- **SAC exclusion.** Added that the construction spec excludes SAC;
  also noted that including TD3 in the rematch is a revisit
  candidate for chapter 23 (TD3 failed for the same linear-$Q$
  reason SAC did, so including it may replay the same failure).

No other discrepancies found.

## Thinking pass

**Status:** Author self-review. No cold-reader sub-agent run this
chapter — the argument here is narrative/framing rather than
mechanistic, and the factual pass did the high-risk verification
work. If the user surfaces concerns, a cold read is a one-prompt
fix in round 2.

**Self-review notes:**

1. **Three-outcomes framing.** The "positive / null / ambiguous"
   structure is deliberate: a careful experiment is one whose
   outcomes are all informative, and I want to establish before
   the chassis work lands that we have thought about what a null
   result would teach us. A chapter that only explained the
   positive-case interpretation would be a hype piece. I think the
   section lands; it is the most important one in the chapter. If
   a reader pushes back it will probably be on the ambiguous
   case's "signal to run the next one differently" — which could
   sound like a "heads I win, tails we learn" move. I addressed
   that directly by pointing to the chassis work as what makes
   "run differently" cheap, not by waving it away.
2. **The narrowness of "physics-aware beats generic RL."** I
   argued explicitly that the rematch is a narrow test, not a
   proof of the general principle. This matters because (a) it
   prevents the chapter from overclaiming if the rematch wins,
   and (b) it frames a null result as evidence about one task
   rather than about the whole pivot. The user's standing feedback
   on "genuine agreement, not passive" and the thermo-RL loop
   north star both point at "do not water down the strategic
   claim, but also do not oversell one data point," and the
   current framing tries to thread that needle. Flagged as a
   paragraph to re-read if the user thinks I've over- or
   under-sold.
3. **The TD3-exclusion question.** I mentioned that TD3 may need
   to be excluded from the rematch pool for the same reason SAC
   was, and flagged this as a chapter 23 revisit. The construction
   spec currently includes TD3. I considered making a stronger
   claim here ("TD3 should be excluded") and decided it was out
   of scope — chapter 30 is about what the question is, not what
   the protocol is. The observation is worth preserving but the
   call belongs in 23.
4. **"The chapter does not specify hyperparameters, budget, or
   statistical gates."** I state this upfront and at the end. The
   repetition is deliberate; the chapter is being read against a
   backdrop where the reader might reasonably expect protocol
   specifics, and not finding them could feel like a gap unless
   they are explicitly told in both directions that the gap is
   on purpose.
5. **The "two observations" break in the D2c section.** I isolated
   "geometry problem (CEM)" from "expressiveness problem (TD3,
   SAC)" as two distinct failure modes. This is load-bearing for
   the rematch framing: the rematch is testing the first problem,
   not the second. If a reader blurs them, the pivot claim loses
   its teeth (because "physics-aware beats linear RL" becomes
   confounded with "any method with enough expressiveness beats
   linear RL"). The paragraph is short but I want to make sure it
   lands. If the user finds it compressed, I can expand it.

**Unstated assumptions I am aware of:**

- That the SR landscape description (broad band, narrow peak) is
  accurate. The numbers come from the d2b test run recorded in the
  memo. If a rerun shifts them substantially, the *shape* claim
  ("broad and flat") may still hold but the specific characterization
  would need updating.
- That "matched-complexity" is a clean concept. SA with a linear
  parameterization of `kt_mult` vs CEM with a linear policy is a
  reasonable match in terms of parameter count and representation
  expressiveness, but the comparison is not exact. Chapter 23 will
  have to be careful about what "matched complexity" means
  operationally.
- That a single SR task is a meaningful test bed at all. A
  determined skeptic could argue that SR is a cherry-picked
  example where physics-aware happens to be well-fit, and that
  any custom optimizer tailored to a landscape will beat generic
  RL on that landscape by construction. The chapter does not
  engage with this. I think the answer is "yes, and that's the
  point — custom tools for custom problems is the thesis, and
  the rematch is testing whether the specific custom tool we
  built actually works" — but this is an argument worth having
  in Part 3 (chapter 32 or 33, TBD), not here.

**Alternatives considered and left out:**

- A discussion of what Parallel Tempering would look like in the
  rematch. PT is listed in the pivot spec's algorithm family and
  is a natural next step if SA is ambiguous or null. Left out
  because (a) the rematch as currently specced is SA-only, (b)
  adding PT to the rematch changes the protocol substantively
  and is a chapter 23 question, and (c) this chapter is about
  the question the rematch is asking, not the menu of methods
  that could be asked to answer it.
- A numeric treatment of "how much better would SA have to be
  for the result to be convincing." That is exactly the chapter
  24 / 23 question about gates and variance. Deferred.

## Second round

**Triggered:** No. The factual pass found a real semantic issue
(kT vs kt_mult) and it was fixed in a same-round revision rather
than requiring a structural rethink. The thinking pass notes are
self-review observations, not argument-level problems. No further
rounds pending unless user review surfaces something new.

## Open questions carried forward

- The TD3-in-rematch-pool question (note 3). Feeds chapter 23.
- "Matched complexity" as an operational definition. Feeds chapter 23.
- Whether to defend SR's status as a test bed at all. Feeds Part 3
  (chapter 32/33, TBD).
- If the next session reruns `d2b_stochastic_resonance_baselines`
  under release, update the peak numbers here if they drift.

## Status

Drafted, factual pass complete, self-reviewed, kT/kt_mult semantic
fix applied. Not yet committed.
