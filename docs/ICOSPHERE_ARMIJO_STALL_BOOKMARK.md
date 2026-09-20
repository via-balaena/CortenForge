# Synthetic-icosphere Armijo stall — bookmark

**Status**: surfaced 2026-09-19 by the final review pass on PR #950
(`insertion-reward-wiring`). ⛔ **PRE-EXISTING — not caused by that PR**;
proven below. Bookmarked rather than fixed there because it is a solver
pathology, not reward-wiring work.

**Where it belongs**: renovation item 3 (Tet10 + IPC in `insertion_sim`), per
`docs/INSERTION_SIM_TET10_RENOVATION_RECON.md` §8. That item changes the
contact formulation and the element order, which is the layer this stall lives
in.

## The failure

```text
cargo test -p cf-sim-research \
    sliding_insertion_ramp_converges_on_synthetic_icosphere -- --ignored
```

```text
panicked at tools/cf-sim-research/src/insertion_sim.rs:
  synthetic-sphere stall must be the Yeoh validity wall (closed-body overlap
  → high local stretch); got unexpected reason:
  Armijo line-search stalled at Newton iter 23, r_norm 6.143e-1
```

Accompanied by repeated `faer` LU fallbacks:
`Llt non-PD pivot: NonPositivePivot { index: 6206 / 6223 / 6224 }`.

## ★ The test is NOT broken — it is a working detector

The assertion is a deliberate safety net. Its own comment:

> *"Any non-validity-wall stall reason is a regression to the SL.3
> deep-interior mechanism (mechanism 1) the cutoff is designed to filter, OR an
> unexpected solver pathology — surface it."*

The observed reason is **neither** the validity wall nor the SL.3 mechanism.
⇒ The gate is correctly reporting a third thing, which is what it was built to
do. **Do not "fix" this by widening the assertion to accept Armijo stalls** —
that would delete the only detector pointing at this pathology.

## ⛔ Proof it is pre-existing

Run on a detached worktree at `origin/main` (`6dc7ee03`, before the PR branch)
versus the PR head (`23c3c869`):

| tree | result |
|---|---|
| `main` `6dc7ee03` | FAILED — `Armijo line-search stalled at Newton iter 23, r_norm 6.143e-1` |
| PR `23c3c869` | FAILED — `Armijo line-search stalled at Newton iter 23, r_norm 6.143e-1` |

**Identical message and identical residual to four significant figures.** The
only difference is the panic's line number (5719 → 6329), which is exactly the
lines PR #950 adds above it. Identical numerics mean that PR does not perturb
this code path.

⚠ Note the fixture re-implements its own inner loop and does not route through
`build_insertion_geometry` / `run_insertion_ramp`, so the reward wiring added in
#950 is not in its call path at all.

## Relationship to the sliding-intruder arc

`docs/SIM_ARC_SLIDING_INTRUDER_CONTACT_RECON.md` §5 CR.3 already anticipated
this fixture's framing being wrong:

> *"this fixture's docstring frames the step-4 stall as 'Yeoh validity wall +
> narrowing cross-section'; **the per-PR-review postmortem already flagged this
> framing as wishful** (the real root cause was the SAME SL.3 mechanism).
> Post-v5 fix, the expectation is that **all 16 steps converge cleanly**."*

Current state against that plan:

- **CR.1 landed** — `interior_cutoff` exists in `sim/L0/soft/src/contact/penalty.rs`.
- **CR.3's expectation did NOT materialise** — the steps do not all converge, and
  the stall reason is a third mechanism the recon did not predict.

⇒ The deep-interior cutoff fixed what it was aimed at, and something else is
still wrong.

⚠ **Path drift**: that recon references `tools/cf-device-design/src/insertion_sim.rs`
throughout. The crate is now `tools/cf-sim-research`. Line numbers in it are
stale; the file moved.

## What the evidence points at

The `Llt non-PD pivot` fallbacks say the condensed tangent is **not positive
definite near the solution**, and the Armijo line search then cannot find a
descent step. That is the same class the growing ramp shows on the cube fixture
(`Armijo line-search stalled at Newton iter 5 ... non-SPD tangent near
solution`), so it is unlikely to be specific to the icosphere.

Candidate causes, none investigated:

1. **Penalty contact's tangent.** `PenaltyRigidContact` can produce an
   indefinite contribution when pairs flip across the smoothing band. IPC's
   barrier Hessian is PSD by construction — which is one reason item 3 may
   dissolve this rather than fix it.
2. **Tet4 near-incompressibility.** `silicone_table` runs ν = 0.40 with no
   locking cure enabled (`config.fbar` is off everywhere in this path), and a
   locked element's tangent conditions badly.
3. **A genuinely non-SPD configuration** at high local stretch, in which case
   the formulation must change (the fixture's own docstring anticipates
   "Dirichlet hybrid, augmented Lagrangian, etc.").

## ⚠ Why this went unnoticed

`cargo test` skips `#[ignore]`d tests silently, and neither the repo's default
suite invocation nor the verification sweep used in PR #950 passed `--ignored`.
Thirteen ignored tests in `cf-sim-research` were never executed across a
branch that was otherwise verified repeatedly. **Any sweep that claims to
cover a crate should state whether it ran the ignored set.**

## Reproduction

```sh
# fails identically on main and on the PR branch
cargo test -p cf-sim-research \
    sliding_insertion_ramp_converges_on_synthetic_icosphere -- --ignored

# the full ignored set: 12 pass, this one fails
cargo test -p cf-sim-research -- --ignored
```
