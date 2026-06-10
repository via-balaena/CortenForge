# Keystone S2 — differentiable coupled step — RECON

*Active recon, opened 2026-06-10. The keystone's research leaf: make the merged forward
soft↔rigid coupling (`sim-coupling`, PR #299) *differentiable* — a gradient that crosses both
engines. `MISSION.md` calls this "the genuinely open research problem." Sibling of `recon.md`.*

> **Establish a gradient of a coupled-step outcome (a rigid-side scalar) with respect to a
> parameter (a soft material/contact param, or the rigid state), validated against finite
> differences. First as a black-box FD oracle (proves the coupled step is differentiable + gives
> the ground truth), then by assembling the analytic factor derivatives against that oracle.**

---

## 1. The chain (from the two-surface investigation)

One coupled step is `s' = f_rigid(s, xfrc = −force_on_soft(plane_pose(s), soft_state(s)))`. The
factor derivatives, and what exists today:

| Factor | Meaning | Today |
|---|---|---|
| `∂s'/∂s` (`A`) | rigid next-state vs rigid state | **available** — `Data::transition_derivatives → TransitionMatrices.A` (dense forward Jacobian, tangent space `2nv+na`) |
| `∂plane_pose/∂s` | contact plane height vs rigid state | **trivial** — `height = xpos[body].z − clearance`, `∂height/∂z = 1` |
| `∂force_on_soft/∂plane_pose` | contact force vs plane height | **MISSING** — force is read off-tape from primal positions; plane offset is never a tape `Var`. Analytic value is `−κ·n` summed over active pairs (derivable, unwritten) |
| `∂s'/∂xfrc` | rigid next-state vs applied Cartesian force | **MISSING** — `B` is `∂x'/∂ctrl` only; `xfrc_applied` is not in `B` and is auto-zeroed each `step`. Needs FD over `xfrc` or a new column-builder |

Plus the soft-state-mediated path: the soft tape gives `∂x*/∂θ` (load) only — **not**
`∂x*/∂(plane pose)` or `∂x*/∂(material)`. So the soft solve's contribution to the coupling
gradient is also unexposed (same root cause).

**The obstacle (precise):** the two engines do not share a differentiation substrate, and the two
interface couplings are exactly the two missing derivatives. The soft `Tape` can't see the rigid
step; the rigid FD Jacobians can't see `xfrc`. Bridging them is the research work.

## 2. Thesis

Forward correctness is done (PR #299). Differentiate the same partitioned step the *same way the
rigid engine already validates its own derivatives* — against a finite-difference oracle
(`derivatives::validate_analytical_vs_fd` is the in-repo precedent). So: (1) treat one coupled
step as a black box and **FD it** — this needs no new code, captures both missing factors at once,
proves the coupled step is differentiable in the smooth (contact-engaged) regime, and yields the
ground-truth gradient; (2) then **assemble the analytic gradient** factor-by-factor (`A` ×
`∂plane/∂s` + analytic `∂force/∂height` + FD-or-analytic `∂s'/∂xfrc`), each gated against the FD
oracle; (3) finally **chain it into the soft autograd tape** as a `VjpOp` so one `tape.backward`
crosses both engines. Penalty-contact non-smoothness caps gradient quality at the active-set
boundary (R3 from `recon.md`); we differentiate in the engaged regime and document the cap (IPC
later).

## 3. End-state (S2 v1)

A finite-difference-validated cross-engine gradient: a committed test computes
`∂(coupled-step outcome)/∂(parameter)` analytically (assembled from the factor derivatives) and
agrees with central finite differences to tolerance, in the contact-engaged (smooth) regime. The
parameter is a co-design-relevant one (a soft material/contact stiffness, and/or the rigid state).
The non-smooth-active-set limitation is documented; the full soft-tape `VjpOp` crossing and
material-parameter autograd are the leaves beyond v1.

## 4. Decisions (head-engineer)

- **D1 — FD oracle first, analytic second, tape third.** Mirrors the rigid engine's own
  `validate_analytical_vs_fd` discipline. The FD oracle is the spec the analytic assembly must meet.
- **D2 — Differentiate in the contact-ENGAGED regime.** The penalty active-set flip is non-smooth
  (R3); evaluate gradients where contact is established (the smooth regime) and document the
  boundary cap. IPC (deferred) is the path to clean active-set gradients.
- **D3 — First gradient = a soft-parameter → rigid-outcome design gradient.** E.g.
  `∂(settled rigid height)/∂(soft μ)`: a stiffer soft body penetrates less, so the rigid body
  settles higher — a clean, co-design-relevant, smooth dependence. (The settled contact *force* is
  μ-invariant — it equals the weight by force balance — a nice consistency check; the *height* is
  the μ-sensitive output.) FD via the public `StaggeredCoupling` API, zero new code.
- **D4 — Add the analytic factors incrementally, each FD-gated.** Analytic `∂force/∂height =
  −κ·n` over active pairs (a small new contact routine); FD `∂s'/∂xfrc` (a column-builder over the
  rigid step); `A` from `transition_derivatives`. Assemble + check vs the FD oracle.
- **D5 — Soft-tape `VjpOp` crossing is the last leaf.** Wrapping the rigid Jacobian as a chassis
  `VjpOp` so `tape.backward` flows across both engines is the deepest piece; earn it after the
  analytic assembly matches FD.

## 5. Sub-leaf ladder

- **S0 — FD differentiability spike (THROWAWAY, `#[ignore]`, uncommitted).** The #1-risk de-risk.
  Via the public `StaggeredCoupling` API, FD `∂(settled rigid height)/∂(soft μ)` (and the
  μ-invariance of the settled force): build couplings at `μ(1±ε)`, settle, central-difference the
  outcome. **Answers before any production derivative code:** is the coupled step *smoothly*
  differentiable in the engaged regime (does the central difference converge / stay consistent
  across ε)? is the sign/scale physical (stiffer → higher settle)? does the force-balance
  μ-invariance hold? This proves differentiability + yields the oracle.
- **S1 — analytic contact-force-vs-pose derivative.** `∂force_on_soft/∂(plane height) = −κ·n`
  summed over active pairs — a small routine (sim-soft contact or sim-coupling), unit + FD checked.
- **S2 — analytic coupled-step Jacobian assembly.** Compose `A` (rigid) × `∂plane/∂s` + the S1
  contact-force factor + FD `∂s'/∂xfrc`; gate the assembled `ds'/ds` (or a scalar gradient) against
  the S0 FD oracle. = a differentiable coupled step.
- **S3 — soft-tape `VjpOp` crossing.** Adapt the rigid Jacobian into a chassis `VjpOp` so one
  `tape.backward` crosses both engines; then the co-design gradient w.r.t. soft material params
  (needs the soft material VJP, currently load-only — its own sub-task).

Each leaf: n+1 cold-read + pre-PR ultra-review; no push/PR without go-ahead.

## Progress · S0 — FD differentiability spike DONE (2026-06-10, throwaway, deleted) — ★ R1 RETIRED

Via the public `StaggeredCoupling` API (zero new code), central-difference `∂(settled rigid
height)/∂(soft μ)` at μ₀=30 kPa, two step sizes, settling 1400 steps each:

| ε | z(μ+) | z(μ−) | dz/dμ |
|---|---|---|---|
| 5.0 % | 0.114798 | 0.114777 | 6.969e-9 |
| 2.5 % | 0.114793 | 0.114783 | 6.949e-9 |

**The two FD step sizes agree to 0.3 %** → the coupled step is **smooth + FD-convergent in the
contact-engaged regime** (★ R1 retired). The gradient sign is physical (stiffer soft → platen
settles higher), and the settled contact **force is μ-invariant** (1.9620 N = weight at every μ —
force balance holds independent of stiffness, exactly as predicted). So a gradient genuinely
crosses both engines (rigid settled height ← soft material stiffness), it is well-behaved, and this
FD value is now the **oracle** for the analytic assembly (S1/S2). Differentiability of the coupled
step is well-posed in the smooth regime. **NEXT = S1** (analytic `∂force/∂height = −κ·n`, FD-gated).

## 6. Risks

- **R1 (#1) — the coupled step may not be usefully smooth** (penalty active-set non-smoothness;
  the dynamic-soft + rigid lockstep may make FD noisy). MITIGATION: S0 measures FD convergence in
  the engaged regime directly; if noisy, that's a measured trigger for IPC / a smoothed contact.
- **R2 — the two missing factors are real new code** (`∂force/∂height`, `∂s'/∂xfrc`). MITIGATION:
  FD-first (S0) needs none; the analytic factors (S1/S2) are each small + FD-gated.
- **R3 — `xfrc_applied` auto-zeroing** complicates FD `∂s'/∂xfrc` (the rigid `step` clears it).
  MITIGATION: set `xfrc` fresh each FD evaluation (the coupling already does this).
- **R4 — soft material gradient is load-only today.** The full co-design gradient (∂outcome/∂μ via
  the tape) needs a soft material VJP that doesn't exist. MITIGATION: S0/v1 gets ∂outcome/∂μ by FD
  on the coupled step (no soft material VJP needed); the analytic material-VJP is a later leaf.

## 7. Validation gate (S2 v1)

CI-runnable: an analytically-assembled cross-engine gradient of a coupled-step outcome agrees with
central finite differences to tolerance in the contact-engaged regime; the FD oracle + the
factor derivatives are tested. Honest scope: smooth/engaged regime only (penalty active-set cap
documented, IPC deferred); the full soft-tape `VjpOp` crossing + material-parameter autograd are
leaves beyond v1.

## Key files / pointers

- Forward coupling: `sim/L1/coupling/src/lib.rs` (`StaggeredCoupling::step`).
- Soft autograd: `sim/L0/soft/src/differentiable/newton_vjp.rs` (`NewtonStepVjp`, IFT adjoint,
  load-only); `solver/mod.rs` (`Solver::step`); `sim_ml_chassis` `Tape`/`VjpOp`.
- Rigid derivatives: `sim/L0/core/src/derivatives/{mod.rs (transition_derivatives,
  TransitionMatrices, validate_analytical_vs_fd), fd.rs, hybrid.rs}`. `B`=ctrl only; `xfrc` not in
  `B` and auto-zeroed (`types/data.rs`).
- Contact force: `sim/L0/soft/src/contact/penalty.rs` (`per_pair_readout`, force `+κ(d̂−sd)·n`);
  `contact/rigid.rs` (`RigidPlane`, `sd = p·n − offset`).
