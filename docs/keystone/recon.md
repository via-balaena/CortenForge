# Layer-2 keystone — differentiable soft↔rigid coupling — RECON

*Active recon, opened 2026-06-10. Head-engineer-owned slicing; chosen at the post-M3 mission
fork (Layer-1 substrate fidelity now validated — M1/M2/M3 merged). This is the mission's named
keystone (`MISSION.md`: "the genuinely open research problem… built first and built correctly").*

> **Couple the validated soft-body FEM (`sim-soft`) and the rigid-body engine (`sim-core`) into
> one simulation where tissue and skeleton/device interact with force transmission *both* ways —
> and make that coupled step *differentiable*, so a single gradient flows across both engines.
> This is Mission Layer-2: the keystone the co-design loop consumes.** Build the FORWARD two-way
> coupling first (de-risk stability + force balance), then layer differentiability.

---

## 1. Empirical baseline (what's true today)

- **Two engines, never linked.** `sim-soft` (`sim/L0/soft`) = backward-Euler hyperelastic FEM,
  Newton-to-equilibrium per step, with a **real reverse-mode autograd tape**
  (`differentiable/newton_vjp.rs`: `NewtonStepVjp` = the implicit-function-theorem adjoint of one
  converged Newton step, reusing the factored tangent; gradient currently **w.r.t. applied loads
  θ** only). `sim-core` (`sim/L0/core`) = native-Rust **MuJoCo-aligned** rigid engine
  (`Model`/`Data`, `data.forward`/`step`/`integrate`); the MSK twin runs on it via MJCF emitted by
  `cf-mjcf-emit`. **No crate depends on both** (cross-grep empty) — they have never run in one
  binary.
- **Coupling is one-way + kinematic.** `sim-soft`'s `PenaltyRigidContact` pushes a soft body
  against any `impl Sdf` rigid primitive (`contact/rigid.rs` `RigidPlane`, spheres, scan SDFs,
  cf-design solids). The rigid primitive is **kinematic** (fixed pose during a step); the
  Newton's-3rd-law reaction `−force_on_soft` is **computed but routed nowhere** (`contact/mod.rs`).
  So: rigid→soft pose coupling absent (rigid is static), soft→rigid force absent.
- **Differentiability is asymmetric.** Soft = composable reverse-mode VJP on the chassis `Tape`.
  Rigid = per-step **dense Jacobians** `TransitionMatrices{A,B,C,D}` (`core/derivatives/`, FD +
  analytic-velocity hybrid) aimed at iLQR/MPC — **no tape, no adjoint**. Chaining requires wrapping
  the rigid Jacobians as a chassis `VjpOp`.
- **Known-limited contact.** Penalty contact is explicitly "a stepping stone to IPC, not a
  production baseline" (`penalty.rs`); its active-set is non-smooth (a gradient hazard). IPC +
  edge/face contact + CCD are the deferred "Phase-H" upgrade. The only `ContactPair` variant today
  is soft-vertex-vs-rigid-primitive.

## 2. Thesis

Layer-1 validated each substrate in isolation (rigid vs OpenSim; soft to measured accuracy). The
mission's value — the differentiable body-to-device co-design loop — needs them **coupled with
gradients through the interface**. We did Layer-1 first precisely so this coupling inherits a
*validated* soft substrate. The coupling is genuinely new research, so we de-risk in the right
order: **(1) a stable FORWARD two-way coupling** reusing both engines as-is (the `Sdf` seam +
routing the already-computed reaction force), validated by force balance and an analytical contact
case; **(2) differentiability** — adapt `sim-core`'s dense Jacobians into the soft tape so one
`tape.backward` crosses both, accepting penalty-contact non-smoothness as a known first-cut cap
(IPC later). Forward correctness before gradient correctness; both before the co-design optimizer.

## 3. End-state (keystone v1 done)

A new bridge crate depending on both engines, exposing a **coupled step** that: (a) drives the soft
contact SDF pose from `sim-core` `Data.xpos/xquat` (rigid→soft), (b) routes `−force_on_soft` onto
the rigid body's `xfrc_applied` (soft→rigid), in a **staggered** scheme; (c) runs stably and
conserves the interface force balance on a minimal scene; (d) reproduces an analytical contact
equilibrium (e.g. rigid indenter on soft half-space → Hertz, which sim-soft already validates
one-way); and (e) — keystone v1's headline — is **differentiable end-to-end**: a finite-difference-
checked gradient of a scalar objective flows from the rigid-side outcome back through the coupled
step to a soft-side (and/or rigid-side) parameter. Non-smooth-contact and material-param-gradient
limitations are documented, not hidden.

## 4. Gap table

| # | Have | Need | Leaf |
|---|---|---|---|
| G1 | Two engines, no shared crate | A **bridge crate** depending on `sim-soft` + `sim-core` | S0/S1 |
| G2 | Kinematic rigid SDF | Rigid→soft: **SDF pose driven by `Data.xpos/xquat`** each step | S0/S1 |
| G3 | `force_on_soft` computed, unrouted | Soft→rigid: **reaction → `xfrc_applied`** on the contacting body | S0/S1 |
| G4 | Each engine steps alone | A **staggered coupled step** that is stable + force-balanced | S0/S1 |
| G5 | Soft tape (loads) + rigid dense Jacobians | A **VjpOp adapter** chaining rigid `A/B` (+ contact-force Jacobian) into the soft tape | S2 |
| G6 | Soft gradient w.r.t. loads only | Gradients w.r.t. the **co-design-relevant** params (material/compliance) — *later* | (deferred) |

## 5. Decisions (head-engineer; revisit if S0 says otherwise)

- **D1 — Staggered (partitioned), not monolithic.** Each engine keeps its own solver; they
  exchange pose + force at the interface once per step. Reuses both as-is; monolithic (one unified
  KKT solve) is far harder and a later option only if staggered proves unstable.
- **D2 — Reuse the `Sdf` seam + the unrouted reaction.** Rigid→soft = transform the soft contact
  SDF query by the rigid body frame (`Data.xpos/xquat`); soft→rigid = sum `−force_on_soft` onto
  that body's `xfrc_applied`. Minimal new surface; exploits exactly what already exists.
- **D3 — A new bridge crate.** It must depend on both L0 engines (the no-shared-dep finding). It
  connects two L0 crates, so it is an L1-tier crate; name/location settled in S0 against the
  workspace layout (candidate `sim/L1/coupling`, crate `sim-coupling`; `cf-mjcf-emit` under
  `tools/` is the bridge-crate precedent).
- **D4 — Forward coupling FIRST, differentiability SECOND.** Do not conflate. S0–S1 are a stable,
  validated *forward* two-way coupling (no gradients). Differentiability (G5) is S2+, its own
  #1-risk spike. The mission's "open research problem" is mostly G5; we earn the right to attempt
  it by making the forward coupling solid.
- **D5 — Penalty contact for v1; IPC deferred.** Accept the non-smooth active-set as a known cap on
  gradient quality and stability; document it. IPC (Phase-H) is the eventual upgrade and the real
  path to clean contact gradients — out of keystone-v1 scope.
- **D6 — Minimal first scene.** The smallest scene exercising two-way force/pose: a rigid `sim-core`
  body (indenter / platform) in penalty contact with one soft block — ideally the **rigid-sphere-
  on-soft-half-space (Hertz)** case sim-soft already validates one-way, now with the sphere a real
  `sim-core` body so the equilibrium must be reached by *both* sides.
- **D7 — Reconcile step semantics explicitly.** Soft = implicit Newton-to-equilibrium; rigid =
  MuJoCo-style integration with PGS contact. Decide which side "owns" the contact each step (lean:
  soft owns the penalty contact, rigid receives the reaction as an external force) and the
  sub-stepping/relaxation if a single exchange per step is unstable (Gauss-Seidel vs Jacobi
  staggering; under-relaxation) — settled empirically in S0.

## 6. Sub-leaf ladder

- **S0 — forward-coupling spike (THROWAWAY, `#[ignore]`, uncommitted).** The #1-risk de-risk. In a
  throwaway binary/test, link `sim-soft` + `sim-core`; build a staggered loop on D6's scene: per
  step (i) read rigid pose → pose the soft contact SDF, (ii) soft Newton solve → `force_on_soft`,
  (iii) apply `−force_on_soft` to the rigid `xfrc_applied`, (iv) rigid step. **Answers before any
  production code:** does it run stably (or need under-relaxation / sub-stepping — D7); does the
  interface **force balance** hold (Newton's 3rd law); does it settle to the **analytical contact
  equilibrium** (Hertz); is the `Sdf`-pose + `xfrc_applied` seam (D2) actually sufficient, or are
  fields/APIs missing on either engine.
- **S1 — forward coupling, productionized (committed).** The bridge crate (D3) + a clean coupled-
  step API + the forward gates (force balance, Hertz equilibrium, stability). No gradients yet.
- **S2 — differentiable coupled step (the research leaf).** Adapt `sim-core`'s `TransitionMatrices`
  (+ the contact-force Jacobian) into a chassis `VjpOp` so one `tape.backward` flows across both
  engines; gate with a **finite-difference gradient check** across the coupled step. Document the
  penalty-non-smoothness cap. Its own entry-spike (is the dense-Jacobian→tape adapter well-posed
  through the active set?).
- **S3+ — extend** toward co-design needs: gradients w.r.t. material/compliance params (G6), multi-
  step rollout adjoint (`time_adjoint`, currently stubbed), richer contact (IPC) — sequenced by the
  consumer (the co-design optimizer), not built speculatively.

Each leaf: n+1 cold-read + pre-PR local ultra-review ([[feedback-pre-pr-local-ultra-review]]); no
push/PR without go-ahead. Slicing head-engineer-owned ([[feedback-head-engineer-owns-technical-calls]]).

## 7. Risks

- **R1 (#1) — staggered two-way coupling may be unstable** (added-mass / stiff-contact instability
  is the classic FSI-style failure of partitioned schemes). MITIGATION: S0 measures stability
  directly; D7's under-relaxation / sub-stepping / Gauss-Seidel exchange are the standard fixes; if
  it can't be stabilized cheaply, that's the measured trigger to consider monolithic (D1).
- **R2 — the rigid side has no reverse-mode** (dense Jacobians only). Chaining gradients (G5/S2) is
  the genuinely new differentiable-coupling work and the mission's "open research problem."
  MITIGATION: D4 sequences it after a solid forward coupling; S2 is its own spike.
- **R3 — penalty-contact non-smoothness caps gradient quality** (active-set flips are
  non-differentiable). MITIGATION: D5 accepts it for v1, documents it; IPC is the real fix, later.
- **R4 — step-semantics mismatch** (implicit-to-equilibrium soft vs explicit/implicit rigid).
  MITIGATION: D7 — pick a contact owner + exchange cadence, settle empirically in S0.
- **R5 — missing engine APIs.** The seam assumes `sim-core` exposes per-body external-force
  application (`xfrc_applied`) and pose reads, and that the soft contact accepts a per-step-posed
  SDF. MITIGATION: S0 confirms both; any gap becomes a small, scoped engine-API add.
- **R6 — scope.** This is a multi-year research arc. MITIGATION: keystone *v1* = forward coupling +
  a single differentiable gradient on a minimal scene; everything else (co-design optimizer,
  system-ID, the exo) is downstream and explicitly out of v1.

## 8. Validation gate (keystone v1 done)

CI-runnable: a staggered soft↔rigid coupled step runs stably on the minimal scene, the interface
force balance holds to tolerance, the coupled equilibrium matches the analytical contact case
(Hertz) the soft side already validates one-way, and a finite-difference-checked gradient flows
across the coupled step to a parameter. Honest scope: penalty contact (non-smooth, capped gradient
quality, IPC deferred); loads/contact-parameter gradients first (material-param gradients later);
one minimal scene (general scenes + the co-design optimizer are downstream).

## Key files / pointers

- Mission: `MISSION.md` (roadmap step 2 = the keystone; the co-design optimizer is its consumer).
- Soft side: `sim/L0/soft/src/{contact/{mod.rs,rigid.rs,penalty.rs}, differentiable/newton_vjp.rs,
  solver/mod.rs (Solver::step + tape), sdf_bridge/sdf.rs (the `Sdf` seam)}`.
- Rigid side: `sim/L0/core/src/{lib.rs (Model/Data, forward/step/integrate),
  derivatives/{mod.rs (TransitionMatrices), fd.rs, hybrid.rs}}`; `tools/cf-mjcf-emit` (the
  cf-msk-lib→sim-core bridge-crate precedent).
- Differentiable chassis: `sim_ml_chassis::{Tape (push_custom), autograd::VjpOp}`.
- Layer-1 substrates feeding in: `docs/soft_fidelity/` (soft), `docs/msk_builder/` (rigid/MSK).
