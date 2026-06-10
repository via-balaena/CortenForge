# M3 / near-incompressibility — substrate upgrade — RECON

*Active recon, opened 2026-06-10. Triggered by the M3-S0 model-form spike, which proved the
ν = 0.40 substrate is structurally inadequate for multi-mode (ν binds ~20 % in equibiaxial; see
`recon.md` §"Progress · S0"). User chose to pull the deferred near-incompressibility ("Phase-H
L2") work forward as M3's foundational first arc. Head-engineer-owned slicing.*

> **Bring `sim-soft`'s substrate to near-incompressibility (ν → ~0.499, real silicone) so the
> material response is correct across deformation modes — fixing both the *constitutive* ν error
> (M3-S0: ~20 % in equibiaxial) and the *element volumetric locking* that near-incompressible
> materials induce in linear Tet4, which is why ν is currently capped at 0.45.**

The repo already holds a detailed banked plan for this (the "Phase-H L2" leg): the book
`docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/{00-locking,
01-mixed-up,02-f-bar}.md` and `docs/sim-research/PHASE_H_ROADMAP.md`. This recon adopts that
plan, reconciles it with the M3-S0 finding, and sets the de-risk order.

---

## 0. The key distinction this recon turns on: *constitutive* ν vs *element locking*

There are **two separate ν effects**, and conflating them mis-scopes the work:

- **(C) Constitutive ν.** The material model itself uses ν = 0.40 (`λ = 4μ`). Even the *exact
  analytical homogeneous* response is then a ν = 0.40 response, which differs from incompressible
  by ~20 % in equibiaxial (M3-S0). **Fix: raise the bulk modulus (`λ ≫ μ`, ν → 0.499).** This is a
  parameter change — *no new element needed.*
- **(L) Element volumetric locking.** When a near-incompressible material (ν → 0.5) is used in
  **linear Tet4**, the element *locks*: the per-tet near-isochoric constraint over-constrains the
  12 DOFs (~6 tets/vertex), producing spuriously stiff, artifact-ridden results in
  **inhomogeneous** deformation. **Fix: a mixed u-p or F-bar formulation.** This is the hard part,
  and it is *why* the standalone materials cap at ν < 0.45.

**Critical consequence for M3's validation gate:** the multi-mode coupons (uniaxial/planar/
equibiaxial) are **homogeneous** constant-strain states — a single Tet4 reproduces them *exactly,
for any ν* (constant-strain patch test, proven for uniaxial in M2-S2). **So M3's measured-fidelity
validation does NOT lock** — it needs only the constitutive fix (C): a near-incompressible
material, currently blocked solely by the 0.45 validity cap. **Element locking (L) bites in the
inhomogeneous scenes the *downstream mission* needs** (device sims, contact, the co-design loop) —
so (L) is what makes near-incompressibility *real across the whole substrate*, not just in
homogeneous coupons. This recon scopes both, and the ladder sequences them so M3 is unblocked early
while the general fix lands properly.

## 1. Empirical baseline (what's true in the code today)

- **Materials cap at ν < 0.45.** `NeoHookean::from_young_poisson` / `Yeoh::from_young_poisson_and_c2`
  assert `nu < 0.45`; `ValidityDomain.poisson_range = (-1.0, 0.45)` (`material/mod.rs:97`,
  `neo_hookean.rs:95`, `yeoh.rs:232`). The solver re-checks validity at step boundaries. **Escape
  hatch for spikes/tests:** `Yeoh::from_lame_and_c2(μ, λ, c2)` is a *raw* Lamé constructor with **no
  ν assert** — `λ = 499μ` gives ν ≈ 0.499 directly.
- **Element = linear Tet4, one Gauss point.** F computed per-tet from nodal positions
  (`backward_euler.rs::deformation_gradient`, ~L2110); volumetric energy `(λ/2)(ln J)²`, stress
  `+λ(ln J)F⁻ᵀ` (`neo_hookean.rs`, `yeoh.rs`). Assembly hard-wires 12 DOF/tet, SPD Cholesky (with
  LU fallback), factor-on-tape for autograd.
- **No locking cure exists.** ν = 0.40 is the deliberate workaround that absorbs locking error into
  calibration (`silicone_table.rs:555` comment). Locking-sensitive tests (`concentric_lame_shells`,
  `bonded_bilayer_beam`, `hertz_sphere_plane`) all run at ν = 0.40 and document "Phase H Tet10 +
  F-bar recovers the near-incompressible regime."
- **M2 shipped roller/per-axis BCs** — relevant: pressure DOFs (mixed u-p) are scalar, so no
  per-axis interaction; and the multi-mode coupons reuse the M2 roller template.

## 2. Thesis

Near-incompressibility is the deepest substrate lever in the soft-FEM stack: *every* confined,
contact, or multiaxial scenario downstream inherits the ν error, not just M3's equibiaxial gate.
M3-S0 turned the long-deferred "we should fix ν someday" into a measured trigger (~20 % equibiaxial
constitutive error; the locking that blocks ν > 0.45 is the reason it was deferred). The banked
plan's call stands: **mixed u-p is the product-grade cure** (Ecoflex is ν ≈ 0.499, *outside*
F-bar's ν ≤ 0.49 validity), with F-bar as a cheaper secondary decorator for less-incompressible
materials. But linear tets are *notoriously* poor for incompressibility, so the formulation must be
**measured on a locking benchmark before the assembly rewrite is committed** — the #1 risk.

## 3. End-state (this arc done)

(a) A near-incompressible material path (ν → 0.499) that is **validity-legal** (the 0.45 cap lifts
via a composition rule, not a bare edit). (b) A locking-cure formulation — **mixed u-p
(`MixedUP<M>` decorator, P1 displacement + P0 per-tet pressure, static condensation to preserve the
SPD reduced solve)** per the banked plan, *unless the S0 benchmark says a different tet-incompressible
cure is needed* (see R1). (c) Locking benchmarks passing at ν = 0.499: a near-incompressible
**cantilever** (tip deflection vs Euler-Bernoulli, bare Tet4 fails/locks → cured passes) and **Cook's
membrane**. (d) The autograd/Newton/contact paths intact (factor-on-tape preserved through static
condensation). (e) M3's homogeneous multi-mode coupons re-run at ν → 0.499 (the constitutive fix),
feeding the multi-mode validation. (f) The ν = 0.40 workaround retired in the locking-sensitive
tests, re-validated at ν = 0.499.

## 4. Gap table

| # | Have | Need | Leaf |
|---|---|---|---|
| G1 | ν < 0.45 cap (asserts + `poisson_range`) | A **composition rule** widening ν → 0.499 only when a locking cure wraps the material | S1 |
| G2 | Compressible homogeneous response | The **constitutive fix**: near-incompressible material for the homogeneous multi-mode coupons (no locking) | S1 |
| G3 | Linear Tet4, no locking cure | A **mixed u-p** (or S0-selected) formulation that relieves locking in inhomogeneous scenes | S2 |
| G4 | SPD Cholesky factor-on-tape | Saddle-point assembly + **static condensation** preserving the reduced SPD solve + autograd | S2 |
| G5 | No locking benchmark | **Cantilever** + **Cook's membrane** gates at ν = 0.499 (bare Tet4 locks → cured passes) | S0/S3 |
| G6 | ν = 0.40 workaround in tests | The locking-sensitive tests re-validated at ν = 0.499 | S3 |

## 5. Decisions (head-engineer; revisit if S0 says otherwise)

- **D1 — Adopt the banked plan: mixed u-p primary, F-bar secondary.** The target is ν ≈ 0.499
  (Ecoflex), outside F-bar's ν ≤ 0.49 validity, so mixed u-p is the product path; F-bar can ship
  later as a cheaper decorator for ν ≤ 0.49 materials. *Conditioned on S0 (R1).*
- **D2 — Decorator architecture (`MixedUP<M: Material>`).** Wrap any `Material`; widen
  `poisson_range` via `ValidityDomain` composition; pressure assembly + static condensation in the
  solver layer; the `Material` trait surface and `first_piola`/`tangent` of the base law stay
  unchanged. Matches the book; keeps the change additive.
- **D3 — Sequence the constitutive fix (C) ahead of the locking cure (L).** S1 lifts the cap (via
  the composition rule) + runs the homogeneous multi-mode coupons at ν → 0.499 — this **unblocks
  M3's measured-fidelity validation immediately** (homogeneous, no locking) and banks an early win,
  while S2 builds the general locking cure. Honest scope: after S1, near-incompressibility is real
  *for homogeneous states*; after S2 it is real *everywhere*.
- **D4 — S0 spike DECIDES the formulation before the assembly rewrite.** Linear tets are notoriously
  poor for incompressibility (P1-P0 has known inf-sup/checkerboard caveats; nodal-averaged-pressure
  and Tet10 are the robust alternatives). S0 must measure, on a real bending benchmark, (i) how badly
  bare Tet4 locks at ν → 0.499, and (ii) whether the banked P1-P0 mixed u-p actually cures it or
  whether a nodal-averaged-pressure / Tet10 cure is needed. *Do not commit the assembly rewrite to a
  formulation the benchmark hasn't vindicated.*
- **D5 — Validity widening is a composition rule, never a bare cap edit.** Lifting `nu < 0.45`
  globally would silently let unwrapped materials lock. The cap lifts *only* through the decorator's
  `ValidityDomain` composition (G2.3 in the roadmap).

## 6. Sub-leaf ladder

- **S0 — locking-disease spike (THROWAWAY, `#[ignore]`, uncommitted).** De-risk R1 + ground the
  whole arc. Using `Yeoh::from_lame_and_c2(μ, 499μ, 0)` (bypasses the cap), on a **cantilever**
  (reuse `HandBuiltTetMesh::cantilever_bilayer_beam` geometry + tip load): (i) quantify how badly
  bare Tet4 **locks** at ν → 0.499 vs Euler-Bernoulli and vs the ν = 0.40 result (expect large
  over-stiffening); (ii) confirm a **homogeneous** coupon (the M2 roller cell) does **NOT** lock at
  ν → 0.499 (recovers the analytical near-incompressible stress) — proving the constitutive-vs-
  locking distinction §0 and that M3's validation is unblocked by S1 alone; (iii) if cheap, prototype
  the F-bar volumetric modification (patch/mean-dilatation `J̄`) to see how much it relieves the
  cantilever locking — informs D1/D4. **Answers before any production code:** is locking real + how
  large in our code; does the homogeneous path need the cure at all; which formulation to commit to.
- **S1 — constitutive fix + validity composition (committed).** A near-incompressibility composition
  rule (the decorator's `ValidityDomain` widening, even before the full solver support — or a guarded
  near-incompressible material path) that makes ν → 0.499 *legal* for the **homogeneous** coupons;
  run the M3 multi-mode coupons at ν → 0.499 and bank the constitutive correction (the ~20 % the M3
  recon needs). Unblocks M3 measured-fidelity validation. *No locking cure yet — scoped to
  homogeneous.*
- **S2 — the locking cure (the main arc).** Implement the S0-selected formulation (mixed u-p
  `MixedUP<M>` per the banked plan, unless S0 redirects): saddle-point assembly + per-tet P0 pressure
  + static condensation preserving the SPD reduced solve and factor-on-tape; widen `poisson_range`
  via composition; keep contact/autograd intact. Gate: cantilever + Cook's membrane at ν = 0.499
  (bare Tet4 locks → cured passes). This is large — slice into its own PR(s).
- **S3 — re-validate + retire the workaround.** Re-run `concentric_lame_shells` / `bonded_bilayer_
  beam` / `hertz_sphere_plane` at ν = 0.499 with the cure; retire the ν = 0.40 stopgap where it was
  a locking workaround; bank the substrate upgrade.

Each leaf: n+1 cold-read + pre-PR local ultra-review ([[feedback-pre-pr-local-ultra-review]]); no
push/PR without go-ahead. S2 especially is multi-PR.

## Progress · S0 — locking-disease spike DONE (2026-06-10, throwaway `tests/spike_locking_s0.rs`, #[ignore], uncommitted)

R1 confirmed + sized in our own solver. Slender cantilever (10×2×2, L/H=5), same μ, ν swept via
`NeoHookean::from_lame(μ, λ(ν))` (raw Lamé, no cap), tip deflection vs Euler-Bernoulli (physical E
rises only ~7 % over the range, so FEM/EB should stay ~constant if there were no locking):

| ν | λ/μ | E/μ | FEM/EB | vs ν=0.40 |
|---|---|---|---|---|
| 0.40 | 4 | 2.80 | **0.425** | 1.00 |
| 0.49 | 49 | 2.98 | **0.129** | 0.29 |
| 0.499 | 499 | 3.00 | **did not converge** | — |
| 0.4999 | 4999 | 3.00 | **did not converge** | — |

**FEM/EB collapses 0.425 → 0.129 by ν=0.49 = 3.3× spurious stiffening; at ν ≥ 0.499 the
near-incompressible bare-Tet4 system fails to converge at all** (Armijo-stall / iter-cap — the
extreme symptom). So bare Tet4 is unusable for near-incompressible *inhomogeneous* deformation —
exactly why the 0.45 cap exists. **The constitutive-vs-locking distinction (§0) holds:** this
bending problem locks, whereas the homogeneous multi-mode coupons are exact for any ν (constant-
strain patch test, M2-S2). ⇒ **M3's measured-fidelity validation is unblocked by S1 (the
constitutive fix) alone; S2 (the locking cure) is what makes near-incompressibility real in the
inhomogeneous device sims the mission needs.** The arc is correctly motivated; the ladder order
(S1 constitutive → S2 cure) stands. (Spike did NOT prototype F-bar — the cure-selection half of
D4/R1 stays for an S2-entry spike; this S0 settled "is the disease real + does the homogeneous
path need the cure", both answered.) Spike is throwaway — delete when S1/S2 lands.

## 7. Risks

- **R1 (#1) — linear Tet4 is fundamentally poor for incompressibility; the banked P1-P0 mixed u-p may
  not fully cure it.** P1-P0 (continuous u, discontinuous constant p) has known inf-sup/checkerboard
  caveats on tets; the robust tet cures are often nodal-averaged-pressure (Bonet-Burton), F-bar-patch,
  or quadratic Tet10. The book is optimistic ("stable on well-shaped Tet4 meshes"). **MITIGATION: S0
  measures the cure on a real benchmark before the assembly rewrite is committed; if P1-P0 is
  inadequate, S0 redirects D1 to nodal-averaged-pressure or Tet10.**
- **R2 — static condensation must preserve the differentiable factor-on-tape path.** The saddle-point
  system breaks the plain SPD assumption; condensation restores an SPD reduced system but the autograd
  VJP (`push_newton_step_vjp`) must thread through. MITIGATION: condensation is local (per-tet scalar
  K_pp); design the reduced system to reuse the existing factor path; test gradcheck.
- **R3 — contact + near-incompressibility interaction.** The penalty/contact path and the validity
  gate run alongside; near-incompressible contact (Hertz) is a known stress test. MITIGATION: keep
  contact tests at their current ν initially; bring Hertz to ν = 0.499 only in S3 after the cure is
  proven on locking benchmarks.
- **R4 — scope.** This is the banked "Phase-H L2" (~a quarter in the roadmap). MITIGATION: D3
  sequences an early homogeneous win (S1) that unblocks M3; S2 (the big arc) is sliced into its own
  PRs; the recon→spike→implement discipline keeps each step gated.
- **R5 — silent global cap lift.** Editing `nu < 0.45` directly would let unwrapped materials lock
  unnoticed. MITIGATION: D5 — widen only via the decorator's `ValidityDomain` composition.

## 8. Validation gate (definition of done)

CI-runnable: bare Tet4 at ν = 0.499 demonstrably locks on the cantilever/Cook's benchmark; the cured
formulation recovers the reference within tol; the homogeneous multi-mode coupons recover the
analytical near-incompressible response; autograd gradcheck passes through the cured assembly;
contact + locking-sensitive tests re-validated at ν = 0.499; the ν = 0.40 workaround retired where it
was a locking stopgap. Honest scope: this is the substrate-side upgrade; it feeds — and is fed by —
M3's measured multi-mode validation against the Hossain oracle.

## Key files / pointers

- Banked plan: `docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/{00-locking,
  01-mixed-up,02-f-bar}.md`; `docs/sim-research/PHASE_H_ROADMAP.md` (L2); `docs/SIM_ARCHITECTURE_AUDIT.md`.
- Materials/cap: `sim/L0/soft/src/material/{mod.rs (ValidityDomain.poisson_range), neo_hookean.rs,
  yeoh.rs (from_lame_and_c2 = raw, no ν assert)}`.
- Element/assembly: `sim/L0/soft/src/solver/backward_euler.rs` (F computation ~L2110, 12-DOF
  assembly, factor-on-tape, `push_newton_step_vjp`).
- Benchmarks: `tests/{bonded_bilayer_beam.rs (cantilever geometry), concentric_lame_shells.rs,
  hertz_sphere_plane.rs}`; coupon template `tests/uniaxial_roller_coupon.rs`.
- Trigger: `m3_deformation_modes/recon.md` §"Progress · S0".
