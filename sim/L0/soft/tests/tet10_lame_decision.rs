//! Tet10 ladder rung 6 — the Lamé accuracy-decision harness.
//!
//! `docs/SIM_SOFT_TET10_PLAN.md` §4 + §5 step 6. This file exists to answer
//! one pre-registered question: **is pure-displacement Tet10 accurate enough
//! at ν = 0.49 (Ecoflex 00-30), or does near-incompressibility need a mixed
//! Taylor-Hood P2-P1 formulation on top?** The rung-6 ladder is:
//!
//! - **6a (landed)** — commit the *real* Tet4 ν = 0.4 baseline `e₄₀` at
//!   the decision mesh. The plan's prior "Tet4 → 0.993 at ν = 0.4" figure was
//!   an uncommitted spike that exists nowhere in the tree; the ν = 0.49
//!   threshold subtracts against a measured number or against nothing.
//! - **6b (this commit)** — Tet10 at ν = 0.4 must match-or-beat Tet4 on the
//!   same harness. A miss *there* localizes to a forward/assembly bug, not to
//!   an incompressibility limit, which is the whole point of running it before
//!   the ν = 0.49 gate.
//! - **6c** — Tet10 at ν = 0.49 against the pre-registered three-way gate.
//!
//! ## Why a second Lamé file, and not a ν knob on `concentric_lame_shells`
//!
//! The shipped IV-5 oracle is a *convergence* gate on the **three-shell**
//! sphere with deliberately loose bands (`< 0.20` fine-end, `< 0.30`
//! sanity). Three properties make it unable to resolve the rung-6 decision,
//! each fixed here (plan §4, agent-verified):
//!
//! 1. **Three shells → wrong signal.** The SDF mesher tags material per tet
//!    at the centroid, so a thin band of straddler-tets at each shell
//!    interface carries the wrong `(μ, λ)`. That error is *element-order
//!    independent* — Tet10 does not fix it — and at these radii it is
//!    comparable to the locking signal it would have to expose. This harness
//!    runs the **uniform single-material** sphere, where the material field
//!    is exact at every Gauss point and the only error left is
//!    discretisation.
//! 2. **Loose bands straddle the decision.** `< 0.20` admits both a healthy
//!    element and a locked one. This harness commits tight per-configuration
//!    numbers.
//! 3. **★ The equal-split cavity load is inconsistent for a quadratic
//!    surface.** IV-5 applies `p·A_v·n̂` with `A_v = 4πR²/N_loaded` split
//!    equally over the loaded band. On a P2 face the consistent load is
//!    `∫ p·N_i dA`, which puts **zero** on the corners (`∫N_corner dA = 0`
//!    exactly) and `A_f/3` on each midside — the exact opposite of an equal
//!    split. This harness therefore applies the consistent boundary-face
//!    load for **both** elements (P1 `A_f/3` per corner for Tet4, P2 for
//!    Tet10) and selects the loaded / pinned bands by **boundary-face
//!    membership**, not by the Tet4-tuned `0.5·cell_size` radial predicate
//!    (which sweeps borderline *radial*-edge midsides — nodes that are not
//!    on the cavity surface at all — into both the load and the readout).
//!    The load rule is not a detail: [`LoadRule`] is a first-class knob and
//!    [`tet4_load_rule_ab_at_nu_0_4`] commits the A/B, so the size of the
//!    convention's effect is a re-runnable measurement rather than a claim.
//!
//! ## ★ What "pre-registered" does and does not cover here
//!
//! The **thresholds** are pre-registered: `ACCEPT iff converged and
//! |rel_err| ≤ max(2·e₄₀, 0.10)`, `REJECT if > 0.20 or Newton stalls`, gray
//! zone `0.10–0.20`. They were fixed in the merged plan (#679) before any
//! measurement and are not touched here.
//!
//! The **harness is not blind**, and saying otherwise would be false. The
//! conventions above — consistent load, corner-only readout, fixed analytic
//! target, load-scaled tolerance — were each chosen while their effect on the
//! ν = 0.49 reading was already known from a spike. Three things a skeptic
//! should weigh, stated plainly rather than buried:
//!
//! - Every convention here is defensible *independently* of the outcome:
//!   `∫N_corner dA = 0` is an identity, not a preference; evaluating a field
//!   outside its domain is an error under any verdict; an L1 load against an
//!   L2 residual is a units mismatch.
//! - The choices did **not** all point one way. The consistent load makes
//!   Tet4 look *worse* (`e₄₀` 0.0140 → 0.0615 versus the equal split) while
//!   improving Tet10 — the direction a skeptic should challenge — but it also
//!   makes the ACCEPT band *stricter* (`max(2·e₄₀, 0.10)` = 0.123 rather than
//!   the 0.299 the shipped oracle's number would have licensed). The band
//!   inherits the convention outright, though, and 6c decides against it:
//!   `max(2·e₄₀, 0.10)` is **0.100** under [`LoadRule::EqualSplit`], **0.123**
//!   under [`LoadRule::Continuum`], **0.251** under [`LoadRule::Facet`].
//! - Note the plan's three rules **overlap**: ACCEPT admits
//!   `|rel_err| ≤ max(2·e₄₀, 0.10)` while the gray zone `(0.10, 0.20]`
//!   defaults to REJECT, so any `e₄₀ > 0.05` leaves a band that satisfies
//!   both. Item 2 below closes it.
//!
//! ### ★ Pre-registered for 6c, fixed here in rung 6b — before any ν = 0.49
//! ### number is committed
//!
//! Rung 6b measured that the load rule moves the Tet10 anchor by 17× and the
//! ACCEPT band by 2×, which makes the rule the largest remaining lever on the
//! verdict. Choosing it after seeing a ν = 0.49 reading would be the whole
//! game. So it is fixed now, while only ν = 0.4 data exists:
//!
//! **★ Every threshold below is on `|rel_err|`, never the signed value.**
//! [`Reading::rel_err`] is signed and volumetric locking drives it *more
//! negative*, so a signed `rel_err ≤ 0.10` would be satisfied trivially by a
//! fully locked element. Absolute value, everywhere.
//!
//! 1. **6c must ACCEPT under BOTH consistent rules** — [`LoadRule::Continuum`]
//!    *and* [`LoadRule::Facet`]. Passing one and failing the other is a
//!    REJECT. This removes the `Continuum`-vs-`Facet` choice as a lever
//!    instead of letting the author pick the flattering one: `Continuum` is
//!    both the shipped default and the rule under which Tet10's margin is
//!    largest, so requiring `Facet` too is the conservative direction. It is a
//!    real tightening, not decoration — at ν = 0.4 `Continuum` sits 40× inside
//!    its bar while `Facet` sits 4.8× inside, so `Facet` binds ~8× harder.
//!    Cost is two extra solves.
//! 2. **The overlap resolves toward REJECT, and the bar is 0.10 under every
//!    rule.** In `[0.10, max(2·e₄₀, 0.10)]` the gray-zone rule wins over the
//!    ACCEPT rule: it is the more specific of the two, and the plan states
//!    gray "defaults to REJECT". In effect, for each consistent rule —
//!    **ACCEPT** needs `|rel_err| ≤ 0.10`; `0.10 < |rel_err| ≤ 0.20` is gray;
//!    `|rel_err| > 0.20` or a stall REJECTs. Since `max(2·e₄₀, 0.10)` is
//!    0.123 under `Continuum` and 0.251 under `Facet`, both exceed 0.10, so
//!    **the `2·e₄₀` term is inert here** — it can only ever tighten the bar
//!    below 0.10, never loosen it above. Do not quote 0.123 / 0.251 as the
//!    operative bars; 0.10 is.
//! 3. **The gray-zone escape is defined, and its failure mode registered.**
//!    "Converges downward through 0.10" means: the h/4 reading under the *same
//!    rule* is both `≤ 0.10` and strictly below the h/2 reading. **If the h/4
//!    solve cannot be run, the verdict is REJECT** — h/4 Tet10 measured 7.13 GB
//!    / 257 s at ν = 0.4, and the plan warns the ν = 0.49 LU-fallback path
//!    doubles fill, so "we could not afford the confirmation" must not become
//!    an ACCEPT by default.
//! 4. **The verdict is read off the absolute gate only.** [`TET10_NU_0_4`] is
//!    reporting context, not a threshold: once a ν = 0.49 number exists a
//!    *differential* reading (`|rel(0.49)| − |rel(0.4)|`) becomes available and
//!    would flatter, since it subtracts away the element's baseline error. It
//!    is not a pre-registered criterion and 6c may not substitute it.
//! 5. **`EqualSplit` is disqualified as a verdict rule**, on the rung-6b
//!    evidence rather than on theory: it inverts the element ordering at
//!    ν = 0.4 (see [`TET10_NU_0_4_EQUAL_SPLIT`]), so a verdict read under it
//!    would be a statement about the load, not the element. ⚠ **This is the
//!    one item pointing the permissive way** — `EqualSplit`'s bar is 0.100
//!    while Tet10 already reads 0.2405 at ν = 0.4, so keeping it as a verdict
//!    rule would force an automatic REJECT. It is disqualified because a rule
//!    that inverts the known ordering cannot adjudicate the element, not
//!    because of where it lands. It stays committed as a control.
//!
//! So 6c is best understood as a *committed, re-runnable confirmation* of a
//! spike result under conventions chosen in the open, not as a blind gate.
//!
//! ## The oracle
//!
//! Single-material thick-walled hollow sphere, internal cavity pressure,
//! **fixed outer surface** — the same closed form IV-5 derives, collapsed to
//! one shell. Under spherical symmetry `u_r(r) = A r + B / r²` with
//! `σ_rr(r) = (3λ + 2μ) A − 4μ B / r³`, and two boundary conditions:
//! `σ_rr(R_a) = −p` (cavity traction, compression sign) and `u_r(R_b) = 0`
//! (the pinned outer skin, which also kills all six rigid-body modes). Two
//! equations, two unknowns, solved in closed form by [`Lame::solve`] — no
//! 6×6 assembly and no LU, because the single-shell case reduces by hand.
//! Timoshenko & Goodier, *Theory of Elasticity* 3rd ed. §141. At the measured
//! 0.45 % cavity-radius inflation the Neo-Hookean response is in its
//! linear-elastic regime, where it reduces to Lamé.
//!
//! **★ Measurement convention — identical node set, fixed analytic target.**
//! The FEM mean is the Saint-Venant average of `‖x_final‖ − ‖x_rest‖` over
//! the cavity-surface **corner** nodes, and the analytic target is the single
//! value `u_r(R_a)`. Two deliberate choices, both learned the hard way:
//!
//! - **Corners only, for both elements.** Enrichment preserves corner ids
//!   and corner positions bit-identically, so Tet4 and Tet10 are read at the
//!   *same physical points*. Including Tet10's midside nodes would widen the
//!   readout set to 434 nodes sitting deeper inside the cavity than Tet4's
//!   110, where `u_r` is larger — an element-dependent readout bias that has
//!   nothing to do with element accuracy. The extra midside DOFs still shape
//!   the solution; they just do not shift the ruler.
//! - **A fixed `u_r(R_a)` target, not a per-node one.** An earlier draft
//!   evaluated `u_r` at each node's own rest radius, reasoning that this
//!   removed the straight-edge geometric offset. The faceted cavity surface
//!   is *inscribed*: measured, **96 of the 110 corner nodes lie BELOW `R_a`**
//!   (mean radius 0.039795 — 0.51 % inside; min 0.039494), where
//!   `u_r = A r + B/r²` is not the solution to any problem but the analytic
//!   continuation into the empty cavity, and the `B/r²` term grows fastest
//!   exactly there. Measured, that convention inflates the analytic target by
//!   **+1.15 % on the corner set and +4.31 % on Tet10's 434-node all-cavity
//!   set** — so with the *old* all-cavity readout it was an element-dependent
//!   ~3 % shift in the oracle itself, larger than the effects being measured.
//!   Note the two fixes do different work: corner-only readout alone removes
//!   the element-*dependence* (the +1.15 % is identical for both elements on
//!   the same nodes), and the fixed target additionally removes that
//!   remaining systematic. `u_r(R_a)` is also what IV-5's own cavity-wall
//!   gate compares against.
//!
//! **Surface-area normalisation, and what it costs.** Under
//! [`LoadRule::Continuum`] the face weights are scaled so the total tributary
//! area equals the exact analytic cavity area `4πR_a²`, and the per-node load
//! direction is the exact radial `r̂` at the node's rest position. The point
//! is to make the discrete load the best available approximation of the
//! *continuum* traction the oracle assumes, so that what is left is element
//! behaviour rather than the mesher's faceting. **This is a real thumb on the
//! scale and the file does not hide it:** the inscribed faceted surface
//! measures `1.926921e-2` against the exact `2.010619e-2`, a **4.16 % area
//! deficit**, so `Continuum` applies 4.34 % more load than the true facet
//! areas — and its radial direction adds more still, because facet normals
//! tilt away from `r̂`. The directly measured consequence at ν = 0.4: Tet4
//! reads **0.0615 under `Continuum` and 0.1257 under
//! [`LoadRule::Facet`]** — a 6.4-point swing, comparable to the Tet4→Tet10
//! element-order gap itself (5.8 points under `Continuum`, 7.3 under `Facet`).
//! The A/B tests commit every rule for both elements so the convention factor
//! stays visible and re-runnable. The element *ordering* is robust across the
//! two **consistent** rules — Tet10 beats Tet4 under both — but **not** across
//! all three: under `EqualSplit` it inverts outright, see
//! [`TET10_NU_0_4_EQUAL_SPLIT`]. The *absolute* numbers are not robust to the
//! choice at all.
//!
//! ## ★ The residual tolerance must scale with the load
//!
//! [`SolverConfig::skeleton`] ships an **absolute** `tol = 1e-10`, and the
//! residual floor of this problem is **element**-dependent as much as
//! ν-dependent: measured on this harness, Tet4 at ν = 0.49 floors at
//! `5.64e-11` (below the shipped tolerance, so it converges), while Tet10 at
//! ν = 0.49 floors at `1.43e-10` — *above* it. A fixed absolute tolerance
//! would therefore stall Armijo on the very configuration rung 6c must judge,
//! and "Newton stalls at ν → 0.5" would be read as a locking REJECT when it
//! is a roundoff artifact. So every run scales the tolerance to the applied
//! load: `tol = TOL_RELATIVE · ‖θ‖₂`, an L2 norm to match the solver's own L2
//! `free_residual_norm` (an L1 load sum against an L2 residual would hand the
//! two elements different effective tolerances — Tet10 spreads the same total
//! thrust over ~3× the nodes, so its L1 and L2 norms diverge).
//!
//! This cannot let an unconverged or locked solution pass. Newton here goes
//! from `O(1)` to the floor in **3 iterations**, so no residual value exists
//! in the window a tighter bound would catch; readings are bit-identical
//! across `tol ∈ [1e-9, 1e-7]`. Locking does not show up as a large residual
//! at all — Tet4 at ν = 0.49 reads a badly wrong displacement with a clean
//! `5.6e-11` residual. [`report_and_check_physical`] asserts the achieved
//! residual and the iteration count directly rather than trusting the
//! tolerance.
//!
//! ## Profile cost — this file runs in ordinary CI
//!
//! Measured, not inherited: the workspace pins `opt-level = 2` on both
//! `[profile.dev]` and `[profile.test]`, so sim-soft's "~30× slower in debug"
//! rule of thumb does not apply to this crate's compute-bound solves. The
//! Tet10 h/2 solve is **5.42 s in debug against 3.16 s in release (1.7×)**,
//! with a bit-identical result; the Tet4 solves are ~0.1 s. So the file is
//! *not* `#[cfg_attr(debug_assertions, ignore)]` and needs no entry in the
//! hand-maintained release `--test` list of `.github/workflows/quality-gate.yml`
//! — it is auto-discovered and executed by the `tests-debug` job like any
//! ordinary integration test. That is deliberately the safer arrangement:
//! a release-only sim-soft test that nobody hand-registers runs in **no CI
//! job at all**, which is the state `bonded_layer_indentation` (the ladder's
//! demand-#1 gate) is in today.
//!
//! The h/4 mesh-stability confirmation is the one thing that stays out: at
//! 94,710 nodes / 284k DOF it measured **7.13 GB peak RSS over 257 s**, above
//! what a 7 GB CI runner has. It is a pre-push one-shot, never CI.

#![allow(
    // The measurements are engineering quantities with committed analytic
    // baselines — displacements, relative errors, Lamé coefficients — not
    // bit patterns. `float_cmp` would fire on every comparison below.
    clippy::float_cmp,
    // Saint-Venant averaging is `Σ u_r / N`; the count-to-f64 cast is the
    // canonical FEM idiom, and these meshes hold far fewer nodes than f64
    // represents exactly.
    clippy::cast_precision_loss,
    // Vertex counts index a `u32` id space (the `Mesh` trait's `VertexId`);
    // the decision meshes hold ~30k vertices, far below `u32::MAX`.
    clippy::cast_possible_truncation,
    // `from_sdf` / scene construction surface meshing failures as test
    // panics — the canonical sphere either meshes or has regressed in a way
    // worth investigating (mirrors the IV-5 convention).
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::element::{TET10_EDGE_NODES, Tet10};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, CpuTet10NHSolver,
    LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_OUTER, LoadAxis, MaterialField, Mesh, NullContact,
    SdfMeshedTetMesh, SoftScene, Solver, SolverConfig, SolverFailure, Tet4, Tet10Mesh, Vec3,
    VertexId,
};
use std::collections::{BTreeMap, BTreeSet};

// ── Material, load, and mesh constants ───────────────────────────────────

/// Shear modulus, shared by every configuration. Matches IV-5's middle
/// composite shell (`MU_MIDDLE`), so the uniform sphere here is IV-5's
/// uniform-2× baseline with a swept Poisson ratio.
const MU: f64 = 2.0e5;

/// Baseline Poisson ratio. Every standalone soft gate runs here because Tet4
/// volumetrically locks as ν → 0.5; `e₄₀` is measured at this ν.
const NU_BASELINE: f64 = 0.4;

/// **The decision Poisson ratio.** Ecoflex 00-30 is ν ≈ 0.49; every standalone
/// soft gate avoids it because Tet4 volumetrically locks as ν → 0.5. Whether
/// pure-displacement Tet10 is accurate enough *here* is the question the whole
/// ladder exists to answer (plan §1 demand #2).
const NU_DECISION: f64 = 0.49;

/// Internal cavity pressure (Pa) — IV-5's `PRESSURE`. Measured cavity
/// inflation at ν = 0.4 is `1.7942e-4 / 0.04 = 0.45 %` of `R_CAVITY`,
/// comfortably inside the small-strain window where Neo-Hookean reduces to
/// linear-elastic Lamé.
const PRESSURE: f64 = 5.0e3;

/// **The decision mesh.** IV-5's `CELL_SIZE_H2`: 6,456 tets → 4,682 Tet4
/// vertices / 13,336 Tet10 nodes (40,008 DOF). The h/4 Tet10 mesh is 94,710
/// nodes / 284k DOF and measured 7.13 GB peak RSS over 257 s, so it stays a
/// pre-push one-shot and never enters CI; the locking signal is an element
/// property visible at moderate refinement, and Lamé convergence here is
/// already super-quadratic h/2 → h/4.
const CELL_SIZE_DECISION: f64 = 0.02;

/// Mesh for the gray-zone / mesh-stability confirmation — IV-5's `CELL_SIZE_H4`.
/// 94,710 Tet10 nodes / 284k DOF. The ν = 0.49 confirmation measured **7.16 GB
/// peak RSS over 500 s for its two solves** (the 7.13 GB / 257 s quoted
/// elsewhere is rung 6a's single-solve ν = 0.4 cost). Pre-push one-shot only:
/// the test that uses it is `#[ignore]`d so it never runs in CI, where a 7 GB
/// runner would OOM.
const CELL_SIZE_CONFIRMATION: f64 = 0.01;

/// Static regime — IV-5's `STATIC_DT`. At `dt = 1` the inertial term `M/Δt²`
/// sits ~4 orders below stiffness, so a single `replay_step` from rest lands
/// on static equilibrium. Corollary: **this whole file is mass-blind** and
/// cannot validate the Tet10 HRZ lumped mass; that is the rung-7 dynamics
/// gate's job. Make no mass claims from these numbers.
const STATIC_DT: f64 = 1.0;

/// Newton budget. IV-5 runs at 50; `fbar_locking` needed 150 at ν = 0.49, so
/// the decision harness carries the larger budget for every configuration
/// rather than handing the near-incompressible run a special allowance.
/// Every configuration measured so far converges in **3** iterations — see
/// [`MAX_EXPECTED_NEWTON_ITER`], which is the bound that actually bites.
const MAX_NEWTON_ITER: usize = 150;

/// Convergence tolerance as a fraction of the applied load's L2 norm — see
/// the module docs' tolerance section. Chosen well above the highest measured
/// residual floor (Tet10 at ν = 0.49: `1.43e-10` at h/2, `2.08e-10` at h/4)
/// and nine orders below the load itself.
const TOL_RELATIVE: f64 = 1e-9;

/// Iteration bound that actually discriminates. `MAX_NEWTON_ITER` cannot
/// fail an assertion — [`Solver::try_replay_step`] returns
/// `Err(SolverFailure::NewtonIterCap)` on budget exhaustion, so any
/// [`Reading`] that exists at all converged inside it. Every configuration
/// measured takes 3 iterations, so a reading above this bound means the
/// convergence regime changed even though the solve still succeeded.
const MAX_EXPECTED_NEWTON_ITER: usize = 10;

/// Absolute ceiling on the accepted residual, independent of the tolerance
/// the solver was configured with. Guards against a future regression that
/// converges to a sloppier residual under a tolerance that silently allows
/// it.
///
/// **The floor scales with λ, so this ceiling must cover the stiffest
/// configuration any test here drives.** Measured: `1.4e-10` at ν = 0.49 h/2,
/// `2.1e-10` at ν = 0.49 h/4, and `1.5e-9` at ν = 0.499 — a 10× rise in λ/μ
/// moves the floor an order of magnitude while the applied load is unchanged,
/// so a load-relative ceiling would not track it either. `3e-9` clears the
/// worst measured case by 2× and still sits below the configured `cfg.tol`
/// (`5.9e-9` for Tet10), which keeps the check non-vacuous: it catches a
/// tolerance set too loose, which convergence alone cannot.
const MAX_ACCEPTED_RESIDUAL: f64 = 3e-9;

// ── Committed measurements (6a) ──────────────────────────────────────────

/// **`e₄₀` — the committed Tet4 ν = 0.4 baseline**, `|measured − analytic| /
/// analytic` for the cavity-corner mean at [`CELL_SIZE_DECISION`] under
/// [`LoadRule::Continuum`]. This is the number the rung-6c ν = 0.49
/// threshold `max(2·e₄₀, 0.10)` is computed from, per plan §5 step 6a.
///
/// **What it is not: a "common mesh floor".** The plan frames `e₄₀` as a
/// floor both elements inherit, so that subtracting it isolates the
/// incompressibility penalty. Measured, that framing does not hold — Tet10
/// on identical geometry, load, tolerance and readout reads far better at
/// the same ν, so most of `e₄₀` is *Tet4 element* error that Tet10 does not
/// inherit. `e₄₀` is retained because the plan's pre-registered threshold is
/// defined in terms of it; it is not evidence about Tet10. 6b commits the
/// Tet10 ν = 0.4 anchor, which is the physically meaningful thing to compare
/// a ν = 0.49 Tet10 reading against.
///
/// **It is also harness-defined.** The shipped
/// `iv_5_uniform_passthrough_at_h2_matches_single_shell_lame` prints `0.1493`
/// for the same material at the same refinement. The harnesses differ in the
/// load distribution rule, the loaded-band selection (134 nodes vs 110), the
/// readout node set, and the analytic-evaluation convention; this file does
/// not apportion the gap among those four, and neither should a reader.
/// `e₄₀` moves to 0.0140 under [`LoadRule::EqualSplit`] and 0.1257 under
/// [`LoadRule::Facet`] — see [`tet4_load_rule_ab_at_nu_0_4`], which commits
/// all three so the convention's weight is measurable rather than asserted.
const E40: f64 = 0.0615;

/// [`LoadRule::EqualSplit`] counterpart of [`E40`] — IV-5's equal per-node
/// split generalised to the face-membership band.
const E40_EQUAL_SPLIT: f64 = 0.0140;

/// [`LoadRule::Facet`] counterpart of [`E40`] — true facet areas and true
/// facet normals, the load that is variationally consistent with the
/// *discrete* faceted boundary rather than with the continuum sphere.
const E40_FACET: f64 = 0.1257;

/// **`e₁₀,₄₀` — the committed Tet10 ν = 0.4 anchor** (rung 6b), same harness,
/// same mesh, same load rule, same readout nodes as [`E40`].
///
/// This — not `e₄₀` — is the physically meaningful thing a ν = 0.49 Tet10
/// reading should be compared against, because it isolates *what
/// near-incompressibility costs Tet10* from what the element and mesh cost
/// anyway. `e₄₀` is retained only because the plan's pre-registered threshold
/// is defined in terms of it (see [`E40`]).
///
/// ⚠ **Do not read the smallness of this number as "Tet10 is 0.3 % accurate".**
/// It is small partly because two effects cancel under
/// [`LoadRule::Continuum`]: the element is over-stiff, and the load rule
/// applies +5.3 % more thrust than the discrete boundary warrants (the Tet10
/// figure; it is +7.4 % for Tet4), which pushes the reading soft. [`TET10_NU_0_4_FACET`] is the same element under the
/// variationally consistent load and is **17× larger** (0.0525).
///
/// The honest margin over Tet4 is therefore the `Facet` one: **2.4×**
/// (0.0525 vs 0.1257). The 20× that `Continuum` shows (0.0031 vs 0.0615) is
/// the *same* cancellation this warning names, applied to a ratio — do not
/// quote it as the element-order margin. And note the load rule is not the
/// only convention larger than this anchor: the readout-convention systematic
/// rung 6a retired (a per-node analytic target) was +1.15 % on this node set,
/// **3.7× the anchor itself**. The *element-order comparison* is what this
/// file supports; a standalone absolute accuracy claim for Tet10 is not.
const TET10_NU_0_4: f64 = 0.0031;

/// [`LoadRule::EqualSplit`] counterpart of [`TET10_NU_0_4`] — an equal per-node
/// split *generalised to the face-membership band* (a proxy for IV-5's shipped
/// rule, not that rule itself; see [`LoadRule::EqualSplit`]) on a
/// quadratic surface, where `∫N_corner dA = 0` makes it not merely imprecise
/// but structurally wrong — and the damage is *element-specific*: against
/// [`E40_EQUAL_SPLIT`] (0.0140) this is **17× worse than Tet4 under the same
/// rule**, an inversion of the true element ordering. Under an equal-split
/// load, Tet10 would FAIL the 6b match-or-beat gate — for a reason about the
/// load rule's mismatch with a quadratic surface, not about the element's
/// accuracy. See [`TET10_NU_0_4_EQUAL_SPLIT_SUPPORT`] for the third of that
/// mismatch which is not even about the P2 corner identity. That is the
/// committed evidence for why 6c may not read its verdict off an equal split.
const TET10_NU_0_4_EQUAL_SPLIT: f64 = 0.2405;

/// [`LoadRule::Facet`] counterpart of [`TET10_NU_0_4`] — true facet areas and
/// normals, i.e. the load consistent with the *discrete* boundary.
const TET10_NU_0_4_FACET: f64 = 0.0525;

/// [`LoadRule::EqualSplitSupport`] counterpart — equal split over the midsides
/// only, i.e. [`LoadRule::EqualSplit`] with the P2 corner loading removed.
///
/// The decomposition this constant exists for: of the total departure from
/// `Continuum`, roughly two thirds is diverting ~25 % of the thrust onto
/// corners where `∫N dA = 0` (the P2-specific cause), and the remaining third
/// is equal-vs-area weighting *among the midsides* — an effect with no
/// connection to the corner identity, which Tet4 shows too. Decisive detail:
/// even with corner loading removed entirely this still inverts the element
/// ordering against Tet4's own equal-split reading, so the P2 corner identity
/// is the dominant cause but **not the whole cause**.
const TET10_NU_0_4_EQUAL_SPLIT_SUPPORT: f64 = 0.0761;

// ── Committed measurements (6c — the decision) ───────────────────────────

/// **The rung-6c decision reading** — Tet10 at ν = 0.49 under
/// [`LoadRule::Continuum`], at [`CELL_SIZE_DECISION`].
const TET10_NU_0_49: f64 = 0.0314;

/// The same reading under [`LoadRule::Facet`]. Pre-registration requires
/// ACCEPT under **both** consistent rules.
///
/// ⚠ **Do not read the two arms as independent evidence.** Measured, ~88 % of
/// the Continuum↔Facet gap at ν = 0.49 is the fixed `+5.26 %` radial-thrust
/// excess `Continuum` applies — so the pair contributes roughly `0.006` of
/// genuinely independent signal against a `0.10` bar, and "passes by 3.2× and
/// by 6.8×" is closer to one measurement quoted twice than to two. (At ν = 0.4
/// `Facet` bound harder; at ν = 0.49 it is the looser arm, because
/// `Continuum`'s bias crosses zero and changes sign here.) `Facet` remains the
/// arm to trust for a *level*: it is the load consistent with the discrete
/// boundary the FEM actually solves on.
const TET10_NU_0_49_FACET: f64 = 0.0148;

/// Tet4 at ν = 0.49 under [`LoadRule::Continuum`], for context only — it is
/// no part of the gate. Reported because the locking signal this ladder exists
/// to cure should be *visible*: compare against [`E40`] (0.0615) for the same
/// element two ν apart, and against [`TET10_NU_0_49`] for the two elements at
/// the same ν.
const TET4_NU_0_49: f64 = 0.1083;

/// The h/4 mesh-stability confirmation reading under [`LoadRule::Continuum`]
/// ([`tet10_nu_0_49_h4_mesh_stability_confirmation`], `#[ignore]`d). Committed
/// for the same reason every other measurement here is: an h/4 number reported
/// only in a commit message is exactly the vanished spike this arc keeps
/// having to retract.
const TET10_NU_0_49_H4: f64 = 0.0053;

/// The h/4 confirmation under [`LoadRule::Facet`].
const TET10_NU_0_49_H4_FACET: f64 = 0.0077;

/// Stress Poisson ratio for the oracle-sensitivity probe — `λ/μ = 499`, a
/// **10× stiffer volumetric constraint than [`NU_DECISION`]**. Not part of the
/// gate; it exists to measure whether this oracle can express progressive
/// locking at all (see
/// [`oracle_locking_sensitivity_saturates_beyond_nu_0_49`]).
const NU_STRESS: f64 = 0.499;

/// Tet4 at ν = 0.499, [`LoadRule::Continuum`]. Compare [`TET4_NU_0_49`]
/// (0.1083): a 10× rise in λ/μ leaves it **unchanged, slightly improved**.
const TET4_NU_0_499: f64 = 0.1062;

/// Tet4 at ν = 0.499, [`LoadRule::Facet`]. Compare [`TET4_NU_0_49_FACET`]
/// (0.1677) — same saturation under the unbiased rule.
const TET4_NU_0_499_FACET: f64 = 0.1653;

/// Tet4 at ν = 0.49 under [`LoadRule::Facet`] — the unbiased counterpart of
/// [`TET4_NU_0_49`], needed because the ν = 0.4 → 0.49 rise is 1.76× under
/// `Continuum` but only 1.33× under `Facet`.
const TET4_NU_0_49_FACET: f64 = 0.1677;

/// Tet10 at ν = 0.499, [`LoadRule::Continuum`].
const TET10_NU_0_499: f64 = 0.0191;

/// Tet10 at ν = 0.499, [`LoadRule::Facet`].
const TET10_NU_0_499_FACET: f64 = 0.0124;

// ── The pre-registered decision bars (plan §5 step 6c, fixed at rung 6b) ──

/// ACCEPT ceiling on `|rel_err|`. The plan's `max(2·e₄₀, 0.10)` is 0.123 under
/// `Continuum` and 0.251 under `Facet`, both above 0.10, so the gray-zone rule
/// wins the overlap and this is the operative bar under every rule — see the
/// module docs' pre-registration section, item 2.
const ACCEPT_BAR: f64 = 0.10;

/// Outright-REJECT floor on `|rel_err|`. Between this and [`ACCEPT_BAR`] is
/// the gray zone, which REJECTs unless h/2 → h/4 converges downward through
/// [`ACCEPT_BAR`].
const REJECT_BAR: f64 = 0.20;

/// Acceptance band around a committed measurement, as a **fraction of the
/// committed value** (see [`assert_committed`]).
///
/// The solves are deterministic — re-running reproduces `measured` to all 17
/// digits, in both profiles — so this bounds cross-platform floating-point
/// drift, not run-to-run noise.
///
/// **What 5 % actually pins.** `rel_err` is a difference of two near-equal
/// quantities, so the tightest band here (±2.0e-4 on [`TET10_NU_0_4`]) is
/// **±1.8e-4 relative on the displacement — 0.018 %**. That is the number to
/// judge cross-platform safety against, and it still leaves 5–7 orders over
/// the `κ(K)·ε ≈ 1e-11…1e-9` drift a 3-iteration faer solve can accumulate.
/// The mesher stage contributes none: it is elementary ops plus IEEE-exact
/// `sqrt` over integer connectivity.
///
/// **Relative, not absolute, because a fixed half-width does not scale.** An
/// earlier `±0.005` was 36 % of the smallest constant it then guarded; rung 6b
/// added `TET10_NU_0_4 = 0.0031`, which that band was **161 %** of — it
/// admitted a 2.6× degradation of the file's headline anchor as green, and a
/// measured 5 % Gauss-weight error in the Tet10 element (which moves the
/// reading to −0.0504) sailed through the match-or-beat gate with only this
/// assertion catching it.
const COMMITTED_BAND_FRACTION: f64 = 0.05;

/// Floor on the acceptance band, so a constant near zero does not demand a
/// bit-pin. It **binds** on exactly one constant today: 5 % of
/// [`TET10_NU_0_4`] is 1.55e-4, below this floor, so that anchor's effective
/// band is 6.45 %, not 5 %.
const COMMITTED_BAND_FLOOR: f64 = 2.0e-4;

// ── Closed-form single-shell Lamé oracle ─────────────────────────────────

/// Radial-displacement coefficients for `u_r(r) = A r + B / r²` in a
/// thick-walled hollow sphere under internal pressure with a fixed outer
/// surface. See the module docs for the two boundary conditions.
///
/// Valid only on `r ∈ [R_a, R_b]`. Outside that interval the expression is
/// the analytic continuation of the field into a region with no material —
/// see the module docs' measurement-convention section for the mistake that
/// invites.
struct Lame {
    a: f64,
    b: f64,
}

impl Lame {
    /// Solve the two-condition system in closed form.
    ///
    /// `u_r(R_b) = 0` gives `A = −B / R_b³` directly; substituting into
    /// `σ_rr(R_a) = K A − 4μ B / R_a³ = −p` with `K = 3λ + 2μ` leaves
    /// `−B (K / R_b³ + 4μ / R_a³) = −p`, so `B` follows by division. Both
    /// bracketed terms are strictly positive, so the system cannot go
    /// singular at any admissible radii — no conditioning gate is needed
    /// here (unlike IV-5's 6×6 three-shell assembly).
    fn solve(mu: f64, lambda: f64, r_a: f64, r_b: f64, pressure: f64) -> Self {
        let k = 3.0_f64.mul_add(lambda, 2.0 * mu);
        let b = pressure / (k / r_b.powi(3) + 4.0 * mu / r_a.powi(3));
        Self {
            a: -b / r_b.powi(3),
            b,
        }
    }

    /// Evaluate `u_r(r) = A r + B / r²`, for `r ∈ [R_a, R_b]`.
    fn u_r(&self, r: f64) -> f64 {
        self.a.mul_add(r, self.b / (r * r))
    }
}

/// Lamé's first parameter for a target Poisson ratio at fixed `μ`:
/// `λ = 2μν / (1 − 2ν)`. Mirrors `fbar_locking::lambda_from_nu`.
fn lambda_from_nu(nu: f64) -> f64 {
    2.0 * MU * nu / (1.0 - 2.0 * nu)
}

/// The exact cavity surface area of the analytic sphere, `4πR_a²`.
fn exact_cavity_area() -> f64 {
    4.0 * std::f64::consts::PI * LAYERED_SPHERE_R_CAVITY * LAYERED_SPHERE_R_CAVITY
}

// ── Boundary-face extraction (the band-selection and load primitive) ─────

/// One boundary face: its three corners, plus the three midside nodes on its
/// edges when the mesh is quadratic.
///
/// Midsides are looked up through the canonical
/// [`TET10_EDGE_NODES`] table — the single source of truth every Tet10 piece
/// cites (plan §5 step 1). Reading them by any other rule (the mesher's
/// `(min, max)` dedup key, say) would give each element its own edge→slot
/// permutation and silently load the wrong nodes.
/// [`boundary_face_midsides_sit_at_edge_midpoints`] is the committed gate
/// against exactly that.
struct BoundaryFace {
    corners: [VertexId; 3],
    midsides: Option<[VertexId; 3]>,
}

/// Extract the boundary faces of a tet mesh: the faces referenced by exactly
/// one tet.
///
/// Deliberately re-derived here rather than read from
/// [`Mesh::boundary_faces`]: that cache is three-node corner-only (rung 3a
/// copies it verbatim into [`Tet10Mesh`], and the six-node upgrade is ladder
/// rung 8), and it carries no tet association, so the midside nodes of a
/// boundary face cannot be recovered from it. Faces are keyed by their
/// sorted corner triple — identity only — while the returned corner order
/// and the midside slots come from the element's own local numbering.
fn boundary_faces(mesh: &dyn Mesh) -> Vec<BoundaryFace> {
    /// Local corner triples of the four faces, one per opposite local node.
    /// These are the four 3-subsets of `{0,1,2,3}`, so this is the face set
    /// under any local corner convention.
    const LOCAL_FACES: [[usize; 3]; 4] = [[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2]];

    let mut seen: BTreeMap<[VertexId; 3], (usize, BoundaryFace)> = BTreeMap::new();
    for tet in 0..mesh.n_tets() as u32 {
        let corners_of_tet = mesh.tet_vertices(tet);
        let midsides_of_tet = mesh.tet_midside_nodes(tet);
        for local in LOCAL_FACES {
            let corners = [
                corners_of_tet[local[0]],
                corners_of_tet[local[1]],
                corners_of_tet[local[2]],
            ];
            let midsides = midsides_of_tet.map(|m| {
                let on_edge = |a: usize, b: usize| {
                    let slot = TET10_EDGE_NODES
                        .iter()
                        .position(|&(x, y)| (x == a && y == b) || (x == b && y == a))
                        .expect("every pair of tet corners is a canonical Tet10 edge");
                    m[slot]
                };
                [
                    on_edge(local[0], local[1]),
                    on_edge(local[1], local[2]),
                    on_edge(local[0], local[2]),
                ]
            });
            let mut key = corners;
            key.sort_unstable();
            seen.entry(key)
                .or_insert((0, BoundaryFace { corners, midsides }))
                .0 += 1;
        }
    }
    seen.into_values()
        .filter(|(shared_by, _)| *shared_by == 1)
        .map(|(_, face)| face)
        .collect()
}

// ── Load rules ───────────────────────────────────────────────────────────

/// How the cavity pressure becomes nodal forces.
///
/// The rule is a first-class knob because it is *not* a detail: it moves the
/// measured error by more than the element order does (module docs
/// §"Surface-area normalisation"), and [`tet4_load_rule_ab_at_nu_0_4`]
/// commits the A/B so that stays a measurement rather than a claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LoadRule {
    /// **The harness default.** Consistent shape-function weights
    /// `∫N_i dA` per face, scaled so the total tributary area equals the
    /// exact analytic `4πR_a²`, applied along the exact radial `r̂` at each
    /// node's rest position. Best available approximation of the *continuum*
    /// traction the Lamé oracle assumes, so what remains is element
    /// behaviour rather than the mesher's surface faceting.
    Continuum,
    /// Consistent shape-function weights on the **true facet areas**, along
    /// the **true facet normals**. Variationally consistent with the
    /// discrete faceted boundary the FEM actually solves on — and therefore
    /// the rule under which the Galerkin over-stiffness argument strictly
    /// applies — but it charges the mesher's `O((h/R)²)` faceting deficit to
    /// the element. Measured radial thrust: `Continuum` is a flat 100.53 N for
    /// both elements, while `Facet` is 95.51 N for Tet10 and 93.58 N for Tet4
    /// — so `Continuum`'s excess is **element-dependent**, +5.3 % for Tet10 and
    /// +7.4 % for Tet4, because a corner node accumulates ~6 facet normals
    /// (more directional cancellation) against a midside's exactly 2. Do not
    /// quote a single "~5 %" across both elements.
    Facet,
    /// A **proxy** for IV-5's rule: the exact analytic cavity area split
    /// **equally** over every cavity-surface node, along `r̂`. The control.
    ///
    /// ⚠ Proxy, not IV-5's shipped rule: IV-5 selects its band with the
    /// `0.5·cell_size` per-vertex radial predicate, which loads **134** nodes
    /// here rather than 110 and sweeps in radial-edge midsides that are not on
    /// the cavity surface at all (module docs §"Why a second Lamé file"). This
    /// variant substitutes the harness's face-membership band, isolating the
    /// *distribution* rule. The reading under IV-5's band selection is
    /// unmeasured.
    EqualSplit,
    /// Equal split over only the nodes the consistent rules put load on —
    /// midsides for Tet10, corners for Tet4 — i.e. [`Self::EqualSplit`] with
    /// the P2 corner-loading removed but the equal-vs-area-weighted
    /// distribution kept.
    ///
    /// The counterfactual that decomposes [`Self::EqualSplit`]'s error into
    /// its two independent causes, so the attribution is a measurement rather
    /// than an argument. For Tet4 it is identical to [`Self::EqualSplit`] by
    /// construction (both split over the same corners), which is a free
    /// self-check on the decomposition.
    EqualSplitSupport,
}

// ── Surface conditions: pinned band, consistent load, readout node set ────

/// Everything the solve and the readout need from the mesh surface.
struct SurfaceConditions {
    /// Rest positions of every node (snapshotted before the mesh moves into
    /// the solver).
    rest: Vec<Vec3>,
    /// Outer-surface nodes, fully Dirichlet-pinned.
    pinned: Vec<VertexId>,
    /// Cavity-surface nodes carrying a non-zero load. Under the consistent
    /// rules on a Tet10 mesh this is the midsides only — a P2 corner's
    /// `∫N_i dA` is exactly zero.
    loaded: Vec<VertexId>,
    /// Packed per-loaded-vertex force vectors, in `loaded` order.
    theta: Tensor<f64>,
    /// **Cavity-surface CORNER nodes** — the Saint-Venant readout set, and
    /// the same physical points for Tet4 and Tet10 because enrichment
    /// preserves corner ids and positions bit-identically (module docs
    /// §"Measurement convention"). Under the consistent rules these carry no
    /// load on a quadratic mesh; they still sit on the cavity wall and still
    /// displace.
    cavity_corners: Vec<VertexId>,
}

/// Build the pinned band, the cavity load, and the readout node set from the
/// mesh's boundary faces.
///
/// A face joins the outer band when all three of its corners lie within a
/// half-cell of `R_OUTER`, and the cavity band when they lie within a
/// half-cell of `R_CAVITY`; the sphere's boundary is exactly these two
/// surfaces, which an assertion enforces. The two bands cannot overlap —
/// `R_OUTER − R_CAVITY = 0.06` against a band width of at most `cell_size`.
/// Node membership then follows from the face, so a midside is selected only
/// when it genuinely lies *on* a surface face, never because a radial edge
/// happened to put its midpoint inside a per-vertex radial band.
fn surface_conditions(mesh: &dyn Mesh, cell_size: f64, rule: LoadRule) -> SurfaceConditions {
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let band = 0.5 * cell_size;
    let on_shell = |nodes: &[VertexId; 3], radius: f64| {
        nodes
            .iter()
            .all(|&v| (rest[v as usize].norm() - radius).abs() < band)
    };

    let mut pinned: BTreeSet<VertexId> = BTreeSet::new();
    let mut cavity_corners: BTreeSet<VertexId> = BTreeSet::new();
    let mut cavity_all: BTreeSet<VertexId> = BTreeSet::new();
    // Consistent shape-function weights ∫N_i dA, accumulated per node.
    let mut tributary: BTreeMap<VertexId, f64> = BTreeMap::new();
    // The same weights carried along each face's own outward normal, for
    // `LoadRule::Facet`.
    let mut facet_force: BTreeMap<VertexId, Vec3> = BTreeMap::new();

    for face in boundary_faces(mesh) {
        if on_shell(&face.corners, LAYERED_SPHERE_R_OUTER) {
            pinned.extend(face.corners);
            // A P2 face pinned only at its corners is not u = 0 on the face.
            pinned.extend(face.midsides.into_iter().flatten());
            continue;
        }
        assert!(
            on_shell(&face.corners, LAYERED_SPHERE_R_CAVITY),
            "boundary face {:?} lies on neither the cavity nor the outer shell at cell_size = \
             {cell_size}; the hollow sphere's surface must partition into exactly those two \
             bands, and an unclassified face would leave a silently unloaded / unpinned hole",
            face.corners,
        );

        let p = face.corners.map(|v| rest[v as usize]);
        let cross = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let area = 0.5 * cross.norm();
        // Orient the facet normal away from the cavity centre, matching the
        // direction internal pressure pushes the solid.
        let centroid = (p[0] + p[1] + p[2]) / 3.0;
        let normal = if cross.dot(&centroid) < 0.0 {
            -cross / cross.norm()
        } else {
            cross / cross.norm()
        };

        cavity_corners.extend(face.corners);
        cavity_all.extend(face.corners);
        // The nodes the consistent weight `∫N_i dA` is non-zero on: midsides
        // for a P2 face (`∫N_corner dA = 0` exactly), corners for a P1 face.
        let weighted: [VertexId; 3] = face.midsides.map_or(face.corners, |midsides| {
            cavity_all.extend(midsides);
            midsides
        });
        for v in weighted {
            *tributary.entry(v).or_insert(0.0) += area / 3.0;
            *facet_force.entry(v).or_insert_with(Vec3::zeros) += PRESSURE * (area / 3.0) * normal;
        }
    }

    assert!(
        !cavity_corners.is_empty() && !pinned.is_empty(),
        "the cavity band or the outer band came up empty at cell_size = {cell_size}; the \
         half-cell face-membership tolerance is below the mesher's cut-point density"
    );

    let radial_force = |v: VertexId, magnitude: f64| {
        let p = rest[v as usize];
        // Exact radial direction at the node's rest position — the analytic
        // traction direction. `‖p‖ > 0` on any cavity-band node.
        (p / p.norm()) * magnitude
    };
    let forces: BTreeMap<VertexId, Vec3> = match rule {
        LoadRule::Continuum => {
            let area_scale = exact_cavity_area() / tributary.values().sum::<f64>();
            tributary
                .iter()
                .map(|(&v, &w)| (v, radial_force(v, PRESSURE * w * area_scale)))
                .collect()
        }
        LoadRule::Facet => facet_force,
        LoadRule::EqualSplit => {
            let per_node = exact_cavity_area() / cavity_all.len() as f64;
            cavity_all
                .iter()
                .map(|&v| (v, radial_force(v, PRESSURE * per_node)))
                .collect()
        }
        LoadRule::EqualSplitSupport => {
            let per_node = exact_cavity_area() / tributary.len() as f64;
            tributary
                .keys()
                .map(|&v| (v, radial_force(v, PRESSURE * per_node)))
                .collect()
        }
    };

    let loaded: Vec<VertexId> = forces.keys().copied().collect();
    let mut theta_data = Vec::with_capacity(3 * loaded.len());
    for f in forces.values() {
        theta_data.extend_from_slice(&[f.x, f.y, f.z]);
    }

    SurfaceConditions {
        theta: Tensor::from_slice(&theta_data, &[3 * loaded.len()]),
        rest,
        pinned: pinned.into_iter().collect(),
        loaded,
        cavity_corners: cavity_corners.into_iter().collect(),
    }
}

// ── The harness ──────────────────────────────────────────────────────────

/// Which element the harness solves with. Both run identical geometry,
/// material, load rule, tolerance, and readout node set — only the element
/// order (and hence the mesh's DOF set) differs, so any gap between two
/// readings is an element-order gap.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ElementOrder {
    Tet4,
    Tet10,
}

/// One converged measurement.
struct Reading {
    /// Signed `(measured − analytic) / analytic`. Negative is over-stiff.
    rel_err: f64,
    measured: f64,
    analytic: f64,
    iter_count: usize,
    residual_norm: f64,
    n_nodes: usize,
    n_loaded: usize,
    n_cavity_corners: usize,
    /// Sum of the cavity-corner `VertexId`s — a fingerprint of the readout
    /// *set*, not just its size, so the 6b comparison can assert the two
    /// elements were read on the same nodes rather than merely on the same
    /// number of nodes.
    cavity_corner_fingerprint: u64,
}

/// Build the uniform sphere at `cell_size`, solve one static step with
/// `order` at Poisson ratio `nu` under `rule`, and read the cavity-corner
/// mean against the closed-form Lamé cavity-wall displacement.
///
/// Returns the solver's own failure rather than panicking on it: a
/// near-incompressible solve that stalls must land as a clean REJECT verdict
/// in 6c, not as a test crash (plan §5 step 6c).
fn solve_and_read(
    order: ElementOrder,
    nu: f64,
    cell_size: f64,
    rule: LoadRule,
) -> Result<Reading, Box<SolverFailure>> {
    let lambda = lambda_from_nu(nu);
    // Only the mesh is taken from the scene helper — the harness builds its
    // own boundary conditions and load, because the helper's are Tet4-tuned
    // (per-vertex radial bands, equal-split load; module docs §3).
    let (mesh4, _bc, _initial, _theta) =
        SoftScene::layered_silicone_sphere(MaterialField::uniform(MU, lambda), cell_size, PRESSURE)
            .expect("layered_silicone_sphere should mesh at the canonical cell sizes");

    let (surface, mesh10) = match order {
        ElementOrder::Tet4 => (surface_conditions(&mesh4, cell_size, rule), None),
        ElementOrder::Tet10 => {
            let mesh10 = Tet10Mesh::from_tet4(&mesh4);
            (surface_conditions(&mesh10, cell_size, rule), Some(mesh10))
        }
    };

    let n_dof = 3 * surface.rest.len();
    let mut x_flat = vec![0.0; n_dof];
    for (v, p) in surface.rest.iter().enumerate() {
        x_flat[3 * v] = p.x;
        x_flat[3 * v + 1] = p.y;
        x_flat[3 * v + 2] = p.z;
    }
    let x_prev = Tensor::from_slice(&x_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);

    let bc = BoundaryConditions {
        pinned_vertices: surface.pinned.clone(),
        roller_vertices: Vec::new(),
        loaded_vertices: surface
            .loaded
            .iter()
            .map(|&v| (v, LoadAxis::FullVector))
            .collect(),
    };

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    // L2 to match the solver's L2 `free_residual_norm` — module docs
    // §"The residual tolerance must scale with the load".
    let load_l2 = surface
        .theta
        .as_slice()
        .iter()
        .map(|f| f * f)
        .sum::<f64>()
        .sqrt();
    cfg.tol = TOL_RELATIVE * load_l2;

    let step = match (order, mesh10) {
        (ElementOrder::Tet4, _) => {
            let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
                CpuNewtonSolver::new(Tet4, mesh4, NullContact, cfg, bc);
            solver.try_replay_step(&x_prev, &v_prev, &surface.theta, cfg.dt)
        }
        (ElementOrder::Tet10, Some(mesh10)) => {
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, mesh10, NullContact, cfg, bc);
            solver.try_replay_step(&x_prev, &v_prev, &surface.theta, cfg.dt)
        }
        (ElementOrder::Tet10, None) => unreachable!("the Tet10 arm always builds its mesh"),
    }
    .map_err(Box::new)?;

    let lame = Lame::solve(
        MU,
        lambda,
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_OUTER,
        PRESSURE,
    );
    // Fixed cavity-wall target: `u_r` is only the solution on [R_a, R_b], and
    // most cavity nodes of the inscribed faceted surface sit just inside R_a.
    let analytic = lame.u_r(LAYERED_SPHERE_R_CAVITY);
    let measured_sum: f64 = surface
        .cavity_corners
        .iter()
        .map(|&v| {
            let i = v as usize;
            let moved = Vec3::new(
                step.x_final[3 * i],
                step.x_final[3 * i + 1],
                step.x_final[3 * i + 2],
            );
            moved.norm() - surface.rest[i].norm()
        })
        .sum();
    let measured = measured_sum / surface.cavity_corners.len() as f64;

    Ok(Reading {
        rel_err: (measured - analytic) / analytic,
        measured,
        analytic,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_nodes: surface.rest.len(),
        n_loaded: surface.loaded.len(),
        n_cavity_corners: surface.cavity_corners.len(),
        cavity_corner_fingerprint: surface.cavity_corners.iter().map(|&v| u64::from(v)).sum(),
    })
}

/// Report a reading and assert the checks every configuration must pass
/// regardless of its accuracy verdict: Newton converged in the expected
/// handful of iterations to a genuinely small residual, the cavity inflated
/// outward, and the strain stayed inside the linear-elastic window the Lamé
/// oracle is valid in.
fn report_and_check_physical(label: &str, reading: &Reading) {
    eprintln!(
        "rung-6 {label}: rel_err = {rel:+.4}, measured = {measured:e}, analytic = {analytic:e}, \
         nodes = {nodes}, loaded = {loaded}, cavity corners = {corners}, newton iters = {iters}, \
         residual = {residual:e}",
        rel = reading.rel_err,
        measured = reading.measured,
        analytic = reading.analytic,
        nodes = reading.n_nodes,
        loaded = reading.n_loaded,
        corners = reading.n_cavity_corners,
        iters = reading.iter_count,
        residual = reading.residual_norm,
    );
    assert!(
        reading.iter_count <= MAX_EXPECTED_NEWTON_ITER,
        "{label}: Newton took {iters} iterations against an expected 3 — the solve succeeded, but \
         the convergence regime changed; investigate before widening the bound",
        iters = reading.iter_count,
    );
    assert!(
        reading.residual_norm < MAX_ACCEPTED_RESIDUAL,
        "{label}: converged to residual {r:e}, above the {MAX_ACCEPTED_RESIDUAL:e} ceiling — the \
         load-scaled tolerance accepted a sloppier solve than any measured configuration needs",
        r = reading.residual_norm,
    );
    assert!(
        reading.measured > 0.0,
        "{label}: cavity wall must inflate outward under internal pressure, got {m:e} — a \
         non-positive mean signals a broken load direction or boundary-condition plumbing",
        m = reading.measured,
    );
    assert!(
        reading.measured < 0.01 * LAYERED_SPHERE_R_CAVITY,
        "{label}: cavity inflation {m:e} exceeds 1 % of R_CAVITY — outside the small-strain \
         window where Neo-Hookean reduces to the linear-elastic Lamé oracle",
        m = reading.measured,
    );
}

/// Assert a measurement matches its committed constant, within a band that is
/// a fixed *fraction* of the constant (floored by [`COMMITTED_BAND_FLOOR`]).
fn assert_committed(label: &str, measured: f64, committed: f64) {
    let band = (COMMITTED_BAND_FRACTION * committed).max(COMMITTED_BAND_FLOOR);
    assert!(
        (measured - committed).abs() < band,
        "{label}: committed {committed:.5} but measured {measured:.5} (band ±{band:.5}). \
         Rung-6 thresholds are computed from these numbers — do NOT re-bake a constant to make \
         the test green without first establishing WHY the discretisation error moved (mesher, \
         load rule, measurement convention, or element)."
    );
}

// ── 6a — the committed Tet4 ν = 0.4 baseline ─────────────────────────────

#[test]
fn tet4_baseline_at_nu_0_4_is_committed_e40() {
    // Ladder rung 6a. Nails down `e₄₀`, the Tet4 ν = 0.4 baseline the plan's
    // pre-registered ν = 0.49 threshold `max(2·e₄₀, 0.10)` is computed from —
    // measured on the harness the decision itself runs on, at the mesh the
    // decision itself runs at. Before this commit the plan quoted an
    // uncommitted "0.993" that exists nowhere in the tree.
    let reading = solve_and_read(
        ElementOrder::Tet4,
        NU_BASELINE,
        CELL_SIZE_DECISION,
        LoadRule::Continuum,
    )
    .expect("the ν = 0.4 Tet4 baseline solve converges in ~3 Newton iterations");
    report_and_check_physical("6a Tet4 ν=0.4 @ h/2 (Continuum)", &reading);

    // Empirically the Tet4 reading is over-stiff at every refinement and ν
    // measured (h, h/2, h/4 × ν ∈ {0.4, 0.49, 0.499}). The Galerkin
    // over-stiffness argument points the same way but does not strictly carry
    // it here: that argument wants the load consistent with the *discrete*
    // faceted domain, and `LoadRule::Continuum` deliberately applies more than
    // that — measured +7.4 % of radial thrust for Tet4 (module docs). So this
    // is an empirical regularity worth
    // pinning, not a theorem — a flip means the harness changed character.
    assert!(
        reading.rel_err < 0.0,
        "e₄₀ should be an over-stiff (negative) error; got {rel:+.4}",
        rel = reading.rel_err,
    );
    assert_committed("e₄₀", reading.rel_err.abs(), E40);
}

#[test]
fn tet4_load_rule_ab_at_nu_0_4() {
    // The load rule moves the measured error by more than the element order
    // does, so it gets a committed A/B rather than a docstring claim. This is
    // the re-runnable form of "the equal-split rule is inconsistent for a
    // quadratic surface" — rung 6b extends it to Tet10, where the P2 corner
    // weight is exactly zero and the rule decides the verdict.
    for (rule, committed, label) in [
        (LoadRule::EqualSplit, E40_EQUAL_SPLIT, "EqualSplit"),
        (LoadRule::Facet, E40_FACET, "Facet"),
    ] {
        let reading = solve_and_read(ElementOrder::Tet4, NU_BASELINE, CELL_SIZE_DECISION, rule)
            .expect("the ν = 0.4 Tet4 solve converges under every load rule");
        report_and_check_physical(&format!("6a Tet4 ν=0.4 @ h/2 ({label})"), &reading);
        assert_committed(label, reading.rel_err.abs(), committed);
    }
}

#[test]
fn boundary_face_midsides_sit_at_edge_midpoints() {
    // The one defect class `boundary_faces` can hit silently: returning a
    // midside node that does not belong to the face edge it was filed under.
    // A `TET10_EDGE_NODES` reordering, or an `enrich_tet4_to_tet10` change
    // that assigns local slots from the `(min, max)` dedup key instead of the
    // canonical table, would load and pin the wrong nodes with no other
    // symptom. Straight edges make the check exact: every midside must sit at
    // the arithmetic midpoint of the two corners of its own edge.
    //
    // This is also what keeps the harness's quadratic path exercised in CI
    // independently of the ν = 0.49 solves that land in 6c.
    let (mesh4, _bc, _initial, _theta) = SoftScene::layered_silicone_sphere(
        MaterialField::uniform(MU, lambda_from_nu(NU_BASELINE)),
        CELL_SIZE_DECISION,
        PRESSURE,
    )
    .expect("layered_silicone_sphere should mesh at the canonical cell sizes");
    let mesh10 = Tet10Mesh::from_tet4(&mesh4);
    let rest = mesh10.positions();

    let faces = boundary_faces(&mesh10);
    assert!(
        !faces.is_empty(),
        "the enriched sphere must have a boundary"
    );
    let mut checked = 0_usize;
    for face in &faces {
        let midsides = face
            .midsides
            .expect("an enriched Tet10 mesh surfaces midside nodes on every face");
        // Slot order mirrors `boundary_faces`: edges (c0,c1), (c1,c2), (c0,c2).
        for (slot, (a, b)) in [(0, 1), (1, 2), (0, 2)].into_iter().enumerate() {
            let expected = 0.5 * (rest[face.corners[a] as usize] + rest[face.corners[b] as usize]);
            let actual = rest[midsides[slot] as usize];
            let drift = (actual - expected).norm();
            assert!(
                drift < 1e-12,
                "boundary face {corners:?} slot {slot}: midside node {node} sits {drift:e} from \
                 the midpoint of corners {a} and {b}. The face's edge→midside mapping has \
                 drifted from the canonical TET10_EDGE_NODES table — the load and the pin would \
                 be applied to the wrong nodes.",
                corners = face.corners,
                node = midsides[slot],
            );
            checked += 1;
        }
    }
    eprintln!(
        "rung-6 midside-slot gate: {checked} face-edges checked over {n} boundary faces",
        n = faces.len()
    );
}

// ── 6b — the Tet10 ν = 0.4 match-or-beat anchor ──────────────────────────

/// Solve both elements at ν = 0.4 under `rule` and assert Tet10 match-or-beats
/// Tet4 on the same oracle. Returns the Tet10 reading.
///
/// Both are solved live rather than compared against the committed [`E40`], so
/// the two readings come from one build of one harness. ⚠ That is a weaker
/// guarantee than it looks: it only neutralises a harness change that moves
/// *both* readings equally, and this file's own A/B is a counterexample — a
/// pure load-rule change moves Tet4 by +0.047 and Tet10 by +0.244 and flips
/// the ordering. Still the right design; not a shield against harness error.
fn assert_tet10_matches_or_beats_tet4(rule: LoadRule, label: &str) -> Reading {
    let tet4 = solve_and_read(ElementOrder::Tet4, NU_BASELINE, CELL_SIZE_DECISION, rule)
        .expect("the ν = 0.4 Tet4 solve converges");
    let tet10 = solve_and_read(ElementOrder::Tet10, NU_BASELINE, CELL_SIZE_DECISION, rule)
        .expect("the ν = 0.4 Tet10 solve converges");
    report_and_check_physical(&format!("6b Tet4 ν=0.4 @ h/2 ({label})"), &tet4);
    report_and_check_physical(&format!("6b Tet10 ν=0.4 @ h/2 ({label})"), &tet10);

    // The Tet10 mesh must really be the enriched one. A bare `>` would pass at
    // one extra node; enrichment adds a midpoint per element edge, so the true
    // factor is ~2.8× (13,336 vs 4,682).
    assert!(
        tet10.n_nodes > 2 * tet4.n_nodes,
        "6b ({label}): the Tet10 mesh has {n10} nodes against Tet4's {n4} — enrichment did not \
         happen (expected ~2.8×), so this comparison is vacuous",
        n10 = tet10.n_nodes,
        n4 = tet4.n_nodes,
    );
    // The same readout SET, not merely the same count: enrichment preserves
    // corner ids, so the id fingerprints must match too.
    assert_eq!(
        (tet10.n_cavity_corners, tet10.cavity_corner_fingerprint),
        (tet4.n_cavity_corners, tet4.cavity_corner_fingerprint),
        "6b ({label}): the two elements must be read on the SAME cavity-corner nodes; the counts \
         or id fingerprints differ, so the readout set drifted between the two solves"
    );
    assert!(
        tet10.rel_err.abs() <= tet4.rel_err.abs(),
        "6b MISS ({label}) — Tet10 ({rel10:+.4}) is worse than Tet4 ({rel4:+.4}) at \
         ν = {NU_BASELINE}, where neither element locks. This localizes to the FORWARD path \
         (rung-4 multi-Gauss-point assembly, rung-3b unpin trio, or enrichment), NOT to \
         near-incompressibility — fix it before reading anything from the ν = 0.49 gate.",
        rel10 = tet10.rel_err,
        rel4 = tet4.rel_err,
    );
    tet10
}

#[test]
fn tet10_at_nu_0_4_matches_or_beats_tet4() {
    // Ladder rung 6b, and deliberately a BUG-LOCALIZER rather than an accuracy
    // claim. On the same mesh the P2 space *contains* the P1 space, so Tet10's
    // Galerkin solution is no worse in energy norm, and at a compressible ν —
    // where neither element locks — it must not read worse. (Nesting is
    // suggestive rather than a strict guarantee on this pointwise readout,
    // because the two discrete load functionals differ — 110 corner loads
    // versus 324 midside loads — which is exactly why the gate is empirical.)
    // A miss here localizes to the forward path, not to incompressibility,
    // which is what lets 6c attribute a bad ν = 0.49 reading to locking.
    //
    // ★ MEASURED, because the obvious rationale for the second arm is wrong.
    // Match-or-beat is a LOOSE bar under both rules, and adding `Facet` does
    // not tighten it: a uniformly 5 %-over-stiff Tet10 element reads -0.0504
    // against Tet4's -0.0615 (Continuum) and -0.0974 against -0.1257 (Facet),
    // and PASSES BOTH. In element-stiffness scale `s` the arms fire outside
    // [0.939, 1.062] and [0.842, 1.084] respectively — Facet's interval
    // strictly CONTAINS Continuum's, so the conjunction is exactly the
    // Continuum gate. The bar is Tet4's own error, and under `Facet` that bar
    // is looser, not tighter.
    //
    // What actually catches a uniform stiffness error is `assert_committed`
    // below (Continuum's band pins `s` to ±0.02 %, Facet's to ±0.28 %). The
    // Facet arm earns its place differently: it is a second, independently
    // conditioned load functional — different node weights AND different
    // directions — so it exercises the ordering claim under a load the
    // `Continuum` normalisation never touches, and it is the un-cancelled read
    // that carries the honest margin (2.4×, not 20×).
    let continuum = assert_tet10_matches_or_beats_tet4(LoadRule::Continuum, "Continuum");
    let facet = assert_tet10_matches_or_beats_tet4(LoadRule::Facet, "Facet");

    // Over-stiff (negative) is the pinned empirical regularity at ν = 0.4 — 6a
    // asserts it for Tet4. Without this, a ± band on an anchor as small as
    // `TET10_NU_0_4` admits either sign, and that constant's cancellation
    // argument is a SIGNED argument.
    for (label, reading) in [("Continuum", &continuum), ("Facet", &facet)] {
        assert!(
            reading.rel_err < 0.0,
            "6b ({label}): Tet10 at ν = 0.4 should read over-stiff (negative), got {rel:+.4}",
            rel = reading.rel_err,
        );
    }
    assert_committed("e₁₀,₄₀ (Continuum)", continuum.rel_err.abs(), TET10_NU_0_4);
    assert_committed("e₁₀,₄₀ (Facet)", facet.rel_err.abs(), TET10_NU_0_4_FACET);
}

#[test]
fn tet10_equal_split_load_inverts_the_element_ordering() {
    // On a QUADRATIC surface the equal-split rule is not merely imprecise but
    // structurally wrong — `∫N_corner dA = 0` exactly, so it loads nodes the
    // consistent formulation says carry none. Committing the size of that
    // error, and its DECOMPOSITION, is what licenses 6c to reject the shipped
    // load rule on evidence rather than theory.
    //
    // ★ The decomposition matters because the single-cause story is only
    // partly true. `EqualSplitSupport` drops the corner loading but keeps
    // equal-vs-area weighting among the midsides; the error it still shows is
    // the part that has nothing to do with the P2 corner identity — and it
    // inverts the ordering on its own.
    let tet4_equal = solve_and_read(
        ElementOrder::Tet4,
        NU_BASELINE,
        CELL_SIZE_DECISION,
        LoadRule::EqualSplit,
    )
    .expect("the ν = 0.4 Tet4 EqualSplit solve converges");
    report_and_check_physical("6b Tet4 ν=0.4 @ h/2 (EqualSplit)", &tet4_equal);

    let mut tet10_readings = Vec::new();
    for (rule, committed, label) in [
        (LoadRule::EqualSplit, TET10_NU_0_4_EQUAL_SPLIT, "EqualSplit"),
        (
            LoadRule::EqualSplitSupport,
            TET10_NU_0_4_EQUAL_SPLIT_SUPPORT,
            "EqualSplitSupport",
        ),
    ] {
        let reading = solve_and_read(ElementOrder::Tet10, NU_BASELINE, CELL_SIZE_DECISION, rule)
            .expect("the ν = 0.4 Tet10 solve converges under every load rule");
        report_and_check_physical(&format!("6b Tet10 ν=0.4 @ h/2 ({label})"), &reading);
        assert_committed(&format!("Tet10 {label}"), reading.rel_err.abs(), committed);
        // The sign IS the finding here: at ν = 0.4 every consistent reading is
        // negative (over-stiff), so a positive one marks an INVERSION rather
        // than a magnitude. (At ν = 0.49 the consistent rules no longer agree
        // on sign — see `TET10_NU_0_49` — so this reasoning is scoped to the
        // ν = 0.4 comparison it is written for.)
        assert!(
            reading.rel_err > 0.0,
            "6b: Tet10 under {label} should read over-SOFT (positive) — that sign is what makes \
             it an inversion of the element ordering; got {rel:+.4}",
            rel = reading.rel_err,
        );
        assert!(
            reading.rel_err.abs() > tet4_equal.rel_err.abs(),
            "6b: under {label} Tet10 ({rel10:+.4}) should be WORSE than Tet4 under the same \
             equal-split rule ({rel4:+.4}) — that inversion is the committed evidence that the \
             load rule, not the element, would decide such a verdict",
            rel10 = reading.rel_err,
            rel4 = tet4_equal.rel_err,
        );
        tet10_readings.push(reading.rel_err);
    }

    // Tet4's consistent support IS its corner set, so the two equal-split
    // variants must coincide exactly for Tet4 — a free self-check that
    // `EqualSplitSupport` isolates what it claims to.
    let tet4_support = solve_and_read(
        ElementOrder::Tet4,
        NU_BASELINE,
        CELL_SIZE_DECISION,
        LoadRule::EqualSplitSupport,
    )
    .expect("the ν = 0.4 Tet4 EqualSplitSupport solve converges");
    assert_eq!(
        tet4_support.rel_err.to_bits(),
        tet4_equal.rel_err.to_bits(),
        "6b: for Tet4 the consistent support IS the corner set, so EqualSplitSupport must be \
         bit-identical to EqualSplit; a difference means the variant is not isolating the P2 \
         corner-loading effect it exists to isolate"
    );
    eprintln!(
        "rung-6 6b equal-split decomposition: Tet10 Continuum {c:+.4} -> EqualSplitSupport \
         {s:+.4} (equal-vs-area weighting among midsides alone) -> EqualSplit {e:+.4} (plus P2 \
         corner loading); Tet4 EqualSplit {t4:+.4}",
        c = -TET10_NU_0_4,
        s = tet10_readings[1],
        e = tet10_readings[0],
        t4 = tet4_equal.rel_err,
    );
}

// ── 6c — the ν = 0.49 ACCEPT/REJECT decision gate ────────────────────────

/// The three-way verdict of plan §5 step 6c.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Verdict {
    /// Converged, and `|rel_err| ≤ ACCEPT_BAR`.
    Accept,
    /// `ACCEPT_BAR < |rel_err| ≤ REJECT_BAR`. Defaults to REJECT unless
    /// h/2 → h/4 converges downward through [`ACCEPT_BAR`]
    /// ([`tet10_nu_0_49_h4_mesh_stability_confirmation`]).
    Gray,
    /// `|rel_err| > REJECT_BAR`, or Newton stalled → Taylor-Hood P2-P1.
    Reject,
}

/// Classify a converged reading against the pre-registered bars.
///
/// **On `|rel_err|`, never the signed value** — locking drives the reading
/// *more negative*, so a signed comparison would ACCEPT a fully locked element
/// trivially (module docs, pre-registration item "Every threshold below is on
/// `|rel_err|`"). A solve that fails to converge never reaches this function:
/// `solve_and_read` returns the `SolverFailure`, which the gate maps to
/// [`Verdict::Reject`] directly.
fn classify(rel_err: f64) -> Verdict {
    let magnitude = rel_err.abs();
    if magnitude <= ACCEPT_BAR {
        Verdict::Accept
    } else if magnitude <= REJECT_BAR {
        Verdict::Gray
    } else {
        Verdict::Reject
    }
}

/// ★★ **What an ACCEPT here does NOT license.** Stated on the gate itself, not
/// only in a commit message, because this arc's standing failure mode is
/// exactly that caveats survive in prose and vanish from the artifact:
///
/// - **One oracle, and a locking-INSENSITIVE one.** Radial inflation of a
///   thick-walled sphere with a fixed outer skin — not contact, not a
///   distorted or sliver mesh. And measured, this oracle cannot express
///   progressive locking at all: Tet4's error saturates across a 10× rise in
///   λ/μ (`oracle_locking_sensitivity_saturates_beyond_nu_0_49`), so it can
///   certify accuracy here but not a locking cure. **The bending evidence
///   lives in `tet10_bending_locking.rs`**, on a cantilever where the control
///   genuinely collapses (Tet4 ν-ratio 0.2341 against a physical 0.8725) and
///   Tet10 does not (0.8058). Read the two files together; neither alone
///   settles ν → 0.5.
/// - **★ A mean-displacement readout is blind to the failure Taylor-Hood
///   actually exists to fix.** The verdict is a Saint-Venant mean radial
///   displacement over 110 corner nodes. Pure-displacement elements at ν → 0.5
///   characteristically deliver acceptable *displacements* alongside a
///   spurious oscillatory (checkerboard) *pressure* field, and suppressing
///   that via inf-sup/LBB stability is the primary reason mixed P2-P1 exists.
///   This harness never evaluates pressure or stress. So the verdict is
///   "accurate enough in mean displacement at ν = 0.49", **not** "the pressure
///   field is sound" — if a later rung needs trustworthy pressures near
///   incompressibility, that question is still open.
/// - **Static, therefore mass-blind** (`STATIC_DT`): says nothing about the
///   Tet10 HRZ lumped mass, which is rung 7's dynamics gate.
/// - **Forward path only.** The differentiable path stays `N == 4`-guarded
///   until rung 7; no adjoint channel is exercised here.
/// - **ν = 0.49, not 0.499.** Untested under this harness.
/// - **Demand #1 is untouched** — the ~9 % Tet4 curved-contact-patch floor
///   (#676) is rung 6d, deferred, and no reading here bears on it.
/// - **The two consistent rules disagree in SIGN at ν = 0.49** (`Continuum`
///   +0.0314 over-soft, `Facet` −0.0148 over-stiff). Both are inside the bar,
///   which is what the gate reads, but the *level* is what is being certified,
///   not a claim about the direction of the residual error.
#[test]
fn tet10_at_nu_0_49_decision_gate() {
    // ★★ LADDER RUNG 6c — THE DECISION. Is pure-displacement Tet10 accurate
    // enough at Ecoflex's ν ≈ 0.49, or does near-incompressibility need a
    // mixed Taylor-Hood P2-P1 formulation on top (a redesign, deferred)?
    //
    // Every threshold that DECIDES the verdict was pre-registered at rung 6b,
    // while only ν = 0.4 data existed — see the module docs' pre-registration
    // section; this commit added lines to the file and deleted none of it. In
    // particular the gate requires ACCEPT under BOTH consistent load rules,
    // because 6b measured that the rule choice moves the anchor 17× and would
    // otherwise be the largest free lever on the verdict. (The
    // `assert_committed` bands and the Tet10-beats-Tet4 check below are NOT
    // pre-registered — they were written against measured ν = 0.49 numbers.
    // Both are tightenings on top of the verdict rule, never a route to
    // ACCEPT, but they were not fixed in advance and are not described as if
    // they were.)
    //
    // Context reading first: Tet4 at the same ν, which is not part of the gate
    // but should make the locking signal visible.
    let tet4 = solve_and_read(
        ElementOrder::Tet4,
        NU_DECISION,
        CELL_SIZE_DECISION,
        LoadRule::Continuum,
    )
    .expect("the ν = 0.49 Tet4 solve converges (context reading, not the gate)");
    report_and_check_physical("6c Tet4 ν=0.49 @ h/2 (Continuum) [context]", &tet4);
    assert_committed("Tet4 ν=0.49", tet4.rel_err.abs(), TET4_NU_0_49);

    let mut verdicts = Vec::new();
    for (rule, label) in [
        (LoadRule::Continuum, "Continuum"),
        (LoadRule::Facet, "Facet"),
    ] {
        // A stall is a clean REJECT that flows through the same assertion as
        // an accuracy miss — not a crash and not a separate code path.
        // `try_replay_step` surfaces `NewtonIterCap` / `ArmijoStall` as
        // errors, and plan §6 expected convergence to be the likely failure
        // mode at ν → 0.5; §5 step 6c routes both to Taylor-Hood alike.
        let (verdict, rel_err) =
            match solve_and_read(ElementOrder::Tet10, NU_DECISION, CELL_SIZE_DECISION, rule) {
                Ok(reading) => {
                    report_and_check_physical(
                        &format!("6c Tet10 ν=0.49 @ h/2 ({label})"),
                        &reading,
                    );
                    // ★ `assert_committed` deliberately does NOT run here. Its
                    // band is ±0.0016, ~60× tighter than the distance from the
                    // reading to `ACCEPT_BAR`, so asserting it first would make
                    // `Verdict::Gray` and `Verdict::Reject` unreachable on the
                    // accuracy path: any reading bad enough to REJECT would
                    // panic as a "committed constant moved" regression long
                    // before the decision logic ran, and the Taylor-Hood
                    // routing message could never print. The verdict is
                    // asserted first, below; the regression pin follows it.
                    (classify(reading.rel_err), Some(reading.rel_err))
                }
                Err(failure) => {
                    eprintln!(
                        "rung-6c ({label}): the solve did not converge at ν = {NU_DECISION} \
                         ({failure:?}) — per plan §5 step 6c a stall REJECTs exactly as an \
                         accuracy miss does."
                    );
                    (Verdict::Reject, None)
                }
            };
        eprintln!(
            "rung-6c VERDICT ({label}): {mag} vs ACCEPT ≤ {ACCEPT_BAR}, REJECT > {REJECT_BAR} \
             → {verdict:?}",
            mag = rel_err.map_or_else(
                || "did not converge".to_owned(),
                |r| format!("|rel_err| = {m:.4}", m = r.abs())
            ),
        );
        verdicts.push((label, verdict, rel_err));
    }

    // The gate: ACCEPT under BOTH consistent rules, or the verdict is REJECT.
    // This runs BEFORE the committed-constant pins so a genuine Gray/Reject
    // reading reports the DECISION, not a regression-band mismatch.
    for (label, verdict, rel_err) in &verdicts {
        assert_eq!(
            *verdict,
            Verdict::Accept,
            "6c VERDICT = {verdict:?} under {label} ({mag}). Pre-registration requires ACCEPT \
             under BOTH consistent load rules; a Gray verdict REJECTs unless the h/4 \
             confirmation converges downward through {ACCEPT_BAR} \
             (`tet10_nu_0_49_h4_mesh_stability_confirmation`, `--ignored`), and a Reject verdict \
             routes to Taylor-Hood P2-P1 per plan §5 step 6c.",
            mag = rel_err.map_or_else(
                || "did not converge".to_owned(),
                |r| format!("|rel_err| = {m:.4}", m = r.abs())
            ),
        );
    }

    // Regression pins, secondary to the verdict: these keep the numbers from
    // drifting silently, but they are NOT the gate and are not pre-registered.
    // Signs are pinned per-rule because at ν = 0.49 the two consistent rules
    // DISAGREE about the direction of the residual error — `Continuum` reads
    // over-soft, `Facet` over-stiff — and that disagreement is itself a
    // finding (see `TET10_NU_0_49_FACET`). 6a and 6b each pin a sign for the
    // same reason: a ± band on a small anchor otherwise admits either one.
    for (label, expected_positive, committed) in [
        ("Continuum", true, TET10_NU_0_49),
        ("Facet", false, TET10_NU_0_49_FACET),
    ] {
        let rel_err = verdicts
            .iter()
            .find(|(l, _, _)| *l == label)
            .and_then(|(_, _, r)| *r)
            .expect("both arms converged (asserted ACCEPT above)");
        assert_eq!(
            rel_err > 0.0,
            expected_positive,
            "6c ({label}): expected the reading to be {dir}, got {rel_err:+.4}. The two \
             consistent rules disagree in sign at ν = 0.49 by design; a flip here means the \
             load convention or the element changed character.",
            dir = if expected_positive {
                "over-soft (positive)"
            } else {
                "over-stiff (negative)"
            },
        );
        assert_committed(&format!("Tet10 ν=0.49 {label}"), rel_err.abs(), committed);
    }

    // Demand #2 is an ELEMENT-ORDER claim, so the ν = 0.49 gate should also
    // show Tet10 beating Tet4 at the ν where Tet4 locks — the same
    // match-or-beat logic rung 6b applied at ν = 0.4.
    let continuum = verdicts[0]
        .2
        .expect("the Continuum arm converged (asserted ACCEPT above)");
    assert!(
        continuum.abs() < tet4.rel_err.abs(),
        "6c: Tet10 ({rel10:+.4}) must beat Tet4 ({rel4:+.4}) at ν = {NU_DECISION}; if it does \
         not, the ν = 0.49 reading is not an element-order result",
        rel10 = continuum,
        rel4 = tet4.rel_err,
    );
}

#[ignore = "pre-push one-shot, NEVER CI — the h/4 Tet10 mesh is 94,710 nodes / 284k DOF and \
            measured 7.16 GB peak RSS over 500 s for the two solves, above what a CI runner \
            has. Run with `cargo test --release -p sim-soft --test tet10_lame_decision -- \
            --ignored tet10_nu_0_49_h4`"]
#[test]
fn tet10_nu_0_49_h4_mesh_stability_confirmation() {
    // The mesh-stability half of the 6c decision, and the committed artifact
    // for the gray-zone escape clause. Plan §5 step 6c pins the decision at
    // h/2 and this at h/4 as a one-shot; before this rung it existed only as
    // an instruction, so its output was exactly the kind of vanished spike
    // number the ladder keeps having to retract.
    //
    // Pre-registered criterion (module docs, item 3): under the SAME rule, the
    // h/4 reading must be both ≤ ACCEPT_BAR and strictly below the h/2
    // reading. At an h/2 verdict of ACCEPT this is confirmation rather than an
    // escape, but the assertion is identical either way.
    //
    // ★ h/2 is solved LIVE here rather than read from its committed constant.
    // The pre-registered clause says "strictly below the h/2 reading", and
    // `assert_committed` admits ±5 %, so comparing against the constant would
    // leave a window where a drifted h/2 and a rising h/4 both pass while the
    // sequence is actually diverging — on the one clause that exists to stop
    // an unearned ACCEPT. The extra h/2 solve costs ~3 s against h/4's ~250 s.
    for (rule, h4_committed, label) in [
        (LoadRule::Continuum, TET10_NU_0_49_H4, "Continuum"),
        (LoadRule::Facet, TET10_NU_0_49_H4_FACET, "Facet"),
    ] {
        let h2 = solve_and_read(ElementOrder::Tet10, NU_DECISION, CELL_SIZE_DECISION, rule)
            .expect("the ν = 0.49 Tet10 h/2 solve converges");
        let h4 = solve_and_read(
            ElementOrder::Tet10,
            NU_DECISION,
            CELL_SIZE_CONFIRMATION,
            rule,
        )
        .expect("the ν = 0.49 Tet10 h/4 solve converges");
        report_and_check_physical(&format!("6c Tet10 ν=0.49 @ h/4 ({label})"), &h4);
        assert_committed(
            &format!("Tet10 ν=0.49 h/4 {label}"),
            h4.rel_err.abs(),
            h4_committed,
        );
        assert!(
            h4.rel_err.abs() <= ACCEPT_BAR,
            "6c h/4 ({label}): |rel_err| = {mag:.4} exceeds the {ACCEPT_BAR} bar — refinement \
             does not carry the reading through the ACCEPT threshold",
            mag = h4.rel_err.abs(),
        );
        assert!(
            h4.rel_err.abs() < h2.rel_err.abs(),
            "6c h/4 ({label}): |rel_err| = {mag:.4} is not strictly below the live h/2 reading \
             {h2mag:.4} — the error is not converging downward under refinement, so the h/2 \
             verdict is not mesh-stable",
            mag = h4.rel_err.abs(),
            h2mag = h2.rel_err.abs(),
        );
        eprintln!(
            "rung-6c h/4 confirmation ({label}): h/2 {h2mag:.4} → h/4 {mag:.4} (converging \
             downward through {ACCEPT_BAR})",
            h2mag = h2.rel_err.abs(),
            mag = h4.rel_err.abs(),
        );
    }
}

// ── Oracle-sensitivity probe: what this oracle can and cannot certify ────

#[test]
fn oracle_locking_sensitivity_saturates_beyond_nu_0_49() {
    // ★★ THE HONEST LIMIT ON RUNG 6c, committed rather than argued.
    //
    // The 6c verdict was originally justified in part by "Tet4's error nearly
    // doubles from ν = 0.4 to ν = 0.49, so the volumetric locking this ladder
    // exists to cure is real and present in this oracle." Extending the same
    // sweep one step FALSIFIES that warrant, and this test pins the
    // falsification so it cannot be quietly forgotten.
    //
    // Volumetric locking is UNBOUNDED as ν → 0.5: the error should grow
    // without limit as λ/μ grows. Measured here, going ν = 0.49 → 0.499 raises
    // λ/μ from 49 to 499 — a 10× stiffer volumetric constraint — and Tet4's
    // error does not grow at all. It SATURATES, and slightly improves.
    //
    // Why that is physically reasonable, and why it matters: this oracle is a
    // radially symmetric inflation, whose exact incompressible solution is the
    // pure volume-preserving mode `u_r ∝ 1/r²`. A displacement field with one
    // spatial degree of freedom does not stress the deviatoric-vs-volumetric
    // competition that makes locking severe in BENDING. Corroborating: the
    // oracle's own docstring predicts "convergence collapse" as ν → 0.5, and
    // Newton converges in 3 iterations at every ν tried, including 0.499.
    //
    // ★ Consequence for the verdict. Rung 6c legitimately certifies that Tet10
    // is accurate in mean displacement at ν = 0.49 on this oracle, and that it
    // beats Tet4 there by a wide margin. It does NOT certify that Tet10 cures
    // volumetric locking, because this oracle cannot make Tet4 progressively
    // lock. A locking-sensitive claim needs a bending oracle — `fbar_locking`
    // is the in-tree one, where plain Tet4 at ν = 0.49 genuinely collapses.
    let mut tet4 = Vec::new();
    for (nu, rule, committed, label) in [
        (
            NU_DECISION,
            LoadRule::Continuum,
            TET4_NU_0_49,
            "0.49 Continuum",
        ),
        (
            NU_STRESS,
            LoadRule::Continuum,
            TET4_NU_0_499,
            "0.499 Continuum",
        ),
        (
            NU_DECISION,
            LoadRule::Facet,
            TET4_NU_0_49_FACET,
            "0.49 Facet",
        ),
        (
            NU_STRESS,
            LoadRule::Facet,
            TET4_NU_0_499_FACET,
            "0.499 Facet",
        ),
    ] {
        let reading = solve_and_read(ElementOrder::Tet4, nu, CELL_SIZE_DECISION, rule)
            .expect("the Tet4 sensitivity solves converge");
        report_and_check_physical(&format!("probe Tet4 ν={label} @ h/2"), &reading);
        assert_committed(&format!("Tet4 ν={label}"), reading.rel_err.abs(), committed);
        tet4.push(reading.rel_err.abs());
    }

    // The falsification itself: 10× the volumetric stiffness, no growth.
    for (rule_label, at_49, at_499) in
        [("Continuum", tet4[0], tet4[1]), ("Facet", tet4[2], tet4[3])]
    {
        assert!(
            at_499 <= at_49,
            "probe ({rule_label}): Tet4 grew from {at_49:.4} at ν = {NU_DECISION} to \
             {at_499:.4} at ν = {NU_STRESS}. If this oracle HAS become \
             locking-sensitive, that is a material change to what rung 6c certifies — \
             revisit the 6c caveat block, do not simply widen this assertion.",
        );
        eprintln!(
            "probe locking sensitivity ({rule_label}): Tet4 |rel_err| {at_49:.4} at λ/μ=49 → \
             {at_499:.4} at λ/μ=499 — SATURATED, so this oracle does not express progressive \
             volumetric locking"
        );
    }

    // Tet10 at the same stress point, for completeness: still well inside the
    // decision bar, which is the one thing the ν = 0.499 step does support.
    for (rule, committed, label) in [
        (LoadRule::Continuum, TET10_NU_0_499, "Continuum"),
        (LoadRule::Facet, TET10_NU_0_499_FACET, "Facet"),
    ] {
        let reading = solve_and_read(ElementOrder::Tet10, NU_STRESS, CELL_SIZE_DECISION, rule)
            .expect("the Tet10 ν = 0.499 solves converge");
        report_and_check_physical(&format!("probe Tet10 ν=0.499 @ h/2 ({label})"), &reading);
        assert_committed(
            &format!("Tet10 ν=0.499 {label}"),
            reading.rel_err.abs(),
            committed,
        );
        assert!(
            reading.rel_err.abs() <= ACCEPT_BAR,
            "probe: Tet10 at ν = {NU_STRESS} ({label}) reads {mag:.4}, outside the \
             {ACCEPT_BAR} decision bar — the ν = 0.49 ACCEPT does not extend as far as this \
             file's caveats claim",
            mag = reading.rel_err.abs(),
        );
    }
}
