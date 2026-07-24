//! Tet10 ladder rung 6 — the Lamé accuracy-decision harness.
//!
//! `docs/SIM_SOFT_TET10_PLAN.md` §4 + §5 step 6. This file exists to answer
//! one pre-registered question: **is pure-displacement Tet10 accurate enough
//! at ν = 0.49 (Ecoflex 00-30), or does near-incompressibility need a mixed
//! Taylor-Hood P2-P1 formulation on top?** The rung-6 ladder is:
//!
//! - **6a (this commit)** — commit the *real* Tet4 ν = 0.4 baseline `e₄₀` at
//!   the decision mesh. The plan's prior "Tet4 → 0.993 at ν = 0.4" figure was
//!   an uncommitted spike that exists nowhere in the tree; the ν = 0.49
//!   threshold subtracts against a measured number or against nothing.
//! - **6b** — Tet10 at ν = 0.4 must match-or-beat Tet4 on the same harness.
//!   A miss *there* localizes to a forward/assembly bug, not to an
//!   incompressibility limit.
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
//! rel_err ≤ max(2·e₄₀, 0.10)`, `REJECT if > 0.20 or Newton stalls`, gray
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
//! - Note the plan's three rules **overlap** on `[0.10, max(2·e₄₀, 0.10)]`
//!   for any `e₄₀ > 0.05`: a reading of 0.12 satisfies ACCEPT *and* falls in
//!   the gray zone that defaults to REJECT. 6c must pre-register a
//!   tie-break before it reads a ν = 0.49 number, not after.
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
//! [`LoadRule::Facet`]** — a 6.4-point swing, larger than the Tet4→Tet10
//! element-order gap. [`tet4_load_rule_ab_at_nu_0_4`] commits all three rules
//! so the convention factor stays visible and re-runnable. The *ordering* of
//! the elements is robust to the choice; the *absolute* number `e₄₀` is not.
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
/// the module docs' tolerance section. Chosen two orders above the highest
/// measured residual floor (Tet10 at ν = 0.49, `1.43e-10`) and nine orders
/// below the load itself.
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
/// it. All measured configurations land at `1.4e-10` or below.
const MAX_ACCEPTED_RESIDUAL: f64 = 1e-9;

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

/// Half-width of the acceptance band around a committed measurement. The
/// solves are deterministic (the assembler is serial by design and the
/// mesher is gated for run-to-run bit-equality by IV-5), so this bounds
/// cross-platform floating-point drift, not run-to-run noise: re-running the
/// baseline reproduces `measured` to all 17 digits, in both profiles. Sized
/// against the effects the file actually discriminates — the load rule spans
/// 0.0140 to 0.1257, and h → h/2 moves the Tet4 reading by several points —
/// so `0.005` separates those without pinning a cross-platform ULP.
const COMMITTED_BAND: f64 = 0.005;

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
    /// the element. Measured, it applies ~5 % less total load than
    /// [`Self::Continuum`] (4.8 % by force magnitude, 5.0 % by radial thrust).
    Facet,
    /// IV-5's rule: the exact analytic cavity area split **equally** over
    /// every cavity-surface node, along `r̂`. The control. Inconsistent for a
    /// quadratic surface, where `∫N_corner dA = 0` exactly.
    EqualSplit,
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
    // Rung 6a lands the harness carrying only the Tet4 measurements. `Tet10`
    // is solved by the 6b match-or-beat anchor and the 6c decision gate,
    // which land in their own PRs on top of this one. The Tet10 *mesh* path
    // is not unexercised in the meantime:
    // `boundary_face_midsides_sit_at_edge_midpoints` drives `boundary_faces`
    // over a real `Tet10Mesh`.
    #[allow(dead_code)]
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

/// Assert a measurement matches its committed constant.
fn assert_committed(label: &str, measured: f64, committed: f64) {
    assert!(
        (measured - committed).abs() < COMMITTED_BAND,
        "{label}: committed {committed:.4} but measured {measured:.4} (band ±{COMMITTED_BAND}). \
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
    // faceted domain, and `LoadRule::Continuum` deliberately applies ~5 %
    // more than that (module docs). So this is an empirical regularity worth
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
    // quadratic surface" — 6b/6c extend it to Tet10, where the P2 corner
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
    // while the Tet10 *solves* wait for 6b/6c.
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
