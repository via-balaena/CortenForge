//! Renovation item 3 — does `Tet10Mesh<Yeoh>` converge under the IPC face
//! barrier, on a flat patch, a curved one, and a cavity closing around a probe?
//!
//! ⭐ Step 0 asked only the first of those. The enveloping cell arrived later
//! and is the one whose CONTACT TOPOLOGY `insertion_sim` has — a closed patch
//! that engages at once, not a Hertzian front. ⚠ It is not the same BODY:
//! `insertion_sim`'s sleeve is a sock over a capsule with an open mouth, and
//! this shell is sealed (see "what this cannot see"). The sections below are
//! written in the order the cells were added, because each answers a question
//! the previous one raised.
//!
//! Item 1 (`6dc7ee03`) made `Tet10Mesh<M>` generic, so `Tet10Mesh<Yeoh>` is
//! **constructible**. Constructible is not converged: when this file was
//! written the *only* Tet10 + IPC solver instantiation in the tree was
//! `tests/tet10_indentation_demand1.rs`, and its material was `NeoHookean`.
//! (Stated in the past tense on purpose — this file adds three more, two of
//! them Yeoh, so a present-tense version of that sentence would be falsified
//! by the change that made it.) Before item 3 rewires the 6 416-line
//! `tools/cf-sim-research/src/insertion_sim.rs`, this file answers the cheap
//! question on a fixture that runs in 33 s: does the triple (Tet10 element
//! × Yeoh material × IPC face barrier) solve, and at what Newton cost relative
//! to the two baselines that already work — Tet4 × Yeoh, and Tet10 ×
//! `NeoHookean`?
//!
//! ## The fixture
//!
//! Two bodies of Ecoflex 00-30, both SDF-meshed through the same
//! `SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh` path `insertion_sim` uses and
//! enriched to Tet10, and both driven by the same [`Cell::ramp`]:
//!
//! - **The plate** — 40 × 40 × 12 mm, top face pinned, pressed onto a plane or
//!   a 60 mm sphere ([`Indenter::Plane`], [`Indenter::Sphere`]) and MARCHED up
//!   through the plate as a compression ramp. 4 240 tets.
//! - **The shell** — a 10 mm spherical bore in a 12 mm wall, outer skin pinned
//!   as `insertion_sim`'s `outer_skin_bc` pins it, with a rigid sphere GROWING
//!   inside the bore ([`Indenter::Bore`]) as an interference ramp. 8 736 tets.
//!
//! ★ The two differ in the body and the boundary set and in nothing else —
//! same element, material, barrier, solver config and ramp driver — which is
//! what makes the contrasts below attributable to the geometry.
//!
//! ★ **This closes §7 item 1 of the recon doc**, which reads: *"No Tet10 solve
//! has been run. Every type-level result above is a compile probe. Nothing
//! here measures convergence, residuals, or wall-clock."* It does not close
//! §7 item 3 (Newton cost for `insertion_sim`'s *scene*) — different geometry,
//! and this file's Tet4 arm is measured-invalid, so there is no clean
//! element-order comparison here either.
//!
//! ## What it found, measured 2026-09-20 at `d12e3bf9`
//!
//! - **It converges, to 44 % engineering compression.** A marched compression
//!   ramp runs clean to 5.29 mm of deflection on a 12 mm plate at
//!   `κ = 1e7`, holding 0.357 mm of standoff throughout. Rest contact solves
//!   in 6 Newton iterations at residual 8.3e-14 over 4 240 tets.
//!
//!   ⚠ An earlier revision reported "2 Newton iterations at residual 5.9e-13"
//!   here. Those are `κ = 1e4` numbers that survived the re-baseline to
//!   `κ = 1e7` — the same class of defect as the three stale gate numbers that
//!   re-baseline already produced. **Re-baselining invalidates every number
//!   calibrated against the old baseline, in prose as well as in gates.**
//!
//!   ⚠ **The system solved is 18 738 free DOF, not the 24 993 that
//!   `3 × positions()` suggests.** 1 244 of the 8 331 Tet10 positions are
//!   orphans the solver auto-pins, and 889 more are the pinned top face. An
//!   earlier revision reported 24 993 and that overstates the solve by 1.33×.
//! - **It converges under CURVED contact too**, which is the cell
//!   `insertion_sim` actually has. Against a 60 mm sphere at the same
//!   `(κ, d̂)` and mesh: 4–5 Newton iterations per increment, residuals
//!   ~8.5e-14, clean to 13.4 % deflection, standoff decreasing monotonically
//!   0.845 → 0.583 mm as the apex advances and the patch widening 185 → 473
//!   pairs. This matters because `tet10_face_contact.rs` states at its own
//!   rung-8c gate that *"8b's gate used a PLANE precisely because `∇²sd = 0`
//!   hides the `b'·∇²sd` Hessian term"* — a flat fixture is structurally
//!   incapable of exercising the curvature term, and the pre-existing curved
//!   gate is `NeoHookean`, so curved × Yeoh was the untested cell.
//! - **The face path is genuinely selected**, asserted on
//!   `Mesh::boundary_faces6` — the selector, not the types, per the recon
//!   doc's §9 correction. The curved cell gets the same treatment: its gate
//!   asserts a non-zero `Sdf::hessian` of magnitude √2/R, because an indenter
//!   that forgets to override the trait default is flat in the tangent however
//!   round it looks.
//! - **Raw `peak_contact_pressure` is corner-dominated on a curved patch.**
//!   Along the curved ramp the net force rises smoothly and monotonically
//!   (0.43 → 5.27 N) while `p_peak` wanders non-monotonically over a ~9× range,
//!   because the winning pair's tributary area is ~1e-10 m² against a median of
//!   2.7e-6 — four orders down — and 13–15 % of pairs are outright degenerate.
//!   `peak_contact_pressure` filters `area ≤ 0`; it cannot filter `area ≈ 0⁺`.
//!   Consistent with why the conformity readout carries `p_peak_smoothed` and
//!   `lq_ratio` rather than the raw max. Reported here, not fixed here.
//! - **`κ` carried over from rung 8b is wrong for this fixture by ≥ 3 orders.**
//!   At `κ = 1e4` the ramp stalls at 0.83 % compression; 1e5 and 1e6 stall
//!   later; 1e7 and 1e8 run clean. The stall is a *marching* failure, not a
//!   solver limit: a barrier too soft to hold the plate ahead of the advancing
//!   plane leaves less clearance than one increment, so the next increment
//!   starts infeasible and the line search dies on Newton iteration 0. Every
//!   iter-0 row in the sweep has `min_sd < RAMP_STEP`, and that relation is
//!   asserted rather than narrated.
//! - **And that mechanism turns out to *derive* `κ`, not just explain it.**
//!   The face barrier integrates `κ·b` over the rest area with quadrature
//!   weights summing to 1, so **`κ·|b'(d)|` is a traction in pascals** —
//!   measured against the solver to floating point by
//!   `the_face_barrier_traction_law_holds_against_the_solver`. Requiring the
//!   barrier to hold one ramp increment open under the measured design
//!   traction — at the patch's TIGHTEST point, not its mean gap — gives a
//!   floor of **2.241e6**. Requiring the standoff to stay inside half the band
//!   gives a ceiling of **2.123e7** — that one is a *stated requirement*, not
//!   a derivation, and its sensitivity is measured by
//!   `the_ceiling_is_a_stated_requirement`. **1e7 is the only decade between
//!   them.** The sweep's bracket and the derivation were
//!   computed from nothing in common, and they agree: at the design traction
//!   1e6 holds 0.056 mm against a 0.1 mm increment (infeasible, and it stalls)
//!   while 1e7 holds 0.424 mm (feasible, and it runs clean).
//! - ⚠ **`κ`'s lower bound belongs to the *marching scheme*, not the physics.**
//!   The material and the imposed compression set the traction; `RAMP_STEP`
//!   sets how much clearance has to survive it. A different ramp schedule
//!   moves the floor, which is why the bridge cannot inherit this number.
//! - **The per-vertex barrier contacted vertices that are in no tetrahedron
//!   — FIXED in #953, and the numbers below are what motivated it.**
//!   `SdfMeshedTetMesh::positions()` is the BCC lattice, not the body: 1 244 of
//!   its 2 348 entries are referenced by no tet. That part is unchanged and is
//!   still what this file's `the_vertex_barrier_contacts_vertices_that_are_in_no_tetrahedron`
//!   measures, through `per_pair_readout`, which keeps the raw list by design.
//!   What changed is the ACTIVE SET: both contact models' `active_pairs` now
//!   apply a tet-incidence filter (`referenced_vertex_mask`), so the solver no
//!   longer sees those pairs. Before that filter, 386 of the Tet4 arm's 607
//!   contact pairs landed on dead nodes — 338 of which sit below the plane in
//!   the *rest* configuration — and the barrier's `d = sd.max(d̂ · 1e-6)` floor
//!   reported ~4e9 N for them. The face path never had the problem:
//!   `boundary_faces6()` is built from tet connectivity.
//!   ⚠ The exposure in the shipped sim was far larger than here: on
//!   `insertion_sim`'s synthetic sliding fixture, 31 852 of 41 432 positions
//!   are dead and **94.3 % of the active pairs at interference 0 mm sat on
//!   them** (6 583 of 6 981; now 398 of 398 live). A shell body's cavity is
//!   exactly where the lattice's dead nodes live and the intruder is driven
//!   into the middle of them, so a solid plate understates it badly.
//!
//! ⚠ **Claims retracted from an earlier revision of this file**, recorded
//! because the corrections are the useful part. Two are structural enough to
//! list; the stale-number retractions are inline above, beside the numbers
//! that replaced them.
//!
//! 1. It claimed Tet10 × Yeoh "walls at `ArmijoStall(iter 0)` by ~1 % strain",
//!    with step-refinement evidence that the wall was not an increment
//!    artefact. The refinement evidence was sound and the conclusion did not
//!    follow — that comparison varied the *increment* and held `κ` fixed, so it
//!    could never have separated a property of Tet10 × Yeoh from a property of
//!    `κ = 1e4`. It was the latter.
//! 2. It claimed the Tet4 arm "converges to a pose 3.52 mm inside the plane",
//!    implying Newton descended past the clamped barrier. It does not: `min_sd`
//!    is −3.52 mm **in the rest configuration** and the solve never moves it
//!    — identical at `κ = 1e4` and `κ = 1e7`, which a stiffness-driven
//!    penetration could not be. The cause is dead lattice vertices, above.
//!    ★ What gave it away was the invariance: a number that does not move when
//!    you change the thing that supposedly causes it is not caused by it.
//!
//! ## What the cavity cell found
//!
//! - **It converges when the material closes AROUND the probe.** The bore runs
//!   clean to **4.72 mm of interference on a 10 mm bore — 47 % radial** at 4
//!   Newton iterations rising to 8, residuals ~1e-13, then stalls at 4.82 mm.
//!   Rest contact is 6 iterations over 8 736 tets.
//! - **The whole wall engages at rest and never recruits** — **434 active pairs
//!   at every rung**, against the curved plate cell's 185 → 473. That is the
//!   regime `insertion_sim` names when it explains why penalty's `κ` had to be
//!   softened there (*"the whole cavity wall engages at once, unlike the rows'
//!   localized probe"*), reproduced under IPC and asserted rather than quoted.
//! - ⛔ **`F_z / A_flat` does not survive the geometry.** Every contact normal
//!   is radial, so the vector sum cancels: `‖ΣF‖ / Σ‖f‖` measures **1.11e-3
//!   falling to 1.45e-4** against the plate's 0.9992. The net force understates
//!   the cavity's contact by three to four orders of magnitude, so the question
//!   "what plays the role of `A_flat`" has no answer — **nothing does, because
//!   the force it divides is gone.** The replacement is to stop going through
//!   force and read the traction the barrier integrates, and the plate is where
//!   the two are shown to be the same measurement (ratio **0.9360–0.9365**,
//!   constant over thirteen rungs). That replacement is then checked *on the
//!   cavity itself* against a second, independent reading —
//!   [`the_force_free_traction_agrees_with_an_independent_reading_on_the_cavity`]
//!   — which agrees to **1.0074–1.0162**.
//!
//!   ⚠ An earlier revision reported **6.8e-4 → 8.7e-5** here. Those are the
//!   `z` COMPONENT of the residual, not its magnitude, and they understate it
//!   by ~1.65x: the cavity's residual is an artefact of mesh asymmetry and has
//!   no preferred axis (measured at rest, `ΣF = (5.76, 5.12, 5.93) mN`). The
//!   gate that produced them asserted on `z` alone while its failure message
//!   claimed it would catch the shell being pushed *sideways* — which is
//!   exactly the two components it never read.
//! - ✅ **The flat plate's `ρ = 1.30` bounds the enveloping patch** —
//!   `ρ ∈ [1.176, 1.221]` all the way to the wall, *tighter* than the plate's
//!   own 1.04–1.26. The constant transfers; [`PATCH_NONUNIFORMITY`]'s warning
//!   that a cavity "has no reason to share it" was right to demand the
//!   measurement and wrong about the outcome.
//! - ⚠ **The marching-feasibility criterion is conservative here, not binding.**
//!   `min_sd` drops below `ρ · RAMP_STEP` at 3.82 mm and the ramp keeps
//!   converging for ten more increments. What ends it is a stall at Newton
//!   iteration **5** with a residual four orders above the scene's scale — not
//!   the iteration-0 infeasible start the plate's `κ` floor is derived from.
//!   The cause is unidentified.
//!
//! ## What this cannot see
//!
//! - **An IRREGULAR closing cavity.** The bore is a sphere, so its gap is
//!   uniform by symmetry and the `ρ` above measures what the DISCRETISATION
//!   contributes — which is now measured rather than asserted:
//!   [`the_enveloping_patch_nonuniformity_is_a_property_of_the_mesh`] holds the
//!   geometry fixed and reports `ρ` = 1.1106 / 1.1759 / 1.0552 at `CELL` =
//!   5 / 4 / 3 mm. ⇒ **the `[1.176, 1.221]` quoted above is a `CELL = 4 mm`
//!   statement.** `insertion_sim`'s cavity is a scan isosurface; its
//!   non-uniformity is a different quantity and is the bridge's risk, not this
//!   cell's result.
//! - ⛔⛔ **AN UNCONFINED WALL. The shell is SEALED and the sleeve is not.**
//!   The outer skin is pinned all the way round and the bore is closed, so a
//!   growing intruder has nowhere to send material: at the deepest converged
//!   rung the wall must compress **22.7 % by volume (J = 0.773)**, since the
//!   annulus outside a 14.72 mm bore holds 3.124e-5 m³ where the rest shell
//!   held 4.041e-5. `insertion_sim`'s device wall is `outer.subtract(cavity)`
//!   on a sock-over-capsule with an **open mouth**, where material escapes
//!   axially instead. ⇒ **this cell is stiffer than the thing it stands for,
//!   and the depth and traction magnitudes are conditional on that.** What does
//!   NOT depend on it: the force cancellation (symmetry), the constant active
//!   set (topology), and `ρ` (a ratio of gaps).
//! - **The strain regime where Yeoh's adequacy is undetermined.** Recon §7
//!   item 2: Yeoh pairs with element order in **none** of the 310 study files,
//!   the book's ladder runs NH → Mooney-Rivlin → **Ogden** with Ogden
//!   *"dominating Ecoflex curve fits above 100% strain"*, and row 23's
//!   stretches `[2.06, 1.22, 0.073]` sit above that line. The plate cell tops
//!   out at 44 % engineering compression (`λ ≈ 0.56`) and the cavity cell
//!   drives the bore 47 % past its rest radius, which puts the wall in hoop
//!   TENSION at a circumferential stretch of **at least 1.47** — a lower bound
//!   that follows from the solve, since the wall stays outside a 14.72 mm
//!   intruder that started at 10 mm. Both are inside the regime Yeoh is fitted
//!   for. So this fixture demonstrates that Tet10 × Yeoh **solves**, in
//!   compression and now in tension, and still says nothing about whether Yeoh
//!   is the right model where the book prefers Ogden.
//!   ⚠ The principal stretches themselves are not read out here; 1.47 is a
//!   bound from the geometry, not the tensor.
//! - ~~**Graded materials.**~~ **No longer true.** The shell now also carries
//!   `insertion_sim`'s row-23 stack (Ecoflex 00-20 / Dragon Skin 10A / Dragon
//!   Skin 20A, innermost first) through the same `LayeredScalarField` keyed on
//!   the cavity SDF. Grading costs **32 % of the depth** (3.220 mm against
//!   4.720 mm) and stiffens the wall by only **1.22-1.72x** where a
//!   volume-weighted modulus predicts 3.48x — the load enters through the
//!   layer at the bore, which is SOFTER than the uniform baseline, so the
//!   layers load in series and the volume share is the wrong weight. The
//!   stiffening is **linear-elastic**: holding ν fixed, the linear modulus
//!   carries 18.5x what Yeoh's `C₂` does.
//!   Both of 3a's readings survive: `rho` stays inside
//!   [`PATCH_NONUNIFORMITY`] and the net force still cancels.
//!   ⛔ What remains unseen here is narrower and stated at
//!   [`the_interface_flag_cannot_isolate_a_layer_boundary_at_this_cell_size`]:
//!   at `CELL` = 4 mm the straddle flag selects 40-63 % of the mesh, so this
//!   fixture says nothing about a layer boundary RESOLVED as a seam.
//! - ~~**A derived `κ`.**~~ **No longer true — see the `κ` bullet above.** `κ`
//!   is now computed from the face barrier's own traction relation, and
//!   `kappa_is_derived_and_not_swept` re-derives it on every build. What
//!   remains unseen is narrower and stated there: the interval is
//!   9.5× wide, so it selects a decade only while the design traction is
//!   known to better than roughly [0.47×, 4.5×].
//! - **`d̂`.** Held fixed throughout. Sweeping it moves `STANDOFF` and so the
//!   initial condition, which is a different experiment.
//! - **Friction.** `SolverConfig::friction_mu` defaults to `0.0` and this
//!   fixture leaves it there — the same frictionless regime `insertion_sim`
//!   already runs in, and the regime rung 8b's face path requires.
//!
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use std::sync::OnceLock;

use nalgebra::Point3;
use sim_ml_chassis::Tensor;
use sim_soft::Material;
use sim_soft::element::{Tet4, Tet10};
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_20A, ECOFLEX_00_20, ECOFLEX_00_30, SiliconeMaterial,
};
use sim_soft::{
    Aabb3, ActivePairsFor, BoundaryConditions, ConstantField, ContactPair, CpuNewtonSolver,
    DifferenceSdf, Field, IpcRigidContact, LayeredScalarField, MaterialField, Mesh, MeshingHints,
    NeoHookean, RigidPlane, Sdf, SdfMeshedTetMesh, Solver, SolverConfig, SolverFailure, SphereSdf,
    Tet10Mesh, TetId, TranslatedSdf, Vec3, VertexId, Yeoh, barrier_derivative,
    boundary_faces_on_isosurface, face_barrier_kappa, face_barrier_standoff, peak_contact_pressure,
    referenced_vertices,
};

// ── fixture geometry ────────────────────────────────────────────────

/// Plate half-extents (m): 40 × 40 × 12 mm. The 12 mm thickness is the
/// sleeve-wall scale the renovation targets (NH walls at ~7 mm, the sleeve
/// needs 8 mm), not the 100 mm block the rung-8b tests use.
const HALF: [f64; 3] = [0.020, 0.020, 0.006];

/// BCC-stuffing cell size (m). Coarse on purpose — this fixture is a
/// convergence probe, not a resolution study.
const CELL: f64 = 0.004;

/// Barrier band (m).
///
/// ★ **Chosen to satisfy the recon doc's governing rule, not by analogy.**
/// `docs/INSERTION_SIM_TET10_RENOVATION_RECON.md` §6 gives
/// **`d̂ < ℓ / 2`** — below `ℓ ≲ 2d̂` both barriers are active at interior
/// vertices, the `b″` contributions add, the Hessian condition number spikes,
/// and thin-material scenes need *"2–5× more Newton iterations per timestep"*.
/// Here `ℓ = 12 mm` ⇒ `d̂ < 6 mm`, and 1.2 mm sits at `d̂/ℓ = 0.1`, an order
/// inside the bound. The measured 4–6 Newton iterations per increment are
/// consistent with the multiplier not firing — though that is consistency, not
/// a test of §6, since nothing here goes thin enough to trip it.
///
/// ⚠ §6 also flags `d̂ < ℓ/2` as *"a practitioner-level engineering heuristic
/// rather than a published theorem"*. It happens to coincide with the rung-8b
/// tests' ratio (`d̂ = 0.01` on a 100 mm block), which is what an earlier
/// revision cited instead — a precedent where a stated constraint existed.
const D_HAT: f64 = 0.0012;

/// The rung-8b face-contact tests' barrier stiffness, carried over unchanged.
///
/// ⛔ **Measured insufficient for this fixture** — see
/// [`the_armijo_wall_against_barrier_stiffness`]. At this value the barrier
/// cannot hold the plate far enough ahead of the advancing plane to keep the
/// next increment feasible, and the ramp stalls at 0.83 % compression. Kept as
/// the counterexample, not used by any gate.
const RUNG_8B_KAPPA: f64 = 1.0e4;

/// Contact traction at the design point (Pa).
///
/// **Measured, not chosen**: the area-mean barrier traction `F_z / A_flat` at
/// the deepest rung of the baseline ramp — 5.2944 mm of deflection on the
/// 12 mm plate, 44.1 % engineering compression.
/// [`is_the_contact_traction_a_property_of_the_scene_or_of_kappa`] re-measures
/// it and is the referent for this number; if the fixture's geometry, material
/// or ramp ceiling moves, that probe is what says so.
///
/// ⚠ **It is not wholly independent of `κ`, and the probe quantifies that.**
/// At a fixed plane height a stiffer barrier holds the plate further off the
/// plane, which — the top face being pinned — compresses it *more*, not less.
/// Measured over a **100×** span of `κ` at one common plane height, the
/// traction spans **1.53×**. So it is dominated by the material and the
/// imposed compression, and the residual `κ` coupling is a stated 1.53× over
/// two decades rather than an assumed zero.
const DESIGN_TRACTION_PA: f64 = 30_395.0;

/// The flat contact patch [`DESIGN_TRACTION_PA`] was measured over (m^2).
///
/// Pinned as a measured area rather than computed from [`HALF`], because the
/// meshed patch is **not** the nominal footprint in general — it is here only
/// because 40 mm happens to tile exactly at [`CELL`] = 4 mm. Widening the
/// plate to 50 mm, for instance, yields 1920 mm^2 and not 2000. Deriving this
/// from the footprint would therefore assert a *meshing* coincidence while
/// looking like it asserted the geometry.
const DESIGN_CONTACT_AREA: f64 = 1.6e-3;

/// Active face count on that patch — the resolution half of the same guard.
const DESIGN_CONTACT_FACES: usize = 520;

/// The standoff the barrier has to keep open, and where it comes from.
///
/// Not a comfort margin: it is the marching scheme's feasibility condition,
/// read off the measured failure mode. An `ArmijoStall` at Newton **iteration
/// 0** means the line search never found a decrease from the increment's
/// starting point — the start was already infeasible — and that happens
/// exactly when the plane advances further in one increment than the barrier
/// was holding the plate off it. [`the_armijo_wall_against_barrier_stiffness`]
/// asserts that relation directly (`min_sd < RAMP_STEP` on every iter-0 row).
///
/// ⭐ So the lower bound on `κ` is set by the **integration scheme**, not by
/// the physics. The physics sets [`DESIGN_TRACTION_PA`]; the scheme sets how
/// much clearance must survive under it. Halve [`RAMP_STEP`] and the floor
/// falls with `|b'|` at the smaller gap.
///
/// ⚠⚠ **The condition is on the MINIMUM gap, and the traction is a MEAN** —
/// see [`PATCH_NONUNIFORMITY`], which is what reconciles them. A first version
/// of this derivation compared the two directly and produced a floor that was
/// optimistic by that factor; it is the same mean-against-order-statistic
/// mistake [`Press::mean_sd`] documents twenty lines away, made again in the
/// arithmetic after being fixed in the gate.
const REQUIRED_STANDOFF: f64 = RAMP_STEP * PATCH_NONUNIFORMITY;

/// How much tighter the tightest gap in the contact patch is than the gap that
/// carries the patch's *mean* traction: `ρ = d_eff / min_sd ≥ 1`.
///
/// **Why the derivation cannot do without it.** The stall condition is on
/// `min_sd` — the single closest point — but the only gap available
/// analytically is `d_eff`, the gap at which a *uniform* patch would carry the
/// measured mean traction. Convexity of `κ·|b'|` puts `d_eff` above `min_sd`
/// (held by the bracket arm in
/// [`is_the_contact_traction_a_property_of_the_scene_or_of_kappa`]), so asking
/// `d_eff > RAMP_STEP` is strictly weaker than asking `min_sd > RAMP_STEP`.
/// Requiring `d_eff > ρ · RAMP_STEP` restores it.
///
/// **Measured, and deliberately the conservative end.** `d_eff / min_sd` reads
/// 1.264 (κ=1e6), 1.111 (κ=1e7), 1.038 (κ=1e8) at the common plane height and
/// 1.186 at the design point. It shrinks monotonically as the barrier
/// stiffens across those three; what drives that has not been isolated here.
/// `1.30` rounds the worst of them up.
/// The probe re-measures it and fails if any pose exceeds this value.
///
/// ★ **Rank this caveat by its measured effect: it moves the NUMBER, not the
/// conclusion.** Omitting the correction entirely (`ρ = 1`) drops the floor
/// from 2.241e6 to 1.730e6, and `ρ` would have to reach **4.236** — 3.35× the
/// worst pose measured here — before the floor rose past `1e7` and excluded
/// it. So this is a correctness fix to a load-bearing quantity with a wide
/// margin behind it, not a near-miss.
///
/// ⚠ A number from ONE fixture's patch. A cavity closing around a probe
/// (renovation item 3) has no reason to share it, and the bridge must
/// re-measure rather than inherit.
const PATCH_NONUNIFORMITY: f64 = 1.30;

/// Removing the min-vs-mean correction must not compile.
///
/// The probe checks that [`PATCH_NONUNIFORMITY`] really bounds the measured
/// `d_eff / min_sd`, but the probe is `#[ignore]`d and runs in no CI job. At
/// `1.0` the correction is gone and the floor silently reverts to bounding the
/// MEAN-equivalent gap while the stall condition is on the MINIMUM one — a
/// load-bearing number changed with nothing failing. A `const` assertion fails
/// the BUILD rather than a test, which is the right severity for deleting a
/// correction by editing one digit.
const _: () = assert!(
    PATCH_NONUNIFORMITY > 1.0,
    "PATCH_NONUNIFORMITY <= 1 removes the min-vs-mean correction the derived \
     floor depends on",
);

/// Barrier stiffness every gate here runs at — **derived, not swept**.
///
/// The surface-integrated face barrier is
/// `E = A_rest · Σ_q ŵ_q · κ · b(sd)` with `Σ_q ŵ_q = 1`, so `κ·|b'(d)|` is a
/// **traction in pascals** ([`sim_soft::contact::barrier`], and
/// [`the_face_barrier_traction_law_holds_against_the_solver`] measures it
/// against the solver). That makes `κ` determined by two requirements rather
/// than found by sweeping decades:
///
/// ```text
///   floor    DERIVED from a measured failure mechanism. The barrier must
///            hold one ramp increment open at the TIGHTEST point of the
///            patch, or the next increment starts infeasible
///              kappa >= sigma / |b'(rho * RAMP_STEP)|     = 2.241e6
///
///   ceiling  a STATED REQUIREMENT, not a derivation. The standoff is a BIAS
///            in the reported contact position; this asks it to stay in the
///            lower half of the tolerance band, above which the barrier
///            cushions rather than enforces
///              kappa <= sigma / |b'(d_hat / 2)|           = 2.123e7
/// ```
///
/// ⚠ **The two bounds do not have the same standing, and the headline above
/// is about the floor.** `|b'(RAMP_STEP · ρ)|` follows from a stall mechanism
/// that was measured; `d̂/2` is a round number chosen for a real reason with no
/// measurement behind the *fraction*. [`the_ceiling_is_a_stated_requirement`]
/// measures how much that choice is load-bearing: the selection survives
/// anywhere in roughly `[d̂/2.8, d̂/1.4]` and breaks at `d̂/3`, where the ceiling
/// falls to 9.05e6 and excludes 1e7.
///
/// **`1e7` is the only decade in `[2.241e6, 2.123e7]`.** It is not the
/// smallest decade that happened to converge; it is the one the two bounds
/// leave. [`kappa_is_derived_and_not_swept`] evaluates both bounds from the
/// shipped barrier and fails if this constant leaves the interval — including
/// if it were lowered to 1e6 or raised to 1e8.
///
/// ✅ The derivation reproduces what the sweep measured, having been computed
/// from neither: 1e6 holds only **0.056 mm** at the design traction, under the
/// 0.1 mm increment ⇒ infeasible, and the sweep stalls there; 1e7 holds
/// **0.424 mm** ⇒ feasible, and the sweep runs clean; 1e8 holds **0.885 mm**,
/// 74 % of the band, which is the cushioning regime the ceiling excludes.
///
/// ⚠ **What the standoff costs, stated rather than hidden**: at `κ = 1e7` the
/// converged pose floats 0.357 mm off the plane, **6.7 %** of the 5.29 mm
/// deflection being measured. That is a systematic bias in every contact
/// position this fixture reports, and shrinking it means a smaller `d̂`, not a
/// larger `κ`.
const KAPPA: f64 = 1.0e7;

/// Plane standoff inside the band, so the bottom face is engaged at rest.
///
/// ⚠ Tied to `D_HAT`, not to a swept `d̂`. A sweep that varied the band would
/// also be varying the rest standoff, i.e. the initial condition — so
/// [`Barrier`] sweeps are `κ`-only while this holds.
const STANDOFF: f64 = 0.4 * D_HAT;

/// The IPC barrier's two tuning parameters, passed rather than read from
/// constants so they can be swept.
///
/// They are packaged together because they are not independent: `d̂` sets the
/// band over which the barrier acts and `κ` its strength within that band, and
/// a claim conditioned on one is conditioned on both.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Barrier {
    kappa: f64,
    d_hat: f64,
}

/// Barrier parameters every gate runs at.
const BASELINE_BARRIER: Barrier = Barrier {
    kappa: KAPPA,
    d_hat: D_HAT,
};

/// The flat cell — what most gates here run at.
const BASELINE: Setup = Setup {
    barrier: BASELINE_BARRIER,
    indenter: Indenter::Plane,
};

/// The curved cell: same barrier, sphere instead of plane. This is the
/// configuration `insertion_sim` actually has.
const CURVED: Setup = Setup {
    barrier: BASELINE_BARRIER,
    indenter: Indenter::Sphere,
};

/// The rung-8b carry-over, retained so the failure it produces stays
/// reproducible rather than becoming a sentence about a number nobody can run.
const SOFT_BARRIER: Setup = Setup {
    barrier: Barrier {
        kappa: RUNG_8B_KAPPA,
        d_hat: D_HAT,
    },
    indenter: Indenter::Plane,
};

/// Radius of the curved indenter (m).
///
/// 60 mm against the plate's 40 mm lateral extent — the same radius-to-extent
/// ratio (1.5) the rung-8c gate in `tet10_face_contact.rs` uses. Large enough
/// that curvature is gentle rather than a stress singularity, small enough that
/// `∇²sd = (I − n̂n̂ᵀ)/‖p−c‖` is a real fraction of the barrier tangent.
const SPHERE_R: f64 = 0.060;

/// The rigid obstacle the plate is pressed onto.
///
/// ⭐ This axis exists because `tet10_face_contact.rs` says, at its own rung-8c
/// gate: *"8b's gate used a PLANE precisely because `∇²sd = 0` hides the
/// `b'·∇²sd` Hessian term."* A plane cannot exercise the curvature term the
/// face barrier carries, and `insertion_sim` is a curved intruder in a curved
/// cavity — so a flat-only result is not evidence about the case item 3 needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Indenter {
    /// Ground plane, normal `+z`. `Sdf::hessian` takes the trait's zero default.
    Plane,
    /// Sphere below the plate. `SphereSdf` **overrides** `Sdf::hessian`
    /// (`sim/L0/soft/src/sdf_bridge/sdf.rs`), so this is the cell that carries the curvature
    /// term into the assembled tangent.
    Sphere,
    /// Sphere *inside* the cavity of the [`shell`] body — the enveloping cell.
    ///
    /// Same primitive as [`Self::Sphere`] and so the same non-zero
    /// `Sdf::hessian`, but the soft material wraps it instead of being
    /// indented by it. The consequences are not cosmetic: the contact patch is
    /// a closed surface rather than a disc, every normal is radial, and the
    /// whole patch engages at once instead of behind an advancing front.
    Bore,
}

impl Indenter {
    /// The primitive with its contact surface advanced by `h` past the soft
    /// body's *rest* surface.
    ///
    /// ★ **`h` is the same physical quantity for all three shapes**, which is
    /// what lets one ramp driver and one [`summarize`] serve every cell. The
    /// plate rests with its contact face on `z = 0`, so the plane's height and
    /// the sphere's apex height are both that advance directly; the shell
    /// rests with its cavity wall at `R_CAVITY`, so the bore's advance is its
    /// radius *beyond* `R_CAVITY`. In every case `h < 0` leaves a gap, `h = 0`
    /// is exact touching, and `h > 0` is interference.
    fn at(self, h: f64) -> Box<dyn Sdf> {
        match self {
            Self::Plane => Box::new(ground_at(h)),
            // Centre a radius below the apex: `sd(0,0,h) = |h − (h−R)| − R = 0`.
            Self::Sphere => Box::new(TranslatedSdf {
                inner: SphereSdf { radius: SPHERE_R },
                offset: Vec3::new(0.0, 0.0, h - SPHERE_R),
            }),
            // Centred at the origin, like the cavity it grows into: the soft
            // shell is the region `|p| > R_CAVITY`, so `sd = |p| − (R_CAVITY+h)`
            // is positive exactly where the material is.
            Self::Bore => Box::new(SphereSdf {
                radius: R_CAVITY + h,
            }),
        }
    }
}

/// One fixture configuration: barrier parameters plus the shape pressed on.
#[derive(Clone, Copy, Debug)]
struct Setup {
    barrier: Barrier,
    indenter: Indenter,
}

impl Setup {
    /// The contact model with the obstacle's surface at height `h`.
    fn against(self, h: f64) -> IpcRigidContact {
        IpcRigidContact::with_params(
            vec![self.indenter.at(h)],
            self.barrier.kappa,
            self.barrier.d_hat,
        )
    }
}

/// Large `dt` ⇒ quasi-static: inertia negligible, contact and elasticity
/// balance. Matches the rung-8b tests' `STATIC_DT`.
const STATIC_DT: f64 = 1.0e3;

const MAX_NEWTON_ITER: usize = 60;

/// Axis-aligned box, exact signed distance (`q = |p − c| − h`), analytic
/// gradient.
///
/// Written here because the repo ships no box primitive — `cf_geometry`
/// exports a sphere, a translation wrapper and a difference. `grad` is checked
/// against central differences by [`box_sdf_gradient_matches_finite_differences`]
/// so the fixture's own geometry is verified rather than trusted.
struct BoxSdf {
    center: Vec3,
    half: Vec3,
}

impl BoxSdf {
    /// `q_i = |p_i − c_i| − h_i`: per-axis signed distance to the slab pair.
    /// Positive outside that slab, negative inside it.
    fn q(&self, p: Point3<f64>) -> Vec3 {
        Vec3::new(
            (p.x - self.center.x).abs() - self.half.x,
            (p.y - self.center.y).abs() - self.half.y,
            (p.z - self.center.z).abs() - self.half.z,
        )
    }

    /// Componentwise sign of `p − c`, with `+1` at exactly zero. Folds the
    /// first-octant gradient back onto the octant `p` is actually in.
    fn fold_sign(&self, p: Point3<f64>) -> Vec3 {
        let s = |d: f64| if d < 0.0 { -1.0 } else { 1.0 };
        Vec3::new(
            s(p.x - self.center.x),
            s(p.y - self.center.y),
            s(p.z - self.center.z),
        )
    }
}

impl Sdf for BoxSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let q = self.q(p);
        // Outside: Euclidean distance to the box, counting only the axes the
        // point overshoots. Inside: every `q_i < 0`, and the nearest face is
        // the least-negative one.
        let outside = Vec3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
        let inside = q.x.max(q.y).max(q.z).min(0.0);
        outside + inside
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        let q = self.q(p);
        let sign = self.fold_sign(p);
        let outside = Vec3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0));
        let n = if outside.norm() > 0.0 {
            // Outside (or on an edge/corner): the direction away from the
            // nearest point on the box, which is the overshoot vector.
            outside.normalize()
        } else {
            // Strictly inside: the gradient points at the nearest face, i.e.
            // along the single axis whose `q` is largest.
            let mut axis = 0;
            if q.y > q[axis] {
                axis = 1;
            }
            if q.z > q[axis] {
                axis = 2;
            }
            let mut e = Vec3::zeros();
            e[axis] = 1.0;
            e
        };
        Vec3::new(sign.x * n.x, sign.y * n.y, sign.z * n.z)
    }
}

/// The plate, sitting with its bottom face on `z = 0`.
const fn plate() -> BoxSdf {
    BoxSdf {
        center: Vec3::new(0.0, 0.0, HALF[2]),
        half: Vec3::new(HALF[0], HALF[1], HALF[2]),
    }
}

/// Ground plane (normal `+z`) whose surface sits at height `plane_h`.
/// `RigidPlane`'s offset *is* the plane height, since `sd(p) = p·n̂ − offset`.
///
/// `plane_h < 0` holds the plate in the barrier band without compressing it;
/// `plane_h > 0` drives the plane up through where the plate's bottom face
/// rests, and with the top face pinned that is a compression of `plane_h` on a
/// `2 · HALF[2]` plate.
fn ground_at(plane_h: f64) -> RigidPlane {
    RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), plane_h)
}

/// The resting plane: `STANDOFF` below the plate's bottom face, so the bottom
/// is inside the barrier band but the plate is essentially unloaded.
const REST_PLANE_H: f64 = -STANDOFF;

/// Uniform Ecoflex 00-30, through the five-field bounded constructor — the
/// same `from_yeoh_fields_with_bounds` path `insertion_sim` builds its layered
/// field with, so the per-tet validity caps reach the solver gate here too.
fn yeoh_field() -> MaterialField {
    let m = ECOFLEX_00_30;
    MaterialField::from_yeoh_fields_with_bounds(
        Box::new(ConstantField::new(m.mu)),
        Box::new(ConstantField::new(m.c2)),
        Box::new(ConstantField::new(m.lambda)),
        Box::new(ConstantField::new(m.validity_max_principal_stretch)),
        Box::new(ConstantField::new(m.validity_min_principal_stretch)),
    )
}

/// Uniform Ecoflex 00-30 as Neo-Hookean — the Tet10 × NH baseline's material,
/// built from the same anchor so the two differ in constitutive model alone.
fn nh_field() -> MaterialField {
    let m = ECOFLEX_00_30;
    MaterialField::uniform(m.mu, m.lambda)
}

/// Tet4 plate carrying per-tet Yeoh materials.
fn tet4_yeoh() -> SdfMeshedTetMesh<Yeoh> {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-HALF[0] - CELL, -HALF[1] - CELL, -CELL),
            Vec3::new(HALF[0] + CELL, HALF[1] + CELL, 2.0 * HALF[2] + CELL),
        ),
        cell_size: CELL,
        material_field: Some(yeoh_field()),
    };
    SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&plate(), &hints).expect("mesh the Yeoh plate")
}

/// Tet4 plate carrying per-tet Neo-Hookean materials, same geometry.
fn tet4_nh() -> SdfMeshedTetMesh<NeoHookean> {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-HALF[0] - CELL, -HALF[1] - CELL, -CELL),
            Vec3::new(HALF[0] + CELL, HALF[1] + CELL, 2.0 * HALF[2] + CELL),
        ),
        cell_size: CELL,
        material_field: Some(nh_field()),
    };
    SdfMeshedTetMesh::from_sdf(&plate(), &hints).expect("mesh the NH plate")
}

/// Top-face vertex ids of `mesh` — the pinned set. Picked by position so it
/// works on both the Tet4 corner set and the Tet10 corner-plus-midside set.
fn top_face_pins<M: sim_soft::Material>(mesh: &dyn Mesh<M>) -> Vec<VertexId> {
    let top = 2.0 * HALF[2];
    mesh.positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| (p.z - top).abs() < 0.25 * CELL)
        .map(|(v, _)| v as VertexId)
        .collect()
}

/// Rest DOF vector, flattened `[x, y, z, ...]`.
fn rest_dofs<M: sim_soft::Material>(mesh: &dyn Mesh<M>) -> Vec<f64> {
    let mut rest = Vec::with_capacity(3 * mesh.n_vertices());
    for p in mesh.positions() {
        rest.extend_from_slice(&[p.x, p.y, p.z]);
    }
    rest
}

/// `skeleton()` already carries `gravity_z = 0.0`, `friction_mu = 0.0` and
/// `fbar = false`, which is exactly the regime this fixture wants: no body
/// load, so the barrier pressing the bottom face against the pinned top is the
/// only thing driving deformation; frictionless, as rung 8b's face path
/// requires; and no locking cure, matching `insertion_sim` (where `fbar`
/// appears zero times).
const fn config() -> SolverConfig {
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg
}

// ── the fixture's own geometry, verified ────────────────────────────

#[test]
fn box_sdf_gradient_matches_finite_differences() {
    let b = plate();
    let h = 1.0e-7;
    // Interior, each face's outside, an edge and a corner — the branches
    // `grad` distinguishes.
    //
    // ⚠ Every interior probe is deliberately OFF the medial SET — which is
    // larger than the one case that first caught this. The obvious member is
    // the plate's mid-plane, where the two z-faces are equidistant; but any
    // interior point whose two largest `q` components tie is also on it, e.g.
    // `(0.018, 0, 0.002)` where `q.x == q.z`. Everywhere on that set the
    // signed distance has a kink and the gradient does not exist: the central
    // difference reads a blend while `grad` picks a side by a strict `>`.
    // That is a property of the distance function, not a defect in this impl.
    let probes = [
        Vec3::new(0.0, 0.0, 0.002),       // interior, below the medial plane
        Vec3::new(0.0, 0.0, 0.010),       // interior, above it
        Vec3::new(0.0, 0.0, -0.002),      // below, face
        Vec3::new(0.030, 0.0, HALF[2]),   // +x, face
        Vec3::new(0.030, 0.030, HALF[2]), // +x+y, edge
        Vec3::new(0.030, 0.030, 2.0 * HALF[2] + 0.010), // corner
    ];
    for p in probes {
        let pt = Point3::from(p);
        let analytic = b.grad(pt);
        let mut fd = Vec3::zeros();
        for i in 0..3 {
            let mut lo = p;
            let mut hi = p;
            lo[i] -= h;
            hi[i] += h;
            fd[i] = (b.eval(Point3::from(hi)) - b.eval(Point3::from(lo))) / (2.0 * h);
        }
        assert!(
            (analytic - fd).norm() < 1.0e-5,
            "box SDF gradient at {p:?}: analytic {analytic:?} vs central-difference {fd:?}",
        );
    }
}

/// The meshed body is the box the fixture asked for.
///
/// `BoxSdf` is hand-written analytic geometry — exact signed distance and a
/// hand-derived gradient — and every mesh in this file comes out of it, so an
/// error there is an error in all of them. The gradient test above checks six
/// points; this checks the pipeline's whole output against the shape it was
/// asked for, which is the stronger statement: `SDF → BCC stuffing → boundary`
/// has to reproduce the box's extent *and* its volume.
///
/// ⚠ Checks the **boundary**, not `positions()`, because here those are
/// different things — see
/// [`the_vertex_barrier_contacts_vertices_that_are_in_no_tetrahedron`]. The
/// node set spans a full cell beyond the body on every side; the body does not.
#[test]
fn the_meshed_body_reproduces_the_box_extent_and_volume() {
    let t4 = tet4_yeoh();
    let pos = t4.positions();

    let mut lo = Vec3::repeat(f64::INFINITY);
    let mut hi = Vec3::repeat(f64::NEG_INFINITY);
    for f in Mesh::<Yeoh>::boundary_faces(&t4) {
        for &v in f {
            let p = pos[v as usize];
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    let want_lo = Vec3::new(-HALF[0], -HALF[1], 0.0);
    let want_hi = Vec3::new(HALF[0], HALF[1], 2.0 * HALF[2]);
    eprintln!(
        "  boundary bbox: [{:.3}, {:.3}] x [{:.3}, {:.3}] x [{:.3}, {:.3}] mm",
        lo.x * 1e3,
        hi.x * 1e3,
        lo.y * 1e3,
        hi.y * 1e3,
        lo.z * 1e3,
        hi.z * 1e3,
    );
    // A tenth of a cell: tight enough that a misplaced face fails, loose enough
    // that it is not asserting exact float equality on a meshed surface.
    let tol = 0.1 * CELL;
    assert!(
        (lo - want_lo).abs().max() < tol && (hi - want_hi).abs().max() < tol,
        "boundary bbox [{lo:?}, {hi:?}] does not match the requested box \
         [{want_lo:?}, {want_hi:?}] within {tol:e} m",
    );

    let mesh_volume: f64 = (0..t4.n_tets() as TetId)
        .map(|t| {
            let v = t4.tet_vertices(t);
            let (v0, v1, v2, v3) = (
                pos[v[0] as usize],
                pos[v[1] as usize],
                pos[v[2] as usize],
                pos[v[3] as usize],
            );
            // Signed tet volume: (v1-v0) x (v2-v0) . (v3-v0) / 6.
            (v1 - v0).cross(&(v2 - v0)).dot(&(v3 - v0)) / 6.0
        })
        .sum();
    let analytic = 8.0 * HALF[0] * HALF[1] * HALF[2];
    eprintln!(
        "  volume: mesh {mesh_volume:.6e} m3 vs analytic {analytic:.6e} m3 (ratio {:.6})",
        mesh_volume / analytic,
    );
    assert!(
        (mesh_volume / analytic - 1.0).abs() < 1.0e-6,
        "meshed volume {mesh_volume:e} m3 differs from the analytic box {analytic:e} m3 \
         by more than 1e-6 relative",
    );
}

// ── the selector, not the types ─────────────────────────────────────

/// The recon doc's §9 correction, applied as a gate.
///
/// A compile probe for `SdfMeshedTetMesh<Yeoh>` in the solver went green for
/// the wrong reason: the type checks, but `boundary_faces6()` returns `None`
/// and contact silently takes the per-vertex path. `IpcRigidContact::active_pairs`
/// selects on that `Option` and nothing else — so the only way to know
/// `Tet10Mesh<Yeoh>` reaches the face barrier is to ask the selector. The Tet4
/// arm is the negative control that keeps this from passing vacuously.
#[test]
fn tet10_yeoh_takes_the_face_path_and_tet4_yeoh_takes_the_vertex_path() {
    let contact = BASELINE.against(REST_PLANE_H);

    let tet4 = tet4_yeoh();
    assert!(
        Mesh::<Yeoh>::boundary_faces6(&tet4).is_none(),
        "a linear mesh must not surface six-node faces",
    );
    let pos4 = tet4.positions().to_vec();
    let pairs4 = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet4, &pos4);
    assert!(!pairs4.is_empty(), "Tet4 × Yeoh must engage the ground");
    assert!(
        pairs4
            .iter()
            .all(|p| matches!(p, ContactPair::Vertex { .. })),
        "a linear mesh must emit only Vertex pairs",
    );

    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    assert!(
        Mesh::<Yeoh>::boundary_faces6(&tet10).is_some(),
        "a quadratic mesh must surface six-node faces",
    );
    let pos10 = tet10.positions().to_vec();
    let pairs10 = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet10, &pos10);
    assert!(!pairs10.is_empty(), "Tet10 × Yeoh must engage the ground");
    assert!(
        pairs10
            .iter()
            .all(|p| matches!(p, ContactPair::Face { .. })),
        "a quadratic mesh must emit only Face pairs — if this reads Vertex, the \
         face barrier is not being exercised and every convergence number below \
         describes the wrong code path",
    );
}

// ── the measurement ─────────────────────────────────────────────────

/// What one quasi-static press did — the convergence numbers **and** the
/// loading witness that says whether they describe a real load case.
///
/// A two-iteration solve at residual 1e-13 is a fine result and is also
/// precisely what a no-op looks like: a plate that never moved satisfies
/// equilibrium immediately. The last three fields are what separate the two.
/// Without them "Tet10 × Yeoh converges" would be a claim about an unloaded
/// plate wearing the words of a claim about contact.
/// One rung of a ramp: the indenter advance (m) and what the solve made of it.
///
/// Named because the raw tuple appears in every ramp signature and in the
/// cached accessors' return types, where `clippy::type_complexity` is right
/// that it had stopped being readable.
type Rung = (f64, Result<Press, String>);

#[derive(Debug, Clone)]
struct Press {
    iters: usize,
    residual: f64,
    /// Largest nodal displacement from rest (m).
    max_disp: f64,
    /// Net contact force on the solid along `+z` (N) at the converged pose.
    /// The plane is below with normal `+z`, so a loaded step is positive.
    ///
    /// ⚠ **One component. Use [`net_force`](Press::net_force) for any claim
    /// about the net force cancelling** — on the flat cell `+z` carries all of
    /// it, but on an enveloping patch the residual is spread across all three
    /// axes and `z` alone understates it (measured 1.64x at rest).
    net_force_z: f64,
    /// Net contact force as a VECTOR (N).
    ///
    /// The flat cell's normals all point `+z`, so `net_force_z` is the whole
    /// story there and this adds nothing. The cavity's do not: the residual
    /// after cancellation is an artefact of the mesh's asymmetry and has no
    /// preferred axis, so a claim that it cancels has to read all three.
    net_force: Vec3,
    /// Peak contact pressure (Pa) over `n_pairs` active pairs.
    peak_pressure: f64,
    n_pairs: usize,
    /// Tributary area (m²) of the pair that produced `peak_pressure`, and the
    /// median tributary area over all active pairs.
    ///
    /// `peak_pressure` is a max over per-pair `|force| / tributary_area`, so a
    /// pair with a vanishing area would dominate it. These two say whether that
    /// is happening rather than leaving it to inference.
    ///
    /// ⚠ **`peak_area` must filter non-finite pressures before taking the
    /// argmax, and the first version of it did not.** `f64::total_cmp` orders
    /// `NaN` **above** every finite value, so a bare
    /// `max_by(|a, b| a.pressure.total_cmp(&b.pressure))` returns a *degenerate*
    /// node, not the peak one. That read out a negative tributary area at every
    /// rung and looked like a defect in `peak_contact_pressure`. It is not:
    /// quadratic-triangle corner weights are exactly zero on a flat face and
    /// slightly negative on a curved one (`sim/L0/soft/src/contact/face.rs`), such nodes report
    /// `NaN` pressure by design, and `peak_contact_pressure` already filters
    /// them (`sim/L0/soft/src/contact/mod.rs`).
    peak_area: f64,
    median_area: f64,
    /// Active pairs whose pressure is non-finite — the degenerate corner nodes
    /// above. A count, not a defect: it is expected to be non-zero on a curved
    /// patch and zero on a flat one.
    degenerate_pairs: usize,
    /// Smallest signed distance over the active pairs at the converged pose (m).
    ///
    /// The interior-point guarantee is `sd > 0`, but `IpcRigidContact::barrier`
    /// clamps its argument (`d = sd.max(d_hat * 1e-6)`), so a solve CAN come to
    /// rest with nodes at or below the plane and still report a tiny residual —
    /// the clamp makes the energy finite where the true barrier is infinite.
    /// This is the field that tells a physically-standing-off solve apart from
    /// one resting on the clamp.
    min_sd: f64,
    /// Tributary-area-weighted **mean** signed distance over the active pairs
    /// (m).
    ///
    /// [`Press::min_sd`] is an order statistic; this is the distribution it is
    /// the bottom of. The pair matters: the barrier traction `kappa*|b'(d)|` is
    /// convex in `d`, so the *mean traction* over a patch with a spread of gaps
    /// is carried at an effective gap strictly between the minimum and the
    /// mean. Comparing a mean traction against `min_sd` is a mean read against
    /// an order statistic and reports a discrepancy that is the spread, not a
    /// defect.
    mean_sd: f64,
    /// Tributary-area-weighted mean of the barrier traction `κ·|b′(sd)|` over
    /// the active pairs (Pa).
    ///
    /// ★ **The force-free reading of the design traction.** [`net_force_z`] is
    /// a projection onto one axis and needs a patch whose normals all point
    /// along it; on an enveloping patch ([`Indenter::Bore`]) the radial forces
    /// cancel and that projection reads ~0 regardless of how hard the contact
    /// is working. This quantity does not project: it is the traction the face
    /// barrier itself integrates, read back at the converged pose.
    ///
    /// The plate is where the two definitions are checked against each other —
    /// see [`the_traction_mean_agrees_with_force_over_area_on_the_plate`].
    ///
    /// [`net_force_z`]: Press::net_force_z
    mean_traction: f64,
    /// Sum of per-pair force MAGNITUDES over the active pairs (N), and the
    /// deformed tributary area they are spread over (m^2).
    ///
    /// The magnitude sum is the enveloping-safe replacement for
    /// [`net_force_z`](Press::net_force_z): a vector sum cancels on a closed
    /// patch, `Σ|f|` does not.
    sum_force_mag: f64,
}

/// Displacement and contact readout at the converged pose.
fn summarize<M: sim_soft::Material>(
    mesh: &dyn Mesh<M>,
    setup: Setup,
    plane_h: f64,
    rest: &[f64],
    x_final: &[f64],
    iters: usize,
    residual: f64,
) -> Press {
    let max_disp = rest
        .chunks_exact(3)
        .zip(x_final.chunks_exact(3))
        .map(|(r, x)| Vec3::new(x[0] - r[0], x[1] - r[1], x[2] - r[2]).norm())
        .fold(0.0_f64, f64::max);
    let positions: Vec<Vec3> = x_final
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    let readouts = setup.against(plane_h).per_pair_readout(mesh, &positions);
    Press {
        iters,
        residual,
        max_disp,
        net_force_z: readouts.iter().map(|r| r.force_on_soft.z).sum(),
        net_force: readouts.iter().map(|r| r.force_on_soft).sum(),
        peak_pressure: peak_contact_pressure(&readouts),
        peak_area: readouts
            .iter()
            .filter(|r| r.pressure.is_finite())
            .max_by(|a, b| a.pressure.total_cmp(&b.pressure))
            .map_or(f64::NAN, |r| r.tributary_area),
        degenerate_pairs: readouts.iter().filter(|r| !r.pressure.is_finite()).count(),
        median_area: {
            let mut a: Vec<f64> = readouts.iter().map(|r| r.tributary_area).collect();
            a.sort_by(f64::total_cmp);
            a.get(a.len() / 2).copied().unwrap_or(f64::NAN)
        },
        n_pairs: readouts.len(),
        min_sd: readouts.iter().map(|r| r.sd).fold(f64::INFINITY, f64::min),
        mean_sd: {
            // Weighted by the deformed tributary patch each node speaks for.
            // Corner nodes report a non-positive area by design (see
            // `Press::peak_area`) and carry ~0 force, so they are excluded
            // rather than allowed to contribute negative weight.
            let (num, den) = readouts
                .iter()
                .filter(|r| r.tributary_area > 0.0)
                .fold((0.0, 0.0), |(n, d), r| {
                    (r.sd.mul_add(r.tributary_area, n), d + r.tributary_area)
                });
            if den > 0.0 { num / den } else { f64::NAN }
        },
        mean_traction: {
            // Same weights as `mean_sd` above, applied to the traction rather
            // than the gap. Weighting the traction and then inverting is not
            // the same as inverting the weighted gap — `|b′|` is convex — and
            // it is the traction that the barrier integrates, so it is the
            // traction that gets averaged.
            let (num, den) =
                readouts
                    .iter()
                    .filter(|r| r.tributary_area > 0.0)
                    .fold((0.0, 0.0), |(n, d), r| {
                        let t = setup.barrier.kappa
                            * barrier_derivative(r.sd, setup.barrier.d_hat).abs();
                        (t.mul_add(r.tributary_area, n), d + r.tributary_area)
                    });
            if den > 0.0 { num / den } else { f64::NAN }
        },
        sum_force_mag: readouts
            .iter()
            .filter(|r| r.tributary_area > 0.0)
            .map(|r| r.force_on_soft.norm())
            .sum(),
    }
}

/// One-line label for a fail-close surface.
///
/// `SolverFailure`'s derived `Debug` embeds `x_partial`, which on this fixture
/// is 24 993 floats — 2.3 MB of log per failed rung, which buries the one thing
/// the rung is being run to learn. This keeps the variant and its diagnostic
/// scalars and drops the state vector.
fn failure_label(e: &SolverFailure) -> String {
    match e {
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => format!("ArmijoStall(iter {last_iter}, r {last_r_norm:.3e})"),
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => format!("NewtonIterCap(max {max_iter}, r {last_r_norm:.3e})"),
        SolverFailure::DoublyFailedFactor {
            last_iter, context, ..
        } => format!("DoublyFailedFactor(iter {last_iter}, {context})"),
        SolverFailure::ValidityViolation {
            tet_id, message, ..
        } => format!("ValidityViolation(tet {tet_id}: {message})"),
    }
}

/// Zero-velocity, zero-θ arguments shared by all three presses.
fn step_inputs(rest: &[f64]) -> (Tensor<f64>, Tensor<f64>, Tensor<f64>) {
    let n_dof = rest.len();
    (
        Tensor::from_slice(rest, &[n_dof]),
        Tensor::from_slice(&vec![0.0; n_dof], &[n_dof]),
        Tensor::from_slice(&[], &[0]),
    )
}

/// Tet10 × Yeoh — the configuration item 3 needs and nothing has ever solved.
///
/// Uses `try_replay_step`, not `replay_step`: the four fail-close surfaces
/// (`ArmijoStall`, `NewtonIterCap`, `DoublyFailedFactor`, `ValidityViolation`)
/// come back as an `Err` variant naming which one fired, where the panicking
/// path would only abort. Which surface fires is the finding.
fn press_tet10_yeoh(setup: Setup, plane_h: f64) -> Result<Press, String> {
    plate_cell().press(setup, plane_h)
}

/// A Tet10 × Yeoh body and the boundary set that holds it.
///
/// The two cells differ in exactly these two things and in nothing about how
/// they are driven, which is why one [`Cell::press`] and one [`Cell::ramp`]
/// serve both. Built once per experiment and reused across a ramp's rungs —
/// meshing and enrichment are the expensive part, and neither depends on where
/// the indenter is.
struct Cell {
    mesh: Tet10Mesh<Yeoh>,
    pins: Vec<VertexId>,
    /// The indenter advance at rest contact, in [`Indenter::at`]'s convention:
    /// the soft surface sits inside the barrier band, essentially unloaded.
    rest_advance: f64,
    /// What the pinned set is, for the assertion message.
    pin_label: &'static str,
}

impl Cell {
    /// The 40 × 40 × 12 mm plate, top face pinned.
    fn plate() -> Self {
        let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4_yeoh());
        let pins = top_face_pins(&mesh);
        Self {
            mesh,
            pins,
            rest_advance: REST_PLANE_H,
            pin_label: "top face",
        }
    }

    /// The thick spherical shell, outer skin pinned.
    fn shell() -> Self {
        let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4_shell());
        let pins = outer_skin_pins(&mesh, OUTER_SKIN_BAND);
        Self {
            mesh,
            pins,
            rest_advance: REST_BORE_W,
            pin_label: "outer skin",
        }
    }

    /// The same shell carrying the three-layer stack instead of one anchor.
    fn graded_shell() -> Self {
        Self::capped_shell([
            STACK[0].validity_max_principal_stretch,
            STACK[1].validity_max_principal_stretch,
            STACK[2].validity_max_principal_stretch,
        ])
    }

    /// The graded shell with its per-layer tensile caps overridden — the lever
    /// [`the_per_tet_validity_gate_fires_in_the_layer_that_owns_the_cap`]
    /// pulls, since the real caps sit far above anything this cell reaches.
    fn capped_shell(caps: [f64; 3]) -> Self {
        Self::shell_with(graded_field_capped(caps))
    }

    /// The shell carrying an arbitrary material field.
    fn shell_with(field: MaterialField) -> Self {
        let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4_shell_with(CELL, field));
        let pins = outer_skin_pins(&mesh, OUTER_SKIN_BAND);
        Self {
            mesh,
            pins,
            rest_advance: REST_BORE_W,
            pin_label: "outer skin",
        }
    }

    /// Solve one pose from the rest configuration.
    fn press(&self, setup: Setup, advance: f64) -> Result<Press, String> {
        let rest = rest_dofs(&self.mesh);
        self.solve_from(setup, advance, &rest, &rest)
            .map(|(p, _)| p)
    }

    /// Solve one pose warm-started from `x_prev`, reporting displacement
    /// against `rest`.
    ///
    /// Returns the summary *and* the converged state, because a ramp needs
    /// both: the summary is what is reported, the state is the next rung's
    /// warm start.
    fn solve_from(
        &self,
        setup: Setup,
        advance: f64,
        rest: &[f64],
        x_prev: &[f64],
    ) -> Result<(Press, Vec<f64>), String> {
        assert!(
            !self.pins.is_empty(),
            "the pinned {} must not be empty",
            self.pin_label,
        );
        let (_, v, theta) = step_inputs(rest);
        let x = Tensor::from_slice(x_prev, &[x_prev.len()]);
        let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
            CpuNewtonSolver::new(
                Tet10,
                self.mesh.clone(),
                setup.against(advance),
                config(),
                BoundaryConditions::new(self.pins.clone(), Vec::new()),
            );
        solver
            .try_replay_step(&x, &v, &theta, STATIC_DT)
            .map(|s| {
                let p = summarize(
                    &self.mesh,
                    setup,
                    advance,
                    rest,
                    &s.x_final,
                    s.iter_count,
                    s.final_residual_norm,
                );
                (p, s.x_final)
            })
            .map_err(|e| failure_label(&e))
    }

    /// March the indenter from rest contact to `max_advance`, re-solving from
    /// each converged state, and stop at the first rung that fails.
    ///
    /// Returns one entry per attempted advance.
    fn ramp(&self, setup: Setup, step: f64, max_advance: f64) -> Vec<Rung> {
        let rest = rest_dofs(&self.mesh);
        let mut x_prev = rest.clone();
        let mut out = Vec::new();
        // Indexed, not accumulated: `h += step` drifts (the first cut of this
        // loop printed a +0.020000000000000025 mm plane height) and a float
        // loop condition is a lint besides.
        let n_steps = ((max_advance - self.rest_advance) / step).ceil() as usize;
        for i in 0..=n_steps {
            let advance = self.rest_advance + (i as f64) * step;
            match self.solve_from(setup, advance, &rest, &x_prev) {
                Ok((p, x_final)) => {
                    x_prev = x_final;
                    out.push((advance, Ok(p)));
                }
                Err(e) => {
                    out.push((advance, Err(e)));
                    break;
                }
            }
        }
        out
    }
}

/// Tet4 × Yeoh — the per-vertex barrier on the same plate, same `(κ, d̂)`.
///
/// ⛔ **Not a valid comparison, and kept because that is the finding.** It
/// reports `Ok` at a tiny residual with `min_sd = −3.52 mm` and a net "contact
/// force" of ~4e9 N where the face path reads single-digit newtons — because
/// 386 of its 607 pairs are on lattice vertices that belong to no tetrahedron
/// and sat below the plane before the solve started. See
/// [`the_vertex_barrier_contacts_vertices_that_are_in_no_tetrahedron`], which
/// measures it. Its iteration count is meaningful; its force and `min_sd` are
/// not a Tet4 property at all.
fn press_tet4_yeoh(setup: Setup, plane_h: f64) -> Result<Press, String> {
    let mesh = tet4_yeoh();
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet4, SdfMeshedTetMesh<Yeoh>, IpcRigidContact, Yeoh> =
        CpuNewtonSolver::new(
            Tet4,
            mesh.clone(),
            setup.against(plane_h),
            config(),
            BoundaryConditions::new(pins, Vec::new()),
        );
    solver
        .try_replay_step(&x, &v, &theta, STATIC_DT)
        .map(|s| {
            summarize(
                &mesh,
                setup,
                plane_h,
                &rest,
                &s.x_final,
                s.iter_count,
                s.final_residual_norm,
            )
        })
        .map_err(|e| failure_label(&e))
}

/// Tet10 × Neo-Hookean — the element baseline, and the configuration
/// `tet10_indentation_demand1` already exercises. Same plate, same contact.
fn press_tet10_nh(setup: Setup, plane_h: f64) -> Result<Press, String> {
    let tet4 = tet4_nh();
    let mesh = Tet10Mesh::<NeoHookean>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet10, Tet10Mesh<NeoHookean>, IpcRigidContact, NeoHookean, 10, 4> =
        CpuNewtonSolver::new(
            Tet10,
            mesh.clone(),
            setup.against(plane_h),
            config(),
            BoundaryConditions::new(pins, Vec::new()),
        );
    solver
        .try_replay_step(&x, &v, &theta, STATIC_DT)
        .map(|s| {
            summarize(
                &mesh,
                setup,
                plane_h,
                &rest,
                &s.x_final,
                s.iter_count,
                s.final_residual_norm,
            )
        })
        .map_err(|e| failure_label(&e))
}

#[test]
fn tet10_yeoh_converges_under_the_ipc_face_barrier() {
    let tet4 = tet4_yeoh();
    let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    eprintln!(
        "fixture: {} tets, Tet4 {} nodes -> Tet10 {} nodes ({} DOF)",
        tet4.n_tets(),
        tet4.n_vertices(),
        tet10.n_vertices(),
        3 * tet10.n_vertices(),
    );
    eprintln!(
        "  Tet4  x Yeoh        : {:?}",
        press_tet4_yeoh(BASELINE, REST_PLANE_H)
    );
    eprintln!(
        "  Tet10 x NeoHookean  : {:?}",
        press_tet10_nh(BASELINE, REST_PLANE_H)
    );

    let p = press_tet10_yeoh(BASELINE, REST_PLANE_H)
        .expect("Tet10 x Yeoh must converge under the IPC face barrier");
    eprintln!("  Tet10 x Yeoh        : {p:?}");

    // ⛔ Two assertions were removed here, and are named so they do not get
    // re-added: `p.residual.is_finite()` and `p.iters <= MAX_NEWTON_ITER`.
    // Neither can fail. Newton converges on `r_norm < self.config.tol`
    // (`sim/L0/soft/src/solver/backward_euler/newton.rs`), and `NaN < tol` is false, so a converged
    // step cannot carry a non-finite residual; exceeding the cap returns
    // `Err(SolverFailure::NewtonIterCap)`, so an `Ok` step is under the cap by
    // construction. Both read as safety checks and neither constrains anything
    // — the iteration COUNT is reported in the printout, where it is useful.

    // The loading witness. Without these three, the iteration count above is
    // compatible with a plate that never touched the plane.
    assert!(
        p.n_pairs > 0,
        "converged with no active contact pairs — nothing was solved",
    );
    assert!(
        p.net_force_z > 0.0 && p.net_force_z.is_finite(),
        "the barrier must push the plate up; net force along +z was {:e} N",
        p.net_force_z,
    );
    assert!(
        p.max_disp > 0.0,
        "the plate did not move: max displacement {:e} m",
        p.max_disp,
    );
    assert!(
        p.peak_pressure > 0.0 && p.peak_pressure.is_finite(),
        "peak contact pressure must be positive and finite; got {:e} Pa",
        p.peak_pressure,
    );
    // The assertion that actually certifies a CONTACT solve. Everything above
    // is satisfied by the Tet4 arm printed alongside, which "converges" to
    // residual 9.3e-13 while 3.5 mm INSIDE the plane.
    assert!(
        p.min_sd > 0.0,
        "the converged pose penetrates the plane by {:e} m — the barrier clamp \
         made that finite, it does not make it a contact solve",
        -p.min_sd,
    );
}

/// Plane travel per ramp increment (m).
///
/// Must stay below the standing barrier gap: each increment lifts the plane by
/// `RAMP_STEP` while the previous step left the deformed bottom face a gap
/// above it, so `RAMP_STEP < gap` is what keeps the next step's initial guess
/// intersection-free. 0.1 mm against `d̂ = 1.2` mm.
const RAMP_STEP: f64 = 0.0001;

/// Highest the plane is driven by the `κ` sweep (m) — 3 mm into a 12 mm plate.
const RAMP_MAX_PLANE_H: f64 = 0.0030;

/// Highest the plane is driven by the always-on envelope gate (m).
///
/// Shallower than the sweep's ceiling on purpose: 1 mm is 8.3 % engineering
/// compression, already well past where Yeoh's `C₂` term separates from
/// Neo-Hookean, and it costs ~15 increments instead of ~36. The gate's job is
/// to certify that the ramp reaches real strain, not to re-measure how deep it
/// can go — that is [`the_armijo_wall_against_barrier_stiffness`]'s job.
const GATE_MAX_PLANE_H: f64 = 0.0010;

/// March the plane up through the plate, re-solving from each converged state,
/// and stop at the first rung that fails.
///
/// Returns one entry per attempted plane height. The mesh, pins and rest
/// configuration are built once; only the contact primitive changes per rung,
/// which is the same per-increment rebuild `tet10_indentation_demand1` uses.
fn ramp_tet10_yeoh(setup: Setup, step: f64, max_plane_h: f64) -> Vec<Rung> {
    plate_cell().ramp(setup, step, max_plane_h)
}

// ── shared fixtures ─────────────────────────────────────────────────
//
// Meshing and Tet10 enrichment cost more than a shallow ramp does, and neither
// depends on where the indenter is — so each cell is built once per test
// binary rather than once per gate. `OnceLock` because the harness runs tests
// on several threads and `get_or_init` is the only initialisation that is
// safe under that without a lock held across the solve.

/// The 40 × 40 × 12 mm plate, built once.
fn plate_cell() -> &'static Cell {
    static CELL_ONCE: OnceLock<Cell> = OnceLock::new();
    CELL_ONCE.get_or_init(Cell::plate)
}

/// The uniform thick spherical shell, built once.
fn shell_cell() -> &'static Cell {
    static CELL_ONCE: OnceLock<Cell> = OnceLock::new();
    CELL_ONCE.get_or_init(Cell::shell)
}

/// The graded shell, built once.
fn graded_cell() -> &'static Cell {
    static CELL_ONCE: OnceLock<Cell> = OnceLock::new();
    CELL_ONCE.get_or_init(Cell::graded_shell)
}

/// The always-on plate ramp at [`BASELINE`], solved once and shared.
///
/// ⚠ Cached by NOTHING but the fact that every always-on caller asks for the
/// same `(setup, step, depth)`. A caller wanting different parameters must
/// call [`ramp_tet10_yeoh`] directly — this returns the `BASELINE` ramp
/// whatever it is asked, because it is not asked.
fn plate_rungs() -> &'static [Rung] {
    static RUNGS: OnceLock<Vec<Rung>> = OnceLock::new();
    RUNGS.get_or_init(|| ramp_tet10_yeoh(BASELINE, RAMP_STEP, GATE_MAX_PLANE_H))
}

/// The always-on uniform cavity ramp, solved once and shared across the four
/// gates that read it.
fn cavity_rungs() -> &'static [Rung] {
    static RUNGS: OnceLock<Vec<Rung>> = OnceLock::new();
    RUNGS.get_or_init(|| ramp_cavity(CAVITY, RAMP_STEP, CAVITY_GATE_MAX_W))
}

/// The always-on graded cavity ramp, solved once and shared.
fn graded_rungs() -> &'static [Rung] {
    static RUNGS: OnceLock<Vec<Rung>> = OnceLock::new();
    RUNGS.get_or_init(|| ramp_graded(CAVITY_GATE_MAX_W))
}

/// Does the ramp reach strain where Yeoh actually differs from Neo-Hookean?
///
/// The resting press above answers "does it solve", and the answer is yes — at
/// 7 µm of deflection, 0.06 % strain. At that amplitude Yeoh and Neo-Hookean
/// agree to four significant figures, so the resting press on its own does not
/// exercise the constitutive model that is the whole reason item 1 made
/// `Tet10Mesh` generic. This walks the plane up through the plate under a
/// pinned top face — a direct compression — to 8.3 %, and reports every rung.
///
/// ⚠ **The plane must be MARCHED, not placed.** A first cut at this test set
/// the plane straight to each target depth from the rest configuration and got
/// `ArmijoStall(iter 0)` at every depth from 1.67 % to 33 % strain — *and
/// identical residuals for Yeoh and Neo-Hookean*, which is the tell: a failure
/// that does not move when the constitutive model changes is not a
/// constitutive failure. The IPC barrier is undefined at `d ≤ 0`, so a plane
/// placed inside the material starts the very first line search from an
/// infeasible point. Every increment here therefore re-solves from the
/// previous converged state, the same way `tet10_indentation_demand1` and
/// `insertion_sim` march theirs.
///
/// ⚠ **The depth reached is printed, not asserted.** Pinning it would pin a
/// number that `κ`, `d̂`, mesh resolution and element order all move — and
/// moving exactly those is item 3's job. What IS asserted is the shape: the
/// ramp clears rungs, every cleared rung is loaded and non-penetrating, and
/// the deepest one moved the plate an order of magnitude further than the
/// first. An earlier revision asserted a wall here; see the module header for
/// why that was retracted.
#[test]
fn tet10_yeoh_convergence_envelope_under_increasing_compression() {
    let thickness = 2.0 * HALF[2];
    eprintln!(
        "compression ramp, {:.0} mm plate, kappa {KAPPA:e}, d_hat {D_HAT:e}, \
         step {:.2} mm",
        thickness * 1e3,
        RAMP_STEP * 1e3,
    );

    let rungs = plate_rungs();
    for (h, r) in rungs {
        match r {
            // `defl` is the largest nodal displacement as a fraction of plate
            // thickness. An earlier revision printed plane travel here and
            // called it strain, which read 0.00 % on a plate already deflected
            // 0.63 mm — the plane is still below the rest face while the
            // barrier band is loading it.
            Ok(p) => eprintln!(
                "  plane {:+.2} mm: defl {:5.2} % of thickness, iters {:2}, r {:.2e}, \
                 disp {:.4} mm, Fz {:.4e} N, p_peak {:.3e} Pa, pairs {}",
                h * 1e3,
                100.0 * p.max_disp / thickness,
                p.iters,
                p.residual,
                p.max_disp * 1e3,
                p.net_force_z,
                p.peak_pressure,
                p.n_pairs,
            ),
            Err(e) => eprintln!("  plane {:+.2} mm: {e}", h * 1e3),
        }
    }

    let converged: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
    assert!(
        converged.len() >= 2,
        "the ramp must clear at least two rungs before it stops; it cleared {}",
        converged.len(),
    );
    for p in &converged {
        assert!(
            p.n_pairs > 0 && p.net_force_z > 0.0 && p.net_force_z.is_finite(),
            "a converged rung must be loaded: {p:?}",
        );
        assert!(
            p.min_sd > 0.0,
            "a converged rung must not penetrate the plane: {p:?}",
        );
    }
    let deepest = converged.last().expect("checked non-empty above");
    assert!(
        deepest.max_disp > converged[0].max_disp,
        "the ramp must load monotonically: deepest rung moved {:e} m against the \
         first rung's {:e} m",
        deepest.max_disp,
        converged[0].max_disp,
    );
    // The load must reach a regime where the constitutive model is doing work.
    // Stated against plate thickness rather than as a ratio to the first rung:
    // an earlier revision asserted "10x the first rung", which was calibrated
    // when a soft barrier made the first rung 7 um and silently became
    // unsatisfiable once the first rung itself deflected 0.63 mm.
    let deflection_fraction = deepest.max_disp / thickness;
    assert!(
        deflection_fraction > 0.10,
        "the ramp must reach a real load: deepest deflection {:.4} mm is {:.2} % \
         of a {:.0} mm plate, under the 10 % this gate exists to certify",
        deepest.max_disp * 1e3,
        deflection_fraction * 100.0,
        thickness * 1e3,
    );
}

/// Is the envelope's end a step-size artefact, or a wall?
///
/// ⛔ **Runs at [`SOFT_BARRIER`] (`κ = 1e4`), not at `BASELINE`** — without
/// that, every number below reads as a `κ = 1e7` result, and at `κ = 1e7`
/// there is no wall to ask about: the ramp runs clean to the ceiling. This
/// experiment only has a subject at the soft barrier, which is why it is
/// pinned there rather than following the baseline.
///
/// `#[ignore]` — two full ramps, ~40 s. It is the discriminating experiment
/// for the one question the envelope test cannot answer from a single step
/// size: refining the increment either walks the ramp arbitrarily deep (in
/// which case the end is bookkeeping) or moves it a little and hits the same
/// fail-close surface (in which case something real is there).
///
/// Measured 2026-09-20 at `d12e3bf9` + this fixture, 4 240 tets:
///
/// | step | rungs | deepest plane | max displacement | ends with |
/// |------|-------|---------------|------------------|-----------|
/// | 0.100 mm | 6 | +0.020 mm | 0.0995 mm | `ArmijoStall(iter 0, r 7.773e2)` |
/// | 0.025 mm | 23 | +0.070 mm | 0.1455 mm | `ArmijoStall(iter 0, r 5.747e2)` |
///
/// ⇒ 4× refinement buys 46 % more deflection and does **not** remove the wall.
/// The stall is at Newton iteration 0 both times: the first line search out of
/// the increment's initial guess cannot find a decrease.
///
/// ⚠ Not the same signature as the open `insertion_sim` pathology.
/// `sliding_insertion_ramp_converges_on_synthetic_icosphere` stalls at
/// **iter 23, r 6.143e-1** — deep into Newton at a small residual. This stalls
/// at **iter 0** at a residual three orders larger. Whether they share a cause
/// has not been established, and this fixture is not evidence that they do.
#[test]
#[ignore = "two full compression ramps, ~40 s — run it when (kappa, d_hat), \
           mesh resolution or element order change"]
fn ramp_increment_refinement_extends_the_envelope_without_removing_it() {
    let mut reached = Vec::new();
    for step in [RAMP_STEP, RAMP_STEP / 4.0] {
        let rungs = ramp_tet10_yeoh(SOFT_BARRIER, step, RAMP_MAX_PLANE_H);
        let ok: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
        let err = rungs.iter().find_map(|(_, r)| r.as_ref().err()).cloned();
        let deepest_plane = rungs.iter().rev().find(|(_, r)| r.is_ok()).map(|(h, _)| *h);
        let max_disp = ok.last().map_or(0.0, |p| p.max_disp);
        eprintln!(
            "  step {:.3} mm: {} rungs, deepest plane {:+.3} mm, max disp {:.4} mm, ends {}",
            step * 1e3,
            ok.len(),
            deepest_plane.unwrap_or(f64::NAN) * 1e3,
            max_disp * 1e3,
            err.clone()
                .unwrap_or_else(|| "(ran to RAMP_MAX_PLANE_H)".into()),
        );
        reached.push((max_disp, err));
    }

    let (coarse_disp, coarse_err) = &reached[0];
    let (fine_disp, fine_err) = &reached[1];
    assert!(
        fine_disp > coarse_disp,
        "refining the increment must reach deeper: fine {fine_disp:e} m vs coarse {coarse_disp:e} m",
    );
    // The wall is the finding. If refinement ever runs a ramp clean to
    // `RAMP_MAX_PLANE_H`, this assertion is what says so out loud rather than
    // letting the table above quietly go stale.
    assert!(
        coarse_err.is_some() && fine_err.is_some(),
        "both ramps were expected to stop early; coarse {coarse_err:?}, fine {fine_err:?}",
    );
}

// ── is the envelope κ-conditioned? ──────────────────────────────────

/// `κ` must actually reach the solver.
///
/// Pre-registered *before* the sweep below, because a sweep whose parameter is
/// silently dropped produces the most misleading possible result: every row
/// identical, which reads as "κ does not matter" — a finding — rather than as
/// "κ was never applied" — a bug. This is the threading check that tells the
/// two apart, and it is definitional rather than empirical: the IPC barrier
/// energy is linear in `κ`, so its force cannot be invariant to it.
#[test]
fn kappa_reaches_the_solver() {
    let soft = SOFT_BARRIER;
    let stiff = Setup {
        barrier: Barrier {
            kappa: 10.0 * SOFT_BARRIER.barrier.kappa,
            d_hat: D_HAT,
        },
        indenter: Indenter::Plane,
    };
    let a = press_tet10_yeoh(soft, REST_PLANE_H).expect("soft kappa must converge at rest");
    let b = press_tet10_yeoh(stiff, REST_PLANE_H).expect("10x kappa must converge at rest");
    eprintln!("  kappa {:e}: Fz {:e} N", soft.barrier.kappa, a.net_force_z);
    eprintln!(
        "  kappa {:e}: Fz {:e} N",
        stiff.barrier.kappa, b.net_force_z
    );
    assert!(
        (a.net_force_z - b.net_force_z).abs() > 0.0,
        "10x kappa produced an identical contact force ({:e} N) — kappa is not \
         reaching the barrier, and any sweep over it is measuring nothing",
        a.net_force_z,
    );
}

/// Is the `ArmijoStall` wall a property of Tet10 × Yeoh, or of `κ = 1e4`?
///
/// `#[ignore]` — one full ramp per `κ`, minutes. The envelope test varies the
/// *increment* and holds `κ` fixed, so it supports "the wall is not a step
/// artefact" and nothing more. A comparison supports only what it varied, and
/// the headline "Tet10 × Yeoh walls at ~1 % strain" quietly reads as a
/// statement about the element and material when it may be a statement about a
/// barrier stiffness carried over from a fixture 8× larger.
///
/// `d̂` is held: sweeping it would move `STANDOFF` and therefore the initial
/// condition, which is a different experiment.
#[test]
#[ignore = "one full compression ramp per kappa, several minutes — run it when \
            the barrier parameters or the fixture geometry change"]
fn the_armijo_wall_against_barrier_stiffness() {
    eprintln!(
        "kappa sweep, ramp step {:.3} mm, d_hat {D_HAT:e} held",
        RAMP_STEP * 1e3
    );
    let mut rows = Vec::new();
    for exp in 2..=8 {
        let setup = Setup {
            barrier: Barrier {
                kappa: 10f64.powi(exp),
                d_hat: D_HAT,
            },
            indenter: Indenter::Plane,
        };
        let rungs = ramp_tet10_yeoh(setup, RAMP_STEP, RAMP_MAX_PLANE_H);
        let ok: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
        let deepest_plane = rungs.iter().rev().find(|(_, r)| r.is_ok()).map(|(h, _)| *h);
        let err = rungs.iter().find_map(|(_, r)| r.as_ref().err()).cloned();
        let max_disp = ok.last().map_or(0.0, |p| p.max_disp);
        let min_sd = ok.last().map_or(f64::NAN, |p| p.min_sd);
        eprintln!(
            "  kappa {:8.0e}: {:2} rungs, deepest plane {:+.3} mm, disp {:.4} mm, \
             min_sd {:+.4} mm, ends {}",
            setup.barrier.kappa,
            ok.len(),
            deepest_plane.unwrap_or(f64::NAN) * 1e3,
            max_disp * 1e3,
            min_sd * 1e3,
            err.clone()
                .unwrap_or_else(|| "(ran clean to the ceiling)".into()),
        );
        // Whatever kappa does to the envelope, it must never buy convergence by
        // letting the plate through the plane.
        for p in &ok {
            assert!(
                p.min_sd > 0.0,
                "kappa {:e} converged a penetrating rung: {p:?}",
                setup.barrier.kappa,
            );
        }
        rows.push((setup.barrier.kappa, max_disp, min_sd, err));
    }

    let depths: Vec<f64> = rows.iter().map(|(_, d, _, _)| *d).collect();
    let spread = depths.iter().copied().fold(f64::MIN, f64::max)
        / depths
            .iter()
            .copied()
            .fold(f64::MAX, f64::min)
            .max(f64::MIN_POSITIVE);
    eprintln!("  deepest/shallowest displacement across 6 orders of kappa: {spread:.2}x");

    // The sweep exists to answer whether the wall is kappa. It is: 1e7 and 1e8
    // run clean to RAMP_MAX_PLANE_H where 1e4 stops at 0.83 % compression.
    assert!(
        rows.iter().any(|(_, _, _, e)| e.is_none()),
        "no kappa in the sweep cleared the ramp — the wall would then NOT be \
         a kappa artefact, and the module header's retraction is wrong",
    );
    assert!(
        rows.iter().any(|(_, _, _, e)| e.is_some()),
        "every kappa cleared the ramp — the sweep has no failing arm and so \
         cannot support a claim about what kappa fixes",
    );

    // And WHY it is kappa, as a relation rather than a story: a stall at Newton
    // iteration 0 is an infeasible starting point, which happens exactly when
    // the increment outruns the standoff the barrier is holding. Every iter-0
    // row must therefore have ended with less clearance than one step.
    for (kappa, _, min_sd, err) in &rows {
        if err.as_deref().is_some_and(stalled_on_the_first_newton_step) {
            assert!(
                *min_sd < RAMP_STEP,
                "kappa {kappa:e} stalled at Newton iteration 0 while holding \
                 {min_sd:e} m of clearance, which is MORE than the {RAMP_STEP:e} m \
                 increment — the infeasible-start explanation does not cover \
                 this row and something else is happening",
            );
        }
    }
}

/// Did this failure land on the very first Newton step?
///
/// Reads the label [`failure_label`] produces, twenty lines up — an iter-0
/// stall means the line search never found a decrease from the increment's
/// starting point, i.e. the start was already infeasible, which is a different
/// diagnosis from a stall part-way through a converging solve.
fn stalled_on_the_first_newton_step(label: &str) -> bool {
    label.starts_with("ArmijoStall(iter 0,")
}

// ── positions() is not the body ──────────────────────────────

/// The per-vertex barrier contacts vertices that belong to no tetrahedron.
///
/// This is the corrected form of a claim an earlier revision of this file got
/// wrong. It reported that the Tet4 arm "converges to a pose 3.52 mm inside the
/// plane", implying Newton descended through the barrier's clamped infinity.
/// It does not. `min_sd` is **−3.52 mm in the rest configuration, before any
/// solve**, and the solve does not move it — the value is identical at
/// `κ = 1e4` and `κ = 1e7`, which a penetration driven by barrier stiffness
/// could not be.
///
/// What is actually happening, measured below:
///
/// - `SdfMeshedTetMesh::positions()` carries the BCC lattice, not just the
///   body: 2 348 entries of which **1 104 are referenced by a tet and 1 244 are
///   dead**. The body itself is correct — `boundary_faces()` spans exactly
///   `z ∈ [0, 12] mm` — but the lattice around it spans `z ∈ [−4, +18] mm`.
/// - Every node below the plane at rest is dead. **Zero live vertices
///   penetrate**, at any `κ`.
/// - `IpcRigidContact::active_vertex_pairs` iterates `positions()`, so **386 of
///   the 607 Tet4 pairs sit on vertices in no tetrahedron** — carrying no
///   elastic force, placed wherever the lattice put them, and turned by the
///   barrier's `d = sd.max(d̂ · 1e-6)` clamp into ~4e9 N of reported force.
/// - The face path is immune: it iterates `boundary_faces6()`, which is built
///   from tet connectivity and therefore contains only live vertices.
///
/// ★ **The solver already defends against this; the contact models do not.**
/// `backward_euler/construct.rs` walks tet incidence and unions every
/// unreferenced vertex into `effective_pinned`, because "an orphan free DOF
/// would have zero mass ... AND zero element contribution to its Hessian
/// row/column, leaving a singular diagonal that faer's Cholesky would either
/// panic on ... or silently produce garbage" — and its comment names this
/// exact case, that `SdfMeshedTetMesh` "retains the full BCC lattice in
/// `positions()`". So the orphans never reach the linear system, which is why
/// this fixture is well-posed.
///
/// ✅ **They used to reach `active_pairs`; since #953 they do not.** Both
/// contact models' `active_pairs` now consult `referenced_vertex_mask`, so the
/// gap that was specifically in the contact models is closed. This test still
/// measures the underlying exposure because it reads `per_pair_readout`, which
/// keeps the unfiltered list by design — so the numbers below are the fixture's
/// dead-node census, not a live defect in the active set.
///
/// ⚠⚠ **It reached the shipped sim, and there it was much worse.**
/// `PenaltyRigidContact::active_pairs` took `_mesh` — it ignored the mesh
/// entirely and looped the same `positions()` — and `insertion_sim` is
/// `SdfMeshedTetMesh<Yeoh>` + penalty contact. Measured on its synthetic
/// sliding fixture (icosphere r = 40 mm, 3 mm cavity inset, 10 mm wall, 4 mm
/// cell), 2026-09-20, BEFORE the filter:
///
/// | quantity | value |
/// |---|---|
/// | `positions()` | 41 432 |
/// | live (in some tet) | 9 580 |
/// | **dead** | **31 852 — 76.9 %** |
/// | active pairs @ interference 0 mm | 6 981, **6 583 dead = 94.3 %** |
/// | active pairs @ interference 3 mm | 9 724, **6 618 dead = 68.1 %** |
///
/// A shell body's cavity is exactly where the BCC lattice's dead nodes sit and
/// the intruder is driven into the middle of them, so this fixture's solid
/// plate understates the effect badly.
///
/// ★ **It was waste, not corruption** — established, not assumed. A vertex
/// pair's gradient is `contributions: vec![(vertex_id, force)]`, touching its
/// own DOF and no other; orphans are auto-pinned out of the free system; and
/// readouts are filtered by `filter_pair_readouts_to_referenced`. So the answer
/// was right, and ~6 600 SDF evaluations and gradient builds per Newton
/// iteration were discarded. #953 verified that by hashing a converged step
/// either side of the filter: bit-identical on both contact models.
///
/// ⚠ The two `insertion_sim` paths were not equally defended, and the reason
/// is worth keeping: `intruder_contact_at` passed **no interior cutoff**, while
/// the sliding builder passed `2 × cavity_inset_m`. That cutoff is a DEPTH
/// heuristic, not an incidence check — applied to the same dead set it still
/// left 2 864 active at 0 mm, because it only excludes nodes deeper than `c`.
/// The incidence filter is what made it complete; see
/// `tests/contact_incidence_filter.rs`.
#[test]
fn the_vertex_barrier_contacts_vertices_that_are_in_no_tetrahedron() {
    let t4 = tet4_yeoh();
    let rest: Vec<Vec3> = t4.positions().to_vec();
    let live: std::collections::BTreeSet<VertexId> = referenced_vertices(&t4 as &dyn Mesh<Yeoh>)
        .into_iter()
        .collect();

    let dead = rest.len() - live.len();
    let dead_below = (0..rest.len())
        .filter(|i| !live.contains(&(*i as VertexId)) && rest[*i].z < REST_PLANE_H)
        .count();
    let live_below = (0..rest.len())
        .filter(|i| live.contains(&(*i as VertexId)) && rest[*i].z < REST_PLANE_H)
        .count();

    let ro4 = BASELINE.against(REST_PLANE_H).per_pair_readout(&t4, &rest);
    let on_dead = ro4
        .iter()
        .filter(|r| match r.pair {
            ContactPair::Vertex { vertex_id, .. } => !live.contains(&vertex_id),
            _ => false,
        })
        .count();
    let min_sd4 = ro4.iter().map(|r| r.sd).fold(f64::INFINITY, f64::min);

    let t10 = Tet10Mesh::<Yeoh>::from_tet4(&t4);
    let quadratic_rest: Vec<Vec3> = t10.positions().to_vec();
    let face_readout = BASELINE
        .against(REST_PLANE_H)
        .per_pair_readout(&t10, &quadratic_rest);
    let min_sd10 = face_readout
        .iter()
        .map(|r| r.sd)
        .fold(f64::INFINITY, f64::min);

    eprintln!(
        "  positions {}, live {}, dead {dead}",
        rest.len(),
        live.len()
    );
    eprintln!("  below the plane AT REST: {dead_below} dead, {live_below} live");
    eprintln!(
        "  vertex-path pairs on dead vertices: {on_dead} of {}",
        ro4.len()
    );
    eprintln!("  REST min_sd: vertex path {min_sd4:+.6} m, face path {min_sd10:+.6} m");

    assert!(
        dead > 0,
        "this fixture's premise is that SdfMeshedTetMesh retains dead lattice \
         nodes; it retained none, so the rest of this test proves nothing",
    );
    assert_eq!(
        live_below, 0,
        "a LIVE vertex below the plane at rest would mean the body itself is \
         penetrating, which is a different and worse problem than dead lattice",
    );
    assert!(
        on_dead > 0 && min_sd4 < 0.0,
        "the per-vertex path was expected to pick up dead vertices ({on_dead} \
         pairs, min_sd {min_sd4:e}) — if it no longer does, the upstream \
         behaviour changed and this file's Tet4 commentary is stale",
    );
    assert!(
        min_sd10 > 0.0,
        "the face path must see only live boundary vertices; it reported \
         min_sd {min_sd10:e}",
    );
}

// ── the curved cell ─────────────────────────────────────────────────

/// The curved cell must actually carry curvature.
///
/// The analogue of the `boundary_faces6` selector gate, one level up: there,
/// the risk was measuring the vertex path while believing it was the face
/// path; here, it is running a "curved" fixture whose barrier tangent is
/// arithmetically identical to the flat one. `Sdf::hessian` defaults to the
/// **zero matrix**, so an indenter that forgets to override it is flat as far
/// as the assembled tangent is concerned, however round it looks.
#[test]
fn the_curved_cell_carries_a_nonzero_hessian_and_the_flat_one_does_not() {
    let flat = Indenter::Plane.at(REST_PLANE_H);
    let curved = Indenter::Sphere.at(REST_PLANE_H);

    // Both surfaces pass through the same apex point, so the two cells are
    // positioned comparably rather than differing in standoff as well as shape.
    let apex = Point3::from(Vec3::new(0.0, 0.0, REST_PLANE_H));
    assert!(
        flat.eval(apex).abs() < 1.0e-12 && curved.eval(apex).abs() < 1.0e-12,
        "both indenters must have their surface at REST_PLANE_H: flat {:e}, curved {:e}",
        flat.eval(apex),
        curved.eval(apex),
    );

    // Inside the contact patch, where it matters.
    let probe = Point3::from(Vec3::new(0.005, 0.0, 0.0));
    let flat_h = flat.hessian(probe);
    let curved_h = curved.hessian(probe);
    eprintln!(
        "  |hessian| at the probe: flat {:e}, curved {:e}",
        flat_h.norm(),
        curved_h.norm()
    );
    assert_eq!(
        flat_h,
        nalgebra::Matrix3::zeros(),
        "a plane's signed distance is affine, so its hessian must be exactly zero",
    );
    assert!(
        curved_h.norm() > 0.0 && curved_h.norm().is_finite(),
        "the sphere must contribute a non-zero curvature term; it gave {:e}",
        curved_h.norm(),
    );

    // The barrier's curvature term scales with 1/R, so a sphere that is
    // effectively flat at this scale would pass the test above while changing
    // nothing. Check the magnitude is the 1/R the geometry implies.
    let expected = 1.0 / SPHERE_R;
    assert!(
        (curved_h.norm() - expected * 2.0_f64.sqrt()).abs() < 0.2 * expected,
        "a sphere's hessian is (I - n n^T)/|p-c|, norm sqrt(2)/R = {:e} near the \
         apex; got {:e}",
        expected * 2.0_f64.sqrt(),
        curved_h.norm(),
    );
}

/// Does Tet10 × Yeoh converge when the contact carries curvature?
///
/// This is the cell `insertion_sim` has and the one a flat fixture
/// structurally cannot reach. Reported against the flat cell at the same
/// heights, same `(κ, d̂)`, same mesh — the only thing varied is the shape.
#[test]
fn tet10_yeoh_converges_against_a_curved_indenter() {
    let thickness = 2.0 * HALF[2];
    let rest = press_tet10_yeoh(CURVED, REST_PLANE_H);
    eprintln!("  curved, rest contact: {rest:?}");
    let rest = rest.expect("Tet10 x Yeoh must converge against a curved indenter at rest");

    assert!(
        rest.n_pairs > 0,
        "the curved indenter must engage: {rest:?}",
    );
    assert!(
        rest.min_sd > 0.0,
        "curved contact must not penetrate: min_sd {:e} m",
        rest.min_sd,
    );
    assert!(
        rest.net_force_z > 0.0 && rest.net_force_z.is_finite(),
        "curved contact must push the plate up; got {:e} N",
        rest.net_force_z,
    );

    // A curved indenter engages a PATCH, not the whole face. If it engaged as
    // many pairs as the plane, the sphere is flat at this scale and the cell is
    // curved in name only.
    let flat = press_tet10_yeoh(BASELINE, REST_PLANE_H).expect("flat cell converges");
    eprintln!("  flat,   rest contact: {flat:?}");
    assert!(
        rest.n_pairs < flat.n_pairs,
        "the sphere must engage fewer pairs than the plane ({} vs {}) or it is \
         not meaningfully curved at this scale",
        rest.n_pairs,
        flat.n_pairs,
    );

    let rungs = ramp_tet10_yeoh(CURVED, RAMP_STEP, GATE_MAX_PLANE_H);
    for (h, r) in &rungs {
        match r {
            Ok(p) => eprintln!(
                "  apex {:+.2} mm: defl {:5.2} % of thickness, iters {:2}, r {:.2e}, \
                 disp {:.4} mm, Fz {:.4e} N, p_peak {:.3e} Pa (area {:.2e} vs median \
                 {:.2e} m2), pairs {} ({} degenerate), min_sd {:+.4} mm",
                h * 1e3,
                100.0 * p.max_disp / thickness,
                p.iters,
                p.residual,
                p.max_disp * 1e3,
                p.net_force_z,
                p.peak_pressure,
                p.peak_area,
                p.median_area,
                p.n_pairs,
                p.degenerate_pairs,
                p.min_sd * 1e3,
            ),
            Err(e) => eprintln!("  apex {:+.2} mm: {e}", h * 1e3),
        }
    }
    let converged: Vec<&Press> = rungs.iter().filter_map(|(_, r)| r.as_ref().ok()).collect();
    for p in &converged {
        assert!(
            p.min_sd > 0.0 && p.n_pairs > 0 && p.net_force_z > 0.0,
            "a converged curved rung must be loaded and non-penetrating: {p:?}",
        );
    }
    assert!(
        converged.len() >= 2,
        "the curved ramp must clear at least two rungs; it cleared {}",
        converged.len(),
    );
}

// ── the face-barrier traction law ───────────────────────────────────

/// Rest-pose contact geometry at a prescribed uniform gap, so the barrier
/// arithmetic can be checked against the shipped solver path without a solve.
struct RestContact {
    /// Every active face's rest area summed (m^2).
    active_area: f64,
    /// Rest area of only those faces lying wholly in the plate's bottom plane
    /// (m^2) — the faces whose six Gauss points all sit at the prescribed gap.
    flat_area: f64,
    /// Net barrier force on the solid along +z (N), from the shipped readout.
    net_force_z: f64,
    /// Number of active face pairs — a direct proxy for surface resolution.
    /// The contact *area* alone does not see a remesh: at [`CELL`] = 5 mm the
    /// 40 mm footprint still tiles to exactly 1600 mm^2 with a different
    /// number of faces, and a different discretisation carries a different
    /// traction.
    n_faces: usize,
}

/// Place the undeformed Tet10 plate `gap` above a plane and read the barrier.
fn rest_contact_at(gap: f64, kappa: f64) -> RestContact {
    let tet4 = tet4_yeoh();
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    let positions = mesh.positions().to_vec();
    let contact = IpcRigidContact::with_params(vec![Indenter::Plane.at(-gap)], kappa, D_HAT);
    let pairs = ActivePairsFor::<Yeoh>::active_pairs(&contact, &mesh, &positions);
    let mut active_area = 0.0;
    let mut flat_area = 0.0;
    for p in &pairs {
        if let ContactPair::Face {
            rest_area, nodes, ..
        } = p
        {
            active_area += *rest_area;
            if nodes.iter().all(|n| positions[*n as usize].z.abs() < 1e-9) {
                flat_area += *rest_area;
            }
        }
    }
    RestContact {
        active_area,
        flat_area,
        n_faces: pairs.len(),
        net_force_z: contact
            .per_pair_readout(&mesh, &positions)
            .iter()
            .map(|r| r.force_on_soft.z)
            .sum(),
    }
}

/// Does `F = kappa * A * |b'(gap)|` actually describe the shipped face barrier,
/// and **which `A`**?
///
/// This is the relation the derived kappa stands on, so it is measured against
/// the solver rather than asserted from the energy expression. The subtlety is
/// the area: a face is active if **any** of its six nodes is inside the band,
/// so the plate's vertical side walls join the active set while most of their
/// Gauss points sit outside it and contribute nothing. Summing rest area over
/// every active face therefore over-counts the load-bearing surface, and the
/// load-bearing area is the flat bottom face.
///
/// The gate is two-sided on purpose. The flat area must **under**-predict the
/// force (the side walls add a little, they cannot subtract), and the excess
/// must stay small — if it ever grew large, "the flat face carries the load"
/// would have stopped being true and every kappa derived from it would be
/// wrong by that factor.
#[test]
fn the_face_barrier_traction_law_holds_against_the_solver() {
    eprintln!("face-barrier traction law at rest, d_hat {D_HAT:e} m, kappa {KAPPA:e}");
    eprintln!(
        "  {:>8} {:>11} {:>11} {:>7} {:>13} {:>13} {:>8}",
        "gap/mm", "A_act/mm2", "A_flat/mm2", "flat%", "F_meas/N", "F_pred/N", "excess",
    );
    let mut excesses = Vec::new();
    for frac in [0.2, 0.4, 0.6, 0.8] {
        let gap = frac * D_HAT;
        let c = rest_contact_at(gap, KAPPA);
        assert!(c.flat_area > 0.0, "gap {gap:e}: no flat face in contact");
        // The shipped barrier, not a local copy of its formula.
        let predicted = KAPPA * c.flat_area * barrier_derivative(gap, D_HAT).abs();
        let excess = c.net_force_z / predicted - 1.0;
        eprintln!(
            "  {:8.4} {:11.2} {:11.2} {:6.1}% {:13.6e} {:13.6e} {:+7.2}%",
            gap * 1e3,
            c.active_area * 1e6,
            c.flat_area * 1e6,
            100.0 * c.flat_area / c.active_area,
            c.net_force_z,
            predicted,
            100.0 * excess,
        );
        assert!(
            excess > -1e-12,
            "gap {gap:e}: the flat bottom face alone predicts MORE force \
             ({predicted:e} N) than the whole active set produces \
             ({:e} N) — the side walls cannot subtract force, so either the \
             flat-area identification or the traction law is wrong",
            c.net_force_z,
        );
        excesses.push((gap, excess));
    }
    // Pinned from measurement, not chosen: the printed excesses are
    // +1.55 %, +1.35 %, +0.48 %, -0.00 %.
    for (gap, excess) in &excesses {
        assert!(
            *excess < 0.05,
            "gap {gap:e}: the side walls contribute {:.1}% of the contact force, \
             so the flat bottom face is no longer the load-bearing area and a \
             kappa derived from it is wrong by that factor",
            100.0 * excess,
        );
    }

    // The excess shrinks as the gap opens, because the band stops reaching the
    // side walls' Gauss points. At the widest gap probed it reaches zero, and
    // there the traction law is not approximate at all: it is the energy
    // expression evaluated, and must agree to floating point. This is the arm
    // that says the law itself is exact and only the *area* was ever the
    // approximation.
    let (widest_gap, widest_excess) = excesses
        .last()
        .copied()
        .expect("the sweep must produce at least one row");
    assert!(
        widest_excess.abs() < 1e-9,
        "at gap {widest_gap:e} m no side-wall Gauss point is inside the band, so \
         `F = kappa * A_flat * |b'(gap)|` should be an identity — it disagreed \
         by {widest_excess:.3e} relative, which means the face barrier is not \
         the energy this fixture thinks it is",
    );
    for w in excesses.windows(2) {
        assert!(
            w[1].1 <= w[0].1 + 1e-12,
            "the side-wall excess must shrink as the gap opens ({:e} m -> {:.3}%, \
             {:e} m -> {:.3}%); a rise means faces are ENTERING the band as it \
             recedes, which no geometry here can do",
            w[0].0,
            100.0 * w[0].1,
            w[1].0,
            100.0 * w[1].1,
        );
    }
}

/// The contact patch [`DESIGN_TRACTION_PA`] was measured over — its flat rest
/// area and its face count, both read from the mesh rather than assumed.
///
/// The flat bottom face is the load-bearing area, as
/// [`the_face_barrier_traction_law_holds_against_the_solver`] establishes.
fn design_contact_patch() -> RestContact {
    rest_contact_at(0.5 * D_HAT, KAPPA)
}

/// Rest area of the plate's flat bottom face (m^2).
fn flat_contact_area() -> f64 {
    design_contact_patch().flat_area
}

/// **Is the contact traction a property of the scene, or of kappa?**
///
/// The whole derivation hinges on this. `kappa = sigma / |b'(d)|` is only a
/// *derivation* if `sigma` — the traction the compressed plate pushes back
/// with — is set by the material and the imposed compression rather than by
/// the barrier stiffness being solved for. If `sigma` moved with `kappa`, the
/// formula would just be re-deriving its own input.
///
/// It does move a little, and it must: a stiffer barrier holds the plate
/// further off the plane, so at a **fixed plane height** it is compressed less
/// and pushes back less. What matters is the size of that coupling against the
/// spread of `kappa` driving it, and this probe measures it rather than
/// assuming it is small.
///
/// ⚠ **Compared at a COMMON plane height, not at each arm's own last rung.**
/// A soft `kappa` stalls early and a stiff one runs to the ceiling, so their
/// final rungs are at different compressions; reading `sigma` off each arm's
/// last rung would report a spread that is mostly *depth* and call it a
/// *kappa* effect. The deepest plane height every arm reached is the only
/// place the three are the same experiment.
///
/// `#[ignore]` — one compression ramp per kappa.
#[test]
#[ignore = "one compression ramp per kappa, minutes — the measurement the \
            derived kappa is built on"]
fn is_the_contact_traction_a_property_of_the_scene_or_of_kappa() {
    let area = flat_contact_area();
    eprintln!(
        "contact traction vs kappa   [d_hat {D_HAT:e} m, flat area {:.1} mm2, \
         ramp to {:+.2} mm]",
        area * 1e6,
        RAMP_MAX_PLANE_H * 1e3,
    );
    // Every arm's full (plane height -> converged pose) series, so the
    // comparison can be taken at a height they all reached.
    let mut arms: Vec<(f64, Vec<(f64, Press)>)> = Vec::new();
    for exp in 6..=8 {
        let kappa = 10f64.powi(exp);
        let setup = Setup {
            barrier: Barrier {
                kappa,
                d_hat: D_HAT,
            },
            indenter: Indenter::Plane,
        };
        let series: Vec<(f64, Press)> = ramp_tet10_yeoh(setup, RAMP_STEP, RAMP_MAX_PLANE_H)
            .into_iter()
            .filter_map(|(h, r)| r.ok().map(|p| (h, p)))
            .collect();
        eprintln!(
            "  kappa {kappa:8.0e}: {:2} rungs converged, deepest plane {:+.3} mm",
            series.len(),
            series.last().map_or(f64::NAN, |(h, _)| h * 1e3),
        );
        arms.push((kappa, series));
    }
    assert!(
        arms.iter().all(|(_, s)| !s.is_empty()),
        "every kappa arm must converge at least one rung",
    );

    // The deepest plane height common to all arms.
    let common_h = arms
        .iter()
        .map(|(_, s)| s.last().map_or(f64::NEG_INFINITY, |(h, _)| *h))
        .fold(f64::INFINITY, f64::min);
    eprintln!("  common plane height: {:+.4} mm", common_h * 1e3);
    eprintln!(
        "  {:>9} {:>10} {:>10} {:>10} {:>11} {:>12} {:>9}",
        "kappa", "disp/mm", "min_sd/mm", "mean_sd", "sigma/kPa", "d_eff/mm", "in bracket",
    );
    let mut rows = Vec::new();
    for (kappa, series) in &arms {
        let (h, p) = series
            .iter()
            .rfind(|(h, _)| *h <= common_h + 1e-12)
            .expect("every arm reached the common height by construction");
        assert!(
            (h - common_h).abs() < 0.5 * RAMP_STEP,
            "arm kappa {kappa:e} has no rung at the common height {common_h:e} \
             (nearest {h:e}) — the arms are not on the same ramp grid and the \
             comparison would be between different compressions",
        );
        let sigma = p.net_force_z / area;
        // The law, run forwards: given this traction, what standoff should a
        // barrier of this stiffness hold? Compared against what it did hold.
        let predicted_sd = face_barrier_standoff(*kappa, D_HAT, sigma);
        eprintln!(
            "  {:9.0e} {:10.4} {:10.4} {:10.4} {:11.3} {:12.4} {:>9}",
            kappa,
            p.max_disp * 1e3,
            p.min_sd * 1e3,
            p.mean_sd * 1e3,
            sigma * 1e-3,
            predicted_sd * 1e3,
            if predicted_sd >= p.min_sd && predicted_sd <= p.mean_sd {
                "yes"
            } else {
                "NO"
            },
        );
        rows.push((*kappa, p.min_sd, p.mean_sd, sigma, predicted_sd));
    }

    let sig_lo = rows.iter().map(|r| r.3).fold(f64::INFINITY, f64::min);
    let sig_hi = rows.iter().map(|r| r.3).fold(f64::NEG_INFINITY, f64::max);
    let k_lo = rows.iter().map(|r| r.0).fold(f64::INFINITY, f64::min);
    let k_hi = rows.iter().map(|r| r.0).fold(f64::NEG_INFINITY, f64::max);
    eprintln!(
        "  at one compression, kappa spans {:.0}x and sigma spans {:.3}x \
         ({:.3} -> {:.3} kPa)",
        k_hi / k_lo,
        sig_hi / sig_lo,
        sig_lo * 1e-3,
        sig_hi * 1e-3,
    );
    // A DIFFERENT quantity, reported separately on purpose: the traction at
    // the ramp ceiling on the baseline arm. The table above is a *controlled*
    // comparison and can only be taken where every arm converged, which is
    // wherever the softest one stalled — shallow. The design point is the
    // deepest compression the fixture is required to reach, and only the
    // baseline arm gets there. It is one arm, not three, and the table above
    // is what licenses reading it as a property of the scene.
    for (kappa, series) in &arms {
        let Some((h, p)) = series.last() else {
            continue;
        };
        eprintln!(
            "  DESIGN POINT  kappa {:8.0e}: plane {:+.3} mm, disp {:.4} mm, \
             sigma {:.3} kPa, min_sd {:.4} mm",
            kappa,
            h * 1e3,
            p.max_disp * 1e3,
            p.net_force_z / area * 1e-3,
            p.min_sd * 1e3,
        );
    }

    assert_the_derivation_holds_on(&arms, area, &rows);
}

/// One arm's reading at the common plane height:
/// `(kappa, min_sd, mean_sd, sigma, effective gap)`.
type LawRow = (f64, f64, f64, f64, f64);

/// The assertions [`is_the_contact_traction_a_property_of_the_scene_or_of_kappa`]
/// draws from its ramps, split out so each half stays readable: above, run the
/// arms and report; here, hold the derivation against them.
fn assert_the_derivation_holds_on(arms: &[(f64, Vec<(f64, Press)>)], area: f64, rows: &[LawRow]) {
    // ── the two things this probe pins for the always-on derivation ──
    //
    // 1. DESIGN_TRACTION_PA itself. `kappa_is_derived_and_not_swept` evaluates
    //    the floor and ceiling from that constant, and its interval is 9.47x
    //    wide (the ratio |b'(rho * step)| / |b'(d_hat/2)|, independent of
    //    traction), so it tolerates the traction being wrong by roughly
    //    0.47x-4.46x before a different decade is selected. That slack is real
    //    and this is where it is closed: the constant is checked against a
    //    fresh measurement.
    let baseline = arms
        .iter()
        .find(|(k, _)| (*k - KAPPA).abs() < 1.0)
        .and_then(|(_, s)| s.last())
        .expect("the baseline kappa must be one of the swept arms");
    let measured_design_traction = baseline.1.net_force_z / area;
    let drift = (measured_design_traction / DESIGN_TRACTION_PA - 1.0).abs();
    eprintln!(
        "  DESIGN_TRACTION_PA {:.1} Pa vs measured {:.1} Pa ({:+.2} %)",
        DESIGN_TRACTION_PA,
        measured_design_traction,
        100.0 * (measured_design_traction / DESIGN_TRACTION_PA - 1.0),
    );
    assert!(
        drift < 0.05,
        "DESIGN_TRACTION_PA is {DESIGN_TRACTION_PA:e} Pa but the fixture now          produces {measured_design_traction:e} Pa ({:.1}% off) at its design          point. Every bound in `kappa_is_derived_and_not_swept` is computed          from that constant, so it is stale and the derived kappa is a          derivation from the wrong number.",
        100.0 * drift,
    );

    // 2. The wall each kappa hits. Past `sigma = kappa * |b'(REQUIRED_STANDOFF)|`
    //    the barrier can no longer hold the clearance the next increment
    //    needs, so the march goes infeasible.
    //
    //    ⚠ THE SENSE MATTERS, and an earlier version of this arm had it
    //    backwards. By monotonicity of `|b'|`, `sigma > threshold` is the same
    //    statement as `d_eff < REQUIRED_STANDOFF` — so CROSSING the threshold
    //    is what a stall looks like, and staying under it is what running
    //    clean looks like. Asserting `reached < threshold` for every arm asked
    //    the stalled arm to behave like the clean ones. It passed only because
    //    the uncorrected threshold (rho = 1, 17.6 kPa) sat high enough to
    //    cover both; correcting the floor for patch non-uniformity dropped it
    //    to 13.6 kPa and the stalled arm's 14.3 kPa finally showed it.
    for (kappa, series) in arms {
        let threshold = kappa * barrier_derivative(REQUIRED_STANDOFF, D_HAT).abs();
        let reached = series.last().map_or(0.0, |(_, p)| p.net_force_z / area);
        let ran_clean = series
            .last()
            .is_some_and(|(h, _)| *h >= RAMP_MAX_PLANE_H - RAMP_STEP);
        eprintln!(
            "  kappa {kappa:8.0e}: threshold {:9.3} kPa, reached {:8.3} kPa, {}",
            threshold * 1e-3,
            reached * 1e-3,
            if ran_clean { "ran clean" } else { "STALLED" },
        );
        if ran_clean {
            assert!(
                reached < threshold,
                "kappa {kappa:e} ran the ramp clean while carrying {reached:e} Pa, \
                 ABOVE the {threshold:e} Pa at which the derivation says it runs \
                 out of clearance. The feasibility argument that sets the kappa \
                 floor predicts a stall here and there was none.",
            );
        } else {
            assert!(
                reached >= threshold,
                "kappa {kappa:e} STALLED while still carrying only {reached:e} Pa, \
                 BELOW the {threshold:e} Pa at which the derivation says it runs \
                 out of clearance. Something other than barrier feasibility \
                 stopped this ramp, so the kappa floor does not explain it.",
            );
        }
    }

    // 3. PATCH_NONUNIFORMITY. The derived floor divides by |b'| at
    //    `rho * RAMP_STEP` rather than at RAMP_STEP, because the stall
    //    condition is on the tightest gap and the traction is a mean. That
    //    correction is only sound if `rho` really does bound `d_eff / min_sd`
    //    on every pose, so every pose is checked against it.
    for (kappa, min_sd, _, _, d_eff) in rows {
        let rho = d_eff / min_sd;
        eprintln!("  kappa {kappa:8.0e}: d_eff / min_sd = {rho:.4}");
        assert!(
            rho <= PATCH_NONUNIFORMITY,
            "kappa {kappa:e}: the patch is {rho:.4}x less uniform than \
             PATCH_NONUNIFORMITY ({PATCH_NONUNIFORMITY}) allows for, so the \
             derived floor is optimistic on this pose and KAPPA may not in \
             fact hold one increment open at the tightest point",
        );
        assert!(
            rho >= 1.0,
            "kappa {kappa:e}: d_eff ({d_eff:e}) is BELOW min_sd ({min_sd:e}), \
             which convexity forbids — the bracket arm below should have \
             caught this first",
        );
    }

    // The law under load. The rest-pose gate
    // `the_face_barrier_traction_law_holds_against_the_solver` tests it on a
    // flat patch at one uniform gap; under load the patch bulges and the gaps
    // spread, so there is no single gap to check against.
    //
    // What survives the spread is a bracket. `kappa*|b'(d)|` is convex and
    // decreasing, so the effective gap that carries the *mean* traction —
    // `face_barrier_standoff(kappa, d_hat, F/A)` — must lie between the
    // minimum gap and the area-weighted mean gap. Below the minimum would mean
    // the patch carries more load than its tightest point could; above the
    // mean would mean convexity ran backwards.
    for (kappa, min_sd, mean_sd, sigma, predicted) in rows {
        assert!(
            predicted >= min_sd,
            "kappa {kappa:e}: the mean traction {sigma:e} Pa is carried at an \
             effective gap of {predicted:e} m, BELOW the tightest gap in the \
             patch ({min_sd:e} m) — no part of the patch is that loaded, so \
             either the flat area or the traction law is wrong",
        );
        assert!(
            predicted <= mean_sd,
            "kappa {kappa:e}: the mean traction {sigma:e} Pa is carried at an \
             effective gap of {predicted:e} m, ABOVE the area-weighted mean gap \
             ({mean_sd:e} m). |b'| is convex, so the effective gap can never \
             exceed the mean — this says the convexity argument does not hold \
             on this patch",
        );
    }
}

// ── the derived kappa ───────────────────────────────────────────────

/// **The derivation.** Both bounds on `κ`, evaluated on the shipped barrier.
///
/// Cheap by construction — no solve, only the barrier arithmetic — so it runs
/// always-on and re-derives `κ` on every build rather than leaving a decade
/// pinned by a sweep nobody re-runs. Every arm here can fail: lowering
/// [`KAPPA`] to 1e6 trips the floor, raising it to 1e8 trips the ceiling, and
/// a [`DESIGN_TRACTION_PA`] off by ~3× moves the interval off 1e7 entirely.
///
/// ⚠ **The bracket `(1e6, 1e7]` it is checked against is MEASURED**, by
/// [`the_armijo_wall_against_barrier_stiffness`] — 1e6 stalls, 1e7 runs clean
/// to the ramp ceiling. That independence is the point: the floor is computed
/// from a traction and a barrier derivative, the bracket comes from running
/// ramps, and they are only allowed to agree if the derivation is right.
#[test]
fn kappa_is_derived_and_not_swept() {
    let floor = face_barrier_kappa(D_HAT, REQUIRED_STANDOFF, DESIGN_TRACTION_PA)
        .expect("one ramp step must lie inside the barrier band");
    let ceiling = face_barrier_kappa(D_HAT, 0.5 * D_HAT, DESIGN_TRACTION_PA)
        .expect("half the band is inside the band");
    let standoff = face_barrier_standoff(KAPPA, D_HAT, DESIGN_TRACTION_PA);
    eprintln!(
        "derived kappa: floor {floor:.4e} <= {KAPPA:.4e} <= ceiling {ceiling:.4e}\n  \
         design traction {:.3} kPa, required standoff {:.4} mm, \
         standoff held {:.4} mm ({:.1} % of band)",
        DESIGN_TRACTION_PA * 1e-3,
        REQUIRED_STANDOFF * 1e3,
        standoff * 1e3,
        100.0 * standoff / D_HAT,
    );

    // DESIGN_TRACTION_PA was measured on a specific contact patch, and only an
    // `#[ignore]`d probe re-measures it. The always-on half of that guard is
    // here: if the geometry it was measured on has moved, the constant is
    // stale even though the arithmetic below still evaluates cleanly. The
    // interval is wide enough (9.47x) to absorb a real traction shift without
    // complaining, so this is the arm that would notice.
    let patch = design_contact_patch();
    let stale = |what: &str| -> String {
        format!(
            "the contact patch DESIGN_TRACTION_PA was measured on has changed \
             ({what}), so the traction constant is stale. Re-run \
             `is_the_contact_traction_a_property_of_the_scene_or_of_kappa` and \
             update the constants before trusting any bound below."
        )
    };
    assert!(
        (patch.flat_area / DESIGN_CONTACT_AREA - 1.0).abs() < 1e-9,
        "{}",
        stale(&format!(
            "flat area {:.1} mm2, was {:.1} mm2",
            patch.flat_area * 1e6,
            DESIGN_CONTACT_AREA * 1e6,
        )),
    );
    assert_eq!(
        patch.n_faces,
        DESIGN_CONTACT_FACES,
        "{}",
        stale(&format!(
            "{} active faces, was {DESIGN_CONTACT_FACES}",
            patch.n_faces,
        )),
    );

    assert!(
        floor < ceiling,
        "the two requirements are inconsistent: the barrier cannot both hold \
         {REQUIRED_STANDOFF:e} m open (needs kappa >= {floor:e}) and stay \
         inside half the band (needs kappa <= {ceiling:e}). That is not a \
         kappa problem — d_hat is too small for this traction.",
    );
    assert!(
        KAPPA >= floor,
        "KAPPA = {KAPPA:e} is below the derived floor {floor:e}: under \
         {DESIGN_TRACTION_PA:e} Pa it holds only {standoff:e} m, less than the \
         {REQUIRED_STANDOFF:e} m increment, so the march starts infeasible and \
         the ramp stalls at Newton iteration 0",
    );
    assert!(
        KAPPA <= ceiling,
        "KAPPA = {KAPPA:e} is above the derived ceiling {ceiling:e}: it holds \
         {standoff:e} m of standoff, more than half the {D_HAT:e} m band, so \
         it is cushioning the contact rather than enforcing it and every \
         contact position it reports carries that bias",
    );

    // The interval must be narrow enough to SELECT a decade. An interval
    // spanning two decades would make "1e7" a choice again, dressed up.
    let decades_inside = (1..=12)
        .map(|e| 10f64.powi(e))
        .filter(|k| *k >= floor && *k <= ceiling)
        .count();
    assert_eq!(
        decades_inside, 1,
        "the derived interval [{floor:e}, {ceiling:e}] contains {decades_inside} \
         decades, so it does not pin one. With more than one, KAPPA is still a \
         preference; with none, the two requirements cannot both be met on a \
         round number and the constant must be the bound itself.",
    );

    // And it must be the decade actually shipped.
    assert!(
        (KAPPA.log10() - KAPPA.log10().round()).abs() < 1e-12,
        "KAPPA {KAPPA:e} is not a decade, so the decade-selection argument \
         above does not describe how it was chosen",
    );

    // Cross-check against the MEASURED sweep: the floor has to explain why the
    // decade below stalls and the shipped one does not.
    let below = face_barrier_standoff(KAPPA / 10.0, D_HAT, DESIGN_TRACTION_PA);
    assert!(
        below < REQUIRED_STANDOFF,
        "the derivation says kappa = {:e} holds {below:e} m, which clears the \
         {REQUIRED_STANDOFF:e} m increment — but the sweep measured that decade \
         STALLING. The derivation would then not explain the wall it was \
         written to explain.",
        KAPPA / 10.0,
    );
    assert!(
        standoff > REQUIRED_STANDOFF,
        "the derivation says the shipped kappa holds {standoff:e} m, under the \
         {REQUIRED_STANDOFF:e} m increment — but the sweep measured it running \
         clean to the ramp ceiling",
    );
}

/// **How load-bearing is `d̂/2`?**
///
/// The ceiling's *form* — `σ / |b'(x)|` — is the same traction relation the
/// floor uses. Its *argument* is a judgement: keep the standoff in the lower
/// half of the tolerance band. "Half" is a round number, and a derivation that
/// silently depends on a round number is a preference wearing a formula.
///
/// So this measures the dependence instead of asserting there is none. It
/// sweeps the bound across `d̂/1.5 … d̂/4` and records where the decade
/// selection survives, which turns "we chose a half" into a stated width.
///
/// ⛔ It is **not** a gate on the shipped value — [`kappa_is_derived_and_not_swept`]
/// is. This one fails only if the *shape* of the dependence changes: if the
/// choice stopped mattering at all (nothing to state) or if `d̂/2` stopped
/// sitting inside the surviving band with room on both sides.
#[test]
fn the_ceiling_is_a_stated_requirement() {
    /// Bounds swept, in order; the assertions below index this.
    const DENOMS: [f64; 6] = [1.5, 2.0, 2.5, 2.8, 3.0, 4.0];

    let floor = face_barrier_kappa(D_HAT, REQUIRED_STANDOFF, DESIGN_TRACTION_PA)
        .expect("the required standoff must lie inside the barrier band");
    eprintln!("ceiling sensitivity  [floor {floor:.4e}, KAPPA {KAPPA:.4e}]");
    let mut admits = Vec::new();
    for denom in DENOMS {
        let bound = D_HAT / denom;
        let ceiling = face_barrier_kappa(D_HAT, bound, DESIGN_TRACTION_PA)
            .expect("a fraction of the band is inside the band");
        let ok = floor <= KAPPA && KAPPA <= ceiling;
        eprintln!(
            "  d_hat/{denom:<4} bound {:.4} mm  ceiling {ceiling:.4e}  KAPPA {}",
            bound * 1e3,
            if ok { "admitted" } else { "EXCLUDED" },
        );
        admits.push((denom, ok));
    }

    // The choice must matter — otherwise there is nothing to state and the
    // caveat in KAPPA's docs is noise.
    assert!(
        admits.iter().any(|(_, ok)| !ok),
        "no bound in the sweep excludes KAPPA, so the ceiling choice is not \
         load-bearing at all and the sensitivity note above should be deleted \
         rather than maintained",
    );
    // And it must not matter so much that the shipped choice is marginal: the
    // bound either side of d_hat/2 must agree with it.
    // Indexed, not looked up by float value: DENOMS is the sweep's own order.
    let admitted: Vec<bool> = admits.iter().map(|(_, ok)| *ok).collect();
    assert_eq!(
        &admitted[0..3],
        &[true, true, true],
        "d_hat/2 no longer sits inside a band of choices ({:?}) that all admit \
         KAPPA — the shipped ceiling is marginal, and a derivation resting on \
         a marginal round number is a preference",
        &DENOMS[0..3],
    );
    assert!(
        !admitted[4],
        "d_hat/{} now admits KAPPA too, so the sweep no longer brackets where \
         the choice starts to bite and the stated width is stale",
        DENOMS[4],
    );
}

// ── the closing cavity ──────────────────────────────────────────────

/// Cavity radius at rest (m) — the bore the intruder closes against.
const R_CAVITY: f64 = 0.010;

/// Outer radius (m). `R_OUTER − R_CAVITY` is 12 mm of wall — the same
/// thickness as the plate, so the two cells differ in *enveloping geometry*
/// and not in the wall scale the renovation targets.
const R_OUTER: f64 = 0.022;

/// The cavity cell's soft body: a thick spherical shell,
/// `SphereSdf{R_OUTER} \ SphereSdf{R_CAVITY}`.
fn shell() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf { radius: R_OUTER }),
        Box::new(SphereSdf { radius: R_CAVITY }),
    )
}

/// Tet4 shell carrying per-tet Yeoh materials, through the same
/// `from_sdf_yeoh` path the plate uses.
fn tet4_shell() -> SdfMeshedTetMesh<Yeoh> {
    tet4_shell_at(CELL)
}

/// The shell meshed at an arbitrary cell size — the geometry held fixed while
/// the discretisation moves, which is what
/// [`the_enveloping_patch_nonuniformity_is_a_property_of_the_MESH`] needs.
fn tet4_shell_at(cell: f64) -> SdfMeshedTetMesh<Yeoh> {
    tet4_shell_with(cell, yeoh_field())
}

/// The shell meshed at `cell` carrying `field` — the seam the graded cell
/// enters through. `tet4_shell_at` is this with the uniform field.
fn tet4_shell_with(cell: f64, field: MaterialField) -> SdfMeshedTetMesh<Yeoh> {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-R_OUTER - cell, -R_OUTER - cell, -R_OUTER - cell),
            Vec3::new(R_OUTER + cell, R_OUTER + cell, R_OUTER + cell),
        ),
        cell_size: cell,
        material_field: Some(field),
    };
    SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&shell(), &hints).expect("mesh the Yeoh shell")
}

/// The layer stack, innermost first — `insertion_sim`'s row-23 anchors.
const STACK: [SiliconeMaterial; 3] = [ECOFLEX_00_20, DRAGON_SKIN_10A, DRAGON_SKIN_20A];

/// Internal layer boundaries as offsets outward from the cavity wall (m),
/// so three 4 mm layers across the 12 mm wall: r = 14 mm and r = 18 mm.
const LAYER_BOUNDARIES: [f64; 2] = [0.004, 0.008];

/// [`STACK`] over the wall, keyed on the cavity SDF exactly as
/// `insertion_sim::layered_param_field` keys on the scan SDF.
fn graded_yeoh_field() -> MaterialField {
    graded_field_capped([
        STACK[0].validity_max_principal_stretch,
        STACK[1].validity_max_principal_stretch,
        STACK[2].validity_max_principal_stretch,
    ])
}

/// Outer-skin vertex ids — the pinned set.
///
/// Follows `insertion_sim`'s `outer_skin_bc`: every vertex within a band of the
/// outer envelope, with the intruder rather than a loaded BC driving the
/// deformation.
///
/// ⚠ **One difference, measured rather than waved at.** `outer_skin_bc` also
/// filters its band to solver-referenced vertices, because
/// `SdfMeshedTetMesh::positions()` is the BCC lattice and not the body. This
/// does not, and on this geometry the two agree exactly — orphans sit off both
/// spheres, so a band around `R_OUTER` never selects one.
/// [`the_meshed_shell_reproduces_the_analytic_hollow_sphere`] asserts the
/// orphan-pin count is zero, so the equivalence is checked and not assumed.
fn outer_skin_pins<M: sim_soft::Material>(mesh: &dyn Mesh<M>, band: f64) -> Vec<VertexId> {
    mesh.positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| (p.norm() - R_OUTER).abs() < band)
        .map(|(v, _)| v as VertexId)
        .collect()
}

/// Dirichlet band for [`outer_skin_pins`] (m).
///
/// `insertion_sim`'s `outer_skin_bc` uses half a cell; the plate's
/// [`top_face_pins`] uses a quarter, and a quarter is what this takes. It
/// selects 4 394 of the shell's 13 598 solver-referenced Tet10 nodes — not a
/// set that needs widening to give the wall something to react against.
const OUTER_SKIN_BAND: f64 = 0.25 * CELL;

/// The bore's advance at rest contact — the cavity wall sits inside the
/// barrier band with the bore not yet interfering. Mirrors [`REST_PLANE_H`].
///
/// ⚠ **"Rest" is not unloaded here, and it is less unloaded than on the
/// plate.** The plate meets the band on one face and reacts against a pinned
/// top. The shell meets it over a CLOSED surface with nothing to react against
/// but its own hoop stress, so the barrier's standoff traction pressurises the
/// cavity from the first rung — measured **7.3 kPa against the plate's
/// 2.9 kPa** at the same [`STANDOFF`]. The interference ramp therefore starts
/// from a pre-loaded state by construction, which is a property of enveloping
/// contact and not of this fixture's choices.
const REST_BORE_W: f64 = -STANDOFF;

/// The cavity cell at the baseline barrier.
const CAVITY: Setup = Setup {
    barrier: BASELINE_BARRIER,
    indenter: Indenter::Bore,
};

/// March the bore outward inside the cavity, re-solving from each converged
/// state. The enveloping sibling of [`ramp_tet10_yeoh`].
fn ramp_cavity(setup: Setup, step: f64, max_w: f64) -> Vec<Rung> {
    shell_cell().ramp(setup, step, max_w)
}

/// How far the always-on cavity gate drives the bore (m).
///
/// 0.5 mm into a 10 mm bore — 5 % radial interference, where the cavity
/// carries ~20 kPa, the same order as the plate's design traction. Shallower
/// than the wall on purpose: certifying that the enveloping cell reaches real
/// load is this gate's job, and finding where it stops is
/// [`how_deep_does_the_closing_cavity_converge`]'s.
const CAVITY_GATE_MAX_W: f64 = 0.0005;

/// The meshed shell is the hollow sphere the fixture asked for.
///
/// The enveloping sibling of
/// [`the_meshed_body_reproduces_the_box_extent_and_volume`], and it matters
/// more here: the plate's geometry comes from hand-written [`BoxSdf`] whose
/// gradient this file also checks, while the shell comes from a
/// [`DifferenceSdf`] of two library spheres through a `max(φa, −φb)` crease.
/// A difference that silently produced a solid ball would still mesh, still
/// solve, and still report contact.
#[test]
fn the_meshed_shell_reproduces_the_analytic_hollow_sphere() {
    let t4 = tet4_shell();
    let pos = Mesh::<Yeoh>::positions(&t4);

    let (mut r_lo, mut r_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for f in Mesh::<Yeoh>::boundary_faces(&t4) {
        for &v in f {
            let r = pos[v as usize].norm();
            r_lo = r_lo.min(r);
            r_hi = r_hi.max(r);
        }
    }
    let mesh_volume: f64 = (0..Mesh::<Yeoh>::n_tets(&t4) as TetId)
        .map(|t| {
            let v = Mesh::<Yeoh>::tet_vertices(&t4, t);
            let (v0, v1, v2, v3) = (
                pos[v[0] as usize],
                pos[v[1] as usize],
                pos[v[2] as usize],
                pos[v[3] as usize],
            );
            (v1 - v0).cross(&(v2 - v0)).dot(&(v3 - v0)) / 6.0
        })
        .sum();
    let analytic_volume = 4.0 / 3.0
        * std::f64::consts::PI
        * R_OUTER.mul_add(R_OUTER * R_OUTER, -(R_CAVITY * R_CAVITY * R_CAVITY));
    let (_, cavity_area) = boundary_faces_on_isosurface(
        pos,
        Mesh::<Yeoh>::boundary_faces(&t4),
        &SphereSdf { radius: R_CAVITY },
        0.0,
    );
    let analytic_area = 4.0 * std::f64::consts::PI * R_CAVITY * R_CAVITY;
    eprintln!(
        "  shell: boundary radius [{:.2}, {:.2}] mm (asked [{:.2}, {:.2}]), \
         volume {:.3} vs {:.3} cm3 ({:.1} %), cavity area {:.1} vs {:.1} mm2 ({:.1} %)",
        r_lo * 1e3,
        r_hi * 1e3,
        R_CAVITY * 1e3,
        R_OUTER * 1e3,
        mesh_volume * 1e6,
        analytic_volume * 1e6,
        100.0 * mesh_volume / analytic_volume,
        cavity_area * 1e6,
        analytic_area * 1e6,
        100.0 * cavity_area / analytic_area,
    );

    // ── pointwise, because every check above is an INTEGRAL ──────────
    // Volume, area and radial extent are all integrals: an off-centre bore, or
    // a lobed one, passes every one of them. This is the exact companion rule
    // 3c asks for — each boundary vertex must lie ON one of the two spheres.
    let mut worst = 0.0_f64;
    let mut worst_r = 0.0_f64;
    for f in Mesh::<Yeoh>::boundary_faces(&t4) {
        for &v in f {
            let r = pos[v as usize].norm();
            let dev = (r - R_CAVITY).abs().min((r - R_OUTER).abs());
            if dev > worst {
                worst = dev;
                worst_r = r;
            }
        }
    }
    eprintln!(
        "  worst boundary vertex deviation from either sphere: {:.4} mm at r={:.3} mm \
         ({:.1} % of CELL)",
        worst * 1e3,
        worst_r * 1e3,
        100.0 * worst / CELL,
    );
    assert!(
        worst < 0.1 * CELL,
        "a boundary vertex sits {worst:e} m from BOTH spheres (at r={worst_r:e} m). \
         The integral checks below cannot see this: an off-centre or lobed bore \
         has the right volume and the right area",
    );

    // The boundary must span BOTH surfaces and neither may be missing: a
    // collapsed difference (a solid ball) has `r_lo ≈ 0`, and a difference
    // taken the wrong way round has `r_hi ≈ R_CAVITY`.
    let tol = 0.1 * CELL;
    assert!(
        (r_lo - R_CAVITY).abs() < tol,
        "the meshed boundary's innermost radius is {r_lo:e} m, not the requested \
         cavity radius {R_CAVITY:e} — the difference did not open a bore",
    );
    assert!(
        (r_hi - R_OUTER).abs() < tol,
        "the meshed boundary's outermost radius is {r_hi:e} m, not the requested \
         outer radius {R_OUTER:e}",
    );
    // A faceted polyhedron through vertices ON both spheres cuts the corners of
    // the outer surface and cuts INTO the cavity, so the two errors have
    // opposite signs and the shell volume is not simply under-reported. A
    // decile is loose enough to be a coarse-mesh statement and tight enough
    // that a missing bore (+33 %) or a doubled one fails.
    assert!(
        (mesh_volume / analytic_volume - 1.0).abs() < 0.02,
        "meshed shell volume {mesh_volume:e} m3 is not within 2 % of the \
         analytic {analytic_volume:e} m3 (measured 99.6 %)",
    );
    assert!(
        (cavity_area / analytic_area - 1.0).abs() < 0.06,
        "the meshed cavity's area {cavity_area:e} m2 is not within 6 % of the \
         analytic {analytic_area:e} m2 (measured 96.4 %) — the patch the barrier \
         acts over is not the sphere the derivation assumes",
    );
}

/// **The cavity's contact goes through the FACE path, and its pin set holds
/// only vertices the solver can see.**
///
/// Split out of [`the_meshed_shell_reproduces_the_analytic_hollow_sphere`]:
/// that gate is about the BODY, these two are about how the solver meets it,
/// and they fail for entirely different reasons.
#[test]
fn the_cavity_takes_the_face_path_and_pins_only_live_vertices() {
    let t4 = tet4_shell();
    let t10 = Tet10Mesh::<Yeoh>::from_tet4(&t4);
    // ── the FACE path, asserted on the selector and not assumed ──────
    // `IpcRigidContact::active_pairs` is
    // `mesh.boundary_faces6().map_or_else(vertex_path, face_path)`
    // (`sim/L0/soft/src/contact/ipc.rs`), so `is_some()` IS the selector. Every
    // traction claim on this cell is about the SURFACE-INTEGRATED barrier, and
    // a shell that silently took the per-vertex path would still mesh, still
    // solve and still report contact — the same way round the plate's own
    // selector gate guards it.
    let faces6 = Mesh::<Yeoh>::boundary_faces6(&t10);
    assert!(
        faces6.is_some(),
        "the shell's Tet10 mesh surfaces no six-node boundary faces, so the \
         cavity is running on the PER-VERTEX barrier and every traction claim \
         about this cell is about a different energy",
    );
    assert!(
        Mesh::<Yeoh>::boundary_faces6(&t4).is_none(),
        "the Tet4 shell now surfaces six-node faces too, so the assertion above \
         no longer discriminates between the two paths",
    );

    // ── the counts the doc comments quote, re-measured here ──────────
    // They were measured once by a sizing probe that is not in the shipped
    // file; without this they are numbers with no referent.
    let referenced = referenced_vertices(&t10 as &dyn Mesh<Yeoh>).len();
    let pins = outer_skin_pins(&t10, OUTER_SKIN_BAND);
    eprintln!(
        "  shell: {} tets, {} Tet10 positions, {referenced} referenced, {} pins, \
         {} six-node boundary faces",
        Mesh::<Yeoh>::n_tets(&t4),
        Mesh::<Yeoh>::n_vertices(&t10),
        pins.len(),
        faces6.map_or(0, <[[VertexId; 6]]>::len),
    );
    assert_eq!(
        (Mesh::<Yeoh>::n_tets(&t4), referenced, pins.len()),
        (8736, 13598, 4394),
        "the shell's discretisation moved; every count quoted in this file's doc \
         comments was measured on the old one",
    );
    // F8: `outer_skin_bc` filters its band to solver-referenced vertices and
    // `outer_skin_pins` does not. Measured equivalent here — orphans sit off
    // both spheres, so the band never selects one — and this is what says so
    // rather than the docstring's word "mirrors".
    let ref_set: std::collections::BTreeSet<VertexId> =
        referenced_vertices(&t10 as &dyn Mesh<Yeoh>)
            .into_iter()
            .collect();
    let orphan_pins = pins.iter().filter(|v| !ref_set.contains(v)).count();
    assert_eq!(
        orphan_pins, 0,
        "{orphan_pins} pinned vertices are in no tet, so this fixture's pin set \
         is no longer equivalent to the referenced-filtered one it claims to \
         mirror",
    );
}

/// **What plays the role of `A_flat` on a closing cavity — and the answer is
/// that nothing does, because the force it divides is gone.**
///
/// [`DESIGN_TRACTION_PA`] is `F_z / A_flat`: a net force projected onto one
/// axis, over the flat rest patch that carries it. Both halves are properties
/// of a plate pressed onto a plane. On an enveloping patch the projection is
/// the half that fails first, and it fails completely rather than
/// approximately — every contact normal is radial, so the vector sum cancels
/// however hard the contact is working.
///
/// This measures both cells the same way and reports the contrast rather than
/// asserting a threshold on one of them. The plate's own number is the control:
/// if `|ΣF| / Σ|f|` were small there too, it would be measuring the readout,
/// not the geometry.
#[test]
fn the_enveloping_patch_cancels_the_net_force_the_flat_patch_reports() {
    let flat = Cell::plate()
        .press(BASELINE, REST_PLANE_H)
        .expect("the plate must reach rest contact");
    let cavity = Cell::shell()
        .press(CAVITY, REST_BORE_W)
        .expect("the shell must reach rest contact");

    // ⚠ **The FULL vector, not its z component.** A first cut of this gate used
    // `net_force_z` and still passed — but it reported 6.74e-4 where the true
    // residual is 1.11e-3, and its message claimed it would catch the shell
    // "being pushed sideways", which is precisely the ΣF_x/ΣF_y it never read.
    // The plate is unaffected (its normals are all `+z`); the cavity's residual
    // has no preferred axis.
    //
    // The reciprocal is the reader-facing number — `F_z` divided by ANY area is
    // that many times too small — so it is reported rather than asserted twice.
    let coherence = |p: &Press| p.net_force.norm() / p.sum_force_mag;
    eprintln!(
        "  flat  : |F| {:.4} N (z {:.4}), sum|f| {:.4} N, coherence {:.4}, \
         mean traction {:.3} kPa\n  \
         cavity: |F| {:.3e} N (z {:.3e}), sum|f| {:.4} N, coherence {:.2e} (1/{:.0}), \
         mean traction {:.3} kPa",
        flat.net_force.norm(),
        flat.net_force_z,
        flat.sum_force_mag,
        coherence(&flat),
        flat.mean_traction / 1e3,
        cavity.net_force.norm(),
        cavity.net_force_z,
        cavity.sum_force_mag,
        coherence(&cavity),
        1.0 / coherence(&cavity),
        cavity.mean_traction / 1e3,
    );

    assert!(
        coherence(&flat) > 0.9,
        "the plate's contact forces no longer point the same way (coherence {:.4}) \
         — this control is what licenses reading `F_z / A_flat` as a traction \
         there, so the cavity contrast below means nothing without it",
        coherence(&flat),
    );
    assert!(
        coherence(&cavity) < 1.0e-2,
        "the cavity's net force is {:.1} % of its force magnitude, not the ~0 a \
         closed radial patch must give — either the bore is not enveloping the \
         contact or the shell is being pushed sideways",
        100.0 * coherence(&cavity),
    );
    // The force-free reading is not merely defined, it is in the same
    // range as the plate's design traction — so it is a traction, not a
    // residue of the cancellation.
    assert!(
        cavity.mean_traction > 1.0e3 && cavity.mean_traction < 1.0e6,
        "the cavity's force-free traction reads {:e} Pa, outside the range any \
         Ecoflex contact in this fixture occupies",
        cavity.mean_traction,
    );
}

/// The force-free traction reading is the same measurement as the plate's, up
/// to a fixed calibration — which is what licenses using it where force is
/// gone.
///
/// [`Press::mean_traction`] never mentions a force or a flat patch, so it
/// survives the cavity. That is necessary and not sufficient: a quantity that
/// survives is only useful if it agrees with the one it replaces *where both
/// are defined*. The plate is the only place both are.
///
/// **Measured**: the ratio is 1.0001 while the active set is still growing,
/// then settles to **0.9360–0.9364 and stays there for thirteen rungs** as the
/// load rises fourfold. The agreement claim is therefore about the ratio being
/// CONSTANT, not about it being one.
///
/// ⚠ **What the 6.4 % is has not been isolated, and two candidate explanations
/// are measured false.** It is not the contact forces splaying off-axis: every
/// active pair's force direction is within 26° of `+z` at every rung, and
/// restricting the mean to those pairs changes it in no printed digit. Nor is
/// it side-wall faces diluting an area-weighted mean: the face-level active set
/// is a constant 520 faces at `flat / active = 0.7692` across every gap this
/// fixture visits, and 0.769 is not 0.936. The two definitions differ in
/// weighting (deformed nodal tributary vs flat rest area) and in sample points
/// (nodal `sd` vs the face quadrature the barrier integrates); which dominates
/// is unmeasured.
///
/// ★ **What the 6.4 % does establish is that `A_flat` is the right
/// normaliser.** Replacing it with the pairs' own summed tributary area — the
/// obvious geometry-free substitute — makes the agreement WORSE and, more
/// importantly, unstable: that ratio drifts monotonically across the same
/// rungs instead of holding, because tributary area is measured on the
/// DEFORMED patch and grows as the plate bulges, while the face barrier
/// integrates over the REST area (`E = A_rest · Σ_q ŵ_q b(sd)`,
/// `sim/L0/soft/src/contact/face.rs`). A drifting normaliser cannot calibrate
/// anything, so the enveloping cell inherits the traction mean and *not* a
/// deformed-area denominator.
///
/// ⚠ **Provenance**: that drift was 1.032 → 1.154 over a deformed area of
/// 1652 → 1976 mm², measured once on 2026-09-21 while choosing the normaliser.
/// The probe is **not retained** — `Press` no longer carries a tributary-area
/// sum, because nothing that shipped needs one — so those four numbers are a
/// recorded measurement and not a re-checkable one. The conclusion they
/// support is re-checked every run by the spread assertion below.
///
/// ★ It does not need to be isolated for the derivation to stand. `κ`'s
/// bracket is `|b′(ρ·step)| / |b′(d̂/2)|` = 9.47× wide, so it selects a decade
/// while `σ` is known to roughly `[0.47×, 4.46×]`. A 6.4 % calibration is
/// fifteen times inside that. A *drifting* ratio would not be, which is why
/// this gate asserts the spread and not just the value.
#[test]
fn the_force_free_traction_tracks_force_over_area_on_the_plate() {
    let area = flat_contact_area();
    let rungs: Vec<Press> = plate_rungs()
        .iter()
        .cloned()
        .map(|(h, r)| {
            r.map_err(|e| format!("plate rung {h:e} m failed: {e}"))
                .expect("every plate rung must converge for the two definitions to be compared")
        })
        .collect();

    // Compare only where the patch has stopped recruiting pairs. While it is
    // still growing the two definitions are sampling different surfaces, and
    // averaging that transient into the calibration would report a spread that
    // is the recruitment, not the disagreement.
    let engaged = rungs
        .iter()
        .map(|p| p.n_pairs)
        .max()
        .expect("the ramp must produce at least one rung");
    let ratios: Vec<f64> = rungs
        .iter()
        .filter(|p| p.n_pairs == engaged)
        .map(|p| p.mean_traction / (p.net_force_z / area))
        .collect();
    let (lo, hi) = ratios
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &r| {
            (l.min(r), h.max(r))
        });
    eprintln!(
        "  {} of {} rungs fully engaged at {engaged} pairs; ratio in [{lo:.4}, {hi:.4}], \
         spread {:.2e}",
        ratios.len(),
        rungs.len(),
        hi - lo,
    );

    assert!(
        ratios.len() >= 8,
        "only {} rungs reached the fully-engaged patch, too few to call the \
         calibration constant",
        ratios.len(),
    );
    assert!(
        (0.92..0.95).contains(&lo) && (0.92..0.95).contains(&hi),
        "the force-free traction reads [{lo:.4}, {hi:.4}] of `F_z / A_flat`, \
         outside the measured 0.9360-0.9364 band — the two are no longer the \
         same measurement up to a calibration",
    );
    assert!(
        hi - lo < 2.0e-3,
        "the calibration DRIFTS by {:.2e} over the ramp, so it is not a \
         calibration. A force-free traction that tracks force-over-area only at \
         one load cannot stand in for it at another, which is the whole use",
        hi - lo,
    );
}

/// **Does `Tet10Mesh<Yeoh>` converge when the material closes AROUND the
/// rigid body instead of being indented by it?**
///
/// The cell this file's own "what this cannot see" list named first. Curvature
/// was covered by [`tet10_yeoh_converges_against_a_curved_indenter`], but a
/// sphere pressed into a plate is still a Hertzian patch behind an advancing
/// front: a growing disc of contact, surrounded by material that is not yet
/// touching. `insertion_sim` is the opposite — a compliant bore closing on a
/// probe, where *"the whole cavity wall engages at once"*
/// (`tools/cf-sim-research/src/insertion_sim.rs`, on why penalty's `κ` had to
/// be softened to `1e3` there). That sentence is the regime, and the active-set
/// assertion below is what turns it from a quotation into a measurement.
///
/// Same element, material, barrier and solver config as the plate. What
/// changes is the body (a thick spherical shell) and the boundary set (the
/// outer skin, as `insertion_sim`'s `outer_skin_bc` pins it).
#[test]
fn tet10_yeoh_converges_against_a_closing_cavity() {
    let rungs = cavity_rungs();
    eprintln!(
        "cavity ramp  [R_cav {:.0} mm, wall {:.0} mm, kappa {KAPPA:e}, d_hat {D_HAT:e} m]",
        R_CAVITY * 1e3,
        (R_OUTER - R_CAVITY) * 1e3,
    );
    eprintln!(
        "  {:>8} {:>4} {:>10} {:>10} {:>10} {:>7} {:>6}",
        "w(mm)", "it", "resid", "trac(kPa)", "min_sd", "rho", "pairs",
    );
    let mut ok = Vec::new();
    for (w, r) in rungs {
        match r {
            Ok(p) => {
                let rho = face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd;
                eprintln!(
                    "  {:>8.3} {:>4} {:>10.2e} {:>10.3} {:>10.3e} {:>7.4} {:>6}",
                    w * 1e3,
                    p.iters,
                    p.residual,
                    p.mean_traction / 1e3,
                    p.min_sd,
                    rho,
                    p.n_pairs,
                );
                ok.push((*w, p, rho));
            }
            Err(e) => eprintln!("  {:>8.3} FAILED {e}", w * 1e3),
        }
    }

    assert_eq!(
        ok.len(),
        rungs.len(),
        "the cavity ramp stalled before its ceiling; every rung above is the \
         measurement, and the first Err line names the surface that fired",
    );
    let last = ok
        .last()
        .expect("the ramp must produce at least one rung")
        .1;

    // ── the enveloping signature ─────────────────────────────────────
    // A Hertzian patch RECRUITS: the curved plate cell goes 185 -> 473 pairs as
    // the apex advances. A closing cavity cannot, because there is no
    // not-yet-touching material to recruit — it is all touching at rest. This
    // is the one assertion here that a convex indenter could not pass.
    let pairs: Vec<usize> = ok.iter().map(|(_, p, _)| p.n_pairs).collect();
    assert!(
        pairs.windows(2).all(|w| w[0] == w[1]),
        "the cavity's active set moved across the ramp ({pairs:?}) — an \
         enveloping patch engages at rest and has nothing left to recruit, so \
         either the bore is not enveloping or pairs are dropping out",
    );

    // ── it reaches real load, and does it cheaply ────────────────────
    assert!(
        last.mean_traction > 15.0e3,
        "the deepest rung carries {:.1} kPa, which is not the order the plate's \
         design point sits at ({:.1} kPa) — the ceiling is too shallow to be \
         evidence about a loaded cavity",
        last.mean_traction / 1e3,
        DESIGN_TRACTION_PA / 1e3,
    );
    for (w, p, _) in &ok {
        assert!(
            p.iters <= 8,
            "rung {:+.3} mm took {} Newton iterations; the plate takes 4-6 and a \
             jump here is the enveloping tangent, which is the thing this cell \
             exists to watch",
            w * 1e3,
            p.iters,
        );
        assert!(
            p.residual < 1.0e-8,
            "rung {:+.3} mm converged to {:e}, not to the ~1e-12 this fixture \
             reaches elsewhere",
            w * 1e3,
            p.residual,
        );
        // Marching feasibility, the same condition the plate's kappa floor is
        // derived from: the barrier has to hold more clearance than one
        // increment consumes, or the next rung starts infeasible.
        assert!(
            p.min_sd > RAMP_STEP,
            "rung {:+.3} mm came to rest holding {:e} m, less than the {RAMP_STEP:e} m \
             the next increment takes — the ramp is running on borrowed \
             feasibility and the rung after it is where that is spent",
            w * 1e3,
            p.min_sd,
        );
    }
}

/// **Does the flat plate's patch non-uniformity bound the enveloping one?**
///
/// [`PATCH_NONUNIFORMITY`] is the correction that makes `κ`'s floor a bound on
/// the patch's TIGHTEST gap rather than its mean, and its value — 1.30 — was
/// measured on a 40 × 40 mm flat patch against a plane. Its own docs say a
/// cavity closing around a probe *"has no reason to share it"*. This is that
/// measurement, and a failure here is a number to re-derive from, not a
/// regression to paper over.
///
/// ⚠ **The two `ρ`s are not computed from the same `σ`, and the difference is
/// in the safe direction.** The plate's probe inverts
/// `σ = F_z / A_flat`; this one inverts the force-free
/// [`Press::mean_traction`], because the cavity has no `F_z` to use. On the
/// plate the force-free reading is 6.4 % LOWER, and
/// [`face_barrier_standoff`] is decreasing in `σ` — a weaker traction inverts
/// to a WIDER gap — so `d_eff` and therefore `ρ` come out slightly high here.
/// A high `ρ` raises the floor, so this comparison errs toward rejecting the
/// shipped `κ`, not toward admitting it.
///
/// ⚠ **This gate sees only as deep as [`CAVITY_GATE_MAX_W`].** The full-depth
/// statement — `ρ` stays under the constant all the way to the convergence wall
/// — comes from [`how_deep_does_the_closing_cavity_converge`], which is
/// `#[ignore]`d and runs in no CI job.
#[test]
fn the_flat_patchs_nonuniformity_constant_bounds_the_enveloping_patch() {
    let rho: Vec<f64> = cavity_rungs()
        .iter()
        .cloned()
        .map(|(w, r)| {
            let p = r
                .map_err(|e| format!("cavity rung {w:e} m failed: {e}"))
                .expect("every cavity rung must converge for rho to be measured on it");
            face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd
        })
        .collect();
    let (lo, hi) = rho
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &r| {
            (l.min(r), h.max(r))
        });
    eprintln!(
        "  enveloping rho in [{lo:.4}, {hi:.4}] over {} rungs; flat-plate \
         constant {PATCH_NONUNIFORMITY}",
        rho.len(),
    );

    assert!(
        lo > 1.0,
        "the enveloping patch reports rho {lo:.4} < 1, which says the gap \
         carrying the mean traction is TIGHTER than the tightest gap — \
         impossible for a convex `|b'|`, so the measurement is wrong rather \
         than the geometry being kind",
    );
    assert!(
        hi < PATCH_NONUNIFORMITY,
        "the enveloping patch's non-uniformity reaches {hi:.4}, above the \
         {PATCH_NONUNIFORMITY} this fixture's kappa floor is derived with. That \
         floor is now optimistic on this geometry: RE-MEASURE and re-derive, do \
         not raise the constant to make this green",
    );
}

/// **How deep does the closing cavity go before it stops converging, and does
/// `ρ` stay bounded all the way there?**
///
/// [`tet10_yeoh_converges_against_a_closing_cavity`] certifies that the cell
/// reaches real load; this one walks it to the wall. The two are separated for
/// the same reason the plate separates its envelope gate from
/// [`the_armijo_wall_against_barrier_stiffness`]: finding the wall costs a full
/// ramp, and re-finding it on every commit buys nothing.
///
/// **Measured 2026-09-21**: clean to **4.72 mm of interference on a 10 mm bore
/// — 47 % radial** — at 4 Newton iterations rising to 8, then 15, then
/// `ArmijoStall(iter 5, r 1.65e5)` at 4.82 mm. `ρ` falls 1.1759 → 1.1639 and
/// then climbs back to **1.2209** at the last converged rung, staying under
/// [`PATCH_NONUNIFORMITY`] throughout.
///
/// ⭐ **The stall is NOT the marching-feasibility one the plate's `κ` floor is
/// derived from, and the derivation sees it coming anyway.** That floor
/// requires `min_sd > ρ · RAMP_STEP` = 0.130 mm; the cavity crosses it at
/// 3.82 mm and keeps converging for another ten increments, failing at 4.82 mm
/// at Newton iteration **5** rather than iteration 0. So on this geometry the
/// feasibility criterion is *conservative* — it predicts trouble a millimetre
/// early — and what actually ends the ramp is something else, unidentified
/// here. Two facts distinguish the modes and neither explains the cause: the
/// plate's `κ`-sweep stalls have `last_iter == 0` and a residual at the
/// scene's own scale, this one has neither.
#[test]
#[ignore = "one full ramp to the convergence wall, ~5 min — the depth and \
            non-uniformity numbers the always-on cavity gates quote"]
fn how_deep_does_the_closing_cavity_converge() {
    let rungs = ramp_cavity(CAVITY, RAMP_STEP, 0.006);
    eprintln!(
        "  {:>8} {:>4} {:>10} {:>10} {:>7} {:>10} {:>10}",
        "w(mm)", "it", "trac(kPa)", "min_sd", "rho", "feasible", "coherence",
    );
    let mut deepest = f64::NEG_INFINITY;
    let mut rho_hi = f64::NEG_INFINITY;
    let mut rho_lo = f64::INFINITY;
    let mut first_infeasible = None;
    let mut failure = None;
    for (w, r) in &rungs {
        match r {
            Ok(p) => {
                let rho = face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd;
                let feasible = p.min_sd > REQUIRED_STANDOFF;
                if !feasible && first_infeasible.is_none() {
                    first_infeasible = Some(*w);
                }
                eprintln!(
                    "  {:>8.3} {:>4} {:>10.3} {:>10.3e} {:>7.4} {:>10} {:>10.2e}",
                    w * 1e3,
                    p.iters,
                    p.mean_traction / 1e3,
                    p.min_sd,
                    rho,
                    feasible,
                    p.net_force.norm() / p.sum_force_mag,
                );
                deepest = *w;
                rho_hi = rho_hi.max(rho);
                rho_lo = rho_lo.min(rho);
            }
            Err(e) => {
                eprintln!("  {:>8.3} FAILED {e}", w * 1e3);
                failure = Some(e.clone());
            }
        }
    }
    eprintln!(
        "  deepest converged {:.3} mm = {:.1} % radial interference; \
         rho in [{rho_lo:.4}, {rho_hi:.4}]; \
         first rung below the required standoff {:?} mm",
        deepest * 1e3,
        100.0 * deepest / R_CAVITY,
        first_infeasible.map(|w| w * 1e3),
    );

    assert!(
        deepest > 0.004,
        "the cavity walled at {:.3} mm of interference, short of the 4.7 mm \
         measured — a regression in depth, or the wall moved",
        deepest * 1e3,
    );
    // ⚠ Both ends. An upper bound ALONE accepts an inverted computation:
    // `min_sd / d_eff` reads ~0.82 here, which is under 1.30 and would pass
    // silently. The always-on gate carries this guard; a first cut of this
    // probe did not, and the asymmetry was invisible because the number it
    // printed looked right.
    assert!(
        rho_lo > 1.0,
        "the enveloping patch reports rho {rho_lo:.4} < 1 at some rung — the gap \
         carrying the mean traction cannot be tighter than the tightest gap, so \
         the measurement is inverted or saturated, not the geometry kind",
    );
    assert!(
        rho_hi < PATCH_NONUNIFORMITY,
        "at full depth the enveloping patch's non-uniformity reaches {rho_hi:.4}, \
         above the {PATCH_NONUNIFORMITY} the kappa floor is derived with — the \
         always-on gate's shallower ceiling is hiding it",
    );
    // The mode matters more than the depth: an iteration-0 stall would mean the
    // marching criterion was the binding one after all, and the claim above
    // that it is conservative here would be wrong.
    let label = failure.expect("the ramp must reach a rung it cannot solve");
    assert!(
        !label.contains("iter 0"),
        "the cavity now ends on an iteration-0 stall ({label}), which IS the \
         marching-feasibility failure the kappa floor bounds — so the floor is \
         binding on this geometry and the conservative-by-a-millimetre reading \
         above is stale",
    );
    let crossing = first_infeasible.expect("the ramp must cross the required standoff");
    assert!(
        crossing < deepest,
        "the ramp never converged past the required standoff, so there is no \
         margin to call conservative",
    );
}

/// **The force-free traction, checked against an independent reading ON THE
/// CAVITY — not on the plate.**
///
/// [`the_force_free_traction_tracks_force_over_area_on_the_plate`] is necessary
/// and not sufficient: its oracle lives on the flat cell, so it says the two
/// definitions agree *where both are defined* and says nothing about the cell
/// that actually uses the force-free one. Without this gate the cavity's `σ` —
/// and therefore its `ρ`, and therefore item 4's inherited floor — rests
/// entirely on the definition transferring.
///
/// ★ **The check exists precisely BECAUSE of the geometry that killed
/// `F_z`.** Vector sums cancel on a closed patch; magnitude sums do not. So
/// `Σ|f| / A_cavity_rest` is a second reading of the same traction, built from
/// the readout's forces and the mesh's rest area, sharing no arithmetic with
/// [`Press::mean_traction`]'s barrier evaluation.
///
/// **Measured**: the two agree to **1.0162 falling to 1.0074** across the gate
/// ramp — an order of magnitude tighter than the plate's 6.4 %, and tightening
/// as the load rises.
///
/// ⭐ That contrast is itself evidence about the plate's unexplained 6.4 %: the
/// discrepancy is ~0 on a UNIFORM patch and 6.4 % on a NON-UNIFORM one, which
/// locates it in patch heterogeneity. It still does not identify the mechanism,
/// and this gate does not claim to.
#[test]
fn the_force_free_traction_agrees_with_an_independent_reading_on_the_cavity() {
    let cell = shell_cell();
    let (_, cavity_area_rest) = boundary_faces_on_isosurface(
        Mesh::<Yeoh>::positions(&cell.mesh),
        Mesh::<Yeoh>::boundary_faces(&cell.mesh),
        &SphereSdf { radius: R_CAVITY },
        0.0,
    );
    assert!(
        cavity_area_rest > 0.0,
        "no boundary face lies on the cavity isosurface, so there is no rest \
         area to divide by and the independent reading does not exist",
    );

    let mut ratios = Vec::new();
    eprintln!(
        "  cavity rest area {:.1} mm2\n  {:>8} {:>12} {:>14} {:>8}",
        cavity_area_rest * 1e6,
        "w(mm)",
        "mean_t(Pa)",
        "sum|f|/A(Pa)",
        "ratio",
    );
    for (w, r) in cavity_rungs().iter().cloned() {
        let p = r
            .map_err(|e| format!("cavity rung {w:e} m failed: {e}"))
            .expect("every cavity rung must converge for the two readings to be compared");
        let sigma_force = p.sum_force_mag / cavity_area_rest;
        eprintln!(
            "  {:>8.3} {:>12.1} {:>14.1} {:>8.4}",
            w * 1e3,
            p.mean_traction,
            sigma_force,
            p.mean_traction / sigma_force,
        );
        ratios.push(p.mean_traction / sigma_force);
    }

    let (lo, hi) = ratios
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &r| {
            (l.min(r), h.max(r))
        });
    assert!(
        (0.97..1.05).contains(&lo) && (0.97..1.05).contains(&hi),
        "the two independent readings of the cavity's traction differ by \
         [{lo:.4}, {hi:.4}], outside the measured 1.0074-1.0162 band — on a \
         patch with a uniform gap they have no room to disagree, so one of them \
         is not reading the traction",
    );
    // Tighter than the plate's, and that ordering is the load-bearing part: if
    // the cavity ever disagreed MORE than the plate, the heterogeneity reading
    // of the plate's 6.4 % would be backwards.
    assert!(
        hi - 1.0 < 0.064,
        "the cavity's readings now disagree by {:.1} %, at or beyond the plate's \
         6.4 % — which inverts the one thing that 6.4 % is currently explained \
         by (a uniform patch has less room to disagree, not more)",
        100.0 * (hi - 1.0),
    );
}

/// **Is the enveloping patch's `ρ` a property of the GEOMETRY or of the MESH?**
///
/// [`the_flat_patchs_nonuniformity_constant_bounds_the_enveloping_patch`]
/// reports `ρ ≈ 1.18` and this file states that on a sphere — whose gap is
/// uniform by symmetry — that number is what the DISCRETISATION contributes
/// rather than what shape irregularity would. That was an explanation with no
/// referent until this gate; it is now a measurement.
///
/// Holds the geometry exactly fixed and moves only `cell_size`. **Measured at
/// rest contact**: `ρ` = 1.1106 (5 mm) · 1.1759 (4 mm) · 1.0552 (3 mm). It
/// moves by ±6 % with the mesh alone, which is the claim; it is **not
/// monotone**, which is not claimed and is not explained here.
///
/// ⇒ The `[1.176, 1.221]` range quoted elsewhere in this file is a
/// `CELL = 4 mm` statement, not a property of a spherical cavity.
///
/// `#[ignore]` — three meshes, the finest 19 752 tets.
#[test]
#[ignore = "three remeshes of the shell, the finest 19 752 tets — the \
            measurement behind 'rho here is discretisation, not shape'"]
fn the_enveloping_patch_nonuniformity_is_a_property_of_the_mesh() {
    let mut rows = Vec::new();
    eprintln!(
        "  {:>9} {:>8} {:>7} {:>12} {:>8}",
        "cell(mm)", "tets", "pairs", "min_sd(m)", "rho"
    );
    for cell in [0.005_f64, 0.004, 0.003] {
        let tet4 = tet4_shell_at(cell);
        let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
        let pins = outer_skin_pins(&mesh, 0.25 * cell);
        assert!(!pins.is_empty(), "no outer-skin pin at cell {cell:e} m");
        let shell_cell = Cell {
            mesh,
            pins,
            rest_advance: REST_BORE_W,
            pin_label: "outer skin",
        };
        let p = shell_cell
            .press(CAVITY, REST_BORE_W)
            .map_err(|e| format!("cell {cell:e} m rest contact failed: {e}"))
            .expect("every cell size must reach rest contact for rho to be compared");
        let rho = face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd;
        eprintln!(
            "  {:>9.0} {:>8} {:>7} {:>12.4e} {:>8.4}",
            cell * 1e3,
            Mesh::<Yeoh>::n_tets(&tet4),
            p.n_pairs,
            p.min_sd,
            rho,
        );
        rows.push((cell, rho));
    }

    let (lo, hi) = rows
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &(_, r)| {
            (l.min(r), h.max(r))
        });
    assert!(
        lo > 1.0 && hi < PATCH_NONUNIFORMITY,
        "rho spans [{lo:.4}, {hi:.4}] across cell sizes, outside \
         (1, {PATCH_NONUNIFORMITY}) — on a sphere the gap is uniform by \
         symmetry, so anything here is discretisation and it has left the band \
         the kappa floor is derived within",
    );
    // The claim is that the MESH moves it. If three cell sizes gave the same
    // rho, that claim would be false and the number would be saying something
    // about the geometry instead.
    assert!(
        hi - lo > 0.02,
        "rho moved only {:.4} across cell sizes 3-5 mm, so it is NOT \
         discretisation-driven and this file's explanation of where ~1.18 comes \
         from is wrong",
        hi - lo,
    );
}

/// `STACK` with per-layer tensile caps overridden — the fail-close lever.
fn graded_field_capped(caps: [f64; 3]) -> MaterialField {
    graded_field_with(
        [STACK[0].mu, STACK[1].mu, STACK[2].mu],
        [STACK[0].c2, STACK[1].c2, STACK[2].c2],
        [STACK[0].lambda, STACK[1].lambda, STACK[2].lambda],
        caps,
    )
}

/// Every per-layer parameter given explicitly — the seam
/// [`the_graded_stiffening_is_carried_by_the_linear_modulus`] needs, because
/// flattening one
/// parameter while the others stay graded is the only way to attribute the
/// response to a parameter rather than to "the material".
fn graded_field_with(
    mu: [f64; 3],
    c2: [f64; 3],
    lambda: [f64; 3],
    caps: [f64; 3],
) -> MaterialField {
    fn layered(values: [f64; 3]) -> Box<dyn Field<f64>> {
        Box::new(LayeredScalarField::new(
            Box::new(SphereSdf { radius: R_CAVITY }),
            LAYER_BOUNDARIES.to_vec(),
            values.to_vec(),
        ))
    }
    MaterialField::from_yeoh_fields_with_bounds(
        layered(mu),
        layered(c2),
        layered(lambda),
        layered(caps),
        layered([
            STACK[0].validity_min_principal_stretch,
            STACK[1].validity_min_principal_stretch,
            STACK[2].validity_min_principal_stretch,
        ]),
    )
}

// ── the graded wall ─────────────────────────────────────────────────

/// Shallow graded ramp — the always-on depth, matching [`ramp_cavity`].
fn ramp_graded(max_w: f64) -> Vec<Rung> {
    graded_cell().ramp(CAVITY, RAMP_STEP, max_w)
}

/// Rest-configuration centroid radius of `tet` (m) — the coordinate the layer
/// stack is keyed on, so it is what says which layer a tet belongs to.
fn centroid_radius<M: sim_soft::Material>(mesh: &dyn Mesh<M>, tet: TetId) -> f64 {
    let positions = mesh.positions();
    let verts = mesh.tet_vertices(tet);
    (verts.iter().map(|&v| positions[v as usize]).sum::<Vec3>() * 0.25).norm()
}

/// Per-tet `(cap, count, min centroid radius, max centroid radius)` over the
/// graded shell, one entry per distinct tensile cap, innermost band first.
///
/// The cap is the label because the three anchors carry three distinct ones
/// (7.56 / 8.80 / 5.76) and [`Yeoh`] exposes no modulus accessor. ⚠ That makes
/// this a statement about the mesh, NOT about the energy — the solver carries
/// the cap but never evaluates it unless a violation fires.
/// [`the_graded_walls_stiffness_is_set_by_the_layer_the_load_enters`] is the
/// partner gate that reads the moduli through the solve.
fn graded_bands() -> &'static [(f64, usize, f64, f64)] {
    static BANDS: OnceLock<Vec<(f64, usize, f64, f64)>> = OnceLock::new();
    BANDS.get_or_init(build_graded_bands)
}

/// The graded Tet4 mesh, built once — `graded_bands` and the population
/// assertion both need it, and meshing it twice was the same duplication the
/// ramp caches removed.
fn graded_tet4() -> &'static SdfMeshedTetMesh<Yeoh> {
    static MESH: OnceLock<SdfMeshedTetMesh<Yeoh>> = OnceLock::new();
    MESH.get_or_init(|| tet4_shell_with(CELL, graded_yeoh_field()))
}

fn build_graded_bands() -> Vec<(f64, usize, f64, f64)> {
    let mesh = graded_tet4();
    let mut bands: Vec<(f64, usize, f64, f64)> = Vec::new();
    for (tet, material) in Mesh::<Yeoh>::materials(mesh).iter().enumerate() {
        let cap = material
            .validity()
            .max_principal_stretch
            .unwrap_or(f64::NAN);
        let radius = centroid_radius(mesh, tet as TetId);
        match bands.iter_mut().find(|b| (b.0 - cap).abs() < 1e-12) {
            Some(band) => {
                band.1 += 1;
                band.2 = band.2.min(radius);
                band.3 = band.3.max(radius);
            }
            None => bands.push((cap, 1, radius, radius)),
        }
    }
    bands.sort_by(|a, b| a.2.total_cmp(&b.2));
    bands
}

/// **Does the layer stack reach the mesh, keyed on radius and the right way
/// round?**
///
/// The cheapest thing that can be wrong about a graded field is also the
/// hardest to see downstream: a sign flip applies the stack outward-in, a
/// threshold in the wrong units collapses it to one material, and the solve
/// still converges and still reports plausible numbers in every case. So this
/// pins the partition itself — three populations, each confined to its band,
/// soft side at the bore.
///
/// ⚠ **Equal thickness is not equal population on a sphere, and it is not the
/// `r³` volume share either.** Three 4 mm layers hold 17.6 / 26.4 / 56.0 % of
/// the tets where the volume shares are 19.0 / 33.8 / 47.2 %. The counts are
/// lattice-quantised (1536 = 3 · 512, 2304 = 9 · 256), so continuum arithmetic
/// mis-sizes the middle layer by 18 %. Measure the split; do not derive it.
#[test]
fn the_layer_stack_partitions_the_wall_by_radius() {
    let bands = graded_bands();
    for (cap, count, lo, hi) in bands {
        eprintln!(
            "  cap {cap:.2} : {count:5} tets, centroid r [{:.2}, {:.2}] mm",
            lo * 1e3,
            hi * 1e3,
        );
    }

    assert_eq!(
        bands.len(),
        STACK.len(),
        "the graded field produced {} distinct per-tet materials, not the {} \
         the stack declares — a collapsed partition still solves and still \
         reports plausible numbers, which is why this is asserted and not \
         inferred",
        bands.len(),
        STACK.len(),
    );
    assert_eq!(
        bands.iter().map(|b| b.1).collect::<Vec<_>>(),
        vec![1536, 2304, 4896],
        "the layer populations moved. They are a function of `CELL`, \
         `LAYER_BOUNDARIES` and the shell geometry alone, so re-measure and \
         re-pin deliberately rather than loosening this",
    );
    assert_eq!(
        bands.iter().map(|b| b.1).sum::<usize>(),
        Mesh::<Yeoh>::n_tets(graded_tet4()),
        "the bands do not account for every tet",
    );

    // Each band confined between the boundaries that define it — EXACTLY, with
    // no tolerance, and the reason is worth stating because the obvious
    // version of this check is nearly vacuous.
    //
    // `materials_from_field`'s `cache_walk` samples the field at
    // `(v0 + v1 + v2 + v3) * 0.25` (`sim/L0/soft/src/material/material_field.rs`),
    // which is the same expression [`centroid_radius`] recomputes. So a tet's
    // band is *decided* by the quantity compared here and cannot be off by any
    // margin at all. Comparisons run in the field's own space — `radius −
    // R_CAVITY` against [`LAYER_BOUNDARIES`], not `radius` against a
    // precomputed edge — so the two sides associate their arithmetic the same
    // way and the boundary convention (`phi == threshold` lands in the OUTER
    // shell) is reproduced rather than approximated.
    //
    // ⚠ A first cut allowed `± CELL`. That is ±4 mm on 4 mm layers: a
    // partition shifted by a whole layer passes it. The gate stayed honest
    // only because the population counts above are pinned.
    for (i, (cap, _, lo, hi)) in bands.iter().enumerate() {
        if let Some(inner) = i.checked_sub(1).and_then(|j| LAYER_BOUNDARIES.get(j)) {
            assert!(
                lo - R_CAVITY >= *inner,
                "band {i} (cap {cap:.2}) starts at {:.3} mm from the bore, \
                 inside its own lower boundary at {:.3} mm — the field is not \
                 keyed on the centroid radius this reads",
                (lo - R_CAVITY) * 1e3,
                inner * 1e3,
            );
        }
        if let Some(outer) = LAYER_BOUNDARIES.get(i) {
            assert!(
                hi - R_CAVITY < *outer,
                "band {i} (cap {cap:.2}) reaches {:.3} mm from the bore, at or \
                 past its upper boundary at {:.3} mm",
                (hi - R_CAVITY) * 1e3,
                outer * 1e3,
            );
        }
    }
    // The outer extremes are a MESH property, not a field one: nothing forces
    // a tet straddling a curved surface to keep its centroid inside the body.
    // Measured [10.03, 21.65] mm, so it holds here and is asserted as what it
    // is rather than folded into the exact checks above.
    assert!(
        bands[0].2 >= R_CAVITY && bands[bands.len() - 1].3 <= R_OUTER,
        "centroids run [{:.2}, {:.2}] mm, outside the shell's [{:.2}, {:.2}] mm \
         — the mesher placed a centroid beyond the body it meshed",
        bands[0].2 * 1e3,
        bands[bands.len() - 1].3 * 1e3,
        R_CAVITY * 1e3,
        R_OUTER * 1e3,
    );

    // Soft side at the bore. `insertion_sim` builds the stack innermost-first
    // and layer 0 is what touches the intruder; an inverted stack is the one
    // error that leaves every count above unchanged.
    let caps: Vec<f64> = bands.iter().map(|b| b.0).collect();
    let declared: Vec<f64> = STACK
        .iter()
        .map(|m| m.validity_max_principal_stretch)
        .collect();
    assert_eq!(
        caps, declared,
        "the caps run {caps:?} from the bore outward but the stack declares \
         {declared:?} — the layer order is reversed, which no population count \
         can see",
    );
}

/// **Do the per-tet moduli reach the ENERGY, or only the mesh?**
///
/// [`the_layer_stack_partitions_the_wall_by_radius`] reads the per-tet
/// validity cap, which the solver carries but never evaluates unless a
/// violation fires — so it would pass unchanged against a field whose `μ` was
/// constant. This is its partner: the same ramp on the uniform and graded
/// cells, and the only thing that differs between them is the material.
///
/// ⭐⭐ **The measured stiffening is half what a volume average predicts, and
/// that is the transferable finding.** Volume-weighting the stack gives
/// 79.9 kPa against Ecoflex 00-30's 23.0 kPa — a 3.48× body. The ramp reads
/// **1.222× at rest rising to 1.332×** over this gate's depth (and 1.719× at
/// the convergence wall, in
/// [`how_deep_does_the_graded_cavity_converge`]).
///
/// The reason has a consequence, so it is worth stating: **the load enters at
/// the bore, and there the graded stack is SOFTER than the uniform baseline**
/// — Ecoflex 00-20 at 18 kPa against 00-30's 23 kPa. The Dragon Skin shells
/// carrying 56 % of the volume sit far from the contact and up against the
/// pinned skin. The layers load in series, not in parallel, so the volume
/// share is the wrong weight. ⇒ **sizing a graded sleeve from a volume-averaged
/// modulus over-predicts its stiffness by about 2×.**
///
/// ⭐ **The estimator finding does not depend on WHICH modulus is averaged.**
/// Volume-weighting gives 3.476× for `μ`, 3.476× for `λ` and 3.453× for `C₂`,
/// because the anchors are a self-similar family
/// ([`the_stack_is_a_self_similar_family`]). So this is a statement about
/// volume-averaging a graded wall, not about `μ` in particular.
///
/// ⚠⚠ **Two earlier revisions of this comment attributed the effect to a
/// single Lamé parameter — first `μ`, then `λ` — and BOTH were wrong.** The
/// first was caught by a surviving mutation; the second only by asking what
/// the parameters physically are. `λ = 4μ` exactly for every anchor, so
/// holding one fixed while the other grades produces a body whose ν varies by
/// layer, which is not a silicone. There is no `μ`-versus-`λ` split to make,
/// and the "`λ` leads because the shell is sealed" reasoning built on it is
/// **retracted**. The split that is physical —
/// [`the_graded_stiffening_is_carried_by_the_linear_modulus`] — holds ν fixed
/// and finds the linear modulus carries **18.5×** what `C₂` does.
///
/// ⚠ **The open-mouth caveat stands, on its own evidence rather than on that
/// reasoning.** Item 3a independently measured that this sealed cell forces
/// 22.7 % volumetric compression at depth and is therefore stiffer than the
/// open-mouth sleeve it proxies. ⇒ read the band as an **upper bound** for
/// `insertion_sim`, because the CELL is stiffer, not because of anything about
/// which parameter carries the grading.
///
/// The ratio *rising* with depth is the same mechanism seen from the other
/// side: as the wall compresses, load transfers outward into the stiff layers.
#[test]
fn the_graded_walls_stiffness_is_set_by_the_layer_the_load_enters() {
    let uniform = cavity_rungs();
    let graded = graded_rungs();
    assert_eq!(
        uniform.len(),
        graded.len(),
        "the two cells did not ramp over the same advances, so no rung-by-rung \
         ratio below is comparing like with like",
    );
    let rungs = &graded;
    // ⛔ An empty or truncated ramp makes every `all(..)` below vacuously
    // true. Assert the collection reached the declared depth, not just that
    // nothing in it was wrong.
    assert!(
        rungs.len() > 1 && rungs.last().is_some_and(|(w, _)| *w >= CAVITY_GATE_MAX_W),
        "the ramp produced {} rungs ending at {:?} mm, short of the declared \
         {:.3} mm — every `all(..)` over it below would pass on an empty set",
        rungs.len(),
        rungs.last().map(|(w, _)| w * 1e3),
        CAVITY_GATE_MAX_W * 1e3,
    );

    let mut ratios = Vec::new();
    for ((w, u), (wg, g)) in uniform.iter().zip(graded) {
        let u = u
            .as_ref()
            .expect("the uniform cell must converge over this gate's depth");
        let g = g
            .as_ref()
            .expect("the graded cell must converge over this gate's depth");
        assert!(
            (w - wg).abs() < 1e-12,
            "advance mismatch {w} vs {wg} — the zip above is misaligned",
        );
        eprintln!(
            "  {:>8.3} uniform {:>8.3} kPa  graded {:>8.3} kPa  ratio {:>6.4}",
            w * 1e3,
            u.mean_traction / 1e3,
            g.mean_traction / 1e3,
            g.mean_traction / u.mean_traction,
        );
        ratios.push(g.mean_traction / u.mean_traction);
    }

    let lo = ratios.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = ratios.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (1.20..1.35).contains(&lo) && (1.20..1.35).contains(&hi),
        "the graded/uniform traction ratio reads [{lo:.4}, {hi:.4}], outside \
         the measured 1.222-1.332 band. Below 1.0 the stack is inverted; at \
         1.0 the moduli never reached the energy and only the mesh was graded",
    );
    assert!(
        ratios.windows(2).all(|w| w[1] > w[0]),
        "the ratio is not strictly increasing: {ratios:?}. It rises because \
         load transfers outward into the stiff layers as the wall compresses, \
         so a flat or falling ratio means the grading is not where it is \
         thought to be",
    );

    // The estimator finding, made executable rather than asserted in prose.
    let bands = graded_bands();
    // The zip below pairs `STACK[i]` with `bands[i]`, which is only right
    // because `graded_bands` sorts innermost-first and so does `STACK`. That
    // coupling is asserted by `the_layer_stack_partitions_the_wall_by_radius`
    // — but this gate must not depend on another test having run, so it is
    // re-checked here where the zip actually happens.
    assert!(
        STACK
            .iter()
            .zip(bands)
            .all(|(m, b)| (m.validity_max_principal_stretch - b.0).abs() < 1e-12),
        "the bands are not in stack order, so the volume weighting below pairs \
         each layer's modulus with another layer's population",
    );
    let total: usize = bands.iter().map(|b| b.1).sum();
    let volume_weighted = STACK
        .iter()
        .zip(bands)
        .map(|(m, b)| m.mu * (b.1 as f64))
        .sum::<f64>()
        / (total as f64);
    let predicted = volume_weighted / ECOFLEX_00_30.mu;
    eprintln!(
        "  volume-weighted mu {:.1} kPa = {predicted:.2}x uniform; measured \
         stiffening tops out at {hi:.3}x",
        volume_weighted / 1e3,
    );
    assert!(
        predicted / hi > 2.4,
        "the volume average over-predicts the measured stiffening by only \
         {:.2}x. The whole point of this gate is that it over-predicts by ~2.6x \
         because the load enters through the SOFT layer — if that gap closed, \
         the series-compliance explanation is wrong",
        predicted / hi,
    );
    assert!(
        STACK[0].mu < ECOFLEX_00_30.mu,
        "the series-compliance reading above requires the innermost layer \
         ({} Pa) to be SOFTER than the uniform baseline ({} Pa); with a \
         stiffer bore layer the volume average would no longer over-predict \
         for the reason claimed, and this gate's band is a coincidence",
        STACK[0].mu,
        ECOFLEX_00_30.mu,
    );
}

/// **What does the shipped interface-flag test actually resolve at this cell
/// size?**
///
/// Ladder item 4 (per-GP material) plans to "ship with a decision about
/// flagged tets" rather than sampling 4× everywhere, and the flag it means is
/// [`Mesh::interface_flags`], populated by the book's `|φ(x_c)| < L_e` straddle
/// rule with `L_e` the tet's six-edge mean. This gate measures what that
/// selects here, because a decision applied to most of the body is a different
/// decision from one applied to a seam.
///
/// ⛔⛔ **The flag is SATURATED at `CELL` = 4 mm and cannot isolate these
/// boundaries.** `L_e` is ≈3.4 mm against 4 mm layers, so the band is nearly as
/// thick as the layer: **3 504 of 8 736 tets (40.1 %)** straddle the r = 14 mm
/// boundary and **5 484 (62.8 %)** straddle r = 18 mm. The sharpest form of it
/// — **boundary 0 flags more tets (3 504) than the entire layer it bounds
/// contains (2 304)**.
///
/// ⚠ This is a property of the RESOLUTION, not a defect in the rule or in this
/// fixture: the rule is scale-relative by construction and the mesh is simply
/// too coarse for 4 mm layers. ⇒ item 4 needs either a finer cell or a
/// criterion that is not `L_e`-wide. ⭐ And this is a *lower* bound on the
/// problem: these boundaries are exactly concentric spheres, so a real scan's
/// irregular offsets can only flag more.
#[test]
fn the_interface_flag_cannot_isolate_a_layer_boundary_at_this_cell_size() {
    let bands = graded_bands();
    let mut flagged = Vec::new();
    for offset in LAYER_BOUNDARIES {
        let mesh = tet4_shell_with(
            CELL,
            graded_yeoh_field().with_interface_sdf(Box::new(SphereSdf {
                radius: R_CAVITY + offset,
            })),
        );
        let flags = Mesh::<Yeoh>::interface_flags(&mesh);
        let hits = flags.iter().filter(|&&f| f).count();
        eprintln!(
            "  boundary at r = {:.1} mm : {hits} of {} tets straddle ({:.1} %)",
            (R_CAVITY + offset) * 1e3,
            flags.len(),
            100.0 * (hits as f64) / (flags.len() as f64),
        );
        flagged.push(hits);
    }

    assert_eq!(
        flagged,
        vec![3504, 5484],
        "the straddle counts moved. They are what ladder item 4's flagged-tet \
         decision would apply to, so a change here changes that decision's \
         blast radius — re-measure and re-pin, do not loosen",
    );

    let total = bands.iter().map(|b| b.1).sum::<usize>();
    assert!(
        flagged.iter().all(|&f| 3 * f > total),
        "the flag now selects under a third of the mesh ({flagged:?} of \
         {total}), so it has become able to isolate an interface at this cell \
         size. That is good news and this gate's premise is stale — item 4's \
         note about flagged tets needs rewriting, not this assertion relaxing",
    );
    assert!(
        flagged[0] > bands[1].1,
        "boundary 0 flags {} tets while the middle layer it bounds holds {} — \
         the band being WIDER than the layer is the whole finding, and it no \
         longer holds",
        flagged[0],
        bands[1].1,
    );
}

/// **Does grading break the solve, or either reading item 3a left the bridge?**
///
/// 3a handed forward two readings that survive an enveloping patch: the
/// force-free traction (because `F_z / A_flat` dies when radial normals cancel)
/// and `ρ`, the min-gap-versus-mean-traction reconciliation that
/// [`PATCH_NONUNIFORMITY`] bounds. Both were measured on a uniform body. If
/// either turned out to be a property of material uniformity rather than of
/// the geometry, the bridge could not lean on it — `insertion_sim`'s wall is
/// layered.
///
/// Measured here: the graded cavity converges at every rung, `ρ` lands in
/// **[1.1632, 1.1761]** against the uniform cell's [1.1683, 1.1766] over the
/// same span, and the cancellation coherence runs **1.068e-3 → 4.023e-4**
/// against the uniform cell's 1.11e-3 → 1.45e-4. ⇒ **both readings are
/// geometric.** Grading widens the `ρ` band by about ±4 % and moves nothing
/// else.
#[test]
fn grading_keeps_the_enveloping_patchs_invariants() {
    let rungs = graded_rungs();
    // ⛔ An empty or truncated ramp makes every `all(..)` below vacuously
    // true. Assert the collection reached the declared depth, not just that
    // nothing in it was wrong.
    assert!(
        rungs.len() > 1 && rungs.last().is_some_and(|(w, _)| *w >= CAVITY_GATE_MAX_W),
        "the ramp produced {} rungs ending at {:?} mm, short of the declared \
         {:.3} mm — every `all(..)` over it below would pass on an empty set",
        rungs.len(),
        rungs.last().map(|(w, _)| w * 1e3),
        CAVITY_GATE_MAX_W * 1e3,
    );
    eprintln!(
        "  {:>8} {:>4} {:>10} {:>10} {:>7} {:>11} {:>6}",
        "w(mm)", "it", "resid", "min_sd", "rho", "coherence", "pairs",
    );
    let mut ok = Vec::new();
    for (w, r) in rungs {
        let p = r
            .as_ref()
            .map_err(|e| format!("the graded cavity must converge at {w}: {e}"))
            .expect("graded rung");
        let rho = face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd;
        let coherence = p.net_force.norm() / p.sum_force_mag;
        eprintln!(
            "  {:>8.3} {:>4} {:>10.2e} {:>10.3e} {:>7.4} {:>11.3e} {:>6}",
            w * 1e3,
            p.iters,
            p.residual,
            p.min_sd,
            rho,
            coherence,
            p.n_pairs,
        );
        ok.push((p, rho, coherence));
    }

    assert_eq!(ok.len(), rungs.len(), "a rung failed");
    assert!(
        ok.iter().all(|(p, ..)| p.iters <= 8 && p.residual < 1.0e-8),
        "grading cost convergence quality: the uniform cell holds 4-6 Newton \
         iterations and a residual under 1e-8 over this span",
    );
    assert!(
        ok.iter().all(|(p, ..)| p.min_sd > RAMP_STEP),
        "a rung closed the gap below one ramp step, so the next increment is \
         marching into the barrier rather than onto it",
    );

    let rho_lo = ok.iter().map(|&(_, r, _)| r).fold(f64::INFINITY, f64::min);
    let rho_hi = ok
        .iter()
        .map(|&(_, r, _)| r)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        rho_lo > 1.0 && rho_hi < PATCH_NONUNIFORMITY,
        "graded rho reads [{rho_lo:.4}, {rho_hi:.4}], outside \
         (1.0, {PATCH_NONUNIFORMITY}). Below 1.0 the ratio is inverted; above \
         the constant, the kappa floor derived from it no longer covers a \
         graded wall and `insertion_sim` cannot use it",
    );
    assert!(
        ok.iter().all(|&(_, _, c)| c < 1.0e-2),
        "the graded patch stopped cancelling its own net force, so the \
         cancellation was a property of material uniformity and not of the \
         enveloping geometry — which would retract 3a's reason for reading \
         traction instead of force",
    );

    // Topology, not material: the same mesh and the same rigid bore, so the
    // set of engaged faces should not know what the tets are made of.
    // Topology, not material. ⚠ The claim is "the SAME as the uniform cell",
    // so the oracle has to BE the uniform cell — an earlier revision asserted
    // the literal 434 while the message spoke about a cell it never read, so
    // a joint move would have passed with the message still claiming a
    // comparison. Rule 3c: the relational check is the claim, the pinned
    // literal catches a drift that moves both together.
    let pairs: Vec<usize> = ok.iter().map(|(p, ..)| p.n_pairs).collect();
    let uniform_pairs: Vec<usize> = cavity_rungs()
        .iter()
        .map(|(_, r)| r.as_ref().expect("the uniform cell must converge").n_pairs)
        .collect();
    assert_eq!(
        pairs, uniform_pairs,
        "the graded cell engages a different active set from the uniform one. \
         The set of engaged faces is a function of the mesh and the indenter, \
         so a material-dependent one means the whole-wall-at-once regime is \
         not purely geometric",
    );
    assert!(
        pairs.iter().all(|&n| n == 434),
        "both cells engage {pairs:?} pairs, agreeing with each other but not \
         with the 434 measured — the geometry or the indenter moved, which the \
         equality above cannot see",
    );
}

/// **Is the per-tet validity gate — the one that walled row 23 — live on this
/// path, and does it name the right layer?**
///
/// ⛔ **The realistic stack never reaches it, and that is why this gate has to
/// force the issue.** The anchors cap tensile stretch at 7.56 / 8.80 / 5.76;
/// [`grading_keeps_the_enveloping_patchs_invariants`] carries the tightest of
/// those on 56 % of its tets and the ramp still ends in an `ArmijoStall`. So a
/// gate written against the shipped caps would pass forever without the
/// validity path ever executing — vacuous in the exact way a green run cannot
/// show. Dropping the innermost cap to 1.10 is what makes it fire.
///
/// Measured: `ValidityViolation` at **w = 0.220 mm, tet 1781**,
/// `max_principal_stretch = 1.102` against the 1.100 bound, singular values
/// **[1.102, 1.026, 0.807]** — hoop tension against radial compression, which
/// is the closing-cavity kinematics. Its centroid sits at **r = 11.12 mm**,
/// inside layer 0's [10.03, 13.75] mm.
///
/// ⭐ **The attribution is sound by construction, not by luck.** The solver
/// reports the lowest-id violator, which is not the same as the first one
/// geometrically — but the other two layers keep caps of 8.80 and 5.76, which
/// this ramp comes nowhere near, so every tet that *can* violate is in layer 0.
#[test]
fn the_per_tet_validity_gate_fires_in_the_layer_that_owns_the_cap() {
    const CAPPED_LAYER_STRETCH: f64 = 1.10;
    let cell = Cell::capped_shell([
        CAPPED_LAYER_STRETCH,
        STACK[1].validity_max_principal_stretch,
        STACK[2].validity_max_principal_stretch,
    ]);

    let failure = cell
        .ramp(CAVITY, RAMP_STEP, CAVITY_GATE_MAX_W)
        .into_iter()
        .find_map(|(w, r)| r.err().map(|e| (w, e)));
    let (w, label) = failure
        .ok_or_else(|| {
            format!(
                "the whole ramp converged with the innermost layer capped at \
                 {CAPPED_LAYER_STRETCH}, so the per-tet validity gate never \
                 fired — either the cap is not reaching the solver, or this \
                 cell no longer stretches the bore layer past it",
            )
        })
        .expect("a capped ramp must fail");
    eprintln!("  capped ramp failed at {:.3} mm: {label}", w * 1e3);

    // ⚠ Two different failures, and an earlier revision reported both as the
    // first: a ramp that stalled instead of violating, and a ramp that DID
    // violate under a `failure_label` whose format moved. The second would
    // have been reported as "not a ValidityViolation", which is false.
    assert!(
        label.starts_with("ValidityViolation"),
        "the capped ramp failed with `{label}` instead of a ValidityViolation. \
         A stall or a factor failure means the low cap changed the SOLVE \
         rather than tripping the material gate, and the gate is unexercised",
    );
    let rest = label
        .strip_prefix("ValidityViolation(tet ")
        .ok_or_else(|| {
            format!(
                "a ValidityViolation fired, but `failure_label` no longer \
                 formats it as `ValidityViolation(tet N: ...)`, so the tet \
                 cannot be recovered from `{label}`. The gate is sound and its \
                 parser is stale — fix the parser, not the solver",
            )
        })
        .expect("the tet id");
    let tet: TetId = rest
        .split(':')
        .next()
        .and_then(|t| t.trim().parse().ok())
        .ok_or_else(|| format!("no tet id in `{label}`"))
        .expect("a tet id");

    assert!(
        label.contains("max_principal_stretch"),
        "the violation fired on something other than the tensile cap this gate \
         lowered: {label}",
    );
    let radius = centroid_radius(&cell.mesh, tet);
    let inner_edge = R_CAVITY + LAYER_BOUNDARIES[0];
    assert!(
        radius < inner_edge,
        "the gate named tet {tet} at r = {:.2} mm, outside the capped layer's \
         [{:.2}, {:.2}] mm. Only layer 0 carries a reachable cap, so a violator \
         anywhere else means the field is not keyed on radius the way \
         `the_layer_stack_partitions_the_wall_by_radius` reads it",
        radius * 1e3,
        R_CAVITY * 1e3,
        inner_edge * 1e3,
    );
}

/// How deep does the graded wall go, and how does it stop?
///
/// The always-on gates run to [`CAVITY_GATE_MAX_W`]; this is the full ramp to
/// the wall, and it is where the depth and stiffening numbers the other gates
/// quote in prose are actually produced.
///
/// ⭐⭐ **Grading costs a third of the depth: 3.220 mm against the uniform
/// cell's 4.720 mm**, with the traction ratio climbing to **1.719×** — still
/// only half the 3.48× a volume-weighted modulus predicts, and by the same
/// series-compliance argument
/// [`the_graded_walls_stiffness_is_set_by_the_layer_the_load_enters`] makes.
///
/// ⭐⭐ **The stall MODE changes, and that is the part worth carrying
/// forward.** The uniform cavity ends at `ArmijoStall(iter 5)` — 3a recorded
/// that this is *not* the marching-feasibility mode the κ floor is derived
/// against, because that one stalls at iteration **0**. The graded cavity ends
/// at `ArmijoStall(iter 0, r 8.191e4)`, which is that mode. ⇒ on a graded wall
/// the derived floor is describing the failure it was built to describe.
/// The feasibility criterion stays conservative either way: graded `min_sd`
/// drops under [`REQUIRED_STANDOFF`] at 2.720 mm and converges six more rungs,
/// where the uniform cell crossed at 3.820 mm and converged ten more.
#[test]
#[ignore = "one full ramp to the graded convergence wall, ~4 min — the depth, \
            stall mode and rho band the always-on graded gates quote"]
fn how_deep_does_the_graded_cavity_converge() {
    let rungs = ramp_graded(0.006);
    eprintln!(
        "  {:>8} {:>4} {:>10} {:>10} {:>7} {:>10}",
        "w(mm)", "it", "trac(kPa)", "min_sd", "rho", "feasible",
    );
    let mut deepest = f64::NEG_INFINITY;
    let mut rho_lo = f64::INFINITY;
    let mut rho_hi = f64::NEG_INFINITY;
    let mut first_infeasible = None;
    let mut failure = None;
    for (w, r) in &rungs {
        match r {
            Ok(p) => {
                let rho = face_barrier_standoff(KAPPA, D_HAT, p.mean_traction) / p.min_sd;
                let feasible = p.min_sd > REQUIRED_STANDOFF;
                if !feasible && first_infeasible.is_none() {
                    first_infeasible = Some(*w);
                }
                eprintln!(
                    "  {:>8.3} {:>4} {:>10.3} {:>10.3e} {:>7.4} {:>10}",
                    w * 1e3,
                    p.iters,
                    p.mean_traction / 1e3,
                    p.min_sd,
                    rho,
                    feasible,
                );
                deepest = *w;
                rho_lo = rho_lo.min(rho);
                rho_hi = rho_hi.max(rho);
            }
            Err(e) => {
                eprintln!("  {:>8.3} FAILED {e}", w * 1e3);
                failure = Some(e.clone());
            }
        }
    }
    eprintln!(
        "  deepest converged {:.3} mm = {:.1} % radial interference; rho in \
         [{rho_lo:.4}, {rho_hi:.4}]; first rung below the required standoff \
         {:?} mm",
        deepest * 1e3,
        100.0 * deepest / R_CAVITY,
        first_infeasible.map(|w| w * 1e3),
    );

    assert!(
        deepest > 0.003,
        "the graded cavity walled at {:.3} mm, short of the 3.22 mm measured",
        deepest * 1e3,
    );
    assert!(
        deepest < 0.004,
        "the graded cavity reached {:.3} mm, at or past the uniform cell's \
         range — the 32 % depth cost of grading is the finding, so losing it \
         means the stack is not reaching the energy",
        deepest * 1e3,
    );
    // Both ends: an upper bound alone accepts `min_sd / d_eff`, the inverted
    // computation, which reads under 1.0 and would pass silently.
    assert!(
        rho_lo > 1.0 && rho_hi < PATCH_NONUNIFORMITY,
        "graded rho over the full ramp reads [{rho_lo:.4}, {rho_hi:.4}], \
         outside (1.0, {PATCH_NONUNIFORMITY})",
    );
    let label = failure.expect("the ramp must reach a wall within 6 mm");
    assert!(
        label.contains("ArmijoStall(iter 0"),
        "the graded wall is `{label}`, not the iteration-0 Armijo stall \
         measured. The ITERATION is the whole distinction between the \
         marching-feasibility mode the kappa floor describes and the \
         unidentified iteration-5 mode the uniform cell hits",
    );
    assert!(
        first_infeasible.is_some_and(|w| w < deepest),
        "the ramp never crossed the required standoff before walling, so the \
         feasibility criterion is no longer conservative here and the kappa \
         floor's margin on a graded wall is unmeasured",
    );
}

/// **Are the layer anchors independent materials, or one material scaled?**
///
/// Load-bearing premise, asserted because reasoning about this stack went
/// wrong once without it. Every anchor used here — the three in [`STACK`] and
/// the uniform baseline — satisfies `λ = 4μ` **exactly** and `C₂ ≈ 0.089 μ`.
/// They are a **self-similar one-parameter family**: ν = 0.400 throughout, and
/// a layer is the baseline scaled by a single stiffness factor.
///
/// ⇒ **`μ` and `λ` cannot be attributed separately on this stack.** Holding
/// one fixed while the other grades produces a body whose Poisson ratio varies
/// by layer, which is not a silicone. That is why
/// [`the_graded_stiffening_is_carried_by_the_linear_modulus`] splits
/// `(μ, λ)`-together against `C₂` instead.
///
/// ⚠ **If this gate ever fails, the retracted experiment becomes meaningful.**
/// An anchor whose `λ/μ` differs is a material with a different ν, and then
/// separating the Lamé parameters is a real question rather than an artefact.
/// So this is not a constant to update — it is a premise to re-read.
#[test]
fn the_stack_is_a_self_similar_family() {
    let all = [STACK[0], STACK[1], STACK[2], ECOFLEX_00_30];
    for m in all {
        let ratio = m.lambda / m.mu;
        let nu = m.lambda / (2.0 * (m.lambda + m.mu));
        eprintln!(
            "  mu {:>8.0} lambda {:>9.0} lambda/mu {ratio:.4} nu {nu:.4} C2/mu {:.4}",
            m.mu,
            m.lambda,
            m.c2 / m.mu,
        );
        assert!(
            (ratio - 4.0).abs() < 1.0e-12,
            "an anchor has lambda/mu = {ratio}, not 4. The Lamé pair is no \
             longer a single scaled parameter, so `mu` and `lambda` CAN be \
             attributed separately and the retracted mu-vs-lambda experiment \
             is meaningful again — re-read the reasoning, do not update this",
        );
        assert!(
            (nu - 0.40).abs() < 1.0e-12,
            "an anchor has nu = {nu}, not 0.40 — the family is no longer \
             self-similar",
        );
        assert!(
            (0.085..0.095).contains(&(m.c2 / m.mu)),
            "an anchor has C2/mu = {:.4}, outside the 0.0875-0.0939 band the \
             others share. C2 stops being a fixed small fraction of the linear \
             modulus, so treating it as a uniform small correction no longer \
             follows",
            m.c2 / m.mu,
        );
    }
}

/// **Which part of the constitutive law carries the graded wall's stiffening —
/// the linear modulus or Yeoh's `C₂` nonlinearity?**
///
/// ⛔⛔ **`μ` and `λ` CANNOT be separated on this stack, and an earlier
/// revision of this fixture tried to.** Every anchor in [`STACK`] satisfies
/// **`λ = 4μ` exactly** (ν = 0.400 by construction) and `C₂ ≈ 0.089 μ`, so the
/// four silicones are a **self-similar one-parameter family**. Flattening one
/// Lamé parameter alone lets Poisson's ratio float across the wall — measured
/// **0.379 → 0.476** flattening `μ`, **0.418 → 0.224** flattening `λ` — which
/// is not "the same material with one influence removed" but a different and
/// unrealisable material. The earlier conclusion drawn that way ("`λ` carries
/// twice what `μ` does, because the shell is sealed") was an artefact of that
/// and is **retracted**. [`the_stack_is_a_self_similar_family`] pins the
/// premise so the mistake cannot be made again silently.
///
/// The physical split holds ν fixed: `(μ, λ)` move together as the linear
/// modulus, against `C₂` as the nonlinear term. Both variants below are
/// realisable silicones.
///
/// ```text
/// baseline                  1.2218 -> 1.3320     excess 0.3320
/// linear (mu, lambda) flat  1.0488 -> 1.0566     excess 0.0566  (83 % gone)
/// C2 flat                   1.2131 -> 1.3171     excess 0.3171  (4.5 % gone)
/// all flat                  1.0000 -> 1.0000     excess 0
/// ```
///
/// ⭐⭐ **The linear modulus carries 18.5× what `C₂` does.** The stiffening is
/// a linear-elastic effect of the layer arrangement, not a nonlinear one — at
/// these stretches `C₂ ≈ 0.089 μ` is a small correction and it behaves like
/// one. The all-flat row reading exactly 1.0 is what proves the comparison is
/// wired to the material at all.
#[test]
#[ignore = "four shallow ramps plus the uniform baseline, ~3 min — the \
            parameter attribution behind the stiffness gate's explanation"]
fn the_graded_stiffening_is_carried_by_the_linear_modulus() {
    let mu = [STACK[0].mu, STACK[1].mu, STACK[2].mu];
    let c2 = [STACK[0].c2, STACK[1].c2, STACK[2].c2];
    let lambda = [STACK[0].lambda, STACK[1].lambda, STACK[2].lambda];
    let caps = [
        STACK[0].validity_max_principal_stretch,
        STACK[1].validity_max_principal_stretch,
        STACK[2].validity_max_principal_stretch,
    ];
    let flat = |v: f64| [v; 3];

    let uniform: Vec<f64> = cavity_rungs()
        .iter()
        .cloned()
        .map(|(_, r)| r.expect("the uniform cell must converge").mean_traction)
        .collect();

    let variants = [
        ("baseline", graded_field_with(mu, c2, lambda, caps)),
        (
            "linear (mu, lambda) flat",
            graded_field_with(flat(ECOFLEX_00_30.mu), c2, flat(ECOFLEX_00_30.lambda), caps),
        ),
        (
            "C2 flat",
            graded_field_with(mu, flat(ECOFLEX_00_30.c2), lambda, caps),
        ),
        (
            "all flat",
            graded_field_with(
                flat(ECOFLEX_00_30.mu),
                flat(ECOFLEX_00_30.c2),
                flat(ECOFLEX_00_30.lambda),
                caps,
            ),
        ),
    ];

    let mut deepest = Vec::new();
    for (label, field) in variants {
        let ratios: Vec<f64> = Cell::shell_with(field)
            .ramp(CAVITY, RAMP_STEP, CAVITY_GATE_MAX_W)
            .into_iter()
            .zip(&uniform)
            .map(|((_, r), u)| r.expect("the graded cell must converge").mean_traction / u)
            .collect();
        assert_eq!(
            ratios.len(),
            uniform.len(),
            "{label} did not ramp over the uniform cell's advances",
        );
        let (lo, hi) = (ratios[0], ratios[ratios.len() - 1]);
        eprintln!("  {label:26} {lo:.4} -> {hi:.4}   excess {:.4}", hi - 1.0);
        deepest.push((label, hi));
    }

    let at = |name: &str| {
        deepest
            .iter()
            .find(|(l, _)| *l == name)
            .map(|&(_, v)| v)
            .ok_or_else(|| format!("no variant {name}"))
            .expect("variant")
    };
    let (base, linear_flat, c2_flat, all_flat) = (
        at("baseline"),
        at("linear (mu, lambda) flat"),
        at("C2 flat"),
        at("all flat"),
    );

    assert!(
        (all_flat - 1.0).abs() < 1.0e-9,
        "flattening every parameter leaves the ratio at {all_flat:.6}, not 1.0. \
         The two cells are then the same material, so anything but 1.0 means \
         this comparison is reading something other than the material — and \
         every attribution below is measuring that instead",
    );
    let (from_linear, from_c2) = (base - linear_flat, base - c2_flat);
    eprintln!("  linear contributes {from_linear:.4}, C2 contributes {from_c2:.4}");
    assert!(
        from_linear > 0.0 && from_c2 > 0.0,
        "flattening a term did not reduce the stiffening (linear \
         {from_linear:+.4}, C2 {from_c2:+.4}) — with the stack soft at the bore, \
         removing grading in either term should soften it",
    );
    assert!(
        from_linear > 10.0 * from_c2,
        "the linear modulus contributes {from_linear:.4} against C2's \
         {from_c2:.4}, under the 18.5x measured. The claim is that this \
         stiffening is LINEAR-ELASTIC — a shrinking gap means C2's \
         nonlinearity has become load-bearing at these stretches and the \
         reasoning that treats it as a small correction needs re-deriving",
    );
    assert!(
        linear_flat < 1.10,
        "flattening the linear modulus leaves {linear_flat:.4}, well above the \
         1.0566 measured — most of the stiffening survived a change that \
         should have removed it",
    );
}

/// **Does a cached ramp equal a freshly solved one, field for field?**
///
/// The memoisation let four pre-existing always-on gates stop solving their
/// own ramps and read one shared result. That was justified on "the suite
/// still passes", which is a statement about every gate's BOOLEAN and not
/// about any number it read — the distinction item 3a's refactor was careful
/// about and this one initially was not.
///
/// Re-solves each cached ramp and compares the whole `Debug` rendering of
/// every rung, which is total over `Press`'s fields rather than a selection of
/// them. Two claims at once:
///
/// - **the cache is neutral** — it changes WHEN a ramp runs, not what it
///   produces;
/// - **the solve is deterministic** — if it were not, memoisation would have
///   silently replaced four independent draws with one, and a flake would
///   change character rather than disappear.
///
/// ⚠ Ignored because it deliberately pays the cost the cache exists to avoid.
#[test]
#[ignore = "re-solves all three cached ramps to compare against the cache, \
            ~2 min — the neutrality and determinism evidence for the \
            OnceLock caches"]
fn the_cached_ramps_equal_a_fresh_solve() {
    let cases: [(&str, &[Rung], Vec<Rung>); 3] = [
        (
            "plate",
            plate_rungs(),
            ramp_tet10_yeoh(BASELINE, RAMP_STEP, GATE_MAX_PLANE_H),
        ),
        (
            "cavity",
            cavity_rungs(),
            ramp_cavity(CAVITY, RAMP_STEP, CAVITY_GATE_MAX_W),
        ),
        ("graded", graded_rungs(), ramp_graded(CAVITY_GATE_MAX_W)),
    ];

    for (label, cached, fresh) in cases {
        assert_eq!(
            cached.len(),
            fresh.len(),
            "{label}: the cache holds {} rungs and a fresh solve produced {} — \
             the cached accessor is not asking for what its callers ask for",
            cached.len(),
            fresh.len(),
        );
        assert!(
            !cached.is_empty(),
            "{label}: an empty ramp makes every comparison below vacuous",
        );
        for (i, ((wc, rc), (wf, rf))) in cached.iter().zip(&fresh).enumerate() {
            assert!(
                (wc - wf).abs() < 1.0e-15,
                "{label} rung {i}: cached advance {wc:e} vs fresh {wf:e}",
            );
            assert_eq!(
                format!("{rc:?}"),
                format!("{rf:?}"),
                "{label} rung {i} ({:.3} mm): the cached rung differs from a \
                 freshly solved one. Either the cache is not neutral, or the \
                 solve is not deterministic — and if it is the latter, the \
                 caches replaced four independent draws with one shared draw \
                 in four pre-existing gates",
                wc * 1e3,
            );
        }
        eprintln!(
            "  {label}: {} rungs identical to a fresh solve",
            cached.len()
        );
    }
}
