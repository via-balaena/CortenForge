//! Renovation item 3, step 0 — does `Tet10Mesh<Yeoh>` converge under the IPC
//! face barrier at all?
//!
//! Item 1 (`6dc7ee03`) made `Tet10Mesh<M>` generic, so `Tet10Mesh<Yeoh>` is
//! **constructible**. Constructible is not converged: when this file was
//! written the *only* Tet10 + IPC solver instantiation in the tree was
//! `tests/tet10_indentation_demand1.rs`, and its material was `NeoHookean`.
//! (Stated in the past tense on purpose — this file adds three more, two of
//! them Yeoh, so a present-tense version of that sentence would be falsified
//! by the change that made it.) Before item 3 rewires the 6 416-line
//! `tools/cf-sim-research/src/insertion_sim.rs`, this file answers the cheap
//! question on a fixture that runs in seconds: does the triple (Tet10 element
//! × Yeoh material × IPC face barrier) solve, and at what Newton cost relative
//! to the two baselines that already work — Tet4 × Yeoh, and Tet10 ×
//! `NeoHookean`?
//!
//! ## The fixture
//!
//! A 40 × 40 × 12 mm plate of Ecoflex 00-30, SDF-meshed through the same
//! `SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh` path `insertion_sim` uses, then
//! enriched to Tet10 and pressed onto a rigid plane held inside the barrier
//! band. Top face pinned, one quasi-static step.
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
//! - **The per-vertex barrier contacts vertices that are in no tetrahedron.**
//!   `SdfMeshedTetMesh::positions()` is the BCC lattice, not the body: 1 244 of
//!   its 2 348 entries are referenced by no tet. `active_vertex_pairs` iterates
//!   `positions()`, so 386 of the Tet4 arm's 607 contact pairs land on dead
//!   nodes — 338 of which sit below the plane in the *rest* configuration — and
//!   the barrier's `d = sd.max(d̂ · 1e-6)` clamp reports ~4e9 N for them. The
//!   face path is immune: `boundary_faces6()` is built from tet connectivity.
//!   ⚠ `PenaltyRigidContact::active_pairs` has the same all-positions loop and
//!   `insertion_sim` uses it, so the mechanism reaches the shipped sim; the
//!   exposure there is unmeasured.
//!
//! ⚠ **Two claims retracted from an earlier revision of this file**, recorded
//! because the corrections are the useful part.
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
//! ## What this cannot see
//!
//! - **A closing cavity.** The curved cell is a convex indenter pressed into a
//!   plate. `insertion_sim` is a compliant cavity closing *around* a probe —
//!   conforming, enveloping contact rather than a Hertzian patch. Curvature is
//!   now covered; enveloping geometry is not.
//! - **Graded materials.** One anchor everywhere. `insertion_sim` carries a
//!   layered per-tet Yeoh field, and the material-validity wall row 23 hit was
//!   a per-tet event at one tet.
//! - **A derived `κ`.** 1e7 is the smallest decade in a sweep that worked, not
//!   a quantity anyone computed from the area-weighted face barrier. The
//!   bracket is 1e6 (stalls) to 1e8 (holds contact 68 % of the band open).
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

use nalgebra::Point3;
use sim_ml_chassis::Tensor;
use sim_soft::element::{Tet4, Tet10};
use sim_soft::material::silicone_table::ECOFLEX_00_30;
use sim_soft::{
    Aabb3, ActivePairsFor, BoundaryConditions, ConstantField, ContactPair, CpuNewtonSolver,
    IpcRigidContact, MaterialField, Mesh, MeshingHints, NeoHookean, RigidPlane, Sdf,
    SdfMeshedTetMesh, Solver, SolverConfig, SolverFailure, SphereSdf, Tet10Mesh, TetId,
    TranslatedSdf, Vec3, VertexId, Yeoh, peak_contact_pressure, referenced_vertices,
};

// ── fixture geometry ────────────────────────────────────────────────

/// Plate half-extents (m): 40 × 40 × 12 mm. The 12 mm thickness is the
/// sleeve-wall scale the renovation targets (NH walls at ~7 mm, the sleeve
/// needs 8 mm), not the 100 mm block the rung-8b tests use.
const HALF: [f64; 3] = [0.020, 0.020, 0.006];

/// BCC-stuffing cell size (m). Coarse on purpose — this fixture is a
/// convergence probe, not a resolution study.
const CELL: f64 = 0.004;

/// Barrier band (m). Scaled to the plate: the rung-8b tests use `d̂ = 0.01` on a
/// 100 mm block, i.e. `d̂ / thickness = 0.1`; 1.2 mm on a 12 mm plate is the
/// same ratio.
const D_HAT: f64 = 0.0012;

/// The rung-8b face-contact tests' barrier stiffness, carried over unchanged.
///
/// ⛔ **Measured insufficient for this fixture** — see
/// [`the_armijo_wall_against_barrier_stiffness`]. At this value the barrier
/// cannot hold the plate far enough ahead of the advancing plane to keep the
/// next increment feasible, and the ramp stalls at 0.83 % compression. Kept as
/// the counterexample, not used by any gate.
const RUNG_8B_KAPPA: f64 = 1.0e4;

/// Barrier stiffness every gate here runs at.
///
/// ⚠ **An empirical floor, not a derivation.** It is the smallest decade in
/// the `κ` sweep at which the compression ramp runs clean to
/// `RAMP_MAX_PLANE_H` (44 % engineering compression): 1e6 still stalls, 1e7
/// and 1e8 do not. 1e8 is not chosen because its standoff is 0.81 mm — 68 % of
/// the barrier band — i.e. it holds contact open rather than enforcing it,
/// where 1e7 sits at 30 %. Deriving `κ` for an area-weighted face barrier
/// rather than bracketing it is still owed.
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
}

impl Indenter {
    /// The primitive with its contact surface — the plane, or the sphere's
    /// apex — at height `h`, so the two shapes are positioned comparably.
    fn at(self, h: f64) -> Box<dyn Sdf> {
        match self {
            Self::Plane => Box::new(ground_at(h)),
            // Centre a radius below the apex: `sd(0,0,h) = |h − (h−R)| − R = 0`.
            Self::Sphere => Box::new(TranslatedSdf {
                inner: SphereSdf { radius: SPHERE_R },
                offset: Vec3::new(0.0, 0.0, h - SPHERE_R),
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
#[derive(Debug)]
struct Press {
    iters: usize,
    residual: f64,
    /// Largest nodal displacement from rest (m).
    max_disp: f64,
    /// Net contact force on the solid along `+z` (N) at the converged pose.
    /// The plane is below with normal `+z`, so a loaded step is positive.
    net_force_z: f64,
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
    let tet4 = tet4_yeoh();
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    assert!(!pins.is_empty(), "the pinned top face must not be empty");
    let rest = rest_dofs(&mesh);
    let (x, v, theta) = step_inputs(&rest);
    let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
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
fn ramp_tet10_yeoh(setup: Setup, step: f64, max_plane_h: f64) -> Vec<(f64, Result<Press, String>)> {
    let tet4 = tet4_yeoh();
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4);
    let pins = top_face_pins(&mesh);
    let rest = rest_dofs(&mesh);
    let n_dof = rest.len();
    let zero_v = Tensor::from_slice(&vec![0.0; n_dof], &[n_dof]);
    let theta = Tensor::from_slice(&[], &[0]);

    let mut x_prev = rest.clone();
    let mut out = Vec::new();
    // Indexed, not accumulated: `h += step` drifts (the first cut of this loop
    // printed a +0.020000000000000025 mm plane height) and a float loop
    // condition is a lint besides.
    let n_steps = ((max_plane_h - REST_PLANE_H) / step).ceil() as usize;
    for i in 0..=n_steps {
        let h = REST_PLANE_H + (i as f64) * step;
        let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
            CpuNewtonSolver::new(
                Tet10,
                mesh.clone(),
                setup.against(h),
                config(),
                BoundaryConditions::new(pins.clone(), Vec::new()),
            );
        match solver.try_replay_step(
            &Tensor::from_slice(&x_prev, &[n_dof]),
            &zero_v,
            &theta,
            STATIC_DT,
        ) {
            Ok(step) => {
                let p = summarize(
                    &mesh,
                    setup,
                    h,
                    &rest,
                    &step.x_final,
                    step.iter_count,
                    step.final_residual_norm,
                );
                x_prev = step.x_final;
                out.push((h, Ok(p)));
            }
            Err(e) => {
                out.push((h, Err(failure_label(&e))));
                break;
            }
        }
    }
    out
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

    let rungs = ramp_tet10_yeoh(BASELINE, RAMP_STEP, GATE_MAX_PLANE_H);
    for (h, r) in &rungs {
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
/// this fixture is well-posed. They DO reach `active_pairs`, which walks
/// `positions()` with no incidence check of its own. ⇒ the gap is specifically
/// in the contact models, not in the solver.
///
/// ⚠ **This reaches the shipped sim.** `PenaltyRigidContact::active_pairs`
/// takes `_mesh` — it ignores the mesh entirely and loops the same
/// `positions()` — and `insertion_sim` is `SdfMeshedTetMesh<Yeoh>` + penalty
/// contact. Whether its primitives actually sit near dead lattice nodes is NOT
/// measured here and is not claimed; the mechanism is present, the exposure is
/// unquantified.
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
