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
//!    surface — and it, not the element, decides the verdict.** IV-5 applies
//!    `p·A_v·n̂` with `A_v = 4πR²/N_loaded` split equally over the loaded
//!    band. On a P2 face the consistent load is `∫ p·N_i dA`, which puts
//!    **zero** on the corners and `A_f/3` on each midside — the exact
//!    opposite of an equal split. Measured on the ν = 0.49 decision mesh, the
//!    equal-split rule reads Tet10 at `+21%` and the consistent rule reads it
//!    at `+1.4%`: the load rule alone moves the answer across the REJECT
//!    threshold. So this harness applies the consistent boundary-face load
//!    for **both** elements (P1 `A_f/3` per corner for Tet4, P2 for Tet10)
//!    and selects the loaded / pinned bands by **boundary-face membership**,
//!    not by the Tet4-tuned `0.5·cell_size` radial predicate (which sweeps
//!    borderline *radial*-edge midsides — nodes that are not on the cavity
//!    surface at all — into both the load and the readout).
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
//! Timoshenko & Goodier, *Theory of Elasticity* 3rd ed. §141. At ~0.2 %
//! cavity-radius inflation the Neo-Hookean response is in its linear-elastic
//! regime, where it reduces to Lamé.
//!
//! **Measurement convention — same node set on both sides.** The FEM mean is
//! the Saint-Venant average of `‖x_final‖ − ‖x_rest‖` over every
//! cavity-surface node, and the analytic mean evaluates `u_r` at each of
//! *those same nodes'* rest radii. Tet10's midside nodes sit at straight-edge
//! midpoints, ~3 % inside `R_CAVITY`, so comparing them against `u_r(R_a)`
//! would charge the element with a geometric offset that is really the
//! straight-edged surface (isoparametric curvature is ladder rung 8). Reading
//! both sides on the same nodes removes that confound; it is the convention
//! IV-5's own per-shell profile gate already uses.
//!
//! **Surface-area normalisation.** Face weights are scaled so the total
//! tributary area equals the exact analytic cavity area `4πR_a²`. The
//! faceted inscribed surface under-reports area by `O((h/R)²)` — ~3 % at this
//! mesh — and that is a *mesher* error which straight-edged Tet10 does not
//! fix either. Normalising charges it to neither element. The per-node load
//! direction is the exact radial `r̂` at the node's rest position, matching
//! the analytic traction, for the same reason.
//!
//! ## ★ The residual tolerance must scale with the load
//!
//! [`SolverConfig::skeleton`] ships an **absolute** `tol = 1e-10`. At
//! ν = 0.49, `λ = 49 μ` raises the assembled residual's cancellation floor by
//! ~50×, and a first spike of this harness stalled Armijo at iteration 19
//! with `r_norm = 1.107e-10` — the roundoff floor, one part in ten thousand
//! above the fixed tolerance. Read naively that is "Newton stalls at ν → 0.5
//! → REJECT", which is a **false reject on a numerical artifact**, not a
//! locking finding. So every run here uses `tol = TOL_RELATIVE · ‖F_ext‖`
//! with `‖F_ext‖ = p·4πR_a²` — identical for every element and every ν, so
//! no configuration is granted a tolerance the others do not get.
//!
//! ## Release-only, and CI-registered
//!
//! The Tet10 solves are ~3 s each in release and ~30× that in debug, so the
//! file is `#[cfg_attr(debug_assertions, ignore)]`. Release-only sim-soft
//! tests are **hand-registered** in the `--test` list of
//! `.github/workflows/quality-gate.yml` — an unregistered one runs in no CI
//! job at all (as `bonded_layer_indentation` currently does). This file is
//! registered there; the whole suite is a few seconds in release.

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

/// Internal cavity pressure (Pa) — IV-5's `PRESSURE`, chosen to land cavity
/// inflation near 1 % of `R_CAVITY` at ν = 0.4, comfortably small-strain.
const PRESSURE: f64 = 5.0e3;

/// **The decision mesh.** IV-5's `CELL_SIZE_H2`, ~6.5k tets → ~4.7k Tet4
/// vertices / ~13.3k Tet10 nodes (~40k DOF). The h/4 Tet10 mesh is ~226k DOF
/// at GB scale, so it stays a pre-push one-shot and never enters CI; the
/// locking signal is an element property visible at moderate refinement, and
/// Lamé convergence here is already super-quadratic h/2 → h/4.
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
const MAX_NEWTON_ITER: usize = 150;

/// Convergence tolerance as a fraction of the applied load `‖F_ext‖` — see
/// the module docs' tolerance section. `1e-10` of a ~100 N load is `1e-8`,
/// still eight orders below the smallest per-node force and two orders above
/// the ν = 0.49 roundoff floor that stalled the first spike.
const TOL_RELATIVE: f64 = 1e-10;

// ── Committed measurement (6a) ───────────────────────────────────────────

/// **`e₄₀` — the committed Tet4 ν = 0.4 baseline**, `|measured − analytic| /
/// analytic` for the cavity-surface mean at [`CELL_SIZE_DECISION`].
///
/// Measured `0.0721` (Tet4 reads `1.79425e-4` against an analytic
/// `1.93369e-4` — i.e. **over-stiff by 7.2 %**, the expected sign for a
/// constant-strain element). This is the common mesh floor the rung-6c
/// ν = 0.49 threshold subtracts against, per plan §5 step 6a.
///
/// **`e₄₀` is harness-defined, not a property of the mesh.** The shipped
/// `iv_5_uniform_passthrough_at_h2_matches_single_shell_lame` prints `0.1493`
/// for the same physics at the same refinement; the difference is entirely
/// the load rule and the measurement convention this file fixes (module docs
/// §"Why a second Lamé file"). Quoting one number against the other's
/// threshold is a category error — that is exactly why 6a commits a baseline
/// on the harness the decision runs on.
const E40: f64 = 0.0721;

/// Half-width of the acceptance band around a committed measurement. The
/// solves are deterministic (the assembler is serial by design and the
/// mesher is gated for run-to-run bit-equality by IV-5), so this bounds
/// cross-platform floating-point drift, not run-to-run noise — it is a
/// regression detector on the number, roughly ±7 % relative, not a bit-pin.
const COMMITTED_BAND: f64 = 0.005;

// ── Closed-form single-shell Lamé oracle ─────────────────────────────────

/// Radial-displacement coefficients for `u_r(r) = A r + B / r²` in a
/// thick-walled hollow sphere under internal pressure with a fixed outer
/// surface. See the module docs for the two boundary conditions.
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

    /// Evaluate `u_r(r) = A r + B / r²`.
    fn u_r(&self, r: f64) -> f64 {
        self.a.mul_add(r, self.b / (r * r))
    }
}

/// Lamé's first parameter for a target Poisson ratio at fixed `μ`:
/// `λ = 2μν / (1 − 2ν)`. Mirrors `fbar_locking::lambda_from_nu`.
fn lambda_from_nu(nu: f64) -> f64 {
    2.0 * MU * nu / (1.0 - 2.0 * nu)
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

// ── Surface conditions: pinned band, consistent load, readout node set ────

/// Everything the solve and the readout need from the mesh surface.
struct SurfaceConditions {
    /// Rest positions of every node (snapshotted before the mesh moves into
    /// the solver).
    rest: Vec<Vec3>,
    /// Outer-surface nodes, fully Dirichlet-pinned.
    pinned: Vec<VertexId>,
    /// Cavity-surface nodes carrying a non-zero consistent load. For Tet10
    /// this is the midsides only — a P2 corner's `∫N_i dA` is exactly zero.
    loaded: Vec<VertexId>,
    /// Packed per-loaded-vertex force vectors, in `loaded` order.
    theta: Tensor<f64>,
    /// **Every** cavity-surface node, loaded or not — the Saint-Venant
    /// readout set. Deliberately wider than `loaded`: a P2 corner carries no
    /// load but still sits on the cavity wall and still displaces.
    cavity: Vec<VertexId>,
}

/// Build the pinned band, the consistent cavity load, and the readout node
/// set from the mesh's boundary faces.
///
/// A face joins the outer band when all three of its corners lie within a
/// half-cell of `R_OUTER`, and the cavity band when they lie within a
/// half-cell of `R_CAVITY`; the sphere's boundary is exactly these two
/// surfaces, which the caller asserts. Node membership then follows from the
/// face, so a midside is selected only when it genuinely lies *on* a surface
/// face — never because a radial edge happened to put its midpoint inside a
/// per-vertex radial band.
fn surface_conditions(mesh: &dyn Mesh, cell_size: f64) -> SurfaceConditions {
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let band = 0.5 * cell_size;
    let on_shell = |nodes: &[VertexId; 3], radius: f64| {
        nodes
            .iter()
            .all(|&v| (rest[v as usize].norm() - radius).abs() < band)
    };

    let mut pinned: BTreeSet<VertexId> = BTreeSet::new();
    let mut cavity: BTreeSet<VertexId> = BTreeSet::new();
    // Tributary areas ∫N_i dA accumulated per node over the cavity faces.
    let mut tributary: BTreeMap<VertexId, f64> = BTreeMap::new();

    for face in boundary_faces(mesh) {
        if on_shell(&face.corners, LAYERED_SPHERE_R_OUTER) {
            pinned.extend(face.corners);
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
        let area = 0.5 * (p[1] - p[0]).cross(&(p[2] - p[0])).norm();
        cavity.extend(face.corners);
        match face.midsides {
            // P2 consistent load: `∫N_corner dA = 0`, `∫N_midside dA = A/3`.
            Some(midsides) => {
                cavity.extend(midsides);
                for v in midsides {
                    *tributary.entry(v).or_insert(0.0) += area / 3.0;
                }
            }
            // P1 consistent load: `∫N_corner dA = A/3`.
            None => {
                for v in face.corners {
                    *tributary.entry(v).or_insert(0.0) += area / 3.0;
                }
            }
        }
    }

    // Normalise the faceted tributary total to the exact analytic cavity
    // area, so the mesher's O((h/R)²) surface-faceting deficit is charged to
    // neither element (module docs, "Surface-area normalisation").
    let exact_area = 4.0 * std::f64::consts::PI * LAYERED_SPHERE_R_CAVITY * LAYERED_SPHERE_R_CAVITY;
    let faceted_area: f64 = tributary.values().sum();
    let area_scale = exact_area / faceted_area;

    let loaded: Vec<VertexId> = tributary.keys().copied().collect();
    let mut theta_data = Vec::with_capacity(3 * loaded.len());
    for &v in &loaded {
        let p = rest[v as usize];
        // Exact radial direction at the node's rest position — the analytic
        // traction direction. `‖p‖ > 0` on any cavity-band node.
        let n_hat = p / p.norm();
        let force = PRESSURE * tributary[&v] * area_scale;
        theta_data.extend_from_slice(&[force * n_hat.x, force * n_hat.y, force * n_hat.z]);
    }

    SurfaceConditions {
        theta: Tensor::from_slice(&theta_data, &[3 * loaded.len()]),
        rest,
        pinned: pinned.into_iter().collect(),
        loaded,
        cavity: cavity.into_iter().collect(),
    }
}

// ── The harness ──────────────────────────────────────────────────────────

/// Which element the harness solves with. Both run identical geometry,
/// material, load rule, tolerance, and readout — only the element order (and
/// hence the mesh's node set) differs, so any gap between two readings is an
/// element-order gap.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ElementOrder {
    Tet4,
    // Rung 6a lands the harness carrying only the Tet4 baseline. `Tet10` is
    // constructed by the 6b match-or-beat anchor and the 6c decision gate,
    // which land in their own PRs on top of this one — the variant exists
    // here so the harness's element-agnostic shape is reviewable as a whole.
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
    n_cavity: usize,
}

/// Build the uniform sphere at `cell_size`, solve one static step with
/// `order` at Poisson ratio `nu`, and read the cavity-surface mean against
/// the closed-form Lamé field on the same nodes.
///
/// Returns the solver's own failure rather than panicking on it: a
/// near-incompressible solve that stalls must land as a clean REJECT verdict
/// in 6c, not as a test crash (plan §5 step 6c).
fn solve_and_read(
    order: ElementOrder,
    nu: f64,
    cell_size: f64,
) -> Result<Reading, Box<SolverFailure>> {
    let lambda = lambda_from_nu(nu);
    // Only the mesh is taken from the scene helper — the harness builds its
    // own boundary conditions and load, because the helper's are Tet4-tuned
    // (per-vertex radial bands, equal-split load; module docs §3).
    let (mesh4, _bc, _initial, _theta) =
        SoftScene::layered_silicone_sphere(MaterialField::uniform(MU, lambda), cell_size, PRESSURE)
            .expect("layered_silicone_sphere should mesh at the canonical cell sizes");
    let mesh10 = Tet10Mesh::from_tet4(&mesh4);

    let surface = match order {
        ElementOrder::Tet4 => surface_conditions(&mesh4, cell_size),
        ElementOrder::Tet10 => surface_conditions(&mesh10, cell_size),
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
    cfg.tol = TOL_RELATIVE
        * PRESSURE
        * 4.0
        * std::f64::consts::PI
        * LAYERED_SPHERE_R_CAVITY
        * LAYERED_SPHERE_R_CAVITY;

    let step = match order {
        ElementOrder::Tet4 => {
            let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
                CpuNewtonSolver::new(Tet4, mesh4, NullContact, cfg, bc);
            solver.try_replay_step(&x_prev, &v_prev, &surface.theta, cfg.dt)
        }
        ElementOrder::Tet10 => {
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, mesh10, NullContact, cfg, bc);
            solver.try_replay_step(&x_prev, &v_prev, &surface.theta, cfg.dt)
        }
    }
    .map_err(Box::new)?;

    let lame = Lame::solve(
        MU,
        lambda,
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_OUTER,
        PRESSURE,
    );
    let mut measured_sum = 0.0;
    let mut analytic_sum = 0.0;
    for &v in &surface.cavity {
        let i = v as usize;
        let moved = Vec3::new(
            step.x_final[3 * i],
            step.x_final[3 * i + 1],
            step.x_final[3 * i + 2],
        );
        let rest_radius = surface.rest[i].norm();
        measured_sum += moved.norm() - rest_radius;
        analytic_sum += lame.u_r(rest_radius);
    }
    let n = surface.cavity.len() as f64;
    let measured = measured_sum / n;
    let analytic = analytic_sum / n;

    Ok(Reading {
        rel_err: (measured - analytic) / analytic,
        measured,
        analytic,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_nodes: surface.rest.len(),
        n_loaded: surface.loaded.len(),
        n_cavity: surface.cavity.len(),
    })
}

/// Report a reading and assert the checks every configuration must pass
/// regardless of its accuracy verdict: Newton converged with budget to
/// spare, the cavity inflated outward, and the strain stayed inside the
/// linear-elastic window the Lamé oracle is valid in.
fn report_and_check_physical(label: &str, reading: &Reading) {
    eprintln!(
        "rung-6 {label}: rel_err = {rel:+.4}, measured = {measured:e}, analytic = {analytic:e}, \
         nodes = {nodes}, loaded = {loaded}, cavity = {cavity}, newton iters = {iters}, \
         residual = {residual:e}",
        rel = reading.rel_err,
        measured = reading.measured,
        analytic = reading.analytic,
        nodes = reading.n_nodes,
        loaded = reading.n_loaded,
        cavity = reading.n_cavity,
        iters = reading.iter_count,
        residual = reading.residual_norm,
    );
    assert!(
        reading.iter_count < MAX_NEWTON_ITER,
        "{label}: Newton consumed the full {MAX_NEWTON_ITER}-iteration budget — the reading \
         below is not a converged solution"
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

// ── 6a — the committed Tet4 ν = 0.4 baseline ─────────────────────────────

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — the rung-6 decision harness solves the h/2 silicone sphere \
              (seconds in release, ~30× that in debug); rerun with `cargo test --release`"
)]
#[test]
fn tet4_baseline_at_nu_0_4_is_committed_e40() {
    // Ladder rung 6a. Nails down `e₄₀`, the common mesh floor the ν = 0.49
    // decision threshold subtracts against — measured on the harness the
    // decision itself runs on, at the mesh the decision itself runs at.
    // Before this commit the plan quoted an uncommitted "0.993" that exists
    // nowhere in the tree.
    let reading = solve_and_read(ElementOrder::Tet4, NU_BASELINE, CELL_SIZE_DECISION)
        .expect("the ν = 0.4 Tet4 baseline solve converges in ~3 Newton iterations");
    report_and_check_physical("6a Tet4 ν=0.4 @ h/2", &reading);

    assert!(
        reading.rel_err < 0.0,
        "e₄₀ should be an OVER-stiff (negative) error — a constant-strain element under-predicts \
         the Lamé inflation. Got {rel:+.4}; a sign flip means the baseline is no longer \
         measuring what the plan's 'common mesh floor' framing assumes",
        rel = reading.rel_err,
    );
    let e40 = reading.rel_err.abs();
    assert!(
        (e40 - E40).abs() < COMMITTED_BAND,
        "committed e₄₀ = {E40:.4} but measured {e40:.4} (band ±{COMMITTED_BAND}). This is the \
         baseline every rung-6 threshold subtracts against — do NOT re-bake the constant to make \
         the test green without first establishing WHY the discretisation error moved (mesher, \
         load rule, measurement convention, or element)."
    );
}
