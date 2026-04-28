//! `SoftScene` — skeleton scene constructors + scene-config bundles.
//!
//! `one_tet_cube()` returns the canonical decimeter-edge tet per spec
//! §2 bundled with its boundary conditions and initial state. The
//! 3-tuple shape `(impl Mesh, BoundaryConditions, SceneInitial)` is
//! Phase 2's canonical scene-emission contract — Decision K + L of
//! [`phase_2_multi_element_fem_scope.md`](../../../../docs/todo/phase_2_multi_element_fem_scope.md).
//! Multi-tet scenes (`HandBuiltTetMesh::two_isolated_tets`,
//! `two_tet_shared_face`) land in Phase 2 commit 2.

use std::collections::BTreeSet;

use sim_ml_chassis::Tensor;

use crate::Vec3;
use crate::material::MaterialField;
use crate::mesh::{Mesh, SingleTetMesh, VertexId, referenced_vertices};
use crate::sdf_bridge::{
    Aabb3, DifferenceSdf, MeshingError, MeshingHints, SdfMeshedTetMesh, SphereSdf,
};

/// Cavity radius of the layered silicone sphere — inner surface of the
/// hollow shell body, where internal pressure is applied.
pub const LAYERED_SPHERE_R_CAVITY: f64 = 0.04;

/// Boundary radius between the inner Ecoflex shell and the carbon-black
/// composite middle shell.
pub const LAYERED_SPHERE_R_INNER_OUTER: f64 = 0.06;

/// Boundary radius between the carbon-black composite middle shell and
/// the outer Ecoflex shell.
pub const LAYERED_SPHERE_R_OUTER_INNER: f64 = 0.08;

/// Outer radius of the layered silicone sphere — outer surface of the
/// hollow shell body, where Dirichlet boundary conditions (`u_r = 0`)
/// pin the rigid-body modes for the IV-5 internal-pressure scene.
pub const LAYERED_SPHERE_R_OUTER: f64 = 0.10;

/// Bounding-box half-extent for the layered silicone sphere mesher hint.
///
/// Wraps `LAYERED_SPHERE_R_OUTER` with a 0.02 m margin (matches the canonical
/// Phase 3 / IV-4 `bbox_half_extent / cell_size = 6.0` ratio at h/2 = 0.02).
pub const LAYERED_SPHERE_BBOX_HALF_EXTENT: f64 = 0.12;

/// Scene constructors. Skeleton ships one constructor for the 1-tet
/// cube; multi-tet siblings (`n_isolated_tets`, `two_tet_shared_face`)
/// land in Phase 2 commit 2 as new methods on the same `impl` block.
pub struct SoftScene;

impl SoftScene {
    /// Canonical 1-tet scene per walking-skeleton spec §2: decimeter edge
    /// (`L = 0.1` m), silicone-class density (`ρ = 1030` kg/m³ — carried
    /// on `SolverConfig`, not here), four vertices at the canonical
    /// right-handed axis-aligned placement.
    ///
    /// Returns the mesh bundled with its boundary conditions
    /// (`v_0..v_2` Dirichlet-pinned, `v_3` `+ẑ`-loaded — the Stage-1
    /// default per Decision L; Stage-2 tests build their own
    /// [`BoundaryConditions`] inline) and its initial state
    /// (rest-configuration positions, zero velocity, flattened to
    /// length-12 `Tensor<f64>` in vertex-major + xyz-inner layout —
    /// DOF `i` is vertex `i / 3`'s `i % 3` component).
    ///
    /// Materials default to a uniform Ecoflex-class field
    /// `(μ, λ) = (1e5, 4e5)` per `material/neo_hookean.rs`'s
    /// skeleton-scene parameters — the Phase 4 IV-1 regression target
    /// per scope memo Decision P. Phase 4+ scenes that exercise graded
    /// fields construct their own `MaterialField` and call the new
    /// mesh constructors directly rather than going through `SoftScene`.
    #[must_use]
    pub fn one_tet_cube() -> (SingleTetMesh, BoundaryConditions, SceneInitial) {
        let mesh = SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5));
        let mut x_prev_flat = [0.0f64; 12];
        for (v, pos) in mesh.positions().iter().enumerate() {
            x_prev_flat[3 * v] = pos.x;
            x_prev_flat[3 * v + 1] = pos.y;
            x_prev_flat[3 * v + 2] = pos.z;
        }
        let bc = BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        };
        let initial = SceneInitial {
            x_prev: Tensor::from_slice(&x_prev_flat, &[12]),
            v_prev: Tensor::zeros(&[12]),
        };
        (mesh, bc, initial)
    }

    /// Three-shell concentric hollow silicone sphere meshed by
    /// [`SdfMeshedTetMesh`] with internal pressure on the cavity surface
    /// and Dirichlet pinning on the outer surface — the canonical IV-5
    /// scene per Phase 4 scope memo §1 IV-5 + §8 commit 11.
    ///
    /// # Geometry
    ///
    /// Hollow shell `SphereSdf{LAYERED_SPHERE_R_OUTER} \
    /// SphereSdf{LAYERED_SPHERE_R_CAVITY}` composed via
    /// [`DifferenceSdf`] (book Part 7 §00 §01 sharp-CSG difference
    /// operator). Three concentric layers, partitioned by
    /// `material_field`:
    ///
    /// - **Inner shell** — radii `[R_CAVITY, R_INNER_OUTER]`
    /// - **Middle shell** — radii `[R_INNER_OUTER, R_OUTER_INNER]`
    /// - **Outer shell** — radii `[R_OUTER_INNER, R_OUTER]`
    ///
    /// Layer Lamé parameters are encoded by the caller in
    /// `material_field`; for the canonical Decision J device the layers
    /// are Ecoflex 00-30 / Ecoflex 00-30 + 15 wt% carbon-black /
    /// Ecoflex 00-30 (Part 1 §04 §00 + §04 §02). Per IV-3 (commit 8)
    /// precedent the canonical IV-5 test runs in the compressible
    /// Neo-Hookean regime (`λ = 4 μ` ⇒ `ν = 0.4`); near-incompressible
    /// Ecoflex (`ν ≈ 0.49`) is recovered with Tet10 + F-bar at Phase H.
    ///
    /// # Boundary conditions
    ///
    /// The outer surface is fully pinned (Dirichlet) and the cavity
    /// surface receives a radially outward per-vertex force traction
    /// computed from `pressure`.
    ///
    /// On the outer surface every vertex within `0.5 × cell_size`
    /// radial distance of `R_OUTER` (BCC mesher cut-points plus
    /// warp-snapped lattice vertices) is pinned at its rest position.
    /// This kills all six rigid-body modes (translation plus rotation)
    /// by construction with perfect spherical symmetry, and the
    /// closed-form Lamé multi-shell solution generalises cleanly to
    /// the fixed-outer-surface variant via the `u_r(R_b) = 0` boundary
    /// condition (vs. the standard free-outer `σ_rr(R_b) = 0`).
    ///
    /// On the cavity surface every vertex within `0.5 × cell_size`
    /// radial distance of `R_CAVITY` carries a per-vertex force
    /// `f_v = pressure · n̂_v · A_v`, where `n̂_v = p_v / |p_v|` is the
    /// radially outward unit vector at v and `A_v = 4π · R_CAVITY² /
    /// N_loaded` is the uniform-distribution Saint-Venant tributary
    /// area. The positive sign drives the cavity wall outward — the
    /// physical signature of internal pressure pushing on a soft
    /// container. Tractions thread through [`LoadAxis::FullVector`].
    ///
    /// # Returns
    ///
    /// Tuple of `(mesh, bc, initial, theta)`:
    ///
    /// - `mesh: SdfMeshedTetMesh` — body meshed via the BCC + Labelle-
    ///   Shewchuk pipeline at `cell_size`, with per-tet materials
    ///   centroid-sampled from `material_field`.
    /// - `bc: BoundaryConditions` — `pinned_vertices` is the
    ///   outer-surface band; `loaded_vertices` is the cavity-surface
    ///   band, every entry paired with `LoadAxis::FullVector`.
    /// - `initial: SceneInitial` — rest positions flattened to length
    ///   `3 · n_vertices`, zero velocity.
    /// - `theta: Tensor<f64>` — per-vertex force triples in
    ///   `loaded_vertices` order, length `3 · N_loaded`. Pass directly
    ///   to [`crate::Solver::replay_step`].
    ///
    /// # Errors
    ///
    /// Forwards [`SdfMeshedTetMesh::from_sdf`]'s error variants:
    /// [`MeshingError::EmptyMesh`] when no tets emit (typically a
    /// `cell_size` too coarse to span the shell band), or
    /// [`MeshingError::NonFiniteSdfValue`] (impossible for the finite-
    /// radii [`DifferenceSdf`] of two [`SphereSdf`]s but propagated for
    /// the general callsite).
    ///
    /// # Panics
    ///
    /// Panics with a structured message if either the outer-surface or
    /// cavity-surface band turns up empty at the supplied `cell_size`
    /// (degenerate geometry — caller picked a cell size too coarse to
    /// resolve the surface). Panics on bbox/cell-size invariants
    /// forwarded from the BCC lattice constructor (non-positive
    /// `cell_size`, ill-formed `bbox`, or a `bbox` degenerate enough to
    /// yield zero cubes along some axis).
    #[must_use = "ignoring the returned scene drops the meshed body and the per-vertex theta tensor"]
    pub fn layered_silicone_sphere(
        material_field: MaterialField,
        cell_size: f64,
        pressure: f64,
    ) -> Result<
        (
            SdfMeshedTetMesh,
            BoundaryConditions,
            SceneInitial,
            Tensor<f64>,
        ),
        MeshingError,
    > {
        let body_sdf = DifferenceSdf::new(
            Box::new(SphereSdf {
                radius: LAYERED_SPHERE_R_OUTER,
            }),
            Box::new(SphereSdf {
                radius: LAYERED_SPHERE_R_CAVITY,
            }),
        );
        let half_extent = LAYERED_SPHERE_BBOX_HALF_EXTENT;
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(-half_extent, -half_extent, -half_extent),
                Vec3::new(half_extent, half_extent, half_extent),
            ),
            cell_size,
            material_field: Some(material_field),
        };
        let mesh = SdfMeshedTetMesh::from_sdf(&body_sdf, &hints)?;

        // Surface-band predicates: half-cell radial tolerance bracket
        // the BCC mesher's cut-points (which lie on the linear-interp
        // secant zero, ≤ O(cell_size²) off the analytic surface) plus
        // any warp-snapped lattice vertices the stuffing pass moved
        // onto the surface.
        //
        // Filter against `referenced_vertices` to drop BCC lattice
        // orphans (lattice corners whose containing BCC tets fell
        // entirely outside the body and got `n_inside == 0`-cased away
        // — they retain a `positions()` slot but no tet references
        // them, so loading them would trip the solver's "load on
        // orphan vertex" panic at backward_euler.rs:274).
        // `referenced_vertices` returns a sorted Vec; collect into
        // BTreeSet for O(log n) `contains` lookup with deterministic
        // iteration order.
        let referenced: BTreeSet<VertexId> = referenced_vertices(&mesh).into_iter().collect();
        let band_tol = 0.5 * cell_size;
        let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| {
            (p.norm() - LAYERED_SPHERE_R_OUTER).abs() < band_tol
        })
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
        let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| {
            (p.norm() - LAYERED_SPHERE_R_CAVITY).abs() < band_tol
        })
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
        let r_outer = LAYERED_SPHERE_R_OUTER;
        let r_cavity = LAYERED_SPHERE_R_CAVITY;
        assert!(
            !pinned.is_empty(),
            "layered_silicone_sphere: outer-surface band turned up empty at cell_size = \
             {cell_size}; half-cell tolerance {band_tol} is below the mesher's cut-point density \
             on R_OUTER = {r_outer}",
        );
        assert!(
            !loaded.is_empty(),
            "layered_silicone_sphere: cavity-surface band turned up empty at cell_size = \
             {cell_size}; half-cell tolerance {band_tol} is below the mesher's cut-point density \
             on R_CAVITY = {r_cavity}",
        );

        // Initial state — rest positions flattened, zero velocity.
        let positions = mesh.positions();
        let n_dof = 3 * positions.len();
        let mut x_prev_flat = vec![0.0; n_dof];
        for (v, pos) in positions.iter().enumerate() {
            x_prev_flat[3 * v] = pos.x;
            x_prev_flat[3 * v + 1] = pos.y;
            x_prev_flat[3 * v + 2] = pos.z;
        }
        let initial = SceneInitial {
            x_prev: Tensor::from_slice(&x_prev_flat, &[n_dof]),
            v_prev: Tensor::zeros(&[n_dof]),
        };

        // Per-vertex tributary area under uniform Saint-Venant
        // distribution: total cavity surface area divided over every
        // loaded vertex. The pressure-times-area-times-normal force at
        // each loaded vertex sums (vectorially) to zero for a closed
        // sphere by symmetry, but its radial component sums to the
        // total internal-pressure thrust 4π R_CAVITY² · p — the analytic
        // load magnitude.
        //
        // `loaded.len() as f64` is an exact f64 representation for any
        // mesh size sim-soft hits; the cast is loss-free.
        #[allow(clippy::cast_precision_loss)]
        let area_per_vertex =
            4.0 * std::f64::consts::PI * LAYERED_SPHERE_R_CAVITY * LAYERED_SPHERE_R_CAVITY
                / loaded.len() as f64;
        let mut theta_data = vec![0.0; 3 * loaded.len()];
        for (i, &v) in loaded.iter().enumerate() {
            let p = positions[v as usize];
            let radius = p.norm();
            // Radially outward unit vector at v's rest position.
            // `radius > 0` is guaranteed by the cavity-band predicate
            // (R_CAVITY = 0.04 ± half-cell, so the smallest plausible
            // radius is > 0.03 at the coarsest cell_size = 0.04).
            let n_hat = p / radius;
            let force = pressure * area_per_vertex;
            theta_data[3 * i] = force * n_hat.x;
            theta_data[3 * i + 1] = force * n_hat.y;
            theta_data[3 * i + 2] = force * n_hat.z;
        }
        let theta = Tensor::from_slice(&theta_data, &[3 * loaded.len()]);

        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
        };

        Ok((mesh, bc, initial, theta))
    }
}

/// Boundary conditions for a soft-body scene.
///
/// Carries the Dirichlet pinned-vertex set (those whose displacement is
/// fixed at the rest configuration) plus the load-application list
/// (vertices that receive external traction, paired with the load axis
/// describing which θ component drives which DOF).
///
/// `CpuNewtonSolver::new` consumes this at construction time —
/// validates pinned/loaded vertex IDs and the no-overlap contract,
/// then derives the cache (free-DOF index map, lumped per-DOF mass,
/// sparse pattern) from `pinned_vertices`; the assembly path reads
/// `loaded_vertices` per Newton iter via `assemble_external_force`.
#[derive(Clone, Debug)]
pub struct BoundaryConditions {
    /// Vertex IDs whose displacement is pinned to their rest position
    /// (full Dirichlet — all three xyz DOFs constrained).
    pub pinned_vertices: Vec<VertexId>,
    /// Vertex IDs that receive external traction, paired with the load
    /// axis describing which θ component drives which DOF.
    pub loaded_vertices: Vec<(VertexId, LoadAxis)>,
}

/// How a θ component maps to a vertex's DOFs.
///
/// Stage 1 (`+ẑ` magnitude) pairs each loaded vertex with one θ scalar
/// driving its `z` DOF; Stage 2 (full traction vector) pairs each
/// loaded vertex with three θ components driving its xyz. Mixed-axis
/// scenes are out of scope for Phase 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadAxis {
    /// Single `+ẑ` traction component — Stage-1 θ broadcasts this
    /// magnitude to the loaded vertex's `z` DOF.
    AxisZ,
    /// Full `(t_x, t_y, t_z)` traction vector — Stage-2 θ supplies all
    /// three components to the loaded vertex's xyz DOFs.
    FullVector,
}

/// Initial state for a forward-pass rollout. Bootstrap position and
/// velocity the first Newton step consumes.
#[derive(Debug)]
pub struct SceneInitial {
    /// Rest-configuration vertex positions, flattened to shape `[3 * N]`.
    pub x_prev: Tensor<f64>,
    /// Rest-configuration vertex velocities, flattened to shape `[3 * N]`.
    pub v_prev: Tensor<f64>,
}

/// Collect every vertex whose rest-configuration position satisfies
/// `predicate`, ascending.
///
/// Walks `mesh.positions()` in `VertexId` order and returns the
/// matching indices. Caller plugs the result into
/// [`BoundaryConditions::pinned_vertices`] (or composes it with
/// [`mesh::referenced_vertices`](crate::mesh::referenced_vertices) to
/// pick a load vertex from a spatial predicate over a mesher-generated
/// mesh whose vertex IDs aren't pinned by hand).
///
/// Free function rather than a method on [`BoundaryConditions`] or the
/// [`Mesh`] trait per scope memo §3 Decision K — keeps both surfaces
/// schema-stable and the helper composable across pinned / loaded use
/// cases.
//
// `as VertexId` is the Mesh-trait API tax: enumerate yields `usize`
// while `pinned_vertices` takes `Vec<VertexId = u32>`. Phase 3 meshes
// stay well below `u32::MAX`.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn pick_vertices_by_predicate(
    mesh: &dyn Mesh,
    predicate: impl Fn(&Vec3) -> bool,
) -> Vec<VertexId> {
    mesh.positions()
        .iter()
        .enumerate()
        .filter_map(|(idx, p)| predicate(p).then_some(idx as VertexId))
        .collect()
}
