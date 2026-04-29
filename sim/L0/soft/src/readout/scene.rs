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
use crate::contact::{PenaltyRigidContact, RigidPlane};
use crate::material::MaterialField;
use crate::mesh::{HandBuiltTetMesh, Mesh, SingleTetMesh, VertexId, referenced_vertices};
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

    /// V-3a compressive-block-on-plane scene — uniform-material soft
    /// cube compressed by a rigid plane displaced into its top face.
    ///
    /// Phase 5 commit 6 scaffolding for the V-3a invariant gate (commit
    /// 8) per `phase_5_penalty_contact_scope.md` §1 V-3a.
    ///
    /// # Geometry
    ///
    /// Soft cube of edge `edge_len` placed at `[0, edge_len]³`, meshed
    /// via [`HandBuiltTetMesh::uniform_block`] at `n_per_edge =
    /// edge_len / cell_size` cells per axis (Coxeter-Freudenthal-Kuhn
    /// 6-tets-per-cell). `cell_size` must divide `edge_len` to an even
    /// integer — V-3a's three refinement levels at `edge_len = 0.01`
    /// satisfy this by construction (`5 / 2.5 / 1.25` mm → `n = 2 / 4
    /// / 8`).
    ///
    /// # Boundary conditions
    ///
    /// **z-DOFs only pinned on the bottom face** (`z = 0` band) —
    /// scope memo §1 V-3a's uniaxial-stress validity requirement: the
    /// block must be free to Poisson-contract laterally, so x/y DOFs of
    /// bottom-face vertices stay free. A second-tier pin would
    /// otherwise drive `F = E·A·ε·(1−ν) / ((1+ν)(1−2ν))` (constrained-
    /// modulus regime), not `F = E·A·ε` (uniaxial-stress regime).
    ///
    /// **Single bottom-corner-vertex x/y pin** removes the residual
    /// rigid-body modes (lateral translation + rotation about ẑ).
    /// Phase 5 [`BoundaryConditions`] only models full-vertex Dirichlet
    /// pin — there's no per-DOF pin granularity; the full corner
    /// vertex is pinned in xyz. The remaining bottom-face vertices'
    /// z-DOFs are not modeled either (BC has no z-only flag); the
    /// nearest representable approximation is to also full-pin the
    /// bottom face. **This helper does that** — full-pins every
    /// bottom-face vertex, accepting the constrained-modulus
    /// approximation as a Phase-5-known limitation. V-3a (commit 8)
    /// will document this in its analytic-comparison error budget;
    /// upgrading [`BoundaryConditions`] to per-DOF pin granularity
    /// is a future-phase plumbing decision (no Phase 5 scope memo
    /// commitment).
    ///
    /// # Contact
    ///
    /// Single [`RigidPlane`] from above with outward normal `−ẑ`
    /// (rigid solid sits in the `+ẑ` half-space). Plane offset
    /// `displacement − edge_len`, so the plane surface lies at `z =
    /// edge_len − displacement` — a top-face vertex at rest position
    /// `z = edge_len` has signed distance `−displacement` (penetrated
    /// by exactly `displacement` at rest config). The penalty force
    /// pushes the top face down; Newton equilibrates to a deformed
    /// top-face strain `δ_eq < displacement` per finite-κ semantics
    /// (`δ_eq → displacement` as `κ_pen → ∞`). V-3a's analytic
    /// comparison `F = E·A·ε` reads `ε = δ_eq / edge_len` at
    /// equilibrium, not `displacement / edge_len`.
    ///
    /// `(κ_pen, d̂)` defaults pinned at
    /// [`PENALTY_KAPPA_DEFAULT`](crate::contact::PenaltyRigidContact)
    /// and [`PENALTY_DHAT_DEFAULT`](crate::contact::PenaltyRigidContact)
    /// per scope memo Decision J — V-7 commit 11 testing surface
    /// (`PenaltyRigidContact::with_params`) is not exposed here.
    ///
    /// # Returns
    ///
    /// 4-tuple `(mesh, bc, initial, contact)` — no theta tensor (the
    /// load is plane-displaced, not externally tractioned).
    ///
    /// # Panics
    ///
    /// - `edge_len` non-positive or non-finite.
    /// - `cell_size` non-positive, non-finite, or doesn't divide
    ///   `edge_len` to an integer (within `1e-9` rounding).
    /// - The implied `n_per_edge` is odd or zero (inherited from
    ///   [`HandBuiltTetMesh::uniform_block`]).
    /// - `displacement` non-finite.
    #[must_use]
    pub fn compressive_block_on_plane(
        edge_len: f64,
        cell_size: f64,
        displacement: f64,
        material_field: &MaterialField,
    ) -> (
        HandBuiltTetMesh,
        BoundaryConditions,
        SceneInitial,
        PenaltyRigidContact,
    ) {
        assert!(
            edge_len.is_finite() && edge_len > 0.0,
            "compressive_block_on_plane: edge_len must be finite and positive, got {edge_len}",
        );
        assert!(
            cell_size.is_finite() && cell_size > 0.0,
            "compressive_block_on_plane: cell_size must be finite and positive, got {cell_size}",
        );
        assert!(
            displacement.is_finite(),
            "compressive_block_on_plane: displacement must be finite, got {displacement}",
        );
        let n_f = edge_len / cell_size;
        assert!(
            (n_f - n_f.round()).abs() < 1e-9,
            "compressive_block_on_plane: cell_size = {cell_size} must divide edge_len = \
             {edge_len} to an integer (got n = {n_f}); V-3a refinement levels assume \
             integer cells per axis",
        );
        // `n_f` is verified as a small positive integer above; the
        // truncation cast can't lose precision.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_per_edge = n_f.round() as usize;

        let mesh = HandBuiltTetMesh::uniform_block(n_per_edge, edge_len, material_field);

        // Full-pin every bottom-face vertex (z = 0 band, half-cell
        // tolerance for FP safety even though the grid vertices land
        // exactly on integer multiples of cell_size). V-3a's uniaxial-
        // stress closed-form ideally wants z-only pinning of the
        // bottom face with x/y free; Phase 5 BoundaryConditions only
        // models full-vertex Dirichlet, so the full pin is the
        // nearest-representable approximation — see the docstring
        // section above on the constrained-modulus approximation.
        let band_tol = 0.5 * cell_size;
        let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < band_tol);

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

        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            loaded_vertices: Vec::new(),
        };

        // Plane from above: outward normal `-ẑ` (rigid solid in the
        // `+ẑ` half-space, soft block in the `-ẑ` half-space relative
        // to the plane). signed_distance(p) = p · (-ẑ) - offset =
        // -p_z - offset; with offset = displacement - edge_len, a
        // top-face vertex at p_z = edge_len has sd = -edge_len -
        // (displacement - edge_len) = -displacement (penetrated by
        // exactly `displacement` at rest config).
        let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), displacement - edge_len);
        let contact = PenaltyRigidContact::new(vec![plane]);

        (mesh, bc, initial, contact)
    }

    /// V-3 sphere-on-plane scene — soft sphere pressed onto a rigid
    /// plane by an axial force, applied through a top-of-sphere
    /// loaded-vertex band.
    ///
    /// Phase 5 commit 6 scaffolding for the V-3 Hertzian gate (commit
    /// 9) per `phase_5_penalty_contact_scope.md` §1 V-3.
    ///
    /// # Geometry
    ///
    /// Soft sphere of radius `radius` meshed via
    /// [`SdfMeshedTetMesh::from_sdf`] with [`SphereSdf`] (centered at
    /// origin), wrapped in a `[-radius - margin, radius + margin]³`
    /// bbox at `cell_size`. Margin pinned at
    /// [`SPHERE_BBOX_MARGIN_RATIO`] × `cell_size` to give the BCC
    /// lattice room to enclose the SDF zero set without surface
    /// clipping; mirrors Phase 4's `LAYERED_SPHERE_BBOX_HALF_EXTENT`
    /// design (Phase 3 / IV-4 `bbox_half_extent / cell_size = 6.0`
    /// ratio at h/2 = 0.02 m).
    ///
    /// The sphere's rest configuration sits centered at the origin —
    /// its bottom pole at `z = -radius`. The rigid plane (per the
    /// **Contact** section below) lives at `z = -radius - d̂` so that
    /// at rest config no soft vertex penetrates the band. V-3 (commit
    /// 9) drives the system with `force` axial-down on the top-of-
    /// sphere band; Newton equilibrates to a Hertzian indentation `δ`
    /// at the south pole.
    ///
    /// # Boundary conditions
    ///
    /// `pinned_vertices` carries the **equator pin set** — the four
    /// referenced vertices nearest the cardinal-direction equator
    /// points `(±radius, 0, 0)` and `(0, ±radius, 0)` (deduplicated;
    /// at coarse cell sizes the BCC lattice may collapse two cardinal
    /// targets onto a single vertex). The four pins kill all six
    /// rigid-body modes (three translation + three rotation) by
    /// construction with `~90°` distribution around the equator,
    /// preserving axial symmetry under the `+ẑ`-aligned axial load.
    ///
    /// **Why pinning is necessary** (resolved at commit 9 V-3
    /// empirical surfacing): the helper's pre-commit-9 design assumed
    /// contact-penalty damping plus loaded-vertex traction asymmetry
    /// would damp rigid-body modes from an empty pinned set. At rest
    /// configuration the south pole sits exactly at `sd = +d̂` (band
    /// edge per the **Contact** section below), so the contact
    /// active-set is empty and the contact tangent is zero —
    /// providing no damping. The `LoadAxis::AxisZ` traction is
    /// pure-`+ẑ` per loaded vertex, with no x/y component to break
    /// the lateral rigid-body translation modes either. Newton's
    /// first iteration produces a search direction along the
    /// undamped rigid-body modes; Armijo line-search stalls (no
    /// step length finds residual decrease). The four-pin equator
    /// set fixes this with documented Saint-Venant distortion at
    /// the pin points (order `ν · δ_Hertz`, e.g. `~130 μm` at V-3
    /// commit-9 parameters where `δ_Hertz ≈ 316 μm`) — small
    /// relative to `δ_Hertz` itself and far from the south contact
    /// patch (Saint-Venant decay over `R = 1 cm`).
    ///
    /// `loaded_vertices` is the **top-of-sphere band** (every vertex
    /// within `0.5 × cell_size` of `z = +radius`, filtered to
    /// `referenced_vertices` to drop BCC lattice orphans), each paired
    /// with [`LoadAxis::AxisZ`] for a `−ẑ` (downward) per-vertex force.
    /// `theta` is a length-`1` tensor carrying the broadcast magnitude
    /// `−force / N_loaded` — under the [`LoadAxis::AxisZ`] convention
    /// (`backward_euler.rs:807-817`) the single scalar is broadcast to
    /// every loaded vertex's z-DOF, summing to `−force` total (the
    /// axial down force pressing the sphere onto the plane).
    ///
    /// # Contact
    ///
    /// Single [`RigidPlane`] below the sphere — outward normal `+ẑ`,
    /// offset `−radius − d̂` so the plane surface lies at `z = −radius
    /// − d̂`. At rest config, the south-pole vertex at `z = −radius`
    /// has signed distance `+d̂` — exactly at the edge of the contact
    /// band, no penalty force yet. V-3 commit 9 drives the system
    /// into contact via `theta`.
    ///
    /// `(κ_pen, d̂)` defaults pinned per scope memo Decision J.
    ///
    /// # Returns
    ///
    /// 5-tuple `(mesh, bc, initial, contact, theta)` — `theta` is the
    /// length-1 broadcast magnitude `−force / N_loaded` (the
    /// [`LoadAxis::AxisZ`] solver convention applies this scalar to
    /// every loaded vertex's z-DOF).
    ///
    /// # Errors
    ///
    /// Forwards [`SdfMeshedTetMesh::from_sdf`]'s error variants
    /// ([`MeshingError::EmptyMesh`] when `cell_size` is too coarse to
    /// span the sphere; [`MeshingError::NonFiniteSdfValue`] forwarded
    /// for the general callsite — impossible for a finite-radius
    /// [`SphereSdf`]).
    ///
    /// # Panics
    ///
    /// - `radius` non-positive or non-finite.
    /// - `cell_size` non-positive or non-finite.
    /// - `force` non-finite.
    /// - The implied loaded-vertex band turns up empty (degenerate
    ///   geometry — caller picked a `cell_size` too coarse to
    ///   resolve the top pole).
    pub fn sphere_on_plane(
        radius: f64,
        cell_size: f64,
        force: f64,
        material_field: MaterialField,
    ) -> Result<
        (
            SdfMeshedTetMesh,
            BoundaryConditions,
            SceneInitial,
            PenaltyRigidContact,
            Tensor<f64>,
        ),
        MeshingError,
    > {
        assert!(
            radius.is_finite() && radius > 0.0,
            "sphere_on_plane: radius must be finite and positive, got {radius}",
        );
        assert!(
            cell_size.is_finite() && cell_size > 0.0,
            "sphere_on_plane: cell_size must be finite and positive, got {cell_size}",
        );
        assert!(
            force.is_finite(),
            "sphere_on_plane: force must be finite, got {force}",
        );

        let half_extent = SPHERE_BBOX_MARGIN_RATIO.mul_add(cell_size, radius);
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(-half_extent, -half_extent, -half_extent),
                Vec3::new(half_extent, half_extent, half_extent),
            ),
            cell_size,
            material_field: Some(material_field),
        };
        let sphere_sdf = SphereSdf { radius };
        let mesh = SdfMeshedTetMesh::from_sdf(&sphere_sdf, &hints)?;

        let referenced: BTreeSet<VertexId> = referenced_vertices(&mesh).into_iter().collect();
        let band_tol = 0.5 * cell_size;
        let loaded: Vec<VertexId> =
            pick_vertices_by_predicate(&mesh, |p| (p.z - radius).abs() < band_tol)
                .into_iter()
                .filter(|v| referenced.contains(v))
                .collect();
        assert!(
            !loaded.is_empty(),
            "sphere_on_plane: top-of-sphere band turned up empty at cell_size = {cell_size}; \
             half-cell tolerance {band_tol} is below the mesher's cut-point density on \
             radius = {radius}",
        );

        // Equator pin set: the four referenced vertices nearest the
        // cardinal-direction equator points. Removes all six rigid-
        // body modes (3 translation + 3 rotation) by construction;
        // see the "Boundary conditions" section in the docstring above
        // for the empirical motivation (Newton fails at iter 0 from
        // rest under empty pinned_vertices because contact + traction
        // asymmetry alone don't damp the modes).
        let positions = mesh.positions();
        let cardinal_targets = [
            Vec3::new(radius, 0.0, 0.0),
            Vec3::new(-radius, 0.0, 0.0),
            Vec3::new(0.0, radius, 0.0),
            Vec3::new(0.0, -radius, 0.0),
        ];
        let mut pinned: Vec<VertexId> = Vec::new();
        for target in cardinal_targets {
            // Walk referenced vertices, find argmin of squared distance
            // to the target. `referenced` is BTreeSet<VertexId> so
            // iteration is deterministic; argmin is unique under generic
            // BCC lattice positioning.
            //
            // `referenced` is nonempty by mesh construction — already
            // implied by the loaded-band assert above (loaded ⊆
            // referenced and loaded nonempty). Convert the empty-set
            // case to an `assert!` so clippy accepts the runtime check
            // pattern used elsewhere in this helper.
            let nearest_opt = referenced.iter().copied().min_by(|&a, &b| {
                let da = (positions[a as usize] - target).norm_squared();
                let db = (positions[b as usize] - target).norm_squared();
                da.total_cmp(&db)
            });
            assert!(
                nearest_opt.is_some(),
                "sphere_on_plane: referenced vertex set must be nonempty by mesh \
                 construction (already asserted via the loaded-band check above)",
            );
            // Cast safe: `is_some()` checked immediately above.
            #[allow(clippy::unwrap_used)]
            pinned.push(nearest_opt.unwrap());
        }
        pinned.sort_unstable();
        pinned.dedup();
        assert!(
            pinned.len() >= 3,
            "sphere_on_plane: equator pin set must contain at least 3 distinct vertices for \
             rigid-body mode removal; got {} at cell_size = {cell_size} (mesh resolution may \
             be too coarse to hit four distinct cardinal-equator points)",
            pinned.len(),
        );

        // `loaded.len()` fits f64 exactly for any sim-soft mesh size.
        // AxisZ convention: theta is a length-1 broadcast magnitude
        // (backward_euler.rs:807-817), NOT a per-vertex tensor. The
        // single scalar is applied to every loaded vertex's z-DOF.
        #[allow(clippy::cast_precision_loss)]
        let force_per_vertex = -force / loaded.len() as f64;
        let theta = Tensor::from_slice(&[force_per_vertex], &[1]);

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

        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
        };

        // Plane below, outward normal `+ẑ`. signed_distance(p) = p_z -
        // offset; offset = -(radius + d̂) so the plane surface is at z
        // = -(radius + d̂). South-pole vertex at z = -radius gets sd =
        // -radius - (-(radius + d̂)) = d̂ at rest config — exactly at
        // the contact-band edge.
        let plane = RigidPlane::new(
            Vec3::new(0.0, 0.0, 1.0),
            -(radius + crate::contact::penalty::PENALTY_DHAT_DEFAULT),
        );
        let contact = PenaltyRigidContact::new(vec![plane]);

        Ok((mesh, bc, initial, contact, theta))
    }

    /// V-5 dropping-sphere scene — soft sphere released above a rigid
    /// plane, freely falling under gravity in the dynamic regime.
    ///
    /// Phase 5 commit 6 scaffolding for the V-5 hygiene gate (commit
    /// 10) per `phase_5_penalty_contact_scope.md` §1 V-5.
    ///
    /// # Geometry
    ///
    /// Soft sphere of radius `radius` meshed identically to
    /// [`Self::sphere_on_plane`] but with rest configuration shifted
    /// upward to `(0, 0, release_height)` — initial positions equal
    /// rest positions plus `+ẑ · release_height` (rigid translation,
    /// zero strain at t=0). The mesh's stored `positions()` (the rest
    /// configuration) stays centered at origin so per-tet centroid
    /// material sampling and reference-geometry assembly remain
    /// canonical; only `x_prev` carries the offset.
    ///
    /// `release_height` must exceed `radius + d̂` so no vertex starts
    /// inside the contact band — Newton's first iteration sees zero
    /// active pairs and the dynamics are pure free-fall until the
    /// sphere reaches the plane.
    ///
    /// # Boundary conditions
    ///
    /// `pinned_vertices` and `loaded_vertices` are both empty — the
    /// sphere is in free flight, with gravity supplied at the solver
    /// level (V-5 commit 10 will wire gravity onto [`SolverConfig`];
    /// Phase 5 commit 6 builds the static scene only).
    ///
    /// # Contact
    ///
    /// Single [`RigidPlane`] at `z = 0` with outward normal `+ẑ`
    /// (rigid solid in the `−ẑ` half-space, "table" the sphere drops
    /// onto). `(κ_pen, d̂)` defaults pinned per scope memo Decision J.
    ///
    /// # Returns
    ///
    /// 4-tuple `(mesh, bc, initial, contact)` — no theta (gravity is a
    /// solver-level body force, not an external traction).
    ///
    /// # Errors
    ///
    /// Forwards [`SdfMeshedTetMesh::from_sdf`]'s error variants per
    /// [`Self::sphere_on_plane`].
    ///
    /// # Panics
    ///
    /// - `radius` non-positive or non-finite.
    /// - `cell_size` non-positive or non-finite.
    /// - `release_height` non-finite or `<= radius +
    ///   PENALTY_DHAT_DEFAULT` (sphere would start in contact band,
    ///   defeating the drop-and-rest setup).
    pub fn dropping_sphere(
        radius: f64,
        cell_size: f64,
        release_height: f64,
        material_field: MaterialField,
    ) -> Result<
        (
            SdfMeshedTetMesh,
            BoundaryConditions,
            SceneInitial,
            PenaltyRigidContact,
        ),
        MeshingError,
    > {
        assert!(
            radius.is_finite() && radius > 0.0,
            "dropping_sphere: radius must be finite and positive, got {radius}",
        );
        assert!(
            cell_size.is_finite() && cell_size > 0.0,
            "dropping_sphere: cell_size must be finite and positive, got {cell_size}",
        );
        let d_hat = crate::contact::penalty::PENALTY_DHAT_DEFAULT;
        assert!(
            release_height.is_finite() && release_height > radius + d_hat,
            "dropping_sphere: release_height = {release_height} must be finite and strictly \
             greater than radius + d̂ = {} so the sphere starts clear of the contact band",
            radius + d_hat,
        );

        let half_extent = SPHERE_BBOX_MARGIN_RATIO.mul_add(cell_size, radius);
        let hints = MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(-half_extent, -half_extent, -half_extent),
                Vec3::new(half_extent, half_extent, half_extent),
            ),
            cell_size,
            material_field: Some(material_field),
        };
        let sphere_sdf = SphereSdf { radius };
        let mesh = SdfMeshedTetMesh::from_sdf(&sphere_sdf, &hints)?;

        let positions = mesh.positions();
        let n_dof = 3 * positions.len();
        let mut x_prev_flat = vec![0.0; n_dof];
        for (v, pos) in positions.iter().enumerate() {
            x_prev_flat[3 * v] = pos.x;
            x_prev_flat[3 * v + 1] = pos.y;
            x_prev_flat[3 * v + 2] = pos.z + release_height;
        }
        let initial = SceneInitial {
            x_prev: Tensor::from_slice(&x_prev_flat, &[n_dof]),
            v_prev: Tensor::zeros(&[n_dof]),
        };

        let bc = BoundaryConditions {
            pinned_vertices: Vec::new(),
            loaded_vertices: Vec::new(),
        };

        let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let contact = PenaltyRigidContact::new(vec![plane]);

        Ok((mesh, bc, initial, contact))
    }
}

/// BCC lattice margin around a sphere SDF, in multiples of `cell_size`.
///
/// Pinned at 6 to match Phase 4's
/// [`LAYERED_SPHERE_BBOX_HALF_EXTENT`] design (`half_extent /
/// cell_size = 6.0` at `cell_size = 0.02`). Gives the mesher room to
/// enclose the SDF zero set without surface clipping. V-3 / V-5 use
/// this for the dynamic radius-and-cell-size combinations they sweep
/// (the layered-silicone-sphere helper hardcodes its margin because
/// its radius is fixed by the [`LAYERED_SPHERE_R_OUTER`] const).
const SPHERE_BBOX_MARGIN_RATIO: f64 = 6.0;

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
