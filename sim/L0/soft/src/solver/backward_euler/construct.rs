//! [`CpuNewtonSolver`](super::CpuNewtonSolver) construction.

use std::collections::BTreeSet;

use faer::Side;
use faer::sparse::linalg::solvers::{SymbolicLlt, SymbolicLu};
use faer::sparse::{SparseColMat, Triplet};
use nalgebra::{Matrix3, SMatrix};

use crate::Vec3;
use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::{Mesh, TetId, VertexId};
use crate::readout::BoundaryConditions;

use super::CpuNewtonSolver;
use super::{ElementGeometry, SolverConfig};

/// Compose the `N` ordered element-local node ids from a tet's 4 corners and
/// (for a higher-order element, `N > 4`) its 6 midside nodes — consuming the
/// rung-3a additive midside channel (§3.2 of `docs/SIM_SOFT_TET10_PLAN.md`).
/// For a linear element (`N == 4`) the midsides are ignored and this returns
/// exactly [`Mesh::tet_vertices`]. Slots `4..N` follow the corners in the
/// canonical [`TET10_EDGE_NODES`](crate::element::TET10_EDGE_NODES) order, as
/// [`Mesh::tet_midside_nodes`] returns them.
//
// expect_used: a higher-order (N > 4) element is only ever constructed against
// a midside-surfacing mesh (e.g. `Tet10Mesh`); a `None` here is a
// construction-time programmer error, declared in `new()`'s `# Panics`.
#[allow(clippy::expect_used)]
fn element_node_ids<const N: usize>(
    corners: [VertexId; 4],
    midsides: Option<[VertexId; 6]>,
) -> [VertexId; N] {
    let mut nodes: [VertexId; N] = [0; N];
    nodes[..4].copy_from_slice(&corners);
    if N > 4 {
        // `N > 4` is Tet10 (`N == 10`) in this ladder — the six midsides fill
        // slots `4..10`.
        let mids = midsides.expect(
            "a higher-order (N > 4) element requires a mesh that surfaces midside \
             nodes, but Mesh::tet_midside_nodes returned None",
        );
        nodes[4..].copy_from_slice(&mids);
    }
    nodes
}

/// HRZ (Hinton–Rock–Zienkiewicz) diagonal mass-lumping weights — the per-node
/// fractions of an element's total mass, summing to 1. Each weight is
/// `∫N_i² / Σ_k ∫N_k²` evaluated at the element's Gauss points; the constant
/// `|detJ|` of a straight-edged element cancels in the ratio, so the weights
/// are element-independent (computed once). Positive by construction
/// (`N_i² ≥ 0`, positive quadrature weights) — which is what keeps the Tet10
/// corner masses positive where naive row-sum lumping (`∫N_i`) goes negative
/// and would break the Cholesky factor at a dynamic dt.
fn hrz_mass_weights<E, const N: usize, const G: usize>(element: &E) -> [f64; N]
where
    E: Element<N, G>,
{
    let mut numerator = [0.0_f64; N];
    for (xi, w) in element.gauss_points() {
        let sf = element.shape_functions(xi);
        for (num_i, &n_i) in numerator.iter_mut().zip(sf.iter()) {
            *num_i += w * n_i * n_i;
        }
    }
    let total: f64 = numerator.iter().sum();
    numerator.map(|n| n / total)
}

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Assemble a solver from its element, mesh, contact, integration
    /// configuration, and boundary conditions. Per-tet `M` instances
    /// are read from `mesh.materials()` at assembly time.
    ///
    /// `Box<dyn Solver<Tape = CpuTape>>` is the intended public handle;
    /// direct access to the concrete type is only needed for
    /// monomorphized benches (Phase E+). The assembly cache
    /// (`element_geometries`, `mass_per_dof`, `free_dof_indices`,
    /// `full_to_free_idx`, `symbolic`, `n_dof`, `n_free`) is populated
    /// here once and consumed by `solve_impl` / `factor_at_position` /
    /// `armijo_backtrack` per Newton iter. `new()` is non-`const fn`
    /// because the cache build runs `mesh.tet_vertices`, sparse-pattern
    /// construction, and faer's symbolic factorization — none of which
    /// are const-evaluable.
    ///
    /// # Panics
    ///
    /// Panics if `boundary_conditions` reference an out-of-range,
    /// doubly-classified (both pinned and loaded), or orphan
    /// (tet-unreferenced) loaded vertex; if a higher-order element (`N > 4`)
    /// is paired with a mesh that does not surface midside nodes
    /// ([`Mesh::tet_midside_nodes`] returns `None`); or if the rest mesh is
    /// malformed (a singular reference Jacobian, or more vertices than fit in
    /// a `u32` `VertexId`).
    //
    // expect_used + panic justifications:
    //   • Singular reference Jacobian = malformed rest mesh, programmer
    //     bug at construction time.
    //   • SymbolicLlt failure = solver-pattern build is wrong (impossible
    //     for any valid mesh + Dirichlet set), programmer bug.
    //   • SymbolicLu failure shares the same "pattern build wrong" rationale;
    //     LU symbolic factorization of a valid free-block pattern only
    //     errors out on OutOfMemory, which on a healthy host is a programmer
    //     bug on the mesh-size axis.
    //
    // Lint allows: same Mesh-trait-API tax + element-node iteration +
    // per-element similar-name pairs as the assembly methods below.
    #[must_use]
    #[allow(
        clippy::expect_used,
        clippy::too_many_lines,
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    pub fn new(
        element: E,
        mesh: Msh,
        contact: C,
        config: SolverConfig,
        boundary_conditions: BoundaryConditions,
    ) -> Self {
        let n_tets = mesh.n_tets();
        let n_vertices = mesh.n_vertices();
        let n_dof = 3 * n_vertices;

        // 0. BoundaryConditions validation. Catch misconfigured scenes
        // at construction time rather than later as opaque
        // out-of-bounds panics inside the cache build or first
        // step()-call. Four checks: (a) pinned vertex IDs are in
        // range, (b) loaded vertex IDs are in range, (c) no vertex
        // appears in both pinned and loaded sets (load on a
        // Dirichlet-clamped DOF is unphysical and would silently
        // overwrite f_ext on a vertex that never moves), (d) loaded
        // vertices are referenced by some tet (load on an orphan is
        // unphysical — no element receives the traction; checked
        // below after the tet incidence walk).
        let pinned_set: BTreeSet<VertexId> = boundary_conditions
            .pinned_vertices
            .iter()
            .copied()
            .collect();
        let n_vertices_u32 =
            VertexId::try_from(n_vertices).expect("n_vertices must fit in VertexId (u32)");
        for &v in &boundary_conditions.pinned_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.pinned_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
        }
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
            assert!(
                !pinned_set.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is also in pinned_vertices — load on a Dirichlet-clamped \
                 vertex is unphysical (f_ext is overwritten but the vertex \
                 never moves)",
            );
        }

        // Roller / per-axis Dirichlet validation + dense mask. A roller
        // pins a vertex to rest on the `true` axes of its mask while
        // leaving the `false` axes free — the per-DOF generalization of
        // `pinned_vertices`. `roller_mask[v]` defaults to all-free; the
        // per-DOF free-DOF construction below consults it per axis.
        let mut roller_mask = vec![[false; 3]; n_vertices];
        let mut roller_seen: BTreeSet<VertexId> = BTreeSet::new();
        for &(v, mask) in &boundary_conditions.roller_vertices {
            assert!(
                v < n_vertices_u32,
                "BoundaryConditions.roller_vertices contains vertex ID {v}, \
                 out of range for {n_vertices}-vertex mesh",
            );
            assert!(
                mask.iter().any(|&b| b),
                "BoundaryConditions.roller_vertices entry for vertex ID {v} has \
                 an all-false mask (no axis constrained) — a free vertex needs no \
                 entry; use a mask with at least one constrained axis",
            );
            assert!(
                roller_seen.insert(v),
                "BoundaryConditions.roller_vertices lists vertex ID {v} twice \
                 (ambiguous per-axis mask)",
            );
            assert!(
                !pinned_set.contains(&v),
                "BoundaryConditions vertex ID {v} is in both pinned_vertices and \
                 roller_vertices — a full pin already constrains every axis, so a \
                 roller on the same vertex is ambiguous",
            );
            roller_mask[v as usize] = mask;
        }
        // A loaded vertex must be free on ALL three axes: the external-force
        // assembly and the autograd VJP both resolve its xyz free-DOF indices
        // (`full_to_free_idx[3v + {0,1,2}]` must be `Some`). Loaded ∩ pinned is
        // rejected above; reject loaded ∩ roller here for the same reason.
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                !roller_seen.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is also roller-constrained — a loaded vertex must be free \
                 on all three axes (assembly + VJP resolve all xyz free indices)",
            );
        }

        // Tet incidence walk. Used immediately below to (a) reject
        // loads on unreferenced vertices and (b) auto-pin
        // unreferenced vertices into `effective_pinned`. An orphan
        // free DOF would have zero mass (no element contributes to
        // `mass_per_dof` for that vertex) AND zero element
        // contribution to its Hessian row/column, leaving a singular
        // diagonal (`(k, k) == 0`) that faer's Cholesky would either
        // panic on (slice-out-of-bounds during pattern permute) or
        // silently produce garbage. Phase 2's hand-built meshes
        // never tripped this because every vertex was referenced;
        // Phase 3's `SdfMeshedTetMesh` retains the full BCC lattice
        // in `positions()` including unreferenced corners outside
        // the SDF zero set (per its module doc) and surfaces this
        // gap via III-3.
        let mut referenced: BTreeSet<VertexId> = BTreeSet::new();
        for tet_id in 0..n_tets as TetId {
            for v in mesh.tet_vertices(tet_id) {
                referenced.insert(v);
            }
            // Rung 3b unpin: a higher-order element's midside nodes are real
            // free DOFs (they carry HRZ mass), so they must be referenced too
            // — otherwise the orphan auto-pin below Dirichlet-clamps them,
            // which is exactly rung 3a's bit-identical-Tet4 behavior (kept for
            // `N == 4`, where `tet_midside_nodes` is `None`).
            if N > 4 {
                for v in mesh
                    .tet_midside_nodes(tet_id)
                    .expect("a higher-order (N > 4) element requires midside nodes")
                {
                    referenced.insert(v);
                }
            }
        }
        for &(v, _) in &boundary_conditions.loaded_vertices {
            assert!(
                referenced.contains(&v),
                "BoundaryConditions.loaded_vertices contains vertex ID {v}, \
                 which is not referenced by any tet — load on an orphan \
                 vertex is unphysical (no element receives the traction)",
            );
        }
        // Effective pinned set: user-supplied pins UNION (vertices
        // not referenced by any tet). Orphans pinned silently —
        // their DOFs were unsolveable anyway, and exposing this as a
        // hard error would force every mesher-generated-mesh test
        // site to construct an orphan-aware BC. Per-iter assembly
        // reads `full_to_free_idx` (built from `effective_pinned`
        // below), so the auto-pin propagates without any other code
        // change. The `boundary_conditions` field stored on the
        // struct keeps the user's supplied form for transparency.
        let mut effective_pinned: BTreeSet<VertexId> = pinned_set;
        for v in 0..n_vertices_u32 {
            if !referenced.contains(&v) {
                effective_pinned.insert(v);
            }
        }

        // 1. Per-element reference geometry. Computed once; all per-iter
        // assembly reads from this cache rather than recomputing.
        let x_rest = mesh.positions();
        let mut element_geometries = Vec::with_capacity(n_tets);
        // Corner constant-strain (parametric) shape gradients. The linear
        // element's own gradients ARE these (constant in ξ), so Tet4 reads
        // them straight from the element and stays byte-identical. A Tet10
        // element's gradients VANISH on the corners at the centroid, and rung
        // 3b keeps the Tet10 corner elastic block constant-strain Tet4 anyway
        // (the six midside nodes ride as stiffness-free floating masses, §3.2)
        // — so `N > 4` uses the linear barycentric corner gradients directly.
        // Ladder rung 4 replaces this single-point cache with the per-Gauss-
        // point Tet10 geometry.
        let grad_xi_corner: SMatrix<f64, 4, 3> = if N > 4 {
            SMatrix::<f64, 4, 3>::new(
                -1.0, -1.0, -1.0, // ∂N_0/∂ξ (complement barycentric)
                1.0, 0.0, 0.0, // ∂N_1/∂ξ
                0.0, 1.0, 0.0, // ∂N_2/∂ξ
                0.0, 0.0, 1.0, // ∂N_3/∂ξ
            )
        } else {
            // Tet4: the element's own constant barycentric gradients (ξ
            // ignored). Routed through the element so the Tet4 geometry path
            // is byte-identical to the pre-Tet10 code.
            let g = element.shape_gradients(Vec3::new(0.25, 0.25, 0.25));
            SMatrix::<f64, 4, 3>::from_fn(|a, j| g[(a, j)])
        };
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let v0 = x_rest[verts[0] as usize];
            let v1 = x_rest[verts[1] as usize];
            let v2 = x_rest[verts[2] as usize];
            let v3 = x_rest[verts[3] as usize];
            let j_0 = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
            let j_0_inv = j_0
                .try_inverse()
                .expect("singular reference Jacobian — malformed rest mesh");
            let volume = j_0.determinant().abs() / 6.0;
            // Chain rule: grad_X N_a = grad_ξ N_a · (∂ξ/∂X) = grad_ξ N_a · J_0⁻¹.
            let grad_x_n = grad_xi_corner * j_0_inv;
            element_geometries.push(ElementGeometry { grad_x_n, volume });
        }

        // F-bar nodal-patch cache (locking cure) — built only when enabled;
        // `None` leaves the plain per-element assembly path bit-equal. Built
        // here (before the symbolic pattern) because the F-bar tangent's
        // node-star coupling widens the Hessian sparsity pattern below.
        let fbar_cache = if config.fbar {
            Some(super::fbar::FbarCache::build(&mesh, &element_geometries))
        } else {
            None
        };

        // 2. Lumped per-DOF mass.
        let mut mass_per_dof = vec![0.0; n_dof];
        if N > 4 {
            // Tet10 (higher-order): HRZ (Hinton–Rock–Zienkiewicz) diagonal-
            // scaling lumping over all `N` nodes. Naive row-sum lumping
            // (`∫N_i`) gives NEGATIVE corner masses on a quadratic tet →
            // indefinite tangent → Cholesky failure at a dynamic dt; HRZ
            // scales the positive consistent-mass diagonal (`∫N_i²`) to the
            // element total, so every nodal mass is positive by construction.
            // Only the diagonal is ever needed (no consistent mass matrix).
            let hrz = hrz_mass_weights(&element);
            for tet_id in 0..n_tets as TetId {
                let nodes = element_node_ids::<N>(
                    mesh.tet_vertices(tet_id),
                    mesh.tet_midside_nodes(tet_id),
                );
                let elem_mass = config.density * element_geometries[tet_id as usize].volume;
                for a in 0..N {
                    let v = nodes[a] as usize;
                    let m = elem_mass * hrz[a];
                    mass_per_dof[3 * v] += m;
                    mass_per_dof[3 * v + 1] += m;
                    mass_per_dof[3 * v + 2] += m;
                }
            }
        } else {
            // Tet4: contribute `ρ V_e / 4` to every DOF of every corner
            // (byte-identical to the pre-Tet10 lumped-mass path). Vertices
            // shared by multiple elements accumulate.
            for tet_id in 0..n_tets as TetId {
                let verts = mesh.tet_vertices(tet_id);
                let mass_per_vertex_share =
                    config.density * element_geometries[tet_id as usize].volume / 4.0;
                for a in 0..4 {
                    let v = verts[a] as usize;
                    mass_per_dof[3 * v] += mass_per_vertex_share;
                    mass_per_dof[3 * v + 1] += mass_per_vertex_share;
                    mass_per_dof[3 * v + 2] += mass_per_vertex_share;
                }
            }
        }

        // 3. Free-DOF index mapping. Walks vertices in natural order
        // (deterministic per scope §15 D-3 + Decision M); uses
        // `effective_pinned` (user pins UNION auto-pinned orphans) and
        // `roller_mask` (per-axis Dirichlet) built at validation time.
        // No HashMap on numeric paths per Decision M.
        //
        // Per-DOF freedom: a full pin fixes all three axes; a roller
        // fixes only its `true` axes. A fully-free vertex pushes 3v,
        // 3v+1, 3v+2 in order, so a scene with no rollers yields a
        // BIT-IDENTICAL free-DOF map to the pre-roller per-vertex
        // construction (regression-guarded in tests).
        let mut free_dof_indices = Vec::with_capacity(n_dof);
        for v in 0..n_vertices as VertexId {
            let v_idx = v as usize;
            let full_pin = effective_pinned.contains(&v);
            for ax in 0..3 {
                if !full_pin && !roller_mask[v_idx][ax] {
                    free_dof_indices.push(3 * v_idx + ax);
                }
            }
        }
        let n_free = free_dof_indices.len();
        let mut full_to_free_idx: Vec<Option<usize>> = vec![None; n_dof];
        for (free_idx, &full_idx) in free_dof_indices.iter().enumerate() {
            full_to_free_idx[full_idx] = Some(free_idx);
        }

        // 4. Symbolic factor of the free-DOF Hessian sparsity pattern. The
        // pattern must be EXACTLY the pattern `assemble_free_hessian_triplets`
        // scatters — a numeric entry outside it pattern-mismatches the factor,
        // and (empirically) a symbolic entry the numeric never fills corrupts
        // faer's numeric read. That assembled pattern is two parts:
        //
        // (a) Corner-corner element incidence (Decision J): for each element,
        //     every (a, b) corner pair contributes a 3×3 block at
        //     (free_idx_a, free_idx_b) IF both are free. This is the element's
        //     actual stiffness coupling in rung 3b — the elastic assembly is
        //     corner-only; the corner↔midside and midside↔midside stiffness
        //     blocks arrive with the multi-Gauss-point kernel in rung 4.
        //
        // (b) The free-DOF mass diagonal (below): every free DOF, including a
        //     rung-3b freed midside, gets a positive `(k, k)`. A stiffness-free
        //     midside has ONLY this entry, so its slot must be present or the
        //     factor pattern-mismatches on the mass scatter.
        //
        // BTreeSet keyed by (col, row) gives sorted column-major lower-triangle
        // iteration without a HashMap.
        let mut triplet_set: BTreeSet<(usize, usize)> = BTreeSet::new();
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            for a in 0..4 {
                for b in 0..4 {
                    let va = verts[a] as usize;
                    let vb = verts[b] as usize;
                    for i in 0..3 {
                        for j in 0..3 {
                            let row_full = 3 * va + i;
                            let col_full = 3 * vb + j;
                            if let (Some(row_free), Some(col_free)) =
                                (full_to_free_idx[row_full], full_to_free_idx[col_full])
                                && row_free >= col_free
                            {
                                triplet_set.insert((col_free, row_free));
                            }
                        }
                    }
                }
            }
        }
        // (b) Mirror the mass-diagonal scatter (`assemble_free_hessian_triplets`,
        // `Mass diagonal:`): a `(k, k)` for every free DOF. Idempotent for a
        // corner DOF (already in its corner-corner block above); the sole slot
        // for a freed, stiffness-free midside DOF. For a linear (Tet4) mesh no
        // midsides are free, so this only re-inserts existing diagonals — the
        // pattern (hence the symbolic factor and every downstream solve) stays
        // bit-identical.
        for k in 0..n_free {
            triplet_set.insert((k, k));
        }
        // F-bar coupling widens the pattern to the node-star 2-ring the `J̄_e`
        // patch average induces (dedups against the 1-ring pairs above).
        if let Some(fbar) = &fbar_cache {
            fbar.insert_free_coupling_pattern(&mesh, &full_to_free_idx, &mut triplet_set);
        }

        let pattern_triplets: Vec<Triplet<usize, usize, f64>> = triplet_set
            .iter()
            .map(|&(c, r)| Triplet::new(r, c, 1.0))
            .collect();
        let pattern_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets)
                .expect("malformed free-block triplet pattern");
        let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
            .expect("symbolic factorization of free-block pattern failed");

        // A2 LU fallback: reflect the structurally-symmetric lower-tri
        // pattern into the full pattern needed by `SymbolicLu` (Lu has
        // no `Side` argument; it reads both halves). Diagonal entries
        // emit once; off-diagonals emit at both (r, c) and (c, r). The
        // numeric Lu factor at fall-through symmetrizes the assembled
        // tangent the same way.
        let mut pattern_triplets_full: Vec<Triplet<usize, usize, f64>> =
            Vec::with_capacity(triplet_set.len() * 2);
        for &(c, r) in &triplet_set {
            pattern_triplets_full.push(Triplet::new(r, c, 1.0));
            if c != r {
                pattern_triplets_full.push(Triplet::new(c, r, 1.0));
            }
        }
        let pattern_mat_full: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets_full)
                .expect("malformed full free-block triplet pattern");
        let symbolic_lu = SymbolicLu::<usize>::try_new(pattern_mat_full.symbolic())
            .expect("symbolic LU factorization of free-block pattern failed");

        Self {
            element,
            mesh,
            contact,
            config,
            boundary_conditions,
            element_geometries,
            mass_per_dof,
            free_dof_indices,
            full_to_free_idx,
            symbolic,
            symbolic_lu,
            n_dof,
            n_free,
            fbar_cache,
            friction_surface_drift: Vec3::zeros(),
            _material: std::marker::PhantomData,
        }
    }

    /// Set the rigid contact surface's within-step tangential drift `Δ_surf` (the
    /// `friction_surface_drift` field) and return the solver, builder-style. The
    /// drift is the kinematic collider's
    /// tangential displacement over the step (`≈ v_surf·dt`); friction measures
    /// the soft body's slip RELATIVE to it, so a non-zero drift lets a moving
    /// collider drag the soft body. `Δ_surf = (0,0,0)` (the default) is PR1's
    /// static-collider friction, byte-identical.
    #[must_use]
    pub const fn with_friction_surface_drift(mut self, drift: Vec3) -> Self {
        self.friction_surface_drift = drift;
        self
    }
}
