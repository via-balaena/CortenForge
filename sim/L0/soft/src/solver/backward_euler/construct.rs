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
use super::helpers::element_node_ids;
use super::{ElementGeometry, GaussGeometry, SolverConfig};

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

/// Build the Llt + Lu symbolic factors from the assembled free-DOF sparsity
/// pattern (`triplet_set`, lower-triangle `(col, row)` keys, `n_free × n_free`).
/// The Llt factor consumes the lower triangle (`Side::Lower`); the A2 Lu
/// fallback needs the full pattern, so the lower triangle is reflected — the
/// diagonal once, each off-diagonal at both `(r, c)` and `(c, r)` — exactly as
/// the numeric Lu factor symmetrizes the assembled tangent at fall-through.
//
// expect_used + panic: a pattern-build failure here is impossible for any valid
// mesh + Dirichlet set (the same programmer-bug rationale as `new()`'s own
// `# Panics`); `SymbolicLu` only errors on OutOfMemory. Extracted from `new()`
// so these expects sit inside the safety `#[allow]` window (they had drifted
// past the 300-line back-window as `new()` grew).
#[allow(clippy::expect_used)]
fn build_symbolic_factors(
    triplet_set: &BTreeSet<(usize, usize)>,
    n_free: usize,
) -> (SymbolicLlt<usize>, SymbolicLu<usize>) {
    let pattern_triplets: Vec<Triplet<usize, usize, f64>> = triplet_set
        .iter()
        .map(|&(c, r)| Triplet::new(r, c, 1.0))
        .collect();
    let pattern_mat: SparseColMat<usize, f64> =
        SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets)
            .expect("malformed free-block triplet pattern");
    let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
        .expect("symbolic factorization of free-block pattern failed");

    // A2 LU fallback: reflect the structurally-symmetric lower-tri pattern into
    // the full pattern `SymbolicLu` needs (Lu has no `Side` argument; it reads
    // both halves).
    let mut pattern_triplets_full: Vec<Triplet<usize, usize, f64>> =
        Vec::with_capacity(triplet_set.len() * 2);
    for &(c, r) in triplet_set {
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

    (symbolic, symbolic_lu)
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
            // — otherwise the orphan auto-pin below Dirichlet-clamps them.
            // Gated on the ELEMENT (`N > 4`), not on the mesh: a linear (Tet4)
            // element on an enriched `Tet10Mesh` (where `tet_midside_nodes` is
            // `Some`) keeps its midsides auto-pinned — rung 3a's bit-identical
            // Tet4 behavior.
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

        // 1. Per-element reference geometry — TWO caches from one Jacobian:
        //    (a) `element_geometries`: the single-point corner geometry the
        //        Tet4-flavored consumers read (F-bar, adjoint, validity, mass) —
        //        byte-identical to rung 3b.
        //    (b) `gauss_geometries`: the per-Gauss-point stiffness geometry the
        //        multi-Gauss-point forward kernels integrate over (rung 4).
        //
        // For a STRAIGHT-EDGED element the isoparametric map is affine, so the
        // Jacobian `J` is the constant corner edge-vector matrix
        // `[v1−v0, v2−v0, v3−v0]` — identical for Tet4 and (straight-edged)
        // Tet10, and constant across the element's Gauss points. Only the
        // parametric shape gradient `∇_ξN(ξ_q)` varies per point, so
        // `grad_x_n^q = ∇_ξN(ξ_q) · J⁻¹` differs per Gauss point while `|detJ|`
        // is shared. (A curved element — deferred rung 8 — would make `J`
        // per-point; do not add that here.)
        let x_rest = mesh.positions();
        let mut element_geometries = Vec::with_capacity(n_tets);
        let mut gauss_geometries = Vec::with_capacity(n_tets);
        let gauss_points = element.gauss_points();
        // Single-point (corner) parametric shape gradient for (a). Tet4 reads
        // the element's own constant barycentric gradient (ξ ignored, byte-
        // identical to the pre-Tet10 path); a Tet10 element's gradients VANISH
        // on the corners at the centroid, so `N > 4` uses the linear barycentric
        // corner gradient — the affine constant-strain block. This is the
        // single-point proxy for the validity / adjoint / F-bar consumers; the
        // real (linearly-varying) Tet10 strain lives in (b).
        let grad_xi_corner: SMatrix<f64, 4, 3> = if N > 4 {
            SMatrix::<f64, 4, 3>::new(
                -1.0, -1.0, -1.0, // ∂N_0/∂ξ (complement barycentric)
                1.0, 0.0, 0.0, // ∂N_1/∂ξ
                0.0, 1.0, 0.0, // ∂N_2/∂ξ
                0.0, 0.0, 1.0, // ∂N_3/∂ξ
            )
        } else {
            let g = element.shape_gradients(Vec3::new(0.25, 0.25, 0.25));
            SMatrix::<f64, 4, 3>::from_fn(|a, j| g[(a, j)])
        };
        for tet_id in 0..n_tets as TetId {
            let verts = mesh.tet_vertices(tet_id);
            let v0 = x_rest[verts[0] as usize];
            let v1 = x_rest[verts[1] as usize];
            let v2 = x_rest[verts[2] as usize];
            let v3 = x_rest[verts[3] as usize];
            // Edge-vector Jacobian (a DISTINCT form from the isoparametric
            // Σ Xᵢ⊗∇_ξNᵢ accumulation — routing Tet4 through that would reorder
            // the arithmetic and break byte-identity; §3.1 trap). For a
            // straight-edged Tet10 this edge-vector `J` equals the affine
            // isoparametric Jacobian exactly.
            let j_0 = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
            let j_0_inv = j_0
                .try_inverse()
                .expect("singular reference Jacobian — malformed rest mesh");
            let det = j_0.determinant();
            let volume = det.abs() / 6.0;

            // (a) Single-point corner geometry — byte-identical to rung 3b.
            let grad_x_n = grad_xi_corner * j_0_inv;
            element_geometries.push(ElementGeometry { grad_x_n, volume });

            // (b) Per-Gauss-point stiffness geometry.
            let gauss: [(SMatrix<f64, N, 3>, f64); G] = std::array::from_fn(|q| {
                let (xi, w_ref) = gauss_points[q];
                // Chain rule: grad_X N_a = grad_ξ N_a(ξ_q) · (∂ξ/∂X) = grad_ξ N_a · J⁻¹.
                let grad_x_n = element.shape_gradients(xi) * j_0_inv;
                // ★ Byte-identity (§3.1 trap): Tet4 (the single-Gauss-point
                // linear element) keeps the literal `|det|/6.0`. Reconstructing
                // it as `w_ref·|det|` would use `w_ref = 1/6`, which is not
                // representable → a silent ULP break. Tet10's `w_ref = 1/24` is
                // representable and applied directly (its weights are
                // tolerance-checked, not byte-frozen). For Tet4 the single pair
                // equals `element_geometries` above bit-for-bit.
                let weight = if N == 4 {
                    det.abs() / 6.0
                } else {
                    w_ref * det.abs()
                };
                (grad_x_n, weight)
            });
            gauss_geometries.push(GaussGeometry { gauss });
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
                let nodes = element_node_ids::<M, Msh, N>(&mesh, tet_id);
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
        // (a) Element node incidence (Decision J): for each element, every
        //     (a, b) node pair contributes a 3×3 block at (free_idx_a,
        //     free_idx_b) IF both are free. ★ This ranges over ALL `N` element
        //     nodes (`element_node_ids`), in EXACT lockstep with the numeric
        //     stiffness assembly (`assemble_free_hessian_triplets`, same
        //     `0..N` node loops). Rung 4's multi-Gauss-point kernel fills the
        //     corner↔midside and midside↔midside blocks, so the numeric pattern
        //     now reaches full 10-node incidence — the symbolic pattern must
        //     widen with it. Symbolic MUST equal numeric: a symbolic entry the
        //     numeric never fills silently corrupts faer's numeric read (a
        //     wrong factor, not a crash — the rung-3b `is_llt` finding). For a
        //     linear (Tet4) mesh `element_node_ids` returns the 4 corners, so
        //     `0..N` is `0..4` and the pattern is bit-identical to pre-rung-4.
        //
        // (b) The free-DOF mass diagonal (below): every free DOF gets a
        //     positive `(k, k)`. Idempotent against (a) for a stiffness-coupled
        //     DOF; retained for robustness (and the sole slot for any
        //     hypothetical uncoupled free DOF).
        //
        // BTreeSet keyed by (col, row) gives sorted column-major lower-triangle
        // iteration without a HashMap.
        let mut triplet_set: BTreeSet<(usize, usize)> = BTreeSet::new();
        for tet_id in 0..n_tets as TetId {
            let nodes = element_node_ids::<M, Msh, N>(&mesh, tet_id);
            for a in 0..N {
                for b in 0..N {
                    let va = nodes[a] as usize;
                    let vb = nodes[b] as usize;
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

        // Symbolic Llt + Lu factors of the finalized free-DOF pattern
        // (extracted so its infallible-in-practice expects stay inside the
        // safety-lint window).
        let (symbolic, symbolic_lu) = build_symbolic_factors(&triplet_set, n_free);

        Self {
            element,
            mesh,
            contact,
            config,
            boundary_conditions,
            element_geometries,
            gauss_geometries,
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
