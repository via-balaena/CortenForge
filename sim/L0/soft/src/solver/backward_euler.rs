//! `CpuNewtonSolver` — backward-Euler Newton with Armijo line-search.
//!
//! Residual form per [`Part 6 Ch 02:13–15`][r]:
//! \\[ r(x; \theta) = (M / \Delta t^2)\,(x - x_\text{prev} - \Delta t\,v_\text{prev})
//!                  + f_\text{int}(x) - f_\text{ext}(\theta) \\]
//!
//! Tangent \\(A = \partial r / \partial x = M / \Delta t^2 + K(x)\\) with
//! \\(K = V \cdot B^{\mathsf{T}} \mathbb{C} B\\) assembled per
//! [`Part 3 Ch 00 00-tet4.md`][tet4]. Only the free-DOF block (scope §2
//! Dirichlet: `v_0`, `v_1`, `v_2` pinned) is factored — the free DOFs
//! start at index `FREE_OFFSET = 9` (private module constant).
//!
//! Solve path: faer `SymbolicLlt::try_new` once per `step`-call, then
//! `Llt::try_new_with_symbolic` + `solve_in_place_with_conj` per Newton
//! iteration (scope §11 S-3 Round-1-verified API shape).
//!
//! After convergence, `step` re-factors `A` at `x_final` via
//! `factor_at_position` and pushes `NewtonStepVjp` onto the tape with
//! `theta_var` as parent. The VJP solves the IFT adjoint `A · λ = g_free`
//! and contracts against `∂r/∂θ` — see `NewtonStepVjp` for the math.
//!
//! [r]: ../../../../../../docs/studies/soft_body_architecture/src/60-differentiability/02-implicit-function.md
//! [tet4]: ../../../../../../docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/00-tet4.md

use std::collections::{BTreeMap, BTreeSet};

use faer::linalg::solvers::SolveCore;
use faer::prelude::Reborrow;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{SparseColMat, Triplet};
use faer::{Conj, MatMut, Side};
use nalgebra::{Matrix3, SMatrix};
use sim_ml_chassis::{Tensor, Var};

use super::{CpuTape, NewtonStep, Solver};
use crate::Vec3;
use crate::contact::ContactModel;
use crate::differentiable::newton_vjp::NewtonStepVjp;
use crate::element::Element;
use crate::material::Material;
use crate::mesh::{Mesh, TetId, VertexId};
use crate::readout::{BoundaryConditions, LoadAxis};

/// Number of DOFs in the skeleton system (4 vertices × 3 spatial axes).
const N_DOF: usize = 12;

/// Number of free DOFs after Dirichlet condensation (only `v_3`'s xyz).
const N_FREE: usize = 3;

/// First free-DOF index — `v_3` lives at DOFs 9, 10, 11 under
/// vertex-major + xyz-inner layout.
const FREE_OFFSET: usize = 9;

/// Armijo sufficient-decrease constant (scope §5 R-1).
const ARMIJO_C1: f64 = 1e-4;

/// Per-element reference-frame geometry, pre-computed at solver
/// construction (Phase 2 commit 4a).
///
/// `grad_x_n` is the material-frame shape-function gradient
/// (`SMatrix<f64, 4, 3>`, one row per node, constant across the
/// element for Tet4); `volume` is the rest-configuration tet volume.
/// One per tet in the mesh; cached so the per-iter assembly path
/// doesn't recompute on every Newton step.
#[derive(Clone, Debug)]
struct ElementGeometry {
    grad_x_n: SMatrix<f64, 4, 3>,
    volume: f64,
}

/// Solver configuration — integration parameters the skeleton scene
/// (spec §2) consumes. `skeleton()` returns the spec-§2 defaults.
#[derive(Clone, Copy, Debug)]
pub struct SolverConfig {
    /// Integration time-step (seconds).
    pub dt: f64,
    /// Newton residual tolerance on the free-DOF residual.
    pub tol: f64,
    /// Reference-configuration mass density (`kg/m^3`). The solver
    /// derives per-DOF lumped mass from this — see `reference_geometry`.
    pub density: f64,
    /// Maximum Newton iterations before declaring divergence.
    pub max_newton_iter: usize,
    /// Maximum Armijo backtracks before declaring line-search stall.
    pub max_line_search_backtracks: usize,
}

impl SolverConfig {
    /// Scope §2 defaults for the walking-skeleton scene: `dt = 1e-2`,
    /// `tol = 1e-10` (five digits below gradcheck's 1e-5 bar),
    /// `density = 1030` (silicone-class), up to 10 Newton iterations +
    /// 20 backtracks per iteration.
    #[must_use]
    pub const fn skeleton() -> Self {
        Self {
            dt: 1e-2,
            tol: 1e-10,
            density: 1030.0,
            max_newton_iter: 10,
            max_line_search_backtracks: 20,
        }
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self::skeleton()
    }
}

/// CPU backward-Euler Newton solver.
///
/// Six generic parameters: material `M`, element `E<N, G>`, mesh `Msh`,
/// contact `C`, and const-generic `(N, G)` for element shape.
/// Monomorphized per skeleton type alias `SkeletonSolver`.
pub struct CpuNewtonSolver<M, E, Msh, C, const N: usize, const G: usize>
where
    M: Material,
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    material: M,
    element: E,
    mesh: Msh,
    // NullContact carries no state, but the generic is held so Phase-C
    // IPC can slot in without a solver-level API break.
    _contact: C,
    config: SolverConfig,
    boundary_conditions: BoundaryConditions,

    // ── Phase 2 commit 4a: assembly cache, populated by `new()`. ──
    // The old solve path (commit 3) still uses the hardcoded N_DOF /
    // N_FREE / FREE_OFFSET constants and recomputes per-element
    // geometry per Newton iter via `reference_geometry`; commit 4b
    // switches `solve_impl` / `factor_at_position` / `armijo_backtrack`
    // over to the cache below and removes the redundant path.
    /// One entry per mesh tet — material-frame shape gradients and
    /// rest-configuration volume.
    element_geometries: Vec<ElementGeometry>,
    /// Lumped per-DOF mass (`length n_dof`). For each DOF `i` belonging
    /// to vertex `v = i / 3`, the entry is `Σ_e (ρ V_e / 4)` over every
    /// element `e` that contains `v`. Phase 2 reproduces the
    /// walking-skeleton's "per-vertex mass = ρ `V_total` / 4" rule when
    /// every vertex sits in exactly one tet.
    mass_per_dof: Vec<f64>,
    /// Full-DOF indices of the free DOFs, in ascending order. For the
    /// 1-tet skeleton: `[9, 10, 11]` (`v_3`'s xyz). For multi-tet:
    /// every non-pinned vertex's three DOFs.
    free_dof_indices: Vec<usize>,
    /// Inverse map of `free_dof_indices`: entry `i` is `Some(k)` when
    /// full-DOF `i` is the k-th free DOF, else `None` (pinned). Used
    /// for O(1) "is this DOF free?" lookups during sparse-pattern
    /// build and tangent assembly. Field name pairs with
    /// `free_dof_indices` (the forward direction: free idx → full
    /// DOF) — `full_to_free_idx` is the inverse direction.
    full_to_free_idx: Vec<Option<usize>>,
    /// Symbolic factor of the free-DOF Hessian sparsity pattern, built
    /// once from element-vertex incidence per Decision J. Per-iter
    /// numeric refactor consumes a `clone()` of this (cheap — faer
    /// 0.24 wraps the symbolic in `Arc` internally).
    symbolic: SymbolicLlt<usize>,
    /// Total DOF count (`3 * n_vertices`), cached for slice indexing.
    n_dof: usize,
    /// Free DOF count (`free_dof_indices.len()`), cached.
    n_free: usize,
}

impl<M, E, Msh, C, const N: usize, const G: usize> CpuNewtonSolver<M, E, Msh, C, N, G>
where
    M: Material,
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    /// Assemble a solver from its material, element, mesh, contact,
    /// integration configuration, and boundary conditions.
    ///
    /// `Box<dyn Solver<Tape = CpuTape>>` is the intended public handle;
    /// direct access to the concrete type is only needed for
    /// monomorphized benches (Phase E+). Phase 2 commit 4a populates
    /// an assembly cache (`element_geometries`, `mass_per_dof`,
    /// `free_dof_indices`, `dof_to_free`, `symbolic`, `n_dof`,
    /// `n_free`) at construction time; commit 4b's `solve_impl` /
    /// `factor_at_position` / `armijo_backtrack` will consume the
    /// cache. `const fn` was dropped here — the cache build runs
    /// `mesh.tet_vertices`, sparse-pattern construction, and faer's
    /// symbolic factorization, none of which are const-evaluable.
    //
    // expect_used + panic justifications:
    //   • Singular reference Jacobian = malformed rest mesh, programmer
    //     bug at construction time.
    //   • SymbolicLlt failure = solver-pattern build is wrong (impossible
    //     for any valid mesh + Dirichlet set), programmer bug.
    //
    // Lint allows: same Mesh-trait-API tax + Tet4 4-node iteration +
    // per-element grad/grad_generic similar-name pair as the assembly
    // methods below.
    #[must_use]
    #[allow(
        clippy::expect_used,
        clippy::too_many_lines,
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    pub fn new(
        material: M,
        element: E,
        mesh: Msh,
        contact: C,
        config: SolverConfig,
        boundary_conditions: BoundaryConditions,
    ) -> Self {
        assert!(
            N == 4,
            "Phase 2 solver is pinned to Tet4 (N=4) per scope §3 Decision A; got N={N}"
        );

        let n_tets = mesh.n_tets();
        let n_vertices = mesh.n_vertices();
        let n_dof = 3 * n_vertices;

        // 0. BoundaryConditions validation. Catch misconfigured scenes
        // at construction time rather than later as opaque
        // out-of-bounds panics inside the cache build or first
        // step()-call. Three checks: (a) pinned vertex IDs are in
        // range, (b) loaded vertex IDs are in range, (c) no vertex
        // appears in both pinned and loaded sets (load on a
        // Dirichlet-clamped DOF is unphysical and would silently
        // overwrite f_ext on a vertex that never moves).
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

        // 1. Per-element reference geometry. Computed once; all per-iter
        // assembly reads from this cache rather than recomputing.
        let x_rest = mesh.positions();
        let mut element_geometries = Vec::with_capacity(n_tets);
        // Parametric gradients are constant for Tet4; pass centroid ξ.
        let grad_xi_n = element.shape_gradients(Vec3::new(0.25, 0.25, 0.25));
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
            // Result is SMatrix<f64, N, 3>; copy first 4 rows out to the
            // concrete 4-node matrix per Tet4 (N == 4 asserted above).
            let grad_x_n_generic: SMatrix<f64, N, 3> = grad_xi_n * j_0_inv;
            let mut grad_x_n = SMatrix::<f64, 4, 3>::zeros();
            for a in 0..4 {
                for j in 0..3 {
                    grad_x_n[(a, j)] = grad_x_n_generic[(a, j)];
                }
            }
            element_geometries.push(ElementGeometry { grad_x_n, volume });
        }

        // 2. Lumped per-DOF mass. For each element `e`, contribute
        // `ρ V_e / 4` to every DOF of every vertex of `e`. Vertices
        // shared by multiple elements accumulate.
        let mut mass_per_dof = vec![0.0; n_dof];
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

        // 3. Free-DOF index mapping. Walks vertices in natural order
        // (deterministic per scope §15 D-3 + Decision M); reuses the
        // `pinned_set` BTreeSet built at validation time (no HashMap
        // on numeric paths per Decision M).
        let mut free_dof_indices = Vec::with_capacity(n_dof);
        for v in 0..n_vertices as VertexId {
            if !pinned_set.contains(&v) {
                let v_idx = v as usize;
                free_dof_indices.push(3 * v_idx);
                free_dof_indices.push(3 * v_idx + 1);
                free_dof_indices.push(3 * v_idx + 2);
            }
        }
        let n_free = free_dof_indices.len();
        let mut full_to_free_idx: Vec<Option<usize>> = vec![None; n_dof];
        for (free_idx, &full_idx) in free_dof_indices.iter().enumerate() {
            full_to_free_idx[full_idx] = Some(free_idx);
        }

        // 4. Symbolic factor of the free-DOF Hessian sparsity pattern,
        // built from element-vertex incidence (Decision J). For each
        // element `e`, every (a, b) vertex pair contributes a 3×3
        // block at the (free_idx_a, free_idx_b) location, IF both `a`
        // and `b` are free. BTreeSet keyed by (col, row) gives sorted
        // column-major lower-triangle iteration without a HashMap.
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
        let pattern_triplets: Vec<Triplet<usize, usize, f64>> = triplet_set
            .iter()
            .map(|&(c, r)| Triplet::new(r, c, 1.0))
            .collect();
        let pattern_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(n_free, n_free, &pattern_triplets)
                .expect("malformed free-block triplet pattern");
        let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
            .expect("symbolic factorization of free-block pattern failed");

        Self {
            material,
            element,
            mesh,
            _contact: contact,
            config,
            boundary_conditions,
            element_geometries,
            mass_per_dof,
            free_dof_indices,
            full_to_free_idx,
            symbolic,
            n_dof,
            n_free,
        }
    }

    /// Reference-frame geometry: material-frame shape gradients
    /// (`4 × 3`), reference volume, and lumped per-DOF mass.
    //
    // Panics on degenerate reference Jacobian (det ≈ 0), which would
    // indicate a malformed rest mesh — a programmer error upstream, not
    // a valid θ-path condition.
    #[allow(clippy::expect_used)]
    fn reference_geometry(&self) -> (SMatrix<f64, 4, 3>, f64, f64) {
        assert!(
            N == 4,
            "skeleton solver is pinned to Tet4 (N=4) per scope §2, got N={N}"
        );
        let x_rest = self.mesh.positions();
        // J_0 columns = (X_1 - X_0, X_2 - X_0, X_3 - X_0). Reference-frame
        // Jacobian ∂X/∂ξ; for canonical decimeter tet this is L·I.
        let j_0 = Matrix3::from_columns(&[
            x_rest[1] - x_rest[0],
            x_rest[2] - x_rest[0],
            x_rest[3] - x_rest[0],
        ]);
        // Malformed rest mesh (collinear/coplanar vertices) is a skeleton
        // bug, not a θ-path failure — panic with a clear message.
        let j_0_inv = j_0
            .try_inverse()
            .expect("singular reference Jacobian — malformed rest mesh");
        let volume = j_0.determinant().abs() / 6.0;
        // Parametric gradients are constant for Tet4; pass centroid ξ.
        let grad_xi_n = self.element.shape_gradients(Vec3::new(0.25, 0.25, 0.25));
        // Chain rule: grad_X N_a = grad_ξ N_a · (∂ξ/∂X) = grad_ξ N_a · J_0⁻¹.
        // Result is SMatrix<f64, N, 3>; copy the first 4 rows out to the
        // concrete 4-node matrix the skeleton's assembly kernels expect
        // (N == 4 asserted above).
        let grad_x_n_generic: SMatrix<f64, N, 3> = grad_xi_n * j_0_inv;
        let mut grad_x = SMatrix::<f64, 4, 3>::zeros();
        for a in 0..4 {
            for j in 0..3 {
                grad_x[(a, j)] = grad_x_n_generic[(a, j)];
            }
        }
        let mass_total = self.config.density * volume;
        // Lumped nodal mass `m_i = ρ V / N_vertices`, shared across each
        // vertex's 3 spatial DOFs — every diagonal entry of the lumped
        // `M` matrix. Skeleton mesh is 4 vertices (N == 4 asserted above),
        // so the literal `4.0` avoids `usize as f64` on `mesh.n_vertices()`.
        // Phase B multi-tet will replace with a safe-converted count.
        let mass_per_dof = mass_total / 4.0;
        (grad_x, volume, mass_per_dof)
    }

    /// Assemble and factor the condensed `A_free` SPD tangent at a
    /// specific position `x` (post-Newton-convergence for the IFT
    /// adjoint). Re-uses the solve-path pattern but discards the solve
    /// — only the factor is needed.
    //
    // expect_used + panic: same rationale as `factor_and_solve_free_block`.
    // Fixed-shape pattern; SPD failure is scope §3 R-2 contract violation.
    #[allow(clippy::expect_used, clippy::panic)]
    fn factor_at_position(&self, x_final: &[f64; N_DOF], dt: f64) -> Llt<usize, f64> {
        let (grad_x_n, volume, mass_per_dof) = self.reference_geometry();
        let mass_over_dt2 = mass_per_dof / (dt * dt);
        let f = deformation_gradient(x_final, &grad_x_n);
        let tangent = self.material.tangent(&f);
        let a_free = assemble_free_tangent_block(&tangent, &grad_x_n, volume, mass_over_dt2);

        let pattern_triplets = free_block_pattern_triplets();
        let pattern_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(N_FREE, N_FREE, &pattern_triplets)
                .expect("malformed free-block triplet pattern");
        let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
            .expect("symbolic factorization of fixed free-block pattern failed");

        let mut triplets: Vec<Triplet<usize, usize, f64>> = Vec::with_capacity(6);
        for col in 0..N_FREE {
            for row in col..N_FREE {
                triplets.push(Triplet::new(row, col, a_free[(row, col)]));
            }
        }
        let a_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(N_FREE, N_FREE, &triplets)
                .expect("malformed condensed-tangent triplet list");
        Llt::<usize, f64>::try_new_with_symbolic(symbolic, a_mat.rb(), Side::Lower).unwrap_or_else(
            |err| {
                panic!(
                    "condensed tangent not SPD at x_final (IFT adjoint factor): \
                     {err:?}. Scope §3 R-2 asserts SPD on the skeleton θ-path."
                )
            },
        )
    }

    /// Free-DOF residual norm (scope §5 R-1 convergence criterion).
    fn free_residual_norm(r_full: &[f64; N_DOF]) -> f64 {
        r_full[FREE_OFFSET..FREE_OFFSET + N_FREE]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            .sqrt()
    }

    /// Inner solver: pure-function-of-θ Newton loop. Shared by `step`
    /// (which will push the tape in step 5) and `replay_step`.
    ///
    /// # Panics
    /// - Newton exceeds `config.max_newton_iter` iterations without
    ///   reaching `config.tol`.
    /// - Armijo backtracks exceed `config.max_line_search_backtracks`.
    /// - Tangent factorization fails SPD (scope §3 R-2 asserts SPD on
    ///   the θ-path; failure is a path-violation bug, not a fallback
    ///   trigger — faer `Lu` fallback lands when a real indefinite
    ///   case surfaces per scope §3).
    // expect_used: malformed pattern / symbolic-analysis failures at
    //   fixed-pattern 3×3 scale are programmer bugs, not runtime inputs.
    // panic: scope §3 R-2 contracts that the NH tangent stays SPD on the
    //   θ-path; a bail here is a contract violation, not a recoverable
    //   condition. Same for Newton non-convergence at R-1's 3-5-iter
    //   prediction — flagging via panic with a diagnostic message is the
    //   right failure mode for the skeleton. Lu fallback will land as a
    //   distinct branch if an indefinite case actually surfaces.
    #[allow(clippy::expect_used, clippy::panic)]
    fn solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<CpuTape> {
        assert!(
            x_prev.as_slice().len() == N_DOF,
            "x_prev must have {N_DOF} entries, got {}",
            x_prev.as_slice().len()
        );
        assert!(
            v_prev.as_slice().len() == N_DOF,
            "v_prev must have {N_DOF} entries, got {}",
            v_prev.as_slice().len()
        );
        assert!(dt > 0.0, "dt must be positive, got {dt}");

        let (grad_x_n, volume, mass_per_dof) = self.reference_geometry();
        let mass_over_dt2 = mass_per_dof / (dt * dt);
        let f_ext = external_force(theta);

        // Initial iterate at the previous position (zero initial guess
        // for the displacement increment; standard backward-Euler warm-
        // start). Scope §15 D-9 notes this is deterministic (no RNG).
        let mut x_curr: [f64; N_DOF] = [0.0; N_DOF];
        x_curr.copy_from_slice(x_prev.as_slice());
        let x_prev_arr: [f64; N_DOF] = x_curr;
        let mut v_prev_arr: [f64; N_DOF] = [0.0; N_DOF];
        v_prev_arr.copy_from_slice(v_prev.as_slice());

        // Symbolic factorization — built once per step (pattern is fixed
        // for 1 tet + fixed Dirichlet). Scope §11 S-3 pattern: symbolic
        // is an `Arc<SymbolicCholesky<I>>` so per-Newton clones are a
        // cheap refcount bump.
        let pattern_triplets = free_block_pattern_triplets();
        let pattern_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(N_FREE, N_FREE, &pattern_triplets)
                .expect("malformed free-block triplet pattern");
        let symbolic = SymbolicLlt::<usize>::try_new(pattern_mat.symbolic(), Side::Lower)
            .expect("symbolic factorization of fixed free-block pattern failed");

        for newton_iter in 0..self.config.max_newton_iter {
            let f = deformation_gradient(&x_curr, &grad_x_n);
            let piola = self.material.first_piola(&f);
            let f_int = assemble_internal_force(&piola, &grad_x_n, volume);
            let r_full = residual(
                &x_curr,
                &x_prev_arr,
                &v_prev_arr,
                &f_int,
                &f_ext,
                mass_over_dt2,
                dt,
            );
            let r_norm = Self::free_residual_norm(&r_full);

            if r_norm < self.config.tol {
                return NewtonStep::new_converged(x_curr.to_vec(), newton_iter, r_norm);
            }

            let tangent = self.material.tangent(&f);
            let a_free = assemble_free_tangent_block(&tangent, &grad_x_n, volume, mass_over_dt2);
            let delta_free =
                factor_and_solve_free_block(&a_free, &r_full, &symbolic, newton_iter, r_norm);
            x_curr = self.armijo_backtrack(
                &x_curr,
                &x_prev_arr,
                &v_prev_arr,
                &f_ext,
                &grad_x_n,
                volume,
                mass_over_dt2,
                dt,
                &delta_free,
                r_norm,
                newton_iter,
            );
        }

        panic!(
            "Newton failed to converge within {max_iter} iterations at \
             tol {tol:e}. Likely causes: θ drives system out of R-2's \
             SPD region, or spec §3 R-1's assumption of 3-5 iter convergence \
             from zero initial guess is wrong for this θ.",
            max_iter = self.config.max_newton_iter,
            tol = self.config.tol,
        );
    }

    // Armijo backtracking per scope §5 R-1: shrink α geometrically until
    // ‖r(x + α δ)‖ ≤ (1 - c₁ α) ‖r‖. Panic-on-stall matches the solve_impl
    // policy — scope §11 S-2 treats stall as a book-level finding, not a
    // recoverable error.
    #[allow(clippy::panic, clippy::too_many_arguments)]
    fn armijo_backtrack(
        &self,
        x_curr: &[f64; N_DOF],
        x_prev_arr: &[f64; N_DOF],
        v_prev_arr: &[f64; N_DOF],
        f_ext: &[f64; N_DOF],
        grad_x_n: &SMatrix<f64, 4, 3>,
        volume: f64,
        mass_over_dt2: f64,
        dt: f64,
        delta_free: &[f64; N_FREE],
        r_norm: f64,
        newton_iter: usize,
    ) -> [f64; N_DOF] {
        let mut alpha = 1.0;
        let mut trial_x = *x_curr;
        for _ in 0..=self.config.max_line_search_backtracks {
            for i in 0..N_FREE {
                trial_x[FREE_OFFSET + i] = x_curr[FREE_OFFSET + i] + alpha * delta_free[i];
            }
            let trial_f = deformation_gradient(&trial_x, grad_x_n);
            let trial_piola = self.material.first_piola(&trial_f);
            let trial_f_int = assemble_internal_force(&trial_piola, grad_x_n, volume);
            let trial_r = residual(
                &trial_x,
                x_prev_arr,
                v_prev_arr,
                &trial_f_int,
                f_ext,
                mass_over_dt2,
                dt,
            );
            let trial_norm = Self::free_residual_norm(&trial_r);
            if trial_norm <= alpha.mul_add(-ARMIJO_C1, 1.0) * r_norm {
                return trial_x;
            }
            alpha *= 0.5;
        }
        panic!(
            "Armijo line-search stalled at Newton iter {newton_iter} \
             (r_norm {r_norm:e}, final α {alpha:e}). Likely causes: \
             non-SPD tangent near solution (spec §3 R-2 violation), \
             or near-singular condensed system."
        );
    }

    // ── Phase 2 commit 4a: cache-using assembly methods. ──
    // Dead until commit 4b's substitution. Live for review now so the
    // 4b diff is a pure rewire, not a wire + design.

    /// Read θ via the load map and emit the global external-force
    /// vector (Decision L). All-`AxisZ` BC (Stage 1) treats `theta` as
    /// a length-1 magnitude broadcast to every loaded vertex's `+ẑ`
    /// DOF; all-`FullVector` BC (Stage 2) consumes `theta` as
    /// `[3 * n_loaded]` per-vertex traction triples. Mixed-axis
    /// scenes are out of Phase 2 scope per `LoadAxis` doc.
    //
    // Shape mismatches (BC contradicting θ length) panic — programmer
    // bug at scene wiring time, not runtime input.
    #[allow(clippy::panic, dead_code)]
    fn assemble_external_force(&self, theta: &Tensor<f64>, f_ext: &mut [f64]) {
        debug_assert!(
            f_ext.len() == self.n_dof,
            "f_ext output buffer length {} ≠ n_dof {}",
            f_ext.len(),
            self.n_dof,
        );
        f_ext.fill(0.0);
        let theta_slice = theta.as_slice();
        let loaded = &self.boundary_conditions.loaded_vertices;

        if loaded.is_empty() {
            assert!(
                theta_slice.is_empty(),
                "θ has length {} but BC has no loaded vertices",
                theta_slice.len(),
            );
            return;
        }

        let all_axis_z = loaded.iter().all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
        let all_full_vec = loaded
            .iter()
            .all(|(_, ax)| matches!(ax, LoadAxis::FullVector));
        assert!(
            all_axis_z || all_full_vec,
            "Mixed-axis loaded_vertices are out of Phase 2 scope; got {loaded:?}"
        );

        if all_axis_z {
            // Stage-1 broadcast: one θ scalar drives `+ẑ` on every loaded vertex.
            assert!(
                theta_slice.len() == 1,
                "AxisZ-loaded θ must have length 1 (broadcast magnitude), got {}",
                theta_slice.len(),
            );
            let mag = theta_slice[0];
            for &(vertex_id, _) in loaded {
                f_ext[3 * vertex_id as usize + 2] = mag;
            }
        } else {
            // Stage-2 per-vertex: θ supplies xyz for each loaded vertex in order.
            let expected = 3 * loaded.len();
            assert!(
                theta_slice.len() == expected,
                "FullVector-loaded θ must have length {expected} (3 × {}), got {}",
                loaded.len(),
                theta_slice.len(),
            );
            for (i, &(vertex_id, _)) in loaded.iter().enumerate() {
                let v = vertex_id as usize;
                f_ext[3 * v] = theta_slice[3 * i];
                f_ext[3 * v + 1] = theta_slice[3 * i + 1];
                f_ext[3 * v + 2] = theta_slice[3 * i + 2];
            }
        }
    }

    /// Walk every element, scatter per-vertex internal-force
    /// contributions into the global `f_int` buffer.
    ///
    /// `f_int` is zeroed inside; caller need not pre-clear. Reads
    /// `x_curr` (length `n_dof`), uses cached `element_geometries` and
    /// the solver's `material`.
    //
    // Lint allows: see assemble_free_hessian_triplets justification.
    #[allow(
        dead_code,
        clippy::cast_possible_truncation,
        clippy::needless_range_loop
    )]
    fn assemble_global_int_force(&self, x_curr: &[f64], f_int: &mut [f64]) {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(f_int.len() == self.n_dof);
        f_int.fill(0.0);
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let piola = self.material.first_piola(&f);
            // Per-vertex internal-force contribution `V · P · grad_X N_a`.
            for a in 0..4 {
                let v = verts[a] as usize;
                for i in 0..3 {
                    let mut sum = 0.0;
                    for j in 0..3 {
                        sum += piola[(i, j)] * geom.grad_x_n[(a, j)];
                    }
                    f_int[3 * v + i] += geom.volume * sum;
                }
            }
        }
    }

    /// Assemble the lower-triangle triplets of the free-DOF Hessian
    /// `A_free = M_free / Δt² + K_free(x_curr)` per Decision J.
    ///
    /// For each element + each (a, b) vertex pair, contributes the
    /// 3×3 block from `B_a^T 𝕔 B_b` (using BF-5's flattening
    /// convention) into the global free-DOF sparse matrix at
    /// `(free_idx_a, free_idx_b)` whenever both DOFs are free (looked
    /// up via `full_to_free_idx`). Diagonal mass added at the end.
    /// `BTreeMap` accumulates with deterministic (col, row) iteration
    /// order per Decision M.
    //
    // Lint allows: `as TetId` is the Mesh-trait-API tax (n_tets
    // returns usize, tet_vertices takes u32) — same as commit 2's
    // HandBuiltTetMesh. `for a/b in 0..4` iterates over Tet4's 4 nodes
    // by index (used for both `verts[a]` AND `geom.grad_x_n[(a, l)]`
    // — the lint flags the verts use only). `va`/`vb` similar-name
    // pair mirrors the (a, b) symmetry of the per-element block math.
    #[allow(
        dead_code,
        clippy::cast_possible_truncation,
        clippy::needless_range_loop,
        clippy::similar_names
    )]
    fn assemble_free_hessian_triplets(
        &self,
        x_curr: &[f64],
        dt: f64,
    ) -> Vec<Triplet<usize, usize, f64>> {
        debug_assert!(x_curr.len() == self.n_dof);
        let dt2_inv = 1.0 / (dt * dt);
        // (col, row) → accumulated value. BTreeMap for sorted iteration
        // (Decision M D-3); no HashMap on numeric paths.
        let mut acc: BTreeMap<(usize, usize), f64> = BTreeMap::new();

        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let tangent_9x9 = self.material.tangent(&f);

            for a in 0..4 {
                let va = verts[a] as usize;
                for b in 0..4 {
                    let vb = verts[b] as usize;
                    for i in 0..3 {
                        for j in 0..3 {
                            let row_full = 3 * va + i;
                            let col_full = 3 * vb + j;
                            if let (Some(row_free), Some(col_free)) = (
                                self.full_to_free_idx[row_full],
                                self.full_to_free_idx[col_full],
                            ) && row_free >= col_free
                            {
                                // (B_a^T 𝕔 B_b)[i,j] = Σ_{l,l'} (grad_X N_a)_l ·
                                //   𝕔[(i+3l), (j+3l')] · (grad_X N_b)_{l'}
                                let mut block = 0.0;
                                for l in 0..3 {
                                    for lp in 0..3 {
                                        block += geom.grad_x_n[(a, l)]
                                            * tangent_9x9[(i + 3 * l, j + 3 * lp)]
                                            * geom.grad_x_n[(b, lp)];
                                    }
                                }
                                *acc.entry((col_free, row_free)).or_insert(0.0) +=
                                    geom.volume * block;
                            }
                        }
                    }
                }
            }
        }

        // Mass diagonal: M_free / Δt² · I on (k, k).
        for k in 0..self.n_free {
            let mass_dof = self.mass_per_dof[self.free_dof_indices[k]];
            *acc.entry((k, k)).or_insert(0.0) += mass_dof * dt2_inv;
        }

        acc.into_iter()
            .map(|((c, r), v)| Triplet::new(r, c, v))
            .collect()
    }
}

impl<M, E, Msh, C, const N: usize, const G: usize> Solver for CpuNewtonSolver<M, E, Msh, C, N, G>
where
    M: Material,
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    type Tape = CpuTape;

    // expect_used: `solve_impl` is contracted to return `x_final` with
    //   exactly N_DOF entries (asserted at entry); the `try_into` here
    //   is a type-level coercion `&[f64]` → `&[f64; N_DOF]` whose failure
    //   mode is "solve_impl violated its own contract" — programmer bug,
    //   not runtime input.
    #[allow(clippy::expect_used)]
    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Snapshot θ's value off the tape so the forward Newton loop can
        // proceed in pure-tensor space. The tape's role is reverse-mode
        // bookkeeping; the primal solve is tape-free.
        let theta_tensor = tape.value_tensor(theta_var).clone();
        let mut step = self.solve_impl(x_prev, v_prev, &theta_tensor, dt);

        // IFT adjoint factor: re-assemble A at x_final (post-convergence)
        // and factor it. Scope §3 R-2 asserts SPD on the θ-path; factor
        // ownership pattern (I-3) verified in tests/invariant_3_factor.rs.
        let x_final_arr: [f64; N_DOF] = step
            .x_final
            .as_slice()
            .try_into()
            .expect("x_final must have exactly N_DOF entries");
        let factor = self.factor_at_position(&x_final_arr, dt);

        // Push `NewtonStepVjp` onto the tape with `theta_var` as parent.
        // The VJP owns the factor; `Tape::backward` feeds the scalar-or-
        // vector cotangent of `x_final` into `vjp` and we solve the
        // adjoint `A · λ = g_free` in place, contracting against the
        // Stage-1 ∂r/∂θ sparsity pattern. See `NewtonStepVjp::vjp`.
        let vjp = NewtonStepVjp::new(factor);
        let x_final_tensor = Tensor::from_slice(&step.x_final, &[N_DOF]);
        let x_final_var = tape.push_custom(&[theta_var], x_final_tensor, Box::new(vjp));
        step.x_final_var = Some(x_final_var);
        step
    }

    fn replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Pure-function counterpart; no tape mutation. At skeleton scale
        // this is the same primal solve — Phase-E checkpoint replay will
        // diverge by reading a stored primal instead of re-solving.
        // `x_final_var` stays `None` — no tape means no Var.
        self.solve_impl(x_prev, v_prev, theta, dt)
    }

    fn current_dt(&self) -> f64 {
        self.config.dt
    }

    fn convergence_tol(&self) -> f64 {
        self.config.tol
    }
}

// ── free functions (assembly kernels) ──────────────────────────────────

/// Per-element deformation gradient `F_ij = Σ_a x_{a,i} · ∂N_a/∂X_j`
/// (direct form).
///
/// Takes 12 element-local position values (vertex-major, xyz-inner).
/// At rest (`x_a = X_a`), returns the identity. Used by both the old
/// `solve_impl` path (which sees a `[f64; N_DOF]` slice — type-
/// compatible since `N_DOF == 12`) and the new cache-using methods
/// (which pass element-extracted `[f64; 12]` directly).
fn deformation_gradient(x_elem: &[f64; 12], grad_x_n: &SMatrix<f64, 4, 3>) -> Matrix3<f64> {
    let mut f = Matrix3::zeros();
    for a in 0..4 {
        for i in 0..3 {
            let x_ai = x_elem[3 * a + i];
            for j in 0..3 {
                f[(i, j)] += x_ai * grad_x_n[(a, j)];
            }
        }
    }
    f
}

/// Extract one element's 12 vertex-DOF values from the global
/// position vector. Vertex-major + xyz-inner layout: `x_elem[3 * a +
/// k] = x_full[3 * verts[a] + k]`.
#[allow(dead_code)]
fn extract_element_dof_values(x_full: &[f64], verts: &[VertexId; 4]) -> [f64; 12] {
    let mut x_elem = [0.0; 12];
    for a in 0..4 {
        let v = verts[a] as usize;
        x_elem[3 * a] = x_full[3 * v];
        x_elem[3 * a + 1] = x_full[3 * v + 1];
        x_elem[3 * a + 2] = x_full[3 * v + 2];
    }
    x_elem
}

/// Per-node internal force `f_int_a = V · P · grad_X N_a`, flattened to
/// 12-entry vertex-major + xyz-inner layout.
fn assemble_internal_force(
    piola: &Matrix3<f64>,
    grad_x_n: &SMatrix<f64, 4, 3>,
    volume: f64,
) -> [f64; N_DOF] {
    let mut f_int = [0.0; N_DOF];
    for a in 0..4 {
        for i in 0..3 {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += piola[(i, j)] * grad_x_n[(a, j)];
            }
            f_int[3 * a + i] = volume * sum;
        }
    }
    f_int
}

/// External force in 12-DOF layout. Scope §2 R-6 θ parameterization:
///
/// - **Stage 1** — `θ ∈ ℝ¹`, traction magnitude along `+ẑ` at `v_3`:
///   `f_ext[11] = θ[0]`, else `0`.
/// - **Stage 2** — `θ ∈ ℝ³`, full traction vector `(t_x, t_y, t_z)` at
///   `v_3`: `f_ext[9..12] = θ[0..3]`, else `0`.
///
/// DOFs 9, 10, 11 are `v_3`'s x, y, z under vertex-major layout. The
/// corresponding `∂r/∂θ` closed forms live in [`NewtonStepVjp::vjp`].
fn external_force(theta: &Tensor<f64>) -> [f64; N_DOF] {
    let theta_slice = theta.as_slice();
    let n = theta_slice.len();
    assert!(
        n == 1 || n == 3,
        "skeleton θ must have length 1 (Stage 1, +ẑ magnitude) or length 3 \
         (Stage 2, full force vector); got length {n}"
    );
    let mut f_ext = [0.0; N_DOF];
    if n == 1 {
        f_ext[FREE_OFFSET + 2] = theta_slice[0];
    } else {
        f_ext[FREE_OFFSET] = theta_slice[0];
        f_ext[FREE_OFFSET + 1] = theta_slice[1];
        f_ext[FREE_OFFSET + 2] = theta_slice[2];
    }
    f_ext
}

/// Full 12-DOF residual per scope §5 R-5: `r = (M/Δt²)·(x - x_prev -
/// Δt·v_prev) + f_int(x) - f_ext(θ)`.
//
// Mass is diagonal with the same `mass_over_dt2` on every DOF (lumped,
// per-vertex mass split to xyz).
fn residual(
    x_curr: &[f64; N_DOF],
    x_prev: &[f64; N_DOF],
    v_prev: &[f64; N_DOF],
    f_int: &[f64; N_DOF],
    f_ext: &[f64; N_DOF],
    mass_over_dt2: f64,
    dt: f64,
) -> [f64; N_DOF] {
    let mut r = [0.0; N_DOF];
    for i in 0..N_DOF {
        let x_hat = dt.mul_add(v_prev[i], x_prev[i]);
        r[i] = mass_over_dt2.mul_add(x_curr[i] - x_hat, f_int[i]) - f_ext[i];
    }
    r
}

/// Condensed 3×3 tangent block for `v_3`'s DOFs:
/// `A_33 = (m/Δt²)·I + V · B_3^T · 𝕔 · B_3`.
///
/// `B_a[(m + 3l), k] = δ_{mk} · (grad_X N_a)_l`, so
/// `(B_3^T · 𝕔 · B_3)[k, m] = Σ_{l,l'} (grad_X N_3)_l · 𝕔[(k + 3l), (m + 3l')] ·
/// (grad_X N_3)_{l'}` — exactly the `BF-5`-ratified flattening
/// convention `row = i + 3j, col = k + 3l` from [`Material::tangent`].
fn assemble_free_tangent_block(
    tangent_9x9: &SMatrix<f64, 9, 9>,
    grad_x_n: &SMatrix<f64, 4, 3>,
    volume: f64,
    mass_over_dt2: f64,
) -> SMatrix<f64, N_FREE, N_FREE> {
    let grad_3: [f64; 3] = [grad_x_n[(3, 0)], grad_x_n[(3, 1)], grad_x_n[(3, 2)]];
    let mut a = SMatrix::<f64, N_FREE, N_FREE>::zeros();
    for k in 0..3 {
        for m in 0..3 {
            let mut acc = 0.0;
            for l in 0..3 {
                for lp in 0..3 {
                    acc += grad_3[l] * tangent_9x9[(k + 3 * l, m + 3 * lp)] * grad_3[lp];
                }
            }
            a[(k, m)] = volume * acc;
        }
    }
    for k in 0..N_FREE {
        a[(k, k)] += mass_over_dt2;
    }
    a
}

/// Sparsity pattern for the condensed 3×3 SPD block — dense lower
/// triangle (6 entries). Values are placeholders (any nonzero works for
/// symbolic analysis; numeric refactor will overwrite).
fn free_block_pattern_triplets() -> Vec<Triplet<usize, usize, f64>> {
    let mut v = Vec::with_capacity(6);
    for col in 0..N_FREE {
        for row in col..N_FREE {
            v.push(Triplet::new(row, col, 1.0));
        }
    }
    v
}

/// Factor the condensed `A_free` (SPD) via faer `Llt` with a shared
/// symbolic pattern, then solve `A · δ = -r_free` in place. Returns the
/// Newton step `δ_free = -A⁻¹ · r_free` for `v_3`.
//
// expect_used + panic justifications:
//   • Triplet-list build is fixed-shape (3 rows × 3 cols, 6 lower-triangle
//     entries) — CreationError at this scale is a programmer bug.
//   • Llt factor failure maps to scope §3 R-2 SPD-contract violation;
//     panicking with a diagnostic residual-norm + iteration context is
//     the right failure mode for the skeleton.
#[allow(clippy::expect_used, clippy::panic)]
fn factor_and_solve_free_block(
    a_free: &SMatrix<f64, N_FREE, N_FREE>,
    r_full: &[f64; N_DOF],
    symbolic: &SymbolicLlt<usize>,
    newton_iter: usize,
    r_norm: f64,
) -> [f64; N_FREE] {
    // Lower-triangular triplets in sorted (col, row) order — deterministic
    // CSR build per scope §15 D-4.
    let mut triplets: Vec<Triplet<usize, usize, f64>> = Vec::with_capacity(6);
    for col in 0..N_FREE {
        for row in col..N_FREE {
            triplets.push(Triplet::new(row, col, a_free[(row, col)]));
        }
    }
    let a_mat: SparseColMat<usize, f64> =
        SparseColMat::try_new_from_triplets(N_FREE, N_FREE, &triplets)
            .expect("malformed condensed-tangent triplet list");
    let llt = Llt::<usize, f64>::try_new_with_symbolic(symbolic.clone(), a_mat.rb(), Side::Lower)
        .unwrap_or_else(|err| {
            panic!(
                "condensed tangent not SPD at Newton iter {newton_iter}, \
                 free residual norm {r_norm:e}: {err:?}. Scope §3 R-2 \
                 asserts SPD on the skeleton θ-path; faer Lu fallback \
                 lands when an indefinite case actually surfaces."
            )
        });
    // `rhs_buf` starts as -r_free; after solve in place it holds +δx_free.
    let mut rhs_buf: [f64; N_FREE] = [
        -r_full[FREE_OFFSET],
        -r_full[FREE_OFFSET + 1],
        -r_full[FREE_OFFSET + 2],
    ];
    let rhs_mat: MatMut<'_, f64> = MatMut::from_column_major_slice_mut(&mut rhs_buf, N_FREE, 1);
    llt.solve_in_place_with_conj(Conj::No, rhs_mat);
    rhs_buf
}

#[cfg(test)]
mod tests {
    //! Phase 2 commit 4a.1 — `BoundaryConditions` validation tests.
    //!
    //! `CpuNewtonSolver::new` validates BC against mesh dimensions
    //! and the no-overlap-between-pinned-and-loaded contract before
    //! the cache build runs. Three `#[should_panic]` cases here cover
    //! the three validation branches; happy-path BC is covered by the
    //! existing seven integration tests in `tests/`.

    use crate::contact::NullContact;
    use crate::material::NeoHookean;
    use crate::mesh::SingleTetMesh;
    use crate::readout::{BoundaryConditions, LoadAxis};
    use crate::solver::{CpuNewtonSolver, SolverConfig};
    use crate::{SkeletonSolver, element::Tet4};

    fn build(bc: BoundaryConditions) -> SkeletonSolver {
        CpuNewtonSolver::new(
            NeoHookean::from_lame(1e5, 4e5),
            Tet4,
            SingleTetMesh::new(),
            NullContact,
            SolverConfig::skeleton(),
            bc,
        )
    }

    #[test]
    #[should_panic(expected = "pinned_vertices contains vertex ID 99")]
    fn pinned_vertex_out_of_range_panics() {
        let bc = BoundaryConditions {
            pinned_vertices: vec![0, 1, 2, 99],
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        };
        let _ = build(bc);
    }

    #[test]
    #[should_panic(expected = "loaded_vertices contains vertex ID 42")]
    fn loaded_vertex_out_of_range_panics() {
        let bc = BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            loaded_vertices: vec![(42, LoadAxis::AxisZ)],
        };
        let _ = build(bc);
    }

    #[test]
    #[should_panic(expected = "which is also in pinned_vertices")]
    fn loaded_vertex_overlapping_pinned_panics() {
        let bc = BoundaryConditions {
            pinned_vertices: vec![0, 1, 2, 3],
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        };
        let _ = build(bc);
    }
}
