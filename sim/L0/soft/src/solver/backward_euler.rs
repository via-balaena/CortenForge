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
use crate::material::{InversionHandling, Material};
use crate::mesh::{Mesh, TetId, VertexId};
use crate::readout::{BoundaryConditions, LoadAxis};

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
    /// Body-force gravitational acceleration along `+ẑ` (`m/s²`).
    /// Pass a negative value for "downward" gravity (e.g. `-9.81`).
    /// Phase 5 commit 10 wiring per `phase_5_penalty_contact_scope.md` §1
    /// V-5; scalar (not `Vec3`) form preserves [`Self::skeleton`]'s
    /// `const fn` signature. Default `0.0` keeps the pre-Phase-5
    /// regression net bit-equal — `assemble_external_force` short-
    /// circuits the body-force scatter when this is exactly zero.
    pub gravity_z: f64,
}

impl SolverConfig {
    /// Scope §2 defaults for the walking-skeleton scene: `dt = 1e-2`,
    /// `tol = 1e-10` (five digits below gradcheck's 1e-5 bar),
    /// `density = 1030` (silicone-class), up to 10 Newton iterations +
    /// 20 backtracks per iteration. `gravity_z = 0` per Phase 5 commit
    /// 10 — V-5 opts in by mutating the field on a constructed config
    /// (mirrors V-3 / V-3a's `STATIC_DT` bumping pattern).
    #[must_use]
    pub const fn skeleton() -> Self {
        Self {
            dt: 1e-2,
            tol: 1e-10,
            density: 1030.0,
            max_newton_iter: 10,
            max_line_search_backtracks: 20,
            gravity_z: 0.0,
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
/// Five generic parameters: element `E<N, G>`, mesh `Msh`, contact `C`,
/// and const-generic `(N, G)` for element shape. The constitutive law
/// is fixed at `NeoHookean` per Phase 4 scope memo Decision G
/// (monomorphization); per-tet `NeoHookean` instances live on the mesh
/// and are read at the assembly hot points via `self.mesh.materials()`.
/// Monomorphized per skeleton type alias `SkeletonSolver`.
pub struct CpuNewtonSolver<E, Msh, C, const N: usize, const G: usize>
where
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    element: E,
    mesh: Msh,
    // Read once per Newton iter from `assemble_global_int_force` and
    // `assemble_free_hessian_triplets` (scope memo Decision H —
    // per-iter active-pair recompute). `NullContact`'s zero-stubs
    // preserve pre-Phase-5 numerics on non-contact scenes; V-1
    // (commit 7) is the regression-net spine.
    contact: C,
    config: SolverConfig,
    boundary_conditions: BoundaryConditions,

    // ── Assembly cache, populated by `new()`. ──
    // Read by `solve_impl` / `factor_at_position` / `armijo_backtrack`
    // and the assembly methods below. Replaces the pre-Phase-2
    // hardcoded `N_DOF` / `N_FREE` / `FREE_OFFSET` constants and the
    // per-iter `reference_geometry` recomputation.
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

impl<E, Msh, C, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, N, G>
where
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    /// Assemble a solver from its element, mesh, contact, integration
    /// configuration, and boundary conditions. Per-tet `NeoHookean`
    /// instances are read from `mesh.materials()` at assembly time.
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
        // (deterministic per scope §15 D-3 + Decision M); uses
        // `effective_pinned` (user pins UNION auto-pinned orphans)
        // built at validation time. No HashMap on numeric paths per
        // Decision M.
        let mut free_dof_indices = Vec::with_capacity(n_dof);
        for v in 0..n_vertices as VertexId {
            if !effective_pinned.contains(&v) {
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
            n_dof,
            n_free,
        }
    }

    /// Check every per-tet [`Material::validity`] domain against the
    /// deformation gradient `F` evaluated at `x_curr` (Phase 4 commit
    /// 12, IV-7 per scope memo Decision Q).
    ///
    /// First-violator-wins: walks tets in ascending `tet_id` order,
    /// computes `F = Σ_a x_{a,i} · ∂N_a/∂X_j` per element, and panics
    /// at the first tet whose `F` falls outside the declared
    /// [`crate::ValidityDomain`]. The two slots checkable from `F`
    /// for every base [`Material`] impl Phase 4 ships are
    /// `inversion` (`det F ≤ 0` under
    /// [`InversionHandling::RequireOrientation`]) and
    /// `max_stretch_deviation` (max `|σ_i − 1|` over the three
    /// singular values `σ_i` of `F`). The other four
    /// [`crate::ValidityDomain`] slots are construction-time
    /// (`poisson_range`) or decorator-only (`temperature_range`,
    /// `strain_rate_range`, `max_rotation` infinite for the
    /// scalar-isotropic NH baseline) and not checked here.
    ///
    /// Diagnostic-only at the solver level — Decision K's "Newton
    /// hot path does not branch on diagnostic metadata" framing
    /// applies to the interface flag, not to validity; this check
    /// runs once per [`Solver::step`] call before the Newton loop
    /// starts (Decision Q "at step start"), and panics on first
    /// violation rather than degrading silently. The book Part 2
    /// §00 §02 prescription is a runtime warning; Decision Q
    /// upgrades to panic for Phase 4 fail-closed semantics.
    ///
    /// # Panics
    ///
    /// On first violator, with structured message
    /// `"validity violation at tet {id}: {slot} = {value:.3} ..."`
    /// where `{slot}` is one of `max_stretch_deviation` or
    /// `inversion`. The `{slot}` substring is the IV-7 contract per
    /// Decision Q — tests pin on it via `#[should_panic(expected =
    /// "max_stretch_deviation")]` etc.
    //
    // similar_names: `tet_id`/`tet` mirrors the assembly methods.
    // cast_possible_truncation: same Mesh-trait API tax as the
    // assembly methods.
    #[allow(clippy::similar_names, clippy::cast_possible_truncation)]
    fn check_validity_at_step_start(&self, x_curr: &[f64]) {
        debug_assert!(x_curr.len() == self.n_dof);
        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let validity = materials[tet_id].validity();

            // Inversion check first: a non-positive `det F` makes the
            // SVD-based stretch check below ambiguous (singular values
            // are non-negative), so the orientation slot is the
            // structurally prior gate. Programs that allow `det F <=
            // 0` declare a non-`RequireOrientation` inversion handler,
            // and Phase 4 has none — Phase H may add `Barrier` /
            // `OptIn` variants when an impl needs them.
            let det_f = f.determinant();
            if matches!(validity.inversion, InversionHandling::RequireOrientation) {
                assert!(
                    det_f > 0.0,
                    "validity violation at tet {tet_id}: inversion = det F = \
                     {det_f:.3} violates RequireOrientation handler (must be \
                     strictly positive). Phase 4 scope memo Decision Q \
                     fail-closed semantics."
                );
            }

            // Principal-stretch deviation: SVD `F = U Σ V^T` gives
            // singular values `σ_i` which are the principal stretches.
            // `max_i |σ_i - 1|` is the Part 2 §00 §02 stretch bound.
            // `f.svd_unordered(false, false)` skips U/V computation
            // (we only need singular values); cheap O(27) FLOPs per
            // tet. Singular values are non-negative; a reflection-only
            // F has σ_i ≥ 0 with `det F < 0`, already caught above.
            let svd = f.svd_unordered(false, false);
            let sigma = svd.singular_values;
            let max_dev = sigma
                .iter()
                .map(|s| (s - 1.0).abs())
                .fold(0.0_f64, f64::max);
            let bound = validity.max_stretch_deviation;
            assert!(
                max_dev <= bound,
                "validity violation at tet {tet_id}: max_stretch_deviation \
                 = {max_dev:.3} exceeds bound {bound:.3} (singular values \
                 of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                 Decision Q fail-closed semantics.",
                s0 = sigma[0],
                s1 = sigma[1],
                s2 = sigma[2],
            );
        }
    }

    /// Free-DOF residual norm (scope §5 R-1 convergence criterion).
    /// Reads only the entries at `self.free_dof_indices`; pinned-DOF
    /// residual entries are excluded from the convergence test.
    fn free_residual_norm(&self, r_full: &[f64]) -> f64 {
        debug_assert!(r_full.len() == self.n_dof);
        self.free_dof_indices
            .iter()
            .map(|&idx| r_full[idx] * r_full[idx])
            .sum::<f64>()
            .sqrt()
    }

    /// Re-factor the free-DOF Hessian at a specific position (post-
    /// Newton-convergence for the IFT adjoint). Re-uses the cached
    /// symbolic factor — no rebuild.
    //
    // expect_used + panic justifications:
    //   • Sparse-pattern construction at the cached n_free dimensions
    //     can only fail on duplicate triplet entries, which the
    //     BTreeMap accumulator in `assemble_free_hessian_triplets`
    //     prevents by construction.
    //   • Llt factor failure is a scope §3 R-2 SPD-contract violation
    //     on the θ-path; faer Lu fallback would be the right path if
    //     an indefinite case ever surfaces, but for the skeleton's
    //     θ-path it's a programmer bug.
    #[allow(clippy::expect_used, clippy::panic)]
    fn factor_at_position(&self, x_curr: &[f64], dt: f64) -> Llt<usize, f64> {
        let triplets = self.assemble_free_hessian_triplets(x_curr, dt);
        let a_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(self.n_free, self.n_free, &triplets)
                .expect("malformed condensed-tangent triplet list");
        Llt::<usize, f64>::try_new_with_symbolic(self.symbolic.clone(), a_mat.rb(), Side::Lower)
            .unwrap_or_else(|err| {
                panic!(
                    "condensed tangent not SPD at x_final (IFT adjoint factor): \
                     {err:?}. Scope §3 R-2 asserts SPD on the θ-path."
                )
            })
    }

    /// Solve `A_free · δ = -r_free` using the cached symbolic factor.
    /// Returns the Newton step `δ_free` of length `self.n_free`.
    ///
    /// `triplets` must be the lower-triangle of `A_free` in `(col, row)`-
    /// sorted order; produced by `assemble_free_hessian_triplets`.
    /// `r_full` is the full-DOF residual; the free-DOF subset is
    /// gathered via `self.free_dof_indices`.
    //
    // expect_used + panic: same SPD-contract / fixed-pattern rationale
    // as `factor_at_position` above (sparse-pattern build can only fail
    // on duplicate triplets, which the BTreeMap accumulator prevents;
    // Llt factor failure = scope §3 R-2 SPD violation, programmer bug
    // on the θ-path).
    #[allow(clippy::expect_used, clippy::panic)]
    fn factor_and_solve_free(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        r_full: &[f64],
        newton_iter: usize,
        r_norm: f64,
    ) -> Vec<f64> {
        let a_mat: SparseColMat<usize, f64> =
            SparseColMat::try_new_from_triplets(self.n_free, self.n_free, triplets)
                .expect("malformed condensed-tangent triplet list");
        let llt = Llt::<usize, f64>::try_new_with_symbolic(
            self.symbolic.clone(),
            a_mat.rb(),
            Side::Lower,
        )
        .unwrap_or_else(|err| {
            panic!(
                "condensed tangent not SPD at Newton iter {newton_iter}, \
                 free residual norm {r_norm:e}: {err:?}. Scope §3 R-2 \
                 asserts SPD on the skeleton θ-path; faer Lu fallback \
                 lands when an indefinite case actually surfaces."
            )
        });
        // RHS = -r_free, gathered from r_full via the free-DOF index map.
        let mut rhs: Vec<f64> = self
            .free_dof_indices
            .iter()
            .map(|&idx| -r_full[idx])
            .collect();
        let rhs_mat: MatMut<'_, f64> =
            MatMut::from_column_major_slice_mut(&mut rhs, self.n_free, 1);
        llt.solve_in_place_with_conj(Conj::No, rhs_mat);
        rhs
    }

    /// Inner solver: pure-function-of-θ Newton loop. Shared by `step`
    /// and `replay_step`.
    ///
    /// # Panics
    /// - Newton exceeds `config.max_newton_iter` iterations without
    ///   reaching `config.tol`.
    /// - Armijo backtracks exceed `config.max_line_search_backtracks`.
    /// - Tangent factorization fails SPD (scope §3 R-2 asserts SPD on
    ///   the θ-path; failure is a path-violation bug, not a fallback
    ///   trigger).
    //
    // panic: scope §3 R-1 (3-5-iter convergence prediction) + R-2
    // (SPD-contract on the θ-path) violations are book-level findings,
    // not runtime-recoverable conditions; faer Lu fallback would land
    // when an indefinite case actually surfaces.
    #[allow(clippy::panic)]
    fn solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<CpuTape> {
        assert!(
            x_prev.as_slice().len() == self.n_dof,
            "x_prev must have {} entries, got {}",
            self.n_dof,
            x_prev.as_slice().len(),
        );
        assert!(
            v_prev.as_slice().len() == self.n_dof,
            "v_prev must have {} entries, got {}",
            self.n_dof,
            v_prev.as_slice().len(),
        );
        assert!(dt > 0.0, "dt must be positive, got {dt}");

        // Initial iterate at the previous position (zero initial guess
        // for the displacement increment; standard backward-Euler warm-
        // start). Scope §15 D-9 notes this is deterministic (no RNG).
        let mut x_curr: Vec<f64> = x_prev.as_slice().to_vec();
        let x_prev_vec: Vec<f64> = x_prev.as_slice().to_vec();
        let v_prev_vec: Vec<f64> = v_prev.as_slice().to_vec();

        let mut f_ext = vec![0.0; self.n_dof];
        self.assemble_external_force(theta, &mut f_ext);

        // Decision Q validity check (Phase 4 commit 12, IV-7) — runs
        // once at step start against `x_curr = x_prev`, panics on the
        // first per-tet `Material::validity()` violation. Diagnostic
        // failure mode, not solver-affecting on the happy path.
        self.check_validity_at_step_start(&x_curr);

        let mut f_int = vec![0.0; self.n_dof];
        let mut r_full = vec![0.0; self.n_dof];

        for newton_iter in 0..self.config.max_newton_iter {
            self.assemble_global_int_force(&x_curr, &mut f_int);
            residual_into(
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_int,
                &f_ext,
                &self.mass_per_dof,
                dt,
                &mut r_full,
            );
            let r_norm = self.free_residual_norm(&r_full);

            if r_norm < self.config.tol {
                return NewtonStep::new_converged(x_curr, newton_iter, r_norm);
            }

            let triplets = self.assemble_free_hessian_triplets(&x_curr, dt);
            let delta_free = self.factor_and_solve_free(&triplets, &r_full, newton_iter, r_norm);
            x_curr = self.armijo_backtrack(
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_ext,
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

    /// Armijo backtracking per scope §5 R-1: shrink α geometrically
    /// until `‖r(x + α δ)‖ ≤ (1 - c₁ α) ‖r‖`. Updates only the free
    /// DOFs of `x` (looked up via `self.free_dof_indices`); pinned
    /// DOFs stay at their `x_curr` values.
    ///
    /// Panic-on-stall matches the `solve_impl` policy — scope §11 S-2
    /// treats stall as a book-level finding, not a recoverable error.
    //
    // panic: stall is a book-level finding, not a recoverable error
    // (scope §11 S-2). too_many_arguments: 8 inputs mirror the residual
    // formula's reads; bundling into a struct adds name-the-fields
    // ceremony for one caller with no readability gain.
    #[allow(clippy::panic, clippy::too_many_arguments)]
    fn armijo_backtrack(
        &self,
        x_curr: &[f64],
        x_prev_vec: &[f64],
        v_prev_vec: &[f64],
        f_ext: &[f64],
        dt: f64,
        delta_free: &[f64],
        r_norm: f64,
        newton_iter: usize,
    ) -> Vec<f64> {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(delta_free.len() == self.n_free);
        let mut alpha = 1.0;
        // trial_x is initialized from x_curr each iteration via the free-DOF
        // re-write below; pre-allocate the buffer once and reuse across
        // backtracks.
        let mut trial_x: Vec<f64> = x_curr.to_vec();
        let mut trial_f_int = vec![0.0; self.n_dof];
        let mut trial_r = vec![0.0; self.n_dof];

        for _ in 0..=self.config.max_line_search_backtracks {
            // Reset free DOFs to `x_curr + α · δ`; pinned DOFs are
            // unchanged from the initial copy.
            for (free_idx, &full_idx) in self.free_dof_indices.iter().enumerate() {
                trial_x[full_idx] = x_curr[full_idx] + alpha * delta_free[free_idx];
            }
            self.assemble_global_int_force(&trial_x, &mut trial_f_int);
            residual_into(
                &trial_x,
                x_prev_vec,
                v_prev_vec,
                &trial_f_int,
                f_ext,
                &self.mass_per_dof,
                dt,
                &mut trial_r,
            );
            let trial_norm = self.free_residual_norm(&trial_r);
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

    // ── Cache-using assembly methods (live since Phase 2 commit 4b). ──

    /// Read θ via the load map and emit the global external-force
    /// vector (Decision L). All-`AxisZ` BC (Stage 1) treats `theta` as
    /// a length-1 magnitude broadcast to every loaded vertex's `+ẑ`
    /// DOF; all-`FullVector` BC (Stage 2) consumes `theta` as
    /// `[3 * n_loaded]` per-vertex traction triples. Mixed-axis
    /// scenes are out of Phase 2 scope per `LoadAxis` doc.
    ///
    /// Phase 5 commit 10: `config.gravity_z` adds a body-force `m_v ·
    /// gravity_z` to every vertex's `+ẑ` DOF after the θ scatter.
    /// Default `gravity_z = 0` short-circuits the loop — bit-equal
    /// regression on V-1 / V-3a / V-3 (which set `gravity_z = 0`).
    //
    // Shape mismatches (BC contradicting θ length) panic — programmer
    // bug at scene wiring time, not runtime input.
    #[allow(clippy::panic)]
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
        } else {
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

        // Phase 5 commit 10: gravity body force `f_z += m_v · g_z` per
        // vertex. The exact-zero short-circuit preserves bit-equality
        // on the regression net (V-1 / V-3a / V-3 leave `gravity_z =
        // 0`). For loaded AxisZ vertices the gravity term adds onto
        // the θ-traction; orphan vertices are auto-pinned at `new()`'s
        // `effective_pinned` step so their zero `mass_per_dof` never
        // reaches a free DOF.
        if self.config.gravity_z != 0.0 {
            let n_vertices = self.n_dof / 3;
            for v in 0..n_vertices {
                f_ext[3 * v + 2] += self.mass_per_dof[3 * v + 2] * self.config.gravity_z;
            }
        }
    }

    /// Walk every element, scatter per-vertex internal-force
    /// contributions into the global `f_int` buffer.
    ///
    /// `f_int` is zeroed inside; caller need not pre-clear. Reads
    /// `x_curr` (length `n_dof`), cached `element_geometries`, the
    /// per-tet `NeoHookean` from `self.mesh.materials()` (Phase 4
    /// commit 5 — Newton hot path reads from the per-tet material
    /// cache per Part 7 §02 §00), and `self.contact` for active-pair
    /// gradient contributions (Phase 5 commit 5; scope memo
    /// Decision H — per-iter active-pair recompute).
    //
    // Lint allows: see assemble_free_hessian_triplets justification.
    #[allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]
    fn assemble_global_int_force(&self, x_curr: &[f64], f_int: &mut [f64]) {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(f_int.len() == self.n_dof);
        f_int.fill(0.0);
        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let piola = materials[tet_id].first_piola(&f);
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

        // Contact gradient contributions. Sign convention per scope
        // memo §6 R-5 lens (v): the gradient `∂E_pen/∂x` is scattered
        // as `+force` into f_int, mirroring the elastic case where
        // `∂Ψ_elastic/∂x` is the +`f_int` contribution above.
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → bit-equal to the pre-Phase-5 elastic-only path.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let g = self.contact.gradient(pair, &positions);
            for &(vid, force) in &g.contributions {
                let v = vid as usize;
                f_int[3 * v] += force.x;
                f_int[3 * v + 1] += force.y;
                f_int[3 * v + 2] += force.z;
            }
        }
    }

    /// Assemble the lower-triangle triplets of the free-DOF Hessian
    /// `A_free = M_free / Δt² + K_free(x_curr) + K_contact(x_curr)`
    /// per Decision J + Phase 5 commit 5.
    ///
    /// For each element + each (a, b) vertex pair, contributes the
    /// 3×3 block from `B_a^T 𝕔 B_b` (using BF-5's flattening
    /// convention) into the global free-DOF sparse matrix at
    /// `(free_idx_a, free_idx_b)` whenever both DOFs are free (looked
    /// up via `full_to_free_idx`). Contact Hessian contributions from
    /// `self.contact.hessian` (rank-1 symmetric `κ·n⊗n` blocks for
    /// `PenaltyRigidContact`; symmetry assumption matches the existing
    /// lower-triangle filter) scatter through the same gate between
    /// the elastic and mass-diagonal passes. Diagonal mass added at
    /// the end. `BTreeMap` accumulates with deterministic (col, row)
    /// iteration order per Decision M.
    //
    // Lint allows: `as TetId` is the Mesh-trait-API tax (n_tets
    // returns usize, tet_vertices takes u32) — same as commit 2's
    // HandBuiltTetMesh. `for a/b in 0..4` iterates over Tet4's 4 nodes
    // by index (used for both `verts[a]` AND `geom.grad_x_n[(a, l)]`
    // — the lint flags the verts use only). `va`/`vb` similar-name
    // pair mirrors the (a, b) symmetry of the per-element block math.
    #[allow(
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
        // `dt * dt` then `mass / dt2` matches the pre-commit-4 code's
        // mass-diagonal expression order (`mass_per_dof / (dt * dt)`).
        // Distinct from `mass * (1.0 / (dt * dt))` at the last bit — the
        // FP-equal form preserves bit-equality with the pre-Phase-2
        // 1-tet path, simplifying any future bisect.
        let dt2 = dt * dt;
        // (col, row) → accumulated value. BTreeMap for sorted iteration
        // (Decision M D-3); no HashMap on numeric paths.
        let mut acc: BTreeMap<(usize, usize), f64> = BTreeMap::new();

        let materials = self.mesh.materials();
        for (tet_id, geom) in self.element_geometries.iter().enumerate() {
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let tangent_9x9 = materials[tet_id].tangent(&f);

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

        // Contact Hessian contributions. Scatter through the same
        // free-DOF + lower-triangle filter as the elastic block above
        // (scope memo §6 R-5 lens (v) — sign-consistent with the
        // f_int gradient scatter in `assemble_global_int_force`).
        // `NullContact` returns an empty contributions Vec → empty
        // for-loops → acc unchanged → bit-equal Hessian.
        let positions = slice_to_vec3s(x_curr);
        let pairs = self.contact.active_pairs(&self.mesh, &positions);
        for pair in &pairs {
            let h = self.contact.hessian(pair, &positions);
            for &(row_vid, col_vid, block) in &h.contributions {
                for i in 0..3 {
                    for j in 0..3 {
                        let row_full = 3 * (row_vid as usize) + i;
                        let col_full = 3 * (col_vid as usize) + j;
                        if let (Some(row_free), Some(col_free)) = (
                            self.full_to_free_idx[row_full],
                            self.full_to_free_idx[col_full],
                        ) && row_free >= col_free
                        {
                            *acc.entry((col_free, row_free)).or_insert(0.0) += block[(i, j)];
                        }
                    }
                }
            }
        }

        // Mass diagonal: M_free / Δt² · I on (k, k).
        for k in 0..self.n_free {
            let mass_dof = self.mass_per_dof[self.free_dof_indices[k]];
            *acc.entry((k, k)).or_insert(0.0) += mass_dof / dt2;
        }

        acc.into_iter()
            .map(|((c, r), v)| Triplet::new(r, c, v))
            .collect()
    }
}

impl<E, Msh, C, const N: usize, const G: usize> Solver for CpuNewtonSolver<E, Msh, C, N, G>
where
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    type Tape = CpuTape;

    // expect_used: the loaded_free_xyz construction below `.expect`s on
    // `full_to_free_idx[loaded_dof]`, which BC validation guarantees is
    // `Some` (loaded ∩ pinned = ∅ asserted in `new()`).
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
        let factor = self.factor_at_position(&step.x_final, dt);

        // Push `NewtonStepVjp` onto the tape with `theta_var` as parent.
        // The VJP owns the factor; `Tape::backward` feeds the scalar-or-
        // vector cotangent of `x_final` into `vjp` and we solve the
        // adjoint `A · λ = g_free` in place, contracting against
        // (∂r/∂θ)_free per the per-stage closed forms in
        // `NewtonStepVjp::vjp`. Pre-resolve loaded vertices' xyz
        // free-DOF indices via `full_to_free_idx` so `vjp` doesn't
        // need solver-side metadata at backward-pass time.
        let loaded_free_xyz: Vec<[usize; 3]> = self
            .boundary_conditions
            .loaded_vertices
            .iter()
            .map(|&(vid, _)| {
                let v = vid as usize;
                [
                    self.full_to_free_idx[3 * v]
                        .expect("loaded vertex must be free (BC validation)"),
                    self.full_to_free_idx[3 * v + 1]
                        .expect("loaded vertex must be free (BC validation)"),
                    self.full_to_free_idx[3 * v + 2]
                        .expect("loaded vertex must be free (BC validation)"),
                ]
            })
            .collect();
        let stage_1 = self
            .boundary_conditions
            .loaded_vertices
            .iter()
            .all(|(_, ax)| matches!(ax, LoadAxis::AxisZ));
        let vjp = NewtonStepVjp::new(
            factor,
            self.n_dof,
            self.free_dof_indices.clone(),
            loaded_free_xyz,
            stage_1,
        );
        let x_final_tensor = Tensor::from_slice(&step.x_final, &[self.n_dof]);
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
/// Takes 12 element-local position values (vertex-major, xyz-inner)
/// from `extract_element_dof_values`. At rest (`x_a = X_a`), returns
/// the identity.
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

/// Full-DOF residual per scope §5 R-5: `r = (M/Δt²)·(x - x_prev -
/// Δt·v_prev) + f_int(x) - f_ext(θ)`, written into the caller-owned
/// `r` buffer.
///
/// `mass_per_dof` is the diagonal of the lumped mass matrix; for
/// vertices shared across multiple elements this exceeds `ρ V_e / 4`
/// of any single element. `mass / (dt * dt)` (rather than
/// `mass * (1.0 / (dt * dt))`) preserves the pre-Phase-2 1-tet
/// expression order at the last bit.
//
// 8 args mirrors the 8 input slices the residual formula reads from;
// bundling into a struct would add a name-the-fields ceremony for two
// callers with no readability gain.
#[allow(clippy::too_many_arguments)]
fn residual_into(
    x_curr: &[f64],
    x_prev: &[f64],
    v_prev: &[f64],
    f_int: &[f64],
    f_ext: &[f64],
    mass_per_dof: &[f64],
    dt: f64,
    r: &mut [f64],
) {
    debug_assert!(x_curr.len() == r.len());
    debug_assert!(x_prev.len() == r.len());
    debug_assert!(v_prev.len() == r.len());
    debug_assert!(f_int.len() == r.len());
    debug_assert!(f_ext.len() == r.len());
    debug_assert!(mass_per_dof.len() == r.len());
    let dt2 = dt * dt;
    for i in 0..r.len() {
        let x_hat = dt.mul_add(v_prev[i], x_prev[i]);
        let mass_over_dt2 = mass_per_dof[i] / dt2;
        r[i] = mass_over_dt2.mul_add(x_curr[i] - x_hat, f_int[i]) - f_ext[i];
    }
}

/// Convert a flat `[f64]` DOF buffer (length `3·N`, vertex-major +
/// xyz-inner) to a `Vec<Vec3>` of length `N`.
///
/// Bridges the solver's flat representation to
/// [`crate::contact::ContactModel`]'s `&[Vec3]` argument shape
/// (Phase 5 commit 5; trait surface unchanged per scope memo
/// Decision E). Allocates per call site — `assemble_global_int_force`
/// and `assemble_free_hessian_triplets` each build their own vec at
/// the contact-dispatch step. Cost analysis at scope memo Decision H:
/// negligible vs FEM assembly even at V-3 finest-level scale.
fn slice_to_vec3s(x_flat: &[f64]) -> Vec<Vec3> {
    debug_assert!(x_flat.len().is_multiple_of(3));
    let n = x_flat.len() / 3;
    let mut out = Vec::with_capacity(n);
    for v in 0..n {
        out.push(Vec3::new(
            x_flat[3 * v],
            x_flat[3 * v + 1],
            x_flat[3 * v + 2],
        ));
    }
    out
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
    use crate::material::MaterialField;
    use crate::mesh::SingleTetMesh;
    use crate::readout::{BoundaryConditions, LoadAxis};
    use crate::solver::{CpuNewtonSolver, SolverConfig};
    use crate::{SkeletonSolver, element::Tet4};

    fn build(bc: BoundaryConditions) -> SkeletonSolver {
        CpuNewtonSolver::new(
            Tet4,
            SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
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
