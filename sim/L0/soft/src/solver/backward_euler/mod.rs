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
//! Solve path: faer `SymbolicLlt::try_new` + `SymbolicLu::try_new`
//! once per `step`-call (one symbolic factor per algorithm; both share
//! the same element-vertex sparsity pattern with `Side::Lower` and full
//! reflection respectively), then `Llt::try_new_with_symbolic` +
//! `solve_in_place_with_conj` per Newton iteration. A2 LU fallback
//! engages on `LltError::Numeric(NonPositivePivot)`: the helper
//! `factor_free_tangent` symmetrizes the lower-tri triplets to full
//! and factors via `Lu` against the cached `SymbolicLu`. Happy path
//! stays bit-identical to the pre-A2 Llt-only code (scope §11 S-3
//! Round-1-verified API shape preserved).
//!
//! After convergence, `step` re-factors `A` at `x_final` via
//! `factor_at_position` and pushes `NewtonStepVjp` onto the tape with
//! `theta_var` as parent. The VJP solves the IFT adjoint `A · λ = g_free`
//! and contracts against `∂r/∂θ` — see `NewtonStepVjp` for the math.
//!
//! [r]: ../../../../../../docs/studies/soft_body_architecture/src/60-differentiability/02-implicit-function.md
//! [tet4]: ../../../../../../docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/00-tet4.md

// Forward-riding placeholders across the backward-Euler module: `CpuNewtonSolver::element`
// carries the (Tet10) element variant forward, and `factor.rs`'s `is_llt` / `is_lu`
// factorization-kind accessors + `factor_and_solve_free` alt-path ride the shape; not yet used.
#![allow(dead_code)]

use faer::sparse::linalg::solvers::{SymbolicLlt, SymbolicLu};
use nalgebra::SMatrix;

use crate::Vec3;
use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::readout::BoundaryConditions;

mod assembly;
mod config;
mod construct;
mod factor;
mod fbar;
mod helpers;
mod newton;
mod sensitivities;
mod trait_impl;

pub use config::{FrictionReactionGradients, FrictionVertexForce, SolverConfig};
pub(crate) use factor::FactoredFreeTangent;

/// Per-element **single-point** reference geometry (Phase 2 commit 4a).
///
/// `grad_x_n` is the constant-strain material-frame shape gradient
/// (`SMatrix<f64, 4, 3>`, one row per corner); `volume` is the
/// rest-configuration tet volume. For a Tet10 element this is the affine
/// corner block (the barycentric constant-strain gradient) — a *single-point*
/// proxy, NOT the element's real (linearly-varying) strain, which lives in the
/// per-Gauss-point [`GaussGeometry`]. It is read by the Tet4-flavored, single-
/// point consumers only: the F-bar assembler, the rung-7-guarded material
/// adjoint, the feasibility (validity) gate, and the lumped-mass volume — none
/// of which the multi-Gauss-point forward stiffness touches.
#[derive(Clone, Debug)]
struct ElementGeometry {
    grad_x_n: SMatrix<f64, 4, 3>,
    volume: f64,
}

/// Per-element **per-Gauss-point** stiffness geometry (Tet10 ladder rung 4).
///
/// Holds the `G` Gauss-point pairs `(grad_x_n, weight)`, where `grad_x_n` is
/// the material-frame shape gradient at that point (`SMatrix<f64, N, 3>`, one
/// row per node) and `weight = w_q · |detJ|`. For a **straight-edged** element
/// the isoparametric map is affine, so `|detJ|` is constant across the element
/// and only `grad_x_n` differs per point (`∇_ξN(ξ_q)` is linear in `ξ`); a
/// curved element (deferred rung 8) would additionally vary `detJ` per point.
///
/// - **Tet4** monomorphizes to `(N, G) = (4, 1)`: the single centroid point,
///   whose pair is bit-identical to the matching [`ElementGeometry`] fields.
/// - **Tet10** is `(10, 4)`: four Stroud points, each with its own
///   `SMatrix<f64, 10, 3>` gradient, sharing one constant `|detJ|`.
///
/// Consumed only by the forward stiffness kernels
/// (`assemble_global_int_force` / `assemble_free_hessian_triplets` /
/// `internal_force_tangent_matvec`). Held alongside [`ElementGeometry`] rather
/// than replacing it so the Tet4-flavored single-point consumers above — F-bar
/// especially — stay byte-identical and untouched.
#[derive(Clone, Debug)]
struct GaussGeometry<const N: usize, const G: usize> {
    /// The `G` Gauss points as `(material-frame shape gradient, weight)`.
    gauss: [(SMatrix<f64, N, 3>, f64); G],
}

/// CPU backward-Euler Newton solver.
///
/// Six generic parameters: element `E<N, G>`, mesh `Msh`, contact `C`,
/// material `M`, and const-generic `(N, G)` for element shape.
///
/// `M` defaults to [`crate::material::NeoHookean`] for back-compat with Phase 4 scope
/// memo Decision G's monomorphization. Yeoh consumers (row 23+) write
/// `M = Yeoh` explicitly, typically via the [`crate::CpuTet4YeohSolver`]
/// alias, and use a `Mesh<Yeoh>` impl such as
/// `SdfMeshedTetMesh<Yeoh>` per arc memo D10. Per-tet `M`
/// instances live on the mesh and are read at the assembly hot
/// points via `self.mesh.materials()`.
pub struct CpuNewtonSolver<
    E,
    Msh,
    C,
    M = crate::material::NeoHookean,
    const N: usize = 4,
    const G: usize = 1,
> where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    element: E,
    mesh: Msh,
    // Read once per Newton iter from `assemble_global_int_force` and
    // `assemble_free_hessian_triplets` (per-iter active-pair
    // recompute). `NullContact`'s zero-stubs preserve pre-penalty
    // numerics on non-contact scenes; `tests/contact_passthrough.rs`
    // is the regression-net spine.
    contact: C,
    config: SolverConfig,
    boundary_conditions: BoundaryConditions,

    // ── Assembly cache, populated by `new()`. ──
    // Read by `solve_impl` / `factor_at_position` / `armijo_backtrack`
    // and the assembly methods below. Replaces the pre-Phase-2
    // hardcoded `N_DOF` / `N_FREE` / `FREE_OFFSET` constants and the
    // per-iter `reference_geometry` recomputation.
    /// One entry per mesh tet — the single-point corner shape gradient and
    /// rest volume. Feeds the Tet4-flavored single-point consumers (F-bar,
    /// material adjoint, validity gate, lumped mass); the forward stiffness
    /// reads [`Self::gauss_geometries`] instead.
    element_geometries: Vec<ElementGeometry>,
    /// One entry per mesh tet — the per-Gauss-point stiffness geometry
    /// (`(grad_x_n, weight)` × `G`) the multi-Gauss-point forward kernels
    /// integrate over (Tet10 ladder rung 4). For Tet4 `(G = 1)` its single pair
    /// matches `element_geometries` bit-for-bit.
    gauss_geometries: Vec<GaussGeometry<N, G>>,
    /// Lumped per-DOF mass (`length n_dof`). For a linear (Tet4) element the
    /// entry for DOF `i` (vertex `v = i / 3`) is `Σ_e (ρ V_e / 4)` over every
    /// element `e` that contains `v` (Phase 2 reproduces the walking
    /// skeleton's "per-vertex mass = ρ `V_total` / 4" rule when every vertex
    /// sits in exactly one tet). For a higher-order element (Tet10, rung 3b)
    /// it is the HRZ (Hinton–Rock–Zienkiewicz) diagonal lump instead —
    /// `Σ_e ρ V_e · (∫N_i² / Σ_k ∫N_k²)` — positive on every node, where
    /// naive row-sum lumping goes negative on quadratic-tet corners.
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
    /// Symbolic factor of the free-DOF Hessian sparsity pattern (Llt
    /// shape, `Side::Lower`), built once from element-vertex incidence
    /// per Decision J. Per-iter numeric refactor consumes a `clone()`
    /// of this (cheap — faer 0.24 wraps the symbolic in `Arc`
    /// internally).
    symbolic: SymbolicLlt<usize>,
    /// Symbolic factor of the same free-DOF Hessian pattern, in Lu
    /// shape (full matrix, no `Side`). Held alongside `symbolic` so
    /// the A2 LU fallback (Lu factorize when Llt hits a non-PD pivot)
    /// can run without rebuilding the symbolic factor at the failure
    /// site. Construction cost is one-shot at `new()` and small
    /// relative to the numeric factor; same `Arc`-internal sharing
    /// makes `clone()` cheap per fall-through.
    symbolic_lu: SymbolicLu<usize>,
    /// Total DOF count (`3 * n_vertices`), cached for slice indexing.
    n_dof: usize,
    /// Free DOF count (`free_dof_indices.len()`), cached.
    n_free: usize,

    /// Nodal-patch topology for the F-bar volumetric-locking cure, built once
    /// at construction when `config.fbar` is set (else `None` → the plain
    /// per-element Tet4 assembly path, bit-equal to the pre-F-bar code). Holds
    /// the nodal rest volumes + vertex incidence the `J̄` average walks.
    fbar_cache: Option<fbar::FbarCache>,

    /// The rigid contact surface's **within-step tangential drift** `Δ_surf` —
    /// the displacement the (kinematic) rigid collider sweeps over the step,
    /// against which the smoothed-Coulomb friction measures the soft vertices'
    /// RELATIVE tangential slip: `u_T = Tⁿᵀ((x_v − xᵗ_v) − Δ_surf)`. Default
    /// `(0,0,0)` recovers PR1's one-way (static-collider) friction
    /// byte-identically; a non-zero drift lets a *moving* collider DRAG the soft
    /// body (the two-way grip a sliding device exerts on a held limb). Set via
    /// [`Self::with_friction_surface_drift`]. Uniform translation only for now
    /// (a single `Δ_surf` for all pairs); per-contact-point rotation (`ω×r`) is a
    /// future refinement. Only consulted when `config.friction_mu > 0`.
    friction_surface_drift: Vec3,

    /// Phantom — `M` only appears in the `Msh: Mesh<M>` and
    /// `C: ContactModel + ActivePairsFor<M>` bounds, not in any
    /// concrete field. The marker tells rustc the type parameter is
    /// intentionally type-only.
    _material: std::marker::PhantomData<M>,
}

#[cfg(test)]
mod tests;
