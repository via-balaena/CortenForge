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
//! Solve path: a nested-dissection-ordered symbolic Cholesky
//! (the private `ordering` module) alongside faer `SymbolicLu::try_new`, both built once
//! per SOLVER CONSTRUCTION and cached for its lifetime — `step` never
//! rebuilds them, and the ordering's break-even in `ordering` is priced
//! on exactly that schedule
//! (one symbolic factor per algorithm; both share the
//! same element-vertex sparsity pattern with `Side::Lower` and full
//! reflection respectively), then a numeric
//! `OrderedLlt` plus `solve_in_place_with_conj`
//! per Newton iteration. A2 LU fallback
//! engages on `LltError::Numeric(NonPositivePivot)`: the helper
//! `factor_free_tangent` symmetrizes the lower-tri triplets to full
//! and factors via `Lu` against the cached `SymbolicLu`. The A2 change
//! left the happy path bit-identical to the pre-A2 Llt-only code
//! (scope §11 S-3 Round-1-verified API shape preserved).
//!
//! ⚠ That A2 invariant is about A2, and does not extend forwards: the
//! nested-dissection ordering DOES move the last bits of any solve
//! above `ordering`'s size threshold, because it changes the order
//! the factorization accumulates in. The **assembled** tangent is
//! untouched — every bit-equality claim in `assembly.rs` is about the
//! triplets going in, and those are unchanged. What moved is the
//! factorization of them. See the `ordering` module's bit-exactness note.
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

use faer::sparse::linalg::solvers::SymbolicLu;
use nalgebra::SMatrix;

use crate::Vec3;
use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::{Element, RestValidity};
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
mod ordering;
pub mod reduced;
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
/// point consumers only: the F-bar assembler, the validity gate's
/// `max_stretch_deviation` slot, and the lumped-mass volume — none of which the
/// multi-Gauss-point forward stiffness touches. (Rung 7 repointed the material
/// adjoint off this cache onto the per-GP [`GaussGeometry`], so no adjoint reads
/// `ElementGeometry`; the validity gate's `inversion` slot was likewise EXTENDED
/// there, since orientation has to hold where the material is evaluated — it now
/// reads both caches, because all four Stroud points are interior and cannot see
/// a fold confined to the corner region.)
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
/// and only `grad_x_n` differs per point (`∇_ξN(ξ_q)` is linear in `ξ`). A
/// **curved** higher-order element (midsides moved off the edge midpoints)
/// instead varies `detJ(ξ_q)` per point too — the genuine isoparametric
/// Jacobian built by
/// [`curved_gauss_geometry`](super::construct); this cache holds those per-point
/// weights when `construct` selects the curved path.
///
/// - **Tet4** monomorphizes to `(N, G) = (4, 1)`: the single centroid point,
///   whose pair is bit-identical to the matching [`ElementGeometry`] fields.
/// - **Tet10** is `(10, 4)`: four Stroud points, each with its own
///   `SMatrix<f64, 10, 3>` gradient — sharing one constant `|detJ|` when
///   straight-edged, or a per-point `|detJ(ξ_q)|` when curved.
///
/// Consumed by the forward stiffness kernels
/// (`assemble_global_int_force` / `assemble_free_hessian_triplets` /
/// `internal_force_tangent_matvec`), with one exception: that assembly's F-bar
/// branch evaluates a patch-modified `F*` built from [`ElementGeometry`] instead.
/// That branch is Tet4-only, where the two caches hold the same tensor.
///
/// ⚠ **No longer read by the step-boundary inversion gate.**
/// `CpuNewtonSolver::check_orientation` certifies `det F > 0` over the whole element
/// from node coordinates, which is a strictly stronger statement than anything the
/// `G` cached points could support. Held alongside [`ElementGeometry`] rather than
/// replacing it so the Tet4-flavored single-point consumers above — F-bar
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
    /// rest volume. Feeds the Tet4-flavored single-point consumers (F-bar, the
    /// validity gate's `max_stretch_deviation` slot, and lumped mass); the forward
    /// stiffness and the material adjoint (since rung 7) read
    /// [`Self::gauss_geometries`] as well. ⚠ The gate's `inversion` slot reads
    /// NEITHER — it certifies from node coordinates.
    element_geometries: Vec<ElementGeometry>,
    /// One entry per mesh tet — the per-Gauss-point stiffness geometry
    /// (`(grad_x_n, weight)` × `G`) the multi-Gauss-point forward kernels
    /// integrate over (Tet10 ladder rung 4). For Tet4 `(G = 1)` its single pair
    /// matches `element_geometries` bit-for-bit.
    gauss_geometries: Vec<GaussGeometry<N, G>>,
    /// Every element whose **rest** configuration [`Element::certify_orientation`]
    /// did not certify, paired with its verdict, ascending by tet id — normally
    /// EMPTY, which is why it is a sparse list rather than one entry per tet.
    ///
    /// ⚠ **Every one, not just the lowest-numbered.** An earlier revision kept only
    /// the first, on the reasoning that the gate is first-violator-wins in ascending
    /// order so no later defect could be the message. That holds only while EVERY
    /// material declares [`InversionHandling::RequireOrientation`]: a lower-numbered
    /// element whose material opts out is skipped by the gate, and the defect it
    /// consumed would then hide a higher-numbered one the gate does reach — which
    /// would pass on an element with an invalid rest. Today
    /// `InversionHandling` has exactly one variant, so the hole is unreachable; that
    /// enum's own docs anticipate more, and this is a third thing such a variant
    /// would otherwise arm. Ascending order makes the lookup a binary search.
    ///
    /// ★ Why the gate needs this, and why it is computed once. `det F = det J_def
    /// / det J_rest`, so certifying `det J_def > 0` over an element proves
    /// `det F > 0` there only where `det J_rest > 0` — and **at rest `det F ≡ 1`**,
    /// so a rest-folded element is invisible to every gate that reads `det F`
    /// alone, including the five-point check this replaced. The rest mesh cannot
    /// change over a solver's lifetime, so the verdict is taken in `new()` and the
    /// step boundaries pay nothing for it.
    ///
    /// ⚠ Held rather than raised at construction. Surfacing it through the gate
    /// keeps a malformed rest mesh on the existing
    /// [`SolverFailure::ValidityViolation`] channel — which `try_step` callers
    /// already handle by skipping the design — instead of adding a fourth panic
    /// to `new()`. It is still fail-closed: the first step boundary reports it.
    rest_orientation_defects: Vec<(usize, RestValidity)>,
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
    /// per Decision J, under a nested-dissection fill-reducing ordering
    /// (see the private `ordering` module for why faer's own AMD is not used).
    /// Per-iter numeric refactor consumes a `clone()` of this (cheap —
    /// it is an `Arc` refcount bump).
    symbolic: ordering::SharedSymbolicCholesky,
    /// The assembled free-DOF tangent's sparsity pattern in CSC form: row
    /// indices, column-major, ascending inside each column. Derived from the
    /// SAME `BTreeSet` `new()` feeds to
    /// [`build_symbolic_factors`](construct), kept rather than dropped so
    /// `assemble_free_hessian_triplets` can accumulate into a flat value
    /// buffer instead of rebuilding a `BTreeMap` on every Newton iteration.
    ///
    /// The column index is deliberately NOT stored: [`Self::pattern_col_ptr`]
    /// already encodes it, so a `(col, row)` pair would double both the resident
    /// size and the bytes each binary-search probe pulls into cache to compare
    /// eight of. Measured pattern density is ~21.6 entries per free DOF
    /// (37 071 at 1 944 free DOF → 563 571 at 26 460), so the pair form costs
    /// ~24 MB at 70k free DOF against ~12 MB here — a real saving, but a small
    /// one beside the roughly one gigabyte a session of that size holds
    /// resident. The primary reason is that the column was redundant state, not
    /// the bytes it cost.
    ///
    /// CSC order is column-major lower-triangle — the same
    /// order `BTreeMap` iterated in, so the emitted triplet vector is
    /// byte-identical to the pre-index path. That identity was verified by
    /// fingerprinting `x_final` across the revision boundary rather than by a
    /// permanent test (the reference is the deleted implementation, so no test
    /// can hold it); see [`assembly::FreeTangentAccumulator`] for the four
    /// flavours checked and their fingerprints.
    ///
    /// Valid because the pattern is a function of ELEMENT INCIDENCE and the
    /// free-DOF map alone — the same invariant
    /// [`Self::replace_contact`] already relies on, and the reason a
    /// self-collision contact model (which would widen it) is not shippable
    /// without revisiting both.
    pattern_rows: Vec<usize>,
    /// Column offsets into [`Self::pattern_rows`]: the rows of free column `c`
    /// are `pattern_rows[pattern_col_ptr[c]..pattern_col_ptr[c + 1]]`, ascending.
    /// Length `n_free + 1`. Turns a scatter lookup into a binary search over
    /// one column's rows (tens of entries) rather than the whole pattern.
    pattern_col_ptr: Vec<usize>,
    /// Symbolic factor of the same free-DOF Hessian pattern, in Lu
    /// shape (full matrix, no `Side`). Held alongside `symbolic` so
    /// the A2 LU fallback (Lu factorize when Llt hits a non-PD pivot)
    /// can run without rebuilding the symbolic factor at the failure
    /// site. Construction cost is one-shot at `new()` and small
    /// relative to the numeric factor; same `Arc`-internal sharing
    /// makes `clone()` cheap per fall-through.
    symbolic_lu: SymbolicLu<usize>,
    /// Numeric Cholesky factorizations performed by this solver since
    /// construction.
    ///
    /// The ordering in [`ordering`] costs a one-off SYMBOLIC charge per
    /// construction and repays a little on every NUMERIC factorization, so its
    /// break-even is denominated in this count — see
    /// `NESTED_DISSECTION_MIN_FREE_DOF`. Without a counter the threshold rests
    /// on a number nobody can regenerate, which is worse than one that is
    /// wrong: a wrong number can be caught.
    ///
    /// `Relaxed` because nothing orders against it; it is a diagnostic tally,
    /// not a synchronisation point. One relaxed increment against a
    /// factorization measured in milliseconds is unmeasurable overhead.
    factorizations: std::sync::atomic::AtomicUsize,
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

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// The solver's free-DOF map: full-DOF index for each free unknown, ascending.
    ///
    /// Exposed for [`reduced`]: a POD basis must span exactly the DOFs the solve treats
    /// as unknown, so the snapshot gatherer needs this map rather than reconstructing
    /// it from the boundary conditions (which would silently diverge if the auto-pin of
    /// orphan vertices, or the roller mask, ever changed shape).
    #[must_use]
    pub fn free_dof_indices(&self) -> &[usize] {
        &self.free_dof_indices
    }

    /// Lumped mass at each **free** DOF, in [`Self::free_dof_indices`] order.
    ///
    /// Exposed for [`reduced::Inner::Mass`], which needs the diagonal mass to form the
    /// energy inner product.
    #[must_use]
    pub fn mass_per_free_dof(&self) -> Vec<f64> {
        self.free_dof_indices
            .iter()
            .map(|&i| self.mass_per_dof[i])
            .collect()
    }
}

#[cfg(test)]
mod tests;
