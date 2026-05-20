//! `sim-soft` — soft-body FEM crate.
//!
//! Scope as of Phase 4 + Yeoh arc: backward-Euler hyperelastic FEM on
//! linear-tetrahedral (`Tet4`) meshes with per-element materials
//! sourced from a per-mesh [`MaterialField`] aggregator. Two material
//! impls today — [`NeoHookean`] (Phase 4 scaffold) and [`Yeoh`] (Yeoh
//! arc, F1+F2+F3+F4.0). Multi-element assembly (Phase 2), pure-Rust
//! SDF→tet bridge via BCC plus Labelle-Shewchuk Isosurface Stuffing
//! (Phase 3), bonded multi-material via spatial field aggregation
//! (Phase 4). Architecture follows the seven γ-locked API names from
//! [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md);
//! `MaterialField` is internal-API-shaped per Phase 4 scope memo
//! Decision M.
//!
//! Mesh + solver are generic over `M: Material` (default
//! [`NeoHookean`]); existing NH consumers compile unchanged via
//! default-type-param elision, Yeoh consumers write `M = Yeoh`
//! explicitly via the [`CpuTet4YeohSolver`] /
//! [`PenaltyRigidContactYeohSolver`] aliases.
//!
//! Forward roadmap: Phase 5 penalty contact, Phase E GPU port; Phase H
//! decorators (HGO anisotropy, viscoelasticity, thermal coupling), Tet10,
//! interface-aware refinement, IPC.

#![allow(
    // Placeholder bodies in `contact/null.rs`, `observable/basic.rs`,
    // `readout/reward_breakdown.rs`, and `differentiable/newton_vjp.rs`
    // (`time_adjoint`, `fd_wrapper`) are intentional `unimplemented!(...)`
    // for Phase 5 / Phase C / Phase G / Phase E+ surfaces that bolt onto
    // the trait shape but ship empty in Phase 4.
    clippy::unimplemented,
    // Placeholder fields like `CpuNewtonSolver::element: E` and
    // `BccLattice::position_of` ride the trait/struct shape forward for
    // upcoming phases (Tet10 element variant, future BCC accessors)
    // without bit-rotting the public surface in the meantime.
    dead_code,
    // Placeholder bodies panic via `unimplemented!`. Documenting it on
    // every method's `# Panics` section adds noise without information.
    clippy::missing_panics_doc
)]

pub mod autograd_ops;
pub mod contact;
pub mod differentiable;
pub mod element;
pub mod field;
pub mod material;
pub mod mesh;
pub mod observable;
pub mod readout;
pub mod sdf_bridge;
pub mod solver;
pub mod viz;

pub use autograd_ops::{DivOp, IndexOp};
pub use contact::{
    ActivePairsFor, ContactGradient, ContactHessian, ContactModel, ContactPair, ContactPairReadout,
    NullContact, PenaltyRigidContact, RigidPlane, filter_pair_readouts_to_referenced,
};
pub use differentiable::{CpuDifferentiable, Differentiable, NewtonStepVjp, TapeNodeKey};
pub use element::{Element, Tet4};
pub use field::{BlendedScalarField, ConstantField, Field, LayeredScalarField};
pub use material::{
    ConstructionSource, InversionHandling, Material, MaterialField, MaterialFieldKind,
    MeasuredMaterialError, NeoHookean, ShoreInterpolationError, ShoreReading, SiliconeMaterial,
    ValidityDomain, Yeoh,
};
pub use mesh::{
    HandBuiltTetMesh, Mesh, MeshAdjacency, QualityMetrics, SingleTetMesh, TetId, VertexId,
    referenced_vertices,
};
pub use observable::{BasicObservable, Observable, PressureField, StressField, TemperatureField};
pub use readout::{
    BoundaryConditions, EditResult, ForwardMap, GradientEstimate, LAYERED_SPHERE_BBOX_HALF_EXTENT,
    LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_INNER_OUTER, LAYERED_SPHERE_R_OUTER,
    LAYERED_SPHERE_R_OUTER_INNER, LoadAxis, ResidualCorrections, RewardBreakdown, RewardWeights,
    SceneInitial, SkeletonForwardMap, SoftScene, pick_vertices_by_predicate,
};
pub use sdf_bridge::{
    Aabb3, DifferenceSdf, MeshingError, MeshingHints, Sdf, SdfMeshedTetMesh, SphereSdf,
};
pub use solver::{
    CpuNewtonSolver, CpuTape, LmConfig, NewtonStep, SaturationPolicy, Solver, SolverConfig,
    SolverFailure,
};
pub use viz::{
    Plane, VizError, boundary_surface, design_scene, design_scene_deformed, design_slab_cut,
    design_surface, design_surface_deformed, slab_cut, slab_cut_deformed,
};

/// Three-component column vector in world space, `f64`. Matches
/// `nalgebra::Vector3<f64>` so per-element locals can plug into
/// nalgebra's dense small-matrix operations.
pub type Vec3 = nalgebra::Vector3<f64>;

/// CPU backward-Euler Newton solver pinned to `Tet4` + `NullContact`
/// + `NeoHookean`, generic over the mesh impl.
///
/// The constitutive law is fixed at [`NeoHookean`] for this alias —
/// per-tet instances live on the mesh and are read at the assembly
/// hot points via `mesh.materials()`. The underlying
/// [`solver::CpuNewtonSolver`] is generic over `M: Material` (default
/// NH); the [`CpuTet4YeohSolver`] alias picks the Yeoh slot.
///
/// Phase 2's canonical solver type for any hand-built tet scene;
/// specialize via the `Msh` parameter (e.g.,
/// `CpuTet4NHSolver<HandBuiltTetMesh>` for the multi-tet gate scenes,
/// `CpuTet4NHSolver<SingleTetMesh>` for the 1-tet skeleton).
pub type CpuTet4NHSolver<Msh> =
    solver::CpuNewtonSolver<element::Tet4, Msh, contact::NullContact, material::NeoHookean, 4, 1>;

/// Yeoh-flavored sibling of [`CpuTet4NHSolver`]. Specialize via the
/// `Msh` parameter (typically `SdfMeshedTetMesh<Yeoh>` per arc
/// memo D10).
pub type CpuTet4YeohSolver<Msh> =
    solver::CpuNewtonSolver<element::Tet4, Msh, contact::NullContact, material::Yeoh, 4, 1>;

/// Walking-skeleton solver alias — `CpuTet4NHSolver` over `SingleTetMesh`.
///
/// Concrete backward-Euler Newton with `NeoHookean` + `Tet4` +
/// `SingleTetMesh` + `NullContact`. One hot-path monomorphization, kept
/// as a stable alias for the walking-skeleton tests post-Phase-2 —
/// same behavior as before, now expressed as a [`CpuTet4NHSolver`]
/// flavor per Phase 2 scope memo Decision H.
pub type SkeletonSolver = CpuTet4NHSolver<mesh::SingleTetMesh>;

/// Contact-aware sibling of [`CpuTet4NHSolver`] — pinned to
/// [`PenaltyRigidContact`] instead of [`NullContact`], generic over
/// the mesh impl.
///
/// Penalty-active invariant tests (`tests/penalty_compressive_block.rs`,
/// `tests/hertz_sphere_plane.rs`, `tests/contact_drop_rest.rs`) drive
/// this flavor; walking-skeleton plus Phase 2/3/4 invariants continue
/// to use [`CpuTet4NHSolver`] (NullContact-pinned). Both flavors live
/// parallel: penalty is a stepping stone — a future contact upgrade
/// replaces penalty as the production baseline (the book's "no penalty
/// even as a baseline" commitment is narrowed to "no penalty as a
/// *production* baseline" — see
/// `docs/studies/soft_body_architecture/src/40-contact/00-why-ipc/00-penalty.md`).
///
/// Specialize via the `Msh` parameter — e.g.,
/// `PenaltyRigidContactSolver<HandBuiltTetMesh>` for the
/// compressive-block scene, `PenaltyRigidContactSolver<SdfMeshedTetMesh>`
/// for sphere-on-plane / drop-and-rest scenes.
pub type PenaltyRigidContactSolver<Msh> = solver::CpuNewtonSolver<
    element::Tet4,
    Msh,
    contact::PenaltyRigidContact,
    material::NeoHookean,
    4,
    1,
>;

/// Yeoh-flavored sibling of [`PenaltyRigidContactSolver`]. Row 23 (the
/// load-bearing F4 consumer of the Yeoh arc) builds against this alias.
pub type PenaltyRigidContactYeohSolver<Msh> =
    solver::CpuNewtonSolver<element::Tet4, Msh, contact::PenaltyRigidContact, material::Yeoh, 4, 1>;
