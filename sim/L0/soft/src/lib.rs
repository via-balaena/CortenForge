//! `sim-soft` — soft-body walking-skeleton scaffold.
//!
//! This crate is the compile-check pre-flight of the walking-skeleton
//! specification at `sim/docs/todo/soft_body_walking_skeleton_scope.md`
//! (tip `01633283`). Seven traits are defined against chassis `Tape` /
//! `VjpOp` and the γ-locked API names from
//! [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md).
//!
//! Every method body is `unimplemented!("skeleton phase 2")`. Phase B
//! fills them in one trait at a time per spec §7. This crate exists to
//! validate the paper design compiles before semantics land.

#![allow(
    // Scaffold bodies are `unimplemented!(...)` by design. Override lifted
    // in Phase B as each trait gets a real implementation.
    clippy::unimplemented,
    // Skeleton fields (e.g. `SingleTetMesh::vertices`) are held for spec
    // fidelity per §14; Phase B's `Mesh::positions` will read them.
    dead_code,
    // Stub bodies panic via `unimplemented!`. Documenting this on every
    // method adds noise without information.
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

pub use autograd_ops::{DivOp, IndexOp};
pub use contact::{ContactGradient, ContactHessian, ContactModel, ContactPair, NullContact};
pub use differentiable::{CpuDifferentiable, Differentiable, NewtonStepVjp, TapeNodeKey};
pub use element::{Element, Tet4};
pub use field::{BlendedScalarField, ConstantField, Field, LayeredScalarField};
pub use material::{InversionHandling, Material, MaterialField, NeoHookean, ValidityDomain};
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
pub use solver::{CpuNewtonSolver, CpuTape, NewtonStep, Solver, SolverConfig};

/// Three-component column vector in world space, `f64`. Matches
/// `nalgebra::Vector3<f64>` so per-element locals can plug into
/// nalgebra's dense small-matrix operations.
pub type Vec3 = nalgebra::Vector3<f64>;

/// CPU backward-Euler Newton solver pinned to `Tet4` + `NullContact`,
/// generic over the mesh impl.
///
/// The constitutive law is fixed at `NeoHookean` per Phase 4 scope memo
/// Decision G — per-tet instances live on the mesh and are read at the
/// assembly hot points via `mesh.materials()` rather than a solver-level
/// `material` field.
///
/// Phase 2's canonical solver type for any hand-built tet scene;
/// specialize via the `Msh` parameter (e.g.,
/// `CpuTet4NHSolver<HandBuiltTetMesh>` for the multi-tet gate scenes,
/// `CpuTet4NHSolver<SingleTetMesh>` for the 1-tet skeleton).
pub type CpuTet4NHSolver<Msh> =
    solver::CpuNewtonSolver<element::Tet4, Msh, contact::NullContact, 4, 1>;

/// Walking-skeleton solver alias — `CpuTet4NHSolver` over `SingleTetMesh`.
///
/// Concrete backward-Euler Newton with `NeoHookean` + `Tet4` +
/// `SingleTetMesh` + `NullContact`. One hot-path monomorphization, kept
/// as a stable alias for the walking-skeleton tests post-Phase-2 —
/// same behavior as before, now expressed as a [`CpuTet4NHSolver`]
/// flavor per Phase 2 scope memo Decision H.
pub type SkeletonSolver = CpuTet4NHSolver<mesh::SingleTetMesh>;
