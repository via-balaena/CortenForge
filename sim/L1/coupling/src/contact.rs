//! Rigid-collider plumbing for the coupling: the [`PlaneContact`] bridge trait
//! (over penalty/IPC contact), the finite `PosedSphere` collider, the `Collider`
//! geometry selector, and the per-step `SoftSolver` alias.

use sim_soft::{
    ActivePairsFor, ContactModel, ContactPairReadout, CpuNewtonSolver, HandBuiltTetMesh,
    IpcRigidContact, NeoHookean, PenaltyRigidContact, Sdf, SphereSdf, Tet4, TranslatedSdf, Vec3,
};

/// A rigid-primitive contact constructible from a single posed [`Sdf`] primitive +
/// `(κ, d̂)`, with an active-pair readout over the keystone block — the bridge that
/// lets [`StaggeredCoupling`](crate::StaggeredCoupling) run over either [`PenaltyRigidContact`] (the stepping
/// stone) or [`IpcRigidContact`] (the C²-barrier successor). Defined here (a local
/// trait) so the soft-side contact types need no change.
///
/// The primitive is generic over [`Sdf`] (not fixed to [`RigidPlane`](sim_soft::RigidPlane)) so the
/// coupling can pose a FINITE collider — a curved end-effector (`PosedSphere`)
/// indenting the soft block — and not just the infinite downward half-space. The
/// plane path constructs `from_primitive(RigidPlane, …)`, byte-identical to the
/// pre-generalization `with_params(vec![plane], …)` (a single boxing of the same
/// `RigidPlane`). The finite-collider path is the L1 finite-contact carry (the
/// gradient of #415's curved-contact term composed through the coupling adjoint).
pub trait PlaneContact: ContactModel + ActivePairsFor<NeoHookean> + Sized {
    /// Build the contact from a single posed rigid primitive (`RigidPlane` for the
    /// keystone half-space, `PosedSphere` for the finite collider) and the
    /// penalty/barrier parameters `(κ, d̂)`.
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self;
    /// Per-active-pair readout at `positions` (the inherent `per_pair_readout`,
    /// surfaced through the trait so the coupling can call it generically).
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout>;
}

impl PlaneContact for PenaltyRigidContact {
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![primitive], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

impl PlaneContact for IpcRigidContact {
    fn from_primitive<S: Sdf + 'static>(primitive: S, kappa: f64, d_hat: f64) -> Self {
        Self::with_params(vec![primitive], kappa, d_hat)
    }
    fn pair_readout(&self, mesh: &HandBuiltTetMesh, positions: &[Vec3]) -> Vec<ContactPairReadout> {
        self.per_pair_readout(mesh, positions)
    }
}

/// The keystone's finite rigid collider: a [`SphereSdf`] posed at a world centre
/// via [`TranslatedSdf`] (the shared `sim_soft` posing wrapper). `SphereSdf` is
/// origin-centred and rotation-invariant, so a query-point translation `p ↦ p −
/// center` is the full pose; `eval`/`grad`/`hessian` forward to the inner sphere
/// (translation leaves the Hessian — the curvature #415 added — unchanged, so the
/// curved-normal pose sensitivity is the sphere's own `∇²sd`).
pub(super) type PosedSphere = TranslatedSdf<SphereSdf>;

/// Pose a [`SphereSdf`] of the given `radius` at world `center` — the keystone's
/// finite curved end-effector (`PosedSphere`).
pub(super) fn posed_sphere(radius: f64, center: Vec3) -> PosedSphere {
    TranslatedSdf {
        inner: SphereSdf { radius },
        offset: center,
    }
}

/// The rigid collider geometry [`StaggeredCoupling`](crate::StaggeredCoupling) poses against the soft block.
#[derive(Clone, Copy, Debug)]
pub(super) enum Collider {
    /// The infinite downward half-space (the keystone #402–#406 scene; default) —
    /// `build_contact` constructs a [`RigidPlane`](sim_soft::RigidPlane), optionally orientation-tracking
    /// under [`StaggeredCoupling::with_rotating_normal`](crate::StaggeredCoupling::with_rotating_normal).
    Plane,
    /// A finite posed sphere of the given radius (the L1 finite-contact gate) — a
    /// curved end-effector indenting the block, exercising #415's curvature term
    /// (`∇²sd ≠ 0`) through the coupling adjoint. The scalar `height` still drives
    /// the vertical carry (the sphere centre rides `+ẑ` with `height`); the lateral
    /// centre defaults over the block for a central + stable patch, and is re-pointable
    /// to a moving end-effector (the arm tip) via
    /// [`StaggeredCoupling::with_contact_geom`](crate::StaggeredCoupling::with_contact_geom).
    Sphere { radius: f64 },
}

/// The soft Newton solver the coupling builds per step, over contact type `C`.
pub(super) type SoftSolver<C> = CpuNewtonSolver<Tet4, HandBuiltTetMesh, C, NeoHookean, 4, 1>;
