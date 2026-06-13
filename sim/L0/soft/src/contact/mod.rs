//! `ContactModel` trait — contact energy-term interface.
//!
//! Five items: active-pair detection, energy, gradient, Hessian, and
//! CCD time-of-impact. Two impls ship as of Phase 5 commit 4:
//! [`NullContact`] is the zero-stub default (real zeros — `Vec::new()`
//! / `0.0` / `f64::INFINITY`) for non-contact scenes, and
//! [`PenaltyRigidContact`] is the first force-bearing impl — soft
//! vertex against kinematic rigid primitives, one-way coupling.
//! Penalty is a stepping stone to IPC at Phase H per BF-12 (Phase 5
//! commit 9).

use crate::Vec3;
use crate::mesh::VertexId;
use nalgebra::Matrix3;

pub mod ipc;
pub mod null;
pub mod penalty;
pub mod rigid;

pub use ipc::IpcRigidContact;
pub use null::NullContact;
pub use penalty::{PenaltyRigidContact, filter_pair_readouts_to_referenced};
pub use rigid::RigidPlane;

/// A pair of geometric primitives active under the contact model.
///
/// Currently the [`Vertex`](Self::Vertex) variant only — a soft-body
/// vertex against a rigid primitive, the one-way coupling case (rigid
/// kinematic; soft side feels the force). A future IPC upgrade will add
/// `EdgeEdge` / `VertexFace` variants for self-contact; the
/// single-variant enum is deliberately heavier than a struct to avoid a
/// breaking change at that handoff.
#[derive(Clone, Debug)]
pub enum ContactPair {
    /// A soft-body vertex contacting a rigid primitive registered at
    /// `primitive_id` in the contact model's primitive list.
    Vertex {
        /// Index of the contacted soft vertex into the mesh's vertex list.
        vertex_id: VertexId,
        /// Index of the rigid primitive into the contact model's
        /// primitive list — opaque to consumers, scoped to the model
        /// that produced the pair.
        primitive_id: u32,
    },
}

/// Per-pair contact-energy gradient with respect to vertex positions.
///
/// Sparse: each `(vertex_id, force)` entry contributes one soft vertex's
/// per-pair gradient. For Phase 5's vertex-vs-rigid case, every active
/// pair produces exactly one entry (the contacted vertex). Phase H IPC
/// may emit multiple entries per pair for edge-edge / vertex-face.
#[derive(Clone, Debug, Default)]
pub struct ContactGradient {
    /// Sparse `(vertex_id, force)` entries — one per contributing vertex.
    /// Empty `Vec` is the additive identity (zero gradient).
    pub contributions: Vec<(VertexId, Vec3)>,
}

/// Per-pair contact-energy Hessian with respect to vertex positions.
///
/// Sparse 3×3-block representation: each `(row_vertex, col_vertex, block)`
/// entry contributes `block` to the global Hessian at the corresponding
/// 3×3 super-block. For Phase 5's vertex-vs-rigid case, every active pair
/// produces exactly one diagonal block — the rank-1 outer product
/// `κ · n ⊗ n`. Phase H IPC may emit off-diagonal cross-blocks for
/// edge-edge / vertex-face.
#[derive(Clone, Debug, Default)]
pub struct ContactHessian {
    /// Sparse `(row_vertex, col_vertex, block)` entries — each entry
    /// adds `block` to the global Hessian's 3×3 super-block at
    /// `(row_vertex, col_vertex)`. Empty `Vec` is the additive identity
    /// (zero Hessian).
    pub contributions: Vec<(VertexId, VertexId, Matrix3<f64>)>,
}

/// Per-active-pair readout — contact geometry plus the force the
/// contact model exerts on the soft side.
///
/// Bundles the outputs of a single active-pair evaluation for
/// downstream readout consumers (the row 18 `contact-force-readout`
/// example, future calibration loops). Sister of [`ContactGradient`] /
/// [`ContactHessian`] at the human-facing readout layer: those return
/// the data the *solver* needs to assemble a residual / tangent;
/// `ContactPairReadout` returns the data a *user* needs to inspect
/// what's happening at the contact interface.
///
/// Capture timing is on-demand — readout consumers pass current
/// positions and the producer recomputes geometry + force from scratch,
/// matching the on-demand semantics of [`ActivePairsFor::active_pairs`].
/// No per-iter cache is maintained.
///
/// **Sign convention** — `force_on_soft` is the force the contact
/// model exerts on the soft side at this pair, equal to `-gradient` of
/// the contact energy at the contacted vertex. For the penalty case
/// with outward primitive normal `n`, this resolves to
/// `+κ·(d̂-sd)·n` (positive scalar times outward normal: the soft body
/// is pushed *away* from the rigid surface, restoring the active config
/// back to the inactive band). The Newton's-3rd-law reaction on the
/// rigid side is `-force_on_soft`.
#[derive(Clone, Debug)]
pub struct ContactPairReadout {
    /// The active pair this readout describes.
    pub pair: ContactPair,
    /// Position of the contacted soft vertex (or relevant geometric
    /// point for future IPC variants) at the readout-time configuration.
    pub position: Vec3,
    /// Signed distance from `position` to the rigid primitive at
    /// readout time. Values strictly less than `d̂` are the active
    /// regime — matching the gate at
    /// [`ActivePairsFor::active_pairs`]; the producer only emits
    /// readouts for active pairs.
    pub sd: f64,
    /// Outward-pointing unit normal of the rigid primitive evaluated
    /// at `position`.
    pub normal: Vec3,
    /// Force the contact model exerts on the soft side at this pair —
    /// see "Sign convention" in the type docs.
    pub force_on_soft: Vec3,
}

/// Contact-energy surface over candidate pairs. `dyn`-compatible at
/// scene construction; monomorphized `C: ContactModel` on the hot path.
///
/// The material-agnostic part — energy, gradient, Hessian, CCD —
/// lives here. Active-pair selection (which depends on mesh
/// topology, hence on `M` via `&dyn Mesh<M>`) lives in the
/// [`ActivePairsFor`] subtrait. The split keeps method-call sites
/// like `c.hessian(...)` from needing M-inference at every test
/// caller.
pub trait ContactModel: Send + Sync {
    /// Contact energy contribution for a single pair (J).
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64;

    /// Contact gradient contribution for a single pair.
    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient;

    /// Contact Hessian contribution for a single pair.
    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian;

    /// Continuous-collision time of impact along the segment `x0 → x1`.
    /// `f64::INFINITY` means no contact within the step.
    fn ccd_toi(&self, pair: &ContactPair, x0: &[Vec3], x1: &[Vec3]) -> f64;

    /// Sensitivity of this pair's contact-residual contribution to a
    /// unit *rigid translation* of its primitive along the world
    /// direction `dir` — `∂(∂E/∂x_pair)/∂δ`, the per-vertex 3-vector that
    /// adds into the global residual derivative `∂r/∂δ`.
    ///
    /// This is the kinematic-pose analog of [`Self::gradient`]: where
    /// `gradient` differentiates the contact residual w.r.t. the soft
    /// vertex position, this differentiates it w.r.t. moving the rigid
    /// obstacle. It is the keystone S3 missing factor — the soft solver's
    /// IFT machinery (`∂x*/∂θ = −A⁻¹∂r/∂θ`) is otherwise load-only and
    /// cannot see the contact-plane pose (baked in at construction).
    ///
    /// Default: empty (a pose-independent / kinematic-free contact such
    /// as [`NullContact`] contributes nothing). [`PenaltyRigidContact`]
    /// overrides it: for a translated rigid SDF `sd(p; δ) = sd₀(p − δ·dir)`
    /// so `∂sd/∂δ = −∇sd·dir = −n̂·dir`, and (planes: `∂n̂/∂δ = 0`)
    /// `∂r_v/∂δ = d²E/dsd² · (−n̂·dir) · n̂`.
    ///
    /// **Contract for new contact models**: any *pose-dependent* contact
    /// MUST override this — the default returns zero, so a forgotten
    /// override yields a silently-zero pose gradient (the IFT pose
    /// sensitivity built on it would be wrong, not merely unsupported)
    /// rather than a compile error.
    ///
    /// **Scope** — engaged regime (the active set must be stable across
    /// the pose perturbation; the penalty active-set boundary is
    /// non-smooth, IPC the deferred cure) and the constant-normal
    /// (plane) case (`∂n̂/∂δ = 0`); curved-primitive normal curvature is
    /// a documented deferral. See `docs/keystone/s3_soft_pose_sensitivity_recon.md`.
    fn pose_residual_derivative(
        &self,
        _pair: &ContactPair,
        _positions: &[Vec3],
        _dir: Vec3,
    ) -> ContactGradient {
        ContactGradient::default()
    }
}

/// Active-pair selection for a [`ContactModel`] over a particular
/// material type `M`.
///
/// Split out from [`ContactModel`] so that `c.hessian(...)` etc.
/// don't trigger M-inference at every method call site (only
/// `active_pairs` actually depends on the mesh, hence on `M`).
/// Implemented generically over `M: Material` for both
/// [`NullContact`] and [`PenaltyRigidContact`]; the same impl carries
/// through any mesh's material model per arc memo D10.
pub trait ActivePairsFor<M: crate::material::Material>: ContactModel {
    /// Active contact pairs for the given positions.
    fn active_pairs(&self, mesh: &dyn crate::mesh::Mesh<M>, positions: &[Vec3])
    -> Vec<ContactPair>;
}
