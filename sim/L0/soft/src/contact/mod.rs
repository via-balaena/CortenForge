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

pub mod null;
pub mod penalty;
pub mod rigid;

pub use null::NullContact;
pub use penalty::PenaltyRigidContact;
pub use rigid::RigidPlane;

/// A pair of geometric primitives active under the contact model.
///
/// Phase 5 ships the [`Vertex`](Self::Vertex) variant only — a soft-body
/// vertex against a rigid primitive, the one-way coupling case per
/// `phase_5_penalty_contact_scope.md` Decision C. Phase H IPC will add
/// `EdgeEdge` / `VertexFace` variants for self-contact per Decision G;
/// the single-variant enum is deliberately heavier than a struct to
/// avoid a breaking change at that handoff.
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
/// matching the on-demand semantics of [`ContactModel::active_pairs`].
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
    /// [`ContactModel::active_pairs`]; the producer only emits
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
pub trait ContactModel: Send + Sync {
    /// Active contact pairs for the given positions.
    fn active_pairs(&self, mesh: &dyn crate::mesh::Mesh, positions: &[Vec3]) -> Vec<ContactPair>;

    /// Contact energy contribution for a single pair (J).
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64;

    /// Contact gradient contribution for a single pair.
    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient;

    /// Contact Hessian contribution for a single pair.
    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian;

    /// Continuous-collision time of impact along the segment `x0 → x1`.
    /// `f64::INFINITY` means no contact within the step.
    fn ccd_toi(&self, pair: &ContactPair, x0: &[Vec3], x1: &[Vec3]) -> f64;
}
