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
