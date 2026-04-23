//! `ContactModel` trait — contact energy-term interface.
//!
//! Five items: active-pair detection, energy, gradient, Hessian, and
//! CCD time-of-impact. Skeleton ships `NullContact` (zero stubs,
//! returns empty/zero) to exercise the trait surface without pulling
//! in IPC machinery — IPC lands in Phase C per spec §8.

use crate::Vec3;

pub mod null;

pub use null::NullContact;

/// A pair of geometric primitives active under the contact model (e.g.
/// vertex-triangle, edge-edge). Unit stub; Phase C populates primitive
/// index tuples + normal direction.
#[derive(Clone, Debug, Default)]
pub struct ContactPair;

/// Per-pair contact-energy gradient with respect to vertex positions.
/// Unit stub; Phase C carries a 3-vector-per-vertex sparse contribution.
#[derive(Clone, Debug, Default)]
pub struct ContactGradient;

/// Per-pair contact-energy Hessian with respect to vertex positions.
/// Unit stub; Phase C carries a sparse block matrix.
#[derive(Clone, Debug, Default)]
pub struct ContactHessian;

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
