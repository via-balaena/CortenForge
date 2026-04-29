//! `NullContact` — empty active-pairs, zero energy/gradient/Hessian.
//!
//! Default contact model — real zero-stubs as of Phase 5. Exercises
//! the trait surface without pulling in IPC machinery.
//! Phase C adds `IpcBarrierModel` with CCD + friction per spec §8.

use super::{ContactGradient, ContactHessian, ContactModel, ContactPair};
use crate::Vec3;

/// Null contact — no active pairs, no contact forces.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullContact;

impl ContactModel for NullContact {
    fn active_pairs(&self, _mesh: &dyn crate::mesh::Mesh, _positions: &[Vec3]) -> Vec<ContactPair> {
        Vec::new()
    }

    fn energy(&self, _pair: &ContactPair, _positions: &[Vec3]) -> f64 {
        0.0
    }

    fn gradient(&self, _pair: &ContactPair, _positions: &[Vec3]) -> ContactGradient {
        ContactGradient::default()
    }

    fn hessian(&self, _pair: &ContactPair, _positions: &[Vec3]) -> ContactHessian {
        ContactHessian::default()
    }

    fn ccd_toi(&self, _pair: &ContactPair, _x0: &[Vec3], _x1: &[Vec3]) -> f64 {
        f64::INFINITY
    }
}
