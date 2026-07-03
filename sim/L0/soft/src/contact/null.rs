//! `NullContact` — empty active-pairs, zero energy/gradient/Hessian.
//!
//! Default contact model — real zero-stubs as of Phase 5. Exercises
//! the trait surface without pulling in IPC machinery. The C²-barrier
//! successor [`IpcRigidContact`](crate::IpcRigidContact) (CCD +
//! friction) has since shipped; `NullContact` remains the zero-cost
//! default for non-contact scenes.

use super::{ActivePairsFor, ContactGradient, ContactHessian, ContactModel, ContactPair};
use crate::Vec3;

/// Null contact — no active pairs, no contact forces.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullContact;

impl ContactModel for NullContact {
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

impl<M: crate::material::Material> ActivePairsFor<M> for NullContact {
    fn active_pairs(
        &self,
        _mesh: &dyn crate::mesh::Mesh<M>,
        _positions: &[Vec3],
    ) -> Vec<ContactPair> {
        Vec::new()
    }
}
