//! `NullContact` — empty active-pairs, zero energy/gradient/Hessian.
//!
//! Skeleton's contact model. Exercises the trait without IPC machinery.
//! Phase C adds `IpcBarrierModel` with CCD + friction per spec §8.

// Stub impl — every method body is `unimplemented!("skeleton phase 2")`
// by design; Phase C lands real IPC machinery. Module-level allow so the
// grader's per-file safety scan matches the `lib.rs` override.
#![allow(clippy::unimplemented)]

use super::{ContactGradient, ContactHessian, ContactModel, ContactPair};
use crate::Vec3;

/// Null contact — no active pairs, no contact forces.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullContact;

impl ContactModel for NullContact {
    fn active_pairs(&self, _mesh: &dyn crate::mesh::Mesh, _positions: &[Vec3]) -> Vec<ContactPair> {
        unimplemented!("skeleton phase 2")
    }

    fn energy(&self, _pair: &ContactPair, _positions: &[Vec3]) -> f64 {
        unimplemented!("skeleton phase 2")
    }

    fn gradient(&self, _pair: &ContactPair, _positions: &[Vec3]) -> ContactGradient {
        unimplemented!("skeleton phase 2")
    }

    fn hessian(&self, _pair: &ContactPair, _positions: &[Vec3]) -> ContactHessian {
        unimplemented!("skeleton phase 2")
    }

    fn ccd_toi(&self, _pair: &ContactPair, _x0: &[Vec3], _x1: &[Vec3]) -> f64 {
        unimplemented!("skeleton phase 2")
    }
}
