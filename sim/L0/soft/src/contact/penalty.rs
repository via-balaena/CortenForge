//! Penalty rigid↔soft contact — first force-bearing
//! [`ContactModel`](super::ContactModel) impl on `sim-soft`.
//!
//! One-way coupling per `phase_5_penalty_contact_scope.md` Decision C
//! (rigid kinematic; soft side feels the force). Penalty is a stepping
//! stone to IPC at Phase H per BF-12 (Phase 5 commit 9), not a
//! production baseline. The structural failure modes documented in
//! book Part 4 §00 §00 — active-set discontinuity at `d = d̂`,
//! parameter sensitivity, oscillation pathology near the boundary —
//! stay valid; the BF-12 amendment narrows §00 §00's "even as a
//! baseline" commitment to "as a *production* baseline." The scope
//! memo authorizes penalty exactly for §00 §00's named validation
//! scope: rigid↔soft co-sim plumbing, first-time
//! [`ContactModel`](super::ContactModel) wiring into the Newton hot
//! path (Phase 5 commit 5), and the Hertzian sphere↔plane analytic
//! gate (Phase 5 commit 9 V-3).
//!
//! ## Formula
//!
//! For a soft vertex at position `p` and a rigid primitive with
//! signed-distance `d = d(p)` and outward unit normal `n = ∂d/∂p`:
//!
//! - **Energy** `E(p) = ½ κ (d̂ − d)²` for `d < d̂`, else `0`.
//! - **Gradient** `∂E/∂p = −κ (d̂ − d) · n` for `d < d̂`, else zero —
//!   at active config (`d < d̂`), `(d̂ − d) > 0` so the gradient points
//!   *opposite* the outward normal, i.e., into the rigid surface,
//!   mirroring the elastic case where `f_int` points further into the
//!   deformed configuration. The restoring force on the vertex is
//!   `−∂E/∂p = +κ (d̂ − d) · n`, along `+n` — back into the half-space
//!   outside the rigid body.
//! - **Hessian** `∂²E/∂p² = κ · n ⊗ n` (rank-1 outer product) for
//!   `d < d̂`, else zero. PSD only — two zero eigenvalues along the
//!   tangent plane to `n`, one positive eigenvalue `κ` along `n`. The
//!   full system tangent becomes SPD after the elastic Hessian is
//!   added.
//! - **CCD** — penalty has no time-of-impact concept; `ccd_toi`
//!   returns [`f64::INFINITY`] per scope memo Decision E. Phase H IPC
//!   delivers proper CCD.

use super::{
    ContactGradient, ContactHessian, ContactModel, ContactPair,
    rigid::{RigidPlane, RigidPrimitive},
};
use crate::{
    Vec3,
    mesh::{Mesh, VertexId},
};
use nalgebra::Matrix3;

/// Default penalty stiffness (N/m). Pinned at Phase 5 commit 4 per
/// scope memo Decision J. Middle of the recommended `1e3..1e5` range
/// — at Ecoflex-class material `E ≈ 200 kPa` and `h ≈ 5 mm` element
/// edge, element stiffness `E·h ≈ 1e3 N/m`; κ at 10× element
/// stiffness is the "stiff but Newton-convergent" regime. May tune
/// at commit 8 / 9 if V-3a / V-3 expose stability or accuracy issues
/// per Decision J.
pub(crate) const PENALTY_KAPPA_DEFAULT: f64 = 1.0e4;

/// Default contact band (m). Pinned at Phase 5 commit 4 per scope
/// memo Decision J. 1 mm — ~20× the expected Hertz indentation
/// `δ ≈ 5e-5 m` at V-3's R = 1 cm soft sphere × Ecoflex-class
/// composite, so the active-set band cleanly contains the contact
/// patch without pulling in spurious distant-vertex pairs. May tune
/// at commit 8 / 9 per Decision J.
pub(crate) const PENALTY_DHAT_DEFAULT: f64 = 1.0e-3;

/// Penalty contact between soft-body vertices and a set of kinematic
/// rigid primitives.
///
/// One [`ContactPair::Vertex`] per `(soft vertex, rigid primitive)`
/// whose signed distance is below the contact band `d̂`. See the
/// [module docs](self) for the energy / gradient / Hessian formulas
/// and sign conventions.
pub struct PenaltyRigidContact {
    primitives: Vec<Box<dyn RigidPrimitive>>,
    kappa: f64,
    d_hat: f64,
}

impl PenaltyRigidContact {
    /// Construct with default `(κ, d̂)` from
    /// [`PENALTY_KAPPA_DEFAULT`] / [`PENALTY_DHAT_DEFAULT`]. Boxes the
    /// concrete planes into the trait-erased primitive list internally
    /// — [`RigidPrimitive`](super::rigid) stays `pub(crate)` per scope
    /// memo §0 row.
    #[must_use]
    pub fn new(planes: Vec<RigidPlane>) -> Self {
        Self::with_params(planes, PENALTY_KAPPA_DEFAULT, PENALTY_DHAT_DEFAULT)
    }

    /// Construct with non-default `(κ, d̂)` — Phase 5 testing surface
    /// for V-* invariants and the V-7 differentiability hook
    /// (commit 11). User-facing parameter tuning is a Phase E
    /// follow-on per scope memo Decision J; production scenes go
    /// through [`new`](Self::new).
    #[must_use]
    pub fn with_params(planes: Vec<RigidPlane>, kappa: f64, d_hat: f64) -> Self {
        let primitives: Vec<Box<dyn RigidPrimitive>> = planes
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn RigidPrimitive>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
        }
    }
}

impl ContactModel for PenaltyRigidContact {
    /// Walks soft vertices outer (`0..positions.len()`) × rigid
    /// primitives inner (`0..self.primitives.len()`); emits a
    /// [`ContactPair::Vertex`] for every `(v, p)` whose signed
    /// distance is below the band `d̂`. Order is deterministic — no
    /// sort, no `HashMap`, no rayon — per scope memo Decision M.
    #[allow(clippy::cast_possible_truncation)]
    fn active_pairs(&self, _mesh: &dyn Mesh, positions: &[Vec3]) -> Vec<ContactPair> {
        // `vid` and `pid` are `Vec`-iteration indices; in practice
        // bounded by mesh / primitive counts that fit comfortably in
        // `u32`. The `as` cast matches the convention used in
        // `mesh/hand_built.rs` for `VertexId` packing. A theoretical
        // overflow on a 64-bit pointer would surface as wrapped
        // indices; not load-bearing for Phase 5 mesh sizes.
        let mut pairs = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            for (pid, prim) in self.primitives.iter().enumerate() {
                if prim.signed_distance(p) < self.d_hat {
                    pairs.push(ContactPair::Vertex {
                        vertex_id: vid as VertexId,
                        primitive_id: pid as u32,
                    });
                }
            }
        }
        pairs
    }

    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64 {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = positions[vertex_id as usize];
        let d = self.primitives[primitive_id as usize].signed_distance(p);
        if d >= self.d_hat {
            0.0
        } else {
            let gap = self.d_hat - d;
            0.5 * self.kappa * gap * gap
        }
    }

    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = positions[vertex_id as usize];
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.signed_distance(p);
        if d >= self.d_hat {
            ContactGradient::default()
        } else {
            let n = prim.outward_normal(p);
            let force = -self.kappa * (self.d_hat - d) * n;
            ContactGradient {
                contributions: vec![(vertex_id, force)],
            }
        }
    }

    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = positions[vertex_id as usize];
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.signed_distance(p);
        if d >= self.d_hat {
            ContactHessian::default()
        } else {
            let n = prim.outward_normal(p);
            let block: Matrix3<f64> = self.kappa * (n * n.transpose());
            ContactHessian {
                contributions: vec![(vertex_id, vertex_id, block)],
            }
        }
    }

    fn ccd_toi(&self, _pair: &ContactPair, _x0: &[Vec3], _x1: &[Vec3]) -> f64 {
        f64::INFINITY
    }
}
