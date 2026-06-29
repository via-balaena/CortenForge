//! `IpcRigidContact` — IPC (Incremental Potential Contact, Li et al. 2020)
//! barrier contact between soft-body vertices and kinematic rigid primitives.
//!
//! The soft-body architecture's committed contact formulation
//! (`docs/studies/soft_body_architecture/src/40-contact/00-why-ipc.md`), replacing
//! the stepping-stone [`PenaltyRigidContact`](super::PenaltyRigidContact). IPC adds
//! a `C²`-smooth divergent **barrier energy** to the total potential the Newton
//! solver already minimizes, so contact is enforced by infinite energy at zero gap
//! (structural non-penetration) and is smoothly differentiable through the
//! make/break boundary (the property penalty lacks — its force is `C⁰`, so its
//! derivative kinks at the tolerance boundary; see `docs/ipc/recon.md` and the
//! keystone time-adjoint's measured 5–25% gradient degradation).
//!
//! ## Barrier
//!
//! For a vertex at signed distance `d = sd` from a rigid primitive, with contact
//! tolerance `d̂` and stiffness `κ`, the per-pair energy is `κ·b(d, d̂)` with
//!
//! ```text
//!     b(d, d̂) = −(d − d̂)²·ln(d / d̂)      for 0 < d < d̂,   0 for d ≥ d̂
//!     b'(d)   = −2(d − d̂)·ln(d/d̂) − (d − d̂)²/d
//!     b''(d)  = −2·ln(d/d̂) − 4(d − d̂)/d + (d − d̂)²/d²
//! ```
//!
//! with the three load-bearing properties: (a) `b → ∞` as `d → 0⁺` (non-penetration
//! by infinite energy, not projection); (b) `C²` on `(0, d̂]` (well-defined Newton
//! tangent + autograd VJP); (c) `b(d̂) = b'(d̂) = b''(d̂) = 0` (toggling a pair at the
//! tolerance boundary introduces no discontinuity in energy, force, or tangent —
//! the fix for the penalty kink).
//!
//! ## Scope (v1)
//!
//! Vertex-vs-rigid-primitive, one-way coupling (rigid kinematic; soft side feels the
//! barrier force) — the keystone-coupling case. The barrier is `−κ·b'·n̂` (force on
//! soft) with rank-1 Hessian `κ·b''·n̂⊗n̂` for the constant-normal plane case. **No
//! CCD** (a fast body could tunnel in one step) and **fixed κ** (no adaptive
//! schedule) in v1 — adequate for the small-`dt` keystone scene; both are deferred
//! robustness follow-ons (`40-contact/01-ipc-internals/{01-adaptive-kappa,02-ccd}.md`).
//! The barrier is undefined at `d ≤ 0`; a Newton / line-search trial may probe there
//! (no CCD), so the evaluation floors `d` at a tiny fraction of `d̂` — keeping the
//! barrier finite and strongly repulsive so the line search rejects penetrating
//! trials (a poor-man's filtering until CCD lands). A converged solve stays `d > 0`.

use super::{
    ContactGradient, ContactHessian, ContactModel, ContactPair, ContactPairReadout, RigidTwist,
};
use crate::{
    Vec3,
    mesh::{Mesh, VertexId},
    sdf_bridge::Sdf,
};
use nalgebra::{Matrix3, Point3};

/// Default barrier stiffness (the `κ` multiplier on `b`).
pub(crate) const IPC_KAPPA_DEFAULT: f64 = 1.0e4;
/// Default contact tolerance `d̂` (m) — the gap below which the barrier activates.
pub(crate) const IPC_DHAT_DEFAULT: f64 = 1.0e-3;

/// Per-pair scalar barrier contributions at a signed distance — the IPC analog of
/// `PenaltyRigidContact`'s `PairContribution`, consumed identically by
/// energy / gradient / Hessian / pose / readout.
struct BarrierContribution {
    energy: f64,
    d_energy_d_sd: f64,
    d2_energy_d_sd2: f64,
}

/// IPC log-barrier contact between soft-body vertices and a set of kinematic rigid
/// primitives (heap-erased [`Sdf`] trait objects). See the [module docs](self).
pub struct IpcRigidContact {
    primitives: Vec<Box<dyn Sdf>>,
    kappa: f64,
    d_hat: f64,
}

impl IpcRigidContact {
    /// Construct with the default `(κ, d̂)` (`IPC_KAPPA_DEFAULT` /
    /// `IPC_DHAT_DEFAULT`).
    #[must_use]
    pub fn new<I>(primitives: I) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        Self::with_params(primitives, IPC_KAPPA_DEFAULT, IPC_DHAT_DEFAULT)
    }

    /// Construct with explicit barrier stiffness `κ` and contact tolerance `d̂`.
    ///
    /// # Panics
    /// Panics if `kappa` or `d_hat` is not strictly positive and finite (the
    /// barrier `ln(d/d̂)` and the `κ` scaling require both).
    #[must_use]
    pub fn with_params<I>(primitives: I, kappa: f64, d_hat: f64) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            kappa > 0.0 && kappa.is_finite(),
            "IPC kappa must be positive and finite, got {kappa}",
        );
        assert!(
            d_hat > 0.0 && d_hat.is_finite(),
            "IPC d_hat must be positive and finite, got {d_hat}",
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
        }
    }

    /// Pair-inclusion gate: active iff the signed distance is below `d̂` (the
    /// barrier support). Penetration (`sd ≤ 0`) is active too — it is the deepest
    /// contact, handled by the floored evaluation in [`Self::barrier`].
    fn pair_is_active(&self, sd: f64) -> bool {
        sd < self.d_hat
    }

    /// Per-pair barrier scalars `(κ·b, κ·b', κ·b'')` at signed distance `sd`, or
    /// `None` if outside the support (`sd ≥ d̂`). The barrier is defined for
    /// `0 < d < d̂`; `sd` is floored at `d̂·1e-6` so a penetrating trial yields a
    /// finite, strongly-repulsive value rather than `NaN` (see the module docs'
    /// scope note).
    fn barrier(&self, sd: f64) -> Option<BarrierContribution> {
        if !self.pair_is_active(sd) {
            return None;
        }
        let d = sd.max(self.d_hat * 1.0e-6);
        let r = d - self.d_hat;
        let ln = (d / self.d_hat).ln();
        // b = −(d−d̂)²·ln(d/d̂);  b' = −2(d−d̂)ln − (d−d̂)²/d;
        // b'' = −2 ln − 4(d−d̂)/d + (d−d̂)²/d².
        Some(BarrierContribution {
            energy: self.kappa * (-(r * r) * ln),
            d_energy_d_sd: self.kappa * r.mul_add(-2.0 * ln, -(r * r) / d),
            d2_energy_d_sd2: self.kappa * (r * r / (d * d) - 4.0 * r / d - 2.0 * ln),
        })
    }

    /// Per-active-pair readout (vertex position, signed distance, outward normal,
    /// and barrier force on the soft side `−κ·b'·n̂`) — the IPC analog of
    /// `PenaltyRigidContact::per_pair_readout`, the surface the coupling and the
    /// keystone factors consume. Same walk order / band gate as
    /// [`ActivePairsFor::active_pairs`](super::ActivePairsFor::active_pairs).
    ///
    /// # Panics
    ///
    /// `positions` must cover the mesh's full `VertexId` space (see
    /// [`PenaltyRigidContact::per_pair_readout`](super::PenaltyRigidContact::per_pair_readout)
    /// for the shared precondition) — the per-pair tributary areas index
    /// `mesh.boundary_faces()` into `positions`.
    // `vid as VertexId` / `pid as u32` are Vec-iteration indices, bounded by mesh /
    // primitive counts that fit in u32 (mirrors the penalty/active_pairs idiom).
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn per_pair_readout<M: crate::material::Material>(
        &self,
        mesh: &dyn Mesh<M>,
        positions: &[Vec3],
    ) -> Vec<ContactPairReadout> {
        // Deformed-surface tributary areas, indexed by VertexId — one
        // pass over the boundary faces, computed lazily on the first
        // active pair so no-contact steps pay nothing (mirrors the
        // penalty producer; see `super::contact_pressure`).
        let mut areas: Option<Vec<f64>> = None;
        let mut readouts = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                let sd = prim.eval(p_pt);
                if let Some(c) = self.barrier(sd) {
                    let normal = prim.grad(p_pt);
                    let force_on_soft = -c.d_energy_d_sd * normal;
                    let tributary_area = areas.get_or_insert_with(|| {
                        crate::boundary_vertex_areas(positions, mesh.boundary_faces())
                    })[vid];
                    readouts.push(ContactPairReadout {
                        pair: ContactPair::Vertex {
                            vertex_id: vid as VertexId,
                            primitive_id: pid as u32,
                        },
                        position: p,
                        sd,
                        normal,
                        force_on_soft,
                        tributary_area,
                        pressure: super::contact_pressure(force_on_soft, tributary_area),
                    });
                }
            }
        }
        readouts
    }
}

impl ContactModel for IpcRigidContact {
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64 {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let sd = self.primitives[primitive_id as usize].eval(p);
        self.barrier(sd).map_or(0.0, |c| c.energy)
    }

    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let sd = prim.eval(p);
        self.barrier(sd).map_or_else(ContactGradient::default, |c| {
            // Residual contribution +∂E/∂x = (κ·b')·n̂ (∂sd/∂x = n̂).
            let n = prim.grad(p);
            ContactGradient {
                contributions: vec![(vertex_id, c.d_energy_d_sd * n)],
            }
        })
    }

    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let sd = prim.eval(p);
        self.barrier(sd).map_or_else(ContactHessian::default, |c| {
            // Rank-1 SPD block κ·b''·n̂⊗n̂ (plane: ∂²sd/∂x² = 0, b'' > 0 on (0,d̂)).
            let n = prim.grad(p);
            let block: Matrix3<f64> = c.d2_energy_d_sd2 * (n * n.transpose());
            ContactHessian {
                contributions: vec![(vertex_id, vertex_id, block)],
            }
        })
    }

    fn ccd_toi(&self, _pair: &ContactPair, _x0: &[Vec3], _x1: &[Vec3]) -> f64 {
        // v1: no CCD (deferred robustness follow-on). See the module docs.
        f64::INFINITY
    }

    fn pose_residual_derivative(
        &self,
        pair: &ContactPair,
        positions: &[Vec3],
        twist: RigidTwist,
    ) -> ContactGradient {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let sd = prim.eval(p);
        self.barrier(sd).map_or_else(ContactGradient::default, |c| {
            // Under an infinitesimal rigid motion (spatial twist `(ω, v)`) of the
            // primitive the normal rotates `δn̂ = ω×n̂` and `sd = p·n̂ − offset`
            // changes by `∂sd/∂s = p·δn̂ − v·n̂`, so the per-vertex residual
            // derivative is `d²E/dsd²·(∂sd/∂s)·n̂ + (dE/dsd)·δn̂` (`d²E/dsd² = κ·b''`).
            // The direction term vanishes for a pure translation (`ω = 0`),
            // recovering `κ·b''·(−n̂·dir)·n̂` exactly.
            //
            // FLAT-PRIMITIVE scope (deferred for IPC): this drops the curved-primitive
            // `−H·u` curvature term (and the dual `dE·H` in `hessian`) that
            // `PenaltyRigidContact` carries via [`Sdf::hessian`] — exact for a plane
            // (`H = 0`) but ~0.7% off for a sphere. The differentiated keystone path uses
            // the penalty contact; lift the same two terms here when IPC is differentiated
            // through a curved primitive.
            let n = prim.grad(p);
            let dn = twist.angular.cross(&n);
            let dsd = p.coords.dot(&dn) - twist.linear.dot(&n);
            ContactGradient {
                contributions: vec![(
                    vertex_id,
                    c.d2_energy_d_sd2 * dsd * n + c.d_energy_d_sd * dn,
                )],
            }
        })
    }
}

impl<M: crate::material::Material> super::ActivePairsFor<M> for IpcRigidContact {
    // `vid as VertexId` / `pid as u32` are Vec-iteration indices (see per_pair_readout).
    #[allow(clippy::cast_possible_truncation)]
    fn active_pairs(&self, _mesh: &dyn Mesh<M>, positions: &[Vec3]) -> Vec<ContactPair> {
        let mut pairs = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                if self.pair_is_active(prim.eval(p_pt)) {
                    pairs.push(ContactPair::Vertex {
                        vertex_id: vid as VertexId,
                        primitive_id: pid as u32,
                    });
                }
            }
        }
        pairs
    }
}
