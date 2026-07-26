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
    face::{self, FaceBarrierEval},
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

    /// Per-point barrier evaluator for a P2 face against primitive
    /// `primitive_id` — the `(n̂, κb, κb', κb'')` closure the
    /// [`face`](super::face) machinery samples at its Gauss points. All SDF and
    /// κ knowledge stays here; the area/quadrature math stays in `face`. An
    /// inactive point (`sd ≥ d̂`) returns zero scalars (continuously — the
    /// barrier and its two derivatives vanish at `d̂`).
    fn face_eval(&self, primitive_id: u32) -> impl Fn(Vec3) -> FaceBarrierEval + '_ {
        let prim = &self.primitives[primitive_id as usize];
        move |x: Vec3| {
            let p = Point3::from(x);
            let normal = prim.grad(p);
            self.barrier(prim.eval(p)).map_or_else(
                || FaceBarrierEval {
                    normal,
                    b: 0.0,
                    b_d: 0.0,
                    b_dd: 0.0,
                },
                |c| FaceBarrierEval {
                    normal,
                    b: c.energy,
                    b_d: c.d_energy_d_sd,
                    b_dd: c.d2_energy_d_sd2,
                },
            )
        }
    }

    /// Gather the six face-node positions into the `[c0,c1,c2,m01,m12,m02]`
    /// array the [`face`](super::face) machinery expects.
    fn gather_face(nodes: &[VertexId; 6], positions: &[Vec3]) -> [Vec3; 6] {
        nodes.map(|vid| positions[vid as usize])
    }
}

impl ContactModel for IpcRigidContact {
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64 {
        match pair {
            ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } => {
                let p = Point3::from(positions[*vertex_id as usize]);
                let sd = self.primitives[*primitive_id as usize].eval(p);
                self.barrier(sd).map_or(0.0, |c| c.energy)
            }
            ContactPair::Face {
                nodes,
                primitive_id,
                rest_area,
            } => {
                // Surface-integrated barrier E = A_rest · Σ ŵ b(sd) over the P2 face.
                let nodes6 = Self::gather_face(nodes, positions);
                face::face_energy(&nodes6, *rest_area, self.face_eval(*primitive_id))
            }
        }
    }

    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient {
        match pair {
            ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } => {
                let p = Point3::from(positions[*vertex_id as usize]);
                let prim = &self.primitives[*primitive_id as usize];
                let sd = prim.eval(p);
                self.barrier(sd).map_or_else(ContactGradient::default, |c| {
                    // Residual contribution +∂E/∂x = (κ·b')·n̂ (∂sd/∂x = n̂).
                    let n = prim.grad(p);
                    ContactGradient {
                        contributions: vec![(*vertex_id, c.d_energy_d_sd * n)],
                    }
                })
            }
            ContactPair::Face {
                nodes,
                primitive_id,
                rest_area,
            } => {
                // +∂E/∂x_i distributed over all six P2 face nodes, purely along
                // the SDF normal (rest-area weight → no tangential term).
                let nodes6 = Self::gather_face(nodes, positions);
                let g = face::face_gradient(&nodes6, *rest_area, self.face_eval(*primitive_id));
                ContactGradient {
                    contributions: (0..6).map(|i| (nodes[i], g[i])).collect(),
                }
            }
        }
    }

    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian {
        match pair {
            ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } => {
                let p = Point3::from(positions[*vertex_id as usize]);
                let prim = &self.primitives[*primitive_id as usize];
                let sd = prim.eval(p);
                self.barrier(sd).map_or_else(ContactHessian::default, |c| {
                    // Rank-1 SPD block κ·b''·n̂⊗n̂ (plane: ∂²sd/∂x² = 0, b'' > 0 on (0,d̂)).
                    let n = prim.grad(p);
                    let block: Matrix3<f64> = c.d2_energy_d_sd2 * (n * n.transpose());
                    ContactHessian {
                        contributions: vec![(*vertex_id, *vertex_id, block)],
                    }
                })
            }
            ContactPair::Face {
                nodes,
                primitive_id,
                rest_area,
            } => {
                // 6×6 cross-node blocks (rank-1 b''·n̂⊗n̂ per Gauss point,
                // distributed via the P2 shape functions). All 36 emitted; the
                // assembler applies its own free-DOF lower-triangle filter and
                // the block set is symmetric.
                let nodes6 = Self::gather_face(nodes, positions);
                let h = face::face_hessian(&nodes6, *rest_area, self.face_eval(*primitive_id));
                let mut contributions = Vec::with_capacity(36);
                for (i, &row) in nodes.iter().enumerate() {
                    for (j, &col) in nodes.iter().enumerate() {
                        contributions.push((row, col, h[i][j]));
                    }
                }
                ContactHessian { contributions }
            }
        }
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
        let ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair
        else {
            // Unreachable in the current wiring: the rigid-pose (keystone)
            // derivative is called only by the coupling pose-sensitivity, which
            // runs on Tet4 meshes → `active_pairs` emits only `Vertex` pairs, so
            // a `Face` pair never reaches here. Face rigid-pose is a separate
            // adjoint channel deferred to a later rung (rung 8b ships the
            // frictionless soft-side face barrier + its soft adjoint only); fail
            // loud rather than return a silently-zero pose gradient.
            unreachable!(
                "ContactPair::Face rigid-pose derivative is unreachable (coupling pose path is \
                 Tet4-only); face-pose reconciliation is a deferred rung"
            )
        };
        let (vertex_id, primitive_id) = (*vertex_id, *primitive_id);
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
    fn active_pairs(&self, mesh: &dyn Mesh<M>, positions: &[Vec3]) -> Vec<ContactPair> {
        // A quadratic (Tet10) mesh surfaces six-node boundary faces → the
        // surface-integrated face barrier (contact restricted to the boundary
        // surface). A linear mesh returns `None` → the per-vertex loop, which
        // is byte-identical to the pre-rung-8b path (Tet4 unchanged).
        mesh.boundary_faces6().map_or_else(
            || self.active_vertex_pairs(positions),
            |faces6| self.active_face_pairs(faces6, positions, mesh.positions()),
        )
    }
}

impl IpcRigidContact {
    // `vid as VertexId` / `pid as u32` are Vec-iteration indices (see per_pair_readout).
    #[allow(clippy::cast_possible_truncation)]
    fn active_vertex_pairs(&self, positions: &[Vec3]) -> Vec<ContactPair> {
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

    // `pid as u32` is a Vec-iteration index (see per_pair_readout).
    #[allow(clippy::cast_possible_truncation)]
    fn active_face_pairs(
        &self,
        faces6: &[[VertexId; 6]],
        positions: &[Vec3],
        rest_positions: &[Vec3],
    ) -> Vec<ContactPair> {
        let mut pairs = Vec::new();
        for face in faces6 {
            // Rest (reference) area weight — the straight-edged rest face's
            // corner-triangle area (rest positions, not the deformed ones), so
            // the barrier weight is a per-face constant.
            let rest_area = Self::face_rest_area(face, rest_positions);
            for (pid, prim) in self.primitives.iter().enumerate() {
                // Emit the face for this primitive if any of its six nodes is
                // within the barrier band. A conservative over-approximation:
                // an included-but-marginally-inactive face contributes ~0
                // (every Gauss point outside the band yields zero scalars), so
                // over-inclusion is harmless while never dropping an active
                // face. (An interior-only dip below d̂ with all six nodes above
                // is not represented — acceptable for the near-flat contact
                // faces rung 8b targets; the curved surface is rung 8c.)
                let active = face.iter().any(|&vid| {
                    self.pair_is_active(prim.eval(Point3::from(positions[vid as usize])))
                });
                if active {
                    pairs.push(ContactPair::Face {
                        nodes: *face,
                        primitive_id: pid as u32,
                        rest_area,
                    });
                }
            }
        }
        pairs
    }

    /// Rest-configuration physical area of a P2 face — the corner triangle's
    /// area at the rest positions (the straight-edged rest face is planar, so
    /// the midsides sit at edge midpoints and do not change the area).
    fn face_rest_area(face: &[VertexId; 6], rest_positions: &[Vec3]) -> f64 {
        let x0 = rest_positions[face[0] as usize];
        let x1 = rest_positions[face[1] as usize];
        let x2 = rest_positions[face[2] as usize];
        0.5 * (x1 - x0).cross(&(x2 - x0)).norm()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Absolute pin on the rest-area weight: a right triangle with legs 3 and 4
    /// has area 6. `rest_area` is a shared multiplicative scale on the face
    /// energy/gradient/Hessian, so every self-consistency and FD gate cancels
    /// it — only this independent check guards the `0.5·|cross|` formula
    /// (triangle, not parallelogram) and the corner-node triple.
    #[test]
    fn face_rest_area_is_the_triangle_area() {
        let pos = [
            Vec3::new(0.0, 0.0, 0.0), // c0
            Vec3::new(3.0, 0.0, 0.0), // c1
            Vec3::new(0.0, 4.0, 0.0), // c2
            Vec3::new(1.5, 0.0, 0.0), // midsides — not read by the corner-area formula
            Vec3::new(1.5, 2.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        let face = [0, 1, 2, 3, 4, 5];
        let a = IpcRigidContact::face_rest_area(&face, &pos);
        assert!((a - 6.0).abs() < 1e-14, "expected triangle area 6, got {a}");
    }
}
