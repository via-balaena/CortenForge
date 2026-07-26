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
            // Curved-SDF term `∇²sd = ∂n̂/∂x` (rung 8c) — `0` for a plane, so the
            // face Hessian reduces to the pre-8c `b''·n̂⊗n̂`. Cheap to sample even
            // for an inactive point; the Hessian scales it by `b'`, which is `0`
            // there, so it costs nothing.
            let curvature = prim.hessian(p);
            self.barrier(prim.eval(p)).map_or_else(
                || FaceBarrierEval {
                    normal,
                    curvature,
                    b: 0.0,
                    b_d: 0.0,
                    b_dd: 0.0,
                },
                |c| FaceBarrierEval {
                    normal,
                    curvature,
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
                    // ∂(g_v)/∂x_v = b''·n̂n̂ᵀ + b'·∂n̂/∂x_v. The first term is the
                    // rank-1 barrier block; the second is the GEOMETRIC stiffness —
                    // as the vertex slides over a CURVED primitive its contact
                    // normal turns, `∂n̂/∂x_v = ∇²sd` ([`Sdf::hessian`]). `∇²sd = 0`
                    // for a plane (constant normal) → the historical `b''·n̂n̂ᵀ`;
                    // for a sphere it adds `b'·∇²sd` (rung 8c — reconciled with the
                    // `PenaltyRigidContact` vertex path, which already carries it).
                    // Indefinite in the band (`b' < 0`, `∇²sd` PSD) — the true
                    // tangent the differentiable adjoint needs; the global tangent
                    // stays PD via material stiffness.
                    let n = prim.grad(p);
                    let block: Matrix3<f64> =
                        c.d2_energy_d_sd2 * (n * n.transpose()) + c.d_energy_d_sd * prim.hessian(p);
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
            // Under an infinitesimal rigid motion (spatial twist `(ω, v)`; the
            // primitive's velocity at the fixed world point `p` is `u = v + ω×p`)
            // the per-vertex residual derivative is
            // `d²E/dsd²·(∂sd/∂s)·n̂ + (dE/dsd)·δn̂` (`d²E/dsd² = κ·b''`), with
            //   `∂sd/∂s = p·(ω×n̂) − v·n̂`   (= `−u·n̂`)
            //   `δn̂     = ω×n̂ − H·u`        (`H = ∇²sd`, the [`Sdf::hessian`])
            // The normal both ROTATES with the primitive (`ω×n̂`) and CONVECTS as
            // the contact point slides over a CURVED primitive (`−H·u`, rung 8c —
            // reconciled with `PenaltyRigidContact`). For a PLANE `H = 0` and a
            // pure translation `ω = 0`, recovering `κ·b''·(−n̂·dir)·n̂` exactly.
            let n = prim.grad(p);
            let omega_x_n = twist.angular.cross(&n);
            let dsd = p.coords.dot(&omega_x_n) - twist.linear.dot(&n);
            let u = twist.linear + twist.angular.cross(&p.coords);
            let dn = omega_x_n - prim.hessian(p) * u;
            ContactGradient {
                contributions: vec![(
                    vertex_id,
                    c.d2_energy_d_sd2 * dsd * n + c.d_energy_d_sd * dn,
                )],
            }
        })
    }

    fn normal_curvature(&self, pair: &ContactPair, positions: &[Vec3]) -> Matrix3<f64> {
        let ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair
        else {
            // A `Face` pair only reaches `normal_curvature` through the friction
            // path, which is frictionless-guarded (fail-loud) for face contact
            // (rung 8b) — so this is unreachable in the current wiring. Face
            // friction (with its own face normal-curvature) is a deferred rung.
            unreachable!(
                "ContactPair::Face normal_curvature is unreachable (face contact is \
                 frictionless-guarded); face-friction reconciliation is a deferred rung"
            )
        };
        // The curvature of the contact-FORCE direction `∂n̂/∂x_v` for
        // `n̂ = force.normalize() = sign(dE/dsd)·n̂_sdf`, so `∂n̂/∂x_v =
        // sign(dE/dsd)·∇²sd`. Same `∇²sd` the Hessian folds into `b'·∇²sd`; the
        // sign is load-bearing because the friction normal-rotation Jacobian
        // chains w.r.t. that exact force direction (rung 8c — matches
        // `PenaltyRigidContact::normal_curvature`). Zero off the active band.
        let p = Point3::from(positions[*vertex_id as usize]);
        let prim = &self.primitives[*primitive_id as usize];
        self.barrier(prim.eval(p)).map_or_else(Matrix3::zeros, |c| {
            c.d_energy_d_sd.signum() * prim.hessian(p)
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

    // ---- Rung 8c: the reconciled curved vertex path (Hessian / pose /
    // normal_curvature) FD-gated directly on a sphere, so no lifted `∇²sd` term
    // ships unverified. `∇²sd = 0` for a plane makes each a `+0`, so these gate
    // exactly the new curved contribution. ----

    use crate::sdf_bridge::{SphereSdf, TranslatedSdf};

    const SPHERE_R: f64 = 0.12;
    const SP_KAPPA: f64 = 1.0e4;
    const SP_DHAT: f64 = 0.05;

    /// A vertex off-axis over a sphere at `centre`, at a gap inside the band, so
    /// `n̂` and `∇²sd` are non-trivial (off-diagonal `∇²sd`).
    fn sphere_vertex(centre: Vec3) -> Vec3 {
        centre + Vec3::new(0.018, -0.011, SPHERE_R + 0.020)
    }

    /// IPC vertex Hessian on a sphere matches a central FD of its gradient — the
    /// gate on the lifted `b'·∇²sd` term. Non-vacuous: also asserts the flat
    /// (`b''·n̂n̂ᵀ` only) block misses the FD by far more than the full block.
    // Conventional single-letter names for the geometry/FD loop (vertex `v`,
    // Hessian `h`, normal `n`, perturbed `vp`/`vm`, axes `d`/`e`) read clearest;
    // renaming them would obscure the finite-difference structure.
    #[allow(clippy::many_single_char_names)]
    #[test]
    fn vertex_hessian_matches_fd_on_sphere() {
        let contact =
            IpcRigidContact::with_params(vec![SphereSdf { radius: SPHERE_R }], SP_KAPPA, SP_DHAT);
        let v = sphere_vertex(Vec3::zeros());
        let pair = ContactPair::Vertex {
            vertex_id: 0,
            primitive_id: 0,
        };
        let grad_force = |x: Vec3| -> Vec3 { contact.gradient(&pair, &[x]).contributions[0].1 };
        let h = contact.hessian(&pair, &[v]).contributions[0].2;
        let eps = 1e-7;
        let mut max_diff = 0.0f64;
        let mut max_mag = 0.0f64;
        for e in 0..3 {
            let (mut vp, mut vm) = (v, v);
            vp[e] += eps;
            vm[e] -= eps;
            let fd = (grad_force(vp) - grad_force(vm)) / (2.0 * eps);
            for d in 0..3 {
                assert!(h[(d, e)].is_finite() && fd[d].is_finite());
                max_diff = max_diff.max((h[(d, e)] - fd[d]).abs());
                max_mag = max_mag.max(h[(d, e)].abs());
            }
        }
        assert!(
            max_diff < 1e-5 * max_mag,
            "IPC vertex Hessian on a sphere must match FD: {max_diff:e} vs {max_mag:e}",
        );
        // Non-vacuity: the block is INDEFINITE (a negative eigenvalue). The flat
        // `b''·n̂n̂ᵀ` term is PSD rank-1 and can NEVER be indefinite, so a negative
        // eigenvalue proves the curved `b'·∇²sd` term (`b' < 0`, `∇²sd` PSD) is
        // present and non-negligible.
        let eigs = h.symmetric_eigenvalues();
        assert!(
            eigs.min() < 0.0,
            "curved vertex Hessian must be indefinite (flat n̂n̂ᵀ is PSD), eigs = {eigs:?}",
        );
    }

    /// IPC vertex pose derivative (translation twist) matches a central FD of the
    /// residual under translating the sphere — gates the lifted `−H·u` term.
    #[test]
    fn vertex_pose_derivative_matches_fd_on_sphere_translation() {
        let centre = Vec3::new(0.0, 0.0, 0.0);
        let v = sphere_vertex(centre);
        let dir = Vec3::new(0.4, -0.6, 0.7).normalize();
        let sphere = |offset: Vec3| TranslatedSdf {
            inner: SphereSdf { radius: SPHERE_R },
            offset,
        };
        let contact = IpcRigidContact::with_params(vec![sphere(centre)], SP_KAPPA, SP_DHAT);
        let pair = ContactPair::Vertex {
            vertex_id: 0,
            primitive_id: 0,
        };
        // Analytic: residual sensitivity to a pure translation of the primitive.
        let an = contact
            .pose_residual_derivative(&pair, &[v], RigidTwist::translation(dir))
            .contributions[0]
            .1;
        // FD: translate the sphere offset by ±ε·dir, re-read the residual (gradient).
        let eps = 1e-7;
        let residual = |offset: Vec3| -> Vec3 {
            IpcRigidContact::with_params(vec![sphere(offset)], SP_KAPPA, SP_DHAT)
                .gradient(&pair, &[v])
                .contributions[0]
                .1
        };
        let fd = (residual(centre + eps * dir) - residual(centre - eps * dir)) / (2.0 * eps);
        let diff = (an - fd).norm();
        let mag = an.norm().max(fd.norm());
        assert!(
            diff < 1e-5 * mag.max(1e-6),
            "IPC vertex pose derivative on a sphere must match FD: {diff:e} vs mag {mag:e}",
        );
    }

    /// IPC `normal_curvature` on a sphere equals `∂(force.normalize())/∂x_v` by
    /// FD — gates the overridden `sign(b')·∇²sd` (was the default zeros).
    #[test]
    fn vertex_normal_curvature_matches_fd_on_sphere() {
        let contact =
            IpcRigidContact::with_params(vec![SphereSdf { radius: SPHERE_R }], SP_KAPPA, SP_DHAT);
        let v = sphere_vertex(Vec3::zeros());
        let pair = ContactPair::Vertex {
            vertex_id: 0,
            primitive_id: 0,
        };
        let force_dir =
            |x: Vec3| -> Vec3 { contact.gradient(&pair, &[x]).contributions[0].1.normalize() };
        let c = contact.normal_curvature(&pair, &[v]);
        let eps = 1e-7;
        let mut max_diff = 0.0f64;
        let mut max_mag = 0.0f64;
        for e in 0..3 {
            let (mut vp, mut vm) = (v, v);
            vp[e] += eps;
            vm[e] -= eps;
            let fd = (force_dir(vp) - force_dir(vm)) / (2.0 * eps);
            for d in 0..3 {
                max_diff = max_diff.max((c[(d, e)] - fd[d]).abs());
                max_mag = max_mag.max(c[(d, e)].abs());
            }
        }
        assert!(
            max_diff < 1e-5 * max_mag.max(1e-6),
            "IPC normal_curvature on a sphere must match FD: {max_diff:e} vs {max_mag:e}",
        );
        // And it is non-zero (was the default zeros before 8c).
        assert!(
            max_mag > 1.0,
            "normal_curvature must be non-trivial, got {max_mag:e}"
        );
    }

    /// A PLANE keeps every curved term at exactly `∇²sd = 0` — the pre-8c path,
    /// bit-for-bit. `normal_curvature` is the zero matrix; the Hessian is the
    /// pure rank-1 `b''·n̂n̂ᵀ`.
    #[test]
    fn plane_curved_terms_are_exactly_zero() {
        let plane = super::super::rigid::RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let contact = IpcRigidContact::with_params(vec![plane], SP_KAPPA, SP_DHAT);
        let v = Vec3::new(0.01, -0.02, 0.4 * SP_DHAT); // inside the band above the plane
        let pair = ContactPair::Vertex {
            vertex_id: 0,
            primitive_id: 0,
        };
        let nc = contact.normal_curvature(&pair, &[v]);
        assert_eq!(
            nc,
            Matrix3::zeros(),
            "plane normal_curvature must be exactly zero"
        );
        // With the curved term at exactly `∇²sd = 0`, the Hessian is the pure
        // rank-1 `b''·n̂n̂ᵀ` with `n̂ = ẑ` — i.e. `diag(0, 0, h_zz)`. Every off-`z`
        // entry must be BIT-exactly zero (a leaked curvature term would perturb
        // them); `h_zz` is the only non-zero entry.
        let h = contact.hessian(&pair, &[v]).contributions[0].2;
        for d in 0..3 {
            for e in 0..3 {
                if d == 2 && e == 2 {
                    assert!(h[(2, 2)] > 0.0, "b''·n̂n̂ᵀ z-block must be positive");
                } else {
                    assert_eq!(
                        h[(d, e)].to_bits(),
                        0.0_f64.to_bits(),
                        "plane Hessian off-z entry ({d},{e}) must be exactly zero",
                    );
                }
            }
        }
    }
}
