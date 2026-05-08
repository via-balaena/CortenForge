//! Penalty rigid↔soft contact — first force-bearing
//! [`ContactModel`] impl on `sim-soft`.
//!
//! One-way coupling (rigid kinematic; soft side feels the force).
//! Penalty is a stepping stone to IPC, not a production baseline. The
//! structural failure modes documented in book Part 4 §00 §00 —
//! active-set discontinuity at `d = d̂`, parameter sensitivity,
//! oscillation pathology near the boundary — stay valid; §00 §00's
//! "no penalty even as a baseline" commitment is narrowed to "no
//! penalty as a *production* baseline" (this implementation is
//! deliberately scoped to rigid↔soft co-sim plumbing, first-time
//! [`ContactModel`] wiring into the Newton hot path, and the Hertzian
//! sphere↔plane analytic gate at `tests/hertz_sphere_plane.rs`).
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
//!   returns [`f64::INFINITY`]. Phase H IPC delivers proper CCD.

use super::{ContactGradient, ContactHessian, ContactModel, ContactPair, ContactPairReadout};
use crate::{
    Vec3,
    mesh::{Mesh, VertexId},
    sdf_bridge::Sdf,
};
use nalgebra::{Matrix3, Point3};

/// Default penalty stiffness (N/m). Middle of the recommended
/// `1e3..1e5` range — at Ecoflex-class material `E ≈ 200 kPa` and
/// `h ≈ 5 mm` element edge, element stiffness `E·h ≈ 1e3 N/m`; κ at
/// 10× element stiffness is the "stiff but Newton-convergent" regime.
/// Fixtures may tune locally via [`PenaltyRigidContact::with_params`]
/// (see `tests/penalty_compressive_block.rs` and
/// `tests/hertz_sphere_plane.rs`).
pub(crate) const PENALTY_KAPPA_DEFAULT: f64 = 1.0e4;

/// Default contact band (m). 1 mm — ~20× the expected Hertz
/// indentation `δ ≈ 5e-5 m` at the canonical R = 1 cm soft sphere ×
/// Ecoflex-class composite, so the active-set band cleanly contains
/// the contact patch without pulling in spurious distant-vertex
/// pairs.
pub(crate) const PENALTY_DHAT_DEFAULT: f64 = 1.0e-3;

/// Penalty contact between soft-body vertices and a set of kinematic
/// rigid primitives.
///
/// One [`ContactPair::Vertex`] per `(soft vertex, rigid primitive)`
/// whose signed distance is below the contact band `d̂`. See the
/// [module docs](self) for the energy / gradient / Hessian formulas
/// and sign conventions.
///
/// Primitives are heap-erased [`Sdf`] trait objects so a single contact
/// model can compose mixed primitive types (planes, spheres,
/// scan-derived `mesh_sdf::SignedDistanceField`, cf-design `Solid`s)
/// in one list. The constructors are generic over any `IntoIterator`
/// of `Sdf + 'static`, so a homogeneous `Vec<RigidPlane>` flows
/// through without explicit boxing.
pub struct PenaltyRigidContact {
    primitives: Vec<Box<dyn Sdf>>,
    kappa: f64,
    d_hat: f64,
}

impl PenaltyRigidContact {
    /// Construct with default `(κ, d̂)` from `PENALTY_KAPPA_DEFAULT` /
    /// `PENALTY_DHAT_DEFAULT` (crate-private; tunable only via
    /// [`with_params`](Self::with_params) for fixture-local tuning).
    #[must_use]
    pub fn new<I>(primitives: I) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        Self::with_params(primitives, PENALTY_KAPPA_DEFAULT, PENALTY_DHAT_DEFAULT)
    }

    /// Construct with non-default `(κ, d̂)` — testing surface for
    /// fixture-local overrides (see
    /// `tests/penalty_compressive_block.rs`,
    /// `tests/hertz_sphere_plane.rs`, `tests/contact_grad_hook.rs`).
    /// User-facing parameter tuning is a future-phase follow-on;
    /// production scenes go through [`new`](Self::new).
    #[must_use]
    pub fn with_params<I>(primitives: I, kappa: f64, d_hat: f64) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
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

    /// Per-active-pair readout — for every `(soft vertex, rigid
    /// primitive)` pair in the contact band, emit a
    /// [`ContactPairReadout`] with the vertex position, signed distance,
    /// outward primitive normal, and the penalty force on the soft side
    /// at the readout-time `positions`.
    ///
    /// Mirrors [`active_pairs`](Self::active_pairs)'s walk order
    /// (vertices outer × primitives inner) and band gate (`sd < d̂`),
    /// so the returned vec is the same length as `active_pairs(...)`
    /// at the same `positions` and the readouts appear in the same
    /// order.
    ///
    /// `force_on_soft` resolves to `+κ·(d̂ − sd)·n` per the type docs'
    /// sign convention — a bit-equivalent reproduction of the energy
    /// gradient [`ContactModel::gradient`] returns (`−κ·(d̂ − sd)·n`),
    /// negated by the force-as-`−∇U` identity. Row 18
    /// (`contact-force-readout`) is the canonical consumer; row 14
    /// (`compressive-block`) reconstructs this surface inline from
    /// known plane geometry, predating this method.
    // `vid as VertexId` and `pid as u32` mirror `active_pairs`'s `Vec`-
    // iteration index packing — bounded by mesh / primitive counts that
    // fit in `u32` for any Phase 5 scene.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn per_pair_readout(
        &self,
        _mesh: &dyn Mesh,
        positions: &[Vec3],
    ) -> Vec<ContactPairReadout> {
        let mut readouts = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                let sd = prim.eval(p_pt);
                if sd < self.d_hat {
                    let normal = prim.grad(p_pt);
                    let force_on_soft = self.kappa * (self.d_hat - sd) * normal;
                    readouts.push(ContactPairReadout {
                        pair: ContactPair::Vertex {
                            vertex_id: vid as VertexId,
                            primitive_id: pid as u32,
                        },
                        position: p,
                        sd,
                        normal,
                        force_on_soft,
                    });
                }
            }
        }
        readouts
    }
}

/// Filter `per_pair_readout` results to vertices in the given referenced
/// set, dropping orphan BCC lattice corners that are not in any tet.
///
/// [`PenaltyRigidContact::per_pair_readout`] returns one
/// [`ContactPairReadout`] for every body vertex in the contact band
/// (`sd < d̂`) regardless of whether that vertex is referenced by any
/// tet. Orphans sit at their rest BCC-lattice positions and are
/// excluded from the solver's free-DOF set; their inclusion in
/// readouts is a deterministic regression gate but pollutes
/// physically-meaningful aggregates. For probe-inside-cavity
/// geometries the orphan share can dominate at 95-97 % of readouts
/// (rows 21 + 22 silicone-sleeve precedent — see pattern (xx) at
/// `project_sim_soft_row_22_patterns.md`).
///
/// Returns a fresh `Vec` containing only the referenced-vertex readouts
/// in the same order as the input. Pass `referenced_vertices(&mesh)`
/// (or any subset thereof) for the `referenced` slice.
#[must_use]
pub fn filter_pair_readouts_to_referenced(
    readouts: Vec<ContactPairReadout>,
    referenced: &[VertexId],
) -> Vec<ContactPairReadout> {
    let set: std::collections::BTreeSet<VertexId> = referenced.iter().copied().collect();
    readouts
        .into_iter()
        .filter(|r| match r.pair {
            ContactPair::Vertex { vertex_id, .. } => set.contains(&vertex_id),
        })
        .collect()
}

impl ContactModel for PenaltyRigidContact {
    /// Walks soft vertices outer (`0..positions.len()`) × rigid
    /// primitives inner (`0..self.primitives.len()`); emits a
    /// [`ContactPair::Vertex`] for every `(v, p)` whose signed
    /// distance is below the band `d̂`. Order is deterministic — no
    /// sort, no `HashMap`, no rayon.
    // `vid as VertexId` and `pid as u32` are `Vec`-iteration indices;
    // in practice bounded by mesh / primitive counts that fit
    // comfortably in `u32`. The `as` cast matches the convention used
    // in `mesh/hand_built.rs` for `VertexId` packing. A theoretical
    // overflow on a 64-bit pointer would surface as wrapped indices;
    // not load-bearing for Phase 5 mesh sizes.
    #[allow(clippy::cast_possible_truncation)]
    fn active_pairs(&self, _mesh: &dyn Mesh, positions: &[Vec3]) -> Vec<ContactPair> {
        let mut pairs = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                if prim.eval(p_pt) < self.d_hat {
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
        let p = Point3::from(positions[vertex_id as usize]);
        let d = self.primitives[primitive_id as usize].eval(p);
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
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.eval(p);
        if d >= self.d_hat {
            ContactGradient::default()
        } else {
            let n = prim.grad(p);
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
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.eval(p);
        if d >= self.d_hat {
            ContactHessian::default()
        } else {
            let n = prim.grad(p);
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
