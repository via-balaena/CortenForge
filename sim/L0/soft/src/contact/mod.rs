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

pub mod friction;
pub mod ipc;
pub mod null;
pub mod penalty;
pub mod rigid;

pub use ipc::IpcRigidContact;
pub use null::NullContact;
pub use penalty::{PenaltyRigidContact, filter_pair_readouts_to_referenced};
pub use rigid::RigidPlane;

/// A pair of geometric primitives active under the contact model.
///
/// Currently the [`Vertex`](Self::Vertex) variant only — a soft-body
/// vertex against a rigid primitive, the one-way coupling case (rigid
/// kinematic; soft side feels the force). A future IPC upgrade will add
/// `EdgeEdge` / `VertexFace` variants for self-contact; the
/// single-variant enum is deliberately heavier than a struct to avoid a
/// breaking change at that handoff.
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

/// Per-active-pair readout — contact geometry plus the force the
/// contact model exerts on the soft side.
///
/// Bundles the outputs of a single active-pair evaluation for
/// downstream readout consumers (the row 18 `contact-force-readout`
/// example, future calibration loops). Sister of [`ContactGradient`] /
/// [`ContactHessian`] at the human-facing readout layer: those return
/// the data the *solver* needs to assemble a residual / tangent;
/// `ContactPairReadout` returns the data a *user* needs to inspect
/// what's happening at the contact interface.
///
/// Capture timing is on-demand — readout consumers pass current
/// positions and the producer recomputes geometry + force from scratch,
/// matching the on-demand semantics of [`ActivePairsFor::active_pairs`].
/// No per-iter cache is maintained.
///
/// **Sign convention** — `force_on_soft` is the force the contact
/// model exerts on the soft side at this pair, equal to `-gradient` of
/// the contact energy at the contacted vertex. For the penalty case
/// with outward primitive normal `n`, this resolves to
/// `+κ·(d̂-sd)·n` (positive scalar times outward normal: the soft body
/// is pushed *away* from the rigid surface, restoring the active config
/// back to the inactive band). The Newton's-3rd-law reaction on the
/// rigid side is `-force_on_soft`.
#[derive(Clone, Debug)]
pub struct ContactPairReadout {
    /// The active pair this readout describes.
    pub pair: ContactPair,
    /// Position of the contacted soft vertex (or relevant geometric
    /// point for future IPC variants) at the readout-time configuration.
    pub position: Vec3,
    /// Signed distance from `position` to the rigid primitive at
    /// readout time. Values strictly less than `d̂` are the active
    /// regime — matching the gate at
    /// [`ActivePairsFor::active_pairs`]; the producer only emits
    /// readouts for active pairs.
    pub sd: f64,
    /// Outward-pointing unit normal of the rigid primitive evaluated
    /// at `position`.
    pub normal: Vec3,
    /// Force the contact model exerts on the soft side at this pair —
    /// see "Sign convention" in the type docs.
    pub force_on_soft: Vec3,
    /// Tributary surface area carried by the contacted vertex on the
    /// deformed boundary — the barycentric lumped area
    /// `⅓·Σ area(f)` over boundary triangles incident to the vertex
    /// (see [`boundary_vertex_areas`](crate::boundary_vertex_areas)),
    /// evaluated at the same readout-time `positions`.
    ///
    /// `0.0` when the contacted vertex lies on no boundary face — an
    /// *interior* vertex that entered the contact band (the
    /// `per_pair_readout` walk tests every vertex, not only boundary
    /// ones), or a degenerate/collapsed surface triangle. This is the
    /// off-nominal case that drives the [`pressure`](Self::pressure)
    /// `NaN` sentinel below.
    pub tributary_area: f64,
    /// Contact pressure at this pair — `|force_on_soft| / tributary_area`,
    /// the force spread over the vertex's deformed surface patch. The
    /// measurable that distinguishes a broad contact (high force, low
    /// pressure) from a concentrated one (low force, high pressure).
    ///
    /// **`NaN` when [`tributary_area`](Self::tributary_area) is `0.0`** —
    /// there is no surface patch over which to define pressure, so the
    /// value is genuinely undefined rather than zero. A `NaN` sentinel
    /// (over a falsely-reassuring `0.0`) is deliberate for a safety
    /// readout: zero-area contacts are off-nominal (an interior vertex
    /// penetrating, or a diverged-solve degeneracy), exactly where a
    /// reported `0.0` would mask danger. Mirrors the `f64::NAN`
    /// sentinel convention in [`reward_breakdown`](crate::Observable::reward_breakdown).
    /// Peak-pressure consumers must filter non-finite values. Pressure
    /// scales as `1/area`, so a *non-zero but tiny* near-degenerate
    /// triangle yields a correspondingly large (finite) value — guarding
    /// genuine geometric degeneracy is the soft solver's validity
    /// responsibility upstream, not this readout's.
    ///
    /// **Per-pair = per-contact-face.** This is the normal stress *this*
    /// primitive imposes on the contacted vertex's surface patch —
    /// `|force_on_soft|` divided by the vertex's tributary area. A vertex
    /// touching several primitives at once (e.g. a pinch between two
    /// jaws) emits one readout per primitive, each the real stress of one
    /// contact face. The peak load on the patch is the *max* over those
    /// faces, not a sum of their forces (summing would cancel a pinch's
    /// opposing forces and double-count co-directed ones) —
    /// [`peak_contact_pressure`] is the reduction that takes that max.
    ///
    /// The triple `pressure · tributary_area · normal` reconstructs
    /// `force_on_soft` for every well-defined (area `> 0`) pair — exact
    /// because `normal` is the unit primitive normal (its field
    /// contract) and the contact force is parallel to it — so
    /// `Σ pressure·area·n̂ == Σ force_on_soft` over a boundary contact,
    /// the faithful-decomposition invariant. (This sum is unaffected by
    /// the per-pair caveat above: the full area cancels in each term.)
    pub pressure: f64,
}

/// Contact pressure from a per-pair force and its tributary area —
/// `|force| / area`, with a `f64::NAN` sentinel when `area == 0.0`.
///
/// The single definition behind [`ContactPairReadout::pressure`],
/// shared by every `per_pair_readout` producer so the zero-area
/// sentinel convention lives in one place. See the
/// [`pressure`](ContactPairReadout::pressure) field docs for why the
/// degenerate case is `NaN` rather than `0.0`.
#[must_use]
pub(crate) fn contact_pressure(force_on_soft: Vec3, tributary_area: f64) -> f64 {
    if tributary_area > 0.0 {
        force_on_soft.norm() / tributary_area
    } else {
        f64::NAN
    }
}

/// Peak contact pressure (stress) over a set of active-pair readouts —
/// the maximum per-pair [`ContactPairReadout::pressure`].
///
/// Contact pressure is a **per-contact-face** quantity: each
/// `(vertex, primitive)` pair already carries the normal stress that
/// primitive imposes on the vertex's surface patch (`|force_on_soft| /
/// tributary_area`). The peak load a patch experiences is therefore the
/// max over its contacting faces — *not* a sum of their forces. Summing
/// would be wrong in both directions: for a pinch between two opposing
/// jaws the per-vertex force *vectors* nearly cancel (`|f_A + f_B| ≈ 0`)
/// and would report ~zero pressure for a maximally-squeezed vertex —
/// the exact danger a safety readout must never miss — while for
/// co-directed faces it double-counts. Taking the max of the per-face
/// stresses reports the true highest local concentration in every case.
///
/// Non-finite per-pair pressures are filtered: a zero-tributary-area
/// pair (an interior vertex in the band, or a degenerate patch) carries
/// the `NaN` sentinel and does not participate, so a finite contact
/// alongside a degenerate one still reports the real stress.
///
/// Return value, consistent with the per-pair `NaN`-sentinel altitude:
/// - `0.0` when there are **no** readouts — genuinely no contact, safe.
/// - `f64::NAN` when readouts are present but **every** pressure is
///   non-finite (all contacts degenerate) — off-nominal and undefined,
///   *not* a falsely-reassuring `0.0` indistinguishable from no contact.
/// - otherwise the maximum finite per-pair pressure.
///
/// Order-independent: `max` over the (NaN-free) finite pressures does
/// not depend on readout order.
#[must_use]
pub fn peak_contact_pressure(readouts: &[ContactPairReadout]) -> f64 {
    if readouts.is_empty() {
        return 0.0; // no contact — safe identity, distinct from degenerate
    }
    let peak = readouts
        .iter()
        .map(|r| r.pressure)
        .filter(|p| p.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);
    // `NEG_INFINITY` survives only when no finite pressure was folded in
    // — contacts present but all degenerate ⇒ undefined (NaN), not 0.0.
    if peak.is_finite() { peak } else { f64::NAN }
}

/// Infinitesimal rigid motion (spatial twist) of a contact primitive —
/// the general pose-rate that [`ContactModel::pose_residual_derivative`]
/// differentiates the contact residual against.
///
/// A rigid primitive moving with angular velocity `angular` (ω) and
/// linear velocity `linear` (v, the velocity of the primitive's material
/// point instantaneously at the world origin) carries the material point
/// at world location `p` with velocity `ξ(p) = v + ω × p`. For the plane
/// primitive this rotates the outward unit normal by `δn̂ = ω × n̂` and
/// shifts the offset by `δoffset = v · n̂` — the reference-point cross
/// terms cancel (`(ω×x)·n̂ + x·(ω×n̂) = 0`), so the offset rate depends
/// only on the linear part. See `docs/keystone/rotating_normal_recon.md`.
///
/// The keystone S3 soft-pose adjoint was scoped to a pure *translation*
/// (`angular = 0`, constant normal `δn̂ = 0`); a rotating contact normal
/// (`angular ≠ 0`) is the rotating-normal leaf. [`Self::translation`]
/// reduces the pose-residual derivative to the original constant-normal
/// form exactly (`δn̂ = 0`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidTwist {
    /// Angular velocity ω of the primitive (per unit pose parameter).
    /// Rotates the primitive's normal: `δn̂ = ω × n̂`.
    pub angular: Vec3,
    /// Linear velocity v of the primitive's material point at the world
    /// origin (per unit pose parameter). Shifts a plane's offset:
    /// `δoffset = v · n̂`.
    pub linear: Vec3,
}

impl RigidTwist {
    /// A pure translation of the primitive along `dir` — `angular = 0`,
    /// `linear = dir`. The keystone S3 translation-only pose; the soft
    /// pose-residual derivative reduces to its original constant-normal
    /// form (`∂n̂/∂δ = 0`).
    #[must_use]
    pub fn translation(dir: Vec3) -> Self {
        Self {
            angular: Vec3::zeros(),
            linear: dir,
        }
    }

    /// A unit-rate rotation of the primitive about the world `axis`
    /// through the `pivot` point — `angular = axis`,
    /// `linear = −axis × pivot` (so the material point at `pivot` is
    /// instantaneously stationary). Tilts a plane's normal about a fixed
    /// pivot on the plane.
    #[must_use]
    pub fn rotation_about(axis: Vec3, pivot: Vec3) -> Self {
        Self {
            angular: axis,
            linear: -axis.cross(&pivot),
        }
    }
}

/// Contact-energy surface over candidate pairs. `dyn`-compatible at
/// scene construction; monomorphized `C: ContactModel` on the hot path.
///
/// The material-agnostic part — energy, gradient, Hessian, CCD —
/// lives here. Active-pair selection (which depends on mesh
/// topology, hence on `M` via `&dyn Mesh<M>`) lives in the
/// [`ActivePairsFor`] subtrait. The split keeps method-call sites
/// like `c.hessian(...)` from needing M-inference at every test
/// caller.
pub trait ContactModel: Send + Sync {
    /// Contact energy contribution for a single pair (J).
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64;

    /// Contact gradient contribution for a single pair.
    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient;

    /// Contact Hessian contribution for a single pair.
    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian;

    /// Continuous-collision time of impact along the segment `x0 → x1`.
    /// `f64::INFINITY` means no contact within the step.
    fn ccd_toi(&self, pair: &ContactPair, x0: &[Vec3], x1: &[Vec3]) -> f64;

    /// Sensitivity of this pair's contact-residual contribution to an
    /// infinitesimal *rigid motion* of its primitive — the spatial
    /// [`RigidTwist`] `(ω, v)` — `∂(∂E/∂x_pair)/∂s`, the per-vertex
    /// 3-vector that adds into the global residual derivative `∂r/∂s`.
    ///
    /// This is the kinematic-pose analog of [`Self::gradient`]: where
    /// `gradient` differentiates the contact residual w.r.t. the soft
    /// vertex position, this differentiates it w.r.t. moving the rigid
    /// obstacle. It is the keystone S3 missing factor — the soft solver's
    /// IFT machinery (`∂x*/∂θ = −A⁻¹∂r/∂θ`) is otherwise load-only and
    /// cannot see the contact-plane pose (baked in at construction).
    ///
    /// Default: empty (a pose-independent / kinematic-free contact such
    /// as [`NullContact`] contributes nothing). [`PenaltyRigidContact`]
    /// overrides it. The contact residual is `g_v = (dE/dsd)·n̂` with
    /// `sd = p·n̂ − offset`; under the twist the normal rotates
    /// `δn̂ = ω×n̂` and `∂sd/∂s = p·δn̂ − v·n̂`, so
    /// `∂g_v/∂s = d²E/dsd²·(∂sd/∂s)·n̂ + (dE/dsd)·δn̂`. The second
    /// (direction) term is the rotating-normal contribution; it vanishes
    /// for a pure translation ([`RigidTwist::translation`], `ω = 0`),
    /// recovering the original constant-normal form `d²E/dsd²·(−n̂·dir)·n̂`.
    ///
    /// **Contract for new contact models**: any *pose-dependent* contact
    /// MUST override this — the default returns zero, so a forgotten
    /// override yields a silently-zero pose gradient (the IFT pose
    /// sensitivity built on it would be wrong, not merely unsupported)
    /// rather than a compile error.
    ///
    /// **Scope** — engaged regime (the active set must be stable across
    /// the pose perturbation; the penalty active-set boundary is
    /// non-smooth, IPC the deferred cure). The normal-rotation term is
    /// exact for a *plane* (`δn̂ = ω×n̂`, constant curvature); a curved
    /// primitive's intrinsic normal curvature is a documented deferral.
    /// See `docs/keystone/rotating_normal_recon.md` and
    /// `docs/keystone/s3_soft_pose_sensitivity_recon.md`.
    fn pose_residual_derivative(
        &self,
        _pair: &ContactPair,
        _positions: &[Vec3],
        _twist: RigidTwist,
    ) -> ContactGradient {
        ContactGradient::default()
    }

    /// The curvature of the contact-FORCE direction, `C = ∂n̂/∂x_v` (3×3), at this pair —
    /// where `n̂ = force.normalize()` is the unit direction of [`Self::gradient`]'s force
    /// (`force = dE/dsd·n̂_sdf`). It equals `sign(dE/dsd)·∇²sd`: the same geometric curvature
    /// [`Self::hessian`] folds into its `dE·H` stiffness and [`Self::pose_residual_derivative`]
    /// into `−H·u`, but carrying the force-direction sign so the friction-reaction curvature
    /// carry can chain it as `∂n̂/∂x = C` (state) and `∂n̂/∂pose = −C·dir` (pose) — `DN = ∂∇D/∂n̂`
    /// (`friction::normal_rotation_term`) is taken w.r.t. that exact force direction, so the sign
    /// is load-bearing (the friction tangent frame is `±n̂`-symmetric, but `DN`'s chain to `x` is
    /// not — it differentiates the ACTUAL `n̂`).
    ///
    /// Default `Matrix3::zeros()` — a constant-normal contact (the infinite plane,
    /// [`NullContact`]) contributes none, keeping the historical plane path byte-identical (the
    /// chained curved terms are then literal `+0`). [`PenaltyRigidContact`] returns
    /// `sign(dE)·(I − n̂n̂ᵀ)/‖p − c‖` for a sphere, `0` for a plane (and off the active band).
    fn normal_curvature(&self, _pair: &ContactPair, _positions: &[Vec3]) -> Matrix3<f64> {
        Matrix3::zeros()
    }
}

/// Active-pair selection for a [`ContactModel`] over a particular
/// material type `M`.
///
/// Split out from [`ContactModel`] so that `c.hessian(...)` etc.
/// don't trigger M-inference at every method call site (only
/// `active_pairs` actually depends on the mesh, hence on `M`).
/// Implemented generically over `M: Material` for both
/// [`NullContact`] and [`PenaltyRigidContact`]; the same impl carries
/// through any mesh's material model per arc memo D10.
pub trait ActivePairsFor<M: crate::material::Material>: ContactModel {
    /// Active contact pairs for the given positions.
    fn active_pairs(&self, mesh: &dyn crate::mesh::Mesh<M>, positions: &[Vec3])
    -> Vec<ContactPair>;
}

#[cfg(test)]
mod tests {
    use super::{
        ContactPair, ContactPairReadout, RigidTwist, VertexId, contact_pressure,
        peak_contact_pressure,
    };
    use crate::Vec3;

    /// Build a minimal active-pair readout for the pressure-reduction
    /// tests — only `pair` (vertex id), `force_on_soft`, and
    /// `tributary_area` matter to `peak_contact_pressure`; the geometry
    /// fields are filler, and `pressure` is set consistently for realism.
    fn readout(vid: VertexId, pid: u32, force: Vec3, area: f64) -> ContactPairReadout {
        ContactPairReadout {
            pair: ContactPair::Vertex {
                vertex_id: vid,
                primitive_id: pid,
            },
            position: Vec3::zeros(),
            sd: -0.1,
            normal: Vec3::new(0.0, 0.0, 1.0),
            force_on_soft: force,
            tributary_area: area,
            pressure: contact_pressure(force, area),
        }
    }

    /// A single contact: peak pressure is that vertex's `|force| / area`.
    #[test]
    fn peak_pressure_single_contact() {
        let rs = [readout(0, 0, Vec3::new(0.0, 0.0, 6.0), 2.0)];
        let peak = peak_contact_pressure(&rs);
        assert!((peak - 3.0).abs() < 1e-12, "expected 6/2 = 3, got {peak}");
    }

    /// The safety-critical multi-primitive case: one vertex pinched
    /// between two OPPOSING primitives. The peak is the max per-face
    /// stress (`F/area`), NOT the vector sum of the forces — which would
    /// cancel (`|+F − F| ≈ 0`) and falsely report a squeezed vertex as
    /// pressure-free.
    #[test]
    fn peak_pressure_pinch_does_not_cancel() {
        // Vertex 0 squeezed by primitive 0 (+4·ẑ) and primitive 1 (−4·ẑ),
        // tributary area 2 → each face stress |4|/2 = 2; vector sum 0.
        let rs = [
            readout(0, 0, Vec3::new(0.0, 0.0, 4.0), 2.0),
            readout(0, 1, Vec3::new(0.0, 0.0, -4.0), 2.0),
        ];
        let peak = peak_contact_pressure(&rs);
        assert!(
            (peak - 2.0).abs() < 1e-12,
            "pinch peak is the per-face stress 2, not the cancelled vector-sum 0; got {peak}",
        );
    }

    /// Across faces (vertices or primitives) the reduction takes the max
    /// per-pair pressure.
    #[test]
    fn peak_pressure_is_max_over_faces() {
        let rs = [
            readout(0, 0, Vec3::new(0.0, 0.0, 2.0), 1.0), // pressure 2
            readout(1, 0, Vec3::new(0.0, 0.0, 9.0), 1.0), // pressure 9 — the peak
            readout(2, 0, Vec3::new(0.0, 0.0, 4.0), 1.0), // pressure 4
        ];
        let peak = peak_contact_pressure(&rs);
        assert!(
            (peak - 9.0).abs() < 1e-12,
            "expected max pressure 9, got {peak}"
        );
    }

    /// A zero-area (NaN-pressure) pair is filtered out; a finite peer
    /// still wins.
    #[test]
    fn peak_pressure_filters_nonfinite() {
        let mixed = [
            readout(0, 0, Vec3::new(0.0, 0.0, 5.0), 0.0), // zero area → NaN, filtered
            readout(1, 0, Vec3::new(0.0, 0.0, 4.0), 2.0), // pressure 2
        ];
        let peak = peak_contact_pressure(&mixed);
        assert!(
            (peak - 2.0).abs() < 1e-12,
            "NaN pair filtered, finite peak 2, got {peak}"
        );
    }

    /// Altitude-consistency with the per-pair `NaN` sentinel: no contact
    /// is a safe `0.0`, but contacts that are ALL degenerate are `NaN`
    /// (undefined / off-nominal), not a danger-masking `0.0`.
    #[test]
    fn peak_pressure_empty_is_zero_but_all_degenerate_is_nan() {
        assert_eq!(
            peak_contact_pressure(&[]).to_bits(),
            0.0_f64.to_bits(),
            "no readouts → 0.0 (no contact)",
        );
        let all_degenerate = [
            readout(0, 0, Vec3::new(0.0, 0.0, 5.0), 0.0),
            readout(1, 0, Vec3::new(0.0, 0.0, 3.0), 0.0),
        ];
        assert!(
            peak_contact_pressure(&all_degenerate).is_nan(),
            "contacts present but all degenerate → NaN (off-nominal, not 0.0)",
        );
    }

    /// `contact_pressure` is `|force| / area` on the well-defined branch.
    #[test]
    fn contact_pressure_is_force_magnitude_over_area() {
        // Force of magnitude 5 (3-4-5 right triangle) over area 2.
        let force = Vec3::new(0.0, 3.0, 4.0);
        let p = contact_pressure(force, 2.0);
        assert!(
            (p - 2.5).abs() < 1e-15,
            "expected |force|/area = 5/2, got {p}"
        );
    }

    /// Zero (or negative) tributary area has no surface patch to define
    /// pressure on — the off-nominal case returns the `NaN` sentinel
    /// rather than a falsely-reassuring `0.0` or an `Inf`.
    #[test]
    fn contact_pressure_zero_area_is_nan() {
        let force = Vec3::new(1.0, 0.0, 0.0);
        assert!(
            contact_pressure(force, 0.0).is_nan(),
            "zero area must yield NaN"
        );
        assert!(
            contact_pressure(force, -1.0).is_nan(),
            "non-positive area must yield NaN"
        );
    }

    /// A zero force over a real surface patch is honest zero pressure —
    /// the `NaN` sentinel is reserved for the missing-area case.
    #[test]
    fn contact_pressure_zero_force_finite_area_is_zero() {
        let p = contact_pressure(Vec3::zeros(), 1.5);
        // Bit-compare against +0.0 (exact; sidesteps clippy::float_cmp).
        assert_eq!(
            p.to_bits(),
            0.0_f64.to_bits(),
            "zero force over positive area is zero pressure"
        );
    }

    /// `translation` is a pure linear motion: zero angular part (constant
    /// normal `δn̂ = ω×n̂ = 0`), linear part equal to the given direction.
    #[test]
    fn translation_is_pure_linear() {
        let dir = Vec3::new(0.3, -0.7, 1.1);
        let t = RigidTwist::translation(dir);
        assert_eq!(t.angular, Vec3::zeros());
        assert_eq!(t.linear, dir);
    }

    /// `rotation_about` leaves the pivot point instantaneously stationary
    /// (`ξ(pivot) = v + ω×pivot = 0`) and rotates with unit angular rate
    /// about the given axis — the contract the rotating-normal gate relies on.
    #[test]
    fn rotation_about_fixes_the_pivot() {
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let pivot = Vec3::new(0.05, 0.05, 0.099);
        let t = RigidTwist::rotation_about(axis, pivot);
        assert_eq!(t.angular, axis);
        // ξ(pivot) = linear + angular × pivot must vanish.
        let xi_pivot = t.linear + t.angular.cross(&pivot);
        assert!(
            xi_pivot.norm() < 1e-15,
            "pivot not stationary: {xi_pivot:?}"
        );
    }
}
