//! SDF-side truncated-pyramid mating-feature primitive.
//!
//! Replaces the cylinder-based registration pin + plug T-bar/T-slot
//! mechanisms with a single shared primitive (recon-1 §G-5 of
//! `docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md`). The truncated-
//! pyramid geometry — rectangular base, tapered flat sides,
//! optional first-layer chamfer band — was picked over cylinders
//! because the FDM-target floor (Bambu A1 + default settings +
//! Jayo per §G-3) prints sharp angles + straight lines + flat
//! planes more reliably than circumferential cylinder-on-cylinder
//! contact (recon-1 §G-2; the workshop user's prior art is
//! "dovetails that are arrow/trapezoid shaped" with sharp angles
//! and straight lines).
//!
//! # Paradigm-boundary placement: SDF, not mesh-CSG
//!
//! The S1 probe-spike (`design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs`,
//! commit `a218ddfb`) characterised mesh-CSG `Manifold::union` of a
//! `Manifold::hull_pts` truncated-pyramid pin against an SDF→MC
//! curved-shell host. Outcome at both production (3 mm) and
//! over-resolved (1 mm) cell sizes: the union introduces 2
//! disconnected components AND a `SelfIntersecting` F4 Critical
//! issue at the boolean junction (§G-7 BRANCH B + BRANCH C BOTH
//! fire). The §G-9 chamfer sweep `{0.0, 0.4, 0.6, 0.8, 1.0 mm}`
//! fails identically — the failure is **chamfer-independent**
//! and lives at the recon-4 (P) "bulk-welded SDF/MC × fine mesh-CSG"
//! paradigm boundary (see [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).
//!
//! The §G-12 #2 bail-out (pin composed pre-MC into the host SDF via
//! [`cf_design::Solid::union`]) PASSES the BRANCH-A criteria — 1
//! connected component, no new F4 Critical types at the junction.
//! That outcome is the architectural correction that picks the
//! revised §G-13 implementation arc: the `PrismaticPin` primitive
//! lives entirely SDF-side. There are no
//! `MatingTransform::UnionPrismaticPin` /
//! `MatingTransform::SubtractPrismaticPin` variants; the S3
//! cup-pin and S4 plug-floor-lock sessions compose
//! the SDF into the per-piece [`cf_design::Solid`] at the assembly
//! layer (the same column as the recon-4 (P) §F-3 SDF-pin pattern
//! that turned out load-bearing for `CylinderParent` on production
//! cup pieces). The chamfer band is part of the SDF composition per
//! the revised §G-9 decision; no slicer-level chamfer fallback is
//! needed.
//!
//! # Geometry construction
//!
//! Build choice: each frustum is the **intersection of six exact
//! half-spaces** (2 axial caps + 4 sloped lateral faces). The
//! intersection of exact-SDF primitives ([`Solid::cuboid`] and
//! [`Solid::plane`]) preserves the exact-SDF property on the
//! convex result, so MC resolves the lateral faces as the exact
//! planes the spec defines — flat, sharp-edged, straight (per the
//! §G-2 design contract).
//!
//! Alternatives considered + rejected:
//!
//! - **`smooth_union` taper.** Rounds the edges and lateral faces,
//!   violates §G-2 (flat + sharp). Rejected.
//! - **Cuboid + subtractive wedge trim.** More boolean ops for the
//!   same convex result; no SDF or topology advantage. Rejected.
//! - **Single piecewise-linear extent function.** Would require a
//!   custom `FieldNode` since the lateral planes change slope at the
//!   chamfer/main boundary. The two-frustum union has identical
//!   semantics with public-API primitives only. Rejected.
//!
//! With the base chamfer enabled, the pin consists of two frustums
//! unioned along the shared `axis = -half_length + chamfer` plane:
//!
//! - **Chamfer band** (axis ∈ `[-half_length, -half_length + chamfer]`):
//!   lateral extent expands from `(base - chamfer)` at the
//!   `-axis_unit` end to `base` at the top of the band. Originally
//!   spec'd as FDM first-layer elephant-foot relief at the bed-
//!   adjacent face (recon-1 §G-6); S6 procedure.rs reclassifies the
//!   band as SDF/MC topology continuity at the deepest-in-material
//!   corner under the revised seam-face-UP cup orientation +
//!   dome-end-DOWN plug orientation. On both pin/socket pairs the
//!   `-axis_unit` end lives deep inside mating-piece material (or
//!   for the socket-side air-side carve, in empty space), never at
//!   the bed-touching first layer and never workshop-visible from
//!   outside the print. The SDF geometry is unchanged — only the
//!   workshop interpretation shifted.
//! - **Main taper** (axis ∈ `[-half_length + chamfer, +half_length]`):
//!   lateral extent shrinks from `base` at the `-axis_unit` side of
//!   the boundary to `tip` at the `+axis_unit` end — the workshop-
//!   visible self-centering wedge.
//!
//! With chamfer disabled (`base_chamfer_m == 0.0`), the chamfer band
//! is omitted and the main taper spans the full half-length.
//!
//! # Pin/socket pair convention
//!
//! [`PrismaticPinSpec`] carries the pin's nominal geometry plus
//! per-side diametral + axial clearance. The socket is the same
//! geometry with extents inflated by `clearance / 2` on each
//! corresponding axis (symmetric `/2` convention matching
//! `crate::PinSpec` — retired in §M-S4). The socket inherits the pin's chamfer band
//! so the socket mouth has a matching flared lead-in — the pin
//! seats without catching at the socket entry.
//!
//! Bit-precise fit invariant: at the spec layer, the socket's
//! derived extents differ from the pin's by exactly `clearance / 2`
//! per axis within machine epsilon (no rounding, no intermediate
//! cast; sub-ulp `a + b - a` drift from non-f64-exact decimal-mm
//! inputs is allowed). Tested in
//! `prismatic_pin_pair_extents_match_spec_clearance_within_machine_epsilon`.
//! For SDF-side primitives this is the analog of the S5
//! [`mesh_csg::build_cylinder_along_axis`][`crate::mesh_csg::build_cylinder_along_axis`]
//! `pin_and_socket_fit_invariant` bit-equality contract — the
//! cylinder primitive's contract was at the mesh-vertex level
//! (same parent → bit-equal vertex tables modulo radial scale)
//! because mesh-CSG primitives are constructed once and consumed
//! by `Manifold` ops; here the SDF kernel composes the primitive
//! lazily and the determinism contract sits at the SDF input
//! layer.

use cf_design::Solid;
use nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, UnitVector3, Vector2, Vector3};

/// Pair specification: pin geometry plus per-side clearance for the
/// matching socket. All dimensions in meters; symmetric `/2`
/// clearance convention (`socket - pin = clearance` total).
#[derive(Debug, Clone, PartialEq)]
pub struct PrismaticPinSpec {
    /// Pin base end half-extents (meters): `(lateral, binormal)`
    /// extents in the pin-local frame at `axis = -half_length` (or
    /// at `axis = -half_length + chamfer` when the chamfer band
    /// is enabled — that is the widest section). Both components
    /// must be positive and strictly greater than the matching
    /// [`Self::pin_tip_half_extents_m`] component.
    pub pin_base_half_extents_m: Vector2<f64>,
    /// Pin tip end half-extents (meters): `(lateral, binormal)`
    /// extents at `axis = +half_length`. Both components must be
    /// positive and strictly less than the matching
    /// [`Self::pin_base_half_extents_m`] component (truncated-pyramid
    /// contract — flat top, narrower than the base).
    pub pin_tip_half_extents_m: Vector2<f64>,
    /// Pin half-length along the pose axis (meters). Total pin
    /// length is `2 * pin_half_length_m`.
    pub pin_half_length_m: f64,
    /// First-layer chamfer band depth at the base end (meters).
    /// `0.0` disables the chamfer (single-frustum pin). When
    /// positive, the bed-facing face contracts to
    /// `(base - chamfer)` in each lateral axis. Must be strictly
    /// less than the smaller of the two
    /// [`Self::pin_base_half_extents_m`] components (otherwise the
    /// bed face inverts).
    pub base_chamfer_m: f64,
    /// Diametral clearance (meters). The socket's lateral extents
    /// inflate by `diametral_clearance_m / 2` per side; total
    /// socket-vs-pin lateral gap equals `diametral_clearance_m`.
    /// Symmetric `/2` convention carried over from the pre-S3 S5
    /// cylindrical-pin design (the cylinder-era `PinSpec` field
    /// migrated into this spec on S3 of the FDM-friendly geometry
    /// arc).
    pub diametral_clearance_m: f64,
    /// Axial clearance (meters). The socket's `half_length` extends
    /// by `axial_clearance_m / 2` on each axial face so the socket
    /// runs past the pin's tip face by `axial_clearance_m / 2`
    /// (relief for socket-bottom over-extrusion bulge).
    pub axial_clearance_m: f64,
}

impl PrismaticPinSpec {
    /// Default cup-pin spec from §G-6 / §G-8: 1.5 mm half-extent
    /// square base, 1.2 mm half-extent square tip (taper ratio
    /// 0.8), 3 mm half-length, 0.6 mm chamfer, 0.30 mm diametral
    /// clearance, 0.50 mm axial clearance. S7 workshop-physical
    /// calibration on Bambu A1 + default + Jayo will refine to
    /// ±0.05 mm precision.
    #[must_use]
    pub const fn cup_pin_default() -> Self {
        Self {
            pin_base_half_extents_m: Vector2::new(0.0015, 0.0015),
            pin_tip_half_extents_m: Vector2::new(0.0012, 0.0012),
            pin_half_length_m: 0.003,
            base_chamfer_m: 0.0006,
            diametral_clearance_m: 0.0003,
            axial_clearance_m: 0.0005,
        }
    }

    /// Default plug-floor-lock spec from §G-6 / §G-8: 4 mm
    /// half-extent square base, 2.8 mm half-extent square tip
    /// (taper ratio 0.7), 4 mm half-length, 0.8 mm chamfer (lead-in
    /// per §G-6; the plug's preferred print orientation places this
    /// face away from the bed so the chamfer is optional, kept here
    /// as a self-centering aid), 0.35 mm diametral clearance, 0.50
    /// mm axial clearance.
    #[must_use]
    pub const fn plug_lock_default() -> Self {
        Self {
            pin_base_half_extents_m: Vector2::new(0.004, 0.004),
            pin_tip_half_extents_m: Vector2::new(0.0028, 0.0028),
            pin_half_length_m: 0.004,
            base_chamfer_m: 0.0008,
            diametral_clearance_m: 0.00035,
            axial_clearance_m: 0.0005,
        }
    }

    /// Resolve the pin-side [`PrismaticPinParams`] for the given
    /// pose. Pin extents are the nominal spec extents.
    #[must_use]
    pub const fn pin_params(&self, pose: PrismaticPinPose) -> PrismaticPinParams {
        PrismaticPinParams {
            pose,
            base_half_extents_m: self.pin_base_half_extents_m,
            tip_half_extents_m: self.pin_tip_half_extents_m,
            half_length_m: self.pin_half_length_m,
            base_chamfer_m: self.base_chamfer_m,
        }
    }

    /// Resolve the socket-side [`PrismaticPinParams`] for the given
    /// pose. Each extent inflates by `clearance / 2`:
    ///
    /// - lateral extents (both base + tip, both axes) +=
    ///   `diametral_clearance_m / 2`
    /// - `half_length_m` += `axial_clearance_m / 2`
    /// - chamfer is unchanged (the socket inherits the same chamfer
    ///   dimension so the pin's chamfered `-axis_unit` base lines up
    ///   with the socket's chamfered `-axis_unit` mouth at the
    ///   shared deepest-engagement end of the pin/socket pair;
    ///   under the S6-revised cup orientation + dome-end-DOWN plug
    ///   orientation the band is SDF/MC topology continuity at the
    ///   deepest-in-material corner rather than the bed-adjacent
    ///   FDM-elephant-foot relief originally envisioned — see
    ///   `procedure.rs` §"First-Layer Chamfer Recipe")
    #[must_use]
    pub fn socket_params(&self, pose: PrismaticPinPose) -> PrismaticPinParams {
        let half_diametral = self.diametral_clearance_m / 2.0;
        let half_axial = self.axial_clearance_m / 2.0;
        let inflate = Vector2::new(half_diametral, half_diametral);
        PrismaticPinParams {
            pose,
            base_half_extents_m: self.pin_base_half_extents_m + inflate,
            tip_half_extents_m: self.pin_tip_half_extents_m + inflate,
            half_length_m: self.pin_half_length_m + half_axial,
            base_chamfer_m: self.base_chamfer_m,
        }
    }
}

/// World-frame pose for a `PrismaticPin`.
///
/// `axis_unit` defines the pin's long axis (the `+Y` direction in
/// pin-local coords); `lateral_unit` defines the pin-local `+X`
/// direction, which is the axis along which the `.x` component of
/// [`PrismaticPinParams::base_half_extents_m`] extends. The third
/// in-plane direction (`+Z` in pin-local) is derived as
/// `lateral_unit × axis_unit`.
///
/// For SQUARE-base pins (`base.x == base.y && tip.x == tip.y`) the
/// rotation about `axis_unit` is geometrically immaterial, but the
/// callers still pass `lateral_unit` explicitly so the rotation is
/// deterministic across calls — same pose → bit-equal rotation
/// matrix → bit-equal SDF tree.
#[derive(Debug, Clone, PartialEq)]
pub struct PrismaticPinPose {
    /// Pin centre in world coords (meters). The pin extends
    /// `±half_length_m` along `axis_unit` from this point.
    pub center_m: Point3<f64>,
    /// Pin long-axis direction (unit length). Maps from pin-local
    /// `+Y` into world coords.
    pub axis_unit: UnitVector3<f64>,
    /// Pin lateral direction (unit length). Maps from pin-local
    /// `+X` into world coords. MUST be orthogonal to `axis_unit`
    /// to within `LATERAL_ORTHOGONALITY_TOLERANCE`.
    pub lateral_unit: UnitVector3<f64>,
}

/// Maximum permitted `|axis_unit · lateral_unit|` in
/// [`PrismaticPinPose::new`].
///
/// `1e-9` is six orders of magnitude tighter than f64
/// default-precision quat conversions and rules out
/// "almost-parallel" frames that would yield a near-singular
/// rotation matrix.
pub const LATERAL_ORTHOGONALITY_TOLERANCE: f64 = 1.0e-9;

impl PrismaticPinPose {
    /// Construct a pose, asserting that `lateral_unit ⊥ axis_unit`.
    ///
    /// # Panics
    ///
    /// Panics if `|axis_unit · lateral_unit| >
    /// LATERAL_ORTHOGONALITY_TOLERANCE`.
    #[must_use]
    pub fn new(
        center_m: Point3<f64>,
        axis_unit: UnitVector3<f64>,
        lateral_unit: UnitVector3<f64>,
    ) -> Self {
        let dot = axis_unit.into_inner().dot(&lateral_unit.into_inner());
        assert!(
            dot.abs() <= LATERAL_ORTHOGONALITY_TOLERANCE,
            "PrismaticPinPose: lateral_unit must be orthogonal to axis_unit (|dot|={} > {})",
            dot.abs(),
            LATERAL_ORTHOGONALITY_TOLERANCE,
        );
        Self {
            center_m,
            axis_unit,
            lateral_unit,
        }
    }

    /// World-coord rotation that maps pin-local basis → world basis:
    /// `(+X_local, +Y_local, +Z_local) → (lateral_unit, axis_unit,
    /// lateral_unit × axis_unit)`. The third basis vector is
    /// `lateral × axis` (not `axis × lateral`) so the resulting
    /// matrix has determinant `+1` — i.e., is a proper rotation,
    /// not a reflection — which is required for
    /// [`UnitQuaternion::from_rotation_matrix`] to yield a
    /// faithful representation. (For the default identity-pose
    /// case `axis = +Y, lateral = +X`, `lateral × axis = +X × +Y =
    /// +Z` reproduces the identity basis; `axis × lateral = +Y × +X
    /// = -Z` would produce `diag(1, 1, -1)` — a reflection that
    /// silently corrupts the SDF pose.)
    fn rotation_local_to_world(&self) -> UnitQuaternion<f64> {
        let axis = self.axis_unit.into_inner();
        let lateral = self.lateral_unit.into_inner();
        let third = lateral.cross(&axis);
        let basis = Matrix3::from_columns(&[lateral, axis, third]);
        // Orthonormality is guaranteed by `new`'s orthogonality
        // assertion + `UnitVector3`'s unit-length invariants.
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(basis))
    }
}

/// Fully-resolved geometric parameters for a single `PrismaticPin`
/// emission (pin or socket side of a pair).
///
/// Typically obtained via [`PrismaticPinSpec::pin_params`] or
/// [`PrismaticPinSpec::socket_params`].
#[derive(Debug, Clone, PartialEq)]
pub struct PrismaticPinParams {
    /// World-frame pose.
    pub pose: PrismaticPinPose,
    /// Base end half-extents (meters): `(lateral, binormal)` in
    /// pin-local coords.
    pub base_half_extents_m: Vector2<f64>,
    /// Tip end half-extents (meters).
    pub tip_half_extents_m: Vector2<f64>,
    /// Half-length along the pose axis (meters).
    pub half_length_m: f64,
    /// Chamfer band depth at the base end (meters). `0.0` disables.
    pub base_chamfer_m: f64,
}

/// Build the SDF for a [`PrismaticPinParams`].
///
/// The result is the truncated-pyramid solid (with optional chamfer
/// band) in world coords. Callers compose it into a per-piece
/// [`cf_design::Solid`] via [`Solid::union`] for pin emission or
/// [`Solid::subtract`] for socket emission — the §G-12 #2
/// architecture explicitly avoids [`crate::mesh_csg::MatingTransform`]
/// for this primitive.
///
/// # Panics
///
/// Panics if any of the [`PrismaticPinParams`] dimensional
/// invariants is violated:
///
/// - non-positive or non-finite half-extents, half-length, or
///   chamfer
/// - tip half-extent ≥ base half-extent in either lateral axis
///   (the primitive degenerates to a prism, not a frustum)
/// - chamfer ≥ smaller base half-extent (the bed-facing face
///   inverts)
#[must_use]
pub fn build_prismatic_pin_sdf(params: &PrismaticPinParams) -> Solid {
    validate_params(params);
    let local = if params.base_chamfer_m > 0.0 {
        let chamfer_band = build_frustum_local(
            params.base_half_extents_m - Vector2::new(params.base_chamfer_m, params.base_chamfer_m),
            params.base_half_extents_m,
            -params.half_length_m,
            -params.half_length_m + params.base_chamfer_m,
        );
        let main_taper = build_frustum_local(
            params.base_half_extents_m,
            params.tip_half_extents_m,
            -params.half_length_m + params.base_chamfer_m,
            params.half_length_m,
        );
        main_taper.union(chamfer_band)
    } else {
        build_frustum_local(
            params.base_half_extents_m,
            params.tip_half_extents_m,
            -params.half_length_m,
            params.half_length_m,
        )
    };
    let rotation = params.pose.rotation_local_to_world();
    local
        .rotate(rotation)
        .translate(params.pose.center_m.coords)
}

/// Validate the [`PrismaticPinParams`] dimensional invariants. Kept
/// as a free function (not a method) so the assertion message
/// references field names + values uniformly across pin + socket
/// emission paths.
fn validate_params(params: &PrismaticPinParams) {
    assert!(
        params.half_length_m > 0.0 && params.half_length_m.is_finite(),
        "PrismaticPinParams::half_length_m must be positive and finite, got {}",
        params.half_length_m,
    );
    for axis in 0..2 {
        let base = params.base_half_extents_m[axis];
        let tip = params.tip_half_extents_m[axis];
        assert!(
            base > 0.0 && base.is_finite(),
            "PrismaticPinParams::base_half_extents_m[{axis}] must be positive and finite, got {base}",
        );
        assert!(
            tip > 0.0 && tip.is_finite(),
            "PrismaticPinParams::tip_half_extents_m[{axis}] must be positive and finite, got {tip}",
        );
        assert!(
            tip < base,
            "PrismaticPinParams::tip_half_extents_m[{axis}] {tip} must be strictly less than \
             base_half_extents_m[{axis}] {base} (truncated-pyramid contract)",
        );
    }
    let min_base = params
        .base_half_extents_m
        .x
        .min(params.base_half_extents_m.y);
    assert!(
        params.base_chamfer_m >= 0.0 && params.base_chamfer_m.is_finite(),
        "PrismaticPinParams::base_chamfer_m must be non-negative and finite, got {}",
        params.base_chamfer_m,
    );
    assert!(
        params.base_chamfer_m < min_base,
        "PrismaticPinParams::base_chamfer_m {} must be strictly less than the smaller base \
         half-extent {min_base} (bed face would invert)",
        params.base_chamfer_m,
    );
}

/// Build a single frustum in pin-local coords as the intersection
/// of six exact-SDF half-spaces (2 axial caps + 4 sloped lateral
/// faces).
///
/// Pin-local frame: axis along `+Y`, lateral along `+X`, binormal
/// along `+Z`. The frustum spans `axis ∈ [y_low, y_high]` with
/// lateral extents linearly interpolating from
/// `bottom_half_extents` at `y_low` to `top_half_extents` at
/// `y_high`. Both extents must be positive in each component; the
/// frustum may EXPAND or SHRINK along the axis (the chamfer band
/// expands, the main taper shrinks).
fn build_frustum_local(
    bottom_half_extents: Vector2<f64>,
    top_half_extents: Vector2<f64>,
    y_low: f64,
    y_high: f64,
) -> Solid {
    let half_axial = (y_high - y_low) / 2.0;
    let y_center = f64::midpoint(y_high, y_low);
    let max_x = bottom_half_extents.x.max(top_half_extents.x);
    let max_z = bottom_half_extents.y.max(top_half_extents.y);
    // Bounding cuboid: encloses the frustum exactly (extents = the
    // widest section in each axis). The four lateral planes carve
    // it down to the taper.
    let core = Solid::cuboid(Vector3::new(max_x, half_axial, max_z))
        .translate(Vector3::new(0.0, y_center, 0.0));
    [
        (Vector3::x(), bottom_half_extents.x, top_half_extents.x),
        (-Vector3::x(), bottom_half_extents.x, top_half_extents.x),
        (Vector3::z(), bottom_half_extents.y, top_half_extents.y),
        (-Vector3::z(), bottom_half_extents.y, top_half_extents.y),
    ]
    .into_iter()
    .fold(core, |acc, (out_dir, ext_bottom, ext_top)| {
        acc.intersect(lateral_face_plane(
            out_dir, ext_bottom, ext_top, y_low, y_high,
        ))
    })
}

/// Build the lateral-face plane for one of the frustum's four
/// sides. `out_dir` is the unit direction (in pin-local coords) the
/// face faces (one of `±X`, `±Z`); `ext_bottom` / `ext_top` are the
/// lateral extents along `out_dir` at `y_low` / `y_high`
/// respectively.
///
/// The plane passes through the two surface points
/// `(ext_bottom · out_dir + y_low · Y)` and
/// `(ext_top · out_dir + y_high · Y)` and is parallel to
/// `axis × out_dir` (the in-plane tangent perpendicular to both).
/// Outward normal in the `(out_dir, axis)` plane:
/// `(dy, -dext) / |·|` where `dy = y_high - y_low` and
/// `dext = ext_top - ext_bottom`. [`Solid::plane`] keeps the
/// half-space `normal · p < offset`, which under the outward
/// normal convention keeps the interior of the frustum.
fn lateral_face_plane(
    out_dir: Vector3<f64>,
    ext_bottom: f64,
    ext_top: f64,
    y_low: f64,
    y_high: f64,
) -> Solid {
    let dy = y_high - y_low;
    let dext = ext_top - ext_bottom;
    // 3D outward normal: `dy * out_dir + (-dext) * axis`.
    let normal = out_dir * dy - Vector3::y() * dext;
    // Offset = normal · (ext_bottom · out_dir + y_low · axis)
    //        = dy * ext_bottom - dext * y_low
    // (`Solid::plane` normalizes both `normal` and `offset` by
    // `|normal|`, so we pass the un-normalized values directly.)
    let offset = dy.mul_add(ext_bottom, -(dext * y_low));
    Solid::plane(normal, offset)
}

#[cfg(test)]
// Tests deliberately compare f64 fields by exact value — these are
// determinism contracts on emission, where any bit-difference would
// indicate a real regression (not measurement noise). The
// `assert_abs_diff_eq!` machine-epsilon variants live alongside, used
// where decimal-mm input values are not f64-exact. `let _ = ...` in
// `should_panic` tests is intentional: the test inspects the panic,
// not the return value.
#[allow(clippy::float_cmp, clippy::let_underscore_must_use)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn default_pose() -> PrismaticPinPose {
        PrismaticPinPose::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::y_axis(),
            Vector3::x_axis(),
        )
    }

    #[test]
    fn cup_pin_default_spec_fields_match_g6_g8_defaults() {
        let s = PrismaticPinSpec::cup_pin_default();
        assert_eq!(s.pin_base_half_extents_m, Vector2::new(0.0015, 0.0015));
        assert_eq!(s.pin_tip_half_extents_m, Vector2::new(0.0012, 0.0012));
        assert_eq!(s.pin_half_length_m, 0.003);
        assert_eq!(s.base_chamfer_m, 0.0006);
        assert_eq!(s.diametral_clearance_m, 0.0003);
        assert_eq!(s.axial_clearance_m, 0.0005);
    }

    #[test]
    fn plug_lock_default_spec_fields_match_g6_g8_defaults() {
        let s = PrismaticPinSpec::plug_lock_default();
        assert_eq!(s.pin_base_half_extents_m, Vector2::new(0.004, 0.004));
        assert_eq!(s.pin_tip_half_extents_m, Vector2::new(0.0028, 0.0028));
        assert_eq!(s.pin_half_length_m, 0.004);
        assert_eq!(s.base_chamfer_m, 0.0008);
        assert_eq!(s.diametral_clearance_m, 0.00035);
        assert_eq!(s.axial_clearance_m, 0.0005);
    }

    /// §G-10 #1 fit invariant: at the spec layer, socket extents
    /// differ from pin extents by exactly `clearance / 2` per
    /// axis, within machine epsilon. This is the SDF-side analog
    /// of S5's `pin_and_socket_fit_invariant` — for mesh-CSG
    /// cylinders the invariant lived at the vertex-table level
    /// (same parent → bit-equal verts modulo radial scale); for
    /// SDF-side primitives the determinism contract is at the SDF
    /// input layer because the SDF kernel composes the primitive
    /// lazily.
    ///
    /// The tolerance is at the f64-rounding floor (`1e-15` is two
    /// ulps near 1 mm-scale values). Bit-exact `assert_eq!` would
    /// fail on decimal-mm input values that are not f64-exact —
    /// e.g., `0.0015 + 0.00015 - 0.0015 != 0.00015` by one ulp.
    /// The spirit of the contract — "no rounding, no intermediate
    /// cast" — is satisfied at the ulp level; sub-ulp `a + b - a`
    /// drift is allowed.
    #[test]
    fn prismatic_pin_pair_extents_match_spec_clearance_within_machine_epsilon() {
        for spec in [
            PrismaticPinSpec::cup_pin_default(),
            PrismaticPinSpec::plug_lock_default(),
        ] {
            let pose = default_pose();
            let pin = spec.pin_params(pose.clone());
            let socket = spec.socket_params(pose);

            let half_diametral = spec.diametral_clearance_m / 2.0;
            let half_axial = spec.axial_clearance_m / 2.0;

            assert_abs_diff_eq!(
                socket.base_half_extents_m.x - pin.base_half_extents_m.x,
                half_diametral,
                epsilon = 1.0e-15,
            );
            assert_abs_diff_eq!(
                socket.base_half_extents_m.y - pin.base_half_extents_m.y,
                half_diametral,
                epsilon = 1.0e-15,
            );
            assert_abs_diff_eq!(
                socket.tip_half_extents_m.x - pin.tip_half_extents_m.x,
                half_diametral,
                epsilon = 1.0e-15,
            );
            assert_abs_diff_eq!(
                socket.tip_half_extents_m.y - pin.tip_half_extents_m.y,
                half_diametral,
                epsilon = 1.0e-15,
            );
            assert_abs_diff_eq!(
                socket.half_length_m - pin.half_length_m,
                half_axial,
                epsilon = 1.0e-15,
            );
            assert_eq!(socket.base_chamfer_m, pin.base_chamfer_m);
        }
    }

    /// SDF-evaluation axial-fit check: at the pin's tip cap face
    /// (probe at pin-local `(0, +half_length, 0)`), the socket
    /// SDF reads `-axial_clearance/2` while the pin SDF reads
    /// exactly `0` (probe is on the pin's tip cap). Their
    /// difference is `axial_clearance / 2` to f64 precision.
    ///
    /// Why probe AXIALLY rather than laterally: the lateral
    /// face slope depends on `(extent_top - extent_bottom) /
    /// (2 * half_length)` — and socket's `half_length` differs
    /// from pin's by `axial_clearance/2`, so the lateral planes have
    /// DIFFERENT slopes between pin and socket, breaking the
    /// "SDF offset equals lateral clearance" simplification. The
    /// axial cap faces are parallel (both normal to `axis_unit`)
    /// so the axial SDF offset equals the axial-cap displacement
    /// exactly. Diametral clearance correctness is covered by
    /// `prismatic_pin_pair_extents_match_spec_clearance_within_machine_epsilon`
    /// at the spec layer (the SDF input).
    #[test]
    fn socket_sdf_offset_at_tip_cap_matches_axial_clearance() {
        let spec = PrismaticPinSpec::cup_pin_default();
        let pose = default_pose();
        let pin_sdf = build_prismatic_pin_sdf(&spec.pin_params(pose.clone()));
        let socket_sdf = build_prismatic_pin_sdf(&spec.socket_params(pose));

        // Pin tip face: pin-local (0, +half_length, 0). On pin
        // surface (SDF = 0); inside socket by `axial_clearance/2`
        // (socket tip face is at `+half_length + axial_clearance/2`).
        let probe = Point3::new(0.0, spec.pin_half_length_m, 0.0);
        let pin_val = pin_sdf.evaluate(&probe);
        let socket_val = socket_sdf.evaluate(&probe);
        let expected_offset = spec.axial_clearance_m / 2.0;
        assert_abs_diff_eq!(pin_val, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(pin_val - socket_val, expected_offset, epsilon = 1.0e-12);
    }

    /// Geometric sanity: at the pin centre (interior point), the
    /// SDF reads negative with magnitude ≥ the smaller of the
    /// lateral half-extents. At a point far outside the pin, the
    /// SDF reads positive.
    #[test]
    fn pin_sdf_centre_is_interior_and_far_point_is_exterior() {
        let spec = PrismaticPinSpec::cup_pin_default();
        let params = spec.pin_params(default_pose());
        let sdf = build_prismatic_pin_sdf(&params);

        let centre = Point3::new(0.0, 0.0, 0.0);
        assert!(
            sdf.evaluate(&centre) < 0.0,
            "pin centre must be interior; SDF={}",
            sdf.evaluate(&centre),
        );

        let far = Point3::new(0.1, 0.1, 0.1);
        assert!(
            sdf.evaluate(&far) > 0.0,
            "pin far point (10 cm out) must be exterior; SDF={}",
            sdf.evaluate(&far),
        );
    }

    /// Geometric sanity: the pin's bounding box in pin-local coords
    /// equals `[-max(base.x, tip.x), +max(base.x, tip.x)] ×
    /// [-half_length, +half_length] × [-max(base.y, tip.y),
    /// +max(base.y, tip.y)]`. Tested via SDF evaluation at the
    /// 8 corner-and-just-outside-corner points; bounds-based AABB
    /// query would test the same property at the
    /// [`cf_design::Solid`] interval-evaluation surface and is
    /// noise for this primitive.
    #[test]
    fn pin_bounding_box_extents_match_widest_section() {
        let spec = PrismaticPinSpec::cup_pin_default();
        let params = spec.pin_params(default_pose());
        let sdf = build_prismatic_pin_sdf(&params);

        // Base (widest section, since chamfer band's widest equals
        // base extents exactly at `y = -half_length + chamfer`).
        let widest_x = spec.pin_base_half_extents_m.x;
        let widest_z = spec.pin_base_half_extents_m.y;
        let widest_y = spec.pin_half_length_m;
        // Just-inside the bounding box corner: SDF should still
        // read at the surface or interior. Use the +X face midway
        // up the chamfer band where the pin extends exactly to
        // `widest_x`.
        let inside = Point3::new(widest_x - 1.0e-6, -widest_y + spec.base_chamfer_m, 0.0);
        assert!(
            sdf.evaluate(&inside) <= 0.0,
            "point just inside `+X` widest face must be on/inside the pin; SDF={}",
            sdf.evaluate(&inside),
        );
        // Just-outside in lateral X: SDF positive.
        let outside_x = Point3::new(widest_x + 1.0e-4, 0.0, 0.0);
        assert!(
            sdf.evaluate(&outside_x) > 0.0,
            "point 0.1 mm outside `+X` widest face must be exterior; SDF={}",
            sdf.evaluate(&outside_x),
        );
        // Just-outside in axial Y (past the tip face): SDF positive.
        let outside_y = Point3::new(0.0, widest_y + 1.0e-4, 0.0);
        assert!(
            sdf.evaluate(&outside_y) > 0.0,
            "point 0.1 mm past the tip face must be exterior; SDF={}",
            sdf.evaluate(&outside_y),
        );
        // Just-outside in binormal Z.
        let outside_z = Point3::new(0.0, 0.0, widest_z + 1.0e-4);
        assert!(
            sdf.evaluate(&outside_z) > 0.0,
            "point 0.1 mm outside `+Z` widest face must be exterior; SDF={}",
            sdf.evaluate(&outside_z),
        );
    }

    /// Chamfer-band well-formedness across the §G-6 typed range
    /// `[0.4, 1.0] mm`: at every chamfer in the range, (a) the
    /// SDF builds without panic, (b) the bed-facing face's lateral
    /// extent equals `(base - chamfer)` to ~1 µm (sampled by
    /// finding the SDF zero-crossing along `+X` at `y =
    /// -half_length`), and (c) the chamfer-top extent equals
    /// `base` to ~1 µm.
    #[test]
    fn chamfer_band_lateral_extents_match_spec_across_g6_range() {
        const CHAMFER_SWEEP_MM: &[f64] = &[0.4, 0.5, 0.6, 0.7, 0.8, 1.0];
        let base_spec = PrismaticPinSpec::cup_pin_default();
        for &chamfer_mm in CHAMFER_SWEEP_MM {
            let spec = PrismaticPinSpec {
                base_chamfer_m: chamfer_mm * 1.0e-3,
                ..base_spec.clone()
            };
            let params = spec.pin_params(default_pose());
            let sdf = build_prismatic_pin_sdf(&params);

            let bed_y = -spec.pin_half_length_m;
            let bed_expected_extent = spec.pin_base_half_extents_m.x - spec.base_chamfer_m;
            // SDF zero-crossing along `+X` at `y = bed_y`: bisect
            // on `[0, base_half_extent + chamfer]`.
            let bed_extent = find_x_zero_crossing(&sdf, bed_y, 0.0, 0.01);
            assert_abs_diff_eq!(bed_extent, bed_expected_extent, epsilon = 1.0e-6);

            let top_y = -spec.pin_half_length_m + spec.base_chamfer_m;
            let top_expected_extent = spec.pin_base_half_extents_m.x;
            let top_extent = find_x_zero_crossing(&sdf, top_y, 0.0, 0.01);
            assert_abs_diff_eq!(top_extent, top_expected_extent, epsilon = 1.0e-6);
        }
    }

    /// Bisection: find the `+X` zero-crossing of the SDF on the
    /// slice `(x, y, 0)` for `x ∈ [lo, hi]`. Assumes
    /// `sdf(lo, y, 0) ≤ 0 < sdf(hi, y, 0)` (interior/on-surface at
    /// `lo`, exterior at `hi`). Probes are taken on a half-open
    /// "interior" predicate (`sdf ≤ +1e-12`) so that SDF values
    /// exactly on the surface from boundary-face probes (e.g.
    /// `y = bed_y` lying exactly on the cuboid's bottom cap) still
    /// move `a` upward — the strict `<= 0.0` predicate left `a`
    /// stuck whenever sub-ulp positive noise leaked into the
    /// boundary SDF and never converged toward the true
    /// zero-crossing.
    fn find_x_zero_crossing(sdf: &Solid, y: f64, lo: f64, hi: f64) -> f64 {
        let mut a = lo;
        let mut b = hi;
        for _ in 0..64 {
            let m = f64::midpoint(a, b);
            if sdf.evaluate(&Point3::new(m, y, 0.0)) <= 1.0e-12 {
                a = m;
            } else {
                b = m;
            }
        }
        f64::midpoint(a, b)
    }

    /// Plug-floor-lock-scale pin sanity: build at the larger §G-6
    /// envelope (4 mm half-extent base, 2.8 mm tip, 4 mm half-
    /// length, 0.8 mm chamfer) and confirm the same interior /
    /// exterior properties hold.
    #[test]
    fn plug_lock_default_pin_sdf_builds_and_centre_is_interior() {
        let spec = PrismaticPinSpec::plug_lock_default();
        let params = spec.pin_params(default_pose());
        let sdf = build_prismatic_pin_sdf(&params);
        assert!(sdf.evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0);
        assert!(sdf.evaluate(&Point3::new(0.1, 0.0, 0.0)) > 0.0);
    }

    /// Pose composition: rotating + translating the pin into world
    /// coords moves the centre to `pose.center_m` and aligns the
    /// long axis to `pose.axis_unit`. Tested by sampling the SDF
    /// at `pose.center_m` (interior) and at `pose.center_m +
    /// 2*half_length * axis_unit` (just past the tip face,
    /// exterior).
    #[test]
    fn pose_translates_centre_and_aligns_axis() {
        let spec = PrismaticPinSpec::cup_pin_default();
        let centre = Point3::new(0.032_5, 0.0, 0.005);
        let pose = PrismaticPinPose::new(
            centre,
            UnitVector3::new_normalize(Vector3::new(1.0, 2.0, 3.0)),
            UnitVector3::new_normalize(Vector3::new(2.0, -1.0, 0.0)),
        );
        let sdf = build_prismatic_pin_sdf(&spec.pin_params(pose.clone()));
        assert!(sdf.evaluate(&centre) < 0.0, "world centre must be interior");
        let past_tip = centre + pose.axis_unit.into_inner() * (2.0 * spec.pin_half_length_m);
        assert!(
            sdf.evaluate(&past_tip) > 0.0,
            "point past the tip face along axis_unit must be exterior; SDF={}",
            sdf.evaluate(&past_tip),
        );
    }

    /// Pose orthogonality assertion fires on non-perpendicular
    /// `lateral_unit`.
    #[test]
    #[should_panic(expected = "PrismaticPinPose: lateral_unit must be orthogonal to axis_unit")]
    fn pose_new_panics_on_non_orthogonal_lateral() {
        let _ = PrismaticPinPose::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::y_axis(),
            Vector3::y_axis(), // intentionally parallel to axis
        );
    }

    /// Validation fires on tip ≥ base (would degenerate the
    /// truncated pyramid into a prism or inverted shape).
    #[test]
    #[should_panic(expected = "must be strictly less than")]
    fn validate_params_panics_when_tip_meets_or_exceeds_base() {
        let mut spec = PrismaticPinSpec::cup_pin_default();
        spec.pin_tip_half_extents_m.x = spec.pin_base_half_extents_m.x;
        let _ = build_prismatic_pin_sdf(&spec.pin_params(default_pose()));
    }

    /// Validation fires on chamfer ≥ smaller base half-extent
    /// (would invert the bed-facing face).
    #[test]
    #[should_panic(expected = "must be strictly less than the smaller base half-extent")]
    fn validate_params_panics_when_chamfer_inverts_bed_face() {
        let mut spec = PrismaticPinSpec::cup_pin_default();
        spec.base_chamfer_m = spec
            .pin_base_half_extents_m
            .x
            .min(spec.pin_base_half_extents_m.y);
        let _ = build_prismatic_pin_sdf(&spec.pin_params(default_pose()));
    }

    /// Zero chamfer disables the chamfer band — the pin is a
    /// single frustum from base to tip. Verify by sampling the
    /// SDF at the bed face: lateral extent equals `base` (no
    /// inset).
    #[test]
    fn zero_chamfer_produces_single_frustum_with_base_width_bed_face() {
        let spec = PrismaticPinSpec {
            base_chamfer_m: 0.0,
            ..PrismaticPinSpec::cup_pin_default()
        };
        let params = spec.pin_params(default_pose());
        let sdf = build_prismatic_pin_sdf(&params);
        let bed_y = -spec.pin_half_length_m;
        let bed_extent = find_x_zero_crossing(&sdf, bed_y, 0.0, 0.01);
        assert_abs_diff_eq!(bed_extent, spec.pin_base_half_extents_m.x, epsilon = 1.0e-6,);
    }
}
