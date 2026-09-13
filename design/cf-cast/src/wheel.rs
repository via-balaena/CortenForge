//! Parametric wheel family — a PU tire cast directly onto a printed rim.
//!
//! The wheel is the first cf-cast subject that is **not** a layered silicone
//! device: a 95A polyurethane tire poured around a 3D-printed PLA rim, six of
//! them against 2 lb of PU. It earns a module here rather than a new crate
//! because both of its solids are [`crate::CastSpec`] *inputs* and its only
//! volume oracle (`pour_volume`) is crate-private.
//!
//! # Frame
//!
//! Built about **Z**, centered at the origin — [`Solid::cylinder`] and
//! [`Solid::sphere`] are Z-aligned by construction, so nothing rotates into
//! frame. The wheel axis is `+Z` and the parting plane is `z = 0`.
//!
//! ⚠ That is the *design* frame. The pour stands the wheel up (axis
//! horizontal) so the cavity has exactly one high point; mapping this frame to
//! the pour frame belongs to the mold, not here.
//!
//! # How it maps onto [`crate::CastSpec`]
//!
//! | `CastSpec` field | wheel |
//! |---|---|
//! | `plug` | [`rim_solid`] — the printed rim |
//! | `layers[0].body` | [`cast_body_solid`] |
//! | `layers[0].material` | 95A PU — [`crate::MoldingMaterial`] at [`NOMINAL_PU_95A_DENSITY_KG_M3`] |
//!
//! Those three are the wheel's. The rest of [`crate::CastSpec`] —
//! `bounding_region`, `wall_thickness_m`, `mass_budget_kg`, the ribbon and its
//! mating features — belongs to the cast rather than to the part, and this
//! module deliberately sets none of it.
//!
//! ⚠⚠ `layers[i].body` is the **cumulative solid**, never an annulus — an
//! annular body re-introduces the plug cavity as "inside the mold piece" (see
//! the crate docstring's cast-layer convention). So [`cast_body_solid`] is a
//! *disc* at the tire's outer radius, and the tire itself is what cf-cast
//! derives as `body ∖ plug`. [`tire_solid`] is that difference, built the same
//! way, so the two cannot drift.
//!
//! ★ **The plug is not retrieved.** The rim stays inside the cured tire — that
//! is what overmolding is, expressed in cf-cast's vocabulary. Two consequences
//! are load-bearing here:
//!
//! - **The rim may carry undercuts.** Draft exists so a plug can be pulled;
//!   this one never is. [`KeyingKind::Dimples`] uses that directly.
//! - **The demold prose is wrong for this subject** — `crate::procedure` tells
//!   the reader to pull the plug out. Fixing that needs a role on the artifact
//!   and is deliberately not in this module.
//!
//! # The bore becomes a locating pin, for free
//!
//! A mold piece's cup wall is a **body-tracking shell**: the points outside
//! the body but within `wall_thickness_m` of its surface, intersected with the
//! ribbon's half-space (`piece::CupWallShellSdf`, §Q-1, 2026-05-26 — it
//! replaced the earlier `bounding_region ∖ body` cuboid form). Because
//! [`cast_body_solid`] subtracts a column through the axle bore, that column
//! is outside the body, so the shell fills it and each half gets a pin the rim
//! slides onto. The pin is undersized by [`WheelSpec::bore_clearance_m`].
//!
//! ⚠⚠ **The shell form makes this conditional, and the old difference form did
//! not.** A shell only reaches `wall_thickness_m` inward from the bore wall, so
//! the pin is solid only while the bore is narrower than the wall is thick —
//! [`locating_pin_is_solid`]. Wider, and the mold grows a hollow tube around
//! the bore with a void down the middle. At the defaults it is 3.9 mm of pin
//! inside a 5 mm wall: 1.1 mm of margin.
//!
//! ⚠ The pin's printability is unverified: ask the slicer, not this module.
//!
//! # Parting plane
//!
//! A planar trim tangent to a curved surface always leaves a zero-thickness
//! edge; the cut must meet the surface where the surface normal is
//! **perpendicular** to the plane normal. Here the tire's outer surface at
//! `z = 0` is a cylinder whose normal is radial while the plane normal is
//! axial — perpendicular, so the seam is transversal and no feather edge can
//! form. `tire_outer_surface_is_transversal_at_the_parting_plane` asserts it
//! rather than leaving it as a symmetry argument.

use nalgebra::{UnitQuaternion, Vector3};

use cf_design::Solid;

/// Tire outer radius (65 mm → 130 mm OD).
const DEFAULT_TIRE_OUTER_RADIUS_M: f64 = 0.065;

/// Rim outer radius (52.5 mm → 105 mm OD), which is also the tire's inner
/// radius: the two surfaces are coincident because the tire is cast against
/// the rim. Leaves a 12.5 mm tread section.
const DEFAULT_RIM_OUTER_RADIUS_M: f64 = 0.0525;

/// Axle bore radius (4 mm → 8 mm bore).
///
/// ⚠ Bearings are undecided. A 608 bearing needs a 22 mm pocket, not an 8 mm
/// bore; this is a plain shaft hole until that question is answered.
const DEFAULT_BORE_RADIUS_M: f64 = 0.004;

/// Radial undersize of the mold's locating pin against the bore (0.1 mm),
/// matching the PLA-on-PLA slide fit [`crate::dowel_hole`] uses for its
/// printed dowels.
const DEFAULT_BORE_CLEARANCE_M: f64 = 0.0001;

/// Tread width (25 mm), shared by rim and tire so their side faces are flush.
///
/// Set by the PU on hand, not by the build plate: six tires at this width are
/// 695 cm³ of the 864 cm³ in 2 lb, leaving 169 cm³ — about 1.4 tires, so the
/// arc absorbs one failed pour. `six_tires_leave_enough_polyurethane_for_one_retry`
/// asserts it.
const DEFAULT_WIDTH_M: f64 = 0.025;

/// Keying dimples around the rim (8).
const DEFAULT_DIMPLE_COUNT: u32 = 8;

/// Keying dimple radius (3 mm), which is also its depth — the dimple is a
/// sphere centered ON the rim's outer surface, so it is a hemispherical
/// pocket in the rim and a hemispherical rivet on the tire.
const DEFAULT_DIMPLE_RADIUS_M: f64 = 0.003;

/// Slack applied to subtracted cylinders' half-height so their flat caps sit
/// outside the parent's faces, keeping the boolean off coincident planes.
/// Same role as [`crate::dowel_hole::HOLE_AXIAL_SLACK_M`].
const AXIAL_SLACK_M: f64 = 0.0005;

/// Nominal bulk density of 95A casting polyurethane (kg/m³).
///
/// ⚠ **Unverified.** 1.05 g/cm³ is the figure the wheel budget was planned
/// against; it has not been read off the product's TDS. Reading it is an open
/// item, and it scales every mass in this module linearly.
pub const NOMINAL_PU_95A_DENSITY_KG_M3: f64 = 1050.0;

/// Hemispherical keying dimples, equally spaced around the rim's outer face.
///
/// The tire's matching rivets resist rotation (the failure mode that matters
/// for a driven wheel — a purely axisymmetric groove resists none of it) and,
/// being bumps, also resist axial walk-off.
///
/// ⚠ The *count* and *radius* are unvalidated: nothing has been poured, so no
/// shear load has been measured. They are parameters for that reason.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DimpleSpec {
    /// How many dimples around the rim.
    pub count: u32,
    /// Sphere radius; also the pocket depth, since the sphere is centered on
    /// the rim's outer surface.
    pub radius_m: f64,
}

impl DimpleSpec {
    /// Iter-1 defaults: 8 dimples of 3 mm radius.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            count: DEFAULT_DIMPLE_COUNT,
            radius_m: DEFAULT_DIMPLE_RADIUS_M,
        }
    }
}

impl Default for DimpleSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// How the tire is mechanically keyed to the rim.
///
/// Follows the crate's `FlangeKind` / `GasketKind` / `DowelHoleKind` shape,
/// but unlike those the default is **not** `None`: an unkeyed tire spins on
/// its rim, so it is not a wheel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyingKind {
    /// No keying. The tire is a plain annulus bonded only by adhesion — for
    /// volume studies and adhesion coupons, not for a wheel.
    None,
    /// Hemispherical dimples in the rim, filled by the pour.
    Dimples(DimpleSpec),
}

impl KeyingKind {
    /// The inner [`DimpleSpec`] for [`KeyingKind::Dimples`], `None` otherwise.
    #[must_use]
    pub const fn dimple_spec(self) -> Option<DimpleSpec> {
        match self {
            Self::None => None,
            Self::Dimples(spec) => Some(spec),
        }
    }
}

/// A wheel: a PU tire overmolded on a printed rim, both about `+Z`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpec {
    /// Tire outer radius (metres). Default 65 mm.
    pub tire_outer_radius_m: f64,
    /// Rim outer radius = tire inner radius (metres). Default 52.5 mm.
    pub rim_outer_radius_m: f64,
    /// Axle bore radius (metres). Default 4 mm.
    pub bore_radius_m: f64,
    /// Radial undersize of the mold's locating pin against the bore
    /// (metres). Default 0.1 mm.
    pub bore_clearance_m: f64,
    /// Axial width of rim and tire (metres). Default 25 mm.
    pub width_m: f64,
    /// Mechanical keying between tire and rim.
    pub keying: KeyingKind,
}

impl WheelSpec {
    /// Iter-1 workshop starting point: 130 mm OD, 105 mm rim, 25 mm wide,
    /// 8 mm bore, dimple-keyed.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            tire_outer_radius_m: DEFAULT_TIRE_OUTER_RADIUS_M,
            rim_outer_radius_m: DEFAULT_RIM_OUTER_RADIUS_M,
            bore_radius_m: DEFAULT_BORE_RADIUS_M,
            bore_clearance_m: DEFAULT_BORE_CLEARANCE_M,
            width_m: DEFAULT_WIDTH_M,
            keying: KeyingKind::Dimples(DimpleSpec::iter1()),
        }
    }

    /// Radial depth of the tread section — `tire_outer − rim_outer`.
    #[must_use]
    pub fn tread_depth_m(&self) -> f64 {
        self.tire_outer_radius_m - self.rim_outer_radius_m
    }

    /// Radius of the locating pin the mold grows through the axle bore —
    /// `bore_radius − bore_clearance`.
    ///
    /// ★ **The one derivation.** [`cast_body_solid`] subtracts a column of
    /// this radius (which is what *creates* the pin), and any consumer sizing
    /// or describing the pin reads it here. Splitting a clearance across two
    /// sites is a shipped-bug shape this crate has already paid for once.
    #[must_use]
    pub fn locating_pin_radius_m(&self) -> f64 {
        self.bore_radius_m - self.bore_clearance_m
    }

    /// Panics unless the dimensions are positive, finite and ordered.
    /// Called by every public builder in this module. Not a completeness
    /// claim: it checks the orderings [`Solid`]'s own constructors cannot
    /// see, not every degenerate wheel.
    fn assert_well_formed(&self) {
        assert!(
            self.width_m > 0.0 && self.width_m.is_finite(),
            "wheel width must be positive and finite, got {}",
            self.width_m
        );
        assert!(
            self.tire_outer_radius_m > self.rim_outer_radius_m,
            "tire outer radius ({}) must exceed rim outer radius ({})",
            self.tire_outer_radius_m,
            self.rim_outer_radius_m
        );
        assert!(
            self.rim_outer_radius_m > self.bore_radius_m,
            "rim outer radius ({}) must exceed bore radius ({})",
            self.rim_outer_radius_m,
            self.bore_radius_m
        );
        assert!(
            self.locating_pin_radius_m() > 0.0,
            "bore clearance ({}) must leave a positive locating pin inside a \
             bore of radius {}",
            self.bore_clearance_m,
            self.bore_radius_m
        );
        if let Some(dimple) = self.keying.dimple_spec() {
            assert!(
                dimple.count > 0,
                "dimple keying needs at least one dimple, got {}",
                dimple.count
            );
            assert!(
                dimple.radius_m > 0.0 && dimple.radius_m < self.tread_depth_m(),
                "dimple radius ({}) must be positive and inside the tread \
                 depth ({})",
                dimple.radius_m,
                self.tread_depth_m()
            );
            assert!(
                dimple.radius_m < self.width_m / 2.0,
                "dimple radius ({}) must fit inside the half-width ({})",
                dimple.radius_m,
                self.width_m / 2.0
            );
        }
    }
}

impl Default for WheelSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// The keying spheres, centered on the rim's outer surface in the plane
/// `z = 0`, or `None` for [`KeyingKind::None`].
///
/// ★ **The one derivation.** [`rim_solid`] is the only caller: it subtracts
/// these, and [`tire_solid`] gains the matching rivets purely as the
/// complement `body ∖ rim`. A pocket and its rivet are therefore the same
/// geometry by construction and cannot desync —
/// `dimples_are_pockets_in_the_rim_and_rivets_on_the_tire` fails if a future
/// edit re-derives either side independently.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn keying_solid(spec: &WheelSpec) -> Option<Solid> {
    spec.assert_well_formed();
    let dimple = spec.keying.dimple_spec()?;

    let seed =
        Solid::sphere(dimple.radius_m).translate(Vector3::new(spec.rim_outer_radius_m, 0.0, 0.0));

    // Polar repeat by folding — `repeat_bounded` is a Cartesian grid and
    // there is no polar domain-repeat in the SDF kernel. `count` is small.
    let mut keys = seed.clone();
    for i in 1..dimple.count {
        let theta = std::f64::consts::TAU * f64::from(i) / f64::from(dimple.count);
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta);
        keys = keys.union(seed.clone().rotate(rotation));
    }
    Some(keys)
}

/// The printed rim — [`crate::CastSpec::plug`].
///
/// A disc at the rim's outer radius, bored for the axle and dimpled for
/// keying.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn rim_solid(spec: &WheelSpec) -> Solid {
    spec.assert_well_formed();
    let half_width = spec.width_m / 2.0;
    let mut disc = Solid::cylinder(spec.rim_outer_radius_m, half_width).subtract(Solid::cylinder(
        spec.bore_radius_m,
        half_width + AXIAL_SLACK_M,
    ));
    if let Some(keys) = keying_solid(spec) {
        disc = disc.subtract(keys);
    }
    disc
}

/// The cumulative cast body — [`crate::CastSpec`]'s `layers[0].body`.
///
/// A **disc** at the tire's outer radius, not an annulus (see the module
/// docstring), less the locating-pin column so the mold grows a pin through
/// the bore.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn cast_body_solid(spec: &WheelSpec) -> Solid {
    spec.assert_well_formed();
    let half_width = spec.width_m / 2.0;
    Solid::cylinder(spec.tire_outer_radius_m, half_width).subtract(Solid::cylinder(
        spec.locating_pin_radius_m(),
        half_width + AXIAL_SLACK_M,
    ))
}

/// Whether the mold's locating pin is solid to the axis, given the cup wall's
/// thickness ([`crate::CastSpec::wall_thickness_m`]).
///
/// The cup wall is a shell that reaches `wall_thickness_m` outward from the
/// body's surface, so it fills the bore column only while the column's radius
/// stays under that. Wider, and what forms is a tube around the bore wall with
/// a void down the middle — still a locator, but a thin-walled one nobody
/// designed. 5 mm is the workshop default wall; the iter-1 pin is 3.9 mm.
///
/// ⚠ Not part of [`WheelSpec`]'s own validation: the wall thickness belongs to
/// the cast, not to the wheel, so a `WheelSpec` cannot check this alone.
#[must_use]
pub fn locating_pin_is_solid(spec: &WheelSpec, wall_thickness_m: f64) -> bool {
    spec.locating_pin_radius_m() < wall_thickness_m
}

/// The cured tire — the PU actually poured.
///
/// Built as `cast_body ∖ rim`, the same difference
/// [`crate::CastSpec::compute_pour_volumes`] takes for layer 0, so what this
/// renders and what cf-cast integrates are the same solid by construction.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn tire_solid(spec: &WheelSpec) -> Solid {
    cast_body_solid(spec).subtract(rim_solid(spec))
}

/// Nominal tire volume in cubic metres — the PU-planning number.
///
/// `π (R_tire² − R_rim²) w` for the annulus, plus one hemisphere per dimple,
/// plus the annular gap between the locating pin and the bore wall.
///
/// ⚠ **This is analytic, not measured, and the dimple term is approximate.** A
/// dimple is bounded by the *cylinder* `r = R_rim`, not by a plane, so its
/// tire-side portion is slightly more than a hemisphere. The excess is
/// second-order in `r / R_rim` and has not been isolated — it is below the
/// integrator's own bias, so no available measurement separates it.
/// `nominal_tire_volume_agrees_with_sdf_integration` checks the built solid
/// by a different route, at that integrator's accuracy.
///
/// ⚠ The pin-gap term is PU that wicks into a slip fit. Whether it actually
/// does is unmeasured; counting it is the conservative direction for a budget.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn nominal_tire_volume_m3(spec: &WheelSpec) -> f64 {
    spec.assert_well_formed();
    let pi = std::f64::consts::PI;

    let annulus = pi
        * (spec.tire_outer_radius_m * spec.tire_outer_radius_m
            - spec.rim_outer_radius_m * spec.rim_outer_radius_m)
        * spec.width_m;

    let dimples = spec.keying.dimple_spec().map_or(0.0, |dimple| {
        let r = dimple.radius_m;
        f64::from(dimple.count) * (2.0 / 3.0) * pi * r * r * r
    });

    let pin_radius = spec.locating_pin_radius_m();
    let pin_gap =
        pi * (spec.bore_radius_m * spec.bore_radius_m - pin_radius * pin_radius) * spec.width_m;

    annulus + dimples + pin_gap
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use approx::assert_relative_eq;
    use nalgebra::Point3;

    use super::{
        DimpleSpec, KeyingKind, NOMINAL_PU_95A_DENSITY_KG_M3, WheelSpec, cast_body_solid,
        locating_pin_is_solid, nominal_tire_volume_m3, rim_solid, tire_solid,
    };
    use crate::error::CastTarget;
    use crate::pour_volume::{
        DEFAULT_MASS_BUDGET_KG, POUR_VOLUME_MIN_CELL_SIZE_M, integrate_negative_sdf_volume,
    };

    /// The production integration cell, so the volume gate measures the path
    /// `CastSpec::compute_pour_volumes` actually takes.
    const PRODUCTION_CELL_M: f64 = POUR_VOLUME_MIN_CELL_SIZE_M;

    fn integrate(spec: &WheelSpec, cell_m: f64) -> f64 {
        integrate_negative_sdf_volume(
            &tire_solid(spec),
            cell_m,
            CastTarget::LayerBody { layer_index: 0 },
        )
        .unwrap()
    }

    #[test]
    fn iter1_is_the_planned_wheel() {
        // Literal pin on the dimensions the PU budget was planned against —
        // a silent constant edit changes how many tires fit in 2 lb.
        let spec = WheelSpec::iter1();
        assert_relative_eq!(spec.tire_outer_radius_m, 0.065);
        assert_relative_eq!(spec.rim_outer_radius_m, 0.0525);
        assert_relative_eq!(spec.width_m, 0.025);
        assert_relative_eq!(spec.bore_radius_m, 0.004);
        assert_relative_eq!(spec.tread_depth_m(), 0.0125);
        assert_relative_eq!(spec.locating_pin_radius_m(), 0.0039);
        assert_eq!(spec.keying, KeyingKind::Dimples(DimpleSpec::iter1()));
    }

    #[test]
    fn nominal_tire_volume_is_the_planned_number() {
        // 115 869.791 mm³ = π(65² − 52.5²)·25 annulus + 8 hemispheres of
        // r=3 + the 0.1 mm pin gap. Literal, so the closed form in
        // `nominal_tire_volume_m3` is not its own oracle.
        let v_mm3 = nominal_tire_volume_m3(&WheelSpec::iter1()) * 1e9;
        assert_relative_eq!(v_mm3, 115_869.791, epsilon = 0.001);
    }

    #[test]
    fn six_tires_leave_enough_polyurethane_for_one_retry() {
        // The plan's "~20 % spare = one retry" as an assertion. Holdings are
        // 2 lb — numerically `DEFAULT_MASS_BUDGET_KG`, but that constant is
        // cf-cast's PER-POUR gate; this is an AGGREGATE claim over six pours,
        // and cf-cast has no aggregate gate. Referenced rather than retyped so
        // the pound conversion lives in one place.
        let holdings_kg = DEFAULT_MASS_BUDGET_KG;
        let one_tire_kg =
            nominal_tire_volume_m3(&WheelSpec::iter1()) * NOMINAL_PU_95A_DENSITY_KG_M3;
        assert_relative_eq!(one_tire_kg, 0.121_663, epsilon = 1e-6);

        let six = 6.0 * one_tire_kg;
        assert!(
            six < holdings_kg,
            "six tires {six} kg exceed {holdings_kg} kg"
        );
        assert!(
            holdings_kg - six >= one_tire_kg,
            "spare {} kg does not cover a retry tire of {one_tire_kg} kg",
            holdings_kg - six
        );
    }

    #[test]
    fn narrowing_the_tread_one_millimetre_frees_four_point_six_cubic_centimetres() {
        // Negative control for the volume gate: the width constant must move
        // the answer, by this much.
        let spec = WheelSpec::iter1();
        let mut narrow = spec;
        narrow.width_m -= 0.001;
        let delta_mm3 = (nominal_tire_volume_m3(&spec) - nominal_tire_volume_m3(&narrow)) * 1e9;
        assert_relative_eq!(delta_mm3, 4_616.696, epsilon = 0.001);
    }

    #[test]
    fn nominal_tire_volume_agrees_with_sdf_integration() {
        // Second route to the same number: a voxel count of the built solid
        // versus the closed form. Catches a wrong radius or a missing
        // subtraction, which move the answer by tens of percent.
        //
        // ⚠ The tolerance is wide because the INTEGRATOR is. Measured relative
        // error against the closed form: −5.5 % at 2 mm, −1.4 % at 1 mm,
        // −2.2 % at 0.5 mm. It is a strict `< 0` corner count, so a corner
        // lying exactly on a face is excluded — and whether corners land on
        // the faces at all depends on float rounding in
        // `ScalarGrid::from_bounds`. Hence: always an undercount, and neither
        // monotone nor smooth in the cell size. This module does not fix that;
        // it records it, because the same bias applies to the mass budget the
        // workshop pours against.
        let spec = WheelSpec::iter1();
        let nominal = nominal_tire_volume_m3(&spec);
        for (cell_m, tolerance) in [(PRODUCTION_CELL_M, 0.07), (0.001, 0.03)] {
            let measured = integrate(&spec, cell_m);
            let rel = (measured - nominal) / nominal;
            assert!(
                rel.abs() < tolerance,
                "cell {cell_m} m: integrated {measured} m³ vs nominal {nominal} m³ ({rel:+.4})"
            );
            assert!(
                rel < 0.0,
                "cell {cell_m} m: expected an undercount, got {rel:+.4}"
            );
        }
    }

    #[test]
    fn tire_outer_surface_is_transversal_at_the_parting_plane() {
        // THE feather-edge gate. A planar trim tangent to a curved surface
        // leaves a zero-thickness edge; it is transversal only where the
        // surface normal is perpendicular to the plane normal. The parting
        // plane is z = 0 with normal +Z, so the cavity's outer surface there
        // must have no Z component — which is what makes the equator the
        // right parting line, rather than symmetry.
        let spec = WheelSpec::iter1();
        let body = cast_body_solid(&spec);
        for step in 0..16 {
            let theta = std::f64::consts::TAU * f64::from(step) / 16.0;
            let p = Point3::new(
                spec.tire_outer_radius_m * theta.cos(),
                spec.tire_outer_radius_m * theta.sin(),
                0.0,
            );
            let n = body.gradient(&p);
            assert!(
                n.z.abs() < 1e-9,
                "cavity normal at θ={theta} has axial component {}",
                n.z
            );
        }
    }

    #[test]
    fn rim_and_tire_partition_the_cast_body() {
        // `layers[0].body ∖ plug` is what cf-cast pours, so body must be
        // exactly rim ⊎ tire with no overlap and no gap. Catches a rim wider
        // or taller than the body, and any dimple that is carved from one
        // side but not filled on the other.
        let spec = WheelSpec::iter1();
        let (body, rim, tire) = (cast_body_solid(&spec), rim_solid(&spec), tire_solid(&spec));
        // ⚠ The lattice must extend well OUTSIDE the part in every direction.
        // A first version stopped at |z| = 12 mm inside a 12.5 mm half-width
        // and could not see a rim 2 % taller than the body — the gate passed
        // on geometry it exists to reject.
        let mut sampled = 0_u32;
        for ix in -16_i32..=16 {
            for iy in -16_i32..=16 {
                for iz in -10_i32..=10 {
                    let p = Point3::new(
                        f64::from(ix) * 0.005,
                        f64::from(iy) * 0.005,
                        f64::from(iz) * 0.002,
                    );
                    // Skip points within a hair of any surface — the
                    // partition is a claim about interiors, not boundaries.
                    if body.evaluate(&p).abs() < 1e-4
                        || rim.evaluate(&p).abs() < 1e-4
                        || tire.evaluate(&p).abs() < 1e-4
                    {
                        continue;
                    }
                    let (in_body, in_rim, in_tire) = (
                        body.evaluate(&p) < 0.0,
                        rim.evaluate(&p) < 0.0,
                        tire.evaluate(&p) < 0.0,
                    );
                    assert!(!(in_rim && in_tire), "rim and tire overlap at {p:?}");
                    assert_eq!(in_body, in_rim || in_tire, "body != rim ⊎ tire at {p:?}");
                    sampled += 1;
                }
            }
        }
        assert!(
            sampled > 20_000,
            "partition gate only sampled {sampled} points"
        );
    }

    #[test]
    fn the_rim_fits_entirely_inside_the_cast_body() {
        // Exact companion to the partition gate, which samples on a 5 × 5 × 2
        // mm lattice and therefore cannot resolve a discrepancy smaller than
        // its step: a rim 2 % too tall (0.25 mm proud) passed it. A plug
        // protruding through the body sits in the cup wall's shell — outside
        // the body and within a wall thickness of it — so the mold would be
        // built intersecting the plug. A cast-validity constraint, not
        // bookkeeping.
        let spec = WheelSpec::iter1();
        let body = cast_body_solid(&spec).bounds().unwrap();
        let rim = rim_solid(&spec).bounds().unwrap();
        for axis in 0..3 {
            assert!(
                rim.min[axis] >= body.min[axis] && rim.max[axis] <= body.max[axis],
                "rim [{}, {}] escapes the cast body [{}, {}] on axis {axis}",
                rim.min[axis],
                rim.max[axis],
                body.min[axis],
                body.max[axis]
            );
        }
    }

    #[test]
    fn dimples_are_pockets_in_the_rim_and_rivets_on_the_tire() {
        // Sampled at the dimple centres, which sit ON the nominal rim
        // surface: each must be tire, not rim. Between them, the same radius
        // must be rim, not tire. A count or radius that desynced between the
        // two builders fails here.
        let spec = WheelSpec::iter1();
        let dimple = spec.keying.dimple_spec().unwrap();
        let (rim, tire) = (rim_solid(&spec), tire_solid(&spec));
        let r = spec.rim_outer_radius_m - dimple.radius_m / 2.0;
        for step in 0..dimple.count {
            let on = std::f64::consts::TAU * f64::from(step) / f64::from(dimple.count);
            let between = on + std::f64::consts::PI / f64::from(dimple.count);
            let at = |theta: f64| Point3::new(r * theta.cos(), r * theta.sin(), 0.0);
            assert!(tire.evaluate(&at(on)) < 0.0, "no rivet at dimple {step}");
            assert!(rim.evaluate(&at(on)) > 0.0, "no pocket at dimple {step}");
            assert!(
                rim.evaluate(&at(between)) < 0.0,
                "rim missing between {step}"
            );
            assert!(
                tire.evaluate(&at(between)) > 0.0,
                "tire intrudes between {step}"
            );
        }
    }

    #[test]
    fn unkeyed_wheel_is_a_plain_annulus_plus_the_pin_gap() {
        let mut spec = WheelSpec::iter1();
        spec.keying = KeyingKind::None;
        // Annulus 115 355.371 mm³ + pin gap 62.046 mm³, no hemispheres.
        let v_mm3 = nominal_tire_volume_m3(&spec) * 1e9;
        assert_relative_eq!(v_mm3, 115_417.402, epsilon = 0.001);

        let rim = rim_solid(&spec);
        let tire = tire_solid(&spec);
        for step in 0..8 {
            let theta = std::f64::consts::TAU * f64::from(step) / 8.0;
            let r = spec.rim_outer_radius_m - 0.0015;
            let p = Point3::new(r * theta.cos(), r * theta.sin(), 0.0);
            assert!(rim.evaluate(&p) < 0.0, "unkeyed rim pocketed at θ={theta}");
            assert!(tire.evaluate(&p) > 0.0, "unkeyed tire riveted at θ={theta}");
        }
    }

    #[test]
    fn the_bore_column_belongs_to_the_mold_not_the_body() {
        // The locating pin exists because `cast_body_solid` subtracts an
        // undersized column, putting it outside the body where the cup wall's
        // shell fills it. This asserts the body side of that — the column is
        // outside all three solids, the clearance ring is tire, and the rim's
        // bore is wider than the pin.
        // `the_mold_grows_a_locating_pin_through_the_bore` asserts the mold
        // side.
        let spec = WheelSpec::iter1();
        let (body, rim, tire) = (cast_body_solid(&spec), rim_solid(&spec), tire_solid(&spec));
        let axis = Point3::origin();
        assert!(body.evaluate(&axis) > 0.0, "pin column is inside the body");
        assert!(rim.evaluate(&axis) > 0.0, "pin column is inside the rim");
        assert!(tire.evaluate(&axis) > 0.0, "pin column is inside the tire");

        let in_gap = Point3::new(spec.bore_radius_m - spec.bore_clearance_m / 2.0, 0.0, 0.0);
        assert!(
            tire.evaluate(&in_gap) < 0.0,
            "clearance ring is not pour volume"
        );
        assert!(spec.locating_pin_radius_m() < spec.bore_radius_m);
    }

    /// The workshop cup-wall thickness — `cf-cast-cli`'s `default_wall_thickness_m`.
    const WORKSHOP_WALL_M: f64 = 0.005;

    /// The cup wall as `piece::CupWallShellSdf` defines it: outside the body,
    /// within `wall_m` of its surface. Built from `Solid` ops rather than by
    /// copying that struct's arithmetic, so the two agree by definition and
    /// not by transcription.
    fn cup_wall(spec: &WheelSpec, wall_m: f64) -> cf_design::Solid {
        cast_body_solid(spec)
            .offset(wall_m)
            .subtract(cast_body_solid(spec))
    }

    #[test]
    fn the_mold_grows_a_locating_pin_through_the_bore() {
        // The module docstring claims the pin is free: the bore column is
        // outside the body, so the body-tracking cup wall fills it. That was
        // prose; this runs the set algebra and looks.
        //
        // ⚠ Scope, twice over. (a) This is the SHELL, not
        // `compose_piece_solid`'s full pipeline — that it survives the
        // half-space intersect, the flange union and marching cubes is M2's
        // to show. (b) A FIRST version of this test used
        // `bounding.subtract(body)`, the composition §Q-1 retired in May. It
        // passed, on algebra the crate no longer runs.
        let spec = WheelSpec::iter1();
        assert!(locating_pin_is_solid(&spec, WORKSHOP_WALL_M));
        let mold = cup_wall(&spec, WORKSHOP_WALL_M);

        // Along the axis, over the full width, the mold is present …
        for step in -5_i32..=5 {
            let z = f64::from(step) * 0.002;
            let p = Point3::new(0.0, 0.0, z);
            assert!(mold.evaluate(&p) < 0.0, "no locating pin at z={z}");
        }
        // … out to the pin's radius, and no further: the clearance ring
        // between pin and bore wall is pour volume, not mold.
        let pin_r = spec.locating_pin_radius_m();
        assert!(mold.evaluate(&Point3::new(pin_r * 0.9, 0.0, 0.0)) < 0.0);
        assert!(mold.evaluate(&Point3::new(spec.bore_radius_m, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn a_bore_wider_than_the_wall_leaves_a_tube_not_a_pin() {
        // The condition the shell form introduced and the retired difference
        // form did not: a shell reaches only `wall_thickness_m` inward, so a
        // wide bore grows a tube with a void down its middle.
        let mut spec = WheelSpec::iter1();
        spec.bore_radius_m = 0.010;
        assert!(!locating_pin_is_solid(&spec, WORKSHOP_WALL_M));

        let mold = cup_wall(&spec, WORKSHOP_WALL_M);
        assert!(
            mold.evaluate(&Point3::origin()) > 0.0,
            "expected a void on the axis, got mold material"
        );
        // The tube itself is there, just under the bore wall.
        let in_tube = spec.locating_pin_radius_m() - WORKSHOP_WALL_M / 2.0;
        assert!(mold.evaluate(&Point3::new(in_tube, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn default_delegates_to_iter1() {
        // Both `Default` impls exist only to forward. Coverage said nothing
        // called them, so nothing would have noticed one drifting away from
        // the workshop starting point.
        assert_eq!(WheelSpec::default(), WheelSpec::iter1());
        assert_eq!(DimpleSpec::default(), DimpleSpec::iter1());
    }

    #[test]
    #[should_panic(expected = "must fit inside the half-width")]
    fn a_dimple_wider_than_the_tread_face_is_rejected() {
        // A dimple deeper than half the width breaks out through the tire's
        // side faces. Needs a narrow wheel to reach: at the defaults the
        // tread depth and the half-width are both 12.5 mm, so the tread-depth
        // assert fires first.
        let mut spec = WheelSpec::iter1();
        spec.width_m = 0.004;
        drop(rim_solid(&spec));
    }

    #[test]
    #[should_panic(expected = "must exceed rim outer radius")]
    fn a_tire_inside_its_rim_is_rejected() {
        let mut spec = WheelSpec::iter1();
        spec.tire_outer_radius_m = spec.rim_outer_radius_m;
        drop(rim_solid(&spec));
    }

    #[test]
    #[should_panic(expected = "must be positive and inside the tread depth")]
    fn a_dimple_deeper_than_the_tread_is_rejected() {
        let mut spec = WheelSpec::iter1();
        spec.keying = KeyingKind::Dimples(DimpleSpec {
            count: 8,
            radius_m: spec.tread_depth_m(),
        });
        drop(rim_solid(&spec));
    }

    #[test]
    #[should_panic(expected = "must leave a positive locating pin")]
    fn a_clearance_that_eats_the_locating_pin_is_rejected() {
        let mut spec = WheelSpec::iter1();
        spec.bore_clearance_m = spec.bore_radius_m;
        drop(cast_body_solid(&spec));
    }
}
