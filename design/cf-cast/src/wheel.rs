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
//! Those three are the wheel's, and so is the seam — [`wheel_ribbon`] builds
//! it, because where a wheel parts is a property of the wheel.
//! [`wheel_mold_ribbon`] goes one step further and applies the physically
//! proven mold configuration to that seam. What this module does NOT set is
//! the `CastSpec` assembly itself: `bounding_region`, `wall_thickness_m`,
//! `mass_budget_kg` and the export are the cast's.
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

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::bolt_pattern::{BoltPatternKind, BoltPatternSpec};
use crate::dowel_hole::{DowelHoleKind, DowelHoleSpec};
use crate::flange::{DemandFlangeSpec, FlangeKind};
use crate::gasket_mold::GasketKind;
use crate::material::MoldingMaterial;
use crate::plug_role::PlugRole;
use crate::pour::{PourGateKind, PourGateLayout, PourGateSpec};
use crate::pour_volume::DEFAULT_MASS_BUDGET_KG;
use crate::ribbon::{Ribbon, RibbonError, SplitNormal};
use crate::spec::{CastLayer, CastSpec};
use mesh_printability::PrinterConfig;

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
/// 695 cm³ of the 864 cm³ in 2 lb, leaving 169 cm³ — enough that the arc
/// absorbs one failed pour, which
/// `six_tires_leave_enough_polyurethane_for_one_retry` asserts.
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
/// Dimples resist rotation and axial walk-off; an axisymmetric groove resists
/// only walk-off.
///
/// ⚠ Which load actually governs is unmeasured, as are the *count* and
/// *radius* — nothing has been poured, so no shear load exists to size them
/// against. They are parameters for that reason.
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
/// Follows the crate's `FlangeKind` / `GasketKind` / `DowelHoleKind` shape.
/// ⚠ Unlike those it has no `Default` of its own, and [`WheelSpec::iter1`]
/// deliberately does not start it at `None`: an unkeyed tire spins on its rim,
/// so it is not a wheel.
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

    /// Radius at which the pour gate anchors — mid-tread, halfway between the
    /// rim and the tire's outer surface.
    ///
    /// ★ **The one derivation, and it is load-bearing twice over.**
    /// [`wheel_ribbon`]'s stub runs through `±` this radius, which is what
    /// makes `apex_axial_pose` put the gate at 12 o'clock in the seam plane.
    /// And it must land **strictly inside** the cast body: the integral funnel
    /// ray-marches the body from the apex and silently degrades to a bore with
    /// no funnel if the apex is not interior. On the tire's outer surface the
    /// body SDF reads exactly `0.0` and the funnel vanishes;
    /// `the_pour_gate_carries_its_funnel` is the gate for that.
    #[must_use]
    pub const fn gate_anchor_radius_m(&self) -> f64 {
        f64::midpoint(self.tire_outer_radius_m, self.rim_outer_radius_m)
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
/// A disc at the rim's outer radius, bored for the axle, and dimpled when
/// [`WheelSpec::keying`] asks for it.
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

/// The wheel's seam, as a [`Ribbon`] cf-cast's mold pipeline accepts.
///
/// The wheel has no centerline and cannot have one — `Ribbon.points` is an
/// OPEN polyline and no producer emits a closed loop — but a planar seam
/// bypasses the curve machinery entirely: [`Ribbon::sdf`] short-circuits to the
/// signed distance to one flat plane. The parting plane is set explicitly to
/// `z = 0` with normal `+Z`.
///
/// ⚠⚠ **The stub is NOT inert.** It was, until the pour gate started reading
/// it: `apex_axial_pose` anchors the gate at `centerline[0]` with its axis
/// along `-tangent`. So the stub runs **12 o'clock → 6 o'clock** through
/// [`WheelSpec::gate_anchor_radius_m`], which puts the gate at the top of the
/// wheel with its axis in the seam plane. Pointed along the wheel AXIS — the
/// obvious choice, and what this did through M2b — `outward` is parallel to
/// the seam normal, `apex_axial_pose` takes its documented pathological
/// branch, and the bore lands on the axis pointing out of the seam plane,
/// straight through the locating pin.
///
/// ⚠ `split_normal` is a FRAME HINT, not the parting-plane normal — the cut
/// normal is `tangent × split_normal`, so setting it to the wheel axis would
/// be wrong. The planar seam is what decides the cut here, measured by
/// `wheel_ribbon_seam_is_exactly_the_parting_plane`;
/// [`SplitNormal::default`] (world `+X`, perpendicular to the axis) keeps the
/// ribbon well-formed either way and is its one infallible constructor.
///
/// The returned ribbon carries no flange, fasteners or pour gate; those are
/// the cast's to add.
///
/// # Errors
///
/// Returns [`RibbonError`] if the two-point stub is rejected. A well-formed
/// [`WheelSpec`] has a positive width, so the points are distinct and this
/// cannot happen — it is propagated rather than asserted because library code
/// in this crate does not panic on a fallible call.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
pub fn wheel_ribbon(spec: &WheelSpec) -> Result<Ribbon, RibbonError> {
    spec.assert_well_formed();
    // Top first: `v_apex_anchor` takes `centerline[0]` with outward
    // `-tangent`, so 12 → 6 o'clock points the gate UP. Reversed, it aims at
    // 6 o'clock and the cavity fills against gravity.
    let anchor = spec.gate_anchor_radius_m();
    let stub = vec![
        Point3::new(0.0, anchor, 0.0),
        Point3::new(0.0, -anchor, 0.0),
    ];
    Ok(Ribbon::new(stub, SplitNormal::default())?
        .with_planar_seam_at(Point3::origin(), Vector3::z()))
}

/// The wheel's seam carrying the mating features of the **physically proven
/// mold configuration** — Demand flange, bolt pattern, symmetric dowel holes,
/// and no gasket.
///
/// That is the `base_mold` config that silicone was actually poured and cured
/// against, and the bolt clamp IS the seal there, which is why
/// [`crate::GasketKind::None`] is correct and not a shortcut.
///
/// ⛔ Gasket-none is also the only safe choice: `crate::gasket_mold` builds its
/// channel around a stored `seam_plane_y` and projects queries onto it
/// (`gasket_mold.rs:406`), with no seam-normal check anywhere, and the wheel's
/// seam is Z-normal. What such a mismatch would actually produce is untested —
/// nothing here builds one.
///
/// ✅ The apex-axial gate completes the proven config: a single bore lying in
/// the seam plane so separating the halves bisects it into open half-troughs,
/// with the integral split funnel fused into each cup. `include_vent` is
/// ignored for this layout — the workshop hand-drills vents at high spots.
///
/// ✅ [`PlugRole::Insert`]: the rim is the product, not tooling. It carves
/// nothing — it is what stops the sheet telling the bencher to release the rim
/// and pull it out of the tire, which on a wheel destroys the part.
///
/// A starting point, not a constraint: every field is a `Ribbon` builder call
/// the caller can override.
///
/// ⚠ What the placement solver does with a CIRCLE is not obvious — dowels are
/// seeded at the loop's principal-axis extremes, and a circle has no principal
/// axis, so the one it picks is decided by floating-point noise.
/// `dowels_land_far_apart_on_an_isotropic_loop` asserts what survives that:
/// two dowels, one radius, a large moment arm. Their absolute bearing does
/// not survive it, and is not the same on every platform.
///
/// # Errors
///
/// Propagates [`RibbonError`] from [`wheel_ribbon`].
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
pub fn wheel_mold_ribbon(spec: &WheelSpec) -> Result<Ribbon, RibbonError> {
    Ok(wheel_ribbon(spec)?
        .with_plug_role(PlugRole::Insert)
        .with_flange(FlangeKind::Demand(DemandFlangeSpec::iter1()))
        .with_dowel_hole(DowelHoleKind::Auto(DowelHoleSpec::iter1()))
        .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()))
        .with_gasket(GasketKind::None)
        .with_pour_gate(PourGateKind::Default(PourGateSpec {
            layout: PourGateLayout::ApexAxial,
            ..PourGateSpec::iter1()
        })))
}

/// Assemble the wheel's [`crate::CastSpec`], ready for
/// [`crate::CastSpec::export_molds_v2`] with [`wheel_mold_ribbon`].
///
/// The wheel supplies what it knows — the cast body, the rim as plug, the 95A
/// PU layer material, the bounding region, and the PU holdings as the mass
/// budget. The caller supplies what belongs to the cast rather than the part:
/// the cup-wall thickness, the marching-cubes cell, and the printer.
///
/// ⚠ `wall_thickness_m` is not free: the mold's locating pin is solid only
/// while [`locating_pin_is_solid`] holds against it.
///
/// ⚠⚠ **`mesh_cell_size_m` does NOT move the reported pour mass.**
/// `compute_pour_volumes` floors the integration cell at
/// [`crate::POUR_VOLUME_MIN_CELL_SIZE_M`] (2 mm) with `max()`, which only ever
/// raises it — so 1.5 mm and 1 mm mesh cells both integrate at 2 mm and report
/// the identical figure. Measured on this wheel: 115.11 g at 2 mm, 1.5 mm and
/// 1 mm alike; only a 3 mm cell differs, because 3 mm is coarser than the
/// floor. The cell is a SURFACE-QUALITY and EXPORT-TIME choice, nothing more.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see [`WheelSpec`].
#[must_use]
pub fn wheel_cast_spec(
    spec: &WheelSpec,
    wall_thickness_m: f64,
    mesh_cell_size_m: f64,
    printer_config: PrinterConfig,
) -> CastSpec {
    CastSpec {
        layers: vec![CastLayer {
            body: cast_body_solid(spec),
            material: MoldingMaterial {
                display_name: "95A polyurethane".to_string(),
                density_kg_m3: NOMINAL_PU_95A_DENSITY_KG_M3,
                anchor_key: None,
            },
        }],
        plug: rim_solid(spec),
        // The documented convention: the outermost body grown by the wall.
        // ⚠ Post-§Q-1 this no longer bounds the cup wall — that is a
        // body-tracking shell — but the field is still required and feeds the
        // platform and gasket paths, neither of which the wheel uses.
        bounding_region: cast_body_solid(spec).offset(wall_thickness_m),
        wall_thickness_m,
        mesh_cell_size_m,
        printer_config,
        mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        scan_mesh_for_plug_layer_0: None,
        plug_layer_0_mesh_cell_size_m: None,
        plug_layer_0_field_skin_m: None,
    }
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
    // Mirrors `dowel_hole` / `funnel`: test-only escapes so a fixture that
    // fails to build aborts the test loudly instead of being handled.
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use approx::assert_relative_eq;

    use nalgebra::{Point3, Vector3};

    use super::PlugRole;
    use super::{
        DimpleSpec, KeyingKind, NOMINAL_PU_95A_DENSITY_KG_M3, WheelSpec, cast_body_solid,
        locating_pin_is_solid, nominal_tire_volume_m3, rim_solid, tire_solid, wheel_cast_spec,
        wheel_mold_ribbon, wheel_ribbon,
    };
    use crate::bolt_pattern::{BoltPatternSpec, plan_smart_bolt_placements};
    use crate::dowel_hole::{DowelHoleSpec, plan_smart_dowel_placements, smart_dowel_footprint};
    use crate::error::CastTarget;
    use crate::mesh_csg::MatingTransform;
    use crate::piece::compose_piece_solid;
    use crate::pour::build_pour_gate_transforms;
    use crate::pour_volume::{
        DEFAULT_MASS_BUDGET_KG, POUR_VOLUME_MIN_CELL_SIZE_M, integrate_negative_sdf_volume,
    };
    use crate::ribbon::PieceSide;
    use crate::ribbon::Ribbon;
    use crate::seam_profile::SeamProfile;
    use crate::seam_solver::DEFAULT_MAX_PITCH_M;
    use crate::silhouette_2d::{SeamPlaneBasis, Silhouette2d};
    use crate::spec::STLS_SUBDIR;

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
        // ⚠ HISTORY, because the numbers here used to be much worse and the
        // gate encoded that. Under CORNER sampling this read −5.5 % at 2 mm,
        // −1.4 % at 1 mm and −2.2 % at 0.5 mm — an undercount at every cell,
        // and NOT monotone, because the error was an alignment artifact: a
        // face on a grid plane contributed no corners. This test asserted
        // `rel < 0.0` on the strength of it. `pour_volume` now samples cell
        // CENTRES, the wheel's flat side faces stop costing a plane, and what
        // remains is curvature — which converges.
        //
        // ⇒ assert the convergence, not a sign. A pinned tolerance per cell
        // would pass a rule that got the right answers for the wrong reason.
        let spec = WheelSpec::iter1();
        let nominal = nominal_tire_volume_m3(&spec);
        let rel = |cell_m: f64| (integrate(&spec, cell_m) - nominal) / nominal;
        let (coarse, fine) = (rel(PRODUCTION_CELL_M), rel(0.001));
        assert!(
            coarse.abs() < 0.07,
            "production cell {PRODUCTION_CELL_M} m is off by {coarse:+.4}"
        );
        assert!(fine.abs() < 0.01, "1 mm cell is off by {fine:+.4}");
        assert!(
            fine.abs() < coarse.abs(),
            "refining the cell must reduce the error; {coarse:+.4} → {fine:+.4}"
        );
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
        // The annulus plus the pin gap, with no hemispheres.
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
    fn the_bore_column_is_outside_every_wheel_solid() {
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
        // `compose_piece_solid`'s full pipeline. Two of the three stages it
        // deferred to have since run: the half-space intersect
        // (`the_locating_pin_survives_piece_composition`) and marching cubes
        // (`a_mold_piece_meshes_as_one_closed_orientable_shell`). The FLANGE
        // stage has not — `wheel_ribbon` carries `FlangeKind::None`, and
        // `compose_piece_solid` skips that branch whole unless
        // `lateral_reach_m()` is `Some`. M2b. (b) A FIRST
        // version of this test used `bounding.subtract(body)`, the composition
        // §Q-1 retired in May. It passed, on algebra the crate no longer runs.
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
        // form did not: the shell reaches only `wall_thickness_m` out from the
        // body's surface, so a wide bore grows a tube with a void down its
        // middle.
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

    /// The cup-wall thickness these mold gates compose at — `cf-cast-cli`'s
    /// `default_wall_thickness_m`.
    const WALL_M: f64 = WORKSHOP_WALL_M;

    #[test]
    fn wheel_ribbon_seam_is_exactly_the_parting_plane() {
        // A planar seam short-circuits `Ribbon::sdf` to the distance to one
        // flat plane, so the curve machinery the wheel cannot supply is never
        // reached. The seam must therefore read back as plain `z`.
        let ribbon = wheel_ribbon(&WheelSpec::iter1()).unwrap();
        for &r in &[0.0, 0.03, 0.065] {
            for &z in &[-0.012, -0.004, 0.0, 0.004, 0.012] {
                let d = ribbon.sdf(&Point3::new(r, 0.0, z));
                assert_relative_eq!(d, z, epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn the_seam_profile_rings_the_tire_not_the_bore() {
        // ⚠ `longest_polyline_with_arc_length` keeps only the LONGEST loop and
        // drops the rest silently. The wheel's parting-plane cross-section has
        // two, so the seal ring is chosen for the wheel rather than by it —
        // correct here, but by accident, which is why it is asserted.
        //
        // ★ The plan predicted the dropped loop would be the RIM. It is not:
        // the silhouette is taken of the CAST BODY, whose inner boundary is
        // the locating-pin column, not the rim. Same conclusion, different
        // geometry, and the number below is what distinguishes them.
        let spec = WheelSpec::iter1();
        let basis = SeamPlaneBasis::from_anchor_normal(Point3::origin(), Vector3::z());
        let reach = spec.tire_outer_radius_m * 1.25;
        let sil = Silhouette2d::from_body_in_plane(
            &cast_body_solid(&spec),
            basis,
            -reach,
            reach,
            -reach,
            reach,
        );

        let loops = sil.polylines();
        assert_eq!(loops.len(), 2, "expected the tire OD and the pin bore");
        // Max radius per loop — for concentric circles that IS the radius, and
        // it avoids a precision-losing `len()` cast.
        let radii: Vec<f64> = loops
            .iter()
            .map(|l| l.iter().map(|q| q.x.hypot(q.z)).fold(0.0_f64, f64::max))
            .collect();
        let (outer, inner) = (
            radii.iter().copied().fold(0.0_f64, f64::max),
            radii.iter().copied().fold(f64::MAX, f64::min),
        );
        assert_relative_eq!(outer, spec.tire_outer_radius_m, epsilon = 5e-4);
        assert_relative_eq!(inner, spec.locating_pin_radius_m(), epsilon = 5e-4);
        assert!(
            (inner - spec.rim_outer_radius_m).abs() > 0.04,
            "the dropped loop is the pin bore, not the rim"
        );

        // The kept loop is the tire's circumference, so the seal rings the
        // cavity's outer edge.
        let (_poly, _cum, perimeter) = sil
            .longest_polyline_with_arc_length()
            .expect("a closed silhouette");
        assert_relative_eq!(
            perimeter,
            std::f64::consts::TAU * spec.tire_outer_radius_m,
            epsilon = 5e-4
        );
        // ⚠ `SeamProfile` RESAMPLES the kept loop at a uniform arc-length step,
        // so it is not the same polygon and its perimeter is not the same
        // number — comparing the two at 1e-9 was an instrument error, not a
        // finding. Both are checked against the analytic circumference
        // instead: the silhouette polygon reads −3.6 ppm low and the
        // resampled profile −1.6e-5 relative, both from inscribing a circle
        // with chords.
        let profile = SeamProfile::from_silhouette(&sil).expect("profile from the kept loop");
        assert_relative_eq!(
            profile.perimeter(),
            std::f64::consts::TAU * spec.tire_outer_radius_m,
            epsilon = 5e-5
        );
    }

    #[test]
    fn each_mold_piece_is_one_half() {
        // The seam cut is SDF-side: `compose_piece_solid` intersects the cup
        // wall with the ribbon's half-space, so a piece must hold material on
        // its own side of z = 0 and none on the other.
        let spec = WheelSpec::iter1();
        let body = cast_body_solid(&spec);
        let r = spec.tire_outer_radius_m + WALL_M / 2.0;
        for (side, sign) in [(PieceSide::Negative, -1.0), (PieceSide::Positive, 1.0)] {
            let (piece, _tf) =
                compose_piece_solid(&body, WALL_M, &wheel_ribbon(&spec).unwrap(), side).unwrap();
            for &z in &[0.004, 0.010] {
                assert!(
                    piece.evaluate(&Point3::new(r, 0.0, sign * z)) < 0.0,
                    "{side:?} has no cup wall on its own side at z={}",
                    sign * z
                );
                assert!(
                    piece.evaluate(&Point3::new(r, 0.0, -sign * z)) > 0.0,
                    "{side:?} reaches across the seam to z={}",
                    -sign * z
                );
            }
        }
    }

    #[test]
    fn the_locating_pin_survives_piece_composition() {
        // M1 could only show the pin in the shell set algebra and said the
        // composed piece was M2's to prove. This is that: after the ribbon's
        // half-space intersect, each half still carries a solid pin down its
        // own half of the bore.
        //
        // ⚠ Scope: `wheel_ribbon` carries `FlangeKind::None`, and
        // `compose_piece_solid` skips the whole placement branch unless
        // `lateral_reach_m()` is `Some`. So the flange, bolt and dowel stages
        // did NOT run here — re-check the pin when M2b adds the Demand
        // flange, which is what puts geometry near the seam plane.
        let spec = WheelSpec::iter1();
        assert!(locating_pin_is_solid(&spec, WALL_M));
        let body = cast_body_solid(&spec);
        let half = spec.width_m / 2.0;
        for (side, sign) in [(PieceSide::Negative, -1.0), (PieceSide::Positive, 1.0)] {
            let (piece, _tf) =
                compose_piece_solid(&body, WALL_M, &wheel_ribbon(&spec).unwrap(), side).unwrap();
            for frac in [0.2_f64, 0.5, 0.8] {
                let z = sign * frac * half;
                assert!(
                    piece.evaluate(&Point3::new(0.0, 0.0, z)) < 0.0,
                    "{side:?} lost the locating pin at z={z}"
                );
            }
            // …and it stops at the pin radius, leaving the clearance ring free.
            let z = sign * half / 2.0;
            assert!(
                piece.evaluate(&Point3::new(spec.bore_radius_m, 0.0, z)) > 0.0,
                "{side:?} pin fills the clearance ring"
            );
        }
    }

    /// Build the loop the placement solver works from, the way
    /// `compose_piece_solid` does.
    fn seam_loops(spec: &WheelSpec, ribbon: &Ribbon) -> Vec<crate::seam_placement::LayerLoop> {
        let body = cast_body_solid(spec);
        let bounds = crate::piece::layer_mc_bounds(&body, WALL_M, ribbon).unwrap();
        crate::seam_placement::build_layer_loops(&[&body], &[bounds], ribbon, &ribbon.flange)
    }

    #[test]
    fn the_seam_loop_builds_on_a_circular_parting_plane() {
        // Everything the placement solver does starts here. Its own fixtures
        // are ELONGATED — a rotated `Solid::cylinder` (`bolt_pattern.rs:431`,
        // `dowel_hole.rs:505`) — so a principal axis is well defined there.
        // A circle has none, so it takes a code path those fixtures do not.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let loops = seam_loops(&spec, &ribbon);
        let (profile, exclusions) = loops[0].as_ref().expect("a circular seam must form a loop");
        assert_relative_eq!(
            profile.perimeter(),
            std::f64::consts::TAU * spec.tire_outer_radius_m,
            epsilon = 5e-5
        );
        // ⚠ This asserted `exclusions.is_empty()` through M2b, which was true
        // only because the ribbon had no pour gate. Now it has one, and the
        // solver excludes the bore as a swept channel so fasteners bracket it
        // rather than collide with it. An empty list here would mean the
        // placement solver does not know the gate exists.
        assert_eq!(exclusions.len(), 1, "the pour bore must be excluded once");
        let crate::seam_solver::Exclusion::Channel { a, b, half_width } = exclusions[0] else {
            panic!("the pour gate excludes a swept channel; got {exclusions:?}")
        };
        // The channel runs outward from the gate anchor along the loop's
        // in-plane v axis — 12 o'clock, on the x = 0 meridian.
        assert_relative_eq!(a.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(b.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(a.z, spec.gate_anchor_radius_m(), epsilon = 1e-9);
        assert!(
            b.z > spec.tire_outer_radius_m,
            "the bore must reach past the cavity to outside; ends at {}",
            b.z
        );
        assert!(half_width > 0.0);
    }

    #[test]
    fn dowels_land_far_apart_on_an_isotropic_loop() {
        // ★ THE DEGENERACY GATE. `plan_smart_dowel_placements` seeds at the
        // loop's PRINCIPAL-AXIS extremes (PCA over the stations) — and a
        // circle has no principal axis. Every placement fixture I checked
        // (`bolt_pattern.rs:431`, `:861`, `dowel_hole.rs:505`, `:670`) is a
        // rotated cylinder, so none of them reaches that branch.
        //
        // ⚠⚠ WHAT IS GUARANTEED IS MUCH WEAKER THAN "DIAMETRICALLY OPPOSITE",
        // and this gate asserted that until CI disproved it. On an isotropic
        // loop the principal axis is numerically ARBITRARY: perturbing the
        // tire radius by 0.1 mm swings the pair's absolute bearing from −0.05°
        // to +68.4° to +27.9°. Only the SEPARATION was stable locally
        // (179.57°–180.00°) — and on Linux the same code separates them by
        // 157.940°. The seeds start opposite; the feasibility solve then moves
        // them, and by how much depends on an axis that is chosen by
        // floating-point noise.
        //
        // ⇒ assert what registration actually needs — two dowels, one radius,
        // a large moment arm — not a number that happens to hold on one
        // platform. `placement_is_deterministic` covers same-platform
        // reproducibility, which is what matters for one exported mold.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let dowels = plan_smart_dowel_placements(
            &seam_loops(&spec, &ribbon),
            &DowelHoleSpec::iter1(),
            &ribbon.flange,
            WALL_M,
        );
        assert_eq!(dowels[0].len(), 2, "clamshell registration wants two");

        let (a, b) = (dowels[0][0], dowels[0][1]);
        let (ra, rb) = (a.x.hypot(a.z), b.x.hypot(b.z));
        assert_relative_eq!(ra, rb, epsilon = 1e-6);
        assert!(
            ra > spec.tire_outer_radius_m + WALL_M,
            "dowels must sit outboard of the cup wall, got r={ra}"
        );
        // ⚠ The raw difference must be WRAPPED. `(θb − θa).abs()` reads 202°
        // for a pair that is 158° apart, which is how the Linux failure first
        // looked like a 202° impossibility.
        let raw = (b.z.atan2(b.x) - a.z.atan2(a.x)).abs().to_degrees();
        let separation = raw.min(360.0 - raw);
        assert!(
            separation > 120.0,
            "clamshell registration needs a large moment arm; the dowels are \
             only {separation}° apart"
        );
    }

    #[test]
    fn bolts_ring_the_flange_clear_of_the_dowels() {
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let loops = seam_loops(&spec, &ribbon);
        let dspec = DowelHoleSpec::iter1();
        let dowels = plan_smart_dowel_placements(&loops, &dspec, &ribbon.flange, WALL_M);
        let footprint = smart_dowel_footprint(&dspec);
        let bolts = plan_smart_bolt_placements(
            &loops,
            &BoltPatternSpec::iter1(),
            &ribbon.flange,
            WALL_M,
            Some(footprint),
            Some(&dowels),
        );
        assert_eq!(bolts[0].len(), 16, "iter1 bolt count for this perimeter");

        // One ring. ⚠ NOT exactly one radius: a bolt centre is
        // `P(s) + d·n̂(s)` and the seam loop is a POLYGON, so the outward
        // normal wobbles between stations. Measured spread is 1.4 µm on a
        // 77 mm radius — 18 ppm. A 1e-6 tolerance failed here and that was the
        // instrument, not the geometry.
        let radii: Vec<f64> = bolts[0].iter().map(|b| b.x.hypot(b.z)).collect();
        let (lo, hi) = (
            radii.iter().copied().fold(f64::MAX, f64::min),
            radii.iter().copied().fold(0.0_f64, f64::max),
        );
        assert!(
            hi - lo < 1e-5,
            "bolt ring should be one radius to within the loop's own faceting; \
             spread {} m",
            hi - lo
        );
        assert!(hi > spec.tire_outer_radius_m + WALL_M);

        // Pitch is set along the SEAM LOOP, not the bolt circle — measured on
        // the loop it is 25.5 mm, inside the 30 mm maximum.
        let profile = loops[0].as_ref().unwrap().0.perimeter();
        let pitch = profile / 16.0;
        assert!(
            pitch <= DEFAULT_MAX_PITCH_M,
            "bolt pitch {pitch} exceeds the {DEFAULT_MAX_PITCH_M} maximum"
        );

        // §3.6: bolts are excluded from the dowel footprints.
        for b in &bolts[0] {
            for d in &dowels[0] {
                let gap = (b.x - d.x).hypot(b.z - d.z);
                assert!(
                    gap > footprint,
                    "bolt at {b:?} intrudes on dowel {d:?} ({gap} <= {footprint})"
                );
            }
        }
    }

    #[test]
    fn placement_is_deterministic() {
        // Six rims get printed against one mold. If placement moved between
        // runs the halves would stop registering, and a PCA fallback on an
        // isotropic loop is exactly where a tie could be broken arbitrarily.
        // `dowel_hole.rs` says ties break to the lowest arc length; this is
        // that claim, run.
        //
        // ⚠⚠ SCOPE: same binary, same platform. It does NOT establish
        // cross-platform reproducibility, and that is not a hypothetical —
        // this geometry separates its dowels by 180.000° on macOS and
        // 157.940° on Linux, because the principal axis of an isotropic loop
        // is decided by floating-point noise. Both are valid molds; they are
        // not the SAME mold. Re-exporting on a different machine after printing
        // one half is a workshop hazard nothing here guards.
        //
        // ⚠ A control for this gate must vary `tire_outer_radius_m`. Changing
        // `width_m` or `bore_radius_m` leaves the placements bit-identical,
        // because the seam loop is the z = 0 cross-section's OUTER boundary
        // and the pin-bore loop is the one that gets discarded
        // (`the_seam_profile_rings_the_tire_not_the_bore`). Perturbing the
        // width looked like a control and proved nothing.
        let spec = WheelSpec::iter1();
        let dspec = DowelHoleSpec::iter1();
        let first = {
            let r = wheel_mold_ribbon(&spec).unwrap();
            plan_smart_dowel_placements(&seam_loops(&spec, &r), &dspec, &r.flange, WALL_M)
        };
        for _ in 0..3 {
            let r = wheel_mold_ribbon(&spec).unwrap();
            let again =
                plan_smart_dowel_placements(&seam_loops(&spec, &r), &dspec, &r.flange, WALL_M);
            assert_eq!(again, first, "placement moved between identical builds");
        }
    }

    #[test]
    fn the_pour_gate_sits_at_twelve_oclock_in_the_seam_plane() {
        // ★ The orientation decision, finally expressed in code. The plan said
        // stand the wheel up so the cavity has ONE high point and gate there;
        // until now that lived only in prose, because the design frame says
        // nothing about gravity.
        //
        // `apex_axial_pose` anchors at `centerline[0]` along `-tangent`, so
        // this is decided entirely by which way `wheel_ribbon`'s stub runs.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let tf = build_pour_gate_transforms(&ribbon);
        assert_eq!(tf.len(), 1, "apex-axial is a single bore, no splayed vent");
        let MatingTransform::SubtractCylinder { params } = &tf[0] else {
            panic!("the pour gate carves a cylinder; got {:?}", tf[0])
        };
        let (c, axis) = (params.parent.center_m, params.parent.axis.into_inner());

        // In the seam plane (z = 0) and on the x = 0 meridian …
        assert_relative_eq!(c[2], 0.0, epsilon = 1e-9);
        assert_relative_eq!(c[0], 0.0, epsilon = 1e-9);
        // … pointing UP, not down: 12 o'clock, not 6.
        assert!(c[1] > 0.0, "gate anchored below the axle; got y={}", c[1]);
        assert_relative_eq!(axis.y, 1.0, epsilon = 1e-9);

        // ⚠ THE CONDITION THAT MAKES IT WORK: the bore axis must lie IN the
        // seam plane. An axis-aligned stub makes `outward` parallel to the
        // seam normal, which takes `apex_axial_pose`'s documented pathological
        // branch — the bore then points out of the seam, does not split
        // evenly, and runs down the wheel axis through the locating pin.
        let seam_normal = ribbon.seam_plane_reference().1.into_inner();
        assert_relative_eq!(axis.dot(&seam_normal), 0.0, epsilon = 1e-9);
    }

    #[test]
    fn the_pour_gate_carries_its_funnel() {
        // The integral split funnel ray-marches the body from the apex, and a
        // non-interior apex degrades it to a bore with NO funnel — silently,
        // returning `Some(channel)` either way. Anchoring mid-tread keeps the
        // apex inside the body; on the tire's outer surface the body SDF reads
        // exactly 0.0 and the cone disappears.
        //
        // ⚠⚠ "Does the piece protrude?" IS NOT A DETECTOR for this. The funnel
        // is `INTEGRAL_FUNNEL_HEIGHT_M` = 18 mm tall and the Demand flange
        // already reaches ~21.5 mm, so the cone never leaves the flange's
        // bounding box. Checking protrusion reported the funnel missing for
        // every anchor, including ones that had it.
        let spec = WheelSpec::iter1();
        let body = cast_body_solid(&spec);
        assert!(
            body.evaluate(&Point3::new(0.0, spec.gate_anchor_radius_m(), 0.0)) < 0.0,
            "the gate anchor must be strictly interior or the funnel degrades away"
        );
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let channel = crate::pour::build_integral_pour_channel(&ribbon, &body, 0.005)
            .expect("apex-axial builds an integral channel");
        let cone = channel
            .funnel_cone
            .expect("a funnel, not a bore-only degrade");
        let cb = cone.bounds().expect("the funnel is a bounded cone");
        // It starts at the cavity's outer surface and rises outward from there.
        assert!(cb.min.y >= spec.tire_outer_radius_m - 1e-3);
        assert!(cb.max.y > cb.min.y);
    }

    #[test]
    fn the_bore_breaches_the_wall_and_leaves_the_pin_alone() {
        // The bore has to open the cavity to the outside world at 12 o'clock,
        // and it has to do that without going anywhere near the axle, where
        // the locating pin stands. With the stub along the wheel axis it did
        // the opposite: a 5 mm-radius bore straight down the axis, through a
        // 3.9 mm-radius pin. (The 9.5 mm in the transform is the funnel's
        // FASTENER-CLEARANCE footprint — bore + funnel wall + gap — not the
        // bore; the SDF carve uses `gate_radius_m`.)
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let (piece, _tf) = compose_piece_solid(
            &cast_body_solid(&spec),
            WALL_M,
            &ribbon,
            PieceSide::Positive,
        )
        .unwrap();

        // Through the cup wall at 12 o'clock the bore is open …
        for y in [0.066_f64, 0.068, 0.070] {
            assert!(
                piece.evaluate(&Point3::new(0.0, y, 0.001)) > 0.0,
                "the bore is blocked at y={y}"
            );
        }
        // … while the same wall a centimetre to the side is solid.
        assert!(
            piece.evaluate(&Point3::new(0.013, 0.068, 0.001)) < 0.0,
            "the bore removed the whole wall, not just its own lumen"
        );
        // … and the pin is untouched.
        assert!(
            piece.evaluate(&Point3::new(0.0, 0.0, spec.width_m / 4.0)) < 0.0,
            "the bore reached the locating pin"
        );
    }

    #[test]
    fn the_locating_pin_survives_the_flange_stage() {
        // ✅ CLOSES THE DEFERRAL CARRIED SINCE M1. No earlier pin gate
        // exercised the flange stage: two never called `compose_piece_solid`
        // at all, and the one that did used `wheel_ribbon`, which carries
        // `FlangeKind::None` — and that makes `compose_piece_solid` skip its
        // whole placement branch. This one runs it: Demand flange, 16 bolt
        // holes and 2 dowel holes all carving near the seam plane, with the
        // pin standing in the middle of it.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let body = cast_body_solid(&spec);
        let half = spec.width_m / 2.0;
        for (side, sign) in [(PieceSide::Negative, -1.0), (PieceSide::Positive, 1.0)] {
            let (piece, transforms) = compose_piece_solid(&body, WALL_M, &ribbon, side).unwrap();
            assert_eq!(
                transforms.len(),
                18,
                "16 bolt holes + 2 dowel holes should be emitted post-MC"
            );
            for frac in [0.2_f64, 0.5, 0.8] {
                let z = sign * frac * half;
                assert!(
                    piece.evaluate(&Point3::new(0.0, 0.0, z)) < 0.0,
                    "{side:?} lost the locating pin at z={z} once the flange stage ran"
                );
            }
        }
    }

    #[test]
    fn a_flanged_piece_meshes_cleanly_through_mesh_csg() {
        // The real risk in this step: manifold3d's boolean carving 18 holes
        // into a genus-1 shell. Asserts component count BEFORE and AFTER the
        // CSG, because that is what caught a shattered mesh the winding
        // counters called clean.
        //
        // ★ Measured while proving this gate can fail: THE FLANGE ROUGHLY
        // DOUBLES THE COARSE-CELL TOLERANCE. Unflanged, the piece fragments at
        // a 5 mm cell (`a_mold_piece_meshes_as_one_closed_orientable_shell`);
        // flanged it survives 8 mm and fragments at 10 mm. Consistent with the
        // cup wall being what binds — the flange adds a thicker slab that
        // holds the piece together. ⚠ Do NOT read that as licence to coarsen:
        // the cavity surface is still the 5 mm wall.
        use crate::error::CastTarget;
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;
        use mesh_repair::components::find_connected_components;
        use mesh_repair::validate_mesh;

        // ⚠⚠ BOTH SIDES. This tested only `Positive` through M2c, and the
        // halves are NOT interchangeable: the first `CastSpec` export found 49
        // F4 issues on Negative against 8 on Positive. Most of that gap is an
        // orientation artifact — F4 measures overhang against +Z while each
        // half prints seam-face-down — but a gate that looks at one half
        // asserts nothing about the other, and this one looked at the cleaner.
        //
        // ★ The halves differ in more than F4's frame. Proving this gate can
        // fail, at a 10 mm cell NEGATIVE still meshes as one shell while
        // POSITIVE fragments — the reverse of the self-intersection asymmetry,
        // where Negative carries 24 pairs and Positive none. Neither half is
        // simply the better one.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        for side in [PieceSide::Negative, PieceSide::Positive] {
            let (solid, transforms) =
                compose_piece_solid(&cast_body_solid(&spec), WALL_M, &ribbon, side).unwrap();
            let target = CastTarget::MoldPiece {
                layer_index: 0,
                piece_side: side,
            };
            let mesh = solid_to_mm_mesh(&solid, 0.0015, target).expect("marching cubes");
            assert_eq!(
                find_connected_components(&mesh).component_count,
                1,
                "{side:?}: must mesh as one shell before any CSG"
            );
            let mesh = apply_mating_transforms(mesh, &transforms, target).expect("mesh-CSG");
            assert_eq!(
                find_connected_components(&mesh).component_count,
                1,
                "{side:?}: carving 18 holes must not detach anything"
            );
            let report = validate_mesh(&mesh);
            let census = report
                .winding
                .as_ref()
                .expect("validate_mesh enables the census by default");
            assert!(
                census.has_judgeable_edges(),
                "{side:?}: vacuous clean bill; {census:?}"
            );
            assert_eq!(
                (
                    census.boundary_edges,
                    census.non_manifold_edges,
                    census.degenerate_faces
                ),
                (0, 0, 0),
                "{side:?}: any of these hides the check below; {census:?}"
            );
            assert_eq!(census.inconsistent_edges, 0, "{side:?}: {census:?}");
        }
    }

    #[test]
    fn the_wheel_exports_a_full_mold_set() {
        // ★ THE EXPORT PATH, GATED. Every other wheel test drives
        // `compose_piece_solid` directly; this is the only one that runs
        // `export_molds_v2` — the entry point that assembles a `CastSpec`,
        // computes the pour volume, runs the F4 printability gate and writes
        // STLs. Without it the arc has "it worked when I ran it once".
        //
        // ⚠ The F4 gate here is the one that refused the v1 example on
        // 2026-05-12 and started this whole arc. It passes at the STRICT 1 mm
        // default min wall — the v2 scan example needs 0.1 mm.
        //
        // ⚠ Files: written under a pid-scoped temp dir and removed BEFORE the
        // assertions, so a failing assert cannot leak them.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let cast = wheel_cast_spec(
            &spec,
            WALL_M,
            // 2 mm keeps CI cheap (~1.4 s). Finer is a surface-quality choice
            // and does NOT change the reported mass — see `wheel_cast_spec`.
            0.0020,
            mesh_printability::PrinterConfig::fdm_default(),
        );

        let out = std::env::temp_dir().join(format!("cf-cast-wheel-{}", std::process::id()));
        std::fs::remove_dir_all(&out).ok();
        std::fs::create_dir_all(&out).expect("temp dir");
        let report = cast.export_molds_v2(&ribbon, &out);

        // Collect everything, THEN clean up, THEN assert.
        let observed = report.map(|r| {
            let stls: Vec<String> = std::fs::read_dir(out.join(STLS_SUBDIR))
                .map(|d| {
                    let mut v: Vec<String> = d
                        .flatten()
                        .map(|e| e.file_name().to_string_lossy().into_owned())
                        .collect();
                    v.sort();
                    v
                })
                .unwrap_or_default();
            let mass_g = r.layers[0].pour_volume.pour_mass_kg * 1000.0;
            let blocking: usize = r.layers[0]
                .pieces
                .iter()
                .map(|p| p.validation.issues.len())
                .sum();
            (
                r.layers.len(),
                r.platform.is_some(),
                r.funnel.is_some(),
                stls,
                mass_g,
                blocking,
            )
        });
        std::fs::remove_dir_all(&out).ok();

        let (layers, platform, funnel, stls, mass_g, _issues) =
            observed.expect("the wheel must survive the F4 gate at the strict 1 mm min wall");
        assert_eq!(layers, 1, "one pour");
        assert!(!platform, "no axial plug pins ⇒ no platform");
        assert!(
            !funnel,
            "apex-axial fuses the funnel into the cups — a separate funnel STL \
             would mean the layout silently fell back to V-at-dome"
        );
        assert_eq!(
            stls,
            vec![
                "dowel.stl".to_string(),
                "mold_layer_0_piece_0.stl".to_string(),
                "mold_layer_0_piece_1.stl".to_string(),
                "plug_layer_0.stl".to_string(),
            ],
            "two mold halves, the plug, and the printed dowel"
        );

        // ⚠ 115.11 g, not the 121.66 g the closed form gives. The integration
        // cell is floored at 2 mm and a 12.5 mm curved section reads −5.4 %
        // there; a finer mesh cell cannot change it. Pinned so the number the
        // workshop weighs against cannot drift unnoticed.
        assert_relative_eq!(mass_g, 115.11, epsilon = 0.01);
        let analytic_g = nominal_tire_volume_m3(&spec) * NOMINAL_PU_95A_DENSITY_KG_M3 * 1000.0;
        assert!(
            mass_g < analytic_g,
            "the reported mass still under-reads the closed form: {mass_g} vs {analytic_g}"
        );
        assert!(
            mass_g * 6.0 < DEFAULT_MASS_BUDGET_KG * 1000.0,
            "six tires must fit the 2 lb budget"
        );
    }

    #[test]
    fn the_wheels_sheet_casts_the_tire_onto_the_rim_and_leaves_it_there() {
        // ★ THE PROSE PATH, GATED. The rim is the product, so the two
        // workshop instructions a silicone cast takes for granted — release
        // the plug, then pull it out — each destroy a wheel. This renders the
        // sheet `write_procedure_v2` writes and reads it for both.
        //
        // ⚠ Rendered, not reasoned: `wheel_mold_ribbon` sets the role, but
        // what reaches the bencher is markdown, and every previous prose bug
        // in this crate was a writer that never learned about a flag.
        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        assert_eq!(
            ribbon.plug_role,
            PlugRole::Insert,
            "the rim is the product, not tooling"
        );
        let cast = wheel_cast_spec(
            &spec,
            WORKSHOP_WALL_M,
            PRODUCTION_CELL_M,
            mesh_printability::PrinterConfig::fdm_default(),
        );
        let pours = cast.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&cast, &pours, &ribbon);

        for phrase in ["Pull the plug", "off the plug", "release to all printed"] {
            assert!(
                !md.contains(phrase),
                "the wheel sheet still treats the rim as tooling: {phrase:?}"
            );
        }
        // ⚠ Anchors, not decoration. Without them this gate passes on an empty
        // string, and `contains` on a whole sheet cannot tell a sentence about
        // the CUPS from one about the plug — so assert the sentence PREFIX.
        assert!(
            md.contains("2. Apply mold release to the two cup halves."),
            "the cup halves still need release"
        );
        assert!(
            md.contains("`plug_layer_0.stl` gets NO mold release"),
            "and the rim still needs to be told it does not"
        );
        assert!(
            md.contains("Leave the plug IN"),
            "demold has to say what happens to the rim"
        );
    }

    #[test]
    fn self_intersections_stay_confined_to_the_seal_land_step() {
        // ⚠ The flanged Negative half carries 24 self-intersecting triangle
        // pairs, and F4 does not block on them — `SelfIntersecting` is not in
        // `is_blocking_critical`'s set, so the export writes the STL anyway.
        //
        // They are NOT the corruption class #764 chased: the mesh passes the
        // full connectivity census (one component, zero boundary, non-manifold,
        // degenerate and inconsistent edges). Located, they are ONE cluster of
        // 10 faces at r = 71.4–72.9 mm, z = −0.67 mm — straddling the seal
        // land's outer edge at 65.5 + 6.0 = 71.5 mm. The Positive half has
        // none.
        //
        // ★ That the land ENDS there is measured, not read off the spec:
        // sampling the piece at 72 angles, r ≤ 71 mm is solid at 69–71 of them
        // (a ring, interrupted only by the bore and the fastener holes) and
        // r ≥ 72 mm at just 25–36 (spokes). The continuous-to-spoked
        // transition brackets 71.5 mm, and the self-intersections sit across
        // it.
        //
        // ⇒ this gate BOUNDS a known artifact rather than asserting it away.
        // It fires if the count grows or if a pair appears anywhere else, which
        // is what would distinguish a new defect from this one. Whether 24
        // pairs in a 1.5 mm band matter is a SLICER question (M5), not a facet
        // one.
        use crate::error::CastTarget;
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;

        let spec = WheelSpec::iter1();
        let ribbon = wheel_mold_ribbon(&spec).unwrap();
        let land_outer_mm = (spec.tire_outer_radius_m * 1000.0) + 0.5 + 6.0;
        for side in [PieceSide::Negative, PieceSide::Positive] {
            let (solid, tf) =
                compose_piece_solid(&cast_body_solid(&spec), WALL_M, &ribbon, side).unwrap();
            let target = CastTarget::MoldPiece {
                layer_index: 0,
                piece_side: side,
            };
            let mesh = solid_to_mm_mesh(&solid, 0.0020, target).expect("marching cubes");
            let mesh = apply_mating_transforms(mesh, &tf, target).expect("mesh-CSG");
            let v = mesh_printability::validate_for_printing(
                &mesh,
                &mesh_printability::PrinterConfig::fdm_default(),
            )
            .expect("F4 runs");

            assert!(
                v.self_intersecting.len() <= 32,
                "{side:?}: {} self-intersecting pairs, was 24 — the artifact grew",
                v.self_intersecting.len()
            );
            for r in &v.self_intersecting {
                let loc = r.approximate_location;
                let radius = loc.x.hypot(loc.y);
                assert!(
                    (radius - land_outer_mm).abs() < 2.0 && loc.z.abs() < 2.0,
                    "{side:?}: a self-intersection escaped the seal-land step — \
                     r={radius:.2} mm z={:.2} mm, step at {land_outer_mm:.1} mm",
                    loc.z
                );
            }
        }
    }

    #[test]
    fn a_mold_piece_meshes_as_one_closed_orientable_shell() {
        // "Does a genus-1 body survive the mesher" was an inference from
        // reading. The cast body is a disc with a bore, and the cup wall that
        // tracks it wraps both surfaces — so this is the first time marching
        // cubes has been asked for that topology here. It does: clean at every
        // cell from 0.5 mm to 4 mm.
        //
        // ⚠ There is a resolution floor, and it is the CUP WALL, not the pin.
        // At a 5 mm cell the default 5 mm wall fragments into 3 components
        // (23 at 6 mm); holding the cell at 5 mm and thickening the wall to
        // 8 mm returns it to 1, while shrinking the pin to ~0.5 mm leaves it
        // at 3. One cell through the wall is not enough. ⇒ a production cell
        // must sit comfortably under `wall_thickness_m`.
        //
        // ★★ Every winding counter read ZERO at 6 mm, with 23 components. So
        // `boundary_edges == 0` would have called a shattered mesh clean — the
        // component precondition is the assertion that actually catches this,
        // which is the funnel docstring's warning made concrete.
        //
        // Follows `funnel::funnel_mesh_is_consistently_wound_and_outward`: a
        // zero inconsistent-edge count means nothing unless one shell was
        // meshed, the census had edges to judge, and it judged ALL of them.
        use crate::error::CastTarget;
        use crate::mesher::solid_to_mm_mesh;
        use mesh_repair::components::find_connected_components;
        use mesh_repair::validate_mesh;

        let spec = WheelSpec::iter1();
        let (piece, _tf) = compose_piece_solid(
            &cast_body_solid(&spec),
            WALL_M,
            &wheel_ribbon(&spec).unwrap(),
            PieceSide::Positive,
        )
        .unwrap();
        let mesh = solid_to_mm_mesh(
            &piece,
            0.0015,
            CastTarget::MoldPiece {
                layer_index: 0,
                piece_side: PieceSide::Positive,
            },
        )
        .expect("marching cubes on the wheel cup-wall half");

        let components = find_connected_components(&mesh).component_count;
        assert_eq!(
            components, 1,
            "the winding checks below compare nothing across shells, so they              are unsound unless there is exactly one; got {components}"
        );
        let report = validate_mesh(&mesh);
        let census = report
            .winding
            .as_ref()
            .expect("validate_mesh enables the census by default");
        assert!(
            census.has_judgeable_edges(),
            "no interior edge was judged, so a clean reading is vacuous; {census:?}"
        );
        assert_eq!(
            (
                census.boundary_edges,
                census.non_manifold_edges,
                census.degenerate_faces
            ),
            (0, 0, 0),
            "a non-manifold edge is dropped before the consistency check and a              degenerate face is skipped whole, so both hide from the assertion              below; {census:?}"
        );
        assert_eq!(
            census.inconsistent_edges, 0,
            "{} interior edges are walked the same way by both their faces;              {census:?}",
            census.inconsistent_edges
        );
    }

    #[test]
    #[should_panic(expected = "must fit inside the half-width")]
    fn a_dimple_wider_than_the_tread_face_is_rejected() {
        // A dimple whose radius reaches half the wheel's width breaks out
        // through its side faces. Needs a narrow wheel to reach: at the
        // defaults the tread depth and the half-width are both 12.5 mm, so
        // the tread-depth assert fires first.
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
