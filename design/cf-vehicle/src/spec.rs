//! The vehicle as a parametric spec: geometry plus a lumped mass budget.

use crate::GRAVITY_M_S2;

/// Which end of the vehicle carries two wheels.
///
/// ★★★ **This is the single most consequential field in the spec.** One
/// axle has two wheels and one has a single wheel on the centreline, and
/// only the paired axle can resist a roll moment — so the layout decides
/// which end of the mass budget buys stability. Everything else about the
/// two layouts is shared; this one bit inverts the sign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Layout {
    /// **Two wheels at the front, one at the rear** — a *reverse trike*,
    /// the Polaris Slingshot arrangement. Roll is resisted at the front,
    /// so weight **forward** is what makes it stable.
    #[default]
    Tadpole,
    /// **One wheel at the front, two at the rear** — the classic pedal
    /// drift trike. Roll is resisted at the rear, so weight **rearward**
    /// is what makes it stable.
    Delta,
}

impl Layout {
    /// Where the lone wheel sits, in metres aft of the front contact
    /// patch, on a vehicle of the given wheelbase.
    #[must_use]
    pub const fn single_wheel_x_m(self, wheelbase_m: f64) -> f64 {
        match self {
            Self::Tadpole => wheelbase_m,
            Self::Delta => 0.0,
        }
    }

    /// Whether the paired axle is the one that steers.
    ///
    /// True for a [`Layout::Tadpole`], whose two front wheels steer on
    /// kingpins; false for a [`Layout::Delta`], whose single front wheel
    /// steers on a fork.
    #[must_use]
    pub const fn steers_on_the_paired_axle(self) -> bool {
        matches!(self, Self::Tadpole)
    }
}

/// How far the sprung mass leans, and what that costs.
///
/// ★★★ **Roll compliance is exactly equivalent to raising the centre of
/// gravity.** Working the moment balance with a roll angle in it, every
/// appearance of the CG height `h` is replaced by
///
/// ```text
/// h_eff  =  h  +  g · m_sprung · h_roll² / K_roll
/// ```
///
/// and nothing else changes. The vertical reactions stay statically
/// determinate — a three-wheeler always is — so the whole effect of
/// suspension on rollover is this one substitution.
///
/// ⚠ `h_roll` enters **squared**, so it hurts twice: a taller sprung mass
/// above the roll axis makes the roll moment bigger *and* makes the same
/// roll angle carry the CG further outboard.
///
/// ⚠ Its **sign does not matter**. A roll axis above the sprung CG makes
/// the body lean *into* the corner, but the CG is then below the axis and
/// still swings outboard. Only the magnitude of the separation counts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RollCompliance {
    /// Mass carried on the springs, kilograms — everything except wheels,
    /// uprights and the outboard half of the linkages.
    pub sprung_mass_kg: f64,
    /// Separation between the sprung mass's centre of gravity and the roll
    /// axis, metres. Positive when the CG is above the axis, which is the
    /// usual case; see the type docs on why the sign is immaterial.
    pub sprung_cg_above_roll_axis_m: f64,
    /// Total roll stiffness, newton-metres per radian — springs **and**
    /// any anti-roll bar, summed.
    pub roll_stiffness_n_m_per_rad: f64,
}

impl RollCompliance {
    /// Build from a wheel rate: `K = wheel_rate · track² / 2`.
    ///
    /// ⚠ **Wheel rate, not spring rate.** The rate measured at the contact
    /// patch is the unambiguous quantity; a coil mounted inboard on a
    /// linkage contributes `spring_rate × motion_ratio²`, and forgetting
    /// that square inflates the answer by `1 / motion_ratio²` — at a
    /// motion ratio of 0.7 the real stiffness is barely half what the
    /// spring rate alone suggests.
    #[must_use]
    pub fn from_wheel_rate(
        sprung_mass_kg: f64,
        sprung_cg_above_roll_axis_m: f64,
        wheel_rate_n_per_m: f64,
        track_m: f64,
    ) -> Self {
        Self {
            sprung_mass_kg,
            sprung_cg_above_roll_axis_m,
            roll_stiffness_n_m_per_rad: wheel_rate_n_per_m * track_m * track_m / 2.0,
        }
    }
}

/// One lumped mass in a vehicle's budget.
///
/// ★ **A budget line, not a part.** The rider, the frame, the front end,
/// the rear assembly — each is one entry with a mass and a position.
/// Splitting an entry in two is always safe (the centroid is unchanged if
/// the pieces are placed correctly); merging two into one loses the
/// ability to correct either independently.
///
/// ⚠ Positions follow the crate's coordinate convention: `x_m` is metres
/// aft of the front contact patch, `z_m` is metres above the ground. See
/// the [crate docs](crate).
#[derive(Debug, Clone, PartialEq)]
pub struct MassItem {
    /// What this mass is, for reporting. Not interpreted.
    pub name: String,
    /// Mass in kilograms. Must be positive and finite.
    pub mass_kg: f64,
    /// Longitudinal position, metres aft of the front contact patch.
    pub x_m: f64,
    /// Height above the ground plane, metres.
    pub z_m: f64,
}

impl MassItem {
    /// A budget line.
    #[must_use]
    pub fn new(name: impl Into<String>, mass_kg: f64, x_m: f64, z_m: f64) -> Self {
        Self {
            name: name.into(),
            mass_kg,
            x_m,
            z_m,
        }
    }
}

/// A rider-carried three-wheeler.
#[derive(Debug, Clone, PartialEq)]
pub struct TrikeSpec {
    /// Which end carries two wheels. See [`Layout`].
    pub layout: Layout,
    /// Front contact patch to rear contact patch, metres.
    pub wheelbase_m: f64,
    /// Lateral distance between the **paired** axle's two contact
    /// patches, metres. The lone wheel is on the centreline and has no
    /// track of its own.
    pub track_m: f64,
    /// Front wheel rolling radius, metres.
    pub front_wheel_radius_m: f64,
    /// Rear wheel rolling radius, metres.
    pub rear_wheel_radius_m: f64,
    /// Steering axis angle from the **horizontal**, degrees.
    ///
    /// ⚠ From horizontal, not from vertical. 90° is a vertical axis;
    /// smaller is raked back. A fork's head angle and a kingpin's caster
    /// are the same quantity in this convention — a 7° caster is 83° here.
    pub steering_axis_angle_deg: f64,
    /// Longitudinal offset of the steered wheel's centre from the steering
    /// axis, metres, positive forward. Fork rake on a delta, caster offset
    /// on a tadpole.
    pub steering_offset_m: f64,
    /// Every mass the vehicle carries, rider included.
    pub masses: Vec<MassItem>,
    /// Suspension roll compliance, or [`None`] for a rigid vehicle.
    ///
    /// ⚠ [`None`] is not "no suspension fitted" — it is "roll is not
    /// modelled". A rigid threshold is an **upper bound**: every real
    /// spring rate lowers it, so a design that only just clears with
    /// [`None`] does not clear at all.
    pub roll: Option<RollCompliance>,
}

impl TrikeSpec {
    /// The planned first trike: a **reverse trike**, two wheels forward.
    ///
    /// ⚠ **Every mass here is an estimate and is meant to be corrected by
    /// weighing.** They are written as separate lines precisely so that a
    /// bathroom scale can replace them one at a time.
    ///
    /// The rider sits behind the front axle and low, which is what the
    /// layout wants: on a tadpole, weight forward raises the rollover
    /// threshold *and* unloads the lone rear wheel that is meant to break
    /// away. Both effects pull the same way, which is the engineering
    /// argument for the layout.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            layout: Layout::Tadpole,
            wheelbase_m: 1.25,
            track_m: 0.90,
            // 16″ front, 406 mm outside diameter.
            front_wheel_radius_m: 0.2032,
            // 11″ rear, 280 mm outside diameter — the polyurethane one.
            rear_wheel_radius_m: 0.140,
            // 8° of caster.
            steering_axis_angle_deg: 82.0,
            steering_offset_m: 0.010,
            masses: vec![
                MassItem::new("rider", 80.0, 0.50, 0.30),
                MassItem::new("frame", 14.0, 0.60, 0.25),
                MassItem::new("front suspension, wheels and steering", 14.0, 0.0, 0.20),
                MassItem::new("rear wheel, swingarm and drive", 10.0, 1.25, 0.15),
                MassItem::new("seat", 2.0, 0.50, 0.22),
            ],
            // ⚠ Rigid, because no suspension has been laid out yet — so
            // every threshold this spec reports is an upper bound.
            roll: None,
        }
    }

    /// Panics unless every field is physically meaningful.
    ///
    /// # Panics
    ///
    /// Panics if any dimension is non-positive or non-finite, if the
    /// steering axis angle is outside `(0°, 90°]`, if the mass budget is
    /// empty or holds a non-positive mass, or if the centre of gravity
    /// does not fall **between the two axles** — a vehicle whose CG is
    /// outside its wheelbase tips over standing still.
    pub fn assert_well_formed(&self) {
        assert!(
            self.wheelbase_m > 0.0 && self.wheelbase_m.is_finite(),
            "wheelbase must be positive and finite, got {}",
            self.wheelbase_m
        );
        assert!(
            self.track_m > 0.0 && self.track_m.is_finite(),
            "track must be positive and finite, got {}",
            self.track_m
        );
        assert!(
            self.front_wheel_radius_m > 0.0 && self.front_wheel_radius_m.is_finite(),
            "front wheel radius must be positive and finite, got {}",
            self.front_wheel_radius_m
        );
        assert!(
            self.rear_wheel_radius_m > 0.0 && self.rear_wheel_radius_m.is_finite(),
            "rear wheel radius must be positive and finite, got {}",
            self.rear_wheel_radius_m
        );
        assert!(
            self.steering_axis_angle_deg > 0.0 && self.steering_axis_angle_deg <= 90.0,
            "the steering axis angle is measured from the horizontal and \
             must lie in (0, 90] degrees, got {}",
            self.steering_axis_angle_deg
        );
        assert!(
            self.steering_offset_m.is_finite(),
            "steering offset must be finite, got {}",
            self.steering_offset_m
        );
        assert!(
            !self.masses.is_empty(),
            "a vehicle with an empty mass budget has no loads to derive"
        );
        for item in &self.masses {
            assert!(
                item.mass_kg > 0.0 && item.mass_kg.is_finite(),
                "mass {:?} must be positive and finite, got {}",
                item.name,
                item.mass_kg
            );
            assert!(
                item.x_m.is_finite() && item.z_m.is_finite(),
                "mass {:?} must sit at a finite position, got ({}, {})",
                item.name,
                item.x_m,
                item.z_m
            );
            assert!(
                item.z_m >= 0.0,
                "mass {:?} sits below the ground plane at z = {}",
                item.name,
                item.z_m
            );
        }
        if let Some(roll) = self.roll {
            assert!(
                roll.sprung_mass_kg > 0.0 && roll.sprung_mass_kg.is_finite(),
                "sprung mass must be positive and finite, got {}",
                roll.sprung_mass_kg
            );
            assert!(
                roll.sprung_mass_kg <= self.total_mass_kg(),
                "sprung mass ({}) exceeds the vehicle's total mass ({})",
                roll.sprung_mass_kg,
                self.total_mass_kg()
            );
            assert!(
                roll.sprung_cg_above_roll_axis_m.is_finite(),
                "the sprung CG to roll axis separation must be finite, got {}",
                roll.sprung_cg_above_roll_axis_m
            );
            assert!(
                roll.roll_stiffness_n_m_per_rad > 0.0
                    && roll.roll_stiffness_n_m_per_rad.is_finite(),
                "roll stiffness must be positive and finite, got {} — a \
                 vehicle with zero roll stiffness has no resistance to \
                 overturning at all",
                roll.roll_stiffness_n_m_per_rad
            );
        }
        // ★ The one check that is engineering rather than hygiene. Outside
        // the wheelbase the static reaction at one axle goes negative,
        // which physically means that axle lifts — the vehicle is on its
        // nose or its tail before it has moved.
        let cg_x = self.cg_x_m();
        assert!(
            cg_x > 0.0 && cg_x < self.wheelbase_m,
            "the centre of gravity is at x = {cg_x} m, outside the \
             wheelbase (0, {}) — the vehicle tips over standing still",
            self.wheelbase_m
        );
    }

    /// Total mass carried, kilograms — rider included.
    #[must_use]
    pub fn total_mass_kg(&self) -> f64 {
        self.masses.iter().map(|m| m.mass_kg).sum()
    }

    /// Total weight, newtons.
    #[must_use]
    pub fn total_weight_n(&self) -> f64 {
        self.total_mass_kg() * GRAVITY_M_S2
    }

    /// Centre of gravity, metres aft of the front contact patch.
    ///
    /// # Panics
    ///
    /// Panics if the mass budget is empty.
    #[must_use]
    pub fn cg_x_m(&self) -> f64 {
        let total = self.total_mass_kg();
        assert!(total > 0.0, "an empty mass budget has no centre of gravity");
        self.masses.iter().map(|m| m.mass_kg * m.x_m).sum::<f64>() / total
    }

    /// Centre of gravity height above the ground, metres.
    ///
    /// # Panics
    ///
    /// Panics if the mass budget is empty.
    #[must_use]
    pub fn cg_z_m(&self) -> f64 {
        let total = self.total_mass_kg();
        assert!(total > 0.0, "an empty mass budget has no centre of gravity");
        self.masses.iter().map(|m| m.mass_kg * m.z_m).sum::<f64>() / total
    }

    /// The fraction of the total weight carried by the **paired** axle.
    ///
    /// ★ **One derivation, both layouts.** Moments about the lone wheel
    /// give the paired axle's share directly: it is the CG's distance from
    /// the lone wheel, as a fraction of the wheelbase. The layout does
    /// nothing but say where the lone wheel is, and the sign of every
    /// stability conclusion follows from that alone.
    ///
    /// # Panics
    ///
    /// Panics if `self` is not well-formed. ⚠ The validation is not
    /// ceremony: the `abs` below would otherwise turn a centre of gravity
    /// that has fallen outside the wheelbase into a small, entirely
    /// plausible-looking share instead of an obvious error.
    #[must_use]
    pub fn paired_axle_share(&self) -> f64 {
        self.assert_well_formed();
        let single_x = self.layout.single_wheel_x_m(self.wheelbase_m);
        (self.cg_x_m() - single_x).abs() / self.wheelbase_m
    }

    /// The centre-of-gravity height the roll arithmetic actually sees.
    ///
    /// Equal to [`TrikeSpec::cg_z_m`] for a rigid vehicle. With roll
    /// compliance it is taller, by `g · m_sprung · h_roll² / K_roll` — see
    /// [`RollCompliance`] for why that single substitution is the whole
    /// effect of suspension on rollover.
    ///
    /// # Panics
    ///
    /// Panics if `self` is not well-formed. ⚠ Again load-bearing: a roll
    /// stiffness of zero would divide to infinity here and report a
    /// rollover threshold of exactly zero, which reads like an answer.
    #[must_use]
    pub fn effective_cg_height_m(&self) -> f64 {
        self.assert_well_formed();
        let h = self.cg_z_m();
        self.roll.map_or(h, |roll| {
            h + GRAVITY_M_S2 * roll.sprung_mass_kg * roll.sprung_cg_above_roll_axis_m.powi(2)
                / roll.roll_stiffness_n_m_per_rad
        })
    }

    /// Rolling radius of the wheels on the paired axle, metres.
    #[must_use]
    pub const fn paired_wheel_radius_m(&self) -> f64 {
        match self.layout {
            Layout::Tadpole => self.front_wheel_radius_m,
            Layout::Delta => self.rear_wheel_radius_m,
        }
    }

    /// Rolling radius of the lone wheel, metres.
    #[must_use]
    pub const fn single_wheel_radius_m(&self) -> f64 {
        match self.layout {
            Layout::Tadpole => self.rear_wheel_radius_m,
            Layout::Delta => self.front_wheel_radius_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Layout, MassItem, RollCompliance, TrikeSpec};
    use approx::assert_relative_eq;

    #[test]
    #[should_panic(expected = "outside the wheelbase")]
    fn a_cg_behind_the_rear_axle_is_rejected() {
        // The reaction at the front axle goes negative, which physically
        // means the front wheel lifts — the trike is sitting on its tail
        // before it has moved.
        TrikeSpec {
            masses: vec![MassItem::new("rider", 80.0, 1.40, 0.35)],
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    #[should_panic(expected = "outside the wheelbase")]
    fn a_cg_ahead_of_the_front_contact_patch_is_rejected() {
        TrikeSpec {
            masses: vec![MassItem::new("rider", 80.0, -0.10, 0.35)],
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    #[should_panic(expected = "below the ground plane")]
    fn a_mass_below_the_ground_is_rejected() {
        TrikeSpec {
            masses: vec![MassItem::new("sunken", 80.0, 0.90, -0.01)],
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    #[should_panic(expected = "measured from the horizontal")]
    fn a_steering_axis_angle_past_vertical_is_rejected() {
        // ⚠ The classic unit trap: quoting the angle from the VERTICAL (an
        // 8° "caster") instead of from the horizontal silently inverts the
        // trail. 112 degrees is the shape that mistake takes.
        TrikeSpec {
            steering_axis_angle_deg: 112.0,
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    fn the_layout_only_says_where_the_lone_wheel_is() {
        // ★ The whole delta/tadpole difference, in one place. Everything
        // downstream reads `paired_axle_share`, which reads this.
        assert_relative_eq!(
            Layout::Tadpole.single_wheel_x_m(1.25),
            1.25,
            epsilon = 1e-12
        );
        assert_relative_eq!(Layout::Delta.single_wheel_x_m(1.25), 0.0, epsilon = 1e-12);
        assert!(Layout::Tadpole.steers_on_the_paired_axle());
        assert!(!Layout::Delta.steers_on_the_paired_axle());
        assert_eq!(Layout::default(), Layout::Tadpole);
    }

    #[test]
    fn the_paired_share_is_a_distance_from_the_lone_wheel() {
        // Both layouts, one derivation: the CG's distance from whichever
        // wheel is alone, over the wheelbase.
        let masses = vec![MassItem::new("all", 100.0, 0.40, 0.30)];
        let tadpole = TrikeSpec {
            layout: Layout::Tadpole,
            masses: masses.clone(),
            ..TrikeSpec::iter1()
        };
        let delta = TrikeSpec {
            layout: Layout::Delta,
            masses,
            ..TrikeSpec::iter1()
        };
        assert_relative_eq!(
            tadpole.paired_axle_share(),
            (1.25 - 0.40) / 1.25,
            epsilon = 1e-12
        );
        assert_relative_eq!(delta.paired_axle_share(), 0.40 / 1.25, epsilon = 1e-12);
        assert_relative_eq!(
            tadpole.paired_axle_share() + delta.paired_axle_share(),
            1.0,
            epsilon = 1e-12
        );
    }

    #[test]
    #[should_panic(expected = "empty mass budget")]
    fn an_empty_mass_budget_is_rejected() {
        TrikeSpec {
            masses: Vec::new(),
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    fn splitting_a_budget_line_in_two_moves_no_centroid() {
        // ★ The claim in `MassItem`'s docs, gated. Refining an estimate by
        // breaking it up must never move the answer on its own, or every
        // refinement would silently be a design change.
        let spec = TrikeSpec::iter1();
        let mut split = spec.masses.clone();
        let rider = split.remove(0);
        split.push(MassItem::new(
            "rider torso",
            rider.mass_kg * 0.6,
            rider.x_m - 0.05,
            rider.z_m + 0.12,
        ));
        split.push(MassItem::new(
            "rider legs",
            rider.mass_kg * 0.4,
            rider.x_m + 0.075,
            rider.z_m - 0.18,
        ));
        let refined = TrikeSpec {
            masses: split,
            ..TrikeSpec::iter1()
        };
        assert_relative_eq!(
            refined.total_mass_kg(),
            spec.total_mass_kg(),
            epsilon = 1e-12
        );
        assert_relative_eq!(refined.cg_x_m(), spec.cg_x_m(), epsilon = 1e-12);
        assert_relative_eq!(refined.cg_z_m(), spec.cg_z_m(), epsilon = 1e-12);
    }

    #[test]
    #[should_panic(expected = "no resistance to overturning")]
    fn zero_roll_stiffness_is_rejected() {
        TrikeSpec {
            roll: Some(RollCompliance {
                sprung_mass_kg: 96.0,
                sprung_cg_above_roll_axis_m: 0.20,
                roll_stiffness_n_m_per_rad: 0.0,
            }),
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    #[should_panic(expected = "exceeds the vehicle's total mass")]
    fn a_sprung_mass_larger_than_the_vehicle_is_rejected() {
        // ⚠ Sprung mass is a SUBSET — wheels, uprights and the outboard
        // half of the linkages are not on the springs. Handing it the
        // whole vehicle mass is the easy mistake, and it silently
        // overstates the roll penalty.
        TrikeSpec {
            roll: Some(RollCompliance::from_wheel_rate(500.0, 0.20, 20_000.0, 0.90)),
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
    }

    #[test]
    fn the_planned_trike_is_well_formed() {
        TrikeSpec::iter1().assert_well_formed();
    }
}
