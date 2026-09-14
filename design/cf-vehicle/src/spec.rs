//! The vehicle as a parametric spec: geometry plus a lumped mass budget.

use crate::GRAVITY_M_S2;

/// One lumped mass in a vehicle's budget.
///
/// ★ **A budget line, not a part.** The rider, the frame, the front end,
/// the rear axle assembly — each is one entry with a mass and a position.
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

/// A rider-carried three-wheeler: one steered wheel at the front, two
/// driven-or-free wheels on a rear axle.
///
/// This is a **delta** layout (1 front, 2 rear), which is what a slingshot
/// drift trike is. ⚠ The rollover arithmetic in
/// [`CorneringLoads`](crate::CorneringLoads) is specific to that layout —
/// a tadpole (2 front, 1 rear) tips about a different line and this crate
/// does not model it.
#[derive(Debug, Clone, PartialEq)]
pub struct TrikeSpec {
    /// Front contact patch to rear axle contact line, metres.
    pub wheelbase_m: f64,
    /// Lateral distance between the two rear contact patches, metres.
    pub rear_track_m: f64,
    /// Front wheel rolling radius, metres.
    pub front_wheel_radius_m: f64,
    /// Rear wheel rolling radius, metres.
    pub rear_wheel_radius_m: f64,
    /// Steering axis angle from the **horizontal**, degrees.
    ///
    /// ⚠ From horizontal, not from vertical. 90° is a vertical head tube;
    /// smaller is slacker, i.e. raked back like a cruiser.
    pub head_angle_deg: f64,
    /// Fork offset — the perpendicular distance from the steering axis to
    /// the front wheel's centre, metres. Positive forward.
    pub fork_rake_m: f64,
    /// Every mass the vehicle carries, rider included.
    pub masses: Vec<MassItem>,
}

impl TrikeSpec {
    /// The planned first trike.
    ///
    /// ⚠ **Every mass here is an estimate and is meant to be corrected by
    /// weighing.** They are written as separate lines precisely so that a
    /// bathroom scale can replace them one at a time. The geometry is a
    /// conventional rideable slingshot layout: a 20″ front wheel, 11″ rear
    /// wheels, and a rider seated low and well aft.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            wheelbase_m: 1.25,
            rear_track_m: 0.75,
            // 20″ BMX front, 508 mm outside diameter.
            front_wheel_radius_m: 0.254,
            // 11″ rear, 280 mm outside diameter.
            rear_wheel_radius_m: 0.140,
            head_angle_deg: 68.0,
            fork_rake_m: 0.040,
            masses: vec![
                MassItem::new("rider", 80.0, 0.95, 0.35),
                MassItem::new("frame", 12.0, 0.60, 0.25),
                MassItem::new("front wheel, fork and bars", 4.0, 0.0, 0.30),
                MassItem::new("rear axle assembly", 6.0, 1.25, 0.14),
                MassItem::new("seat", 2.0, 0.95, 0.25),
            ],
        }
    }

    /// Panics unless every field is physically meaningful.
    ///
    /// # Panics
    ///
    /// Panics if any dimension is non-positive or non-finite, if the head
    /// angle is outside `(0°, 90°]`, if the mass budget is empty or holds
    /// a non-positive mass, or if the centre of gravity does not fall
    /// **between the two axles** — a vehicle whose CG is outside its
    /// wheelbase tips over standing still.
    pub fn assert_well_formed(&self) {
        assert!(
            self.wheelbase_m > 0.0 && self.wheelbase_m.is_finite(),
            "wheelbase must be positive and finite, got {}",
            self.wheelbase_m
        );
        assert!(
            self.rear_track_m > 0.0 && self.rear_track_m.is_finite(),
            "rear track must be positive and finite, got {}",
            self.rear_track_m
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
            self.head_angle_deg > 0.0 && self.head_angle_deg <= 90.0,
            "head angle is measured from the horizontal and must lie in \
             (0, 90] degrees, got {}",
            self.head_angle_deg
        );
        assert!(
            self.fork_rake_m >= 0.0 && self.fork_rake_m.is_finite(),
            "fork rake must be non-negative and finite, got {}",
            self.fork_rake_m
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
}

#[cfg(test)]
mod tests {
    use super::{MassItem, TrikeSpec};
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
    fn a_head_angle_past_vertical_is_rejected() {
        // ⚠ The classic unit trap: quoting a head angle from the VERTICAL
        // (a 22° "rake") instead of from the horizontal silently inverts
        // the trail. 112 degrees is the shape that mistake takes.
        TrikeSpec {
            head_angle_deg: 112.0,
            ..TrikeSpec::iter1()
        }
        .assert_well_formed();
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
    fn the_planned_trike_is_well_formed() {
        TrikeSpec::iter1().assert_well_formed();
    }
}
