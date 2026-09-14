//! What the geometry and the mass budget imply: axle loads, cornering
//! loads, the rollover threshold, and steering trail.
//!
//! ★ **A three-wheeler is statically determinate.** Three contact points,
//! three unknown normal forces, three equilibrium equations — so every
//! number in this module is solved, not apportioned. Four-wheelers need a
//! roll-stiffness split to answer the same question, and that split is an
//! assumption. We get to skip it.

use crate::TrikeSpec;

/// The vehicle standing still on level ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticLoads {
    /// Total weight, newtons.
    pub total_weight_n: f64,
    /// Vertical reaction at the single front contact patch, newtons.
    pub front_n: f64,
    /// Vertical reaction summed over both rear contact patches, newtons.
    pub rear_total_n: f64,
    /// Vertical reaction at one rear wheel, newtons.
    pub per_rear_wheel_n: f64,
    /// Fraction of the total weight carried by the rear axle, `0..1`.
    pub rear_share: f64,
    /// Centre of gravity, metres aft of the front contact patch.
    pub cg_x_m: f64,
    /// Centre of gravity height, metres.
    pub cg_z_m: f64,
}

impl StaticLoads {
    /// Solve the standing-still load case.
    ///
    /// Moments about the rear axle give the front reaction directly:
    /// `front = W · (L − x_cg) / L`, so the **rear share is `x_cg / L`** and
    /// nothing else. ★ This is the number the wheel arc previously carried
    /// as a hand-written 70 %.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed — see
    /// [`TrikeSpec::assert_well_formed`].
    #[must_use]
    pub fn of(spec: &TrikeSpec) -> Self {
        spec.assert_well_formed();
        let total_weight_n = spec.total_weight_n();
        let cg_x_m = spec.cg_x_m();
        let rear_share = cg_x_m / spec.wheelbase_m;
        let rear_total_n = total_weight_n * rear_share;
        Self {
            total_weight_n,
            front_n: total_weight_n - rear_total_n,
            rear_total_n,
            per_rear_wheel_n: rear_total_n / 2.0,
            rear_share,
            cg_x_m,
            cg_z_m: spec.cg_z_m(),
        }
    }

    /// One rear wheel's vertical load multiplied by a dynamic factor.
    ///
    /// ⚠ **The factor is the caller's, deliberately.** Kerbs, drops and
    /// potholes are not derivable from a static mass budget, and a
    /// hard-coded 3× buried in a library is exactly the kind of unowned
    /// number this crate exists to abolish. Pass one you can defend.
    #[must_use]
    pub fn per_rear_wheel_dynamic_n(&self, factor: f64) -> f64 {
        self.per_rear_wheel_n * factor
    }
}

/// The vehicle in a steady-state corner on level ground.
///
/// ⚠ **Valid only below [`CorneringLoads::rollover_threshold_g`].** Above
/// it the inner rear wheel has left the ground, the vehicle is on two
/// contacts, and these numbers describe a rigid body that no longer
/// matches reality. [`CorneringLoads::inner_rear_lifted`] reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorneringLoads {
    /// The lateral acceleration solved for, in units of `g`.
    pub lateral_accel_g: f64,
    /// Vertical reaction at the front contact patch, newtons.
    ///
    /// ★ Unchanged from static. A pure lateral acceleration makes no
    /// moment about the lateral axis, so it cannot move load fore or aft.
    pub front_n: f64,
    /// Vertical reaction at the **outer** rear wheel, newtons.
    pub outer_rear_n: f64,
    /// Vertical reaction at the **inner** rear wheel, newtons.
    ///
    /// Negative means the rigid-body solution would need the wheel to pull
    /// down on the road; physically it has lifted.
    pub inner_rear_n: f64,
    /// Lateral acceleration at which the inner rear wheel lifts, in `g`.
    pub rollover_threshold_g: f64,
}

impl CorneringLoads {
    /// Solve the cornering load case at a given lateral acceleration.
    ///
    /// ★★★ **The whole roll moment lands on the rear axle.** The front
    /// contact patch sits on the centreline, so it has no lateral lever
    /// about the roll axis and resists none of it — the two rear wheels
    /// take all of `W · (a/g) · z_cg`, reacted as a couple across the
    /// track. That, and not a low CG alone, is why three-wheelers tip.
    ///
    /// The transfer is therefore `ΔF = W · (a/g) · z_cg / t`, using the
    /// **total** weight and not the rear axle's share of it.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed, or if `lateral_accel_g` is not
    /// finite and non-negative.
    #[must_use]
    pub fn at(spec: &TrikeSpec, lateral_accel_g: f64) -> Self {
        assert!(
            lateral_accel_g >= 0.0 && lateral_accel_g.is_finite(),
            "lateral acceleration must be non-negative and finite, got \
             {lateral_accel_g}"
        );
        let statics = StaticLoads::of(spec);
        let transfer_n =
            statics.total_weight_n * lateral_accel_g * statics.cg_z_m / spec.rear_track_m;
        Self {
            lateral_accel_g,
            front_n: statics.front_n,
            outer_rear_n: statics.per_rear_wheel_n + transfer_n,
            inner_rear_n: statics.per_rear_wheel_n - transfer_n,
            rollover_threshold_g: rollover_threshold_g(spec),
        }
    }

    /// Whether the inner rear wheel has left the ground.
    #[must_use]
    pub fn inner_rear_lifted(&self) -> bool {
        self.inner_rear_n <= 0.0
    }

    /// The lateral force one rear wheel transmits at its contact patch at
    /// a given tyre friction coefficient, newtons.
    ///
    /// This is what loads a wheel out of its own plane — on an FDM rim,
    /// straight into the layer-adhesion axis.
    #[must_use]
    pub fn outer_rear_lateral_n(&self, tyre_mu: f64) -> f64 {
        self.outer_rear_n * tyre_mu
    }

    /// The moment that lateral force applies at the rear hub, newton-metres.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed.
    #[must_use]
    pub fn outer_rear_hub_moment_n_m(&self, spec: &TrikeSpec, tyre_mu: f64) -> f64 {
        spec.assert_well_formed();
        self.outer_rear_lateral_n(tyre_mu) * spec.rear_wheel_radius_m
    }

    /// ★★★ **Does it slide before it tips?**
    ///
    /// The road can transmit at most `µ · g` of lateral acceleration. If
    /// that is below the rollover threshold the tyres break away first and
    /// the vehicle drifts; if it is above, the geometry runs out before
    /// the grip does and the vehicle rolls over.
    ///
    /// ⚠ Conservative by construction: it uses the largest `µ` present at
    /// any contact, because the question is whether the road *can* deliver
    /// enough lateral force to reach the threshold at all.
    #[must_use]
    pub fn slides_before_it_tips(&self, tyre_mu: f64) -> bool {
        tyre_mu < self.rollover_threshold_g
    }
}

/// The lateral acceleration, in `g`, at which the inner rear wheel lifts.
///
/// Setting the inner reaction to zero and solving gives a compact result:
///
/// ```text
/// a/g  =  rear_share · track / (2 · cg_height)
/// ```
///
/// ★ **That is the familiar two-wheeled static stability factor,
/// `t / 2h`, scaled by the rear share** — a delta trike is exactly
/// `rear_share` as roll-stable as a four-wheeler of the same track and CG
/// height. Two consequences fall straight out, and the second one is
/// counter-intuitive:
///
/// - the term that dominates is **CG height**, not track;
/// - moving weight **rearward makes a delta trike more stable**, because
///   the tipping line runs from the single front contact to the outer rear
///   one, and a CG further aft sits further inboard of it. The opposite is
///   true of a tadpole, which this crate does not model.
///
/// ⚠ Quasi-static. Kerbs, camber and abrupt steering all tip vehicles that
/// clear this number.
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see
/// [`TrikeSpec::assert_well_formed`].
#[must_use]
pub fn rollover_threshold_g(spec: &TrikeSpec) -> f64 {
    spec.assert_well_formed();
    let rear_share = spec.cg_x_m() / spec.wheelbase_m;
    rear_share * spec.rear_track_m / (2.0 * spec.cg_z_m())
}

/// Front-end steering geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeringGeometry {
    /// Ground trail — how far behind the steering axis the front contact
    /// patch drags, measured along the ground, metres.
    pub trail_m: f64,
    /// Mechanical trail — the perpendicular distance from the contact
    /// patch to the steering axis, metres. This is the lever the tyre's
    /// side force actually pulls on, so it, not ground trail, sets steering
    /// feel.
    pub mechanical_trail_m: f64,
}

impl SteeringGeometry {
    /// Derive trail from head angle, fork rake and front wheel radius.
    ///
    /// With the head angle `α` measured from the horizontal,
    /// `trail = (R · cos α − rake) / sin α`, and mechanical trail is that
    /// times `sin α`, i.e. `R · cos α − rake`.
    ///
    /// ⚠ Negative trail is geometrically possible — enough fork rake puts
    /// the contact patch *ahead* of the steering axis — and it makes the
    /// steering diverge instead of self-centre. It is reported, not
    /// rejected, because the caller may be sweeping rake deliberately.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed — see
    /// [`TrikeSpec::assert_well_formed`].
    #[must_use]
    pub fn of(spec: &TrikeSpec) -> Self {
        spec.assert_well_formed();
        let alpha = spec.head_angle_deg.to_radians();
        let mechanical_trail_m = spec.front_wheel_radius_m * alpha.cos() - spec.fork_rake_m;
        Self {
            trail_m: mechanical_trail_m / alpha.sin(),
            mechanical_trail_m,
        }
    }

    /// Whether the front end self-centres.
    #[must_use]
    pub fn self_centres(&self) -> bool {
        self.trail_m > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::{CorneringLoads, StaticLoads, SteeringGeometry, rollover_threshold_g};
    use crate::{MassItem, TrikeSpec};
    use approx::assert_relative_eq;

    /// A spec that is `iter1` except for its mass budget.
    fn with_masses(masses: Vec<MassItem>) -> TrikeSpec {
        TrikeSpec {
            masses,
            ..TrikeSpec::iter1()
        }
    }

    #[test]
    fn the_rear_share_is_the_cg_position_not_a_chosen_fraction() {
        // ★ THE CLAIM THIS CRATE EXISTS FOR. The wheel arc carried a
        // hand-written 70 %. A rear share is `x_cg / L` and cannot be
        // anything else, so it must MOVE when the mass budget moves.
        let spec = TrikeSpec::iter1();
        let base = StaticLoads::of(&spec);
        assert_relative_eq!(
            base.rear_share,
            spec.cg_x_m() / spec.wheelbase_m,
            epsilon = 1e-12
        );

        // Slide the rider 150 mm aft: the share must rise, and by the
        // amount the CG moved — not by some damped fraction of it.
        let mut aft = spec.masses.clone();
        aft[0].x_m += 0.15;
        let moved = StaticLoads::of(&with_masses(aft));
        assert!(
            moved.rear_share > base.rear_share,
            "moving the rider aft left the rear share at {}",
            moved.rear_share
        );
        let expected_delta = 0.15 * 80.0 / spec.total_mass_kg() / spec.wheelbase_m;
        assert_relative_eq!(
            moved.rear_share - base.rear_share,
            expected_delta,
            epsilon = 1e-12
        );
    }

    #[test]
    fn the_axle_reactions_sum_to_the_weight() {
        // Conservation. Nothing may be created or lost between the CG and
        // the three contact patches.
        for shift in [-0.30, -0.10, 0.0, 0.10, 0.25] {
            let mut masses = TrikeSpec::iter1().masses;
            for m in &mut masses {
                m.x_m += shift;
            }
            let loads = StaticLoads::of(&with_masses(masses));
            assert_relative_eq!(
                loads.front_n + loads.rear_total_n,
                loads.total_weight_n,
                epsilon = 1e-9
            );
            assert_relative_eq!(
                loads.per_rear_wheel_n * 2.0,
                loads.rear_total_n,
                epsilon = 1e-12
            );
        }
    }

    #[test]
    fn a_cg_over_the_rear_axle_puts_every_newton_on_it() {
        // Limit case, approached rather than reached: `assert_well_formed`
        // rejects a CG exactly on an axle. At 99 % of the wheelbase the
        // front must carry exactly the remaining 1 %.
        let spec = TrikeSpec::iter1();
        let x = 0.99 * spec.wheelbase_m;
        let loads = StaticLoads::of(&with_masses(vec![MassItem::new("all", 100.0, x, 0.3)]));
        assert_relative_eq!(loads.rear_share, 0.99, epsilon = 1e-12);
        assert_relative_eq!(loads.front_n, 0.01 * loads.total_weight_n, epsilon = 1e-9);
    }

    #[test]
    fn the_front_wheel_resists_no_roll_moment() {
        // ★★★ THE NON-OBVIOUS ONE. The front contact sits on the
        // centreline, so it has no lever about the roll axis: the rear
        // axle takes the WHOLE roll moment, sized by the TOTAL weight and
        // not by the rear axle's share of it.
        //
        // Two vehicles, same total mass, same track, same CG height,
        // wildly different fore-aft balance. If the transfer used the rear
        // share, these would differ by 80 %.
        let level = 0.30;
        let a = with_masses(vec![
            MassItem::new("fore", 50.0, 0.40, level),
            MassItem::new("aft", 50.0, 0.85, level),
        ]);
        let b = with_masses(vec![
            MassItem::new("fore", 50.0, 1.00, level),
            MassItem::new("aft", 50.0, 1.25, level),
        ]);
        assert_relative_eq!(a.cg_z_m(), b.cg_z_m(), epsilon = 1e-12);
        assert_relative_eq!(a.total_mass_kg(), b.total_mass_kg(), epsilon = 1e-12);
        assert!(
            (a.cg_x_m() - b.cg_x_m()).abs() > 0.4,
            "the two fixtures must actually differ in balance"
        );

        let transfer = |spec: &TrikeSpec| {
            let c = CorneringLoads::at(spec, 0.5);
            (c.outer_rear_n - c.inner_rear_n) / 2.0
        };
        assert_relative_eq!(transfer(&a), transfer(&b), epsilon = 1e-9);

        // And it is the total weight, not the rear share, that sets it.
        let expected = 100.0 * crate::GRAVITY_M_S2 * 0.5 * level / a.rear_track_m;
        assert_relative_eq!(transfer(&a), expected, epsilon = 1e-9);
    }

    #[test]
    fn the_inner_rear_lifts_exactly_at_the_closed_form_threshold() {
        // ★ CROSS-CHECK THAT BYPASSES THE ARTIFACT. `rollover_threshold_g`
        // is a closed form. This finds the same number by bisecting the
        // *solved load case* for the acceleration at which the inner
        // reaction crosses zero — a different code path entirely.
        for spec in [
            TrikeSpec::iter1(),
            with_masses(vec![MassItem::new("low", 90.0, 0.70, 0.18)]),
            with_masses(vec![MassItem::new("high", 60.0, 1.10, 0.55)]),
        ] {
            let (mut lo, mut hi) = (0.0_f64, 10.0_f64);
            assert!(
                !CorneringLoads::at(&spec, hi)
                    .inner_rear_n
                    .is_sign_positive(),
                "10 g must be past the threshold for any sane trike"
            );
            for _ in 0..200 {
                let mid = f64::midpoint(lo, hi);
                if CorneringLoads::at(&spec, mid).inner_rear_n > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            assert_relative_eq!(
                f64::midpoint(lo, hi),
                rollover_threshold_g(&spec),
                epsilon = 1e-9
            );
        }
    }

    #[test]
    fn a_cg_on_the_rear_axle_recovers_the_two_wheel_stability_factor() {
        // Round numbers on purpose: track 0.8 m at a CG height of 0.4 m is
        // a four-wheeler SSF of exactly 1.0 g. A delta trike with 80 % of
        // its weight aft must land on exactly 0.8 of that.
        let spec = TrikeSpec {
            wheelbase_m: 1.0,
            rear_track_m: 0.80,
            masses: vec![MassItem::new("all", 100.0, 0.80, 0.40)],
            ..TrikeSpec::iter1()
        };
        assert_relative_eq!(
            spec.rear_track_m / (2.0 * spec.cg_z_m()),
            1.0,
            epsilon = 1e-12
        );
        assert_relative_eq!(rollover_threshold_g(&spec), 0.80, epsilon = 1e-12);
    }

    #[test]
    fn moving_weight_rearward_makes_a_delta_trike_more_stable() {
        // ⚠ COUNTER-INTUITIVE, AND GATED BECAUSE OF IT. The tipping line
        // runs from the single front contact to the outer rear one, so a
        // CG further aft sits further inboard of it. A tadpole behaves the
        // other way round; this crate does not model one.
        let spec = TrikeSpec::iter1();
        let base = rollover_threshold_g(&spec);
        let shifted = |dx: f64| {
            let mut masses = spec.masses.clone();
            masses[0].x_m += dx;
            rollover_threshold_g(&with_masses(masses))
        };
        assert!(
            shifted(0.15) > base,
            "rearward must raise the threshold: {} vs {base}",
            shifted(0.15)
        );
        assert!(
            shifted(-0.15) < base,
            "forward must lower it: {} vs {base}",
            shifted(-0.15)
        );
    }

    #[test]
    fn trail_matches_an_independent_vector_construction() {
        // ★ MIRROR ORACLE. The implementation uses the closed form
        // `(R cos α − rake) / sin α`. This builds the steering axis and the
        // wheel centre as vectors, drops the contact patch onto the ground
        // and measures — never touching that expression.
        for (head_deg, rake, radius) in [
            (68.0, 0.040, 0.254),
            (71.0, 0.045, 0.350),
            (90.0, 0.000, 0.200),
            (55.0, 0.070, 0.300),
        ] {
            let spec = TrikeSpec {
                head_angle_deg: head_deg,
                fork_rake_m: rake,
                front_wheel_radius_m: radius,
                ..TrikeSpec::iter1()
            };
            let a = head_deg.to_radians();
            // Steering axis through the origin, pointing down and forward;
            // it therefore meets the ground at x = 0.
            let axis = (a.cos(), -a.sin());
            // Forward-pointing perpendicular to the axis.
            let normal = (a.sin(), a.cos());
            // Slide along the axis until the offset wheel centre sits at
            // exactly one wheel radius above the ground.
            let t = (rake * normal.1 - radius) / -axis.1;
            let centre_x = t * axis.0 + rake * normal.0;
            // The contact patch is directly below the centre, and the axis
            // met the ground at the origin.
            let constructed_trail = -centre_x;

            let derived = SteeringGeometry::of(&spec);
            assert_relative_eq!(derived.trail_m, constructed_trail, epsilon = 1e-12);
            assert_relative_eq!(
                derived.mechanical_trail_m,
                derived.trail_m * a.sin(),
                epsilon = 1e-12
            );
        }
    }

    #[test]
    fn a_vertical_head_tube_has_trail_equal_to_minus_the_rake() {
        // Sharp limit: with the steering axis vertical the contact patch
        // sits exactly `rake` AHEAD of it, so trail is negative and the
        // front end diverges instead of self-centring.
        let spec = TrikeSpec {
            head_angle_deg: 90.0,
            fork_rake_m: 0.040,
            ..TrikeSpec::iter1()
        };
        let geo = SteeringGeometry::of(&spec);
        assert_relative_eq!(geo.trail_m, -0.040, epsilon = 1e-12);
        assert_relative_eq!(geo.mechanical_trail_m, -0.040, epsilon = 1e-12);
        assert!(!geo.self_centres());
        assert!(SteeringGeometry::of(&TrikeSpec::iter1()).self_centres());
    }

    #[test]
    fn iter1_is_the_planned_trike() {
        // Hand-computed from the mass budget, so a spec edit is visible
        // rather than silently re-baselined.
        let spec = TrikeSpec::iter1();
        assert_relative_eq!(spec.total_mass_kg(), 104.0, epsilon = 1e-12);
        assert_relative_eq!(spec.cg_x_m(), 0.890_384_615_384_6, epsilon = 1e-9);
        assert_relative_eq!(spec.cg_z_m(), 0.3225, epsilon = 1e-12);

        let loads = StaticLoads::of(&spec);
        assert_relative_eq!(loads.total_weight_n, 1019.8916, epsilon = 1e-4);
        assert_relative_eq!(loads.rear_share, 0.712_307_692_3, epsilon = 1e-9);
        assert_relative_eq!(loads.front_n, 293.415, epsilon = 1e-3);
        assert_relative_eq!(loads.per_rear_wheel_n, 363.238, epsilon = 1e-3);

        assert_relative_eq!(rollover_threshold_g(&spec), 0.828_265, epsilon = 1e-6);

        let geo = SteeringGeometry::of(&spec);
        assert_relative_eq!(geo.trail_m, 0.059_481, epsilon = 1e-6);
        assert_relative_eq!(geo.mechanical_trail_m, 0.055_150, epsilon = 1e-6);
    }

    #[test]
    fn the_polyurethane_grip_range_straddles_the_rollover_threshold() {
        // ★★★ THE FINDING, GATED. Cast polyurethane on asphalt runs about
        // µ = 0.6–1.0. The iter1 geometry tips at 0.83 g, so the SAME
        // vehicle slides at the slippery end of that range and rolls over
        // at the grippy end. Tyre hardness is a stability decision here,
        // not a feel one.
        let corner = CorneringLoads::at(&TrikeSpec::iter1(), 0.5);
        assert!(corner.slides_before_it_tips(0.60));
        assert!(!corner.slides_before_it_tips(1.00));

        // ★ AND THE CHEAPER FIX IS NOT THE OBVIOUS ONE. Track enters
        // linearly and CG height enters as its reciprocal, so buying the
        // same margin costs +156 mm of track but only 72 mm of seat drop
        // — and a slingshot trike is already trying to sit low. A whole
        // 150 mm of extra track still MISSES, at 0.994 g.
        let wide = TrikeSpec {
            rear_track_m: 0.90,
            ..TrikeSpec::iter1()
        };
        assert!(
            !CorneringLoads::at(&wide, 0.5).slides_before_it_tips(1.00),
            "+150 mm of track was expected to fall just short at 0.994 g"
        );

        let mut lowered = TrikeSpec::iter1().masses;
        lowered[0].z_m -= 0.075;
        assert!(
            CorneringLoads::at(&with_masses(lowered), 0.5).slides_before_it_tips(1.00),
            "dropping the rider 75 mm should clear the whole grip range"
        );
    }

    #[test]
    fn a_corner_below_the_threshold_keeps_both_rear_wheels_down() {
        let spec = TrikeSpec::iter1();
        let threshold = rollover_threshold_g(&spec);
        let safe = CorneringLoads::at(&spec, threshold * 0.9);
        assert!(!safe.inner_rear_lifted());
        assert!(safe.outer_rear_n > safe.inner_rear_n);
        let past = CorneringLoads::at(&spec, threshold * 1.1);
        assert!(past.inner_rear_lifted());
        // Fore-aft balance is untouched by a pure lateral acceleration.
        assert_relative_eq!(
            safe.front_n,
            StaticLoads::of(&spec).front_n,
            epsilon = 1e-12
        );
    }

    #[test]
    fn a_margin_that_is_exactly_zero_is_not_a_margin() {
        // ⚠ BOUNDARY, AND IT IS A SAFETY ONE. A vehicle whose grip exactly
        // equals its rollover threshold is marginal, not safe, so the
        // comparison must be strict. Caught only by mutation — reading the
        // `<` tells you nothing about which side of it is intended.
        let corner = CorneringLoads::at(&TrikeSpec::iter1(), 0.5);
        let exact = corner.rollover_threshold_g;
        assert!(!corner.slides_before_it_tips(exact));
        assert!(corner.slides_before_it_tips(exact * (1.0 - 1e-12)));
    }

    #[test]
    fn zero_trail_does_not_self_centre() {
        // Trail IS the restoring lever, so at exactly zero there is none
        // and the front end is neutral. Same rule as above: neutral must
        // not report as stable. Rake of `R cos α` zeroes it exactly.
        let head = 68.0_f64;
        let radius = 0.254;
        let geo = SteeringGeometry::of(&TrikeSpec {
            head_angle_deg: head,
            front_wheel_radius_m: radius,
            fork_rake_m: radius * head.to_radians().cos(),
            ..TrikeSpec::iter1()
        });
        assert_relative_eq!(geo.trail_m, 0.0, epsilon = 1e-15);
        assert!(!geo.self_centres());
    }

    #[test]
    fn the_rear_wheel_load_case_a_spoke_design_must_survive() {
        // ★ THE HANDOFF. These are the numbers a rear wheel is sized
        // against, pinned here so that changing the trike changes them
        // visibly rather than silently.
        //
        // ⚠ Note what the cornering case does to the static one: at the
        // tipping point the inner wheel carries nothing, so the OUTER
        // wheel alone carries the whole rear axle load — exactly twice its
        // static share. A wheel sized on the static number is sized on
        // half the load it sees in a corner.
        let spec = TrikeSpec::iter1();
        let statics = StaticLoads::of(&spec);
        assert_relative_eq!(statics.per_rear_wheel_n, 363.238, epsilon = 0.01);

        // A realistic drift: hard polyurethane, µ = 0.6.
        let drift = CorneringLoads::at(&spec, 0.60);
        assert_relative_eq!(drift.outer_rear_n, 626.370, epsilon = 0.01);
        assert_relative_eq!(drift.inner_rear_n, 100.106, epsilon = 0.01);
        assert_relative_eq!(drift.outer_rear_lateral_n(0.60), 375.822, epsilon = 0.01);
        assert_relative_eq!(
            drift.outer_rear_hub_moment_n_m(&spec, 0.60),
            52.615,
            epsilon = 0.01
        );

        // At the limit the outer wheel takes the entire rear axle load.
        let limit = CorneringLoads::at(&spec, rollover_threshold_g(&spec));
        assert_relative_eq!(limit.outer_rear_n, statics.rear_total_n, epsilon = 1e-9);
        assert_relative_eq!(limit.inner_rear_n, 0.0, epsilon = 1e-9);
        assert_relative_eq!(
            limit.outer_rear_n,
            2.0 * statics.per_rear_wheel_n,
            epsilon = 1e-9
        );
    }

    #[test]
    fn the_dynamic_factor_belongs_to_the_caller() {
        let loads = StaticLoads::of(&TrikeSpec::iter1());
        assert_relative_eq!(
            loads.per_rear_wheel_dynamic_n(3.0),
            loads.per_rear_wheel_n * 3.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn the_hub_moment_is_the_lateral_force_on_the_wheel_radius() {
        let spec = TrikeSpec::iter1();
        let corner = CorneringLoads::at(&spec, 0.5);
        assert_relative_eq!(
            corner.outer_rear_hub_moment_n_m(&spec, 0.8),
            corner.outer_rear_n * 0.8 * spec.rear_wheel_radius_m,
            epsilon = 1e-9
        );
    }
}
