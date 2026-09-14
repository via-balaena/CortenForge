//! What the geometry and the mass budget imply: axle loads, cornering
//! loads, the rollover threshold, and steering trail.
//!
//! ★ **A three-wheeler is statically determinate.** Three contact points,
//! three unknown normal forces, three equilibrium equations — so every
//! number in this module is solved, not apportioned. Four-wheelers need a
//! roll-stiffness split to answer the same question, and that split is an
//! assumption. We get to skip it.
//!
//! ★★★ **Both layouts share one derivation.** Nothing here branches on
//! [`Layout`](crate::spec::Layout) except through
//! [`TrikeSpec::paired_axle_share`], because the physics does not care
//! which end is doubled — only that one axle has two contact patches and
//! the other has one on the centreline.

use crate::TrikeSpec;

/// The vehicle standing still on level ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticLoads {
    /// Total weight, newtons.
    pub total_weight_n: f64,
    /// Vertical reaction at the lone centreline wheel, newtons.
    pub single_wheel_n: f64,
    /// Vertical reaction summed over the paired axle, newtons.
    pub paired_axle_total_n: f64,
    /// Vertical reaction at one wheel of the paired axle, newtons.
    pub per_paired_wheel_n: f64,
    /// Fraction of the total weight carried by the paired axle, `0..1`.
    pub paired_axle_share: f64,
    /// Centre of gravity, metres aft of the front contact patch.
    pub cg_x_m: f64,
    /// Centre of gravity height, metres — as measured with a tape.
    pub cg_z_m: f64,
    /// The CG height the roll arithmetic sees, metres.
    ///
    /// Equal to `cg_z_m` on a rigid vehicle; taller once
    /// [`RollCompliance`](crate::spec::RollCompliance) is modelled.
    pub effective_cg_z_m: f64,
}

impl StaticLoads {
    /// Solve the standing-still load case.
    ///
    /// Moments about the lone wheel give the paired axle's reaction
    /// directly, so the **paired share is the CG's distance from the lone
    /// wheel over the wheelbase** and nothing else. ★ This is the number
    /// the wheel arc previously carried as a hand-written 70 %.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed — see
    /// [`TrikeSpec::assert_well_formed`].
    #[must_use]
    pub fn of(spec: &TrikeSpec) -> Self {
        spec.assert_well_formed();
        let total_weight_n = spec.total_weight_n();
        let paired_axle_share = spec.paired_axle_share();
        let paired_axle_total_n = total_weight_n * paired_axle_share;
        Self {
            total_weight_n,
            single_wheel_n: total_weight_n - paired_axle_total_n,
            paired_axle_total_n,
            per_paired_wheel_n: paired_axle_total_n / 2.0,
            paired_axle_share,
            cg_x_m: spec.cg_x_m(),
            cg_z_m: spec.cg_z_m(),
            effective_cg_z_m: spec.effective_cg_height_m(),
        }
    }

    /// A wheel's vertical load multiplied by a dynamic factor.
    ///
    /// ⚠ **The factor is the caller's, deliberately.** Kerbs, drops and
    /// potholes are not derivable from a static mass budget, and a
    /// hard-coded 3× buried in a library is exactly the kind of unowned
    /// number this crate exists to abolish. Pass one you can defend.
    #[must_use]
    pub fn dynamic_n(wheel_load_n: f64, factor: f64) -> f64 {
        wheel_load_n * factor
    }
}

/// The vehicle in a steady-state corner on level ground.
///
/// ⚠ **Valid only below [`CorneringLoads::rollover_threshold_g`].** Above
/// it the inner wheel of the paired axle has left the ground, the vehicle
/// is on two contacts, and these numbers describe a rigid body that no
/// longer matches reality. [`CorneringLoads::inner_wheel_lifted`] reports
/// it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorneringLoads {
    /// The lateral acceleration solved for, in units of `g`.
    pub lateral_accel_g: f64,
    /// Vertical reaction at the lone centreline wheel, newtons.
    ///
    /// ★ Unchanged from static. A pure lateral acceleration makes no
    /// moment about the lateral axis, so it cannot move load fore or aft.
    pub single_wheel_n: f64,
    /// Vertical reaction at the **outer** wheel of the paired axle.
    pub outer_wheel_n: f64,
    /// Vertical reaction at the **inner** wheel of the paired axle.
    ///
    /// Negative means the rigid-body solution would need the wheel to pull
    /// down on the road; physically it has lifted.
    pub inner_wheel_n: f64,
    /// Lateral acceleration at which the inner wheel lifts, in `g`.
    pub rollover_threshold_g: f64,
}

impl CorneringLoads {
    /// Solve the cornering load case at a given lateral acceleration.
    ///
    /// ★★★ **The whole roll moment lands on the paired axle.** The lone
    /// wheel sits on the centreline, so it has no lateral lever about the
    /// roll axis and resists none of it — the two paired wheels take all
    /// of `W · (a/g) · z_cg`, reacted as a couple across the track. That,
    /// and not a high CG alone, is why three-wheelers tip.
    ///
    /// The transfer is therefore `ΔF = W · (a/g) · z_cg / t`, using the
    /// **total** weight and not the paired axle's share of it.
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
        // ⚠ The EFFECTIVE height, not the measured one. With roll
        // modelled the leaning body adds `W · Δy` to the roll moment, and
        // that extra term is exactly what `effective_cg_height_m` folds
        // into the height — so transfer and threshold stay consistent.
        let transfer_n =
            statics.total_weight_n * lateral_accel_g * statics.effective_cg_z_m / spec.track_m;
        Self {
            lateral_accel_g,
            single_wheel_n: statics.single_wheel_n,
            outer_wheel_n: statics.per_paired_wheel_n + transfer_n,
            inner_wheel_n: statics.per_paired_wheel_n - transfer_n,
            rollover_threshold_g: rollover_threshold_g(spec),
        }
    }

    /// Whether the inner wheel of the paired axle has left the ground.
    #[must_use]
    pub fn inner_wheel_lifted(&self) -> bool {
        self.inner_wheel_n <= 0.0
    }

    /// The lateral force the outer paired wheel transmits at its contact
    /// patch at a given tyre friction coefficient, newtons.
    ///
    /// This is what loads a wheel out of its own plane — on an FDM rim,
    /// straight into the layer-adhesion axis.
    #[must_use]
    pub fn outer_wheel_lateral_n(&self, tyre_mu: f64) -> f64 {
        self.outer_wheel_n * tyre_mu
    }

    /// The moment that lateral force applies at the outer wheel's hub,
    /// newton-metres.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed.
    #[must_use]
    pub fn outer_wheel_hub_moment_n_m(&self, spec: &TrikeSpec, tyre_mu: f64) -> f64 {
        spec.assert_well_formed();
        self.outer_wheel_lateral_n(tyre_mu) * spec.paired_wheel_radius_m()
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

/// The lateral acceleration, in `g`, at which the inner wheel of the
/// paired axle lifts.
///
/// Setting that wheel's reaction to zero and solving gives a compact
/// result that holds for either layout:
///
/// ```text
/// a/g  =  paired_axle_share · track / (2 · cg_height)
/// ```
///
/// ★ **That is the familiar two-wheeled static stability factor,
/// `t / 2h`, scaled by the paired axle's share of the weight** — a trike
/// is exactly `paired_axle_share` as roll-stable as a four-wheeler of the
/// same track and CG height. Two consequences fall straight out, and the
/// second one is where the layouts part company:
///
/// - the term that dominates is **CG height**, not track, because track
///   enters linearly and CG height as its reciprocal;
/// - weight moved **towards the paired axle** raises the threshold. On a
///   [`Tadpole`](crate::spec::Layout::Tadpole) that means weight
///   **forward**; on a [`Delta`](crate::spec::Layout::Delta) it means
///   weight **rearward**. The same mass budget gives two different answers
///   depending on which end is doubled.
///
/// ⚠ Quasi-static. Kerbs, camber and abrupt steering all tip vehicles that
/// clear this number.
///
/// ⚠⚠ **With `spec.roll` set to [`None`] this is an UPPER BOUND**, not an
/// estimate. Every real spring rate raises the effective CG height and
/// lowers the answer — see [`RollCompliance`](crate::spec::RollCompliance).
///
/// # Panics
///
/// Panics if `spec` is not well-formed — see
/// [`TrikeSpec::assert_well_formed`].
#[must_use]
pub fn rollover_threshold_g(spec: &TrikeSpec) -> f64 {
    spec.assert_well_formed();
    spec.paired_axle_share() * spec.track_m / (2.0 * spec.effective_cg_height_m())
}

/// Front-end steering geometry.
///
/// ⚠ Models the **steered axis**, which is the front end in both layouts:
/// a fork on a delta, a pair of kingpins on a tadpole. The arithmetic is
/// the same; only the hardware differs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeringGeometry {
    /// Ground trail — how far behind the steering axis the contact patch
    /// drags, measured along the ground, metres.
    pub trail_m: f64,
    /// Mechanical trail — the perpendicular distance from the contact
    /// patch to the steering axis, metres. This is the lever the tyre's
    /// side force actually pulls on, so it, not ground trail, sets steering
    /// feel.
    pub mechanical_trail_m: f64,
}

impl SteeringGeometry {
    /// Derive trail from the steering axis angle, the steering offset and
    /// the front wheel radius.
    ///
    /// With the axis angle `α` measured from the horizontal,
    /// `trail = (R · cos α − offset) / sin α`, and mechanical trail is that
    /// times `sin α`, i.e. `R · cos α − offset`.
    ///
    /// ⚠ Negative trail is geometrically possible — enough offset puts the
    /// contact patch *ahead* of the steering axis — and it makes the
    /// steering diverge instead of self-centre. It is reported, not
    /// rejected, because the caller may be sweeping offset deliberately.
    ///
    /// # Panics
    ///
    /// Panics if `spec` is not well-formed — see
    /// [`TrikeSpec::assert_well_formed`].
    #[must_use]
    pub fn of(spec: &TrikeSpec) -> Self {
        spec.assert_well_formed();
        let alpha = spec.steering_axis_angle_deg.to_radians();
        let mechanical_trail_m = spec.front_wheel_radius_m * alpha.cos() - spec.steering_offset_m;
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
    use crate::spec::Layout;
    use crate::{MassItem, RollCompliance, TrikeSpec};
    use approx::assert_relative_eq;

    /// A spec that is `iter1` except for its mass budget.
    fn with_masses(masses: Vec<MassItem>) -> TrikeSpec {
        TrikeSpec {
            masses,
            ..TrikeSpec::iter1()
        }
    }

    #[test]
    fn the_paired_share_is_the_cg_position_not_a_chosen_fraction() {
        // ★ THE CLAIM THIS CRATE EXISTS FOR. The wheel arc carried a
        // hand-written 70 %. An axle share is a CG distance over a
        // wheelbase and cannot be anything else, so it must MOVE when the
        // mass budget moves.
        let spec = TrikeSpec::iter1();
        let base = StaticLoads::of(&spec);
        assert_relative_eq!(
            base.paired_axle_share,
            (spec.wheelbase_m - spec.cg_x_m()) / spec.wheelbase_m,
            epsilon = 1e-12
        );

        // Slide the rider 150 mm forward. On a tadpole that moves weight
        // TOWARDS the paired axle, so the share must rise — and by exactly
        // what the CG moved, not a damped fraction of it.
        let mut fwd = spec.masses.clone();
        fwd[0].x_m -= 0.15;
        let moved = StaticLoads::of(&with_masses(fwd));
        let expected_delta = 0.15 * 80.0 / spec.total_mass_kg() / spec.wheelbase_m;
        assert_relative_eq!(
            moved.paired_axle_share - base.paired_axle_share,
            expected_delta,
            epsilon = 1e-12
        );
    }

    #[test]
    fn the_axle_reactions_sum_to_the_weight() {
        // Conservation. Nothing may be created or lost between the CG and
        // the three contact patches — in either layout.
        for layout in [Layout::Tadpole, Layout::Delta] {
            for shift in [-0.30, -0.10, 0.0, 0.10, 0.25] {
                let mut masses = TrikeSpec::iter1().masses;
                for m in &mut masses {
                    m.x_m += shift;
                }
                let loads = StaticLoads::of(&TrikeSpec {
                    layout,
                    masses,
                    ..TrikeSpec::iter1()
                });
                assert_relative_eq!(
                    loads.single_wheel_n + loads.paired_axle_total_n,
                    loads.total_weight_n,
                    epsilon = 1e-9
                );
                assert_relative_eq!(
                    loads.per_paired_wheel_n * 2.0,
                    loads.paired_axle_total_n,
                    epsilon = 1e-12
                );
            }
        }
    }

    #[test]
    fn a_cg_over_the_paired_axle_puts_every_newton_on_it() {
        // Limit case, approached rather than reached: `assert_well_formed`
        // rejects a CG exactly on an axle. At 99 % of the way there, the
        // lone wheel must carry exactly the remaining 1 %.
        let spec = TrikeSpec::iter1();
        let x = 0.01 * spec.wheelbase_m; // 99 % of the way to the front pair
        let loads = StaticLoads::of(&with_masses(vec![MassItem::new("all", 100.0, x, 0.3)]));
        assert_relative_eq!(loads.paired_axle_share, 0.99, epsilon = 1e-12);
        assert_relative_eq!(
            loads.single_wheel_n,
            0.01 * loads.total_weight_n,
            epsilon = 1e-9
        );
    }

    #[test]
    fn the_lone_wheel_resists_no_roll_moment() {
        // ★★★ THE NON-OBVIOUS ONE. The lone wheel sits on the centreline,
        // so it has no lever about the roll axis: the paired axle takes
        // the WHOLE roll moment, sized by the TOTAL weight and not by that
        // axle's share of it.
        //
        // Two vehicles, same total mass, same track, same CG height,
        // wildly different fore-aft balance. If the transfer used the
        // paired share, these would differ by more than half.
        let level = 0.30;
        let a = with_masses(vec![
            MassItem::new("fore", 50.0, 0.10, level),
            MassItem::new("aft", 50.0, 0.35, level),
        ]);
        let b = with_masses(vec![
            MassItem::new("fore", 50.0, 0.65, level),
            MassItem::new("aft", 50.0, 1.05, level),
        ]);
        assert_relative_eq!(a.cg_z_m(), b.cg_z_m(), epsilon = 1e-12);
        assert_relative_eq!(a.total_mass_kg(), b.total_mass_kg(), epsilon = 1e-12);
        assert!(
            (a.paired_axle_share() - b.paired_axle_share()).abs() > 0.3,
            "the two fixtures must actually differ in balance"
        );

        let transfer = |spec: &TrikeSpec| {
            let c = CorneringLoads::at(spec, 0.5);
            (c.outer_wheel_n - c.inner_wheel_n) / 2.0
        };
        assert_relative_eq!(transfer(&a), transfer(&b), epsilon = 1e-9);

        // And it is the total weight, not the paired share, that sets it.
        let expected = 100.0 * crate::GRAVITY_M_S2 * 0.5 * level / a.track_m;
        assert_relative_eq!(transfer(&a), expected, epsilon = 1e-9);
    }

    #[test]
    fn a_tadpole_and_a_delta_are_mirror_images() {
        // ★★★ THE LAYOUT INVARIANT. Reflecting every mass about the
        // wheelbase midpoint and swapping which end is doubled must leave
        // the stability arithmetic bit-for-bit unchanged. Nothing in the
        // physics knows which way the vehicle points; only which axle has
        // two contact patches.
        //
        // ⚠ This is the gate that would have caught building the whole
        // crate for the wrong layout, which is exactly what happened.
        let delta = TrikeSpec {
            layout: Layout::Delta,
            ..TrikeSpec::iter1()
        };
        let mirrored = TrikeSpec {
            layout: Layout::Tadpole,
            masses: delta
                .masses
                .iter()
                .map(|m| MassItem::new(m.name.clone(), m.mass_kg, delta.wheelbase_m - m.x_m, m.z_m))
                .collect(),
            ..TrikeSpec::iter1()
        };

        assert_relative_eq!(
            delta.paired_axle_share(),
            mirrored.paired_axle_share(),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            rollover_threshold_g(&delta),
            rollover_threshold_g(&mirrored),
            epsilon = 1e-12
        );

        let (d, m) = (StaticLoads::of(&delta), StaticLoads::of(&mirrored));
        assert_relative_eq!(d.per_paired_wheel_n, m.per_paired_wheel_n, epsilon = 1e-9);
        assert_relative_eq!(d.single_wheel_n, m.single_wheel_n, epsilon = 1e-9);

        // And the two layouts really are different vehicles when NOT
        // mirrored — otherwise the invariant above would be vacuous.
        // The same budget really does give the two layouts different
        // answers — 0.98 g pointing one way, 0.69 g the other — so the
        // invariant above is not vacuously true of everything.
        assert!(rollover_threshold_g(&TrikeSpec::iter1()) > 0.98);
        assert!(rollover_threshold_g(&delta) < 0.70);
    }

    #[test]
    fn weight_towards_the_paired_axle_stabilises_either_layout() {
        // ⚠ THE SIGN FLIP, GATED IN BOTH DIRECTIONS. The tipping line runs
        // from the lone contact to the outer paired one, so a CG nearer
        // the paired axle sits further inboard of it. Forward helps a
        // reverse trike; rearward helps a delta. Writing the rule once and
        // gating one layout is how the wrong sign survives.
        let shifted = |layout: Layout, dx: f64| {
            let mut masses = TrikeSpec::iter1().masses;
            masses[0].x_m += dx;
            rollover_threshold_g(&TrikeSpec {
                layout,
                masses,
                ..TrikeSpec::iter1()
            })
        };
        for (layout, towards) in [(Layout::Tadpole, -0.15), (Layout::Delta, 0.15)] {
            let base = shifted(layout, 0.0);
            assert!(
                shifted(layout, towards) > base,
                "{layout:?}: moving towards the paired axle must raise the \
                 threshold, got {} vs {base}",
                shifted(layout, towards)
            );
            assert!(
                shifted(layout, -towards) < base,
                "{layout:?}: moving away must lower it, got {} vs {base}",
                shifted(layout, -towards)
            );
        }
    }

    #[test]
    fn the_inner_wheel_lifts_exactly_at_the_closed_form_threshold() {
        // ★ CROSS-CHECK THAT BYPASSES THE ARTIFACT. `rollover_threshold_g`
        // is a closed form. This finds the same number by bisecting the
        // *solved load case* for the acceleration at which the inner
        // reaction crosses zero — a different code path entirely.
        let mut specs = Vec::new();
        for layout in [Layout::Tadpole, Layout::Delta] {
            for masses in [
                TrikeSpec::iter1().masses,
                vec![MassItem::new("low", 90.0, 0.70, 0.18)],
                vec![MassItem::new("high", 60.0, 0.40, 0.55)],
            ] {
                specs.push(TrikeSpec {
                    layout,
                    masses,
                    ..TrikeSpec::iter1()
                });
            }
        }
        for spec in specs {
            let (mut lo, mut hi) = (0.0_f64, 10.0_f64);
            assert!(
                !CorneringLoads::at(&spec, hi)
                    .inner_wheel_n
                    .is_sign_positive(),
                "10 g must be past the threshold for any sane trike"
            );
            for _ in 0..200 {
                let mid = f64::midpoint(lo, hi);
                if CorneringLoads::at(&spec, mid).inner_wheel_n > 0.0 {
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
    fn a_cg_on_the_paired_axle_recovers_the_two_wheel_stability_factor() {
        // Round numbers on purpose: track 0.8 m at a CG height of 0.4 m is
        // a four-wheeler SSF of exactly 1.0 g. A trike with 80 % of its
        // weight on the paired axle must land on exactly 0.8 of that.
        let spec = TrikeSpec {
            layout: Layout::Tadpole,
            wheelbase_m: 1.0,
            track_m: 0.80,
            masses: vec![MassItem::new("all", 100.0, 0.20, 0.40)],
            ..TrikeSpec::iter1()
        };
        assert_relative_eq!(spec.track_m / (2.0 * spec.cg_z_m()), 1.0, epsilon = 1e-12);
        assert_relative_eq!(spec.paired_axle_share(), 0.80, epsilon = 1e-12);
        assert_relative_eq!(rollover_threshold_g(&spec), 0.80, epsilon = 1e-12);
    }

    #[test]
    fn trail_matches_an_independent_vector_construction() {
        // ★ MIRROR ORACLE. The implementation uses the closed form
        // `(R cos α − offset) / sin α`. This builds the steering axis and
        // the wheel centre as vectors, drops the contact patch onto the
        // ground and measures — never touching that expression.
        for (axis_deg, offset, radius) in [
            (82.0, 0.010, 0.2032),
            (71.0, 0.045, 0.350),
            (90.0, 0.000, 0.200),
            (55.0, 0.070, 0.300),
        ] {
            let spec = TrikeSpec {
                steering_axis_angle_deg: axis_deg,
                steering_offset_m: offset,
                front_wheel_radius_m: radius,
                ..TrikeSpec::iter1()
            };
            let a = axis_deg.to_radians();
            // Steering axis through the origin, pointing down and forward;
            // it therefore meets the ground at x = 0.
            let axis = (a.cos(), -a.sin());
            // Forward-pointing perpendicular to the axis.
            let normal = (a.sin(), a.cos());
            // Slide along the axis until the offset wheel centre sits at
            // exactly one wheel radius above the ground.
            let t = (offset * normal.1 - radius) / -axis.1;
            let centre_x = t * axis.0 + offset * normal.0;
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
    fn a_vertical_steering_axis_has_trail_equal_to_minus_the_offset() {
        // Sharp limit: with the steering axis vertical the contact patch
        // sits exactly `offset` AHEAD of it, so trail is negative and the
        // front end diverges instead of self-centring.
        let geo = SteeringGeometry::of(&TrikeSpec {
            steering_axis_angle_deg: 90.0,
            steering_offset_m: 0.040,
            ..TrikeSpec::iter1()
        });
        assert_relative_eq!(geo.trail_m, -0.040, epsilon = 1e-12);
        assert_relative_eq!(geo.mechanical_trail_m, -0.040, epsilon = 1e-12);
        assert!(!geo.self_centres());
        assert!(SteeringGeometry::of(&TrikeSpec::iter1()).self_centres());
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
        // not report as stable. An offset of `R cos α` zeroes it exactly.
        let axis = 82.0_f64;
        let radius = 0.2032;
        let geo = SteeringGeometry::of(&TrikeSpec {
            steering_axis_angle_deg: axis,
            front_wheel_radius_m: radius,
            steering_offset_m: radius * axis.to_radians().cos(),
            ..TrikeSpec::iter1()
        });
        assert_relative_eq!(geo.trail_m, 0.0, epsilon = 1e-15);
        assert!(!geo.self_centres());
    }

    #[test]
    fn iter1_is_the_planned_reverse_trike() {
        // Hand-computed from the mass budget, so a spec edit is visible
        // rather than silently re-baselined.
        let spec = TrikeSpec::iter1();
        assert_eq!(spec.layout, Layout::Tadpole);
        assert!(spec.layout.steers_on_the_paired_axle());
        assert_relative_eq!(spec.total_mass_kg(), 120.0, epsilon = 1e-12);
        assert_relative_eq!(spec.cg_x_m(), 0.515_833_333_3, epsilon = 1e-9);
        assert_relative_eq!(spec.cg_z_m(), 0.268_666_666_7, epsilon = 1e-9);

        let loads = StaticLoads::of(&spec);
        assert_relative_eq!(loads.total_weight_n, 1176.798, epsilon = 1e-3);
        assert_relative_eq!(loads.paired_axle_share, 0.587_333_333_3, epsilon = 1e-9);
        assert_relative_eq!(loads.per_paired_wheel_n, 345.586, epsilon = 1e-3);
        assert_relative_eq!(loads.single_wheel_n, 485.625, epsilon = 1e-3);

        assert_relative_eq!(rollover_threshold_g(&spec), 0.983_747, epsilon = 1e-6);

        let geo = SteeringGeometry::of(&spec);
        assert_relative_eq!(geo.trail_m, 0.018_460, epsilon = 1e-6);
        assert_relative_eq!(geo.mechanical_trail_m, 0.018_280, epsilon = 1e-6);
    }

    #[test]
    fn the_reverse_layout_is_a_hairs_breadth_from_clearing_the_grip_range() {
        // ★★★ THE FINDING, GATED. Cast polyurethane on asphalt runs about
        // µ = 0.6–1.0. The reverse trike tips at 0.984 g, so it slides
        // across almost the whole range and is marginal only at the very
        // grippy end.
        let corner = CorneringLoads::at(&TrikeSpec::iter1(), 0.5);
        assert!(corner.slides_before_it_tips(0.60));
        assert!(!corner.slides_before_it_tips(1.00));

        // ⚠ AND THE FIX IS NEARLY FREE, WHICH IS THE ARGUMENT FOR THE
        // LAYOUT. 15 mm of extra track clears the entire range. The same
        // mass budget as a delta needed 156 mm — an order of magnitude
        // more — because a delta puts the rider AWAY from its paired axle
        // while a reverse trike puts them towards it.
        let wide = TrikeSpec {
            track_m: 0.92,
            ..TrikeSpec::iter1()
        };
        assert!(CorneringLoads::at(&wide, 0.5).slides_before_it_tips(1.00));

        let as_delta = TrikeSpec {
            layout: Layout::Delta,
            track_m: 0.92,
            ..TrikeSpec::iter1()
        };
        assert!(
            !CorneringLoads::at(&as_delta, 0.5).slides_before_it_tips(1.00),
            "the same widened track must NOT rescue the delta"
        );
    }

    #[test]
    fn a_corner_below_the_threshold_keeps_both_paired_wheels_down() {
        let spec = TrikeSpec::iter1();
        let threshold = rollover_threshold_g(&spec);
        let safe = CorneringLoads::at(&spec, threshold * 0.9);
        assert!(!safe.inner_wheel_lifted());
        assert!(safe.outer_wheel_n > safe.inner_wheel_n);
        assert!(CorneringLoads::at(&spec, threshold * 1.1).inner_wheel_lifted());
        // Fore-aft balance is untouched by a pure lateral acceleration.
        assert_relative_eq!(
            safe.single_wheel_n,
            StaticLoads::of(&spec).single_wheel_n,
            epsilon = 1e-12
        );
    }

    #[test]
    fn the_wheel_load_case_a_spoke_design_must_survive() {
        // ★ THE HANDOFF. These are the numbers a wheel is sized against.
        //
        // ⚠ Note what the cornering case does: at the tipping point the
        // inner wheel carries nothing, so the OUTER wheel alone carries
        // the whole paired axle — exactly twice its static share. A wheel
        // sized on the static number is sized on half the load.
        let spec = TrikeSpec::iter1();
        let statics = StaticLoads::of(&spec);
        assert_relative_eq!(statics.per_paired_wheel_n, 345.586, epsilon = 0.01);
        // The lone REAR wheel is the polyurethane one, and it is the most
        // heavily loaded single wheel on the vehicle.
        assert!(statics.single_wheel_n > statics.per_paired_wheel_n);
        assert_relative_eq!(statics.single_wheel_n, 485.625, epsilon = 0.01);

        let drift = CorneringLoads::at(&spec, 0.60);
        assert_relative_eq!(drift.outer_wheel_n, 556.363_943, epsilon = 0.001);
        assert_relative_eq!(drift.inner_wheel_n, 134.808_749, epsilon = 0.001);
        assert_relative_eq!(
            drift.outer_wheel_lateral_n(0.60),
            333.818_366,
            epsilon = 0.001
        );

        let limit = CorneringLoads::at(&spec, rollover_threshold_g(&spec));
        assert_relative_eq!(
            limit.outer_wheel_n,
            statics.paired_axle_total_n,
            epsilon = 1e-9
        );
        assert_relative_eq!(limit.inner_wheel_n, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn a_rigid_spec_is_untouched_by_the_roll_model_existing() {
        // ★ PURE ADDITION, MEASURED. With `roll: None` the effective
        // height must BE the measured height, so every number this crate
        // produced before the roll model still holds exactly.
        let spec = TrikeSpec::iter1();
        assert_eq!(spec.roll, None);
        assert_relative_eq!(spec.effective_cg_height_m(), spec.cg_z_m(), epsilon = 0.0);
        assert_relative_eq!(rollover_threshold_g(&spec), 0.983_746_898, epsilon = 1e-9);
    }

    #[test]
    fn roll_compliance_is_exactly_a_taller_centre_of_gravity() {
        // ★★★ THE CLAIM, AS A MIRROR ORACLE. If the whole effect of
        // suspension really is the substitution `h -> h_eff`, then a
        // compliant vehicle and a RIGID one built at that taller height
        // must be indistinguishable — same threshold, same wheel loads, at
        // every acceleration. Nothing about the compliant path is reused
        // to build the rigid twin except the one number under test.
        let compliant = TrikeSpec {
            roll: Some(RollCompliance::from_wheel_rate(96.0, 0.20, 20_000.0, 0.90)),
            ..TrikeSpec::iter1()
        };
        let h_eff = compliant.effective_cg_height_m();
        assert!(h_eff > compliant.cg_z_m());

        let rigid_twin = TrikeSpec {
            masses: vec![MassItem::new(
                "lumped",
                compliant.total_mass_kg(),
                compliant.cg_x_m(),
                h_eff,
            )],
            roll: None,
            ..TrikeSpec::iter1()
        };
        assert_relative_eq!(rigid_twin.cg_z_m(), h_eff, epsilon = 1e-12);

        assert_relative_eq!(
            rollover_threshold_g(&compliant),
            rollover_threshold_g(&rigid_twin),
            epsilon = 1e-12
        );
        for accel in [0.0, 0.25, 0.60, 0.90] {
            let (a, b) = (
                CorneringLoads::at(&compliant, accel),
                CorneringLoads::at(&rigid_twin, accel),
            );
            assert_relative_eq!(a.outer_wheel_n, b.outer_wheel_n, epsilon = 1e-9);
            assert_relative_eq!(a.inner_wheel_n, b.inner_wheel_n, epsilon = 1e-9);
        }
    }

    #[test]
    fn the_sign_of_the_roll_axis_separation_does_not_matter() {
        // ⚠ A roll axis ABOVE the sprung CG makes the body lean INTO the
        // corner — but the CG is then below the axis and still swings
        // outboard by the same amount. The separation enters squared, and
        // that is not an accident of algebra.
        let at = |sep: f64| {
            rollover_threshold_g(&TrikeSpec {
                roll: Some(RollCompliance::from_wheel_rate(96.0, sep, 20_000.0, 0.90)),
                ..TrikeSpec::iter1()
            })
        };
        assert_relative_eq!(at(0.20), at(-0.20), epsilon = 1e-12);
        // And a CG exactly on the roll axis cannot roll at all, so it is
        // the rigid case.
        assert_relative_eq!(
            at(0.0),
            rollover_threshold_g(&TrikeSpec::iter1()),
            epsilon = 1e-12
        );
    }

    #[test]
    fn every_finite_roll_stiffness_lowers_the_threshold() {
        // ⚠⚠ THE REASON THE RIGID NUMBER IS A BOUND, NOT AN ESTIMATE.
        // Monotone in stiffness, and the rigid answer is the unreachable
        // limit as the springs go solid.
        let rigid = rollover_threshold_g(&TrikeSpec::iter1());
        let at = |k: f64| {
            rollover_threshold_g(&TrikeSpec {
                roll: Some(RollCompliance::from_wheel_rate(96.0, 0.20, k, 0.90)),
                ..TrikeSpec::iter1()
            })
        };
        let mut previous = 0.0;
        for wheel_rate in [5_000.0, 10_000.0, 20_000.0, 40_000.0, 1.0e9] {
            let now = at(wheel_rate);
            assert!(
                now < rigid,
                "{wheel_rate} N/m gave {now}, not below {rigid}"
            );
            assert!(
                now > previous,
                "stiffer springs must not lower the threshold"
            );
            previous = now;
        }
        assert_relative_eq!(at(1.0e12), rigid, epsilon = 1e-6);
    }

    #[test]
    fn the_roll_penalty_on_the_planned_trike() {
        // Hand-computed, and the reason this model was worth building: the
        // rigid 0.984 g needs only 15 mm of extra track to clear µ = 1.0,
        // and roll compliance eats more than that on its own.
        for (wheel_rate, expected) in [(20_000.0, 0.967_013_444), (10_000.0, 0.950_839_737)] {
            let spec = TrikeSpec {
                roll: Some(RollCompliance::from_wheel_rate(
                    96.0, 0.20, wheel_rate, 0.90,
                )),
                ..TrikeSpec::iter1()
            };
            assert_relative_eq!(rollover_threshold_g(&spec), expected, epsilon = 1e-9);
            assert!(!CorneringLoads::at(&spec, 0.5).slides_before_it_tips(1.00));
        }

        // ⚠ AND THE RIGID FIX IS NOT ENOUGH. 15 mm of track clears µ = 1.0
        // on the rigid model; on a 20 N/mm wheel rate it does not.
        let widened = TrikeSpec {
            track_m: 0.92,
            roll: Some(RollCompliance::from_wheel_rate(96.0, 0.20, 20_000.0, 0.92)),
            ..TrikeSpec::iter1()
        };
        assert!(!CorneringLoads::at(&widened, 0.5).slides_before_it_tips(1.00));
    }

    #[test]
    fn the_wheel_rate_helper_squares_the_track() {
        // `K = wheel_rate · track² / 2`. The square is the part that gets
        // dropped, so it gets a gate: doubling the track must quadruple
        // the roll stiffness.
        let narrow = RollCompliance::from_wheel_rate(96.0, 0.20, 20_000.0, 0.90);
        let wide = RollCompliance::from_wheel_rate(96.0, 0.20, 20_000.0, 1.80);
        assert_relative_eq!(narrow.roll_stiffness_n_m_per_rad, 8100.0, epsilon = 1e-9);
        assert_relative_eq!(
            wide.roll_stiffness_n_m_per_rad,
            4.0 * narrow.roll_stiffness_n_m_per_rad,
            epsilon = 1e-9
        );
    }

    #[test]
    fn the_dynamic_factor_belongs_to_the_caller() {
        let loads = StaticLoads::of(&TrikeSpec::iter1());
        assert_relative_eq!(
            StaticLoads::dynamic_n(loads.single_wheel_n, 3.0),
            loads.single_wheel_n * 3.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn the_hub_moment_is_the_lateral_force_on_the_paired_wheel_radius() {
        let spec = TrikeSpec::iter1();
        let corner = CorneringLoads::at(&spec, 0.5);
        assert_relative_eq!(
            corner.outer_wheel_hub_moment_n_m(&spec, 0.8),
            corner.outer_wheel_n * 0.8 * spec.front_wheel_radius_m,
            epsilon = 1e-9
        );
        // ⚠ On a tadpole the paired wheels are the FRONT ones, so the hub
        // moment must use the front radius, not the rear.
        assert_relative_eq!(
            spec.paired_wheel_radius_m(),
            spec.front_wheel_radius_m,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            spec.single_wheel_radius_m(),
            spec.rear_wheel_radius_m,
            epsilon = 1e-12
        );
    }
}
