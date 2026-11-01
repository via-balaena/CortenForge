//! Actuator processing.
//!
//! Converts MJCF `<actuator>` elements into Model actuator arrays: each
//! actuator's name and transmission from its element, every other value from
//! its [`ActuatorSpec`], its class's actuator default as the element wrote it
//! (`DefaultResolver::apply_to_actuator`), MuJoCo's default where no writer
//! set one.

use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, GainType, InterpolationType, MjJointType,
};
use tracing::warn;

use super::ModelBuilder;
use crate::actuator_spec::{ActuatorSpec, DEFAULT_DYNPRM, DEFAULT_GAINPRM};
use crate::error::{ElementKind, MjcfError, Result};
use crate::keywords;
use crate::types::{MjcfActuator, MjcfActuatorType};

/// MuJoCo's default `gear` (`user_init.c` `mjs_defaultActuator`).
const DEFAULT_GEAR: [f64; 6] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];

/// A Hill or Millard muscle's `gainprm` where no writer set one (CortenForge
/// extensions): [range lo, range hi, F0 (-1: computed), scale, optimal fiber
/// length, tendon slack length, vmax, pennation, tendon stiffness].
const MUSCLE_MODEL_GAINPRM: [f64; 9] = [0.75, 1.05, -1.0, 200.0, 0.10, 0.20, 10.0, 0.0, 35.0];

/// A Hill or Millard muscle's `dynprm` where no writer set one: its
/// activation and deactivation time constants.
const MUSCLE_MODEL_DYNPRM: [f64; 10] = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

impl ModelBuilder {
    pub(crate) fn process_actuator(
        &mut self,
        actuator: &MjcfActuator,
        spec: &ActuatorSpec,
    ) -> Result<usize> {
        let act_id = self.actuator_trntype.len();

        // Register actuator name for sensor wiring
        if !actuator.name.is_empty() {
            self.actuator_name_to_id
                .insert(actuator.name.clone(), act_id);
        }

        let cranklength = spec.cranklength.unwrap_or(0.0);
        let (trntype, trnid) = self.resolve_transmission(actuator, cranklength)?;
        let dyntype = spec.dyntype.unwrap_or(ActuatorDynamics::None);
        let (gaintype, biastype, gainprm, biasprm, dynprm) = parameters(spec, dyntype);
        // MuJoCo takes `inheritrange` on an actuator whose gain and bias are a
        // position servo's (`user_objects.cc:6824-6826`), whatever its element;
        // the exact test is MuJoCo's: both values come from one kp.
        #[allow(clippy::float_cmp)]
        let servo = gaintype == GainType::Fixed
            && biastype == BiasType::Affine
            && gainprm[0] == -biasprm[1];
        let fraction = spec.inheritrange.unwrap_or(0.0);
        let inherited = if servo && fraction > 0.0 {
            Some(self.inherited_range(actuator, fraction, trntype, trnid[0], dyntype)?)
        } else {
            None
        };
        let (ctrlrange, actrange) = match inherited {
            Some(Inherited::Ctrl(range)) => (Some(range), spec.actrange),
            Some(Inherited::Act(range)) => (spec.ctrlrange, Some(range)),
            None => (spec.ctrlrange, spec.actrange),
        };
        let autolimits = self.compiler.autolimits;
        let ctrllimited = super::is_limited(spec.ctrllimited, autolimits, ctrlrange);

        self.actuator_trntype.push(trntype);
        self.actuator_dyntype.push(dyntype);
        self.actuator_trnid.push(trnid);
        // MuJoCo's default gear, `1 0 0 0 0 0`, under the values written.
        self.actuator_gear.push(
            spec.gear
                .map_or(DEFAULT_GEAR, |gear| gear.over(DEFAULT_GEAR)),
        );

        // Gate ctrlrange/forcerange on ctrllimited/forcelimited (MuJoCo semantics).
        // When limited=false, range is effectively unbounded regardless of attribute value.
        self.actuator_ctrlrange.push(if ctrllimited {
            ctrlrange.unwrap_or((-1.0, 1.0))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });
        let forcelimited = super::is_limited(spec.forcelimited, autolimits, spec.forcerange);
        self.actuator_forcerange.push(if forcelimited {
            spec.forcerange
                .unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });

        // §34: Activation clamping — actlimited / actrange / actearly, the same
        // autolimits pattern as ctrllimited/forcelimited.
        let actlimited = super::is_limited(spec.actlimited, autolimits, actrange);
        self.actuator_actlimited.push(actlimited);
        self.actuator_actrange.push(if actlimited {
            actrange.unwrap_or((0.0, 0.0))
        } else {
            (0.0, 0.0) // Unused when actlimited=false
        });
        self.actuator_actearly.push(spec.actearly.unwrap_or(false));
        self.actuator_cranklength.push(cranklength);

        // Interpolation attributes: parse interp keyword → InterpolationType enum
        let interp = match &spec.interp {
            Some(s) => keywords::interp(s)
                .ok_or_else(|| MjcfError::invalid_keyword("actuator", "interp", s))?,
            None => InterpolationType::Zoh,
        };

        // MuJoCo compiler validation: delay > 0 requires nsample > 0
        let nsample = spec.nsample.unwrap_or(0);
        let delay = spec.delay.unwrap_or(0.0);
        if delay > 0.0 && nsample <= 0 {
            return Err(MjcfError::invalid_value(
                "actuator",
                "delay",
                "setting delay > 0 without a history buffer (nsample must be > 0)",
            ));
        }

        self.actuator_nsample.push(nsample);
        self.actuator_interp.push(interp);
        self.actuator_delay.push(delay);
        self.actuator_user_raw
            .push(spec.user.clone().unwrap_or_default());

        // §34 S5: Muscle default actrange [0, 1] when actlimited is not set. MuJoCo
        // leaves a <muscle> unlimited.
        // Hill/Millard (CortenForge extensions) use the same default: activation ∈ [0, 1].
        if (actuator.actuator_type == MjcfActuatorType::Muscle
            || dyntype == ActuatorDynamics::HillMuscle
            || dyntype == ActuatorDynamics::MillardMuscle)
            && spec.actlimited.is_none()
        {
            let idx = self.actuator_actlimited.len() - 1;
            self.actuator_actlimited[idx] = true;
            self.actuator_actrange[idx] = (0.0, 1.0);
        }

        self.actuator_name.push(if actuator.name.is_empty() {
            None
        } else {
            Some(actuator.name.clone())
        });

        // Compute activation state count based on dynamics type
        let act_num = match dyntype {
            ActuatorDynamics::None => 0,
            ActuatorDynamics::Filter
            | ActuatorDynamics::FilterExact
            | ActuatorDynamics::Integrator
            | ActuatorDynamics::Muscle
            | ActuatorDynamics::HillMuscle
            | ActuatorDynamics::MillardMuscle
            | ActuatorDynamics::User => 1,
        };

        self.actuator_act_adr.push(self.na);
        self.actuator_act_num.push(act_num);
        self.na += act_num;

        // HillMuscle parameter validation (CortenForge extension).
        if dyntype == ActuatorDynamics::HillMuscle {
            let opt_len = gainprm[4];
            let slack_len = gainprm[5];
            let vmax = gainprm[6];
            let penn = gainprm[7];
            let stiff = gainprm[8];
            if opt_len <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "HillMuscle: optimal_fiber_length (gainprm[4]) must be positive",
                ));
            }
            if slack_len < 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "HillMuscle: tendon_slack_length (gainprm[5]) must be non-negative",
                ));
            }
            if vmax <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "HillMuscle: max_contraction_velocity (gainprm[6]) must be positive",
                ));
            }
            if !(0.0..std::f64::consts::FRAC_PI_2).contains(&penn) {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "HillMuscle: pennation_angle (gainprm[7]) must be in [0, π/2)",
                ));
            }
            if stiff <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "HillMuscle: tendon_stiffness (gainprm[8]) must be positive",
                ));
            }
        }

        // MillardMuscle parameter validation (CortenForge extension). Rigid tendon —
        // no tendon-stiffness term, so gainprm[8] is unchecked (unlike HillMuscle).
        if dyntype == ActuatorDynamics::MillardMuscle {
            if gainprm[4] <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "MillardMuscle: optimal_fiber_length (gainprm[4]) must be positive",
                ));
            }
            if gainprm[5] < 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "MillardMuscle: tendon_slack_length (gainprm[5]) must be non-negative",
                ));
            }
            if gainprm[6] <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "MillardMuscle: max_contraction_velocity (gainprm[6]) must be positive",
                ));
            }
            if !(0.0..std::f64::consts::FRAC_PI_2).contains(&gainprm[7]) {
                return Err(MjcfError::invalid_value(
                    "general",
                    "gainprm",
                    "MillardMuscle: pennation_angle (gainprm[7]) must be in [0, π/2)",
                ));
            }
        }

        self.actuator_gaintype.push(gaintype);
        self.actuator_biastype.push(biastype);
        self.actuator_gainprm.push(gainprm);
        self.actuator_biasprm.push(biasprm);
        self.actuator_dynprm.push(dynprm);

        // Per-actuator group assignment (§41 S7b)
        self.actuator_group.push(spec.group.unwrap_or(0));

        // Lengthrange and acc0: initialized to zero, computed by compute_actuator_params()
        self.actuator_lengthrange.push((0.0, 0.0));
        self.actuator_acc0.push(0.0);

        Ok(act_id)
    }

    /// Resolve the transmission target from the actuator's joint/tendon/site/body/
    /// jointinparent/cranksite attributes.
    ///
    /// Returns `(ActuatorTransmission, [primary_id, secondary_id])`.
    /// Secondary slot is `usize::MAX` when unused.
    fn resolve_transmission(
        &self,
        actuator: &MjcfActuator,
        cranklength: f64,
    ) -> Result<(ActuatorTransmission, [usize; 2])> {
        if let Some(ref joint_name) = actuator.joint {
            let jnt_id = self
                .joint_name_to_id
                .get(joint_name)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Joint, joint_name))?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses joint transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            Ok((ActuatorTransmission::Joint, [*jnt_id, usize::MAX]))
        } else if let Some(ref tendon_name) = actuator.tendon {
            let tendon_idx = *self
                .tendon_name_to_id
                .get(tendon_name.as_str())
                .ok_or_else(|| MjcfError::undefined(ElementKind::Tendon, tendon_name))?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses tendon transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            Ok((ActuatorTransmission::Tendon, [tendon_idx, usize::MAX]))
        } else if let Some(ref site_name) = actuator.site {
            let site_id = *self
                .site_name_to_id
                .get(site_name)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Site, site_name))?;
            let refsite_id = if let Some(ref refsite_name) = actuator.refsite {
                *self
                    .site_name_to_id
                    .get(refsite_name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Site, refsite_name))?
            } else {
                usize::MAX
            };
            Ok((ActuatorTransmission::Site, [site_id, refsite_id]))
        } else if let Some(ref body_name) = actuator.body {
            let body_id = *self
                .body_name_to_id
                .get(body_name.as_str())
                .ok_or_else(|| MjcfError::undefined(ElementKind::Body, body_name))?;
            Ok((ActuatorTransmission::Body, [body_id, usize::MAX]))
        } else if let Some(ref joint_name) = actuator.jointinparent {
            let jnt_id = self
                .joint_name_to_id
                .get(joint_name)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Joint, joint_name))?;
            Ok((ActuatorTransmission::JointInParent, [*jnt_id, usize::MAX]))
        } else if let Some(ref crank_name) = actuator.cranksite {
            let slider_name = actuator.slidersite.as_ref().ok_or_else(|| {
                MjcfError::invalid_model("cranksite specified without slidersite")
            })?;
            let crank_id = *self
                .site_name_to_id
                .get(crank_name.as_str())
                .ok_or_else(|| MjcfError::undefined(ElementKind::Site, crank_name))?;
            let slider_id = *self
                .site_name_to_id
                .get(slider_name.as_str())
                .ok_or_else(|| MjcfError::undefined(ElementKind::Site, slider_name))?;
            let rod = cranklength;
            if rod <= 0.0 {
                return Err(MjcfError::invalid_value(
                    "actuator",
                    "cranklength",
                    format!("must be positive, got {rod}"),
                ));
            }
            Ok((ActuatorTransmission::SliderCrank, [crank_id, slider_id]))
        } else {
            Err(MjcfError::invalid_model(
                "actuator has no transmission target \
                 (joint, tendon, site, body, jointinparent, or cranksite)",
            ))
        }
    }

    /// A position servo's range taken from its target, as MuJoCo 3.5.0's
    /// `mjCActuator::Compile` takes it (`user_objects.cc:6824-6866`): the
    /// target's range as MuJoCo compiles it (`jnt_range`,
    /// `tendon_range_compiled`: a limited hinge's in radians, an unlimited
    /// one's as written), `inheritrange` times its
    /// half-width about its middle, as the control range of an actuator with
    /// no dynamics or filterexact dynamics and as the activation range of an
    /// integrator. The caller has checked the gain and bias and that
    /// `inheritrange` is positive.
    fn inherited_range(
        &self,
        actuator: &MjcfActuator,
        fraction: f64,
        trntype: ActuatorTransmission,
        target: usize,
        dyntype: ActuatorDynamics,
    ) -> Result<Inherited> {
        let element = actuator.actuator_type.as_str();
        let refuse = |reason: &str| MjcfError::invalid_value(element, "inheritrange", reason);
        let into: fn((f64, f64)) -> Inherited = match dyntype {
            ActuatorDynamics::None | ActuatorDynamics::FilterExact => Inherited::Ctrl,
            ActuatorDynamics::Integrator => Inherited::Act,
            _ => {
                return Err(refuse(
                    "inheritrange only available for position and intvelocity actuators",
                ));
            }
        };
        let (name, range) = match trntype {
            ActuatorTransmission::Joint => {
                if !matches!(
                    self.jnt_type[target],
                    MjJointType::Hinge | MjJointType::Slide
                ) {
                    return Err(refuse(
                        "inheritrange can only be used with hinge and slide joints",
                    ));
                }
                (&actuator.joint, self.jnt_range[target])
            }
            ActuatorTransmission::Tendon => (&actuator.tendon, self.tendon_range_compiled[target]),
            _ => {
                return Err(refuse(
                    "inheritrange can only be used with joint and tendon transmission",
                ));
            }
        };
        #[allow(clippy::float_cmp)] // MuJoCo's exact test of an empty range.
        if range.0 == range.1 {
            return Err(refuse(&format!(
                "inheritrange used but target '{}' has no range defined",
                name.as_deref().unwrap_or_default()
            )));
        }
        let mean = 0.5 * (range.1 + range.0);
        let radius = 0.5 * (range.1 - range.0) * fraction;
        Ok(into((mean - radius, mean + radius)))
    }
}

/// The range an actuator takes from its target (`inheritrange`).
#[derive(Debug, Clone, Copy)]
enum Inherited {
    /// Its control range.
    Ctrl((f64, f64)),
    /// Its activation range.
    Act((f64, f64)),
}

/// The gain, bias and dynamics types and parameters `spec` gives, each
/// MuJoCo's default where no writer set it (`mjs_defaultActuator`: a fixed
/// gain, no bias, no dynamics, `gainprm[0]` and `dynprm[0]` 1), but a Hill
/// or Millard muscle's, a `<general dyntype="hillmuscle">` or
/// `"millardmuscle"` (CortenForge extensions): its own types and defaults.
fn parameters(
    spec: &ActuatorSpec,
    dyntype: ActuatorDynamics,
) -> (GainType, BiasType, [f64; 9], [f64; 9], [f64; 10]) {
    let (gaintype, biastype, gain, dynamics) = match dyntype {
        ActuatorDynamics::HillMuscle => (
            GainType::HillMuscle,
            BiasType::HillMuscle,
            MUSCLE_MODEL_GAINPRM,
            MUSCLE_MODEL_DYNPRM,
        ),
        ActuatorDynamics::MillardMuscle => (
            GainType::MillardMuscle,
            BiasType::MillardMuscle,
            MUSCLE_MODEL_GAINPRM,
            MUSCLE_MODEL_DYNPRM,
        ),
        _ => (
            GainType::Fixed,
            BiasType::None,
            DEFAULT_GAINPRM,
            DEFAULT_DYNPRM,
        ),
    };
    let fill = |slots: &[Option<f64>], defaults: &[f64], out: &mut [f64]| {
        for ((out, slot), default) in out.iter_mut().zip(slots).zip(defaults) {
            *out = slot.unwrap_or(*default);
        }
    };
    let (mut gainprm, mut biasprm, mut dynprm) = ([0.0; 9], [0.0; 9], [0.0; 10]);
    fill(&spec.gainprm, &gain, &mut gainprm);
    fill(&spec.biasprm, &[0.0; 9], &mut biasprm);
    fill(&spec.dynprm, &dynamics, &mut dynprm);
    (
        spec.gaintype.unwrap_or(gaintype),
        spec.biastype.unwrap_or(biastype),
        gainprm,
        biasprm,
        dynprm,
    )
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use crate::error::MjcfError;
    use sim_core::{ActuatorDynamics, ActuatorTransmission, BiasType, GainType};

    /// Test that tendon actuators return proper error.
    #[test]
    fn test_tendon_actuator_unknown_tendon_error() {
        // Actuator referencing a nonexistent tendon should fail with "unknown tendon"
        let result = load_model(
            r#"
            <mujoco model="tendon_test">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="m1" tendon="some_tendon" gear="1"/>
                </actuator>
            </mujoco>
        "#,
        );

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("unknown tendon"));
    }

    /// Test site actuator transmission.
    #[test]
    fn test_site_actuator() {
        let model = load_model(
            r#"
            <mujoco model="site_actuator_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <site name="end_effector" pos="0 0 -1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general name="site_act" site="end_effector" gear="50"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Site);
        assert_eq!(model.actuator_trnid[0][0], 0); // First site
        assert_eq!(model.actuator_trnid[0][1], usize::MAX); // No refsite
        assert!((model.actuator_gear[0][0] - 50.0).abs() < 1e-10);
    }

    /// Test actuator activation states (na) computation.
    #[test]
    fn test_actuator_activation_states() {
        let model = load_model(
            r#"
            <mujoco model="activation_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <joint name="j2" type="hinge" axis="1 0 0"/>
                        <joint name="j3" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="j1" gear="10"/>
                    <position name="pos1" joint="j2" kp="100"/>
                    <velocity name="vel1" joint="j3" kv="10"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 3);

        // Motor has no dynamics -> 0 activation states
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[0], 0);
        assert_eq!(model.actuator_act_num[0], 0);

        // Position servo (no timeconst) has no dynamics -> 0 activation states
        // MuJoCo: Position defaults to dyntype=None when timeconst=0.
        assert_eq!(model.actuator_dyntype[1], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[1], 0);
        assert_eq!(model.actuator_act_num[1], 0);

        // Velocity servo always has no dynamics -> 0 activation states
        assert_eq!(model.actuator_dyntype[2], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[2], 0);
        assert_eq!(model.actuator_act_num[2], 0);

        // Total activation states
        assert_eq!(model.na, 0); // 0 + 0 + 0 = 0
    }

    // ========================================================================
    // <general> actuator MJCF attributes (spec section 8)
    // ========================================================================

    /// Helper: minimal MJCF with a single body+joint for actuator tests.
    fn general_actuator_model(actuator_xml: &str) -> String {
        format!(
            r#"
            <mujoco model="general_actuator_test">
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    {actuator_xml}
                </actuator>
            </mujoco>
            "#
        )
    }

    /// A short biasprm or dynprm on a HillMuscle or MillardMuscle (sim-mjcf
    /// extensions) keeps that type's defaults past the values written.
    #[test]
    fn test_muscle_extension_short_bias_and_dyn_params() {
        for dyntype in ["hillmuscle", "millardmuscle"] {
            let xml = general_actuator_model(&format!(
                r#"<general joint="j" dyntype="{dyntype}" biasprm="7" dynprm="0.02"/>"#
            ));
            let model = load_model(&xml).expect("should load");
            let bias = [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            for (got, want) in model.actuator_biasprm[0].iter().zip(bias) {
                assert!((got - want).abs() < 1e-10, "{dyntype} biasprm");
            }
            assert!(
                (model.actuator_dynprm[0][0] - 0.02).abs() < 1e-10,
                "{dyntype}"
            );
            assert!(
                (model.actuator_dynprm[0][1] - 0.04).abs() < 1e-10,
                "{dyntype}"
            );
        }
    }

    /// Criterion 1: Explicit gaintype/biastype on `<general>`.
    #[test]
    fn test_general_explicit_gaintype_biastype() {
        let xml = general_actuator_model(
            r#"<general joint="j" gaintype="affine" gainprm="0 0 -5"
                       biastype="none" ctrllimited="true" ctrlrange="0 1"/>"#,
        );
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        // gainprm = [0, 0, -5, 0, 0, 0, 0, 0, 0]
        assert!((model.actuator_gainprm[0][0]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][2] - (-5.0)).abs() < 1e-10);
        for i in 3..9 {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
    }

    /// Criterion 2: Explicit dyntype with dynprm.
    #[test]
    fn test_general_explicit_dyntype() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="filter" dynprm="0.05"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::Filter);
        assert!((model.actuator_dynprm[0][0] - 0.05).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
        // Filter -> 1 activation state
        assert_eq!(model.actuator_act_num[0], 1);
    }

    /// Criterion 3: Bare `<general>` backward compatibility (Motor-like defaults).
    #[test]
    fn test_general_bare_defaults() {
        let xml = general_actuator_model(r#"<general joint="j"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        // gainprm = [1, 0, ..., 0] (MuJoCo default)
        assert!((model.actuator_gainprm[0][0] - 1.0).abs() < 1e-10);
        for i in 1..9 {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
        // biasprm = [0, ..., 0]
        for i in 0..9 {
            assert!((model.actuator_biasprm[0][i]).abs() < 1e-10);
        }
        // dynprm = [1, 0, 0] (MuJoCo default)
        assert!((model.actuator_dynprm[0][0] - 1.0).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
        // No dynamics -> 0 activation states
        assert_eq!(model.actuator_act_num[0], 0);
    }

    /// Criterion 4: PD servo via `<general>` matches `<position>` shortcut.
    #[test]
    fn test_general_pd_servo_equivalence() {
        let general_xml = general_actuator_model(
            r#"<general joint="j" gaintype="fixed" gainprm="100"
                       biastype="affine" biasprm="0 -100 -10"
                       dyntype="filterexact" dynprm="0.01"/>"#,
        );
        let position_xml =
            general_actuator_model(r#"<position joint="j" kp="100" kv="10" timeconst="0.01"/>"#);

        let m_gen = load_model(&general_xml).expect("should load general");
        let m_pos = load_model(&position_xml).expect("should load position");

        assert_eq!(m_gen.actuator_gaintype[0], m_pos.actuator_gaintype[0]);
        assert_eq!(m_gen.actuator_biastype[0], m_pos.actuator_biastype[0]);
        assert_eq!(m_gen.actuator_dyntype[0], m_pos.actuator_dyntype[0]);
        for i in 0..9 {
            assert!(
                (m_gen.actuator_gainprm[0][i] - m_pos.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_gainprm[0][i],
                m_pos.actuator_gainprm[0][i]
            );
            assert!(
                (m_gen.actuator_biasprm[0][i] - m_pos.actuator_biasprm[0][i]).abs() < 1e-10,
                "biasprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_biasprm[0][i],
                m_pos.actuator_biasprm[0][i]
            );
        }
        for i in 0..3 {
            assert!(
                (m_gen.actuator_dynprm[0][i] - m_pos.actuator_dynprm[0][i]).abs() < 1e-10,
                "dynprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_dynprm[0][i],
                m_pos.actuator_dynprm[0][i]
            );
        }
        assert_eq!(m_gen.actuator_act_num[0], m_pos.actuator_act_num[0]);
    }

    /// Criterion 5: Muscle via `<general>` matches `<muscle>` shortcut.
    #[test]
    fn test_general_muscle_equivalence() {
        let general_xml = general_actuator_model(
            r#"<general joint="j" gaintype="muscle" biastype="muscle"
                       dyntype="muscle"
                       gainprm="0.75 1.05 -1 200 0.5 1.6 1.5 0.6 1.4"
                       dynprm="0.01 0.04 0"/>"#,
        );
        let muscle_xml = general_actuator_model(
            r#"<muscle joint="j" range="0.75 1.05" force="-1" scale="200"
                      lmin="0.5" lmax="1.6" vmax="1.5" fpmax="0.6" fvmax="1.4"
                      timeconst="0.01 0.04"/>"#,
        );

        let m_gen = load_model(&general_xml).expect("should load general");
        let m_mus = load_model(&muscle_xml).expect("should load muscle");

        assert_eq!(m_gen.actuator_gaintype[0], m_mus.actuator_gaintype[0]);
        assert_eq!(m_gen.actuator_biastype[0], m_mus.actuator_biastype[0]);
        assert_eq!(m_gen.actuator_dyntype[0], m_mus.actuator_dyntype[0]);
        for i in 0..9 {
            assert!(
                (m_gen.actuator_gainprm[0][i] - m_mus.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_gainprm[0][i],
                m_mus.actuator_gainprm[0][i]
            );
        }
        for i in 0..3 {
            assert!(
                (m_gen.actuator_dynprm[0][i] - m_mus.actuator_dynprm[0][i]).abs() < 1e-10,
                "dynprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_dynprm[0][i],
                m_mus.actuator_dynprm[0][i]
            );
        }
        assert_eq!(m_gen.actuator_act_num[0], m_mus.actuator_act_num[0]);
    }

    /// Criterion 6: Default class inheritance for `<general>` attributes.
    #[test]
    fn test_general_default_class_inheritance() {
        let xml = r#"
            <mujoco model="general_defaults">
                <default>
                    <general gaintype="affine" gainprm="0 0 -10"/>
                </default>
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_model(xml).expect("should load");
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        assert!((model.actuator_gainprm[0][0]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][2] - (-10.0)).abs() < 1e-10);
    }

    /// Criterion 6 (continued): Explicit attribute overrides class default.
    #[test]
    fn test_general_default_class_override() {
        let xml = r#"
            <mujoco model="general_defaults_override">
                <default>
                    <general gaintype="affine" gainprm="0 0 -10"/>
                </default>
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j" gaintype="fixed" gainprm="50"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_model(xml).expect("should load");
        // Explicit gaintype="fixed" overrides class default "affine"
        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        // Explicit gainprm="50" is written over class default "0 0 -10": the
        // values not written keep the class's (MuJoCo 3.5.0: 50 0 -10 0 ...).
        assert!((model.actuator_gainprm[0][0] - 50.0).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][2] - (-10.0)).abs() < 1e-10);
        for i in [1, 3, 4, 5, 6, 7, 8] {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
    }

    /// Criterion 7: Partial gainprm (fewer than 9 elements).
    #[test]
    fn test_general_partial_gainprm() {
        let xml = general_actuator_model(r#"<general joint="j" gainprm="50"/>"#);
        let model = load_model(&xml).expect("should load");

        assert!((model.actuator_gainprm[0][0] - 50.0).abs() < 1e-10);
        for i in 1..9 {
            assert!(
                (model.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] should be 0, got {}",
                model.actuator_gainprm[0][i]
            );
        }
    }

    /// Criterion 8: dynprm defaults to [1,0,0] when dyntype is set but dynprm absent.
    #[test]
    fn test_general_dynprm_default() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="filter"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::Filter);
        assert!((model.actuator_dynprm[0][0] - 1.0).abs() < 1e-10); // tau = 1.0s
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
    }

    /// Criterion 9: Invalid enum produces an invalid-keyword error.
    #[test]
    fn test_general_invalid_gaintype_error() {
        let xml = general_actuator_model(r#"<general joint="j" gaintype="invalid"/>"#);
        let err = load_model(&xml).unwrap_err();
        assert!(
            matches!(
                &err,
                MjcfError::InvalidKeyword { attribute, value, .. }
                    if attribute == "gaintype" && value == "invalid"
            ),
            "{err}"
        );
    }

    /// Criterion 10: the "user" types are MuJoCo's callback types
    /// (mjGAIN_USER, mjBIAS_USER, mjDYN_USER), which MuJoCo 3.5.0 loads.
    #[test]
    fn test_general_user_types_are_the_callback_types() {
        let xml = general_actuator_model(
            r#"<general joint="j" gaintype="user" biastype="user" dyntype="user"/>"#,
        );
        let model = load_model(&xml).expect("user types load");
        assert_eq!(model.actuator_gaintype[0], GainType::User);
        assert_eq!(model.actuator_biastype[0], BiasType::User);
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::User);
        assert_eq!(model.actuator_act_num[0], 1);
    }

    /// Criterion 11: Shortcut types ignore gaintype/biastype/dyntype attributes.
    #[test]
    fn test_shortcut_types_ignore_general_attrs() {
        // A <motor> with gaintype/biastype should still produce Motor-like behavior.
        // These attributes are not parsed for shortcut types (gated by actuator_type == General).
        let xml = general_actuator_model(r#"<motor joint="j" gear="1"/>"#);
        let model = load_model(&xml).expect("should load motor");

        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        assert!((model.actuator_gainprm[0][0] - 1.0).abs() < 1e-10);
    }

    /// Criterion 12: All dyntype values wire correctly.
    #[test]
    fn test_general_all_dyntype_values() {
        let cases = [
            ("none", ActuatorDynamics::None, 0),
            ("integrator", ActuatorDynamics::Integrator, 1),
            ("filter", ActuatorDynamics::Filter, 1),
            ("filterexact", ActuatorDynamics::FilterExact, 1),
            ("muscle", ActuatorDynamics::Muscle, 1),
            ("hillmuscle", ActuatorDynamics::HillMuscle, 1),
            ("millardmuscle", ActuatorDynamics::MillardMuscle, 1),
            ("user", ActuatorDynamics::User, 1),
        ];
        for (dyntype_str, expected_dyn, expected_act_num) in &cases {
            let xml =
                general_actuator_model(&format!(r#"<general joint="j" dyntype="{dyntype_str}"/>"#));
            let model = load_model(&xml).expect("should load dyntype");

            assert_eq!(
                model.actuator_dyntype[0], *expected_dyn,
                "dyntype={dyntype_str}: wrong ActuatorDynamics"
            );
            assert_eq!(
                model.actuator_act_num[0], *expected_act_num,
                "dyntype={dyntype_str}: wrong act_num"
            );
        }
    }

    /// Criterion 13: a nonzero tenth gainprm element is refused, not
    /// truncated.
    #[test]
    fn test_general_tenth_gainprm_is_a_stated_limitation() {
        // MuJoCo 3.5.0 stores ten gain parameters, sim-core nine: a nonzero
        // tenth is refused (D-GAINPRM-10).
        let xml = general_actuator_model(r#"<general joint="j" gainprm="1 2 3 4 5 6 7 8 9 10"/>"#);
        let err = load_model(&xml).unwrap_err();
        assert!(matches!(err, MjcfError::Unsupported { .. }), "{err}");
    }

    // ── HillMuscle builder tests (Spec C: T10, T11, T14) ──

    /// T10: MJCF parsing round-trip for dyntype="hillmuscle" (AC10).
    #[test]
    fn test_hillmuscle_parsing_roundtrip() {
        let xml = general_actuator_model(
            r#"<general name="hill1" joint="j" dyntype="hillmuscle"
                       gainprm="0.75 1.05 -1 200 0.10 0.20 10.0 0.349066 35.0"/>"#,
        );
        let model = load_model(&xml).expect("should parse HillMuscle");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::HillMuscle);
        assert_eq!(model.actuator_gaintype[0], GainType::HillMuscle);
        assert_eq!(model.actuator_biastype[0], BiasType::HillMuscle);
        assert_eq!(model.actuator_act_num[0], 1);
        assert!(model.actuator_actlimited[0]);
        assert_eq!(model.actuator_actrange[0], (0.0, 1.0));

        // gainprm values
        assert!((model.actuator_gainprm[0][4] - 0.10).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][7] - 0.349_066).abs() < 1e-10);
    }

    /// T10 continued: HillMuscle auto-set defaults (no explicit gaintype/biastype).
    #[test]
    fn test_hillmuscle_auto_defaults() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="hillmuscle"/>"#);
        let model = load_model(&xml).expect("should parse HillMuscle with defaults");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::HillMuscle);
        assert_eq!(model.actuator_gaintype[0], GainType::HillMuscle);
        assert_eq!(model.actuator_biastype[0], BiasType::HillMuscle);

        // Default gainprm values
        assert!((model.actuator_gainprm[0][0] - 0.75).abs() < 1e-10); // range_lo
        assert!((model.actuator_gainprm[0][4] - 0.10).abs() < 1e-10); // opt_len
        assert!((model.actuator_gainprm[0][5] - 0.20).abs() < 1e-10); // slack_len

        // Default dynprm: [0.01, 0.04, 0, ...]
        assert!((model.actuator_dynprm[0][0] - 0.01).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][1] - 0.04).abs() < 1e-10);
    }

    /// T11: Builder validation rejects invalid parameters (AC11).
    #[test]
    fn test_hillmuscle_validation_zero_optlen() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="hillmuscle"
                       gainprm="0.75 1.05 -1 200 0.0 0.20 10.0 0.0 35.0"/>"#,
        );
        let result = load_model(&xml);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("optimal_fiber_length"),
            "error should mention optimal_fiber_length: {err}"
        );
    }

    #[test]
    fn test_hillmuscle_validation_bad_pennation() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="hillmuscle"
                       gainprm="0.75 1.05 -1 200 0.10 0.20 10.0 1.6 35.0"/>"#,
        );
        let result = load_model(&xml);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("pennation_angle"),
            "error should mention pennation_angle: {err}"
        );
    }

    #[test]
    fn test_hillmuscle_validation_negative_vmax() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="hillmuscle"
                       gainprm="0.75 1.05 -1 200 0.10 0.20 -1.0 0.0 35.0"/>"#,
        );
        let result = load_model(&xml);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("max_contraction_velocity"),
            "error should mention max_contraction_velocity: {err}"
        );
    }

    /// T14: Mixed gain/bias type override (AC16).
    #[test]
    fn test_hillmuscle_gaintype_override() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="hillmuscle" gaintype="affine"
                       gainprm="100 0 0" dynprm="0.01 0.04 0.0"/>"#,
        );
        let model = load_model(&xml).expect("should parse with override");

        // dyntype is HillMuscle
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::HillMuscle);
        // gaintype is user-overridden to Affine
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        // biastype is auto-set to HillMuscle (not overridden)
        assert_eq!(model.actuator_biastype[0], BiasType::HillMuscle);
    }

    // ── MillardMuscle (CortenForge extension), mirroring the HillMuscle suite ──

    #[test]
    fn test_millardmuscle_parsing_roundtrip() {
        let xml = general_actuator_model(
            r#"<general name="mil1" joint="j" dyntype="millardmuscle"
                       gainprm="0 0 819 0 0.0564 0.0822 10.0 0.139626 0"/>"#,
        );
        let model = load_model(&xml).expect("should parse MillardMuscle");
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::MillardMuscle);
        assert_eq!(model.actuator_gaintype[0], GainType::MillardMuscle);
        assert_eq!(model.actuator_biastype[0], BiasType::MillardMuscle);
        assert_eq!(model.actuator_act_num[0], 1);
        assert!(model.actuator_actlimited[0]);
        assert_eq!(model.actuator_actrange[0], (0.0, 1.0));
        assert!((model.actuator_gainprm[0][2] - 819.0).abs() < 1e-10); // F0
        assert!((model.actuator_gainprm[0][4] - 0.0564).abs() < 1e-10); // L0
        assert!((model.actuator_gainprm[0][7] - 0.139_626).abs() < 1e-10); // penn
    }

    #[test]
    fn test_millardmuscle_auto_defaults() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="millardmuscle"/>"#);
        let model = load_model(&xml).expect("should parse MillardMuscle with defaults");
        assert_eq!(model.actuator_gaintype[0], GainType::MillardMuscle);
        assert_eq!(model.actuator_biastype[0], BiasType::MillardMuscle);
        assert!((model.actuator_gainprm[0][4] - 0.10).abs() < 1e-10); // opt_len
        assert!((model.actuator_gainprm[0][6] - 10.0).abs() < 1e-10); // vmax
        assert!((model.actuator_dynprm[0][0] - 0.01).abs() < 1e-10); // tau_act
        assert!((model.actuator_dynprm[0][1] - 0.04).abs() < 1e-10); // tau_deact
    }

    #[test]
    fn test_millardmuscle_validation_zero_optlen() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="millardmuscle"
                       gainprm="0 0 800 0 0.0 0.20 10.0 0.0 0"/>"#,
        );
        let err = load_model(&xml).unwrap_err().to_string();
        assert!(err.contains("optimal_fiber_length"), "got: {err}");
    }

    #[test]
    fn test_millardmuscle_validation_negative_slack() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="millardmuscle"
                       gainprm="0 0 800 0 0.10 -0.01 10.0 0.0 0"/>"#,
        );
        let err = load_model(&xml).unwrap_err().to_string();
        assert!(err.contains("tendon_slack_length"), "got: {err}");
    }

    #[test]
    fn test_millardmuscle_validation_negative_vmax() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="millardmuscle"
                       gainprm="0 0 800 0 0.10 0.20 -1.0 0.0 0"/>"#,
        );
        let err = load_model(&xml).unwrap_err().to_string();
        assert!(err.contains("max_contraction_velocity"), "got: {err}");
    }

    #[test]
    fn test_millardmuscle_validation_bad_pennation() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="millardmuscle"
                       gainprm="0 0 800 0 0.10 0.20 10.0 1.6 0"/>"#,
        );
        let err = load_model(&xml).unwrap_err().to_string();
        assert!(err.contains("pennation_angle"), "got: {err}");
    }

    #[test]
    fn test_millardmuscle_gaintype_override() {
        let xml = general_actuator_model(
            r#"<general joint="j" dyntype="millardmuscle" gaintype="affine"
                       gainprm="100 0 0" dynprm="0.01 0.04 0.0"/>"#,
        );
        let model = load_model(&xml).expect("should parse with override");
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::MillardMuscle);
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        // biastype is auto-set to MillardMuscle (not overridden).
        assert_eq!(model.actuator_biastype[0], BiasType::MillardMuscle);
    }

    /// An `<intvelocity>`'s compiled fields, as MuJoCo 3.5.0's (measured on
    /// the oracle): integrator dynamics, position's gain and bias, the
    /// actuator's default `dynprm`, an activation range always applied.
    // MuJoCo's values as the oracle prints them, compared exactly.
    #[allow(clippy::float_cmp, clippy::unreadable_literal)]
    #[test]
    fn an_intvelocity_is_mujoco_3_5_0s() {
        let model = load_model(
            r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator><intvelocity joint="j" kp="10" actrange="-1 1"/><intvelocity joint="j" kp="10" kv="2" actrange="-1 1"/>
<intvelocity joint="j" kp="10" dampratio="0.5" actrange="-1 1"/></actuator></mujoco>"#,
        )
        .expect("loads");
        assert_eq!(model.nu, 3);
        assert_eq!(model.na, 3);
        for i in 0..3 {
            assert_eq!(model.actuator_dyntype[i], ActuatorDynamics::Integrator);
            assert_eq!(model.actuator_gaintype[i], GainType::Fixed);
            assert_eq!(model.actuator_biastype[i], BiasType::Affine);
            assert_eq!(model.actuator_gainprm[i][0], 10.0);
            assert_eq!(model.actuator_biasprm[i][1], -10.0);
            assert_eq!(model.actuator_dynprm[i][0], 1.0);
            assert!(model.actuator_actlimited[i]);
            assert_eq!(model.actuator_actrange[i], (-1.0, 1.0));
            assert_eq!(model.actuator_act_num[i], 1);
        }
        // MuJoCo's +0: it sets biasprm[2] only from a written kv or dampratio.
        assert_eq!(model.actuator_biasprm[0][2].to_bits(), 0.0f64.to_bits());
        assert_eq!(model.actuator_biasprm[1][2], -2.0);
        assert_eq!(
            model.actuator_biasprm[2][2].to_bits(),
            (-0.4093306831785955f64).to_bits()
        );
        // A class's kp, as MuJoCo's (7).
        let classed = load_model(
            r#"<mujoco><default><default class="c"><intvelocity kp="7"/></default></default>
<worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator><intvelocity joint="j" class="c" actrange="-1 1"/></actuator></mujoco>"#,
        )
        .expect("loads");
        assert_eq!(classed.actuator_gainprm[0][0], 7.0);
        assert_eq!(classed.actuator_biasprm[0][1], -7.0);
        // With no activation range MuJoCo refuses it ("invalid actrange for
        // actuator"); sim-mjcf refuses it when the `Data` is made
        // (D-MODEL-CHECKS).
        let no_range = load_model(
            r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator><intvelocity joint="j" kp="10"/></actuator></mujoco>"#,
        )
        .expect("loads");
        assert!(no_range.try_make_data().is_err());
    }

    /// A servo's own `kv`, `dampratio` and `timeconst` refused as MuJoCo
    /// 3.5.0's `mjs_setToPosition` refuses them, on a position, on an
    /// intvelocity (where MuJoCo drops the refusal and builds the servo with
    /// no bias: D-INTVELOCITY-SERVO) and on a default class's element.
    #[test]
    fn servo_gains_refused_as_mujoco_3_5_0() {
        let refusal = |act: &str| {
            let xml = format!(
                r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator>{act}</actuator></mujoco>"#
            );
            load_model(&xml).expect_err(act).to_string()
        };
        for kind in ["position", "intvelocity"] {
            let range = if kind == "intvelocity" {
                r#" actrange="-1 1""#
            } else {
                ""
            };
            let err = refusal(&format!(
                r#"<{kind} joint="j" kv="2" dampratio="0.5"{range}/>"#
            ));
            assert!(
                err.contains("kv and dampratio cannot both be defined"),
                "{kind}: {err}"
            );
            let err = refusal(&format!(r#"<{kind} joint="j" kv="-2"{range}/>"#));
            assert!(err.contains("kv cannot be negative"), "{kind}: {err}");
            let err = refusal(&format!(r#"<{kind} joint="j" dampratio="-0.5"{range}/>"#));
            assert!(
                err.contains("dampratio cannot be negative"),
                "{kind}: {err}"
            );
        }
        let err = refusal(r#"<position joint="j" timeconst="-0.1"/>"#);
        assert!(err.contains("timeconst cannot be negative"), "{err}");
        let err = load_model(
            r#"<mujoco><default><default class="c"><position kv="2" dampratio="0.5"/></default></default>
<worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator><position joint="j" class="c"/></actuator></mujoco>"#,
        )
        .expect_err("a class's kv and dampratio")
        .to_string();
        assert!(
            err.contains("kv and dampratio cannot both be defined"),
            "{err}"
        );
        // Zero is not negative: MuJoCo loads each (measured).
        for act in [
            r#"<position joint="j" kv="0"/>"#,
            r#"<position joint="j" kv="-0"/>"#,
            r#"<position joint="j" dampratio="0"/>"#,
            r#"<position joint="j" timeconst="0"/>"#,
        ] {
            let xml = format!(
                r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator>{act}</actuator></mujoco>"#
            );
            load_model(&xml).expect(act);
        }
        // An actuator built in code is held to the same checks.
        let mut mjcf = crate::parse_mjcf_str(
            r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody>
<actuator><position joint="j"/></actuator></mujoco>"#,
        )
        .expect("parses");
        mjcf.actuators[0].kv = Some(-2.0);
        let err = crate::model_from_mjcf(&mjcf, None).expect_err("code-built kv -2");
        assert!(err.to_string().contains("kv cannot be negative"), "{err}");
    }

    /// The attributes MuJoCo 3.5.0's schema does not give a servo are
    /// refused as it refuses them ("unrecognized attribute"): an
    /// intvelocity's `timeconst` and `actlimited`, and `inheritrange` on any
    /// element but a position or intvelocity, as an element or a class's.
    #[test]
    fn a_servo_attribute_mujoco_does_not_read_is_refused() {
        for (act, default) in [
            (
                r#"<intvelocity joint="j" timeconst="0.1" actrange="-1 1"/>"#,
                "",
            ),
            (
                r#"<intvelocity joint="j" actlimited="false" actrange="-1 1"/>"#,
                "",
            ),
            (r#"<general joint="j" inheritrange="1"/>"#, ""),
            (r#"<motor joint="j" inheritrange="1"/>"#, ""),
            (
                r#"<motor joint="j"/>"#,
                r#"<default><general inheritrange="1"/></default>"#,
            ),
        ] {
            let xml = format!(
                r#"<mujoco>{default}<worldbody><body><joint name="j" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody>
<actuator>{act}</actuator></mujoco>"#
            );
            let err = load_model(&xml).expect_err(act);
            assert!(
                matches!(err, MjcfError::UnknownAttribute { .. }),
                "{act} {default}: {err}"
            );
        }
    }

    /// A model whose joints and tendons a servo's `inheritrange` reads: a
    /// hinge in degrees, a slide, a hinge with no range, a hinge whose range
    /// is written but not limited, a ball (own body), a site, and a tendon
    /// with and without a range.
    fn inheritrange_model(actuator: &str, default: &str) -> String {
        format!(
            r#"<mujoco><compiler autolimits="true"/>{default}<worldbody><body>
<joint name="h" type="hinge" range="-30 60"/><joint name="s" type="slide" range="-0.2 0.6"/>
<joint name="n" type="hinge"/><joint name="u" type="hinge" range="-30 60" limited="false"/>
<geom size="0.1"/><site name="st"/></body>
<body pos="1 0 0"><joint name="b" type="ball"/><geom size="0.1"/></body></worldbody>
<tendon><fixed name="t" range="0.1 0.5"><joint joint="h" coef="1"/></fixed>
<fixed name="tn"><joint joint="h" coef="1"/></fixed></tendon>
<actuator>{actuator}</actuator></mujoco>"#
        )
    }

    /// `inheritrange` as MuJoCo 3.5.0's `mjCActuator::Compile` takes it,
    /// every range bit for bit against the oracle's.
    // The oracle's printed values, digit for digit: -0.5235987755982989 is
    // MuJoCo's -30 degrees in radians, not a written constant.
    #[allow(clippy::unreadable_literal, clippy::approx_constant)]
    #[test]
    fn inheritrange_as_mujoco_3_5_0() {
        let bits = |r: (f64, f64)| (r.0.to_bits(), r.1.to_bits());
        let ctrl = |act: &str, default: &str| {
            let model = load_model(&inheritrange_model(act, default)).expect(act);
            assert!(
                model.actuator_ctrlrange[0].0.is_finite(),
                "{act}: not limited"
            );
            bits(model.actuator_ctrlrange[0])
        };
        let hinge = bits((-0.5235987755982989, 1.0471975511965976));
        assert_eq!(
            ctrl(r#"<position joint="h" kp="3" inheritrange="1"/>"#, ""),
            hinge
        );
        assert_eq!(
            ctrl(r#"<position joint="h" kp="3" inheritrange="0.5"/>"#, ""),
            bits((-0.13089969389957473, 0.6544984694978735))
        );
        assert_eq!(
            ctrl(r#"<position joint="s" kp="3" inheritrange="1"/>"#, ""),
            bits((-0.20000000000000004, 0.6))
        );
        assert_eq!(
            ctrl(r#"<position tendon="t" kp="3" inheritrange="1"/>"#, ""),
            bits((0.09999999999999998, 0.5))
        );
        assert_eq!(
            ctrl(
                r#"<position joint="h" kp="3" timeconst="0.2" inheritrange="1"/>"#,
                ""
            ),
            hinge
        );
        // An unlimited hinge's range stays as written, in degrees: MuJoCo
        // converts a range to radians only when the joint is limited
        // (`user_objects.cc:2906-2924`; measured: ctrlrange -30 60).
        assert_eq!(
            ctrl(r#"<position joint="u" kp="3" inheritrange="1"/>"#, ""),
            bits((-30.0, 60.0))
        );
        // A written `ctrlrange` of 0 0 is no range.
        assert_eq!(
            ctrl(
                r#"<position joint="h" kp="3" ctrlrange="0 0" inheritrange="1"/>"#,
                ""
            ),
            hinge
        );
        // From a class, on a position and on a general with a position's gain
        // and bias.
        let class = r#"<default><default class="c"><position kp="3" inheritrange="1"/></default></default>"#;
        assert_eq!(ctrl(r#"<position joint="h" class="c"/>"#, class), hinge);
        assert_eq!(
            ctrl(
                r#"<general joint="h" class="c" gaintype="fixed" biastype="affine" gainprm="5" biasprm="0 -5"/>"#,
                class
            ),
            hinge
        );
        // A motor's bias is none: the class's inheritrange is not taken.
        let motor = load_model(&inheritrange_model(
            r#"<motor joint="h" class="c"/>"#,
            class,
        ))
        .expect("motor");
        assert!(motor.actuator_ctrlrange[0].0.is_infinite());
        // An actuator takes a class's inheritrange only with a position
        // servo's gain and bias: no bias, a bias not -kp, an affine gain each
        // take nothing (MuJoCo: ctrllimited 0, measured).
        for general in [
            r#"<general joint="h" class="c" gaintype="fixed" biastype="none" gainprm="5" biasprm="0 -5"/>"#,
            r#"<general joint="h" class="c" gaintype="fixed" biastype="affine" gainprm="5" biasprm="0 -4"/>"#,
            r#"<general joint="h" class="c" gaintype="affine" biastype="affine" gainprm="5" biasprm="0 -5"/>"#,
        ] {
            let model = load_model(&inheritrange_model(general, class)).expect(general);
            assert!(model.actuator_ctrlrange[0].0.is_infinite(), "{general}");
        }
        // A class's inheritrange reaches a class below it.
        let nested = r#"<default><default class="p"><position inheritrange="1"/><default class="c"><position kp="3"/></default></default></default>"#;
        assert_eq!(ctrl(r#"<position joint="h" class="c"/>"#, nested), hinge);
        // A negative inheritrange beside a ctrlrange is no conflict: the range
        // written stays (MuJoCo: ctrlrange -1 1).
        let kept = load_model(&inheritrange_model(
            r#"<position joint="h" kp="3" ctrlrange="-1 1" inheritrange="-1"/>"#,
            "",
        ))
        .expect("kept");
        assert_eq!(kept.actuator_ctrlrange[0], (-1.0, 1.0));
        // An intvelocity class's ctrlrange beside its inheritrange is no
        // refusal while no actuator uses the class: MuJoCo's
        // mjs_setToIntVelocity drops it (measured: loads).
        let unused = load_model(&inheritrange_model(
            r#"<motor joint="h"/>"#,
            r#"<default><default class="k"><intvelocity ctrlrange="-1 1" inheritrange="1"/></default></default>"#,
        ))
        .expect("an unused intvelocity class");
        assert_eq!(unused.nu, 1);
        // A negative inheritrange takes nothing.
        let negative = load_model(&inheritrange_model(
            r#"<position joint="h" kp="3" inheritrange="-1"/>"#,
            "",
        ))
        .expect("negative");
        assert!(negative.actuator_ctrlrange[0].0.is_infinite());
        // An intvelocity's activation range.
        let act = |a: &str| {
            let model = load_model(&inheritrange_model(a, "")).expect(a);
            assert!(model.actuator_actlimited[0]);
            bits(model.actuator_actrange[0])
        };
        assert_eq!(
            act(r#"<intvelocity joint="h" kp="3" inheritrange="1"/>"#),
            hinge
        );
        assert_eq!(
            act(r#"<intvelocity tendon="t" kp="3" inheritrange="0.5"/>"#),
            bits((0.19999999999999998, 0.4))
        );
    }

    /// `inheritrange`'s refusals, with MuJoCo 3.5.0's messages (measured), but
    /// where a case's comment says MuJoCo refuses it otherwise.
    #[test]
    fn inheritrange_refusals_as_mujoco_3_5_0() {
        let class = r#"<default><default class="c"><position kp="3" inheritrange="1"/></default></default>"#;
        for (act, default, message) in [
            (
                r#"<position joint="b" kp="3" inheritrange="1"/>"#,
                "",
                "inheritrange can only be used with hinge and slide joints",
            ),
            (
                r#"<position site="st" kp="3" inheritrange="1"/>"#,
                "",
                "inheritrange can only be used with joint and tendon transmission",
            ),
            (
                r#"<position jointinparent="h" kp="3" inheritrange="1"/>"#,
                "",
                "inheritrange can only be used with joint and tendon transmission",
            ),
            (
                r#"<position joint="n" kp="3" inheritrange="1"/>"#,
                "",
                "inheritrange used but target 'n' has no range defined",
            ),
            (
                r#"<position tendon="tn" kp="3" inheritrange="1"/>"#,
                "",
                "inheritrange used but target 'tn' has no range defined",
            ),
            (
                r#"<position joint="h" kp="3" ctrlrange="-1 1" inheritrange="1"/>"#,
                "",
                "ctrlrange and inheritrange cannot both be defined",
            ),
            (
                r#"<intvelocity joint="h" kp="3" actrange="-1 1" inheritrange="1"/>"#,
                "",
                "actrange and inheritrange cannot both be defined",
            ),
            (
                r#"<intvelocity joint="h" kp="3" ctrlrange="-1 1" actrange="-1 1" inheritrange="1"/>"#,
                "",
                "actrange and inheritrange cannot both be defined",
            ),
            // MuJoCo refuses it as "invalid actrange for actuator".
            (
                r#"<intvelocity joint="h" kp="3" ctrlrange="-1 1" inheritrange="1"/>"#,
                "",
                "ctrlrange and inheritrange cannot both be defined",
            ),
            (
                r#"<general joint="h" class="c" dyntype="filter" gaintype="fixed" biastype="affine" gainprm="5" biasprm="0 -5"/>"#,
                class,
                "inheritrange only available for position and intvelocity actuators",
            ),
            // MuJoCo loads this one, the class's affine bias making it a
            // servo that takes the range (measured; D-INTVELOCITY-SERVO).
            (
                r#"<intvelocity class="c2" joint="h" kp="3" ctrlrange="-1 1" inheritrange="1"/>"#,
                r#"<default><default class="c2"><general biastype="affine"/></default></default>"#,
                "ctrlrange and inheritrange cannot both be defined",
            ),
            // A class's own range beside its own inheritrange is refused when
            // the class is read, used or not (MuJoCo, measured).
            (
                r#"<motor joint="h"/>"#,
                r#"<default><default class="k"><position ctrlrange="-1 1" inheritrange="1"/></default></default>"#,
                "ctrlrange and inheritrange cannot both be defined",
            ),
            (
                r#"<motor joint="h"/>"#,
                r#"<default><default class="k"><intvelocity actrange="-1 1" inheritrange="1"/></default></default>"#,
                "actrange and inheritrange cannot both be defined",
            ),
        ] {
            let err = load_model(&inheritrange_model(act, default))
                .expect_err(act)
                .to_string();
            assert!(err.contains(message), "{act}: {err}");
        }
    }
}
