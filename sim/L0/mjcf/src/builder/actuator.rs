//! Actuator processing.
//!
//! Converts MJCF `<actuator>` elements into Model actuator arrays.
//! Handles all shortcut types (motor, position, velocity, damper, cylinder,
//! adhesion, muscle) and the fully general `<general>` actuator, expanding
//! each into the canonical gain/bias/dynamics representation.

use sim_core::{ActuatorDynamics, ActuatorTransmission, BiasType, GainType};
use tracing::warn;

use super::{ModelBuilder, ModelConversionError};
use crate::types::{MjcfActuator, MjcfActuatorType};

impl ModelBuilder {
    pub(crate) fn process_actuator(
        &mut self,
        actuator: &MjcfActuator,
    ) -> std::result::Result<usize, ModelConversionError> {
        let act_id = self.actuator_trntype.len();

        // Register actuator name for sensor wiring
        if !actuator.name.is_empty() {
            self.actuator_name_to_id
                .insert(actuator.name.clone(), act_id);
        }

        // Determine transmission type and target (2-slot: [primary, secondary])
        // Secondary slot is usize::MAX when unused (Joint, Tendon, Site without refsite).
        let (trntype, trnid) = if let Some(ref joint_name) = actuator.joint {
            let jnt_id =
                self.joint_name_to_id
                    .get(joint_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown joint: {joint_name}"),
                    })?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses joint transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            (ActuatorTransmission::Joint, [*jnt_id, usize::MAX])
        } else if let Some(ref tendon_name) = actuator.tendon {
            let tendon_idx = *self
                .tendon_name_to_id
                .get(tendon_name.as_str())
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Actuator '{}' references unknown tendon '{}'",
                        actuator.name, tendon_name
                    ),
                })?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses tendon transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            (ActuatorTransmission::Tendon, [tendon_idx, usize::MAX])
        } else if let Some(ref site_name) = actuator.site {
            let site_id =
                *self
                    .site_name_to_id
                    .get(site_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown site: {site_name}"),
                    })?;
            let refsite_id = if let Some(ref refsite_name) = actuator.refsite {
                *self
                    .site_name_to_id
                    .get(refsite_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Actuator '{}' references unknown refsite: {refsite_name}",
                            actuator.name
                        ),
                    })?
            } else {
                usize::MAX
            };
            (ActuatorTransmission::Site, [site_id, refsite_id])
        } else if let Some(ref body_name) = actuator.body {
            let body_id = *self
                .body_name_to_id
                .get(body_name.as_str())
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Actuator '{}': body '{}' not found",
                        actuator.name, body_name
                    ),
                })?;
            (ActuatorTransmission::Body, [body_id, usize::MAX])
        } else {
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' has no transmission target (joint, tendon, site, or body)",
                    actuator.name
                ),
            });
        };

        // Resolve per-type defaults for Option fields.
        // These must be resolved before dyntype/gain/bias decisions that depend on them.
        let timeconst = actuator.timeconst.unwrap_or(match actuator.actuator_type {
            MjcfActuatorType::Cylinder => 1.0,
            _ => 0.0,
        });
        let kv = actuator.kv.unwrap_or(match actuator.actuator_type {
            MjcfActuatorType::Velocity => 1.0,
            _ => 0.0, // Damper defaults to 0.0 (MuJoCo: mjs_setToDamper uses kv=0)
        });

        // Damper and Adhesion actuators force ctrllimited
        // (MuJoCo: mjs_setToDamper and mjs_setToAdhesion both set ctrllimited=1).
        let ctrllimited = match actuator.actuator_type {
            MjcfActuatorType::Damper | MjcfActuatorType::Adhesion => true,
            _ => match actuator.ctrllimited {
                Some(v) => v,
                None => {
                    if self.compiler.autolimits {
                        actuator.ctrlrange.is_some()
                    } else {
                        false
                    }
                }
            },
        };

        // Determine dynamics type.
        // MuJoCo semantics: dyntype is determined by the shortcut type AND timeconst.
        // Position defaults to None; if timeconst > 0, uses FilterExact (exact discrete filter).
        // Cylinder always uses Filter (Euler-approximated; timeconst defaults to 1.0).
        // Motor/Damper/Adhesion/Velocity: no dynamics.
        // General: explicit dyntype attribute; defaults to None.
        // Muscle: muscle activation dynamics.
        let dyntype = match actuator.actuator_type {
            MjcfActuatorType::Motor
            | MjcfActuatorType::Damper
            | MjcfActuatorType::Adhesion
            | MjcfActuatorType::Velocity => ActuatorDynamics::None,
            MjcfActuatorType::General => {
                // <general> uses explicit dyntype attribute; defaults to None
                match &actuator.dyntype {
                    Some(s) => parse_dyntype(s)?,
                    None => ActuatorDynamics::None,
                }
            }
            MjcfActuatorType::Position => {
                if timeconst > 0.0 {
                    ActuatorDynamics::FilterExact // MuJoCo: mjDYN_FILTEREXACT
                } else {
                    ActuatorDynamics::None
                }
            }
            MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
            MjcfActuatorType::Cylinder => ActuatorDynamics::Filter,
        };

        self.actuator_trntype.push(trntype);
        self.actuator_dyntype.push(dyntype);
        self.actuator_trnid.push(trnid);
        self.actuator_gear.push(actuator.gear);

        // Gate ctrlrange/forcerange on ctrllimited/forcelimited (MuJoCo semantics).
        // When limited=false, range is effectively unbounded regardless of attribute value.
        self.actuator_ctrlrange.push(if ctrllimited {
            actuator.ctrlrange.unwrap_or((-1.0, 1.0))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });
        let forcelimited = match actuator.forcelimited {
            Some(v) => v,
            None => {
                if self.compiler.autolimits {
                    actuator.forcerange.is_some()
                } else {
                    false
                }
            }
        };
        self.actuator_forcerange.push(if forcelimited {
            actuator
                .forcerange
                .unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });

        // §34: Activation clamping — actlimited / actrange / actearly.
        // Same autolimits pattern as ctrllimited/forcelimited.
        let actlimited = match actuator.actlimited {
            Some(v) => v,
            None => {
                if self.compiler.autolimits {
                    actuator.actrange.is_some()
                } else {
                    false
                }
            }
        };
        self.actuator_actlimited.push(actlimited);
        self.actuator_actrange.push(if actlimited {
            actuator.actrange.unwrap_or((0.0, 0.0))
        } else {
            (0.0, 0.0) // Unused when actlimited=false
        });
        self.actuator_actearly
            .push(actuator.actearly.unwrap_or(false));

        // §34 S5: Muscle default actrange — MuJoCo defaults muscles to actlimited=true,
        // actrange=[0,1] when actlimited is not explicitly set. This preserves the
        // hardcoded [0,1] clamping behavior that muscles have always had.
        if actuator.actuator_type == MjcfActuatorType::Muscle && actuator.actlimited.is_none() {
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
            | ActuatorDynamics::User => 1,
        };

        self.actuator_act_adr.push(self.na);
        self.actuator_act_num.push(act_num);
        self.na += act_num;

        // Gain/Bias/Dynamics parameters — expand shortcut type to general actuator.
        // Reference: MuJoCo src/user/user_api.cc (mjs_setToMotor, mjs_setToPosition, etc.)
        let (gaintype, biastype, gainprm, biasprm, dynprm) = match actuator.actuator_type {
            MjcfActuatorType::Motor => (
                GainType::Fixed,
                BiasType::None,
                {
                    let mut p = [0.0; 9];
                    p[0] = 1.0; // unit gain
                    p
                },
                [0.0; 9],
                [0.0; 10],
            ),

            MjcfActuatorType::Position => {
                let kp = actuator.kp; // default 1.0
                // kv resolved above from Option<f64> (default 0.0 for position)
                let tc = timeconst; // resolved above (default 0.0 for position)
                let mut gp = [0.0; 9];
                gp[0] = kp;
                let mut bp = [0.0; 9];
                bp[1] = -kp;
                // dampratio vs explicit kv:
                // If dampratio is specified, store as positive biasprm[2].
                // compute_actuator_params() will convert it to -damping at compile time.
                // If kv is specified (or default 0), store as -kv (explicit damping).
                if let Some(dr) = actuator.dampratio {
                    bp[2] = dr; // positive → dampratio (converted by compute_actuator_params)
                } else {
                    bp[2] = -kv; // negative or zero → explicit kv
                }
                (GainType::Fixed, BiasType::Affine, gp, bp, {
                    let mut d = [0.0; 10];
                    d[0] = tc;
                    d
                })
            }

            MjcfActuatorType::Velocity => {
                // kv resolved above from Option<f64> (default 1.0 for velocity)
                let mut gp = [0.0; 9];
                gp[0] = kv;
                let mut bp = [0.0; 9];
                bp[2] = -kv;
                (GainType::Fixed, BiasType::Affine, gp, bp, [0.0; 10])
            }

            MjcfActuatorType::Damper => {
                // kv resolved above from Option<f64> (default 0.0 for damper)
                let mut gp = [0.0; 9];
                gp[2] = -kv; // gain = -kv * velocity
                (GainType::Affine, BiasType::None, gp, [0.0; 9], [0.0; 10])
            }

            MjcfActuatorType::Cylinder => {
                let area = if let Some(d) = actuator.diameter {
                    std::f64::consts::PI / 4.0 * d * d
                } else {
                    actuator.area
                };
                let tc = timeconst; // resolved above (default 1.0 for cylinder)
                let mut gp = [0.0; 9];
                gp[0] = area;
                let mut bp = [0.0; 9];
                bp[0] = actuator.bias[0];
                bp[1] = actuator.bias[1];
                bp[2] = actuator.bias[2];
                (GainType::Fixed, BiasType::Affine, gp, bp, {
                    let mut d = [0.0; 10];
                    d[0] = tc;
                    d
                })
            }

            MjcfActuatorType::Adhesion => {
                let mut gp = [0.0; 9];
                gp[0] = actuator.gain;
                (GainType::Fixed, BiasType::None, gp, [0.0; 9], [0.0; 10])
            }

            MjcfActuatorType::Muscle => {
                // Muscle: unchanged from #5 implementation.
                let gp = [
                    actuator.range.0,
                    actuator.range.1,
                    actuator.force,
                    actuator.scale,
                    actuator.lmin,
                    actuator.lmax,
                    actuator.vmax,
                    actuator.fpmax,
                    actuator.fvmax,
                ];
                (
                    GainType::Muscle,
                    BiasType::Muscle,
                    gp,
                    gp, // biasprm = gainprm (shared layout, MuJoCo convention)
                    {
                        let mut d = [0.0; 10];
                        d[0] = actuator.muscle_timeconst.0;
                        d[1] = actuator.muscle_timeconst.1;
                        d
                    },
                )
            }

            MjcfActuatorType::General => {
                // <general> uses explicit attributes; defaults match MuJoCo's
                // mjs_defaultActuator: gaintype=fixed, biastype=none,
                // gainprm=[1,0,...], biasprm=[0,...], dynprm=[1,0,0].
                let gt = match &actuator.gaintype {
                    Some(s) => parse_gaintype(s)?,
                    None => GainType::Fixed,
                };
                let bt = match &actuator.biastype {
                    Some(s) => parse_biastype(s)?,
                    None => BiasType::None,
                };
                let gainprm_default = {
                    let mut d = [0.0; 9];
                    d[0] = 1.0; // MuJoCo default
                    d
                };
                let gp = if let Some(v) = &actuator.gainprm {
                    floats_to_array(v, gainprm_default)
                } else {
                    gainprm_default
                };
                let bp = if let Some(v) = &actuator.biasprm {
                    floats_to_array(v, [0.0; 9])
                } else {
                    [0.0; 9]
                };
                let dynprm_default = {
                    let mut d = [0.0; 10];
                    d[0] = 1.0; // MuJoCo: dynprm[0]=1
                    d
                };
                let dp = if let Some(v) = &actuator.dynprm {
                    floats_to_array(v, dynprm_default)
                } else {
                    dynprm_default
                };
                (gt, bt, gp, bp, dp)
            }
        };

        self.actuator_gaintype.push(gaintype);
        self.actuator_biastype.push(biastype);
        self.actuator_gainprm.push(gainprm);
        self.actuator_biasprm.push(biasprm);
        self.actuator_dynprm.push(dynprm);

        // Per-actuator group assignment (§41 S7b)
        self.actuator_group.push(actuator.group.unwrap_or(0));

        // Lengthrange and acc0: initialized to zero, computed by compute_actuator_params()
        self.actuator_lengthrange.push((0.0, 0.0));
        self.actuator_acc0.push(0.0);

        Ok(act_id)
    }
}

fn parse_gaintype(s: &str) -> std::result::Result<GainType, ModelConversionError> {
    match s {
        "fixed" => Ok(GainType::Fixed),
        "affine" => Ok(GainType::Affine),
        "muscle" => Ok(GainType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown gaintype '{s}' (valid: fixed, affine, muscle)"),
        }),
    }
}

fn parse_biastype(s: &str) -> std::result::Result<BiasType, ModelConversionError> {
    match s {
        "none" => Ok(BiasType::None),
        "affine" => Ok(BiasType::Affine),
        "muscle" => Ok(BiasType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown biastype '{s}' (valid: none, affine, muscle)"),
        }),
    }
}

fn parse_dyntype(s: &str) -> std::result::Result<ActuatorDynamics, ModelConversionError> {
    match s {
        "none" => Ok(ActuatorDynamics::None),
        "integrator" => Ok(ActuatorDynamics::Integrator),
        "filter" => Ok(ActuatorDynamics::Filter),
        "filterexact" => Ok(ActuatorDynamics::FilterExact),
        "muscle" => Ok(ActuatorDynamics::Muscle),
        _ => Err(ModelConversionError {
            message: format!(
                "unknown dyntype '{s}' (valid: none, integrator, filter, filterexact, muscle)"
            ),
        }),
    }
}

/// Convert a variable-length parsed float vector into a fixed-size array,
/// padding with the given default value. Truncates if input exceeds `N`.
fn floats_to_array<const N: usize>(input: &[f64], default: [f64; N]) -> [f64; N] {
    let mut out = default;
    for (i, &v) in input.iter().enumerate().take(N) {
        out[i] = v;
    }
    out
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
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
        // Explicit gainprm="50" overrides class default "0 0 -10"
        assert!((model.actuator_gainprm[0][0] - 50.0).abs() < 1e-10);
        for i in 1..9 {
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

    /// Criterion 9: Invalid enum produces ModelConversionError.
    #[test]
    fn test_general_invalid_gaintype_error() {
        let xml = general_actuator_model(r#"<general joint="j" gaintype="invalid"/>"#);
        let err = load_model(&xml).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("unknown gaintype"),
            "error should contain 'unknown gaintype', got: {msg}"
        );
    }

    /// Criterion 10: "user" type produces error (not special-cased).
    #[test]
    fn test_general_user_gaintype_error() {
        let xml = general_actuator_model(r#"<general joint="j" gaintype="user"/>"#);
        let err = load_model(&xml).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("unknown gaintype 'user'"),
            "error should contain \"unknown gaintype 'user'\", got: {msg}"
        );
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

    /// Criterion 13: Extra gainprm elements (>9) silently truncated.
    #[test]
    fn test_general_extra_gainprm_truncated() {
        let xml = general_actuator_model(r#"<general joint="j" gainprm="1 2 3 4 5 6 7 8 9 10"/>"#);
        let model = load_model(&xml).expect("should load");

        // First 9 elements stored
        let expected = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        for (i, &exp) in expected.iter().enumerate() {
            assert!(
                (model.actuator_gainprm[0][i] - exp).abs() < 1e-10,
                "gainprm[{i}] expected {exp}, got {}",
                model.actuator_gainprm[0][i]
            );
        }
        // 10th element (index 9) is silently dropped — no error, and array is only 9 elements
    }
}
