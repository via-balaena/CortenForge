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
            | ActuatorDynamics::Muscle => 1,
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
                [0.0; 3],
            ),

            MjcfActuatorType::Position => {
                let kp = actuator.kp; // default 1.0
                // kv resolved above from Option<f64> (default 0.0 for position)
                let tc = timeconst; // resolved above (default 0.0 for position)
                let mut gp = [0.0; 9];
                gp[0] = kp;
                let mut bp = [0.0; 9];
                bp[1] = -kp;
                bp[2] = -kv;
                (GainType::Fixed, BiasType::Affine, gp, bp, [tc, 0.0, 0.0])
            }

            MjcfActuatorType::Velocity => {
                // kv resolved above from Option<f64> (default 1.0 for velocity)
                let mut gp = [0.0; 9];
                gp[0] = kv;
                let mut bp = [0.0; 9];
                bp[2] = -kv;
                (GainType::Fixed, BiasType::Affine, gp, bp, [0.0; 3])
            }

            MjcfActuatorType::Damper => {
                // kv resolved above from Option<f64> (default 0.0 for damper)
                let mut gp = [0.0; 9];
                gp[2] = -kv; // gain = -kv * velocity
                (GainType::Affine, BiasType::None, gp, [0.0; 9], [0.0; 3])
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
                (GainType::Fixed, BiasType::Affine, gp, bp, [tc, 0.0, 0.0])
            }

            MjcfActuatorType::Adhesion => {
                let mut gp = [0.0; 9];
                gp[0] = actuator.gain;
                (GainType::Fixed, BiasType::None, gp, [0.0; 9], [0.0; 3])
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
                    [
                        actuator.muscle_timeconst.0,
                        actuator.muscle_timeconst.1,
                        0.0,
                    ],
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
                let dynprm_default = [1.0, 0.0, 0.0]; // MuJoCo: dynprm[0]=1
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

        // Lengthrange and acc0: initialized to zero, computed by compute_muscle_params()
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
