//! MJCF model builder — remaining functions pending extraction.
//!
//! Items extracted to `crate::builder` are re-imported below. This module
//! retains functions that will move to `builder/` sub-modules in later
//! Phase 10 sessions. It will be deleted in Phase 12.

use nalgebra::{DVector, UnitQuaternion, Vector3};
use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, ContactPair, ENABLE_SLEEP, EqualityType,
    GainType, GeomType, Integrator, MjJointType, MjObjectType, MjSensorDataType, MjSensorType,
    Model, SleepPolicy, TendonType, WrapType, compute_dof_lengths,
};
use tracing::warn;

use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfContact, MjcfEquality, MjcfFlex, MjcfSensor,
    MjcfSensorType, MjcfTendon, MjcfTendonType, SpatialPathElement,
};

// Re-imports from builder/ (extracted in Phase 10)
pub use crate::builder::DEFAULT_SOLIMP;
pub use crate::builder::DEFAULT_SOLREF;
pub use crate::builder::ModelBuilder;
pub use crate::builder::ModelConversionError;

// resolve_keyframe → crate::builder::resolve_keyframe (Phase 10)
#[cfg(test)]
use crate::builder::{load_model, load_model_from_file, model_from_mjcf};

// compute_geom_fluid, geom_semi_axes, get_added_mass_kappa, KRONROD_* → crate::builder::fluid (Phase 10)

// ModelBuilder struct, new(), set_options() → crate::builder (Phase 10)

// process_mesh, process_hfield → crate::builder::mesh (Phase 10)

// process_worldbody_geoms_and_sites, process_body, process_body_with_world_frame
// → crate::builder::body (Phase 10)

impl ModelBuilder {
    // process_joint → crate::builder::joint (Phase 10)
    // process_geom, process_site → crate::builder::geom (Phase 10)

    pub(crate) fn process_tendons(
        &mut self,
        tendons: &[MjcfTendon],
    ) -> std::result::Result<(), ModelConversionError> {
        for (t_idx, tendon) in tendons.iter().enumerate() {
            let tendon = self.resolver.apply_to_tendon(tendon);
            if !tendon.name.is_empty() {
                self.tendon_name_to_id.insert(tendon.name.clone(), t_idx);
            }
            self.tendon_type.push(match tendon.tendon_type {
                MjcfTendonType::Fixed => TendonType::Fixed,
                MjcfTendonType::Spatial => TendonType::Spatial,
            });
            self.tendon_range
                .push(tendon.range.unwrap_or((-f64::MAX, f64::MAX)));
            // Autolimits: infer limited from range presence.
            let tendon_limited = match tendon.limited {
                Some(v) => v,
                None => {
                    if self.compiler.autolimits {
                        tendon.range.is_some()
                    } else {
                        false
                    }
                }
            };
            self.tendon_limited.push(tendon_limited);
            self.tendon_stiffness.push(tendon.stiffness.unwrap_or(0.0));
            self.tendon_damping.push(tendon.damping.unwrap_or(0.0));
            self.tendon_frictionloss
                .push(tendon.frictionloss.unwrap_or(0.0));
            self.tendon_solref_fri
                .push(tendon.solreffriction.unwrap_or(DEFAULT_SOLREF));
            self.tendon_solimp_fri
                .push(tendon.solimpfriction.unwrap_or(DEFAULT_SOLIMP));
            self.tendon_name.push(if tendon.name.is_empty() {
                None
            } else {
                Some(tendon.name.clone())
            });
            self.tendon_group.push(tendon.group.unwrap_or(0));
            // Default tendon rgba: MuJoCo uses [0.5, 0.5, 0.5, 1.0] for unspecified.
            let default_rgba = [0.5, 0.5, 0.5, 1.0];
            self.tendon_rgba
                .push(tendon.rgba.map_or(default_rgba, |v| [v.x, v.y, v.z, v.w]));
            self.tendon_solref
                .push(tendon.solref.unwrap_or(DEFAULT_SOLREF));
            self.tendon_solimp
                .push(tendon.solimp.unwrap_or(DEFAULT_SOLIMP));
            // S1/S3: Use parsed springlength, or sentinel [-1, -1] for auto-compute
            self.tendon_lengthspring.push(match tendon.springlength {
                Some(pair) => pair.into(),
                None => [-1.0, -1.0], // sentinel: resolved to [length0, length0] later
            });
            self.tendon_length0.push(0.0); // Computed after construction from qpos0

            let wrap_start = self.wrap_type.len();
            self.tendon_adr.push(wrap_start);

            match tendon.tendon_type {
                MjcfTendonType::Fixed => {
                    for (joint_name, coef) in &tendon.joints {
                        // Resolve joint name → index → DOF address
                        let jnt_idx =
                            *self
                                .joint_name_to_id
                                .get(joint_name.as_str())
                                .ok_or_else(|| ModelConversionError {
                                    message: format!(
                                        "Tendon '{}' references unknown joint '{}'",
                                        tendon.name, joint_name
                                    ),
                                })?;
                        // Fixed tendons assume qposadr == dofadr (true for hinge/slide).
                        // Ball/free joints have different qpos and dof dimensions, so
                        // a linear coupling L = coef * qpos[dof_adr] would be incorrect.
                        let jnt_type = self.jnt_type[jnt_idx];
                        if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                            warn!(
                                "Tendon '{}' references {} joint '{}' — fixed tendons only \
                                 support hinge/slide joints. Ball/free joints will produce \
                                 incorrect results.",
                                tendon.name,
                                match jnt_type {
                                    MjJointType::Ball => "ball",
                                    MjJointType::Free => "free",
                                    _ => "unknown",
                                },
                                joint_name
                            );
                        }
                        let dof_adr = self.jnt_dof_adr[jnt_idx];
                        self.wrap_type.push(WrapType::Joint);
                        self.wrap_objid.push(dof_adr);
                        self.wrap_prm.push(*coef);
                        self.wrap_sidesite.push(usize::MAX);
                    }
                }
                MjcfTendonType::Spatial => {
                    for elem in &tendon.path_elements {
                        match elem {
                            SpatialPathElement::Site { site } => {
                                let site_idx = *self
                                    .site_name_to_id
                                    .get(site.as_str())
                                    .ok_or_else(|| ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' references unknown site '{}'",
                                            tendon.name, site
                                        ),
                                    })?;
                                self.wrap_type.push(WrapType::Site);
                                self.wrap_objid.push(site_idx);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                            SpatialPathElement::Geom { geom, sidesite } => {
                                let geom_id =
                                    *self.geom_name_to_id.get(geom.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown geom '{}'",
                                                tendon.name, geom
                                            ),
                                        }
                                    })?;
                                // Validate geom type: must be Sphere or Cylinder
                                let gt = self.geom_type[geom_id];
                                if gt != GeomType::Sphere && gt != GeomType::Cylinder {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has type {:?}, \
                                             but only Sphere and Cylinder are supported",
                                            tendon.name, geom, gt
                                        ),
                                    });
                                }
                                // Validate geom radius > 0 (rule 5)
                                let radius = self.geom_size[geom_id].x;
                                if radius <= 0.0 {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has zero or \
                                             negative radius {}",
                                            tendon.name, geom, radius
                                        ),
                                    });
                                }
                                let ss_id = if let Some(ss) = sidesite {
                                    *self.site_name_to_id.get(ss.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown sidesite '{}'",
                                                tendon.name, ss
                                            ),
                                        }
                                    })?
                                } else {
                                    usize::MAX
                                };
                                self.wrap_type.push(WrapType::Geom);
                                self.wrap_objid.push(geom_id);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(ss_id);
                            }
                            SpatialPathElement::Pulley { divisor } => {
                                self.wrap_type.push(WrapType::Pulley);
                                self.wrap_objid.push(0);
                                self.wrap_prm.push(*divisor);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                        }
                    }
                }
            }
            self.tendon_num.push(self.wrap_type.len() - wrap_start);
        }
        Ok(())
    }

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

    /// Process sensor definitions from MJCF.
    ///
    /// Converts `MjcfSensor` objects into the 13 pipeline sensor arrays.
    /// Must be called AFTER all bodies, joints, tendons, and actuators are processed
    /// (requires name→id lookups for object resolution).
    pub(crate) fn process_sensors(
        &mut self,
        sensors: &[MjcfSensor],
    ) -> std::result::Result<(), ModelConversionError> {
        let mut adr = 0usize;

        for mjcf_sensor in sensors {
            let mjcf_sensor = self.resolver.apply_to_sensor(mjcf_sensor);
            let Some(sensor_type) = convert_sensor_type(mjcf_sensor.sensor_type) else {
                // Unsupported type (Jointlimitfrc, Tendonlimitfrc) — skip with log
                warn!(
                    "Skipping unsupported sensor type '{:?}' (sensor '{}')",
                    mjcf_sensor.sensor_type, mjcf_sensor.name,
                );
                continue;
            };
            let dim = sensor_type.dim();
            let datatype = sensor_datatype(sensor_type);
            let (objtype, objid) =
                self.resolve_sensor_object(sensor_type, mjcf_sensor.objname.as_deref())?;

            self.sensor_type.push(sensor_type);
            self.sensor_datatype.push(datatype);
            self.sensor_objtype.push(objtype);
            self.sensor_objid.push(objid);
            self.sensor_reftype.push(MjObjectType::None);
            self.sensor_refid.push(0);
            self.sensor_adr.push(adr);
            self.sensor_dim.push(dim);
            self.sensor_noise.push(mjcf_sensor.noise);
            self.sensor_cutoff.push(mjcf_sensor.cutoff);
            self.sensor_name_list.push(if mjcf_sensor.name.is_empty() {
                None
            } else {
                Some(mjcf_sensor.name.clone())
            });

            adr += dim;
        }

        self.nsensor = self.sensor_type.len();
        self.nsensordata = adr;
        Ok(())
    }

    /// Process `<contact>` element: resolve `<pair>` and `<exclude>` entries.
    ///
    /// For each `<pair>`: apply defaults class, resolve geom names to indices,
    /// compute geom-combination fallbacks for unspecified attributes, and build
    /// a fully-resolved `ContactPair`. Duplicate pairs (same geom pair) use
    /// last-wins semantics.
    ///
    /// For each `<exclude>`: resolve body names to indices and insert into the
    /// exclude set.
    pub(crate) fn process_contact(
        &mut self,
        contact: &MjcfContact,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process pairs
        for mjcf_pair in &contact.pairs {
            // Stage 1: apply defaults class
            let pair = self.resolver.apply_to_pair(mjcf_pair);

            // Resolve geom names to indices
            let g1 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom1",
                            pair.geom1
                        ),
                    })?;
            let g2 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom2)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom2",
                            pair.geom2
                        ),
                    })?;

            // Stage 2: geom-combination fallbacks for unspecified attributes
            let condim = pair
                .condim
                .unwrap_or_else(|| self.geom_condim[g1].max(self.geom_condim[g2]));

            let friction = pair.friction.unwrap_or_else(|| {
                let f1 = self.geom_friction[g1];
                let f2 = self.geom_friction[g2];
                // Geometric mean in 3D, then expand to 5D: [s, s, t, r, r]
                let s = (f1.x * f2.x).sqrt();
                let t = (f1.y * f2.y).sqrt();
                let r = (f1.z * f2.z).sqrt();
                [s, s, t, r, r]
            });

            let solref = pair.solref.unwrap_or_else(|| {
                // Element-wise min (our standing approximation)
                [
                    self.geom_solref[g1][0].min(self.geom_solref[g2][0]),
                    self.geom_solref[g1][1].min(self.geom_solref[g2][1]),
                ]
            });

            let solimp = pair.solimp.unwrap_or_else(|| {
                // Element-wise max (our standing approximation)
                let s1 = self.geom_solimp[g1];
                let s2 = self.geom_solimp[g2];
                [
                    s1[0].max(s2[0]),
                    s1[1].max(s2[1]),
                    s1[2].max(s2[2]),
                    s1[3].max(s2[3]),
                    s1[4].max(s2[4]),
                ]
            });

            // solreffriction: [0,0] sentinel means "use solref" (MuJoCo convention).
            // Only explicit <pair solreffriction="..."/> sets a nonzero value.
            let solreffriction = pair.solreffriction.unwrap_or([0.0, 0.0]);

            // margin/gap: geom-level not yet parsed, default to 0.0
            let margin = pair.margin.unwrap_or(0.0);
            let gap = pair.gap.unwrap_or(0.0);

            let contact_pair = ContactPair {
                geom1: g1,
                geom2: g2,
                condim,
                friction,
                solref,
                solreffriction,
                solimp,
                margin,
                gap,
            };

            // Deduplicate: canonical key (min, max) for symmetry
            let key = (g1.min(g2), g1.max(g2));
            if self.contact_pair_set.contains(&key) {
                // Last-wins: replace existing entry
                let pos = self
                    .contact_pairs
                    .iter()
                    .position(|p| (p.geom1.min(p.geom2), p.geom1.max(p.geom2)) == key)
                    .ok_or_else(|| ModelConversionError {
                        message: "contact_pair_set and contact_pairs out of sync".into(),
                    })?;
                self.contact_pairs[pos] = contact_pair;
            } else {
                self.contact_pair_set.insert(key);
                self.contact_pairs.push(contact_pair);
            }
        }

        // Process excludes
        for mjcf_exclude in &contact.excludes {
            let b1 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body1",
                        mjcf_exclude.body1
                    ),
                })?;
            let b2 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body2)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body2",
                        mjcf_exclude.body2
                    ),
                })?;
            // Canonical key for symmetry
            self.contact_excludes.insert((b1.min(b2), b1.max(b2)));
        }

        Ok(())
    }

    /// Returns (MjObjectType, objid) for a given sensor type and MJCF objname.
    fn resolve_sensor_object(
        &self,
        sensor_type: MjSensorType,
        objname: Option<&str>,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        // User sensors have no object reference
        if sensor_type == MjSensorType::User {
            return Ok((MjObjectType::None, 0));
        }

        let name = objname.ok_or_else(|| ModelConversionError {
            message: "sensor missing object name".into(),
        })?;

        match sensor_type {
            // Joint sensors: objname is a joint name
            MjSensorType::JointPos
            | MjSensorType::JointVel
            | MjSensorType::BallQuat
            | MjSensorType::BallAngVel
            | MjSensorType::JointLimitFrc => {
                let id = *self
                    .joint_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown joint '{name}'"),
                    })?;
                Ok((MjObjectType::Joint, id))
            }

            // Tendon sensors: objname is a tendon name
            MjSensorType::TendonPos | MjSensorType::TendonVel | MjSensorType::TendonLimitFrc => {
                let id = *self
                    .tendon_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown tendon '{name}'"),
                    })?;
                Ok((MjObjectType::Tendon, id))
            }

            // Actuator sensors: objname is an actuator name
            MjSensorType::ActuatorPos | MjSensorType::ActuatorVel | MjSensorType::ActuatorFrc => {
                let id =
                    *self
                        .actuator_name_to_id
                        .get(name)
                        .ok_or_else(|| ModelConversionError {
                            message: format!("sensor references unknown actuator '{name}'"),
                        })?;
                Ok((MjObjectType::Actuator, id))
            }

            // Site-attached sensors: objname is a site name
            MjSensorType::Accelerometer
            | MjSensorType::Velocimeter
            | MjSensorType::Gyro
            | MjSensorType::Force
            | MjSensorType::Torque
            | MjSensorType::Magnetometer
            | MjSensorType::Rangefinder => {
                let id = *self
                    .site_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown site '{name}'"),
                    })?;
                Ok((MjObjectType::Site, id))
            }

            // Touch: MJCF uses site=, but pipeline expects geom ID.
            // Resolve site → body → first geom on that body.
            MjSensorType::Touch => {
                let site_id =
                    *self
                        .site_name_to_id
                        .get(name)
                        .ok_or_else(|| ModelConversionError {
                            message: format!("touch sensor references unknown site '{name}'"),
                        })?;
                let body_id = *self
                    .site_body
                    .get(site_id)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "touch sensor: site_id {site_id} out of range for site_body"
                        ),
                    })?;
                // Find first geom belonging to this body
                let geom_id = self
                    .geom_body
                    .iter()
                    .position(|&b| b == body_id)
                    .unwrap_or(usize::MAX);
                Ok((MjObjectType::Geom, geom_id))
            }

            // Frame sensors: resolve objname by trying site → body → geom
            MjSensorType::FramePos
            | MjSensorType::FrameQuat
            | MjSensorType::FrameXAxis
            | MjSensorType::FrameYAxis
            | MjSensorType::FrameZAxis
            | MjSensorType::FrameLinVel
            | MjSensorType::FrameAngVel
            | MjSensorType::FrameLinAcc
            | MjSensorType::FrameAngAcc => {
                if let Some(&id) = self.site_name_to_id.get(name) {
                    Ok((MjObjectType::Site, id))
                } else if let Some(&id) = self.body_name_to_id.get(name) {
                    Ok((MjObjectType::Body, id))
                } else if let Some(&id) = self.geom_name_to_id.get(name) {
                    Ok((MjObjectType::Geom, id))
                } else {
                    Err(ModelConversionError {
                        message: format!("frame sensor references unknown object '{name}'"),
                    })
                }
            }

            // Subtree sensors: objname is a body name
            MjSensorType::SubtreeCom
            | MjSensorType::SubtreeLinVel
            | MjSensorType::SubtreeAngMom => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown body '{name}'"),
                    })?;
                Ok((MjObjectType::Body, id))
            }

            // User handled above (early return)
            MjSensorType::User => unreachable!(),
        }
    }

    /// Process equality constraints from MJCF.
    ///
    /// Converts `MjcfEquality` (connects, welds, joints, distances) into Model arrays.
    /// Must be called AFTER all bodies and joints are processed (requires name lookups).
    ///
    /// # MuJoCo Semantics
    ///
    /// - **Connect**: Ball-and-socket constraint. Removes 3 translational DOFs.
    ///   `eq_data[0..3]` = anchor point in body1's local frame.
    ///
    /// - **Weld**: Fixed frame constraint. Removes 6 DOFs (translation + rotation).
    ///   `eq_data[0..3]` = anchor point, `eq_data[3..7]` = relative quaternion [w,x,y,z].
    ///
    /// - **Joint**: Polynomial coupling between joints.
    ///   `eq_data[0..5]` = polynomial coefficients (q2 = c0 + c1*q1 + c2*q1² + ...).
    ///
    /// - **Distance**: Maintains fixed distance between geom centers.
    ///   `eq_data[0]` = target distance.
    pub(crate) fn process_equality_constraints(
        &mut self,
        equality: &MjcfEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process Connect constraints (3 DOF position constraint)
        for connect in &equality.connects {
            let body1_id =
                self.body_name_to_id
                    .get(&connect.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body1: '{}'",
                            connect.body1
                        ),
                    })?;

            // body2 defaults to world (body 0) if not specified
            let body2_id = if let Some(ref body2_name) = connect.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Pack data: anchor point in body1 frame, rest zeroed
            let mut data = [0.0; 11];
            data[0] = connect.anchor.x;
            data[1] = connect.anchor.y;
            data[2] = connect.anchor.z;

            self.eq_type.push(EqualityType::Connect);
            self.eq_obj1id.push(*body1_id);
            self.eq_obj2id.push(body2_id);
            self.eq_data.push(data);
            self.eq_active.push(connect.active);
            self.eq_solimp
                .push(connect.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref
                .push(connect.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(connect.name.clone());
        }

        // Process Weld constraints (6 DOF pose constraint)
        for weld in &equality.welds {
            let body1_id =
                self.body_name_to_id
                    .get(&weld.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body1: '{}'",
                            weld.body1
                        ),
                    })?;

            let body2_id = if let Some(ref body2_name) = weld.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Pack data: anchor point + relative pose quaternion
            // relpose format: [x, y, z, qw, qx, qy, qz]
            let mut data = [0.0; 11];
            data[0] = weld.anchor.x;
            data[1] = weld.anchor.y;
            data[2] = weld.anchor.z;
            if let Some(relpose) = weld.relpose {
                // relpose = [x, y, z, qw, qx, qy, qz]
                // We store anchor separately, so relpose goes to data[3..10]
                data[3] = relpose[3]; // qw
                data[4] = relpose[4]; // qx
                data[5] = relpose[5]; // qy
                data[6] = relpose[6]; // qz
                // Note: relpose position offset is in addition to anchor
                data[7] = relpose[0]; // dx
                data[8] = relpose[1]; // dy
                data[9] = relpose[2]; // dz
            } else {
                // Default: identity quaternion, no offset
                data[3] = 1.0; // qw = 1 (identity)
            }

            self.eq_type.push(EqualityType::Weld);
            self.eq_obj1id.push(*body1_id);
            self.eq_obj2id.push(body2_id);
            self.eq_data.push(data);
            self.eq_active.push(weld.active);
            self.eq_solimp.push(weld.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref.push(weld.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(weld.name.clone());
        }

        // Process Joint coupling constraints
        for joint_eq in &equality.joints {
            let joint1_id = self.joint_name_to_id.get(&joint_eq.joint1).ok_or_else(|| {
                ModelConversionError {
                    message: format!(
                        "Joint equality references unknown joint1: '{}'",
                        joint_eq.joint1
                    ),
                }
            })?;

            // joint2 is optional - if not specified, constraint locks joint1 to polycoef[0]
            let joint2_id = if let Some(ref joint2_name) = joint_eq.joint2 {
                *self
                    .joint_name_to_id
                    .get(joint2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Joint equality references unknown joint2: '{joint2_name}'"
                        ),
                    })?
            } else {
                // MuJoCo convention: joint2 = -1 means constraint locks joint1 to constant
                // We use usize::MAX as sentinel (will never be a valid joint ID)
                usize::MAX
            };

            // Pack polynomial coefficients (up to 5 terms)
            // MuJoCo: q2 = c0 + c1*q1 + c2*q1² + c3*q1³ + c4*q1⁴
            let mut data = [0.0; 11];
            for (i, &coef) in joint_eq.polycoef.iter().take(5).enumerate() {
                data[i] = coef;
            }

            self.eq_type.push(EqualityType::Joint);
            self.eq_obj1id.push(*joint1_id);
            self.eq_obj2id.push(joint2_id);
            self.eq_data.push(data);
            self.eq_active.push(joint_eq.active);
            self.eq_solimp
                .push(joint_eq.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref
                .push(joint_eq.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(joint_eq.name.clone());
        }

        // Process Distance constraints (1 DOF scalar distance between geom centers)
        for dist in &equality.distances {
            let geom1_id =
                *self
                    .geom_name_to_id
                    .get(&dist.geom1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Distance constraint references unknown geom1: '{}'",
                            dist.geom1
                        ),
                    })?;

            // geom2 defaults to usize::MAX sentinel (world origin) if not specified
            let geom2_id = if let Some(ref geom2_name) = dist.geom2 {
                *self
                    .geom_name_to_id
                    .get(geom2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Distance constraint references unknown geom2: '{geom2_name}'"
                        ),
                    })?
            } else {
                usize::MAX
            };

            let target_distance = if let Some(d) = dist.distance {
                d.max(0.0) // Distance is non-negative by definition
            } else {
                self.compute_initial_geom_distance(geom1_id, geom2_id)
            };

            let mut data = [0.0; 11];
            data[0] = target_distance;

            self.eq_type.push(EqualityType::Distance);
            self.eq_obj1id.push(geom1_id);
            self.eq_obj2id.push(geom2_id);
            self.eq_data.push(data);
            self.eq_active.push(dist.active);
            self.eq_solimp.push(dist.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref.push(dist.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(dist.name.clone());
        }

        // Process Tendon equality constraints
        for ten_eq in &equality.tendons {
            let t1_id = *self
                .tendon_name_to_id
                .get(ten_eq.tendon1.as_str())
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Tendon equality references unknown tendon1: '{}'",
                        ten_eq.tendon1
                    ),
                })?;

            let t2_id = if let Some(ref t2_name) = ten_eq.tendon2 {
                *self
                    .tendon_name_to_id
                    .get(t2_name.as_str())
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Tendon equality references unknown tendon2: '{t2_name}'"),
                    })?
            } else {
                usize::MAX // Sentinel: single-tendon mode
            };

            // Pack polynomial coefficients (up to 5 terms)
            let mut data = [0.0; 11];
            for (i, &coef) in ten_eq.polycoef.iter().take(5).enumerate() {
                data[i] = coef;
            }

            self.eq_type.push(EqualityType::Tendon);
            self.eq_obj1id.push(t1_id);
            self.eq_obj2id.push(t2_id);
            self.eq_data.push(data);
            self.eq_active.push(ten_eq.active);
            self.eq_solimp.push(ten_eq.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref.push(ten_eq.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(ten_eq.name.clone());
        }

        Ok(())
    }

    /// Compute world-space distance between two geom centers at build time.
    ///
    /// Used when `distance: None` to derive target from initial configuration.
    /// `geom2_id == usize::MAX` means world origin.
    fn compute_initial_geom_distance(&self, geom1_id: usize, geom2_id: usize) -> f64 {
        let p1 = self.geom_world_position(geom1_id);
        let p2 = if geom2_id == usize::MAX {
            Vector3::zeros()
        } else {
            self.geom_world_position(geom2_id)
        };
        (p1 - p2).norm()
    }

    /// Compute the world-space position of a geom at build time.
    fn geom_world_position(&self, geom_id: usize) -> Vector3<f64> {
        let body_id = self.geom_body[geom_id];
        if body_id == 0 {
            self.geom_pos[geom_id]
        } else {
            let body_world_pos = self.body_world_pos[body_id - 1];
            let body_world_quat = self.body_world_quat[body_id - 1];
            body_world_pos + body_world_quat * self.geom_pos[geom_id]
        }
    }

    // apply_mass_pipeline → crate::builder::mass (Phase 10)

    /// Process flex deformable bodies: create a real body (+ 3 slide joints for unpinned
    /// vertices) per flex vertex, extract edges/hinges, populate Model flex_* arrays.
    ///
    /// MuJoCo architecture: flex vertices ARE bodies in the kinematic tree. `mj_crb`/`mj_rne`
    /// have zero flex-specific code — the standard rigid-body pipeline handles everything.
    pub(crate) fn process_flex_bodies(
        &mut self,
        flex_list: &[MjcfFlex],
    ) -> std::result::Result<(), ModelConversionError> {
        use std::collections::HashMap;

        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

        for flex_orig in flex_list {
            // Resolve `node` attribute: if `node` names are present and no vertices
            // exist, use the named bodies as flex vertices (MuJoCo semantics).
            //
            // Each node body IS the vertex. The new vertex body is parented to the
            // node body with body_pos = zeros (vertex is at node body origin).
            // We store the node body's world position as the vertex position so that
            // edge rest-lengths and mass lumping use correct geometry. Then override
            // body_pos to zeros when creating the vertex body (see `is_node_flex`
            // flag below).
            let resolved;
            let is_node_flex;
            let flex = if !flex_orig.node.is_empty() && flex_orig.vertices.is_empty() {
                resolved = {
                    let mut f = flex_orig.clone();
                    for node_name in &flex_orig.node {
                        let &bid = self.body_name_to_id.get(node_name).ok_or_else(|| {
                            ModelConversionError {
                                message: format!(
                                    "flex '{}' node references undefined body '{}'",
                                    flex_orig.name, node_name
                                ),
                            }
                        })?;
                        // World position for geometry (rest-lengths, mass lumping).
                        // body_world_pos is indexed by (body_id - 1) since worldbody
                        // has no entry. Node bodies are always non-world (bid >= 1).
                        let world_pos = if bid == 0 {
                            Vector3::zeros()
                        } else {
                            self.body_world_pos[bid - 1]
                        };
                        f.vertices.push(world_pos);
                        f.body.push(node_name.clone());
                    }
                    f
                };
                is_node_flex = true;
                &resolved
            } else {
                is_node_flex = false;
                flex_orig
            };
            let flex_id = self.nflex;
            self.nflex += 1;

            let vert_start = self.nflexvert;

            // Compute per-vertex masses (uniform from mass attr, or element-based lumping)
            let vertex_masses = compute_vertex_masses(flex);

            // Create a body (+ joints for unpinned) per vertex
            for (i, pos) in flex.vertices.iter().enumerate() {
                let mass = vertex_masses[i];
                let pinned = flex.pinned.contains(&i);
                let inv_mass = if pinned || mass <= 0.0 {
                    0.0
                } else {
                    1.0 / mass
                };

                self.nflexvert += 1;

                // --- Create body for this vertex ---
                let body_id = self.body_parent.len();

                // Resolve parent body: for bare <flex> use the named body attr;
                // for <flexcomp> (no body attr), parent is worldbody (0).
                let parent_id = if i < flex.body.len() {
                    if let Some(&bid) = self.body_name_to_id.get(&flex.body[i]) {
                        bid
                    } else {
                        eprintln!(
                            "Warning: flex '{}' vertex {} references unknown body '{}', \
                             parenting to worldbody",
                            flex.name, i, flex.body[i]
                        );
                        0
                    }
                } else {
                    0 // <flexcomp> → parent is worldbody
                };

                // Each flex vertex body parented to worldbody is its own kinematic tree root.
                // For bodies parented to a non-world body, inherit the root.
                let root_id = if parent_id == 0 {
                    body_id
                } else {
                    self.body_rootid[parent_id]
                };

                let jnt_adr = self.jnt_type.len();
                let dof_adr = self.nv;
                let geom_adr = self.geom_type.len();

                // Push body arrays (mirrors process_body_with_world_frame pattern)
                self.body_parent.push(parent_id);
                self.body_rootid.push(root_id);
                self.body_jnt_adr.push(jnt_adr);
                self.body_dof_adr.push(dof_adr);
                self.body_geom_adr.push(geom_adr);
                // For node-derived vertices, body_pos = zeros (vertex is at the node
                // body's origin; the slide joints provide displacement from there).
                // For flexcomp/bare-flex, body_pos = vertex world position (parent is
                // worldbody, so local offset == world position).
                let body_pos_val = if is_node_flex { Vector3::zeros() } else { *pos };
                self.body_pos.push(body_pos_val);
                self.body_quat.push(UnitQuaternion::identity());
                self.body_ipos.push(Vector3::zeros()); // point mass: CoM at body origin
                self.body_iquat.push(UnitQuaternion::identity());
                self.body_mass.push(mass);
                self.body_inertia.push(Vector3::zeros()); // point mass: zero inertia
                self.body_mocapid.push(None);
                self.body_name.push(None);
                self.body_sleep_policy.push(None);
                // *pos is the vertex world position from node resolution or flexcomp.
                // For flexcomp (parent=world), local offset == world position.
                // For node-flex, *pos is the node body's world position.
                self.body_world_pos.push(*pos);
                self.body_world_quat.push(UnitQuaternion::identity());

                if pinned {
                    // Pinned vertex: body exists but has no joints/DOFs.
                    // Body is fixed at its initial position (like a welded body).
                    self.body_jnt_num.push(0);
                    self.body_dof_num.push(0);
                    self.body_geom_num.push(0);

                    self.flexvert_qposadr.push(usize::MAX); // sentinel: no qpos
                    self.flexvert_dofadr.push(usize::MAX); // sentinel: no dof
                } else {
                    // Unpinned vertex: 3 slide joints (X, Y, Z)
                    // dof_parent chain: first DOF has no parent (worldbody has no DOFs),
                    // subsequent DOFs chain to previous within this vertex.
                    let mut last_dof: Option<usize> = None;
                    // Determine parent's last DOF for kinematic tree linkage.
                    // Worldbody (parent_id=0) has no DOFs → None.
                    // Non-world parent: use their last DOF.
                    let parent_last_dof = if parent_id == 0 {
                        None
                    } else {
                        let pdof_adr = self.body_dof_adr[parent_id];
                        let pdof_num = self.body_dof_num[parent_id];
                        if pdof_num > 0 {
                            Some(pdof_adr + pdof_num - 1)
                        } else {
                            None
                        }
                    };

                    for (axis_idx, axis) in axes.iter().enumerate() {
                        let jnt_id = self.jnt_type.len();
                        let qpos_adr = self.nq;
                        let dof_idx = self.nv;

                        // Joint arrays
                        self.jnt_type.push(MjJointType::Slide);
                        self.jnt_body.push(body_id);
                        self.jnt_qpos_adr.push(qpos_adr);
                        self.jnt_dof_adr.push(dof_idx);
                        self.jnt_pos.push(Vector3::zeros());
                        self.jnt_axis.push(*axis);
                        self.jnt_limited.push(false);
                        self.jnt_range
                            .push((-std::f64::consts::PI, std::f64::consts::PI));
                        self.jnt_stiffness.push(0.0);
                        self.jnt_springref.push(0.0);
                        self.jnt_damping.push(0.0);
                        self.jnt_armature.push(0.0);

                        // DOF arrays
                        let dof_parent_val = if axis_idx == 0 {
                            parent_last_dof // first DOF links to parent body's last DOF
                        } else {
                            last_dof // subsequent DOFs chain within this vertex
                        };
                        self.dof_parent.push(dof_parent_val);
                        self.dof_body.push(body_id);
                        self.dof_jnt.push(jnt_id);
                        self.dof_armature.push(0.0);
                        self.dof_damping.push(0.0);
                        self.dof_frictionloss.push(0.0);
                        self.dof_solref.push(DEFAULT_SOLREF);
                        self.dof_solimp.push(DEFAULT_SOLIMP);

                        // qpos0 = 0 for each slide DOF (body_pos encodes initial position)
                        self.qpos0_values.push(0.0);

                        self.nq += 1;
                        self.nv += 1;
                        last_dof = Some(dof_idx);
                    }

                    self.body_jnt_num.push(3);
                    self.body_dof_num.push(3);
                    self.body_geom_num.push(0);

                    self.flexvert_qposadr.push(dof_adr); // first of 3 slide joint qpos addresses
                    self.flexvert_dofadr.push(dof_adr); // first of 3 DOF addresses
                }

                // Convenience mirrors for edge/element force code
                self.flexvert_mass.push(mass);
                self.flexvert_invmass.push(inv_mass);
                self.flexvert_radius.push(flex.radius);
                self.flexvert_flexid.push(flex_id);
                self.flexvert_bodyid.push(body_id);
                self.flexvert_initial_pos.push(*pos);
            }

            // Extract edges from element connectivity
            let mut edge_set: HashMap<(usize, usize), bool> = HashMap::new();
            for elem in &flex.elements {
                let n = elem.len();
                for i_v in 0..n {
                    for j_v in (i_v + 1)..n {
                        let (a, b) = if elem[i_v] < elem[j_v] {
                            (elem[i_v], elem[j_v])
                        } else {
                            (elem[j_v], elem[i_v])
                        };
                        edge_set.entry((a, b)).or_insert(true);
                    }
                }
            }
            for &(a, b) in edge_set.keys() {
                let rest_len = (flex.vertices[b] - flex.vertices[a]).norm();
                self.flexedge_vert.push([vert_start + a, vert_start + b]);
                self.flexedge_length0.push(rest_len);
                self.flexedge_flexid.push(flex_id);
                self.nflexedge += 1;
            }

            // Extract bending hinges from adjacent elements
            let mut edge_elements: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
            for (elem_idx, verts) in flex.elements.iter().enumerate() {
                let n = verts.len();
                for i_v in 0..n {
                    for j_v in (i_v + 1)..n {
                        let (a, b) = if verts[i_v] < verts[j_v] {
                            (verts[i_v], verts[j_v])
                        } else {
                            (verts[j_v], verts[i_v])
                        };
                        edge_elements.entry((a, b)).or_default().push(elem_idx);
                    }
                }
            }
            for ((ve0, ve1), elems) in &edge_elements {
                if elems.len() != 2 {
                    continue; // boundary edge
                }
                let opp_a = flex.elements[elems[0]]
                    .iter()
                    .find(|&&v| v != *ve0 && v != *ve1)
                    .copied();
                let opp_b = flex.elements[elems[1]]
                    .iter()
                    .find(|&&v| v != *ve0 && v != *ve1)
                    .copied();
                if let (Some(va), Some(vb)) = (opp_a, opp_b) {
                    let rest_angle = compute_dihedral_angle(
                        flex.vertices[*ve0],
                        flex.vertices[*ve1],
                        flex.vertices[va],
                        flex.vertices[vb],
                    );
                    self.flexhinge_vert.push([
                        vert_start + *ve0,
                        vert_start + *ve1,
                        vert_start + va,
                        vert_start + vb,
                    ]);
                    self.flexhinge_angle0.push(rest_angle);
                    self.flexhinge_flexid.push(flex_id);
                    self.nflexhinge += 1;
                }
            }

            // Process elements
            for elem in &flex.elements {
                let adr = self.flexelem_data.len();
                for &v in elem {
                    self.flexelem_data.push(vert_start + v);
                }
                self.flexelem_dataadr.push(adr);
                self.flexelem_datanum.push(elem.len());
                self.flexelem_flexid.push(flex_id);

                let rest_vol = if elem.len() == 4 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let e3 = flex.vertices[elem[3]] - flex.vertices[elem[0]];
                    // Use signed volume to match the constraint assembly formula.
                    // Positive for right-handed vertex ordering, negative for inverted tets.
                    e1.dot(&e2.cross(&e3)) / 6.0
                } else {
                    0.0
                };
                self.flexelem_volume0.push(rest_vol);
                self.nflexelem += 1;
            }

            // Compute solref from material properties
            let edge_solref = compute_edge_solref(flex);
            let k_bend = compute_bend_stiffness_from_material(flex);
            let b_bend = compute_bend_damping_from_material(flex, k_bend);

            // Per-flex arrays
            self.flex_dim.push(flex.dim);
            self.flex_vertadr.push(vert_start);
            self.flex_vertnum.push(flex.vertices.len());
            self.flex_damping.push(flex.damping);
            self.flex_friction.push(flex.friction);
            self.flex_condim.push(flex.condim);
            self.flex_margin.push(flex.margin);
            self.flex_gap.push(flex.gap);
            self.flex_priority.push(flex.priority);
            self.flex_solmix.push(flex.solmix);
            self.flex_solref.push(flex.solref);
            self.flex_solimp.push(flex.solimp);
            self.flex_edge_solref.push(edge_solref);
            self.flex_edge_solimp.push(flex.solimp);
            self.flex_bend_stiffness.push(k_bend);
            self.flex_bend_damping.push(b_bend);
            self.flex_young.push(flex.young);
            self.flex_poisson.push(flex.poisson);
            self.flex_thickness.push(flex.thickness);
            self.flex_density.push(flex.density);
            self.flex_group.push(flex.group);
            self.flex_contype.push(flex.contype.unwrap_or(1) as u32);
            self.flex_conaffinity
                .push(flex.conaffinity.unwrap_or(1) as u32);
            // MuJoCo default is "auto" (enabled). Only "none" disables self-collision.
            // None (absent) → true; Some("none") → false; all other keywords → true.
            self.flex_selfcollide
                .push(flex.selfcollide.as_deref() != Some("none"));
            self.flex_edgestiffness.push(flex.edge_stiffness);
            self.flex_edgedamping.push(flex.edge_damping);
        }
        Ok(())
    }

    pub(crate) fn build(self) -> Model {
        let njnt = self.jnt_type.len();
        let nbody = self.body_parent.len();
        let ngeom = self.geom_type.len();
        let nsite = self.site_body.len();
        let nu = self.actuator_trntype.len();
        let ntendon = self.tendon_type.len();

        // Pre-compute flex address/count/crosssection tables before self is consumed.
        let flex_edgeadr = compute_flex_address_table(&self.flexedge_flexid, self.nflex);
        let flex_edgenum = compute_flex_count_table(&self.flexedge_flexid, self.nflex);
        let flex_elemadr = compute_flex_address_table(&self.flexelem_flexid, self.nflex);
        let flex_elemnum = compute_flex_count_table(&self.flexelem_flexid, self.nflex);
        let flexedge_crosssection = compute_flexedge_crosssection(
            &self.flexedge_flexid,
            &self.flexedge_length0,
            &self.flex_dim,
            &self.flex_thickness,
            &self.flexvert_radius,
            &self.flex_vertadr,
        );

        let mut model = Model {
            name: self.name,
            nq: self.nq,
            nv: self.nv,
            nbody,
            njnt,
            ngeom,
            nsite,
            nu,
            na: self.na,
            nmocap: self.nmocap,
            nkeyframe: 0,

            // Kinematic trees (§16.0) — computed below
            ntree: 0,
            tree_body_adr: vec![],
            tree_body_num: vec![],
            tree_dof_adr: vec![],
            tree_dof_num: vec![],
            body_treeid: vec![usize::MAX; nbody],
            dof_treeid: vec![0; self.nv],
            tree_sleep_policy: vec![],
            dof_length: vec![1.0; self.nv],
            sleep_tolerance: self.sleep_tolerance,

            body_parent: self.body_parent,
            body_rootid: self.body_rootid,
            body_jnt_adr: self.body_jnt_adr,
            body_jnt_num: self.body_jnt_num,
            body_dof_adr: self.body_dof_adr,
            body_dof_num: self.body_dof_num,
            body_geom_adr: self.body_geom_adr,
            body_geom_num: self.body_geom_num,
            body_pos: self.body_pos,
            body_quat: self.body_quat,
            body_ipos: self.body_ipos,
            body_iquat: self.body_iquat,
            body_mass: self.body_mass,
            body_inertia: self.body_inertia,
            body_name: self.body_name,
            body_subtreemass: vec![0.0; nbody], // Computed after model construction
            body_mocapid: self.body_mocapid,
            ngravcomp: self.body_gravcomp.iter().filter(|&&gc| gc != 0.0).count(),
            body_gravcomp: self.body_gravcomp,

            jnt_type: self.jnt_type,
            jnt_body: self.jnt_body,
            jnt_qpos_adr: self.jnt_qpos_adr,
            jnt_dof_adr: self.jnt_dof_adr,
            jnt_pos: self.jnt_pos,
            jnt_axis: self.jnt_axis,
            jnt_limited: self.jnt_limited,
            jnt_range: self.jnt_range,
            jnt_stiffness: self.jnt_stiffness,
            jnt_springref: self.jnt_springref,
            jnt_damping: self.jnt_damping,
            jnt_armature: self.jnt_armature,
            jnt_solref: self.jnt_solref,
            jnt_solimp: self.jnt_solimp,
            jnt_name: self.jnt_name,
            jnt_group: self.jnt_group,

            dof_body: self.dof_body,
            dof_jnt: self.dof_jnt,
            dof_parent: self.dof_parent,
            dof_armature: self.dof_armature,
            dof_damping: self.dof_damping,
            dof_frictionloss: self.dof_frictionloss,
            dof_solref: self.dof_solref,
            dof_solimp: self.dof_solimp,

            // Sparse LDL CSR metadata (computed below via compute_qld_csr_metadata)
            qLD_rowadr: vec![],
            qLD_rownnz: vec![],
            qLD_colind: vec![],
            qLD_nnz: 0,

            geom_type: self.geom_type,
            geom_body: self.geom_body,
            geom_pos: self.geom_pos,
            geom_quat: self.geom_quat,
            geom_size: self.geom_size,
            geom_friction: self.geom_friction,
            geom_condim: self.geom_condim,
            geom_contype: self.geom_contype,
            geom_conaffinity: self.geom_conaffinity,
            geom_margin: self.geom_margin,
            geom_gap: self.geom_gap,
            geom_priority: self.geom_priority,
            geom_solmix: self.geom_solmix,
            geom_solimp: self.geom_solimp,
            geom_solref: self.geom_solref,
            geom_name: self.geom_name,
            // Pre-computed bounding radii (computed below after we have geom_type and geom_size)
            geom_rbound: vec![0.0; ngeom],
            // Mesh index for each geom (populated by process_geom)
            geom_mesh: self.geom_mesh,
            // Hfield index for each geom (populated by process_geom)
            geom_hfield: self.geom_hfield,
            // SDF index for each geom (always None from MJCF — programmatic only)
            geom_sdf: self.geom_sdf,
            geom_group: self.geom_group,
            geom_rgba: self.geom_rgba,
            geom_fluid: self.geom_fluid,

            // Flex bodies
            nflex: self.nflex,
            nflexvert: self.nflexvert,
            nflexedge: self.nflexedge,
            nflexelem: self.nflexelem,
            nflexhinge: self.nflexhinge,
            flex_dim: self.flex_dim,
            flex_vertadr: self.flex_vertadr,
            flex_vertnum: self.flex_vertnum,
            flex_edgeadr,
            flex_edgenum,
            flex_elemadr,
            flex_elemnum,
            flex_young: self.flex_young,
            flex_poisson: self.flex_poisson,
            flex_damping: self.flex_damping,
            flex_thickness: self.flex_thickness,
            flex_friction: self.flex_friction,
            flex_solref: self.flex_solref,
            flex_solimp: self.flex_solimp,
            flex_condim: self.flex_condim,
            flex_margin: self.flex_margin,
            flex_gap: self.flex_gap,
            flex_priority: self.flex_priority,
            flex_solmix: self.flex_solmix,
            flex_contype: self.flex_contype,
            flex_conaffinity: self.flex_conaffinity,
            flex_selfcollide: self.flex_selfcollide,
            flex_edgestiffness: self.flex_edgestiffness,
            flex_edgedamping: self.flex_edgedamping,
            flex_edge_solref: self.flex_edge_solref,
            flex_edge_solimp: self.flex_edge_solimp,
            flex_bend_stiffness: self.flex_bend_stiffness,
            flex_bend_damping: self.flex_bend_damping,
            flex_density: self.flex_density,
            flex_group: self.flex_group,
            flexvert_qposadr: self.flexvert_qposadr,
            flexvert_dofadr: self.flexvert_dofadr,
            flexvert_mass: self.flexvert_mass,
            flexvert_invmass: self.flexvert_invmass,
            flexvert_radius: self.flexvert_radius,
            flexvert_flexid: self.flexvert_flexid,
            flexvert_bodyid: self.flexvert_bodyid,
            flexedge_vert: self.flexedge_vert,
            flexedge_length0: self.flexedge_length0,
            flexedge_crosssection,
            flexedge_flexid: self.flexedge_flexid,
            flexelem_data: self.flexelem_data,
            flexelem_dataadr: self.flexelem_dataadr,
            flexelem_datanum: self.flexelem_datanum,
            flexelem_volume0: self.flexelem_volume0,
            flexelem_flexid: self.flexelem_flexid,
            flexhinge_vert: self.flexhinge_vert,
            flexhinge_angle0: self.flexhinge_angle0,
            flexhinge_flexid: self.flexhinge_flexid,

            // Mesh assets (from MJCF <asset><mesh> elements)
            nmesh: self.mesh_data.len(),
            mesh_name: self.mesh_name,
            mesh_data: self.mesh_data,

            // Height field assets (from MJCF <asset><hfield> elements)
            nhfield: self.hfield_data.len(),
            hfield_name: self.hfield_name,
            hfield_data: self.hfield_data,
            hfield_size: self.hfield_size,

            // SDF assets (programmatic — always empty from MJCF)
            nsdf: 0,
            sdf_data: vec![],

            site_body: self.site_body,
            site_type: self.site_type,
            site_pos: self.site_pos,
            site_quat: self.site_quat,
            site_size: self.site_size,
            site_name: self.site_name,
            site_group: self.site_group,
            site_rgba: self.site_rgba,

            // Sensors (populated by process_sensors)
            nsensor: self.nsensor,
            nsensordata: self.nsensordata,
            sensor_type: self.sensor_type,
            sensor_datatype: self.sensor_datatype,
            sensor_objtype: self.sensor_objtype,
            sensor_objid: self.sensor_objid,
            sensor_reftype: self.sensor_reftype,
            sensor_refid: self.sensor_refid,
            sensor_adr: self.sensor_adr,
            sensor_dim: self.sensor_dim,
            sensor_noise: self.sensor_noise,
            sensor_cutoff: self.sensor_cutoff,
            sensor_name: self.sensor_name_list,

            actuator_trntype: self.actuator_trntype,
            actuator_dyntype: self.actuator_dyntype,
            actuator_trnid: self.actuator_trnid,
            actuator_gear: self.actuator_gear,
            actuator_ctrlrange: self.actuator_ctrlrange,
            actuator_forcerange: self.actuator_forcerange,
            actuator_name: self.actuator_name,
            actuator_act_adr: self.actuator_act_adr,
            actuator_act_num: self.actuator_act_num,
            actuator_gaintype: self.actuator_gaintype,
            actuator_biastype: self.actuator_biastype,
            actuator_dynprm: self.actuator_dynprm,
            actuator_gainprm: self.actuator_gainprm,
            actuator_biasprm: self.actuator_biasprm,
            actuator_lengthrange: self.actuator_lengthrange,
            actuator_acc0: self.actuator_acc0,
            actuator_actlimited: self.actuator_actlimited,
            actuator_actrange: self.actuator_actrange,
            actuator_actearly: self.actuator_actearly,

            // Tendons (populated by process_tendons)
            ntendon: self.tendon_type.len(),
            nwrap: self.wrap_type.len(),
            tendon_type: self.tendon_type,
            tendon_range: self.tendon_range,
            tendon_limited: self.tendon_limited,
            tendon_stiffness: self.tendon_stiffness,
            tendon_damping: self.tendon_damping,
            tendon_frictionloss: self.tendon_frictionloss,
            tendon_solref_fri: self.tendon_solref_fri,
            tendon_solimp_fri: self.tendon_solimp_fri,
            // tendon_treenum/tendon_tree computed below after tree enumeration
            tendon_treenum: vec![0; ntendon],
            tendon_tree: vec![usize::MAX; 2 * ntendon],
            tendon_lengthspring: self.tendon_lengthspring,
            tendon_length0: self.tendon_length0,
            tendon_num: self.tendon_num,
            tendon_adr: self.tendon_adr,
            tendon_name: self.tendon_name,
            tendon_group: self.tendon_group,
            tendon_rgba: self.tendon_rgba,
            tendon_solref: self.tendon_solref,
            tendon_solimp: self.tendon_solimp,
            wrap_type: self.wrap_type,
            wrap_objid: self.wrap_objid,
            wrap_prm: self.wrap_prm,
            wrap_sidesite: self.wrap_sidesite,

            // Equality constraints (populated by process_equality_constraints)
            neq: self.eq_type.len(),
            eq_type: self.eq_type,
            eq_obj1id: self.eq_obj1id,
            eq_obj2id: self.eq_obj2id,
            eq_data: self.eq_data,
            eq_active: self.eq_active,
            eq_solimp: self.eq_solimp,
            eq_solref: self.eq_solref,
            eq_name: self.eq_name,

            // Contact pairs / excludes (populated by process_contact)
            contact_pairs: self.contact_pairs,
            contact_pair_set: self.contact_pair_set,
            contact_excludes: self.contact_excludes,

            timestep: self.timestep,
            gravity: self.gravity,
            qpos0: DVector::from_vec(self.qpos0_values),
            keyframes: Vec::new(), // Populated post-build in model_from_mjcf()
            wind: self.wind,
            magnetic: self.magnetic,
            density: self.density,
            viscosity: self.viscosity,
            solver_iterations: self.solver_iterations,
            solver_tolerance: self.solver_tolerance,
            impratio: self.impratio,
            regularization: self.regularization,
            friction_smoothing: self.friction_smoothing,
            cone: self.cone,
            stat_meaninertia: 1.0, // Computed post-build in Step 3
            ls_iterations: self.ls_iterations,
            ls_tolerance: self.ls_tolerance,
            noslip_iterations: self.noslip_iterations,
            noslip_tolerance: self.noslip_tolerance,
            disableflags: self.disableflags,
            enableflags: self.enableflags,
            integrator: self.integrator,
            solver_type: self.solver_type,

            // Cached implicit integration parameters (computed below)
            implicit_stiffness: DVector::zeros(self.nv),
            implicit_damping: DVector::zeros(self.nv),
            implicit_springref: DVector::zeros(self.nv),

            // Pre-computed kinematic data (will be populated by compute_ancestors)
            body_ancestor_joints: vec![vec![]; nbody],
            body_ancestor_mask: vec![vec![]; nbody], // Multi-word bitmask, computed by compute_ancestors
        };

        // Pre-compute ancestor lists for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters (K, D, q_eq diagonals)
        model.compute_implicit_params();

        // Pre-compute CSR sparsity metadata for sparse LDL factorization.
        // Must be called after dof_parent is finalized and before anything that
        // calls mj_crba (which uses mj_factor_sparse).
        model.compute_qld_csr_metadata();

        // Compute tendon_length0 for spatial tendons (requires FK via mj_fwd_position).
        // Must run before compute_muscle_params() which needs valid tendon_length0.
        model.compute_spatial_tendon_length0();

        // Pre-compute muscle-derived parameters (lengthrange, acc0, F0)
        model.compute_muscle_params();

        // Compute stat_meaninertia = trace(M) / nv at qpos0 (for Newton solver scaling, §15.11)
        model.compute_stat_meaninertia();

        // Pre-compute bounding sphere radii for all geoms (used in collision broad-phase)
        for geom_id in 0..ngeom {
            model.geom_rbound[geom_id] = if let Some(mesh_id) = model.geom_mesh[geom_id] {
                // Mesh geom: bounding sphere radius = distance from AABB center to corner
                // This is the half-diagonal of the AABB, guaranteeing all vertices are inside.
                let (aabb_min, aabb_max) = model.mesh_data[mesh_id].aabb();
                let half_diagonal = (aabb_max - aabb_min) / 2.0;
                half_diagonal.norm()
            } else if let Some(hfield_id) = model.geom_hfield[geom_id] {
                // Hfield geom: half-diagonal of AABB (same pattern as mesh).
                // HeightFieldData::aabb() returns corner-origin bounds; the half-diagonal
                // is origin-independent so the centering offset does not affect it.
                let (aabb_min, aabb_max) = model.hfield_data[hfield_id].aabb();
                let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
                half_diagonal.norm()
            } else if let Some(sdf_id) = model.geom_sdf[geom_id] {
                // SDF geom: half-diagonal of AABB (same pattern as mesh/hfield).
                // SdfCollisionData::aabb() returns (Point3, Point3).
                let (aabb_min, aabb_max) = model.sdf_data[sdf_id].aabb();
                let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
                half_diagonal.norm()
            } else {
                // Primitive geom: use GeomType::bounding_radius() - the single source of truth
                model.geom_type[geom_id].bounding_radius(model.geom_size[geom_id])
            };
        }

        // Pre-compute tendon length0 and lengthspring from qpos0.
        // For fixed tendons: length = Σ coef_w * qpos0[dof_adr_w].
        for t in 0..model.ntendon {
            if model.tendon_type[t] == TendonType::Fixed {
                let adr = model.tendon_adr[t];
                let num = model.tendon_num[t];
                let mut length = 0.0;
                for w in adr..(adr + num) {
                    let dof_adr = model.wrap_objid[w];
                    let coef = model.wrap_prm[w];
                    if dof_adr < model.qpos0.len() {
                        length += coef * model.qpos0[dof_adr];
                    }
                }
                model.tendon_length0[t] = length;
                // S3: Replace sentinel [-1, -1] with computed length at qpos0
                // (MuJoCo uses qpos_spring; see S3 divergence note).
                // Unconditional — MuJoCo resolves sentinel regardless of stiffness.
                // Sentinel is an exact literal, never a computed float.
                #[allow(clippy::float_cmp)]
                if model.tendon_lengthspring[t] == [-1.0, -1.0] {
                    model.tendon_lengthspring[t] = [length, length];
                }
            }
        }

        // ===== Kinematic Tree Enumeration (§16.0) =====
        // Group bodies by body_rootid to discover kinematic trees.
        // Body 0 (world) is excluded — it is its own tree but never sleeps.
        {
            use std::collections::BTreeMap;
            let mut trees: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for body_id in 1..nbody {
                trees
                    .entry(model.body_rootid[body_id])
                    .or_default()
                    .push(body_id);
            }
            model.ntree = trees.len();
            model.tree_body_adr = Vec::with_capacity(model.ntree);
            model.tree_body_num = Vec::with_capacity(model.ntree);
            model.tree_dof_adr = Vec::with_capacity(model.ntree);
            model.tree_dof_num = Vec::with_capacity(model.ntree);
            model.tree_sleep_policy = vec![SleepPolicy::Auto; model.ntree];

            // body_treeid[0] = usize::MAX (world sentinel, already set)
            for (tree_idx, (_root_body, body_ids)) in trees.iter().enumerate() {
                let first_body = body_ids[0];
                let body_count = body_ids.len();
                model.tree_body_adr.push(first_body);
                model.tree_body_num.push(body_count);

                // Assign tree id to each body
                for &bid in body_ids {
                    model.body_treeid[bid] = tree_idx;
                }

                // DOF range: find min dof and total DOFs for this tree
                let mut min_dof = model.nv;
                let mut total_dofs = 0usize;
                for &bid in body_ids {
                    let dof_start = model.body_dof_adr[bid];
                    let dof_count = model.body_dof_num[bid];
                    if dof_count > 0 && dof_start < min_dof {
                        min_dof = dof_start;
                    }
                    total_dofs += dof_count;
                }
                if total_dofs == 0 {
                    min_dof = 0; // Bodyless tree (e.g., static geoms)
                }
                model.tree_dof_adr.push(min_dof);
                model.tree_dof_num.push(total_dofs);

                // Assign tree id to each DOF
                for &bid in body_ids {
                    let dof_start = model.body_dof_adr[bid];
                    let dof_count = model.body_dof_num[bid];
                    for dof in dof_start..(dof_start + dof_count) {
                        model.dof_treeid[dof] = tree_idx;
                    }
                }
            }

            // (§27F) Flex vertex DOFs are now real body DOFs in the kinematic tree.
            // Their tree IDs are set by the standard tree-building loop above.
            // Pinned vertices (dofadr == usize::MAX) have no DOFs — nothing to set.

            // ===== Tendon Tree Mapping (§16.10.1) =====
            // Compute tendon_treenum/tendon_tree by scanning each tendon's waypoints.
            for t in 0..model.ntendon {
                let mut tree_set = std::collections::BTreeSet::new();
                let adr = model.tendon_adr[t];
                let num = model.tendon_num[t];
                for w in adr..adr + num {
                    let bid = match model.wrap_type[w] {
                        WrapType::Joint => {
                            let dof_adr = model.wrap_objid[w];
                            if dof_adr < model.nv {
                                Some(model.dof_body[dof_adr])
                            } else {
                                None
                            }
                        }
                        WrapType::Site => {
                            let site_idx = model.wrap_objid[w];
                            if site_idx < model.nsite {
                                Some(model.site_body[site_idx])
                            } else {
                                None
                            }
                        }
                        WrapType::Geom => {
                            let geom_id = model.wrap_objid[w];
                            if geom_id < model.ngeom {
                                Some(model.geom_body[geom_id])
                            } else {
                                None
                            }
                        }
                        WrapType::Pulley => None,
                    };
                    if let Some(bid) = bid {
                        if bid > 0 {
                            let tree = model.body_treeid[bid];
                            if tree < model.ntree {
                                tree_set.insert(tree);
                            }
                        }
                    }
                }
                model.tendon_treenum[t] = tree_set.len();
                if !tree_set.is_empty() {
                    let mut iter = tree_set.iter();
                    if let Some(&a) = iter.next() {
                        model.tendon_tree[2 * t] = a;
                    }
                    if let Some(&b) = iter.next() {
                        model.tendon_tree[2 * t + 1] = b;
                    }
                }
            }

            // ===== Sleep Policy Resolution (§16.0 steps 1-3) =====
            // Step 2: Mark trees with actuators as AutoNever
            for act_id in 0..nu {
                let trn = model.actuator_trntype[act_id];
                let trnid = model.actuator_trnid[act_id];
                let body_id = match trn {
                    ActuatorTransmission::Joint => {
                        // trnid[0] is the joint index
                        if trnid[0] < model.njnt {
                            Some(model.jnt_body[trnid[0]])
                        } else {
                            None
                        }
                    }
                    ActuatorTransmission::Tendon => {
                        // §16.26.5: Tendon-actuator policy resolution.
                        // Mark all trees spanned by the tendon as AutoNever.
                        let tendon_idx = trnid[0];
                        if tendon_idx < model.ntendon {
                            let wrap_start = model.tendon_adr[tendon_idx];
                            let wrap_count = model.tendon_num[tendon_idx];
                            for w in wrap_start..wrap_start + wrap_count {
                                let bid = match model.wrap_type[w] {
                                    WrapType::Joint => {
                                        // wrap_objid is dof_adr for fixed tendon joints
                                        let dof_adr = model.wrap_objid[w];
                                        if dof_adr < model.nv {
                                            Some(model.dof_body[dof_adr])
                                        } else {
                                            None
                                        }
                                    }
                                    WrapType::Site => {
                                        let site_idx = model.wrap_objid[w];
                                        if site_idx < model.nsite {
                                            Some(model.site_body[site_idx])
                                        } else {
                                            None
                                        }
                                    }
                                    WrapType::Geom => {
                                        let geom_id = model.wrap_objid[w];
                                        if geom_id < model.ngeom {
                                            Some(model.geom_body[geom_id])
                                        } else {
                                            None
                                        }
                                    }
                                    WrapType::Pulley => None, // No body
                                };
                                if let Some(bid) = bid {
                                    if bid > 0 {
                                        let tree = model.body_treeid[bid];
                                        if tree < model.ntree {
                                            model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                                        }
                                    }
                                }
                            }
                        }
                        None // body_id not used below — trees already marked above
                    }
                    ActuatorTransmission::Site => {
                        // trnid[0] is the site index
                        if trnid[0] < model.nsite {
                            Some(model.site_body[trnid[0]])
                        } else {
                            None
                        }
                    }
                    ActuatorTransmission::Body => {
                        // trnid[0] is the body index directly
                        let bid = trnid[0];
                        if bid > 0 && bid < model.nbody {
                            Some(bid)
                        } else {
                            None
                        }
                    }
                };
                if let Some(bid) = body_id {
                    if bid > 0 {
                        let tree = model.body_treeid[bid];
                        if tree < model.ntree {
                            model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                        }
                    }
                }
            }

            // §16.18.2: Multi-tree tendon policy relaxation.
            // Passive multi-tree tendons (spanning 2 trees) with nonzero stiffness,
            // damping, or active limits create inter-tree coupling forces that prevent
            // independent sleeping. Mark their spanning trees as AutoNever.
            // Zero-stiffness/zero-damping/unlimited tendons are purely geometric
            // (observational) and allow sleep.
            for t in 0..model.ntendon {
                if model.tendon_treenum[t] < 2 {
                    continue; // Single-tree or zero-tree tendon — no coupling
                }
                let has_stiffness = model.tendon_stiffness[t].abs() > 0.0;
                let has_damping = model.tendon_damping[t].abs() > 0.0;
                let has_limit = model.tendon_limited[t];
                if has_stiffness || has_damping || has_limit {
                    // This tendon creates passive inter-tree forces → AutoNever
                    let t1 = model.tendon_tree[2 * t];
                    let t2 = model.tendon_tree[2 * t + 1];
                    if t1 < model.ntree && model.tree_sleep_policy[t1] == SleepPolicy::Auto {
                        model.tree_sleep_policy[t1] = SleepPolicy::AutoNever;
                    }
                    if t2 < model.ntree && model.tree_sleep_policy[t2] == SleepPolicy::Auto {
                        model.tree_sleep_policy[t2] = SleepPolicy::AutoNever;
                    }
                }
            }

            // §16.26.4: Flex body policy guard. Flex DOFs are assigned
            // dof_treeid = usize::MAX (permanently awake) so they don't participate
            // in the tree-level sleep system. If flex vertices are ever assigned to
            // specific kinematic trees, their trees should be marked AutoNever.

            // Step 2b: Apply explicit body-level sleep policies from MJCF
            for body_id in 1..nbody {
                if let Some(policy) = self.body_sleep_policy[body_id] {
                    let tree = model.body_treeid[body_id];
                    if tree < model.ntree {
                        // Warn if set on non-root body (propagates to tree root)
                        let root_body = model.tree_body_adr[tree];
                        if body_id != root_body {
                            let body_name =
                                model.body_name[body_id].as_deref().unwrap_or("unnamed");
                            warn!(
                                "body '{}': sleep attribute on non-root body propagates to tree root",
                                body_name
                            );
                        }
                        // Explicit policy overrides automatic resolution
                        model.tree_sleep_policy[tree] = policy;
                    }
                }
            }

            // Step 3: Convert remaining Auto to AutoAllowed
            for t in 0..model.ntree {
                if model.tree_sleep_policy[t] == SleepPolicy::Auto {
                    model.tree_sleep_policy[t] = SleepPolicy::AutoAllowed;
                }
            }

            // ===== dof_length Computation (§16.14) =====
            // Compute mechanism lengths: rotational DOFs get the body's subtree
            // extent (converts rad/s to m/s at tip), translational DOFs get 1.0.
            compute_dof_lengths(&mut model);

            // ===== RK4 Incompatibility Guard (§16.6) =====
            if model.enableflags & ENABLE_SLEEP != 0 && model.integrator == Integrator::RungeKutta4
            {
                warn!("Sleeping is incompatible with RK4 integrator. Disabling sleep.");
                model.enableflags &= !ENABLE_SLEEP;
            }
        }

        model
    }
}

// ============================================================================
// <general> actuator helpers
// ============================================================================

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

// quat_from_wxyz, quat_to_wxyz, euler_seq_to_quat, resolve_orientation → crate::builder::orientation (Phase 10)
#[cfg(test)]
use crate::builder::orientation::euler_seq_to_quat;

// frame_accum_child, validate_childclass_references, validate_frame_childclass_refs,
// expand_frames, expand_single_frame → crate::builder::frame (Phase 10)

// apply_discardvisual, apply_fusestatic, fuse_static_body → crate::builder::compiler (Phase 10)

// =============================================================================
// Mesh File Loading
// =============================================================================

// AssetKind, resolve_asset_path → crate::builder::asset (Phase 10)

// load_mesh_file, convert_mjcf_hfield, convert_mjcf_mesh, convert_embedded_mesh,
// compute_mesh_inertia → crate::builder::mesh (Phase 10)
// extract_inertial_properties, compute_inertia_from_geoms, apply_mass_pipeline
// → crate::builder::mass (Phase 10)

#[cfg(test)]
use crate::builder::compiler::apply_fusestatic;
#[cfg(test)]
use crate::builder::geom::compute_geom_inertia;
#[cfg(test)]
use crate::builder::mesh::{convert_mjcf_mesh, load_mesh_file};
#[cfg(test)]
use crate::types::{MjcfCompiler, MjcfGeom, MjcfGeomType, MjcfMesh};

/// Map `MjcfSensorType` to pipeline `MjSensorType`.
/// Returns `None` for types not yet implemented in the pipeline.
fn convert_sensor_type(mjcf: MjcfSensorType) -> Option<MjSensorType> {
    match mjcf {
        MjcfSensorType::Jointpos => Some(MjSensorType::JointPos),
        MjcfSensorType::Jointvel => Some(MjSensorType::JointVel),
        MjcfSensorType::Ballquat => Some(MjSensorType::BallQuat),
        MjcfSensorType::Ballangvel => Some(MjSensorType::BallAngVel),
        MjcfSensorType::Tendonpos => Some(MjSensorType::TendonPos),
        MjcfSensorType::Tendonvel => Some(MjSensorType::TendonVel),
        MjcfSensorType::Actuatorpos => Some(MjSensorType::ActuatorPos),
        MjcfSensorType::Actuatorvel => Some(MjSensorType::ActuatorVel),
        MjcfSensorType::Actuatorfrc => Some(MjSensorType::ActuatorFrc),
        MjcfSensorType::Framepos => Some(MjSensorType::FramePos),
        MjcfSensorType::Framequat => Some(MjSensorType::FrameQuat),
        MjcfSensorType::Framexaxis => Some(MjSensorType::FrameXAxis),
        MjcfSensorType::Frameyaxis => Some(MjSensorType::FrameYAxis),
        MjcfSensorType::Framezaxis => Some(MjSensorType::FrameZAxis),
        MjcfSensorType::Framelinvel => Some(MjSensorType::FrameLinVel),
        MjcfSensorType::Frameangvel => Some(MjSensorType::FrameAngVel),
        MjcfSensorType::Framelinacc => Some(MjSensorType::FrameLinAcc),
        MjcfSensorType::Frameangacc => Some(MjSensorType::FrameAngAcc),
        MjcfSensorType::Touch => Some(MjSensorType::Touch),
        MjcfSensorType::Force => Some(MjSensorType::Force),
        MjcfSensorType::Torque => Some(MjSensorType::Torque),
        MjcfSensorType::Accelerometer => Some(MjSensorType::Accelerometer),
        MjcfSensorType::Gyro => Some(MjSensorType::Gyro),
        MjcfSensorType::Velocimeter => Some(MjSensorType::Velocimeter),
        MjcfSensorType::Magnetometer => Some(MjSensorType::Magnetometer),
        MjcfSensorType::Rangefinder => Some(MjSensorType::Rangefinder),
        MjcfSensorType::Subtreecom => Some(MjSensorType::SubtreeCom),
        MjcfSensorType::Subtreelinvel => Some(MjSensorType::SubtreeLinVel),
        MjcfSensorType::Subtreeangmom => Some(MjSensorType::SubtreeAngMom),
        MjcfSensorType::User => Some(MjSensorType::User),
        MjcfSensorType::Jointlimitfrc => Some(MjSensorType::JointLimitFrc),
        MjcfSensorType::Tendonlimitfrc => Some(MjSensorType::TendonLimitFrc),
    }
}

/// Determine which evaluation stage processes a given sensor type.
fn sensor_datatype(t: MjSensorType) -> MjSensorDataType {
    match t {
        // Position stage (evaluated in mj_sensor_pos, after FK)
        MjSensorType::JointPos
        | MjSensorType::BallQuat
        | MjSensorType::TendonPos
        | MjSensorType::ActuatorPos
        | MjSensorType::FramePos
        | MjSensorType::FrameQuat
        | MjSensorType::FrameXAxis
        | MjSensorType::FrameYAxis
        | MjSensorType::FrameZAxis
        | MjSensorType::SubtreeCom
        | MjSensorType::Rangefinder
        | MjSensorType::Magnetometer => MjSensorDataType::Position,

        // Velocity stage (evaluated in mj_sensor_vel, after velocity FK)
        MjSensorType::JointVel
        | MjSensorType::BallAngVel
        | MjSensorType::TendonVel
        | MjSensorType::ActuatorVel
        | MjSensorType::Gyro
        | MjSensorType::Velocimeter
        | MjSensorType::FrameLinVel
        | MjSensorType::FrameAngVel
        | MjSensorType::SubtreeLinVel
        | MjSensorType::SubtreeAngMom => MjSensorDataType::Velocity,

        // Acceleration stage (evaluated in mj_sensor_acc, after constraint solver).
        // User sensors also default to acceleration (evaluated last).
        MjSensorType::Accelerometer
        | MjSensorType::Force
        | MjSensorType::Torque
        | MjSensorType::Touch
        | MjSensorType::ActuatorFrc
        | MjSensorType::JointLimitFrc
        | MjSensorType::TendonLimitFrc
        | MjSensorType::FrameLinAcc
        | MjSensorType::FrameAngAcc
        | MjSensorType::User => MjSensorDataType::Acceleration,
    }
}

// ============================================================================
// Flex helper functions
// ============================================================================

/// Compute per-edge cross-section area based on flex dimensionality.
///
/// - dim=1 (cable): PI * radius^2
/// - dim=2 (shell): thickness * rest_length (dual edge cross-section)
/// - dim=3 (solid): rest_length^2 (L^2 approximation)
fn compute_flexedge_crosssection(
    flexedge_flexid: &[usize],
    flexedge_length0: &[f64],
    flex_dim: &[usize],
    flex_thickness: &[f64],
    flexvert_radius: &[f64],
    flex_vertadr: &[usize],
) -> Vec<f64> {
    let nedge = flexedge_flexid.len();
    let mut crosssection = vec![1.0; nedge];
    for i in 0..nedge {
        let fid = flexedge_flexid[i];
        if fid >= flex_dim.len() {
            continue;
        }
        let dim = flex_dim[fid];
        let rest_len = flexedge_length0[i];
        crosssection[i] = match dim {
            1 => {
                // Cable: PI * radius^2
                let radius = if fid < flex_vertadr.len() {
                    let vert0 = flex_vertadr[fid];
                    if vert0 < flexvert_radius.len() {
                        flexvert_radius[vert0]
                    } else {
                        0.005
                    }
                } else {
                    0.005
                };
                std::f64::consts::PI * radius * radius
            }
            2 => {
                // Shell: thickness * rest_length
                let thickness = flex_thickness[fid];
                thickness * rest_len
            }
            3 => {
                // Solid: L^2 approximation
                rest_len * rest_len
            }
            _ => 1.0,
        };
    }
    crosssection
}

/// Compute per-flex address table from a per-item `flexid` array.
///
/// Returns a Vec of length `nflex` where `result[i]` is the index of the first
/// item belonging to flex `i` in the corresponding data array.
fn compute_flex_address_table(item_flexid: &[usize], nflex: usize) -> Vec<usize> {
    let mut adr = vec![0usize; nflex];
    let mut offset = 0;
    for (flex_id, a) in adr.iter_mut().enumerate() {
        *a = offset;
        offset += item_flexid.iter().filter(|&&fid| fid == flex_id).count();
    }
    adr
}

/// Compute per-flex count table from a per-item `flexid` array.
///
/// Returns a Vec of length `nflex` where `result[i]` is the number of items
/// belonging to flex `i`.
fn compute_flex_count_table(item_flexid: &[usize], nflex: usize) -> Vec<usize> {
    let mut count = vec![0usize; nflex];
    for &fid in item_flexid {
        if fid < nflex {
            count[fid] += 1;
        }
    }
    count
}

/// Compute per-vertex masses.
///
/// Two paths:
/// 1. **Uniform mass** (`flex.mass` is `Some`): distribute `mass / npnt` to every vertex
///    (including pinned). MuJoCo conformant — pinned share is silently discarded downstream.
/// 2. **Element-based lumping** (fallback): density * element_measure / vertices_per_element.
fn compute_vertex_masses(flex: &MjcfFlex) -> Vec<f64> {
    let npnt = flex.vertices.len();
    if npnt == 0 {
        return Vec::new();
    }

    // Path 1: uniform mass from <flexcomp mass="...">
    // No min-mass floor: mass=0 → per_vert=0 → invmass=0 (static vertices).
    // This matches MuJoCo semantics where explicit mass=0 pins vertices.
    if let Some(total_mass) = flex.mass {
        #[allow(clippy::cast_precision_loss)] // npnt is vertex count, never near 2^52
        let per_vert = total_mass / npnt as f64;
        return vec![per_vert; npnt];
    }

    // Path 2: element-based mass lumping from density
    let mut masses = vec![0.0f64; npnt];

    match flex.dim {
        1 => {
            let density = flex.density;
            for elem in &flex.elements {
                if elem.len() >= 2 {
                    let len = (flex.vertices[elem[1]] - flex.vertices[elem[0]]).norm();
                    let mass_per_vert = density * len / 2.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                }
            }
        }
        2 => {
            let area_density = flex.density * flex.thickness;
            for elem in &flex.elements {
                if elem.len() >= 3 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let area = e1.cross(&e2).norm() / 2.0;
                    let mass_per_vert = area_density * area / 3.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                    masses[elem[2]] += mass_per_vert;
                }
            }
        }
        3 => {
            let density = flex.density;
            for elem in &flex.elements {
                if elem.len() >= 4 {
                    let e1 = flex.vertices[elem[1]] - flex.vertices[elem[0]];
                    let e2 = flex.vertices[elem[2]] - flex.vertices[elem[0]];
                    let e3 = flex.vertices[elem[3]] - flex.vertices[elem[0]];
                    let volume = e1.dot(&e2.cross(&e3)).abs() / 6.0;
                    let mass_per_vert = density * volume / 4.0;
                    masses[elem[0]] += mass_per_vert;
                    masses[elem[1]] += mass_per_vert;
                    masses[elem[2]] += mass_per_vert;
                    masses[elem[3]] += mass_per_vert;
                }
            }
        }
        _ => {}
    }

    // Ensure minimum mass
    for m in &mut masses {
        if *m < 1e-10 {
            *m = 0.001;
        }
    }

    masses
}

/// Compute dihedral angle between two triangles sharing an edge.
fn compute_dihedral_angle(
    pe0: Vector3<f64>,
    pe1: Vector3<f64>,
    pa: Vector3<f64>,
    pb: Vector3<f64>,
) -> f64 {
    let e = pe1 - pe0;
    let e_len_sq = e.norm_squared();
    if e_len_sq < 1e-20 {
        return 0.0;
    }
    let offset_a = pa - pe0;
    let offset_b = pb - pe0;
    let normal_face_a = e.cross(&offset_a);
    let normal_face_b = offset_b.cross(&e);

    let norm_sq_a = normal_face_a.norm_squared();
    let norm_sq_b = normal_face_b.norm_squared();
    if norm_sq_a < 1e-20 || norm_sq_b < 1e-20 {
        return 0.0;
    }

    let e_dir = e / e_len_sq.sqrt();
    let normal_unit_a = normal_face_a / norm_sq_a.sqrt();
    let normal_unit_b = normal_face_b / norm_sq_b.sqrt();
    let cos_theta = normal_unit_a.dot(&normal_unit_b).clamp(-1.0, 1.0);
    let sin_theta = normal_unit_a.cross(&normal_unit_b).dot(&e_dir);
    sin_theta.atan2(cos_theta)
}

/// Compute per-flex edge constraint solref.
///
/// MuJoCo architecture note: Edge constraint solref comes from the equality
/// constraint definition (eq_solref), not from material properties. The
/// `<edge stiffness="..." damping="..."/>` attributes control passive spring-
/// damper forces (applied in `mj_fwd_passive`), not constraint solver params.
///
/// Our current architecture uses flex-level solref for all edge constraints,
/// which matches MuJoCo's behavior when no explicit equality constraint
/// overrides are defined.
fn compute_edge_solref(flex: &MjcfFlex) -> [f64; 2] {
    flex.solref
}

/// Compute bending stiffness from material properties (passive force coefficient).
///
/// For dim=2 (shells): Kirchhoff-Love thin plate bending stiffness
///   D = E * t^3 / (12 * (1 - nu^2))
/// For dim=3 (solids): characteristic stiffness D = E
///   (the Bridson dihedral gradient provides geometric scaling)
/// For dim=1 (cables): 0.0 (no bending)
fn compute_bend_stiffness_from_material(flex: &MjcfFlex) -> f64 {
    if flex.young <= 0.0 {
        return 0.0;
    }
    match flex.dim {
        2 => {
            let nu = flex.poisson.clamp(0.0, 0.499);
            flex.young * flex.thickness.powi(3) / (12.0 * (1.0 - nu * nu))
        }
        3 => flex.young,
        _ => 0.0,
    }
}

/// Compute bending damping from material properties.
///
/// Proportional damping model: b_bend = damping * k_bend.
fn compute_bend_damping_from_material(flex: &MjcfFlex, k_bend: f64) -> f64 {
    if flex.damping <= 0.0 || k_bend <= 0.0 {
        return 0.0;
    }
    flex.damping * k_bend
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="pendulum">
                <option timestep="0.001" gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 2); // world + link1
        assert_eq!(model.njnt, 1);
        assert_eq!(model.nq, 1); // hinge has 1 qpos
        assert_eq!(model.nv, 1); // hinge has 1 DOF
        assert_eq!(model.ngeom, 1);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world

        // Check joint
        assert_eq!(model.jnt_type[0], MjJointType::Hinge);
        assert_eq!(model.jnt_body[0], 1); // joint on link1

        // Check options
        assert!((model.timestep - 0.001).abs() < 1e-10);
        assert!((model.gravity.z - (-9.81)).abs() < 1e-10);
    }

    #[test]
    fn test_double_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="double_pendulum">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.25"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                            <geom type="capsule" size="0.05 0.25"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 3); // world + link1 + link2
        assert_eq!(model.njnt, 2);
        assert_eq!(model.nq, 2);
        assert_eq!(model.nv, 2);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world
        assert_eq!(model.body_parent[2], 1); // link2's parent is link1
    }

    #[test]
    fn test_with_actuator() {
        let model = load_model(
            r#"
            <mujoco model="actuated">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="shoulder" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="shoulder" gear="100"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Joint);
        assert_eq!(model.actuator_trnid[0][0], 0); // First joint
        assert_eq!(model.actuator_trnid[0][1], usize::MAX); // No refsite
        assert!((model.actuator_gear[0][0] - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="floating">
                <worldbody>
                    <body name="ball" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 7); // 3 pos + 4 quat
        assert_eq!(model.nv, 6); // 3 linear + 3 angular
        assert_eq!(model.jnt_type[0], MjJointType::Free);
    }

    #[test]
    fn test_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 4); // quaternion
        assert_eq!(model.nv, 3); // 3 angular DOFs
        assert_eq!(model.jnt_type[0], MjJointType::Ball);
    }

    /// Test that dof_parent forms correct kinematic tree for simple chain.
    ///
    /// For a 2-link pendulum (world → link1 → link2), each with 1 hinge:
    /// - DOF 0 (link1's hinge): parent = None (attached to world)
    /// - DOF 1 (link2's hinge): parent = Some(0) (attached to link1's DOF)
    #[test]
    fn test_dof_parent_chain() {
        let model = load_model(
            r#"
            <mujoco model="chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        // DOF 0: link1's hinge, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: link2's hinge, parent is DOF 0 (link1's hinge)
        assert_eq!(model.dof_parent[1], Some(0));
    }

    /// Test dof_parent for multi-DOF joints (ball joint).
    ///
    /// For a ball joint (3 DOFs):
    /// - DOF 0: parent = parent body's last DOF (or None)
    /// - DOF 1: parent = DOF 0
    /// - DOF 2: parent = DOF 1
    #[test]
    fn test_dof_parent_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);
        // DOF 0: first DOF of ball, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: second DOF of ball, parent is DOF 0
        assert_eq!(model.dof_parent[1], Some(0));
        // DOF 2: third DOF of ball, parent is DOF 1
        assert_eq!(model.dof_parent[2], Some(1));
    }

    /// Test dof_parent for free joint (6 DOFs).
    #[test]
    fn test_dof_parent_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="free_body">
                <worldbody>
                    <body name="floating" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 6);
        // DOF 0: first DOF of free, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOFs 1-5: chain within free joint
        for i in 1..6 {
            assert_eq!(model.dof_parent[i], Some(i - 1));
        }
    }

    /// Test dof_parent for chain with mixed joint types.
    ///
    /// world → link1 (ball, 3 DOF) → link2 (hinge, 1 DOF)
    /// - DOFs 0,1,2: ball joint, parents: None, 0, 1
    /// - DOF 3: hinge, parent is DOF 2 (last DOF of ball)
    #[test]
    fn test_dof_parent_mixed_joints() {
        let model = load_model(
            r#"
            <mujoco model="mixed">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="ball" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="hinge" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 4); // 3 (ball) + 1 (hinge)

        // Ball joint DOFs
        assert_eq!(model.dof_parent[0], None);
        assert_eq!(model.dof_parent[1], Some(0));
        assert_eq!(model.dof_parent[2], Some(1));

        // Hinge DOF: parent is ball's last DOF
        assert_eq!(model.dof_parent[3], Some(2));
    }

    /// Test dof_parent for branching tree (two children of same parent).
    ///
    /// world → base (hinge) ─┬→ left_arm (hinge)
    ///                       └→ right_arm (hinge)
    ///
    /// DOF 0: base
    /// DOF 1: left_arm, parent = 0
    /// DOF 2: right_arm, parent = 0
    #[test]
    fn test_dof_parent_branching() {
        let model = load_model(
            r#"
            <mujoco model="branching">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <joint name="j_base" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="2.0" diaginertia="0.2 0.2 0.1"/>
                        <body name="left_arm" pos="0.5 0 0">
                            <joint name="j_left" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                        <body name="right_arm" pos="-0.5 0 0">
                            <joint name="j_right" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);

        // Base joint: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Both arms attach to base
        assert_eq!(model.dof_parent[1], Some(0)); // left_arm
        assert_eq!(model.dof_parent[2], Some(0)); // right_arm
    }

    /// Test dof_parent for body with multiple joints (uncommon but valid).
    ///
    /// In MuJoCo, a single body can have multiple joints. The DOFs chain:
    /// world → link (hinge1, hinge2)
    /// - DOF 0: hinge1, parent = None
    /// - DOF 1: hinge2, parent = DOF 0
    #[test]
    fn test_dof_parent_multiple_joints_one_body() {
        let model = load_model(
            r#"
            <mujoco model="multi_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="1 0 0"/>
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        assert_eq!(model.njnt, 2);

        // First joint's DOF: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Second joint's DOF: attached to first joint
        assert_eq!(model.dof_parent[1], Some(0));
    }

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

    /// Test capsule inertia computation is physically reasonable.
    #[test]
    fn test_capsule_inertia() {
        // A capsule should have I_z < I_x = I_y (axially symmetric, longer than wide)
        let geom = MjcfGeom {
            name: None,
            class: None,
            geom_type: Some(MjcfGeomType::Capsule),
            pos: Some(Vector3::zeros()),
            quat: Some(nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0)),
            euler: None,
            axisangle: None,
            xyaxes: None,
            zaxis: None,
            size: vec![0.1, 0.5], // radius=0.1, half-height=0.5
            fromto: None,
            density: Some(1000.0),
            mass: None,
            friction: Some(Vector3::new(1.0, 0.005, 0.0001)),
            rgba: Some(nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0)),
            contype: Some(1),
            conaffinity: Some(1),
            condim: Some(3),
            mesh: None,
            hfield: None,
            solref: None,
            solimp: None,
            priority: Some(0),
            solmix: Some(1.0),
            margin: Some(0.0),
            gap: Some(0.0),
            group: Some(0),
            material: None,
            fluidshape: None,
            fluidcoef: None,
        };

        let inertia = compute_geom_inertia(&geom, None);

        // Ix = Iy (axially symmetric) — diagonal elements (0,0) and (1,1)
        assert!((inertia[(0, 0)] - inertia[(1, 1)]).abs() < 1e-10);

        // Iz < Ix (thin cylinder is easier to spin about long axis)
        assert!(inertia[(2, 2)] < inertia[(0, 0)]);

        // All positive
        assert!(inertia[(0, 0)] > 0.0);
        assert!(inertia[(1, 1)] > 0.0);
        assert!(inertia[(2, 2)] > 0.0);
    }

    /// Test site parsing and model population.
    #[test]
    fn test_sites() {
        let model = load_model(
            r#"
            <mujoco model="site_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="capsule" size="0.05 0.5"/>
                        <site name="end_effector" pos="0 0 -1" size="0.02"/>
                        <site name="sensor_mount" pos="0.1 0 0" size="0.01 0.01 0.02"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nsite, 2);
        assert_eq!(model.site_body.len(), 2);

        // Check first site (end_effector)
        assert_eq!(model.site_body[0], 1); // Attached to arm (body 1)
        assert!((model.site_pos[0].z - (-1.0)).abs() < 1e-10);
        assert_eq!(model.site_name[0], Some("end_effector".to_string()));

        // Check second site (sensor_mount)
        assert_eq!(model.site_body[1], 1);
        assert!((model.site_pos[1].x - 0.1).abs() < 1e-10);
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

    // resolve_asset_path tests → builder/asset.rs

    // =========================================================================
    // Mesh file loading tests
    // =========================================================================

    /// Helper to create a simple STL file for testing.
    fn create_test_stl(path: &std::path::Path) {
        use mesh_types::{IndexedMesh, Vertex};

        let mut mesh = IndexedMesh::new();
        // Simple tetrahedron (4 vertices, 4 faces)
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]); // base
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left

        mesh_io::save_mesh(&mesh, path).expect("Failed to save test STL");
    }

    /// Test load_mesh_file with STL format.
    #[test]
    fn test_load_mesh_file_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        create_test_stl(&mesh_path);

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "test_mesh",
        );

        assert!(result.is_ok(), "Should load STL file");
        let mesh_data = result.unwrap();
        assert!(
            mesh_data.vertices().len() >= 4,
            "Should have at least 4 vertices"
        );
        assert!(
            mesh_data.triangles().len() >= 4,
            "Should have at least 4 triangles"
        );
    }

    /// Test load_mesh_file applies scale correctly.
    #[test]
    fn test_load_mesh_file_with_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("scaled.stl");
        create_test_stl(&mesh_path);

        // Load without scale
        let unscaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unscaled",
        )
        .unwrap();

        // Load with 2x scale
        let scaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(2.0, 2.0, 2.0),
            "scaled",
        )
        .unwrap();

        // Verify scale was applied: scaled vertices should be 2x unscaled
        // Find corresponding vertices and compare
        assert_eq!(scaled.vertices().len(), unscaled.vertices().len());
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!(
                (s.x - u.x * 2.0).abs() < 1e-10,
                "X should be scaled: {} vs {}",
                s.x,
                u.x * 2.0
            );
            assert!(
                (s.y - u.y * 2.0).abs() < 1e-10,
                "Y should be scaled: {} vs {}",
                s.y,
                u.y * 2.0
            );
            assert!(
                (s.z - u.z * 2.0).abs() < 1e-10,
                "Z should be scaled: {} vs {}",
                s.z,
                u.z * 2.0
            );
        }
    }

    /// Test load_mesh_file with non-uniform scale.
    #[test]
    fn test_load_mesh_file_nonuniform_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("nonuniform.stl");
        create_test_stl(&mesh_path);

        // Load with non-uniform scale: 1x, 2x, 3x
        let scaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 2.0, 3.0),
            "nonuniform",
        )
        .unwrap();

        // Load unscaled for comparison
        let unscaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unscaled",
        )
        .unwrap();

        // Verify non-uniform scale: x unchanged, y doubled, z tripled
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!((s.x - u.x).abs() < 1e-10, "X should be unchanged");
            assert!((s.y - u.y * 2.0).abs() < 1e-10, "Y should be 2x");
            assert!((s.z - u.z * 3.0).abs() < 1e-10, "Z should be 3x");
        }
    }

    /// Test load_mesh_file fails for unsupported format.
    #[test]
    fn test_load_mesh_file_unsupported_format() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.xyz");
        std::fs::write(&mesh_path, b"invalid mesh data").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unsupported",
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("failed to load"));
    }

    /// Test load_mesh_file fails gracefully for corrupt STL data.
    ///
    /// A file with `.stl` extension but invalid content should produce
    /// a clear error, not panic.
    #[test]
    fn test_load_mesh_file_corrupt_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("corrupt.stl");

        // Write invalid STL data (valid STL needs specific binary/ASCII structure)
        std::fs::write(&mesh_path, b"this is not a valid stl file content").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "corrupt_mesh",
        );

        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("corrupt_mesh") && err_msg.contains("failed to load"),
            "error should identify mesh and indicate load failure: {err_msg}"
        );
    }

    /// Test convert_mjcf_mesh with file attribute.
    #[test]
    fn test_convert_mjcf_mesh_from_file() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("mesh.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "test_mesh".to_string(),
            file: Some(mesh_path.to_string_lossy().to_string()),
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert!(!mesh_data.vertices().is_empty());
        assert!(!mesh_data.triangles().is_empty());
    }

    /// Test convert_mjcf_mesh with file attribute and relative path.
    #[test]
    fn test_convert_mjcf_mesh_relative_path() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let meshes_dir = temp_dir.path().join("assets");
        std::fs::create_dir_all(&meshes_dir).unwrap();
        let mesh_path = meshes_dir.join("model.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "relative_mesh".to_string(),
            file: Some("assets/model.stl".to_string()),
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, Some(temp_dir.path()), &MjcfCompiler::default());
        assert!(result.is_ok());
    }

    /// Test convert_mjcf_mesh with embedded vertex data (no file).
    #[test]
    fn test_convert_mjcf_mesh_embedded() {
        let mjcf_mesh = MjcfMesh {
            name: "embedded".to_string(),
            file: None,
            vertex: Some(vec![
                0.0, 0.0, 0.0, // v0
                1.0, 0.0, 0.0, // v1
                0.5, 1.0, 0.0, // v2
                0.5, 0.5, 1.0, // v3
            ]),
            face: Some(vec![
                0, 1, 2, // f0
                0, 1, 3, // f1
                1, 2, 3, // f2
                2, 0, 3, // f3
            ]),
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert_eq!(mesh_data.vertices().len(), 4);
        assert_eq!(mesh_data.triangles().len(), 4); // 4 triangles
    }

    /// Test convert_mjcf_mesh fails when neither file nor vertex is present.
    #[test]
    fn test_convert_mjcf_mesh_no_data() {
        let mjcf_mesh = MjcfMesh {
            name: "empty".to_string(),
            file: None,
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("no vertex data"));
    }

    /// Test load_model_from_file with mesh geometry.
    #[test]
    fn test_load_model_from_file_with_mesh() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");

        // Create mesh file
        let mesh_path = temp_dir.path().join("cube.stl");
        create_test_stl(&mesh_path);

        // Create MJCF file referencing the mesh
        let mjcf_content = r#"
            <mujoco model="mesh_test">
                <asset>
                    <mesh name="cube_mesh" file="cube.stl"/>
                </asset>
                <worldbody>
                    <body name="cube" pos="0 0 1">
                        <geom type="mesh" mesh="cube_mesh" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#;
        let mjcf_path = temp_dir.path().join("model.xml");
        std::fs::write(&mjcf_path, mjcf_content).unwrap();

        let result = load_model_from_file(&mjcf_path);
        assert!(
            result.is_ok(),
            "Should load model with mesh: {:?}",
            result.err()
        );
        let model = result.unwrap();
        assert_eq!(model.name, "mesh_test");
        assert_eq!(model.nmesh, 1);
        assert!(!model.mesh_data.is_empty());
    }

    /// Test model_from_mjcf with explicit base_path.
    #[test]
    fn test_model_from_mjcf_with_base_path() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");

        // Create mesh in subdirectory
        let assets_dir = temp_dir.path().join("assets");
        std::fs::create_dir_all(&assets_dir).unwrap();
        let mesh_path = assets_dir.join("shape.stl");
        create_test_stl(&mesh_path);

        // Parse MJCF referencing relative mesh path
        let mjcf = crate::parse_mjcf_str(
            r#"
            <mujoco model="base_path_test">
                <asset>
                    <mesh name="shape" file="assets/shape.stl"/>
                </asset>
                <worldbody>
                    <body name="obj" pos="0 0 1">
                        <geom type="mesh" mesh="shape" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("Should parse MJCF");

        // Convert with base_path
        let result = model_from_mjcf(&mjcf, Some(temp_dir.path()));
        assert!(
            result.is_ok(),
            "Should convert with base_path: {:?}",
            result.err()
        );
    }

    // ========================================================================
    // <general> actuator MJCF attributes (spec §8)
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
        // Filter → 1 activation state
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
        // No dynamics → 0 activation states
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
        assert!((model.actuator_dynprm[0][0] - 1.0).abs() < 1e-10); // τ = 1.0s
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

    // euler_seq_to_quat unit tests → builder/orientation.rs

    #[test]
    fn test_angle_conversion_hinge_joint_degrees() {
        // Default compiler: angle=degree. Hinge range in degrees → converted to radians.
        let model = load_model(
            r#"
            <mujoco model="deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-90 90"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        assert!(
            (model.jnt_range[0].0 - (-pi / 2.0)).abs() < 1e-10,
            "range lo: expected {}, got {}",
            -pi / 2.0,
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - (pi / 2.0)).abs() < 1e-10,
            "range hi: expected {}, got {}",
            pi / 2.0,
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_degrees() {
        // Ball joint range is angle-valued (swing/twist limits).
        // With default compiler.angle=degree, range values must be converted to radians.
        // MuJoCo semantics: range="0 60" means max-swing=0°, max-twist=60° (or similar).
        let model = load_model(
            r#"
            <mujoco model="ball_deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 60"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        let expected_lo = 0.0;
        let expected_hi = 60.0 * pi / 180.0; // π/3
        assert!(
            (model.jnt_range[0].0 - expected_lo).abs() < 1e-10,
            "ball range lo: expected {expected_lo}, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - expected_hi).abs() < 1e-10,
            "ball range hi: expected {expected_hi}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_radian_passthrough() {
        // With angle="radian", ball joint range values pass through unchanged.
        let pi_3 = std::f64::consts::PI / 3.0;
        let model = load_model(&format!(
            r#"
            <mujoco model="ball_rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 {pi_3}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - 0.0).abs() < 1e-10,
            "ball radian range lo: expected 0.0, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - pi_3).abs() < 1e-10,
            "ball radian range hi: expected {pi_3}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_radian_passthrough() {
        // With angle="radian", values pass through unchanged.
        let model = load_model(
            r#"
            <mujoco model="rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-1.57 1.57"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-1.57)).abs() < 1e-10,
            "range lo: expected -1.57, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - 1.57).abs() < 1e-10,
            "range hi: expected 1.57, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_slide_joint_not_converted() {
        // Slide joint range is translational — never converted even with angle=degree.
        let model = load_model(
            r#"
            <mujoco model="slide_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="slide" limited="true" range="-0.5 0.5"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-0.5)).abs() < 1e-10,
            "slide range should not be converted"
        );
        assert!(
            (model.jnt_range[0].1 - 0.5).abs() < 1e-10,
            "slide range should not be converted"
        );
    }

    #[test]
    fn test_angle_conversion_springref_degrees() {
        // Hinge springref in degrees → converted to radians.
        let model = load_model(
            r#"
            <mujoco model="springref_deg">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" stiffness="100" springref="45"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = 45.0 * std::f64::consts::PI / 180.0;
        assert!(
            (model.jnt_springref[0] - expected).abs() < 1e-10,
            "springref: expected {expected}, got {}",
            model.jnt_springref[0]
        );
    }

    #[test]
    fn test_body_axisangle_orientation() {
        // Body with axisangle should produce correct quaternion.
        // axisangle="0 0 1 90" with default degree → 90° around Z.
        let model = load_model(
            r#"
            <mujoco model="axisangle_test">
                <worldbody>
                    <body name="b" axisangle="0 0 1 90">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // 90° around Z: quaternion = [cos(45°), 0, 0, sin(45°)]
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.body_quat[1]; // body 0 is world, body 1 is "b"
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle 90° Z: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_body_axisangle_radian() {
        // With angle="radian", axisangle angle is in radians.
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
                <mujoco model="axisangle_rad">
                    <compiler angle="radian"/>
                    <worldbody>
                        <body name="b" axisangle="0 0 1 {pi_2}">
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </worldbody>
                </mujoco>
                "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), pi_2);
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle radian: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_body_orientation() {
        // With eulerseq="ZYX" and angle="radian", body euler should use ZYX sequence.
        let model = load_model(
            r#"
            <mujoco model="eulerseq_test">
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b" euler="0.3 0.2 0.1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "ZYX euler: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_autolimits_infers_limited_from_range() {
        // Default autolimits=true: range present + no explicit limited → limited=true.
        let model = load_model(
            r#"
            <mujoco model="autolimits_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            model.jnt_limited[0],
            "autolimits should infer limited=true from range"
        );
    }

    #[test]
    fn test_autolimits_false_requires_explicit_limited() {
        // With autolimits=false, range without limited → limited=false.
        let model = load_model(
            r#"
            <mujoco model="no_autolimits">
                <compiler angle="radian" autolimits="false"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            !model.jnt_limited[0],
            "autolimits=false should not infer limited"
        );
    }

    #[test]
    fn test_autolimits_explicit_limited_takes_precedence() {
        // Explicit limited=false overrides autolimits inference.
        let model = load_model(
            r#"
            <mujoco model="explicit_limited">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="false" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            !model.jnt_limited[0],
            "explicit limited=false should override autolimits"
        );
    }

    #[test]
    fn test_autolimits_no_range_no_limited() {
        // No range and no limited → limited=false (regardless of autolimits).
        let model = load_model(
            r#"
            <mujoco model="no_range">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(!model.jnt_limited[0], "no range should mean not limited");
    }

    // ── Mass pipeline tests ──────────────────────────────────────────

    #[test]
    fn test_inertiafromgeom_true_overrides_explicit_inertial() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="999.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // inertiafromgeom="true" → geom mass (2.0) overrides explicit 999.0
        assert!(
            (model.body_mass[1] - 2.0).abs() < 1e-10,
            "mass should come from geom, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_uses_explicit_only() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="5.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 5.0).abs() < 1e-10,
            "mass should come from explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_no_inertial_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "mass should be zero without explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_auto_no_geoms_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint type="hinge" axis="0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Auto mode with no geoms and no inertial → zero mass
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "empty body should have zero mass, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundmass_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="0.5"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 0.5).abs() < 1e-10,
            "mass should be clamped to 0.5, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundinertia_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundinertia="0.01"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.01).abs() < 1e-10
                && (inertia.y - 0.01).abs() < 1e-10
                && (inertia.z - 0.01).abs() < 1e-10,
            "inertia should be clamped to 0.01, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_fixes_triangle_inequality() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // 0.1 + 0.1 < 0.5 violates triangle inequality → mean = (0.1+0.1+0.5)/3
        let expected = (0.1 + 0.1 + 0.5) / 3.0;
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - expected).abs() < 1e-10
                && (inertia.y - expected).abs() < 1e-10
                && (inertia.z - expected).abs() < 1e-10,
            "inertia should be balanced to mean {expected}, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_no_change_when_valid() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.3 0.3 0.3"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.3).abs() < 1e-10
                && (inertia.y - 0.3).abs() < 1e-10
                && (inertia.z - 0.3).abs() < 1e-10,
            "valid inertia should not change, got {inertia:?}"
        );
    }

    #[test]
    fn test_settotalmass_rescales() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="10.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 3 + 7 = 10, settotalmass = 10 → no change
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 10.0).abs() < 1e-10,
            "total mass should be 10.0, got {total}"
        );
    }

    #[test]
    fn test_settotalmass_rescales_different_target() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="20.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 10, target = 20 → scale = 2x
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 20.0).abs() < 1e-10,
            "total mass should be 20.0, got {total}"
        );
        // Mass ratios should be preserved: a=6.0, b=14.0
        assert!(
            (model.body_mass[1] - 6.0).abs() < 1e-10,
            "body a mass should be 6.0, got {}",
            model.body_mass[1]
        );
        assert!(
            (model.body_mass[2] - 14.0).abs() < 1e-10,
            "body b mass should be 14.0, got {}",
            model.body_mass[2]
        );
    }

    #[test]
    fn test_mass_pipeline_order_bound_then_settotalmass() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="1.0" settotalmass="5.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.1"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // After boundmass: a=1.0, b=2.0, total=3.0
        // After settotalmass(5.0): scale=5/3, a=5/3, b=10/3
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 5.0).abs() < 1e-10,
            "total mass should be 5.0, got {total}"
        );
    }

    // ── Include file integration tests ───────────────────────────────

    #[test]
    fn test_load_model_from_file_with_includes() {
        let dir = tempfile::tempdir().expect("tempdir");

        // Main model file
        std::fs::write(
            dir.path().join("main.xml"),
            r#"<mujoco model="included">
                <compiler angle="radian"/>
                <worldbody>
                    <include file="arm.xml"/>
                </worldbody>
                <include file="actuators.xml"/>
            </mujoco>"#,
        )
        .unwrap();

        // Body definitions
        std::fs::write(
            dir.path().join("arm.xml"),
            r#"<wrapper>
                <body name="link1" pos="0 0 0.5">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                </body>
            </wrapper>"#,
        )
        .unwrap();

        // Actuator definitions
        std::fs::write(
            dir.path().join("actuators.xml"),
            r#"<mujoco>
                <actuator>
                    <motor joint="j1" name="m1"/>
                </actuator>
            </mujoco>"#,
        )
        .unwrap();

        let model = load_model_from_file(dir.path().join("main.xml")).expect("should load");
        assert_eq!(model.name, "included");
        assert!(model.nbody >= 2, "should have world + link1");
        assert_eq!(model.njnt, 1, "should have 1 joint");
        assert_eq!(model.nu, 1, "should have 1 actuator");
    }

    #[test]
    fn test_load_model_string_rejects_includes() {
        let result = load_model(
            r#"<mujoco>
                <worldbody>
                    <include file="bodies.xml"/>
                </worldbody>
            </mujoco>"#,
        );
        assert!(result.is_err(), "string API should reject includes");
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("include"),
            "error should mention include: {err}"
        );
    }

    // ── discardvisual / fusestatic tests ─────────────────────────────

    #[test]
    fn test_discardvisual_removes_visual_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Only collision geom should remain
        assert_eq!(model.ngeom, 1, "visual geom should be discarded");
    }

    #[test]
    fn test_discardvisual_false_keeps_all_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(
            model.ngeom, 2,
            "all geoms should remain when discardvisual=false"
        );
    }

    #[test]
    fn test_fusestatic_merges_jointless_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // static_child (no joints) should be fused into parent
        // parent + world = 2 bodies (static_child merged)
        assert_eq!(
            model.nbody, 2,
            "static child should be fused: nbody={}",
            model.nbody
        );
        // Geoms: parent sphere + fused box = 2
        assert_eq!(model.ngeom, 2, "both geoms should remain after fusion");
    }

    #[test]
    fn test_fusestatic_preserves_jointed_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="base" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="link" pos="0 0 0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <geom type="sphere" size="0.08"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Both bodies have joints — no fusion
        assert_eq!(
            model.nbody, 3,
            "jointed bodies should not be fused: nbody={}",
            model.nbody
        );
    }

    #[test]
    fn test_fusestatic_false_preserves_all_bodies() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(model.nbody, 3, "fusestatic=false should keep all bodies");
    }

    #[test]
    fn test_fusestatic_orientation_handling() {
        // A static body rotated 90° around Z should rotate its child geom position and orientation.
        // Parent body at origin with a static child rotated 90° around Z at pos [1, 0, 0].
        // Child has a geom at local pos [0.5, 0, 0].
        // After fusion: geom pos should be rotated by 90° Z: [1, 0, 0] + rot90z([0.5, 0, 0])
        //   = [1, 0, 0] + [0, 0.5, 0] = [1, 0.5, 0]
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="parent_geom" type="sphere" size="0.1" mass="1"/>
                        <body name="rotated_static" pos="1 0 0" euler="0 0 90">
                            <geom name="child_geom" type="sphere" size="0.05" pos="0.5 0 0" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // The static body should be fused into parent: 2 bodies (world + parent)
        assert_eq!(model.nbody, 2, "rotated static body should be fused");
        // Parent should have 2 geoms (its own + child's)
        assert_eq!(
            model.body_geom_num[1], 2,
            "parent should have 2 geoms after fusion"
        );

        // Check the fused geom position: should be [1, 0.5, 0] after rotation
        // geom[0] is parent_geom (from worldbody geom processing), geom[1] is child_geom
        let child_geom_idx = 1; // second geom on this body
        let fused_pos = model.geom_pos[child_geom_idx];
        assert!(
            (fused_pos.x - 1.0).abs() < 1e-10,
            "fused geom x should be 1.0, got {}",
            fused_pos.x
        );
        assert!(
            (fused_pos.y - 0.5).abs() < 1e-10,
            "fused geom y should be 0.5 (rotated), got {}",
            fused_pos.y
        );
        assert!(
            fused_pos.z.abs() < 1e-10,
            "fused geom z should be 0, got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_no_rotation_preserves_geom_euler() {
        // A static body with NO rotation but a geom that HAS euler orientation.
        // After fusion the geom euler should still produce the correct orientation.
        // This tests the no-rotation branch: euler must remain valid in parent frame.
        //
        // Setup: static body at pos [0, 0, 1] with identity orientation.
        //        geom at local pos [0, 0, 0] with euler="0 0 90" (45° around Z).
        // After fusion: geom at [0, 0, 1] with same 90° Z rotation.
        // Compare against a model WITHOUT fusestatic to verify identical geom_quat.
        let with_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("fuse model should load");

        let without_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("no-fuse model should load");

        // Find g1's geom index in each model
        // With fusion: 2 bodies (world + parent), g1 is second geom on parent (idx 1)
        // Without fusion: 3 bodies (world + parent + static_child), g1 is first geom on static_child (idx 1)
        assert_eq!(with_fuse.nbody, 2, "fused model should have 2 bodies");
        assert_eq!(without_fuse.nbody, 3, "unfused model should have 3 bodies");

        // Both models should have 2 geoms total (g0 + g1)
        assert_eq!(with_fuse.ngeom, 2);
        assert_eq!(without_fuse.ngeom, 2);

        // Geom quaternions should match — proving euler is preserved correctly through fusion
        let fused_quat = with_fuse.geom_quat[1];
        let unfused_quat = without_fuse.geom_quat[1];
        let diff = (fused_quat.into_inner() - unfused_quat.into_inner()).norm();
        assert!(
            diff < 1e-10,
            "fused geom quat should match unfused: fused={fused_quat:?}, unfused={unfused_quat:?}, diff={diff}"
        );

        // Geom positions should account for body offset
        let fused_pos = with_fuse.geom_pos[1];
        assert!(
            (fused_pos.z - 1.0).abs() < 1e-10,
            "fused geom z should be 1.0 (body offset), got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_protects_sensor_referenced_body() {
        // A subtreecom sensor references a body by name — that body must not be fused.
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                        <body name="sensor_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
                <sensor>
                    <subtreecom name="com_sensor" body="sensor_target"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        // sensor_target should NOT be fused because it's referenced by a sensor
        assert_eq!(
            model.nbody, 3,
            "sensor-referenced body should be protected from fusion"
        );
    }

    #[test]
    fn test_fusestatic_protects_actuator_referenced_body() {
        // An adhesion actuator references a body by name — that body must not be fused.
        // Test at the MjcfModel level since body transmission isn't wired to Model yet.
        let mut mjcf = crate::parse_mjcf_str(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="actuator_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                        <body name="unprotected" pos="0 0 1">
                            <geom type="sphere" size="0.05"/>
                        </body>
                    </body>
                </worldbody>
                <actuator>
                    <adhesion name="stick" body="actuator_target" gain="1"/>
                </actuator>
            </mujoco>
            "#,
        )
        .expect("should parse");

        apply_fusestatic(&mut mjcf);

        // actuator_target should still exist, unprotected should be fused
        let parent = &mjcf.worldbody.children[0];
        let child_names: Vec<&str> = parent.children.iter().map(|c| c.name.as_str()).collect();
        assert!(
            child_names.contains(&"actuator_target"),
            "actuator-referenced body should be protected: {child_names:?}"
        );
        assert!(
            !child_names.contains(&"unprotected"),
            "unprotected body should be fused away: {child_names:?}"
        );
    }

    #[test]
    fn test_discardvisual_protects_sensor_referenced_geom() {
        // A visual geom (contype=0, conaffinity=0) referenced by a frame sensor
        // should NOT be discarded.
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="collision_geom" type="sphere" size="0.1" mass="1"/>
                        <geom name="visual_ref" type="sphere" size="0.2" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
                <sensor>
                    <framepos name="vis_sensor" objname="visual_ref"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        // visual_ref should be kept because it's referenced by a sensor
        assert_eq!(
            model.ngeom, 2,
            "sensor-referenced visual geom should be protected from discard"
        );
    }

    // resolve_asset_path texturedir tests → builder/asset.rs

    // =========================================================================
    // Frame expansion + childclass tests (Spec #19, 27 acceptance criteria)
    // =========================================================================

    // AC1: Frame position
    #[test]
    fn test_ac01_frame_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <frame pos="1 0 0">
                            <geom name="fg" type="sphere" size="0.1" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom 0 is the body's own geom, geom 1 is the frame's geom
        assert_eq!(model.geom_pos.len(), 2);
        let pos = model.geom_pos[1];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC2: Frame rotation
    #[test]
    fn test_ac02_frame_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "90deg Z rotation should map (1,0,0) to (0,1,0), got {pos:?}"
        );
    }

    // AC3: Frame position + rotation
    #[test]
    fn test_ac03_frame_pos_plus_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        // frame_pos + frame_rot * geom_pos = (1,0,0) + (0,1,0) = (1,1,0)
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC4: Nested frames
    #[test]
    fn test_ac04_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <geom name="fg" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-10,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC5: 3-deep nested frames
    #[test]
    fn test_ac05_three_deep_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <frame pos="0 0 1">
                                    <geom name="fg" type="sphere" size="0.1"/>
                                </frame>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
            "expected (1,1,1), got {pos:?}"
        );
    }

    // AC6: Frame wrapping body
    #[test]
    fn test_ac06_frame_wrapping_body() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="parent">
                        <frame pos="2 0 0">
                            <body name="child" pos="1 0 0">
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // body_pos for child should be (3,0,0)
        // body 0 = world, body 1 = parent, body 2 = child
        assert_eq!(model.body_pos.len(), 3);
        let child_pos = model.body_pos[2];
        assert!(
            (child_pos - Vector3::new(3.0, 0.0, 0.0)).norm() < 1e-10,
            "body_pos should be (3,0,0), got {child_pos:?}"
        );
    }

    // AC7: Frame with fromto geom
    #[test]
    fn test_ac07_frame_fromto_geom() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // fromto endpoints become (1,0,0)-(1,0,1), pos = midpoint (1, 0, 0.5)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-6,
            "expected midpoint (1,0,0.5), got {pos:?}"
        );
    }

    // AC8: Frame with fromto geom + rotation
    #[test]
    fn test_ac08_frame_fromto_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 90 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // 90deg Y rotation maps Z to X. Endpoints: (0,0,0)-(1,0,0)
        // Geom pos ≈ midpoint (0.5, 0, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-6,
            "expected midpoint (0.5,0,0), got {pos:?}"
        );
    }

    // AC9: Frame with site
    #[test]
    fn test_ac09_frame_with_site() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="0.5 0 0">
                            <site name="s" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-10,
            "site should be at (0.5,0,0), got {pos:?}"
        );
    }

    // AC10: Empty frame
    #[test]
    fn test_ac10_empty_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Model should load without error, geom at (0,0,0) unaffected
        assert_eq!(model.geom_pos.len(), 1);
        assert!(
            (model.geom_pos[0]).norm() < 1e-10,
            "geom should be at origin (empty frame has no children)"
        );
    }

    // AC11: Frame at worldbody level
    #[test]
    fn test_ac11_frame_at_worldbody() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <frame pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </frame>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "worldbody frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC12: Frame with only orientation (no pos)
    #[test]
    fn test_ac12_frame_only_orientation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC13: Frame with only position (no orientation)
    #[test]
    fn test_ac13_frame_only_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="3 0 0">
                            <geom type="sphere" size="0.1" pos="0 2 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(3.0, 2.0, 0.0)).norm() < 1e-10,
            "expected (3,2,0), got {pos:?}"
        );
    }

    // AC14: Frame with xyaxes
    #[test]
    fn test_ac14_frame_xyaxes() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame xyaxes="0 1 0 -1 0 0">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // xyaxes "0 1 0 -1 0 0" = 90deg Z rotation. Geom at (0, 1, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC15: Frame with zaxis
    #[test]
    fn test_ac15_frame_zaxis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame zaxis="1 0 0">
                            <geom type="sphere" size="0.1" pos="0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // zaxis "1 0 0" maps Z to X. Geom at local (0,0,1) → body-relative (1, 0, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-6,
            "expected (1,0,0), got {pos:?}"
        );
    }

    // AC16: Frame with axisangle
    #[test]
    fn test_ac16_frame_axisangle() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame axisangle="0 0 1 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // 90deg about Z. Geom at (0, 1, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC17: Joint inside frame = error
    #[test]
    fn test_ac17_joint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <joint name="j" type="hinge"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "joint inside frame should error");
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("not allowed inside <frame>"),
            "error should mention invalid element: {err}"
        );
    }

    // AC18: Freejoint inside frame = error
    #[test]
    fn test_ac18_freejoint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <freejoint/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "freejoint inside frame should error");
    }

    // AC19: Inertial inside frame = error
    #[test]
    fn test_ac19_inertial_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "inertial inside frame should error");
    }

    // AC20: Childclass on body
    #[test]
    fn test_ac20_childclass_on_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom should inherit "red" defaults (contype = 7)
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'red', got {}",
            model.geom_contype[0]
        );
    }

    // AC21: Childclass override by explicit class
    #[test]
    fn test_ac21_childclass_override_by_explicit() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="blue">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom class="blue" type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom uses explicit class "blue" (contype=3), not childclass "red" (contype=7)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should use explicit class 'blue' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC22: Childclass on frame
    #[test]
    fn test_ac22_childclass_on_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="green">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <frame childclass="green">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom should inherit "green" (frame's childclass overrides body's)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit frame childclass 'green' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC23: Childclass inheritance through body hierarchy
    #[test]
    fn test_ac23_childclass_inheritance_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <joint damping="5.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="parent" childclass="robot">
                        <body name="child" pos="0 0 1">
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Inner body's joint should inherit class "robot" from parent
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'robot', got {}",
            model.dof_damping[0]
        );
    }

    // AC24: Childclass on child body overrides parent
    #[test]
    fn test_ac24_childclass_child_overrides_parent() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="A">
                        <geom contype="7"/>
                    </default>
                    <default class="B">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="outer" childclass="A">
                        <body name="inner" childclass="B">
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom inherits "B" (child body's childclass overrides parent's)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit child body childclass 'B' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC25: Regression - no frames, no childclass
    #[test]
    fn test_ac25_regression_no_frames() {
        // Simple model without frames or childclass should work identically
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="1 2 3">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s" pos="0.5 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2); // world + b
        assert_eq!(model.ngeom, 1);
        let body_pos = model.body_pos[1];
        assert!((body_pos - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
    }

    // AC26: Geom orientation composition
    #[test]
    fn test_ac26_geom_orientation_composition() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // geom orientation = frame_quat * geom_quat
        // Both euler values resolved, then composed
        // Just verify model loads and geom has a non-identity orientation
        let quat = model.geom_quat[0];
        // The composed rotation should NOT be identity
        let is_identity = (quat.w - 1.0).abs() < 1e-6
            && quat.i.abs() < 1e-6
            && quat.j.abs() < 1e-6
            && quat.k.abs() < 1e-6;
        assert!(!is_identity, "composed rotation should not be identity");
    }

    // AC27: Frame + angle="radian"
    #[test]
    fn test_ac27_frame_radian_mode() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 1.5707963">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        // Same as degree case: geom at (0, 1, 0)
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-4,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // ========================================================================
    // AC28–AC35: Childclass edge cases (item #20)
    // ========================================================================

    // AC28: Childclass referencing a nested default class (default hierarchy)
    #[test]
    fn test_ac28_childclass_nested_default_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <geom contype="5"/>
                        <default class="arm">
                            <geom conaffinity="3"/>
                        </default>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="arm">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // "arm" inherits contype=5 from parent "robot", and sets conaffinity=3 directly
        assert_eq!(
            model.geom_contype[0], 5,
            "geom should inherit contype=5 from parent class 'robot' through 'arm', got {}",
            model.geom_contype[0]
        );
        assert_eq!(
            model.geom_conaffinity[0], 3,
            "geom should get conaffinity=3 from class 'arm', got {}",
            model.geom_conaffinity[0]
        );
    }

    // AC29: Childclass applies to geom, joint, AND site simultaneously
    #[test]
    fn test_ac29_childclass_multi_element() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="R">
                        <geom contype="7"/>
                        <joint damping="5.0"/>
                        <site size="0.05"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="R">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'R', got {}",
            model.geom_contype[0]
        );
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'R', got {}",
            model.dof_damping[0]
        );
        // Site size default is [0.01]; class "R" sets it to [0.05]
        let site_size = model.site_size[0];
        assert!(
            (site_size.x - 0.05).abs() < 1e-10,
            "site should inherit size=0.05 from childclass 'R', got {}",
            site_size.x
        );
    }

    // AC30: 3-level deep propagation without override
    #[test]
    fn test_ac30_childclass_3level_propagation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="X">
                        <joint damping="3.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="top" childclass="X">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 3.0).abs() < 1e-10,
            "joint at 3rd level should inherit damping=3.0 from top's childclass 'X', got {}",
            model.dof_damping[0]
        );
    }

    // AC31: 3-level with mid-hierarchy override
    #[test]
    fn test_ac31_childclass_3level_mid_override() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="A"><joint damping="1.0"/></default>
                    <default class="B"><joint damping="9.0"/></default>
                </default>
                <worldbody>
                    <body name="top" childclass="A">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" childclass="B" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 9.0).abs() < 1e-10,
            "joint should inherit damping=9.0 from mid's childclass 'B' (not top's 'A'), got {}",
            model.dof_damping[0]
        );
    }

    // AC32: Nested frames with childclass inheritance and override
    #[test]
    fn test_ac32_nested_frames_childclass() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="F1"><geom contype="4"/></default>
                    <default class="F2"><geom contype="8"/></default>
                </default>
                <worldbody>
                    <body name="b">
                        <frame childclass="F1">
                            <frame>
                                <geom name="g1" type="sphere" size="0.1"/>
                            </frame>
                            <frame childclass="F2">
                                <geom name="g2" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // g1 inherits outer frame's F1 through inner frame (no override)
        assert_eq!(
            model.geom_contype[0], 4,
            "g1 should inherit contype=4 from outer frame childclass 'F1', got {}",
            model.geom_contype[0]
        );
        // g2 gets F2 from inner frame override
        assert_eq!(
            model.geom_contype[1], 8,
            "g2 should get contype=8 from inner frame childclass 'F2', got {}",
            model.geom_contype[1]
        );
    }

    // AC33: childclass="nonexistent" on body produces error
    #[test]
    fn test_ac33_childclass_nonexistent_body_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" childclass="nonexistent">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='nonexistent' should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("nonexistent"),
            "error message should mention 'nonexistent', got: {err_msg}"
        );
    }

    // AC34: Body with childclass but no child elements succeeds
    #[test]
    fn test_ac34_childclass_empty_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="empty"><geom contype="5"/></default>
                </default>
                <worldbody>
                    <body name="b" childclass="empty" pos="0 0 1">
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2, "should have world + b");
        assert_eq!(model.ngeom, 0, "body has no geoms");
    }

    // AC35: childclass="ghost" on frame produces error
    #[test]
    fn test_ac35_childclass_nonexistent_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame childclass="ghost">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='ghost' on frame should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("ghost"),
            "error message should mention 'ghost', got: {err_msg}"
        );
    }

    // ========================================================================
    // Additional coverage: fusestatic + orientation fields, discardvisual + frame
    // ========================================================================

    #[test]
    fn test_fusestatic_geom_with_xyaxes() {
        // A geom with xyaxes orientation on a static body (no joints).
        // fusestatic should correctly resolve xyaxes via resolve_orientation
        // before composing with the parent body's transform.
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="1 0 0" euler="0 0 90">
                            <geom type="sphere" size="0.1" xyaxes="0 1 0 -1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // static_child is fused into parent. The geom's xyaxes (90° Z rotation)
        // should compose with the body's 90° Z euler rotation = 180° total Z rotation.
        // Position: body_quat * (0,0,0) + (1,0,0) = (1,0,0).
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-4,
            "expected (1,0,0), got {pos:?}"
        );
    }

    #[test]
    fn test_fusestatic_site_with_axisangle() {
        // A site with axisangle orientation on a static body.
        // fusestatic should correctly resolve axisangle via resolve_orientation.
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="0 0 1">
                            <site name="s" pos="0 0 0" axisangle="0 0 1 90"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Site should be at (0,0,1) after fusion. No body rotation so just translation.
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-4,
            "expected (0,0,1), got {pos:?}"
        );
    }

    #[test]
    fn test_discardvisual_geom_inside_frame() {
        // A visual-only geom (contype=0, conaffinity=0) inside a frame
        // should be discarded by discardvisual after frame expansion.
        let model = load_model(
            r#"
            <mujoco>
                <compiler discardvisual="true"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
                        </frame>
                        <geom type="sphere" size="0.1" pos="0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Only the non-visual geom should survive
        assert_eq!(
            model.geom_pos.len(),
            1,
            "discardvisual should remove visual geom from frame"
        );
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 0.0)).norm() < 1e-4,
            "surviving geom should be at origin"
        );
    }

    #[test]
    fn test_fromto_geom_in_frame_with_pos_and_rotation() {
        // fromto geom inside a frame that has both position and rotation.
        // Endpoints should be transformed through the full frame transform.
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom type="capsule" size="0.05" fromto="0 0 0 0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Frame: pos=(1,0,0), 90° Z rotation.
        // fromto "0 0 0 0 0 1": from=(0,0,0), to=(0,0,1)
        // Transformed from: (1,0,0) + rot*(0,0,0) = (1,0,0)
        // Transformed to:   (1,0,0) + rot*(0,0,1) = (1,0,1)
        // Midpoint: (1, 0, 0.5)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-4,
            "expected (1,0,0.5), got {pos:?}"
        );
    }

    // =========================================================================
    // Site orientation tests (item #21)
    // =========================================================================

    /// AC1: Site euler orientation with default angle="degree".
    #[test]
    fn test_site_euler_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90° X deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC2: Site euler orientation with explicit angle="radian".
    #[test]
    fn test_site_euler_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="{pi_2} 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler pi/2 X rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC3: Site euler with non-default eulerseq="ZYX".
    #[test]
    fn test_site_euler_orientation_zyx_eulerseq() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="0.3 0.2 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler ZYX: got {got:?}, expected {expected:?}"
        );
    }

    /// AC4: Site axisangle orientation with default angle="degree".
    #[test]
    fn test_site_axisangle_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 1 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle 90° Z deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC5: Site axisangle orientation with angle="radian".
    #[test]
    fn test_site_axisangle_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 1 0 {pi_2}"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle pi/2 Y rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC6: Site xyaxes orientation with orthogonal inputs (90° Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_orthogonal() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="0 1 0 -1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // x=(0,1,0), y=(-1,0,0), z=cross(x,y)=(0,0,1) → 90° about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site xyaxes orthogonal 90° Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC7: Site xyaxes with non-orthogonal inputs (Gram-Schmidt → 45° Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_gram_schmidt() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="1 1 0 0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Gram-Schmidt: x=norm(1,1,0)=(1/√2, 1/√2, 0), y orthogonalized → 45° about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 45.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site xyaxes Gram-Schmidt 45° Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC8: Site zaxis general direction (Z→X = 90° about Y).
    #[test]
    fn test_site_zaxis_orientation_general() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Minimal rotation from (0,0,1) to (1,0,0): 90° about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis Z→X (90° Y): got {got:?}, expected {expected:?}"
        );
    }

    /// AC9: Site zaxis parallel to default Z → identity.
    #[test]
    fn test_site_zaxis_orientation_parallel_identity() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis (0,0,1) identity: got {got:?}, expected {expected:?}"
        );
    }

    /// AC10: Site zaxis anti-parallel (0,0,-1) → 180° about X.
    #[test]
    fn test_site_zaxis_orientation_antiparallel() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 -1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Anti-parallel: fallback axis = X, angle = atan2(0, -1) = PI → 180° about X
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis anti-parallel 180° X: got {got:?}, expected {expected:?}"
        );
    }

    /// AC11: Site quat orientation (regression for pre-#19 behavior).
    #[test]
    fn test_site_quat_orientation_regression() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" quat="0.7071068 0 0.7071068 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Direct quaternion (wxyz): 90° about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site quat regression 90° Y: got {got:?}, expected {expected:?}"
        );
    }

    /// AC12: Site orientation with default class (defaults provide type/size,
    /// element provides orientation — no interference).
    #[test]
    fn test_site_orientation_with_default_class() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="sensor_site">
                        <site type="cylinder" size="0.02 0.01"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" class="sensor_site" euler="0 0 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Type from default class
        assert_eq!(
            model.site_type[0],
            GeomType::Cylinder,
            "site type should come from default class"
        );

        // Orientation from element (not interfered by defaults)
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90° Z with default class: got {got:?}, expected {expected:?}"
        );
    }

    /// AC13: Orientation priority — euler takes precedence over quat when both specified.
    #[test]
    fn test_site_orientation_priority_euler_over_quat() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0" quat="1 0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // euler wins over quat per priority chain. quat=(1,0,0,0) is identity;
        // euler="90 0 0" is 90° X. Result must be 90° X, NOT identity.
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "euler should take priority over quat: got {got:?}, expected {expected:?}"
        );
    }

    /// AC14: Site orientation composed with frame rotation.
    #[test]
    fn test_site_orientation_in_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <site name="s" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Frame: 90° Z, Site: 90° X. Composed: frame_q * site_q
        let frame_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let site_q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let expected = frame_q * site_q;
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site in frame composed: got {got:?}, expected {expected:?}"
        );
    }

    /// AC15: Site axisangle with non-unit axis (normalization).
    #[test]
    fn test_site_axisangle_non_unit_axis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 3 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Axis (0,0,3) normalizes to (0,0,1) → same as axisangle="0 0 1 90"
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle non-unit axis: got {got:?}, expected {expected:?}"
        );
    }

    /// AC16: Site zaxis with non-unit direction (normalization, parallel to Z).
    #[test]
    fn test_site_zaxis_non_unit_direction() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // (0,0,5) normalizes to (0,0,1) → parallel to default Z → identity
        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis non-unit parallel: got {got:?}, expected {expected:?}"
        );
    }

    // ========================================================================
    // #27E — compute_vertex_masses: mass attribute path
    // ========================================================================

    /// #27E AC1: mass attribute distributes uniformly across vertices
    #[test]
    fn test_vertex_masses_from_mass_attribute() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(1.0);
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2], vec![1, 3, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 4);
        // 1.0 / 4 = 0.25 per vertex
        for &m in &masses {
            assert!((m - 0.25).abs() < 1e-15, "expected 0.25, got {m}");
        }
    }

    /// #27E AC2: mass=0 → per_vert=0 (static vertices, no min-mass floor)
    #[test]
    fn test_vertex_masses_zero_mass_produces_zero() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(0.0);
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 3);
        for &m in &masses {
            assert!(
                m == 0.0,
                "mass=0 should produce per_vert=0.0 (static), got {m}"
            );
        }
    }

    /// #27E AC3: mass takes precedence over density
    #[test]
    fn test_vertex_masses_mass_overrides_density() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = Some(2.0);
        flex.density = 99999.0; // Should be ignored when mass is set
        flex.thickness = 0.01;
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.5, 0.5, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 4], vec![1, 3, 4], vec![3, 2, 4], vec![2, 0, 4]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 5);
        // 2.0 / 5 = 0.4 per vertex (density is ignored)
        for &m in &masses {
            assert!((m - 0.4).abs() < 1e-15, "expected 0.4, got {m}");
        }
    }

    /// #27E: density path still applies min-mass floor
    #[test]
    fn test_vertex_masses_density_path_has_floor() {
        let mut flex = MjcfFlex::default();
        flex.dim = 2;
        flex.mass = None; // No mass attribute → density path
        flex.density = 0.0; // Zero density → zero mass per vertex before floor
        flex.thickness = 0.01;
        flex.vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        flex.elements = vec![vec![0, 1, 2]];

        let masses = compute_vertex_masses(&flex);

        assert_eq!(masses.len(), 3);
        // Density=0 → element mass=0 → min-mass floor kicks in → 0.001
        for &m in &masses {
            assert!(
                (m - 0.001).abs() < 1e-15,
                "density path should apply min-mass floor 0.001, got {m}"
            );
        }
    }
}
