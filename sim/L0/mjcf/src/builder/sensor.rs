//! Sensor processing.
//!
//! Converts MJCF `<sensor>` elements into the 17 pipeline sensor arrays.
//! Handles object resolution (joint, tendon, actuator, site, body, geom)
//! and sensor type/datatype mapping.

use sim_core::{InterpolationType, MjJointType, MjObjectType, MjSensorDataType, MjSensorType};
use tracing::warn;

use super::{ModelBuilder, ModelConversionError};
use crate::types::{MjcfSensor, MjcfSensorType};

impl ModelBuilder {
    /// Process sensor definitions from MJCF.
    ///
    /// Converts `MjcfSensor` objects into the 17 pipeline sensor arrays.
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
                // Unknown sensor type — skip with log
                warn!(
                    "Skipping unsupported sensor type '{:?}' (sensor '{}')",
                    mjcf_sensor.sensor_type, mjcf_sensor.name,
                );
                continue;
            };
            let dim = sensor_type.dim();
            let datatype = sensor_datatype(sensor_type);

            // Dual-object sensors (GeomDist/Normal/FromTo) use geom1/body1 + geom2/body2
            // instead of the standard objname/refname resolution path.
            let (objtype, objid, reftype, refid) = if matches!(
                sensor_type,
                MjSensorType::GeomDist | MjSensorType::GeomNormal | MjSensorType::GeomFromTo
            ) {
                let (ot, oi) = self.resolve_dual_object_side(
                    mjcf_sensor.geom1.as_deref(),
                    mjcf_sensor.body1.as_deref(),
                    "1",
                )?;
                let (rt, ri) = self.resolve_dual_object_side(
                    mjcf_sensor.geom2.as_deref(),
                    mjcf_sensor.body2.as_deref(),
                    "2",
                )?;
                // Self-distance rejection: same geom on both sides
                if ot == MjObjectType::Geom && rt == MjObjectType::Geom && oi == ri {
                    return Err(ModelConversionError {
                        message:
                            "distance sensor: 1st body/geom must be different from 2nd body/geom"
                                .into(),
                    });
                }
                if ot == MjObjectType::Body && rt == MjObjectType::Body && oi == ri {
                    return Err(ModelConversionError {
                        message:
                            "distance sensor: 1st body/geom must be different from 2nd body/geom"
                                .into(),
                    });
                }
                (ot, oi, rt, ri)
            } else {
                let (ot, oi) = self.resolve_sensor_object(
                    sensor_type,
                    mjcf_sensor.objname.as_deref(),
                    mjcf_sensor.objtype.as_deref(),
                )?;
                let (rt, ri) = self.resolve_reference_object(
                    mjcf_sensor.reftype.as_deref(),
                    mjcf_sensor.refname.as_deref(),
                )?;
                (ot, oi, rt, ri)
            };

            // Parse interp keyword → InterpolationType (mirrors actuator.rs)
            let interp = match &mjcf_sensor.interp {
                Some(s) => s
                    .parse::<InterpolationType>()
                    .map_err(|e| ModelConversionError { message: e })?,
                None => InterpolationType::Zoh,
            };

            // MuJoCo compiler validation: delay > 0 requires nsample > 0
            let nsample = mjcf_sensor.nsample.unwrap_or(0);
            let delay = mjcf_sensor.delay.unwrap_or(0.0);
            if delay > 0.0 && nsample <= 0 {
                return Err(ModelConversionError {
                    message: "setting delay > 0 without a history buffer (nsample must be > 0)"
                        .into(),
                });
            }

            // MuJoCo compiler validation: negative interval rejected
            let interval = mjcf_sensor.interval.unwrap_or(0.0);
            if interval < 0.0 {
                return Err(ModelConversionError {
                    message: "negative interval in sensor".into(),
                });
            }

            self.sensor_type.push(sensor_type);
            self.sensor_datatype.push(datatype);
            self.sensor_objtype.push(objtype);
            self.sensor_objid.push(objid);
            self.sensor_reftype.push(reftype);
            self.sensor_refid.push(refid);
            self.sensor_adr.push(adr);
            self.sensor_dim.push(dim);
            self.sensor_noise.push(mjcf_sensor.noise);
            self.sensor_cutoff.push(mjcf_sensor.cutoff);
            let sensor_id = self.sensor_name_list.len();
            self.sensor_name_list.push(if mjcf_sensor.name.is_empty() {
                None
            } else {
                self.sensor_name_to_id
                    .insert(mjcf_sensor.name.clone(), sensor_id);
                Some(mjcf_sensor.name.clone())
            });
            self.sensor_nsample.push(nsample);
            self.sensor_interp.push(interp);
            self.sensor_delay.push(delay);
            self.sensor_interval.push((interval, 0.0));

            adr += dim;
        }

        self.nsensor = self.sensor_type.len();
        self.nsensordata = adr;
        Ok(())
    }

    /// Returns (MjObjectType, objid) for a given sensor type and MJCF objname.
    ///
    /// `objtype_str` is the explicit `objtype` attribute from MJCF. Only frame
    /// sensors honor it; all other sensor types ignore it.
    fn resolve_sensor_object(
        &self,
        sensor_type: MjSensorType,
        objname: Option<&str>,
        objtype_str: Option<&str>,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        // Clock and User sensors have no object reference
        if sensor_type == MjSensorType::User || sensor_type == MjSensorType::Clock {
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

            // Touch: always site-based (MuJoCo: sensor_objtype = mjOBJ_SITE).
            // objtype attribute is silently ignored for non-frame sensors.
            MjSensorType::Touch => {
                let site_id =
                    *self
                        .site_name_to_id
                        .get(name)
                        .ok_or_else(|| ModelConversionError {
                            message: format!("touch sensor references unknown site '{name}'"),
                        })?;
                Ok((MjObjectType::Site, site_id))
            }

            // Frame sensors: use explicit objtype if provided, else infer from name
            MjSensorType::FramePos
            | MjSensorType::FrameQuat
            | MjSensorType::FrameXAxis
            | MjSensorType::FrameYAxis
            | MjSensorType::FrameZAxis
            | MjSensorType::FrameLinVel
            | MjSensorType::FrameAngVel
            | MjSensorType::FrameLinAcc
            | MjSensorType::FrameAngAcc => {
                if let Some(ot) = objtype_str {
                    // Explicit objtype attribute — dispatch directly
                    match ot {
                        "site" => {
                            let id = *self.site_name_to_id.get(name).ok_or_else(|| {
                                ModelConversionError {
                                    message: format!(
                                        "frame sensor references unknown site '{name}'"
                                    ),
                                }
                            })?;
                            Ok((MjObjectType::Site, id))
                        }
                        "body" => {
                            let id = *self.body_name_to_id.get(name).ok_or_else(|| {
                                ModelConversionError {
                                    message: format!(
                                        "frame sensor references unknown body '{name}'"
                                    ),
                                }
                            })?;
                            Ok((MjObjectType::Body, id))
                        }
                        "xbody" => {
                            let id = *self.body_name_to_id.get(name).ok_or_else(|| {
                                ModelConversionError {
                                    message: format!(
                                        "frame sensor references unknown body '{name}'"
                                    ),
                                }
                            })?;
                            Ok((MjObjectType::XBody, id))
                        }
                        "geom" => {
                            let id = *self.geom_name_to_id.get(name).ok_or_else(|| {
                                ModelConversionError {
                                    message: format!(
                                        "frame sensor references unknown geom '{name}'"
                                    ),
                                }
                            })?;
                            Ok((MjObjectType::Geom, id))
                        }
                        "camera" => {
                            // DT-120: Camera deferred. Warn and fall through to heuristic.
                            warn!(
                                "objtype='camera' not yet supported (DT-120); \
                                 falling back to name heuristic for sensor"
                            );
                            self.resolve_frame_sensor_by_name(name)
                        }
                        other => Err(ModelConversionError {
                            message: format!(
                                "frame sensor has invalid objtype '{other}' \
                                 (expected site/body/xbody/geom/camera)"
                            ),
                        }),
                    }
                } else {
                    // No explicit objtype — infer from name lookup
                    // Priority: site → body (as XBody) → geom
                    self.resolve_frame_sensor_by_name(name)
                }
            }

            // JointActuatorFrc: objname is a joint name (hinge/slide only)
            MjSensorType::JointActuatorFrc => {
                let id = *self
                    .joint_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "jointactuatorfrc sensor references unknown joint '{name}'"
                        ),
                    })?;
                // Validate joint type: MuJoCo rejects ball/free joints
                let jnt_type = self.jnt_type[id];
                if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                    return Err(ModelConversionError {
                        message: format!(
                            "jointactuatorfrc sensor: joint '{name}' must be slide or hinge"
                        ),
                    });
                }
                Ok((MjObjectType::Joint, id))
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

            // GeomDist/GeomNormal/GeomFromTo handled in process_sensors() via
            // resolve_dual_object_side() — they never reach resolve_sensor_object().
            MjSensorType::GeomDist | MjSensorType::GeomNormal | MjSensorType::GeomFromTo => {
                unreachable!("geom distance sensors use dual-object resolution")
            }

            // Clock and User handled above (early return)
            MjSensorType::Clock | MjSensorType::User => unreachable!(),
        }
    }

    /// Resolve a frame sensor object by name lookup heuristic.
    ///
    /// Priority: site → body (as XBody) → geom.
    /// MuJoCo inference: `body=` → `mjOBJ_XBODY` (joint frame, NOT inertial frame).
    fn resolve_frame_sensor_by_name(
        &self,
        name: &str,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        if let Some(&id) = self.site_name_to_id.get(name) {
            Ok((MjObjectType::Site, id))
        } else if let Some(&id) = self.body_name_to_id.get(name) {
            // body= without objtype → XBody (MuJoCo: mjOBJ_XBODY)
            Ok((MjObjectType::XBody, id))
        } else if let Some(&id) = self.geom_name_to_id.get(name) {
            Ok((MjObjectType::Geom, id))
        } else {
            Err(ModelConversionError {
                message: format!("frame sensor references unknown object '{name}'"),
            })
        }
    }

    /// Resolve one side of a dual-object sensor (geom1/body1 or geom2/body2).
    ///
    /// Exactly one of geom_name or body_name must be Some (validated by parser).
    /// Returns (MjObjectType::Geom, id) or (MjObjectType::Body, id).
    fn resolve_dual_object_side(
        &self,
        geom_name: Option<&str>,
        body_name: Option<&str>,
        side: &str,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        match (geom_name, body_name) {
            (Some(name), None) => {
                let id = *self
                    .geom_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("distance sensor references unknown geom{side} '{name}'"),
                    })?;
                Ok((MjObjectType::Geom, id))
            }
            (None, Some(name)) => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("distance sensor references unknown body{side} '{name}'"),
                    })?;
                Ok((MjObjectType::Body, id))
            }
            _ => Err(ModelConversionError {
                message: format!(
                    "distance sensor: exactly one of (geom{side}, body{side}) \
                     must be specified"
                ),
            }),
        }
    }

    /// Resolve reftype/refname to (MjObjectType, refid).
    ///
    /// Resolution rules (matching MuJoCo):
    /// - Both absent → (None, 0) — no transform
    /// - reftype present, refname absent → (None, 0) — no transform
    /// - refname present, reftype absent → infer type from name lookup
    ///   (site → body as XBody → geom priority)
    /// - Both present → dispatch on reftype string, look up refname
    fn resolve_reference_object(
        &self,
        reftype_str: Option<&str>,
        refname: Option<&str>,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        let Some(name) = refname else {
            return Ok((MjObjectType::None, 0));
        };

        match reftype_str {
            Some("site") => {
                let id = *self
                    .site_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor refname references unknown site '{name}'"),
                    })?;
                Ok((MjObjectType::Site, id))
            }
            Some("body") => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor refname references unknown body '{name}'"),
                    })?;
                Ok((MjObjectType::Body, id))
            }
            Some("xbody") => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor refname references unknown body '{name}'"),
                    })?;
                Ok((MjObjectType::XBody, id))
            }
            Some("geom") => {
                let id = *self
                    .geom_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor refname references unknown geom '{name}'"),
                    })?;
                Ok((MjObjectType::Geom, id))
            }
            Some("camera") => {
                // DT-120: Camera not yet implemented. Warn and ignore.
                warn!(
                    "reftype='camera' not yet supported (DT-120); \
                     ignoring reftype/refname for sensor"
                );
                Ok((MjObjectType::None, 0))
            }
            Some(other) => Err(ModelConversionError {
                message: format!(
                    "sensor has invalid reftype '{other}' \
                     (expected site/body/xbody/geom/camera)"
                ),
            }),
            None => {
                // refname without reftype — infer type from name lookup
                // Priority: site → body (as XBody) → geom
                if let Some(&id) = self.site_name_to_id.get(name) {
                    Ok((MjObjectType::Site, id))
                } else if let Some(&id) = self.body_name_to_id.get(name) {
                    Ok((MjObjectType::XBody, id))
                } else if let Some(&id) = self.geom_name_to_id.get(name) {
                    Ok((MjObjectType::Geom, id))
                } else {
                    Err(ModelConversionError {
                        message: format!("sensor refname references unknown object '{name}'"),
                    })
                }
            }
        }
    }
}

/// Map `MjcfSensorType` to pipeline `MjSensorType`.
/// All standard sensor types are mapped; returns `Some(...)` for every variant.
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
        MjcfSensorType::Clock => Some(MjSensorType::Clock),
        MjcfSensorType::Jointactuatorfrc => Some(MjSensorType::JointActuatorFrc),
        MjcfSensorType::Distance => Some(MjSensorType::GeomDist),
        MjcfSensorType::Normal => Some(MjSensorType::GeomNormal),
        MjcfSensorType::Fromto => Some(MjSensorType::GeomFromTo),
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
        | MjSensorType::Magnetometer
        | MjSensorType::Clock
        | MjSensorType::GeomDist
        | MjSensorType::GeomNormal
        | MjSensorType::GeomFromTo => MjSensorDataType::Position,

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
        | MjSensorType::JointActuatorFrc
        | MjSensorType::User => MjSensorDataType::Acceleration,
    }
}
