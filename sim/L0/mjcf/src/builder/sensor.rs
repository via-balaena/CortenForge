//! Sensor processing.
//!
//! Converts MJCF `<sensor>` elements into the 13 pipeline sensor arrays.
//! Handles object resolution (joint, tendon, actuator, site, body, geom)
//! and sensor type/datatype mapping.

use sim_core::{MjObjectType, MjSensorDataType, MjSensorType};
use tracing::warn;

use super::{ModelBuilder, ModelConversionError};
use crate::types::{MjcfSensor, MjcfSensorType};

impl ModelBuilder {
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
            let (objtype, objid) = self.resolve_sensor_object(
                sensor_type,
                mjcf_sensor.objname.as_deref(),
                mjcf_sensor.objtype.as_deref(),
            )?;

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
            let sensor_id = self.sensor_name_list.len();
            self.sensor_name_list.push(if mjcf_sensor.name.is_empty() {
                None
            } else {
                self.sensor_name_to_id
                    .insert(mjcf_sensor.name.clone(), sensor_id);
                Some(mjcf_sensor.name.clone())
            });

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
                            // DT-117: Camera deferred. Warn and fall through to heuristic.
                            warn!(
                                "objtype='camera' not yet supported (DT-117); \
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
}

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
