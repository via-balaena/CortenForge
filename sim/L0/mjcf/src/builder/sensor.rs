//! Sensor processing.
//!
//! Converts MJCF `<sensor>` elements into the 17 pipeline sensor arrays.
//! Handles object resolution (joint, tendon, actuator, site, body, geom)
//! and sensor type/datatype mapping.

use sim_core::{InterpolationType, MjJointType, MjObjectType, MjSensorDataType, MjSensorType};
use tracing::warn;

use super::ModelBuilder;
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::keywords;
use crate::types::{MjcfSensor, MjcfSensorType};

impl ModelBuilder {
    /// Process sensor definitions from MJCF.
    ///
    /// Converts `MjcfSensor` objects into the 17 pipeline sensor arrays.
    /// Must be called AFTER all bodies, joints, tendons, and actuators are processed
    /// (requires name→id lookups for object resolution).
    pub(crate) fn process_sensors(&mut self, sensors: &[MjcfSensor]) -> Result<()> {
        let mut adr = 0usize;
        for (i, mjcf_sensor) in sensors.iter().enumerate() {
            self.process_sensor(mjcf_sensor, &mut adr).map_err(|e| {
                e.with_location_if_unknown(Location::nth(
                    ElementKind::Sensor,
                    Some(&mjcf_sensor.name),
                    i,
                ))
            })?;
        }
        self.nsensor = self.sensor_type.len();
        self.nsensordata = adr;
        Ok(())
    }

    /// One sensor's entries, its data at `adr`, which it advances by its size.
    fn process_sensor(&mut self, mjcf_sensor: &MjcfSensor, adr: &mut usize) -> Result<()> {
        let mjcf_sensor = self.resolver.apply_to_sensor(mjcf_sensor);
        let Some(sensor_type) = convert_sensor_type(mjcf_sensor.sensor_type) else {
            // Unknown sensor type — skip with log
            warn!(
                "Skipping unsupported sensor type '{:?}' (sensor '{}')",
                mjcf_sensor.sensor_type, mjcf_sensor.name,
            );
            return Ok(());
        };
        if sensor_type == MjSensorType::User {
            user_sensor_read_checks(
                mjcf_sensor.objtype.as_deref(),
                mjcf_sensor.objname.as_deref(),
                mjcf_sensor.dim,
                mjcf_sensor.needstage.as_deref(),
                mjcf_sensor.datatype.as_deref(),
            )?;
        }

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
                return Err(MjcfError::invalid_model(
                    "distance sensor: 1st body/geom must be different from 2nd body/geom",
                ));
            }
            if ot == MjObjectType::Body && rt == MjObjectType::Body && oi == ri {
                return Err(MjcfError::invalid_model(
                    "distance sensor: 1st body/geom must be different from 2nd body/geom",
                ));
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
        // A user sensor's object is resolved before its dimension is checked,
        // as MuJoCo's `mjCSensor::Compile` orders them.
        let (dim, datatype) = if sensor_type == MjSensorType::User {
            user_sensor_layout(&mjcf_sensor)?
        } else {
            (sensor_type.dim(), sensor_datatype(sensor_type))
        };

        // Parse interp keyword → InterpolationType (mirrors actuator.rs)
        let interp = match &mjcf_sensor.interp {
            Some(s) => keywords::interp(s)
                .ok_or_else(|| MjcfError::invalid_keyword("sensor", "interp", s))?,
            None => InterpolationType::Zoh,
        };

        // MuJoCo compiler validation: delay > 0 requires nsample > 0
        let nsample = mjcf_sensor.nsample.unwrap_or(0);
        let delay = mjcf_sensor.delay.unwrap_or(0.0);
        if delay > 0.0 && nsample <= 0 {
            return Err(MjcfError::invalid_value(
                "sensor",
                "delay",
                "setting delay > 0 without a history buffer (nsample must be > 0)",
            ));
        }

        // MuJoCo compiler validation: negative interval rejected
        let interval = mjcf_sensor.interval.unwrap_or(0.0);
        if interval < 0.0 {
            return Err(MjcfError::invalid_value(
                "sensor",
                "interval",
                "negative interval in sensor",
            ));
        }

        self.sensor_type.push(sensor_type);
        self.sensor_datatype.push(datatype);
        self.sensor_objtype.push(objtype);
        self.sensor_objid.push(objid);
        self.sensor_reftype.push(reftype);
        self.sensor_refid.push(refid);
        self.sensor_adr.push(*adr);
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
        self.sensor_user_raw.push(mjcf_sensor.user);

        *adr += dim;
        Ok(())
    }

    /// A user sensor's object, as MuJoCo 3.5.0's `ResolveReferences` takes
    /// it (measured): none when neither `objtype` nor `objname` is written
    /// (`user_sensor_read_checks` refuses one without the other); else the
    /// object of that name of one of `mju_str2Type`'s types. An empty name
    /// and a dof, which MuJoCo keeps no names of, are refused with its
    /// messages. A type sim-core keeps no object of is not supported
    /// (D-USER-SENSOR), whether or not the name exists; a word that is no
    /// type, which MuJoCo leaves an unknown object with no id, is refused
    /// (D-USER-SENSOR-OBJTYPE).
    fn resolve_user_object(
        &self,
        objtype: Option<&str>,
        objname: Option<&str>,
    ) -> Result<(MjObjectType, usize)> {
        let (Some(objtype), Some(name)) = (objtype, objname) else {
            return Ok((MjObjectType::None, 0));
        };
        let refuse = |reason: String| MjcfError::invalid_value("user", "objname", reason);
        let (kind, names, object) = match objtype {
            "body" => (ElementKind::Body, &self.body_name_to_id, MjObjectType::Body),
            "xbody" => (
                ElementKind::Body,
                &self.body_name_to_id,
                MjObjectType::XBody,
            ),
            "joint" => (
                ElementKind::Joint,
                &self.joint_name_to_id,
                MjObjectType::Joint,
            ),
            "geom" => (ElementKind::Geom, &self.geom_name_to_id, MjObjectType::Geom),
            "site" => (ElementKind::Site, &self.site_name_to_id, MjObjectType::Site),
            "tendon" => (
                ElementKind::Tendon,
                &self.tendon_name_to_id,
                MjObjectType::Tendon,
            ),
            "actuator" => (
                ElementKind::Actuator,
                &self.actuator_name_to_id,
                MjObjectType::Actuator,
            ),
            // MuJoCo finds no object of a dof by name (`user_model.cc:854-877`).
            "dof" => {
                return Err(refuse(format!(
                    "unrecognized name '{name}' of sensorized object"
                )));
            }
            "camera" | "light" | "flex" | "mesh" | "skin" | "hfield" | "texture" | "material"
            | "pair" | "exclude" | "equality" | "sensor" | "numeric" | "text" | "tuple" | "key"
            | "plugin" => {
                return Err(MjcfError::unsupported(format!(
                    "a user sensor on a {objtype}"
                )));
            }
            other => return Err(MjcfError::invalid_keyword("user", "objtype", other)),
        };
        if name.is_empty() {
            return Err(refuse("missing name of sensorized object in sensor".into()));
        }
        let id = *names
            .get(name)
            .ok_or_else(|| MjcfError::undefined(kind, name))?;
        Ok((object, id))
    }

    /// Returns (MjObjectType, objid) for a given sensor type and MJCF objname.
    ///
    /// `objtype_str` is the explicit `objtype` attribute from MJCF. Frame and
    /// user sensors honor it; all other sensor types ignore it.
    fn resolve_sensor_object(
        &self,
        sensor_type: MjSensorType,
        objname: Option<&str>,
        objtype_str: Option<&str>,
    ) -> Result<(MjObjectType, usize)> {
        if sensor_type == MjSensorType::User {
            return self.resolve_user_object(objtype_str, objname);
        }
        // A clock has no object reference
        if sensor_type == MjSensorType::Clock {
            return Ok((MjObjectType::None, 0));
        }

        let name = objname.ok_or_else(|| MjcfError::missing_attribute("sensor", "objname"))?;

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
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Joint, name))?;
                Ok((MjObjectType::Joint, id))
            }

            // Tendon sensors: objname is a tendon name
            MjSensorType::TendonPos
            | MjSensorType::TendonVel
            | MjSensorType::TendonLimitFrc
            | MjSensorType::TendonActuatorFrc => {
                let id = *self
                    .tendon_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Tendon, name))?;
                Ok((MjObjectType::Tendon, id))
            }

            // Actuator sensors: objname is an actuator name
            MjSensorType::ActuatorPos | MjSensorType::ActuatorVel | MjSensorType::ActuatorFrc => {
                let id = *self
                    .actuator_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Actuator, name))?;
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
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Site, name))?;
                Ok((MjObjectType::Site, id))
            }

            // Touch: always site-based (MuJoCo: sensor_objtype = mjOBJ_SITE).
            // objtype attribute is silently ignored for non-frame sensors.
            MjSensorType::Touch => {
                let site_id = *self
                    .site_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Site, name))?;
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
                            let id = *self
                                .site_name_to_id
                                .get(name)
                                .ok_or_else(|| MjcfError::undefined(ElementKind::Site, name))?;
                            Ok((MjObjectType::Site, id))
                        }
                        "body" => {
                            let id = *self
                                .body_name_to_id
                                .get(name)
                                .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                            Ok((MjObjectType::Body, id))
                        }
                        "xbody" => {
                            let id = *self
                                .body_name_to_id
                                .get(name)
                                .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                            Ok((MjObjectType::XBody, id))
                        }
                        "geom" => {
                            let id = *self
                                .geom_name_to_id
                                .get(name)
                                .ok_or_else(|| MjcfError::undefined(ElementKind::Geom, name))?;
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
                        other => Err(MjcfError::invalid_keyword("sensor", "objtype", other)),
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
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Joint, name))?;
                // Validate joint type: MuJoCo rejects ball/free joints
                let jnt_type = self.jnt_type[id];
                if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                    return Err(MjcfError::invalid_model(format!(
                        "jointactuatorfrc sensor: joint '{name}' must be slide or hinge"
                    )));
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
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                Ok((MjObjectType::Body, id))
            }

            // GeomDist/GeomNormal/GeomFromTo handled in process_sensors() via
            // resolve_dual_object_side() — they never reach resolve_sensor_object().
            MjSensorType::GeomDist | MjSensorType::GeomNormal | MjSensorType::GeomFromTo => {
                unreachable!("geom distance sensors use dual-object resolution")
            }

            // Clock, User, and Plugin handled above (early return)
            MjSensorType::Clock | MjSensorType::User | MjSensorType::Plugin => unreachable!(),
        }
    }

    /// Resolve a frame sensor object by name lookup heuristic.
    ///
    /// Priority: site → body (as XBody) → geom.
    /// MuJoCo inference: `body=` → `mjOBJ_XBODY` (joint frame, NOT inertial frame).
    fn resolve_frame_sensor_by_name(&self, name: &str) -> Result<(MjObjectType, usize)> {
        if let Some(&id) = self.site_name_to_id.get(name) {
            Ok((MjObjectType::Site, id))
        } else if let Some(&id) = self.body_name_to_id.get(name) {
            // body= without objtype → XBody (MuJoCo: mjOBJ_XBODY)
            Ok((MjObjectType::XBody, id))
        } else if let Some(&id) = self.geom_name_to_id.get(name) {
            Ok((MjObjectType::Geom, id))
        } else {
            Err(MjcfError::invalid_value(
                "sensor",
                "objname",
                format!("unknown object '{name}'"),
            ))
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
    ) -> Result<(MjObjectType, usize)> {
        match (geom_name, body_name) {
            (Some(name), None) => {
                let id = *self
                    .geom_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Geom, name))?;
                Ok((MjObjectType::Geom, id))
            }
            (None, Some(name)) => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                Ok((MjObjectType::Body, id))
            }
            _ => Err(MjcfError::invalid_model(format!(
                "distance sensor: exactly one of (geom{side}, body{side}) must be specified"
            ))),
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
    ) -> Result<(MjObjectType, usize)> {
        let Some(name) = refname else {
            return Ok((MjObjectType::None, 0));
        };

        match reftype_str {
            Some("site") => {
                let id = *self
                    .site_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Site, name))?;
                Ok((MjObjectType::Site, id))
            }
            Some("body") => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                Ok((MjObjectType::Body, id))
            }
            Some("xbody") => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Body, name))?;
                Ok((MjObjectType::XBody, id))
            }
            Some("geom") => {
                let id = *self
                    .geom_name_to_id
                    .get(name)
                    .ok_or_else(|| MjcfError::undefined(ElementKind::Geom, name))?;
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
            Some(other) => Err(MjcfError::invalid_keyword("sensor", "reftype", other)),
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
                    Err(MjcfError::invalid_value(
                        "sensor",
                        "refname",
                        format!("unknown object '{name}'"),
                    ))
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
        MjcfSensorType::Tendonactuatorfrc => Some(MjSensorType::TendonActuatorFrc),
        MjcfSensorType::Distance => Some(MjSensorType::GeomDist),
        MjcfSensorType::Normal => Some(MjSensorType::GeomNormal),
        MjcfSensorType::Fromto => Some(MjSensorType::GeomFromTo),
    }
}

/// A user sensor's attributes checked as MuJoCo 3.5.0's reader checks them,
/// in its order (`xml_native_reader.cc:4419-4438`, measured): `objtype` and
/// `objname` written together, `dim` written, then the `needstage` and
/// `datatype` words (`stage_map`, `datatype_map`). The parser checks a parsed
/// sensor, the build one built in code.
pub fn user_sensor_read_checks(
    objtype: Option<&str>,
    objname: Option<&str>,
    dim: Option<i32>,
    needstage: Option<&str>,
    datatype: Option<&str>,
) -> Result<()> {
    match (objtype, objname) {
        (Some(t), None) => {
            return Err(MjcfError::invalid_value(
                "user",
                "objtype",
                format!("objtype '{t}' given but objname is missing"),
            ));
        }
        (None, Some(n)) => {
            return Err(MjcfError::invalid_value(
                "user",
                "objname",
                format!("objname '{n}' given but objtype is missing"),
            ));
        }
        _ => {}
    }
    if dim.is_none() {
        return Err(MjcfError::missing_attribute("user", "dim"));
    }
    if let Some(word) = needstage.filter(|w| keywords::sensor_stage(w).is_none()) {
        return Err(MjcfError::invalid_keyword("user", "needstage", word));
    }
    if let Some(word) = datatype.filter(|w| keywords::sensor_data_kind(w).is_none()) {
        return Err(MjcfError::invalid_keyword("user", "datatype", word));
    }
    Ok(())
}

/// A user sensor's number of values and its stage, its object resolved, as
/// MuJoCo 3.5.0's `mjCSensor::Compile` checks them in its order (measured):
/// `dim` not negative, 0 included; an `axis` of 3 values and a `quaternion`
/// of 4 (`user_objects.cc:7582-7590`); `needstage` `pos`, `vel` or `acc`
/// (the default). Then what sim-core cannot make is not supported
/// (D-USER-SENSOR): a `needstage` of `none`, which it has no stage for, and
/// a `datatype` other than `real`, which it does not keep per sensor.
fn user_sensor_layout(sensor: &MjcfSensor) -> Result<(usize, MjSensorDataType)> {
    let dim = sensor
        .dim
        .ok_or_else(|| MjcfError::missing_attribute("user", "dim"))?;
    let dim = usize::try_from(dim).map_err(|_| {
        MjcfError::invalid_value("user", "dim", "sensor dim must be non-negative in sensor")
    })?;
    let kind = sensor.datatype.as_deref().unwrap_or("real");
    for (word, needs, name) in [("axis", 3, "AXIS"), ("quaternion", 4, "QUATERNION")] {
        if kind == word && dim != needs {
            return Err(MjcfError::invalid_value(
                "user",
                "dim",
                format!("datatype {name} requires dim={needs} in sensor"),
            ));
        }
    }
    let stage = match sensor.needstage.as_deref().unwrap_or("acc") {
        "pos" => MjSensorDataType::Position,
        "vel" => MjSensorDataType::Velocity,
        "acc" => MjSensorDataType::Acceleration,
        "none" => {
            return Err(MjcfError::unsupported("a user sensor's needstage \"none\""));
        }
        other => return Err(MjcfError::invalid_keyword("user", "needstage", other)),
    };
    match kind {
        "real" => {}
        "positive" | "axis" | "quaternion" => {
            return Err(MjcfError::unsupported(format!(
                "a user sensor's datatype \"{kind}\""
            )));
        }
        other => return Err(MjcfError::invalid_keyword("user", "datatype", other)),
    }
    Ok((dim, stage))
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
        | MjSensorType::TendonActuatorFrc
        | MjSensorType::User
        | MjSensorType::Plugin => MjSensorDataType::Acceleration,
    }
}
