//! Auto-discovery sensor visualization using Bevy gizmos.
//!
//! Replaces the old manual-push `SensorVisualData` / `SensorVisualType` API with
//! an auto-discovery system that reads directly from `Model` / `Data`.
//!
//! # Architecture
//!
//! 1. `update_sensor_visualization` — reads `PhysicsModel` + `PhysicsData`,
//!    builds a `Vec<SensorVizEntry>` with world-frame gizmo descriptions.
//! 2. `draw_sensor_gizmos` — renders each entry via Bevy Gizmos API.
//!
//! All 38 `MjSensorType` variants are mapped to one of six gizmo kinds
//! (`Frame`, `Arrow`, `Highlight`, `Ray`, `DistanceLine`, `HudOnly`).

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters
#![allow(clippy::cast_possible_truncation)] // f64 -> f32 intentional for Bevy
#![allow(clippy::module_name_repetitions)]

use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};
use sim_core::{MjObjectType, MjSensorType, Model};

use crate::convert::{quat_from_physics_matrix, vec3_from_vector};
use crate::model_data::{PhysicsData, PhysicsModel};
use crate::resources::ViewerConfig;

// ============================================================================
// Types
// ============================================================================

/// What to draw for a sensor reading.
#[derive(Debug, Clone)]
pub enum SensorGizmo {
    /// RGB coordinate frame at a site (Accelerometer, Gyro, `BallQuat`, `FrameQuat`).
    Frame {
        /// World-space position.
        position: Vector3<f64>,
        /// 3×3 rotation matrix (columns = world-frame axes).
        orientation: Matrix3<f64>,
    },
    /// Arrow from an origin in a direction.
    /// `direction` is in **world frame** (pre-rotated by the update system).
    Arrow {
        /// World-space origin.
        origin: Vector3<f64>,
        /// World-frame direction (magnitude = value magnitude × scale).
        direction: Vector3<f64>,
        /// Arrow color.
        color: Color,
    },
    /// Scalar highlight (Touch — sphere that glows when active).
    Highlight {
        /// World-space position.
        position: Vector3<f64>,
        /// 0.0 = inactive, 1.0 = max.
        intensity: f64,
    },
    /// Ray with hit point (Rangefinder).
    Ray {
        /// World-space origin.
        origin: Vector3<f64>,
        /// World-frame direction (unit vector).
        direction: Vector3<f64>,
        /// Measured distance.
        distance: f64,
    },
    /// Distance line between two points (`GeomDist`, `GeomFromTo`).
    DistanceLine {
        /// First world-space point.
        from: Vector3<f64>,
        /// Second world-space point.
        to: Vector3<f64>,
        /// Signed distance between the two.
        distance: f64,
    },
    /// No gizmo — scalar sensors displayed only on HUD.
    HudOnly,
}

/// One sensor's visualization state for a single frame.
#[derive(Debug, Clone)]
pub struct SensorVizEntry {
    /// Index into `model.sensor_*` arrays.
    pub sensor_id: usize,
    /// Human-readable name (from `model.sensor_name`).
    pub name: String,
    /// The sensor's type enum.
    pub sensor_type: MjSensorType,
    /// What to draw.
    pub gizmo: SensorGizmo,
    /// Raw sensor data this frame.
    pub value: Vec<f64>,
}

/// Auto-populated sensor visualization resource.
///
/// Cleared and rebuilt each frame by [`update_sensor_visualization`].
#[derive(Resource, Default)]
pub struct SensorVisualization {
    entries: Vec<SensorVizEntry>,
}

impl SensorVisualization {
    /// Get all sensor viz entries.
    #[must_use]
    pub fn entries(&self) -> &[SensorVizEntry] {
        &self.entries
    }

    /// Check if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Number of sensors.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }
}

// ============================================================================
// Systems
// ============================================================================

/// Reads `Model` sensor metadata + `Data.sensordata`, populates
/// [`SensorVisualization`] with world-frame gizmos.
pub fn update_sensor_visualization(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    config: Res<ViewerConfig>,
    mut viz: ResMut<SensorVisualization>,
) {
    if !config.show_sensors {
        viz.entries.clear();
        return;
    }

    viz.entries.clear();
    viz.entries.reserve(model.nsensor);

    for sensor_id in 0..model.nsensor {
        let value = data.sensor_data(&model, sensor_id).to_vec();
        let gizmo = sensor_type_to_gizmo(&model, &data, sensor_id, &value);
        viz.entries.push(SensorVizEntry {
            sensor_id,
            name: model.sensor_name[sensor_id].clone().unwrap_or_default(),
            sensor_type: model.sensor_type[sensor_id],
            gizmo,
            value,
        });
    }
}

/// Renders each [`SensorVizEntry`] via Bevy Gizmos.
pub fn draw_sensor_gizmos(
    mut gizmos: Gizmos,
    viz: Res<SensorVisualization>,
    config: Res<ViewerConfig>,
) {
    if !config.show_sensors {
        return;
    }

    for entry in viz.entries() {
        match &entry.gizmo {
            SensorGizmo::Frame {
                position,
                orientation,
            } => {
                let pos = vec3_from_vector(position);
                let rot = quat_from_physics_matrix(orientation);
                let axis_len = config.sensor_axis_length;

                let x = rot * Vec3::X * axis_len;
                let y = rot * Vec3::Y * axis_len;
                let z = rot * Vec3::Z * axis_len;

                gizmos.arrow(pos, pos + x, Color::srgb(1.0, 0.0, 0.0));
                gizmos.arrow(pos, pos + y, Color::srgb(0.0, 1.0, 0.0));
                gizmos.arrow(pos, pos + z, Color::srgb(0.0, 0.0, 1.0));

                gizmos.sphere(
                    Isometry3d::from_translation(pos),
                    config.sensor_axis_length * 0.15,
                    config.colors.sensor_imu,
                );
            }
            SensorGizmo::Arrow {
                origin,
                direction,
                color,
            } => {
                let start = vec3_from_vector(origin);
                let dir = vec3_from_vector(direction) * config.sensor_force_scale;
                if dir.length() > 0.001 {
                    gizmos.arrow(start, start + dir, *color);
                }
                gizmos.sphere(
                    Isometry3d::from_translation(start),
                    config.sensor_axis_length * 0.08,
                    *color,
                );
            }
            SensorGizmo::Highlight {
                position,
                intensity,
            } => {
                let pos = vec3_from_vector(position);
                let t = intensity.clamp(0.0, 1.0) as f32;
                let color = if t > 0.01 {
                    config.colors.sensor_touch_active
                } else {
                    config.colors.sensor_touch_active.with_alpha(0.3)
                };
                let radius = config.sensor_axis_length * (0.1 + 0.1 * t);
                gizmos.sphere(Isometry3d::from_translation(pos), radius, color);
            }
            SensorGizmo::Ray {
                origin,
                direction,
                distance,
            } => {
                let start = vec3_from_vector(origin);
                let dir = vec3_from_vector(direction);
                let end = start + dir * (*distance as f32);

                gizmos.line(start, end, config.colors.sensor_imu.with_alpha(0.6));
                gizmos.sphere(
                    Isometry3d::from_translation(end),
                    config.sensor_axis_length * 0.08,
                    config.colors.sensor_imu,
                );
            }
            SensorGizmo::DistanceLine { from, to, .. } => {
                let a = vec3_from_vector(from);
                let b = vec3_from_vector(to);
                gizmos.line(a, b, config.colors.sensor_force_torque);
                gizmos.sphere(
                    Isometry3d::from_translation(a),
                    config.sensor_axis_length * 0.06,
                    config.colors.sensor_force_torque,
                );
                gizmos.sphere(
                    Isometry3d::from_translation(b),
                    config.sensor_axis_length * 0.06,
                    config.colors.sensor_force_torque,
                );
            }
            SensorGizmo::HudOnly => {}
        }
    }
}

// ============================================================================
// Mapping
// ============================================================================

/// Map a sensor to its gizmo representation.
///
/// Handles local-frame → world-frame rotation for site-attached sensors,
/// world-frame passthrough for Frame* sensors, and dual-object geometry for
/// GeomDist/GeomNormal/GeomFromTo.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn sensor_type_to_gizmo(
    model: &Model,
    data: &sim_core::Data,
    sensor_id: usize,
    value: &[f64],
) -> SensorGizmo {
    let stype = model.sensor_type[sensor_id];
    let objid = model.sensor_objid[sensor_id];

    match stype {
        // ── Frame gizmos (local-frame sensors) ──────────────────────────
        MjSensorType::Accelerometer | MjSensorType::Gyro => {
            // Site-attached, reading in site local frame → show site frame
            let site_id = objid;
            let pos = site_position(data, site_id);
            let mat = site_rotation(data, site_id);
            SensorGizmo::Frame {
                position: pos,
                orientation: mat,
            }
        }

        MjSensorType::BallQuat => {
            // 4D quaternion → rotation matrix, displayed at joint body position
            if value.len() >= 4 {
                let q = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    value[0], value[1], value[2], value[3],
                ));
                let body_id = if objid < model.njnt {
                    model.jnt_body[objid]
                } else {
                    0
                };
                SensorGizmo::Frame {
                    position: data.xpos[body_id],
                    orientation: *q.to_rotation_matrix().matrix(),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::FrameQuat => {
            // 4D quaternion from frame sensor
            if value.len() >= 4 {
                let q = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    value[0], value[1], value[2], value[3],
                ));
                let pos = object_position(model, data, sensor_id);
                SensorGizmo::Frame {
                    position: pos,
                    orientation: *q.to_rotation_matrix().matrix(),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── Arrow gizmos (local-frame → rotate by site_xmat) ───────────
        MjSensorType::Force | MjSensorType::Torque => {
            if value.len() >= 3 {
                let local = Vector3::new(value[0], value[1], value[2]);
                let site_id = objid;
                let world = site_rotation(data, site_id) * local;
                let color = Color::srgb(0.8, 0.4, 0.8); // sensor_force_torque
                SensorGizmo::Arrow {
                    origin: site_position(data, site_id),
                    direction: world,
                    color,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::Velocimeter => {
            if value.len() >= 3 {
                let local = Vector3::new(value[0], value[1], value[2]);
                let site_id = objid;
                let world = site_rotation(data, site_id) * local;
                SensorGizmo::Arrow {
                    origin: site_position(data, site_id),
                    direction: world,
                    color: Color::srgb(0.2, 0.8, 1.0),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::Magnetometer => {
            if value.len() >= 3 {
                let local = Vector3::new(value[0], value[1], value[2]);
                let site_id = objid;
                let world = site_rotation(data, site_id) * local;
                SensorGizmo::Arrow {
                    origin: site_position(data, site_id),
                    direction: world,
                    color: Color::srgb(0.8, 0.0, 0.8),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::BallAngVel => {
            // Angular velocity of ball joint, in joint local frame
            if value.len() >= 3 {
                let local = Vector3::new(value[0], value[1], value[2]);
                let body_id = if objid < model.njnt {
                    model.jnt_body[objid]
                } else {
                    0
                };
                let world = data.xmat[body_id] * local;
                SensorGizmo::Arrow {
                    origin: data.xpos[body_id],
                    direction: world,
                    color: Color::srgb(1.0, 0.2, 0.8),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── Arrow gizmos (world-frame or relative-frame) ───────────────
        MjSensorType::FrameLinVel
        | MjSensorType::FrameAngVel
        | MjSensorType::FrameLinAcc
        | MjSensorType::FrameAngAcc => {
            if value.len() >= 3 {
                let raw = Vector3::new(value[0], value[1], value[2]);
                let world = frame_sensor_to_world(model, data, sensor_id, raw);
                let pos = object_position(model, data, sensor_id);
                let color = match stype {
                    MjSensorType::FrameLinVel | MjSensorType::FrameLinAcc => {
                        Color::srgb(0.2, 0.8, 1.0)
                    }
                    _ => Color::srgb(1.0, 0.2, 0.8),
                };
                SensorGizmo::Arrow {
                    origin: pos,
                    direction: world,
                    color,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
            if value.len() >= 3 {
                // World-frame direction vector
                let dir = Vector3::new(value[0], value[1], value[2]);
                let pos = object_position(model, data, sensor_id);
                let color = match stype {
                    MjSensorType::FrameXAxis => Color::srgb(1.0, 0.0, 0.0),
                    MjSensorType::FrameYAxis => Color::srgb(0.0, 1.0, 0.0),
                    _ => Color::srgb(0.0, 0.0, 1.0),
                };
                SensorGizmo::Arrow {
                    origin: pos,
                    direction: dir,
                    color,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── Arrow gizmos (subtree sensors — world vectors at COM) ──────
        MjSensorType::SubtreeLinVel | MjSensorType::SubtreeAngMom => {
            if value.len() >= 3 {
                let dir = Vector3::new(value[0], value[1], value[2]);
                let body_id = objid;
                let com = if body_id < data.subtree_com.len() {
                    data.subtree_com[body_id]
                } else {
                    Vector3::zeros()
                };
                let color = match stype {
                    MjSensorType::SubtreeLinVel => Color::srgb(0.2, 0.8, 1.0),
                    _ => Color::srgb(1.0, 0.2, 0.8),
                };
                SensorGizmo::Arrow {
                    origin: com,
                    direction: dir,
                    color,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── Highlight gizmo (Touch) ────────────────────────────────────
        MjSensorType::Touch => {
            let site_id = objid;
            let intensity = if value.is_empty() {
                0.0
            } else {
                value[0].abs()
            };
            // Normalize: typical touch forces are 0-100N, map to 0-1
            let normalized = (intensity / 100.0).min(1.0);
            SensorGizmo::Highlight {
                position: site_position(data, site_id),
                intensity: normalized,
            }
        }

        // ── Highlight gizmo (FramePos, SubtreeCom — world position) ───
        MjSensorType::FramePos | MjSensorType::SubtreeCom => {
            if value.len() >= 3 {
                SensorGizmo::Highlight {
                    position: Vector3::new(value[0], value[1], value[2]),
                    intensity: 0.5,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── Ray gizmo (Rangefinder) ────────────────────────────────────
        MjSensorType::Rangefinder => {
            let site_id = objid;
            let pos = site_position(data, site_id);
            let mat = site_rotation(data, site_id);
            // Rangefinder shoots along site Z-axis (local)
            let world_dir = mat * Vector3::z();
            let dist = if value.is_empty() { 0.0 } else { value[0] };
            SensorGizmo::Ray {
                origin: pos,
                direction: world_dir,
                distance: dist.max(0.0),
            }
        }

        // ── DistanceLine gizmo (dual-object geometry sensors) ──────────
        MjSensorType::GeomDist => {
            // Scalar distance; endpoints from geom centers
            let geom1 = objid;
            let geom2 = model.sensor_refid[sensor_id];
            let p1 = geom_position(data, geom1);
            let p2 = geom_position(data, geom2);
            let dist = if value.is_empty() { 0.0 } else { value[0] };
            SensorGizmo::DistanceLine {
                from: p1,
                to: p2,
                distance: dist,
            }
        }

        MjSensorType::GeomFromTo => {
            // 6D: from_xyz + to_xyz in world coords
            if value.len() >= 6 {
                let from = Vector3::new(value[0], value[1], value[2]);
                let to = Vector3::new(value[3], value[4], value[5]);
                let dist = (to - from).norm();
                SensorGizmo::DistanceLine {
                    from,
                    to,
                    distance: dist,
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        MjSensorType::GeomNormal => {
            // 3D world-frame normal at nearest point
            if value.len() >= 3 {
                let normal = Vector3::new(value[0], value[1], value[2]);
                // Position: midpoint between the two geoms
                let geom1 = objid;
                let geom2 = model.sensor_refid[sensor_id];
                let p1 = geom_position(data, geom1);
                let p2 = geom_position(data, geom2);
                let midpoint = (p1 + p2) * 0.5;
                SensorGizmo::Arrow {
                    origin: midpoint,
                    direction: normal,
                    color: Color::srgb(0.2, 1.0, 0.2),
                }
            } else {
                SensorGizmo::HudOnly
            }
        }

        // ── HudOnly (scalar sensors) ───────────────────────────────────
        MjSensorType::JointPos
        | MjSensorType::JointVel
        | MjSensorType::TendonPos
        | MjSensorType::TendonVel
        | MjSensorType::ActuatorPos
        | MjSensorType::ActuatorVel
        | MjSensorType::ActuatorFrc
        | MjSensorType::JointLimitFrc
        | MjSensorType::TendonLimitFrc
        | MjSensorType::JointActuatorFrc
        | MjSensorType::Clock
        | MjSensorType::User
        | MjSensorType::Plugin => SensorGizmo::HudOnly,
    }
}

// ============================================================================
// Helpers
// ============================================================================

/// Get site position in world frame.
fn site_position(data: &sim_core::Data, site_id: usize) -> Vector3<f64> {
    if site_id < data.site_xpos.len() {
        data.site_xpos[site_id]
    } else {
        Vector3::zeros()
    }
}

/// Get site rotation matrix in world frame.
fn site_rotation(data: &sim_core::Data, site_id: usize) -> Matrix3<f64> {
    if site_id < data.site_xmat.len() {
        data.site_xmat[site_id]
    } else {
        Matrix3::identity()
    }
}

/// Get geom position in world frame.
fn geom_position(data: &sim_core::Data, geom_id: usize) -> Vector3<f64> {
    if geom_id < data.geom_xpos.len() {
        data.geom_xpos[geom_id]
    } else {
        Vector3::zeros()
    }
}

/// Get the world-frame position for the object a sensor is attached to.
fn object_position(model: &Model, data: &sim_core::Data, sensor_id: usize) -> Vector3<f64> {
    let objtype = model.sensor_objtype[sensor_id];
    let objid = model.sensor_objid[sensor_id];
    object_xpos(data, objtype, objid)
}

/// Look up world-frame position for an (objtype, objid) pair.
fn object_xpos(data: &sim_core::Data, objtype: MjObjectType, objid: usize) -> Vector3<f64> {
    match objtype {
        MjObjectType::Body => {
            if objid < data.xipos.len() {
                data.xipos[objid]
            } else {
                Vector3::zeros()
            }
        }
        MjObjectType::XBody => {
            if objid < data.xpos.len() {
                data.xpos[objid]
            } else {
                Vector3::zeros()
            }
        }
        MjObjectType::Geom => geom_position(data, objid),
        MjObjectType::Site => site_position(data, objid),
        _ => Vector3::zeros(),
    }
}

/// Look up world-frame rotation matrix for an (objtype, objid) pair.
fn object_xmat(data: &sim_core::Data, objtype: MjObjectType, objid: usize) -> Matrix3<f64> {
    match objtype {
        MjObjectType::Body => {
            if objid < data.ximat.len() {
                data.ximat[objid]
            } else {
                Matrix3::identity()
            }
        }
        MjObjectType::XBody => {
            if objid < data.xmat.len() {
                data.xmat[objid]
            } else {
                Matrix3::identity()
            }
        }
        MjObjectType::Geom => {
            if objid < data.geom_xmat.len() {
                data.geom_xmat[objid]
            } else {
                Matrix3::identity()
            }
        }
        MjObjectType::Site => site_rotation(data, objid),
        _ => Matrix3::identity(),
    }
}

/// Convert a Frame* sensor reading to world frame.
///
/// - If `sensor_reftype == None` → already world-frame, return as-is.
/// - Otherwise → reading is in reference object's frame, rotate by ref xmat.
fn frame_sensor_to_world(
    model: &Model,
    data: &sim_core::Data,
    sensor_id: usize,
    raw: Vector3<f64>,
) -> Vector3<f64> {
    let reftype = model.sensor_reftype[sensor_id];
    if reftype == MjObjectType::None {
        // World-frame: no rotation needed
        raw
    } else {
        // Relative-frame: rotate by reference object's world rotation
        let refid = model.sensor_refid[sensor_id];
        let ref_mat = object_xmat(data, reftype, refid);
        ref_mat * raw
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::panic)]
mod tests {
    use super::*;

    /// Build a minimal Model + Data for testing sensor viz.
    fn test_model_data(sensor_types: &[MjSensorType]) -> (Model, sim_core::Data) {
        use sim_core::MjSensorDataType;

        let nsensor = sensor_types.len();
        // Compute sensor addresses and total data size
        let mut adr = Vec::with_capacity(nsensor);
        let mut dims = Vec::with_capacity(nsensor);
        let mut total = 0usize;
        for stype in sensor_types {
            adr.push(total);
            let dim = sensor_dim(*stype);
            dims.push(dim);
            total += dim;
        }

        let mut model = Model::empty();
        model.nsensor = nsensor;
        model.nsensordata = total;
        model.sensor_type = sensor_types.to_vec();
        model.sensor_adr = adr;
        model.sensor_dim = dims;
        model.sensor_name = vec![Some("test_sensor".to_string()); nsensor];
        model.sensor_datatype = vec![MjSensorDataType::Position; nsensor];
        model.sensor_objtype = vec![MjObjectType::Site; nsensor];
        model.sensor_objid = vec![0; nsensor];
        model.sensor_reftype = vec![MjObjectType::None; nsensor];
        model.sensor_refid = vec![0; nsensor];
        model.sensor_noise = vec![0.0; nsensor];
        model.sensor_cutoff = vec![0.0; nsensor];
        model.sensor_nsample = vec![0; nsensor];
        model.sensor_interp = vec![sim_core::InterpolationType::default(); nsensor];
        model.sensor_historyadr = vec![0; nsensor];
        model.sensor_delay = vec![0.0; nsensor];
        model.sensor_interval = vec![(0.0, 0.0); nsensor];
        model.sensor_plugin = vec![None; nsensor];

        // Provide 1 site, 1 geom for sensors to attach to
        model.nsite = 1;
        model.site_body = vec![0];
        model.site_pos = vec![Vector3::new(1.0, 2.0, 3.0)];
        model.site_quat = vec![nalgebra::UnitQuaternion::identity()];
        model.site_size = vec![Vector3::new(0.01, 0.01, 0.01)];
        model.site_name = vec![Some("test_site".to_string())];
        model.site_type = vec![sim_core::GeomType::Sphere];
        model.site_group = vec![0];
        model.site_rgba = vec![[1.0, 1.0, 1.0, 1.0]];

        model.ngeom = 1;
        model.body_geom_num = vec![1];
        model.geom_body = vec![0];
        model.geom_pos = vec![Vector3::zeros()];
        model.geom_quat = vec![nalgebra::UnitQuaternion::identity()];
        model.geom_size = vec![Vector3::new(0.1, 0.1, 0.1)];
        model.geom_type = vec![sim_core::GeomType::Sphere];
        model.geom_name = vec![Some("test_geom".to_string())];
        model.geom_rbound = vec![0.1];

        // make_data creates arrays sized for the model
        let mut data = model.make_data();
        // Fill sensordata with 1.0 values
        data.sensordata = nalgebra::DVector::from_element(total, 1.0);
        // Set site pose
        if !data.site_xpos.is_empty() {
            data.site_xpos[0] = Vector3::new(1.0, 2.0, 3.0);
        }

        (model, data)
    }

    /// Return the expected sensor dimension for a given type.
    fn sensor_dim(stype: MjSensorType) -> usize {
        match stype {
            MjSensorType::Accelerometer
            | MjSensorType::Velocimeter
            | MjSensorType::Gyro
            | MjSensorType::Force
            | MjSensorType::Torque
            | MjSensorType::Magnetometer
            | MjSensorType::BallAngVel
            | MjSensorType::FramePos
            | MjSensorType::FrameXAxis
            | MjSensorType::FrameYAxis
            | MjSensorType::FrameZAxis
            | MjSensorType::FrameLinVel
            | MjSensorType::FrameAngVel
            | MjSensorType::FrameLinAcc
            | MjSensorType::FrameAngAcc
            | MjSensorType::SubtreeCom
            | MjSensorType::SubtreeLinVel
            | MjSensorType::SubtreeAngMom
            | MjSensorType::GeomNormal => 3,

            MjSensorType::BallQuat | MjSensorType::FrameQuat => 4,
            MjSensorType::GeomFromTo => 6,

            // All 1D sensors (Touch, Rangefinder, JointPos, JointVel, etc.)
            MjSensorType::Touch
            | MjSensorType::Rangefinder
            | MjSensorType::JointPos
            | MjSensorType::JointVel
            | MjSensorType::TendonPos
            | MjSensorType::TendonVel
            | MjSensorType::ActuatorPos
            | MjSensorType::ActuatorVel
            | MjSensorType::ActuatorFrc
            | MjSensorType::JointLimitFrc
            | MjSensorType::TendonLimitFrc
            | MjSensorType::JointActuatorFrc
            | MjSensorType::GeomDist
            | MjSensorType::Clock
            | MjSensorType::User
            | MjSensorType::Plugin => 1,
        }
    }

    #[test]
    fn sensor_type_to_gizmo_all_types_no_panic() {
        let all_types = [
            MjSensorType::Touch,
            MjSensorType::Accelerometer,
            MjSensorType::Velocimeter,
            MjSensorType::Gyro,
            MjSensorType::Force,
            MjSensorType::Torque,
            MjSensorType::Magnetometer,
            MjSensorType::Rangefinder,
            MjSensorType::JointPos,
            MjSensorType::JointVel,
            MjSensorType::BallQuat,
            MjSensorType::BallAngVel,
            MjSensorType::TendonPos,
            MjSensorType::TendonVel,
            MjSensorType::ActuatorPos,
            MjSensorType::ActuatorVel,
            MjSensorType::ActuatorFrc,
            MjSensorType::JointLimitFrc,
            MjSensorType::TendonLimitFrc,
            MjSensorType::FramePos,
            MjSensorType::FrameQuat,
            MjSensorType::FrameXAxis,
            MjSensorType::FrameYAxis,
            MjSensorType::FrameZAxis,
            MjSensorType::FrameLinVel,
            MjSensorType::FrameAngVel,
            MjSensorType::FrameLinAcc,
            MjSensorType::FrameAngAcc,
            MjSensorType::SubtreeCom,
            MjSensorType::SubtreeLinVel,
            MjSensorType::SubtreeAngMom,
            MjSensorType::Clock,
            MjSensorType::JointActuatorFrc,
            MjSensorType::GeomDist,
            MjSensorType::GeomNormal,
            MjSensorType::GeomFromTo,
            MjSensorType::User,
            MjSensorType::Plugin,
        ];

        // Test each type individually to get clear failure messages
        for stype in &all_types {
            let (model, data) = test_model_data(&[*stype]);
            let value = data.sensor_data(&model, 0).to_vec();
            let _gizmo = sensor_type_to_gizmo(&model, &data, 0, &value);
        }
    }

    #[test]
    fn local_frame_sensors_produce_rotated_arrows() {
        // 90° rotation about Z: X→Y, Y→-X
        let rot_z_90 = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

        let (model, mut data) = test_model_data(&[MjSensorType::Force]);
        data.site_xmat[0] = rot_z_90;
        // Force sensor value: [1, 0, 0] in local frame
        data.sensordata = nalgebra::DVector::from_vec(vec![1.0, 0.0, 0.0]);

        let value = data.sensor_data(&model, 0).to_vec();
        let gizmo = sensor_type_to_gizmo(&model, &data, 0, &value);

        match gizmo {
            SensorGizmo::Arrow { direction, .. } => {
                // Local [1,0,0] rotated by 90° about Z → world [0,1,0]
                assert!(
                    (direction.x).abs() < 1e-10,
                    "x should be ~0, got {}",
                    direction.x
                );
                assert!(
                    (direction.y - 1.0).abs() < 1e-10,
                    "y should be ~1, got {}",
                    direction.y
                );
                assert!(
                    (direction.z).abs() < 1e-10,
                    "z should be ~0, got {}",
                    direction.z
                );
            }
            other => panic!("Expected Arrow, got {other:?}"),
        }
    }

    #[test]
    fn world_frame_sensors_produce_unrotated_arrows() {
        let (model, mut data) = test_model_data(&[MjSensorType::FrameLinVel]);
        // reftype = None means world-frame — reading should pass through unrotated
        data.sensordata = nalgebra::DVector::from_vec(vec![1.0, 2.0, 3.0]);

        let value = data.sensor_data(&model, 0).to_vec();
        let gizmo = sensor_type_to_gizmo(&model, &data, 0, &value);

        match gizmo {
            SensorGizmo::Arrow { direction, .. } => {
                assert!((direction.x - 1.0).abs() < 1e-10);
                assert!((direction.y - 2.0).abs() < 1e-10);
                assert!((direction.z - 3.0).abs() < 1e-10);
            }
            other => panic!("Expected Arrow, got {other:?}"),
        }
    }

    #[test]
    fn model_with_multiple_sensors_populates_viz() {
        let types = [
            MjSensorType::Accelerometer,
            MjSensorType::Touch,
            MjSensorType::Rangefinder,
            MjSensorType::Force,
            MjSensorType::FrameLinVel,
            MjSensorType::JointPos,
        ];
        let (model, data) = test_model_data(&types);

        let mut entries = Vec::new();
        for sensor_id in 0..model.nsensor {
            let value = data.sensor_data(&model, sensor_id).to_vec();
            let gizmo = sensor_type_to_gizmo(&model, &data, sensor_id, &value);
            entries.push(SensorVizEntry {
                sensor_id,
                name: model.sensor_name[sensor_id].clone().unwrap_or_default(),
                sensor_type: model.sensor_type[sensor_id],
                gizmo,
                value,
            });
        }

        assert_eq!(entries.len(), 6);
        // Check gizmo types match expectations
        assert!(matches!(entries[0].gizmo, SensorGizmo::Frame { .. })); // Accelerometer
        assert!(matches!(entries[1].gizmo, SensorGizmo::Highlight { .. })); // Touch
        assert!(matches!(entries[2].gizmo, SensorGizmo::Ray { .. })); // Rangefinder
        assert!(matches!(entries[3].gizmo, SensorGizmo::Arrow { .. })); // Force
        assert!(matches!(entries[4].gizmo, SensorGizmo::Arrow { .. })); // FrameLinVel
        assert!(matches!(entries[5].gizmo, SensorGizmo::HudOnly)); // JointPos
    }

    #[test]
    fn relative_frame_sensor_rotated_by_ref() {
        // FrameLinVel with reftype = Body → reading in ref body's frame
        let (mut model, mut data) = test_model_data(&[MjSensorType::FrameLinVel]);
        model.sensor_reftype = vec![MjObjectType::Body];
        model.sensor_refid = vec![0];
        // 90° rotation about Z on the ref body
        let rot_z_90 = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        data.ximat = vec![rot_z_90];
        data.sensordata = nalgebra::DVector::from_vec(vec![1.0, 0.0, 0.0]);

        let value = data.sensor_data(&model, 0).to_vec();
        let gizmo = sensor_type_to_gizmo(&model, &data, 0, &value);

        match gizmo {
            SensorGizmo::Arrow { direction, .. } => {
                assert!((direction.x).abs() < 1e-10);
                assert!((direction.y - 1.0).abs() < 1e-10);
                assert!((direction.z).abs() < 1e-10);
            }
            other => panic!("Expected Arrow, got {other:?}"),
        }
    }
}
