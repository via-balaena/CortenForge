//! Position-dependent sensor evaluation (`mj_sensor_pos`).
//!
//! Called after forward kinematics. Evaluates sensors that depend only on
//! position: joint position, frame pose, subtree COM, rangefinder, magnetometer,
//! actuator/tendon position.

use crate::collision::narrow::geom_to_collision_shape;
use crate::collision_shape::CollisionShape;
use crate::raycast::raycast_shape;
use crate::types::flags::disabled;
use crate::types::{
    ActuatorTransmission, DISABLE_SENSOR, Data, ENABLE_SLEEP, GeomType, MjJointType, MjObjectType,
    MjSensorDataType, MjSensorType, Model, SensorStage, SleepState,
};
use nalgebra::{Matrix3, Point3, UnitQuaternion, UnitVector3, Vector3};
use sim_types::Pose;
use std::sync::Arc;

use super::postprocess::{sensor_write, sensor_write3, sensor_write4};
use super::sensor_body_id;

/// Compute position-dependent sensor values.
///
/// This is called after `mj_fwd_position` and computes sensors that depend only
/// on position (not velocity or acceleration):
/// - `JointPos`: joint position
/// - `FramePos`: site/body position
/// - `FrameQuat`: site/body orientation
/// - FrameXAxis/YAxis/ZAxis: frame axes
/// - `TendonPos`: tendon length
/// - `ActuatorPos`: actuator length
/// - Rangefinder: distance measurement
/// - Touch: contact detection
#[allow(clippy::too_many_lines)]
pub fn mj_sensor_pos(model: &Model, data: &mut Data) {
    // S4.10: Early return — sensordata is NOT zeroed (intentional MuJoCo match).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-position sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Position {
            continue;
        }

        // §16.5d: Skip sensors on sleeping bodies — values frozen at sleep time
        if sleep_enabled {
            if let Some(body_id) = sensor_body_id(model, sensor_id) {
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::JointPos => {
                // Scalar joint position (hinge/slide only).
                // Ball joints use BallQuat, free joints use FramePos + FrameQuat.
                if objid < model.njnt {
                    let qpos_adr = model.jnt_qpos_adr[objid];
                    match model.jnt_type[objid] {
                        MjJointType::Hinge | MjJointType::Slide => {
                            sensor_write(&mut data.sensordata, adr, 0, data.qpos[qpos_adr]);
                        }
                        _ => {} // Ball/Free not supported by JointPos; use BallQuat/FramePos
                    }
                }
            }

            MjSensorType::BallQuat => {
                // Ball joint quaternion [w, x, y, z]
                if objid < model.njnt && model.jnt_type[objid] == MjJointType::Ball {
                    let qpos_adr = model.jnt_qpos_adr[objid];
                    // Read quaternion from qpos and normalize in locals
                    // (MuJoCo does mju_normalize4). Use 1e-10 threshold and
                    // identity reset to match our normalize_quaternion() convention.
                    let (w, x, y, z) = (
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    );
                    let norm = (w * w + x * x + y * y + z * z).sqrt();
                    if norm > 1e-10 {
                        sensor_write4(
                            &mut data.sensordata,
                            adr,
                            w / norm,
                            x / norm,
                            y / norm,
                            z / norm,
                        );
                    } else {
                        // Degenerate — reset to identity [w=1, x=0, y=0, z=0]
                        sensor_write4(&mut data.sensordata, adr, 1.0, 0.0, 0.0, 0.0);
                    }
                }
            }

            MjSensorType::FramePos => {
                // Position of site/body in world frame
                let pos = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xpos[objid],
                    MjObjectType::Body if objid < model.nbody => data.xpos[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xpos[objid],
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &pos);
            }

            MjSensorType::FrameQuat => {
                // Orientation of site/body as quaternion [w, x, y, z]
                let quat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        // Compute site quaternion from rotation matrix
                        let mat = data.site_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => data.xquat[objid],
                    MjObjectType::Geom if objid < model.ngeom => {
                        let mat = data.geom_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    _ => UnitQuaternion::identity(),
                };
                sensor_write4(&mut data.sensordata, adr, quat.w, quat.i, quat.j, quat.k);
            }

            MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
                // Frame axis in world coordinates
                let mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xmat[objid],
                    _ => Matrix3::identity(),
                };
                // These are the only types that can reach here due to the outer match
                #[allow(clippy::match_same_arms)]
                let col_idx = match model.sensor_type[sensor_id] {
                    MjSensorType::FrameXAxis => 0,
                    MjSensorType::FrameYAxis => 1,
                    MjSensorType::FrameZAxis => 2,
                    _ => 0, // Unreachable but needed for exhaustiveness
                };
                let col = Vector3::new(mat[(0, col_idx)], mat[(1, col_idx)], mat[(2, col_idx)]);
                sensor_write3(&mut data.sensordata, adr, &col);
            }

            MjSensorType::SubtreeCom => {
                // Read from persistent subtree_com field (computed in position stage)
                if objid < model.nbody {
                    sensor_write3(&mut data.sensordata, adr, &data.subtree_com[objid]);
                }
            }

            MjSensorType::Rangefinder => {
                // Rangefinder: ray-cast along site's positive Z axis to find
                // distance to nearest geom surface. Skips the geom attached to
                // the sensor's parent body to avoid self-intersection.
                if model.sensor_objtype[sensor_id] == MjObjectType::Site && objid < model.nsite {
                    let ray_origin = Point3::from(data.site_xpos[objid]);
                    // MuJoCo convention: rangefinder shoots along +Z of site frame
                    let site_z = Vector3::new(
                        data.site_xmat[objid][(0, 2)],
                        data.site_xmat[objid][(1, 2)],
                        data.site_xmat[objid][(2, 2)],
                    );
                    let ray_dir = site_z;

                    // Normalize ray direction (should already be unit, but be safe)
                    let ray_norm = ray_dir.norm();
                    if ray_norm < 1e-10 {
                        sensor_write(&mut data.sensordata, adr, 0, -1.0); // Invalid direction
                    } else {
                        let ray_direction = UnitVector3::new_normalize(ray_dir);

                        // Determine max range: use cutoff if positive, otherwise large default
                        let max_range = if model.sensor_cutoff[sensor_id] > 0.0 {
                            model.sensor_cutoff[sensor_id]
                        } else {
                            100.0 // Default max range
                        };

                        // Exclude geoms belonging to the sensor's parent body
                        let parent_body = model.site_body[objid];

                        let mut closest_dist = -1.0_f64; // -1 means no hit (MuJoCo convention)

                        for geom_id in 0..model.ngeom {
                            // Skip geoms on the sensor's parent body (self-intersection)
                            if model.geom_body[geom_id] == parent_body {
                                continue;
                            }

                            // Build shape pose from geom world transform
                            let geom_pos = data.geom_xpos[geom_id];
                            let geom_mat = data.geom_xmat[geom_id];
                            let geom_quat = UnitQuaternion::from_rotation_matrix(
                                &nalgebra::Rotation3::from_matrix_unchecked(geom_mat),
                            );
                            let shape_pose =
                                Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

                            // Convert geom to collision shape for ray testing
                            if let Some(shape) = geom_to_collision_shape(
                                model.geom_type[geom_id],
                                model.geom_size[geom_id],
                            ) {
                                if let Some(hit) = raycast_shape(
                                    &shape,
                                    &shape_pose,
                                    ray_origin,
                                    ray_direction,
                                    max_range,
                                ) {
                                    if closest_dist < 0.0 || hit.distance < closest_dist {
                                        closest_dist = hit.distance;
                                    }
                                }
                            }

                            // Also handle mesh geoms
                            if model.geom_type[geom_id] == GeomType::Mesh {
                                if let Some(mesh_id) = model.geom_mesh[geom_id] {
                                    let mesh_shape = CollisionShape::TriangleMesh {
                                        data: Arc::clone(&model.mesh_data[mesh_id]),
                                    };
                                    let shape_pose = Pose::from_position_rotation(
                                        Point3::from(geom_pos),
                                        geom_quat,
                                    );
                                    if let Some(hit) = raycast_shape(
                                        &mesh_shape,
                                        &shape_pose,
                                        ray_origin,
                                        ray_direction,
                                        max_range,
                                    ) {
                                        if closest_dist < 0.0 || hit.distance < closest_dist {
                                            closest_dist = hit.distance;
                                        }
                                    }
                                }
                            }
                        }

                        sensor_write(&mut data.sensordata, adr, 0, closest_dist);
                    }
                } else {
                    sensor_write(&mut data.sensordata, adr, 0, -1.0);
                }
            }

            MjSensorType::Magnetometer => {
                // Magnetometer: measures the global magnetic field in the sensor's
                // local frame. Only depends on site_xmat (available after FK).
                //
                // B_sensor = R_site^T * B_world
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let b_sensor = site_mat.transpose() * model.magnetic;
                sensor_write3(&mut data.sensordata, adr, &b_sensor);
            }

            MjSensorType::ActuatorPos => {
                // Actuator position: transmission length = gear * joint_position.
                // For joint-type transmissions, this is gear[0] * qpos[qpos_adr].
                if objid < model.nu {
                    match model.actuator_trntype[objid] {
                        ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
                            let jnt_id = model.actuator_trnid[objid][0];
                            if jnt_id < model.njnt {
                                let qpos_adr = model.jnt_qpos_adr[jnt_id];
                                let gear = model.actuator_gear[objid][0];
                                sensor_write(
                                    &mut data.sensordata,
                                    adr,
                                    0,
                                    gear * data.qpos[qpos_adr],
                                );
                            }
                        }
                        ActuatorTransmission::Tendon => {
                            let tendon_id = model.actuator_trnid[objid][0];
                            let value = if tendon_id < model.ntendon {
                                data.ten_length[tendon_id] * model.actuator_gear[objid][0]
                            } else {
                                0.0
                            };
                            sensor_write(&mut data.sensordata, adr, 0, value);
                        }
                        ActuatorTransmission::Site
                        | ActuatorTransmission::Body
                        | ActuatorTransmission::SliderCrank => {
                            // Length set by transmission function (runs before this).
                            sensor_write(&mut data.sensordata, adr, 0, data.actuator_length[objid]);
                        }
                    }
                }
            }

            MjSensorType::TendonPos => {
                let tendon_id = model.sensor_objid[sensor_id];
                let value = if tendon_id < model.ntendon {
                    data.ten_length[tendon_id]
                } else {
                    0.0
                };
                sensor_write(&mut data.sensordata, adr, 0, value);
            }

            // DT-79: User-defined sensors at position stage
            MjSensorType::User => {
                if model.sensor_datatype[sensor_id] == MjSensorDataType::Position {
                    if let Some(ref cb) = model.cb_sensor {
                        (cb.0)(model, data, sensor_id, SensorStage::Pos);
                    }
                }
            }

            // Skip velocity/acceleration-dependent sensors
            _ => {}
        }
    }
}
