//! Position-dependent sensor evaluation (`mj_sensor_pos`).
//!
//! Called after forward kinematics. Evaluates sensors that depend only on
//! position: joint position, frame pose, subtree COM, rangefinder, magnetometer,
//! actuator/tendon position.

use super::geom_distance::geom_distance;
use super::postprocess::{sensor_write, sensor_write3, sensor_write4, sensor_write6};
use crate::quat;
use crate::types::flags::disabled;
use crate::types::{
    DISABLE_SENSOR, Data, ENABLE_SLEEP, MjJointType, MjObjectType, MjSensorDataType, MjSensorType,
    Model, SensorStage,
};
use nalgebra::{Matrix3, Point3, UnitQuaternion, UnitVector3, Vector3};

/// Resolve position and rotation matrix for a reference object.
/// Mirrors MuJoCo's `get_xpos_xmat()` dispatch.
fn get_ref_pos_mat(
    model: &Model,
    data: &Data,
    reftype: MjObjectType,
    refid: usize,
) -> (Vector3<f64>, Matrix3<f64>) {
    match reftype {
        MjObjectType::Site if refid < model.nsite => (data.site_xpos[refid], data.site_xmat[refid]),
        MjObjectType::XBody if refid < model.nbody => (data.xpos[refid], data.xmat[refid]),
        MjObjectType::Body if refid < model.nbody => (data.xipos[refid], data.ximat[refid]),
        MjObjectType::Geom if refid < model.ngeom => (data.geom_xpos[refid], data.geom_xmat[refid]),
        _ => (Vector3::zeros(), Matrix3::identity()),
    }
}

/// An object's global orientation, as MuJoCo's `get_xquat`
/// (`engine_sensor.c:122-144`): its body's `xquat` times its local
/// quaternion, by `mju_mulQuat`.
fn object_xquat(model: &Model, data: &Data, objtype: MjObjectType, objid: usize) -> [f64; 4] {
    let q = |u: &UnitQuaternion<f64>| [u.w, u.i, u.j, u.k];
    match objtype {
        MjObjectType::XBody if objid < model.nbody => q(&data.xquat[objid]),
        MjObjectType::Body if objid < model.nbody => {
            quat::mul_quat(&q(&data.xquat[objid]), &q(&model.body_iquat[objid]))
        }
        MjObjectType::Geom if objid < model.ngeom => quat::mul_quat(
            &q(&data.xquat[model.geom_body[objid]]),
            &q(&model.geom_quat[objid]),
        ),
        MjObjectType::Site if objid < model.nsite => quat::mul_quat(
            &q(&data.xquat[model.site_body[objid]]),
            &q(&model.site_quat[objid]),
        ),
        _ => [1.0, 0.0, 0.0, 0.0],
    }
}

/// The geoms a geom-distance sensor's object names: the geom, or the body's
/// own geoms in index order (MuJoCo's `body_geomadr..+body_geomnum`,
/// `engine_sensor.c:676-692`), taken from `geom_body`:
/// `Model::add_ground_plane` may number the world's geoms apart and widen the
/// world's range over another body's.
fn sensor_geoms(model: &Model, objtype: MjObjectType, id: usize) -> Vec<usize> {
    if objtype == MjObjectType::Body {
        (0..model.ngeom)
            .filter(|&g| model.geom_body[g] == id)
            .collect()
    } else {
        vec![id]
    }
}

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
pub fn mj_sensor_pos(model: &Model, data: &mut Data) {
    // S4.10: Early return — sensordata is NOT zeroed (intentional MuJoCo match).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }

    // MuJoCo skips a sleeping sensor while some body sleeps (`engine_sensor.c:1467-1475`).
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.nbody_awake < model.nbody;

    for sensor_id in 0..model.nsensor {
        // Skip non-position sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Position {
            continue;
        }

        // §16.5d: a sleeping sensor keeps its value
        if sleep_filter && crate::island::sensor_asleep(model, data, sensor_id) {
            continue;
        }

        // MuJoCo `compute_or_read_sensor` (engine_sensor.c:1346-1388): a
        // delayed sensor, or an interval sensor between ticks, reads its
        // buffer instead.
        if crate::history::sensor_reads_history(model, data, sensor_id) {
            crate::history::read_sensor(model, data, sensor_id);
            continue;
        }
        compute_pos_sensor(model, data, sensor_id);
    }

    // §66: Plugin sensor dispatch at position stage
    super::compute_plugin_sensors(model, data, crate::plugin::PluginStage::Pos);
}

/// Compute pos-stage sensor `sensor_id` into `sensordata`, without its
/// cutoff. MuJoCo `mj_computeSensorPos`.
// The per-sensor-type dispatch (joint/site/body/camera) as one function, so the branching reads end-to-end.
#[allow(clippy::too_many_lines)]
#[allow(clippy::unreachable)] // position-stage sensor dispatch; velocity/acceleration sensors handled in separate stages
pub fn compute_pos_sensor(model: &Model, data: &mut Data, sensor_id: usize) {
    let adr = model.sensor_adr[sensor_id];
    let objid = model.sensor_objid[sensor_id];

    match model.sensor_type[sensor_id] {
        MjSensorType::JointPos if objid < model.njnt => {
            // Scalar joint position (hinge/slide only).
            // Ball joints use BallQuat, free joints use FramePos + FrameQuat.
            let qpos_adr = model.jnt_qpos_adr[objid];
            match model.jnt_type[objid] {
                MjJointType::Hinge | MjJointType::Slide => {
                    sensor_write(&mut data.sensordata, adr, 0, data.qpos[qpos_adr]);
                }
                _ => {} // Ball/Free not supported by JointPos; use BallQuat/FramePos
            }
        }

        MjSensorType::BallQuat
            if objid < model.njnt && model.jnt_type[objid] == MjJointType::Ball =>
        {
            // Ball joint quaternion [w, x, y, z], normalized as MuJoCo's
            // (`mju_normalize4`, `engine_sensor.c:572-575`).
            let q = quat::qpos_quat(&data.qpos, model.jnt_qpos_adr[objid]);
            sensor_write4(&mut data.sensordata, adr, q[0], q[1], q[2], q[3]);
        }

        MjSensorType::FramePos => {
            // Position of site/body/geom
            let pos = match model.sensor_objtype[sensor_id] {
                MjObjectType::Site if objid < model.nsite => data.site_xpos[objid],
                MjObjectType::XBody if objid < model.nbody => data.xpos[objid],
                MjObjectType::Body if objid < model.nbody => data.xipos[objid],
                MjObjectType::Geom if objid < model.ngeom => data.geom_xpos[objid],
                _ => Vector3::zeros(),
            };
            // Reference-frame transform: p_relative = R_ref^T * (p_obj - p_ref)
            if model.sensor_reftype[sensor_id] == MjObjectType::None {
                sensor_write3(&mut data.sensordata, adr, &pos);
            } else {
                let (ref_pos, ref_mat) = get_ref_pos_mat(
                    model,
                    data,
                    model.sensor_reftype[sensor_id],
                    model.sensor_refid[sensor_id],
                );
                let relative = ref_mat.transpose() * (pos - ref_pos);
                sensor_write3(&mut data.sensordata, adr, &relative);
            }
        }

        MjSensorType::FrameQuat => {
            // MuJoCo `mjSENS_FRAMEQUAT` (`engine_sensor.c:632-651`): the
            // object's quaternion, or the reference frame's conjugate times it.
            let obj = object_xquat(model, data, model.sensor_objtype[sensor_id], objid);
            let quat = if model.sensor_reftype[sensor_id] == MjObjectType::None {
                obj
            } else {
                let refquat = object_xquat(
                    model,
                    data,
                    model.sensor_reftype[sensor_id],
                    model.sensor_refid[sensor_id],
                );
                quat::mul_quat(&quat::neg_quat(&refquat), &obj)
            };
            sensor_write4(
                &mut data.sensordata,
                adr,
                quat[0],
                quat[1],
                quat[2],
                quat[3],
            );
        }

        MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
            // Frame axis
            let mat = match model.sensor_objtype[sensor_id] {
                MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                MjObjectType::XBody if objid < model.nbody => data.xmat[objid],
                MjObjectType::Body if objid < model.nbody => data.ximat[objid],
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
            // Reference-frame transform: axis_in_ref = R_ref^T * axis_world
            if model.sensor_reftype[sensor_id] == MjObjectType::None {
                sensor_write3(&mut data.sensordata, adr, &col);
            } else {
                let (_ref_pos, ref_mat) = get_ref_pos_mat(
                    model,
                    data,
                    model.sensor_reftype[sensor_id],
                    model.sensor_refid[sensor_id],
                );
                let relative = ref_mat.transpose() * col;
                sensor_write3(&mut data.sensordata, adr, &relative);
            }
        }

        MjSensorType::SubtreeCom if objid < model.nbody => {
            // Read from persistent subtree_com field (computed in position stage)
            sensor_write3(&mut data.sensordata, adr, &data.subtree_com[objid]);
        }

        MjSensorType::Rangefinder => {
            // Rangefinder: ray-cast along site's positive Z axis to find
            // distance to nearest geom surface. Skips geoms on the
            // sensor's parent body to avoid self-intersection.
            if model.sensor_objtype[sensor_id] == MjObjectType::Site && objid < model.nsite {
                let ray_origin = Point3::from(data.site_xpos[objid]);
                // MuJoCo convention: rangefinder shoots along +Z of site frame
                let site_z = Vector3::new(
                    data.site_xmat[objid][(0, 2)],
                    data.site_xmat[objid][(1, 2)],
                    data.site_xmat[objid][(2, 2)],
                );

                let ray_norm = site_z.norm();
                if ray_norm < 1e-10 {
                    sensor_write(&mut data.sensordata, adr, 0, -1.0);
                } else {
                    let ray_direction = UnitVector3::new_normalize(site_z);
                    let max_range = if model.sensor_cutoff[sensor_id] > 0.0 {
                        model.sensor_cutoff[sensor_id]
                    } else {
                        100.0
                    };
                    let parent_body = model.site_body[objid];

                    let closest_dist = crate::raycast::raycast_scene(
                        model,
                        data,
                        ray_origin,
                        ray_direction,
                        max_range,
                        Some(parent_body),
                        None,
                    )
                    .map_or(-1.0, |r| r.hit.distance);

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

        MjSensorType::ActuatorPos if objid < model.nu => {
            // The transmission's length, which the position stage set before
            // the sensors (MuJoCo `engine_sensor.c:568-570`).
            sensor_write(&mut data.sensordata, adr, 0, data.actuator_length[objid]);
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
        MjSensorType::User if model.sensor_datatype[sensor_id] == MjSensorDataType::Position => {
            if let Some(ref cb) = model.cb_sensor {
                (cb.0)(model, data, sensor_id, SensorStage::Pos);
            }
        }

        // Clock: reads simulation time. MuJoCo: mjSENS_CLOCK in mj_computeSensorPos.
        MjSensorType::Clock => {
            let adr = model.sensor_adr[sensor_id];
            sensor_write(&mut data.sensordata, adr, 0, data.time);
        }

        // Geom distance sensors: shared dual-object + all-pairs minimum distance.
        // MuJoCo: mj_computeSensorPos case mjSENS_GEOMDIST/GEOMNORMAL/GEOMFROMTO.
        MjSensorType::GeomDist | MjSensorType::GeomNormal | MjSensorType::GeomFromTo => {
            let adr = model.sensor_adr[sensor_id];
            let objtype = model.sensor_objtype[sensor_id];
            let objid = model.sensor_objid[sensor_id];
            let reftype = model.sensor_reftype[sensor_id];
            let refid = model.sensor_refid[sensor_id];
            let cutoff = model.sensor_cutoff[sensor_id];
            let sensor_type = model.sensor_type[sensor_id];

            let geoms1 = sensor_geoms(model, objtype, objid);
            let geoms2 = sensor_geoms(model, reftype, refid);

            // All-pairs minimum distance
            let mut dist = cutoff;
            let mut fromto = [0.0f64; 6];
            for &g1 in &geoms1 {
                for &g2 in &geoms2 {
                    let (dist_new, fromto_new) = geom_distance(model, data, g1, g2, cutoff);
                    if dist_new < dist {
                        dist = dist_new;
                        fromto = fromto_new;
                    }
                }
            }

            // Per-type output
            match sensor_type {
                MjSensorType::GeomDist => {
                    sensor_write(&mut data.sensordata, adr, 0, dist);
                }
                MjSensorType::GeomNormal => {
                    let normal = Vector3::new(
                        fromto[3] - fromto[0],
                        fromto[4] - fromto[1],
                        fromto[5] - fromto[2],
                    );
                    // Zero-vector guard: only normalize if non-zero
                    let output = if normal.norm_squared() > 0.0 {
                        normal.normalize()
                    } else {
                        normal // stays [0, 0, 0]
                    };
                    sensor_write3(&mut data.sensordata, adr, &output);
                }
                MjSensorType::GeomFromTo => {
                    sensor_write6(&mut data.sensordata, adr, &fromto);
                }
                // Outer dispatch filters to position-sensor variants; other types never reach here.
                _ => unreachable!(),
            }
        }

        // Skip velocity/acceleration-dependent sensors
        _ => {}
    }
}
