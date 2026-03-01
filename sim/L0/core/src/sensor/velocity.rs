//! Velocity-dependent sensor evaluation (`mj_sensor_vel`).
//!
//! Called after velocity kinematics. Evaluates sensors that depend on velocity:
//! joint velocity, gyro, velocimeter, frame linear/angular velocity,
//! subtree linear velocity, subtree angular momentum, actuator/tendon velocity.

use crate::types::flags::disabled;
use crate::types::{
    DISABLE_SENSOR, Data, ENABLE_SLEEP, MjJointType, MjObjectType, MjSensorDataType, MjSensorType,
    Model, SensorStage, SleepState,
};
use nalgebra::{Matrix3, Vector3};

use super::postprocess::{sensor_write, sensor_write3};
use super::sensor_body_id;
use crate::dynamics::object_velocity;
use crate::forward::mj_subtree_vel;

/// Compute velocity-dependent sensor values.
///
/// This is called after `mj_fwd_velocity` and computes sensors that depend on
/// velocity (but not acceleration):
/// - `JointVel`: joint velocity
/// - Gyro: angular velocity
/// - Velocimeter: linear velocity
/// - `FrameLinVel`: site/body linear velocity
/// - `FrameAngVel`: site/body angular velocity
/// - `TendonVel`: tendon velocity
/// - `ActuatorVel`: actuator velocity
#[allow(clippy::too_many_lines)]
pub fn mj_sensor_vel(model: &Model, data: &mut Data) {
    // S4.10: Early return — sensordata is NOT zeroed (intentional MuJoCo match).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-velocity sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Velocity {
            continue;
        }

        // §16.5d: Skip sensors on sleeping bodies
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
            MjSensorType::JointVel => {
                // Scalar joint velocity (hinge/slide only).
                // Ball joints use BallAngVel, free joints use FrameLinVel + FrameAngVel.
                if objid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[objid];
                    match model.jnt_type[objid] {
                        MjJointType::Hinge | MjJointType::Slide => {
                            sensor_write(&mut data.sensordata, adr, 0, data.qvel[dof_adr]);
                        }
                        _ => {} // Ball/Free not supported by JointVel; use BallAngVel/FrameLinVel
                    }
                }
            }

            MjSensorType::BallAngVel => {
                // Ball joint angular velocity [wx, wy, wz] in local (child body) frame
                if objid < model.njnt && model.jnt_type[objid] == MjJointType::Ball {
                    let dof_adr = model.jnt_dof_adr[objid];
                    let omega = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    sensor_write3(&mut data.sensordata, adr, &omega);
                }
            }

            MjSensorType::Gyro => {
                // Angular velocity in sensor (site) frame
                let (omega_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        let omega = Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        );
                        (omega, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let omega = Vector3::new(
                            data.cvel[objid][0],
                            data.cvel[objid][1],
                            data.cvel[objid][2],
                        );
                        (omega, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Transform to sensor frame
                let omega_sensor = site_mat.transpose() * omega_world;
                sensor_write3(&mut data.sensordata, adr, &omega_sensor);
            }

            MjSensorType::Velocimeter => {
                let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => (
                        model.site_body[objid],
                        data.site_xpos[objid],
                        data.site_xmat[objid],
                    ),
                    MjObjectType::XBody if objid < model.nbody => {
                        (objid, data.xpos[objid], data.xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        (objid, data.xipos[objid], data.ximat[objid])
                    }
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };
                let (_omega, v) = object_velocity(data, body_id, &site_pos, Some(&site_mat));
                sensor_write3(&mut data.sensordata, adr, &v);
            }

            MjSensorType::FrameLinVel => {
                let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        (model.site_body[objid], data.site_xpos[objid])
                    }
                    MjObjectType::XBody if objid < model.nbody => (objid, data.xpos[objid]),
                    MjObjectType::Body if objid < model.nbody => (objid, data.xipos[objid]),
                    MjObjectType::Geom if objid < model.ngeom => {
                        (model.geom_body[objid], data.geom_xpos[objid])
                    }
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };

                if model.sensor_reftype[sensor_id] == MjObjectType::None {
                    let (_omega, v) = object_velocity(data, body_id, &obj_pos, None);
                    sensor_write3(&mut data.sensordata, adr, &v);
                } else {
                    let reftype = model.sensor_reftype[sensor_id];
                    let refid = model.sensor_refid[sensor_id];

                    // Reference object body_id and position
                    let (ref_body_id, ref_pos) = match reftype {
                        MjObjectType::Site if refid < model.nsite => {
                            (model.site_body[refid], data.site_xpos[refid])
                        }
                        MjObjectType::XBody if refid < model.nbody => (refid, data.xpos[refid]),
                        MjObjectType::Body if refid < model.nbody => (refid, data.xipos[refid]),
                        MjObjectType::Geom if refid < model.ngeom => {
                            (model.geom_body[refid], data.geom_xpos[refid])
                        }
                        _ => {
                            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                            continue;
                        }
                    };
                    let ref_mat = match reftype {
                        MjObjectType::Site if refid < model.nsite => data.site_xmat[refid],
                        MjObjectType::XBody if refid < model.nbody => data.xmat[refid],
                        MjObjectType::Body if refid < model.nbody => data.ximat[refid],
                        MjObjectType::Geom if refid < model.ngeom => data.geom_xmat[refid],
                        _ => Matrix3::identity(),
                    };

                    // Object velocity in world frame (flg_local = 0 → local_rot = None)
                    let (_w_obj, v_obj) = object_velocity(data, body_id, &obj_pos, None);
                    // Reference velocity in world frame
                    let (w_ref, v_ref) = object_velocity(data, ref_body_id, &ref_pos, None);

                    // Coriolis correction: v_relative = v_obj - v_ref - w_ref × (p_obj - p_ref)
                    let dif = obj_pos - ref_pos;
                    let coriolis = w_ref.cross(&dif);
                    let v_relative = ref_mat.transpose() * (v_obj - v_ref - coriolis);
                    sensor_write3(&mut data.sensordata, adr, &v_relative);
                }
            }

            MjSensorType::FrameAngVel => {
                // Angular velocity (reference-point-independent for rigid bodies)
                let w_obj = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        )
                    }
                    MjObjectType::XBody | MjObjectType::Body if objid < model.nbody => {
                        Vector3::new(
                            data.cvel[objid][0],
                            data.cvel[objid][1],
                            data.cvel[objid][2],
                        )
                    }
                    MjObjectType::Geom if objid < model.ngeom => {
                        let body_id = model.geom_body[objid];
                        Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        )
                    }
                    _ => Vector3::zeros(),
                };

                if model.sensor_reftype[sensor_id] == MjObjectType::None {
                    sensor_write3(&mut data.sensordata, adr, &w_obj);
                } else {
                    let reftype = model.sensor_reftype[sensor_id];
                    let refid = model.sensor_refid[sensor_id];

                    // Reference angular velocity in world frame
                    let ref_body_id = match reftype {
                        MjObjectType::Site if refid < model.nsite => model.site_body[refid],
                        MjObjectType::XBody | MjObjectType::Body if refid < model.nbody => refid,
                        MjObjectType::Geom if refid < model.ngeom => model.geom_body[refid],
                        _ => {
                            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                            continue;
                        }
                    };
                    let w_ref = Vector3::new(
                        data.cvel[ref_body_id][0],
                        data.cvel[ref_body_id][1],
                        data.cvel[ref_body_id][2],
                    );
                    let ref_mat = match reftype {
                        MjObjectType::Site if refid < model.nsite => data.site_xmat[refid],
                        MjObjectType::XBody if refid < model.nbody => data.xmat[refid],
                        MjObjectType::Body if refid < model.nbody => data.ximat[refid],
                        MjObjectType::Geom if refid < model.ngeom => data.geom_xmat[refid],
                        _ => Matrix3::identity(),
                    };

                    // w_relative = R_ref^T * (w_obj - w_ref), NO Coriolis for angular velocity
                    let w_relative = ref_mat.transpose() * (w_obj - w_ref);
                    sensor_write3(&mut data.sensordata, adr, &w_relative);
                }
            }

            MjSensorType::SubtreeLinVel => {
                // Subtree linear velocity — read from persistent field.
                if !data.flg_subtreevel {
                    mj_subtree_vel(model, data);
                }
                if objid < model.nbody {
                    sensor_write3(&mut data.sensordata, adr, &data.subtree_linvel[objid]);
                }
            }

            MjSensorType::SubtreeAngMom => {
                // Subtree angular momentum about subtree COM — read from persistent field.
                if !data.flg_subtreevel {
                    mj_subtree_vel(model, data);
                }
                if objid < model.nbody {
                    sensor_write3(&mut data.sensordata, adr, &data.subtree_angmom[objid]);
                }
            }

            MjSensorType::ActuatorVel => {
                // Read from pre-computed actuator_velocity (populated by mj_actuator_length,
                // which runs before mj_sensor_vel in forward()).
                let act_id = model.sensor_objid[sensor_id];
                if act_id < model.nu {
                    sensor_write(&mut data.sensordata, adr, 0, data.actuator_velocity[act_id]);
                }
            }

            MjSensorType::TendonVel => {
                let tendon_id = model.sensor_objid[sensor_id];
                let value = if tendon_id < model.ntendon {
                    data.ten_velocity[tendon_id]
                } else {
                    0.0
                };
                sensor_write(&mut data.sensordata, adr, 0, value);
            }

            // DT-79: User-defined sensors at velocity stage.
            // Conservative: trigger mj_subtree_vel for opaque user callbacks.
            MjSensorType::User => {
                if model.sensor_datatype[sensor_id] == MjSensorDataType::Velocity {
                    if !data.flg_subtreevel {
                        mj_subtree_vel(model, data);
                    }
                    if let Some(ref cb) = model.cb_sensor {
                        (cb.0)(model, data, sensor_id, SensorStage::Vel);
                    }
                }
            }

            // Skip position/acceleration-dependent sensors
            _ => {}
        }
    }
}
