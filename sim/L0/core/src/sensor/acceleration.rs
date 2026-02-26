//! Acceleration-dependent sensor evaluation (`mj_sensor_acc`).
//!
//! Called after constraint solve. Evaluates sensors that depend on acceleration:
//! accelerometer, force/torque, touch, frame linear/angular acceleration,
//! actuator force, joint/tendon limit force.
//!
//! Accelerometer, FrameLinAcc, and FrameAngAcc read from `cacc` (populated by
//! `mj_body_accumulators`). Force and Torque read from `cfrc_int`. This matches
//! MuJoCo's `mj_sensorAcc` → `mj_objectAcceleration` / `mju_transformSpatial`.

use crate::types::flags::disabled;
use crate::types::{
    ConstraintType, DISABLE_SENSOR, Data, ENABLE_SLEEP, MjObjectType, MjSensorDataType,
    MjSensorType, Model, SensorStage, SleepState,
};
use nalgebra::Vector3;

use crate::dynamics::{object_acceleration, object_force};
use crate::forward::mj_body_accumulators;

use super::postprocess::{sensor_write, sensor_write3};
use super::sensor_body_id;

/// Compute acceleration-dependent sensor values.
///
/// This is called after `mj_fwd_acceleration` and computes sensors that depend
/// on acceleration:
/// - Accelerometer: proper acceleration in sensor frame (reads `cacc`)
/// - Force: constraint force at site (reads `cfrc_int`)
/// - Torque: constraint torque at site (reads `cfrc_int`)
/// - `FrameLinAcc`: linear acceleration in world frame (reads `cacc`)
/// - `FrameAngAcc`: angular acceleration in world frame (reads `cacc`)
/// - `ActuatorFrc`: actuator force
pub fn mj_sensor_acc(model: &Model, data: &mut Data) {
    // S4.10: Early return — sensordata is NOT zeroed (intentional MuJoCo match).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-acceleration sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Acceleration {
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

        // Lazy gate: trigger mj_body_accumulators on demand for sensor types
        // that read cacc/cfrc_int/cfrc_ext. Touch, ActuatorFrc, and limit
        // sensors read efc_force directly and do NOT need body accumulators.
        if !data.flg_rnepost {
            match model.sensor_type[sensor_id] {
                MjSensorType::Accelerometer
                | MjSensorType::Force
                | MjSensorType::Torque
                | MjSensorType::FrameLinAcc
                | MjSensorType::FrameAngAcc => {
                    mj_body_accumulators(model, data);
                }
                MjSensorType::User
                    if model.sensor_datatype[sensor_id] == MjSensorDataType::Acceleration =>
                {
                    mj_body_accumulators(model, data);
                }
                _ => {}
            }
        }

        match model.sensor_type[sensor_id] {
            MjSensorType::Accelerometer => {
                let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => (
                        model.site_body[objid],
                        data.site_xpos[objid],
                        data.site_xmat[objid],
                    ),
                    MjObjectType::Body if objid < model.nbody => {
                        (objid, data.xpos[objid], data.xmat[objid])
                    }
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };
                let (_alpha, a_lin) =
                    object_acceleration(data, body_id, &site_pos, Some(&site_mat));
                sensor_write3(&mut data.sensordata, adr, &a_lin);
            }

            MjSensorType::Force => {
                let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => (
                        model.site_body[objid],
                        data.site_xpos[objid],
                        data.site_xmat[objid],
                    ),
                    MjObjectType::Body if objid < model.nbody => {
                        (objid, data.xpos[objid], data.xmat[objid])
                    }
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };
                // Force is translation-invariant — only torque shifts. The torque
                // component is computed but discarded (minor wasted work, acceptable
                // for API consistency with Torque arm which reads the same wrench).
                let (_torque, force) = object_force(data, body_id, &site_pos, Some(&site_mat));
                sensor_write3(&mut data.sensordata, adr, &force);
            }

            MjSensorType::Torque => {
                let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => (
                        model.site_body[objid],
                        data.site_xpos[objid],
                        data.site_xmat[objid],
                    ),
                    MjObjectType::Body if objid < model.nbody => {
                        (objid, data.xpos[objid], data.xmat[objid])
                    }
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };
                let (torque, _force) = object_force(data, body_id, &site_pos, Some(&site_mat));
                sensor_write3(&mut data.sensordata, adr, &torque);
            }

            MjSensorType::Touch => {
                // Touch sensor: sum of normal contact forces on the attached geom.
                //
                // Scans efc_force directly for contact constraint rows involving
                // the sensor's geom. The first row of each contact group holds the
                // normal force (always >= 0 after projection).
                let mut total_force = 0.0;
                let nefc = data.efc_type.len();
                let mut ei = 0;
                while ei < nefc {
                    let dim = data.efc_dim[ei];
                    if matches!(
                        data.efc_type[ei],
                        ConstraintType::ContactElliptic
                            | ConstraintType::ContactFrictionless
                            | ConstraintType::ContactPyramidal
                    ) {
                        let ci = data.efc_id[ei];
                        if ci < data.contacts.len() {
                            let c = &data.contacts[ci];
                            if c.geom1 == objid || c.geom2 == objid {
                                if data.efc_type[ei] == ConstraintType::ContactPyramidal {
                                    // §32: Normal force = sum of ALL facet forces
                                    for k in 0..dim {
                                        total_force += data.efc_force[ei + k];
                                    }
                                } else {
                                    // Elliptic/frictionless: first row is normal force
                                    total_force += data.efc_force[ei];
                                }
                            }
                        }
                        ei += dim;
                    } else {
                        ei += 1;
                    }
                }
                sensor_write(&mut data.sensordata, adr, 0, total_force);
            }

            MjSensorType::ActuatorFrc => {
                // Scalar actuator force (transmission-independent).
                // Matches MuJoCo: actuatorfrc sensor = actuator_force[objid].
                if objid < model.nu {
                    sensor_write(&mut data.sensordata, adr, 0, data.actuator_force[objid]);
                }
            }

            MjSensorType::JointLimitFrc => {
                // Joint limit force: read cached constraint force magnitude.
                // objid is the joint index (resolved by model builder).
                if objid < data.jnt_limit_frc.len() {
                    sensor_write(&mut data.sensordata, adr, 0, data.jnt_limit_frc[objid]);
                }
            }

            MjSensorType::TendonLimitFrc => {
                // Tendon limit force: read cached constraint force magnitude.
                // objid is the tendon index (resolved by model builder).
                if objid < data.ten_limit_frc.len() {
                    sensor_write(&mut data.sensordata, adr, 0, data.ten_limit_frc[objid]);
                }
            }

            MjSensorType::FrameLinAcc => {
                let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        (model.site_body[objid], data.site_xpos[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };
                let (_alpha, a_lin) = object_acceleration(data, body_id, &obj_pos, None);
                sensor_write3(&mut data.sensordata, adr, &a_lin);
            }

            MjSensorType::FrameAngAcc => {
                // Angular acceleration in world frame. Reads the angular
                // component of cacc. Angular acceleration is reference-point-
                // independent for a rigid body — no Coriolis correction needed.
                let body_id = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => model.site_body[objid],
                    MjObjectType::Body if objid < model.nbody => objid,
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };

                let cacc = data.cacc[body_id];
                let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
                sensor_write3(&mut data.sensordata, adr, &alpha);
            }

            // DT-79: User-defined sensors at acceleration stage
            MjSensorType::User => {
                if model.sensor_datatype[sensor_id] == MjSensorDataType::Acceleration {
                    if let Some(ref cb) = model.cb_sensor {
                        (cb.0)(model, data, sensor_id, SensorStage::Acc);
                    }
                }
            }

            // Skip position/velocity-dependent sensors
            _ => {}
        }
    }
}
