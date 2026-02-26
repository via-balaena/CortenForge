//! Acceleration-dependent sensor evaluation (`mj_sensor_acc`).
//!
//! Called after constraint solve. Evaluates sensors that depend on acceleration:
//! accelerometer, force/torque, touch, frame linear/angular acceleration,
//! actuator force, joint/tendon limit force.

use crate::types::flags::disabled;
use crate::types::{
    ConstraintType, DISABLE_SENSOR, Data, ENABLE_SLEEP, MjObjectType, MjSensorDataType,
    MjSensorType, Model, SensorStage, SleepState,
};
use nalgebra::{Matrix3, Vector3};

use crate::forward::mj_body_accumulators;

use super::derived::{
    compute_body_acceleration, compute_body_angular_acceleration, compute_site_force_torque,
};
use super::postprocess::{sensor_write, sensor_write3};
use super::sensor_body_id;

/// Compute acceleration-dependent sensor values.
///
/// This is called after `mj_fwd_acceleration` and computes sensors that depend
/// on acceleration:
/// - Accelerometer: linear acceleration (includes gravity in sensor frame)
/// - Force: constraint force at site
/// - Torque: constraint torque at site
/// - `FrameLinAcc`: linear acceleration
/// - `FrameAngAcc`: angular acceleration
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
                // Linear acceleration in sensor frame (includes gravity)
                // a_sensor = R^T * (a_world - g)
                // For a body at rest in gravity, accelerometer reads +g (opposing gravity)
                let (a_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        // Compute body acceleration from qacc
                        let a = compute_body_acceleration(model, data, body_id);
                        (a, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let a = compute_body_acceleration(model, data, objid);
                        (a, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Proper acceleration = a_body - g_field
                // At rest: a_body=0, g_field=(0,0,-9.81), so a_proper=(0,0,+9.81) (upward)
                // In free fall: a_body~g_field, so a_proper~0. Matches real IMU behavior.
                let a_proper = a_world - model.gravity;
                let a_sensor = site_mat.transpose() * a_proper;
                sensor_write3(&mut data.sensordata, adr, &a_sensor);
            }

            MjSensorType::Force => {
                // Force sensor: measures the interaction force at a site.
                //
                // MuJoCo computes this via inverse dynamics on the subtree rooted at
                // the site's body. The force is the net force that the rest of the
                // system exerts on the subtree through the site's body, expressed
                // in the site's local frame.
                //
                // F_site = R_site^T * (m*a - f_external)
                //
                // We compute this as: the total constraint + applied force transmitted
                // through the kinematic chain at the sensor's body, projected into
                // the site frame.
                let (force_world, _) = compute_site_force_torque(model, data, sensor_id);
                // Transform to site frame
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let force_site = site_mat.transpose() * force_world;
                sensor_write3(&mut data.sensordata, adr, &force_site);
            }

            MjSensorType::Torque => {
                // Torque sensor: measures the interaction torque at a site.
                //
                // Similar to Force but returns the torque component. The torque is
                // the net torque that the rest of the system exerts on the subtree
                // through the sensor body, expressed in the site's local frame.
                let (_, torque_world) = compute_site_force_torque(model, data, sensor_id);
                // Transform to site frame
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let torque_site = site_mat.transpose() * torque_world;
                sensor_write3(&mut data.sensordata, adr, &torque_site);
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
                // Linear acceleration in world frame
                let a = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &a);
            }

            MjSensorType::FrameAngAcc => {
                // Angular acceleration in world frame
                // Compute from qacc using Jacobian
                let alpha = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_angular_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_angular_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
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
