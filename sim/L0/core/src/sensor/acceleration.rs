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
                // Proper acceleration in sensor frame. Reads cacc (spatial
                // acceleration at xpos[body_id]) and applies:
                //   1. Motion spatial transform to shift from body origin to site
                //   2. Coriolis correction: omega × v_at_site
                //   3. Rotation into sensor frame
                //
                // cacc already includes gravity pseudo-acceleration
                // (cacc[0] = [0,0,0, -gx,-gy,-gz]), so a body at rest reads
                // [0,0,+9.81] — proper acceleration. No explicit gravity
                // subtraction needed.
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

                let cacc = data.cacc[body_id];
                let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
                let a_lin = Vector3::new(cacc[3], cacc[4], cacc[5]);

                // Shift from xpos[body_id] to site_pos (motion spatial transform).
                // Convention: our cacc/cvel are at xpos[body_id] (body origin).
                // MuJoCo stores these at subtree_com[body_rootid[b]] — when porting
                // MuJoCo code, substitute xpos[body_id] for subtree_com[root].
                let r = site_pos - data.xpos[body_id];
                let a_at_site = a_lin + alpha.cross(&r);

                // Coriolis correction: a += omega × v_linear_at_site
                let cvel = data.cvel[body_id];
                let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
                let v_lin = Vector3::new(cvel[3], cvel[4], cvel[5]);
                let v_at_site = v_lin + omega.cross(&r);
                let a_corrected = a_at_site + omega.cross(&v_at_site);

                let a_sensor = site_mat.transpose() * a_corrected;
                sensor_write3(&mut data.sensordata, adr, &a_sensor);
            }

            MjSensorType::Force => {
                // Force sensor: reads cfrc_int (spatial wrench [torque; force]
                // at xpos[body_id]). Force is translation-invariant — no
                // reference-point shift needed. Rotated into site frame.
                let (body_id, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        (model.site_body[objid], data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => (objid, data.xmat[objid]),
                    _ => {
                        sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                        continue;
                    }
                };

                let cfrc = data.cfrc_int[body_id];
                let force = Vector3::new(cfrc[3], cfrc[4], cfrc[5]);
                let force_site = site_mat.transpose() * force;
                sensor_write3(&mut data.sensordata, adr, &force_site);
            }

            MjSensorType::Torque => {
                // Torque sensor: reads cfrc_int and applies spatial force
                // transform to shift torque from xpos[body_id] to site_pos:
                //   torque_at_site = torque_at_origin - r × force
                // Then rotated into site frame.
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

                let cfrc = data.cfrc_int[body_id];
                let torque_at_origin = Vector3::new(cfrc[0], cfrc[1], cfrc[2]);
                let force = Vector3::new(cfrc[3], cfrc[4], cfrc[5]);

                // Spatial force transform: shift from xpos[body_id] to site_pos.
                // Convention: our cfrc_int is at xpos[body_id] (body origin).
                // MuJoCo stores it at subtree_com[body_rootid[b]] — when porting
                // MuJoCo code, substitute xpos[body_id] for subtree_com[root].
                let r = site_pos - data.xpos[body_id];
                let torque_at_site = torque_at_origin - r.cross(&force);

                let torque_site = site_mat.transpose() * torque_at_site;
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
                // Linear acceleration in world frame. Reads cacc with spatial
                // motion transform + Coriolis correction.
                //
                // Includes gravity pseudo-acceleration (matches MuJoCo).
                // A static body reads [0, 0, +9.81]. This is a conformance
                // fix — the previous implementation returned [0, 0, 0].
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

                let cacc = data.cacc[body_id];
                let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
                let a_lin = Vector3::new(cacc[3], cacc[4], cacc[5]);
                // Convention: our cacc is at xpos[body_id], not subtree_com[root].
                // See Accelerometer arm above for full porting note.
                let r = obj_pos - data.xpos[body_id];
                let a_at_point = a_lin + alpha.cross(&r);

                // Coriolis correction
                let cvel = data.cvel[body_id];
                let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
                let v_lin = Vector3::new(cvel[3], cvel[4], cvel[5]);
                let v_at_point = v_lin + omega.cross(&r);
                let a_corrected = a_at_point + omega.cross(&v_at_point);

                sensor_write3(&mut data.sensordata, adr, &a_corrected);
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
