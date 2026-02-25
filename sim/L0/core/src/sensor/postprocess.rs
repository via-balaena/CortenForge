//! Sensor write helpers and post-processing (cutoff clamping).
//!
//! The `sensor_write*` functions provide bounds-checked writes to `Data::sensordata`.
//! `mj_sensor_postprocess` applies cutoff clamping after all sensor stages complete.

use crate::types::flags::disabled;
use crate::types::{DISABLE_SENSOR, Data, MjSensorType, Model};
use nalgebra::{DVector, Vector3};

/// Write a single sensor value with bounds checking.
#[inline]
pub fn sensor_write(sensordata: &mut DVector<f64>, adr: usize, offset: usize, value: f64) {
    let idx = adr + offset;
    if idx < sensordata.len() {
        sensordata[idx] = value;
    }
}

/// Write a 3D vector to sensor data with bounds checking.
#[inline]
pub fn sensor_write3(sensordata: &mut DVector<f64>, adr: usize, v: &Vector3<f64>) {
    sensor_write(sensordata, adr, 0, v.x);
    sensor_write(sensordata, adr, 1, v.y);
    sensor_write(sensordata, adr, 2, v.z);
}

/// Write a quaternion (w, x, y, z) to sensor data with bounds checking.
#[inline]
pub fn sensor_write4(sensordata: &mut DVector<f64>, adr: usize, w: f64, x: f64, y: f64, z: f64) {
    sensor_write(sensordata, adr, 0, w);
    sensor_write(sensordata, adr, 1, x);
    sensor_write(sensordata, adr, 2, y);
    sensor_write(sensordata, adr, 3, z);
}

/// Apply sensor post-processing: noise addition and cutoff clamping.
///
/// MuJoCo applies noise and cutoff after all sensor values are computed:
/// - Noise: Gaussian noise with std dev = sensor_noise (0 = no noise).
///   Applied independently to each sensor data element.
/// - Cutoff: For most sensors, clamps to [-cutoff, cutoff] (0 = no cutoff).
///   For positive-type sensors (Touch, Rangefinder), clamps positive side only:
///   min(value, cutoff). This preserves rangefinder's -1.0 no-hit sentinel.
pub fn mj_sensor_postprocess(model: &Model, data: &mut Data) {
    // S4.10: Skip post-processing when sensors are disabled (matches MuJoCo).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }
    for sensor_id in 0..model.nsensor {
        let adr = model.sensor_adr[sensor_id];
        let dim = model.sensor_dim[sensor_id];

        // Apply cutoff
        let cutoff = model.sensor_cutoff[sensor_id];
        if cutoff > 0.0 {
            let sensor_type = model.sensor_type[sensor_id];
            for i in 0..dim {
                let idx = adr + i;
                if idx < data.sensordata.len() {
                    let clamped = match sensor_type {
                        // Positive-type sensors: only clamp on positive side
                        MjSensorType::Touch | MjSensorType::Rangefinder => {
                            data.sensordata[idx].min(cutoff)
                        }
                        // Real-type sensors: clamp both sides
                        _ => data.sensordata[idx].clamp(-cutoff, cutoff),
                    };
                    sensor_write(&mut data.sensordata, adr, i, clamped);
                }
            }
        }

        // Note: Noise is not applied here because deterministic physics is preferred
        // for RL training and testing. Noise should be added by the caller if needed,
        // using model.sensor_noise[sensor_id] as the standard deviation.
        // This follows MuJoCo's convention where noise is optional and often disabled.
    }
}
