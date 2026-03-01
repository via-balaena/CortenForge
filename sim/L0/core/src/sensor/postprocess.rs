//! Sensor write helpers and post-processing (cutoff clamping).
//!
//! The `sensor_write*` functions provide bounds-checked writes to `Data::sensordata`.
//! `mj_sensor_postprocess` applies cutoff clamping after all sensor stages complete.

use crate::types::flags::disabled;
use crate::types::{DISABLE_SENSOR, Data, MjSensorDataKind, MjSensorType, Model};
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

/// Write a 6D vector to sensor data with bounds checking.
/// Used by GeomFromTo sensor (fromto points).
#[inline]
pub fn sensor_write6(sensordata: &mut DVector<f64>, adr: usize, v: &[f64; 6]) {
    sensor_write(sensordata, adr, 0, v[0]);
    sensor_write(sensordata, adr, 1, v[1]);
    sensor_write(sensordata, adr, 2, v[2]);
    sensor_write(sensordata, adr, 3, v[3]);
    sensor_write(sensordata, adr, 4, v[4]);
    sensor_write(sensordata, adr, 5, v[5]);
}

/// Apply sensor post-processing: cutoff clamping.
///
/// Matches MuJoCo's `apply_cutoff()` in `engine_sensor.c:64–89` using a
/// two-layer dispatch:
///
/// **Layer 1 — explicit type exemptions:**
/// - `Touch` (`mjSENS_CONTACT`): cutoff ignored entirely. MuJoCo does not
///   clamp touch sensor values; cutoff has no effect on touch sensors.
/// - `GeomFromTo` (`mjSENS_GEOMFROMTO`): cutoff is consumed during evaluation
///   as the `mj_geomDistance()` search radius, not as a value clamp.
///
/// **Layer 2 — data-kind dispatch** (`MjSensorDataKind`):
/// - `Real` → `clamp(-cutoff, cutoff)` — symmetric clamping.
/// - `Positive` → `min(cutoff, value)` — preserves negative sentinels (e.g.,
///   rangefinder's -1.0 no-hit value).
/// - `Axis` / `Quaternion` → no clamping. MuJoCo's if/else chain only has
///   branches for REAL and POSITIVE; AXIS and QUATERNION fall through without
///   clamping. This correctly skips FrameXAxis/YAxis/ZAxis, GeomNormal,
///   BallQuat, and FrameQuat.
///
/// Noise is not applied here — CortenForge keeps the forward pass fully
/// deterministic. Noise metadata (`sensor_noise`) is available for
/// caller-side injection if needed.
pub fn mj_sensor_postprocess(model: &Model, data: &mut Data) {
    // S4.10: Skip post-processing when sensors are disabled (matches MuJoCo).
    if disabled(model, DISABLE_SENSOR) {
        return;
    }
    for sensor_id in 0..model.nsensor {
        let adr = model.sensor_adr[sensor_id];
        let dim = model.sensor_dim[sensor_id];

        let cutoff = model.sensor_cutoff[sensor_id];
        if cutoff <= 0.0 {
            continue;
        }

        let sensor_type = model.sensor_type[sensor_id];

        // Layer 1: explicit type exemptions.
        // MuJoCo: `if (type == mjSENS_CONTACT || type == mjSENS_GEOMFROMTO) return;`
        if matches!(sensor_type, MjSensorType::Touch | MjSensorType::GeomFromTo) {
            continue;
        }

        // Layer 2: data-kind dispatch.
        // MuJoCo: if/else chain on sensor_datatype — REAL clamps, POSITIVE
        // clamps one-sided, AXIS/QUATERNION fall through (no clamping).
        let data_kind = sensor_type.data_kind();
        match data_kind {
            MjSensorDataKind::Real => {
                for i in 0..dim {
                    let idx = adr + i;
                    if idx < data.sensordata.len() {
                        let clamped = data.sensordata[idx].clamp(-cutoff, cutoff);
                        sensor_write(&mut data.sensordata, adr, i, clamped);
                    }
                }
            }
            MjSensorDataKind::Positive => {
                for i in 0..dim {
                    let idx = adr + i;
                    if idx < data.sensordata.len() {
                        let clamped = data.sensordata[idx].min(cutoff);
                        sensor_write(&mut data.sensordata, adr, i, clamped);
                    }
                }
            }
            // AXIS and QUATERNION: no clamping (MuJoCo has no branch for these).
            MjSensorDataKind::Axis | MjSensorDataKind::Quaternion => {}
        }
    }
}
