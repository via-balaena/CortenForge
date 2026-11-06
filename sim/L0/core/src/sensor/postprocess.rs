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

/// Apply cutoff clamping to a single sensor.
///
/// Extracted from `mj_sensor_postprocess` for use by plugin sensor dispatch
/// (§66), which applies cutoff per-sensor immediately after plugin compute.
pub fn apply_sensor_cutoff(model: &Model, data: &mut Data, sensor_id: usize) {
    let adr = model.sensor_adr[sensor_id];
    let dim = model.sensor_dim[sensor_id];
    let cutoff = model.sensor_cutoff[sensor_id];
    if cutoff <= 0.0 {
        return;
    }
    let sensor_type = model.sensor_type[sensor_id];
    if sensor_type == MjSensorType::GeomFromTo {
        return;
    }
    let data_kind = sensor_type.data_kind();
    for i in 0..dim {
        let idx = adr + i;
        if idx < data.sensordata.len() {
            let cut = under_cutoff(data_kind, cutoff, data.sensordata[idx]);
            sensor_write(&mut data.sensordata, adr, i, cut);
        }
    }
}

/// `value` under `cutoff` as MuJoCo's `apply_cutoff` takes it
/// (`engine_sensor.c:79-87`), with its `mju_clip` and `mju_min`
/// (`engine_util_misc.c:1286-1314`): a real value held within `[-cutoff,
/// cutoff]`, a positive one at or below `cutoff`, an axis or quaternion left.
/// By comparisons, as theirs: a NaN cutoff leaves the value, and a NaN value
/// stays NaN.
fn under_cutoff(kind: MjSensorDataKind, cutoff: f64, value: f64) -> f64 {
    match kind {
        MjSensorDataKind::Real => {
            if value < -cutoff {
                -cutoff
            } else if value > cutoff {
                cutoff
            } else {
                value
            }
        }
        MjSensorDataKind::Positive => {
            if cutoff <= value {
                cutoff
            } else {
                value
            }
        }
        MjSensorDataKind::Axis | MjSensorDataKind::Quaternion => value,
    }
}

/// Apply sensor post-processing: cutoff clamping.
///
/// Matches MuJoCo's `apply_cutoff()` in `engine_sensor.c:64–89` using a
/// two-layer dispatch:
///
/// **Layer 1 — explicit type exemptions:**
/// - `GeomFromTo` (`mjSENS_GEOMFROMTO`): cutoff is consumed during evaluation
///   as the `mj_geomDistance()` search radius, not as a value clamp.
///
/// **Layer 2 — data-kind dispatch** (`MjSensorDataKind`), as
/// `under_cutoff` takes each value: `Real` held within `[-cutoff, cutoff]`,
/// `Positive` at or below `cutoff` (a negative value is kept), `Axis` and
/// `Quaternion` left (MuJoCo's if/else chain has branches for REAL and
/// POSITIVE only).
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
        // A value read from the history buffer was clamped when it was
        // computed, and an interpolated one is not clamped: MuJoCo applies the
        // cutoff in `mj_computeSensor` only (`engine_sensor.c:1341-1342`).
        if crate::history::sensor_reads_history(model, data, sensor_id) {
            continue;
        }

        let sensor_type = model.sensor_type[sensor_id];

        // Layer 1: explicit type exemptions.
        // MuJoCo: `if (type == mjSENS_CONTACT || type == mjSENS_GEOMFROMTO) return;`
        // sim-core has no contact sensor (`mjSENS_CONTACT`); touch is
        // `mjSENS_TOUCH`, which MuJoCo clamps.
        if sensor_type == MjSensorType::GeomFromTo {
            continue;
        }

        // Layer 2: data-kind dispatch.
        let data_kind = sensor_type.data_kind();
        for i in 0..dim {
            let idx = adr + i;
            if idx < data.sensordata.len() {
                let cut = under_cutoff(data_kind, cutoff, data.sensordata[idx]);
                sensor_write(&mut data.sensordata, adr, i, cut);
            }
        }
    }
}
