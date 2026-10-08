//! Actuator and sensor history buffers: delays, as MuJoCo 3.5.0.
//!
//! The ring-buffer primitives (`mju_historyInsert`, `mju_historyRead`,
//! `engine_util_misc.c:913-1124`), their initialisation in `_resetData`
//! (`engine_io.c:1377-1427`), the delayed reads (`mj_readCtrl`,
//! `mj_readSensor`, `engine_support.c:845-890`), the read-or-compute choice
//! for sensors (`compute_or_read_sensor`, `engine_sensor.c:1346-1388`) and
//! the insertion when the state advances (`mj_advance`,
//! `engine_forward.c:837-884`).
//!
//! The buffer of an actuator or a sensor with `nsample > 0` starts at its
//! `historyadr` in [`Data::history`](crate::Data::history):
//! `[user, cursor, times(n), values(n * dim)]`. `user` is a sensor's last
//! compute tick in interval mode; `cursor` is the physical index of the
//! newest sample, stored as an `f64` as MuJoCo stores it as an `mjtNum`.

use crate::types::{Data, InterpolationType, MjSensorType, Model};

/// MuJoCo's `mjMINVAL` (`mjtnum.h:26`): two times closer than this are one.
const MINVAL: f64 = 1e-15;

/// The physical index of logical index `logical` (0 is the oldest sample,
/// `n - 1` the newest). MuJoCo `historyPhysicalIndex`.
fn physical(cursor: usize, n: usize, logical: usize) -> usize {
    (cursor + 1 + logical) % n
}

/// The buffer's cursor. MuJoCo `(int)buf[1]`.
// It is a small non-negative integer stored as an f64, always in [0, n).
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn cursor_of(buf: &[f64]) -> usize {
    buf[1] as usize
}

/// The smallest logical `i` with `times[physical(i)] >= t`: 0 if `t` is at
/// or before the oldest sample, `n` if it is after the newest. MuJoCo
/// `historyFindIndex`.
fn find_index(times: &[f64], n: usize, cursor: usize, t: f64) -> usize {
    if t <= times[physical(cursor, n, 0)] {
        return 0;
    }
    if t > times[physical(cursor, n, n - 1)] {
        return n;
    }
    let (mut lo, mut hi) = (0, n - 1);
    while hi - lo > 1 {
        let mid = usize::midpoint(lo, hi);
        if times[physical(cursor, n, mid)] < t {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// Make room for a sample at time `t` and return the offset, in `buf`, of
/// the `dim` values the caller writes: the sample at `t` if there is one,
/// else a new sample that drops the oldest. MuJoCo `mju_historyInsert`.
pub fn insert(buf: &mut [f64], n: usize, dim: usize, t: f64) -> usize {
    let cursor = cursor_of(buf);
    let i = find_index(&buf[2..2 + n], n, cursor, t);
    let values = 2 + n;
    if i < n {
        let p = physical(cursor, n, i);
        if (t - buf[2 + p]).abs() < MINVAL {
            return values + p * dim;
        }
    }
    // older than the oldest: replace the oldest
    if i == 0 {
        let p = physical(cursor, n, 0);
        buf[2 + p] = t;
        return values + p * dim;
    }
    // newer than the newest: advance the cursor onto the oldest
    if i == n {
        let cursor = (cursor + 1) % n;
        #[allow(clippy::cast_precision_loss)] // n <= 2^24, exact in an f64
        {
            buf[1] = cursor as f64;
        }
        buf[2 + cursor] = t;
        return values + cursor * dim;
    }
    // out of order: shift logical [1, i - 1] down one, dropping the oldest
    for j in 0..i - 1 {
        let src = physical(cursor, n, j + 1);
        let dst = physical(cursor, n, j);
        buf[2 + dst] = buf[2 + src];
        buf.copy_within(
            values + src * dim..values + (src + 1) * dim,
            values + dst * dim,
        );
    }
    let p = physical(cursor, n, i - 1);
    buf[2 + p] = t;
    values + p * dim
}

/// The value at time `t`: `Some(offset)` of a stored sample (before the
/// oldest, after the newest, an exact match, or zero-order hold), or `None`
/// after writing an interpolated value into `res`. Times outside the buffer
/// clamp to its ends; nothing is extrapolated. MuJoCo `mju_historyRead`.
pub fn read(
    buf: &[f64],
    n: usize,
    dim: usize,
    res: &mut [f64],
    t: f64,
    interp: InterpolationType,
) -> Option<usize> {
    let cursor = cursor_of(buf);
    let times = &buf[2..2 + n];
    let values = 2 + n;
    let oldest = physical(cursor, n, 0);
    let newest = physical(cursor, n, n - 1);
    if t <= times[oldest] + MINVAL {
        return Some(values + oldest * dim);
    }
    if t >= times[newest] - MINVAL {
        return Some(values + newest * dim);
    }
    let i = find_index(times, n, cursor, t);
    let hi = physical(cursor, n, i);
    if (t - times[hi]).abs() < MINVAL {
        return Some(values + hi * dim);
    }
    let lo = physical(cursor, n, i - 1);
    if interp == InterpolationType::Zoh {
        return Some(values + lo * dim);
    }
    let dt = times[hi] - times[lo];
    let alpha = (t - times[lo]) / dt;
    let v = |p: usize, d: usize| buf[values + p * dim + d];
    if interp == InterpolationType::Linear {
        for (d, r) in res.iter_mut().enumerate().take(dim) {
            *r = v(lo, d) + alpha * (v(hi, d) - v(lo, d));
        }
    } else {
        // cubic Hermite with Catmull-Rom slopes, 0 at the buffer's ends
        let alpha2 = alpha * alpha;
        let alpha3 = alpha2 * alpha;
        let h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
        let h10 = alpha3 - 2.0 * alpha2 + alpha;
        let h01 = -2.0 * alpha3 + 3.0 * alpha2;
        let h11 = alpha3 - alpha2;
        for (d, r) in res.iter_mut().enumerate().take(dim) {
            let m_lo = if i > 1 {
                let lo_prev = physical(cursor, n, i - 2);
                (v(hi, d) - v(lo_prev, d)) / (times[hi] - times[lo_prev])
            } else {
                0.0
            };
            let m_hi = if i < n - 1 {
                let hi_next = physical(cursor, n, i + 1);
                (v(hi_next, d) - v(lo, d)) / (times[hi_next] - times[lo])
            } else {
                0.0
            };
            *r = h00 * v(lo, d) + h10 * dt * m_lo + h01 * v(hi, d) + h11 * dt * m_hi;
        }
    }
    None
}

/// `nsample` as a sample count.
// Callers check `nsample > 0` first.
#[allow(clippy::cast_sign_loss)]
fn count(nsample: i32) -> usize {
    nsample as usize
}

/// `historyadr` as an index.
// It is >= 0 whenever nsample > 0, which callers check first.
#[allow(clippy::cast_sign_loss)]
fn address(historyadr: i32) -> usize {
    historyadr as usize
}

/// Fill `history` (length `model.nhistory`) as `_resetData` does: each
/// buffer's newest sample last, timestamps one timestep apart and ending at
/// `-timestep` (or, for an interval sensor, one period apart and ending at
/// its phase, each rounded up to a timestep), and values 0.
#[allow(clippy::cast_precision_loss)] // sample counts <= 2^24
pub fn init(model: &Model, history: &mut [f64]) {
    history.fill(0.0);
    let dt = model.timestep;
    for i in 0..model.nu {
        if model.actuator_nsample[i] <= 0 {
            continue;
        }
        let n = count(model.actuator_nsample[i]);
        let buf = &mut history[address(model.actuator_historyadr[i])..];
        buf[1] = (n - 1) as f64;
        for j in 0..n {
            buf[2 + j] = -((n - j) as f64) * dt;
        }
    }
    for i in 0..model.nsensor {
        if model.sensor_nsample[i] <= 0 {
            continue;
        }
        let n = count(model.sensor_nsample[i]);
        let (period, phase) = model.sensor_interval[i];
        let buf = &mut history[address(model.sensor_historyadr[i])..];
        let t0 = if phase == 0.0 { -period } else { phase };
        buf[0] = if period > 0.0 { t0 } else { -dt };
        buf[1] = (n - 1) as f64;
        for j in 0..n {
            buf[2 + j] = if period > 0.0 {
                ((t0 - ((n - 1 - j) as f64) * period) / dt).ceil() * dt
            } else {
                -((n - j) as f64) * dt
            };
        }
    }
}

/// The control actuator `i` acts on at `time`: `ctrl[i]` when it has no
/// buffer, else its buffer read at `time - actuator_delay[i]` with the
/// model's interpolation. MuJoCo `mj_readCtrl`.
pub fn read_ctrl(model: &Model, data: &Data, i: usize, time: f64) -> f64 {
    if model.actuator_nsample[i] <= 0 {
        return data.ctrl[i];
    }
    let buf = &data.history[address(model.actuator_historyadr[i])..];
    let mut res = [0.0];
    match read(
        buf,
        count(model.actuator_nsample[i]),
        1,
        &mut res,
        time - model.actuator_delay[i],
        model.actuator_interp[i],
    ) {
        Some(offset) => buf[offset],
        None => res[0],
    }
}

/// Whether sensor `i` takes its value from its buffer in this pass instead
/// of being computed: a delayed sensor, or an interval sensor between its
/// ticks. User and plugin sensors never do; MuJoCo's stage loops handle them
/// apart. MuJoCo `compute_or_read_sensor`.
pub fn sensor_reads_history(model: &Model, data: &Data, i: usize) -> bool {
    // MuJoCo's own guard (`mj_advance` checks `nhistory`); it also keeps a
    // code-built model that leaves the history arrays empty working.
    if model.nhistory == 0 || model.sensor_nsample[i] <= 0 {
        return false;
    }
    if matches!(
        model.sensor_type[i],
        MjSensorType::User | MjSensorType::Plugin
    ) {
        return false;
    }
    if model.sensor_delay[i] > 0.0 {
        return true;
    }
    let period = model.sensor_interval[i].0;
    period > 0.0 && data.history[address(model.sensor_historyadr[i])] + period > data.time
}

/// Write sensor `i`'s value at `data.time - sensor_delay[i]`, read from its
/// buffer, into `sensordata`. MuJoCo `mj_readSensor`.
pub fn read_sensor(model: &Model, data: &mut Data, i: usize) {
    let dim = model.sensor_dim[i];
    let adr = model.sensor_adr[i];
    let buf = &data.history[address(model.sensor_historyadr[i])..];
    let out = &mut data.sensordata.as_mut_slice()[adr..adr + dim];
    let t = data.time - model.sensor_delay[i];
    if let Some(offset) = read(
        buf,
        count(model.sensor_nsample[i]),
        dim,
        out,
        t,
        model.sensor_interp[i],
    ) {
        out.copy_from_slice(&buf[offset..offset + dim]);
    }
}

/// Insert every buffered actuator's `ctrl` at `time` (`engine_forward.c:838-847`).
pub fn advance_ctrl(model: &Model, data: &mut Data, time: f64) {
    if model.nhistory == 0 {
        return;
    }
    for i in 0..model.nu {
        if model.actuator_nsample[i] <= 0 {
            continue;
        }
        let buf = &mut data.history[address(model.actuator_historyadr[i])..];
        let offset = insert(buf, count(model.actuator_nsample[i]), 1, time);
        buf[offset] = data.ctrl[i];
    }
}

/// Insert a sample of every buffered sensor at `data.time`
/// (`engine_forward.c:849-883`): computed now, with its cutoff, when the
/// sensor is delayed; copied from `sensordata` otherwise. An interval sensor
/// inserts only on its tick.
pub fn advance_sensors(model: &Model, data: &mut Data) {
    if model.nhistory == 0 {
        return;
    }
    for i in 0..model.nsensor {
        if model.sensor_nsample[i] <= 0 {
            continue;
        }
        let at = address(model.sensor_historyadr[i]);
        let period = model.sensor_interval[i].0;
        if period > 0.0 {
            if data.history[at] + period > data.time {
                continue;
            }
            data.history[at] += period;
        }
        let n = count(model.sensor_nsample[i]);
        let offset = at + insert(&mut data.history[at..], n, model.sensor_dim[i], data.time);
        fill_slot(model, data, i, offset);
    }
}

/// Write sensor `i`'s sample into `history[offset..offset + dim]`.
fn fill_slot(model: &Model, data: &mut Data, i: usize, offset: usize) {
    let dim = model.sensor_dim[i];
    let adr = model.sensor_adr[i];
    if model.sensor_delay[i] > 0.0 {
        // compute into sensordata with the slot's value parked there, then swap back
        swap(data, adr, offset, dim);
        crate::sensor::compute_sensor(model, data, i);
        swap(data, adr, offset, dim);
    } else {
        let src = &data.sensordata.as_slice()[adr..adr + dim];
        data.history[offset..offset + dim].copy_from_slice(src);
    }
}

fn swap(data: &mut Data, adr: usize, offset: usize, dim: usize) {
    data.sensordata.as_mut_slice()[adr..adr + dim]
        .swap_with_slice(&mut data.history[offset..offset + dim]);
}
