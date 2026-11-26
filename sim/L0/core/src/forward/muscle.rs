//! Muscle force-length-velocity curves and parameter precomputation — entry point.
//!
//! This module re-exports from siblings [`hill`](super::hill) and
//! [`fiber`](super::fiber), and houses an actuator's length range as MuJoCo
//! 3.5.0's compiler computes it ([`Model::set_length_range`]: its
//! `mjCModel::LengthRange` and `mj_setLengthRange`).

// Re-export hill.rs — muscle curves and activation dynamics.
pub use super::hill::{
    muscle_activation_dynamics, muscle_bias, muscle_f0, muscle_gain, muscle_gain_length,
    muscle_gain_velocity, muscle_passive_force,
};

use super::fiber::build_actuator_moment;
use crate::dynamics::factor::mj_solve_m;
use crate::linalg::mju_norm;
use crate::types::{
    ActuatorTransmission, BiasType, DISABLE_ACTUATION, DISABLE_CONTACT, DISABLE_DAMPER,
    DISABLE_FRICTIONLOSS, DISABLE_GRAVITY, DISABLE_SPRING, Data, ENABLE_SLEEP, GainType,
    LengthRangeError, LengthRangeMode, LengthRangeOpt, MIN_VAL, Model, StepError,
};

/// The forces MuJoCo's compiler turns off while it computes the length
/// ranges, replacing the model's disable flags (`user_model.cc:2411-2412`):
/// every other force and constraint the model disables is on for the run.
const LENGTH_RANGE_DISABLE: u32 = DISABLE_FRICTIONLOSS
    | DISABLE_CONTACT
    | DISABLE_SPRING
    | DISABLE_DAMPER
    | DISABLE_GRAVITY
    | DISABLE_ACTUATION;

/// C's `mjMAX(a, b)`, `a > b ? a : b`: `b` where either is NaN.
fn mj_max(a: f64, b: f64) -> f64 {
    if a > b { a } else { b }
}

impl Model {
    /// Each actuator's length range, as MuJoCo 3.5.0's compiler computes it
    /// after `mj_setConst` (`mjCModel::LengthRange`, `user_model.cc:2408-2507`,
    /// and `mj_setLengthRange`, `engine_setconst.c:1145-1272`), actuator by
    /// actuator, stopping at the first that fails.
    ///
    /// An actuator `opt.mode` leaves out keeps its range, as does one whose
    /// range increases under `opt.useexisting`. Under `opt.uselimit` a joint
    /// or tendon transmission whose joint or tendon is limited takes its range
    /// as written. Otherwise two runs from `qpos0`, one pushing the length
    /// down and one up, each with the force that gives an acceleration of
    /// `opt.accel`, find the range: the least length of the first run's last
    /// `opt.interval` and the greatest of the second's. Each run steps a copy
    /// of the model whose disable flags are friction loss, contacts, springs,
    /// dampers, gravity and actuation (in place of the model's), with sleep
    /// off and the timestep `opt.timestep` where it is positive.
    ///
    /// Not part of [`Self::compute_actuator_params`] nor
    /// [`Self::recompute_derived`], as MuJoCo's `mj_setConst` leaves the
    /// length range out (`engine_setconst.c:1088`).
    ///
    /// Each actuator that runs steps the model twice for `opt.inttotal`, all on
    /// the calling thread (MuJoCo's compiler, under `usethread` and with two
    /// or more actuators to run, shares them among threads).
    ///
    /// # Errors
    ///
    /// The first actuator's [`LengthRangeError`]. Its range is then the range
    /// its completed runs found, as MuJoCo's model holds it: both sides for
    /// an empty range or one that does not converge, the first side's bound
    /// at most for the others. The actuators after it keep theirs.
    pub fn set_length_range(&mut self, opt: &LengthRangeOpt) -> Result<(), LengthRangeError> {
        let mut run = self.clone();
        run.disableflags = LENGTH_RANGE_DISABLE;
        // The compiler's `mjData` is made with sleep off (`user_model.cc:5111-5112`).
        // Kept as MuJoCo's step: the run's `Data` already starts every tree awake.
        run.enableflags &= !ENABLE_SLEEP;
        if opt.timestep > 0.0 {
            run.timestep = opt.timestep;
        }
        for i in 0..self.nu {
            self.set_actuator_length_range(&run, i, opt)?;
        }
        Ok(())
    }

    /// One actuator's [`Self::set_length_range`], its runs stepping `run`.
    fn set_actuator_length_range(
        &mut self,
        run: &Self,
        i: usize,
        opt: &LengthRangeOpt,
    ) -> Result<(), LengthRangeError> {
        let is_muscle = self.actuator_gaintype[i] == GainType::Muscle
            || self.actuator_biastype[i] == BiasType::Muscle;
        let is_user = self.actuator_gaintype[i] == GainType::User
            || self.actuator_biastype[i] == BiasType::User;
        let skip = match opt.mode {
            LengthRangeMode::None => true,
            LengthRangeMode::Muscle => !is_muscle,
            LengthRangeMode::MuscleUser => !is_muscle && !is_user,
            LengthRangeMode::All => false,
        };
        let (lo, hi) = self.actuator_lengthrange[i];
        if skip || (opt.useexisting && lo < hi) {
            return Ok(());
        }
        if opt.uselimit {
            let id = self.actuator_trnid[i][0];
            match self.actuator_trntype[i] {
                ActuatorTransmission::Joint | ActuatorTransmission::JointInParent
                    if self.jnt_limited[id] =>
                {
                    self.actuator_lengthrange[i] = self.jnt_range[id];
                    return Ok(());
                }
                ActuatorTransmission::Tendon if self.tendon_limited[id] => {
                    self.actuator_lengthrange[i] = self.tendon_range[id];
                    return Ok(());
                }
                _ => {}
            }
        }

        // The time never reaches an infinite `inttotal`; every other option
        // runs as MuJoCo's comparisons take it.
        if opt.inttotal == f64::INFINITY {
            return Err(LengthRangeError::InfiniteInttotal { actuator: i });
        }

        let (mut lmin, mut lmax) = ([0.0_f64; 2], [0.0_f64; 2]);
        for side in 0..2 {
            let mut data = run.make_data_for_derivation();
            let mut updated = false;
            // MuJoCo's loop, on the time each step advances.
            #[allow(clippy::while_float)]
            while data.time < opt.inttotal {
                let len = eval_act(run, &mut data, i, side, opt)?;
                // A step that reset the state (registry `D-LENGTHRANGE-RESET`).
                // MuJoCo's own "Unstable", a step that leaves the time at 0,
                // is a zero timestep's, which `eval_act` refuses so.
                if data.divergence_detected() {
                    return Err(LengthRangeError::Unstable { actuator: i });
                }
                if data.time > opt.inttotal - opt.interval {
                    if len < lmin[side] || !updated {
                        lmin[side] = len;
                    }
                    if len > lmax[side] || !updated {
                        lmax[side] = len;
                    }
                    updated = true;
                }
            }
            if side == 0 {
                self.actuator_lengthrange[i].0 = lmin[0];
            } else {
                self.actuator_lengthrange[i].1 = lmax[1];
            }
        }

        let range = self.actuator_lengthrange[i];
        let dif = range.1 - range.0;
        if dif <= 0.0 {
            return Err(LengthRangeError::InvalidRange { actuator: i, range });
        }
        for side in 0..2 {
            if lmax[side] - lmin[side] > opt.tolrange * dif {
                return Err(LengthRangeError::NotConverged {
                    actuator: i,
                    side,
                    eval: (lmin[side], lmax[side]),
                    range,
                });
            }
        }
        Ok(())
    }
}

/// One step of a length-range run, MuJoCo's `evalAct`
/// (`engine_setconst.c:1107-1141`): the velocity damped by
/// `exp(-h / max(0.01, timeconst))`, the position and velocity stages, the
/// force along the actuator's moment that gives an acceleration of
/// `opt.accel` (pushing the length down on side 0, up on side 1) capped at
/// `opt.maxforce`, then the acceleration stage and the advance. Returns the
/// actuator's length at the position stage, before the advance.
fn eval_act(
    run: &Model,
    data: &mut Data,
    i: usize,
    side: usize,
    opt: &LengthRangeOpt,
) -> Result<f64, LengthRangeError> {
    let refused = |error: StepError| match error {
        // A zero timestep leaves the time at 0, MuJoCo's "Unstable".
        StepError::InvalidTimestep if run.timestep == 0.0 => {
            LengthRangeError::Unstable { actuator: i }
        }
        error => LengthRangeError::Step { actuator: i, error },
    };
    let decay = (-run.timestep / mj_max(0.01, opt.timeconst)).exp();
    for v in data.qvel.iter_mut() {
        *v *= decay;
    }
    data.step1(run).map_err(refused)?;

    let moment = build_actuator_moment(run, data, i);
    let mut x = moment.clone();
    mj_solve_m(run, data, x.as_mut_slice());
    let sign = if side == 0 { -1.0 } else { 1.0 };
    let scale = sign * opt.accel / mj_max(MIN_VAL, mju_norm(x.as_slice()));
    for (f, m) in data.qfrc_applied.iter_mut().zip(moment.iter()) {
        *f = m * scale;
    }
    let nrm = mju_norm(data.qfrc_applied.as_slice());
    if opt.maxforce > 0.0 && nrm > opt.maxforce {
        let scale = opt.maxforce / mj_max(MIN_VAL, nrm);
        for f in data.qfrc_applied.iter_mut() {
            *f *= scale;
        }
    }

    data.step2_stages(run).map_err(refused)?;
    Ok(data.actuator_length[i])
}
