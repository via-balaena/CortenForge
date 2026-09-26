//! The CPU executor: [`crate::executor::Executor`] on the CPU, at `f32` (the
//! working precision, which the GPU also runs) and at `f64` (the reference
//! K3 checks `f32` against, plan §15a).
//!
//! One source, `src/cpu/executor.rs`, is compiled into both modules, as the
//! shared math is (plan §14c).

use crate::ExplicitModel;
use crate::executor::{
    Executor, Monitors, Obstacle, ObstacleError, PhaseOutputs, Snapshot, TopMode, check_obstacle,
    check_poses,
};

/// The CPU executor at `f32`.
pub mod f32 {
    use super::{
        Executor, ExplicitModel, Monitors, Obstacle, ObstacleError, PhaseOutputs, Snapshot,
        TopMode, check_obstacle, check_poses, fill, update,
    };
    use crate::f32 as shared;

    /// This module's scalar.
    type R = f32;

    include!("cpu/executor.rs");
}

/// The CPU executor at `f64`.
pub mod f64 {
    use super::{
        Executor, ExplicitModel, Monitors, Obstacle, ObstacleError, PhaseOutputs, Snapshot,
        TopMode, check_obstacle, check_poses, fill, update,
    };
    use crate::f64 as shared;

    /// This module's scalar.
    type R = f64;

    include!("cpu/executor.rs");
}

/// `out[i] = f(i)` for every index: under rayon on native, in order on
/// wasm32, which has no threads to give. Each output depends only on its
/// index, so the result does not depend on how the work is split.
#[cfg(not(target_arch = "wasm32"))]
fn fill<T: Send>(out: &mut [T], f: impl Fn(usize) -> T + Sync + Send) {
    use rayon::prelude::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
    out.par_iter_mut()
        .enumerate()
        .for_each(|(i, slot)| *slot = f(i));
}

/// `out[i] = f(i)` for every index, in order (wasm32).
#[cfg(target_arch = "wasm32")]
fn fill<T>(out: &mut [T], f: impl Fn(usize) -> T) {
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = f(i);
    }
}

/// `out[i] = f(i, out[i])` for every index, as [`fill`] runs.
#[cfg(not(target_arch = "wasm32"))]
fn update<T: Send + Copy>(out: &mut [T], f: impl Fn(usize, T) -> T + Sync + Send) {
    use rayon::prelude::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
    out.par_iter_mut()
        .enumerate()
        .for_each(|(i, slot)| *slot = f(i, *slot));
}

/// `out[i] = f(i, out[i])` for every index, in order (wasm32).
#[cfg(target_arch = "wasm32")]
fn update<T: Copy>(out: &mut [T], f: impl Fn(usize, T) -> T) {
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = f(i, *slot);
    }
}
