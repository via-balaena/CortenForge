//! Tendon kinematics — length, Jacobian, and wrapping computations.
//!
//! Dispatches to sub-modules for fixed-path and spatial-path tendon types.
//! Corresponds to the tendon section of MuJoCo's `engine_core_smooth.c`.

pub(crate) mod fixed;
pub(crate) mod spatial;
pub(crate) mod wrap_math;

pub(crate) use fixed::*;
pub(crate) use spatial::*;

use crate::types::{Data, Model, TendonType};

use crate::forward::mj_fwd_position;

/// Compute tendon lengths and Jacobians from current joint state.
///
/// For fixed tendons: `L = Σ coef_i * qpos[dof_adr_i]`, `J[dof_adr_i] = coef_i`.
/// For spatial tendons: 3D pairwise routing via `mj_fwd_tendon_spatial()`,
/// with wrap visualization data (`wrap_xpos`/`wrap_obj`, §40b).
///
/// As MuJoCo's `mj_tendon` (`engine_core_smooth.c:928-950`), every tendon's
/// length, Jacobian row and chain are cleared first, and a sleeping tendon
/// is skipped while some tree sleeps: its length and row stay 0, so it
/// makes no constraint rows, and its wrap path keeps its last address.
///
/// Called from mj_fwd_position() after site transforms and the subtree COM.
pub(crate) fn mj_fwd_tendon(model: &Model, data: &mut Data) {
    if model.ntendon == 0 {
        return;
    }

    data.ten_length.fill(0.0);
    for row in &mut data.ten_J {
        row.fill(0.0);
    }
    data.ten_J_rownnz.fill(0);
    let sleep_filter = crate::island::constraint_sleep_filter(model, data);

    let mut wrapcount: usize = 0;
    for t in 0..model.ntendon {
        if sleep_filter && crate::island::tendon_asleep(model, data, t) {
            continue;
        }
        match model.tendon_type[t] {
            TendonType::Fixed => {
                data.ten_wrapadr[t] = wrapcount;
                data.ten_wrapnum[t] = 0;
                mj_fwd_tendon_fixed(model, data, t);
            }
            TendonType::Spatial => {
                mj_fwd_tendon_spatial(model, data, t, &mut wrapcount);
            }
        }
    }
}

impl Model {
    /// Compute `tendon_length0` for spatial tendons from the kinematics at
    /// `qpos0`, and replace a spatial tendon's `tendon_lengthspring` of
    /// `[-1, -1]` with its length at `qpos_spring`, as MuJoCo's `set0` and
    /// `setSpring` do (3.5.0 `engine_setconst.c:1069-1084`). Fixed tendons:
    /// [`Self::compute_fixed_tendon_lengths`].
    ///
    /// Must be called after `compute_implicit_params()` and before
    /// `compute_actuator_params()`.
    ///
    /// Note: Rule 9 (sidesite outside wrapping geometry) is retired — sidesites
    /// inside wrapping geometry are now handled by the `wrap_inside` algorithm (§39).
    pub fn compute_spatial_tendon_length0(&mut self) {
        let spatial: Vec<usize> = (0..self.ntendon)
            .filter(|&t| self.tendon_type[t] == TendonType::Spatial)
            .collect();
        // Early return if no spatial tendons — avoid unnecessary FK + Data allocation.
        if spatial.is_empty() {
            return;
        }

        let mut data = self.make_data_for_derivation();
        mj_fwd_position(self, &mut data); // runs FK + mj_fwd_tendon at qpos0
        for &t in &spatial {
            self.tendon_length0[t] = data.ten_length[t];
        }

        // The sentinel is an exact literal, never a computed float.
        #[allow(clippy::float_cmp)]
        let unset: Vec<usize> = spatial
            .into_iter()
            .filter(|&t| self.tendon_lengthspring[t] == [-1.0, -1.0])
            .collect();
        if unset.is_empty() {
            return;
        }
        data.qpos.as_mut_slice().copy_from_slice(&self.qpos_spring);
        mj_fwd_position(self, &mut data);
        for t in unset {
            self.tendon_lengthspring[t] = [data.ten_length[t], data.ten_length[t]];
        }
    }
}
