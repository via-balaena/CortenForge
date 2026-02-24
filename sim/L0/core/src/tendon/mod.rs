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
/// For fixed tendons: L = Σ coef_i * qpos[dof_adr_i], J[dof_adr_i] = coef_i.
/// For spatial tendons: 3D pairwise routing via `mj_fwd_tendon_spatial()`,
/// with wrap visualization data (`wrap_xpos`/`wrap_obj`, §40b).
///
/// Called from mj_fwd_position() after site transforms, before subtree COM.
pub(crate) fn mj_fwd_tendon(model: &Model, data: &mut Data) {
    if model.ntendon == 0 {
        return;
    }

    let mut wrapcount: usize = 0;
    for t in 0..model.ntendon {
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
    /// Compute `tendon_length0` for spatial tendons via FK at `qpos0`.
    ///
    /// 1. Sets `tendon_length0[t] = ten_length[t]` from the FK result.
    /// 2. Defaults `lengthspring` to `tendon_length0` when stiffness > 0.
    ///
    /// Must be called after `compute_implicit_params()` and before
    /// `compute_muscle_params()` (which needs valid `tendon_length0` for all types).
    ///
    /// Note: Rule 9 (sidesite outside wrapping geometry) is retired — sidesites
    /// inside wrapping geometry are now handled by the `wrap_inside` algorithm (§39).
    pub fn compute_spatial_tendon_length0(&mut self) {
        // Early return if no spatial tendons — avoid unnecessary FK + Data allocation.
        let has_spatial = (0..self.ntendon).any(|t| self.tendon_type[t] == TendonType::Spatial);
        if !has_spatial {
            return;
        }

        let mut data = self.make_data();
        mj_fwd_position(self, &mut data); // runs FK + mj_fwd_tendon

        // Compute tendon_length0 and default lengthspring for spatial tendons.
        for t in 0..self.ntendon {
            if self.tendon_type[t] == TendonType::Spatial {
                self.tendon_length0[t] = data.ten_length[t];
                // S3: Replace sentinel [-1, -1] with computed length at qpos0
                // Sentinel is an exact literal, never a computed float.
                #[allow(clippy::float_cmp)]
                if self.tendon_lengthspring[t] == [-1.0, -1.0] {
                    self.tendon_lengthspring[t] = [data.ten_length[t], data.ten_length[t]];
                }
            }
        }
    }
}
