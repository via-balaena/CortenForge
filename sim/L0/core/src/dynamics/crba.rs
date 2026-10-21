//! Composite Rigid Body Algorithm for mass matrix computation.
//!
//! Builds the joint-space mass matrix `qM` from the bodies' composite
//! inertias as MuJoCo's `mj_crb`. Also caches per-body effective
//! mass/inertia for constraint force limiting.

use crate::dynamics::spatial::mju_mul_inert_vec;
use crate::joint_visitor::{JointContext, JointVisitor};
use crate::linalg::mji_dot6;
use crate::types::{Data, ENABLE_SLEEP, Model, SleepState};

use super::factor::mj_factor_m;

/// Minimum inertia/mass threshold for numerical stability.
///
/// Values below this threshold are treated as numerical noise and ignored
/// when computing effective mass for constraint force limiting.
const MIN_INERTIA_THRESHOLD: f64 = 1e-10;

/// Default mass/inertia when no valid DOFs are found for a body.
///
/// This is used when a body has no translational/rotational DOFs that we
/// can extract mass from (e.g., kinematic bodies). Using 1.0 provides
/// reasonable default behavior.
pub const DEFAULT_MASS_FALLBACK: f64 = 1.0;

/// The composite inertias `crb` and the joint-space mass matrix `qM`, as
/// MuJoCo's `mj_crb` (`engine_core_smooth.c:1811-1885`): each body's `cinert`
/// summed into its ancestors' (the world's left out), then for each awake
/// dof `i` and each dof `j` it descends from (itself included),
/// `M(i, j) = cdof_j · (crb_body(i) · cdof_i)`, plus `i`'s armature on the
/// diagonal; a simple dof's row is its diagonal alone, `dof_M0`. Then the
/// factor (`mj_factor_m`) and each body's smallest mass and inertia on the
/// diagonal.
pub fn mj_crba(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    data.qLD_valid = false;

    // The composite inertias, then the mass matrix, as MuJoCo's `mj_crb`
    // (`engine_core_smooth.c:1811-1885`), whose sleep filter counts dofs.
    let crb_filter = sleep_enabled && data.nv_awake < model.nv;
    let (nbody, nparent) = if crb_filter {
        (data.nbody_awake, data.nparent_awake)
    } else {
        (model.nbody, model.nbody)
    };

    // crb = cinert, then each body's added into its parent's (not the world's).
    for b in 0..nbody {
        let i = if crb_filter {
            data.body_awake_ind[b]
        } else {
            b
        };
        data.crb[i] = data.cinert[i];
    }
    for b in (0..nparent).rev() {
        let i = if crb_filter {
            data.parent_awake_ind[b]
        } else {
            b
        };
        let parent = model.body_parent[i];
        if parent > 0 {
            let child = data.crb[i];
            for (sum, add) in data.crb[parent].iter_mut().zip(child) {
                *sum += add;
            }
        }
    }

    if model.nv == 0 {
        return;
    }

    // Clear the matrix, or under the filter the awake dofs' rows on its
    // pattern (`mju_zeroSparse`): a sleeping dof's row keeps its values.
    if crb_filter {
        for v in 0..data.nv_awake {
            let i = data.dof_awake_ind[v];
            let mut j = Some(i);
            while let Some(jj) = j {
                data.qM[(i, jj)] = 0.0;
                data.qM[(jj, i)] = 0.0;
                j = model.dof_parent[jj];
            }
        }
    } else {
        data.qM.fill(0.0);
    }

    // Each awake dof's row: its armature, then `cdof_j · (crb_i · cdof_i)`
    // for it and each dof it descends from. A simple dof's row is its
    // diagonal alone, `dof_M0` (so two slides of one body on one axis are not
    // coupled).
    let nv = if crb_filter { data.nv_awake } else { model.nv };
    for v in 0..nv {
        let i = if crb_filter { data.dof_awake_ind[v] } else { v };
        if model.dof_simplenum.get(i).is_some_and(|&n| n > 0) {
            data.qM[(i, i)] = model.dof_M0[i];
            continue;
        }
        let buf = mju_mul_inert_vec(&data.crb[model.dof_body[i]], &data.cdof[i]);
        data.qM[(i, i)] = model.dof_armature[i];
        let mut j = Some(i);
        while let Some(jj) = j {
            data.qM[(i, jj)] += mji_dot6(data.cdof[jj].as_slice(), buf.as_slice());
            data.qM[(jj, i)] = data.qM[(i, jj)];
            j = model.dof_parent[jj];
        }
    }

    // M = L'·D·L on M's pattern (MuJoCo's mj_factorM): the awake
    // rows only when a tree sleeps, a sleeping row keeping its factor.
    mj_factor_m(model, data);

    // ============================================================
    // Cache body effective mass/inertia from qM diagonal
    // ============================================================
    // Extract per-body min mass/inertia for constraint force limiting.
    // This avoids O(joints) traversal per constraint.
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;
    cache_body_effective_mass(model, data, sleep_filter);
}

/// Cache per-body minimum mass and inertia from the mass matrix diagonal.
///
/// This extracts the minimum diagonal elements for each body's DOFs and stores
/// them in `data.body_min_mass` and `data.body_min_inertia`. These cached values
/// are used by constraint force limiting to avoid repeated mass matrix queries.
///
/// Uses the `JointVisitor` pattern to ensure consistency with joint iteration
/// elsewhere in the codebase.
///
/// When `sleep_filter` is true, sleeping bodies' cached values are preserved
/// from their last awake step (not recomputed, not zeroed). All three sub-phases
/// (reset, visitor, fallback) skip sleeping bodies. (§16.29.3 Phase 6)
///
/// Must be called after `mj_crba()` has computed the mass matrix.
fn cache_body_effective_mass(model: &Model, data: &mut Data, sleep_filter: bool) {
    // Visitor struct for JointVisitor pattern (defined before statements per clippy)
    struct MassCacheVisitor<'a> {
        model: &'a Model,
        data: &'a mut Data,
        sleep_filter: bool,
    }

    impl JointVisitor for MassCacheVisitor<'_> {
        fn visit_free(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
            // Linear DOFs at 0-2
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let mass = self.data.qM[(dof, dof)];
                    if mass > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_mass[body_id] =
                            self.data.body_min_mass[body_id].min(mass);
                    }
                }
            }
            // Angular DOFs at 3-5
            for i in 3..6 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_ball(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
            // All 3 DOFs are angular
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_hinge(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
            // Single angular DOF
            if ctx.dof_adr < self.model.nv {
                let inertia = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if inertia > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_inertia[body_id] =
                        self.data.body_min_inertia[body_id].min(inertia);
                }
            }
        }

        fn visit_slide(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
            // Single linear DOF
            if ctx.dof_adr < self.model.nv {
                let mass = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if mass > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_mass[body_id] = self.data.body_min_mass[body_id].min(mass);
                }
            }
        }
    }

    // Sub-phase 1: Reset only awake bodies.
    // CRITICAL: sleeping bodies' cached values must be preserved.
    for i in 1..model.nbody {
        if sleep_filter && data.body_sleep_state[i] == SleepState::Asleep {
            continue; // Preserve cached values from last awake step
        }
        data.body_min_mass[i] = f64::INFINITY;
        data.body_min_inertia[i] = f64::INFINITY;
    }

    // Sub-phase 2: Visitor with sleep guard.
    let mut visitor = MassCacheVisitor {
        model,
        data,
        sleep_filter,
    };
    model.visit_joints(&mut visitor);

    // Sub-phase 3: Fallback — only awake bodies.
    for i in 1..model.nbody {
        if sleep_filter && data.body_sleep_state[i] == SleepState::Asleep {
            continue; // Preserve cached values from last awake step
        }
        if data.body_min_mass[i] == f64::INFINITY {
            data.body_min_mass[i] = DEFAULT_MASS_FALLBACK;
        }
        if data.body_min_inertia[i] == f64::INFINITY {
            data.body_min_inertia[i] = DEFAULT_MASS_FALLBACK;
        }
    }
}
