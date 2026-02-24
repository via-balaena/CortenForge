//! Composite Rigid Body Algorithm for mass matrix computation.
//!
//! Implements CRBA (Featherstone Chapter 6) to build the joint-space mass
//! matrix `qM` from body spatial inertias. Also caches per-body effective
//! mass/inertia for constraint force limiting.

use nalgebra::Vector6;

use crate::dynamics::shift_spatial_inertia;
use crate::joint_visitor::{JointContext, JointVisitor, joint_motion_subspace};
use crate::types::{Data, ENABLE_SLEEP, Model, SleepState};

use super::factor::mj_factor_sparse_selective;

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

/// Composite Rigid Body Algorithm: build joint-space mass matrix from body inertias.
///
/// ## Algorithm Overview
///
/// 1. Forward pass: initialize composite inertias from body spatial inertias
/// 2. Backward pass: Ic[parent] += transform(Ic[child])
/// 3. For each joint, compute M elements from Ic and joint motion subspace
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 6
#[allow(
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::op_ref
)]
pub fn mj_crba(model: &Model, data: &mut Data) {
    // ============================================================
    // Phase 0: Preamble — sleep filter + selective qM zeroing (§16.29.3)
    // ============================================================
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;

    data.qLD_valid = false;

    if sleep_filter {
        // Selective path: zero only awake trees' DOF diagonal blocks.
        // DOFs within a tree are contiguous (§16.0 body ordering guarantee).
        // Cross-tree entries in qM are always zero (CRBA's dof_parent walk
        // never crosses tree boundaries), so zeroing only the intra-tree
        // diagonal block for each awake tree is sufficient.
        for tree_id in 0..model.ntree {
            if !data.tree_awake[tree_id] {
                continue; // Sleeping tree — preserve qM entries
            }
            let dof_start = model.tree_dof_adr[tree_id];
            let dof_count = model.tree_dof_num[tree_id];
            for i in dof_start..(dof_start + dof_count) {
                for j in dof_start..(dof_start + dof_count) {
                    data.qM[(i, j)] = 0.0;
                }
            }
        }
    } else {
        // Fast path: all bodies awake, zero everything (original behavior)
        data.qM.fill(0.0);
    }

    if model.nv == 0 {
        return;
    }

    // ============================================================
    // Phase 1: Initialize composite inertias from body inertias
    // ============================================================
    // Copy cinert (computed once in FK) to crb_inertia as starting point.
    // cinert contains 6x6 spatial inertias for individual bodies in world frame.
    // Sleeping bodies' crb_inertia values are preserved from their last awake
    // step — Phase 1 is overwrite (=), not accumulate (+=), so there is no
    // stale-accumulation hazard.
    if sleep_filter {
        for idx in 0..data.nbody_awake {
            let body_id = data.body_awake_ind[idx];
            data.crb_inertia[body_id] = data.cinert[body_id];
        }
    } else {
        for body_id in 0..model.nbody {
            data.crb_inertia[body_id] = data.cinert[body_id];
        }
    }

    // ============================================================
    // Phase 2: Backward pass - accumulate composite inertias
    // ============================================================
    // Process bodies from leaves to root, adding child inertia to parent.
    // This gives Ic[i] = inertia of subtree rooted at body i.
    //
    // Each cinert[body_id] is a 6x6 spatial inertia about that body's origin
    // (xpos[body_id]). Before adding child to parent, we must shift the
    // child's spatial inertia to the parent's reference point using the
    // parallel axis theorem. Without this shift, the rotational block would
    // be incorrect because the two inertias are about different points.
    //
    // §16.29.3 §16.27 exception note: Unlike RNE backward pass (which
    // accumulates velocity-dependent cfrc_bias and needs sleeping roots for
    // correct body 0 accumulation), CRBA backward pass accumulates
    // crb_inertia (position-only, frozen for sleeping bodies) and body 0
    // has no DOFs so its crb_inertia is never consumed by Phase 3.
    // Therefore body_awake_ind (not parent_awake_ind) is correct here.
    if sleep_filter {
        // Selective: iterate only awake bodies in reverse.
        // Per-tree invariant: all bodies in a tree share sleep state, so
        // the backward accumulation only involves bodies in the same tree,
        // all of which are in body_awake_ind.
        for idx in (1..data.nbody_awake).rev() {
            let body_id = data.body_awake_ind[idx];
            let parent_id = model.body_parent[body_id];
            if parent_id != 0 {
                let d = data.xpos[body_id] - data.xpos[parent_id];
                let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
                data.crb_inertia[parent_id] += child_shifted;
            }
        }
    } else {
        for body_id in (1..model.nbody).rev() {
            let parent_id = model.body_parent[body_id];
            if parent_id != 0 {
                // Shift child's spatial inertia from child origin to parent origin,
                // then add to parent's composite inertia.
                let d = data.xpos[body_id] - data.xpos[parent_id];
                let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
                data.crb_inertia[parent_id] += child_shifted;
            }
        }
    }

    // ============================================================
    // Phase 3: Build mass matrix from composite inertias
    // ============================================================
    // Per-DOF iteration with dof_parent walk (MuJoCo-style).
    //
    // This correctly handles cross-entries M[dof_A, dof_B] for bodies with
    // multiple joints (e.g., two hinges on one body), because dof_parent
    // chains same-body DOFs together. The prior per-joint approach missed
    // these cross-entries because body_parent skipped the starting body.

    // Pre-compute per-DOF motion subspace columns (cdof).
    // First build per-joint subspaces, then extract individual DOF columns.
    // Sleeping joints' subspaces are computed but never read because the
    // outer loop skips sleeping DOFs, and the dof_parent walk only visits
    // ancestors in the same awake tree.
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    // Build cdof only for rigid DOFs (flex DOFs have dof_jnt = usize::MAX,
    // (§27F) All DOFs now have real joints — build cdof for all DOFs.
    let cdof: Vec<Vector6<f64>> = (0..model.nv)
        .map(|dof| {
            let jnt = model.dof_jnt[dof];
            let dof_in_jnt = dof - model.jnt_dof_adr[jnt];
            joint_subspaces[jnt].column(dof_in_jnt).into_owned()
        })
        .collect();

    let nv_iter = if sleep_filter {
        data.nv_awake
    } else {
        model.nv
    };
    for v in 0..nv_iter {
        let dof_i = if sleep_filter {
            data.dof_awake_ind[v]
        } else {
            v
        };
        let body_i = model.dof_body[dof_i];
        let ic = &data.crb_inertia[body_i];

        // buf = Ic[body_i] * cdof[dof_i]  (spatial force at body_i's frame)
        let mut buf: Vector6<f64> = ic * &cdof[dof_i];

        // Diagonal: M[i,i] = cdof[i]^T * buf
        data.qM[(dof_i, dof_i)] = cdof[dof_i].dot(&buf);

        // Walk dof_parent chain for off-diagonal entries.
        // All ancestors are in the same awake tree (per-tree invariant),
        // so no sleep check is needed in the ancestor walk.
        let mut current_body = body_i;
        let mut j = model.dof_parent[dof_i];

        while let Some(dof_j) = j {
            let body_j = model.dof_body[dof_j];

            // Spatial force transform only when crossing a body boundary.
            // Same-body DOFs share an origin, so no transform is needed.
            if body_j != current_body {
                let r = data.xpos[current_body] - data.xpos[body_j];

                // Shift spatial force: angular += r × linear
                // Linear part is unchanged (spatial force transform property).
                let fl_x = buf[3];
                let fl_y = buf[4];
                let fl_z = buf[5];
                buf[0] += r.y * fl_z - r.z * fl_y;
                buf[1] += r.z * fl_x - r.x * fl_z;
                buf[2] += r.x * fl_y - r.y * fl_x;

                current_body = body_j;
            }

            // Off-diagonal: M[j,i] = cdof[j]^T * buf
            let m_ji = cdof[dof_j].dot(&buf);
            data.qM[(dof_j, dof_i)] = m_ji;
            data.qM[(dof_i, dof_j)] = m_ji; // symmetry

            j = model.dof_parent[dof_j];
        }
    }

    // ============================================================
    // Phase 4: Add armature inertia to diagonal
    // ============================================================
    for jnt_id in 0..model.njnt {
        if sleep_filter && data.body_sleep_state[model.jnt_body[jnt_id]] == SleepState::Asleep {
            continue;
        }
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let nv = model.jnt_type[jnt_id].nv();
        let armature = model.jnt_armature[jnt_id];

        for i in 0..nv {
            data.qM[(dof_adr + i, dof_adr + i)] += armature;
            if let Some(&dof_arm) = model.dof_armature.get(dof_adr + i) {
                data.qM[(dof_adr + i, dof_adr + i)] += dof_arm;
            }
        }
    }

    // Phase 5b: Sparse L^T D L factorization
    // ============================================================
    // Exploits tree sparsity from dof_parent for O(n) factorization and solve.
    // Reused in mj_fwd_acceleration_explicit() and pgs_solve_contacts().
    // C3b: Selective factorization skips sleeping DOFs when sleep is enabled.
    // Sleeping DOFs' qLD entries are preserved from their last awake step.
    mj_factor_sparse_selective(model, data);

    // Phase 5c: (§27F) Flex DOFs now handled by standard sparse factorization above.

    // ============================================================
    // Phase 6: Cache body effective mass/inertia from qM diagonal
    // ============================================================
    // Extract per-body min mass/inertia for constraint force limiting.
    // This avoids O(joints) traversal per constraint.
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
