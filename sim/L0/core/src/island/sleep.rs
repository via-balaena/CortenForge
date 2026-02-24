//! Sleep/wake state machine — body deactivation and reactivation (§16).
//!
//! Implements sleep eligibility, velocity thresholds, wake-on-contact,
//! wake-on-tendon, wake-on-equality, and the circular-linked-list sleep
//! cycle mechanism. Corresponds to MuJoCo's sleep logic in `engine_island.c`.

use nalgebra::{UnitQuaternion, Vector3};

use crate::dynamics::SpatialVector;
use crate::linalg::UnionFind;
use crate::types::{
    Data, ENABLE_SLEEP, MIN_AWAKE, MjJointType, Model, SleepError, SleepPolicy, SleepState,
};

use super::equality_trees;

// ---------------------------------------------------------------------------
// Data sleep query methods (§16.25)
// ---------------------------------------------------------------------------

impl Data {
    /// Query the sleep state of a body.
    ///
    /// Returns `SleepState::Static` for the world body (body 0),
    /// `SleepState::Asleep` for sleeping bodies, `SleepState::Awake`
    /// for active bodies.
    #[must_use]
    pub fn sleep_state(&self, body_id: usize) -> SleepState {
        self.body_sleep_state[body_id]
    }

    /// Query whether a kinematic tree is awake.
    #[must_use]
    pub fn tree_awake(&self, tree_id: usize) -> bool {
        self.tree_asleep[tree_id] < 0
    }

    /// Query the number of awake bodies (including the world body).
    #[must_use]
    pub fn nbody_awake(&self) -> usize {
        self.nbody_awake
    }

    /// Query the number of constraint islands discovered this step.
    #[must_use]
    pub fn nisland(&self) -> usize {
        self.nisland
    }
}

// ---------------------------------------------------------------------------
// Sleep update (end-of-step)
// ---------------------------------------------------------------------------

/// Sleep update: check velocity thresholds, transition sleeping trees (§16.3).
///
/// Called at the end of `step()`, after integration completes and before
/// the warmstart save. This is the central sleep state machine.
/// Phase B sleep transition function (§16.12.2).
///
/// Three-phase approach:
/// 1. Countdown: awake trees that can sleep have their timer incremented
/// 2. Island sleep: entire islands where ALL trees are ready (timer == -1)
/// 3. Singleton sleep: unconstrained trees (no island) that are ready
///
/// Returns the number of trees that were put to sleep.
#[allow(clippy::cast_sign_loss)]
pub fn mj_sleep(model: &Model, data: &mut Data) -> usize {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return 0;
    }

    // Phase 1: Countdown for awake trees
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue; // Already asleep
        }
        if !tree_can_sleep(model, data, t) {
            data.tree_asleep[t] = -(1 + MIN_AWAKE); // Reset
            continue;
        }
        // Increment toward -1 (ready to sleep)
        if data.tree_asleep[t] < -1 {
            data.tree_asleep[t] += 1;
        }
    }

    let mut nslept = 0;

    // Phase 2: Sleep entire islands where ALL trees are ready
    for island in 0..data.nisland {
        let itree_start = data.island_itreeadr[island];
        let itree_end = itree_start + data.island_ntree[island];

        let all_ready = (itree_start..itree_end).all(|idx| {
            let tree = data.map_itree2tree[idx];
            data.tree_asleep[tree] == -1
        });

        if all_ready {
            let trees: Vec<usize> = (itree_start..itree_end)
                .map(|idx| data.map_itree2tree[idx])
                .collect();
            sleep_trees(model, data, &trees);
            nslept += trees.len();
        }
    }

    // Phase 3: Sleep unconstrained singleton trees that are ready
    for t in 0..model.ntree {
        if data.tree_island[t] < 0 && data.tree_asleep[t] == -1 {
            sleep_trees(model, data, &[t]); // Self-link
            nslept += 1;
        }
    }

    nslept
}

/// Check if a tree is eligible to sleep (§16.12.2).
///
/// Returns `false` if policy forbids sleeping, external forces are applied,
/// or any DOF velocity exceeds the sleep threshold.
fn tree_can_sleep(model: &Model, data: &Data, tree: usize) -> bool {
    // Policy check
    if model.tree_sleep_policy[tree] == SleepPolicy::Never
        || model.tree_sleep_policy[tree] == SleepPolicy::AutoNever
    {
        return false;
    }

    // External force check: xfrc_applied on any body in tree
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        let f = &data.xfrc_applied[body_id];
        if f[0] != 0.0 || f[1] != 0.0 || f[2] != 0.0 || f[3] != 0.0 || f[4] != 0.0 || f[5] != 0.0 {
            return false;
        }
    }

    // External force check: qfrc_applied on any DOF in tree
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        if data.qfrc_applied[dof] != 0.0 {
            return false;
        }
    }

    // Velocity threshold check
    tree_velocity_below_threshold(model, data, tree)
}

/// Check if all DOFs in a tree have velocities below the sleep threshold.
fn tree_velocity_below_threshold(model: &Model, data: &Data, tree: usize) -> bool {
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        if data.qvel[dof].abs() > model.sleep_tolerance * model.dof_length[dof] {
            return false;
        }
    }
    true // All DOFs below threshold (L∞ norm check)
}

/// Sleep a set of trees as a circular linked list (§16.12.1).
///
/// Creates a circular sleep cycle among the given trees, zeros all DOF-level
/// and body-level arrays, and syncs xpos/xquat with post-integration qpos.
/// For a single tree, this creates a self-link (Phase A compatible).
#[allow(clippy::cast_possible_wrap)]
fn sleep_trees(model: &Model, data: &mut Data, trees: &[usize]) {
    let n = trees.len();
    if n == 0 {
        return;
    }

    for i in 0..n {
        let tree = trees[i];
        let next = trees[(i + 1) % n];

        // Create circular linked list
        data.tree_asleep[tree] = next as i32;

        // Zero DOF-level arrays
        let dof_start = model.tree_dof_adr[tree];
        let dof_end = dof_start + model.tree_dof_num[tree];
        for dof in dof_start..dof_end {
            data.qvel[dof] = 0.0;
            data.qacc[dof] = 0.0;
            data.qfrc_bias[dof] = 0.0;
            data.qfrc_passive[dof] = 0.0;
            data.qfrc_constraint[dof] = 0.0;
            data.qfrc_actuator[dof] = 0.0; // §16.26.7: needed for policy relaxation
        }

        // Zero body-level arrays
        let body_start = model.tree_body_adr[tree];
        let body_end = body_start + model.tree_body_num[tree];
        for body_id in body_start..body_end {
            data.cvel[body_id] = SpatialVector::zeros();
            data.cacc_bias[body_id] = SpatialVector::zeros();
            data.cfrc_bias[body_id] = SpatialVector::zeros();
        }

        // Sync xpos/xquat with post-integration qpos (§16.15 compatibility)
        sync_tree_fk(model, data, tree);
    }
}

/// Recompute FK for a single tree's bodies to sync xpos/xquat with current qpos.
///
/// Called after integration to prevent false positives in qpos change detection.
fn sync_tree_fk(model: &Model, data: &mut Data, tree: usize) {
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        let parent_id = model.body_parent[body_id];
        let mut pos = data.xpos[parent_id];
        let mut quat = data.xquat[parent_id];

        // Apply body offset in parent frame
        pos += quat * model.body_pos[body_id];
        quat *= model.body_quat[body_id];

        // Apply each joint
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];
        for jnt_id in jnt_start..jnt_end {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let angle = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    let anchor = model.jnt_pos[jnt_id];
                    let world_anchor = pos + quat * anchor;
                    let world_axis = quat * axis;
                    let rot = if let Some(unit_axis) = nalgebra::Unit::try_new(world_axis, 1e-10) {
                        UnitQuaternion::from_axis_angle(&unit_axis, angle)
                    } else {
                        UnitQuaternion::identity()
                    };
                    quat = rot * quat;
                    pos = world_anchor + rot * (pos - world_anchor);
                }
                MjJointType::Slide => {
                    let displacement = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    pos += quat * (axis * displacement);
                }
                MjJointType::Ball => {
                    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    quat *= q;
                }
                MjJointType::Free => {
                    pos = Vector3::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                    );
                    quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ));
                }
            }
        }
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

/// Re-initialize all sleep state from model policies (§16.7).
///
/// Called by `Data::reset()` and `Data::reset_to_keyframe()` to ensure sleep
/// state matches the model's tree sleep policies after a reset.
pub fn reset_sleep_state(model: &Model, data: &mut Data) {
    // First: set all trees to awake
    for t in 0..model.ntree {
        data.tree_asleep[t] = -(1 + MIN_AWAKE); // Fully awake
    }

    // Then: validate and create sleep cycles for Init trees
    if let Err(e) = validate_init_sleep(model, data) {
        // Log warning and degrade Init trees to awake (spec §16.24)
        #[cfg(debug_assertions)]
        eprintln!("Init-sleep validation failed: {e}");
        let _ = e; // Suppress unused warning in release
    }

    mj_update_sleep_arrays(model, data);
}

/// Validate Init-sleep trees and create sleep cycles (§16.24).
///
/// Uses union-find over model-time adjacency (equality constraints +
/// multi-tree tendons) to group Init trees. Creates circular sleep cycles
/// per group. Returns an error if validation fails.
fn validate_init_sleep(model: &Model, data: &mut Data) -> Result<(), SleepError> {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return Ok(());
    }

    // Phase 1: Basic per-tree validation
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] != SleepPolicy::Init {
            continue;
        }
        if model.tree_dof_num[t] == 0 {
            return Err(SleepError::InitSleepInvalidTree { tree: t });
        }
    }

    // Phase 2: Check for mixed Init/non-Init in statically-coupled groups
    let mut uf = UnionFind::new(model.ntree);

    // Equality constraint edges
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;
        }
        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a < model.ntree && tree_b < model.ntree && tree_a != tree_b {
            uf.union(tree_a, tree_b);
        }
    }

    // Multi-tree tendon edges
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] == 2 {
            let tree_a = model.tendon_tree[2 * t];
            let tree_b = model.tendon_tree[2 * t + 1];
            if tree_a < model.ntree && tree_b < model.ntree {
                uf.union(tree_a, tree_b);
            }
        }
    }

    // Check each group for mixed Init/non-Init
    let mut group_has_init = vec![false; model.ntree];
    let mut group_has_noninit = vec![false; model.ntree];
    for t in 0..model.ntree {
        let root = uf.find(t);
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            group_has_init[root] = true;
        } else {
            group_has_noninit[root] = true;
        }
    }
    for root in 0..model.ntree {
        if group_has_init[root] && group_has_noninit[root] {
            return Err(SleepError::InitSleepMixedIsland { group_root: root });
        }
    }

    // Phase 3: Create sleep cycles for validated Init-sleep groups
    // Group Init trees by their union-find root
    let mut init_groups: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            let root = uf.find(t);
            init_groups.entry(root).or_default().push(t);
        }
    }
    for trees in init_groups.values() {
        sleep_trees(model, data, trees);
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Derived sleep arrays
// ---------------------------------------------------------------------------

/// Recompute derived sleep arrays from `tree_asleep` (§16.3, §16.17).
///
/// Updates: tree_awake, body_sleep_state, ntree_awake, nv_awake,
/// and the awake-index indirection arrays (body_awake_ind, parent_awake_ind, dof_awake_ind).
pub fn mj_update_sleep_arrays(model: &Model, data: &mut Data) {
    data.ntree_awake = 0;
    data.nv_awake = 0;

    for t in 0..model.ntree {
        let awake = data.tree_asleep[t] < 0;
        data.tree_awake[t] = awake;
        if awake {
            data.ntree_awake += 1;
        }
    }

    // --- Body sleep states + body_awake_ind + parent_awake_ind (§16.17.1) ---
    let mut nbody_awake = 0;
    let mut nparent_awake = 0;

    // Body 0 (world) is always Static and always in both indirection arrays
    if !data.body_sleep_state.is_empty() {
        data.body_sleep_state[0] = SleepState::Static;
        if !data.body_awake_ind.is_empty() {
            data.body_awake_ind[0] = 0;
            nbody_awake = 1;
        }
        if !data.parent_awake_ind.is_empty() {
            data.parent_awake_ind[0] = 0;
            nparent_awake = 1;
        }
    }

    // Update per-body sleep states and build indirection arrays
    if model.body_treeid.len() == model.nbody {
        for body_id in 1..model.nbody {
            let tree = model.body_treeid[body_id];
            let awake = if tree < model.ntree {
                data.tree_awake[tree]
            } else {
                true // No tree info → treat as awake
            };

            data.body_sleep_state[body_id] = if awake {
                SleepState::Awake
            } else {
                SleepState::Asleep
            };

            // Include in body_awake_ind if awake
            if awake && nbody_awake < data.body_awake_ind.len() {
                data.body_awake_ind[nbody_awake] = body_id;
                nbody_awake += 1;
            }

            // Include in parent_awake_ind if parent is awake or static
            let parent = model.body_parent[body_id];
            let parent_awake = data.body_sleep_state[parent] != SleepState::Asleep;
            if parent_awake && nparent_awake < data.parent_awake_ind.len() {
                data.parent_awake_ind[nparent_awake] = body_id;
                nparent_awake += 1;
            }
        }
    }

    data.nbody_awake = nbody_awake;
    data.nparent_awake = nparent_awake;

    // --- DOF awake indices (§16.17.1) ---
    let mut nv_awake = 0;
    for dof in 0..model.nv {
        let is_awake = if model.dof_treeid.len() > dof {
            let tree = model.dof_treeid[dof];
            // Flex DOFs have tree == usize::MAX (no tree) → always awake
            tree >= model.ntree || data.tree_awake[tree]
        } else {
            // No tree info → treat as awake
            true
        };
        if is_awake {
            if nv_awake < data.dof_awake_ind.len() {
                data.dof_awake_ind[nv_awake] = dof;
            }
            nv_awake += 1;
        }
    }
    data.nv_awake = nv_awake;
}

// ---------------------------------------------------------------------------
// Wake detection
// ---------------------------------------------------------------------------

/// Check if any sleeping tree's qpos was externally modified (§16.15).
///
/// Reads `tree_qpos_dirty` flags set by `mj_fwd_position()` during FK,
/// wakes affected trees, then clears all dirty flags.
/// Returns `true` if any tree was newly woken.
pub fn mj_check_qpos_changed(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntree {
        if data.tree_qpos_dirty[t] && data.tree_asleep[t] >= 0 {
            // Tree was sleeping but FK detected a pose change from external qpos modification.
            mj_wake_tree(model, data, t);
            woke_any = true;
        }
    }

    // Clear all dirty flags (whether or not they triggered a wake)
    data.tree_qpos_dirty.fill(false);

    woke_any
}

/// Wake detection: check user-applied forces on sleeping bodies (§16.4).
///
/// Called at the start of `forward()`, before any pipeline stage.
///
/// Returns `true` if any tree was woken (caller must update sleep arrays).
pub fn mj_wake(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;

    // Check xfrc_applied (per-body Cartesian forces)
    for body_id in 1..model.nbody {
        if data.body_sleep_state[body_id] != SleepState::Asleep {
            continue;
        }
        // Bytewise nonzero check (matches MuJoCo: -0.0 wakes because sign bit is set).
        // Use to_bits() != 0 instead of != 0.0 because IEEE 754 treats -0.0 == 0.0.
        let force = &data.xfrc_applied[body_id];
        if force.iter().any(|&v| v.to_bits() != 0) {
            mj_wake_tree(model, data, model.body_treeid[body_id]);
            woke_any = true;
        }
    }

    // Check qfrc_applied (per-DOF generalized forces)
    for dof in 0..model.nv {
        let tree = model.dof_treeid[dof];
        if !data.tree_awake[tree] && data.qfrc_applied[dof].to_bits() != 0 {
            mj_wake_tree(model, data, tree);
            woke_any = true;
        }
    }

    woke_any
}

/// Wake detection after collision: check contacts between sleeping and awake bodies (§16.4).
///
/// Returns `true` if any tree was woken (triggers re-collision).
pub fn mj_wake_collision(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for contact_idx in 0..data.ncon {
        let contact = &data.contacts[contact_idx];
        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];
        let state1 = data.body_sleep_state[body1];
        let state2 = data.body_sleep_state[body2];

        // Wake sleeping body if partner is awake (not static — static bodies
        // like the world/ground don't wake sleeping bodies).
        let need_wake = match (state1, state2) {
            (SleepState::Asleep, SleepState::Awake) => Some(body1),
            (SleepState::Awake, SleepState::Asleep) => Some(body2),
            _ => None,
        };

        if let Some(body_id) = need_wake {
            let tree = model.body_treeid[body_id];
            if tree < model.ntree {
                mj_wake_tree(model, data, tree);
                woke_any = true;
            }
        }
    }
    woke_any
}

/// Return the canonical (minimum) tree index in a sleep cycle (§16.10.3).
///
/// Used to identify whether two sleeping trees belong to the same cycle.
#[allow(clippy::cast_sign_loss)]
fn mj_sleep_cycle(tree_asleep: &[i32], start: usize) -> usize {
    if tree_asleep[start] < 0 {
        return start; // Not asleep — return self
    }
    let mut min_tree = start;
    let mut current = tree_asleep[start] as usize;
    while current != start {
        if current < min_tree {
            min_tree = current;
        }
        current = tree_asleep[current] as usize;
    }
    min_tree
}

/// Check if a tendon's limit constraint is active (§16.13.2).
fn tendon_limit_active(model: &Model, data: &Data, t: usize) -> bool {
    if !model.tendon_limited[t] {
        return false;
    }
    let length = data.ten_length[t];
    let (limit_min, limit_max) = model.tendon_range[t];
    length < limit_min || length > limit_max
}

/// Wake sleeping trees coupled by multi-tree tendons with active limits (§16.13.2).
///
/// Returns `true` if any tree was woken.
#[allow(clippy::cast_sign_loss)]
pub fn mj_wake_tendon(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] != 2 {
            continue;
        }
        if !tendon_limit_active(model, data, t) {
            continue;
        }

        let tree_a = model.tendon_tree[2 * t];
        let tree_b = model.tendon_tree[2 * t + 1];
        if tree_a >= model.ntree || tree_b >= model.ntree {
            continue;
        }
        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep in different cycles: merge by waking both
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
            }
            _ => {} // Both awake — no action
        }
    }
    woke_any
}

/// Wake sleeping trees coupled by active equality constraints (§16.13.3).
///
/// Returns `true` if any tree was woken.
#[allow(clippy::cast_sign_loss)]
pub fn mj_wake_equality(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;
        }

        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a >= model.ntree || tree_b >= model.ntree || tree_a == tree_b {
            continue; // Same tree or invalid — no cross-tree coupling
        }

        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep in different cycles: merge by waking both
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
            }
            _ => {} // Both awake — no action
        }
    }
    woke_any
}

/// Wake a tree and its entire sleep cycle (§16.12.3).
///
/// Traverses the circular linked list to wake all trees in the sleeping
/// island. Eagerly updates `tree_awake` and `body_sleep_state` so
/// subsequent wake functions in the same pass see the updated state.
#[allow(clippy::cast_sign_loss)]
fn mj_wake_tree(model: &Model, data: &mut Data, tree: usize) {
    if data.tree_awake[tree] {
        return; // Already awake
    }

    if data.tree_asleep[tree] < 0 {
        // Awake but tree_awake flag stale — just update the flag
        data.tree_awake[tree] = true;
        return;
    }

    // Traverse the sleep cycle, waking each tree
    let mut current = tree;
    loop {
        let next = data.tree_asleep[current] as usize;
        data.tree_asleep[current] = -(1 + MIN_AWAKE); // Fully awake
        data.tree_awake[current] = true;

        // Update body states
        let body_start = model.tree_body_adr[current];
        let body_end = body_start + model.tree_body_num[current];
        for body_id in body_start..body_end {
            data.body_sleep_state[body_id] = SleepState::Awake;
        }

        current = next;
        if current == tree {
            break; // Full cycle traversed
        }
    }
}
