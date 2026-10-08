//! Sleep/wake state machine — body deactivation and reactivation (§16).
//!
//! Implements sleep eligibility, velocity thresholds, wake-on-contact,
//! wake-on-tendon, wake-on-equality, and the circular-linked-list sleep
//! cycle mechanism. Corresponds to MuJoCo's `engine_sleep.c`.

use crate::linalg::UnionFind;
use crate::types::{
    Data, ENABLE_SLEEP, EqualityType, MIN_AWAKE, Model, SleepError, SleepPolicy, SleepState,
};

use super::equality_trees;

// ---------------------------------------------------------------------------
// Data sleep query methods (§16.25)
// ---------------------------------------------------------------------------

impl Data {
    /// Query the sleep state of a body.
    ///
    /// Returns `SleepState::Static` for a body in no tree (the world and
    /// the bodies welded to it, except under a mocap body),
    /// `SleepState::Asleep` for the bodies of a sleeping tree and
    /// `SleepState::Awake` for the rest.
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
// Sleep update (in the advance)
// ---------------------------------------------------------------------------

/// Sleep update: check velocity thresholds, transition sleeping trees (§16.3),
/// as MuJoCo's `mj_sleep` (`engine_sleep.c:499-568`).
///
/// Called in the advance (`Data::integrate`), after the activations and
/// before the velocity update, as MuJoCo's `mj_advance` calls it. With
/// constraint rows but no islands (`DISABLE_ISLAND`) no tree sleeps.
///
/// Three-phase approach:
/// 1. Countdown: awake trees that can sleep have their timer incremented
/// 2. Island sleep: entire islands where ALL trees are ready (timer == -1)
/// 3. Singleton sleep: unconstrained trees (no island) that are ready
///
/// Returns the number of trees that were put to sleep.
// `island_id` and contact-island indices are i32 in MuJoCo's spec but always non-negative.
#[allow(clippy::cast_sign_loss)]
pub fn mj_sleep(model: &Model, data: &mut Data) -> usize {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return 0;
    }
    if !data.efc_type.is_empty() && data.nisland == 0 {
        return 0;
    }

    // Phase 1: Countdown for awake trees
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue; // Already asleep
        }
        if !tree_can_sleep(model, data, t, model.sleep_tolerance) {
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

/// Whether tree `tree` can sleep (MuJoCo `treeCanSleep`,
/// `engine_sleep.c:125-152`): its policy allows it; no `xfrc_applied` of its
/// bodies and no `qfrc_applied` of its dofs has a bit set, so `-0.0` blocks;
/// and, with `tol` other than zero, `dof_length · |qvel|` is below `tol` for
/// every dof, or, with `tol` zero, every `qvel` is `+0.0`.
fn tree_can_sleep(model: &Model, data: &Data, tree: usize, tol: f64) -> bool {
    if matches!(
        model.tree_sleep_policy[tree],
        SleepPolicy::Never | SleepPolicy::AutoNever
    ) {
        return false;
    }
    let bodies = model.tree_body_adr[tree]..model.tree_body_adr[tree] + model.tree_body_num[tree];
    if bodies
        .into_iter()
        .any(|b| !data.xfrc_applied[b].is_zero_bytes())
    {
        return false;
    }
    let dofs = model.tree_dof_adr[tree]..model.tree_dof_adr[tree] + model.tree_dof_num[tree];
    if dofs.clone().any(|d| data.qfrc_applied[d].to_bits() != 0) {
        return false;
    }
    if tol == 0.0 {
        return dofs.into_iter().all(|d| data.qvel[d].to_bits() == 0);
    }
    // MuJoCo's `isSmaller` (`:110-121`) refuses once its running maximum
    // reaches `tol`; every maximum before that is below `tol`, so it refuses
    // exactly when one product reaches `tol` (a NaN never does).
    !dofs
        .into_iter()
        .any(|d| model.dof_length[d] * data.qvel[d].abs() >= tol)
}

/// Sleep a set of trees as a circular linked list (§16.12.1), as MuJoCo's
/// `sleepTrees` (`engine_sleep.c:461-484`): each tree points at the next, and
/// its `qvel` and `qacc` are zeroed. The forward pass the advance then runs
/// recomputes the rest at the zeroed velocity.
// `nbody`/`nv` model dimensions are usize but stored as i32 in mjData; bounded by realistic model sizes.
#[allow(clippy::cast_possible_wrap)]
fn sleep_trees(model: &Model, data: &mut Data, trees: &[usize]) {
    let n = trees.len();
    for (i, &tree) in trees.iter().enumerate() {
        data.tree_asleep[tree] = trees[(i + 1) % n] as i32;
        let dofs = model.tree_dof_adr[tree]..model.tree_dof_adr[tree] + model.tree_dof_num[tree];
        for dof in dofs {
            data.qvel[dof] = 0.0;
            data.qacc[dof] = 0.0;
        }
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

/// Re-initialize all sleep state from model policies (§16.7).
///
/// Called when a `Data` is made or reset (through `Model::start_sleep`), so
/// the sleep state matches the model's tree sleep policies.
pub fn reset_sleep_state(model: &Model, data: &mut Data) {
    // First: set all trees to awake
    for t in 0..model.ntree {
        data.tree_asleep[t] = -(1 + MIN_AWAKE); // Fully awake
    }

    // Then: validate and create sleep cycles for Init trees
    if let Err(e) = validate_init_sleep(model, data) {
        // Log warning and degrade Init trees to awake (spec §16.24)
        log::warn!("Init-sleep validation failed: {e}");
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

/// Whether constraint assembly filters sleeping objects: sleep enabled and
/// some tree asleep (MuJoCo's `sleep_filter`, `engine_core_constraint.c:390`).
pub fn constraint_sleep_filter(model: &Model, data: &Data) -> bool {
    model.enableflags & ENABLE_SLEEP != 0 && data.ntree_awake < model.ntree
}

/// Whether dof `dof`'s tree is asleep (MuJoCo `mj_sleepState` of the dof's
/// body, from the sleep arrays).
pub fn dof_asleep(model: &Model, data: &Data, dof: usize) -> bool {
    !data.tree_awake[model.dof_treeid[dof]]
}

/// Whether equality `eq` is asleep: neither of its objects awake, a static
/// or missing object counting as not awake (MuJoCo `mj_equalitySleepState`,
/// `engine_sleep.c:629-660`). A tendon is awake when one of its two trees is,
/// static with none, and awake with more than two (`:572-594`).
pub fn equality_asleep(model: &Model, data: &Data, eq: usize) -> bool {
    let body_awake = |body: usize| data.body_sleep_state.get(body) == Some(&SleepState::Awake);
    let tendon_awake = |t: usize| {
        let trees = &model.tendon_tree[2 * t..2 * t + 2];
        match model.tendon_treenum[t] {
            0 => false,
            1 => data.tree_awake[trees[0]],
            2 => data.tree_awake[trees[0]] || data.tree_awake[trees[1]],
            _ => true,
        }
    };
    let awake = |id: usize| -> bool {
        match model.eq_type[eq] {
            EqualityType::Connect | EqualityType::Weld => body_awake(id),
            EqualityType::Joint => model.jnt_body.get(id).is_some_and(|&b| body_awake(b)),
            EqualityType::Distance => model.geom_body.get(id).is_some_and(|&b| body_awake(b)),
            EqualityType::Tendon => id < model.ntendon && tendon_awake(id),
        }
    };
    !awake(model.eq_obj1id[eq]) && !awake(model.eq_obj2id[eq])
}

/// A body's state from its tree's (MuJoCo `mj_updateSleepInit`,
/// `engine_sleep.c:62-82`): a body in no tree is `Static`, or `Awake` under a
/// mocap root body.
fn body_sleep_state(model: &Model, tree_awake: &[bool], body_id: usize) -> SleepState {
    let tree = model.body_treeid[body_id];
    if tree < model.ntree {
        if tree_awake[tree] {
            SleepState::Awake
        } else {
            SleepState::Asleep
        }
    } else if model.body_mocapid[model.body_rootid[body_id]].is_some() {
        SleepState::Awake
    } else {
        SleepState::Static
    }
}

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
            let state = body_sleep_state(model, &data.tree_awake, body_id);
            data.body_sleep_state[body_id] = state;
            let awake = state != SleepState::Asleep;

            // Include in body_awake_ind if awake or static
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
            // Every dof is in a tree once the tree tables are computed
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
        if !data.xfrc_applied[body_id].is_zero_bytes() {
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
        let (body1, body2) = contact.bodies(model);
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
// Body/joint indices stored as i32 in mjData are non-negative by construction.
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
/// Uses margin-aware activation matching assembly.rs and MuJoCo's
/// `mj_instantiateLimit()`: constraint fires when `dist < margin`.
fn tendon_limit_active(model: &Model, data: &Data, t: usize) -> bool {
    if !model.tendon_limited[t] {
        return false;
    }
    let length = data.ten_length[t];
    let (limit_min, limit_max) = model.tendon_range[t];
    let margin = model.tendon_margin[t];
    (length - limit_min) < margin || (limit_max - length) < margin
}

/// Wake sleeping trees coupled by multi-tree tendons with active limits (§16.13.2).
///
/// Returns `true` if any tree was woken.
// Body/joint indices stored as i32 in mjData are non-negative by construction.
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
// Body/joint indices stored as i32 in mjData are non-negative by construction.
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
// Body/joint indices stored as i32 in mjData are non-negative by construction.
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
