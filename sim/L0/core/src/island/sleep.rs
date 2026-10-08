//! Sleep/wake state machine — body deactivation and reactivation (§16).
//!
//! Implements sleep eligibility, velocity thresholds, wake-on-contact,
//! wake-on-tendon, wake-on-equality, and the circular-linked-list sleep
//! cycle mechanism. Corresponds to MuJoCo's `engine_sleep.c`.

use crate::types::{Data, ENABLE_SLEEP, EqualityType, MIN_AWAKE, Model, SleepPolicy, SleepState};

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

/// A fully awake tree's `tree_asleep` (MuJoCo's `kAwake`).
pub const K_AWAKE: i32 = -(1 + MIN_AWAKE);

/// Every tree fully awake, and the sleep arrays (§16.7): the state MuJoCo's
/// `mj_resetData` starts from (`engine_io.c:1440`). `Model::start_sleep`
/// then puts the trees that start asleep to sleep.
pub fn reset_sleep_state(model: &Model, data: &mut Data) {
    data.tree_asleep[..model.ntree].fill(K_AWAKE);
    mj_update_sleep_arrays(model, data);
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
// Wake detection (MuJoCo engine_sleep.c:191-455)
// ---------------------------------------------------------------------------

/// Wake what the user changed (MuJoCo `mj_wake`, `engine_sleep.c:240-275`):
/// with sleep disabled, every sleeping tree; else a sleeping tree whose pose
/// the kinematics found changed (`tree_qpos_dirty`, MuJoCo's mark in
/// `tree_awake`) or that could no longer sleep at a tolerance of 0 (its
/// policy, any bit of its applied forces or of its velocity). Runs after the
/// kinematics, before the mass matrix, sleep enabled or not; clears the
/// marks. The caller refreshes the sleep arrays when it returns `true`.
pub fn mj_wake(model: &Model, data: &mut Data) -> bool {
    let ntree = model.ntree;
    let mut woke = false;
    if model.enableflags & ENABLE_SLEEP == 0 {
        if data.tree_asleep[..ntree].iter().any(|&a| a >= 0) {
            data.tree_asleep[..ntree].fill(K_AWAKE);
            woke = true;
        }
    } else {
        for t in 0..ntree {
            if data.tree_asleep[t] >= 0
                && (data.tree_qpos_dirty[t] || !tree_can_sleep(model, data, t, 0.0))
            {
                woke |= mj_wake_tree(&mut data.tree_asleep, t, K_AWAKE) > 0;
            }
        }
    }
    data.tree_qpos_dirty.fill(false);
    woke
}

/// Wake the sleeping trees an awake tree touches (MuJoCo `mj_wakeCollision`,
/// `:279-329`): geom–geom contacts only, a static partner waking nothing; the
/// woken cycle takes the awake tree's countdown. Reads the sleep arrays,
/// which the caller refreshes when it returns `true`.
pub fn mj_wake_collision(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }
    let mut woke = false;
    for c in 0..data.ncon {
        let contact = &data.contacts[c];
        if contact.flex_vertex.is_some() || contact.flex_vertex2.is_some() {
            continue;
        }
        let tree_of = |geom: usize| {
            Some(model.body_treeid[model.geom_body[geom]]).filter(|&t| t < model.ntree)
        };
        let (Some(tree1), Some(tree2)) = (tree_of(contact.geom1), tree_of(contact.geom2)) else {
            continue;
        };
        let (awake1, awake2) = (data.tree_awake[tree1], data.tree_awake[tree2]);
        if awake1 == awake2 {
            continue;
        }
        let (sleeping, awake) = if awake1 {
            (tree2, tree1)
        } else {
            (tree1, tree2)
        };
        let wakeval = data.tree_asleep[awake];
        woke |= mj_wake_tree(&mut data.tree_asleep, sleeping, wakeval) > 0;
    }
    woke
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

/// Wake the sleeping tree of a two-tree tendon at its limit whose other tree
/// is awake (MuJoCo `mj_wakeTendon`, `:333-362`), with the awake tree's
/// countdown. Reads the sleep arrays, which the caller refreshes when it
/// returns `true`.
pub fn mj_wake_tendon(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }
    let mut woke = false;
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] != 2 || !tendon_limit_active(model, data, t) {
            continue;
        }
        let (tree1, tree2) = (model.tendon_tree[2 * t], model.tendon_tree[2 * t + 1]);
        let (awake1, awake2) = (data.tree_awake[tree1], data.tree_awake[tree2]);
        if awake1 != awake2 {
            let (sleeping, awake) = if awake1 {
                (tree2, tree1)
            } else {
                (tree1, tree2)
            };
            let wakeval = data.tree_asleep[awake];
            woke |= mj_wake_tree(&mut data.tree_asleep, sleeping, wakeval) > 0;
        }
    }
    woke
}

/// Wake the trees an active connect, weld or joint equality couples to an
/// awake tree, or two sleeping trees it couples across sleep cycles (MuJoCo
/// `mj_wakeEquality`, `:366-455`), fully awake. A static object wakes
/// nothing; other equality types are left out (MuJoCo refuses a tendon
/// equality with sleep enabled, as [`Data::step`](crate::Data::step) does).
/// Reads the sleep arrays, which the caller refreshes when it returns `true`.
pub fn mj_wake_equality(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }
    let ntree = model.ntree;
    let tree_of_body = |body: usize| Some(model.body_treeid[body]).filter(|&t| t < ntree);
    let mut woke = false;
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;
        }
        let (id1, id2) = (model.eq_obj1id[eq], model.eq_obj2id[eq]);
        let (tree1, tree2) = match model.eq_type[eq] {
            EqualityType::Connect | EqualityType::Weld => (tree_of_body(id1), tree_of_body(id2)),
            EqualityType::Joint => {
                let tree_of_joint = |j: usize| model.jnt_body.get(j).and_then(|&b| tree_of_body(b));
                (tree_of_joint(id1), tree_of_joint(id2))
            }
            EqualityType::Distance | EqualityType::Tendon => continue,
        };
        // A static object, or one tree: nothing to wake
        let (Some(tree1), Some(tree2)) = (tree1, tree2) else {
            continue;
        };
        if tree1 == tree2 {
            continue;
        }
        match (data.tree_awake[tree1], data.tree_awake[tree2]) {
            (true, true) => {}
            (false, false) => {
                if mj_sleep_cycle(&data.tree_asleep, tree1)
                    != mj_sleep_cycle(&data.tree_asleep, tree2)
                {
                    woke |= mj_wake_tree(&mut data.tree_asleep, tree1, K_AWAKE) > 0;
                    woke |= mj_wake_tree(&mut data.tree_asleep, tree2, K_AWAKE) > 0;
                }
            }
            (false, true) => woke |= mj_wake_tree(&mut data.tree_asleep, tree1, K_AWAKE) > 0,
            (true, false) => woke |= mj_wake_tree(&mut data.tree_asleep, tree2, K_AWAKE) > 0,
        }
    }
    woke
}

/// The smallest tree of the sleep cycle through `start` (MuJoCo
/// `mj_sleepCycle`, `:156-187`), `start` itself for an awake tree.
// Tree indices stored as i32 in mjData are non-negative by construction.
#[allow(clippy::cast_sign_loss)]
fn mj_sleep_cycle(tree_asleep: &[i32], start: usize) -> usize {
    if tree_asleep[start] < 0 {
        return start;
    }
    let mut smallest = start;
    let mut current = tree_asleep[start] as usize;
    while current != start {
        smallest = smallest.min(current);
        current = tree_asleep[current] as usize;
    }
    smallest
}

/// Wake tree `tree` (MuJoCo `mj_wakeTree`, `:191-234`): an awake tree takes
/// `wakeval` if that is lower; a sleeping one wakes with its whole cycle,
/// each tree at `wakeval`. Returns the number woken. The sleep arrays are
/// left to the caller.
// Tree indices stored as i32 in mjData are non-negative by construction.
#[allow(clippy::cast_sign_loss)]
fn mj_wake_tree(tree_asleep: &mut [i32], tree: usize, wakeval: i32) -> usize {
    if tree_asleep[tree] < 0 {
        tree_asleep[tree] = tree_asleep[tree].min(wakeval);
        return 0;
    }
    let mut woken = 0;
    let mut current = tree;
    loop {
        let next = tree_asleep[current] as usize;
        tree_asleep[current] = wakeval;
        woken += 1;
        current = next;
        if current == tree {
            return woken;
        }
    }
}
