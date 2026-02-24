//! Island discovery — constraint-graph connected components (§16.11).
//!
//! Builds a tree-tree adjacency graph from contacts, equality constraints,
//! and multi-tree tendons, then runs flood-fill to partition awake trees into
//! constraint islands. Corresponds to MuJoCo's `engine_island.c`.

#![allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]

pub(crate) mod sleep;

use crate::types::{ConstraintType, DISABLE_ISLAND, Data, EqualityType, Model};

// Re-exports — island functions for external consumers.
pub(crate) use sleep::{
    mj_check_qpos_changed, mj_sleep, mj_update_sleep_arrays, mj_wake, mj_wake_collision,
    mj_wake_equality, mj_wake_tendon, reset_sleep_state,
};

/// Discover constraint islands from the active constraint set (§16.11).
///
/// Builds a tree-tree adjacency graph from contacts, equality constraints,
/// and multi-tree tendons, then runs flood-fill to find connected components.
/// Populates all island arrays in `data`.
///
/// Works directly from raw data sources (contacts, model equality constraints,
/// tendon limits, joint limits) rather than from `efc_*` arrays, so it is
/// independent of which solver path (Newton vs PGS/CG) is active.
#[allow(
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]
pub(crate) fn mj_island(model: &Model, data: &mut Data) {
    let ntree = model.ntree;

    // Early return: DISABLE_ISLAND flag (§16.11.5)
    if model.disableflags & DISABLE_ISLAND != 0 || ntree == 0 {
        data.nisland = 0;
        data.tree_island[..ntree].fill(-1);
        return;
    }

    // === Phase 1: Edge extraction (§16.11.2) ===
    // Collect edges as (tree_a, tree_b) pairs from raw data sources.
    // Only awake trees participate in island discovery — sleeping trees
    // don't contribute to constraint solving.

    // Clear scratch
    data.island_scratch_rownnz[..ntree].fill(0);
    data.island_scratch_colind.clear();

    // Helper closure: returns true if tree is awake (eligible for islands)
    let tree_awake = |tree: usize| -> bool { tree < ntree && data.tree_asleep[tree] < 0 };

    let ncon = data.contacts.len();
    let capacity = ncon + model.neq + model.njnt + model.ntendon;
    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(capacity);

    // 1a: Contacts → tree pairs from geom → body → tree
    for contact in &data.contacts {
        let body1 = if contact.geom1 < model.geom_body.len() {
            model.geom_body[contact.geom1]
        } else {
            continue;
        };
        let body2 = if contact.geom2 < model.geom_body.len() {
            model.geom_body[contact.geom2]
        } else {
            continue;
        };
        let tree1 = if body1 > 0 && body1 < model.body_treeid.len() {
            model.body_treeid[body1]
        } else {
            usize::MAX // World body
        };
        let tree2 = if body2 > 0 && body2 < model.body_treeid.len() {
            model.body_treeid[body2]
        } else {
            usize::MAX // World body
        };

        if tree1 < ntree && tree2 < ntree && tree_awake(tree1) && tree_awake(tree2) {
            edges.push((tree1, tree2));
            if tree1 != tree2 {
                edges.push((tree2, tree1));
            }
        } else if tree1 < ntree && tree_awake(tree1) {
            edges.push((tree1, tree1)); // Contact with world
        } else if tree2 < ntree && tree_awake(tree2) {
            edges.push((tree2, tree2)); // Contact with world
        }
    }

    // 1b: Active equality constraints → tree pairs (awake only)
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id] {
            continue;
        }
        let (tree1, tree2) = equality_trees(model, eq_id);
        if tree_awake(tree1) {
            edges.push((tree1, tree1));
            if tree_awake(tree2) && tree2 != tree1 {
                edges.push((tree1, tree2));
                edges.push((tree2, tree1));
            } else if tree_awake(tree2) {
                edges.push((tree2, tree2));
            }
        } else if tree_awake(tree2) {
            edges.push((tree2, tree2));
        }
    }

    // 1c: Active joint limits → self-edges (awake only)
    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }
        // Check if limit is actually violated (active)
        let (limit_min, limit_max) = model.jnt_range[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let q = data.qpos[qpos_adr];
        if q < limit_min || q > limit_max {
            let body = model.jnt_body[jnt_id];
            if body > 0 && body < model.body_treeid.len() {
                let tree = model.body_treeid[body];
                if tree_awake(tree) {
                    edges.push((tree, tree));
                }
            }
        }
    }

    // 1d: Active tendon limits → tree pair edges (awake only)
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }
        // Check if tendon limit is actually violated (active)
        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];
        if length < limit_min || length > limit_max {
            if model.tendon_treenum[t] == 2 {
                let t1 = model.tendon_tree[2 * t];
                let t2 = model.tendon_tree[2 * t + 1];
                if tree_awake(t1) && tree_awake(t2) {
                    edges.push((t1, t2));
                    edges.push((t2, t1));
                }
            } else if model.tendon_treenum[t] == 1 {
                let tree = model.tendon_tree[2 * t];
                if tree_awake(tree) {
                    edges.push((tree, tree));
                }
            }
        }
    }

    // === Phase 2: Build CSR adjacency graph (§16.11.3) ===

    // Count edges per tree (rownnz)
    for &(ta, _) in &edges {
        if ta < ntree {
            data.island_scratch_rownnz[ta] += 1;
        }
    }

    // Compute row addresses (prefix sum)
    data.island_scratch_rowadr[..ntree].fill(0);
    if ntree > 0 {
        let mut prefix = 0;
        for t in 0..ntree {
            data.island_scratch_rowadr[t] = prefix;
            prefix += data.island_scratch_rownnz[t];
        }
        data.island_scratch_colind.resize(prefix, 0);
    }

    // Fill column indices
    let mut fill_count = vec![0usize; ntree];
    for &(ta, tb) in &edges {
        if ta < ntree {
            let idx = data.island_scratch_rowadr[ta] + fill_count[ta];
            if idx < data.island_scratch_colind.len() {
                data.island_scratch_colind[idx] = tb;
            }
            fill_count[ta] += 1;
        }
    }

    // === Phase 3: Flood fill (§16.11.3) ===
    // We need to clone scratch arrays to avoid borrow conflicts
    let rownnz: Vec<usize> = data.island_scratch_rownnz[..ntree].to_vec();
    let rowadr: Vec<usize> = data.island_scratch_rowadr[..ntree].to_vec();
    let colind: Vec<usize> = data.island_scratch_colind.clone();

    let mut island_out = vec![-1i32; ntree];
    let mut stack = vec![0usize; ntree];

    let nisland = mj_flood_fill(
        &mut island_out,
        &rownnz,
        &rowadr,
        &colind,
        &mut stack,
        ntree,
    );

    // Copy results back
    data.tree_island[..ntree].copy_from_slice(&island_out[..ntree]);
    data.nisland = nisland;

    // === Phase 4: Populate island arrays (§16.11.4) ===

    // 4a: Trees per island
    data.island_ntree[..nisland].fill(0);
    for t in 0..ntree {
        if island_out[t] >= 0 {
            let isl = island_out[t] as usize;
            if isl < nisland {
                data.island_ntree[isl] += 1;
            }
        }
    }

    // Compute island_itreeadr (prefix sum of island_ntree)
    if nisland > 0 {
        let mut prefix = 0;
        for i in 0..nisland {
            data.island_itreeadr[i] = prefix;
            prefix += data.island_ntree[i];
        }
    }

    // Pack map_itree2tree grouped by island
    let mut island_fill = vec![0usize; nisland];
    for t in 0..ntree {
        if island_out[t] >= 0 {
            let isl = island_out[t] as usize;
            if isl < nisland {
                let idx = data.island_itreeadr[isl] + island_fill[isl];
                if idx < data.map_itree2tree.len() {
                    data.map_itree2tree[idx] = t;
                }
                island_fill[isl] += 1;
            }
        }
    }

    // 4b: DOFs per island
    data.island_nv[..nisland].fill(0);
    data.dof_island[..model.nv].fill(-1);
    data.map_dof2idof[..model.nv].fill(-1);

    // First pass: compute island_nv and island_idofadr
    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < ntree && island_out[tree] >= 0 {
                let isl = island_out[tree] as usize;
                if isl < nisland {
                    data.dof_island[dof] = isl as i32;
                    data.island_nv[isl] += 1;
                }
            }
        }
    }

    // Compute island_idofadr (prefix sum of island_nv)
    if nisland > 0 {
        let mut prefix = 0;
        for i in 0..nisland {
            data.island_idofadr[i] = prefix;
            prefix += data.island_nv[i];
        }
    }

    // Pack map_idof2dof and build map_dof2idof
    let mut island_dof_fill = vec![0usize; nisland];
    for dof in 0..model.nv {
        let isl_i32 = data.dof_island[dof];
        if isl_i32 >= 0 {
            let isl = isl_i32 as usize;
            if isl < nisland {
                let local_idx = island_dof_fill[isl];
                let global_idx = data.island_idofadr[isl] + local_idx;
                if global_idx < data.map_idof2dof.len() {
                    data.map_idof2dof[global_idx] = dof;
                }
                data.map_dof2idof[dof] = local_idx as i32;
                island_dof_fill[isl] += 1;
            }
        }
    }

    // 4c: Contacts per island (§16.16)
    // Assign each contact to an island based on its bodies' trees.
    // For contacts with two dynamic bodies, use the first body's tree
    // (both trees are in the same island by construction of the edge graph).
    let ncon = data.contacts.len();
    data.contact_island.resize(ncon, -1);

    for ci in 0..ncon {
        let contact = &data.contacts[ci];
        let body1 = if contact.geom1 < model.geom_body.len() {
            model.geom_body[contact.geom1]
        } else {
            continue;
        };
        let body2 = if contact.geom2 < model.geom_body.len() {
            model.geom_body[contact.geom2]
        } else {
            continue;
        };
        // Pick the first dynamic body's tree for island assignment
        let tree = if body1 > 0 && body1 < model.body_treeid.len() {
            model.body_treeid[body1]
        } else if body2 > 0 && body2 < model.body_treeid.len() {
            model.body_treeid[body2]
        } else {
            continue; // Both world — shouldn't happen but skip
        };
        if tree < ntree && island_out[tree] >= 0 {
            data.contact_island[ci] = island_out[tree];
        }
    }

    // efc_island population is now deferred to populate_efc_island(),
    // called from mj_fwd_constraint() after assemble_unified_constraints().
    // This ensures efc_island is based on the CURRENT step's constraint rows.
}

/// Get the tree pair spanned by an equality constraint (§16.11.2).
///
/// Returns `(tree1, tree2)` where `tree1` and `tree2` may be equal
/// for single-tree constraints. Returns `(usize::MAX, usize::MAX)`
/// if the trees cannot be determined.
pub(super) fn equality_trees(model: &Model, eq_id: usize) -> (usize, usize) {
    let sentinel = usize::MAX;
    match model.eq_type[eq_id] {
        EqualityType::Connect | EqualityType::Weld => {
            // obj1/obj2 are body IDs
            let b1 = model.eq_obj1id[eq_id];
            let b2 = model.eq_obj2id[eq_id];
            let t1 = if b1 > 0 && b1 < model.body_treeid.len() {
                model.body_treeid[b1]
            } else {
                sentinel
            };
            let t2 = if b2 > 0 && b2 < model.body_treeid.len() {
                model.body_treeid[b2]
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Joint => {
            // obj1/obj2 are joint IDs → jnt_body → body_treeid
            let j1 = model.eq_obj1id[eq_id];
            let j2 = model.eq_obj2id[eq_id];
            let t1 = if j1 < model.jnt_body.len() {
                let b = model.jnt_body[j1];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            let t2 = if j2 < model.jnt_body.len() {
                let b = model.jnt_body[j2];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Distance => {
            // obj1/obj2 are geom IDs → geom_body → body_treeid
            let g1 = model.eq_obj1id[eq_id];
            let g2 = model.eq_obj2id[eq_id];
            let t1 = if g1 < model.geom_body.len() {
                let b = model.geom_body[g1];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            let t2 = if g2 < model.geom_body.len() {
                let b = model.geom_body[g2];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Tendon => {
            let t1_id = model.eq_obj1id[eq_id];
            // Primary tree for tendon 1 (sentinel if treenum == 0, i.e. static)
            let tree1 = if model.tendon_treenum[t1_id] >= 1 {
                model.tendon_tree[2 * t1_id]
            } else {
                sentinel
            };

            if model.eq_obj2id[eq_id] == usize::MAX {
                // Single-tendon: if it spans two trees, return both
                if model.tendon_treenum[t1_id] == 2 {
                    (
                        model.tendon_tree[2 * t1_id],
                        model.tendon_tree[2 * t1_id + 1],
                    )
                } else {
                    (tree1, tree1)
                }
            } else {
                // Two-tendon coupling: primary tree from each tendon
                let t2_id = model.eq_obj2id[eq_id];
                let tree2 = if model.tendon_treenum[t2_id] >= 1 {
                    model.tendon_tree[2 * t2_id]
                } else {
                    sentinel
                };
                (tree1, tree2)
            }
        }
    }
}

/// Determine the primary tree for a constraint row (§16.11.2).
///
/// Used for assigning constraint rows to islands. Returns `usize::MAX`
/// if the tree cannot be determined.
#[allow(clippy::too_many_lines)]
fn constraint_tree(model: &Model, data: &Data, row: usize) -> usize {
    let sentinel = usize::MAX;
    let ctype = data.efc_type[row];
    let id = data.efc_id[row];
    let ntree = model.ntree;

    match ctype {
        ConstraintType::ContactFrictionless
        | ConstraintType::ContactPyramidal
        | ConstraintType::ContactElliptic => {
            if id >= data.contacts.len() {
                return sentinel;
            }
            let contact = &data.contacts[id];
            let body1 = if contact.geom1 < model.geom_body.len() {
                model.geom_body[contact.geom1]
            } else {
                return sentinel;
            };
            // Use body1's tree as the primary tree for this contact
            if body1 > 0 && body1 < model.body_treeid.len() {
                let tree = model.body_treeid[body1];
                if tree < ntree {
                    return tree;
                }
            }
            // Fallback: try body2
            let body2 = if contact.geom2 < model.geom_body.len() {
                model.geom_body[contact.geom2]
            } else {
                return sentinel;
            };
            if body2 > 0 && body2 < model.body_treeid.len() {
                let tree = model.body_treeid[body2];
                if tree < ntree {
                    return tree;
                }
            }
            sentinel
        }
        ConstraintType::Equality => {
            if id < model.neq {
                let (t1, t2) = equality_trees(model, id);
                if t1 < ntree {
                    t1
                } else if t2 < ntree {
                    t2
                } else {
                    sentinel
                }
            } else {
                sentinel
            }
        }
        ConstraintType::LimitJoint => {
            if id < model.jnt_body.len() {
                let body = model.jnt_body[id];
                if body > 0 && body < model.body_treeid.len() {
                    let tree = model.body_treeid[body];
                    if tree < ntree {
                        return tree;
                    }
                }
            }
            sentinel
        }
        ConstraintType::LimitTendon => {
            // Use precomputed tendon tree mapping
            if id < model.ntendon && model.tendon_treenum[id] >= 1 {
                if model.tendon_treenum[id] == 2 {
                    let t = model.tendon_tree[2 * id];
                    if t < ntree {
                        return t;
                    }
                }
                // Single tree: scan Jacobian
                for dof in 0..model.nv {
                    if data.efc_J[(row, dof)].abs() > 0.0 && dof < model.dof_treeid.len() {
                        let tree = model.dof_treeid[dof];
                        if tree < ntree {
                            return tree;
                        }
                    }
                }
            }
            sentinel
        }
        ConstraintType::FrictionLoss => {
            if id < model.dof_treeid.len() {
                let tree = model.dof_treeid[id];
                if tree < ntree {
                    return tree;
                }
            }
            sentinel
        }
        ConstraintType::FlexEdge => {
            // Flex constraints: scan Jacobian for nonzero DOFs and find their tree
            for dof in 0..model.nv {
                if data.efc_J[(row, dof)].abs() > 0.0 && dof < model.dof_treeid.len() {
                    let tree = model.dof_treeid[dof];
                    if tree < ntree {
                        return tree;
                    }
                }
            }
            sentinel
        }
    }
}

/// Flood-fill connected components on a tree-tree adjacency graph (§16.11.3).
///
/// Uses DFS with an explicit stack. Trees with no edges (`rownnz[t] == 0`) get
/// `island[t] = -1` (singletons). Returns the number of islands found.
#[allow(
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::needless_continue
)]
fn mj_flood_fill(
    island: &mut [i32],
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
    stack: &mut [usize],
    ntree: usize,
) -> usize {
    island[..ntree].fill(-1);
    let mut nisland = 0usize;

    for seed in 0..ntree {
        if island[seed] >= 0 || rownnz[seed] == 0 {
            continue;
        }

        // DFS from seed using explicit stack with size counter
        let mut stack_len = 1;
        stack[0] = seed;
        island[seed] = nisland as i32;

        while stack_len > 0 {
            stack_len -= 1;
            let node = stack[stack_len];

            for j in rowadr[node]..rowadr[node] + rownnz[node] {
                let neighbor = colind[j];
                if island[neighbor] < 0 {
                    island[neighbor] = nisland as i32;
                    stack[stack_len] = neighbor;
                    stack_len += 1;
                }
            }
        }

        nisland += 1;
    }

    nisland
}

/// Populate `efc_island` and related arrays from the current step's constraint rows.
///
/// Called from `mj_fwd_constraint()` AFTER `assemble_unified_constraints()` so
/// that the efc_island data reflects the current step's constraints (not stale data from
/// a previous step). Previously this was done inside `mj_island()` but that ran before
/// constraint assembly, causing stale/incorrect efc_island arrays.
pub(crate) fn populate_efc_island(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    let nisland = data.nisland;
    let ntree = model.ntree;

    if nefc == 0 || nisland == 0 {
        data.efc_island.clear();
        data.map_efc2iefc.clear();
        data.map_iefc2efc.clear();
        if nisland > 0 {
            data.island_nefc[..nisland].fill(0);
            data.island_iefcadr[..nisland].fill(0);
        }
        return;
    }

    data.efc_island.resize(nefc, -1);
    data.efc_island.fill(-1);
    data.map_efc2iefc.resize(nefc, -1);
    data.map_efc2iefc.fill(-1);
    data.map_iefc2efc.resize(nefc, 0);
    data.island_nefc[..nisland].fill(0);

    for r in 0..nefc {
        let tree = constraint_tree(model, data, r);
        if tree < ntree && data.tree_island[tree] >= 0 {
            let isl = usize::try_from(data.tree_island[tree]).unwrap_or(0);
            if isl < nisland {
                data.efc_island[r] = i32::try_from(isl).unwrap_or(0);
                data.island_nefc[isl] += 1;
            }
        }
    }

    // Compute prefix sums for island_iefcadr
    let mut prefix = 0;
    for i in 0..nisland {
        data.island_iefcadr[i] = prefix;
        prefix += data.island_nefc[i];
    }

    // Build bidirectional maps
    let mut island_efc_fill = vec![0usize; nisland];
    for r in 0..nefc {
        let isl_i32 = data.efc_island[r];
        if isl_i32 >= 0 {
            let isl = usize::try_from(isl_i32).unwrap_or(0);
            if isl < nisland {
                let local_idx = island_efc_fill[isl];
                let global_idx = data.island_iefcadr[isl] + local_idx;
                data.map_efc2iefc[r] = i32::try_from(local_idx).unwrap_or(0);
                if global_idx < data.map_iefc2efc.len() {
                    data.map_iefc2efc[global_idx] = r;
                }
                island_efc_fill[isl] += 1;
            }
        }
    }
}
