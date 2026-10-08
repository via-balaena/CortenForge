//! Island discovery — constraint-graph connected components (§16.11).
//!
//! Builds a tree-tree adjacency graph from the constraint rows, then runs a
//! flood fill to partition the trees into constraint islands. Corresponds to
//! MuJoCo's `engine_island.c`.

#![allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]

pub(crate) mod sleep;

use crate::types::{ConstraintType, DISABLE_ISLAND, Data, EqualityType, Model};

// Re-exports — island functions for external consumers.
pub(crate) use sleep::{
    constraint_sleep_filter, dof_asleep, equality_asleep, mj_check_qpos_changed, mj_sleep,
    mj_update_sleep_arrays, mj_wake, mj_wake_collision, mj_wake_equality, mj_wake_tendon,
    reset_sleep_state,
};

/// Discover the constraint islands from the constraint rows, as MuJoCo's
/// `mj_island` (3.5.0 `engine_island.c:374-606`): each constraint joins the
/// trees its rows reach, a flood fill over those edges makes the islands, and
/// the trees, dofs and rows of each island are listed in island order, the
/// unconstrained ones after them. No islands without rows, or with
/// `DISABLE_ISLAND`, or for a model without tree tables (a hand-built one
/// that never ran [`Model::compute_kinematic_trees`]); with sleep enabled or
/// not.
///
/// Runs after constraint assembly. MuJoCo makes the rows, then the islands,
/// in the position stage; this crate makes the rows in the acceleration
/// stage, from the same positions, so the partition is the same. The
/// block-diagonal copies of the mass matrix and the Jacobian MuJoCo's island
/// solver takes are not made: the solver here is global.
// Tree, dof and row indices are usize stored as i32 in mjData's arrays, bounded by realistic model sizes.
#[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss)]
pub(crate) fn mj_island(model: &Model, data: &mut Data) {
    let (ntree, nv) = (model.ntree, model.nv);
    let nefc = data.efc_type.len();
    data.nisland = 0;
    data.tree_island[..ntree].fill(-1);
    data.dof_island[..nv].fill(-1);
    data.efc_island.clear();
    data.map_efc2iefc.clear();
    data.map_iefc2efc.clear();
    data.contact_island.clear();
    data.contact_island.resize(data.contacts.len(), -1);
    if model.disableflags & DISABLE_ISLAND != 0 || nefc == 0 || ntree == 0 {
        return;
    }

    // Edges (`findEdges`): one constraint per run of rows with the same type
    // and id. (MuJoCo reads each row of a flex equality, whose rows share an
    // id; a flex edge row here has its own.)
    let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); ntree];
    let mut add_edge = |a: Option<usize>, b: Option<usize>| match (a, b) {
        (Some(a), Some(b)) => {
            adjacent[a].push(b);
            if a != b {
                adjacent[b].push(a);
            }
        }
        // A static object makes a self-edge.
        (Some(t), None) | (None, Some(t)) => adjacent[t].push(t),
        (None, None) => {}
    };
    let mut previous = None;
    for row in 0..nefc {
        let key = (data.efc_type[row], data.efc_id[row]);
        if previous == Some(key) {
            continue;
        }
        previous = Some(key);
        if let Some((a, b)) = row_tree_pair(model, data, row) {
            add_edge(a, b);
        } else {
            let trees = row_trees(model, data, row);
            if let [only] = trees[..] {
                add_edge(Some(only), None);
            }
            for pair in trees.windows(2) {
                add_edge(Some(pair[0]), Some(pair[1]));
            }
        }
    }

    // Flood fill (`mj_floodFill`): islands numbered by their lowest tree; a
    // tree with no edge is in none.
    let mut stack = Vec::new();
    let mut nisland = 0;
    for seed in 0..ntree {
        if data.tree_island[seed] >= 0 || adjacent[seed].is_empty() {
            continue;
        }
        stack.push(seed);
        while let Some(tree) = stack.pop() {
            if data.tree_island[tree] >= 0 {
                continue;
            }
            data.tree_island[tree] = nisland as i32;
            stack.extend_from_slice(&adjacent[tree]);
        }
        nisland += 1;
    }
    data.nisland = nisland;
    if nisland == 0 {
        return;
    }
    let island_of = |i: i32| usize::try_from(i).ok();

    // Trees: per island, then the unconstrained ones.
    data.island_ntree[..nisland].fill(0);
    for tree in 0..ntree {
        if let Some(isl) = island_of(data.tree_island[tree]) {
            data.island_ntree[isl] += 1;
        }
    }
    let mut next = prefix_sum(&data.island_ntree[..nisland], &mut data.island_itreeadr);
    let mut unconstrained = data.island_ntree[..nisland].iter().sum::<usize>();
    for tree in 0..ntree {
        let slot = match island_of(data.tree_island[tree]) {
            Some(isl) => &mut next[isl],
            None => &mut unconstrained,
        };
        data.map_itree2tree[*slot] = tree;
        *slot += 1;
    }

    // Dofs: per island, then the unconstrained ones.
    data.island_nv[..nisland].fill(0);
    for dof in 0..nv {
        let island = data.tree_island[model.dof_treeid[dof]];
        data.dof_island[dof] = island;
        if let Some(isl) = island_of(island) {
            data.island_nv[isl] += 1;
        }
    }
    let mut next = prefix_sum(&data.island_nv[..nisland], &mut data.island_idofadr);
    let mut unconstrained = data.island_nv[..nisland].iter().sum::<usize>();
    for dof in 0..nv {
        let slot = match island_of(data.dof_island[dof]) {
            Some(isl) => &mut next[isl],
            None => &mut unconstrained,
        };
        data.map_dof2idof[dof] = *slot as i32;
        data.map_idof2dof[*slot] = dof;
        *slot += 1;
    }

    // Rows: each in the island of its first tree.
    data.efc_island.resize(nefc, -1);
    data.island_nefc[..nisland].fill(0);
    for row in 0..nefc {
        let first = match row_tree_pair(model, data, row) {
            Some((a, b)) => a.or(b),
            None => row_trees(model, data, row).first().copied(),
        };
        if let Some(isl) = first.and_then(|t| island_of(data.tree_island[t])) {
            data.efc_island[row] = isl as i32;
            data.island_nefc[isl] += 1;
        }
    }
    let mut next = prefix_sum(&data.island_nefc[..nisland], &mut data.island_iefcadr);
    data.map_efc2iefc.resize(nefc, -1);
    data.map_iefc2efc.resize(nefc, 0);
    for row in 0..nefc {
        if let Some(isl) = island_of(data.efc_island[row]) {
            data.map_efc2iefc[row] = next[isl] as i32;
            data.map_iefc2efc[next[isl]] = row;
            next[isl] += 1;
        }
    }

    // Contacts (this crate's readout; MuJoCo has none): the island of the
    // first of its bodies that is in a tree (a static body is in none).
    for (c, contact) in data.contacts.iter().enumerate() {
        let tree_of = |geom: usize| {
            let body = *model.geom_body.get(geom)?;
            model.body_treeid.get(body).copied().filter(|&t| t < ntree)
        };
        if let Some(tree) = tree_of(contact.geom1).or_else(|| tree_of(contact.geom2)) {
            data.contact_island[c] = data.tree_island[tree];
        }
    }
}

/// Write the exclusive prefix sums of `counts` into `adr` and return a copy,
/// the next free slot of each.
fn prefix_sum(counts: &[usize], adr: &mut [usize]) -> Vec<usize> {
    let mut total = 0;
    for (a, &n) in adr.iter_mut().zip(counts) {
        *a = total;
        total += n;
    }
    adr[..counts.len()].to_vec()
}

/// The trees of row `row` where MuJoCo reads them from the constraint's
/// objects (`treeFirst`'s special cases): a joint limit's tree, a geom
/// contact's two and a connect's or weld's two, a static body's as `None`.
/// `None` for every other row, whose trees come from its Jacobian. A dof
/// friction row, which MuJoCo also special-cases, has one nonzero column, so
/// its Jacobian gives the same tree.
fn row_tree_pair(model: &Model, data: &Data, row: usize) -> Option<(Option<usize>, Option<usize>)> {
    let id = data.efc_id[row];
    let tree_of_body = |body: usize| Some(model.body_treeid[body]).filter(|&t| t < model.ntree);
    match data.efc_type[row] {
        ConstraintType::LimitJoint => Some((Some(model.dof_treeid[model.jnt_dof_adr[id]]), None)),
        ConstraintType::ContactFrictionless
        | ConstraintType::ContactPyramidal
        | ConstraintType::ContactElliptic => {
            let contact = &data.contacts[id];
            if contact.flex_vertex.is_some() || contact.flex_vertex2.is_some() {
                return None;
            }
            let b1 = model.geom_body[contact.geom1];
            let b2 = model.geom_body[contact.geom2];
            Some((tree_of_body(b1), tree_of_body(b2)))
        }
        ConstraintType::Equality
            if matches!(
                model.eq_type[id],
                EqualityType::Connect | EqualityType::Weld
            ) =>
        {
            let b1 = model.eq_obj1id[id];
            let b2 = model.eq_obj2id[id];
            Some((tree_of_body(b1), tree_of_body(b2)))
        }
        _ => None,
    }
}

/// The distinct trees of row `row`'s nonzero Jacobian columns, in column
/// order (MuJoCo's `treeNext` scan). Dofs are numbered tree by tree, so each
/// tree appears once.
fn row_trees(model: &Model, data: &Data, row: usize) -> Vec<usize> {
    let mut trees: Vec<usize> = Vec::new();
    for dof in 0..model.nv {
        if data.efc_J[(row, dof)] != 0.0 {
            let tree = model.dof_treeid[dof];
            if trees.last() != Some(&tree) {
                trees.push(tree);
            }
        }
    }
    trees
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
