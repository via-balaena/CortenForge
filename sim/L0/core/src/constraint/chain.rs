//! A constraint row's dof chain at the sparse size ([`Model::is_sparse`]):
//! the dofs MuJoCo's sparse Jacobian lists for the row (`efc_J_colind`),
//! ascending, whatever their values, as each row kind passes them to
//! `mj_addConstraint` (`engine_core_constraint.c:237-329`).

use crate::types::{Data, Model};

/// Bodies `b1` and `b2`'s merged dof chain, as `mj_jacDifPair` forms it
/// (`engine_core_util.c:429-448`): every dof from each body's weld body up
/// to the world, merged (`mj_mergeChain`). Two bodies welded to the world
/// have none. For two different simple bodies MuJoCo takes
/// `mj_mergeChainSimple`, which lists the same dofs (a simple body's parent
/// has none); for one body with itself (a `<pair>` of two of its geoms) it
/// lists each dof twice and stops at the sparse size, where this lists each
/// once (ledger L222).
pub fn body_pair_chain(model: &Model, b1: usize, b2: usize) -> Vec<usize> {
    let last = |body: usize| {
        let weld = model.body_weldid[body];
        (weld != 0).then(|| model.body_dof_adr[weld] + model.body_dof_num[weld] - 1)
    };
    // Each body's chain from its last dof down to the root, the higher dof
    // first; a dof both share is listed once.
    let (mut d1, mut d2) = (last(b1), last(b2));
    let mut chain = Vec::new();
    while let Some(next) = d1.max(d2) {
        chain.push(next);
        if d1 == Some(next) {
            d1 = model.dof_parent[next];
        }
        if d2 == Some(next) {
            d2 = model.dof_parent[next];
        }
    }
    chain.reverse();
    chain
}

/// `chain()` at the sparse size, where a row records its chain; nothing at
/// the dense size, where no row does.
pub fn at_sparse_size(model: &Model, chain: impl FnOnce() -> Vec<usize>) -> Vec<usize> {
    if model.is_sparse() {
        chain()
    } else {
        Vec::new()
    }
}

/// Tendon `t`'s dof chain (`ten_J_colind`, ascending).
pub fn tendon_chain(data: &Data, nv: usize, t: usize) -> Vec<usize> {
    data.ten_J_colind[t * nv..t * nv + data.ten_J_rownnz[t]].to_vec()
}

/// The union of two chains, ascending, each dof once (the pattern
/// `mju_combineSparse` leaves, `engine_util_sparse.h:258-331`).
pub fn union_chains(first: &[usize], second: &[usize]) -> Vec<usize> {
    let mut chain = [first, second].concat();
    chain.sort_unstable();
    chain.dedup();
    chain
}
