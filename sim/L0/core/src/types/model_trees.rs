//! Kinematic-tree tables (§16.0), computed from the body tree.
//!
//! Moved from sim-mjcf's builder so that every `Model` producer (the MJCF
//! builder, the factories, the test fixtures, a hand-built model) gets them.
//! MuJoCo computes these tables in `setFixed`, part of `mj_setConst`.

use std::collections::{BTreeMap, BTreeSet};

use super::enums::{ActuatorTransmission, SleepPolicy, WrapType};
use super::model::Model;

impl Model {
    /// Compute the kinematic-tree tables and resolve automatic sleep policies.
    ///
    /// Writes `ntree`, `tree_body_adr`, `tree_body_num`, `tree_dof_adr`,
    /// `tree_dof_num`, `body_treeid`, `dof_treeid`, `tendon_treenum`, `tendon_tree`
    /// and the automatic entries of `tree_sleep_policy`. Reads `body_rootid`,
    /// `body_dof_adr`, `body_dof_num`, the actuator transmissions and the tendon
    /// wraps. An explicit policy (`Never`, `Allowed`, `Init`) survives a recompute
    /// when `ntree` is unchanged.
    pub fn compute_kinematic_trees(&mut self) {
        let mut trees: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for body_id in 1..self.nbody {
            trees
                .entry(self.body_rootid[body_id])
                .or_default()
                .push(body_id);
        }
        let ntree = trees.len();
        let explicit: Vec<Option<SleepPolicy>> = if self.tree_sleep_policy.len() == ntree {
            self.tree_sleep_policy
                .iter()
                .map(|p| match p {
                    SleepPolicy::Never | SleepPolicy::Allowed | SleepPolicy::Init => Some(*p),
                    _ => None,
                })
                .collect()
        } else {
            vec![None; ntree]
        };
        self.ntree = ntree;
        self.tree_body_adr = Vec::with_capacity(ntree);
        self.tree_body_num = Vec::with_capacity(ntree);
        self.tree_dof_adr = Vec::with_capacity(ntree);
        self.tree_dof_num = Vec::with_capacity(ntree);
        self.tree_sleep_policy = vec![SleepPolicy::Auto; ntree];
        self.body_treeid = vec![usize::MAX; self.nbody];
        self.dof_treeid = vec![0; self.nv];
        for (tree_idx, (_root, body_ids)) in trees.iter().enumerate() {
            self.tree_body_adr.push(body_ids[0]);
            self.tree_body_num.push(body_ids.len());
            let mut min_dof = self.nv;
            let mut total_dofs = 0usize;
            for &bid in body_ids {
                self.body_treeid[bid] = tree_idx;
                let (start, count) = (self.body_dof_adr[bid], self.body_dof_num[bid]);
                if count > 0 && start < min_dof {
                    min_dof = start;
                }
                total_dofs += count;
                for dof in start..start + count {
                    self.dof_treeid[dof] = tree_idx;
                }
            }
            self.tree_dof_adr
                .push(if total_dofs == 0 { 0 } else { min_dof });
            self.tree_dof_num.push(total_dofs);
        }
        self.compute_tendon_trees();
        self.resolve_auto_sleep_policies();
        for (policy, kept) in self.tree_sleep_policy.iter_mut().zip(explicit) {
            if let Some(kept) = kept {
                *policy = kept;
            }
        }
    }

    fn wrap_body(&self, w: usize) -> Option<usize> {
        let id = self.wrap_objid[w];
        match self.wrap_type[w] {
            WrapType::Joint => (id < self.nv).then(|| self.dof_body[id]),
            WrapType::Site => (id < self.nsite).then(|| self.site_body[id]),
            WrapType::Geom => (id < self.ngeom).then(|| self.geom_body[id]),
            WrapType::Pulley => None,
        }
    }

    fn tree_of_body(&self, bid: usize) -> Option<usize> {
        let tree = *self.body_treeid.get(bid)?;
        (bid > 0 && tree < self.ntree).then_some(tree)
    }

    fn compute_tendon_trees(&mut self) {
        self.tendon_treenum = vec![0; self.ntendon];
        self.tendon_tree = vec![usize::MAX; 2 * self.ntendon];
        for t in 0..self.ntendon {
            let mut set = BTreeSet::new();
            for w in self.tendon_adr[t]..self.tendon_adr[t] + self.tendon_num[t] {
                if let Some(tree) = self.wrap_body(w).and_then(|b| self.tree_of_body(b)) {
                    set.insert(tree);
                }
            }
            self.tendon_treenum[t] = set.len();
            let mut it = set.iter();
            if let Some(&a) = it.next() {
                self.tendon_tree[2 * t] = a;
            }
            if let Some(&b) = it.next() {
                self.tendon_tree[2 * t + 1] = b;
            }
        }
    }

    /// The automatic policies: a tree an actuator acts on, or that a passive
    /// multi-tree tendon couples, is `AutoNever`; every other `Auto` tree is
    /// `AutoAllowed`.
    fn resolve_auto_sleep_policies(&mut self) {
        let mut never = vec![false; self.ntree];
        for act in 0..self.nu {
            let trnid = self.actuator_trnid[act];
            let bodies: Vec<usize> = match self.actuator_trntype[act] {
                ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => (trnid[0]
                    < self.njnt)
                    .then(|| self.jnt_body[trnid[0]])
                    .into_iter()
                    .collect(),
                ActuatorTransmission::Tendon if trnid[0] < self.ntendon => {
                    let t = trnid[0];
                    (self.tendon_adr[t]..self.tendon_adr[t] + self.tendon_num[t])
                        .filter_map(|w| self.wrap_body(w))
                        .collect()
                }
                ActuatorTransmission::Site => (trnid[0] < self.nsite)
                    .then(|| self.site_body[trnid[0]])
                    .into_iter()
                    .collect(),
                ActuatorTransmission::Body => vec![trnid[0]],
                ActuatorTransmission::SliderCrank => [trnid[0], trnid[1]]
                    .into_iter()
                    .filter(|&s| s < self.nsite)
                    .map(|s| self.site_body[s])
                    .collect(),
                ActuatorTransmission::Tendon => vec![],
            };
            for bid in bodies {
                if let Some(tree) = self.tree_of_body(bid) {
                    never[tree] = true;
                }
            }
        }
        for t in 0..self.ntendon {
            if self.tendon_treenum[t] < 2 {
                continue;
            }
            if self.tendon_stiffness[t].abs() > 0.0
                || self.tendon_damping[t].abs() > 0.0
                || self.tendon_limited[t]
            {
                for k in [self.tendon_tree[2 * t], self.tendon_tree[2 * t + 1]] {
                    if k < self.ntree {
                        never[k] = true;
                    }
                }
            }
        }
        for (policy, never) in self.tree_sleep_policy.iter_mut().zip(never) {
            if *policy == SleepPolicy::Auto {
                *policy = if never {
                    SleepPolicy::AutoNever
                } else {
                    SleepPolicy::AutoAllowed
                };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    /// The test fixtures' `finalize` derives the kinematic trees and
    /// `dof_length`, as the factories do.
    #[test]
    fn finalize_derives_trees_and_dof_length() {
        let model = crate::test_fixtures::hinge_chain(3);
        assert_eq!(model.ntree, 1);
        assert_eq!(model.dof_treeid, vec![0; 3]);
        assert_eq!(model.dof_length.len(), 3);
    }
}
