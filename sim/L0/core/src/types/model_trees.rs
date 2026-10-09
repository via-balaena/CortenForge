//! Kinematic-tree tables (§16.0), computed from the body tree.
//!
//! Moved from sim-mjcf's builder so that every `Model` producer (the MJCF
//! builder, the factories, the test fixtures, a hand-built model) gets them.
//! MuJoCo counts the trees when it compiles (`user_model.cc:3006-3013`) and
//! computes the rest of these tables in `setFixed`, part of `mj_setConst`
//! (3.5.0 `engine_setconst.c:106-276`).

use super::enums::{ActuatorTransmission, SleepPolicy, WrapType};
use super::model::Model;

impl Model {
    /// Compute the kinematic-tree tables and resolve automatic sleep policies.
    ///
    /// A tree starts at every dof with no parent dof: a moving body whose
    /// ancestors are all static. A body belongs to the tree of the body it is
    /// welded to, so a static body, welded to the world, belongs to none
    /// (`body_treeid` is `usize::MAX`), and every tree has a dof.
    ///
    /// Writes `ntree`, `tree_body_adr`, `tree_body_num`, `tree_dof_adr`,
    /// `tree_dof_num`, `body_treeid`, `dof_treeid`, `tendon_treenum`, `tendon_tree`
    /// and the automatic entries of `tree_sleep_policy`. Reads `dof_parent`,
    /// `body_weldid` (so it runs after [`Self::compute_ancestors`]),
    /// `body_dof_adr`, `body_dof_num`, the actuator transmissions, the tendon
    /// wraps and the flex vertex bodies. An explicit policy (`Never`, `Allowed`,
    /// `Init`) survives a recompute when `ntree` is unchanged.
    pub fn compute_kinematic_trees(&mut self) {
        let mut ntree = 0;
        self.dof_treeid = vec![0; self.nv];
        for dof in 0..self.nv {
            if self.dof_parent[dof].is_none() {
                ntree += 1;
            }
            self.dof_treeid[dof] = ntree - 1;
        }
        self.body_treeid = vec![usize::MAX; self.nbody];
        for body_id in 1..self.nbody {
            let weld = self.body_weldid[body_id];
            if self.body_dof_num[weld] > 0 {
                self.body_treeid[body_id] = self.dof_treeid[self.body_dof_adr[weld]];
            }
        }
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
        self.tree_sleep_policy = vec![SleepPolicy::Auto; ntree];
        // The bodies of a tree, and its dofs, are contiguous and in tree order.
        self.tree_body_adr = vec![0; ntree];
        self.tree_body_num = vec![0; ntree];
        for body_id in 1..self.nbody {
            let tree = self.body_treeid[body_id];
            if tree < ntree {
                if self.tree_body_num[tree] == 0 {
                    self.tree_body_adr[tree] = body_id;
                }
                self.tree_body_num[tree] += 1;
            }
        }
        self.tree_dof_adr = vec![0; ntree];
        self.tree_dof_num = vec![0; ntree];
        for dof in 0..self.nv {
            let tree = self.dof_treeid[dof];
            if self.tree_dof_num[tree] == 0 {
                self.tree_dof_adr[tree] = dof;
            }
            self.tree_dof_num[tree] += 1;
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

    /// The trees tendon `tendon`'s wraps reach, each once, in the order of
    /// its wraps (MuJoCo's `GetWrapBodyTreeId` over the wraps). A pulley
    /// reaches none, nor does a wrap on a static body.
    pub fn tendon_trees(&self, tendon: usize) -> impl Iterator<Item = usize> + '_ {
        let mut seen = vec![false; self.ntree];
        (self.tendon_adr[tendon]..self.tendon_adr[tendon] + self.tendon_num[tendon])
            .filter_map(|w| self.wrap_body(w).and_then(|b| self.tree_of_body(b)))
            .filter(move |&tree| !std::mem::replace(&mut seen[tree], true))
    }

    /// `tendon_treenum` and the first two of [`Self::tendon_trees`] in
    /// `tendon_tree`, as MuJoCo keeps them.
    fn compute_tendon_trees(&mut self) {
        self.tendon_treenum = vec![0; self.ntendon];
        self.tendon_tree = vec![usize::MAX; 2 * self.ntendon];
        for t in 0..self.ntendon {
            let trees: Vec<usize> = self.tendon_trees(t).collect();
            self.tendon_treenum[t] = trees.len();
            for (slot, &tree) in trees.iter().take(2).enumerate() {
                self.tendon_tree[2 * t + slot] = tree;
            }
        }
    }

    /// The automatic policies (`engine_setconst.c:163-276`): a tree that holds
    /// an actuator's joint, site (a slider-crank's crank site) or body, or a
    /// wrap of an actuated tendon; every tree of a tendon that reaches more than
    /// two trees, or two with stiffness or damping; and a tree that holds a
    /// flex vertex body are `AutoNever`. Every other `Auto` tree is
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
                ActuatorTransmission::Site | ActuatorTransmission::SliderCrank => (trnid[0]
                    < self.nsite)
                    .then(|| self.site_body[trnid[0]])
                    .into_iter()
                    .collect(),
                ActuatorTransmission::Body => vec![trnid[0]],
                ActuatorTransmission::Tendon => vec![],
            };
            for bid in bodies {
                if let Some(tree) = self.tree_of_body(bid) {
                    never[tree] = true;
                }
            }
        }
        for t in 0..self.ntendon {
            let treenum = self.tendon_treenum[t];
            let passive = self.tendon_stiffness[t] != 0.0 || self.tendon_damping[t] != 0.0;
            if treenum > 2 || (treenum == 2 && passive) {
                for tree in self.tendon_trees(t) {
                    never[tree] = true;
                }
            }
        }
        for &body_id in &self.flexvert_bodyid {
            if let Some(tree) = self.tree_of_body(body_id) {
                never[tree] = true;
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
