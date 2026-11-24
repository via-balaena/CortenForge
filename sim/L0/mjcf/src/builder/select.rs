//! Every array of the builder's bodies, geoms or sites taken again in a new
//! order, each entry from its old index: what `discardvisual` keeps
//! ([`super::discard`]) and how `fusestatic` numbers the fused tree
//! ([`super::fuse`]). The name maps are made again from the names.

use std::collections::HashMap;

use super::ModelBuilder;

/// `v`'s entries at `order`, in that order.
fn select<T: Clone>(v: &mut Vec<T>, order: &[usize]) {
    *v = order.iter().map(|&i| v[i].clone()).collect();
}

/// Each non-empty name of `names` to its index.
fn name_map(names: &[Option<String>]) -> HashMap<String, usize> {
    names
        .iter()
        .enumerate()
        .filter_map(|(i, name)| Some((name.clone().filter(|n| !n.is_empty())?, i)))
        .collect()
}

impl ModelBuilder {
    /// Every geom array at `order`, the geom name map made again. Each
    /// geom's body and each body's geom range are the caller's.
    pub(crate) fn select_geoms(&mut self, order: &[usize]) {
        select(&mut self.geom_type, order);
        select(&mut self.geom_body, order);
        select(&mut self.geom_pos, order);
        select(&mut self.geom_quat, order);
        select(&mut self.geom_size, order);
        select(&mut self.geom_friction, order);
        select(&mut self.geom_condim, order);
        select(&mut self.geom_shellinertia_on_mesh, order);
        select(&mut self.geom_sdf_names_no_mesh, order);
        select(&mut self.geom_contype, order);
        select(&mut self.geom_conaffinity, order);
        select(&mut self.geom_name, order);
        select(&mut self.geom_mesh, order);
        select(&mut self.geom_solref, order);
        select(&mut self.geom_solimp, order);
        select(&mut self.geom_priority, order);
        select(&mut self.geom_solmix, order);
        select(&mut self.geom_margin, order);
        select(&mut self.geom_gap, order);
        select(&mut self.geom_group, order);
        select(&mut self.geom_rgba, order);
        select(&mut self.geom_fluid, order);
        select(&mut self.geom_hfield, order);
        select(&mut self.geom_sdf, order);
        select(&mut self.geom_user_raw, order);
        self.geom_name_to_id = name_map(&self.geom_name);
    }

    /// Every site array at `order`, the site name map made again. Each
    /// site's body is the caller's.
    pub(crate) fn select_sites(&mut self, order: &[usize]) {
        select(&mut self.site_body, order);
        select(&mut self.site_type, order);
        select(&mut self.site_pos, order);
        select(&mut self.site_quat, order);
        select(&mut self.site_size, order);
        select(&mut self.site_name, order);
        select(&mut self.site_group, order);
        select(&mut self.site_rgba, order);
        select(&mut self.site_user_raw, order);
        self.site_name_to_id = name_map(&self.site_name);
    }

    /// Every body array at `order` (the world's first), the body name map
    /// made again. Each body's parent, root and geom range, and each
    /// joint's and dof's body, are the caller's.
    pub(crate) fn select_bodies(&mut self, order: &[usize]) {
        debug_assert_eq!(order.first(), Some(&0), "the world is the first body");
        select(&mut self.body_parent, order);
        select(&mut self.body_rootid, order);
        select(&mut self.body_jnt_adr, order);
        select(&mut self.body_jnt_num, order);
        select(&mut self.body_dof_adr, order);
        select(&mut self.body_dof_num, order);
        select(&mut self.body_geom_adr, order);
        select(&mut self.body_geom_num, order);
        select(&mut self.body_pos, order);
        select(&mut self.body_quat, order);
        select(&mut self.body_ipos, order);
        select(&mut self.body_iquat, order);
        select(&mut self.body_mass, order);
        select(&mut self.body_inertia, order);
        select(&mut self.body_name, order);
        select(&mut self.body_mocapid, order);
        select(&mut self.body_sleep_policy, order);
        select(&mut self.body_gravcomp, order);
        select(&mut self.body_faults, order);
        select(&mut self.body_user_raw, order);
        // The world has no entry in these: body b's is at b - 1.
        let below_world: Vec<usize> = order.iter().skip(1).map(|&b| b - 1).collect();
        select(&mut self.body_world_pos, &below_world);
        select(&mut self.body_world_quat, &below_world);
        self.body_name_to_id = name_map(&self.body_name);
    }
}
