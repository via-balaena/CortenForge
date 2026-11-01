//! Structural checks of a [`Model`], for tests: a model the MJCF builder
//! or a factory makes sizes every per-element array to its element count.

use crate::types::Model;

/// Every per-element array of a [`Model`] whose length is not its element
/// count, as `"field len/count"`, and the fields checked.
///
/// The arrays are the per-joint, per-dof, per-body, per-geom, per-site,
/// per-tendon, per-actuator and per-sensor ones; `tendon_tree` holds two
/// entries per tendon.
#[must_use]
pub fn short_arrays(m: &Model) -> (Vec<String>, Vec<&'static str>) {
    let mut short = Vec::new();
    let mut checked = Vec::new();
    let mut check = |field: &'static str, len: usize, n: usize| {
        checked.push(field);
        if len != n {
            short.push(format!("{field} {len}/{n}"));
        }
    };
    check("jnt_type", m.jnt_type.len(), m.njnt);
    check("jnt_body", m.jnt_body.len(), m.njnt);
    check("jnt_qpos_adr", m.jnt_qpos_adr.len(), m.njnt);
    check("jnt_dof_adr", m.jnt_dof_adr.len(), m.njnt);
    check("jnt_pos", m.jnt_pos.len(), m.njnt);
    check("jnt_axis", m.jnt_axis.len(), m.njnt);
    check("jnt_limited", m.jnt_limited.len(), m.njnt);
    check("jnt_range", m.jnt_range.len(), m.njnt);
    check("jnt_stiffness", m.jnt_stiffness.len(), m.njnt);
    check("jnt_springref", m.jnt_springref.len(), m.njnt);
    check("jnt_damping", m.jnt_damping.len(), m.njnt);
    check("jnt_solref", m.jnt_solref.len(), m.njnt);
    check("jnt_solimp", m.jnt_solimp.len(), m.njnt);
    check("jnt_name", m.jnt_name.len(), m.njnt);
    check("jnt_group", m.jnt_group.len(), m.njnt);
    check("jnt_actgravcomp", m.jnt_actgravcomp.len(), m.njnt);
    check("jnt_margin", m.jnt_margin.len(), m.njnt);
    check("jnt_user", m.jnt_user.len(), m.njnt);
    check("dof_treeid", m.dof_treeid.len(), m.nv);
    check("dof_length", m.dof_length.len(), m.nv);
    check("dof_body", m.dof_body.len(), m.nv);
    check("dof_jnt", m.dof_jnt.len(), m.nv);
    check("dof_parent", m.dof_parent.len(), m.nv);
    check("dof_armature", m.dof_armature.len(), m.nv);
    check("dof_damping", m.dof_damping.len(), m.nv);
    check("dof_frictionloss", m.dof_frictionloss.len(), m.nv);
    check("dof_solref", m.dof_solref.len(), m.nv);
    check("dof_solimp", m.dof_solimp.len(), m.nv);
    check("dof_invweight0", m.dof_invweight0.len(), m.nv);
    check("dof_simplenum", m.dof_simplenum.len(), m.nv);
    check("dof_M0", m.dof_M0.len(), m.nv);
    check("body_treeid", m.body_treeid.len(), m.nbody);
    check("body_parent", m.body_parent.len(), m.nbody);
    check("body_rootid", m.body_rootid.len(), m.nbody);
    check("body_sameframe", m.body_sameframe.len(), m.nbody);
    check("body_simple", m.body_simple.len(), m.nbody);
    check("body_jnt_adr", m.body_jnt_adr.len(), m.nbody);
    check("body_jnt_num", m.body_jnt_num.len(), m.nbody);
    check("body_dof_adr", m.body_dof_adr.len(), m.nbody);
    check("body_dof_num", m.body_dof_num.len(), m.nbody);
    check("body_geom_adr", m.body_geom_adr.len(), m.nbody);
    check("body_geom_num", m.body_geom_num.len(), m.nbody);
    check("body_pos", m.body_pos.len(), m.nbody);
    check("body_quat", m.body_quat.len(), m.nbody);
    check("body_ipos", m.body_ipos.len(), m.nbody);
    check("body_iquat", m.body_iquat.len(), m.nbody);
    check("body_mass", m.body_mass.len(), m.nbody);
    check("body_inertia", m.body_inertia.len(), m.nbody);
    check("body_name", m.body_name.len(), m.nbody);
    check("body_subtreemass", m.body_subtreemass.len(), m.nbody);
    check("body_mocapid", m.body_mocapid.len(), m.nbody);
    check("body_gravcomp", m.body_gravcomp.len(), m.nbody);
    check("body_invweight0", m.body_invweight0.len(), m.nbody);
    check("body_weldid", m.body_weldid.len(), m.nbody);
    check("body_margin", m.body_margin.len(), m.nbody);
    check("body_bvhadr", m.body_bvhadr.len(), m.nbody);
    check("body_bvhnum", m.body_bvhnum.len(), m.nbody);
    check(
        "body_ancestor_joints",
        m.body_ancestor_joints.len(),
        m.nbody,
    );
    check("body_ancestor_mask", m.body_ancestor_mask.len(), m.nbody);
    check("body_user", m.body_user.len(), m.nbody);
    check("body_plugin", m.body_plugin.len(), m.nbody);
    check("geom_type", m.geom_type.len(), m.ngeom);
    check("geom_body", m.geom_body.len(), m.ngeom);
    check("geom_pos", m.geom_pos.len(), m.ngeom);
    check("geom_quat", m.geom_quat.len(), m.ngeom);
    check("geom_size", m.geom_size.len(), m.ngeom);
    check("geom_friction", m.geom_friction.len(), m.ngeom);
    check("geom_condim", m.geom_condim.len(), m.ngeom);
    check("geom_contype", m.geom_contype.len(), m.ngeom);
    check("geom_conaffinity", m.geom_conaffinity.len(), m.ngeom);
    check("geom_margin", m.geom_margin.len(), m.ngeom);
    check("geom_gap", m.geom_gap.len(), m.ngeom);
    check("geom_priority", m.geom_priority.len(), m.ngeom);
    check("geom_solmix", m.geom_solmix.len(), m.ngeom);
    check("geom_solimp", m.geom_solimp.len(), m.ngeom);
    check("geom_solref", m.geom_solref.len(), m.ngeom);
    check("geom_fluid", m.geom_fluid.len(), m.ngeom);
    check("geom_name", m.geom_name.len(), m.ngeom);
    check("geom_rbound", m.geom_rbound.len(), m.ngeom);
    check("geom_sameframe", m.geom_sameframe.len(), m.ngeom);
    check("geom_aabb", m.geom_aabb.len(), m.ngeom);
    check("geom_mesh", m.geom_mesh.len(), m.ngeom);
    check("geom_hfield", m.geom_hfield.len(), m.ngeom);
    check("geom_shape", m.geom_shape.len(), m.ngeom);
    check("geom_group", m.geom_group.len(), m.ngeom);
    check("geom_rgba", m.geom_rgba.len(), m.ngeom);
    check("geom_user", m.geom_user.len(), m.ngeom);
    check("geom_plugin", m.geom_plugin.len(), m.ngeom);
    check("site_body", m.site_body.len(), m.nsite);
    check("site_type", m.site_type.len(), m.nsite);
    check("site_pos", m.site_pos.len(), m.nsite);
    check("site_quat", m.site_quat.len(), m.nsite);
    check("site_size", m.site_size.len(), m.nsite);
    check("site_name", m.site_name.len(), m.nsite);
    check("site_group", m.site_group.len(), m.nsite);
    check("site_rgba", m.site_rgba.len(), m.nsite);
    check("site_user", m.site_user.len(), m.nsite);
    check("site_sameframe", m.site_sameframe.len(), m.nsite);
    check("tendon_range", m.tendon_range.len(), m.ntendon);
    check("tendon_limited", m.tendon_limited.len(), m.ntendon);
    check("tendon_stiffness", m.tendon_stiffness.len(), m.ntendon);
    check("tendon_damping", m.tendon_damping.len(), m.ntendon);
    check(
        "tendon_lengthspring",
        m.tendon_lengthspring.len(),
        m.ntendon,
    );
    check("tendon_length0", m.tendon_length0.len(), m.ntendon);
    check("tendon_num", m.tendon_num.len(), m.ntendon);
    check("tendon_adr", m.tendon_adr.len(), m.ntendon);
    check("tendon_name", m.tendon_name.len(), m.ntendon);
    check("tendon_type", m.tendon_type.len(), m.ntendon);
    check("tendon_solref_lim", m.tendon_solref_lim.len(), m.ntendon);
    check("tendon_solimp_lim", m.tendon_solimp_lim.len(), m.ntendon);
    check("tendon_margin", m.tendon_margin.len(), m.ntendon);
    check(
        "tendon_frictionloss",
        m.tendon_frictionloss.len(),
        m.ntendon,
    );
    check("tendon_solref_fri", m.tendon_solref_fri.len(), m.ntendon);
    check("tendon_solimp_fri", m.tendon_solimp_fri.len(), m.ntendon);
    check("tendon_group", m.tendon_group.len(), m.ntendon);
    check("tendon_rgba", m.tendon_rgba.len(), m.ntendon);
    check("tendon_treenum", m.tendon_treenum.len(), m.ntendon);
    check("tendon_invweight0", m.tendon_invweight0.len(), m.ntendon);
    check("tendon_user", m.tendon_user.len(), m.ntendon);
    check("actuator_trntype", m.actuator_trntype.len(), m.nu);
    check("actuator_dyntype", m.actuator_dyntype.len(), m.nu);
    check("actuator_trnid", m.actuator_trnid.len(), m.nu);
    check("actuator_gear", m.actuator_gear.len(), m.nu);
    check("actuator_ctrlrange", m.actuator_ctrlrange.len(), m.nu);
    check("actuator_forcerange", m.actuator_forcerange.len(), m.nu);
    check("actuator_name", m.actuator_name.len(), m.nu);
    check("actuator_act_adr", m.actuator_act_adr.len(), m.nu);
    check("actuator_act_num", m.actuator_act_num.len(), m.nu);
    check("actuator_gaintype", m.actuator_gaintype.len(), m.nu);
    check("actuator_biastype", m.actuator_biastype.len(), m.nu);
    check("actuator_dynprm", m.actuator_dynprm.len(), m.nu);
    check("actuator_gainprm", m.actuator_gainprm.len(), m.nu);
    check("actuator_biasprm", m.actuator_biasprm.len(), m.nu);
    check("actuator_lengthrange", m.actuator_lengthrange.len(), m.nu);
    check("actuator_acc0", m.actuator_acc0.len(), m.nu);
    check("actuator_actlimited", m.actuator_actlimited.len(), m.nu);
    check("actuator_actrange", m.actuator_actrange.len(), m.nu);
    check("actuator_actearly", m.actuator_actearly.len(), m.nu);
    check("actuator_cranklength", m.actuator_cranklength.len(), m.nu);
    check("actuator_nsample", m.actuator_nsample.len(), m.nu);
    check("actuator_interp", m.actuator_interp.len(), m.nu);
    check("actuator_historyadr", m.actuator_historyadr.len(), m.nu);
    check("actuator_delay", m.actuator_delay.len(), m.nu);
    check("actuator_group", m.actuator_group.len(), m.nu);
    check("actuator_user", m.actuator_user.len(), m.nu);
    check("actuator_plugin", m.actuator_plugin.len(), m.nu);
    check("tendon_tree", m.tendon_tree.len(), 2 * m.ntendon);
    check("sensor_type", m.sensor_type.len(), m.nsensor);
    check("sensor_datatype", m.sensor_datatype.len(), m.nsensor);
    check("sensor_objtype", m.sensor_objtype.len(), m.nsensor);
    check("sensor_objid", m.sensor_objid.len(), m.nsensor);
    check("sensor_reftype", m.sensor_reftype.len(), m.nsensor);
    check("sensor_refid", m.sensor_refid.len(), m.nsensor);
    check("sensor_adr", m.sensor_adr.len(), m.nsensor);
    check("sensor_dim", m.sensor_dim.len(), m.nsensor);
    check("sensor_noise", m.sensor_noise.len(), m.nsensor);
    check("sensor_cutoff", m.sensor_cutoff.len(), m.nsensor);
    check("sensor_name", m.sensor_name.len(), m.nsensor);
    check("sensor_nsample", m.sensor_nsample.len(), m.nsensor);
    check("sensor_interp", m.sensor_interp.len(), m.nsensor);
    check("sensor_historyadr", m.sensor_historyadr.len(), m.nsensor);
    check("sensor_delay", m.sensor_delay.len(), m.nsensor);
    check("sensor_interval", m.sensor_interval.len(), m.nsensor);
    check("sensor_user", m.sensor_user.len(), m.nsensor);
    check("sensor_plugin", m.sensor_plugin.len(), m.nsensor);
    (short, checked)
}
