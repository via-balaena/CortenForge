//! Final Model assembly.
//!
//! Consumes a fully-populated [`ModelBuilder`] and produces a physics-ready
//! [`Model`]. Handles kinematic tree enumeration, sleep policy resolution,
//! tendon length computation, bounding sphere pre-computation, and all other
//! post-processing that requires the complete builder state.

use nalgebra::{DVector, Vector3};
use sim_core::{
    ActuatorTransmission, Bounded, ENABLE_SLEEP, GeomType, Integrator, Model, SleepPolicy,
    TendonType, WrapType, compute_dof_lengths,
};
use tracing::warn;

use super::ModelBuilder;
use super::flex::{
    compute_element_adjacency, compute_flex_address_table, compute_flex_count_table,
    compute_flexedge_crosssection,
};

impl ModelBuilder {
    /// Build a physics-ready [`Model`] from the fully-populated builder.
    ///
    /// Orchestrates model assembly and all post-construction computations.
    pub(crate) fn build(self) -> Model {
        let body_sleep_policy = self.body_sleep_policy.clone();
        let mut model = self.assemble_model();

        // Structural pre-computation
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        compute_history_addresses(&mut model);

        // Tendon and actuator derived parameters
        model.compute_spatial_tendon_length0();
        model.compute_actuator_params();
        model.compute_stat_meaninertia();
        model.compute_invweight0();

        // Bounding geometry and tendon rest lengths
        compute_geom_bounding_radii(&mut model);
        compute_fixed_tendon_lengths(&mut model);

        // Kinematic tree enumeration and sleep policy
        discover_kinematic_trees(&mut model);
        compute_tendon_tree_mapping(&mut model);
        resolve_sleep_policies(&mut model, &body_sleep_policy);
        compute_dof_lengths(&mut model);
        guard_rk4_sleep(&mut model);

        model
    }

    /// Assemble the Model struct from builder fields.
    ///
    /// Moves all accumulated builder data into a fresh `Model`, performing only
    /// the minimal pre-computations needed before the struct literal (nuser
    /// resolution, flex address/count tables, element adjacency).
    fn assemble_model(self) -> Model {
        let njnt = self.jnt_type.len();
        let nbody = self.body_parent.len();
        let ngeom = self.geom_type.len();
        let nsite = self.site_body.len();
        let nu = self.actuator_trntype.len();
        let ntendon = self.tendon_type.len();
        let nsensor = self.sensor_type.len();

        // Compute resolved nuser_* values before moving raw arrays.
        // After finalize_user_data, all inner vecs have uniform length per type.
        // User data lengths are always small (< 100 typically), so truncation is safe.
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_body = self.body_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_jnt = self.jnt_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_geom = self.geom_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_site = self.site_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_tendon = self.tendon_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_actuator = self.actuator_user_raw.first().map_or(0, |v| v.len()) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let nuser_sensor = self.sensor_user_raw.first().map_or(0, |v| v.len()) as i32;

        // Pre-compute flex address/count/crosssection tables before self is consumed.
        let flex_edgeadr = compute_flex_address_table(&self.flexedge_flexid, self.nflex);
        let flex_edgenum = compute_flex_count_table(&self.flexedge_flexid, self.nflex);
        let flex_elemadr = compute_flex_address_table(&self.flexelem_flexid, self.nflex);
        let flex_elemnum = compute_flex_count_table(&self.flexelem_flexid, self.nflex);
        let flexedge_crosssection = compute_flexedge_crosssection(
            &self.flexedge_flexid,
            &self.flexedge_length0,
            &self.flex_dim,
            &self.flex_thickness,
            &self.flexvert_radius,
            &self.flex_vertadr,
        );

        // Compute element adjacency for self-collision dispatch (Spec C S4).
        let (flex_elem_adj, flex_elem_adj_adr, flex_elem_adj_num) = compute_element_adjacency(
            &self.flexelem_data,
            &self.flexelem_dataadr,
            &self.flexelem_datanum,
            self.flexelem_flexid.len(),
            self.nflexvert,
        );

        Model {
            name: self.name,
            nq: self.nq,
            nv: self.nv,
            nbody,
            njnt,
            ngeom,
            nsite,
            nu,
            na: self.na,
            nmocap: self.nmocap,
            nkeyframe: 0,

            // Kinematic trees (§16.0) — computed below
            ntree: 0,
            tree_body_adr: vec![],
            tree_body_num: vec![],
            tree_dof_adr: vec![],
            tree_dof_num: vec![],
            body_treeid: vec![usize::MAX; nbody],
            dof_treeid: vec![0; self.nv],
            tree_sleep_policy: vec![],
            dof_length: vec![1.0; self.nv],
            sleep_tolerance: self.sleep_tolerance,

            body_parent: self.body_parent,
            body_rootid: self.body_rootid,
            body_jnt_adr: self.body_jnt_adr,
            body_jnt_num: self.body_jnt_num,
            body_dof_adr: self.body_dof_adr,
            body_dof_num: self.body_dof_num,
            body_geom_adr: self.body_geom_adr,
            body_geom_num: self.body_geom_num,
            body_pos: self.body_pos,
            body_quat: self.body_quat,
            body_ipos: self.body_ipos,
            body_iquat: self.body_iquat,
            body_mass: self.body_mass,
            body_inertia: self.body_inertia,
            body_name: self.body_name,
            body_subtreemass: vec![0.0; nbody], // Computed after model construction
            body_mocapid: self.body_mocapid,
            ngravcomp: self.body_gravcomp.iter().filter(|&&gc| gc != 0.0).count(),
            body_gravcomp: self.body_gravcomp,
            body_invweight0: vec![[0.0; 2]; nbody], // Computed by compute_invweight0()

            jnt_type: self.jnt_type,
            jnt_body: self.jnt_body,
            jnt_qpos_adr: self.jnt_qpos_adr,
            jnt_dof_adr: self.jnt_dof_adr,
            jnt_pos: self.jnt_pos,
            jnt_axis: self.jnt_axis,
            jnt_limited: self.jnt_limited,
            jnt_range: self.jnt_range,
            jnt_stiffness: self.jnt_stiffness,
            jnt_springref: self.jnt_springref,
            jnt_damping: self.jnt_damping,
            jnt_armature: self.jnt_armature,
            jnt_solref: self.jnt_solref,
            jnt_solimp: self.jnt_solimp,
            jnt_name: self.jnt_name,
            jnt_group: self.jnt_group,
            jnt_actgravcomp: self.jnt_actgravcomp,
            jnt_margin: self.jnt_margin,

            dof_body: self.dof_body,
            dof_jnt: self.dof_jnt,
            dof_parent: self.dof_parent,
            dof_armature: self.dof_armature,
            dof_damping: self.dof_damping,
            dof_frictionloss: self.dof_frictionloss,
            dof_solref: self.dof_solref,
            dof_solimp: self.dof_solimp,
            dof_invweight0: vec![0.0; self.nv], // Computed by compute_invweight0()

            // Sparse LDL CSR metadata (computed below via compute_qld_csr_metadata)
            qLD_rowadr: vec![],
            qLD_rownnz: vec![],
            qLD_colind: vec![],
            qLD_nnz: 0,

            geom_type: self.geom_type,
            geom_body: self.geom_body,
            geom_pos: self.geom_pos,
            geom_quat: self.geom_quat,
            geom_size: self.geom_size,
            geom_friction: self.geom_friction,
            geom_condim: self.geom_condim,
            geom_contype: self.geom_contype,
            geom_conaffinity: self.geom_conaffinity,
            geom_margin: self.geom_margin,
            geom_gap: self.geom_gap,
            geom_priority: self.geom_priority,
            geom_solmix: self.geom_solmix,
            geom_solimp: self.geom_solimp,
            geom_solref: self.geom_solref,
            geom_name: self.geom_name,
            // Pre-computed bounding volumes (computed below after we have geom_type and geom_size)
            geom_rbound: vec![0.0; ngeom],
            geom_aabb: vec![[0.0; 6]; ngeom],
            // Mesh index for each geom (populated by process_geom)
            geom_mesh: self.geom_mesh,
            // Hfield index for each geom (populated by process_geom)
            geom_hfield: self.geom_hfield,
            // SDF index for each geom (always None from MJCF — programmatic only)
            geom_shape: self.geom_sdf,
            geom_group: self.geom_group,
            geom_rgba: self.geom_rgba,
            geom_fluid: self.geom_fluid,

            // Flex bodies
            nflex: self.nflex,
            nflexvert: self.nflexvert,
            nflexedge: self.nflexedge,
            nflexelem: self.nflexelem,
            nflexhinge: self.nflexhinge,
            flex_dim: self.flex_dim,
            flex_vertadr: self.flex_vertadr,
            flex_vertnum: self.flex_vertnum,
            flex_edgeadr,
            flex_edgenum,
            flex_elemadr,
            flex_elemnum,
            flex_young: self.flex_young,
            flex_poisson: self.flex_poisson,
            flex_damping: self.flex_damping,
            flex_thickness: self.flex_thickness,
            flex_friction: self.flex_friction,
            flex_solref: self.flex_solref,
            flex_solimp: self.flex_solimp,
            flex_condim: self.flex_condim,
            flex_margin: self.flex_margin,
            flex_gap: self.flex_gap,
            flex_priority: self.flex_priority,
            flex_solmix: self.flex_solmix,
            flex_contype: self.flex_contype,
            flex_conaffinity: self.flex_conaffinity,
            flex_selfcollide: self.flex_selfcollide,
            flex_internal: self.flex_internal,
            flex_activelayers: self.flex_activelayers,
            flex_vertcollide: self.flex_vertcollide,
            flex_passive: self.flex_passive,
            flex_edgestiffness: self.flex_edgestiffness,
            flex_edgedamping: self.flex_edgedamping,
            flex_edge_solref: self.flex_edge_solref,
            flex_edge_solimp: self.flex_edge_solimp,
            flex_bend_stiffness: self.flex_bend_stiffness,
            flex_bend_damping: self.flex_bend_damping,
            flex_density: self.flex_density,
            flex_group: self.flex_group,
            flex_rigid: self.flex_rigid,
            flex_bending_type: self.flex_bending_type,
            flexvert_qposadr: self.flexvert_qposadr,
            flexvert_dofadr: self.flexvert_dofadr,
            flexvert_mass: self.flexvert_mass,
            flexvert_invmass: self.flexvert_invmass,
            flexvert_radius: self.flexvert_radius,
            flexvert_flexid: self.flexvert_flexid,
            flexvert_bodyid: self.flexvert_bodyid,
            flexedge_vert: self.flexedge_vert,
            flexedge_length0: self.flexedge_length0,
            flexedge_crosssection,
            flexedge_flexid: self.flexedge_flexid,
            flexedge_rigid: self.flexedge_rigid,
            flexedge_flap: self.flexedge_flap,
            flex_bending: self.flex_bending,
            flexedge_J_rownnz: self.flexedge_J_rownnz,
            flexedge_J_rowadr: self.flexedge_J_rowadr,
            flexedge_J_colind: self.flexedge_J_colind,
            flexelem_data: self.flexelem_data,
            flexelem_dataadr: self.flexelem_dataadr,
            flexelem_datanum: self.flexelem_datanum,
            flexelem_volume0: self.flexelem_volume0,
            flexelem_flexid: self.flexelem_flexid,
            flex_elem_adj,
            flex_elem_adj_adr,
            flex_elem_adj_num,
            flexhinge_vert: self.flexhinge_vert,
            flexhinge_angle0: self.flexhinge_angle0,
            flexhinge_flexid: self.flexhinge_flexid,

            // Mesh assets (from MJCF <asset><mesh> elements)
            nmesh: self.mesh_data.len(),
            mesh_name: self.mesh_name,
            mesh_data: self.mesh_data,

            // Height field assets (from MJCF <asset><hfield> elements)
            nhfield: self.hfield_data.len(),
            hfield_name: self.hfield_name,
            hfield_data: self.hfield_data,
            hfield_size: self.hfield_size,

            // Shape assets (programmatic — always empty from MJCF)
            nshape: 0,
            shape_data: vec![],
            gpu_collider: None,

            site_body: self.site_body,
            site_type: self.site_type,
            site_pos: self.site_pos,
            site_quat: self.site_quat,
            site_size: self.site_size,
            site_name: self.site_name,
            site_group: self.site_group,
            site_rgba: self.site_rgba,

            // Sensors (populated by process_sensors)
            nsensor: self.nsensor,
            nsensordata: self.nsensordata,
            sensor_type: self.sensor_type,
            sensor_datatype: self.sensor_datatype,
            sensor_objtype: self.sensor_objtype,
            sensor_objid: self.sensor_objid,
            sensor_reftype: self.sensor_reftype,
            sensor_refid: self.sensor_refid,
            sensor_adr: self.sensor_adr,
            sensor_dim: self.sensor_dim,
            sensor_noise: self.sensor_noise,
            sensor_cutoff: self.sensor_cutoff,
            sensor_name: self.sensor_name_list,
            sensor_nsample: self.sensor_nsample,
            sensor_interp: self.sensor_interp,
            sensor_delay: self.sensor_delay,
            sensor_interval: self.sensor_interval,
            sensor_historyadr: vec![],

            actuator_trntype: self.actuator_trntype,
            actuator_dyntype: self.actuator_dyntype,
            actuator_trnid: self.actuator_trnid,
            actuator_gear: self.actuator_gear,
            actuator_ctrlrange: self.actuator_ctrlrange,
            actuator_forcerange: self.actuator_forcerange,
            actuator_name: self.actuator_name,
            actuator_act_adr: self.actuator_act_adr,
            actuator_act_num: self.actuator_act_num,
            actuator_gaintype: self.actuator_gaintype,
            actuator_biastype: self.actuator_biastype,
            actuator_dynprm: self.actuator_dynprm,
            actuator_gainprm: self.actuator_gainprm,
            actuator_biasprm: self.actuator_biasprm,
            actuator_lengthrange: self.actuator_lengthrange,
            actuator_acc0: self.actuator_acc0,
            actuator_actlimited: self.actuator_actlimited,
            actuator_actrange: self.actuator_actrange,
            actuator_actearly: self.actuator_actearly,
            actuator_cranklength: self.actuator_cranklength,
            actuator_nsample: self.actuator_nsample,
            actuator_interp: self.actuator_interp,
            // historyadr and nhistory computed post-hoc below
            actuator_historyadr: vec![],
            actuator_delay: self.actuator_delay,
            nhistory: 0,

            // Tendons (populated by process_tendons)
            ntendon: self.tendon_type.len(),
            nwrap: self.wrap_type.len(),
            tendon_type: self.tendon_type,
            tendon_range: self.tendon_range,
            tendon_limited: self.tendon_limited,
            tendon_stiffness: self.tendon_stiffness,
            tendon_damping: self.tendon_damping,
            tendon_frictionloss: self.tendon_frictionloss,
            tendon_solref_fri: self.tendon_solref_fri,
            tendon_solimp_fri: self.tendon_solimp_fri,
            // tendon_treenum/tendon_tree computed below after tree enumeration
            tendon_treenum: vec![0; ntendon],
            tendon_tree: vec![usize::MAX; 2 * ntendon],
            tendon_invweight0: vec![0.0; ntendon], // Computed by compute_invweight0()
            tendon_lengthspring: self.tendon_lengthspring,
            tendon_length0: self.tendon_length0,
            tendon_num: self.tendon_num,
            tendon_adr: self.tendon_adr,
            tendon_name: self.tendon_name,
            tendon_group: self.tendon_group,
            tendon_rgba: self.tendon_rgba,
            tendon_solref_lim: self.tendon_solref_lim,
            tendon_solimp_lim: self.tendon_solimp_lim,
            tendon_margin: self.tendon_margin,
            wrap_type: self.wrap_type,
            wrap_objid: self.wrap_objid,
            wrap_prm: self.wrap_prm,
            wrap_sidesite: self.wrap_sidesite,

            // Equality constraints (populated by process_equality_constraints)
            neq: self.eq_type.len(),
            eq_type: self.eq_type,
            eq_obj1id: self.eq_obj1id,
            eq_obj2id: self.eq_obj2id,
            eq_data: self.eq_data,
            eq_active: self.eq_active,
            eq_solimp: self.eq_solimp,
            eq_solref: self.eq_solref,
            eq_name: self.eq_name,

            // Name↔index lookup (§59) — transfer builder maps
            body_name_to_id: self.body_name_to_id,
            jnt_name_to_id: self.joint_name_to_id,
            geom_name_to_id: self.geom_name_to_id,
            site_name_to_id: self.site_name_to_id,
            tendon_name_to_id: self.tendon_name_to_id,
            actuator_name_to_id: self.actuator_name_to_id,
            sensor_name_to_id: self.sensor_name_to_id,
            mesh_name_to_id: self.mesh_name_to_id,
            hfield_name_to_id: self.hfield_name_to_id,
            eq_name_to_id: self.eq_name_to_id,

            // Contact pairs / excludes (populated by process_contact)
            contact_pairs: self.contact_pairs,
            contact_pair_set: self.contact_pair_set,
            contact_excludes: self.contact_excludes,

            // User callbacks (DT-79) — not set from MJCF, user assigns post-load
            cb_passive: None,
            cb_control: None,
            cb_contactfilter: None,
            cb_sensor: None,
            cb_act_dyn: None,
            cb_act_gain: None,
            cb_act_bias: None,

            timestep: self.timestep,
            gravity: self.gravity,
            qpos0: DVector::from_vec(self.qpos0_values),
            qpos_spring: self.qpos_spring_values,
            keyframes: Vec::new(), // Populated post-build in model_from_mjcf()
            wind: self.wind,
            magnetic: self.magnetic,
            density: self.density,
            viscosity: self.viscosity,
            solver_iterations: self.solver_iterations,
            solver_tolerance: self.solver_tolerance,
            impratio: self.impratio,
            regularization: self.regularization,
            friction_smoothing: self.friction_smoothing,
            cone: self.cone,
            stat_meaninertia: 1.0, // Computed post-build in Step 3
            ls_iterations: self.ls_iterations,
            ls_tolerance: self.ls_tolerance,
            noslip_iterations: self.noslip_iterations,
            noslip_tolerance: self.noslip_tolerance,
            diagapprox_bodyweight: true, // MuJoCo uses bodyweight diagApprox
            disableflags: self.disableflags,
            enableflags: self.enableflags,
            disableactuator: self.disableactuator,
            actuator_group: self.actuator_group,
            o_margin: self.o_margin,
            o_solref: self.o_solref,
            o_solimp: self.o_solimp,
            o_friction: self.o_friction,
            ccd_iterations: self.ccd_iterations,
            ccd_tolerance: self.ccd_tolerance,
            sdf_iterations: self.sdf_iterations,
            sdf_initpoints: self.sdf_initpoints,
            sdf_maxcontact: self.sdf_maxcontact,
            integrator: self.integrator,
            solver_type: self.solver_type,

            // Cached implicit integration parameters (computed below)
            implicit_stiffness: DVector::zeros(self.nv),
            implicit_damping: DVector::zeros(self.nv),
            implicit_springref: DVector::zeros(self.nv),

            // Pre-computed kinematic data (will be populated by compute_ancestors)
            body_ancestor_joints: vec![vec![]; nbody],
            body_ancestor_mask: vec![vec![]; nbody], // Multi-word bitmask, computed by compute_ancestors

            // Per-element user data (§55, Spec C)
            body_user: self.body_user_raw,
            jnt_user: self.jnt_user_raw,
            geom_user: self.geom_user_raw,
            site_user: self.site_user_raw,
            tendon_user: self.tendon_user_raw,
            actuator_user: self.actuator_user_raw,
            sensor_user: self.sensor_user_raw,
            nuser_body,
            nuser_jnt,
            nuser_geom,
            nuser_site,
            nuser_tendon,
            nuser_actuator,
            nuser_sensor,

            // §66: Plugin instances (empty — populated by plugin resolution)
            nplugin: 0,
            npluginstate: 0,
            body_plugin: vec![None; nbody],
            geom_plugin: vec![None; ngeom],
            actuator_plugin: vec![None; nu],
            sensor_plugin: vec![None; nsensor],
            plugin_objects: Vec::new(),
            plugin_needstage: Vec::new(),
            plugin_capabilities: Vec::new(),
            plugin_stateadr: Vec::new(),
            plugin_statenum: Vec::new(),
            plugin_attr: Vec::new(),
            plugin_attradr: Vec::new(),
            plugin_attrnum: Vec::new(),
            plugin_name: Vec::new(),
        }
    }
}

// ===== Post-construction standalone helpers =====
//
// These operate on a fully-assembled Model and cannot be `impl Model` methods
// because `Model` is defined in sim-core, not sim-mjcf.

/// Compute `actuator_historyadr`, `sensor_historyadr`, and `nhistory`.
///
/// Layout: actuators first (offset 0 → act_total), sensors after (act_total → nhistory).
/// Must run BEFORE `compute_actuator_params()` because that calls `make_data()`
/// which reads `actuator_historyadr` for history buffer pre-population.
fn compute_history_addresses(model: &mut Model) {
    // Actuator historyadr
    let nu = model.actuator_nsample.len();
    let mut act_historyadr = vec![-1i32; nu];
    let mut offset: i32 = 0;
    for (adr, &ns) in act_historyadr.iter_mut().zip(model.actuator_nsample.iter()) {
        if ns > 0 {
            *adr = offset;
            offset += 2 * ns + 2;
        }
    }
    model.actuator_historyadr = act_historyadr;

    // Sensor historyadr (appends after actuators)
    let nsens = model.sensor_nsample.len();
    let mut sens_historyadr = vec![-1i32; nsens];
    for (i, (adr, &ns)) in sens_historyadr
        .iter_mut()
        .zip(model.sensor_nsample.iter())
        .enumerate()
    {
        if ns > 0 {
            *adr = offset;
            // nsample * (dim + 1) + 2 — generalized formula
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
            let dim = model.sensor_dim[i] as i32;
            offset += ns * (dim + 1) + 2;
        }
    }
    model.sensor_historyadr = sens_historyadr;

    #[allow(clippy::cast_sign_loss)]
    {
        model.nhistory = offset as usize;
    }
}

/// Pre-compute bounding volumes for all geoms (collision broad-phase).
///
/// Populates both `geom_rbound` (bounding sphere radius) and `geom_aabb`
/// (local-frame AABB as `[cx, cy, cz, hx, hy, hz]` — center offset +
/// half-extents). For meshes/hfield/sdf, bounds come from actual geometry
/// data. For primitives, bounds come from `geom_size`.
///
/// `geom_aabb` is transformed to world-space at runtime by `mj_collision`,
/// replacing the per-type dispatch in `aabb_from_geom`. This eliminates the
/// `MESH_DEFAULT_EXTENT` fallback and matches MuJoCo's `mjModel.geom_aabb`.
fn compute_geom_bounding_radii(model: &mut Model) {
    for geom_id in 0..model.ngeom {
        let (rbound, aabb) = if let Some(mesh_id) = model.geom_mesh[geom_id] {
            // Mesh geom: bounds from actual vertex positions.
            let (aabb_min, aabb_max) = model.mesh_data[mesh_id].aabb();
            let center = (aabb_min.coords + aabb_max.coords) * 0.5;
            let half = (aabb_max.coords - aabb_min.coords) * 0.5;
            let rbound = half.norm();
            (
                rbound,
                [center.x, center.y, center.z, half.x, half.y, half.z],
            )
        } else if let Some(hfield_id) = model.geom_hfield[geom_id] {
            // Hfield geom: bounds from heightfield data.
            let aabb = model.hfield_data[hfield_id].aabb();
            let center = (aabb.min.coords + aabb.max.coords) * 0.5;
            let half = (aabb.max.coords - aabb.min.coords) * 0.5;
            let rbound = half.norm();
            (
                rbound,
                [center.x, center.y, center.z, half.x, half.y, half.z],
            )
        } else if let Some(sdf_id) = model.geom_shape[geom_id] {
            // SDF geom: bounds from SDF grid data.
            let aabb = model.shape_data[sdf_id].sdf_grid().aabb();
            let center = (aabb.min.coords + aabb.max.coords) * 0.5;
            let half = (aabb.max.coords - aabb.min.coords) * 0.5;
            let rbound = half.norm();
            (
                rbound,
                [center.x, center.y, center.z, half.x, half.y, half.z],
            )
        } else {
            // Primitive geom: local AABB from size parameters.
            let rbound = model.geom_type[geom_id].bounding_radius(model.geom_size[geom_id]);
            let aabb =
                local_aabb_from_primitive(model.geom_type[geom_id], model.geom_size[geom_id]);
            (rbound, aabb)
        };
        model.geom_rbound[geom_id] = rbound;
        model.geom_aabb[geom_id] = aabb;
    }
}

/// Compute local-frame AABB `[cx, cy, cz, hx, hy, hz]` for a primitive geom.
///
/// Center offset is `(0,0,0)` for all primitives (symmetric about their origin).
/// Half-extents come from the MuJoCo size convention for each type.
fn local_aabb_from_primitive(geom_type: GeomType, size: Vector3<f64>) -> [f64; 6] {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            [0.0, 0.0, 0.0, r, r, r]
        }
        GeomType::Box | GeomType::Ellipsoid => [0.0, 0.0, 0.0, size.x, size.y, size.z],
        GeomType::Capsule | GeomType::Cylinder => {
            // Local Z is the axis. Half-extents: radius in XY, radius+half_length in Z.
            let r = size.x;
            let h = size.y;
            [0.0, 0.0, 0.0, r, r, r + h]
        }
        GeomType::Plane => {
            // Infinite extent. The PLANE_EXTENT constant is applied at runtime
            // in aabb_from_geom_aabb since it depends on world-frame orientation.
            const INF: f64 = 1e6;
            [0.0, 0.0, 0.0, INF, INF, INF]
        }
        // Mesh/Hfield/Sdf are handled by the caller using actual geometry data.
        // This arm is a fallback for programmatic geoms that bypass the builder.
        GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => [0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
    }
}

/// Pre-compute fixed tendon `length0` and resolve `lengthspring` sentinels from `qpos0`/`qpos_spring`.
///
/// For fixed tendons: `length0 = Σ coef_w * qpos0[jnt_qposadr_w]`.
/// MuJoCo ref: `setSpring()` in `engine_setconst.c` uses `qpos_spring` for sentinel resolution.
fn compute_fixed_tendon_lengths(model: &mut Model) {
    for t in 0..model.ntendon {
        if model.tendon_type[t] == TendonType::Fixed {
            let adr = model.tendon_adr[t];
            let num = model.tendon_num[t];
            // Compute tendon_length0 at qpos0 configuration
            let mut length0 = 0.0;
            for w in adr..(adr + num) {
                let dof_adr = model.wrap_objid[w];
                let coef = model.wrap_prm[w];
                if dof_adr < model.nv {
                    let jnt_id = model.dof_jnt[dof_adr];
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    if qpos_adr < model.qpos0.len() {
                        length0 += coef * model.qpos0[qpos_adr];
                    }
                }
            }
            model.tendon_length0[t] = length0;

            // Resolve sentinel [-1, -1] at qpos_spring configuration (not qpos0).
            #[allow(clippy::float_cmp)]
            if model.tendon_lengthspring[t] == [-1.0, -1.0] {
                let mut spring_length = 0.0;
                for w in adr..(adr + num) {
                    let dof_adr = model.wrap_objid[w];
                    let coef = model.wrap_prm[w];
                    if dof_adr < model.nv {
                        let jnt_id = model.dof_jnt[dof_adr];
                        let qpos_adr = model.jnt_qpos_adr[jnt_id];
                        if qpos_adr < model.qpos_spring.len() {
                            spring_length += coef * model.qpos_spring[qpos_adr];
                        }
                    }
                }
                model.tendon_lengthspring[t] = [spring_length, spring_length];
            }
        }
    }
}

/// Kinematic tree enumeration (§16.0).
///
/// Groups bodies by `body_rootid` to discover kinematic trees.
/// Body 0 (world) is excluded — it is its own tree but never sleeps.
fn discover_kinematic_trees(model: &mut Model) {
    use std::collections::BTreeMap;

    let mut trees: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for body_id in 1..model.nbody {
        trees
            .entry(model.body_rootid[body_id])
            .or_default()
            .push(body_id);
    }
    model.ntree = trees.len();
    model.tree_body_adr = Vec::with_capacity(model.ntree);
    model.tree_body_num = Vec::with_capacity(model.ntree);
    model.tree_dof_adr = Vec::with_capacity(model.ntree);
    model.tree_dof_num = Vec::with_capacity(model.ntree);
    model.tree_sleep_policy = vec![SleepPolicy::Auto; model.ntree];

    // body_treeid[0] = usize::MAX (world sentinel, already set)
    for (tree_idx, (_root_body, body_ids)) in trees.iter().enumerate() {
        let first_body = body_ids[0];
        let body_count = body_ids.len();
        model.tree_body_adr.push(first_body);
        model.tree_body_num.push(body_count);

        // Assign tree id to each body
        for &bid in body_ids {
            model.body_treeid[bid] = tree_idx;
        }

        // DOF range: find min dof and total DOFs for this tree
        let mut min_dof = model.nv;
        let mut total_dofs = 0usize;
        for &bid in body_ids {
            let dof_start = model.body_dof_adr[bid];
            let dof_count = model.body_dof_num[bid];
            if dof_count > 0 && dof_start < min_dof {
                min_dof = dof_start;
            }
            total_dofs += dof_count;
        }
        if total_dofs == 0 {
            min_dof = 0; // Bodyless tree (e.g., static geoms)
        }
        model.tree_dof_adr.push(min_dof);
        model.tree_dof_num.push(total_dofs);

        // Assign tree id to each DOF
        for &bid in body_ids {
            let dof_start = model.body_dof_adr[bid];
            let dof_count = model.body_dof_num[bid];
            for dof in dof_start..(dof_start + dof_count) {
                model.dof_treeid[dof] = tree_idx;
            }
        }
    }
}

/// Tendon tree mapping (§16.10.1).
///
/// Compute `tendon_treenum`/`tendon_tree` by scanning each tendon's waypoints.
fn compute_tendon_tree_mapping(model: &mut Model) {
    for t in 0..model.ntendon {
        let mut tree_set = std::collections::BTreeSet::new();
        let adr = model.tendon_adr[t];
        let num = model.tendon_num[t];
        for w in adr..adr + num {
            let bid = match model.wrap_type[w] {
                WrapType::Joint => {
                    let dof_adr = model.wrap_objid[w];
                    if dof_adr < model.nv {
                        Some(model.dof_body[dof_adr])
                    } else {
                        None
                    }
                }
                WrapType::Site => {
                    let site_idx = model.wrap_objid[w];
                    if site_idx < model.nsite {
                        Some(model.site_body[site_idx])
                    } else {
                        None
                    }
                }
                WrapType::Geom => {
                    let geom_id = model.wrap_objid[w];
                    if geom_id < model.ngeom {
                        Some(model.geom_body[geom_id])
                    } else {
                        None
                    }
                }
                WrapType::Pulley => None,
            };
            if let Some(bid) = bid {
                if bid > 0 {
                    let tree = model.body_treeid[bid];
                    if tree < model.ntree {
                        tree_set.insert(tree);
                    }
                }
            }
        }
        model.tendon_treenum[t] = tree_set.len();
        if !tree_set.is_empty() {
            let mut iter = tree_set.iter();
            if let Some(&a) = iter.next() {
                model.tendon_tree[2 * t] = a;
            }
            if let Some(&b) = iter.next() {
                model.tendon_tree[2 * t + 1] = b;
            }
        }
    }
}

/// Sleep policy resolution (§16.0 steps 1-3).
///
/// Step 1: Mark trees with actuators as `AutoNever`.
/// Step 2: Apply explicit body-level sleep policies from MJCF.
/// Step 3: Convert remaining `Auto` to `AutoAllowed`.
fn resolve_sleep_policies(model: &mut Model, body_sleep_policy: &[Option<SleepPolicy>]) {
    // Step 1: Mark trees with actuators as AutoNever
    for act_id in 0..model.nu {
        let trn = model.actuator_trntype[act_id];
        let trnid = model.actuator_trnid[act_id];
        let body_id = match trn {
            ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
                if trnid[0] < model.njnt {
                    Some(model.jnt_body[trnid[0]])
                } else {
                    None
                }
            }
            ActuatorTransmission::Tendon => {
                // §16.26.5: Mark all trees spanned by the tendon as AutoNever.
                let tendon_idx = trnid[0];
                if tendon_idx < model.ntendon {
                    let wrap_start = model.tendon_adr[tendon_idx];
                    let wrap_count = model.tendon_num[tendon_idx];
                    for w in wrap_start..wrap_start + wrap_count {
                        let bid = match model.wrap_type[w] {
                            WrapType::Joint => {
                                let dof_adr = model.wrap_objid[w];
                                if dof_adr < model.nv {
                                    Some(model.dof_body[dof_adr])
                                } else {
                                    None
                                }
                            }
                            WrapType::Site => {
                                let site_idx = model.wrap_objid[w];
                                if site_idx < model.nsite {
                                    Some(model.site_body[site_idx])
                                } else {
                                    None
                                }
                            }
                            WrapType::Geom => {
                                let geom_id = model.wrap_objid[w];
                                if geom_id < model.ngeom {
                                    Some(model.geom_body[geom_id])
                                } else {
                                    None
                                }
                            }
                            WrapType::Pulley => None,
                        };
                        if let Some(bid) = bid {
                            if bid > 0 {
                                let tree = model.body_treeid[bid];
                                if tree < model.ntree {
                                    model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                                }
                            }
                        }
                    }
                }
                None
            }
            ActuatorTransmission::Site => {
                if trnid[0] < model.nsite {
                    Some(model.site_body[trnid[0]])
                } else {
                    None
                }
            }
            ActuatorTransmission::Body => {
                let bid = trnid[0];
                if bid > 0 && bid < model.nbody {
                    Some(bid)
                } else {
                    None
                }
            }
            ActuatorTransmission::SliderCrank => {
                // Mark both crank and slider site trees as AutoNever
                let crank_id = trnid[0];
                let slider_id = trnid[1];
                for &sid in &[crank_id, slider_id] {
                    if sid < model.nsite {
                        let bid = model.site_body[sid];
                        if bid > 0 {
                            let tree = model.body_treeid[bid];
                            if tree < model.ntree {
                                model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                            }
                        }
                    }
                }
                None
            }
        };
        if let Some(bid) = body_id {
            if bid > 0 {
                let tree = model.body_treeid[bid];
                if tree < model.ntree {
                    model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                }
            }
        }
    }

    // §16.18.2: Multi-tree tendon policy relaxation.
    // Passive multi-tree tendons with nonzero stiffness, damping, or active limits
    // create inter-tree coupling forces that prevent independent sleeping.
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] < 2 {
            continue;
        }
        let has_stiffness = model.tendon_stiffness[t].abs() > 0.0;
        let has_damping = model.tendon_damping[t].abs() > 0.0;
        let has_limit = model.tendon_limited[t];
        if has_stiffness || has_damping || has_limit {
            let t1 = model.tendon_tree[2 * t];
            let t2 = model.tendon_tree[2 * t + 1];
            if t1 < model.ntree && model.tree_sleep_policy[t1] == SleepPolicy::Auto {
                model.tree_sleep_policy[t1] = SleepPolicy::AutoNever;
            }
            if t2 < model.ntree && model.tree_sleep_policy[t2] == SleepPolicy::Auto {
                model.tree_sleep_policy[t2] = SleepPolicy::AutoNever;
            }
        }
    }

    // Step 2: Apply explicit body-level sleep policies from MJCF
    for (body_id, policy_opt) in body_sleep_policy.iter().enumerate().skip(1) {
        if let Some(policy) = policy_opt {
            let tree = model.body_treeid[body_id];
            if tree < model.ntree {
                let root_body = model.tree_body_adr[tree];
                if body_id != root_body {
                    let body_name = model.body_name[body_id].as_deref().unwrap_or("unnamed");
                    warn!(
                        "body '{}': sleep attribute on non-root body propagates to tree root",
                        body_name
                    );
                }
                model.tree_sleep_policy[tree] = *policy;
            }
        }
    }

    // Step 3: Convert remaining Auto to AutoAllowed
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] == SleepPolicy::Auto {
            model.tree_sleep_policy[t] = SleepPolicy::AutoAllowed;
        }
    }
}

/// RK4 incompatibility guard (§16.6).
///
/// Sleeping is incompatible with the RK4 integrator; disable sleep if both are active.
fn guard_rk4_sleep(model: &mut Model) {
    if model.enableflags & ENABLE_SLEEP != 0 && model.integrator == Integrator::RungeKutta4 {
        warn!("Sleeping is incompatible with RK4 integrator. Disabling sleep.");
        model.enableflags &= !ENABLE_SLEEP;
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use sim_core::{ActuatorTransmission, MjJointType};

    #[test]
    fn test_simple_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="pendulum">
                <option timestep="0.001" gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 2); // world + link1
        assert_eq!(model.njnt, 1);
        assert_eq!(model.nq, 1); // hinge has 1 qpos
        assert_eq!(model.nv, 1); // hinge has 1 DOF
        assert_eq!(model.ngeom, 1);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world

        // Check joint
        assert_eq!(model.jnt_type[0], MjJointType::Hinge);
        assert_eq!(model.jnt_body[0], 1); // joint on link1

        // Check options
        assert!((model.timestep - 0.001).abs() < 1e-10);
        assert!((model.gravity.z - (-9.81)).abs() < 1e-10);
    }

    #[test]
    fn test_double_pendulum() {
        let model = load_model(
            r#"
            <mujoco model="double_pendulum">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <geom type="capsule" size="0.05 0.25"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                            <geom type="capsule" size="0.05 0.25"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nbody, 3); // world + link1 + link2
        assert_eq!(model.njnt, 2);
        assert_eq!(model.nq, 2);
        assert_eq!(model.nv, 2);

        // Check body tree
        assert_eq!(model.body_parent[1], 0); // link1's parent is world
        assert_eq!(model.body_parent[2], 1); // link2's parent is link1
    }

    #[test]
    fn test_with_actuator() {
        let model = load_model(
            r#"
            <mujoco model="actuated">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="shoulder" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="shoulder" gear="100"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Joint);
        assert_eq!(model.actuator_trnid[0][0], 0); // First joint
        assert_eq!(model.actuator_trnid[0][1], usize::MAX); // No refsite
        assert!((model.actuator_gear[0][0] - 100.0).abs() < 1e-10);
    }

    // ===== Spec C: Per-element user data tests (T1–T18) =====

    /// T1: `<size>` element parsing — multiple elements merge (last-writer-wins).
    #[test]
    fn t1_size_element_parsing() {
        let mjcf = crate::parse_mjcf_str(
            r#"<mujoco><size nuser_body="5" nuser_geom="3"/>
            <size nuser_jnt="2"/><worldbody/></mujoco>"#,
        )
        .expect("parse");
        assert_eq!(mjcf.nuser_body, 5);
        assert_eq!(mjcf.nuser_geom, 3);
        assert_eq!(mjcf.nuser_jnt, 2);
        // Others remain at -1 (auto-size sentinel)
        assert_eq!(mjcf.nuser_site, -1);
        assert_eq!(mjcf.nuser_tendon, -1);
        assert_eq!(mjcf.nuser_actuator, -1);
        assert_eq!(mjcf.nuser_sensor, -1);
    }

    /// T2: Auto-sizing with world body and padding.
    #[test]
    fn t2_auto_sizing_with_world_body_and_padding() {
        let model = load_model(
            r#"<mujoco>
            <worldbody>
                <body name="b1" pos="0 0 1" user="1 2 3">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
                <body name="b2" pos="0 0 2" user="4 5">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_body, 3);
        // World body (index 0): no user attr → zeros
        assert_eq!(model.body_user[0], vec![0.0, 0.0, 0.0]);
        // b1: user="1 2 3"
        assert_eq!(model.body_user[1], vec![1.0, 2.0, 3.0]);
        // b2: user="4 5" → padded to nuser_body=3
        assert_eq!(model.body_user[2], vec![4.0, 5.0, 0.0]);
    }

    /// T2 sub-test: explicit nuser_body=-1 gives identical auto-sizing result.
    #[test]
    fn t2_explicit_minus_one_auto_size() {
        let model = load_model(
            r#"<mujoco>
            <size nuser_body="-1"/>
            <worldbody>
                <body name="b1" pos="0 0 1" user="1 2 3">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
                <body name="b2" pos="0 0 2" user="4 5">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_body, 3);
        assert_eq!(model.body_user[0], vec![0.0, 0.0, 0.0]);
        assert_eq!(model.body_user[1], vec![1.0, 2.0, 3.0]);
        assert_eq!(model.body_user[2], vec![4.0, 5.0, 0.0]);
    }

    /// T3: Explicit nuser_* with zero-padding.
    #[test]
    fn t3_explicit_nuser_with_padding() {
        let model = load_model(
            r#"<mujoco>
            <size nuser_geom="5"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom name="g1" type="sphere" size="0.1" user="1.5 2.5"/>
                    <geom name="g2" type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_geom, 5);
        let g1 = model.geom_name_to_id["g1"];
        let g2 = model.geom_name_to_id["g2"];
        assert_eq!(model.geom_user[g1], vec![1.5, 2.5, 0.0, 0.0, 0.0]);
        assert_eq!(model.geom_user[g2], vec![0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    /// T4: Too-long user data → build error.
    #[test]
    fn t4_too_long_user_data_error() {
        let err = load_model(
            r#"<mujoco>
            <size nuser_geom="2"/>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1" user="10 20 30 40"/>
                </body>
            </worldbody>
            </mujoco>"#,
        );
        assert!(err.is_err());
        let msg = err.unwrap_err().to_string();
        assert!(
            msg.contains("nuser_geom"),
            "error should mention nuser_geom: {msg}"
        );
    }

    /// T5: Default class inheritance with full replacement.
    #[test]
    fn t5_default_inheritance_full_replacement() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <joint user="100 200"/>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <joint name="j2" type="hinge" axis="0 0 1" user="300"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let j1 = model.jnt_name_to_id["j1"];
        let j2 = model.jnt_name_to_id["j2"];
        assert_eq!(model.nuser_jnt, 2);
        // j1 inherits default [100, 200]
        assert_eq!(model.jnt_user[j1], vec![100.0, 200.0]);
        // j2 overrides with [300] → full replacement, padded to nuser=2
        assert_eq!(model.jnt_user[j2], vec![300.0, 0.0]);
    }

    /// T6: No-user-data model — all nuser_*=0, empty inner vecs.
    #[test]
    fn t6_no_user_data_model() {
        let model = load_model(
            r#"<mujoco>
            <worldbody>
                <body pos="0 0 1">
                    <joint type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                    <site pos="0 0 0"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_body, 0);
        assert_eq!(model.nuser_jnt, 0);
        assert_eq!(model.nuser_geom, 0);
        assert_eq!(model.nuser_site, 0);
        assert_eq!(model.nuser_tendon, 0);
        assert_eq!(model.nuser_actuator, 0);
        assert_eq!(model.nuser_sensor, 0);
        // Arrays populated but each inner vec is empty
        assert!(model.body_user.iter().all(|u| u.is_empty()));
        assert!(model.jnt_user.iter().all(|u| u.is_empty()));
        assert!(model.geom_user.iter().all(|u| u.is_empty()));
        assert!(model.site_user.iter().all(|u| u.is_empty()));
    }

    /// T7: Sensor user data wired to Model.
    #[test]
    fn t7_sensor_user_data() {
        let model = load_model(
            r#"<mujoco>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1" user="5 10"/>
            </sensor>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_sensor, 2);
        assert_eq!(model.sensor_user[0], vec![5.0, 10.0]);
    }

    /// T8: user="" (not specified → defaults cascade) vs user="0" (data).
    #[test]
    fn t8_empty_string_vs_zero() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <geom user="5 6 7"/>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom name="g1" type="sphere" size="0.1" user=""/>
                    <geom name="g2" type="sphere" size="0.1" user="0"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let g1 = model.geom_name_to_id["g1"];
        let g2 = model.geom_name_to_id["g2"];
        // g1: user="" = not specified → inherits default [5, 6, 7]
        assert_eq!(model.geom_user[g1], vec![5.0, 6.0, 7.0]);
        // g2: user="0" = data → overrides default, padded to nuser=3
        assert_eq!(model.geom_user[g2], vec![0.0, 0.0, 0.0]);
    }

    /// T9: Actuator subtype defaults last-write-wins.
    #[test]
    fn t9_actuator_subtype_defaults() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <general user="100"/>
                <motor user="200"/>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1"/>
            </actuator>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_actuator, 1);
        assert_eq!(model.actuator_user[0], vec![200.0]);
    }

    /// T10: Transitive 3-level default inheritance.
    #[test]
    fn t10_transitive_three_level_inheritance() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <default class="A">
                    <joint user="1 2 3"/>
                    <default class="B">
                        <default class="C">
                        </default>
                    </default>
                </default>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" class="C"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let j1 = model.jnt_name_to_id["j1"];
        assert_eq!(model.jnt_user[j1], vec![1.0, 2.0, 3.0]);
    }

    /// T11: All 7 element types with user data.
    #[test]
    fn t11_all_seven_element_types() {
        let model = load_model(
            r#"<mujoco>
            <worldbody>
                <body name="b1" pos="0 0 1" user="10">
                    <joint name="j1" type="hinge" axis="0 1 0" user="20 21"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom name="g1" type="sphere" size="0.1" user="30 31 32"/>
                    <site name="s1" pos="0 0 0" user="40"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1" user="50 51">
                    <joint joint="j1" coef="1"/>
                </fixed>
            </tendon>
            <actuator>
                <motor name="a1" joint="j1" user="60 61 62 63"/>
            </actuator>
            <sensor>
                <jointpos joint="j1" user="70 71"/>
            </sensor>
            </mujoco>"#,
        )
        .expect("load");
        assert!(model.nuser_body > 0);
        assert!(model.nuser_jnt > 0);
        assert!(model.nuser_geom > 0);
        assert!(model.nuser_site > 0);
        assert!(model.nuser_tendon > 0);
        assert!(model.nuser_actuator > 0);
        assert!(model.nuser_sensor > 0);
        // Verify values
        assert_eq!(model.body_user[1], vec![10.0]);
        let j1 = model.jnt_name_to_id["j1"];
        assert_eq!(model.jnt_user[j1], vec![20.0, 21.0]);
        let g1 = model.geom_name_to_id["g1"];
        assert_eq!(model.geom_user[g1], vec![30.0, 31.0, 32.0]);
        let s1 = model.site_name_to_id["s1"];
        assert_eq!(model.site_user[s1], vec![40.0]);
        assert_eq!(model.tendon_user[0], vec![50.0, 51.0]);
        assert_eq!(model.actuator_user[0], vec![60.0, 61.0, 62.0, 63.0]);
        assert_eq!(model.sensor_user[0], vec![70.0, 71.0]);
    }

    /// T12: Too-long inherited default vs explicit override.
    #[test]
    fn t12_inherited_default_too_long_error() {
        // Sub-test A: inherited default is 3 values, nuser_jnt=2 → error
        let err = load_model(
            r#"<mujoco>
            <size nuser_jnt="2"/>
            <default>
                <joint user="1 2 3"/>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        );
        assert!(
            err.is_err(),
            "should fail: inherited user len 3 > nuser_jnt 2"
        );
    }

    /// T12 sub-test B: explicit override within nuser limit → success.
    #[test]
    fn t12_explicit_override_within_limit() {
        let model = load_model(
            r#"<mujoco>
            <size nuser_jnt="2"/>
            <default>
                <joint user="1 2 3"/>
            </default>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" user="10 20"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let j1 = model.jnt_name_to_id["j1"];
        assert_eq!(model.jnt_user[j1], vec![10.0, 20.0]);
    }

    /// T13: Flexcomp scale + quat transform.
    #[test]
    fn t13_flexcomp_scale_quat_transform() {
        let mjcf = crate::parse_mjcf_str(
            r#"<mujoco>
            <worldbody/>
            <deformable>
                <flexcomp type="grid" count="2 2" spacing="1.0"
                    scale="2 3 1" quat="0.7071068 0 0 0.7071068"/>
            </deformable>
            </mujoco>"#,
        )
        .expect("parse");
        // Grid 2x2 with spacing 1.0 generates (row-major): (0,0,0), (1,0,0), (0,1,0), (1,1,0)
        // Scale (2,3,1): (0,0,0), (2,0,0), (0,3,0), (2,3,0)
        // Quat w=0.707 z=0.707 → 90° rotation about Z: (x,y,z) → (-y,x,z)
        // idx0: (0,0,0)→(0,0,0), idx1: (2,0,0)→(0,2,0),
        // idx2: (0,3,0)→(-3,0,0), idx3: (2,3,0)→(-3,2,0)
        let flex = &mjcf.flex[0];
        assert_eq!(flex.vertices.len(), 4);
        let eps = 1e-6;
        // Compare in generation order (row-major: iy then ix)
        let expected = [
            [0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [-3.0, 0.0, 0.0],
            [-3.0, 2.0, 0.0],
        ];
        for (i, (v, e)) in flex.vertices.iter().zip(expected.iter()).enumerate() {
            assert!(
                (v.x - e[0]).abs() < eps && (v.y - e[1]).abs() < eps && (v.z - e[2]).abs() < eps,
                "vertex {i}: got [{:.6}, {:.6}, {:.6}], expected [{:.6}, {:.6}, {:.6}]",
                v.x,
                v.y,
                v.z,
                e[0],
                e[1],
                e[2]
            );
        }
    }

    /// T14: Flexcomp inertiabox and file parsed.
    #[test]
    fn t14_flexcomp_inertiabox_and_file() {
        let mjcf = crate::parse_mjcf_str(
            r#"<mujoco>
            <worldbody/>
            <deformable>
                <flexcomp type="grid" count="2 2" spacing="1.0"
                    inertiabox="0.5" file="mesh.obj"/>
            </deformable>
            </mujoco>"#,
        )
        .expect("parse");
        let flex = &mjcf.flex[0];
        assert!((flex.inertiabox - 0.5).abs() < 1e-10);
        assert_eq!(flex.flexcomp_file, Some("mesh.obj".to_string()));
    }

    /// T15: User data physics inertness — user data must not affect simulation.
    #[test]
    fn t15_user_data_physics_inertness() {
        let model_a = load_model(
            r#"<mujoco>
            <worldbody>
                <body pos="0 0 1" user="999 888 777">
                    <joint name="j1" type="hinge" axis="0 1 0" user="1 2"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.01"/>
                    <geom type="sphere" size="0.1" user="3 4 5"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1" user="6 7"/>
            </sensor>
            <actuator>
                <motor joint="j1"/>
            </actuator>
            </mujoco>"#,
        )
        .expect("load A");

        let model_b = load_model(
            r#"<mujoco>
            <worldbody>
                <body pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.01"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1"/>
            </sensor>
            <actuator>
                <motor joint="j1"/>
            </actuator>
            </mujoco>"#,
        )
        .expect("load B");

        let mut data_a = model_a.make_data();
        let mut data_b = model_b.make_data();
        for _ in 0..10 {
            data_a.ctrl[0] = 1.0;
            data_b.ctrl[0] = 1.0;
            data_a.step(&model_a).expect("step A");
            data_b.step(&model_b).expect("step B");
        }
        // qpos and qvel must be bit-identical
        assert_eq!(data_a.qpos, data_b.qpos, "qpos diverged with user data");
        assert_eq!(data_a.qvel, data_b.qvel, "qvel diverged with user data");
    }

    /// T16: childclass propagation with nearest-ancestor rule.
    #[test]
    fn t16_childclass_nearest_ancestor() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <default class="A">
                    <geom user="1 2"/>
                </default>
                <default class="B">
                    <geom user="10 20 30"/>
                </default>
            </default>
            <worldbody>
                <body pos="0 0 1" childclass="A">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom name="g1" type="sphere" size="0.1"/>
                    <body pos="0 0 1" childclass="B">
                        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                        <geom name="g2" type="sphere" size="0.1"/>
                    </body>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let g1 = model.geom_name_to_id["g1"];
        let g2 = model.geom_name_to_id["g2"];
        assert_eq!(model.nuser_geom, 3);
        // g1 from class A: [1, 2] padded to nuser=3
        assert_eq!(model.geom_user[g1], vec![1.0, 2.0, 0.0]);
        // g2 from class B: [10, 20, 30]
        assert_eq!(model.geom_user[g2], vec![10.0, 20.0, 30.0]);
    }

    /// T17: childclass does not shadow root default.
    #[test]
    fn t17_childclass_does_not_shadow_root() {
        let model = load_model(
            r#"<mujoco>
            <default>
                <joint user="1 2 3"/>
                <default class="myclass">
                </default>
            </default>
            <worldbody>
                <body pos="0 0 1" childclass="myclass">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        let j1 = model.jnt_name_to_id["j1"];
        // myclass defines no joint user → inherits root default [1, 2, 3]
        assert_eq!(model.jnt_user[j1], vec![1.0, 2.0, 3.0]);
    }

    /// T18: nuser_* validation edge cases.
    #[test]
    fn t18_nuser_validation_edge_cases() {
        // Sub-test A: nuser_geom=-2 → error
        let err = load_model(
            r#"<mujoco>
            <size nuser_geom="-2"/>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        );
        assert!(err.is_err(), "nuser_geom=-2 should be rejected");

        // Sub-test B: nuser_geom=0 with user data → error
        let err = load_model(
            r#"<mujoco>
            <size nuser_geom="0"/>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1" user="1 2 3"/>
                </body>
            </worldbody>
            </mujoco>"#,
        );
        assert!(
            err.is_err(),
            "user data with nuser_geom=0 should be rejected"
        );

        // Sub-test C: nuser_geom=0 with no user data → success
        let model = load_model(
            r#"<mujoco>
            <size nuser_geom="0"/>
            <worldbody>
                <body pos="0 0 1">
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            </mujoco>"#,
        )
        .expect("load");
        assert_eq!(model.nuser_geom, 0);
        assert!(model.geom_user.iter().all(|u| u.is_empty()));
    }
}
