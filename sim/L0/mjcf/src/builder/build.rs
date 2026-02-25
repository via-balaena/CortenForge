//! Final Model assembly.
//!
//! Consumes a fully-populated [`ModelBuilder`] and produces a physics-ready
//! [`Model`]. Handles kinematic tree enumeration, sleep policy resolution,
//! tendon length computation, bounding sphere pre-computation, and all other
//! post-processing that requires the complete builder state.

use nalgebra::DVector;
use sim_core::{
    ActuatorTransmission, ENABLE_SLEEP, Integrator, Model, SleepPolicy, TendonType, WrapType,
    compute_dof_lengths,
};
use tracing::warn;

use super::ModelBuilder;
use super::flex::{
    compute_flex_address_table, compute_flex_count_table, compute_flexedge_crosssection,
};

impl ModelBuilder {
    pub(crate) fn build(self) -> Model {
        let njnt = self.jnt_type.len();
        let nbody = self.body_parent.len();
        let ngeom = self.geom_type.len();
        let nsite = self.site_body.len();
        let nu = self.actuator_trntype.len();
        let ntendon = self.tendon_type.len();

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

        let mut model = Model {
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

            dof_body: self.dof_body,
            dof_jnt: self.dof_jnt,
            dof_parent: self.dof_parent,
            dof_armature: self.dof_armature,
            dof_damping: self.dof_damping,
            dof_frictionloss: self.dof_frictionloss,
            dof_solref: self.dof_solref,
            dof_solimp: self.dof_solimp,

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
            // Pre-computed bounding radii (computed below after we have geom_type and geom_size)
            geom_rbound: vec![0.0; ngeom],
            // Mesh index for each geom (populated by process_geom)
            geom_mesh: self.geom_mesh,
            // Hfield index for each geom (populated by process_geom)
            geom_hfield: self.geom_hfield,
            // SDF index for each geom (always None from MJCF — programmatic only)
            geom_sdf: self.geom_sdf,
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
            flex_edgestiffness: self.flex_edgestiffness,
            flex_edgedamping: self.flex_edgedamping,
            flex_edge_solref: self.flex_edge_solref,
            flex_edge_solimp: self.flex_edge_solimp,
            flex_bend_stiffness: self.flex_bend_stiffness,
            flex_bend_damping: self.flex_bend_damping,
            flex_density: self.flex_density,
            flex_group: self.flex_group,
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
            flexelem_data: self.flexelem_data,
            flexelem_dataadr: self.flexelem_dataadr,
            flexelem_datanum: self.flexelem_datanum,
            flexelem_volume0: self.flexelem_volume0,
            flexelem_flexid: self.flexelem_flexid,
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

            // SDF assets (programmatic — always empty from MJCF)
            nsdf: 0,
            sdf_data: vec![],

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
            tendon_lengthspring: self.tendon_lengthspring,
            tendon_length0: self.tendon_length0,
            tendon_num: self.tendon_num,
            tendon_adr: self.tendon_adr,
            tendon_name: self.tendon_name,
            tendon_group: self.tendon_group,
            tendon_rgba: self.tendon_rgba,
            tendon_solref: self.tendon_solref,
            tendon_solimp: self.tendon_solimp,
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

            // Contact pairs / excludes (populated by process_contact)
            contact_pairs: self.contact_pairs,
            contact_pair_set: self.contact_pair_set,
            contact_excludes: self.contact_excludes,

            timestep: self.timestep,
            gravity: self.gravity,
            qpos0: DVector::from_vec(self.qpos0_values),
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
            disableflags: self.disableflags,
            enableflags: self.enableflags,
            disableactuator: self.disableactuator,
            actuator_group: self.actuator_group,
            integrator: self.integrator,
            solver_type: self.solver_type,

            // Cached implicit integration parameters (computed below)
            implicit_stiffness: DVector::zeros(self.nv),
            implicit_damping: DVector::zeros(self.nv),
            implicit_springref: DVector::zeros(self.nv),

            // Pre-computed kinematic data (will be populated by compute_ancestors)
            body_ancestor_joints: vec![vec![]; nbody],
            body_ancestor_mask: vec![vec![]; nbody], // Multi-word bitmask, computed by compute_ancestors
        };

        // Pre-compute ancestor lists for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters (K, D, q_eq diagonals)
        model.compute_implicit_params();

        // Pre-compute CSR sparsity metadata for sparse LDL factorization.
        // Must be called after dof_parent is finalized and before anything that
        // calls mj_crba (which uses mj_factor_sparse).
        model.compute_qld_csr_metadata();

        // Compute tendon_length0 for spatial tendons (requires FK via mj_fwd_position).
        // Must run before compute_muscle_params() which needs valid tendon_length0.
        model.compute_spatial_tendon_length0();

        // Pre-compute muscle-derived parameters (lengthrange, acc0, F0)
        model.compute_muscle_params();

        // Compute stat_meaninertia = trace(M) / nv at qpos0 (for Newton solver scaling, §15.11)
        model.compute_stat_meaninertia();

        // Pre-compute bounding sphere radii for all geoms (used in collision broad-phase)
        for geom_id in 0..ngeom {
            model.geom_rbound[geom_id] = if let Some(mesh_id) = model.geom_mesh[geom_id] {
                // Mesh geom: bounding sphere radius = distance from AABB center to corner
                // This is the half-diagonal of the AABB, guaranteeing all vertices are inside.
                let (aabb_min, aabb_max) = model.mesh_data[mesh_id].aabb();
                let half_diagonal = (aabb_max - aabb_min) / 2.0;
                half_diagonal.norm()
            } else if let Some(hfield_id) = model.geom_hfield[geom_id] {
                // Hfield geom: half-diagonal of AABB (same pattern as mesh).
                // HeightFieldData::aabb() returns corner-origin bounds; the half-diagonal
                // is origin-independent so the centering offset does not affect it.
                let (aabb_min, aabb_max) = model.hfield_data[hfield_id].aabb();
                let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
                half_diagonal.norm()
            } else if let Some(sdf_id) = model.geom_sdf[geom_id] {
                // SDF geom: half-diagonal of AABB (same pattern as mesh/hfield).
                // SdfCollisionData::aabb() returns (Point3, Point3).
                let (aabb_min, aabb_max) = model.sdf_data[sdf_id].aabb();
                let half_diagonal = (aabb_max.coords - aabb_min.coords) / 2.0;
                half_diagonal.norm()
            } else {
                // Primitive geom: use GeomType::bounding_radius() - the single source of truth
                model.geom_type[geom_id].bounding_radius(model.geom_size[geom_id])
            };
        }

        // Pre-compute tendon length0 and lengthspring from qpos0.
        // For fixed tendons: length = Σ coef_w * qpos0[dof_adr_w].
        for t in 0..model.ntendon {
            if model.tendon_type[t] == TendonType::Fixed {
                let adr = model.tendon_adr[t];
                let num = model.tendon_num[t];
                let mut length = 0.0;
                for w in adr..(adr + num) {
                    let dof_adr = model.wrap_objid[w];
                    let coef = model.wrap_prm[w];
                    if dof_adr < model.qpos0.len() {
                        length += coef * model.qpos0[dof_adr];
                    }
                }
                model.tendon_length0[t] = length;
                // S3: Replace sentinel [-1, -1] with computed length at qpos0
                // (MuJoCo uses qpos_spring; see S3 divergence note).
                // Unconditional — MuJoCo resolves sentinel regardless of stiffness.
                // Sentinel is an exact literal, never a computed float.
                #[allow(clippy::float_cmp)]
                if model.tendon_lengthspring[t] == [-1.0, -1.0] {
                    model.tendon_lengthspring[t] = [length, length];
                }
            }
        }

        // ===== Kinematic Tree Enumeration (§16.0) =====
        // Group bodies by body_rootid to discover kinematic trees.
        // Body 0 (world) is excluded — it is its own tree but never sleeps.
        {
            use std::collections::BTreeMap;
            let mut trees: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for body_id in 1..nbody {
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

            // (§27F) Flex vertex DOFs are now real body DOFs in the kinematic tree.
            // Their tree IDs are set by the standard tree-building loop above.
            // Pinned vertices (dofadr == usize::MAX) have no DOFs — nothing to set.

            // ===== Tendon Tree Mapping (§16.10.1) =====
            // Compute tendon_treenum/tendon_tree by scanning each tendon's waypoints.
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

            // ===== Sleep Policy Resolution (§16.0 steps 1-3) =====
            // Step 2: Mark trees with actuators as AutoNever
            for act_id in 0..nu {
                let trn = model.actuator_trntype[act_id];
                let trnid = model.actuator_trnid[act_id];
                let body_id = match trn {
                    ActuatorTransmission::Joint => {
                        // trnid[0] is the joint index
                        if trnid[0] < model.njnt {
                            Some(model.jnt_body[trnid[0]])
                        } else {
                            None
                        }
                    }
                    ActuatorTransmission::Tendon => {
                        // §16.26.5: Tendon-actuator policy resolution.
                        // Mark all trees spanned by the tendon as AutoNever.
                        let tendon_idx = trnid[0];
                        if tendon_idx < model.ntendon {
                            let wrap_start = model.tendon_adr[tendon_idx];
                            let wrap_count = model.tendon_num[tendon_idx];
                            for w in wrap_start..wrap_start + wrap_count {
                                let bid = match model.wrap_type[w] {
                                    WrapType::Joint => {
                                        // wrap_objid is dof_adr for fixed tendon joints
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
                                    WrapType::Pulley => None, // No body
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
                        None // body_id not used below — trees already marked above
                    }
                    ActuatorTransmission::Site => {
                        // trnid[0] is the site index
                        if trnid[0] < model.nsite {
                            Some(model.site_body[trnid[0]])
                        } else {
                            None
                        }
                    }
                    ActuatorTransmission::Body => {
                        // trnid[0] is the body index directly
                        let bid = trnid[0];
                        if bid > 0 && bid < model.nbody {
                            Some(bid)
                        } else {
                            None
                        }
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
            // Passive multi-tree tendons (spanning 2 trees) with nonzero stiffness,
            // damping, or active limits create inter-tree coupling forces that prevent
            // independent sleeping. Mark their spanning trees as AutoNever.
            // Zero-stiffness/zero-damping/unlimited tendons are purely geometric
            // (observational) and allow sleep.
            for t in 0..model.ntendon {
                if model.tendon_treenum[t] < 2 {
                    continue; // Single-tree or zero-tree tendon — no coupling
                }
                let has_stiffness = model.tendon_stiffness[t].abs() > 0.0;
                let has_damping = model.tendon_damping[t].abs() > 0.0;
                let has_limit = model.tendon_limited[t];
                if has_stiffness || has_damping || has_limit {
                    // This tendon creates passive inter-tree forces → AutoNever
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

            // §16.26.4: Flex body policy guard. Flex DOFs are assigned
            // dof_treeid = usize::MAX (permanently awake) so they don't participate
            // in the tree-level sleep system. If flex vertices are ever assigned to
            // specific kinematic trees, their trees should be marked AutoNever.

            // Step 2b: Apply explicit body-level sleep policies from MJCF
            for body_id in 1..nbody {
                if let Some(policy) = self.body_sleep_policy[body_id] {
                    let tree = model.body_treeid[body_id];
                    if tree < model.ntree {
                        // Warn if set on non-root body (propagates to tree root)
                        let root_body = model.tree_body_adr[tree];
                        if body_id != root_body {
                            let body_name =
                                model.body_name[body_id].as_deref().unwrap_or("unnamed");
                            warn!(
                                "body '{}': sleep attribute on non-root body propagates to tree root",
                                body_name
                            );
                        }
                        // Explicit policy overrides automatic resolution
                        model.tree_sleep_policy[tree] = policy;
                    }
                }
            }

            // Step 3: Convert remaining Auto to AutoAllowed
            for t in 0..model.ntree {
                if model.tree_sleep_policy[t] == SleepPolicy::Auto {
                    model.tree_sleep_policy[t] = SleepPolicy::AutoAllowed;
                }
            }

            // ===== dof_length Computation (§16.14) =====
            // Compute mechanism lengths: rotational DOFs get the body's subtree
            // extent (converts rad/s to m/s at tip), translational DOFs get 1.0.
            compute_dof_lengths(&mut model);

            // ===== RK4 Incompatibility Guard (§16.6) =====
            if model.enableflags & ENABLE_SLEEP != 0 && model.integrator == Integrator::RungeKutta4
            {
                warn!("Sleeping is incompatible with RK4 integrator. Disabling sleep.");
                model.enableflags &= !ENABLE_SLEEP;
            }
        }

        model
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
}
