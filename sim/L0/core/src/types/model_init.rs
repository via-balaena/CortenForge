//! Model construction and precomputation methods.
//!
//! This module contains [`Model::empty()`], [`Model::make_data()`], and the
//! precomputation methods called at model build time: ancestor computation,
//! implicit integration parameter caching, mean-inertia statistics, and
//! per-DOF mechanism length computation (§16.14).

use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::enums::{Integrator, MIN_AWAKE, MjJointType, SleepPolicy, SleepState, SolverType};
use super::model::Model;

// Types from dynamics module (Phase 7 extraction)
use crate::dynamics::SpatialVector;
use crate::dynamics::crba::{DEFAULT_MASS_FALLBACK, mj_crba};

use super::data::Data;
use crate::forward::mj_fwd_position;
use crate::island::{mj_update_sleep_arrays, reset_sleep_state};

impl Model {
    /// Create an empty model with no bodies/joints.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            // Metadata
            name: String::new(),

            // Dimensions
            nq: 0,
            nv: 0,
            nbody: 1, // World body 0 always exists
            njnt: 0,
            ngeom: 0,
            nsite: 0,
            nu: 0,
            na: 0,
            nmocap: 0,
            nkeyframe: 0,

            // Flex dimensions (empty)
            nflex: 0,
            nflexvert: 0,
            nflexedge: 0,
            nflexelem: 0,
            nflexhinge: 0,
            // Kinematic trees (§16.0) — empty model has no trees
            ntree: 0,
            tree_body_adr: vec![],
            tree_body_num: vec![],
            tree_dof_adr: vec![],
            tree_dof_num: vec![],
            body_treeid: vec![usize::MAX], // World body sentinel
            dof_treeid: vec![],
            tree_sleep_policy: vec![],
            dof_length: vec![],
            sleep_tolerance: 1e-4,

            // Body tree (initialize world body)
            body_parent: vec![0], // World is its own parent
            body_rootid: vec![0],
            body_jnt_adr: vec![0],
            body_jnt_num: vec![0],
            body_dof_adr: vec![0],
            body_dof_num: vec![0],
            body_geom_adr: vec![0],
            body_geom_num: vec![0],

            // Body properties
            body_pos: vec![Vector3::zeros()],
            body_quat: vec![UnitQuaternion::identity()],
            body_ipos: vec![Vector3::zeros()],
            body_iquat: vec![UnitQuaternion::identity()],
            body_mass: vec![0.0], // World has no mass
            body_inertia: vec![Vector3::zeros()],
            body_name: vec![Some("world".to_string())],
            body_subtreemass: vec![0.0], // World subtree mass (will be total system mass)
            body_mocapid: vec![None],    // world body
            body_gravcomp: vec![0.0],    // world body has no gravcomp
            ngravcomp: 0,

            // Joints (empty)
            jnt_type: vec![],
            jnt_body: vec![],
            jnt_qpos_adr: vec![],
            jnt_dof_adr: vec![],
            jnt_pos: vec![],
            jnt_axis: vec![],
            jnt_limited: vec![],
            jnt_range: vec![],
            jnt_stiffness: vec![],
            jnt_springref: vec![],
            jnt_damping: vec![],
            jnt_armature: vec![],
            jnt_solref: vec![],
            jnt_solimp: vec![],
            jnt_name: vec![],
            jnt_group: vec![],
            jnt_actgravcomp: vec![],

            // DOFs (empty)
            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],
            dof_frictionloss: vec![],
            dof_solref: vec![],
            dof_solimp: vec![],

            // Sparse LDL CSR metadata (empty — populated by compute_qld_csr_metadata)
            qLD_rowadr: vec![],
            qLD_rownnz: vec![],
            qLD_colind: vec![],
            qLD_nnz: 0,

            // Geoms (empty)
            geom_type: vec![],
            geom_body: vec![],
            geom_pos: vec![],
            geom_quat: vec![],
            geom_size: vec![],
            geom_friction: vec![],
            geom_condim: vec![],
            geom_contype: vec![],
            geom_conaffinity: vec![],
            geom_margin: vec![],
            geom_gap: vec![],
            geom_priority: vec![],
            geom_solmix: vec![],
            geom_solimp: vec![],
            geom_solref: vec![],
            geom_name: vec![],
            geom_rbound: vec![],
            geom_mesh: vec![],
            geom_hfield: vec![],
            geom_sdf: vec![],
            geom_group: vec![],
            geom_rgba: vec![],
            geom_fluid: vec![],

            // Flex bodies (empty)
            flex_dim: vec![],
            flex_vertadr: vec![],
            flex_vertnum: vec![],
            flex_edgeadr: vec![],
            flex_edgenum: vec![],
            flex_elemadr: vec![],
            flex_elemnum: vec![],
            flex_young: vec![],
            flex_poisson: vec![],
            flex_damping: vec![],
            flex_thickness: vec![],
            flex_friction: vec![],
            flex_solref: vec![],
            flex_solimp: vec![],
            flex_condim: vec![],
            flex_margin: vec![],
            flex_gap: vec![],
            flex_priority: vec![],
            flex_solmix: vec![],
            flex_contype: vec![],
            flex_conaffinity: vec![],
            flex_selfcollide: vec![],
            flex_edgestiffness: vec![],
            flex_edgedamping: vec![],
            flex_edge_solref: vec![],
            flex_edge_solimp: vec![],
            flex_bend_stiffness: vec![],
            flex_bend_damping: vec![],
            flex_density: vec![],
            flex_group: vec![],
            flexvert_qposadr: vec![],
            flexvert_dofadr: vec![],
            flexvert_mass: vec![],
            flexvert_invmass: vec![],
            flexvert_radius: vec![],
            flexvert_flexid: vec![],
            flexvert_bodyid: vec![],
            flexedge_vert: vec![],
            flexedge_length0: vec![],
            flexedge_crosssection: vec![],
            flexedge_flexid: vec![],
            flexelem_data: vec![],
            flexelem_dataadr: vec![],
            flexelem_datanum: vec![],
            flexelem_volume0: vec![],
            flexelem_flexid: vec![],
            flexhinge_vert: vec![],
            flexhinge_angle0: vec![],
            flexhinge_flexid: vec![],

            // Meshes (empty)
            nmesh: 0,
            mesh_name: vec![],
            mesh_data: vec![],

            // Height fields (empty)
            nhfield: 0,
            hfield_name: vec![],
            hfield_data: vec![],
            hfield_size: vec![],

            // SDFs (empty)
            nsdf: 0,
            sdf_data: vec![],

            // Sites (empty)
            site_body: vec![],
            site_type: vec![],
            site_pos: vec![],
            site_quat: vec![],
            site_size: vec![],
            site_name: vec![],
            site_group: vec![],
            site_rgba: vec![],

            // Sensors (empty)
            nsensor: 0,
            nsensordata: 0,
            sensor_type: vec![],
            sensor_datatype: vec![],
            sensor_objtype: vec![],
            sensor_objid: vec![],
            sensor_reftype: vec![],
            sensor_refid: vec![],
            sensor_adr: vec![],
            sensor_dim: vec![],
            sensor_noise: vec![],
            sensor_cutoff: vec![],
            sensor_name: vec![],

            // Actuators (empty)
            actuator_trntype: vec![],
            actuator_dyntype: vec![],
            actuator_trnid: vec![],
            actuator_gear: vec![],
            actuator_ctrlrange: vec![],
            actuator_forcerange: vec![],
            actuator_name: vec![],
            actuator_act_adr: vec![],
            actuator_act_num: vec![],
            actuator_gaintype: vec![],
            actuator_biastype: vec![],
            actuator_dynprm: vec![],
            actuator_gainprm: vec![],
            actuator_biasprm: vec![],
            actuator_lengthrange: vec![],
            actuator_acc0: vec![],
            actuator_actlimited: vec![],
            actuator_actrange: vec![],
            actuator_actearly: vec![],
            actuator_cranklength: vec![],

            // Tendons (empty)
            ntendon: 0,
            nwrap: 0,
            tendon_range: vec![],
            tendon_limited: vec![],
            tendon_stiffness: vec![],
            tendon_damping: vec![],
            tendon_lengthspring: vec![],
            tendon_length0: vec![],
            tendon_num: vec![],
            tendon_adr: vec![],
            tendon_name: vec![],
            tendon_type: vec![],
            tendon_solref: vec![],
            tendon_solimp: vec![],
            tendon_frictionloss: vec![],
            tendon_solref_fri: vec![],
            tendon_solimp_fri: vec![],
            tendon_group: vec![],
            tendon_rgba: vec![],
            tendon_treenum: vec![],
            tendon_tree: vec![],
            wrap_type: vec![],
            wrap_objid: vec![],
            wrap_prm: vec![],
            wrap_sidesite: vec![],

            // Equality constraints (empty)
            neq: 0,
            eq_type: vec![],
            eq_obj1id: vec![],
            eq_obj2id: vec![],
            eq_data: vec![],
            eq_active: vec![],
            eq_solimp: vec![],
            eq_solref: vec![],
            eq_name: vec![],

            // Options (MuJoCo defaults)
            timestep: 0.002,                        // 500 Hz
            gravity: Vector3::new(0.0, 0.0, -9.81), // Z-up
            qpos0: DVector::zeros(0),
            // Keyframes
            keyframes: Vec::new(),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,   // No fluid by default
            viscosity: 0.0, // No fluid by default
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,              // MuJoCo default
            regularization: 1e-6,       // PGS constraint softness
            friction_smoothing: 1000.0, // tanh transition sharpness
            cone: 0,                    // Pyramidal friction cone (MuJoCo default)
            stat_meaninertia: 1.0,      // Default (computed at model build from CRBA)
            ls_iterations: 50,          // Newton line search iterations
            ls_tolerance: 0.01,         // Newton line search gradient tolerance
            noslip_iterations: 0,       // Default: no noslip post-processing
            noslip_tolerance: 1e-6,     // Default tolerance for noslip convergence
            disableflags: 0,            // Nothing disabled
            enableflags: 0,             // MuJoCo default: all enable bits clear
            disableactuator: 0,         // No actuator groups disabled
            actuator_group: Vec::new(), // All actuators in group 0 (empty for empty model)
            o_margin: 0.0,
            o_solref: [0.02, 1.0],
            o_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            o_friction: [1.0, 1.0, 0.005, 0.0001, 0.0001],
            integrator: Integrator::Euler,
            solver_type: SolverType::PGS,

            // Cached implicit integration parameters (empty for empty model)
            implicit_stiffness: DVector::zeros(0),
            implicit_damping: DVector::zeros(0),
            implicit_springref: DVector::zeros(0),

            // Pre-computed kinematic data (world body has no ancestors)
            body_ancestor_joints: vec![vec![]],
            body_ancestor_mask: vec![vec![]], // Empty vec for world body (no joints yet)

            // Name↔index lookup (§59) — empty maps for empty model
            body_name_to_id: HashMap::new(),
            jnt_name_to_id: HashMap::new(),
            geom_name_to_id: HashMap::new(),
            site_name_to_id: HashMap::new(),
            tendon_name_to_id: HashMap::new(),
            actuator_name_to_id: HashMap::new(),
            sensor_name_to_id: HashMap::new(),
            mesh_name_to_id: HashMap::new(),
            hfield_name_to_id: HashMap::new(),
            eq_name_to_id: HashMap::new(),

            // Contact pairs / excludes (empty = no explicit pairs or excludes)
            contact_pairs: vec![],
            contact_pair_set: HashSet::new(),
            contact_excludes: HashSet::new(),

            // User callbacks (DT-79) — default: no callbacks
            cb_passive: None,
            cb_control: None,
            cb_contactfilter: None,
            cb_sensor: None,
            cb_act_dyn: None,
            cb_act_gain: None,
            cb_act_bias: None,
        }
    }

    /// Create initial Data struct for this model with all arrays pre-allocated.
    #[must_use]
    pub fn make_data(&self) -> Data {
        let mut data = Data {
            // Generalized coordinates
            qpos: self.qpos0.clone(),
            qvel: DVector::zeros(self.nv),
            qacc: DVector::zeros(self.nv),
            qacc_warmstart: DVector::zeros(self.nv),

            // Actuation
            ctrl: DVector::zeros(self.nu),
            act: DVector::zeros(self.na),
            qfrc_actuator: DVector::zeros(self.nv),
            actuator_length: vec![0.0; self.nu],
            actuator_velocity: vec![0.0; self.nu],
            actuator_force: vec![0.0; self.nu],
            actuator_moment: vec![DVector::zeros(self.nv); self.nu],
            act_dot: DVector::zeros(self.na),

            // Body states
            xpos: vec![Vector3::zeros(); self.nbody],
            xquat: vec![UnitQuaternion::identity(); self.nbody],
            xmat: vec![Matrix3::identity(); self.nbody],
            xipos: vec![Vector3::zeros(); self.nbody],
            ximat: vec![Matrix3::identity(); self.nbody],

            // Mocap bodies (default to body_pos/body_quat for each mocap body)
            mocap_pos: if self.nmocap == 0 {
                Vec::new()
            } else {
                self.body_mocapid
                    .iter()
                    .enumerate()
                    .filter_map(|(i, mid)| mid.map(|_| self.body_pos[i]))
                    .collect()
            },
            mocap_quat: if self.nmocap == 0 {
                Vec::new()
            } else {
                self.body_mocapid
                    .iter()
                    .enumerate()
                    .filter_map(|(i, mid)| mid.map(|_| self.body_quat[i]))
                    .collect()
            },

            // Geom poses
            geom_xpos: vec![Vector3::zeros(); self.ngeom],
            geom_xmat: vec![Matrix3::identity(); self.ngeom],

            // Site poses
            site_xpos: vec![Vector3::zeros(); self.nsite],
            site_xmat: vec![Matrix3::identity(); self.nsite],
            site_xquat: vec![UnitQuaternion::identity(); self.nsite],

            // Flex vertex poses
            flexvert_xpos: vec![Vector3::zeros(); self.nflexvert],

            // Velocities
            cvel: vec![SpatialVector::zeros(); self.nbody],
            cdof: vec![SpatialVector::zeros(); self.nv],

            // RNE intermediate quantities
            cacc_bias: vec![SpatialVector::zeros(); self.nbody],
            cfrc_bias: vec![SpatialVector::zeros(); self.nbody],

            // Forces
            qfrc_applied: DVector::zeros(self.nv),
            qfrc_bias: DVector::zeros(self.nv),
            qfrc_passive: DVector::zeros(self.nv),
            qfrc_spring: DVector::zeros(self.nv),
            qfrc_damper: DVector::zeros(self.nv),
            qfrc_fluid: DVector::zeros(self.nv),
            qfrc_gravcomp: DVector::zeros(self.nv),
            qfrc_constraint: DVector::zeros(self.nv),
            jnt_limit_frc: vec![0.0; self.njnt],
            ten_limit_frc: vec![0.0; self.ntendon],
            xfrc_applied: vec![SpatialVector::zeros(); self.nbody],

            // Mass matrix (dense)
            qM: DMatrix::zeros(self.nv, self.nv),

            // Mass matrix (sparse L^T D L — computed in mj_crba via mj_factor_sparse)
            qLD_data: vec![0.0; self.qLD_nnz],
            qLD_diag_inv: vec![0.0; self.nv],
            qLD_valid: false,

            // Body spatial inertias (computed once in FK, used by CRBA and RNE)
            cinert: vec![Matrix6::zeros(); self.nbody],
            // Composite rigid body inertias (for Featherstone CRBA)
            crb_inertia: vec![Matrix6::zeros(); self.nbody],

            // Subtree mass/COM/velocity
            subtree_mass: vec![0.0; self.nbody],
            subtree_com: vec![Vector3::zeros(); self.nbody],
            subtree_linvel: vec![Vector3::zeros(); self.nbody],
            subtree_angmom: vec![Vector3::zeros(); self.nbody],
            flg_subtreevel: false,

            // Tendon state
            ten_length: vec![0.0; self.ntendon],
            ten_velocity: vec![0.0; self.ntendon],
            ten_force: vec![0.0; self.ntendon],
            ten_J: vec![DVector::zeros(self.nv); self.ntendon],

            // Tendon wrap visualization data
            wrap_xpos: vec![Vector3::zeros(); self.nwrap * 2],
            wrap_obj: vec![0i32; self.nwrap * 2],
            ten_wrapadr: vec![0usize; self.ntendon],
            ten_wrapnum: vec![0usize; self.ntendon],

            // Equality constraint state
            eq_violation: vec![0.0; self.neq * 6], // max 6 DOF per constraint (weld)
            eq_force: vec![0.0; self.neq * 6],

            // Contacts
            contacts: Vec::with_capacity(256), // Pre-allocate typical capacity
            ncon: 0,

            // Solver state
            solver_niter: 0,
            solver_nnz: 0,

            // Unified constraint system (Newton solver) — initially empty (zero-length).
            // Populated by assemble_unified_constraints() only when Newton solver is active.
            qfrc_frictionloss: DVector::zeros(self.nv),
            qacc_smooth: DVector::zeros(self.nv),
            qfrc_smooth: DVector::zeros(self.nv),
            efc_b: DVector::zeros(0),
            efc_J: DMatrix::zeros(0, self.nv),
            efc_type: Vec::new(),
            efc_pos: Vec::new(),
            efc_margin: Vec::new(),
            efc_vel: DVector::zeros(0),
            efc_solref: Vec::new(),
            efc_solimp: Vec::new(),
            efc_diagApprox: Vec::new(),
            efc_R: Vec::new(),
            efc_D: Vec::new(),
            efc_imp: Vec::new(),
            efc_aref: DVector::zeros(0),
            efc_floss: Vec::new(),
            efc_mu: Vec::new(),
            efc_dim: Vec::new(),
            efc_id: Vec::new(),
            efc_state: Vec::new(),
            efc_force: DVector::zeros(0),
            efc_jar: DVector::zeros(0),
            efc_cost: 0.0,
            efc_cone_hessian: Vec::new(),
            ncone: 0,
            ne: 0,
            nf: 0,
            newton_solved: false,
            solver_stat: Vec::new(),
            stat_meaninertia: 1.0,

            // Sensors
            sensordata: DVector::zeros(self.nsensordata),

            // Warnings
            warnings: [super::warning::WarningStat::default(); super::warning::NUM_WARNINGS],

            // Energy
            energy_potential: 0.0,
            energy_kinetic: 0.0,
            solver_fwdinv: [0.0, 0.0],

            // Sleep state (§16.7) — initialized from tree sleep policies.
            // For models not built through MJCF (ntree == 0 or body_treeid not populated),
            // all bodies start awake and sleep is effectively a no-op.
            #[allow(clippy::needless_range_loop)]
            tree_asleep: {
                let mut v = vec![-(1 + MIN_AWAKE); self.ntree];
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
                        {
                            v[t] = t as i32; // Start asleep (ntree ≤ nbody ≪ i32::MAX)
                        }
                    }
                }
                v
            },
            #[allow(clippy::needless_range_loop)]
            tree_awake: {
                let mut v = vec![true; self.ntree];
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        v[t] = false;
                    }
                }
                v
            },
            #[allow(clippy::needless_range_loop)]
            body_sleep_state: {
                let mut v = vec![SleepState::Awake; self.nbody];
                if self.nbody > 0 {
                    v[0] = SleepState::Static; // World body
                }
                // Mark bodies in Init trees as Asleep (only if tree enumeration was run)
                if self.body_treeid.len() == self.nbody {
                    for body_id in 1..self.nbody {
                        let tree = self.body_treeid[body_id];
                        if tree < self.ntree && self.tree_sleep_policy[tree] == SleepPolicy::Init {
                            v[body_id] = SleepState::Asleep;
                        }
                    }
                }
                v
            },
            ntree_awake: {
                let mut count = self.ntree;
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        count -= 1;
                    }
                }
                count
            },
            nv_awake: {
                let mut count = self.nv;
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        count -= self.tree_dof_num[t];
                    }
                }
                count
            },

            // Awake-index indirection arrays (§16.17).
            // Allocated to worst-case size; populated by mj_update_sleep_arrays().
            body_awake_ind: vec![0; self.nbody],
            nbody_awake: 0, // Set by mj_update_sleep_arrays below
            parent_awake_ind: vec![0; self.nbody],
            nparent_awake: 0,
            dof_awake_ind: vec![0; self.nv],

            // Island discovery arrays (§16.11) — worst-case: each tree is its own island.
            nisland: 0,
            tree_island: vec![-1_i32; self.ntree],
            island_ntree: vec![0; self.ntree],
            island_itreeadr: vec![0; self.ntree],
            map_itree2tree: vec![0; self.ntree],
            dof_island: vec![-1_i32; self.nv],
            island_nv: vec![0; self.ntree],
            island_idofadr: vec![0; self.ntree],
            map_dof2idof: vec![-1_i32; self.nv],
            map_idof2dof: vec![0; self.nv],
            efc_island: Vec::new(),
            island_nefc: vec![0; self.ntree],
            island_iefcadr: vec![0; self.ntree],
            map_efc2iefc: Vec::new(),
            map_iefc2efc: Vec::new(),
            contact_island: Vec::new(),

            // Island scratch space (§16.11)
            island_scratch_stack: vec![0; self.ntree],
            island_scratch_rownnz: vec![0; self.ntree],
            island_scratch_rowadr: vec![0; self.ntree],
            island_scratch_colind: Vec::new(),

            // qpos change detection (§16.15)
            tree_qpos_dirty: vec![false; self.ntree],

            // Time
            time: 0.0,

            // Scratch buffers (pre-allocated for allocation-free stepping)
            scratch_m_impl: DMatrix::zeros(self.nv, self.nv),
            scratch_force: DVector::zeros(self.nv),
            scratch_rhs: DVector::zeros(self.nv),
            scratch_v_new: DVector::zeros(self.nv),
            scratch_lu_piv: vec![0; self.nv],

            // RK4 scratch buffers
            rk4_qpos_saved: DVector::zeros(self.nq),
            rk4_qpos_stage: DVector::zeros(self.nq),
            rk4_qvel: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_qacc: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_dX_vel: DVector::zeros(self.nv),
            rk4_dX_acc: DVector::zeros(self.nv),
            rk4_act_saved: DVector::zeros(self.na),
            rk4_act_dot: std::array::from_fn(|_| DVector::zeros(self.na)),

            // Derivative scratch buffers (for mjd_smooth_vel / mjd_rne_vel)
            qDeriv: DMatrix::zeros(self.nv, self.nv),
            deriv_Dcvel: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcacc: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcfrc: vec![DMatrix::zeros(6, self.nv); self.nbody],

            // Inverse dynamics (§52)
            qfrc_inverse: DVector::zeros(self.nv),

            // Body force accumulators (§51)
            cacc: vec![SpatialVector::zeros(); self.nbody],
            cfrc_int: vec![SpatialVector::zeros(); self.nbody],
            cfrc_ext: vec![SpatialVector::zeros(); self.nbody],
            flg_rnepost: false,

            // Cached body mass/inertia (computed in forward() after CRBA)
            // Initialize world body (index 0) to infinity, others to default
            body_min_mass: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
            body_min_inertia: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
        };

        // Run initial FK to populate body/geom/site positions from qpos0.
        // This must happen BEFORE sleep gating takes effect so that Init-asleep
        // bodies have correct world positions. We temporarily mark all bodies
        // as Awake, run FK, then restore the Init-sleep state.
        // Only needed when there are Init-asleep trees; other models get FK
        // from their first forward()/step() call.
        let has_init_asleep = (0..self.ntree).any(|t| data.tree_asleep[t] >= 0);
        if has_init_asleep {
            // Temporarily wake all bodies for FK + CRBA
            let saved_body_sleep = data.body_sleep_state.clone();
            let saved_tree_asleep = data.tree_asleep.clone();
            let saved_tree_awake = data.tree_awake.clone();
            for b in 1..self.nbody {
                data.body_sleep_state[b] = SleepState::Awake;
            }
            for t in 0..self.ntree {
                data.tree_awake[t] = true;
                data.tree_asleep[t] = -(1 + MIN_AWAKE);
            }
            // Temporarily update awake-index arrays so CRBA's sleep_filter
            // evaluates to false (all bodies awake).
            mj_update_sleep_arrays(self, &mut data);

            // Run FK and CRBA to populate body positions and mass matrix.
            // CRBA is required so that Init-asleep bodies have valid qM
            // entries before selective CRBA (§16.29.3) begins preserving them.
            mj_fwd_position(self, &mut data);
            mj_crba(self, &mut data);

            // Restore sleep states
            data.body_sleep_state = saved_body_sleep;
            data.tree_asleep = saved_tree_asleep;
            data.tree_awake = saved_tree_awake;
        }

        // Initialize sleep state with union-find validation (§16.24).
        // This replaces the inline Init-sleep self-links with proper
        // island-aware sleep cycles.
        reset_sleep_state(self, &mut data);

        data
    }

    /// Compute pre-computed kinematic data (ancestor lists and masks).
    ///
    /// This must be called after the model topology is finalized. It builds:
    /// - `body_ancestor_joints`: For each body, the list of all ancestor joints
    /// - `body_ancestor_mask`: Multi-word bitmask for O(1) ancestor testing
    ///
    /// These enable O(n) CRBA/RNE algorithms instead of O(n³).
    ///
    /// Following `MuJoCo`'s principle: heavy computation at model load time,
    /// minimal computation at simulation time.
    pub fn compute_ancestors(&mut self) {
        // Number of u64 words needed for the bitmask
        #[allow(clippy::manual_div_ceil)]
        let num_words = (self.njnt + 63) / 64; // ceil(njnt / 64)

        // Clear and resize
        self.body_ancestor_joints = vec![vec![]; self.nbody];
        self.body_ancestor_mask = vec![vec![0u64; num_words]; self.nbody];

        // For each body, walk up to root collecting ancestor joints
        for body_id in 1..self.nbody {
            let mut current = body_id;
            while current != 0 {
                // Add joints attached to this body
                let jnt_start = self.body_jnt_adr[current];
                let jnt_end = jnt_start + self.body_jnt_num[current];
                for jnt_id in jnt_start..jnt_end {
                    self.body_ancestor_joints[body_id].push(jnt_id);
                    // Set bit in multi-word mask (supports unlimited joints)
                    let word = jnt_id / 64;
                    let bit = jnt_id % 64;
                    self.body_ancestor_mask[body_id][word] |= 1u64 << bit;
                }
                current = self.body_parent[current];
            }
        }
    }

    /// Compute cached implicit integration parameters from joint parameters.
    ///
    /// This expands per-joint K/D/springref into per-DOF vectors used by
    /// implicit integration. Must be called after all joints are added.
    ///
    /// For Hinge/Slide joints: `K[dof]` = jnt_stiffness, `D[dof]` = jnt_damping
    /// For Ball/Free joints: `K[dof]` = 0, `D[dof]` = dof_damping (per-DOF)
    pub fn compute_implicit_params(&mut self) {
        // Resize to nv DOFs
        self.implicit_stiffness = DVector::zeros(self.nv);
        self.implicit_damping = DVector::zeros(self.nv);
        self.implicit_springref = DVector::zeros(self.nv);

        for jnt_id in 0..self.njnt {
            let dof_adr = self.jnt_dof_adr[jnt_id];
            let jnt_type = self.jnt_type[jnt_id];
            let nv_jnt = jnt_type.nv();

            match jnt_type {
                MjJointType::Hinge | MjJointType::Slide => {
                    self.implicit_stiffness[dof_adr] = self.jnt_stiffness[jnt_id];
                    self.implicit_damping[dof_adr] = self.jnt_damping[jnt_id];
                    self.implicit_springref[dof_adr] = self.jnt_springref[jnt_id];
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free: per-DOF damping only (no spring for quaternion DOFs)
                    for i in 0..nv_jnt {
                        let dof_idx = dof_adr + i;
                        self.implicit_stiffness[dof_idx] = 0.0;
                        self.implicit_damping[dof_idx] = self.dof_damping[dof_idx];
                        self.implicit_springref[dof_idx] = 0.0;
                    }
                }
            }
        }
    }

    /// Compute `stat_meaninertia = trace(M) / nv` at `qpos0`.
    ///
    /// Creates a temporary Data, runs FK + CRBA to fill qM, then takes the
    /// trace of the mass matrix divided by nv. Guards nv == 0 → 1.0.
    /// Called once at model build time (§15.11).
    pub fn compute_stat_meaninertia(&mut self) {
        if self.nv == 0 {
            self.stat_meaninertia = 1.0;
            return;
        }
        let mut data = self.make_data();
        mj_fwd_position(self, &mut data);
        mj_crba(self, &mut data);
        // (§27F) Pinned flex vertices now have no DOFs — no need to skip them.
        let mut trace = 0.0_f64;
        for i in 0..self.nv {
            trace += data.qM[(i, i)];
        }
        #[allow(clippy::cast_precision_loss)]
        let mean = if self.nv > 0 {
            trace / self.nv as f64
        } else {
            1.0
        };
        self.stat_meaninertia = mean;
        // Guard against degenerate models with zero inertia
        if self.stat_meaninertia <= 0.0 {
            self.stat_meaninertia = 1.0;
        }
    }
}

/// Compute characteristic body length for dof_length normalization (§16.14.1).
///
/// For each body, compute the maximum extent from this body through the
/// kinematic chain to any descendant. This gives a length scale that converts
/// angular velocity [rad/s] to tip velocity [m/s] for the mechanism rooted
/// at this body.
fn compute_body_lengths(model: &Model) -> Vec<f64> {
    let mut body_length = vec![0.0_f64; model.nbody];

    // Backward pass: accumulate subtree extents from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent = model.body_parent[body_id];

        // Distance from parent to this body (local position in parent frame)
        let pos = &model.body_pos[body_id];
        let dist = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();

        // This body's extent: own subtree extent + distance to parent
        let child_extent = body_length[body_id] + dist;
        body_length[parent] = body_length[parent].max(child_extent);
    }

    // Ensure minimum length (no normalization for tiny/zero-extent bodies)
    for length in &mut body_length {
        if *length < 1e-10 {
            *length = 1.0;
        }
    }

    body_length
}

/// Compute per-DOF mechanism lengths (§16.14.2).
///
/// Rotational DOFs get the body length (converts rad/s to m/s at the tip).
/// Translational DOFs keep 1.0 (already in m/s).
///
/// Called during model construction to replace the Phase A uniform 1.0.
pub fn compute_dof_lengths(model: &mut Model) {
    let body_length = compute_body_lengths(model);

    // (§27F) All DOFs now have real joints — iterate all DOFs uniformly.
    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let jnt_type = model.jnt_type[jnt_id];
        let offset = dof - model.jnt_dof_adr[jnt_id];

        let is_rotational = match jnt_type {
            MjJointType::Hinge | MjJointType::Ball => true,
            MjJointType::Free => offset >= 3, // DOFs 3,4,5 are rotational
            MjJointType::Slide => false,
        };

        if is_rotational {
            model.dof_length[dof] = body_length[model.dof_body[dof]];
        } else {
            model.dof_length[dof] = 1.0; // translational: already in [m/s]
        }
    }
}
