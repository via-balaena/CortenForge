//! Model construction and precomputation methods.
//!
//! This module contains [`Model::empty()`], [`Model::make_data()`], and the
//! precomputation methods called at model build time: ancestor computation,
//! implicit integration parameter caching, mean-inertia statistics, and
//! per-DOF mechanism length computation (§16.14).

use nalgebra::{DMatrix, DVector, Matrix3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::body_wrench::BodyWrench;
use super::enums::{
    DISABLE_NATIVECCD, ENABLE_SLEEP, EqualityType, Integrator, JacobianType, JointLayoutError,
    MakeDataError, MjJointType, MjSensorType, RangeError, ResetError, SleepPolicy, SleepState,
    SolverType, StepError,
};
use super::model::Model;

/// What [`Model::try_make_data`] names when it refuses [`DISABLE_NATIVECCD`].
const NATIVECCD_DISABLED: &str =
    "DISABLE_NATIVECCD: MuJoCo's libccd convex collision is not ported";

// Types from dynamics module (Phase 7 extraction)
use crate::dynamics::SpatialVector;
use crate::dynamics::crba::{DEFAULT_MASS_FALLBACK, mj_crba};
use crate::dynamics::factor::mj_solve_m;
use crate::dynamics::spatial::mju_mul_inert_vec;
use crate::forward::MjStage;
use crate::jacobian::mj_jac_body_com;
use crate::linalg::mju_dot;

use super::data::Data;
use crate::forward::mj_fwd_position;
use crate::island::{K_AWAKE, mj_sleep, mj_update_sleep_arrays, reset_sleep_state};

/// Why a model's history buffers cannot be initialised, shared by
/// [`MakeDataError`] and [`ResetError`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryRefusal {
    /// History buffers with a timestep that is not positive.
    InvalidTimestep,
    /// A user or plugin sensor with a delay.
    DelayedUserSensor(usize),
}

impl From<HistoryRefusal> for MakeDataError {
    fn from(e: HistoryRefusal) -> Self {
        match e {
            HistoryRefusal::InvalidTimestep => Self::InvalidTimestep,
            HistoryRefusal::DelayedUserSensor(sensor) => Self::DelayedUserSensor { sensor },
        }
    }
}

impl From<HistoryRefusal> for ResetError {
    fn from(e: HistoryRefusal) -> Self {
        match e {
            HistoryRefusal::InvalidTimestep => Self::InvalidTimestep,
            HistoryRefusal::DelayedUserSensor(sensor) => Self::DelayedUserSensor { sensor },
        }
    }
}

/// Why the trees that start asleep could not be put to sleep, shared by
/// [`MakeDataError`] and [`ResetError`] (see [`Model::start_sleep`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitSleepRefusal {
    /// The forward pass before they sleep failed.
    Forward(StepError),
    /// `mj_sleep` put `slept` of the `marked` trees to sleep.
    NotSlept {
        /// The trees whose policy is `Init`.
        marked: usize,
        /// The trees put to sleep.
        slept: usize,
        /// The first `Init` tree left awake.
        tree: usize,
        /// That tree's root body.
        root_body: usize,
    },
}

impl From<InitSleepRefusal> for MakeDataError {
    fn from(e: InitSleepRefusal) -> Self {
        match e {
            InitSleepRefusal::Forward(e) => Self::InitForward(e),
            InitSleepRefusal::NotSlept {
                marked,
                slept,
                tree,
                root_body,
            } => Self::InitSleep {
                marked,
                slept,
                tree,
                root_body,
            },
        }
    }
}

impl From<InitSleepRefusal> for ResetError {
    fn from(e: InitSleepRefusal) -> Self {
        match e {
            InitSleepRefusal::Forward(e) => Self::InitForward(e),
            InitSleepRefusal::NotSlept {
                marked,
                slept,
                tree,
                root_body,
            } => Self::InitSleep {
                marked,
                slept,
                tree,
                root_body,
            },
        }
    }
}

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
            body_invweight0: vec![[0.0; 2]], // world body

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
            jnt_margin: vec![],

            // DOFs (empty)
            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],
            dof_frictionloss: vec![],
            dof_solref: vec![],
            dof_solimp: vec![],
            dof_invweight0: vec![],

            // Simple bodies and dof–dof patterns (computed by
            // compute_simple_bodies and compute_dof_patterns)
            body_sameframe: vec![],
            geom_sameframe: vec![],
            site_sameframe: vec![],
            body_simple: vec![],
            dof_simplenum: vec![],
            M_rownnz: vec![],
            dof_M0: vec![],
            M_rowadr: vec![],
            M_colind: vec![],
            D_rownnz: vec![],
            D_rowadr: vec![],
            D_diag: vec![],
            D_colind: vec![],

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
            geom_aabb: vec![],
            body_margin: vec![0.0],
            body_bvhadr: vec![None],
            body_bvhnum: vec![0],
            bvh_aabb: vec![],
            bvh_child: vec![],
            bvh_nodeid: vec![],
            bvh_depth: vec![],
            geom_mesh: vec![],
            geom_hfield: vec![],
            geom_shape: vec![],
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
            flex_internal: vec![],
            flex_activelayers: vec![],
            flex_vertcollide: vec![],
            flex_passive: vec![],
            flex_edgestiffness: vec![],
            flex_edgedamping: vec![],
            flex_edgeequality: vec![],
            flex_bend_stiffness: vec![],
            flex_bend_damping: vec![],
            flex_density: vec![],
            flex_group: vec![],
            flex_rigid: vec![],
            flex_bending_type: vec![],
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
            flexedge_rigid: vec![],
            flexedge_invweight0: vec![],
            flexedge_flap: vec![],
            flex_bending: vec![],
            flexedge_J_rownnz: vec![],
            flexedge_J_rowadr: vec![],
            flexedge_J_colind: vec![],
            flexelem_data: vec![],
            flexelem_dataadr: vec![],
            flexelem_datanum: vec![],
            flexelem_volume0: vec![],
            flexelem_flexid: vec![],
            flex_elem_adj: vec![],
            flex_elem_adj_adr: vec![],
            flex_elem_adj_num: vec![],
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

            // Shapes (empty)
            nshape: 0,
            shape_data: vec![],

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
            sensor_nsample: vec![],
            sensor_interp: vec![],
            sensor_historyadr: vec![],
            sensor_delay: vec![],
            sensor_interval: vec![],

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
            actuator_nsample: vec![],
            actuator_interp: vec![],
            actuator_historyadr: vec![],
            actuator_delay: vec![],
            nhistory: 0,

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
            tendon_solref_lim: vec![],
            tendon_solimp_lim: vec![],
            tendon_margin: vec![],
            tendon_frictionloss: vec![],
            tendon_solref_fri: vec![],
            tendon_solimp_fri: vec![],
            tendon_group: vec![],
            tendon_rgba: vec![],
            tendon_treenum: vec![],
            tendon_tree: vec![],
            tendon_invweight0: vec![],
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
            qpos_spring: vec![],
            // Keyframes
            keyframes: Vec::new(),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,   // No fluid by default
            viscosity: 0.0, // No fluid by default
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,                // MuJoCo default
            regularization: 1e-6,         // PGS constraint softness
            friction_smoothing: 1000.0,   // tanh transition sharpness
            cone: 0,                      // Pyramidal friction cone (MuJoCo default)
            jacobian: JacobianType::Auto, // MuJoCo's default
            stat_meaninertia: 1.0,        // Default (computed at model build from CRBA)
            ls_iterations: 50,            // Newton line search iterations
            ls_tolerance: 0.01,           // Newton line search gradient tolerance
            noslip_iterations: 0,         // Default: no noslip post-processing
            noslip_tolerance: 1e-6,       // Default tolerance for noslip convergence
            diagapprox_bodyweight: false, // Default: exact M⁻¹ solve
            disableflags: 0,              // Nothing disabled
            enableflags: 0,               // MuJoCo default: all enable bits clear
            disableactuator: 0,           // No actuator groups disabled
            actuator_group: Vec::new(),   // All actuators in group 0 (empty for empty model)
            o_margin: 0.0,
            o_solref: [0.02, 1.0],
            o_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            o_friction: [1.0, 1.0, 0.005, 0.0001, 0.0001],
            ccd_iterations: 35,  // MuJoCo default for convex solver iterations
            ccd_tolerance: 1e-6, // MuJoCo default for convex solver tolerance
            sdf_iterations: 10,  // MuJoCo default for SDF Newton iterations
            sdf_initpoints: 40,  // MuJoCo default for SDF initial sample points
            sdf_maxcontact: 50,  // Cap SDF-SDF contacts per pair (see the field's doc)
            integrator: Integrator::Euler,
            solver_type: SolverType::Newton, // MuJoCo's default since 2.0; matches the GPU path

            // Cached implicit integration parameters (empty for empty model)
            implicit_stiffness: DVector::zeros(0),
            implicit_damping: DVector::zeros(0),
            implicit_springref: DVector::zeros(0),

            // Pre-computed kinematic data (world body has no ancestors)
            // The world is its own weld group.
            body_weldid: vec![0],
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
            keyframe_name_to_id: HashMap::new(),

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

            // Per-element user data (§55)
            body_user: vec![],
            jnt_user: vec![],
            geom_user: vec![],
            site_user: vec![],
            tendon_user: vec![],
            actuator_user: vec![],
            sensor_user: vec![],
            nuser_body: 0,
            nuser_jnt: 0,
            nuser_geom: 0,
            nuser_site: 0,
            nuser_tendon: 0,
            nuser_actuator: 0,
            nuser_sensor: 0,

            // Plugin instances (§66) — empty for models without plugins
            nplugin: 0,
            npluginstate: 0,
            body_plugin: vec![None; 1], // World body 0
            geom_plugin: vec![],
            actuator_plugin: vec![],
            sensor_plugin: vec![],
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

    /// Check the per-body joint layout, in MuJoCo's order for each body:
    ///
    /// 1. a body's joints have at most 6 degrees of freedom (MuJoCo 3.5.0
    ///    `user_objects.cc:2524-2526`), so a free joint is its body's only
    ///    joint;
    /// 2. a ball joint is the last joint on its body. MuJoCo refuses a ball
    ///    followed by a ball or hinge (`:2528-2537`); this also refuses one
    ///    followed by a slide.
    ///
    /// The motion subspace of a ball or free joint is read from the body's
    /// final orientation (`xquat[body]`), which is the right frame only when no
    /// later same-body joint rotates it.
    ///
    /// Not checked: a free joint below the root, which sim-core supports.
    ///
    /// # Errors
    /// The first [`JointLayoutError`] found.
    ///
    /// # Panics
    /// Panics if `body_jnt_adr`, `body_jnt_num` or `jnt_type` is shorter than
    /// the model's bodies and joints need.
    pub fn check_joint_layout(&self) -> Result<(), JointLayoutError> {
        for body in 0..self.nbody {
            let joints = self.body_jnt_adr[body]..self.body_jnt_adr[body] + self.body_jnt_num[body];
            let ndof: usize = joints.clone().map(|j| self.jnt_type[j].nv()).sum();
            if ndof > 6 {
                return Err(JointLayoutError::TooManyDofs { body, ndof });
            }
            let last = joints.end;
            for joint in joints {
                if self.jnt_type[joint] == MjJointType::Ball && joint + 1 != last {
                    return Err(JointLayoutError::BallNotLast { body, joint });
                }
            }
        }
        Ok(())
    }

    /// Check every limited range, as MuJoCo's compiler does:
    ///
    /// - a limited hinge or slide joint, a limited tendon and an actuator's
    ///   activation range when limited: `lower < upper` (MuJoCo 3.5.0
    ///   `user_objects.cc:2909`, `:6419`, `:6889`);
    /// - a limited ball joint: `lower == 0` (`:2912`);
    /// - an actuator's ctrl and force range: `lower < upper` (`:6883-6888`).
    ///   They have no `limited` flag here; an unlimited one is stored as
    ///   `(-inf, inf)`, so these two may have infinite bounds.
    ///
    /// The others must also be finite, which MuJoCo does not check. A NaN
    /// bound fails every rule.
    ///
    /// # Errors
    /// The first [`RangeError`] found, naming the field and the entry.
    pub fn check_ranges(&self) -> Result<(), RangeError> {
        let finite_and_increasing =
            |(lo, hi): (f64, f64)| lo.is_finite() && hi.is_finite() && lo < hi;
        let joints = self
            .jnt_type
            .iter()
            .zip(&self.jnt_limited)
            .zip(&self.jnt_range);
        for (index, ((&kind, &limited), &(lo, hi))) in joints.enumerate() {
            let valid = match kind {
                MjJointType::Hinge | MjJointType::Slide => finite_and_increasing((lo, hi)),
                MjJointType::Ball => lo == 0.0 && hi.is_finite(),
                MjJointType::Free => true,
            };
            if limited && !valid {
                return Err(RangeError {
                    field: "jnt_range",
                    index,
                });
            }
        }
        let limited_ranges = [
            ("tendon_range", &self.tendon_limited, &self.tendon_range),
            (
                "actuator_actrange",
                &self.actuator_actlimited,
                &self.actuator_actrange,
            ),
        ];
        for (field, limited, ranges) in limited_ranges {
            for (index, (&limited, &range)) in limited.iter().zip(ranges).enumerate() {
                if limited && !finite_and_increasing(range) {
                    return Err(RangeError { field, index });
                }
            }
        }
        let clamps = [
            ("actuator_ctrlrange", &self.actuator_ctrlrange),
            ("actuator_forcerange", &self.actuator_forcerange),
        ];
        for (field, ranges) in clamps {
            for (index, &(lo, hi)) in ranges.iter().enumerate() {
                let increasing = lo < hi;
                if !increasing {
                    return Err(RangeError { field, index });
                }
            }
        }
        Ok(())
    }

    /// Create the `Data` for this model: [`Self::try_make_data`], panicking on
    /// its error.
    ///
    /// # Panics
    /// Panics with the [`MakeDataError`]'s message where `try_make_data`
    /// returns it, and as [`Self::check_joint_layout`] says.
    #[must_use]
    // The documented panic: `try_make_data` is the non-panicking form.
    #[allow(clippy::panic)]
    pub fn make_data(&self) -> Data {
        self.try_make_data().unwrap_or_else(|e| panic!("{e}"))
    }

    /// Create the `Data` for this model, with every array allocated: the joint
    /// layout and range checks, then the arrays, each plugin's `init`, and a
    /// [`Data::reset`], which clears what `init` wrote outside the plugin
    /// state, sets the sleep state and runs each plugin's `reset`, as MuJoCo
    /// 3.5.0's `mj_makeData` runs `mj_initPlugin` and then `mj_resetData`
    /// (`engine_io.c:1110-1111`).
    ///
    /// # Errors
    /// [`MakeDataError::JointLayout`] ([`Self::check_joint_layout`]),
    /// [`MakeDataError::Range`] ([`Self::check_ranges`]), the history
    /// buffers' refusals ([`MakeDataError::InvalidTimestep`],
    /// [`MakeDataError::DelayedUserSensor`]), [`MakeDataError::PluginInit`]
    /// if a plugin's `init` returns an error, or the reset's refusal of the
    /// trees that start asleep ([`MakeDataError::InitSleep`],
    /// [`MakeDataError::InitForward`]); [`MakeDataError::Unsupported`] with
    /// [`DISABLE_NATIVECCD`] set, under which MuJoCo collides convex pairs
    /// with libccd, which sim-core does not port (a flag set after this call
    /// is not checked).
    ///
    /// # Panics
    /// As [`Self::check_joint_layout`] says.
    pub fn try_make_data(&self) -> Result<Data, MakeDataError> {
        self.check_joint_layout()?;
        self.check_ranges()?;
        if self.disableflags & DISABLE_NATIVECCD != 0 {
            return Err(MakeDataError::Unsupported {
                feature: NATIVECCD_DISABLED,
            });
        }
        if let Some(refusal) = self.history_refusal() {
            return Err(refusal.into());
        }
        let mut data = self.allocate_data();
        for i in 0..self.nplugin {
            self.plugin_objects[i]
                .init(self, &mut data, i)
                .map_err(|message| MakeDataError::PluginInit {
                    instance: i,
                    message,
                })?;
        }
        data.reset_checked(self)?;
        Ok(data)
    }

    /// Why this model's history buffers cannot be initialised, if they
    /// cannot: they need a positive timestep (MuJoCo `_resetData`,
    /// `engine_io.c:1266-1270`, tests `nhistory && dt <= 0`), and a delayed
    /// sample of a user or plugin sensor cannot be computed (MuJoCo's
    /// `mj_computeSensor` has no such type and `mjERROR`s when it inserts
    /// one, `engine_sensor.c:739-740`, `:858-859`, `:1316-1317`).
    pub(crate) fn history_refusal(&self) -> Option<HistoryRefusal> {
        if self.nhistory > 0 && self.timestep <= 0.0 {
            return Some(HistoryRefusal::InvalidTimestep);
        }
        (0..self.nsensor)
            .find(|&i| {
                matches!(
                    self.sensor_type[i],
                    MjSensorType::User | MjSensorType::Plugin
                ) && self.sensor_nsample[i] > 0
                    && self.sensor_delay[i] > 0.0
            })
            .map(HistoryRefusal::DelayedUserSensor)
    }

    /// The `Data` that building a model derives values from (`acc0`,
    /// `lengthrange`, `invweight0`, `stat_meaninertia`, tendon `length0`):
    /// every array allocated, with no checks and no plugin `init`. The MJCF
    /// builder and [`Self::recompute_derived`] run those derivations only on a
    /// model whose joint layout and ranges pass, so they never step a joint
    /// layout or a range `try_make_data` refuses.
    pub(crate) fn make_data_for_derivation(&self) -> Data {
        let mut data = self.allocate_data();
        // Every tree awake: MuJoCo derives with sleep disabled
        // (`user_model.cc:5108-5113`).
        reset_sleep_state(self, &mut data);
        data
    }

    /// Every array of a `Data` for this model, as a reset leaves it before
    /// the sleep state and the plugins: `qpos0`, the mocap bodies' poses, the
    /// history buffers' timestamps, and zero elsewhere. No checks and no
    /// plugin `init`.
    pub(crate) fn allocate_data(&self) -> Data {
        Data {
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

            // History buffers, as `_resetData` fills them
            history: {
                let mut buf = vec![0.0; self.nhistory];
                if self.nhistory > 0 {
                    crate::history::init(self, &mut buf);
                }
                buf
            },

            // Body states
            xpos: vec![Vector3::zeros(); self.nbody],
            xquat: vec![UnitQuaternion::identity(); self.nbody],
            xmat: vec![Matrix3::identity(); self.nbody],
            xipos: vec![Vector3::zeros(); self.nbody],
            ximat: vec![Matrix3::identity(); self.nbody],
            xanchor: vec![Vector3::zeros(); self.njnt],
            xaxis: vec![Vector3::zeros(); self.njnt],

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

            // Flex edge pre-computed fields
            flexedge_length: vec![0.0; self.nflexedge],
            flexedge_velocity: vec![0.0; self.nflexedge],
            flexedge_J: vec![0.0; self.flexedge_J_colind.len()],

            // Velocities
            cvel: vec![SpatialVector::zeros(); self.nbody],
            cdof: vec![SpatialVector::zeros(); self.nv],
            cdof_dot: vec![SpatialVector::zeros(); self.nv],

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
            xfrc_applied: vec![BodyWrench::default(); self.nbody],

            // Mass matrix (dense)
            qM: DMatrix::zeros(self.nv, self.nv),

            // Mass matrix (sparse L^T D L — computed in mj_crba via mj_factor_m)
            qLD_data: vec![0.0; self.M_colind.len()],
            qLD_diag_inv: vec![0.0; self.nv],
            qLD_valid: false,

            // The implicit integrators' factors (MuJoCo qH, qHDiagInv, qLU)
            qH: vec![0.0; self.M_colind.len()],
            qHDiagInv: vec![0.0; self.nv],
            qLU: vec![0.0; self.D_colind.len()],

            // Body spatial inertias (computed once in FK, used by CRBA and RNE)
            cinert: vec![[0.0; 10]; self.nbody],
            // Composite rigid body inertias (for Featherstone CRBA)
            crb: vec![[0.0; 10]; self.nbody],

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
            ten_J_rownnz: vec![0; self.ntendon],
            ten_J_colind: vec![0; self.ntendon * self.nv],

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

            // Unified constraint system: the rows are made in the position stage.
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
            primal_qacc: false,
            solver_stat: Vec::new(),
            stat_meaninertia: 1.0,

            // Sensors
            sensordata: DVector::zeros(self.nsensordata),

            // Warnings
            warnings: [super::warning::WarningStat::default(); super::warning::NUM_WARNINGS],

            // Energy
            energy_potential: 0.0,
            energy_kinetic: 0.0,
            energy_initial: 0.0,
            energy_initial_captured: false,
            solver_fwdinv: [0.0, 0.0],

            // Sleep state (§16.7): every tree awake; `Model::start_sleep` puts
            // the trees that start asleep to sleep and fills the arrays.
            tree_asleep: vec![K_AWAKE; self.ntree],
            tree_awake: vec![true; self.ntree],
            body_sleep_state: vec![SleepState::Awake; self.nbody],
            ntree_awake: self.ntree,
            nv_awake: self.nv,

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

            // qpos change detection (§16.15)
            tree_qpos_dirty: vec![false; self.ntree],

            // Time
            time: 0.0,

            // Scratch buffers
            scratch_m_impl: DMatrix::zeros(self.nv, self.nv),
            scratch_force: DVector::zeros(self.nv),
            scratch_rhs: DVector::zeros(self.nv),
            scratch_v_new: DVector::zeros(self.nv),
            qacc_implicit: DVector::zeros(self.nv),

            // RK4 scratch buffers
            rk4_qpos_saved: DVector::zeros(self.nq),
            rk4_qpos_stage: DVector::zeros(self.nq),
            rk4_qvel: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_qacc: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_dX_vel: DVector::zeros(self.nv),
            rk4_dX_acc: DVector::zeros(self.nv),
            rk4_act_saved: DVector::zeros(self.na),
            rk4_act_dot: std::array::from_fn(|_| DVector::zeros(self.na)),
            rk4_dof_asleep: vec![false; self.nv],

            // Derivative scratch buffers (for mjd_smooth_vel / mjd_rne_vel)
            qDeriv: DMatrix::zeros(self.nv, self.nv),
            deriv_Dcvel: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcacc: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcfrc: vec![DMatrix::zeros(6, self.nv); self.nbody],

            // Position derivative scratch buffers (for mjd_smooth_pos / mjd_rne_pos)
            qDeriv_pos: DMatrix::zeros(self.nv, self.nv),
            deriv_Dcvel_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcacc_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcfrc_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],

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

            // §66: Plugin state
            plugin_state: vec![0.0; self.npluginstate],
            plugin_data: (0..self.nplugin).map(|_| None).collect(),
        }
    }

    /// Start the sleep state, as MuJoCo's `mj_resetData` (3.5.0
    /// `engine_io.c:1440-1505`): every tree awake; then, with sleep enabled
    /// and a tree whose policy is `Init`, a forward pass, those trees marked
    /// ready, `mj_sleep`, and `qacc_smooth`, `qfrc_smooth` and the constraint
    /// rows cleared. With sleep enabled and no such tree, MuJoCo's reset also
    /// computes the kinematics, centres of mass, cameras and tendons
    /// (`:1453-1458`); this computes none of them.
    ///
    /// # Errors
    /// The forward pass's error, or the trees that start asleep that
    /// `mj_sleep` could not put to sleep (a tree in an island with an awake
    /// one): MuJoCo raises an error for both.
    pub(crate) fn start_sleep(&self, data: &mut Data) -> Result<(), InitSleepRefusal> {
        reset_sleep_state(self, data);
        let init = |t: &usize| self.tree_sleep_policy[*t] == SleepPolicy::Init;
        let marked = (0..self.ntree).filter(init).count();
        if self.enableflags & ENABLE_SLEEP == 0 || marked == 0 {
            return Ok(());
        }
        // MuJoCo's reset runs `mj_forward`, which checks no timestep (a
        // timestep at or below 0 is refused only with history buffers,
        // before this) and raises on a tendon equality with sleep.
        crate::forward::check::check_tendon_equality_sleep(self)
            .map_err(InitSleepRefusal::Forward)?;
        data.forward_skip_unchecked(self, MjStage::None, false)
            .map_err(InitSleepRefusal::Forward)?;
        for t in 0..self.ntree {
            data.tree_asleep[t] = if init(&t) { -1 } else { K_AWAKE };
        }
        let slept = mj_sleep(self, data);
        if slept != marked {
            let tree = (0..self.ntree)
                .find(|t| init(t) && data.tree_asleep[*t] < 0)
                .unwrap_or(0);
            return Err(InitSleepRefusal::NotSlept {
                marked,
                slept,
                tree,
                root_body: self.tree_body_adr[tree],
            });
        }
        data.qacc_smooth.fill(0.0);
        data.qfrc_smooth.fill(0.0);
        crate::constraint::clear_constraint_rows(self, data);
        mj_update_sleep_arrays(self, data);
        Ok(())
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

        // ── Weld groups (MuJoCo's `body_weldid`) ────────────────────
        // Walk up while each body contributes no degrees of freedom: a
        // zero-dof joint is a weld, so the body is rigidly part of its
        // parent. The walk stops at the first body with dofs, or at the
        // world — which is why a jointless body hanging off the world lands
        // in weld group 0 together with the ground, and static bodies
        // therefore never collide with each other.
        self.body_weldid = vec![0; self.nbody];
        for body_id in 0..self.nbody {
            let mut current = body_id;
            while current != 0 && self.body_dof_num[current] == 0 {
                current = self.body_parent[current];
            }
            self.body_weldid[body_id] = current;
        }

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
                    self.implicit_springref[dof_adr] = self.qpos_spring[self.jnt_qpos_adr[jnt_id]];
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

    /// Each dof's mass-matrix diagonal at `qpos0`, `dof_M0`, as MuJoCo's
    /// `mj_setM0` (`engine_setconst.c:36-59`): the position stage at `qpos0`
    /// with every tree awake, each body's `cinert` summed into its parent's
    /// (not the world's), then the dof's armature plus `cdof · (crb · cdof)`
    /// (`mju_dot`). The mass matrix takes a simple dof's row from it.
    ///
    /// Must be called after `compute_simple_bodies()` and
    /// `compute_dof_patterns()`, and once the bodies, joints and armature are
    /// populated; before what builds the mass matrix at `qpos0`
    /// ([`Self::compute_actuator_params`], [`Self::compute_stat_meaninertia`],
    /// [`Self::compute_invweight0`]).
    pub fn compute_dof_m0(&mut self) {
        if self.nv == 0 {
            self.dof_M0 = Vec::new();
            return;
        }
        let mut data = self.make_data_for_derivation();
        mj_fwd_position(self, &mut data);
        let mut crb = data.cinert.clone();
        for i in (1..self.nbody).rev() {
            let parent = self.body_parent[i];
            if parent > 0 {
                let child = crb[i];
                for (sum, add) in crb[parent].iter_mut().zip(child) {
                    *sum += add;
                }
            }
        }
        self.dof_M0 = (0..self.nv)
            .map(|i| {
                let buf = mju_mul_inert_vec(&crb[self.dof_body[i]], &data.cdof[i]);
                self.dof_armature[i] + mju_dot(data.cdof[i].as_slice(), buf.as_slice())
            })
            .collect();
    }

    /// Compute `body_invweight0`, `dof_invweight0`, and `tendon_invweight0`.
    ///
    /// MuJoCo ref: `setInertia()` in `engine_setconst.c`.
    ///
    /// - `body_invweight0[b]` = operational-space inverse inertia at body COM:
    ///   `[0]` = avg translational diagonal of `J·M⁻¹·J^T` (6×6),
    ///   `[1]` = avg rotational diagonal; for a simple body whose joints are
    ///   all slides (`body_simple` 2), `[1/mass, 0]`, as MuJoCo's
    ///   (`engine_setconst.c:628-635`)
    /// - `dof_invweight0[d]` = avg diagonal of `M⁻¹` subblock for that joint's DOFs;
    ///   `1/mass` on such a body, its armature left out, as MuJoCo's (`:663-673`)
    /// - `tendon_invweight0[t]` = `J_tendon · M⁻¹ · J_tendon^T` (full quadratic form,
    ///   matching MuJoCo's `setInertia()` in `engine_setconst.c`)
    /// - `flex_edgeequality[f]`: whether a flex equality names flex `f`, set
    ///   first, since an edge's Jacobian is made only then
    /// - `flexedge_invweight0[e]` as MuJoCo's (`engine_setconst.c:716-750`): 0
    ///   for a rigid edge, `(1/m₁ + 1/m₂)/2` when both vertex bodies are simple
    ///   with slides only (`body_simple` 2), else `J_edge · M⁻¹ · J_edgeᵀ`;
    ///   MuJoCo's while every vertex body hangs from the world or all from one
    ///   body: the edge Jacobian takes a vertex's own dofs alone, and an edge
    ///   is rigid when both its vertices have no mass (`flexedge_rigid`), so
    ///   with vertex bodies on different moving bodies the two differ (book
    ///   Rigid-loading L10, rule (c))
    ///
    /// Must be called after `compute_simple_bodies()` and
    /// `compute_dof_patterns()`, and once body_mass, body_inertia, dof_body,
    /// dof_jnt, jnt_type, jnt_dof_adr, and tendon/wrap arrays are populated.
    pub fn compute_invweight0(&mut self) {
        const MIN_VAL: f64 = 1e-15; // mjMINVAL

        // --- Subtree mass (still needed for body_subtreemass field used elsewhere) ---
        let mut subtree_mass = self.body_mass.clone();
        for b in (1..self.nbody).rev() {
            let parent = self.body_parent[b];
            subtree_mass[parent] += subtree_mass[b];
        }
        self.body_subtreemass = subtree_mass;

        // --- Initialize invweight arrays (world body and static bodies stay [0,0]) ---
        self.body_invweight0 = vec![[0.0; 2]; self.nbody];
        self.dof_invweight0 = vec![0.0; self.nv];
        self.flexedge_invweight0 = vec![0.0; self.nflexedge];
        self.flex_edgeequality = (0..self.nflex)
            .map(|f| {
                (0..self.neq)
                    .any(|eq| self.eq_type[eq] == EqualityType::Flex && self.eq_obj1id[eq] == f)
            })
            .collect();

        if self.nv == 0 {
            // No DOFs: all bodies are static, invweight stays zero.
            self.tendon_invweight0 = vec![0.0; self.ntendon];
            return;
        }

        // --- MuJoCo algorithm: invweight via M⁻¹ at qpos0 ---
        // Create temporary Data, run FK, the flex edges (their Jacobians for
        // a flex an equality names) and CRBA (which factors M) with every
        // tree awake.
        let mut data = self.make_data_for_derivation();
        mj_fwd_position(self, &mut data);
        crate::dynamics::flex::mj_flex(self, &mut data);
        crate::dynamics::flex::mj_flex_edge(self, &mut data);
        mj_crba(self, &mut data);

        // A simple body whose joints are all slides (MuJoCo's `body_simple`
        // 2) takes MuJoCo's short form (`engine_setconst.c:628-635`).
        let slides_only: Vec<bool> = (0..self.nbody)
            .map(|b| self.body_simple.get(b) == Some(&2))
            .collect();

        // --- body_invweight0: avg diagonal of J_com · M⁻¹ · J_com^T (6×6) ---
        for (i, &slides) in slides_only.iter().enumerate().skip(1) {
            if slides {
                self.body_invweight0[i] = [1.0 / self.body_mass[i].max(MIN_VAL), 0.0];
                continue;
            }

            // 6×nv Jacobian at body COM. Our layout: rows 0-2 = angular, rows 3-5 = linear.
            let jac = mj_jac_body_com(self, &data, i);

            // Check for static body (all-zero Jacobian means no DOFs in chain)
            let jac_norm_sq: f64 = jac.iter().map(|&v| v * v).sum();
            if jac_norm_sq < MIN_VAL {
                continue; // stays [0, 0]
            }

            // Solve M · W = J^T for W = M⁻¹ · J^T (nv × 6)
            let mut jt = jac.transpose();
            mj_solve_m(self, &data, jt.as_mut_slice());

            // A = J · W = J · M⁻¹ · J^T (6×6)
            let a = &jac * &jt;

            // Our layout: rows 3-5 = linear (translational), rows 0-2 = angular (rotational)
            let tran = (a[(3, 3)] + a[(4, 4)] + a[(5, 5)]) / 3.0;
            let rot = (a[(0, 0)] + a[(1, 1)] + a[(2, 2)]) / 3.0;

            // Fallback for degenerate cases (MuJoCo: if one is near-zero, copy the other)
            self.body_invweight0[i] = if tran < MIN_VAL && rot > MIN_VAL {
                [rot, rot]
            } else if rot < MIN_VAL && tran > MIN_VAL {
                [tran, tran]
            } else {
                [tran, rot]
            };
        }

        // --- dof_invweight0: avg diagonal of M⁻¹ subblock per joint ---
        for jnt_id in 0..self.njnt {
            let dof_adr = self.jnt_dof_adr[jnt_id];
            let jnt_type = self.jnt_type[jnt_id];
            let body = self.jnt_body[jnt_id];
            if slides_only[body] {
                self.dof_invweight0[dof_adr] = 1.0 / self.body_mass[body].max(MIN_VAL);
                continue;
            }

            let dnum = match jnt_type {
                MjJointType::Free => 6,
                MjJointType::Ball => 3,
                _ => 1, // Hinge or Slide
            };

            // Build selector RHS: column k has 1.0 at row dof_adr+k
            let mut rhs = DMatrix::zeros(self.nv, dnum);
            for j in 0..dnum {
                rhs[(dof_adr + j, j)] = 1.0;
            }

            // Solve M · W = selector → W columns are M⁻¹ columns at DOF positions
            mj_solve_m(self, &data, rhs.as_mut_slice());

            // Extract diagonal: rhs[(dof_adr+k, k)] = M⁻¹[dof_adr+k, dof_adr+k]
            match jnt_type {
                MjJointType::Free => {
                    let tran =
                        (rhs[(dof_adr, 0)] + rhs[(dof_adr + 1, 1)] + rhs[(dof_adr + 2, 2)]) / 3.0;
                    let rot =
                        (rhs[(dof_adr + 3, 3)] + rhs[(dof_adr + 4, 4)] + rhs[(dof_adr + 5, 5)])
                            / 3.0;
                    for j in 0..3 {
                        self.dof_invweight0[dof_adr + j] = tran;
                    }
                    for j in 3..6 {
                        self.dof_invweight0[dof_adr + j] = rot;
                    }
                }
                MjJointType::Ball => {
                    let avg =
                        (rhs[(dof_adr, 0)] + rhs[(dof_adr + 1, 1)] + rhs[(dof_adr + 2, 2)]) / 3.0;
                    for j in 0..3 {
                        self.dof_invweight0[dof_adr + j] = avg;
                    }
                }
                _ => {
                    // Hinge or Slide: single scalar M⁻¹[id,id]
                    self.dof_invweight0[dof_adr] = rhs[(dof_adr, 0)];
                }
            }
        }

        // --- flexedge_invweight0 (`engine_setconst.c:716-750`) ---
        for e in 0..self.nflexedge {
            let [v1, v2] = self.flexedge_vert[e];
            let (b1, b2) = (self.flexvert_bodyid[v1], self.flexvert_bodyid[v2]);
            self.flexedge_invweight0[e] = if self.flexedge_rigid[e] {
                0.0
            } else if slides_only[b1] && slides_only[b2] {
                // MuJoCo's expression, operation for operation.
                #[allow(clippy::manual_midpoint)]
                let w = (1.0 / self.body_mass[b1] + 1.0 / self.body_mass[b2]) / 2.0;
                w
            } else {
                let mut j = vec![0.0; self.nv];
                let adr = self.flexedge_J_rowadr[e];
                for k in adr..adr + self.flexedge_J_rownnz[e] {
                    j[self.flexedge_J_colind[k]] = data.flexedge_J[k];
                }
                let mut w = j.clone();
                mj_solve_m(self, &data, &mut w);
                mju_dot(&j, &w)
            };
        }

        // --- tendon_invweight0 ---
        // MuJoCo ref: setInertia() in engine_setconst.c
        // For each tendon: invweight0 = J_tendon · M⁻¹ · J_tendon^T
        // where J_tendon = data.ten_J[t] (populated by mj_fwd_tendon in mj_fwd_position).
        // This uses the FULL quadratic form, capturing off-diagonal M⁻¹ coupling
        // between DOFs in serial kinematic chains. Applies uniformly to all tendon
        // types (fixed and spatial) — no type-specific branching needed.
        self.tendon_invweight0 = vec![0.0; self.ntendon];
        let mut rhs = DMatrix::zeros(self.nv, 1);
        for t in 0..self.ntendon {
            let j_tendon = &data.ten_J[t];

            // Skip zero Jacobians (no DOFs contribute to this tendon)
            let j_norm_sq: f64 = j_tendon.iter().map(|&v| v * v).sum();
            if j_norm_sq < MIN_VAL {
                self.tendon_invweight0[t] = MIN_VAL;
                continue;
            }

            // Copy J_tendon into reusable 1-column RHS for the batch solver
            rhs.column_mut(0).copy_from(j_tendon);

            // Solve M · w = J_tendon → w = M⁻¹ · J_tendon
            mj_solve_m(self, &data, rhs.as_mut_slice());

            // tendon_invweight0 = J_tendon · M⁻¹ · J_tendon^T
            let w = j_tendon.dot(&rhs.column(0));
            self.tendon_invweight0[t] = w.max(MIN_VAL);
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
        let mut data = self.make_data_for_derivation();
        mj_fwd_position(self, &mut data);
        mj_crba(self, &mut data);
        // (§27F) Pinned flex vertices now have no DOFs — no need to skip them.
        let mut trace = 0.0_f64;
        for i in 0..self.nv {
            trace += data.qM[(i, i)];
        }
        // `nbody`/`nv` model dimensions are usize → f64 for diagnostic averaging; bounded by realistic model sizes.
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

/// Each body's size, MuJoCo's `setStat` at `qpos0` (3.5.0
/// `engine_setconst.c:976-1025`): the largest distance from the body's centre
/// of mass to the anchor of a joint of its own or of a child's; then the
/// largest `rbound` plus the distance from the centre of mass to the geom, over
/// its geoms with a finite positive `rbound` (a plane's is 0 in MuJoCo and
/// infinite here); at least 1e-5. MuJoCo also takes the flex
/// edge lengths at a flex vertex body; a flex vertex body here has slide
/// joints only, whose dofs do not read the size, so that term is left out.
fn compute_body_lengths(model: &Model) -> Vec<f64> {
    let mut data = model.make_data_for_derivation();
    mj_fwd_position(model, &mut data);
    let mut size = vec![0.0_f64; model.nbody];
    for jnt in 0..model.njnt {
        let body = model.jnt_body[jnt];
        for b in [body, model.body_parent[body]] {
            size[b] = size[b].max((data.xipos[b] - data.xanchor[jnt]).norm());
        }
    }
    for (b, body_size) in size.iter_mut().enumerate().skip(1) {
        for g in model.body_geom_adr[b]..model.body_geom_adr[b] + model.body_geom_num[b] {
            let rbound = model.geom_rbound[g];
            if rbound > 0.0 && rbound.is_finite() {
                *body_size = body_size.max(rbound + (data.xipos[b] - data.geom_xpos[g]).norm());
            }
        }
        *body_size = body_size.max(1e-5);
    }
    size
}

/// Each dof's length, MuJoCo's `dof_length` (`engine_setconst.c:1027-1043`).
///
/// A rotational dof (hinge, ball, a free joint's last three) takes its body's
/// size (`compute_body_lengths`), a translational one 1. Sleep compares
/// `dof_length · |qvel|` with the sleep tolerance.
///
/// The sizes come from the kinematics at `qpos0`, so they need a model whose
/// joint layout and ranges [`Model::try_make_data`] accepts; for another
/// model, which cannot be stepped, every dof takes 1.
pub fn compute_dof_lengths(model: &mut Model) {
    model.dof_length = vec![1.0; model.nv];
    if model.nv == 0 || model.check_joint_layout().is_err() || model.check_ranges().is_err() {
        return;
    }
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

#[cfg(test)]
mod derivation_data_tests {
    use crate::island::K_AWAKE;
    use crate::types::{ENABLE_SLEEP, Model, SleepPolicy};
    use nalgebra::Vector3;

    /// The derivations run with every tree awake, as MuJoCo derives with
    /// sleep disabled (`user_model.cc:5108-5113`), on a model whose tree a
    /// `Data` starts asleep (ledger L80).
    #[test]
    fn derivation_data_keeps_a_tree_that_starts_asleep_awake() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.enableflags |= ENABLE_SLEEP;
        model.tree_sleep_policy[0] = SleepPolicy::Init;
        let data = model.make_data();
        assert!(
            data.tree_asleep[0] >= 0,
            "the fixture's tree starts asleep, tree_asleep {:?}",
            data.tree_asleep
        );
        let derivation = model.make_data_for_derivation();
        assert_eq!(derivation.tree_asleep, vec![K_AWAKE; model.ntree]);
        assert_eq!(derivation.ntree_awake, model.ntree);
    }
}

#[cfg(test)]
mod dof_length_tests {
    use super::compute_dof_lengths;
    use crate::types::Model;
    use nalgebra::Vector3;

    /// A plane on a moving body (MuJoCo refuses one; this crate takes it) has
    /// an infinite bounding radius, so it does not size its body: the free
    /// body's centre of mass is its joint anchor, which leaves MuJoCo's
    /// floor, 1e-5.
    #[test]
    fn a_plane_does_not_size_its_body() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.geom_body[0] = 1;
        model.body_geom_num[0] = 0;
        model.body_geom_adr[1] = 0;
        model.body_geom_num[1] = 1;
        model.compute_geom_bounding_radii();
        assert!(model.geom_rbound[0].is_infinite());
        compute_dof_lengths(&mut model);
        assert_eq!(model.dof_length, vec![1.0, 1.0, 1.0, 1e-5, 1e-5, 1e-5]);
    }

    /// A geom with a bounding radius of 0 does not size its body either, as
    /// MuJoCo takes only positive radii: a zero-radius sphere 0.3 from the
    /// centre of mass leaves the floor.
    #[test]
    fn a_zero_radius_geom_does_not_size_its_body() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.geom_type[0] = crate::types::GeomType::Sphere;
        model.geom_size[0] = Vector3::zeros();
        model.geom_pos[0] = Vector3::new(0.3, 0.0, 0.0);
        model.geom_body[0] = 1;
        model.body_geom_num[0] = 0;
        model.body_geom_adr[1] = 0;
        model.body_geom_num[1] = 1;
        model.compute_simple_bodies();
        model.compute_geom_bounding_radii();
        assert_eq!(model.geom_rbound[..1], [0.0]);
        compute_dof_lengths(&mut model);
        assert_eq!(model.dof_length, vec![1.0, 1.0, 1.0, 1e-5, 1e-5, 1e-5]);
    }
}

#[cfg(test)]
mod joint_layout_tests {
    #![allow(clippy::expect_used)]
    use crate::test_fixtures::builders::{
        add_ball_joint, add_body, add_freejoint, add_hinge_joint, add_slide_joint, finalize,
    };
    use crate::types::{JointLayoutError, MakeDataError, Model, ModelError, RangeError};
    use nalgebra::Vector3;

    fn body(model: &mut Model, parent: usize, name: &str) -> usize {
        add_body(
            model,
            parent,
            name,
            Vector3::zeros(),
            1.0,
            Vector3::new(0.02, 0.03, 0.04),
            Vector3::new(0.01, -0.02, -0.15),
        )
    }

    fn hinge(m: &mut Model, b: usize, name: &str) {
        add_hinge_joint(m, b, name, Vector3::x(), 0.0, 0.0, 0.0, false, (-3.0, 3.0));
    }

    /// A ball joint that is the LAST joint on its body is valid (hinge → ball).
    #[test]
    fn ball_last_on_body_is_valid() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        hinge(&mut m, b, "h0");
        add_ball_joint(&mut m, b, "ball0");
        finalize(&mut m);
        assert_eq!(m.check_joint_layout(), Ok(()));
        assert!(m.try_make_data().is_ok());
    }

    /// MuJoCo collides convex pairs with libccd's MPR under
    /// `DISABLE_NATIVECCD`, which sim-core does not port: `try_make_data`
    /// refuses the model, and makes its `Data` without the flag.
    #[test]
    fn try_make_data_refuses_disable_nativeccd() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        m.disableflags |= super::DISABLE_NATIVECCD;
        assert_eq!(
            m.try_make_data().err(),
            Some(MakeDataError::Unsupported {
                feature: super::NATIVECCD_DISABLED
            })
        );
        m.disableflags &= !super::DISABLE_NATIVECCD;
        assert!(m.try_make_data().is_ok());
    }

    /// `make_data` panics with the refusal's message, which names the flag.
    #[test]
    #[should_panic(
        expected = "unsupported: DISABLE_NATIVECCD: MuJoCo's libccd convex collision is not ported"
    )]
    fn make_data_panics_naming_disable_nativeccd() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        m.disableflags |= super::DISABLE_NATIVECCD;
        let _data = m.make_data();
    }

    #[test]
    fn try_make_data_refuses_ball_before_hinge() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        assert_eq!(
            m.try_make_data().err(),
            Some(MakeDataError::JointLayout(JointLayoutError::BallNotLast {
                body: b,
                joint: 0
            }))
        );
    }

    /// A model whose joint layout `try_make_data` refuses cannot be stepped,
    /// so no body is sized: every `dof_length` is 1.
    #[test]
    fn a_refused_layout_takes_unit_dof_lengths() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        super::compute_dof_lengths(&mut m);
        assert_eq!(m.dof_length, vec![1.0; 4]);
    }

    /// MuJoCo refuses a ball followed by a rotation; this also refuses a ball
    /// followed by a slide.
    #[test]
    fn try_make_data_refuses_ball_before_slide() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        add_slide_joint(&mut m, b, "s0", Vector3::x(), 0.0, 0.0, 0.0);
        finalize(&mut m);
        assert_eq!(
            m.check_joint_layout(),
            Err(JointLayoutError::BallNotLast { body: b, joint: 0 })
        );
    }

    /// A free joint and a hinge on one body are 7 degrees of freedom, which is
    /// what MuJoCo reports for it ("more than 6 dofs").
    #[test]
    fn try_make_data_refuses_a_free_joint_sharing_its_body() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_freejoint(&mut m, b, "free0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        assert_eq!(
            m.try_make_data().err(),
            Some(MakeDataError::JointLayout(JointLayoutError::TooManyDofs {
                body: b,
                ndof: 7
            }))
        );
    }

    #[test]
    fn try_make_data_refuses_seven_dofs_on_one_body() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        for i in 0..7 {
            hinge(&mut m, b, &format!("h{i}"));
        }
        finalize(&mut m);
        assert_eq!(
            m.check_joint_layout(),
            Err(JointLayoutError::TooManyDofs { body: b, ndof: 7 })
        );
    }

    /// MuJoCo counts a body's degrees of freedom before it checks the ball
    /// rule: a ball before four hinges is reported as 7 dofs.
    #[test]
    fn the_dof_count_is_checked_before_the_ball_rule() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        for i in 0..4 {
            hinge(&mut m, b, &format!("h{i}"));
        }
        finalize(&mut m);
        assert_eq!(
            m.check_joint_layout(),
            Err(JointLayoutError::TooManyDofs { body: b, ndof: 7 })
        );
    }

    /// `recompute_derived` refuses a bad joint layout before it derives
    /// anything, and leaves the model as it was.
    #[test]
    fn recompute_derived_refuses_a_bad_joint_layout() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        let before = format!("{m:#?}");
        assert_eq!(
            m.recompute_derived(),
            Err(ModelError::JointLayout(JointLayoutError::BallNotLast {
                body: b,
                joint: 0
            }))
        );
        assert!(
            format!("{m:#?}") == before,
            "a refused recompute changed the model"
        );
    }

    #[test]
    #[should_panic(expected = "must be the LAST joint")]
    fn make_data_still_panics_on_a_bad_layout() {
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        let _data = m.make_data();
    }

    /// The joint layout is checked before the ranges, and a range refusal
    /// names its entry.
    #[test]
    fn try_make_data_checks_the_layout_first_and_names_the_entry() {
        let mut m = crate::test_fixtures::hinge_chain(2);
        m.jnt_limited[1] = true;
        m.jnt_range[1] = (1.0, -1.0);
        assert_eq!(
            m.try_make_data().err(),
            Some(MakeDataError::Range(RangeError {
                field: "jnt_range",
                index: 1
            }))
        );
        let mut m = Model::empty();
        let b = body(&mut m, 0, "l0");
        add_ball_joint(&mut m, b, "ball0");
        hinge(&mut m, b, "h0");
        finalize(&mut m);
        m.jnt_limited[1] = true;
        m.jnt_range[1] = (1.0, -1.0);
        assert!(matches!(
            m.try_make_data().err(),
            Some(MakeDataError::JointLayout(_))
        ));
    }

    /// A backwards ctrl range used to build a `Data` and panic in `f64::clamp`
    /// at the first step.
    #[test]
    fn try_make_data_refuses_a_backwards_limited_ctrlrange() {
        let mut m = crate::test_fixtures::hinge_chain(1);
        m.actuator_ctrlrange[0] = (1.0, -1.0);
        assert_eq!(
            m.try_make_data().err(),
            Some(MakeDataError::Range(RangeError {
                field: "actuator_ctrlrange",
                index: 0
            }))
        );
    }

    /// Each rule of `check_ranges`, on a one-hinge chain with a motor and on a
    /// body with a ball joint.
    #[test]
    fn check_ranges_refuses_each_invalid_limited_range() {
        type Edit = fn(&mut Model);
        let inf = f64::INFINITY;
        let refused = |field| Err(RangeError { field, index: 0 });
        let chain: [(&str, Edit, Result<(), RangeError>); 15] = [
            ("as built", |_| {}, Ok(())),
            (
                "hinge backwards",
                |m| set_joint(m, true, (1.0, -1.0)),
                refused("jnt_range"),
            ),
            (
                "hinge empty",
                |m| set_joint(m, true, (1.0, 1.0)),
                refused("jnt_range"),
            ),
            (
                "hinge infinite",
                |m| set_joint(m, true, (0.0, f64::INFINITY)),
                refused("jnt_range"),
            ),
            (
                "hinge NaN",
                |m| set_joint(m, true, (f64::NAN, 1.0)),
                refused("jnt_range"),
            ),
            (
                "hinge unlimited backwards",
                |m| set_joint(m, false, (1.0, -1.0)),
                Ok(()),
            ),
            (
                "ctrl empty",
                |m| m.actuator_ctrlrange[0] = (1.0, 1.0),
                refused("actuator_ctrlrange"),
            ),
            (
                "ctrl NaN",
                |m| m.actuator_ctrlrange[0] = (f64::NAN, 1.0),
                refused("actuator_ctrlrange"),
            ),
            (
                "ctrl unlimited",
                |m| m.actuator_ctrlrange[0] = (-f64::INFINITY, f64::INFINITY),
                Ok(()),
            ),
            (
                "force backwards",
                |m| m.actuator_forcerange[0] = (2.0, -2.0),
                refused("actuator_forcerange"),
            ),
            (
                "act limited backwards",
                |m| set_act(m, true, (1.0, 0.0)),
                refused("actuator_actrange"),
            ),
            (
                "act limited infinite",
                |m| set_act(m, true, (0.0, f64::INFINITY)),
                refused("actuator_actrange"),
            ),
            (
                "act unlimited backwards",
                |m| set_act(m, false, (1.0, 0.0)),
                Ok(()),
            ),
            (
                "tendon limited backwards",
                |m| add_tendon_range(m, true, (1.0, 0.0)),
                refused("tendon_range"),
            ),
            (
                "tendon unlimited backwards",
                |m| add_tendon_range(m, false, (1.0, 0.0)),
                Ok(()),
            ),
        ];
        for (case, edit, want) in chain {
            let mut m = crate::test_fixtures::hinge_chain(1);
            edit(&mut m);
            assert_eq!(m.check_ranges(), want, "{case}");
        }
        let ball: [(&str, (f64, f64), Result<(), RangeError>); 4] = [
            ("from 0", (0.0, 1.0), Ok(())),
            ("not from 0", (0.1, 1.0), refused("jnt_range")),
            ("infinite", (0.0, inf), refused("jnt_range")),
            ("NaN", (f64::NAN, 1.0), refused("jnt_range")),
        ];
        for (case, range, want) in ball {
            let mut m = Model::empty();
            let b = body(&mut m, 0, "l0");
            add_ball_joint(&mut m, b, "ball0");
            finalize(&mut m);
            set_joint(&mut m, true, range);
            assert_eq!(m.check_ranges(), want, "ball {case}");
        }
    }

    fn set_joint(m: &mut Model, limited: bool, range: (f64, f64)) {
        m.jnt_limited[0] = limited;
        m.jnt_range[0] = range;
    }

    fn set_act(m: &mut Model, limited: bool, range: (f64, f64)) {
        m.actuator_actlimited[0] = limited;
        m.actuator_actrange[0] = range;
    }

    /// Only `check_ranges` reads these two arrays here; the model has no tendon.
    fn add_tendon_range(m: &mut Model, limited: bool, range: (f64, f64)) {
        m.tendon_limited.push(limited);
        m.tendon_range.push(range);
    }
}
