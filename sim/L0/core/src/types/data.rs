//! Data struct definition, Clone impl, and core accessor methods.
//!
//! [`Data`] is the dynamic simulation state: generalized coordinates (qpos, qvel),
//! computed quantities (body poses, forces, contacts), and solver state.
//! It is the mutable counterpart to [`Model`] — one `Data` is created per
//! simulation environment via `model.make_data()`.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, UnitQuaternion, Vector3};

// Imports from sibling modules
use super::enums::{ConstraintState, ConstraintType, ResetError, SleepState};
use super::model::Model;

use super::contact_types::Contact;
use super::enums::SolverStat;

// Spatial algebra (extracted in Phase 2)
use crate::dynamics::SpatialVector;
use crate::island::reset_sleep_state;

/// Dynamic simulation state (like mjData).
///
/// All arrays pre-allocated - no heap allocation during simulation.
/// This contains the current state (qpos, qvel) and computed quantities
/// (body poses, forces, contacts).
///
/// # Key Invariant
///
/// `qpos` and `qvel` are the ONLY state variables. Everything else
/// (xpos, xquat, qfrc_*, etc.) is COMPUTED from them via forward dynamics.
#[derive(Debug)]
#[allow(non_snake_case)] // qM matches MuJoCo naming convention
pub struct Data {
    // ==================== Generalized Coordinates (THE source of truth) ====================
    /// Joint positions (length `nq`) - includes quaternion components for ball/free joints.
    pub qpos: DVector<f64>,
    /// Joint velocities (length `nv`).
    pub qvel: DVector<f64>,
    /// Joint accelerations (length `nv`) - computed by forward dynamics.
    pub qacc: DVector<f64>,
    /// Warm-start for constraint solver (length `nv`).
    pub qacc_warmstart: DVector<f64>,

    // ==================== Control / Actuation ====================
    /// Actuator control inputs (length `nu`).
    pub ctrl: DVector<f64>,
    /// Actuator activation states (length `na`) (for muscles/filters).
    pub act: DVector<f64>,
    /// Actuator forces in joint space (length `nv`).
    pub qfrc_actuator: DVector<f64>,

    /// Actuator length (gear * transmission_length, length `nu`).
    /// For Joint transmission: `gear * qpos[qpos_adr]` (hinge/slide only).
    /// For Tendon transmission: `gear * ten_length[tendon_id]`.
    pub actuator_length: Vec<f64>,

    /// Actuator velocity (gear * transmission_velocity, length `nu`).
    /// For Joint transmission: `gear * qvel[dof_adr]` (hinge/slide only).
    /// For Tendon transmission: `gear * ten_velocity[tendon_id]`.
    pub actuator_velocity: Vec<f64>,

    /// Actuator force output (length `nu`).
    /// The scalar force produced by each actuator after gain/bias/activation.
    pub actuator_force: Vec<f64>,

    /// Actuator moment vectors (length `nu`, each nv-dimensional).
    /// Populated for Site transmissions by `mj_transmission_site`.
    /// Maps scalar actuator force to generalized forces: `qfrc += moment * force`.
    pub actuator_moment: Vec<DVector<f64>>,

    /// Activation time-derivative (length `na`).
    /// Computed by `mj_fwd_actuation()`, integrated by the integrator (Euler/RK4).
    /// Separating derivative from integration matches MuJoCo's `mjData.act_dot`
    /// and is required for correct RK4 integration of activation states.
    pub act_dot: DVector<f64>,

    // ==================== Mocap Bodies ====================
    /// Mocap body positions in world frame (length nmocap).
    /// User-settable: modified between steps to drive mocap body poses.
    /// Initialized to the body_pos offset for each mocap body.
    pub mocap_pos: Vec<Vector3<f64>>,
    /// Mocap body orientations in world frame (length nmocap).
    /// User-settable: modified between steps to drive mocap body poses.
    /// Initialized to the body_quat offset for each mocap body.
    pub mocap_quat: Vec<UnitQuaternion<f64>>,

    // ==================== Computed Body States (from FK - outputs, not inputs) ====================
    /// Body positions in world frame (length `nbody`).
    pub xpos: Vec<Vector3<f64>>,
    /// Body orientations in world frame (length `nbody`).
    pub xquat: Vec<UnitQuaternion<f64>>,
    /// Body rotation matrices (cached) (length `nbody`).
    pub xmat: Vec<Matrix3<f64>>,
    /// Body inertial frame positions (length `nbody`).
    pub xipos: Vec<Vector3<f64>>,
    /// Body inertial frame rotations (length `nbody`).
    pub ximat: Vec<Matrix3<f64>>,

    // Geom poses (for collision detection)
    /// Geom positions in world frame (length `ngeom`).
    pub geom_xpos: Vec<Vector3<f64>>,
    /// Geom rotation matrices (length `ngeom`).
    pub geom_xmat: Vec<Matrix3<f64>>,

    // Site poses (for attachment points, sensors)
    /// Site positions in world frame (length `nsite`).
    pub site_xpos: Vec<Vector3<f64>>,
    /// Site rotation matrices (length `nsite`).
    pub site_xmat: Vec<Matrix3<f64>>,
    /// Site orientations in world frame (length `nsite`).
    pub site_xquat: Vec<UnitQuaternion<f64>>,

    // ==================== Flex Vertex Poses ====================
    /// World-frame flex vertex positions (copied from qpos each step).
    /// Length `nflexvert`. Updated by `mj_flex()`.
    pub flexvert_xpos: Vec<Vector3<f64>>,

    // ==================== Velocities (computed from qvel) ====================
    /// Body spatial velocities (length `nbody`): (angular, linear).
    pub cvel: Vec<SpatialVector>,
    /// DOF velocities in Cartesian space (length `nv`).
    pub cdof: Vec<SpatialVector>,

    // ==================== RNE Intermediate Quantities ====================
    /// Body bias accelerations for RNE (Coriolis/centrifugal).
    /// `a_bias[i] = X[i] @ a_bias[parent] + v[i] ×_m S[i] @ qdot[i]`
    pub cacc_bias: Vec<SpatialVector>,
    /// Body forces for RNE backward pass.
    /// `f[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])`
    pub cfrc_bias: Vec<SpatialVector>,

    // ==================== Forces in Generalized Coordinates ====================
    /// User-applied generalized forces (length `nv`).
    pub qfrc_applied: DVector<f64>,
    /// Coriolis + centrifugal + gravity bias forces (length `nv`).
    pub qfrc_bias: DVector<f64>,
    /// Passive forces (springs + dampers + gravcomp + fluid) (length `nv`).
    pub qfrc_passive: DVector<f64>,
    /// Fluid forces (length `nv`), zeroed each step.
    /// Separated from `qfrc_passive` for derivative computation.
    pub qfrc_fluid: DVector<f64>,
    /// Gravity compensation forces (length `nv`), zeroed each step.
    /// Stored separately for future `jnt_actgravcomp` routing.
    pub qfrc_gravcomp: DVector<f64>,
    /// Constraint forces (contacts + joint limits) (length `nv`).
    pub qfrc_constraint: DVector<f64>,

    /// Per-joint limit force cache (length `njnt`).
    /// Populated by `mj_fwd_constraint()`. Contains the unsigned constraint force
    /// magnitude for each joint's active limit constraint. Zero when the joint is
    /// within its limits or is not limited.
    ///
    /// Extracted from `efc_force[]` by scanning for `mjCNSTR_LIMIT_JOINT` entries
    /// after the unified constraint solver runs. Cached per-joint to avoid
    /// re-scanning efc_force at sensor eval time.
    pub jnt_limit_frc: Vec<f64>,

    /// Per-tendon limit force cache (length `ntendon`).
    /// Populated by `mj_fwd_constraint()`. Contains the unsigned constraint force
    /// magnitude for each tendon's active limit constraint. Zero when the tendon
    /// is within its limits or is not limited.
    pub ten_limit_frc: Vec<f64>,

    // Cartesian forces (alternative input method)
    /// Applied spatial forces in world frame (length `nbody`).
    pub xfrc_applied: Vec<SpatialVector>,

    // ==================== Mass Matrix ====================
    /// Joint-space inertia matrix (`nv` x `nv`).
    /// For small systems (nv <= 32), dense storage is used.
    /// For large systems, only lower triangle is filled (sparse via qLD).
    pub qM: DMatrix<f64>,

    // ==================== Sparse L^T D L Factorization ====================
    // Computed in mj_crba() via mj_factor_sparse(). Exploits tree sparsity from
    // dof_parent for O(nv) factorization and solve vs O(nv³) for dense Cholesky.
    // Reused by mj_fwd_acceleration_explicit() and pgs_solve_contacts().
    //
    /// Sparse L^T D L factorization exploiting tree structure.
    /// M = L^T D L where L is unit lower triangular and D is diagonal.
    /// For tree-structured robots, this achieves O(n) factorization and solve.
    ///
    /// Layout matches MuJoCo's combined format — each CSR row stores:
    /// - Off-diagonal L entries at positions `0..rownnz[i]-1`
    /// - Diagonal D\[i,i\] at position `rownnz[i]-1` (last element)
    ///
    /// The sparsity pattern is determined by the kinematic tree:
    /// `L\[i,j\]` is non-zero only if DOF j is an ancestor of DOF i.
    ///
    /// Use `data.qld_diag(model, i)` to read D\[i,i\] from the CSR data.
    ///
    /// Flat CSR value buffer (off-diagonal L + diagonal D).
    /// Layout defined by `Model::qLD_rowadr`/`qLD_rownnz`/`qLD_colind`.
    /// Length: `model.qLD_nnz`.
    pub qLD_data: Vec<f64>,
    /// Precomputed inverse diagonal: `qLD_diag_inv[i] = 1.0 / D\[i,i\]`.
    /// Computed during `mj_factor_sparse()`. Used by `mj_solve_sparse()` to
    /// replace division with multiplication (matching MuJoCo's `qLDiagInv`).
    /// Length: `nv`.
    pub qLD_diag_inv: Vec<f64>,
    /// Whether sparse factorization is valid and should be used.
    /// Set to true after `mj_factor_sparse()` is called.
    pub qLD_valid: bool,

    // ==================== Body and Composite Inertia (for Featherstone CRBA/RNE) ====================
    /// Body spatial inertia in world frame (before composite accumulation).
    /// These are 6×6 spatial inertias for individual bodies, computed once in FK.
    /// Used by both CRBA (as starting point) and RNE (for bias forces).
    /// This is equivalent to MuJoCo's `mjData.cinert`.
    pub cinert: Vec<Matrix6<f64>>,
    /// Composite rigid body inertia in world frame.
    /// These are 6×6 spatial inertias accumulated from subtrees during CRBA.
    /// Starts as a copy of `cinert`, then accumulates child inertias.
    pub crb_inertia: Vec<Matrix6<f64>>,

    // ==================== Subtree Mass/COM (for O(n) RNE gravity) ====================
    /// Total mass of subtree rooted at each body (including the body itself).
    /// Computed during forward kinematics via backward pass.
    pub subtree_mass: Vec<f64>,
    /// Center of mass of subtree in world frame (length `nbody`).
    /// Computed during forward kinematics via backward pass.
    pub subtree_com: Vec<Vector3<f64>>,

    // ==================== Tendon State ====================
    /// Current tendon lengths (length `ntendon`).
    /// Computed from wrap path through kinematics.
    pub ten_length: Vec<f64>,
    /// Tendon velocities (length `ntendon`).
    /// Computed as J_tendon @ qvel.
    pub ten_velocity: Vec<f64>,
    /// Tendon forces from springs/limits (length `ntendon`).
    pub ten_force: Vec<f64>,
    /// Tendon Jacobian: d(length)/d(qpos) (sparse, length varies).
    /// Maps tendon length changes to joint velocities.
    pub ten_J: Vec<DVector<f64>>,

    /// Tendon wrap path point positions in world frame.
    /// Flat storage: `wrap_xpos[ten_wrapadr[t] .. ten_wrapadr[t] + ten_wrapnum[t]]`
    /// gives the ordered path points for tendon `t`.
    /// Allocated with capacity `nwrap * 2` (upper bound).
    pub wrap_xpos: Vec<Vector3<f64>>,

    /// Object marker for each wrap path point.
    /// -2 = pulley, -1 = site, >= 0 = geom id (tangent point on wrapping surface).
    /// Parallel to `wrap_xpos`.
    pub wrap_obj: Vec<i32>,

    /// Start index in `wrap_xpos`/`wrap_obj` for each tendon (length `ntendon`).
    pub ten_wrapadr: Vec<usize>,

    /// Number of path points for each tendon (length `ntendon`).
    pub ten_wrapnum: Vec<usize>,

    // ==================== Equality Constraint State ====================
    /// Equality constraint violation (length `neq` * max_dim).
    /// For Connect: 3D position error. For Weld: 6D pose error.
    pub eq_violation: Vec<f64>,
    /// Equality constraint forces (Lagrange multipliers).
    pub eq_force: Vec<f64>,

    // ==================== Contacts ====================
    /// Active contacts (pre-allocated with capacity).
    pub contacts: Vec<Contact>,
    /// Number of active contacts (`contacts.len()` but tracked explicitly).
    pub ncon: usize,

    // ==================== Solver State ====================
    /// Iterations used in last constraint solve.
    pub solver_niter: usize,
    /// Non-zeros in constraint Jacobian.
    pub solver_nnz: usize,
    // ==================== Unified Constraint System (Newton solver, §15) ====================
    // These fields are populated by `assemble_unified_constraints()` and consumed
    // by the Newton solver loop. For PGS/CG solvers, they remain empty (zero-length).
    /// Friction loss component of qfrc_passive (length nv). Separated from
    /// qfrc_passive for Newton solver which treats friction loss as explicit
    /// constraint rows (§15.0 approach b).
    pub qfrc_frictionloss: DVector<f64>,

    /// Smooth unconstrained acceleration: M⁻¹·qfrc_smooth (length nv).
    /// Computed once per step by `compute_qacc_smooth()`, used by all solvers.
    pub qacc_smooth: DVector<f64>,
    /// Smooth unconstrained force: applied + actuator + passive − bias (length nv).
    /// Computed once per step by `compute_qacc_smooth()`, used by all solvers.
    pub qfrc_smooth: DVector<f64>,

    /// Constraint bias: J·qacc_smooth − aref (length nefc). Computed during
    /// assembly, consumed by warmstart cost comparison (§15.8 INITIALIZE).
    /// Persists in Data for diagnostic access (matches MuJoCo's d->efc_b),
    /// but is NOT updated by the solver loop.
    pub efc_b: DVector<f64>,
    /// Unified constraint Jacobian (nefc × nv). Rows ordered: equality,
    /// friction loss, joint limits, tendon limits, contacts.
    pub efc_J: DMatrix<f64>,
    /// Per-row constraint type annotation.
    pub efc_type: Vec<ConstraintType>,
    /// Per-row constraint violation (penetration distance for contacts).
    pub efc_pos: Vec<f64>,
    /// Per-row margin (geom margin for contacts, 0 for non-contacts).
    pub efc_margin: Vec<f64>,
    /// Per-row constraint-space velocity: J·qvel.
    pub efc_vel: DVector<f64>,
    /// Per-row solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub efc_solref: Vec<[f64; 2]>,
    /// Per-row solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub efc_solimp: Vec<[f64; 5]>,
    /// Per-row diagonal approximation of A = J·M⁻¹·J^T (for regularization scaling).
    #[allow(non_snake_case)]
    pub efc_diagApprox: Vec<f64>,
    /// Per-row regularization R (constraint softness).
    #[allow(non_snake_case)]
    pub efc_R: Vec<f64>,
    /// Per-row D = 1/R (constraint stiffness, inverse of regularization).
    #[allow(non_snake_case)]
    pub efc_D: Vec<f64>,
    /// Per-row impedance value (from solimp sigmoid, in [dmin, dmax]).
    pub efc_imp: Vec<f64>,
    // Note: MuJoCo packs K, B, imp, and impP into a single `efc_KBIP[4*i+{0,1,2,3}]`
    // array. We decompose this: K and B are transient (used only during `aref`
    // computation, not stored); `imp` is stored as `efc_imp`; `impP` (the derivative
    // of impedance w.r.t. penetration distance) is omitted — it is computed and stored
    // by MuJoCo but never read by any MuJoCo subsystem (solvers, sensors, derivatives).
    // It exists solely for external API introspection. If external parity is needed,
    // Phase C can add `efc_impP: Vec<f64>`.
    /// Per-row reference acceleration (aref = -B·vel - K·imp·(pos-margin)).
    pub efc_aref: DVector<f64>,
    /// Per-row friction loss saturation value (R * floss for Huber threshold).
    pub efc_floss: Vec<f64>,
    /// Per-row friction coefficients. For contacts: [mu1, mu2, ...] from geom.
    /// For non-contacts: [0; 5].
    pub efc_mu: Vec<[f64; 5]>,
    /// Per-row condim of parent contact (1 for non-contact rows).
    pub efc_dim: Vec<usize>,
    /// Per-row source object index (joint ID for LimitJoint, tendon ID for
    /// LimitTendon, equality ID for Equality, contact index for Contact*,
    /// DOF/tendon index for FrictionLoss).
    pub efc_id: Vec<usize>,

    // Solver output (written to Data in RECOVER after convergence)
    /// Per-row constraint state (Quadratic, Satisfied, LinearNeg/Pos, Cone).
    pub efc_state: Vec<ConstraintState>,
    /// Per-row constraint force (length nefc).
    pub efc_force: DVector<f64>,
    /// Per-row J·qacc − aref (constraint residual, length nefc).
    pub efc_jar: DVector<f64>,
    /// Total cost (Gauss + constraint terms) at convergence.
    pub efc_cost: f64,
    /// Per-contact cone Hessian matrices (dim x dim). Indexed parallel to contacts.
    /// Only populated for contacts in the Cone state. `None` for non-cone or Satisfied contacts.
    pub efc_cone_hessian: Vec<Option<DMatrix<f64>>>,
    /// Number of contacts currently in the Cone state.
    pub ncone: usize,
    /// Number of equality constraint rows (Equality + FlexEdge).
    /// These occupy `efc_*[0..ne)`.
    pub ne: usize,
    /// Number of friction loss constraint rows (DOF + tendon).
    /// These occupy `efc_*[ne..ne+nf)`.
    pub nf: usize,
    /// Whether Newton solver computed qacc directly this step.
    /// When true, mj_fwd_acceleration should be skipped.
    pub newton_solved: bool,

    /// Per-iteration Newton solver statistics (length `solver_niter` after solve).
    /// Populated by Newton solver; empty for PGS/CG.
    pub solver_stat: Vec<SolverStat>,

    /// Per-step mean inertia: `trace(qM) / nv`. More accurate than the
    /// model-level constant for configuration-dependent inertia.
    /// Updated at the start of each Newton solve from the current `qM`.
    pub stat_meaninertia: f64,

    // ==================== Sensors ====================
    /// Sensor data array (length `nsensordata`).
    /// Each sensor writes to `sensordata[sensor_adr[i]..sensor_adr[i]+sensor_dim[i]]`.
    pub sensordata: DVector<f64>,

    // ==================== Energy (for debugging/validation) ====================
    /// Potential energy (gravity + springs).
    pub energy_potential: f64,
    /// Kinetic energy.
    pub energy_kinetic: f64,

    // ==================== Sleep State (§16.1) ====================
    /// Per-tree sleep timer (length `ntree`).
    /// `< 0`: Tree is awake. Countdown from `-(1 + MIN_AWAKE)` toward `-1`.
    /// `≥ 0`: Tree is asleep (self-link in Phase A).
    pub tree_asleep: Vec<i32>,
    /// Per-tree awake flag for fast branching (length `ntree`).
    pub tree_awake: Vec<bool>,
    /// Per-body sleep state for efficient per-body queries (length `nbody`).
    pub body_sleep_state: Vec<SleepState>,
    /// Number of awake trees (diagnostics / early-exit).
    pub ntree_awake: usize,
    /// Number of awake DOFs (diagnostics / performance monitoring).
    pub nv_awake: usize,

    // ==================== Awake-Index Indirection (§16.17) ====================
    /// Sorted indices of awake + static bodies (length ≤ nbody).
    /// Used for cache-friendly iteration in FK, velocity kinematics, etc.
    pub body_awake_ind: Vec<usize>,
    /// Number of entries in `body_awake_ind`.
    pub nbody_awake: usize,
    /// Sorted indices of bodies whose parent is awake or static (length ≤ nbody).
    /// Used for backward-pass loops (subtree COM, RNE).
    pub parent_awake_ind: Vec<usize>,
    /// Number of entries in `parent_awake_ind`.
    pub nparent_awake: usize,
    /// Sorted indices of awake DOFs (length ≤ nv).
    /// Used for velocity integration, CRBA, force accumulation.
    pub dof_awake_ind: Vec<usize>,

    // ==================== Island Discovery (§16.11) ====================
    /// Number of constraint islands discovered this step.
    pub nisland: usize,
    /// Island index for each tree. -1 if unconstrained singleton. Length: ntree.
    pub tree_island: Vec<i32>,
    /// Number of trees per island. Length: ≤ ntree.
    pub island_ntree: Vec<usize>,
    /// Start index in `map_itree2tree` for each island. Length: ≤ ntree.
    pub island_itreeadr: Vec<usize>,
    /// Packed tree indices grouped by island. Length: ntree.
    pub map_itree2tree: Vec<usize>,
    /// Island index for each DOF. -1 if unconstrained. Length: nv.
    pub dof_island: Vec<i32>,
    /// Number of DOFs per island. Length: ≤ ntree.
    pub island_nv: Vec<usize>,
    /// Start index in `map_idof2dof` for each island. Length: ≤ ntree.
    pub island_idofadr: Vec<usize>,
    /// DOF → island-local DOF index. Length: nv.
    pub map_dof2idof: Vec<i32>,
    /// Island-local DOF → global DOF. Length: nv.
    pub map_idof2dof: Vec<usize>,
    /// Island index for each constraint row. Resized per step.
    pub efc_island: Vec<i32>,
    /// Per-island constraint row count. Length: ≤ ntree.
    pub island_nefc: Vec<usize>,
    /// Start index in `map_iefc2efc` for each island. Length: ≤ ntree.
    pub island_iefcadr: Vec<usize>,
    /// Global constraint row → island-local row. Resized per step.
    pub map_efc2iefc: Vec<i32>,
    /// Island-local row → global constraint row. Resized per step.
    pub map_iefc2efc: Vec<usize>,
    /// Island assignment for each contact. Length: data.contacts.len(). Resized per step.
    /// -1 = not in any island (e.g., contact between two world-body geoms).
    pub contact_island: Vec<i32>,

    // ==================== Island Scratch Space (§16.11) ====================
    /// DFS stack for flood-fill. Length: ntree.
    pub island_scratch_stack: Vec<usize>,
    /// Per-tree edge counts (CSR rownnz). Length: ntree.
    pub island_scratch_rownnz: Vec<usize>,
    /// Per-tree CSR row pointers. Length: ntree.
    pub island_scratch_rowadr: Vec<usize>,
    /// CSR column indices (edge targets). Resized per step.
    pub island_scratch_colind: Vec<usize>,

    // ==================== qpos Change Detection (§16.15) ====================
    /// Per-tree dirty flag set by mj_kinematics1() when a sleeping body's
    /// xpos/xquat changed. Read/cleared by mj_check_qpos_changed(). Length: ntree.
    pub tree_qpos_dirty: Vec<bool>,

    // ==================== Time ====================
    /// Simulation time in seconds.
    pub time: f64,

    // ==================== Scratch Buffers (for allocation-free stepping) ====================
    /// Scratch matrix for implicit integration: M + h*D + h²*K.
    /// Pre-allocated to avoid O(n²) allocation per step.
    pub scratch_m_impl: DMatrix<f64>,
    /// Scratch vector for force accumulation (length `nv`).
    pub scratch_force: DVector<f64>,
    /// Scratch vector for RHS of linear solves (length `nv`).
    pub scratch_rhs: DVector<f64>,
    /// Scratch vector for new velocity in implicit solve (length `nv`).
    /// Used to hold v_new while computing qacc = (v_new - v_old) / h.
    pub scratch_v_new: DVector<f64>,
    /// Pivot permutation for LU factorization in `Integrator::Implicit` (length `nv`).
    /// Stores row swap indices from partial pivoting. Persists after forward pass
    /// for reuse by derivative column solves in `mjd_transition_hybrid`.
    pub scratch_lu_piv: Vec<usize>,

    // ==================== RK4 Scratch Buffers ====================
    // State and rate buffers for 4-stage Runge-Kutta integration.
    // Pre-allocated in make_data() — no heap allocation during stepping.
    /// Saved initial position for RK4: `X[0].qpos` (length nq).
    /// Read at every stage (step 2c) and the final advance (step 4).
    pub rk4_qpos_saved: DVector<f64>,

    /// Scratch position for the current RK4 stage (length nq).
    /// Written at step 2c, copied to data.qpos at step 2e, then overwritten
    /// next iteration. Intermediate positions are not needed for the final
    /// B-weight combination (which only sums velocities and accelerations).
    pub rk4_qpos_stage: DVector<f64>,

    /// RK4 stage velocities: `X[i].qvel` (4 buffers, each length nv).
    /// All 4 are needed for the final B-weight combination in step 3.
    pub rk4_qvel: [DVector<f64>; 4],

    /// RK4 stage accelerations: `F[i].qacc` (4 buffers, each length nv).
    /// All 4 are needed for the final B-weight combination in step 3.
    pub rk4_qacc: [DVector<f64>; 4],

    /// Weighted velocity sum for manifold position integration (length nv).
    pub rk4_dX_vel: DVector<f64>,

    /// Weighted acceleration sum for velocity update (length nv).
    pub rk4_dX_acc: DVector<f64>,

    /// RK4 saved activation state (length na). Stores initial `act` before RK4 stages.
    pub rk4_act_saved: DVector<f64>,

    /// RK4 stage activation derivatives: `act_dot` at each stage (4 buffers, each length na).
    pub rk4_act_dot: [DVector<f64>; 4],

    // ==================== Derivative Scratch Buffers ====================
    /// Analytical derivative of smooth forces w.r.t. velocity: ∂(qfrc_smooth)/∂qvel.
    /// Dense nv × nv matrix. Populated by `mjd_smooth_vel()`.
    ///
    /// Components:
    ///   ∂(qfrc_passive)/∂qvel  = diagonal damping + tendon damping J^T·b·J
    ///                            (all integrators, including ImplicitSpringDamper per DT-35)
    ///   ∂(qfrc_actuator)/∂qvel = affine velocity-dependent gain/bias terms
    ///   −∂(qfrc_bias)/∂qvel    = −C(q,v) (Coriolis matrix)
    ///
    /// MuJoCo equivalent: `mjData.qDeriv` (sparse nv×nv). Dense here because
    /// nv < 100 for target use cases.
    #[allow(non_snake_case)]
    pub qDeriv: DMatrix<f64>,

    /// Scratch Jacobian ∂(cvel)/∂(qvel) per body (length `nbody`, each 6 × nv).
    /// Used by `mjd_rne_vel()` for chain-rule derivative propagation.
    #[allow(non_snake_case)]
    pub deriv_Dcvel: Vec<DMatrix<f64>>,
    /// Scratch Jacobian ∂(cacc)/∂(qvel) per body (length `nbody`, each 6 × nv).
    #[allow(non_snake_case)]
    pub deriv_Dcacc: Vec<DMatrix<f64>>,
    /// Scratch Jacobian ∂(cfrc)/∂(qvel) per body (length `nbody`, each 6 × nv).
    #[allow(non_snake_case)]
    pub deriv_Dcfrc: Vec<DMatrix<f64>>,

    // ==================== Cached Body Effective Mass/Inertia ====================
    // These are extracted from the mass matrix diagonal during forward() and cached
    // for use by constraint force limiting. This avoids O(joints) traversal per constraint.
    /// Minimum translational mass for each body (length `nbody`).
    /// Extracted from qM diagonal for linear DOFs (free joint indices 0-2, slide joints).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_mass: Vec<f64>,

    /// Minimum rotational inertia for each body (length `nbody`).
    /// Extracted from qM diagonal for angular DOFs (free joint 3-5, ball 0-2, hinge).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_inertia: Vec<f64>,
}

#[allow(non_snake_case)] // qM, qLD matches MuJoCo naming convention
impl Clone for Data {
    fn clone(&self) -> Self {
        Self {
            // Generalized coordinates
            qpos: self.qpos.clone(),
            qvel: self.qvel.clone(),
            qacc: self.qacc.clone(),
            qacc_warmstart: self.qacc_warmstart.clone(),
            // Control / Actuation
            ctrl: self.ctrl.clone(),
            act: self.act.clone(),
            qfrc_actuator: self.qfrc_actuator.clone(),
            actuator_length: self.actuator_length.clone(),
            actuator_velocity: self.actuator_velocity.clone(),
            actuator_force: self.actuator_force.clone(),
            actuator_moment: self.actuator_moment.clone(),
            act_dot: self.act_dot.clone(),
            // Mocap bodies
            mocap_pos: self.mocap_pos.clone(),
            mocap_quat: self.mocap_quat.clone(),
            // Computed body states
            xpos: self.xpos.clone(),
            xquat: self.xquat.clone(),
            xmat: self.xmat.clone(),
            xipos: self.xipos.clone(),
            ximat: self.ximat.clone(),
            geom_xpos: self.geom_xpos.clone(),
            geom_xmat: self.geom_xmat.clone(),
            site_xpos: self.site_xpos.clone(),
            site_xmat: self.site_xmat.clone(),
            site_xquat: self.site_xquat.clone(),
            // Flex vertex poses
            flexvert_xpos: self.flexvert_xpos.clone(),
            // Velocities
            cvel: self.cvel.clone(),
            cdof: self.cdof.clone(),
            // RNE intermediate
            cacc_bias: self.cacc_bias.clone(),
            cfrc_bias: self.cfrc_bias.clone(),
            // Forces
            qfrc_applied: self.qfrc_applied.clone(),
            qfrc_bias: self.qfrc_bias.clone(),
            qfrc_passive: self.qfrc_passive.clone(),
            qfrc_fluid: self.qfrc_fluid.clone(),
            qfrc_gravcomp: self.qfrc_gravcomp.clone(),
            qfrc_constraint: self.qfrc_constraint.clone(),
            jnt_limit_frc: self.jnt_limit_frc.clone(),
            ten_limit_frc: self.ten_limit_frc.clone(),
            xfrc_applied: self.xfrc_applied.clone(),
            // Mass matrix
            qM: self.qM.clone(),
            // Sparse factorization
            qLD_data: self.qLD_data.clone(),
            qLD_diag_inv: self.qLD_diag_inv.clone(),
            qLD_valid: self.qLD_valid,
            // Body/composite inertia
            cinert: self.cinert.clone(),
            crb_inertia: self.crb_inertia.clone(),
            // Subtree mass/COM
            subtree_mass: self.subtree_mass.clone(),
            subtree_com: self.subtree_com.clone(),
            // Tendon state
            ten_length: self.ten_length.clone(),
            ten_velocity: self.ten_velocity.clone(),
            ten_force: self.ten_force.clone(),
            ten_J: self.ten_J.clone(),
            // Tendon wrap visualization
            wrap_xpos: self.wrap_xpos.clone(),
            wrap_obj: self.wrap_obj.clone(),
            ten_wrapadr: self.ten_wrapadr.clone(),
            ten_wrapnum: self.ten_wrapnum.clone(),
            // Equality constraints
            eq_violation: self.eq_violation.clone(),
            eq_force: self.eq_force.clone(),
            // Contacts
            contacts: self.contacts.clone(),
            ncon: self.ncon,
            // Solver state
            solver_niter: self.solver_niter,
            solver_nnz: self.solver_nnz,
            // Unified constraint system (Newton solver)
            qfrc_frictionloss: self.qfrc_frictionloss.clone(),
            qacc_smooth: self.qacc_smooth.clone(),
            qfrc_smooth: self.qfrc_smooth.clone(),
            efc_b: self.efc_b.clone(),
            efc_J: self.efc_J.clone(),
            efc_type: self.efc_type.clone(),
            efc_pos: self.efc_pos.clone(),
            efc_margin: self.efc_margin.clone(),
            efc_vel: self.efc_vel.clone(),
            efc_solref: self.efc_solref.clone(),
            efc_solimp: self.efc_solimp.clone(),
            efc_diagApprox: self.efc_diagApprox.clone(),
            efc_R: self.efc_R.clone(),
            efc_D: self.efc_D.clone(),
            efc_imp: self.efc_imp.clone(),
            efc_aref: self.efc_aref.clone(),
            efc_floss: self.efc_floss.clone(),
            efc_mu: self.efc_mu.clone(),
            efc_dim: self.efc_dim.clone(),
            efc_id: self.efc_id.clone(),
            efc_state: self.efc_state.clone(),
            efc_force: self.efc_force.clone(),
            efc_jar: self.efc_jar.clone(),
            efc_cost: self.efc_cost,
            efc_cone_hessian: self.efc_cone_hessian.clone(),
            ncone: self.ncone,
            ne: self.ne,
            nf: self.nf,
            newton_solved: self.newton_solved,
            solver_stat: self.solver_stat.clone(),
            stat_meaninertia: self.stat_meaninertia,
            // Sensors
            sensordata: self.sensordata.clone(),
            // Energy
            energy_potential: self.energy_potential,
            energy_kinetic: self.energy_kinetic,
            // Sleep state
            tree_asleep: self.tree_asleep.clone(),
            tree_awake: self.tree_awake.clone(),
            body_sleep_state: self.body_sleep_state.clone(),
            ntree_awake: self.ntree_awake,
            nv_awake: self.nv_awake,
            // Awake-index indirection (§16.17)
            body_awake_ind: self.body_awake_ind.clone(),
            nbody_awake: self.nbody_awake,
            parent_awake_ind: self.parent_awake_ind.clone(),
            nparent_awake: self.nparent_awake,
            dof_awake_ind: self.dof_awake_ind.clone(),
            // Island discovery (§16.11)
            nisland: self.nisland,
            tree_island: self.tree_island.clone(),
            island_ntree: self.island_ntree.clone(),
            island_itreeadr: self.island_itreeadr.clone(),
            map_itree2tree: self.map_itree2tree.clone(),
            dof_island: self.dof_island.clone(),
            island_nv: self.island_nv.clone(),
            island_idofadr: self.island_idofadr.clone(),
            map_dof2idof: self.map_dof2idof.clone(),
            map_idof2dof: self.map_idof2dof.clone(),
            efc_island: self.efc_island.clone(),
            island_nefc: self.island_nefc.clone(),
            island_iefcadr: self.island_iefcadr.clone(),
            map_efc2iefc: self.map_efc2iefc.clone(),
            map_iefc2efc: self.map_iefc2efc.clone(),
            contact_island: self.contact_island.clone(),
            // Island scratch
            island_scratch_stack: self.island_scratch_stack.clone(),
            island_scratch_rownnz: self.island_scratch_rownnz.clone(),
            island_scratch_rowadr: self.island_scratch_rowadr.clone(),
            island_scratch_colind: self.island_scratch_colind.clone(),
            // qpos change detection
            tree_qpos_dirty: self.tree_qpos_dirty.clone(),
            // Time
            time: self.time,
            // Scratch buffers
            scratch_m_impl: self.scratch_m_impl.clone(),
            scratch_force: self.scratch_force.clone(),
            scratch_rhs: self.scratch_rhs.clone(),
            scratch_v_new: self.scratch_v_new.clone(),
            scratch_lu_piv: self.scratch_lu_piv.clone(),
            // RK4 scratch
            rk4_qpos_saved: self.rk4_qpos_saved.clone(),
            rk4_qpos_stage: self.rk4_qpos_stage.clone(),
            rk4_qvel: self.rk4_qvel.clone(),
            rk4_qacc: self.rk4_qacc.clone(),
            rk4_dX_vel: self.rk4_dX_vel.clone(),
            rk4_dX_acc: self.rk4_dX_acc.clone(),
            rk4_act_saved: self.rk4_act_saved.clone(),
            rk4_act_dot: self.rk4_act_dot.clone(),
            // Derivative scratch buffers
            qDeriv: self.qDeriv.clone(),
            deriv_Dcvel: self.deriv_Dcvel.clone(),
            deriv_Dcacc: self.deriv_Dcacc.clone(),
            deriv_Dcfrc: self.deriv_Dcfrc.clone(),
            // Cached body mass/inertia
            body_min_mass: self.body_min_mass.clone(),
            body_min_inertia: self.body_min_inertia.clone(),
        }
    }
}

#[allow(non_snake_case)] // qLD matches MuJoCo naming convention
impl Data {
    /// Read the diagonal entry `D\[i,i\]` from the sparse LDL factorization.
    ///
    /// Reads from `qLD_data[rowadr[i] + rownnz[i] - 1]` (last element of CSR row).
    #[inline]
    #[must_use]
    pub fn qld_diag(&self, model: &Model, i: usize) -> f64 {
        let (rowadr, rownnz, _) = model.qld_csr();
        self.qLD_data[rowadr[i] + rownnz[i] - 1]
    }

    /// Reset state to model defaults.
    pub fn reset(&mut self, model: &Model) {
        self.qpos = model.qpos0.clone();
        self.qvel.fill(0.0);
        self.qacc.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.ctrl.fill(0.0);
        self.act.fill(0.0);
        self.act_dot.fill(0.0);
        self.actuator_length.fill(0.0);
        self.actuator_velocity.fill(0.0);
        self.actuator_force.fill(0.0);
        self.sensordata.fill(0.0);
        self.time = 0.0;
        self.ncon = 0;
        self.contacts.clear();

        // Reset mocap poses to model defaults (body_pos/body_quat offsets).
        let mut mocap_idx = 0;
        for (body_id, mid) in model.body_mocapid.iter().enumerate() {
            if mid.is_some() {
                self.mocap_pos[mocap_idx] = model.body_pos[body_id];
                self.mocap_quat[mocap_idx] = model.body_quat[body_id];
                mocap_idx += 1;
            }
        }

        // Reset sleep state from model policies (§16.7).
        reset_sleep_state(model, self);

        // Clear island discovery state (will be recomputed on next forward()).
        self.nisland = 0;
        self.tree_island[..model.ntree].fill(-1);
        self.contact_island.clear();
    }

    /// Reset simulation state to a keyframe by index.
    ///
    /// Overwrites `time`, `qpos`, `qvel`, `act`, `ctrl`, `mocap_pos`, and
    /// `mocap_quat` from the keyframe. Clears derived quantities (`qacc`,
    /// `qacc_warmstart`, actuator arrays, `sensordata`, contacts).
    /// Does **not** clear `qfrc_applied` or `xfrc_applied` — matching the
    /// convention of `Data::reset()`, which also preserves user-applied forces.
    /// Caller must invoke `forward()` after reset to recompute derived state.
    ///
    /// # Errors
    ///
    /// Returns `Err` if `keyframe_idx >= model.nkeyframe`.
    pub fn reset_to_keyframe(
        &mut self,
        model: &Model,
        keyframe_idx: usize,
    ) -> Result<(), ResetError> {
        let kf = model
            .keyframes
            .get(keyframe_idx)
            .ok_or(ResetError::InvalidKeyframeIndex {
                index: keyframe_idx,
                nkeyframe: model.nkeyframe,
            })?;

        // Overwrite primary state from keyframe.
        self.time = kf.time;
        self.qpos.copy_from(&kf.qpos);
        self.qvel.copy_from(&kf.qvel);
        self.act.copy_from(&kf.act);
        self.ctrl.copy_from(&kf.ctrl);

        // Mocap state (length may be 0 if no mocap bodies).
        self.mocap_pos.copy_from_slice(&kf.mpos);
        self.mocap_quat.copy_from_slice(&kf.mquat);

        // Clear derived quantities (matching Data::reset() convention).
        self.qacc.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.act_dot.fill(0.0);
        self.actuator_length.fill(0.0);
        self.actuator_velocity.fill(0.0);
        self.actuator_force.fill(0.0);
        self.sensordata.fill(0.0);
        self.ncon = 0;
        self.contacts.clear();

        // Reset sleep state from model policies (§16.7).
        reset_sleep_state(model, self);

        Ok(())
    }
}
