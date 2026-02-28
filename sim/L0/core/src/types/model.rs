//! Model struct definition and field accessors.
//!
//! [`Model`] is the static, immutable description of the simulated system:
//! kinematic tree, joint definitions, geometries, constraints, and options.
//! It is constructed by sim-mjcf's model builder and shared (read-only)
//! across all pipeline stages.

use nalgebra::{DVector, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

// Imports from sibling modules
use super::enums::{
    ActuatorDynamics, ActuatorTransmission, BiasType, EqualityType, GainType, GeomType, Integrator,
    InterpolationType, MjJointType, MjObjectType, MjSensorDataType, MjSensorType, SleepPolicy,
    SolverType, TendonType, WrapType,
};

// Imports from sibling modules
use super::contact_types::ContactPair;
use super::keyframe::Keyframe;

// External crate types used by Model fields
use crate::heightfield::HeightFieldData;
use crate::mesh::TriangleMeshData;
use crate::sdf::SdfCollisionData;

/// Static model definition (like mjModel).
///
/// Immutable after construction - all memory allocated upfront.
/// This contains the kinematic tree structure, body properties,
/// joint properties, and simulation options.
///
/// # Memory Layout
///
/// Arrays are indexed by their respective IDs:
/// - `body_*` arrays indexed by `body_id` (0 = world)
/// - `jnt_*` arrays indexed by `joint_id`
/// - `dof_*` arrays indexed by `dof_id` (velocity dimension index)
/// - `geom_*` arrays indexed by `geom_id`
/// - `actuator_*` arrays indexed by `actuator_id`
#[derive(Debug, Clone)]
#[allow(non_snake_case)] // qLD_* matches MuJoCo naming convention
pub struct Model {
    // ==================== Metadata ====================
    /// Model name (from MJCF model attribute or URDF robot name).
    pub name: String,

    // ==================== Dimensions ====================
    /// Number of generalized position coordinates (includes quaternions).
    pub nq: usize,
    /// Number of generalized velocity coordinates (DOFs, always <= nq).
    pub nv: usize,
    /// Number of bodies (including world body 0).
    pub nbody: usize,
    /// Number of joints.
    pub njnt: usize,
    /// Number of collision geometries.
    pub ngeom: usize,
    /// Number of sites (attachment points).
    pub nsite: usize,
    /// Number of actuators.
    pub nu: usize,
    /// Number of activation states (for muscle/filter actuators).
    pub na: usize,
    /// Number of mocap bodies. Mocap arrays are indexed 0..nmocap.
    pub nmocap: usize,
    /// Number of keyframes parsed from MJCF `<keyframe>`.
    pub nkeyframe: usize,

    // ==================== Flex Dimensions ====================
    /// Number of flex objects.
    pub nflex: usize,
    /// Total flex vertices across all flex objects.
    pub nflexvert: usize,
    /// Total flex edges across all flex objects.
    pub nflexedge: usize,
    /// Total flex elements (triangles for dim=2, tetrahedra for dim=3).
    pub nflexelem: usize,
    /// Total bending hinges (pairs of adjacent elements sharing an edge).
    pub nflexhinge: usize,
    // (§27F) nq_rigid/nv_rigid removed — flex DOFs are now real body DOFs.

    // ==================== Kinematic Trees (§16.0) ====================
    /// Number of kinematic trees (excluding world body).
    pub ntree: usize,
    /// First body index for tree `t` (length `ntree`).
    pub tree_body_adr: Vec<usize>,
    /// Number of bodies in tree `t` (length `ntree`).
    pub tree_body_num: Vec<usize>,
    /// First DOF index for tree `t` (length `ntree`).
    pub tree_dof_adr: Vec<usize>,
    /// Number of DOFs in tree `t` (length `ntree`).
    pub tree_dof_num: Vec<usize>,
    /// Tree index for each body (body 0 → `usize::MAX` sentinel, length `nbody`).
    pub body_treeid: Vec<usize>,
    /// Tree index for each DOF (length `nv`).
    pub dof_treeid: Vec<usize>,
    /// Per-tree sleep policy (computed at model build, length `ntree`).
    pub tree_sleep_policy: Vec<SleepPolicy>,
    /// Per-DOF length scale for sleep threshold normalization (length `nv`).
    /// Translational DOFs = 1.0; rotational DOFs = mechanism length estimate.
    pub dof_length: Vec<f64>,
    /// Sleep velocity tolerance. Bodies with all DOF velocities below
    /// `sleep_tolerance * dof_length[dof]` for `MIN_AWAKE` consecutive steps
    /// are eligible for sleep. Default: `1e-4`. Units: `[m/s]`.
    pub sleep_tolerance: f64,

    // ==================== Body Tree (indexed by body_id, 0 = world) ====================
    /// Parent body index (0 for root bodies attached to world).
    pub body_parent: Vec<usize>,
    /// Root body of kinematic tree (for multi-tree systems).
    pub body_rootid: Vec<usize>,
    /// First joint index for this body in jnt_* arrays.
    pub body_jnt_adr: Vec<usize>,
    /// Number of joints attached to this body.
    pub body_jnt_num: Vec<usize>,
    /// First DOF index for this body in dof_* and qvel arrays.
    pub body_dof_adr: Vec<usize>,
    /// Number of DOFs for this body.
    pub body_dof_num: Vec<usize>,
    /// First geom index for this body.
    pub body_geom_adr: Vec<usize>,
    /// Number of geoms attached to this body.
    pub body_geom_num: Vec<usize>,

    // Body properties (in body-local frame)
    /// Position offset from parent joint frame to body frame.
    pub body_pos: Vec<Vector3<f64>>,
    /// Orientation offset from parent joint frame to body frame.
    pub body_quat: Vec<UnitQuaternion<f64>>,
    /// Center of mass position in body frame.
    pub body_ipos: Vec<Vector3<f64>>,
    /// Inertial frame orientation in body frame.
    pub body_iquat: Vec<UnitQuaternion<f64>>,
    /// Body mass in kg.
    pub body_mass: Vec<f64>,
    /// Diagonal inertia in principal axes (`body_iquat` frame).
    pub body_inertia: Vec<Vector3<f64>>,
    /// Optional body names for lookup.
    pub body_name: Vec<Option<String>>,
    /// Total mass of subtree rooted at this body (precomputed).
    /// `body_subtreemass[0]` is total mass of entire system.
    pub body_subtreemass: Vec<f64>,
    /// Maps body_id to mocap array index. `None` for non-mocap bodies.
    /// Length: nbody.
    ///
    /// **Ordering invariant:** Mocap IDs are assigned sequentially during body
    /// processing in topological order. Since the body traversal visits
    /// parents before children, and mocap bodies must be direct world children,
    /// all mocap bodies are visited before any non-world-child body. The
    /// mocap_id for body B equals the count of mocap bodies with body_id < B.
    /// This invariant is relied upon by `make_data()` and `Data::reset()`,
    /// which iterate bodies in order and use `enumerate()` to correlate with
    /// mocap array indices.
    pub body_mocapid: Vec<Option<usize>>,
    /// Per-body gravity compensation factor (length `nbody`).
    /// 0=none, 1=full, >1=over-compensate, <0=amplify gravity.
    pub body_gravcomp: Vec<f64>,
    /// Count of bodies with `body_gravcomp != 0.0` (early-exit optimization).
    pub ngravcomp: usize,

    // ==================== Joints (indexed by jnt_id) ====================
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: Vec<MjJointType>,
    /// Body this joint belongs to (the child body).
    pub jnt_body: Vec<usize>,
    /// Start index in qpos array.
    pub jnt_qpos_adr: Vec<usize>,
    /// Start index in qvel/qacc arrays.
    pub jnt_dof_adr: Vec<usize>,
    /// Joint anchor position in body frame.
    pub jnt_pos: Vec<Vector3<f64>>,
    /// Joint axis for hinge/slide (in body frame).
    pub jnt_axis: Vec<Vector3<f64>>,
    /// Whether joint has limits.
    pub jnt_limited: Vec<bool>,
    /// Joint limits [min, max].
    pub jnt_range: Vec<(f64, f64)>,
    /// Spring stiffness coefficient (N/rad for hinge, N/m for slide).
    /// Applied as: τ = -stiffness * (q - springref)
    pub jnt_stiffness: Vec<f64>,
    /// Spring equilibrium position (rad for hinge, m for slide).
    /// This is the position where spring force is zero.
    /// Distinct from `qpos0` which is the initial position at model load.
    pub jnt_springref: Vec<f64>,
    /// Damping coefficient (Ns/rad for hinge, Ns/m for slide).
    /// Applied as: τ = -damping * qvel
    pub jnt_damping: Vec<f64>,
    /// Armature inertia (motor rotor inertia).
    pub jnt_armature: Vec<f64>,
    /// Solver reference parameters for joint limits [timeconst, dampratio].
    /// Controls how stiffly/softly limits are enforced.
    /// Default: [0.02, 1.0] (MuJoCo defaults)
    pub jnt_solref: Vec<[f64; 2]>,
    /// Solver impedance parameters for joint limits [d0, d_width, width, midpoint, power].
    /// Default: [0.9, 0.95, 0.001, 0.5, 2.0] (MuJoCo defaults)
    pub jnt_solimp: Vec<[f64; 5]>,
    /// Optional joint names.
    pub jnt_name: Vec<Option<String>>,
    /// Visualization group (0–5) for each joint. Used by renderers for group-based filtering.
    pub jnt_group: Vec<i32>,
    /// Per-joint flag: if true, gravcomp routes through `qfrc_actuator`
    /// instead of `qfrc_passive`. Parsed from `<joint actuatorgravcomp="true"/>`.
    pub jnt_actgravcomp: Vec<bool>,

    // ==================== DOFs (indexed by dof_id) ====================
    /// Body for this DOF.
    pub dof_body: Vec<usize>,
    /// Joint for this DOF.
    pub dof_jnt: Vec<usize>,
    /// Parent DOF in kinematic tree (None for root DOF).
    pub dof_parent: Vec<Option<usize>>,
    /// Armature inertia for this DOF.
    pub dof_armature: Vec<f64>,
    /// Damping coefficient for this DOF.
    pub dof_damping: Vec<f64>,
    /// Friction loss (dry friction) for this DOF.
    /// Applied via solver constraint rows with Huber-type cost (§29).
    pub dof_frictionloss: Vec<f64>,
    /// Per-DOF solver reference parameters for friction loss [timeconst, dampratio].
    /// Resolved at compile time from joint solreffriction → default chain → DEFAULT_SOLREF.
    pub dof_solref: Vec<[f64; 2]>,
    /// Per-DOF solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    /// Resolved at compile time from joint solimpfriction → default chain → DEFAULT_SOLIMP.
    pub dof_solimp: Vec<[f64; 5]>,

    // ==================== Sparse LDL CSR Metadata (immutable, computed once at build) =====
    // The sparsity pattern of the LDL factorization is determined by `dof_parent` chains
    // and is invariant across simulation steps. Column indices and row layout are computed
    // once; only the values in `Data::qLD_data` change each step.
    //
    // Each row stores off-diagonal entries (ancestors) followed by the diagonal (self-index)
    // as the last element, matching MuJoCo's `mj_factorI` layout.
    /// Starting address of row k's entries in `Data::qLD_data` (length `nv`).
    pub qLD_rowadr: Vec<usize>,
    /// Total non-zeros in row k, including diagonal (length `nv`).
    /// `rownnz[i] - 1` = off-diagonal count = depth of DOF k in `dof_parent` tree.
    /// `rownnz[i] >= 1` always (at minimum the diagonal entry).
    pub qLD_rownnz: Vec<usize>,
    /// Column indices for flat CSR storage (length `qLD_nnz`).
    /// `qLD_colind[qLD_rowadr[k] + j]` = column of j-th entry in row k.
    /// Off-diagonals sorted ascending (ancestors from root toward DOF k),
    /// followed by self-index `k` as the last element (diagonal).
    pub qLD_colind: Vec<usize>,
    /// Total number of non-zeros across all rows (including diagonals).
    pub qLD_nnz: usize,

    // ==================== Geoms (indexed by geom_id) ====================
    /// Geometry type.
    pub geom_type: Vec<GeomType>,
    /// Parent body.
    pub geom_body: Vec<usize>,
    /// Position in body frame.
    pub geom_pos: Vec<Vector3<f64>>,
    /// Orientation in body frame.
    pub geom_quat: Vec<UnitQuaternion<f64>>,
    /// Type-specific size parameters [size0, size1, size2].
    pub geom_size: Vec<Vector3<f64>>,
    /// Friction coefficients [sliding, torsional, rolling].
    pub geom_friction: Vec<Vector3<f64>>,
    /// Contact dimensionality (1=frictionless, 3=sliding, 4=+torsional, 6=+rolling).
    pub geom_condim: Vec<i32>,
    /// Contact type bitmask.
    pub geom_contype: Vec<u32>,
    /// Contact affinity bitmask.
    pub geom_conaffinity: Vec<u32>,
    /// Contact margin (distance at which contact becomes active).
    pub geom_margin: Vec<f64>,
    /// Contact gap (minimum allowed separation).
    pub geom_gap: Vec<f64>,
    /// Contact priority. When priorities differ, higher-priority geom's params win.
    pub geom_priority: Vec<i32>,
    /// Solver mixing weight for contact parameter combination (default 1.0).
    pub geom_solmix: Vec<f64>,
    /// Solver impedance parameters [d0, dwidth, width, midpoint, power].
    /// Controls constraint softness and behavior.
    pub geom_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [d, dmin].
    /// Controls constraint dynamics.
    pub geom_solref: Vec<[f64; 2]>,
    /// Fluid interaction data per geom (12 elements, matching MuJoCo's `geom_fluid`).
    /// Layout: `[interaction_coef, C_blunt, C_slender, C_ang, C_K, C_M,
    ///           virtual_mass[3], virtual_inertia[3]]`.
    pub geom_fluid: Vec<[f64; 12]>,
    /// Optional geom names.
    pub geom_name: Vec<Option<String>>,
    /// Pre-computed bounding sphere radius for each geom (in local frame).
    /// Used for fast distance culling in collision broad-phase.
    /// For primitives, computed from geom_size. For meshes, computed from mesh AABB.
    pub geom_rbound: Vec<f64>,
    /// Mesh index for each geom (`None` if not a mesh geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Mesh` have `Some(mesh_id)`.
    pub geom_mesh: Vec<Option<usize>>,
    /// Hfield index for each geom (`None` if not an hfield geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Hfield` have `Some(hfield_id)`.
    pub geom_hfield: Vec<Option<usize>>,
    /// SDF index for each geom (`None` if not an SDF geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Sdf` have `Some(sdf_id)`.
    pub geom_sdf: Vec<Option<usize>>,
    /// Visualization group (0–5) for each geom. Used by renderers for group-based filtering.
    pub geom_group: Vec<i32>,
    /// RGBA color per geom [r, g, b, a]. Default: [0.5, 0.5, 0.5, 1.0].
    pub geom_rgba: Vec<[f64; 4]>,

    // ==================== Flex Bodies ====================
    // --- Per-flex arrays (length nflex) ---
    /// Dimensionality: 1 (cable), 2 (shell), 3 (solid).
    pub flex_dim: Vec<usize>,
    /// First vertex index in flexvert_* arrays.
    pub flex_vertadr: Vec<usize>,
    /// Number of vertices in this flex.
    pub flex_vertnum: Vec<usize>,
    /// First edge index in flexedge_* arrays.
    pub flex_edgeadr: Vec<usize>,
    /// Number of edges in this flex.
    pub flex_edgenum: Vec<usize>,
    /// First element index in flexelem_* arrays.
    pub flex_elemadr: Vec<usize>,
    /// Number of elements in this flex.
    pub flex_elemnum: Vec<usize>,
    /// Per-flex material: Young's modulus (Pa).
    pub flex_young: Vec<f64>,
    /// Per-flex material: Poisson's ratio (0–0.5).
    pub flex_poisson: Vec<f64>,
    /// Per-flex material: damping coefficient.
    pub flex_damping: Vec<f64>,
    /// Per-flex material: thickness (for dim=2 shells).
    pub flex_thickness: Vec<f64>,
    /// Per-flex: contact friction coefficients (tangential, torsional, rolling).
    pub flex_friction: Vec<Vector3<f64>>,
    /// Per-flex: contact solver reference [timeconst, dampratio].
    pub flex_solref: Vec<[f64; 2]>,
    /// Per-flex: contact solver impedance.
    pub flex_solimp: Vec<[f64; 5]>,
    /// Per-flex: contact condim (1, 3, 4, or 6).
    pub flex_condim: Vec<i32>,
    /// Per-flex: contact collision margin for broadphase expansion.
    pub flex_margin: Vec<f64>,
    /// Per-flex: contact gap (buffer zone within margin).
    pub flex_gap: Vec<f64>,
    /// Per-flex: contact priority (default 0).
    pub flex_priority: Vec<i32>,
    /// Per-flex: solver mixing weight (default 1.0).
    pub flex_solmix: Vec<f64>,
    /// Per-flex: collision type bitmask (default 1). Used for flex-rigid and
    /// flex-flex bitmask filtering per MuJoCo `filterBitmask()` protocol.
    pub flex_contype: Vec<u32>,
    /// Per-flex: collision affinity bitmask (default 1). See `flex_contype`.
    pub flex_conaffinity: Vec<u32>,
    /// Per-flex: self-collision enabled (`selfcollide != "none"`).
    /// MuJoCo gates self-collision behind THREE conjunctive conditions:
    ///   1. `!flex_rigid[f]` — rigid flexes skip entirely
    ///   2. `(flex_contype[f] & flex_conaffinity[f]) != 0` — self-bitmask check
    ///   3. `flex_selfcollide[f]` — dedicated flag
    ///
    /// Both `internal` (adjacent elements) and `selfcollide` (non-adjacent)
    /// are independently gated behind conditions 1+2.
    pub flex_selfcollide: Vec<bool>,
    /// Per-flex: passive edge spring stiffness (from `<edge stiffness="..."/>`).
    /// Used in passive force path (spring-damper), not constraint solver.
    pub flex_edgestiffness: Vec<f64>,
    /// Per-flex: passive edge damping coefficient (from `<edge damping="..."/>`).
    /// Used in passive force path (spring-damper), not constraint solver.
    pub flex_edgedamping: Vec<f64>,
    /// Per-flex: edge constraint solref. Uses flex-level solref (from
    /// `<contact solref="..."/>`). MuJoCo derives this from eq_solref on the
    /// parent equality constraint; our architecture uses flex.solref as the
    /// default, which matches MuJoCo when no explicit equality override exists.
    pub flex_edge_solref: Vec<[f64; 2]>,
    /// Per-flex: edge constraint solimp.
    pub flex_edge_solimp: Vec<[f64; 5]>,
    /// Per-flex: bending stiffness (passive spring, N·m/rad).
    /// Derived from Young's modulus, thickness, and Poisson ratio.
    /// dim=2 (shell): D = E·t³ / (12·(1−ν²))  (Kirchhoff-Love plate theory).
    /// dim=3 (solid): D = E  (characteristic stiffness; gradient provides geometry).
    pub flex_bend_stiffness: Vec<f64>,
    /// Per-flex: bending damping (passive damper, N·m·s/rad).
    /// Proportional damping: b = damping × k_bend.
    pub flex_bend_damping: Vec<f64>,
    /// Per-flex: density. Units depend on dim:
    /// dim=1 (cable): linear density kg/m.
    /// dim=2 (shell): volumetric density kg/m³ (multiplied by thickness for area density).
    /// dim=3 (solid): volumetric density kg/m³.
    pub flex_density: Vec<f64>,
    /// Visualization group (0–5) for each flex. Used by renderers for group-based filtering.
    pub flex_group: Vec<i32>,

    // --- Per-vertex arrays (length nflexvert) ---
    /// Start index in qpos for this vertex (3 consecutive DOFs).
    pub flexvert_qposadr: Vec<usize>,
    /// Start index in qvel for this vertex (3 consecutive DOFs).
    pub flexvert_dofadr: Vec<usize>,
    /// Vertex mass (kg).
    pub flexvert_mass: Vec<f64>,
    /// Inverse mass (0 for pinned vertices).
    pub flexvert_invmass: Vec<f64>,
    /// Collision radius per vertex.
    pub flexvert_radius: Vec<f64>,
    /// Which flex object this vertex belongs to (for material lookup).
    pub flexvert_flexid: Vec<usize>,
    /// Body ID for this vertex. Each flex vertex has a dedicated body created
    /// by `process_flex_bodies()`. The body's FK gives the vertex world position.
    /// Used by `mj_flex()` to populate `flexvert_xpos`.
    pub flexvert_bodyid: Vec<usize>,

    // --- Per-edge arrays (length nflexedge) ---
    /// Edge connectivity: vertex index pair [v0, v1].
    pub flexedge_vert: Vec<[usize; 2]>,
    /// Rest length of each edge.
    pub flexedge_length0: Vec<f64>,
    /// Effective cross-section area per edge (m²). Precomputed during build():
    /// dim=1: π * radius², dim=2: thickness * dual_edge_len, dim=3: vol^{2/3} / L.
    pub flexedge_crosssection: Vec<f64>,
    /// Which flex object this edge belongs to.
    pub flexedge_flexid: Vec<usize>,

    // --- Per-element arrays (length nflexelem) ---
    /// Element connectivity: vertex indices. Length varies by dim:
    /// dim=1: 2 (edge), dim=2: 3 (triangle), dim=3: 4 (tetrahedron).
    /// Stored as flat `Vec<usize>` with flexelem_dataadr/datanum for indexing.
    pub flexelem_data: Vec<usize>,
    /// Start index in flexelem_data for this element.
    pub flexelem_dataadr: Vec<usize>,
    /// Number of vertex indices per element (2, 3, or 4).
    pub flexelem_datanum: Vec<usize>,
    /// Rest volume of each element (dim=3 only, topology data for future SVK).
    pub flexelem_volume0: Vec<f64>,
    /// Which flex object this element belongs to.
    pub flexelem_flexid: Vec<usize>,

    // --- Per-hinge arrays (bending topology, length nflexhinge) ---
    // A hinge is a pair of adjacent elements sharing an edge.
    // dim=2: two triangles sharing an edge → 4 distinct vertices.
    // dim=3: two tetrahedra sharing a face → 5 distinct vertices (but bending
    //         force acts on the 4 vertices of the shared face + 2 opposite,
    //         simplified to the same 4-vertex dihedral formulation).
    /// Hinge vertex indices: [v_e0, v_e1, v_opp_a, v_opp_b] where (v_e0, v_e1)
    /// is the shared edge and v_opp_a, v_opp_b are opposite vertices.
    pub flexhinge_vert: Vec<[usize; 4]>,
    /// Rest dihedral angle (radians).
    pub flexhinge_angle0: Vec<f64>,
    /// Which flex object this hinge belongs to.
    pub flexhinge_flexid: Vec<usize>,

    // ==================== Meshes (indexed by mesh_id) ====================
    /// Number of mesh assets.
    pub nmesh: usize,
    /// Mesh names (for lookup by name).
    pub mesh_name: Vec<String>,
    /// Triangle mesh data with prebuilt BVH.
    /// `Arc` for cheap cloning (multiple geoms can reference the same mesh asset).
    pub mesh_data: Vec<Arc<TriangleMeshData>>,

    // ==================== Height Fields (indexed by hfield_id) ====================
    /// Number of height field assets.
    pub nhfield: usize,
    /// Height field names (for lookup by name).
    pub hfield_name: Vec<String>,
    /// Height field terrain data.
    pub hfield_data: Vec<Arc<HeightFieldData>>,
    /// Original MuJoCo size `[x, y, z_top, z_bottom]` for centering offset at collision time.
    pub hfield_size: Vec<[f64; 4]>,

    // ==================== SDFs (indexed by sdf_id) ====================
    /// Number of SDF assets.
    pub nsdf: usize,
    /// SDF collision data.
    /// `Arc` for cheap cloning (multiple geoms can reference the same SDF asset).
    pub sdf_data: Vec<Arc<SdfCollisionData>>,

    // ==================== Sites (indexed by site_id) ====================
    /// Parent body for each site.
    pub site_body: Vec<usize>,
    /// Site geometry type (for visualization, uses GeomType).
    pub site_type: Vec<GeomType>,
    /// Site position in body frame.
    pub site_pos: Vec<Vector3<f64>>,
    /// Site orientation in body frame.
    pub site_quat: Vec<UnitQuaternion<f64>>,
    /// Site size (for visualization).
    pub site_size: Vec<Vector3<f64>>,
    /// Optional site names.
    pub site_name: Vec<Option<String>>,
    /// Visualization group (0–5) for each site. Used by renderers for group-based filtering.
    pub site_group: Vec<i32>,
    /// RGBA color per site [r, g, b, a]. Default: [0.5, 0.5, 0.5, 1.0].
    pub site_rgba: Vec<[f64; 4]>,

    // ==================== Sensors (indexed by sensor_id) ====================
    /// Number of sensors.
    pub nsensor: usize,
    /// Number of sensor data elements (sum of all sensor dims).
    pub nsensordata: usize,
    /// Sensor type.
    pub sensor_type: Vec<MjSensorType>,
    /// Sensor data type (position/velocity/acceleration dependent).
    pub sensor_datatype: Vec<MjSensorDataType>,
    /// Object type the sensor is attached to.
    pub sensor_objtype: Vec<MjObjectType>,
    /// Object ID the sensor is attached to (body/joint/site/geom id).
    pub sensor_objid: Vec<usize>,
    /// Reference object type (for relative sensors).
    pub sensor_reftype: Vec<MjObjectType>,
    /// Reference object ID.
    pub sensor_refid: Vec<usize>,
    /// Start address in sensordata array.
    pub sensor_adr: Vec<usize>,
    /// Number of data elements for this sensor.
    pub sensor_dim: Vec<usize>,
    /// Noise standard deviation (0 = no noise).
    pub sensor_noise: Vec<f64>,
    /// Cutoff for sensor value (0 = no cutoff).
    pub sensor_cutoff: Vec<f64>,
    /// Optional sensor names.
    pub sensor_name: Vec<Option<String>>,

    // ==================== Actuators (indexed by actuator_id) ====================
    /// Transmission type (Joint, Tendon, Site).
    pub actuator_trntype: Vec<ActuatorTransmission>,
    /// Dynamics type (None, Filter, Integrator, Muscle).
    pub actuator_dyntype: Vec<ActuatorDynamics>,
    /// Transmission target ID (joint/tendon/site). Second slot is refsite for
    /// site transmissions, `usize::MAX` when unused.
    pub actuator_trnid: Vec<[usize; 2]>,
    /// Transmission gear ratio (6D: [tx ty tz rx ry rz]).
    pub actuator_gear: Vec<[f64; 6]>,
    /// Control input limits [min, max].
    pub actuator_ctrlrange: Vec<(f64, f64)>,
    /// Force output limits [min, max].
    pub actuator_forcerange: Vec<(f64, f64)>,
    /// Optional actuator names.
    pub actuator_name: Vec<Option<String>>,
    /// Start index in act array for each actuator's activation states.
    pub actuator_act_adr: Vec<usize>,
    /// Number of activation states per actuator (0 for None dynamics, 1 for Filter/Integrator/Muscle).
    pub actuator_act_num: Vec<usize>,
    /// Gain type per actuator — dispatches force gain computation.
    pub actuator_gaintype: Vec<GainType>,
    /// Bias type per actuator — dispatches force bias computation.
    pub actuator_biastype: Vec<BiasType>,

    /// Dynamics parameters per actuator (10 elements each, MuJoCo parity).
    /// For Muscle: [tau_act, tau_deact, tausmooth, 0, 0, 0, 0, 0, 0, 0].
    /// Default: [0.01, 0.04, 0.0, ...]. For Filter: [tau, 0, ...].
    /// Elements 3–9 are reserved (zero-initialized).
    pub actuator_dynprm: Vec<[f64; 10]>,

    /// Gain parameters per actuator (9 elements each).
    /// For Muscle: [range0, range1, force, scale, lmin, lmax, vmax, fpmax, fvmax].
    /// Default: [0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2].
    /// For other types: [gain, 0, ...] (gain multiplier, index 0 only).
    pub actuator_gainprm: Vec<[f64; 9]>,

    /// Bias parameters per actuator (9 elements each).
    /// For Muscle: same layout as gainprm (shared parameter set in MuJoCo).
    /// For other types: [bias0, bias1, bias2, 0, ...] (constant + length + velocity).
    pub actuator_biasprm: Vec<[f64; 9]>,

    /// Actuator length range [min, max] — the transmission length extremes.
    /// For tendon-transmission muscles: computed from tendon length at joint limits.
    /// Used to normalize muscle length: L = range0 + (len - lengthrange0) / L0.
    pub actuator_lengthrange: Vec<(f64, f64)>,

    /// Acceleration produced by unit actuator force (scalar, per actuator).
    /// Used for auto-computing F0 when `force < 0`: F0 = scale / acc0.
    /// Computed at model build time from M^{-1} and the transmission Jacobian.
    pub actuator_acc0: Vec<f64>,

    /// Whether activation clamping is enabled for each actuator.
    /// When true, activation is clamped to `actuator_actrange` after integration.
    /// MuJoCo reference: `m->actuator_actlimited[i]`.
    pub actuator_actlimited: Vec<bool>,
    /// Activation clamping range (min, max) per actuator.
    /// Only enforced when `actuator_actlimited[i]` is true.
    /// MuJoCo reference: `m->actuator_actrange[2*i]`, `m->actuator_actrange[2*i+1]`.
    pub actuator_actrange: Vec<(f64, f64)>,
    /// Whether to use predicted next-step activation for force computation.
    /// When true, force at time t uses act(t+h) instead of act(t), removing
    /// the one-timestep delay between control input and force response.
    /// MuJoCo reference: `m->actuator_actearly[i]`.
    pub actuator_actearly: Vec<bool>,

    /// Crank rod length for slider-crank transmissions.
    /// Only meaningful for `ActuatorTransmission::SliderCrank`.
    /// MuJoCo reference: `m->actuator_cranklength[i]`.
    pub actuator_cranklength: Vec<f64>,

    /// History sample count per actuator (length `nu`).
    /// MuJoCo: `actuator_history[2*i + 0]`.  Default: 0 (no history).
    /// Signed to match MuJoCo — accepts negative values (treated as no history).
    pub actuator_nsample: Vec<i32>,

    /// Interpolation type per actuator (length `nu`).
    /// MuJoCo: `actuator_history[2*i + 1]` as int 0/1/2.  Default: Zoh.
    pub actuator_interp: Vec<InterpolationType>,

    /// Cumulative offset into `Data.history` per actuator (length `nu`).
    /// MuJoCo: `actuator_historyadr`.  Equals -1 when `nsample <= 0`.
    pub actuator_historyadr: Vec<i32>,

    /// Time delay per actuator in seconds (length `nu`).
    /// MuJoCo: `actuator_delay`.  Default: 0.0.  Present for all actuators.
    pub actuator_delay: Vec<f64>,

    /// Total history buffer size (actuator contributions only).
    /// MuJoCo: `nhistory` (includes sensors — ours is actuator-only until
    /// sensor history is implemented).
    pub nhistory: usize,

    // ==================== Tendons (indexed by tendon_id) ====================
    /// Number of tendons.
    pub ntendon: usize,
    /// Number of wrap objects across all tendons.
    pub nwrap: usize,
    /// Tendon path length limits [min, max]. Limited if min < max.
    pub tendon_range: Vec<(f64, f64)>,
    /// Whether tendon has length limits.
    pub tendon_limited: Vec<bool>,
    /// Tendon stiffness (force per unit length).
    pub tendon_stiffness: Vec<f64>,
    /// Tendon damping coefficient.
    pub tendon_damping: Vec<f64>,
    /// Tendon spring rest length pair [low, high] for deadband spring.
    /// When low == high, acts as a classical spring with rest length = low.
    /// When low < high, force is zero within [low, high] (deadband).
    pub tendon_lengthspring: Vec<[f64; 2]>,
    /// Tendon length at qpos0 (precomputed reference).
    pub tendon_length0: Vec<f64>,
    /// Number of wrapping objects for this tendon.
    pub tendon_num: Vec<usize>,
    /// Start address in wrap_* arrays for this tendon's path.
    pub tendon_adr: Vec<usize>,
    /// Optional tendon names.
    pub tendon_name: Vec<Option<String>>,
    /// Tendon type: Fixed (linear coupling) or Spatial (3D path routing).
    pub tendon_type: Vec<TendonType>,
    /// Solver parameters for tendon limit constraints (2 elements per tendon).
    /// \[0\] = timeconst (>0) or -stiffness (≤0), \[1\] = dampratio or -damping.
    /// Default: \[0.02, 1.0\] (standard MuJoCo solref).
    pub tendon_solref: Vec<[f64; 2]>,
    /// Impedance parameters for tendon limit constraints (5 elements per tendon).
    /// [d_min, d_max, width, midpoint, power]. Default: [0.9, 0.95, 0.001, 0.5, 2.0].
    pub tendon_solimp: Vec<[f64; 5]>,
    /// Velocity-dependent friction loss per tendon (N).
    /// When > 0, adds a friction force opposing tendon velocity: F = -frictionloss * sign(v).
    pub tendon_frictionloss: Vec<f64>,
    /// Per-tendon solver reference parameters for friction loss [timeconst, dampratio].
    pub tendon_solref_fri: Vec<[f64; 2]>,
    /// Per-tendon solver impedance parameters for friction loss [d0, d_width, width, midpoint, power].
    pub tendon_solimp_fri: Vec<[f64; 5]>,
    /// Visualization group (0–5) for each tendon. Used by renderers for group-based filtering.
    pub tendon_group: Vec<i32>,
    /// RGBA color per tendon [r, g, b, a]. Default: [0.5, 0.5, 0.5, 1.0].
    pub tendon_rgba: Vec<[f64; 4]>,
    /// Number of distinct kinematic trees spanned by each tendon (§16.10.1).
    /// 0 = no bodies, 1 = single tree, 2 = two trees. Length: ntendon.
    pub tendon_treenum: Vec<usize>,
    /// Packed tree indices for two-tree tendons (§16.10.1).
    /// For tendon t: `tendon_tree[2*t]` and `tendon_tree[2*t+1]`.
    /// `tendon_tree[2*t]` is populated when `treenum >= 1`; `tendon_tree[2*t+1]`
    /// is populated when `treenum == 2`. Unused slots are `usize::MAX`.
    /// Length: 2 * ntendon.
    pub tendon_tree: Vec<usize>,

    // Tendon wrapping path elements (indexed by wrap_id, grouped by tendon; total length = nwrap)
    /// Wrap object type (Site, Geom, Joint, Pulley).
    pub wrap_type: Vec<WrapType>,
    /// Object ID for the wrap point (site/geom/joint id).
    pub wrap_objid: Vec<usize>,
    /// Wrap parameter (coefficient for Joint, divisor for Pulley, 0.0 for Site/Geom).
    pub wrap_prm: Vec<f64>,
    /// Sidesite ID for wrapping geom wrap objects. `usize::MAX` if no sidesite.
    /// Indexed in parallel with `wrap_type`/`wrap_objid`/`wrap_prm`.
    pub wrap_sidesite: Vec<usize>,

    // ==================== Equality Constraints (indexed by eq_id) ====================
    /// Number of equality constraints.
    pub neq: usize,
    /// Equality constraint type (Connect, Weld, Joint, Tendon, Distance).
    pub eq_type: Vec<EqualityType>,
    /// First object ID (body for Connect/Weld, joint for Joint, geom for Distance).
    pub eq_obj1id: Vec<usize>,
    /// Second object ID (body/joint/geom, or `usize::MAX` for world origin in Distance).
    pub eq_obj2id: Vec<usize>,
    /// Constraint parameters (meaning depends on type).
    /// - Connect: anchor point in body1 frame [x, y, z]
    /// - Weld: relative pose [x, y, z, qw, qx, qy, qz] + torque scale
    /// - Joint: polycoef[0..5] for polynomial coupling
    /// - Tendon: polycoef[0..5] for polynomial coupling
    /// - Distance: target distance in `[0]`; `[1..10]` unused
    pub eq_data: Vec<[f64; 11]>,
    /// Whether this equality constraint is active.
    pub eq_active: Vec<bool>,
    /// Solver impedance parameters for this constraint.
    pub eq_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters for this constraint.
    pub eq_solref: Vec<[f64; 2]>,
    /// Optional equality constraint names.
    pub eq_name: Vec<Option<String>>,

    // ==================== Options ====================
    /// Simulation timestep in seconds.
    pub timestep: f64,
    /// Gravity vector in world frame.
    pub gravity: Vector3<f64>,
    /// Default/reference joint positions.
    pub qpos0: DVector<f64>,
    /// Named state snapshots for quick reset.
    pub keyframes: Vec<Keyframe>,
    /// Wind velocity in world frame (for aerodynamic forces).
    pub wind: Vector3<f64>,
    /// Magnetic field in world frame (for magnetic actuators).
    pub magnetic: Vector3<f64>,
    /// Medium density (for fluid drag, kg/m³).
    pub density: f64,
    /// Medium viscosity (for fluid drag, Pa·s).
    pub viscosity: f64,

    // Solver options
    /// Maximum constraint solver iterations.
    pub solver_iterations: usize,
    /// Early termination tolerance for solver.
    pub solver_tolerance: f64,
    /// Constraint impedance ratio (for soft constraints).
    pub impratio: f64,
    /// Base regularization (softness) for PGS constraint matrix diagonal (default: 1e-6).
    /// CFM scales as `regularization + (1 - impedance) * regularization * 100`.
    pub regularization: f64,
    /// DEPRECATED: Friction smoothing factor — previously used for tanh transition.
    /// Friction loss is now handled by solver constraint rows (§29).
    /// Kept for serialization compatibility; not used in simulation.
    pub friction_smoothing: f64,
    /// Friction cone type: 0=pyramidal, 1=elliptic.
    pub cone: u8,

    // Newton solver parameters (§15)
    /// Mean inertia: trace(M) / nv at qpos0. Used by Newton solver for scaling.
    /// Computed at model build time by running CRBA at qpos0.
    pub stat_meaninertia: f64,
    /// Maximum line search iterations for Newton solver (default 50).
    pub ls_iterations: usize,
    /// Line search gradient tolerance for Newton solver (default 0.01).
    pub ls_tolerance: f64,
    /// Noslip solver iterations (parsed from MJCF, stored, not yet implemented; default 0).
    pub noslip_iterations: usize,
    /// Noslip solver tolerance (parsed from MJCF, stored, not yet implemented; default 1e-6).
    pub noslip_tolerance: f64,

    /// Disable flags (bitmask for disabling default behaviors).
    pub disableflags: u32,
    /// Enable flags (bitmask for enabling optional behaviors).
    pub enableflags: u32,
    /// Per-group actuator disable bitmask. Bit `i` set = group `i` disabled.
    /// Parsed from `<option actuatorgroupdisable="2 5"/>` (space-separated
    /// list of group IDs, each 0–30) or set at runtime.
    pub disableactuator: u32,
    /// Group assignment per actuator (0–30). Default 0.
    /// Parsed from `<actuator><general group="..."/>`.
    pub actuator_group: Vec<i32>,

    // ==================== Contact Override (§41 S10) ====================
    /// Global contact margin override. Used when `ENABLE_OVERRIDE` is set.
    /// Default: 0.0. Parsed from `<option o_margin="..."/>`.
    pub o_margin: f64,
    /// Global contact solver reference override `[timeconst, dampratio]`.
    /// Default: `[0.02, 1.0]`. Parsed from `<option o_solref="..."/>`.
    pub o_solref: [f64; 2],
    /// Global contact solver impedance override `[dmin, dmax, width, midpoint, power]`.
    /// Default: `[0.9, 0.95, 0.001, 0.5, 2.0]`. Parsed from `<option o_solimp="..."/>`.
    pub o_solimp: [f64; 5],
    /// Global contact friction override `[tan1, tan2, torsional, rolling1, rolling2]`.
    /// Default: `[1.0, 1.0, 0.005, 0.0001, 0.0001]`. Parsed from `<option o_friction="..."/>`.
    pub o_friction: [f64; 5],

    /// Integration method.
    pub integrator: Integrator,
    /// Contact constraint solver algorithm (PGS or CG).
    pub solver_type: SolverType,

    // ==================== Cached Implicit Integration Parameters ====================
    // These are pre-computed from joint properties for implicit spring-damper integration.
    // Avoids O(nv) allocation per step by caching model-invariant diagonal matrices.
    /// Diagonal stiffness matrix K for implicit integration (length nv).
    /// `K\[i\]` = jnt_stiffness for Hinge/Slide DOFs, 0 for Ball/Free DOFs.
    pub implicit_stiffness: DVector<f64>,
    /// Diagonal damping matrix D for implicit integration (length nv).
    /// `D\[i\]` = jnt_damping for Hinge/Slide, dof_damping for Ball/Free DOFs.
    pub implicit_damping: DVector<f64>,
    /// Spring equilibrium positions for implicit integration (length nv).
    /// `q_eq\[i\]` = jnt_springref for Hinge/Slide, 0 for Ball/Free DOFs.
    pub implicit_springref: DVector<f64>,

    // ==================== Pre-computed Kinematic Data ====================
    // These are computed once at model construction to avoid O(n) lookups
    // in the inner loops of CRBA and RNE, achieving O(n) vs O(n³) complexity.
    /// For each body: list of ancestor joint indices (from body to root).
    /// `body_ancestor_joints[i]` contains all joints in the kinematic chain
    /// from body `i` to the world. Empty for body 0 (world).
    pub body_ancestor_joints: Vec<Vec<usize>>,

    /// For each body: set of ancestor joint indices for O(1) membership testing.
    /// Multi-word bitmask: `body_ancestor_mask[body_id][word]` where word = jnt_id / 64.
    /// Bit (jnt_id % 64) is set if joint jnt_id is an ancestor of this body.
    /// Supports unlimited joints with O(1) lookup per word.
    pub body_ancestor_mask: Vec<Vec<u64>>,

    // ==================== Name↔Index Lookup (§59) ====================
    /// Body name → index. Populated from `body_name` entries.
    pub body_name_to_id: HashMap<String, usize>,
    /// Joint name → index. Populated from `jnt_name` entries.
    pub jnt_name_to_id: HashMap<String, usize>,
    /// Geom name → index. Populated from `geom_name` entries.
    pub geom_name_to_id: HashMap<String, usize>,
    /// Site name → index. Populated from `site_name` entries.
    pub site_name_to_id: HashMap<String, usize>,
    /// Tendon name → index. Populated from `tendon_name` entries.
    pub tendon_name_to_id: HashMap<String, usize>,
    /// Actuator name → index. Populated from `actuator_name` entries.
    pub actuator_name_to_id: HashMap<String, usize>,
    /// Sensor name → index. Populated from `sensor_name` entries.
    pub sensor_name_to_id: HashMap<String, usize>,
    /// Mesh name → index. Populated from `mesh_name` entries.
    pub mesh_name_to_id: HashMap<String, usize>,
    /// Height field name → index. Populated from `hfield_name` entries.
    pub hfield_name_to_id: HashMap<String, usize>,
    /// Equality constraint name → index. Populated from `eq_name` entries.
    pub eq_name_to_id: HashMap<String, usize>,

    // ==================== Explicit Contact Pairs/Excludes ====================
    /// Explicit contact pairs from `<contact><pair>`.
    /// Processed in mechanism 2 (bypass kinematic and bitmask filters).
    pub contact_pairs: Vec<ContactPair>,
    /// Explicit pair geom-pair set for O(1) lookup during automatic pipeline.
    /// Canonical key: `(min(geom1, geom2), max(geom1, geom2))`.
    /// Used to suppress automatic-pipeline contacts for pairs that have explicit overrides.
    pub contact_pair_set: HashSet<(usize, usize)>,
    /// Excluded body-pair set from `<contact><exclude>`.
    /// Canonical key: `(min(body1, body2), max(body1, body2))`.
    pub contact_excludes: HashSet<(usize, usize)>,

    // ==================== User Callbacks (DT-79) ====================
    /// Passive force callback: called at end of `mj_fwd_passive()`.
    pub cb_passive: Option<super::callbacks::CbPassive>,
    /// Control callback: called between velocity and acceleration stages (§53).
    pub cb_control: Option<super::callbacks::CbControl>,
    /// Contact filter callback: called after affinity check in collision.
    pub cb_contactfilter: Option<super::callbacks::CbContactFilter>,
    /// User sensor callback: called for `MjSensorType::User` sensors.
    pub cb_sensor: Option<super::callbacks::CbSensor>,
    /// User actuator dynamics callback: called for `ActuatorDynamics::User`.
    pub cb_act_dyn: Option<super::callbacks::CbActDyn>,
    /// User actuator gain callback: called for `GainType::User`.
    pub cb_act_gain: Option<super::callbacks::CbActGain>,
    /// User actuator bias callback: called for `BiasType::User`.
    pub cb_act_bias: Option<super::callbacks::CbActBias>,
}

// ============================================================================
// Length-range estimation types (Phase 5, Spec A §S4)
// ============================================================================

/// Options for simulation-based actuator length-range estimation.
/// Matches MuJoCo's `mjLROpt` struct with identical defaults.
#[derive(Debug, Clone)]
pub struct LengthRangeOpt {
    /// Which actuators to compute for.
    pub mode: LengthRangeMode,
    /// Skip if range already set (lo < hi).
    pub useexisting: bool,
    /// Copy from joint/tendon limits when available.
    pub uselimit: bool,
    /// Target acceleration magnitude for the applied force.
    pub accel: f64,
    /// Force cap (0 = unlimited).
    pub maxforce: f64,
    /// Velocity damping time constant (seconds).
    pub timeconst: f64,
    /// Internal simulation timestep (seconds).
    pub timestep: f64,
    /// Total simulation time (seconds).
    pub inttotal: f64,
    /// Measurement interval — last N seconds used for convergence.
    pub interval: f64,
    /// Convergence tolerance (fraction of total range).
    pub tolrange: f64,
}

impl Default for LengthRangeOpt {
    fn default() -> Self {
        Self {
            mode: LengthRangeMode::Muscle,
            useexisting: true,
            uselimit: true,
            accel: 20.0,
            maxforce: 0.0,
            timeconst: 1.0,
            timestep: 0.01,
            inttotal: 10.0,
            interval: 2.0,
            tolrange: 0.05,
        }
    }
}

/// Which actuators get simulation-based length-range estimation.
/// Matches MuJoCo's `mjLRMODE_*` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LengthRangeMode {
    /// Disabled entirely.
    None,
    /// Compute only for muscle actuators (default).
    #[default]
    Muscle,
    /// Compute for muscle and user-defined actuators.
    MuscleUser,
    /// Compute for all actuators.
    All,
}

/// Errors from simulation-based length-range estimation.
#[derive(Debug)]
pub enum LengthRangeError {
    /// Length range has zero or negative extent (max <= min).
    InvalidRange {
        /// Index of the actuator.
        actuator: usize,
    },
    /// Simulation did not converge within tolerance.
    ConvergenceFailed {
        /// Index of the actuator.
        actuator: usize,
    },
}

impl Model {
    /// Returns CSR metadata for the sparse LDL factorization: `(rowadr, rownnz, colind)`.
    #[inline]
    #[must_use]
    pub fn qld_csr(&self) -> (&[usize], &[usize], &[usize]) {
        (&self.qLD_rowadr, &self.qLD_rownnz, &self.qLD_colind)
    }

    /// Get reference position for specified joint (from qpos0).
    ///
    /// Returns `None` if `jnt_id` is out of bounds.
    #[must_use]
    pub fn joint_qpos0(&self, jnt_id: usize) -> Option<&[f64]> {
        if jnt_id >= self.njnt {
            return None;
        }
        let start = self.jnt_qpos_adr[jnt_id];
        let len = self.jnt_type[jnt_id].nq();
        Some(&self.qpos0.as_slice()[start..start + len])
    }

    /// Look up element index by name (O(1) via HashMap).
    ///
    /// Returns `None` if the name does not exist for the given element type.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use sim_core::ElementType;
    /// let joint_id = model.name2id(ElementType::Joint, "shoulder").unwrap();
    /// ```
    #[must_use]
    pub fn name2id(&self, element: super::enums::ElementType, name: &str) -> Option<usize> {
        use super::enums::ElementType;
        match element {
            ElementType::Body => self.body_name_to_id.get(name).copied(),
            ElementType::Joint => self.jnt_name_to_id.get(name).copied(),
            ElementType::Geom => self.geom_name_to_id.get(name).copied(),
            ElementType::Site => self.site_name_to_id.get(name).copied(),
            ElementType::Tendon => self.tendon_name_to_id.get(name).copied(),
            ElementType::Actuator => self.actuator_name_to_id.get(name).copied(),
            ElementType::Sensor => self.sensor_name_to_id.get(name).copied(),
            ElementType::Mesh => self.mesh_name_to_id.get(name).copied(),
            ElementType::Hfield => self.hfield_name_to_id.get(name).copied(),
            ElementType::Equality => self.eq_name_to_id.get(name).copied(),
        }
    }

    /// Look up element name by index (O(1) via Vec indexing).
    ///
    /// Returns `None` if the index is out of bounds or the element has no name.
    #[must_use]
    pub fn id2name(&self, element: super::enums::ElementType, id: usize) -> Option<&str> {
        use super::enums::ElementType;
        match element {
            ElementType::Body => self.body_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Joint => self.jnt_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Geom => self.geom_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Site => self.site_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Tendon => self.tendon_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Actuator => self.actuator_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Sensor => self.sensor_name.get(id).and_then(|n| n.as_deref()),
            ElementType::Mesh => self.mesh_name.get(id).map(String::as_str),
            ElementType::Hfield => self.hfield_name.get(id).map(String::as_str),
            ElementType::Equality => self.eq_name.get(id).and_then(|n| n.as_deref()),
        }
    }

    // ==================== Callback Setters (DT-79) ====================

    /// Set the passive force callback.
    pub fn set_passive_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &mut super::data::Data) + Send + Sync + 'static,
    {
        self.cb_passive = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the passive force callback.
    pub fn clear_passive_callback(&mut self) {
        self.cb_passive = None;
    }

    /// Set the control callback.
    pub fn set_control_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &mut super::data::Data) + Send + Sync + 'static,
    {
        self.cb_control = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the control callback.
    pub fn clear_control_callback(&mut self) {
        self.cb_control = None;
    }

    /// Set the contact filter callback.
    ///
    /// The callback receives `(model, data, geom1_id, geom2_id)` and should
    /// return `true` to KEEP the contact, `false` to REJECT it.
    pub fn set_contactfilter_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &super::data::Data, usize, usize) -> bool + Send + Sync + 'static,
    {
        self.cb_contactfilter = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the contact filter callback.
    pub fn clear_contactfilter_callback(&mut self) {
        self.cb_contactfilter = None;
    }

    /// Set the user sensor callback.
    ///
    /// Called for each `MjSensorType::User` sensor at the appropriate stage.
    pub fn set_sensor_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &mut super::data::Data, usize, super::enums::SensorStage)
            + Send
            + Sync
            + 'static,
    {
        self.cb_sensor = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the sensor callback.
    pub fn clear_sensor_callback(&mut self) {
        self.cb_sensor = None;
    }

    /// Set the user actuator dynamics callback.
    ///
    /// Called for each actuator with `ActuatorDynamics::User`.
    /// Should return `act_dot` (activation derivative).
    pub fn set_act_dyn_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &super::data::Data, usize) -> f64 + Send + Sync + 'static,
    {
        self.cb_act_dyn = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the actuator dynamics callback.
    pub fn clear_act_dyn_callback(&mut self) {
        self.cb_act_dyn = None;
    }

    /// Set the user actuator gain callback.
    ///
    /// Called for each actuator with `GainType::User`.
    /// Should return the gain value.
    pub fn set_act_gain_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &super::data::Data, usize) -> f64 + Send + Sync + 'static,
    {
        self.cb_act_gain = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the actuator gain callback.
    pub fn clear_act_gain_callback(&mut self) {
        self.cb_act_gain = None;
    }

    /// Set the user actuator bias callback.
    ///
    /// Called for each actuator with `BiasType::User`.
    /// Should return the bias value.
    pub fn set_act_bias_callback<F>(&mut self, f: F)
    where
        F: Fn(&Self, &super::data::Data, usize) -> f64 + Send + Sync + 'static,
    {
        self.cb_act_bias = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
    }

    /// Clear the actuator bias callback.
    pub fn clear_act_bias_callback(&mut self) {
        self.cb_act_bias = None;
    }

    /// Check if joint is an ancestor of body using pre-computed data.
    ///
    /// Uses O(1) multi-word bitmask lookup for all model sizes.
    /// Returns `false` for invalid body_id or jnt_id (no panic).
    #[inline]
    #[must_use]
    pub fn is_ancestor(&self, body_id: usize, jnt_id: usize) -> bool {
        // Bounds check for body_id
        if body_id >= self.body_ancestor_mask.len() {
            return false;
        }
        let word = jnt_id / 64;
        let bit = jnt_id % 64;
        if word < self.body_ancestor_mask[body_id].len() {
            (self.body_ancestor_mask[body_id][word] & (1u64 << bit)) != 0
        } else {
            false // Joint ID out of range
        }
    }
}
