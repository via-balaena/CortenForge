//! MJCF to Model conversion (Phase 4 of MuJoCo consolidation).
//!
//! This module converts parsed `MjcfModel` structures directly into the
//! MuJoCo-aligned `Model` struct from sim-core.
//!
//! # MuJoCo Semantics
//!
//! The conversion follows MuJoCo's exact semantics for:
//! - **dof_parent**: Forms a kinematic tree linking DOFs across joints and bodies
//! - **nq vs nv**: Position dimension can exceed velocity dimension (e.g., Ball: nq=4, nv=3)
//! - **Inertia computation**: Parallel axis theorem for composite bodies
//! - **Capsule inertia**: Exact formula including hemispherical end caps

use nalgebra::{DVector, Matrix3, Point3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::HeightFieldData;
use sim_core::mesh::TriangleMeshData;
use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, ContactPair, ENABLE_SLEEP, EqualityType,
    GainType, GeomType, Integrator, Keyframe, MjJointType, MjObjectType, MjSensorDataType,
    MjSensorType, Model, SleepPolicy, SolverType, TendonType, WrapType, compute_dof_lengths,
};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tracing::warn;

use crate::defaults::DefaultResolver;
use crate::error::Result;
use crate::types::{
    AngleUnit, InertiaFromGeom, MjcfActuator, MjcfActuatorType, MjcfBody, MjcfCompiler,
    MjcfContact, MjcfEquality, MjcfFrame, MjcfGeom, MjcfGeomType, MjcfHfield, MjcfInertial,
    MjcfIntegrator, MjcfJoint, MjcfJointType, MjcfKeyframe, MjcfMesh, MjcfModel, MjcfOption,
    MjcfSensor, MjcfSensorType, MjcfSite, MjcfSolverType, MjcfTendon, MjcfTendonType,
    SpatialPathElement,
};

/// Default solref parameters [timeconst, dampratio] (MuJoCo defaults).
///
/// - timeconst = 0.02s: 50 Hz natural frequency
/// - dampratio = 1.0: critical damping
const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];

/// Default solimp parameters `[d0, d_width, width, midpoint, power]` (MuJoCo defaults).
///
/// - d0 = 0.9: initial impedance (close to rigid)
/// - d_width = 0.95: impedance at full penetration
/// - width = 0.001: penetration depth at which impedance transitions
/// - midpoint = 0.5, power = 2.0: sigmoid transition shape
const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

/// Error type for MJCF to Model conversion.
#[derive(Debug, Clone)]
pub struct ModelConversionError {
    /// Error message.
    pub message: String,
}

impl std::fmt::Display for ModelConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MJCF to Model conversion error: {}", self.message)
    }
}

impl std::error::Error for ModelConversionError {}

// Resolve an `MjcfKeyframe` into a `Keyframe` with model-validated dimensions.
//
// Validates lengths, finiteness, and fills missing fields with model defaults.
fn resolve_keyframe(
    mjcf_kf: &MjcfKeyframe,
    model: &Model,
) -> std::result::Result<Keyframe, ModelConversionError> {
    // Validate time finiteness
    if !mjcf_kf.time.is_finite() {
        return Err(ModelConversionError {
            message: format!(
                "keyframe '{}': time is not finite ({})",
                mjcf_kf.name, mjcf_kf.time
            ),
        });
    }

    // Helper: validate length and finiteness for a float array field.
    let validate_field = |field: &Option<Vec<f64>>,
                          field_name: &str,
                          expected_len: usize|
     -> std::result::Result<(), ModelConversionError> {
        if let Some(ref v) = *field {
            if v.len() != expected_len {
                return Err(ModelConversionError {
                    message: format!(
                        "keyframe '{}': {} length {} does not match {}",
                        mjcf_kf.name,
                        field_name,
                        v.len(),
                        expected_len
                    ),
                });
            }
            if let Some(pos) = v.iter().position(|x| !x.is_finite()) {
                return Err(ModelConversionError {
                    message: format!(
                        "keyframe '{}': {}[{}] is not finite ({})",
                        mjcf_kf.name, field_name, pos, v[pos]
                    ),
                });
            }
        }
        Ok(())
    };

    validate_field(&mjcf_kf.qpos, "qpos", model.nq)?;
    validate_field(&mjcf_kf.qvel, "qvel", model.nv)?;
    validate_field(&mjcf_kf.act, "act", model.na)?;
    validate_field(&mjcf_kf.ctrl, "ctrl", model.nu)?;
    validate_field(&mjcf_kf.mpos, "mpos", 3 * model.nmocap)?;
    validate_field(&mjcf_kf.mquat, "mquat", 4 * model.nmocap)?;

    // Build resolved keyframe with model defaults for missing fields
    let qpos = match mjcf_kf.qpos {
        Some(ref v) => DVector::from_vec(v.clone()),
        None => model.qpos0.clone(),
    };
    let qvel = match mjcf_kf.qvel {
        Some(ref v) => DVector::from_vec(v.clone()),
        None => DVector::zeros(model.nv),
    };
    let act = match mjcf_kf.act {
        Some(ref v) => DVector::from_vec(v.clone()),
        None => DVector::zeros(model.na),
    };
    let ctrl = match mjcf_kf.ctrl {
        Some(ref v) => DVector::from_vec(v.clone()),
        None => DVector::zeros(model.nu),
    };

    // Mocap positions: reshape flat array into Vec<Vector3<f64>>
    let mpos = match mjcf_kf.mpos {
        Some(ref v) => v
            .chunks_exact(3)
            .map(|c| Vector3::new(c[0], c[1], c[2]))
            .collect(),
        None => (0..model.nbody)
            .filter_map(|i| model.body_mocapid[i].map(|_| model.body_pos[i]))
            .collect(),
    };

    // Mocap quaternions: reshape flat array into Vec<UnitQuaternion<f64>>
    let mquat = match mjcf_kf.mquat {
        Some(ref v) => v
            .chunks_exact(4)
            .map(|c| {
                UnitQuaternion::new_normalize(nalgebra::Quaternion::new(c[0], c[1], c[2], c[3]))
            })
            .collect(),
        None => (0..model.nbody)
            .filter_map(|i| model.body_mocapid[i].map(|_| model.body_quat[i]))
            .collect(),
    };

    Ok(Keyframe {
        name: mjcf_kf.name.clone(),
        time: mjcf_kf.time,
        qpos,
        qvel,
        act,
        ctrl,
        mpos,
        mquat,
    })
}

/// Convert a parsed [`MjcfModel`] into a physics [`Model`].
///
/// This is the primary entry point for Model-based MJCF loading. It builds
/// all Model arrays in a single tree traversal, resolves keyframes, and
/// validates mocap body constraints.
///
/// # Arguments
///
/// * `mjcf` - The parsed MJCF model
/// * `base_path` - Base directory for resolving relative mesh file paths.
///   If `None`, meshes with relative file paths will fail to load.
///   For embedded vertex data, this parameter is ignored.
///
/// # Example
///
/// ```ignore
/// use sim_mjcf::{parse_mjcf_str, model_from_mjcf};
///
/// let mjcf = parse_mjcf_str(r#"
///     <mujoco>
///         <worldbody>
///             <body name="pendulum" pos="0 0 1">
///                 <joint type="hinge" axis="0 1 0"/>
///                 <geom type="capsule" size="0.05 0.5"/>
///             </body>
///         </worldbody>
///     </mujoco>
/// "#)?;
///
/// let model = model_from_mjcf(&mjcf, None)?;
/// let mut data = model.make_data();
/// data.step(&model).expect("step failed");
/// ```
pub fn model_from_mjcf(
    mjcf: &MjcfModel,
    base_path: Option<&Path>,
) -> std::result::Result<Model, ModelConversionError> {
    // Apply compiler pre-processing passes on a mutable clone
    let mut mjcf = mjcf.clone();

    // Validate childclass references before frame expansion dissolves frames.
    // MuJoCo rejects undefined childclass at schema validation (S7).
    let pre_resolver = DefaultResolver::from_model(&mjcf);
    validate_childclass_references(&mjcf.worldbody, &pre_resolver)?;

    // Expand <frame> elements first: compose transforms into children, lift onto parent bodies.
    // This must run BEFORE discardvisual/fusestatic so that geoms/sites inside frames are
    // visible to those passes (matches MuJoCo, which expands frames during XML parsing).
    expand_frames(&mut mjcf.worldbody, &mjcf.compiler, None);

    if mjcf.compiler.discardvisual {
        apply_discardvisual(&mut mjcf);
    }
    if mjcf.compiler.fusestatic {
        apply_fusestatic(&mut mjcf);
    }

    let mut builder = ModelBuilder::new();
    builder.resolver = DefaultResolver::from_model(&mjcf);
    builder.compiler = mjcf.compiler.clone();

    // Set model name
    builder.name.clone_from(&mjcf.name);

    // Set global options
    builder.set_options(&mjcf.option);

    // Process mesh assets FIRST (before geoms can reference them)
    for mesh in &mjcf.meshes {
        builder.process_mesh(mesh, base_path)?;
    }

    // Process hfield assets (before geoms can reference them)
    for hfield in &mjcf.hfields {
        builder.process_hfield(hfield)?;
    }

    // Process worldbody's own geoms and sites (attached to body 0)
    // These must be processed BEFORE child bodies so geom indices are correct
    builder.process_worldbody_geoms_and_sites(&mjcf.worldbody)?;

    // Recursively process body tree starting from worldbody children.
    // World body (body 0) has no DOFs, so parent_last_dof is None.
    // Note: worldbody.childclass is always None for parsed MJCF — MuJoCo's schema
    // rejects attributes on <worldbody>. Our parser (parse_worldbody) never reads
    // worldbody attributes, so this is correct by construction.
    for child in &mjcf.worldbody.children {
        builder.process_body(child, 0, None, mjcf.worldbody.childclass.as_deref())?;
    }

    // Validate tendons before processing (catches bad references early)
    crate::validation::validate_tendons(&mjcf).map_err(|e| ModelConversionError {
        message: format!("Tendon validation failed: {e}"),
    })?;

    // Process tendons (must be before actuators, since actuators may reference tendons)
    builder.process_tendons(&mjcf.tendons)?;

    // Process actuators
    for actuator in &mjcf.actuators {
        let actuator = builder.resolver.apply_to_actuator(actuator);
        builder.process_actuator(&actuator)?;
    }

    // Process equality constraints (must be after bodies and joints)
    builder.process_equality_constraints(&mjcf.equality)?;

    // Process sensors (must be after bodies, joints, tendons, actuators)
    builder.process_sensors(&mjcf.sensors)?;

    // Process contact pairs and excludes (must be after body tree for name-to-id maps)
    builder.process_contact(&mjcf.contact)?;

    // Apply mass pipeline (balanceinertia, boundmass/boundinertia, settotalmass)
    builder.apply_mass_pipeline();

    // Build final model
    let mut model = builder.build();

    // Process keyframes (after build, so nq/nv/na/nu/nmocap are finalized).
    // Collect into a temporary Vec first — calling resolve_keyframe(&model)
    // borrows model immutably, which conflicts with model.keyframes.push()
    // (mutable borrow). Collecting then assigning avoids the overlap.
    model.keyframes = mjcf
        .keyframes
        .iter()
        .map(|kf| resolve_keyframe(kf, &model))
        .collect::<std::result::Result<Vec<_>, _>>()?;
    model.nkeyframe = model.keyframes.len();

    Ok(model)
}

/// Parse MJCF XML and convert directly to Model.
///
/// Convenience function that combines parsing and conversion.
/// Uses no base path, so meshes must use embedded data or absolute paths.
///
/// Models containing `<include>` elements cannot be loaded from strings —
/// use [`load_model_from_file`] instead. This function will return an error
/// if `<include` is detected in the XML.
pub fn load_model(xml: &str) -> Result<Model> {
    if xml.contains("<include") {
        return Err(crate::error::MjcfError::IncludeError(
            "<include> elements require file-based loading; use load_model_from_file() instead"
                .to_string(),
        ));
    }
    let mjcf = crate::parse_mjcf_str(xml)?;
    model_from_mjcf(&mjcf, None).map_err(|e| crate::error::MjcfError::Unsupported(e.message))
}

/// Load Model from an MJCF file path.
///
/// Automatically resolves mesh file paths relative to the model file's directory.
///
/// # Arguments
///
/// * `path` - Path to the MJCF XML file
///
/// # Example
///
/// ```ignore
/// use sim_mjcf::load_model_from_file;
///
/// let model = load_model_from_file("models/humanoid.xml")?;
/// let mut data = model.make_data();
/// data.step(&model).expect("step failed");
/// ```
pub fn load_model_from_file<P: AsRef<Path>>(path: P) -> Result<Model> {
    let path = path.as_ref();
    let xml = std::fs::read_to_string(path).map_err(|e| {
        crate::error::MjcfError::Unsupported(format!(
            "failed to read file '{}': {}",
            path.display(),
            e
        ))
    })?;

    // Expand <include> elements before parsing
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    let expanded_xml = crate::include::expand_includes(&xml, base_dir)?;

    let mjcf = crate::parse_mjcf_str(&expanded_xml)?;
    let base_path = path.parent();

    model_from_mjcf(&mjcf, base_path).map_err(|e| crate::error::MjcfError::Unsupported(e.message))
}

/// Builder for constructing Model from MJCF.
struct ModelBuilder {
    // Model name
    name: String,

    // Default class resolver (applies <default> class attributes to elements)
    resolver: DefaultResolver,

    // Compiler settings (controls angle conversion, euler sequence, etc.)
    compiler: MjcfCompiler,

    // Dimensions (computed during building)
    nq: usize,
    nv: usize,

    // Body arrays
    body_parent: Vec<usize>,
    body_rootid: Vec<usize>,
    body_jnt_adr: Vec<usize>,
    body_jnt_num: Vec<usize>,
    body_dof_adr: Vec<usize>,
    body_dof_num: Vec<usize>,
    body_geom_adr: Vec<usize>,
    body_geom_num: Vec<usize>,
    body_pos: Vec<Vector3<f64>>,
    body_quat: Vec<UnitQuaternion<f64>>,
    body_ipos: Vec<Vector3<f64>>,
    body_iquat: Vec<UnitQuaternion<f64>>,
    body_mass: Vec<f64>,
    body_inertia: Vec<Vector3<f64>>,
    body_name: Vec<Option<String>>,
    body_mocapid: Vec<Option<usize>>,
    /// Per-body sleep policy from MJCF `sleep` attribute (None = default Auto).
    body_sleep_policy: Vec<Option<SleepPolicy>>,
    nmocap: usize,

    // Joint arrays
    jnt_type: Vec<MjJointType>,
    jnt_body: Vec<usize>,
    jnt_qpos_adr: Vec<usize>,
    jnt_dof_adr: Vec<usize>,
    jnt_pos: Vec<Vector3<f64>>,
    jnt_axis: Vec<Vector3<f64>>,
    jnt_limited: Vec<bool>,
    jnt_range: Vec<(f64, f64)>,
    jnt_stiffness: Vec<f64>,
    jnt_springref: Vec<f64>,
    jnt_damping: Vec<f64>,
    jnt_armature: Vec<f64>,
    jnt_solref: Vec<[f64; 2]>,
    jnt_solimp: Vec<[f64; 5]>,
    jnt_name: Vec<Option<String>>,

    // DOF arrays
    dof_body: Vec<usize>,
    dof_jnt: Vec<usize>,
    dof_parent: Vec<Option<usize>>,
    dof_armature: Vec<f64>,
    dof_damping: Vec<f64>,
    dof_frictionloss: Vec<f64>,

    // Geom arrays
    geom_type: Vec<GeomType>,
    geom_body: Vec<usize>,
    geom_pos: Vec<Vector3<f64>>,
    geom_quat: Vec<UnitQuaternion<f64>>,
    geom_size: Vec<Vector3<f64>>,
    geom_friction: Vec<Vector3<f64>>,
    geom_condim: Vec<i32>,
    geom_contype: Vec<u32>,
    geom_conaffinity: Vec<u32>,
    geom_name: Vec<Option<String>>,
    /// Mesh index for each geom (`None` for non-mesh geoms).
    geom_mesh: Vec<Option<usize>>,
    /// Solver reference parameters for each geom [timeconst, dampratio].
    geom_solref: Vec<[f64; 2]>,
    /// Solver impedance parameters for each geom [d0, d_width, width, midpoint, power].
    geom_solimp: Vec<[f64; 5]>,

    // Mesh arrays (built from MJCF <asset><mesh> elements)
    /// Name-to-index lookup for mesh assets.
    mesh_name_to_id: HashMap<String, usize>,
    /// Mesh names (for Model).
    mesh_name: Vec<String>,
    /// Triangle mesh data with prebuilt BVH (Arc for cheap cloning).
    mesh_data: Vec<Arc<TriangleMeshData>>,

    // Height field arrays (built from MJCF <asset><hfield> elements)
    /// Name-to-index lookup for hfield assets.
    hfield_name_to_id: HashMap<String, usize>,
    /// Hfield names (for Model).
    hfield_name: Vec<String>,
    /// Height field terrain data (Arc for cheap cloning).
    hfield_data: Vec<Arc<HeightFieldData>>,
    /// Original MuJoCo size [x, y, z_top, z_bottom] per hfield asset.
    hfield_size: Vec<[f64; 4]>,
    /// Hfield index per geom (like geom_mesh).
    geom_hfield: Vec<Option<usize>>,
    /// SDF index per geom (like geom_mesh). Always `None` from MJCF — SDF geoms
    /// are populated programmatically.
    geom_sdf: Vec<Option<usize>>,

    // Site arrays
    site_body: Vec<usize>,
    site_type: Vec<GeomType>,
    site_pos: Vec<Vector3<f64>>,
    site_quat: Vec<UnitQuaternion<f64>>,
    site_size: Vec<Vector3<f64>>,
    site_name: Vec<Option<String>>,

    // World frame tracking (for free joint qpos0 initialization)
    // Stores accumulated world positions during tree traversal
    body_world_pos: Vec<Vector3<f64>>,
    body_world_quat: Vec<UnitQuaternion<f64>>,

    // Actuator arrays
    actuator_trntype: Vec<ActuatorTransmission>,
    actuator_dyntype: Vec<ActuatorDynamics>,
    actuator_trnid: Vec<[usize; 2]>,
    actuator_gear: Vec<[f64; 6]>,
    actuator_ctrlrange: Vec<(f64, f64)>,
    actuator_forcerange: Vec<(f64, f64)>,
    actuator_name: Vec<Option<String>>,
    actuator_act_adr: Vec<usize>,
    actuator_act_num: Vec<usize>,
    actuator_gaintype: Vec<GainType>,
    actuator_biastype: Vec<BiasType>,
    actuator_dynprm: Vec<[f64; 3]>,
    actuator_gainprm: Vec<[f64; 9]>,
    actuator_biasprm: Vec<[f64; 9]>,
    actuator_lengthrange: Vec<(f64, f64)>,
    actuator_acc0: Vec<f64>,

    // Total activation states (sum of actuator_act_num)
    na: usize,

    // Options
    timestep: f64,
    gravity: Vector3<f64>,
    wind: Vector3<f64>,
    magnetic: Vector3<f64>,
    density: f64,
    viscosity: f64,
    solver_iterations: usize,
    solver_tolerance: f64,
    impratio: f64,
    regularization: f64,
    default_eq_stiffness: f64,
    default_eq_damping: f64,
    max_constraint_vel: f64,
    max_constraint_angvel: f64,
    friction_smoothing: f64,
    cone: u8,
    ls_iterations: usize,
    ls_tolerance: f64,
    noslip_iterations: usize,
    noslip_tolerance: f64,
    disableflags: u32,
    enableflags: u32,
    integrator: Integrator,
    solver_type: SolverType,
    sleep_tolerance: f64,

    // qpos0 values (built as we process joints)
    qpos0_values: Vec<f64>,

    // Name to index lookups (for actuator wiring)
    joint_name_to_id: HashMap<String, usize>,
    body_name_to_id: HashMap<String, usize>,
    site_name_to_id: HashMap<String, usize>,
    geom_name_to_id: HashMap<String, usize>,

    // Actuator name lookup (for sensor wiring)
    actuator_name_to_id: HashMap<String, usize>,

    // Tendon arrays (populated by process_tendons)
    tendon_name_to_id: HashMap<String, usize>,
    tendon_type: Vec<TendonType>,
    tendon_range: Vec<(f64, f64)>,
    tendon_limited: Vec<bool>,
    tendon_stiffness: Vec<f64>,
    tendon_damping: Vec<f64>,
    tendon_frictionloss: Vec<f64>,
    tendon_lengthspring: Vec<[f64; 2]>,
    tendon_length0: Vec<f64>,
    tendon_name: Vec<Option<String>>,
    tendon_solref: Vec<[f64; 2]>,
    tendon_solimp: Vec<[f64; 5]>,
    tendon_num: Vec<usize>,
    tendon_adr: Vec<usize>,
    wrap_type: Vec<WrapType>,
    wrap_objid: Vec<usize>,
    wrap_prm: Vec<f64>,
    wrap_sidesite: Vec<usize>,

    // Equality constraint arrays
    eq_type: Vec<EqualityType>,
    eq_obj1id: Vec<usize>,
    eq_obj2id: Vec<usize>,
    eq_data: Vec<[f64; 11]>,
    eq_active: Vec<bool>,
    eq_solimp: Vec<[f64; 5]>,
    eq_solref: Vec<[f64; 2]>,
    eq_name: Vec<Option<String>>,

    // Sensor accumulation fields (populated by process_sensors)
    sensor_type: Vec<MjSensorType>,
    sensor_datatype: Vec<MjSensorDataType>,
    sensor_objtype: Vec<MjObjectType>,
    sensor_objid: Vec<usize>,
    sensor_reftype: Vec<MjObjectType>,
    sensor_refid: Vec<usize>,
    sensor_adr: Vec<usize>,
    sensor_dim: Vec<usize>,
    sensor_noise: Vec<f64>,
    sensor_cutoff: Vec<f64>,
    sensor_name_list: Vec<Option<String>>,
    nsensor: usize,
    nsensordata: usize,

    // Contact pairs / excludes (populated by process_contact)
    contact_pairs: Vec<ContactPair>,
    contact_pair_set: HashSet<(usize, usize)>,
    contact_excludes: HashSet<(usize, usize)>,
}

impl ModelBuilder {
    fn new() -> Self {
        // Initialize with world body (body 0)
        Self {
            name: String::new(),
            resolver: DefaultResolver::default(),
            compiler: MjcfCompiler::default(),
            nq: 0,
            nv: 0,

            // World body initialized
            body_parent: vec![0],
            body_rootid: vec![0],
            body_jnt_adr: vec![0],
            body_jnt_num: vec![0],
            body_dof_adr: vec![0],
            body_dof_num: vec![0],
            body_geom_adr: vec![0],
            body_geom_num: vec![0],
            body_pos: vec![Vector3::zeros()],
            body_quat: vec![UnitQuaternion::identity()],
            body_ipos: vec![Vector3::zeros()],
            body_iquat: vec![UnitQuaternion::identity()],
            body_mass: vec![0.0],
            body_inertia: vec![Vector3::zeros()],
            body_name: vec![Some("world".to_string())],
            body_mocapid: vec![None],      // world body is not mocap
            body_sleep_policy: vec![None], // world body has no sleep policy
            nmocap: 0,

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

            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],
            dof_frictionloss: vec![],

            geom_type: vec![],
            geom_body: vec![],
            geom_pos: vec![],
            geom_quat: vec![],
            geom_size: vec![],
            geom_friction: vec![],
            geom_condim: vec![],
            geom_contype: vec![],
            geom_conaffinity: vec![],
            geom_name: vec![],
            geom_mesh: vec![],
            geom_solref: vec![],
            geom_solimp: vec![],

            mesh_name_to_id: HashMap::new(),
            mesh_name: vec![],
            mesh_data: vec![],

            hfield_name_to_id: HashMap::new(),
            hfield_name: vec![],
            hfield_data: vec![],
            hfield_size: vec![],
            geom_hfield: vec![],
            geom_sdf: vec![],

            site_body: vec![],
            site_type: vec![],
            site_pos: vec![],
            site_quat: vec![],
            site_size: vec![],
            site_name: vec![],

            // World frame tracking (world body at origin)
            body_world_pos: vec![],
            body_world_quat: vec![],

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

            na: 0,

            // MuJoCo defaults
            timestep: 0.002,
            gravity: Vector3::new(0.0, 0.0, -9.81),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,
            viscosity: 0.0,
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,
            regularization: 1e-6,
            default_eq_stiffness: 10000.0,
            default_eq_damping: 1000.0,
            max_constraint_vel: 1.0,
            max_constraint_angvel: 1.0,
            friction_smoothing: 1000.0,
            cone: 0,
            ls_iterations: 50,
            ls_tolerance: 0.01,
            noslip_iterations: 0,
            noslip_tolerance: 1e-6,
            disableflags: 0,
            enableflags: 0,
            integrator: Integrator::Euler,
            solver_type: SolverType::PGS,
            sleep_tolerance: 1e-4,

            qpos0_values: vec![],

            joint_name_to_id: HashMap::new(),
            body_name_to_id: HashMap::from([("world".to_string(), 0)]),
            site_name_to_id: HashMap::new(),
            geom_name_to_id: HashMap::new(),

            // Tendons (populated by process_tendons)
            tendon_name_to_id: HashMap::new(),
            tendon_type: vec![],
            tendon_range: vec![],
            tendon_limited: vec![],
            tendon_stiffness: vec![],
            tendon_damping: vec![],
            tendon_frictionloss: vec![],
            tendon_lengthspring: vec![],
            tendon_length0: vec![],
            tendon_name: vec![],
            tendon_solref: vec![],
            tendon_solimp: vec![],
            tendon_num: vec![],
            tendon_adr: vec![],
            wrap_type: vec![],
            wrap_objid: vec![],
            wrap_prm: vec![],
            wrap_sidesite: vec![],

            // Actuator name lookup
            actuator_name_to_id: HashMap::new(),

            // Equality constraints (populated by process_equality_constraints)
            eq_type: vec![],
            eq_obj1id: vec![],
            eq_obj2id: vec![],
            eq_data: vec![],
            eq_active: vec![],
            eq_solimp: vec![],
            eq_solref: vec![],
            eq_name: vec![],

            // Sensors (populated by process_sensors)
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
            sensor_name_list: vec![],
            nsensor: 0,
            nsensordata: 0,

            // Contact pairs / excludes (populated by process_contact)
            contact_pairs: vec![],
            contact_pair_set: HashSet::new(),
            contact_excludes: HashSet::new(),
        }
    }

    fn set_options(&mut self, option: &MjcfOption) {
        self.timestep = option.timestep;
        self.gravity = option.gravity;
        self.solver_iterations = option.iterations;
        self.solver_tolerance = option.tolerance;
        self.impratio = option.impratio;
        self.regularization = option.regularization;
        self.default_eq_stiffness = option.default_eq_stiffness;
        self.default_eq_damping = option.default_eq_damping;
        self.max_constraint_vel = option.max_constraint_vel;
        self.max_constraint_angvel = option.max_constraint_angvel;
        self.friction_smoothing = option.friction_smoothing;
        self.integrator = match option.integrator {
            MjcfIntegrator::Euler => Integrator::Euler,
            MjcfIntegrator::RK4 => Integrator::RungeKutta4,
            MjcfIntegrator::Implicit => Integrator::Implicit,
            MjcfIntegrator::ImplicitFast => Integrator::ImplicitFast,
            MjcfIntegrator::ImplicitSpringDamper => Integrator::ImplicitSpringDamper,
        };
        self.solver_type = match option.solver {
            MjcfSolverType::PGS => SolverType::PGS,
            MjcfSolverType::CG => SolverType::CG,
            MjcfSolverType::Newton => SolverType::Newton,
        };
        self.ls_iterations = option.ls_iterations;
        self.ls_tolerance = option.ls_tolerance;
        self.noslip_iterations = option.noslip_iterations;
        self.noslip_tolerance = option.noslip_tolerance;
        self.magnetic = option.magnetic;
        self.wind = option.wind;
        self.density = option.density;
        self.viscosity = option.viscosity;
        self.sleep_tolerance = option.sleep_tolerance;
        // Set ENABLE_SLEEP from flag
        if option.flag.sleep {
            self.enableflags |= ENABLE_SLEEP;
        }
    }

    /// Process a single mesh asset from MJCF.
    ///
    /// Converts `MjcfMesh` to `TriangleMeshData` with BVH and registers it
    /// in the mesh lookup table. The mesh can then be referenced by geoms.
    ///
    /// # Arguments
    ///
    /// * `mjcf_mesh` - The MJCF mesh definition
    /// * `base_path` - Base directory for resolving relative mesh file paths
    ///
    /// # Errors
    ///
    /// Returns `ModelConversionError` if:
    /// - Mesh name is a duplicate of an already-processed mesh
    /// - Mesh conversion fails (see [`convert_mjcf_mesh`] for details)
    fn process_mesh(
        &mut self,
        mjcf_mesh: &MjcfMesh,
        base_path: Option<&Path>,
    ) -> std::result::Result<(), ModelConversionError> {
        // Check for duplicate mesh names
        if self.mesh_name_to_id.contains_key(&mjcf_mesh.name) {
            return Err(ModelConversionError {
                message: format!("mesh '{}': duplicate mesh name", mjcf_mesh.name),
            });
        }

        // Convert MJCF mesh to TriangleMeshData
        let mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler)?;

        // Register in lookup table
        let mesh_id = self.mesh_data.len();
        self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
        self.mesh_name.push(mjcf_mesh.name.clone());
        self.mesh_data.push(Arc::new(mesh_data));

        Ok(())
    }

    /// Process a height field asset from MJCF.
    fn process_hfield(
        &mut self,
        hfield: &MjcfHfield,
    ) -> std::result::Result<(), ModelConversionError> {
        if self.hfield_name_to_id.contains_key(&hfield.name) {
            return Err(ModelConversionError {
                message: format!("duplicate hfield '{}'", hfield.name),
            });
        }
        let data = convert_mjcf_hfield(hfield)?;
        let id = self.hfield_data.len();
        self.hfield_name_to_id.insert(hfield.name.clone(), id);
        self.hfield_name.push(hfield.name.clone());
        self.hfield_data.push(Arc::new(data));
        self.hfield_size.push(hfield.size);
        Ok(())
    }

    /// Process geoms and sites directly attached to worldbody (body 0).
    ///
    /// In MJCF, the worldbody can have geoms (like ground planes) and sites
    /// directly attached to it. These are static geometries at world coordinates.
    ///
    /// No childclass is applied here — MuJoCo's worldbody accepts no attributes
    /// (including childclass), so worldbody elements use only their own explicit
    /// class or the unnamed top-level default.
    fn process_worldbody_geoms_and_sites(
        &mut self,
        worldbody: &MjcfBody,
    ) -> std::result::Result<(), ModelConversionError> {
        // Track geom start address for body 0
        let geom_adr = self.geom_type.len();

        // Process worldbody geoms
        for geom in &worldbody.geoms {
            let geom = self.resolver.apply_to_geom(geom);
            self.process_geom(&geom, 0)?;
        }

        // Process worldbody sites
        for site in &worldbody.sites {
            let site = self.resolver.apply_to_site(site);
            self.process_site(&site, 0)?;
        }

        // Update body 0's geom range
        let num_geoms = self.geom_type.len() - geom_adr;
        self.body_geom_adr[0] = geom_adr;
        self.body_geom_num[0] = num_geoms;

        Ok(())
    }

    /// Process a body and its descendants.
    ///
    /// This is the public-facing entry point that initializes world frame tracking.
    ///
    /// # Arguments
    /// * `body` - The MJCF body to process
    /// * `parent_id` - Index of the parent body in the Model
    /// * `parent_last_dof` - Index of the last DOF in the parent body's kinematic chain
    ///   (None for bodies attached to world)
    ///
    /// # Returns
    /// The body index in the Model
    fn process_body(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        // Start with parent's world frame (world body is at origin)
        let (parent_world_pos, parent_world_quat) = if parent_id == 0 {
            (Vector3::zeros(), UnitQuaternion::identity())
        } else {
            // For non-world parents, we need to look up their world positions
            // Since we process in topological order, parent is already processed
            // We stored parent's world position when we added them
            (
                self.body_world_pos[parent_id - 1],
                self.body_world_quat[parent_id - 1],
            )
        };
        self.process_body_with_world_frame(
            body,
            parent_id,
            parent_last_dof,
            parent_world_pos,
            parent_world_quat,
            inherited_childclass,
        )
    }

    /// Internal body processing with world frame tracking.
    fn process_body_with_world_frame(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        parent_world_pos: Vector3<f64>,
        parent_world_quat: UnitQuaternion<f64>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        let body_id = self.body_parent.len();

        // Validate mocap body constraints (before any state mutations)
        if body.mocap {
            if parent_id != 0 {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must be a direct child of worldbody",
                        body.name
                    ),
                });
            }
            if !body.joints.is_empty() {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must not have joints (has {})",
                        body.name,
                        body.joints.len()
                    ),
                });
            }
        }

        // Store name mapping
        if !body.name.is_empty() {
            self.body_name_to_id.insert(body.name.clone(), body_id);
        }

        // Determine root: if parent is world, this body is its own root
        let root_id = if parent_id == 0 {
            body_id
        } else {
            self.body_rootid[parent_id]
        };

        // Body position/orientation relative to parent.
        // Priority: euler > axisangle > xyaxes > zaxis > quat (MuJoCo convention).
        let body_pos = body.pos;
        let body_quat = resolve_orientation(
            body.quat,
            body.euler,
            body.axisangle,
            None,
            None,
            &self.compiler,
        );

        // Compute world frame position for this body (used for free joint qpos0)
        let world_pos = parent_world_pos + parent_world_quat * body_pos;
        let world_quat = parent_world_quat * body_quat;

        // Store world positions for child body processing
        self.body_world_pos.push(world_pos);
        self.body_world_quat.push(world_quat);

        // Determine effective childclass for this body's children
        let effective_childclass = body.childclass.as_deref().or(inherited_childclass);

        // Resolve geom defaults once for both inertia computation and geom processing.
        // Apply childclass: if a geom has no explicit class, set it from effective_childclass.
        let resolved_geoms: Vec<MjcfGeom> = body
            .geoms
            .iter()
            .map(|g| {
                let mut g = g.clone();
                if g.class.is_none() {
                    g.class = effective_childclass.map(|s| s.to_string());
                }
                self.resolver.apply_to_geom(&g)
            })
            .collect();

        // Process inertial properties with full MuJoCo semantics.
        // Gated by compiler.inertiafromgeom:
        //   True  — always compute from geoms (overrides explicit <inertial>)
        //   Auto  — compute from geoms only when no explicit <inertial>
        //   False — use explicit <inertial> or zero
        let (mass, inertia, ipos, iquat) = match self.compiler.inertiafromgeom {
            InertiaFromGeom::True => {
                compute_inertia_from_geoms(&resolved_geoms, &self.mesh_name_to_id, &self.mesh_data)
            }
            InertiaFromGeom::Auto => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    compute_inertia_from_geoms(
                        &resolved_geoms,
                        &self.mesh_name_to_id,
                        &self.mesh_data,
                    )
                }
            }
            InertiaFromGeom::False => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    (
                        0.0,
                        Vector3::zeros(),
                        Vector3::zeros(),
                        UnitQuaternion::identity(),
                    )
                }
            }
        };

        // Track joint/DOF addresses for this body
        let jnt_adr = self.jnt_type.len();
        let dof_adr = self.nv;
        let geom_adr = self.geom_type.len();

        // Add body arrays
        self.body_parent.push(parent_id);
        self.body_rootid.push(root_id);
        self.body_jnt_adr.push(jnt_adr);
        self.body_dof_adr.push(dof_adr);
        self.body_geom_adr.push(geom_adr);
        self.body_pos.push(body_pos);
        self.body_quat.push(body_quat);
        self.body_ipos.push(ipos);
        self.body_iquat.push(iquat);
        self.body_mass.push(mass);
        self.body_inertia.push(inertia);
        if body.mocap {
            self.body_mocapid.push(Some(self.nmocap));
            self.nmocap += 1;
        } else {
            self.body_mocapid.push(None);
        }
        self.body_name.push(if body.name.is_empty() {
            None
        } else {
            Some(body.name.clone())
        });
        // Parse body-level sleep policy
        let sleep_policy = body.sleep.as_ref().and_then(|s| match s.as_str() {
            "auto" => Some(SleepPolicy::Auto),
            "allowed" => Some(SleepPolicy::Allowed),
            "never" => Some(SleepPolicy::Never),
            "init" => Some(SleepPolicy::Init),
            _ => {
                warn!(
                    "body '{}': unknown sleep policy '{}', ignoring",
                    body.name, s
                );
                None
            }
        });
        self.body_sleep_policy.push(sleep_policy);

        // Process joints for this body, tracking the last DOF for kinematic tree linkage
        // MuJoCo semantics: first DOF of first joint links to parent body's last DOF,
        // subsequent DOFs form a chain within and across joints
        let mut body_nv = 0;
        let mut current_last_dof = parent_last_dof;

        for joint in &body.joints {
            let mut joint = joint.clone();
            if joint.class.is_none() {
                joint.class = effective_childclass.map(|s| s.to_string());
            }
            let joint = self.resolver.apply_to_joint(&joint);
            let jnt_id =
                self.process_joint(&joint, body_id, current_last_dof, world_pos, world_quat)?;
            let jnt_nv = self.jnt_type[jnt_id].nv();
            body_nv += jnt_nv;

            // Update current_last_dof to the last DOF of this joint
            if jnt_nv > 0 {
                current_last_dof = Some(self.nv - 1);
            }
        }

        // Update body joint/DOF counts
        self.body_jnt_num.push(body.joints.len());
        self.body_dof_num.push(body_nv);

        // Process geoms for this body (using pre-resolved defaults)
        for geom in &resolved_geoms {
            self.process_geom(geom, body_id)?;
        }
        self.body_geom_num.push(body.geoms.len());

        // Process sites for this body
        for site in &body.sites {
            let mut site = site.clone();
            if site.class.is_none() {
                site.class = effective_childclass.map(|s| s.to_string());
            }
            let site = self.resolver.apply_to_site(&site);
            self.process_site(&site, body_id)?;
        }

        // Recursively process children, passing this body's last DOF and world frame
        for child in &body.children {
            self.process_body_with_world_frame(
                child,
                body_id,
                current_last_dof,
                world_pos,
                world_quat,
                effective_childclass,
            )?;
        }

        Ok(body_id)
    }

    /// Process a joint and add its DOFs to the kinematic tree.
    ///
    /// # Arguments
    /// * `joint` - The MJCF joint to process
    /// * `body_id` - Index of the body this joint belongs to
    /// * `parent_last_dof` - Index of the last DOF in the parent kinematic chain
    ///   (from parent body or previous joint on same body)
    /// * `world_pos` - Body's world-frame position (for free joint qpos0)
    /// * `world_quat` - Body's world-frame orientation (for free joint qpos0)
    ///
    /// # MuJoCo Semantics
    ///
    /// The `dof_parent` array forms a kinematic tree:
    /// - First DOF of this joint links to `parent_last_dof`
    /// - Subsequent DOFs within this joint link to the previous DOF
    ///
    /// This enables correct propagation in CRBA/RNE algorithms.
    fn process_joint(
        &mut self,
        joint: &MjcfJoint,
        body_id: usize,
        parent_last_dof: Option<usize>,
        world_pos: Vector3<f64>,
        world_quat: UnitQuaternion<f64>,
    ) -> std::result::Result<usize, ModelConversionError> {
        let jnt_id = self.jnt_type.len();

        // Store name mapping
        if !joint.name.is_empty() {
            self.joint_name_to_id.insert(joint.name.clone(), jnt_id);
        }

        // Convert joint type
        let jnt_type = match joint.joint_type {
            MjcfJointType::Hinge => MjJointType::Hinge,
            MjcfJointType::Slide => MjJointType::Slide,
            MjcfJointType::Ball => MjJointType::Ball,
            MjcfJointType::Free => MjJointType::Free,
            MjcfJointType::Cylindrical | MjcfJointType::Planar => {
                return Err(ModelConversionError {
                    message: format!(
                        "Joint type {:?} not yet supported in Model",
                        joint.joint_type
                    ),
                });
            }
        };

        let nq = jnt_type.nq();
        let nv = jnt_type.nv();
        let qpos_adr = self.nq;
        let dof_adr = self.nv;

        // Add joint arrays
        self.jnt_type.push(jnt_type);
        self.jnt_body.push(body_id);
        self.jnt_qpos_adr.push(qpos_adr);
        self.jnt_dof_adr.push(dof_adr);
        self.jnt_pos.push(joint.pos);
        // Normalize joint axis, handling zero vector edge case
        // MuJoCo defaults to Z-axis for unspecified/zero axes
        let axis = if joint.axis.norm() > 1e-10 {
            joint.axis.normalize()
        } else {
            warn!(
                joint_name = ?joint.name,
                "Joint axis is zero or near-zero, defaulting to Z-axis"
            );
            Vector3::z()
        };
        self.jnt_axis.push(axis);
        // Autolimits: infer limited from range presence when no explicit value.
        let limited = match joint.limited {
            Some(v) => v,
            None => {
                if self.compiler.autolimits {
                    joint.range.is_some()
                } else {
                    false
                }
            }
        };
        self.jnt_limited.push(limited);

        // Angle conversion: hinge and ball joint range/ref/springref are angle-valued.
        // When compiler.angle == Degree (default), convert to radians.
        // Slide and Free joints are not angle-valued — never convert.
        let deg2rad = std::f64::consts::PI / 180.0;
        let is_angle_joint = matches!(jnt_type, MjJointType::Hinge | MjJointType::Ball);
        let convert = is_angle_joint && self.compiler.angle == AngleUnit::Degree;

        let range = joint
            .range
            .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI));
        self.jnt_range.push(if convert {
            (range.0 * deg2rad, range.1 * deg2rad)
        } else {
            range
        });
        self.jnt_stiffness.push(joint.stiffness);

        // ref_pos and spring_ref: angle-valued for hinge joints only.
        // Ball joint ref is a quaternion, not angle-valued.
        let convert_ref =
            matches!(jnt_type, MjJointType::Hinge) && self.compiler.angle == AngleUnit::Degree;
        self.jnt_springref.push(if convert_ref {
            joint.spring_ref * deg2rad
        } else {
            joint.spring_ref
        });
        self.jnt_damping.push(joint.damping);
        self.jnt_armature.push(joint.armature);
        self.jnt_solref
            .push(joint.solref_limit.unwrap_or(DEFAULT_SOLREF));
        self.jnt_solimp
            .push(joint.solimp_limit.unwrap_or(DEFAULT_SOLIMP));
        self.jnt_name.push(if joint.name.is_empty() {
            None
        } else {
            Some(joint.name.clone())
        });

        // Add DOF arrays with correct kinematic tree linkage
        // MuJoCo semantics: dof_parent forms a tree structure for CRBA/RNE
        for i in 0..nv {
            self.dof_body.push(body_id);
            self.dof_jnt.push(jnt_id);

            // First DOF links to parent chain, subsequent DOFs link within joint
            let parent = if i == 0 {
                parent_last_dof
            } else {
                Some(dof_adr + i - 1)
            };
            self.dof_parent.push(parent);

            self.dof_armature.push(joint.armature);
            self.dof_damping.push(joint.damping);
            self.dof_frictionloss.push(joint.frictionloss);
        }

        // Add qpos0 values (default positions)
        match jnt_type {
            MjJointType::Hinge => {
                let ref_val = if self.compiler.angle == AngleUnit::Degree {
                    joint.ref_pos * deg2rad
                } else {
                    joint.ref_pos
                };
                self.qpos0_values.push(ref_val);
            }
            MjJointType::Slide => {
                // Slide ref is translational — not angle-valued
                self.qpos0_values.push(joint.ref_pos);
            }
            MjJointType::Ball => {
                // Quaternion identity [w, x, y, z] = [1, 0, 0, 0]
                self.qpos0_values.extend_from_slice(&[1.0, 0.0, 0.0, 0.0]);
            }
            MjJointType::Free => {
                // Free joint qpos0 is the body's world position and orientation
                // Position [x, y, z] from world frame, quaternion [w, x, y, z] from world frame
                let q = world_quat.into_inner();
                self.qpos0_values.extend_from_slice(&[
                    world_pos.x,
                    world_pos.y,
                    world_pos.z,
                    q.w,
                    q.i,
                    q.j,
                    q.k,
                ]);
            }
        }

        // Update dimensions
        self.nq += nq;
        self.nv += nv;

        Ok(jnt_id)
    }

    fn process_geom(
        &mut self,
        geom: &MjcfGeom,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let geom_id = self.geom_type.len();

        // Convert geom type
        let geom_type = match geom.geom_type {
            MjcfGeomType::Sphere => GeomType::Sphere,
            MjcfGeomType::Box => GeomType::Box,
            MjcfGeomType::Capsule => GeomType::Capsule,
            MjcfGeomType::Cylinder => GeomType::Cylinder,
            MjcfGeomType::Ellipsoid => GeomType::Ellipsoid,
            MjcfGeomType::Plane => GeomType::Plane,
            MjcfGeomType::Mesh | MjcfGeomType::TriangleMesh => GeomType::Mesh,
            MjcfGeomType::Hfield => GeomType::Hfield,
            MjcfGeomType::Sdf => GeomType::Sdf,
        };

        // Handle mesh geom linking
        let geom_mesh_ref = if geom_type == GeomType::Mesh {
            match &geom.mesh {
                Some(mesh_name) => {
                    let mesh_id = self.mesh_name_to_id.get(mesh_name).ok_or_else(|| {
                        ModelConversionError {
                            message: format!(
                                "geom '{}': references undefined mesh '{}'",
                                geom.name.as_deref().unwrap_or("<unnamed>"),
                                mesh_name
                            ),
                        }
                    })?;
                    Some(*mesh_id)
                }
                None => {
                    return Err(ModelConversionError {
                        message: format!(
                            "geom '{}': type is mesh but no mesh attribute specified",
                            geom.name.as_deref().unwrap_or("<unnamed>")
                        ),
                    });
                }
            }
        } else {
            None
        };

        // Handle hfield geom linking
        let geom_hfield_ref = if geom_type == GeomType::Hfield {
            match &geom.hfield {
                Some(name) => {
                    let hfield_id =
                        self.hfield_name_to_id
                            .get(name)
                            .ok_or_else(|| ModelConversionError {
                                message: format!(
                                    "geom '{}': references undefined hfield '{}'",
                                    geom.name.as_deref().unwrap_or("<unnamed>"),
                                    name
                                ),
                            })?;
                    Some(*hfield_id)
                }
                None => {
                    return Err(ModelConversionError {
                        message: format!(
                            "geom '{}': type is hfield but no hfield attribute specified",
                            geom.name.as_deref().unwrap_or("<unnamed>")
                        ),
                    });
                }
            }
        } else {
            None
        };

        // Handle fromto for capsules/cylinders
        let (pos, quat, size) = if let Some(fromto) = geom.fromto {
            compute_fromto_pose(fromto, &geom.size)
        } else {
            // Orientation resolution: euler > axisangle > xyaxes > zaxis > quat.
            let orientation = resolve_orientation(
                geom.quat,
                geom.euler,
                geom.axisangle,
                geom.xyaxes,
                geom.zaxis,
                &self.compiler,
            );
            (
                geom.pos,
                orientation,
                geom_size_to_vec3(&geom.size, geom_type),
            )
        };

        // Override geom_size for hfield geoms with asset dimensions.
        // geom_size_to_vec3() has no access to hfield asset data (only gets the MJCF
        // geom's size attribute, which is typically absent for hfield geoms).
        let size = if let Some(hfield_id) = geom_hfield_ref {
            let hf_size = &self.hfield_size[hfield_id];
            Vector3::new(hf_size[0], hf_size[1], hf_size[2])
        } else {
            size
        };

        self.geom_type.push(geom_type);
        self.geom_body.push(body_id);
        self.geom_pos.push(pos);
        self.geom_quat.push(quat);
        self.geom_size.push(size);
        self.geom_friction.push(geom.friction);

        // Validate and clamp condim to valid values {1, 3, 4, 6}
        // Invalid values are rounded up to the next valid value per MuJoCo convention
        let condim = match geom.condim {
            1 => 1,
            2 => {
                tracing::warn!(
                    "Geom {:?} has invalid condim=2, rounding up to 3",
                    geom.name
                );
                3
            }
            3 => 3,
            4 => 4,
            5 => {
                tracing::warn!(
                    "Geom {:?} has invalid condim=5, rounding up to 6",
                    geom.name
                );
                6
            }
            c if c >= 6 => {
                if c > 6 {
                    tracing::warn!(
                        "Geom {:?} has invalid condim={}, clamping to 6",
                        geom.name,
                        c
                    );
                }
                6
            }
            c => {
                // condim <= 0
                tracing::warn!(
                    "Geom {:?} has invalid condim={}, defaulting to 3",
                    geom.name,
                    c
                );
                3
            }
        };
        self.geom_condim.push(condim);
        #[allow(clippy::cast_sign_loss)]
        {
            self.geom_contype.push(geom.contype as u32);
            self.geom_conaffinity.push(geom.conaffinity as u32);
        }
        self.geom_name.push(geom.name.clone());
        if let Some(ref name) = geom.name {
            if !name.is_empty() {
                self.geom_name_to_id.insert(name.clone(), geom_id);
            }
        }
        self.geom_mesh.push(geom_mesh_ref);
        self.geom_hfield.push(geom_hfield_ref);
        self.geom_sdf.push(None); // SDF geoms are programmatic — never set via MJCF

        // Solver parameters (fall back to MuJoCo defaults if not specified in MJCF)
        self.geom_solref.push(geom.solref.unwrap_or(DEFAULT_SOLREF));
        self.geom_solimp.push(geom.solimp.unwrap_or(DEFAULT_SOLIMP));

        Ok(geom_id)
    }

    fn process_site(
        &mut self,
        site: &MjcfSite,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let site_id = self.site_body.len();

        // Store name mapping for actuator site transmission
        if !site.name.is_empty() {
            self.site_name_to_id.insert(site.name.clone(), site_id);
        }

        self.site_body.push(body_id);

        // Convert site type string to GeomType (sphere is the MuJoCo default)
        let geom_type = match site.site_type.as_str() {
            "capsule" => GeomType::Capsule,
            "cylinder" => GeomType::Cylinder,
            "box" => GeomType::Box,
            "ellipsoid" => GeomType::Ellipsoid,
            _ => GeomType::Sphere, // Default for "sphere" and unknown types
        };
        self.site_type.push(geom_type);

        self.site_pos.push(site.pos);
        self.site_quat.push(resolve_orientation(
            site.quat,
            site.euler,
            site.axisangle,
            site.xyaxes,
            site.zaxis,
            &self.compiler,
        ));

        // Convert site size (MuJoCo uses single value for sphere, array for others)
        let size = if site.size.is_empty() {
            Vector3::new(0.01, 0.01, 0.01) // Default small size
        } else if site.size.len() == 1 {
            let s = site.size[0];
            Vector3::new(s, s, s)
        } else {
            Vector3::new(
                site.size[0],
                site.size.get(1).copied().unwrap_or(site.size[0]),
                site.size.get(2).copied().unwrap_or(site.size[0]),
            )
        };
        self.site_size.push(size);

        self.site_name.push(if site.name.is_empty() {
            None
        } else {
            Some(site.name.clone())
        });

        Ok(site_id)
    }

    fn process_tendons(
        &mut self,
        tendons: &[MjcfTendon],
    ) -> std::result::Result<(), ModelConversionError> {
        for (t_idx, tendon) in tendons.iter().enumerate() {
            let tendon = self.resolver.apply_to_tendon(tendon);
            if !tendon.name.is_empty() {
                self.tendon_name_to_id.insert(tendon.name.clone(), t_idx);
            }
            self.tendon_type.push(match tendon.tendon_type {
                MjcfTendonType::Fixed => TendonType::Fixed,
                MjcfTendonType::Spatial => TendonType::Spatial,
            });
            self.tendon_range
                .push(tendon.range.unwrap_or((-f64::MAX, f64::MAX)));
            // Autolimits: infer limited from range presence.
            let tendon_limited = match tendon.limited {
                Some(v) => v,
                None => {
                    if self.compiler.autolimits {
                        tendon.range.is_some()
                    } else {
                        false
                    }
                }
            };
            self.tendon_limited.push(tendon_limited);
            self.tendon_stiffness.push(tendon.stiffness);
            self.tendon_damping.push(tendon.damping);
            self.tendon_frictionloss.push(tendon.frictionloss);
            self.tendon_name.push(if tendon.name.is_empty() {
                None
            } else {
                Some(tendon.name.clone())
            });
            self.tendon_solref.push(DEFAULT_SOLREF);
            self.tendon_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]); // MuJoCo defaults
            // S1/S3: Use parsed springlength, or sentinel [-1, -1] for auto-compute
            self.tendon_lengthspring.push(match tendon.springlength {
                Some(pair) => pair.into(),
                None => [-1.0, -1.0], // sentinel: resolved to [length0, length0] later
            });
            self.tendon_length0.push(0.0); // Computed after construction from qpos0

            let wrap_start = self.wrap_type.len();
            self.tendon_adr.push(wrap_start);

            match tendon.tendon_type {
                MjcfTendonType::Fixed => {
                    for (joint_name, coef) in &tendon.joints {
                        // Resolve joint name → index → DOF address
                        let jnt_idx =
                            *self
                                .joint_name_to_id
                                .get(joint_name.as_str())
                                .ok_or_else(|| ModelConversionError {
                                    message: format!(
                                        "Tendon '{}' references unknown joint '{}'",
                                        tendon.name, joint_name
                                    ),
                                })?;
                        // Fixed tendons assume qposadr == dofadr (true for hinge/slide).
                        // Ball/free joints have different qpos and dof dimensions, so
                        // a linear coupling L = coef * qpos[dof_adr] would be incorrect.
                        let jnt_type = self.jnt_type[jnt_idx];
                        if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                            warn!(
                                "Tendon '{}' references {} joint '{}' — fixed tendons only \
                                 support hinge/slide joints. Ball/free joints will produce \
                                 incorrect results.",
                                tendon.name,
                                match jnt_type {
                                    MjJointType::Ball => "ball",
                                    MjJointType::Free => "free",
                                    _ => "unknown",
                                },
                                joint_name
                            );
                        }
                        let dof_adr = self.jnt_dof_adr[jnt_idx];
                        self.wrap_type.push(WrapType::Joint);
                        self.wrap_objid.push(dof_adr);
                        self.wrap_prm.push(*coef);
                        self.wrap_sidesite.push(usize::MAX);
                    }
                }
                MjcfTendonType::Spatial => {
                    for elem in &tendon.path_elements {
                        match elem {
                            SpatialPathElement::Site { site } => {
                                let site_idx = *self
                                    .site_name_to_id
                                    .get(site.as_str())
                                    .ok_or_else(|| ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' references unknown site '{}'",
                                            tendon.name, site
                                        ),
                                    })?;
                                self.wrap_type.push(WrapType::Site);
                                self.wrap_objid.push(site_idx);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                            SpatialPathElement::Geom { geom, sidesite } => {
                                let geom_id =
                                    *self.geom_name_to_id.get(geom.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown geom '{}'",
                                                tendon.name, geom
                                            ),
                                        }
                                    })?;
                                // Validate geom type: must be Sphere or Cylinder
                                let gt = self.geom_type[geom_id];
                                if gt != GeomType::Sphere && gt != GeomType::Cylinder {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has type {:?}, \
                                             but only Sphere and Cylinder are supported",
                                            tendon.name, geom, gt
                                        ),
                                    });
                                }
                                // Validate geom radius > 0 (rule 5)
                                let radius = self.geom_size[geom_id].x;
                                if radius <= 0.0 {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has zero or \
                                             negative radius {}",
                                            tendon.name, geom, radius
                                        ),
                                    });
                                }
                                let ss_id = if let Some(ss) = sidesite {
                                    *self.site_name_to_id.get(ss.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown sidesite '{}'",
                                                tendon.name, ss
                                            ),
                                        }
                                    })?
                                } else {
                                    usize::MAX
                                };
                                self.wrap_type.push(WrapType::Geom);
                                self.wrap_objid.push(geom_id);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(ss_id);
                            }
                            SpatialPathElement::Pulley { divisor } => {
                                self.wrap_type.push(WrapType::Pulley);
                                self.wrap_objid.push(0);
                                self.wrap_prm.push(*divisor);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                        }
                    }
                }
            }
            self.tendon_num.push(self.wrap_type.len() - wrap_start);
        }
        Ok(())
    }

    fn process_actuator(
        &mut self,
        actuator: &MjcfActuator,
    ) -> std::result::Result<usize, ModelConversionError> {
        let act_id = self.actuator_trntype.len();

        // Register actuator name for sensor wiring
        if !actuator.name.is_empty() {
            self.actuator_name_to_id
                .insert(actuator.name.clone(), act_id);
        }

        // Determine transmission type and target (2-slot: [primary, secondary])
        // Secondary slot is usize::MAX when unused (Joint, Tendon, Site without refsite).
        let (trntype, trnid) = if let Some(ref joint_name) = actuator.joint {
            let jnt_id =
                self.joint_name_to_id
                    .get(joint_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown joint: {joint_name}"),
                    })?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses joint transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            (ActuatorTransmission::Joint, [*jnt_id, usize::MAX])
        } else if let Some(ref tendon_name) = actuator.tendon {
            let tendon_idx = *self
                .tendon_name_to_id
                .get(tendon_name.as_str())
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "Actuator '{}' references unknown tendon '{}'",
                        actuator.name, tendon_name
                    ),
                })?;
            if actuator.refsite.is_some() {
                warn!(
                    "Actuator '{}' has refsite but uses tendon transmission — \
                     refsite is only meaningful for site transmissions; ignoring",
                    actuator.name
                );
            }
            (ActuatorTransmission::Tendon, [tendon_idx, usize::MAX])
        } else if let Some(ref site_name) = actuator.site {
            let site_id =
                *self
                    .site_name_to_id
                    .get(site_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("Actuator references unknown site: {site_name}"),
                    })?;
            let refsite_id = if let Some(ref refsite_name) = actuator.refsite {
                *self
                    .site_name_to_id
                    .get(refsite_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Actuator '{}' references unknown refsite: {refsite_name}",
                            actuator.name
                        ),
                    })?
            } else {
                usize::MAX
            };
            (ActuatorTransmission::Site, [site_id, refsite_id])
        } else if let Some(ref body_name) = actuator.body {
            // Body transmission (adhesion actuators) not yet implemented
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' uses body transmission '{}' which is not yet supported.",
                    actuator.name, body_name
                ),
            });
        } else {
            return Err(ModelConversionError {
                message: format!(
                    "Actuator '{}' has no transmission target (joint, tendon, site, or body)",
                    actuator.name
                ),
            });
        };

        // Resolve per-type defaults for Option fields.
        // These must be resolved before dyntype/gain/bias decisions that depend on them.
        let timeconst = actuator.timeconst.unwrap_or(match actuator.actuator_type {
            MjcfActuatorType::Cylinder => 1.0,
            _ => 0.0,
        });
        let kv = actuator.kv.unwrap_or(match actuator.actuator_type {
            MjcfActuatorType::Velocity => 1.0,
            _ => 0.0, // Damper defaults to 0.0 (MuJoCo: mjs_setToDamper uses kv=0)
        });

        // Damper and Adhesion actuators force ctrllimited
        // (MuJoCo: mjs_setToDamper and mjs_setToAdhesion both set ctrllimited=1).
        let ctrllimited = match actuator.actuator_type {
            MjcfActuatorType::Damper | MjcfActuatorType::Adhesion => true,
            _ => match actuator.ctrllimited {
                Some(v) => v,
                None => {
                    if self.compiler.autolimits {
                        actuator.ctrlrange.is_some()
                    } else {
                        false
                    }
                }
            },
        };

        // Determine dynamics type.
        // MuJoCo semantics: dyntype is determined by the shortcut type AND timeconst.
        // Position defaults to None; if timeconst > 0, uses FilterExact (exact discrete filter).
        // Cylinder always uses Filter (Euler-approximated; timeconst defaults to 1.0).
        // Motor/Damper/Adhesion/Velocity: no dynamics.
        // General: explicit dyntype attribute; defaults to None.
        // Muscle: muscle activation dynamics.
        let dyntype = match actuator.actuator_type {
            MjcfActuatorType::Motor
            | MjcfActuatorType::Damper
            | MjcfActuatorType::Adhesion
            | MjcfActuatorType::Velocity => ActuatorDynamics::None,
            MjcfActuatorType::General => {
                // <general> uses explicit dyntype attribute; defaults to None
                match &actuator.dyntype {
                    Some(s) => parse_dyntype(s)?,
                    None => ActuatorDynamics::None,
                }
            }
            MjcfActuatorType::Position => {
                if timeconst > 0.0 {
                    ActuatorDynamics::FilterExact // MuJoCo: mjDYN_FILTEREXACT
                } else {
                    ActuatorDynamics::None
                }
            }
            MjcfActuatorType::Muscle => ActuatorDynamics::Muscle,
            MjcfActuatorType::Cylinder => ActuatorDynamics::Filter,
        };

        self.actuator_trntype.push(trntype);
        self.actuator_dyntype.push(dyntype);
        self.actuator_trnid.push(trnid);
        self.actuator_gear.push(actuator.gear);

        // Gate ctrlrange/forcerange on ctrllimited/forcelimited (MuJoCo semantics).
        // When limited=false, range is effectively unbounded regardless of attribute value.
        self.actuator_ctrlrange.push(if ctrllimited {
            actuator.ctrlrange.unwrap_or((-1.0, 1.0))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });
        let forcelimited = match actuator.forcelimited {
            Some(v) => v,
            None => {
                if self.compiler.autolimits {
                    actuator.forcerange.is_some()
                } else {
                    false
                }
            }
        };
        self.actuator_forcerange.push(if forcelimited {
            actuator
                .forcerange
                .unwrap_or((f64::NEG_INFINITY, f64::INFINITY))
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        });

        self.actuator_name.push(if actuator.name.is_empty() {
            None
        } else {
            Some(actuator.name.clone())
        });

        // Compute activation state count based on dynamics type
        let act_num = match dyntype {
            ActuatorDynamics::None => 0,
            ActuatorDynamics::Filter
            | ActuatorDynamics::FilterExact
            | ActuatorDynamics::Integrator
            | ActuatorDynamics::Muscle => 1,
        };

        self.actuator_act_adr.push(self.na);
        self.actuator_act_num.push(act_num);
        self.na += act_num;

        // Gain/Bias/Dynamics parameters — expand shortcut type to general actuator.
        // Reference: MuJoCo src/user/user_api.cc (mjs_setToMotor, mjs_setToPosition, etc.)
        let (gaintype, biastype, gainprm, biasprm, dynprm) = match actuator.actuator_type {
            MjcfActuatorType::Motor => (
                GainType::Fixed,
                BiasType::None,
                {
                    let mut p = [0.0; 9];
                    p[0] = 1.0; // unit gain
                    p
                },
                [0.0; 9],
                [0.0; 3],
            ),

            MjcfActuatorType::Position => {
                let kp = actuator.kp; // default 1.0
                // kv resolved above from Option<f64> (default 0.0 for position)
                let tc = timeconst; // resolved above (default 0.0 for position)
                let mut gp = [0.0; 9];
                gp[0] = kp;
                let mut bp = [0.0; 9];
                bp[1] = -kp;
                bp[2] = -kv;
                (GainType::Fixed, BiasType::Affine, gp, bp, [tc, 0.0, 0.0])
            }

            MjcfActuatorType::Velocity => {
                // kv resolved above from Option<f64> (default 1.0 for velocity)
                let mut gp = [0.0; 9];
                gp[0] = kv;
                let mut bp = [0.0; 9];
                bp[2] = -kv;
                (GainType::Fixed, BiasType::Affine, gp, bp, [0.0; 3])
            }

            MjcfActuatorType::Damper => {
                // kv resolved above from Option<f64> (default 0.0 for damper)
                let mut gp = [0.0; 9];
                gp[2] = -kv; // gain = -kv * velocity
                (GainType::Affine, BiasType::None, gp, [0.0; 9], [0.0; 3])
            }

            MjcfActuatorType::Cylinder => {
                let area = if let Some(d) = actuator.diameter {
                    std::f64::consts::PI / 4.0 * d * d
                } else {
                    actuator.area
                };
                let tc = timeconst; // resolved above (default 1.0 for cylinder)
                let mut gp = [0.0; 9];
                gp[0] = area;
                let mut bp = [0.0; 9];
                bp[0] = actuator.bias[0];
                bp[1] = actuator.bias[1];
                bp[2] = actuator.bias[2];
                (GainType::Fixed, BiasType::Affine, gp, bp, [tc, 0.0, 0.0])
            }

            MjcfActuatorType::Adhesion => {
                let mut gp = [0.0; 9];
                gp[0] = actuator.gain;
                (GainType::Fixed, BiasType::None, gp, [0.0; 9], [0.0; 3])
            }

            MjcfActuatorType::Muscle => {
                // Muscle: unchanged from #5 implementation.
                let gp = [
                    actuator.range.0,
                    actuator.range.1,
                    actuator.force,
                    actuator.scale,
                    actuator.lmin,
                    actuator.lmax,
                    actuator.vmax,
                    actuator.fpmax,
                    actuator.fvmax,
                ];
                (
                    GainType::Muscle,
                    BiasType::Muscle,
                    gp,
                    gp, // biasprm = gainprm (shared layout, MuJoCo convention)
                    [
                        actuator.muscle_timeconst.0,
                        actuator.muscle_timeconst.1,
                        0.0,
                    ],
                )
            }

            MjcfActuatorType::General => {
                // <general> uses explicit attributes; defaults match MuJoCo's
                // mjs_defaultActuator: gaintype=fixed, biastype=none,
                // gainprm=[1,0,...], biasprm=[0,...], dynprm=[1,0,0].
                let gt = match &actuator.gaintype {
                    Some(s) => parse_gaintype(s)?,
                    None => GainType::Fixed,
                };
                let bt = match &actuator.biastype {
                    Some(s) => parse_biastype(s)?,
                    None => BiasType::None,
                };
                let gainprm_default = {
                    let mut d = [0.0; 9];
                    d[0] = 1.0; // MuJoCo default
                    d
                };
                let gp = if let Some(v) = &actuator.gainprm {
                    floats_to_array(v, gainprm_default)
                } else {
                    gainprm_default
                };
                let bp = if let Some(v) = &actuator.biasprm {
                    floats_to_array(v, [0.0; 9])
                } else {
                    [0.0; 9]
                };
                let dynprm_default = [1.0, 0.0, 0.0]; // MuJoCo: dynprm[0]=1
                let dp = if let Some(v) = &actuator.dynprm {
                    floats_to_array(v, dynprm_default)
                } else {
                    dynprm_default
                };
                (gt, bt, gp, bp, dp)
            }
        };

        self.actuator_gaintype.push(gaintype);
        self.actuator_biastype.push(biastype);
        self.actuator_gainprm.push(gainprm);
        self.actuator_biasprm.push(biasprm);
        self.actuator_dynprm.push(dynprm);

        // Lengthrange and acc0: initialized to zero, computed by compute_muscle_params()
        self.actuator_lengthrange.push((0.0, 0.0));
        self.actuator_acc0.push(0.0);

        Ok(act_id)
    }

    /// Process sensor definitions from MJCF.
    ///
    /// Converts `MjcfSensor` objects into the 13 pipeline sensor arrays.
    /// Must be called AFTER all bodies, joints, tendons, and actuators are processed
    /// (requires name→id lookups for object resolution).
    fn process_sensors(
        &mut self,
        sensors: &[MjcfSensor],
    ) -> std::result::Result<(), ModelConversionError> {
        let mut adr = 0usize;

        for mjcf_sensor in sensors {
            let mjcf_sensor = self.resolver.apply_to_sensor(mjcf_sensor);
            let Some(sensor_type) = convert_sensor_type(mjcf_sensor.sensor_type) else {
                // Unsupported type (Jointlimitfrc, Tendonlimitfrc) — skip with log
                warn!(
                    "Skipping unsupported sensor type '{:?}' (sensor '{}')",
                    mjcf_sensor.sensor_type, mjcf_sensor.name,
                );
                continue;
            };
            let dim = sensor_type.dim();
            let datatype = sensor_datatype(sensor_type);
            let (objtype, objid) =
                self.resolve_sensor_object(sensor_type, mjcf_sensor.objname.as_deref())?;

            self.sensor_type.push(sensor_type);
            self.sensor_datatype.push(datatype);
            self.sensor_objtype.push(objtype);
            self.sensor_objid.push(objid);
            self.sensor_reftype.push(MjObjectType::None);
            self.sensor_refid.push(0);
            self.sensor_adr.push(adr);
            self.sensor_dim.push(dim);
            self.sensor_noise.push(mjcf_sensor.noise);
            self.sensor_cutoff.push(mjcf_sensor.cutoff);
            self.sensor_name_list.push(if mjcf_sensor.name.is_empty() {
                None
            } else {
                Some(mjcf_sensor.name.clone())
            });

            adr += dim;
        }

        self.nsensor = self.sensor_type.len();
        self.nsensordata = adr;
        Ok(())
    }

    /// Process `<contact>` element: resolve `<pair>` and `<exclude>` entries.
    ///
    /// For each `<pair>`: apply defaults class, resolve geom names to indices,
    /// compute geom-combination fallbacks for unspecified attributes, and build
    /// a fully-resolved `ContactPair`. Duplicate pairs (same geom pair) use
    /// last-wins semantics.
    ///
    /// For each `<exclude>`: resolve body names to indices and insert into the
    /// exclude set.
    fn process_contact(
        &mut self,
        contact: &MjcfContact,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process pairs
        for mjcf_pair in &contact.pairs {
            // Stage 1: apply defaults class
            let pair = self.resolver.apply_to_pair(mjcf_pair);

            // Resolve geom names to indices
            let g1 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom1",
                            pair.geom1
                        ),
                    })?;
            let g2 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom2)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom2",
                            pair.geom2
                        ),
                    })?;

            // Stage 2: geom-combination fallbacks for unspecified attributes
            let condim = pair
                .condim
                .unwrap_or_else(|| self.geom_condim[g1].max(self.geom_condim[g2]));

            let friction = pair.friction.unwrap_or_else(|| {
                let f1 = self.geom_friction[g1];
                let f2 = self.geom_friction[g2];
                // Geometric mean in 3D, then expand to 5D: [s, s, t, r, r]
                let s = (f1.x * f2.x).sqrt();
                let t = (f1.y * f2.y).sqrt();
                let r = (f1.z * f2.z).sqrt();
                [s, s, t, r, r]
            });

            let solref = pair.solref.unwrap_or_else(|| {
                // Element-wise min (our standing approximation)
                [
                    self.geom_solref[g1][0].min(self.geom_solref[g2][0]),
                    self.geom_solref[g1][1].min(self.geom_solref[g2][1]),
                ]
            });

            let solimp = pair.solimp.unwrap_or_else(|| {
                // Element-wise max (our standing approximation)
                let s1 = self.geom_solimp[g1];
                let s2 = self.geom_solimp[g2];
                [
                    s1[0].max(s2[0]),
                    s1[1].max(s2[1]),
                    s1[2].max(s2[2]),
                    s1[3].max(s2[3]),
                    s1[4].max(s2[4]),
                ]
            });

            // solreffriction falls back to the pair's resolved solref
            let solreffriction = pair.solreffriction.unwrap_or(solref);

            // margin/gap: geom-level not yet parsed, default to 0.0
            let margin = pair.margin.unwrap_or(0.0);
            let gap = pair.gap.unwrap_or(0.0);

            let contact_pair = ContactPair {
                geom1: g1,
                geom2: g2,
                condim,
                friction,
                solref,
                solreffriction,
                solimp,
                margin,
                gap,
            };

            // Deduplicate: canonical key (min, max) for symmetry
            let key = (g1.min(g2), g1.max(g2));
            if self.contact_pair_set.contains(&key) {
                // Last-wins: replace existing entry
                let pos = self
                    .contact_pairs
                    .iter()
                    .position(|p| (p.geom1.min(p.geom2), p.geom1.max(p.geom2)) == key)
                    .ok_or_else(|| ModelConversionError {
                        message: "contact_pair_set and contact_pairs out of sync".into(),
                    })?;
                self.contact_pairs[pos] = contact_pair;
            } else {
                self.contact_pair_set.insert(key);
                self.contact_pairs.push(contact_pair);
            }
        }

        // Process excludes
        for mjcf_exclude in &contact.excludes {
            let b1 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body1",
                        mjcf_exclude.body1
                    ),
                })?;
            let b2 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body2)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body2",
                        mjcf_exclude.body2
                    ),
                })?;
            // Canonical key for symmetry
            self.contact_excludes.insert((b1.min(b2), b1.max(b2)));
        }

        Ok(())
    }

    /// Returns (MjObjectType, objid) for a given sensor type and MJCF objname.
    fn resolve_sensor_object(
        &self,
        sensor_type: MjSensorType,
        objname: Option<&str>,
    ) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
        // User sensors have no object reference
        if sensor_type == MjSensorType::User {
            return Ok((MjObjectType::None, 0));
        }

        let name = objname.ok_or_else(|| ModelConversionError {
            message: "sensor missing object name".into(),
        })?;

        match sensor_type {
            // Joint sensors: objname is a joint name
            MjSensorType::JointPos
            | MjSensorType::JointVel
            | MjSensorType::BallQuat
            | MjSensorType::BallAngVel
            | MjSensorType::JointLimitFrc => {
                let id = *self
                    .joint_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown joint '{name}'"),
                    })?;
                Ok((MjObjectType::Joint, id))
            }

            // Tendon sensors: objname is a tendon name
            MjSensorType::TendonPos | MjSensorType::TendonVel | MjSensorType::TendonLimitFrc => {
                let id = *self
                    .tendon_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown tendon '{name}'"),
                    })?;
                Ok((MjObjectType::Tendon, id))
            }

            // Actuator sensors: objname is an actuator name
            MjSensorType::ActuatorPos | MjSensorType::ActuatorVel | MjSensorType::ActuatorFrc => {
                let id =
                    *self
                        .actuator_name_to_id
                        .get(name)
                        .ok_or_else(|| ModelConversionError {
                            message: format!("sensor references unknown actuator '{name}'"),
                        })?;
                Ok((MjObjectType::Actuator, id))
            }

            // Site-attached sensors: objname is a site name
            MjSensorType::Accelerometer
            | MjSensorType::Velocimeter
            | MjSensorType::Gyro
            | MjSensorType::Force
            | MjSensorType::Torque
            | MjSensorType::Magnetometer
            | MjSensorType::Rangefinder => {
                let id = *self
                    .site_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown site '{name}'"),
                    })?;
                Ok((MjObjectType::Site, id))
            }

            // Touch: MJCF uses site=, but pipeline expects geom ID.
            // Resolve site → body → first geom on that body.
            MjSensorType::Touch => {
                let site_id =
                    *self
                        .site_name_to_id
                        .get(name)
                        .ok_or_else(|| ModelConversionError {
                            message: format!("touch sensor references unknown site '{name}'"),
                        })?;
                let body_id = *self
                    .site_body
                    .get(site_id)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "touch sensor: site_id {site_id} out of range for site_body"
                        ),
                    })?;
                // Find first geom belonging to this body
                let geom_id = self
                    .geom_body
                    .iter()
                    .position(|&b| b == body_id)
                    .unwrap_or(usize::MAX);
                Ok((MjObjectType::Geom, geom_id))
            }

            // Frame sensors: resolve objname by trying site → body → geom
            MjSensorType::FramePos
            | MjSensorType::FrameQuat
            | MjSensorType::FrameXAxis
            | MjSensorType::FrameYAxis
            | MjSensorType::FrameZAxis
            | MjSensorType::FrameLinVel
            | MjSensorType::FrameAngVel
            | MjSensorType::FrameLinAcc
            | MjSensorType::FrameAngAcc => {
                if let Some(&id) = self.site_name_to_id.get(name) {
                    Ok((MjObjectType::Site, id))
                } else if let Some(&id) = self.body_name_to_id.get(name) {
                    Ok((MjObjectType::Body, id))
                } else if let Some(&id) = self.geom_name_to_id.get(name) {
                    Ok((MjObjectType::Geom, id))
                } else {
                    Err(ModelConversionError {
                        message: format!("frame sensor references unknown object '{name}'"),
                    })
                }
            }

            // Subtree sensors: objname is a body name
            MjSensorType::SubtreeCom
            | MjSensorType::SubtreeLinVel
            | MjSensorType::SubtreeAngMom => {
                let id = *self
                    .body_name_to_id
                    .get(name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!("sensor references unknown body '{name}'"),
                    })?;
                Ok((MjObjectType::Body, id))
            }

            // User handled above (early return)
            MjSensorType::User => unreachable!(),
        }
    }

    /// Process equality constraints from MJCF.
    ///
    /// Converts `MjcfEquality` (connects, welds, joints, distances) into Model arrays.
    /// Must be called AFTER all bodies and joints are processed (requires name lookups).
    ///
    /// # MuJoCo Semantics
    ///
    /// - **Connect**: Ball-and-socket constraint. Removes 3 translational DOFs.
    ///   `eq_data[0..3]` = anchor point in body1's local frame.
    ///
    /// - **Weld**: Fixed frame constraint. Removes 6 DOFs (translation + rotation).
    ///   `eq_data[0..3]` = anchor point, `eq_data[3..7]` = relative quaternion [w,x,y,z].
    ///
    /// - **Joint**: Polynomial coupling between joints.
    ///   `eq_data[0..5]` = polynomial coefficients (q2 = c0 + c1*q1 + c2*q1² + ...).
    ///
    /// - **Distance**: Maintains fixed distance between geom centers.
    ///   `eq_data[0]` = target distance.
    fn process_equality_constraints(
        &mut self,
        equality: &MjcfEquality,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process Connect constraints (3 DOF position constraint)
        for connect in &equality.connects {
            let body1_id =
                self.body_name_to_id
                    .get(&connect.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body1: '{}'",
                            connect.body1
                        ),
                    })?;

            // body2 defaults to world (body 0) if not specified
            let body2_id = if let Some(ref body2_name) = connect.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Connect constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Pack data: anchor point in body1 frame, rest zeroed
            let mut data = [0.0; 11];
            data[0] = connect.anchor.x;
            data[1] = connect.anchor.y;
            data[2] = connect.anchor.z;

            self.eq_type.push(EqualityType::Connect);
            self.eq_obj1id.push(*body1_id);
            self.eq_obj2id.push(body2_id);
            self.eq_data.push(data);
            self.eq_active.push(connect.active);
            self.eq_solimp
                .push(connect.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref
                .push(connect.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(connect.name.clone());
        }

        // Process Weld constraints (6 DOF pose constraint)
        for weld in &equality.welds {
            let body1_id =
                self.body_name_to_id
                    .get(&weld.body1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body1: '{}'",
                            weld.body1
                        ),
                    })?;

            let body2_id = if let Some(ref body2_name) = weld.body2 {
                *self
                    .body_name_to_id
                    .get(body2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Weld constraint references unknown body2: '{body2_name}'"
                        ),
                    })?
            } else {
                0 // World body
            };

            // Pack data: anchor point + relative pose quaternion
            // relpose format: [x, y, z, qw, qx, qy, qz]
            let mut data = [0.0; 11];
            data[0] = weld.anchor.x;
            data[1] = weld.anchor.y;
            data[2] = weld.anchor.z;
            if let Some(relpose) = weld.relpose {
                // relpose = [x, y, z, qw, qx, qy, qz]
                // We store anchor separately, so relpose goes to data[3..10]
                data[3] = relpose[3]; // qw
                data[4] = relpose[4]; // qx
                data[5] = relpose[5]; // qy
                data[6] = relpose[6]; // qz
                // Note: relpose position offset is in addition to anchor
                data[7] = relpose[0]; // dx
                data[8] = relpose[1]; // dy
                data[9] = relpose[2]; // dz
            } else {
                // Default: identity quaternion, no offset
                data[3] = 1.0; // qw = 1 (identity)
            }

            self.eq_type.push(EqualityType::Weld);
            self.eq_obj1id.push(*body1_id);
            self.eq_obj2id.push(body2_id);
            self.eq_data.push(data);
            self.eq_active.push(weld.active);
            self.eq_solimp.push(weld.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref.push(weld.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(weld.name.clone());
        }

        // Process Joint coupling constraints
        for joint_eq in &equality.joints {
            let joint1_id = self.joint_name_to_id.get(&joint_eq.joint1).ok_or_else(|| {
                ModelConversionError {
                    message: format!(
                        "Joint equality references unknown joint1: '{}'",
                        joint_eq.joint1
                    ),
                }
            })?;

            // joint2 is optional - if not specified, constraint locks joint1 to polycoef[0]
            let joint2_id = if let Some(ref joint2_name) = joint_eq.joint2 {
                *self
                    .joint_name_to_id
                    .get(joint2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Joint equality references unknown joint2: '{joint2_name}'"
                        ),
                    })?
            } else {
                // MuJoCo convention: joint2 = -1 means constraint locks joint1 to constant
                // We use usize::MAX as sentinel (will never be a valid joint ID)
                usize::MAX
            };

            // Pack polynomial coefficients (up to 5 terms)
            // MuJoCo: q2 = c0 + c1*q1 + c2*q1² + c3*q1³ + c4*q1⁴
            let mut data = [0.0; 11];
            for (i, &coef) in joint_eq.polycoef.iter().take(5).enumerate() {
                data[i] = coef;
            }

            self.eq_type.push(EqualityType::Joint);
            self.eq_obj1id.push(*joint1_id);
            self.eq_obj2id.push(joint2_id);
            self.eq_data.push(data);
            self.eq_active.push(joint_eq.active);
            self.eq_solimp
                .push(joint_eq.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref
                .push(joint_eq.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(joint_eq.name.clone());
        }

        // Process Distance constraints (1 DOF scalar distance between geom centers)
        for dist in &equality.distances {
            let geom1_id =
                *self
                    .geom_name_to_id
                    .get(&dist.geom1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Distance constraint references unknown geom1: '{}'",
                            dist.geom1
                        ),
                    })?;

            // geom2 defaults to usize::MAX sentinel (world origin) if not specified
            let geom2_id = if let Some(ref geom2_name) = dist.geom2 {
                *self
                    .geom_name_to_id
                    .get(geom2_name)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "Distance constraint references unknown geom2: '{geom2_name}'"
                        ),
                    })?
            } else {
                usize::MAX
            };

            let target_distance = if let Some(d) = dist.distance {
                d.max(0.0) // Distance is non-negative by definition
            } else {
                self.compute_initial_geom_distance(geom1_id, geom2_id)
            };

            let mut data = [0.0; 11];
            data[0] = target_distance;

            self.eq_type.push(EqualityType::Distance);
            self.eq_obj1id.push(geom1_id);
            self.eq_obj2id.push(geom2_id);
            self.eq_data.push(data);
            self.eq_active.push(dist.active);
            self.eq_solimp.push(dist.solimp.unwrap_or(DEFAULT_SOLIMP));
            self.eq_solref.push(dist.solref.unwrap_or(DEFAULT_SOLREF));
            self.eq_name.push(dist.name.clone());
        }

        Ok(())
    }

    /// Compute world-space distance between two geom centers at build time.
    ///
    /// Used when `distance: None` to derive target from initial configuration.
    /// `geom2_id == usize::MAX` means world origin.
    fn compute_initial_geom_distance(&self, geom1_id: usize, geom2_id: usize) -> f64 {
        let p1 = self.geom_world_position(geom1_id);
        let p2 = if geom2_id == usize::MAX {
            Vector3::zeros()
        } else {
            self.geom_world_position(geom2_id)
        };
        (p1 - p2).norm()
    }

    /// Compute the world-space position of a geom at build time.
    fn geom_world_position(&self, geom_id: usize) -> Vector3<f64> {
        let body_id = self.geom_body[geom_id];
        if body_id == 0 {
            self.geom_pos[geom_id]
        } else {
            let body_world_pos = self.body_world_pos[body_id - 1];
            let body_world_quat = self.body_world_quat[body_id - 1];
            body_world_pos + body_world_quat * self.geom_pos[geom_id]
        }
    }

    /// Apply the mass pipeline post-processing in MuJoCo order:
    ///   1. inertiafromgeom — already handled per-body in process_body
    ///   2. balanceinertia — triangle inequality correction
    ///   3. boundmass / boundinertia — minimum clamping
    ///   4. settotalmass — rescale to target total mass
    fn apply_mass_pipeline(&mut self) {
        let nbody = self.body_mass.len();

        // Step 2: balanceinertia (A7)
        // For each non-world body, check the triangle inequality on diagonal inertia.
        // If violated (A + B < C for any permutation), set all three to their mean.
        if self.compiler.balanceinertia {
            for i in 1..nbody {
                let inertia = &self.body_inertia[i];
                let a = inertia.x;
                let b = inertia.y;
                let c = inertia.z;
                if a + b < c || a + c < b || b + c < a {
                    let mean = (a + b + c) / 3.0;
                    self.body_inertia[i] = Vector3::new(mean, mean, mean);
                }
            }
        }

        // Step 3: boundmass / boundinertia (A6)
        // Clamp every non-world body's mass and inertia to compiler minimums.
        if self.compiler.boundmass > 0.0 {
            for i in 1..nbody {
                if self.body_mass[i] < self.compiler.boundmass {
                    self.body_mass[i] = self.compiler.boundmass;
                }
            }
        }
        if self.compiler.boundinertia > 0.0 {
            for i in 1..nbody {
                let inertia = &mut self.body_inertia[i];
                inertia.x = inertia.x.max(self.compiler.boundinertia);
                inertia.y = inertia.y.max(self.compiler.boundinertia);
                inertia.z = inertia.z.max(self.compiler.boundinertia);
            }
        }

        // Step 4: settotalmass (A8)
        // When positive, rescale all body masses and inertias so total == target.
        if self.compiler.settotalmass > 0.0 {
            let total_mass: f64 = (1..nbody).map(|i| self.body_mass[i]).sum();
            if total_mass > 0.0 {
                let scale = self.compiler.settotalmass / total_mass;
                for i in 1..nbody {
                    self.body_mass[i] *= scale;
                    self.body_inertia[i] *= scale;
                }
            }
        }
    }

    fn build(self) -> Model {
        let njnt = self.jnt_type.len();
        let nbody = self.body_parent.len();
        let ngeom = self.geom_type.len();
        let nsite = self.site_body.len();
        let nu = self.actuator_trntype.len();
        let ntendon = self.tendon_type.len();

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

            dof_body: self.dof_body,
            dof_jnt: self.dof_jnt,
            dof_parent: self.dof_parent,
            dof_armature: self.dof_armature,
            dof_damping: self.dof_damping,
            dof_frictionloss: self.dof_frictionloss,

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
            geom_margin: vec![0.0; ngeom], // Default margin
            geom_gap: vec![0.0; ngeom],    // Default gap
            geom_solimp: self.geom_solimp, // From parsed MJCF
            geom_solref: self.geom_solref, // From parsed MJCF
            geom_name: self.geom_name,
            // Pre-computed bounding radii (computed below after we have geom_type and geom_size)
            geom_rbound: vec![0.0; ngeom],
            // Mesh index for each geom (populated by process_geom)
            geom_mesh: self.geom_mesh,
            // Hfield index for each geom (populated by process_geom)
            geom_hfield: self.geom_hfield,
            // SDF index for each geom (always None from MJCF — programmatic only)
            geom_sdf: self.geom_sdf,

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

            // Tendons (populated by process_tendons)
            ntendon: self.tendon_type.len(),
            nwrap: self.wrap_type.len(),
            tendon_type: self.tendon_type,
            tendon_range: self.tendon_range,
            tendon_limited: self.tendon_limited,
            tendon_stiffness: self.tendon_stiffness,
            tendon_damping: self.tendon_damping,
            tendon_frictionloss: self.tendon_frictionloss,
            // tendon_treenum/tendon_tree computed below after tree enumeration
            tendon_treenum: vec![0; ntendon],
            tendon_tree: vec![usize::MAX; 2 * ntendon],
            tendon_lengthspring: self.tendon_lengthspring,
            tendon_length0: self.tendon_length0,
            tendon_num: self.tendon_num,
            tendon_adr: self.tendon_adr,
            tendon_name: self.tendon_name,
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
            default_eq_stiffness: self.default_eq_stiffness,
            default_eq_damping: self.default_eq_damping,
            max_constraint_vel: self.max_constraint_vel,
            max_constraint_angvel: self.max_constraint_angvel,
            friction_smoothing: self.friction_smoothing,
            cone: self.cone,
            stat_meaninertia: 1.0, // Computed post-build in Step 3
            ls_iterations: self.ls_iterations,
            ls_tolerance: self.ls_tolerance,
            noslip_iterations: self.noslip_iterations,
            noslip_tolerance: self.noslip_tolerance,
            disableflags: self.disableflags,
            enableflags: self.enableflags,
            #[cfg(feature = "deformable")]
            deformable_condim: 3,
            #[cfg(feature = "deformable")]
            deformable_solref: [0.02, 1.0],
            #[cfg(feature = "deformable")]
            deformable_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
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
                if tree_set.len() == 2 {
                    let mut iter = tree_set.iter();
                    if let (Some(&a), Some(&b)) = (iter.next(), iter.next()) {
                        model.tendon_tree[2 * t] = a;
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

            // §16.26.4: Deformable body policy guard. In the current architecture,
            // deformable bodies are registered at runtime (Data::register_deformable_body),
            // not at model construction time. Therefore, this guard cannot run here —
            // it will need to run at Data::register_deformable_body() time instead,
            // marking the body's tree as AutoNever. This is deferred until the
            // deformable feature uses tree-level sleeping.

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

// ============================================================================
// <general> actuator helpers
// ============================================================================

fn parse_gaintype(s: &str) -> std::result::Result<GainType, ModelConversionError> {
    match s {
        "fixed" => Ok(GainType::Fixed),
        "affine" => Ok(GainType::Affine),
        "muscle" => Ok(GainType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown gaintype '{s}' (valid: fixed, affine, muscle)"),
        }),
    }
}

fn parse_biastype(s: &str) -> std::result::Result<BiasType, ModelConversionError> {
    match s {
        "none" => Ok(BiasType::None),
        "affine" => Ok(BiasType::Affine),
        "muscle" => Ok(BiasType::Muscle),
        _ => Err(ModelConversionError {
            message: format!("unknown biastype '{s}' (valid: none, affine, muscle)"),
        }),
    }
}

fn parse_dyntype(s: &str) -> std::result::Result<ActuatorDynamics, ModelConversionError> {
    match s {
        "none" => Ok(ActuatorDynamics::None),
        "integrator" => Ok(ActuatorDynamics::Integrator),
        "filter" => Ok(ActuatorDynamics::Filter),
        "filterexact" => Ok(ActuatorDynamics::FilterExact),
        "muscle" => Ok(ActuatorDynamics::Muscle),
        _ => Err(ModelConversionError {
            message: format!(
                "unknown dyntype '{s}' (valid: none, integrator, filter, filterexact, muscle)"
            ),
        }),
    }
}

/// Convert a variable-length parsed float vector into a fixed-size array,
/// padding with the given default value. Truncates if input exceeds `N`.
fn floats_to_array<const N: usize>(input: &[f64], default: [f64; N]) -> [f64; N] {
    let mut out = default;
    for (i, &v) in input.iter().enumerate().take(N) {
        out[i] = v;
    }
    out
}

/// Convert MJCF quaternion (w, x, y, z) to `UnitQuaternion`.
fn quat_from_wxyz(q: nalgebra::Vector4<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]))
}

/// Convert euler angles (radians) to quaternion using a configurable rotation sequence.
///
/// Mirrors MuJoCo's `mju_euler2Quat` exactly:
/// - Each character in `seq` selects the rotation axis (x/y/z).
/// - **Lowercase** = intrinsic (body-fixed): **post-multiply** `q = q * R_i`.
/// - **Uppercase** = extrinsic (space-fixed): **pre-multiply** `q = R_i * q`.
///
/// Examples:
/// - `"xyz"` (default) → intrinsic XYZ: `q = Rx * Ry * Rz`
/// - `"XYZ"` → extrinsic XYZ: `q = Rz * Ry * Rx`
/// - `"XYz"` → extrinsic X, extrinsic Y, intrinsic z: `q = Ry * Rx * Rz`
fn euler_seq_to_quat(euler_rad: Vector3<f64>, seq: &str) -> UnitQuaternion<f64> {
    let mut q = UnitQuaternion::identity();
    for (i, ch) in seq.chars().enumerate() {
        let angle = euler_rad[i];
        let axis = match ch.to_ascii_lowercase() {
            'x' => Vector3::x_axis(),
            'y' => Vector3::y_axis(),
            // 'z' or validated-at-parse-time fallback
            _ => Vector3::z_axis(),
        };
        let r = UnitQuaternion::from_axis_angle(&axis, angle);
        if ch.is_ascii_lowercase() {
            // Intrinsic: post-multiply
            q *= r;
        } else {
            // Extrinsic: pre-multiply
            q = r * q;
        }
    }
    q
}

/// Unified orientation resolution.
///
/// Priority when multiple alternatives are present (MuJoCo errors on
/// multiple; we silently use the highest-priority one for robustness):
/// `euler` > `axisangle` > `xyaxes` > `zaxis` > `quat`.
fn resolve_orientation(
    quat: Vector4<f64>,
    euler: Option<Vector3<f64>>,
    axisangle: Option<Vector4<f64>>,
    xyaxes: Option<[f64; 6]>,
    zaxis: Option<Vector3<f64>>,
    compiler: &MjcfCompiler,
) -> UnitQuaternion<f64> {
    if let Some(euler) = euler {
        let euler_rad = if compiler.angle == AngleUnit::Degree {
            euler * (std::f64::consts::PI / 180.0)
        } else {
            euler
        };
        euler_seq_to_quat(euler_rad, &compiler.eulerseq)
    } else if let Some(aa) = axisangle {
        let axis = Vector3::new(aa.x, aa.y, aa.z);
        let angle = if compiler.angle == AngleUnit::Degree {
            aa.w * (std::f64::consts::PI / 180.0)
        } else {
            aa.w
        };
        let norm = axis.norm();
        if norm > 1e-10 {
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle)
        } else {
            UnitQuaternion::identity()
        }
    } else if let Some(xy) = xyaxes {
        // Gram-Schmidt orthogonalization (matches MuJoCo's ResolveOrientation).
        let mut x = Vector3::new(xy[0], xy[1], xy[2]);
        let x_norm = x.norm();
        if x_norm < 1e-10 {
            return UnitQuaternion::identity();
        }
        x /= x_norm;

        let mut y = Vector3::new(xy[3], xy[4], xy[5]);
        // Orthogonalize y against x
        y -= x * x.dot(&y);
        let y_norm = y.norm();
        if y_norm < 1e-10 {
            return UnitQuaternion::identity();
        }
        y /= y_norm;

        let z = x.cross(&y);
        // z is already unit-length since x, y are orthonormal

        let rot = Matrix3::from_columns(&[x, y, z]);
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(rot);
        UnitQuaternion::from_rotation_matrix(&rotation)
    } else if let Some(zdir) = zaxis {
        // Minimal rotation from (0,0,1) to given direction.
        // Matches MuJoCo's mjuu_z2quat.
        let zn = zdir.norm();
        if zn < 1e-10 {
            return UnitQuaternion::identity();
        }
        let zdir = zdir / zn;

        let default_z = Vector3::z();
        let mut axis = default_z.cross(&zdir);
        let s = axis.norm();

        if s < 1e-10 {
            // Parallel or anti-parallel
            axis = Vector3::x();
        } else {
            axis /= s;
        }

        let angle = s.atan2(zdir.z);
        let half = angle / 2.0;
        let w = half.cos();
        let xyz = axis * half.sin();
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, xyz.x, xyz.y, xyz.z))
    } else {
        quat_from_wxyz(quat)
    }
}

// =============================================================================
// Frame Expansion
// =============================================================================

/// Core SE(3) composition — direct translation of MuJoCo's `mjuu_frameaccumChild`.
///
/// Composes a parent frame transform into a child's pos/quat:
/// - `child_pos_new = frame_pos + frame_quat * child_pos_old`
/// - `child_quat_new = frame_quat * child_quat_old`
fn frame_accum_child(
    frame_pos: &Vector3<f64>,
    frame_quat: &UnitQuaternion<f64>,
    child_pos: &mut Vector3<f64>,
    child_quat: &mut UnitQuaternion<f64>,
) {
    *child_pos = *frame_pos + frame_quat.transform_vector(child_pos);
    *child_quat = *frame_quat * *child_quat;
}

/// Validate that all `childclass` references in the body tree point to defined
/// default classes. Must run BEFORE `expand_frames()` because frame expansion
/// dissolves frames (via `std::mem::take`), losing `frame.childclass` values.
///
/// MuJoCo rejects undefined childclass references at schema validation (S7).
fn validate_childclass_references(
    body: &MjcfBody,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError> {
    if let Some(ref cc) = body.childclass {
        if resolver.get_defaults(Some(cc.as_str())).is_none() {
            return Err(ModelConversionError {
                message: format!(
                    "childclass '{}' on body '{}' references undefined default class",
                    cc, body.name
                ),
            });
        }
    }
    for frame in &body.frames {
        validate_frame_childclass_refs(frame, resolver)?;
    }
    for child in &body.children {
        validate_childclass_references(child, resolver)?;
    }
    Ok(())
}

/// Validate childclass references on a frame and its nested contents.
fn validate_frame_childclass_refs(
    frame: &MjcfFrame,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError> {
    if let Some(ref cc) = frame.childclass {
        if resolver.get_defaults(Some(cc.as_str())).is_none() {
            return Err(ModelConversionError {
                message: format!(
                    "childclass '{}' on frame '{}' references undefined default class",
                    cc,
                    frame.name.as_deref().unwrap_or("<unnamed>")
                ),
            });
        }
    }
    for nested in &frame.frames {
        validate_frame_childclass_refs(nested, resolver)?;
    }
    for child_body in &frame.bodies {
        validate_childclass_references(child_body, resolver)?;
    }
    Ok(())
}

/// Recursively expand all `<frame>` elements in a body tree.
///
/// Frames are coordinate-system wrappers that compose their transform into
/// each child element and then disappear. This runs before `discardvisual`/`fusestatic`
/// (matching MuJoCo's order) and before `ModelBuilder` processes bodies.
fn expand_frames(body: &mut MjcfBody, compiler: &MjcfCompiler, parent_childclass: Option<&str>) {
    // Resolve effective childclass early and own it to avoid borrow conflicts
    let effective_owned: Option<String> = body
        .childclass
        .clone()
        .or_else(|| parent_childclass.map(|s| s.to_string()));
    let effective = effective_owned.as_deref();

    // Take frames out of the body to avoid borrow issues
    let frames = std::mem::take(&mut body.frames);

    for frame in &frames {
        expand_single_frame(
            body,
            frame,
            &Vector3::zeros(),
            &UnitQuaternion::identity(),
            compiler,
            effective,
        );
    }

    // Recursively expand frames in child bodies
    for child in &mut body.children {
        expand_frames(child, compiler, effective);
    }
}

/// Expand a single frame, composing its transform into children and lifting
/// them onto the parent body.
fn expand_single_frame(
    body: &mut MjcfBody,
    frame: &MjcfFrame,
    accumulated_pos: &Vector3<f64>,
    accumulated_quat: &UnitQuaternion<f64>,
    compiler: &MjcfCompiler,
    parent_childclass: Option<&str>,
) {
    // 1. Resolve the frame's own orientation
    let frame_quat = resolve_orientation(
        frame.quat,
        frame.euler,
        frame.axisangle,
        frame.xyaxes,
        frame.zaxis,
        compiler,
    );

    // 2. Compose with accumulated parent-frame transform
    let mut composed_pos = frame.pos;
    let mut composed_quat = frame_quat;
    frame_accum_child(
        accumulated_pos,
        accumulated_quat,
        &mut composed_pos,
        &mut composed_quat,
    );

    // 3. Determine effective childclass: frame's overrides parent's
    let effective = frame.childclass.as_deref().or(parent_childclass);

    // 4. Process child geoms
    for geom in &frame.geoms {
        let mut g = geom.clone();

        // Apply childclass if geom has no explicit class
        if g.class.is_none() {
            g.class = effective.map(|s| s.to_string());
        }

        if let Some(ref mut fromto) = g.fromto {
            // fromto geoms: transform both endpoints through the composed frame.
            // Leave pos/quat untouched — compute_fromto_pose() will derive them later.
            let from = Vector3::new(fromto[0], fromto[1], fromto[2]);
            let to = Vector3::new(fromto[3], fromto[4], fromto[5]);
            let from_new = composed_pos + composed_quat.transform_vector(&from);
            let to_new = composed_pos + composed_quat.transform_vector(&to);
            *fromto = [
                from_new.x, from_new.y, from_new.z, to_new.x, to_new.y, to_new.z,
            ];
        } else {
            // Non-fromto geoms: resolve orientation, compose with frame
            let geom_quat =
                resolve_orientation(g.quat, g.euler, g.axisangle, g.xyaxes, g.zaxis, compiler);
            let mut pos = g.pos;
            let mut quat = geom_quat;
            frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
            g.pos = pos;
            g.quat = quat_to_wxyz(&quat);
            // Clear alternative orientations (already resolved)
            g.euler = None;
            g.axisangle = None;
            g.xyaxes = None;
            g.zaxis = None;
        }

        body.geoms.push(g);
    }

    // 5. Process child sites
    for site in &frame.sites {
        let mut s = site.clone();

        if s.class.is_none() {
            s.class = effective.map(|s| s.to_string());
        }

        let site_quat =
            resolve_orientation(s.quat, s.euler, s.axisangle, s.xyaxes, s.zaxis, compiler);
        let mut pos = s.pos;
        let mut quat = site_quat;
        frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
        s.pos = pos;
        s.quat = quat_to_wxyz(&quat);
        s.euler = None;
        s.axisangle = None;
        s.xyaxes = None;
        s.zaxis = None;

        body.sites.push(s);
    }

    // 6. Process child bodies
    for child_body in &frame.bodies {
        let mut b = child_body.clone();

        // If body has no own childclass, inherit from frame's effective childclass
        if b.childclass.is_none() {
            b.childclass = effective.map(|s| s.to_string());
        }

        let body_quat = resolve_orientation(b.quat, b.euler, b.axisangle, None, None, compiler);
        let mut pos = b.pos;
        let mut quat = body_quat;
        frame_accum_child(&composed_pos, &composed_quat, &mut pos, &mut quat);
        b.pos = pos;
        b.quat = quat_to_wxyz(&quat);
        b.euler = None;
        b.axisangle = None;

        body.children.push(b);
    }

    // 7. Recurse into nested frames
    for nested in &frame.frames {
        expand_single_frame(
            body,
            nested,
            &composed_pos,
            &composed_quat,
            compiler,
            effective,
        );
    }
}

// =============================================================================
// Mesh File Loading
// =============================================================================

/// Asset type for path resolution.
#[allow(dead_code)] // Texture used when texture loading lands
enum AssetKind {
    /// Mesh or hfield file (uses `meshdir`).
    Mesh,
    /// Texture file (uses `texturedir`).
    Texture,
}

/// Resolve an asset file path using compiler path settings.
///
/// # Path Resolution Rules (matches MuJoCo exactly)
///
/// 1. If `strippath` is enabled, strip directory components first.
/// 2. If the filename is an absolute path → use it directly.
/// 3. If the type-specific dir (`meshdir` for Mesh, `texturedir` for Texture) is set:
///    - If that dir is absolute → `dir / filename`.
///    - Else → `model_file_dir / dir / filename`.
/// 4. Else if `assetdir` is set → same logic as above using `assetdir`.
/// 5. Else → `model_file_dir / filename`.
///
/// # Errors
///
/// Returns `ModelConversionError` if:
/// - Path is relative but no base path or directory is available
/// - Resolved path does not exist
fn resolve_asset_path(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    kind: AssetKind,
) -> std::result::Result<PathBuf, ModelConversionError> {
    // A9. strippath: strip directory components, keeping only the base filename.
    let file_name = if compiler.strippath {
        Path::new(file_path)
            .file_name()
            .map(|f| f.to_string_lossy().to_string())
            .unwrap_or_else(|| file_path.to_string())
    } else {
        file_path.to_string()
    };

    let path = Path::new(&file_name);

    // Absolute paths are used directly
    if path.is_absolute() {
        if !path.exists() {
            return Err(ModelConversionError {
                message: format!("asset file not found: '{}'", path.display()),
            });
        }
        return Ok(path.to_path_buf());
    }

    // A3. Type-specific directory takes precedence over assetdir.
    let type_dir = match kind {
        AssetKind::Mesh => compiler.meshdir.as_deref(),
        AssetKind::Texture => compiler.texturedir.as_deref(),
    };

    // Try: type-specific dir, then assetdir, then bare base_path
    let resolved = if let Some(dir) = type_dir.or(compiler.assetdir.as_deref()) {
        let dir_path = Path::new(dir);
        if dir_path.is_absolute() {
            dir_path.join(path)
        } else if let Some(base) = base_path {
            base.join(dir_path).join(path)
        } else {
            // No base path but relative dir — try dir/filename as-is
            dir_path.join(path)
        }
    } else if let Some(base) = base_path {
        base.join(path)
    } else {
        return Err(ModelConversionError {
            message: format!(
                "asset file '{file_path}' is relative but no base path provided (use load_model_from_file or pass base_path to model_from_mjcf)"
            ),
        });
    };

    if !resolved.exists() {
        return Err(ModelConversionError {
            message: format!(
                "asset file not found: '{}' (resolved to '{}')",
                file_path,
                resolved.display()
            ),
        });
    }

    Ok(resolved)
}

/// Load mesh data from a file and convert to `TriangleMeshData`.
///
/// Supports STL, OBJ, PLY, and 3MF formats (auto-detected from extension).
/// Scale is applied to all vertices during conversion.
///
/// # Arguments
///
/// * `file_path` - Path string from MJCF `<mesh file="..."/>` attribute
/// * `base_path` - Base directory for resolving relative paths
/// * `scale` - Scale factors [x, y, z] to apply to vertices
/// * `mesh_name` - Mesh name for error messages
///
/// # Errors
///
/// Returns `ModelConversionError` if:
/// - Path resolution fails (see [`resolve_mesh_path`])
/// - File format is unsupported or corrupt
/// - Mesh contains no vertices or faces
fn load_mesh_file(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    scale: Vector3<f64>,
    mesh_name: &str,
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // 1. Resolve path using compiler path settings
    let resolved_path = resolve_asset_path(file_path, base_path, compiler, AssetKind::Mesh)?;

    // 2. Load mesh via mesh-io
    let indexed_mesh = mesh_io::load_mesh(&resolved_path).map_err(|e| ModelConversionError {
        message: format!("mesh '{mesh_name}': failed to load '{file_path}': {e}"),
    })?;

    // 3. Validate non-empty
    if indexed_mesh.vertices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{mesh_name}': file '{file_path}' contains no vertices"),
        });
    }
    if indexed_mesh.faces.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{mesh_name}': file '{file_path}' contains no faces"),
        });
    }

    // 4. Convert vertices with scale applied
    let vertices: Vec<Point3<f64>> = indexed_mesh
        .vertices
        .iter()
        .map(|v| {
            Point3::new(
                v.position.x * scale.x,
                v.position.y * scale.y,
                v.position.z * scale.z,
            )
        })
        .collect();

    // 5. Convert faces to flat indices
    //    mesh_io uses [u32; 3] per face, we need Vec<usize>
    let indices: Vec<usize> = indexed_mesh
        .faces
        .iter()
        .flat_map(|f| [f[0] as usize, f[1] as usize, f[2] as usize])
        .collect();

    // 6. Build TriangleMeshData (BVH constructed automatically)
    Ok(TriangleMeshData::new(vertices, indices))
}

/// Convert an MJCF hfield asset to `HeightFieldData`.
///
/// MuJoCo elevation formula: `vertex_z = elevation[i] × size[2]`.
/// MuJoCo vertex positions: centered at origin, x ∈ `[−size[0], +size[0]]`, y ∈ `[−size[1], +size[1]]`.
/// `HeightFieldData` uses corner-origin `(0,0)` with uniform `cell_size`.
#[allow(
    clippy::cast_precision_loss,   // nrow/ncol are small ints, well within f64 mantissa
    clippy::cast_possible_truncation, // f64→usize for grid indices after bounds checks
    clippy::cast_sign_loss,        // values are guaranteed non-negative in this context
)]
fn convert_mjcf_hfield(
    hfield: &MjcfHfield,
) -> std::result::Result<HeightFieldData, ModelConversionError> {
    let ncol = hfield.ncol;
    let nrow = hfield.nrow;
    let z_top = hfield.size[2];

    // Compute cell spacings
    let dx = 2.0 * hfield.size[0] / (ncol as f64 - 1.0);
    let dy = 2.0 * hfield.size[1] / (nrow as f64 - 1.0);

    // HeightFieldData requires uniform cell_size (square cells)
    let (cell_size, heights) = if (dx - dy).abs() / dx.max(dy) < 0.01 {
        // Within 1% tolerance: use average
        let cell_size = f64::midpoint(dx, dy);
        let heights: Vec<f64> = hfield.elevation.iter().map(|&e| e * z_top).collect();
        (cell_size, heights)
    } else {
        // Non-square: resample to finer resolution
        warn!(
            hfield_name = %hfield.name,
            dx, dy,
            "hfield has non-square cells (dx={dx:.4}, dy={dy:.4}), resampling to uniform grid"
        );
        let cell_size = dx.min(dy);
        let new_ncol = ((2.0 * hfield.size[0]) / cell_size).round() as usize + 1;
        let new_nrow = ((2.0 * hfield.size[1]) / cell_size).round() as usize + 1;

        let mut heights = Vec::with_capacity(new_nrow * new_ncol);
        for row in 0..new_nrow {
            let fy = row as f64 / (new_nrow as f64 - 1.0) * (nrow as f64 - 1.0);
            let iy = (fy as usize).min(nrow - 2);
            let ty = fy - iy as f64;
            for col in 0..new_ncol {
                let fx = col as f64 / (new_ncol as f64 - 1.0) * (ncol as f64 - 1.0);
                let ix = (fx as usize).min(ncol - 2);
                let tx = fx - ix as f64;
                // Bilinear interpolation
                let e00 = hfield.elevation[iy * ncol + ix];
                let e10 = hfield.elevation[iy * ncol + ix + 1];
                let e01 = hfield.elevation[(iy + 1) * ncol + ix];
                let e11 = hfield.elevation[(iy + 1) * ncol + ix + 1];
                let e = e00 * (1.0 - tx) * (1.0 - ty)
                    + e10 * tx * (1.0 - ty)
                    + e01 * (1.0 - tx) * ty
                    + e11 * tx * ty;
                heights.push(e * z_top);
            }
        }
        return Ok(HeightFieldData::new(heights, new_ncol, new_nrow, cell_size));
    };

    Ok(HeightFieldData::new(heights, ncol, nrow, cell_size))
}

/// Convert MJCF mesh asset to `TriangleMeshData`.
///
/// Handles two sources of mesh data:
/// 1. **Embedded data**: `vertex` and `face` attributes in MJCF
/// 2. **File-based**: `file` attribute pointing to STL/OBJ/PLY/3MF
///
/// Scale is applied to all vertices. BVH is built automatically.
fn convert_mjcf_mesh(
    mjcf_mesh: &MjcfMesh,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // Dispatch based on data source
    match (&mjcf_mesh.vertex, &mjcf_mesh.file) {
        // Embedded vertex data takes precedence (MuJoCo semantics)
        (Some(verts), _) => convert_embedded_mesh(mjcf_mesh, verts),
        // File-based loading
        (None, Some(file_path)) => load_mesh_file(
            file_path,
            base_path,
            compiler,
            mjcf_mesh.scale,
            &mjcf_mesh.name,
        ),
        // Neither specified
        (None, None) => Err(ModelConversionError {
            message: format!(
                "mesh '{}': no vertex data and no file specified",
                mjcf_mesh.name
            ),
        }),
    }
}

/// Convert embedded mesh data from MJCF to `TriangleMeshData`.
///
/// This handles the case where vertex and face data are embedded directly
/// in the MJCF XML via `vertex="..."` and `face="..."` attributes.
fn convert_embedded_mesh(
    mjcf_mesh: &MjcfMesh,
    verts: &[f64],
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // Validate vertex data
    if verts.len() % 3 != 0 {
        return Err(ModelConversionError {
            message: format!(
                "mesh '{}': vertex count ({}) not divisible by 3",
                mjcf_mesh.name,
                verts.len()
            ),
        });
    }

    // Apply scale while converting to Point3
    let vertices: Vec<Point3<f64>> = verts
        .chunks_exact(3)
        .map(|chunk| {
            Point3::new(
                chunk[0] * mjcf_mesh.scale.x,
                chunk[1] * mjcf_mesh.scale.y,
                chunk[2] * mjcf_mesh.scale.z,
            )
        })
        .collect();

    // Extract and validate faces (convert u32 -> usize)
    let indices: Vec<usize> = match &mjcf_mesh.face {
        Some(face_data) => {
            if face_data.len() % 3 != 0 {
                return Err(ModelConversionError {
                    message: format!(
                        "mesh '{}': face index count ({}) not divisible by 3",
                        mjcf_mesh.name,
                        face_data.len()
                    ),
                });
            }
            face_data.iter().map(|&idx| idx as usize).collect()
        }
        None => {
            return Err(ModelConversionError {
                message: format!(
                    "mesh '{}': embedded vertex data requires face data",
                    mjcf_mesh.name
                ),
            });
        }
    };

    // Validate non-empty
    if vertices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{}': empty vertex array", mjcf_mesh.name),
        });
    }
    if indices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{}': empty face array", mjcf_mesh.name),
        });
    }

    // Validate index bounds
    let max_idx = vertices.len();
    for (i, &idx) in indices.iter().enumerate() {
        if idx >= max_idx {
            return Err(ModelConversionError {
                message: format!(
                    "mesh '{}': face index {} at position {} out of bounds (max: {})",
                    mjcf_mesh.name,
                    idx,
                    i,
                    max_idx - 1
                ),
            });
        }
    }

    // TriangleMeshData::new() builds the BVH automatically
    Ok(TriangleMeshData::new(vertices, indices))
}

/// Compute exact mass properties of a triangle mesh using signed tetrahedron
/// decomposition (Mirtich 1996).
///
/// Returns `(volume, com, inertia_at_com)` where:
/// - `volume` is the signed volume (positive for outward-facing normals)
/// - `com` is the center of mass (assuming uniform density)
/// - `inertia_at_com` is the full 3×3 inertia tensor about the COM
///   (assuming unit density; multiply by actual density for physical values)
#[allow(clippy::suspicious_operation_groupings)] // Formulas are correct: a²+b²+c²+ab+ac+bc
fn compute_mesh_inertia(mesh: &TriangleMeshData) -> (f64, Vector3<f64>, Matrix3<f64>) {
    let vertices = mesh.vertices();
    let triangles = mesh.triangles();

    let mut total_volume = 0.0;
    let mut com_accum = Vector3::zeros();

    // Second-moment integrals (products of vertex coordinates over volume)
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut zz = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;

    for tri in triangles {
        let a = vertices[tri.v0].coords;
        let b = vertices[tri.v1].coords;
        let c = vertices[tri.v2].coords;

        // Signed volume of tetrahedron formed with origin: V = (a × b) · c / 6
        let det = a.cross(&b).dot(&c);
        let vol = det / 6.0;
        total_volume += vol;

        // COM contribution: centroid of tet = (a + b + c) / 4, weighted by vol
        com_accum += vol * (a + b + c) / 4.0;

        // Second-moment integrals over tetrahedron (origin, a, b, c):
        // ∫x² dV = det/60 * (a.x² + b.x² + c.x² + a.x*b.x + a.x*c.x + b.x*c.x)
        // ∫xy dV = det/120 * (2*a.x*a.y + 2*b.x*b.y + 2*c.x*c.y
        //          + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x)
        let f60 = det / 60.0;
        let f120 = det / 120.0;

        xx += f60 * (a.x * a.x + b.x * b.x + c.x * c.x + a.x * b.x + a.x * c.x + b.x * c.x);
        yy += f60 * (a.y * a.y + b.y * b.y + c.y * c.y + a.y * b.y + a.y * c.y + b.y * c.y);
        zz += f60 * (a.z * a.z + b.z * b.z + c.z * c.z + a.z * b.z + a.z * c.z + b.z * c.z);

        xy += f120
            * (2.0 * a.x * a.y
                + 2.0 * b.x * b.y
                + 2.0 * c.x * c.y
                + a.x * b.y
                + a.y * b.x
                + a.x * c.y
                + a.y * c.x
                + b.x * c.y
                + b.y * c.x);
        xz += f120
            * (2.0 * a.x * a.z
                + 2.0 * b.x * b.z
                + 2.0 * c.x * c.z
                + a.x * b.z
                + a.z * b.x
                + a.x * c.z
                + a.z * c.x
                + b.x * c.z
                + b.z * c.x);
        yz += f120
            * (2.0 * a.y * a.z
                + 2.0 * b.y * b.z
                + 2.0 * c.y * c.z
                + a.y * b.z
                + a.z * b.y
                + a.y * c.z
                + a.z * c.y
                + b.y * c.z
                + b.z * c.y);
    }

    // Zero-volume fallback: degenerate mesh (coplanar triangles, etc.)
    if total_volume.abs() < 1e-10 {
        let (aabb_min, aabb_max) = mesh.aabb();
        let extents = aabb_max - aabb_min;
        let volume = extents.x * extents.y * extents.z;
        let com = nalgebra::center(&aabb_min, &aabb_max).coords;
        // Box inertia (unit density): I_ii = V/12 * (a² + b²)
        let c = volume / 12.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(
            c * (extents.y.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.y.powi(2)),
        ));
        return (volume, com, inertia);
    }

    let com = com_accum / total_volume;

    // Build inertia tensor at origin from accumulated integrals
    // I_origin[i,i] = sum of the other two second moments (e.g., Ixx = yy + zz)
    // I_origin[i,j] = -cross_moment (e.g., Ixy = -xy)
    let i_origin = Matrix3::new(
        yy + zz,
        -xy,
        -xz, // row 0
        -xy,
        xx + zz,
        -yz, // row 1
        -xz,
        -yz,
        xx + yy, // row 2
    );

    // Shift to COM using parallel axis theorem (full tensor):
    // I_com = I_origin - V * (d·d * I₃ - d ⊗ d)
    // where d = com and V = total_volume (unit density, so mass = volume)
    let d = com;
    let d_sq = d.dot(&d);
    let parallel_shift = total_volume * (Matrix3::identity() * d_sq - d * d.transpose());
    let i_com = i_origin - parallel_shift;

    (total_volume, com, i_com)
}

/// Extract inertial properties from MjcfInertial with full MuJoCo semantics.
///
/// Handles both `diaginertia` and `fullinertia` specifications.
/// When `fullinertia` is specified, diagonalizes via eigendecomposition
/// and returns the principal axis orientation in `iquat`.
fn extract_inertial_properties(
    inertial: &MjcfInertial,
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    let mass = inertial.mass;
    let ipos = inertial.pos;

    // Priority: fullinertia > diaginertia > default
    if let Some(full) = inertial.fullinertia {
        // Full inertia tensor [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        // Build symmetric matrix and diagonalize
        let inertia_matrix = Matrix3::new(
            full[0], full[3], full[4], // Ixx, Ixy, Ixz
            full[3], full[1], full[5], // Ixy, Iyy, Iyz
            full[4], full[5], full[2], // Ixz, Iyz, Izz
        );

        // Eigendecomposition to get principal axes
        let eigen = inertia_matrix.symmetric_eigen();
        let principal_inertia = Vector3::new(
            eigen.eigenvalues[0].abs(),
            eigen.eigenvalues[1].abs(),
            eigen.eigenvalues[2].abs(),
        );

        // Eigenvectors form rotation matrix to principal axes
        // Ensure right-handed coordinate system
        let mut rot = eigen.eigenvectors;
        if rot.determinant() < 0.0 {
            // Flip one column to make it right-handed
            rot.set_column(2, &(-rot.column(2)));
        }

        let iquat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&rot));

        // Combine with any existing inertial frame orientation
        let base_iquat = quat_from_wxyz(inertial.quat);
        let final_iquat = base_iquat * iquat;

        (mass, principal_inertia, ipos, final_iquat)
    } else if let Some(diag) = inertial.diaginertia {
        // Diagonal inertia already in principal axes
        (mass, diag, ipos, quat_from_wxyz(inertial.quat))
    } else {
        // Default: small uniform inertia
        (
            mass,
            Vector3::new(0.001, 0.001, 0.001),
            ipos,
            quat_from_wxyz(inertial.quat),
        )
    }
}

/// Apply `discardvisual` pre-processing: remove visual-only geoms (contype=0, conaffinity=0)
/// and unreferenced mesh assets. Geoms referenced by sensors or actuators are protected.
fn apply_discardvisual(mjcf: &mut MjcfModel) {
    // Collect geom names referenced by sensors (site-based sensors reference sites, not geoms,
    // but frame sensors can resolve to geoms) and actuators (no geom refs currently, but future-proof).
    let mut protected_geoms: HashSet<String> = HashSet::new();
    for sensor in &mjcf.sensors {
        // Frame sensors can resolve objname as a geom; protect any objname that might be a geom.
        if let Some(ref name) = sensor.objname {
            protected_geoms.insert(name.clone());
        }
    }

    fn remove_visual_geoms(body: &mut MjcfBody, protected: &HashSet<String>) {
        body.geoms.retain(|g| {
            g.contype != 0
                || g.conaffinity != 0
                || g.name.as_ref().is_some_and(|n| protected.contains(n))
        });
        for child in &mut body.children {
            remove_visual_geoms(child, protected);
        }
    }

    remove_visual_geoms(&mut mjcf.worldbody, &protected_geoms);

    // Collect all mesh names still referenced
    fn collect_mesh_refs(body: &MjcfBody, refs: &mut HashSet<String>) {
        for g in &body.geoms {
            if let Some(ref m) = g.mesh {
                refs.insert(m.clone());
            }
        }
        for child in &body.children {
            collect_mesh_refs(child, refs);
        }
    }
    let mut used_meshes = HashSet::new();
    collect_mesh_refs(&mjcf.worldbody, &mut used_meshes);

    // Remove unreferenced meshes
    mjcf.meshes.retain(|m| used_meshes.contains(&m.name));
}

/// Apply `fusestatic` pre-processing: fuse jointless bodies into their parent.
///
/// Bodies without joints are "static" relative to their parent. Fusing them
/// transfers their geoms, sites, and children to the parent, reducing the body
/// count. Bodies referenced by equality constraints, sensors, or actuators are
/// not fused (their names must remain valid).
fn apply_fusestatic(mjcf: &mut MjcfModel) {
    // Collect names of bodies referenced by constraints, sensors, actuators, skins
    let mut protected: HashSet<String> = HashSet::new();

    // Equality constraints
    for c in &mjcf.equality.connects {
        protected.insert(c.body1.clone());
        if let Some(ref b2) = c.body2 {
            protected.insert(b2.clone());
        }
    }
    for w in &mjcf.equality.welds {
        protected.insert(w.body1.clone());
        if let Some(ref b2) = w.body2 {
            protected.insert(b2.clone());
        }
    }

    // Sensors that reference bodies: subtree sensors always reference body names,
    // frame sensors can resolve to bodies. Protect objnames for both.
    for sensor in &mjcf.sensors {
        use crate::types::MjcfSensorType;
        match sensor.sensor_type {
            MjcfSensorType::Subtreecom
            | MjcfSensorType::Subtreelinvel
            | MjcfSensorType::Subtreeangmom
            | MjcfSensorType::Framepos
            | MjcfSensorType::Framequat
            | MjcfSensorType::Framexaxis
            | MjcfSensorType::Frameyaxis
            | MjcfSensorType::Framezaxis
            | MjcfSensorType::Framelinvel
            | MjcfSensorType::Frameangvel
            | MjcfSensorType::Framelinacc
            | MjcfSensorType::Frameangacc => {
                if let Some(ref name) = sensor.objname {
                    protected.insert(name.clone());
                }
            }
            _ => {}
        }
    }

    // Actuators with body transmission (adhesion actuators)
    for actuator in &mjcf.actuators {
        if let Some(ref body_name) = actuator.body {
            protected.insert(body_name.clone());
        }
    }

    // Skin bones reference body names
    for skin in &mjcf.skins {
        for bone in &skin.bones {
            protected.insert(bone.body.clone());
        }
    }

    // Recursively fuse static children in the worldbody tree.
    fuse_static_body(&mut mjcf.worldbody, &protected, &mjcf.compiler);
}

/// Convert a `UnitQuaternion` back to MJCF `Vector4<f64>` (w, x, y, z order).
fn quat_to_wxyz(q: &UnitQuaternion<f64>) -> Vector4<f64> {
    let qi = q.into_inner();
    Vector4::new(qi.w, qi.i, qi.j, qi.k)
}

/// Recursively fuse jointless, unprotected children into their parent body.
/// Operates on a parent body: scans its children, and for any that are static
/// (no joints, not protected), transfers their geoms, sites, and grandchildren
/// to the parent, adjusting positions and orientations by the fused body's frame.
fn fuse_static_body(parent: &mut MjcfBody, protected: &HashSet<String>, compiler: &MjcfCompiler) {
    // Depth-first: fuse within each child first
    for child in &mut parent.children {
        fuse_static_body(child, protected, compiler);
    }

    // Now fuse static children into this parent
    let mut i = 0;
    while i < parent.children.len() {
        let is_static =
            parent.children[i].joints.is_empty() && !protected.contains(&parent.children[i].name);

        if is_static {
            let mut removed = parent.children.remove(i);
            let body_pos = removed.pos;
            let body_quat = resolve_orientation(
                removed.quat,
                removed.euler,
                removed.axisangle,
                None,
                None,
                compiler,
            );
            let has_rotation = body_quat.angle() > 1e-10;

            // Transform geoms into parent frame
            for g in &mut removed.geoms {
                if has_rotation {
                    g.pos = body_quat * g.pos + body_pos;
                    let geom_quat = resolve_orientation(
                        g.quat,
                        g.euler,
                        g.axisangle,
                        g.xyaxes,
                        g.zaxis,
                        compiler,
                    );
                    let composed = body_quat * geom_quat;
                    g.quat = quat_to_wxyz(&composed);
                    // Normalize to quat-only after composition
                    g.euler = None;
                    g.axisangle = None;
                    g.xyaxes = None;
                    g.zaxis = None;
                } else {
                    g.pos += body_pos;
                }
            }

            // Transform sites into parent frame
            for s in &mut removed.sites {
                if has_rotation {
                    s.pos = body_quat * s.pos + body_pos;
                    let site_quat = resolve_orientation(
                        s.quat,
                        s.euler,
                        s.axisangle,
                        s.xyaxes,
                        s.zaxis,
                        compiler,
                    );
                    let composed = body_quat * site_quat;
                    s.quat = quat_to_wxyz(&composed);
                    // Normalize to quat-only after composition
                    s.euler = None;
                    s.axisangle = None;
                    s.xyaxes = None;
                    s.zaxis = None;
                } else {
                    s.pos += body_pos;
                }
            }

            // Transform child body frames into parent frame
            for c in &mut removed.children {
                if has_rotation {
                    c.pos = body_quat * c.pos + body_pos;
                    let child_quat =
                        resolve_orientation(c.quat, c.euler, c.axisangle, None, None, compiler);
                    let composed = body_quat * child_quat;
                    c.quat = quat_to_wxyz(&composed);
                    c.euler = None;
                    c.axisangle = None;
                } else {
                    c.pos += body_pos;
                }
            }

            // Transfer geoms and sites to parent
            parent.geoms.extend(removed.geoms);
            parent.sites.extend(removed.sites);

            // Splice grandchildren at position i
            for (j, gc) in removed.children.into_iter().enumerate() {
                parent.children.insert(i + j, gc);
            }
            // Don't increment i — re-check from the same position
        } else {
            i += 1;
        }
    }
}

/// Resolve mesh data for a geom, if it is a mesh-type geom.
fn resolve_mesh(
    geom: &MjcfGeom,
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> Option<Arc<TriangleMeshData>> {
    if geom.geom_type == MjcfGeomType::Mesh {
        geom.mesh
            .as_ref()
            .and_then(|name| mesh_lookup.get(name))
            .and_then(|&id| mesh_data.get(id))
            .cloned()
    } else {
        None
    }
}

/// Compute inertia from geoms (fallback when no explicit inertial).
///
/// Accumulates full 3×3 inertia tensor with geom orientation handling,
/// then eigendecomposes to extract principal inertia and orientation.
fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    if geoms.is_empty() {
        // No geoms: zero mass/inertia (matches MuJoCo).
        // Use boundmass/boundinertia to set minimums instead.
        return (
            0.0,
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
    }

    let mut total_mass = 0.0;
    let mut com = Vector3::zeros();

    // First pass: compute total mass and COM
    for geom in geoms {
        let mesh = resolve_mesh(geom, mesh_lookup, mesh_data);
        let geom_mass = compute_geom_mass(geom, mesh.as_deref());
        total_mass += geom_mass;
        com += geom.pos * geom_mass;
    }

    if total_mass > 1e-10 {
        com /= total_mass;
    }

    // Second pass: accumulate full 3×3 inertia tensor about COM
    let mut inertia_tensor = Matrix3::zeros();
    for geom in geoms {
        let mesh = resolve_mesh(geom, mesh_lookup, mesh_data);
        let geom_mass = compute_geom_mass(geom, mesh.as_deref());
        let geom_inertia = compute_geom_inertia(geom, mesh.as_deref());

        // 1. Rotate local inertia to body frame: I_rot = R * I_local * Rᵀ
        let r = UnitQuaternion::from_quaternion(Quaternion::new(
            geom.quat[0],
            geom.quat[1],
            geom.quat[2],
            geom.quat[3],
        ));
        let rot = r.to_rotation_matrix();
        let i_rotated = rot * geom_inertia * rot.transpose();

        // 2. Parallel axis theorem (full tensor):
        //    I_shifted = I_rotated + m * (d·d * I₃ - d ⊗ d)
        let d = geom.pos - com;
        let d_sq = d.dot(&d);
        let parallel_axis = geom_mass * (Matrix3::identity() * d_sq - d * d.transpose());
        inertia_tensor += i_rotated + parallel_axis;
    }

    // Eigendecompose to get principal axes
    let eigen = inertia_tensor.symmetric_eigen();
    let principal_inertia = Vector3::new(
        eigen.eigenvalues[0].abs(),
        eigen.eigenvalues[1].abs(),
        eigen.eigenvalues[2].abs(),
    );

    // Eigenvectors form rotation to principal axes
    // Ensure right-handed coordinate system
    let mut rot = eigen.eigenvectors;
    if rot.determinant() < 0.0 {
        rot.set_column(2, &(-rot.column(2)));
    }
    let iquat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&rot));

    (total_mass, principal_inertia, com, iquat)
}

/// Compute mass of a single geom.
fn compute_geom_mass(geom: &MjcfGeom, mesh_data: Option<&TriangleMeshData>) -> f64 {
    if let Some(mass) = geom.mass {
        return mass;
    }

    let volume = match geom.geom_type {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
        }
        MjcfGeomType::Box => {
            let x = geom.size.first().copied().unwrap_or(0.1);
            let y = geom.size.get(1).copied().unwrap_or(x);
            let z = geom.size.get(2).copied().unwrap_or(y);
            8.0 * x * y * z // size is half-extents
        }
        MjcfGeomType::Capsule => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * (h * 2.0 + (4.0 / 3.0) * r)
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * h * 2.0
        }
        MjcfGeomType::Mesh => {
            if let Some(mesh) = mesh_data {
                let (volume, _, _) = compute_mesh_inertia(mesh);
                volume.abs()
            } else {
                0.001
            }
        }
        _ => 0.001, // Default small volume for other types (Plane, Hfield)
    };

    geom.density * volume
}

/// Compute inertia tensor of a single geom about its center (geom-local frame).
///
/// Returns a full 3×3 matrix. Primitive geoms produce diagonal tensors;
/// mesh geoms may have off-diagonal terms.
///
/// Uses exact formulas matching MuJoCo's computation:
/// - Sphere: I = (2/5) m r²
/// - Box: I_x = (1/12) m (y² + z²), etc.
/// - Cylinder: I_x = (1/12) m (3r² + h²), I_z = (1/2) m r²
/// - Capsule: Exact formula including hemispherical end caps
/// - Mesh: Signed tetrahedron decomposition (Mirtich 1996)
fn compute_geom_inertia(geom: &MjcfGeom, mesh_data: Option<&TriangleMeshData>) -> Matrix3<f64> {
    let mass = compute_geom_mass(geom, mesh_data);

    match geom.geom_type {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let i = 0.4 * mass * r.powi(2); // (2/5) m r²
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box => {
            // Full dimensions (size is half-extents)
            let x = geom.size.first().copied().unwrap_or(0.1) * 2.0;
            let y = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            let z = geom.size.get(2).copied().unwrap_or(0.1) * 2.0;
            let c = mass / 12.0;
            Matrix3::from_diagonal(&Vector3::new(
                c * (y * y + z * z),
                c * (x * x + z * z),
                c * (x * x + y * y),
            ))
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Full height
            // Solid cylinder about center
            let ix = mass * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let iz = 0.5 * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Capsule => {
            // Exact capsule inertia (cylinder + two hemispheres)
            // Reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Cylinder height (not including caps)

            // Volume components
            let v_cyl = std::f64::consts::PI * r.powi(2) * h;
            let v_sphere = (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
            let v_total = v_cyl + v_sphere;

            // Mass components (assuming uniform density)
            let m_cyl = mass * v_cyl / v_total;
            let m_hemi = mass * v_sphere / (2.0 * v_total); // Each hemisphere

            // Cylinder inertia about its own center
            let i_cyl_x = m_cyl * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let i_cyl_z = 0.5 * m_cyl * r.powi(2);

            // Hemisphere inertia about its own center (half of sphere)
            let i_hemi_own = 0.4 * m_hemi * r.powi(2);

            // Distance from capsule center to hemisphere center
            let d = h / 2.0 + (3.0 / 8.0) * r; // Center of hemisphere from cylinder end

            // Parallel axis theorem for hemispheres
            let i_hemi_x = i_hemi_own + m_hemi * d.powi(2);
            let i_hemi_z = i_hemi_own; // No parallel axis for z (axial)

            // Total: cylinder + 2 hemispheres
            let ix = i_cyl_x + 2.0 * i_hemi_x;
            let iz = i_cyl_z + 2.0 * i_hemi_z;

            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Ellipsoid => {
            // Ellipsoid inertia: I_x = (1/5) m (b² + c²), etc.
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            let coeff = mass / 5.0;
            Matrix3::from_diagonal(&Vector3::new(
                coeff * (b * b + c * c),
                coeff * (a * a + c * c),
                coeff * (a * a + b * b),
            ))
        }
        MjcfGeomType::Mesh => {
            if let Some(mesh) = mesh_data {
                let (volume, _, inertia_unit) = compute_mesh_inertia(mesh);
                let mass_actual = geom.mass.unwrap_or_else(|| geom.density * volume.abs());
                let scale = if volume.abs() > 1e-10 {
                    mass_actual / volume.abs()
                } else {
                    geom.density
                };
                inertia_unit * scale
            } else {
                Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001))
            }
        }
        _ => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)), // Default small inertia
    }
}

/// Compute pose from fromto specification.
fn compute_fromto_pose(
    fromto: [f64; 6],
    size: &[f64],
) -> (Vector3<f64>, UnitQuaternion<f64>, Vector3<f64>) {
    let from = Vector3::new(fromto[0], fromto[1], fromto[2]);
    let to = Vector3::new(fromto[3], fromto[4], fromto[5]);

    let center = (from + to) / 2.0;
    let axis = to - from;
    let half_length = axis.norm() / 2.0;
    let radius = size.first().copied().unwrap_or(0.05);

    // Compute rotation to align Z with axis
    let quat = if axis.norm() > 1e-10 {
        let axis_normalized = axis.normalize();
        let z_axis = Vector3::z();

        if (axis_normalized - z_axis).norm() < 1e-10 {
            UnitQuaternion::identity()
        } else if (axis_normalized + z_axis).norm() < 1e-10 {
            UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::x()),
                std::f64::consts::PI,
            )
        } else {
            let rot_axis = z_axis.cross(&axis_normalized);
            // Clamp to avoid NaN from floating-point precision issues
            let angle = z_axis.dot(&axis_normalized).clamp(-1.0, 1.0).acos();
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(rot_axis), angle)
        }
    } else {
        UnitQuaternion::identity()
    };

    // Convention: size.x = radius, size.y = half_length, size.z = 0.0
    // This matches geom_size_to_vec3 for Capsule/Cylinder.
    (center, quat, Vector3::new(radius, half_length, 0.0))
}

/// Convert geom size array to `Vector3`.
fn geom_size_to_vec3(size: &[f64], geom_type: GeomType) -> Vector3<f64> {
    match geom_type {
        GeomType::Sphere => {
            let r = size.first().copied().unwrap_or(0.1);
            Vector3::new(r, r, r)
        }
        GeomType::Box => {
            let x = size.first().copied().unwrap_or(0.1);
            let y = size.get(1).copied().unwrap_or(x);
            let z = size.get(2).copied().unwrap_or(y);
            Vector3::new(x, y, z)
        }
        GeomType::Capsule | GeomType::Cylinder => {
            // Convention: size.x = radius, size.y = half_length, size.z unused
            // This matches the collision code expectations.
            let r = size.first().copied().unwrap_or(0.1);
            let h = size.get(1).copied().unwrap_or(0.1);
            Vector3::new(r, h, 0.0)
        }
        GeomType::Ellipsoid => {
            // Ellipsoid radii (rx, ry, rz)
            let rx = size.first().copied().unwrap_or(0.1);
            let ry = size.get(1).copied().unwrap_or(rx);
            let rz = size.get(2).copied().unwrap_or(ry);
            Vector3::new(rx, ry, rz)
        }
        _ => Vector3::new(0.1, 0.1, 0.1),
    }
}

/// Map `MjcfSensorType` to pipeline `MjSensorType`.
/// Returns `None` for types not yet implemented in the pipeline.
fn convert_sensor_type(mjcf: MjcfSensorType) -> Option<MjSensorType> {
    match mjcf {
        MjcfSensorType::Jointpos => Some(MjSensorType::JointPos),
        MjcfSensorType::Jointvel => Some(MjSensorType::JointVel),
        MjcfSensorType::Ballquat => Some(MjSensorType::BallQuat),
        MjcfSensorType::Ballangvel => Some(MjSensorType::BallAngVel),
        MjcfSensorType::Tendonpos => Some(MjSensorType::TendonPos),
        MjcfSensorType::Tendonvel => Some(MjSensorType::TendonVel),
        MjcfSensorType::Actuatorpos => Some(MjSensorType::ActuatorPos),
        MjcfSensorType::Actuatorvel => Some(MjSensorType::ActuatorVel),
        MjcfSensorType::Actuatorfrc => Some(MjSensorType::ActuatorFrc),
        MjcfSensorType::Framepos => Some(MjSensorType::FramePos),
        MjcfSensorType::Framequat => Some(MjSensorType::FrameQuat),
        MjcfSensorType::Framexaxis => Some(MjSensorType::FrameXAxis),
        MjcfSensorType::Frameyaxis => Some(MjSensorType::FrameYAxis),
        MjcfSensorType::Framezaxis => Some(MjSensorType::FrameZAxis),
        MjcfSensorType::Framelinvel => Some(MjSensorType::FrameLinVel),
        MjcfSensorType::Frameangvel => Some(MjSensorType::FrameAngVel),
        MjcfSensorType::Framelinacc => Some(MjSensorType::FrameLinAcc),
        MjcfSensorType::Frameangacc => Some(MjSensorType::FrameAngAcc),
        MjcfSensorType::Touch => Some(MjSensorType::Touch),
        MjcfSensorType::Force => Some(MjSensorType::Force),
        MjcfSensorType::Torque => Some(MjSensorType::Torque),
        MjcfSensorType::Accelerometer => Some(MjSensorType::Accelerometer),
        MjcfSensorType::Gyro => Some(MjSensorType::Gyro),
        MjcfSensorType::Velocimeter => Some(MjSensorType::Velocimeter),
        MjcfSensorType::Magnetometer => Some(MjSensorType::Magnetometer),
        MjcfSensorType::Rangefinder => Some(MjSensorType::Rangefinder),
        MjcfSensorType::Subtreecom => Some(MjSensorType::SubtreeCom),
        MjcfSensorType::Subtreelinvel => Some(MjSensorType::SubtreeLinVel),
        MjcfSensorType::Subtreeangmom => Some(MjSensorType::SubtreeAngMom),
        MjcfSensorType::User => Some(MjSensorType::User),
        MjcfSensorType::Jointlimitfrc => Some(MjSensorType::JointLimitFrc),
        MjcfSensorType::Tendonlimitfrc => Some(MjSensorType::TendonLimitFrc),
    }
}

/// Determine which evaluation stage processes a given sensor type.
fn sensor_datatype(t: MjSensorType) -> MjSensorDataType {
    match t {
        // Position stage (evaluated in mj_sensor_pos, after FK)
        MjSensorType::JointPos
        | MjSensorType::BallQuat
        | MjSensorType::TendonPos
        | MjSensorType::ActuatorPos
        | MjSensorType::FramePos
        | MjSensorType::FrameQuat
        | MjSensorType::FrameXAxis
        | MjSensorType::FrameYAxis
        | MjSensorType::FrameZAxis
        | MjSensorType::SubtreeCom
        | MjSensorType::Rangefinder
        | MjSensorType::Magnetometer => MjSensorDataType::Position,

        // Velocity stage (evaluated in mj_sensor_vel, after velocity FK)
        MjSensorType::JointVel
        | MjSensorType::BallAngVel
        | MjSensorType::TendonVel
        | MjSensorType::ActuatorVel
        | MjSensorType::Gyro
        | MjSensorType::Velocimeter
        | MjSensorType::FrameLinVel
        | MjSensorType::FrameAngVel
        | MjSensorType::SubtreeLinVel
        | MjSensorType::SubtreeAngMom => MjSensorDataType::Velocity,

        // Acceleration stage (evaluated in mj_sensor_acc, after constraint solver).
        // User sensors also default to acceleration (evaluated last).
        MjSensorType::Accelerometer
        | MjSensorType::Force
        | MjSensorType::Torque
        | MjSensorType::Touch
        | MjSensorType::ActuatorFrc
        | MjSensorType::JointLimitFrc
        | MjSensorType::TendonLimitFrc
        | MjSensorType::FrameLinAcc
        | MjSensorType::FrameAngAcc
        | MjSensorType::User => MjSensorDataType::Acceleration,
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

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

    #[test]
    fn test_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="floating">
                <worldbody>
                    <body name="ball" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 7); // 3 pos + 4 quat
        assert_eq!(model.nv, 6); // 3 linear + 3 angular
        assert_eq!(model.jnt_type[0], MjJointType::Free);
    }

    #[test]
    fn test_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nq, 4); // quaternion
        assert_eq!(model.nv, 3); // 3 angular DOFs
        assert_eq!(model.jnt_type[0], MjJointType::Ball);
    }

    /// Test that dof_parent forms correct kinematic tree for simple chain.
    ///
    /// For a 2-link pendulum (world → link1 → link2), each with 1 hinge:
    /// - DOF 0 (link1's hinge): parent = None (attached to world)
    /// - DOF 1 (link2's hinge): parent = Some(0) (attached to link1's DOF)
    #[test]
    fn test_dof_parent_chain() {
        let model = load_model(
            r#"
            <mujoco model="chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        // DOF 0: link1's hinge, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: link2's hinge, parent is DOF 0 (link1's hinge)
        assert_eq!(model.dof_parent[1], Some(0));
    }

    /// Test dof_parent for multi-DOF joints (ball joint).
    ///
    /// For a ball joint (3 DOFs):
    /// - DOF 0: parent = parent body's last DOF (or None)
    /// - DOF 1: parent = DOF 0
    /// - DOF 2: parent = DOF 1
    #[test]
    fn test_dof_parent_ball_joint() {
        let model = load_model(
            r#"
            <mujoco model="ball_chain">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="j1" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);
        // DOF 0: first DOF of ball, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOF 1: second DOF of ball, parent is DOF 0
        assert_eq!(model.dof_parent[1], Some(0));
        // DOF 2: third DOF of ball, parent is DOF 1
        assert_eq!(model.dof_parent[2], Some(1));
    }

    /// Test dof_parent for free joint (6 DOFs).
    #[test]
    fn test_dof_parent_free_joint() {
        let model = load_model(
            r#"
            <mujoco model="free_body">
                <worldbody>
                    <body name="floating" pos="0 0 1">
                        <joint type="free"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 6);
        // DOF 0: first DOF of free, parent is world (None)
        assert_eq!(model.dof_parent[0], None);
        // DOFs 1-5: chain within free joint
        for i in 1..6 {
            assert_eq!(model.dof_parent[i], Some(i - 1));
        }
    }

    /// Test dof_parent for chain with mixed joint types.
    ///
    /// world → link1 (ball, 3 DOF) → link2 (hinge, 1 DOF)
    /// - DOFs 0,1,2: ball joint, parents: None, 0, 1
    /// - DOF 3: hinge, parent is DOF 2 (last DOF of ball)
    #[test]
    fn test_dof_parent_mixed_joints() {
        let model = load_model(
            r#"
            <mujoco model="mixed">
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <joint name="ball" type="ball"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <body name="link2" pos="0 0 -0.5">
                            <joint name="hinge" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 4); // 3 (ball) + 1 (hinge)

        // Ball joint DOFs
        assert_eq!(model.dof_parent[0], None);
        assert_eq!(model.dof_parent[1], Some(0));
        assert_eq!(model.dof_parent[2], Some(1));

        // Hinge DOF: parent is ball's last DOF
        assert_eq!(model.dof_parent[3], Some(2));
    }

    /// Test dof_parent for branching tree (two children of same parent).
    ///
    /// world → base (hinge) ─┬→ left_arm (hinge)
    ///                       └→ right_arm (hinge)
    ///
    /// DOF 0: base
    /// DOF 1: left_arm, parent = 0
    /// DOF 2: right_arm, parent = 0
    #[test]
    fn test_dof_parent_branching() {
        let model = load_model(
            r#"
            <mujoco model="branching">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <joint name="j_base" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="2.0" diaginertia="0.2 0.2 0.1"/>
                        <body name="left_arm" pos="0.5 0 0">
                            <joint name="j_left" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                        <body name="right_arm" pos="-0.5 0 0">
                            <joint name="j_right" type="hinge" axis="0 1 0"/>
                            <inertial pos="0 0 0" mass="0.5" diaginertia="0.05 0.05 0.01"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 3);

        // Base joint: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Both arms attach to base
        assert_eq!(model.dof_parent[1], Some(0)); // left_arm
        assert_eq!(model.dof_parent[2], Some(0)); // right_arm
    }

    /// Test dof_parent for body with multiple joints (uncommon but valid).
    ///
    /// In MuJoCo, a single body can have multiple joints. The DOFs chain:
    /// world → link (hinge1, hinge2)
    /// - DOF 0: hinge1, parent = None
    /// - DOF 1: hinge2, parent = DOF 0
    #[test]
    fn test_dof_parent_multiple_joints_one_body() {
        let model = load_model(
            r#"
            <mujoco model="multi_joint">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="1 0 0"/>
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nv, 2);
        assert_eq!(model.njnt, 2);

        // First joint's DOF: attached to world
        assert_eq!(model.dof_parent[0], None);
        // Second joint's DOF: attached to first joint
        assert_eq!(model.dof_parent[1], Some(0));
    }

    /// Test that tendon actuators return proper error.
    #[test]
    fn test_tendon_actuator_unknown_tendon_error() {
        // Actuator referencing a nonexistent tendon should fail with "unknown tendon"
        let result = load_model(
            r#"
            <mujoco model="tendon_test">
                <worldbody>
                    <body name="link" pos="0 0 1">
                        <joint name="j1" type="hinge"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="m1" tendon="some_tendon" gear="1"/>
                </actuator>
            </mujoco>
        "#,
        );

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("unknown tendon"));
    }

    /// Test capsule inertia computation is physically reasonable.
    #[test]
    fn test_capsule_inertia() {
        // A capsule should have I_z < I_x = I_y (axially symmetric, longer than wide)
        let geom = MjcfGeom {
            name: None,
            class: None,
            geom_type: MjcfGeomType::Capsule,
            pos: Vector3::zeros(),
            quat: nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0),
            euler: None,
            axisangle: None,
            xyaxes: None,
            zaxis: None,
            size: vec![0.1, 0.5], // radius=0.1, half-height=0.5
            fromto: None,
            density: 1000.0,
            mass: None,
            friction: Vector3::new(1.0, 0.005, 0.0001),
            rgba: nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0),
            contype: 1,
            conaffinity: 1,
            condim: 3,
            mesh: None,
            hfield: None,
            solref: None,
            solimp: None,
        };

        let inertia = compute_geom_inertia(&geom, None);

        // Ix = Iy (axially symmetric) — diagonal elements (0,0) and (1,1)
        assert!((inertia[(0, 0)] - inertia[(1, 1)]).abs() < 1e-10);

        // Iz < Ix (thin cylinder is easier to spin about long axis)
        assert!(inertia[(2, 2)] < inertia[(0, 0)]);

        // All positive
        assert!(inertia[(0, 0)] > 0.0);
        assert!(inertia[(1, 1)] > 0.0);
        assert!(inertia[(2, 2)] > 0.0);
    }

    /// Test site parsing and model population.
    #[test]
    fn test_sites() {
        let model = load_model(
            r#"
            <mujoco model="site_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="capsule" size="0.05 0.5"/>
                        <site name="end_effector" pos="0 0 -1" size="0.02"/>
                        <site name="sensor_mount" pos="0.1 0 0" size="0.01 0.01 0.02"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nsite, 2);
        assert_eq!(model.site_body.len(), 2);

        // Check first site (end_effector)
        assert_eq!(model.site_body[0], 1); // Attached to arm (body 1)
        assert!((model.site_pos[0].z - (-1.0)).abs() < 1e-10);
        assert_eq!(model.site_name[0], Some("end_effector".to_string()));

        // Check second site (sensor_mount)
        assert_eq!(model.site_body[1], 1);
        assert!((model.site_pos[1].x - 0.1).abs() < 1e-10);
    }

    /// Test site actuator transmission.
    #[test]
    fn test_site_actuator() {
        let model = load_model(
            r#"
            <mujoco model="site_actuator_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <site name="end_effector" pos="0 0 -1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general name="site_act" site="end_effector" gear="50"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Site);
        assert_eq!(model.actuator_trnid[0][0], 0); // First site
        assert_eq!(model.actuator_trnid[0][1], usize::MAX); // No refsite
        assert!((model.actuator_gear[0][0] - 50.0).abs() < 1e-10);
    }

    /// Test actuator activation states (na) computation.
    #[test]
    fn test_actuator_activation_states() {
        let model = load_model(
            r#"
            <mujoco model="activation_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <joint name="j2" type="hinge" axis="1 0 0"/>
                        <joint name="j3" type="hinge" axis="0 0 1"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="j1" gear="10"/>
                    <position name="pos1" joint="j2" kp="100"/>
                    <velocity name="vel1" joint="j3" kv="10"/>
                </actuator>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nu, 3);

        // Motor has no dynamics -> 0 activation states
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[0], 0);
        assert_eq!(model.actuator_act_num[0], 0);

        // Position servo (no timeconst) has no dynamics -> 0 activation states
        // MuJoCo: Position defaults to dyntype=None when timeconst=0.
        assert_eq!(model.actuator_dyntype[1], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[1], 0);
        assert_eq!(model.actuator_act_num[1], 0);

        // Velocity servo always has no dynamics -> 0 activation states
        assert_eq!(model.actuator_dyntype[2], ActuatorDynamics::None);
        assert_eq!(model.actuator_act_adr[2], 0);
        assert_eq!(model.actuator_act_num[2], 0);

        // Total activation states
        assert_eq!(model.na, 0); // 0 + 0 + 0 = 0
    }

    // =========================================================================
    // Mesh file loading tests
    // =========================================================================

    /// Test resolve_asset_path with absolute path.
    #[test]
    fn test_resolve_asset_path_absolute() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let abs_path = mesh_path.to_string_lossy().to_string();
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(&abs_path, None, &compiler, AssetKind::Mesh);
        assert!(
            result.is_ok(),
            "Absolute path should resolve without base_path"
        );
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path fails for non-existent absolute path.
    #[test]
    fn test_resolve_asset_path_absolute_not_found() {
        // Use platform-appropriate absolute path that definitely doesn't exist
        #[cfg(windows)]
        let nonexistent_path = "C:\\nonexistent_dir_12345\\mesh.stl";
        #[cfg(not(windows))]
        let nonexistent_path = "/nonexistent_dir_12345/mesh.stl";

        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(nonexistent_path, None, &compiler, AssetKind::Mesh);
        assert!(result.is_err());
        assert!(
            result.unwrap_err().to_string().contains("not found"),
            "Error should mention 'not found' for non-existent absolute path"
        );
    }

    /// Test resolve_asset_path with relative path and base_path.
    #[test]
    fn test_resolve_asset_path_relative() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("meshes").join("test.stl");
        std::fs::create_dir_all(mesh_path.parent().unwrap()).unwrap();
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(
            "meshes/test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path fails for relative path without base_path.
    #[test]
    fn test_resolve_asset_path_relative_no_base() {
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path("meshes/test.stl", None, &compiler, AssetKind::Mesh);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("relative"));
        assert!(err.contains("no base path"));
    }

    /// Test resolve_asset_path fails for relative path when file doesn't exist.
    #[test]
    fn test_resolve_asset_path_relative_not_found() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(
            "nonexistent.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("not found"));
    }

    /// Test resolve_asset_path with meshdir setting.
    #[test]
    fn test_resolve_asset_path_meshdir() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("custom_meshes").join("test.stl");
        std::fs::create_dir_all(mesh_path.parent().unwrap()).unwrap();
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let mut compiler = MjcfCompiler::default();
        compiler.meshdir = Some("custom_meshes".to_string());
        let result = resolve_asset_path(
            "test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok(), "meshdir should resolve: {result:?}");
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path with strippath.
    #[test]
    fn test_resolve_asset_path_strippath() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let mut compiler = MjcfCompiler::default();
        compiler.strippath = true;
        // File path has directory components, but strippath removes them
        let result = resolve_asset_path(
            "some/nested/dir/test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok(), "strippath should strip dirs: {result:?}");
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Helper to create a simple STL file for testing.
    fn create_test_stl(path: &std::path::Path) {
        use mesh_types::{IndexedMesh, Vertex};

        let mut mesh = IndexedMesh::new();
        // Simple tetrahedron (4 vertices, 4 faces)
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]); // base
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left

        mesh_io::save_mesh(&mesh, path).expect("Failed to save test STL");
    }

    /// Test load_mesh_file with STL format.
    #[test]
    fn test_load_mesh_file_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        create_test_stl(&mesh_path);

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "test_mesh",
        );

        assert!(result.is_ok(), "Should load STL file");
        let mesh_data = result.unwrap();
        assert!(
            mesh_data.vertices().len() >= 4,
            "Should have at least 4 vertices"
        );
        assert!(
            mesh_data.triangles().len() >= 4,
            "Should have at least 4 triangles"
        );
    }

    /// Test load_mesh_file applies scale correctly.
    #[test]
    fn test_load_mesh_file_with_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("scaled.stl");
        create_test_stl(&mesh_path);

        // Load without scale
        let unscaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unscaled",
        )
        .unwrap();

        // Load with 2x scale
        let scaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(2.0, 2.0, 2.0),
            "scaled",
        )
        .unwrap();

        // Verify scale was applied: scaled vertices should be 2x unscaled
        // Find corresponding vertices and compare
        assert_eq!(scaled.vertices().len(), unscaled.vertices().len());
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!(
                (s.x - u.x * 2.0).abs() < 1e-10,
                "X should be scaled: {} vs {}",
                s.x,
                u.x * 2.0
            );
            assert!(
                (s.y - u.y * 2.0).abs() < 1e-10,
                "Y should be scaled: {} vs {}",
                s.y,
                u.y * 2.0
            );
            assert!(
                (s.z - u.z * 2.0).abs() < 1e-10,
                "Z should be scaled: {} vs {}",
                s.z,
                u.z * 2.0
            );
        }
    }

    /// Test load_mesh_file with non-uniform scale.
    #[test]
    fn test_load_mesh_file_nonuniform_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("nonuniform.stl");
        create_test_stl(&mesh_path);

        // Load with non-uniform scale: 1x, 2x, 3x
        let scaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 2.0, 3.0),
            "nonuniform",
        )
        .unwrap();

        // Load unscaled for comparison
        let unscaled = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unscaled",
        )
        .unwrap();

        // Verify non-uniform scale: x unchanged, y doubled, z tripled
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!((s.x - u.x).abs() < 1e-10, "X should be unchanged");
            assert!((s.y - u.y * 2.0).abs() < 1e-10, "Y should be 2x");
            assert!((s.z - u.z * 3.0).abs() < 1e-10, "Z should be 3x");
        }
    }

    /// Test load_mesh_file fails for unsupported format.
    #[test]
    fn test_load_mesh_file_unsupported_format() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.xyz");
        std::fs::write(&mesh_path, b"invalid mesh data").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "unsupported",
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("failed to load"));
    }

    /// Test load_mesh_file fails gracefully for corrupt STL data.
    ///
    /// A file with `.stl` extension but invalid content should produce
    /// a clear error, not panic.
    #[test]
    fn test_load_mesh_file_corrupt_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("corrupt.stl");

        // Write invalid STL data (valid STL needs specific binary/ASCII structure)
        std::fs::write(&mesh_path, b"this is not a valid stl file content").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            Vector3::new(1.0, 1.0, 1.0),
            "corrupt_mesh",
        );

        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("corrupt_mesh") && err_msg.contains("failed to load"),
            "error should identify mesh and indicate load failure: {err_msg}"
        );
    }

    /// Test convert_mjcf_mesh with file attribute.
    #[test]
    fn test_convert_mjcf_mesh_from_file() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("mesh.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "test_mesh".to_string(),
            file: Some(mesh_path.to_string_lossy().to_string()),
            vertex: None,
            face: None,
            scale: Vector3::new(1.0, 1.0, 1.0),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert!(!mesh_data.vertices().is_empty());
        assert!(!mesh_data.triangles().is_empty());
    }

    /// Test convert_mjcf_mesh with file attribute and relative path.
    #[test]
    fn test_convert_mjcf_mesh_relative_path() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let meshes_dir = temp_dir.path().join("assets");
        std::fs::create_dir_all(&meshes_dir).unwrap();
        let mesh_path = meshes_dir.join("model.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "relative_mesh".to_string(),
            file: Some("assets/model.stl".to_string()),
            vertex: None,
            face: None,
            scale: Vector3::new(1.0, 1.0, 1.0),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, Some(temp_dir.path()), &MjcfCompiler::default());
        assert!(result.is_ok());
    }

    /// Test convert_mjcf_mesh with embedded vertex data (no file).
    #[test]
    fn test_convert_mjcf_mesh_embedded() {
        let mjcf_mesh = MjcfMesh {
            name: "embedded".to_string(),
            file: None,
            vertex: Some(vec![
                0.0, 0.0, 0.0, // v0
                1.0, 0.0, 0.0, // v1
                0.5, 1.0, 0.0, // v2
                0.5, 0.5, 1.0, // v3
            ]),
            face: Some(vec![
                0, 1, 2, // f0
                0, 1, 3, // f1
                1, 2, 3, // f2
                2, 0, 3, // f3
            ]),
            scale: Vector3::new(1.0, 1.0, 1.0),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert_eq!(mesh_data.vertices().len(), 4);
        assert_eq!(mesh_data.triangles().len(), 4); // 4 triangles
    }

    /// Test convert_mjcf_mesh fails when neither file nor vertex is present.
    #[test]
    fn test_convert_mjcf_mesh_no_data() {
        let mjcf_mesh = MjcfMesh {
            name: "empty".to_string(),
            file: None,
            vertex: None,
            face: None,
            scale: Vector3::new(1.0, 1.0, 1.0),
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("no vertex data"));
    }

    /// Test load_model_from_file with mesh geometry.
    #[test]
    fn test_load_model_from_file_with_mesh() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");

        // Create mesh file
        let mesh_path = temp_dir.path().join("cube.stl");
        create_test_stl(&mesh_path);

        // Create MJCF file referencing the mesh
        let mjcf_content = r#"
            <mujoco model="mesh_test">
                <asset>
                    <mesh name="cube_mesh" file="cube.stl"/>
                </asset>
                <worldbody>
                    <body name="cube" pos="0 0 1">
                        <geom type="mesh" mesh="cube_mesh" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#;
        let mjcf_path = temp_dir.path().join("model.xml");
        std::fs::write(&mjcf_path, mjcf_content).unwrap();

        let result = load_model_from_file(&mjcf_path);
        assert!(
            result.is_ok(),
            "Should load model with mesh: {:?}",
            result.err()
        );
        let model = result.unwrap();
        assert_eq!(model.name, "mesh_test");
        assert_eq!(model.nmesh, 1);
        assert!(!model.mesh_data.is_empty());
    }

    /// Test model_from_mjcf with explicit base_path.
    #[test]
    fn test_model_from_mjcf_with_base_path() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");

        // Create mesh in subdirectory
        let assets_dir = temp_dir.path().join("assets");
        std::fs::create_dir_all(&assets_dir).unwrap();
        let mesh_path = assets_dir.join("shape.stl");
        create_test_stl(&mesh_path);

        // Parse MJCF referencing relative mesh path
        let mjcf = crate::parse_mjcf_str(
            r#"
            <mujoco model="base_path_test">
                <asset>
                    <mesh name="shape" file="assets/shape.stl"/>
                </asset>
                <worldbody>
                    <body name="obj" pos="0 0 1">
                        <geom type="mesh" mesh="shape" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("Should parse MJCF");

        // Convert with base_path
        let result = model_from_mjcf(&mjcf, Some(temp_dir.path()));
        assert!(
            result.is_ok(),
            "Should convert with base_path: {:?}",
            result.err()
        );
    }

    // ========================================================================
    // <general> actuator MJCF attributes (spec §8)
    // ========================================================================

    /// Helper: minimal MJCF with a single body+joint for actuator tests.
    fn general_actuator_model(actuator_xml: &str) -> String {
        format!(
            r#"
            <mujoco model="general_actuator_test">
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    {actuator_xml}
                </actuator>
            </mujoco>
            "#
        )
    }

    /// Criterion 1: Explicit gaintype/biastype on `<general>`.
    #[test]
    fn test_general_explicit_gaintype_biastype() {
        let xml = general_actuator_model(
            r#"<general joint="j" gaintype="affine" gainprm="0 0 -5"
                       biastype="none" ctrllimited="true" ctrlrange="0 1"/>"#,
        );
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        // gainprm = [0, 0, -5, 0, 0, 0, 0, 0, 0]
        assert!((model.actuator_gainprm[0][0]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][2] - (-5.0)).abs() < 1e-10);
        for i in 3..9 {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
    }

    /// Criterion 2: Explicit dyntype with dynprm.
    #[test]
    fn test_general_explicit_dyntype() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="filter" dynprm="0.05"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::Filter);
        assert!((model.actuator_dynprm[0][0] - 0.05).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
        // Filter → 1 activation state
        assert_eq!(model.actuator_act_num[0], 1);
    }

    /// Criterion 3: Bare `<general>` backward compatibility (Motor-like defaults).
    #[test]
    fn test_general_bare_defaults() {
        let xml = general_actuator_model(r#"<general joint="j"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.nu, 1);
        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        // gainprm = [1, 0, ..., 0] (MuJoCo default)
        assert!((model.actuator_gainprm[0][0] - 1.0).abs() < 1e-10);
        for i in 1..9 {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
        // biasprm = [0, ..., 0]
        for i in 0..9 {
            assert!((model.actuator_biasprm[0][i]).abs() < 1e-10);
        }
        // dynprm = [1, 0, 0] (MuJoCo default)
        assert!((model.actuator_dynprm[0][0] - 1.0).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
        // No dynamics → 0 activation states
        assert_eq!(model.actuator_act_num[0], 0);
    }

    /// Criterion 4: PD servo via `<general>` matches `<position>` shortcut.
    #[test]
    fn test_general_pd_servo_equivalence() {
        let general_xml = general_actuator_model(
            r#"<general joint="j" gaintype="fixed" gainprm="100"
                       biastype="affine" biasprm="0 -100 -10"
                       dyntype="filterexact" dynprm="0.01"/>"#,
        );
        let position_xml =
            general_actuator_model(r#"<position joint="j" kp="100" kv="10" timeconst="0.01"/>"#);

        let m_gen = load_model(&general_xml).expect("should load general");
        let m_pos = load_model(&position_xml).expect("should load position");

        assert_eq!(m_gen.actuator_gaintype[0], m_pos.actuator_gaintype[0]);
        assert_eq!(m_gen.actuator_biastype[0], m_pos.actuator_biastype[0]);
        assert_eq!(m_gen.actuator_dyntype[0], m_pos.actuator_dyntype[0]);
        for i in 0..9 {
            assert!(
                (m_gen.actuator_gainprm[0][i] - m_pos.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_gainprm[0][i],
                m_pos.actuator_gainprm[0][i]
            );
            assert!(
                (m_gen.actuator_biasprm[0][i] - m_pos.actuator_biasprm[0][i]).abs() < 1e-10,
                "biasprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_biasprm[0][i],
                m_pos.actuator_biasprm[0][i]
            );
        }
        for i in 0..3 {
            assert!(
                (m_gen.actuator_dynprm[0][i] - m_pos.actuator_dynprm[0][i]).abs() < 1e-10,
                "dynprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_dynprm[0][i],
                m_pos.actuator_dynprm[0][i]
            );
        }
        assert_eq!(m_gen.actuator_act_num[0], m_pos.actuator_act_num[0]);
    }

    /// Criterion 5: Muscle via `<general>` matches `<muscle>` shortcut.
    #[test]
    fn test_general_muscle_equivalence() {
        let general_xml = general_actuator_model(
            r#"<general joint="j" gaintype="muscle" biastype="muscle"
                       dyntype="muscle"
                       gainprm="0.75 1.05 -1 200 0.5 1.6 1.5 0.6 1.4"
                       dynprm="0.01 0.04 0"/>"#,
        );
        let muscle_xml = general_actuator_model(
            r#"<muscle joint="j" range="0.75 1.05" force="-1" scale="200"
                      lmin="0.5" lmax="1.6" vmax="1.5" fpmax="0.6" fvmax="1.4"
                      timeconst="0.01 0.04"/>"#,
        );

        let m_gen = load_model(&general_xml).expect("should load general");
        let m_mus = load_model(&muscle_xml).expect("should load muscle");

        assert_eq!(m_gen.actuator_gaintype[0], m_mus.actuator_gaintype[0]);
        assert_eq!(m_gen.actuator_biastype[0], m_mus.actuator_biastype[0]);
        assert_eq!(m_gen.actuator_dyntype[0], m_mus.actuator_dyntype[0]);
        for i in 0..9 {
            assert!(
                (m_gen.actuator_gainprm[0][i] - m_mus.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_gainprm[0][i],
                m_mus.actuator_gainprm[0][i]
            );
        }
        for i in 0..3 {
            assert!(
                (m_gen.actuator_dynprm[0][i] - m_mus.actuator_dynprm[0][i]).abs() < 1e-10,
                "dynprm[{i}] mismatch: {} vs {}",
                m_gen.actuator_dynprm[0][i],
                m_mus.actuator_dynprm[0][i]
            );
        }
        assert_eq!(m_gen.actuator_act_num[0], m_mus.actuator_act_num[0]);
    }

    /// Criterion 6: Default class inheritance for `<general>` attributes.
    #[test]
    fn test_general_default_class_inheritance() {
        let xml = r#"
            <mujoco model="general_defaults">
                <default>
                    <general gaintype="affine" gainprm="0 0 -10"/>
                </default>
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_model(xml).expect("should load");
        assert_eq!(model.actuator_gaintype[0], GainType::Affine);
        assert!((model.actuator_gainprm[0][0]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_gainprm[0][2] - (-10.0)).abs() < 1e-10);
    }

    /// Criterion 6 (continued): Explicit attribute overrides class default.
    #[test]
    fn test_general_default_class_override() {
        let xml = r#"
            <mujoco model="general_defaults_override">
                <default>
                    <general gaintype="affine" gainprm="0 0 -10"/>
                </default>
                <worldbody>
                    <body name="b" pos="0 0 1">
                        <joint name="j" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j" gaintype="fixed" gainprm="50"/>
                </actuator>
            </mujoco>
        "#;

        let model = load_model(xml).expect("should load");
        // Explicit gaintype="fixed" overrides class default "affine"
        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        // Explicit gainprm="50" overrides class default "0 0 -10"
        assert!((model.actuator_gainprm[0][0] - 50.0).abs() < 1e-10);
        for i in 1..9 {
            assert!((model.actuator_gainprm[0][i]).abs() < 1e-10);
        }
    }

    /// Criterion 7: Partial gainprm (fewer than 9 elements).
    #[test]
    fn test_general_partial_gainprm() {
        let xml = general_actuator_model(r#"<general joint="j" gainprm="50"/>"#);
        let model = load_model(&xml).expect("should load");

        assert!((model.actuator_gainprm[0][0] - 50.0).abs() < 1e-10);
        for i in 1..9 {
            assert!(
                (model.actuator_gainprm[0][i]).abs() < 1e-10,
                "gainprm[{i}] should be 0, got {}",
                model.actuator_gainprm[0][i]
            );
        }
    }

    /// Criterion 8: dynprm defaults to [1,0,0] when dyntype is set but dynprm absent.
    #[test]
    fn test_general_dynprm_default() {
        let xml = general_actuator_model(r#"<general joint="j" dyntype="filter"/>"#);
        let model = load_model(&xml).expect("should load");

        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::Filter);
        assert!((model.actuator_dynprm[0][0] - 1.0).abs() < 1e-10); // τ = 1.0s
        assert!((model.actuator_dynprm[0][1]).abs() < 1e-10);
        assert!((model.actuator_dynprm[0][2]).abs() < 1e-10);
    }

    /// Criterion 9: Invalid enum produces ModelConversionError.
    #[test]
    fn test_general_invalid_gaintype_error() {
        let xml = general_actuator_model(r#"<general joint="j" gaintype="invalid"/>"#);
        let err = load_model(&xml).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("unknown gaintype"),
            "error should contain 'unknown gaintype', got: {msg}"
        );
    }

    /// Criterion 10: "user" type produces error (not special-cased).
    #[test]
    fn test_general_user_gaintype_error() {
        let xml = general_actuator_model(r#"<general joint="j" gaintype="user"/>"#);
        let err = load_model(&xml).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("unknown gaintype 'user'"),
            "error should contain \"unknown gaintype 'user'\", got: {msg}"
        );
    }

    /// Criterion 11: Shortcut types ignore gaintype/biastype/dyntype attributes.
    #[test]
    fn test_shortcut_types_ignore_general_attrs() {
        // A <motor> with gaintype/biastype should still produce Motor-like behavior.
        // These attributes are not parsed for shortcut types (gated by actuator_type == General).
        let xml = general_actuator_model(r#"<motor joint="j" gear="1"/>"#);
        let model = load_model(&xml).expect("should load motor");

        assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
        assert_eq!(model.actuator_biastype[0], BiasType::None);
        assert_eq!(model.actuator_dyntype[0], ActuatorDynamics::None);
        assert!((model.actuator_gainprm[0][0] - 1.0).abs() < 1e-10);
    }

    /// Criterion 12: All dyntype values wire correctly.
    #[test]
    fn test_general_all_dyntype_values() {
        let cases = [
            ("none", ActuatorDynamics::None, 0),
            ("integrator", ActuatorDynamics::Integrator, 1),
            ("filter", ActuatorDynamics::Filter, 1),
            ("filterexact", ActuatorDynamics::FilterExact, 1),
            ("muscle", ActuatorDynamics::Muscle, 1),
        ];
        for (dyntype_str, expected_dyn, expected_act_num) in &cases {
            let xml =
                general_actuator_model(&format!(r#"<general joint="j" dyntype="{dyntype_str}"/>"#));
            let model = load_model(&xml).expect("should load dyntype");

            assert_eq!(
                model.actuator_dyntype[0], *expected_dyn,
                "dyntype={dyntype_str}: wrong ActuatorDynamics"
            );
            assert_eq!(
                model.actuator_act_num[0], *expected_act_num,
                "dyntype={dyntype_str}: wrong act_num"
            );
        }
    }

    /// Criterion 13: Extra gainprm elements (>9) silently truncated.
    #[test]
    fn test_general_extra_gainprm_truncated() {
        let xml = general_actuator_model(r#"<general joint="j" gainprm="1 2 3 4 5 6 7 8 9 10"/>"#);
        let model = load_model(&xml).expect("should load");

        // First 9 elements stored
        let expected = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        for (i, &exp) in expected.iter().enumerate() {
            assert!(
                (model.actuator_gainprm[0][i] - exp).abs() < 1e-10,
                "gainprm[{i}] expected {exp}, got {}",
                model.actuator_gainprm[0][i]
            );
        }
        // 10th element (index 9) is silently dropped — no error, and array is only 9 elements
    }

    // =========================================================================
    // Compiler / Angle Conversion / Euler Sequence Tests
    // =========================================================================

    #[test]
    fn test_euler_seq_to_quat_intrinsic_xyz() {
        // Default MuJoCo sequence: intrinsic xyz → Rx * Ry * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "xyz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rx * ry * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic xyz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_extrinsic_xyz() {
        // Extrinsic XYZ → pre-multiply: Rz * Ry * Rx (= intrinsic zyx)
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYZ");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "extrinsic XYZ failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_mixed_case() {
        // Mixed: XYz → extrinsic X, extrinsic Y, intrinsic z
        // q = identity
        // ch='X' (extrinsic): q = Rx * q = Rx
        // ch='Y' (extrinsic): q = Ry * q = Ry * Rx
        // ch='z' (intrinsic): q = q * Rz = Ry * Rx * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = ry * rx * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "mixed XYz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_matches_different_order() {
        // Intrinsic zyx should match nalgebra's from_euler_angles(roll, pitch, yaw)
        // which is extrinsic XYZ
        let euler = Vector3::new(0.3, 0.2, 0.1); // z, y, x angles
        let q = euler_seq_to_quat(euler, "zyx");

        // Intrinsic zyx: Rz * Ry * Rx
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic zyx failed"
        );
    }

    #[test]
    fn test_angle_conversion_hinge_joint_degrees() {
        // Default compiler: angle=degree. Hinge range in degrees → converted to radians.
        let model = load_model(
            r#"
            <mujoco model="deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-90 90"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        assert!(
            (model.jnt_range[0].0 - (-pi / 2.0)).abs() < 1e-10,
            "range lo: expected {}, got {}",
            -pi / 2.0,
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - (pi / 2.0)).abs() < 1e-10,
            "range hi: expected {}, got {}",
            pi / 2.0,
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_degrees() {
        // Ball joint range is angle-valued (swing/twist limits).
        // With default compiler.angle=degree, range values must be converted to radians.
        // MuJoCo semantics: range="0 60" means max-swing=0°, max-twist=60° (or similar).
        let model = load_model(
            r#"
            <mujoco model="ball_deg_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 60"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let pi = std::f64::consts::PI;
        let expected_lo = 0.0;
        let expected_hi = 60.0 * pi / 180.0; // π/3
        assert!(
            (model.jnt_range[0].0 - expected_lo).abs() < 1e-10,
            "ball range lo: expected {expected_lo}, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - expected_hi).abs() < 1e-10,
            "ball range hi: expected {expected_hi}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_ball_joint_range_radian_passthrough() {
        // With angle="radian", ball joint range values pass through unchanged.
        let pi_3 = std::f64::consts::PI / 3.0;
        let model = load_model(&format!(
            r#"
            <mujoco model="ball_rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="ball" limited="true" range="0 {pi_3}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - 0.0).abs() < 1e-10,
            "ball radian range lo: expected 0.0, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - pi_3).abs() < 1e-10,
            "ball radian range hi: expected {pi_3}, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_radian_passthrough() {
        // With angle="radian", values pass through unchanged.
        let model = load_model(
            r#"
            <mujoco model="rad_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="true" range="-1.57 1.57"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-1.57)).abs() < 1e-10,
            "range lo: expected -1.57, got {}",
            model.jnt_range[0].0
        );
        assert!(
            (model.jnt_range[0].1 - 1.57).abs() < 1e-10,
            "range hi: expected 1.57, got {}",
            model.jnt_range[0].1
        );
    }

    #[test]
    fn test_angle_conversion_slide_joint_not_converted() {
        // Slide joint range is translational — never converted even with angle=degree.
        let model = load_model(
            r#"
            <mujoco model="slide_test">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="slide" limited="true" range="-0.5 0.5"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            (model.jnt_range[0].0 - (-0.5)).abs() < 1e-10,
            "slide range should not be converted"
        );
        assert!(
            (model.jnt_range[0].1 - 0.5).abs() < 1e-10,
            "slide range should not be converted"
        );
    }

    #[test]
    fn test_angle_conversion_springref_degrees() {
        // Hinge springref in degrees → converted to radians.
        let model = load_model(
            r#"
            <mujoco model="springref_deg">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" stiffness="100" springref="45"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = 45.0 * std::f64::consts::PI / 180.0;
        assert!(
            (model.jnt_springref[0] - expected).abs() < 1e-10,
            "springref: expected {expected}, got {}",
            model.jnt_springref[0]
        );
    }

    #[test]
    fn test_body_axisangle_orientation() {
        // Body with axisangle should produce correct quaternion.
        // axisangle="0 0 1 90" with default degree → 90° around Z.
        let model = load_model(
            r#"
            <mujoco model="axisangle_test">
                <worldbody>
                    <body name="b" axisangle="0 0 1 90">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // 90° around Z: quaternion = [cos(45°), 0, 0, sin(45°)]
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.body_quat[1]; // body 0 is world, body 1 is "b"
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle 90° Z: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_body_axisangle_radian() {
        // With angle="radian", axisangle angle is in radians.
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
                <mujoco model="axisangle_rad">
                    <compiler angle="radian"/>
                    <worldbody>
                        <body name="b" axisangle="0 0 1 {pi_2}">
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </worldbody>
                </mujoco>
                "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), pi_2);
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle radian: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_body_orientation() {
        // With eulerseq="ZYX" and angle="radian", body euler should use ZYX sequence.
        let model = load_model(
            r#"
            <mujoco model="eulerseq_test">
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b" euler="0.3 0.2 0.1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "ZYX euler: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_autolimits_infers_limited_from_range() {
        // Default autolimits=true: range present + no explicit limited → limited=true.
        let model = load_model(
            r#"
            <mujoco model="autolimits_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            model.jnt_limited[0],
            "autolimits should infer limited=true from range"
        );
    }

    #[test]
    fn test_autolimits_false_requires_explicit_limited() {
        // With autolimits=false, range without limited → limited=false.
        let model = load_model(
            r#"
            <mujoco model="no_autolimits">
                <compiler angle="radian" autolimits="false"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            !model.jnt_limited[0],
            "autolimits=false should not infer limited"
        );
    }

    #[test]
    fn test_autolimits_explicit_limited_takes_precedence() {
        // Explicit limited=false overrides autolimits inference.
        let model = load_model(
            r#"
            <mujoco model="explicit_limited">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge" limited="false" range="-1.0 1.0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(
            !model.jnt_limited[0],
            "explicit limited=false should override autolimits"
        );
    }

    #[test]
    fn test_autolimits_no_range_no_limited() {
        // No range and no limited → limited=false (regardless of autolimits).
        let model = load_model(
            r#"
            <mujoco model="no_range">
                <worldbody>
                    <body name="b">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        assert!(!model.jnt_limited[0], "no range should mean not limited");
    }

    // ── Mass pipeline tests ──────────────────────────────────────────

    #[test]
    fn test_inertiafromgeom_true_overrides_explicit_inertial() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="999.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // inertiafromgeom="true" → geom mass (2.0) overrides explicit 999.0
        assert!(
            (model.body_mass[1] - 2.0).abs() < 1e-10,
            "mass should come from geom, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_uses_explicit_only() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="5.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 5.0).abs() < 1e-10,
            "mass should come from explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_no_inertial_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "mass should be zero without explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_auto_no_geoms_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint type="hinge" axis="0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Auto mode with no geoms and no inertial → zero mass
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "empty body should have zero mass, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundmass_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="0.5"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 0.5).abs() < 1e-10,
            "mass should be clamped to 0.5, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundinertia_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundinertia="0.01"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.01).abs() < 1e-10
                && (inertia.y - 0.01).abs() < 1e-10
                && (inertia.z - 0.01).abs() < 1e-10,
            "inertia should be clamped to 0.01, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_fixes_triangle_inequality() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // 0.1 + 0.1 < 0.5 violates triangle inequality → mean = (0.1+0.1+0.5)/3
        let expected = (0.1 + 0.1 + 0.5) / 3.0;
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - expected).abs() < 1e-10
                && (inertia.y - expected).abs() < 1e-10
                && (inertia.z - expected).abs() < 1e-10,
            "inertia should be balanced to mean {expected}, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_no_change_when_valid() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.3 0.3 0.3"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.3).abs() < 1e-10
                && (inertia.y - 0.3).abs() < 1e-10
                && (inertia.z - 0.3).abs() < 1e-10,
            "valid inertia should not change, got {inertia:?}"
        );
    }

    #[test]
    fn test_settotalmass_rescales() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="10.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 3 + 7 = 10, settotalmass = 10 → no change
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 10.0).abs() < 1e-10,
            "total mass should be 10.0, got {total}"
        );
    }

    #[test]
    fn test_settotalmass_rescales_different_target() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="20.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 10, target = 20 → scale = 2x
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 20.0).abs() < 1e-10,
            "total mass should be 20.0, got {total}"
        );
        // Mass ratios should be preserved: a=6.0, b=14.0
        assert!(
            (model.body_mass[1] - 6.0).abs() < 1e-10,
            "body a mass should be 6.0, got {}",
            model.body_mass[1]
        );
        assert!(
            (model.body_mass[2] - 14.0).abs() < 1e-10,
            "body b mass should be 14.0, got {}",
            model.body_mass[2]
        );
    }

    #[test]
    fn test_mass_pipeline_order_bound_then_settotalmass() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="1.0" settotalmass="5.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.1"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // After boundmass: a=1.0, b=2.0, total=3.0
        // After settotalmass(5.0): scale=5/3, a=5/3, b=10/3
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 5.0).abs() < 1e-10,
            "total mass should be 5.0, got {total}"
        );
    }

    // ── Include file integration tests ───────────────────────────────

    #[test]
    fn test_load_model_from_file_with_includes() {
        let dir = tempfile::tempdir().expect("tempdir");

        // Main model file
        std::fs::write(
            dir.path().join("main.xml"),
            r#"<mujoco model="included">
                <compiler angle="radian"/>
                <worldbody>
                    <include file="arm.xml"/>
                </worldbody>
                <include file="actuators.xml"/>
            </mujoco>"#,
        )
        .unwrap();

        // Body definitions
        std::fs::write(
            dir.path().join("arm.xml"),
            r#"<wrapper>
                <body name="link1" pos="0 0 0.5">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                </body>
            </wrapper>"#,
        )
        .unwrap();

        // Actuator definitions
        std::fs::write(
            dir.path().join("actuators.xml"),
            r#"<mujoco>
                <actuator>
                    <motor joint="j1" name="m1"/>
                </actuator>
            </mujoco>"#,
        )
        .unwrap();

        let model = load_model_from_file(dir.path().join("main.xml")).expect("should load");
        assert_eq!(model.name, "included");
        assert!(model.nbody >= 2, "should have world + link1");
        assert_eq!(model.njnt, 1, "should have 1 joint");
        assert_eq!(model.nu, 1, "should have 1 actuator");
    }

    #[test]
    fn test_load_model_string_rejects_includes() {
        let result = load_model(
            r#"<mujoco>
                <worldbody>
                    <include file="bodies.xml"/>
                </worldbody>
            </mujoco>"#,
        );
        assert!(result.is_err(), "string API should reject includes");
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("include"),
            "error should mention include: {err}"
        );
    }

    // ── discardvisual / fusestatic tests ─────────────────────────────

    #[test]
    fn test_discardvisual_removes_visual_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Only collision geom should remain
        assert_eq!(model.ngeom, 1, "visual geom should be discarded");
    }

    #[test]
    fn test_discardvisual_false_keeps_all_geoms() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom name="collision" type="sphere" size="0.1" contype="1" conaffinity="1"/>
                        <geom name="visual" type="sphere" size="0.12" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(
            model.ngeom, 2,
            "all geoms should remain when discardvisual=false"
        );
    }

    #[test]
    fn test_fusestatic_merges_jointless_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // static_child (no joints) should be fused into parent
        // parent + world = 2 bodies (static_child merged)
        assert_eq!(
            model.nbody, 2,
            "static child should be fused: nbody={}",
            model.nbody
        );
        // Geoms: parent sphere + fused box = 2
        assert_eq!(model.ngeom, 2, "both geoms should remain after fusion");
    }

    #[test]
    fn test_fusestatic_preserves_jointed_body() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="base" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="link" pos="0 0 0.5">
                            <joint name="j2" type="hinge" axis="0 1 0"/>
                            <geom type="sphere" size="0.08"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Both bodies have joints — no fusion
        assert_eq!(
            model.nbody, 3,
            "jointed bodies should not be fused: nbody={}",
            model.nbody
        );
    }

    #[test]
    fn test_fusestatic_false_preserves_all_bodies() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="static_child" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert_eq!(model.nbody, 3, "fusestatic=false should keep all bodies");
    }

    #[test]
    fn test_fusestatic_orientation_handling() {
        // A static body rotated 90° around Z should rotate its child geom position and orientation.
        // Parent body at origin with a static child rotated 90° around Z at pos [1, 0, 0].
        // Child has a geom at local pos [0.5, 0, 0].
        // After fusion: geom pos should be rotated by 90° Z: [1, 0, 0] + rot90z([0.5, 0, 0])
        //   = [1, 0, 0] + [0, 0.5, 0] = [1, 0.5, 0]
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="parent_geom" type="sphere" size="0.1" mass="1"/>
                        <body name="rotated_static" pos="1 0 0" euler="0 0 90">
                            <geom name="child_geom" type="sphere" size="0.05" pos="0.5 0 0" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // The static body should be fused into parent: 2 bodies (world + parent)
        assert_eq!(model.nbody, 2, "rotated static body should be fused");
        // Parent should have 2 geoms (its own + child's)
        assert_eq!(
            model.body_geom_num[1], 2,
            "parent should have 2 geoms after fusion"
        );

        // Check the fused geom position: should be [1, 0.5, 0] after rotation
        // geom[0] is parent_geom (from worldbody geom processing), geom[1] is child_geom
        let child_geom_idx = 1; // second geom on this body
        let fused_pos = model.geom_pos[child_geom_idx];
        assert!(
            (fused_pos.x - 1.0).abs() < 1e-10,
            "fused geom x should be 1.0, got {}",
            fused_pos.x
        );
        assert!(
            (fused_pos.y - 0.5).abs() < 1e-10,
            "fused geom y should be 0.5 (rotated), got {}",
            fused_pos.y
        );
        assert!(
            fused_pos.z.abs() < 1e-10,
            "fused geom z should be 0, got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_no_rotation_preserves_geom_euler() {
        // A static body with NO rotation but a geom that HAS euler orientation.
        // After fusion the geom euler should still produce the correct orientation.
        // This tests the no-rotation branch: euler must remain valid in parent frame.
        //
        // Setup: static body at pos [0, 0, 1] with identity orientation.
        //        geom at local pos [0, 0, 0] with euler="0 0 90" (45° around Z).
        // After fusion: geom at [0, 0, 1] with same 90° Z rotation.
        // Compare against a model WITHOUT fusestatic to verify identical geom_quat.
        let with_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("fuse model should load");

        let without_fuse = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="false"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="g0" type="sphere" size="0.1" mass="1"/>
                        <body name="static_child" pos="0 0 1">
                            <geom name="g1" type="sphere" size="0.05" pos="0 0 0" euler="0 0 90" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("no-fuse model should load");

        // Find g1's geom index in each model
        // With fusion: 2 bodies (world + parent), g1 is second geom on parent (idx 1)
        // Without fusion: 3 bodies (world + parent + static_child), g1 is first geom on static_child (idx 1)
        assert_eq!(with_fuse.nbody, 2, "fused model should have 2 bodies");
        assert_eq!(without_fuse.nbody, 3, "unfused model should have 3 bodies");

        // Both models should have 2 geoms total (g0 + g1)
        assert_eq!(with_fuse.ngeom, 2);
        assert_eq!(without_fuse.ngeom, 2);

        // Geom quaternions should match — proving euler is preserved correctly through fusion
        let fused_quat = with_fuse.geom_quat[1];
        let unfused_quat = without_fuse.geom_quat[1];
        let diff = (fused_quat.into_inner() - unfused_quat.into_inner()).norm();
        assert!(
            diff < 1e-10,
            "fused geom quat should match unfused: fused={fused_quat:?}, unfused={unfused_quat:?}, diff={diff}"
        );

        // Geom positions should account for body offset
        let fused_pos = with_fuse.geom_pos[1];
        assert!(
            (fused_pos.z - 1.0).abs() < 1e-10,
            "fused geom z should be 1.0 (body offset), got {}",
            fused_pos.z
        );
    }

    #[test]
    fn test_fusestatic_protects_sensor_referenced_body() {
        // A subtreecom sensor references a body by name — that body must not be fused.
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                        <body name="sensor_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05" mass="0.5"/>
                        </body>
                    </body>
                </worldbody>
                <sensor>
                    <subtreecom name="com_sensor" body="sensor_target"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        // sensor_target should NOT be fused because it's referenced by a sensor
        assert_eq!(
            model.nbody, 3,
            "sensor-referenced body should be protected from fusion"
        );
    }

    #[test]
    fn test_fusestatic_protects_actuator_referenced_body() {
        // An adhesion actuator references a body by name — that body must not be fused.
        // Test at the MjcfModel level since body transmission isn't wired to Model yet.
        let mut mjcf = crate::parse_mjcf_str(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom type="sphere" size="0.1"/>
                        <body name="actuator_target" pos="0 0 0.5">
                            <geom type="box" size="0.05 0.05 0.05"/>
                        </body>
                        <body name="unprotected" pos="0 0 1">
                            <geom type="sphere" size="0.05"/>
                        </body>
                    </body>
                </worldbody>
                <actuator>
                    <adhesion name="stick" body="actuator_target" gain="1"/>
                </actuator>
            </mujoco>
            "#,
        )
        .expect("should parse");

        apply_fusestatic(&mut mjcf);

        // actuator_target should still exist, unprotected should be fused
        let parent = &mjcf.worldbody.children[0];
        let child_names: Vec<&str> = parent.children.iter().map(|c| c.name.as_str()).collect();
        assert!(
            child_names.contains(&"actuator_target"),
            "actuator-referenced body should be protected: {child_names:?}"
        );
        assert!(
            !child_names.contains(&"unprotected"),
            "unprotected body should be fused away: {child_names:?}"
        );
    }

    #[test]
    fn test_discardvisual_protects_sensor_referenced_geom() {
        // A visual geom (contype=0, conaffinity=0) referenced by a frame sensor
        // should NOT be discarded.
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" discardvisual="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint name="j1" type="hinge" axis="0 0 1"/>
                        <geom name="collision_geom" type="sphere" size="0.1" mass="1"/>
                        <geom name="visual_ref" type="sphere" size="0.2" contype="0" conaffinity="0"/>
                    </body>
                </worldbody>
                <sensor>
                    <framepos name="vis_sensor" objname="visual_ref"/>
                </sensor>
            </mujoco>
            "#,
        )
        .expect("should load");

        // visual_ref should be kept because it's referenced by a sensor
        assert_eq!(
            model.ngeom, 2,
            "sensor-referenced visual geom should be protected from discard"
        );
    }

    #[test]
    fn test_resolve_asset_path_texturedir() {
        let dir = tempfile::tempdir().unwrap();
        let tex_dir = dir.path().join("textures");
        std::fs::create_dir_all(&tex_dir).unwrap();
        std::fs::write(tex_dir.join("wood.png"), b"fake_texture").unwrap();

        let mut compiler = MjcfCompiler::default();
        compiler.texturedir = Some("textures/".to_string());

        let result =
            resolve_asset_path("wood.png", Some(dir.path()), &compiler, AssetKind::Texture);
        assert!(
            result.is_ok(),
            "texturedir should resolve texture paths: {result:?}"
        );
        assert!(
            result.unwrap().ends_with("textures/wood.png"),
            "should resolve to textures/wood.png"
        );
    }

    #[test]
    fn test_resolve_asset_path_texturedir_vs_meshdir() {
        // meshdir applies to meshes, texturedir applies to textures
        let dir = tempfile::tempdir().unwrap();
        let mesh_dir = dir.path().join("meshes");
        let tex_dir = dir.path().join("textures");
        std::fs::create_dir_all(&mesh_dir).unwrap();
        std::fs::create_dir_all(&tex_dir).unwrap();
        std::fs::write(mesh_dir.join("box.stl"), b"fake_mesh").unwrap();
        std::fs::write(tex_dir.join("wood.png"), b"fake_texture").unwrap();

        let mut compiler = MjcfCompiler::default();
        compiler.meshdir = Some("meshes/".to_string());
        compiler.texturedir = Some("textures/".to_string());

        let mesh_result =
            resolve_asset_path("box.stl", Some(dir.path()), &compiler, AssetKind::Mesh);
        let tex_result =
            resolve_asset_path("wood.png", Some(dir.path()), &compiler, AssetKind::Texture);
        assert!(mesh_result.is_ok(), "mesh should resolve via meshdir");
        assert!(tex_result.is_ok(), "texture should resolve via texturedir");
        assert!(
            mesh_result.unwrap().to_string_lossy().contains("meshes"),
            "mesh path should use meshdir"
        );
        assert!(
            tex_result.unwrap().to_string_lossy().contains("textures"),
            "texture path should use texturedir"
        );
    }

    // =========================================================================
    // Frame expansion + childclass tests (Spec #19, 27 acceptance criteria)
    // =========================================================================

    // AC1: Frame position
    #[test]
    fn test_ac01_frame_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                        <frame pos="1 0 0">
                            <geom name="fg" type="sphere" size="0.1" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom 0 is the body's own geom, geom 1 is the frame's geom
        assert_eq!(model.geom_pos.len(), 2);
        let pos = model.geom_pos[1];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC2: Frame rotation
    #[test]
    fn test_ac02_frame_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "90deg Z rotation should map (1,0,0) to (0,1,0), got {pos:?}"
        );
    }

    // AC3: Frame position + rotation
    #[test]
    fn test_ac03_frame_pos_plus_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom name="fg" type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        // frame_pos + frame_rot * geom_pos = (1,0,0) + (0,1,0) = (1,1,0)
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC4: Nested frames
    #[test]
    fn test_ac04_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <geom name="fg" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 0.0)).norm() < 1e-10,
            "expected (1,1,0), got {pos:?}"
        );
    }

    // AC5: 3-deep nested frames
    #[test]
    fn test_ac05_three_deep_nested_frames() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <frame pos="0 1 0">
                                <frame pos="0 0 1">
                                    <geom name="fg" type="sphere" size="0.1"/>
                                </frame>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
            "expected (1,1,1), got {pos:?}"
        );
    }

    // AC6: Frame wrapping body
    #[test]
    fn test_ac06_frame_wrapping_body() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="parent">
                        <frame pos="2 0 0">
                            <body name="child" pos="1 0 0">
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // body_pos for child should be (3,0,0)
        // body 0 = world, body 1 = parent, body 2 = child
        assert_eq!(model.body_pos.len(), 3);
        let child_pos = model.body_pos[2];
        assert!(
            (child_pos - Vector3::new(3.0, 0.0, 0.0)).norm() < 1e-10,
            "body_pos should be (3,0,0), got {child_pos:?}"
        );
    }

    // AC7: Frame with fromto geom
    #[test]
    fn test_ac07_frame_fromto_geom() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // fromto endpoints become (1,0,0)-(1,0,1), pos = midpoint (1, 0, 0.5)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-6,
            "expected midpoint (1,0,0.5), got {pos:?}"
        );
    }

    // AC8: Frame with fromto geom + rotation
    #[test]
    fn test_ac08_frame_fromto_rotation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 90 0">
                            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.05"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // 90deg Y rotation maps Z to X. Endpoints: (0,0,0)-(1,0,0)
        // Geom pos ≈ midpoint (0.5, 0, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-6,
            "expected midpoint (0.5,0,0), got {pos:?}"
        );
    }

    // AC9: Frame with site
    #[test]
    fn test_ac09_frame_with_site() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="0.5 0 0">
                            <site name="s" pos="0 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.5, 0.0, 0.0)).norm() < 1e-10,
            "site should be at (0.5,0,0), got {pos:?}"
        );
    }

    // AC10: Empty frame
    #[test]
    fn test_ac10_empty_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Model should load without error, geom at (0,0,0) unaffected
        assert_eq!(model.geom_pos.len(), 1);
        assert!(
            (model.geom_pos[0]).norm() < 1e-10,
            "geom should be at origin (empty frame has no children)"
        );
    }

    // AC11: Frame at worldbody level
    #[test]
    fn test_ac11_frame_at_worldbody() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <frame pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </frame>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-10,
            "worldbody frame geom should be at (1,0,0), got {pos:?}"
        );
    }

    // AC12: Frame with only orientation (no pos)
    #[test]
    fn test_ac12_frame_only_orientation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC13: Frame with only position (no orientation)
    #[test]
    fn test_ac13_frame_only_position() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="3 0 0">
                            <geom type="sphere" size="0.1" pos="0 2 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(3.0, 2.0, 0.0)).norm() < 1e-10,
            "expected (3,2,0), got {pos:?}"
        );
    }

    // AC14: Frame with xyaxes
    #[test]
    fn test_ac14_frame_xyaxes() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame xyaxes="0 1 0 -1 0 0">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // xyaxes "0 1 0 -1 0 0" = 90deg Z rotation. Geom at (0, 1, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC15: Frame with zaxis
    #[test]
    fn test_ac15_frame_zaxis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame zaxis="1 0 0">
                            <geom type="sphere" size="0.1" pos="0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // zaxis "1 0 0" maps Z to X. Geom at local (0,0,1) → body-relative (1, 0, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-6,
            "expected (1,0,0), got {pos:?}"
        );
    }

    // AC16: Frame with axisangle
    #[test]
    fn test_ac16_frame_axisangle() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame axisangle="0 0 1 90">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // 90deg about Z. Geom at (0, 1, 0)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // AC17: Joint inside frame = error
    #[test]
    fn test_ac17_joint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <joint name="j" type="hinge"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "joint inside frame should error");
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("not allowed inside <frame>"),
            "error should mention invalid element: {err}"
        );
    }

    // AC18: Freejoint inside frame = error
    #[test]
    fn test_ac18_freejoint_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <freejoint/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "freejoint inside frame should error");
    }

    // AC19: Inertial inside frame = error
    #[test]
    fn test_ac19_inertial_inside_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame>
                            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(result.is_err(), "inertial inside frame should error");
    }

    // AC20: Childclass on body
    #[test]
    fn test_ac20_childclass_on_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom should inherit "red" defaults (contype = 7)
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'red', got {}",
            model.geom_contype[0]
        );
    }

    // AC21: Childclass override by explicit class
    #[test]
    fn test_ac21_childclass_override_by_explicit() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="blue">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <geom class="blue" type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom uses explicit class "blue" (contype=3), not childclass "red" (contype=7)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should use explicit class 'blue' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC22: Childclass on frame
    #[test]
    fn test_ac22_childclass_on_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="red">
                        <geom contype="7"/>
                    </default>
                    <default class="green">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="red">
                        <frame childclass="green">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom should inherit "green" (frame's childclass overrides body's)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit frame childclass 'green' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC23: Childclass inheritance through body hierarchy
    #[test]
    fn test_ac23_childclass_inheritance_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <joint damping="5.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="parent" childclass="robot">
                        <body name="child" pos="0 0 1">
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Inner body's joint should inherit class "robot" from parent
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'robot', got {}",
            model.dof_damping[0]
        );
    }

    // AC24: Childclass on child body overrides parent
    #[test]
    fn test_ac24_childclass_child_overrides_parent() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="A">
                        <geom contype="7"/>
                    </default>
                    <default class="B">
                        <geom contype="3"/>
                    </default>
                </default>
                <worldbody>
                    <body name="outer" childclass="A">
                        <body name="inner" childclass="B">
                            <geom type="sphere" size="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Geom inherits "B" (child body's childclass overrides parent's)
        assert_eq!(
            model.geom_contype[0], 3,
            "geom should inherit child body childclass 'B' contype=3, got {}",
            model.geom_contype[0]
        );
    }

    // AC25: Regression - no frames, no childclass
    #[test]
    fn test_ac25_regression_no_frames() {
        // Simple model without frames or childclass should work identically
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" pos="1 2 3">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s" pos="0.5 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2); // world + b
        assert_eq!(model.ngeom, 1);
        let body_pos = model.body_pos[1];
        assert!((body_pos - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-10);
    }

    // AC26: Geom orientation composition
    #[test]
    fn test_ac26_geom_orientation_composition() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <geom type="sphere" size="0.1" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // geom orientation = frame_quat * geom_quat
        // Both euler values resolved, then composed
        // Just verify model loads and geom has a non-identity orientation
        let quat = model.geom_quat[0];
        // The composed rotation should NOT be identity
        let is_identity = (quat.w - 1.0).abs() < 1e-6
            && quat.i.abs() < 1e-6
            && quat.j.abs() < 1e-6
            && quat.k.abs() < 1e-6;
        assert!(!is_identity, "composed rotation should not be identity");
    }

    // AC27: Frame + angle="radian"
    #[test]
    fn test_ac27_frame_radian_mode() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 1.5707963">
                            <geom type="sphere" size="0.1" pos="1 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        let pos = model.geom_pos[0];
        // Same as degree case: geom at (0, 1, 0)
        assert!(
            (pos - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-4,
            "expected (0,1,0), got {pos:?}"
        );
    }

    // ========================================================================
    // AC28–AC35: Childclass edge cases (item #20)
    // ========================================================================

    // AC28: Childclass referencing a nested default class (default hierarchy)
    #[test]
    fn test_ac28_childclass_nested_default_hierarchy() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="robot">
                        <geom contype="5"/>
                        <default class="arm">
                            <geom conaffinity="3"/>
                        </default>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="arm">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // "arm" inherits contype=5 from parent "robot", and sets conaffinity=3 directly
        assert_eq!(
            model.geom_contype[0], 5,
            "geom should inherit contype=5 from parent class 'robot' through 'arm', got {}",
            model.geom_contype[0]
        );
        assert_eq!(
            model.geom_conaffinity[0], 3,
            "geom should get conaffinity=3 from class 'arm', got {}",
            model.geom_conaffinity[0]
        );
    }

    // AC29: Childclass applies to geom, joint, AND site simultaneously
    #[test]
    fn test_ac29_childclass_multi_element() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="R">
                        <geom contype="7"/>
                        <joint damping="5.0"/>
                        <site size="0.05"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b" childclass="R">
                        <joint name="j" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                        <site name="s"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(
            model.geom_contype[0], 7,
            "geom should inherit contype=7 from childclass 'R', got {}",
            model.geom_contype[0]
        );
        assert!(
            (model.dof_damping[0] - 5.0).abs() < 1e-10,
            "joint should inherit damping=5.0 from childclass 'R', got {}",
            model.dof_damping[0]
        );
        // Site size default is [0.01]; class "R" sets it to [0.05]
        let site_size = model.site_size[0];
        assert!(
            (site_size.x - 0.05).abs() < 1e-10,
            "site should inherit size=0.05 from childclass 'R', got {}",
            site_size.x
        );
    }

    // AC30: 3-level deep propagation without override
    #[test]
    fn test_ac30_childclass_3level_propagation() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="X">
                        <joint damping="3.0"/>
                    </default>
                </default>
                <worldbody>
                    <body name="top" childclass="X">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 3.0).abs() < 1e-10,
            "joint at 3rd level should inherit damping=3.0 from top's childclass 'X', got {}",
            model.dof_damping[0]
        );
    }

    // AC31: 3-level with mid-hierarchy override
    #[test]
    fn test_ac31_childclass_3level_mid_override() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <default>
                    <default class="A"><joint damping="1.0"/></default>
                    <default class="B"><joint damping="9.0"/></default>
                </default>
                <worldbody>
                    <body name="top" childclass="A">
                        <geom type="sphere" size="0.1"/>
                        <body name="mid" childclass="B" pos="0 0 1">
                            <geom type="sphere" size="0.1"/>
                            <body name="bot" pos="0 0 1">
                                <joint name="j" type="hinge"/>
                                <geom type="sphere" size="0.1"/>
                            </body>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert!(
            (model.dof_damping[0] - 9.0).abs() < 1e-10,
            "joint should inherit damping=9.0 from mid's childclass 'B' (not top's 'A'), got {}",
            model.dof_damping[0]
        );
    }

    // AC32: Nested frames with childclass inheritance and override
    #[test]
    fn test_ac32_nested_frames_childclass() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="F1"><geom contype="4"/></default>
                    <default class="F2"><geom contype="8"/></default>
                </default>
                <worldbody>
                    <body name="b">
                        <frame childclass="F1">
                            <frame>
                                <geom name="g1" type="sphere" size="0.1"/>
                            </frame>
                            <frame childclass="F2">
                                <geom name="g2" type="sphere" size="0.1"/>
                            </frame>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // g1 inherits outer frame's F1 through inner frame (no override)
        assert_eq!(
            model.geom_contype[0], 4,
            "g1 should inherit contype=4 from outer frame childclass 'F1', got {}",
            model.geom_contype[0]
        );
        // g2 gets F2 from inner frame override
        assert_eq!(
            model.geom_contype[1], 8,
            "g2 should get contype=8 from inner frame childclass 'F2', got {}",
            model.geom_contype[1]
        );
    }

    // AC33: childclass="nonexistent" on body produces error
    #[test]
    fn test_ac33_childclass_nonexistent_body_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b" childclass="nonexistent">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='nonexistent' should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("nonexistent"),
            "error message should mention 'nonexistent', got: {err_msg}"
        );
    }

    // AC34: Body with childclass but no child elements succeeds
    #[test]
    fn test_ac34_childclass_empty_body() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="empty"><geom contype="5"/></default>
                </default>
                <worldbody>
                    <body name="b" childclass="empty" pos="0 0 1">
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        assert_eq!(model.nbody, 2, "should have world + b");
        assert_eq!(model.ngeom, 0, "body has no geoms");
    }

    // AC35: childclass="ghost" on frame produces error
    #[test]
    fn test_ac35_childclass_nonexistent_frame_error() {
        let result = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame childclass="ghost">
                            <geom type="sphere" size="0.1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        );
        assert!(
            result.is_err(),
            "childclass='ghost' on frame should produce an error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("ghost"),
            "error message should mention 'ghost', got: {err_msg}"
        );
    }

    // ========================================================================
    // Additional coverage: fusestatic + orientation fields, discardvisual + frame
    // ========================================================================

    #[test]
    fn test_fusestatic_geom_with_xyaxes() {
        // A geom with xyaxes orientation on a static body (no joints).
        // fusestatic should correctly resolve xyaxes via resolve_orientation
        // before composing with the parent body's transform.
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="1 0 0" euler="0 0 90">
                            <geom type="sphere" size="0.1" xyaxes="0 1 0 -1 0 0"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // static_child is fused into parent. The geom's xyaxes (90° Z rotation)
        // should compose with the body's 90° Z euler rotation = 180° total Z rotation.
        // Position: body_quat * (0,0,0) + (1,0,0) = (1,0,0).
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-4,
            "expected (1,0,0), got {pos:?}"
        );
    }

    #[test]
    fn test_fusestatic_site_with_axisangle() {
        // A site with axisangle orientation on a static body.
        // fusestatic should correctly resolve axisangle via resolve_orientation.
        let model = load_model(
            r#"
            <mujoco>
                <compiler fusestatic="true"/>
                <worldbody>
                    <body name="parent">
                        <joint type="hinge"/>
                        <body name="static_child" pos="0 0 1">
                            <site name="s" pos="0 0 0" axisangle="0 0 1 90"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Site should be at (0,0,1) after fusion. No body rotation so just translation.
        let pos = model.site_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-4,
            "expected (0,0,1), got {pos:?}"
        );
    }

    #[test]
    fn test_discardvisual_geom_inside_frame() {
        // A visual-only geom (contype=0, conaffinity=0) inside a frame
        // should be discarded by discardvisual after frame expansion.
        let model = load_model(
            r#"
            <mujoco>
                <compiler discardvisual="true"/>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0">
                            <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
                        </frame>
                        <geom type="sphere" size="0.1" pos="0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Only the non-visual geom should survive
        assert_eq!(
            model.geom_pos.len(),
            1,
            "discardvisual should remove visual geom from frame"
        );
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(0.0, 0.0, 0.0)).norm() < 1e-4,
            "surviving geom should be at origin"
        );
    }

    #[test]
    fn test_fromto_geom_in_frame_with_pos_and_rotation() {
        // fromto geom inside a frame that has both position and rotation.
        // Endpoints should be transformed through the full frame transform.
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame pos="1 0 0" euler="0 0 90">
                            <geom type="capsule" size="0.05" fromto="0 0 0 0 0 1"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .unwrap();
        // Frame: pos=(1,0,0), 90° Z rotation.
        // fromto "0 0 0 0 0 1": from=(0,0,0), to=(0,0,1)
        // Transformed from: (1,0,0) + rot*(0,0,0) = (1,0,0)
        // Transformed to:   (1,0,0) + rot*(0,0,1) = (1,0,1)
        // Midpoint: (1, 0, 0.5)
        let pos = model.geom_pos[0];
        assert!(
            (pos - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-4,
            "expected (1,0,0.5), got {pos:?}"
        );
    }

    // =========================================================================
    // Site orientation tests (item #21)
    // =========================================================================

    /// AC1: Site euler orientation with default angle="degree".
    #[test]
    fn test_site_euler_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90° X deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC2: Site euler orientation with explicit angle="radian".
    #[test]
    fn test_site_euler_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="{pi_2} 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler pi/2 X rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC3: Site euler with non-default eulerseq="ZYX".
    #[test]
    fn test_site_euler_orientation_zyx_eulerseq() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="0.3 0.2 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler ZYX: got {got:?}, expected {expected:?}"
        );
    }

    /// AC4: Site axisangle orientation with default angle="degree".
    #[test]
    fn test_site_axisangle_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 1 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle 90° Z deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC5: Site axisangle orientation with angle="radian".
    #[test]
    fn test_site_axisangle_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 1 0 {pi_2}"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle pi/2 Y rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC6: Site xyaxes orientation with orthogonal inputs (90° Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_orthogonal() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="0 1 0 -1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // x=(0,1,0), y=(-1,0,0), z=cross(x,y)=(0,0,1) → 90° about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site xyaxes orthogonal 90° Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC7: Site xyaxes with non-orthogonal inputs (Gram-Schmidt → 45° Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_gram_schmidt() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="1 1 0 0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Gram-Schmidt: x=norm(1,1,0)=(1/√2, 1/√2, 0), y orthogonalized → 45° about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 45.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site xyaxes Gram-Schmidt 45° Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC8: Site zaxis general direction (Z→X = 90° about Y).
    #[test]
    fn test_site_zaxis_orientation_general() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Minimal rotation from (0,0,1) to (1,0,0): 90° about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis Z→X (90° Y): got {got:?}, expected {expected:?}"
        );
    }

    /// AC9: Site zaxis parallel to default Z → identity.
    #[test]
    fn test_site_zaxis_orientation_parallel_identity() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis (0,0,1) identity: got {got:?}, expected {expected:?}"
        );
    }

    /// AC10: Site zaxis anti-parallel (0,0,-1) → 180° about X.
    #[test]
    fn test_site_zaxis_orientation_antiparallel() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 -1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Anti-parallel: fallback axis = X, angle = atan2(0, -1) = PI → 180° about X
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis anti-parallel 180° X: got {got:?}, expected {expected:?}"
        );
    }

    /// AC11: Site quat orientation (regression for pre-#19 behavior).
    #[test]
    fn test_site_quat_orientation_regression() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" quat="0.7071068 0 0.7071068 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Direct quaternion (wxyz): 90° about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site quat regression 90° Y: got {got:?}, expected {expected:?}"
        );
    }

    /// AC12: Site orientation with default class (defaults provide type/size,
    /// element provides orientation — no interference).
    #[test]
    fn test_site_orientation_with_default_class() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="sensor_site">
                        <site type="cylinder" size="0.02 0.01"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" class="sensor_site" euler="0 0 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Type from default class
        assert_eq!(
            model.site_type[0],
            GeomType::Cylinder,
            "site type should come from default class"
        );

        // Orientation from element (not interfered by defaults)
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90° Z with default class: got {got:?}, expected {expected:?}"
        );
    }

    /// AC13: Orientation priority — euler takes precedence over quat when both specified.
    #[test]
    fn test_site_orientation_priority_euler_over_quat() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0" quat="1 0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // euler wins over quat per priority chain. quat=(1,0,0,0) is identity;
        // euler="90 0 0" is 90° X. Result must be 90° X, NOT identity.
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "euler should take priority over quat: got {got:?}, expected {expected:?}"
        );
    }

    /// AC14: Site orientation composed with frame rotation.
    #[test]
    fn test_site_orientation_in_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <site name="s" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Frame: 90° Z, Site: 90° X. Composed: frame_q * site_q
        let frame_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let site_q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let expected = frame_q * site_q;
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site in frame composed: got {got:?}, expected {expected:?}"
        );
    }

    /// AC15: Site axisangle with non-unit axis (normalization).
    #[test]
    fn test_site_axisangle_non_unit_axis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 3 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Axis (0,0,3) normalizes to (0,0,1) → same as axisangle="0 0 1 90"
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle non-unit axis: got {got:?}, expected {expected:?}"
        );
    }

    /// AC16: Site zaxis with non-unit direction (normalization, parallel to Z).
    #[test]
    fn test_site_zaxis_non_unit_direction() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // (0,0,5) normalizes to (0,0,1) → parallel to default Z → identity
        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis non-unit parallel: got {got:?}, expected {expected:?}"
        );
    }
}
