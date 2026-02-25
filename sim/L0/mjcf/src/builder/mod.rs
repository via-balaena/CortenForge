//! MJCF-to-Model builder — top-level orchestration.
//!
//! Converts parsed [`MjcfModel`] into a physics-ready [`Model`] (sim-core).
//! Sub-modules handle individual MJCF element types (geom, joint, body, etc.);
//! this module defines the [`ModelBuilder`] struct, the public entry points
//! ([`model_from_mjcf`], [`load_model`], [`load_model_from_file`]), and the
//! orchestration sequence that wires everything together.

pub mod actuator;
pub mod asset;
pub mod body;
mod build;
pub mod compiler;
pub mod contact;
pub mod equality;
pub mod flex;
pub mod fluid;
pub mod frame;
pub mod geom;
mod init;
pub mod joint;
pub mod mass;
pub mod mesh;
pub mod orientation;
pub mod sensor;
pub mod tendon;

use nalgebra::{DVector, UnitQuaternion, Vector3};
use sim_core::{
    ActuatorDynamics, ActuatorTransmission, BiasType, ContactPair, DISABLE_ACTUATION,
    DISABLE_AUTORESET, DISABLE_CLAMPCTRL, DISABLE_CONSTRAINT, DISABLE_CONTACT, DISABLE_DAMPER,
    DISABLE_EQUALITY, DISABLE_EULERDAMP, DISABLE_FILTERPARENT, DISABLE_FRICTIONLOSS,
    DISABLE_GRAVITY, DISABLE_ISLAND, DISABLE_LIMIT, DISABLE_MIDPHASE, DISABLE_NATIVECCD,
    DISABLE_REFSAFE, DISABLE_SENSOR, DISABLE_SPRING, DISABLE_WARMSTART, ENABLE_ENERGY,
    ENABLE_FWDINV, ENABLE_INVDISCRETE, ENABLE_MULTICCD, ENABLE_OVERRIDE, ENABLE_SLEEP,
    EqualityType, GainType, GeomType, Integrator, Keyframe, MjJointType, MjObjectType,
    MjSensorDataType, MjSensorType, Model, SleepPolicy, SolverType, TendonType, WrapType,
};
use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::Arc;

use crate::defaults::DefaultResolver;
use crate::error::Result;
use crate::types::{
    MjcfCompiler, MjcfConeType, MjcfIntegrator, MjcfKeyframe, MjcfModel, MjcfOption, MjcfSolverType,
};

use self::compiler::{apply_discardvisual, apply_fusestatic};
use self::frame::{expand_frames, validate_childclass_references};

/// Default solref parameters [timeconst, dampratio] (MuJoCo defaults).
///
/// - timeconst = 0.02s: 50 Hz natural frequency
/// - dampratio = 1.0: critical damping
pub const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];

/// Default solimp parameters `[d0, d_width, width, midpoint, power]` (MuJoCo defaults).
///
/// - d0 = 0.9: initial impedance (close to rigid)
/// - d_width = 0.95: impedance at full penetration
/// - width = 0.001: penetration depth at which impedance transitions
/// - midpoint = 0.5, power = 2.0: sigmoid transition shape
pub const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

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
        let mesh = builder.resolver.apply_to_mesh(mesh);
        builder.process_mesh(&mesh, base_path)?;
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

    // Process flex deformable bodies (after mass pipeline, before build)
    builder.process_flex_bodies(&mjcf.flex)?;

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
pub struct ModelBuilder {
    // Model name
    pub(crate) name: String,

    // Default class resolver (applies <default> class attributes to elements)
    pub(crate) resolver: DefaultResolver,

    // Compiler settings (controls angle conversion, euler sequence, etc.)
    pub(crate) compiler: MjcfCompiler,

    // Dimensions (computed during building)
    pub(crate) nq: usize,
    pub(crate) nv: usize,

    // Body arrays
    pub(crate) body_parent: Vec<usize>,
    pub(crate) body_rootid: Vec<usize>,
    pub(crate) body_jnt_adr: Vec<usize>,
    pub(crate) body_jnt_num: Vec<usize>,
    pub(crate) body_dof_adr: Vec<usize>,
    pub(crate) body_dof_num: Vec<usize>,
    pub(crate) body_geom_adr: Vec<usize>,
    pub(crate) body_geom_num: Vec<usize>,
    pub(crate) body_pos: Vec<Vector3<f64>>,
    pub(crate) body_quat: Vec<UnitQuaternion<f64>>,
    pub(crate) body_ipos: Vec<Vector3<f64>>,
    pub(crate) body_iquat: Vec<UnitQuaternion<f64>>,
    pub(crate) body_mass: Vec<f64>,
    pub(crate) body_inertia: Vec<Vector3<f64>>,
    pub(crate) body_name: Vec<Option<String>>,
    pub(crate) body_mocapid: Vec<Option<usize>>,
    /// Per-body sleep policy from MJCF `sleep` attribute (None = default Auto).
    pub(crate) body_sleep_policy: Vec<Option<SleepPolicy>>,
    /// Per-body gravity compensation factor (0=none, 1=full).
    pub(crate) body_gravcomp: Vec<f64>,
    pub(crate) nmocap: usize,

    // Joint arrays
    pub(crate) jnt_type: Vec<MjJointType>,
    pub(crate) jnt_body: Vec<usize>,
    pub(crate) jnt_qpos_adr: Vec<usize>,
    pub(crate) jnt_dof_adr: Vec<usize>,
    pub(crate) jnt_pos: Vec<Vector3<f64>>,
    pub(crate) jnt_axis: Vec<Vector3<f64>>,
    pub(crate) jnt_limited: Vec<bool>,
    pub(crate) jnt_range: Vec<(f64, f64)>,
    pub(crate) jnt_stiffness: Vec<f64>,
    pub(crate) jnt_springref: Vec<f64>,
    pub(crate) jnt_damping: Vec<f64>,
    pub(crate) jnt_armature: Vec<f64>,
    pub(crate) jnt_solref: Vec<[f64; 2]>,
    pub(crate) jnt_solimp: Vec<[f64; 5]>,
    pub(crate) jnt_name: Vec<Option<String>>,
    /// Visualization group per joint (0–5).
    pub(crate) jnt_group: Vec<i32>,
    /// Per-joint: gravcomp routes through qfrc_actuator instead of qfrc_passive.
    pub(crate) jnt_actgravcomp: Vec<bool>,

    // DOF arrays
    pub(crate) dof_body: Vec<usize>,
    pub(crate) dof_jnt: Vec<usize>,
    pub(crate) dof_parent: Vec<Option<usize>>,
    pub(crate) dof_armature: Vec<f64>,
    pub(crate) dof_damping: Vec<f64>,
    pub(crate) dof_frictionloss: Vec<f64>,
    pub(crate) dof_solref: Vec<[f64; 2]>,
    pub(crate) dof_solimp: Vec<[f64; 5]>,

    // Geom arrays
    pub(crate) geom_type: Vec<GeomType>,
    pub(crate) geom_body: Vec<usize>,
    pub(crate) geom_pos: Vec<Vector3<f64>>,
    pub(crate) geom_quat: Vec<UnitQuaternion<f64>>,
    pub(crate) geom_size: Vec<Vector3<f64>>,
    pub(crate) geom_friction: Vec<Vector3<f64>>,
    pub(crate) geom_condim: Vec<i32>,
    pub(crate) geom_contype: Vec<u32>,
    pub(crate) geom_conaffinity: Vec<u32>,
    pub(crate) geom_name: Vec<Option<String>>,
    /// Mesh index for each geom (`None` for non-mesh geoms).
    pub(crate) geom_mesh: Vec<Option<usize>>,
    /// Solver reference parameters for each geom [timeconst, dampratio].
    pub(crate) geom_solref: Vec<[f64; 2]>,
    /// Solver impedance parameters for each geom [d0, d_width, width, midpoint, power].
    pub(crate) geom_solimp: Vec<[f64; 5]>,
    /// Contact priority per geom (MuJoCo mj_contactParam).
    pub(crate) geom_priority: Vec<i32>,
    /// Solver mixing weight per geom (MuJoCo mj_contactParam).
    pub(crate) geom_solmix: Vec<f64>,
    /// Contact margin per geom (broadphase + narrow-phase activation distance).
    pub(crate) geom_margin: Vec<f64>,
    /// Contact gap per geom (buffer zone within margin).
    pub(crate) geom_gap: Vec<f64>,
    /// Visualization group per geom (0–5).
    pub(crate) geom_group: Vec<i32>,
    /// RGBA color per geom [r, g, b, a].
    pub(crate) geom_rgba: Vec<[f64; 4]>,
    /// Fluid interaction data per geom: 12 elements matching MuJoCo's `geom_fluid`.
    /// See §40 spec for layout: [interaction_coef, C_blunt, C_slender, C_ang, C_K, C_M,
    /// virtual_mass[3], virtual_inertia[3]].
    pub(crate) geom_fluid: Vec<[f64; 12]>,

    // Mesh arrays (built from MJCF <asset><mesh> elements)
    /// Name-to-index lookup for mesh assets.
    pub(crate) mesh_name_to_id: HashMap<String, usize>,
    /// Mesh names (for Model).
    pub(crate) mesh_name: Vec<String>,
    /// Triangle mesh data with prebuilt BVH (Arc for cheap cloning).
    pub(crate) mesh_data: Vec<Arc<sim_core::mesh::TriangleMeshData>>,

    // Height field arrays (built from MJCF <asset><hfield> elements)
    /// Name-to-index lookup for hfield assets.
    pub(crate) hfield_name_to_id: HashMap<String, usize>,
    /// Hfield names (for Model).
    pub(crate) hfield_name: Vec<String>,
    /// Height field terrain data (Arc for cheap cloning).
    pub(crate) hfield_data: Vec<Arc<sim_core::HeightFieldData>>,
    /// Original MuJoCo size [x, y, z_top, z_bottom] per hfield asset.
    pub(crate) hfield_size: Vec<[f64; 4]>,
    /// Hfield index per geom (like geom_mesh).
    pub(crate) geom_hfield: Vec<Option<usize>>,
    /// SDF index per geom (like geom_mesh). Always `None` from MJCF — SDF geoms
    /// are populated programmatically.
    pub(crate) geom_sdf: Vec<Option<usize>>,

    // Site arrays
    pub(crate) site_body: Vec<usize>,
    pub(crate) site_type: Vec<GeomType>,
    pub(crate) site_pos: Vec<Vector3<f64>>,
    pub(crate) site_quat: Vec<UnitQuaternion<f64>>,
    pub(crate) site_size: Vec<Vector3<f64>>,
    pub(crate) site_name: Vec<Option<String>>,
    /// Visualization group per site (0–5).
    pub(crate) site_group: Vec<i32>,
    /// RGBA color per site [r, g, b, a].
    pub(crate) site_rgba: Vec<[f64; 4]>,

    // World frame tracking (for free joint qpos0 initialization)
    // Stores accumulated world positions during tree traversal
    pub(crate) body_world_pos: Vec<Vector3<f64>>,
    pub(crate) body_world_quat: Vec<UnitQuaternion<f64>>,

    // Actuator arrays
    pub(crate) actuator_trntype: Vec<ActuatorTransmission>,
    pub(crate) actuator_dyntype: Vec<ActuatorDynamics>,
    pub(crate) actuator_trnid: Vec<[usize; 2]>,
    pub(crate) actuator_gear: Vec<[f64; 6]>,
    pub(crate) actuator_ctrlrange: Vec<(f64, f64)>,
    pub(crate) actuator_forcerange: Vec<(f64, f64)>,
    pub(crate) actuator_name: Vec<Option<String>>,
    pub(crate) actuator_act_adr: Vec<usize>,
    pub(crate) actuator_act_num: Vec<usize>,
    pub(crate) actuator_gaintype: Vec<GainType>,
    pub(crate) actuator_biastype: Vec<BiasType>,
    pub(crate) actuator_dynprm: Vec<[f64; 3]>,
    pub(crate) actuator_gainprm: Vec<[f64; 9]>,
    pub(crate) actuator_biasprm: Vec<[f64; 9]>,
    pub(crate) actuator_lengthrange: Vec<(f64, f64)>,
    pub(crate) actuator_acc0: Vec<f64>,
    pub(crate) actuator_actlimited: Vec<bool>,
    pub(crate) actuator_actrange: Vec<(f64, f64)>,
    pub(crate) actuator_actearly: Vec<bool>,

    // Total activation states (sum of actuator_act_num)
    pub(crate) na: usize,

    // Options
    pub(crate) timestep: f64,
    pub(crate) gravity: Vector3<f64>,
    pub(crate) wind: Vector3<f64>,
    pub(crate) magnetic: Vector3<f64>,
    pub(crate) density: f64,
    pub(crate) viscosity: f64,
    pub(crate) solver_iterations: usize,
    pub(crate) solver_tolerance: f64,
    pub(crate) impratio: f64,
    pub(crate) regularization: f64,
    pub(crate) friction_smoothing: f64,
    pub(crate) cone: u8,
    pub(crate) ls_iterations: usize,
    pub(crate) ls_tolerance: f64,
    pub(crate) noslip_iterations: usize,
    pub(crate) noslip_tolerance: f64,
    pub(crate) disableflags: u32,
    pub(crate) enableflags: u32,
    pub(crate) disableactuator: u32,
    pub(crate) actuator_group: Vec<i32>,
    pub(crate) o_margin: f64,
    pub(crate) o_solref: [f64; 2],
    pub(crate) o_solimp: [f64; 5],
    pub(crate) o_friction: [f64; 5],
    pub(crate) integrator: Integrator,
    pub(crate) solver_type: SolverType,
    pub(crate) sleep_tolerance: f64,

    // qpos0 values (built as we process joints)
    pub(crate) qpos0_values: Vec<f64>,

    // Name to index lookups (for actuator wiring)
    pub(crate) joint_name_to_id: HashMap<String, usize>,
    pub(crate) body_name_to_id: HashMap<String, usize>,
    pub(crate) site_name_to_id: HashMap<String, usize>,
    pub(crate) geom_name_to_id: HashMap<String, usize>,

    // Actuator name lookup (for sensor wiring)
    pub(crate) actuator_name_to_id: HashMap<String, usize>,

    // Tendon arrays (populated by process_tendons)
    pub(crate) tendon_name_to_id: HashMap<String, usize>,
    pub(crate) tendon_type: Vec<TendonType>,
    pub(crate) tendon_range: Vec<(f64, f64)>,
    pub(crate) tendon_limited: Vec<bool>,
    pub(crate) tendon_stiffness: Vec<f64>,
    pub(crate) tendon_damping: Vec<f64>,
    pub(crate) tendon_frictionloss: Vec<f64>,
    pub(crate) tendon_solref_fri: Vec<[f64; 2]>,
    pub(crate) tendon_solimp_fri: Vec<[f64; 5]>,
    pub(crate) tendon_lengthspring: Vec<[f64; 2]>,
    pub(crate) tendon_length0: Vec<f64>,
    pub(crate) tendon_name: Vec<Option<String>>,
    pub(crate) tendon_solref: Vec<[f64; 2]>,
    pub(crate) tendon_solimp: Vec<[f64; 5]>,
    pub(crate) tendon_num: Vec<usize>,
    pub(crate) tendon_adr: Vec<usize>,
    /// Visualization group per tendon (0–5).
    pub(crate) tendon_group: Vec<i32>,
    /// RGBA color per tendon [r, g, b, a].
    pub(crate) tendon_rgba: Vec<[f64; 4]>,
    pub(crate) wrap_type: Vec<WrapType>,
    pub(crate) wrap_objid: Vec<usize>,
    pub(crate) wrap_prm: Vec<f64>,
    pub(crate) wrap_sidesite: Vec<usize>,

    // Equality constraint arrays
    pub(crate) eq_type: Vec<EqualityType>,
    pub(crate) eq_obj1id: Vec<usize>,
    pub(crate) eq_obj2id: Vec<usize>,
    pub(crate) eq_data: Vec<[f64; 11]>,
    pub(crate) eq_active: Vec<bool>,
    pub(crate) eq_solimp: Vec<[f64; 5]>,
    pub(crate) eq_solref: Vec<[f64; 2]>,
    pub(crate) eq_name: Vec<Option<String>>,

    // Sensor accumulation fields (populated by process_sensors)
    pub(crate) sensor_type: Vec<MjSensorType>,
    pub(crate) sensor_datatype: Vec<MjSensorDataType>,
    pub(crate) sensor_objtype: Vec<MjObjectType>,
    pub(crate) sensor_objid: Vec<usize>,
    pub(crate) sensor_reftype: Vec<MjObjectType>,
    pub(crate) sensor_refid: Vec<usize>,
    pub(crate) sensor_adr: Vec<usize>,
    pub(crate) sensor_dim: Vec<usize>,
    pub(crate) sensor_noise: Vec<f64>,
    pub(crate) sensor_cutoff: Vec<f64>,
    pub(crate) sensor_name_list: Vec<Option<String>>,
    pub(crate) nsensor: usize,
    pub(crate) nsensordata: usize,

    // Contact pairs / excludes (populated by process_contact)
    pub(crate) contact_pairs: Vec<ContactPair>,
    pub(crate) contact_pair_set: HashSet<(usize, usize)>,
    pub(crate) contact_excludes: HashSet<(usize, usize)>,

    // Flex deformable body arrays (populated by process_flex_bodies)
    pub(crate) nflex: usize,
    pub(crate) nflexvert: usize,
    pub(crate) nflexedge: usize,
    pub(crate) nflexelem: usize,
    pub(crate) nflexhinge: usize,
    pub(crate) flex_dim: Vec<usize>,
    pub(crate) flex_vertadr: Vec<usize>,
    pub(crate) flex_vertnum: Vec<usize>,
    pub(crate) flex_damping: Vec<f64>,
    pub(crate) flex_friction: Vec<Vector3<f64>>,
    pub(crate) flex_condim: Vec<i32>,
    pub(crate) flex_margin: Vec<f64>,
    pub(crate) flex_gap: Vec<f64>,
    pub(crate) flex_priority: Vec<i32>,
    pub(crate) flex_solmix: Vec<f64>,
    pub(crate) flex_solref: Vec<[f64; 2]>,
    pub(crate) flex_solimp: Vec<[f64; 5]>,
    pub(crate) flex_edge_solref: Vec<[f64; 2]>,
    pub(crate) flex_edge_solimp: Vec<[f64; 5]>,
    pub(crate) flex_bend_stiffness: Vec<f64>,
    pub(crate) flex_bend_damping: Vec<f64>,
    pub(crate) flex_young: Vec<f64>,
    pub(crate) flex_poisson: Vec<f64>,
    pub(crate) flex_thickness: Vec<f64>,
    pub(crate) flex_density: Vec<f64>,
    pub(crate) flex_group: Vec<i32>,
    pub(crate) flex_contype: Vec<u32>,
    pub(crate) flex_conaffinity: Vec<u32>,
    pub(crate) flex_selfcollide: Vec<bool>,
    pub(crate) flex_edgestiffness: Vec<f64>,
    pub(crate) flex_edgedamping: Vec<f64>,
    pub(crate) flexvert_qposadr: Vec<usize>,
    pub(crate) flexvert_dofadr: Vec<usize>,
    pub(crate) flexvert_mass: Vec<f64>,
    pub(crate) flexvert_invmass: Vec<f64>,
    pub(crate) flexvert_radius: Vec<f64>,
    pub(crate) flexvert_flexid: Vec<usize>,
    pub(crate) flexvert_bodyid: Vec<usize>,
    pub(crate) flexvert_initial_pos: Vec<Vector3<f64>>,
    pub(crate) flexedge_vert: Vec<[usize; 2]>,
    pub(crate) flexedge_length0: Vec<f64>,
    pub(crate) flexedge_flexid: Vec<usize>,
    pub(crate) flexelem_data: Vec<usize>,
    pub(crate) flexelem_dataadr: Vec<usize>,
    pub(crate) flexelem_datanum: Vec<usize>,
    pub(crate) flexelem_volume0: Vec<f64>,
    pub(crate) flexelem_flexid: Vec<usize>,
    pub(crate) flexhinge_vert: Vec<[usize; 4]>,
    pub(crate) flexhinge_angle0: Vec<f64>,
    pub(crate) flexhinge_flexid: Vec<usize>,
}

impl ModelBuilder {
    fn set_options(&mut self, option: &MjcfOption) {
        self.timestep = option.timestep;
        self.gravity = option.gravity;
        self.solver_iterations = option.iterations;
        self.solver_tolerance = option.tolerance;
        self.impratio = option.impratio;
        self.regularization = option.regularization;
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
        self.cone = match option.cone {
            MjcfConeType::Pyramidal => 0,
            MjcfConeType::Elliptic => 1,
        };
        self.magnetic = option.magnetic;
        self.wind = option.wind;
        self.density = option.density;
        self.viscosity = option.viscosity;
        self.sleep_tolerance = option.sleep_tolerance;
        self.disableactuator = option.actuatorgroupdisable;

        // S10-stub: Wire override parameters from parsed <option>.
        if option.o_margin >= 0.0 {
            self.o_margin = option.o_margin;
        }
        if let Some(sr) = option.o_solref {
            self.o_solref = sr;
        }
        if let Some(si) = option.o_solimp {
            self.o_solimp = si;
        }
        if let Some(fr) = option.o_friction {
            self.o_friction = fr;
        }

        // Wire all flags from parsed MJCF to Model bitfields.
        apply_flags(&option.flag, &mut self.disableflags, &mut self.enableflags);
    }
}

/// Convert parsed `MjcfFlag` booleans into Model disable/enable bitfields.
///
/// Set or clear an enable flag bit based on a boolean.
#[inline]
fn set_enable(enableflags: &mut u32, bit: u32, value: bool) {
    if value {
        *enableflags |= bit;
    } else {
        *enableflags &= !bit;
    }
}

/// Disable flags: field `true` = feature enabled = bit NOT set.
/// Enable flags: field `true` = feature enabled = bit SET.
fn apply_flags(flag: &crate::types::MjcfFlag, disableflags: &mut u32, enableflags: &mut u32) {
    // Disable flags: field true = feature enabled = bit NOT set.
    if !flag.constraint {
        *disableflags |= DISABLE_CONSTRAINT;
    }
    if !flag.equality {
        *disableflags |= DISABLE_EQUALITY;
    }
    if !flag.frictionloss {
        *disableflags |= DISABLE_FRICTIONLOSS;
    }
    if !flag.limit {
        *disableflags |= DISABLE_LIMIT;
    }
    if !flag.contact {
        *disableflags |= DISABLE_CONTACT;
    }
    if !flag.spring {
        *disableflags |= DISABLE_SPRING;
    }
    if !flag.damper {
        *disableflags |= DISABLE_DAMPER;
    }
    if !flag.gravity {
        *disableflags |= DISABLE_GRAVITY;
    }
    if !flag.clampctrl {
        *disableflags |= DISABLE_CLAMPCTRL;
    }
    if !flag.warmstart {
        *disableflags |= DISABLE_WARMSTART;
    }
    if !flag.filterparent {
        *disableflags |= DISABLE_FILTERPARENT;
    }
    if !flag.actuation {
        *disableflags |= DISABLE_ACTUATION;
    }
    if !flag.refsafe {
        *disableflags |= DISABLE_REFSAFE;
    }
    if !flag.sensor {
        *disableflags |= DISABLE_SENSOR;
    }
    if !flag.midphase {
        *disableflags |= DISABLE_MIDPHASE;
    }
    if !flag.eulerdamp {
        *disableflags |= DISABLE_EULERDAMP;
    }
    if !flag.autoreset {
        *disableflags |= DISABLE_AUTORESET;
    }
    if !flag.nativeccd {
        *disableflags |= DISABLE_NATIVECCD;
    }
    if !flag.island {
        *disableflags |= DISABLE_ISLAND;
    }

    // Enable flags: field true = bit SET, field false = bit CLEARED.
    // Unlike disable flags (which only OR in bits), enable flags must
    // also clear bits to handle defaults like energy=true being overridden
    // by <flag energy="false"/>.
    set_enable(enableflags, ENABLE_OVERRIDE, flag.override_contacts);
    set_enable(enableflags, ENABLE_ENERGY, flag.energy);
    set_enable(enableflags, ENABLE_FWDINV, flag.fwdinv);
    set_enable(enableflags, ENABLE_INVDISCRETE, flag.invdiscrete);
    set_enable(enableflags, ENABLE_MULTICCD, flag.multiccd);
    set_enable(enableflags, ENABLE_SLEEP, flag.sleep);

    // Stub-only flags: subsystem not yet implemented.
    // Warn so users don't silently get no-op behavior.
    if flag.fwdinv {
        tracing::warn!(
            "ENABLE_FWDINV set but inverse dynamics (§52) not implemented — flag has no effect"
        );
    }
    if flag.invdiscrete {
        tracing::warn!(
            "ENABLE_INVDISCRETE set but inverse dynamics (§52) not implemented — flag has no effect"
        );
    }
    if flag.multiccd {
        tracing::warn!("ENABLE_MULTICCD set but CCD (§50) not implemented — flag has no effect");
    }
    if !flag.nativeccd {
        // default is true (enabled); warn when user disables it
        tracing::warn!("DISABLE_NATIVECCD set but CCD (§50) not implemented — flag has no effect");
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::{load_model, load_model_from_file, model_from_mjcf};

    /// Helper to create a simple STL file for testing.
    fn create_test_stl(path: &std::path::Path) {
        use mesh_types::{IndexedMesh, Vertex};

        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);

        mesh_io::save_mesh(&mesh, path).expect("Failed to save test STL");
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

    // -- Include file integration tests --

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
}
