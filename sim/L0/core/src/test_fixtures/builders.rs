//! Reusable Model-construction helpers for `test_fixtures`.
//!
//! Internal to `test_fixtures` — these helpers compose into the fixture
//! functions in [`super::particles`], [`super::pendulums`], etc. They
//! mirror the body/joint/dof/geom/site/actuator/sensor field-pushing
//! pattern already established in [`crate::types::model_factories`]
//! (`Model::n_link_pendulum`, `Model::free_body`).
//!
//! # Construction order invariant
//!
//! Within a fixture, callers must add a body's joints/dofs/geoms/sites
//! immediately after the body, before adding the next body. This keeps
//! `body_jnt_adr`/`body_geom_adr`/etc. ranges contiguous in MuJoCo's
//! convention. Add helpers update the most-recently-added body's
//! `body_*_num` counter, so the typical pattern is:
//!
//! ```ignore
//! let body = add_body(...);
//! add_hinge_joint(model, body, ...);
//! add_capsule_geom(model, body, ...);
//! // now next body
//! let body2 = add_body(...);
//! ```
//!
//! After all bodies/joints/etc. are added, call [`finalize`] to run the
//! standard pre-computation sequence (mirrors what the existing
//! `Model::n_link_pendulum` factory does internally).

use nalgebra::{DVector, UnitQuaternion, Vector3};

use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};
use crate::types::Model;
use crate::types::enums::{
    ActuatorDynamics, ActuatorTransmission, BiasType, GainType, GeomType, InterpolationType,
    MjJointType, MjObjectType, MjSensorDataType, MjSensorType,
};

/// Add a body whose frame origin is at `pos` in its parent's frame,
/// with given mass and diagonal principal inertia. The COM offset
/// (`ipos`) defaults to zero. Updates `nbody`. Initializes the body's
/// joint / dof / geom counts to zero (subsequent add helpers increment
/// them).
///
/// Returns the new body id (1-based; world is 0).
// Body construction is parallel to MuJoCo's body element fields (parent, pos,
// mass, inertia, ipos…); a struct-spec API would be heavier than the value of
// flattening these into named arguments at the fixture call site.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_body(
    model: &mut Model,
    parent_id: usize,
    name: &str,
    pos: Vector3<f64>,
    mass: f64,
    inertia: Vector3<f64>,
    ipos: Vector3<f64>,
) -> usize {
    let body_id = model.nbody;
    model.nbody += 1;

    model.body_parent.push(parent_id);
    // For top-level bodies whose parent is world, root id is the body itself
    // (matches MuJoCo's tree-rooting convention). For deeper bodies, inherit
    // the parent's root id.
    let rootid = if parent_id == 0 {
        body_id
    } else {
        model.body_rootid[parent_id]
    };
    model.body_rootid.push(rootid);
    model.body_jnt_adr.push(model.njnt);
    model.body_jnt_num.push(0);
    model.body_dof_adr.push(model.nv);
    model.body_dof_num.push(0);
    model.body_geom_adr.push(model.ngeom);
    model.body_geom_num.push(0);

    model.body_pos.push(pos);
    model.body_quat.push(UnitQuaternion::identity());
    model.body_ipos.push(ipos);
    model.body_iquat.push(UnitQuaternion::identity());
    model.body_mass.push(mass);
    model.body_inertia.push(inertia);
    model.body_name.push(Some(name.to_string()));
    model.body_subtreemass.push(0.0); // populated by compute_invweight0
    model.body_mocapid.push(None);
    model.body_gravcomp.push(0.0);
    model.body_invweight0.push([0.0; 2]);
    model.body_plugin.push(None);

    body_id
}

/// Add a mocap body (kinematic, no joints, no DOF, position driven by
/// `data.mocap_pos`/`data.mocap_quat`). MuJoCo requires mocap bodies to
/// be direct children of worldbody. Updates `nbody` and `nmocap`.
pub(super) fn add_mocap_body(model: &mut Model, name: &str, pos: Vector3<f64>) -> usize {
    let body_id = model.nbody;
    model.nbody += 1;

    model.body_parent.push(0);
    model.body_rootid.push(body_id);
    model.body_jnt_adr.push(model.njnt);
    model.body_jnt_num.push(0);
    model.body_dof_adr.push(model.nv);
    model.body_dof_num.push(0);
    model.body_geom_adr.push(model.ngeom);
    model.body_geom_num.push(0);

    model.body_pos.push(pos);
    model.body_quat.push(UnitQuaternion::identity());
    model.body_ipos.push(Vector3::zeros());
    model.body_iquat.push(UnitQuaternion::identity());
    model.body_mass.push(0.0);
    model.body_inertia.push(Vector3::zeros());
    model.body_name.push(Some(name.to_string()));
    model.body_subtreemass.push(0.0);
    model.body_mocapid.push(Some(model.nmocap));
    model.nmocap += 1;
    model.body_gravcomp.push(0.0);
    model.body_invweight0.push([0.0; 2]);
    model.body_plugin.push(None);

    body_id
}

/// Common scalar-joint setup shared by Hinge and Slide.
///
/// Increments `body_jnt_num` and `body_dof_num` on `body_id`, plus the
/// model-level `njnt`/`nq`/`nv`. Returns the new joint id.
// Internal helper — joint setup needs every field MuJoCo's hinge/slide accept
// (axis, stiffness, damping, springref, limited, range, jnt_type). A spec
// struct would just rename the same fields at the call site.
#[allow(clippy::too_many_arguments)]
fn add_scalar_joint(
    model: &mut Model,
    body_id: usize,
    name: &str,
    jnt_type: MjJointType,
    axis: Vector3<f64>,
    stiffness: f64,
    damping: f64,
    springref: f64,
    limited: bool,
    range: (f64, f64),
) -> usize {
    debug_assert!(matches!(jnt_type, MjJointType::Hinge | MjJointType::Slide));
    let jnt_id = model.njnt;
    let qpos_adr = model.nq;
    let dof_id = model.nv;
    model.njnt += 1;
    model.nq += 1;
    model.nv += 1;

    model.jnt_type.push(jnt_type);
    model.jnt_body.push(body_id);
    // qpos_adr / dof_adr are absolute indices into qpos / qvel, not joint
    // ids — they only coincide for fixtures without prior free / ball
    // joints.
    model.jnt_qpos_adr.push(qpos_adr);
    model.jnt_dof_adr.push(dof_id);
    model.jnt_pos.push(Vector3::zeros());
    model.jnt_axis.push(axis);
    model.jnt_limited.push(limited);
    model.jnt_range.push(range);
    model.jnt_stiffness.push(stiffness);
    model.jnt_springref.push(springref);
    model.jnt_damping.push(damping);
    model.jnt_armature.push(0.0);
    model.jnt_solref.push(DEFAULT_SOLREF);
    model.jnt_solimp.push(DEFAULT_SOLIMP);
    model.jnt_name.push(Some(name.to_string()));
    model.jnt_group.push(0);
    model.jnt_actgravcomp.push(false);
    model.jnt_margin.push(0.0);
    model.qpos_spring.push(springref);

    model.dof_body.push(body_id);
    model.dof_jnt.push(jnt_id);
    let dof_parent = if dof_id == 0 { None } else { Some(dof_id - 1) };
    model.dof_parent.push(dof_parent);
    model.dof_armature.push(0.0);
    model.dof_damping.push(damping);
    model.dof_frictionloss.push(0.0);
    model.dof_solref.push(DEFAULT_SOLREF);
    model.dof_solimp.push(DEFAULT_SOLIMP);
    model.dof_invweight0.push(0.0);

    model.body_jnt_num[body_id] += 1;
    model.body_dof_num[body_id] += 1;

    jnt_id
}

/// Add a hinge joint to `body_id`, rotating about `axis`.
// Same wide-arg shape as add_scalar_joint — see justification there.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_hinge_joint(
    model: &mut Model,
    body_id: usize,
    name: &str,
    axis: Vector3<f64>,
    stiffness: f64,
    damping: f64,
    springref: f64,
    limited: bool,
    range: (f64, f64),
) -> usize {
    add_scalar_joint(
        model,
        body_id,
        name,
        MjJointType::Hinge,
        axis,
        stiffness,
        damping,
        springref,
        limited,
        range,
    )
}

/// Add a slide (prismatic) joint to `body_id`, translating along `axis`.
// Same wide-arg shape as add_scalar_joint — see justification there.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_slide_joint(
    model: &mut Model,
    body_id: usize,
    name: &str,
    axis: Vector3<f64>,
    stiffness: f64,
    damping: f64,
    springref: f64,
) -> usize {
    add_scalar_joint(
        model,
        body_id,
        name,
        MjJointType::Slide,
        axis,
        stiffness,
        damping,
        springref,
        false,
        (-1e10, 1e10),
    )
}

/// Add a free (6-DOF) joint to `body_id`. Free joints contribute 7 to
/// `nq` (3 pos + 4 quat) and 6 to `nv` (3 lin + 3 ang).
pub(super) fn add_freejoint(model: &mut Model, body_id: usize, name: &str) -> usize {
    let jnt_id = model.njnt;
    let dof_adr = model.nv;
    let qpos_adr = model.nq;
    model.njnt += 1;
    model.nq += 7;
    model.nv += 6;

    model.jnt_type.push(MjJointType::Free);
    model.jnt_body.push(body_id);
    model.jnt_qpos_adr.push(qpos_adr);
    model.jnt_dof_adr.push(dof_adr);
    model.jnt_pos.push(Vector3::zeros());
    model.jnt_axis.push(Vector3::z());
    model.jnt_limited.push(false);
    model.jnt_range.push((-1e10, 1e10));
    model.jnt_stiffness.push(0.0);
    model.jnt_springref.push(0.0);
    model.jnt_damping.push(0.0);
    model.jnt_armature.push(0.0);
    model.jnt_solref.push(DEFAULT_SOLREF);
    model.jnt_solimp.push(DEFAULT_SOLIMP);
    model.jnt_name.push(Some(name.to_string()));
    model.jnt_group.push(0);
    model.jnt_actgravcomp.push(false);
    model.jnt_margin.push(0.0);
    model
        .qpos_spring
        .extend_from_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

    for i in 0..6 {
        model.dof_body.push(body_id);
        model.dof_jnt.push(jnt_id);
        model
            .dof_parent
            .push(if i == 0 { None } else { Some(dof_adr + i - 1) });
        model.dof_armature.push(0.0);
        model.dof_damping.push(0.0);
        model.dof_frictionloss.push(0.0);
        model.dof_solref.push(DEFAULT_SOLREF);
        model.dof_solimp.push(DEFAULT_SOLIMP);
        model.dof_invweight0.push(0.0);
    }

    model.body_jnt_num[body_id] += 1;
    model.body_dof_num[body_id] += 6;

    jnt_id
}

/// Common geom field defaults shared across primitive shapes. Increments
/// `ngeom` and the body's `body_geom_num`. Returns the new geom id.
// Internal helper — primitive geom setup needs every shape-relevant field
// (geom_type/pos/quat/size/contact). Wrapping in a struct would only push the
// arg count from this call site to the wrapping caller.
#[allow(clippy::too_many_arguments)]
fn push_geom(
    model: &mut Model,
    body_id: usize,
    name: Option<&str>,
    geom_type: GeomType,
    pos: Vector3<f64>,
    quat: UnitQuaternion<f64>,
    size: Vector3<f64>,
    contact: bool,
) -> usize {
    let geom_id = model.ngeom;
    model.ngeom += 1;

    model.geom_type.push(geom_type);
    model.geom_body.push(body_id);
    model.geom_pos.push(pos);
    model.geom_quat.push(quat);
    model.geom_size.push(size);
    model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
    model.geom_condim.push(3);
    let (contype, conaffinity) = if contact { (1, 1) } else { (0, 0) };
    model.geom_contype.push(contype);
    model.geom_conaffinity.push(conaffinity);
    model.geom_margin.push(0.0);
    model.geom_gap.push(0.0);
    model.geom_priority.push(0);
    model.geom_solmix.push(1.0);
    model.geom_solimp.push(DEFAULT_SOLIMP);
    model.geom_solref.push(DEFAULT_SOLREF);
    model.geom_fluid.push([0.0; 12]);
    model.geom_name.push(name.map(str::to_string));
    model.geom_rbound.push(0.0);
    model.geom_aabb.push([0.0, 0.0, 0.0, 1e6, 1e6, 1e6]);
    model.geom_mesh.push(None);
    model.geom_hfield.push(None);
    model.geom_shape.push(None);
    model.geom_group.push(0);
    model.geom_rgba.push([0.5, 0.5, 0.5, 1.0]);
    model.geom_user.push(vec![]);
    model.geom_plugin.push(None);

    model.body_geom_num[body_id] += 1;
    geom_id
}

/// Add a sphere geom of `radius` at body-local position `pos`.
pub(super) fn add_sphere_geom(
    model: &mut Model,
    body_id: usize,
    name: Option<&str>,
    pos: Vector3<f64>,
    radius: f64,
    contact: bool,
) -> usize {
    push_geom(
        model,
        body_id,
        name,
        GeomType::Sphere,
        pos,
        UnitQuaternion::identity(),
        Vector3::new(radius, 0.0, 0.0),
        contact,
    )
}

/// Add a capsule geom from `from` to `to` (both body-local) with given
/// `radius`. Mirrors MuJoCo's `fromto` shorthand: the geom is stored
/// with `pos` = midpoint, `size` = (radius, half_length, 0), and `quat`
/// rotates the geom's local Z axis onto the segment direction
/// (MuJoCo's capsule convention is "axis along local Z, half-length
/// stored in `size[1]`").
pub(super) fn add_capsule_geom(
    model: &mut Model,
    body_id: usize,
    name: Option<&str>,
    from: Vector3<f64>,
    to: Vector3<f64>,
    radius: f64,
    contact: bool,
) -> usize {
    let segment = to - from;
    let half_length = segment.norm() * 0.5;
    let pos = (from + to) * 0.5;
    // Aim the capsule axis (its local Z) along the segment direction.
    let quat = if half_length > 0.0 {
        let dir = segment.normalize();
        let z = Vector3::z();
        if (dir - z).norm() < 1e-12 {
            UnitQuaternion::identity()
        } else if (dir + z).norm() < 1e-12 {
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
        } else {
            UnitQuaternion::rotation_between(&z, &dir).unwrap_or_else(UnitQuaternion::identity)
        }
    } else {
        UnitQuaternion::identity()
    };
    push_geom(
        model,
        body_id,
        name,
        GeomType::Capsule,
        pos,
        quat,
        Vector3::new(radius, half_length, 0.0),
        contact,
    )
}

/// Add a box geom centered at body-local `pos` with `half_extents`
/// (size = half-edge along each principal axis).
pub(super) fn add_box_geom(
    model: &mut Model,
    body_id: usize,
    name: Option<&str>,
    pos: Vector3<f64>,
    half_extents: Vector3<f64>,
    contact: bool,
) -> usize {
    push_geom(
        model,
        body_id,
        name,
        GeomType::Box,
        pos,
        UnitQuaternion::identity(),
        half_extents,
        contact,
    )
}

/// Add a ground plane geom on the world body at z=0.
pub(super) fn add_ground_plane(model: &mut Model) -> usize {
    let id = push_geom(
        model,
        0,
        Some("ground"),
        GeomType::Plane,
        Vector3::zeros(),
        UnitQuaternion::identity(),
        Vector3::new(40.0, 40.0, 0.1),
        true,
    );
    // Ground gets a slight transparent rgba to match the existing
    // `Model::add_ground_plane` convention.
    if let Some(rgba) = model.geom_rgba.last_mut() {
        *rgba = [0.5, 0.5, 0.5, 0.3];
    }
    id
}

/// Add a site (massless reference frame) on `body_id`.
pub(super) fn add_site(
    model: &mut Model,
    body_id: usize,
    name: &str,
    pos: Vector3<f64>,
    size: f64,
) -> usize {
    let site_id = model.nsite;
    model.nsite += 1;

    model.site_body.push(body_id);
    model.site_type.push(GeomType::Sphere);
    model.site_pos.push(pos);
    model.site_quat.push(UnitQuaternion::identity());
    model.site_size.push(Vector3::new(size, 0.0, 0.0));
    model.site_name.push(Some(name.to_string()));
    model.site_group.push(0);
    model.site_rgba.push([0.5, 0.5, 0.5, 1.0]);

    site_id
}

/// Common actuator field defaults. Increments `nu`. `ctrlrange` of
/// `None` means unlimited.
// Internal helper — actuator setup needs MuJoCo's full gain/bias/dyntype
// parameter set (gainprm, biasprm, dynprm + their tag enums). A spec struct
// would just rename the same fields.
#[allow(clippy::too_many_arguments)]
fn push_actuator(
    model: &mut Model,
    joint_id: usize,
    name: Option<&str>,
    gear: f64,
    ctrlrange: Option<(f64, f64)>,
    gainprm: [f64; 9],
    biasprm: [f64; 9],
    dynprm: [f64; 10],
    gaintype: GainType,
    biastype: BiasType,
    dyntype: ActuatorDynamics,
) -> usize {
    let act_id = model.nu;
    model.nu += 1;

    model.actuator_trntype.push(ActuatorTransmission::Joint);
    model.actuator_dyntype.push(dyntype);
    model.actuator_trnid.push([joint_id, usize::MAX]);
    let mut gear6 = [0.0; 6];
    gear6[0] = gear;
    model.actuator_gear.push(gear6);
    // MuJoCo encodes "unlimited" by setting ctrlrange to a huge sentinel
    // pair; the `actuation` step clamps to the range unconditionally and
    // an effectively-infinite range is a no-op clamp.
    let range = ctrlrange.unwrap_or((-1e10, 1e10));
    model.actuator_ctrlrange.push(range);
    model.actuator_forcerange.push((-1e10, 1e10));
    model.actuator_name.push(name.map(str::to_string));
    model.actuator_act_adr.push(model.na);
    model.actuator_act_num.push(0);
    model.actuator_gaintype.push(gaintype);
    model.actuator_biastype.push(biastype);
    model.actuator_dynprm.push(dynprm);
    model.actuator_gainprm.push(gainprm);
    model.actuator_biasprm.push(biasprm);
    model.actuator_lengthrange.push((0.0, 0.0));
    model.actuator_acc0.push(0.0);
    model.actuator_actlimited.push(false);
    model.actuator_actrange.push((0.0, 0.0));
    model.actuator_actearly.push(false);
    model.actuator_cranklength.push(0.0);
    model.actuator_nsample.push(0);
    model.actuator_interp.push(InterpolationType::Zoh);
    model.actuator_historyadr.push(0);
    model.actuator_delay.push(0.0);
    model.actuator_group.push(0);

    act_id
}

/// Add a motor actuator (gain=1, no bias, no dynamics).
pub(super) fn add_motor(
    model: &mut Model,
    joint_id: usize,
    name: Option<&str>,
    gear: f64,
    ctrlrange: Option<(f64, f64)>,
) -> usize {
    let mut gainprm = [0.0; 9];
    gainprm[0] = 1.0;
    push_actuator(
        model,
        joint_id,
        name,
        gear,
        ctrlrange,
        gainprm,
        [0.0; 9],
        [0.0; 10],
        GainType::Fixed,
        BiasType::None,
        ActuatorDynamics::None,
    )
}

/// Add a `general` actuator with explicit gain/bias parameter arrays.
/// Used by the thermostat-style `general` actuators where `gainprm[0]=0`
/// makes the actuator inert (control input is observable but produces
/// no joint force) — the test harness reads `data.ctrl` separately.
// Same wide-arg shape as push_actuator — see justification there.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_general_actuator(
    model: &mut Model,
    joint_id: usize,
    name: Option<&str>,
    gainprm: [f64; 9],
    biasprm: [f64; 9],
    ctrlrange: Option<(f64, f64)>,
) -> usize {
    push_actuator(
        model,
        joint_id,
        name,
        1.0,
        ctrlrange,
        gainprm,
        biasprm,
        [0.0; 10],
        GainType::Fixed,
        BiasType::None,
        ActuatorDynamics::None,
    )
}

/// Add a scalar joint sensor (`JointPos` or `JointVel`) on a hinge/slide
/// joint. Returns the new sensor id.
fn add_scalar_joint_sensor(
    model: &mut Model,
    joint_id: usize,
    name: &str,
    sensor_type: MjSensorType,
    datatype: MjSensorDataType,
) -> usize {
    let sensor_id = model.nsensor;
    let adr = model.nsensordata;
    model.nsensor += 1;
    model.nsensordata += 1; // dim=1 for JointPos/JointVel

    model.sensor_type.push(sensor_type);
    model.sensor_datatype.push(datatype);
    model.sensor_objtype.push(MjObjectType::Joint);
    model.sensor_objid.push(joint_id);
    model.sensor_reftype.push(MjObjectType::None);
    model.sensor_refid.push(0);
    model.sensor_adr.push(adr);
    model.sensor_dim.push(1);
    model.sensor_noise.push(0.0);
    model.sensor_cutoff.push(0.0);
    model.sensor_name.push(Some(name.to_string()));
    model.sensor_nsample.push(0);
    model.sensor_interp.push(InterpolationType::Zoh);
    model.sensor_historyadr.push(0);
    model.sensor_delay.push(0.0);
    model.sensor_interval.push((0.0, 0.0));

    sensor_id
}

/// Add a `jointpos` sensor on a hinge/slide joint.
pub(super) fn add_jointpos_sensor(model: &mut Model, joint_id: usize, name: &str) -> usize {
    add_scalar_joint_sensor(
        model,
        joint_id,
        name,
        MjSensorType::JointPos,
        MjSensorDataType::Position,
    )
}

/// Add a `jointvel` sensor on a hinge/slide joint.
pub(super) fn add_jointvel_sensor(model: &mut Model, joint_id: usize, name: &str) -> usize {
    add_scalar_joint_sensor(
        model,
        joint_id,
        name,
        MjSensorType::JointVel,
        MjSensorDataType::Velocity,
    )
}

/// Set scalar simulation options. Convenience wrapper for the common
/// "fixture preamble" of timestep + gravity + integrator + (optional)
/// contact-disable flag.
pub(super) fn set_options(
    model: &mut Model,
    timestep: f64,
    gravity: Vector3<f64>,
    integrator: crate::types::enums::Integrator,
    contact_disabled: bool,
) {
    model.timestep = timestep;
    model.gravity = gravity;
    model.integrator = integrator;
    if contact_disabled {
        model.disableflags |= crate::types::DISABLE_CONTACT;
    }
}

/// Run the standard structural pre-computation sequence. Mirrors what
/// the existing `Model::n_link_pendulum` factory does internally
/// (`compute_ancestors`, `compute_implicit_params`,
/// `compute_qld_csr_metadata`). Sets `qpos0` from the accumulated
/// `qpos_spring`-style state — for our fixtures, qpos0 starts at the
/// joint's reference configuration (springref for hinge/slide,
/// `[1,0,0,0]` for ball, `[0,0,0,1,0,0,0]` for free).
pub(super) fn finalize(model: &mut Model) {
    // qpos0 mirrors qpos_spring (reference/spring rest configuration).
    if model.qpos_spring.is_empty() {
        model.qpos0 = DVector::zeros(0);
    } else {
        model.qpos0 = DVector::from_vec(model.qpos_spring.clone());
    }
    model.compute_ancestors();
    model.compute_implicit_params();
    model.compute_qld_csr_metadata();
}
