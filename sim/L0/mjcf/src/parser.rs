//! MJCF XML parser.
//!
//! Parses MJCF XML into the intermediate representation types.

use nalgebra::{Vector3, Vector4};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

/// Safe axis normalization with Z fallback for zero-length vectors.
#[inline]
fn safe_normalize_axis(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { Vector3::z() }
}
use crate::types::{
    MjcfActuator, MjcfActuatorDefaults, MjcfActuatorType, MjcfBody, MjcfConeType, MjcfConnect,
    MjcfContact, MjcfContactExclude, MjcfContactPair, MjcfDefault, MjcfDistance, MjcfEquality,
    MjcfFlag, MjcfGeom, MjcfGeomDefaults, MjcfGeomType, MjcfHfield, MjcfInertial, MjcfIntegrator,
    MjcfJacobianType, MjcfJoint, MjcfJointDefaults, MjcfJointEquality, MjcfJointType, MjcfKeyframe,
    MjcfMesh, MjcfMeshDefaults, MjcfModel, MjcfOption, MjcfPairDefaults, MjcfSensor,
    MjcfSensorDefaults, MjcfSensorType, MjcfSite, MjcfSiteDefaults, MjcfSkin, MjcfSkinBone,
    MjcfSkinVertex, MjcfSolverType, MjcfTendon, MjcfTendonDefaults, MjcfTendonType, MjcfWeld,
    SpatialPathElement,
};

/// Parse an MJCF string into a model.
///
/// # Errors
///
/// Returns an error if the XML is malformed or missing required elements.
pub fn parse_mjcf_str(xml: &str) -> Result<MjcfModel> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    parse_mjcf_reader(&mut reader)
}

/// Parse MJCF from a reader.
fn parse_mjcf_reader<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfModel> {
    let mut buf = Vec::new();
    let mut model: Option<MjcfModel> = None;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"mujoco" => {
                model = Some(parse_mujoco(reader, e)?);
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    model.ok_or_else(|| MjcfError::missing_element("mujoco", "MJCF document"))
}

/// Parse the mujoco root element and its children.
fn parse_mujoco<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfModel> {
    let name = get_attribute_opt(start, "model").unwrap_or_else(|| "unnamed".to_string());
    let mut model = MjcfModel::new(name);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"option" => {
                        model.option = parse_option(reader, e)?;
                    }
                    b"default" => {
                        let defaults = parse_default(reader, e, None)?;
                        model.defaults.extend(defaults);
                    }
                    b"asset" => {
                        let (meshes, hfields) = parse_asset(reader)?;
                        model.meshes = meshes;
                        model.hfields = hfields;
                    }
                    b"worldbody" => {
                        model.worldbody = parse_worldbody(reader)?;
                    }
                    b"actuator" => {
                        let actuators = parse_actuators(reader)?;
                        model.actuators = actuators;
                    }
                    b"equality" => {
                        model.equality = parse_equality(reader)?;
                    }
                    b"deformable" => {
                        // Deformable section contains skin elements
                        let skins = parse_deformable(reader)?;
                        model.skins = skins;
                    }
                    b"tendon" => {
                        let tendons = parse_tendons(reader)?;
                        model.tendons = tendons;
                    }
                    b"sensor" => {
                        let sensors = parse_sensors(reader)?;
                        model.sensors = sensors;
                    }
                    b"contact" => {
                        model.contact = parse_contact(reader)?;
                    }
                    b"keyframe" => {
                        model.keyframes = parse_keyframes(reader)?;
                    }
                    // Skip other elements
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                // Handle self-closing elements
                if e.name().as_ref() == b"option" {
                    model.option = parse_option_attrs(e)?;
                } else if e.name().as_ref() == b"contact" {
                    model.contact = MjcfContact::default();
                } else if e.name().as_ref() == b"keyframe" {
                    // Empty <keyframe/> — no keyframes defined. model.keyframes is
                    // already Vec::new() from Default, so nothing to do.
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"mujoco" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in mujoco".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(model)
}

/// Parse option element.
fn parse_option<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfOption> {
    let mut option = parse_option_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                // Handle flag element
                if e.name().as_ref() == b"flag" {
                    option.flag = parse_flag_attrs(e)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"option" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in option".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(option)
}

/// Parse option attributes only.
fn parse_option_attrs(e: &BytesStart) -> Result<MjcfOption> {
    let mut option = MjcfOption::default();

    // Core simulation
    if let Some(ts) = parse_float_attr(e, "timestep") {
        option.timestep = ts;
    }
    if let Some(integrator) = get_attribute_opt(e, "integrator") {
        if let Some(int) = MjcfIntegrator::from_str(&integrator) {
            option.integrator = int;
        }
    }

    // Solver configuration
    if let Some(solver) = get_attribute_opt(e, "solver") {
        if let Some(s) = MjcfSolverType::from_str(&solver) {
            option.solver = s;
        }
    }
    if let Some(iter) = parse_int_attr(e, "iterations") {
        option.iterations = iter.max(0) as usize;
    }
    if let Some(tol) = parse_float_attr(e, "tolerance") {
        option.tolerance = tol;
    }
    if let Some(ls_iter) = parse_int_attr(e, "ls_iterations") {
        option.ls_iterations = ls_iter.max(0) as usize;
    }
    if let Some(ls_tol) = parse_float_attr(e, "ls_tolerance") {
        option.ls_tolerance = ls_tol;
    }
    if let Some(noslip) = parse_int_attr(e, "noslip_iterations") {
        option.noslip_iterations = noslip.max(0) as usize;
    }
    if let Some(noslip_tol) = parse_float_attr(e, "noslip_tolerance") {
        option.noslip_tolerance = noslip_tol;
    }
    if let Some(ccd) = parse_int_attr(e, "ccd_iterations") {
        option.ccd_iterations = ccd.max(0) as usize;
    }

    // Contact configuration
    if let Some(cone) = get_attribute_opt(e, "cone") {
        if let Some(c) = MjcfConeType::from_str(&cone) {
            option.cone = c;
        }
    }
    if let Some(jacobian) = get_attribute_opt(e, "jacobian") {
        if let Some(j) = MjcfJacobianType::from_str(&jacobian) {
            option.jacobian = j;
        }
    }
    if let Some(impratio) = parse_float_attr(e, "impratio") {
        option.impratio = impratio;
    }
    if let Some(regularization) = parse_float_attr(e, "regularization") {
        option.regularization = regularization;
    }
    if let Some(eq_stiffness) = parse_float_attr(e, "default_eq_stiffness") {
        option.default_eq_stiffness = eq_stiffness;
    }
    if let Some(eq_damping) = parse_float_attr(e, "default_eq_damping") {
        option.default_eq_damping = eq_damping;
    }
    if let Some(max_vel) = parse_float_attr(e, "max_constraint_vel") {
        option.max_constraint_vel = max_vel;
    }
    if let Some(max_angvel) = parse_float_attr(e, "max_constraint_angvel") {
        option.max_constraint_angvel = max_angvel;
    }
    if let Some(fsmooth) = parse_float_attr(e, "friction_smoothing") {
        option.friction_smoothing = fsmooth;
    }

    // Physics environment
    if let Some(grav) = get_attribute_opt(e, "gravity") {
        option.gravity = parse_vector3(&grav)?;
    }
    if let Some(wind) = get_attribute_opt(e, "wind") {
        option.wind = parse_vector3(&wind)?;
    }
    if let Some(magnetic) = get_attribute_opt(e, "magnetic") {
        option.magnetic = parse_vector3(&magnetic)?;
    }
    if let Some(density) = parse_float_attr(e, "density") {
        option.density = density;
    }
    if let Some(viscosity) = parse_float_attr(e, "viscosity") {
        option.viscosity = viscosity;
    }

    // Constraint limits
    if let Some(nconmax) = parse_int_attr(e, "nconmax") {
        option.nconmax = nconmax.max(0) as usize;
    }
    if let Some(njmax) = parse_int_attr(e, "njmax") {
        option.njmax = njmax.max(0) as usize;
    }

    // Overrides
    if let Some(o_margin) = parse_float_attr(e, "o_margin") {
        option.o_margin = o_margin;
    }
    if let Some(o_solimp) = get_attribute_opt(e, "o_solimp") {
        let parts = parse_float_array(&o_solimp)?;
        if parts.len() >= 5 {
            option.o_solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(o_solref) = get_attribute_opt(e, "o_solref") {
        let parts = parse_float_array(&o_solref)?;
        if parts.len() >= 2 {
            option.o_solref = Some([parts[0], parts[1]]);
        }
    }
    if let Some(o_friction) = get_attribute_opt(e, "o_friction") {
        let parts = parse_float_array(&o_friction)?;
        if parts.len() >= 5 {
            option.o_friction = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }

    Ok(option)
}

/// Parse flag element attributes.
fn parse_flag_attrs(e: &BytesStart) -> Result<MjcfFlag> {
    let mut flag = MjcfFlag::default();

    // Helper to parse enable/disable attributes
    fn parse_flag(e: &BytesStart, name: &str, default: bool) -> bool {
        get_attribute_opt(e, name).map_or(default, |v| v != "disable")
    }

    flag.constraint = parse_flag(e, "constraint", flag.constraint);
    flag.equality = parse_flag(e, "equality", flag.equality);
    flag.frictionloss = parse_flag(e, "frictionloss", flag.frictionloss);
    flag.limit = parse_flag(e, "limit", flag.limit);
    flag.contact = parse_flag(e, "contact", flag.contact);
    flag.passive = parse_flag(e, "passive", flag.passive);
    flag.gravity = parse_flag(e, "gravity", flag.gravity);
    flag.clampctrl = parse_flag(e, "clampctrl", flag.clampctrl);
    flag.warmstart = parse_flag(e, "warmstart", flag.warmstart);
    flag.filterparent = parse_flag(e, "filterparent", flag.filterparent);
    flag.actuation = parse_flag(e, "actuation", flag.actuation);
    flag.refsafe = parse_flag(e, "refsafe", flag.refsafe);
    flag.sensor = parse_flag(e, "sensor", flag.sensor);
    flag.midphase = parse_flag(e, "midphase", flag.midphase);
    flag.nativeccd = parse_flag(e, "nativeccd", flag.nativeccd);
    flag.eulerdamp = parse_flag(e, "eulerdamp", flag.eulerdamp);
    flag.override_contacts = parse_flag(e, "override", flag.override_contacts);
    flag.energy = parse_flag(e, "energy", flag.energy);
    flag.island = parse_flag(e, "island", flag.island);
    flag.multiccd = parse_flag(e, "multiccd", flag.multiccd);

    Ok(flag)
}

/// Parse default element and its nested defaults.
fn parse_default<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    parent_class: Option<String>,
) -> Result<Vec<MjcfDefault>> {
    let class = get_attribute_opt(start, "class").unwrap_or_default();
    let mut default = MjcfDefault {
        class: class.clone(),
        parent_class,
        ..Default::default()
    };
    let mut nested_defaults = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"joint" => {
                        default.joint = Some(parse_joint_defaults(e)?);
                    }
                    b"geom" => {
                        default.geom = Some(parse_geom_defaults(e)?);
                    }
                    b"actuator" | b"motor" | b"position" | b"velocity" | b"general" => {
                        default.actuator = Some(parse_actuator_defaults(e)?);
                    }
                    b"tendon" => {
                        default.tendon = Some(parse_tendon_defaults(e)?);
                    }
                    b"sensor" => {
                        default.sensor = Some(parse_sensor_defaults(e)?);
                    }
                    b"mesh" => {
                        default.mesh = Some(parse_mesh_defaults(e)?);
                    }
                    b"site" => {
                        default.site = Some(parse_site_defaults(e)?);
                    }
                    b"pair" => {
                        default.pair = Some(parse_pair_defaults(e)?);
                    }
                    b"default" => {
                        // Nested default class
                        let nested = parse_default(reader, e, Some(class.clone()))?;
                        nested_defaults.extend(nested);
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"joint" => {
                    default.joint = Some(parse_joint_defaults(e)?);
                }
                b"geom" => {
                    default.geom = Some(parse_geom_defaults(e)?);
                }
                b"actuator" | b"motor" | b"position" | b"velocity" | b"general" => {
                    default.actuator = Some(parse_actuator_defaults(e)?);
                }
                b"tendon" => {
                    default.tendon = Some(parse_tendon_defaults(e)?);
                }
                b"sensor" => {
                    default.sensor = Some(parse_sensor_defaults(e)?);
                }
                b"mesh" => {
                    default.mesh = Some(parse_mesh_defaults(e)?);
                }
                b"site" => {
                    default.site = Some(parse_site_defaults(e)?);
                }
                b"pair" => {
                    default.pair = Some(parse_pair_defaults(e)?);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"default" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in default".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    let mut result = vec![default];
    result.extend(nested_defaults);
    Ok(result)
}

/// Parse joint defaults.
fn parse_joint_defaults(e: &BytesStart) -> Result<MjcfJointDefaults> {
    let mut defaults = MjcfJointDefaults::default();

    if let Some(jtype) = get_attribute_opt(e, "type") {
        defaults.joint_type = MjcfJointType::from_str(&jtype);
    }
    if let Some(limited) = get_attribute_opt(e, "limited") {
        defaults.limited = Some(limited == "true");
    }
    if let Some(axis) = get_attribute_opt(e, "axis") {
        defaults.axis = Some(parse_vector3(&axis)?);
    }
    defaults.damping = parse_float_attr(e, "damping");
    defaults.stiffness = parse_float_attr(e, "stiffness");
    defaults.armature = parse_float_attr(e, "armature");

    Ok(defaults)
}

/// Parse geom defaults.
fn parse_geom_defaults(e: &BytesStart) -> Result<MjcfGeomDefaults> {
    let mut defaults = MjcfGeomDefaults::default();

    if let Some(gtype) = get_attribute_opt(e, "type") {
        defaults.geom_type = MjcfGeomType::from_str(&gtype);
    }
    if let Some(friction) = get_attribute_opt(e, "friction") {
        defaults.friction = Some(parse_vector3(&friction)?);
    }
    defaults.density = parse_float_attr(e, "density");
    defaults.mass = parse_float_attr(e, "mass");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        defaults.rgba = Some(parse_vector4(&rgba)?);
    }
    defaults.contype = parse_int_attr(e, "contype");
    defaults.conaffinity = parse_int_attr(e, "conaffinity");

    Ok(defaults)
}

/// Parse actuator defaults.
fn parse_actuator_defaults(e: &BytesStart) -> Result<MjcfActuatorDefaults> {
    let mut defaults = MjcfActuatorDefaults::default();

    if let Some(ctrlrange) = get_attribute_opt(e, "ctrlrange") {
        let parts = parse_float_array(&ctrlrange)?;
        if parts.len() >= 2 {
            defaults.ctrlrange = Some((parts[0], parts[1]));
        }
    }
    if let Some(forcerange) = get_attribute_opt(e, "forcerange") {
        let parts = parse_float_array(&forcerange)?;
        if parts.len() >= 2 {
            defaults.forcerange = Some((parts[0], parts[1]));
        }
    }
    if let Some(gear_str) = get_attribute_opt(e, "gear") {
        let parts = parse_float_array(&gear_str)?;
        let mut gear = [0.0f64; 6];
        for (i, &v) in parts.iter().take(6).enumerate() {
            gear[i] = v;
        }
        defaults.gear = Some(gear);
    }
    defaults.kp = parse_float_attr(e, "kp");
    defaults.kv = parse_float_attr(e, "kv");

    if let Some(ctrllimited) = get_attribute_opt(e, "ctrllimited") {
        defaults.ctrllimited = Some(ctrllimited == "true");
    }
    if let Some(forcelimited) = get_attribute_opt(e, "forcelimited") {
        defaults.forcelimited = Some(forcelimited == "true");
    }

    // <general>-specific attributes — parsed unconditionally because
    // parse_actuator_defaults() is called for all actuator element names
    // (actuator|motor|position|velocity|general). The model builder
    // ignores these for shortcut types.
    defaults.gaintype = get_attribute_opt(e, "gaintype");
    defaults.biastype = get_attribute_opt(e, "biastype");
    defaults.dyntype = get_attribute_opt(e, "dyntype");
    defaults.gainprm = parse_float_array_opt(e, "gainprm")?;
    defaults.biasprm = parse_float_array_opt(e, "biasprm")?;
    defaults.dynprm = parse_float_array_opt(e, "dynprm")?;

    Ok(defaults)
}

/// Parse tendon defaults.
fn parse_tendon_defaults(e: &BytesStart) -> Result<MjcfTendonDefaults> {
    let mut defaults = MjcfTendonDefaults::default();

    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            defaults.range = Some((parts[0], parts[1]));
        }
    }
    if let Some(limited) = get_attribute_opt(e, "limited") {
        defaults.limited = Some(limited == "true");
    }
    defaults.stiffness = parse_float_attr(e, "stiffness");
    defaults.damping = parse_float_attr(e, "damping");
    defaults.frictionloss = parse_float_attr(e, "frictionloss");
    defaults.width = parse_float_attr(e, "width");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        defaults.rgba = Some(parse_vector4(&rgba)?);
    }

    Ok(defaults)
}

/// Parse sensor defaults.
fn parse_sensor_defaults(e: &BytesStart) -> Result<MjcfSensorDefaults> {
    let mut defaults = MjcfSensorDefaults::default();

    defaults.noise = parse_float_attr(e, "noise");
    defaults.cutoff = parse_float_attr(e, "cutoff");

    if let Some(user) = get_attribute_opt(e, "user") {
        defaults.user = Some(parse_float_array(&user)?);
    }

    Ok(defaults)
}

/// Parse mesh defaults.
fn parse_mesh_defaults(e: &BytesStart) -> Result<MjcfMeshDefaults> {
    let mut defaults = MjcfMeshDefaults::default();

    if let Some(scale) = get_attribute_opt(e, "scale") {
        defaults.scale = Some(parse_vector3(&scale)?);
    }

    Ok(defaults)
}

/// Parse site defaults.
fn parse_site_defaults(e: &BytesStart) -> Result<MjcfSiteDefaults> {
    let mut defaults = MjcfSiteDefaults::default();

    defaults.site_type = get_attribute_opt(e, "type");
    if let Some(size) = get_attribute_opt(e, "size") {
        defaults.size = Some(parse_float_array(&size)?);
    }
    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        defaults.rgba = Some(parse_vector4(&rgba)?);
    }

    Ok(defaults)
}

/// Parse pair defaults from `<default><pair .../>`.
fn parse_pair_defaults(e: &BytesStart) -> Result<MjcfPairDefaults> {
    let mut defaults = MjcfPairDefaults::default();

    defaults.condim = parse_int_attr(e, "condim");
    defaults.margin = parse_float_attr(e, "margin");
    defaults.gap = parse_float_attr(e, "gap");

    if let Some(friction) = get_attribute_opt(e, "friction") {
        let parts = parse_float_array(&friction)?;
        if parts.len() >= 5 {
            defaults.friction = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            defaults.solref = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solreffriction) = get_attribute_opt(e, "solreffriction") {
        let parts = parse_float_array(&solreffriction)?;
        if parts.len() >= 2 {
            defaults.solreffriction = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            defaults.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }

    Ok(defaults)
}

/// Parse `<contact>` element containing `<pair>` and `<exclude>` children.
fn parse_contact<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfContact> {
    let mut contact = MjcfContact::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"pair" => {
                        let pair = parse_contact_pair_attrs(e)?;
                        contact.pairs.push(pair);
                        skip_element(reader, &elem_name)?;
                    }
                    b"exclude" => {
                        let exclude = parse_contact_exclude_attrs(e)?;
                        contact.excludes.push(exclude);
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"pair" => {
                    let pair = parse_contact_pair_attrs(e)?;
                    contact.pairs.push(pair);
                }
                b"exclude" => {
                    let exclude = parse_contact_exclude_attrs(e)?;
                    contact.excludes.push(exclude);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"contact" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in contact".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(contact)
}

/// Parse attributes of a `<contact><pair .../>` element.
fn parse_contact_pair_attrs(e: &BytesStart) -> Result<MjcfContactPair> {
    let geom1 = get_attribute_opt(e, "geom1")
        .ok_or_else(|| MjcfError::missing_attribute("geom1", "pair"))?;
    let geom2 = get_attribute_opt(e, "geom2")
        .ok_or_else(|| MjcfError::missing_attribute("geom2", "pair"))?;

    let mut pair = MjcfContactPair {
        name: get_attribute_opt(e, "name"),
        class: get_attribute_opt(e, "class"),
        geom1,
        geom2,
        condim: parse_int_attr(e, "condim"),
        friction: None,
        solref: None,
        solreffriction: None,
        solimp: None,
        margin: parse_float_attr(e, "margin"),
        gap: parse_float_attr(e, "gap"),
    };

    if let Some(friction) = get_attribute_opt(e, "friction") {
        let parts = parse_float_array(&friction)?;
        if parts.len() >= 5 {
            pair.friction = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            pair.solref = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solreffriction) = get_attribute_opt(e, "solreffriction") {
        let parts = parse_float_array(&solreffriction)?;
        if parts.len() >= 2 {
            pair.solreffriction = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            pair.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }

    Ok(pair)
}

/// Parse attributes of a `<contact><exclude .../>` element.
fn parse_contact_exclude_attrs(e: &BytesStart) -> Result<MjcfContactExclude> {
    let body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "exclude"))?;
    let body2 = get_attribute_opt(e, "body2")
        .ok_or_else(|| MjcfError::missing_attribute("body2", "exclude"))?;

    Ok(MjcfContactExclude {
        name: get_attribute_opt(e, "name"),
        body1,
        body2,
    })
}

// ============================================================================
// Keyframe parsing
// ============================================================================

/// Parse the `<keyframe>` element containing `<key>` children.
fn parse_keyframes<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfKeyframe>> {
    let mut keyframes = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"key" => {
                        keyframes.push(parse_key_attrs(e)?);
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                if e.name().as_ref() == b"key" {
                    keyframes.push(parse_key_attrs(e)?);
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"keyframe" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in keyframe".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(keyframes)
}

/// Parse attributes of a single `<key>` element.
fn parse_key_attrs(e: &BytesStart) -> Result<MjcfKeyframe> {
    let mut kf = MjcfKeyframe::default();

    kf.name = get_attribute_opt(e, "name").unwrap_or_default();
    kf.time = parse_float_attr(e, "time").unwrap_or(0.0);
    kf.qpos = parse_float_array_opt(e, "qpos")?;
    kf.qvel = parse_float_array_opt(e, "qvel")?;
    kf.act = parse_float_array_opt(e, "act")?;
    kf.ctrl = parse_float_array_opt(e, "ctrl")?;
    kf.mpos = parse_float_array_opt(e, "mpos")?;
    kf.mquat = parse_float_array_opt(e, "mquat")?;

    Ok(kf)
}

/// Parse asset element (contains mesh, texture, material definitions).
fn parse_asset<R: BufRead>(reader: &mut Reader<R>) -> Result<(Vec<MjcfMesh>, Vec<MjcfHfield>)> {
    let mut meshes = Vec::new();
    let mut hfields = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"mesh" => {
                        let mesh = parse_mesh(reader, e)?;
                        meshes.push(mesh);
                    }
                    b"hfield" => {
                        let hfield = parse_hfield_attrs(e)?;
                        // <hfield> has no child elements — skip to closing tag
                        skip_element(reader, &elem_name)?;
                        hfields.push(hfield);
                    }
                    // Skip other asset types (texture, material, etc.)
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                if e.name().as_ref() == b"mesh" {
                    let mesh = parse_mesh_attrs(e)?;
                    meshes.push(mesh);
                } else if e.name().as_ref() == b"hfield" {
                    let hfield = parse_hfield_attrs(e)?;
                    hfields.push(hfield);
                }
                // Skip other self-closing asset types
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"asset" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in asset".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok((meshes, hfields))
}

/// Parse mesh element.
fn parse_mesh<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfMesh> {
    let mesh = parse_mesh_attrs(start)?;
    let mut buf = Vec::new();

    // Meshes typically don't have children, but we still need to read to the end
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"mesh" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in mesh".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(mesh)
}

/// Parse mesh attributes only.
fn parse_mesh_attrs(e: &BytesStart) -> Result<MjcfMesh> {
    let mut mesh = MjcfMesh::default();

    mesh.name = get_attribute_opt(e, "name").unwrap_or_default();
    mesh.file = get_attribute_opt(e, "file");

    if let Some(scale) = get_attribute_opt(e, "scale") {
        mesh.scale = parse_vector3(&scale)?;
    }

    // Parse embedded vertex data
    if let Some(vertex) = get_attribute_opt(e, "vertex") {
        mesh.vertex = Some(parse_float_array(&vertex)?);
    }

    // Parse embedded face data
    if let Some(face) = get_attribute_opt(e, "face") {
        let face_data = parse_float_array(&face)?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let face_indices: Vec<u32> = face_data.iter().map(|f| *f as u32).collect();
        mesh.face = Some(face_indices);
    }

    Ok(mesh)
}

/// Parse hfield attributes from a `<hfield>` element.
fn parse_hfield_attrs(e: &BytesStart) -> Result<MjcfHfield> {
    let name = get_attribute_opt(e, "name").ok_or_else(|| {
        MjcfError::XmlParse("hfield element missing required 'name' attribute".into())
    })?;

    let nrow: usize = get_attribute_opt(e, "nrow")
        .ok_or_else(|| {
            MjcfError::XmlParse("hfield element missing required 'nrow' attribute".into())
        })?
        .parse()
        .map_err(|_| MjcfError::XmlParse("hfield 'nrow' must be a positive integer".into()))?;

    let ncol: usize = get_attribute_opt(e, "ncol")
        .ok_or_else(|| {
            MjcfError::XmlParse("hfield element missing required 'ncol' attribute".into())
        })?
        .parse()
        .map_err(|_| MjcfError::XmlParse("hfield 'ncol' must be a positive integer".into()))?;

    if nrow < 2 || ncol < 2 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': nrow ({nrow}) and ncol ({ncol}) must be >= 2",
        )));
    }

    let size_str = get_attribute_opt(e, "size").ok_or_else(|| {
        MjcfError::XmlParse("hfield element missing required 'size' attribute".into())
    })?;
    let size_vec = parse_float_array(&size_str)?;
    if size_vec.len() != 4 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{}': size requires exactly 4 values, got {}",
            name,
            size_vec.len(),
        )));
    }
    let size = [size_vec[0], size_vec[1], size_vec[2], size_vec[3]];
    if size[0] <= 0.0 || size[1] <= 0.0 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': size[0] and size[1] (half-extents) must be > 0",
        )));
    }
    if size[2] < 0.0 || size[3] < 0.0 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': size[2] (z_top) and size[3] (z_bottom) must be >= 0",
        )));
    }

    let elevation_str = get_attribute_opt(e, "elevation").ok_or_else(|| {
        MjcfError::XmlParse("hfield element missing required 'elevation' attribute".into())
    })?;
    let elevation = parse_float_array(&elevation_str)?;
    if elevation.len() != nrow * ncol {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{}': elevation length ({}) must equal nrow * ncol ({})",
            name,
            elevation.len(),
            nrow * ncol,
        )));
    }

    Ok(MjcfHfield {
        name,
        size,
        nrow,
        ncol,
        elevation,
    })
}

/// Parse worldbody element.
fn parse_worldbody<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfBody> {
    let mut worldbody = MjcfBody::new("world");
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"body" => {
                        let body = parse_body(reader, e)?;
                        worldbody.children.push(body);
                    }
                    b"geom" => {
                        // Static geom attached to world
                        let geom = parse_geom(reader, e)?;
                        worldbody.geoms.push(geom);
                    }
                    b"site" => {
                        let site = parse_site(reader, e)?;
                        worldbody.sites.push(site);
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"geom" => {
                    let geom = parse_geom_attrs(e)?;
                    worldbody.geoms.push(geom);
                }
                b"site" => {
                    let site = parse_site_attrs(e)?;
                    worldbody.sites.push(site);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"worldbody" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in worldbody".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(worldbody)
}

/// Parse body element.
fn parse_body<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfBody> {
    let mut body = parse_body_attrs(start)?;
    let mut buf = Vec::new();
    let mut joint_counter = 0;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"body" => {
                        let child = parse_body(reader, e)?;
                        body.children.push(child);
                    }
                    b"joint" => {
                        let mut joint = parse_joint(reader, e)?;
                        // Auto-generate name if not specified
                        if joint.name.is_empty() {
                            joint.name = format!("{}_joint{}", body.name, joint_counter);
                            joint_counter += 1;
                        }
                        joint.body = Some(body.name.clone());
                        body.joints.push(joint);
                    }
                    b"geom" => {
                        let geom = parse_geom(reader, e)?;
                        body.geoms.push(geom);
                    }
                    b"inertial" => {
                        body.inertial = Some(parse_inertial(reader, e)?);
                    }
                    b"site" => {
                        let site = parse_site(reader, e)?;
                        body.sites.push(site);
                    }
                    b"freejoint" => {
                        // <freejoint>...</freejoint> form (rare but valid)
                        let mut joint = parse_freejoint_attrs(e)?;
                        if joint.name.is_empty() {
                            joint.name = format!("{}_joint{}", body.name, joint_counter);
                            joint_counter += 1;
                        }
                        joint.body = Some(body.name.clone());
                        body.joints.push(joint);
                        // Skip to end tag (freejoint has no children)
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"joint" => {
                    let mut joint = parse_joint_attrs(e)?;
                    if joint.name.is_empty() {
                        joint.name = format!("{}_joint{}", body.name, joint_counter);
                        joint_counter += 1;
                    }
                    joint.body = Some(body.name.clone());
                    body.joints.push(joint);
                }
                b"freejoint" => {
                    // <freejoint/> is MuJoCo shorthand for <joint type="free"/>
                    let mut joint = parse_freejoint_attrs(e)?;
                    if joint.name.is_empty() {
                        joint.name = format!("{}_joint{}", body.name, joint_counter);
                        joint_counter += 1;
                    }
                    joint.body = Some(body.name.clone());
                    body.joints.push(joint);
                }
                b"geom" => {
                    let geom = parse_geom_attrs(e)?;
                    body.geoms.push(geom);
                }
                b"inertial" => {
                    body.inertial = Some(parse_inertial_attrs(e)?);
                }
                b"site" => {
                    let site = parse_site_attrs(e)?;
                    body.sites.push(site);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"body" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in body".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(body)
}

/// Parse body attributes only.
fn parse_body_attrs(e: &BytesStart) -> Result<MjcfBody> {
    let name = get_attribute_opt(e, "name").unwrap_or_else(|| "unnamed".to_string());
    let mut body = MjcfBody::new(name);

    if let Some(pos) = get_attribute_opt(e, "pos") {
        body.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        body.quat = parse_vector4(&quat)?;
    }
    if let Some(euler) = get_attribute_opt(e, "euler") {
        body.euler = Some(parse_vector3(&euler)?);
    }
    if let Some(aa) = get_attribute_opt(e, "axisangle") {
        body.axisangle = Some(parse_vector4(&aa)?);
    }
    if let Some(mocap) = get_attribute_opt(e, "mocap") {
        body.mocap = mocap == "true";
    }

    Ok(body)
}

/// Parse joint element.
fn parse_joint<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfJoint> {
    let joint = parse_joint_attrs(start)?;
    let mut buf = Vec::new();

    // Joints typically don't have children, but we still need to read to the end
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"joint" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in joint".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(joint)
}

/// Parse joint attributes only.
fn parse_joint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let mut joint = MjcfJoint::default();

    joint.name = get_attribute_opt(e, "name").unwrap_or_default();
    joint.class = get_attribute_opt(e, "class");

    if let Some(jtype) = get_attribute_opt(e, "type") {
        joint.joint_type =
            MjcfJointType::from_str(&jtype).ok_or_else(|| MjcfError::UnknownJointType(jtype))?;
    }

    if let Some(pos) = get_attribute_opt(e, "pos") {
        joint.pos = parse_vector3(&pos)?;
    }
    if let Some(axis) = get_attribute_opt(e, "axis") {
        joint.axis = safe_normalize_axis(parse_vector3(&axis)?);
    }
    if let Some(limited) = get_attribute_opt(e, "limited") {
        joint.limited = limited == "true";
    }
    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            joint.range = Some((parts[0], parts[1]));
        }
    }
    joint.ref_pos = parse_float_attr(e, "ref").unwrap_or(0.0);
    joint.spring_ref = parse_float_attr(e, "springref").unwrap_or(0.0);
    joint.damping = parse_float_attr(e, "damping").unwrap_or(0.0);
    joint.stiffness = parse_float_attr(e, "stiffness").unwrap_or(0.0);
    joint.armature = parse_float_attr(e, "armature").unwrap_or(0.0);
    joint.frictionloss = parse_float_attr(e, "frictionloss").unwrap_or(0.0);

    // Joint limit solver parameters: solreflimit=[timeconst, dampratio],
    // solimplimit=[d0, d_width, width, midpoint, power]
    if let Some(solref) = get_attribute_opt(e, "solreflimit") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            joint.solref_limit = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solimp) = get_attribute_opt(e, "solimplimit") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            joint.solimp_limit = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }

    Ok(joint)
}

/// Parse freejoint attributes (MuJoCo shorthand for `<joint type="free"/>`).
///
/// The `<freejoint/>` element is a convenience shorthand in MuJoCo that creates
/// a 6-DOF free joint. It only supports `name` and `group` attributes.
fn parse_freejoint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let mut joint = MjcfJoint::default();
    joint.joint_type = MjcfJointType::Free;
    joint.name = get_attribute_opt(e, "name").unwrap_or_default();
    // Note: MuJoCo's freejoint also supports 'group' attribute, but we don't use it
    Ok(joint)
}

/// Parse geom element.
fn parse_geom<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfGeom> {
    let geom = parse_geom_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"geom" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in geom".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(geom)
}

/// Parse geom attributes only.
fn parse_geom_attrs(e: &BytesStart) -> Result<MjcfGeom> {
    let mut geom = MjcfGeom::default();

    geom.name = get_attribute_opt(e, "name");
    geom.class = get_attribute_opt(e, "class");

    if let Some(gtype) = get_attribute_opt(e, "type") {
        geom.geom_type =
            MjcfGeomType::from_str(&gtype).ok_or_else(|| MjcfError::UnknownGeomType(gtype))?;
    }

    if let Some(pos) = get_attribute_opt(e, "pos") {
        geom.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        geom.quat = parse_vector4(&quat)?;
    }
    if let Some(euler) = get_attribute_opt(e, "euler") {
        geom.euler = Some(parse_vector3(&euler)?);
    }
    if let Some(size) = get_attribute_opt(e, "size") {
        geom.size = parse_float_array(&size)?;
    }
    if let Some(fromto) = get_attribute_opt(e, "fromto") {
        let parts = parse_float_array(&fromto)?;
        if parts.len() >= 6 {
            geom.fromto = Some([parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]]);
        }
    }
    if let Some(friction) = get_attribute_opt(e, "friction") {
        geom.friction = parse_vector3(&friction)?;
    }
    if let Some(density) = parse_float_attr(e, "density") {
        geom.density = density;
    }
    geom.mass = parse_float_attr(e, "mass");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        geom.rgba = parse_vector4(&rgba)?;
    }
    if let Some(contype) = parse_int_attr(e, "contype") {
        geom.contype = contype;
    }
    if let Some(conaffinity) = parse_int_attr(e, "conaffinity") {
        geom.conaffinity = conaffinity;
    }
    if let Some(condim) = parse_int_attr(e, "condim") {
        geom.condim = condim;
    }
    geom.mesh = get_attribute_opt(e, "mesh");
    geom.hfield = get_attribute_opt(e, "hfield");

    // Contact solver parameters: solref=[timeconst, dampratio],
    // solimp=[d0, d_width, width, midpoint, power].
    // When two geoms collide, their params are combined (min for solref, max for solimp).
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            geom.solref = Some([parts[0], parts[1]]);
        }
    }
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            geom.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }

    Ok(geom)
}

/// Parse inertial element.
fn parse_inertial<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfInertial> {
    let inertial = parse_inertial_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"inertial" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in inertial".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(inertial)
}

/// Parse inertial attributes only.
fn parse_inertial_attrs(e: &BytesStart) -> Result<MjcfInertial> {
    let mut inertial = MjcfInertial::default();

    if let Some(pos) = get_attribute_opt(e, "pos") {
        inertial.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        inertial.quat = parse_vector4(&quat)?;
    }
    if let Some(mass) = parse_float_attr(e, "mass") {
        inertial.mass = mass;
    }
    if let Some(diag) = get_attribute_opt(e, "diaginertia") {
        inertial.diaginertia = Some(parse_vector3(&diag)?);
    }
    if let Some(full) = get_attribute_opt(e, "fullinertia") {
        let parts = parse_float_array(&full)?;
        if parts.len() >= 6 {
            inertial.fullinertia =
                Some([parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]]);
        }
    }

    Ok(inertial)
}

/// Parse site element.
fn parse_site<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfSite> {
    let site = parse_site_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"site" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in site".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(site)
}

/// Parse site attributes only.
fn parse_site_attrs(e: &BytesStart) -> Result<MjcfSite> {
    let mut site = MjcfSite::default();

    site.name = get_attribute_opt(e, "name").unwrap_or_default();

    if let Some(site_type) = get_attribute_opt(e, "type") {
        site.site_type = site_type;
    }
    if let Some(pos) = get_attribute_opt(e, "pos") {
        site.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        site.quat = parse_vector4(&quat)?;
    }
    if let Some(size) = get_attribute_opt(e, "size") {
        site.size = parse_float_array(&size)?;
    }
    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        site.rgba = parse_vector4(&rgba)?;
    }

    Ok(site)
}

/// Parse actuators element.
fn parse_actuators<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfActuator>> {
    let mut actuators = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let actuator = parse_actuator_attrs(e, actuator_type)?;
                    actuators.push(actuator);
                    // Skip to closing tag
                    skip_element(reader, elem_name.as_bytes())?;
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let actuator = parse_actuator_attrs(e, actuator_type)?;
                    actuators.push(actuator);
                    // Self-closing, no need to skip
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"actuator" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in actuator".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(actuators)
}

/// Parse actuator attributes.
fn parse_actuator_attrs(e: &BytesStart, actuator_type: MjcfActuatorType) -> Result<MjcfActuator> {
    let mut actuator = MjcfActuator {
        actuator_type,
        ..Default::default()
    };

    // Common attributes
    actuator.name = get_attribute_opt(e, "name").unwrap_or_default();
    actuator.class = get_attribute_opt(e, "class");
    actuator.joint = get_attribute_opt(e, "joint");
    actuator.site = get_attribute_opt(e, "site");
    actuator.tendon = get_attribute_opt(e, "tendon");
    actuator.body = get_attribute_opt(e, "body");

    if let Some(gear_str) = get_attribute_opt(e, "gear") {
        let parts = parse_float_array(&gear_str)?;
        let mut gear = [0.0f64; 6];
        for (i, &v) in parts.iter().take(6).enumerate() {
            gear[i] = v;
        }
        actuator.gear = gear;
    }
    actuator.refsite = get_attribute_opt(e, "refsite");

    if let Some(ctrlrange) = get_attribute_opt(e, "ctrlrange") {
        let parts = parse_float_array(&ctrlrange)?;
        if parts.len() >= 2 {
            actuator.ctrlrange = Some((parts[0], parts[1]));
        }
    }
    if let Some(forcerange) = get_attribute_opt(e, "forcerange") {
        let parts = parse_float_array(&forcerange)?;
        if parts.len() >= 2 {
            actuator.forcerange = Some((parts[0], parts[1]));
        }
    }

    if let Some(ctrllimited) = get_attribute_opt(e, "ctrllimited") {
        actuator.ctrllimited = ctrllimited == "true";
    }
    if let Some(forcelimited) = get_attribute_opt(e, "forcelimited") {
        actuator.forcelimited = forcelimited == "true";
    }

    if let Some(kp) = parse_float_attr(e, "kp") {
        actuator.kp = kp;
    }
    if let Some(kv) = parse_float_attr(e, "kv") {
        actuator.kv = Some(kv);
    }

    // ========================================================================
    // <general>-specific attributes
    // ========================================================================
    // These are only parsed for <general> actuators. For shortcut types,
    // these attributes are not part of the MJCF schema and are ignored.
    if actuator_type == MjcfActuatorType::General {
        actuator.gaintype = get_attribute_opt(e, "gaintype");
        actuator.biastype = get_attribute_opt(e, "biastype");
        actuator.dyntype = get_attribute_opt(e, "dyntype");
        actuator.gainprm = parse_float_array_opt(e, "gainprm")?;
        actuator.biasprm = parse_float_array_opt(e, "biasprm")?;
        actuator.dynprm = parse_float_array_opt(e, "dynprm")?;
    }

    // ========================================================================
    // Cylinder-specific attributes
    // ========================================================================
    if let Some(area) = parse_float_attr(e, "area") {
        actuator.area = area;
    }
    if let Some(diameter) = parse_float_attr(e, "diameter") {
        actuator.diameter = Some(diameter);
    }
    if let Some(timeconst) = parse_float_attr(e, "timeconst") {
        actuator.timeconst = Some(timeconst);
    }
    if let Some(bias) = get_attribute_opt(e, "bias") {
        let parts = parse_float_array(&bias)?;
        if parts.len() >= 3 {
            actuator.bias = [parts[0], parts[1], parts[2]];
        }
    }

    // ========================================================================
    // Muscle-specific attributes
    // ========================================================================
    // For muscle, timeconst is a pair [activation, deactivation]
    if actuator_type == MjcfActuatorType::Muscle {
        if let Some(tc) = get_attribute_opt(e, "timeconst") {
            let parts = parse_float_array(&tc)?;
            if parts.len() >= 2 {
                actuator.muscle_timeconst = (parts[0], parts[1]);
            } else if parts.len() == 1 {
                // Single value means both are the same
                actuator.muscle_timeconst = (parts[0], parts[0]);
            }
        }
    }
    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            actuator.range = (parts[0], parts[1]);
        }
    }
    if let Some(force) = parse_float_attr(e, "force") {
        actuator.force = force;
    }
    if let Some(scale) = parse_float_attr(e, "scale") {
        actuator.scale = scale;
    }
    if let Some(lmin) = parse_float_attr(e, "lmin") {
        actuator.lmin = lmin;
    }
    if let Some(lmax) = parse_float_attr(e, "lmax") {
        actuator.lmax = lmax;
    }
    if let Some(vmax) = parse_float_attr(e, "vmax") {
        actuator.vmax = vmax;
    }
    if let Some(fpmax) = parse_float_attr(e, "fpmax") {
        actuator.fpmax = fpmax;
    }
    if let Some(fvmax) = parse_float_attr(e, "fvmax") {
        actuator.fvmax = fvmax;
    }

    // ========================================================================
    // Adhesion-specific attributes
    // ========================================================================
    if let Some(gain) = parse_float_attr(e, "gain") {
        actuator.gain = gain;
    }

    Ok(actuator)
}

/// Parse equality constraints element.
fn parse_equality<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfEquality> {
    let mut equality = MjcfEquality::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"connect" => {
                        let connect = parse_connect_attrs(e)?;
                        equality.connects.push(connect);
                        // Skip to closing tag
                        skip_element(reader, &elem_name)?;
                    }
                    b"weld" => {
                        let weld = parse_weld_attrs(e)?;
                        equality.welds.push(weld);
                        skip_element(reader, &elem_name)?;
                    }
                    b"joint" => {
                        let joint_eq = parse_joint_equality_attrs(e)?;
                        equality.joints.push(joint_eq);
                        skip_element(reader, &elem_name)?;
                    }
                    b"distance" => {
                        let distance = parse_distance_attrs(e)?;
                        equality.distances.push(distance);
                        skip_element(reader, &elem_name)?;
                    }
                    // Skip other equality constraint types (tendon, flex)
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                match e.name().as_ref() {
                    b"connect" => {
                        let connect = parse_connect_attrs(e)?;
                        equality.connects.push(connect);
                    }
                    b"weld" => {
                        let weld = parse_weld_attrs(e)?;
                        equality.welds.push(weld);
                    }
                    b"joint" => {
                        let joint_eq = parse_joint_equality_attrs(e)?;
                        equality.joints.push(joint_eq);
                    }
                    b"distance" => {
                        let distance = parse_distance_attrs(e)?;
                        equality.distances.push(distance);
                    }
                    _ => {} // Ignore other self-closing equality constraint types
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"equality" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in equality".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(equality)
}

/// Parse connect constraint attributes.
fn parse_connect_attrs(e: &BytesStart) -> Result<MjcfConnect> {
    let mut connect = MjcfConnect::default();

    connect.name = get_attribute_opt(e, "name");
    connect.class = get_attribute_opt(e, "class");

    // body1 is required
    connect.body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "connect"))?;

    // body2 is optional (defaults to world)
    connect.body2 = get_attribute_opt(e, "body2");

    // Parse anchor point
    if let Some(anchor) = get_attribute_opt(e, "anchor") {
        connect.anchor = parse_vector3(&anchor)?;
    }

    // Parse solver parameters
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            connect.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            connect.solref = Some([parts[0], parts[1]]);
        }
    }

    // Parse active flag
    if let Some(active) = get_attribute_opt(e, "active") {
        connect.active = active != "false";
    }

    Ok(connect)
}

/// Parse weld constraint attributes.
fn parse_weld_attrs(e: &BytesStart) -> Result<MjcfWeld> {
    let mut weld = MjcfWeld::default();

    weld.name = get_attribute_opt(e, "name");
    weld.class = get_attribute_opt(e, "class");

    // body1 is required
    weld.body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "weld"))?;

    // body2 is optional (defaults to world)
    weld.body2 = get_attribute_opt(e, "body2");

    // Parse anchor point
    if let Some(anchor) = get_attribute_opt(e, "anchor") {
        weld.anchor = parse_vector3(&anchor)?;
    }

    // Parse relpose [x y z qw qx qy qz]
    if let Some(relpose) = get_attribute_opt(e, "relpose") {
        let parts = parse_float_array(&relpose)?;
        if parts.len() >= 7 {
            weld.relpose = Some([
                parts[0], parts[1], parts[2], parts[3], parts[4], parts[5], parts[6],
            ]);
        }
    }

    // Parse solver parameters
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            weld.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            weld.solref = Some([parts[0], parts[1]]);
        }
    }

    // Parse active flag
    if let Some(active) = get_attribute_opt(e, "active") {
        weld.active = active != "false";
    }

    Ok(weld)
}

/// Parse joint equality constraint attributes.
fn parse_joint_equality_attrs(e: &BytesStart) -> Result<MjcfJointEquality> {
    let mut joint_eq = MjcfJointEquality::default();

    joint_eq.name = get_attribute_opt(e, "name");
    joint_eq.class = get_attribute_opt(e, "class");

    // joint1 is required
    joint_eq.joint1 = get_attribute_opt(e, "joint1")
        .ok_or_else(|| MjcfError::missing_attribute("joint1", "joint equality"))?;

    // joint2 is optional (for coupling constraints)
    joint_eq.joint2 = get_attribute_opt(e, "joint2");

    // Parse polycoef (polynomial coefficients for coupling)
    if let Some(polycoef) = get_attribute_opt(e, "polycoef") {
        joint_eq.polycoef = parse_float_array(&polycoef)?;
    }

    // Parse solver parameters
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            joint_eq.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            joint_eq.solref = Some([parts[0], parts[1]]);
        }
    }

    // Parse active flag
    if let Some(active) = get_attribute_opt(e, "active") {
        joint_eq.active = active != "false";
    }

    Ok(joint_eq)
}

/// Parse distance constraint attributes.
fn parse_distance_attrs(e: &BytesStart) -> Result<MjcfDistance> {
    let mut distance = MjcfDistance::default();

    distance.name = get_attribute_opt(e, "name");
    distance.class = get_attribute_opt(e, "class");

    // geom1 is required
    distance.geom1 = get_attribute_opt(e, "geom1")
        .ok_or_else(|| MjcfError::missing_attribute("geom1", "distance"))?;

    // geom2 is optional (defaults to world origin)
    distance.geom2 = get_attribute_opt(e, "geom2");

    // Parse target distance
    if let Some(dist_str) = get_attribute_opt(e, "distance") {
        distance.distance = dist_str.parse().ok();
    }

    // Parse solver parameters
    if let Some(solimp) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&solimp)?;
        if parts.len() >= 5 {
            distance.solimp = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
        }
    }
    if let Some(solref) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&solref)?;
        if parts.len() >= 2 {
            distance.solref = Some([parts[0], parts[1]]);
        }
    }

    // Parse active flag
    if let Some(active) = get_attribute_opt(e, "active") {
        distance.active = active != "false";
    }

    Ok(distance)
}

// ============================================================================
// Deformable / Skin parsing
// ============================================================================

/// Parse deformable section (contains skin elements).
fn parse_deformable<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfSkin>> {
    let mut skins = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                if elem_name.as_slice() == b"skin" {
                    let skin = parse_skin(reader, e)?;
                    skins.push(skin);
                } else {
                    // Skip other deformable elements (flex, etc.)
                    skip_element(reader, &elem_name)?;
                }
            }
            Ok(Event::Empty(ref e)) => {
                // Handle self-closing skin elements (unlikely but possible)
                if e.name().as_ref() == b"skin" {
                    let skin = parse_skin_attrs(e);
                    skins.push(skin);
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"deformable" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in deformable".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(skins)
}

/// Parse a skin element and its children.
fn parse_skin<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfSkin> {
    let mut skin = parse_skin_attrs(start);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"bone" => {
                        let bone = parse_skin_bone_attrs(e)?;
                        skin.bones.push(bone);
                        // Skip to closing tag if not self-closing
                        skip_element(reader, &elem_name)?;
                    }
                    b"vertex" => {
                        let vertex = parse_skin_vertex_attrs(e)?;
                        skin.vertices.push(vertex);
                        // Skip to closing tag if not self-closing
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"bone" => {
                    let bone = parse_skin_bone_attrs(e)?;
                    skin.bones.push(bone);
                }
                b"vertex" => {
                    let vertex = parse_skin_vertex_attrs(e)?;
                    skin.vertices.push(vertex);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"skin" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in skin".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(skin)
}

/// Parse skin element attributes.
fn parse_skin_attrs(e: &BytesStart) -> MjcfSkin {
    let mut skin = MjcfSkin::default();

    skin.name = get_attribute_opt(e, "name").unwrap_or_default();
    skin.mesh = get_attribute_opt(e, "mesh");
    skin.material = get_attribute_opt(e, "material");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        if let Ok(v) = parse_vector4(&rgba) {
            skin.rgba = v;
        }
    }

    if let Some(inflate) = parse_float_attr(e, "inflate") {
        skin.inflate = inflate;
    }

    // Parse embedded vertex positions if present
    if let Some(vertex) = get_attribute_opt(e, "vertex") {
        if let Ok(verts) = parse_float_array(&vertex) {
            skin.vertex_positions = Some(verts);
        }
    }

    // Parse embedded face data if present
    if let Some(face) = get_attribute_opt(e, "face") {
        if let Ok(faces) = parse_int_array(&face) {
            skin.faces = Some(faces);
        }
    }

    skin
}

/// Parse skin bone element attributes.
fn parse_skin_bone_attrs(e: &BytesStart) -> Result<MjcfSkinBone> {
    let mut bone = MjcfSkinBone::default();

    // body is required
    bone.body =
        get_attribute_opt(e, "body").ok_or_else(|| MjcfError::missing_attribute("body", "bone"))?;

    if let Some(bindpos) = get_attribute_opt(e, "bindpos") {
        bone.bindpos = parse_vector3(&bindpos)?;
    }

    if let Some(bindquat) = get_attribute_opt(e, "bindquat") {
        bone.bindquat = parse_vector4(&bindquat)?;
    }

    Ok(bone)
}

/// Parse skin vertex element attributes.
fn parse_skin_vertex_attrs(e: &BytesStart) -> Result<MjcfSkinVertex> {
    // id is required (vertex index in mesh)
    let id = parse_int_attr(e, "id").ok_or_else(|| MjcfError::missing_attribute("id", "vertex"))?
        as usize;

    // bone is required (bone index in skin)
    let bone = parse_int_attr(e, "bone")
        .ok_or_else(|| MjcfError::missing_attribute("bone", "vertex"))? as usize;

    // weight defaults to 1.0 if not specified
    let weight = parse_float_attr(e, "weight").unwrap_or(1.0);

    Ok(MjcfSkinVertex::new(id, bone, weight))
}

/// Parse a space-separated array of unsigned integers.
fn parse_int_array(s: &str) -> Result<Vec<u32>> {
    s.split_whitespace()
        .map(|p| {
            p.parse::<u32>()
                .map_err(|_| MjcfError::XmlParse(format!("invalid integer: {p}")))
        })
        .collect()
}

// ============================================================================
// Helper functions
// ============================================================================

/// Get an optional attribute value.
fn get_attribute_opt(e: &BytesStart, name: &str) -> Option<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == name.as_bytes() {
            return String::from_utf8(attr.value.to_vec()).ok();
        }
    }
    None
}

/// Parse a float attribute, returning None if not present or invalid.
fn parse_float_attr(e: &BytesStart, name: &str) -> Option<f64> {
    get_attribute_opt(e, name).and_then(|s| s.parse().ok())
}

/// Parse an integer attribute.
fn parse_int_attr(e: &BytesStart, name: &str) -> Option<i32> {
    get_attribute_opt(e, name).and_then(|s| s.parse().ok())
}

/// Parse a space-separated vector3 string.
fn parse_vector3(s: &str) -> Result<Vector3<f64>> {
    let parts = parse_float_array(s)?;

    if parts.len() < 3 {
        return Err(MjcfError::XmlParse(format!(
            "expected 3 values in vector, got {}: {s}",
            parts.len()
        )));
    }

    Ok(Vector3::new(parts[0], parts[1], parts[2]))
}

/// Parse a space-separated vector4 string.
fn parse_vector4(s: &str) -> Result<Vector4<f64>> {
    let parts = parse_float_array(s)?;

    if parts.len() < 4 {
        return Err(MjcfError::XmlParse(format!(
            "expected 4 values in vector, got {}: {s}",
            parts.len()
        )));
    }

    Ok(Vector4::new(parts[0], parts[1], parts[2], parts[3]))
}

/// Parse a space-separated array of floats.
fn parse_float_array(s: &str) -> Result<Vec<f64>> {
    s.split_whitespace()
        .map(|p| {
            p.parse::<f64>()
                .map_err(|_| MjcfError::XmlParse(format!("invalid float: {p}")))
        })
        .collect()
}

/// Parse an optional space-separated float array attribute.
/// Returns `Ok(None)` when absent, `Ok(Some(vec))` when present and valid,
/// `Err` when present but contains unparseable floats.
fn parse_float_array_opt(e: &BytesStart, name: &str) -> Result<Option<Vec<f64>>> {
    match get_attribute_opt(e, name) {
        Some(s) => Ok(Some(parse_float_array(&s)?)),
        None => Ok(None),
    }
}

// ============================================================================
// Tendon parsing
// ============================================================================

/// Parse the tendon section.
fn parse_tendons<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfTendon>> {
    let mut tendons = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(tendon_type) = MjcfTendonType::from_str(&elem_name) {
                    let tendon = parse_tendon(reader, e, tendon_type)?;
                    tendons.push(tendon);
                } else {
                    // Unknown tendon type, skip it
                    skip_element(reader, elem_name.as_bytes())?;
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(tendon_type) = MjcfTendonType::from_str(&elem_name) {
                    let tendon = parse_tendon_attrs(e, tendon_type);
                    tendons.push(tendon);
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"tendon" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in tendon".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(tendons)
}

/// Parse a tendon element and its children (sites for spatial, joints for fixed).
fn parse_tendon<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    tendon_type: MjcfTendonType,
) -> Result<MjcfTendon> {
    let mut tendon = parse_tendon_attrs(start, tendon_type);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let name_bytes = e.name().as_ref().to_vec();
                match name_bytes.as_slice() {
                    b"site" => {
                        // Site reference for spatial tendon path
                        if let Some(site_name) = get_attribute_opt(e, "site") {
                            tendon
                                .path_elements
                                .push(SpatialPathElement::Site { site: site_name });
                        }
                    }
                    b"joint" => {
                        if tendon.tendon_type == MjcfTendonType::Spatial {
                            return Err(MjcfError::XmlParse(format!(
                                "Joint elements inside spatial tendons are not supported \
                                 (tendon '{}'). Use a fixed tendon for joint coupling.",
                                tendon.name
                            )));
                        }
                        // Joint reference for fixed tendon
                        if let Some(joint_name) = get_attribute_opt(e, "joint") {
                            let coef = parse_float_attr(e, "coef").unwrap_or(1.0);
                            tendon.joints.push((joint_name, coef));
                        }
                    }
                    b"geom" => {
                        // Wrapping geom reference for spatial tendon path
                        if let Some(geom_name) = get_attribute_opt(e, "geom") {
                            let sidesite = get_attribute_opt(e, "sidesite");
                            tendon.path_elements.push(SpatialPathElement::Geom {
                                geom: geom_name,
                                sidesite,
                            });
                        }
                    }
                    b"pulley" => {
                        // Pulley element for spatial tendon path
                        let divisor = parse_float_attr(e, "divisor").unwrap_or(1.0);
                        tendon
                            .path_elements
                            .push(SpatialPathElement::Pulley { divisor });
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) => {
                let name_bytes = e.name().as_ref().to_vec();
                if name_bytes == b"spatial" || name_bytes == b"fixed" {
                    break;
                }
            }
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(
                    "unexpected EOF in tendon element".into(),
                ));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(tendon)
}

/// Parse tendon attributes.
fn parse_tendon_attrs(e: &BytesStart, tendon_type: MjcfTendonType) -> MjcfTendon {
    let mut tendon = MjcfTendon {
        tendon_type,
        ..Default::default()
    };

    tendon.name = get_attribute_opt(e, "name").unwrap_or_default();
    tendon.class = get_attribute_opt(e, "class");

    if let Some(range) = get_attribute_opt(e, "range") {
        if let Ok(parts) = parse_float_array(&range) {
            if parts.len() >= 2 {
                tendon.range = Some((parts[0], parts[1]));
            }
        }
    }

    if let Some(limited) = get_attribute_opt(e, "limited") {
        tendon.limited = limited == "true";
    }

    if let Some(stiffness) = parse_float_attr(e, "stiffness") {
        tendon.stiffness = stiffness;
    }

    if let Some(damping) = parse_float_attr(e, "damping") {
        tendon.damping = damping;
    }

    if let Some(frictionloss) = parse_float_attr(e, "frictionloss") {
        tendon.frictionloss = frictionloss;
    }

    if let Some(width) = parse_float_attr(e, "width") {
        tendon.width = width;
    }

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        if let Ok(parts) = parse_float_array(&rgba) {
            if parts.len() >= 4 {
                tendon.rgba = Vector4::new(parts[0], parts[1], parts[2], parts[3]);
            }
        }
    }

    tendon
}

// ============================================================================
// Sensor parsing
// ============================================================================

/// Parse the sensor section.
fn parse_sensors<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfSensor>> {
    let mut sensors = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(sensor_type) = MjcfSensorType::from_str(&elem_name) {
                    let sensor = parse_sensor_attrs(e, sensor_type);
                    sensors.push(sensor);
                    // Skip to closing tag
                    skip_element(reader, elem_name.as_bytes())?;
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(sensor_type) = MjcfSensorType::from_str(&elem_name) {
                    let sensor = parse_sensor_attrs(e, sensor_type);
                    sensors.push(sensor);
                    // Self-closing, no need to skip
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"sensor" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in sensor".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(sensors)
}

/// Parse sensor attributes.
fn parse_sensor_attrs(e: &BytesStart, sensor_type: MjcfSensorType) -> MjcfSensor {
    let mut sensor = MjcfSensor {
        sensor_type,
        ..Default::default()
    };

    sensor.name = get_attribute_opt(e, "name").unwrap_or_default();
    sensor.class = get_attribute_opt(e, "class");

    // Different sensor types use different attribute names for their target
    sensor.objname = get_attribute_opt(e, "joint")
        .or_else(|| get_attribute_opt(e, "site"))
        .or_else(|| get_attribute_opt(e, "body"))
        .or_else(|| get_attribute_opt(e, "tendon"))
        .or_else(|| get_attribute_opt(e, "actuator"))
        .or_else(|| get_attribute_opt(e, "objname"));

    sensor.refname = get_attribute_opt(e, "reftype").or_else(|| get_attribute_opt(e, "refname"));

    if let Some(noise) = parse_float_attr(e, "noise") {
        sensor.noise = noise;
    }

    if let Some(cutoff) = parse_float_attr(e, "cutoff") {
        sensor.cutoff = cutoff;
    }

    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            sensor.user = parts;
        }
    }

    sensor
}

/// Skip an element and all its children.
fn skip_element<R: BufRead>(reader: &mut Reader<R>, name: &[u8]) -> Result<()> {
    let mut buf = Vec::new();
    let mut depth = 1;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == name => {
                depth += 1;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == name => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_parse_simple_model() {
        let xml = r#"
            <mujoco model="test_model">
                <worldbody>
                    <body name="base">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.name, "test_model");
        assert_eq!(model.worldbody.children.len(), 1);

        let base = &model.worldbody.children[0];
        assert_eq!(base.name, "base");
        assert_eq!(base.geoms.len(), 1);
        assert_eq!(base.geoms[0].geom_type, MjcfGeomType::Sphere);
    }

    #[test]
    fn test_parse_option() {
        let xml = r#"
            <mujoco model="test">
                <option timestep="0.001" gravity="0 0 -10"/>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_relative_eq!(model.option.timestep, 0.001, epsilon = 1e-10);
        assert_relative_eq!(model.option.gravity.z, -10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_joint() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge" axis="0 1 0" limited="true" range="-1.57 1.57" damping="0.5"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let body = &model.worldbody.children[0];
        assert_eq!(body.joints.len(), 1);

        let joint = &body.joints[0];
        assert_eq!(joint.name, "joint1");
        assert_eq!(joint.joint_type, MjcfJointType::Hinge);
        assert!(joint.limited);
        assert_eq!(joint.range, Some((-1.57, 1.57)));
        assert_relative_eq!(joint.damping, 0.5, epsilon = 1e-10);
        assert_relative_eq!(joint.axis.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_inertial() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <inertial pos="0 0 0.1" mass="2.0" diaginertia="0.1 0.2 0.3"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let body = &model.worldbody.children[0];
        let inertial = body.inertial.as_ref().expect("should have inertial");

        assert_relative_eq!(inertial.mass, 2.0, epsilon = 1e-10);
        assert_relative_eq!(inertial.pos.z, 0.1, epsilon = 1e-10);

        let diag = inertial.diaginertia.expect("should have diaginertia");
        assert_relative_eq!(diag.x, 0.1, epsilon = 1e-10);
        assert_relative_eq!(diag.y, 0.2, epsilon = 1e-10);
        assert_relative_eq!(diag.z, 0.3, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_nested_bodies() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="base" pos="0 0 0.1">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="child" pos="0 0 0.2">
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let base = &model.worldbody.children[0];
        assert_eq!(base.name, "base");
        assert_eq!(base.children.len(), 1);

        let child = &base.children[0];
        assert_eq!(child.name, "child");
        assert_eq!(child.joints.len(), 1);
        assert_relative_eq!(child.pos.z, 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_actuators() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="joint1" gear="100"/>
                    <position name="servo1" joint="joint1" kp="10"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.actuators.len(), 2);

        let motor = &model.actuators[0];
        assert_eq!(motor.name, "motor1");
        assert_eq!(motor.actuator_type, MjcfActuatorType::Motor);
        assert_eq!(motor.joint, Some("joint1".to_string()));
        assert_relative_eq!(motor.gear[0], 100.0, epsilon = 1e-10);

        let servo = &model.actuators[1];
        assert_eq!(servo.name, "servo1");
        assert_eq!(servo.actuator_type, MjcfActuatorType::Position);
        assert_relative_eq!(servo.kp, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_geom_fromto() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.05"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let geom = &model.worldbody.children[0].geoms[0];
        assert_eq!(geom.geom_type, MjcfGeomType::Capsule);

        let fromto = geom.fromto.expect("should have fromto");
        assert_relative_eq!(fromto[5], 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_unknown_joint_type() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint type="invalid"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let result = parse_mjcf_str(xml);
        assert!(matches!(result, Err(MjcfError::UnknownJointType(_))));
    }

    #[test]
    fn test_parse_vector3() {
        let v = parse_vector3("1.0 2.0 3.0").expect("should parse");
        assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(v.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(v.z, 3.0, epsilon = 1e-10);

        // With extra whitespace
        let v = parse_vector3("  1   2   3  ").expect("should parse");
        assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Option parsing tests
    // ========================================================================

    #[test]
    fn test_parse_option_all_attributes() {
        let xml = r#"
            <mujoco model="test">
                <option
                    timestep="0.005"
                    integrator="RK4"
                    solver="PGS"
                    iterations="200"
                    tolerance="1e-6"
                    ls_iterations="100"
                    noslip_iterations="10"
                    ccd_iterations="25"
                    cone="elliptic"
                    jacobian="sparse"
                    impratio="2.0"
                    gravity="0 0 -10.5"
                    wind="1 2 0"
                    magnetic="0.1 0.2 0.3"
                    density="1.2"
                    viscosity="0.01"
                    nconmax="500"
                    njmax="1000"
                    o_margin="0.002"
                />
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let opt = &model.option;

        // Core simulation
        assert_relative_eq!(opt.timestep, 0.005, epsilon = 1e-10);
        assert_eq!(opt.integrator, MjcfIntegrator::RK4);

        // Solver configuration
        assert_eq!(opt.solver, MjcfSolverType::PGS);
        assert_eq!(opt.iterations, 200);
        assert_relative_eq!(opt.tolerance, 1e-6, epsilon = 1e-15);
        assert_eq!(opt.ls_iterations, 100);
        assert_eq!(opt.noslip_iterations, 10);
        assert_eq!(opt.ccd_iterations, 25);

        // Contact configuration
        assert_eq!(opt.cone, MjcfConeType::Elliptic);
        assert_eq!(opt.jacobian, MjcfJacobianType::Sparse);
        assert_relative_eq!(opt.impratio, 2.0, epsilon = 1e-10);

        // Physics environment
        assert_relative_eq!(opt.gravity.z, -10.5, epsilon = 1e-10);
        assert_relative_eq!(opt.wind.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(opt.wind.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(opt.magnetic.x, 0.1, epsilon = 1e-10);
        assert_relative_eq!(opt.density, 1.2, epsilon = 1e-10);
        assert_relative_eq!(opt.viscosity, 0.01, epsilon = 1e-10);

        // Constraint limits
        assert_eq!(opt.nconmax, 500);
        assert_eq!(opt.njmax, 1000);

        // Overrides
        assert_relative_eq!(opt.o_margin, 0.002, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_option_integrator_types() {
        for (name, expected) in [
            ("Euler", MjcfIntegrator::Euler),
            ("RK4", MjcfIntegrator::RK4),
            ("implicit", MjcfIntegrator::Implicit),
            ("implicitfast", MjcfIntegrator::ImplicitFast),
            ("implicitspringdamper", MjcfIntegrator::ImplicitSpringDamper),
        ] {
            let xml = format!(
                r#"
                <mujoco model="test">
                    <option integrator="{name}"/>
                    <worldbody/>
                </mujoco>
            "#
            );

            let model = parse_mjcf_str(&xml).expect("should parse");
            assert_eq!(model.option.integrator, expected, "integrator: {name}");
        }
    }

    #[test]
    fn test_parse_option_solver_types() {
        for (name, expected) in [
            ("PGS", MjcfSolverType::PGS),
            ("CG", MjcfSolverType::CG),
            ("Newton", MjcfSolverType::Newton),
        ] {
            let xml = format!(
                r#"
                <mujoco model="test">
                    <option solver="{name}"/>
                    <worldbody/>
                </mujoco>
            "#
            );

            let model = parse_mjcf_str(&xml).expect("should parse");
            assert_eq!(model.option.solver, expected, "solver: {name}");
        }
    }

    #[test]
    fn test_parse_option_with_flag() {
        let xml = r#"
            <mujoco model="test">
                <option timestep="0.002">
                    <flag
                        contact="disable"
                        gravity="disable"
                        constraint="enable"
                        warmstart="disable"
                        energy="enable"
                        island="enable"
                    />
                </option>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let flag = &model.option.flag;

        assert!(!flag.contact);
        assert!(!flag.gravity);
        assert!(flag.constraint);
        assert!(!flag.warmstart);
        assert!(flag.energy);
        assert!(flag.island);
    }

    #[test]
    fn test_parse_option_self_closing_with_defaults() {
        let xml = r#"
            <mujoco model="test">
                <option timestep="0.001"/>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let opt = &model.option;

        // Specified value
        assert_relative_eq!(opt.timestep, 0.001, epsilon = 1e-10);

        // Defaults
        assert_eq!(opt.integrator, MjcfIntegrator::Euler);
        assert_eq!(opt.solver, MjcfSolverType::Newton);
        assert_eq!(opt.iterations, 100);
        assert_eq!(opt.cone, MjcfConeType::Pyramidal);
        assert!(opt.flag.contact);
        assert!(opt.flag.gravity);
    }

    #[test]
    fn test_parse_option_override_arrays() {
        let xml = r#"
            <mujoco model="test">
                <option
                    o_solimp="0.9 0.95 0.001 0.5 2"
                    o_solref="0.02 1"
                    o_friction="1 0.005 0.0001 0.0001 0.0001"
                />
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let opt = &model.option;

        let solimp = opt.o_solimp.expect("should have o_solimp");
        assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
        assert_relative_eq!(solimp[1], 0.95, epsilon = 1e-10);
        assert_relative_eq!(solimp[2], 0.001, epsilon = 1e-10);

        let solref = opt.o_solref.expect("should have o_solref");
        assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);
        assert_relative_eq!(solref[1], 1.0, epsilon = 1e-10);

        let friction = opt.o_friction.expect("should have o_friction");
        assert_relative_eq!(friction[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(friction[1], 0.005, epsilon = 1e-10);
    }

    #[test]
    fn test_option_default_values() {
        let xml = r#"
            <mujoco model="test">
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let opt = &model.option;

        // Check MuJoCo defaults
        assert_relative_eq!(opt.timestep, 0.002, epsilon = 1e-10);
        assert_relative_eq!(opt.gravity.z, -9.81, epsilon = 1e-10);
        assert_eq!(opt.integrator, MjcfIntegrator::Euler);
        assert_eq!(opt.solver, MjcfSolverType::Newton);
        assert_eq!(opt.iterations, 100);
        assert_relative_eq!(opt.tolerance, 1e-8, epsilon = 1e-15);
        assert_eq!(opt.ls_iterations, 50);
        assert_eq!(opt.ccd_iterations, 50);
        assert_eq!(opt.cone, MjcfConeType::Pyramidal);
        assert_eq!(opt.jacobian, MjcfJacobianType::Dense);
        assert_relative_eq!(opt.impratio, 1.0, epsilon = 1e-10);
        assert_relative_eq!(opt.density, 0.0, epsilon = 1e-10);
        assert!(opt.flag.contact);
        assert!(opt.flag.gravity);
        assert!(opt.flag.warmstart);
    }

    #[test]
    fn test_option_helper_methods() {
        let mut opt = MjcfOption::default();

        // Gravity enabled by default
        assert!(opt.gravity_enabled());
        assert!(opt.contacts_enabled());

        // Disable via flag
        opt.flag.gravity = false;
        assert!(!opt.gravity_enabled());

        opt.flag.contact = false;
        assert!(!opt.contacts_enabled());

        // Effective margin
        assert!(opt.effective_margin().is_none()); // Default is negative
        opt.o_margin = 0.001;
        assert_eq!(opt.effective_margin(), Some(0.001));
    }

    // ========================================================================
    // Default parsing tests
    // ========================================================================

    #[test]
    fn test_parse_default_joint() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.5" stiffness="10.0" armature="0.01"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let joint_defaults = default.joint.as_ref().expect("should have joint defaults");
        assert_relative_eq!(joint_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(joint_defaults.stiffness.unwrap(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(joint_defaults.armature.unwrap(), 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_default_geom() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <geom type="box" density="500" friction="0.8 0.01 0.001" rgba="1 0 0 1"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let geom_defaults = default.geom.as_ref().expect("should have geom defaults");
        assert_eq!(geom_defaults.geom_type, Some(MjcfGeomType::Box));
        assert_relative_eq!(geom_defaults.density.unwrap(), 500.0, epsilon = 1e-10);
        let friction = geom_defaults.friction.unwrap();
        assert_relative_eq!(friction.x, 0.8, epsilon = 1e-10);
        let rgba = geom_defaults.rgba.unwrap();
        assert_relative_eq!(rgba.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(rgba.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_default_actuator() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <motor gear="100" ctrlrange="-1 1" kp="50" kv="5" ctrllimited="true"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let actuator_defaults = default
            .actuator
            .as_ref()
            .expect("should have actuator defaults");
        assert_relative_eq!(actuator_defaults.gear.unwrap()[0], 100.0, epsilon = 1e-10);
        assert_eq!(actuator_defaults.ctrlrange, Some((-1.0, 1.0)));
        assert_relative_eq!(actuator_defaults.kp.unwrap(), 50.0, epsilon = 1e-10);
        assert_relative_eq!(actuator_defaults.kv.unwrap(), 5.0, epsilon = 1e-10);
        assert_eq!(actuator_defaults.ctrllimited, Some(true));
    }

    #[test]
    fn test_parse_default_tendon() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <tendon stiffness="1000" damping="10" width="0.01" limited="true" range="0 0.5"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let tendon_defaults = default
            .tendon
            .as_ref()
            .expect("should have tendon defaults");
        assert_relative_eq!(tendon_defaults.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.damping.unwrap(), 10.0, epsilon = 1e-10);
        assert_relative_eq!(tendon_defaults.width.unwrap(), 0.01, epsilon = 1e-10);
        assert_eq!(tendon_defaults.limited, Some(true));
        assert_eq!(tendon_defaults.range, Some((0.0, 0.5)));
    }

    #[test]
    fn test_parse_default_sensor() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <sensor noise="0.01" cutoff="100"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let sensor_defaults = default
            .sensor
            .as_ref()
            .expect("should have sensor defaults");
        assert_relative_eq!(sensor_defaults.noise.unwrap(), 0.01, epsilon = 1e-10);
        assert_relative_eq!(sensor_defaults.cutoff.unwrap(), 100.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_nested_defaults() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.1"/>
                    <default class="arm">
                        <joint damping="0.5" armature="0.01"/>
                        <geom rgba="0.8 0.2 0.2 1"/>
                    </default>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 2);

        // Root default
        let root = model
            .defaults
            .iter()
            .find(|d| d.class.is_empty())
            .expect("root");
        assert!(root.parent_class.is_none());
        let root_joint = root.joint.as_ref().expect("root joint");
        assert_relative_eq!(root_joint.damping.unwrap(), 0.1, epsilon = 1e-10);

        // Arm class
        let arm = model
            .defaults
            .iter()
            .find(|d| d.class == "arm")
            .expect("arm");
        assert_eq!(arm.parent_class, Some(String::new()));
        let arm_joint = arm.joint.as_ref().expect("arm joint");
        assert_relative_eq!(arm_joint.damping.unwrap(), 0.5, epsilon = 1e-10);
        assert_relative_eq!(arm_joint.armature.unwrap(), 0.01, epsilon = 1e-10);
        let arm_geom = arm.geom.as_ref().expect("arm geom");
        assert_relative_eq!(arm_geom.rgba.unwrap().x, 0.8, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_default_self_closing() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.5"/>
                    <geom density="500"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        assert!(default.joint.is_some());
        assert!(default.geom.is_some());
    }

    #[test]
    fn test_parse_default_mesh() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <mesh scale="0.001 0.001 0.001"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let mesh_defaults = default.mesh.as_ref().expect("should have mesh defaults");
        let scale = mesh_defaults.scale.unwrap();
        assert_relative_eq!(scale.x, 0.001, epsilon = 1e-10);
        assert_relative_eq!(scale.y, 0.001, epsilon = 1e-10);
        assert_relative_eq!(scale.z, 0.001, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_default_site() {
        let xml = r#"
            <mujoco model="test">
                <default>
                    <site type="sphere" size="0.02" rgba="0 1 0 1"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.defaults.len(), 1);

        let default = &model.defaults[0];
        let site_defaults = default.site.as_ref().expect("should have site defaults");
        assert_eq!(site_defaults.site_type, Some("sphere".to_string()));
        let size = site_defaults.size.as_ref().unwrap();
        assert_relative_eq!(size[0], 0.02, epsilon = 1e-10);
        let rgba = site_defaults.rgba.unwrap();
        assert_relative_eq!(rgba.y, 1.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Asset parsing tests (mesh)
    // ========================================================================

    #[test]
    fn test_parse_mesh_asset_from_file() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="robot_body" file="meshes/body.stl"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.meshes.len(), 1);

        let mesh = &model.meshes[0];
        assert_eq!(mesh.name, "robot_body");
        assert_eq!(mesh.file, Some("meshes/body.stl".to_string()));
        // Default scale is 1.0
        assert_relative_eq!(mesh.scale.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(mesh.scale.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(mesh.scale.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_mesh_asset_with_scale() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="scaled_mesh" file="model.obj" scale="0.001 0.001 0.001"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.meshes.len(), 1);

        let mesh = &model.meshes[0];
        assert_eq!(mesh.name, "scaled_mesh");
        assert_relative_eq!(mesh.scale.x, 0.001, epsilon = 1e-10);
        assert_relative_eq!(mesh.scale.y, 0.001, epsilon = 1e-10);
        assert_relative_eq!(mesh.scale.z, 0.001, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_mesh_asset_with_embedded_vertices() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="triangle" vertex="0 0 0 1 0 0 0 1 0"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.meshes.len(), 1);

        let mesh = &model.meshes[0];
        assert_eq!(mesh.name, "triangle");
        assert!(mesh.file.is_none());
        assert!(mesh.has_embedded_data());
        assert_eq!(mesh.vertex_count(), 3);

        let vertices = mesh.vertex.as_ref().unwrap();
        assert_eq!(vertices.len(), 9); // 3 vertices * 3 components
        assert_relative_eq!(vertices[0], 0.0, epsilon = 1e-10);
        assert_relative_eq!(vertices[3], 1.0, epsilon = 1e-10);
        assert_relative_eq!(vertices[7], 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_multiple_mesh_assets() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="mesh1" file="a.stl"/>
                    <mesh name="mesh2" file="b.stl" scale="2 2 2"/>
                    <mesh name="mesh3" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.meshes.len(), 3);

        assert_eq!(model.meshes[0].name, "mesh1");
        assert_eq!(model.meshes[1].name, "mesh2");
        assert_eq!(model.meshes[2].name, "mesh3");

        assert_relative_eq!(model.meshes[1].scale.x, 2.0, epsilon = 1e-10);
        assert_eq!(model.meshes[2].vertex_count(), 4);
    }

    #[test]
    fn test_parse_geom_with_mesh_type() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="cube" vertex="0 0 0 1 0 0 1 1 0 0 1 0 0 0 1 1 0 1 1 1 1 0 1 1"/>
                </asset>
                <worldbody>
                    <body name="link">
                        <geom type="mesh" mesh="cube"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.meshes.len(), 1);
        assert_eq!(model.worldbody.children.len(), 1);

        let body = &model.worldbody.children[0];
        assert_eq!(body.geoms.len(), 1);

        let geom = &body.geoms[0];
        assert_eq!(geom.geom_type, MjcfGeomType::Mesh);
        assert_eq!(geom.mesh, Some("cube".to_string()));
    }

    #[test]
    fn test_model_mesh_lookup() {
        let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="mesh_a" file="a.stl"/>
                    <mesh name="mesh_b" file="b.stl"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");

        assert!(model.mesh("mesh_a").is_some());
        assert!(model.mesh("mesh_b").is_some());
        assert!(model.mesh("nonexistent").is_none());

        let mesh_a = model.mesh("mesh_a").unwrap();
        assert_eq!(mesh_a.file, Some("a.stl".to_string()));
    }

    // ========================================================================
    // Equality constraint parsing tests
    // ========================================================================

    #[test]
    fn test_parse_connect_constraint_basic() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2" anchor="0.5 0 0"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.equality.connects.len(), 1);

        let connect = &model.equality.connects[0];
        assert_eq!(connect.body1, "body1");
        assert_eq!(connect.body2, Some("body2".to_string()));
        assert_relative_eq!(connect.anchor.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(connect.anchor.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(connect.anchor.z, 0.0, epsilon = 1e-10);
        assert!(connect.active);
    }

    #[test]
    fn test_parse_connect_constraint_with_name() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1"/>
                    <body name="link2"/>
                </worldbody>
                <equality>
                    <connect name="ball_joint" body1="link1" body2="link2"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let connect = &model.equality.connects[0];
        assert_eq!(connect.name, Some("ball_joint".to_string()));
    }

    #[test]
    fn test_parse_connect_constraint_to_world() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="floating_body"/>
                </worldbody>
                <equality>
                    <connect body1="floating_body" anchor="0 0 1"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let connect = &model.equality.connects[0];
        assert_eq!(connect.body1, "floating_body");
        assert!(connect.body2.is_none()); // Defaults to world
        assert_relative_eq!(connect.anchor.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_connect_constraint_with_solver_params() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1"/>
                    <body name="body2"/>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2"
                             solref="0.02 1"
                             solimp="0.9 0.95 0.001 0.5 2"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let connect = &model.equality.connects[0];

        let solref = connect.solref.expect("should have solref");
        assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);
        assert_relative_eq!(solref[1], 1.0, epsilon = 1e-10);

        let solimp = connect.solimp.expect("should have solimp");
        assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
        assert_relative_eq!(solimp[1], 0.95, epsilon = 1e-10);
        assert_relative_eq!(solimp[2], 0.001, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_connect_constraint_inactive() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1"/>
                    <body name="body2"/>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2" active="false"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let connect = &model.equality.connects[0];
        assert!(!connect.active);
    }

    #[test]
    fn test_parse_multiple_connect_constraints() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="a"/>
                    <body name="b"/>
                    <body name="c"/>
                </worldbody>
                <equality>
                    <connect name="c1" body1="a" body2="b"/>
                    <connect name="c2" body1="b" body2="c"/>
                    <connect name="c3" body1="c" anchor="0 0 0"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.equality.connects.len(), 3);
        assert_eq!(model.equality.connects[0].name, Some("c1".to_string()));
        assert_eq!(model.equality.connects[1].name, Some("c2".to_string()));
        assert_eq!(model.equality.connects[2].name, Some("c3".to_string()));
    }

    #[test]
    fn test_parse_connect_constraint_missing_body1() {
        let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <equality>
                    <connect body2="body2"/>
                </equality>
            </mujoco>
        "#;

        let result = parse_mjcf_str(xml);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, MjcfError::MissingAttribute { .. }));
    }

    #[test]
    fn test_parse_empty_equality() {
        let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <equality/>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert!(model.equality.is_empty());
    }

    #[test]
    fn test_equality_len() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="a"/>
                    <body name="b"/>
                </worldbody>
                <equality>
                    <connect body1="a" body2="b"/>
                    <connect body1="b"/>
                </equality>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.equality.len(), 2);
        assert!(!model.equality.is_empty());
    }

    // ========================================================================
    // Cylinder, Muscle, and Adhesion actuator parsing tests
    // ========================================================================

    #[test]
    fn test_parse_cylinder_actuator() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <cylinder name="cyl1" joint="joint1" area="0.002" timeconst="0.5" bias="1 2 3"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.actuators.len(), 1);

        let cyl = &model.actuators[0];
        assert_eq!(cyl.name, "cyl1");
        assert_eq!(cyl.actuator_type, MjcfActuatorType::Cylinder);
        assert_eq!(cyl.joint, Some("joint1".to_string()));
        assert_relative_eq!(cyl.area, 0.002, epsilon = 1e-10);
        assert_eq!(cyl.timeconst, Some(0.5));
        assert_relative_eq!(cyl.bias[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(cyl.bias[1], 2.0, epsilon = 1e-10);
        assert_relative_eq!(cyl.bias[2], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_cylinder_actuator_with_diameter() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <cylinder name="cyl1" joint="joint1" diameter="0.05"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let cyl = &model.actuators[0];
        assert!(cyl.diameter.is_some());
        assert_relative_eq!(cyl.diameter.unwrap(), 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_muscle_actuator() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <muscle name="muscle1" joint="joint1" timeconst="0.02 0.06" range="0.8 1.1" force="500" scale="300" lmin="0.4" lmax="1.8" vmax="2.0" fpmax="1.5" fvmax="1.3"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.actuators.len(), 1);

        let muscle = &model.actuators[0];
        assert_eq!(muscle.name, "muscle1");
        assert_eq!(muscle.actuator_type, MjcfActuatorType::Muscle);
        assert_eq!(muscle.joint, Some("joint1".to_string()));

        // Muscle-specific attributes
        assert_relative_eq!(muscle.muscle_timeconst.0, 0.02, epsilon = 1e-10);
        assert_relative_eq!(muscle.muscle_timeconst.1, 0.06, epsilon = 1e-10);
        assert_relative_eq!(muscle.range.0, 0.8, epsilon = 1e-10);
        assert_relative_eq!(muscle.range.1, 1.1, epsilon = 1e-10);
        assert_relative_eq!(muscle.force, 500.0, epsilon = 1e-10);
        assert_relative_eq!(muscle.scale, 300.0, epsilon = 1e-10);
        assert_relative_eq!(muscle.lmin, 0.4, epsilon = 1e-10);
        assert_relative_eq!(muscle.lmax, 1.8, epsilon = 1e-10);
        assert_relative_eq!(muscle.vmax, 2.0, epsilon = 1e-10);
        assert_relative_eq!(muscle.fpmax, 1.5, epsilon = 1e-10);
        assert_relative_eq!(muscle.fvmax, 1.3, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_muscle_actuator_defaults() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <muscle name="muscle1" joint="joint1"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let muscle = &model.actuators[0];

        // Check MuJoCo default values
        assert_relative_eq!(muscle.muscle_timeconst.0, 0.01, epsilon = 1e-10);
        assert_relative_eq!(muscle.muscle_timeconst.1, 0.04, epsilon = 1e-10);
        assert_relative_eq!(muscle.range.0, 0.75, epsilon = 1e-10);
        assert_relative_eq!(muscle.range.1, 1.05, epsilon = 1e-10);
        assert_relative_eq!(muscle.force, -1.0, epsilon = 1e-10); // Negative triggers auto computation
        assert_relative_eq!(muscle.scale, 200.0, epsilon = 1e-10);
        assert_relative_eq!(muscle.lmin, 0.5, epsilon = 1e-10);
        assert_relative_eq!(muscle.lmax, 1.6, epsilon = 1e-10);
        assert_relative_eq!(muscle.vmax, 1.5, epsilon = 1e-10);
        assert_relative_eq!(muscle.fpmax, 1.3, epsilon = 1e-10);
        assert_relative_eq!(muscle.fvmax, 1.2, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_adhesion_actuator() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="gripper">
                        <geom type="box" size="0.05 0.05 0.01"/>
                    </body>
                </worldbody>
                <actuator>
                    <adhesion name="grip1" body="gripper" gain="100" ctrlrange="0 1"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.actuators.len(), 1);

        let adhesion = &model.actuators[0];
        assert_eq!(adhesion.name, "grip1");
        assert_eq!(adhesion.actuator_type, MjcfActuatorType::Adhesion);
        assert_eq!(adhesion.body, Some("gripper".to_string()));
        assert_relative_eq!(adhesion.gain, 100.0, epsilon = 1e-10);
        assert_eq!(adhesion.ctrlrange, Some((0.0, 1.0)));
    }

    #[test]
    fn test_parse_mixed_actuator_types() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <joint name="j1" type="hinge"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                    <body name="link2">
                        <joint name="j2" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                    <body name="gripper">
                        <geom type="box" size="0.05 0.05 0.01"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="j1" gear="50"/>
                    <position name="pos1" joint="j1" kp="100"/>
                    <cylinder name="cyl1" joint="j2" area="0.001"/>
                    <muscle name="mus1" joint="j1" force="200"/>
                    <adhesion name="adh1" body="gripper" gain="50"/>
                </actuator>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.actuators.len(), 5);

        assert_eq!(model.actuators[0].actuator_type, MjcfActuatorType::Motor);
        assert_eq!(model.actuators[1].actuator_type, MjcfActuatorType::Position);
        assert_eq!(model.actuators[2].actuator_type, MjcfActuatorType::Cylinder);
        assert_eq!(model.actuators[3].actuator_type, MjcfActuatorType::Muscle);
        assert_eq!(model.actuators[4].actuator_type, MjcfActuatorType::Adhesion);
    }

    #[test]
    fn test_actuator_type_as_str() {
        assert_eq!(MjcfActuatorType::Motor.as_str(), "motor");
        assert_eq!(MjcfActuatorType::Position.as_str(), "position");
        assert_eq!(MjcfActuatorType::Velocity.as_str(), "velocity");
        assert_eq!(MjcfActuatorType::General.as_str(), "general");
        assert_eq!(MjcfActuatorType::Muscle.as_str(), "muscle");
        assert_eq!(MjcfActuatorType::Cylinder.as_str(), "cylinder");
        assert_eq!(MjcfActuatorType::Damper.as_str(), "damper");
        assert_eq!(MjcfActuatorType::Adhesion.as_str(), "adhesion");
    }

    // ========================================================================
    // Tendon parsing tests
    // ========================================================================

    #[test]
    fn test_parse_spatial_tendon() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <site name="s1" pos="0 0 0"/>
                    </body>
                    <body name="link2">
                        <site name="s2" pos="0 0 0.5"/>
                    </body>
                    <body name="link3">
                        <site name="s3" pos="0 0 1"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable1" stiffness="1000" damping="10" width="0.005">
                        <site site="s1"/>
                        <site site="s2"/>
                        <site site="s3"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.tendons.len(), 1);

        let tendon = &model.tendons[0];
        assert_eq!(tendon.name, "cable1");
        assert_eq!(tendon.tendon_type, MjcfTendonType::Spatial);
        assert_relative_eq!(tendon.stiffness, 1000.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.damping, 10.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.width, 0.005, epsilon = 1e-10);
        assert_eq!(tendon.path_elements.len(), 3);
        assert_eq!(
            tendon.path_elements[0],
            SpatialPathElement::Site {
                site: "s1".to_string()
            }
        );
        assert_eq!(
            tendon.path_elements[1],
            SpatialPathElement::Site {
                site: "s2".to_string()
            }
        );
        assert_eq!(
            tendon.path_elements[2],
            SpatialPathElement::Site {
                site: "s3".to_string()
            }
        );
    }

    #[test]
    fn test_parse_fixed_tendon() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="link2">
                        <joint name="j2" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="coupling" limited="true" range="0 0.5">
                        <joint joint="j1" coef="1"/>
                        <joint joint="j2" coef="-1"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.tendons.len(), 1);

        let tendon = &model.tendons[0];
        assert_eq!(tendon.name, "coupling");
        assert_eq!(tendon.tendon_type, MjcfTendonType::Fixed);
        assert!(tendon.limited);
        assert_eq!(tendon.range, Some((0.0, 0.5)));
        assert_eq!(
            tendon.joints,
            vec![("j1".to_string(), 1.0), ("j2".to_string(), -1.0)]
        );
    }

    #[test]
    fn test_parse_multiple_tendons() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <site name="s1" pos="0 0 0"/>
                        <site name="s2" pos="0 0 0.5"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="t1">
                        <site site="s1"/>
                        <site site="s2"/>
                    </spatial>
                    <fixed name="t2">
                        <joint joint="j1" coef="2"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.tendons.len(), 2);
        assert_eq!(model.tendons[0].name, "t1");
        assert_eq!(model.tendons[0].tendon_type, MjcfTendonType::Spatial);
        assert_eq!(model.tendons[1].name, "t2");
        assert_eq!(model.tendons[1].tendon_type, MjcfTendonType::Fixed);
    }

    #[test]
    fn test_parse_tendon_with_rgba() {
        let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <tendon>
                    <spatial name="colored" rgba="1 0 0 1"/>
                </tendon>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let tendon = &model.tendons[0];
        assert_relative_eq!(tendon.rgba.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.rgba.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.rgba.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(tendon.rgba.w, 1.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Sensor parsing tests
    // ========================================================================

    #[test]
    fn test_parse_joint_sensors() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos name="j1_pos" joint="j1"/>
                    <jointvel name="j1_vel" joint="j1" noise="0.01"/>
                </sensor>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.sensors.len(), 2);

        let pos_sensor = &model.sensors[0];
        assert_eq!(pos_sensor.name, "j1_pos");
        assert_eq!(pos_sensor.sensor_type, MjcfSensorType::Jointpos);
        assert_eq!(pos_sensor.objname, Some("j1".to_string()));
        assert_relative_eq!(pos_sensor.noise, 0.0, epsilon = 1e-10);

        let vel_sensor = &model.sensors[1];
        assert_eq!(vel_sensor.name, "j1_vel");
        assert_eq!(vel_sensor.sensor_type, MjcfSensorType::Jointvel);
        assert_relative_eq!(vel_sensor.noise, 0.01, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_imu_sensors() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <site name="imu_site" pos="0 0 0"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <accelerometer name="accel" site="imu_site"/>
                    <gyro name="gyro" site="imu_site" noise="0.001"/>
                </sensor>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.sensors.len(), 2);

        let accel = &model.sensors[0];
        assert_eq!(accel.name, "accel");
        assert_eq!(accel.sensor_type, MjcfSensorType::Accelerometer);
        assert_eq!(accel.objname, Some("imu_site".to_string()));

        let gyro = &model.sensors[1];
        assert_eq!(gyro.name, "gyro");
        assert_eq!(gyro.sensor_type, MjcfSensorType::Gyro);
        assert_relative_eq!(gyro.noise, 0.001, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_force_sensors() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <site name="ft_site" pos="0 0 0"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <force name="force1" site="ft_site"/>
                    <torque name="torque1" site="ft_site"/>
                    <touch name="contact1" site="ft_site"/>
                </sensor>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        assert_eq!(model.sensors.len(), 3);

        assert_eq!(model.sensors[0].sensor_type, MjcfSensorType::Force);
        assert_eq!(model.sensors[1].sensor_type, MjcfSensorType::Torque);
        assert_eq!(model.sensors[2].sensor_type, MjcfSensorType::Touch);
    }

    #[test]
    fn test_parse_sensor_with_cutoff() {
        let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos name="filtered" joint="j1" cutoff="50"/>
                </sensor>
            </mujoco>
        "#;

        let model = parse_mjcf_str(xml).expect("should parse");
        let sensor = &model.sensors[0];
        assert_relative_eq!(sensor.cutoff, 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_type_enum() {
        // Position sensors
        assert_eq!(
            MjcfSensorType::from_str("jointpos"),
            Some(MjcfSensorType::Jointpos)
        );
        assert_eq!(
            MjcfSensorType::from_str("tendonpos"),
            Some(MjcfSensorType::Tendonpos)
        );

        // Velocity sensors
        assert_eq!(
            MjcfSensorType::from_str("jointvel"),
            Some(MjcfSensorType::Jointvel)
        );

        // IMU sensors
        assert_eq!(
            MjcfSensorType::from_str("accelerometer"),
            Some(MjcfSensorType::Accelerometer)
        );
        assert_eq!(MjcfSensorType::from_str("gyro"), Some(MjcfSensorType::Gyro));

        // Force sensors
        assert_eq!(
            MjcfSensorType::from_str("force"),
            Some(MjcfSensorType::Force)
        );
        assert_eq!(
            MjcfSensorType::from_str("torque"),
            Some(MjcfSensorType::Torque)
        );
        assert_eq!(
            MjcfSensorType::from_str("touch"),
            Some(MjcfSensorType::Touch)
        );

        // Unknown
        assert_eq!(MjcfSensorType::from_str("invalid"), None);
    }

    #[test]
    fn test_sensor_dimensionality() {
        // Scalar sensors
        assert_eq!(MjcfSensorType::Jointpos.dim(), 1);
        assert_eq!(MjcfSensorType::Jointvel.dim(), 1);

        // 3D sensors
        assert_eq!(MjcfSensorType::Accelerometer.dim(), 3);
        assert_eq!(MjcfSensorType::Gyro.dim(), 3);
        assert_eq!(MjcfSensorType::Force.dim(), 3);

        // 4D sensors (quaternion)
        assert_eq!(MjcfSensorType::Framequat.dim(), 4);
        assert_eq!(MjcfSensorType::Ballquat.dim(), 4);
    }
}
