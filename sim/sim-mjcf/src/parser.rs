//! MJCF XML parser.
//!
//! Parses MJCF XML into the intermediate representation types.

use nalgebra::{Vector3, Vector4};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfBody, MjcfDefault, MjcfGeom, MjcfGeomDefaults,
    MjcfGeomType, MjcfInertial, MjcfJoint, MjcfJointDefaults, MjcfJointType, MjcfModel, MjcfOption,
    MjcfSite,
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
                    b"worldbody" => {
                        model.worldbody = parse_worldbody(reader)?;
                    }
                    b"actuator" => {
                        let actuators = parse_actuators(reader)?;
                        model.actuators = actuators;
                    }
                    // Skip other elements (asset, contact, equality, tendon, etc.)
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                // Handle self-closing elements
                if e.name().as_ref() == b"option" {
                    model.option = parse_option_attrs(e)?;
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
                    if let Some(contact) = get_attribute_opt(e, "contact") {
                        option.flag_contact = contact != "disable";
                    }
                    if let Some(gravity) = get_attribute_opt(e, "gravity") {
                        option.flag_gravity = gravity != "disable";
                    }
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

    if let Some(ts) = parse_float_attr(e, "timestep") {
        option.timestep = ts;
    }
    if let Some(grav) = get_attribute_opt(e, "gravity") {
        option.gravity = parse_vector3(&grav)?;
    }
    if let Some(integrator) = get_attribute_opt(e, "integrator") {
        option.integrator = integrator;
    }
    if let Some(iter) = parse_int_attr(e, "iterations") {
        option.iterations = iter as usize;
    }
    if let Some(tol) = parse_float_attr(e, "tolerance") {
        option.tolerance = tol;
    }

    Ok(option)
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
        joint.axis = parse_vector3(&axis)?.normalize();
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

    actuator.name = get_attribute_opt(e, "name").unwrap_or_default();
    actuator.class = get_attribute_opt(e, "class");
    actuator.joint = get_attribute_opt(e, "joint");
    actuator.site = get_attribute_opt(e, "site");
    actuator.tendon = get_attribute_opt(e, "tendon");

    if let Some(gear) = parse_float_attr(e, "gear") {
        actuator.gear = gear;
    }

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
        actuator.kv = kv;
    }

    Ok(actuator)
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
        assert_relative_eq!(motor.gear, 100.0, epsilon = 1e-10);

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
}
