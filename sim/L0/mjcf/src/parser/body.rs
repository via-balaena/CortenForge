//! `<worldbody>` / `<body>` tree parsing (joints, geoms, sites, frames).

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    FluidShape, MjcfBody, MjcfFrame, MjcfGeom, MjcfGeomType, MjcfInertial, MjcfJoint,
    MjcfJointType, MjcfSite,
};

use super::attrs::{
    attr_bool, attr_fixed, attr_vec3, attr_vec4, get_attribute_opt, parse_float_array,
    parse_float_attr, parse_int_attr, parse_vector3, parse_vector4, safe_normalize_axis,
    skip_element,
};
use super::composite::parse_composite;
use super::extension::{parse_plugin_ref, parse_plugin_ref_empty};

/// Parse worldbody element.
pub(super) fn parse_worldbody<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfBody> {
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
                    b"frame" => {
                        let frame = parse_frame(reader, e)?;
                        worldbody.frames.push(frame);
                    }
                    b"composite" => {
                        let composite = parse_composite(reader, e)?;
                        worldbody.composites.push(composite);
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
                b"frame" => {
                    let frame = parse_frame_attrs(e)?;
                    worldbody.frames.push(frame);
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
pub(super) fn parse_body<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfBody> {
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
                        // Auto-generate name only when the parent body has a
                        // name. Anonymous joints in anonymous bodies stay
                        // empty — mirrors the parser's anonymous-body fix
                        // (commit `c7fa0c25`); validation.rs:211 exempts
                        // empty joint names from the dedup check.
                        if joint.name.is_empty() && !body.name.is_empty() {
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
                        if joint.name.is_empty() && !body.name.is_empty() {
                            joint.name = format!("{}_joint{}", body.name, joint_counter);
                            joint_counter += 1;
                        }
                        joint.body = Some(body.name.clone());
                        body.joints.push(joint);
                        // Skip to end tag (freejoint has no children)
                        skip_element(reader, &elem_name)?;
                    }
                    b"frame" => {
                        let frame = parse_frame(reader, e)?;
                        body.frames.push(frame);
                    }
                    b"composite" => {
                        let composite = parse_composite(reader, e)?;
                        body.composites.push(composite);
                    }
                    b"plugin" => {
                        let plugin_ref = parse_plugin_ref(reader, e)?;
                        body.plugin = Some(plugin_ref);
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"joint" => {
                    let mut joint = parse_joint_attrs(e)?;
                    if joint.name.is_empty() && !body.name.is_empty() {
                        joint.name = format!("{}_joint{}", body.name, joint_counter);
                        joint_counter += 1;
                    }
                    joint.body = Some(body.name.clone());
                    body.joints.push(joint);
                }
                b"freejoint" => {
                    // <freejoint/> is MuJoCo shorthand for <joint type="free"/>
                    let mut joint = parse_freejoint_attrs(e)?;
                    if joint.name.is_empty() && !body.name.is_empty() {
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
                b"frame" => {
                    let frame = parse_frame_attrs(e)?;
                    body.frames.push(frame);
                }
                b"plugin" => {
                    let plugin_ref = parse_plugin_ref_empty(e)?;
                    body.plugin = Some(plugin_ref);
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
pub(super) fn parse_body_attrs(e: &BytesStart) -> Result<MjcfBody> {
    // Empty string for missing name; matches the builder's anonymous-body
    // contract (body.rs:127, 200 — name lookup and Option<String> mapping
    // both use is_empty() as the anonymous sentinel). The previous "unnamed"
    // placeholder collided across multiple anonymous bodies.
    let name = get_attribute_opt(e, "name").unwrap_or_default();
    let mut body = MjcfBody::new(name);

    if let Some(pos) = get_attribute_opt(e, "pos") {
        body.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        body.quat = parse_vector4(&quat)?;
    }
    body.euler = attr_vec3(e, "euler")?;
    body.axisangle = attr_vec4(e, "axisangle")?;
    body.childclass = get_attribute_opt(e, "childclass");
    if let Some(mocap) = get_attribute_opt(e, "mocap") {
        body.mocap = mocap == "true";
    }
    if let Some(sleep) = get_attribute_opt(e, "sleep") {
        body.sleep = Some(sleep);
    }
    if let Some(gc) = get_attribute_opt(e, "gravcomp") {
        body.gravcomp = Some(gc.parse::<f64>().map_err(|_| {
            crate::error::MjcfError::invalid_attribute(
                "gravcomp",
                "body",
                format!("expected float, got '{gc}'"),
            )
        })?);
    }
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            body.user = parts;
        }
    }

    Ok(body)
}

/// Parse joint element.
pub(super) fn parse_joint<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfJoint> {
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
pub(super) fn parse_joint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let mut joint = MjcfJoint::default();

    joint.name = get_attribute_opt(e, "name").unwrap_or_default();
    joint.class = get_attribute_opt(e, "class");

    if let Some(jtype) = get_attribute_opt(e, "type") {
        joint.joint_type = Some(
            MjcfJointType::from_str(&jtype).ok_or_else(|| MjcfError::UnknownJointType(jtype))?,
        );
    }

    joint.pos = attr_vec3(e, "pos")?;
    if let Some(axis) = get_attribute_opt(e, "axis") {
        joint.axis = Some(safe_normalize_axis(parse_vector3(&axis)?));
    }
    joint.limited = attr_bool(e, "limited");
    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            joint.range = Some((parts[0], parts[1]));
        }
    }
    joint.ref_pos = parse_float_attr(e, "ref");
    joint.spring_ref = parse_float_attr(e, "springref");
    joint.damping = parse_float_attr(e, "damping");
    joint.stiffness = parse_float_attr(e, "stiffness");
    joint.armature = parse_float_attr(e, "armature");
    joint.frictionloss = parse_float_attr(e, "frictionloss");
    joint.group = parse_int_attr(e, "group");

    // Joint limit solver parameters: solreflimit=[timeconst, dampratio],
    // solimplimit=[d0, d_width, width, midpoint, power]
    joint.solref_limit = attr_fixed::<2>(e, "solreflimit")?;
    joint.solimp_limit = attr_fixed::<5>(e, "solimplimit")?;

    // Joint friction loss solver parameters: solreffriction=[timeconst, dampratio],
    // solimpfriction=[d0, d_width, width, midpoint, power]
    joint.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    joint.solimpfriction = attr_fixed::<5>(e, "solimpfriction")?;

    // Gravity compensation routing (S4.2a).
    joint.actuatorgravcomp = attr_bool(e, "actuatorgravcomp");

    joint.margin = parse_float_attr(e, "margin");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            joint.user = parts;
        }
    }

    Ok(joint)
}

/// Parse freejoint attributes (MuJoCo shorthand for `<joint type="free"/>`).
///
/// The `<freejoint/>` element is a convenience shorthand in MuJoCo that creates
/// a 6-DOF free joint. It only supports `name` and `group` attributes.
pub(super) fn parse_freejoint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let mut joint = MjcfJoint::default();
    joint.joint_type = Some(MjcfJointType::Free);
    joint.name = get_attribute_opt(e, "name").unwrap_or_default();
    // Note: MuJoCo's freejoint also supports 'group' attribute, but we don't use it
    Ok(joint)
}

/// Parse geom element.
pub(super) fn parse_geom<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfGeom> {
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
pub(super) fn parse_geom_attrs(e: &BytesStart) -> Result<MjcfGeom> {
    let mut geom = MjcfGeom::default();

    geom.name = get_attribute_opt(e, "name");
    geom.class = get_attribute_opt(e, "class");

    if let Some(gtype) = get_attribute_opt(e, "type") {
        geom.geom_type =
            Some(MjcfGeomType::from_str(&gtype).ok_or_else(|| MjcfError::UnknownGeomType(gtype))?);
    }

    geom.pos = attr_vec3(e, "pos")?;
    geom.quat = attr_vec4(e, "quat")?;
    geom.euler = attr_vec3(e, "euler")?;
    geom.axisangle = attr_vec4(e, "axisangle")?;
    geom.xyaxes = attr_fixed::<6>(e, "xyaxes")?;
    geom.zaxis = attr_vec3(e, "zaxis")?;
    if let Some(size) = get_attribute_opt(e, "size") {
        geom.size = parse_float_array(&size)?;
    }
    geom.fromto = attr_fixed::<6>(e, "fromto")?;
    geom.friction = attr_vec3(e, "friction")?;
    geom.density = parse_float_attr(e, "density");
    geom.mass = parse_float_attr(e, "mass");

    geom.rgba = attr_vec4(e, "rgba")?;
    geom.contype = parse_int_attr(e, "contype");
    geom.conaffinity = parse_int_attr(e, "conaffinity");
    geom.condim = parse_int_attr(e, "condim");
    geom.mesh = get_attribute_opt(e, "mesh");
    geom.hfield = get_attribute_opt(e, "hfield");

    // Contact solver parameters: solref=[timeconst, dampratio],
    // solimp=[d0, d_width, width, midpoint, power].
    // When two geoms collide, their params are combined via mj_contactParam:
    // friction = element-wise max, solref/solimp = solmix-weighted average, condim = max.
    geom.solref = attr_fixed::<2>(e, "solref")?;
    geom.solimp = attr_fixed::<5>(e, "solimp")?;

    // Contact parameter combination attributes (MuJoCo mj_contactParam)
    geom.priority = parse_int_attr(e, "priority");
    geom.solmix = parse_float_attr(e, "solmix");
    geom.margin = parse_float_attr(e, "margin");
    geom.gap = parse_float_attr(e, "gap");
    geom.group = parse_int_attr(e, "group");
    geom.material = get_attribute_opt(e, "material");

    // Fluid force parameters
    if let Some(fs) = get_attribute_opt(e, "fluidshape") {
        geom.fluidshape = Some(match fs.as_str() {
            "none" => FluidShape::None,
            "ellipsoid" => FluidShape::Ellipsoid,
            _ => return Err(MjcfError::InvalidFluidShape(fs)),
        });
    }
    if let Some(coef_str) = get_attribute_opt(e, "fluidcoef") {
        let parts = parse_float_array(&coef_str)?;
        if parts.len() != 5 {
            return Err(MjcfError::InvalidFluidCoef(parts.len()));
        }
        geom.fluidcoef = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
    }
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            geom.user = parts;
        }
    }

    // Parse shellinertia (boolean attribute)
    geom.shellinertia = attr_bool(e, "shellinertia");

    Ok(geom)
}

/// Parse inertial element.
pub(super) fn parse_inertial<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfInertial> {
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
pub(super) fn parse_inertial_attrs(e: &BytesStart) -> Result<MjcfInertial> {
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
    inertial.diaginertia = attr_vec3(e, "diaginertia")?;
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
pub(super) fn parse_site<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfSite> {
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
pub(super) fn parse_site_attrs(e: &BytesStart) -> Result<MjcfSite> {
    let mut site = MjcfSite::default();

    site.name = get_attribute_opt(e, "name").unwrap_or_default();
    site.class = get_attribute_opt(e, "class");

    site.site_type = get_attribute_opt(e, "type");
    site.pos = attr_vec3(e, "pos")?;
    site.quat = attr_vec4(e, "quat")?;
    site.euler = attr_vec3(e, "euler")?;
    site.axisangle = attr_vec4(e, "axisangle")?;
    site.xyaxes = attr_fixed::<6>(e, "xyaxes")?;
    site.zaxis = attr_vec3(e, "zaxis")?;
    if let Some(size) = get_attribute_opt(e, "size") {
        site.size = Some(parse_float_array(&size)?);
    }
    site.rgba = attr_vec4(e, "rgba")?;
    site.group = parse_int_attr(e, "group");
    site.material = get_attribute_opt(e, "material");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            site.user = parts;
        }
    }

    Ok(site)
}

/// Parse frame element (non-self-closing).
pub(super) fn parse_frame<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfFrame> {
    let mut frame = parse_frame_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"body" => {
                        let body = parse_body(reader, e)?;
                        frame.bodies.push(body);
                    }
                    b"geom" => {
                        let geom = parse_geom(reader, e)?;
                        frame.geoms.push(geom);
                    }
                    b"site" => {
                        let site = parse_site(reader, e)?;
                        frame.sites.push(site);
                    }
                    b"frame" => {
                        let child_frame = parse_frame(reader, e)?;
                        frame.frames.push(child_frame);
                    }
                    b"camera" | b"light" => {
                        tracing::warn!(
                            "skipping <{}> inside <frame>: camera/light not yet supported",
                            String::from_utf8_lossy(&elem_name)
                        );
                        skip_element(reader, &elem_name)?;
                    }
                    b"joint" | b"freejoint" | b"inertial" => {
                        return Err(MjcfError::InvalidElement(format!(
                            "<{}> is not allowed inside <frame>",
                            String::from_utf8_lossy(&elem_name)
                        )));
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"geom" => {
                    let geom = parse_geom_attrs(e)?;
                    frame.geoms.push(geom);
                }
                b"site" => {
                    let site = parse_site_attrs(e)?;
                    frame.sites.push(site);
                }
                b"frame" => {
                    let child_frame = parse_frame_attrs(e)?;
                    frame.frames.push(child_frame);
                }
                b"camera" | b"light" => {
                    tracing::warn!(
                        "skipping <{}/> inside <frame>: camera/light not yet supported",
                        String::from_utf8_lossy(e.name().as_ref())
                    );
                }
                b"joint" | b"freejoint" | b"inertial" => {
                    return Err(MjcfError::InvalidElement(format!(
                        "<{}/> is not allowed inside <frame>",
                        String::from_utf8_lossy(e.name().as_ref())
                    )));
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"frame" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in frame".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(frame)
}

/// Parse frame attributes only (for self-closing `<frame ... />`).
pub(super) fn parse_frame_attrs(e: &BytesStart) -> Result<MjcfFrame> {
    let mut frame = MjcfFrame::default();

    frame.name = get_attribute_opt(e, "name");
    frame.childclass = get_attribute_opt(e, "childclass");

    if let Some(pos) = get_attribute_opt(e, "pos") {
        frame.pos = parse_vector3(&pos)?;
    }
    if let Some(quat) = get_attribute_opt(e, "quat") {
        frame.quat = parse_vector4(&quat)?;
    }
    frame.euler = attr_vec3(e, "euler")?;
    frame.axisangle = attr_vec4(e, "axisangle")?;
    frame.xyaxes = attr_fixed::<6>(e, "xyaxes")?;
    frame.zaxis = attr_vec3(e, "zaxis")?;

    Ok(frame)
}
