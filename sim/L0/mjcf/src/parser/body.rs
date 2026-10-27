//! `<worldbody>` / `<body>` tree parsing (joints, geoms, sites, frames).

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    FluidShape, MjcfBody, MjcfFrame, MjcfGeom, MjcfGeomType, MjcfInertial, MjcfJoint,
    MjcfJointType, MjcfSite,
};

use super::attrs::{Attrs, safe_normalize_axis, skip_element};
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
                return Err(MjcfError::eof_in("worldbody"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("body")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
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
    let a = Attrs::new(e)?;
    let name = a.string("name").unwrap_or_default();
    let mut body = MjcfBody::new(name);

    if let Some(pos) = a.vec3("pos")? {
        body.pos = pos;
    }
    if let Some(quat) = a.vec4("quat")? {
        body.quat = quat;
    }
    body.euler = a.vec3("euler")?;
    body.axisangle = a.vec4("axisangle")?;
    body.childclass = a.string("childclass");
    if let Some(mocap) = a.is_true("mocap") {
        body.mocap = mocap;
    }
    if let Some(sleep) = a.string("sleep") {
        body.sleep = Some(sleep);
    }
    if let Some(gc) = a.real("gravcomp")? {
        body.gravcomp = Some(gc);
    }
    if let Some(user) = a.vector("user")? {
        body.user = user;
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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("joint")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(joint)
}

/// Parse joint attributes only.
pub(super) fn parse_joint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let a = Attrs::new(e)?;
    let mut joint = MjcfJoint::default();

    joint.name = a.string("name").unwrap_or_default();
    joint.class = a.string("class");

    if let Some(jtype) = a.text("type") {
        joint.joint_type = Some(
            MjcfJointType::from_str(jtype)
                .ok_or_else(|| MjcfError::invalid_keyword("joint", "type", jtype))?,
        );
    }

    joint.pos = a.vec3("pos")?;
    if let Some(axis) = a.vec3("axis")? {
        joint.axis = Some(safe_normalize_axis(axis));
    }
    joint.limited = a.is_true("limited");
    if let Some(pair) = a.array::<2>("range")? {
        joint.range = Some(pair.into());
    }
    joint.ref_pos = a.real("ref")?;
    joint.spring_ref = a.real("springref")?;
    joint.damping = a.real("damping")?;
    joint.stiffness = a.real("stiffness")?;
    joint.armature = a.real("armature")?;
    joint.frictionloss = a.real("frictionloss")?;
    joint.group = a.int("group")?;

    // Joint limit solver parameters: solreflimit=[timeconst, dampratio],
    // solimplimit=[d0, d_width, width, midpoint, power]
    joint.solref_limit = a.prefix("solreflimit")?;
    joint.solimp_limit = a.prefix("solimplimit")?;

    // Joint friction loss solver parameters: solreffriction=[timeconst, dampratio],
    // solimpfriction=[d0, d_width, width, midpoint, power]
    joint.solreffriction = a.prefix("solreffriction")?;
    joint.solimpfriction = a.prefix("solimpfriction")?;

    // Gravity compensation routing (S4.2a).
    joint.actuatorgravcomp = a.is_true("actuatorgravcomp");

    joint.margin = a.real("margin")?;
    if let Some(user) = a.vector("user")? {
        joint.user = user;
    }

    Ok(joint)
}

/// Parse freejoint attributes (MuJoCo shorthand for `<joint type="free"/>`).
///
/// The `<freejoint/>` element is a convenience shorthand in MuJoCo that creates
/// a 6-DOF free joint. It only supports `name` and `group` attributes.
pub(super) fn parse_freejoint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let a = Attrs::new(e)?;
    let mut joint = MjcfJoint::default();
    joint.joint_type = Some(MjcfJointType::Free);
    joint.name = a.string("name").unwrap_or_default();
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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("geom")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(geom)
}

/// Parse geom attributes only.
pub(super) fn parse_geom_attrs(e: &BytesStart) -> Result<MjcfGeom> {
    let a = Attrs::new(e)?;
    let mut geom = MjcfGeom::default();

    geom.name = a.string("name");
    geom.class = a.string("class");

    if let Some(gtype) = a.text("type") {
        geom.geom_type = Some(
            MjcfGeomType::from_str(gtype)
                .ok_or_else(|| MjcfError::invalid_keyword("geom", "type", gtype))?,
        );
    }

    geom.pos = a.vec3("pos")?;
    geom.quat = a.vec4("quat")?;
    geom.euler = a.vec3("euler")?;
    geom.axisangle = a.vec4("axisangle")?;
    geom.xyaxes = a.array("xyaxes")?;
    geom.zaxis = a.vec3("zaxis")?;
    // One up to three numbers (MuJoCo's `exact = false`); the type stays a
    // list until the class overlay of a geom's size is MuJoCo's (book L18).
    if let Some(size) = a.prefix::<3>("size")? {
        geom.size = size.given().to_vec();
    }
    geom.fromto = a.array("fromto")?;
    geom.friction = a.prefix("friction")?;
    geom.density = a.real("density")?;
    geom.mass = a.real("mass")?;

    geom.rgba = a.vec4("rgba")?;
    geom.contype = a.int("contype")?;
    geom.conaffinity = a.int("conaffinity")?;
    geom.condim = a.int("condim")?;
    geom.mesh = a.string("mesh");
    geom.hfield = a.string("hfield");

    // Contact solver parameters: solref=[timeconst, dampratio],
    // solimp=[d0, d_width, width, midpoint, power].
    // When two geoms collide, their params are combined via mj_contactParam:
    // friction = element-wise max, solref/solimp = solmix-weighted average, condim = max.
    geom.solref = a.prefix("solref")?;
    geom.solimp = a.prefix("solimp")?;

    // Contact parameter combination attributes (MuJoCo mj_contactParam)
    geom.priority = a.int("priority")?;
    geom.solmix = a.real("solmix")?;
    geom.margin = a.real("margin")?;
    geom.gap = a.real("gap")?;
    geom.group = a.int("group")?;
    geom.material = a.string("material");

    // Fluid force parameters
    if let Some(fs) = a.text("fluidshape") {
        geom.fluidshape = Some(match fs {
            "none" => FluidShape::None,
            "ellipsoid" => FluidShape::Ellipsoid,
            _ => return Err(MjcfError::invalid_keyword("geom", "fluidshape", fs)),
        });
    }
    geom.fluidcoef = a.prefix("fluidcoef")?;
    if let Some(user) = a.vector("user")? {
        geom.user = user;
    }

    // Parse shellinertia (boolean attribute)
    geom.shellinertia = a.is_true("shellinertia");

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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("inertial")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(inertial)
}

/// Parse inertial attributes only.
pub(super) fn parse_inertial_attrs(e: &BytesStart) -> Result<MjcfInertial> {
    let a = Attrs::new(e)?;
    let mut inertial = MjcfInertial::default();

    if let Some(pos) = a.vec3("pos")? {
        inertial.pos = pos;
    }
    if let Some(quat) = a.vec4("quat")? {
        inertial.quat = quat;
    }
    if let Some(mass) = a.real("mass")? {
        inertial.mass = mass;
    }
    inertial.diaginertia = a.vec3("diaginertia")?;
    inertial.fullinertia = a.array("fullinertia")?;

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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("site")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(site)
}

/// Parse site attributes only.
pub(super) fn parse_site_attrs(e: &BytesStart) -> Result<MjcfSite> {
    let a = Attrs::new(e)?;
    let mut site = MjcfSite::default();

    site.name = a.string("name").unwrap_or_default();
    site.class = a.string("class");

    site.site_type = a.string("type");
    site.pos = a.vec3("pos")?;
    site.quat = a.vec4("quat")?;
    site.euler = a.vec3("euler")?;
    site.axisangle = a.vec4("axisangle")?;
    site.xyaxes = a.array("xyaxes")?;
    site.zaxis = a.vec3("zaxis")?;
    site.fromto = a.array("fromto")?;
    site.size = a.prefix("size")?;
    site.rgba = a.vec4("rgba")?;
    site.group = a.int("group")?;
    site.material = a.string("material");
    if let Some(user) = a.vector("user")? {
        site.user = user;
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
                        return Err(MjcfError::unsupported(format!(
                            "<{}> inside <frame>",
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
                    return Err(MjcfError::unsupported(format!(
                        "<{}/> inside <frame>",
                        String::from_utf8_lossy(e.name().as_ref())
                    )));
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"frame" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("frame")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(frame)
}

/// Parse frame attributes only (for self-closing `<frame ... />`).
pub(super) fn parse_frame_attrs(e: &BytesStart) -> Result<MjcfFrame> {
    let a = Attrs::new(e)?;
    let mut frame = MjcfFrame::default();

    frame.name = a.string("name");
    frame.childclass = a.string("childclass");

    if let Some(pos) = a.vec3("pos")? {
        frame.pos = pos;
    }
    if let Some(quat) = a.vec4("quat")? {
        frame.quat = quat;
    }
    frame.euler = a.vec3("euler")?;
    frame.axisangle = a.vec4("axisangle")?;
    frame.xyaxes = a.array("xyaxes")?;
    frame.zaxis = a.vec3("zaxis")?;

    Ok(frame)
}
