//! `<worldbody>` / `<body>` tree parsing (joints, geoms, sites, frames).

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{
    MjcfBody, MjcfFrame, MjcfGeom, MjcfGeomType, MjcfInertial, MjcfJoint, MjcfJointType, MjcfSite,
};

use super::attrs::{Attrs, skip_element};
use super::composite::parse_composite;
use super::extension::parse_plugin_ref;

/// Parse worldbody element (`empty`: it closed itself, so it has no children).
pub(super) fn parse_worldbody<R: BufRead>(reader: &mut Reader<R>, empty: bool) -> Result<MjcfBody> {
    let mut worldbody = MjcfBody::new("world");
    if empty {
        return Ok(worldbody);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            // A self-closing element is the same element with no children, so
            // one arm reads both forms (MuJoCo's reader does not tell them apart).
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"body" => worldbody.children.push(parse_body(reader, e, child_empty)?),
                    b"geom" => {
                        // Static geom attached to world
                        let geom = parse_leaf(reader, e, child_empty, parse_geom_attrs)?;
                        worldbody.geoms.push(geom);
                    }
                    b"site" => {
                        let site = parse_leaf(reader, e, child_empty, parse_site_attrs)?;
                        worldbody.sites.push(site);
                    }
                    b"frame" => worldbody.frames.push(parse_frame(reader, e, child_empty)?),
                    b"composite" => {
                        let composite = parse_composite(reader, e, child_empty)?;
                        worldbody.composites.push(composite);
                    }
                    tag @ (b"replicate" | b"attach") => return Err(attach_refusal(tag)),
                    name => skip_element(reader, name, child_empty)?,
                }
            }
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

/// Parse body element (`empty`: it closed itself, so it has no children).
pub(super) fn parse_body<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfBody> {
    let mut body = parse_body_attrs(start)?;
    if empty {
        return Ok(body);
    }
    let mut buf = Vec::new();
    let mut joint_counter = 0;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"body" => body.children.push(parse_body(reader, e, child_empty)?),
                    tag @ (b"joint" | b"freejoint") => {
                        // <freejoint/> is MuJoCo shorthand for <joint type="free"/>
                        let read = if tag == b"joint" {
                            parse_joint_attrs
                        } else {
                            parse_freejoint_attrs
                        };
                        let mut joint = parse_leaf(reader, e, child_empty, read)?;
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
                        let geom = parse_leaf(reader, e, child_empty, parse_geom_attrs)?;
                        body.geoms.push(geom);
                    }
                    b"inertial" => {
                        body.inertial =
                            Some(parse_leaf(reader, e, child_empty, parse_inertial_attrs)?);
                    }
                    b"site" => {
                        let site = parse_leaf(reader, e, child_empty, parse_site_attrs)?;
                        body.sites.push(site);
                    }
                    b"frame" => body.frames.push(parse_frame(reader, e, child_empty)?),
                    b"composite" => {
                        let composite = parse_composite(reader, e, child_empty)?;
                        body.composites.push(composite);
                    }
                    b"plugin" => body.plugin = Some(parse_plugin_ref(reader, e, child_empty)?),
                    tag @ (b"replicate" | b"attach") => return Err(attach_refusal(tag)),
                    name => skip_element(reader, name, child_empty)?,
                }
            }
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
    if let Some(quat) = a.quat("quat")? {
        body.quat = quat;
    }
    a.one_orientation()?;
    body.euler = a.vec3("euler")?;
    body.axisangle = a.vec4("axisangle")?;
    body.xyaxes = a.array("xyaxes")?;
    body.zaxis = a.vec3("zaxis")?;
    body.childclass = a.string("childclass");
    if let Some(mocap) = a.boolean("mocap")? {
        body.mocap = mocap;
    }
    body.sleep = a.keyword_text("sleep", keywords::body_sleep)?;
    if let Some(gc) = a.real("gravcomp")? {
        body.gravcomp = Some(gc);
    }
    if let Some(user) = a.vector("user")? {
        body.user = user;
    }

    Ok(body)
}

/// Parse joint attributes only.
pub(super) fn parse_joint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let a = Attrs::new(e)?;
    let mut joint = MjcfJoint::default();

    joint.name = a.string("name").unwrap_or_default();
    joint.class = a.string("class");

    joint.joint_type = a.keyword("type", MjcfJointType::from_str)?;

    joint.pos = a.vec3("pos")?;
    joint.axis = a.vec3("axis")?;
    joint.limited = a.tf_auto("limited")?;
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
    joint.actuatorgravcomp = a.boolean("actuatorgravcomp")?;

    joint.margin = a.real("margin")?;
    if let Some(user) = a.vector("user")? {
        joint.user = user;
    }

    Ok(joint)
}

/// Parse freejoint attributes: a free joint MuJoCo 3.5.0 makes without
/// defaults (`mjs_addFreeJoint`), so no class reaches it; it reads `name` and
/// `group` (its `align` is not read).
pub(super) fn parse_freejoint_attrs(e: &BytesStart) -> Result<MjcfJoint> {
    let a = Attrs::new(e)?;
    let mut joint = MjcfJoint::default();
    joint.joint_type = Some(MjcfJointType::Free);
    joint.freejoint = true;
    joint.name = a.string("name").unwrap_or_default();
    joint.group = a.int("group")?;
    Ok(joint)
}

/// Parse geom attributes only.
pub(super) fn parse_geom_attrs(e: &BytesStart) -> Result<MjcfGeom> {
    let a = Attrs::new(e)?;
    let mut geom = MjcfGeom::default();

    geom.name = a.string("name");
    geom.class = a.string("class");

    geom.geom_type = a.keyword("type", MjcfGeomType::from_str)?;

    geom.pos = a.vec3("pos")?;
    geom.quat = a.quat("quat")?;
    a.one_orientation()?;
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
    geom.fluidshape = a.keyword("fluidshape", keywords::fluid_shape)?;
    geom.fluidcoef = a.prefix("fluidcoef")?;
    if let Some(user) = a.vector("user")? {
        geom.user = user;
    }

    // Parse shellinertia (boolean attribute)
    geom.shellinertia = a.boolean("shellinertia")?;

    Ok(geom)
}

/// Parse inertial attributes only.
pub(super) fn parse_inertial_attrs(e: &BytesStart) -> Result<MjcfInertial> {
    let a = Attrs::new(e)?;
    let mut inertial = MjcfInertial::default();

    // MuJoCo requires both (`xml_native_reader.cc`, measured: "required
    // attribute missing"); an empty value is a missing one.
    inertial.pos = a
        .vec3("pos")?
        .ok_or_else(|| MjcfError::missing_attribute("inertial", "pos"))?;
    if let Some(quat) = a.quat("quat")? {
        inertial.quat = quat;
    }
    a.one_orientation()?;
    inertial.mass = a
        .real("mass")?
        .ok_or_else(|| MjcfError::missing_attribute("inertial", "mass"))?;
    inertial.diaginertia = a.vec3("diaginertia")?;
    inertial.fullinertia = a.array("fullinertia")?;

    Ok(inertial)
}

/// Parse site attributes only.
pub(super) fn parse_site_attrs(e: &BytesStart) -> Result<MjcfSite> {
    let a = Attrs::new(e)?;
    let mut site = MjcfSite::default();

    site.name = a.string("name").unwrap_or_default();
    site.class = a.string("class");

    site.site_type = a.keyword_text("type", MjcfGeomType::from_str)?;
    site.pos = a.vec3("pos")?;
    site.quat = a.quat("quat")?;
    a.one_orientation()?;
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

/// Parse frame element (`empty`: it closed itself, so it has no children).
pub(super) fn parse_frame<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfFrame> {
    let mut frame = parse_frame_attrs(start)?;
    if empty {
        return Ok(frame);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"body" => frame.bodies.push(parse_body(reader, e, child_empty)?),
                    b"geom" => {
                        let geom = parse_leaf(reader, e, child_empty, parse_geom_attrs)?;
                        frame.geoms.push(geom);
                    }
                    b"site" => {
                        let site = parse_leaf(reader, e, child_empty, parse_site_attrs)?;
                        frame.sites.push(site);
                    }
                    b"frame" => frame.frames.push(parse_frame(reader, e, child_empty)?),
                    tag @ (b"camera" | b"light") => {
                        tracing::warn!(
                            "skipping <{}> inside <frame>: camera/light not yet supported",
                            String::from_utf8_lossy(tag)
                        );
                        skip_element(reader, tag, child_empty)?;
                    }
                    tag @ (b"joint" | b"freejoint" | b"inertial") => {
                        return Err(MjcfError::unsupported(format!(
                            "<{}> inside <frame>",
                            String::from_utf8_lossy(tag)
                        )));
                    }
                    tag @ (b"replicate" | b"attach") => return Err(attach_refusal(tag)),
                    name => skip_element(reader, name, child_empty)?,
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"frame" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("frame")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(frame)
}

/// The refusal of a `<replicate>` or an `<attach>`, which sim-mjcf does not
/// make (registry D-REPLICATE, D-ATTACH): MuJoCo 3.5.0 makes a `<replicate>`
/// and an `<attach>` of another model with `mjs_attach`, and an `<attach>`
/// whose `prefix` and `body` name a body the document already has moves that
/// body into the attach's frame (refused when the frame's parent is not the
/// body's).
fn attach_refusal(tag: &[u8]) -> MjcfError {
    MjcfError::unsupported(format!("<{}>", String::from_utf8_lossy(tag)))
}

/// An element whose children this parser does not read: its attributes, by
/// `read`, then past its end tag (nothing more when it closed itself).
fn parse_leaf<R: BufRead, T>(
    reader: &mut Reader<R>,
    e: &BytesStart,
    empty: bool,
    read: fn(&BytesStart) -> Result<T>,
) -> Result<T> {
    let value = read(e)?;
    skip_element(reader, e.name().as_ref(), empty)?;
    Ok(value)
}

/// Parse frame attributes only.
pub(super) fn parse_frame_attrs(e: &BytesStart) -> Result<MjcfFrame> {
    let a = Attrs::new(e)?;
    let mut frame = MjcfFrame::default();

    frame.name = a.string("name");
    frame.childclass = a.string("childclass");

    if let Some(pos) = a.vec3("pos")? {
        frame.pos = pos;
    }
    if let Some(quat) = a.quat("quat")? {
        frame.quat = quat;
    }
    a.one_orientation()?;
    frame.euler = a.vec3("euler")?;
    frame.axisangle = a.vec4("axisangle")?;
    frame.xyaxes = a.array("xyaxes")?;
    frame.zaxis = a.vec3("zaxis")?;

    Ok(frame)
}
