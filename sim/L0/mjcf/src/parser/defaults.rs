//! `<default>` class parsing for every element type.

use std::collections::HashSet;

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{
    MjcfActuator, MjcfActuatorType, MjcfDefault, MjcfEqualityDefaults, MjcfGeomDefaults,
    MjcfGeomType, MjcfJointDefaults, MjcfJointType, MjcfMeshDefaults, MjcfPairDefaults,
    MjcfSensorDefaults, MjcfSiteDefaults, MjcfTendonDefaults,
};

use super::attrs::{Attrs, parse_maxhullvert, skip_element};

/// Parse default element and its nested defaults (`empty`: it closed itself,
/// so it has none; the class is still defined, as MuJoCo 3.5.0 adds a class
/// before reading its children, `mjXReader::Default`).
pub(super) fn parse_default<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    parent_class: Option<String>,
    empty: bool,
) -> Result<Vec<MjcfDefault>> {
    let class = Attrs::new(start)?.string("class").unwrap_or_default();
    let mut default = MjcfDefault {
        class: class.clone(),
        parent_class,
        ..Default::default()
    };
    let mut nested_defaults = Vec::new();
    if empty {
        return Ok(vec![default]);
    }
    let mut buf = Vec::new();
    // MuJoCo 3.5.0's schema takes each element of a class once ("unique
    // element"), each actuator shortcut's tag apart.
    let mut seen = HashSet::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"default" {
                    // Nested default class; it reads its own children.
                    let nested = parse_default(reader, e, Some(class.clone()), child_empty)?;
                    nested_defaults.extend(nested);
                } else {
                    let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    if !seen.insert(tag.clone()) {
                        return Err(MjcfError::repeated_element(tag, "default"));
                    }
                    read_class_element(&mut default, e)?;
                    skip_element(reader, e.name().as_ref(), child_empty)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"default" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("default")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    let mut result = vec![default];
    result.extend(nested_defaults);
    Ok(result)
}

/// One element of a default class, other than a nested class, read onto
/// `default`; an element a class does not take is ignored.
fn read_class_element(default: &mut MjcfDefault, e: &BytesStart) -> Result<()> {
    match e.name().as_ref() {
        b"joint" => {
            default.joint = Some(parse_joint_defaults(e)?);
        }
        b"geom" => {
            default.geom = Some(parse_geom_defaults(e)?);
        }
        b"motor" | b"position" | b"velocity" | b"general" | b"cylinder" | b"muscle"
        | b"adhesion" | b"damper" | b"intvelocity" => {
            default.actuators.push(parse_actuator_defaults(e)?);
        }
        b"actuator" => {
            return Err(MjcfError::unsupported(
                "an <actuator> in a default class (MuJoCo 3.5.0 has none): write its kind, \
                 <general>, <motor> or another",
            ));
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
        b"equality" => {
            default.equality = Some(parse_equality_defaults(e)?);
        }
        _ => {}
    }
    Ok(())
}

/// Parse joint defaults.
pub(super) fn parse_joint_defaults(e: &BytesStart) -> Result<MjcfJointDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfJointDefaults::default();

    defaults.joint_type = a.keyword("type", MjcfJointType::from_str)?;
    defaults.pos = a.vec3("pos")?;
    defaults.limited = a.tf_auto("limited")?;
    defaults.axis = a.vec3("axis")?;
    defaults.ref_pos = a.real("ref")?;
    defaults.spring_ref = a.real("springref")?;
    defaults.damping = a.real("damping")?;
    defaults.stiffness = a.real("stiffness")?;
    defaults.armature = a.real("armature")?;
    defaults.frictionloss = a.real("frictionloss")?;
    defaults.group = a.int("group")?;
    if let Some(pair) = a.array::<2>("range")? {
        defaults.range = Some(pair.into());
    }
    defaults.solref_limit = a.prefix("solreflimit")?;
    defaults.solimp_limit = a.prefix("solimplimit")?;
    defaults.solreffriction = a.prefix("solreffriction")?;
    defaults.solimpfriction = a.prefix("solimpfriction")?;
    defaults.actuatorgravcomp = a.boolean("actuatorgravcomp")?;
    defaults.margin = a.real("margin")?;
    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse geom defaults.
pub(super) fn parse_geom_defaults(e: &BytesStart) -> Result<MjcfGeomDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfGeomDefaults::default();

    defaults.geom_type = a.keyword("type", MjcfGeomType::from_str)?;
    defaults.friction = a.prefix("friction")?;
    defaults.density = a.real("density")?;
    defaults.mass = a.real("mass")?;

    defaults.rgba = a.vec4("rgba")?;
    defaults.contype = a.int("contype")?;
    defaults.conaffinity = a.int("conaffinity")?;
    defaults.condim = a.int("condim")?;
    defaults.priority = a.int("priority")?;
    defaults.solmix = a.real("solmix")?;
    defaults.margin = a.real("margin")?;
    defaults.gap = a.real("gap")?;
    defaults.solref = a.prefix("solref")?;
    defaults.solimp = a.prefix("solimp")?;
    defaults.group = a.int("group")?;
    defaults.pos = a.vec3("pos")?;
    defaults.quat = a.quat("quat")?;
    a.one_orientation()?;
    defaults.euler = a.vec3("euler")?;
    defaults.axisangle = a.vec4("axisangle")?;
    defaults.xyaxes = a.array("xyaxes")?;
    defaults.zaxis = a.vec3("zaxis")?;

    // Exotic geom defaults
    defaults.fromto = a.array("fromto")?;
    defaults.mesh = a.string("mesh");
    defaults.hfield = a.string("hfield");

    // Rendering
    defaults.material = a.string("material");

    // Fluid force parameters
    defaults.fluidshape = a.keyword("fluidshape", keywords::fluid_shape)?;
    defaults.fluidcoef = a.prefix("fluidcoef")?;
    defaults.user = a.vector("user")?;
    defaults.shellinertia = a.boolean("shellinertia")?;

    Ok(defaults)
}

/// A default class's actuator element: its values, as an actuator's
/// ([`super::actuator::actuator_values`]), and no `tausmooth`.
pub(super) fn parse_actuator_defaults(e: &BytesStart) -> Result<MjcfActuator> {
    let a = Attrs::new(e)?;
    let kind = MjcfActuatorType::from_str(a.element())
        .ok_or_else(|| MjcfError::unsupported(format!("a default <{}>", a.element())))?;
    // MuJoCo 3.5.0's default schema gives no kind a `tausmooth`.
    if a.has("tausmooth") {
        return Err(MjcfError::unknown_attribute(a.element(), "tausmooth"));
    }
    super::actuator::actuator_values(&a, kind)
}

/// Parse tendon defaults.
pub(super) fn parse_tendon_defaults(e: &BytesStart) -> Result<MjcfTendonDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfTendonDefaults::default();

    if let Some(pair) = a.array::<2>("range")? {
        defaults.range = Some(pair.into());
    }
    defaults.limited = a.tf_auto("limited")?;
    defaults.stiffness = a.real("stiffness")?;
    defaults.damping = a.real("damping")?;
    defaults.frictionloss = a.real("frictionloss")?;

    // B3: Parse springlength in defaults
    if let Some(sl) = a.prefix::<2>("springlength")? {
        defaults.springlength = super::tendon::springlength(sl.given())?;
    }

    defaults.width = a.real("width")?;

    defaults.rgba = a.vec4("rgba")?;
    defaults.group = a.int("group")?;

    // Solver parameters
    defaults.solref_limit = a.prefix("solreflimit")?;
    defaults.solimp_limit = a.prefix("solimplimit")?;
    defaults.margin = a.real("margin")?;

    // Friction loss solver parameters
    defaults.solreffriction = a.prefix("solreffriction")?;
    defaults.solimpfriction = a.prefix("solimpfriction")?;

    // Rendering
    defaults.material = a.string("material");
    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse sensor defaults.
pub(super) fn parse_sensor_defaults(e: &BytesStart) -> Result<MjcfSensorDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfSensorDefaults::default();

    defaults.noise = a.real("noise")?;
    defaults.cutoff = a.real("cutoff")?;

    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse mesh defaults.
pub(super) fn parse_mesh_defaults(e: &BytesStart) -> Result<MjcfMeshDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfMeshDefaults::default();

    defaults.scale = a.vec3("scale")?;

    if let Some(n) = a.int("maxhullvert")? {
        defaults.maxhullvert = parse_maxhullvert(n)?;
    }

    defaults.inertia = a.keyword("inertia", keywords::mesh_inertia)?;

    Ok(defaults)
}

/// Parse site defaults.
pub(super) fn parse_site_defaults(e: &BytesStart) -> Result<MjcfSiteDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfSiteDefaults::default();

    defaults.site_type = a.keyword_text("type", MjcfGeomType::from_str)?;
    defaults.size = a.prefix("size")?;
    defaults.rgba = a.vec4("rgba")?;
    defaults.group = a.int("group")?;
    defaults.pos = a.vec3("pos")?;
    defaults.quat = a.quat("quat")?;
    a.one_orientation()?;
    defaults.euler = a.vec3("euler")?;
    defaults.axisangle = a.vec4("axisangle")?;
    defaults.xyaxes = a.array("xyaxes")?;
    defaults.zaxis = a.vec3("zaxis")?;
    defaults.fromto = a.array("fromto")?;

    // Rendering
    defaults.material = a.string("material");
    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse pair defaults from `<default><pair .../>`.
pub(super) fn parse_pair_defaults(e: &BytesStart) -> Result<MjcfPairDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfPairDefaults::default();

    defaults.condim = a.int("condim")?;
    defaults.margin = a.real("margin")?;
    defaults.gap = a.real("gap")?;

    defaults.friction = a.prefix("friction")?;
    defaults.solref = a.prefix("solref")?;
    defaults.solreffriction = a.prefix("solreffriction")?;
    defaults.solimp = a.prefix("solimp")?;

    Ok(defaults)
}

/// Parse `<default><equality>` attributes: active, solref, solimp.
/// MuJoCo ref: `OneEquality()` in `xml_native_reader.cc` (defaults context).
pub(super) fn parse_equality_defaults(e: &BytesStart) -> Result<MjcfEqualityDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfEqualityDefaults::default();

    defaults.active = a.boolean("active")?;
    defaults.solref = a.prefix("solref")?;
    defaults.solimp = a.prefix("solimp")?;

    Ok(defaults)
}
