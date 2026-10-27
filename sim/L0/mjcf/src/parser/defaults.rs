//! `<default>` class parsing for every element type.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    FluidShape, MeshInertia, MjcfActuatorDefaults, MjcfDefault, MjcfEqualityDefaults,
    MjcfGeomDefaults, MjcfGeomType, MjcfJointDefaults, MjcfJointType, MjcfMeshDefaults,
    MjcfPairDefaults, MjcfSensorDefaults, MjcfSiteDefaults, MjcfTendonDefaults,
};

use super::attrs::{Attrs, parse_maxhullvert, skip_element};

/// Parse default element and its nested defaults.
pub(super) fn parse_default<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    parent_class: Option<String>,
) -> Result<Vec<MjcfDefault>> {
    let class = Attrs::new(start)?.string("class").unwrap_or_default();
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
                    b"actuator" | b"motor" | b"position" | b"velocity" | b"general"
                    | b"cylinder" | b"muscle" | b"adhesion" | b"damper" | b"intvelocity" => {
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
                    b"equality" => {
                        default.equality = Some(parse_equality_defaults(e)?);
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
                b"actuator" | b"motor" | b"position" | b"velocity" | b"general" | b"cylinder"
                | b"muscle" | b"adhesion" | b"damper" | b"intvelocity" => {
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
                b"equality" => {
                    default.equality = Some(parse_equality_defaults(e)?);
                }
                _ => {}
            },
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

/// Parse joint defaults.
pub(super) fn parse_joint_defaults(e: &BytesStart) -> Result<MjcfJointDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfJointDefaults::default();

    if let Some(jtype) = a.text("type") {
        defaults.joint_type = MjcfJointType::from_str(jtype);
    }
    defaults.pos = a.vec3("pos")?;
    defaults.limited = a.is_true("limited");
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
    defaults.actuatorgravcomp = a.is_true("actuatorgravcomp");
    defaults.margin = a.real("margin")?;
    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse geom defaults.
pub(super) fn parse_geom_defaults(e: &BytesStart) -> Result<MjcfGeomDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfGeomDefaults::default();

    if let Some(gtype) = a.text("type") {
        defaults.geom_type = MjcfGeomType::from_str(gtype);
    }
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
    defaults.quat = a.vec4("quat")?;
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
    if let Some(fs) = a.text("fluidshape") {
        defaults.fluidshape = Some(match fs {
            "none" => FluidShape::None,
            "ellipsoid" => FluidShape::Ellipsoid,
            _ => return Err(MjcfError::invalid_keyword("geom", "fluidshape", fs)),
        });
    }
    defaults.fluidcoef = a.prefix("fluidcoef")?;
    defaults.user = a.vector("user")?;
    defaults.shellinertia = a.is_true("shellinertia");

    Ok(defaults)
}

/// Parse actuator defaults.
pub(super) fn parse_actuator_defaults(e: &BytesStart) -> Result<MjcfActuatorDefaults> {
    let a = Attrs::new(e)?;
    super::actuator::refuse_actdim(&a)?;
    let mut defaults = MjcfActuatorDefaults::default();

    if let Some(pair) = a.array::<2>("ctrlrange")? {
        defaults.ctrlrange = Some(pair.into());
    }
    if let Some(pair) = a.array::<2>("forcerange")? {
        defaults.forcerange = Some(pair.into());
    }
    defaults.gear = a.prefix("gear")?;
    defaults.kp = a.real("kp")?;
    defaults.kv = a.real("kv")?;
    defaults.dampratio = a.real("dampratio")?;

    defaults.ctrllimited = a.is_true("ctrllimited");
    defaults.forcelimited = a.is_true("forcelimited");

    // <general>-specific attributes — parsed unconditionally because
    // parse_actuator_defaults() is called for all actuator element names
    // (actuator|motor|position|velocity|general). The model builder
    // ignores these for shortcut types.
    defaults.gaintype = a.string("gaintype");
    defaults.biastype = a.string("biastype");
    defaults.dyntype = a.string("dyntype");
    defaults.gainprm = super::actuator::gain_or_bias(&a, "gainprm")?;
    defaults.biasprm = super::actuator::gain_or_bias(&a, "biasprm")?;
    defaults.dynprm = a.prefix("dynprm")?;

    // Activation parameters
    defaults.group = a.int("group")?;
    defaults.actlimited = a.is_true("actlimited");
    if let Some(pair) = a.array::<2>("actrange")? {
        defaults.actrange = Some(pair.into());
    }
    defaults.actearly = a.is_true("actearly");
    if let Some(pair) = a.array::<2>("lengthrange")? {
        defaults.lengthrange = Some(pair.into());
    }

    // Interpolation attributes (common to all actuator types)
    defaults.nsample = a.int("nsample")?;
    defaults.interp = a.string("interp");
    defaults.delay = a.real("delay")?;

    // Cylinder-specific attributes
    defaults.area = a.real("area")?;
    defaults.diameter = a.real("diameter")?;
    // A muscle's `timeconst` is a pair [activation, deactivation]; every other
    // type's is one number (`xml_native_reader.cc` reads 2 and 1, exactly).
    if a.element() == "muscle" {
        if let Some(pair) = a.array::<2>("timeconst")? {
            defaults.muscle_timeconst = Some(pair.into());
        }
    } else {
        defaults.timeconst = a.real("timeconst")?;
    }
    defaults.bias = a.array("bias")?;

    // Muscle-specific attributes
    if let Some(pair) = a.array::<2>("range")? {
        defaults.range = Some(pair.into());
    }
    defaults.force = a.real("force")?;
    defaults.scale = a.real("scale")?;
    defaults.lmin = a.real("lmin")?;
    defaults.lmax = a.real("lmax")?;
    defaults.vmax = a.real("vmax")?;
    defaults.fpmax = a.real("fpmax")?;
    defaults.fvmax = a.real("fvmax")?;

    // Adhesion-specific attributes
    defaults.gain = a.real("gain")?;
    defaults.user = a.vector("user")?;

    Ok(defaults)
}

/// Parse tendon defaults.
pub(super) fn parse_tendon_defaults(e: &BytesStart) -> Result<MjcfTendonDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfTendonDefaults::default();

    if let Some(pair) = a.array::<2>("range")? {
        defaults.range = Some(pair.into());
    }
    defaults.limited = a.is_true("limited");
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

    if let Some(inertia_str) = a.text("inertia") {
        defaults.inertia = Some(match inertia_str {
            "convex" => MeshInertia::Convex,
            "exact" => MeshInertia::Exact,
            "legacy" => MeshInertia::Legacy,
            "shell" => MeshInertia::Shell,
            other => {
                return Err(MjcfError::invalid_keyword("mesh", "inertia", other));
            }
        });
    }

    Ok(defaults)
}

/// Parse site defaults.
pub(super) fn parse_site_defaults(e: &BytesStart) -> Result<MjcfSiteDefaults> {
    let a = Attrs::new(e)?;
    let mut defaults = MjcfSiteDefaults::default();

    defaults.site_type = a.string("type");
    defaults.size = a.prefix("size")?;
    defaults.rgba = a.vec4("rgba")?;
    defaults.group = a.int("group")?;
    defaults.pos = a.vec3("pos")?;
    defaults.quat = a.vec4("quat")?;
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

    defaults.active = a.is_true("active");
    defaults.solref = a.prefix("solref")?;
    defaults.solimp = a.prefix("solimp")?;

    Ok(defaults)
}
