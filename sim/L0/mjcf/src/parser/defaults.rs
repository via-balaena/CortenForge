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

use super::attrs::{
    attr_bool, attr_fixed, attr_vec3, attr_vec4, get_attribute_opt, parse_float_array,
    parse_float_array_opt, parse_float_attr, parse_int_attr, parse_maxhullvert, skip_element,
};

/// Parse default element and its nested defaults.
pub(super) fn parse_default<R: BufRead>(
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
pub(super) fn parse_joint_defaults(e: &BytesStart) -> Result<MjcfJointDefaults> {
    let mut defaults = MjcfJointDefaults::default();

    if let Some(jtype) = get_attribute_opt(e, "type") {
        defaults.joint_type = MjcfJointType::from_str(&jtype);
    }
    defaults.pos = attr_vec3(e, "pos")?;
    defaults.limited = attr_bool(e, "limited");
    defaults.axis = attr_vec3(e, "axis")?;
    defaults.ref_pos = parse_float_attr(e, "ref");
    defaults.spring_ref = parse_float_attr(e, "springref");
    defaults.damping = parse_float_attr(e, "damping");
    defaults.stiffness = parse_float_attr(e, "stiffness");
    defaults.armature = parse_float_attr(e, "armature");
    defaults.frictionloss = parse_float_attr(e, "frictionloss");
    defaults.group = parse_int_attr(e, "group");
    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            defaults.range = Some((parts[0], parts[1]));
        }
    }
    defaults.solref_limit = attr_fixed::<2>(e, "solreflimit")?;
    defaults.solimp_limit = attr_fixed::<5>(e, "solimplimit")?;
    defaults.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    defaults.solimpfriction = attr_fixed::<5>(e, "solimpfriction")?;
    defaults.actuatorgravcomp = attr_bool(e, "actuatorgravcomp");
    defaults.margin = parse_float_attr(e, "margin");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            if !parts.is_empty() {
                defaults.user = Some(parts);
            }
        }
    }

    Ok(defaults)
}

/// Parse geom defaults.
pub(super) fn parse_geom_defaults(e: &BytesStart) -> Result<MjcfGeomDefaults> {
    let mut defaults = MjcfGeomDefaults::default();

    if let Some(gtype) = get_attribute_opt(e, "type") {
        defaults.geom_type = MjcfGeomType::from_str(&gtype);
    }
    defaults.friction = attr_vec3(e, "friction")?;
    defaults.density = parse_float_attr(e, "density");
    defaults.mass = parse_float_attr(e, "mass");

    defaults.rgba = attr_vec4(e, "rgba")?;
    defaults.contype = parse_int_attr(e, "contype");
    defaults.conaffinity = parse_int_attr(e, "conaffinity");
    defaults.condim = parse_int_attr(e, "condim");
    defaults.priority = parse_int_attr(e, "priority");
    defaults.solmix = parse_float_attr(e, "solmix");
    defaults.margin = parse_float_attr(e, "margin");
    defaults.gap = parse_float_attr(e, "gap");
    defaults.solref = attr_fixed::<2>(e, "solref")?;
    defaults.solimp = attr_fixed::<5>(e, "solimp")?;
    defaults.group = parse_int_attr(e, "group");
    defaults.pos = attr_vec3(e, "pos")?;
    defaults.quat = attr_vec4(e, "quat")?;
    defaults.euler = attr_vec3(e, "euler")?;
    defaults.axisangle = attr_vec4(e, "axisangle")?;
    defaults.xyaxes = attr_fixed::<6>(e, "xyaxes")?;
    defaults.zaxis = attr_vec3(e, "zaxis")?;

    // Exotic geom defaults
    defaults.fromto = attr_fixed::<6>(e, "fromto")?;
    defaults.mesh = get_attribute_opt(e, "mesh");
    defaults.hfield = get_attribute_opt(e, "hfield");

    // Rendering
    defaults.material = get_attribute_opt(e, "material");

    // Fluid force parameters
    if let Some(fs) = get_attribute_opt(e, "fluidshape") {
        defaults.fluidshape = Some(match fs.as_str() {
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
        defaults.fluidcoef = Some([parts[0], parts[1], parts[2], parts[3], parts[4]]);
    }
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            if !parts.is_empty() {
                defaults.user = Some(parts);
            }
        }
    }
    defaults.shellinertia = attr_bool(e, "shellinertia");

    Ok(defaults)
}

/// Parse actuator defaults.
pub(super) fn parse_actuator_defaults(e: &BytesStart) -> Result<MjcfActuatorDefaults> {
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
    defaults.dampratio = parse_float_attr(e, "dampratio");

    defaults.ctrllimited = attr_bool(e, "ctrllimited");
    defaults.forcelimited = attr_bool(e, "forcelimited");

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

    // Activation parameters
    defaults.group = parse_int_attr(e, "group");
    defaults.actlimited = attr_bool(e, "actlimited");
    if let Some(actrange) = get_attribute_opt(e, "actrange") {
        let parts = parse_float_array(&actrange)?;
        if parts.len() >= 2 {
            defaults.actrange = Some((parts[0], parts[1]));
        }
    }
    defaults.actearly = attr_bool(e, "actearly");
    if let Some(lengthrange) = get_attribute_opt(e, "lengthrange") {
        let parts = parse_float_array(&lengthrange)?;
        if parts.len() >= 2 {
            defaults.lengthrange = Some((parts[0], parts[1]));
        }
    }

    // Interpolation attributes (common to all actuator types)
    defaults.nsample = parse_int_attr(e, "nsample");
    defaults.interp = get_attribute_opt(e, "interp");
    defaults.delay = parse_float_attr(e, "delay");

    // Cylinder-specific attributes
    defaults.area = parse_float_attr(e, "area");
    defaults.diameter = parse_float_attr(e, "diameter");
    // `timeconst` is ambiguous: 1 value → cylinder timeconst, 2 values → muscle timeconst.
    // MuJoCo: <cylinder timeconst="real(1)">, <muscle timeconst="real(2)">.
    if let Some(tc_str) = get_attribute_opt(e, "timeconst") {
        let parts = parse_float_array(&tc_str)?;
        if parts.len() == 1 {
            defaults.timeconst = Some(parts[0]);
        } else if parts.len() >= 2 {
            defaults.muscle_timeconst = Some((parts[0], parts[1]));
        }
    }
    defaults.bias = attr_fixed::<3>(e, "bias")?;

    // Muscle-specific attributes
    if let Some(range_str) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range_str)?;
        if parts.len() >= 2 {
            defaults.range = Some((parts[0], parts[1]));
        }
    }
    defaults.force = parse_float_attr(e, "force");
    defaults.scale = parse_float_attr(e, "scale");
    defaults.lmin = parse_float_attr(e, "lmin");
    defaults.lmax = parse_float_attr(e, "lmax");
    defaults.vmax = parse_float_attr(e, "vmax");
    defaults.fpmax = parse_float_attr(e, "fpmax");
    defaults.fvmax = parse_float_attr(e, "fvmax");

    // Adhesion-specific attributes
    defaults.gain = parse_float_attr(e, "gain");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            if !parts.is_empty() {
                defaults.user = Some(parts);
            }
        }
    }

    Ok(defaults)
}

/// Parse tendon defaults.
pub(super) fn parse_tendon_defaults(e: &BytesStart) -> Result<MjcfTendonDefaults> {
    let mut defaults = MjcfTendonDefaults::default();

    if let Some(range) = get_attribute_opt(e, "range") {
        let parts = parse_float_array(&range)?;
        if parts.len() >= 2 {
            defaults.range = Some((parts[0], parts[1]));
        }
    }
    defaults.limited = attr_bool(e, "limited");
    defaults.stiffness = parse_float_attr(e, "stiffness");
    defaults.damping = parse_float_attr(e, "damping");
    defaults.frictionloss = parse_float_attr(e, "frictionloss");

    // B3: Parse springlength in defaults
    if let Some(sl_str) = get_attribute_opt(e, "springlength") {
        let parts = parse_float_array(&sl_str)?;
        match parts.len() {
            1 => {
                if parts[0] < 0.0 || !parts[0].is_finite() {
                    return Err(MjcfError::XmlParse(format!(
                        "tendon default springlength: value ({}) must be finite and >= 0",
                        parts[0]
                    )));
                }
                defaults.springlength = Some((parts[0], parts[0]));
            }
            n if n >= 2 => {
                if parts[0] < 0.0
                    || parts[1] < 0.0
                    || !parts[0].is_finite()
                    || !parts[1].is_finite()
                {
                    return Err(MjcfError::XmlParse(format!(
                        "tendon default springlength: values ({}, {}) must be finite and >= 0",
                        parts[0], parts[1]
                    )));
                }
                if parts[0] > parts[1] {
                    return Err(MjcfError::XmlParse(format!(
                        "tendon default springlength: low ({}) must be <= high ({})",
                        parts[0], parts[1]
                    )));
                }
                defaults.springlength = Some((parts[0], parts[1]));
            }
            _ => {}
        }
    }

    defaults.width = parse_float_attr(e, "width");

    defaults.rgba = attr_vec4(e, "rgba")?;
    defaults.group = parse_int_attr(e, "group");

    // Solver parameters
    defaults.solref_limit = attr_fixed::<2>(e, "solreflimit")?;
    defaults.solimp_limit = attr_fixed::<5>(e, "solimplimit")?;
    defaults.margin = parse_float_attr(e, "margin");

    // Friction loss solver parameters
    defaults.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    defaults.solimpfriction = attr_fixed::<5>(e, "solimpfriction")?;

    // Rendering
    defaults.material = get_attribute_opt(e, "material");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            if !parts.is_empty() {
                defaults.user = Some(parts);
            }
        }
    }

    Ok(defaults)
}

/// Parse sensor defaults.
pub(super) fn parse_sensor_defaults(e: &BytesStart) -> Result<MjcfSensorDefaults> {
    let mut defaults = MjcfSensorDefaults::default();

    defaults.noise = parse_float_attr(e, "noise");
    defaults.cutoff = parse_float_attr(e, "cutoff");

    if let Some(user) = get_attribute_opt(e, "user") {
        defaults.user = Some(parse_float_array(&user)?);
    }

    Ok(defaults)
}

/// Parse mesh defaults.
pub(super) fn parse_mesh_defaults(e: &BytesStart) -> Result<MjcfMeshDefaults> {
    let mut defaults = MjcfMeshDefaults::default();

    defaults.scale = attr_vec3(e, "scale")?;

    if let Some(n) = parse_int_attr(e, "maxhullvert") {
        defaults.maxhullvert = parse_maxhullvert(n)?;
    }

    if let Some(inertia_str) = get_attribute_opt(e, "inertia") {
        defaults.inertia = Some(match inertia_str.as_str() {
            "convex" => MeshInertia::Convex,
            "exact" => MeshInertia::Exact,
            "legacy" => MeshInertia::Legacy,
            "shell" => MeshInertia::Shell,
            other => {
                return Err(MjcfError::XmlParse(format!(
                    "mesh default: invalid inertia mode '{other}'"
                )));
            }
        });
    }

    Ok(defaults)
}

/// Parse site defaults.
pub(super) fn parse_site_defaults(e: &BytesStart) -> Result<MjcfSiteDefaults> {
    let mut defaults = MjcfSiteDefaults::default();

    defaults.site_type = get_attribute_opt(e, "type");
    if let Some(size) = get_attribute_opt(e, "size") {
        defaults.size = Some(parse_float_array(&size)?);
    }
    defaults.rgba = attr_vec4(e, "rgba")?;
    defaults.group = parse_int_attr(e, "group");
    defaults.pos = attr_vec3(e, "pos")?;
    defaults.quat = attr_vec4(e, "quat")?;
    defaults.euler = attr_vec3(e, "euler")?;
    defaults.axisangle = attr_vec4(e, "axisangle")?;
    defaults.xyaxes = attr_fixed::<6>(e, "xyaxes")?;
    defaults.zaxis = attr_vec3(e, "zaxis")?;

    // Rendering
    defaults.material = get_attribute_opt(e, "material");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            if !parts.is_empty() {
                defaults.user = Some(parts);
            }
        }
    }

    Ok(defaults)
}

/// Parse pair defaults from `<default><pair .../>`.
pub(super) fn parse_pair_defaults(e: &BytesStart) -> Result<MjcfPairDefaults> {
    let mut defaults = MjcfPairDefaults::default();

    defaults.condim = parse_int_attr(e, "condim");
    defaults.margin = parse_float_attr(e, "margin");
    defaults.gap = parse_float_attr(e, "gap");

    defaults.friction = attr_fixed::<5>(e, "friction")?;
    defaults.solref = attr_fixed::<2>(e, "solref")?;
    defaults.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    defaults.solimp = attr_fixed::<5>(e, "solimp")?;

    Ok(defaults)
}

/// Parse `<default><equality>` attributes: active, solref, solimp.
/// MuJoCo ref: `OneEquality()` in `xml_native_reader.cc` (defaults context).
pub(super) fn parse_equality_defaults(e: &BytesStart) -> Result<MjcfEqualityDefaults> {
    let mut defaults = MjcfEqualityDefaults::default();

    defaults.active = attr_bool(e, "active");
    defaults.solref = attr_fixed::<2>(e, "solref")?;
    defaults.solimp = attr_fixed::<5>(e, "solimp")?;

    Ok(defaults)
}
