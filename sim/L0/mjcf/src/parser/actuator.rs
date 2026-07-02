//! `<actuator>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfActuator, MjcfActuatorType};

use super::attrs::{
    attr_bool, get_attribute_opt, parse_float_array, parse_float_array_opt, parse_float_attr,
    parse_int_attr, skip_element,
};
use super::extension::{parse_plugin_ref, parse_plugin_ref_empty};

/// Parse actuators element.
pub(super) fn parse_actuators<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfActuator>> {
    let mut actuators = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let mut actuator = parse_actuator_attrs(e, actuator_type)?;
                    // Parse children (may contain <plugin> sub-element)
                    parse_actuator_children(reader, elem_name.as_bytes(), &mut actuator)?;
                    actuators.push(actuator);
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
pub(super) fn parse_actuator_attrs(
    e: &BytesStart,
    actuator_type: MjcfActuatorType,
) -> Result<MjcfActuator> {
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
    actuator.jointinparent = get_attribute_opt(e, "jointinparent");
    actuator.cranksite = get_attribute_opt(e, "cranksite");
    actuator.slidersite = get_attribute_opt(e, "slidersite");
    actuator.cranklength = parse_float_attr(e, "cranklength");

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

    actuator.ctrllimited = attr_bool(e, "ctrllimited");
    actuator.forcelimited = attr_bool(e, "forcelimited");

    if let Some(kp) = parse_float_attr(e, "kp") {
        actuator.kp = kp;
    }
    if let Some(kv) = parse_float_attr(e, "kv") {
        actuator.kv = Some(kv);
    }
    if let Some(dampratio) = parse_float_attr(e, "dampratio") {
        actuator.dampratio = Some(dampratio);
    }

    // Interpolation attributes (common to all actuator types)
    actuator.nsample = parse_int_attr(e, "nsample");
    actuator.interp = get_attribute_opt(e, "interp");
    actuator.delay = parse_float_attr(e, "delay");

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

    // ========================================================================
    // Activation parameters
    // ========================================================================
    actuator.group = parse_int_attr(e, "group");
    actuator.actlimited = attr_bool(e, "actlimited");
    if let Some(actrange) = get_attribute_opt(e, "actrange") {
        let parts = parse_float_array(&actrange)?;
        if parts.len() >= 2 {
            actuator.actrange = Some((parts[0], parts[1]));
        }
    }
    actuator.actearly = attr_bool(e, "actearly");
    if let Some(lengthrange) = get_attribute_opt(e, "lengthrange") {
        let parts = parse_float_array(&lengthrange)?;
        if parts.len() >= 2 {
            actuator.lengthrange = Some((parts[0], parts[1]));
        }
    }
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            actuator.user = parts;
        }
    }

    Ok(actuator)
}

/// Parse children of an actuator element (may contain `<plugin>` sub-element).
pub(super) fn parse_actuator_children<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
    actuator: &mut MjcfActuator,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref(reader, e)?;
                actuator.plugin = Some(plugin_ref);
            }
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref_empty(e)?;
                actuator.plugin = Some(plugin_ref);
            }
            Ok(Event::Start(ref e)) => {
                // Unknown child — skip
                let name = e.name().as_ref().to_vec();
                skip_element(reader, &name)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == parent_tag => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(format!(
                    "unexpected EOF in {}",
                    String::from_utf8_lossy(parent_tag)
                )));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(())
}
