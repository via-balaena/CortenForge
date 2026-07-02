//! `<tendon>` parsing.

use nalgebra::Vector4;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfTendon, MjcfTendonType, SpatialPathElement};

use super::attrs::{
    attr_bool, attr_fixed, get_attribute_opt, parse_float_array, parse_float_attr, parse_int_attr,
    skip_element,
};

// ============================================================================
// Tendon parsing
// ============================================================================

/// Parse the tendon section.
pub(super) fn parse_tendons<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfTendon>> {
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
                    let tendon = parse_tendon_attrs(e, tendon_type)?;
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
pub(super) fn parse_tendon<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    tendon_type: MjcfTendonType,
) -> Result<MjcfTendon> {
    let mut tendon = parse_tendon_attrs(start, tendon_type)?;
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
pub(super) fn parse_tendon_attrs(
    e: &BytesStart,
    tendon_type: MjcfTendonType,
) -> Result<MjcfTendon> {
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

    tendon.limited = attr_bool(e, "limited");

    tendon.stiffness = parse_float_attr(e, "stiffness");
    tendon.damping = parse_float_attr(e, "damping");
    tendon.frictionloss = parse_float_attr(e, "frictionloss");

    // B2: Parse springlength (1 or 2 values)
    if let Some(sl_str) = get_attribute_opt(e, "springlength") {
        if let Ok(parts) = parse_float_array(&sl_str) {
            match parts.len() {
                1 => {
                    if parts[0] < 0.0 || !parts[0].is_finite() {
                        return Err(MjcfError::invalid_attribute(
                            "springlength",
                            tendon.name,
                            format!("value ({}) must be finite and >= 0", parts[0]),
                        ));
                    }
                    tendon.springlength = Some((parts[0], parts[0])); // S2
                }
                n if n >= 2 => {
                    if parts[0] < 0.0
                        || parts[1] < 0.0
                        || !parts[0].is_finite()
                        || !parts[1].is_finite()
                    {
                        return Err(MjcfError::invalid_attribute(
                            "springlength",
                            tendon.name,
                            format!(
                                "values ({}, {}) must be finite and >= 0",
                                parts[0], parts[1]
                            ),
                        ));
                    }
                    if parts[0] > parts[1] {
                        // S4
                        return Err(MjcfError::invalid_attribute(
                            "springlength",
                            tendon.name,
                            format!("low ({}) must be <= high ({})", parts[0], parts[1]),
                        ));
                    }
                    tendon.springlength = Some((parts[0], parts[1]));
                }
                _ => {} // empty string, ignore
            }
        }
    }

    tendon.width = parse_float_attr(e, "width");
    tendon.group = parse_int_attr(e, "group");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        if let Ok(parts) = parse_float_array(&rgba) {
            if parts.len() >= 4 {
                tendon.rgba = Some(Vector4::new(parts[0], parts[1], parts[2], parts[3]));
            }
        }
    }

    // Solver parameters
    tendon.solref_limit = attr_fixed::<2>(e, "solreflimit")?;
    tendon.solimp_limit = attr_fixed::<5>(e, "solimplimit")?;
    tendon.margin = parse_float_attr(e, "margin");

    // Friction loss solver parameters
    tendon.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    tendon.solimpfriction = attr_fixed::<5>(e, "solimpfriction")?;

    // Rendering
    tendon.material = get_attribute_opt(e, "material");
    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            tendon.user = parts;
        }
    }

    Ok(tendon)
}
