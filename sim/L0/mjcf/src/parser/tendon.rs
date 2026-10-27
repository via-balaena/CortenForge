//! `<tendon>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfTendon, MjcfTendonType, SpatialPathElement};

use super::attrs::{Attrs, skip_element};

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
            Ok(Event::Eof) => return Err(MjcfError::eof_in("tendon")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
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
                let a = Attrs::new(e)?;
                match name_bytes.as_slice() {
                    b"site" => {
                        // Site reference for spatial tendon path
                        if let Some(site_name) = a.string("site") {
                            tendon
                                .path_elements
                                .push(SpatialPathElement::Site { site: site_name });
                        }
                    }
                    b"joint" => {
                        if tendon.tendon_type == MjcfTendonType::Spatial {
                            return Err(MjcfError::invalid_model(
                                "a <spatial> tendon's path takes sites, geoms and pulleys, \
                                 not a <joint>; a <fixed> tendon couples joints",
                            ));
                        }
                        // Joint reference for fixed tendon
                        if let Some(joint_name) = a.string("joint") {
                            let coef = a.real("coef")?.unwrap_or(1.0);
                            tendon.joints.push((joint_name, coef));
                        }
                    }
                    b"geom" => {
                        // Wrapping geom reference for spatial tendon path
                        if let Some(geom_name) = a.string("geom") {
                            let sidesite = a.string("sidesite");
                            tendon.path_elements.push(SpatialPathElement::Geom {
                                geom: geom_name,
                                sidesite,
                            });
                        }
                    }
                    b"pulley" => {
                        // Pulley element for spatial tendon path
                        let divisor = a.real("divisor")?.unwrap_or(1.0);
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
                return Err(MjcfError::eof_in("tendon element"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
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
    let a = Attrs::new(e)?;
    let mut tendon = MjcfTendon {
        tendon_type,
        ..Default::default()
    };

    tendon.name = a.string("name").unwrap_or_default();
    tendon.class = a.string("class");

    if let Some(pair) = a.array::<2>("range")? {
        tendon.range = Some(pair.into());
    }

    tendon.limited = a.is_true("limited");

    tendon.stiffness = a.real("stiffness")?;
    tendon.damping = a.real("damping")?;
    tendon.frictionloss = a.real("frictionloss")?;

    // B2: Parse springlength (1 or 2 values)
    if let Some(sl) = a.prefix::<2>("springlength")? {
        tendon.springlength = springlength(sl.given())?; // S2
    }

    tendon.width = a.real("width")?;
    tendon.group = a.int("group")?;
    tendon.rgba = a.vec4("rgba")?;

    // Solver parameters
    tendon.solref_limit = a.prefix("solreflimit")?;
    tendon.solimp_limit = a.prefix("solimplimit")?;
    tendon.margin = a.real("margin")?;

    // Friction loss solver parameters
    tendon.solreffriction = a.prefix("solreffriction")?;
    tendon.solimpfriction = a.prefix("solimpfriction")?;

    // Rendering
    tendon.material = a.string("material");
    if let Some(user) = a.vector("user")? {
        tendon.user = user;
    }

    Ok(tendon)
}

/// A tendon's `springlength` from its values: one is both ends (S2), two are
/// low and high (S4), none is unset. Each must be finite and non-negative.
pub(super) fn springlength(parts: &[f64]) -> Result<Option<(f64, f64)>> {
    let (low, high) = match parts {
        [] => return Ok(None),
        [v] => (*v, *v),
        [low, high, ..] => (*low, *high),
    };
    for v in [low, high] {
        if !v.is_finite() {
            return Err(MjcfError::non_finite("tendon", "springlength", v));
        }
        if v < 0.0 {
            return Err(MjcfError::invalid_value(
                "tendon",
                "springlength",
                format!("value ({v}) must be >= 0"),
            ));
        }
    }
    if low > high {
        return Err(MjcfError::invalid_value(
            "tendon",
            "springlength",
            format!("low ({low}) must be <= high ({high})"),
        ));
    }
    Ok(Some((low, high)))
}
