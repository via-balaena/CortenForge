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

/// Parse the tendon section (`empty`: it closed itself, so it has none).
pub(super) fn parse_tendons<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<Vec<MjcfTendon>> {
    let mut tendons = Vec::new();
    if empty {
        return Ok(tendons);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(tendon_type) = MjcfTendonType::from_str(&elem_name) {
                    tendons.push(parse_tendon(reader, e, tendon_type, child_empty)?);
                } else {
                    // Unknown tendon type, skip it
                    skip_element(reader, e.name().as_ref(), child_empty)?;
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

/// Parse a tendon element and its children (sites for spatial, joints for fixed;
/// `empty`: it closed itself, so it has none).
pub(super) fn parse_tendon<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    tendon_type: MjcfTendonType,
    empty: bool,
) -> Result<MjcfTendon> {
    let mut tendon = parse_tendon_attrs(start, tendon_type)?;
    if empty {
        return Ok(tendon);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                let name_bytes = e.name().as_ref().to_vec();
                let a = Attrs::new(e)?;
                match name_bytes.as_slice() {
                    b"site" => {
                        // Site reference for spatial tendon path
                        let site = a
                            .string("site")
                            .ok_or_else(|| MjcfError::missing_attribute("site", "site"))?;
                        tendon.path_elements.push(SpatialPathElement::Site { site });
                    }
                    b"joint" => {
                        if tendon.tendon_type == MjcfTendonType::Spatial {
                            return Err(MjcfError::invalid_model(
                                "a <spatial> tendon's path takes sites, geoms and pulleys, \
                                 not a <joint>; a <fixed> tendon couples joints",
                            ));
                        }
                        // Joint reference for fixed tendon
                        let joint = a
                            .string("joint")
                            .ok_or_else(|| MjcfError::missing_attribute("joint", "joint"))?;
                        let coef = a
                            .real("coef")?
                            .ok_or_else(|| MjcfError::missing_attribute("joint", "coef"))?;
                        tendon.joints.push((joint, coef));
                    }
                    b"geom" => {
                        // Wrapping geom reference for spatial tendon path
                        let geom = a
                            .string("geom")
                            .ok_or_else(|| MjcfError::missing_attribute("geom", "geom"))?;
                        let sidesite = a.string("sidesite");
                        tendon
                            .path_elements
                            .push(SpatialPathElement::Geom { geom, sidesite });
                    }
                    b"pulley" => {
                        // Pulley element for spatial tendon path
                        let divisor = a
                            .real("divisor")?
                            .ok_or_else(|| MjcfError::missing_attribute("pulley", "divisor"))?;
                        tendon
                            .path_elements
                            .push(SpatialPathElement::Pulley { divisor });
                    }
                    _ => {}
                }
                skip_element(reader, &name_bytes, child_empty)?;
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

    tendon.limited = a.tf_auto("limited")?;

    tendon.stiffness = a.real("stiffness")?;
    tendon.damping = a.real("damping")?;
    tendon.armature = a.real("armature")?;
    tendon.frictionloss = a.real("frictionloss")?;

    // B2: Parse springlength (1 or 2 values)
    if let Some(sl) = a.prefix::<2>("springlength")? {
        tendon.springlength = springlength(sl.given());
        // The element's own, which no class replaces, refused where this
        // crate refused it before book L21; a class's once applied
        // (`validation.rs`).
        if let Some((low, high)) = tendon.springlength
            && low > high
        {
            return Err(MjcfError::invalid_value(
                "tendon",
                "springlength",
                "invalid springlength in tendon",
            ));
        }
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

/// A tendon's `springlength` from its values: one is both ends, two are low
/// and high, none is unset, as MuJoCo 3.5.0 reads it
/// (`xml_native_reader.cc:2269-2272`). A negative value is a length, as
/// MuJoCo takes it, and `-1 -1` asks for the tendon's length at
/// `qpos_spring`, as MuJoCo makes it (`engine_setconst.c:1069-1084`). A low
/// above its high is refused by the caller, and a class's once applied
/// (`validation.rs`).
pub(super) fn springlength(parts: &[f64]) -> Option<(f64, f64)> {
    match parts {
        [] => None,
        [v] => Some((*v, *v)),
        [low, high, ..] => Some((*low, *high)),
    }
}
