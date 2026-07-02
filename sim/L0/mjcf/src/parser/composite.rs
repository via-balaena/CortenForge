//! `<composite>` body parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    CompositeGeom, CompositeJoint, CompositeShape, CompositeType, MjcfComposite, MjcfGeomType,
};

use super::attrs::{
    get_attribute_opt, parse_float_array, parse_float_attr, parse_int_attr, skip_element,
};

// ============================================================================
// Composite parsing
// ============================================================================

/// Parse a `<composite>` element with all its attributes and child elements.
pub(super) fn parse_composite<R: BufRead>(
    reader: &mut Reader<R>,
    e: &BytesStart,
) -> Result<MjcfComposite> {
    let prefix = get_attribute_opt(e, "prefix").unwrap_or_default();

    let comp_type = match get_attribute_opt(e, "type")
        .ok_or_else(|| MjcfError::missing_attribute("type", "composite"))?
        .as_str()
    {
        "particle" => CompositeType::Particle,
        "grid" => CompositeType::Grid,
        "rope" => CompositeType::Rope,
        "loop" => CompositeType::Loop,
        "cable" => CompositeType::Cable,
        "cloth" => CompositeType::Cloth,
        other => {
            return Err(MjcfError::invalid_attribute(
                "type",
                "composite",
                format!("unknown composite type: \"{other}\""),
            ));
        }
    };

    let count = if let Some(s) = get_attribute_opt(e, "count") {
        let parts: Vec<i32> = s
            .split_whitespace()
            .map(|p| p.parse::<i32>().unwrap_or(1))
            .collect();
        [
            parts.first().copied().unwrap_or(1),
            parts.get(1).copied().unwrap_or(1),
            parts.get(2).copied().unwrap_or(1),
        ]
    } else {
        [1, 1, 1]
    };

    let offset = if let Some(s) = get_attribute_opt(e, "offset") {
        let parts = parse_float_array(&s)?;
        [
            parts.first().copied().unwrap_or(0.0),
            parts.get(1).copied().unwrap_or(0.0),
            parts.get(2).copied().unwrap_or(0.0),
        ]
    } else {
        [0.0; 3]
    };

    let quat = if let Some(s) = get_attribute_opt(e, "quat") {
        let parts = parse_float_array(&s)?;
        [
            parts.first().copied().unwrap_or(1.0),
            parts.get(1).copied().unwrap_or(0.0),
            parts.get(2).copied().unwrap_or(0.0),
            parts.get(3).copied().unwrap_or(0.0),
        ]
    } else {
        [1.0, 0.0, 0.0, 0.0]
    };

    let initial = get_attribute_opt(e, "initial").unwrap_or_else(|| "ball".to_string());

    let size = if let Some(s) = get_attribute_opt(e, "size") {
        let parts = parse_float_array(&s)?;
        [
            parts.first().copied().unwrap_or(1.0),
            parts.get(1).copied().unwrap_or(0.0),
            parts.get(2).copied().unwrap_or(0.0),
        ]
    } else {
        [1.0, 0.0, 0.0]
    };

    let curve = if let Some(s) = get_attribute_opt(e, "curve") {
        parse_curve_shapes(&s)?
    } else {
        [CompositeShape::Zero; 3]
    };

    let uservert = if let Some(s) = get_attribute_opt(e, "vertex") {
        parse_float_array(&s)?
    } else {
        Vec::new()
    };

    // Parse child elements: <joint kind="main">, <geom>, and skip others.
    let mut joint = None;
    let mut geom = None;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref child)) => {
                let child_name = child.name().as_ref().to_vec();
                match child_name.as_slice() {
                    b"joint" => {
                        let kind =
                            get_attribute_opt(child, "kind").unwrap_or_else(|| "main".to_string());
                        if kind == "main" {
                            joint = Some(parse_composite_joint_attrs(child));
                        }
                        // Skip to closing tag (composite <joint> has no children)
                        skip_element(reader, &child_name)?;
                    }
                    b"geom" => {
                        geom = Some(parse_composite_geom_attrs(child)?);
                        skip_element(reader, &child_name)?;
                    }
                    _ => skip_element(reader, &child_name)?,
                }
            }
            Ok(Event::Empty(ref child)) => match child.name().as_ref() {
                b"joint" => {
                    let kind =
                        get_attribute_opt(child, "kind").unwrap_or_else(|| "main".to_string());
                    if kind == "main" {
                        joint = Some(parse_composite_joint_attrs(child));
                    }
                }
                b"geom" => {
                    geom = Some(parse_composite_geom_attrs(child)?);
                }
                _ => {}
            },
            Ok(Event::End(ref end)) if end.name().as_ref() == b"composite" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in composite".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(MjcfComposite {
        prefix,
        comp_type,
        count,
        offset,
        quat,
        initial,
        curve,
        size,
        uservert,
        joint,
        geom,
    })
}

/// Parse attributes of a `<composite><joint kind="main">` child.
pub(super) fn parse_composite_joint_attrs(e: &BytesStart) -> CompositeJoint {
    let range = if let Some(s) = get_attribute_opt(e, "range") {
        parse_float_array(&s).ok().and_then(|parts| {
            if parts.len() >= 2 {
                Some([parts[0], parts[1]])
            } else {
                None
            }
        })
    } else {
        None
    };

    CompositeJoint {
        group: parse_int_attr(e, "group"),
        stiffness: parse_float_attr(e, "stiffness"),
        damping: parse_float_attr(e, "damping"),
        armature: parse_float_attr(e, "armature"),
        frictionloss: parse_float_attr(e, "frictionloss"),
        limited: get_attribute_opt(e, "limited").and_then(|s| match s.as_str() {
            "true" => Some(true),
            "false" => Some(false),
            _ => None,
        }),
        range,
    }
}

/// Parse attributes of a `<composite><geom>` child.
pub(super) fn parse_composite_geom_attrs(e: &BytesStart) -> Result<CompositeGeom> {
    let geom_type = if let Some(s) = get_attribute_opt(e, "type") {
        MjcfGeomType::from_str(&s).ok_or_else(|| MjcfError::UnknownGeomType(s))?
    } else {
        MjcfGeomType::Capsule
    };

    let size = if let Some(s) = get_attribute_opt(e, "size") {
        let parts = parse_float_array(&s)?;
        [
            parts.first().copied().unwrap_or(0.005),
            parts.get(1).copied().unwrap_or(0.0),
            parts.get(2).copied().unwrap_or(0.0),
        ]
    } else {
        [0.005, 0.0, 0.0]
    };

    let rgba = if let Some(s) = get_attribute_opt(e, "rgba") {
        let parts = parse_float_array(&s)?;
        if parts.len() >= 4 {
            Some([parts[0], parts[1], parts[2], parts[3]])
        } else {
            None
        }
    } else {
        None
    };

    let friction = if let Some(s) = get_attribute_opt(e, "friction") {
        let parts = parse_float_array(&s)?;
        if parts.len() >= 3 {
            Some([parts[0], parts[1], parts[2]])
        } else {
            None
        }
    } else {
        None
    };

    let solref = if let Some(s) = get_attribute_opt(e, "solref") {
        let parts = parse_float_array(&s)?;
        if parts.len() >= 2 {
            Some([parts[0], parts[1]])
        } else {
            None
        }
    } else {
        None
    };

    let solimp = if let Some(s) = get_attribute_opt(e, "solimp") {
        let parts = parse_float_array(&s)?;
        if parts.len() >= 5 {
            Some([parts[0], parts[1], parts[2], parts[3], parts[4]])
        } else {
            None
        }
    } else {
        None
    };

    Ok(CompositeGeom {
        geom_type,
        size,
        rgba,
        contype: parse_int_attr(e, "contype"),
        conaffinity: parse_int_attr(e, "conaffinity"),
        condim: parse_int_attr(e, "condim"),
        group: parse_int_attr(e, "group"),
        friction,
        mass: parse_float_attr(e, "mass"),
        density: parse_float_attr(e, "density"),
        solmix: parse_float_attr(e, "solmix"),
        solref,
        solimp,
        margin: parse_float_attr(e, "margin"),
        gap: parse_float_attr(e, "gap"),
        material: get_attribute_opt(e, "material"),
        priority: parse_int_attr(e, "priority"),
    })
}

/// Parse curve shapes from space-separated string (e.g., "s 0 l").
pub(super) fn parse_curve_shapes(s: &str) -> Result<[CompositeShape; 3]> {
    let mut shapes = [CompositeShape::Zero; 3];
    for (i, token) in s.split_whitespace().enumerate() {
        if i >= 3 {
            break;
        }
        shapes[i] = match token {
            "s" => CompositeShape::Sin,
            "c" => CompositeShape::Cos,
            "l" => CompositeShape::Line,
            "0" | "zero" => CompositeShape::Zero,
            other => {
                return Err(MjcfError::invalid_attribute(
                    "curve",
                    "composite",
                    format!("unknown curve shape: \"{other}\""),
                ));
            }
        };
    }
    Ok(shapes)
}
