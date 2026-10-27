//! `<composite>` body parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result, ValueCount};
use crate::keywords;

use crate::types::{CompositeGeom, CompositeJoint, CompositeShape, MjcfComposite, MjcfGeomType};

use super::attrs::{Attrs, skip_element};

// ============================================================================
// Composite parsing
// ============================================================================

/// Parse a `<composite>` element with all its attributes and child elements.
pub(super) fn parse_composite<R: BufRead>(
    reader: &mut Reader<R>,
    e: &BytesStart,
) -> Result<MjcfComposite> {
    let a = Attrs::new(e)?;
    let prefix = a.string("prefix").unwrap_or_default();

    let comp_type = a
        .keyword("type", keywords::composite_type)?
        .ok_or_else(|| MjcfError::missing_attribute("composite", "type"))?;

    // One up to three counts, the rest 1 (MuJoCo reads three with `exact =
    // false`).
    let mut count = [1, 1, 1];
    if let Some(given) = a.int_vector("count")? {
        if given.len() > 3 {
            return Err(a.wrong_count("count", ValueCount::AtMost(3), given.len()));
        }
        count[..given.len()].copy_from_slice(&given);
    }

    let offset = a.array::<3>("offset")?.unwrap_or([0.0; 3]);
    let quat = a.array::<4>("quat")?.unwrap_or([1.0, 0.0, 0.0, 0.0]);

    let initial = a.string("initial").unwrap_or_else(|| "ball".to_string());

    let size = a
        .prefix::<3>("size")?
        .map_or([1.0, 0.0, 0.0], |size| size.over([1.0, 0.0, 0.0]));

    let curve = if let Some(s) = a.text("curve") {
        parse_curve_shapes(s)?
    } else {
        [CompositeShape::Zero; 3]
    };

    let uservert = a.vector("vertex")?.unwrap_or_default();

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
                        joint = Some(parse_composite_joint_attrs(child)?);
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
                    joint = Some(parse_composite_joint_attrs(child)?);
                }
                b"geom" => {
                    geom = Some(parse_composite_geom_attrs(child)?);
                }
                _ => {}
            },
            Ok(Event::End(ref end)) if end.name().as_ref() == b"composite" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("composite"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
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

/// Parse attributes of a `<composite><joint kind="main">` child. `kind` is
/// required, and `main` is MuJoCo's one kind (`jkind_map`).
pub(super) fn parse_composite_joint_attrs(e: &BytesStart) -> Result<CompositeJoint> {
    let a = Attrs::new(e)?;
    a.keyword("kind", |s| (s == "main").then_some(()))?
        .ok_or_else(|| MjcfError::missing_attribute("joint", "kind"))?;
    Ok(CompositeJoint {
        group: a.int("group")?,
        stiffness: a.real("stiffness")?,
        damping: a.real("damping")?,
        armature: a.real("armature")?,
        frictionloss: a.real("frictionloss")?,
        // MuJoCo reads this from the <composite> element, which may not hold
        // it, so a written value is never read (D-COMPOSITE-JOINT-LIMITED).
        limited: a.tf_auto("limited")?,
        range: a.array("range")?,
    })
}

/// Parse attributes of a `<composite><geom>` child.
pub(super) fn parse_composite_geom_attrs(e: &BytesStart) -> Result<CompositeGeom> {
    let a = Attrs::new(e)?;
    let geom_type = a
        .keyword("type", MjcfGeomType::from_str)?
        .unwrap_or(MjcfGeomType::Capsule);

    let size = a
        .prefix::<3>("size")?
        .map_or([0.005, 0.0, 0.0], |size| size.over([0.005, 0.0, 0.0]));

    Ok(CompositeGeom {
        geom_type,
        size,
        rgba: a.array("rgba")?,
        contype: a.int("contype")?,
        conaffinity: a.int("conaffinity")?,
        condim: a.int("condim")?,
        group: a.int("group")?,
        friction: a.prefix("friction")?,
        mass: a.real("mass")?,
        density: a.real("density")?,
        solmix: a.real("solmix")?,
        solref: a.prefix("solref")?,
        solimp: a.prefix("solimp")?,
        margin: a.real("margin")?,
        gap: a.real("gap")?,
        material: a.string("material"),
        priority: a.int("priority")?,
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
                return Err(MjcfError::invalid_keyword("composite", "curve", other));
            }
        };
    }
    Ok(shapes)
}
