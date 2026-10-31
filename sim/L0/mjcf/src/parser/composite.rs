//! `<composite>` body parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result, ValueCount};
use crate::keywords;

use crate::types::{
    CompositeGeom, CompositeJoint, CompositeShape, CompositeSite, MjcfComposite, MjcfGeomType,
};

use super::attrs::{Attrs, is_c_space, skip_element};

// ============================================================================
// Composite parsing
// ============================================================================

/// Parse a `<composite>` element with all its attributes and child elements
/// (`empty`: it closed itself, so it has none).
pub(super) fn parse_composite<R: BufRead>(
    reader: &mut Reader<R>,
    e: &BytesStart,
    empty: bool,
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
        parse_curve_shapes(&a, s)?
    } else {
        [CompositeShape::Zero; 3]
    };

    let uservert = a.vector("vertex")?.unwrap_or_default();

    let (joint, geom, site) = if empty {
        (None, None, None)
    } else {
        parse_composite_children(reader)?
    };

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
        site,
    })
}

/// A `<composite>`'s children, up to its end tag: its `<joint kind="main">`,
/// `<geom>` and `<site>`; others are skipped.
fn parse_composite_children<R: BufRead>(
    reader: &mut Reader<R>,
) -> Result<(
    Option<CompositeJoint>,
    Option<CompositeGeom>,
    Option<CompositeSite>,
)> {
    let mut joint = None;
    let mut geom = None;
    let mut site = None;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref child) | Event::Empty(ref child))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match child.name().as_ref() {
                    b"joint" => joint = Some(parse_composite_joint_attrs(child)?),
                    b"geom" => geom = Some(parse_composite_geom_attrs(child)?),
                    b"site" => site = Some(parse_composite_site_attrs(child)?),
                    _ => {}
                }
                skip_element(reader, child.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref end)) if end.name().as_ref() == b"composite" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("composite"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok((joint, geom, site))
}

/// Parse attributes of a `<composite><site>` child: its `size` over MuJoCo's
/// default 0.005 for each, element-wise (`ReadAttr` with `exact` false), its
/// `group`, `material` and `rgba` (`xml_native_reader.cc`, OneComposite).
pub(super) fn parse_composite_site_attrs(e: &BytesStart) -> Result<CompositeSite> {
    let a = Attrs::new(e)?;
    Ok(CompositeSite {
        size: a
            .prefix::<3>("size")?
            .map_or([0.005; 3], |size| size.over([0.005; 3])),
        group: a.int("group")?,
        material: a.string("material"),
        rgba: a.array("rgba")?,
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

/// Parse attributes of a `<composite><geom>` child, over MuJoCo's composite
/// default (`mjCComposite`'s `def`, made by `mjs_defaultGeom`,
/// `user_init.c:109-113`): a sphere of size 0. A cable refuses a sphere
/// (`MakeCable`, `user_composite.cc:249-254`) and the size check a size of 0
/// (`checksize`), so a cable's geom must write its type and its size.
pub(super) fn parse_composite_geom_attrs(e: &BytesStart) -> Result<CompositeGeom> {
    let a = Attrs::new(e)?;
    let geom_type = a
        .keyword("type", MjcfGeomType::from_str)?
        .unwrap_or(MjcfGeomType::Sphere);

    let size = a
        .prefix::<3>("size")?
        .map_or([0.0; 3], |size| size.over([0.0; 3]));

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

/// A `<composite>`'s `curve`: up to three of MuJoCo's shapes
/// ([`keywords::composite_shape`]), split on C's `isspace`, the rest zero;
/// four or more are refused, as MuJoCo's "maximum of 3 components". Space
/// after the last shape is read as no more shapes. MuJoCo's read loop takes
/// the last shape once more there (`xml_native_reader.cc`, OneComposite): it
/// reads `"s "` as `"s s"` (D-CURVE-TRAILING-SPACE), and refuses three shapes
/// then space, or a curve of only space (D-CURVE-WHITESPACE).
fn parse_curve_shapes(a: &Attrs, text: &str) -> Result<[CompositeShape; 3]> {
    let tokens: Vec<&str> = text.split(is_c_space).filter(|t| !t.is_empty()).collect();
    if tokens.len() > 3 {
        return Err(a.wrong_count("curve", ValueCount::AtMost(3), tokens.len()));
    }
    let mut shapes = [CompositeShape::Zero; 3];
    for (shape, token) in shapes.iter_mut().zip(tokens) {
        *shape = keywords::composite_shape(token)
            .ok_or_else(|| MjcfError::invalid_keyword("composite", "curve", token))?;
    }
    Ok(shapes)
}
