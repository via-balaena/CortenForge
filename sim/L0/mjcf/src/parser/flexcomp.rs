//! A `<flexcomp>` written in a body, a frame or the world body, read as
//! MuJoCo 3.5.0's `OneFlexcomp` reads it (`xml_native_reader.cc:2702-2858`):
//! its attributes in that order, then its first `<edge>`, its first
//! `<elasticity>`, the two checked together, its first `<contact>`, every
//! `<pin>` and its `<plugin>`, in that order whatever order the document
//! writes them in. Its visual attributes (`material`, `rgba`, `flatskin`,
//! `texcoord`) are read for their form and not kept, as a `<flex>`'s are
//! not. The builder makes it (`builder::flexcomp`).

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;
use crate::types::{FlexcompType, MjcfFlexcomp, MjcfFlexcompEdgeEquality};

use super::attrs::{Attrs, skip_element};
use super::deformable::{parse_flex_contact_attrs, parse_flex_elasticity_attrs};

/// A `<flexcomp>` element (`empty`: it closed itself, so it has no children).
pub(super) fn parse_flexcomp<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfFlexcomp> {
    let mut fc = read_attributes(start)?;
    let children = if empty {
        Children::default()
    } else {
        read_children(reader)?
    };
    if let Some(edge) = &children.edge {
        read_edge(edge, &mut fc)?;
    }
    if let Some(elasticity) = &children.elasticity {
        parse_flex_elasticity_attrs(elasticity, &mut fc.flex)?;
    }
    // `:2789-2791`, before the contact is read.
    if fc.flex.elastic2d.stretches() && fc.flex.edge_equality.is_some() {
        return Err(MjcfError::invalid_value(
            "flexcomp",
            "elastic2d",
            "elasticity and edge constraints cannot both be present",
        ));
    }
    if let Some(contact) = &children.contact {
        parse_flex_contact_attrs(contact, &mut fc.flex)?;
    }
    for pin in &children.pins {
        let a = Attrs::new(pin)?;
        for (name, list) in [
            ("id", &mut fc.pins.id),
            ("range", &mut fc.pins.range),
            ("grid", &mut fc.pins.grid),
            ("gridrange", &mut fc.pins.gridrange),
        ] {
            if let Some(values) = a.int_vector(name)? {
                list.extend(values);
            }
        }
    }
    fc.plugin = children.plugin;
    Ok(fc)
}

/// The flexcomp's attributes, in `OneFlexcomp`'s order (`:2711-2766`).
fn read_attributes(e: &BytesStart) -> Result<MjcfFlexcomp> {
    let a = Attrs::new(e)?;
    let mut fc = MjcfFlexcomp {
        name: a
            .string("name")
            .ok_or_else(|| MjcfError::missing_attribute("flexcomp", "name"))?,
        ..MjcfFlexcomp::default()
    };
    if let Some(kind) = a.keyword("type", keywords::flexcomp_type)? {
        fc.kind = kind;
    }
    if let Some(count) = a.int_array::<3>("count")? {
        fc.count = count;
    }
    if let Some(spacing) = a.array::<3>("spacing")? {
        fc.spacing = spacing;
    }
    if let Some(scale) = a.array::<3>("scale")? {
        fc.scale = scale;
    }
    if let Some(mass) = a.real("mass")? {
        fc.mass = mass;
    }
    if let Some(inertiabox) = a.real("inertiabox")? {
        fc.inertiabox = inertiabox;
    }
    fc.file = a.string("file");
    // `material` (any text), `rgba` and `flatskin`: read, not kept (a
    // `<flex>`'s are not).
    a.array::<4>("rgba")?;
    a.boolean("flatskin")?;
    if let Some(dim) = a.int("dim")? {
        fc.dim = dim;
    }
    if let Some(radius) = a.real("radius")? {
        fc.flex.radius = radius;
    }
    if let Some(group) = a.int("group")? {
        fc.flex.group = group;
    }
    fc.origin = a.array::<3>("origin")?;
    if fc.origin.is_none() && fc.kind == FlexcompType::Mesh && fc.dim == 3 {
        return Err(MjcfError::invalid_value(
            "flexcomp",
            "origin",
            "origin must be specified for mesh flexcomps if dim=3",
        ));
    }
    if let Some(pos) = a.vec3("pos")? {
        fc.pos = pos;
    }
    // As written: MuJoCo reads a flexcomp's `quat` with `ReadAttr`, not
    // `ReadQuat`, so a zero one loads (it turns nothing, `mjuu_rotVecQuat`).
    if let Some(quat) = a.vec4("quat")? {
        fc.quat = quat;
    }
    a.one_orientation()?;
    fc.axisangle = a.vec4("axisangle")?;
    fc.xyaxes = a.array::<6>("xyaxes")?;
    fc.zaxis = a.vec3("zaxis")?;
    fc.euler = a.vec3("euler")?;
    if let Some(rigid) = a.boolean("rigid")? {
        fc.rigid = rigid;
    }
    if let Some(point) = a.vector("point")? {
        fc.point = point;
    }
    if let Some(element) = a.int_vector("element")? {
        fc.element = element;
    }
    // `texcoord`: read, not kept.
    a.vector("texcoord")?;
    if let Some(dof) = a.keyword("dof", keywords::flexcomp_dof)? {
        fc.dof = dof;
    }
    Ok(fc)
}

/// The children MuJoCo reads, each kept to be read in its order: the first
/// `<edge>`, `<elasticity>` and `<contact>` (`FirstChildElement`), every
/// `<pin>`, and whether a `<plugin>` is written.
#[derive(Default)]
struct Children {
    edge: Option<BytesStart<'static>>,
    elasticity: Option<BytesStart<'static>>,
    contact: Option<BytesStart<'static>>,
    pins: Vec<BytesStart<'static>>,
    plugin: bool,
}

/// The flexcomp's children, up to its end tag.
fn read_children<R: BufRead>(reader: &mut Reader<R>) -> Result<Children> {
    let mut children = Children::default();
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                let name = e.name().as_ref().to_vec();
                let owned = e.clone().into_owned();
                match name.as_slice() {
                    b"edge" => {
                        children.edge.get_or_insert(owned);
                    }
                    b"elasticity" => {
                        children.elasticity.get_or_insert(owned);
                    }
                    b"contact" => {
                        children.contact.get_or_insert(owned);
                    }
                    b"pin" => children.pins.push(owned),
                    b"plugin" => children.plugin = true,
                    _ => {}
                }
                skip_element(reader, &name, child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flexcomp" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("flexcomp")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(children)
}

/// The flexcomp's `<edge>` (`:2768-2776`): the equality it asks for, that
/// equality's `solref` and `solimp`, then its passive stiffness and damping.
fn read_edge(e: &BytesStart, fc: &mut MjcfFlexcomp) -> Result<()> {
    let a = Attrs::new(e)?;
    let equality = a.keyword("equality", keywords::flex_edge_equality)?;
    let (solref, solimp) = (a.prefix("solref")?, a.prefix("solimp")?);
    if let Some(stiffness) = a.real("stiffness")? {
        fc.flex.edge_stiffness = stiffness;
    }
    if let Some(damping) = a.real("damping")? {
        fc.flex.edge_damping = damping;
    }
    let vertices = match equality {
        None | Some(keywords::FlexEdgeEquality::None) => return Ok(()),
        Some(keywords::FlexEdgeEquality::Edges) => false,
        Some(keywords::FlexEdgeEquality::Vertices) => true,
    };
    fc.flex.edge_equality = Some(MjcfFlexcompEdgeEquality {
        solref,
        solimp,
        vertices,
    });
    Ok(())
}
