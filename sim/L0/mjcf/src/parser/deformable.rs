//! `<deformable>`'s `<flex>` and `<skin>` parsing. A `<flexcomp>` in it is
//! refused, as MuJoCo 3.5.0's schema refuses it; MuJoCo reads one in a body
//! (`super::flexcomp`).

use nalgebra::Vector3;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfFlex, MjcfSkin, MjcfSkinBone, MjcfSkinVertex};

use super::attrs::{Attrs, skip_element};

// ============================================================================
// Deformable / Skin parsing
// ============================================================================

/// Parse deformable section (contains skin and flex elements; `empty`: it
/// closed itself, so it has none). A `<flexcomp>` in it is refused, as MuJoCo
/// 3.5.0's schema refuses it ('unrecognized element'), where the reader meets
/// it. MuJoCo's order beside another fault differs both ways, one kind per
/// row of `refusal_order.rs`'s `SCHEMA_COSTS` and `REFUSAL_FIRST_COSTS`
/// (ledgers L315, L353).
pub(super) fn parse_deformable<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<(Vec<MjcfSkin>, Vec<MjcfFlex>)> {
    let mut skins = Vec::new();
    let mut flex_bodies = Vec::new();
    if empty {
        return Ok((skins, flex_bodies));
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"skin" => skins.push(parse_skin(reader, e, child_empty)?),
                    b"flex" => flex_bodies.push(parse_flex(reader, e, child_empty)?),
                    b"flexcomp" => {
                        return Err(MjcfError::unknown_element("flexcomp", "deformable"));
                    }
                    name => skip_element(reader, name, child_empty)?,
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"deformable" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("deformable"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok((skins, flex_bodies))
}

/// Parse a `<flex>` element: MuJoCo 3.5.0's, whose vertices are the bodies
/// `body` names, with `vertex` offsets and `element` ids, and its `<contact>`,
/// `<elasticity>` and `<edge>` children. MuJoCo's schema refuses the
/// `<vertex>`, `<element>` and `<pin>` children, and the attributes sim-mjcf
/// once read for its own flexcomp (`mass`, `density`, `inertiabox`, `scale`,
/// `quat`, `file`: the first written, before any of this `<flex>`'s values is
/// read, as MuJoCo's schema names it; other faults beside it:
/// [`parse_deformable`]), before a missing `body` or `element`, as here; a bad
/// attribute value read here before a refused child is refused first here,
/// the child first by MuJoCo's schema (Rigid-loading L47). `empty`: it closed
/// itself, so it has no children.
pub(super) fn parse_flex<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfFlex> {
    let a = Attrs::new(start)?;
    if let Some(name) = a.first_of(&["mass", "density", "inertiabox", "scale", "quat", "file"]) {
        return Err(MjcfError::unknown_attribute("flex", name));
    }
    let mut flex = parse_flex_attrs(start)?;
    read_flex_data(&a, &mut flex)?;
    // The vertex bodies; MuJoCo 3.5.0 requires `body` (checked below).
    if let Some(s) = a.text("body") {
        flex.body = s.split_whitespace().map(str::to_string).collect();
    }
    // An empty `node` is no node, as MuJoCo 3.5.0 reads it (measured).
    if let Some(s) = a.text("node") {
        flex.node = s.split_whitespace().map(str::to_string).collect();
    }
    if !empty {
        read_flex_children(reader, &mut flex)?;
    }

    // MuJoCo 3.5.0 requires both (`xml_native_reader.cc`, `OneFlex`); it
    // reads an empty `element` as missing, an empty `body` as given (measured).
    if !a.has("body") {
        return Err(MjcfError::missing_attribute("flex", "body"));
    }
    if flex.elements.is_empty() {
        return Err(MjcfError::missing_attribute("flex", "element"));
    }
    Ok(flex)
}

/// A `<flex>`'s children, up to its end tag: its `<contact>`, `<elasticity>`
/// and `<edge>` attributes onto `flex`, and MuJoCo's refused children refused.
fn read_flex_children<R: BufRead>(reader: &mut Reader<R>, flex: &mut MjcfFlex) -> Result<()> {
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"contact" => parse_flex_contact_attrs(e, flex)?,
                    b"elasticity" => parse_flex_elasticity_attrs(e, flex)?,
                    b"edge" => parse_flex_edge_attrs(e, flex)?,
                    name @ (b"vertex" | b"element" | b"pin") => {
                        return Err(MjcfError::unknown_element(
                            String::from_utf8_lossy(name),
                            "flex",
                        ));
                    }
                    _ => {}
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flex" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("flex"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(())
}

/// A `<flex>`'s `vertex` and `element` attributes, read as MuJoCo 3.5.0
/// reads them (`OneFlex`) and sized as its compiler checks them
/// (`mjCFlex::Compile`, `user_mesh.cc`): three reals per vertex, `dim + 1`
/// vertex ids per element. A trailing partial element is kept for the
/// builder's element check, which refuses it.
fn read_flex_data(a: &Attrs, flex: &mut MjcfFlex) -> Result<()> {
    if let Some(reals) = a.vector("vertex")? {
        if reals.len() % 3 != 0 {
            return Err(MjcfError::invalid_value(
                a.element(),
                "vertex",
                "vert size must be a multiple of 3",
            ));
        }
        flex.vertices = reals
            .chunks(3)
            .map(|c| Vector3::new(c[0], c[1], c[2]))
            .collect();
    }
    if let Some(ids) = indices(a, "element")? {
        // Under a dim that is not 1, 2 or 3 the ids stay one element, and the
        // builder refuses the dim, as MuJoCo's compiler does.
        let size = if (1..=3).contains(&flex.dim) {
            flex.dim + 1
        } else {
            ids.len()
        };
        flex.elements = ids.chunks(size).map(<[usize]>::to_vec).collect();
    }
    Ok(())
}

/// A `<flex>`'s own attributes: `name`, `dim`, `radius` and `group`. Its
/// contact, elasticity and edge attributes are its children's, read by
/// `parse_flex_contact_attrs`, `parse_flex_elasticity_attrs` and
/// `parse_flex_edge_attrs`.
fn parse_flex_attrs(e: &BytesStart) -> Result<MjcfFlex> {
    let a = Attrs::new(e)?;
    let mut flex = MjcfFlex::default();

    if let Some(s) = a.string("name") {
        flex.name = s;
    }
    if let Some(dim) = a.int("dim")? {
        flex.dim = usize::try_from(dim)
            .map_err(|_| MjcfError::invalid_value(a.element(), "dim", "dim is negative"))?;
    }
    if let Some(radius) = a.real("radius")? {
        flex.radius = radius;
    }
    // Visualization group (0-5).
    if let Some(group) = a.int("group")? {
        flex.group = group;
    }
    Ok(flex)
}

/// Parse `<contact>` child element attributes into MjcfFlex, in the order
/// MuJoCo 3.5.0's readers of a `<flex>` and a `<flexcomp>` read them
/// (`xml_native_reader.cc:2796-2816`), so that of two malformed attributes
/// the same one is named.
pub(super) fn parse_flex_contact_attrs(e: &BytesStart, flex: &mut MjcfFlex) -> Result<()> {
    let a = Attrs::new(e)?;
    flex.contype = a.int("contype")?;
    flex.conaffinity = a.int("conaffinity")?;
    if let Some(condim) = a.int("condim")? {
        flex.condim = condim;
    }
    if let Some(priority) = a.int("priority")? {
        flex.priority = priority;
    }
    // MuJoCo friction is real(3): "tangential torsional rolling", read with
    // `exact = false`: the values not given keep the flex's, MuJoCo's
    // defaults, as solref and solimp below do.
    if let Some(friction) = a.prefix("friction")? {
        flex.friction = Vector3::from(friction.over(flex.friction.into()));
    }
    if let Some(solmix) = a.real("solmix")? {
        flex.solmix = solmix;
    }
    if let Some(solref) = a.prefix("solref")? {
        flex.solref = solref.over(flex.solref);
    }
    if let Some(solimp) = a.prefix("solimp")? {
        flex.solimp = solimp.over(flex.solimp);
    }
    if let Some(margin) = a.real("margin")? {
        flex.margin = margin;
    }
    if let Some(gap) = a.real("gap")? {
        flex.gap = gap;
    }
    // DT-85: Flex contact runtime attributes
    if let Some(internal) = a.boolean("internal")? {
        flex.internal = internal;
    }
    if let Some(s) = a.keyword_text("selfcollide", keywords::flex_selfcollide)? {
        flex.selfcollide = Some(s);
    }
    if let Some(vertcollide) = a.boolean("vertcollide")? {
        flex.vertcollide = vertcollide;
    }
    if let Some(passive) = a.boolean("passive")? {
        flex.passive = passive;
    }
    if let Some(val) = a.int("activelayers")? {
        flex.activelayers = val;
    }
    Ok(())
}

/// Parse `<elasticity>` child element attributes into MjcfFlex.
pub(super) fn parse_flex_elasticity_attrs(e: &BytesStart, flex: &mut MjcfFlex) -> Result<()> {
    let a = Attrs::new(e)?;
    if let Some(young) = a.real("young")? {
        flex.young = young;
    }
    if let Some(poisson) = a.real("poisson")? {
        flex.poisson = poisson;
    }
    if let Some(damping) = a.real("damping")? {
        flex.damping = damping;
    }
    if let Some(thickness) = a.real("thickness")? {
        flex.thickness = thickness;
    }
    if let Some(elastic2d) = a.keyword("elastic2d", keywords::elastic2d)? {
        flex.elastic2d = elastic2d;
    }
    // §42B S4: bending_model, an extension attribute, written only beside
    // `elastic2d` bend or both. MuJoCo's schema refuses a second
    // `<elasticity>`; this parser does not (ledger L315).
    if let Some(model) = a.keyword("bending_model", keywords::bending_model)? {
        if !flex.elastic2d.bends() {
            return Err(MjcfError::invalid_value(
                "elasticity",
                "bending_model",
                "a bending model needs elastic2d bend or both",
            ));
        }
        flex.bending_model = model;
    }
    Ok(())
}

/// Parse `<edge>` child element attributes into MjcfFlex.
/// These are passive spring-damper coefficients (not constraint params).
pub(super) fn parse_flex_edge_attrs(e: &BytesStart, flex: &mut MjcfFlex) -> Result<()> {
    let a = Attrs::new(e)?;
    if let Some(stiffness) = a.real("stiffness")? {
        flex.edge_stiffness = stiffness;
    }
    if let Some(damping) = a.real("damping")? {
        flex.edge_damping = damping;
    }
    Ok(())
}

/// Non-negative integers: vertex or element indices.
fn indices(a: &Attrs, name: &str) -> Result<Option<Vec<usize>>> {
    a.int_vector(name)?
        .map(|ints| {
            ints.into_iter()
                .map(|i| {
                    usize::try_from(i).map_err(|_| {
                        MjcfError::invalid_value(
                            a.element(),
                            name,
                            format!("index {i} is negative"),
                        )
                    })
                })
                .collect()
        })
        .transpose()
}

/// Parse a skin element and its children (`empty`: it closed itself, so it
/// has none).
pub(super) fn parse_skin<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfSkin> {
    let mut skin = parse_skin_attrs(start)?;
    if empty {
        return Ok(skin);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"bone" => skin.bones.push(parse_skin_bone_attrs(e)?),
                    b"vertex" => skin.vertices.push(parse_skin_vertex_attrs(e)?),
                    _ => {}
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"skin" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("skin")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(skin)
}

/// Parse skin element attributes.
pub(super) fn parse_skin_attrs(e: &BytesStart) -> Result<MjcfSkin> {
    let a = Attrs::new(e)?;
    let mut skin = MjcfSkin::default();

    skin.name = a.string("name").unwrap_or_default();
    skin.mesh = a.string("mesh");
    skin.material = a.string("material");

    if let Some(rgba) = a.vec4("rgba")? {
        skin.rgba = rgba;
    }

    if let Some(inflate) = a.real("inflate")? {
        skin.inflate = inflate;
    }

    // Parse embedded vertex positions if present
    skin.vertex_positions = a.vector("vertex")?;

    // Parse embedded face data if present
    if let Some(faces) = a.int_vector("face")? {
        let faces = faces
            .into_iter()
            .map(u32::try_from)
            .collect::<std::result::Result<Vec<u32>, _>>()
            .map_err(|_| MjcfError::invalid_value("skin", "face", "a vertex index is negative"))?;
        skin.faces = Some(faces);
    }

    Ok(skin)
}

/// Parse skin bone element attributes.
pub(super) fn parse_skin_bone_attrs(e: &BytesStart) -> Result<MjcfSkinBone> {
    let a = Attrs::new(e)?;
    let mut bone = MjcfSkinBone::default();

    // body is required
    bone.body = a
        .string("body")
        .ok_or_else(|| MjcfError::missing_attribute("bone", "body"))?;

    // MuJoCo 3.5.0 requires the bind pose too (measured).
    bone.bindpos = a
        .vec3("bindpos")?
        .ok_or_else(|| MjcfError::missing_attribute("bone", "bindpos"))?;
    bone.bindquat = a
        .vec4("bindquat")?
        .ok_or_else(|| MjcfError::missing_attribute("bone", "bindquat"))?;

    Ok(bone)
}

/// Parse skin vertex element attributes.
pub(super) fn parse_skin_vertex_attrs(e: &BytesStart) -> Result<MjcfSkinVertex> {
    let a = Attrs::new(e)?;
    let index = |name: &str| -> Result<usize> {
        let i = a
            .int(name)?
            .ok_or_else(|| MjcfError::missing_attribute("vertex", name))?;
        usize::try_from(i)
            .map_err(|_| MjcfError::invalid_value("vertex", name, format!("index {i} is negative")))
    };
    // id is required (vertex index in mesh)
    let id = index("id")?;

    // bone is required (bone index in skin)
    let bone = index("bone")?;

    // weight defaults to 1.0 if not specified
    let weight = a.real("weight")?.unwrap_or(1.0);

    Ok(MjcfSkinVertex::new(id, bone, weight))
}
