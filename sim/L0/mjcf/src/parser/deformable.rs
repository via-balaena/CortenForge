//! `<deformable>` flex / flexcomp / skin parsing.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{FlexVertexBodies, MjcfFlex, MjcfSkin, MjcfSkinBone, MjcfSkinVertex};

use super::attrs::{Attrs, skip_element};

// ============================================================================
// Deformable / Skin parsing
// ============================================================================

/// Parse deformable section (contains skin and flex elements).
pub(super) fn parse_deformable<R: BufRead>(
    reader: &mut Reader<R>,
) -> Result<(Vec<MjcfSkin>, Vec<MjcfFlex>)> {
    let mut skins = Vec::new();
    let mut flex_bodies = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"skin" => {
                        let skin = parse_skin(reader, e)?;
                        skins.push(skin);
                    }
                    b"flex" => {
                        let flex = parse_flex(reader, e)?;
                        flex_bodies.push(flex);
                    }
                    b"flexcomp" => {
                        let flex = parse_flexcomp(reader, e)?;
                        flex_bodies.push(flex);
                    }
                    _ => {
                        skip_element(reader, &elem_name)?;
                    }
                }
            }
            Ok(Event::Empty(ref e)) => {
                match e.name().as_ref() {
                    b"skin" => {
                        let skin = parse_skin_attrs(e)?;
                        skins.push(skin);
                    }
                    b"flexcomp" => {
                        // Self-closing <flexcomp .../> — parse attrs and generate mesh
                        let flex = parse_flexcomp_empty(e)?;
                        flex_bodies.push(flex);
                    }
                    _ => {}
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
/// `<vertex>`, `<element>` and `<pin>` children and the `mass` sim-mjcf once
/// read, before a missing `body` or `element`, as here; a bad attribute value
/// read here before a refused child is refused first here, the child first
/// by MuJoCo's schema (Rigid-loading L47).
pub(super) fn parse_flex<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfFlex> {
    let a = Attrs::new(start)?;
    if a.has("mass") {
        return Err(MjcfError::unknown_attribute("flex", "mass"));
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
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"contact" => parse_flex_contact_attrs(e, &mut flex)?,
                    b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex)?,
                    b"edge" => parse_flex_edge_attrs(e, &mut flex)?,
                    b"vertex" | b"element" | b"pin" => {
                        return Err(MjcfError::unknown_element(
                            String::from_utf8_lossy(&elem_name),
                            "flex",
                        ));
                    }
                    _ => {}
                }
                skip_element(reader, &elem_name)?;
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"contact" => parse_flex_contact_attrs(e, &mut flex)?,
                b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex)?,
                b"edge" => parse_flex_edge_attrs(e, &mut flex)?,
                name @ (b"vertex" | b"element" | b"pin") => {
                    return Err(MjcfError::unknown_element(
                        String::from_utf8_lossy(name),
                        "flex",
                    ));
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flex" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("flex"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
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

/// Parse direct `<flex>` / `<flexcomp>` top-level attributes.
///
/// Only parses attributes that belong directly on the element: `name`, `dim`, `radius`, `mass`.
/// Contact, elasticity, and edge attributes belong on child elements and are parsed
/// by `parse_flex_contact_attrs`, `parse_flex_elasticity_attrs`, `parse_flex_edge_attrs`.
///
/// `mass` distributes total mass uniformly across all vertices (MuJoCo `<flexcomp mass="...">`).
/// `density` is a non-standard fallback for element-based mass lumping.
pub(super) fn parse_flex_attrs(e: &BytesStart) -> Result<MjcfFlex> {
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
    // MuJoCo: <flexcomp mass="..."> distributes total mass uniformly across all vertices.
    // When present, overrides element-based mass lumping from density.
    if let Some(mass) = a.real("mass")? {
        flex.mass = Some(mass);
    }
    // Flexcomp attributes (DT-88)
    if let Some(inertiabox) = a.real("inertiabox")? {
        flex.inertiabox = inertiabox;
    }
    flex.flexcomp_scale = a.vec3("scale")?;
    if let Some([w, x, y, z]) = a.array::<4>("quat")? {
        flex.flexcomp_quat = Some(UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)));
    }
    if let Some(s) = a.string("file") {
        flex.flexcomp_file = Some(s);
    }
    Ok(flex)
}

/// Parse `<contact>` child element attributes into MjcfFlex.
pub(super) fn parse_flex_contact_attrs(e: &BytesStart, flex: &mut MjcfFlex) -> Result<()> {
    let a = Attrs::new(e)?;
    if let Some(priority) = a.int("priority")? {
        flex.priority = priority;
    }
    if let Some(solmix) = a.real("solmix")? {
        flex.solmix = solmix;
    }
    if let Some(gap) = a.real("gap")? {
        flex.gap = gap;
    }
    // MuJoCo friction is real(3): "tangential torsional rolling", read with
    // `exact = false`: the values not given keep the flex's, MuJoCo's
    // defaults, as solref and solimp below do.
    if let Some(friction) = a.prefix("friction")? {
        flex.friction = Vector3::from(friction.over(flex.friction.into()));
    }
    if let Some(condim) = a.int("condim")? {
        flex.condim = condim;
    }
    if let Some(margin) = a.real("margin")? {
        flex.margin = margin;
    }
    if let Some(solref) = a.prefix("solref")? {
        flex.solref = solref.over(flex.solref);
    }
    if let Some(solimp) = a.prefix("solimp")? {
        flex.solimp = solimp.over(flex.solimp);
    }
    flex.contype = a.int("contype")?;
    flex.conaffinity = a.int("conaffinity")?;
    if let Some(s) = a.keyword_text("selfcollide", keywords::flex_selfcollide)? {
        flex.selfcollide = Some(s);
    }
    // DT-85: Flex contact runtime attributes
    if let Some(internal) = a.boolean("internal")? {
        flex.internal = internal;
    }
    if let Some(val) = a.int("activelayers")? {
        flex.activelayers = val;
    }
    if let Some(vertcollide) = a.boolean("vertcollide")? {
        flex.vertcollide = vertcollide;
    }
    if let Some(passive) = a.boolean("passive")? {
        flex.passive = passive;
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
    // §42B S4: bending_model, an extension attribute.
    if let Some(model) = a.keyword("bending_model", keywords::bending_model)? {
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

/// Parse a `<flexcomp>` element (procedural mesh generation).
///
/// Generates vertices and elements based on the `type` attribute (grid, box, cylinder).
pub(super) fn parse_flexcomp<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfFlex> {
    let mut flex = flexcomp_mesh(start)?;

    // Parse child elements (<contact>, <elasticity>, <edge>, <pin>)
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"contact" => {
                        parse_flex_contact_attrs(e, &mut flex)?;
                        skip_element(reader, &elem_name)?;
                    }
                    b"elasticity" => {
                        parse_flex_elasticity_attrs(e, &mut flex)?;
                        skip_element(reader, &elem_name)?;
                    }
                    b"edge" => {
                        parse_flex_edge_attrs(e, &mut flex)?;
                        skip_element(reader, &elem_name)?;
                    }
                    b"pin" => {
                        if let Some(ids) = indices(&Attrs::new(e)?, "id")? {
                            flex.pinned.extend(ids);
                        }
                        skip_element(reader, &elem_name)?;
                    }
                    _ => {
                        skip_element(reader, &elem_name)?;
                    }
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"contact" => parse_flex_contact_attrs(e, &mut flex)?,
                    b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex)?,
                    b"edge" => parse_flex_edge_attrs(e, &mut flex)?,
                    b"pin" => {
                        if let Some(ids) = indices(&Attrs::new(e)?, "id")? {
                            flex.pinned.extend(ids);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flexcomp" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("flexcomp"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(flex)
}

/// Parse a self-closing `<flexcomp ... />` (Event::Empty).
///
/// Does the same attribute parsing and mesh generation as `parse_flexcomp`
/// but without consuming reader events (since there's no End event to find).
pub(super) fn parse_flexcomp_empty(e: &BytesStart) -> Result<MjcfFlex> {
    flexcomp_mesh(e)
}

/// A `<flexcomp>`'s attributes and the mesh they generate. MuJoCo reads
/// `count` and `spacing` as exactly three values each (`xml_native_reader.cc`).
fn flexcomp_mesh(e: &BytesStart) -> Result<MjcfFlex> {
    let a = Attrs::new(e)?;
    // A flexcomp makes its vertices' bodies; MuJoCo 3.5.0's schema has no
    // `body` on it ("unrecognized attribute", measured).
    if a.has("body") {
        return Err(MjcfError::unknown_attribute("flexcomp", "body"));
    }
    let mut flex = parse_flex_attrs(e)?;
    flex.vertex_bodies = FlexVertexBodies::Generated;

    let comp_type = a
        .keyword("type", keywords::flexcomp_type)?
        .unwrap_or("grid");
    // MuJoCo's first checks of a flexcomp, in its order (`mjCFlexcomp::Make`,
    // `user_flexcomp.cc:114-124`): its dim, then its count, here before the
    // mesh is generated from them.
    if !(1..=3).contains(&flex.dim) {
        return Err(MjcfError::invalid_value(
            "flexcomp",
            "dim",
            "Invalid dim, must be between 1 and 3",
        ));
    }
    let count = parse_flexcomp_count(&a)?;
    if count.contains(&0) {
        return Err(MjcfError::invalid_value(
            "flexcomp",
            "count",
            "Count too small",
        ));
    }
    let spacing = a.array::<3>("spacing")?.unwrap_or([0.02; 3]);

    match comp_type {
        "grid" => generate_grid(&mut flex, count, spacing),
        "box" => generate_box_mesh(&mut flex, count, spacing),
        // A type of MuJoCo's this `<deformable>` form does not generate (the
        // form itself is sim-mjcf's, refused by MuJoCo's schema).
        other => {
            return Err(MjcfError::invalid_value(
                "flexcomp",
                "type",
                format!(
                    "sim-mjcf's <deformable><flexcomp> generates only grid and box, not {other}"
                ),
            ));
        }
    }

    // Apply scale and rotation to generated vertices (DT-88)
    apply_flexcomp_transforms(&mut flex);

    Ok(flex)
}

/// Apply flexcomp scale and quat transforms to generated vertices.
/// Order: scale first (component-wise), then quaternion rotation.
pub(super) fn apply_flexcomp_transforms(flex: &mut MjcfFlex) {
    if let Some(scale) = flex.flexcomp_scale {
        for v in &mut flex.vertices {
            v.x *= scale.x;
            v.y *= scale.y;
            v.z *= scale.z;
        }
    }
    if let Some(ref quat) = flex.flexcomp_quat {
        for v in &mut flex.vertices {
            *v = quat.transform_vector(v);
        }
    }
}

/// Parse flexcomp count attribute (3 ints).
pub(super) fn parse_flexcomp_count(a: &Attrs) -> Result<[usize; 3]> {
    let Some(count) = a.int_array::<3>("count")? else {
        return Ok([10, 10, 1]);
    };
    let mut out = [0; 3];
    for (o, n) in out.iter_mut().zip(count) {
        *o = usize::try_from(n)
            .map_err(|_| MjcfError::invalid_value("flexcomp", "count", "a count is negative"))?;
    }
    Ok(out)
}

/// Non-negative integers: vertex, element or pin indices.
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

/// Generate a 2D grid mesh for flexcomp type="grid".
#[allow(clippy::cast_precision_loss)] // mesh dimensions never exceed 2^52
pub(super) fn generate_grid(flex: &mut MjcfFlex, count: [usize; 3], spacing: [f64; 3]) {
    flex.dim = 2;
    let nx = count[0];
    let ny = count[1];

    // Generate vertices
    for iy in 0..ny {
        for ix in 0..nx {
            let x = ix as f64 * spacing[0];
            let y = iy as f64 * spacing[1];
            flex.vertices.push(Vector3::new(x, y, 0.0));
        }
    }

    // Generate triangles (2 per quad)
    for iy in 0..(ny - 1) {
        for ix in 0..(nx - 1) {
            let v00 = iy * nx + ix;
            let v10 = iy * nx + ix + 1;
            let v01 = (iy + 1) * nx + ix;
            let v11 = (iy + 1) * nx + ix + 1;
            flex.elements.push(vec![v00, v10, v01]);
            flex.elements.push(vec![v10, v11, v01]);
        }
    }
}

/// Generate a 3D box mesh for flexcomp type="box".
#[allow(clippy::cast_precision_loss)] // mesh dimensions never exceed 2^52
pub(super) fn generate_box_mesh(flex: &mut MjcfFlex, count: [usize; 3], spacing: [f64; 3]) {
    flex.dim = 3;
    let nx = count[0];
    let ny = count[1];
    let nz = count[2];

    // Generate vertices
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let x = ix as f64 * spacing[0];
                let y = iy as f64 * spacing[1];
                let z = iz as f64 * spacing[2];
                flex.vertices.push(Vector3::new(x, y, z));
            }
        }
    }

    // Generate tetrahedra (5 per cube)
    for iz in 0..(nz - 1) {
        for iy in 0..(ny - 1) {
            for ix in 0..(nx - 1) {
                let v = |dx: usize, dy: usize, dz: usize| -> usize {
                    (iz + dz) * ny * nx + (iy + dy) * nx + (ix + dx)
                };
                // 5-tet decomposition of a cube
                let v000 = v(0, 0, 0);
                let v100 = v(1, 0, 0);
                let v010 = v(0, 1, 0);
                let v110 = v(1, 1, 0);
                let v001 = v(0, 0, 1);
                let v101 = v(1, 0, 1);
                let v011 = v(0, 1, 1);
                let v111 = v(1, 1, 1);
                flex.elements.push(vec![v000, v100, v010, v001]);
                flex.elements.push(vec![v100, v110, v010, v111]);
                flex.elements.push(vec![v001, v101, v100, v111]);
                flex.elements.push(vec![v001, v011, v010, v111]);
                flex.elements.push(vec![v100, v010, v001, v111]);
            }
        }
    }
}

/// Parse a skin element and its children.
pub(super) fn parse_skin<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfSkin> {
    let mut skin = parse_skin_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"bone" => {
                        let bone = parse_skin_bone_attrs(e)?;
                        skin.bones.push(bone);
                        // Skip to closing tag if not self-closing
                        skip_element(reader, &elem_name)?;
                    }
                    b"vertex" => {
                        let vertex = parse_skin_vertex_attrs(e)?;
                        skin.vertices.push(vertex);
                        // Skip to closing tag if not self-closing
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"bone" => {
                    let bone = parse_skin_bone_attrs(e)?;
                    skin.bones.push(bone);
                }
                b"vertex" => {
                    let vertex = parse_skin_vertex_attrs(e)?;
                    skin.vertices.push(vertex);
                }
                _ => {}
            },
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
