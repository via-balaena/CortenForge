//! `<deformable>` flex / flexcomp / skin parsing.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfFlex, MjcfSkin, MjcfSkinBone, MjcfSkinVertex};

use super::attrs::{
    get_attribute_opt, parse_float_array, parse_float_attr, parse_int_array, parse_int_attr,
    parse_vector3, parse_vector4, skip_element,
};

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
                        let skin = parse_skin_attrs(e);
                        skins.push(skin);
                    }
                    b"flexcomp" => {
                        // Self-closing <flexcomp .../> — parse attrs and generate mesh
                        let flex = parse_flexcomp_empty(e);
                        flex_bodies.push(flex);
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"deformable" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in deformable".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok((skins, flex_bodies))
}

/// Parse a `<flex>` element and its children (`<vertex>`, `<element>`, `<pin>`).
pub(super) fn parse_flex<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfFlex> {
    let mut flex = parse_flex_attrs(start);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"contact" => {
                        parse_flex_contact_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"elasticity" => {
                        parse_flex_elasticity_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"edge" => {
                        parse_flex_edge_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"vertex" => {
                        // Parse child vertex elements until end of <vertex>
                        if let Some(s) = get_attribute_opt(e, "pos") {
                            // Inline attribute: flat array of xyz triples
                            let floats = parse_float_array(&s)?;
                            for chunk in floats.chunks(3) {
                                if chunk.len() == 3 {
                                    flex.vertices
                                        .push(Vector3::new(chunk[0], chunk[1], chunk[2]));
                                }
                            }
                        }
                        skip_element(reader, &elem_name)?;
                    }
                    b"element" => {
                        if let Some(s) = get_attribute_opt(e, "data") {
                            // Parse element connectivity
                            let ints: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
                            let verts_per_elem = match flex.dim {
                                1 => 2, // cable: edges
                                3 => 4, // solid: tetrahedra
                                _ => 3, // shell (dim=2) and default: triangles
                            };
                            for chunk in ints.chunks(verts_per_elem) {
                                if chunk.len() == verts_per_elem {
                                    flex.elements.push(chunk.to_vec());
                                }
                            }
                        }
                        skip_element(reader, &elem_name)?;
                    }
                    b"pin" => {
                        if let Some(s) = get_attribute_opt(e, "id") {
                            let ids: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
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
                    b"contact" => parse_flex_contact_attrs(e, &mut flex),
                    b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex),
                    b"edge" => parse_flex_edge_attrs(e, &mut flex),
                    b"vertex" => {
                        if let Some(s) = get_attribute_opt(e, "pos") {
                            let floats = parse_float_array(&s)?;
                            for chunk in floats.chunks(3) {
                                if chunk.len() == 3 {
                                    flex.vertices
                                        .push(Vector3::new(chunk[0], chunk[1], chunk[2]));
                                }
                            }
                        }
                    }
                    b"element" => {
                        if let Some(s) = get_attribute_opt(e, "data") {
                            let ints: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
                            let verts_per_elem = match flex.dim {
                                1 => 2,
                                3 => 4,
                                _ => 3, // shell (dim=2) and default
                            };
                            for chunk in ints.chunks(verts_per_elem) {
                                if chunk.len() == verts_per_elem {
                                    flex.elements.push(chunk.to_vec());
                                }
                            }
                        }
                    }
                    b"pin" => {
                        if let Some(s) = get_attribute_opt(e, "id") {
                            let ids: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
                            flex.pinned.extend(ids);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flex" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in flex".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(flex)
}

/// Parse direct `<flex>` / `<flexcomp>` top-level attributes.
///
/// Only parses attributes that belong directly on the element: `name`, `dim`, `radius`, `mass`.
/// Contact, elasticity, and edge attributes belong on child elements and are parsed
/// by `parse_flex_contact_attrs`, `parse_flex_elasticity_attrs`, `parse_flex_edge_attrs`.
///
/// `mass` distributes total mass uniformly across all vertices (MuJoCo `<flexcomp mass="...">`).
/// `density` is a non-standard fallback for element-based mass lumping.
pub(super) fn parse_flex_attrs(e: &BytesStart) -> MjcfFlex {
    let mut flex = MjcfFlex::default();

    if let Some(s) = get_attribute_opt(e, "name") {
        flex.name = s;
    }
    if let Some(s) = get_attribute_opt(e, "dim") {
        flex.dim = s.parse().unwrap_or(2);
    }
    if let Some(s) = get_attribute_opt(e, "radius") {
        flex.radius = s.parse().unwrap_or(0.005);
    }
    // Body names for vertex attachment (required on <flex>, auto-generated by <flexcomp>).
    if let Some(s) = get_attribute_opt(e, "body") {
        flex.body = s.split_whitespace().map(|t| t.to_string()).collect();
    }
    // Node body names (alternative to <vertex> positions).
    if let Some(s) = get_attribute_opt(e, "node") {
        flex.node = s.split_whitespace().map(|t| t.to_string()).collect();
    }
    // Visualization group (0-5).
    if let Some(group) = parse_int_attr(e, "group") {
        flex.group = group;
    }
    // MuJoCo: <flexcomp mass="..."> distributes total mass uniformly across all vertices.
    // When present, overrides element-based mass lumping from density.
    if let Some(s) = get_attribute_opt(e, "mass") {
        flex.mass = s.parse().ok();
    }
    // Flexcomp attributes (DT-88)
    if let Some(s) = get_attribute_opt(e, "inertiabox") {
        flex.inertiabox = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "scale") {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|t| t.parse().ok())
            .collect();
        if vals.len() >= 3 {
            flex.flexcomp_scale = Some(Vector3::new(vals[0], vals[1], vals[2]));
        }
    }
    if let Some(s) = get_attribute_opt(e, "quat") {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|t| t.parse().ok())
            .collect();
        if vals.len() >= 4 {
            flex.flexcomp_quat = Some(UnitQuaternion::from_quaternion(Quaternion::new(
                vals[0], vals[1], vals[2], vals[3],
            )));
        }
    }
    if let Some(s) = get_attribute_opt(e, "file") {
        flex.flexcomp_file = Some(s);
    }
    flex
}

/// Parse `<contact>` child element attributes into MjcfFlex.
pub(super) fn parse_flex_contact_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "priority") {
        flex.priority = s.parse().unwrap_or(0);
    }
    if let Some(s) = get_attribute_opt(e, "solmix") {
        flex.solmix = s.parse().unwrap_or(1.0);
    }
    if let Some(s) = get_attribute_opt(e, "gap") {
        flex.gap = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "friction") {
        // MuJoCo friction is real(3): "tangential torsional rolling".
        // Accepts 1–3 values, filling MuJoCo defaults for missing components.
        let parts: Vec<f64> = s
            .split_whitespace()
            .filter_map(|t| t.parse().ok())
            .collect();
        flex.friction = Vector3::new(
            parts.first().copied().unwrap_or(1.0),
            parts.get(1).copied().unwrap_or(0.005),
            parts.get(2).copied().unwrap_or(0.0001),
        );
    }
    if let Some(s) = get_attribute_opt(e, "condim") {
        flex.condim = s.parse().unwrap_or(3);
    }
    if let Some(s) = get_attribute_opt(e, "margin") {
        flex.margin = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "solref") {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|t| t.parse().ok())
            .collect();
        if vals.len() >= 2 {
            flex.solref = [vals[0], vals[1]];
        }
    }
    if let Some(s) = get_attribute_opt(e, "solimp") {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|t| t.parse().ok())
            .collect();
        if vals.len() >= 5 {
            flex.solimp = [vals[0], vals[1], vals[2], vals[3], vals[4]];
        }
    }
    flex.contype = parse_int_attr(e, "contype");
    flex.conaffinity = parse_int_attr(e, "conaffinity");
    if let Some(s) = get_attribute_opt(e, "selfcollide") {
        flex.selfcollide = Some(s);
    }
    // DT-85: Flex contact runtime attributes
    if let Some(s) = get_attribute_opt(e, "internal") {
        flex.internal = s == "true";
    }
    if let Some(val) = parse_int_attr(e, "activelayers") {
        flex.activelayers = val;
    }
    if let Some(s) = get_attribute_opt(e, "vertcollide") {
        flex.vertcollide = s == "true";
    }
    if let Some(s) = get_attribute_opt(e, "passive") {
        flex.passive = s == "true";
    }
}

/// Parse `<elasticity>` child element attributes into MjcfFlex.
pub(super) fn parse_flex_elasticity_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "young") {
        flex.young = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "poisson") {
        flex.poisson = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "damping") {
        flex.damping = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "thickness") {
        flex.thickness = s.parse().unwrap_or(-1.0);
    }
    // §42B S4: Parse bending_model (CortenForge extension attribute).
    if let Some(s) = get_attribute_opt(e, "bending_model") {
        flex.bending_model = match s.as_str() {
            "bridson" => sim_core::FlexBendingType::Bridson,
            _ => sim_core::FlexBendingType::Cotangent, // default for unknown or "cotangent"
        };
    }
}

/// Parse `<edge>` child element attributes into MjcfFlex.
/// These are passive spring-damper coefficients (not constraint params).
pub(super) fn parse_flex_edge_attrs(e: &BytesStart, flex: &mut MjcfFlex) {
    if let Some(s) = get_attribute_opt(e, "stiffness") {
        flex.edge_stiffness = s.parse().unwrap_or(0.0);
    }
    if let Some(s) = get_attribute_opt(e, "damping") {
        flex.edge_damping = s.parse().unwrap_or(0.0);
    }
}

/// Parse a `<flexcomp>` element (procedural mesh generation).
///
/// Generates vertices and elements based on the `type` attribute (grid, box, cylinder).
pub(super) fn parse_flexcomp<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfFlex> {
    let mut flex = parse_flex_attrs(start);

    let comp_type = get_attribute_opt(start, "type").unwrap_or_else(|| "grid".to_string());
    let count = parse_flexcomp_count(start);
    let spacing: f64 = get_attribute_opt(start, "spacing")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.02);

    match comp_type.as_str() {
        "grid" => generate_grid(&mut flex, count, spacing),
        "box" => generate_box_mesh(&mut flex, count, spacing),
        _ => {} // Unsupported type: leave empty
    }

    // Apply scale and rotation to generated vertices (DT-88)
    apply_flexcomp_transforms(&mut flex);

    // Parse child elements (<contact>, <elasticity>, <edge>, <pin>)
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"contact" => {
                        parse_flex_contact_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"elasticity" => {
                        parse_flex_elasticity_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"edge" => {
                        parse_flex_edge_attrs(e, &mut flex);
                        skip_element(reader, &elem_name)?;
                    }
                    b"pin" => {
                        if let Some(s) = get_attribute_opt(e, "id") {
                            let ids: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
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
                    b"contact" => parse_flex_contact_attrs(e, &mut flex),
                    b"elasticity" => parse_flex_elasticity_attrs(e, &mut flex),
                    b"edge" => parse_flex_edge_attrs(e, &mut flex),
                    b"pin" => {
                        if let Some(s) = get_attribute_opt(e, "id") {
                            let ids: Vec<usize> = s
                                .split_whitespace()
                                .filter_map(|t| t.parse().ok())
                                .collect();
                            flex.pinned.extend(ids);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"flexcomp" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in flexcomp".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(flex)
}

/// Parse a self-closing `<flexcomp ... />` (Event::Empty).
///
/// Does the same attribute parsing and mesh generation as `parse_flexcomp`
/// but without consuming reader events (since there's no End event to find).
pub(super) fn parse_flexcomp_empty(e: &BytesStart) -> MjcfFlex {
    let mut flex = parse_flex_attrs(e);

    let comp_type = get_attribute_opt(e, "type").unwrap_or_else(|| "grid".to_string());
    let count = parse_flexcomp_count(e);
    let spacing: f64 = get_attribute_opt(e, "spacing")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.02);

    match comp_type.as_str() {
        "grid" => generate_grid(&mut flex, count, spacing),
        "box" => generate_box_mesh(&mut flex, count, spacing),
        _ => {} // Unsupported type: leave empty
    }

    // Apply scale and rotation to generated vertices (DT-88)
    apply_flexcomp_transforms(&mut flex);

    flex
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
pub(super) fn parse_flexcomp_count(e: &BytesStart) -> [usize; 3] {
    get_attribute_opt(e, "count")
        .map(|s| {
            let vals: Vec<usize> = s
                .split_whitespace()
                .filter_map(|t| t.parse().ok())
                .collect();
            [
                vals.first().copied().unwrap_or(10),
                vals.get(1).copied().unwrap_or(10),
                vals.get(2).copied().unwrap_or(1),
            ]
        })
        .unwrap_or([10, 10, 1])
}

/// Generate a 2D grid mesh for flexcomp type="grid".
#[allow(clippy::cast_precision_loss)] // mesh dimensions never exceed 2^52
pub(super) fn generate_grid(flex: &mut MjcfFlex, count: [usize; 3], spacing: f64) {
    flex.dim = 2;
    let nx = count[0];
    let ny = count[1];

    // Generate vertices
    for iy in 0..ny {
        for ix in 0..nx {
            let x = ix as f64 * spacing;
            let y = iy as f64 * spacing;
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
pub(super) fn generate_box_mesh(flex: &mut MjcfFlex, count: [usize; 3], spacing: f64) {
    flex.dim = 3;
    let nx = count[0];
    let ny = count[1];
    let nz = count[2];

    // Generate vertices
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let x = ix as f64 * spacing;
                let y = iy as f64 * spacing;
                let z = iz as f64 * spacing;
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
    let mut skin = parse_skin_attrs(start);
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
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in skin".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(skin)
}

/// Parse skin element attributes.
pub(super) fn parse_skin_attrs(e: &BytesStart) -> MjcfSkin {
    let mut skin = MjcfSkin::default();

    skin.name = get_attribute_opt(e, "name").unwrap_or_default();
    skin.mesh = get_attribute_opt(e, "mesh");
    skin.material = get_attribute_opt(e, "material");

    if let Some(rgba) = get_attribute_opt(e, "rgba") {
        if let Ok(v) = parse_vector4(&rgba) {
            skin.rgba = v;
        }
    }

    if let Some(inflate) = parse_float_attr(e, "inflate") {
        skin.inflate = inflate;
    }

    // Parse embedded vertex positions if present
    if let Some(vertex) = get_attribute_opt(e, "vertex") {
        if let Ok(verts) = parse_float_array(&vertex) {
            skin.vertex_positions = Some(verts);
        }
    }

    // Parse embedded face data if present
    if let Some(face) = get_attribute_opt(e, "face") {
        if let Ok(faces) = parse_int_array(&face) {
            skin.faces = Some(faces);
        }
    }

    skin
}

/// Parse skin bone element attributes.
pub(super) fn parse_skin_bone_attrs(e: &BytesStart) -> Result<MjcfSkinBone> {
    let mut bone = MjcfSkinBone::default();

    // body is required
    bone.body =
        get_attribute_opt(e, "body").ok_or_else(|| MjcfError::missing_attribute("body", "bone"))?;

    if let Some(bindpos) = get_attribute_opt(e, "bindpos") {
        bone.bindpos = parse_vector3(&bindpos)?;
    }

    if let Some(bindquat) = get_attribute_opt(e, "bindquat") {
        bone.bindquat = parse_vector4(&bindquat)?;
    }

    Ok(bone)
}

/// Parse skin vertex element attributes.
pub(super) fn parse_skin_vertex_attrs(e: &BytesStart) -> Result<MjcfSkinVertex> {
    // id is required (vertex index in mesh)
    let id = parse_int_attr(e, "id").ok_or_else(|| MjcfError::missing_attribute("id", "vertex"))?
        as usize;

    // bone is required (bone index in skin)
    let bone = parse_int_attr(e, "bone")
        .ok_or_else(|| MjcfError::missing_attribute("bone", "vertex"))? as usize;

    // weight defaults to 1.0 if not specified
    let weight = parse_float_attr(e, "weight").unwrap_or(1.0);

    Ok(MjcfSkinVertex::new(id, bone, weight))
}
