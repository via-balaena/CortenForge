//! `<asset>` mesh and hfield parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MeshInertia, MjcfHfield, MjcfMesh};

use super::attrs::{
    attr_vec3, get_attribute_opt, parse_float_array, parse_int_attr, parse_maxhullvert,
    skip_element,
};

/// Parse asset element (contains mesh, texture, material definitions).
pub(super) fn parse_asset<R: BufRead>(
    reader: &mut Reader<R>,
) -> Result<(Vec<MjcfMesh>, Vec<MjcfHfield>)> {
    let mut meshes = Vec::new();
    let mut hfields = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"mesh" => {
                        let mesh = parse_mesh(reader, e)?;
                        meshes.push(mesh);
                    }
                    b"hfield" => {
                        let hfield = parse_hfield_attrs(e)?;
                        // <hfield> has no child elements — skip to closing tag
                        skip_element(reader, &elem_name)?;
                        hfields.push(hfield);
                    }
                    // Skip other asset types (texture, material, etc.)
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                if e.name().as_ref() == b"mesh" {
                    let mesh = parse_mesh_attrs(e)?;
                    meshes.push(mesh);
                } else if e.name().as_ref() == b"hfield" {
                    let hfield = parse_hfield_attrs(e)?;
                    hfields.push(hfield);
                }
                // Skip other self-closing asset types
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"asset" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in asset".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok((meshes, hfields))
}

/// Parse mesh element.
pub(super) fn parse_mesh<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfMesh> {
    let mesh = parse_mesh_attrs(start)?;
    let mut buf = Vec::new();

    // Meshes typically don't have children, but we still need to read to the end
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::End(ref e)) if e.name().as_ref() == b"mesh" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in mesh".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(mesh)
}

/// Parse mesh attributes only.
pub(super) fn parse_mesh_attrs(e: &BytesStart) -> Result<MjcfMesh> {
    let mut mesh = MjcfMesh::default();

    mesh.name = get_attribute_opt(e, "name").unwrap_or_default();
    mesh.file = get_attribute_opt(e, "file");

    mesh.scale = attr_vec3(e, "scale")?;

    // Parse embedded vertex data
    if let Some(vertex) = get_attribute_opt(e, "vertex") {
        mesh.vertex = Some(parse_float_array(&vertex)?);
    }

    // Parse embedded face data
    if let Some(face) = get_attribute_opt(e, "face") {
        let face_data = parse_float_array(&face)?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let face_indices: Vec<u32> = face_data.iter().map(|f| *f as u32).collect();
        mesh.face = Some(face_indices);
    }

    // Parse maxhullvert (maximum convex hull vertices)
    if let Some(n) = parse_int_attr(e, "maxhullvert") {
        mesh.maxhullvert = parse_maxhullvert(n)?;
    }

    // Parse mesh inertia mode
    if let Some(inertia_str) = get_attribute_opt(e, "inertia") {
        mesh.inertia = Some(match inertia_str.as_str() {
            "convex" => MeshInertia::Convex,
            "exact" => MeshInertia::Exact,
            "legacy" => MeshInertia::Legacy,
            "shell" => MeshInertia::Shell,
            other => {
                return Err(MjcfError::XmlParse(format!(
                    "mesh '{}': invalid inertia mode '{}' \
                     (expected convex, exact, legacy, or shell)",
                    mesh.name, other
                )));
            }
        });
    }

    Ok(mesh)
}

/// Parse hfield attributes from a `<hfield>` element.
///
/// Supports two data sources:
/// - **File-based:** `file="terrain.png"` — `nrow`/`ncol`/`elevation` derived from PNG at build time.
/// - **Inline:** `nrow`, `ncol`, `elevation` attributes in the XML.
///
/// At least one of `file` or `elevation` must be present.
pub(super) fn parse_hfield_attrs(e: &BytesStart) -> Result<MjcfHfield> {
    let name = get_attribute_opt(e, "name").ok_or_else(|| {
        MjcfError::XmlParse("hfield element missing required 'name' attribute".into())
    })?;

    let file = get_attribute_opt(e, "file");

    // Parse nrow/ncol (optional when file is present, required otherwise)
    let nrow: Option<usize> =
        if let Some(s) = get_attribute_opt(e, "nrow") {
            Some(s.parse().map_err(|_| {
                MjcfError::XmlParse("hfield 'nrow' must be a positive integer".into())
            })?)
        } else {
            None
        };

    let ncol: Option<usize> =
        if let Some(s) = get_attribute_opt(e, "ncol") {
            Some(s.parse().map_err(|_| {
                MjcfError::XmlParse("hfield 'ncol' must be a positive integer".into())
            })?)
        } else {
            None
        };

    // Parse size (always required)
    let size_str = get_attribute_opt(e, "size").ok_or_else(|| {
        MjcfError::XmlParse("hfield element missing required 'size' attribute".into())
    })?;
    let size_vec = parse_float_array(&size_str)?;
    if size_vec.len() != 4 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{}': size requires exactly 4 values, got {}",
            name,
            size_vec.len(),
        )));
    }
    let size = [size_vec[0], size_vec[1], size_vec[2], size_vec[3]];
    if size[0] <= 0.0 || size[1] <= 0.0 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': size[0] and size[1] (half-extents) must be > 0",
        )));
    }
    if size[2] < 0.0 || size[3] < 0.0 {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': size[2] (z_top) and size[3] (z_bottom) must be >= 0",
        )));
    }

    // Parse elevation (optional when file is present, required otherwise)
    let elevation = if let Some(elevation_str) = get_attribute_opt(e, "elevation") {
        Some(parse_float_array(&elevation_str)?)
    } else {
        None
    };

    // Validation: at least one data source must be present
    if file.is_none() && elevation.is_none() {
        return Err(MjcfError::XmlParse(format!(
            "hfield '{name}': at least one of 'file' or 'elevation' must be specified",
        )));
    }

    // Validate inline data completeness
    if let Some(ref elev) = elevation {
        let nr = nrow.ok_or_else(|| {
            MjcfError::XmlParse(format!(
                "hfield '{name}': 'nrow' is required when 'elevation' is specified",
            ))
        })?;
        let nc = ncol.ok_or_else(|| {
            MjcfError::XmlParse(format!(
                "hfield '{name}': 'ncol' is required when 'elevation' is specified",
            ))
        })?;
        if nr < 2 || nc < 2 {
            return Err(MjcfError::XmlParse(format!(
                "hfield '{name}': nrow ({nr}) and ncol ({nc}) must be >= 2",
            )));
        }
        if elev.len() != nr * nc {
            return Err(MjcfError::XmlParse(format!(
                "hfield '{}': elevation length ({}) must equal nrow * ncol ({})",
                name,
                elev.len(),
                nr * nc,
            )));
        }
    }

    Ok(MjcfHfield {
        name,
        size,
        nrow,
        ncol,
        elevation,
        file,
    })
}
