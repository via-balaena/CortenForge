//! `<asset>` mesh and hfield parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfHfield, MjcfMesh};

use super::attrs::{Attrs, parse_maxhullvert, skip_element};

/// Parse asset element (contains mesh, texture, material definitions;
/// `empty`: it closed itself, so it has none).
pub(super) fn parse_asset<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<(Vec<MjcfMesh>, Vec<MjcfHfield>)> {
    let mut meshes = Vec::new();
    let mut hfields = Vec::new();
    if empty {
        return Ok((meshes, hfields));
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"mesh" => meshes.push(parse_mesh_attrs(e)?),
                    b"hfield" => hfields.push(parse_hfield_attrs(e)?),
                    // Skip other asset types (texture, material, etc.)
                    _ => {}
                }
                // A mesh's or hfield's children are not read either.
                skip_element(reader, e.name().as_ref(), empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"asset" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("asset")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok((meshes, hfields))
}

/// Parse mesh attributes only.
pub(super) fn parse_mesh_attrs(e: &BytesStart) -> Result<MjcfMesh> {
    let a = Attrs::new(e)?;
    let mut mesh = MjcfMesh::default();

    mesh.name = a.string("name").unwrap_or_default();
    mesh.file = a.string("file");

    mesh.scale = a.vec3("scale")?;

    // Parse embedded vertex data
    mesh.vertex = a.vector("vertex")?;

    // Parse embedded face data: vertex indices, integers
    if let Some(face) = a.int_vector("face")? {
        let indices = face
            .iter()
            .map(|&i| u32::try_from(i))
            .collect::<std::result::Result<Vec<u32>, _>>()
            .map_err(|_| MjcfError::invalid_value("mesh", "face", "a vertex index is negative"))?;
        mesh.face = Some(indices);
    }

    // Parse maxhullvert (maximum convex hull vertices)
    if let Some(n) = a.int("maxhullvert")? {
        mesh.maxhullvert = parse_maxhullvert(n)?;
    }

    // Parse mesh inertia mode
    mesh.inertia = a.keyword("inertia", keywords::mesh_inertia)?;

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
    let a = Attrs::new(e)?;
    // Unwritten, the name is the file's (the builder's `name_assets`).
    let name = a.string("name").unwrap_or_default();

    let file = a.string("file");

    // Parse nrow/ncol (optional when file is present, required otherwise)
    let count = |attr: &str| -> Result<Option<usize>> {
        a.int(attr)?
            .map(|n| {
                usize::try_from(n).map_err(|_| {
                    MjcfError::invalid_value("hfield", attr, "must be a positive integer")
                })
            })
            .transpose()
    };
    let nrow = count("nrow")?;
    let ncol = count("ncol")?;

    // Parse size (always required)
    let size = a
        .array::<4>("size")?
        .ok_or_else(|| MjcfError::missing_attribute("hfield", "size"))?;
    if size[0] <= 0.0 || size[1] <= 0.0 {
        return Err(MjcfError::invalid_value(
            "hfield",
            "size",
            "size[0] and size[1] (half-extents) must be > 0",
        ));
    }
    if size[2] < 0.0 || size[3] < 0.0 {
        return Err(MjcfError::invalid_value(
            "hfield",
            "size",
            "size[2] (z_top) and size[3] (z_bottom) must be >= 0",
        ));
    }

    // Parse elevation (optional when file is present, required otherwise)
    let elevation = a.vector("elevation")?;

    // Validation: at least one data source must be present
    if file.is_none() && elevation.is_none() {
        return Err(MjcfError::invalid_value(
            "hfield",
            "elevation",
            "at least one of 'file' or 'elevation' must be specified",
        ));
    }

    // Validate inline data completeness
    if let Some(ref elev) = elevation {
        let nr = nrow.ok_or_else(|| MjcfError::missing_attribute("hfield", "nrow"))?;
        let nc = ncol.ok_or_else(|| MjcfError::missing_attribute("hfield", "ncol"))?;
        if nr < 2 || nc < 2 {
            return Err(MjcfError::invalid_value(
                "hfield",
                "nrow",
                format!("nrow ({nr}) and ncol ({nc}) must be >= 2"),
            ));
        }
        if elev.len() != nr * nc {
            return Err(MjcfError::invalid_value(
                "hfield",
                "elevation",
                format!(
                    "elevation length ({}) must equal nrow * ncol ({})",
                    elev.len(),
                    nr * nc
                ),
            ));
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
