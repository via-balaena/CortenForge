//! `<asset>` mesh and hfield parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfHfield, MjcfMesh};

use super::attrs::{Attrs, parse_maxhullvert, skip_element};
use super::mesh_builtin::{self, MeshBuiltin};

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
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    // A mesh's children are not read, but whether a
                    // `<plugin>` is among them.
                    b"mesh" => {
                        let mut mesh = parse_mesh_attrs(e)?;
                        mesh.plugin = skip_noting_plugin(reader, child_empty)?;
                        meshes.push(mesh);
                    }
                    // A height field's children are not read either.
                    b"hfield" => {
                        hfields.push(parse_hfield_attrs(e)?);
                        skip_element(reader, b"hfield", child_empty)?;
                    }
                    // Skip other asset types (texture, material, etc.)
                    other => {
                        let other = other.to_vec();
                        skip_element(reader, &other, child_empty)?;
                    }
                }
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

/// Past a `<mesh>`'s end tag (nothing more when it closed itself), whether a
/// `<plugin>` is among its children.
fn skip_noting_plugin<R: BufRead>(reader: &mut Reader<R>, empty: bool) -> Result<bool> {
    if empty {
        return Ok(false);
    }
    let mut buf = Vec::new();
    let mut plugin = false;
    let mut depth = 1;
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                plugin |= depth == 1 && e.name().as_ref() == b"plugin";
                depth += 1;
            }
            Ok(Event::Empty(ref e)) => plugin |= depth == 1 && e.name().as_ref() == b"plugin",
            Ok(Event::End(_)) => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            Ok(Event::Eof) => return Err(MjcfError::eof_in("mesh")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(plugin)
}

/// Parse mesh attributes only.
pub(super) fn parse_mesh_attrs(e: &BytesStart) -> Result<MjcfMesh> {
    let a = Attrs::new(e)?;
    let mut mesh = MjcfMesh::default();

    mesh.name = a.string("name").unwrap_or_default();
    mesh.class = a.string("class");
    mesh.file = a.string("file");
    mesh.content_type = a.string("content_type");

    // In MuJoCo 3.5.0's `OneMesh` order (`xml_native_reader.cc:1568-1619`),
    // so of two attributes that do not read, the one it names is named:
    // the transform, the inertia mode, `maxhullvert`, then the written data.
    if let Some(refpos) = a.vec3("refpos")? {
        mesh.refpos = refpos;
    }
    if let Some(refquat) = a.array::<4>("refquat")? {
        mesh.refquat = refquat;
    }
    mesh.scale = a.vec3("scale")?;
    mesh.inertia = a.keyword("inertia", keywords::mesh_inertia)?;
    if let Some(n) = a.int("maxhullvert")? {
        mesh.maxhullvert = Some(parse_maxhullvert(n)?);
    }

    // The written vertices, normals, texture coordinates and faces, each
    // checked when the mesh is built: a face's vertex index, negative or
    // not, then too (`CheckInitialMesh`, `user_mesh.cc:1835-1841`).
    mesh.vertex = a.vector("vertex")?;
    mesh.normal = a.vector("normal")?;
    mesh.texcoord = a.vector("texcoord")?;
    let face = a.int_vector("face")?;

    let made_faces = match a.keyword("builtin", MeshBuiltin::from_keyword)? {
        Some(builtin) => make_builtin(&mut mesh, builtin, &a)?,
        None => false,
    };
    if !made_faces {
        mesh.face = face;
    }

    Ok(mesh)
}

/// A built-in mesh made into `mesh` as MuJoCo 3.5.0's reader has
/// `mjs_makeMesh` make it
/// (`mjXReader::OneMesh`, `xml_native_reader.cc:1621-1634`): `params`
/// required, refused beside a `file` or written vertices, then the
/// generator's vertices, its faces where it makes them, and the plate's
/// shell inertia, in place of what was written ([`mesh_builtin`]). Whether
/// it made the faces.
fn make_builtin(mesh: &mut MjcfMesh, builtin: MeshBuiltin, a: &Attrs) -> Result<bool> {
    let params = a
        .vector("params")?
        .ok_or_else(|| MjcfError::missing_attribute("mesh", "params"))?;
    if mesh.file.is_some() {
        return Err(MjcfError::invalid_value(
            "mesh",
            "builtin",
            "builtin cannot be used with a mesh file",
        ));
    }
    if mesh.vertex.is_some() {
        return Err(MjcfError::invalid_value(
            "mesh",
            "builtin",
            "builtin mesh cannot be used with user vertex data",
        ));
    }
    let made = mesh_builtin::make_mesh(builtin, &params)
        .map_err(|reason| MjcfError::invalid_value("mesh", "params", reason))?;
    // An empty list is no data, as an empty `vertex` or `face` reads.
    mesh.vertex =
        (!made.vertex.is_empty()).then(|| made.vertex.into_iter().map(f64::from).collect());
    let made_faces = made.face.is_some();
    if let Some(face) = made.face {
        // MuJoCo's `userface` holds ints; the generator's size check keeps
        // each id below `i32::MAX`.
        #[allow(clippy::cast_possible_wrap)]
        let face: Vec<i32> = face.into_iter().map(|id| id as i32).collect();
        mesh.face = (!face.is_empty()).then_some(face);
    }
    if made.inertia.is_some() {
        mesh.inertia = made.inertia;
    }
    Ok(made_faces)
}

/// A `<hfield>`'s attributes as MuJoCo 3.5.0's reader takes them
/// (`xml_native_reader.cc:3403-3458`): its `name`, `content_type`, `file`,
/// `nrow`, `ncol` and `size`, and its `elevation` only where it names no file
/// and both counts are positive, refused there where it holds other than
/// `nrow · ncol` numbers ("elevation data length must match nrow*ncol"). The
/// builder compiles its data (`compile_hfield`), refusing one with none
/// ("hfield not specified"), and its sizes, as MuJoCo's compiler does.
pub(super) fn parse_hfield_attrs(e: &BytesStart) -> Result<MjcfHfield> {
    let a = Attrs::new(e)?;
    // Unwritten, the name is the file's (the builder's `name_assets`).
    let name = a.string("name").unwrap_or_default();
    let content_type = a.string("content_type");
    let file = a.string("file");
    let nrow = a.int("nrow")?.unwrap_or(0);
    let ncol = a.int("ncol")?.unwrap_or(0);
    let size = a
        .array::<4>("size")?
        .ok_or_else(|| MjcfError::missing_attribute("hfield", "size"))?;
    let elevation = if file.is_none() && nrow > 0 && ncol > 0 {
        let elevation = a.vector("elevation")?;
        let cells = usize::try_from(i64::from(nrow) * i64::from(ncol)).ok();
        if elevation.as_ref().is_some_and(|e| Some(e.len()) != cells) {
            return Err(MjcfError::invalid_value(
                "hfield",
                "elevation",
                "elevation data length must match nrow*ncol",
            ));
        }
        elevation
    } else {
        None
    };
    Ok(MjcfHfield {
        name,
        size,
        nrow,
        ncol,
        elevation,
        file,
        content_type,
    })
}
