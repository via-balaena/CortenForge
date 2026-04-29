//! PLY (Polygon File Format) support.
//!
//! PLY is a flexible format for storing 3D data, supporting both ASCII and binary variants.
//! It's commonly used in 3D scanning and computer graphics.
//!
//! # Supported Properties
//!
//! - Vertex positions (x, y, z) - required
//! - Face vertex indices (`vertex_indices` or `vertex_index`) - required for meshes
//!
//! # Format Variants
//!
//! - **ASCII** - Human-readable, larger files
//! - **Binary Little Endian** - Compact, fast to read/write
//! - **Binary Big Endian** - Compact, for big-endian systems
//!
//! # Example
//!
//! ```no_run
//! use mesh_io::{load_ply, save_ply};
//!
//! let mesh = load_ply("model.ply").unwrap();
//! save_ply(&mesh, "output.ply", true).unwrap(); // Binary
//! ```

// PLY parsing converts between u32 mesh indices, i32 PLY header fields, and f32 coords;
// cast lints fire for legitimate domain conversions that are bounded by spec/format.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::path::Path;

use mesh_types::{AttributedMesh, IndexedMesh, Point3, Vector3, VertexColor};
use ply_rs::parser::Parser;
use ply_rs::ply::{
    Addable, DefaultElement, ElementDef, Encoding, Ply, Property, PropertyDef, PropertyType,
    ScalarType,
};
use ply_rs::writer::Writer;

use crate::error::{IoError, IoResult};

/// Load a mesh from a PLY file.
///
/// Supports ASCII, binary little-endian, and binary big-endian formats.
///
/// # Arguments
///
/// * `path` - Path to the PLY file
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be read
/// - The file is not valid PLY format
/// - Required properties (x, y, z for vertices) are missing
///
/// # Example
///
/// ```no_run
/// use mesh_io::load_ply;
///
/// let mesh = load_ply("model.ply").unwrap();
/// println!("Loaded {} vertices, {} faces", mesh.vertices.len(), mesh.faces.len());
/// ```
pub fn load_ply<P: AsRef<Path>>(path: P) -> IoResult<IndexedMesh> {
    let path = path.as_ref();
    let file = File::open(path).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            IoError::FileNotFound {
                path: path.to_path_buf(),
            }
        } else {
            IoError::Io(e)
        }
    })?;
    let mut reader = BufReader::new(file);

    // Use generic DefaultElement parser - works for all element types
    let parser = Parser::<DefaultElement>::new();

    // Parse header
    let header = parser
        .read_header(&mut reader)
        .map_err(|e| IoError::invalid_content(format!("failed to parse PLY header: {e}")))?;

    // Read the entire payload
    let payload = parser
        .read_payload(&mut reader, &header)
        .map_err(|e| IoError::invalid_content(format!("failed to read PLY payload: {e}")))?;

    // Extract vertices
    let mut mesh = IndexedMesh::new();

    if let Some(vertex_elements) = payload.get("vertex") {
        mesh.vertices.reserve(vertex_elements.len());
        for element in vertex_elements {
            let x = get_float_property(element, "x").unwrap_or(0.0);
            let y = get_float_property(element, "y").unwrap_or(0.0);
            let z = get_float_property(element, "z").unwrap_or(0.0);
            mesh.vertices
                .push(Point3::new(f64::from(x), f64::from(y), f64::from(z)));
        }
    }

    // Extract faces
    if let Some(face_elements) = payload.get("face") {
        mesh.faces.reserve(face_elements.len());
        for element in face_elements {
            let indices = get_index_list(element);
            if indices.len() >= 3 {
                // Triangulate if necessary (fan triangulation for convex polygons)
                for i in 1..indices.len() - 1 {
                    mesh.faces
                        .push([indices[0] as u32, indices[i] as u32, indices[i + 1] as u32]);
                }
            }
        }
    }

    Ok(mesh)
}

/// Extract a float property from a PLY element.
fn get_float_property(element: &DefaultElement, key: &str) -> Option<f32> {
    match element.get(key)? {
        Property::Float(v) => Some(*v),
        Property::Double(v) => Some(*v as f32),
        _ => None,
    }
}

/// Extract vertex index list from a face element.
fn get_index_list(element: &DefaultElement) -> Vec<usize> {
    // Try common property names for face indices
    for key in &["vertex_indices", "vertex_index"] {
        if let Some(prop) = element.get(*key) {
            return match prop {
                Property::ListInt(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListUInt(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListUChar(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListChar(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListShort(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListUShort(v) => v.iter().map(|&i| i as usize).collect(),
                _ => continue,
            };
        }
    }
    Vec::new()
}

/// Save a mesh to a PLY file.
///
/// # Arguments
///
/// * `mesh` - The mesh to save
/// * `path` - Output file path
/// * `binary` - If true, save as binary little-endian; if false, save as ASCII
///
/// # Errors
///
/// Returns an error if the file cannot be written.
///
/// # Example
///
/// ```no_run
/// use mesh_io::{load_ply, save_ply};
///
/// let mesh = load_ply("input.ply").unwrap();
/// save_ply(&mesh, "output.ply", true).unwrap(); // Binary
/// save_ply(&mesh, "output_ascii.ply", false).unwrap(); // ASCII
/// ```
pub fn save_ply<P: AsRef<Path>>(mesh: &IndexedMesh, path: P, binary: bool) -> IoResult<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    if binary {
        save_ply_binary(mesh, &mut writer)
    } else {
        save_ply_ascii(mesh, &mut writer)
    }
}

/// Save mesh as binary PLY (little-endian).
///
/// Note: We implement this manually because ply-rs has a bug with binary list
/// property writing where it uses element count instead of list length.
fn save_ply_binary<W: std::io::Write>(mesh: &IndexedMesh, writer: &mut W) -> IoResult<()> {
    // Write header
    writeln!(writer, "ply")?;
    writeln!(writer, "format binary_little_endian 1.0")?;
    writeln!(writer, "comment Generated by CortenForge mesh-io")?;
    writeln!(writer, "element vertex {}", mesh.vertices.len())?;
    writeln!(writer, "property float x")?;
    writeln!(writer, "property float y")?;
    writeln!(writer, "property float z")?;
    writeln!(writer, "element face {}", mesh.faces.len())?;
    writeln!(writer, "property list uchar int vertex_indices")?;
    writeln!(writer, "end_header")?;

    // Write vertex data
    for v in &mesh.vertices {
        {
            writer.write_all(&(v.x as f32).to_le_bytes())?;
            writer.write_all(&(v.y as f32).to_le_bytes())?;
            writer.write_all(&(v.z as f32).to_le_bytes())?;
        }
    }

    // Write face data
    for &[i0, i1, i2] in &mesh.faces {
        // List count (3 vertices per face)
        writer.write_all(&[3u8])?;
        // Vertex indices as i32
        {
            writer.write_all(&(i0 as i32).to_le_bytes())?;
            writer.write_all(&(i1 as i32).to_le_bytes())?;
            writer.write_all(&(i2 as i32).to_le_bytes())?;
        }
    }

    Ok(())
}

/// Save mesh as ASCII PLY using ply-rs.
fn save_ply_ascii<W: std::io::Write>(mesh: &IndexedMesh, writer: &mut W) -> IoResult<()> {
    // Build PLY structure
    let mut ply = Ply::<DefaultElement>::new();
    ply.header.encoding = Encoding::Ascii;
    ply.header
        .comments
        .push("Generated by CortenForge mesh-io".to_string());

    // Define vertex element
    let mut vertex_def = ElementDef::new("vertex".to_string());
    vertex_def.properties.add(PropertyDef::new(
        "x".to_string(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_def.properties.add(PropertyDef::new(
        "y".to_string(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_def.properties.add(PropertyDef::new(
        "z".to_string(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_def.count = mesh.vertices.len();
    ply.header.elements.add(vertex_def);

    // Define face element
    let mut face_def = ElementDef::new("face".to_string());
    face_def.properties.add(PropertyDef::new(
        "vertex_indices".to_string(),
        PropertyType::List(ScalarType::UChar, ScalarType::Int),
    ));
    face_def.count = mesh.faces.len();
    ply.header.elements.add(face_def);

    // Add vertex data
    let mut vertex_elements = Vec::with_capacity(mesh.vertices.len());
    for v in &mesh.vertices {
        let mut element = DefaultElement::new();
        {
            element.insert("x".to_string(), Property::Float(v.x as f32));
            element.insert("y".to_string(), Property::Float(v.y as f32));
            element.insert("z".to_string(), Property::Float(v.z as f32));
        }
        vertex_elements.push(element);
    }
    ply.payload.insert("vertex".to_string(), vertex_elements);

    // Add face data
    let mut face_elements = Vec::with_capacity(mesh.faces.len());
    for &[i0, i1, i2] in &mesh.faces {
        let mut element = DefaultElement::new();
        let indices = vec![i0 as i32, i1 as i32, i2 as i32];
        element.insert("vertex_indices".to_string(), Property::ListInt(indices));
        face_elements.push(element);
    }
    ply.payload.insert("face".to_string(), face_elements);

    // Write
    let ply_writer = Writer::new();
    ply_writer
        .write_ply(writer, &mut ply)
        .map_err(|e| IoError::invalid_content(format!("failed to write PLY: {e}")))?;

    Ok(())
}

// ---------------------------------------------------------------------------
// Attributed PLY: round-trip AttributedMesh with normals, colors, zone_ids,
// clearances, offsets, uvs, and the open-ended `extras` map. Property names
// follow PLY conventions so files load cleanly in ParaView, MeshLab, Blender.
// ---------------------------------------------------------------------------

/// PLY-format keywords that cannot appear as property names.
const PLY_KEYWORDS: &[&str] = &[
    "ply",
    "format",
    "comment",
    "element",
    "property",
    "end_header",
    "obj_info",
];

/// If `name` collides with a canonical [`AttributedMesh`] slot or PLY keyword,
/// returns a static reason string explaining the rejection. Otherwise `None`.
fn reserved_reason(name: &str) -> Option<&'static str> {
    match name {
        "x" | "y" | "z" => Some("name reserved by PLY for vertex positions"),
        "nx" | "ny" | "nz" => Some("name reserved by PLY for vertex normals"),
        "red" | "green" | "blue" | "alpha" => Some("name reserved by PLY for vertex colors"),
        "zone_id" => Some("name reserved for AttributedMesh zone_ids"),
        "clearance" => Some("name reserved for AttributedMesh clearances"),
        "offset" => Some("name reserved for AttributedMesh offsets"),
        "s" | "t" => Some("name reserved by PLY for vertex texture coordinates"),
        n if PLY_KEYWORDS.contains(&n) => Some("PLY-format keyword cannot be used as a name"),
        _ => None,
    }
}

/// Validate a single `extras` key for use as a PLY property name.
fn validate_extra_name(name: &str) -> IoResult<()> {
    if name.is_empty() {
        return Err(IoError::InvalidAttributeName {
            name: name.to_string(),
            reason: "PLY property names must not be empty",
        });
    }
    if name.bytes().any(|b| b.is_ascii_whitespace()) {
        return Err(IoError::InvalidAttributeName {
            name: name.to_string(),
            reason: "PLY property names must not contain whitespace",
        });
    }
    if name.bytes().any(|b| !b.is_ascii() || b.is_ascii_control()) {
        return Err(IoError::InvalidAttributeName {
            name: name.to_string(),
            reason: "PLY property names must be printable ASCII",
        });
    }
    if let Some(reason) = reserved_reason(name) {
        return Err(IoError::InvalidAttributeName {
            name: name.to_string(),
            reason,
        });
    }
    Ok(())
}

/// Verify that a populated slot length matches the mesh's vertex count.
fn check_slot_length(slot: &str, len: usize, expected: usize) -> IoResult<()> {
    if len == expected {
        Ok(())
    } else {
        Err(IoError::invalid_content(format!(
            "AttributedMesh slot `{slot}` has {len} entries but mesh has {expected} vertices"
        )))
    }
}

/// Verify that every populated optional slot has length equal to the
/// vertex count. [`AttributedMesh`] slots are public fields, so direct
/// mutation can violate the invariant; we re-check at the I/O boundary.
fn validate_slot_lengths(mesh: &AttributedMesh) -> IoResult<()> {
    let n = mesh.vertex_count();
    let check = check_slot_length;
    if let Some(v) = mesh.normals.as_ref() {
        check("normals", v.len(), n)?;
    }
    if let Some(v) = mesh.colors.as_ref() {
        check("colors", v.len(), n)?;
    }
    if let Some(v) = mesh.zone_ids.as_ref() {
        check("zone_ids", v.len(), n)?;
    }
    if let Some(v) = mesh.clearances.as_ref() {
        check("clearances", v.len(), n)?;
    }
    if let Some(v) = mesh.offsets.as_ref() {
        check("offsets", v.len(), n)?;
    }
    if let Some(v) = mesh.uvs.as_ref() {
        check("uvs", v.len(), n)?;
    }
    for (k, v) in &mesh.extras {
        check(&format!("extras[{k}]"), v.len(), n)?;
    }
    Ok(())
}

/// Save an [`AttributedMesh`] to PLY, preserving normals, colors, zone IDs,
/// clearances, offsets, UVs, and arbitrary per-vertex `extras`.
///
/// Property names follow PLY conventions so the file loads cleanly in
/// `ParaView`, `MeshLab`, and Blender:
///
/// | Slot | PLY properties | Type |
/// |---|---|---|
/// | `geometry.vertices` | `x, y, z` | `float` |
/// | `normals` | `nx, ny, nz` | `float` |
/// | `colors` | `red, green, blue` | `uchar` |
/// | `zone_ids` | `zone_id` | `uint` |
/// | `clearances` | `clearance` | `float` |
/// | `offsets` | `offset` | `float` |
/// | `uvs` | `s, t` | `float` |
/// | `extras["k"]` | `k` | `float` |
///
/// `extras` keys are written in `BTreeMap` iteration order (deterministic).
///
/// # Arguments
///
/// * `mesh` - The mesh to save
/// * `path` - Output file path
/// * `binary` - If true, save as binary little-endian; if false, save as ASCII
///
/// # Errors
///
/// Returns an error if:
/// - An `extras` key collides with a reserved property name (e.g. `nx`),
///   contains whitespace, is empty, or is a PLY-format keyword.
/// - A populated optional slot has a length other than `vertex_count()`.
/// - The output file cannot be written.
///
/// On validation failure, no file is created.
///
/// # Example
///
/// ```no_run
/// use mesh_io::save_ply_attributed;
/// use mesh_types::{AttributedMesh, unit_cube};
///
/// let mut mesh = AttributedMesh::new(unit_cube());
/// mesh.compute_normals();
/// mesh.insert_extra("stress", vec![0.0_f32; 8]).unwrap();
/// save_ply_attributed(&mesh, "out.ply", true).unwrap();
/// ```
pub fn save_ply_attributed<P: AsRef<Path>>(
    mesh: &AttributedMesh,
    path: P,
    binary: bool,
) -> IoResult<()> {
    for name in mesh.extras.keys() {
        validate_extra_name(name)?;
    }
    validate_slot_lengths(mesh)?;

    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    write_attributed_header(&mut writer, mesh, binary)?;
    if binary {
        write_attributed_body_binary(&mut writer, mesh)?;
    } else {
        write_attributed_body_ascii(&mut writer, mesh)?;
    }
    Ok(())
}

/// Write the PLY header for an attributed mesh, declaring all populated slots.
fn write_attributed_header<W: Write>(
    writer: &mut W,
    mesh: &AttributedMesh,
    binary: bool,
) -> IoResult<()> {
    writeln!(writer, "ply")?;
    writeln!(
        writer,
        "format {} 1.0",
        if binary {
            "binary_little_endian"
        } else {
            "ascii"
        }
    )?;
    writeln!(
        writer,
        "comment Generated by CortenForge mesh-io (attributed)"
    )?;
    writeln!(writer, "element vertex {}", mesh.vertex_count())?;
    writeln!(writer, "property float x")?;
    writeln!(writer, "property float y")?;
    writeln!(writer, "property float z")?;
    if mesh.normals.is_some() {
        writeln!(writer, "property float nx")?;
        writeln!(writer, "property float ny")?;
        writeln!(writer, "property float nz")?;
    }
    if mesh.colors.is_some() {
        writeln!(writer, "property uchar red")?;
        writeln!(writer, "property uchar green")?;
        writeln!(writer, "property uchar blue")?;
    }
    if mesh.zone_ids.is_some() {
        writeln!(writer, "property uint zone_id")?;
    }
    if mesh.clearances.is_some() {
        writeln!(writer, "property float clearance")?;
    }
    if mesh.offsets.is_some() {
        writeln!(writer, "property float offset")?;
    }
    if mesh.uvs.is_some() {
        writeln!(writer, "property float s")?;
        writeln!(writer, "property float t")?;
    }
    for name in mesh.extras.keys() {
        writeln!(writer, "property float {name}")?;
    }
    writeln!(writer, "element face {}", mesh.face_count())?;
    writeln!(writer, "property list uchar int vertex_indices")?;
    writeln!(writer, "end_header")?;
    Ok(())
}

/// Write the binary-LE body for an attributed mesh.
fn write_attributed_body_binary<W: Write>(writer: &mut W, mesh: &AttributedMesh) -> IoResult<()> {
    let n = mesh.vertex_count();
    for i in 0..n {
        let v = &mesh.geometry.vertices[i];
        writer.write_all(&(v.x as f32).to_le_bytes())?;
        writer.write_all(&(v.y as f32).to_le_bytes())?;
        writer.write_all(&(v.z as f32).to_le_bytes())?;
        if let Some(ns) = mesh.normals.as_ref() {
            let nrm = &ns[i];
            writer.write_all(&(nrm.x as f32).to_le_bytes())?;
            writer.write_all(&(nrm.y as f32).to_le_bytes())?;
            writer.write_all(&(nrm.z as f32).to_le_bytes())?;
        }
        if let Some(cs) = mesh.colors.as_ref() {
            let c = &cs[i];
            writer.write_all(&[c.r, c.g, c.b])?;
        }
        if let Some(zs) = mesh.zone_ids.as_ref() {
            writer.write_all(&zs[i].to_le_bytes())?;
        }
        if let Some(cs) = mesh.clearances.as_ref() {
            writer.write_all(&cs[i].to_le_bytes())?;
        }
        if let Some(os) = mesh.offsets.as_ref() {
            writer.write_all(&os[i].to_le_bytes())?;
        }
        if let Some(uv) = mesh.uvs.as_ref() {
            let (u, t) = uv[i];
            writer.write_all(&u.to_le_bytes())?;
            writer.write_all(&t.to_le_bytes())?;
        }
        for v in mesh.extras.values() {
            writer.write_all(&v[i].to_le_bytes())?;
        }
    }
    for &[i0, i1, i2] in &mesh.geometry.faces {
        writer.write_all(&[3u8])?;
        writer.write_all(&(i0 as i32).to_le_bytes())?;
        writer.write_all(&(i1 as i32).to_le_bytes())?;
        writer.write_all(&(i2 as i32).to_le_bytes())?;
    }
    Ok(())
}

/// Write the ASCII body for an attributed mesh, one element per line.
fn write_attributed_body_ascii<W: Write>(writer: &mut W, mesh: &AttributedMesh) -> IoResult<()> {
    let n = mesh.vertex_count();
    for i in 0..n {
        let v = &mesh.geometry.vertices[i];
        write!(writer, "{} {} {}", v.x as f32, v.y as f32, v.z as f32)?;
        if let Some(ns) = mesh.normals.as_ref() {
            let nrm = &ns[i];
            write!(
                writer,
                " {} {} {}",
                nrm.x as f32, nrm.y as f32, nrm.z as f32
            )?;
        }
        if let Some(cs) = mesh.colors.as_ref() {
            let c = &cs[i];
            write!(writer, " {} {} {}", c.r, c.g, c.b)?;
        }
        if let Some(zs) = mesh.zone_ids.as_ref() {
            write!(writer, " {}", zs[i])?;
        }
        if let Some(cs) = mesh.clearances.as_ref() {
            write!(writer, " {}", cs[i])?;
        }
        if let Some(os) = mesh.offsets.as_ref() {
            write!(writer, " {}", os[i])?;
        }
        if let Some(uv) = mesh.uvs.as_ref() {
            let (u, t) = uv[i];
            write!(writer, " {u} {t}")?;
        }
        for v in mesh.extras.values() {
            write!(writer, " {}", v[i])?;
        }
        writeln!(writer)?;
    }
    for &[i0, i1, i2] in &mesh.geometry.faces {
        writeln!(writer, "3 {i0} {i1} {i2}")?;
    }
    Ok(())
}

/// Load an [`AttributedMesh`] from PLY, recovering normals, colors, zone IDs,
/// clearances, offsets, UVs, and any extra per-vertex float properties into
/// the `extras` map.
///
/// Property recognition:
/// - `x, y, z` → `geometry.vertices` (required).
/// - `nx, ny, nz` (all three present) → `normals`.
/// - `red, green, blue` (all three present, `uchar`) → `colors`.
/// - `zone_id` → `zone_ids`.
/// - `clearance` → `clearances`.
/// - `offset` → `offsets`.
/// - `s, t` (both present) → `uvs`.
/// - Other `float` / `double` properties → `extras` under their raw name.
/// - Other property types are silently ignored (forward-compat).
///
/// Partial canonical sets (e.g., `nx` without `ny, nz`) are dropped; they are
/// reserved names so they do not appear in `extras` either.
///
/// # Errors
///
/// Returns an error if the file cannot be read or is not valid PLY.
///
/// # Example
///
/// ```no_run
/// use mesh_io::load_ply_attributed;
///
/// let mesh = load_ply_attributed("model.ply").unwrap();
/// println!("{} vertices, {} extras", mesh.vertex_count(), mesh.extras.len());
/// ```
pub fn load_ply_attributed<P: AsRef<Path>>(path: P) -> IoResult<AttributedMesh> {
    let path = path.as_ref();
    let file = File::open(path).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            IoError::FileNotFound {
                path: path.to_path_buf(),
            }
        } else {
            IoError::Io(e)
        }
    })?;
    let mut reader = BufReader::new(file);
    let parser = Parser::<DefaultElement>::new();
    let header = parser
        .read_header(&mut reader)
        .map_err(|e| IoError::invalid_content(format!("failed to parse PLY header: {e}")))?;
    let payload = parser
        .read_payload(&mut reader, &header)
        .map_err(|e| IoError::invalid_content(format!("failed to read PLY payload: {e}")))?;

    let mut indexed = IndexedMesh::new();
    if let Some(vs) = payload.get("vertex") {
        indexed.vertices.reserve(vs.len());
        for el in vs {
            let x = get_float_property(el, "x").unwrap_or(0.0);
            let y = get_float_property(el, "y").unwrap_or(0.0);
            let z = get_float_property(el, "z").unwrap_or(0.0);
            indexed
                .vertices
                .push(Point3::new(f64::from(x), f64::from(y), f64::from(z)));
        }
    }
    if let Some(fs) = payload.get("face") {
        indexed.faces.reserve(fs.len());
        for el in fs {
            let idx = get_index_list(el);
            if idx.len() >= 3 {
                for i in 1..idx.len() - 1 {
                    indexed
                        .faces
                        .push([idx[0] as u32, idx[i] as u32, idx[i + 1] as u32]);
                }
            }
        }
    }

    let mut mesh = AttributedMesh::new(indexed);
    if let Some(vs) = payload.get("vertex") {
        populate_attributes_from_vertex_elements(&mut mesh, vs);
    }
    Ok(mesh)
}

/// Pull canonical attribute slots and unknown float properties out of the
/// vertex element list and into `mesh`.
fn populate_attributes_from_vertex_elements(
    mesh: &mut AttributedMesh,
    vertices: &[DefaultElement],
) {
    let n = vertices.len();
    if n == 0 {
        return;
    }

    // Discover property names by scanning the first element. PLY guarantees
    // every element of a given kind has the same property set.
    let names: Vec<String> = vertices[0].keys().cloned().collect();

    let has = |k: &str| names.iter().any(|n| n == k);

    if has("nx") && has("ny") && has("nz") {
        let mut ns = Vec::with_capacity(n);
        for el in vertices {
            let x = get_float_property(el, "nx").unwrap_or(0.0);
            let y = get_float_property(el, "ny").unwrap_or(0.0);
            let z = get_float_property(el, "nz").unwrap_or(0.0);
            ns.push(Vector3::new(f64::from(x), f64::from(y), f64::from(z)));
        }
        mesh.normals = Some(ns);
    }
    if has("red") && has("green") && has("blue") {
        let mut want = Vec::with_capacity(n);
        let mut all_uchar = true;
        for el in vertices {
            if let (Some(Property::UChar(r)), Some(Property::UChar(g)), Some(Property::UChar(b))) =
                (el.get("red"), el.get("green"), el.get("blue"))
            {
                want.push(VertexColor::new(*r, *g, *b));
            } else {
                all_uchar = false;
                break;
            }
        }
        if all_uchar {
            mesh.colors = Some(want);
        }
    }
    if has("zone_id") {
        let mut zs = Vec::with_capacity(n);
        for el in vertices {
            zs.push(get_uint_property(el, "zone_id").unwrap_or(0));
        }
        mesh.zone_ids = Some(zs);
    }
    if has("clearance") {
        let mut cs = Vec::with_capacity(n);
        for el in vertices {
            cs.push(get_float_property(el, "clearance").unwrap_or(0.0));
        }
        mesh.clearances = Some(cs);
    }
    if has("offset") {
        let mut os = Vec::with_capacity(n);
        for el in vertices {
            os.push(get_float_property(el, "offset").unwrap_or(0.0));
        }
        mesh.offsets = Some(os);
    }
    if has("s") && has("t") {
        let mut uvs = Vec::with_capacity(n);
        for el in vertices {
            let s = get_float_property(el, "s").unwrap_or(0.0);
            let t = get_float_property(el, "t").unwrap_or(0.0);
            uvs.push((s, t));
        }
        mesh.uvs = Some(uvs);
    }

    // Anything that's a float-typed property and isn't a reserved canonical
    // name lands in extras. Reserved-but-incomplete sets are dropped, not
    // routed to extras (the names are reserved regardless of completeness).
    let mut extras: BTreeMap<String, Vec<f32>> = BTreeMap::new();
    for name in &names {
        if reserved_reason(name).is_some() {
            continue;
        }
        let mut values = Vec::with_capacity(n);
        let mut all_float = true;
        for el in vertices {
            match el.get(name) {
                Some(Property::Float(v)) => values.push(*v),
                Some(Property::Double(v)) => values.push(*v as f32),
                _ => {
                    all_float = false;
                    break;
                }
            }
        }
        if all_float {
            extras.insert(name.clone(), values);
        }
    }
    mesh.extras = extras;
}

/// Extract a `uint`-style integer property from a PLY element.
fn get_uint_property(element: &DefaultElement, key: &str) -> Option<u32> {
    match element.get(key)? {
        Property::UInt(v) => Some(*v),
        Property::Int(v) => Some(*v as u32),
        Property::UShort(v) => Some(u32::from(*v)),
        Property::Short(v) => Some(*v as u32),
        Property::UChar(v) => Some(u32::from(*v)),
        Property::Char(v) => Some(*v as u32),
        _ => None,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn create_test_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a unit cube
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 1.0));
        mesh.vertices.push(Point3::new(1.0, 1.0, 1.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 1.0));

        // 12 triangles (2 per face)
        // Bottom
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Top
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        // Front
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // Back
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // Left
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        // Right
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn roundtrip_binary() {
        let original = create_test_triangle();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("test.ply");
            save_ply(&original, &path, true).ok();

            if let Ok(loaded) = load_ply(&path) {
                assert_eq!(loaded.face_count(), original.face_count());
                assert_eq!(loaded.vertex_count(), original.vertex_count());

                // Check vertex positions
                for (orig, load) in original.vertices.iter().zip(loaded.vertices.iter()) {
                    assert!((orig.x - load.x).abs() < 1e-5);
                    assert!((orig.y - load.y).abs() < 1e-5);
                    assert!((orig.z - load.z).abs() < 1e-5);
                }
            }
        }
    }

    #[test]
    fn roundtrip_ascii() {
        let original = create_test_triangle();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("test_ascii.ply");
            save_ply(&original, &path, false).ok();

            if let Ok(loaded) = load_ply(&path) {
                assert_eq!(loaded.face_count(), original.face_count());
                assert_eq!(loaded.vertex_count(), original.vertex_count());
            }
        }
    }

    #[test]
    fn roundtrip_cube_binary() {
        let original = create_test_cube();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("cube.ply");
            save_ply(&original, &path, true).ok();

            if let Ok(loaded) = load_ply(&path) {
                assert_eq!(loaded.face_count(), original.face_count());
                assert_eq!(loaded.vertex_count(), original.vertex_count());
            }
        }
    }

    #[test]
    fn load_nonexistent_file() {
        let result = load_ply("nonexistent_file_12345.ply");
        assert!(result.is_err());
        if let Err(IoError::FileNotFound { path }) = result {
            assert!(path.to_string_lossy().contains("nonexistent"));
        }
    }

    // -----------------------------------------------------------------
    // AttributedMesh round-trip + validation tests
    // -----------------------------------------------------------------

    fn cube_with_attrs() -> AttributedMesh {
        let mesh = AttributedMesh::new(create_test_cube());
        assert_eq!(mesh.vertex_count(), 8);
        mesh
    }

    fn write_then_read(mesh: &AttributedMesh, binary: bool, name: &str) -> AttributedMesh {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(name);
        save_ply_attributed(mesh, &path, binary).expect("save");
        load_ply_attributed(&path).expect("load")
    }

    #[test]
    fn roundtrip_attributed_extras_only_binary() {
        let mut mesh = cube_with_attrs();
        let stress: Vec<f32> = (0_u8..8).map(|i| f32::from(i) * 0.5).collect();
        mesh.insert_extra("stress", stress.clone()).unwrap();
        let loaded = write_then_read(&mesh, true, "extras_only.ply");
        assert_eq!(loaded.vertex_count(), mesh.vertex_count());
        assert_eq!(loaded.face_count(), mesh.face_count());
        assert_eq!(loaded.extras.get("stress"), Some(&stress));
    }

    #[test]
    fn roundtrip_attributed_extras_only_ascii() {
        let mut mesh = cube_with_attrs();
        let stress: Vec<f32> = (0_u8..8).map(|i| f32::from(i) * 0.25).collect();
        mesh.insert_extra("stress", stress.clone()).unwrap();
        let loaded = write_then_read(&mesh, false, "extras_only_ascii.ply");
        assert_eq!(loaded.extras.get("stress"), Some(&stress));
    }

    #[test]
    fn roundtrip_attributed_normals_binary() {
        let mut mesh = cube_with_attrs();
        mesh.compute_normals();
        let original = mesh.normals.clone().unwrap();
        let loaded = write_then_read(&mesh, true, "normals.ply");
        let got = loaded.normals.expect("normals present");
        assert_eq!(got.len(), original.len());
        for (a, b) in original.iter().zip(got.iter()) {
            assert!((a.x - b.x).abs() < 1e-5);
            assert!((a.y - b.y).abs() < 1e-5);
            assert!((a.z - b.z).abs() < 1e-5);
        }
    }

    #[test]
    fn roundtrip_attributed_colors_ascii() {
        let mut mesh = cube_with_attrs();
        mesh.colors = Some(vec![VertexColor::new(255, 64, 0); 8]);
        let loaded = write_then_read(&mesh, false, "colors.ply");
        assert_eq!(loaded.colors.as_ref().map(Vec::len), Some(8));
        for c in loaded.colors.as_ref().unwrap() {
            assert_eq!((c.r, c.g, c.b), (255, 64, 0));
        }
    }

    #[test]
    fn roundtrip_attributed_full_binary() {
        let mut mesh = cube_with_attrs();
        mesh.compute_normals();
        mesh.colors = Some(vec![VertexColor::new(10, 20, 30); 8]);
        mesh.zone_ids = Some((0..8_u32).collect());
        mesh.clearances = Some((0_u8..8).map(f32::from).collect());
        mesh.offsets = Some((0_u8..8).map(|i| -f32::from(i)).collect());
        mesh.uvs = Some(
            (0_u8..8)
                .map(|i| (f32::from(i) * 0.1, f32::from(i) * 0.2))
                .collect(),
        );
        mesh.insert_extra("a", vec![1.0_f32; 8]).unwrap();
        mesh.insert_extra("b", vec![2.0_f32; 8]).unwrap();
        let loaded = write_then_read(&mesh, true, "full_binary.ply");
        assert!(loaded.normals.is_some());
        assert_eq!(loaded.colors.as_ref().unwrap()[0].r, 10);
        assert_eq!(loaded.zone_ids.as_ref().unwrap()[7], 7);
        assert!((loaded.clearances.as_ref().unwrap()[3] - 3.0).abs() < f32::EPSILON);
        assert!((loaded.offsets.as_ref().unwrap()[5] + 5.0).abs() < f32::EPSILON);
        let (s, t) = loaded.uvs.as_ref().unwrap()[4];
        assert!((s - 0.4).abs() < 1e-6 && (t - 0.8).abs() < 1e-6);
        assert_eq!(loaded.extras.get("a"), Some(&vec![1.0_f32; 8]));
        assert_eq!(loaded.extras.get("b"), Some(&vec![2.0_f32; 8]));
    }

    #[test]
    fn roundtrip_attributed_full_ascii() {
        let mut mesh = cube_with_attrs();
        mesh.compute_normals();
        mesh.colors = Some(vec![VertexColor::new(200, 100, 50); 8]);
        mesh.zone_ids = Some(vec![42_u32; 8]);
        mesh.insert_extra("mass", vec![2.5_f32; 8]).unwrap();
        let loaded = write_then_read(&mesh, false, "full_ascii.ply");
        assert!(loaded.normals.is_some());
        assert_eq!(loaded.colors.as_ref().unwrap()[0].r, 200);
        assert_eq!(loaded.zone_ids.as_ref().unwrap()[3], 42);
        assert_eq!(loaded.extras.get("mass"), Some(&vec![2.5_f32; 8]));
    }

    #[test]
    fn save_attributed_no_attrs_loads_via_load_ply() {
        // Empty AttributedMesh should produce a PLY readable by the
        // geometry-only loader, recovering the same positions and faces.
        let mesh = cube_with_attrs();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plain.ply");
        save_ply_attributed(&mesh, &path, true).unwrap();
        let loaded = load_ply(&path).unwrap();
        assert_eq!(loaded.vertex_count(), mesh.vertex_count());
        assert_eq!(loaded.face_count(), mesh.face_count());
    }

    #[test]
    fn save_ply_attributed_rejects_reserved_extra_x() {
        let mut mesh = cube_with_attrs();
        mesh.insert_extra("x", vec![0.0_f32; 8]).unwrap();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.ply");
        let err = save_ply_attributed(&mesh, &path, true).unwrap_err();
        match err {
            IoError::InvalidAttributeName { name, .. } => assert_eq!(name, "x"),
            other => panic!("expected InvalidAttributeName, got {other:?}"),
        }
        assert!(
            !path.exists(),
            "no file should be created on validation error"
        );
    }

    #[test]
    fn save_ply_attributed_rejects_reserved_extra_nx() {
        let mut mesh = cube_with_attrs();
        mesh.insert_extra("nx", vec![0.0_f32; 8]).unwrap();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.ply");
        let err = save_ply_attributed(&mesh, &path, true).unwrap_err();
        assert!(matches!(err, IoError::InvalidAttributeName { ref name, .. } if name == "nx"));
    }

    #[test]
    fn save_ply_attributed_rejects_whitespace_in_extra_name() {
        let mut mesh = cube_with_attrs();
        mesh.insert_extra("strain rate", vec![0.0_f32; 8]).unwrap();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.ply");
        let err = save_ply_attributed(&mesh, &path, true).unwrap_err();
        assert!(
            matches!(err, IoError::InvalidAttributeName { reason, .. } if reason.contains("whitespace"))
        );
    }

    #[test]
    fn save_ply_attributed_rejects_normals_length_mismatch() {
        let mut mesh = cube_with_attrs();
        // Bypass insert_extra invariant via direct field mutation.
        mesh.normals = Some(vec![Vector3::new(0.0, 0.0, 1.0); 4]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.ply");
        let err = save_ply_attributed(&mesh, &path, true).unwrap_err();
        match err {
            IoError::InvalidContent { message } => {
                assert!(message.contains("normals"));
                assert!(message.contains('4'));
                assert!(message.contains('8'));
            }
            other => panic!("expected InvalidContent, got {other:?}"),
        }
    }

    #[test]
    fn load_ply_attributed_collects_unknown_float_into_extras() {
        // Hand-author an ASCII PLY with a custom float property and round-trip
        // through the attributed loader, verifying the extras pickup.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("external.ply");
        std::fs::write(
            &path,
            "ply\n\
             format ascii 1.0\n\
             element vertex 3\n\
             property float x\n\
             property float y\n\
             property float z\n\
             property float confidence\n\
             element face 1\n\
             property list uchar int vertex_indices\n\
             end_header\n\
             0 0 0 0.1\n\
             1 0 0 0.5\n\
             0 1 0 0.9\n\
             3 0 1 2\n",
        )
        .unwrap();
        let loaded = load_ply_attributed(&path).unwrap();
        assert_eq!(loaded.vertex_count(), 3);
        assert_eq!(loaded.face_count(), 1);
        let conf = loaded
            .extras
            .get("confidence")
            .expect("confidence in extras");
        assert!((conf[0] - 0.1).abs() < 1e-6);
        assert!((conf[1] - 0.5).abs() < 1e-6);
        assert!((conf[2] - 0.9).abs() < 1e-6);
    }
}
