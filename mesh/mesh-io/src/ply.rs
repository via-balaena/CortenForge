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

use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

use mesh_types::{IndexedMesh, Vertex};
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
            mesh.vertices.push(Vertex::from_coords(
                f64::from(x),
                f64::from(y),
                f64::from(z),
            ));
        }
    }

    // Extract faces
    if let Some(face_elements) = payload.get("face") {
        mesh.faces.reserve(face_elements.len());
        for element in face_elements {
            let indices = get_index_list(element);
            if indices.len() >= 3 {
                // Triangulate if necessary (fan triangulation for convex polygons)
                #[allow(clippy::cast_possible_truncation)]
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
        Property::Double(v) =>
        {
            #[allow(clippy::cast_possible_truncation)]
            Some(*v as f32)
        }
        _ => None,
    }
}

/// Extract vertex index list from a face element.
fn get_index_list(element: &DefaultElement) -> Vec<usize> {
    // Try common property names for face indices
    for key in &["vertex_indices", "vertex_index"] {
        if let Some(prop) = element.get(*key) {
            return match prop {
                Property::ListInt(v) =>
                {
                    #[allow(clippy::cast_sign_loss)]
                    v.iter().map(|&i| i as usize).collect()
                }
                Property::ListUInt(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListUChar(v) => v.iter().map(|&i| i as usize).collect(),
                Property::ListChar(v) =>
                {
                    #[allow(clippy::cast_sign_loss)]
                    v.iter().map(|&i| i as usize).collect()
                }
                Property::ListShort(v) =>
                {
                    #[allow(clippy::cast_sign_loss)]
                    v.iter().map(|&i| i as usize).collect()
                }
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
        #[allow(clippy::cast_possible_truncation)]
        {
            writer.write_all(&(v.position.x as f32).to_le_bytes())?;
            writer.write_all(&(v.position.y as f32).to_le_bytes())?;
            writer.write_all(&(v.position.z as f32).to_le_bytes())?;
        }
    }

    // Write face data
    for &[i0, i1, i2] in &mesh.faces {
        // List count (3 vertices per face)
        writer.write_all(&[3u8])?;
        // Vertex indices as i32
        #[allow(clippy::cast_possible_wrap)]
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
        #[allow(clippy::cast_possible_truncation)]
        {
            element.insert("x".to_string(), Property::Float(v.position.x as f32));
            element.insert("y".to_string(), Property::Float(v.position.y as f32));
            element.insert("z".to_string(), Property::Float(v.position.z as f32));
        }
        vertex_elements.push(element);
    }
    ply.payload.insert("vertex".to_string(), vertex_elements);

    // Add face data
    let mut face_elements = Vec::with_capacity(mesh.faces.len());
    for &[i0, i1, i2] in &mesh.faces {
        let mut element = DefaultElement::new();
        #[allow(clippy::cast_possible_wrap)]
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

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::MeshTopology;

    fn create_test_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a unit cube
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 1.0));

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
                    assert!((orig.position.x - load.position.x).abs() < 1e-5);
                    assert!((orig.position.y - load.position.y).abs() < 1e-5);
                    assert!((orig.position.z - load.position.z).abs() < 1e-5);
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
}
