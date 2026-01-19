//! 3MF (3D Manufacturing Format) support.
//!
//! 3MF is a ZIP-based format designed for 3D printing, containing XML files
//! that describe geometry and optionally materials, colors, and metadata.
//!
//! # Format Structure
//!
//! A 3MF file is a ZIP archive containing:
//! - `3D/3dmodel.model` - Main model XML file
//! - `\[Content_Types\].xml` - MIME type mappings
//! - `_rels/.rels` - Relationships
//!
//! # Supported Features
//!
//! - Mesh geometry (vertices and triangles)
//! - Multiple objects in a single file
//!
//! # Limitations
//!
//! - Materials, colors, and textures are not currently supported
//! - Beam lattice extension is not supported
//! - Build items (transformations) are ignored
//!
//! # Example
//!
//! ```no_run
//! use mesh_io::{load_3mf, save_3mf};
//!
//! let mesh = load_3mf("model.3mf").unwrap();
//! save_3mf(&mesh, "output.3mf").unwrap();
//! ```

use std::fs::File;
use std::io::{BufReader, Cursor, Read, Write};
use std::path::Path;

use mesh_types::{IndexedMesh, Vertex};
use quick_xml::events::{BytesDecl, BytesEnd, BytesStart, Event};
use quick_xml::{Reader, Writer};
use zip::write::SimpleFileOptions;
use zip::{ZipArchive, ZipWriter};

use crate::error::{IoError, IoResult};

/// 3MF namespace URI.
const NAMESPACE_3MF: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";

/// Load a mesh from a 3MF file.
///
/// Loads all mesh objects from the 3MF file and combines them into a single mesh.
/// If multiple objects exist, they are concatenated (vertex indices are offset).
///
/// # Arguments
///
/// * `path` - Path to the 3MF file
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be read
/// - The file is not a valid ZIP archive
/// - The 3MF model file is missing or invalid
///
/// # Example
///
/// ```no_run
/// use mesh_io::load_3mf;
///
/// let mesh = load_3mf("model.3mf").unwrap();
/// println!("Loaded {} vertices, {} faces", mesh.vertices.len(), mesh.faces.len());
/// ```
pub fn load_3mf<P: AsRef<Path>>(path: P) -> IoResult<IndexedMesh> {
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
    let reader = BufReader::new(file);

    let mut archive = ZipArchive::new(reader)
        .map_err(|e| IoError::invalid_content(format!("invalid ZIP archive: {e}")))?;

    // Find and read the 3D model file
    let model_content = read_model_file(&mut archive)?;

    // Parse the XML model
    parse_3mf_model(&model_content)
}

/// Read the 3D model file from the archive.
fn read_model_file<R: Read + std::io::Seek>(archive: &mut ZipArchive<R>) -> IoResult<String> {
    // Try standard path first
    let model_paths = ["3D/3dmodel.model", "3d/3dmodel.model", "3D/3DModel.model"];

    for model_path in &model_paths {
        if let Ok(mut file) = archive.by_name(model_path) {
            let mut content = String::new();
            file.read_to_string(&mut content)?;
            return Ok(content);
        }
    }

    // Try to find any .model file
    for i in 0..archive.len() {
        let file = archive
            .by_index(i)
            .map_err(|e| IoError::invalid_content(format!("failed to read archive entry: {e}")))?;
        let name = file.name().to_lowercase();
        if std::path::Path::new(&name)
            .extension()
            .is_some_and(|ext| ext.eq_ignore_ascii_case("model"))
        {
            drop(file);
            let mut file = archive
                .by_index(i)
                .map_err(|e| IoError::invalid_content(format!("failed to read model file: {e}")))?;
            let mut content = String::new();
            file.read_to_string(&mut content)?;
            return Ok(content);
        }
    }

    Err(IoError::invalid_content(
        "3MF archive does not contain a model file",
    ))
}

/// Parse the 3MF model XML content.
fn parse_3mf_model(content: &str) -> IoResult<IndexedMesh> {
    let mut reader = Reader::from_str(content);
    reader.config_mut().trim_text(true);

    let mut mesh = IndexedMesh::new();
    let mut in_mesh = false;
    let mut in_vertices = false;
    let mut in_triangles = false;
    let mut current_vertex_offset: u32 = 0;

    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e) | Event::Empty(ref e)) => {
                let local_name = e.local_name();
                match local_name.as_ref() {
                    b"mesh" => {
                        in_mesh = true;
                        // Track vertex offset for this mesh object
                        #[allow(clippy::cast_possible_truncation)]
                        {
                            current_vertex_offset = mesh.vertices.len() as u32;
                        }
                    }
                    b"vertices" => {
                        if in_mesh {
                            in_vertices = true;
                        }
                    }
                    b"triangles" => {
                        if in_mesh {
                            in_triangles = true;
                        }
                    }
                    b"vertex" => {
                        if in_vertices {
                            let vertex = parse_vertex_element(e)?;
                            mesh.vertices.push(vertex);
                        }
                    }
                    b"triangle" => {
                        if in_triangles {
                            let face = parse_triangle_element(e, current_vertex_offset)?;
                            mesh.faces.push(face);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) => {
                let local_name = e.local_name();
                match local_name.as_ref() {
                    b"mesh" => in_mesh = false,
                    b"vertices" => in_vertices = false,
                    b"triangles" => in_triangles = false,
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => {
                return Err(IoError::invalid_content(format!("XML parse error: {e}")));
            }
            _ => {}
        }
        buf.clear();
    }

    Ok(mesh)
}

/// Parse a vertex element from XML attributes.
fn parse_vertex_element(element: &BytesStart<'_>) -> IoResult<Vertex> {
    let mut x = 0.0_f64;
    let mut y = 0.0_f64;
    let mut z = 0.0_f64;

    for attr in element.attributes().flatten() {
        let key = attr.key.local_name();
        let value = std::str::from_utf8(&attr.value)
            .map_err(|e| IoError::invalid_content(format!("invalid UTF-8 in attribute: {e}")))?;

        match key.as_ref() {
            b"x" => {
                x = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid x coordinate: {e}")))?;
            }
            b"y" => {
                y = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid y coordinate: {e}")))?;
            }
            b"z" => {
                z = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid z coordinate: {e}")))?;
            }
            _ => {}
        }
    }

    Ok(Vertex::from_coords(x, y, z))
}

/// Parse a triangle element from XML attributes.
fn parse_triangle_element(element: &BytesStart<'_>, vertex_offset: u32) -> IoResult<[u32; 3]> {
    let mut v1 = 0_u32;
    let mut v2 = 0_u32;
    let mut v3 = 0_u32;

    for attr in element.attributes().flatten() {
        let key = attr.key.local_name();
        let value = std::str::from_utf8(&attr.value)
            .map_err(|e| IoError::invalid_content(format!("invalid UTF-8 in attribute: {e}")))?;

        match key.as_ref() {
            b"v1" => {
                v1 = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid v1 index: {e}")))?;
            }
            b"v2" => {
                v2 = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid v2 index: {e}")))?;
            }
            b"v3" => {
                v3 = value
                    .parse()
                    .map_err(|e| IoError::invalid_content(format!("invalid v3 index: {e}")))?;
            }
            _ => {}
        }
    }

    Ok([v1 + vertex_offset, v2 + vertex_offset, v3 + vertex_offset])
}

/// Save a mesh to a 3MF file.
///
/// Creates a valid 3MF archive with the mesh geometry.
///
/// # Arguments
///
/// * `mesh` - The mesh to save
/// * `path` - Output file path
///
/// # Errors
///
/// Returns an error if the file cannot be written.
///
/// # Example
///
/// ```no_run
/// use mesh_io::{load_3mf, save_3mf};
///
/// let mesh = load_3mf("input.3mf").unwrap();
/// save_3mf(&mesh, "output.3mf").unwrap();
/// ```
pub fn save_3mf<P: AsRef<Path>>(mesh: &IndexedMesh, path: P) -> IoResult<()> {
    let file = File::create(path)?;
    let mut zip = ZipWriter::new(file);

    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    // Write [Content_Types].xml
    zip.start_file("[Content_Types].xml", options)
        .map_err(|e| {
            IoError::invalid_content(format!("failed to create content types file: {e}"))
        })?;
    zip.write_all(CONTENT_TYPES_XML.as_bytes())?;

    // Write _rels/.rels
    zip.start_file("_rels/.rels", options)
        .map_err(|e| IoError::invalid_content(format!("failed to create rels file: {e}")))?;
    zip.write_all(RELS_XML.as_bytes())?;

    // Write 3D/3dmodel.model
    let model_xml = generate_model_xml(mesh)?;
    zip.start_file("3D/3dmodel.model", options)
        .map_err(|e| IoError::invalid_content(format!("failed to create model file: {e}")))?;
    zip.write_all(model_xml.as_bytes())?;

    zip.finish()
        .map_err(|e| IoError::invalid_content(format!("failed to finalize ZIP archive: {e}")))?;

    Ok(())
}

/// Content types XML for 3MF.
const CONTENT_TYPES_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#;

/// Relationships XML for 3MF.
const RELS_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;

/// Generate the 3D model XML content.
fn generate_model_xml(mesh: &IndexedMesh) -> IoResult<String> {
    let mut buffer = Vec::new();
    let mut writer = Writer::new_with_indent(Cursor::new(&mut buffer), b' ', 2);

    // XML declaration
    writer
        .write_event(Event::Decl(BytesDecl::new("1.0", Some("UTF-8"), None)))
        .map_err(|e| IoError::invalid_content(format!("failed to write XML declaration: {e}")))?;

    // Root element: model
    let mut model = BytesStart::new("model");
    model.push_attribute(("xmlns", NAMESPACE_3MF));
    model.push_attribute(("unit", "millimeter"));
    model.push_attribute(("xml:lang", "en-US"));
    writer
        .write_event(Event::Start(model))
        .map_err(|e| IoError::invalid_content(format!("failed to write model element: {e}")))?;

    // Resources element
    writer
        .write_event(Event::Start(BytesStart::new("resources")))
        .map_err(|e| IoError::invalid_content(format!("failed to write resources element: {e}")))?;

    // Object element (single object with id="1")
    let mut object = BytesStart::new("object");
    object.push_attribute(("id", "1"));
    object.push_attribute(("type", "model"));
    writer
        .write_event(Event::Start(object))
        .map_err(|e| IoError::invalid_content(format!("failed to write object element: {e}")))?;

    // Mesh element
    writer
        .write_event(Event::Start(BytesStart::new("mesh")))
        .map_err(|e| IoError::invalid_content(format!("failed to write mesh element: {e}")))?;

    // Vertices
    writer
        .write_event(Event::Start(BytesStart::new("vertices")))
        .map_err(|e| IoError::invalid_content(format!("failed to write vertices element: {e}")))?;

    for v in &mesh.vertices {
        let mut vertex = BytesStart::new("vertex");
        vertex.push_attribute(("x", format!("{:.6}", v.position.x).as_str()));
        vertex.push_attribute(("y", format!("{:.6}", v.position.y).as_str()));
        vertex.push_attribute(("z", format!("{:.6}", v.position.z).as_str()));
        writer
            .write_event(Event::Empty(vertex))
            .map_err(|e| IoError::invalid_content(format!("failed to write vertex: {e}")))?;
    }

    writer
        .write_event(Event::End(BytesEnd::new("vertices")))
        .map_err(|e| IoError::invalid_content(format!("failed to close vertices: {e}")))?;

    // Triangles
    writer
        .write_event(Event::Start(BytesStart::new("triangles")))
        .map_err(|e| IoError::invalid_content(format!("failed to write triangles element: {e}")))?;

    for &[v1, v2, v3] in &mesh.faces {
        let mut triangle = BytesStart::new("triangle");
        triangle.push_attribute(("v1", v1.to_string().as_str()));
        triangle.push_attribute(("v2", v2.to_string().as_str()));
        triangle.push_attribute(("v3", v3.to_string().as_str()));
        writer
            .write_event(Event::Empty(triangle))
            .map_err(|e| IoError::invalid_content(format!("failed to write triangle: {e}")))?;
    }

    writer
        .write_event(Event::End(BytesEnd::new("triangles")))
        .map_err(|e| IoError::invalid_content(format!("failed to close triangles: {e}")))?;

    // Close mesh, object, resources
    writer
        .write_event(Event::End(BytesEnd::new("mesh")))
        .map_err(|e| IoError::invalid_content(format!("failed to close mesh: {e}")))?;
    writer
        .write_event(Event::End(BytesEnd::new("object")))
        .map_err(|e| IoError::invalid_content(format!("failed to close object: {e}")))?;
    writer
        .write_event(Event::End(BytesEnd::new("resources")))
        .map_err(|e| IoError::invalid_content(format!("failed to close resources: {e}")))?;

    // Build element (references the object)
    writer
        .write_event(Event::Start(BytesStart::new("build")))
        .map_err(|e| IoError::invalid_content(format!("failed to write build element: {e}")))?;

    let mut item = BytesStart::new("item");
    item.push_attribute(("objectid", "1"));
    writer
        .write_event(Event::Empty(item))
        .map_err(|e| IoError::invalid_content(format!("failed to write item: {e}")))?;

    writer
        .write_event(Event::End(BytesEnd::new("build")))
        .map_err(|e| IoError::invalid_content(format!("failed to close build: {e}")))?;

    // Close model
    writer
        .write_event(Event::End(BytesEnd::new("model")))
        .map_err(|e| IoError::invalid_content(format!("failed to close model: {e}")))?;

    String::from_utf8(buffer)
        .map_err(|e| IoError::invalid_content(format!("invalid UTF-8 in generated XML: {e}")))
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::unnecessary_raw_string_hashes
)]
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
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn roundtrip_triangle() {
        let original = create_test_triangle();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("test.3mf");
            save_3mf(&original, &path).ok();

            if let Ok(loaded) = load_3mf(&path) {
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
    fn roundtrip_cube() {
        let original = create_test_cube();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("cube.3mf");
            save_3mf(&original, &path).ok();

            if let Ok(loaded) = load_3mf(&path) {
                assert_eq!(loaded.face_count(), original.face_count());
                assert_eq!(loaded.vertex_count(), original.vertex_count());
            }
        }
    }

    #[test]
    fn load_nonexistent_file() {
        let result = load_3mf("nonexistent_file_12345.3mf");
        assert!(result.is_err());
        if let Err(IoError::FileNotFound { path }) = result {
            assert!(path.to_string_lossy().contains("nonexistent"));
        }
    }

    #[test]
    fn generate_valid_xml() {
        let mesh = create_test_triangle();
        let xml = generate_model_xml(&mesh);
        assert!(xml.is_ok());

        let xml = xml.map(|x| x);
        if let Ok(content) = xml {
            // Check that essential elements are present
            assert!(content.contains("<model"));
            assert!(content.contains("<mesh"));
            assert!(content.contains("<vertices"));
            assert!(content.contains("<triangles"));
            assert!(content.contains("<vertex"));
            assert!(content.contains("<triangle"));
            assert!(content.contains("</model>"));
        }
    }

    #[test]
    fn parse_minimal_model() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="1" y="0" z="0"/>
          <vertex x="0" y="1" z="0"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
        </triangles>
      </mesh>
    </object>
  </resources>
</model>"#;

        let mesh = parse_3mf_model(xml);
        assert!(mesh.is_ok());

        let mesh = mesh.map(|m| m);
        if let Ok(m) = mesh {
            assert_eq!(m.vertex_count(), 3);
            assert_eq!(m.face_count(), 1);
        }
    }
}
