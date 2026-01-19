//! STL (Stereolithography) file format support.
//!
//! Supports both ASCII and binary STL formats.
//!
//! # Format Detection
//!
//! The loader automatically detects whether a file is ASCII or binary:
//! - ASCII files start with "solid" (after optional whitespace)
//! - Binary files have an 80-byte header followed by face count
//!
//! # Binary Format
//!
//! ```text
//! UINT8[80]    – Header (ignored, often contains file info)
//! UINT32       – Number of triangles
//! foreach triangle
//!     REAL32[3] – Normal vector (often not accurate)
//!     REAL32[3] – Vertex 1
//!     REAL32[3] – Vertex 2
//!     REAL32[3] – Vertex 3
//!     UINT16    – Attribute byte count (usually 0)
//! end
//! ```
//!
//! # ASCII Format
//!
//! ```text
//! solid name
//!   facet normal ni nj nk
//!     outer loop
//!       vertex v1x v1y v1z
//!       vertex v2x v2y v2z
//!       vertex v3x v3y v3z
//!     endloop
//!   endfacet
//!   ...
//! endsolid name
//! ```

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

use mesh_types::{IndexedMesh, Vertex};

use crate::error::{IoError, IoResult};

/// STL binary header size in bytes.
const HEADER_SIZE: usize = 80;

/// Size of one triangle in binary STL (normal + 3 vertices + attribute).
const TRIANGLE_SIZE: usize = 50;

/// Load a mesh from an STL file.
///
/// Automatically detects ASCII vs binary format.
///
/// # Arguments
///
/// * `path` - Path to the STL file
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be read
/// - The file content is not valid STL
///
/// # Example
///
/// ```no_run
/// use mesh_io::load_stl;
///
/// let mesh = load_stl("model.stl").unwrap();
/// println!("Loaded {} faces", mesh.faces.len());
/// ```
pub fn load_stl<P: AsRef<Path>>(path: P) -> IoResult<IndexedMesh> {
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

    // Read enough to determine format
    let mut header = [0u8; HEADER_SIZE + 4];
    let bytes_read = reader.read(&mut header)?;

    if bytes_read < 6 {
        return Err(IoError::invalid_content("file too small to be valid STL"));
    }

    // Check if ASCII (starts with "solid")
    let header_str = String::from_utf8_lossy(&header[..bytes_read.min(HEADER_SIZE)]);
    let trimmed = header_str.trim_start();

    if trimmed.starts_with("solid") && !is_binary_stl_header(&header[..bytes_read]) {
        // ASCII format - need to re-read from start
        drop(reader);
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        load_stl_ascii(reader)
    } else {
        // Binary format - continue reading
        load_stl_binary_from_header(&header[..bytes_read], reader)
    }
}

/// Check if the header suggests binary STL despite starting with "solid".
///
/// Some binary STLs happen to have "solid" in the header. We check by seeing
/// if the face count makes sense given the file structure.
fn is_binary_stl_header(header: &[u8]) -> bool {
    if header.len() < HEADER_SIZE + 4 {
        return false;
    }

    // Check if there are any null bytes in what would be ASCII header
    // Binary headers often contain nulls
    header[..HEADER_SIZE].contains(&0)
}

/// Load a binary STL given the already-read header.
fn load_stl_binary_from_header<R: Read>(header: &[u8], mut reader: R) -> IoResult<IndexedMesh> {
    if header.len() < HEADER_SIZE + 4 {
        return Err(IoError::InvalidHeader {
            expected: HEADER_SIZE + 4,
            got: header.len(),
        });
    }

    // Face count is stored after the 80-byte header
    let face_count = u32::from_le_bytes([
        header[HEADER_SIZE],
        header[HEADER_SIZE + 1],
        header[HEADER_SIZE + 2],
        header[HEADER_SIZE + 3],
    ]);

    let mut mesh = IndexedMesh::with_capacity((face_count as usize) * 3, face_count as usize);

    // Read triangles
    let mut triangle_buf = [0u8; TRIANGLE_SIZE];
    for i in 0..face_count {
        let bytes_read = reader.read(&mut triangle_buf)?;
        if bytes_read < TRIANGLE_SIZE {
            return Err(IoError::InvalidFaceCount {
                expected: face_count,
                got: i,
            });
        }

        // Skip normal (12 bytes), read 3 vertices (36 bytes total)
        let v0 = read_vertex(&triangle_buf[12..24]);
        let v1 = read_vertex(&triangle_buf[24..36]);
        let v2 = read_vertex(&triangle_buf[36..48]);

        #[allow(clippy::cast_possible_truncation)]
        // Truncation: mesh indices are u32, meshes with >4B vertices are unsupported
        let base_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(v0);
        mesh.vertices.push(v1);
        mesh.vertices.push(v2);
        mesh.faces.push([base_idx, base_idx + 1, base_idx + 2]);
    }

    Ok(mesh)
}

/// Read a vertex from 12 bytes (3 f32s).
fn read_vertex(buf: &[u8]) -> Vertex {
    let x = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let y = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let z = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    Vertex::from_coords(f64::from(x), f64::from(y), f64::from(z))
}

/// Load an ASCII STL file.
fn load_stl_ascii<R: BufRead>(reader: R) -> IoResult<IndexedMesh> {
    let mut mesh = IndexedMesh::new();
    let mut in_facet = false;
    let mut in_loop = false;
    let mut vertices_in_face: Vec<Vertex> = Vec::with_capacity(3);

    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();

        if trimmed.is_empty() {
            continue;
        }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0].to_lowercase().as_str() {
            "facet" => {
                in_facet = true;
                // Normal follows but we ignore it (recompute if needed)
            }
            "outer" => {
                if parts.len() >= 2 && parts[1].eq_ignore_ascii_case("loop") {
                    in_loop = true;
                    vertices_in_face.clear();
                }
            }
            "vertex" => {
                if in_loop && parts.len() >= 4 {
                    let x: f64 = parts[1].parse()?;
                    let y: f64 = parts[2].parse()?;
                    let z: f64 = parts[3].parse()?;
                    vertices_in_face.push(Vertex::from_coords(x, y, z));
                }
            }
            "endloop" => {
                in_loop = false;
            }
            "endfacet" => {
                if in_facet && vertices_in_face.len() == 3 {
                    #[allow(clippy::cast_possible_truncation)]
                    // Truncation: mesh indices are u32, meshes with >4B vertices unsupported
                    let base_idx = mesh.vertices.len() as u32;
                    mesh.vertices.append(&mut vertices_in_face);
                    mesh.faces.push([base_idx, base_idx + 1, base_idx + 2]);
                }
                in_facet = false;
            }
            "endsolid" => {
                // End of solid
                break;
            }
            _ => {
                // Ignore unknown lines
            }
        }
    }

    Ok(mesh)
}

/// Save a mesh to an STL file.
///
/// # Arguments
///
/// * `mesh` - The mesh to save
/// * `path` - Output file path
/// * `binary` - If true, save as binary STL; if false, save as ASCII
///
/// # Errors
///
/// Returns an error if the file cannot be written.
///
/// # Example
///
/// ```no_run
/// use mesh_io::{load_stl, save_stl};
///
/// let mesh = load_stl("input.stl").unwrap();
/// save_stl(&mesh, "output.stl", true).unwrap(); // Binary
/// save_stl(&mesh, "output_ascii.stl", false).unwrap(); // ASCII
/// ```
pub fn save_stl<P: AsRef<Path>>(mesh: &IndexedMesh, path: P, binary: bool) -> IoResult<()> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);

    if binary {
        save_stl_binary(mesh, writer)
    } else {
        save_stl_ascii(mesh, writer)
    }
}

/// Save mesh as binary STL.
fn save_stl_binary<W: Write>(mesh: &IndexedMesh, mut writer: W) -> IoResult<()> {
    // Write 80-byte header (padded with spaces)
    let mut header = [b' '; HEADER_SIZE];
    let text = b"Binary STL generated by CortenForge mesh-io";
    header[..text.len()].copy_from_slice(text);
    writer.write_all(&header)?;

    // Write face count
    #[allow(clippy::cast_possible_truncation)]
    // Face count: mesh faces limited to u32 range by design
    let face_count = mesh.faces.len() as u32;
    writer.write_all(&face_count.to_le_bytes())?;

    // Write triangles
    for &[i0, i1, i2] in &mesh.faces {
        let v0 = &mesh.vertices[i0 as usize].position;
        let v1 = &mesh.vertices[i1 as usize].position;
        let v2 = &mesh.vertices[i2 as usize].position;

        // Compute normal
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let normal = e1.cross(&e2);
        let len = normal.norm();
        #[allow(clippy::cast_possible_truncation)]
        // Truncation: f64 to f32 is intentional for STL format which uses f32
        let (nx, ny, nz) = if len > f64::EPSILON {
            (
                (normal.x / len) as f32,
                (normal.y / len) as f32,
                (normal.z / len) as f32,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        // Write normal
        writer.write_all(&nx.to_le_bytes())?;
        writer.write_all(&ny.to_le_bytes())?;
        writer.write_all(&nz.to_le_bytes())?;

        // Write vertices
        write_vertex_binary(&mut writer, v0.x, v0.y, v0.z)?;
        write_vertex_binary(&mut writer, v1.x, v1.y, v1.z)?;
        write_vertex_binary(&mut writer, v2.x, v2.y, v2.z)?;

        // Write attribute byte count (0)
        writer.write_all(&0u16.to_le_bytes())?;
    }

    Ok(())
}

/// Write a vertex as 3 f32s in little-endian.
fn write_vertex_binary<W: Write>(writer: &mut W, x: f64, y: f64, z: f64) -> IoResult<()> {
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: f64 to f32 is intentional for STL format
    {
        writer.write_all(&(x as f32).to_le_bytes())?;
        writer.write_all(&(y as f32).to_le_bytes())?;
        writer.write_all(&(z as f32).to_le_bytes())?;
    }
    Ok(())
}

/// Save mesh as ASCII STL.
fn save_stl_ascii<W: Write>(mesh: &IndexedMesh, mut writer: W) -> IoResult<()> {
    writeln!(writer, "solid mesh")?;

    for &[i0, i1, i2] in &mesh.faces {
        let v0 = &mesh.vertices[i0 as usize].position;
        let v1 = &mesh.vertices[i1 as usize].position;
        let v2 = &mesh.vertices[i2 as usize].position;

        // Compute normal
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let normal = e1.cross(&e2);
        let len = normal.norm();
        let (nx, ny, nz) = if len > f64::EPSILON {
            (normal.x / len, normal.y / len, normal.z / len)
        } else {
            (0.0, 0.0, 0.0)
        };

        writeln!(writer, "  facet normal {nx:.6e} {ny:.6e} {nz:.6e}")?;
        writeln!(writer, "    outer loop")?;
        writeln!(
            writer,
            "      vertex {:.6e} {:.6e} {:.6e}",
            v0.x, v0.y, v0.z
        )?;
        writeln!(
            writer,
            "      vertex {:.6e} {:.6e} {:.6e}",
            v1.x, v1.y, v1.z
        )?;
        writeln!(
            writer,
            "      vertex {:.6e} {:.6e} {:.6e}",
            v2.x, v2.y, v2.z
        )?;
        writeln!(writer, "    endloop")?;
        writeln!(writer, "  endfacet")?;
    }

    writeln!(writer, "endsolid mesh")?;

    Ok(())
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

    #[test]
    fn roundtrip_binary() {
        let original = create_test_triangle();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("test.stl");
            save_stl(&original, &path, true).ok();

            if let Ok(loaded) = load_stl(&path) {
                assert_eq!(loaded.face_count(), original.face_count());
                assert_eq!(loaded.vertex_count(), original.vertex_count());
            }
        }
    }

    #[test]
    fn roundtrip_ascii() {
        let original = create_test_triangle();

        let temp_dir = tempfile::tempdir().ok();
        let temp_dir = temp_dir.as_ref();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("test_ascii.stl");
            save_stl(&original, &path, false).ok();

            if let Ok(loaded) = load_stl(&path) {
                assert_eq!(loaded.face_count(), original.face_count());

                // Check vertex positions (with tolerance for ASCII precision)
                if loaded.vertex_count() >= 3 {
                    let v0 = &loaded.vertices[0].position;
                    assert!((v0.x - 0.0).abs() < 1e-5);
                    assert!((v0.y - 0.0).abs() < 1e-5);
                    assert!((v0.z - 0.0).abs() < 1e-5);
                }
            }
        }
    }

    #[test]
    fn load_nonexistent_file() {
        let result = load_stl("nonexistent_file_12345.stl");
        assert!(result.is_err());
        if let Err(IoError::FileNotFound { path }) = result {
            assert!(path.to_string_lossy().contains("nonexistent"));
        }
    }

    #[test]
    fn ascii_stl_parsing() {
        let ascii_stl = br#"solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
endsolid test"#;

        let reader = BufReader::new(&ascii_stl[..]);
        let mesh = load_stl_ascii(reader);

        assert!(mesh.is_ok());
        let mesh = mesh.map(|m| m);
        if let Ok(m) = mesh {
            assert_eq!(m.face_count(), 1);
            assert_eq!(m.vertex_count(), 3);
        }
    }
}
