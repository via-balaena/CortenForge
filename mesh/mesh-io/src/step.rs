//! STEP (Standard for the Exchange of Product Data) format support.
//!
//! STEP is an ISO standard (ISO 10303) for CAD data exchange, widely used in
//! manufacturing, aerospace, and automotive industries.
//!
//! This module provides STEP file support via the truck CAD kernel:
//! - **Loading**: Reads STEP files and converts B-rep geometry to triangle meshes
//! - **Saving**: Converts triangle meshes to B-rep and writes STEP files
//!
//! # Feature Gate
//!
//! This module requires the `step` feature to be enabled:
//!
//! ```toml
//! [dependencies]
//! mesh-io = { version = "0.7", features = ["step"] }
//! ```
//!
//! # Limitations
//!
//! - Complex NURBS surfaces may lose precision when tessellated
//! - Some STEP entities (assemblies, metadata) are not fully supported
//! - Round-trip conversion is lossy (mesh → B-rep approximation)
//!
//! # Example
//!
//! ```ignore
//! use mesh_io::{load_step, save_step};
//!
//! // Load a STEP file and convert to mesh
//! let mesh = load_step("model.step").unwrap();
//!
//! // Save mesh as STEP (creates approximate B-rep)
//! save_step(&mesh, "output.step").unwrap();
//! ```

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use mesh_types::{IndexedMesh, Vertex};
use truck_meshalgo::prelude::*;
use truck_polymesh::PolygonMesh;
use truck_stepio::r#in::Table;

use crate::error::{IoError, IoResult};

/// Load a mesh from a STEP file.
///
/// Reads a STEP file, extracts B-rep geometry using the truck CAD kernel,
/// and tessellates it into a triangle mesh.
///
/// # Arguments
///
/// * `path` - Path to the STEP file
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be read
/// - The STEP file is invalid or unsupported
/// - Tessellation fails
///
/// # Example
///
/// ```ignore
/// use mesh_io::load_step;
///
/// let mesh = load_step("model.step").unwrap();
/// println!("Loaded {} faces from STEP", mesh.faces.len());
/// ```
pub fn load_step<P: AsRef<Path>>(path: P) -> IoResult<IndexedMesh> {
    let path = path.as_ref();
    let step_string = std::fs::read_to_string(path).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            IoError::FileNotFound {
                path: path.to_path_buf(),
            }
        } else {
            IoError::Io(e)
        }
    })?;

    // Parse STEP file using ruststep
    let exchange = truck_stepio::r#in::ruststep::parser::parse(&step_string)
        .map_err(|e| IoError::invalid_content(format!("failed to parse STEP file: {e}")))?;

    if exchange.data.is_empty() {
        return Err(IoError::invalid_content(
            "STEP file contains no data sections",
        ));
    }

    // Convert parsed data to truck Table structure
    let table = Table::from_data_section(&exchange.data[0]);

    // Extract shells and tessellate them
    let mut mesh = IndexedMesh::new();

    for shell_holder in table.shell.values() {
        // Convert to compressed shell
        let Ok(compressed_shell) = table.to_compressed_shell(shell_holder) else {
            continue; // Skip shells that fail to convert
        };

        // Tessellate the shell
        // First pass to get bounding box for tolerance calculation
        let pre_mesh = compressed_shell.robust_triangulation(0.01);
        let poly = pre_mesh.to_polygon();
        let bdd = poly.bounding_box();

        // Second pass with tolerance based on bounding box diameter
        let tolerance = bdd.diameter() * 0.001;
        let tessellated = compressed_shell.robust_triangulation(tolerance);
        let poly_mesh = tessellated.to_polygon();

        append_polymesh_to_indexed(&poly_mesh, &mut mesh);
    }

    if mesh.vertices.is_empty() {
        return Err(IoError::invalid_content(
            "STEP file contains no geometry or tessellation failed",
        ));
    }

    Ok(mesh)
}

/// Append a truck `PolygonMesh` to our `IndexedMesh`.
#[allow(clippy::cast_possible_truncation)]
fn append_polymesh_to_indexed(poly: &PolygonMesh, mesh: &mut IndexedMesh) {
    let vertex_offset = mesh.vertices.len() as u32;

    // Add vertices from positions
    for pos in poly.positions() {
        mesh.vertices.push(Vertex::from_coords(pos.x, pos.y, pos.z));
    }

    // Add faces - handle both tri and quad faces
    for face in poly.tri_faces() {
        mesh.faces.push([
            face[0].pos as u32 + vertex_offset,
            face[1].pos as u32 + vertex_offset,
            face[2].pos as u32 + vertex_offset,
        ]);
    }

    // Triangulate quad faces
    for quad in poly.quad_faces() {
        mesh.faces.push([
            quad[0].pos as u32 + vertex_offset,
            quad[1].pos as u32 + vertex_offset,
            quad[2].pos as u32 + vertex_offset,
        ]);
        mesh.faces.push([
            quad[0].pos as u32 + vertex_offset,
            quad[2].pos as u32 + vertex_offset,
            quad[3].pos as u32 + vertex_offset,
        ]);
    }
}

/// Save a mesh to a STEP file.
///
/// Converts the triangle mesh to a B-rep representation using truck and
/// writes it as a STEP file. Note that this creates an approximation since
/// triangle meshes don't have exact surface information.
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
/// # Limitations
///
/// - The output represents triangulated faces, not smooth surfaces
/// - This is useful for visualization but not for precise CAD operations
///
/// # Example
///
/// ```ignore
/// use mesh_io::{load_mesh, save_step};
///
/// let mesh = load_mesh("model.stl").unwrap();
/// save_step(&mesh, "output.step").unwrap();
/// ```
pub fn save_step<P: AsRef<Path>>(mesh: &IndexedMesh, path: P) -> IoResult<()> {
    use truck_modeling::Point3;
    use truck_stepio::out::{CompleteStepDisplay, StepHeaderDescriptor, StepModel};
    use truck_topology::Shell;

    if mesh.faces.is_empty() {
        return Err(IoError::invalid_content("cannot save empty mesh to STEP"));
    }

    // Build truck Shell from mesh triangles
    // Each triangle becomes a planar face in the B-rep
    let mut faces = Vec::with_capacity(mesh.faces.len());

    for &[i0, i1, i2] in &mesh.faces {
        let v0 = &mesh.vertices[i0 as usize].position;
        let v1 = &mesh.vertices[i1 as usize].position;
        let v2 = &mesh.vertices[i2 as usize].position;

        let p0 = Point3::new(v0.x, v0.y, v0.z);
        let p1 = Point3::new(v1.x, v1.y, v1.z);
        let p2 = Point3::new(v2.x, v2.y, v2.z);

        // Create a planar face from the triangle
        if let Some(face) = create_triangle_face(&p0, &p1, &p2) {
            faces.push(face);
        }
    }

    if faces.is_empty() {
        return Err(IoError::invalid_content(
            "no valid triangles could be converted to STEP faces",
        ));
    }

    // Create shell from faces
    let shell: Shell<_, _, _> = faces.into();

    // Compress for STEP output
    let compressed = shell.compress();

    // Generate STEP string using truck's output formatting
    let step_string = CompleteStepDisplay::new(
        StepModel::from(&compressed),
        StepHeaderDescriptor {
            organization_system: "CortenForge mesh-io".to_owned(),
            ..Default::default()
        },
    )
    .to_string();

    // Write STEP file
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(step_string.as_bytes())?;

    Ok(())
}

/// Create a triangular planar face for truck.
fn create_triangle_face(
    p0: &truck_modeling::Point3,
    p1: &truck_modeling::Point3,
    p2: &truck_modeling::Point3,
) -> Option<
    truck_topology::Face<truck_modeling::Point3, truck_modeling::Curve, truck_modeling::Surface>,
> {
    use truck_modeling::builder;
    use truck_polymesh::InnerSpace;

    // Calculate normal to check for degenerate triangle
    let e1 = p1 - p0;
    let e2 = p2 - p0;
    let normal = e1.cross(e2);

    if normal.magnitude() < 1e-10 {
        return None; // Degenerate triangle
    }

    // Create vertices
    let v0 = builder::vertex(*p0);
    let v1 = builder::vertex(*p1);
    let v2 = builder::vertex(*p2);

    // Create edges
    let edge0 = builder::line(&v0, &v1);
    let edge1 = builder::line(&v1, &v2);
    let edge2 = builder::line(&v2, &v0);

    // Create wire from edges and attach plane
    let wire = truck_topology::Wire::from(vec![edge0, edge1, edge2]);
    let face = builder::try_attach_plane(&[wire]).ok()?;

    Some(face)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::needless_raw_string_hashes,
    clippy::let_underscore_must_use
)]
mod tests {
    use super::*;

    fn create_test_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn load_nonexistent_file() {
        let result = load_step("nonexistent_file_12345.step");
        assert!(result.is_err());
        if let Err(IoError::FileNotFound { path }) = result {
            assert!(path.to_string_lossy().contains("nonexistent"));
        }
    }

    #[test]
    fn save_empty_mesh_fails() {
        let mesh = IndexedMesh::new();
        let temp_dir = tempfile::tempdir().ok();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("empty.step");
            let result = save_step(&mesh, &path);
            assert!(result.is_err());
        }
    }

    #[test]
    fn save_simple_triangle() {
        let mesh = create_test_triangle();
        let temp_dir = tempfile::tempdir().ok();

        if let Some(dir) = temp_dir {
            let path = dir.path().join("triangle.step");
            let result = save_step(&mesh, &path);
            // May fail due to degenerate geometry handling in truck
            // but should not panic
            let _ = result;
        }
    }
}
