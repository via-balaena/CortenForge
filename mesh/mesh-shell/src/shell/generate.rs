//! Shell generation algorithm.
//!
//! Generates a printable shell from the inner surface by creating
//! an outer surface and connecting them with a rim.

// Mesh processing uses u32 indices; truncation would only occur for meshes with >4B vertices
// which exceeds practical limits.
#![allow(clippy::cast_possible_truncation)]

use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_repair::{MeshAdjacency, flip_winding, remove_unreferenced_vertices, weld_vertices};
use mesh_types::IndexedMesh;
use nalgebra::Vector3;
use tracing::{debug, info, warn};

use super::rim::{generate_rim, generate_rim_for_sdf_shell};
use super::validation::{ShellValidationResult, validate_shell};
use crate::error::{ShellError, ShellResult};

/// Method for generating the outer surface of the shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WallGenerationMethod {
    /// Normal-based offset (fast, but may have inconsistent thickness at corners).
    ///
    /// Each vertex is offset along its normal by the wall thickness.
    /// Pros: Fast, preserves vertex correspondence with inner surface.
    /// Cons: Wall thickness varies at corners (thinner at convex, thicker at concave).
    #[default]
    Normal,

    /// SDF-based offset (robust, consistent wall thickness).
    ///
    /// Computes a signed distance field and extracts an isosurface at the
    /// desired wall thickness distance. This ensures consistent wall thickness
    /// regardless of surface curvature.
    /// Pros: Consistent wall thickness, handles concave regions correctly.
    /// Cons: Slower, may change vertex count, requires additional memory.
    ///
    /// **Open inputs (with boundary edges) automatically fall back to the
    /// `Normal` method.** The SDF level set of an open mesh has its own
    /// boundary that cannot be reliably stitched to the inner mesh's boundary
    /// — bowl semantics require the 1:1 vertex correspondence Normal provides.
    Sdf,
}

impl std::fmt::Display for WallGenerationMethod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Normal => write!(f, "normal"),
            Self::Sdf => write!(f, "sdf"),
        }
    }
}

/// Parameters for shell generation.
#[derive(Debug, Clone)]
pub struct ShellParams {
    /// Uniform wall thickness in mm.
    pub wall_thickness_mm: f64,
    /// Minimum acceptable wall thickness.
    pub min_thickness_mm: f64,
    /// Whether to validate the shell after generation.
    pub validate_after_generation: bool,
    /// Method for generating the outer surface.
    pub wall_generation_method: WallGenerationMethod,
    /// Voxel size for SDF-based wall generation (mm).
    /// Smaller values give more detail but use more memory.
    /// Only used when `wall_generation_method` is `Sdf`.
    pub sdf_voxel_size_mm: f64,
}

impl Default for ShellParams {
    fn default() -> Self {
        Self {
            wall_thickness_mm: 2.5,
            min_thickness_mm: 1.5,
            validate_after_generation: true,
            wall_generation_method: WallGenerationMethod::Normal,
            sdf_voxel_size_mm: 0.5,
        }
    }
}

impl ShellParams {
    /// Create params optimized for high-quality output with consistent wall thickness.
    ///
    /// Uses SDF-based wall generation for consistent thickness at corners.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            wall_generation_method: WallGenerationMethod::Sdf,
            sdf_voxel_size_mm: 0.3,
            ..Self::default()
        }
    }

    /// Create params optimized for fast generation.
    ///
    /// Uses normal-based offset which is faster but may have inconsistent thickness.
    #[must_use]
    pub fn fast() -> Self {
        Self {
            wall_generation_method: WallGenerationMethod::Normal,
            validate_after_generation: false,
            ..Self::default()
        }
    }
}

/// Result of shell generation.
#[derive(Debug)]
pub struct ShellGenerationResult {
    /// Number of inner surface vertices.
    pub inner_vertex_count: usize,
    /// Number of outer surface vertices.
    pub outer_vertex_count: usize,
    /// Number of rim faces generated.
    pub rim_face_count: usize,
    /// Total face count.
    pub total_face_count: usize,
    /// Boundary loop size (number of edges).
    pub boundary_size: usize,
    /// Validation result (if validation was performed).
    pub validation: Option<ShellValidationResult>,
    /// Wall generation method used.
    pub wall_method: WallGenerationMethod,
}

/// Generate a printable shell from the inner surface.
///
/// Creates an outer surface using the configured method (normal or SDF-based),
/// then connects inner and outer at boundaries with a rim.
///
/// # Arguments
/// * `inner_mesh` - The inner surface mesh
/// * `params` - Shell generation parameters
///
/// # Returns
/// A tuple of (shell mesh, generation result).
///
/// # Errors
///
/// Returns an error if the mesh is empty or shell generation fails.
pub fn generate_shell(
    inner_mesh: &IndexedMesh,
    params: &ShellParams,
) -> ShellResult<(IndexedMesh, ShellGenerationResult)> {
    if inner_mesh.faces.is_empty() {
        return Err(ShellError::empty_mesh());
    }

    info!(
        "Generating shell with thickness={:.2}mm, method={}",
        params.wall_thickness_mm, params.wall_generation_method
    );

    match params.wall_generation_method {
        WallGenerationMethod::Normal => generate_shell_normal(inner_mesh, params),
        WallGenerationMethod::Sdf => generate_shell_sdf(inner_mesh, params),
    }
}

/// Generate shell using normal-based offset (fast method).
#[allow(clippy::unnecessary_wraps)] // Result for consistency with generate_shell_sdf
fn generate_shell_normal(
    inner_mesh: &IndexedMesh,
    params: &ShellParams,
) -> ShellResult<(IndexedMesh, ShellGenerationResult)> {
    let n = inner_mesh.vertices.len();
    let mut shell = IndexedMesh::new();

    // Step 1: Compute vertex normals
    let normals = compute_vertex_normals(inner_mesh);

    // Step 2: Copy inner vertices
    for vertex in &inner_mesh.vertices {
        shell.vertices.push(*vertex);
    }

    // Step 3: Generate outer vertices by offsetting along normals
    for (i, vertex) in inner_mesh.vertices.iter().enumerate() {
        let normal = normals.get(i).copied().unwrap_or_else(Vector3::z);
        let outer_pos = vertex + normal * params.wall_thickness_mm;

        shell.vertices.push(outer_pos);
    }

    debug!("Generated {} inner + {} outer vertices", n, n);

    // Step 4: Copy inner faces (reversed winding so normal points inward)
    for face in &inner_mesh.faces {
        shell.faces.push([face[0], face[2], face[1]]);
    }

    // Step 5: Generate outer faces with offset indices (original winding for outward normals)
    let n32 = n as u32;
    for face in &inner_mesh.faces {
        shell
            .faces
            .push([face[0] + n32, face[1] + n32, face[2] + n32]);
    }

    let inner_face_count = inner_mesh.faces.len();
    debug!(
        "Added {} inner + {} outer faces",
        inner_face_count, inner_face_count
    );

    // Step 6: Find boundary edges and generate rim
    let (rim_faces, boundary_size) = generate_rim(inner_mesh, n);

    let rim_face_count = rim_faces.len();
    for face in rim_faces {
        shell.faces.push(face);
    }

    info!(
        "Shell generation complete: {} vertices, {} faces",
        shell.vertices.len(),
        shell.faces.len()
    );

    // Optionally validate the generated shell
    let validation = if params.validate_after_generation {
        let validation_result = validate_shell(&shell);
        if !validation_result.is_printable() {
            warn!(
                "Generated shell has {} validation issue(s)",
                validation_result.issue_count()
            );
        }
        Some(validation_result)
    } else {
        None
    };

    let result = ShellGenerationResult {
        inner_vertex_count: n,
        outer_vertex_count: n,
        rim_face_count,
        total_face_count: shell.faces.len(),
        boundary_size,
        validation,
        wall_method: WallGenerationMethod::Normal,
    };

    Ok((shell, result))
}

/// Generate shell using SDF-based offset for consistent wall thickness.
fn generate_shell_sdf(
    inner_mesh: &IndexedMesh,
    params: &ShellParams,
) -> ShellResult<(IndexedMesh, ShellGenerationResult)> {
    // Precondition: SDF wall generation requires a closed (watertight) input.
    // An open input's SDF level set has its own boundary that cannot be reliably
    // lidded against the inner mesh's boundary. Fall back to Normal method,
    // which produces 1:1 inner/outer vertex correspondence + rim quads at the
    // open boundary (bowl semantics).
    let inner_boundary = MeshAdjacency::build(&inner_mesh.faces).boundary_edge_count();
    if inner_boundary > 0 {
        warn!(
            "SDF wall generation: input has {} boundary edges (open mesh); \
             falling back to Normal method since SDF level set cannot lid an open mesh",
            inner_boundary
        );
        return generate_shell_normal(inner_mesh, params);
    }

    let inner_vertex_count = inner_mesh.vertices.len();

    // Step 1: Create outer mesh using SDF offset
    let offset_config = OffsetConfig::default().with_resolution(params.sdf_voxel_size_mm);

    let mut outer_mesh = match offset_mesh(inner_mesh, params.wall_thickness_mm, &offset_config) {
        Ok(m) => m,
        Err(e) => {
            warn!("SDF offset failed: {:?}, falling back to normal method", e);
            return generate_shell_normal(inner_mesh, params);
        }
    };

    // mesh-offset's marching cubes returns vertex-soup (vertex_count == 3 ×
    // face_count); weld + compact so the outer is a proper indexed mesh
    // before concatenation. Sub-voxel epsilon merges only MC's exactly
    // coincident edge-interpolated duplicates, never legitimately distinct
    // vertices.
    let weld_eps = params.sdf_voxel_size_mm * 1e-3;
    let merged = weld_vertices(&mut outer_mesh, weld_eps);
    let orphans = remove_unreferenced_vertices(&mut outer_mesh);
    debug!(
        "Welded {} duplicate verts in outer mesh ({} orphans compacted)",
        merged, orphans
    );

    // mesh-offset 0.7.x's marching cubes produces inside-out winding (commits
    // 9+10 platform truth). Flip every outer face so normals point outward
    // from the wall material; otherwise the assembled shell has
    // signed_volume < 0 + is_inside_out == true and slicers see backfaces.
    flip_winding(&mut outer_mesh);

    let outer_vertex_count = outer_mesh.vertices.len();
    debug!(
        "Created outer surface: {} vertices, {} faces",
        outer_vertex_count,
        outer_mesh.faces.len()
    );

    // Step 2: Combine inner and outer surfaces into shell
    let mut shell = IndexedMesh::new();

    // Add inner vertices (with reversed normals conceptually - handled by face winding)
    for vertex in &inner_mesh.vertices {
        shell.vertices.push(*vertex);
    }

    // Add outer vertices (offset by inner count)
    let inner_count = inner_mesh.vertices.len() as u32;
    for vertex in &outer_mesh.vertices {
        shell.vertices.push(*vertex);
    }

    // Add inner faces (reversed winding so normal points inward)
    for face in &inner_mesh.faces {
        shell.faces.push([face[0], face[2], face[1]]);
    }

    // Add outer faces (keep original winding, offset indices)
    for face in &outer_mesh.faces {
        shell.faces.push([
            face[0] + inner_count,
            face[1] + inner_count,
            face[2] + inner_count,
        ]);
    }

    // Step 3: Generate rim connecting inner and outer boundaries
    let (rim_faces, boundary_size) =
        generate_rim_for_sdf_shell(inner_mesh, &outer_mesh, inner_count as usize);

    let rim_face_count = rim_faces.len();
    for face in rim_faces {
        shell.faces.push(face);
    }

    info!(
        "SDF shell generation complete: {} vertices, {} faces (rim: {})",
        shell.vertices.len(),
        shell.faces.len(),
        rim_face_count
    );

    // Optionally validate the generated shell
    let validation = if params.validate_after_generation {
        let validation_result = validate_shell(&shell);
        if !validation_result.is_printable() {
            warn!(
                "Generated shell has {} validation issue(s)",
                validation_result.issue_count()
            );
        }
        Some(validation_result)
    } else {
        None
    };

    let result = ShellGenerationResult {
        inner_vertex_count,
        outer_vertex_count,
        rim_face_count,
        total_face_count: shell.faces.len(),
        boundary_size,
        validation,
        wall_method: WallGenerationMethod::Sdf,
    };

    Ok((shell, result))
}

/// Generate a shell without automatic validation.
///
/// This is equivalent to calling `generate_shell` with `validate_after_generation = false`.
///
/// # Errors
///
/// Returns an error if the mesh is empty or shell generation fails.
pub fn generate_shell_no_validation(
    inner_mesh: &IndexedMesh,
    params: &ShellParams,
) -> ShellResult<(IndexedMesh, ShellGenerationResult)> {
    let mut params = params.clone();
    params.validate_after_generation = false;
    generate_shell(inner_mesh, &params)
}

/// Compute vertex normals from face normals.
fn compute_vertex_normals(mesh: &IndexedMesh) -> Vec<Vector3<f64>> {
    let vertex_count = mesh.vertices.len();
    let mut normals: Vec<Vector3<f64>> = vec![Vector3::zeros(); vertex_count];
    let mut counts: Vec<usize> = vec![0; vertex_count];

    // Build adjacency to find faces per vertex
    let adjacency = MeshAdjacency::build(&mesh.faces);

    // For each face, compute face normal and add to each vertex
    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let face_normal = edge1.cross(&edge2);

        // Skip degenerate triangles
        let len = face_normal.norm();
        if len < 1e-10 {
            continue;
        }

        let face_normal = face_normal / len;

        // Add to each vertex of this face
        for &vertex_idx in face {
            let vi = vertex_idx as usize;
            normals[vi] += face_normal;
            counts[vi] += 1;
        }
    }

    // Normalize
    for (normal, count) in normals.iter_mut().zip(counts.iter()) {
        if *count > 0 {
            let len = normal.norm();
            if len > 1e-10 {
                *normal /= len;
            } else {
                *normal = Vector3::z();
            }
        } else {
            *normal = Vector3::z();
        }
    }

    let _ = adjacency; // Used conceptually for vertex-face relationships
    normals
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    fn create_closed_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 10.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 10.0));
        // Bottom (-z)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Top (+z)
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        // Front (-y)
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // Back (+y)
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // Left (-x)
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        // Right (+x)
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);
        mesh
    }

    fn create_open_box() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 10.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 10.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 10.0));

        // Bottom
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
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
        // Top is OPEN

        mesh
    }

    #[test]
    fn test_shell_params_default() {
        let params = ShellParams::default();
        assert!((params.wall_thickness_mm - 2.5).abs() < 1e-6);
        assert!((params.min_thickness_mm - 1.5).abs() < 1e-6);
        assert!(params.validate_after_generation);
        assert_eq!(params.wall_generation_method, WallGenerationMethod::Normal);
    }

    #[test]
    fn test_shell_params_high_quality() {
        let params = ShellParams::high_quality();
        assert_eq!(params.wall_generation_method, WallGenerationMethod::Sdf);
        assert!(params.sdf_voxel_size_mm < 0.5);
    }

    #[test]
    fn test_shell_params_fast() {
        let params = ShellParams::fast();
        assert_eq!(params.wall_generation_method, WallGenerationMethod::Normal);
        assert!(!params.validate_after_generation);
    }

    #[test]
    fn test_wall_generation_method_display() {
        assert_eq!(format!("{}", WallGenerationMethod::Normal), "normal");
        assert_eq!(format!("{}", WallGenerationMethod::Sdf), "sdf");
    }

    #[test]
    fn test_generate_shell_doubles_vertices() {
        let inner = create_open_box();
        let params = ShellParams::fast();

        let (shell, result) = generate_shell(&inner, &params).expect("should succeed");

        // Should have 2x vertices (inner + outer) for normal method
        assert_eq!(shell.vertices.len(), inner.vertices.len() * 2);
        assert_eq!(result.inner_vertex_count, inner.vertices.len());
        assert_eq!(result.outer_vertex_count, inner.vertices.len());
        assert_eq!(result.wall_method, WallGenerationMethod::Normal);
    }

    #[test]
    fn test_shell_has_more_faces() {
        let inner = create_open_box();
        let params = ShellParams::fast();

        let (shell, result) = generate_shell(&inner, &params).expect("should succeed");

        // Should have inner + outer + rim faces
        assert!(shell.faces.len() > inner.faces.len() * 2);
        assert!(result.rim_face_count > 0);
    }

    #[test]
    fn test_generate_shell_empty_mesh() {
        let empty = IndexedMesh::new();
        let params = ShellParams::default();

        let result = generate_shell(&empty, &params);
        assert!(result.is_err());
    }

    #[test]
    fn test_compute_vertex_normals() {
        let mesh = create_open_box();
        let normals = compute_vertex_normals(&mesh);

        // Should have one normal per vertex
        assert_eq!(normals.len(), mesh.vertices.len());

        // All normals should be unit length
        for normal in &normals {
            let len = normal.norm();
            assert!(
                (len - 1.0).abs() < 0.01,
                "Normal should be unit length, got {len}"
            );
        }
    }

    #[test]
    fn test_sdf_shell_on_open_input_falls_back_to_normal() {
        // The SDF code path's precondition: closed input only. Open inputs
        // fall back to Normal because the SDF level set has its own boundary
        // that can't be reliably lidded.
        let inner = create_open_box();
        let mut params = ShellParams::high_quality();
        // Coarsen the voxel size so the test runs quickly even though the
        // fallback short-circuits SDF computation entirely.
        params.sdf_voxel_size_mm = 1.0;

        let (_shell, result) = generate_shell(&inner, &params).expect("fallback should succeed");

        // Fallback fired: result reports Normal, not Sdf.
        assert_eq!(result.wall_method, WallGenerationMethod::Normal);
        // Normal method gives 1:1 vertex correspondence on outer.
        assert_eq!(result.outer_vertex_count, inner.vertices.len());
        // Open box → 4 boundary edges at the top → rim quads close them.
        assert!(result.rim_face_count > 0);
    }

    #[test]
    fn test_sdf_shell_open_input_validation_is_printable() {
        // Bug 3 fallback: an open input requesting SDF walls should still
        // produce a printable shell (via the Normal-method fallback path)
        // rather than the previous non-printable garbage.
        let inner = create_open_box();
        let mut params = ShellParams::high_quality();
        params.sdf_voxel_size_mm = 1.0;

        let (_shell, result) = generate_shell(&inner, &params).expect("fallback should succeed");

        let validation = result
            .validation
            .as_ref()
            .expect("validation requested by high_quality()");
        assert!(
            validation.is_printable(),
            "open-input fallback should produce a printable shell; got {validation:?}"
        );
    }

    #[test]
    fn test_sdf_shell_on_closed_cube_is_printable() {
        // Primary regression: post-fix, .high_quality() on a closed input
        // produces a printable shell. Pre-fix would have failed with
        // is_watertight=false (vertex-soup) + is_inside_out=true.
        let inner = create_closed_cube();
        let mut params = ShellParams::high_quality();
        params.sdf_voxel_size_mm = 1.0;

        let (_shell, result) =
            generate_shell(&inner, &params).expect("SDF generation should succeed");

        // Closed input should NOT trigger the open-input fallback.
        assert_eq!(result.wall_method, WallGenerationMethod::Sdf);

        let validation = result
            .validation
            .as_ref()
            .expect("validation requested by high_quality()");
        assert!(
            validation.is_watertight,
            "closed cube SDF shell should be watertight; got {validation:?}"
        );
        assert!(
            validation.is_manifold,
            "closed cube SDF shell should be manifold; got {validation:?}"
        );
        assert!(
            validation.is_printable(),
            "closed cube SDF shell should be printable; got {validation:?}"
        );
    }

    #[test]
    fn test_sdf_shell_outer_winding_outward_after_flip() {
        // Bug 2 anchor: post-fix, the outer's per-face flip means the
        // assembled shell's signed_volume is positive (outward-pointing
        // normals via the divergence-theorem integral).
        let inner = create_closed_cube();
        let mut params = ShellParams::high_quality();
        params.sdf_voxel_size_mm = 1.0;

        let (shell, _result) =
            generate_shell(&inner, &params).expect("SDF generation should succeed");

        let report = mesh_repair::validate_mesh(&shell);
        assert!(
            !report.is_inside_out,
            "shell should not be inside-out after per-face flip; signed_volume reported as inside-out"
        );
    }

    #[test]
    fn test_sdf_shell_outer_is_welded_no_soup() {
        // Bug 1 anchor: post-fix, the outer is welded into a proper indexed
        // mesh, so vertex_count is much smaller than 3 × outer_face_count
        // (which would be the case for unwelded MC vertex-soup).
        let inner = create_closed_cube();
        let mut params = ShellParams::high_quality();
        params.sdf_voxel_size_mm = 1.0;

        let (shell, result) =
            generate_shell(&inner, &params).expect("SDF generation should succeed");

        // For a closed input, no rim faces; total = inner + outer.
        let outer_face_count = result.total_face_count - inner.faces.len() - result.rim_face_count;
        // Welded: vert/face ratio is roughly 0.5 for closed manifolds (each
        // vert shared by ~6 faces). Soup would give 3.0. Use a generous
        // threshold (< 4.0) to allow MC mesh quality variation.
        assert!(
            shell.vertices.len() < 4 * outer_face_count,
            "outer appears unwelded (vertex-soup signature): {} verts vs {} outer faces; \
             expected verts < 4 × outer_face_count",
            shell.vertices.len(),
            outer_face_count
        );
    }
}
