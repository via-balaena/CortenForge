//! Lattice generation functions.
//!
//! This module performs geometric computations where numeric conversions between
//! floating-point and integer types are fundamental to the algorithm (grid indexing,
//! voxel counts, etc.). These conversions are intentional and safe within the
//! expected bounds of lattice dimensions.

// Allow numeric casts that are inherent to geometry algorithms
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_sign_loss)]

use crate::beam::BeamLatticeData;
use crate::error::LatticeError;
use crate::marching_cubes::extract_isosurface;
use crate::params::LatticeParams;
use crate::strut::{combine_struts, generate_strut};
use crate::tpms::{density_to_threshold, diamond, gyroid, schwarz_p};
use crate::types::{LatticeResult, LatticeType};
use hashbrown::HashMap;
use mesh_types::{IndexedMesh, MeshTopology};
use nalgebra::Point3;

/// Generates a lattice structure within a bounding box.
///
/// # Arguments
///
/// * `params` - Lattice configuration parameters
/// * `bounds` - Bounding box as (`min_corner`, `max_corner`)
///
/// # Returns
///
/// A `LatticeResult` containing the mesh and statistics, or an error.
///
/// # Errors
///
/// Returns [`LatticeError`] if:
/// - The bounding box is invalid
/// - Parameters are invalid
/// - Generation fails
///
/// # Examples
///
/// ```
/// use mesh_lattice::{generate_lattice, LatticeParams};
/// use nalgebra::Point3;
///
/// let params = LatticeParams::cubic(5.0);
/// let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(50.0, 50.0, 50.0));
/// let result = generate_lattice(&params, bounds);
///
/// match result {
///     Ok(lattice) => println!("Generated {} vertices", lattice.vertex_count()),
///     Err(e) => eprintln!("Error: {}", e),
/// }
/// ```
pub fn generate_lattice(
    params: &LatticeParams,
    bounds: (Point3<f64>, Point3<f64>),
) -> Result<LatticeResult, LatticeError> {
    // Validate bounds
    let (min, max) = bounds;
    if min.x >= max.x || min.y >= max.y || min.z >= max.z {
        return Err(LatticeError::InvalidBounds {
            min: [min.x, min.y, min.z],
            max: [max.x, max.y, max.z],
        });
    }

    // Validate parameters
    params.validate()?;

    // Generate based on lattice type
    match params.lattice_type {
        LatticeType::Cubic => generate_cubic_lattice(params, bounds),
        LatticeType::OctetTruss => generate_octet_truss_lattice(params, bounds),
        LatticeType::Gyroid => generate_tpms_lattice(params, bounds, "gyroid"),
        LatticeType::SchwarzP => generate_tpms_lattice(params, bounds, "schwarz_p"),
        LatticeType::Diamond => generate_tpms_lattice(params, bounds, "diamond"),
        LatticeType::Voronoi => generate_voronoi_lattice(params, bounds),
    }
}

/// Generates a cubic lattice.
#[allow(clippy::too_many_lines)] // Lattice generation is inherently sequential
#[allow(clippy::unnecessary_wraps)] // Consistent API with generate_lattice
fn generate_cubic_lattice(
    params: &LatticeParams,
    bounds: (Point3<f64>, Point3<f64>),
) -> Result<LatticeResult, LatticeError> {
    let (min, max) = bounds;
    let size = max - min;
    let cell_size = params.cell_size;

    // Calculate number of cells in each dimension
    let cells_x = (size.x / cell_size).ceil() as usize;
    let cells_y = (size.y / cell_size).ceil() as usize;
    let cells_z = (size.z / cell_size).ceil() as usize;
    let cell_count = cells_x * cells_y * cells_z;

    let radius = params.strut_thickness / 2.0;

    // For beam data export
    let mut beam_data = if params.preserve_beam_data {
        Some(BeamLatticeData::new(radius))
    } else {
        None
    };

    // Vertex deduplication map
    let mut vertex_map: HashMap<[i64; 3], u32> = HashMap::new();
    let quantize = |p: Point3<f64>| -> [i64; 3] {
        let scale = 1e6;
        [
            (p.x * scale) as i64,
            (p.y * scale) as i64,
            (p.z * scale) as i64,
        ]
    };

    let mut struts = Vec::new();
    let mut total_length = 0.0;

    // Generate struts along each axis
    for iz in 0..=cells_z {
        for iy in 0..=cells_y {
            for ix in 0..=cells_x {
                let x = (ix as f64).mul_add(cell_size, min.x);
                let y = (iy as f64).mul_add(cell_size, min.y);
                let z = (iz as f64).mul_add(cell_size, min.z);
                let node = Point3::new(x, y, z);

                // Get density at this point
                let density = params.density_at(node);

                // Skip low-density regions
                if density < 0.05 {
                    continue;
                }

                // Scale radius by density
                let local_radius = radius * density.sqrt();

                // Strut along X
                if ix < cells_x {
                    let end = Point3::new(x + cell_size, y, z);
                    if params.trim_to_bounds && (end.x > max.x || end.y > max.y || end.z > max.z) {
                        // Skip struts that extend outside bounds
                    } else {
                        if let Some(strut) = generate_strut(node, end, local_radius) {
                            total_length += cell_size;
                            struts.push(strut);
                        }

                        // Record beam data
                        if let Some(ref mut data) = beam_data {
                            let key_start = quantize(node);
                            let key_end = quantize(end);

                            let v1 = *vertex_map
                                .entry(key_start)
                                .or_insert_with(|| data.add_vertex(node));
                            let v2 = *vertex_map
                                .entry(key_end)
                                .or_insert_with(|| data.add_vertex(end));
                            data.add_beam_with_radius(v1, v2, local_radius);
                        }
                    }
                }

                // Strut along Y
                if iy < cells_y {
                    let end = Point3::new(x, y + cell_size, z);
                    if params.trim_to_bounds && (end.x > max.x || end.y > max.y || end.z > max.z) {
                        // Skip
                    } else {
                        if let Some(strut) = generate_strut(node, end, local_radius) {
                            total_length += cell_size;
                            struts.push(strut);
                        }

                        if let Some(ref mut data) = beam_data {
                            let key_start = quantize(node);
                            let key_end = quantize(end);

                            let v1 = *vertex_map
                                .entry(key_start)
                                .or_insert_with(|| data.add_vertex(node));
                            let v2 = *vertex_map
                                .entry(key_end)
                                .or_insert_with(|| data.add_vertex(end));
                            data.add_beam_with_radius(v1, v2, local_radius);
                        }
                    }
                }

                // Strut along Z
                if iz < cells_z {
                    let end = Point3::new(x, y, z + cell_size);
                    if params.trim_to_bounds && (end.x > max.x || end.y > max.y || end.z > max.z) {
                        // Skip
                    } else {
                        if let Some(strut) = generate_strut(node, end, local_radius) {
                            total_length += cell_size;
                            struts.push(strut);
                        }

                        if let Some(ref mut data) = beam_data {
                            let key_start = quantize(node);
                            let key_end = quantize(end);

                            let v1 = *vertex_map
                                .entry(key_start)
                                .or_insert_with(|| data.add_vertex(node));
                            let v2 = *vertex_map
                                .entry(key_end)
                                .or_insert_with(|| data.add_vertex(end));
                            data.add_beam_with_radius(v1, v2, local_radius);
                        }
                    }
                }
            }
        }
    }

    let mesh = combine_struts(struts.into_iter());

    // Estimate actual density
    let bounds_volume = size.x * size.y * size.z;
    let strut_volume = estimate_strut_volume(&mesh, radius);
    let actual_density = (strut_volume / bounds_volume).min(1.0);

    let mut result =
        LatticeResult::new(mesh, actual_density, cell_count).with_strut_length(total_length);

    if let Some(data) = beam_data {
        result = result.with_beam_data(data);
    }

    Ok(result)
}

/// Generates an octet-truss lattice.
#[allow(clippy::unnecessary_wraps)] // Consistent API with generate_lattice
fn generate_octet_truss_lattice(
    params: &LatticeParams,
    bounds: (Point3<f64>, Point3<f64>),
) -> Result<LatticeResult, LatticeError> {
    let (min, max) = bounds;
    let size = max - min;
    let cell_size = params.cell_size;
    let half_cell = cell_size / 2.0;

    let cells_x = (size.x / cell_size).ceil() as usize;
    let cells_y = (size.y / cell_size).ceil() as usize;
    let cells_z = (size.z / cell_size).ceil() as usize;
    let cell_count = cells_x * cells_y * cells_z;

    let radius = params.strut_thickness / 2.0;

    let mut struts = Vec::new();
    let mut total_length = 0.0;

    // Octet-truss: corners connect to face centers
    for iz in 0..cells_z {
        for iy in 0..cells_y {
            for ix in 0..cells_x {
                let x = (ix as f64).mul_add(cell_size, min.x);
                let y = (iy as f64).mul_add(cell_size, min.y);
                let z = (iz as f64).mul_add(cell_size, min.z);

                // Cell corners
                let corners = [
                    Point3::new(x, y, z),
                    Point3::new(x + cell_size, y, z),
                    Point3::new(x + cell_size, y + cell_size, z),
                    Point3::new(x, y + cell_size, z),
                    Point3::new(x, y, z + cell_size),
                    Point3::new(x + cell_size, y, z + cell_size),
                    Point3::new(x + cell_size, y + cell_size, z + cell_size),
                    Point3::new(x, y + cell_size, z + cell_size),
                ];

                // Cell center
                let center = Point3::new(x + half_cell, y + half_cell, z + half_cell);

                // Get density at center
                let density = params.density_at(center);
                if density < 0.05 {
                    continue;
                }

                let local_radius = radius * density.sqrt();

                // Connect all corners to center (8 struts)
                for corner in &corners {
                    if let Some(strut) = generate_strut(*corner, center, local_radius) {
                        total_length += (*corner - center).norm();
                        struts.push(strut);
                    }
                }

                // Edge struts along the 12 edges
                let edges = [
                    (0, 1),
                    (1, 2),
                    (2, 3),
                    (3, 0), // Bottom face
                    (4, 5),
                    (5, 6),
                    (6, 7),
                    (7, 4), // Top face
                    (0, 4),
                    (1, 5),
                    (2, 6),
                    (3, 7), // Vertical edges
                ];

                for (i, j) in edges {
                    if let Some(strut) = generate_strut(corners[i], corners[j], local_radius) {
                        total_length += (corners[i] - corners[j]).norm();
                        struts.push(strut);
                    }
                }
            }
        }
    }

    let mesh = combine_struts(struts.into_iter());

    let bounds_volume = size.x * size.y * size.z;
    let strut_volume = estimate_strut_volume(&mesh, radius);
    let actual_density = (strut_volume / bounds_volume).min(1.0);

    Ok(LatticeResult::new(mesh, actual_density, cell_count).with_strut_length(total_length))
}

/// Generates a TPMS-based lattice with specified type.
#[allow(clippy::unnecessary_wraps)] // Consistent API with generate_lattice
fn generate_tpms_lattice(
    params: &LatticeParams,
    bounds: (Point3<f64>, Point3<f64>),
    tpms_type: &str,
) -> Result<LatticeResult, LatticeError> {
    let (min, max) = bounds;
    let size = max - min;
    let cell_size = params.cell_size;

    let cells_x = (size.x / cell_size).ceil() as usize;
    let cells_y = (size.y / cell_size).ceil() as usize;
    let cells_z = (size.z / cell_size).ceil() as usize;
    let cell_count = cells_x * cells_y * cells_z;

    // Create the TPMS function
    let tpms_fn: Box<dyn Fn(Point3<f64>) -> f64> = match tpms_type.to_lowercase().as_str() {
        "gyroid" => Box::new(move |p| gyroid(p, cell_size)),
        "schwarz_p" | "schwarzp" => Box::new(move |p| schwarz_p(p, cell_size)),
        "diamond" => Box::new(move |p| diamond(p, cell_size)),
        _ => Box::new(move |p| gyroid(p, cell_size)),
    };

    // Calculate threshold from density
    let threshold = density_to_threshold(params.density, tpms_type);

    // Create shell SDF (thickened surface)
    let wall_thickness = params.wall_thickness;
    let half_thickness = wall_thickness / 2.0;
    let shell_sdf = move |p: Point3<f64>| -> f64 {
        let value = tpms_fn(p);
        // Shell: |f(x)| - half_thickness = 0 means on surface
        // We want negative inside the shell
        (value - threshold).abs() - half_thickness
    };

    // Calculate resolution based on bounds
    let max_dim = size.x.max(size.y).max(size.z);
    let samples_per_cell = params.resolution;
    let total_resolution = ((max_dim / cell_size) * samples_per_cell as f64).ceil() as usize;
    let resolution = total_resolution.clamp(10, 200);

    // Extract isosurface
    let mesh = extract_isosurface(&shell_sdf, bounds, resolution, 0.0);

    // Estimate actual density
    let bounds_volume = size.x * size.y * size.z;
    let mesh_volume = estimate_mesh_volume(&mesh);
    let actual_density = (mesh_volume / bounds_volume).clamp(0.0, 1.0);

    Ok(LatticeResult::new(mesh, actual_density, cell_count))
}

/// Generates a Voronoi-style lattice (currently simplified as perturbed cubic).
#[allow(clippy::unnecessary_wraps)] // Consistent API with generate_lattice
fn generate_voronoi_lattice(
    params: &LatticeParams,
    bounds: (Point3<f64>, Point3<f64>),
) -> Result<LatticeResult, LatticeError> {
    let (min, max) = bounds;
    let size = max - min;
    let cell_size = params.cell_size;

    let cells_x = (size.x / cell_size).ceil() as usize;
    let cells_y = (size.y / cell_size).ceil() as usize;
    let cells_z = (size.z / cell_size).ceil() as usize;
    let cell_count = cells_x * cells_y * cells_z;

    let radius = params.strut_thickness / 2.0;

    // Perturbation amount
    let perturb = cell_size * 0.2;

    // Simple LCG for deterministic random
    let mut seed: u64 = 42;
    let mut random = || -> f64 {
        seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
        ((seed >> 16) & 0x7fff) as f64 / 32768.0 - 0.5
    };

    // Generate perturbed grid points
    let mut nodes: Vec<Point3<f64>> = Vec::new();
    for iz in 0..=cells_z {
        for iy in 0..=cells_y {
            for ix in 0..=cells_x {
                let x = (ix as f64).mul_add(cell_size, min.x) + random() * perturb;
                let y = (iy as f64).mul_add(cell_size, min.y) + random() * perturb;
                let z = (iz as f64).mul_add(cell_size, min.z) + random() * perturb;

                // Clamp to bounds
                let x = x.clamp(min.x, max.x);
                let y = y.clamp(min.y, max.y);
                let z = z.clamp(min.z, max.z);

                nodes.push(Point3::new(x, y, z));
            }
        }
    }

    // Connect nodes in a grid pattern (similar to cubic, but with perturbed positions)
    let mut struts = Vec::new();
    let mut total_length = 0.0;

    let idx = |ix: usize, iy: usize, iz: usize| -> usize {
        iz * (cells_y + 1) * (cells_x + 1) + iy * (cells_x + 1) + ix
    };

    for iz in 0..=cells_z {
        for iy in 0..=cells_y {
            for ix in 0..=cells_x {
                let node_idx = idx(ix, iy, iz);
                let node = nodes[node_idx];

                let density = params.density_at(node);
                if density < 0.05 {
                    continue;
                }

                let local_radius = radius * density.sqrt();

                // Connect to neighbors
                if ix < cells_x {
                    let neighbor_idx = idx(ix + 1, iy, iz);
                    let neighbor = nodes[neighbor_idx];
                    if let Some(strut) = generate_strut(node, neighbor, local_radius) {
                        total_length += (neighbor - node).norm();
                        struts.push(strut);
                    }
                }

                if iy < cells_y {
                    let neighbor_idx = idx(ix, iy + 1, iz);
                    let neighbor = nodes[neighbor_idx];
                    if let Some(strut) = generate_strut(node, neighbor, local_radius) {
                        total_length += (neighbor - node).norm();
                        struts.push(strut);
                    }
                }

                if iz < cells_z {
                    let neighbor_idx = idx(ix, iy, iz + 1);
                    let neighbor = nodes[neighbor_idx];
                    if let Some(strut) = generate_strut(node, neighbor, local_radius) {
                        total_length += (neighbor - node).norm();
                        struts.push(strut);
                    }
                }
            }
        }
    }

    let mesh = combine_struts(struts.into_iter());

    let bounds_volume = size.x * size.y * size.z;
    let strut_volume = estimate_strut_volume(&mesh, radius);
    let actual_density = (strut_volume / bounds_volume).min(1.0);

    Ok(LatticeResult::new(mesh, actual_density, cell_count).with_strut_length(total_length))
}

/// Estimates strut volume from mesh (approximate).
fn estimate_strut_volume(mesh: &IndexedMesh, radius: f64) -> f64 {
    use std::f64::consts::PI;

    // Count triangles, estimate strut count
    let triangle_count = mesh.face_count();
    // Each strut has ~24 triangles (6 sides × 2 + 12 caps)
    let estimated_struts = triangle_count as f64 / 24.0;

    // Average strut length estimation based on vertex spread
    if mesh.is_empty() {
        return 0.0;
    }

    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);

    for v in &mesh.vertices {
        let p = &v.position;
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }

    let diagonal = (max - min).norm();
    let avg_strut_length = diagonal / (estimated_struts.cbrt() + 1.0);

    // Volume = n × π × r² × L
    estimated_struts * PI * radius * radius * avg_strut_length
}

/// Estimates mesh volume using signed volume method.
fn estimate_mesh_volume(mesh: &IndexedMesh) -> f64 {
    let vertices = &mesh.vertices;
    let faces = &mesh.faces;

    let mut volume = 0.0_f64;

    for face in faces {
        let i0 = face[0] as usize;
        let i1 = face[1] as usize;
        let i2 = face[2] as usize;

        if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
            continue;
        }

        let v0 = &vertices[i0].position;
        let v1 = &vertices[i1].position;
        let v2 = &vertices[i2].position;

        // Signed volume of tetrahedron from origin
        volume += v0.coords.dot(&v1.coords.cross(&v2.coords));
    }

    (volume / 6.0).abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    #[test]
    fn test_generate_cubic_lattice() {
        let params = LatticeParams::cubic(5.0);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(25.0, 25.0, 25.0));
        let result = generate_lattice(&params, bounds);

        assert!(result.is_ok());
        let lattice = result.unwrap();
        assert!(lattice.vertex_count() > 0);
        assert!(lattice.cell_count > 0);
        assert!(lattice.total_strut_length.is_some());
    }

    #[test]
    fn test_generate_octet_truss_lattice() {
        let params = LatticeParams::octet_truss(10.0);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(30.0, 30.0, 30.0));
        let result = generate_lattice(&params, bounds);

        assert!(result.is_ok());
        let lattice = result.unwrap();
        assert!(lattice.vertex_count() > 0);
    }

    #[test]
    fn test_generate_gyroid_lattice() {
        let params = LatticeParams::gyroid(10.0)
            .with_density(0.3)
            .with_resolution(8);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(30.0, 30.0, 30.0));
        let result = generate_lattice(&params, bounds);

        assert!(result.is_ok());
        let lattice = result.unwrap();
        assert!(lattice.vertex_count() > 0);
    }

    #[test]
    fn test_generate_voronoi_lattice() {
        let params = LatticeParams::voronoi(8.0);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(24.0, 24.0, 24.0));
        let result = generate_lattice(&params, bounds);

        assert!(result.is_ok());
        let lattice = result.unwrap();
        assert!(lattice.vertex_count() > 0);
    }

    #[test]
    fn test_invalid_bounds() {
        let params = LatticeParams::cubic(5.0);
        let bounds = (Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
        let result = generate_lattice(&params, bounds);

        assert!(matches!(result, Err(LatticeError::InvalidBounds { .. })));
    }

    #[test]
    fn test_beam_data_export() {
        let params = LatticeParams::cubic(5.0).with_beam_export(true);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(15.0, 15.0, 15.0));
        let result = generate_lattice(&params, bounds);

        assert!(result.is_ok());
        let lattice = result.unwrap();
        assert!(lattice.beam_data.is_some());

        let beam_data = lattice.beam_data.unwrap();
        assert!(beam_data.vertex_count() > 0);
        assert!(beam_data.beam_count() > 0);
    }

    #[test]
    fn test_small_bounds() {
        let params = LatticeParams::cubic(10.0);
        let bounds = (Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 5.0, 5.0));
        let result = generate_lattice(&params, bounds);

        // Should still work, just with fewer cells
        assert!(result.is_ok());
    }

    #[test]
    fn test_estimate_mesh_volume() {
        // Create a simple tetrahedron
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0)),
            Vertex::new(Point3::new(0.0, 1.0, 0.0)),
            Vertex::new(Point3::new(0.0, 0.0, 1.0)),
        ];
        let faces = vec![
            [0, 1, 2], // Base
            [0, 1, 3], // Side 1
            [1, 2, 3], // Side 2
            [2, 0, 3], // Side 3
        ];
        let mesh = IndexedMesh::from_parts(vertices, faces);

        let volume = estimate_mesh_volume(&mesh);
        // Tetrahedron volume = 1/6
        assert!((volume - 1.0 / 6.0).abs() < 0.1);
    }
}
