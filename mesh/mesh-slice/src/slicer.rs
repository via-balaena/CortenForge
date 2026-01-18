//! Main slicing implementation.
//!
//! Generates layer-by-layer slices of a mesh for 3D printing preview.

// Mesh indices and layer counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]

use mesh_measure::cross_section;
use mesh_types::{IndexedMesh, Point3};
use tracing::{debug, info};

use crate::layer::{Contour, Layer, LayerBounds};
use crate::params::SliceParams;
use crate::result::SliceResult;

/// Slice a mesh into layers for 3D printing preview.
///
/// # Arguments
///
/// * `mesh` - The mesh to slice
/// * `params` - Slicing parameters
///
/// # Returns
///
/// A [`SliceResult`] containing all layers and statistics.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_slice::{slice_mesh, SliceParams};
///
/// let cube = unit_cube();
/// let result = slice_mesh(&cube, &SliceParams::default());
/// assert!(result.layer_count > 0);
/// ```
#[must_use]
pub fn slice_mesh(mesh: &IndexedMesh, params: &SliceParams) -> SliceResult {
    // Find Z bounds
    let (min_z, max_z) = find_z_bounds(mesh, &params.direction);
    let total_height = max_z - min_z;

    if total_height <= 0.0 || mesh.vertices.is_empty() {
        return SliceResult::empty(params.clone());
    }

    info!(
        total_height = format!("{:.2}", total_height),
        layer_height = params.layer_height,
        "Starting mesh slicing"
    );

    // Calculate layer heights
    // Use integer layer count to avoid while-float comparison
    let remaining_height = total_height - params.first_layer_height;
    let layer_count = if remaining_height > 0.0 && params.layer_height > 0.0 {
        (remaining_height / params.layer_height).ceil() as usize + 1
    } else {
        1
    };

    let mut z_heights = Vec::with_capacity(layer_count);
    z_heights.push((min_z + params.first_layer_height, params.first_layer_height));

    let base_z = min_z + params.first_layer_height;
    let max_z_threshold = params.layer_height.mul_add(0.5, max_z);
    for i in 1..layer_count {
        let z = (i as f64).mul_add(params.layer_height, base_z);
        if z <= max_z_threshold {
            z_heights.push((z, params.layer_height));
        }
    }

    debug!(layer_count = z_heights.len(), "Calculated layer heights");

    // Generate layers
    let mut layers = Vec::with_capacity(z_heights.len());
    let mut max_area = 0.0;
    let mut max_area_layer = 0;
    let mut max_perimeter = 0.0;
    let mut max_perimeter_layer = 0;
    let mut total_print_time = 0.0;
    let mut total_filament = 0.0;

    for (index, (z, thickness)) in z_heights.iter().enumerate() {
        let layer = generate_layer(mesh, index, *z, *thickness, params);

        if layer.area > max_area {
            max_area = layer.area;
            max_area_layer = index;
        }
        if layer.perimeter > max_perimeter {
            max_perimeter = layer.perimeter;
            max_perimeter_layer = index;
        }

        total_print_time += layer.print_time;
        total_filament += layer.filament_length;

        layers.push(layer);
    }

    // Calculate filament volume (assuming 1.75mm diameter)
    let filament_diameter: f64 = 1.75;
    let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);
    let filament_volume = total_filament * filament_area;

    info!(
        layers = layers.len(),
        print_time_min = format!("{:.1}", total_print_time / 60.0),
        "Slicing complete"
    );

    SliceResult {
        layers,
        total_height,
        layer_count: z_heights.len(),
        estimated_print_time: total_print_time / 60.0, // Convert to minutes
        estimated_filament_length: total_filament,
        estimated_filament_volume: filament_volume,
        max_area_layer,
        max_perimeter_layer,
        params: params.clone(),
    }
}

/// Generate a slice preview at a specific height.
///
/// # Arguments
///
/// * `mesh` - The mesh to slice
/// * `z` - The Z height to slice at
/// * `params` - Slicing parameters
///
/// # Returns
///
/// A [`Layer`] at the specified height.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_slice::{slice_preview, SliceParams};
///
/// let cube = unit_cube();
/// let layer = slice_preview(&cube, 0.5, &SliceParams::default());
/// assert!(layer.area > 0.0);
/// ```
#[must_use]
pub fn slice_preview(mesh: &IndexedMesh, z: f64, params: &SliceParams) -> Layer {
    generate_layer(mesh, 0, z, params.layer_height, params)
}

// ============================================================================
// Internal helper functions
// ============================================================================

fn find_z_bounds(mesh: &IndexedMesh, direction: &mesh_types::Vector3<f64>) -> (f64, f64) {
    if mesh.vertices.is_empty() {
        return (0.0, 0.0);
    }

    let dir = direction.normalize();
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for v in &mesh.vertices {
        let z = v.position.coords.dot(&dir);
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }

    (min_z, max_z)
}

fn generate_layer(
    mesh: &IndexedMesh,
    index: usize,
    z: f64,
    thickness: f64,
    params: &SliceParams,
) -> Layer {
    // Get cross-section at this Z height
    let plane_point = Point3::new(0.0, 0.0, z);
    let section = cross_section(mesh, plane_point, params.direction);

    // Convert cross-section to contours
    let contours = extract_contours(&section);
    let island_count = contours.iter().filter(|c| c.is_outer).count();

    // Calculate bounds
    let bounds = calculate_layer_bounds(&contours);

    // Calculate print time estimate
    let print_time = estimate_layer_print_time(&contours, params);

    // Calculate filament usage
    let filament_length = estimate_filament_usage(&contours, thickness, params);

    Layer {
        index,
        z_height: z,
        thickness,
        area: section.area,
        perimeter: section.perimeter,
        contours,
        print_time,
        filament_length,
        island_count,
        bounds,
    }
}

fn extract_contours(section: &mesh_measure::CrossSection) -> Vec<Contour> {
    if section.points.is_empty() {
        return Vec::new();
    }

    // For simplicity, treat the entire cross-section as one contour
    // In a full implementation, we would separate inner/outer contours
    let perimeter = section.perimeter;
    let area = section.area;

    vec![Contour {
        points: section.points.clone(),
        area,
        perimeter,
        is_outer: true,
        centroid: section.centroid,
    }]
}

fn calculate_layer_bounds(contours: &[Contour]) -> LayerBounds {
    if contours.is_empty() {
        return LayerBounds::default();
    }

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for contour in contours {
        for p in &contour.points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
    }

    LayerBounds {
        min_x,
        max_x,
        min_y,
        max_y,
    }
}

fn estimate_layer_print_time(contours: &[Contour], params: &SliceParams) -> f64 {
    if contours.is_empty() {
        return 0.0;
    }

    let mut time = 0.0;

    for contour in contours {
        // Perimeter time
        let perimeter_passes = params.perimeters as f64;
        let perimeter_length = contour.perimeter * perimeter_passes;
        if params.perimeter_speed > 0.0 {
            time += perimeter_length / params.perimeter_speed;
        }

        // Infill time (simplified: assume infill is proportional to area)
        if params.infill_density > 0.0 && params.infill_speed > 0.0 {
            // Approximate infill path length based on area and density
            let infill_spacing = params.perimeter_width / params.infill_density;
            let infill_length = contour.area / infill_spacing;
            time += infill_length / params.infill_speed;
        }
    }

    // Add travel time (rough estimate: 10% of print time)
    time *= 1.1;

    time
}

fn estimate_filament_usage(contours: &[Contour], layer_height: f64, params: &SliceParams) -> f64 {
    if contours.is_empty() {
        return 0.0;
    }

    let mut volume = 0.0;

    for contour in contours {
        // Perimeter volume
        let perimeter_passes = params.perimeters as f64;
        let perimeter_length = contour.perimeter * perimeter_passes;
        let perimeter_cross_section = params.perimeter_width * layer_height;
        volume += perimeter_length * perimeter_cross_section;

        // Infill volume
        if params.infill_density > 0.0 {
            let infill_volume = contour.area * layer_height * params.infill_density;
            volume += infill_volume;
        }
    }

    volume *= params.extrusion_multiplier;

    // Convert volume to filament length (assuming 1.75mm diameter)
    let filament_diameter: f64 = 1.75;
    let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);
    volume / filament_area
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{unit_cube, Vertex};

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // Create a 10x10x10 cube
        let vertices = [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (10.0, 10.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (10.0, 0.0, 10.0),
            (10.0, 10.0, 10.0),
            (0.0, 10.0, 10.0),
        ];

        for (x, y, z) in vertices {
            mesh.vertices.push(Vertex::from_coords(x, y, z));
        }

        // 12 triangles for 6 faces
        let faces = [
            // Bottom
            [0, 1, 2],
            [0, 2, 3],
            // Top
            [4, 6, 5],
            [4, 7, 6],
            // Front
            [0, 5, 1],
            [0, 4, 5],
            // Back
            [2, 7, 3],
            [2, 6, 7],
            // Left
            [0, 3, 7],
            [0, 7, 4],
            // Right
            [1, 5, 6],
            [1, 6, 2],
        ];

        for f in faces {
            mesh.faces.push(f);
        }

        mesh
    }

    #[test]
    fn test_slice_mesh() {
        let mesh = create_test_cube();
        let result = slice_mesh(&mesh, &SliceParams::default());

        // 10mm height with 0.3mm first layer and 0.2mm layers
        assert!(result.layer_count > 0);
        assert!((result.total_height - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_slice_preview() {
        let mesh = create_test_cube();
        let layer = slice_preview(&mesh, 5.0, &SliceParams::default());

        // At z=5, should intersect the cube
        assert!(layer.area > 0.0);
        assert!(layer.perimeter > 0.0);
    }

    #[test]
    fn test_empty_mesh_slice() {
        let mesh = IndexedMesh::new();
        let result = slice_mesh(&mesh, &SliceParams::default());

        assert_eq!(result.layer_count, 0);
        assert!((result.total_height).abs() < 0.001);
    }

    #[test]
    fn test_unit_cube_slice() {
        let cube = unit_cube();
        let result = slice_mesh(&cube, &SliceParams::default());

        // Unit cube has height of 1.0mm, so should have some layers
        assert!(result.layer_count >= 1);
    }

    #[test]
    fn test_layer_bounds() {
        let mesh = create_test_cube();
        let layer = slice_preview(&mesh, 5.0, &SliceParams::default());

        // Bounds should be reasonable for a 10x10 cube
        assert!(layer.bounds.width() <= 10.5);
        assert!(layer.bounds.height() <= 10.5);
    }

    #[test]
    fn test_stats() {
        let mesh = create_test_cube();
        let result = slice_mesh(&mesh, &SliceParams::default());
        let stats = result.stats();

        // For a cube, all internal layers should have similar areas
        assert!(stats.max_area >= stats.min_area);
    }
}
