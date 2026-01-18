//! Automatic print orientation optimization.
//!
//! Provides functionality to find the best orientation for printing
//! a mesh, minimizing supports and improving print quality.

use mesh_types::{IndexedMesh, MeshTopology, Vector3};
use nalgebra::{Unit, UnitQuaternion};

use crate::config::PrinterConfig;

/// Result of orientation analysis.
#[derive(Debug, Clone)]
pub struct OrientationResult {
    /// The recommended rotation to apply to the mesh.
    pub rotation: UnitQuaternion<f64>,

    /// Estimated support volume after rotation (mm³).
    pub support_volume: f64,

    /// Estimated overhang area after rotation (mm²).
    pub overhang_area: f64,

    /// Score for this orientation (lower is better).
    pub score: f64,
}

impl OrientationResult {
    /// Create a new orientation result.
    #[must_use]
    pub fn new(rotation: UnitQuaternion<f64>, support_volume: f64, overhang_area: f64) -> Self {
        // Score combines support volume and overhang area
        let score = support_volume + overhang_area * 0.1;
        Self {
            rotation,
            support_volume,
            overhang_area,
            score,
        }
    }

    /// Identity orientation (no rotation).
    #[must_use]
    pub fn identity() -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            support_volume: 0.0,
            overhang_area: 0.0,
            score: 0.0,
        }
    }
}

/// Find the optimal print orientation for a mesh.
///
/// Analyzes multiple orientations and returns the one that minimizes
/// support material requirements.
///
/// # Arguments
///
/// * `mesh` - The mesh to analyze
/// * `config` - Printer configuration
/// * `samples` - Number of orientation samples to test (more = slower but better)
///
/// # Example
///
/// ```
/// use mesh_types::IndexedMesh;
/// use mesh_printability::{find_optimal_orientation, PrinterConfig};
///
/// let mesh = IndexedMesh::new();
/// // let result = find_optimal_orientation(&mesh, &PrinterConfig::fdm_default(), 12);
/// ```
#[must_use]
pub fn find_optimal_orientation(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    samples: usize,
) -> OrientationResult {
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return OrientationResult::identity();
    }

    let samples = samples.max(6); // Minimum 6 samples (cube faces)
    let mut best = OrientationResult::identity();
    best.score = f64::INFINITY;

    // Generate sample orientations
    let orientations = generate_sample_orientations(samples);

    for rotation in orientations {
        let result = evaluate_orientation(mesh, config, rotation);
        if result.score < best.score {
            best = result;
        }
    }

    best
}

/// Generate sample orientations for testing.
fn generate_sample_orientations(samples: usize) -> Vec<UnitQuaternion<f64>> {
    let mut orientations = Vec::with_capacity(samples);

    // Always include identity and major axis alignments
    orientations.push(UnitQuaternion::identity());

    // 90° rotations around X axis
    orientations.push(UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
        std::f64::consts::FRAC_PI_2,
    ));
    orientations.push(UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
        std::f64::consts::PI,
    ));
    orientations.push(UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
        -std::f64::consts::FRAC_PI_2,
    ));

    // 90° rotations around Y axis
    orientations.push(UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
        std::f64::consts::FRAC_PI_2,
    ));
    orientations.push(UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
        -std::f64::consts::FRAC_PI_2,
    ));

    // Add more samples if requested
    if samples > 6 {
        // 45° rotations
        let angles = [
            std::f64::consts::FRAC_PI_4,
            -std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_4 * 3.0,
            -std::f64::consts::FRAC_PI_4 * 3.0,
        ];

        for angle in &angles {
            if orientations.len() >= samples {
                break;
            }
            orientations.push(UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                *angle,
            ));
        }

        for angle in &angles {
            if orientations.len() >= samples {
                break;
            }
            orientations.push(UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
                *angle,
            ));
        }
    }

    // Fibonacci sphere sampling for more orientations
    if samples > orientations.len() {
        let remaining = samples - orientations.len();
        let golden_ratio = (1.0 + 5.0_f64.sqrt()) / 2.0;

        for i in 0..remaining {
            let theta = 2.0 * std::f64::consts::PI * (i as f64) / golden_ratio;
            let phi = (1.0 - 2.0 * (i as f64 + 0.5) / remaining as f64).acos();

            let axis = Vector3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());

            if let Some(unit_axis) = Unit::try_new(axis, 1e-10) {
                orientations.push(UnitQuaternion::from_axis_angle(
                    &unit_axis,
                    std::f64::consts::FRAC_PI_2,
                ));
            }
        }
    }

    orientations
}

/// Evaluate a single orientation.
fn evaluate_orientation(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    rotation: UnitQuaternion<f64>,
) -> OrientationResult {
    let up = Vector3::new(0.0, 0.0, 1.0);
    let max_angle_rad = config.max_overhang_angle.to_radians();

    let mut total_overhang_area = 0.0;
    let rotation_matrix = rotation.to_rotation_matrix();

    let num_triangles = mesh.face_count();
    for i in 0..num_triangles {
        let face = mesh.faces[i];
        let idx0 = face[0] as usize;
        let idx1 = face[1] as usize;
        let idx2 = face[2] as usize;

        if idx0 >= mesh.vertices.len() || idx1 >= mesh.vertices.len() || idx2 >= mesh.vertices.len()
        {
            continue;
        }

        let v0 = mesh.vertices[idx0].position;
        let v1 = mesh.vertices[idx1].position;
        let v2 = mesh.vertices[idx2].position;

        // Compute face normal
        let edge1 = Vector3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let edge2 = Vector3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let normal = edge1.cross(&edge2);

        let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
        if len < 1e-10 {
            continue;
        }

        let normal = Vector3::new(normal.x / len, normal.y / len, normal.z / len);

        // Rotate normal
        let rotated_normal = rotation_matrix * normal;

        // Check if face is pointing downward (overhang)
        let dot = rotated_normal.dot(&up);
        let angle = dot.acos();

        if dot < 0.0 {
            let overhang_angle = std::f64::consts::PI - angle;
            if overhang_angle > max_angle_rad {
                let area = len / 2.0;
                total_overhang_area += area;
            }
        }
    }

    // Estimate support volume (rough approximation)
    let support_volume = total_overhang_area * 5.0;

    OrientationResult::new(rotation, support_volume, total_overhang_area)
}

/// Apply an orientation result to transform a mesh.
///
/// Returns a new mesh with the rotation applied.
#[must_use]
pub fn apply_orientation(mesh: &IndexedMesh, orientation: &OrientationResult) -> IndexedMesh {
    let rotation = orientation.rotation.to_rotation_matrix();
    let mut result = mesh.clone();

    for vertex in &mut result.vertices {
        let v = Vector3::new(vertex.position.x, vertex.position.y, vertex.position.z);
        let rotated = rotation * v;
        vertex.position.x = rotated.x;
        vertex.position.y = rotated.y;
        vertex.position.z = rotated.z;

        // Also rotate vertex normals if present
        if let Some(ref mut normal) = vertex.attributes.normal {
            let n = Vector3::new(normal.x, normal.y, normal.z);
            let rotated_n = rotation * n;
            normal.x = rotated_n.x;
            normal.y = rotated_n.y;
            normal.z = rotated_n.z;
        }
    }

    result
}

/// Place mesh on the build plate (move so minimum Z is 0).
#[must_use]
pub fn place_on_build_plate(mesh: &IndexedMesh) -> IndexedMesh {
    if mesh.vertices.is_empty() {
        return mesh.clone();
    }

    let min_z = mesh
        .vertices
        .iter()
        .map(|v| v.position.z)
        .fold(f64::INFINITY, f64::min);

    let mut result = mesh.clone();
    for vertex in &mut result.vertices {
        vertex.position.z -= min_z;
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_simple_mesh() -> IndexedMesh {
        let vertices = vec![
            Vertex::from_coords(0.0, 0.0, 0.0),
            Vertex::from_coords(10.0, 0.0, 0.0),
            Vertex::from_coords(5.0, 10.0, 0.0),
            Vertex::from_coords(5.0, 5.0, 10.0),
        ];

        // Tetrahedron
        let faces = vec![[0, 1, 2], [0, 3, 1], [1, 3, 2], [2, 3, 0]];

        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_identity_orientation() {
        let result = OrientationResult::identity();
        assert!((result.score - 0.0).abs() < f64::EPSILON);
        assert!((result.support_volume - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_generate_sample_orientations() {
        let samples = generate_sample_orientations(6);
        assert_eq!(samples.len(), 6);

        let samples = generate_sample_orientations(12);
        assert!(samples.len() >= 12);
    }

    #[test]
    fn test_find_optimal_orientation_empty() {
        let mesh = IndexedMesh::new();
        let config = PrinterConfig::fdm_default();
        let result = find_optimal_orientation(&mesh, &config, 6);
        assert!((result.score - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_find_optimal_orientation() {
        let mesh = create_simple_mesh();
        let config = PrinterConfig::fdm_default();
        let result = find_optimal_orientation(&mesh, &config, 12);

        // Should find some orientation
        assert!(result.score >= 0.0);
    }

    #[test]
    fn test_apply_orientation() {
        let mesh = create_simple_mesh();
        let rotation = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            std::f64::consts::FRAC_PI_2,
        );
        let orientation = OrientationResult::new(rotation, 0.0, 0.0);

        let rotated = apply_orientation(&mesh, &orientation);
        assert_eq!(rotated.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_place_on_build_plate() {
        let vertices = vec![
            Vertex::from_coords(0.0, 0.0, 5.0),
            Vertex::from_coords(10.0, 0.0, 10.0),
            Vertex::from_coords(5.0, 10.0, 15.0),
        ];
        let faces = vec![[0, 1, 2]];
        let mesh = IndexedMesh::from_parts(vertices, faces);

        let placed = place_on_build_plate(&mesh);

        let min_z = placed
            .vertices
            .iter()
            .map(|v| v.position.z)
            .fold(f64::INFINITY, f64::min);

        assert!(min_z.abs() < 1e-10);
    }

    #[test]
    fn test_place_on_build_plate_empty() {
        let mesh = IndexedMesh::new();
        let placed = place_on_build_plate(&mesh);
        assert!(placed.vertices.is_empty());
    }

    #[test]
    fn test_orientation_score() {
        let rotation = UnitQuaternion::identity();
        let result = OrientationResult::new(rotation, 100.0, 50.0);

        // Score = support_volume + overhang_area * 0.1
        let expected = 100.0 + 50.0 * 0.1;
        assert!((result.score - expected).abs() < f64::EPSILON);
    }
}
