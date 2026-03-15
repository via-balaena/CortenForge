//! Mesh helpers for mesh-types domain.

use cf_geometry::{Bounded, IndexedMesh};
use nalgebra::Point3;

/// Helper function to create a unit cube mesh.
///
/// Creates a cube from (0,0,0) to (1,1,1) with outward-facing normals.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
///
/// let cube = unit_cube();
/// assert_eq!(cube.vertex_count(), 8);
/// assert_eq!(cube.face_count(), 12);
/// ```
#[must_use]
pub fn unit_cube() -> IndexedMesh {
    let mut mesh = IndexedMesh::with_capacity(8, 12);

    // 8 vertices of the cube
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0)); // 0
    mesh.vertices.push(Point3::new(1.0, 0.0, 0.0)); // 1
    mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
    mesh.vertices.push(Point3::new(0.0, 1.0, 0.0)); // 3
    mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4
    mesh.vertices.push(Point3::new(1.0, 0.0, 1.0)); // 5
    mesh.vertices.push(Point3::new(1.0, 1.0, 1.0)); // 6
    mesh.vertices.push(Point3::new(0.0, 1.0, 1.0)); // 7

    // 12 triangles (2 per face), CCW winding when viewed from outside
    // Bottom face (z=0)
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);
    // Top face (z=1)
    mesh.faces.push([4, 5, 6]);
    mesh.faces.push([4, 6, 7]);
    // Front face (y=0)
    mesh.faces.push([0, 1, 5]);
    mesh.faces.push([0, 5, 4]);
    // Back face (y=1)
    mesh.faces.push([3, 7, 6]);
    mesh.faces.push([3, 6, 2]);
    // Left face (x=0)
    mesh.faces.push([0, 4, 7]);
    mesh.faces.push([0, 7, 3]);
    // Right face (x=1)
    mesh.faces.push([1, 2, 6]);
    mesh.faces.push([1, 6, 5]);

    mesh
}

/// Translate a mesh so its minimum Z is at zero.
///
/// Useful for placing meshes on a build plate.
pub fn place_on_z_zero(mesh: &mut IndexedMesh) {
    let bounds = mesh.aabb();
    if !bounds.is_empty() {
        let offset = -bounds.min.z;
        for v in &mut mesh.vertices {
            v.z += offset;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_cube_basic() {
        let cube = unit_cube();
        assert_eq!(cube.vertex_count(), 8);
        assert_eq!(cube.face_count(), 12);
    }

    #[test]
    fn unit_cube_volume() {
        let cube = unit_cube();
        let vol = cube.signed_volume();
        assert!(
            (vol - 1.0).abs() < 1e-10,
            "Unit cube volume should be 1.0, got {vol}"
        );
    }

    #[test]
    fn unit_cube_surface_area() {
        let cube = unit_cube();
        let area = cube.surface_area();
        assert!(
            (area - 6.0).abs() < 1e-10,
            "Unit cube surface area should be 6.0, got {area}"
        );
    }

    #[test]
    fn unit_cube_not_inside_out() {
        let cube = unit_cube();
        assert!(!cube.is_inside_out());
    }

    #[test]
    fn unit_cube_bounds() {
        let cube = unit_cube();
        let aabb = cube.aabb();
        assert!(aabb.min.x.abs() < f64::EPSILON);
        assert!((aabb.max.x - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn place_on_z_zero_works() {
        let mut cube = unit_cube();
        cube.translate(nalgebra::Vector3::new(0.0, 0.0, 5.0));
        place_on_z_zero(&mut cube);
        let aabb = cube.aabb();
        assert!(aabb.min.z.abs() < f64::EPSILON);
    }
}
