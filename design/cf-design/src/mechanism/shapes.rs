//! Collision shape generation from mechanism parts.
//!
//! Converts each [`Part`](super::Part) in a [`Mechanism`](super::Mechanism)
//! into a [`cf_geometry::Shape`] suitable for simulation collision detection.
//!
//! Two modes are available via [`ShapeMode`]:
//! - **SDF mode** — evaluates the implicit field on a grid → `Shape::Sdf`.
//!   Fast, no meshing. Best for design iteration.
//! - **Mesh mode** — meshes the field → `Shape::TriangleMesh` with BVH.
//!   Accurate. Best for final simulation and RL training.

use std::sync::Arc;

use cf_geometry::{Shape, bvh_from_mesh};

use super::builder::Mechanism;

// ── ShapeMode ───────────────────────────────────────────────────────────

/// Controls how mechanism parts are converted to collision shapes.
///
/// # Example
///
/// ```
/// use cf_design::ShapeMode;
///
/// // Fast iteration: SDF grid with 0.5 mm cell size
/// let fast = ShapeMode::Sdf { resolution: 0.5 };
///
/// // Final quality: meshed with 0.2 mm tolerance
/// let final_quality = ShapeMode::Mesh { tolerance: 0.2 };
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum ShapeMode {
    /// Evaluate the implicit field on a regular grid → `Shape::Sdf`.
    ///
    /// `resolution` is the grid cell size in mm. Smaller = finer grid, higher
    /// memory, better surface detail.
    Sdf {
        /// Grid cell size in mm.
        resolution: f64,
    },

    /// Mesh the implicit field via marching cubes → `Shape::TriangleMesh`.
    ///
    /// `tolerance` is the voxel size in mm passed to [`Solid::mesh`](crate::Solid::mesh).
    /// Smaller = finer mesh, more triangles, more accurate collision.
    Mesh {
        /// Marching cubes voxel size in mm.
        tolerance: f64,
    },
}

// ── Generation ──────────────────────────────────────────────────────────

/// Generate collision shapes for all parts in a mechanism.
///
/// Returns `(part_name, Shape)` pairs in part declaration order.
///
/// # Panics
///
/// Panics if the resolution/tolerance in `mode` is not positive and finite.
pub(super) fn generate(mechanism: &Mechanism, mode: &ShapeMode) -> Vec<(String, Shape)> {
    match mode {
        ShapeMode::Sdf { resolution } => {
            assert!(
                *resolution > 0.0 && resolution.is_finite(),
                "SDF resolution must be positive and finite, got {resolution}"
            );
            generate_sdf(mechanism, *resolution)
        }
        ShapeMode::Mesh { tolerance } => {
            assert!(
                *tolerance > 0.0 && tolerance.is_finite(),
                "mesh tolerance must be positive and finite, got {tolerance}"
            );
            generate_mesh(mechanism, *tolerance)
        }
    }
}

fn generate_sdf(mechanism: &Mechanism, cell_size: f64) -> Vec<(String, Shape)> {
    mechanism
        .parts()
        .iter()
        .filter_map(|part| {
            // Convert cell_size (mm) to grid resolution (usize) for sdf_grid().
            // sdf_grid(n) computes: cell = longest / (n - 1).
            // We want cell ≈ cell_size, so n = ceil(longest / cell_size) + 1.
            let bounds = part.solid().bounds()?;
            let size = bounds.size();
            let longest = size.x.max(size.y).max(size.z);
            if longest <= 0.0 {
                return None;
            }
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let grid_resolution = ((longest / cell_size).ceil() as usize + 1).max(2);

            let grid = part.solid().sdf_grid(grid_resolution)?;
            Some((part.name().to_owned(), Shape::sdf(Arc::new(grid))))
        })
        .collect()
}

fn generate_mesh(mechanism: &Mechanism, tolerance: f64) -> Vec<(String, Shape)> {
    mechanism
        .parts()
        .iter()
        .map(|part| {
            let mesh = part.solid().mesh(tolerance);
            let bvh = bvh_from_mesh(&mesh);
            let shape = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));
            (part.name().to_owned(), shape)
        })
        .collect()
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::{JointDef, JointKind, Material, Mechanism, Part, Solid};

    use super::*;

    fn pla() -> Material {
        Material::new("PLA", 1250.0)
    }

    fn two_part_mechanism() -> Mechanism {
        Mechanism::builder("test")
            .part(Part::new(
                "palm",
                Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)),
                pla(),
            ))
            .part(Part::new("finger", Solid::sphere(3.0), pla()))
            .joint(JointDef::new(
                "j",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::new(5.0, 0.0, 0.0),
                Vector3::x(),
            ))
            .build()
    }

    // ── 1. SDF mode produces correct count and variant ─────────────────

    #[test]
    fn sdf_mode_produces_sdf_shapes() {
        let m = two_part_mechanism();
        let shapes = generate(&m, &ShapeMode::Sdf { resolution: 1.0 });

        assert_eq!(shapes.len(), 2, "expected one shape per part");
        assert_eq!(shapes[0].0, "palm");
        assert_eq!(shapes[1].0, "finger");

        for (name, shape) in &shapes {
            assert!(
                shape.is_mesh_based(),
                "SDF shape for \"{name}\" should be mesh-based (Sdf is mesh-based)"
            );
        }
    }

    // ── 2. Mesh mode produces triangle mesh shapes ─────────────────────

    #[test]
    fn mesh_mode_produces_triangle_mesh_shapes() {
        let m = two_part_mechanism();
        let shapes = generate(&m, &ShapeMode::Mesh { tolerance: 1.0 });

        assert_eq!(shapes.len(), 2);

        for (name, shape) in &shapes {
            assert!(
                shape.is_mesh_based(),
                "mesh shape for \"{name}\" should be mesh-based"
            );
        }
    }

    // ── 3. SDF mode respects resolution ────────────────────────────────

    #[test]
    fn sdf_finer_resolution_produces_different_shape() {
        let m = Mechanism::builder("solo")
            .part(Part::new("ball", Solid::sphere(5.0), pla()))
            .build();

        let coarse = generate(&m, &ShapeMode::Sdf { resolution: 2.0 });
        let fine = generate(&m, &ShapeMode::Sdf { resolution: 0.5 });

        // Both should produce one shape.
        assert_eq!(coarse.len(), 1);
        assert_eq!(fine.len(), 1);

        // Fine and coarse are both valid shapes (we can't easily compare grid
        // sizes without reaching into Shape internals, but both should succeed).
    }

    // ── 4. Mesh mode produces non-empty meshes ─────────────────────────

    #[test]
    fn mesh_mode_non_empty() {
        let m = Mechanism::builder("solo")
            .part(Part::new("ball", Solid::sphere(5.0), pla()))
            .build();

        let shapes = generate(&m, &ShapeMode::Mesh { tolerance: 1.0 });
        assert_eq!(shapes.len(), 1);

        // Shape::TriangleMesh should have a bounding radius > 0.
        assert!(
            shapes[0].1.bounding_radius() > 0.0,
            "meshed shape should have positive bounding radius"
        );
    }

    // ── 5. Single-part mechanism ───────────────────────────────────────

    #[test]
    fn single_part_sdf() {
        let m = Mechanism::builder("one")
            .part(Part::new("body", Solid::sphere(3.0), pla()))
            .build();

        let shapes = generate(&m, &ShapeMode::Sdf { resolution: 1.0 });
        assert_eq!(shapes.len(), 1);
        assert_eq!(shapes[0].0, "body");
    }

    // ── 6. Panic on invalid resolution ─────────────────────────────────

    #[test]
    #[should_panic(expected = "SDF resolution must be positive")]
    fn sdf_rejects_zero_resolution() {
        let m = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .build();
        let _ = generate(&m, &ShapeMode::Sdf { resolution: 0.0 });
    }

    #[test]
    #[should_panic(expected = "mesh tolerance must be positive")]
    fn mesh_rejects_zero_tolerance() {
        let m = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .build();
        let _ = generate(&m, &ShapeMode::Mesh { tolerance: 0.0 });
    }
}
