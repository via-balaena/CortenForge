//! [`CutPoints::Root`]: the mesher's cut points on the SDF's own zero set.
//!
//! The default, [`CutPoints::Interpolated`], puts each cut where the line through its lattice edge's two
//! samples crosses zero, which sits off a curved or kinked SDF's zero set. `Root` locates it on the SDF,
//! and the warp reads the located cut too. These tests pin that every boundary vertex then lies on the zero
//! set (a sphere, and a shell whose distance is kinked inside the solid), that the output keeps
//! Labelle–Shewchuk's angle bounds, and that a non-finite value met while locating a cut is reported.

// A meshing failure surfaces as the test's panic.
#![allow(clippy::expect_used)]

use nalgebra::{Point3, Vector3};
use sim_soft::sdf_bridge::{
    Aabb3, CutPoints, MeshingError, MeshingHints, Sdf, SdfMeshedTetMesh, SphereSdf,
};
use sim_soft::{Mesh, Vec3};

/// Labelle–Shewchuk's Theorem 1 with the mesher's α's: every dihedral angle in [9.3171°, 161.6432°].
const DIHEDRAL_BOUNDS_DEG: (f64, f64) = (9.3171, 161.6432);

/// A spherical shell about the origin, `|‖p‖ − radius| − half`: its distance has a kink at `radius`, inside
/// the solid, so a lattice edge that crosses its surface can straddle the kink.
struct Shell {
    radius: f64,
    half: f64,
}

impl Sdf for Shell {
    fn eval(&self, p: Point3<f64>) -> f64 {
        (p.coords.norm() - self.radius).abs() - self.half
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let out = p.coords.normalize();
        if p.coords.norm() >= self.radius {
            out
        } else {
            -out
        }
    }
}

/// `sdf` meshed on a lattice of spacing `cell` over the cube of half-side `half`, off the origin by a
/// fraction of a cell so no lattice plane is a symmetry plane.
fn mesh(sdf: &dyn Sdf, half: f64, cell: f64, cut_points: CutPoints) -> SdfMeshedTetMesh {
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(
                -half - 0.31 * cell,
                -half - 0.17 * cell,
                -half - 0.23 * cell,
            ),
            Vec3::new(half, half, half),
        ),
        cell_size: cell,
        material_field: None,
    };
    SdfMeshedTetMesh::from_sdf_with(sdf, &hints, cut_points).expect("the scene must mesh")
}

/// The largest `|sdf|` over the vertices of `mesh`'s boundary faces.
fn off_the_zero_set(mesh: &SdfMeshedTetMesh, sdf: &dyn Sdf) -> f64 {
    mesh.boundary_faces()
        .iter()
        .flatten()
        .map(|&v| sdf.eval(Point3::from(mesh.positions()[v as usize])).abs())
        .fold(0.0, f64::max)
}

#[test]
fn root_cuts_put_every_boundary_vertex_on_a_spheres_surface() {
    let radius = 0.1;
    let sphere = SphereSdf { radius };
    for cells in [3.0, 5.0] {
        let cell = radius / cells;
        let root = off_the_zero_set(&mesh(&sphere, 1.3 * radius, cell, CutPoints::Root), &sphere);
        let linear = off_the_zero_set(
            &mesh(&sphere, 1.3 * radius, cell, CutPoints::Interpolated),
            &sphere,
        );
        assert!(
            root <= 1e-12 * radius,
            "at {cells} cells a radius, a root cut sits {root:e} m off the sphere"
        );
        assert!(
            linear >= 1e-3 * radius,
            "the linear rule's cuts sit off the sphere, here only {linear:e} m"
        );
    }
}

#[test]
fn root_cuts_find_a_kinked_distances_zero_set() {
    // The shell is 0.8 of a cell thick, so most edges that cross its surface straddle the kink between.
    let cell = 0.01;
    let shell = Shell {
        radius: 0.05,
        half: 0.4 * cell,
    };
    let root = off_the_zero_set(&mesh(&shell, 0.07, cell, CutPoints::Root), &shell);
    let linear = off_the_zero_set(&mesh(&shell, 0.07, cell, CutPoints::Interpolated), &shell);
    assert!(
        root <= 1e-12 * shell.radius,
        "a root cut sits {root:e} m off the shell"
    );
    assert!(
        linear >= 0.1 * shell.half,
        "the linear rule's cuts sit off the kinked shell, here only {linear:e} m"
    );
}

#[test]
fn root_cuts_keep_the_angle_bounds() {
    let radius = 0.1;
    let sphere = SphereSdf { radius };
    let shell = Shell {
        radius: 0.05,
        half: 0.013,
    };
    let (low, high) = DIHEDRAL_BOUNDS_DEG;
    for (label, sdf, half, cell) in [
        (
            "sphere, 3 cells a radius",
            &sphere as &dyn Sdf,
            0.13,
            radius / 3.0,
        ),
        ("sphere, 8 cells a radius", &sphere, 0.13, radius / 8.0),
        ("shell", &shell, 0.07, 0.01),
    ] {
        let mesh = mesh(sdf, half, cell, CutPoints::Root);
        let quality = mesh.quality();
        let smallest = quality
            .dihedral_min
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b))
            .to_degrees();
        let largest = quality
            .dihedral_max
            .iter()
            .fold(0.0, |a: f64, &b| a.max(b))
            .to_degrees();
        assert!(
            smallest >= low - 1e-4 && largest <= high + 1e-4,
            "{label}: dihedral angles {smallest}°–{largest}°, outside Theorem 1's {low}°–{high}°"
        );
        assert!(quality.signed_volume.iter().all(|&v| v > 0.0), "{label}");
    }
}

/// A sphere whose SDF is `NaN` off the lattice's half-cell planes in x, so every lattice vertex samples finite
/// and only a point inside an edge that runs across x does not.
struct NanBetweenPlanes {
    radius: f64,
    cell: f64,
}

impl Sdf for NanBetweenPlanes {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let halves = 2.0 * p.x / self.cell;
        if (halves - halves.round()).abs() > 1e-9 {
            f64::NAN
        } else {
            p.coords.norm() - self.radius
        }
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        p.coords.normalize()
    }
}

#[test]
fn a_non_finite_sdf_inside_a_crossed_edge_is_reported() {
    let sdf = NanBetweenPlanes {
        radius: 0.1,
        cell: 0.025,
    };
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-0.15, -0.15, -0.15), Vec3::new(0.15, 0.15, 0.15)),
        cell_size: sdf.cell,
        material_field: None,
    };
    assert!(
        SdfMeshedTetMesh::from_sdf_with(&sdf, &hints, CutPoints::Interpolated).is_ok(),
        "the lattice's own samples are finite"
    );
    let located = SdfMeshedTetMesh::from_sdf_with(&sdf, &hints, CutPoints::Root);
    assert!(
        matches!(
            &located,
            Err(MeshingError::NonFiniteSdfOnEdge { vertices, value })
                if value.is_nan() && vertices[0] != vertices[1]
        ),
        "locating the cuts must report the edge, got {:?}",
        located.as_ref().err()
    );
}
