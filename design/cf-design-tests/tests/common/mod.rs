//! Shared helpers for the geometry-fidelity ladder integration tests.
//!
//! Importing a real vertebra mesh in its native coordinates, building the exact
//! signed-distance oracle, and locating the vertebral body from the field are
//! the common substrate of the L4/L5 rung tests (rung4b facet contact, rung5
//! ligament tension, …). They live here so a change to the load/weld path, the
//! oracle construction, or the body-center scan is made once, not per rung.

#![allow(dead_code)] // each test binary consumes only a subset of these helpers.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_geometry::{Aabb, IndexedMesh};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, Vector3};

/// The exact mesh-derived metric oracle (signed distance to a vertebra surface):
/// a parry BVH distance composed with a pseudo-normal inside/outside sign.
pub type MeshOracle = Signed<TriMeshDistance, PseudoNormalSign>;

/// Load + weld-repair a vertebra STL named by the environment variable
/// `path_var`, KEEPING native coordinates (do not recenter — sibling vertebrae
/// must stay in their shared anatomical frame so they stack).
pub fn load_native(path_var: &str) -> IndexedMesh {
    let path =
        std::env::var(path_var).unwrap_or_else(|_| panic!("set ${path_var} to a vertebra STL"));
    let mut mesh = load_stl(&path).unwrap_or_else(|e| panic!("load {path_var}: {e}"));
    let rep = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!(
        "[{path_var}] welded {} verts -> {} verts / {} faces",
        rep.vertices_welded,
        mesh.vertices.len(),
        mesh.faces.len()
    );
    mesh
}

/// Build the exact signed distance oracle for a mesh (parry BVH + pseudo-normal
/// sign — metric by construction).
pub fn oracle(mesh: &IndexedMesh) -> MeshOracle {
    let dist = TriMeshDistance::new(mesh.clone()).unwrap();
    let sign = PseudoNormalSign::from_distance(&dist);
    Signed {
        distance: dist,
        sign,
    }
}

/// Locate the vertebral body WITHOUT any axis assumption: the deepest interior
/// point of the signed field (most negative distance) is the thickest solid
/// mass, which for a vertebra is the body. Coarse AABB scan. Returns the point
/// and its depth (negative = inside).
pub fn body_center(mesh: &IndexedMesh, sdf: &MeshOracle) -> (Point3<f64>, f64) {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let step = 2.0;
    let span = bbox.max - bbox.min;
    let count = |len: f64| (len / step).ceil() as usize + 1;
    let (nx, ny, nz) = (count(span.x), count(span.y), count(span.z));
    let (mut depth, mut center) = (f64::MAX, Point3::origin());
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let pt = bbox.min + Vector3::new(ix as f64, iy as f64, iz as f64) * step;
                let val = sdf.evaluate(pt);
                if val < depth {
                    depth = val;
                    center = pt;
                }
            }
        }
    }
    (center, depth)
}
