//! Parry-backed unsigned-distance and sign oracles.
//!
//! [`TriMeshDistance`] wraps a parry3d `TriMesh` BVH and exposes the
//! per-query unsigned distance + closest-point oracle. [`PseudoNormalSign`]
//! shares the same `Arc<TriMesh>` and exposes parry's pseudo-normal-based
//! inside test. Compose them via [`crate::Signed`] to get a signed-distance
//! source.
//!
//! # Sign contract
//!
//! [`PseudoNormalSign`] is **fast** on watertight, well-formed meshes
//! but **fragile** on decimated / cleaned scans with cap fans whose
//! winding flipped during reconstruction. New consumers that derive an
//! SDF from a cf-scan-prep cleaned scan should prefer the flood-fill
//! sign oracle [`crate::FloodFillSign`] — see
//! `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the rationale.

use std::sync::Arc;

use mesh_types::IndexedMesh;
use nalgebra::Point3;
use parry3d::math::{Point as ParryPoint, Real as ParryReal};
use parry3d::query::PointQuery;
use parry3d::shape::{TriMesh, TriMeshFlags};

use crate::error::{SdfError, SdfResult};
use crate::oracle::{Sign, Signed, UnsignedDistance};

/// Unsigned-distance oracle backed by a parry3d BVH.
///
/// Construction builds the BVH once; queries are O(log faces). The
/// inner `Arc<TriMesh>` is the share point — `PseudoNormalSign`
/// instances built over the same mesh hold a clone of the same Arc so
/// the BVH is built exactly once per source mesh.
#[derive(Debug, Clone)]
pub struct TriMeshDistance {
    tri_mesh: Arc<TriMesh>,
    mesh: IndexedMesh,
}

impl TriMeshDistance {
    /// Build the BVH over `mesh`.
    ///
    /// # Errors
    ///
    /// Returns [`SdfError::EmptyMesh`] if `mesh` has no faces.
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        let tri_mesh = Arc::new(build_tri_mesh(&mesh));
        Ok(Self { tri_mesh, mesh })
    }

    /// Borrow the underlying `IndexedMesh` the BVH was built over.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        &self.mesh
    }

    /// Borrow the shared parry `TriMesh`.
    pub(crate) fn shared_tri_mesh(&self) -> &Arc<TriMesh> {
        &self.tri_mesh
    }
}

impl UnsignedDistance for TriMeshDistance {
    fn distance(&self, point: Point3<f64>) -> f64 {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(point), false);
        (point - parry_to_f64(projection.point)).norm()
    }

    fn closest_point(&self, point: Point3<f64>) -> Point3<f64> {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(point), false);
        parry_to_f64(projection.point)
    }
}

/// Sign oracle backed by parry3d's pseudo-normal inside test.
///
/// Fast (one BVH closest-feature query per call) and reliable on
/// watertight, well-formed meshes. Fragile on decimated / cleaned
/// scans with inverted-winding cap fans or high-valence apex vertices
/// — see the crate-level sign-contract docs.
#[derive(Debug, Clone)]
pub struct PseudoNormalSign {
    tri_mesh: Arc<TriMesh>,
}

impl PseudoNormalSign {
    /// Build from an owned mesh — convenience constructor; rebuilds
    /// the BVH internally. Prefer composing with a
    /// [`TriMeshDistance`] (and sharing the underlying `Arc<TriMesh>`
    /// via [`Self::from_distance`]) when both oracles are wanted.
    ///
    /// # Errors
    ///
    /// Returns [`SdfError::EmptyMesh`] if `mesh` has no faces.
    pub fn new(mesh: &IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        Ok(Self {
            tri_mesh: Arc::new(build_tri_mesh(mesh)),
        })
    }

    /// Share the parry BVH already built by a [`TriMeshDistance`].
    ///
    /// Use this when constructing both oracles over the same mesh —
    /// the resulting `Signed<TriMeshDistance, PseudoNormalSign>` then
    /// owns exactly one BVH.
    #[must_use]
    pub fn from_distance(distance: &TriMeshDistance) -> Self {
        Self {
            tri_mesh: Arc::clone(distance.shared_tri_mesh()),
        }
    }
}

impl Sign for PseudoNormalSign {
    fn is_inside(&self, point: Point3<f64>) -> bool {
        self.tri_mesh
            .project_local_point(&f64_to_parry(point), false)
            .is_inside
    }
}

/// `mesh()` accessor on any `Signed<TriMeshDistance, S>` —
/// independent of the sign oracle, so every composition over a
/// shared `TriMeshDistance` (regardless of which [`Sign`] oracle
/// it pairs with) borrows the same source mesh without
/// re-implementing the accessor per instantiation.
impl<S: Sign> Signed<TriMeshDistance, S> {
    /// Borrow the source mesh the distance oracle's BVH was built
    /// over.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        self.distance.mesh()
    }
}

/// Build a parry `TriMesh` from an `IndexedMesh`.
///
/// Sets `TriMeshFlags::ORIENTED` so pseudo-normals are computed; this
/// is what makes [`PseudoNormalSign::is_inside`] meaningful on
/// watertight meshes. On non-manifold input, pseudo-normal computation
/// may be partial — `is_inside` then becomes undefined.
fn build_tri_mesh(mesh: &IndexedMesh) -> TriMesh {
    let vertices: Vec<ParryPoint<ParryReal>> = mesh
        .vertices
        .iter()
        .map(|v| ParryPoint::new(v.x as ParryReal, v.y as ParryReal, v.z as ParryReal))
        .collect();
    let indices: Vec<[u32; 3]> = mesh.faces.to_vec();
    TriMesh::with_flags(vertices, indices, TriMeshFlags::ORIENTED)
}

#[inline]
fn f64_to_parry(p: Point3<f64>) -> ParryPoint<ParryReal> {
    ParryPoint::new(p.x as ParryReal, p.y as ParryReal, p.z as ParryReal)
}

#[inline]
fn parry_to_f64(p: ParryPoint<ParryReal>) -> Point3<f64> {
    Point3::new(p.x as f64, p.y as f64, p.z as f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_fixtures::unit_tetrahedron;
    use approx::assert_relative_eq;
    use mesh_types::Point3;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    /// Open-top unit cube (no +z face): 10 triangles instead of 12.
    /// Non-manifold by construction — sign is undefined by contract.
    fn open_top_unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // 8 corners of the unit cube
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0)); // 0: ---
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0)); // 1: +--
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2: ++-
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0)); // 3: -+-
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4: --+
        mesh.vertices.push(Point3::new(1.0, 0.0, 1.0)); // 5: +-+
        mesh.vertices.push(Point3::new(1.0, 1.0, 1.0)); // 6: +++
        mesh.vertices.push(Point3::new(0.0, 1.0, 1.0)); // 7: -++

        // Bottom (-z), outward normal -z
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // -y face
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // +x face
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);
        // +y face
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // -x face
        mesh.faces.push([3, 0, 4]);
        mesh.faces.push([3, 4, 7]);
        // +z face deliberately omitted -> open top, non-manifold.
        mesh
    }

    /// Closed pyramid (square base + apex), watertight via a cap-fan
    /// on the base. The kind of geometry where the old face-normal
    /// sign heuristic could flip far from the surface; pseudo-normals
    /// aggregate per-vertex so the apex returns a consistent outward
    /// direction regardless of which BVH leaf is returned.
    fn closed_pyramid() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // 4 base corners at z=0, apex at z=1.
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0)); // 0
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0)); // 1
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Point3::new(-1.0, 1.0, 0.0)); // 3
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4: apex

        // Base, outward normal -z (CCW from below).
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // 4 sloped sides, CCW from outside.
        mesh.faces.push([0, 1, 4]); // -y side
        mesh.faces.push([1, 2, 4]); // +x side
        mesh.faces.push([2, 3, 4]); // +y side
        mesh.faces.push([3, 0, 4]); // -x side
        mesh
    }

    /// Compose `Signed<TriMeshDistance, PseudoNormalSign>` from a
    /// mesh — the standard parry-pseudo-normal composition used across
    /// the tests below.
    fn build_sdf(mesh: IndexedMesh) -> Signed<TriMeshDistance, PseudoNormalSign> {
        let distance = TriMeshDistance::new(mesh).expect("should create SDF");
        let sign = PseudoNormalSign::from_distance(&distance);
        Signed { distance, sign }
    }

    #[test]
    fn distance_oracle_rejects_empty_mesh() {
        let mesh = IndexedMesh::new();
        assert!(TriMeshDistance::new(mesh).is_err());
    }

    #[test]
    fn distance_oracle_accepts_valid_mesh() {
        let mesh = simple_triangle();
        assert!(TriMeshDistance::new(mesh).is_ok());
    }

    #[test]
    fn sdf_distance_above_triangle() {
        let sdf = build_sdf(simple_triangle());

        // Point directly above center of triangle
        let dist = sdf.distance(Point3::new(5.0, 3.33, 5.0));
        assert_relative_eq!(dist.abs(), 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_distance_on_surface() {
        let sdf = build_sdf(simple_triangle());

        // Point on the triangle
        let dist = sdf.distance(Point3::new(5.0, 3.0, 0.0));
        assert_relative_eq!(dist.abs(), 0.0, epsilon = 0.01);
    }

    #[test]
    fn sdf_closest_point() {
        let sdf = build_sdf(simple_triangle());

        // Point above center
        let query = Point3::new(5.0, 3.0, 5.0);
        let closest = sdf.closest_point(query);

        // Closest point should be on the plane z=0
        assert_relative_eq!(closest.z, 0.0, epsilon = 0.01);
    }

    #[test]
    fn sdf_inside_tetrahedron() {
        let sdf = build_sdf(unit_tetrahedron());

        // Centroid of tetrahedron is inside
        let centroid = Point3::new(0.5, 0.385, 0.204);
        assert!(sdf.is_inside(centroid) || sdf.distance(centroid) < 0.1);
    }

    #[test]
    fn sdf_outside_tetrahedron() {
        let sdf = build_sdf(unit_tetrahedron());

        // Point far outside
        let outside = Point3::new(10.0, 10.0, 10.0);
        assert!(!sdf.is_inside(outside));
    }

    #[test]
    fn sdf_mesh_accessor() {
        let sdf = build_sdf(simple_triangle());

        assert_eq!(sdf.mesh().faces.len(), 1);
    }

    /// Far-field sign is reliable on a watertight pyramid (cap-fan
    /// style) — the failure mode the old face-normal heuristic
    /// produced on real cleaned scans (see project memory
    /// `pinned-floor-visual-gate-postmortem`, `min_sdf = -89.5 mm` on
    /// a 71 mm body). With parry's pseudo-normals, probes far above
    /// / below / lateral of the pyramid all get the expected outside
    /// sign, independent of which BVH leaf face is hit.
    #[test]
    fn far_field_sign_reliable_on_pathological_cap_fan() {
        let sdf = build_sdf(closed_pyramid());

        // Probes far outside in every cardinal direction.
        let probes_outside = [
            Point3::new(0.0, 0.0, 5.0),  // above apex
            Point3::new(0.0, 0.0, -5.0), // below base
            Point3::new(5.0, 0.0, 0.5),  // far +x at mid-height
            Point3::new(-5.0, 0.0, 0.5), // far -x at mid-height
            Point3::new(0.0, 5.0, 0.5),  // far +y at mid-height
            Point3::new(0.0, -5.0, 0.5), // far -y at mid-height
        ];
        for p in probes_outside {
            let d = sdf.distance(p);
            assert!(
                d > 0.0,
                "outside probe at {p:?} must have positive distance, got {d}"
            );
        }

        // Probe deep inside, near apex on axis.
        let inside = Point3::new(0.0, 0.0, 0.5);
        let d_in = sdf.distance(inside);
        assert!(
            d_in < 0.0,
            "inside probe must have negative distance, got {d_in}"
        );
    }

    /// Non-manifold contract — on an open-top cube,
    /// `unsigned_distance` must remain correct, deterministic, and
    /// finite. `distance` must remain finite. Sign is undefined by
    /// contract.
    #[test]
    fn non_manifold_unsigned_distance_correct_and_deterministic() {
        let sdf = build_sdf(open_top_unit_cube());
        let probes = [
            Point3::new(0.5, 0.5, 2.0),  // above the open top
            Point3::new(0.5, 0.5, -1.0), // below the closed bottom
            Point3::new(2.0, 0.5, 0.5),  // beside the closed +x wall
        ];
        for p in probes {
            let d1 = sdf.unsigned_distance(p);
            let d2 = sdf.unsigned_distance(p);
            assert_eq!(d1, d2, "unsigned_distance must be deterministic");
            assert!(d1.is_finite() && d1 >= 0.0);

            let signed = sdf.distance(p);
            assert!(signed.is_finite(), "distance finite even on non-manifold");
        }
    }

    /// `TriMeshDistance` and `PseudoNormalSign` constructed together
    /// share one BVH allocation; constructed separately, two.
    #[test]
    fn shared_arc_construction_one_bvh() {
        let mesh = unit_tetrahedron();
        let distance = TriMeshDistance::new(mesh).expect("non-empty mesh");
        let sign = PseudoNormalSign::from_distance(&distance);
        assert!(
            Arc::ptr_eq(distance.shared_tri_mesh(), &sign.tri_mesh),
            "from_distance must share the TriMeshDistance's Arc"
        );
    }
}
