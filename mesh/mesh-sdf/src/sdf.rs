//! Parry-backed unsigned-distance and sign oracles.
//!
//! [`TriMeshDistance`] wraps a parry3d `TriMesh` BVH and exposes the
//! per-query unsigned distance + closest-point oracle. [`PseudoNormalSign`]
//! shares the same `Arc<TriMesh>` and exposes parry's pseudo-normal-based
//! inside test. Composed via [`crate::Signed`], they recover the
//! previous `SignedDistanceField` behavior — kept around as a deprecated
//! type alias + constructor so legacy call sites keep building during
//! the D arc consumer migration.
//!
//! # Sign contract
//!
//! [`PseudoNormalSign`] is **fast** on watertight, well-formed meshes
//! but **fragile** on decimated / cleaned scans with cap fans whose
//! winding flipped during reconstruction. cf-cast-cli's
//! `NotWatertight: 3156 open edges` on iter-1's `plug_layer_0.stl`
//! ([[project-cf-cast-plug-layer-0-watertight-discovery]]) is the
//! canonical failure mode. New consumers that derive an SDF from a
//! cf-scan-prep cleaned scan should prefer the flood-fill sign oracle
//! shipped in D.2.

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

    /// Build from a shared `Arc<TriMesh>` plus the source mesh — used
    /// internally to keep `TriMeshDistance` and [`PseudoNormalSign`]
    /// sharing one BVH allocation when constructed together.
    pub(crate) fn from_shared(tri_mesh: Arc<TriMesh>, mesh: IndexedMesh) -> Self {
        Self { tri_mesh, mesh }
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

/// **Deprecated.** Type alias preserved so existing call sites keep
/// building during the D arc consumer migration.
///
/// The previous monolithic `SignedDistanceField` conflated unsigned
/// distance and sign in a single struct, which papered over the choice
/// of sign oracle and let consumers silently ship without sign
/// defense. The replacement is an explicit composition:
///
/// ```ignore
/// // Old:
/// let sdf = SignedDistanceField::new(mesh)?;
///
/// // New (preserves old behavior):
/// let distance = TriMeshDistance::new(mesh)?;
/// let sign = PseudoNormalSign::from_distance(&distance);
/// let sdf = Signed { distance, sign };
///
/// // Preferred on cf-scan-prep cleaned scans (D.2 ships FloodFillSign):
/// // let sign = FloodFillSign::build(&distance, bounds, cell_size, 0.75)?;
/// // let sdf = Signed { distance, sign };
/// ```
///
/// The convenience constructor [`SignedDistanceField::new`] is
/// retained for one release cycle to bridge the migration; it returns
/// this alias type directly so `Arc<SignedDistanceField>` and similar
/// patterns work unchanged. See
/// `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the migration
/// plan.
#[deprecated(
    since = "0.1.0",
    note = "Conflates distance and sign — pick a sign oracle explicitly. \
            `SignedDistanceField::new(mesh)?` is equivalent to \
            `Signed::new(TriMeshDistance::new(mesh)?, PseudoNormalSign::from_distance(&distance))` \
            but `PseudoNormalSign` is unreliable on cf-scan-prep cleaned scans. Prefer \
            `FloodFillSign` (mesh-sdf D.2) for any SDF derived from a body-part scan. See \
            docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md."
)]
pub type SignedDistanceField = Signed<TriMeshDistance, PseudoNormalSign>;

/// Convenience constructor + `mesh()` accessor for the specific
/// `Signed<TriMeshDistance, PseudoNormalSign>` composition — bridges
/// the deprecated `SignedDistanceField::new(mesh)?` API.
///
/// `mesh()` lives here (not on the generic `Signed<D, S>` impl)
/// because it only makes sense when `D` carries the source mesh; the
/// `TriMeshDistance`-specific impl makes that explicit.
impl Signed<TriMeshDistance, PseudoNormalSign> {
    /// **Deprecated.** Build a `Signed<TriMeshDistance,
    /// PseudoNormalSign>` from a mesh — preserves the old
    /// `SignedDistanceField::new` shape.
    ///
    /// # Errors
    ///
    /// Returns [`SdfError::EmptyMesh`] if `mesh` has no faces.
    #[deprecated(
        since = "0.1.0",
        note = "Conflates distance and sign — pick a sign oracle explicitly. Construct via \
                `TriMeshDistance::new(mesh)?` + `PseudoNormalSign::from_distance(&distance)` + \
                `Signed::new(distance, sign)`. Prefer `FloodFillSign` (D.2) on cleaned scans."
    )]
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        let tri_mesh = Arc::new(build_tri_mesh(&mesh));
        let distance = TriMeshDistance::from_shared(Arc::clone(&tri_mesh), mesh);
        let sign = PseudoNormalSign { tri_mesh };
        Ok(Signed { distance, sign })
    }

    /// Borrow the source mesh.
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

/// **Deprecated.** One-shot signed distance — rebuilds the BVH per
/// call.
///
/// Retained for one release cycle to bridge insertion_sim's
/// `unsigned_distance` consumer migration (planned D.3). New code
/// should build a `TriMeshDistance` once and compose with a sign
/// oracle.
///
/// Returns `f64::MAX` on an empty mesh.
#[deprecated(
    since = "0.1.0",
    note = "Rebuilds the BVH per call and ships PseudoNormalSign's fragile sign branch. \
            Build a TriMeshDistance once and compose with a Sign oracle (PseudoNormalSign or \
            FloodFillSign) explicitly."
)]
#[must_use]
pub fn signed_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }
    let tri_mesh = build_tri_mesh(mesh);
    let projection = tri_mesh.project_local_point(&f64_to_parry(point), false);
    let unsigned = (point - parry_to_f64(projection.point)).norm();
    if projection.is_inside {
        -unsigned
    } else {
        unsigned
    }
}

/// **Deprecated.** One-shot unsigned distance — rebuilds the BVH per
/// call.
///
/// Retained for one release cycle to bridge insertion_sim's existing
/// consumer (planned D.3 migration). New code should build a
/// [`TriMeshDistance`] once and call `distance` per query.
///
/// Returns `f64::MAX` on an empty mesh.
#[deprecated(
    since = "0.1.0",
    note = "Rebuilds the BVH per call. Build a TriMeshDistance once and call distance() per query."
)]
#[must_use]
pub fn unsigned_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }
    let tri_mesh = build_tri_mesh(mesh);
    let projection = tri_mesh.project_local_point(&f64_to_parry(point), false);
    (point - parry_to_f64(projection.point)).norm()
}

#[cfg(test)]
#[allow(deprecated)]
mod tests {
    use super::*;
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

    fn unit_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));

        // CCW winding when viewed from outside
        mesh.faces.push([0, 2, 1]); // bottom
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left
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

    #[test]
    fn sdf_new_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_err());
    }

    #[test]
    fn sdf_new_valid_mesh() {
        let mesh = simple_triangle();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn sdf_distance_above_triangle() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point directly above center of triangle
        let dist = sdf.distance(Point3::new(5.0, 3.33, 5.0));
        assert_relative_eq!(dist.abs(), 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_distance_on_surface() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point on the triangle
        let dist = sdf.distance(Point3::new(5.0, 3.0, 0.0));
        assert_relative_eq!(dist.abs(), 0.0, epsilon = 0.01);
    }

    #[test]
    fn sdf_closest_point() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point above center
        let query = Point3::new(5.0, 3.0, 5.0);
        let closest = sdf.closest_point(query);

        // Closest point should be on the plane z=0
        assert_relative_eq!(closest.z, 0.0, epsilon = 0.01);
    }

    #[test]
    fn signed_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = signed_distance(point, &mesh);
        assert!(dist.abs() > 0.0);
    }

    #[test]
    fn unsigned_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = unsigned_distance(point, &mesh);
        assert!(dist >= 0.0);
        assert_relative_eq!(dist, 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_inside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Centroid of tetrahedron is inside
        let centroid = Point3::new(0.5, 0.385, 0.204);
        assert!(sdf.is_inside(centroid) || sdf.distance(centroid) < 0.1);
    }

    #[test]
    fn sdf_outside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point far outside
        let outside = Point3::new(10.0, 10.0, 10.0);
        assert!(!sdf.is_inside(outside));
    }

    #[test]
    fn sdf_mesh_accessor() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        assert_eq!(sdf.mesh().faces.len(), 1);
    }

    #[test]
    fn empty_mesh_distance() {
        let mesh = IndexedMesh::new();
        let dist = unsigned_distance(Point3::new(0.0, 0.0, 0.0), &mesh);
        assert_eq!(dist, f64::MAX);
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
        let mesh = closed_pyramid();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

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
        let open_cube = open_top_unit_cube();
        let sdf = SignedDistanceField::new(open_cube).expect("should create SDF");
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

    /// The deprecated `SignedDistanceField::new` constructor also
    /// shares the Arc internally — distance + sign both pull from one
    /// BVH.
    #[test]
    fn deprecated_constructor_shares_arc_between_fields() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("non-empty mesh");
        assert!(
            Arc::ptr_eq(sdf.distance.shared_tri_mesh(), &sdf.sign.tri_mesh),
            "deprecated convenience must share the BVH between its two fields"
        );
    }
}
