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
    /// Power-of-two factor the BVH's coordinates were built at, relative to `mesh`.
    ///
    /// The oracle lifts every query into this internal frame and brings the answer
    /// back, so the **caller's frame never moves** — see [`Self::with_scale`].
    scale: f64,
}

impl TriMeshDistance {
    /// Build the BVH over `mesh`, internally normalised so its smallest triangle clears
    /// parry's f32 area floor.
    ///
    /// The scale is chosen from the mesh's own geometry ([`choose_scale`]) and applied
    /// **inside** the oracle: queries are lifted in and answers brought back, so results
    /// are in the caller's units and the caller's frame never moves. Building the same
    /// surface in millimetres or in metres now gives the same answers — which it did not
    /// before, and is why the FSU disc meshed 30 % phantom material after a rescale to SI.
    ///
    /// # Errors
    ///
    /// - [`SdfError::EmptyMesh`] if `mesh` has no faces.
    /// - [`SdfError::UnscalableMesh`] if no scale can satisfy both ends of the rule.
    /// - [`SdfError::FaceIndexOutOfRange`] if a face names a missing vertex.
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        let scale = choose_scale(&mesh)?;
        Self::with_scale(mesh, scale)
    }

    /// Build the BVH over `mesh` scaled by `scale`, an explicit override of the factor
    /// [`Self::new`] would choose.
    ///
    /// **The caller's frame is unaffected.** `scale` changes only the coordinates parry's
    /// f32 BVH is built at; queries are lifted in and answers brought back, so every
    /// public result is in the caller's units regardless of `scale`. For a power of two
    /// this round trip is a pure exponent shift and therefore exact — measured over 2⁴⁰
    /// by `power_of_two_rescale_is_bit_identical` and, for the gradient, across the whole
    /// usable band by `internal_normalisation_leaves_grad_bit_identical`.
    ///
    /// `pub(crate)` on purpose. It is the seam the scale-regime gates need in order to
    /// hold the scale fixed while the rule that chooses one varies — not a knob for
    /// consumers, who should never have to know this frame exists.
    ///
    /// # Errors
    ///
    /// Returns [`SdfError::EmptyMesh`] if `mesh` has no faces.
    pub(crate) fn with_scale(mesh: IndexedMesh, scale: f64) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        let tri_mesh = Arc::new(build_tri_mesh(&mesh, scale));
        Ok(Self {
            tri_mesh,
            mesh,
            scale,
        })
    }

    /// Borrow the underlying `IndexedMesh` the BVH was built over.
    ///
    /// Always in the **caller's** units — the internal scale is not applied here.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        &self.mesh
    }

    /// Borrow the shared parry `TriMesh`.
    pub(crate) fn shared_tri_mesh(&self) -> &Arc<TriMesh> {
        &self.tri_mesh
    }

    /// The internal power-of-two scale this oracle's BVH was built at.
    pub(crate) fn scale(&self) -> f64 {
        self.scale
    }
}

/// Lift a caller-frame point into the oracle's internal frame.
#[inline]
fn lift(p: Point3<f64>, scale: f64) -> Point3<f64> {
    Point3::new(p.x * scale, p.y * scale, p.z * scale)
}

/// The area at or below which `parry3d` silently skips a triangle.
///
/// `parry3d` is the **f32** build, so `parry3d::math::DEFAULT_EPSILON` is `f32::EPSILON`.
/// `TriMesh::compute_pseudo_normals` accumulates a triangle only `if let Some(n) =
/// tri.normal()`, and `Triangle::normal` is `Unit::try_new(ab × ac, DEFAULT_EPSILON)`.
/// `‖ab × ac‖` is twice the area, so a triangle at or under `f32::EPSILON / 2` is skipped
/// — leaving a **zero** pseudo-normal, which parry's `dpt.dot(&pn) <= 0.0` inside test
/// satisfies unconditionally. A zero pseudo-normal reports "inside" at any distance.
fn area_floor() -> f64 {
    f64::from(f32::EPSILON) / 2.0
}

/// How far above [`area_floor`] the smallest triangle is placed, in binades.
///
/// **Not a fitted constant, and deliberately not measured against any particular mesh.**
/// The cost function here is one-sided: too small silently fails to fix the sign bug,
/// while too large costs *nothing* — a power-of-two rescale is an exponent shift, proven
/// bit-neutral over 2⁴⁰ (`power_of_two_rescale_is_bit_identical`) and across the whole
/// gradient band (`internal_normalisation_leaves_grad_bit_identical`). So there is no
/// optimum to find: there is a floor to clear and a distant cliff to avoid, and the right
/// move is to sit far from both.
///
/// Fitting this to the FSU disc would have reintroduced the arc's original defect one
/// level up — a constant validated on one geometry, which is exactly how `Sdf::grad`'s
/// `1e-6` and parry's own area floor became load-bearing. Instead the value sits near the
/// middle of a feasible band tens of binades wide, and
/// `the_chosen_scale_is_insensitive_to_the_margin_across_decades` asserts that the
/// **result does not depend on it** — which is a stronger claim than any fitted number.
const AREA_MARGIN_BINADES: i32 = 40;

/// Largest internal coordinate magnitude the BVH may be built at.
///
/// parry computes `ab.cross(&ac)` and squared norms in **f32**, so intermediates grow as
/// coordinate², and they overflow to infinity above `sqrt(f32::MAX)`. Ten binades of
/// headroom below that, derived rather than hand-picked so it tracks the type instead of
/// a remembered number.
fn coordinate_cap() -> f64 {
    f64::from(f32::MAX).sqrt() / 1024.0
}

/// Smallest triangle area and largest absolute coordinate, in the caller's units.
///
/// # Errors
///
/// Returns [`SdfError::FaceIndexOutOfRange`] if a face names a vertex the mesh does not
/// have — indexing would otherwise panic here, and a scale derived from a mesh whose
/// faces do not resolve is meaningless anyway.
fn mesh_scale_inputs(mesh: &IndexedMesh) -> SdfResult<(f64, f64)> {
    let vertex_count = mesh.vertices.len();
    let mut extent = 0.0_f64;
    for v in &mesh.vertices {
        extent = extent.max(v.x.abs()).max(v.y.abs()).max(v.z.abs());
    }
    let mut min_area = f64::INFINITY;
    for f in &mesh.faces {
        let corner = |i: u32| {
            mesh.vertices
                .get(i as usize)
                .copied()
                .ok_or(SdfError::FaceIndexOutOfRange {
                    index: i,
                    vertex_count,
                })
        };
        let (a, b, c) = (corner(f[0])?, corner(f[1])?, corner(f[2])?);
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        // `f64::min` propagates the non-NaN operand, so a NaN area would be silently
        // dropped and the scale derived from the rest of the mesh. Take it explicitly so
        // a corrupt vertex surfaces as an unscalable mesh rather than a plausible number.
        min_area = if area.is_nan() {
            f64::NAN
        } else {
            min_area.min(area)
        };
        if min_area.is_nan() {
            break;
        }
    }
    Ok((min_area, extent))
}

/// The power-of-two scale [`TriMeshDistance::new`] builds its BVH at.
///
/// Smallest `2^k`, `k >= 0`, that lifts the mesh's own smallest triangle
/// [`AREA_MARGIN_BINADES`] above [`area_floor`] — **derived from the geometry, which is
/// what makes it dimensionless.** A caller who rescales millimetres to metres shrinks
/// every area by 1e6 and gets a `k` six binades larger; the oracle behaves identically
/// either way, which is the property the whole remedy is for.
///
/// `k >= 0` is a contract, not an optimisation: the rule only ever **lifts**. Shrinking a
/// mesh that already clears the floor could push it under, turning a working oracle into
/// a broken one.
///
/// # Errors
///
/// - [`SdfError::UnscalableMesh`] if the smallest triangle is zero-area or non-finite (no
///   scale lifts a zero off the floor), or if clearing the floor would push coordinates
///   past [`coordinate_cap`].
/// - [`SdfError::FaceIndexOutOfRange`] via [`mesh_scale_inputs`].
#[allow(
    clippy::cast_possible_truncation,
    reason = "`k` is bounded above by `max_k`, itself finite because `extent > 0` is \
              established by the zero-area guard, so the i32 cast cannot truncate"
)]
pub(crate) fn choose_scale(mesh: &IndexedMesh) -> SdfResult<f64> {
    let (min_area, extent) = mesh_scale_inputs(mesh)?;
    // Written as a positive test so NaN falls through to the error arm rather than
    // slipping past a negated comparison: NaN is neither `> 0.0` nor `<= 0.0`.
    let liftable = min_area > 0.0 && min_area.is_finite() && extent.is_finite();
    if !liftable {
        return Err(SdfError::UnscalableMesh {
            min_area,
            extent,
            reason: "a zero-area or non-finite triangle cannot be lifted off parry's area \
                     floor by any scale",
        });
    }

    let target = area_floor() * 2.0_f64.powi(AREA_MARGIN_BINADES);
    // Area goes as length², so the scale needed is the square root of the area shortfall.
    let k = (target / min_area).sqrt().log2().ceil().max(0.0);
    let max_k = (coordinate_cap() / extent).log2().floor();
    if k > max_k {
        return Err(SdfError::UnscalableMesh {
            min_area,
            extent,
            reason: "clearing the area floor would push coordinates past the f32 \
                     product-overflow cap — this mesh's triangle areas span more dynamic \
                     range than f32 can hold",
        });
    }
    Ok(2.0_f64.powi(k as i32))
}

impl UnsignedDistance for TriMeshDistance {
    fn distance(&self, point: Point3<f64>) -> f64 {
        // ⚠ The norm is taken in the INTERNAL frame and divided back, not taken between
        //   the caller's point and an unlifted projection. The two differ in their
        //   rounding, and this is the form `internal_normalisation_leaves_grad_bit_identical`
        //   measured as bit-neutral — shipping the other one would ship something the
        //   arc never measured.
        let lifted = lift(point, self.scale);
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(lifted), false);
        (lifted - parry_to_f64(projection.point)).norm() / self.scale
    }

    fn closest_point(&self, point: Point3<f64>) -> Point3<f64> {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(lift(point, self.scale)), false);
        let q = parry_to_f64(projection.point);
        Point3::new(q.x / self.scale, q.y / self.scale, q.z / self.scale)
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
    /// Must match the scale the shared `tri_mesh` was built at, or `is_inside` probes a
    /// point that is not where the caller asked. [`Self::from_distance`] copies it from
    /// the distance oracle, which is the only way it can disagree — see that method.
    scale: f64,
}

impl PseudoNormalSign {
    /// Build from an owned mesh — convenience constructor; rebuilds
    /// the BVH internally. Prefer composing with a
    /// [`TriMeshDistance`] (and sharing the underlying `Arc<TriMesh>`
    /// via [`Self::from_distance`]) when both oracles are wanted.
    ///
    /// # Errors
    ///
    /// - [`SdfError::EmptyMesh`] if `mesh` has no faces.
    /// - [`SdfError::UnscalableMesh`] / [`SdfError::FaceIndexOutOfRange`] via
    ///   [`choose_scale`].
    pub fn new(mesh: &IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }
        // Same rule on the same mesh as `TriMeshDistance::new`, so a pair built this way
        // agrees by construction even though it does not share a BVH.
        let scale = choose_scale(mesh)?;
        Ok(Self {
            tri_mesh: Arc::new(build_tri_mesh(mesh, scale)),
            scale,
        })
    }

    /// Share the parry BVH already built by a [`TriMeshDistance`].
    ///
    /// Use this when constructing both oracles over the same mesh —
    /// the resulting `Signed<TriMeshDistance, PseudoNormalSign>` then
    /// owns exactly one BVH.
    ///
    /// **Takes the distance oracle's internal scale with the BVH.** The two travel
    /// together because they must: probing the shared `tri_mesh` at a differently-scaled
    /// point would return a sign for somewhere else entirely. Sharing the `Arc` and
    /// copying the scale in one place is what makes that unrepresentable.
    #[must_use]
    pub fn from_distance(distance: &TriMeshDistance) -> Self {
        Self {
            tri_mesh: Arc::clone(distance.shared_tri_mesh()),
            scale: distance.scale(),
        }
    }
}

impl Sign for PseudoNormalSign {
    fn is_inside(&self, point: Point3<f64>) -> bool {
        self.tri_mesh
            .project_local_point(&f64_to_parry(lift(point, self.scale)), false)
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

/// Build a parry `TriMesh` from an `IndexedMesh`, with its coordinates scaled by
/// `scale`.
///
/// Sets `TriMeshFlags::ORIENTED` so pseudo-normals are computed; this
/// is what makes [`PseudoNormalSign::is_inside`] meaningful on
/// watertight meshes. On non-manifold input, pseudo-normal computation
/// may be partial — `is_inside` then becomes undefined.
///
/// **The single f64 → f32 narrowing point in the crate**, and therefore the only place
/// the internal scale is applied. `TriMeshDistance::with_scale` and
/// `PseudoNormalSign::new` are its only callers, and `PseudoNormalSign::from_distance`
/// shares the result — so one change here covers distance *and* sign.
///
/// The multiply happens in f64, before the narrowing, so for a power-of-two `scale` it
/// is exact and the narrowing rounds the same way it would have at `scale = 1`.
fn build_tri_mesh(mesh: &IndexedMesh, scale: f64) -> TriMesh {
    let vertices: Vec<ParryPoint<ParryReal>> = mesh
        .vertices
        .iter()
        .map(|v| {
            ParryPoint::new(
                (v.x * scale) as ParryReal,
                (v.y * scale) as ParryReal,
                (v.z * scale) as ParryReal,
            )
        })
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
    use crate::test_fixtures::{unit_tetrahedron, uv_sphere};
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

    /// Unit octahedron: 6 vertices on the ±axes at radius 1, 8 CCW faces.
    /// Integer coords make it an exact closed-form L1-ball — the surface
    /// is `|x| + |y| + |z| = 1`, so the signed distance to it is
    /// `(|x| + |y| + |z| − 1) / √3` (the `/√3` is the L1→L2 conversion,
    /// each face normal being `(±1, ±1, ±1)/√3`).
    fn unit_octahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0)); // 0 +X
        mesh.vertices.push(Point3::new(-1.0, 0.0, 0.0)); // 1 -X
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0)); // 2 +Y
        mesh.vertices.push(Point3::new(0.0, -1.0, 0.0)); // 3 -Y
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4 +Z
        mesh.vertices.push(Point3::new(0.0, 0.0, -1.0)); // 5 -Z
        mesh.faces.push([0, 2, 4]); // +x+y+z
        mesh.faces.push([0, 3, 5]); // +x-y-z
        mesh.faces.push([1, 2, 5]); // -x+y-z
        mesh.faces.push([1, 3, 4]); // -x-y+z
        mesh.faces.push([0, 5, 2]); // +x+y-z
        mesh.faces.push([0, 4, 3]); // +x-y+z
        mesh.faces.push([1, 4, 2]); // -x+y+z
        mesh.faces.push([1, 5, 3]); // -x-y-z
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

    /// Closed-form magnitude oracle: the octahedron surface is the L1 unit
    /// ball, so the signed distance is exactly `(|x|+|y|+|z| − 1)/√3` and the
    /// closest surface point moves each interior query outward by that amount
    /// along the `(±1,±1,±1)/√3` face normal. The other sdf tests pin sign +
    /// loose magnitude bounds against tetrahedron/pyramid fixtures; this pins
    /// exact magnitude + closest-point against an analytical primitive.
    /// (Ported from the `mesh-sdf-distance-query` example's octahedron anchor.)
    /// Tolerance is `1e-6` — parry's BVH is f32 internally, well outside f32
    /// roundoff yet far inside any practical surface-locating tolerance.
    #[test]
    fn signed_distance_matches_closed_form_l1_ball_on_octahedron() {
        const TOL: f64 = 1e-6;
        let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
        let third = 1.0 / 3.0;
        let sdf = build_sdf(unit_octahedron());

        // Generic interior (0.05, 0.07, 0.11): L1-norm 0.23 → SDF -0.77/√3.
        let generic = Point3::new(0.05, 0.07, 0.11);
        assert_relative_eq!(sdf.distance(generic), -0.77 * inv_sqrt3, epsilon = TOL);
        assert_relative_eq!(
            sdf.unsigned_distance(generic),
            0.77 * inv_sqrt3,
            epsilon = TOL
        );
        assert!(sdf.is_inside(generic));
        let cp = sdf.closest_point(generic);
        assert_relative_eq!(cp.x, 0.05 + 0.77 / 3.0, epsilon = TOL);
        assert_relative_eq!(cp.y, 0.07 + 0.77 / 3.0, epsilon = TOL);
        assert_relative_eq!(cp.z, 0.11 + 0.77 / 3.0, epsilon = TOL);

        // 6 face-region interior queries at (±0.3, ±0.3, ±0.3): L1-norm 0.9
        // → SDF -0.1/√3; closest point clamps to the face center (±1/3)³.
        let face_signs: [(f64, f64, f64); 6] = [
            (1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (1.0, 1.0, -1.0),
            (-1.0, -1.0, -1.0),
        ];
        for (sx, sy, sz) in face_signs {
            let q = Point3::new(0.3 * sx, 0.3 * sy, 0.3 * sz);
            assert_relative_eq!(sdf.distance(q), -0.1 * inv_sqrt3, epsilon = TOL);
            assert!(sdf.is_inside(q));
            let cp = sdf.closest_point(q);
            assert_relative_eq!(cp.x, sx * third, epsilon = TOL);
            assert_relative_eq!(cp.y, sy * third, epsilon = TOL);
            assert_relative_eq!(cp.z, sz * third, epsilon = TOL);
        }

        // Vertex-direction exterior probe (2,0,0): clamps to the +X vertex at
        // distance exactly 1.
        let vertex_q = Point3::new(2.0, 0.0, 0.0);
        assert_relative_eq!(sdf.distance(vertex_q), 1.0, epsilon = TOL);
        assert!(!sdf.is_inside(vertex_q));
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

    /// **The scale regime `PseudoNormalSign` is valid in — pinned, because it is
    /// narrower than it looks and nothing else in the tree says so.**
    ///
    /// `parry3d` is the **f32** build, so `parry3d::math::DEFAULT_EPSILON` is
    /// `f32::EPSILON`. `TriMesh::compute_pseudo_normals` accumulates a triangle
    /// only `if let Some(n) = tri.normal()`, and `Triangle::normal` is
    /// `Unit::try_new(ab × ac, DEFAULT_EPSILON)`. Since `‖ab × ac‖` is twice the
    /// triangle's area, **a triangle whose area is at or under `f32::EPSILON / 2`
    /// ≈ 5.96e-8 is silently skipped** — and a skipped triangle leaves a **zero**
    /// pseudo-normal on its vertices and edges. Parry's inside test is
    /// `dpt.dot(&pseudo_normal) <= 0.0`, which a zero vector satisfies
    /// unconditionally, so **a zero pseudo-normal reports "inside" at any
    /// distance from the surface**.
    ///
    /// That floor is **absolute**, and it applies to a quantity with dimension:
    /// area goes as length². Rescaling a mesh from millimetres to metres shrinks
    /// every triangle's area by 1e6 and moves the whole mesh that much closer to
    /// the floor, with no change to its shape. A caller who rescales before
    /// building the oracle can therefore get **sign-inverted** results from
    /// geometry that was fine a moment earlier.
    ///
    /// This is not hypothetical: it is how the FSU disc pipeline came to mesh
    /// ~30 % phantom material, by building its oracle after rescaling to SI
    /// metres while every other consumer in the workspace builds in native
    /// millimetres.
    ///
    /// The assert covers the regime consumers **rely on** — radii from 1e3 down
    /// to 1e-2, where no triangle of this fixture is near the floor — and
    /// deliberately stops there. Below it the oracle is known-wrong, and pinning
    /// a broken result would freeze it as intended behaviour rather than flag it.
    #[test]
    fn pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use() {
        for radius in [1000.0, 10.0, 1.0, 0.1, 0.05, 0.025, 0.01] {
            let mesh = uv_sphere(radius, 24, 48);

            // Precondition, checked rather than assumed: this fixture is clear of
            // the area floor at this radius. If it were not, the sign assert below
            // would be testing the broken regime and would fail for a reason that
            // has nothing to do with a regression.
            let floor = f64::from(f32::EPSILON) / 2.0;
            let smallest = mesh
                .faces
                .iter()
                .map(|f| {
                    let (a, b, c) = (
                        mesh.vertices[f[0] as usize],
                        mesh.vertices[f[1] as usize],
                        mesh.vertices[f[2] as usize],
                    );
                    0.5 * (b - a).cross(&(c - a)).norm()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                smallest > floor,
                "r={radius}: fixture's smallest triangle ({smallest:.4e}) is at or under \
                 parry's area floor ({floor:.4e}) — this radius is outside the valid regime"
            );

            let sdf = build_sdf(mesh);
            // Sample a lattice reaching to 1.6 r per axis, so the box corners sit at
            // 2.77 r — well into the region where the closest feature is an edge or a
            // vertex, never a face interior. That is where an angle-weighted
            // pseudo-normal is load-bearing and where a zeroed one shows up.
            let n = 12;
            for i in 0..=n {
                for j in 0..=n {
                    for k in 0..=n {
                        let q =
                            |idx: i32| 1.6 * radius * (f64::from(idx) / f64::from(n) - 0.5) * 2.0;
                        let p = Point3::new(q(i), q(j), q(k));
                        let truth = p.coords.norm() - radius;
                        // Skip the tessellation's own uncertainty band: within the chord
                        // error the polyhedron and the ideal sphere genuinely differ.
                        if truth.abs() < 0.005 * radius {
                            continue;
                        }
                        assert_eq!(
                            sdf.distance(p) < 0.0,
                            truth < 0.0,
                            "r={radius}: sign wrong at {p:?} — oracle {:.6}, truth {truth:.6}",
                            sdf.distance(p),
                        );
                    }
                }
            }
        }
    }

    /// Scale every vertex of `m` by exactly `s` — no re-tessellation, so the result is the
    /// base mesh times `s` and nothing else.
    fn scaled(m: &IndexedMesh, s: f64) -> IndexedMesh {
        let mut out = m.clone();
        for v in &mut out.vertices {
            *v = Point3::new(v.x * s, v.y * s, v.z * s);
        }
        out
    }

    /// **Is this oracle bit-identical under a power-of-two rescale?**
    ///
    /// The scale sensitivity pinned by
    /// `pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use` comes from a
    /// **absolute** epsilon applied to a quantity with dimension. The obvious cure is for the
    /// oracle to normalise internally instead of trusting the caller's units — but a
    /// normalisation that perturbs results would churn every golden in the workspace, which
    /// makes it far more expensive than the bug.
    ///
    /// **Unless the factor is a power of two.** Scaling a float by `2^k` only shifts its
    /// exponent, so it is exact; every operation the query performs — subtraction, dot,
    /// cross, `sqrt` (the exponent shift is even), division — commutes with that scaling; and
    /// the `f64 -> f32` narrowing parry does at construction rounds the same way before and
    /// after. If that reasoning holds end to end, an internal power-of-two normalisation is
    /// **free**: bit-identical for every mesh already in a good regime, and different only
    /// where the answer was already wrong.
    ///
    /// This measures whether it holds rather than trusting the argument, because the argument
    /// runs through a third-party BVH build, an f32 narrowing and a pseudo-normal
    /// accumulation, any of which could carry a scale-dependent decision the reasoning does
    /// not see.
    ///
    /// The **non-power-of-two control is the load-bearing half**: without it, a pass here
    /// would be consistent with "this oracle is insensitive to scale", which is exactly the
    /// claim the sibling test refutes. The control has to lose bits for the power-of-two
    /// property to be doing any work.
    ///
    /// ## The claim splits in two, and the split is the result
    ///
    /// The first version of this test swept `2^-20 … 2^20` demanding full signed bit-identity
    /// and claimed in prose that radius 1.0 kept every copy clear of the area floor. It did
    /// not: at `2^-20` this fixture's triangles land near `1e-15`, far under `5.96e-8`. So the
    /// low rows sat in the *broken* regime, and the test failed there — but **it failed
    /// informatively**: `got -1.94563862372096019e0` against `want 1.94563862372096019e0`.
    /// Identical to all 17 digits, opposite sign.
    ///
    /// That is the whole answer. The **magnitude** survives a `2^-20` round trip bit for bit,
    /// so the arithmetic genuinely is scale-neutral under powers of two; the only
    /// scale-dependent thing in the entire pipeline is the epsilon **decision**. So this now
    /// asserts magnitude-identity at *every* power-of-two scale, and full signed identity only
    /// where the fixture is clear of the floor — with the regime **computed** from the
    /// fixture's own smallest triangle rather than asserted in a comment.
    ///
    /// **What that buys the design:** an internal power-of-two normalisation cannot move a
    /// magnitude, ever, and can only change a sign that was already decided by the floor. It
    /// is free for every consumer currently in a good regime.
    #[test]
    fn power_of_two_rescale_is_bit_identical() {
        let base = uv_sphere(1.0, 24, 48);
        let sdf1 = build_sdf(base.clone());

        // The regime boundary, computed from the fixture instead of assumed about it.
        let floor = f64::from(f32::EPSILON) / 2.0;
        let min_area = base
            .faces
            .iter()
            .map(|f| {
                let (a, b, c) = (
                    base.vertices[f[0] as usize],
                    base.vertices[f[1] as usize],
                    base.vertices[f[2] as usize],
                );
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .fold(f64::INFINITY, f64::min);

        // Probe points spanning inside, surface-adjacent, and well outside.
        let n = 6;
        let probes: Vec<Point3<f64>> = (0..=n)
            .flat_map(|i| {
                (0..=n).flat_map(move |j| {
                    (0..=n).map(move |k| {
                        let q = |idx: i32| 1.7 * (f64::from(idx) / f64::from(n) - 0.5) * 2.0;
                        Point3::new(q(i), q(j), q(k))
                    })
                })
            })
            .collect();

        // ── Powers of two. Magnitude must be bit-identical at EVERY scale; the sign only
        //    where the fixture is clear of the floor. ──
        let (mut n_in, mut n_out) = (0usize, 0usize);
        for k in [-20_i32, -10, -3, -1, 1, 3, 10, 20] {
            let s = 2.0_f64.powi(k);
            let in_regime = min_area * s * s > floor;
            if in_regime {
                n_in += 1;
            } else {
                n_out += 1;
            }
            let sdf_s = build_sdf(scaled(&base, s));
            let mut checked = 0usize;
            for &p in &probes {
                let want = sdf1.distance(p);
                let got = sdf_s.distance(Point3::new(p.x * s, p.y * s, p.z * s)) / s;
                // THE ARITHMETIC CLAIM — holds even where the sign is broken, which is what
                // isolates the epsilon decision as the sole scale-dependent step.
                assert_eq!(
                    got.abs().to_bits(),
                    want.abs().to_bits(),
                    "2^{k}: MAGNITUDE is not bit-neutral at {p:?} — got {got:.17e}, want \
                     {want:.17e}. Then power-of-two scaling is not exact through this \
                     pipeline and an internal normalisation would move goldens."
                );
                if in_regime {
                    assert_eq!(
                        got.to_bits(),
                        want.to_bits(),
                        "2^{k}: SIGN differs at {p:?} while the fixture is clear of the area \
                         floor (min scaled area {:.3e} > {floor:.3e}) — got {got:.17e}, want \
                         {want:.17e}. In-regime, a power-of-two rescale must change nothing.",
                        min_area * s * s,
                    );
                }
                checked += 1;
            }
            assert!(checked > 100, "2^{k}: too few probes to mean anything");
        }
        // Non-vacuity: the sweep has to straddle the boundary, or one of the two claims above
        // was never exercised and this test proves half of what it says.
        assert!(
            n_in > 0 && n_out > 0,
            "the sweep must contain both in-regime ({n_in}) and out-of-regime ({n_out}) \
             scales — otherwise one of the two assertions never ran"
        );

        // ── The control: a non-power-of-two scale must NOT be bit-neutral. ──
        // Asserted as "at least one probe differs", not "all differ": individual points can
        // coincide by luck, and requiring all of them would make this flake for a reason that
        // has nothing to do with the property.
        for s in [1.0e-3_f64, 0.3, 7.0] {
            let sdf_s = build_sdf(scaled(&base, s));
            let differing = probes
                .iter()
                .filter(|&&p| {
                    let want = sdf1.distance(p);
                    let got = sdf_s.distance(Point3::new(p.x * s, p.y * s, p.z * s)) / s;
                    got.to_bits() != want.to_bits()
                })
                .count();
            assert!(
                differing > 0,
                "scale {s} came out bit-identical everywhere — then this oracle is insensitive \
                 to scale in general, the power-of-two result above proves nothing specific, \
                 and the sibling scale-regime test should be failing too"
            );
        }
    }
}
