//! Construction-time census of the artifact [`TriMeshDistance`] hands to parry.
//!
//! # Why this exists
//!
//! Nothing in the geometry pipeline validates that an artifact is what it claims. A surface
//! goes in, a signed oracle comes out, and the first thing that checks anything is a physics
//! assert several stages downstream — which is how the FSU disc came to mesh phantom
//! material, and how the diagnosis took four hand-offs to reach.
//!
//! [`SurfaceHealth`] is the missing check, in the only form that is safe to add: it
//! **reports**. See [`TriMeshDistance::health`] for why it does not refuse.
//!
//! # ★ Three layers, and keeping them apart is the point
//!
//! This arc's recurring failure is measuring a proxy and acting as though it were the
//! consequence. So the report states all three, separately, and claims nothing about the
//! relationship between them — which is known to be **non-linear**: a single zeroed
//! pseudo-normal reports "inside" at any distance, so 0.3 % of triangles under the floor can
//! stuff phantom material as far from the surface as 30 % can.
//!
//! | layer | fields | computed from |
//! |---|---|---|
//! | **proxy** | `min_area_caller`, `faces_under_floor_caller` | the caller's mesh, in f64 |
//! | **decision** | `min_area_internal`, `median_area_internal`, `faces_skipped`, `area_margin_binades_achieved`, `clamped` | the built f32 artifact, by parry's own test |
//! | **consequence** | `zero_pseudo_normal_vertices` / `_edges`, `zero_face_normals` | parry's own pseudo-normal arrays |
//!
//! One invariant does hold across two of the layers, and it is worth knowing because it is
//! the only inequality here that is not an empirical question: an exactly degenerate
//! triangle has `‖ab × ac‖² = 0`, which is trivially under any epsilon, so
//! `zero_face_normals <= faces_skipped` always.
//!
//! The consequence layer is the one that matters, and it is read off the artifact that will
//! actually be queried rather than re-derived — `TriMesh::pseudo_normals()` is public, so
//! there is no reason to model what parry did when parry will say.

use std::collections::HashSet;
use std::fmt;

use mesh_types::IndexedMesh;
use parry3d::math::{DEFAULT_EPSILON, Vector as ParryVector};
use parry3d::shape::TriMesh;

use crate::sdf::{AREA_MARGIN_BINADES, ScaleRule, TriMeshDistance, area_floor, scale_rule};

/// What [`TriMeshDistance::health`] found in the oracle's own BVH.
///
/// Every field is a count or a measured quantity — there is no verdict, no score and no
/// threshold. Read the module docs for why the three layers are kept apart.
///
/// Plain public fields and `Debug + Clone`, matching [`crate::FloodFillReport`] — the
/// crate's other construction-time diagnostic, and the one whose report-don't-refuse shape
/// this deliberately copies.
#[derive(Debug, Clone)]
pub struct SurfaceHealth {
    // ── the artifact ──
    /// Triangles in the surface parry meshes.
    pub faces: usize,
    /// Vertices at least one face names.
    ///
    /// ⚠ **Not `mesh.vertices.len()`.** A vertex no face references is not part of the
    /// surface — parry's QBVH is built from triangle AABBs, so such a vertex can never be a
    /// closest feature — yet it keeps an all-zero entry in `vertices_pseudo_normal` for
    /// ever. Counting over the whole vertex buffer would report a fleet of defects on
    /// `cf-cap-planes::dome_wall_only_mesh`, which strands vertices **on purpose** and says
    /// so in its own doc. This is the denominator `zero_pseudo_normal_vertices` is a
    /// numerator of.
    ///
    /// ★ **Against `3 * faces` it also tells you how the surface is welded, which is what
    /// makes every count below interpretable.** Equal to `3 * faces` means triangle soup —
    /// each triangle owns its vertices and shares no edge — and there a *single* skipped
    /// triangle zeroes three vertices and three edges at once, with no neighbours to carry
    /// them. Well below it means welded, where a vertex has many incident triangles and
    /// survives losing one: the FSU disc needed ~30 % of its triangles under the floor
    /// before it broke. **That 30 % is a welded-mesh figure and does not transfer.**
    /// `mesh-offset`'s marching cubes emits soup and `mesh-lattice` consumes it.
    pub referenced_vertices: usize,
    /// The internal power-of-two scale the BVH was built at.
    pub scale: f64,

    // ── layer 1: the proxy, in the caller's frame ──
    /// Smallest **positive** triangle area in the caller's units.
    ///
    /// Positive-only, matching the scale rule: an exactly degenerate triangle is counted by
    /// `zero_face_normals` instead, because no scale can lift a zero off the floor.
    pub min_area_caller: f64,
    /// Triangles at or under parry's area floor (`f32::EPSILON / 2`) **in the caller's
    /// frame** — i.e. how many parry would have skipped had the oracle been built in the
    /// caller's units, as it was before internal normalisation.
    ///
    /// This is the number the FSU diagnosis was written in (4342 of 14489 for the scanned
    /// L4–L5 disc, once rescaled to metres) and the reason the field is worth keeping: it
    /// says how much danger internal normalisation took the caller out of. It is an f64
    /// model of a decision parry makes in f32; `faces_skipped` is the real one.
    ///
    /// ⚠ **Includes exactly degenerate triangles, where `min_area_caller` excludes them.**
    /// The asymmetry is deliberate and neither half is free to change: the minimum feeds
    /// the scale rule, which has nothing to say about a zero, while the count reproduces
    /// `cf_fsu_model::report_area_floor_margin`'s `area <= floor` — the definition the
    /// known value this instrument is checked against was measured under.
    pub faces_under_floor_caller: usize,

    // ── layer 2: the decision, in the frame parry actually sees ──
    /// Smallest positive triangle area in the built f32 artifact.
    ///
    /// Positive-only for the same reason as `min_area_caller`, which means
    /// `area_margin_binades_achieved` — derived from this — describes the margin of the
    /// triangles a scale can actually help. The ones it cannot are `zero_face_normals`.
    pub min_area_internal: f64,
    /// Median triangle area in the built f32 artifact.
    ///
    /// ⚠ **The min is a reduction, and this arc has already established it is a weak
    /// predictor** — `mesh-lattice`'s marching-cubes output carries 1e-30 slivers routinely
    /// and signs perfectly. The median says whether the min is one outlier or the whole mesh
    /// sits close to the floor, which is the distinction the min alone cannot make.
    pub median_area_internal: f64,
    /// Triangles parry **actually skipped** when accumulating pseudo-normals, by its own
    /// test (`‖ab × ac‖² > f32::EPSILON²`) on the f32 coordinates it built.
    ///
    /// A skipped triangle contributes nothing to its vertices' or edges' pseudo-normals. It
    /// is still meshed, still projected onto, and still signed correctly wherever its
    /// *interior* is the closest feature — see `zero_face_normals`, which is always a
    /// subset of this count.
    pub faces_skipped: usize,
    /// Binades of area margin over the floor the smallest positive triangle actually
    /// achieved, `log2(min_area_internal / floor)`.
    pub area_margin_binades_achieved: f64,
    /// Binades of area margin the scale rule asked for.
    pub area_margin_binades_requested: i32,
    /// Whether the f32 coordinate cap stopped the rule reaching the requested margin.
    ///
    /// **α.1's obligation made visible.** The rule clamps rather than failing, which is
    /// right, but a clamp that nothing reports is a silent shortfall — exactly the class of
    /// hand-off this arc exists to close. A property of the mesh, so it is meaningful even
    /// for an oracle built at an explicit scale.
    pub clamped: bool,

    // ── layer 3: the consequence, read off parry's own arrays ──
    /// Referenced vertices whose pseudo-normal is exactly zero.
    ///
    /// **This is the defect, not a proxy for it.** Parry's inside test is
    /// `dpt.dot(&pseudo_normal) <= 0.0`, which a zero vector satisfies unconditionally — so
    /// a query whose closest feature is such a vertex is reported **inside at any
    /// distance**.
    pub zero_pseudo_normal_vertices: usize,
    /// Distinct edges whose pseudo-normal is exactly zero — same consequence, via the
    /// closest-feature-is-an-edge branch.
    ///
    /// ⚠ Deduped. Parry stores `edges_pseudo_normal` **per triangle, three per face**, so
    /// every interior edge appears twice; counting the raw array inflates this ~2×.
    pub zero_pseudo_normal_edges: usize,
    /// Distinct edges named by at least one face — the denominator for the above.
    pub distinct_edges: usize,
    /// Faces whose f32 cross product is exactly zero.
    ///
    /// ★ **The one channel internal normalisation can never repair**, and the reason it is
    /// counted apart from `faces_skipped`. For a query whose closest feature is a face
    /// *interior*, parry does not consult the pseudo-normal arrays at all — it uses
    /// `Triangle::scaled_normal()`, the raw cross product, with no epsilon. So a merely
    /// sub-floor triangle still signs its own interior correctly. An **exactly** degenerate
    /// one does not: its scaled normal is the zero vector, and the same
    /// `dot(..) <= 0.0` reads "inside". No power of two lifts a zero, which is why the scale
    /// rule skips these rather than trying.
    pub zero_face_normals: usize,
}

impl fmt::Display for SurfaceHealth {
    /// One line, for callers that want to log the census without unpacking it.
    ///
    /// Deliberately not `tracing`: `mesh-sdf` is an L0 crate with five workspace
    /// dependencies, and a diagnostic is not worth a logging framework in the dependency
    /// graph of every consumer.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} faces / {} referenced vertices | scale 2^{:.0} | area margin {:.1} of {} binades\
             {} | min area {:.3e} caller -> {:.3e} internal (median {:.3e}) | under floor: {} \
             caller, {} skipped by parry | ZEROED: {} vertices, {} of {} edges, {} face normals",
            self.faces,
            self.referenced_vertices,
            self.scale.log2(),
            self.area_margin_binades_achieved,
            self.area_margin_binades_requested,
            if self.clamped { " (CLAMPED)" } else { "" },
            self.min_area_caller,
            self.min_area_internal,
            self.median_area_internal,
            self.faces_under_floor_caller,
            self.faces_skipped,
            self.zero_pseudo_normal_vertices,
            self.zero_pseudo_normal_edges,
            self.distinct_edges,
            self.zero_face_normals,
        )
    }
}

impl SurfaceHealth {
    /// Is every feature of this surface signable?
    ///
    /// True iff no vertex, edge or face normal parry can pick as a closest feature is zero
    /// — i.e. there is no point in space at which this oracle reports "inside"
    /// unconditionally.
    ///
    /// ⚠ **Not a quality score, and deliberately not the negation of one.** It says nothing
    /// about the margin, the clamp, or how close to the floor the mesh sits; a surface can
    /// be fully signable with 99 % of its triangles under the floor, and `mesh-lattice`'s
    /// output routinely is.
    #[must_use]
    pub fn is_fully_signable(&self) -> bool {
        self.zero_pseudo_normal_vertices == 0
            && self.zero_pseudo_normal_edges == 0
            && self.zero_face_normals == 0
    }
}

impl TriMeshDistance {
    /// Census the BVH this oracle built: how far its geometry sits from parry's area floor,
    /// and — the part that matters — whether any feature of it has been left **unsignable**.
    ///
    /// Computed on demand from the artifact already in hand, so construction pays nothing
    /// for a caller that never asks. `O(faces)`.
    ///
    /// # ★★ It reports. It does not refuse.
    ///
    /// [`crate::SdfError`]'s own doc makes this layer the one *entitled* to refuse — it
    /// measures the consequence, where the scale rule can only see a proxy. It does not
    /// exercise that entitlement, on purpose:
    ///
    /// - Making a constructor fatal on a count nobody has measured yet is precisely the
    ///   mistake the scale rule made one rung earlier. Its first version errored on a
    ///   proxy, rejected `mesh-lattice`'s perfectly signable marching-cubes output, and took
    ///   six gates red.
    /// - A zeroed feature is not obviously fatal even so. It is wrong only for queries whose
    ///   closest feature is that one vertex or edge; whether any consumer ever queries there
    ///   is a fact about the consumer, not about the mesh.
    ///
    /// Revisit when the census has readings, not before.
    ///
    /// # Panics
    ///
    /// Does not panic, but the guarantee is parry's rather than this function's and is
    /// worth stating where it comes from. Walking the artifact indexes its vertex buffer by
    /// face index — and a face naming a vertex the mesh does not have would already have
    /// panicked *inside* `TriMesh::with_flags`, whose `compute_pseudo_normals` indexes
    /// unguarded, long before any oracle existed to ask. `TriMeshDistance::new` rejects the
    /// same input earlier still, with [`crate::SdfError::FaceIndexOutOfRange`].
    #[must_use]
    pub fn health(&self) -> SurfaceHealth {
        let tri_mesh = self.shared_tri_mesh();
        let floor = area_floor();

        // ── layer 1: the proxy, from the caller's mesh in f64 ──
        let (min_area_caller, faces_under_floor_caller) = caller_frame_areas(self.mesh(), floor);

        // ── layer 2: the decision, from the f32 artifact, by parry's own test ──
        let (min_area_internal, median_area_internal, faces_skipped, zero_face_normals) =
            internal_frame_areas(tri_mesh);

        // ── layer 3: the consequence, from parry's own arrays ──
        let (referenced_vertices, distinct_edges, zero_v, zero_e) = zeroed_features(tri_mesh);

        // The one relation between layers that is not an empirical question, enforced where
        // it is computed rather than only asserted in the module docs. A prose invariant
        // nothing checks is a prose invariant that rots — and this one is free: an exactly
        // zero cross product is trivially at or under any epsilon, so a face counted in
        // `zero_face_normals` must also have been counted in `faces_skipped`.
        debug_assert!(
            zero_face_normals <= faces_skipped,
            "{zero_face_normals} faces have an exactly zero cross product but only \
             {faces_skipped} were skipped — the two are computed from the same cross \
             products and the first is a subset of the second by construction"
        );

        // The clamp is a property of the mesh, evaluated by the same rule that ships. A
        // mesh with no positive-area triangle cannot reach here through `new`, and a
        // `with_scale` caller that forced one through gets `clamped = false` rather than a
        // panic — the honest answer being that the rule was never consulted.
        let rule = scale_rule(self.mesh(), AREA_MARGIN_BINADES).ok();

        SurfaceHealth {
            faces: tri_mesh.indices().len(),
            referenced_vertices,
            scale: self.scale(),
            min_area_caller,
            faces_under_floor_caller,
            min_area_internal,
            median_area_internal,
            faces_skipped,
            area_margin_binades_achieved: (min_area_internal / floor).log2(),
            area_margin_binades_requested: AREA_MARGIN_BINADES,
            clamped: rule.is_some_and(ScaleRule::clamped),
            zero_pseudo_normal_vertices: zero_v,
            zero_pseudo_normal_edges: zero_e,
            distinct_edges,
            zero_face_normals,
        }
    }
}

/// Smallest positive triangle area and the count at or under `floor`, in the caller's units.
///
/// f64 throughout, and `<=` against the floor, so this reproduces the definition the FSU
/// diagnosis was written in (`cf_fsu_model`'s `report_area_floor_margin`) exactly. That
/// agreement is what lets a brand-new instrument be checked against a known value instead of
/// being trusted.
fn caller_frame_areas(mesh: &IndexedMesh, floor: f64) -> (f64, usize) {
    let mut min_area = f64::INFINITY;
    let mut under = 0usize;
    for f in &mesh.faces {
        let (Some(&a), Some(&b), Some(&c)) = (
            mesh.vertices.get(f[0] as usize),
            mesh.vertices.get(f[1] as usize),
            mesh.vertices.get(f[2] as usize),
        ) else {
            // Unreachable through `new`, which rejects an out-of-range index outright. A
            // face that does not resolve has no area to contribute either way.
            continue;
        };
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        if area <= floor {
            under += 1;
        }
        if area > 0.0 {
            min_area = min_area.min(area);
        }
    }
    (min_area, under)
}

/// Areas of the built f32 artifact, and how many triangles parry drops.
///
/// ⚠ **Computed the way parry computes it, not the way the caller's mesh would suggest.**
/// `Triangle::normal` is `Unit::try_new(ab × ac, DEFAULT_EPSILON)`, and `Unit::try_new` keeps
/// a vector iff `‖v‖² > eps²` — all in f32, on the narrowed coordinates. Modelling that in
/// f64 would agree almost everywhere and disagree exactly at the boundary this instrument
/// exists to describe.
///
/// The epsilon is taken from `parry3d::math::DEFAULT_EPSILON` rather than written out as
/// `f32::EPSILON`. They are the same value today — `DEFAULT_EPSILON` is *defined* as
/// `Real::EPSILON` — and that is exactly why the constant is read instead of reproduced: if
/// parry ever moves it, a census that hardcoded the identity would keep reporting a decision
/// parry had stopped making.
///
/// Returns `(min positive area, median area, skipped, exactly degenerate)`.
fn internal_frame_areas(tri_mesh: &TriMesh) -> (f64, f64, usize, usize) {
    let eps = DEFAULT_EPSILON;
    let vertices = tri_mesh.vertices();
    let mut areas: Vec<f64> = Vec::with_capacity(tri_mesh.indices().len());
    let mut skipped = 0usize;
    let mut degenerate = 0usize;

    for idx in tri_mesh.indices() {
        let (a, b, c) = (
            vertices[idx[0] as usize],
            vertices[idx[1] as usize],
            vertices[idx[2] as usize],
        );
        let cross = (b - a).cross(&(c - a));
        // Parry's exact test, in parry's own precision.
        if cross.norm_squared() <= eps * eps {
            skipped += 1;
        }
        // The face branch of the sign consults `scaled_normal()` directly, with no epsilon,
        // so only an exact zero breaks it.
        if cross == ParryVector::zeros() {
            degenerate += 1;
        }
        areas.push(f64::from(cross.norm()) / 2.0);
    }

    let min_positive = areas
        .iter()
        .copied()
        .filter(|a| *a > 0.0)
        .fold(f64::INFINITY, f64::min);
    areas.sort_by(f64::total_cmp);
    let median = areas.get(areas.len() / 2).copied().unwrap_or(f64::NAN);
    (min_positive, median, skipped, degenerate)
}

/// Walk parry's own pseudo-normal arrays and count the features it left at zero.
///
/// Returns `(referenced vertices, distinct edges, zeroed vertices, zeroed edges)`.
///
/// ⚠ Two indexing traps, both of which inflate the answer if missed:
/// - `vertices_pseudo_normal` is sized to the **whole** vertex buffer, so a vertex no face
///   references sits at zero for ever without being a defect. Only referenced vertices are
///   counted.
/// - `edges_pseudo_normal` is `Vec<[Vector; 3]>` indexed **per triangle**, and its slots are
///   the edges `(i0,i1)`, `(i1,i2)`, `(i2,i0)` in that order. Every interior edge therefore
///   appears twice. Deduped on the sorted endpoint pair, which is the key parry itself built
///   the values under.
fn zeroed_features(tri_mesh: &TriMesh) -> (usize, usize, usize, usize) {
    let indices = tri_mesh.indices();
    let mut referenced: HashSet<u32> = HashSet::new();
    let mut edges: HashSet<(u32, u32)> = HashSet::new();
    let mut zero_edges: HashSet<(u32, u32)> = HashSet::new();

    let sorted = |a: u32, b: u32| if a <= b { (a, b) } else { (b, a) };
    let pseudo_normals = tri_mesh.pseudo_normals();

    for (face, idx) in indices.iter().enumerate() {
        for &v in idx {
            referenced.insert(v);
        }
        let slots = [
            sorted(idx[0], idx[1]),
            sorted(idx[1], idx[2]),
            sorted(idx[2], idx[0]),
        ];
        for (slot, edge) in slots.into_iter().enumerate() {
            edges.insert(edge);
            let is_zero = pseudo_normals.is_some_and(|pn| {
                pn.edges_pseudo_normal
                    .get(face)
                    .is_some_and(|e| e[slot] == ParryVector::zeros())
            });
            if is_zero {
                zero_edges.insert(edge);
            }
        }
    }

    let zero_vertices = pseudo_normals.map_or(0, |pn| {
        referenced
            .iter()
            .filter(|&&v| {
                pn.vertices_pseudo_normal
                    .get(v as usize)
                    .is_some_and(|n| *n == ParryVector::zeros())
            })
            .count()
    });

    (
        referenced.len(),
        edges.len(),
        zero_vertices,
        zero_edges.len(),
    )
}

#[cfg(test)]
mod tests {
    // `unwrap`/`expect` are denied in library code; tests may use them.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use crate::oracle::Signed;
    use crate::sdf::PseudoNormalSign;
    use crate::test_fixtures::{build_sdf_at, uv_sphere};
    use nalgebra::Point3;

    /// Count probes on a lattice whose sign the oracle gets wrong against the analytic
    /// sphere, skipping the tessellation's own uncertainty band.
    fn wrong_signs(sdf: &Signed<TriMeshDistance, PseudoNormalSign>, radius: f64) -> (usize, usize) {
        let n = 10;
        let (mut wrong, mut checked) = (0usize, 0usize);
        for i in 0..=n {
            for j in 0..=n {
                for k in 0..=n {
                    let q = |idx: i32| 1.6 * radius * (f64::from(idx) / f64::from(n) - 0.5) * 2.0;
                    let p = Point3::new(q(i), q(j), q(k));
                    let truth = p.coords.norm() - radius;
                    if truth.abs() < 0.005 * radius {
                        continue;
                    }
                    checked += 1;
                    if (sdf.distance(p) < 0.0) != (truth < 0.0) {
                        wrong += 1;
                    }
                }
            }
        }
        (wrong, checked)
    }

    /// **The census sees the same defect the sign test sees — that is the whole claim.**
    ///
    /// `SurfaceHealth` exists to report a consequence, so the thing worth pinning is not
    /// that its counters move but that they move *with the oracle's actual wrongness*. A
    /// counter that lit up on healthy meshes, or stayed dark on broken ones, would be worse
    /// than no instrument: it would be an instrument that gets believed.
    ///
    /// So both arms are measured on both axes, on a fixture that genuinely has the bug — a
    /// sphere of radius 1e-3, whose pole-fan triangles sit under parry's area floor:
    ///
    /// | | zeroed features | wrong signs |
    /// |---|---|---|
    /// | `with_scale(mesh, 1.0)` — the pre-α.1 oracle | **> 0** | **> 0** |
    /// | `new(mesh)` — internally normalised | **0** | **0** |
    ///
    /// ⚠ **The broken arm is the load-bearing half.** Asserting only that the fixed oracle
    /// is clean would pass just as well on a fixture that never had the bug, and on a
    /// census hard-coded to return zero.
    ///
    /// ## Its relationship to `internal_normalisation_fixes_the_sign_below_the_area_floor`
    ///
    /// The two overlap on purpose and neither is redundant. That one is **α.1's** claim —
    /// internal normalisation repairs the sign — and it owns the radius sweep (1e-3 and
    /// 1e-4) and the sign counts. This is **α.2's**: the census reports what that repair
    /// fixed. It borrows the sign measurement only as the anchor its counters are checked
    /// against, because a census validated against nothing is the failure mode this whole
    /// arc is about. Delete the sibling and this one still passes while meaning much less.
    #[test]
    fn the_census_moves_with_the_oracle_s_actual_wrongness() {
        let radius = 1e-3;
        let mesh = uv_sphere(radius, 24, 48);

        let broken = build_sdf_at(&mesh, 1.0);
        let broken_health = broken.distance.health();
        let (broken_wrong, checked) = wrong_signs(&broken, radius);

        let fixed_distance = TriMeshDistance::new(mesh.clone()).expect("sphere is scalable");
        let fixed_sign = PseudoNormalSign::from_distance(&fixed_distance);
        let fixed = Signed {
            distance: fixed_distance,
            sign: fixed_sign,
        };
        let fixed_health = fixed.distance.health();
        let (fixed_wrong, _) = wrong_signs(&fixed, radius);

        println!(
            "\nbroken (scale 1.0): {broken_health}\n  -> {broken_wrong}/{checked} signs wrong"
        );
        println!("fixed  (normalised): {fixed_health}\n  -> {fixed_wrong}/{checked} signs wrong");

        assert!(checked > 500, "only {checked} probes cleared the band");

        // ── The broken arm: the census must SEE the defect the signs demonstrate. ──
        assert!(
            broken_wrong > 0,
            "the pre-α.1 oracle got every sign right, so this fixture does not reproduce \
             the area-floor bug and nothing below is earned"
        );
        assert!(
            broken_health.zero_pseudo_normal_vertices > 0,
            "the oracle gets {broken_wrong} of {checked} signs wrong, yet the census reports \
             no zeroed vertices. Then the census is blind to the very defect it exists to \
             report: {broken_health}"
        );
        assert!(
            !broken_health.is_fully_signable(),
            "`is_fully_signable` says yes on an oracle that gets {broken_wrong} of {checked} \
             signs wrong: {broken_health}"
        );

        // ── The fixed arm: no zeroed features, and no wrong signs to go with them. ──
        assert_eq!(
            fixed_wrong, 0,
            "the normalised oracle still gets {fixed_wrong} of {checked} signs wrong"
        );
        assert!(
            fixed_health.is_fully_signable(),
            "the normalised oracle signs every probe correctly, yet the census still reports \
             zeroed features — it is crying wolf on a mesh that works: {fixed_health}"
        );

        // The proxy layer moved too, and in the direction that makes the caller-frame
        // column worth carrying: the caller's own units were deep under the floor.
        assert!(
            broken_health.faces_under_floor_caller > 0,
            "the fixture is not under the floor in the caller's frame: {broken_health}"
        );
    }

    /// **A vertex no face references must not be counted as a defect.**
    ///
    /// `vertices_pseudo_normal` is sized to the whole vertex buffer, so a stranded vertex
    /// keeps an all-zero entry for ever — while being no part of the surface parry meshes,
    /// since the QBVH is built from triangle AABBs and no query can ever land on it.
    /// Counting the raw array would report a fleet of unsignable features on
    /// `cf-cap-planes::dome_wall_only_mesh`, which strands vertices **on purpose** and says
    /// so in its own doc.
    ///
    /// This is α.1's `extent`-over-`mesh.vertices` bug one rung later, and the reason it is
    /// worth a gate rather than a comment: the census would have been wrong in exactly the
    /// way the scale rule already was, on exactly the same contract.
    ///
    /// ⚠ **The fixture must actually strand a vertex, and the obvious choice does not.**
    /// The sweep tried `dome_wall_only_mesh` on a cube: stripping the cap leaves all eight
    /// corners referenced by the remaining faces, so `referenced_vertices` is 8 either way
    /// and the test would pass with the defect fully reintroduced. Hence the explicit
    /// append, and the assert below that the mesh really does carry an unreferenced vertex.
    #[test]
    fn a_stranded_vertex_is_not_counted_as_an_unsignable_feature() {
        let clean = uv_sphere(1.0, 24, 48);
        let mut stranded = clean.clone();
        stranded.vertices.push(Point3::new(99.0, 99.0, 99.0));

        let clean_health = TriMeshDistance::new(clean.clone())
            .expect("scalable")
            .health();
        let stranded_health = TriMeshDistance::new(stranded.clone())
            .expect("scalable")
            .health();

        // Non-vacuity: there IS a stranded vertex here for a naive count to trip over.
        assert_eq!(
            stranded.vertices.len(),
            clean.vertices.len() + 1,
            "fixture did not gain a vertex"
        );
        assert!(
            stranded_health.referenced_vertices < stranded.vertices.len(),
            "the mesh carries no unreferenced vertex, so a count over the whole vertex \
             buffer would agree with a count over face corners and this gate is blind: \
             {stranded_health}"
        );

        assert_eq!(
            stranded_health.referenced_vertices, clean_health.referenced_vertices,
            "appending a vertex no face references changed the referenced-vertex count \
             ({} -> {})",
            clean_health.referenced_vertices, stranded_health.referenced_vertices,
        );
        assert_eq!(
            stranded_health.zero_pseudo_normal_vertices, 0,
            "a vertex no face references was counted as an unsignable feature. parry gives \
             it an all-zero pseudo-normal because nothing accumulates into it, but no query \
             can ever land there — the QBVH is built from triangles: {stranded_health}"
        );
        assert!(
            stranded_health.is_fully_signable(),
            "a mesh whose only oddity is an unreferenced vertex is reported unsignable: \
             {stranded_health}"
        );
    }

    /// **Whether the counts are alarming depends on how the mesh is welded, and the report
    /// carries the tell.**
    ///
    /// `referenced_vertices` against `3 × faces` says whether a surface is welded or
    /// triangle soup, and that decides how far one skipped triangle propagates:
    ///
    /// - **Welded** — a vertex has many incident triangles, so it survives losing one. The
    ///   FSU disc needed **~30 %** of its triangles under the floor before it broke.
    /// - **Soup** — every triangle owns its three vertices and shares no edge, so a single
    ///   skipped triangle zeroes three vertices and three edges *immediately*. There are no
    ///   neighbours to carry them.
    ///
    /// So the "30 %" figure is a property of a welded mesh and **does not transfer**.
    /// `mesh-offset`'s marching cubes emits soup, and `mesh-lattice` consumes it — measured
    /// during α.2's sweep at exactly `3 × faces` for both counters, at every resolution.
    ///
    /// This is pinned because it is the interpretation key for every count in the report,
    /// and because reading the numbers without it invites exactly the linear-extrapolation
    /// mistake the arc has already made once with the disc's 30 %.
    #[test]
    fn the_report_distinguishes_a_welded_surface_from_triangle_soup() {
        let welded = uv_sphere(1.0, 24, 48);
        let welded_health = TriMeshDistance::new(welded.clone())
            .expect("scalable")
            .health();

        // Unweld it: give every triangle its own three vertices, changing no geometry.
        let mut soup = IndexedMesh::new();
        for f in &welded.faces {
            let base = u32::try_from(soup.vertices.len()).expect("fits");
            for &i in f {
                soup.vertices.push(welded.vertices[i as usize]);
            }
            soup.faces.push([base, base + 1, base + 2]);
        }
        let soup_health = TriMeshDistance::new(soup.clone())
            .expect("scalable")
            .health();

        println!("\nwelded: {welded_health}\nsoup:   {soup_health}");

        assert_eq!(
            soup.faces.len(),
            welded.faces.len(),
            "unwelding changed the face count"
        );
        assert_eq!(
            soup_health.referenced_vertices,
            soup_health.faces * 3,
            "soup must reference exactly three vertices per face: {soup_health}"
        );
        assert_eq!(
            soup_health.distinct_edges,
            soup_health.faces * 3,
            "in soup no edge is shared, so distinct edges must equal 3x faces: {soup_health}"
        );
        assert!(
            welded_health.referenced_vertices * 4 < welded_health.faces * 3,
            "the welded fixture is not meaningfully welded — {} referenced vertices against \
             {} face corners — so the contrast this test draws is not visible: \
             {welded_health}",
            welded_health.referenced_vertices,
            welded_health.faces * 3,
        );

        // ★ Euler's formula, `V - E + F = 2` for a closed surface — a known value from
        //   outside this crate, and the only independent anchor the edge count has.
        //
        //   It is here because the edge dedup is otherwise unpinned: parry stores
        //   `edges_pseudo_normal` per triangle, three per face, so every interior edge
        //   appears twice and a census that forgot to dedupe would report ~2x. Nothing else
        //   in the suite would notice — the soup asserts above pass either way, since soup
        //   shares no edge for a dedup to collapse, and the zeroed-edge counts are only ever
        //   compared against this same total.
        let (v, e, f) = (
            i64::try_from(welded_health.referenced_vertices).expect("fits"),
            i64::try_from(welded_health.distinct_edges).expect("fits"),
            i64::try_from(welded_health.faces).expect("fits"),
        );
        assert_eq!(
            v - e + f,
            2,
            "the welded sphere is closed, so Euler's formula requires V - E + F = 2; got \
             {v} - {e} + {f} = {}. An un-deduplicated edge count lands near -{f}, because \
             parry indexes edge pseudo-normals per TRIANGLE and stores every interior edge \
             twice: {welded_health}",
            v - e + f,
        );

        // Both are perfectly signable. Soup is not a defect; it only changes how far one
        // skipped triangle would propagate, which is what the counts are read against.
        assert!(welded_health.is_fully_signable() && soup_health.is_fully_signable());
    }
}
