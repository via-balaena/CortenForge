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
