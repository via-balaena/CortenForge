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
    /// The scale is chosen from the mesh's own geometry (`choose_scale`) and applied
    /// **inside** the oracle: queries are lifted in and answers brought back, so results
    /// are in the caller's units and the caller's frame never moves. Building the same
    /// surface in millimetres or in metres now gives the same answers — which it did not
    /// before, and is why the FSU disc meshed phantom material once a rescale to SI put
    /// **30 % of its triangles** under parry's area floor.
    ///
    /// # Errors
    ///
    /// - [`SdfError::EmptyMesh`] if `mesh` has no faces.
    /// - [`SdfError::UnscalableMesh`] if the mesh has no positive-area triangle to scale from.
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
        // The contract lives in the doc above and nowhere else, which is not enough for a
        // value that divides every result. Zero silently yields infinite distances,
        // negative mirrors the mesh, and NaN poisons the field — all of which would read
        // as a geometry problem rather than a caller passing the wrong number.
        debug_assert!(
            scale.is_finite() && scale > 0.0,
            "internal scale must be finite and positive, got {scale}"
        );
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
pub(crate) fn area_floor() -> f64 {
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
pub(crate) const AREA_MARGIN_BINADES: i32 = 40;

/// Largest internal coordinate magnitude the BVH may be built at.
///
/// The binding operation is **`norm_squared` of a cross product**, which grows as
/// coordinate⁴: `Triangle::normal` is `Unit::try_new(ab × ac, DEFAULT_EPSILON)`, and
/// `try_new` decides on `value.norm_squared()`. The cross itself is only coordinate², and
/// parry's point-triangle projection carries the same fourth power in its barycentric
/// determinants. So the ceiling is the **fourth root** of `f32::MAX`, not its square root.
///
/// Dividing the coordinate by 2¹⁰ puts the fourth power **forty binades** below overflow —
/// the same margin, and for the same one-sided reason, as [`AREA_MARGIN_BINADES`].
///
/// # ★★ This was wrong until α.2, and the failure is why it is now measured
///
/// It read `sqrt(f32::MAX) / 1024` on the stated grounds that "intermediates grow as
/// coordinate²" — true of the cross, false of the `norm_squared` that actually decides.
/// Six orders too permissive, and the consequence is **worse than an overflow**:
///
/// `Unit::try_new` sees `sq_norm = inf`, finds `inf > eps²`, takes `n = sqrt(inf) = inf`,
/// and divides — so `Triangle::normal()` returns **`Some([0, 0, 0])`, not `None`.** The
/// triangle is not skipped. It is accumulated *with a zero normal*, which zeroes the
/// pseudo-normal of every vertex and edge it touches. It is the area floor's failure
/// arriving from the opposite end of the scale: not "too small, dropped" but "too large,
/// kept and zeroed".
///
/// α.1 shipped this and no gate caught it, because the gate that should have
/// (`a_sliver_clamps_the_scale_instead_of_failing_the_mesh`) computed the lifted area in
/// **f64 on the caller's mesh** and concluded the sliver cleared the floor — while the f32
/// artifact parry had actually built held `inf` there. It measured a model of the artifact
/// instead of the artifact. `TriMeshDistance::health` found it on its first real sweep:
/// one live `mesh-lattice` mesh at scale 2⁴⁸ with **all 40836 vertices and all 122502 edges
/// zeroed**, and `faces_skipped` reading a reassuring 0.
///
/// So `the_coordinate_cap_sits_below_where_parry_overflows` no longer trusts the algebra
/// above: it asks parry where it breaks and requires this constant to be below the answer.
fn coordinate_cap() -> f64 {
    // Fourth root, written as two square roots: `sqrt` is correctly rounded where `powf`
    // is not, and it reads as the power it is.
    f64::from(f32::MAX).sqrt().sqrt() / 1024.0
}

/// Smallest triangle area and largest absolute coordinate, in the caller's units.
///
/// ⚠ **Both quantities are taken over FACE CORNERS, never over `mesh.vertices`.**
/// `cf-cap-planes`'s `dome_wall_only_mesh` strips cap faces and deliberately leaves their
/// vertices behind — its doc names the contract it is relying on: *"the SDF construction
/// tolerates unreferenced vertices (`TriMeshDistance::new` walks faces, not vertices)"* —
/// and it feeds `TriMeshDistance::new` from both `cf-device-geometry` and `cf-cast-cli`.
/// A stranded vertex is not part of the surface parry meshes (the QBVH is built from
/// triangle AABBs), so letting one inflate `extent` would shrink the scale cap on account
/// of geometry that does not exist. `mesh-repair`'s own fixture shows the shape: a
/// stranded vertex at `(99, 99, 99)` on an otherwise unit-scale mesh.
///
/// # Errors
///
/// Returns [`SdfError::FaceIndexOutOfRange`] if a face names a vertex the mesh does not
/// have — indexing would otherwise panic here, and a scale derived from a mesh whose
/// faces do not resolve is meaningless anyway.
fn mesh_scale_inputs(mesh: &IndexedMesh) -> SdfResult<(f64, f64)> {
    let vertex_count = mesh.vertices.len();
    let mut extent = 0.0_f64;
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
        for v in [&a, &b, &c] {
            extent = extent.max(v.x.abs()).max(v.y.abs()).max(v.z.abs());
        }
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        // `f64::min` propagates the non-NaN operand, so a NaN area would be silently
        // dropped and the scale derived from the rest of the mesh. Take it explicitly so
        // a corrupt vertex surfaces as an unscalable mesh rather than a plausible number.
        if area.is_nan() {
            return Ok((f64::NAN, extent));
        }
        // ⚠ Exact zeros are SKIPPED rather than minimised over. No scale lifts a zero off
        //   the floor, so including one would hide the smallest *liftable* triangle behind
        //   a shortfall the rule can never close. A degenerate triangle among many is also
        //   the status quo — parry has always skipped it — so treating it as fatal would
        //   break meshes that work today. The scale is chosen for the triangles it can
        //   actually help.
        if area > 0.0 {
            min_area = min_area.min(area);
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
/// - [`SdfError::UnscalableMesh`] if no triangle has positive finite area (nothing to
///   derive a scale from). A shortfall against the margin is CLAMPED, not fatal — see
///   the reasoning at the clamp.
/// - [`SdfError::FaceIndexOutOfRange`] via [`mesh_scale_inputs`].
pub(crate) fn choose_scale(mesh: &IndexedMesh) -> SdfResult<f64> {
    scale_for_margin(mesh, AREA_MARGIN_BINADES)
}

/// [`choose_scale`] with the margin injectable, so
/// `the_chosen_scale_is_insensitive_to_the_margin_across_decades` can vary the one input
/// whose value is not derived from anything and show the result does not move.
///
/// A constant that cannot be varied cannot be shown not to matter.
///
/// # Errors
///
/// As `choose_scale`.
pub(crate) fn scale_for_margin(mesh: &IndexedMesh, margin_binades: i32) -> SdfResult<f64> {
    Ok(scale_rule(mesh, margin_binades)?.chosen())
}

/// The two exponents [`scale_for_margin`] decides between, before the clamp resolves them.
///
/// Split out because **the guard has to report the clamp, and a second copy of this
/// arithmetic would drift from the one that ships.** `TriMeshDistance::health` needs to say
/// how far short of the requested margin a mesh landed, which means it needs `wanted` — a
/// number `scale_for_margin` used to compute and throw away. One producer, queried twice.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ScaleRule {
    /// Power-of-two exponent that would put the smallest triangle the full margin above
    /// [`area_floor`], ignoring the coordinate cap.
    pub wanted: f64,
    /// Largest exponent [`coordinate_cap`] permits for this mesh's extent.
    pub max_k: f64,
}

impl ScaleRule {
    /// The scale actually built at: `2^clamp(wanted, 0, max_k)`.
    // `k` is clamped to `[0, max_k]`, and `max_k` is finite because `scale_rule`'s guard
    // establishes `extent > 0` and `coordinate_cap()` is a finite constant — so the value
    // cast to i32 is a small non-negative integer and cannot truncate.
    #[allow(clippy::cast_possible_truncation)]
    pub(crate) fn chosen(self) -> f64 {
        let k = self.wanted.clamp(0.0, self.max_k.max(0.0));
        2.0_f64.powi(k as i32)
    }

    /// Did the coordinate cap stop the rule reaching its requested margin?
    ///
    /// A property of the **mesh**, not of the scale that happened to be used, so it is
    /// meaningful even for an oracle built through the `with_scale` test seam.
    pub(crate) fn clamped(self) -> bool {
        self.wanted > self.max_k
    }
}

/// Evaluate the scale rule without resolving the clamp.
///
/// # Errors
///
/// As [`choose_scale`].
pub(crate) fn scale_rule(mesh: &IndexedMesh, margin_binades: i32) -> SdfResult<ScaleRule> {
    let (min_area, extent) = mesh_scale_inputs(mesh)?;
    // Written as a positive test so NaN falls through to the error arm rather than
    // slipping past a negated comparison: NaN is neither `> 0.0` nor `<= 0.0`.
    // `min_area == INFINITY` means no triangle had positive finite area at all.
    let usable = min_area > 0.0 && min_area.is_finite() && extent.is_finite() && extent > 0.0;
    if !usable {
        return Err(SdfError::UnscalableMesh {
            min_area,
            extent,
            reason: "no triangle has a positive finite area, so there is no geometry here to \
                     scale — the mesh is fully degenerate or carries non-finite vertices",
        });
    }

    let target = area_floor() * 2.0_f64.powi(margin_binades);
    // Area goes as length², so the scale needed is the square root of the area shortfall.
    let wanted = (target / min_area).sqrt().log2().ceil();
    let max_k = (coordinate_cap() / extent).log2().floor();

    // ★★ CLAMP, do not fail — and the reason is a measurement, not a preference.
    //
    // The first version of this rule errored when `wanted > max_k`, on the argument that
    // handing back a signed oracle for a surface that cannot be signed is the defect this
    // arc exists to remove. Running it across mesh-sdf's dependents killed that argument
    // in one shot: `mesh-lattice`'s marching-cubes output carries a sliver of area
    // **1.537e-30** across a 48.8-unit mesh — routine output when an isosurface clips a
    // cell corner — and six of its gates went red.
    //
    // Two things were wrong, both proxy errors of the kind this arc keeps catching:
    //
    // 1. **That mesh was never unsignable.** At `max_k = 48` its smallest triangle lands at
    //    1.2e-1, twenty-one binades clear of the floor. The rule rejected it for failing to
    //    reach forty. Greedy, then fatal about it.
    // 2. **"Under the floor" does not mean "cannot be signed."** A skipped triangle only
    //    zeroes a pseudo-normal where *every* triangle incident to a vertex is skipped; the
    //    FSU disc needed ~30 % of its triangles under before it broke, and R1's sweep found
    //    targets leaving the smallest at 0.05x and 0.74x of the floor still meshing
    //    correctly. Erroring on a single sliver measures the proxy, not the consequence.
    //
    // Clamping is also **monotone**: `k` is never below 0, so the result is never worse
    // than the un-normalised oracle every consumer has today. Erroring, by contrast, is a
    // strict regression for meshes that currently work. Reporting the residual belongs to
    // [`TriMeshDistance::health`], which measures the consequence properly — the clamp is
    // resolved by `ScaleRule::chosen` and reported by `ScaleRule::clamped`.
    Ok(ScaleRule { wanted, max_k })
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
    ///   `choose_scale`.
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
    /// phantom material, by building its oracle after rescaling to SI metres —
    /// which put **30 % of the disc's triangles** under the floor — while every
    /// other consumer in the workspace builds in native millimetres.
    ///
    /// ⚠ The 30 % is a count of TRIANGLES, produced by `cf_fsu_model`'s
    /// `report_area_floor_margin`. It is not a measure of how much phantom
    /// material resulted: the re-anchor list puts that at 999 tets, 6.22 % of
    /// kept volume. Three docs in this crate previously attached the triangle
    /// fraction to the word "material"; #711 introduced the conflation and it
    /// propagated twice more before anyone chased the producer.
    ///
    /// ## ★ α.1 removed the lower bound this test used to stop at
    ///
    /// It previously swept 1e3 down to **1e-2** and said so explicitly: *"deliberately stops
    /// there. Below it the oracle is known-wrong, and pinning a broken result would freeze it
    /// as intended behaviour."* That was the correct posture while the caller's units decided
    /// whether the oracle worked. `TriMeshDistance::new` now normalises internally, so there
    /// is no "below" — the sweep runs down to **1e-6**, six decades past the old cliff, and
    /// `internal_normalisation_fixes_the_sign_below_the_area_floor` measures the same repair
    /// against analytic truth with the pre-α.1 oracle as its control.
    ///
    /// ⚠ **The precondition had to change with it, and the old one would now be actively
    /// misleading.** It checked the smallest triangle in the *caller's* units against the
    /// floor — which is exactly the quantity that no longer decides anything. Asserting it
    /// would fail at every radius the repair added, for the very reason the repair exists. The
    /// meaningful precondition is on the **internal** area, after the scale the oracle actually
    /// picked, so that is what this now checks.
    #[test]
    fn pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use() {
        for radius in [1000.0, 10.0, 1.0, 0.1, 0.05, 0.025, 0.01, 1e-3, 1e-4, 1e-6] {
            let mesh = uv_sphere(radius, 24, 48);

            // Precondition, checked rather than assumed: after internal normalisation this
            // fixture is clear of the area floor. Stated on the INTERNAL area — the caller's
            // is no longer the quantity parry sees, and at the small radii below it is far
            // under the floor while the oracle is nonetheless exact.
            let floor = f64::from(f32::EPSILON) / 2.0;
            let scale = crate::sdf::choose_scale(&mesh).expect("sphere is scalable");
            let smallest = smallest_area(&mesh);
            let internal = smallest * scale * scale;
            assert!(
                internal > floor,
                "r={radius}: after normalisation by {scale:.3e} the smallest triangle is \
                 {internal:.4e}, still at or under parry's area floor ({floor:.4e}) — \
                 `choose_scale` did not do its job"
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

    /// **`AREA_MARGIN_BINADES` is the one input to the scale rule that is not derived
    /// from anything — so this asserts the answer does not depend on it.**
    ///
    /// Every other quantity in `scale_for_margin` tracks something real: `area_floor()`
    /// is `f32::EPSILON / 2` because that is where parry stops computing a pseudo-normal,
    /// `coordinate_cap()` is `sqrt(f32::MAX)` because that is where f32 products overflow,
    /// and `min_area` / `extent` come from the mesh. The margin is a choice.
    ///
    /// The temptation was to *measure* it — find the smallest margin that fixes the FSU
    /// disc. That would have been the arc's original defect one level up: a constant
    /// validated on one geometry, which is precisely how `Sdf::grad`'s `1e-6` and parry's
    /// area floor became load-bearing in the first place. And it answers the wrong
    /// question, because the cost function is one-sided — too small silently fails to fix
    /// the sign, too large costs nothing until a cliff tens of binades away.
    ///
    /// So the claim is insensitivity, not fitness: sweep the margin across a wide range
    /// and require the signed distance to be **bit-identical** throughout. If that holds,
    /// the value is provably not load-bearing, which is stronger than any fitted number.
    ///
    /// **The non-vacuity check is the load-bearing half.** If every margin in the sweep
    /// happened to round to the same `2^k`, bit-identity would be trivially true and this
    /// test would assert nothing. The sweep must actually move the scale.
    ///
    /// ## ★ α.2: non-vacuity is a claim about the SWEEP, not about every row
    ///
    /// It used to require *each* fixture's scale to move, which was right while the
    /// coordinate cap sat six orders too high and nothing was ever fully clamped. With the
    /// cap corrected the slivered row pins at one scale for **every** margin — and that is
    /// the rule behaving correctly, not a hole in the test: once the clamp binds at every
    /// margin, the margin genuinely cannot move the answer.
    ///
    /// Demanding movement there would force the fixture to be weakened until it stopped
    /// clamping, which would delete the only row that covers the clamped path at all. So
    /// each row now discharges non-vacuity one of two ways — **its scale moved**, or **it
    /// is provably pinned by the clamp** — and the sweep as a whole must contain at least
    /// one of each. A row that is pinned for neither reason still fails.
    #[test]
    fn the_chosen_scale_is_insensitive_to_the_margin_across_decades() {
        // Probes spanning inside, surface-adjacent and well outside, at a fixed relative
        // position so the geometry is identical at every radius.
        let dirs: Vec<f64> = (0..=12).map(|i| 0.25 + f64::from(i) * 0.125).collect();

        // ⚠ The last entry is CLAMPED and the others are not, which is the only reason
        //   this sweep covers the interaction at all. None of the plain spheres come near
        //   the cap, so without a slivered fixture the margin sweep would only ever
        //   exercise the unclamped path, and "the margin does not matter" would be
        //   unproven exactly where the rule stops honouring it.
        let (mut rows_that_moved, mut rows_pinned_by_the_clamp) = (0usize, 0usize);
        for (radius, sliver) in [
            (1e-3_f64, false),
            (1e-2, false),
            (1.0, false),
            (100.0, false),
            (SLIVERED_FIXTURE_RADIUS, true),
        ] {
            let mesh = if sliver {
                slivered_sphere(radius)
            } else {
                uv_sphere(radius, 24, 48)
            };
            let mut scales: Vec<u64> = Vec::new();
            let mut reference: Option<Vec<u64>> = None;

            for margin in [0_i32, 8, 16, 24, 32, 40, 48, 56, 64] {
                let s = crate::sdf::scale_for_margin(&mesh, margin).expect("sphere is scalable");
                scales.push(s.to_bits());
                let sdf = build_sdf_at(&mesh, s);

                let got: Vec<u64> = dirs
                    .iter()
                    .map(|f| sdf.distance(Point3::new(f * radius, 0.0, 0.0)).to_bits())
                    .collect();
                match &reference {
                    None => reference = Some(got),
                    Some(want) => assert_eq!(
                        &got,
                        want,
                        "r={radius}: margin {margin} binades gives a different signed distance \
                         than the first margin in the sweep (scale 2^{}). Then \
                         AREA_MARGIN_BINADES is load-bearing and picking it needs a \
                         justification this crate does not have.",
                        s.log2(),
                    ),
                }
            }

            scales.sort_unstable();
            scales.dedup();
            if scales.len() > 1 {
                rows_that_moved += 1;
            } else {
                // The one other way a row may be pinned: the clamp binds at every margin in
                // the sweep, including the smallest. Checked at margin 0 — if the rule is
                // capped even when it asks for nothing, it is capped throughout, since
                // `wanted` only grows with the margin.
                let pinned = crate::sdf::scale_rule(&mesh, 0).expect("sphere is scalable");
                assert!(
                    pinned.clamped(),
                    "r={radius}: every margin chose the same scale AND the coordinate cap \
                     is not binding, so the bit-identity above is vacuous — the margin was \
                     never actually exercised on this fixture"
                );
                rows_pinned_by_the_clamp += 1;
            }
        }

        // The sweep must cover both regimes. Without a moving row the insensitivity claim
        // is untested; without a pinned row the clamped path — where the rule stops
        // honouring the margin at all — is uncovered, which is the hole the slivered
        // fixture was added to close.
        assert!(
            rows_that_moved > 0,
            "no fixture in the sweep changed its scale as the margin varied, so every \
             bit-identity assertion above is vacuous"
        );
        assert!(
            rows_pinned_by_the_clamp > 0,
            "no fixture in the sweep was pinned by the coordinate cap, so the clamped path \
             is uncovered — the margin's irrelevance is unproven exactly where the rule \
             stops honouring it"
        );
    }

    /// **Where does parry actually stop producing a usable normal? Ask it, then require
    /// [`coordinate_cap`] to sit below the answer.**
    ///
    /// The cap's derivation is an argument about which f32 intermediate overflows first,
    /// and α.1 shipped that argument wrong — it protected the cross product (coordinate²)
    /// while the deciding operation was the cross's `norm_squared` (coordinate⁴). An
    /// argument that has already been wrong once does not get to be the only thing standing
    /// between the oracle and a mesh with every pseudo-normal zeroed.
    ///
    /// So this measures. It walks a triangle up the powers of two, asks parry for its
    /// normal at each, and finds the first scale where the answer stops being a usable
    /// direction — including the failure mode that is **not** `None`: `Unit::try_new`
    /// divides by `sqrt(inf)` and hands back `Some([0, 0, 0])`, which every caller in parry
    /// treats as a real normal.
    ///
    /// ⚠ **The non-vacuity check is the load-bearing half.** If the sweep never reached a
    /// breaking scale, "the cap is below the break" would be trivially true and this would
    /// assert nothing at all.
    #[test]
    fn the_coordinate_cap_sits_below_where_parry_overflows() {
        use parry3d::math::Point as ParryPoint;
        use parry3d::shape::Triangle;

        // A right triangle with legs of the swept coordinate magnitude — the simplest shape
        // whose cross product is exactly the product of its two legs, so the fourth power
        // enters through `norm_squared` and nothing else.
        let normal_is_usable = |coord: f32| {
            let tri = Triangle::new(
                ParryPoint::new(0.0, 0.0, 0.0),
                ParryPoint::new(coord, 0.0, 0.0),
                ParryPoint::new(0.0, coord, 0.0),
            );
            // `None` is the honest failure. `Some(zero)` is the dangerous one: parry's
            // inside test is `dpt.dot(&pn) <= 0.0`, which a zero vector satisfies for every
            // query point, so the surface reports "inside" at any distance.
            tri.normal()
                .is_some_and(|n| n.iter().any(|c| *c != 0.0) && n.iter().all(|c| c.is_finite()))
        };

        let mut first_broken: Option<i32> = None;
        for k in 0..=127 {
            if !normal_is_usable(2.0_f32.powi(k)) {
                first_broken = Some(k);
                break;
            }
        }

        let first_broken = first_broken.expect(
            "parry produced a usable normal at every coordinate up to 2^127 — then either \
             this probe is not reaching the overflow or parry's arithmetic changed, and \
             the assert below would be vacuous either way",
        );

        let break_coord = 2.0_f64.powi(first_broken);
        let cap = coordinate_cap();
        println!(
            "\nparry's normal degenerates at coordinate 2^{first_broken} ({break_coord:.3e}); \
             coordinate_cap() = {cap:.3e} = 2^{:.1}, {:.1} binades below",
            cap.log2(),
            break_coord.log2() - cap.log2(),
        );
        assert!(
            cap < break_coord,
            "coordinate_cap() is {cap:.3e}, at or above the {break_coord:.3e} where parry \
             stops returning a usable normal. A mesh clamped to that cap gets an oracle \
             whose pseudo-normals are all zero — which reports INSIDE at any distance. This \
             is exactly the defect α.2 found on a live mesh-lattice mesh at scale 2^48."
        );
    }

    /// **The consequence of the cap, measured end to end rather than on a bare triangle.**
    ///
    /// The sibling above locates parry's breaking point on a single triangle. This asks the
    /// question a consumer cares about: build the *whole* oracle at rising scales and watch
    /// [`TriMeshDistance::health`] — does the census stay clean right up to the cap, and
    /// does it go catastrophic past it?
    ///
    /// The "past it" arm is what makes the cap's value meaningful rather than merely
    /// asserted, and it is the regression gate for α.1's defect: at 2⁴⁸ this fixture had
    /// **every** vertex and **every** edge zeroed while `faces_skipped` read 0.
    #[test]
    fn the_census_stays_clean_up_to_the_cap_and_breaks_past_it() {
        let mesh = uv_sphere(1.0, 24, 48);
        let extent = 1.0_f64;
        let max_k = (coordinate_cap() / extent).log2().floor();

        // At and below the cap: nothing zeroed, and the areas parry sees stay finite.
        for k in [0_i32, 4, 8, 12, 16, 20] {
            if f64::from(k) > max_k {
                continue;
            }
            let h = TriMeshDistance::with_scale(mesh.clone(), 2.0_f64.powi(k))
                .expect("sphere is non-empty")
                .health();
            assert!(
                h.is_fully_signable(),
                "2^{k} is within the cap (2^{max_k}) yet the census found zeroed features: {h}"
            );
            assert!(
                h.median_area_internal.is_finite(),
                "2^{k}: parry's own areas overflowed to infinity inside the cap — {h}"
            );
        }

        // Past it: the catastrophe, reproduced. Deliberately far past, because the point is
        // not to locate the edge (the sibling test does that) but to show this fixture can
        // reach the state α.1 shipped — otherwise the arm above proves nothing.
        let over = 2.0_f64.powi(48);
        assert!(
            over > coordinate_cap() / extent,
            "the control scale must be OUTSIDE the cap or it is not a control"
        );
        let broken = TriMeshDistance::with_scale(mesh.clone(), over)
            .expect("sphere is non-empty")
            .health();
        println!("\npast the cap, 2^48: {broken}");
        assert_eq!(
            broken.zero_pseudo_normal_vertices, broken.referenced_vertices,
            "past the cap every vertex should be zeroed (parry divides by sqrt(inf) and \
             stores Some([0,0,0])), but only {} of {} were — the mechanism this cap guards \
             against is not the one being reproduced: {broken}",
            broken.zero_pseudo_normal_vertices, broken.referenced_vertices,
        );
        assert_eq!(
            broken.faces_skipped, 0,
            "★ the whole point: past the cap parry skips NOTHING, so an instrument that \
             only counted skipped faces would call this mesh healthy while every one of its \
             pseudo-normals is zero: {broken}"
        );
    }

    /// Coordinate extent of the slivered fixture, from the mesh that broke α.1 — output of
    /// **`mesh-offset`'s** marching cubes (which has no degenerate filter), consumed by
    /// `mesh-lattice`'s gates, which depend on that crate.
    ///
    /// ⚠ Earlier prose here credited `mesh-lattice`'s own `extract_isosurface`. That is a
    /// different function and it *cannot* produce this shape: it drops degenerate triangles
    /// at `cross.norm_squared() < f64::EPSILON`, forty-odd orders above this sliver. The
    /// misattribution sent α.2 looking in the wrong crate; the number itself was always a
    /// real measurement, printed by α.1's own `UnscalableMesh` error.
    ///
    /// Load-bearing: with the sliver's ~1e-30 area it puts `wanted` far above the cap, so
    /// the clamp BINDS — which is what the gates below need in order to observe anything.
    const SLIVERED_FIXTURE_RADIUS: f64 = 48.8;

    /// A sphere of `radius` with one near-degenerate triangle appended — sliver area taken
    /// from the mesh that actually broke α.1 (see [`SLIVERED_FIXTURE_RADIUS`] for which
    /// crate produced it), and `radius` chosen by the caller to set the coordinate extent
    /// that decides the clamp.
    ///
    /// Two gates share it: `a_sliver_clamps_the_scale_instead_of_failing_the_mesh` (which
    /// documents where the shape came from) and
    /// `an_unreferenced_vertex_does_not_move_the_chosen_scale`, which needs a mesh whose
    /// clamp is *binding* or it cannot see the defect it exists to catch.
    ///
    /// The radius is [`SLIVERED_FIXTURE_RADIUS`] at every call site; it is a parameter
    /// only so the value is stated once, at the constant, rather than at each caller.
    #[allow(clippy::cast_possible_truncation)]
    fn slivered_sphere(radius: f64) -> IndexedMesh {
        let mut mesh = uv_sphere(radius, 24, 48);
        let base = mesh.vertices.len() as u32;
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 3.074e-30, 0.0));
        mesh.faces.push([base, base + 1, base + 2]);
        mesh
    }

    /// **An unreferenced vertex must not move the scale — the contract `cf-cap-planes`
    /// names in its own docs.**
    ///
    /// `dome_wall_only_mesh` strips cap faces and deliberately leaves their vertices
    /// behind, on the stated grounds that *"`TriMeshDistance::new` walks faces, not
    /// vertices"*, and it feeds this constructor from both `cf-device-geometry` and
    /// `cf-cast-cli`. α.1's first cut computed the coordinate extent over `mesh.vertices`
    /// and broke that silently: a stranded vertex is not part of the surface parry meshes,
    /// but it would shrink the scale cap as though it were.
    ///
    /// ⚠ **The fixture has to be one whose clamp is BINDING, and the obvious choice is
    /// blind.** `extent` reaches the answer only through `max_k`, and `max_k` only changes
    /// anything when it is below `wanted`. On a plain `uv_sphere(1.0)` the rule wants 2¹³
    /// against a cap of 2⁵⁴, so a stranded vertex moves the cap to 2⁴⁷ and the chosen
    /// scale does not budge — the test would pass with the defect fully reintroduced. It
    /// did, when this gate was first written that way.
    ///
    /// The slivered sphere clamps at 2¹⁶, so a vertex at 99x its extent drags it to 2⁹
    /// and the difference is visible. Verified by reintroducing the defect and watching
    /// this fail.
    ///
    /// ⚠ Those two exponents were 2⁴⁸ and 2⁴¹ until α.2 corrected [`coordinate_cap`]. The
    /// gate discriminates either way — what matters is that the clamp binds, not where —
    /// but the numbers are restated rather than left to rot, because a reader checking them
    /// against the code is exactly the reader this test is written for.
    #[test]
    fn an_unreferenced_vertex_does_not_move_the_chosen_scale() {
        let clean = slivered_sphere(SLIVERED_FIXTURE_RADIUS);
        let mut stranded = clean.clone();
        let far = SLIVERED_FIXTURE_RADIUS * 99.0;
        stranded.vertices.push(Point3::new(far, far, far));

        let a = crate::sdf::choose_scale(&clean).expect("sphere is scalable");
        let b = crate::sdf::choose_scale(&stranded).expect("sphere is scalable");
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "a vertex no face references changed the internal scale ({a} -> {b}). Then the \
             scale rule reads geometry that is not part of the surface, and \
             `cf-cap-planes::dome_wall_only_mesh` — which leaves stranded vertices on \
             purpose — silently gets a worse scale than the same surface compacted."
        );
    }

    /// **A sliver must clamp the scale, not kill the mesh — the regression gate for the
    /// six `mesh-lattice` failures α.1's downstream sweep turned up.**
    ///
    /// The first version of the rule errored when it could not reach its full area margin
    /// without passing the f32 coordinate cap, on the argument that a surface which cannot
    /// be signed should not get a signed oracle. One run refuted that: a sliver of area
    /// **1.537e-30** across a 48.8-unit mesh — what an isosurface produces whenever it
    /// clips a cell corner — took six `mesh-lattice` gates red. The mesh itself is
    /// `mesh-offset`'s marching-cubes output, which `mesh-lattice` consumes.
    ///
    /// The mesh was never unsignable *as a whole*. This reproduces its shape and measures
    /// what clamping actually buys: the rule **succeeds**, and the damage is **confined to
    /// the sliver's own three features** while every other vertex signs correctly.
    ///
    /// ⚠ The `wanted > achieved` assert is the load-bearing half. Without it the fixture
    /// might not be clamping at all, and "the rule succeeded" would say nothing about the
    /// path this test exists to cover.
    ///
    /// ## ★★ α.2 replaced this test's other half, because that half is what hid a bug
    ///
    /// It used to close by computing `min_area * scale * scale` **in f64, from the caller's
    /// mesh**, and asserting the result cleared the floor — concluding the clamp "bought
    /// most of what mattered". Both the arithmetic and the conclusion were wrong, and they
    /// were wrong in the way this whole arc is about: *it never looked at the artifact.*
    ///
    /// At the cap α.1 shipped, parry's f32 held `inf` where that f64 model said `1.2e-1`,
    /// because `Unit::try_new` squares a cross product and the coordinates had gone past
    /// `f32::MAX^(1/4)`. Every pseudo-normal in the mesh was zero. The test passed
    /// throughout. See [`coordinate_cap`].
    ///
    /// With the cap corrected the sliver does **not** clear the floor — no scale can lift
    /// it that far without overflowing — and that is the honest answer. What matters is not
    /// the sliver's own margin but how far the damage spreads, so that is what is asserted
    /// now, read off [`TriMeshDistance::health`] rather than modelled.
    #[test]
    fn a_sliver_clamps_the_scale_instead_of_failing_the_mesh() {
        let mesh = slivered_sphere(SLIVERED_FIXTURE_RADIUS);
        let min_area = smallest_area(&mesh);
        assert!(
            min_area < 1e-29,
            "fixture's sliver ({min_area:.3e}) is not small enough to force a clamp"
        );

        let scale = crate::sdf::choose_scale(&mesh)
            .expect("a sliver must clamp the scale, not make the mesh unscalable");

        // It really did clamp: the unclamped rule wanted more than the cap allows.
        let floor = f64::from(f32::EPSILON) / 2.0;
        let target = floor * 2.0_f64.powi(40);
        let wanted = (target / min_area).sqrt().log2().ceil();
        let achieved = scale.log2();
        assert!(
            wanted > achieved,
            "fixture did not exercise the clamp — the rule wanted 2^{wanted} and got \
             2^{achieved}, so nothing was capped"
        );

        // What the clamp actually bought, read off the artifact parry built rather than
        // modelled in f64 from the caller's mesh.
        let health = TriMeshDistance::new(mesh.clone())
            .expect("a sliver must clamp the scale, not make the mesh unscalable")
            .health();
        println!(
            "\nsliver {min_area:.3e}, extent {SLIVERED_FIXTURE_RADIUS} -> wanted 2^{wanted}, \
             clamped 2^{achieved}\n  {health}"
        );

        // The floor is genuinely out of reach for this sliver, and saying so is the point:
        // the previous version of this test asserted the opposite and was believed.
        assert!(
            health.min_area_internal <= floor,
            "the sliver now clears the floor ({:.3e} > {floor:.3e}). That may be good news, \
             but this test's whole subject is what happens when it CANNOT be lifted — \
             re-derive the fixture before deleting the case",
            health.min_area_internal,
        );

        // ★ The real claim: the damage is confined to the sliver's own three vertices and
        //   three edges. They belong to a triangle appended in isolation, so nothing else
        //   is incident to them and no scale can rescue them — whereas all 1106 vertices of
        //   the sphere proper sign correctly.
        assert_eq!(
            (
                health.zero_pseudo_normal_vertices,
                health.zero_pseudo_normal_edges
            ),
            (3, 3),
            "clamping left {} vertices and {} edges unsignable, not the 3 + 3 belonging to \
             the isolated sliver. The damage has spread beyond the degenerate triangle, \
             which is the coordinate-cap overflow this fixture also guards: {health}",
            health.zero_pseudo_normal_vertices,
            health.zero_pseudo_normal_edges,
        );
        assert!(
            health.referenced_vertices > 1000 && health.median_area_internal.is_finite(),
            "the sphere around the sliver must be intact and its areas finite, or the \
             '3 + 3' above is confinement to a wreck rather than to a sliver: {health}"
        );
    }

    /// **The headline claim, in licence-free form: the oracle no longer depends on the
    /// caller's units.**
    ///
    /// This is the FSU disc's bug reproduced on a fixture. A sphere of radius 1e-3 has
    /// pole-fan triangles at ~1.1e-9, well under parry's 5.96e-8 area floor, so its
    /// pseudo-normals are zeroed and the oracle reports "inside" at any distance — the
    /// same mechanism that put 30 % of the FSU disc's triangles under the floor once
    /// `prepare_disc_at` rescaled it to metres.
    ///
    /// Both arms are asserted, and the broken one is what gives the fixed one meaning:
    /// `with_scale(mesh, 1.0)` (the pre-α.1 oracle) must still get the sign **wrong**,
    /// and `new` must get it **right**. A test that only checked the second would pass
    /// just as well on a fixture that never had the bug.
    #[test]
    fn internal_normalisation_fixes_the_sign_below_the_area_floor() {
        // Sample a lattice reaching to 1.6 r per axis — corners land at 2.77 r, where the
        // closest feature is an edge or a vertex and a zeroed pseudo-normal shows up.
        let n = 10;
        for radius in [1e-3_f64, 1e-4] {
            let mesh = uv_sphere(radius, 24, 48);
            let floor = f64::from(f32::EPSILON) / 2.0;
            let min_area = smallest_area(&mesh);
            assert!(
                min_area < floor,
                "r={radius}: fixture's smallest triangle ({min_area:.3e}) already clears the \
                 floor ({floor:.3e}) — this radius never had the bug and proves nothing"
            );

            let fixed = build_sdf(mesh.clone());
            let broken = build_sdf_at(&mesh, 1.0);
            let (mut fixed_wrong, mut broken_wrong, mut checked) = (0usize, 0usize, 0usize);

            for i in 0..=n {
                for j in 0..=n {
                    for k in 0..=n {
                        let q =
                            |idx: i32| 1.6 * radius * (f64::from(idx) / f64::from(n) - 0.5) * 2.0;
                        let p = Point3::new(q(i), q(j), q(k));
                        let truth = p.coords.norm() - radius;
                        // Skip the tessellation's own uncertainty band, where the
                        // polyhedron and the ideal sphere genuinely differ.
                        if truth.abs() < 0.005 * radius {
                            continue;
                        }
                        checked += 1;
                        if (fixed.distance(p) < 0.0) != (truth < 0.0) {
                            fixed_wrong += 1;
                        }
                        if (broken.distance(p) < 0.0) != (truth < 0.0) {
                            broken_wrong += 1;
                        }
                    }
                }
            }

            println!(
                "r={radius:.0e}  min_area {min_area:.3e} / floor {floor:.3e} = \
                 {:.3}  ->  pre-α.1 wrong at {broken_wrong}/{checked}, normalised wrong at \
                 {fixed_wrong}/{checked}",
                min_area / floor,
            );
            assert!(
                checked > 500,
                "r={radius}: only {checked} probes cleared the band"
            );
            assert!(
                broken_wrong > 0,
                "r={radius}: the pre-α.1 oracle got every sign RIGHT, so this fixture does \
                 not reproduce the area-floor bug and the fix below is unearned"
            );
            assert_eq!(
                fixed_wrong, 0,
                "r={radius}: the internally normalised oracle still gets {fixed_wrong} of \
                 {checked} signs wrong"
            );
        }
    }

    /// Smallest triangle area — the quantity parry's area floor applies to.
    fn smallest_area(m: &IndexedMesh) -> f64 {
        m.faces
            .iter()
            .map(|f| {
                let (a, b, c) = (
                    m.vertices[f[0] as usize],
                    m.vertices[f[1] as usize],
                    m.vertices[f[2] as usize],
                );
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// Compose the standard pair at an explicit internal scale.
    fn build_sdf_at(mesh: &IndexedMesh, scale: f64) -> Signed<TriMeshDistance, PseudoNormalSign> {
        let distance =
            TriMeshDistance::with_scale(mesh.clone(), scale).expect("fixture is non-empty");
        let sign = PseudoNormalSign::from_distance(&distance);
        Signed { distance, sign }
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

        // ── Powers of two. Full SIGNED bit-identity at every scale. ──
        //
        // ★ α.1 collapsed the split this loop used to carry. It asserted magnitude
        //   everywhere but sign only where `min_area * s * s > floor` — because below the
        //   floor the sign genuinely was scale-dependent, and that was the finding. Now
        //   `TriMeshDistance::new` normalises internally, so the caller-frame area decides
        //   nothing and the weaker branch would be asserting less than is true.
        //
        //   The split's non-vacuity check went with it, and its removal is the load-bearing
        //   part: it required the sweep to STRADDLE the floor, and `min_area * s * s` still
        //   straddles it, so that assert would keep passing while guarding a distinction the
        //   oracle no longer makes. A guard that cannot fail for the right reason is worse
        //   than no guard, because it reads as coverage.
        let mut total_checked = 0usize;
        for k in [-20_i32, -10, -3, -1, 1, 3, 10, 20] {
            let s = 2.0_f64.powi(k);
            // The scales at the low end are far under the floor in caller units — which is
            // exactly why full signed identity there is the interesting claim now.
            let caller_frame_area = min_area * s * s;
            let sdf_s = build_sdf(scaled(&base, s));
            let mut checked = 0usize;
            for &p in &probes {
                let want = sdf1.distance(p);
                let got = sdf_s.distance(Point3::new(p.x * s, p.y * s, p.z * s)) / s;
                assert_eq!(
                    got.to_bits(),
                    want.to_bits(),
                    "2^{k}: a power-of-two rescale changed the SIGNED distance at {p:?} — got \
                     {got:.17e}, want {want:.17e}. The fixture's smallest triangle is \
                     {caller_frame_area:.3e} in caller units against a {floor:.3e} floor, but \
                     internal normalisation is supposed to make that irrelevant."
                );
                checked += 1;
            }
            assert!(checked > 100, "2^{k}: too few probes to mean anything");
            total_checked += checked;
        }
        assert!(
            total_checked > 1000,
            "only {total_checked} probe comparisons ran across the power-of-two sweep"
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
