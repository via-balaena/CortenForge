//! `pinned_floor_shell` — anisotropic-offset composite primitive for
//! layered silicone device design.
//!
//! Built around the user's geometric model from
//! `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md` §1: a cavity
//! / layer shell whose dome + side walls grow and shrink with `offset_m`
//! while every floor stays pinned to its cap plane. The construction is
//! a set-theoretic offset over a dome-wall-only rind that thickens
//! around the open surface but never touches the cap polygon, leaving
//! the body's cap polygon — and the floor it sits on — invariant under
//! `offset_m`.
//!
//! # Construction (candidate A from
//! `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_REDESIGN_SPEC.md`)
//!
//! ```text
//! rind  = Solid::from_sdf(|open_sdf(p)| − T, bounds)
//! cavity (offset_m < 0): body.subtract(rind)
//! outer  (offset_m > 0): body.union(rind)
//! offset_m == 0       : body
//! then intersect with body-interior half-space of every cap plane.
//! ```
//!
//! Because the cap polygon is stripped from the open mesh, `open_sdf`
//! never spawns a rind atom on the cap polygon — so the body's cap
//! polygon survives the rind subtraction / union untouched, and every
//! shell ends up with a flat floor at that cap polygon's plane.
//!
//! # Empty caps — fast path
//!
//! When the caller passes no cap planes, the primitive degenerates to a
//! plain isotropic offset of the closed body (`open_sdf` is unused).
//! This keeps pre-pinned-floor callers byte-identical: a no-caps device
//! ships the same geometry it did before this primitive existed.

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

use crate::{Sdf, Solid};

/// Anisotropic-offset shell pinned to one or more cap planes.
///
/// Produces a [`Solid`] AST that evaluates to the user's pinned-floor
/// geometric model: dome + side walls offset by `offset_m`, with every
/// floor remaining at its cap polygon's plane regardless of the offset.
///
/// `closed_sdf` must be the SDF of the full closed body (cap polygons
/// included) — it supplies the sign source. `open_sdf` is the SDF of
/// the same body with cap polygons stripped (dome + side walls only) —
/// only its `.eval(p).abs()` is consumed via a private unsigned-rind
/// adapter, so the open SDF's own sign heuristic (notoriously fragile
/// on non-manifold open meshes) is irrelevant.
///
/// `cap_planes` is a list of `(centroid, outward_normal)` tuples in the
/// same physics frame as the SDFs. The primitive intersects each shell
/// with the body-interior half-space of every plane —
/// `(p − centroid) · normal ≤ 0`. The schema-agnostic tuple shape lets
/// upstream crates (cf-cap-planes' `CapPlane`) keep their own runtime
/// type without dragging it into cf-design's layer.
///
/// `bounds` is the AABB hint required by [`Solid::from_sdf`] for both
/// the closed-body and rind nodes; pick one that encloses every point
/// where the resulting shell will be queried.
///
/// `offset_m` follows the [`Solid::offset`] sign convention: negative
/// shrinks (cavity inset), positive grows (layer outer thickness), zero
/// returns the body clipped by the caps.
///
/// # Floor pinning is structural
///
/// `pinned_floor_shell_cavity_iso_zero_on_cap_plane_at_body_center` in
/// the test module is the load-bearing structural-validation gate from
/// the redesign spec §1 Q8 — it asserts that the cavity SDF evaluates
/// to ≈ 0 at the body center on the cap plane on a closed half-box
/// fixture, the construction-level claim the previous scope-C
/// `sign × |open|` primitive failed at visual gate #1. If that test
/// fails on any change to this primitive, the construction has lost the
/// floor-pinning property and downstream consumers must NOT propagate
/// the change.
///
/// # Empty caps
///
/// Returns `Solid::from_sdf(closed_sdf, bounds).offset(offset_m)` —
/// byte-identical to a pre-pinned-floor uniform offset. `open_sdf` is
/// consumed but never queried; cheap if it's an `Arc<dyn Sdf>` clone
/// the caller already had on hand.
///
/// # Panics
///
/// Panics if any cap-plane normal is not unit length and finite, or if
/// any centroid coordinate is non-finite — delegated to
/// [`Solid::plane`].
pub fn pinned_floor_shell<C, O>(
    closed_sdf: C,
    open_sdf: O,
    bounds: Aabb,
    cap_planes: &[(Point3<f64>, Vector3<f64>)],
    offset_m: f64,
) -> Solid
where
    C: Sdf + 'static,
    O: Sdf + 'static,
{
    if cap_planes.is_empty() {
        // No-caps fast path: standard isotropic offset, behaviorally
        // identical to pre-pinned-floor callers. `open_sdf` is consumed
        // here but never queried (it gets dropped immediately).
        let _ = open_sdf;
        return Solid::from_sdf(closed_sdf, bounds).offset(offset_m);
    }
    let body = Solid::from_sdf(closed_sdf, bounds);
    let mut shell = if offset_m < 0.0 {
        let rind = Solid::from_sdf(UnsignedRindSdf::new(open_sdf, -offset_m), bounds);
        body.subtract(rind)
    } else if offset_m > 0.0 {
        let rind = Solid::from_sdf(UnsignedRindSdf::new(open_sdf, offset_m), bounds);
        body.union(rind)
    } else {
        let _ = open_sdf;
        body
    };
    for (centroid, normal) in cap_planes {
        // `Solid::plane(normal, offset)` SDF is `p·normal − offset`,
        // negative on the side opposite the normal. Cap normals point
        // OUTWARD (away from body), so the body-interior half-space is
        // `p·normal ≤ centroid·normal` — exactly where this plane SDF
        // is non-positive. `shell.intersect(plane)` keeps that side.
        let plane_offset = centroid.coords.dot(normal);
        shell = shell.intersect(Solid::plane(*normal, plane_offset));
    }
    shell
}

/// Private adapter: unsigned distance to the dome-wall surface minus a
/// half-thickness.
///
/// `eval(p) = open.eval(p).abs() − half_thickness`. Negative inside the
/// `half_thickness`-thick band around the open surface, positive
/// outside, zero on the band's boundary. Used as the rind SDF in
/// [`pinned_floor_shell`]'s candidate-A construction: `body − rind`
/// shrinks the body anisotropically (rind only thickens around dome +
/// side walls, not cap polygons), `body ∪ rind` grows it the same way.
///
/// Consuming only `open.eval(p).abs()` is the load-bearing design
/// choice — the open SDF's sign on a non-manifold cap-stripped mesh is
/// unreliable (`mesh_sdf::SignedDistanceField`'s face-normal heuristic
/// fails ~12% of far-field probes per the `insertion_sim` diagnostic),
/// but its unsigned magnitude is robust.
///
/// Gradient is a central finite difference; the eval is piecewise
/// smooth with kinks on `open.eval == 0`, so no analytic form is
/// available in closed form. Matches `cf_design::Sdf for
/// SignedDistanceField`'s precedent.
struct UnsignedRindSdf<O: Sdf> {
    open: O,
    half_thickness: f64,
}

/// Central-finite-difference step (meters) for [`UnsignedRindSdf`]'s
/// gradient. Matches `Sdf for SignedDistanceField`'s 1e-6 step — small
/// enough that the central-difference truncation error on a piecewise-
/// smooth SDF stays well below surface-locating tolerances, large
/// enough that f64 cancellation does not eat the signal.
const UNSIGNED_RIND_GRAD_EPS: f64 = 1e-6;

impl<O: Sdf> UnsignedRindSdf<O> {
    const fn new(open: O, half_thickness: f64) -> Self {
        Self {
            open,
            half_thickness,
        }
    }
}

impl<O: Sdf> Sdf for UnsignedRindSdf<O> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.open.eval(p).abs() - self.half_thickness
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let eps = UNSIGNED_RIND_GRAD_EPS;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.eval(Point3::new(p.x + eps, p.y, p.z))
                - self.eval(Point3::new(p.x - eps, p.y, p.z)))
                * inv_2eps,
            (self.eval(Point3::new(p.x, p.y + eps, p.z))
                - self.eval(Point3::new(p.x, p.y - eps, p.z)))
                * inv_2eps,
            (self.eval(Point3::new(p.x, p.y, p.z + eps))
                - self.eval(Point3::new(p.x, p.y, p.z - eps)))
                * inv_2eps,
        )
    }
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` denied at crate level; allow inside tests.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::sync::Arc;

    use mesh_sdf::SignedDistanceField;
    use mesh_types::IndexedMesh;

    use super::*;

    // ----- Fixtures ----------------------------------------------------

    /// Closed half-box centered laterally on the origin with its top
    /// face (the cap polygon) on `z = 0` and the rest extending to
    /// `z = -2`. 12 triangles (6 quad faces, 2 triangles each); outward
    /// normals follow CCW convention.
    fn closed_half_box() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        let mut push = |x: f64, y: f64, z: f64| {
            mesh.vertices.push(Point3::new(x, y, z));
        };
        // Bottom z=-2.
        push(-1.0, -1.0, -2.0); // 0
        push(1.0, -1.0, -2.0); // 1
        push(1.0, 1.0, -2.0); // 2
        push(-1.0, 1.0, -2.0); // 3
        // Top z=0 (cap polygon).
        push(-1.0, -1.0, 0.0); // 4
        push(1.0, -1.0, 0.0); // 5
        push(1.0, 1.0, 0.0); // 6
        push(-1.0, 1.0, 0.0); // 7
        mesh.faces.extend_from_slice(&[
            // Bottom (-z outward).
            [0, 2, 1],
            [0, 3, 2],
            // Top (+z outward) — the cap polygon.
            [4, 5, 6],
            [4, 6, 7],
            // -x side.
            [0, 4, 7],
            [0, 7, 3],
            // +x side.
            [1, 2, 6],
            [1, 6, 5],
            // -y side.
            [0, 1, 5],
            [0, 5, 4],
            // +y side.
            [2, 3, 7],
            [2, 7, 6],
        ]);
        mesh
    }

    /// [`closed_half_box`] with the two cap-polygon (top) triangles
    /// stripped. The cap-stripping rule (face-normal alignment with the
    /// cap-plane normal) lives in cf-cap-planes; here we inline the
    /// equivalent for the fixture so cf-design's tests do not pull in
    /// cf-cap-planes as a dev-dep.
    fn open_dome_only() -> IndexedMesh {
        let closed = closed_half_box();
        IndexedMesh {
            vertices: closed.vertices,
            // Drop the two `+z` top faces (indices 2 and 3 in the face
            // list above).
            faces: closed
                .faces
                .into_iter()
                .enumerate()
                .filter_map(|(i, f)| (i != 2 && i != 3).then_some(f))
                .collect(),
        }
    }

    fn fixture_bounds() -> Aabb {
        Aabb::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0))
    }

    fn cap_at_origin_plus_z() -> (Point3<f64>, Vector3<f64>) {
        (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0))
    }

    // ----- Tests -------------------------------------------------------

    #[test]
    fn pinned_floor_shell_empty_caps_byte_identical_to_offset() {
        // No caps → open_sdf unused; result must equal a plain
        // `Solid::from_sdf(closed).offset(T)` at every probe.
        let closed = closed_half_box();
        let open = open_dome_only();
        let closed_sdf = SignedDistanceField::new(closed).unwrap();
        let open_sdf = SignedDistanceField::new(open).unwrap();
        let bounds = fixture_bounds();

        let shell = pinned_floor_shell(closed_sdf.clone(), open_sdf, bounds, &[], -0.1);
        let reference = Solid::from_sdf(closed_sdf, bounds).offset(-0.1);

        let probes = [
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.5, 0.0, -1.0),
            Point3::new(0.0, 1.5, -1.0),
            Point3::new(-0.95, -0.95, -1.5),
            Point3::new(0.0, 0.0, -2.0),
            Point3::new(0.25, -0.5, -1.0),
            Point3::new(-0.5, 0.25, -1.0),
            Point3::new(0.0, 0.0, 0.0),
        ];
        for p in probes {
            let a = shell.evaluate(&p);
            let b = reference.evaluate(&p);
            assert!(
                (a - b).abs() < 1e-12,
                "no-caps fast path drifted at {p:?}: shell={a}, reference={b}",
            );
        }
    }

    #[test]
    fn pinned_floor_shell_cavity_iso_zero_on_cap_plane_at_body_center() {
        // STRUCTURAL VERIFICATION GATE (redesign spec §1 Q8). If this
        // fails, the construction has lost floor pinning — do not
        // propagate to consumers (A3/A4/A5); recon iteration 3 starts
        // instead. See `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_FALSIFICATION_BOOKMARK.md`
        // for what scope-C did wrong and why this test is load-bearing.
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();

        let cavity = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], -0.1);

        // BODY CENTER ON CAP PLANE — must sit on the cavity boundary.
        let sd_on_cap = cavity.evaluate(&Point3::new(0.0, 0.0, 0.0));
        assert!(
            sd_on_cap.abs() < 0.01,
            "expected ~0 at body center on cap plane (floor pinned), got {sd_on_cap}",
        );
        // ABOVE the cap plane — strictly outside the cavity (clipped).
        let sd_above = cavity.evaluate(&Point3::new(0.0, 0.0, 0.1));
        assert!(
            sd_above > 0.0,
            "above cap plane must be outside cavity, got {sd_above}",
        );
        // DEEP INSIDE the body, well away from any wall — strictly
        // inside the cavity.
        let sd_deep = cavity.evaluate(&Point3::new(0.0, 0.0, -1.0));
        assert!(
            sd_deep < 0.0,
            "deep interior must be inside cavity, got {sd_deep}",
        );
    }

    #[test]
    fn pinned_floor_shell_outer_iso_zero_on_cap_plane_at_body_center() {
        // Same structural gate for the +offset_m branch (layer outer).
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();

        let outer = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], 0.1);

        let sd_on_cap = outer.evaluate(&Point3::new(0.0, 0.0, 0.0));
        assert!(
            sd_on_cap.abs() < 0.01,
            "expected ~0 at body center on cap plane (floor pinned), got {sd_on_cap}",
        );
        let sd_above = outer.evaluate(&Point3::new(0.0, 0.0, 0.1));
        assert!(
            sd_above > 0.0,
            "above cap plane must be outside outer, got {sd_above}",
        );
        let sd_deep = outer.evaluate(&Point3::new(0.0, 0.0, -1.0));
        assert!(
            sd_deep < 0.0,
            "deep interior must be inside outer, got {sd_deep}",
        );
    }

    #[test]
    fn pinned_floor_shell_cavity_floor_outline_shrunk_by_t() {
        // Cavity floor = cap polygon shrunk inward by T. Probes sit
        // just below the cap plane (z = -0.01) so the cavity interior
        // is strictly < 0 there, not on the boundary.
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();
        let t = 0.2_f64;

        let cavity = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], -t);

        // Just inside the +x wall by T − ε = 0.15 m: still within the
        // rind, so subtracted out of the cavity → OUTSIDE.
        let near_wall = cavity.evaluate(&Point3::new(0.85, 0.0, -0.01));
        assert!(
            near_wall > 0.0,
            "point within T of dome wall must be outside cavity, got {near_wall}",
        );
        // Inside by T + ε = 0.25 m: clear of the rind → INSIDE cavity.
        let inside_floor = cavity.evaluate(&Point3::new(0.75, 0.0, -0.01));
        assert!(
            inside_floor < 0.0,
            "point beyond T from dome wall must be inside cavity, got {inside_floor}",
        );
    }

    #[test]
    fn pinned_floor_shell_outer_floor_outline_extended_by_t() {
        // Layer-outer floor = cap polygon extended outward by T.
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();
        let t = 0.2_f64;

        let outer = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], t);

        // Just outside the +x wall by 0.15 m (< T): inside the rind, so
        // unioned in → INSIDE the outer.
        let near_wall = outer.evaluate(&Point3::new(1.15, 0.0, -0.01));
        assert!(
            near_wall < 0.0,
            "point within T outside dome wall must be inside outer, got {near_wall}",
        );
        // Beyond T outward at 0.25 m: clear of the rind → OUTSIDE.
        let far = outer.evaluate(&Point3::new(1.25, 0.0, -0.01));
        assert!(
            far > 0.0,
            "point beyond T from dome wall must be outside outer, got {far}",
        );
    }

    #[test]
    fn pinned_floor_shell_multi_cap_clips_all_caps() {
        // Closed body extending z ∈ [-1, 1] with caps at BOTH ends. Cap
        // extensions (above +z cap, below −z cap) must be outside the
        // shell regardless of which one we probe.
        let mut closed = IndexedMesh::new();
        for &(x, y, z) in &[
            (-1.0_f64, -1.0_f64, -1.0_f64),
            (1.0, -1.0, -1.0),
            (1.0, 1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, 1.0),
            (-1.0, 1.0, 1.0),
        ] {
            closed.vertices.push(Point3::new(x, y, z));
        }
        closed.faces.extend_from_slice(&[
            [0, 2, 1],
            [0, 3, 2], // -z cap.
            [4, 5, 6],
            [4, 6, 7], // +z cap.
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
        ]);
        // Open mesh: drop the two cap-pair faces (indices 0,1,2,3).
        let open = IndexedMesh {
            vertices: closed.vertices.clone(),
            faces: closed
                .faces
                .iter()
                .enumerate()
                .filter_map(|(i, f)| (i >= 4).then_some(*f))
                .collect(),
        };
        let closed_sdf = SignedDistanceField::new(closed).unwrap();
        let open_sdf = SignedDistanceField::new(open).unwrap();
        let bounds = Aabb::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0));
        let caps = [
            (Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
            (Point3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, -1.0)),
        ];

        let cavity = pinned_floor_shell(closed_sdf, open_sdf, bounds, &caps, -0.1);

        // Above the +z cap and below the −z cap are clipped out by the
        // respective half-space intersections.
        let above = cavity.evaluate(&Point3::new(0.0, 0.0, 1.1));
        assert!(above > 0.0, "above +z cap must be outside, got {above}");
        let below = cavity.evaluate(&Point3::new(0.0, 0.0, -1.1));
        assert!(below > 0.0, "below −z cap must be outside, got {below}");
        // Body center sits strictly inside the cavity.
        let center = cavity.evaluate(&Point3::new(0.0, 0.0, 0.0));
        assert!(
            center < 0.0,
            "body center must be inside cavity, got {center}"
        );
    }

    #[test]
    fn pinned_floor_shell_offset_zero_returns_body_clipped_by_caps() {
        // T == 0 short-circuits past the rind ops; result is the body
        // intersected with the cap half-spaces.
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();

        let shell = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], 0.0);

        // Above the cap plane: clipped → outside.
        let above = shell.evaluate(&Point3::new(0.0, 0.0, 0.1));
        assert!(above > 0.0, "above cap plane must be outside, got {above}");
        // Deep inside body, well below the cap plane: inside the body
        // and inside the half-space → inside the shell.
        let deep = shell.evaluate(&Point3::new(0.0, 0.0, -1.0));
        assert!(deep < 0.0, "deep interior must be inside shell, got {deep}");
    }

    #[test]
    fn pinned_floor_shell_meshes_cleanly_via_solid_mesh() {
        // Spot-check that the Solid AST extracted by `pinned_floor_shell`
        // meshes — same downstream codepath cf-device-design / cf-cast-
        // cli go through to surface the cavity.
        let closed_sdf = SignedDistanceField::new(closed_half_box()).unwrap();
        let open_sdf = SignedDistanceField::new(open_dome_only()).unwrap();
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();

        let cavity = pinned_floor_shell(closed_sdf, open_sdf, bounds, &[cap], -0.1);
        let mesh = cavity.mesh(0.1);
        assert!(
            !mesh.geometry.faces.is_empty(),
            "pinned_floor_shell cavity must produce a non-empty mesh",
        );
    }

    #[test]
    fn unsigned_rind_sdf_eval_returns_unsigned_distance_minus_half_thickness() {
        // Bench `eval` against the analytical formula on a synthetic
        // SDF — exercises the adapter without depending on cf-design's
        // larger primitives.
        struct ConstSdf(f64);
        impl Sdf for ConstSdf {
            fn eval(&self, _p: Point3<f64>) -> f64 {
                self.0
            }
            fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
                Vector3::zeros()
            }
        }
        let adapter_pos = UnsignedRindSdf::new(ConstSdf(0.5), 0.2);
        assert!((adapter_pos.eval(Point3::origin()) - 0.3).abs() < 1e-12);
        // Sign of the open SDF must NOT affect the result (the .abs()
        // is the load-bearing design choice; mesh-sdf's sign heuristic
        // is unreliable on non-manifold open meshes, so the primitive
        // discards it by construction).
        let adapter_neg = UnsignedRindSdf::new(ConstSdf(-0.5), 0.2);
        assert!((adapter_neg.eval(Point3::origin()) - 0.3).abs() < 1e-12);
        // On the surface of the open body (open.eval == 0) the rind SDF
        // is just `-half_thickness` — strictly inside the rind.
        let adapter_zero = UnsignedRindSdf::new(ConstSdf(0.0), 0.2);
        assert!((adapter_zero.eval(Point3::origin()) + 0.2).abs() < 1e-12);
    }

    #[test]
    fn pinned_floor_shell_accepts_arc_dyn_sdf_for_repeated_calls() {
        // Consumer pattern (cf-device-design / cf-cast-cli): wrap each
        // SDF in `Arc<dyn Sdf>` once and pass `Arc::clone`s to multiple
        // `pinned_floor_shell` calls (plug + per-layer bodies). The
        // `Sdf for Arc<T>` blanket impl makes this work without each
        // consumer re-building its SDFs per call.
        let closed: Arc<dyn Sdf> = Arc::new(SignedDistanceField::new(closed_half_box()).unwrap());
        let open: Arc<dyn Sdf> = Arc::new(SignedDistanceField::new(open_dome_only()).unwrap());
        let bounds = fixture_bounds();
        let cap = cap_at_origin_plus_z();
        let plug = pinned_floor_shell(closed.clone(), open.clone(), bounds, &[cap], -0.05);
        let layer1 = pinned_floor_shell(closed.clone(), open.clone(), bounds, &[cap], 0.05);
        let layer2 = pinned_floor_shell(closed, open, bounds, &[cap], 0.10);
        // All three evaluate at body center on cap plane — pinned to
        // ~0 by construction.
        for shell in [&plug, &layer1, &layer2] {
            let sd = shell.evaluate(&Point3::new(0.0, 0.0, 0.0));
            assert!(sd.abs() < 0.01, "Arc-cloned consumer drifted: sd={sd}");
        }
    }
}
