//! Composite-primitive constructors that build on top of [`Solid`]'s
//! primitive constructors + boolean operations.
//!
//! Currently a single primitive — [`pinned_floor_shell`] — but the
//! module exists to keep composite-primitive functions separate from
//! `solid.rs`'s primitive constructors so future additions
//! (e.g. ribbed-shell, capped-cylinder, gusseted-flange) have a
//! natural home.

use nalgebra::{Point3, Vector3};

use crate::{Aabb, Sdf, Solid};

/// Build a pinned-floor closed shell: the SDF body offset by
/// `offset_m`, intersected with the body-interior half-space of every
/// cap plane.
///
/// Returns a [`Solid`] whose zero iso surface is a CLOSED manifold for
/// any body SDF that's continuous across the cap planes. Used by the
/// pinned-floor scope-C arc to build closed cavity + outer shells whose
/// floors are pinned to the cleaned scan's cap plane(s) — the same
/// geometric primitive cf-device-design's `extract_layer_surface`
/// composes for its cached-grid preview path, but at the
/// `cf_design::Solid` level so the FEM insertion sim
/// (`tools/cf-device-design/src/insertion_sim.rs`) and the cf-cast-cli
/// mold generation (`tools/cf-cast-cli/src/derive.rs`) can compose it
/// against their own SDF construction without going through a triangle
/// mesh.
///
/// # Cap-normal convention
///
/// `cap_planes` are `(centroid, outward_normal)` tuples in the same
/// coordinate frame as `sdf` and `bounds`. "Outward" = away from the
/// body interior — matches cf-scan-prep's `orient_cap_normal_outward`
/// heuristic and `cf_cap_planes::CapPlane.normal`. The body-interior
/// side of each cap plane satisfies `(p - centroid) · normal ≤ 0`; the
/// constructed `Solid` keeps that side (via [`Solid::intersect`]) and
/// clips the cap-extension side.
///
/// # Why the tuple form (not `&[CapPlane]`)
///
/// cf-design stays schema-agnostic: it has no dep on cf-cap-planes
/// (which depends on cf-design, NOT the other way around — adding the
/// reverse dep would couple cf-design to cf-scan-prep's `.prep.toml`
/// schema). Callers convert with `cap_planes.iter().map(|c|
/// c.as_tuple()).collect::<Vec<_>>()` at the call site.
///
/// # Multi-cap behavior
///
/// Folding `Solid::intersect` over N caps yields the SDF of the
/// intersection of all N body-interior half-spaces with the offset
/// body — every cap floor closes the same shell. Zero caps short-
/// circuits to `Solid::from_sdf(sdf).offset(offset_m)` (no cap
/// half-spaces in the intersection), matching the no-caps fast path
/// in `cf-device-design::sdf_layers::extract_layer_surface`.
///
/// # Sign convention
///
/// `Solid::plane(normal, offset)` SDF is `p·normal − offset`, with the
/// `normal` side positive (outside). We compute `offset = centroid ·
/// normal` so the plane passes through `centroid`; intersecting with
/// this plane is `max(body_sdf, plane_sdf)`, which keeps the
/// `plane_sdf ≤ 0` side (the body-interior side) since `max` is
/// dominated by `body_sdf` where `plane_sdf` is negative.
///
/// # Panics
///
/// Forwards [`Solid::offset`]'s panic on non-finite `offset_m` and
/// [`Solid::plane`]'s panics on degenerate cap normals or non-finite
/// dot products. Pinned-floor scope-C v1 consumers all source caps
/// from cf-cap-planes' `parse_cap_planes`, which normalizes normals
/// and rejects malformed records before they reach here.
#[must_use]
pub fn pinned_floor_shell<S: Sdf + 'static>(
    sdf: S,
    bounds: Aabb,
    cap_planes: &[(Point3<f64>, Vector3<f64>)],
    offset_m: f64,
) -> Solid {
    let mut shell = Solid::from_sdf(sdf, bounds).offset(offset_m);
    for (centroid, normal) in cap_planes {
        let d = centroid.coords.dot(normal);
        shell = shell.intersect(Solid::plane(*normal, d));
    }
    shell
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use approx::assert_relative_eq;

    /// Trivial sphere SDF for unit-cube-bounded tests.
    fn unit_sphere_solid() -> Solid {
        Solid::sphere(1.0)
    }

    fn unit_bounds() -> Aabb {
        Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0))
    }

    #[test]
    fn pinned_floor_shell_empty_caps_byte_identical_to_offset() {
        // Empty cap_planes path is the degenerate-correct no-cap fast
        // path: equivalent to `Solid::from_sdf(sdf).offset(offset_m)`
        // at every sample point. Pins that no spurious intersection
        // sneaks into the no-caps path.
        let sphere = unit_sphere_solid();
        let baseline = Solid::from_sdf(sphere.clone(), unit_bounds()).offset(0.1);
        let shell = pinned_floor_shell(sphere, unit_bounds(), &[], 0.1);

        for &p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.5, 0.0, 0.0),
            Point3::new(0.0, 0.7, 0.7),
            Point3::new(-1.2, 0.3, 0.0),
            Point3::new(0.3, -0.4, 1.1),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        ] {
            assert_relative_eq!(
                shell.evaluate(&p),
                baseline.evaluate(&p),
                epsilon = 0.0,
                max_relative = 0.0,
            );
        }
    }

    #[test]
    fn pinned_floor_shell_sphere_inset_pins_floor() {
        // Unit sphere, inward offset T = -0.1, cap plane at z = 0 with
        // outward normal +Z. Pinned floor at z = 0.
        // - At (0, 0, -0.05) (body-interior, below cap, well inside
        //   offset sphere): expect negative (inside shell).
        // - At (0, 0, +0.1) (above cap on cap-extension side): expect
        //   positive (clipped by cap).
        // - At (0, 0, 0.0) (on cap plane, at sphere center laterally):
        //   expect ~0 (on boundary).
        let sphere = unit_sphere_solid();
        let cap = (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let shell = pinned_floor_shell(sphere, unit_bounds(), &[cap], -0.1);

        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, -0.5)) < 0.0,
            "deep interior below cap should be inside",
        );
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, 0.1)) > 0.0,
            "above cap plane on cap-extension side should be outside",
        );
        // Boundary: at z = 0 in the interior — should be the larger of
        // (sphere_sd - (-0.1)) = (0 - radial_to_surface) - (-0.1) and
        // (z - 0) = 0. The max is whichever is on the boundary; either
        // way it's ≤ 0.
        let on_cap = shell.evaluate(&Point3::new(0.0, 0.0, 0.0));
        assert!(on_cap.abs() < 0.11, "cap-plane-interior eval: {on_cap}");
    }

    #[test]
    fn pinned_floor_shell_sphere_outer_pins_floor() {
        // Unit sphere, outward offset T = +0.2, same cap. Outer shell
        // reaches to radius 1.2 below the cap plane, clipped at z = 0
        // above.
        let sphere = unit_sphere_solid();
        let cap = (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let shell = pinned_floor_shell(sphere, unit_bounds(), &[cap], 0.2);

        // Just below the cap plane, well inside the offset sphere.
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, -0.5)) < 0.0,
            "deep interior below cap should be inside outer shell",
        );
        // Far above the cap plane: clipped.
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, 0.5)) > 0.0,
            "above cap plane should be clipped (outside)",
        );
        // Past the outer offset radius below the cap: outside.
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, -1.5)) > 0.0,
            "past outer offset radius should be outside",
        );
    }

    #[test]
    fn pinned_floor_shell_multi_cap_intersects_all() {
        // Two opposed caps (top at +Z, bottom at -Z); inset offset.
        // - Sphere center (0,0,0): inside both half-spaces, inside
        //   offset sphere → interior.
        // - (0, 0, +0.5): above top cap → clipped.
        // - (0, 0, -0.5): below bottom cap → clipped.
        let sphere = unit_sphere_solid();
        let top = (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let bot = (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0));
        let shell = pinned_floor_shell(sphere, unit_bounds(), &[top, bot], -0.5);

        // Both caps pass through z=0 — interior is degenerate (a thin
        // disk at z=0). Sample exactly there.
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, 0.0)) <= 0.0,
            "on both cap planes + at sphere center should be ≤ 0",
        );
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, 0.1)) > 0.0,
            "above top cap should be clipped",
        );
        assert!(
            shell.evaluate(&Point3::new(0.0, 0.0, -0.1)) > 0.0,
            "below bottom cap should be clipped",
        );
    }

    #[test]
    fn pinned_floor_shell_grad_dominated_by_cap_normal_outside_cap_plane() {
        // Sphere with cap at z=0 normal +Z, inset offset. At a point
        // ABOVE the cap plane (cap-extension side), the cap-plane
        // half-space SDF dominates the intersection (max), so the
        // gradient should match the cap normal.
        let sphere = unit_sphere_solid();
        let cap = (Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let shell = pinned_floor_shell(sphere, unit_bounds(), &[cap], -0.1);

        // Just above the cap plane, well inside the lateral sphere
        // extent. The plane SDF f(p) = z - 0 = +0.05 dominates the
        // sphere SDF at this point (which is ~-0.85, well below).
        let g = shell.gradient(&Point3::new(0.0, 0.0, 0.05));
        // The cap-plane gradient is (0, 0, 1).
        assert_relative_eq!(g.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(g.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(g.z, 1.0, epsilon = 1e-6);
    }
}
