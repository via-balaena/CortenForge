//! STL mesh generation from mechanism parts.
//!
//! Generates per-part triangle meshes suitable for 3D printing (STL export).
//! When a [`PrintProfile`](super::PrintProfile) is set on the mechanism, each
//! part is shrunk by half the clearance via [`Solid::offset`](crate::Solid::offset)
//! so that mating surfaces have the required gap.

use cf_geometry::IndexedMesh;

use super::builder::Mechanism;

// ── Generation ──────────────────────────────────────────────────────────

/// Generate per-part meshes for 3D printing.
///
/// Returns `(part_name, IndexedMesh)` pairs in part declaration order.
///
/// If the mechanism has a [`PrintProfile`](super::PrintProfile), each part's
/// solid is offset by `-clearance / 2` before meshing. Both mating surfaces
/// shrink by half the clearance, so the total gap equals the full clearance.
///
/// # Panics
///
/// Panics if `tolerance` is not positive and finite.
pub(super) fn generate(mechanism: &Mechanism, tolerance: f64) -> Vec<(String, IndexedMesh)> {
    assert!(
        tolerance > 0.0 && tolerance.is_finite(),
        "STL tolerance must be positive and finite, got {tolerance}"
    );

    let clearance_offset = mechanism
        .print_profile()
        .map_or(0.0, |pp| -pp.clearance / 2.0);

    mechanism
        .parts()
        .iter()
        .map(|part| {
            let mesh = if clearance_offset == 0.0 {
                part.solid().mesh(tolerance)
            } else {
                // Clone the solid and apply clearance offset (shrink).
                let offset_solid = part.solid().clone().offset(clearance_offset);
                offset_solid.mesh(tolerance)
            };
            (part.name().to_owned(), mesh)
        })
        .collect()
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::{JointDef, JointKind, Material, Mechanism, Part, PrintProfile, Solid};

    use super::*;

    fn pla() -> Material {
        Material::new("PLA", 1250.0)
    }

    fn two_part_mechanism() -> Mechanism {
        Mechanism::builder("gripper")
            .part(Part::new(
                "palm",
                Solid::cuboid(Vector3::new(10.0, 10.0, 5.0)),
                pla(),
            ))
            .part(Part::new(
                "finger",
                Solid::cuboid(Vector3::new(3.0, 3.0, 8.0)),
                pla(),
            ))
            .joint(JointDef::new(
                "j",
                "palm",
                "finger",
                JointKind::Revolute,
                Point3::new(10.0, 0.0, 0.0),
                Vector3::x(),
            ))
            .build()
    }

    // ── 1. Generates meshes for all parts ──────────────────────────────

    #[test]
    fn generates_mesh_per_part() {
        let m = two_part_mechanism();
        let stls = generate(&m, 1.0);

        assert_eq!(stls.len(), 2);
        assert_eq!(stls[0].0, "palm");
        assert_eq!(stls[1].0, "finger");

        for (name, mesh) in &stls {
            assert!(!mesh.is_empty(), "mesh for \"{name}\" should not be empty");
        }
    }

    // ── 2. Meshes are non-empty with valid geometry ────────────────────

    #[test]
    fn meshes_have_positive_volume() {
        let m = two_part_mechanism();
        let stls = generate(&m, 1.0);

        for (name, mesh) in &stls {
            assert!(
                mesh.volume() > 0.0,
                "mesh for \"{name}\" should have positive volume"
            );
        }
    }

    // ── 3. Without clearance: no offset applied ────────────────────────

    #[test]
    fn no_clearance_no_offset() {
        let m = two_part_mechanism();
        assert!(m.print_profile().is_none());

        let stls = generate(&m, 1.0);
        // Just verify meshes are generated — no offset applied.
        assert_eq!(stls.len(), 2);
    }

    // ── 4. With clearance: parts are smaller ───────────────────────────

    #[test]
    fn clearance_shrinks_parts() {
        // Build two identical mechanisms — one with clearance, one without.
        let m_no_clearance = Mechanism::builder("no_clear")
            .part(Part::new(
                "box",
                Solid::cuboid(Vector3::new(10.0, 10.0, 10.0)),
                pla(),
            ))
            .build();

        let m_with_clearance = Mechanism::builder("with_clear")
            .part(Part::new(
                "box",
                Solid::cuboid(Vector3::new(10.0, 10.0, 10.0)),
                pla(),
            ))
            .print_profile(PrintProfile::new(0.5, 1.0, 2.0))
            .build();

        let stl_no = generate(&m_no_clearance, 0.5);
        let stl_yes = generate(&m_with_clearance, 0.5);

        let vol_no = stl_no[0].1.volume();
        let vol_yes = stl_yes[0].1.volume();

        assert!(
            vol_yes < vol_no,
            "clearance-offset mesh volume ({vol_yes}) should be less than unmodified ({vol_no})"
        );
    }

    // ── 5. Single-part mechanism ───────────────────────────────────────

    #[test]
    fn single_part_stl() {
        let m = Mechanism::builder("one")
            .part(Part::new("body", Solid::sphere(5.0), pla()))
            .build();

        let stls = generate(&m, 1.0);
        assert_eq!(stls.len(), 1);
        assert_eq!(stls[0].0, "body");
        assert!(!stls[0].1.is_empty());
    }

    // ── 6. Clearance offset dimensional accuracy ─────────────────────

    #[test]
    fn clearance_offset_shrinks_bounding_box() {
        let clearance = 0.6; // total gap between mating surfaces
        let half = Vector3::new(10.0, 10.0, 10.0);

        let m_no_clear = Mechanism::builder("no_clear")
            .part(Part::new("box", Solid::cuboid(half), pla()))
            .build();
        let m_with_clear = Mechanism::builder("with_clear")
            .part(Part::new("box", Solid::cuboid(half), pla()))
            .print_profile(PrintProfile::new(clearance, 1.0, 2.0))
            .build();

        let stl_no = generate(&m_no_clear, 0.5);
        let stl_yes = generate(&m_with_clear, 0.5);

        // Compute bounding box extents.
        let extent = |mesh: &cf_geometry::IndexedMesh| -> [f64; 3] {
            let (mut mn, mut mx) = ([f64::MAX; 3], [f64::MIN; 3]);
            for v in &mesh.vertices {
                mn[0] = mn[0].min(v.x);
                mn[1] = mn[1].min(v.y);
                mn[2] = mn[2].min(v.z);
                mx[0] = mx[0].max(v.x);
                mx[1] = mx[1].max(v.y);
                mx[2] = mx[2].max(v.z);
            }
            [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]]
        };

        let ext_no = extent(&stl_no[0].1);
        let ext_yes = extent(&stl_yes[0].1);

        // Each axis should shrink by approximately clearance (half from each side).
        for axis in 0..3 {
            let shrinkage = ext_no[axis] - ext_yes[axis];
            assert!(
                shrinkage > 0.0,
                "axis {axis}: clearance should shrink extent, got no={:.3}, yes={:.3}",
                ext_no[axis],
                ext_yes[axis]
            );
            // Expect shrinkage ≈ clearance (±50% tolerance due to meshing discretization).
            assert!(
                shrinkage < clearance * 2.0,
                "axis {axis}: shrinkage {shrinkage:.3} is unreasonably large vs clearance {clearance}"
            );
        }
    }

    // ── 7. Panic on invalid tolerance ──────────────────────────────────

    #[test]
    #[should_panic(expected = "STL tolerance must be positive")]
    fn rejects_zero_tolerance() {
        let m = Mechanism::builder("bad")
            .part(Part::new("a", Solid::sphere(1.0), pla()))
            .build();
        let _ = generate(&m, 0.0);
    }
}
