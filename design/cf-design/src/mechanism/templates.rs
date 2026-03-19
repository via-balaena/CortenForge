//! Parameterized part templates.
//!
//! Helper functions returning pre-composed [`Part`] configurations with
//! sensible defaults. Templates compose primitives, booleans, organic
//! blends, and optionally `FlexZone`s with lattice infill.
//!
//! # Templates
//!
//! - [`finger`] — bio-inspired finger with phalanges and flex joints
//! - [`link`] / [`link_infilled`] — bone-like structural link
//! - [`bracket`] — flat mounting plate with rounded edges

use nalgebra::Vector3;

use super::flex_split::split_part;
use super::joint::{JointDef, JointKind};
use super::material::Material;
use super::part::{FlexZone, Part, Plane};
use crate::solid::{InfillKind, Solid};

// ── Finger ───────────────────────────────────────────────────────────────

/// Create a bio-inspired finger — an organic shape split into phalanges
/// by flex zones with derived spring-damper joints.
///
/// The finger is Z-aligned, base at `z=0`, tip at `z=length`. The shape
/// is a smooth blend of overlapping capsule segments (one per phalanx),
/// producing organic knuckle-like transitions.
///
/// Returns `(parts, joints)` where `parts[0]` (named `"{name}_0"`) is the
/// proximal phalanx — connect it to the parent body with a joint.
///
/// The material should have [`youngs_modulus`](Material::with_youngs_modulus)
/// set for meaningful spring stiffness; without it, flex joints have zero
/// stiffness.
///
/// # Panics
///
/// Panics if `name` is empty, `length` or `radius` is not positive/finite,
/// or `n_phalanges < 2`.
///
/// # Example
///
/// ```
/// use cf_design::{Material, templates::finger};
///
/// let pla = Material::new("PLA", 1250.0).with_youngs_modulus(3.5e9);
/// let (parts, joints) = finger("f1", 25.0, 3.0, 3, pla);
///
/// assert_eq!(parts.len(), 3);  // 3 phalanges
/// assert_eq!(joints.len(), 2); // 2 flex joints
/// assert_eq!(parts[0].name(), "f1_0"); // proximal
/// ```
#[must_use]
pub fn finger(
    name: &str,
    length: f64,
    radius: f64,
    n_phalanges: usize,
    material: Material,
) -> (Vec<Part>, Vec<JointDef>) {
    assert!(!name.is_empty(), "finger name must not be empty");
    assert!(
        length > 0.0 && length.is_finite(),
        "finger length must be positive and finite, got {length}"
    );
    assert!(
        radius > 0.0 && radius.is_finite(),
        "finger radius must be positive and finite, got {radius}"
    );
    assert!(
        n_phalanges >= 2,
        "n_phalanges must be at least 2, got {n_phalanges}"
    );

    // ── Build organic finger solid ──────────────────────────────────
    // Each phalanx is a capsule segment; smooth_union_all blends them
    // into an organic shape with natural knuckle transitions.
    #[allow(clippy::cast_precision_loss)]
    let phalanx_length = length / n_phalanges as f64;
    let seg_half_height = (phalanx_length / 2.0 - radius).max(0.01);

    let segments: Vec<Solid> = (0..n_phalanges)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let z_center = phalanx_length * (0.5 + i as f64);
            Solid::capsule(radius, seg_half_height).translate(Vector3::new(0.0, 0.0, z_center))
        })
        .collect();

    let solid = Solid::smooth_union_all(segments, radius * 0.5);

    // ── Add flex zones at phalanx boundaries ────────────────────────
    let flex_width = phalanx_length * 0.15;
    let mut part = Part::new(name, solid, material);

    for i in 1..n_phalanges {
        #[allow(clippy::cast_precision_loss)]
        let z = phalanx_length * i as f64;
        part = part.with_flex_zone(FlexZone::new(
            format!("{name}_flex_{}", i - 1),
            Plane::new(Vector3::z(), z),
            flex_width,
            JointKind::Revolute,
            Vector3::x(),
        ));
    }

    // ── Split into sub-bodies with spring-damper joints ─────────────
    let split = split_part(&part);
    let joints = split
        .joints
        .into_iter()
        .map(|fj| {
            let mut def = fj.def;
            if fj.stiffness > 0.0 {
                def = def.with_stiffness(fj.stiffness);
            }
            if fj.damping > 0.0 {
                def = def.with_damping(fj.damping);
            }
            def
        })
        .collect();

    (split.parts, joints)
}

// ── Link ─────────────────────────────────────────────────────────────────

/// Create a bone-like structural link.
///
/// Z-aligned capsule with base at `z=0` and tip at `z=length`. Rigid
/// (no flex zones). For a lightweight version with lattice infill, see
/// [`link_infilled`].
///
/// # Panics
///
/// Panics if `name` is empty, or `length`/`radius` is not positive/finite.
///
/// # Example
///
/// ```
/// use cf_design::{Material, templates::link};
///
/// let bone = link("femur", 30.0, 5.0, Material::new("PLA", 1250.0));
/// assert_eq!(bone.name(), "femur");
/// ```
#[must_use]
pub fn link(name: &str, length: f64, radius: f64, material: Material) -> Part {
    assert!(!name.is_empty(), "link name must not be empty");
    assert!(
        length > 0.0 && length.is_finite(),
        "link length must be positive and finite, got {length}"
    );
    assert!(
        radius > 0.0 && radius.is_finite(),
        "link radius must be positive and finite, got {radius}"
    );

    let half_height = (length / 2.0 - radius).max(0.0);
    let solid = Solid::capsule(radius, half_height).translate(Vector3::new(0.0, 0.0, length / 2.0));
    Part::new(name, solid, material)
}

/// Create a bone-like structural link with lattice infill.
///
/// Same shape as [`link`] but the interior is replaced with a periodic
/// lattice structure (gyroid or Schwarz P) while preserving a solid outer
/// shell. Reduces mass while maintaining structural integrity.
///
/// Infill defaults:
/// - `lattice_thickness = radius × 0.1`
/// - `wall_thickness = radius × 0.15`
/// - `scale = 2π / (radius × 0.8)` (one lattice period per ~0.8 × radius)
///
/// # Panics
///
/// Same as [`link`].
///
/// # Example
///
/// ```
/// use cf_design::{Material, InfillKind, templates::link_infilled};
///
/// let bone = link_infilled(
///     "femur", 30.0, 5.0,
///     Material::new("PLA", 1250.0),
///     InfillKind::Gyroid,
/// );
/// assert_eq!(bone.name(), "femur");
/// ```
#[must_use]
pub fn link_infilled(
    name: &str,
    length: f64,
    radius: f64,
    material: Material,
    kind: InfillKind,
) -> Part {
    assert!(!name.is_empty(), "link name must not be empty");
    assert!(
        length > 0.0 && length.is_finite(),
        "link length must be positive and finite, got {length}"
    );
    assert!(
        radius > 0.0 && radius.is_finite(),
        "link radius must be positive and finite, got {radius}"
    );

    let half_height = (length / 2.0 - radius).max(0.0);
    let scale = std::f64::consts::TAU / (radius * 0.8);
    let lattice_thickness = radius * 0.1;
    let wall_thickness = radius * 0.15;

    let solid = Solid::capsule(radius, half_height)
        .translate(Vector3::new(0.0, 0.0, length / 2.0))
        .infill(kind, scale, lattice_thickness, wall_thickness);
    Part::new(name, solid, material)
}

// ── Bracket ──────────────────────────────────────────────────────────────

/// Create a mounting bracket — a flat plate with rounded edges.
///
/// Centered at the origin. Dimensions are full extents (not half-extents).
/// Rounding radius is `min(thickness/4, 1.0)` for print-friendly edges.
///
/// # Panics
///
/// Panics if `name` is empty, or any dimension is not positive/finite.
///
/// # Example
///
/// ```
/// use cf_design::{Material, templates::bracket};
///
/// let mount = bracket("base", 40.0, 30.0, 4.0, Material::new("PLA", 1250.0));
/// assert_eq!(mount.name(), "base");
/// ```
#[must_use]
pub fn bracket(name: &str, width: f64, height: f64, thickness: f64, material: Material) -> Part {
    assert!(!name.is_empty(), "bracket name must not be empty");
    assert!(
        width > 0.0 && width.is_finite(),
        "bracket width must be positive and finite, got {width}"
    );
    assert!(
        height > 0.0 && height.is_finite(),
        "bracket height must be positive and finite, got {height}"
    );
    assert!(
        thickness > 0.0 && thickness.is_finite(),
        "bracket thickness must be positive and finite, got {thickness}"
    );

    let rounding = (thickness / 4.0).min(1.0);
    let solid =
        Solid::cuboid(Vector3::new(width / 2.0, height / 2.0, thickness / 2.0)).round(rounding);
    Part::new(name, solid, material)
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    fn pla() -> Material {
        Material::new("PLA", 1250.0).with_youngs_modulus(3.5e9)
    }

    fn pla_no_e() -> Material {
        Material::new("PLA", 1250.0)
    }

    // ── Finger ──────────────────────────────────────────────────────

    #[test]
    fn finger_three_phalanges() {
        let (parts, joints) = finger("f", 25.0, 3.0, 3, pla());

        assert_eq!(parts.len(), 3, "3 phalanges → 3 parts");
        assert_eq!(joints.len(), 2, "3 phalanges → 2 joints");

        assert_eq!(parts[0].name(), "f_0");
        assert_eq!(parts[1].name(), "f_1");
        assert_eq!(parts[2].name(), "f_2");

        assert_eq!(joints[0].parent(), "f_0");
        assert_eq!(joints[0].child(), "f_1");
        assert_eq!(joints[1].parent(), "f_1");
        assert_eq!(joints[1].child(), "f_2");
    }

    #[test]
    fn finger_two_phalanges() {
        let (parts, joints) = finger("f", 20.0, 2.5, 2, pla());

        assert_eq!(parts.len(), 2);
        assert_eq!(joints.len(), 1);

        assert_eq!(parts[0].name(), "f_0");
        assert_eq!(parts[1].name(), "f_1");
    }

    #[test]
    fn finger_four_phalanges() {
        let (parts, joints) = finger("digit", 40.0, 3.0, 4, pla());

        assert_eq!(parts.len(), 4);
        assert_eq!(joints.len(), 3);
    }

    #[test]
    fn finger_has_stiffness_with_youngs_modulus() {
        let (_, joints) = finger("f", 25.0, 3.0, 3, pla());

        for (i, j) in joints.iter().enumerate() {
            assert!(
                j.stiffness().unwrap_or(0.0) > 0.0,
                "joint {i} should have positive stiffness"
            );
            assert!(
                j.damping().unwrap_or(0.0) > 0.0,
                "joint {i} should have positive damping"
            );
        }
    }

    #[test]
    fn finger_zero_stiffness_without_youngs_modulus() {
        let (_, joints) = finger("f", 25.0, 3.0, 3, pla_no_e());

        // Without Young's modulus, stiffness and damping should not be set.
        for j in &joints {
            assert!(
                j.stiffness().is_none(),
                "stiffness should be None without Young's modulus"
            );
        }
    }

    #[test]
    fn finger_sub_bodies_contain_expected_regions() {
        let (parts, _) = finger("f", 30.0, 3.0, 3, pla());

        // Phalanx length = 10. Centers at z = 5, 15, 25.
        // Sub-body 0 should contain z=5 region.
        assert!(
            parts[0].solid().evaluate(&Point3::new(0.0, 0.0, 5.0)) < 0.0,
            "f_0 should contain z=5"
        );
        // Sub-body 2 should contain z=25 region.
        assert!(
            parts[2].solid().evaluate(&Point3::new(0.0, 0.0, 25.0)) < 0.0,
            "f_2 should contain z=25"
        );
    }

    #[test]
    fn finger_joints_are_revolute() {
        let (_, joints) = finger("f", 25.0, 3.0, 3, pla());

        for j in &joints {
            assert_eq!(j.kind(), JointKind::Revolute);
        }
    }

    #[test]
    #[should_panic(expected = "n_phalanges must be at least 2")]
    fn finger_rejects_one_phalanx() {
        drop(finger("f", 20.0, 3.0, 1, pla()));
    }

    #[test]
    #[should_panic(expected = "finger name must not be empty")]
    fn finger_rejects_empty_name() {
        drop(finger("", 20.0, 3.0, 2, pla()));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn finger_rejects_zero_length() {
        drop(finger("f", 0.0, 3.0, 2, pla()));
    }

    // ── Link ────────────────────────────────────────────────────────

    #[test]
    fn link_basic() {
        let p = link("bone", 15.0, 4.0, pla());

        assert_eq!(p.name(), "bone");
        // Should be solid at the center (z = 7.5).
        assert!(
            p.solid().evaluate(&Point3::new(0.0, 0.0, 7.5)) < 0.0,
            "link center should be solid"
        );
        // Should be outside well beyond the ends.
        assert!(
            p.solid().evaluate(&Point3::new(0.0, 0.0, -5.0)) > 0.0,
            "below base should be outside"
        );
        assert!(
            p.solid().evaluate(&Point3::new(0.0, 0.0, 20.0)) > 0.0,
            "above tip should be outside"
        );
    }

    #[test]
    fn link_no_flex_zones() {
        let p = link("bone", 15.0, 4.0, pla());
        assert!(p.flex_zones().is_empty());
    }

    #[test]
    #[should_panic(expected = "link name must not be empty")]
    fn link_rejects_empty_name() {
        drop(link("", 15.0, 4.0, pla()));
    }

    // ── Link infilled ───────────────────────────────────────────────

    #[test]
    fn link_infilled_has_lattice() {
        let p = link_infilled("bone", 20.0, 5.0, pla(), InfillKind::Gyroid);

        assert_eq!(p.name(), "bone");

        // Interior should have mixed solid/void regions (lattice).
        // Sample several points along the axis interior — some should
        // be inside (lattice wall) and some outside (lattice void).
        let mut has_inside = false;
        let mut has_outside = false;
        for i in 0..20 {
            #[allow(clippy::cast_precision_loss)]
            let z = f64::from(i).mul_add(0.7, 3.0);
            let val = p.solid().evaluate(&Point3::new(0.0, 0.0, z));
            if val < 0.0 {
                has_inside = true;
            } else {
                has_outside = true;
            }
        }
        assert!(
            has_inside && has_outside,
            "infilled link should have both solid and void regions in interior"
        );
    }

    // ── Bracket ─────────────────────────────────────────────────────

    #[test]
    fn bracket_basic() {
        let p = bracket("mount", 20.0, 15.0, 3.0, pla());

        assert_eq!(p.name(), "mount");
        // Center should be inside.
        assert!(
            p.solid().evaluate(&Point3::origin()) < 0.0,
            "bracket center should be solid"
        );
        // Far outside should be outside.
        assert!(
            p.solid().evaluate(&Point3::new(20.0, 20.0, 20.0)) > 0.0,
            "far point should be outside"
        );
    }

    #[test]
    fn bracket_no_flex_zones() {
        let p = bracket("mount", 20.0, 15.0, 3.0, pla());
        assert!(p.flex_zones().is_empty());
    }

    #[test]
    #[should_panic(expected = "bracket name must not be empty")]
    fn bracket_rejects_empty_name() {
        drop(bracket("", 20.0, 15.0, 3.0, pla()));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn bracket_rejects_zero_width() {
        drop(bracket("b", 0.0, 15.0, 3.0, pla()));
    }
}
