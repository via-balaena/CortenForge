//! `FlexZone` auto-splitting.
//!
//! Splits a [`Part`] at its [`FlexZone`]s into sub-bodies connected by joints
//! with spring-damper properties derived from material Young's modulus and
//! cross-section geometry.
//!
//! # Stiffness derivation
//!
//! Uses Euler-Bernoulli beam theory for short flexure sections:
//!
//! - **Revolute/Ball**: `k = E × I / L` (rotational stiffness, N·mm/rad)
//! - **Prismatic**: `k = E × A / L` (translational stiffness, N/mm)
//! - **Free**: `k = 0` (unconstrained)
//!
//! Where `E` is Young's modulus (converted from Pa to N/mm²), `I` is the second
//! moment of area about the flex axis, `A` is the cross-section area, and `L` is
//! the flex zone width.
//!
//! Damping is proportional to stiffness: `c = 0.01 × k`.

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

use super::joint::{JointDef, JointKind};
use super::part::{FlexZone, Part, Plane};
use crate::Solid;

/// Damping time constant (seconds). Damping coefficient = stiffness × this value.
const DAMPING_TIME_CONSTANT: f64 = 0.01;

/// Conversion factor: Pa (N/m²) → N/mm².
const PA_TO_N_PER_MM2: f64 = 1e-6;

// ── Types ──────────────────────────────────────────────────────────────────

/// Cross-section properties measured at a flex zone plane.
///
/// Computed by sampling the solid on the flex zone plane and integrating.
#[derive(Debug, Clone)]
pub struct CrossSection {
    /// Cross-section area (mm²).
    pub area: f64,
    /// Second moment of area about the flex axis through the centroid (mm⁴).
    pub second_moment: f64,
    /// Centroid of the cross-section (part-local frame, mm).
    pub centroid: Point3<f64>,
}

/// Joint generated from a [`FlexZone`] with derived spring-damper properties.
///
/// The stiffness and damping are derived from the material's Young's modulus
/// and the cross-section geometry using Euler-Bernoulli beam theory.
#[derive(Debug)]
pub struct FlexJoint {
    /// Joint definition connecting two sub-parts.
    pub def: JointDef,
    /// Spring stiffness (N·mm/rad for revolute/ball, N/mm for prismatic, 0 for free).
    pub stiffness: f64,
    /// Damping coefficient (proportional to stiffness: 0.01 × k).
    pub damping: f64,
}

/// Result of splitting a [`Part`] at its [`FlexZone`]s.
///
/// For a [`Part`] with N flex zones, produces N+1 sub-body Parts and N [`FlexJoint`]s.
/// Sub-bodies are named `"{original_name}_{index}"`.
#[derive(Debug)]
pub struct SplitResult {
    /// Sub-body Parts. For N flex zones, this contains N+1 parts.
    pub parts: Vec<Part>,
    /// Joints connecting adjacent sub-bodies with spring-damper properties.
    pub joints: Vec<FlexJoint>,
}

// ── Public API ─────────────────────────────────────────────────────────────

/// Split a Part at its flex zones into sub-bodies with spring-damper joints.
///
/// - If the part has no flex zones, returns the original part with no joints.
/// - If the solid has no finite bounds, returns the original part with no joints.
/// - If the material lacks Young's modulus, geometry splits but stiffness = 0.
///
/// Flex zones are sorted by plane offset (ascending). All flex zones are assumed
/// to share the same normal direction — the common case for segmented links.
#[must_use]
pub fn split_part(part: &Part) -> SplitResult {
    if part.flex_zones().is_empty() {
        return SplitResult {
            parts: vec![part.clone()],
            joints: Vec::new(),
        };
    }

    let solid = part.solid();
    let Some(bounds) = solid.bounds() else {
        return SplitResult {
            parts: vec![part.clone()],
            joints: Vec::new(),
        };
    };

    // Sort zones by plane offset (ascending).
    let mut zones: Vec<&FlexZone> = part.flex_zones().iter().collect();
    zones.sort_by(|a, b| a.plane().offset().total_cmp(&b.plane().offset()));

    let n = zones.len();
    let youngs_modulus = part.material().youngs_modulus;
    let cell_size = adaptive_cell_size(&bounds);

    // Measure cross-sections.
    let cross_sections: Vec<CrossSection> = zones
        .iter()
        .map(|z| measure_cross_section(solid, z.plane(), z.axis(), cell_size))
        .collect();

    // Split the solid into N+1 sub-bodies.
    let mut sub_solids: Vec<Solid> = Vec::with_capacity(n + 1);

    for i in 0..=n {
        let sub = if i == 0 {
            // Below first zone.
            let normal = *zones[0].plane().normal();
            let offset = zones[0].plane().offset();
            solid.clone().intersect(Solid::plane(normal, offset))
        } else if i == n {
            // Above last zone.
            let normal = *zones[n - 1].plane().normal();
            let offset = zones[n - 1].plane().offset();
            solid.clone().intersect(Solid::plane(-normal, -offset))
        } else {
            // Between zone i-1 and zone i.
            let n_prev = *zones[i - 1].plane().normal();
            let off_prev = zones[i - 1].plane().offset();
            let n_next = *zones[i].plane().normal();
            let off_next = zones[i].plane().offset();
            solid
                .clone()
                .intersect(Solid::plane(-n_prev, -off_prev))
                .intersect(Solid::plane(n_next, off_next))
        };
        sub_solids.push(sub);
    }

    // Create sub-body Parts.
    let parts: Vec<Part> = sub_solids
        .into_iter()
        .enumerate()
        .map(|(i, sub_solid)| {
            Part::new(
                format!("{}_{i}", part.name()),
                sub_solid,
                part.material().clone(),
            )
        })
        .collect();

    // Create FlexJoints.
    let joints: Vec<FlexJoint> = zones
        .iter()
        .enumerate()
        .map(|(i, zone)| {
            let cs = &cross_sections[i];
            let (stiffness, damping) =
                compute_stiffness(cs, youngs_modulus, zone.width(), zone.kind());

            let def = JointDef::new(
                zone.name(),
                format!("{}_{i}", part.name()),
                format!("{}_{}", part.name(), i + 1),
                zone.kind(),
                cs.centroid,
                *zone.axis(),
            );

            FlexJoint {
                def,
                stiffness,
                damping,
            }
        })
        .collect();

    SplitResult { parts, joints }
}

/// Measure cross-section properties of a solid at a plane.
///
/// Samples the solid on the given plane using a 2D grid. Returns the area,
/// second moment of area about `axis` through the centroid, and the centroid.
///
/// The `cell_size` controls sampling resolution (mm). Smaller values give
/// more accurate results but take longer.
#[must_use]
pub fn measure_cross_section(
    solid: &Solid,
    plane: &Plane,
    axis: &Vector3<f64>,
    cell_size: f64,
) -> CrossSection {
    let empty = CrossSection {
        area: 0.0,
        second_moment: 0.0,
        centroid: Point3::origin(),
    };

    let Some(bounds) = solid.bounds() else {
        return empty;
    };

    let normal = *plane.normal();
    let offset = plane.offset();
    let (basis_u, basis_v) = orthonormal_basis(&normal);

    // Project AABB onto the plane to find sampling extents (support function).
    let half_size = bounds.size() * 0.5;
    let center = bounds.center();

    let u_center = center.coords.dot(&basis_u);
    let u_half = half_size.x.mul_add(
        basis_u.x.abs(),
        half_size
            .y
            .mul_add(basis_u.y.abs(), half_size.z * basis_u.z.abs()),
    );
    let v_center = center.coords.dot(&basis_v);
    let v_half = half_size.x.mul_add(
        basis_v.x.abs(),
        half_size
            .y
            .mul_add(basis_v.y.abs(), half_size.z * basis_v.z.abs()),
    );

    let u_min = u_center - u_half;
    let v_min = v_center - v_half;

    // Index/count conversion bounded by domain (size well below 2^32).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let nu = (2.0 * u_half / cell_size).ceil().max(1.0) as usize;
    // Index/count conversion bounded by domain (size well below 2^32).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let nv = (2.0 * v_half / cell_size).ceil().max(1.0) as usize;

    let cell_area = cell_size * cell_size;
    let half_step = cell_size * 0.5;
    let plane_origin = offset * normal;

    // Collect interior points.
    let mut interior: Vec<Point3<f64>> = Vec::new();

    for iv in 0..nv {
        // Precision loss acceptable for approximate / visualization values.
        #[allow(clippy::cast_precision_loss)]
        let vc = v_min + (iv as f64).mul_add(cell_size, half_step);
        for iu in 0..nu {
            // Precision loss acceptable for approximate / visualization values.
            #[allow(clippy::cast_precision_loss)]
            let uc = u_min + (iu as f64).mul_add(cell_size, half_step);

            let pt = Point3::origin() + plane_origin + uc * basis_u + vc * basis_v;
            if solid.evaluate(&pt) < 0.0 {
                interior.push(pt);
            }
        }
    }

    if interior.is_empty() {
        return empty;
    }

    // Precision loss acceptable for approximate / visualization values.
    #[allow(clippy::cast_precision_loss)]
    let area = interior.len() as f64 * cell_area;

    // Centroid.
    let mut sum = Vector3::zeros();
    for pt in &interior {
        sum += pt.coords;
    }
    // Precision loss acceptable for approximate / visualization values.
    #[allow(clippy::cast_precision_loss)]
    let n_pts = interior.len() as f64;
    let centroid = Point3::from(sum / n_pts);

    // Second moment of area about the flex axis through the centroid.
    let axis_unit = axis.normalize();
    let mut second_moment = 0.0;
    for pt in &interior {
        let rel = pt - centroid;
        let along = rel.dot(&axis_unit);
        let perp_sq = along.mul_add(-along, rel.norm_squared());
        second_moment += perp_sq * cell_area;
    }

    CrossSection {
        area,
        second_moment,
        centroid,
    }
}

// ── Internal helpers ───────────────────────────────────────────────────────

/// Compute spring stiffness and damping from cross-section properties.
///
/// Returns `(stiffness, damping)`. If Young's modulus is `None`, both are 0.
fn compute_stiffness(
    cs: &CrossSection,
    youngs_modulus: Option<f64>,
    width: f64,
    kind: JointKind,
) -> (f64, f64) {
    let Some(e_pa) = youngs_modulus else {
        return (0.0, 0.0);
    };

    let e = e_pa * PA_TO_N_PER_MM2; // N/mm²

    let k = match kind {
        JointKind::Revolute | JointKind::Ball => e * cs.second_moment / width,
        JointKind::Prismatic => e * cs.area / width,
        JointKind::Free => 0.0,
    };

    let c = DAMPING_TIME_CONSTANT * k;
    (k, c)
}

/// Construct an orthonormal basis `(u, v)` perpendicular to the given normal.
fn orthonormal_basis(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let not_parallel = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = normal.cross(&not_parallel).normalize();
    let v = normal.cross(&u);
    (u, v)
}

/// Choose an adaptive cell size for cross-section sampling based on AABB.
fn adaptive_cell_size(bounds: &Aabb) -> f64 {
    let size = bounds.size();
    let min_dim = size.x.min(size.y).min(size.z);
    // ~50 samples across the smallest dimension.
    (min_dim / 50.0).max(0.01)
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::{FlexZone, JointKind, Material, Part, Plane, Solid};

    fn pla_with_e() -> Material {
        Material::new("PLA", 1250.0).with_youngs_modulus(3.5e9)
    }

    fn pla_no_e() -> Material {
        Material::new("PLA", 1250.0)
    }

    // ── 1. Cross-section: rectangular beam ───────────────────────────

    #[test]
    fn cross_section_rectangular() {
        // Cuboid 4×4×40 (half-extents 2×2×20). Cross-section at z=0 is 4×4.
        let solid = Solid::cuboid(Vector3::new(2.0, 2.0, 20.0));
        let plane = Plane::new(Vector3::z(), 0.0);
        let axis = Vector3::x();

        let cs = measure_cross_section(&solid, &plane, &axis, 0.05);

        let expected_area = 16.0; // 4 × 4
        let expected_i = 4.0 * 4.0_f64.powi(3) / 12.0; // bh³/12 = 21.333

        let area_err = (cs.area - expected_area).abs() / expected_area;
        let i_err = (cs.second_moment - expected_i).abs() / expected_i;

        assert!(
            area_err < 0.05,
            "area {:.3} vs expected {expected_area}, err {area_err:.4}",
            cs.area
        );
        assert!(
            i_err < 0.05,
            "I {:.3} vs expected {expected_i:.3}, err {i_err:.4}",
            cs.second_moment
        );

        // Centroid should be near origin (symmetric cross-section).
        assert!(
            cs.centroid.coords.norm() < 0.2,
            "centroid should be near origin, got {:?}",
            cs.centroid
        );
    }

    // ── 2. Cross-section: circular beam ──────────────────────────────

    #[test]
    fn cross_section_circular() {
        // Cylinder r=3, half_height=15. Cross-section at z=0 is circle r=3.
        let solid = Solid::cylinder(3.0, 15.0);
        let plane = Plane::new(Vector3::z(), 0.0);
        let axis = Vector3::x();

        let cs = measure_cross_section(&solid, &plane, &axis, 0.05);

        let expected_area = PI * 9.0; // πr² ≈ 28.274
        let expected_i = PI * 81.0 / 4.0; // πr⁴/4 ≈ 63.617

        let area_err = (cs.area - expected_area).abs() / expected_area;
        let i_err = (cs.second_moment - expected_i).abs() / expected_i;

        assert!(
            area_err < 0.05,
            "area {:.3} vs expected {expected_area:.3}, err {area_err:.4}",
            cs.area
        );
        assert!(
            i_err < 0.05,
            "I {:.3} vs expected {expected_i:.3}, err {i_err:.4}",
            cs.second_moment
        );

        assert!(
            cs.centroid.coords.norm() < 0.2,
            "centroid should be near origin, got {:?}",
            cs.centroid
        );
    }

    // ── 3. Split geometry: single zone ───────────────────────────────

    #[test]
    fn split_single_zone_geometry() {
        let part = Part::new(
            "beam",
            Solid::cuboid(Vector3::new(2.0, 2.0, 20.0)),
            pla_with_e(),
        )
        .with_flex_zone(FlexZone::new(
            "flex_0",
            Plane::new(Vector3::z(), 0.0),
            2.0,
            JointKind::Revolute,
            Vector3::x(),
        ));

        let result = split_part(&part);

        assert_eq!(result.parts.len(), 2, "single zone → 2 sub-bodies");
        assert_eq!(result.joints.len(), 1, "single zone → 1 joint");

        // Sub-body 0: z < 0 region.
        let p0 = &result.parts[0];
        assert_eq!(p0.name(), "beam_0");
        assert!(
            p0.solid().evaluate(&Point3::new(0.0, 0.0, -10.0)) < 0.0,
            "beam_0 should contain (0,0,-10)"
        );
        assert!(
            p0.solid().evaluate(&Point3::new(0.0, 0.0, 10.0)) > 0.0,
            "beam_0 should NOT contain (0,0,10)"
        );

        // Sub-body 1: z > 0 region.
        let p1 = &result.parts[1];
        assert_eq!(p1.name(), "beam_1");
        assert!(
            p1.solid().evaluate(&Point3::new(0.0, 0.0, 10.0)) < 0.0,
            "beam_1 should contain (0,0,10)"
        );
        assert!(
            p1.solid().evaluate(&Point3::new(0.0, 0.0, -10.0)) > 0.0,
            "beam_1 should NOT contain (0,0,-10)"
        );

        // Joint connects beam_0 to beam_1.
        let j = &result.joints[0];
        assert_eq!(j.def.name(), "flex_0");
        assert_eq!(j.def.parent(), "beam_0");
        assert_eq!(j.def.child(), "beam_1");
        assert_eq!(j.def.kind(), JointKind::Revolute);
    }

    // ── 4. Split stiffness: single zone ──────────────────────────────

    #[test]
    fn split_single_zone_stiffness() {
        // Cuboid 4×4×40, E=3.5 GPa, FlexZone width=2, revolute about X.
        // Expected: k = E_local × I / L = 3500 × 21.333 / 2 ≈ 37333 N·mm/rad
        let part = Part::new(
            "beam",
            Solid::cuboid(Vector3::new(2.0, 2.0, 20.0)),
            pla_with_e(),
        )
        .with_flex_zone(FlexZone::new(
            "flex_0",
            Plane::new(Vector3::z(), 0.0),
            2.0,
            JointKind::Revolute,
            Vector3::x(),
        ));

        let result = split_part(&part);
        let j = &result.joints[0];

        let e_local = 3.5e9 * 1e-6; // 3500 N/mm²
        let i_x = 4.0 * 4.0_f64.powi(3) / 12.0; // 21.333 mm⁴
        let width = 2.0;
        let expected_k = e_local * i_x / width; // ≈ 37333 N·mm/rad
        let expected_c = 0.01 * expected_k;

        let k_err = (j.stiffness - expected_k).abs() / expected_k;
        let c_err = (j.damping - expected_c).abs() / expected_c;

        assert!(
            k_err < 0.05,
            "stiffness {:.1} vs expected {expected_k:.1}, err {k_err:.4}",
            j.stiffness
        );
        assert!(
            c_err < 0.05,
            "damping {:.2} vs expected {expected_c:.2}, err {c_err:.4}",
            j.damping
        );
    }

    // ── 5. Split: multiple zones ─────────────────────────────────────

    #[test]
    fn split_multiple_zones() {
        let part = Part::new(
            "link",
            Solid::cuboid(Vector3::new(2.0, 2.0, 30.0)),
            pla_with_e(),
        )
        .with_flex_zone(FlexZone::new(
            "flex_a",
            Plane::new(Vector3::z(), -10.0),
            1.5,
            JointKind::Revolute,
            Vector3::x(),
        ))
        .with_flex_zone(FlexZone::new(
            "flex_b",
            Plane::new(Vector3::z(), 10.0),
            1.5,
            JointKind::Revolute,
            Vector3::x(),
        ));

        let result = split_part(&part);

        assert_eq!(result.parts.len(), 3, "2 zones → 3 sub-bodies");
        assert_eq!(result.joints.len(), 2, "2 zones → 2 joints");

        assert_eq!(result.parts[0].name(), "link_0");
        assert_eq!(result.parts[1].name(), "link_1");
        assert_eq!(result.parts[2].name(), "link_2");

        assert_eq!(result.joints[0].def.name(), "flex_a");
        assert_eq!(result.joints[0].def.parent(), "link_0");
        assert_eq!(result.joints[0].def.child(), "link_1");

        assert_eq!(result.joints[1].def.name(), "flex_b");
        assert_eq!(result.joints[1].def.parent(), "link_1");
        assert_eq!(result.joints[1].def.child(), "link_2");

        // Middle sub-body should contain z=0, not z=±20.
        let mid = &result.parts[1];
        assert!(
            mid.solid().evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0,
            "link_1 should contain (0,0,0)"
        );
        assert!(
            mid.solid().evaluate(&Point3::new(0.0, 0.0, -20.0)) > 0.0,
            "link_1 should NOT contain (0,0,-20)"
        );
        assert!(
            mid.solid().evaluate(&Point3::new(0.0, 0.0, 20.0)) > 0.0,
            "link_1 should NOT contain (0,0,20)"
        );
    }

    // ── 6. Split: no zones (passthrough) ─────────────────────────────

    #[test]
    fn split_no_zones() {
        let part = Part::new(
            "block",
            Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)),
            pla_with_e(),
        );

        let result = split_part(&part);

        assert_eq!(result.parts.len(), 1);
        assert_eq!(result.joints.len(), 0);
        assert_eq!(result.parts[0].name(), "block");
    }

    // ── 7. Split: missing Young's modulus ────────────────────────────

    #[test]
    fn split_missing_youngs_modulus() {
        let part = Part::new(
            "beam",
            Solid::cuboid(Vector3::new(2.0, 2.0, 20.0)),
            pla_no_e(),
        )
        .with_flex_zone(FlexZone::new(
            "flex_0",
            Plane::new(Vector3::z(), 0.0),
            2.0,
            JointKind::Revolute,
            Vector3::x(),
        ));

        let result = split_part(&part);

        // Geometry should still split.
        assert_eq!(result.parts.len(), 2);
        assert_eq!(result.joints.len(), 1);

        // Stiffness and damping should be zero.
        assert!(
            result.joints[0].stiffness.abs() < f64::EPSILON,
            "stiffness should be 0 without Young's modulus"
        );
        assert!(
            result.joints[0].damping.abs() < f64::EPSILON,
            "damping should be 0 without Young's modulus"
        );

        // Geometry still correct.
        assert!(
            result.parts[0]
                .solid()
                .evaluate(&Point3::new(0.0, 0.0, -10.0))
                < 0.0
        );
        assert!(
            result.parts[1]
                .solid()
                .evaluate(&Point3::new(0.0, 0.0, 10.0))
                < 0.0
        );
    }
}
