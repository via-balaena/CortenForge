//! Per-tet quality computation kernel.
//!
//! Closed-form per-tet primitives — signed volume, aspect ratio
//! (inscribed-over-circumscribed sphere ratio), and dihedral-angle
//! min/max across the six edges — over a 4-vertex tet. Lives in
//! `mesh/` (not `sdf_bridge/`) to avoid a `mesh → sdf_bridge` cycle:
//! `sdf_bridge` already imports from `mesh` (`Mesh` trait,
//! `VertexId`, `QualityMetrics`), so the reverse direction would
//! close the loop.
//!
//! Determinism: f64 `sqrt` and `acos` are bit-deterministic on a
//! fixed hardware/libm pair. No FP-fragile operations beyond `acos`
//! of normalised dot products and `sqrt` in radii.

use crate::Vec3;
use nalgebra::Matrix3;

fn signed_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
    m.determinant() / 6.0
}

fn aspect_ratio(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    inradius(p0, p1, p2, p3) / circumradius(p0, p1, p2, p3)
}

fn dihedral_min_max(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> (f64, f64) {
    let angles = [
        dihedral_angle(p0, p1, p2, p3),
        dihedral_angle(p0, p2, p1, p3),
        dihedral_angle(p0, p3, p1, p2),
        dihedral_angle(p1, p2, p0, p3),
        dihedral_angle(p1, p3, p0, p2),
        dihedral_angle(p2, p3, p0, p1),
    ];
    let min = angles.iter().copied().fold(f64::INFINITY, f64::min);
    let max = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    (min, max)
}

fn triangle_area(a: Vec3, b: Vec3, c: Vec3) -> f64 {
    0.5 * (b - a).cross(&(c - a)).norm()
}

fn inradius(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    let v = signed_volume(p0, p1, p2, p3).abs();
    let a = triangle_area(p1, p2, p3)
        + triangle_area(p0, p2, p3)
        + triangle_area(p0, p1, p3)
        + triangle_area(p0, p1, p2);
    3.0 * v / a
}

fn circumradius(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    let v1 = p1 - p0;
    let v2 = p2 - p0;
    let v3 = p3 - p0;
    let denom = 2.0 * v1.dot(&v2.cross(&v3));
    let num = v1.norm_squared() * v2.cross(&v3)
        + v2.norm_squared() * v3.cross(&v1)
        + v3.norm_squared() * v1.cross(&v2);
    (num / denom).norm()
}

fn dihedral_angle(edge_a: Vec3, edge_b: Vec3, opp1: Vec3, opp2: Vec3) -> f64 {
    let e = (edge_b - edge_a).normalize();
    let u = opp1 - edge_a;
    let v = opp2 - edge_a;
    let u_perp = u - e * u.dot(&e);
    let v_perp = v - e * v.dot(&e);
    let cos_theta = u_perp.dot(&v_perp) / (u_perp.norm() * v_perp.norm());
    // Clamp into [-1, 1] before acos: FP rounding of normalised dot
    // products can overshoot the unit interval by O(eps), which would
    // surface as NaN downstream.
    cos_theta.clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn regular_tet() -> [Vec3; 4] {
        // Unit-edge regular tet with one vertex at origin and one
        // edge along +x. Apex height = sqrt(2/3); base centroid is
        // at (1/2, sqrt(3)/6, 0). All six edges have length 1.
        let h = (2.0_f64 / 3.0).sqrt();
        let s3 = 3.0_f64.sqrt();
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.5, s3 / 2.0, 0.0),
            Vec3::new(0.5, s3 / 6.0, h),
        ]
    }

    fn canonical_decimeter_tet() -> [Vec3; 4] {
        // Corner tet: three perpendicular edges of length L = 0.1
        // from the origin.
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.1, 0.0, 0.0),
            Vec3::new(0.0, 0.1, 0.0),
            Vec3::new(0.0, 0.0, 0.1),
        ]
    }

    #[test]
    fn signed_volume_regular_tet_unit_edge() {
        // V = sqrt(2)/12 for a regular tet of edge 1.
        let [p0, p1, p2, p3] = regular_tet();
        assert_relative_eq!(
            signed_volume(p0, p1, p2, p3),
            2.0_f64.sqrt() / 12.0,
            epsilon = 1e-12,
        );
    }

    #[test]
    fn signed_volume_canonical_decimeter() {
        // Corner tet of edge L: V = L^3 / 6.
        let [p0, p1, p2, p3] = canonical_decimeter_tet();
        assert_relative_eq!(
            signed_volume(p0, p1, p2, p3),
            0.1_f64.powi(3) / 6.0,
            epsilon = 1e-15,
        );
    }

    #[test]
    fn signed_volume_negative_under_left_handed_orientation() {
        // Swap p2 and p3 — swaps two columns of the determinant matrix,
        // flipping the sign while preserving magnitude.
        let [p0, p1, p2, p3] = canonical_decimeter_tet();
        let right_handed = signed_volume(p0, p1, p2, p3);
        let left_handed = signed_volume(p0, p1, p3, p2);
        assert_relative_eq!(left_handed, -right_handed, epsilon = 1e-15);
        assert!(left_handed < 0.0);
    }

    #[test]
    fn aspect_ratio_regular_tet() {
        // r_ins = sqrt(6)/12, r_circ = sqrt(6)/4 → ratio = 1/3,
        // the theoretical maximum for any tet.
        let [p0, p1, p2, p3] = regular_tet();
        assert_relative_eq!(aspect_ratio(p0, p1, p2, p3), 1.0 / 3.0, epsilon = 1e-12,);
    }

    #[test]
    fn aspect_ratio_canonical_decimeter() {
        // Corner tet: r_ins = L/(sqrt(3) + 3), r_circ = L*sqrt(3)/2.
        // Ratio simplifies to (sqrt(3) - 1)/3 ≈ 0.244 — the same
        // shape class as a 6-tet Kuhn cube decomposition.
        let [p0, p1, p2, p3] = canonical_decimeter_tet();
        let expected = (3.0_f64.sqrt() - 1.0) / 3.0;
        assert_relative_eq!(aspect_ratio(p0, p1, p2, p3), expected, epsilon = 1e-12,);
    }

    #[test]
    fn dihedral_min_max_regular_tet() {
        // All six dihedrals are arccos(1/3) ≈ 70.5288° for a regular tet.
        let [p0, p1, p2, p3] = regular_tet();
        let (min, max) = dihedral_min_max(p0, p1, p2, p3);
        let expected = (1.0_f64 / 3.0).acos();
        assert_relative_eq!(min, expected, epsilon = 1e-12);
        assert_relative_eq!(max, expected, epsilon = 1e-12);
    }

    #[test]
    fn dihedral_min_max_canonical_decimeter() {
        // Three dihedrals at the orthogonal vertex are π/2 (the three
        // pairs of mutually perpendicular faces meeting at the corner).
        // Three dihedrals on the opposite (equilateral) face are
        // arccos(1/sqrt(3)) ≈ 54.7356°.
        let [p0, p1, p2, p3] = canonical_decimeter_tet();
        let (min, max) = dihedral_min_max(p0, p1, p2, p3);
        let expected_min = (1.0_f64 / 3.0_f64.sqrt()).acos();
        let expected_max = std::f64::consts::FRAC_PI_2;
        assert_relative_eq!(min, expected_min, epsilon = 1e-12);
        assert_relative_eq!(max, expected_max, epsilon = 1e-12);
    }
}
