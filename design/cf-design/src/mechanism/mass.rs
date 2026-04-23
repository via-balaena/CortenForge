//! Volumetric mass property computation from implicit fields.
//!
//! Computes mass, center of mass, and inertia tensor by integrating over the
//! interior of a [`Solid`] on a regular grid. Interior is defined
//! as the region where the field evaluates to ≤ 0.
//!
//! # Units
//!
//! cf-design geometry is in **millimeters** (natural for 3D printing), but
//! material density is in **kg/m³** (natural for physics/engineering). These
//! two unit systems meet here. The conversion factor:
//!
//! ```text
//! 1 mm³ = 1e-9 m³
//! dm = density_kg_per_m3 * cell_volume_mm3 * MM3_TO_M3
//! ```
//!
//! Output units:
//! - **mass**: kg
//! - **`center_of_mass`**: mm (same frame as geometry)
//! - **inertia**: kg·mm² (about center of mass)

use nalgebra::Point3;

use crate::Solid;

/// 1 mm³ = 1e-9 m³.
///
/// Geometry is in mm, density is in kg/m³. This factor bridges the two.
const MM3_TO_M3: f64 = 1e-9;

/// Mass properties of a solid body: mass, center of mass, and inertia tensor.
///
/// Computed via grid integration over the implicit field. Accuracy depends on
/// `cell_size` — finer grids yield more accurate results at higher cost.
///
/// # Units
///
/// - `mass`: kg
/// - `center_of_mass`: mm (part-local frame, same as geometry)
/// - `inertia`: \[Ixx, Iyy, Izz, Ixy, Ixz, Iyz\] in kg·mm², about center of mass
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    /// Total mass (kg).
    pub mass: f64,
    /// Center of mass in part-local frame (mm).
    pub center_of_mass: Point3<f64>,
    /// Inertia tensor about the center of mass: \[Ixx, Iyy, Izz, Ixy, Ixz, Iyz\].
    ///
    /// Symmetric 3×3 matrix stored as 6 independent components (kg·mm²).
    /// Off-diagonal terms use physics sign convention (Ixy = -Σ dm·x·y).
    pub inertia: [f64; 6],
}

/// Compute mass properties of a solid via grid integration.
///
/// Evaluates the implicit field on a regular grid with spacing `cell_size`.
/// Boundary voxels are weighted by a linear fraction for O(h²) convergence.
///
/// Returns `None` if the solid has no finite bounds (e.g., an infinite plane)
/// or if the interior is empty at the given resolution.
///
/// # Units
///
/// - `density`: kg/m³ (e.g., PLA = 1250)
/// - `cell_size`: mm (grid spacing — smaller = more accurate, slower)
/// - Result: mass in kg, center of mass in mm, inertia in kg·mm²
///
/// # Panics
///
/// Panics if `density` is not positive/finite or `cell_size` is not positive/finite.
#[must_use]
pub fn mass_properties(solid: &Solid, density: f64, cell_size: f64) -> Option<MassProperties> {
    assert!(
        density > 0.0 && density.is_finite(),
        "density must be positive and finite, got {density}"
    );
    assert!(
        cell_size > 0.0 && cell_size.is_finite(),
        "cell_size must be positive and finite, got {cell_size}"
    );

    let bounds = solid.bounds()?;

    // Expand bounds by half a cell to catch surface voxels.
    let expanded = bounds.expanded(cell_size * 0.5);
    let grid_size = expanded.size();

    // Index/count conversion bounded by domain (size well below 2^32).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let grid_dims = [
        (grid_size.x / cell_size).ceil() as usize,
        (grid_size.y / cell_size).ceil() as usize,
        (grid_size.z / cell_size).ceil() as usize,
    ];

    if grid_dims[0] == 0 || grid_dims[1] == 0 || grid_dims[2] == 0 {
        return None;
    }

    let cell_vol_mm3 = cell_size * cell_size * cell_size;
    // dm for each interior voxel: density (kg/m³) × cell volume (mm³) × (mm³→m³)
    let dm = density * cell_vol_mm3 * MM3_TO_M3;

    let accum = integrate_grid(solid, &expanded.min, cell_size, &grid_dims, dm);

    if accum.mass <= 0.0 {
        return None;
    }

    Some(accum.finish())
}

// ── Grid integration ────────────────────────────────────────────────────

/// Accumulated mass moments from grid integration.
struct MassAccumulator {
    mass: f64,
    // First moments (Σ w·coord).
    moment_x: f64,
    moment_y: f64,
    moment_z: f64,
    // Second moments about origin (Σ w·coord²).
    second_xx: f64,
    second_yy: f64,
    second_zz: f64,
    second_xy: f64,
    second_xz: f64,
    second_yz: f64,
}

impl MassAccumulator {
    const fn new() -> Self {
        Self {
            mass: 0.0,
            moment_x: 0.0,
            moment_y: 0.0,
            moment_z: 0.0,
            second_xx: 0.0,
            second_yy: 0.0,
            second_zz: 0.0,
            second_xy: 0.0,
            second_xz: 0.0,
            second_yz: 0.0,
        }
    }

    /// Add a weighted voxel at position (px, py, pz).
    fn add(&mut self, weighted_dm: f64, px: f64, py: f64, pz: f64) {
        self.mass += weighted_dm;
        self.moment_x = weighted_dm.mul_add(px, self.moment_x);
        self.moment_y = weighted_dm.mul_add(py, self.moment_y);
        self.moment_z = weighted_dm.mul_add(pz, self.moment_z);
        self.second_xx = (weighted_dm * px).mul_add(px, self.second_xx);
        self.second_yy = (weighted_dm * py).mul_add(py, self.second_yy);
        self.second_zz = (weighted_dm * pz).mul_add(pz, self.second_zz);
        self.second_xy = (weighted_dm * px).mul_add(py, self.second_xy);
        self.second_xz = (weighted_dm * px).mul_add(pz, self.second_xz);
        self.second_yz = (weighted_dm * py).mul_add(pz, self.second_yz);
    }

    /// Compute final mass properties from accumulated moments.
    // Domain notation preserves geometric conventions.
    #[allow(clippy::similar_names)]
    fn finish(self) -> MassProperties {
        let com_x = self.moment_x / self.mass;
        let com_y = self.moment_y / self.mass;
        let com_z = self.moment_z / self.mass;

        // Shift second moments to center of mass via parallel axis theorem:
        //   Σ dm·(x - cx)² = Σ dm·x² - M·cx²
        let shifted_xx = (self.mass * com_x).mul_add(-com_x, self.second_xx);
        let shifted_yy = (self.mass * com_y).mul_add(-com_y, self.second_yy);
        let shifted_zz = (self.mass * com_z).mul_add(-com_z, self.second_zz);
        let shifted_xy = (self.mass * com_x).mul_add(-com_y, self.second_xy);
        let shifted_xz = (self.mass * com_x).mul_add(-com_z, self.second_xz);
        let shifted_yz = (self.mass * com_y).mul_add(-com_z, self.second_yz);

        // Inertia tensor about center of mass:
        //   Ixx = Σ dm·(y² + z²),  Ixy = -Σ dm·x·y, etc.
        MassProperties {
            mass: self.mass,
            center_of_mass: Point3::new(com_x, com_y, com_z),
            inertia: [
                shifted_yy + shifted_zz, // Ixx
                shifted_xx + shifted_zz, // Iyy
                shifted_xx + shifted_yy, // Izz
                -shifted_xy,             // Ixy
                -shifted_xz,             // Ixz
                -shifted_yz,             // Iyz
            ],
        }
    }
}

/// Integrate the implicit field over a regular grid.
///
/// Uses fractional boundary weighting for O(h²) convergence: voxels near
/// the surface are weighted by `clamp(0.5 − value / cell_size, 0, 1)` instead
/// of a binary inside/outside test. A cell centered exactly on the surface
/// (value = 0) gets weight 0.5 — correctly accounting for the half-volume inside.
fn integrate_grid(
    solid: &Solid,
    origin: &Point3<f64>,
    cell_size: f64,
    dims: &[usize; 3],
    dm: f64,
) -> MassAccumulator {
    let inv_cell = 1.0 / cell_size;
    let half_step = cell_size * 0.5;
    let mut accum = MassAccumulator::new();

    for iz in 0..dims[2] {
        // Precision loss acceptable for approximate / visualization values.
        #[allow(clippy::cast_precision_loss)]
        let pz = origin.z + (iz as f64).mul_add(cell_size, half_step);

        for iy in 0..dims[1] {
            // Precision loss acceptable for approximate / visualization values.
            #[allow(clippy::cast_precision_loss)]
            let py = origin.y + (iy as f64).mul_add(cell_size, half_step);

            for ix in 0..dims[0] {
                // Precision loss acceptable for approximate / visualization values.
                #[allow(clippy::cast_precision_loss)]
                let px = origin.x + (ix as f64).mul_add(cell_size, half_step);

                let value = solid.evaluate(&Point3::new(px, py, pz));
                let weight = (0.5 - value * inv_cell).clamp(0.0, 1.0);

                if weight > 0.0 {
                    accum.add(dm * weight, px, py, pz);
                }
            }
        }
    }

    accum
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use
)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::Vector3;

    use super::*;

    const PLA_DENSITY: f64 = 1250.0; // kg/m³

    // ── 1. Sphere mass ─────────────────────────────────────────────────

    #[test]
    fn sphere_mass_within_2_percent() {
        let radius = 5.0; // mm
        let solid = Solid::sphere(radius);
        let cell_size = 0.25; // fine grid for accuracy

        let props = mass_properties(&solid, PLA_DENSITY, cell_size).unwrap();

        // Analytic: V = (4/3)πr³ mm³, mass = density × V × 1e-9 kg
        let volume_mm3 = (4.0 / 3.0) * PI * radius.powi(3);
        let expected_mass = PLA_DENSITY * volume_mm3 * 1e-9;

        let error = (props.mass - expected_mass).abs() / expected_mass;
        assert!(
            error < 0.02,
            "sphere mass error {:.4}% (got {}, expected {})",
            error * 100.0,
            props.mass,
            expected_mass
        );
    }

    // ── 2. Sphere center of mass at origin ─────────────────────────────

    #[test]
    fn sphere_center_of_mass_near_origin() {
        let solid = Solid::sphere(5.0);
        let props = mass_properties(&solid, PLA_DENSITY, 0.5).unwrap();

        let com = props.center_of_mass;
        let dist = (com.x * com.x + com.y * com.y + com.z * com.z).sqrt();
        assert!(
            dist < 0.5,
            "sphere COM should be near origin, got distance {dist}"
        );
    }

    // ── 3. Sphere inertia tensor ───────────────────────────────────────

    #[test]
    fn sphere_inertia_within_2_percent() {
        let radius = 5.0;
        let solid = Solid::sphere(radius);
        let cell_size = 0.25;

        let props = mass_properties(&solid, PLA_DENSITY, cell_size).unwrap();

        // Analytic: I = (2/5) m r² for a solid sphere.
        // r in mm, mass in kg → I in kg·mm².
        let volume_mm3 = (4.0 / 3.0) * PI * radius.powi(3);
        let expected_mass = PLA_DENSITY * volume_mm3 * 1e-9;
        let expected_i = 0.4 * expected_mass * radius * radius;

        // Diagonal terms should all be equal (sphere is symmetric).
        for (idx, label) in [(0, "Ixx"), (1, "Iyy"), (2, "Izz")] {
            let error = (props.inertia[idx] - expected_i).abs() / expected_i;
            assert!(
                error < 0.02,
                "sphere {label} error {:.4}% (got {}, expected {})",
                error * 100.0,
                props.inertia[idx],
                expected_i
            );
        }

        // Off-diagonal terms should be near zero.
        for (idx, label) in [(3, "Ixy"), (4, "Ixz"), (5, "Iyz")] {
            assert!(
                props.inertia[idx].abs() < expected_i * 0.02,
                "sphere {label} should be near zero, got {}",
                props.inertia[idx]
            );
        }
    }

    // ── 4. Cuboid mass ─────────────────────────────────────────────────

    #[test]
    fn cuboid_mass_within_2_percent() {
        let half = Vector3::new(10.0, 5.0, 3.0); // mm half-extents
        let solid = Solid::cuboid(half);
        let cell_size = 0.4;

        let props = mass_properties(&solid, PLA_DENSITY, cell_size).unwrap();

        // Analytic: V = 8 × hx × hy × hz
        let volume_mm3 = 8.0 * half.x * half.y * half.z;
        let expected_mass = PLA_DENSITY * volume_mm3 * 1e-9;

        let error = (props.mass - expected_mass).abs() / expected_mass;
        assert!(
            error < 0.02,
            "cuboid mass error {:.4}% (got {}, expected {})",
            error * 100.0,
            props.mass,
            expected_mass
        );
    }

    // ── 5. Cuboid inertia tensor ───────────────────────────────────────

    #[test]
    fn cuboid_inertia_within_2_percent() {
        let half = Vector3::new(10.0, 5.0, 3.0);
        let solid = Solid::cuboid(half);
        let cell_size = 0.4;

        let props = mass_properties(&solid, PLA_DENSITY, cell_size).unwrap();

        // Analytic: Ixx = m/12 × ((2hy)² + (2hz)²), etc.
        let volume_mm3 = 8.0 * half.x * half.y * half.z;
        let expected_mass = PLA_DENSITY * volume_mm3 * 1e-9;
        let full_w = 2.0 * half.x;
        let full_h = 2.0 * half.y;
        let full_d = 2.0 * half.z;
        let expected_ixx = expected_mass / 12.0 * (full_h * full_h + full_d * full_d);
        let expected_iyy = expected_mass / 12.0 * (full_w * full_w + full_d * full_d);
        let expected_izz = expected_mass / 12.0 * (full_w * full_w + full_h * full_h);

        let cases = [
            (0, "Ixx", expected_ixx),
            (1, "Iyy", expected_iyy),
            (2, "Izz", expected_izz),
        ];
        for (idx, label, expected) in cases {
            let error = (props.inertia[idx] - expected).abs() / expected;
            assert!(
                error < 0.02,
                "cuboid {label} error {:.4}% (got {}, expected {})",
                error * 100.0,
                props.inertia[idx],
                expected
            );
        }

        // Off-diagonal terms should be near zero (symmetric, centered cuboid).
        let max_diag = expected_ixx.max(expected_iyy).max(expected_izz);
        for (idx, label) in [(3, "Ixy"), (4, "Ixz"), (5, "Iyz")] {
            assert!(
                props.inertia[idx].abs() < max_diag * 0.02,
                "cuboid {label} should be near zero, got {}",
                props.inertia[idx]
            );
        }
    }

    // ── 6. Returns None for unbounded solid ────────────────────────────

    #[test]
    fn unbounded_solid_returns_none() {
        // A half-space (plane) has no finite bounds.
        let solid = Solid::plane(Vector3::z(), 0.0);
        let result = mass_properties(&solid, PLA_DENSITY, 1.0);
        assert!(result.is_none(), "expected None for unbounded solid");
    }

    // ── 7. Translated solid shifts center of mass ──────────────────────

    #[test]
    fn translated_solid_shifts_com() {
        let offset = Vector3::new(10.0, 20.0, 30.0);
        let solid = Solid::sphere(5.0).translate(offset);
        let props = mass_properties(&solid, PLA_DENSITY, 0.5).unwrap();

        let diff = props.center_of_mass - Point3::from(offset);
        let dist = (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z).sqrt();
        assert!(
            dist < 1.0,
            "COM should be near ({}, {}, {}), got {:?} (dist={dist})",
            offset.x,
            offset.y,
            offset.z,
            props.center_of_mass
        );
    }

    // ── 8. Shell mass less than solid ───────────────────────────────────

    #[test]
    fn shell_mass_less_than_solid() {
        let solid_sphere = Solid::sphere(5.0);
        let hollow_sphere = Solid::sphere(5.0).shell(0.5);

        let props_solid = mass_properties(&solid_sphere, PLA_DENSITY, 0.4).unwrap();
        let props_hollow = mass_properties(&hollow_sphere, PLA_DENSITY, 0.4).unwrap();

        assert!(
            props_hollow.mass < props_solid.mass,
            "hollow sphere mass ({}) should be less than solid ({})",
            props_hollow.mass,
            props_solid.mass
        );
        // Shell should be a fraction of solid mass.
        // shell(0.5) on r=5 sphere → wall from r=4.5 to r=5.5 → ~60% of solid volume.
        assert!(
            props_hollow.mass < props_solid.mass * 0.8,
            "shell(0.5) of r=5 sphere should be well below solid mass"
        );
    }

    // ── 9. Asymmetric shape has offset COM ───────────────────────────

    #[test]
    fn asymmetric_com_offset() {
        // Union of sphere at origin + sphere at x=10 → COM should be near x=5
        let shape =
            Solid::sphere(3.0).union(Solid::sphere(3.0).translate(Vector3::new(10.0, 0.0, 0.0)));
        let props = mass_properties(&shape, PLA_DENSITY, 0.5).unwrap();

        assert!(
            (props.center_of_mass.x - 5.0).abs() < 1.0,
            "COM x should be near 5.0 (midpoint), got {:.2}",
            props.center_of_mass.x
        );
        assert!(
            props.center_of_mass.y.abs() < 1.0,
            "COM y should be near 0.0, got {:.2}",
            props.center_of_mass.y
        );
    }

    // ── 10. Composed shape mass (smooth union) ──────────────────────

    #[test]
    fn smooth_union_mass_between_one_and_two_spheres() {
        let r = 4.0;
        let single = Solid::sphere(r);
        let composed = Solid::sphere(r)
            .smooth_union(Solid::sphere(r).translate(Vector3::new(5.0, 0.0, 0.0)), 2.0);

        let mass_one = mass_properties(&single, PLA_DENSITY, 0.4).unwrap().mass;
        let mass_composed = mass_properties(&composed, PLA_DENSITY, 0.4).unwrap().mass;

        assert!(
            mass_composed > mass_one,
            "smooth union mass ({mass_composed}) should exceed single sphere ({mass_one})"
        );
        assert!(
            mass_composed < 2.0 * mass_one,
            "smooth union mass ({mass_composed}) should be less than two spheres ({})",
            2.0 * mass_one
        );
    }

    // ── 11. Panics on bad inputs ────────────────────────────────────────

    #[test]
    #[should_panic(expected = "density must be positive")]
    fn rejects_zero_density() {
        let _ = mass_properties(&Solid::sphere(1.0), 0.0, 1.0);
    }

    #[test]
    #[should_panic(expected = "cell_size must be positive")]
    fn rejects_zero_cell_size() {
        let _ = mass_properties(&Solid::sphere(1.0), 1000.0, 0.0);
    }
}
