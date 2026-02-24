//! Mass pipeline processing.
//!
//! Computes body inertial properties from geoms (when no explicit `<inertial>`
//! is provided), extracts explicit inertial specifications, and applies the
//! post-processing mass pipeline: balanceinertia, boundmass/boundinertia,
//! settotalmass.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::mesh::TriangleMeshData;
use std::collections::HashMap;
use std::sync::Arc;

use super::ModelBuilder;
use super::geom::{compute_geom_inertia, compute_geom_mass, geom_effective_com};
use super::mesh::{MeshProps, compute_mesh_inertia, resolve_mesh};
use super::orientation::quat_from_wxyz;
use crate::types::{MjcfGeom, MjcfInertial};

impl ModelBuilder {
    /// Apply the mass post-processing pipeline (MuJoCo §A6-A8).
    ///
    /// Steps (in order):
    ///   1. inertiafromgeom — already handled during `process_body_with_world_frame`
    ///   2. balanceinertia — triangle inequality correction
    ///   3. boundmass / boundinertia — minimum clamping
    ///   4. settotalmass — rescale to target total mass
    pub(crate) fn apply_mass_pipeline(&mut self) {
        let nbody = self.body_mass.len();

        // Step 2: balanceinertia (A7)
        // For each non-world body, check the triangle inequality on diagonal inertia.
        // If violated (A + B < C for any permutation), set all three to their mean.
        if self.compiler.balanceinertia {
            for i in 1..nbody {
                let inertia = &self.body_inertia[i];
                let a = inertia.x;
                let b = inertia.y;
                let c = inertia.z;
                if a + b < c || a + c < b || b + c < a {
                    let mean = (a + b + c) / 3.0;
                    self.body_inertia[i] = Vector3::new(mean, mean, mean);
                }
            }
        }

        // Step 3: boundmass / boundinertia (A6)
        // Clamp every non-world body's mass and inertia to compiler minimums.
        if self.compiler.boundmass > 0.0 {
            for i in 1..nbody {
                if self.body_mass[i] < self.compiler.boundmass {
                    self.body_mass[i] = self.compiler.boundmass;
                }
            }
        }
        if self.compiler.boundinertia > 0.0 {
            for i in 1..nbody {
                let inertia = &mut self.body_inertia[i];
                inertia.x = inertia.x.max(self.compiler.boundinertia);
                inertia.y = inertia.y.max(self.compiler.boundinertia);
                inertia.z = inertia.z.max(self.compiler.boundinertia);
            }
        }

        // Step 4: settotalmass (A8)
        // When positive, rescale all body masses and inertias so total == target.
        if self.compiler.settotalmass > 0.0 {
            let total_mass: f64 = (1..nbody).map(|i| self.body_mass[i]).sum();
            if total_mass > 0.0 {
                let scale = self.compiler.settotalmass / total_mass;
                for i in 1..nbody {
                    self.body_mass[i] *= scale;
                    self.body_inertia[i] *= scale;
                }
            }
        }
    }
}

/// Extract inertial properties from MjcfInertial with full MuJoCo semantics.
///
/// Handles both `diaginertia` and `fullinertia` specifications.
/// When `fullinertia` is specified, diagonalizes via eigendecomposition
/// and returns the principal axis orientation in `iquat`.
pub fn extract_inertial_properties(
    inertial: &MjcfInertial,
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    let mass = inertial.mass;
    let ipos = inertial.pos;

    // Priority: fullinertia > diaginertia > default
    if let Some(full) = inertial.fullinertia {
        // Full inertia tensor [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        // Build symmetric matrix and diagonalize
        let inertia_matrix = Matrix3::new(
            full[0], full[3], full[4], // Ixx, Ixy, Ixz
            full[3], full[1], full[5], // Ixy, Iyy, Iyz
            full[4], full[5], full[2], // Ixz, Iyz, Izz
        );

        // Eigendecomposition to get principal axes
        let eigen = inertia_matrix.symmetric_eigen();
        let principal_inertia = Vector3::new(
            eigen.eigenvalues[0].abs(),
            eigen.eigenvalues[1].abs(),
            eigen.eigenvalues[2].abs(),
        );

        // Eigenvectors form rotation matrix to principal axes
        // Ensure right-handed coordinate system
        let mut rot = eigen.eigenvectors;
        if rot.determinant() < 0.0 {
            // Flip one column to make it right-handed
            rot.set_column(2, &(-rot.column(2)));
        }

        let iquat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&rot));

        // Combine with any existing inertial frame orientation
        let base_iquat = quat_from_wxyz(inertial.quat);
        let final_iquat = base_iquat * iquat;

        (mass, principal_inertia, ipos, final_iquat)
    } else if let Some(diag) = inertial.diaginertia {
        // Diagonal inertia already in principal axes
        (mass, diag, ipos, quat_from_wxyz(inertial.quat))
    } else {
        // Default: small uniform inertia
        (
            mass,
            Vector3::new(0.001, 0.001, 0.001),
            ipos,
            quat_from_wxyz(inertial.quat),
        )
    }
}

/// Compute inertia from geoms (fallback when no explicit inertial).
///
/// Accumulates full 3×3 inertia tensor with geom orientation handling,
/// then eigendecomposes to extract principal inertia and orientation.
///
/// Mesh inertia is computed once per geom and cached across the mass and
/// inertia passes (avoids O(3n) calls to `compute_mesh_inertia`).
pub fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    if geoms.is_empty() {
        // No geoms: zero mass/inertia (matches MuJoCo).
        // Use boundmass/boundinertia to set minimums instead.
        return (
            0.0,
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
    }

    // Pre-compute mesh inertia once per geom (avoids 3× recomputation).
    let mesh_props: Vec<Option<MeshProps>> = geoms
        .iter()
        .map(|geom| {
            resolve_mesh(geom, mesh_lookup, mesh_data).map(|mesh| compute_mesh_inertia(&mesh))
        })
        .collect();

    let mut total_mass = 0.0;
    let mut com = Vector3::zeros();

    // First pass: compute total mass and COM
    // For mesh geoms, the effective center of mass in the body frame is
    // `geom.pos + R * mesh_com` (mesh COM is in mesh-local coordinates).
    // For primitive geoms, the local COM is at the geom origin: `geom.pos`.
    for (geom, props) in geoms.iter().zip(mesh_props.iter()) {
        let geom_mass = compute_geom_mass(geom, props.as_ref());
        let effective_pos = geom_effective_com(geom, props.as_ref());
        total_mass += geom_mass;
        com += effective_pos * geom_mass;
    }

    if total_mass > 1e-10 {
        com /= total_mass;
    }

    // Second pass: accumulate full 3×3 inertia tensor about COM
    let mut inertia_tensor = Matrix3::zeros();
    for (geom, props) in geoms.iter().zip(mesh_props.iter()) {
        let geom_mass = compute_geom_mass(geom, props.as_ref());
        let geom_inertia = compute_geom_inertia(geom, props.as_ref());

        // 1. Rotate local inertia to body frame: I_rot = R * I_local * Rᵀ
        let q = geom.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0));
        let r = UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3]));
        let rot = r.to_rotation_matrix();
        let i_rotated = rot * geom_inertia * rot.transpose();

        // 2. Parallel axis theorem (full tensor):
        //    I_shifted = I_rotated + m * (d·d * I₃ - d ⊗ d)
        //    d = displacement from body COM to geom effective COM
        let effective_pos = geom_effective_com(geom, props.as_ref());
        let d = effective_pos - com;
        let d_sq = d.dot(&d);
        let parallel_axis = geom_mass * (Matrix3::identity() * d_sq - d * d.transpose());
        inertia_tensor += i_rotated + parallel_axis;
    }

    // Eigendecompose to get principal axes
    let eigen = inertia_tensor.symmetric_eigen();
    let principal_inertia = Vector3::new(
        eigen.eigenvalues[0].abs(),
        eigen.eigenvalues[1].abs(),
        eigen.eigenvalues[2].abs(),
    );

    // Eigenvectors form rotation to principal axes
    // Ensure right-handed coordinate system
    let mut rot = eigen.eigenvectors;
    if rot.determinant() < 0.0 {
        rot.set_column(2, &(-rot.column(2)));
    }
    let iquat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&rot));

    (total_mass, principal_inertia, com, iquat)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;

    // -- Mass pipeline tests --

    #[test]
    fn test_inertiafromgeom_true_overrides_explicit_inertial() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="999.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // inertiafromgeom="true" -> geom mass (2.0) overrides explicit 999.0
        assert!(
            (model.body_mass[1] - 2.0).abs() < 1e-10,
            "mass should come from geom, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_uses_explicit_only() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="5.0" diaginertia="1 1 1"/>
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 5.0).abs() < 1e-10,
            "mass should come from explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_false_no_inertial_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" inertiafromgeom="false"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "mass should be zero without explicit inertial, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_inertiafromgeom_auto_no_geoms_gives_zero() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <joint type="hinge" axis="0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Auto mode with no geoms and no inertial -> zero mass
        assert!(
            model.body_mass[1].abs() < 1e-10,
            "empty body should have zero mass, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundmass_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="0.5"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        assert!(
            (model.body_mass[1] - 0.5).abs() < 1e-10,
            "mass should be clamped to 0.5, got {}",
            model.body_mass[1]
        );
    }

    #[test]
    fn test_boundinertia_clamps_minimum() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundinertia="0.01"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.01).abs() < 1e-10
                && (inertia.y - 0.01).abs() < 1e-10
                && (inertia.z - 0.01).abs() < 1e-10,
            "inertia should be clamped to 0.01, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_fixes_triangle_inequality() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // 0.1 + 0.1 < 0.5 violates triangle inequality -> mean = (0.1+0.1+0.5)/3
        let expected = (0.1 + 0.1 + 0.5) / 3.0;
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - expected).abs() < 1e-10
                && (inertia.y - expected).abs() < 1e-10
                && (inertia.z - expected).abs() < 1e-10,
            "inertia should be balanced to mean {expected}, got {inertia:?}"
        );
    }

    #[test]
    fn test_balanceinertia_no_change_when_valid() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" balanceinertia="true"/>
                <worldbody>
                    <body name="b" pos="0 0 0">
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.3 0.3 0.3"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        let inertia = model.body_inertia[1];
        assert!(
            (inertia.x - 0.3).abs() < 1e-10
                && (inertia.y - 0.3).abs() < 1e-10
                && (inertia.z - 0.3).abs() < 1e-10,
            "valid inertia should not change, got {inertia:?}"
        );
    }

    #[test]
    fn test_settotalmass_rescales() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="10.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 3 + 7 = 10, settotalmass = 10 -> no change
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 10.0).abs() < 1e-10,
            "total mass should be 10.0, got {total}"
        );
    }

    #[test]
    fn test_settotalmass_rescales_different_target() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" settotalmass="20.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.1" mass="3.0"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="7.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Original total = 10, target = 20 -> scale = 2x
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 20.0).abs() < 1e-10,
            "total mass should be 20.0, got {total}"
        );
        // Mass ratios should be preserved: a=6.0, b=14.0
        assert!(
            (model.body_mass[1] - 6.0).abs() < 1e-10,
            "body a mass should be 6.0, got {}",
            model.body_mass[1]
        );
        assert!(
            (model.body_mass[2] - 14.0).abs() < 1e-10,
            "body b mass should be 14.0, got {}",
            model.body_mass[2]
        );
    }

    #[test]
    fn test_mass_pipeline_order_bound_then_settotalmass() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" boundmass="1.0" settotalmass="5.0"/>
                <worldbody>
                    <body name="a" pos="0 0 0">
                        <geom type="sphere" size="0.01" mass="0.1"/>
                    </body>
                    <body name="b" pos="1 0 0">
                        <geom type="sphere" size="0.1" mass="2.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // After boundmass: a=1.0, b=2.0, total=3.0
        // After settotalmass(5.0): scale=5/3, a=5/3, b=10/3
        let total: f64 = (1..model.nbody).map(|i| model.body_mass[i]).sum();
        assert!(
            (total - 5.0).abs() < 1e-10,
            "total mass should be 5.0, got {total}"
        );
    }
}
