//! Mass pipeline processing.
//!
//! Computes body inertial properties from geoms (when no explicit `<inertial>`
//! is provided), extracts explicit inertial specifications, and applies the
//! post-processing mass pipeline: each body's boundmass and boundinertia, a
//! value still negative, the moments' triangle (balanced or recorded); then,
//! after every check of the masses, settotalmass.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::mesh::TriangleMeshData;
use std::collections::HashMap;
use std::sync::Arc;

use super::ModelBuilder;
use super::eig3::full_inertia;
use super::geom::{compute_geom_inertia, compute_geom_mass, geom_effective_com};
use super::mesh::{MeshProps, compute_mesh_inertia_by_mode, resolve_mesh};
use super::orientation::quat_from_wxyz;
use crate::error::{MjcfError, Result};
use crate::types::{MeshInertia, MjcfGeom, MjcfInertial};

/// MuJoCo's `mjMINVAL` (`mjtnum.h:26`).
pub const MJ_MINVAL: f64 = 1e-15;

impl ModelBuilder {
    /// Apply the mass post-processing pipeline: each body's `boundmass` and
    /// `boundinertia`, then a mass or moment still negative, then its
    /// principal moments' triangle, A + B >= C, made the moments' mean under
    /// `balanceinertia`; each fault recorded for [`Self::check_bodies`] to
    /// refuse, as MuJoCo 3.5.0's `mjCBody::Compile` does
    /// (`user_objects.cc:2448-2469`).
    pub(crate) fn apply_mass_pipeline(&mut self) {
        let nbody = self.body_mass.len();

        for i in 1..nbody {
            if self.body_mass[i] < self.compiler.boundmass {
                self.body_mass[i] = self.compiler.boundmass;
            }
            let inertia = &mut self.body_inertia[i];
            for moment in inertia.iter_mut() {
                if *moment < self.compiler.boundinertia {
                    *moment = self.compiler.boundinertia;
                }
            }
            if self.body_mass[i] < 0.0 || inertia.iter().any(|&moment| moment < 0.0) {
                self.body_faults[i].negative = true;
            }
            let (a, b, c) = (inertia.x, inertia.y, inertia.z);
            if a + b < c || a + c < b || b + c < a {
                if self.compiler.balanceinertia {
                    let mean = (a + b + c) / 3.0;
                    *inertia = Vector3::new(mean, mean, mean);
                } else {
                    self.body_faults[i].triangle = true;
                }
            }
        }
    }

    /// `settotalmass`, as MuJoCo 3.5.0's `mj_setTotalmass` applies it once
    /// the model is compiled (`engine_support.c:748-761`, called at
    /// `user_model.cc:5040-5043`): every body's mass and moments, a
    /// flexcomp's vertex bodies' too, times `max(mjMINVAL, new /
    /// max(mjMINVAL, total))`. The caller runs it after every check of the
    /// bodies' masses.
    pub(crate) fn apply_settotalmass(&mut self) {
        if self.compiler.settotalmass > 0.0 {
            let total: f64 = self.body_mass[1..].iter().sum();
            let scale = MJ_MINVAL.max(self.compiler.settotalmass / MJ_MINVAL.max(total));
            for i in 1..self.body_mass.len() {
                self.body_mass[i] *= scale;
                self.body_inertia[i] *= scale;
            }
        }
    }
}

/// Extract inertial properties from MjcfInertial with full MuJoCo semantics.
///
/// Handles both `diaginertia` and `fullinertia` specifications. A
/// `fullinertia` is made principal as MuJoCo's compiler makes it
/// (`mjuu_fullInertia`, `user_objects.cc:2413-2417`): its moments in
/// decreasing order and their frame in `iquat`, refused where the smallest
/// is below mjEPS, or where they or their frame are not finite.
pub fn extract_inertial_properties(
    inertial: &MjcfInertial,
) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)> {
    let mass = inertial.mass;
    let ipos = inertial.pos;

    // Priority: fullinertia > diaginertia > default
    if let Some(full) = inertial.fullinertia {
        let (moments, frame) = principal_full_inertia(&full)?;
        if !moments.iter().chain(frame.iter()).all(|x| x.is_finite()) {
            return Err(MjcfError::invalid_value(
                "inertial",
                "fullinertia",
                "its principal moments or axes are not finite",
            ));
        }
        // The principal frame is the inertial frame: MuJoCo's
        // `mjuu_fullInertia` overwrites `iquat` (`user_objects.cc:2412-2416`),
        // and the parser refuses an orientation written beside a
        // `fullinertia`, as MuJoCo's reader does.
        let iquat =
            UnitQuaternion::new_unchecked(Quaternion::new(frame[0], frame[1], frame[2], frame[3]));
        Ok((mass, Vector3::from(moments), ipos, iquat))
    } else if let Some(diag) = inertial.diaginertia {
        // Diagonal inertia already in principal axes
        Ok((mass, diag, ipos, quat_from_wxyz(inertial.quat)))
    } else {
        // MuJoCo's default inertia is 0 (`mjs_defaultBody`, `user_init.c:75-76`).
        Ok((mass, Vector3::zeros(), ipos, quat_from_wxyz(inertial.quat)))
    }
}

/// A `fullinertia` made principal (`full_inertia`), refused with MuJoCo's
/// text where its smallest moment is below mjEPS.
fn principal_full_inertia(full: &[f64; 6]) -> Result<([f64; 3], [f64; 4])> {
    full_inertia(full).map_err(|reason| MjcfError::invalid_value("inertial", "fullinertia", reason))
}

/// MuJoCo's refusal of a `fullinertia` that `<compiler
/// inertiafromgeom="true">` replaces with the geoms' inertia: MuJoCo makes
/// every fullinertia principal before that (`user_objects.cc:2413-2418`), so
/// its refusal stands, and what it makes, a non-finite one included, is
/// replaced.
pub fn check_replaced_full_inertia(inertial: Option<&MjcfInertial>) -> Result<()> {
    if let Some(full) = inertial.and_then(|i| i.fullinertia) {
        principal_full_inertia(&full)?;
    }
    Ok(())
}

/// A mesh geom's mesh properties, by its mesh's inertia mode; `None` for
/// any other geom.
pub fn geom_mesh_props(
    geom: &MjcfGeom,
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
    mesh_inertia_modes: &[MeshInertia],
) -> Option<MeshProps> {
    resolve_mesh(geom, mesh_lookup, mesh_data).map(|mesh| {
        let mesh_id = geom
            .mesh
            .as_ref()
            .and_then(|name| mesh_lookup.get(name))
            .copied()
            .unwrap_or(0);
        let mode = mesh_inertia_modes
            .get(mesh_id)
            .copied()
            .unwrap_or(MeshInertia::Convex);
        compute_mesh_inertia_by_mode(&mesh, mode)
    })
}

/// A height-field geom's asset's four sizes; `None` for any other geom.
pub fn geom_hfield_size<'a>(
    geom: &MjcfGeom,
    hfield_lookup: &HashMap<String, usize>,
    hfield_size: &'a [[f64; 4]],
) -> Option<&'a [f64; 4]> {
    let id = geom
        .hfield
        .as_ref()
        .and_then(|name| hfield_lookup.get(name))?;
    hfield_size.get(*id)
}

/// Compute inertia from geoms (fallback when no explicit inertial).
///
/// Accumulates full 3×3 inertia tensor with geom orientation handling,
/// then eigendecomposes to extract principal inertia and orientation.
/// Refuses a body whose mass, inertia or principal moments and axes are not
/// finite, checked on the tensor and on its decomposition before the frame
/// is fitted: a geom's size or place can overflow them where MuJoCo loads
/// the body (registry row D-INERTIA-NONFINITE).
///
/// Mesh inertia is computed once per geom and cached across the mass and
/// inertia passes (avoids O(3n) calls to `compute_mesh_inertia`); a height
/// field geom's mass and inertia read its asset's sizes (`hfield_size`, by
/// `hfield_lookup`).
pub fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
    mesh_inertia_modes: &[MeshInertia],
    hfield_lookup: &HashMap<String, usize>,
    hfield_size: &[[f64; 4]],
) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)> {
    if geoms.is_empty() {
        // No geoms: zero mass/inertia (matches MuJoCo).
        // Use boundmass/boundinertia to set minimums instead.
        return Ok((
            0.0,
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        ));
    }

    // Each geom's `fromto` is already its pos, quat and size (`resolve_geom_fromto`,
    // once its class is applied: `resolve::apply_defaults`).

    // Pre-compute mesh inertia once per geom (avoids 3× recomputation).
    let mesh_props: Vec<Option<MeshProps>> = geoms
        .iter()
        .map(|geom| geom_mesh_props(geom, mesh_lookup, mesh_data, mesh_inertia_modes))
        .collect();
    let hfields: Vec<Option<&[f64; 4]>> = geoms
        .iter()
        .map(|geom| geom_hfield_size(geom, hfield_lookup, hfield_size))
        .collect();

    let mut total_mass = 0.0;
    let mut com = Vector3::zeros();

    // First pass: compute total mass and COM
    // For mesh geoms, the effective center of mass in the body frame is
    // `geom.pos + R * mesh_com` (mesh COM is in mesh-local coordinates).
    // For primitive geoms, the local COM is at the geom origin: `geom.pos`.
    for ((geom, props), &hfield) in geoms.iter().zip(mesh_props.iter()).zip(&hfields) {
        let geom_mass = compute_geom_mass(geom, props.as_ref(), hfield);
        let effective_pos = geom_effective_com(geom, props.as_ref());
        total_mass += geom_mass;
        com += effective_pos * geom_mass;
    }

    if total_mass > 1e-10 {
        com /= total_mass;
    }

    // Second pass: accumulate full 3×3 inertia tensor about COM
    let mut inertia_tensor = Matrix3::zeros();
    for ((geom, props), &hfield) in geoms.iter().zip(mesh_props.iter()).zip(&hfields) {
        let geom_mass = compute_geom_mass(geom, props.as_ref(), hfield);
        let geom_inertia = compute_geom_inertia(geom, props.as_ref(), hfield);

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

    let not_finite = || {
        MjcfError::invalid_model("the body's mass or inertia computed from its geoms is not finite")
    };
    if !total_mass.is_finite() || !inertia_tensor.iter().all(|x| x.is_finite()) {
        return Err(not_finite());
    }

    // Eigendecompose to get principal axes. MuJoCo copies a single geom's
    // frame and moments, and finds several geoms' with `eig3`
    // (`InertiaFromGeom`, `user_objects.cc:2157-2227`): Rigid-loading L33's.
    // A finite tensor's moments can overflow (two geoms far apart).
    let eigen = inertia_tensor.symmetric_eigen();
    if !eigen
        .eigenvalues
        .iter()
        .chain(eigen.eigenvectors.iter())
        .all(|x| x.is_finite())
    {
        return Err(not_finite());
    }
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

    Ok((total_mass, principal_inertia, com, iquat))
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
                    <body name="b" pos="0 0 0"/>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");
        // Auto mode with no geoms and no inertial -> zero mass (a static
        // body: MuJoCo refuses a moving one this light).
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

    /// A full inertia's principal frame is the inertial frame whatever
    /// `quat` an `MjcfInertial` built in code carries, as MuJoCo's spec
    /// overwrites `iquat` with it (`user_objects.cc:2412-2416`).
    #[test]
    fn a_full_inertias_frame_replaces_a_quat_built_in_code() {
        let full = crate::types::MjcfInertial {
            mass: 1.0,
            fullinertia: Some([3.0, 2.0, 1.0, 0.1, 0.0, 0.0]),
            ..Default::default()
        };
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let turned = crate::types::MjcfInertial {
            quat: nalgebra::Vector4::new(half, 0.0, 0.0, half),
            ..full
        };
        let (_, _, _, plain) = super::extract_inertial_properties(&full).expect("principal");
        let (_, _, _, kept) = super::extract_inertial_properties(&turned).expect("principal");
        assert_eq!(plain, kept);
    }
}
