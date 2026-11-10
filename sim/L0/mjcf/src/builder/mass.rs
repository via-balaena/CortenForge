//! Mass pipeline processing.
//!
//! Computes body inertial properties from geoms (when no explicit `<inertial>`
//! is provided), extracts explicit inertial specifications, and applies the
//! post-processing mass pipeline: each body's boundmass and boundinertia, a
//! value still negative, the moments' triangle (balanced or recorded); then,
//! after every check of the masses, settotalmass.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::collections::HashMap;

use super::ModelBuilder;
use super::eig3::full_inertia;
use super::geom::{compute_geom_inertia, compute_geom_mass};
use super::mesh::MeshProps;
use super::orientation::{MJ_EPS, mulquat, mulvecmat, quat2mat};
use crate::error::{MjcfError, Result};
use crate::types::{MjcfGeom, MjcfGeomType, MjcfInertial};

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
            // A pinned flexcomp point's stand-in: MuJoCo has no body there.
            if self.flex_stand_ins.binary_search(&i).is_ok() {
                continue;
            }
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

/// A body's mass, principal moments, centre of mass and inertial frame.
pub type InertialValues = (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>);

/// An `<inertial>`'s values as MuJoCo 3.5.0's compiler makes them, before a
/// body's geoms (`user_objects.cc:2402-2427`), its principal axes `frame`
/// where it writes no `fullinertia` (its quaternion or orientation
/// alternative, resolved by the caller). A `fullinertia` is made principal
/// as `mjuu_fullInertia` makes it (`:2413-2417`): its moments in decreasing
/// order and their frame in `iquat`, refused where the smallest is below
/// mjEPS. What it makes may not be finite (a `fullinertia` near the largest
/// double), which MuJoCo keeps or, under `inertiafromgeom="true"`, replaces
/// with its geoms'.
pub fn inertial_values(
    inertial: &MjcfInertial,
    frame: UnitQuaternion<f64>,
) -> Result<InertialValues> {
    let mass = inertial.mass;
    let ipos = inertial.pos;

    // Priority: fullinertia > diaginertia > default
    if let Some(full) = inertial.fullinertia {
        let (moments, frame) = full_inertia(&full)
            .map_err(|reason| MjcfError::invalid_value("inertial", "fullinertia", reason))?;
        // The principal frame is the inertial frame: MuJoCo's
        // `mjuu_fullInertia` overwrites `iquat` (`user_objects.cc:2412-2416`),
        // and the parser refuses an orientation written beside a
        // `fullinertia`, as MuJoCo's reader does.
        let iquat =
            UnitQuaternion::new_unchecked(Quaternion::new(frame[0], frame[1], frame[2], frame[3]));
        Ok((mass, Vector3::from(moments), ipos, iquat))
    } else if let Some(diag) = inertial.diaginertia {
        // Diagonal inertia already in principal axes
        Ok((mass, diag, ipos, frame))
    } else {
        // MuJoCo's default inertia is 0 (`mjs_defaultBody`, `user_init.c:75-76`).
        Ok((mass, Vector3::zeros(), ipos, frame))
    }
}

/// This crate's refusal of an `<inertial>` whose principal moments or frame
/// are not finite, which MuJoCo loads: only a `fullinertia` can make one
/// (the reader refuses a value that is not finite).
pub fn refuse_non_finite_inertial(values: &InertialValues) -> Result<()> {
    let (_, moments, _, frame) = values;
    let frame = [frame.w, frame.i, frame.j, frame.k];
    if moments.iter().chain(frame.iter()).all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(MjcfError::invalid_value(
            "inertial",
            "fullinertia",
            "its principal moments or axes are not finite",
        ))
    }
}

/// A mesh or SDF geom's mesh's mass properties, as `process_mesh` computed
/// them; `None` for any other geom, and for an SDF that names no mesh.
pub fn geom_mesh_props(
    geom: &MjcfGeom,
    mesh_lookup: &HashMap<String, usize>,
    mesh_props: &[MeshProps],
) -> Option<MeshProps> {
    if !matches!(geom.geom_type?, MjcfGeomType::Mesh | MjcfGeomType::Sdf) {
        return None;
    }
    let id = mesh_lookup.get(geom.mesh.as_ref()?)?;
    mesh_props.get(*id).copied()
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

/// A geom as its body's inertia reads it: the geom and its frame in its
/// body (`pos`, `quat`), as `process_geom` resolved them from its `pos` and
/// its quaternion or orientation alternative.
pub struct PlacedGeom<'a> {
    /// The geom, its classes applied.
    pub geom: &'a MjcfGeom,
    /// Its position in its body's frame.
    pub pos: Vector3<f64>,
    /// Its orientation in its body's frame.
    pub quat: UnitQuaternion<f64>,
}

/// A geom as MuJoCo 3.5.0's `InertiaFromGeom` reads it: a compiled
/// geom's `pos`, `quat`, `mass_` and `inertia` (`user_objects.cc:2158-2224`),
/// its principal moments in its frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialGeom {
    /// Its centre of mass in its body's frame.
    pub pos: [f64; 3],
    /// Its principal axes in its body's frame, (w, x, y, z).
    pub quat: [f64; 4],
    /// Its mass.
    pub mass: f64,
    /// Its principal moments about `pos`, along `quat`'s axes.
    pub inertia: [f64; 3],
}

impl InertialGeom {
    /// `placed` as MuJoCo holds it once compiled. A primitive's moments are
    /// its inertia's diagonal ([`compute_geom_inertia`]) in its own frame. A
    /// mesh's frame is moved to its mesh's centre of mass and turned to its
    /// mesh's principal axes ([`MeshProps`]), accumulated into the geom's as
    /// `mjuu_frameaccum` does (`user_objects.cc:3748-3754`), and its moments
    /// are the geom's along those axes.
    pub fn new(
        placed: &PlacedGeom<'_>,
        mesh_props: Option<&MeshProps>,
        hfield_size: Option<&[f64; 4]>,
    ) -> Self {
        let geom = placed.geom;
        let mass = compute_geom_mass(geom, mesh_props, hfield_size);
        let pos = [placed.pos.x, placed.pos.y, placed.pos.z];
        let quat = [placed.quat.w, placed.quat.i, placed.quat.j, placed.quat.k];
        let inertia = compute_geom_inertia(geom, mesh_props, hfield_size);
        let inertia = [inertia[(0, 0)], inertia[(1, 1)], inertia[(2, 2)]];
        if let (Some(MjcfGeomType::Mesh | MjcfGeomType::Sdf), Some(props)) =
            (geom.geom_type, mesh_props)
        {
            let turned = mulvecmat(&props.com, &quat2mat(&quat));
            return Self {
                pos: [pos[0] + turned[0], pos[1] + turned[1], pos[2] + turned[2]],
                quat: mulquat(&quat, &props.quat),
                mass,
                inertia,
            };
        }
        Self {
            pos,
            quat,
            mass,
            inertia,
        }
    }
}

/// Why MuJoCo 3.5.0's `InertiaFromGeom` refuses a body, or this crate does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeomInertiaFault {
    /// Its selected geoms' mass is below mjEPS (`user_objects.cc:2194-2196`).
    MassTooSmall,
    /// `mjuu_fullInertia` refuses the summed inertia (`:2219-2222`).
    PrincipalAxes(&'static str),
    /// The mass, moments, centre or frame it gives are not all finite, where
    /// MuJoCo loads them (registry row D-INERTIA-NONFINITE).
    NotFinite,
}

impl GeomInertiaFault {
    /// The refusal, in MuJoCo's words where MuJoCo refuses.
    pub fn error(self) -> MjcfError {
        match self {
            Self::MassTooSmall => {
                MjcfError::invalid_model("body mass is too small, cannot compute center of mass")
            }
            Self::PrincipalAxes(reason) => MjcfError::invalid_model(format!(
                "error '{reason}' in alternative for principal axes"
            )),
            Self::NotFinite => MjcfError::invalid_model(
                "the body's mass or inertia computed from its geoms is not finite",
            ),
        }
    }
}

/// A body's mass, principal moments, centre of mass and inertial frame
/// (w, x, y, z), as [`inertia_from_geoms`] gives them.
pub type BodyInertial = (f64, [f64; 3], [f64; 3], [f64; 4]);

/// MuJoCo 3.5.0's `InertiaFromGeom` (`user_objects.cc:2158-2224`) over the
/// geoms it selects (their group in `inertiagrouprange`, their mass above
/// mjEPS), each
/// product and sum in its order: none gives nothing; one is copied, its
/// moments and frame as they are; several give their total mass, their
/// centre of mass and the principal moments and axes of their inertia
/// about it (`mjuu_globalinertia`, `mjuu_offcenter`, `mjuu_fullInertia`).
/// Where that inertia's first entry is not a number, `mjuu_fullInertia`
/// returns without writing (`user_util.cc:786-788`), and the body keeps
/// `prior` (its moments and frame before its geoms).
///
/// A result that is not all finite is refused here, where MuJoCo loads it
/// (D-INERTIA-NONFINITE); MuJoCo's two refusals are kept.
pub fn inertia_from_geoms(
    geoms: &[InertialGeom],
    prior: ([f64; 3], [f64; 4]),
) -> std::result::Result<Option<BodyInertial>, GeomInertiaFault> {
    let inertial = match geoms {
        [] => return Ok(None),
        [geom] => (geom.mass, geom.inertia, geom.pos, geom.quat),
        _ => {
            let mut mass = 0.0;
            let mut com = [0.0; 3];
            for geom in geoms {
                mass += geom.mass;
                com[0] += geom.mass * geom.pos[0];
                com[1] += geom.mass * geom.pos[1];
                com[2] += geom.mass * geom.pos[2];
            }
            // Unreachable from a document: each geom here has a mass above
            // mjEPS and there are two or more. MuJoCo's own check
            // (`user_objects.cc:2194-2196`) is unreachable for the same
            // reason; it is kept as MuJoCo keeps it.
            if mass < MJ_EPS {
                return Err(GeomInertiaFault::MassTooSmall);
            }
            let ipos = [com[0] / mass, com[1] / mass, com[2] / mass];
            let mut toti = [0.0; 6];
            for geom in geoms {
                let dpos = [
                    geom.pos[0] - ipos[0],
                    geom.pos[1] - ipos[1],
                    geom.pos[2] - ipos[2],
                ];
                let inert0 = global_inertia(&geom.inertia, &geom.quat);
                let inert1 = off_center(geom.mass, &dpos);
                for j in 0..6 {
                    toti[j] = toti[j] + inert0[j] + inert1[j];
                }
            }
            let (inertia, iquat) = if toti[0].is_nan() {
                prior
            } else {
                full_inertia(&toti).map_err(GeomInertiaFault::PrincipalAxes)?
            };
            (mass, inertia, ipos, iquat)
        }
    };
    let (mass, inertia, ipos, iquat) = inertial;
    if !std::iter::once(mass)
        .chain(inertia)
        .chain(ipos)
        .chain(iquat)
        .all(f64::is_finite)
    {
        return Err(GeomInertiaFault::NotFinite);
    }
    Ok(Some(inertial))
}

/// `mjuu_globalinertia` (`user_util.cc:509-525`): principal moments `local`
/// along the axes of `quat`, as the inertia `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`.
fn global_inertia(local: &[f64; 3], quat: &[f64; 4]) -> [f64; 6] {
    let mat = quat2mat(quat);
    let tmp = [
        mat[0] * local[0],
        mat[3] * local[0],
        mat[6] * local[0],
        mat[1] * local[1],
        mat[4] * local[1],
        mat[7] * local[1],
        mat[2] * local[2],
        mat[5] * local[2],
        mat[8] * local[2],
    ];
    [
        mat[0] * tmp[0] + mat[1] * tmp[3] + mat[2] * tmp[6],
        mat[3] * tmp[1] + mat[4] * tmp[4] + mat[5] * tmp[7],
        mat[6] * tmp[2] + mat[7] * tmp[5] + mat[8] * tmp[8],
        mat[0] * tmp[1] + mat[1] * tmp[4] + mat[2] * tmp[7],
        mat[0] * tmp[2] + mat[1] * tmp[5] + mat[2] * tmp[8],
        mat[3] * tmp[2] + mat[4] * tmp[5] + mat[5] * tmp[8],
    ]
}

/// `mjuu_offcenter` (`user_util.cc:530-537`): the inertia `mass` adds at
/// `vec` from the centre, `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`.
fn off_center(mass: f64, vec: &[f64; 3]) -> [f64; 6] {
    [
        mass * (vec[1] * vec[1] + vec[2] * vec[2]),
        mass * (vec[0] * vec[0] + vec[2] * vec[2]),
        mass * (vec[0] * vec[0] + vec[1] * vec[1]),
        -mass * vec[0] * vec[1],
        -mass * vec[0] * vec[2],
        -mass * vec[1] * vec[2],
    ]
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
        let identity = nalgebra::UnitQuaternion::identity();
        let turn = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0);
        let (_, _, _, plain) = super::inertial_values(&full, identity).expect("principal");
        let (_, _, _, kept) = super::inertial_values(&turned, turn).expect("principal");
        assert_eq!(plain, kept);
    }
}
