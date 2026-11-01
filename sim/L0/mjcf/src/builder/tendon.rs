//! Tendon processing.
//!
//! Converts MJCF `<tendon>` elements (fixed and spatial) into Model
//! tendon/wrap arrays. Handles joint resolution for fixed tendons,
//! site/geom/pulley resolution for spatial tendons, and autolimits.

use sim_core::{GeomType, MjJointType, Model, TendonType, WrapType};
use tracing::warn;

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, written_over};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{MjcfTendon, MjcfTendonType, SpatialPathElement};

impl ModelBuilder {
    pub(crate) fn process_tendons(&mut self, tendons: &[MjcfTendon]) -> Result<()> {
        for (t_idx, tendon) in tendons.iter().enumerate() {
            let at = || Location::nth(ElementKind::Tendon, Some(&tendon.name), t_idx);
            if !tendon.name.is_empty() {
                self.tendon_name_to_id.insert(tendon.name.clone(), t_idx);
            }
            self.tendon_type.push(match tendon.tendon_type {
                MjcfTendonType::Fixed => TendonType::Fixed,
                MjcfTendonType::Spatial => TendonType::Spatial,
            });
            self.tendon_range
                .push(tendon.range.unwrap_or((-f64::MAX, f64::MAX)));
            self.tendon_range_compiled
                .push(tendon.range.unwrap_or((0.0, 0.0)));
            // Autolimits: infer limited from range presence.
            let tendon_limited =
                super::is_limited(tendon.limited, self.compiler.autolimits, tendon.range);
            self.tendon_limited.push(tendon_limited);
            self.tendon_stiffness.push(tendon.stiffness.unwrap_or(0.0));
            self.tendon_damping.push(tendon.damping.unwrap_or(0.0));
            self.tendon_armature.push(tendon.armature.unwrap_or(0.0));
            self.tendon_frictionloss
                .push(tendon.frictionloss.unwrap_or(0.0));
            self.tendon_solref_fri
                .push(written_over(tendon.solreffriction, DEFAULT_SOLREF));
            self.tendon_solimp_fri
                .push(written_over(tendon.solimpfriction, DEFAULT_SOLIMP));
            self.tendon_name.push(if tendon.name.is_empty() {
                None
            } else {
                Some(tendon.name.clone())
            });
            self.tendon_group.push(tendon.group.unwrap_or(0));
            // Default tendon rgba: MuJoCo uses [0.5, 0.5, 0.5, 1.0] for unspecified.
            let default_rgba = [0.5, 0.5, 0.5, 1.0];
            self.tendon_rgba
                .push(tendon.rgba.map_or(default_rgba, |v| [v.x, v.y, v.z, v.w]));
            self.tendon_solref_lim
                .push(written_over(tendon.solref_limit, DEFAULT_SOLREF));
            self.tendon_solimp_lim
                .push(written_over(tendon.solimp_limit, DEFAULT_SOLIMP));
            self.tendon_margin.push(tendon.margin.unwrap_or(0.0));
            // S1/S3: Use parsed springlength, or sentinel [-1, -1] for auto-compute
            self.tendon_lengthspring.push(match tendon.springlength {
                Some(pair) => pair.into(),
                None => [-1.0, -1.0], // sentinel: resolved to [length0, length0] later
            });
            self.tendon_length0.push(0.0); // Computed after construction from qpos0
            self.tendon_user_raw.push(tendon.user.clone());

            let wrap_start = self.wrap_type.len();
            self.tendon_adr.push(wrap_start);

            match tendon.tendon_type {
                MjcfTendonType::Fixed => {
                    for (joint_name, coef) in &tendon.joints {
                        // Resolve joint name → index → DOF address
                        let jnt_idx =
                            *self
                                .joint_name_to_id
                                .get(joint_name.as_str())
                                .ok_or_else(|| {
                                    MjcfError::undefined(ElementKind::Joint, joint_name).at(at())
                                })?;
                        // Fixed tendons couple to the first qpos component of the joint
                        // (MuJoCo ref: mj_tendon() in engine_core_smooth.c).
                        // For hinge/slide: qposadr == dofadr (scalar joint).
                        // For ball: couples to quaternion w component (unusual).
                        // For free: couples to x position component (unusual).
                        // All cases are handled correctly at runtime via dof_jnt → jnt_qpos_adr lookup.
                        let jnt_type = self.jnt_type[jnt_idx];
                        if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                            warn!(
                                "Tendon '{}' references {} joint '{}' — fixed tendons \
                                 typically use hinge/slide joints. Ball/free joint coupling \
                                 is to the first qpos component only.",
                                tendon.name,
                                match jnt_type {
                                    MjJointType::Ball => "ball",
                                    MjJointType::Free => "free",
                                    _ => "unknown",
                                },
                                joint_name
                            );
                        }
                        let dof_adr = self.jnt_dof_adr[jnt_idx];
                        self.wrap_type.push(WrapType::Joint);
                        self.wrap_objid.push(dof_adr);
                        self.wrap_prm.push(*coef);
                        self.wrap_sidesite.push(usize::MAX);
                    }
                }
                MjcfTendonType::Spatial => {
                    for elem in &tendon.path_elements {
                        match elem {
                            SpatialPathElement::Site { site } => {
                                let site_idx =
                                    *self.site_name_to_id.get(site.as_str()).ok_or_else(|| {
                                        MjcfError::undefined(ElementKind::Site, site).at(at())
                                    })?;
                                self.wrap_type.push(WrapType::Site);
                                self.wrap_objid.push(site_idx);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                            SpatialPathElement::Geom { geom, sidesite } => {
                                let geom_id =
                                    *self.geom_name_to_id.get(geom.as_str()).ok_or_else(|| {
                                        MjcfError::undefined(ElementKind::Geom, geom).at(at())
                                    })?;
                                // Validate geom type: must be Sphere or Cylinder
                                let gt = self.geom_type[geom_id];
                                if gt != GeomType::Sphere && gt != GeomType::Cylinder {
                                    return Err(MjcfError::invalid_model(format!(
                                        "wrapping geom '{geom}' is not a sphere or a cylinder \
                                         (its type is {gt:?})"
                                    ))
                                    .at(at()));
                                }
                                // Validate geom radius > 0 (rule 5)
                                let radius = self.geom_size[geom_id].x;
                                if radius <= 0.0 {
                                    return Err(MjcfError::invalid_model(format!(
                                        "wrapping geom '{geom}' has zero or negative radius {radius}"
                                    ))
                                    .at(at()));
                                }
                                let ss_id = if let Some(ss) = sidesite {
                                    *self.site_name_to_id.get(ss.as_str()).ok_or_else(|| {
                                        MjcfError::undefined(ElementKind::Site, ss).at(at())
                                    })?
                                } else {
                                    usize::MAX
                                };
                                self.wrap_type.push(WrapType::Geom);
                                self.wrap_objid.push(geom_id);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(ss_id);
                            }
                            SpatialPathElement::Pulley { divisor } => {
                                self.wrap_type.push(WrapType::Pulley);
                                self.wrap_objid.push(0);
                                self.wrap_prm.push(*divisor);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                        }
                    }
                }
            }
            self.tendon_num.push(self.wrap_type.len() - wrap_start);
        }
        Ok(())
    }

    /// Where tendon `t` is, for an error.
    fn tendon_location(&self, t: usize) -> Location {
        Location::nth(ElementKind::Tendon, self.tendon_name[t].as_deref(), t)
    }

    /// MuJoCo 3.5.0's checks of a tendon's armature (`mjCTendon::Compile`),
    /// tendon by tendon: a spatial tendon's negative armature
    /// (`user_objects.cc:6342-6346`), then armature on a tendon that wraps a
    /// sphere or a cylinder, whose bias force MuJoCo cannot differentiate
    /// (`:6389-6393`). The caller runs it after every check this crate makes
    /// before building the model, after the bodies' joints and before the free
    /// joint's; a keyframe's fault comes after it, in MuJoCo as here.
    pub(crate) fn check_tendon_armature(&self) -> Result<()> {
        for t in 0..self.tendon_type.len() {
            let armature = self.tendon_armature[t];
            if self.tendon_type[t] == TendonType::Spatial && armature < 0.0 {
                return Err(negative_armature().at(self.tendon_location(t)));
            }
            let path = self.tendon_adr[t]..self.tendon_adr[t] + self.tendon_num[t];
            if armature > 0.0 && self.wrap_type[path].contains(&WrapType::Geom) {
                return Err(MjcfError::invalid_model(
                    "geom wrapping not supported by tendon armature",
                )
                .at(self.tendon_location(t)));
            }
        }
        Ok(())
    }
}

/// A fixed tendon's negative armature in the built `model`, refused
/// (`D-TENDON-ARMATURE-NEGATIVE`): MuJoCo 3.5.0's check of the sign sits in
/// the spatial path's branch (`user_objects.cc:6342-6346`), so it loads a fixed
/// tendon's and makes an indefinite mass matrix of it, silently. The caller
/// runs it once the keyframes are resolved.
pub fn check_fixed_tendon_armature(model: &Model) -> Result<()> {
    match (0..model.ntendon)
        .find(|&t| model.tendon_type[t] == TendonType::Fixed && model.tendon_armature[t] < 0.0)
    {
        Some(t) => Err(negative_armature().at(Location::nth(
            ElementKind::Tendon,
            model.tendon_name[t].as_deref(),
            t,
        ))),
        None => Ok(()),
    }
}

/// MuJoCo's refusal of a negative tendon armature.
fn negative_armature() -> MjcfError {
    MjcfError::invalid_value("tendon", "armature", "tendon armature cannot be negative")
}
