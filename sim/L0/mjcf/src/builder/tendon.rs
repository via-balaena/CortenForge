//! Tendon processing.
//!
//! Converts MJCF `<tendon>` elements (fixed and spatial) into Model
//! tendon/wrap arrays. Handles joint resolution for fixed tendons,
//! site/geom/pulley resolution for spatial tendons, and autolimits.

use sim_core::{GeomType, MjJointType, TendonType, WrapType};
use tracing::warn;

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::{MjcfTendon, MjcfTendonType, SpatialPathElement};

impl ModelBuilder {
    pub(crate) fn process_tendons(
        &mut self,
        tendons: &[MjcfTendon],
    ) -> std::result::Result<(), ModelConversionError> {
        for (t_idx, tendon) in tendons.iter().enumerate() {
            let tendon = self.resolver.apply_to_tendon(tendon);
            if !tendon.name.is_empty() {
                self.tendon_name_to_id.insert(tendon.name.clone(), t_idx);
            }
            self.tendon_type.push(match tendon.tendon_type {
                MjcfTendonType::Fixed => TendonType::Fixed,
                MjcfTendonType::Spatial => TendonType::Spatial,
            });
            self.tendon_range
                .push(tendon.range.unwrap_or((-f64::MAX, f64::MAX)));
            // Autolimits: infer limited from range presence.
            let tendon_limited = match tendon.limited {
                Some(v) => v,
                None => {
                    if self.compiler.autolimits {
                        tendon.range.is_some()
                    } else {
                        false
                    }
                }
            };
            self.tendon_limited.push(tendon_limited);
            self.tendon_stiffness.push(tendon.stiffness.unwrap_or(0.0));
            self.tendon_damping.push(tendon.damping.unwrap_or(0.0));
            self.tendon_frictionloss
                .push(tendon.frictionloss.unwrap_or(0.0));
            self.tendon_solref_fri
                .push(tendon.solreffriction.unwrap_or(DEFAULT_SOLREF));
            self.tendon_solimp_fri
                .push(tendon.solimpfriction.unwrap_or(DEFAULT_SOLIMP));
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
            self.tendon_solref
                .push(tendon.solref.unwrap_or(DEFAULT_SOLREF));
            self.tendon_solimp
                .push(tendon.solimp.unwrap_or(DEFAULT_SOLIMP));
            // S1/S3: Use parsed springlength, or sentinel [-1, -1] for auto-compute
            self.tendon_lengthspring.push(match tendon.springlength {
                Some(pair) => pair.into(),
                None => [-1.0, -1.0], // sentinel: resolved to [length0, length0] later
            });
            self.tendon_length0.push(0.0); // Computed after construction from qpos0

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
                                .ok_or_else(|| ModelConversionError {
                                    message: format!(
                                        "Tendon '{}' references unknown joint '{}'",
                                        tendon.name, joint_name
                                    ),
                                })?;
                        // Fixed tendons assume qposadr == dofadr (true for hinge/slide).
                        // Ball/free joints have different qpos and dof dimensions, so
                        // a linear coupling L = coef * qpos[dof_adr] would be incorrect.
                        let jnt_type = self.jnt_type[jnt_idx];
                        if jnt_type != MjJointType::Hinge && jnt_type != MjJointType::Slide {
                            warn!(
                                "Tendon '{}' references {} joint '{}' — fixed tendons only \
                                 support hinge/slide joints. Ball/free joints will produce \
                                 incorrect results.",
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
                                let site_idx = *self
                                    .site_name_to_id
                                    .get(site.as_str())
                                    .ok_or_else(|| ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' references unknown site '{}'",
                                            tendon.name, site
                                        ),
                                    })?;
                                self.wrap_type.push(WrapType::Site);
                                self.wrap_objid.push(site_idx);
                                self.wrap_prm.push(0.0);
                                self.wrap_sidesite.push(usize::MAX);
                            }
                            SpatialPathElement::Geom { geom, sidesite } => {
                                let geom_id =
                                    *self.geom_name_to_id.get(geom.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown geom '{}'",
                                                tendon.name, geom
                                            ),
                                        }
                                    })?;
                                // Validate geom type: must be Sphere or Cylinder
                                let gt = self.geom_type[geom_id];
                                if gt != GeomType::Sphere && gt != GeomType::Cylinder {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has type {:?}, \
                                             but only Sphere and Cylinder are supported",
                                            tendon.name, geom, gt
                                        ),
                                    });
                                }
                                // Validate geom radius > 0 (rule 5)
                                let radius = self.geom_size[geom_id].x;
                                if radius <= 0.0 {
                                    return Err(ModelConversionError {
                                        message: format!(
                                            "Tendon '{}' wrapping geom '{}' has zero or \
                                             negative radius {}",
                                            tendon.name, geom, radius
                                        ),
                                    });
                                }
                                let ss_id = if let Some(ss) = sidesite {
                                    *self.site_name_to_id.get(ss.as_str()).ok_or_else(|| {
                                        ModelConversionError {
                                            message: format!(
                                                "Tendon '{}' references unknown sidesite '{}'",
                                                tendon.name, ss
                                            ),
                                        }
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
}
