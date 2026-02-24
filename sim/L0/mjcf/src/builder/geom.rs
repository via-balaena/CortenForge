//! Geometry and site processing.
//!
//! Converts MJCF `<geom>` and `<site>` elements into `Model` arrays.
//! Includes helpers for computing mass, inertia, fromto poses, and
//! size vector conversion for each geom type.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::GeomType;

use super::fluid::compute_geom_fluid;
use super::mesh::MeshProps;
use super::orientation::resolve_orientation;
use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, ModelConversionError};
use crate::types::{FluidShape, MjcfGeom, MjcfGeomType, MjcfSite};

impl ModelBuilder {
    pub(crate) fn process_geom(
        &mut self,
        geom: &MjcfGeom,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let geom_id = self.geom_type.len();

        // Convert geom type (MuJoCo defaults to Sphere when unspecified)
        let geom_type = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
            MjcfGeomType::Sphere => GeomType::Sphere,
            MjcfGeomType::Box => GeomType::Box,
            MjcfGeomType::Capsule => GeomType::Capsule,
            MjcfGeomType::Cylinder => GeomType::Cylinder,
            MjcfGeomType::Ellipsoid => GeomType::Ellipsoid,
            MjcfGeomType::Plane => GeomType::Plane,
            MjcfGeomType::Mesh | MjcfGeomType::TriangleMesh => GeomType::Mesh,
            MjcfGeomType::Hfield => GeomType::Hfield,
            MjcfGeomType::Sdf => GeomType::Sdf,
        };

        // Handle mesh geom linking
        let geom_mesh_ref = if geom_type == GeomType::Mesh {
            match &geom.mesh {
                Some(mesh_name) => {
                    let mesh_id = self.mesh_name_to_id.get(mesh_name).ok_or_else(|| {
                        ModelConversionError {
                            message: format!(
                                "geom '{}': references undefined mesh '{}'",
                                geom.name.as_deref().unwrap_or("<unnamed>"),
                                mesh_name
                            ),
                        }
                    })?;
                    Some(*mesh_id)
                }
                None => {
                    return Err(ModelConversionError {
                        message: format!(
                            "geom '{}': type is mesh but no mesh attribute specified",
                            geom.name.as_deref().unwrap_or("<unnamed>")
                        ),
                    });
                }
            }
        } else {
            None
        };

        // Handle hfield geom linking
        let geom_hfield_ref = if geom_type == GeomType::Hfield {
            match &geom.hfield {
                Some(name) => {
                    let hfield_id =
                        self.hfield_name_to_id
                            .get(name)
                            .ok_or_else(|| ModelConversionError {
                                message: format!(
                                    "geom '{}': references undefined hfield '{}'",
                                    geom.name.as_deref().unwrap_or("<unnamed>"),
                                    name
                                ),
                            })?;
                    Some(*hfield_id)
                }
                None => {
                    return Err(ModelConversionError {
                        message: format!(
                            "geom '{}': type is hfield but no hfield attribute specified",
                            geom.name.as_deref().unwrap_or("<unnamed>")
                        ),
                    });
                }
            }
        } else {
            None
        };

        // Handle fromto for capsules/cylinders
        let (pos, quat, size) = if let Some(fromto) = geom.fromto {
            compute_fromto_pose(fromto, &geom.size)
        } else {
            // Orientation resolution: euler > axisangle > xyaxes > zaxis > quat.
            let orientation = resolve_orientation(
                geom.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
                geom.euler,
                geom.axisangle,
                geom.xyaxes,
                geom.zaxis,
                &self.compiler,
            );
            (
                geom.pos.unwrap_or_else(Vector3::zeros),
                orientation,
                geom_size_to_vec3(&geom.size, geom_type),
            )
        };

        // Override geom_size for hfield geoms with asset dimensions.
        // geom_size_to_vec3() has no access to hfield asset data (only gets the MJCF
        // geom's size attribute, which is typically absent for hfield geoms).
        let size = if let Some(hfield_id) = geom_hfield_ref {
            let hf_size = &self.hfield_size[hfield_id];
            Vector3::new(hf_size[0], hf_size[1], hf_size[2])
        } else {
            size
        };

        self.geom_type.push(geom_type);
        self.geom_body.push(body_id);
        self.geom_pos.push(pos);
        self.geom_quat.push(quat);
        self.geom_size.push(size);
        self.geom_friction
            .push(geom.friction.unwrap_or(Vector3::new(1.0, 0.005, 0.0001)));

        // Validate and clamp condim to valid values {1, 3, 4, 6}
        // Invalid values are rounded up to the next valid value per MuJoCo convention
        let condim = match geom.condim.unwrap_or(3) {
            1 => 1,
            2 => {
                tracing::warn!(
                    "Geom {:?} has invalid condim=2, rounding up to 3",
                    geom.name
                );
                3
            }
            3 => 3,
            4 => 4,
            5 => {
                tracing::warn!(
                    "Geom {:?} has invalid condim=5, rounding up to 6",
                    geom.name
                );
                6
            }
            c if c >= 6 => {
                if c > 6 {
                    tracing::warn!(
                        "Geom {:?} has invalid condim={}, clamping to 6",
                        geom.name,
                        c
                    );
                }
                6
            }
            c => {
                // condim <= 0
                tracing::warn!(
                    "Geom {:?} has invalid condim={}, defaulting to 3",
                    geom.name,
                    c
                );
                3
            }
        };
        self.geom_condim.push(condim);
        #[allow(clippy::cast_sign_loss)]
        {
            self.geom_contype.push(geom.contype.unwrap_or(1) as u32);
            self.geom_conaffinity
                .push(geom.conaffinity.unwrap_or(1) as u32);
        }
        self.geom_name.push(geom.name.clone());
        if let Some(ref name) = geom.name {
            if !name.is_empty() {
                self.geom_name_to_id.insert(name.clone(), geom_id);
            }
        }
        self.geom_mesh.push(geom_mesh_ref);
        self.geom_hfield.push(geom_hfield_ref);
        self.geom_sdf.push(None); // SDF geoms are programmatic — never set via MJCF

        // Solver parameters (fall back to MuJoCo defaults if not specified in MJCF)
        self.geom_solref.push(geom.solref.unwrap_or(DEFAULT_SOLREF));
        self.geom_solimp.push(geom.solimp.unwrap_or(DEFAULT_SOLIMP));

        // Contact parameter combination fields (MuJoCo mj_contactParam)
        self.geom_priority.push(geom.priority.unwrap_or(0));
        self.geom_solmix.push(geom.solmix.unwrap_or(1.0));
        self.geom_margin.push(geom.margin.unwrap_or(0.0));
        self.geom_gap.push(geom.gap.unwrap_or(0.0));
        self.geom_group.push(geom.group.unwrap_or(0));
        // Default gray: MuJoCo uses [0.5, 0.5, 0.5, 1.0] for unspecified geom rgba.
        let default_rgba = [0.5, 0.5, 0.5, 1.0];
        self.geom_rgba
            .push(geom.rgba.map_or(default_rgba, |v| [v.x, v.y, v.z, v.w]));

        // Fluid interaction data (§40)
        let fluid = compute_geom_fluid(
            geom.fluidshape.unwrap_or(FluidShape::None),
            geom.fluidcoef,
            geom_type,
            size,
        );
        self.geom_fluid.push(fluid);

        Ok(geom_id)
    }

    pub(crate) fn process_site(
        &mut self,
        site: &MjcfSite,
        body_id: usize,
    ) -> std::result::Result<usize, ModelConversionError> {
        let site_id = self.site_body.len();

        // Store name mapping for actuator site transmission
        if !site.name.is_empty() {
            self.site_name_to_id.insert(site.name.clone(), site_id);
        }

        self.site_body.push(body_id);

        // Convert site type string to GeomType (sphere is the MuJoCo default)
        let site_type_str = site.site_type.as_deref().unwrap_or("sphere");
        let geom_type = match site_type_str {
            "capsule" => GeomType::Capsule,
            "cylinder" => GeomType::Cylinder,
            "box" => GeomType::Box,
            "ellipsoid" => GeomType::Ellipsoid,
            _ => GeomType::Sphere, // Default for "sphere" and unknown types
        };
        self.site_type.push(geom_type);

        self.site_pos.push(site.pos.unwrap_or_else(Vector3::zeros));
        self.site_quat.push(resolve_orientation(
            site.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
            site.euler,
            site.axisangle,
            site.xyaxes,
            site.zaxis,
            &self.compiler,
        ));

        // Convert site size (MuJoCo uses single value for sphere, array for others)
        let site_size = site.size.as_deref().unwrap_or(&[0.01]);
        let size = if site_size.is_empty() {
            Vector3::new(0.01, 0.01, 0.01) // Default small size
        } else if site_size.len() == 1 {
            let s = site_size[0];
            Vector3::new(s, s, s)
        } else {
            Vector3::new(
                site_size[0],
                site_size.get(1).copied().unwrap_or(site_size[0]),
                site_size.get(2).copied().unwrap_or(site_size[0]),
            )
        };
        self.site_size.push(size);

        self.site_name.push(if site.name.is_empty() {
            None
        } else {
            Some(site.name.clone())
        });
        self.site_group.push(site.group.unwrap_or(0));
        // Default site rgba: MuJoCo uses [0.5, 0.5, 0.5, 1.0] for unspecified site rgba.
        let default_rgba = [0.5, 0.5, 0.5, 1.0];
        self.site_rgba
            .push(site.rgba.map_or(default_rgba, |v| [v.x, v.y, v.z, v.w]));

        Ok(site_id)
    }
}

/// Compute the effective center of mass of a geom in the body frame.
///
/// For primitive geoms, the local COM is at the geom origin (`geom.pos`).
/// For mesh geoms, the mesh may have a non-zero internal COM in mesh-local
/// coordinates. The effective COM is `geom.pos + R * mesh_com` where R is
/// the geom's orientation in the body frame.
pub fn geom_effective_com(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> Vector3<f64> {
    let pos = geom.pos.unwrap_or_else(Vector3::zeros);
    let q = geom.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0));
    if let Some(&(_, mesh_com, _)) = mesh_props {
        if mesh_com.norm() > 1e-14 {
            let r = UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3]));
            return pos + r.transform_vector(&mesh_com);
        }
    }
    pos
}

/// Compute mass of a single geom.
///
/// For mesh geoms, accepts pre-computed mesh properties to avoid redundant
/// `compute_mesh_inertia()` calls (see `MeshProps`).
pub fn compute_geom_mass(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> f64 {
    if let Some(mass) = geom.mass {
        return mass;
    }

    let volume = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
        }
        MjcfGeomType::Box => {
            let x = geom.size.first().copied().unwrap_or(0.1);
            let y = geom.size.get(1).copied().unwrap_or(x);
            let z = geom.size.get(2).copied().unwrap_or(y);
            8.0 * x * y * z // size is half-extents
        }
        MjcfGeomType::Capsule => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * (h * 2.0 + (4.0 / 3.0) * r)
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1);
            std::f64::consts::PI * r.powi(2) * h * 2.0
        }
        MjcfGeomType::Mesh => {
            if let Some(&(volume, _, _)) = mesh_props {
                volume.abs()
            } else {
                0.001
            }
        }
        _ => 0.001, // Default small volume for other types (Plane, Hfield)
    };

    geom.density.unwrap_or(1000.0) * volume
}

/// Compute inertia tensor of a single geom about its center (geom-local frame).
///
/// Returns a full 3×3 matrix. Primitive geoms produce diagonal tensors;
/// mesh geoms may have off-diagonal terms.
///
/// For mesh geoms, accepts pre-computed mesh properties to avoid redundant
/// `compute_mesh_inertia()` calls (see `MeshProps`).
///
/// Uses exact formulas matching MuJoCo's computation:
/// - Sphere: I = (2/5) m r²
/// - Box: I_x = (1/12) m (y² + z²), etc.
/// - Cylinder: I_x = (1/12) m (3r² + h²), I_z = (1/2) m r²
/// - Capsule: Exact formula including hemispherical end caps
/// - Mesh: Signed tetrahedron decomposition (Mirtich 1996)
pub fn compute_geom_inertia(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> Matrix3<f64> {
    let mass = compute_geom_mass(geom, mesh_props);

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let i = 0.4 * mass * r.powi(2); // (2/5) m r²
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box => {
            // Full dimensions (size is half-extents)
            let x = geom.size.first().copied().unwrap_or(0.1) * 2.0;
            let y = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            let z = geom.size.get(2).copied().unwrap_or(0.1) * 2.0;
            let c = mass / 12.0;
            Matrix3::from_diagonal(&Vector3::new(
                c * (y * y + z * z),
                c * (x * x + z * z),
                c * (x * x + y * y),
            ))
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Full height
            // Solid cylinder about center
            let ix = mass * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let iz = 0.5 * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Capsule => {
            // Exact capsule inertia (cylinder + two hemispheres)
            // Reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0; // Cylinder height (not including caps)

            // Volume components
            let v_cyl = std::f64::consts::PI * r.powi(2) * h;
            let v_sphere = (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
            let v_total = v_cyl + v_sphere;

            // Mass components (assuming uniform density)
            let m_cyl = mass * v_cyl / v_total;
            let m_hemi = mass * v_sphere / (2.0 * v_total); // Each hemisphere

            // Cylinder inertia about its own center
            let i_cyl_x = m_cyl * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let i_cyl_z = 0.5 * m_cyl * r.powi(2);

            // Hemisphere inertia about its own center (half of sphere)
            let i_hemi_own = 0.4 * m_hemi * r.powi(2);

            // Distance from capsule center to hemisphere center
            let d = h / 2.0 + (3.0 / 8.0) * r; // Center of hemisphere from cylinder end

            // Parallel axis theorem for hemispheres
            let i_hemi_x = i_hemi_own + m_hemi * d.powi(2);
            let i_hemi_z = i_hemi_own; // No parallel axis for z (axial)

            // Total: cylinder + 2 hemispheres
            let ix = i_cyl_x + 2.0 * i_hemi_x;
            let iz = i_cyl_z + 2.0 * i_hemi_z;

            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Ellipsoid => {
            // Ellipsoid inertia: I_x = (1/5) m (b² + c²), etc.
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            let coeff = mass / 5.0;
            Matrix3::from_diagonal(&Vector3::new(
                coeff * (b * b + c * c),
                coeff * (a * a + c * c),
                coeff * (a * a + b * b),
            ))
        }
        MjcfGeomType::Mesh => {
            if let Some(&(volume, _, inertia_unit)) = mesh_props {
                let density = geom.density.unwrap_or(1000.0);
                let mass_actual = geom.mass.unwrap_or_else(|| density * volume.abs());
                let scale = if volume.abs() > 1e-10 {
                    mass_actual / volume.abs()
                } else {
                    density
                };
                inertia_unit * scale
            } else {
                Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001))
            }
        }
        _ => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)), // Default small inertia
    }
}

/// Compute pose from fromto specification.
pub fn compute_fromto_pose(
    fromto: [f64; 6],
    size: &[f64],
) -> (Vector3<f64>, UnitQuaternion<f64>, Vector3<f64>) {
    let from = Vector3::new(fromto[0], fromto[1], fromto[2]);
    let to = Vector3::new(fromto[3], fromto[4], fromto[5]);

    let center = (from + to) / 2.0;
    let axis = to - from;
    let half_length = axis.norm() / 2.0;
    let radius = size.first().copied().unwrap_or(0.05);

    // Compute rotation to align Z with axis
    let quat = if axis.norm() > 1e-10 {
        let axis_normalized = axis.normalize();
        let z_axis = Vector3::z();

        if (axis_normalized - z_axis).norm() < 1e-10 {
            UnitQuaternion::identity()
        } else if (axis_normalized + z_axis).norm() < 1e-10 {
            UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(Vector3::x()),
                std::f64::consts::PI,
            )
        } else {
            let rot_axis = z_axis.cross(&axis_normalized);
            // Clamp to avoid NaN from floating-point precision issues
            let angle = z_axis.dot(&axis_normalized).clamp(-1.0, 1.0).acos();
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(rot_axis), angle)
        }
    } else {
        UnitQuaternion::identity()
    };

    // Convention: size.x = radius, size.y = half_length, size.z = 0.0
    // This matches geom_size_to_vec3 for Capsule/Cylinder.
    (center, quat, Vector3::new(radius, half_length, 0.0))
}

/// Convert geom size array to `Vector3`.
pub fn geom_size_to_vec3(size: &[f64], geom_type: GeomType) -> Vector3<f64> {
    match geom_type {
        GeomType::Sphere => {
            let r = size.first().copied().unwrap_or(0.1);
            Vector3::new(r, r, r)
        }
        GeomType::Box => {
            let x = size.first().copied().unwrap_or(0.1);
            let y = size.get(1).copied().unwrap_or(x);
            let z = size.get(2).copied().unwrap_or(y);
            Vector3::new(x, y, z)
        }
        GeomType::Capsule | GeomType::Cylinder => {
            // Convention: size.x = radius, size.y = half_length, size.z unused
            // This matches the collision code expectations.
            let r = size.first().copied().unwrap_or(0.1);
            let h = size.get(1).copied().unwrap_or(0.1);
            Vector3::new(r, h, 0.0)
        }
        GeomType::Ellipsoid => {
            // Ellipsoid radii (rx, ry, rz)
            let rx = size.first().copied().unwrap_or(0.1);
            let ry = size.get(1).copied().unwrap_or(rx);
            let rz = size.get(2).copied().unwrap_or(ry);
            Vector3::new(rx, ry, rz)
        }
        _ => Vector3::new(0.1, 0.1, 0.1),
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::compute_geom_inertia;
    use crate::types::{MjcfGeom, MjcfGeomType};
    use nalgebra::Vector3;

    /// Test capsule inertia computation is physically reasonable.
    #[test]
    fn test_capsule_inertia() {
        // A capsule should have I_z < I_x = I_y (axially symmetric, longer than wide)
        let geom = MjcfGeom {
            name: None,
            class: None,
            geom_type: Some(MjcfGeomType::Capsule),
            pos: Some(Vector3::zeros()),
            quat: Some(nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0)),
            euler: None,
            axisangle: None,
            xyaxes: None,
            zaxis: None,
            size: vec![0.1, 0.5], // radius=0.1, half-height=0.5
            fromto: None,
            density: Some(1000.0),
            mass: None,
            friction: Some(Vector3::new(1.0, 0.005, 0.0001)),
            rgba: Some(nalgebra::Vector4::new(0.5, 0.5, 0.5, 1.0)),
            contype: Some(1),
            conaffinity: Some(1),
            condim: Some(3),
            mesh: None,
            hfield: None,
            solref: None,
            solimp: None,
            priority: Some(0),
            solmix: Some(1.0),
            margin: Some(0.0),
            gap: Some(0.0),
            group: Some(0),
            material: None,
            fluidshape: None,
            fluidcoef: None,
        };

        let inertia = compute_geom_inertia(&geom, None);

        // Ix = Iy (axially symmetric) — diagonal elements (0,0) and (1,1)
        assert!((inertia[(0, 0)] - inertia[(1, 1)]).abs() < 1e-10);

        // Iz < Ix (thin cylinder is easier to spin about long axis)
        assert!(inertia[(2, 2)] < inertia[(0, 0)]);

        // All positive
        assert!(inertia[(0, 0)] > 0.0);
        assert!(inertia[(1, 1)] > 0.0);
        assert!(inertia[(2, 2)] > 0.0);
    }
}
