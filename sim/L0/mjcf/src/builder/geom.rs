//! Geometry and site processing.
//!
//! Converts MJCF `<geom>` and `<site>` elements into `Model` arrays.
//! Includes helpers for computing mass, inertia, fromto poses, and
//! size vector conversion for each geom type.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::GeomType;

use super::fluid::compute_geom_fluid;
use super::mesh::MeshProps;
use super::orientation::{MJ_EPS, normvec, resolve_orientation, site_orientation, z2quat};
use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, written_over};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{FluidShape, MjcfGeom, MjcfGeomType, MjcfSite, Prefix};

impl ModelBuilder {
    pub(crate) fn process_geom(&mut self, geom: &MjcfGeom, body_id: usize) -> Result<usize> {
        let geom_id = self.geom_type.len();
        let geom_type = convert_geom_type(geom.geom_type);
        let at = || {
            Location::nth(ElementKind::Geom, geom.name.as_deref(), geom_id).within_nth(
                ElementKind::Body,
                self.body_name.get(body_id).and_then(Option::as_deref),
                body_id,
            )
        };

        // Reject shellinertia on mesh geoms (MuJoCo 3.5.0 behavior)
        if geom.shellinertia == Some(true) && geom_type == GeomType::Mesh {
            return Err(MjcfError::invalid_value(
                "geom",
                "shellinertia",
                "for mesh geoms, inertia should be specified in the mesh asset",
            )
            .at(at()));
        }

        let (geom_mesh_ref, geom_hfield_ref) = self
            .resolve_geom_asset_links(geom, geom_type)
            .map_err(|e| e.with_location_if_unknown(at()))?;

        // A geom's `fromto` is already its `pos`, `quat` and `size`
        // (`resolve_geom_fromto`, once its class is applied:
        // `resolve::apply_defaults`).
        let quat = resolve_orientation(
            geom.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0)),
            geom.euler,
            geom.axisangle,
            geom.xyaxes,
            geom.zaxis,
            &self.compiler,
        )
        .map_err(|e| e.on("geom").at(at()))?;
        let pos = geom.pos.unwrap_or_else(Vector3::zeros);
        let size = geom_size_to_vec3(written_over(geom.size, [0.0; 3]), geom_type);

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
        self.geom_friction.push(Vector3::from(written_over(
            geom.friction,
            [1.0, 0.005, 0.0001],
        )));

        self.geom_condim
            .push(validate_condim(geom.condim, geom.name.as_ref()));
        // `contype`/`conaffinity` are MuJoCo bitmask attributes accepted as
        // i32 from XML but always non-negative by spec.
        #[allow(clippy::cast_sign_loss)]
        {
            self.geom_contype.push(geom.contype.unwrap_or(1) as u32);
            self.geom_conaffinity
                .push(geom.conaffinity.unwrap_or(1) as u32);
        }
        self.geom_name.push(geom.name.clone());
        if let Some(ref name) = geom.name
            && !name.is_empty()
        {
            self.geom_name_to_id.insert(name.clone(), geom_id);
        }
        self.geom_mesh.push(geom_mesh_ref);
        self.geom_hfield.push(geom_hfield_ref);
        self.geom_sdf.push(None); // SDF geoms are programmatic — never set via MJCF

        // Solver parameters (fall back to MuJoCo defaults if not specified in MJCF)
        self.geom_solref
            .push(written_over(geom.solref, DEFAULT_SOLREF));
        self.geom_solimp
            .push(written_over(geom.solimp, DEFAULT_SOLIMP));

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
        self.geom_user_raw.push(geom.user.clone());

        Ok(geom_id)
    }

    pub(crate) fn process_site(&mut self, site: &MjcfSite, body_id: usize) -> Result<usize> {
        let site_id = self.site_body.len();

        // Store name mapping for actuator site transmission
        if !site.name.is_empty() {
            self.site_name_to_id.insert(site.name.clone(), site_id);
        }

        let at = || {
            Location::named(ElementKind::Site, &site.name).within_nth(
                ElementKind::Body,
                self.body_name.get(body_id).and_then(Option::as_deref),
                body_id,
            )
        };
        // Its type and size are checked already (`validation::check_sizes`).
        let site_type = site_geom_type(site).map_err(|e| e.with_location_if_unknown(at()))?;
        self.site_body.push(body_id);
        self.site_type.push(site_type);

        // The site's class values and its `fromto` are already in it
        // (`resolve::apply_defaults`).
        self.site_pos.push(site.pos.unwrap_or_else(Vector3::zeros));
        self.site_quat
            .push(site_orientation(site, &self.compiler).map_err(|e| e.on("site").at(at()))?);
        self.site_size.push(site_size(site));

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
        self.site_user_raw.push(site.user.clone());

        Ok(site_id)
    }

    /// Resolve mesh and hfield asset references for a geom; the caller
    /// locates the errors.
    fn resolve_geom_asset_links(
        &self,
        geom: &MjcfGeom,
        geom_type: GeomType,
    ) -> Result<(Option<usize>, Option<usize>)> {
        let geom_mesh_ref = if geom_type == GeomType::Mesh {
            match &geom.mesh {
                Some(mesh_name) => {
                    let mesh_id = self
                        .mesh_name_to_id
                        .get(mesh_name)
                        .ok_or_else(|| MjcfError::undefined(ElementKind::Mesh, mesh_name))?;
                    Some(*mesh_id)
                }
                None => {
                    return Err(MjcfError::missing_attribute("geom", "mesh"));
                }
            }
        } else {
            None
        };

        let geom_hfield_ref = if geom_type == GeomType::Hfield {
            match &geom.hfield {
                Some(name) => {
                    let hfield_id = self
                        .hfield_name_to_id
                        .get(name)
                        .ok_or_else(|| MjcfError::undefined(ElementKind::Hfield, name))?;
                    Some(*hfield_id)
                }
                None => {
                    return Err(MjcfError::missing_attribute("geom", "hfield"));
                }
            }
        } else {
            None
        };

        Ok((geom_mesh_ref, geom_hfield_ref))
    }
}

/// Convert MJCF geom type to Model geom type (defaults to Sphere when unspecified).
fn convert_geom_type(geom_type: Option<MjcfGeomType>) -> GeomType {
    match geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => GeomType::Sphere,
        MjcfGeomType::Box => GeomType::Box,
        MjcfGeomType::Capsule => GeomType::Capsule,
        MjcfGeomType::Cylinder => GeomType::Cylinder,
        MjcfGeomType::Ellipsoid => GeomType::Ellipsoid,
        MjcfGeomType::Plane => GeomType::Plane,
        MjcfGeomType::Mesh => GeomType::Mesh,
        MjcfGeomType::Hfield => GeomType::Hfield,
        MjcfGeomType::Sdf => GeomType::Sdf,
    }
}

/// Validate and clamp condim to valid values {1, 3, 4, 6}.
///
/// Invalid values are rounded up to the next valid value per MuJoCo convention.
fn validate_condim(condim: Option<i32>, geom_name: Option<&String>) -> i32 {
    match condim.unwrap_or(3) {
        1 => 1,
        2 => {
            tracing::warn!(
                "Geom {:?} has invalid condim=2, rounding up to 3",
                geom_name
            );
            3
        }
        3 => 3,
        4 => 4,
        5 => {
            tracing::warn!(
                "Geom {:?} has invalid condim=5, rounding up to 6",
                geom_name
            );
            6
        }
        c if c >= 6 => {
            if c > 6 {
                tracing::warn!(
                    "Geom {:?} has invalid condim={}, clamping to 6",
                    geom_name,
                    c
                );
            }
            6
        }
        c => {
            tracing::warn!(
                "Geom {:?} has invalid condim={}, defaulting to 3",
                geom_name,
                c
            );
            3
        }
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
    if let Some(&(_, mesh_com, _)) = mesh_props
        && mesh_com.norm() > 1e-14
    {
        let r = UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3]));
        return pos + r.transform_vector(&mesh_com);
    }
    pos
}

/// A site's type (sphere is the MuJoCo default), held to MuJoCo's geom
/// type words as the reader holds it, so a site built in code is too: a
/// plane, mesh or height field refused, as MuJoCo 3.5.0 refuses one
/// (`mjCSite::Compile`, `user_objects.cc:3933-3935`), and a signed distance
/// field kept, as MuJoCo keeps it (a touch sensor on one: `D-SITE-SDF-TOUCH`).
fn site_geom_type(site: &MjcfSite) -> Result<GeomType> {
    let word = site.site_type.as_deref().unwrap_or("sphere");
    let mjcf_type = MjcfGeomType::from_str(word)
        .ok_or_else(|| MjcfError::invalid_keyword("site", "type", word))?;
    Ok(match mjcf_type {
        MjcfGeomType::Sphere => GeomType::Sphere,
        MjcfGeomType::Capsule => GeomType::Capsule,
        MjcfGeomType::Cylinder => GeomType::Cylinder,
        MjcfGeomType::Box => GeomType::Box,
        MjcfGeomType::Ellipsoid => GeomType::Ellipsoid,
        MjcfGeomType::Sdf => GeomType::Sdf,
        MjcfGeomType::Plane | MjcfGeomType::Mesh | MjcfGeomType::Hfield => {
            return Err(MjcfError::invalid_value(
                "site",
                "type",
                "meshes, hfields and planes not allowed in site",
            ));
        }
    })
}

/// A site's type and size, checked as MuJoCo 3.5.0's `mjCSite::Compile`
/// checks them (`site_geom_type`, `check_site_size`); the caller locates the
/// error.
///
/// # Errors
///
/// [`MjcfError::InvalidKeyword`] for an unknown type word,
/// [`MjcfError::InvalidValue`] for a type or size MuJoCo refuses,
/// [`MjcfError::NonFinite`] for a size that is not finite.
pub fn check_site(site: &MjcfSite) -> Result<()> {
    check_site_size(site, site_geom_type(site)?)
}

/// A site's size checked as MuJoCo 3.5.0's `checksize` checks it
/// (`user_objects.cc:152-167`, called for a site at `:3991`, after its
/// `fromto`): finite, and each of the first `mjGEOMINFO[type]` values above 0
/// (`user_objects.h:77`: a sphere 1, a capsule or cylinder 2, an ellipsoid or
/// box 3, a signed distance field none), a place neither the site nor its
/// class writes being MuJoCo's 0.005 (`site_size`).
fn check_site_size(site: &MjcfSite, site_type: GeomType) -> Result<()> {
    let size = written_over(site.size, [0.005; 3]);
    if let Some(v) = size.iter().find(|v| !v.is_finite()) {
        return Err(MjcfError::non_finite("site", "size", *v));
    }
    let places = match site_type {
        GeomType::Sphere => 1,
        GeomType::Capsule | GeomType::Cylinder => 2,
        GeomType::Ellipsoid | GeomType::Box => 3,
        _ => 0,
    };
    if let Some(i) = (0..places).find(|&i| size[i] <= 0.0) {
        return Err(MjcfError::invalid_value(
            "site",
            "size",
            format!("size {i} must be positive"),
        ));
    }
    Ok(())
}

/// MuJoCo's site size: 0.005 in each place (`mjs_defaultSite`,
/// `user_init.c:159`), the numbers written, the site's over its class's, in
/// the first places.
fn site_size(site: &MjcfSite) -> Vector3<f64> {
    Vector3::from(written_over(site.size, [0.005; 3]))
}

/// A `fromto` made into a position, an orientation (w, x, y, z) and a size as
/// MuJoCo 3.5.0's compiler makes them for a geom (`mjCGeom::Compile`,
/// `user_objects.cc:3677-3715`) and a site (`mjCSite::Compile`, `:3937-3971`), in
/// the frame the element is written in: on a capsule, cylinder, ellipsoid or box
/// only, and not beside a `pos` other than 0; half the segment's length in
/// `size[1]` (in `size[2]` for an ellipsoid or a box, whose `size[1]` takes
/// `size[0]`), its middle the position, and the turn of z onto `from − to` the
/// orientation. Points less than 1e-7 apart are refused: `normvec` gives their
/// segment a norm of 0. `Err` holds the refusal's reason.
fn fromto_pose(
    fromto: [f64; 6],
    geom_type: GeomType,
    pos: Option<Vector3<f64>>,
    mut size: Vector3<f64>,
) -> std::result::Result<(Vector3<f64>, [f64; 4], Vector3<f64>), &'static str> {
    if !matches!(
        geom_type,
        GeomType::Capsule | GeomType::Cylinder | GeomType::Ellipsoid | GeomType::Box
    ) {
        return Err("fromto requires a capsule, cylinder, ellipsoid or box");
    }
    if pos.is_some_and(|p| p.x != 0.0 || p.y != 0.0 || p.z != 0.0) {
        return Err("both pos and fromto defined");
    }
    let mut vec = [
        fromto[0] - fromto[3],
        fromto[1] - fromto[4],
        fromto[2] - fromto[5],
    ];
    let half = normvec(&mut vec) / 2.0;
    if half < MJ_EPS {
        return Err("fromto points too close");
    }
    size[1] = half;
    if matches!(geom_type, GeomType::Ellipsoid | GeomType::Box) {
        size[2] = size[1];
        size[1] = size[0];
    }
    // MuJoCo's arithmetic (`(a + b) / 2`), which `f64::midpoint` is not near
    // overflow.
    #[allow(clippy::manual_midpoint)]
    let middle = Vector3::new(
        (fromto[0] + fromto[3]) / 2.0,
        (fromto[1] + fromto[4]) / 2.0,
        (fromto[2] + fromto[5]) / 2.0,
    );
    Ok((middle, z2quat(&vec), size))
}

/// A site's `fromto` made into its `pos`, `quat` and `size` (`fromto_pose`), in
/// place of any other orientation the site or its class gives.
pub fn resolve_site_fromto(site: &mut MjcfSite) -> Result<()> {
    let Some(fromto) = site.fromto.take() else {
        return Ok(());
    };
    let (pos, quat, size) = fromto_pose(fromto, site_geom_type(site)?, site.pos, site_size(site))
        .map_err(|what| MjcfError::invalid_value("site", "fromto", what))?;
    site.pos = Some(pos);
    site.quat = Some(Vector4::new(quat[0], quat[1], quat[2], quat[3]));
    site.size = Some(Prefix::full([size.x, size.y, size.z]));
    site.euler = None;
    site.axisangle = None;
    site.xyaxes = None;
    site.zaxis = None;
    Ok(())
}

/// A geom's `fromto` made into its `pos`, `quat` and `size` (`fromto_pose`),
/// in its own frame, once its class is applied and before a frame, fusion,
/// its mass or its arrays take it (its class's `pos` counts: an element
/// starts from its class's values), in place
/// of any other orientation the geom or its class gives (MuJoCo never resolves
/// an alternative beside a `fromto`). A geom that also names a mesh (a name
/// that is not empty) is refused after, as MuJoCo refuses it
/// (`user_objects.cc:3727-3730`). It starts from the geom's size over its
/// class's, 0 in a place neither writes (MuJoCo's default), and keeps what
/// MuJoCo keeps: a capsule's or cylinder's radius and third value, a box's or
/// ellipsoid's first.
///
/// # Errors
///
/// [`MjcfError::NonFinite`] on `fromto` for a value that is not finite (a
/// document's is refused when it is read; this one was built in code);
/// [`MjcfError::InvalidValue`] on `fromto` for each refusal above.
pub fn resolve_geom_fromto(geom: &mut MjcfGeom) -> Result<()> {
    let Some(fromto) = geom.fromto.take() else {
        return Ok(());
    };
    if let Some(v) = fromto.iter().find(|v| !v.is_finite()) {
        return Err(MjcfError::non_finite("geom", "fromto", *v));
    }
    let refuse = |what: &str| MjcfError::invalid_value("geom", "fromto", what);
    let (pos, quat, size) = fromto_pose(
        fromto,
        convert_geom_type(geom.geom_type),
        geom.pos,
        Vector3::from(written_over(geom.size, [0.0; 3])),
    )
    .map_err(refuse)?;
    // An empty name names no mesh (MuJoCo resolves a non-empty one only).
    if geom.mesh.as_deref().is_some_and(|m| !m.is_empty()) {
        return Err(refuse("fromto cannot be used with mesh geom"));
    }
    geom.pos = Some(pos);
    geom.quat = Some(Vector4::new(quat[0], quat[1], quat[2], quat[3]));
    geom.size = Some(Prefix::full([size.x, size.y, size.z]));
    geom.euler = None;
    geom.axisangle = None;
    geom.xyaxes = None;
    geom.zaxis = None;
    Ok(())
}

/// Compute mass of a single geom.
///
/// For mesh geoms, accepts pre-computed mesh properties to avoid redundant
/// `compute_mesh_inertia()` calls (see `MeshProps`).
pub fn compute_geom_mass(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> f64 {
    // As validated: every place the type reads is above 0 (`check_geom_size`).
    let size = written_over(geom.size, [0.0; 3]);
    if let Some(mass) = geom.mass {
        return mass;
    }

    // Shell inertia for primitive geoms (not mesh)
    if geom.shellinertia == Some(true)
        && geom.geom_type.unwrap_or(MjcfGeomType::Sphere) != MjcfGeomType::Mesh
    {
        return compute_geom_shell_mass(geom);
    }

    let volume = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            (4.0 / 3.0) * std::f64::consts::PI * r.powi(3)
        }
        MjcfGeomType::Box => {
            let x = size[0];
            let y = size[1];
            let z = size[2];
            8.0 * x * y * z // size is half-extents
        }
        MjcfGeomType::Capsule => {
            let r = size[0];
            let h = size[1];
            std::f64::consts::PI * r.powi(2) * (h * 2.0 + (4.0 / 3.0) * r)
        }
        MjcfGeomType::Cylinder => {
            let r = size[0];
            let h = size[1];
            std::f64::consts::PI * r.powi(2) * h * 2.0
        }
        MjcfGeomType::Mesh => {
            if let Some(&(volume, _, _)) = mesh_props {
                volume.abs()
            } else {
                0.001
            }
        }
        MjcfGeomType::Ellipsoid => {
            let a = size[0];
            let b = size[1];
            let c = size[2];
            (4.0 / 3.0) * std::f64::consts::PI * a * b * c
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
    // As validated: every place the type reads is above 0 (`check_geom_size`).
    let size = written_over(geom.size, [0.0; 3]);
    // Shell inertia for primitive geoms
    if geom.shellinertia == Some(true)
        && geom.geom_type.unwrap_or(MjcfGeomType::Sphere) != MjcfGeomType::Mesh
    {
        return compute_geom_shell_inertia(geom);
    }

    let mass = compute_geom_mass(geom, mesh_props);

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            let i = 0.4 * mass * r.powi(2); // (2/5) m r²
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box => {
            // Full dimensions (size is half-extents)
            let x = size[0] * 2.0;
            let y = size[1] * 2.0;
            let z = size[2] * 2.0;
            let c = mass / 12.0;
            Matrix3::from_diagonal(&Vector3::new(
                c * (y * y + z * z),
                c * (x * x + z * z),
                c * (x * x + y * y),
            ))
        }
        MjcfGeomType::Cylinder => {
            let r = size[0];
            let h = size[1] * 2.0; // Full height
            // Solid cylinder about center
            let ix = mass * (3.0 * r.powi(2) + h.powi(2)) / 12.0;
            let iz = 0.5 * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Capsule => {
            // Exact capsule inertia (cylinder + two hemispheres)
            // Reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
            let r = size[0];
            let h = size[1] * 2.0; // Cylinder height (not including caps)

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

            // Hemisphere inertia about flat face center (= sphere center).
            // For any axis through the sphere center: I = 2/5 * m * r².
            let i_hemi_flat = 0.4 * m_hemi * r.powi(2);

            // Hemisphere COM is at 3r/8 from the flat face along the capsule axis.
            let com_offset = (3.0 / 8.0) * r;

            // Hemisphere transverse inertia about its own COM:
            // I_com = I_flat - m * com_offset²  (parallel axis theorem, reverse)
            let i_hemi_com = i_hemi_flat - m_hemi * com_offset.powi(2);

            // Distance from capsule center to hemisphere COM
            let d = h / 2.0 + com_offset;

            // Parallel axis theorem: shift from hemisphere COM to capsule center
            let i_hemi_x = i_hemi_com + m_hemi * d.powi(2);
            // Axial: hemisphere Izz = 2/5 * m * r², no parallel axis (on-axis)
            let i_hemi_z = i_hemi_flat;

            // Total: cylinder + 2 hemispheres
            let ix = i_cyl_x + 2.0 * i_hemi_x;
            let iz = i_cyl_z + 2.0 * i_hemi_z;

            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Ellipsoid => {
            // Ellipsoid inertia: I_x = (1/5) m (b² + c²), etc.
            let a = size[0];
            let b = size[1];
            let c = size[2];
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

/// Compute shell (surface-area-based) mass of a primitive geom.
///
/// Mass = density × surface_area.
fn compute_geom_shell_mass(geom: &MjcfGeom) -> f64 {
    // As validated: every place the type reads is above 0 (`check_geom_size`).
    let size = written_over(geom.size, [0.0; 3]);
    let density = geom.density.unwrap_or(1000.0);
    let sa = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            4.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Box => {
            let a = size[0];
            let b = size[1];
            let c = size[2];
            8.0 * (a * b + a * c + b * c)
        }
        MjcfGeomType::Cylinder => {
            let r = size[0];
            let h = size[1] * 2.0;
            2.0 * std::f64::consts::PI * r * h + 2.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Capsule => {
            let r = size[0];
            let h = size[1] * 2.0;
            2.0 * std::f64::consts::PI * r * h + 4.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Ellipsoid => {
            let a = size[0];
            let b = size[1];
            let c = size[2];
            ellipsoid_surface_area(a, b, c)
        }
        _ => 0.001,
    };

    density * sa
}

/// Knud Thomsen approximation for ellipsoid surface area.
/// SA ≈ 4π × ((a^p b^p + a^p c^p + b^p c^p) / 3)^(1/p), p ≈ 1.6075.
fn ellipsoid_surface_area(a: f64, b: f64, c: f64) -> f64 {
    let p = 1.6075;
    let ap = a.powf(p);
    let bp = b.powf(p);
    let cp = c.powf(p);
    let inner = (ap * bp + ap * cp + bp * cp) / 3.0;
    4.0 * std::f64::consts::PI * inner.powf(1.0 / p)
}

/// Compute shell inertia tensor of a primitive geom.
fn compute_geom_shell_inertia(geom: &MjcfGeom) -> Matrix3<f64> {
    // As validated: every place the type reads is above 0 (`check_geom_size`).
    let size = written_over(geom.size, [0.0; 3]);
    let mass = compute_geom_shell_mass(geom);

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            let i = (2.0 / 3.0) * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box => {
            let a = size[0];
            let b = size[1];
            let c = size[2];
            let density = if let Some(m) = geom.mass {
                m / (8.0 * (a * b + a * c + b * c))
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_box_inertia(a, b, c, density)
        }
        MjcfGeomType::Cylinder => {
            let r = size[0];
            let h = size[1] * 2.0;
            let density = if let Some(m) = geom.mass {
                let sa =
                    2.0 * std::f64::consts::PI * r * h + 2.0 * std::f64::consts::PI * r.powi(2);
                m / sa
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_cylinder_inertia(r, h, density)
        }
        MjcfGeomType::Capsule => {
            let r = size[0];
            let h = size[1] * 2.0;
            let density = if let Some(m) = geom.mass {
                let sa =
                    2.0 * std::f64::consts::PI * r * h + 4.0 * std::f64::consts::PI * r.powi(2);
                m / sa
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_capsule_inertia(r, h, density)
        }
        MjcfGeomType::Ellipsoid => {
            let a = size[0];
            let b = size[1];
            let c = size[2];
            shell_ellipsoid_inertia(a, b, c, mass)
        }
        _ => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)),
    }
}

/// Box shell inertia via face decomposition + PAT.
fn shell_box_inertia(a: f64, b: f64, c: f64, density: f64) -> Matrix3<f64> {
    // ±x faces (area 4bc each)
    let m_x = density * 4.0 * b * c;
    let ix_x = m_x * (b * b + c * c) / 3.0;
    let iy_x = m_x * (a * a + c * c / 3.0);
    let iz_x = m_x * (a * a + b * b / 3.0);

    // ±y faces (area 4ac each)
    let m_y = density * 4.0 * a * c;
    let ix_y = m_y * (b * b + c * c / 3.0);
    let iy_y = m_y * (a * a + c * c) / 3.0;
    let iz_y = m_y * (b * b + a * a / 3.0);

    // ±z faces (area 4ab each)
    let m_z = density * 4.0 * a * b;
    let ix_z = m_z * (b * b / 3.0 + c * c);
    let iy_z = m_z * (a * a / 3.0 + c * c);
    let iz_z = m_z * (a * a + b * b) / 3.0;

    Matrix3::from_diagonal(&Vector3::new(
        2.0 * (ix_x + ix_y + ix_z),
        2.0 * (iy_x + iy_y + iy_z),
        2.0 * (iz_x + iz_y + iz_z),
    ))
}

/// Cylinder shell inertia: lateral shell + 2 disk caps.
fn shell_cylinder_inertia(r: f64, h: f64, density: f64) -> Matrix3<f64> {
    let pi = std::f64::consts::PI;

    // Lateral shell
    let m_lat = density * 2.0 * pi * r * h;
    let ix_lat = m_lat * (r * r / 2.0 + h * h / 12.0);
    let iz_lat = m_lat * r * r;

    // End cap (thin disk), each
    let m_cap = density * pi * r * r;
    let ix_cap_own = m_cap * r * r / 4.0;
    let iz_cap_own = m_cap * r * r / 2.0;
    let ix_cap = ix_cap_own + m_cap * (h / 2.0).powi(2);

    Matrix3::from_diagonal(&Vector3::new(
        ix_lat + 2.0 * ix_cap,
        ix_lat + 2.0 * ix_cap,
        iz_lat + 2.0 * iz_cap_own,
    ))
}

/// Capsule shell inertia: open cylinder lateral + 2 hemisphere shells.
fn shell_capsule_inertia(r: f64, h: f64, density: f64) -> Matrix3<f64> {
    let pi = std::f64::consts::PI;

    // Lateral (open cylinder — no end caps)
    let m_lat = density * 2.0 * pi * r * h;
    let ix_lat = m_lat * (r * r / 2.0 + h * h / 12.0);
    let iz_lat = m_lat * r * r;

    // Hemisphere shell (each)
    let m_hemi = density * 2.0 * pi * r * r;
    // Inertia about own COM (COM is at r/2 from equator)
    let ix_hemi_own = m_hemi * 5.0 * r * r / 12.0;
    let iz_hemi = 2.0 / 3.0 * m_hemi * r * r;
    // PAT: distance from hemisphere COM to capsule center
    let d = h / 2.0 + r / 2.0;
    let ix_hemi = ix_hemi_own + m_hemi * d * d;

    Matrix3::from_diagonal(&Vector3::new(
        ix_lat + 2.0 * ix_hemi,
        ix_lat + 2.0 * ix_hemi,
        iz_lat + 2.0 * iz_hemi,
    ))
}

/// Ellipsoid shell inertia via numerical quadrature on parametric surface.
fn shell_ellipsoid_inertia(a: f64, b: f64, c: f64, mass: f64) -> Matrix3<f64> {
    let n_theta = 64_usize;
    let n_phi = 128_usize;

    let (theta_nodes, theta_weights) = gauss_legendre_mapped(n_theta, 0.0, std::f64::consts::PI);
    let (phi_nodes, phi_weights) = gauss_legendre_mapped(n_phi, 0.0, 2.0 * std::f64::consts::PI);

    let mut sa_num = 0.0;
    let mut int_xx = 0.0;
    let mut int_yy = 0.0;
    let mut int_zz = 0.0;

    for i in 0..n_theta {
        let theta = theta_nodes[i];
        let wt = theta_weights[i];
        let sin_t = theta.sin();
        let cos_t = theta.cos();

        for j in 0..n_phi {
            let phi = phi_nodes[j];
            let wp = phi_weights[j];
            let sin_p = phi.sin();
            let cos_p = phi.cos();

            // Surface element magnitude
            let e = (c * c * sin_t * sin_t * (b * b * cos_p * cos_p + a * a * sin_p * sin_p)
                + a * a * b * b * cos_t * cos_t)
                .sqrt();
            let ds = sin_t * e * wt * wp;

            sa_num += ds;
            int_xx += (a * sin_t * cos_p).powi(2) * ds;
            int_yy += (b * sin_t * sin_p).powi(2) * ds;
            int_zz += (c * cos_t).powi(2) * ds;
        }
    }

    // Normalize: I_x = m × (⟨y²⟩ + ⟨z²⟩) where ⟨x²⟩ = ∫x² dS / SA
    let ix = mass * (int_yy + int_zz) / sa_num;
    let iy = mass * (int_xx + int_zz) / sa_num;
    let iz = mass * (int_xx + int_yy) / sa_num;

    Matrix3::from_diagonal(&Vector3::new(ix, iy, iz))
}

/// Gauss-Legendre quadrature nodes and weights mapped to [lo, hi].
fn gauss_legendre_mapped(n: usize, lo: f64, hi: f64) -> (Vec<f64>, Vec<f64>) {
    let (std_nodes, std_weights) = gauss_legendre_standard(n);
    let mid = f64::midpoint(lo, hi);
    let half = (hi - lo) / 2.0;
    let nodes: Vec<f64> = std_nodes.iter().map(|&x| mid + half * x).collect();
    let weights: Vec<f64> = std_weights.iter().map(|&w| w * half).collect();
    (nodes, weights)
}

/// Compute Gauss-Legendre nodes and weights on [-1, 1] via Newton iteration.
// `n` is the quadrature order (typically <= 64); `as f64` is exact below 2^52.
#[allow(clippy::cast_precision_loss)]
fn gauss_legendre_standard(n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut nodes = vec![0.0; n];
    let mut weights = vec![0.0; n];
    let n_f = n as f64;

    for i in 0..n {
        // Initial guess using Chebyshev approximation
        let mut x = (std::f64::consts::PI * (4.0 * i as f64 + 3.0) / (4.0 * n_f + 2.0)).cos();

        // Newton's method to find root of P_n(x)
        for _ in 0..100 {
            let (p, dp) = legendre_pd(n, x);
            let dx = -p / dp;
            x += dx;
            if dx.abs() < 1e-15 {
                break;
            }
        }

        nodes[i] = x;
        let (_, dp) = legendre_pd(n, x);
        weights[i] = 2.0 / ((1.0 - x * x) * dp * dp);
    }

    (nodes, weights)
}

/// Evaluate Legendre polynomial P_n(x) and its derivative P'_n(x).
// Recurrence index `k` is the quadrature order, well below 2^52.
#[allow(clippy::cast_precision_loss)]
fn legendre_pd(n: usize, x: f64) -> (f64, f64) {
    let mut p0 = 1.0;
    let mut p1 = x;
    for k in 1..n {
        let k_f = k as f64;
        let p2 = ((2.0 * k_f + 1.0) * x * p1 - k_f * p0) / (k_f + 1.0);
        p0 = p1;
        p1 = p2;
    }
    let n_f = n as f64;
    let dp = n_f * (x * p1 - p0) / (x * x - 1.0);
    (p1, dp)
}

/// A geom's size as the model stores it, from its size as resolved (written
/// over its class's, 0 in a place neither writes). A mesh's, a height
/// field's or an SDF's is 0.1 in each place, not MuJoCo's from its asset
/// (ledger L354); the caller replaces a height field's with its asset's.
pub fn geom_size_to_vec3(size: [f64; 3], geom_type: GeomType) -> Vector3<f64> {
    match geom_type {
        // The radius in each place, as before (MuJoCo keeps the other two as
        // resolved; the census compares the radius only).
        GeomType::Sphere => Vector3::new(size[0], size[0], size[0]),
        // As resolved, as MuJoCo keeps it (a capsule's or cylinder's third
        // value too).
        GeomType::Box
        | GeomType::Capsule
        | GeomType::Cylinder
        | GeomType::Ellipsoid
        | GeomType::Plane => Vector3::from(size),
        _ => Vector3::new(0.1, 0.1, 0.1),
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::compute_geom_inertia;
    use crate::types::{MjcfGeom, MjcfGeomType, Prefix};
    use nalgebra::Vector3;

    /// A plane's size as written, as MuJoCo 3.5.0 keeps it (`geom_size`,
    /// measured on the S21 oracle; book Rigid-loading L14).
    #[test]
    fn a_planes_size_is_kept_as_written() {
        for (size, mujoco) in [("2 3 0.5", [2.0, 3.0, 0.5]), ("0 0 1", [0.0, 0.0, 1.0])] {
            let model = crate::builder::load_model(&format!(
                r#"<mujoco><worldbody><geom type="plane" size="{size}"/></worldbody></mujoco>"#
            ))
            .expect("loads");
            let got = model.geom_size[0];
            assert_eq!(
                [got.x, got.y, got.z].map(f64::to_bits),
                mujoco.map(f64::to_bits),
                "plane size=\"{size}\": {got:?}"
            );
        }
    }

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
            size: Prefix::new(&[0.1, 0.5]), // radius=0.1, half-height=0.5
            fromto: None,
            density: Some(1000.0),
            mass: None,
            friction: Some(Prefix::full([1.0, 0.005, 0.0001])),
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
            user: vec![],
            shellinertia: None,
            plugin: None,
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
