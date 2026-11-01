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

/// Compute mass of a single geom, as MuJoCo 3.5.0's compiler computes it
/// (`mjCGeom::Compile`, `user_objects.cc:3780-3800`): a written mass of 0 is
/// 0; any other written mass is the geom's where its volume is above mjEPS
/// and 0 where it is not; with no mass written, a density of 0 gives 0 and
/// any other the density times the volume ([`geom_volume`]).
///
/// For mesh geoms, accepts pre-computed mesh properties to avoid redundant
/// `compute_mesh_inertia()` calls (see `MeshProps`); for a height field, its
/// asset's four sizes.
pub fn compute_geom_mass(
    geom: &MjcfGeom,
    mesh_props: Option<&MeshProps>,
    hfield_size: Option<&[f64; 4]>,
) -> f64 {
    let volume = geom_volume(geom, mesh_props, hfield_size);
    match geom.mass {
        // A volume that is not a number keeps the mass at 0, as MuJoCo's
        // comparison does.
        Some(mass) => {
            if volume > MJ_EPS {
                mass
            } else {
                0.0
            }
        }
        None => match geom.density.unwrap_or(1000.0) {
            0.0 => 0.0,
            density => density * volume,
        },
    }
}

/// A geom's volume, as MuJoCo 3.5.0 computes it (`mjCGeom::GetVolume`,
/// `user_objects.cc:3115-3189`), each product in MuJoCo's order: a height
/// field's is the box of the sizes MuJoCo gives it from its asset
/// ([`hfield_geom_size`]), a plane's 0. A mesh's is its own volume (book
/// Rigid-loading L35's), and a primitive geom's with `shellinertia` its
/// surface's area ([`shell_area`]).
fn geom_volume(
    geom: &MjcfGeom,
    mesh_props: Option<&MeshProps>,
    hfield_size: Option<&[f64; 4]>,
) -> f64 {
    use std::f64::consts::PI;
    let geom_type = geom.geom_type.unwrap_or(MjcfGeomType::Sphere);
    if geom.shellinertia == Some(true) && geom_type != MjcfGeomType::Mesh {
        return shell_area(geom, hfield_size);
    }
    let size = mass_sizes(geom, hfield_size);
    match geom_type {
        MjcfGeomType::Sphere => {
            let r = size[0];
            4.0 * PI * r * r * r / 3.0
        }
        MjcfGeomType::Capsule => {
            let (r, height) = (size[0], 2.0 * size[1]);
            PI * (r * r * height + 4.0 * r * r * r / 3.0)
        }
        MjcfGeomType::Cylinder => {
            let (r, height) = (size[0], 2.0 * size[1]);
            PI * r * r * height
        }
        MjcfGeomType::Ellipsoid => 4.0 * PI * size[0] * size[1] * size[2] / 3.0,
        MjcfGeomType::Box | MjcfGeomType::Hfield => size[0] * size[1] * size[2] * 8.0,
        MjcfGeomType::Mesh => mesh_props.map_or(0.001, |&(volume, _, _)| volume.abs()),
        // MuJoCo's is its mesh's volume, as a mesh geom's (L35's); kept.
        MjcfGeomType::Sdf => 0.001,
        MjcfGeomType::Plane => 0.0,
    }
}

/// The sizes MuJoCo 3.5.0 gives a height-field geom from its asset's four
/// (`user_objects.cc:3761-3764`), which its volume and inertia read: the
/// asset's two half-extents, and a quarter of its top plus half its base.
fn hfield_geom_size(asset: &[f64; 4]) -> [f64; 3] {
    [asset[0], asset[1], 0.25 * asset[2] + 0.5 * asset[3]]
}

/// The sizes MuJoCo 3.5.0 computes a geom's volume and inertia from: its size
/// as resolved (`check_geom_size` has validated every place its type reads),
/// or a height field's from its asset ([`hfield_geom_size`]).
fn mass_sizes(geom: &MjcfGeom, hfield_size: Option<&[f64; 4]>) -> [f64; 3] {
    if geom.geom_type == Some(MjcfGeomType::Hfield) {
        hfield_size.map_or([0.0; 3], hfield_geom_size)
    } else {
        written_over(geom.size, [0.0; 3])
    }
}

/// Compute inertia tensor of a single geom about its center (geom-local frame).
///
/// Returns a full 3×3 matrix. Primitive geoms produce diagonal tensors;
/// mesh geoms may have off-diagonal terms.
///
/// For mesh geoms, accepts pre-computed mesh properties to avoid redundant
/// `compute_mesh_inertia()` calls (see `MeshProps`); for a height field, its
/// asset's four sizes.
///
/// A primitive geom's is MuJoCo 3.5.0's (`mjCGeom::SetInertia`,
/// `user_objects.cc:3194-3413`), each product and quotient in MuJoCo's
/// order: a capsule's is its cylinder's plus its two hemispheres', each
/// taking the share of the mass its volume takes; a height field's is its
/// box's. A mesh's is its tetrahedra's (Mirtich 1996; book Rigid-loading
/// L35's), and a primitive geom's with `shellinertia` its surface's (ledger
/// L355's).
pub fn compute_geom_inertia(
    geom: &MjcfGeom,
    mesh_props: Option<&MeshProps>,
    hfield_size: Option<&[f64; 4]>,
) -> Matrix3<f64> {
    // MuJoCo leaves a geom of mass 0 with an inertia of 0 (it never computes
    // one, or computes one in proportion to the mass).
    let mass = compute_geom_mass(geom, mesh_props, hfield_size);
    if mass == 0.0 {
        return Matrix3::zeros();
    }
    let size = mass_sizes(geom, hfield_size);
    // Shell inertia for primitive geoms
    if geom.shellinertia == Some(true)
        && geom.geom_type.unwrap_or(MjcfGeomType::Sphere) != MjcfGeomType::Mesh
    {
        return compute_geom_shell_inertia(geom, hfield_size);
    }

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            let i = 2.0 * mass * r * r / 5.0;
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box | MjcfGeomType::Hfield => {
            let [s00, s11, s22] = size.map(|s| s * s);
            Matrix3::from_diagonal(&Vector3::new(
                mass * (s11 + s22) / 3.0,
                mass * (s00 + s22) / 3.0,
                mass * (s00 + s11) / 3.0,
            ))
        }
        MjcfGeomType::Cylinder => {
            let (r, height) = (size[0], 2.0 * size[1]);
            let ix = mass * (3.0 * r * r + height * height) / 12.0;
            let iz = mass * r * r / 2.0;
            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Capsule => {
            let (r, height) = (size[0], 2.0 * size[1]);
            // The two hemispheres' share of the mass, their volume's.
            let sphere_mass = mass * 4.0 * r / (4.0 * r + 3.0 * height);
            let cylinder_mass = mass - sphere_mass;
            // A sphere's, plus each hemisphere's displacement along the axis.
            let sphere_inertia = 2.0 * sphere_mass * r * r / 5.0;
            let ix = cylinder_mass * (3.0 * r * r + height * height) / 12.0
                + (sphere_inertia + sphere_mass * height * (3.0 * r + 2.0 * height) / 8.0);
            let iz = cylinder_mass * r * r / 2.0 + sphere_inertia;
            Matrix3::from_diagonal(&Vector3::new(ix, ix, iz))
        }
        MjcfGeomType::Ellipsoid => {
            let [s00, s11, s22] = size.map(|s| s * s);
            Matrix3::from_diagonal(&Vector3::new(
                mass * (s11 + s22) / 5.0,
                mass * (s00 + s22) / 5.0,
                mass * (s00 + s11) / 5.0,
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
        // MuJoCo's is its mesh's, as a mesh geom's (L35's); kept.
        MjcfGeomType::Sdf => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)),
        // MuJoCo's default arm.
        MjcfGeomType::Plane => Matrix3::zeros(),
    }
}

/// Compute shell (surface-area-based) mass of a primitive geom.
///
/// Mass = density × surface_area.
fn compute_geom_shell_mass(geom: &MjcfGeom, hfield_size: Option<&[f64; 4]>) -> f64 {
    geom.density.unwrap_or(1000.0) * shell_area(geom, hfield_size)
}

/// The surface area of a primitive geom with `shellinertia`, as MuJoCo
/// 3.5.0's volume reads it for a shell (`mjCGeom::GetVolume`,
/// `user_objects.cc:3115-3189`), each product in MuJoCo's order: an
/// ellipsoid's by Thomsen's approximation, a height field's its box's on
/// the sizes MuJoCo gives it from its asset ([`hfield_geom_size`]), a
/// plane's 0. An SDF's is kept at 0.001, as its volume is (MuJoCo's is its
/// mesh's).
fn shell_area(geom: &MjcfGeom, hfield_size: Option<&[f64; 4]>) -> f64 {
    use std::f64::consts::PI;
    let size = mass_sizes(geom, hfield_size);
    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            4.0 * PI * r * r
        }
        MjcfGeomType::Capsule => {
            let (r, height) = (size[0], 2.0 * size[1]);
            4.0 * PI * r * r + 2.0 * PI * r * height
        }
        MjcfGeomType::Cylinder => {
            let (r, height) = (size[0], 2.0 * size[1]);
            2.0 * PI * r * r + 2.0 * PI * r * height
        }
        MjcfGeomType::Ellipsoid => {
            let p = 1.6075;
            let tmp = (size[0] * size[1]).powf(p)
                + (size[1] * size[2]).powf(p)
                + (size[2] * size[0]).powf(p);
            4.0 * PI * (tmp / 3.0).powf(1.0 / p)
        }
        MjcfGeomType::Box | MjcfGeomType::Hfield => {
            8.0 * (size[0] * size[1] + size[1] * size[2] + size[2] * size[0])
        }
        MjcfGeomType::Mesh | MjcfGeomType::Sdf => 0.001,
        MjcfGeomType::Plane => 0.0,
    }
}

/// Compute shell inertia tensor of a primitive geom; a height field's is
/// its box's, on the sizes MuJoCo gives it from its asset.
fn compute_geom_shell_inertia(geom: &MjcfGeom, hfield_size: Option<&[f64; 4]>) -> Matrix3<f64> {
    let size = mass_sizes(geom, hfield_size);
    let mass = compute_geom_shell_mass(geom, hfield_size);

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = size[0];
            let i = (2.0 / 3.0) * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box | MjcfGeomType::Hfield => {
            let [a, b, c] = size;
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
        // As resolved, as MuJoCo keeps it (a sphere's places after its
        // radius, a capsule's or cylinder's third value too).
        GeomType::Sphere
        | GeomType::Box
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

    /// A sphere's size as resolved, as MuJoCo 3.5.0 keeps it (`geom_size`,
    /// measured on the census's unfused oracle; ledger L352): the radius, then whatever the
    /// element or its class writes after it, 0 where neither does.
    #[test]
    fn a_spheres_size_is_kept_as_resolved() {
        for (class, size, mujoco) in [
            ("", r#"size="0.1""#, [0.1, 0.0, 0.0]),
            ("", r#"size="0.1 0.2""#, [0.1, 0.2, 0.0]),
            ("", r#"size="0.1 0.2 0.3""#, [0.1, 0.2, 0.3]),
            (
                r#"<default><geom size="0.3 0.4 0.5"/></default>"#,
                r#"size="0.1""#,
                [0.1, 0.4, 0.5],
            ),
            (
                r#"<default><geom size="0.3 0.4 0.5"/></default>"#,
                "",
                [0.3, 0.4, 0.5],
            ),
        ] {
            let model = crate::builder::load_model(&format!(
                r#"<mujoco>{class}<worldbody><geom type="sphere" {size}/></worldbody></mujoco>"#
            ))
            .expect("loads");
            let got = model.geom_size[0];
            assert_eq!(
                [got.x, got.y, got.z].map(f64::to_bits),
                mujoco.map(f64::to_bits),
                "{class} {size}: {got:?}"
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

        let inertia = compute_geom_inertia(&geom, None, None);

        // Ix = Iy (axially symmetric) — diagonal elements (0,0) and (1,1)
        assert!((inertia[(0, 0)] - inertia[(1, 1)]).abs() < 1e-10);

        // Iz < Ix (thin cylinder is easier to spin about long axis)
        assert!(inertia[(2, 2)] < inertia[(0, 0)]);

        // All positive
        assert!(inertia[(0, 0)] > 0.0);
        assert!(inertia[(1, 1)] > 0.0);
        assert!(inertia[(2, 2)] > 0.0);
    }

    /// A primitive geom's inertia is MuJoCo 3.5.0's (`mjCGeom::SetInertia`,
    /// `user_objects.cc:3194-3413`), bit for bit on every platform (products
    /// and quotients only). Each row's moments are those MuJoCo compiled for a
    /// body holding that geom alone, which takes the geom's own whatever its
    /// pose (`InertiaFromGeom`, `:2175-2181`), on the census's unfused oracle
    /// (2026-11-01; the documents are in `tests/geom_masses.rs`): the geom's
    /// type, its size as resolved (a `fromto` capsule's), its mass or
    /// density, and a height field's asset sizes.
    // MuJoCo's values, printed to the last digit.
    #[allow(clippy::unreadable_literal)]
    #[test]
    fn a_geoms_inertia_is_mujocos() {
        type Row = (
            MjcfGeomType,
            &'static [f64],
            Option<f64>,
            Option<f64>,
            Option<[f64; 4]>,
            [f64; 3],
        );
        let rows: &[Row] = &[
            // val_sphere
            (
                MjcfGeomType::Sphere,
                &[0.05],
                Some(0.8),
                None,
                None,
                [
                    0.0008000000000000001,
                    0.0008000000000000001,
                    0.0008000000000000001,
                ],
            ),
            // val_sphere_density
            (
                MjcfGeomType::Sphere,
                &[0.07],
                None,
                None,
                None,
                [
                    0.002816039878873797,
                    0.002816039878873797,
                    0.002816039878873797,
                ],
            ),
            // v_sphere_mass_odd
            (
                MjcfGeomType::Sphere,
                &[0.13],
                Some(0.7),
                None,
                None,
                [0.004732, 0.004732, 0.004732],
            ),
            // val_box
            (
                MjcfGeomType::Box,
                &[0.1, 0.2, 0.3],
                Some(2.0),
                None,
                None,
                [
                    0.08666666666666667,
                    0.06666666666666667,
                    0.03333333333333334,
                ],
            ),
            // val_box_density
            (
                MjcfGeomType::Box,
                &[0.1, 0.2, 0.3],
                None,
                None,
                None,
                [2.0800000000000005, 1.6000000000000003, 0.8000000000000003],
            ),
            // v_box_mass_odd
            (
                MjcfGeomType::Box,
                &[0.13, 0.07, 0.29],
                Some(1.3),
                None,
                None,
                [
                    0.038566666666666666,
                    0.04376666666666667,
                    0.009446666666666667,
                ],
            ),
            // v_small_box_mass_static
            (
                MjcfGeomType::Box,
                &[2e-05, 2e-05, 2e-05],
                Some(2.0),
                None,
                None,
                [
                    5.333333333333335e-10,
                    5.333333333333335e-10,
                    5.333333333333335e-10,
                ],
            ),
            // val_cylinder
            (
                MjcfGeomType::Cylinder,
                &[0.1, 0.3],
                None,
                None,
                None,
                [0.6126105674500096, 0.6126105674500096, 0.0942477796076938],
            ),
            // val_cylinder_mass
            (
                MjcfGeomType::Cylinder,
                &[0.1, 0.3],
                Some(1.7),
                None,
                None,
                [0.05525, 0.05525, 0.0085],
            ),
            // v_cylinder_density
            (
                MjcfGeomType::Cylinder,
                &[0.07, 0.21],
                None,
                Some(900.0),
                None,
                [0.0926653122641908, 0.0926653122641908, 0.014256201886798589],
            ),
            // v_cylinder_small
            (
                MjcfGeomType::Cylinder,
                &[0.01, 0.01],
                None,
                None,
                None,
                [
                    3.6651914291880924e-07,
                    3.6651914291880924e-07,
                    3.141592653589794e-07,
                ],
            ),
            // val_capsule
            (
                MjcfGeomType::Capsule,
                &[0.2, 0.5],
                None,
                None,
                None,
                [23.155632252059178, 23.155632252059178, 3.049439269084494],
            ),
            // val_capsule_mass
            (
                MjcfGeomType::Capsule,
                &[0.2, 0.5],
                Some(3.0),
                None,
                None,
                [0.436421052631579, 0.436421052631579, 0.057473684210526316],
            ),
            // v_capsule_density
            (
                MjcfGeomType::Capsule,
                &[0.03, 0.17],
                None,
                Some(1100.0),
                None,
                [
                    0.01454080190687994,
                    0.01454080190687994,
                    0.0005206435841088221,
                ],
            ),
            // v_capsule_fromto
            (
                MjcfGeomType::Capsule,
                &[0.05, 0.3],
                None,
                None,
                None,
                [
                    0.19785488732920722,
                    0.19785488732920722,
                    0.006414085001079163,
                ],
            ),
            // val_ellipsoid
            (
                MjcfGeomType::Ellipsoid,
                &[0.04, 0.03, 0.02],
                None,
                None,
                None,
                [
                    2.6138050877867084e-05,
                    4.021238596594936e-05,
                    5.026548245743669e-05,
                ],
            ),
            // v_ellipsoid_mass
            (
                MjcfGeomType::Ellipsoid,
                &[0.3, 0.2, 0.1],
                Some(2.5),
                None,
                None,
                [0.025000000000000005, 0.05, 0.065],
            ),
            // hfield_moving_mass
            (
                MjcfGeomType::Hfield,
                &[],
                None,
                None,
                Some([1.0, 1.0, 1.0, 0.5]),
                [1666.6666666666667, 1666.6666666666667, 2666.6666666666665],
            ),
            // v_hfield_s3
            (
                MjcfGeomType::Hfield,
                &[],
                None,
                None,
                Some([2.0, 3.0, 0.4, 0.2]),
                [28928.000000000004, 12928.000000000002, 41600.00000000001],
            ),
            // v_hfield_density
            (
                MjcfGeomType::Hfield,
                &[],
                None,
                Some(7.0),
                Some([0.5, 0.25, 0.3, 0.1]),
                [
                    0.022786458333333332,
                    0.07747395833333333,
                    0.09114583333333333,
                ],
            ),
            // v_hfield_mass
            (
                MjcfGeomType::Hfield,
                &[],
                Some(3.0),
                None,
                Some([0.5, 0.25, 0.3, 0.1]),
                [0.078125, 0.265625, 0.3125],
            ),
            // v_capsule_orders
            (
                MjcfGeomType::Capsule,
                &[0.03, 0.13],
                None,
                None,
                None,
                [
                    0.006589522006831126,
                    0.006589522006831126,
                    0.00037152474721352893,
                ],
            ),
            // v_ellipsoid_orders
            (
                MjcfGeomType::Ellipsoid,
                &[0.01, 0.01, 0.04],
                None,
                None,
                None,
                [
                    5.696754678509493e-06,
                    5.696754678509493e-06,
                    6.702064327658226e-07,
                ],
            ),
        ];
        let mut wrong = Vec::new();
        for &(geom_type, size, mass, density, hfield, mujoco) in rows {
            let geom = MjcfGeom {
                geom_type: Some(geom_type),
                size: Prefix::new(size),
                mass,
                density,
                ..MjcfGeom::default()
            };
            let i = compute_geom_inertia(&geom, None, hfield.as_ref());
            let got = [i[(0, 0)], i[(1, 1)], i[(2, 2)]];
            if got.map(f64::to_bits) != mujoco.map(f64::to_bits) {
                wrong.push(format!(
                    "{geom_type:?} {size:?}: {got:?} where MuJoCo's is {mujoco:?}"
                ));
            }
        }
        assert!(
            wrong.is_empty(),
            "{} of {}:\n{}",
            wrong.len(),
            rows.len(),
            wrong.join("\n")
        );
    }

    /// A shell ellipsoid's mass is MuJoCo 3.5.0's: its surface by Thomsen's
    /// approximation, as `GetVolume` takes it (`user_objects.cc:3163-3171`),
    /// times the density 1000 (the census's unfused oracle, 2026-11-01); for
    /// `0.05 0.1 0.13` main's order of the same sum rounded otherwise. The
    /// area takes `powf`, which a platform's libm may round otherwise: bit for
    /// bit on macOS arm64, where the oracle ran, within a relative 1e-12
    /// elsewhere.
    #[test]
    fn a_shell_ellipsoids_mass_is_mujocos() {
        for (size, mujoco) in [
            ([0.3, 0.2, 0.1], 489.719_333_337_173_1_f64),
            ([0.05, 0.1, 0.13], 108.042_583_619_473_8_f64),
        ] {
            let geom = MjcfGeom {
                geom_type: Some(MjcfGeomType::Ellipsoid),
                size: Prefix::new(&size),
                shellinertia: Some(true),
                ..MjcfGeom::default()
            };
            let got = super::compute_geom_mass(&geom, None, None);
            if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
                assert_eq!(got.to_bits(), mujoco.to_bits(), "{size:?}: {got}");
            } else {
                assert!((got - mujoco).abs() <= 1e-12 * mujoco, "{size:?}: {got}");
            }
        }
    }
}
