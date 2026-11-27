//! Geometry and site processing.
//!
//! Converts MJCF `<geom>` and `<site>` elements into `Model` arrays.
//! Includes helpers for computing mass, inertia, fromto poses, and
//! size vector conversion for each geom type.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3, Vector4};
use sim_core::GeomType;
use std::borrow::Cow;

use super::fluid::compute_geom_fluid;
use super::frame::frame_accum_child;
use super::mass::geom_mesh_id;
use super::mesh::MeshProps;
use super::orientation::{
    MJ_EPS, normvec, resolve_orientation, rot_vec_quat, site_orientation, z2quat,
};
use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder, written_over};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{FluidShape, MjcfBody, MjcfGeom, MjcfGeomType, MjcfModel, MjcfSite, Prefix};

impl ModelBuilder {
    /// Build one of body `body_id`'s geoms into the model, as MuJoCo 3.5.0's
    /// `mjCGeom::Compile` compiles it. A sphere, capsule, cylinder, ellipsoid
    /// or box that names a mesh is first fitted to it ([`Self::fit`]);
    /// returns the geom as built, fitted or not, which the body's mass reads.
    pub(crate) fn process_geom<'g>(
        &mut self,
        geom: &'g MjcfGeom,
        body_id: usize,
    ) -> Result<Cow<'g, MjcfGeom>> {
        let fit = self.fit(geom);
        let geom = match &fit {
            Some(fit) => Cow::Owned(fit.applied_to(geom)),
            None => Cow::Borrowed(geom),
        };
        let geom_id = self.geom_type.len();
        let geom_type = convert_geom_type(geom.geom_type);
        let at = || {
            Location::nth(ElementKind::Geom, geom.name.as_deref(), geom_id).within_nth(
                ElementKind::Body,
                self.body_name.get(body_id).and_then(Option::as_deref),
                body_id,
            )
        };

        let (geom_mesh_ref, geom_hfield_ref) = self
            .resolve_geom_asset_links(&geom, geom_type)
            .map_err(|e| e.with_location_if_unknown(at()))?;

        // A geom's `fromto` is already its `pos`, `quat` and `size`
        // (`resolve_geom_fromto`).
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

        // A mesh or SDF geom, and a primitive fitted to its mesh, take the
        // mesh's frame, to which `process_mesh` moved the mesh, into their
        // own: the fitted shape's centre (a mesh or SDF geom's is the
        // origin) turned by the mesh's turn and added to its centre, then
        // composed with the geom's own pose (MuJoCo 3.5.0's `mjCGeom::Compile`,
        // `user_objects.cc:3748-3754`). A mesh or SDF geom's size along each
        // axis is the larger magnitude of the mesh's bounds there
        // (`:3765-3769`); a fitted geom's is its fit's, which MuJoCo then
        // checks as any written size (`checksize`, `:3757`).
        let mesh_frame = match (geom_mesh_ref, &fit) {
            (Some(mesh), _) => Some((mesh, [0.0; 3])),
            (None, Some(fit)) => Some((fit.mesh, fit.center)),
            (None, None) => None,
        };
        let (pos, quat) = match mesh_frame {
            Some((mesh, center)) => {
                let props = &self.mesh_props[mesh];
                let mut mesh_pos =
                    Vector3::from(rot_vec_quat(center, props.quat)) + Vector3::from(props.com);
                let [w, x, y, z] = props.quat;
                let mut mesh_quat = UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z));
                frame_accum_child(&pos, &quat, &mut mesh_pos, &mut mesh_quat);
                (mesh_pos, mesh_quat)
            }
            None => (pos, quat),
        };
        // The `<frame>` holding the geom, composed last
        // (`user_objects.cc:3831-3834`).
        let (pos, quat) = match geom.frame {
            Some((frame_pos, q)) => {
                let frame_quat =
                    UnitQuaternion::new_unchecked(Quaternion::new(q[0], q[1], q[2], q[3]));
                let (mut pos, mut quat) = (pos, quat);
                frame_accum_child(&frame_pos, &frame_quat, &mut pos, &mut quat);
                (pos, quat)
            }
            None => (pos, quat),
        };
        let size = match geom_mesh_ref {
            Some(mesh) => Vector3::from(self.mesh_data[mesh].origin_box()),
            None => size,
        };
        if let Some(fit) = &fit {
            crate::validation::checksize(geom.geom_type.unwrap_or(MjcfGeomType::Sphere), fit.size)
                .map_err(|e| e.at(at()))?;
        }

        // A height-field geom's size is its asset's, as MuJoCo 3.5.0 sets it
        // ([`hfield_geom_size`]), whatever the geom writes.
        let size = if let Some(hfield_id) = geom_hfield_ref {
            Vector3::from(hfield_geom_size(&self.hfield_size[hfield_id]))
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

        // As resolved; a value other than 1, 3, 4 or 6 is refused once every
        // geom is built (`check_bodies`), as MuJoCo 3.5.0 refuses it.
        self.geom_condim.push(geom.condim.unwrap_or(3));
        // A mesh or SDF geom's inertia is its mesh's: `shellinertia` on one
        // is refused once every geom is built (`check_bodies`), where MuJoCo
        // 3.5.0 refuses it.
        self.geom_shellinertia_on_mesh.push(
            geom.shellinertia == Some(true) && matches!(geom_type, GeomType::Mesh | GeomType::Sdf),
        );
        // An SDF geom that names no mesh is refused once every geom is built
        // (`check_bodies`), after its condim, as MuJoCo 3.5.0 refuses it
        // (ledger L357); a mesh geom's is refused as its mesh is looked up.
        self.geom_sdf_names_no_mesh
            .push(geom_type == GeomType::Sdf && geom.mesh.as_deref().is_none_or(str::is_empty));
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
        // A shape of its own is given in code, never by MJCF: a loaded SDF
        // geom's field is its mesh's octree.
        self.geom_sdf.push(None);

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

        Ok(geom)
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
        let geom_mesh_ref = match geom_type {
            // A mesh geom that names no mesh, refused as MuJoCo 3.5.0 refuses
            // it (`mjCGeom::Compile`, `user_objects.cc:3656-3659`; an empty
            // name names none), where this crate refused it before
            // Rigid-loading PR RL-9d, in other words (ledger L353, L357).
            GeomType::Mesh => match geom.mesh.as_deref().filter(|m| !m.is_empty()) {
                Some(mesh_name) => {
                    let mesh_id = self
                        .mesh_name_to_id
                        .get(mesh_name)
                        .ok_or_else(|| MjcfError::undefined(ElementKind::Mesh, mesh_name))?;
                    Some(*mesh_id)
                }
                None => {
                    return Err(MjcfError::invalid_model("mesh geom must have valid meshid"));
                }
            },
            // An SDF geom's mesh, as MuJoCo's `geom_dataid` holds it: its
            // bounds are its mesh's.
            GeomType::Sdf => geom_mesh_id(geom, &self.mesh_name_to_id),
            _ => None,
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

/// This crate's refusal of a geom of type `sdf` with a `<plugin>` (registry
/// D-GEOM-SDF-PLUGIN): MuJoCo reads such a geom's field from the plugin
/// (`engine_collision_sdf.c:275-276`, `engine_ray.c:899-901`), and sim-core
/// runs no MuJoCo plugin. The first such geom that `discardvisual` keeps
/// (with `referenced`, one that is not visual), the world's geoms first,
/// then each body's before its children's; `None` where there is none.
pub fn sdf_plugin_fault(
    mjcf: &MjcfModel,
    referenced: Option<&super::discard::Referenced>,
) -> Option<MjcfError> {
    fn first<'a>(
        body: &'a MjcfBody,
        referenced: Option<&super::discard::Referenced>,
    ) -> Option<&'a MjcfGeom> {
        body.geoms
            .iter()
            .find(|g| {
                g.geom_type == Some(MjcfGeomType::Sdf)
                    && g.plugin.is_some()
                    && referenced.is_none_or(|r| !r.geom_is_visual(g))
            })
            .or_else(|| body.children.iter().find_map(|c| first(c, referenced)))
    }
    first(&mjcf.worldbody, referenced).map(|g| {
        MjcfError::unsupported("a <plugin> on a geom of type sdf (registry D-GEOM-SDF-PLUGIN)").at(
            Location::named(ElementKind::Geom, g.name.as_deref().unwrap_or("")),
        )
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
/// A mesh geom's volume is its mesh's ([`MeshProps::volume`]); a height
/// field's comes from its asset's four sizes.
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
/// ([`hfield_geom_size`]), a plane's 0. A mesh or SDF geom's is its mesh's
/// ([`MeshProps::volume`]), and a primitive geom's with `shellinertia` its
/// surface's area ([`shell_area`]).
pub(super) fn geom_volume(
    geom: &MjcfGeom,
    mesh_props: Option<&MeshProps>,
    hfield_size: Option<&[f64; 4]>,
) -> f64 {
    use std::f64::consts::PI;
    let geom_type = geom.geom_type.unwrap_or(MjcfGeomType::Sphere);
    if geom.shellinertia == Some(true)
        && !matches!(geom_type, MjcfGeomType::Mesh | MjcfGeomType::Sdf)
    {
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
        MjcfGeomType::Mesh => mesh_props.map_or(0.001, |props| props.volume),
        // Its mesh's, as a mesh geom's (`GetVolume`'s first arm).
        MjcfGeomType::Sdf => mesh_props.map_or(0.001, |props| props.volume),
        MjcfGeomType::Plane => 0.0,
    }
}

/// The sizes MuJoCo 3.5.0 gives a height-field geom from its asset's four
/// (`user_objects.cc:3761-3764`), stored as its size and read by its volume
/// and inertia: the asset's two half-extents, and a quarter of its top plus
/// half its base.
fn hfield_geom_size(asset: &[f64; 4]) -> [f64; 3] {
    [asset[0], asset[1], 0.25 * asset[2] + 0.5 * asset[3]]
}

/// The sizes MuJoCo 3.5.0 computes a geom's volume and inertia from: its size
/// as resolved (`check_geom_size` has validated every place its type reads,
/// or `process_geom` a fitted geom's), or a height field's from its asset
/// ([`hfield_geom_size`]).
fn mass_sizes(geom: &MjcfGeom, hfield_size: Option<&[f64; 4]>) -> [f64; 3] {
    if geom.geom_type == Some(MjcfGeomType::Hfield) {
        hfield_size.map_or([0.0; 3], hfield_geom_size)
    } else {
        written_over(geom.size, [0.0; 3])
    }
}

/// A geom's principal moments about its centre of mass, as a diagonal
/// matrix along its axes: a primitive's along the geom's own, a mesh's along
/// its mesh's principal axes ([`MeshProps::quat`]). A mesh geom's takes its
/// mesh's [`MeshProps`]; a height field's its asset's four sizes.
///
/// A primitive geom's is MuJoCo 3.5.0's (`mjCGeom::SetInertia`,
/// `user_objects.cc:3194-3413`), each product and quotient in MuJoCo's
/// order: a capsule's is its cylinder's plus its two hemispheres', each
/// taking the share of the mass its volume takes; a height field's is its
/// box's; a mesh's the box of its mesh's volume and principal moments
/// ([`MeshProps::boxsz`]). A primitive geom's with `shellinertia` is its
/// surface's ([`shell_inertia`]).
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
    let geom_type = geom.geom_type.unwrap_or(MjcfGeomType::Sphere);
    if geom.shellinertia == Some(true)
        && !matches!(geom_type, MjcfGeomType::Mesh | MjcfGeomType::Sdf)
    {
        return shell_inertia(geom_type, size, mass);
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
        // MuJoCo's mesh arm, an SDF's too: the box of the mesh's volume and
        // principal moments, along the mesh's principal axes.
        MjcfGeomType::Mesh | MjcfGeomType::Sdf => match mesh_props {
            Some(props) => {
                let [b00, b11, b22] = props.boxsz.map(|b| b * b);
                Matrix3::from_diagonal(&Vector3::new(
                    mass * (b11 + b22) / 3.0,
                    mass * (b00 + b22) / 3.0,
                    mass * (b00 + b11) / 3.0,
                ))
            }
            None => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)),
        },
        // MuJoCo's default arm.
        MjcfGeomType::Plane => Matrix3::zeros(),
    }
}

/// The surface area of a primitive geom with `shellinertia`, as MuJoCo
/// 3.5.0's volume reads it for a shell (`mjCGeom::GetVolume`,
/// `user_objects.cc:3115-3189`), each product in MuJoCo's order: an
/// ellipsoid's by Thomsen's approximation, a height field's its box's on
/// the sizes MuJoCo gives it from its asset ([`hfield_geom_size`]), a
/// plane's 0. A mesh or SDF geom does not reach it: its volume and inertia
/// are its mesh's ([`geom_volume`], [`compute_geom_inertia`]).
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

/// A primitive geom's principal moments with `shellinertia`, as MuJoCo
/// 3.5.0's `mjCGeom::SetInertia` computes them (`user_objects.cc:3194-3413`,
/// its `mjINERTIA_SHELL` arms), each product and quotient in MuJoCo's order,
/// from the geom's `mass` (written, or its density times its surface's area,
/// [`compute_geom_mass`]): a sphere's thin shell; a capsule's cylinder and
/// two hemispheres, and a cylinder's side and two disks, each taking the
/// share of the mass its area takes; an ellipsoid's the difference between
/// it and itself grown by 1e-6 at the one density that gives the shell
/// between them its mass; a box's (a height field's too) its six faces, each
/// pair the share of the mass its area takes. A plane's is 0.
fn shell_inertia(geom_type: MjcfGeomType, size: [f64; 3], mass: f64) -> Matrix3<f64> {
    use std::f64::consts::PI;
    let diagonal = |i: [f64; 3]| Matrix3::from_diagonal(&Vector3::from(i));
    let [s00, s11, s22] = size.map(|s| s * s);
    match geom_type {
        MjcfGeomType::Sphere => {
            let i = 2.0 * mass * size[0] * size[0] / 3.0;
            diagonal([i, i, i])
        }
        MjcfGeomType::Capsule => {
            let (halfheight, height, radius) = (size[1], 2.0 * size[1], size[0]);
            let a_sphere = 4.0 * PI * radius * radius;
            let a_cylinder = 2.0 * PI * radius * height;
            let a_total = a_sphere + a_cylinder;
            let sphere_mass = mass * a_sphere / a_total;
            let cylinder_mass = mass - sphere_mass;
            let side = cylinder_mass * (6.0 * radius * radius + height * height) / 12.0;
            let axial = cylinder_mass * radius * radius;
            // The two hemispheres, each displaced along the axis.
            let sphere_inertia = 2.0 * sphere_mass * radius * radius / 3.0;
            let hs_com = radius / 2.0;
            let hs_pos = halfheight + hs_com;
            let shifted = sphere_inertia + sphere_mass * (hs_pos * hs_pos - hs_com * hs_com);
            diagonal([side + shifted, side + shifted, axial + sphere_inertia])
        }
        MjcfGeomType::Cylinder => {
            let (halfheight, radius) = (size[1], size[0]);
            let height = 2.0 * halfheight;
            let a_disk = PI * radius * radius;
            let a_cylinder = 2.0 * PI * radius * height;
            let a_total = 2.0 * a_disk + a_cylinder;
            let mass_disk = mass * a_disk / a_total;
            let mass_cylinder = mass - 2.0 * mass_disk;
            let side = mass_cylinder * (6.0 * radius * radius + height * height) / 12.0;
            let axial = mass_cylinder * radius * radius;
            // The top and bottom disks.
            let disk_x = mass_disk * radius * radius / 4.0 + mass_disk * halfheight * halfheight;
            let disk_z = mass_disk * radius * radius / 2.0;
            diagonal([
                side + 2.0 * disk_x,
                side + 2.0 * disk_x,
                axial + 2.0 * disk_z,
            ])
        }
        MjcfGeomType::Ellipsoid => {
            let eps = 1e-6;
            let va = 4.0 * PI * size[0] * size[1] * size[2] / 3.0;
            let (ae, be, ce) = (size[0] + eps, size[1] + eps, size[2] + eps);
            let vb = 4.0 * PI * ae * be * ce / 3.0;
            let density = mass / (vb - va);
            let mass_a = va * density;
            let mass_b = vb * density;
            diagonal([
                mass_b * (be * be + ce * ce) / 5.0 - mass_a * (s11 + s22) / 5.0,
                mass_b * (ae * ae + ce * ce) / 5.0 - mass_a * (s00 + s22) / 5.0,
                mass_b * (ae * ae + be * be) / 5.0 - mass_a * (s00 + s11) / 5.0,
            ])
        }
        MjcfGeomType::Box | MjcfGeomType::Hfield => {
            let (lx, ly, lz) = (2.0 * size[0], 2.0 * size[1], 2.0 * size[2]);
            let (a0, a1, a2) = (lx * ly, ly * lz, lz * lx);
            let a_total = 2.0 * (a0 + a1 + a2);
            let mass0 = mass * a0 / a_total;
            let (ix0, iy0, iz0) = (
                mass0 * ly * ly / 12.0,
                mass0 * lx * lx / 12.0,
                mass0 * (lx * lx + ly * ly) / 12.0,
            );
            let mass1 = mass * a1 / a_total;
            let (ix1, iy1, iz1) = (
                mass1 * (ly * ly + lz * lz) / 12.0,
                mass1 * lz * lz / 12.0,
                mass1 * ly * ly / 12.0,
            );
            let mass2 = mass * a2 / a_total;
            let (ix2, iy2, iz2) = (
                mass2 * lz * lz / 12.0,
                mass2 * (lx * lx + lz * lz) / 12.0,
                mass2 * lx * lx / 12.0,
            );
            diagonal([
                2.0 * (mass0 * s22 + mass2 * s11 + ix0 + ix1 + ix2),
                2.0 * (mass0 * s22 + mass1 * s00 + iy0 + iy1 + iy2),
                2.0 * (mass1 * s00 + mass2 * s11 + iz0 + iz1 + iz2),
            ])
        }
        // MuJoCo's default arm; a mesh or SDF geom does not reach it.
        MjcfGeomType::Plane | MjcfGeomType::Mesh | MjcfGeomType::Sdf => Matrix3::zeros(),
    }
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
            frame: None,
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
            fitscale: None,
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
            // v_sphere_r001
            (
                MjcfGeomType::Sphere,
                &[0.01],
                None,
                None,
                None,
                [
                    1.6755160819145565e-07,
                    1.6755160819145565e-07,
                    1.6755160819145565e-07,
                ],
            ),
            // v_capsule_r002
            (
                MjcfGeomType::Capsule,
                &[0.02, 0.03],
                Some(1.0),
                None,
                None,
                [
                    0.0007415384615384616,
                    0.0007415384615384616,
                    0.0001876923076923077,
                ],
            ),
            // v_box_thin_mass
            (
                MjcfGeomType::Box,
                &[0.1, 0.01, 0.03],
                Some(1.3),
                None,
                None,
                [
                    0.00043333333333333337,
                    0.004723333333333334,
                    0.004376666666666668,
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
