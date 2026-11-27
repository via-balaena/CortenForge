//! Mesh and heightfield asset processing.
//!
//! Handles a mesh's vertices and faces, from its file (`mesh_file`) or its
//! embedded vertex/face data, computing mesh inertia properties, loading
//! height fields, and registering mesh/hfield assets in the builder's lookup
//! tables.

use nalgebra::{Point3, Vector3};
use sim_core::HeightFieldData;
use sim_core::mesh::{HullError, TriangleMeshData};
use std::collections::HashSet;
use std::path::Path;
use std::sync::Arc;
use tracing::warn;

use super::ModelBuilder;
use super::asset::{AssetKind, resolve_asset_path, stripped_file};
use super::eig3::eig3;
use super::mass::MJ_MINVAL;
use super::mesh_file::{content_type_of, load_mesh_file};
use super::orientation::{mulvecmat, mulvecmat_t, normvec, quat2mat};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{
    MeshInertia, MjcfBody, MjcfCompiler, MjcfGeomType, MjcfHfield, MjcfMesh, MjcfModel,
};

/// Two meshes, or two height fields, of one written name, refused as MuJoCo
/// 3.5.0's reader refuses the second as it reads it (`mjs_setName`,
/// `user_api.cc:1734-1748`), before [`name_assets`] names the unnamed ones.
/// It runs before the model's classes are applied, where this crate checked
/// every mesh's and height field's name before Rigid-loading PR RL-9d.
/// MuJoCo's reader takes `<default>` before `<asset>`
/// (`xml_native_reader.cc:1018`, `:1034`), so a fault in a class is named
/// after the repeat here (ledger L353).
pub fn check_written_names(mjcf: &MjcfModel) -> Result<()> {
    repeated_name(
        ElementKind::Mesh,
        mjcf.meshes.iter().map(|m| m.name.as_str()),
    )?;
    repeated_name(
        ElementKind::Hfield,
        mjcf.hfields.iter().map(|h| h.name.as_str()),
    )
}

/// The first name of `names` met twice, refused as repeated; an empty name
/// is no name.
fn repeated_name<'a>(kind: ElementKind, names: impl Iterator<Item = &'a str>) -> Result<()> {
    let mut seen = std::collections::HashSet::new();
    names
        .filter(|name| !name.is_empty())
        .find(|name| !seen.insert(*name))
        .map_or(Ok(()), |name| Err(MjcfError::duplicate_name(kind, name)))
}

/// Gives each unnamed mesh and height field the name MuJoCo 3.5.0 gives it
/// (`mjCMesh::CopyFromSpec`, `mjCHField::CopyFromSpec`): its `file`'s text
/// past the last `/` or `\`, cut at the last `.` (`mjuu_strippath`,
/// `mjuu_stripext`, `user_util.cc:768-829`), an empty one refused as
/// MuJoCo refuses it ("empty name in mesh", "empty name in height field",
/// `user_model.cc:2034-2041`), and two of one name refused as a repeated
/// name, as MuJoCo's `ProcessLists` refuses them before any pass drops an
/// unused mesh (`user_model.cc:4888-4891`). It runs on the model before its
/// classes are applied, where this crate named the assets before
/// Rigid-loading PR RL-9d, and before validation and every compiler pass,
/// which find assets by name. MuJoCo names them as its compile begins
/// (`user_model.cc:4872-4888`), after every fault its reader meets: such a
/// fault is named after these here (ledger L353).
pub fn name_assets(mjcf: &mut MjcfModel) -> Result<()> {
    let named = |name: &str, file: Option<&str>, what: &str| {
        if !name.is_empty() {
            return Ok(name.to_owned());
        }
        let stem = file.map_or("", |file| {
            let base = file.rsplit(['/', '\\']).next().unwrap_or(file);
            base.rfind('.').map_or(base, |dot| &base[..dot])
        });
        if stem.is_empty() {
            return Err(MjcfError::invalid_model(format!("empty name in {what}")));
        }
        Ok(stem.to_owned())
    };
    for (i, mesh) in mjcf.meshes.iter_mut().enumerate() {
        mesh.name = named(&mesh.name, mesh.file.as_deref(), "mesh")
            .map_err(|e| e.at(Location::nth(ElementKind::Mesh, None, i)))?;
    }
    for (i, hfield) in mjcf.hfields.iter_mut().enumerate() {
        hfield.name = named(&hfield.name, hfield.file.as_deref(), "height field")
            .map_err(|e| e.at(Location::nth(ElementKind::Hfield, None, i)))?;
    }
    repeated_name(
        ElementKind::Mesh,
        mjcf.meshes.iter().map(|m| m.name.as_str()),
    )?;
    repeated_name(
        ElementKind::Hfield,
        mjcf.hfields.iter().map(|h| h.name.as_str()),
    )
}

/// Each mesh's written vertices and faces as MuJoCo 3.5.0's `CopyFromSpec`
/// checks them (`mjCModel::TryCompile`, `user_model.cc:4872`;
/// `ProcessVertices`, `user_mesh.cc:543-561`): the vertex count a multiple of
/// 3, then the face count, then each vertex finite (a `NaN` or an infinity
/// written in a document is refused as it is read, registry `D-NONFINITE`;
/// one set in code is refused here). MuJoCo checks them before it names the
/// meshes and checks their names; this crate checks them since Rigid-loading
/// PR RL-9d, after every name (ledger L353).
pub fn check_written_counts(meshes: &[MjcfMesh]) -> Result<()> {
    for (i, mesh) in meshes.iter().enumerate() {
        let vertex = mesh.vertex.as_deref().unwrap_or_default();
        let face = mesh.face.as_deref().unwrap_or_default();
        let reason = if !vertex.len().is_multiple_of(3) {
            "vertex data must be a multiple of 3".to_owned()
        } else if !face.len().is_multiple_of(3) {
            "face data must be a multiple of 3".to_owned()
        } else if let Some(v) = vertex
            .chunks_exact(3)
            .position(|c| !c.iter().all(|x| x.is_finite()))
        {
            format!("vertex coordinate {v} is not finite")
        } else {
            continue;
        };
        return Err(MjcfError::invalid_model(reason).at(Location::nth(
            ElementKind::Mesh,
            Some(&mesh.name),
            i,
        )));
    }
    Ok(())
}

/// The meshes MuJoCo 3.5.0 makes a convex hull of (`mjCModel::TryCompile`,
/// `user_model.cc:4906-4923`): each that a mesh or SDF geom names where the
/// geom's `contype` or `conaffinity` is not 0 or a `<pair>` names the geom,
/// and each `convex` mesh. MuJoCo marks a `convex` mesh only where a mesh or
/// SDF geom names it, and crashes on one that none names, reading the hull it
/// did not make (registry `D-MESH-CONVEX-UNNAMED`); this crate makes the hull
/// of every `convex` mesh. Read on the whole model, before `discardvisual`
/// deletes anything, as MuJoCo marks the meshes (`user_model.cc:4906-4923`,
/// before `:4976`).
pub fn meshes_needing_a_hull(mjcf: &MjcfModel) -> HashSet<String> {
    let paired: HashSet<&str> = mjcf
        .contact
        .pairs
        .iter()
        .flat_map(|pair| [pair.geom1.as_str(), pair.geom2.as_str()])
        .collect();
    let mut needed: HashSet<String> = mjcf
        .meshes
        .iter()
        .filter(|mesh| mesh.inertia.unwrap_or_default() == MeshInertia::Convex)
        .map(|mesh| mesh.name.clone())
        .collect();
    let mut bodies: Vec<&MjcfBody> = vec![&mjcf.worldbody];
    while let Some(body) = bodies.pop() {
        for geom in &body.geoms {
            let shaped = matches!(geom.geom_type, Some(MjcfGeomType::Mesh | MjcfGeomType::Sdf));
            let collides = geom.contype.unwrap_or(1) != 0
                || geom.conaffinity.unwrap_or(1) != 0
                || geom
                    .name
                    .as_deref()
                    .is_some_and(|name| paired.contains(name));
            if shaped
                && collides
                && let Some(mesh) = &geom.mesh
            {
                needed.insert(mesh.clone());
            }
        }
        bodies.extend(&body.children);
    }
    needed
}

/// The meshes MuJoCo 3.5.0 makes an octree for (`needsdf`): each that a geom
/// of type `sdf` names, whatever its `contype` and `conaffinity`, as
/// `mjCModel::IndexAssets` marks them (`user_model.cc:1885-1900`) in its
/// first call (`:4899`), before its second (`:4976`) discards visual geoms
/// (`:1987-2023`): read on the whole model, so a mesh a discarded visual
/// SDF geom names gets its octree where another geom keeps the mesh.
pub fn meshes_needing_an_octree(mjcf: &MjcfModel) -> HashSet<String> {
    let mut needed = HashSet::new();
    let mut bodies: Vec<&MjcfBody> = vec![&mjcf.worldbody];
    while let Some(body) = bodies.pop() {
        for geom in &body.geoms {
            if geom.geom_type == Some(MjcfGeomType::Sdf)
                && let Some(mesh) = &geom.mesh
            {
                needed.insert(mesh.clone());
            }
        }
        bodies.extend(&body.children);
    }
    needed
}

/// This crate's refusal of a mesh with a `<plugin>` (registry
/// D-MESH-PLUGIN): MuJoCo makes such a mesh, or the field of a geom of type
/// `sdf` over it, from the plugin, and sim-core runs no MuJoCo plugin. The
/// first such mesh in asset order that a geom names, a visual one among them
/// (every geom is built before `discardvisual` deletes any), or, without
/// `discardvisual`, any; `None` where there is none. A plugin mesh with no
/// file that no geom names, which `discardvisual` deletes, is not built.
pub fn plugin_fault(
    mjcf: &MjcfModel,
    referenced: Option<&super::discard::Referenced>,
) -> Option<MjcfError> {
    let refused = |name: &str| referenced.is_none_or(|r| r.names_mesh(name));
    mjcf.meshes
        .iter()
        .find(|m| m.plugin && refused(&m.name))
        .map(plugin_error)
}

/// [`plugin_fault`]'s refusal of `mesh`.
pub fn plugin_error(mesh: &MjcfMesh) -> MjcfError {
    MjcfError::unsupported("a <plugin> on a mesh (registry D-MESH-PLUGIN)")
        .at(Location::named(ElementKind::Mesh, &mesh.name))
}

/// A mesh's mass properties as MuJoCo 3.5.0's `mjCMesh::Process` leaves
/// them (`user_mesh.cc:1605-1668`), from its vertices as transformed
/// ([`apply_transformations`]): computed once per mesh by
/// [`mesh_mass_properties`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshProps {
    /// Its volume, or under `shell` its surface's area (`GetVolumeRef`):
    /// the total `ComputeInertia` returns, which replaces the one checked
    /// before it (`user_mesh.cc:1634-1639`).
    pub volume: f64,
    /// Its centre of mass in its vertices' frame (`pos_`).
    pub com: [f64; 3],
    /// Its principal axes in that frame, (w, x, y, z) (`quat_`): `eig3`'s
    /// of its inertia about `com`.
    pub quat: [f64; 4],
    /// The half sizes of the box with its volume and principal moments
    /// (`boxsz_`), from which a mesh geom's inertia is computed.
    pub boxsz: [f64; 3],
}

impl ModelBuilder {
    /// Process a single mesh asset from MJCF.
    ///
    /// Converts `MjcfMesh` to `TriangleMeshData` with BVH, computes its mass
    /// properties, moves it to its centre of mass and principal axes, as
    /// MuJoCo 3.5.0 keeps a mesh, and registers it in the mesh lookup table.
    /// The mesh can then be referenced by geoms.
    ///
    /// # Arguments
    ///
    /// * `mjcf_mesh` - The MJCF mesh definition
    /// * `base_path` - Base directory for resolving relative mesh file paths
    /// * `needhull` - Whether its convex hull is made for its geoms
    ///   ([`meshes_needing_a_hull`]); a mesh with no faces gets one whatever
    ///   this says
    /// * `needsdf` - Whether its octree is made for a signed distance field
    ///   geom ([`meshes_needing_an_octree`]), after it is moved to its frame,
    ///   as MuJoCo makes it after `mjCMesh::Process` (`user_mesh.cc:774-830`)
    ///
    /// # Errors
    ///
    /// Returns [`MjcfError::DuplicateName`] if the mesh's name is an
    /// already-processed mesh's, the conversion's error if it fails (see
    /// [`convert_mjcf_mesh`]), and [`MjcfError::Asset`] where its mass
    /// properties or its octree cannot be made.
    pub(crate) fn process_mesh(
        &mut self,
        mjcf_mesh: &MjcfMesh,
        base_path: Option<&Path>,
        needhull: bool,
        needsdf: bool,
    ) -> Result<()> {
        // Check for duplicate mesh names
        if self.mesh_name_to_id.contains_key(&mjcf_mesh.name) {
            return Err(MjcfError::duplicate_name(
                ElementKind::Mesh,
                &mjcf_mesh.name,
            ));
        }

        // Convert MJCF mesh to TriangleMeshData
        let mut mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler, needhull)?;
        let props = mesh_mass_properties(&mesh_data, mjcf_mesh.inertia.unwrap_or_default())
            .map_err(|reason| MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason))?;
        mesh_data.replace_vertices(to_principal_frame(mesh_data.vertices(), &props));
        if needsdf {
            mesh_data.make_octree().map_err(|reason| {
                MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason.to_string())
            })?;
        }

        // Register in lookup table
        let mesh_id = self.mesh_data.len();
        self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
        self.mesh_name.push(mjcf_mesh.name.clone());
        self.mesh_data.push(Arc::new(mesh_data));
        self.mesh_props.push(props);

        Ok(())
    }

    /// Register a height field asset from MJCF: its name and its sizes,
    /// which a geom reads as it is built. Its data is compiled with the other
    /// height fields after the bodies ([`Self::compile_hfields`]), where
    /// MuJoCo 3.5.0 compiles it.
    pub(crate) fn process_hfield(&mut self, hfield: &MjcfHfield) -> Result<()> {
        if self.hfield_name_to_id.contains_key(&hfield.name) {
            return Err(MjcfError::duplicate_name(ElementKind::Hfield, &hfield.name));
        }
        let id = self.hfield_name.len();
        self.hfield_name_to_id.insert(hfield.name.clone(), id);
        self.hfield_name.push(hfield.name.clone());
        self.hfield_size.push(hfield.size);
        Ok(())
    }

    /// Each height field compiled as MuJoCo 3.5.0 compiles it, one after
    /// another ([`compile_hfield`]), used by a geom or not; `model_from_mjcf`
    /// says where its fault is named.
    pub(crate) fn compile_hfields(
        &mut self,
        hfields: &[MjcfHfield],
        base_path: Option<&Path>,
    ) -> Result<()> {
        for (h, hfield) in hfields.iter().enumerate() {
            let data = compile_hfield(hfield, base_path, &self.compiler).map_err(|reason| {
                MjcfError::invalid_model(reason).at(Location::nth(
                    ElementKind::Hfield,
                    Some(&hfield.name),
                    h,
                ))
            })?;
            self.hfield_data.push(Arc::new(data));
        }
        Ok(())
    }
}

/// A mesh's data as MuJoCo 3.5.0 checks it before it processes it
/// (`mjCMesh::CheckInitialMesh`, `user_mesh.cc:1806-1842`): its vertices,
/// an STL's merged where repeated; its faces' vertex ids, three to a face,
/// signed; how many normal and texture-coordinate numbers it holds; whether
/// its file gave each face its normals' ids (MuJoCo's `facenormal_`); and
/// whether it is an OBJ (`IsObj`). Each is its file's ([`load_mesh_file`]),
/// or its own where the file gave none or it names no file
/// ([`take_written`]).
#[derive(Debug)]
pub(super) struct InitialMesh {
    pub(super) vertices: Vec<Point3<f64>>,
    pub(super) faces: Vec<i64>,
    pub(super) normals: usize,
    pub(super) texcoords: usize,
    pub(super) face_normals: bool,
    pub(super) obj: bool,
}

/// MuJoCo 3.5.0's `mjEPS`: a field whose elevations span no more is
/// shifted to 0, not divided by its span.
const MJ_EPS: f64 = 1e-14;

/// A height field's data as MuJoCo 3.5.0's `mjCHField::Compile` makes it
/// (`user_objects.cc:4456-4573`), in its order:
///
/// 1. its elevations, where it holds any, `nrow · ncol` of them ("elevation
///    data length must match nrow*ncol"), the counts not read beside a file
///    (`CopyFromSpec` zeroes them, `:4349-4352`);
/// 2. its four sizes positive ("size parameter is not positive in hfield");
/// 3. its data: its file's where it names one that `strippath` leaves
///    something of ([`read_hfield_file`]); else its elevations with their
///    rows reversed, so the document's first row is the field's highest Y
///    (`xml_native_reader.cc:3442-3449`), or `nrow · ncol` zeros where it
///    writes neither elevation nor file (MuJoCo's reader fills them);
/// 4. a field with no grid or no data refused ("hfield not specified");
/// 5. its data moved to start at 0 and divided by its span where the span
///    is above [`MJ_EPS`], in single precision ([`normalise`]).
///
/// A grid whose `nrow · ncol` passes a C `int` is refused before any of
/// these, "elevation data length must match nrow*ncol" (registry
/// `D-HFIELD-COUNTS`: MuJoCo's product overflows there).
///
/// Each height is that elevation times `size[2]`, as MuJoCo's collision and
/// rays read it (`engine_collision_convex.c:1177`). The grid's spacing along
/// each axis is its extent over its cells, `2·size[0]/(ncol − 1)` and
/// `2·size[1]/(nrow − 1)`; a field of one column or row has no extent along
/// that axis, and its spacing there is the axis's whole width; a spacing
/// below the least positive normal double (a step that underflows, or a size
/// that is not a number in a model built in code) is raised to it. MuJoCo's
/// readers (contacts, rays, flex elements) take positions from `size` and the
/// grid, not from this spacing, which cf-geometry's own queries read.
///
/// # Errors
///
/// MuJoCo's reason, as the list above and [`read_hfield_file`] give it.
// The grid's counts are positive `i32`s whose product fits one, so exact as
// `usize` and as `f64`; each written elevation was read as a float, so exact
// as one.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn compile_hfield(
    hfield: &MjcfHfield,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> std::result::Result<HeightFieldData, String> {
    let length = || "elevation data length must match nrow*ncol".to_string();
    // The reader's file (written, perhaps empty) and `CopyFromSpec`'s (not
    // empty), beside which the counts are not read.
    let written = hfield.file.as_deref();
    let file = written.filter(|f| !f.is_empty());
    let (nrow, ncol) = if file.is_some() {
        (0, 0)
    } else {
        (hfield.nrow, hfield.ncol)
    };
    // MuJoCo multiplies the counts only where both are positive.
    let cells = i64::from(nrow) * i64::from(ncol);
    if nrow > 0 && ncol > 0 && cells > i64::from(i32::MAX) {
        return Err(length());
    }
    let elevation = hfield.elevation.as_deref().filter(|e| !e.is_empty());
    if elevation.is_some_and(|e| i64::try_from(e.len()).ok() != Some(cells)) {
        return Err(length());
    }
    if hfield.size.iter().any(|&s| s <= 0.0) {
        return Err("size parameter is not positive in hfield".to_string());
    }
    let stripped = written
        .map(|f| stripped_file(f, compiler))
        .unwrap_or_default();
    let (nrow, ncol, mut data) = if let Some(file) = file.filter(|_| !stripped.is_empty()) {
        read_hfield_file(file, hfield.content_type.as_deref(), base_path, compiler)?
    } else if nrow > 0 && ncol > 0 && (elevation.is_some() || written.is_none()) {
        let (nrow, ncol) = (nrow as usize, ncol as usize);
        let mut data = vec![0.0_f32; nrow * ncol];
        for (i, row) in elevation.unwrap_or_default().chunks_exact(ncol).enumerate() {
            let flip = nrow - 1 - i;
            for (j, &e) in row.iter().enumerate() {
                data[flip * ncol + j] = e as f32;
            }
        }
        (nrow, ncol, data)
    } else {
        (0, 0, Vec::new())
    };
    if nrow < 1 || ncol < 1 || data.is_empty() {
        return Err("hfield not specified".to_string());
    }
    normalise(&mut data, &stripped)?;

    let [x, y, top, _] = hfield.size;
    let heights = data.iter().map(|&d| f64::from(d) * top).collect();
    let step = |half: f64, n: usize| {
        let step = if n > 1 {
            2.0 * half / (n - 1) as f64
        } else {
            2.0 * half
        };
        step.max(f64::MIN_POSITIVE)
    };
    Ok(HeightFieldData::with_spacing(
        heights,
        ncol,
        nrow,
        [step(x, ncol), step(y, nrow)],
    ))
}

/// MuJoCo 3.5.0's normalisation of a height field's data, in single
/// precision (`user_objects.cc:4558-4572`): its least and greatest values,
/// each `std::min` and `std::max` from 1E+10 and -1E+10, so a NaN changes
/// neither and a field above 1E+10 or below -1E+10 keeps that bound; each
/// value less the least, divided by their difference where it is above
/// [`MJ_EPS`].
///
/// # Errors
///
/// `invalid data range in hfield '<file>'` where the least is above the
/// greatest, as where every value is NaN.
fn normalise(data: &mut [f32], file: &str) -> std::result::Result<(), String> {
    let (mut emin, mut emax) = (1e10_f32, -1e10_f32);
    for &d in data.iter() {
        if d < emin {
            emin = d;
        }
        if emax < d {
            emax = d;
        }
    }
    if emin > emax {
        return Err(format!("invalid data range in hfield '{file}'"));
    }
    for d in data.iter_mut() {
        *d -= emin;
        if f64::from(emax - emin) > MJ_EPS {
            *d /= emax - emin;
        }
    }
    Ok(())
}

/// A height field file's data, as MuJoCo 3.5.0 reads it
/// (`mjCHField::Compile`, `user_objects.cc:4484-4508`): its format by its
/// content type ([`hfield_content_type`]), MuJoCo's own where it names
/// none; a PNG ([`read_hfield_png`]) or MuJoCo's format
/// ([`read_hfield_custom`]). Returns its rows, its columns and its data,
/// the field's lowest Y first.
///
/// # Errors
///
/// `unsupported content type: '<type>'` for a type of neither; the file
/// not found or read; the format's own reasons.
fn read_hfield_file(
    file: &str,
    content_type: Option<&str>,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> std::result::Result<(usize, usize, Vec<f32>), String> {
    let kind = hfield_content_type(&stripped_file(file, compiler), content_type)
        .unwrap_or_else(|| "image/vnd.mujoco.hfield".to_string());
    if kind != "image/png" && kind != "image/vnd.mujoco.hfield" {
        return Err(format!("unsupported content type: '{kind}'"));
    }
    // Without the `hfields` feature, a PNG is refused naming the feature,
    // before its file is looked for.
    if kind == "image/png" && !cfg!(feature = "hfields") {
        return read_hfield_png(&[], file);
    }
    let path = resolve_asset_path(file, base_path, compiler, AssetKind::Mesh)?;
    let bytes =
        std::fs::read(&path).map_err(|e| format!("could not read hfield file '{file}': {e}"))?;
    let name = path.display().to_string();
    if kind == "image/png" {
        read_hfield_png(&bytes, &name)
    } else {
        read_hfield_custom(&bytes, &name)
    }
}

/// A height field file's content type as MuJoCo 3.5.0 names it
/// (`GetAssetContentType`, `user_objects.cc:1225-1237`): its `content_type`'s
/// `type/subtype` where written, none where that is not of the form MuJoCo
/// accepts ([`content_type_of`]); else its extension's, the text from the
/// file's last `.` in any case (`mjuu_extToContentType`,
/// `user_util.cc:957-973`): `.png` an image's, `.stl`, `.obj`, `.ply` and
/// `.msh` a model's, any other none.
fn hfield_content_type(file: &str, content_type: Option<&str>) -> Option<String> {
    if let Some(text) = content_type.filter(|text| !text.is_empty()) {
        return content_type_of(text);
    }
    let ext = file
        .rfind('.')
        .map(|dot| file[dot..].to_ascii_lowercase())?;
    let kind = match ext.as_str() {
        ".stl" => "model/stl",
        ".obj" => "model/obj",
        ".ply" => "model/ply",
        ".msh" => "model/vnd.mujoco.msh",
        ".png" => "image/png",
        _ => return None,
    };
    Some(kind.to_string())
}

/// A PNG height field's data as MuJoCo 3.5.0 reads it (`mjCHField::LoadPNG`,
/// `user_objects.cc:4438-4452`): decoded to 8-bit grey as lodepng converts it
/// (`lodepng.cpp:3500`, `:3524`, `:3593`), a colour pixel's red, a 16-bit
/// sample's high byte, a grey of fewer bits scaled to 255; its rows reversed,
/// so the image's top row is the field's highest Y. Returns its rows, its
/// columns and its data.
///
/// # Errors
///
/// `empty PNG file '<file>'`, as MuJoCo's words; `error decoding PNG file
/// '<file>': …` where the image does not decode, in the decoder's words
/// (registry `D-HFIELD-PNG`: lodepng decides some malformed files
/// otherwise); the `hfields` feature's absence.
#[cfg(feature = "hfields")]
fn read_hfield_png(
    bytes: &[u8],
    file: &str,
) -> std::result::Result<(usize, usize, Vec<f32>), String> {
    use image::DynamicImage;
    if bytes.is_empty() {
        return Err(format!("empty PNG file '{file}'"));
    }
    // An image of more pixels than a C `int` counts, MuJoCo's grid's, refused
    // before it is decoded, in the words lodepng gives an image past 2⁶¹
    // pixels (registry `D-HFIELD-COUNTS`).
    if let Some((width, height)) = png_dimensions(bytes)
        && u64::from(width) * u64::from(height) > u64::from(i32::MAX.unsigned_abs())
    {
        return Err(format!(
            "error decoding PNG file '{file}': integer overflow due to too many pixels"
        ));
    }
    // The decoder's memory bounded at [`PNG_DECODE_LIMIT`]; a panic in it (a
    // palette whose length is not a multiple of 3) refused (registry
    // `D-HFIELD-PNG`).
    let decoded = std::panic::catch_unwind(|| {
        let mut reader =
            image::ImageReader::with_format(std::io::Cursor::new(bytes), image::ImageFormat::Png);
        let mut limits = image::Limits::default();
        limits.max_alloc = Some(PNG_DECODE_LIMIT);
        reader.limits(limits);
        reader.decode()
    });
    let image = match decoded {
        Ok(Ok(image)) => image,
        Ok(Err(e)) => return Err(format!("error decoding PNG file '{file}': {e}")),
        Err(_) => {
            return Err(format!(
                "error decoding PNG file '{file}': the decoder failed"
            ));
        }
    };
    let (ncol, nrow) = (image.width() as usize, image.height() as usize);
    let high = |v: u16| v.to_be_bytes()[0];
    let grey: Vec<u8> = match image {
        DynamicImage::ImageLuma8(i) => i.into_raw(),
        DynamicImage::ImageLumaA8(i) => i.pixels().map(|p| p.0[0]).collect(),
        DynamicImage::ImageRgb8(i) => i.pixels().map(|p| p.0[0]).collect(),
        DynamicImage::ImageRgba8(i) => i.pixels().map(|p| p.0[0]).collect(),
        DynamicImage::ImageLuma16(i) => i.pixels().map(|p| high(p.0[0])).collect(),
        DynamicImage::ImageLumaA16(i) => i.pixels().map(|p| high(p.0[0])).collect(),
        DynamicImage::ImageRgb16(i) => i.pixels().map(|p| high(p.0[0])).collect(),
        DynamicImage::ImageRgba16(i) => i.pixels().map(|p| high(p.0[0])).collect(),
        _ => {
            return Err(format!(
                "error decoding PNG file '{file}': not an 8- or 16-bit image"
            ));
        }
    };
    let mut data = Vec::with_capacity(nrow * ncol);
    for r in 0..nrow {
        let row = (nrow - 1 - r) * ncol;
        data.extend(grey[row..row + ncol].iter().map(|&v| f32::from(v)));
    }
    Ok((nrow, ncol, data))
}

/// The most a PNG height field's decoding may allocate, 8 GiB: above it the
/// image is refused, "Memory limit exceeded" (registry `D-HFIELD-PNG`). The
/// decoder's own 512 MiB refused images MuJoCo loads, and no limit let it
/// abort on an image whose header claims more pixels than memory holds.
#[cfg(feature = "hfields")]
const PNG_DECODE_LIMIT: u64 = 8 << 30;

/// A PNG's width and height from its `IHDR`, where it holds one where the
/// format puts it (the signature, then the chunk's length and name).
#[cfg(feature = "hfields")]
fn png_dimensions(bytes: &[u8]) -> Option<(u32, u32)> {
    if bytes.get(12..16)? != b"IHDR" {
        return None;
    }
    let word = |at: usize| {
        bytes
            .get(at..at + 4)?
            .try_into()
            .ok()
            .map(u32::from_be_bytes)
    };
    Some((word(16)?, word(20)?))
}

/// Without the `hfields` feature a PNG height field is refused, naming the
/// feature.
#[cfg(not(feature = "hfields"))]
fn read_hfield_png(
    _bytes: &[u8],
    file: &str,
) -> std::result::Result<(usize, usize, Vec<f32>), String> {
    Err(format!(
        "PNG height field '{file}' needs the `hfields` feature (enable \
         `sim-mjcf/hfields`, or write its elevations inline)"
    ))
}

/// A height field in MuJoCo 3.5.0's own format (`mjCHField::LoadCustom`,
/// `user_objects.cc:4394-4435`): two `int`s, its rows and its columns, then
/// `nrow · ncol` `float`s, each in the machine's byte order, the field's
/// lowest Y first. Returns its rows, its columns and its data.
///
/// # Errors
///
/// MuJoCo's words: an empty file, "could not read hfield file"; one shorter
/// than the two counts, "hfield missing header"; a count below 1,
/// "non-positive hfield dimensions in file"; a length other than the counts
/// give, "unexpected file size in file".
fn read_hfield_custom(
    bytes: &[u8],
    file: &str,
) -> std::result::Result<(usize, usize, Vec<f32>), String> {
    if bytes.is_empty() {
        return Err(format!("could not read hfield file '{file}'"));
    }
    let (Some(rows), Some(cols)) = (bytes.get(0..4), bytes.get(4..8)) else {
        return Err(format!("hfield missing header '{file}'"));
    };
    let count = |b: &[u8]| i32::from_ne_bytes([b[0], b[1], b[2], b[3]]);
    let (nrow, ncol) = (count(rows), count(cols));
    let (Ok(nrow), Ok(ncol)) = (usize::try_from(nrow), usize::try_from(ncol)) else {
        return Err(format!("non-positive hfield dimensions in file '{file}'"));
    };
    if nrow < 1 || ncol < 1 {
        return Err(format!("non-positive hfield dimensions in file '{file}'"));
    }
    let expected = nrow
        .checked_mul(ncol)
        .and_then(|n| n.checked_mul(4))
        .and_then(|n| n.checked_add(8));
    if expected != Some(bytes.len()) {
        return Err(format!("unexpected file size in file '{file}'"));
    }
    let data = bytes[8..]
        .chunks_exact(4)
        .map(|b| f32::from_ne_bytes([b[0], b[1], b[2], b[3]]))
        .collect();
    Ok((nrow, ncol, data))
}

/// Convert MJCF mesh asset to `TriangleMeshData`, as MuJoCo 3.5.0 compiles
/// a mesh (`mjCMesh::TryCompile`, `user_mesh.cc:686-770`), in its order:
///
/// 1. its file read, where it names one ([`load_mesh_file`]): an STL, OBJ,
///    MSH or PLY file (also 3MF when the `threemf` feature is enabled);
/// 2. its own normals, texture coordinates, faces and vertices taken where
///    the file gave none, and refused where it gave some ([`take_written`]);
/// 3. the whole checked ([`check_initial_mesh`]);
/// 4. an `exact` mesh's face orientation checked ([`check_face_orientation`]);
///
/// then, as `mjCMesh::Process` goes on (`user_mesh.cc:1567-1603`), on its
/// vertices as written: its convex hull made where `needhull` or it has no
/// faces ([`make_graph`]), a mesh with no faces given the hull's; its
/// polygons merged, from the hull's faces where it has a hull. Its vertices
/// are then moved, turned and scaled as MuJoCo 3.5.0 does
/// ([`apply_transformations`]); the BVH is built over them. Its written
/// vertex and face counts were checked before ([`check_written_counts`]).
pub fn convert_mjcf_mesh(
    mjcf_mesh: &MjcfMesh,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    needhull: bool,
) -> Result<TriangleMeshData> {
    let refuse = |reason: String| MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason);
    let file = match &mjcf_mesh.file {
        Some(file) => Some(load_mesh_file(file, base_path, compiler, mjcf_mesh)?),
        None => None,
    };
    let mesh = take_written(file, mjcf_mesh).map_err(refuse)?;
    check_initial_mesh(&mesh).map_err(refuse)?;
    let InitialMesh {
        vertices, faces, ..
    } = mesh;
    // Each id names a vertex the mesh holds (`check_initial_mesh`).
    let indices: Vec<usize> = faces
        .into_iter()
        .filter_map(|id| usize::try_from(id).ok())
        .collect();
    check_face_orientation(&vertices, &indices, mjcf_mesh)?;
    let faceless = indices.is_empty();
    let mut mesh = TriangleMeshData::new(vertices, indices);
    if needhull || faceless {
        make_graph(&mut mesh, mjcf_mesh, faceless)?;
    } else {
        mesh.make_polygons();
    }
    let mut moved = mesh.vertices().to_vec();
    apply_transformations(&mut moved, mjcf_mesh);
    mesh.move_vertices(moved);
    Ok(mesh)
}

/// The mesh's convex hull as MuJoCo 3.5.0's `mjCMesh::MakeGraph` makes it
/// (`user_mesh.cc:1861-2042`), of its vertices as they are, with the faces
/// and polygons [`TriangleMeshData::compute_hull`] gives a mesh; where none
/// is made, the mesh is refused or goes on as [`hull_refusal`] says.
fn make_graph(mesh: &mut TriangleMeshData, mjcf_mesh: &MjcfMesh, faceless: bool) -> Result<()> {
    // MuJoCo's -1, written or not, for no limit.
    let limit = mjcf_mesh.maxhullvert.and_then(|n| usize::try_from(n).ok());
    let Err(err) = mesh.compute_hull(limit) else {
        return Ok(());
    };
    if let Some(reason) = hull_refusal(&err, faceless) {
        return Err(MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason));
    }
    warn!(mesh = %mjcf_mesh.name, "Could not construct convex hull graph");
    mesh.make_polygons();
    Ok(())
}

/// Why a mesh is refused where no hull was made (`err`), in MuJoCo 3.5.0's
/// words: "qhull error" where qhull stops, MuJoCo's own where its check of
/// qhull's result fails (`MakeGraph`, `user_mesh.cc:1914-2027`). `None` for
/// MuJoCo's warning ([`HullError::GRAPH_WARNING`]), after which a mesh with
/// faces goes on with no hull; one with none (`faceless`), whose
/// faces MuJoCo's `CopyGraph` then reads through the graph pointer
/// `MakeGraph` set to null as it warned (`user_mesh.cc:2003-2007`, `:2054`;
/// read, not run), is refused in those words. Where the vertices reach a
/// routine of qhull's that sim-qhull does not hold, this crate's words naming
/// it (registry `D-QHULL-UNPORTED`). No input is known to reach any but the
/// first.
fn hull_refusal(err: &HullError, faceless: bool) -> Option<String> {
    match err {
        HullError::Qhull { .. } => Some("qhull error".to_owned()),
        HullError::Graph { .. } if err.is_mujocos_warning() && !faceless => None,
        HullError::Graph { message } => Some((*message).to_owned()),
        HullError::Unported { .. } => Some(err.to_string()),
    }
}

/// The mesh's data: its file's, with its own normals, texture coordinates,
/// faces and vertices taken where the file gave none, as MuJoCo 3.5.0 takes
/// them (`mjCMesh::TryCompile`, `user_mesh.cc:731-760`), in that order, then
/// its own normals refused beside the file's face normals (`:752-753`); its
/// own alone where it names no file. It is an OBJ where its file is one or,
/// with no file, where its `content_type` is written `model/obj` (`IsObj`,
/// `:631-633`, the type as the mesh holds it).
///
/// # Errors
///
/// MuJoCo's words for the first of the five that the file and the mesh both
/// give.
fn take_written(
    file: Option<InitialMesh>,
    mjcf_mesh: &MjcfMesh,
) -> std::result::Result<InitialMesh, String> {
    let mut mesh = file.unwrap_or_else(|| InitialMesh {
        vertices: Vec::new(),
        faces: Vec::new(),
        normals: 0,
        texcoords: 0,
        face_normals: false,
        obj: mjcf_mesh.content_type.as_deref() == Some("model/obj"),
    });
    let normals = mjcf_mesh.normal.as_ref().map_or(0, Vec::len);
    let texcoords = mjcf_mesh.texcoord.as_ref().map_or(0, Vec::len);
    let faces = mjcf_mesh.face.as_deref().unwrap_or_default();
    let vertices = mjcf_mesh.vertex.as_deref().unwrap_or_default();
    if mesh.normals > 0 && normals > 0 {
        return Err("repeated normal specification".to_owned());
    }
    if mesh.normals == 0 {
        mesh.normals = normals;
    }
    if mesh.texcoords > 0 && texcoords > 0 {
        return Err("repeated texcoord specification".to_owned());
    }
    if mesh.texcoords == 0 {
        mesh.texcoords = texcoords;
    }
    if !mesh.faces.is_empty() && !faces.is_empty() {
        return Err("repeated face specification".to_owned());
    }
    if mesh.faces.is_empty() {
        mesh.faces = faces.iter().map(|&id| i64::from(id)).collect();
    }
    if !mesh.vertices.is_empty() && !vertices.is_empty() {
        return Err("repeated vertex specification".to_owned());
    }
    if mesh.vertices.is_empty() {
        mesh.vertices = vertices
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
    }
    if mesh.face_normals && normals > 0 {
        return Err("repeated facenormal specification".to_owned());
    }
    Ok(mesh)
}

/// MuJoCo 3.5.0's checks of a mesh's data before it processes it
/// (`mjCMesh::CheckInitialMesh`, `user_mesh.cc:1806-1842`), in its order
/// and words: at least 4 vertices; normals three numbers each; texture
/// coordinates two each and, but in an OBJ, one pair per vertex (MuJoCo
/// waives it too where the faces carry texture-coordinate indices, which
/// only an OBJ's do); then each face's vertex ids, in order, naming a vertex
/// the mesh holds. Its vertex and face counts, a file's multiples of 3,
/// were checked as written before ([`check_written_counts`]).
fn check_initial_mesh(mesh: &InitialMesh) -> std::result::Result<(), String> {
    let nvert = mesh.vertices.len();
    if nvert < 4 {
        return Err("at least 4 vertices required".to_owned());
    }
    if !mesh.normals.is_multiple_of(3) {
        return Err("normal data must be a multiple of 3".to_owned());
    }
    if !mesh.texcoords.is_multiple_of(2) {
        return Err("texcoord must be a multiple of 2".to_owned());
    }
    if mesh.texcoords != 0 && mesh.texcoords != 2 * nvert && !mesh.obj {
        return Err(
            "texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file"
                .to_owned(),
        );
    }
    let held = |id: &i64| usize::try_from(*id).is_ok_and(|id| id < nvert);
    match mesh.faces.iter().position(|id| !held(id)) {
        Some(i) => Err(format!(
            "in face {}, vertex index {} does not exist",
            i / 3,
            mesh.faces[i]
        )),
        None => Ok(()),
    }
}

/// MuJoCo 3.5.0's check of an `exact` mesh's face orientation
/// (`mjCMesh::Process`, `user_mesh.cc:1538-1563`), on its vertices as
/// written: each face whose area is above sqrt(mjMINVAL) ([`triangle`])
/// gives its three edges in its own order, and two faces walking one edge the
/// same way — the edges sorted, the first repeated one — refuse the mesh,
/// naming that edge's vertices counted from 1. A mesh of any other mode is
/// not refused for it.
fn check_face_orientation(
    vertices: &[Point3<f64>],
    indices: &[usize],
    mjcf_mesh: &MjcfMesh,
) -> Result<()> {
    if mjcf_mesh.inertia.unwrap_or_default() != MeshInertia::Exact {
        return Ok(());
    }
    let at = |i: usize| [vertices[i].x, vertices[i].y, vertices[i].z];
    let mut edges = Vec::with_capacity(indices.len());
    for face in indices.chunks_exact(3) {
        let [v0, v1, v2] = [face[0], face[1], face[2]];
        if triangle(&at(v0), &at(v1), &at(v2)).area > MJ_MINVAL.sqrt() {
            edges.extend([(v0, v1), (v1, v2), (v2, v0)]);
        }
    }
    edges.sort_unstable();
    match edges.windows(2).find(|pair| pair[0] == pair[1]) {
        Some(pair) => Err(MjcfError::asset(
            ElementKind::Mesh,
            &mjcf_mesh.name,
            format!(
                "its faces have inconsistent orientation. Please check the faces containing \
                 the vertices {} and {}.",
                pair[0].0 + 1,
                pair[0].1 + 1
            ),
        )),
        None => Ok(()),
    }
}

/// A triangle's area, unit normal and centre, as MuJoCo 3.5.0's `triangle`
/// (`user_mesh.cc:155-188`) computes them, each sum and product in its
/// order: the normal is the cross product of the edges from the first
/// vertex; where its length is below mjMINVAL the area is 0 and the normal
/// is left as that product.
#[derive(Debug, Clone, Copy)]
struct Triangle {
    area: f64,
    normal: [f64; 3],
    center: [f64; 3],
}

fn triangle(v1: &[f64; 3], v2: &[f64; 3], v3: &[f64; 3]) -> Triangle {
    let center = [
        (v1[0] + v2[0] + v3[0]) / 3.0,
        (v1[1] + v2[1] + v3[1]) / 3.0,
        (v1[2] + v2[2] + v3[2]) / 3.0,
    ];
    let b = [v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]];
    let c = [v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]];
    let normal = [
        b[1] * c[2] - b[2] * c[1],
        b[2] * c[0] - b[0] * c[2],
        b[0] * c[1] - b[1] * c[0],
    ];
    let len = dot3(&normal, &normal).sqrt();
    if len < MJ_MINVAL {
        return Triangle {
            area: 0.0,
            normal,
            center,
        };
    }
    Triangle {
        area: 0.5 * len,
        normal: [normal[0] / len, normal[1] / len, normal[2] / len],
        center,
    }
}

/// `mjuu_dot3` (`user_util.cc:126-128`).
fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// MuJoCo 3.5.0's `mjCMesh::ApplyTransformations` (`user_mesh.cc:1440-
/// 1505`) on a mesh's vertices, in its order: each moved by minus `refpos`
/// where `refpos` is not 0, turned by the transpose of `refquat`'s matrix
/// (`refquat` made unit by `mjuu_normvec`) where `refquat` is not (1, 0, 0,
/// 0), and multiplied by `scale` where `scale` is not (1, 1, 1).
// MuJoCo's tests are exact (`refpos[0] != 0 || …`).
#[allow(clippy::float_cmp)]
fn apply_transformations(vertices: &mut [Point3<f64>], mjcf_mesh: &MjcfMesh) {
    let refpos = mjcf_mesh.refpos;
    if refpos.x != 0.0 || refpos.y != 0.0 || refpos.z != 0.0 {
        for v in vertices.iter_mut() {
            v.x -= refpos.x;
            v.y -= refpos.y;
            v.z -= refpos.z;
        }
    }
    if mjcf_mesh.refquat != [1.0, 0.0, 0.0, 0.0] {
        let mut quat = mjcf_mesh.refquat;
        normvec(&mut quat);
        let mat = quat2mat(&quat);
        for v in vertices.iter_mut() {
            let [x, y, z] = mulvecmat_t(&[v.x, v.y, v.z], &mat);
            *v = Point3::new(x, y, z);
        }
    }
    let scale = mjcf_mesh.scale.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
    if scale.x != 1.0 || scale.y != 1.0 || scale.z != 1.0 {
        for v in vertices.iter_mut() {
            v.x *= scale.x;
            v.y *= scale.y;
            v.z *= scale.z;
        }
    }
}

/// A mesh's vertices moved to its centre of mass and turned to its principal
/// axes, as MuJoCo 3.5.0's `mjCMesh::Process` moves them (`user_mesh.cc:1676-
/// 1686`): each less the centre, then times the matrix of the axes'
/// conjugate (`Rotate`, `:1779-1785`).
fn to_principal_frame(vertices: &[Point3<f64>], props: &MeshProps) -> Vec<Point3<f64>> {
    let [qw, qx, qy, qz] = props.quat;
    let mat = quat2mat(&[qw, -qx, -qy, -qz]);
    let [cx, cy, cz] = props.com;
    vertices
        .iter()
        .map(|v| {
            let [x, y, z] = mulvecmat(&[v.x - cx, v.y - cy, v.z - cz], &mat);
            Point3::new(x, y, z)
        })
        .collect()
}

/// A mesh's mass properties as MuJoCo 3.5.0's `mjCMesh::Process` computes
/// them (`user_mesh.cc:1605-1668`), from its vertices as transformed, each
/// sum and product in MuJoCo's order:
///
/// - the centre of its faces, weighted by area (`ComputeFaceCentroid`,
///   `:1510-1533`): refused where their area is below mjMINVAL;
/// - its volume and centre of mass from pyramids with their apex at that
///   centre (`ComputeVolume`, `:1373-1407`), over its hull's faces under
///   `convex`, each pyramid's volume taken positive under `legacy`: refused
///   where the volume is below mjMINVAL, as negative where it is; or under
///   `shell` its surface's area and centre (`ComputeSurfaceArea`,
///   `:1412-1435`), refused below mjMINVAL;
/// - its inertia about that centre of mass (`ComputeInertia`,
///   `:1713-1775`), whose total volume (or area) replaces the one above;
/// - its principal moments and axes by `eig3`, refused where the last is not
///   positive or one exceeds the other two's sum beyond a relative 1e-6 and
///   an absolute 1e-9 (`:1653-1662`), and the half sizes of the box with
///   that volume and those moments (`:1664-1668`).
///
/// Under `convex` its faces are its hull's, made of its vertices as written,
/// as MuJoCo makes it ([`make_graph`]).
///
/// # Errors
///
/// MuJoCo's reason for refusing the mesh, without its name.
pub fn mesh_mass_properties(
    mesh: &TriangleMeshData,
    mode: MeshInertia,
) -> std::result::Result<MeshProps, &'static str> {
    let at = |p: &Point3<f64>| [p.x, p.y, p.z];
    let vertices = mesh.vertices();
    let faces: Vec<[[f64; 3]; 3]> = mesh
        .triangles()
        .iter()
        .map(|face| face.map(|v| at(&vertices[v as usize])))
        .collect();
    // The faces the volume and the inertia are summed over: the hull's under
    // `convex` (MuJoCo's `GraphFaces`), the mesh's otherwise, and the mesh's
    // too where it has no hull: after MuJoCo's warning that it could not
    // construct one, where MuJoCo reads the graph through the pointer it set
    // to null (registry `D-MESH-HULL-WARNING`).
    let solid: Vec<[[f64; 3]; 3]> = match (mode, mesh.hull()) {
        (MeshInertia::Convex, Some(hull)) => hull
            .faces
            .iter()
            .map(|face| face.map(|v| at(&vertices[v as usize])))
            .collect(),
        _ => faces.clone(),
    };
    let shell = mode == MeshInertia::Shell;
    let legacy = mode == MeshInertia::Legacy;

    let mut facecen = [0.0; 3];
    let mut total_area = 0.0;
    for [v1, v2, v3] in &faces {
        let t = triangle(v1, v2, v3);
        for (f, c) in facecen.iter_mut().zip(t.center) {
            *f += t.area * c;
        }
        total_area += t.area;
    }
    if total_area >= MJ_MINVAL {
        facecen = facecen.map(|c| c / total_area);
    }
    if total_area < MJ_MINVAL {
        return Err("mesh surface area is too small");
    }

    // A pyramid's centre of mass, three quarters of the way from its apex to
    // its base's centre.
    let pyramid_com = |center: &[f64; 3], k: usize| center[k] * 3.0 / 4.0 + facecen[k] / 4.0;
    let mut com = [0.0; 3];
    if shell {
        let mut surface = 0.0;
        for [v1, v2, v3] in &faces {
            let t = triangle(v1, v2, v3);
            surface += t.area;
            for (k, c) in com.iter_mut().enumerate() {
                *c += t.area * pyramid_com(&t.center, k);
            }
        }
        if surface >= MJ_MINVAL {
            com = com.map(|c| c / surface);
        }
        if surface < MJ_MINVAL {
            return Err("mesh surface area is too small");
        }
    } else {
        let mut volume = 0.0;
        for [v1, v2, v3] in &solid {
            let t = triangle(v1, v2, v3);
            let apex_to_face = [
                t.center[0] - facecen[0],
                t.center[1] - facecen[1],
                t.center[2] - facecen[2],
            ];
            let mut pyramid = dot3(&apex_to_face, &t.normal) * t.area / 3.0;
            if legacy {
                pyramid = pyramid.abs();
            }
            volume += pyramid;
            for (k, c) in com.iter_mut().enumerate() {
                *c += pyramid * pyramid_com(&t.center, k);
            }
        }
        if volume >= MJ_MINVAL {
            com = com.map(|c| c / volume);
        }
        if volume < MJ_MINVAL {
            return Err(if volume < 0.0 {
                "mesh volume is negative (misoriented triangles)"
            } else {
                "mesh volume is too small. Try setting inertia to shell"
            });
        }
    }

    // The products of inertia about the centre of mass, over the faces the
    // volume was summed over (the mesh's own under `shell`).
    const PAIRS: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)];
    let divisor = if shell { 12.0 } else { 20.0 };
    let centered = |v: &[f64; 3]| [v[0] - com[0], v[1] - com[1], v[2] - com[2]];
    let mut products = [0.0; 6];
    let mut volume = 0.0;
    for [v1, v2, v3] in if mode == MeshInertia::Convex {
        &solid
    } else {
        &faces
    } {
        let (d, e, f) = (centered(v1), centered(v2), centered(v3));
        let t = triangle(&d, &e, &f);
        let mut piece = if shell {
            t.area
        } else {
            dot3(&t.center, &t.normal) * t.area / 3.0
        };
        if legacy {
            piece = piece.abs();
        }
        volume += piece;
        for (p, &(a, b)) in products.iter_mut().zip(&PAIRS) {
            *p += piece / divisor
                * (2.0 * (d[a] * d[b] + e[a] * e[b] + f[a] * f[b])
                    + d[a] * e[b]
                    + d[b] * e[a]
                    + d[a] * f[b]
                    + d[b] * f[a]
                    + e[a] * f[b]
                    + e[b] * f[a]);
        }
    }
    let [pxx, pyy, pzz, pxy, pxz, pyz] = products;
    let (ixx, iyy, izz) = (pyy + pzz, pxx + pzz, pxx + pyy);
    let (ixy, ixz, iyz) = (-pxy, -pxz, -pyz);
    let (eigval, quat) = eig3(&[ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]);

    if eigval[2] <= 0.0 {
        return Err("eigenvalue of mesh inertia must be positive");
    }
    const RTOL: f64 = 1e-6;
    const ATOL: f64 = 1e-9;
    if eigval[0] + eigval[1] < eigval[2] * (1.0 - RTOL) - ATOL
        || eigval[0] + eigval[2] < eigval[1] * (1.0 - RTOL) - ATOL
        || eigval[1] + eigval[2] < eigval[0] * (1.0 - RTOL) - ATOL
    {
        return Err("eigenvalues of mesh inertia violate A + B >= C");
    }
    let boxsz = [
        0.5 * (6.0 * (eigval[1] + eigval[2] - eigval[0]) / volume).sqrt(),
        0.5 * (6.0 * (eigval[0] + eigval[2] - eigval[1]) / volume).sqrt(),
        0.5 * (6.0 * (eigval[0] + eigval[1] - eigval[2]) / volume).sqrt(),
    ];
    Ok(MeshProps {
        volume,
        com,
        quat,
        boxsz,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::{convert_mjcf_mesh, hull_refusal, mesh_mass_properties};
    use crate::builder::mesh_file::tests::create_test_stl;
    use crate::types::{MeshInertia, MjcfCompiler, MjcfMesh};
    use nalgebra::Vector3;
    use sim_core::mesh::{HullError, TriangleMeshData};

    /// The file's mesh, loaded by `convert_mjcf_mesh` with `scale`.
    fn file_mesh(path: &std::path::Path, scale: Vector3<f64>) -> TriangleMeshData {
        let mjcf_mesh = MjcfMesh {
            name: "scaled".to_string(),
            file: Some(path.to_string_lossy().to_string()),
            scale: Some(scale),
            ..MjcfMesh::default()
        };
        convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default(), false).unwrap()
    }

    /// A file mesh's `scale` multiplies its vertices.
    #[test]
    fn test_load_mesh_file_with_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("scaled.stl");
        create_test_stl(&mesh_path);

        let unscaled = file_mesh(&mesh_path, Vector3::new(1.0, 1.0, 1.0));
        let scaled = file_mesh(&mesh_path, Vector3::new(2.0, 2.0, 2.0));

        // Verify scale was applied: scaled vertices should be 2x unscaled
        // Find corresponding vertices and compare
        assert_eq!(scaled.vertices().len(), unscaled.vertices().len());
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!(
                (s.x - u.x * 2.0).abs() < 1e-10,
                "X should be scaled: {} vs {}",
                s.x,
                u.x * 2.0
            );
            assert!(
                (s.y - u.y * 2.0).abs() < 1e-10,
                "Y should be scaled: {} vs {}",
                s.y,
                u.y * 2.0
            );
            assert!(
                (s.z - u.z * 2.0).abs() < 1e-10,
                "Z should be scaled: {} vs {}",
                s.z,
                u.z * 2.0
            );
        }
    }

    /// A file mesh's non-uniform `scale` multiplies each axis by its own.
    #[test]
    fn test_load_mesh_file_nonuniform_scale() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("nonuniform.stl");
        create_test_stl(&mesh_path);

        let scaled = file_mesh(&mesh_path, Vector3::new(1.0, 2.0, 3.0));
        let unscaled = file_mesh(&mesh_path, Vector3::new(1.0, 1.0, 1.0));

        // Verify non-uniform scale: x unchanged, y doubled, z tripled
        for (s, u) in scaled.vertices().iter().zip(unscaled.vertices().iter()) {
            assert!((s.x - u.x).abs() < 1e-10, "X should be unchanged");
            assert!((s.y - u.y * 2.0).abs() < 1e-10, "Y should be 2x");
            assert!((s.z - u.z * 3.0).abs() < 1e-10, "Z should be 3x");
        }
    }

    /// Test convert_mjcf_mesh with file attribute.
    #[test]
    fn test_convert_mjcf_mesh_from_file() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("mesh.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "test_mesh".to_string(),
            file: Some(mesh_path.to_string_lossy().to_string()),
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
            ..MjcfMesh::default()
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default(), false);
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert!(!mesh_data.vertices().is_empty());
        assert!(!mesh_data.triangles().is_empty());
    }

    /// Test convert_mjcf_mesh with file attribute and relative path.
    #[test]
    fn test_convert_mjcf_mesh_relative_path() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let meshes_dir = temp_dir.path().join("assets");
        std::fs::create_dir_all(&meshes_dir).unwrap();
        let mesh_path = meshes_dir.join("model.stl");
        create_test_stl(&mesh_path);

        let mjcf_mesh = MjcfMesh {
            name: "relative_mesh".to_string(),
            file: Some("assets/model.stl".to_string()),
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
            ..MjcfMesh::default()
        };

        let result = convert_mjcf_mesh(
            &mjcf_mesh,
            Some(temp_dir.path()),
            &MjcfCompiler::default(),
            false,
        );
        assert!(result.is_ok());
    }

    /// Test convert_mjcf_mesh with embedded vertex data (no file).
    #[test]
    fn test_convert_mjcf_mesh_embedded() {
        let mjcf_mesh = MjcfMesh {
            name: "embedded".to_string(),
            file: None,
            vertex: Some(vec![
                0.0, 0.0, 0.0, // v0
                1.0, 0.0, 0.0, // v1
                0.5, 1.0, 0.0, // v2
                0.5, 0.5, 1.0, // v3
            ]),
            face: Some(vec![
                0, 1, 2, // f0
                0, 1, 3, // f1
                1, 2, 3, // f2
                2, 0, 3, // f3
            ]),
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
            ..MjcfMesh::default()
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default(), false);
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert_eq!(mesh_data.vertices().len(), 4);
        assert_eq!(mesh_data.triangles().len(), 4); // 4 triangles
    }

    /// A mesh of neither file nor vertices is refused for its count of
    /// vertices, in MuJoCo 3.5.0's words.
    #[test]
    fn test_convert_mjcf_mesh_no_data() {
        let mjcf_mesh = MjcfMesh {
            name: "empty".to_string(),
            file: None,
            vertex: None,
            face: None,
            scale: Some(Vector3::new(1.0, 1.0, 1.0)),
            ..MjcfMesh::default()
        };

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default(), false);
        assert_eq!(
            result.unwrap_err().to_string(),
            "mesh 'empty': at least 4 vertices required"
        );
    }

    /// Under `convex`, a mesh with no hull (after MuJoCo's warning, which no
    /// known input reaches) is summed over its own faces, as under `exact`:
    /// registry `D-MESH-HULL-WARNING`. The L-prism is not convex, so a hull
    /// made of it would give another volume.
    #[test]
    fn a_convex_mesh_with_no_hull_is_summed_over_its_own_faces() {
        let corners = [
            [0, 0, 0],
            [2, 0, 0],
            [2, 1, 0],
            [1, 1, 0],
            [1, 2, 0],
            [0, 2, 0],
            [0, 0, 1],
            [2, 0, 1],
            [2, 1, 1],
            [1, 1, 1],
            [1, 2, 1],
            [0, 2, 1],
        ];
        let vertices = corners
            .iter()
            .map(|c| nalgebra::Point3::from(c.map(f64::from)))
            .collect();
        let faces = vec![
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 6, 7, 8, 6, 8, 9, 6, 9, 10, 6, 10, 11, 0, 1, 7, 0,
            7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, 3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5,
            0, 6, 5, 6, 11,
        ];
        let mesh = TriangleMeshData::new(vertices, faces);
        assert!(mesh.hull().is_none());
        let convex = mesh_mass_properties(&mesh, MeshInertia::Convex).expect("summed");
        let exact = mesh_mass_properties(&mesh, MeshInertia::Exact).expect("summed");
        assert_eq!(convex, exact);
    }

    /// Each reason for no hull, as the loader takes it: qhull's in MuJoCo's
    /// words; MuJoCo's warning as going on where the mesh has faces and as a
    /// refusal in its words where it has none; MuJoCo's other checks in its
    /// words; a routine sim-qhull does not hold named in this crate's
    /// (registry `D-QHULL-UNPORTED`). No known input reaches any but the
    /// first, so each is made here.
    #[test]
    fn a_mesh_with_no_hull_is_refused_in_mujocos_words() {
        let qhull = HullError::Qhull {
            code: 6154,
            message: "initial simplex is flat",
        };
        let warning = HullError::Graph {
            message: HullError::GRAPH_WARNING,
        };
        let check = HullError::Graph {
            message: "Vertex id not found in convex hull",
        };
        let unported = HullError::Unported {
            routine: "qh_mergecycle",
        };
        assert_eq!(hull_refusal(&qhull, false).as_deref(), Some("qhull error"));
        assert_eq!(hull_refusal(&warning, false), None);
        assert_eq!(
            hull_refusal(&warning, true).as_deref(),
            Some(HullError::GRAPH_WARNING)
        );
        assert_eq!(
            hull_refusal(&check, false).as_deref(),
            Some("Vertex id not found in convex hull")
        );
        let words = hull_refusal(&unported, false).expect("refused");
        assert!(words.contains("qh_mergecycle"), "{words}");
    }
}
