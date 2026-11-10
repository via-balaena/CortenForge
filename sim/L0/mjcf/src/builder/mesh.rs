//! Mesh and heightfield asset processing.
//!
//! Handles loading mesh data from files (STL, OBJ, PLY; also 3MF when
//! the `threemf` feature is enabled), converting embedded vertex/face
//! data, computing mesh inertia properties, and registering mesh/hfield
//! assets in the builder's lookup tables.

use nalgebra::{Point3, Vector3};
use sim_core::HeightFieldData;
use sim_core::mesh::TriangleMeshData;
use std::path::Path;
use std::sync::Arc;
use tracing::warn;

use super::ModelBuilder;
use super::asset::{AssetKind, resolve_asset_path};
use super::eig3::eig3;
use super::mass::MJ_MINVAL;
use super::orientation::{mulvecmat_t, normvec, quat2mat};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{MeshInertia, MjcfCompiler, MjcfHfield, MjcfMesh, MjcfModel};

/// Gives each unnamed mesh and height field the name MuJoCo 3.5.0 gives it
/// (`mjCMesh::CopyFromSpec`, `mjCHField::CopyFromSpec`): its `file`'s text
/// past the last `/` or `\`, cut at the last `.` (`mjuu_strippath`,
/// `mjuu_stripext`, `user_util.cc:768-829`), an empty one refused as
/// MuJoCo refuses it ("empty name in mesh", "empty name in height field",
/// `user_model.cc:2034-2041`), and two of one name refused as a repeated
/// name, as MuJoCo's `ProcessLists` refuses them before any pass drops an
/// unused mesh (`user_model.cc:4888-4891`). It runs on the model before
/// validation and every compiler pass, which find assets by name.
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
    let repeated = |kind: ElementKind, names: Vec<&str>| {
        let mut seen = std::collections::HashSet::new();
        names
            .into_iter()
            .find(|name| !seen.insert(*name))
            .map_or(Ok(()), |name| Err(MjcfError::duplicate_name(kind, name)))
    };
    repeated(
        ElementKind::Mesh,
        mjcf.meshes.iter().map(|m| m.name.as_str()).collect(),
    )?;
    repeated(
        ElementKind::Hfield,
        mjcf.hfields.iter().map(|h| h.name.as_str()).collect(),
    )?;
    Ok(())
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
    /// Converts `MjcfMesh` to `TriangleMeshData` with BVH and registers it
    /// in the mesh lookup table. The mesh can then be referenced by geoms.
    ///
    /// # Arguments
    ///
    /// * `mjcf_mesh` - The MJCF mesh definition
    /// * `base_path` - Base directory for resolving relative mesh file paths
    ///
    /// # Errors
    ///
    /// Returns [`MjcfError::DuplicateName`] if the mesh's name is an
    /// already-processed mesh's, and the conversion's error if it fails (see
    /// [`convert_mjcf_mesh`]).
    pub(crate) fn process_mesh(
        &mut self,
        mjcf_mesh: &MjcfMesh,
        base_path: Option<&Path>,
    ) -> Result<()> {
        // Check for duplicate mesh names
        if self.mesh_name_to_id.contains_key(&mjcf_mesh.name) {
            return Err(MjcfError::duplicate_name(
                ElementKind::Mesh,
                &mjcf_mesh.name,
            ));
        }

        // Convert MJCF mesh to TriangleMeshData
        let mut mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler)?;

        // Compute convex hull at build time (matching MuJoCo's MakeGraph())
        let max_hull_vert = mjcf_mesh.maxhullvert;
        mesh_data.compute_convex_hull(max_hull_vert);

        let props = mesh_mass_properties(&mesh_data, mjcf_mesh.inertia.unwrap_or_default())
            .map_err(|reason| MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason))?;

        // Register in lookup table
        let mesh_id = self.mesh_data.len();
        self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
        self.mesh_name.push(mjcf_mesh.name.clone());
        self.mesh_data.push(Arc::new(mesh_data));
        self.mesh_props.push(props);

        Ok(())
    }

    /// Process a height field asset from MJCF.
    ///
    /// Supports two data sources:
    /// - **File-based:** `file` attribute pointing to a PNG image
    /// - **Inline:** `nrow`, `ncol`, `elevation` attributes in the XML
    pub(crate) fn process_hfield(
        &mut self,
        hfield: &MjcfHfield,
        base_path: Option<&Path>,
    ) -> Result<()> {
        if self.hfield_name_to_id.contains_key(&hfield.name) {
            return Err(MjcfError::duplicate_name(ElementKind::Hfield, &hfield.name));
        }
        let data = convert_mjcf_hfield(hfield, base_path, &self.compiler)?;
        let id = self.hfield_data.len();
        self.hfield_name_to_id.insert(hfield.name.clone(), id);
        self.hfield_name.push(hfield.name.clone());
        self.hfield_data.push(Arc::new(data));
        self.hfield_size.push(hfield.size);
        Ok(())
    }

    /// MuJoCo 3.5.0's check of each height field, in order: its four sizes
    /// positive (`mjCHField::Compile`, `user_objects.cc:4471-4474`), used
    /// or not. The parser has refused a first or second size that is not
    /// positive and a third or fourth that is negative; this refuses a third
    /// or fourth of 0, where MuJoCo does, after the bodies' checks and
    /// before the tendons' (`user_model.cc:4932-4956`).
    pub(crate) fn check_hfield_sizes(&self) -> Result<()> {
        let not_positive = |size: &[f64; 4]| size.iter().any(|&s| s <= 0.0);
        match self.hfield_size.iter().position(not_positive) {
            Some(h) => Err(
                MjcfError::invalid_model("size parameter is not positive in hfield").at(
                    Location::nth(ElementKind::Hfield, Some(&self.hfield_name[h]), h),
                ),
            ),
            None => Ok(()),
        }
    }
}

/// A mesh's vertices and its faces' vertex indices (three per face), as its
/// file or its `vertex` and `face` attributes hold them, before MuJoCo's
/// transformations ([`apply_transformations`]).
pub type RawMesh = (Vec<Point3<f64>>, Vec<usize>);

/// Load a mesh file's vertices and faces as the file holds them.
///
/// Supports STL, OBJ, and PLY formats out of the box (auto-detected from
/// extension). Enable the `threemf` feature to add 3MF; otherwise `.3mf`
/// paths return an `UnknownFormat` error from mesh-io.
///
/// # Arguments
///
/// * `file_path` - Path string from MJCF `<mesh file="..."/>` attribute
/// * `base_path` - Base directory for resolving relative paths
/// * `mesh_name` - Mesh name for error messages
///
/// # Errors
///
/// Returns an [`MjcfError::Asset`] if:
/// - Path resolution fails (see [`resolve_asset_path`])
/// - File format is unsupported or corrupt
/// - Mesh contains no vertices or faces
pub fn load_mesh_file(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    mesh_name: &str,
) -> Result<RawMesh> {
    // 1. Resolve path using compiler path settings
    let refuse = |reason: String| MjcfError::asset(ElementKind::Mesh, mesh_name, reason);
    let resolved_path =
        resolve_asset_path(file_path, base_path, compiler, AssetKind::Mesh).map_err(refuse)?;

    // 2. Load mesh via mesh-io
    let indexed_mesh = mesh_io::load_mesh(&resolved_path)
        .map_err(|e| refuse(format!("failed to load '{file_path}': {e}")))?;

    // 3. Validate non-empty
    if indexed_mesh.vertices.is_empty() {
        return Err(refuse(format!("file '{file_path}' contains no vertices")));
    }
    if indexed_mesh.faces.is_empty() {
        return Err(refuse(format!("file '{file_path}' contains no faces")));
    }

    // 4. Faces to flat indices: mesh-io holds [u32; 3] per face.
    let indices: Vec<usize> = indexed_mesh
        .faces
        .iter()
        .flat_map(|f| [f[0] as usize, f[1] as usize, f[2] as usize])
        .collect();

    // 5. MuJoCo 3.5.0's loaders hand a file's vertices to `ProcessVertices`
    // as floats (`user_mesh.cc:543`; tinyobjloader's for OBJ, the files'
    // own for STL and MSH), so each is rounded to one, as an embedded
    // mesh's are when they are read.
    #[allow(clippy::cast_possible_truncation)] // the rounding is MuJoCo's
    let vertices: Vec<Point3<f64>> = indexed_mesh
        .vertices
        .iter()
        .map(|v| v.map(|c| f64::from(c as f32)))
        .collect();

    // 6. Each face's vertices exist (`CheckInitialMesh`,
    // `user_mesh.cc:1835-1841`), in MuJoCo's words.
    if let Some(i) = indices.iter().position(|&index| index >= vertices.len()) {
        return Err(refuse(format!(
            "in face {}, vertex index {} does not exist",
            i / 3,
            indices[i]
        )));
    }

    Ok((vertices, indices))
}

/// Convert an MJCF hfield asset to `HeightFieldData`.
///
/// Handles two data sources:
/// 1. **File-based:** `file` attribute → load PNG, derive nrow/ncol/elevation
///    (requires the `hfields` feature; otherwise returns a clear error)
/// 2. **Inline:** `nrow`/`ncol`/`elevation` attributes in XML (always available)
///
/// MuJoCo elevation formula: `vertex_z = elevation[i] × size[2]`.
/// MuJoCo vertex positions: centered at origin, x ∈ `[−size[0], +size[0]]`, y ∈ `[−size[1], +size[1]]`.
/// `HeightFieldData` uses corner-origin `(0,0)` with uniform `cell_size`.
// nrow/ncol are small grid extents (well within f64 mantissa); f64→usize
// truncations sit behind explicit bounds checks; sign-loss casts apply to
// values already proven non-negative upstream.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
pub fn convert_mjcf_hfield(
    hfield: &MjcfHfield,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> Result<HeightFieldData> {
    // Resolve data source: file-based or inline
    let (ncol, nrow, elevation) = match (&hfield.file, &hfield.elevation) {
        // Inline elevation data takes precedence (consistent with mesh semantics)
        (_, Some(elev)) => {
            let at = || Location::named(ElementKind::Hfield, &hfield.name);
            let nr = hfield
                .nrow
                .ok_or_else(|| MjcfError::missing_attribute("hfield", "nrow").at(at()))?;
            let nc = hfield
                .ncol
                .ok_or_else(|| MjcfError::missing_attribute("hfield", "ncol").at(at()))?;
            (nc, nr, elev.clone())
        }
        // File-based loading (requires `hfields` feature)
        #[cfg(feature = "hfields")]
        (Some(file_path), None) => load_hfield_png(file_path, base_path, compiler, &hfield.name)?,
        #[cfg(not(feature = "hfields"))]
        (Some(file_path), None) => {
            // Suppress unused-parameter warnings; load_hfield_png isn't compiled in.
            let _ = (base_path, compiler);
            return Err(MjcfError::unsupported(format!(
                "hfield file=\"{file_path}\" without the `hfields` feature \
                 (enable `sim-mjcf/hfields` to load PNG height fields, or use \
                 inline `elevation=\"...\"` data)",
            ))
            .at(Location::named(ElementKind::Hfield, &hfield.name)));
        }
        // Neither (should not happen — parser validates)
        (None, None) => {
            return Err(MjcfError::asset(
                ElementKind::Hfield,
                &hfield.name,
                "no elevation data and no file specified",
            ));
        }
    };

    let z_top = hfield.size[2];

    // Compute cell spacings
    let dx = 2.0 * hfield.size[0] / (ncol as f64 - 1.0);
    let dy = 2.0 * hfield.size[1] / (nrow as f64 - 1.0);

    // HeightFieldData requires uniform cell_size (square cells)
    let (cell_size, heights) = if (dx - dy).abs() / dx.max(dy) < 0.01 {
        // Within 1% tolerance: use average
        let cell_size = f64::midpoint(dx, dy);
        let heights: Vec<f64> = elevation.iter().map(|&e| e * z_top).collect();
        (cell_size, heights)
    } else {
        // Non-square: resample to finer resolution
        warn!(
            hfield_name = %hfield.name,
            dx, dy,
            "hfield has non-square cells (dx={dx:.4}, dy={dy:.4}), resampling to uniform grid"
        );
        let cell_size = dx.min(dy);
        let new_ncol = ((2.0 * hfield.size[0]) / cell_size).round() as usize + 1;
        let new_nrow = ((2.0 * hfield.size[1]) / cell_size).round() as usize + 1;

        let mut heights = Vec::with_capacity(new_nrow * new_ncol);
        for row in 0..new_nrow {
            let fy = row as f64 / (new_nrow as f64 - 1.0) * (nrow as f64 - 1.0);
            let iy = (fy as usize).min(nrow - 2);
            let ty = fy - iy as f64;
            for col in 0..new_ncol {
                let fx = col as f64 / (new_ncol as f64 - 1.0) * (ncol as f64 - 1.0);
                let ix = (fx as usize).min(ncol - 2);
                let tx = fx - ix as f64;
                // Bilinear interpolation
                let e00 = elevation[iy * ncol + ix];
                let e10 = elevation[iy * ncol + ix + 1];
                let e01 = elevation[(iy + 1) * ncol + ix];
                let e11 = elevation[(iy + 1) * ncol + ix + 1];
                let e = e00 * (1.0 - tx) * (1.0 - ty)
                    + e10 * tx * (1.0 - ty)
                    + e01 * (1.0 - tx) * ty
                    + e11 * tx * ty;
                heights.push(e * z_top);
            }
        }
        return Ok(HeightFieldData::new(heights, new_ncol, new_nrow, cell_size));
    };

    Ok(HeightFieldData::new(heights, ncol, nrow, cell_size))
}

/// Load hfield elevation data from a PNG image file.
///
/// MuJoCo behavior (from `user_model.cc`):
/// - Loads image as grayscale
/// - Normalizes pixel values to `[0.0, 1.0]`: 8-bit → `/255`, 16-bit → `/65535`
/// - `nrow` = image height, `ncol` = image width
///
/// Returns `(ncol, nrow, elevation)`.
// 8-bit/16-bit pixel intensities and grid dimensions are far below 2^52.
#[cfg(feature = "hfields")]
#[allow(clippy::cast_precision_loss)]
fn load_hfield_png(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    hfield_name: &str,
) -> Result<(usize, usize, Vec<f64>)> {
    // Resolve path using the same rules as mesh files (hfields use meshdir)
    let refuse = |reason: String| MjcfError::asset(ElementKind::Hfield, hfield_name, reason);
    let resolved_path =
        resolve_asset_path(file_path, base_path, compiler, AssetKind::Mesh).map_err(refuse)?;

    // Load image
    let img = image::open(&resolved_path)
        .map_err(|e| refuse(format!("failed to load '{file_path}': {e}")))?;

    // Convert to 16-bit grayscale for maximum precision
    let gray = img.into_luma16();
    let ncol = gray.width() as usize;
    let nrow = gray.height() as usize;

    if nrow < 2 || ncol < 2 {
        return Err(refuse(format!(
            "image dimensions ({ncol}x{nrow}) must be >= 2x2"
        )));
    }

    // Normalize to [0.0, 1.0]
    let elevation: Vec<f64> = gray.pixels().map(|p| f64::from(p.0[0]) / 65535.0).collect();

    Ok((ncol, nrow, elevation))
}

/// Convert MJCF mesh asset to `TriangleMeshData`.
///
/// Handles two sources of mesh data:
/// 1. **Embedded data**: `vertex` and `face` attributes in MJCF
/// 2. **File-based**: `file` attribute pointing to STL/OBJ/PLY (also
///    3MF when the `threemf` feature is enabled)
///
/// Its vertices are moved, turned and scaled as MuJoCo 3.5.0 does
/// ([`apply_transformations`]); the BVH is built over them.
pub fn convert_mjcf_mesh(
    mjcf_mesh: &MjcfMesh,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> Result<TriangleMeshData> {
    // Dispatch based on data source
    let (mut vertices, indices) = match (&mjcf_mesh.vertex, &mjcf_mesh.file) {
        // Embedded vertex data takes precedence (MuJoCo semantics)
        (Some(verts), _) => convert_embedded_mesh(mjcf_mesh, verts)?,
        // File-based loading
        (None, Some(file_path)) => load_mesh_file(file_path, base_path, compiler, &mjcf_mesh.name)?,
        // Neither specified
        (None, None) => {
            return Err(MjcfError::asset(
                ElementKind::Mesh,
                &mjcf_mesh.name,
                "no vertex data and no file specified",
            ));
        }
    };
    check_face_orientation(&vertices, &indices, mjcf_mesh)?;
    apply_transformations(&mut vertices, mjcf_mesh);
    Ok(TriangleMeshData::new(vertices, indices))
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

/// An embedded mesh's vertices and faces, from its `vertex` and `face`
/// attributes, checked and as written.
pub fn convert_embedded_mesh(mjcf_mesh: &MjcfMesh, verts: &[f64]) -> Result<RawMesh> {
    let refuse = |reason: String| MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason);
    // Validate vertex data
    if !verts.len().is_multiple_of(3) {
        return Err(refuse(format!(
            "vertex count ({}) not divisible by 3",
            verts.len()
        )));
    }

    let vertices: Vec<Point3<f64>> = verts
        .chunks_exact(3)
        .map(|chunk| Point3::new(chunk[0], chunk[1], chunk[2]))
        .collect();

    // Extract and validate faces (convert u32 -> usize)
    let indices: Vec<usize> = match &mjcf_mesh.face {
        Some(face_data) => {
            if face_data.len() % 3 != 0 {
                return Err(refuse(format!(
                    "face index count ({}) not divisible by 3",
                    face_data.len()
                )));
            }
            face_data.iter().map(|&idx| idx as usize).collect()
        }
        None => {
            return Err(refuse("embedded vertex data requires face data".into()));
        }
    };

    // Validate non-empty
    if vertices.is_empty() {
        return Err(refuse("empty vertex array".into()));
    }
    if indices.is_empty() {
        return Err(refuse("empty face array".into()));
    }

    // Validate index bounds
    let max_idx = vertices.len();
    for (i, &idx) in indices.iter().enumerate() {
        if idx >= max_idx {
            return Err(refuse(format!(
                "face index {idx} at position {i} out of bounds (max: {})",
                max_idx - 1
            )));
        }
    }

    Ok((vertices, indices))
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
/// Its hull's faces under `convex` are this crate's hull's, equal to
/// MuJoCo's as sets where they agree, not in order or in how a flat facet is
/// cut into triangles (book Rigid-loading L36), so its numbers there are
/// MuJoCo's to rounding, not to the bit.
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
    // too where this crate's hull builder makes no hull, which it does for a
    // solid thinner than qhull's tolerance (measured: a 1e-10-thick plate,
    // whose hull is its own faces).
    let solid: Vec<[[f64; 3]; 3]> = match (mode, mesh.convex_hull()) {
        (MeshInertia::Convex, Some(hull)) => hull
            .faces
            .iter()
            .map(|face| face.map(|v| at(&hull.vertices[v as usize])))
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
    use super::{convert_mjcf_mesh, load_mesh_file};
    use crate::types::{MjcfCompiler, MjcfMesh};
    use nalgebra::Vector3;
    use sim_core::mesh::TriangleMeshData;

    /// Helper to create a simple STL file for testing.
    fn create_test_stl(path: &std::path::Path) {
        use mesh_types::{IndexedMesh, Point3};

        let mut mesh = IndexedMesh::new();
        // Simple tetrahedron (4 vertices, 4 faces)
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]); // base
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left

        mesh_io::save_mesh(&mesh, path).expect("Failed to save test STL");
    }

    /// Test load_mesh_file with STL format.
    #[test]
    fn test_load_mesh_file_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        create_test_stl(&mesh_path);

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            "test_mesh",
        );

        assert!(result.is_ok(), "Should load STL file");
        let (vertices, indices) = result.unwrap();
        assert!(vertices.len() >= 4, "Should have at least 4 vertices");
        assert!(indices.len() >= 12, "Should have at least 4 triangles");
    }

    /// The file's mesh, loaded by `convert_mjcf_mesh` with `scale`.
    fn file_mesh(path: &std::path::Path, scale: Vector3<f64>) -> TriangleMeshData {
        let mjcf_mesh = MjcfMesh {
            name: "scaled".to_string(),
            file: Some(path.to_string_lossy().to_string()),
            scale: Some(scale),
            ..MjcfMesh::default()
        };
        convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default()).unwrap()
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

    /// Test load_mesh_file fails for unsupported format.
    #[test]
    fn test_load_mesh_file_unsupported_format() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.xyz");
        std::fs::write(&mesh_path, b"invalid mesh data").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            "unsupported",
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("failed to load"));
    }

    /// Test load_mesh_file fails gracefully for corrupt STL data.
    ///
    /// A file with `.stl` extension but invalid content should produce
    /// a clear error, not panic.
    #[test]
    fn test_load_mesh_file_corrupt_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("corrupt.stl");

        // Write invalid STL data (valid STL needs specific binary/ASCII structure)
        std::fs::write(&mesh_path, b"this is not a valid stl file content").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            "corrupt_mesh",
        );

        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("corrupt_mesh") && err_msg.contains("failed to load"),
            "error should identify mesh and indicate load failure: {err_msg}"
        );
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

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
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

        let result = convert_mjcf_mesh(&mjcf_mesh, Some(temp_dir.path()), &MjcfCompiler::default());
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

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_ok());
        let mesh_data = result.unwrap();
        assert_eq!(mesh_data.vertices().len(), 4);
        assert_eq!(mesh_data.triangles().len(), 4); // 4 triangles
    }

    /// Test convert_mjcf_mesh fails when neither file nor vertex is present.
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

        let result = convert_mjcf_mesh(&mjcf_mesh, None, &MjcfCompiler::default());
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("no vertex data"));
    }
}
