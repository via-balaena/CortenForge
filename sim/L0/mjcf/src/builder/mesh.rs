//! Mesh and heightfield asset processing.
//!
//! Handles loading mesh data from files (STL, OBJ, PLY, 3MF), converting
//! embedded vertex/face data, computing mesh inertia properties, and
//! registering mesh/hfield assets in the builder's lookup tables.

use nalgebra::{Matrix3, Point3, Vector3};
use sim_core::HeightFieldData;
use sim_core::mesh::TriangleMeshData;
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use tracing::warn;

use super::asset::{AssetKind, resolve_asset_path};
use super::{ModelBuilder, ModelConversionError};
use crate::types::{MjcfCompiler, MjcfGeom, MjcfGeomType, MjcfHfield, MjcfMesh};

/// Cached mesh mass properties: `(volume, com, inertia_at_com_unit_density)`.
/// Avoids recomputing `compute_mesh_inertia()` multiple times per mesh geom.
pub type MeshProps = (f64, Vector3<f64>, Matrix3<f64>);

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
    /// Returns `ModelConversionError` if:
    /// - Mesh name is a duplicate of an already-processed mesh
    /// - Mesh conversion fails (see [`convert_mjcf_mesh`] for details)
    pub(crate) fn process_mesh(
        &mut self,
        mjcf_mesh: &MjcfMesh,
        base_path: Option<&Path>,
    ) -> std::result::Result<(), ModelConversionError> {
        // Check for duplicate mesh names
        if self.mesh_name_to_id.contains_key(&mjcf_mesh.name) {
            return Err(ModelConversionError {
                message: format!("mesh '{}': duplicate mesh name", mjcf_mesh.name),
            });
        }

        // Convert MJCF mesh to TriangleMeshData
        let mesh_data = convert_mjcf_mesh(mjcf_mesh, base_path, &self.compiler)?;

        // Register in lookup table
        let mesh_id = self.mesh_data.len();
        self.mesh_name_to_id.insert(mjcf_mesh.name.clone(), mesh_id);
        self.mesh_name.push(mjcf_mesh.name.clone());
        self.mesh_data.push(Arc::new(mesh_data));

        Ok(())
    }

    /// Process a height field asset from MJCF.
    pub(crate) fn process_hfield(
        &mut self,
        hfield: &MjcfHfield,
    ) -> std::result::Result<(), ModelConversionError> {
        if self.hfield_name_to_id.contains_key(&hfield.name) {
            return Err(ModelConversionError {
                message: format!("duplicate hfield '{}'", hfield.name),
            });
        }
        let data = convert_mjcf_hfield(hfield)?;
        let id = self.hfield_data.len();
        self.hfield_name_to_id.insert(hfield.name.clone(), id);
        self.hfield_name.push(hfield.name.clone());
        self.hfield_data.push(Arc::new(data));
        self.hfield_size.push(hfield.size);
        Ok(())
    }
}

/// Load mesh data from a file and convert to `TriangleMeshData`.
///
/// Supports STL, OBJ, PLY, and 3MF formats (auto-detected from extension).
/// Scale is applied to all vertices during conversion.
///
/// # Arguments
///
/// * `file_path` - Path string from MJCF `<mesh file="..."/>` attribute
/// * `base_path` - Base directory for resolving relative paths
/// * `scale` - Scale factors [x, y, z] to apply to vertices
/// * `mesh_name` - Mesh name for error messages
///
/// # Errors
///
/// Returns `ModelConversionError` if:
/// - Path resolution fails (see [`resolve_mesh_path`])
/// - File format is unsupported or corrupt
/// - Mesh contains no vertices or faces
pub fn load_mesh_file(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    scale: Vector3<f64>,
    mesh_name: &str,
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // 1. Resolve path using compiler path settings
    let resolved_path = resolve_asset_path(file_path, base_path, compiler, AssetKind::Mesh)?;

    // 2. Load mesh via mesh-io
    let indexed_mesh = mesh_io::load_mesh(&resolved_path).map_err(|e| ModelConversionError {
        message: format!("mesh '{mesh_name}': failed to load '{file_path}': {e}"),
    })?;

    // 3. Validate non-empty
    if indexed_mesh.vertices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{mesh_name}': file '{file_path}' contains no vertices"),
        });
    }
    if indexed_mesh.faces.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{mesh_name}': file '{file_path}' contains no faces"),
        });
    }

    // 4. Convert vertices with scale applied
    let vertices: Vec<Point3<f64>> = indexed_mesh
        .vertices
        .iter()
        .map(|v| {
            Point3::new(
                v.position.x * scale.x,
                v.position.y * scale.y,
                v.position.z * scale.z,
            )
        })
        .collect();

    // 5. Convert faces to flat indices
    //    mesh_io uses [u32; 3] per face, we need Vec<usize>
    let indices: Vec<usize> = indexed_mesh
        .faces
        .iter()
        .flat_map(|f| [f[0] as usize, f[1] as usize, f[2] as usize])
        .collect();

    // 6. Build TriangleMeshData (BVH constructed automatically)
    Ok(TriangleMeshData::new(vertices, indices))
}

/// Convert an MJCF hfield asset to `HeightFieldData`.
///
/// MuJoCo elevation formula: `vertex_z = elevation[i] × size[2]`.
/// MuJoCo vertex positions: centered at origin, x ∈ `[−size[0], +size[0]]`, y ∈ `[−size[1], +size[1]]`.
/// `HeightFieldData` uses corner-origin `(0,0)` with uniform `cell_size`.
#[allow(
    clippy::cast_precision_loss,   // nrow/ncol are small ints, well within f64 mantissa
    clippy::cast_possible_truncation, // f64→usize for grid indices after bounds checks
    clippy::cast_sign_loss,        // values are guaranteed non-negative in this context
)]
pub fn convert_mjcf_hfield(
    hfield: &MjcfHfield,
) -> std::result::Result<HeightFieldData, ModelConversionError> {
    let ncol = hfield.ncol;
    let nrow = hfield.nrow;
    let z_top = hfield.size[2];

    // Compute cell spacings
    let dx = 2.0 * hfield.size[0] / (ncol as f64 - 1.0);
    let dy = 2.0 * hfield.size[1] / (nrow as f64 - 1.0);

    // HeightFieldData requires uniform cell_size (square cells)
    let (cell_size, heights) = if (dx - dy).abs() / dx.max(dy) < 0.01 {
        // Within 1% tolerance: use average
        let cell_size = f64::midpoint(dx, dy);
        let heights: Vec<f64> = hfield.elevation.iter().map(|&e| e * z_top).collect();
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
                let e00 = hfield.elevation[iy * ncol + ix];
                let e10 = hfield.elevation[iy * ncol + ix + 1];
                let e01 = hfield.elevation[(iy + 1) * ncol + ix];
                let e11 = hfield.elevation[(iy + 1) * ncol + ix + 1];
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

/// Convert MJCF mesh asset to `TriangleMeshData`.
///
/// Handles two sources of mesh data:
/// 1. **Embedded data**: `vertex` and `face` attributes in MJCF
/// 2. **File-based**: `file` attribute pointing to STL/OBJ/PLY/3MF
///
/// Scale is applied to all vertices. BVH is built automatically.
pub fn convert_mjcf_mesh(
    mjcf_mesh: &MjcfMesh,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // Dispatch based on data source
    match (&mjcf_mesh.vertex, &mjcf_mesh.file) {
        // Embedded vertex data takes precedence (MuJoCo semantics)
        (Some(verts), _) => convert_embedded_mesh(mjcf_mesh, verts),
        // File-based loading
        (None, Some(file_path)) => load_mesh_file(
            file_path,
            base_path,
            compiler,
            mjcf_mesh.scale.unwrap_or(Vector3::new(1.0, 1.0, 1.0)),
            &mjcf_mesh.name,
        ),
        // Neither specified
        (None, None) => Err(ModelConversionError {
            message: format!(
                "mesh '{}': no vertex data and no file specified",
                mjcf_mesh.name
            ),
        }),
    }
}

/// Convert embedded mesh data from MJCF to `TriangleMeshData`.
///
/// This handles the case where vertex and face data are embedded directly
/// in the MJCF XML via `vertex="..."` and `face="..."` attributes.
pub fn convert_embedded_mesh(
    mjcf_mesh: &MjcfMesh,
    verts: &[f64],
) -> std::result::Result<TriangleMeshData, ModelConversionError> {
    // Validate vertex data
    if verts.len() % 3 != 0 {
        return Err(ModelConversionError {
            message: format!(
                "mesh '{}': vertex count ({}) not divisible by 3",
                mjcf_mesh.name,
                verts.len()
            ),
        });
    }

    // Apply scale while converting to Point3
    let scale = mjcf_mesh.scale.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
    let vertices: Vec<Point3<f64>> = verts
        .chunks_exact(3)
        .map(|chunk| Point3::new(chunk[0] * scale.x, chunk[1] * scale.y, chunk[2] * scale.z))
        .collect();

    // Extract and validate faces (convert u32 -> usize)
    let indices: Vec<usize> = match &mjcf_mesh.face {
        Some(face_data) => {
            if face_data.len() % 3 != 0 {
                return Err(ModelConversionError {
                    message: format!(
                        "mesh '{}': face index count ({}) not divisible by 3",
                        mjcf_mesh.name,
                        face_data.len()
                    ),
                });
            }
            face_data.iter().map(|&idx| idx as usize).collect()
        }
        None => {
            return Err(ModelConversionError {
                message: format!(
                    "mesh '{}': embedded vertex data requires face data",
                    mjcf_mesh.name
                ),
            });
        }
    };

    // Validate non-empty
    if vertices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{}': empty vertex array", mjcf_mesh.name),
        });
    }
    if indices.is_empty() {
        return Err(ModelConversionError {
            message: format!("mesh '{}': empty face array", mjcf_mesh.name),
        });
    }

    // Validate index bounds
    let max_idx = vertices.len();
    for (i, &idx) in indices.iter().enumerate() {
        if idx >= max_idx {
            return Err(ModelConversionError {
                message: format!(
                    "mesh '{}': face index {} at position {} out of bounds (max: {})",
                    mjcf_mesh.name,
                    idx,
                    i,
                    max_idx - 1
                ),
            });
        }
    }

    // TriangleMeshData::new() builds the BVH automatically
    Ok(TriangleMeshData::new(vertices, indices))
}

/// Compute exact mass properties of a triangle mesh using signed tetrahedron
/// decomposition (Mirtich 1996).
///
/// Returns `(volume, com, inertia_at_com)` where:
/// - `volume` is the signed volume (positive for outward-facing normals)
/// - `com` is the center of mass (assuming uniform density)
/// - `inertia_at_com` is the full 3×3 inertia tensor about the COM
///   (assuming unit density; multiply by actual density for physical values)
#[allow(clippy::suspicious_operation_groupings)] // Formulas are correct: a²+b²+c²+ab+ac+bc
pub fn compute_mesh_inertia(mesh: &TriangleMeshData) -> (f64, Vector3<f64>, Matrix3<f64>) {
    let vertices = mesh.vertices();
    let triangles = mesh.triangles();

    let mut total_volume = 0.0;
    let mut com_accum = Vector3::zeros();

    // Second-moment integrals (products of vertex coordinates over volume)
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut zz = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;

    for tri in triangles {
        let a = vertices[tri.v0].coords;
        let b = vertices[tri.v1].coords;
        let c = vertices[tri.v2].coords;

        // Signed volume of tetrahedron formed with origin: V = (a × b) · c / 6
        let det = a.cross(&b).dot(&c);
        let vol = det / 6.0;
        total_volume += vol;

        // COM contribution: centroid of tet = (a + b + c) / 4, weighted by vol
        com_accum += vol * (a + b + c) / 4.0;

        // Second-moment integrals over tetrahedron (origin, a, b, c):
        // ∫x² dV = det/60 * (a.x² + b.x² + c.x² + a.x*b.x + a.x*c.x + b.x*c.x)
        // ∫xy dV = det/120 * (2*a.x*a.y + 2*b.x*b.y + 2*c.x*c.y
        //          + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x)
        let f60 = det / 60.0;
        let f120 = det / 120.0;

        xx += f60 * (a.x * a.x + b.x * b.x + c.x * c.x + a.x * b.x + a.x * c.x + b.x * c.x);
        yy += f60 * (a.y * a.y + b.y * b.y + c.y * c.y + a.y * b.y + a.y * c.y + b.y * c.y);
        zz += f60 * (a.z * a.z + b.z * b.z + c.z * c.z + a.z * b.z + a.z * c.z + b.z * c.z);

        xy += f120
            * (2.0 * a.x * a.y
                + 2.0 * b.x * b.y
                + 2.0 * c.x * c.y
                + a.x * b.y
                + a.y * b.x
                + a.x * c.y
                + a.y * c.x
                + b.x * c.y
                + b.y * c.x);
        xz += f120
            * (2.0 * a.x * a.z
                + 2.0 * b.x * b.z
                + 2.0 * c.x * c.z
                + a.x * b.z
                + a.z * b.x
                + a.x * c.z
                + a.z * c.x
                + b.x * c.z
                + b.z * c.x);
        yz += f120
            * (2.0 * a.y * a.z
                + 2.0 * b.y * b.z
                + 2.0 * c.y * c.z
                + a.y * b.z
                + a.z * b.y
                + a.y * c.z
                + a.z * c.y
                + b.y * c.z
                + b.z * c.y);
    }

    // Zero-volume fallback: degenerate mesh (coplanar triangles, etc.)
    if total_volume.abs() < 1e-10 {
        let (aabb_min, aabb_max) = mesh.aabb();
        let extents = aabb_max - aabb_min;
        let volume = extents.x * extents.y * extents.z;
        let com = nalgebra::center(&aabb_min, &aabb_max).coords;
        // Box inertia (unit density): I_ii = V/12 * (a² + b²)
        let c = volume / 12.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(
            c * (extents.y.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.y.powi(2)),
        ));
        return (volume, com, inertia);
    }

    let com = com_accum / total_volume;

    // Build inertia tensor at origin from accumulated integrals
    // I_origin[i,i] = sum of the other two second moments (e.g., Ixx = yy + zz)
    // I_origin[i,j] = -cross_moment (e.g., Ixy = -xy)
    let i_origin = Matrix3::new(
        yy + zz,
        -xy,
        -xz, // row 0
        -xy,
        xx + zz,
        -yz, // row 1
        -xz,
        -yz,
        xx + yy, // row 2
    );

    // Shift to COM using parallel axis theorem (full tensor):
    // I_com = I_origin - V * (d·d * I₃ - d ⊗ d)
    // where d = com and V = total_volume (unit density, so mass = volume)
    let d = com;
    let d_sq = d.dot(&d);
    let parallel_shift = total_volume * (Matrix3::identity() * d_sq - d * d.transpose());
    let i_com = i_origin - parallel_shift;

    (total_volume, com, i_com)
}

/// Resolve mesh data for a geom, if it is a mesh-type geom.
pub fn resolve_mesh(
    geom: &MjcfGeom,
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
) -> Option<Arc<TriangleMeshData>> {
    if geom.geom_type.unwrap_or(MjcfGeomType::Sphere) == MjcfGeomType::Mesh {
        geom.mesh
            .as_ref()
            .and_then(|name| mesh_lookup.get(name))
            .and_then(|&id| mesh_data.get(id))
            .cloned()
    } else {
        None
    }
}
