//! The live 3D scan/mold viewer — the headless, testable half.
//!
//! Mesh loading (→ smooth per-vertex normals + a bounding sphere) and the
//! [`OrbitCamera`] (→ a model-view-projection matrix) are pure math with
//! no GPU dependency, so they live here and are unit-tested. The wgpu
//! pipeline that turns a [`MeshData`] + [`OrbitCamera`] into a rendered
//! `slint::Image` lives in `main.rs` — it needs Slint's device/queue and
//! can't run in a headless test.
//!
//! This is the viewer first proven in the `spike-wgpu` bin, folded in once
//! the render + live drag-orbit were verified on a real scan.

use std::path::Path;

use nalgebra::{Matrix4, Perspective3, Point3, Vector3};

/// Vertical field of view, radians-worth expressed in degrees at use site.
const FOV_Y_DEG: f32 = 50.0;
/// Drag sensitivity: screen-pixels → radians of orbit.
const ORBIT_SPEED: f32 = 0.01;
/// Clamp on elevation so the camera can't flip over the poles.
const ELEV_LIMIT: f32 = 1.5;
/// Pull-back slack so the framed mesh doesn't touch the viewport edges.
const FRAME_MARGIN: f32 = 1.4;

/// One mesh vertex as uploaded to the GPU: position + smooth normal.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    /// Object-space position.
    pub pos: [f32; 3],
    /// Normalized smooth vertex normal.
    pub normal: [f32; 3],
}

/// The shader's uniform block: a single model-view-projection matrix
/// (column-major, matching WGSL `mat4x4`).
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    /// Column-major MVP.
    pub mvp: [f32; 16],
}

/// A CPU-side mesh ready to upload, plus its bounding sphere (center +
/// radius) for auto-framing the camera.
#[derive(Debug, Clone)]
pub struct MeshData {
    /// Interleaved position + normal vertices.
    pub vertices: Vec<Vertex>,
    /// Triangle indices into `vertices`.
    pub indices: Vec<u32>,
    /// Bounding-box center (the orbit target).
    pub center: [f32; 3],
    /// Half the bounding-box diagonal (the framing radius); always > 0.
    pub radius: f32,
}

impl MeshData {
    /// Number of indices, as the draw call wants it.
    #[must_use]
    pub fn index_count(&self) -> u32 {
        self.indices.len() as u32
    }
}

/// Load a mesh file (STL/OBJ/PLY/… via `mesh-io`) into render-ready
/// [`MeshData`] with smooth normals and a framing sphere.
///
/// # Errors
/// The loader's error message if the file is missing or unparseable.
pub fn load_mesh_data(path: &Path) -> Result<MeshData, String> {
    let mesh = mesh_io::load_mesh(path).map_err(|e| e.to_string())?;
    let positions: Vec<[f32; 3]> = mesh
        .vertices
        .iter()
        .map(|v| [v.x as f32, v.y as f32, v.z as f32])
        .collect();
    Ok(mesh_data_from_geometry(&positions, &mesh.faces))
}

/// Build render-ready [`MeshData`] from an in-memory [`mesh_types::IndexedMesh`] — the
/// live working mesh the step-2 [`EditSession`] mutates. Re-derived after
/// every edit so the viewport reflects the current state.
///
/// [`EditSession`]: cf_studio_engine::EditSession
#[must_use]
pub fn mesh_data_from_indexed(mesh: &mesh_types::IndexedMesh) -> MeshData {
    let positions: Vec<[f32; 3]> = mesh
        .vertices
        .iter()
        .map(|v| [v.x as f32, v.y as f32, v.z as f32])
        .collect();
    mesh_data_from_geometry(&positions, &mesh.faces)
}

/// Build render-ready [`MeshData`] from raw positions + triangle faces.
/// Smooth (area-weighted) normals; bounding sphere from the AABB.
///
/// Exposed so the GPU-free geometry math is unit-testable without a file.
#[must_use]
pub fn mesh_data_from_geometry(positions: &[[f32; 3]], faces: &[[u32; 3]]) -> MeshData {
    if positions.is_empty() {
        return MeshData {
            vertices: Vec::new(),
            indices: Vec::new(),
            center: [0.0, 0.0, 0.0],
            radius: 1.0,
        };
    }
    let pts: Vec<Point3<f32>> = positions
        .iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();

    // Smooth normals: sum each face's (un-normalized, so area-weighted)
    // normal into its three vertices, then normalize.
    let mut normals = vec![Vector3::<f32>::zeros(); pts.len()];
    for f in faces {
        let (a, b, c) = (pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize]);
        let face_n = (b - a).cross(&(c - a));
        for &i in f {
            normals[i as usize] += face_n;
        }
    }

    let vertices: Vec<Vertex> = pts
        .iter()
        .zip(normals.iter())
        .map(|(p, n)| {
            let nn = if n.norm() > 1e-12 {
                n.normalize()
            } else {
                Vector3::y()
            };
            Vertex {
                pos: [p.x, p.y, p.z],
                normal: [nn.x, nn.y, nn.z],
            }
        })
        .collect();
    let indices: Vec<u32> = faces.iter().flat_map(|f| f.iter().copied()).collect();

    let mut min = Point3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max = Point3::new(f32::MIN, f32::MIN, f32::MIN);
    for p in &pts {
        min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    let center = nalgebra::center(&min, &max);
    let radius = ((max - min).norm() / 2.0).max(1e-3);

    MeshData {
        vertices,
        indices,
        center: [center.x, center.y, center.z],
        radius,
    }
}

/// An orbit camera framing a target point.
#[derive(Debug, Clone, Copy)]
pub struct OrbitCamera {
    /// Horizontal angle (radians).
    pub azimuth: f32,
    /// Vertical angle (radians), clamped away from the poles.
    pub elevation: f32,
    /// Distance from the target.
    pub distance: f32,
    /// The point the camera looks at and orbits around.
    pub target: [f32; 3],
}

impl OrbitCamera {
    /// Frame a bounding sphere: pick a distance that fits `radius` in the
    /// vertical FOV, with a little margin, at a pleasant 3/4 angle.
    #[must_use]
    pub fn framing(center: [f32; 3], radius: f32) -> Self {
        let fov = FOV_Y_DEG.to_radians();
        let distance = radius / (fov / 2.0).sin() * FRAME_MARGIN;
        Self {
            azimuth: 0.6,
            elevation: 0.5,
            distance,
            target: center,
        }
    }

    /// Apply a mouse-drag delta (in screen pixels): horizontal drag spins
    /// azimuth, vertical drag tilts elevation (clamped).
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.azimuth -= dx * ORBIT_SPEED;
        self.elevation = (self.elevation + dy * ORBIT_SPEED).clamp(-ELEV_LIMIT, ELEV_LIMIT);
    }

    /// Column-major MVP for a square viewport, near/far derived from
    /// `radius` so the mesh is never clipped. Includes the OpenGL→wgpu
    /// depth-range correction (z ∈ `[-1,1]` → `[0,1]`).
    ///
    /// **+Z is up** — the cast-frame convention (`UpAxis::PlusZ`, the
    /// demolding direction). A scan auto-oriented with its long axis along
    /// +Z therefore stands vertical in the viewport, matching the
    /// cf-scan-prep tool. Elevation is the angle above the XY plane;
    /// azimuth sweeps around +Z.
    #[must_use]
    pub fn mvp(&self, radius: f32) -> [f32; 16] {
        let fov = FOV_Y_DEG.to_radians();
        let (ce, se) = (self.elevation.cos(), self.elevation.sin());
        let (ca, sa) = (self.azimuth.cos(), self.azimuth.sin());
        let target = Point3::new(self.target[0], self.target[1], self.target[2]);
        let dir = Vector3::new(ce * sa, ce * ca, se);
        let eye = target + dir * self.distance;
        let view = Matrix4::look_at_rh(&eye, &target, &Vector3::z());

        let near = (self.distance - radius).max(self.distance * 0.05);
        let far = self.distance + radius * 2.0;
        let proj = Perspective3::new(1.0, fov, near, far).to_homogeneous();

        let correction = Matrix4::new(
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.5, 0.5, //
            0.0, 0.0, 0.0, 1.0,
        );

        let mvp = correction * proj * view;
        let mut out = [0.0_f32; 16];
        out.copy_from_slice(mvp.as_slice());
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A unit CCW triangle in the z=0 plane → normals point +z.
    const TRI: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    const TRI_FACES: [[u32; 3]; 1] = [[0, 1, 2]];

    #[test]
    fn geometry_builds_vertices_indices_and_upward_normals() {
        let m = mesh_data_from_geometry(&TRI, &TRI_FACES);
        assert_eq!(m.vertices.len(), 3);
        assert_eq!(m.indices, vec![0, 1, 2]);
        assert_eq!(m.index_count(), 3);
        for v in &m.vertices {
            // CCW in the xy-plane → +z normal.
            assert!(
                (v.normal[2] - 1.0).abs() < 1e-5,
                "normal up: {:?}",
                v.normal
            );
        }
    }

    #[test]
    fn bounding_sphere_centers_and_sizes_the_mesh() {
        let m = mesh_data_from_geometry(&TRI, &TRI_FACES);
        assert!((m.center[0] - 0.5).abs() < 1e-5);
        assert!((m.center[1] - 0.5).abs() < 1e-5);
        assert!((m.center[2]).abs() < 1e-5);
        assert!(m.radius > 0.0);
    }

    #[test]
    fn empty_geometry_is_safe() {
        let m = mesh_data_from_geometry(&[], &[]);
        assert!(m.vertices.is_empty());
        assert_eq!(m.radius, 1.0, "non-zero radius keeps the camera math sane");
    }

    #[test]
    fn framing_sits_outside_the_sphere() {
        let c = OrbitCamera::framing([0.0, 0.0, 0.0], 2.0);
        assert!(c.distance > 2.0, "camera pulled back beyond the radius");
        assert_eq!(c.target, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn orbit_spins_azimuth_and_clamps_elevation() {
        let mut c = OrbitCamera::framing([0.0, 0.0, 0.0], 1.0);
        let a0 = c.azimuth;
        c.orbit(10.0, 0.0);
        assert!((c.azimuth - (a0 - 10.0 * ORBIT_SPEED)).abs() < 1e-6);

        c.orbit(0.0, 100_000.0); // huge upward drag
        assert!(c.elevation <= ELEV_LIMIT, "elevation clamped at the pole");
        c.orbit(0.0, -1_000_000.0);
        assert!(
            c.elevation >= -ELEV_LIMIT,
            "elevation clamped at the other pole"
        );
    }

    #[test]
    fn mvp_is_sixteen_finite_floats() {
        let c = OrbitCamera::framing([0.0, 0.0, 0.0], 1.0);
        let m = c.mvp(1.0);
        assert_eq!(m.len(), 16);
        assert!(m.iter().all(|x| x.is_finite()), "no NaN/inf in the MVP");
    }
}
