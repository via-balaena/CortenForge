//! Indexed triangle mesh — the canonical mesh type.
//!
//! [`IndexedMesh`] stores positions only (`Vec<Point3<f64>>`). No normals, no
//! colors, no attributes. Domain-specific attributes stay in the consuming
//! layer (e.g. `mesh-types::AttributedMesh`).
//!
//! Replaces `mesh_types::IndexedMesh` and `sim_core::TriangleMeshData`.

use nalgebra::{Point3, Vector3};

use crate::bounded::Bounded;
use crate::{Aabb, Triangle};

/// Indexed triangle mesh.
///
/// Positions plus a CCW-wound face index list. Per-vertex attributes
/// (normals, colors, UVs, custom scalars) live one layer up in
/// `mesh-types::AttributedMesh`.
///
/// # Ownership lifecycle
///
/// ```text
/// mesh-io loads (owned) → mesh-repair fixes (owned, mutated)
///   → Arc::new() → sim-core, sim-bevy share (zero-copy)
/// ```
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IndexedMesh {
    /// Vertex positions.
    pub vertices: Vec<Point3<f64>>,

    /// Triangle faces as indices into the vertex array.
    /// Each face is `[v0, v1, v2]` with counter-clockwise winding.
    pub faces: Vec<[u32; 3]>,
}

impl IndexedMesh {
    /// Creates a new empty mesh.
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Creates a mesh with pre-allocated capacity.
    #[inline]
    #[must_use]
    pub fn with_capacity(vertex_count: usize, face_count: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_count),
            faces: Vec::with_capacity(face_count),
        }
    }

    /// Creates a mesh from vertices and faces.
    #[inline]
    #[must_use]
    pub const fn from_parts(vertices: Vec<Point3<f64>>, faces: Vec<[u32; 3]>) -> Self {
        Self { vertices, faces }
    }

    /// Creates a mesh from raw coordinate and index data.
    ///
    /// `positions` is a flat `[x0, y0, z0, x1, y1, z1, ...]` array.
    /// `indices` is a flat `[v0a, v1a, v2a, v0b, v1b, v2b, ...]` array.
    ///
    /// Returns an empty mesh if either array length is not divisible by 3.
    #[must_use]
    pub fn from_raw(positions: &[f64], indices: &[u32]) -> Self {
        if !positions.len().is_multiple_of(3) || !indices.len().is_multiple_of(3) {
            return Self::new();
        }

        let vertices = positions
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();

        let faces = indices
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();

        Self { vertices, faces }
    }

    // --- Topology (replaces MeshTopology trait — only 1 implementor) ---

    /// Returns the number of vertices.
    #[inline]
    #[must_use]
    pub const fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the number of faces.
    #[inline]
    #[must_use]
    pub const fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Returns `true` if the mesh has no vertices or no faces.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.vertices.is_empty() || self.faces.is_empty()
    }

    /// Returns a face by index, or `None` if out of bounds.
    #[inline]
    #[must_use]
    pub fn face(&self, index: usize) -> Option<[u32; 3]> {
        self.faces.get(index).copied()
    }

    /// Returns a triangle with resolved vertex positions, or `None` if
    /// the face index is out of bounds.
    #[must_use]
    pub fn triangle(&self, face_index: usize) -> Option<Triangle> {
        self.faces.get(face_index).map(|&[i0, i1, i2]| Triangle {
            v0: self.vertices[i0 as usize],
            v1: self.vertices[i1 as usize],
            v2: self.vertices[i2 as usize],
        })
    }

    /// Iterates over all triangles with resolved vertex positions.
    pub fn triangles(&self) -> impl Iterator<Item = Triangle> + '_ {
        self.faces.iter().map(|&[i0, i1, i2]| Triangle {
            v0: self.vertices[i0 as usize],
            v1: self.vertices[i1 as usize],
            v2: self.vertices[i2 as usize],
        })
    }

    /// Returns the vertex positions as a slice.
    #[inline]
    #[must_use]
    pub fn positions(&self) -> &[Point3<f64>] {
        &self.vertices
    }

    // --- Geometric queries ---

    /// Computes the signed volume of the mesh using the divergence theorem.
    ///
    /// For a closed mesh with outward-facing normals (CCW winding from
    /// outside), this returns a positive value. Negative means inside-out.
    /// Near-zero means the mesh is not closed or has inconsistent winding.
    #[must_use]
    pub fn signed_volume(&self) -> f64 {
        let mut volume = 0.0;

        for &[i0, i1, i2] in &self.faces {
            let v0 = &self.vertices[i0 as usize];
            let v1 = &self.vertices[i1 as usize];
            let v2 = &self.vertices[i2 as usize];

            // Signed volume of tetrahedron with origin = (v0 · (v1 × v2)) / 6
            let cross = Vector3::new(
                v1.y.mul_add(v2.z, -(v1.z * v2.y)),
                v1.z.mul_add(v2.x, -(v1.x * v2.z)),
                v1.x.mul_add(v2.y, -(v1.y * v2.x)),
            );
            volume += v0.z.mul_add(cross.z, v0.x.mul_add(cross.x, v0.y * cross.y));
        }

        volume / 6.0
    }

    /// Computes the absolute volume of the mesh.
    #[inline]
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.signed_volume().abs()
    }

    /// Returns `true` if the mesh appears inside-out (negative signed volume).
    #[inline]
    #[must_use]
    pub fn is_inside_out(&self) -> bool {
        self.signed_volume() < 0.0
    }

    /// Computes the total surface area.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        self.triangles().map(|tri| tri.area()).sum()
    }

    /// Computes a face normal for each face (unit-length, CCW winding).
    ///
    /// Returns a zero vector for degenerate faces.
    #[must_use]
    pub fn compute_face_normals(&self) -> Vec<Vector3<f64>> {
        self.faces
            .iter()
            .map(|&[i0, i1, i2]| {
                let v0 = &self.vertices[i0 as usize];
                let v1 = &self.vertices[i1 as usize];
                let v2 = &self.vertices[i2 as usize];

                let e1 = v1 - v0;
                let e2 = v2 - v0;
                let cross = e1.cross(&e2);
                let len_sq = cross.norm_squared();

                if len_sq > f64::EPSILON {
                    cross / len_sq.sqrt()
                } else {
                    Vector3::zeros()
                }
            })
            .collect()
    }

    /// Computes area-weighted vertex normals by averaging adjacent face normals.
    ///
    /// Each face's contribution is weighted by its area (the unnormalized
    /// cross product). The result is normalized per vertex. Vertices with
    /// no adjacent faces get a zero vector.
    #[must_use]
    pub fn compute_vertex_normals(&self) -> Vec<Vector3<f64>> {
        let mut normals = vec![Vector3::zeros(); self.vertices.len()];

        for &[i0, i1, i2] in &self.faces {
            let v0 = &self.vertices[i0 as usize];
            let v1 = &self.vertices[i1 as usize];
            let v2 = &self.vertices[i2 as usize];

            // Unnormalized cross product = area-weighted normal
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let face_normal = e1.cross(&e2);

            normals[i0 as usize] += face_normal;
            normals[i1 as usize] += face_normal;
            normals[i2 as usize] += face_normal;
        }

        for n in &mut normals {
            let len_sq = n.norm_squared();
            if len_sq > f64::EPSILON {
                *n /= len_sq.sqrt();
            }
        }

        normals
    }

    // --- Transforms ---

    /// Translates the mesh by the given offset.
    pub fn translate(&mut self, offset: Vector3<f64>) {
        for v in &mut self.vertices {
            *v += offset;
        }
    }

    /// Scales the mesh uniformly around the origin.
    pub fn scale(&mut self, factor: f64) {
        for v in &mut self.vertices {
            v.coords *= factor;
        }
    }

    /// Scales the mesh uniformly around its center.
    pub fn scale_centered(&mut self, factor: f64) {
        let center = self.aabb().center();
        for v in &mut self.vertices {
            *v = center + (*v - center) * factor;
        }
    }

    // --- Combinators ---

    /// Merges another mesh into this one, adjusting face indices.
    // Index/count conversion bounded by domain (mesh/grid size < 2^32).
    #[allow(clippy::cast_possible_truncation)]
    pub fn merge(&mut self, other: &Self) {
        let vertex_offset = self.vertices.len() as u32;

        self.vertices.extend_from_slice(&other.vertices);

        for &face in &other.faces {
            self.faces.push([
                face[0] + vertex_offset,
                face[1] + vertex_offset,
                face[2] + vertex_offset,
            ]);
        }
    }

    /// Reserves capacity for additional vertices and faces.
    pub fn reserve(&mut self, additional_vertices: usize, additional_faces: usize) {
        self.vertices.reserve(additional_vertices);
        self.faces.reserve(additional_faces);
    }
}

impl Bounded for IndexedMesh {
    fn aabb(&self) -> Aabb {
        if self.vertices.is_empty() {
            return Aabb::empty();
        }
        Aabb::from_points(self.vertices.iter())
    }
}
