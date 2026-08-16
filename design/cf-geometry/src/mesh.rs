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
    /// For a closed mesh whose winding is **consistent**, this is a property of
    /// the mesh alone: positive for outward-facing normals (CCW seen from
    /// outside), negative for an inside-out one.
    ///
    /// ⚠ **Consistency is a precondition, not something this can check.** The
    /// sum is over origin-apex tetrahedra, so each face's term scales with its
    /// distance from the world origin. Under a translation `t` the total moves
    /// by `t · Σ A_f n_f`; on a closed, consistently wound mesh that vector is
    /// zero and the volume is a property of the mesh alone. Break the winding
    /// and it generally is not, so the answer becomes a fact about where the
    /// caller put the origin. A negative or near-zero result is therefore *not*
    /// evidence of inconsistent winding, and a positive one is not evidence
    /// against it.
    ///
    /// ⚠ Neither is stability under translation, in either direction. `Σ A_f
    /// n_f` is a *vector*: transporting along a direction orthogonal to it
    /// changes nothing at any magnitude, and two opposed flips cancel in the
    /// sum outright, leaving a defective mesh perfectly frame-invariant. Both
    /// are measured in `tests/mesh_tests.rs` — see
    /// `translation_invariance_is_not_evidence_of_consistent_winding`.
    ///
    /// Measured in this crate by
    /// `one_flipped_face_makes_the_global_volume_test_frame_dependent`
    /// (`tests/mesh_tests.rs`): flip one face of twelve on a unit cube and this
    /// returns `+0.667` at the origin — positive, reads clean, wholly blind —
    /// then `-332.67` translated 1e3 **along that face's normal**, the scale
    /// anatomical meshes are kept at in their native frames. Transported 1e6
    /// the other way it still reads `+0.667`, which is why the direction is
    /// stated and not just the distance. Corroborated downstream by `mesh-loft`'s
    /// `the_global_inside_out_test_changes_its_answer_under_a_2mm_translation`,
    /// where a puck with 4 inconsistent edges returns *exactly* the correct
    /// unit-box volume of `1.0` at the origin and flips verdict after 2 mm.
    ///
    /// ★ The local instrument is `mesh_repair::winding_census` — per-edge,
    /// seed-free and coordinate-free.
    ///
    /// ⚠ **A clean census is necessary, not sufficient.** It only ever compares
    /// faces that share an edge, so it never compares two *disjoint* shells: a
    /// mesh whose second shell is wound backwards reports zero inconsistent
    /// edges while this function quietly nets one shell against the other.
    /// Orientation across components needs a per-component verdict —
    /// `cf_fsu_geometry::SurfaceReport`'s `inward_facing_components` is the
    /// worked example. (And a per-shell verdict is not itself a defect test: a
    /// genuine enclosed cavity is *correctly* wound inward.)
    ///
    /// `cf-geometry` can call none of them, and not by oversight: `mesh-repair`
    /// depends on this crate, so the dependency would be a cycle. Judge winding
    /// one layer up, where these are reachable.
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
    ///
    /// ⚠ This is `abs()` of [`Self::signed_volume`], so it inherits that
    /// method's precondition *and* discards the sign that would at least have
    /// hinted at trouble. On the flipped unit cube measured there it returns
    /// `0.667` at the origin and `332.67` translated 1e3 — neither of which is
    /// the volume of anything.
    #[inline]
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.signed_volume().abs()
    }

    /// Returns `true` if the signed volume is negative.
    ///
    /// ⚠ **That is the same thing as "inside-out" only on a consistently wound
    /// closed mesh**, and this cannot tell you whether you have one — see
    /// [`Self::signed_volume`] for the measured failure in both directions
    /// (blind to a local flip near the origin, firing on one far from it).
    /// Establish consistent winding first and read this as a verdict only then;
    /// `mesh-loft` gates it on `Bushing::is_closed_and_consistent()`, which is
    /// backed by `mesh_repair::winding_census`.
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
