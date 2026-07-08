//! `SdfMeshedTetMesh` — `Mesh` impl backed by the BCC + Labelle-Shewchuk
//! Isosurface Stuffing pipeline.
//!
//! [`SdfMeshedTetMesh::from_sdf`] runs the full pipeline:
//!
//! 1. Build the BCC lattice spanning `hints.bbox` at `hints.cell_size`
//!    via [`BccLattice::new`].
//! 2. Sample the caller-supplied SDF at every lattice vertex in
//!    sequential `VertexId` order. Any non-finite value (`NaN` or
//!    `±inf`) surfaces as [`MeshingError::NonFiniteSdfValue`] before
//!    any mesh state is constructed; sequential iteration pins the
//!    failing-vertex diagnostic deterministically (smallest tripping
//!    `VertexId` is reported).
//! 3. Apply the SDF sign convention adapter (scope memo §3 Decision A):
//!    sim-soft uses negative-inside per the standard SDF convention,
//!    while the Labelle-Shewchuk paper uses positive-inside. The
//!    sampled value is negated before being handed to warp + stuffing,
//!    so the paper's stencil tables apply directly.
//! 4. Apply the warp step in place via [`stuffing::warp_lattice`]
//!    (scope memo §3 Decision M D-11 deterministic 14-edge walk).
//! 5. Walk BCC tets in [`BccLattice::tets`] order and dispatch each
//!    through [`stuffing::dispatch_case`] with a single shared
//!    `BTreeMap` cut cache (scope memo §3 Decision M D-9
//!    sorted-pair-with-sublattice-tag key).
//! 6. If no sub-tet was emitted, surface [`MeshingError::EmptyMesh`].
//!    The output mesh retains every lattice vertex in
//!    `output_positions[..n_lattice]`; lattice vertices in BCC tets
//!    that fell entirely outside the SDF (trivial `n_in == 0` case)
//!    are unreferenced "orphans" by design, filtered downstream by
//!    `mesh::referenced_vertices` per scope memo §3 Decision K
//!    post-pivot revision.
//! 7. Compute per-tet [`QualityMetrics`] via
//!    `mesh::quality::compute_metrics`.
//!
//! Memo cite: scope memo §2 file plan + §3 Decision A (algorithm),
//! Decision H (constructor signature + error variants — no
//! `NegativeVolumeTet` variant; orientation is by-construction
//! right-handed, the D-10 backstop silently drops sub-volume-floor
//! sub-tets), Decision I (`QualityMetrics` four `Vec<f64>`), Decision
//! J (`MeshAdjacency` unit struct), Decision M (D-8/D-9/D-10/D-11).

use std::collections::BTreeMap;

use nalgebra::Point3;

use crate::Vec3;
use crate::material::{BuildableFromField, MaterialField, NeoHookean, Yeoh};
use crate::mesh::{
    Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, boundary_faces_from_topology,
    interface_flags_from_field, materials_from_field, quality,
};

use super::MeshingHints;
use super::lattice::BccLattice;
use super::sdf::Sdf;
use super::stuffing::{self, EdgeKey};

/// Tet mesh built by sampling an [`Sdf`] over a BCC lattice.
///
/// Each lattice tet is dispatched through the Labelle-Shewchuk
/// Isosurface Stuffing case table; implements the [`Mesh`] trait so
/// it plugs into [`crate::CpuTet4NHSolver`] alongside `SingleTetMesh`
/// and `HandBuiltTetMesh`.
///
/// Generic over `M: BuildableFromField` so the same pipeline produces
/// either NH or Yeoh per-tet caches. NH consumers omit the type
/// parameter (defaults to [`NeoHookean`]); Yeoh consumers (row 23+)
/// write `SdfMeshedTetMesh<Yeoh>` and use [`SdfMeshedTetMesh::from_sdf_yeoh`]
/// per arc memo D10.
#[derive(Clone, Debug)]
pub struct SdfMeshedTetMesh<M: BuildableFromField = NeoHookean> {
    vertices: Vec<Vec3>,
    tets: Vec<[VertexId; 4]>,
    adj: MeshAdjacency,
    q: QualityMetrics,
    material_cache: Vec<M>,
    interface_flags: Vec<bool>,
    boundary_faces: Vec<[VertexId; 3]>,
}

/// Errors returned by [`SdfMeshedTetMesh::from_sdf`].
///
/// Per scope memo §3 Decision H there is **no** `NegativeVolumeTet`
/// variant: under correct BCC + warp + stuffing operation, sub-tets
/// are emitted right-handed by construction, and the
/// `stuffing::EPSILON_VOLUME` D-10 defensive backstop silently
/// drops any sub-tet that emerges below the volume floor. Any
/// surviving non-positive-volume tet is a structural algorithm bug
/// surfaced by III-2's strict `signed_volume > 0` assertion, not by
/// `MeshingError`.
#[derive(Clone, Debug)]
pub enum MeshingError {
    /// The mesher produced no sub-tets. Typically a `bbox` placed far
    /// from the SDF's zero set so every BCC tet falls into the
    /// trivial `n_inside == 0` case.
    EmptyMesh,
    /// The SDF returned a non-finite value (`NaN` or `±inf`) at the
    /// reported lattice vertex. Detection runs in sequential
    /// `VertexId` order — the smallest tripping `VertexId` is
    /// returned, making the diagnostic deterministic.
    NonFiniteSdfValue {
        /// Lattice `VertexId` whose SDF sample tripped the check.
        vertex_id: VertexId,
        /// Raw value returned by the SDF in sim-soft's
        /// negative-inside convention (i.e., pre-negation). Reported
        /// as-returned so the caller can match against their
        /// `Sdf::eval` impl directly.
        value: f64,
    },
}

/// Internal generic builder shared by [`SdfMeshedTetMesh<NeoHookean>::from_sdf`]
/// (NH) and [`SdfMeshedTetMesh<Yeoh>::from_sdf_yeoh`] (Yeoh). Per-`M`
/// public entry points avoid the type-inference papercut that bare
/// `from_sdf` would trigger when both NH and Yeoh impls coexist.
fn build<M: BuildableFromField>(
    sdf: &dyn Sdf,
    hints: &MeshingHints,
    material_field: &MaterialField,
) -> Result<SdfMeshedTetMesh<M>, MeshingError> {
    let lattice = BccLattice::new(hints);
    let n_lattice = lattice.positions.len();

    // Step 2 + 3: sample SDF in sequential VertexId order, detect
    // non-finite, then negate per Decision A SDF sign convention
    // adapter (sim-soft negative-inside → paper positive-inside).
    // The sequential walk pins the failing-vertex diagnostic — the
    // first non-finite value is the smallest `VertexId` that trips.
    let mut sdf_values: Vec<f64> = Vec::with_capacity(n_lattice);
    for (vid, position) in lattice.positions.iter().enumerate() {
        let raw = sdf.eval(Point3::from(*position));
        if !raw.is_finite() {
            // BccLattice::new caps `n_lattice` at i32-safe range; the
            // u32 cast is in range by construction.
            #[allow(clippy::cast_possible_truncation)]
            let vertex_id = vid as VertexId;
            return Err(MeshingError::NonFiniteSdfValue {
                vertex_id,
                value: raw,
            });
        }
        sdf_values.push(-raw);
    }

    // Step 4: warp displaces near-boundary lattice vertices in place.
    // We work on a clone so the lattice itself stays anchored to its
    // unwarped points (`BccLattice::position_of` etc. would otherwise
    // diverge from `warped_positions`).
    let mut warped_positions: Vec<Vec3> = lattice.positions.clone();
    stuffing::warp_lattice(&lattice, &mut warped_positions, &mut sdf_values);

    // Step 5: walk BCC tets and dispatch each through the stuffing
    // case table. `output_positions` starts with the warped-lattice
    // prefix copied in so lattice `VertexId`s in `tet_vids` index
    // directly into it; cut points appended by `get_or_insert_cut`
    // get fresh `VertexId`s starting at `output_positions.len()`.
    let mut output_positions: Vec<Vec3> = warped_positions.clone();
    let mut output_tets: Vec<[VertexId; 4]> = Vec::new();
    let mut cut_cache: BTreeMap<EdgeKey, VertexId> = BTreeMap::new();

    for &tet_vids in &lattice.tets {
        stuffing::dispatch_case(
            tet_vids,
            &lattice,
            &warped_positions,
            &sdf_values,
            &mut output_positions,
            &mut output_tets,
            &mut cut_cache,
        );
    }

    // Step 6: empty-mesh detection (typically a bbox far from the
    // SDF zero set; canonical sphere parameters never trip this).
    if output_tets.is_empty() {
        return Err(MeshingError::EmptyMesh);
    }

    // Step 7: per-tet QualityMetrics. Computed once at construction
    // (Decision I) so III-1 can assert bit-equality across runs.
    let q = quality::compute_metrics(&output_positions, &output_tets);

    // Step 8: per-tet material cache. Centroid-sampled from
    // `material_field` per Part 7 §02 §00 + scope memo Decision K.
    // Walked in `tet_id` order via `materials_from_field`; orphan
    // lattice vertices are unreferenced and contribute nothing.
    let material_cache: Vec<M> =
        materials_from_field(&output_positions, &output_tets, material_field);

    // Step 9: per-tet interface-flag cache. `|φ(x_c)| < L_e`
    // rule per Part 7 §02 §01 + scope memo Decision K (commit 12,
    // IV-6); diagnostic-only — Newton hot path does not branch.
    // All-`false` payload of length `n_tets` when
    // `material_field` carries no interface SDF (uniform /
    // `LayeredScalarField`-only fields go through this path).
    let interface_flags =
        interface_flags_from_field(&output_positions, &output_tets, material_field);

    // Step 10: per-mesh boundary-face cache. Pure topology;
    // outward winding inherits from the right-handed sub-tets the
    // stuffing pipeline emits by construction (Decision H — no
    // negative-volume tet survives the D-10 backstop).
    let boundary_faces = boundary_faces_from_topology(&output_tets);

    Ok(SdfMeshedTetMesh {
        vertices: output_positions,
        tets: output_tets,
        adj: MeshAdjacency,
        q,
        material_cache,
        interface_flags,
        boundary_faces,
    })
}

impl<M: BuildableFromField + Clone> SdfMeshedTetMesh<M> {
    /// Return a copy keeping only the tets of the **largest face-connected
    /// component**.
    ///
    /// The BCC isosurface-stuffing pipeline fragments a sub-cell-thin feature — e.g.
    /// a lens-shaped intervertebral disc's tapering rim — into a main body plus many
    /// small disconnected islands. Those islands are structurally unconstrained
    /// (free rigid-body modes), so they poison the tangent's conditioning (a
    /// near-singular Newton system) and render as scattered surface fragments. A
    /// physical solid is a single connected component, so filtering to the largest
    /// one restores that model invariant.
    ///
    /// Two tets are connected when they share a triangular face. Vertices are
    /// retained as-is — the now-unreferenced island vertices are handled downstream by
    /// [`referenced_vertices`](crate::mesh::referenced_vertices), exactly as the
    /// pipeline's own orphan lattice corners already are. The per-tet caches
    /// (materials, interface flags) are subset to the kept tets; the boundary-face and
    /// quality caches are recomputed from them.
    #[must_use]
    pub fn largest_component(&self) -> Self {
        // Sorted triangular faces of a tet (its four opposite-a-vertex faces).
        fn faces(t: &[VertexId; 4]) -> [[VertexId; 3]; 4] {
            let s = |mut f: [VertexId; 3]| {
                f.sort_unstable();
                f
            };
            [
                s([t[1], t[2], t[3]]),
                s([t[0], t[2], t[3]]),
                s([t[0], t[1], t[3]]),
                s([t[0], t[1], t[2]]),
            ]
        }
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]]; // path halving
                x = parent[x];
            }
            x
        }

        let n = self.tets.len();
        let mut parent: Vec<usize> = (0..n).collect();
        // A face shared by two tets unions them.
        let mut owner: std::collections::HashMap<[VertexId; 3], usize> =
            std::collections::HashMap::new();
        for (ti, tet) in self.tets.iter().enumerate() {
            for f in faces(tet) {
                if let Some(&other) = owner.get(&f) {
                    let (ra, rb) = (find(&mut parent, ti), find(&mut parent, other));
                    parent[rb] = ra;
                } else {
                    owner.insert(f, ti);
                }
            }
        }
        let roots: Vec<usize> = (0..n).map(|t| find(&mut parent, t)).collect();
        let mut per_root: std::collections::HashMap<usize, usize> =
            std::collections::HashMap::new();
        for &r in &roots {
            *per_root.entry(r).or_default() += 1;
        }
        // Largest by tet count; DETERMINISTIC tie-break on the smallest root index
        // (HashMap iteration order is not stable, so a plain `max_by_key` would pick an
        // arbitrary component run-to-run when two are equal-sized).
        let Some((&largest, _)) = per_root
            .iter()
            .max_by(|(root_a, count_a), (root_b, count_b)| {
                count_a.cmp(count_b).then_with(|| root_b.cmp(root_a))
            })
        else {
            return self.clone(); // no tets — nothing to filter
        };

        let keep: Vec<usize> = (0..n).filter(|&t| roots[t] == largest).collect();
        let tets: Vec<[VertexId; 4]> = keep.iter().map(|&t| self.tets[t]).collect();
        let material_cache: Vec<M> = keep
            .iter()
            .map(|&t| self.material_cache[t].clone())
            .collect();
        let interface_flags: Vec<bool> = keep.iter().map(|&t| self.interface_flags[t]).collect();
        let boundary_faces = boundary_faces_from_topology(&tets);
        let q = quality::compute_metrics(&self.vertices, &tets);
        Self {
            vertices: self.vertices.clone(),
            tets,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
        }
    }
}

impl SdfMeshedTetMesh<NeoHookean> {
    /// Build an NH mesh by running the BCC + Labelle-Shewchuk
    /// Isosurface Stuffing pipeline (see module doc).
    ///
    /// When `hints.material_field` is `None`, falls back to the IV-1
    /// baseline (Ecoflex 00-30 compressible regime, μ=1e5 / λ=4e5)
    /// via [`MaterialField::skeleton_default`] — a single named
    /// constant so consumers grep cleanly.
    ///
    /// # Errors
    ///
    /// - [`MeshingError::EmptyMesh`] when no sub-tet is emitted
    ///   across the full lattice walk.
    /// - [`MeshingError::NonFiniteSdfValue`] at SDF sampling time
    ///   when `sdf.eval(p)` returns `NaN` or `±inf` on any lattice
    ///   vertex; reports the smallest tripping `VertexId`.
    ///
    /// # Panics
    ///
    /// Forwards `BccLattice::new`'s panics for invalid `hints` (non-
    /// positive `cell_size`, ill-formed `bbox`, or a `bbox`
    /// degenerate enough to yield zero cubes along some axis). These
    /// are caller-supplied invariants, not runtime errors; the
    /// canonical Phase 3 sphere parameters never trip them.
    ///
    /// Also panics if `hints.material_field` is a Yeoh-variant field —
    /// call [`SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh`] instead.
    pub fn from_sdf(sdf: &dyn Sdf, hints: &MeshingHints) -> Result<Self, MeshingError> {
        let fallback = MaterialField::skeleton_default();
        let material_field = hints.material_field.as_ref().unwrap_or(&fallback);
        build(sdf, hints, material_field)
    }
}

impl SdfMeshedTetMesh<Yeoh> {
    /// Build a Yeoh mesh by running the BCC + Labelle-Shewchuk
    /// Isosurface Stuffing pipeline. Mirror of
    /// [`SdfMeshedTetMesh::<NeoHookean>::from_sdf`] for Yeoh consumers
    /// (arc memo D10).
    ///
    /// `hints.material_field` MUST be `Some` and built via
    /// [`MaterialField::from_yeoh_fields`] — there is no
    /// "Yeoh skeleton default" since Yeoh requires per-row C₂
    /// calibration that no synthesized fallback could honestly carry.
    ///
    /// # Errors
    ///
    /// Same set as [`SdfMeshedTetMesh::<NeoHookean>::from_sdf`].
    ///
    /// # Panics
    ///
    /// - All the same `BccLattice::new` invariants.
    /// - Panics if `hints.material_field` is `None` (the Yeoh path
    ///   has no fallback).
    /// - Panics if `hints.material_field` is `Some` but built via NH
    ///   constructors — call
    ///   [`SdfMeshedTetMesh::<NeoHookean>::from_sdf`] instead.
    // Build-time API contract: Yeoh has no skeleton default, so a
    // missing material_field is a caller error worth panicking on at
    // construction. Documented in the # Panics section above.
    #[allow(clippy::expect_used)]
    pub fn from_sdf_yeoh(sdf: &dyn Sdf, hints: &MeshingHints) -> Result<Self, MeshingError> {
        let material_field = hints.material_field.as_ref().expect(
            "SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh requires hints.material_field to be Some \
             — there is no Yeoh skeleton default. Pass MaterialField::from_yeoh_fields(...) \
             via the hints",
        );
        build(sdf, hints, material_field)
    }
}

impl<M: BuildableFromField> Mesh<M> for SdfMeshedTetMesh<M> {
    fn n_tets(&self) -> usize {
        self.tets.len()
    }

    fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4] {
        let idx = tet as usize;
        assert!(
            idx < self.tets.len(),
            "tet ID {idx} out of bounds for {n}-tet SdfMeshedTetMesh",
            n = self.tets.len(),
        );
        self.tets[idx]
    }

    fn positions(&self) -> &[Vec3] {
        &self.vertices
    }

    fn adjacency(&self) -> &MeshAdjacency {
        &self.adj
    }

    fn quality(&self) -> &QualityMetrics {
        &self.q
    }

    fn materials(&self) -> &[M] {
        &self.material_cache
    }

    fn interface_flags(&self) -> &[bool] {
        &self.interface_flags
    }

    fn boundary_faces(&self) -> &[[VertexId; 3]] {
        &self.boundary_faces
    }

    // Mirrors `HandBuiltTetMesh::equals_structurally`: same vertex
    // count + same tet count + same per-tet vertex indices in tet-id
    // order. Positions deliberately excluded — those are the
    // change-detection signal, not structural identity (Part 11 Ch 00
    // §02 mesh claim 3).
    //
    // `as TetId` cast is the Mesh-trait API tax: `n_tets()` returns
    // `usize`, `tet_vertices()` takes `TetId = u32`. Phase 3 meshes
    // stay well below `u32::MAX` tets.
    #[allow(clippy::cast_possible_truncation)]
    fn equals_structurally(&self, other: &dyn Mesh<M>) -> bool {
        if self.n_tets() != other.n_tets() {
            return false;
        }
        if self.n_vertices() != other.n_vertices() {
            return false;
        }
        for tet_id in 0..self.n_tets() as TetId {
            if self.tet_vertices(tet_id) != other.tet_vertices(tet_id) {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod largest_component_tests {
    #![allow(clippy::expect_used, clippy::cast_possible_truncation)]

    use super::{MeshingHints, SdfMeshedTetMesh};
    use crate::Vec3;
    use crate::material::MaterialField;
    use crate::mesh::{Mesh, TetId, VertexId};
    use crate::sdf_bridge::{Aabb3, Sdf, SphereSdf};
    use nalgebra::Point3;

    /// A two-solid SDF (min of two spheres) so meshing yields ≥2 disconnected
    /// components — the exact shape [`SdfMeshedTetMesh::largest_component`] filters.
    struct TwoSpheres {
        a_c: Vec3,
        a_r: f64,
        b_c: Vec3,
        b_r: f64,
    }

    impl Sdf for TwoSpheres {
        fn eval(&self, p: Point3<f64>) -> f64 {
            let da = (p.coords - self.a_c).norm() - self.a_r;
            let db = (p.coords - self.b_c).norm() - self.b_r;
            da.min(db) // union of two solids
        }
        fn grad(&self, p: Point3<f64>) -> Vec3 {
            let da = (p.coords - self.a_c).norm() - self.a_r;
            let db = (p.coords - self.b_c).norm() - self.b_r;
            let c = if da <= db { self.a_c } else { self.b_c };
            (p.coords - c).normalize()
        }
    }

    /// Number of face-connected tet components (independent of the method's own
    /// union-find, so it cross-checks rather than mirrors it).
    fn n_components(mesh: &SdfMeshedTetMesh) -> usize {
        fn sorted_face(tet: &[VertexId; 4], idx: [usize; 3]) -> [VertexId; 3] {
            let mut tri = [tet[idx[0]], tet[idx[1]], tet[idx[2]]];
            tri.sort_unstable();
            tri
        }
        fn find(parent: &mut [usize], mut node: usize) -> usize {
            while parent[node] != node {
                parent[node] = parent[parent[node]];
                node = parent[node];
            }
            node
        }
        let n_tets = mesh.n_tets();
        let mut parent: Vec<usize> = (0..n_tets).collect();
        let mut owner: std::collections::HashMap<[VertexId; 3], usize> =
            std::collections::HashMap::new();
        for ti in 0..n_tets {
            let tet = mesh.tet_vertices(ti as TetId);
            for tri in [
                sorted_face(&tet, [1, 2, 3]),
                sorted_face(&tet, [0, 2, 3]),
                sorted_face(&tet, [0, 1, 3]),
                sorted_face(&tet, [0, 1, 2]),
            ] {
                if let Some(&other) = owner.get(&tri) {
                    let (root_a, root_b) = (find(&mut parent, ti), find(&mut parent, other));
                    parent[root_b] = root_a;
                } else {
                    owner.insert(tri, ti);
                }
            }
        }
        (0..n_tets)
            .map(|ti| find(&mut parent, ti))
            .collect::<std::collections::HashSet<_>>()
            .len()
    }

    fn hints(min: Vec3, max: Vec3) -> MeshingHints {
        MeshingHints {
            bbox: Aabb3::new(min, max),
            cell_size: 0.02,
            material_field: Some(MaterialField::uniform(1.0e5, 4.0e5)),
        }
    }

    #[test]
    fn largest_component_keeps_only_the_biggest_connected_solid() {
        // A big sphere (r=0.10) and a disjoint small one (r=0.04) well apart.
        let sdf = TwoSpheres {
            a_c: Vec3::new(-0.15, 0.0, 0.0),
            a_r: 0.10,
            b_c: Vec3::new(0.22, 0.0, 0.0),
            b_r: 0.04,
        };
        let mesh = SdfMeshedTetMesh::from_sdf(
            &sdf,
            &hints(Vec3::new(-0.28, -0.13, -0.13), Vec3::new(0.28, 0.13, 0.13)),
        )
        .expect("two-sphere scene meshes");
        assert!(
            n_components(&mesh) >= 2,
            "two disjoint spheres must mesh as ≥2 components, got {}",
            n_components(&mesh)
        );

        let filtered = mesh.largest_component();
        assert_eq!(
            n_components(&filtered),
            1,
            "filter must leave exactly one connected component"
        );
        assert!(
            filtered.n_tets() < mesh.n_tets(),
            "filter must drop the smaller sphere's tets ({} vs {})",
            filtered.n_tets(),
            mesh.n_tets()
        );
        assert!(
            filtered.n_tets() > mesh.n_tets() / 2,
            "the kept component must be the LARGER sphere (majority of tets)"
        );
        // Per-tet caches are subset consistently; vertices retained (orphans kept,
        // handled downstream by referenced_vertices); boundary faces stay in range.
        assert_eq!(filtered.materials().len(), filtered.n_tets());
        assert_eq!(filtered.n_vertices(), mesh.n_vertices());
        let nv = filtered.n_vertices();
        assert!(
            filtered
                .boundary_faces()
                .iter()
                .flatten()
                .all(|&v| (v as usize) < nv),
            "every filtered boundary-face vertex must index into the vertex buffer"
        );
    }

    #[test]
    fn largest_component_is_a_noop_on_a_single_solid() {
        let mesh = SdfMeshedTetMesh::from_sdf(
            &SphereSdf { radius: 0.1 },
            &hints(Vec3::new(-0.13, -0.13, -0.13), Vec3::new(0.13, 0.13, 0.13)),
        )
        .expect("sphere scene meshes");
        assert_eq!(n_components(&mesh), 1, "a sphere is one component");
        let filtered = mesh.largest_component();
        assert_eq!(
            filtered.n_tets(),
            mesh.n_tets(),
            "a single-component mesh must lose no tets"
        );
        assert_eq!(n_components(&filtered), 1);
    }
}
