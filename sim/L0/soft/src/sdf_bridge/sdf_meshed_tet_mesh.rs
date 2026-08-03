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

use std::collections::{BTreeMap, HashMap, HashSet};

use nalgebra::{Point3, SMatrix};

use crate::Vec3;
use crate::element::{Element, Tet4};
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
    /// The BCC isosurface-stuffing pipeline can fragment a solid into a main body plus
    /// many small disconnected islands. Those islands are structurally unconstrained
    /// (free rigid-body modes), so they poison the tangent's conditioning (a
    /// near-singular Newton system) and render as scattered surface fragments. A
    /// physical solid is a single connected component, so filtering to the largest
    /// one restores that model invariant.
    ///
    /// ⚠ **This doc used to attribute the fragmentation to "a sub-cell-thin feature —
    /// e.g. a lens-shaped intervertebral disc's tapering rim". That attribution was never
    /// measured, and for the FSU disc it is now measured to be WRONG.** The islands there
    /// are material the stuffer emitted *outside* the solid, because the caller built its
    /// signed-distance oracle on a mesh rescaled to SI metres — a frame in which parry
    /// silently skips triangles under an absolute area floor and returns a zero
    /// pseudo-normal, which its inside test reads as "inside". See `mesh-sdf`'s
    /// `pseudo_normal_sign_is_exact_across_the_scale_regime_consumers_use` for the regime,
    /// and `cf-fsu-model`'s `mesh_stability_instrument_check_fom` for the chain.
    ///
    /// So treat a large discarded fraction as a **symptom to investigate upstream**, not as
    /// expected behaviour on thin features. Whether a correctly-signed oracle still
    /// fragments a genuinely sub-cell-thin rim is **open** — nobody has run it.
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

    /// Return a copy with the given nodes projected toward new positions, backing
    /// each move off just far enough to keep every incident tet's rest Jacobian
    /// above a fraction `quality_floor` of its original value — so no element
    /// inverts or collapses to a sliver.
    ///
    /// `moves` lists `(vertex, target)` pairs; each named vertex is moved toward
    /// its `target`. This is the Tet4 companion to
    /// [`Tet10Mesh::with_sdf_projected_boundary`](crate::Tet10Mesh::with_sdf_projected_boundary),
    /// with two differences suited to conforming a bonded surface onto a real body:
    ///
    /// - the caller supplies explicit targets, so the projection can honour a
    ///   direction / anatomy gate the mesh layer knows nothing about (which of two
    ///   candidate surfaces, an SI-alignment test) rather than projecting onto one
    ///   [`Sdf`](crate::Sdf);
    /// - the validity bar is a *quality floor* `detJ ≥ quality_floor · detJ_rest`,
    ///   not merely `detJ > 0`. A bare `detJ > 0` bisects each backed-off node onto
    ///   the `detJ → 0⁺` degeneracy boundary — manufacturing slivers exactly where
    ///   the move was largest; holding `detJ` above a fraction of its healthy rest
    ///   value keeps the backed-off elements well-shaped.
    ///
    /// For each move the full target is tried first; if it would drop an incident
    /// element below the floor, the node is bisected back along the segment
    /// `original → target` to the furthest point that keeps every incident element
    /// above the floor. The original position (`t = 0`) is always feasible
    /// (`detJ_rest ≥ quality_floor · detJ_rest` for `quality_floor < 1`), so a node
    /// in the worst case simply stays put. Nodes are swept in ascending `VertexId`
    /// order for a deterministic result.
    ///
    /// The reference `detJ_rest` per element is captured once, from `self`, before
    /// any node moves — so projecting several nodes of the same element still
    /// measures each against that element's original, healthy volume.
    ///
    /// Vertices not named in `moves` are untouched; the topology (tets, boundary
    /// faces, materials, interface flags) is unchanged, and per-tet
    /// [`QualityMetrics`] are recomputed from the new positions. Like
    /// [`Self::largest_component`], the per-tet material and interface caches are
    /// kept as-is: a conforming move is sub-element in scale, so a tet's centroid
    /// stays in the same material region (the disc's field is uniform regardless).
    ///
    /// # Panics
    ///
    /// Panics if `quality_floor` is not in `[0, 1)` — a floor `≥ 1` would reject the
    /// original mesh and a negative floor is meaningless — or if a move names a
    /// vertex out of range.
    #[must_use]
    pub fn with_projected_nodes(mut self, moves: &[(VertexId, Vec3)], quality_floor: f64) -> Self {
        /// Bisection steps for the back-off (~`2⁻⁴⁰` blend resolution).
        const BISECT_ITERS: usize = 40;

        assert!(
            (0.0..1.0).contains(&quality_floor),
            "quality_floor must be in [0, 1); got {quality_floor}"
        );

        if moves.is_empty() {
            return self;
        }

        // Take positions and tets out so the sweep can mutate positions in place
        // while reading the (fixed) connectivity — disjoint borrows, put back after.
        let tets = std::mem::take(&mut self.tets);
        let mut positions = std::mem::take(&mut self.vertices);

        let to_project: HashSet<VertexId> = moves.iter().map(|&(v, _)| v).collect();
        // Incident tets per moved node: validity is an element property, so a moved
        // node must keep every tet that references it above the floor.
        let mut incident: HashMap<VertexId, Vec<usize>> = HashMap::new();
        for (ti, t) in tets.iter().enumerate() {
            for &v in t {
                if to_project.contains(&v) {
                    incident.entry(v).or_default().push(ti);
                }
            }
        }

        // Tet4's affine map gives one rest-Jacobian determinant per element.
        let element = Tet4;
        let det_of = |positions: &[Vec3], ti: usize| -> f64 {
            let t = tets[ti];
            let x_ref = SMatrix::<f64, 4, 3>::from_fn(|a, k| positions[t[a] as usize][k]);
            element.rest_jacobian_dets(&x_ref)[0]
        };
        // Reference determinant per incident tet, captured from the ORIGINAL mesh
        // before any node moves — the healthy volume each back-off is measured
        // against.
        let mut orig_det: HashMap<usize, f64> = HashMap::new();
        for tis in incident.values() {
            for &ti in tis {
                orig_det.entry(ti).or_insert_with(|| det_of(&positions, ti));
            }
        }

        // A move keeps element `ti` valid iff its current determinant is finite and
        // at least `quality_floor` of its original. Finiteness is asserted first: a
        // NaN determinant must read as invalid, never slip through a bare `≥`.
        let incident_ok = |positions: &[Vec3], tis: &[usize]| -> bool {
            tis.iter().all(|&ti| {
                let d = det_of(positions, ti);
                d.is_finite() && d >= quality_floor * orig_det[&ti]
            })
        };

        // Deterministic sweep in ascending VertexId order.
        let mut sorted = moves.to_vec();
        sorted.sort_by_key(|&(v, _)| v);
        for (v, target) in sorted {
            let Some(tis) = incident.get(&v) else {
                continue; // an orphan vertex, referenced by no tet — nothing to keep valid
            };
            let vi = v as usize;
            let straight = positions[vi];
            let delta = target - straight;
            // Fast path: the full target keeps every incident element above the floor.
            positions[vi] = target;
            if incident_ok(&positions, tis) {
                continue;
            }
            // Back off: largest blend `t ∈ [0, 1]` with `straight + t·delta` valid.
            // `t = 0` (straight) is always feasible, so `lo` stays a valid lower bound.
            let mut lo = 0.0_f64;
            let mut hi = 1.0_f64;
            for _ in 0..BISECT_ITERS {
                let t = 0.5 * (lo + hi);
                positions[vi] = straight + delta * t;
                if incident_ok(&positions, tis) {
                    lo = t;
                } else {
                    hi = t;
                }
            }
            positions[vi] = straight + delta * lo;
        }

        let q = quality::compute_metrics(&positions, &tets);
        self.tets = tets;
        self.vertices = positions;
        self.q = q;
        self
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

#[cfg(test)]
mod projected_nodes_tests {
    #![allow(clippy::expect_used, clippy::cast_possible_truncation)]

    use super::{MeshingHints, SdfMeshedTetMesh};
    use crate::Vec3;
    use crate::material::MaterialField;
    use crate::mesh::{Mesh, VertexId};
    use crate::sdf_bridge::{Aabb3, SphereSdf};

    fn hints(min: Vec3, max: Vec3) -> MeshingHints {
        MeshingHints {
            bbox: Aabb3::new(min, max),
            cell_size: 0.03,
            material_field: Some(MaterialField::uniform(1.0e5, 4.0e5)),
        }
    }

    /// Signed rest-Jacobian determinant of a tet — the SAME quantity the back-off
    /// gates on (`Tet4::rest_jacobian_dets`), written independently here as the
    /// triple product of the edge vectors so the test cross-checks rather than
    /// mirrors production.
    fn tet_det(positions: &[Vec3], tet: [VertexId; 4]) -> f64 {
        let p = |i: usize| positions[tet[i] as usize];
        let (e1, e2, e3) = (p(1) - p(0), p(2) - p(0), p(3) - p(0));
        e1.cross(&e2).dot(&e3)
    }

    /// The worst per-element determinant ratio `det / det_orig` over every tet —
    /// the quantity the floor bounds. `< quality_floor` anywhere means an element
    /// dropped below its healthy fraction (a sliver or an inversion).
    fn worst_ratio(orig: &SdfMeshedTetMesh, moved: &SdfMeshedTetMesh) -> f64 {
        (0..orig.n_tets())
            .map(|ti| {
                let tet = orig.tet_vertices(ti as u32);
                let d0 = tet_det(orig.positions(), tet);
                let d1 = tet_det(moved.positions(), tet);
                d1 / d0
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn sphere_mesh() -> SdfMeshedTetMesh {
        SdfMeshedTetMesh::from_sdf(
            &SphereSdf { radius: 0.1 },
            &hints(Vec3::new(-0.13, -0.13, -0.13), Vec3::new(0.13, 0.13, 0.13)),
        )
        .expect("sphere scene meshes")
    }

    #[test]
    fn quality_floor_back_off_prevents_slivers_and_inversions() {
        const FLOOR: f64 = 0.05;
        let mesh = sphere_mesh();
        // A boundary vertex (radius ≈ 0.1) dragged to the sphere centre — a ~0.1 m
        // inward move that would collapse and invert its incident tets if applied
        // in full.
        let v0 = mesh.boundary_faces()[0][0];
        let center = Vec3::zeros();

        let moved = mesh.clone().with_projected_nodes(&[(v0, center)], FLOOR);

        // No element fell below the floor (nor inverted — that would be a NEGATIVE
        // ratio, also caught).
        let worst = worst_ratio(&mesh, &moved);
        assert!(
            worst >= FLOOR - 1e-9,
            "an element dropped below the {FLOOR} quality floor (worst ratio {worst:.4})"
        );

        // The back-off actually engaged: the node neither reached the target nor
        // stayed put, and it lies on the straight `original → target` segment.
        let p0 = mesh.positions()[v0 as usize];
        let pf = moved.positions()[v0 as usize];
        assert!(
            (pf - center).norm() > 1e-6,
            "node reached the target — the move should have been backed off"
        );
        assert!((pf - p0).norm() > 1e-6, "node did not move at all");
        let seg = (center - p0).normalize();
        assert!(
            (pf - p0).cross(&seg).norm() < 1e-9,
            "backed-off node left the original→target segment"
        );
    }

    #[test]
    fn a_gentle_move_is_applied_in_full() {
        let mesh = sphere_mesh();
        let v0 = mesh.boundary_faces()[0][0];
        let p0 = mesh.positions()[v0 as usize];
        // A tiny outward nudge (0.1 mm) keeps every incident tet well above the
        // floor, so the full target is taken exactly — no back-off.
        let target = p0 + p0.normalize() * 1.0e-4;
        let moved = mesh.with_projected_nodes(&[(v0, target)], 0.05);
        let pf = moved.positions()[v0 as usize];
        assert_eq!(pf, target, "a valid move must be applied exactly");
    }

    #[test]
    fn the_quality_floor_is_the_operative_bar() {
        // Mutation guard: relaxing the floor to 0 (bare `detJ > 0`) lets the same
        // node travel strictly further toward the target than the 0.05 floor does.
        // If the floor were inert the two placements would coincide.
        let mesh = sphere_mesh();
        let v0 = mesh.boundary_faces()[0][0];
        let center = Vec3::zeros();

        let floored = mesh.clone().with_projected_nodes(&[(v0, center)], 0.05);
        let unfloored = mesh.with_projected_nodes(&[(v0, center)], 0.0);

        let d_floored = (floored.positions()[v0 as usize] - center).norm();
        let d_unfloored = (unfloored.positions()[v0 as usize] - center).norm();
        assert!(
            d_unfloored < d_floored - 1e-6,
            "floor 0 should push the node further in (dist {d_unfloored:.4}) than floor 0.05 (dist {d_floored:.4})"
        );
    }

    #[test]
    fn projection_is_deterministic() {
        let mesh = sphere_mesh();
        let v0 = mesh.boundary_faces()[0][0];
        let center = Vec3::zeros();
        let a = mesh.clone().with_projected_nodes(&[(v0, center)], 0.05);
        let b = mesh.with_projected_nodes(&[(v0, center)], 0.05);
        for (pa, pb) in a.positions().iter().zip(b.positions()) {
            assert_eq!(
                pa.as_slice(),
                pb.as_slice(),
                "projection must be deterministic"
            );
        }
    }

    #[test]
    fn multi_node_back_off_keeps_shared_tets_valid_and_is_order_independent() {
        const FLOOR: f64 = 0.05;
        // Two vertices of the SAME boundary face share an incident tet, so dragging BOTH to the
        // centre exercises the interacting greedy-sweep path a single-node move never reaches (a
        // tet with two moving vertices, each back-off re-validating against the other's placement).
        let mesh = sphere_mesh();
        let face = mesh.boundary_faces()[0];
        let (v0, v1) = (face[0], face[1]);
        let center = Vec3::zeros();

        let moved = mesh
            .clone()
            .with_projected_nodes(&[(v0, center), (v1, center)], FLOOR);
        let worst = worst_ratio(&mesh, &moved);
        assert!(
            worst >= FLOOR - 1e-9,
            "a tet with two moved vertices dropped below the {FLOOR} floor (worst {worst:.4})"
        );

        // Reversing the caller's move order yields a byte-identical mesh: the sweep sorts by
        // ascending `VertexId` internally, so it is order-independent. (A determinism/property
        // check — in this symmetric two-node config the interacting back-offs happen to resolve
        // identically regardless of sweep order, so it does not by itself pin the sort; the
        // single-node gates carry the floor and fast-path teeth.)
        let reversed = mesh.with_projected_nodes(&[(v1, center), (v0, center)], FLOOR);
        for (a, b) in moved.positions().iter().zip(reversed.positions()) {
            assert_eq!(
                a.as_slice(),
                b.as_slice(),
                "the sweep must be order-independent (sorted by VertexId)"
            );
        }
    }

    #[test]
    fn empty_moves_is_a_noop() {
        let mesh = sphere_mesh();
        let before: Vec<Vec3> = mesh.positions().to_vec();
        let after = mesh.with_projected_nodes(&[], 0.05);
        assert_eq!(after.positions(), before.as_slice());
    }
}
