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
//!    [`mesh::quality::compute_metrics`].
//!
//! Memo cite: scope memo §2 file plan + §3 Decision A (algorithm),
//! Decision H (constructor signature + error variants — no
//! `NegativeVolumeTet` variant; orientation is by-construction
//! right-handed, the D-10 backstop silently drops sub-volume-floor
//! sub-tets), Decision I (`QualityMetrics` four `Vec<f64>`), Decision
//! J (`MeshAdjacency` unit struct), Decision M (D-8/D-9/D-10/D-11).

use std::collections::BTreeMap;

use crate::Vec3;
use crate::material::{MaterialField, NeoHookean};
use crate::mesh::{
    Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, interface_flags_from_field,
    materials_from_field, quality,
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
#[derive(Clone, Debug)]
pub struct SdfMeshedTetMesh {
    vertices: Vec<Vec3>,
    tets: Vec<[VertexId; 4]>,
    adj: MeshAdjacency,
    q: QualityMetrics,
    material_cache: Vec<NeoHookean>,
    interface_flags: Vec<bool>,
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

impl SdfMeshedTetMesh {
    /// Build the mesh by running the BCC + Labelle-Shewchuk
    /// Isosurface Stuffing pipeline (see module doc).
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
    pub fn from_sdf(sdf: &dyn Sdf, hints: &MeshingHints) -> Result<Self, MeshingError> {
        // Per scope memo §0 + §8 commit 9: hints carry the canonical
        // `(SdfField, MaterialField)` pair (book Part 7 §00). When
        // `hints.material_field` is `None`, fall back to the IV-1
        // baseline (Ecoflex 00-30 compressible regime, μ=1e5 / λ=4e5)
        // — a single named constant via `MaterialField::skeleton_default`
        // so consumers grep cleanly. Building the fallback unconditionally
        // is two `Box<ConstantField<f64>>` allocations (microseconds);
        // `from_sdf` is a one-shot per scene construction.
        let fallback = MaterialField::skeleton_default();
        let material_field = hints.material_field.as_ref().unwrap_or(&fallback);

        let lattice = BccLattice::new(hints);
        let n_lattice = lattice.positions.len();

        // Step 2 + 3: sample SDF in sequential VertexId order, detect
        // non-finite, then negate per Decision A SDF sign convention
        // adapter (sim-soft negative-inside → paper positive-inside).
        // The sequential walk pins the failing-vertex diagnostic — the
        // first non-finite value is the smallest `VertexId` that trips.
        let mut sdf_values: Vec<f64> = Vec::with_capacity(n_lattice);
        for (vid, position) in lattice.positions.iter().enumerate() {
            let raw = sdf.eval(*position);
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
        let material_cache = materials_from_field(&output_positions, &output_tets, material_field);

        // Step 9: per-tet interface-flag cache. `|φ(x_c)| < L_e`
        // rule per Part 7 §02 §01 + scope memo Decision K (commit 12,
        // IV-6); diagnostic-only — Newton hot path does not branch.
        // All-`false` payload of length `n_tets` when
        // `material_field` carries no interface SDF (uniform /
        // `LayeredScalarField`-only fields go through this path).
        let interface_flags =
            interface_flags_from_field(&output_positions, &output_tets, material_field);

        Ok(Self {
            vertices: output_positions,
            tets: output_tets,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
        })
    }
}

impl Mesh for SdfMeshedTetMesh {
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

    fn materials(&self) -> &[NeoHookean] {
        &self.material_cache
    }

    fn interface_flags(&self) -> &[bool] {
        &self.interface_flags
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
    fn equals_structurally(&self, other: &dyn Mesh) -> bool {
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
