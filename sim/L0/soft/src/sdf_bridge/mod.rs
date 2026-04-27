//! `sdf_bridge` — SDF→tet meshing bridge (Phase 3, C1 chunk 1b).
//!
//! Phase 3 ships exactly two of the six sub-areas named in the book's
//! `08-sdf-bridge.md`: SDF ingest (via the [`Sdf`] trait + [`MeshingHints`])
//! and tetrahedralization (BCC + Labelle-Shewchuk Isosurface Stuffing
//! placeholder mesher, spanning commits 4–6 — BCC lattice, warp +
//! stencil dispatch, and `SdfMeshedTetMesh` constructor respectively).
//! The other four — material sampling, change classifier, warm-start
//! state-transfer, FD-wrapper trigger — are Phase 4 / Phase G work. See
//! `sim/docs/todo/phase_3_sdf_tet_bridge_scope.md` §0 + BF-10.
//!
//! Crate-root re-exports of `Sdf`, `SphereSdf`, `Aabb3`, `MeshingHints`,
//! `SdfMeshedTetMesh`, and `MeshingError` are present at the public
//! surface block of `lib.rs`; this module re-exports them here as well
//! so external code can reach the SDF→tet API via either
//! `sim_soft::*` (idiomatic crate-root path) or
//! `sim_soft::sdf_bridge::*` (module path, kept for backward
//! compatibility with the Phase 3 in-progress test files).

mod lattice;
mod sdf;
mod sdf_meshed_tet_mesh;
mod stuffing;

pub use sdf::{Sdf, SphereSdf};
pub use sdf_meshed_tet_mesh::{MeshingError, SdfMeshedTetMesh};

use crate::Vec3;

/// Axis-aligned bounding box in world space.
///
/// Carried by [`MeshingHints`] to define the lattice extent that the
/// placeholder mesher walks. Field-access only in Phase 3; future
/// fTetWild may grow operations on this type.
#[derive(Clone, Debug)]
pub struct Aabb3 {
    /// Minimum corner of the box.
    pub min: Vec3,
    /// Maximum corner of the box.
    pub max: Vec3,
}

impl Aabb3 {
    /// Construct an [`Aabb3`] from explicit `min` and `max` corners.
    ///
    /// Caller invariant: `min[i] <= max[i]` componentwise. Phase 3 does
    /// not validate; degenerate boxes surface as `MeshingError::EmptyMesh`
    /// at meshing time (Decision H).
    #[must_use]
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }
}

/// Knobs handed to the placeholder mesher's `from_sdf` constructor.
///
/// Phase 3 ships `bbox` + `cell_size` only; `resolution_hint` and
/// `envelope_tolerance` (book Ch 00 §04) land when fTetWild does.
#[derive(Clone, Debug)]
pub struct MeshingHints {
    /// Lattice extent — the mesher samples the SDF on a uniform cubic
    /// lattice spanning this box.
    pub bbox: Aabb3,
    /// Lattice spacing in world units (metres). Smaller is finer.
    pub cell_size: f64,
}
