// `dead_code` is allowed at module scope because this module's public
// surface (`InsertionRamp`, `StepReadout`, `TetReadout`,
// `InsertionResult`, etc.) is consumed selectively by three different
// callers — the integration tests in this file, the slice-7.4
// `insertion_sim_ui` panel, and (pending) slice-7.5/7.6 + slice-8/9
// downstream wiring — so the binary's `main.rs` reads only a subset
// at any given slice. The "never read" lint is rightly conservative
// for a `pub(crate)` module in a binary crate; documenting it as a
// known-and-deliberate carve-out here is cheaper than chasing per-
// field `#[allow]`s as each slice lands.
#![allow(dead_code)]

//! `insertion_sim` — FEM insertion-simulation pipeline for cf-device-design.
//!
//! Slice 7 (sub-commit 7.0) seeds this module with the **SDF bridge
//! spike**: the Route-A geometry path that turns the cleaned scan into
//! a tet mesh the sim-soft FEM solver can consume.
//!
//! Route A (settled with the user before 7.0): keep the decimated mesh
//! proxy for the live viewport, but at simulate-time re-derive geometry
//! from a `mesh_sdf` SDF of the *original cleaned scan* — an
//! `outer.subtract(cavity)` device-wall body offset from that SDF —
//! mirroring the validated sim-soft rows 21–25 layered-sleeve path.
//!
//! - **7.0** seeded the module with the SDF-bridge *spike* —
//!   `run_sdf_bridge_spike` is a measurement harness that proved
//!   Route A end-to-end and characterized the decimation/timing
//!   tradeoff. `SignedDistanceField::distance` is brute-force
//!   O(faces) and the mesher samples the SDF at every BCC lattice
//!   vertex, so the raw 3.34 M-face scan must be decimated — the
//!   spike found a low target (~1.5–3k faces) is best.
//! - **7.1** adds `build_insertion_geometry` — the real builder
//!   that turns a `SimDesign` (cavity inset + layer stack) into the
//!   device-wall `SdfMeshedTetMesh` with per-tet Yeoh materials,
//!   plus the rigid intruder SDF.
//! - **7.2** adds `run_single_insertion_step` — one static FEM solve
//!   (`CpuNewtonSolver` + `PenaltyRigidContact`) that presses the
//!   intruder a chosen interference into the cavity and returns the
//!   converged deformed positions. One step only — the quasi-static
//!   ramp is 7.3.
//! - **7.3b.1 / 7.3b.2** add `run_insertion_ramp` + `InsertionResult`
//!   — quasi-static interference ramp with per-step + per-tet
//!   engineering readouts.
//! - **7.4** un-gates this module (`pub(crate)`) for the Insertion-Sim
//!   panel in `main.rs`; see `crate::insertion_sim_ui` for the egui
//!   surface, async-compute task wiring, and heat-map projection
//!   that surfaces the `InsertionResult` per-tet field onto the
//!   per-layer shell meshes via Option-C per-vertex coloring.
//!
//! Note on doc-link convention: this module uses plain-backtick code
//! spans, not `[`name`]` intra-doc links, for cross-references inside
//! the module-level docstring. Binary-crate rustdoc does not always
//! resolve item-path links from a `pub(crate)` module's `//!`
//! comments; switching to code-spans keeps the docs warning-free
//! across the 7.0→7.4 ladder without papering over a real ambiguity
//! with `#[allow]`.

use std::collections::BTreeSet;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;
use std::time::Instant;

use anyhow::{Context, Result, anyhow};
use cf_cap_planes::{CapPlane, dome_wall_only_mesh};
use cf_design::{Aabb, SdfGrid, Solid, pinned_floor_shell};
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::{CachedGridSdf, PseudoNormalSign, Signed, TriMeshDistance};
use mesh_types::IndexedMesh;
use meshopt::simplify_sloppy_decoder;
use nalgebra::{Isometry3, Matrix3, Point3, Vector3};
use sim_ml_chassis::Tensor;
#[cfg(test)]
use sim_soft::ContactPair;
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10,
    ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50,
};
use sim_soft::{
    Aabb3, BoundaryConditions, ConstantField, ContactPairReadout, CpuNewtonSolver, Field,
    LayeredScalarField, LmConfig, Material, MaterialField, Mesh, MeshingHints, PenaltyRigidContact,
    Sdf, SdfMeshedTetMesh, ShoreReading, SiliconeMaterial, Solver, SolverConfig, SolverFailure,
    Tet4, TetId, Vec3, VertexId, Yeoh, filter_pair_readouts_to_referenced,
    pick_vertices_by_predicate, referenced_vertices,
};

/// Weld epsilon (meters) for the pre-decimation vertex weld — matches
/// `main.rs`'s `ENVELOPE_PROXY_WELD_EPSILON_M`. The cleaned scan's STL
/// load produces 3-per-triangle unshared vertices that meshopt needs
/// welded to find collapsible edges.
const SPIKE_WELD_EPSILON_M: f64 = 1e-6;

/// `simplify_sloppy_decoder`'s target-error cap — effectively
/// unbounded, same rationale as `main.rs`'s
/// `ENVELOPE_PROXY_SIMPLIFY_TARGET_ERROR`: the face-count target is
/// the binding constraint, this cap is only a defensive upper bound.
const SPIKE_SIMPLIFY_TARGET_ERROR: f32 = 10.0;

/// Decimate the cleaned scan to roughly `target_faces` triangles for
/// SDF construction.
///
/// Separate from `main.rs`'s `compute_envelope_proxy_mesh` (which
/// decimates hard — ~1500 faces — for *viewport* speed): here the
/// face count trades `SignedDistanceField`'s brute-force O(faces)
/// query cost — paid once per BCC lattice vertex — against
/// isosurface-landing fidelity. [`run_sdf_bridge_spike`] sweeps
/// `target_faces` so 7.1 can pick that tradeoff point from measured
/// data; the 7.0 spike found tet count + element quality are governed
/// by the BCC `cell_size`, *not* the SDF face count, so a low
/// resolution is preferred (see the slice-7 ship log).
///
/// Pipeline mirrors the proxy builder: weld unshared STL vertices,
/// `simplify_sloppy_decoder` (topology-non-preserving — required for
/// the iter-1 scan's disconnected components + degenerate triangles
/// that block topology-preserving collapse), strip unreferenced
/// vertices. Returns the scan unchanged (modulo the vertex weld) when
/// it is already at or below `target_faces`.
pub(crate) fn decimate_for_sdf(scan: &IndexedMesh, target_faces: usize) -> IndexedMesh {
    let mut welded = scan.clone();
    weld_vertices(&mut welded, SPIKE_WELD_EPSILON_M);

    // f64 → f32 cast is intentional: meshopt's C API operates on f32.
    #[allow(clippy::cast_possible_truncation)]
    let positions: Vec<[f32; 3]> = welded
        .vertices
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let indices: Vec<u32> = welded.faces.iter().flatten().copied().collect();
    let target_index_count = target_faces.saturating_mul(3);

    let simplified = if target_index_count < indices.len() {
        let mut result_error = 0.0_f32;
        simplify_sloppy_decoder(
            &indices,
            &positions,
            target_index_count,
            SPIKE_SIMPLIFY_TARGET_ERROR,
            Some(&mut result_error),
        )
    } else {
        // Already at or below target — skip decimation.
        indices
    };

    let mut decimated = welded;
    decimated.faces = simplified
        .chunks_exact(3)
        .map(|tri| [tri[0], tri[1], tri[2]])
        .collect();
    remove_unreferenced_vertices(&mut decimated);
    decimated
}

/// Axis-aligned bounding box of `scan`'s vertices, expanded by
/// `margin_m` on every side.
///
/// Used both as the [`Solid::from_sdf`] interval-pruning bound and —
/// via [`aabb3_for_meshing`] — as the BCC lattice extent. The margin
/// must cover the outer envelope: the body geometry is
/// `scan.offset(t)`, so the lattice has to reach `t` beyond the raw
/// scan or the offset surface is clipped.
fn scan_aabb(scan: &IndexedMesh, margin_m: f64) -> Aabb {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &scan.vertices {
        min = Point3::new(min.x.min(v.x), min.y.min(v.y), min.z.min(v.z));
        max = Point3::new(max.x.max(v.x), max.y.max(v.y), max.z.max(v.z));
    }
    let m = Vector3::new(margin_m, margin_m, margin_m);
    Aabb::new(min - m, max + m)
}

/// Convert a cf-geometry [`Aabb`] into sim-soft's [`Aabb3`] (the BCC
/// lattice extent carried by [`MeshingHints`]). cf-geometry stores
/// corners as `Point3`; sim-soft as `Vec3` — this bridges the two.
fn aabb3_for_meshing(bounds: &Aabb) -> Aabb3 {
    Aabb3::new(bounds.min.coords, bounds.max.coords)
}

/// Measurements from one [`run_sdf_bridge_spike`] invocation — the
/// spike's payload. Timing and quality are *reported*, not asserted:
/// they are the data 7.1 picks an SDF-source resolution from.
#[derive(Debug, Clone)]
pub struct SpikeReport {
    /// Decimation target handed to [`decimate_for_sdf`].
    pub target_faces: usize,
    /// Actual face count of the decimated SDF-source mesh.
    pub decimated_faces: usize,
    /// BCC lattice spacing (meters) used for tet meshing.
    pub cell_size_m: f64,
    /// Wall thickness (meters) of the `scan.offset(t)` outer envelope.
    pub wall_thickness_m: f64,
    /// Wall-clock time to decimate the scan.
    pub decimate_ms: f64,
    /// Wall-clock time to build the
    /// [`Signed<TriMeshDistance, PseudoNormalSign>`](Signed).
    pub sdf_build_ms: f64,
    /// Wall-clock time for [`SdfMeshedTetMesh::from_sdf`] — the
    /// dominant cost. One brute-force SDF query per BCC lattice
    /// vertex, ×2 for the `outer.subtract(scan)` composition.
    pub mesh_build_ms: f64,
    /// Tet count of the resulting mesh.
    pub n_tets: usize,
    /// Vertex count (includes unreferenced BCC orphans by design —
    /// see `SdfMeshedTetMesh` docs).
    pub n_vertices: usize,
    /// Smallest per-tet aspect ratio (inscribed/circumscribed sphere
    /// ratio; 1/3 is the regular-tet max, 0 a degenerate sliver).
    pub min_aspect_ratio: f64,
    /// Mean per-tet aspect ratio.
    pub mean_aspect_ratio: f64,
    /// Count of tets with non-positive signed volume — must be 0 for
    /// a solver-usable mesh.
    pub inverted_tets: usize,
}

impl std::fmt::Display for SpikeReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "SDF bridge spike — target {} faces (cell {:.1} mm, wall {:.1} mm)",
            self.target_faces,
            self.cell_size_m * 1e3,
            self.wall_thickness_m * 1e3,
        )?;
        writeln!(
            f,
            "  decimated: {} faces  |  decimate {:.0} ms  sdf-build {:.0} ms  \
             mesh-build {:.0} ms",
            self.decimated_faces, self.decimate_ms, self.sdf_build_ms, self.mesh_build_ms,
        )?;
        write!(
            f,
            "  tets: {} ({} vertices, {} inverted)  |  aspect-ratio min {:.4} mean {:.4}",
            self.n_tets,
            self.n_vertices,
            self.inverted_tets,
            self.min_aspect_ratio,
            self.mean_aspect_ratio,
        )
    }
}

/// Run the Route-A SDF bridge end-to-end on `scan` and report timing +
/// tet-mesh quality.
///
/// Pipeline: [`decimate_for_sdf`] →
/// [`Signed<TriMeshDistance, PseudoNormalSign>`](Signed) →
/// [`Solid::from_sdf`] → `outer.subtract(scan)` body (`outer =
/// scan.offset(wall_thickness_m)`) → [`SdfMeshedTetMesh::from_sdf`] at
/// `cell_size_m`. Geometry only — no materials (skeleton-default
/// Neo-Hookean), no intruder, no solve.
///
/// # Errors
///
/// Propagates [`TriMeshDistance::new`] (empty mesh) and
/// [`SdfMeshedTetMesh::from_sdf`] (empty mesh, non-finite SDF value)
/// failures with context.
pub fn run_sdf_bridge_spike(
    scan: &IndexedMesh,
    target_faces: usize,
    cell_size_m: f64,
    wall_thickness_m: f64,
) -> Result<SpikeReport> {
    // Bounds are taken from the *original* scan: decimation only drops
    // faces/vertices, never extends the envelope, so the original
    // bbox safely contains the decimated SDF source. The margin
    // covers the `scan.offset(t)` outer envelope plus one cell of
    // slack so the BCC lattice fully contains the outer isosurface.
    let bounds = scan_aabb(scan, wall_thickness_m + cell_size_m);

    let t = Instant::now();
    let decimated = decimate_for_sdf(scan, target_faces);
    let decimate_ms = elapsed_ms(t);
    let decimated_faces = decimated.faces.len();

    let t = Instant::now();
    let sdf_distance =
        TriMeshDistance::new(decimated).context("build TriMeshDistance from the decimated scan")?;
    let sdf_sign = PseudoNormalSign::from_distance(&sdf_distance);
    let sdf = Signed {
        distance: sdf_distance,
        sign: sdf_sign,
    };
    let sdf_build_ms = elapsed_ms(t);

    // Route-A geometry: body = outer.subtract(scan), mirroring the
    // sim-soft rows 21–25 layered-sleeve precedent. `SignedDistanceField`
    // is `Clone` and the decimated mesh is small, so cloning it for the
    // two CSG operands is cheap.
    let t = Instant::now();
    let outer = Solid::from_sdf(sdf.clone(), bounds).offset(wall_thickness_m);
    let cavity = Solid::from_sdf(sdf, bounds);
    let body = outer.subtract(cavity);
    let hints = MeshingHints {
        bbox: aabb3_for_meshing(&bounds),
        cell_size: cell_size_m,
        material_field: None,
    };
    // `MeshingError` does not implement `std::error::Error`, so it
    // can't ride `anyhow::Context` — wrap it by hand via its `Debug`.
    let mesh = SdfMeshedTetMesh::from_sdf(&body, &hints).map_err(|e| {
        anyhow!("tet-mesh the Route-A device-wall body via SdfMeshedTetMesh::from_sdf: {e:?}")
    })?;
    let mesh_build_ms = elapsed_ms(t);

    let q = mesh.quality();
    let n_tets = mesh.n_tets();
    let min_aspect_ratio = q.aspect_ratio.iter().copied().fold(f64::MAX, f64::min);
    let mean_aspect_ratio = if n_tets == 0 {
        0.0
    } else {
        // Tet counts are nowhere near f64's 2^53 exact-integer ceiling.
        #[allow(clippy::cast_precision_loss)]
        let denom = n_tets as f64;
        q.aspect_ratio.iter().sum::<f64>() / denom
    };
    let inverted_tets = q.signed_volume.iter().filter(|&&v| v <= 0.0).count();

    Ok(SpikeReport {
        target_faces,
        decimated_faces,
        cell_size_m,
        wall_thickness_m,
        decimate_ms,
        sdf_build_ms,
        mesh_build_ms,
        n_tets,
        n_vertices: mesh.n_vertices(),
        min_aspect_ratio,
        mean_aspect_ratio,
        inverted_tets,
    })
}

/// Milliseconds elapsed since `start`.
fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1e3
}

// ── 7.1 — insertion geometry + per-layer Yeoh material ──────────────

/// One concentric layer of the device wall, projected for the
/// insertion sim: a radial thickness + a base-silicone anchor key +
/// the per-layer Slacker mass fraction.
///
/// The decimated form of `main.rs`'s `LayerSpec` (drops `visible` —
/// a viewport concern). `slacker_fraction` is plumbed straight
/// through: slice 7.5 added the wiring from `LayerSpec.slacker_fraction`
/// to the sim's per-tet Yeoh material via `effective_silicone_for_layer`
/// — see the `Slacker → sim-modulus` section at module top for the
/// resolution scheme + the Shore-000 floor.
#[derive(Debug, Clone)]
pub struct SimLayer {
    /// Radial thickness (meters). Innermost-first ordering, same as
    /// `LayerSpec`.
    pub thickness_m: f64,
    /// Smooth-On base-silicone anchor key — one of the eight in
    /// `main.rs`'s `LAYER_MATERIALS` catalog.
    pub anchor_key: String,
    /// Slacker mass as a fraction of the base silicone's Part A+B
    /// mass — `0.0` for the base alone, matches the slice 6.5
    /// `LayerSpec.slacker_fraction` semantics. The TB-tabulated
    /// fractions are `{0.0, 0.25, 0.50, 0.75, 1.00}` per the
    /// Smooth-On Slacker Technical Bulletin (rev 011524DH);
    /// `crate::slacker::support(anchor_key)` returns the curve.
    /// Off-curve values are snapped by `crate::resolve_slacker_fraction`
    /// before reaching the sim.
    pub slacker_fraction: f64,
}

/// Device-design parameters the insertion sim consumes: the cavity
/// inset + the ordered (innermost-first) layer stack. The decimated
/// projection of `main.rs`'s `CavityState` + `LayersState`.
#[derive(Debug, Clone)]
pub struct SimDesign {
    /// Distance (meters) the cavity surface sits *inside* the scan
    /// surface — `CavityState::inset_m`. The cavity surface is at
    /// scan-SDF offset `-cavity_inset_m`.
    pub cavity_inset_m: f64,
    /// Innermost-first layer stack. `layers[0]`'s inner surface is
    /// the cavity surface; `layers[i]`'s outer surface is
    /// `layers[i + 1]`'s inner surface.
    pub layers: Vec<SimLayer>,
}

/// Resolve a `main.rs` `LAYER_MATERIALS` anchor key to the sim-soft
/// [`SiliconeMaterial`] anchor it mirrors.
///
/// The eight keys are a closed catalog — an unrecognized key is a
/// wiring bug, surfaced as an error rather than silently substituted.
/// (`main.rs`'s `material_density` *does* fall back defensively, but
/// a wrong *modulus* would quietly corrupt the sim, not just a mass
/// readout — so the sim path fails loud instead.)
fn silicone_for_anchor(anchor_key: &str) -> Result<SiliconeMaterial> {
    match anchor_key {
        "ECOFLEX_00_10" => Ok(ECOFLEX_00_10),
        "ECOFLEX_00_20" => Ok(ECOFLEX_00_20),
        "ECOFLEX_00_30" => Ok(ECOFLEX_00_30),
        "ECOFLEX_00_50" => Ok(ECOFLEX_00_50),
        "DRAGON_SKIN_10A" => Ok(DRAGON_SKIN_10A),
        "DRAGON_SKIN_15" => Ok(DRAGON_SKIN_15),
        "DRAGON_SKIN_20A" => Ok(DRAGON_SKIN_20A),
        "DRAGON_SKIN_30A" => Ok(DRAGON_SKIN_30A),
        other => Err(anyhow!(
            "unrecognized silicone anchor key {other:?} — not in the cf-device-design catalog"
        )),
    }
}

/// How `effective_silicone_for_layer` resolved a layer — surfaces
/// alongside the returned `SiliconeMaterial` so the panel can flag
/// fallbacks to the user.
///
/// Slice 7.5: Slacker pushes silicones across Shore scales (A → 00 →
/// 000). sim-soft's anchor table covers Shore 00 + Shore A; Shore 000
/// (the gel scale, where most Slacker outcomes land) has no published
/// Yeoh anchors. The variants below capture every path the resolver
/// can take.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlackerResolution {
    /// `slacker_fraction == 0.0` (or `Support::NotRecommended` /
    /// `Support::NoData`) — material is the base anchor unchanged.
    Base,
    /// Slacker fraction snapped to a TB curve point whose effective
    /// hardness lands in Shore A or Shore 00, both of which sim-soft
    /// anchors. The material was built via
    /// `SiliconeMaterial::from_effective_shore`.
    Interpolated,
    /// Slacker fraction snapped to a TB point whose effective hardness
    /// is Shore 000 (OOO) — softer than sim-soft's softest published
    /// anchor (`ECOFLEX_00_10`). The resolver returns the
    /// `ECOFLEX_00_10` material as a conservative floor (slight
    /// over-stiffness vs ground truth). A proper Shore-000 calibration
    /// is a future slice.
    FlooredAtSoftestAnchor,
}

/// Resolve a layer's effective `SiliconeMaterial` from
/// `(anchor_key, slacker_fraction)` — slice 7.5's Slacker → sim-modulus
/// wiring.
///
/// **Resolution table** (driven by `crate::slacker::support`):
///
/// | `slacker_fraction` | `support(anchor)`        | Result |
/// |---|---|---|
/// | `0.0` (any anchor) | any                      | base anchor unchanged, `SlackerResolution::Base` |
/// | `> 0.0`            | `NotRecommended`/`NoData` | base anchor unchanged, `SlackerResolution::Base` (defensive; the UI disables the picker for these anchors) |
/// | `> 0.0`            | `Curve(c)`, point lands on Shore A  | `from_effective_shore(ShoreReading::A(points))`, `Interpolated` |
/// | `> 0.0`            | `Curve(c)`, point lands on Shore 00 | `from_effective_shore(ShoreReading::DoubleZero(points))`, `Interpolated` |
/// | `> 0.0`            | `Curve(c)`, point lands on Shore 000 | `ECOFLEX_00_10` material, `FlooredAtSoftestAnchor` |
/// | `> 0.0`            | `Curve(c)`, no exact-match point at the fraction | base + a warning surfaced via `Err`; the UI's `resolve_slacker_fraction` snaps off-curve inputs before they reach here |
///
/// # Errors
///
/// - The anchor key is not in the catalog (delegated to
///   [`silicone_for_anchor`]).
/// - The slacker fraction is `> 0.0` but `Support::Curve` has no
///   point at that fraction (within `f64::EPSILON`) — a wiring bug
///   the UI's `resolve_slacker_fraction` should have prevented.
/// - sim-soft's `from_effective_shore` rejects the interpolated
///   point (out-of-range against the anchor family's bracketing
///   pair).
fn effective_silicone_for_layer(layer: &SimLayer) -> Result<(SiliconeMaterial, SlackerResolution)> {
    let base = silicone_for_anchor(&layer.anchor_key)?;
    // Zero-fraction shortcut: the base material is the answer
    // regardless of `Support`. Bit-exact identity to pre-7.5
    // behavior, which is what keeps both regression ramps' numbers
    // unchanged.
    if layer.slacker_fraction == 0.0 {
        return Ok((base, SlackerResolution::Base));
    }
    let support = crate::slacker::support(&layer.anchor_key);
    let curve = match support {
        crate::slacker::Support::Curve(c) => c,
        // The recipe panel disables the picker for these two
        // variants, so a non-zero fraction reaching here is a wiring
        // surprise — fall back to base rather than fail the sim.
        crate::slacker::Support::NotRecommended | crate::slacker::Support::NoData => {
            return Ok((base, SlackerResolution::Base));
        }
    };
    // The UI snaps `slacker_fraction` to the curve's tabulated points
    // (`resolve_slacker_fraction` in `main.rs`). Linear search through
    // ≤ 5 points; binary-search overhead would be noise.
    let point = curve
        .iter()
        .find(|p| (p.slacker_fraction - layer.slacker_fraction).abs() < f64::EPSILON)
        .ok_or_else(|| {
            anyhow!(
                "slacker fraction {} is off the {} curve — the UI's resolve_slacker_fraction \
                 should have snapped it to an exact point",
                layer.slacker_fraction,
                layer.anchor_key,
            )
        })?;
    // `points: u32` from the TB; harmless cast to f64 (Shore values
    // are 0-100 / 0-50 ranges, far under `2^53`).
    #[allow(clippy::cast_precision_loss)]
    let shore_points = f64::from(point.hardness.points);
    match point.hardness.scale {
        crate::slacker::ShoreScale::A => {
            let mat = SiliconeMaterial::from_effective_shore(ShoreReading::A(shore_points), None)
                .map_err(|e| {
                anyhow!("sim-soft from_effective_shore(Shore A {shore_points}) failed: {e:?}")
            })?;
            Ok((mat, SlackerResolution::Interpolated))
        }
        crate::slacker::ShoreScale::OO => {
            let mat = SiliconeMaterial::from_effective_shore(
                ShoreReading::DoubleZero(shore_points),
                None,
            )
            .map_err(|e| {
                anyhow!("sim-soft from_effective_shore(Shore 00 {shore_points}) failed: {e:?}")
            })?;
            Ok((mat, SlackerResolution::Interpolated))
        }
        crate::slacker::ShoreScale::OOO => {
            // Shore 000 (gel scale) is softer than ECOFLEX_00_10 (the
            // softest sim-soft anchor). No published Yeoh data;
            // floor to ECOFLEX_00_10 as a conservative over-stiffness.
            // The user can read the recipe's TRUE hardness in the
            // panel; the sim simply caps below 00-10.
            Ok((ECOFLEX_00_10, SlackerResolution::FlooredAtSoftestAnchor))
        }
    }
}

/// Per-scan-SDF offsets (meters) of the *internal* layer boundaries —
/// one per adjacent layer pair, so `N` layers yield `N - 1`
/// thresholds.
///
/// Boundary `i` (between layer `i` and layer `i + 1`) is layer `i`'s
/// outer surface, at `sum(thickness[0..=i]) - cavity_inset_m`.
/// Strictly increasing (thicknesses are positive), as
/// [`LayeredScalarField::new`] requires. Empty for a single-layer
/// design — the caller uses a [`ConstantField`] instead.
pub fn layer_boundary_thresholds(design: &SimDesign) -> Vec<f64> {
    let mut cumulative = 0.0;
    design
        .layers
        .iter()
        .take(design.layers.len().saturating_sub(1))
        .map(|layer| {
            cumulative += layer.thickness_m;
            cumulative - design.cavity_inset_m
        })
        .collect()
}

/// Build one per-tet scalar parameter field (μ, C₂, or λ) over the
/// layer stack, keyed on the scan SDF.
///
/// `thresholds` are the [`layer_boundary_thresholds`]; `values` is
/// the per-layer parameter innermost-first, with
/// `values.len() == thresholds.len() + 1`. For a single-layer design
/// `thresholds` is empty — [`LayeredScalarField::new`] panics on
/// empty thresholds, so that case uses a [`ConstantField`] of the
/// lone value.
fn layered_param_field(
    scan_sdf: &GridSdf,
    thresholds: &[f64],
    values: Vec<f64>,
) -> Box<dyn Field<f64>> {
    if thresholds.is_empty() {
        // `values` is caller-guaranteed non-empty (≥ 1 layer); the
        // `0.0` fallback is unreachable defensive code.
        Box::new(ConstantField::new(values.first().copied().unwrap_or(0.0)))
    } else {
        Box::new(LayeredScalarField::new(
            Box::new(scan_sdf.clone()),
            thresholds.to_vec(),
            values,
        ))
    }
}

/// The 7.1 deliverable — the device-wall tet mesh + the rigid
/// intruder, geometry-and-materials only (no solve; that is 7.2+).
///
/// Intentionally no `Debug` derive: `mesh` + `intruder` hold the full
/// tet mesh and scan SDF (tens of thousands of tets / faces), so a
/// derived `Debug` would be unreadable and a `dbg!` footgun — mirrors
/// `MeshingHints`'s no-`Debug` rationale. Inspect the small fields
/// (the offsets, `bounds`, `cell_size_m`, `n_tets`) directly.
pub struct InsertionGeometry {
    /// Device-wall tet mesh with per-tet Yeoh materials sampled from
    /// the layer stack. The solver (7.2) consumes this by value.
    pub mesh: SdfMeshedTetMesh<Yeoh>,
    /// The scan-derived rigid intruder, as the flood-fill [`GridSdf`]
    /// the penalty contact primitive consumes — the same SDF that
    /// drove the body geometry; the solve offsets it per interference.
    pub intruder: GridSdf,
    /// Scan-SDF offset (m) of the cavity surface — `-cavity_inset_m`.
    pub cavity_offset_m: f64,
    /// Scan-SDF offset (m) of the outer skin —
    /// `total_thickness - cavity_inset_m`.
    pub outer_offset_m: f64,
    /// The BCC-lattice / interval-prune bound the `mesh` was built
    /// with. Kept so the 7.2 solve can rebuild the outer-envelope +
    /// intruder `Solid`s from `intruder` without re-deriving it (and
    /// so 7.3's per-step re-mesh can reuse it).
    pub bounds: Aabb,
    /// The BCC lattice spacing (m) the `mesh` was built with — the
    /// 7.2 solve sizes the Dirichlet pin-band at `0.5 * cell_size_m`.
    pub cell_size_m: f64,
    /// Tet count of `mesh` — surfaced for the 7.4 UI readout + tests.
    pub n_tets: usize,
    /// Slice 7.4 — per-tet layer index, length `n_tets`, indexed by
    /// [`TetId`]. Derived from the scan SDF distance at each tet's
    /// centroid bucketed against [`layer_boundary_thresholds`]:
    /// values land in `0..design.layers.len()`, innermost-first
    /// (`0` = cavity-side layer, `n−1` = outer skin). Drives the
    /// Insertion-Sim panel's per-layer aggregates + heat-map
    /// projection (each layer's per-tet readouts feed only that
    /// layer's shell-vertex coloring). The same partition that
    /// builds the per-tet Yeoh material, surfaced explicitly so a
    /// consumer doesn't have to back it out by comparing
    /// `Yeoh` parameters (which collapse when two layers happen to
    /// share an anchor key).
    pub per_tet_layer: Vec<usize>,
}

/// Build the Route-A insertion-sim geometry: the device-wall tet mesh
/// with per-tet Yeoh materials, plus the rigid intruder SDF.
///
/// Geometry (mirrors the sim-soft rows 21–25 layered-sleeve path):
/// decimate the scan → flood-fill [`GridSdf`] ([`build_grid_sdf`] —
/// the 7.3a fix for `mesh_sdf`'s ~12%-wrong sign on the non-manifold
/// decimated scan) → cavity = `scan.offset(-inset)`, outer skin =
/// `scan.offset(total - inset)`, `body = outer.subtract(cavity)`.
/// Materials: each layer's base
/// silicone ([`silicone_for_anchor`]) supplies `(μ, C₂, λ)`; a
/// [`LayeredScalarField`] per parameter partitions the wall by
/// distance-from-scan at [`layer_boundary_thresholds`] (a
/// [`ConstantField`] for a single-layer design). The mesh is built
/// via `SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh`.
///
/// `sdf_target_faces` decimates the SDF source — the 7.0 spike found
/// low (~1.5–3k) is best (tet count + quality track `cell_size_m`,
/// not face count). `cell_size_m` is the BCC lattice spacing; the
/// rows 21–25 contact-robustness envelope wants ≈ 4 mm, ideally with
/// `cell_size_m ≤` the thinnest layer so each layer gets ≥ 1 cell.
///
/// No solve, no contact wiring — that is 7.2+.
///
/// # Errors
///
/// - the design has no layers;
/// - a layer has a non-positive or non-finite thickness, or
///   `cavity_inset_m` is non-finite — a strictly-monotone wall
///   partition needs positive, finite thicknesses (otherwise
///   `LayeredScalarField::new` would panic rather than error);
/// - a layer names an anchor key outside the catalog;
/// - [`build_grid_sdf`] fails (the bbox margin is too small to seed
///   the outside flood);
/// - `SdfMeshedTetMesh::from_sdf_yeoh` fails (empty mesh — e.g. a
///   degenerate design whose cavity has collapsed — or a non-finite
///   SDF sample).
///
/// # Panics
///
/// `cell_size_m` is a programmer-set knob, not validated here: a
/// non-positive `cell_size_m` (or a scan degenerate enough that its
/// bbox is ill-formed) forwards a panic from `SdfGrid::new` /
/// `BccLattice::new` — the same "caller-supplied invariant" posture
/// sim-soft + cf-geometry document for that argument.
pub fn build_insertion_geometry(
    scan: &IndexedMesh,
    design: &SimDesign,
    cap_planes: &[CapPlane],
    sdf_target_faces: usize,
    cell_size_m: f64,
) -> Result<InsertionGeometry> {
    if design.layers.is_empty() {
        return Err(anyhow!("insertion-sim design has no layers"));
    }
    if !design.cavity_inset_m.is_finite() {
        return Err(anyhow!(
            "insertion-sim cavity inset is non-finite ({})",
            design.cavity_inset_m
        ));
    }
    // Every layer needs a positive, finite thickness: the wall
    // partition's thresholds are cumulative thicknesses, and
    // `LayeredScalarField::new` *panics* (not errors) on a
    // non-monotone or non-finite threshold list. Catch it here so the
    // `Result` contract holds.
    for (i, layer) in design.layers.iter().enumerate() {
        if !(layer.thickness_m.is_finite() && layer.thickness_m > 0.0) {
            return Err(anyhow!(
                "insertion-sim layer {i} has a non-positive or non-finite thickness ({})",
                layer.thickness_m
            ));
        }
    }

    let total_thickness_m: f64 = design.layers.iter().map(|l| l.thickness_m).sum();
    let cavity_offset_m = -design.cavity_inset_m;
    let outer_offset_m = total_thickness_m - design.cavity_inset_m;

    // The BCC lattice / interval-prune bound must contain the whole
    // body. The body's outermost surface is `scan.offset(outer_offset_m)`;
    // when `outer_offset_m > 0` it reaches that far beyond the scan
    // bbox, when ≤ 0 the body sits inside it. Either way, one cell of
    // slack past the larger extent suffices.
    let bounds = scan_aabb(scan, outer_offset_m.max(0.0) + cell_size_m);

    // Per-layer Yeoh parameters, innermost-first. Slice 7.5: resolve
    // each layer through `effective_silicone_for_layer` so the
    // Slacker fraction shifts the effective Shore + Yeoh params
    // (representable Shore A / Shore 00 outcomes lift through
    // `SiliconeMaterial::from_effective_shore`; Shore 000 outcomes
    // floor to `ECOFLEX_00_10` — see `SlackerResolution` for the
    // resolution table). At `slacker_fraction = 0.0` every layer
    // returns its base anchor bit-exact, so both regression ramps
    // keep their pre-7.5 numbers.
    let materials: Vec<SiliconeMaterial> = design
        .layers
        .iter()
        .map(|layer| effective_silicone_for_layer(layer).map(|(mat, _resolution)| mat))
        .collect::<Result<Vec<_>>>()?;
    let thresholds = layer_boundary_thresholds(design);

    let decimated = decimate_for_sdf(scan, sdf_target_faces);
    // Flood-fill `GridSdf`, not `mesh_sdf::SignedDistanceField`: the
    // 7.3a diagnostic found the closest-face-normal sign ~12% wrong on
    // the sloppy-decimated (non-manifold) scan. The grid is finer than
    // the BCC cell so trilinear interp stays sub-mm; the wall-band
    // threshold (0.75·grid_cell ≥ 0.5·grid_cell) keeps the flood
    // leak-proof.
    let grid_cell_m = 0.75 * cell_size_m;
    let (scan_sdf, _grid_report) =
        build_grid_sdf(&decimated, bounds, grid_cell_m, 0.75 * grid_cell_m)
            .context("build flood-fill GridSdf from the decimated scan")?;

    // Candidate-A two-SDF body geometry (per the redesign spec §2 A4
    // at `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_REDESIGN_SPEC.md`).
    // Closed-body SDF supplies the sign; open-body SDF (cap polygons
    // stripped) supplies the unsigned-rind magnitude that pinned-floor
    // shell anchors the floor on. The open SDF's sign is sidestepped
    // by construction — `pinned_floor_shell`'s private
    // `UnsignedRindSdf` adapter consumes only `.abs()`. With no caps
    // the primitive short-circuits to a plain isotropic offset, so we
    // skip the open-mesh decimation + SDF build entirely and reuse the
    // closed `Arc` for both arguments (it is never queried).
    let closed_sdf_arc: Arc<dyn cf_design::Sdf> = Arc::new(scan_sdf.clone());
    let open_sdf_arc: Arc<dyn cf_design::Sdf> = if cap_planes.is_empty() {
        Arc::clone(&closed_sdf_arc)
    } else {
        let decimated_open = dome_wall_only_mesh(&decimated, cap_planes);
        let (open_sdf, _open_grid_report) =
            build_grid_sdf(&decimated_open, bounds, grid_cell_m, 0.75 * grid_cell_m)
                .context("build flood-fill GridSdf from the cap-stripped decimated scan")?;
        Arc::new(open_sdf)
    };
    let cap_tuples: Vec<(Point3<f64>, Vector3<f64>)> =
        cap_planes.iter().map(CapPlane::as_tuple).collect();

    // Five `LayeredScalarField`s (or `ConstantField`s) over the same
    // scan-distance partition — three Yeoh parameters + two
    // calibrated principal-stretch caps — mirroring the row-23
    // `build_material_field` precedent + threading the per-anchor
    // `0.8 · λ_break` / `0.30` bounds through to the per-tet
    // validity gate per H4 (`docs/CANDIDATE_H4_YEOH_BOUND_CALIBRATION_SPEC.md`).
    // The 5-arg constructor `from_yeoh_fields_with_bounds` routes
    // each per-tet `Yeoh` through `with_principal_stretch_bounds`,
    // so the solver's `check_validity_at_step_start` gates against
    // the silicone's calibrated cap rather than the legacy
    // symmetric `max_stretch_deviation = 1.0` fallback that the
    // pre-H4 3-arg path implicitly used.
    let material_field = MaterialField::from_yeoh_fields_with_bounds(
        layered_param_field(
            &scan_sdf,
            &thresholds,
            materials.iter().map(|m| m.mu).collect(),
        ),
        layered_param_field(
            &scan_sdf,
            &thresholds,
            materials.iter().map(|m| m.c2).collect(),
        ),
        layered_param_field(
            &scan_sdf,
            &thresholds,
            materials.iter().map(|m| m.lambda).collect(),
        ),
        layered_param_field(
            &scan_sdf,
            &thresholds,
            materials
                .iter()
                .map(|m| m.validity_max_principal_stretch)
                .collect(),
        ),
        layered_param_field(
            &scan_sdf,
            &thresholds,
            materials
                .iter()
                .map(|m| m.validity_min_principal_stretch)
                .collect(),
        ),
    );

    // Route-A device wall: outer skin minus cavity void. Both shells
    // go through `pinned_floor_shell` (candidate A) — when `cap_planes`
    // is empty this degenerates to the previous
    // `Solid::from_sdf(scan_sdf).offset(...)` byte-identically; when
    // caps are present each shell gains a flat floor pinned at every
    // cap polygon. `cavity_offset_m` is negative (cavity inset *inside*
    // the scan); `outer_offset_m` may be positive (outer skin extends
    // out from the scan) or negative (thin total wall sits inside).
    let cavity = pinned_floor_shell(
        closed_sdf_arc.clone(),
        open_sdf_arc.clone(),
        bounds,
        &cap_tuples,
        cavity_offset_m,
    );
    let outer = pinned_floor_shell(
        closed_sdf_arc,
        open_sdf_arc,
        bounds,
        &cap_tuples,
        outer_offset_m,
    );
    let body = outer.subtract(cavity);

    let hints = MeshingHints {
        bbox: aabb3_for_meshing(&bounds),
        cell_size: cell_size_m,
        material_field: Some(material_field),
    };
    // `MeshingError` does not implement `std::error::Error` — wrap by
    // hand via its `Debug`, same as `run_sdf_bridge_spike`.
    let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints).map_err(|e| {
        anyhow!("tet-mesh the device-wall body via SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh: {e:?}")
    })?;
    let n_tets = mesh.n_tets();

    // Slice 7.4 per-tet layer assignment. Sample the scan SDF at each
    // tet centroid, bucket against `thresholds`: the partition is
    // monotone — `scan_sdf(centroid) < thresholds[0]` is layer 0
    // (innermost), `< thresholds[i]` is layer `i`, the tail bucket
    // is layer `n_layers - 1` (outermost). Matches the
    // `LayeredScalarField` partition exactly (the material field is
    // sampled at the centroid by `materials_from_field` per
    // `sim/L0/soft/src/mesh/mod.rs:209`).
    let n_layers = design.layers.len();
    let per_tet_layer: Vec<usize> = (0..n_tets)
        .map(|t| {
            // `t as TetId` (u32) — Phase 4 BCC meshes stay well under
            // `u32::MAX` per the `Mesh` trait docs.
            #[allow(clippy::cast_possible_truncation)]
            let verts = mesh.tet_vertices(t as TetId);
            let positions = mesh.positions();
            let centroid = (positions[verts[0] as usize]
                + positions[verts[1] as usize]
                + positions[verts[2] as usize]
                + positions[verts[3] as usize])
                * 0.25;
            let sd = scan_sdf.eval(Point3::from(centroid));
            // Linear scan — `thresholds.len()` is at most
            // `LAYER_COUNT_MAX − 1 = 5`, no payoff from binary
            // search. `n_layers - 1` covers the tail bucket
            // (sd ≥ last threshold).
            thresholds
                .iter()
                .position(|&th| sd < th)
                .unwrap_or(n_layers - 1)
        })
        .collect();

    Ok(InsertionGeometry {
        mesh,
        // The scan SDF doubles as the rigid intruder — the press-fit
        // ramp (7.2) drives this into the cavity.
        intruder: scan_sdf,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets,
        per_tet_layer,
    })
}

// ── 7.2 — single static insertion solve ────────────────────────────

/// Static-solve time-step. `dt = 1.0` collapses inertia for a
/// quasi-static solve — the rows 21–25 `STATIC_DT` precedent.
const STATIC_DT: f64 = 1.0;

/// Newton-iteration cap for the Yeoh insertion solve. Matches the
/// row-23 `scan-fit-3layer-sleeve-yeoh-ramp` cap: the Yeoh path needs
/// more iterations than row 22's Neo-Hookean 100 as contact deepens.
/// `replay_step` *panics* if the solve doesn't converge within this
/// cap — see [`run_single_insertion_step`]'s `# Panics`.
const MAX_NEWTON_ITER: usize = 150;

/// Newton convergence tolerance (free-DOF residual norm, in newtons)
/// for the insertion solve. `SolverConfig::skeleton()`'s `1e-10`
/// default is a walking-skeleton bar far tighter than this tool
/// needs: cf-device-design is a *relative-comparison* engineering aid
/// (Fork B), and a `0.1`-N out-of-balance residual is physically
/// negligible against the tens-of-newtons contact forces.
///
/// `1e-1` is not arbitrary — the 7.3b.1 finding is that the deeper
/// ramp steps Armijo-*stall* (non-SPD tangent near the solution; the
/// capsule geometry's secondary pathology) at a residual floor right
/// around `0.1 N`. Setting `tol` at that floor converts those stalls
/// into clean (loose-but-physically-exact) convergences, which is
/// what lets the ramp seat the intruder to a meaningful depth. The
/// shallow steps still converge far below this (to ~`1e-5`) — `tol`
/// only bites once a step hits the stall floor.
const INSERTION_SOLVE_TOL: f64 = 1e-1;

/// Shared solver config for the insertion solve — the walking-
/// skeleton defaults with `dt` (static), `max_newton_iter`, and `tol`
/// set for this tool. Used by [`run_single_insertion_step`] (7.2) and
/// [`run_insertion_ramp`] (7.3b).
fn insertion_solver_config() -> SolverConfig {
    let mut config = SolverConfig::skeleton();
    config.dt = STATIC_DT;
    config.max_newton_iter = MAX_NEWTON_ITER;
    config.tol = INSERTION_SOLVE_TOL;
    // F3 recon candidate A — gated LM opt-in (per
    // `docs/F3_RECON_A_GATED_LM_SPEC.md`). The same `LmConfig::fork_b()`
    // preset F3.4 used; the behavioral change is in sim-soft's
    // `try_solve_impl` — LM rescue now fires ONLY on first-pass LU +
    // Armijo failure (gated activation), not on every Llt non-PD
    // detection (eager activation per F3 spec §2.2 — empirically
    // falsified 2026-05-18 EVENING, see `docs/F3_FALSIFICATION_BOOKMARK.md`).
    // The bit-equal-when-dormant contract preserves the cavity = 3 mm
    // baseline by keeping LM inactive when the LU + Armijo path
    // succeeds; LM escalates only when needed at hard-conditioned
    // iters. `try_replay_step` + `solver_failure_message` +
    // `catch_unwind` belt-and-suspenders surface plumbing reused
    // unchanged from F3.4 — gated A adds no SolverFailure variants.
    config.lm_regularization = Some(LmConfig::fork_b());
    config
}

/// Penalty-contact stiffness `κ` for the insertion solve. 7.3b.1
/// found `PenaltyRigidContact::new`'s default `1e4` keeps the Newton
/// tangent non-SPD near the solution past ~1.5 mm interference —
/// full-surface contact (the *whole* cavity wall engages at once,
/// unlike the rows' localized probe) concentrates the penalty
/// Hessian. A gentler `1e3` widens the convergeable depth envelope;
/// the tradeoff is slightly more residual penetration, acceptable
/// for this relative-comparison tool (Fork B).
///
/// Composes orthogonally with the C′.a-pinned smoothing window
/// [`INSERTION_CONTACT_SMOOTHING_EPS_M`] (the const's docstring
/// carries the pinned value + the full sweep table; see
/// `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` for the original
/// candidate-C design + `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md`
/// §9 for the C′.a case-A ship rationale).
const INSERTION_CONTACT_KAPPA: f64 = 1.0e3;

/// Penalty-contact band `d̂` (meters) for the insertion solve —
/// matches sim-soft's crate-private `PENALTY_DHAT_DEFAULT`
/// (1 mm). `with_params` requires it explicitly once `κ` is tuned.
const INSERTION_CONTACT_DHAT: f64 = 1.0e-3;

/// One-sided smoothing window `ε` (meters) above `d̂` for the
/// insertion solve's penalty contact (F3 recon B candidate C). Pairs
/// with `sd ∈ (d̂, d̂+ε)` contribute a quintic-Hermite-tapered
/// penalty that reaches 0 at `sd = d̂+ε`; this makes the assembled
/// contact Hessian `H_contact(x)` C⁰ across active-pair boundaries.
/// See `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` for the design +
/// `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` for the C.2
/// sweep falsification + C′.a bisection that found this value.
///
/// **PINNED at ε = 0.075 mm** per the C′.a ε-bisection sweep
/// 2026-05-18 LATE-EVENING (cavity = 5 mm, layers 10+3 mm,
/// sock_over_capsule.cleaned.stl):
///
/// | ε (mm) | steps converged | r_norm floor | stall mode | LM rescues |
/// |---|---|---|---|---|
/// | 0 (gated-A baseline) | 0/16 | 1.784 | Armijo iter 61 | 1 mild |
/// | 0.025 (C′.a) | 0/16 | 0.231 | Armijo iter 108 | 3 stiff |
/// | 0.05 (C′.a) | 0/16 | 0.200 | Armijo iter 147 | 4 moderate |
/// | **0.075 (C′.a)** | **16/16** | converges seated 83.35 mm | — | **0** |
/// | 0.1 (C.2) | 0/16 | 0.384 | Armijo iter 126 | 2 stiff |
/// | 0.25 (C.2) | 0/16 | 0.753 | iter cap 150 | 2 stiff |
///
/// The response is U-shaped: a **narrow converging window centered
/// at ε ≈ 0.075 mm**.  The two sides are asymmetric — below the
/// window the residual floor plateaus around r_norm ≈ 0.2
/// (smoothing band too narrow to cover enough chattering pairs);
/// above the window the floor climbs sharply with ε (0.1 →
/// r_norm 0.384, 0.25 → 0.753, band-widening backfire dominates).
/// The C.0 spec's "monotonic improvement with ε" prediction was
/// wrong; the empirical structure is a sweet spot where the
/// band-widening backfire (hyp 3 in the falsification bookmark)
/// and the chattering-suppression effect balance.  C′.a confirms
/// hyp 3 on the upper side — wider ε bands bring more pairs into
/// the tapered regime, degrading the assembled tangent's
/// eigenstructure once past the optimum.
///
/// **CAVITY-SPECIFIC**: the chosen ε = 0.075 mm converges 16/16 at
/// cavity ≤ 5 mm but stalls at cavity 6 mm (C.3 probe gate,
/// r_norm 0.536).  See `CavityState::inset_slider_range_m` in
/// `main.rs` for the UI cap that enforces this bound + the bookmark
/// §9.4 for the probe-gate data.  Generalizing past 5 mm would
/// require a per-cavity ε (would need a UI slider per
/// [[feedback-strip-the-knob-when-default-works]] — deferred until
/// empirical multi-modal evidence) or a composed mechanism
/// (smoothed contact + SDF-normal smoothing for hyp 1 / step-0
/// warmup for hyp 2).
///
/// **Wire-up preserved** per
/// [[feedback-spec-falsified-revert-opt-in-keep-surface]]:
/// `intruder_contact_at` + `intruder_contact_sliding_at` route
/// through C.1's
/// [`PenaltyRigidContact::with_params_and_smoothing`] +
/// [`PenaltyRigidContact::with_params_and_smoothing_and_interior_cutoff`].
/// Future recon (e.g. SDF-normal smoothing for hyp 1 or step-0
/// warmup for hyp 2) flips this const + composes with the existing
/// C.1 surface.
const INSERTION_CONTACT_SMOOTHING_EPS_M: f64 = 0.075e-3;

/// Number of offset samples for per-query contact-normal averaging
/// (F3 recon B candidate E.b — orthogonal axis to
/// [`INSERTION_CONTACT_SMOOTHING_EPS_M`], which smooths the gap
/// function `(d̂ − sd)` instead). `1` (default) disables averaging —
/// `n = prim.grad(p)` bit-equal to pre-E.b behavior. `7` enables the
/// 6-face axis-aligned neighborhood
/// (`n_avg = normalize(prim.grad(p) + Σ prim.grad(p ± r·e_{x,y,z}))`).
///
/// **Pinned at `1` (disabled) — E.b.4 case-E falsification ship
/// 2026-05-19**.  The cavity = 6 mm sweep at
/// `(k=7, r ∈ {0.5, 1.0, 2.0} mm)` all converged step 1 (seated
/// 5.21 mm) but hit a Yeoh material-validity wall at step 2.  The
/// 5 mm sanity gate at the candidate pin `(7, 1.0 mm)` ALSO
/// regressed to 1/16 (step 2 Yeoh wall, tet 3258
/// max_stretch_deviation = 1.002).  3 mm sanity gate held at 16/16
/// ZERO LM rescues, but the 5 mm regression is the spec's CASE E —
/// E.b's averaging shifts the equilibrium toward the Yeoh validity
/// bound at every cavity, costing 5 mm baseline for no net gain at
/// 6 mm (the 1/16 "win" was the Yeoh wall surfacing — same outcome
/// a hypothetical no-chattering cavity-6mm solve would have
/// reached, empirically confirmed by the post-case-E N_STEPS sweep
/// at §10 of the bookmark — N=8/12/20/24 all hit Yeoh tet 3206 at
/// step 2 WITHOUT E.b).  Full falsification analysis in
/// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md`; §10 has the
/// post-case-E sweep + reframe (cavity > 5 mm is a real material
/// limit at this mesh resolution, NOT chattering) + the end-of-
/// solve validity check that surfaces invalid converged states
/// honestly (commit `2739717e` in `sim-soft`).
///
/// **Sweep table** (cavity = 6 mm, layers 10+3 mm,
/// sock_over_capsule.cleaned.stl, 2026-05-19):
///
/// | `(k, r)` | step 1 | F (N) | iters | step 2 |
/// |---|---|---|---|---|
/// | (1, 0) baseline | 0/16 r_norm 0.536 chattering stall | — | 150 cap | — |
/// | (7, 0.5 mm) | 1/16 seated 5.21 mm | 15.23 | 34 | Yeoh tet 1458 |
/// | (7, 1.0 mm) | 1/16 seated 5.21 mm | 8.1 | ~26 | Yeoh tet 3206 |
/// | (7, 2.0 mm) | 1/16 seated 5.21 mm | 8.15 | 25 | Yeoh tet (likely) |
///
/// Sanity gates at `(k=7, r=1.0 mm)`: 3 mm 16/16 clean ✓; 5 mm
/// 1/16 Yeoh tet 3258 ✗ (case-E regression).
///
/// **Bit-equal-when-disabled wire-up** per
/// [[feedback-spec-falsified-revert-opt-in-keep-surface]] —
/// `intruder_contact_at` and `intruder_contact_sliding_at` both
/// route through the
/// [`PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging`]
/// family (the `..._and_interior_cutoff` variant for the sliding
/// case). At `k = 1` the `averaged_normal` helper short-circuits to
/// `prim.grad(p)` bit-equal — pre-E.b arithmetic preserved. Surface
/// plumbing (sim-soft constructors + helper + 9 unit tests in
/// `penalty_normal_averaging.rs` + this routing) survives the
/// revert; a future recon composing E.b with a Yeoh-wall fix flips
/// this const + pairs with
/// [`INSERTION_CONTACT_NORMAL_AVG_RADIUS_M`].
const INSERTION_CONTACT_NORMAL_AVG_K: u8 = 1;

/// Offset radius (m) for per-query contact-normal averaging. Only
/// consulted when [`INSERTION_CONTACT_NORMAL_AVG_K`] `> 1`. Pinned
/// at `0.0` per the case-E falsification — see the K const
/// docstring above for the full sweep table and falsification
/// rationale. A future recon composing E.b with a Yeoh-wall fix
/// would re-pin this value (initial sweep range tested:
/// `{0.5, 1.0, 2.0} mm` per
/// `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` §2.3).
const INSERTION_CONTACT_NORMAL_AVG_RADIUS_M: f64 = 0.0;

/// Build the rigid-intruder contact primitive for a given press-fit
/// `interference_m` — the scan SDF offset by
/// `interference_m + cavity_offset_m` (so `interference_m =
/// cavity_inset_m` reproduces the bare scan, `0` sits flush with the
/// cavity wall). Shared by [`run_single_insertion_step`] (7.2) and
/// [`run_insertion_ramp`] (7.3b — the ramp rebuilds this per step as
/// the intruder seats deeper).
fn intruder_contact_at(
    intruder: &GridSdf,
    bounds: Aabb,
    interference_m: f64,
    cavity_offset_m: f64,
) -> PenaltyRigidContact {
    let intruder_solid =
        Solid::from_sdf(intruder.clone(), bounds).offset(interference_m + cavity_offset_m);
    PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![intruder_solid],
        INSERTION_CONTACT_KAPPA,
        INSERTION_CONTACT_DHAT,
        INSERTION_CONTACT_SMOOTHING_EPS_M,
        INSERTION_CONTACT_NORMAL_AVG_K,
        INSERTION_CONTACT_NORMAL_AVG_RADIUS_M,
    )
}

/// Build the Dirichlet boundary conditions: pin the outer-skin
/// vertices — those within `0.5 * cell_size_m` of the outer envelope
/// `scan.offset(outer_offset_m)`, filtered to solver-referenced
/// vertices (BCC orphans are in no tet). The intruder, not a loaded
/// BC, drives the deformation. Constant across a ramp (the outer skin
/// does not move) — [`run_insertion_ramp`] builds it once and clones.
///
/// # Errors
///
/// No outer-skin vertex lands in the pin-band — the wall has nothing
/// to react against (`cell_size_m` too coarse for the wall, or a
/// degenerate geometry).
fn outer_skin_bc(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    intruder: &GridSdf,
    bounds: Aabb,
    outer_offset_m: f64,
    cell_size_m: f64,
) -> Result<BoundaryConditions> {
    let referenced: BTreeSet<VertexId> = referenced_vertices(mesh).into_iter().collect();
    let outer_envelope = Solid::from_sdf(intruder.clone(), bounds).offset(outer_offset_m);
    let band_tol = 0.5 * cell_size_m;
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| {
        outer_envelope.eval(Point3::from(*p)).abs() < band_tol
    })
    .into_iter()
    .filter(|v| referenced.contains(v))
    .collect();
    if pinned.is_empty() {
        return Err(anyhow!(
            "no outer-skin vertex landed in the {band_tol:.4} m Dirichlet band — the \
             device wall has nothing pinned to react against (cell_size_m too coarse \
             for the wall, or a degenerate geometry)"
        ));
    }
    Ok(BoundaryConditions {
        pinned_vertices: pinned,
        loaded_vertices: Vec::new(),
    })
}

/// Outputs from one static insertion solve step.
///
/// A returned `InsertionStep` is converged by construction —
/// `replay_step` panics rather than return a non-converged step
/// (see [`run_single_insertion_step`]'s `# Panics`), so there is no
/// `converged` flag; `iter_count` + `final_residual_norm` are
/// convergence *diagnostics*, not a pass/fail.
///
/// Derives `Debug` (unlike [`InsertionGeometry`], which omits it):
/// `x_final` is a flat `Vec<f64>` that prints readably, and seeing
/// the iter count / residual / pin count inline in a test-failure
/// message is worth the verbosity — `InsertionGeometry`'s opaque
/// tet-mesh + SDF structs are not.
#[derive(Debug, Clone)]
pub struct InsertionStep {
    /// Converged vertex positions, vertex-major xyz (length
    /// `3 * n_vertices`).
    pub x_final: Vec<f64>,
    /// Newton iterations the solve took.
    pub iter_count: usize,
    /// Free-DOF residual norm at convergence.
    pub final_residual_norm: f64,
    /// Number of outer-skin vertices pinned as Dirichlet BCs — the
    /// device wall reacts against these.
    pub n_pinned: usize,
}

/// Run ONE static insertion solve: press the scan-derived rigid
/// intruder `interference_m` into the device cavity and solve for the
/// deformed device wall.
///
/// Mirrors the rows 21–25 single-step pattern. Boundary conditions
/// pin the outer-skin vertices (within `0.5 * cell_size_m` of the
/// outer envelope, filtered to solver-referenced vertices); the
/// intruder drives the deformation through `PenaltyRigidContact`. The
/// intruder at interference `d` is the scan SDF offset by
/// `d + cavity_offset_m` — at `d = cavity_inset_m` it is the bare
/// scan (full press-fit), at `d = 0` it sits flush with the cavity
/// wall (no penetration). The solve is static (`dt = 1.0`).
///
/// Consumes `geometry` — `CpuNewtonSolver::new` takes the mesh by
/// value. [`run_insertion_ramp`] (7.3b) clones the prebuilt mesh per
/// step instead; `run_single_insertion_step` is one step only.
///
/// # Errors
///
/// - `interference_m` is non-finite;
/// - [`outer_skin_bc`] fails — no outer-skin vertex in the Dirichlet
///   pin-band (the wall has nothing to react against);
/// - the Newton solve fails to converge — Armijo line-search stall,
///   [`MAX_NEWTON_ITER`] cap, or doubly-failed Llt-then-Lu factor.
///   Under F3.4's Fork-B LM opt-in (see [`insertion_solver_config`]),
///   `try_replay_step` surfaces these as `Err(SolverFailure)` instead
///   of panicking; each variant is translated to an [`anyhow::Error`]
///   carrying the iter index + residual / context.
pub fn run_single_insertion_step(
    geometry: InsertionGeometry,
    interference_m: f64,
) -> Result<InsertionStep> {
    if !interference_m.is_finite() {
        return Err(anyhow!(
            "insertion interference is non-finite ({interference_m})"
        ));
    }

    let InsertionGeometry {
        mesh,
        intruder,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets: _,
        per_tet_layer: _,
    } = geometry;

    let n_vertices = mesh.n_vertices();
    let n_dof = 3 * n_vertices;

    // x_prev = the rest (undeformed) vertex positions, vertex-major xyz.
    let mut x_prev_flat = vec![0.0_f64; n_dof];
    for (v, p) in mesh.positions().iter().enumerate() {
        x_prev_flat[3 * v] = p.x;
        x_prev_flat[3 * v + 1] = p.y;
        x_prev_flat[3 * v + 2] = p.z;
    }

    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();
    let contact = intruder_contact_at(&intruder, bounds, interference_m, cavity_offset_m);
    let config = insertion_solver_config();

    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    // Empty θ — the insertion solve carries no differentiable
    // parameters (the intruder is kinematic, fixed at construction).
    let empty_theta: [f64; 0] = [];
    let theta = Tensor::from_slice(&empty_theta, &[0]);

    let solver = CpuNewtonSolver::new(Tet4, mesh, contact, config, bc);
    // MAINTENANCE NOTE — these `SolverFailure` arms mirror the variant
    // wording in `solver_failure_message` (the ramp consumer). Different
    // surfaces (anyhow! errors here, `failure_reason` strings there) and
    // intentionally different prefixes ("insertion solve …" vs the
    // step-N-prefixed viewport reason), but the per-variant *fields*
    // pulled out + the `{...:.3e}` formatting must stay in sync. If a
    // future `SolverFailure` variant lands, update BOTH sites.
    let step = match solver.try_replay_step(&x_prev, &v_prev, &theta, config.dt) {
        Ok(step) => step,
        Err(SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        }) => {
            return Err(anyhow!(
                "insertion solve Armijo-stalled at Newton iter {last_iter}, \
                 r_norm {last_r_norm:.3e}"
            ));
        }
        Err(SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        }) => {
            return Err(anyhow!(
                "insertion solve hit Newton iter cap {max_iter}, \
                 last r_norm {last_r_norm:.3e}"
            ));
        }
        Err(SolverFailure::DoublyFailedFactor {
            last_iter, context, ..
        }) => {
            return Err(anyhow!(
                "insertion solve doubly-failed factor at Newton iter {last_iter}: {context}"
            ));
        }
    };

    Ok(InsertionStep {
        x_final: step.x_final,
        iter_count: step.iter_count,
        final_residual_norm: step.final_residual_norm,
        n_pinned,
    })
}

// ── 7.3a fix — grid-sampled SDF with flood-fill sign ────────────────

/// A signed distance field backed by a [`SdfGrid`] whose **sign comes
/// from a flood fill**, not from `mesh_sdf`'s closest-face normal.
///
/// The 7.3a diagnostic root-caused the real-scan solve failure to
/// `mesh_sdf::SignedDistanceField`'s closest-face-normal sign being
/// ~12% wrong on the sloppy-decimated (non-manifold) scan. `GridSdf`
/// sidesteps it: [`build_grid_sdf`] samples the *unsigned* distance
/// (always reliable — just closest-triangle, topology-blind) on a
/// lattice, then flood-fills "outside" inward from the bounding-box
/// corners. The sign is therefore **topological** — immune to
/// non-manifold edges, inconsistent winding, and duplicate faces. It
/// is NOT immune to holes larger than a grid cell (the flood would
/// leak through), but the diagnostic confirmed the decimated iter-1
/// scan has only 2 boundary edges — far under that.
///
/// Intentionally no `Debug` derive: the inner [`SdfGrid`] is a
/// `Vec<f64>` of tens to hundreds of thousands of samples — a derived
/// `Debug` would be unreadable and a `dbg!` footgun (same rationale
/// as [`InsertionGeometry`]). Inspect via [`GridSdfReport`] instead.
#[derive(Clone)]
pub struct GridSdf {
    grid: SdfGrid,
}

impl Sdf for GridSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        // Clamped: contact / BC queries may land a hair outside the
        // grid; the grid spans the body + margin so this is exact in
        // practice and graceful at the edge.
        self.grid.distance_clamped(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.grid.gradient_clamped(p)
    }
}

/// Flood-fill health diagnostics from one [`build_grid_sdf`] call.
///
/// A topologically-sound result has the inside region as a *single*
/// connected component — a limb scan is one solid blob; more than one
/// means the flood leaked through a hole (the grid is too coarse, or
/// the scan has a genuine hole wider than a cell).
#[derive(Debug, Clone)]
pub struct GridSdfReport {
    /// Lattice dimensions `[width, height, depth]`.
    pub dims: [usize; 3],
    /// Lattice spacing (meters).
    pub grid_cell_m: f64,
    /// Lattice points flood-labelled `Outside`.
    pub n_outside: usize,
    /// Lattice points labelled `Inside` (interior + the inside half
    /// of the wall band, after label expansion).
    pub n_inside: usize,
    /// Wall-band lattice points (within `wall_threshold_m` of the
    /// surface) — pre-expansion count.
    pub n_wall: usize,
    /// Connected-component count of the final inside region. **1 is
    /// healthy**; more means the flood leaked.
    pub inside_components: usize,
    /// Wall-clock build time.
    pub build_ms: f64,
}

// `Region` enum + `neighbours6` helper + the inline 4-pass flood-fill
// in `build_grid_sdf` were promoted to mesh-sdf D.2's `CachedGridSdf`
// primitive (`docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`). The shared
// classifier core in mesh-sdf preserves the exact algorithm + region
// label semantics; this module now invokes it via
// [`CachedGridSdf::build`] and re-applies the Gaussian post-pass
// externally on the returned signed buffer (FEM contact-gradient C¹
// approximation, NOT load-bearing for sign correctness — kept here so
// mesh-sdf primitives stay smoothing-free).

/// Per-cell σ of the [`GridSdf`] signed-distance Gaussian pre-smooth
/// (slice 7.3c–7.3d, [`gaussian_smooth_3d_separable`]).
///
/// The smoothing makes the trilinear interpolant inside `SdfGrid`
/// approximate a C¹ function so the contact-side
/// `gradient = ∇φ / ‖∇φ‖` does not carry the C⁰ kinks the polyhedral
/// source mesh injects at facet boundaries.
///
/// Slice 7.3c shipped σ = 0.5 cell and saw the synthetic-icosphere
/// ramp from 87 % → 100 % but the iter-1 scan ramp only reaching
/// 75 % before what looked like a Yeoh-stretch-validity wall.
/// Recon-iter-4 (commit `<this commit>`) measured σ ∈ {0.5, 0.75,
/// 1.0, 1.5} and found the wall was **numerical, not material**:
/// at σ = 1.0 cell the iter-1 ramp reaches the **full 3 mm inset
/// (16 / 16)** with max 5 Newton iters per step. Synthetic stays at
/// 16 / 16 with cleaner iters than σ = 0.5 (`max 4` vs `max 47`).
/// σ = 0.75 cell is a regression-into-σ-resonance datapoint
/// (iter-1 drops to 69 %); σ = 1.5 cell also reaches full depth but
/// at 9× σ = 0.5's bias. **σ = 1.0 is the empirically-best
/// operating point** — minimum bias of the full-depth-reaching σ
/// values.
///
/// Side effect: surface-position bias ≈ `σ²·κ / 2` where κ is local
/// mean curvature. At σ = 1.0 cell × 3 mm grid = 3 mm physical:
/// bias ≈ 0.11 mm on a 40 mm-radius sphere, ≈ 0.9 mm on 5 mm-radius
/// features. Both inside the Fork-B relative-comparison tolerance
/// (≤ the 4 mm BCC cell budget); the 0.9 mm on sharp features is
/// non-trivial but consistent across design comparisons, so
/// relative quantities (stress, contact pressure ratios) are not
/// affected by it. See `docs/INSERTION_SIM_RECON.md`
/// §"Recon iter 4 results" for the σ sweep + the demoted
/// "Yeoh-validity wall is material side" interpretation.
const GRID_SDF_SMOOTH_SIGMA_CELLS: f64 = 1.0;

/// Separable 3D Gaussian smoothing on a flat `w × h × d` scalar
/// buffer.
///
/// Used by [`build_grid_sdf`] (slice 7.3c) to pre-smooth the
/// signed-distance values so the [`GridSdf`] trilinear interpolant
/// approximates a C¹ function — the contact-side
/// `gradient = ∇φ / ‖∇φ‖` then carries far smaller cell-face
/// direction artifacts than the unsmoothed FD path. See
/// [`GRID_SDF_SMOOTH_SIGMA_CELLS`] for the rationale + measured
/// envelope effect.
///
/// Kernel radius `r = ceil(2σ)` so the dropped tails carry < 0.5 %
/// of total weight. The current default σ = 1.0 cell
/// ([`GRID_SDF_SMOOTH_SIGMA_CELLS`], settled at slice 7.3d) uses a
/// 5-tap kernel (`r = ceil(2·1.0) = 2`, so `2r+1 = 5`); the earlier
/// slice-7.3c σ = 0.5 cell used a 3-tap. Boundary handling is
/// clamp-at-edge — the bbox margin (≥ `cell_size` past the scan
/// outer envelope) keeps the outside well-saturated, so clamp is
/// faithful and adds no sign-flip risk.
///
/// `sigma_cells = 0.0` short-circuits to a copy of the input
/// (identity); negative σ would produce an empty kernel and
/// `debug_assert!`s out.
fn gaussian_smooth_3d_separable(
    field: &[f64],
    w: usize,
    h: usize,
    d: usize,
    sigma_cells: f64,
) -> Vec<f64> {
    debug_assert!(
        sigma_cells.is_finite() && sigma_cells >= 0.0,
        "gaussian_smooth_3d_separable: sigma_cells must be finite and non-negative, got {sigma_cells}",
    );
    if sigma_cells == 0.0 {
        return field.to_vec();
    }
    debug_assert_eq!(
        field.len(),
        w * h * d,
        "gaussian_smooth_3d_separable: field length {} != w*h*d = {}*{}*{}",
        field.len(),
        w,
        h,
        d,
    );

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let r = (2.0 * sigma_cells).ceil() as i32;
    let two_sigma_sq = 2.0 * sigma_cells * sigma_cells;
    let mut kernel: Vec<f64> = (-r..=r)
        .map(|i| (-(f64::from(i)).powi(2) / two_sigma_sq).exp())
        .collect();
    let kernel_sum: f64 = kernel.iter().sum();
    for k in &mut kernel {
        *k /= kernel_sum;
    }
    #[allow(clippy::cast_possible_wrap)]
    let clamp_idx = |i: i32, n: usize| i.max(0).min(n as i32 - 1) as usize;
    let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;

    // X-pass: field → tmp1
    let mut tmp1 = vec![0.0_f64; field.len()];
    for z in 0..d {
        for y in 0..h {
            for x in 0..w {
                let mut acc = 0.0;
                for (ki, &kv) in kernel.iter().enumerate() {
                    #[allow(clippy::cast_possible_wrap)]
                    let dx = ki as i32 - r;
                    #[allow(clippy::cast_possible_wrap)]
                    let xn = clamp_idx(x as i32 + dx, w);
                    acc += kv * field[idx(xn, y, z)];
                }
                tmp1[idx(x, y, z)] = acc;
            }
        }
    }
    // Y-pass: tmp1 → tmp2
    let mut tmp2 = vec![0.0_f64; field.len()];
    for z in 0..d {
        for y in 0..h {
            for x in 0..w {
                let mut acc = 0.0;
                for (ki, &kv) in kernel.iter().enumerate() {
                    #[allow(clippy::cast_possible_wrap)]
                    let dy = ki as i32 - r;
                    #[allow(clippy::cast_possible_wrap)]
                    let yn = clamp_idx(y as i32 + dy, h);
                    acc += kv * tmp1[idx(x, yn, z)];
                }
                tmp2[idx(x, y, z)] = acc;
            }
        }
    }
    // Z-pass: tmp2 → out
    let mut out = vec![0.0_f64; field.len()];
    for z in 0..d {
        for y in 0..h {
            for x in 0..w {
                let mut acc = 0.0;
                for (ki, &kv) in kernel.iter().enumerate() {
                    #[allow(clippy::cast_possible_wrap)]
                    let dz = ki as i32 - r;
                    #[allow(clippy::cast_possible_wrap)]
                    let zn = clamp_idx(z as i32 + dz, d);
                    acc += kv * tmp2[idx(x, y, zn)];
                }
                out[idx(x, y, z)] = acc;
            }
        }
    }
    out
}

/// Build a flood-fill-signed [`GridSdf`] of `scan` over `bbox`.
///
/// Pipeline (post-D.3b): build a parry BVH-backed
/// [`TriMeshDistance`] over `scan`, delegate to mesh-sdf D.2's
/// [`CachedGridSdf::build`] which runs the shared 3-region
/// (Inside / Outside / Wall) flood-fill + multi-source label
/// expansion + `inside_components` health count, walk the lattice
/// once more to sample the cached signed values into a flat row-
/// major `Vec<f64>`, then apply a separable 3D Gaussian pre-smooth
/// (σ = [`GRID_SDF_SMOOTH_SIGMA_CELLS`] = 1.0 cell, settled at slice
/// 7.3d after the σ sweep; slice 7.3c shipped at 0.5 cell) on the
/// signed buffer — see [`GRID_SDF_SMOOTH_SIGMA_CELLS`] for the
/// envelope-extension trail. The Gaussian post-pass stays here
/// rather than inside `CachedGridSdf` because it is FEM-specific
/// (contact-gradient C¹ approximation) and not load-bearing for
/// sign correctness; mesh-sdf primitives are smoothing-free by
/// design.
///
/// `wall_threshold_m` must be `≥ 0.5 * grid_cell_m` so the wall band
/// is 6-connectivity-watertight (a surface crossing between adjacent
/// lattice points always lands one of them within half a cell). The
/// 7.3a fix spike sweeps `grid_cell_m`; `0.75 * grid_cell_m` is the
/// recommended threshold (safe margin without over-thickening the
/// band). Internally the value is converted to the dimensionless
/// `wall_threshold_factor = wall_threshold_m / grid_cell_m` that
/// `CachedGridSdf::build` takes.
///
/// # Errors
///
/// Forwards [`CachedGridSdf::build`] failures with context:
/// non-finite or non-positive `grid_cell_m`
/// ([`mesh_sdf::FloodFillError::NonPositiveCellSize`]), degenerate
/// bbox ([`mesh_sdf::FloodFillError::DegenerateBounds`]), or all
/// eight bbox corners landing within the wall band so no outside
/// seed exists ([`mesh_sdf::FloodFillError::NoOutsideSeed`] —
/// bbox margin too small or grid too coarse). Also forwards
/// [`TriMeshDistance::new`] failure on an empty mesh.
pub fn build_grid_sdf(
    scan: &IndexedMesh,
    bbox: Aabb,
    grid_cell_m: f64,
    wall_threshold_m: f64,
) -> Result<(GridSdf, GridSdfReport)> {
    let t = Instant::now();

    // Build the parry BVH-backed distance source once; mesh-sdf D.2's
    // `CachedGridSdf::build` runs the shared 3-region flood-fill,
    // multi-source label expansion, and `inside_components` count over
    // it. The shared core preserves the same Region labels + corner
    // seeds + BFS order the bespoke copy used, so signs are
    // bit-equivalent on every well-formed input.
    let distance = TriMeshDistance::new(scan.clone())
        .context("build TriMeshDistance from the scan for flood-fill GridSdf")?;
    let factor = wall_threshold_m / grid_cell_m;
    let (cached, ff_report) = CachedGridSdf::build(&distance, bbox, grid_cell_m, factor)
        .with_context(|| {
            format!(
                "build CachedGridSdf (grid_cell_m={grid_cell_m} m, \
                 wall_threshold_m={wall_threshold_m} m) — bbox margin too small or grid too \
                 coarse to seed the outside flood"
            )
        })?;
    let [w, h, d] = ff_report.dims;
    let n = w * h * d;
    let origin = bbox.min;

    // Sample the cached signed grid back into a flat row-major
    // `Vec<f64>` aligned with the existing post-processing pipeline.
    // CachedGridSdf's internal lattice has identical dims + origin +
    // cell_size, so trilinear sampling at each world point collapses
    // to the stored cell value within f64 ulps.
    #[allow(clippy::cast_precision_loss)] // lattice index → world coord
    let world = |x: usize, y: usize, z: usize| {
        Point3::new(
            origin.x + x as f64 * grid_cell_m,
            origin.y + y as f64 * grid_cell_m,
            origin.z + z as f64 * grid_cell_m,
        )
    };
    let flat = |x: usize, y: usize, z: usize| z * w * h + y * w + x;
    let mut signed = vec![0.0_f64; n];
    for z in 0..d {
        for y in 0..h {
            for x in 0..w {
                signed[flat(x, y, z)] = cached.signed_distance(world(x, y, z));
            }
        }
    }

    // Slice 7.3c–7.3d — separable 3D Gaussian pre-smooth on the
    // signed buffer so the trilinear interpolant inside `SdfGrid`
    // approximates a C¹ function. σ = 1.0 cell (recon-iter-4)
    // delivers the full 3 mm inset on both the synthetic-icosphere
    // and iter-1 cleaned-scan ramps; σ = 0.5 cell (slice 7.3c)
    // left iter-1 at 75 % depth on what looked like a Yeoh-validity
    // wall but was actually under-bandwidth smoothing feeding sharp
    // gradients into specific tets. See `GRID_SDF_SMOOTH_SIGMA_CELLS`
    // for the σ-sweep trail + bias.
    //
    // Applied externally (not folded into `CachedGridSdf`) so the
    // mesh-sdf primitive stays smoothing-free — Gaussian smoothing
    // is FEM-specific (contact-gradient C¹ approximation) and not
    // load-bearing for sign correctness.
    let smoothed = gaussian_smooth_3d_separable(&signed, w, h, d, GRID_SDF_SMOOTH_SIGMA_CELLS);
    let grid = SdfGrid::new(smoothed, w, h, d, grid_cell_m, origin);
    let report = GridSdfReport {
        dims: ff_report.dims,
        grid_cell_m,
        n_outside: ff_report.outside_cells,
        n_inside: ff_report.inside_cells,
        n_wall: ff_report.wall_cells,
        inside_components: ff_report.inside_components,
        build_ms: elapsed_ms(t),
    };
    Ok((GridSdf { grid }, report))
}

// ── 7.3b — quasi-static insertion ramp ─────────────────────────────

/// One converged step of a quasi-static insertion ramp.
///
/// Slice 7.3b.2 added [`x_final`](Self::x_final) and
/// [`readout`](Self::readout) so each step carries the engineering data
/// the layer-engineering tool will consume: the converged positions
/// (so per-tet detail is derivable on demand via
/// [`compute_tet_readouts`]) and pre-aggregated scalar metrics (contact
/// force, principal-stretch extrema, peak stress, mean strain energy).
/// Intentionally no `Debug` derive — `x_final` is a flat `Vec<f64>` of
/// `3 * n_vertices`, same `dbg!`-footgun rationale as [`InsertionRamp`].
#[derive(Clone)]
pub struct RampStep {
    /// Press-fit interference (m) the intruder was seated to at this
    /// step — `(k + 1) / n_steps * cavity_inset_m`.
    pub interference_m: f64,
    /// Newton iterations this step's solve took.
    pub iter_count: usize,
    /// Free-DOF residual norm at this step's convergence.
    pub final_residual_norm: f64,
    /// Slice 7.3b.2 — converged vertex positions at this step,
    /// vertex-major xyz (length `3 * n_vertices`). Lets a caller
    /// reconstruct per-tet F at *any* step on demand via
    /// [`compute_tet_readouts`].
    pub x_final: Vec<f64>,
    /// Slice 7.3b.2 — pre-aggregated engineering scalars for this step
    /// (F-d curve ordinate, principal-stretch extrema, peak stress,
    /// mean strain-energy density). Derived once when the step
    /// converges.
    pub readout: StepReadout,
}

/// Scalar per-step engineering aggregates — slice 7.3b.2's per-step
/// summary of the per-tet stress / stretch field and the contact
/// reaction. Stored on each converged [`RampStep`] and assembled into
/// the [`force_displacement_curve`](InsertionResult::force_displacement_curve)
/// on [`InsertionResult`].
///
/// Per-tet detail at a specific step is derivable from
/// [`RampStep::x_final`] via [`compute_tet_readouts`]; pre-aggregating
/// the scalar headlines keeps the F-d curve + Yeoh-validity sentinels
/// cheap to inspect without re-walking every tet.
#[derive(Debug, Clone)]
pub struct StepReadout {
    /// Orphan-filtered active contact pairs at this step — the count
    /// surviving [`filter_pair_readouts_to_referenced`] (drops BCC
    /// lattice corners not in any tet, per row 22 pattern (xx)).
    pub n_active_contact_pairs: usize,
    /// Vector sum of `force_on_soft` over orphan-filtered active
    /// pairs (N) — Newton's-3rd-law: this is the soft body's net
    /// resistance to the intruder seating, with sign opposite the
    /// intruder's penetration direction.
    pub contact_force_total_n: Vec3,
    /// Magnitude of [`contact_force_total_n`](Self::contact_force_total_n)
    /// — the natural F-d-curve ordinate.
    pub contact_force_magnitude_n: f64,
    /// Maximum singular value of `F` across every tet at this step
    /// (peak principal stretch). Tracks proximity to the Yeoh
    /// `validity.max_principal_stretch` cap; a step approaching it is
    /// the warning the layer is past its calibrated stretch range.
    pub max_principal_stretch: f64,
    /// Minimum singular value of `F` across every tet (peak
    /// compressive principal stretch). Mirrors
    /// [`max_principal_stretch`](Self::max_principal_stretch) for the
    /// `validity.min_principal_stretch` cap.
    pub min_principal_stretch: f64,
    /// Maximum Frobenius norm of first-Piola stress (Pa) across every
    /// tet — the peak stress hotspot magnitude.
    pub max_first_piola_frobenius_pa: f64,
    /// Mean strain-energy density (J/m³) across every tet — the
    /// per-step "how strained" scalar; integrating `× tet_volume`
    /// recovers total elastic energy (volumes are in the mesh's
    /// `QualityMetrics::signed_volume`, not aggregated here).
    pub mean_strain_energy_density_j_per_m3: f64,
}

/// Per-tet engineering readout at a single step — slice 7.3b.2's
/// per-tet detail surface.
///
/// Reconstructed by [`compute_tet_readouts`] from rest positions,
/// current positions, tet connectivity, and per-tet [`Yeoh`] materials
/// (the four pieces stored in the mesh that `run_insertion_ramp`
/// snapshots before consuming `geometry`). The UI layer-heat-map
/// (slice 7.4) consumes this directly; the per-step aggregates in
/// [`StepReadout`] are reductions over these.
#[derive(Debug, Clone)]
pub struct TetReadout {
    /// Deformation gradient `F = D_curr · D_rest⁻¹`, where each `D` is
    /// the 3×3 column matrix `[v1−v0, v2−v0, v3−v0]` for the tet's
    /// four vertices. The Yeoh element evaluates energy + stress at
    /// `F`; SVD of `F` gives the principal stretches.
    pub f: Matrix3<f64>,
    /// First Piola stress `P = Yeoh::first_piola(F)` (Pa) — the
    /// material-frame stress conjugate to `F`.
    pub first_piola: Matrix3<f64>,
    /// Frobenius norm of [`first_piola`](Self::first_piola) (Pa) —
    /// the scalar hotspot intensity.
    pub first_piola_frobenius_pa: f64,
    /// Strain-energy density `Ψ = Yeoh::energy(F)` (J/m³).
    pub energy_density_j_per_m3: f64,
    /// Principal stretches — the three singular values of `F` from
    /// `f.svd_unordered(false, false).singular_values`. Order is the
    /// unordered SVD's (NOT sorted); see
    /// `sim/L0/soft/src/solver/backward_euler.rs:613` for the
    /// algorithm-shared canonical call.
    pub principal_stretches: Vector3<f64>,
}

/// Per-tet engineering readouts at the **final** converged step of a
/// quasi-static ramp, plus the ramp's force-displacement curve. Slice
/// 7.3b.2's `InsertionResult` per `docs/INSERTION_SIM_STATE.md` §"What
/// 7.3b.2 and 7.3b.3 look like after the recon".
///
/// `final_per_tet` is the heat-map data for the deepest seating the
/// ramp reached (the most interesting state for a layer-engineering
/// review). Per-step per-tet detail is derivable on demand via
/// [`compute_tet_readouts`] using [`RampStep::x_final`]; pre-computing
/// it for every step would blow up memory (≈ 184 bytes × n_tets ×
/// n_steps, well into hundreds of MB at production cell sizes), and
/// the UI consumes one step's detail at a time anyway.
///
/// Intentionally no `Debug` derive: `final_per_tet` is a `Vec<TetReadout>`
/// at `O(n_tets)` — printing it in test failures or via `dbg!` is the
/// same footgun [`InsertionGeometry`] and [`InsertionRamp`] dodge by
/// omitting `Debug`.
#[derive(Clone)]
pub struct InsertionResult {
    /// Per-tet readouts at the final converged step. Length
    /// `geometry.n_tets`; indexed by [`TetId`] matching the ramp
    /// mesh's tet ordering.
    pub final_per_tet: Vec<TetReadout>,
    /// Force-displacement curve over the ramp:
    /// `(interference_m, contact_force_magnitude_n)` pairs, one per
    /// converged step in ramp order. Length equals
    /// [`InsertionRamp::steps`]`.len()`; values come straight from
    /// [`StepReadout::contact_force_magnitude_n`].
    pub force_displacement_curve: Vec<(f64, f64)>,
}

/// Result of a quasi-static insertion ramp — see [`run_insertion_ramp`].
///
/// Intentionally no `Debug` derive: `final_x` is a flat `Vec<f64>` of
/// `3 * n_vertices` — the same `dbg!`-footgun rationale as
/// [`InsertionGeometry`]. The per-step [`RampStep`]s + `failed_at_step`
/// + [`result`](Self::result) are the inspectable summary.
pub struct InsertionRamp {
    /// Per-step records in ramp order — only the *converged* steps, so
    /// `steps.len()` is how many steps converged.
    pub steps: Vec<RampStep>,
    /// `Some(k)` if step `k` failed to converge — the solver hit a
    /// non-SPD tangent / Armijo stall and `replay_step` panicked; the
    /// ramp stopped there and `steps` holds `0..k`. `None` if every
    /// requested step converged.
    pub failed_at_step: Option<usize>,
    /// The solver's panic message for the failed step — the "why"
    /// behind `failed_at_step` (Armijo stall at residual X, or the
    /// Newton-iteration cap). `None` if every step converged. Picking
    /// the convergence lever (tol / kappa / more steps) needs this.
    pub failure_reason: Option<String>,
    /// Deformed vertex positions (vertex-major xyz) at the last
    /// converged step — the chained `x_final`; the rest positions if
    /// step 0 itself failed. Mirrors [`RampStep::x_final`] of
    /// [`steps`](Self::steps)`.last()` when at least one step
    /// converged; kept as a top-level field for callers that only
    /// need the final state.
    pub final_x: Vec<f64>,
    /// Dirichlet-pinned outer-skin vertex count — constant across the
    /// ramp (the outer skin does not move).
    pub n_pinned: usize,
    /// Slice 7.3b.2 — per-tet engineering readouts at the final
    /// converged step + the ramp-wide force-displacement curve.
    /// `None` if no step converged (the ramp panicked at step 0 and
    /// there is no deformed state to report).
    pub result: Option<InsertionResult>,
}

/// Reconstruct the per-tet deformation gradient `F = D_curr · D_rest⁻¹`
/// from one tet's vertex indices, the rest positions, and the current
/// positions. The same construction
/// `sim/L0/soft/src/solver/backward_euler.rs::solve_impl` and the row
/// 22 / 23 layered-sleeve examples use: each `D` is the 3×3 matrix
/// whose columns are the three edge vectors from vertex 0 to vertices
/// 1, 2, 3.
///
/// # Panics
///
/// `D_rest.try_inverse()` returns `None` for a degenerate (zero-volume)
/// rest tet. Production meshers (BCC + Isosurface Stuffing) reject
/// degenerate tets before assembly via the signed-volume gate; a
/// `None` here would mean the mesh was constructed inconsistent with
/// its `QualityMetrics::signed_volume`, a construction-side contract
/// violation worth surfacing fail-closed (same posture as the row 23
/// fixture's `.expect`, mirroring the solver's validity gate at
/// `sim/L0/soft/src/solver/backward_euler.rs:585`).
//
// `clippy::panic` is denied crate-wide, but a degenerate `D_rest`
// indicates the mesh constructor skipped its signed-volume gate — a
// construction-side bug, not a runtime data dependence. The local
// `#[allow]` is the documented "this is an upstream-invariant
// violation, surface it loudly" carve-out (matches sim-soft's
// internal pattern for invariant-violation assertions).
#[allow(clippy::panic)]
fn deformation_gradient(verts: [VertexId; 4], rest: &[Vec3], curr: &[Vec3]) -> Matrix3<f64> {
    let r0 = rest[verts[0] as usize];
    let r1 = rest[verts[1] as usize];
    let r2 = rest[verts[2] as usize];
    let r3 = rest[verts[3] as usize];
    let c0 = curr[verts[0] as usize];
    let c1 = curr[verts[1] as usize];
    let c2 = curr[verts[2] as usize];
    let c3 = curr[verts[3] as usize];
    let d_rest = Matrix3::from_columns(&[r1 - r0, r2 - r0, r3 - r0]);
    let d_curr = Matrix3::from_columns(&[c1 - c0, c2 - c0, c3 - c0]);
    let d_rest_inv = d_rest.try_inverse().unwrap_or_else(|| {
        panic!(
            "tet rest configuration is degenerate (D_rest non-invertible) — \
             the mesh constructor's signed-volume gate should have rejected it"
        )
    });
    d_curr * d_rest_inv
}

/// Compute one tet's [`TetReadout`] from its vertex indices, the rest
/// and current positions, and its [`Yeoh`] material.
///
/// Pure: no IO, no allocation beyond the `Matrix3` SVD's internal
/// buffer. Three calls into the Yeoh material's [`Material`] surface
/// (`first_piola`, `energy`) plus one SVD of `F` (`O(27)` FLOPs, cheap).
fn tet_readout(verts: [VertexId; 4], rest: &[Vec3], curr: &[Vec3], material: &Yeoh) -> TetReadout {
    let f = deformation_gradient(verts, rest, curr);
    let first_piola = material.first_piola(&f);
    let first_piola_frobenius_pa = first_piola.norm();
    let energy_density_j_per_m3 = material.energy(&f);
    // Canonical sim-soft SVD-for-principal-stretches call, mirroring
    // `sim/L0/soft/src/solver/backward_euler.rs:613` so a future
    // re-validation of stretches against the solver's validity gate
    // is bit-for-bit comparable.
    let principal_stretches = f.svd_unordered(false, false).singular_values;
    TetReadout {
        f,
        first_piola,
        first_piola_frobenius_pa,
        energy_density_j_per_m3,
        principal_stretches,
    }
}

/// Compute per-tet readouts ([`TetReadout`]) for every tet given the
/// rest positions, current positions, tet connectivity, and per-tet
/// [`Yeoh`] materials.
///
/// The public free helper: callers re-derive per-step per-tet detail
/// from [`RampStep::x_final`] without going back through the ramp's
/// consumed `geometry`. The four input slices are exactly the data
/// the ramp snapshots before consuming `InsertionGeometry`.
///
/// # Panics
///
/// `materials.len()` must equal `tets.len()` (each tet has its own
/// Yeoh material per
/// [`Mesh::materials`]); mismatched lengths panic via slice indexing.
/// `tets[t][i] as usize` must be in bounds for both `rest` and `curr`;
/// out-of-range vertex IDs panic via slice indexing.
#[must_use]
pub fn compute_tet_readouts(
    rest: &[Vec3],
    curr: &[Vec3],
    tets: &[[VertexId; 4]],
    materials: &[Yeoh],
) -> Vec<TetReadout> {
    assert_eq!(
        tets.len(),
        materials.len(),
        "compute_tet_readouts: tets.len() = {} must match materials.len() = {} \
         (one Yeoh material per tet per Mesh::materials)",
        tets.len(),
        materials.len(),
    );
    tets.iter()
        .zip(materials.iter())
        .map(|(&verts, mat)| tet_readout(verts, rest, curr, mat))
        .collect()
}

/// Reduce per-tet readouts + orphan-filtered contact-pair readouts to
/// the scalar [`StepReadout`] aggregates a single ramp step records.
fn aggregate_step_readout(
    per_tet: &[TetReadout],
    contact_readouts: &[ContactPairReadout],
) -> StepReadout {
    let n_active_contact_pairs = contact_readouts.len();
    let contact_force_total_n: Vec3 = contact_readouts
        .iter()
        .map(|r| r.force_on_soft)
        .fold(Vec3::zeros(), |a, b| a + b);
    let contact_force_magnitude_n = contact_force_total_n.norm();

    let mut max_principal_stretch = f64::NEG_INFINITY;
    let mut min_principal_stretch = f64::INFINITY;
    let mut max_first_piola_frobenius_pa = 0.0_f64;
    let mut sum_energy = 0.0_f64;
    for t in per_tet {
        for &s in t.principal_stretches.iter() {
            if s > max_principal_stretch {
                max_principal_stretch = s;
            }
            if s < min_principal_stretch {
                min_principal_stretch = s;
            }
        }
        if t.first_piola_frobenius_pa > max_first_piola_frobenius_pa {
            max_first_piola_frobenius_pa = t.first_piola_frobenius_pa;
        }
        sum_energy += t.energy_density_j_per_m3;
    }
    let mean_strain_energy_density_j_per_m3 = if per_tet.is_empty() {
        0.0
    } else {
        // `per_tet.len()` here is the mesh's `n_tets`, bounded well
        // under `f64`'s exact-integer ceiling (Phase 4 BCC grids cap
        // in the millions, far under 2^53).
        #[allow(clippy::cast_precision_loss)]
        let n = per_tet.len() as f64;
        sum_energy / n
    };
    // An empty `per_tet` (degenerate geometry with zero tets) makes
    // the stretch sentinels meaningless; collapse to zero so the
    // readout is still finite-valued.
    if per_tet.is_empty() {
        max_principal_stretch = 0.0;
        min_principal_stretch = 0.0;
    }

    StepReadout {
        n_active_contact_pairs,
        contact_force_total_n,
        contact_force_magnitude_n,
        max_principal_stretch,
        min_principal_stretch,
        max_first_piola_frobenius_pa,
        mean_strain_energy_density_j_per_m3,
    }
}

/// Convert a flat `Vec<f64>` vertex-major xyz into the `Vec<Vec3>` slice
/// view of vertex positions that `compute_tet_readouts` and
/// `PenaltyRigidContact::per_pair_readout` take. The
/// `solver.replay_step` API hands back the flat form; the readout
/// helpers prefer the `Vec3` form.
fn positions_from_flat(flat: &[f64]) -> Vec<Vec3> {
    flat.chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect()
}

/// Extract a human-readable message from a `catch_unwind` panic
/// payload — panics from `panic!` / `assert!` carry a `String` or a
/// `&'static str`.
fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_string()))
        .unwrap_or_else(|| "<non-string panic payload>".to_string())
}

/// Format a [`SolverFailure`] into the same `failure_reason` shape the
/// pre-F3.4 catch_unwind pattern produced via [`panic_message`] — so
/// the SL.4 viewport's "stalled at step N: <reason>" surface reads the
/// same regardless of whether the failure came through `try_replay_step`
/// (F3.4 Fork-B path) or, for the still-panicking
/// `run_insertion_ramp` (growing-intruder) path, `catch_unwind`.
///
/// MAINTENANCE NOTE — the per-variant field extraction here mirrors the
/// inline match arms in `run_single_insertion_step` (which wrap the
/// same variants in `anyhow::Error` for its `Result` return).
/// Intentionally separate surfaces (viewport string vs CLI anyhow), so
/// the prefix wording diverges by design — but if a future
/// `SolverFailure` variant lands, update BOTH sites.
fn solver_failure_message(failure: &SolverFailure) -> String {
    match failure {
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => format!(
            "Armijo line-search stalled at Newton iter {last_iter}, r_norm {last_r_norm:.3e}"
        ),
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => format!(
            "Newton iter cap {max_iter} reached without convergence, \
             last r_norm {last_r_norm:.3e}"
        ),
        SolverFailure::DoublyFailedFactor {
            last_iter, context, ..
        } => format!("doubly-failed factor at Newton iter {last_iter}: {context}"),
    }
}

/// Run a quasi-static insertion ramp: seat the scan-derived intruder
/// into the device cavity in `n_steps` equal interference increments
/// (`0 → cavity_inset_m`), each step's Newton solve warm-started from
/// the previous step's converged `x_final`.
///
/// Warm-starting is the answer to the 7.2 / 7.3a finding that a
/// *single* static step to a meaningful interference stalls near the
/// solution on a non-SPD tangent: each ramp increment is small enough
/// to stay in the convergeable regime, and chaining `x_final` keeps
/// every step close to its solution.
///
/// The rest geometry + per-tet materials are constant across the ramp
/// — only the intruder moves — so the prebuilt `geometry.mesh` and the
/// outer-skin BCs are *cloned* per step rather than re-meshed (faster
/// than the rows-21–25 re-mesh-per-step precedent, and bit-identical).
/// `replay_step` panics on non-convergence; each step is wrapped in
/// `catch_unwind`, so a stall is reported as `failed_at_step` rather
/// than aborting — the ramp returns the steps that *did* converge.
///
/// Consumes `geometry` (the mesh is cloned per step; the original
/// drops at the end). Slice 7.3b.2 added per-step engineering readouts
/// and a final-step per-tet [`InsertionResult`]: each [`RampStep`]
/// now carries the converged `x_final` and a scalar [`StepReadout`]
/// (orphan-filtered contact-force sum, principal-stretch extrema,
/// peak stress, mean strain energy), and [`InsertionRamp::result`]
/// holds the final-step per-tet detail and the force-displacement
/// curve.
///
/// # Errors
///
/// - `n_steps` is zero;
/// - [`outer_skin_bc`] fails (no outer-skin vertex in the pin-band).
pub fn run_insertion_ramp(geometry: InsertionGeometry, n_steps: usize) -> Result<InsertionRamp> {
    if n_steps == 0 {
        return Err(anyhow!("insertion ramp needs at least one step"));
    }

    let InsertionGeometry {
        mesh,
        intruder,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets,
        per_tet_layer: _,
    } = geometry;

    let n_vertices = mesh.n_vertices();
    let n_dof = 3 * n_vertices;

    // BCs are constant across the ramp (the outer skin does not move)
    // — build once, clone per step.
    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot per-tet immutables before consuming the mesh into the
    // per-step solver clones: rest positions, tet connectivity, per-tet
    // Yeoh materials, and the referenced-vertex set for orphan
    // filtering (`SdfMeshedTetMesh` retains BCC lattice corners not
    // referenced by any tet — see `referenced_vertices` docs). The
    // ramp builds per-step readouts from these without needing the
    // mesh after the loop ends. Materials are `Yeoh: Clone`.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    // `tet_id as TetId` is a `u32` cap; Phase 4 meshes stay well under
    // `u32::MAX` per `Mesh` trait docs.
    #[allow(clippy::cast_possible_truncation)]
    let tets: Vec<[VertexId; 4]> = (0..n_tets as TetId).map(|t| mesh.tet_vertices(t)).collect();
    let materials: Vec<Yeoh> = mesh.materials().to_vec();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);

    // Full press-fit interference = the cavity inset; the ramp seats
    // the intruder there in `n_steps` equal increments.
    let inset_m = -cavity_offset_m;

    let config = insertion_solver_config();

    // x_prev starts at rest; each converged step chains its x_final in.
    let mut x_prev_flat: Vec<f64> = rest_positions
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    let v_prev = Tensor::zeros(&[n_dof]);
    let empty_theta: [f64; 0] = [];
    let theta = Tensor::from_slice(&empty_theta, &[0]);

    let mut steps: Vec<RampStep> = Vec::with_capacity(n_steps);
    let mut failed_at_step = None;
    let mut failure_reason = None;
    for k in 0..n_steps {
        // k + 1 and n_steps are tiny — well under f64's exact-integer ceiling.
        #[allow(clippy::cast_precision_loss)]
        let interference_m = (k + 1) as f64 / n_steps as f64 * inset_m;
        let contact = intruder_contact_at(&intruder, bounds, interference_m, cavity_offset_m);
        let solver = CpuNewtonSolver::new(Tet4, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        // `replay_step` panics on non-convergence — catch it so the
        // ramp records `failed_at_step` + the panic's reason instead
        // of aborting.
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        match outcome {
            Ok(step) => {
                // Per-step readout (slice 7.3b.2). The solver consumed
                // the step-`k` `contact`; rebuild a fresh one with the
                // same parameters for `per_pair_readout` at the
                // converged positions. `PenaltyRigidContact` is not
                // `Clone` (its `Vec<Box<dyn Sdf>>` rules it out), so
                // double-build is the row 23 precedent
                // ("inspection_contact" in `scan-fit-3layer-sleeve-
                // yeoh-ramp`).
                let positions_k: Vec<Vec3> = positions_from_flat(&step.x_final);
                let readout_contact =
                    intruder_contact_at(&intruder, bounds, interference_m, cavity_offset_m);
                let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                let contact_readouts =
                    filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                let per_tet =
                    compute_tet_readouts(&rest_positions, &positions_k, &tets, &materials);
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts);

                steps.push(RampStep {
                    interference_m,
                    iter_count: step.iter_count,
                    final_residual_norm: step.final_residual_norm,
                    x_final: step.x_final.clone(),
                    readout: step_readout,
                });
                x_prev_flat = step.x_final; // chain the warm start
            }
            Err(payload) => {
                failed_at_step = Some(k);
                failure_reason = Some(panic_message(&*payload));
                break;
            }
        }
    }

    // Slice 7.3b.2 — final-step per-tet detail + ramp force-displacement
    // curve. `result = None` only when no step converged; otherwise the
    // final converged step's positions drive the per-tet readout.
    let result = steps.last().map(|last| {
        let final_positions = positions_from_flat(&last.x_final);
        let final_per_tet =
            compute_tet_readouts(&rest_positions, &final_positions, &tets, &materials);
        let force_displacement_curve = steps
            .iter()
            .map(|s| (s.interference_m, s.readout.contact_force_magnitude_n))
            .collect();
        InsertionResult {
            final_per_tet,
            force_displacement_curve,
        }
    });

    Ok(InsertionRamp {
        steps,
        failed_at_step,
        failure_reason,
        final_x: x_prev_flat,
        n_pinned,
        result,
    })
}

// ───────────────────────────────────────────────────────────────────────
// SL.1 — sliding-intruder scaffolding
// ───────────────────────────────────────────────────────────────────────
//
// Helpers consumed at SL.2 by `run_sliding_insertion_ramp`. The
// module-level `#![allow(dead_code)]` already covers the unused-
// warning until that ramp lands; section header surfaces the intent.
//
// Cross-references: `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §3b
// (TransformedSdf), §3c (slide_pose_at), + decision D-Slide2
// (local copies of the polyline helpers — no tools→tools dep on
// cf-scan-prep; sources at `tools/cf-scan-prep/src/main.rs:1016` +
// `:1060`).

/// Adapter: present a static [`Sdf`] as if it had been rigidly
/// transformed by `transform` in world space.
///
/// For a rigid transform `T: p ↦ R p + t`:
/// - `eval(q) = inner.eval(T⁻¹ q)` (signed distance is rigid-invariant);
/// - `grad(q) = R · inner.grad(T⁻¹ q)` (the gradient rotates with `T`,
///   recovering the world-frame outward normal of the moved surface).
///
/// For translation-only (`R = I`), `grad` collapses to a query-point
/// shift + pass-through gradient.
///
/// Designed for sliding-intruder contact: an immutable [`GridSdf`] of
/// the cleaned scan is wrapped per slide step at a fresh `slide_pose`
/// (from [`slide_pose_at`]) and handed to [`PenaltyRigidContact`]
/// (`docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §3a). The
/// `Isometry3<f64>` parameter shape absorbs the iter-2 rotation
/// followup without API churn even though iter-1 only uses
/// translation.
///
/// `T⁻¹` is cached at construction — eval/grad are called inside the
/// contact-pair inner loop ([`PenaltyRigidContact::active_pairs`])
/// where even scalar-cheap recomputation matters at full BCC-mesh
/// scale.
///
/// **Grid-coverage caveat**: `inner` may be backed by a finite-extent
/// [`GridSdf`] with `distance_clamped` semantics outside the grid.
/// After the inverse transform, queries falling outside the grid are
/// silently clamped to the nearest grid sample. For sliding-intruder
/// use this is benign — the static intruder grid covers the body
/// bbox + outer margin, and active contact pairs (`sd < d̂ = 1 mm`)
/// fire only when the moved intruder surface is within 1 mm of the
/// body wall, putting the inverse-transformed query well inside the
/// grid. Inactive pairs report large `sd` and are skipped by
/// [`PenaltyRigidContact::active_pairs`].
#[derive(Clone)]
pub(crate) struct TransformedSdf<S: Sdf> {
    inner: S,
    transform: Isometry3<f64>,
    inverse: Isometry3<f64>,
}

impl<S: Sdf> TransformedSdf<S> {
    pub(crate) fn new(inner: S, transform: Isometry3<f64>) -> Self {
        let inverse = transform.inverse();
        Self {
            inner,
            transform,
            inverse,
        }
    }
}

impl<S: Sdf> Sdf for TransformedSdf<S> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.inner.eval(self.inverse * p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.transform.rotation * self.inner.grad(self.inverse * p)
    }
}

/// Walk along `polyline`'s arc and return the position + local tangent
/// at arc-length `distance_m` from `polyline[0]`. The tangent is the
/// unit direction of the segment containing the target point, pointing
/// from `polyline[0]` toward `polyline.last()`.
///
/// Local copy of `tools/cf-scan-prep/src/main.rs:1016` per D-Slide2
/// (no `tools/` → `tools/` dependency).
///
/// Returns `None` for a `< 2`-point polyline. For `distance_m` past the
/// polyline's total arc length, returns a linear extrapolation along
/// the last segment's tangent.
pub(crate) fn point_along_polyline_at_arc_distance(
    polyline: &[Point3<f64>],
    distance_m: f64,
) -> Option<(Point3<f64>, Vector3<f64>)> {
    if polyline.len() < 2 {
        return None;
    }
    let target_m = distance_m.max(0.0);
    let mut walked_m = 0.0_f64;
    for i in 0..polyline.len() - 1 {
        let seg_vec = polyline[i + 1].coords - polyline[i].coords;
        let seg_len = seg_vec.norm();
        if seg_len < f64::EPSILON {
            continue;
        }
        if walked_m + seg_len >= target_m {
            let t = ((target_m - walked_m) / seg_len).clamp(0.0, 1.0);
            let pos = Point3::from(polyline[i].coords + t * seg_vec);
            let tangent = seg_vec / seg_len;
            return Some((pos, tangent));
        }
        walked_m += seg_len;
    }
    let n = polyline.len();
    let last_seg = polyline[n - 1].coords - polyline[n - 2].coords;
    let last_seg_len = last_seg.norm();
    if last_seg_len < f64::EPSILON {
        return None;
    }
    let overrun_m = target_m - walked_m;
    let tangent = last_seg / last_seg_len;
    let pos = Point3::from(polyline[n - 1].coords + tangent * overrun_m);
    Some((pos, tangent))
}

/// Total arc length of `polyline` in meters. Returns `0.0` for `< 2`
/// points.
///
/// Local copy of `tools/cf-scan-prep/src/main.rs:1060` per D-Slide2.
pub(crate) fn polyline_arc_length_m(polyline: &[Point3<f64>]) -> f64 {
    polyline
        .windows(2)
        .map(|w| (w[1].coords - w[0].coords).norm())
        .sum()
}

/// Compute the rigid transform for the intruder at slide fraction
/// `t ∈ [0, 1]` along the cleaned-scan centerline polyline.
///
/// Centerline ordering convention (cf-scan-prep
/// `compute_centerline_polyline` + `trim_mesh_along_centerline` docs):
/// `centerline[0]` is the TIP (closed end / dome apex);
/// `centerline.last()` is the FLOOR (open end / cap rim).
///
/// - `t = 1`: tip at rest (identity transform).
/// - `t = 0`: tip translated to `centerline.last()` (cap mouth); body
///   extends arc-length past the cap into the air outside.
/// - `t ∈ (0, 1)`: tip walks backward from rest by arc-length
///   `L · (1 − t)`.
///
/// Translation-only iter-1 (D-Slide2). The `Isometry3<f64>` return
/// type absorbs the banked iter-2 rotation followup without API
/// churn. For a degenerate polyline (< 2 points or zero arc length),
/// returns identity — defensive; sliding-ramp drivers validate
/// polyline shape up front and surface their own error.
pub(crate) fn slide_pose_at(centerline: &[Point3<f64>], t: f64) -> Isometry3<f64> {
    if centerline.len() < 2 {
        return Isometry3::identity();
    }
    let l_m = polyline_arc_length_m(centerline);
    let tip_rest = centerline[0];
    let walk_from_tip = (l_m * (1.0 - t.clamp(0.0, 1.0))).max(0.0);
    let (tip_world, _tangent) = point_along_polyline_at_arc_distance(centerline, walk_from_tip)
        .unwrap_or((tip_rest, Vector3::z()));
    let translation = tip_world.coords - tip_rest.coords;
    Isometry3::translation(translation.x, translation.y, translation.z)
}

// ───────────────────────────────────────────────────────────────────────
// SL.2 — sliding-intruder ramp solver
// ───────────────────────────────────────────────────────────────────────
//
// Per `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §3a + §3d. Mirrors
// [`run_insertion_ramp`] (growing-intruder, preserved per Q1) but
// with a rigid intruder of constant geometry translated along the
// centerline arc — the contact primitive at each step is the
// transformed scan offset inward by `cavity_offset_m`, NOT the
// scan grown outward by interference. The per-step `x_final` shape
// + `StepReadout` aggregates are byte-identical to the growing
// ramp so the existing render plumbing (`cavity_boundary_faces`,
// `per_layer_outer_faces`, `deformed_layer_slab_mesh_at`) consumes
// the sliding output unchanged at SL.3.

/// Default per-step slide distance (m) — `5 mm` per the spec
/// (D-Slide5). Larger gives a visibly-propagating deformation wave
/// across playback steps; smaller gives a continuous-looking wave at
/// the cost of solver iters. Programmer-set, not user-tunable —
/// same posture as [`DEFAULT_N_STEPS`] (the growing-ramp counterpart
/// is a fixed `n_steps`, not a step size).
pub const DEFAULT_SLIDE_STEP_SIZE_M: f64 = 5.0e-3;

/// Build the sliding-intruder contact primitive at a given slide pose
/// and engineered interference.
///
/// The contact SDF is `Solid::from_sdf(TransformedSdf(intruder,
/// slide_pose), bounds).offset(interference_m + cavity_offset_m)` —
/// the rigid-translated scan, offset by `interference_m +
/// cavity_offset_m`. With `cavity_offset_m = -design.cavity_inset_m`:
/// - `interference_m = 0` → composed offset = `-cavity_inset_m` →
///   transformed scan SHRUNK by `cavity_inset_m` (pre-F4 sliding
///   model; the intruder coincides with the cavity wall at rest pose,
///   and local interference at non-rest poses comes purely from the
///   pose-induced geometric mismatch).
/// - `interference_m = cavity_inset_m` → composed offset = `0` → bare
///   transformed scan (full engineered interference uniformly applied:
///   the body geometry overlaps the un-deformed cavity by
///   `cavity_inset_m` everywhere the surfaces are in contact).
///
/// `interference_m` is the F4 homotopy parameter
/// (`docs/CAVITY_INSET_STALL_BOOKMARK.md` §9-§10): [`run_sliding_insertion_ramp`]
/// ramps it `0 → cavity_inset_m` across K warmup substeps to ease the
/// solver into the full engineered-interference contact at step 0.
/// Single-step callers (the wire-up tests) pass `interference_m = 0`
/// to reproduce the pre-F4 shrunk-scan model bit-equal — analogous to
/// growing-mode [`intruder_contact_at`]'s `interference_m + cavity_offset_m`
/// composition where `interference_m = 0` sits flush with the cavity
/// wall and `interference_m = cavity_inset_m` reproduces the bare scan.
///
/// Penalty `(κ, d̂)` reuses [`INSERTION_CONTACT_KAPPA`] +
/// [`INSERTION_CONTACT_DHAT`] from the growing ramp; the kappa
/// tradeoff (7.3b.1 wider convergeable envelope at slight cost in
/// residual penetration) is independent of the contact model.
///
/// `cavity_inset_m` (positive — same value as `design.cavity_inset_m`)
/// is the engineered shell-wall interference. The contact model passes
/// `2 × cavity_inset_m` as the
/// [`PenaltyRigidContact::with_params_and_smoothing_and_interior_cutoff`]
/// cutoff — one design margin past the engineered interference.
/// Physical contact pairs (composed sd ∈ [-cavity_inset_m, d̂ + ε])
/// stay in the active set; pose-dependent deep-interior pairs whose
/// inverse-transformed position lands inside the closed scan body
/// (composed sd << -cavity_inset_m) are silently excluded — the FEM
/// can't productively resolve them, and including them breaks Newton
/// convergence at non-rest slide poses. See
/// `docs/SIM_ARC_SLIDING_INTRUDER_CONTACT_RECON.md` §2-§3 for the
/// full derivation.
///
/// The smoothing window `ε` ([`INSERTION_CONTACT_SMOOTHING_EPS_M`],
/// pinned per C′.a — see the const's docstring for the pinned value
/// and full sweep table, and `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md`
/// §9 for the case-A ship rationale) extends the active band's upper
/// edge from `d̂` to `d̂ + ε` and makes the assembled contact Hessian
/// C⁰ across pair flips (F3 recon B candidate C lineage —
/// `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` for the original
/// design that shipped; bookmark §9 for the empirical ship rationale
/// and §5 for the spec's partially-falsified predictions).
///
/// The per-query normal averaging `(k, r)` pair
/// ([`INSERTION_CONTACT_NORMAL_AVG_K`] +
/// [`INSERTION_CONTACT_NORMAL_AVG_RADIUS_M`], F3 recon B candidate
/// E.b — see `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md`) smooths
/// the contact normal direction at the per-pair query site by
/// averaging `prim.grad` over the `k - 1` axis-aligned offset
/// points + the center sample. Initial pin `(1, 0.0)` short-circuits
/// to `prim.grad(p)` bit-equal pre-E.b; the E.b.4 case-A re-pin
/// engages the averaging if the cavity = 6 mm sweep finds a
/// converging `(k, r)`.
///
/// The constructor short-circuits to the pre-C.2
/// [`PenaltyRigidContact::with_params_and_interior_cutoff`] path at
/// `ε = 0`; flipping the const back to 0 would restore bit-equal
/// pre-C.2 arithmetic and lose the C′.a chattering-suppression.
fn intruder_contact_sliding_at(
    intruder: &GridSdf,
    bounds: Aabb,
    slide_pose: Isometry3<f64>,
    interference_m: f64,
    cavity_offset_m: f64,
    cavity_inset_m: f64,
) -> PenaltyRigidContact {
    let transformed = TransformedSdf::new(intruder.clone(), slide_pose);
    let intruder_solid =
        Solid::from_sdf(transformed, bounds).offset(interference_m + cavity_offset_m);
    PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging_and_interior_cutoff(
        vec![intruder_solid],
        INSERTION_CONTACT_KAPPA,
        INSERTION_CONTACT_DHAT,
        INSERTION_CONTACT_SMOOTHING_EPS_M,
        INSERTION_CONTACT_NORMAL_AVG_K,
        INSERTION_CONTACT_NORMAL_AVG_RADIUS_M,
        2.0 * cavity_inset_m,
    )
}

/// One converged step of a sliding-intruder insertion ramp.
///
/// Sliding-ramp sibling of [`RampStep`]. Carries the same shape
/// (`x_final` + `readout`) so render plumbing built around `RampStep`
/// works unchanged on the per-step output. The per-step *abscissa*
/// differs: `slide_fraction_t` + `arc_length_s_m` replace the growing
/// ramp's `interference_m`. The F-d curve at SL.3 reads
/// `(arc_length_s_m, contact_force_magnitude_n)` (per D-Slide7 +
/// `force_arc_length_curve` on [`SlideResult`]).
#[derive(Clone)]
pub struct SlideRampStep {
    /// Slide fraction `t ∈ (0, 1]` — `(k + 1) / n_steps`. `t = 1` is
    /// the fully-seated (rest-pose) step, `t = 1/n_steps` is the
    /// first step. Per D-Slide3: rest pose at `t = 0` is implicit
    /// warm-start, not a recorded step.
    pub slide_fraction_t: f64,
    /// Arc length the intruder has slid from the cap mouth (m) =
    /// `t * polyline_arc_length`. The F-d-curve abscissa replaces the
    /// growing ramp's `interference_m`.
    pub arc_length_s_m: f64,
    /// Newton iterations this step's solve took.
    pub iter_count: usize,
    /// Free-DOF residual norm at this step's convergence.
    pub final_residual_norm: f64,
    /// Converged vertex positions at this step, vertex-major xyz.
    /// Same shape as [`RampStep::x_final`]; per-tet detail derivable
    /// via [`compute_tet_readouts`].
    pub x_final: Vec<f64>,
    /// Pre-aggregated engineering scalars. Same shape + semantics as
    /// the growing ramp's [`StepReadout`] (contact force,
    /// principal-stretch extrema, peak Piola, mean energy). The
    /// engineering interpretation differs (sliding vs press-fit) but
    /// the scalars are identically defined.
    pub readout: StepReadout,
}

/// Per-tet engineering readouts at the final converged step of a
/// sliding ramp, plus the ramp-wide force–arc-length curve.
///
/// Mirrors [`InsertionResult`]. The only headline difference is the
/// curve's abscissa: arc-length-slid (m) instead of press-fit
/// interference (m) — D-Slide7. Renamed
/// [`force_arc_length_curve`](Self::force_arc_length_curve) to make
/// the abscissa unambiguous at call sites.
#[derive(Clone)]
pub struct SlideResult {
    /// Per-tet readouts at the final converged step.
    pub final_per_tet: Vec<TetReadout>,
    /// Force vs slide arc length over the ramp:
    /// `(arc_length_s_m, contact_force_magnitude_n)` pairs in ramp
    /// order. Replaces the growing ramp's
    /// `force_displacement_curve`; SL.5's F-d plot reads this and
    /// labels the abscissa "Slide arc-length (mm)".
    pub force_arc_length_curve: Vec<(f64, f64)>,
}

/// Result of a sliding insertion ramp — see [`run_sliding_insertion_ramp`].
///
/// Mirrors [`InsertionRamp`] one-for-one with two additions:
/// [`intruder_poses`](Self::intruder_poses) — one `Isometry3` per
/// converged step recording where the rigid intruder sat for that
/// step's solve — and the `result` is a [`SlideResult`] keyed on
/// arc-length instead of interference. The render plumbing at SL.4
/// uses `intruder_poses[step]` to position the constant intruder
/// mesh in the viewport.
pub struct SlideRamp {
    /// Per-step records in ramp order — only the *converged* steps.
    pub steps: Vec<SlideRampStep>,
    /// `Some(k)` if step `k` failed to converge (Fork-B graceful
    /// stall — D-Slide6); `None` if every step converged.
    pub failed_at_step: Option<usize>,
    /// Human-readable description of the failure for the failed step.
    /// Two sources: F3.4 Fork-B graceful `SolverFailure` formatted via
    /// [`solver_failure_message`], or — for undocumented solver-internal
    /// panics (e.g., Yeoh material validity) caught by `catch_unwind`
    /// — the panic payload formatted via [`panic_message`]. `None` if
    /// every step converged.
    pub failure_reason: Option<String>,
    /// Deformed vertex positions (vertex-major xyz) at the last
    /// converged step — or rest positions if step 0 itself failed.
    pub final_x: Vec<f64>,
    /// Dirichlet-pinned outer-skin vertex count (constant across the
    /// ramp; the outer skin does not move).
    pub n_pinned: usize,
    /// Per-tet detail at the final converged step + the ramp-wide
    /// force–arc-length curve. `None` if no step converged.
    pub result: Option<SlideResult>,
    /// One `Isometry3` per converged step recording the intruder's
    /// pose at that step's solve — the per-step source-of-truth for
    /// the SL.4 viewport render (`intruder_pose_at(displayed_step)`).
    pub intruder_poses: Vec<Isometry3<f64>>,
}

/// Run a quasi-static sliding-intruder insertion ramp: translate the
/// rigid scan along the centerline from `t = 1/n_steps` (just seated)
/// to `t = 1.0` (fully seated, rest pose) in `n_steps` equal
/// arc-length increments, solving the FEM at each step.
///
/// At each step the contact primitive is the transformed scan offset
/// inward by `cavity_offset_m` (see [`intruder_contact_sliding_at`]).
/// Cavity-wall deformation is localized to the contact zone where
/// the intruder currently sits, NOT propagated uniformly across the
/// wall as in the growing ramp.
///
/// Solver convergence: each Newton solve is warm-started from the
/// previous step's `x_final` (D-Slide4 — no convection-aware
/// remapping in iter-1; the contact-set discontinuity at slide-step
/// boundaries may stall some intermediate solves). Two graceful
/// surfaces (both feed `failure_reason`): the F3.4 Fork-B LM opt-in
/// makes `try_replay_step` return `Err(SolverFailure)` for the three
/// documented failure variants, and `catch_unwind` wraps the call to
/// catch undocumented solver-internal panics (Yeoh material validity
/// at `backward_euler.rs:678`, `debug_assert!`s) that are NOT
/// `SolverFailure` variants.
///
/// `cavity_inset_m` (positive — same value as `design.cavity_inset_m`)
/// is the engineered shell-wall interference. It is forwarded
/// unchanged to [`intruder_contact_sliding_at`] for the per-step
/// contact build; see that function's docstring + the CR.1 recon spec
/// for the active-set interior-cutoff derivation.
///
/// # Errors
///
/// - `n_steps` is zero;
/// - `centerline_polyline_m` has fewer than 2 points (no segments to
///   walk);
/// - [`outer_skin_bc`] fails (no outer-skin vertex in the pin-band).
pub fn run_sliding_insertion_ramp(
    geometry: InsertionGeometry,
    centerline_polyline_m: &[Point3<f64>],
    n_steps: usize,
    cavity_inset_m: f64,
) -> Result<SlideRamp> {
    if n_steps == 0 {
        return Err(anyhow!("sliding insertion ramp needs at least one step"));
    }
    if centerline_polyline_m.len() < 2 {
        return Err(anyhow!(
            "sliding insertion ramp needs a centerline polyline of ≥ 2 points (got {})",
            centerline_polyline_m.len(),
        ));
    }

    let InsertionGeometry {
        mesh,
        intruder,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets,
        per_tet_layer: _,
    } = geometry;

    let n_vertices = mesh.n_vertices();
    let n_dof = 3 * n_vertices;

    // BCs are constant across the ramp (the outer skin does not move
    // for sliding either; per spec §3d the outer_skin_bc primitive is
    // shared unchanged with the growing ramp).
    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot per-tet immutables before consuming the mesh into the
    // per-step solver clones — same dance as the growing ramp.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    #[allow(clippy::cast_possible_truncation)]
    let tets: Vec<[VertexId; 4]> = (0..n_tets as TetId).map(|t| mesh.tet_vertices(t)).collect();
    let materials: Vec<Yeoh> = mesh.materials().to_vec();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let l_m = polyline_arc_length_m(centerline_polyline_m);

    let config = insertion_solver_config();

    // x_prev starts at rest; each converged step chains its x_final in.
    let mut x_prev_flat: Vec<f64> = rest_positions
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    let v_prev = Tensor::zeros(&[n_dof]);
    let empty_theta: [f64; 0] = [];
    let theta = Tensor::from_slice(&empty_theta, &[0]);

    let mut steps: Vec<SlideRampStep> = Vec::with_capacity(n_steps);
    let mut intruder_poses: Vec<Isometry3<f64>> = Vec::with_capacity(n_steps);
    let mut failed_at_step = None;
    let mut failure_reason = None;
    for k in 0..n_steps {
        // `(k + 1) as f64 / n_steps as f64`: tiny integers — well
        // under f64's exact-integer ceiling.
        #[allow(clippy::cast_precision_loss)]
        let t = (k + 1) as f64 / n_steps as f64;
        let pose = slide_pose_at(centerline_polyline_m, t);
        let contact = intruder_contact_sliding_at(
            &intruder,
            bounds,
            pose,
            0.0,
            cavity_offset_m,
            cavity_inset_m,
        );
        let solver = CpuNewtonSolver::new(Tet4, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        // F3.4 Fork-B: `try_replay_step` surfaces Armijo stall +
        // Newton-iter-cap + doubly-failed-factor as `Err(SolverFailure)`
        // (richer context than a panic message). BUT only those three
        // variants — solver-internal panics (Yeoh material validity at
        // `backward_euler.rs:678`, `debug_assert!`s, OOM) still unwind
        // and would crash the Bevy app if not caught. Belt-and-suspenders:
        // `catch_unwind` outside catches those, `try_replay_step` inside
        // gives the graceful path SolverFailure context. Both feed into
        // `failure_reason` so the viewport reads "stalled at step N:
        // <reason>" regardless of which surface tripped.
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.try_replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        match outcome {
            Ok(Ok(step)) => {
                let positions_k: Vec<Vec3> = positions_from_flat(&step.x_final);
                // `PenaltyRigidContact` is not `Clone`; rebuild for
                // the readout pass (same row-23 precedent the growing
                // ramp uses at `run_insertion_ramp`).
                let readout_contact = intruder_contact_sliding_at(
                    &intruder,
                    bounds,
                    pose,
                    0.0,
                    cavity_offset_m,
                    cavity_inset_m,
                );
                let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                let contact_readouts =
                    filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                let per_tet =
                    compute_tet_readouts(&rest_positions, &positions_k, &tets, &materials);
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts);

                steps.push(SlideRampStep {
                    slide_fraction_t: t,
                    arc_length_s_m: t * l_m,
                    iter_count: step.iter_count,
                    final_residual_norm: step.final_residual_norm,
                    x_final: step.x_final.clone(),
                    readout: step_readout,
                });
                intruder_poses.push(pose);
                x_prev_flat = step.x_final;
            }
            Ok(Err(failure)) => {
                // F3.4 graceful SolverFailure (ArmijoStall /
                // NewtonIterCap / DoublyFailedFactor).
                failed_at_step = Some(k);
                failure_reason = Some(solver_failure_message(&failure));
                break;
            }
            Err(payload) => {
                // Undocumented panic from inside the solver — e.g.,
                // Yeoh material validity (Phase 4 Decision Q
                // fail-closed). Not a SolverFailure variant; surface
                // it as the partial-ramp `failure_reason` via the
                // pre-F3.4 `panic_message` path.
                failed_at_step = Some(k);
                failure_reason = Some(panic_message(&*payload));
                break;
            }
        }
    }

    // Per-tet detail at the final converged step + ramp force-arc-length
    // curve. `result = None` only when no step converged.
    let result = steps.last().map(|last| {
        let final_positions = positions_from_flat(&last.x_final);
        let final_per_tet =
            compute_tet_readouts(&rest_positions, &final_positions, &tets, &materials);
        let force_arc_length_curve = steps
            .iter()
            .map(|s| (s.arc_length_s_m, s.readout.contact_force_magnitude_n))
            .collect();
        SlideResult {
            final_per_tet,
            force_arc_length_curve,
        }
    });

    Ok(SlideRamp {
        steps,
        failed_at_step,
        failure_reason,
        final_x: x_prev_flat,
        n_pinned,
        result,
        intruder_poses,
    })
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level; the test
    // module opts out, same posture as `main.rs`'s test module.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::PathBuf;

    use mesh_io::{load_stl, save_stl};
    use mesh_repair::{
        RepairParams, find_connected_components, fix_winding_order, repair_mesh, validate_mesh,
    };

    use super::*;

    /// Build a unit-cube `IndexedMesh` (8 vertices, 12 faces) centered
    /// at the origin for the pure-helper tests.
    fn unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        for &(x, y, z) in &[
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        for tri in [
            [0, 2, 1],
            [0, 3, 2], // -z
            [4, 5, 6],
            [4, 6, 7], // +z
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [1, 2, 6],
            [1, 6, 5], // +x
            [0, 4, 7],
            [0, 7, 3], // -x
        ] {
            mesh.faces.push(tri);
        }
        mesh
    }

    // ─── SL.1 — TransformedSdf + slide_pose_at unit tests ──────────
    //
    // 7 tests pin the spec §3b + §3c primitive contract before they
    // get consumed by SL.2's `run_sliding_insertion_ramp`. Inner SDF
    // is `RigidPlane` (sim-soft) — flat surface + constant normal
    // keeps the rotated/translated expectations algebraically
    // closed-form.

    /// `TransformedSdf` with the identity transform must be a
    /// bit-exact pass-through of the inner SDF (no inverse-mul drift,
    /// no spurious rotation of `grad`).
    #[test]
    fn transformed_sdf_identity_passes_through_eval_and_grad() {
        let plane = sim_soft::RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let wrapped = TransformedSdf::new(plane, Isometry3::identity());
        for &p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(-0.5, 0.25, -1.5),
        ] {
            assert!(
                (wrapped.eval(p) - Sdf::eval(&plane, p)).abs() < 1e-12,
                "identity-transformed eval drifted at {p:?}",
            );
            let g_wrapped = wrapped.grad(p);
            let g_plane = Sdf::grad(&plane, p);
            assert!(
                (g_wrapped - g_plane).norm() < 1e-12,
                "identity-transformed grad drifted at {p:?}: {g_wrapped:?} vs {g_plane:?}",
            );
        }
    }

    /// Pure translation by `Δ` along the plane normal must shift the
    /// signed distance by `−Δ·n` (plane sitting `Δ` higher reports
    /// `+Δ` smaller signed distance for a point above the original).
    /// `grad` stays as the original outward normal — translation does
    /// not rotate the gradient.
    #[test]
    fn transformed_sdf_pure_translation_shifts_eval_passes_grad() {
        // Plane `z = 0`, outward normal +Z. Translating it to `z = 0.5`
        // moves the surface up; for `p` at `(0, 0, 1)`, the signed
        // distance to the translated plane is `0.5` (was `1.0` to the
        // original plane).
        let plane = sim_soft::RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let translation = Isometry3::translation(0.0, 0.0, 0.5);
        let wrapped = TransformedSdf::new(plane, translation);
        let p = Point3::new(0.0, 0.0, 1.0);
        assert!(
            (wrapped.eval(p) - 0.5).abs() < 1e-12,
            "translated eval wrong: {}",
            wrapped.eval(p),
        );
        let g = wrapped.grad(p);
        assert!(
            (g - Vec3::new(0.0, 0.0, 1.0)).norm() < 1e-12,
            "translation should not rotate grad, got {g:?}",
        );
    }

    /// Pure rotation of the plane (about an axis NOT parallel to its
    /// normal) rotates the world-frame outward normal. A plane with
    /// rest normal `+Z` rotated 90° about the +Y axis ends up with
    /// world normal `+X`; `grad` must reflect that.
    #[test]
    fn transformed_sdf_pure_rotation_rotates_grad() {
        let plane = sim_soft::RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        // Rotate +Z → +X via 90° rotation about +Y.
        let rotation = Isometry3::rotation(Vector3::new(0.0, std::f64::consts::FRAC_PI_2, 0.0));
        let wrapped = TransformedSdf::new(plane, rotation);
        // Query at `(1, 0, 0)`: the rotated plane passes through the
        // origin with normal +X, so signed distance is `+1`.
        let p = Point3::new(1.0, 0.0, 0.0);
        assert!(
            (wrapped.eval(p) - 1.0).abs() < 1e-12,
            "rotated eval wrong: {}",
            wrapped.eval(p),
        );
        let g = wrapped.grad(p);
        let expected = Vec3::new(1.0, 0.0, 0.0);
        assert!(
            (g - expected).norm() < 1e-12,
            "rotated grad should be +X, got {g:?}",
        );
    }

    /// Combined translation + rotation must apply both: rotate the
    /// plane to the new orientation, then translate it. The plane
    /// originally `{z = 0, n = +Z}` rotated 90° about +Y becomes
    /// `{x = 0, n = +X}`; translating by `+1.0` along +X moves the
    /// surface to `x = 1`. A query at `(2, 0, 0)` then sits at signed
    /// distance `+1` with outward normal `+X`.
    #[test]
    fn transformed_sdf_combined_translation_and_rotation() {
        let plane = sim_soft::RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let rotation = nalgebra::UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let translation = nalgebra::Translation3::new(1.0, 0.0, 0.0);
        let iso = Isometry3::from_parts(translation, rotation);
        let wrapped = TransformedSdf::new(plane, iso);
        let p = Point3::new(2.0, 0.0, 0.0);
        assert!(
            (wrapped.eval(p) - 1.0).abs() < 1e-12,
            "combined eval wrong: {}",
            wrapped.eval(p),
        );
        let g = wrapped.grad(p);
        assert!(
            (g - Vec3::new(1.0, 0.0, 0.0)).norm() < 1e-12,
            "combined grad should be +X, got {g:?}",
        );
    }

    /// Helper: straight 4-segment polyline along +Z spanning
    /// `z ∈ [0, 4]`. `centerline[0]` = TIP (closed end), index 4 =
    /// FLOOR (open end / cap rim).
    fn straight_z_centerline() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, 0.0), // tip (rest pose)
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.0, 0.0, 4.0), // floor / cap mouth
        ]
    }

    /// `slide_pose_at(t = 1)` = fully seated = identity transform.
    /// The intruder's tip sits at its rest position
    /// `centerline[0]`; the cleaned-scan body geometry is unchanged.
    #[test]
    fn slide_pose_at_t_eq_1_is_identity() {
        let centerline = straight_z_centerline();
        let pose = slide_pose_at(&centerline, 1.0);
        let p = Point3::new(0.5, -0.25, 1.0);
        let transformed = pose * p;
        assert!(
            (transformed - p).norm() < 1e-12,
            "t=1 should be identity, got transformed = {transformed:?}",
        );
    }

    /// `slide_pose_at(t = 0)` = start of slide = tip translated all
    /// the way from the rest position (centerline[0]) to the floor /
    /// cap mouth (centerline.last()). For the straight `+Z` fixture
    /// of length 4, that's a `(0, 0, +4)` translation of every body
    /// point.
    #[test]
    fn slide_pose_at_t_eq_0_translates_tip_to_floor() {
        let centerline = straight_z_centerline();
        let pose = slide_pose_at(&centerline, 0.0);
        let tip_rest = centerline[0];
        let tip_world = pose * tip_rest;
        let floor = *centerline.last().unwrap();
        assert!(
            (tip_world - floor).norm() < 1e-12,
            "t=0 should land tip at floor {floor:?}, got {tip_world:?}",
        );
    }

    /// `slide_pose_at(t = 0.5)` lands the intruder tip at the
    /// arc-length midpoint of the centerline. For the straight `+Z`
    /// fixture of length 4, the midpoint is `(0, 0, 2)`.
    #[test]
    fn slide_pose_at_t_eq_0_5_translates_tip_to_arc_midpoint() {
        let centerline = straight_z_centerline();
        let pose = slide_pose_at(&centerline, 0.5);
        let tip_rest = centerline[0];
        let tip_world = pose * tip_rest;
        let midpoint = Point3::new(0.0, 0.0, 2.0);
        assert!(
            (tip_world - midpoint).norm() < 1e-12,
            "t=0.5 should land tip at midpoint {midpoint:?}, got {tip_world:?}",
        );
    }

    #[test]
    fn scan_aabb_wraps_vertices_with_margin() {
        let aabb = scan_aabb(&unit_cube(), 0.25);
        assert_eq!(aabb.min, Point3::new(-0.75, -0.75, -0.75));
        assert_eq!(aabb.max, Point3::new(0.75, 0.75, 0.75));
    }

    #[test]
    fn aabb3_for_meshing_preserves_corners() {
        let aabb = Aabb::new(Point3::new(-1.0, -2.0, -3.0), Point3::new(4.0, 5.0, 6.0));
        let a3 = aabb3_for_meshing(&aabb);
        assert_eq!(a3.min, Vec3::new(-1.0, -2.0, -3.0));
        assert_eq!(a3.max, Vec3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn decimate_for_sdf_is_noop_below_target() {
        // 12-face cube, target 10_000 — already under target, so the
        // face set is returned unchanged (the cube survives the weld).
        let out = decimate_for_sdf(&unit_cube(), 10_000);
        assert_eq!(out.faces.len(), 12);
    }

    /// Slice 7.3c — [`gaussian_smooth_3d_separable`] is the identity on
    /// a constant field. For any σ, a Gaussian-weighted average of a
    /// constant is the constant; the smoother must preserve that bit-
    /// exactly (modulo fp normalization round-off), because constant
    /// regions of the signed buffer correspond to the bbox-margin
    /// "deep outside" / "deep inside" that the contact solver relies
    /// on for sign discrimination.
    #[test]
    fn gaussian_smooth_is_bit_exact_on_constant_field() {
        let (w, h, d) = (5, 4, 3);
        let c = 7.5_f64;
        let field = vec![c; w * h * d];
        for &sigma in &[0.0, 0.5, 1.0, 1.5] {
            let out = gaussian_smooth_3d_separable(&field, w, h, d, sigma);
            assert_eq!(out.len(), field.len());
            for (i, &v) in out.iter().enumerate() {
                assert!(
                    (v - c).abs() < 1e-12,
                    "constant-field smoothing at σ={sigma} drifted at index {i}: \
                     got {v}, expected {c}",
                );
            }
        }
    }

    /// Slice 7.3c — `sigma_cells = 0.0` is a true identity (the
    /// implementation short-circuits to `field.to_vec()`). This pins
    /// the contract that callers can disable smoothing without
    /// fp-noise side effects.
    #[test]
    fn gaussian_smooth_with_zero_sigma_is_exact_identity() {
        let (w, h, d) = (3, 3, 3);
        // A field with structure — not a constant — so any
        // accidental convolution would visibly mutate it.
        let field: Vec<f64> = (0..w * h * d)
            .map(|i| (i as f64).sin() * 17.5 - 0.25)
            .collect();
        let out = gaussian_smooth_3d_separable(&field, w, h, d, 0.0);
        assert_eq!(out, field, "σ=0 must return the field bit-exactly");
    }

    /// Slice 7.3c — separability means the smoother shifts feature
    /// position by O(σ²·κ) and preserves a *linear* field exactly
    /// (within fp). Linear-along-x is bit-exact for any axis-aligned
    /// kernel: each x-pass averages `f(x-1) + f(x) + f(x+1) = 3·f(x)`
    /// (for an odd, symmetric, normalized kernel on a linear field —
    /// the symmetric weights pair to zero around the center). This
    /// pins the no-bias-on-flat-surfaces contract; non-zero
    /// curvature is the only thing the smoother shifts.
    #[test]
    fn gaussian_smooth_preserves_linear_field_interior() {
        let (w, h, d) = (7, 5, 5);
        let idx = |x: usize, y: usize, z: usize| z * w * h + y * w + x;
        // Linear along x: f(x, y, z) = 2x + 3 (interior is exact;
        // boundary picks up the clamp-edge mirror, expected).
        let mut field = vec![0.0_f64; w * h * d];
        for z in 0..d {
            for y in 0..h {
                for x in 0..w {
                    field[idx(x, y, z)] = 2.0 * x as f64 + 3.0;
                }
            }
        }
        let out = gaussian_smooth_3d_separable(&field, w, h, d, 0.5);
        // Interior x ∈ {1 .. w-2} away from clamp boundaries.
        for z in 0..d {
            for y in 0..h {
                for x in 1..w - 1 {
                    let expected = 2.0 * x as f64 + 3.0;
                    let got = out[idx(x, y, z)];
                    assert!(
                        (got - expected).abs() < 1e-12,
                        "linear-field smoothing at ({x},{y},{z}) drifted: \
                         got {got}, expected {expected}",
                    );
                }
            }
        }
    }

    /// One [`SimLayer`] — test sugar. `slacker_fraction` defaults to
    /// `0.0` (base material). Tests that exercise slice 7.5's
    /// Slacker-effective-shore path use `layer_with_slacker` instead.
    fn layer(thickness_m: f64, anchor: &str) -> SimLayer {
        SimLayer {
            thickness_m,
            anchor_key: anchor.to_string(),
            slacker_fraction: 0.0,
        }
    }

    /// Same as [`layer`] but with an explicit `slacker_fraction` —
    /// slice 7.5's effective-Yeoh tests use this.
    fn layer_with_slacker(thickness_m: f64, anchor: &str, slacker_fraction: f64) -> SimLayer {
        SimLayer {
            thickness_m,
            anchor_key: anchor.to_string(),
            slacker_fraction,
        }
    }

    #[test]
    fn silicone_for_anchor_resolves_the_catalog() {
        // All eight cf-device-design catalog keys resolve.
        for key in [
            "ECOFLEX_00_10",
            "ECOFLEX_00_20",
            "ECOFLEX_00_30",
            "ECOFLEX_00_50",
            "DRAGON_SKIN_10A",
            "DRAGON_SKIN_15",
            "DRAGON_SKIN_20A",
            "DRAGON_SKIN_30A",
        ] {
            assert!(
                silicone_for_anchor(key).is_ok(),
                "catalog key {key} should resolve"
            );
        }
        // The firmest Dragon Skin grade is stiffer than the softest
        // Ecoflex — a sanity check that distinct keys map to distinct
        // materials, not all to one fallback.
        let soft = silicone_for_anchor("ECOFLEX_00_10").unwrap();
        let firm = silicone_for_anchor("DRAGON_SKIN_30A").unwrap();
        assert!(
            firm.mu > soft.mu,
            "DRAGON_SKIN_30A (μ {}) should be stiffer than ECOFLEX_00_10 (μ {})",
            firm.mu,
            soft.mu,
        );
        // An off-catalog key is an error, not a silent substitution.
        assert!(silicone_for_anchor("UNOBTANIUM").is_err());
    }

    // ── 7.5 — Slacker → sim-modulus resolution ────────────────────

    /// `slacker_fraction = 0.0` returns the base anchor bit-exact —
    /// the regression contract that keeps both ramps unchanged
    /// post-7.5.
    #[test]
    fn effective_silicone_at_zero_slacker_is_base() {
        for key in [
            "ECOFLEX_00_10",
            "ECOFLEX_00_20",
            "ECOFLEX_00_30",
            "ECOFLEX_00_50",
            "DRAGON_SKIN_10A",
            "DRAGON_SKIN_15",
            "DRAGON_SKIN_20A",
            "DRAGON_SKIN_30A",
        ] {
            let l = layer_with_slacker(0.005, key, 0.0);
            let (mat, res) = effective_silicone_for_layer(&l).unwrap();
            let base = silicone_for_anchor(key).unwrap();
            assert_eq!(res, SlackerResolution::Base, "{key}");
            // Bit-exact identity: the regression ramps depend on
            // this.
            assert_eq!(mat.mu, base.mu, "{key}: μ must match base");
            assert_eq!(mat.lambda, base.lambda, "{key}: λ must match base");
            assert_eq!(mat.c2, base.c2, "{key}: C₂ must match base");
        }
    }

    /// `Support::NotRecommended` (Ecoflex 00-10 + Slacker) and
    /// `Support::NoData` (Dragon Skin 15 / 20A / 30A) fall back to
    /// the base anchor even when `slacker_fraction > 0` — the UI
    /// disables the picker for these, so a non-zero value reaching
    /// the sim is defensive territory; we surface it as `Base`
    /// rather than failing.
    #[test]
    fn effective_silicone_unsupported_anchors_fall_back_to_base() {
        for key in [
            "ECOFLEX_00_10",
            "DRAGON_SKIN_15",
            "DRAGON_SKIN_20A",
            "DRAGON_SKIN_30A",
        ] {
            let l = layer_with_slacker(0.005, key, 0.50);
            let (mat, res) = effective_silicone_for_layer(&l).unwrap();
            let base = silicone_for_anchor(key).unwrap();
            assert_eq!(res, SlackerResolution::Base, "{key}");
            assert_eq!(mat.mu, base.mu, "{key}");
        }
    }

    /// `DRAGON_SKIN_10A + 0.25 Slacker` lands at Shore 00-30 (per the
    /// TB curve), which sim-soft anchors exactly at ECOFLEX_00_30.
    /// `from_effective_shore` returns the anchor's Yeoh params
    /// bit-exact (the bracket interpolation collapses at the anchor
    /// point). Resolution: `Interpolated`.
    #[test]
    fn effective_silicone_ds10a_quarter_slacker_lands_at_ecoflex_00_30() {
        let l = layer_with_slacker(0.005, "DRAGON_SKIN_10A", 0.25);
        let (mat, res) = effective_silicone_for_layer(&l).unwrap();
        assert_eq!(res, SlackerResolution::Interpolated);
        let target = silicone_for_anchor("ECOFLEX_00_30").unwrap();
        // `from_effective_shore` interpolates across the Shore-00
        // anchor table. Shore 00-30 happens to be an exact anchor
        // point (ECOFLEX_00_30) so the bracket interpolation collapses
        // to that anchor; μ / λ / C₂ are bit-exact.
        assert!(
            (mat.mu - target.mu).abs() < 1e-9,
            "μ at DS10A+0.25 should match ECOFLEX_00_30 (got {} vs {})",
            mat.mu,
            target.mu,
        );
        assert!(
            (mat.c2 - target.c2).abs() < 1e-9,
            "C₂ at DS10A+0.25 should match ECOFLEX_00_30 (got {} vs {})",
            mat.c2,
            target.c2,
        );
        // The effective material is SOFTER than the base DS10A.
        let base = silicone_for_anchor("DRAGON_SKIN_10A").unwrap();
        assert!(mat.mu < base.mu, "+Slacker must soften the material");
    }

    /// Most Slacker-modified silicones land in Shore 000 (gel scale),
    /// which sim-soft does not anchor. The resolver floors to
    /// `ECOFLEX_00_10` (the softest published anchor) and flags
    /// `FlooredAtSoftestAnchor`.
    #[test]
    fn effective_silicone_shore_000_outcomes_floor_at_ecoflex_00_10() {
        // ECOFLEX_00_30 + 0.50 Slacker → 000-20 → floor.
        let l = layer_with_slacker(0.005, "ECOFLEX_00_30", 0.50);
        let (mat, res) = effective_silicone_for_layer(&l).unwrap();
        assert_eq!(res, SlackerResolution::FlooredAtSoftestAnchor);
        let floor = silicone_for_anchor("ECOFLEX_00_10").unwrap();
        assert_eq!(mat.mu, floor.mu);
        assert_eq!(mat.lambda, floor.lambda);
        assert_eq!(mat.c2, floor.c2);

        // DRAGON_SKIN_10A + 0.50 Slacker → 000-50 → also floor.
        let l = layer_with_slacker(0.005, "DRAGON_SKIN_10A", 0.50);
        let (_, res) = effective_silicone_for_layer(&l).unwrap();
        assert_eq!(res, SlackerResolution::FlooredAtSoftestAnchor);
    }

    /// Off-curve `slacker_fraction` (not on any tabulated TB point)
    /// is a wiring-side bug — surface it as an `Err` rather than
    /// silently rounding. The UI's `resolve_slacker_fraction`
    /// snaps inputs to the curve before they reach the sim.
    #[test]
    fn effective_silicone_off_curve_fraction_errors() {
        let l = layer_with_slacker(0.005, "ECOFLEX_00_30", 0.10); // not on the curve
        let err = effective_silicone_for_layer(&l).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("off the ECOFLEX_00_30 curve"),
            "error should name the off-curve fraction + anchor (got {msg:?})"
        );
    }

    /// As Slacker fraction increases along a curve, the effective μ
    /// must decrease monotonically (softening). Spot-checks the
    /// monotonicity across DS10A's published points (the only curve
    /// with a Shore-A → Shore-00 → Shore-000 sweep representable in
    /// the sim).
    #[test]
    fn effective_silicone_softens_monotonically_along_ds10a() {
        let base = silicone_for_anchor("DRAGON_SKIN_10A").unwrap();
        // 0.0 → DS10A native.
        // 0.25 → Shore 00-30 (representable; ECOFLEX_00_30 anchor).
        // 0.50+ → Shore 000-X (floored to ECOFLEX_00_10).
        let mu_at = |frac: f64| -> f64 {
            let l = layer_with_slacker(0.005, "DRAGON_SKIN_10A", frac);
            effective_silicone_for_layer(&l).unwrap().0.mu
        };
        let m0 = base.mu;
        let m1 = mu_at(0.25);
        let m2 = mu_at(0.50);
        let m3 = mu_at(0.75);
        let m4 = mu_at(1.00);
        assert!(m1 < m0, "0.25 must soften from base");
        assert!(m2 < m1, "0.50 must soften from 0.25");
        // 0.50 / 0.75 / 1.00 all hit the same floor (Shore 000 → 00-10),
        // so m2 == m3 == m4.
        assert_eq!(m2, m3, "floor at 0.50 should match 0.75 (both Shore 000)");
        assert_eq!(m3, m4, "floor at 0.75 should match 1.00 (both Shore 000)");
        // The floor is ECOFLEX_00_10's μ — pin that explicitly.
        let floor_mu = silicone_for_anchor("ECOFLEX_00_10").unwrap().mu;
        assert_eq!(m2, floor_mu);
    }

    #[test]
    fn layer_boundary_thresholds_single_layer_is_empty() {
        // One layer → zero internal boundaries → `ConstantField` path.
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        assert!(layer_boundary_thresholds(&design).is_empty());
    }

    #[test]
    fn layer_boundary_thresholds_are_cumulative_offsets_from_scan() {
        // 3 layers, thicknesses 2/3/4 mm, cavity inset 3 mm. The two
        // internal boundaries (layer 0|1, layer 1|2) sit at cumulative
        // thickness minus the inset: 2-3 = -1 mm, 5-3 = +2 mm.
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![
                layer(0.002, "ECOFLEX_00_30"),
                layer(0.003, "DRAGON_SKIN_10A"),
                layer(0.004, "DRAGON_SKIN_20A"),
            ],
        };
        let t = layer_boundary_thresholds(&design);
        assert_eq!(t.len(), 2);
        assert!((t[0] - (-0.001)).abs() < 1e-12);
        assert!((t[1] - 0.002).abs() < 1e-12);
        // Strictly increasing — `LayeredScalarField::new` requires it.
        assert!(t[1] > t[0]);
    }

    // ── Sub-leaf A4: candidate-A pinned-floor consumer ───────────────

    /// Compact cube (half-extent 25 mm) sized so a 4 mm BCC build
    /// completes in well under a second — small enough for a unit
    /// test but with enough internal room for a `cavity_inset_m=3 mm
    /// + layer_thickness=5 mm` design.
    fn small_test_cube() -> IndexedMesh {
        let h = 0.025_f64;
        let mut mesh = IndexedMesh::new();
        for &(x, y, z) in &[
            (-h, -h, -h),
            (h, -h, -h),
            (h, h, -h),
            (-h, h, -h),
            (-h, -h, h),
            (h, -h, h),
            (h, h, h),
            (-h, h, h),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        for tri in [
            [0, 2, 1],
            [0, 3, 2], // -z
            [4, 5, 6],
            [4, 6, 7], // +z
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [1, 2, 6],
            [1, 6, 5], // +x
            [0, 4, 7],
            [0, 7, 3], // -x
        ] {
            mesh.faces.push(tri);
        }
        mesh
    }

    #[test]
    fn build_insertion_geometry_no_caps_byte_identical_to_pre_pinned_floor() {
        // The no-caps fast path threads through
        // `pinned_floor_shell(closed_sdf, _, bounds, &[], offset)`,
        // which short-circuits to `Solid::from_sdf(closed).offset(offset)`
        // — bit-identical to the pre-pinned-floor uniform-offset call
        // the consumer used before sub-leaf 5 (and what scope-C broke).
        // The candidate-A primitive's no-caps byte-equality is pinned
        // at the cf-design level
        // (`solid_layered::tests::pinned_floor_shell_empty_caps_byte_identical_to_offset`);
        // at the consumer level we just need to prove the call site
        // still produces a deterministic, non-empty mesh on a tiny
        // fixture so any future refactor that breaks the no-caps
        // short-circuit surfaces here.
        let scan = small_test_cube();
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        let g_a = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("no-caps build must succeed");
        let g_b = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("no-caps re-build must succeed");
        assert!(
            g_a.n_tets > 0,
            "no-caps build must produce a non-empty mesh"
        );
        // Determinism: identical inputs → identical mesh — proves the
        // no-caps fast path is stable across rebuilds and surfaces any
        // accidental iterator-order non-determinism the new
        // `Arc<dyn Sdf>` wiring might introduce.
        assert_eq!(
            g_a.n_tets, g_b.n_tets,
            "no-caps build must be deterministic"
        );
        let pa = g_a.mesh.positions();
        let pb = g_b.mesh.positions();
        assert_eq!(pa.len(), pb.len());
        for (a, b) in pa.iter().zip(pb.iter()) {
            assert!(
                (a - b).norm() < 1e-12,
                "no-caps positions diverged across rebuilds: {a:?} vs {b:?}",
            );
        }
        // Intruder is the closed-body SDF — must still read the
        // bbox-min corner as outside (positive sign). The
        // `Arc::new(scan_sdf.clone())` wrapping must not flip sign.
        assert!(
            g_a.intruder.eval(g_a.bounds.min) > 0.0,
            "intruder GridSdf must read bbox-min corner positive (outside)",
        );
    }

    /// H4 plumbing sentinel — every per-tet `Yeoh` in the produced
    /// mesh carries `(Some(max), Some(min))` principal-stretch caps
    /// rather than the legacy `(None, None)` fallback. Pinned at
    /// `silicone.validity_max_principal_stretch` /
    /// `validity_min_principal_stretch` for the single ECOFLEX_00_30
    /// layer used in the fixture so a future refactor that flips
    /// `build_insertion_geometry` back to the bounds-less 3-arg
    /// constructor (or that silently drops one bound) trips here.
    /// Closes the `MaterialField`-drops-bounds gap diagnosed at
    /// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4.
    #[test]
    fn build_insertion_geometry_per_tet_yeoh_carries_calibrated_principal_stretch_bounds() {
        let scan = small_test_cube();
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        let g = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("build must succeed on the small test cube fixture");

        let materials = g.mesh.materials();
        assert!(!materials.is_empty(), "must produce at least one tet");
        let expected_max = ECOFLEX_00_30.validity_max_principal_stretch;
        let expected_min = ECOFLEX_00_30.validity_min_principal_stretch;
        for (tet_id, yeoh) in materials.iter().enumerate() {
            let validity = yeoh.validity();
            assert_eq!(
                validity.max_principal_stretch,
                Some(expected_max),
                "tet {tet_id} max_principal_stretch must equal ECOFLEX_00_30's calibrated 0.8·λ_break"
            );
            assert_eq!(
                validity.min_principal_stretch,
                Some(expected_min),
                "tet {tet_id} min_principal_stretch must equal the family-uniform 0.30 compressive cap"
            );
        }
    }

    #[test]
    fn build_insertion_geometry_with_caps_opens_body_at_cap_plane() {
        // Build the same design twice: once without caps, once with a
        // single cap on the +z face. Under candidate A the cap-plane
        // fold inside `pinned_floor_shell` pins the cavity's floor at
        // the cap plane — the cavity reaches all the way up to z = h,
        // and the body shell at the cap plane collapses to an annular
        // rim (instead of the full domed wall the no-caps uniform
        // offset gives). At BCC cell size 4 mm the tet count drops by
        // a factor of ~5 on the 50 mm cube fixture (24884 → 4564 on
        // the iter-1 reference run); the structural difference between
        // the user's pinned-floor geometric model and the pre-pinned
        // uniform offset surfaces at the consumer level here.
        let h = 0.025_f64;
        let scan = small_test_cube();
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        let cap = CapPlane {
            centroid: Point3::new(0.0, 0.0, h),
            normal: Vector3::new(0.0, 0.0, 1.0),
            vertex_count: 4,
            loop_index: 0,
        };

        let g_no_caps = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("no-caps build must succeed");
        let g_caps = build_insertion_geometry(&scan, &design, &[cap], 2_500, 0.004)
            .expect("with-caps build must succeed");

        // Both meshes non-empty.
        assert!(g_no_caps.n_tets > 0);
        assert!(g_caps.n_tets > 0);
        // The body wall loses its dome over the cavity (the cap plane
        // is the cavity floor); tet count must drop by ≥ 2× to count
        // as a meaningful structural difference, not just a noise-
        // level BCC-lattice shift.
        assert!(
            g_caps.n_tets * 2 < g_no_caps.n_tets,
            "with-caps build must produce ≥ 2× fewer tets than no-caps \
             (got no_caps={}, with_caps={}); the cap fold should open the cavity \
             at the cap plane and remove the dome shell",
            g_no_caps.n_tets,
            g_caps.n_tets,
        );
    }

    #[test]
    fn build_insertion_geometry_rejects_degenerate_designs() {
        // Degenerate inputs are caught up front as `Err` — never a
        // panic deep in `LayeredScalarField::new`. The validation runs
        // before the scan is decimated, so a stub cube is enough and
        // the test stays fast.
        let scan = unit_cube();
        let build = |design: &SimDesign| build_insertion_geometry(&scan, design, &[], 2_500, 0.004);

        let cases = [
            (
                "no layers",
                SimDesign {
                    cavity_inset_m: 0.003,
                    layers: vec![],
                },
            ),
            (
                "zero thickness",
                SimDesign {
                    cavity_inset_m: 0.003,
                    layers: vec![layer(0.0, "ECOFLEX_00_30")],
                },
            ),
            (
                "negative thickness",
                SimDesign {
                    cavity_inset_m: 0.003,
                    layers: vec![layer(-0.002, "ECOFLEX_00_30")],
                },
            ),
            (
                "non-finite thickness",
                SimDesign {
                    cavity_inset_m: 0.003,
                    layers: vec![layer(f64::NAN, "ECOFLEX_00_30")],
                },
            ),
            (
                "non-finite inset",
                SimDesign {
                    cavity_inset_m: f64::INFINITY,
                    layers: vec![layer(0.005, "ECOFLEX_00_30")],
                },
            ),
            (
                "off-catalog anchor",
                SimDesign {
                    cavity_inset_m: 0.003,
                    layers: vec![layer(0.005, "UNOBTANIUM")],
                },
            ),
        ];
        for (label, design) in &cases {
            assert!(
                build(design).is_err(),
                "{label} design should be rejected as Err",
            );
        }
    }

    /// SDF bridge spike against the iter-1 cleaned scan.
    ///
    /// `#[ignore]` — needs the repo-excluded iter-1 fixture
    /// (`/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl`,
    /// override with the `CF_DEVICE_DESIGN_SPIKE_SCAN` env var). Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design insertion_sim -- --ignored --nocapture
    /// ```
    ///
    /// Skips gracefully (no failure) when the fixture is absent, so the
    /// `--ignored` sweep is portable across machines.
    #[test]
    #[ignore = "needs the repo-excluded iter-1 scan fixture; run with --ignored"]
    fn sdf_bridge_spike_on_iter1_scan() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }

        let scan = load_stl(&path).expect("load the iter-1 cleaned scan");
        eprintln!(
            "loaded {} ({} faces, {} vertices)",
            path.display(),
            scan.faces.len(),
            scan.vertices.len(),
        );

        // Sweep decimation targets at the rows 21–25 safe cell size
        // (4 mm) to find the knee where SDF queries stay tractable but
        // geometry fidelity is still honest. 1500 = the viewport
        // proxy's target (sanity floor); 8k / 40k probe upward.
        let cell_size_m = 0.004;
        let wall_thickness_m = 0.006;
        for target_faces in [1_500_usize, 8_000, 40_000] {
            let report = run_sdf_bridge_spike(&scan, target_faces, cell_size_m, wall_thickness_m)
                .expect("SDF bridge spike should produce a tet mesh");
            eprintln!("{report}");
            // Sanity only — a solver-usable mesh is non-empty with no
            // inverted tets. Timing + quality are the spike's payload,
            // reported above, not asserted.
            assert!(report.n_tets > 0, "tet mesh must be non-empty");
            assert_eq!(
                report.inverted_tets, 0,
                "tet mesh must have no inverted (non-positive-volume) tets",
            );
        }
    }

    /// [`build_insertion_geometry`] against the iter-1 cleaned scan.
    ///
    /// `#[ignore]` — same repo-excluded fixture + `CF_DEVICE_DESIGN_SPIKE_SCAN`
    /// override as [`sdf_bridge_spike_on_iter1_scan`]; skips
    /// gracefully when the fixture is absent. Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design insertion_sim -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "needs the repo-excluded iter-1 scan fixture; run with --ignored"]
    fn build_insertion_geometry_on_iter1_scan() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }
        let scan = load_stl(&path).expect("load the iter-1 cleaned scan");

        // Low SDF-source resolution per the 7.0 finding; the rows
        // 21–25 safe BCC cell size.
        let sdf_target_faces = 2_500;
        let cell_size_m = 0.004;

        let count_inverted = |g: &InsertionGeometry| {
            g.mesh
                .quality()
                .signed_volume
                .iter()
                .filter(|&&v| v <= 0.0)
                .count()
        };

        // (1) The default-shaped device — a single Ecoflex 00-30
        // layer. `ConstantField` path: every tet carries one material.
        let single = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        let g1 = build_insertion_geometry(&scan, &single, &[], sdf_target_faces, cell_size_m)
            .expect("single-layer geometry should build");
        eprintln!(
            "single-layer: {} tets, cavity-offset {:.1} mm, outer-offset {:.1} mm",
            g1.n_tets,
            g1.cavity_offset_m * 1e3,
            g1.outer_offset_m * 1e3,
        );
        assert!(g1.n_tets > 0, "single-layer mesh must be non-empty");
        assert_eq!(
            count_inverted(&g1),
            0,
            "single-layer mesh must have no inverted tets"
        );
        let ecoflex_mu = silicone_for_anchor("ECOFLEX_00_30").unwrap().mu;
        assert!(
            g1.mesh
                .materials()
                .iter()
                .all(|m| (m.mu() - ecoflex_mu).abs() < 1e-9),
            "single-layer tets should all carry the ECOFLEX_00_30 modulus",
        );
        // The rigid intruder is the flood-fill `GridSdf` — confirm it
        // is signed sanely: the bbox-min corner is outside the scan,
        // so it must read positive (a wrong sign here was the whole
        // 7.2 failure mode; the dedicated proof is `grid_sdf_fix_spike`
        // + `run_single_insertion_step_on_iter1_scan`).
        assert!(
            g1.intruder.eval(g1.bounds.min) > 0.0,
            "intruder GridSdf must read the bbox-min corner as outside (positive)",
        );

        // (2) A three-layer device, three different silicones, each
        // layer thicker than the BCC cell so the partition is clean —
        // the `LayeredScalarField` must produce ≥ 2 distinct per-tet
        // moduli.
        let triple = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![
                layer(0.005, "ECOFLEX_00_30"),
                layer(0.005, "DRAGON_SKIN_10A"),
                layer(0.005, "DRAGON_SKIN_20A"),
            ],
        };
        let g3 = build_insertion_geometry(&scan, &triple, &[], sdf_target_faces, cell_size_m)
            .expect("three-layer geometry should build");
        eprintln!(
            "three-layer: {} tets, cavity-offset {:.1} mm, outer-offset {:.1} mm",
            g3.n_tets,
            g3.cavity_offset_m * 1e3,
            g3.outer_offset_m * 1e3,
        );
        assert!(g3.n_tets > 0, "three-layer mesh must be non-empty");
        assert_eq!(
            count_inverted(&g3),
            0,
            "three-layer mesh must have no inverted tets"
        );
        let first_mu = g3.mesh.materials().first().map_or(0.0, Yeoh::mu);
        assert!(
            g3.mesh
                .materials()
                .iter()
                .any(|m| (m.mu() - first_mu).abs() > 1e-9),
            "three-layer mesh must carry ≥ 2 distinct per-tet moduli \
             (LayeredScalarField partition)",
        );
    }

    // ── 7.3b.2 — per-tet readout sanity ──────────────────────────────

    /// Hand-built right-handed unit tet for the F-reconstruction tests
    /// — corner vertices at `(0,0,0)`, `(1,0,0)`, `(0,1,0)`, `(0,0,1)`.
    /// `D_rest` is the identity, so for any displacement field `x =
    /// A·X + t` the deformation gradient is exactly `A`.
    fn unit_tet_rest() -> Vec<Vec3> {
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ]
    }

    /// Yeoh material for the F-reconstruction tests — ECOFLEX_00_30
    /// converted via the silicone table, matching the synthetic-ramp
    /// fixture so any future change in silicone-table calibration is
    /// caught here too.
    fn unit_tet_material() -> Yeoh {
        silicone_for_anchor("ECOFLEX_00_30")
            .expect("ECOFLEX_00_30 in the silicone table")
            .to_yeoh()
    }

    /// Undeformed tet ⇒ `F = I`. `Yeoh::first_piola(I)` ⇒ zero stress.
    /// `Yeoh::energy(I)` ⇒ zero strain energy. Principal stretches
    /// ⇒ `[1, 1, 1]`. The bedrock of every other F-reconstruction
    /// assertion.
    #[test]
    fn tet_readout_undeformed_is_identity() {
        let rest = unit_tet_rest();
        let curr = rest.clone();
        let verts: [VertexId; 4] = [0, 1, 2, 3];
        let material = unit_tet_material();
        let readout = tet_readout(verts, &rest, &curr, &material);

        let identity = Matrix3::<f64>::identity();
        let diff = (readout.f - identity).norm();
        assert!(
            diff < 1e-12,
            "undeformed tet must give F = I (got ‖F − I‖ = {diff:.3e})"
        );
        assert!(
            readout.first_piola_frobenius_pa < 1e-6,
            "first-Piola stress at F = I must be zero (got ‖P‖ = {:.3e} Pa)",
            readout.first_piola_frobenius_pa,
        );
        assert!(
            readout.energy_density_j_per_m3.abs() < 1e-9,
            "strain-energy density at F = I must be zero (got {} J/m³)",
            readout.energy_density_j_per_m3,
        );
        for &s in readout.principal_stretches.iter() {
            assert!(
                (s - 1.0).abs() < 1e-12,
                "principal stretches at F = I must all be 1 (got {s})"
            );
        }
    }

    /// Pure rigid translation ⇒ `F = I`. Tests that the
    /// edge-vector construction in [`deformation_gradient`] is
    /// translation-invariant (sanity check on the `D_curr · D_rest⁻¹`
    /// formula).
    #[test]
    fn tet_readout_pure_translation_is_identity() {
        let rest = unit_tet_rest();
        let t = Vec3::new(0.5, -0.3, 1.7);
        let curr: Vec<Vec3> = rest.iter().map(|p| p + t).collect();
        let verts: [VertexId; 4] = [0, 1, 2, 3];
        let f = deformation_gradient(verts, &rest, &curr);
        let diff = (f - Matrix3::<f64>::identity()).norm();
        assert!(
            diff < 1e-12,
            "pure translation must give F = I (got ‖F − I‖ = {diff:.3e})"
        );
    }

    /// Uniaxial stretch by `λ` along x with `1/√λ` on y and z (the
    /// near-incompressible mode). `F = diag(λ, 1/√λ, 1/√λ)`; principal
    /// stretches are those three values (in some order, since SVD is
    /// unordered). The compressible Yeoh material won't be *exactly*
    /// incompressible, but the F-reconstruction itself is purely
    /// kinematic, so the diagonal must match exactly.
    #[test]
    fn tet_readout_uniaxial_stretch_principal_stretches() {
        let lambda = 1.5_f64;
        let trans = 1.0 / lambda.sqrt();
        let a = Matrix3::from_diagonal(&Vec3::new(lambda, trans, trans));
        let rest = unit_tet_rest();
        let curr: Vec<Vec3> = rest.iter().map(|p| a * p).collect();
        let verts: [VertexId; 4] = [0, 1, 2, 3];
        let material = unit_tet_material();
        let readout = tet_readout(verts, &rest, &curr, &material);

        let diff = (readout.f - a).norm();
        assert!(
            diff < 1e-12,
            "uniaxial stretch must give F = diag(λ, 1/√λ, 1/√λ) (got ‖F − A‖ = {diff:.3e})"
        );

        // Singular values appear in some order — sort and compare to
        // the expected sorted vector `[λ, 1/√λ, 1/√λ]`.
        let mut sigma: Vec<f64> = readout.principal_stretches.iter().copied().collect();
        sigma.sort_by(|a, b| b.partial_cmp(a).unwrap());
        let mut expected = vec![lambda, trans, trans];
        expected.sort_by(|a, b| b.partial_cmp(a).unwrap());
        for (got, want) in sigma.iter().zip(expected.iter()) {
            assert!(
                (got - want).abs() < 1e-12,
                "uniaxial-stretch principal stretches mismatch — got {sigma:?}, want {expected:?}"
            );
        }

        // F = A is diagonal with det > 0, so Yeoh::first_piola is
        // diagonal too (Yeoh is isotropic). It is *not* zero (we're
        // not at the natural state). Sanity-check finiteness and
        // tension along the stretched axis.
        assert!(
            readout.first_piola_frobenius_pa.is_finite() && readout.first_piola_frobenius_pa > 0.0,
            "stress must be finite + positive away from F = I (got {} Pa)",
            readout.first_piola_frobenius_pa,
        );
        assert!(
            readout.first_piola[(0, 0)] > 0.0,
            "uniaxial extension ⇒ first-Piola P_xx must be tensile (got {} Pa)",
            readout.first_piola[(0, 0)],
        );
    }

    /// `aggregate_step_readout` over an empty per-tet slice + empty
    /// contact readouts must produce finite zeros — degenerate but
    /// non-panicking.
    #[test]
    fn aggregate_step_readout_empty_is_zeroed() {
        let r = aggregate_step_readout(&[], &[]);
        assert_eq!(r.n_active_contact_pairs, 0);
        assert!(r.contact_force_total_n.norm() < 1e-12);
        assert!((r.contact_force_magnitude_n).abs() < 1e-12);
        assert_eq!(r.max_principal_stretch, 0.0);
        assert_eq!(r.min_principal_stretch, 0.0);
        assert_eq!(r.max_first_piola_frobenius_pa, 0.0);
        assert_eq!(r.mean_strain_energy_density_j_per_m3, 0.0);
    }

    /// `aggregate_step_readout` aggregates per-tet extrema and means
    /// correctly across two hand-built readouts. Independent of the
    /// Yeoh material's calibration — uses synthetic values straight
    /// into the [`TetReadout`] fields.
    #[test]
    fn aggregate_step_readout_aggregates_correctly() {
        let mk = |stretches: [f64; 3], frob: f64, psi: f64| TetReadout {
            f: Matrix3::<f64>::identity(),
            first_piola: Matrix3::<f64>::identity(),
            first_piola_frobenius_pa: frob,
            energy_density_j_per_m3: psi,
            principal_stretches: Vec3::new(stretches[0], stretches[1], stretches[2]),
        };
        let per_tet = vec![
            mk([1.2, 0.9, 0.95], 1.0e5, 1.0),
            mk([1.5, 0.8, 1.0], 2.0e5, 3.0),
        ];
        let readouts: Vec<ContactPairReadout> = vec![
            ContactPairReadout {
                pair: ContactPair::Vertex {
                    vertex_id: 0,
                    primitive_id: 0,
                },
                position: Vec3::zeros(),
                sd: -0.1,
                normal: Vec3::new(0.0, 0.0, 1.0),
                force_on_soft: Vec3::new(0.0, 0.0, 3.0),
            },
            ContactPairReadout {
                pair: ContactPair::Vertex {
                    vertex_id: 1,
                    primitive_id: 0,
                },
                position: Vec3::zeros(),
                sd: -0.1,
                normal: Vec3::new(0.0, 0.0, 1.0),
                force_on_soft: Vec3::new(0.0, 0.0, 5.0),
            },
        ];

        let r = aggregate_step_readout(&per_tet, &readouts);
        assert_eq!(r.n_active_contact_pairs, 2);
        assert!((r.contact_force_total_n.z - 8.0).abs() < 1e-12);
        assert!((r.contact_force_magnitude_n - 8.0).abs() < 1e-12);
        assert!((r.max_principal_stretch - 1.5).abs() < 1e-12);
        assert!((r.min_principal_stretch - 0.8).abs() < 1e-12);
        assert!((r.max_first_piola_frobenius_pa - 2.0e5).abs() < 1e-9);
        assert!((r.mean_strain_energy_density_j_per_m3 - 2.0).abs() < 1e-12);
    }

    /// `compute_tet_readouts` panics on a mismatched `tets` /
    /// `materials` slice length — guards the slice-indexed Yeoh
    /// lookup against silently picking a wrong material.
    #[test]
    #[should_panic(expected = "compute_tet_readouts: tets.len()")]
    fn compute_tet_readouts_mismatched_lengths_panic() {
        let rest = unit_tet_rest();
        let tets = vec![[0_u32, 1, 2, 3]];
        // 0 materials vs 1 tet.
        let materials: Vec<Yeoh> = vec![];
        let _ = compute_tet_readouts(&rest, &rest, &tets, &materials);
    }

    /// Build an icosphere `IndexedMesh` of `radius` (meters), centered
    /// at the origin, refined `subdivisions` times (0 = the bare
    /// 20-face icosahedron; 3 = 1280 faces). Smooth, convex, closed —
    /// `mesh_sdf` signs are reliable on it and there is no apex stress
    /// concentration, so it is the well-conditioned synthetic stand-in
    /// for the 7.2 single-step solve test. Subdivision midpoints are
    /// not deduplicated; `decimate_for_sdf`'s vertex weld handles that.
    fn icosphere(radius: f64, subdivisions: usize) -> IndexedMesh {
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let corners: [[f64; 3]; 12] = [
            [-1.0, phi, 0.0],
            [1.0, phi, 0.0],
            [-1.0, -phi, 0.0],
            [1.0, -phi, 0.0],
            [0.0, -1.0, phi],
            [0.0, 1.0, phi],
            [0.0, -1.0, -phi],
            [0.0, 1.0, -phi],
            [phi, 0.0, -1.0],
            [phi, 0.0, 1.0],
            [-phi, 0.0, -1.0],
            [-phi, 0.0, 1.0],
        ];
        let mut mesh = IndexedMesh::new();
        for c in &corners {
            let p = Vector3::new(c[0], c[1], c[2]).normalize() * radius;
            mesh.vertices.push(Point3::from(p));
        }
        let mut faces: Vec<[u32; 3]> = vec![
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        for _ in 0..subdivisions {
            let mut next: Vec<[u32; 3]> = Vec::with_capacity(faces.len() * 4);
            for &[a, b, c] in &faces {
                let pa = mesh.vertices[a as usize].coords;
                let pb = mesh.vertices[b as usize].coords;
                let pc = mesh.vertices[c as usize].coords;
                let mut push_mid = |p: Vector3<f64>, q: Vector3<f64>| -> u32 {
                    let m = ((p + q) / 2.0).normalize() * radius;
                    let idx = u32::try_from(mesh.vertices.len())
                        .expect("icosphere vertex count fits u32");
                    mesh.vertices.push(Point3::from(m));
                    idx
                };
                let ab = push_mid(pa, pb);
                let bc = push_mid(pb, pc);
                let ca = push_mid(pc, pa);
                next.push([a, ab, ca]);
                next.push([b, bc, ab]);
                next.push([c, ca, bc]);
                next.push([ab, bc, ca]);
            }
            faces = next;
        }
        mesh.faces = faces;
        mesh
    }

    /// [`run_single_insertion_step`] on a well-conditioned synthetic
    /// device — a spherical-shell device with a slightly-smaller
    /// sphere intruder.
    ///
    /// 7.2 proves the solver + contact + BC + config *wiring* on a
    /// benign geometry: a smooth convex icosphere has reliable
    /// `mesh_sdf` signs, no apex stress concentration, and uniform
    /// radial contact — none of the real iter-1 scan's contact-
    /// robustness pitfalls. The real scan's single-step solve does
    /// *not* converge (non-PD pivots + Armijo stall, residual that
    /// does not scale with interference); hardening contact for the
    /// real scan is 7.3's battle — see the slice-7 memo's "7.2
    /// real-scan finding".
    ///
    /// `#[ignore]` — a release-mode FEM solve, too slow under a debug
    /// `cargo test`. Self-contained (no fixture). Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design insertion_sim -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "release-mode FEM solve — slow under debug; run with --release --ignored"]
    fn run_single_insertion_step_on_synthetic_sphere() {
        // 40 mm icosphere "scan"; a chunky 10 mm single-layer wall
        // (well-conditioned — ~2.5 BCC cells across) inset 3 mm.
        let scan = icosphere(0.040, 3);
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        let geometry = build_insertion_geometry(&scan, &design, &[], 2_000, 0.004)
            .expect("synthetic-sphere geometry should build");
        let n_tets = geometry.n_tets;
        // Rest positions captured before the solver consumes the mesh.
        let rest: Vec<f64> = geometry
            .mesh
            .positions()
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();

        // 0.5 mm interference — small, but real contact on a clean
        // convex geometry.
        let interference_m = 0.0005;
        let step = run_single_insertion_step(geometry, interference_m)
            .expect("synthetic single insertion step should converge");
        eprintln!(
            "synthetic single-step solve: {n_tets} tets, {} pinned, \
             {} Newton iters, residual {:.2e}",
            step.n_pinned, step.iter_count, step.final_residual_norm,
        );

        assert_eq!(
            step.x_final.len(),
            rest.len(),
            "x_final must cover every rest DOF",
        );
        assert!(
            step.x_final.iter().all(|v| v.is_finite()),
            "every converged DOF must be finite",
        );
        assert!(
            step.n_pinned > 0,
            "the outer skin must have pinned vertices to react against",
        );
        // The solve must do physics — penalty contact at a real
        // interference moves at least one DOF off its rest position.
        let max_disp = rest
            .iter()
            .zip(&step.x_final)
            .map(|(r, f)| (f - r).abs())
            .fold(0.0, f64::max);
        assert!(
            max_disp > 0.0,
            "a {interference_m} m interference solve should displace at least one DOF",
        );
    }

    /// **7.3a fix characterization** — `run_single_insertion_step` on
    /// the real iter-1 scan, post-`GridSdf` wire-in.
    ///
    /// 7.2's single-step solve did *not* converge on the real scan
    /// (non-PD pivots + Armijo stall, residual ~6e4 *not scaling with
    /// interference*); the 7.3a diagnostic root-caused it to
    /// `mesh_sdf`'s ~12%-wrong sign on the sloppy-decimated scan. The
    /// 7.3a fix swapped `build_insertion_geometry` onto the flood-fill
    /// [`GridSdf`] — and it works: the residual collapses from ~6e4
    /// into the ~0.1 regime (0.5 mm interference → 0.157 at iter 38).
    /// The geometry/SDF problem is solved.
    ///
    /// What it does *not* yet do is reach `tol = 1e-10`: the residual
    /// stalls near the solution on a non-SPD tangent (the capsule
    /// geometry's secondary pathology) — `replay_step` panics rather
    /// than return a non-converged step. Closing that last mile (the
    /// quasi-static ramp's warm-starting, `tol` / `kappa` tuning) is
    /// **7.3b**. So this is a `catch_unwind` *characterization*
    /// harness, not a pass/fail test: it asserts the geometry builds
    /// and reports the solve outcome — a regression guard that the
    /// GridSdf wire-in keeps the real scan in the convergeable regime,
    /// and a ready harness for 7.3b to measure progress against.
    ///
    /// `#[ignore]` — needs the iter-1 fixture + a release-mode solve.
    #[test]
    #[ignore = "7.3a fix characterization — needs the iter-1 scan + a release solve; run with --ignored"]
    fn iter1_single_step_solve_characterization() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }
        let scan = load_stl(&path).expect("load the iter-1 cleaned scan");

        // 10 mm single-layer wall (well-conditioned — ~2.5 BCC cells
        // across, the same as the converging synthetic case).
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        let geometry = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("iter-1 geometry should build on the GridSdf");
        let n_tets = geometry.n_tets;
        eprintln!("iter-1 geometry: {n_tets} tets (built on the flood-fill GridSdf)");

        // `replay_step` panics on non-convergence — catch it so this
        // characterization harness reports rather than fails. 7.3b
        // turns this into a hard pass/fail once the ramp + tuning
        // close the last mile to `tol`.
        let interference_m = 0.0005;
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            run_single_insertion_step(geometry, interference_m)
        }));
        match outcome {
            Ok(Ok(step)) => eprintln!(
                "  CONVERGED at {interference_m} m — {} pinned, {} Newton iters, \
                 residual {:.2e}",
                step.n_pinned, step.iter_count, step.final_residual_norm,
            ),
            Ok(Err(e)) => eprintln!("  errored (not a panic): {e:#}"),
            Err(payload) => {
                let msg = payload
                    .downcast_ref::<String>()
                    .map(String::as_str)
                    .or_else(|| payload.downcast_ref::<&str>().copied())
                    .unwrap_or("<non-string panic payload>");
                eprintln!(
                    "  did not reach tol at {interference_m} m (expected at 7.3a — the \
                     near-solution non-SPD tangent is 7.3b's ramp + tuning):\n    {msg}"
                );
            }
        }
    }

    /// **7.3a diagnostic spike** — why does the real iter-1 scan's
    /// single-step solve fail? (7.2 finding: non-PD pivots + Armijo
    /// stall, residual ~6e4 that does *not* scale with interference.)
    ///
    /// Four phases, all *reporting* (no hard asserts beyond "the
    /// fixture loaded") — this is a measurement harness, not a
    /// regression test:
    ///
    /// 1. **Topology audit** — `validate_mesh` + components on the raw
    ///    scan, the current decimated SDF source, and a repair-pass
    ///    candidate. `mesh_sdf`'s closest-face-normal sign is only
    ///    reliable on a watertight + manifold + consistently-wound
    ///    mesh.
    /// 2. **SDF sign cross-check** — `distance()` sign (closest-face
    ///    normal, what `Sdf::eval` feeds the contact + BCs) vs
    ///    `is_inside()` (ray cast). Disagreement = unreliable sign.
    /// 3. **Controlled thick-wall experiment** — re-run the solve on
    ///    the real scan with a 10 mm wall (vs 7.2's 5 mm). Isolates
    ///    the thin-wall conditioning confound from the geometry/SDF.
    /// 4. **STL exports** — the decimated SDF source + the meshed
    ///    device-wall body, for eyes-on review in a mesh viewer.
    ///
    /// `#[ignore]` — needs the iter-1 fixture; run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design diagnose_iter1 -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "7.3a diagnostic — needs the repo-excluded iter-1 scan; run with --ignored --nocapture"]
    #[allow(clippy::cast_precision_loss)] // diagnostic counters → f64 for %/grid math
    fn diagnose_iter1_scan_geometry() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }
        let raw = load_stl(&path).expect("load the iter-1 cleaned scan");
        eprintln!("\n=== 7.3a diagnostic — {} ===", path.display());

        // ── Phase 1 — topology audit ────────────────────────────────
        let audit = |label: &str, m: &IndexedMesh| {
            let r = validate_mesh(m);
            let c = find_connected_components(m);
            eprintln!(
                "  [{label}]\n    {} faces / {} verts | watertight={} manifold={} \
                 inside_out={}\n    boundary_edges={} non_manifold_edges={} \
                 degenerate={} duplicate={} | components={}",
                r.face_count,
                r.vertex_count,
                r.is_watertight,
                r.is_manifold,
                r.is_inside_out,
                r.boundary_edge_count,
                r.non_manifold_edge_count,
                r.degenerate_face_count,
                r.duplicate_face_count,
                c.component_count,
            );
        };
        eprintln!(
            "\nPHASE 1 — topology audit (mesh_sdf sign needs watertight + manifold + \
             consistent winding):"
        );
        audit("raw scan", &raw);
        let decimated = decimate_for_sdf(&raw, 2_500);
        audit("decimated @2500 — the current SDF source", &decimated);
        // Candidate 7.3a fix: does a basic mesh-repair pass clean it up?
        let mut repaired = decimated.clone();
        let _ = fix_winding_order(&mut repaired);
        let _ = repair_mesh(&mut repaired, &RepairParams::for_scans());
        audit(
            "decimated + fix_winding_order + repair_mesh(for_scans)",
            &repaired,
        );

        // ── Phase 2 — mesh_sdf sign cross-check ─────────────────────
        eprintln!(
            "\nPHASE 2 — mesh_sdf sign cross-check (closest-face-normal `distance()` vs \
             ray-cast `is_inside()`):"
        );
        let sdf_distance =
            TriMeshDistance::new(decimated.clone()).expect("decimated scan TriMeshDistance");
        let sdf_sign = PseudoNormalSign::from_distance(&sdf_distance);
        let sdf = Signed {
            distance: sdf_distance,
            sign: sdf_sign,
        };
        let bbox = scan_aabb(&decimated, 0.005);
        let n = 18_usize;
        let (mut total, mut disagree, mut near_total, mut near_disagree) = (0, 0, 0, 0);
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    let axis =
                        |i: usize, lo: f64, hi: f64| lo + (hi - lo) * (i as f64 + 0.5) / n as f64;
                    let p = Point3::new(
                        axis(ix, bbox.min.x, bbox.max.x),
                        axis(iy, bbox.min.y, bbox.max.y),
                        axis(iz, bbox.min.z, bbox.max.z),
                    );
                    let d = sdf.distance(p);
                    let mismatched = (d < 0.0) != sdf.is_inside(p);
                    total += 1;
                    disagree += usize::from(mismatched);
                    // Near-surface points are the contact-relevant ones.
                    if d.abs() < 0.008 {
                        near_total += 1;
                        near_disagree += usize::from(mismatched);
                    }
                }
            }
        }
        let pct = |num: usize, den: usize| {
            if den == 0 {
                0.0
            } else {
                100.0 * num as f64 / den as f64
            }
        };
        eprintln!(
            "  {disagree}/{total} grid points disagree ({:.1}%) | near-surface (|d|<8mm): \
             {near_disagree}/{near_total} ({:.1}%)",
            pct(disagree, total),
            pct(near_disagree, near_total),
        );
        // Centroid sanity: a closed blob's vertex centroid must read inside.
        let centroid = {
            let mut c = Vector3::zeros();
            for v in &decimated.vertices {
                c += v.coords;
            }
            Point3::from(c / decimated.vertices.len() as f64)
        };
        eprintln!(
            "  vertex centroid: distance()={:.4} m (expect < 0), is_inside()={} (expect true)",
            sdf.distance(centroid),
            sdf.is_inside(centroid),
        );

        // ── Phase 3 — controlled thick-wall solve experiment ────────
        eprintln!(
            "\nPHASE 3 — controlled thick-wall solve experiment (real scan, 10 mm wall — \
             7.2 failed at 5 mm / ~1.25 cells):"
        );
        let design_10mm = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        match build_insertion_geometry(&raw, &design_10mm, &[], 2_500, 0.004) {
            Ok(geometry) => {
                eprintln!("  geometry: {} tets", geometry.n_tets);
                let outcome = catch_unwind(AssertUnwindSafe(|| {
                    run_single_insertion_step(geometry, 0.0002)
                }));
                match outcome {
                    Ok(Ok(step)) => eprintln!(
                        "  CONVERGED — {} pinned, {} Newton iters, residual {:.2e}\n  \
                         → the thin wall was the 7.2 confound",
                        step.n_pinned, step.iter_count, step.final_residual_norm,
                    ),
                    Ok(Err(e)) => eprintln!("  errored (not a panic): {e:#}"),
                    Err(_) => eprintln!(
                        "  PANICKED (Newton non-convergence) — a thicker wall does NOT fix \
                         it; the geometry/SDF is the culprit"
                    ),
                }
            }
            Err(e) => eprintln!("  geometry build failed: {e:#}"),
        }

        // ── Phase 4 — STL exports for eyes-on review ────────────────
        eprintln!("\nPHASE 4 — STL exports (open in a mesh viewer):");
        let out_dir = std::env::temp_dir().join("cf_device_design_diag");
        std::fs::create_dir_all(&out_dir).expect("create diag output dir");

        let decimated_path = out_dir.join("decimated_scan.stl");
        save_stl(&decimated, &decimated_path, true).expect("save decimated scan STL");
        eprintln!(
            "  {} — the SDF source; check for holes / flipped faces / stray components",
            decimated_path.display(),
        );

        let design_5mm = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        match build_insertion_geometry(&raw, &design_5mm, &[], 2_500, 0.004) {
            Ok(geometry) => {
                let mut surface = IndexedMesh::new();
                surface.vertices = geometry
                    .mesh
                    .positions()
                    .iter()
                    .map(|p| Point3::from(*p))
                    .collect();
                surface.faces = geometry.mesh.boundary_faces().to_vec();
                let body_path = out_dir.join("device_wall_body_5mm.stl");
                save_stl(&surface, &body_path, true).expect("save device-wall body STL");
                eprintln!(
                    "  {} — the meshed 5 mm device wall ({} boundary faces); should be a \
                     clean closed shell, not islands/holes",
                    body_path.display(),
                    surface.faces.len(),
                );
            }
            Err(e) => eprintln!("  body export skipped — geometry build failed: {e:#}"),
        }
        eprintln!("\n=== end 7.3a diagnostic ===\n");
    }

    /// **7.3a fix spike** — does the flood-fill [`GridSdf`] fix the
    /// ~12% `mesh_sdf` sign error the 7.3a diagnostic root-caused?
    ///
    /// Builds a `GridSdf` of the iter-1 scan at a sweep of grid
    /// resolutions and reports flood-fill health + sign correctness:
    ///
    /// - **inside_components == 1** — a limb is one solid blob; more
    ///   means the flood leaked through a hole (grid too coarse).
    /// - **spot checks** — the vertex centroid must read inside, the
    ///   eight bbox corners outside.
    /// - **sign vs the legacy methods** — where `distance()` (closest-
    ///   face normal) and `is_inside()` (ray cast) *agree* (the
    ///   confident ~88%), `GridSdf` should agree too; the ~12% they
    ///   *dispute* is exactly what `GridSdf` resolves.
    ///
    /// The actual solve-convergence proof is the 7.3a wire-in
    /// (sub-commit 2). `#[ignore]` — needs the iter-1 fixture; run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design grid_sdf_fix_spike -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "7.3a fix spike — needs the repo-excluded iter-1 scan; run with --ignored --nocapture"]
    #[allow(clippy::cast_precision_loss)] // diagnostic counters → f64 for %/grid math
    fn grid_sdf_fix_spike() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }
        let raw = load_stl(&path).expect("load the iter-1 cleaned scan");
        let decimated = decimate_for_sdf(&raw, 2_500);
        let bbox = scan_aabb(&decimated, 0.010);
        let legacy_distance =
            TriMeshDistance::new(decimated.clone()).expect("decimated scan TriMeshDistance");
        let legacy_sign = PseudoNormalSign::from_distance(&legacy_distance);
        let legacy = Signed {
            distance: legacy_distance,
            sign: legacy_sign,
        };

        // Spot-check probes: the vertex centroid (must read inside) +
        // the eight bbox corners (must read outside).
        let centroid = {
            let mut c = Vector3::zeros();
            for v in &decimated.vertices {
                c += v.coords;
            }
            Point3::from(c / decimated.vertices.len() as f64)
        };
        let corners: Vec<Point3<f64>> = {
            let (lo, hi) = (bbox.min, bbox.max);
            let mut cs = Vec::with_capacity(8);
            for &x in &[lo.x, hi.x] {
                for &y in &[lo.y, hi.y] {
                    for &z in &[lo.z, hi.z] {
                        cs.push(Point3::new(x, y, z));
                    }
                }
            }
            cs
        };

        eprintln!("\n=== 7.3a fix spike — flood-fill GridSdf ===");
        for grid_cell_m in [0.004, 0.003, 0.002] {
            let wall_threshold_m = 0.75 * grid_cell_m;
            let (grid_sdf, report) =
                build_grid_sdf(&decimated, bbox, grid_cell_m, wall_threshold_m)
                    .expect("grid SDF builds");
            eprintln!(
                "\n  grid {:.1} mm — dims {:?}, {} ms\n    {} outside / {} inside / {} wall \
                 | inside_components={} (1 = healthy)",
                report.grid_cell_m * 1e3,
                report.dims,
                report.build_ms as u64,
                report.n_outside,
                report.n_inside,
                report.n_wall,
                report.inside_components,
            );

            // Spot checks.
            let centroid_d = grid_sdf.eval(centroid);
            let corners_outside = corners.iter().all(|&c| grid_sdf.eval(c) > 0.0);
            eprintln!(
                "    centroid eval={centroid_d:.4} m (expect < 0) | all 8 corners outside: \
                 {corners_outside}"
            );
            assert!(
                centroid_d < 0.0,
                "vertex centroid must read inside the scan"
            );
            assert!(
                corners_outside,
                "every bbox corner must read outside the scan"
            );

            // Sign vs the legacy methods over a sample grid.
            let samples = 16_usize;
            let (mut confident, mut confident_agree, mut disputed) = (0, 0, 0);
            for ix in 0..samples {
                for iy in 0..samples {
                    for iz in 0..samples {
                        let axis = |i: usize, lo: f64, hi: f64| {
                            lo + (hi - lo) * (i as f64 + 0.5) / samples as f64
                        };
                        let p = Point3::new(
                            axis(ix, bbox.min.x, bbox.max.x),
                            axis(iy, bbox.min.y, bbox.max.y),
                            axis(iz, bbox.min.z, bbox.max.z),
                        );
                        let grid_inside = grid_sdf.eval(p) < 0.0;
                        let legacy_dist_inside = legacy.distance(p) < 0.0;
                        let legacy_ray_inside = legacy.is_inside(p);
                        if legacy_dist_inside == legacy_ray_inside {
                            confident += 1;
                            if grid_inside == legacy_dist_inside {
                                confident_agree += 1;
                            }
                        } else {
                            disputed += 1;
                        }
                    }
                }
            }
            let agree_pct = 100.0 * confident_agree as f64 / confident as f64;
            eprintln!(
                "    sign vs legacy: agrees with {confident_agree}/{confident} confident \
                 points ({agree_pct:.1}%) | resolves {disputed} disputed (the ~12%)"
            );
        }
        eprintln!("\n=== end 7.3a fix spike ===\n");
    }

    /// [`run_insertion_ramp`] on the well-conditioned synthetic
    /// icosphere — slice 7.3c (Gaussian pre-smooth on the GridSdf
    /// signed buffer) seats the intruder to the **full 3 mm inset**
    /// in all 16 ramp steps. Pre-7.3c (FD-on-unsmoothed-trilinear) was
    /// 14 / 16 (≈ 2.62 mm) with a contact-side Armijo stall at the
    /// 15th step; post-7.3c the contact side is no longer the binding
    /// constraint and the ramp completes cleanly. The regression
    /// assertion is therefore `== 16 steps` (full depth). If a future
    /// change re-introduces the contact wall, this test fires.
    ///
    /// `#[ignore]` — a release-mode multi-step solve. Self-contained
    /// (no fixture). Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     --bin cf-device-design run_insertion_ramp_on_synthetic \
    ///     -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "release-mode multi-step solve — slow under debug; run with --release --ignored"]
    fn run_insertion_ramp_on_synthetic_sphere() {
        let scan = icosphere(0.040, 3);
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        let geometry = build_insertion_geometry(&scan, &design, &[], 2_000, 0.004)
            .expect("synthetic-sphere geometry should build");
        let n_dof = geometry.mesh.positions().len() * 3;

        // 16 steps — small increments so each step's warm start lands
        // close (full-surface contact reshuffles the whole contact set
        // per increment, so coarse steps cost as much as a cold solve).
        let n_steps = 16;
        let n_tets = geometry.n_tets;
        let ramp = run_insertion_ramp(geometry, n_steps).expect("synthetic ramp should run");
        for s in &ramp.steps {
            eprintln!(
                "  step interference {:.2} mm — {} Newton iters, residual {:.2e} \
                 — contact F = {:.2} N over {} pairs — λ ∈ [{:.3}, {:.3}] — max ‖P‖ = {:.2e} Pa",
                s.interference_m * 1e3,
                s.iter_count,
                s.final_residual_norm,
                s.readout.contact_force_magnitude_n,
                s.readout.n_active_contact_pairs,
                s.readout.min_principal_stretch,
                s.readout.max_principal_stretch,
                s.readout.max_first_piola_frobenius_pa,
            );
        }
        if let Some(k) = ramp.failed_at_step {
            eprintln!(
                "  stalled at step {k}: {}",
                ramp.failure_reason.as_deref().unwrap_or("<no reason>"),
            );
        }

        // Slice 7.3c: Gaussian pre-smooth on the GridSdf signed buffer
        // lifts the synthetic envelope from 14 / 16 (pre-fix Armijo
        // stall) to the full 16 / 16. The regression assertion pins
        // *full depth*; if a future change re-introduces a contact
        // wall, the assert fires.
        assert_eq!(
            ramp.steps.len(),
            n_steps,
            "the synthetic ramp must reach the full {n_steps}-step depth \
             (slice 7.3c — Gaussian pre-smooth); got only {}",
            ramp.steps.len(),
        );
        assert!(
            ramp.failed_at_step.is_none(),
            "slice 7.3c: full-depth ramp must not stall — got failure at step {:?}",
            ramp.failed_at_step,
        );
        assert!(
            ramp.n_pinned > 0,
            "the outer skin must have pinned vertices"
        );
        // Interference ramps strictly monotonically.
        for pair in ramp.steps.windows(2) {
            assert!(
                pair[1].interference_m > pair[0].interference_m,
                "ramp interference must increase each step",
            );
        }
        assert_eq!(ramp.final_x.len(), n_dof, "final_x covers every DOF");
        assert!(
            ramp.final_x.iter().all(|v| v.is_finite()),
            "every final DOF must be finite",
        );

        // Slice 7.3b.2 — `InsertionResult` + per-step `StepReadout`
        // contracts. The synthetic ramp is the canonical regression
        // floor for these too: full convergence ⇒ `result.is_some()`,
        // per-tet detail covers every tet, principal stretches finite
        // and well-bounded, force-displacement curve is monotone
        // non-decreasing.
        for (k, s) in ramp.steps.iter().enumerate() {
            assert_eq!(
                s.x_final.len(),
                n_dof,
                "step {k} x_final must cover every DOF (got {} of {n_dof})",
                s.x_final.len(),
            );
            assert!(
                s.x_final.iter().all(|v| v.is_finite()),
                "step {k} x_final must be all-finite",
            );
            let r = &s.readout;
            assert!(
                r.contact_force_magnitude_n.is_finite() && r.contact_force_magnitude_n >= 0.0,
                "step {k} contact_force_magnitude_n must be finite + non-negative \
                 (got {})",
                r.contact_force_magnitude_n,
            );
            assert!(
                r.max_principal_stretch.is_finite() && r.min_principal_stretch.is_finite(),
                "step {k} principal stretches must be finite \
                 (got [{:.3}, {:.3}])",
                r.min_principal_stretch,
                r.max_principal_stretch,
            );
            assert!(
                r.min_principal_stretch >= 0.0,
                "step {k} min_principal_stretch must be non-negative (got {})",
                r.min_principal_stretch,
            );
            assert!(
                r.max_principal_stretch >= r.min_principal_stretch,
                "step {k} max ≥ min principal stretch (got [{:.3}, {:.3}])",
                r.min_principal_stretch,
                r.max_principal_stretch,
            );
            assert!(
                r.max_first_piola_frobenius_pa.is_finite() && r.max_first_piola_frobenius_pa >= 0.0,
                "step {k} max ‖P‖ must be finite + non-negative",
            );
            assert!(
                r.mean_strain_energy_density_j_per_m3.is_finite()
                    && r.mean_strain_energy_density_j_per_m3 >= 0.0,
                "step {k} mean Ψ must be finite + non-negative \
                 (Yeoh energy is non-negative at det F > 0)",
            );
        }

        // The first ramp step (the smallest interference) should
        // record a non-zero contact force — the intruder has crossed
        // the cavity wall and the penalty contact is engaged. A zero
        // here would mean the contact band missed the cavity surface.
        assert!(
            ramp.steps
                .first()
                .is_some_and(|s| s.readout.contact_force_magnitude_n > 0.0),
            "first step must have a non-zero contact force (the contact penalty is engaged)",
        );

        // Force-displacement should rise meaningfully across the
        // ramp on a convex synthetic geometry. Per-step monotonicity
        // is *not* asserted: at `INSERTION_SOLVE_TOL = 1e-1` (the
        // tuned Fork-B physically-negligible bar), successive
        // converged residuals carry ~0.1 N of solver-tolerance
        // noise, so the per-step F-d curve has ~10-15 % jitter on
        // top of the monotone underlying trend. The end-to-end
        // rise — first vs last — is what the contract pins; the
        // visible per-step trace above already exposes any
        // pathological drop for eyes-on review. Yeoh is a
        // monotone-stiffening polynomial (the C₂(I₁−3)² term is
        // convex in stretch), so an "underlying" non-monotone curve
        // would indicate a contact-side regression, not a material
        // one.
        let f_first = ramp.steps[0].readout.contact_force_magnitude_n;
        let f_last = ramp
            .steps
            .last()
            .map(|s| s.readout.contact_force_magnitude_n)
            .unwrap_or(0.0);
        assert!(
            f_last > 2.0 * f_first,
            "synthetic ramp F-d must rise meaningfully end-to-end \
             (got first = {f_first:.3} N, last = {f_last:.3} N — \
             expected ≥ 2× growth across the full 3 mm seating)",
        );

        let result = ramp
            .result
            .as_ref()
            .expect("full-depth ramp must populate InsertionResult");
        assert_eq!(
            result.final_per_tet.len(),
            n_tets,
            "InsertionResult per-tet detail must cover every tet",
        );
        assert_eq!(
            result.force_displacement_curve.len(),
            ramp.steps.len(),
            "F-d curve length matches converged step count",
        );
        // Final-step per-tet detail must be all-finite and within the
        // Yeoh material's calibrated principal-stretch envelope —
        // ECOFLEX_00_30's `max_principal_stretch ≈ 6` (Smooth-On TDS
        // elongation-at-break × 0.8 calibration), `min_principal_
        // stretch ≈ 0.30`. A finite-out-of-bound result indicates a
        // material-side regression even if the ramp converged.
        let ecoflex = silicone_for_anchor("ECOFLEX_00_30").unwrap().to_yeoh();
        let validity = Material::validity(&ecoflex);
        // FP rounding floor for the non-negativity assertions —
        // Yeoh energy at det F > 0 is mathematically non-negative,
        // but the polynomial expansion has cancelling terms that can
        // drift a hair negative at near-rest tets (interior, lightly
        // strained — `½μ(I₁−3) − μ·ln_j` cancels to ~`O(μ · ε²)` at
        // |F − I| → 0, hitting `f64` ULP noise). `-1e-3 J/m³` is
        // ≫ any plausible rounding floor for ECOFLEX-class material
        // and ≪ any physically meaningful energy density at the
        // ramp's converged depth (mean Ψ is `O(10² – 10³) J/m³`).
        let psi_floor = -1.0e-3;
        for (t, tr) in result.final_per_tet.iter().enumerate() {
            assert!(
                tr.first_piola_frobenius_pa.is_finite(),
                "tet {t} ‖P‖ at final step must be finite (got {})",
                tr.first_piola_frobenius_pa,
            );
            assert!(
                tr.energy_density_j_per_m3.is_finite() && tr.energy_density_j_per_m3 >= psi_floor,
                "tet {t} Ψ at final step must be finite + ≥ {psi_floor} J/m³ (got {})",
                tr.energy_density_j_per_m3,
            );
            for &s in tr.principal_stretches.iter() {
                assert!(
                    s.is_finite() && s > 0.0,
                    "tet {t} stretch must be finite + positive (got {s})"
                );
                if let Some(cap) = validity.max_principal_stretch {
                    assert!(
                        s <= cap,
                        "tet {t} principal stretch {s:.3} exceeds Yeoh validity cap {cap:.3}",
                    );
                }
                if let Some(floor) = validity.min_principal_stretch {
                    assert!(
                        s >= floor,
                        "tet {t} principal stretch {s:.3} below Yeoh validity floor {floor:.3}",
                    );
                }
            }
        }
    }

    /// **7.3b.1 payoff** — [`run_insertion_ramp`] on the real iter-1
    /// scan.
    ///
    /// Ladder of envelopes (each on top of the prior commit):
    /// - **pre-7.3c** baseline: 5 / 16 (≈ 0.94 mm, 31 %), contact-side
    ///   Armijo stall — recon-iter-3 baseline `c05c2eb8`.
    /// - **slice 7.3c** (Gaussian pre-smooth σ = 0.5 cell): 12 / 16
    ///   (≈ 2.25 mm, 75 %), what looked like a Yeoh stretch-validity
    ///   stop — `47806a37`.
    /// - **slice 7.3d** (σ retuning to 1.0 cell): **16 / 16
    ///   (3.00 mm, 100 %)**, full depth, max 5 Newton iters per step
    ///   — recon-iter-4 (`<this commit>`).
    ///
    /// Recon-iter-4 discovered that what slice 7.3c read as a
    /// material-side wall was numerical: σ = 0.5 cell smoothing was
    /// too narrow to suppress the sharp local features that
    /// concentrated bad-Jacobian stress into specific tets, and the
    /// `max_stretch_deviation ≤ 1.0` validity check fired on those
    /// tets *before* the actual material limit. σ = 1.0 unlocks
    /// past the false wall.
    ///
    /// Reports every step and asserts the ramp *mechanics* (monotonic
    /// interference, finite `final_x`, ≥ 1 step). The exact converged-
    /// step count is *not* asserted — the iter-1 scan is repo-excluded
    /// and exact-depth assertions on it would be brittle. The
    /// synthetic test pins the full-depth contract for the regression
    /// floor; this test characterizes the real-scan behavior.
    ///
    /// `#[ignore]` — needs the iter-1 fixture + a release-mode ramp.
    #[test]
    #[ignore = "7.3b.1 payoff — needs the iter-1 scan + a release ramp; run with --ignored --nocapture"]
    fn run_insertion_ramp_on_iter1_scan() {
        let path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !path.exists() {
            eprintln!("skip: iter-1 scan fixture not found at {}", path.display());
            return;
        }
        let scan = load_stl(&path).expect("load the iter-1 cleaned scan");
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        // `sdf_target_faces = 2 500` per the slice 7.0 spike's "tet
        // quality is governed by cell_size, not SDF face count"
        // finding. The recon-iter-3 N2 experiment (`f981a442`) showed
        // 25 000 faces *alone* doubles the iter-1 envelope (31 → 62 %)
        // — but **the combo with the slice 7.3c Gaussian smooth
        // regresses to 56 %**: at 25 k faces the scan-capture noise
        // amplitude (rotating-table artifacts) is binding, and the
        // σ = 0.5 cell smoothing tuned for the 2 500-face proxy's
        // polyhedral kinks is too weak to suppress it. A clean N2
        // follow-up requires σ retuning — recon-iter-4 territory if
        // and when slice 8/9 surfaces a deeper-envelope requirement.
        // `sdf_target_faces = 2 500` per the slice 7.0 spike's "tet
        // quality is governed by cell_size, not SDF face count"
        // finding. The recon-iter-3 N2 experiment (`f981a442`) showed
        // 25 000 faces *alone* doubles the iter-1 envelope (31 → 62 %)
        // — but **the combo with the slice 7.3c Gaussian smooth
        // regresses to 56 %**: at 25 k faces the scan-capture noise
        // amplitude (rotating-table artifacts) is binding, and the
        // σ = 0.5 cell smoothing tuned for the 2 500-face proxy's
        // polyhedral kinks is too weak to suppress it. Recon-iter-4
        // (`<this commit>`) retested at σ = 1.0 cell and the combo
        // *still* regresses to 14/16 (vs N3-alone-at-σ=1.0's full
        // 16/16): faithful 25 k-face resolution exposes scan noise
        // even when σ is widened. The 2 500-face proxy + σ = 1.0
        // cell is the empirically-best operating point.
        let geometry = build_insertion_geometry(&scan, &design, &[], 2_500, 0.004)
            .expect("iter-1 geometry should build");
        let n_tets = geometry.n_tets;
        let n_dof = geometry.mesh.positions().len() * 3;

        let n_steps = 16;
        let ramp = run_insertion_ramp(geometry, n_steps).expect("iter-1 ramp should run");
        eprintln!(
            "iter-1 ramp — {n_tets} tets, {} pinned, {n_steps} requested steps:",
            ramp.n_pinned
        );
        for s in &ramp.steps {
            eprintln!(
                "  interference {:.2} mm — {} Newton iters, residual {:.2e}",
                s.interference_m * 1e3,
                s.iter_count,
                s.final_residual_norm,
            );
        }
        match ramp.failed_at_step {
            None => eprintln!("  → converged all {n_steps} steps to the full 3 mm inset"),
            Some(k) => eprintln!(
                "  → stalled at step {k} (interference {:.2} mm) — warm-starting got \
                 {} steps in; the rest needs 7.3b's tol / kappa tuning",
                (k + 1) as f64 / n_steps as f64 * 3.0,
                ramp.steps.len(),
            ),
        }

        // Ramp mechanics — true regardless of how far convergence got.
        assert!(
            !ramp.steps.is_empty(),
            "the ramp must converge at least one step"
        );
        assert!(
            ramp.n_pinned > 0,
            "the outer skin must have pinned vertices"
        );
        for pair in ramp.steps.windows(2) {
            assert!(
                pair[1].interference_m > pair[0].interference_m,
                "ramp interference must increase each step",
            );
        }
        assert_eq!(ramp.final_x.len(), n_dof, "final_x covers every DOF");
        assert!(
            ramp.final_x.iter().all(|v| v.is_finite()),
            "every final DOF must be finite",
        );
    }

    /// **H4.3 sweep** — sliding-intruder ramps at cavity =
    /// 3, 5, 6, 7, 8 mm on iter-1 sock_over_capsule with the iter-1
    /// GUI-default 10+3 mm dual-layer stack (ECOFLEX_00_30 outer
    /// 10 mm + DRAGON_SKIN_20A inner 3 mm) + the H4-plumbed
    /// calibrated principal-stretch bounds. Pre-H4 cavity > 5 mm
    /// fake-converged step 1 into an invalid Yeoh state and
    /// panicked at step 2 (per
    /// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10) — or,
    /// post-commit `2739717e` end-of-solve check, panics honestly
    /// at step 1. DRAGON_SKIN_20A's calibrated
    /// `0.8 · λ_break = 5.76` tensile cap is well above the σ ≈ 2.05
    /// peak the E.b sweep measured, so H4 expected to clear
    /// cavity > 5 mm cleanly.
    ///
    /// Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     h4_sweep_sliding_ramp_on_iter1_scan -- --ignored --nocapture
    /// ```
    ///
    /// `#[ignore]` — needs the repo-excluded iter-1 scan + prep.toml
    /// + a release-mode 5-cavity ramp; total wall-clock ~10-15 min.
    #[test]
    #[ignore = "H4.3 sweep — needs the iter-1 scan + a release ramp; run with --ignored --nocapture"]
    fn h4_sweep_sliding_ramp_on_iter1_scan() {
        let scan_path = std::env::var("CF_DEVICE_DESIGN_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if !scan_path.exists() {
            eprintln!(
                "skip: iter-1 scan fixture not found at {}",
                scan_path.display()
            );
            return;
        }
        let prep_path = PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.prep.toml");
        let prep_text =
            std::fs::read_to_string(&prep_path).expect("load iter-1 prep.toml for centerline");
        let centerline =
            crate::parse_centerline(&prep_text).expect("parse centerline from prep.toml");
        assert!(
            centerline.len() >= 2,
            "iter-1 prep.toml must carry a centerline polyline",
        );
        let scan = load_stl(&scan_path).expect("load the iter-1 cleaned scan");

        let cavities_mm = [3.0_f64, 5.0, 6.0, 7.0, 8.0];
        let n_steps = 16_usize;
        let cell_size_m = 0.004_f64;

        eprintln!(
            "H4.3 sweep — iter-1 sock_over_capsule.cleaned.stl, dual-layer \
             DS20A inner 3 mm + Ecoflex 00-30 outer 10 mm, n_steps = {}, \
             centerline {} points",
            n_steps,
            centerline.len(),
        );
        eprintln!(
            "  DS20A bounds: max_principal_stretch = {:.2}, min = {:.2}",
            DRAGON_SKIN_20A.validity_max_principal_stretch,
            DRAGON_SKIN_20A.validity_min_principal_stretch,
        );
        eprintln!(
            "  Ecoflex 00-30 bounds: max_principal_stretch = {:.2}, min = {:.2}",
            ECOFLEX_00_30.validity_max_principal_stretch,
            ECOFLEX_00_30.validity_min_principal_stretch,
        );

        for &cavity_mm in &cavities_mm {
            let cavity_inset_m = cavity_mm * 1e-3;
            eprintln!("\n--- cavity = {cavity_mm:.1} mm ---");
            let design = SimDesign {
                cavity_inset_m,
                // `SimDesign.layers` is innermost-first; iter-1
                // GUI default the prior C′.a + E.b baselines used.
                layers: vec![
                    layer(0.003, "DRAGON_SKIN_20A"),
                    layer(0.010, "ECOFLEX_00_30"),
                ],
            };
            let t0 = Instant::now();
            let geometry = match build_insertion_geometry(&scan, &design, &[], 2_500, cell_size_m) {
                Ok(g) => g,
                Err(e) => {
                    eprintln!("  build_insertion_geometry FAILED: {e}");
                    continue;
                }
            };
            eprintln!(
                "  geometry: {} tets, {} vertices, built in {:.1}s",
                geometry.n_tets,
                geometry.mesh.positions().len(),
                t0.elapsed().as_secs_f64(),
            );

            let t1 = Instant::now();
            let ramp =
                match run_sliding_insertion_ramp(geometry, &centerline, n_steps, cavity_inset_m) {
                    Ok(r) => r,
                    Err(e) => {
                        eprintln!("  run_sliding_insertion_ramp FAILED: {e}");
                        continue;
                    }
                };
            let elapsed = t1.elapsed().as_secs_f64();
            let n_converged = ramp.steps.len();
            let last = ramp.steps.last();
            eprintln!("  ramp: {n_converged}/{n_steps} steps converged in {elapsed:.1}s",);
            for (k, s) in ramp.steps.iter().enumerate() {
                eprintln!(
                    "    step {k:2} t={:.3} arc={:5.2}mm — {} iters, r={:.2e}, F={:6.2}N, \
                     pairs={}",
                    s.slide_fraction_t,
                    s.arc_length_s_m * 1e3,
                    s.iter_count,
                    s.final_residual_norm,
                    s.readout.contact_force_magnitude_n,
                    s.readout.n_active_contact_pairs,
                );
            }
            if let Some(k) = ramp.failed_at_step {
                let reason = ramp.failure_reason.as_deref().unwrap_or("<no reason>");
                eprintln!("  STALL at step {k}: {reason}");
                let _ = last; // result-borrow placeholder; failure branch
            } else {
                eprintln!("  → ALL {n_steps} STEPS CONVERGED ✓");
            }
        }
    }

    /// **Recon discriminating experiment (post-7.3b.1)** — isolate
    /// "full-surface contact period" from "GridSdf gradient roughness"
    /// per `docs/INSERTION_SIM_STATE.md` Q3.
    ///
    /// Mirrors [`run_insertion_ramp_on_synthetic_sphere`] one-for-one
    /// (κ = 1e3, tol = 1e-1, n_steps = 16, single-layer ECOFLEX_00_30,
    /// 10 mm wall, cell_size 4 mm, cavity_inset 3 mm) except: the
    /// intruder + body are built from `Solid::sphere` CSG directly —
    /// **no `GridSdf` anywhere on either side**. Compare the stall
    /// depth here against the icosphere baseline (which stalls at step
    /// 13 / 16 ≈ 2.62 mm). The only axis of variation is "analytical
    /// SDF vs `GridSdf` of an icosphere stand-in".
    ///
    /// Two outcomes:
    /// - Reaches step 16 (full 3 mm) → `GridSdf` gradient roughness is
    ///   a material contributor to the stall; a finer grid (or moving
    ///   to an analytical / smoothed SDF where possible) unlocks
    ///   envelope.
    /// - Stalls at similar depth (≈ 2.6 mm) → full-surface penalty
    ///   contact IS the wall regardless of SDF kind, and the
    ///   formulation must change (Dirichlet hybrid, augmented
    ///   Lagrangian, etc.) to push past it. The recon expects this
    ///   outcome.
    ///
    /// `#[ignore]` — release-mode multi-step solve, no fixture. Self-
    /// contained: does NOT go through [`build_insertion_geometry`] or
    /// [`run_insertion_ramp`] (both are typed on `GridSdf`); the inner
    /// loop is re-implemented here to swap the intruder primitive.
    /// Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     run_insertion_ramp_on_analytical_sphere_shell \
    ///     -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "recon discriminating experiment — release-mode ramp; run with --ignored --nocapture"]
    fn run_insertion_ramp_on_analytical_sphere_shell() {
        // Match the synthetic-icosphere ramp parameters one-for-one;
        // the only varying axis is "analytical SDF vs GridSdf".
        let r_intruder = 0.040; // 40 mm — same as icosphere(0.040, 3)
        let cavity_inset_m = 0.003;
        let wall_m = 0.010;
        let cell_size_m = 0.004;
        let n_steps = 16usize;

        // Analytical SDFs. The "scan" surface sits at radius
        // `r_intruder`; the cavity surface is inset inward by
        // `cavity_inset_m`; the outer envelope is offset outward by
        // `wall_m - cavity_inset_m`. body = outer ⊖ cavity (closed
        // silicone shell, same wall thickness as the icosphere case).
        let intruder_sdf = Solid::sphere(r_intruder);
        let cavity = Solid::sphere(r_intruder).offset(-cavity_inset_m);
        let outer = Solid::sphere(r_intruder).offset(wall_m - cavity_inset_m);
        let body = outer.clone().subtract(cavity);

        // Bounds large enough to contain the outer envelope plus a
        // cell of slack — same posture as `build_insertion_geometry`.
        let outer_r = r_intruder + wall_m - cavity_inset_m;
        let half = outer_r + cell_size_m;
        let bounds = Aabb::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        );

        // Single-layer ECOFLEX_00_30, `ConstantField` path — matches
        // the icosphere case's effective material distribution
        // (`layer_boundary_thresholds` is empty for a single layer →
        // `layered_param_field` returns a `ConstantField` too).
        // Mirrors `build_insertion_geometry`'s switch to the
        // calibrated 5-arg `from_yeoh_fields_with_bounds`
        // constructor (H4 plumbing) so per-tet `Yeoh`s carry
        // ECOFLEX_00_30's 8.00 tensile + 0.30 compressive caps
        // rather than the legacy `max_stretch_deviation = 1.0`
        // fallback — keeps the analytical-sphere recon
        // experiment apples-to-apples with the production
        // GridSdf geometry.
        let silicone = silicone_for_anchor("ECOFLEX_00_30").unwrap();
        let material_field = MaterialField::from_yeoh_fields_with_bounds(
            Box::new(ConstantField::new(silicone.mu)),
            Box::new(ConstantField::new(silicone.c2)),
            Box::new(ConstantField::new(silicone.lambda)),
            Box::new(ConstantField::new(silicone.validity_max_principal_stretch)),
            Box::new(ConstantField::new(silicone.validity_min_principal_stretch)),
        );

        let hints = MeshingHints {
            bbox: aabb3_for_meshing(&bounds),
            cell_size: cell_size_m,
            material_field: Some(material_field),
        };
        let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints)
            .expect("analytical sphere-shell should mesh");
        let n_tets = mesh.n_tets();
        let n_vertices = mesh.n_vertices();
        let n_dof = 3 * n_vertices;

        // Pin the outer-skin vertices: those within `0.5 * cell_size`
        // of the analytical outer envelope. Filter to solver-
        // referenced vertices (BCC may produce orphans outside the
        // body) — same posture as `outer_skin_bc`.
        let band_tol = 0.5 * cell_size_m;
        let referenced: BTreeSet<VertexId> = referenced_vertices(&mesh).into_iter().collect();
        let pinned: Vec<VertexId> =
            pick_vertices_by_predicate(&mesh, |p| outer.eval(Point3::from(*p)).abs() < band_tol)
                .into_iter()
                .filter(|v| referenced.contains(v))
                .collect();
        assert!(
            !pinned.is_empty(),
            "the analytical outer-envelope pin-band must catch at least one vertex"
        );
        let n_pinned = pinned.len();
        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            loaded_vertices: Vec::new(),
        };

        let config = insertion_solver_config();
        let mut x_prev_flat: Vec<f64> = mesh
            .positions()
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        let v_prev = Tensor::zeros(&[n_dof]);
        let empty_theta: [f64; 0] = [];
        let theta = Tensor::from_slice(&empty_theta, &[0]);

        eprintln!(
            "analytical sphere-shell ramp — {n_tets} tets, {n_pinned} pinned, \
             {n_steps} requested steps:"
        );

        let mut steps_converged = 0usize;
        let mut failed_at_step: Option<usize> = None;
        let mut failure_reason: Option<String> = None;
        for k in 0..n_steps {
            #[allow(clippy::cast_precision_loss)]
            let interference_m = (k + 1) as f64 / n_steps as f64 * cavity_inset_m;
            // Analytical contact intruder: `Solid::sphere` offset back
            // toward / past the cavity wall, mirroring
            // `intruder_contact_at`'s `interference + cavity_offset`
            // (cavity_offset = -cavity_inset_m).
            let contact_intruder = intruder_sdf.clone().offset(interference_m - cavity_inset_m);
            let contact = PenaltyRigidContact::with_params(
                vec![contact_intruder],
                INSERTION_CONTACT_KAPPA,
                INSERTION_CONTACT_DHAT,
            );
            let solver = CpuNewtonSolver::new(Tet4, mesh.clone(), contact, config, bc.clone());
            let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
            let outcome = catch_unwind(AssertUnwindSafe(|| {
                solver.replay_step(&x_prev, &v_prev, &theta, config.dt)
            }));
            match outcome {
                Ok(step) => {
                    eprintln!(
                        "  step interference {:.2} mm — {} Newton iters, residual {:.2e}",
                        interference_m * 1e3,
                        step.iter_count,
                        step.final_residual_norm,
                    );
                    x_prev_flat = step.x_final;
                    steps_converged += 1;
                }
                Err(payload) => {
                    failed_at_step = Some(k);
                    failure_reason = Some(panic_message(&*payload));
                    break;
                }
            }
        }

        match failed_at_step {
            None => eprintln!(
                "  → converged all {n_steps} steps to the full {:.2} mm inset",
                cavity_inset_m * 1e3,
            ),
            Some(k) => {
                #[allow(clippy::cast_precision_loss)]
                let depth_mm = (k as f64) / n_steps as f64 * cavity_inset_m * 1e3;
                eprintln!(
                    "  → stalled at step {k} (last converged depth ~{depth_mm:.2} mm); \
                     reason: {}",
                    failure_reason.as_deref().unwrap_or("<no reason>"),
                );
            }
        }

        // The setup is wired correctly: at least one step converged
        // (a panic at step 0 would mean the mesh + BC + contact are
        // ill-posed; nothing about the stall hypothesis to learn).
        assert!(
            steps_converged >= 1,
            "the analytical-sphere ramp must converge at least one step \
             (else the experiment is mis-wired)"
        );
        assert_eq!(x_prev_flat.len(), n_dof);
        assert!(x_prev_flat.iter().all(|v| v.is_finite()));
    }

    // ─── SL.2 — sliding-intruder ramp solver-only fixture ──────────
    //
    // Per `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §4 SL.2 row +
    // §5 gate (4): the synthetic-icosphere fixture validates FEM
    // convergence + the central locality assertion BEFORE the UI
    // plumbing at SL.3. Body, centerline, and asserts share names
    // prefixed `sliding_insertion_ramp_` so the spec's filter
    // (`cargo test ... sliding_insertion_ramp_tests`) catches them.

    /// Straight centerline along `+Z` spanning the icosphere body
    /// diameter — `index 0` (TIP) at the `-Z` apex, `last` (FLOOR /
    /// cap mouth) at the `+Z` apex. Arc length = `2 * radius = 0.080 m`
    /// for the standard `icosphere(0.040, 3)` fixture.
    fn synthetic_icosphere_centerline_z() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, -0.040), // tip (rest pose)
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.040), // floor / cap mouth
        ]
    }

    /// Build a sliding-ramp fixture: `icosphere(40 mm, 3)` body + 3 mm
    /// cavity inset + 10 mm `ECOFLEX_00_30` layer + ONE cap plane at
    /// the `+Z` apex. Shared by the heavyweight tests below.
    ///
    /// The cap plane is load-bearing: without it, `build_insertion_
    /// geometry` short-circuits the open-body SDF to equal the closed-
    /// body SDF and the cavity has no mouth — the sliding intruder
    /// then crashes the cavity-wall material into the Yeoh validity
    /// wall as it approaches full coincidence at `t = 1`. With a cap
    /// plane at `+Z`, `dome_wall_only_mesh` strips the icosphere's
    /// `+Z`-apex triangles, `pinned_floor_shell` opens the cavity at
    /// that plane, and the intruder enters cleanly along the `+Z`
    /// centerline through the open mouth — matching the product
    /// pipeline shape (cf-scan-prep records caps in `.prep.toml`,
    /// cf-device-design carves the mouth via the same primitive in
    /// `build_insertion_geometry`).
    fn synthetic_sliding_geometry() -> InsertionGeometry {
        let scan = icosphere(0.040, 3);
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        let cap = CapPlane {
            centroid: Point3::new(0.0, 0.0, 0.040),
            normal: Vector3::new(0.0, 0.0, 1.0),
            vertex_count: 0,
            loop_index: 0,
        };
        build_insertion_geometry(&scan, &design, &[cap], 2_000, 0.004)
            .expect("synthetic-sphere geometry should build for the sliding fixture")
    }

    /// A `< 2`-point centerline cannot be walked — the sliding ramp
    /// MUST error closed before consuming the geometry, so the caller
    /// sees a clean failure instead of an opaque inner panic.
    #[test]
    fn sliding_insertion_ramp_rejects_degenerate_centerline() {
        let geometry = synthetic_sliding_geometry();
        let single_point = vec![Point3::new(0.0, 0.0, 0.0)];
        // `SlideRamp` intentionally omits `Debug` (same posture as
        // `InsertionRamp`'s flat-Vec footgun) so `expect_err` is
        // unavailable; assert `is_err` then destructure with a guarded
        // `let-else` (cheaper than a bare `panic!` which trips the
        // crate-wide `clippy::panic` deny).
        let result = run_sliding_insertion_ramp(geometry, &single_point, 8, 3.0e-3);
        assert!(result.is_err(), "single-point centerline must be rejected");
        let Err(err) = result else { unreachable!() };
        let msg = format!("{err:#}");
        assert!(
            msg.contains("centerline polyline of ≥ 2 points"),
            "error message must surface the polyline-length requirement, got: {msg}",
        );
    }

    /// CR.3 fast unit gate: a probe deep inside the closed-body
    /// GridSdf (where flood-fill sign returns far-negative) must be
    /// silently excluded from the active set by the `interior_cutoff`
    /// filter — independent of the FEM solve. Pins the CR.2 wire-up:
    /// `intruder_contact_sliding_at` must call
    /// `PenaltyRigidContact::with_params_and_interior_cutoff` (NOT
    /// `with_params`); regression to `with_params` would produce a
    /// non-empty active set here.
    ///
    /// **Scope**: this test is a wire-up gate, NOT a filter-math gate.
    /// The strict-vs-non-strict, sign-convention, and band-gate
    /// semantics of the cutoff filter itself are pinned in
    /// `sim/L0/soft/tests/penalty_interior_cutoff.rs` against a
    /// `RigidPlane` fixture with precisely-controllable sd. Here the
    /// icosphere `GridSdf` is the load-bearing surface — we're only
    /// asserting that cf-device-design's contact-build call site
    /// actually plumbs the cutoff through to sim-soft.
    ///
    /// Probe at `(0, 0, 0)` with `pose = slide_pose_at(centerline, 1.0)`
    /// (rest pose — intruder centered at origin per the centerline's
    /// rest-pose convention): the inverse-transformed point lands at
    /// the body center where the icosphere GridSdf's flood-fill sign
    /// reports `sd ≈ -40 mm`. Composed sd via
    /// `Solid::offset(cavity_offset_m = -3 mm)` = `raw + 3 mm ≈
    /// -37 mm`. The `2 × cavity_inset_m = 6 mm` interior_cutoff
    /// filters at composed `< -6 mm`, so this probe is excluded.
    #[test]
    fn intruder_contact_sliding_at_excludes_deep_interior_probe() {
        use sim_soft::ActivePairsFor;
        let geometry = synthetic_sliding_geometry();
        let centerline = synthetic_icosphere_centerline_z();
        let pose = slide_pose_at(&centerline, 1.0);
        let contact = intruder_contact_sliding_at(
            &geometry.intruder,
            geometry.bounds,
            pose,
            0.0, // interference_m — pre-F4 shrunk-scan model (the
            // wire-up gate is independent of the F4 homotopy knob)
            -0.003, // cavity_offset_m
            3.0e-3, // cavity_inset_m → 6 mm interior_cutoff
        );
        let probe = vec![Vec3::new(0.0, 0.0, 0.0)];
        let pairs = contact.active_pairs(&geometry.mesh, &probe);
        assert!(
            pairs.is_empty(),
            "deep-interior probe at body center (composed sd ≈ -37 mm) must be \
             excluded by the 6 mm interior_cutoff; got {} pairs",
            pairs.len(),
        );
    }

    /// At `t = 0` the rigid intruder is translated all the way from
    /// rest (centered at the origin) to centered at the FLOOR end of
    /// the centerline (`+Z` apex of the sphere). For the synthetic
    /// icosphere fixture (closed body, not a cup), the intruder
    /// overlaps the body's upper-bbox region by construction — the
    /// "zero active pairs at t=0" intent in the spec applies to cup
    /// geometries (real iter-1 sock_over_capsule has an open mouth),
    /// not to a closed sphere. What we CAN test here is the slide-
    /// pose direction: active contact must be in the upper-bbox
    /// region only; the LOWER hemisphere of the body should see no
    /// active contact pairs at `t = 0`. A sign-flipped inverse inside
    /// [`TransformedSdf`] would surface here as spurious lower-half
    /// contact (the intruder would appear to be at `-Z` instead of
    /// `+Z`).
    #[test]
    fn sliding_insertion_ramp_at_t_eq_0_has_no_lower_hemisphere_contact() {
        let geometry = synthetic_sliding_geometry();
        let centerline = synthetic_icosphere_centerline_z();

        let pose = slide_pose_at(&centerline, 0.0);
        let contact = intruder_contact_sliding_at(
            &geometry.intruder,
            geometry.bounds,
            pose,
            0.0, // interference_m — pre-F4 shrunk-scan model (the
            // sign-of-pose gate is independent of the F4 homotopy knob)
            -0.003, // cavity_offset_m = -cavity_inset_m
            3.0e-3, // cavity_inset_m
        );
        let rest_positions: Vec<Vec3> = geometry.mesh.positions().to_vec();
        let readouts = contact.per_pair_readout(&geometry.mesh, &rest_positions);
        let n_lower_half_active = readouts.iter().filter(|r| r.position.z < -0.005).count();
        assert_eq!(
            n_lower_half_active, 0,
            "intruder at t=0 (translated +Z) must NOT fire active contact in \
             the lower hemisphere — sign-flipped inverse would surface here",
        );
    }

    /// F3 recon B candidate C′.a (ε bisection) — pin
    /// [`INSERTION_CONTACT_SMOOTHING_EPS_M`] at the post-C′.a-sweep
    /// chosen value (`0.075 mm`) + carry the C′.a evidence table.
    ///
    /// **C.2 + C′.a combined sweep** (2026-05-18, cavity = 5 mm,
    /// layers 10+3 mm, sock_over_capsule.cleaned.stl, per
    /// `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md`):
    ///
    /// | ε (mm) | steps converged | r_norm floor | stall mode | LM rescues |
    /// |---|---|---|---|---|
    /// | 0 (gated-A baseline) | 0/16 | 1.784 | Armijo iter 61 | 1 mild |
    /// | 0.025 (C′.a) | 0/16 | 0.231 | Armijo iter 108 | 3 stiff |
    /// | 0.05 (C′.a) | 0/16 | 0.200 | Armijo iter 147 | 4 moderate |
    /// | **0.075 (C′.a)** | **16/16** | converges seated 83.35 mm | — | **0** |
    /// | 0.1 (C.2) | 0/16 | 0.384 | Armijo iter 126 | 2 stiff |
    /// | 0.25 (C.2) | 0/16 | 0.753 | iter cap 150 | 2 stiff |
    ///
    /// U-shaped response with a **narrow converging window centered
    /// at ε ≈ 0.075 mm**.  Sweet spot where band-widening backfire
    /// (hyp 3 from falsification bookmark) balances against
    /// chattering-suppression effect.  No LM rescues + no Yeoh
    /// failures + no panics at the chosen ε.
    ///
    /// MAINTENANCE NOTE: this pinned value + the sweep table mirror
    /// the docstring on [`INSERTION_CONTACT_SMOOTHING_EPS_M`].  If
    /// the const value changes, update both surfaces in lockstep.
    #[test]
    fn insertion_contact_smoothing_eps_m_sentinel() {
        // 0.075 mm = 7.5e-5 m; use a tolerance well below the
        // ε-bisection step (0.025 mm sample spacing → 2.5e-5 m).
        let expected = 0.075e-3;
        assert!(
            (INSERTION_CONTACT_SMOOTHING_EPS_M - expected).abs() < 1e-9,
            "INSERTION_CONTACT_SMOOTHING_EPS_M expected {expected} m \
             (C′.a sweep pinned at 0.075 mm — narrow converging \
             window 2026-05-18 per \
             docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md); got \
             {INSERTION_CONTACT_SMOOTHING_EPS_M}. If you changed the \
             value to test a recon candidate, also update the sweep \
             evidence comment AND the const docstring.",
        );
        // Must remain non-negative + finite so any future
        // re-pinning routes through C.1's smoothing surface cleanly.
        assert!(
            INSERTION_CONTACT_SMOOTHING_EPS_M >= 0.0
                && INSERTION_CONTACT_SMOOTHING_EPS_M.is_finite(),
            "smoothing window must be non-negative + finite",
        );
    }

    /// F3 recon B candidate E.b (per-query normal averaging) —
    /// initial state sentinel: `(k, r) = (1, 0.0)` pins the
    /// bit-equal-when-disabled wire-up. The E.b.4 case-A ship
    /// re-pins these values, updates the assertion expected values,
    /// and mirrors the full `(k, r)` sweep table in the const
    /// docstrings (parallel to C′.a's 3-surface mirror pattern).
    ///
    /// MAINTENANCE NOTE: this sentinel + the
    /// [`INSERTION_CONTACT_NORMAL_AVG_K`] /
    /// [`INSERTION_CONTACT_NORMAL_AVG_RADIUS_M`] docstrings mirror
    /// each other.  If the const values change, update both
    /// surfaces in lockstep (sentinel asserts the new value; const
    /// docstrings carry the sweep evidence).
    #[test]
    fn insertion_contact_normal_avg_sentinel() {
        assert_eq!(
            INSERTION_CONTACT_NORMAL_AVG_K, 1,
            "INSERTION_CONTACT_NORMAL_AVG_K expected 1 (E.b disabled — \
             pending the cavity = 6 mm sweep per \
             docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md §6); got \
             {INSERTION_CONTACT_NORMAL_AVG_K}. If you changed the \
             value to test a recon candidate, also update the const \
             docstring with the sweep evidence + this sentinel's \
             expected value.",
        );
        // f64 equality is intentional + correct: 0.0 is exactly
        // representable + the initial pinned value is exactly 0.0.
        #[allow(clippy::float_cmp)]
        let r_is_zero = INSERTION_CONTACT_NORMAL_AVG_RADIUS_M == 0.0;
        assert!(
            r_is_zero,
            "INSERTION_CONTACT_NORMAL_AVG_RADIUS_M expected 0.0 \
             (matches the disabled k=1 state); got \
             {INSERTION_CONTACT_NORMAL_AVG_RADIUS_M}. If you changed \
             the value to test a recon candidate, also update the \
             const docstring + this sentinel's expected value.",
        );
        // Bounds well-formedness — guards against future re-pinning
        // outside the closed-set k validation in the constructor.
        assert!(
            matches!(INSERTION_CONTACT_NORMAL_AVG_K, 1 | 7),
            "k must be in sim-soft's iter-1 closed set {{1, 7}}",
        );
        assert!(
            INSERTION_CONTACT_NORMAL_AVG_RADIUS_M >= 0.0
                && INSERTION_CONTACT_NORMAL_AVG_RADIUS_M.is_finite(),
            "normal-averaging radius must be non-negative + finite",
        );
        // Composition invariant: k > 1 requires r > 0 to avoid the
        // silent-no-op caller mistake the sim-soft constructor asserts.
        // At the disabled pin (k = 1) the inner assertion's predicate
        // is statically true-or-skipped; #[allow] suppresses clippy's
        // assertion-has-constant-value lint because the gate IS the
        // invariant — when E.b.4 case A re-pins k to 7, this branch
        // becomes the load-bearing assertion.
        #[allow(clippy::assertions_on_constants)]
        if INSERTION_CONTACT_NORMAL_AVG_K > 1 {
            assert!(
                INSERTION_CONTACT_NORMAL_AVG_RADIUS_M > 0.0,
                "normal-averaging radius must be strictly positive when k > 1",
            );
        }
    }

    /// Solver-only FEM-correctness gate for the synthetic icosphere
    /// cup. The fixture is a cup (icosphere with the `+Z` apex carved
    /// open by a cap plane — see `synthetic_sliding_geometry`).
    ///
    /// Two failure modes share this test's coverage; the gates
    /// discriminate them:
    ///
    /// 1. **Pre-v5 SL.3 deep-interior firing** (the bug v5 fixed).
    ///    Closed-body `TransformedSdf<GridSdf>` returned deep-negative
    ///    `sd` for BCC vertices whose inverse-transformed position
    ///    landed in the static intruder body interior, generating
    ///    `κ·(d̂ − sd)·n` forces in the kN range and breaking Newton
    ///    convergence at step 0. v5 (CR.1) `interior_cutoff` filters
    ///    those pairs at the active-set walk; CR.2 wires
    ///    `2 × cavity_inset_m = 6 mm` cutoff into
    ///    `intruder_contact_sliding_at`. Empirical signature post-v5:
    ///    **step-0 contact force ≈ 0.28 N** (vs kN pre-v5). The
    ///    50-N step-0 sentinel below is the sharpest gate against
    ///    regression to this mode.
    ///
    /// 2. **Yeoh validity wall** (genuine, geometry-driven). The
    ///    icosphere has a NARROWING cross-section along the slide
    ///    direction — once the intruder slides past ~15 mm the local
    ///    cavity-wall stretch climbs past `max_stretch_deviation = 1.0`
    ///    and the Phase-4-scope-memo-Decision-Q fail-closed semantics
    ///    fire at `sim-soft/src/solver/backward_euler.rs:625`.
    ///    Empirically (post-v5, observed during CR.3 implementation):
    ///    the ramp converges 3 steps (t = 1/16, 2/16, 3/16; arc 5, 10,
    ///    15 mm) and stalls at step 3 with `max_stretch_deviation ≈
    ///    1.33`. The Fork-B "partial seating is honest engineering
    ///    data" stall handling (D-Slide6) absorbs this gracefully.
    ///
    /// The CR.3 recon spec asserted that the "narrowing cross-section"
    /// framing was wishful — empirical measurement during CR.3
    /// implementation falsified that. The narrowing IS real for this
    /// fixture; what was wishful was the assumption that the SL.3 mode
    /// was the ONLY stall mechanism. The 50-N sentinel handles SL.3
    /// regression; Fork-B handles the genuine Yeoh stall. Real iter-1
    /// `sock_over_capsule` is sock-shaped (tube of roughly constant
    /// radius) and is the natural surface for the all-steps-converge
    /// gate — that's CR.4 (visual gate), not this fixture.
    ///
    /// `n_steps = 16` per spec D-Slide5 default for an 80 mm centerline
    /// (`max(16, ceil(L_m / 5e-3))`).
    ///
    /// `#[ignore]` — release-mode multi-step solve; mirrors the
    /// growing-ramp synthetic test's posture. Run:
    ///
    /// ```text
    /// cargo test -p cf-device-design --release \
    ///     sliding_insertion_ramp_converges -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "release-mode multi-step solve — slow under debug; run with --release --ignored"]
    fn sliding_insertion_ramp_converges_on_synthetic_icosphere() {
        let geometry = synthetic_sliding_geometry();
        let centerline = synthetic_icosphere_centerline_z();
        let n_dof = geometry.mesh.positions().len() * 3;
        let n_steps = 16;

        let ramp = run_sliding_insertion_ramp(geometry, &centerline, n_steps, 3.0e-3)
            .expect("synthetic sliding ramp should run");
        for s in &ramp.steps {
            eprintln!(
                "  step t={:.3} (arc {:5.2} mm) — {} Newton iters, residual {:.2e} \
                 — contact F = {:.2} N over {} pairs",
                s.slide_fraction_t,
                s.arc_length_s_m * 1e3,
                s.iter_count,
                s.final_residual_norm,
                s.readout.contact_force_magnitude_n,
                s.readout.n_active_contact_pairs,
            );
        }
        if let Some(k) = ramp.failed_at_step {
            eprintln!(
                "  stalled at step {k}: {}",
                ramp.failure_reason.as_deref().unwrap_or("<no reason>"),
            );
        }

        // Fork-B contract: at least one step must converge (a step-0
        // panic would mean the contact + BC + mesh are ill-posed). The
        // synthetic icosphere fixture genuinely stalls at the Yeoh
        // validity wall around step 3 due to the narrowing cross-
        // section (see the docstring's mechanism 2). The all-steps-
        // converge gate belongs on a sock-shaped fixture (real iter-1
        // covers that at CR.4 visual gate).
        assert!(
            !ramp.steps.is_empty(),
            "sliding ramp must converge at least one step",
        );
        assert!(
            ramp.n_pinned > 0,
            "the outer skin must have pinned vertices",
        );
        // Slide fraction climbs monotonically; arc length too.
        for pair in ramp.steps.windows(2) {
            assert!(
                pair[1].slide_fraction_t > pair[0].slide_fraction_t,
                "slide fraction must increase each step",
            );
            assert!(
                pair[1].arc_length_s_m > pair[0].arc_length_s_m,
                "arc length must increase each step",
            );
        }
        for (k, s) in ramp.steps.iter().enumerate() {
            assert_eq!(s.x_final.len(), n_dof);
            assert!(
                s.x_final.iter().all(|v| v.is_finite()),
                "step {k} x_final must be all-finite",
            );
            assert!(
                s.readout.contact_force_magnitude_n.is_finite()
                    && s.readout.contact_force_magnitude_n >= 0.0,
                "step {k} contact force must be finite + non-negative \
                 (got {})",
                s.readout.contact_force_magnitude_n,
            );
        }
        assert_eq!(ramp.intruder_poses.len(), ramp.steps.len());

        // Contact-force regression sentinel — step 0 (t = 1/16) has
        // small interference + modest active-pair count; plausible
        // upper bound 50 N (see docstring derivation). Pre-v5 SL.3
        // would report this in the kN range due to the deep-interior
        // firing of body-bulk + orphan vertices. The sentinel directly
        // catches regression to the SL.3 mode even if the all-converge
        // gate above happens to pass for an unrelated reason.
        const STEP_0_CONTACT_FORCE_BOUND_N: f64 = 50.0;
        let step_0_force = ramp.steps[0].readout.contact_force_magnitude_n;
        assert!(
            step_0_force < STEP_0_CONTACT_FORCE_BOUND_N,
            "step-0 contact force {step_0_force:.2} N exceeds plausible \
             upper bound {STEP_0_CONTACT_FORCE_BOUND_N:.0} N — likely \
             regression to the pre-v5 SL.3 deep-interior firing",
        );

        let result = ramp.result.as_ref().expect("at least one step converged");
        assert_eq!(
            result.force_arc_length_curve.len(),
            ramp.steps.len(),
            "force-arc-length curve has one point per converged step",
        );
        // Safety net: the synthetic icosphere stall mode is the Yeoh
        // validity wall (docstring mechanism 2). Any non-validity-
        // wall stall reason is a regression to the SL.3 deep-interior
        // mechanism (mechanism 1) the cutoff is designed to filter,
        // OR an unexpected solver pathology — surface it.
        if let Some(reason) = &ramp.failure_reason {
            assert!(
                reason.contains("validity violation") || reason.contains("stretch"),
                "synthetic-sphere stall must be the Yeoh validity wall (closed-body \
                 overlap → high local stretch); got unexpected reason: {reason}",
            );
        }
    }

    /// FEM-correctness gate: at the LAST converged step of the sliding
    /// ramp (whichever it is — Fork-B partial seating is acceptable per
    /// D-Slide6), referenced vertices in the LOWER hemisphere (well
    /// outside the upper-half contact zone) deform less than `0.5 mm`
    /// from rest. This is the spec §4 SL.2 gate (c) + §5 gate (4)
    /// assertion: sliding contact is LOCAL — far-from-contact regions
    /// stay near rest — distinguishing it from the growing ramp's
    /// UNIFORM offset (which would deform every cavity vertex by
    /// roughly the same `interference_m`).
    ///
    /// Empirically `~3 µm` on the cup fixture, three orders of
    /// magnitude under the bound; a growing-ramp regression would
    /// produce `O(3 mm)` displacement everywhere (uniform offset
    /// across the full cavity), failing the bound by 3 orders.
    ///
    /// `#[ignore]` — release-mode multi-step solve.
    #[test]
    #[ignore = "release-mode multi-step solve — slow under debug; run with --release --ignored"]
    fn sliding_insertion_ramp_localizes_deformation_at_intermediate_step() {
        let geometry = synthetic_sliding_geometry();
        let centerline = synthetic_icosphere_centerline_z();
        let rest_positions: Vec<Vec3> = geometry.mesh.positions().to_vec();
        let referenced: Vec<VertexId> = referenced_vertices(&geometry.mesh);
        let n_steps = 16;

        let ramp = run_sliding_insertion_ramp(geometry, &centerline, n_steps, 3.0e-3)
            .expect("synthetic sliding ramp should run");
        assert!(
            !ramp.steps.is_empty(),
            "ramp must converge at least one step for the locality assertion",
        );

        let last = ramp.steps.last().expect("non-empty steps");
        let positions_k: Vec<Vec3> = positions_from_flat(&last.x_final);
        eprintln!(
            "  locality on last converged step: t={:.3} (arc {:5.2} mm), \
             contact F = {:.2} N over {} pairs",
            last.slide_fraction_t,
            last.arc_length_s_m * 1e3,
            last.readout.contact_force_magnitude_n,
            last.readout.n_active_contact_pairs,
        );

        // Far-from-contact = referenced vertices with rest z < -20 mm
        // (the lower-hemisphere region of the body, well below the
        // upper-half contact zone the sliding intruder sweeps through).
        let mut max_far_displacement_m = 0.0_f64;
        let mut n_far = 0_usize;
        for &vid in &referenced {
            let rest = rest_positions[vid as usize];
            if rest.z >= -0.020 {
                continue;
            }
            n_far += 1;
            let displacement = (positions_k[vid as usize] - rest).norm();
            max_far_displacement_m = max_far_displacement_m.max(displacement);
        }
        eprintln!(
            "  locality: {n_far} far-from-contact referenced vertices, \
             max displacement {:.3} mm",
            max_far_displacement_m * 1e3,
        );
        assert!(
            n_far >= 4,
            "fixture must expose enough lower-hemisphere referenced vertices \
             to be meaningful; got {n_far}",
        );
        // 0.5 mm bound per spec §4 SL.2 gate column — empirically
        // `~3 µm` on the cup fixture (3 orders under), so the bound
        // is sensitive enough to catch a growing-intruder regression
        // (which would produce `O(3 mm)` everywhere).
        assert!(
            max_far_displacement_m < 0.0005,
            "sliding contact must be local — lower-hemisphere displacement {:.3} mm \
             ≥ 0.5 mm bound (would indicate the FEM is propagating deformation \
             uniformly, i.e. a regression to growing-intruder behavior)",
            max_far_displacement_m * 1e3,
        );
    }
}
