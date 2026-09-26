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

//! `insertion_sim` — FEM insertion-simulation pipeline for cf-sim-research.
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
//!   tradeoff. Post parry-accel the distance query itself is
//!   O(log faces) via BVH, but BVH build cost + MC fidelity over
//!   the raw 3.34 M-face scan still favor decimation — the spike
//!   found a low target (~1.5–3k faces) is best.
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
use baby_shark::{
    decimation::{AlwaysDecimate, EdgeDecimator},
    mesh::{corner_table::CornerTableF, traits::TriangleMesh},
};
use cf_cap_planes::{CapPlane, dome_wall_only_mesh};
use cf_design::{Aabb, SdfGrid, Solid, pinned_floor_shell};
use cf_device_types::{SimDesign, SimLayer, SlackerResolution, slacker};
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::{CachedGridSdf, PseudoNormalSign, Signed, TriMeshDistance};
use mesh_types::IndexedMesh;
use nalgebra::{
    Isometry3, Matrix3, Point3, Rotation3, SMatrix, Translation3, UnitQuaternion, Vector3,
};
use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10,
    ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50,
};
use sim_soft::readout::{ConformityParams, ConformityReadout, conformity_breakdown};
use sim_soft::{
    Aabb3, BoundaryConditions, ConstantField, ContactPair, ContactPairReadout, CpuNewtonSolver,
    Element, Field, IpcRigidContact, LayeredScalarField, LmConfig, Material, MaterialField, Mesh,
    MeshingHints, PenaltyRigidContact, Sdf, SdfMeshedTetMesh, ShoreReading, SiliconeMaterial,
    Solver, SolverConfig, SolverFailure, Tet4, Tet10Mesh, TetId, Vec3, VertexId, Yeoh,
    boundary_faces_on_isosurface, face_barrier_kappa, filter_pair_readouts_to_referenced,
    pick_vertices_by_predicate, referenced_vertices,
};

/// Weld epsilon (meters) for the pre-decimation vertex weld — matches
/// `main.rs`'s `ENVELOPE_PROXY_WELD_EPSILON_M`. The cleaned scan's STL
/// load produces 3-per-triangle unshared vertices that meshopt needs
/// welded to find collapsible edges.
const SPIKE_WELD_EPSILON_M: f64 = 1e-6;

/// Decimate the cleaned scan to roughly `target_faces` triangles for
/// SDF construction.
///
/// Separate from `main.rs`'s `compute_envelope_proxy_mesh` (which
/// decimates hard — ~1500 faces — for *viewport* speed): here the
/// face count trades parry BVH build cost + per-BCC-vertex query
/// constant factor against isosurface-landing fidelity (smoothing
/// over fingertip / sliver detail before MC sees it). The 7.0 spike
/// found tet count + element quality are governed by the BCC
/// `cell_size`, *not* the SDF face count, so a low resolution is
/// preferred (see the slice-7 ship log). [`run_sdf_bridge_spike`]
/// sweeps `target_faces` so 7.1 can pick that tradeoff point from
/// measured data.
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
    remove_unreferenced_vertices(&mut welded);

    if welded.faces.len() <= target_faces {
        return welded;
    }

    // baby_shark `EdgeDecimator` with `keep_boundary(false)` ≈ the
    // meshopt `simplify_sloppy_decoder` posture this path used pre-
    // 2026-05-26: sacrifices boundary fidelity to push past
    // topological collapse limits on uncleaned scans (disconnected
    // components, sliver triangles). Switched from meshopt per
    // S1.1 probes 7/8 — meshopt's decimator introduces non-manifold
    // edges + duplicate faces on a manifold input.
    #[allow(clippy::cast_possible_truncation)]
    let positions_f32: Vec<Vector3<f32>> = welded
        .vertices
        .iter()
        .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let flat_indices: Vec<usize> = welded
        .faces
        .iter()
        .flat_map(|tri| [tri[0] as usize, tri[1] as usize, tri[2] as usize])
        .collect();
    let mut ct_mesh = CornerTableF::from_vertex_and_face_slices(&positions_f32, &flat_indices);

    let mut decimator: EdgeDecimator<f32, AlwaysDecimate> = EdgeDecimator::default()
        .decimation_criteria(AlwaysDecimate)
        .min_faces_count(Some(target_faces))
        .keep_boundary(false);
    decimator.decimate(&mut ct_mesh);

    let mut out = IndexedMesh::new();
    let mut vid_to_dense: std::collections::HashMap<_, u32> = std::collections::HashMap::new();
    for vid in <CornerTableF as TriangleMesh>::vertices(&ct_mesh) {
        let pos = <CornerTableF as TriangleMesh>::position(&ct_mesh, vid);
        let dense_idx = out.vertices.len() as u32;
        out.vertices.push(Point3::new(
            f64::from(pos[0]),
            f64::from(pos[1]),
            f64::from(pos[2]),
        ));
        vid_to_dense.insert(vid, dense_idx);
    }
    for face_vids in <CornerTableF as TriangleMesh>::faces(&ct_mesh) {
        // Invariant: each `face_vids[i]` was emitted by `vertices()`
        // above, so the map lookup always succeeds. Use `if let` over
        // `expect` to satisfy the crate's `expect_used = deny` lint
        // without changing behavior — the no-match arm is unreachable
        // under the documented baby_shark TriangleMesh contract.
        if let (Some(&a), Some(&b), Some(&c)) = (
            vid_to_dense.get(&face_vids[0]),
            vid_to_dense.get(&face_vids[1]),
            vid_to_dense.get(&face_vids[2]),
        ) {
            out.faces.push([a, b, c]);
        }
    }
    out
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
    /// dominant cost. One parry-BVH SDF query per BCC lattice vertex,
    /// ×2 for the `outer.subtract(scan)` composition.
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
/// Propagates [`TriMeshDistance::new`] (empty mesh, no positive-area
/// triangle to derive an internal scale from, or a face naming a
/// missing vertex) and
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
    // sim-soft rows 21–25 layered-sleeve precedent. The composed
    // `Signed<TriMeshDistance, _>` is `Clone` and the decimated mesh
    // is small, so cloning it for the two CSG operands is cheap.
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

// `SimLayer`, `SimDesign`, `SlackerResolution` live in
// `cf-device-types` so both this binary and (Phase 2+) cf-sim-research
// project a layered-silicone device the same way. See
// `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` §3 A1 Phase 1.

/// Resolve an anchor key from `cf_device_types::LAYER_MATERIALS` to
/// the sim-soft [`SiliconeMaterial`] it mirrors.
///
/// The eight keys are a closed catalog — an unrecognized key is a
/// wiring bug, surfaced as an error rather than silently substituted.
/// (`cf_device_types::material_density` *does* fall back defensively,
/// but a wrong *modulus* would quietly corrupt the sim, not just a
/// mass readout — so the sim path fails loud instead.)
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
            "unrecognized silicone anchor key {other:?} — not in the cf-device-types silicone catalog"
        )),
    }
}

/// Resolve a layer's effective `SiliconeMaterial` from
/// `(anchor_key, slacker_fraction)` — slice 7.5's Slacker → sim-modulus
/// wiring.
///
/// **Resolution table** (driven by `slacker::support`):
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
    let support = slacker::support(&layer.anchor_key);
    let curve = match support {
        slacker::Support::Curve(c) => c,
        // The recipe panel disables the picker for these two
        // variants, so a non-zero fraction reaching here is a wiring
        // surprise — fall back to base rather than fail the sim.
        slacker::Support::NotRecommended | slacker::Support::NoData => {
            return Ok((base, SlackerResolution::Base));
        }
    };
    // The UI snaps `slacker_fraction` to the curve's tabulated points
    // (`cf_device_types::resolve_slacker_fraction`). Linear search
    // through ≤ 5 points; binary-search overhead would be noise.
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
        slacker::ShoreScale::A => {
            let mat = SiliconeMaterial::from_effective_shore(ShoreReading::A(shore_points), None)
                .map_err(|e| {
                anyhow!("sim-soft from_effective_shore(Shore A {shore_points}) failed: {e:?}")
            })?;
            Ok((mat, SlackerResolution::Interpolated))
        }
        slacker::ShoreScale::OO => {
            let mat = SiliconeMaterial::from_effective_shore(
                ShoreReading::DoubleZero(shore_points),
                None,
            )
            .map_err(|e| {
                anyhow!("sim-soft from_effective_shore(Shore 00 {shore_points}) failed: {e:?}")
            })?;
            Ok((mat, SlackerResolution::Interpolated))
        }
        slacker::ShoreScale::OOO => {
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
    ///
    /// This is the level that defines Γ, the intended contact surface: the
    /// cavity wall is the scan isosurface at this offset.
    pub cavity_offset_m: f64,
    /// Ultimate tensile strength (Pa) of the **innermost** layer — the one
    /// that actually contacts the intruder.
    ///
    /// Read as the conformity reward's peak-pressure ceiling `p_max`. Layers
    /// accumulate outward from the cavity (see [`layer_boundary_thresholds`]),
    /// so `design.layers[0]` is the contacting material.
    ///
    /// ⚠ `NaN` when the innermost layer's material was built by
    /// `SiliconeMaterial::from_measured`, which cannot know tensile strength.
    /// The peak term then reports `NaN` and `score_with` drops it rather than
    /// scoring against an invented ceiling.
    pub cavity_tensile_strength_pa: f64,
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
    // Flood-fill `GridSdf`, not parry pseudo-normal sign: the
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
    // `build_material_field` precedent.  The 5-arg constructor
    // `from_yeoh_fields_with_bounds` threads the per-anchor
    // `0.8 · λ_break` tensile cap + `0.20` compressive cap into
    // `MaterialFieldInner::Yeoh.bounds`; `MaterialField::sample_yeoh`
    // routes each per-tet `Yeoh` through
    // `with_max_principal_stretch_only` per H4-2-C — only the
    // tensile cap reaches the solver's
    // `check_validity_at_step_start` gate, the compressive value
    // is sampled then dropped (preserved in `bounds` for future
    // Option B / Phase H F-bar re-enable per
    // `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5).  `det F > 0`
    // inversion is the only remaining compressive safety net.
    // Pre-H4 3-arg path used the legacy `max_stretch_deviation`
    // gate (symmetric σ ∈ [0, 2]) — H4-2-C matches that behavior
    // on the compressive side while adding the calibrated tensile
    // cap.
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

    // Layers accumulate outward from the cavity, so layer 0 is the material
    // that actually touches the intruder — its tensile strength is the ceiling
    // the conformity reward's peak barrier diverges at.
    let cavity_tensile_strength_pa = match design.layers.first() {
        Some(inner) => silicone_for_anchor(&inner.anchor_key)?.tensile_strength_pa,
        None => f64::NAN,
    };

    Ok(InsertionGeometry {
        mesh,
        // The scan SDF doubles as the rigid intruder — the press-fit
        // ramp (7.2) drives this into the cavity.
        intruder: scan_sdf,
        cavity_offset_m,
        cavity_tensile_strength_pa,
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
/// needs: cf-sim-research is a *relative-comparison* engineering aid
/// (Fork B), and a `0.1`-N out-of-balance residual is physically
/// negligible against the tens-of-newtons contact forces.
///
/// `1e-1` was not arbitrary when it was chosen — the 7.3b.1 finding
/// was that the deeper ramp steps Armijo-*stall* (non-SPD tangent near
/// the solution) at a residual floor right around `0.1 N`, and putting
/// `tol` at that floor converted those stalls into convergences, which
/// is what let the ramp seat the intruder to a meaningful depth.
///
/// ⚠ **Three claims this docstring used to make are contradicted by
/// measurement, and are corrected here rather than left standing.**
/// Measured 2026-09-22 at `a0cfa901`, by re-running all three
/// `#[ignore]`d ramps at `tol` = 1e-6:
///
/// 1. It said `0.1 N` is *"physically negligible against the
///    tens-of-newtons contact forces"*. The synthetic ramp's contact
///    force is **0.18 N rising to 0.67 N** — so the tolerance is
///    between 15 % and 56 % of the total contact force it is being
///    called negligible against, not a small fraction of tens of
///    newtons.
/// 2. It said the stall floor sits *"right around `0.1 N`"*. That was
///    true before the slice-7.3d Gaussian pre-smooth
///    ([`GRID_SDF_SMOOTH_SIGMA_CELLS`]). After it, **both synthetic
///    ramps reach 16/16 at `tol` = 1e-6**; only the real scan stalls,
///    and at `r_norm` **4.13e-3**, not 0.1.
/// 3. It said the shallow steps *"converge far below this (to
///    ~`1e-5`)"*. Measured, they land around **1e-2**.
///
/// ⛔ So "converged" at this tolerance is **not** the
/// "loose-but-physically-exact" answer the original wording promised.
/// On the same scene the solver reaches **2.286e-7** when asked for
/// 1e-6 and returns **5.339e-2** when asked for `1e-1` — the same word,
/// five decades apart. What the tolerance costs is measured per
/// fixture: nothing on the idealised ones beyond iterations, and **4x
/// the usable depth on the real scan** (3.00 mm against 0.75 mm).
/// Pinned by `the_insertion_solves_convergence_is_bounded_by_its_tolerance`.
///
/// ▶ It is left at `1e-1` deliberately: changing it is a behaviour
/// change to every consumer of this tool and belongs to the bridge, not
/// to the measurement that found the problem.
pub(crate) const INSERTION_SOLVE_TOL: f64 = 1e-1;

/// Shared solver config for the insertion solve — the walking-
/// skeleton defaults with `dt` (static), `max_newton_iter`, and `tol`
/// set for this tool. Used by [`run_single_insertion_step`] (7.2) and
/// [`run_insertion_ramp`] (7.3b).
fn insertion_solver_config() -> SolverConfig {
    insertion_solver_config_at_tol(INSERTION_SOLVE_TOL)
}

/// The insertion solver config at a caller-chosen residual tolerance.
///
/// Exists so a test can ask what the solve does at a tolerance other
/// than the shipped [`INSERTION_SOLVE_TOL`]. That const is pinned *at*
/// the Armijo stall floor (see its docstring), which makes "converged"
/// a statement about the tolerance rather than about the solution —
/// and a claim like that is only worth making if it can be measured.
/// `the_insertion_solves_convergence_is_bounded_by_its_tolerance` is
/// what measures it.
///
/// Everything except `tol` is identical to [`insertion_solver_config`],
/// which delegates here; there is one config, not two.
fn insertion_solver_config_at_tol(tol: f64) -> SolverConfig {
    let mut config = SolverConfig::skeleton();
    config.dt = STATIC_DT;
    config.max_newton_iter = MAX_NEWTON_ITER;
    config.tol = tol;
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
/// ⚠ **The cavity bound recorded here was measured before the
/// slice-7.3d Gaussian pre-smooth and no longer holds.** It read:
/// *"ε = 0.075 mm converges 16/16 at cavity ≤ 5 mm but stalls at
/// cavity 6 mm (C.3 probe gate, r_norm 0.536)"*, and named
/// `cf_device_types::CavityState::inset_slider_range_m` as "the UI cap
/// that enforces this bound". Both halves are now wrong: measured
/// 2026-09-22 on the synthetic icosphere ramp, **every inset from
/// 3 mm to 8 mm converges 16/16** (3/4/5/6/7/8 mm, 35 670–45 654 tets,
/// 12–55 s release); and that slider's cap is
/// `cf_device_types::CAVITY_INSET_SLIDER_MAX_M` = **8 mm**, which
/// never enforced a 5 mm bound. The bookmark §9.4 probe-gate data
/// stands as history. Reproduce with
/// `the_ramp_converges_across_the_whole_cavity_slider_range`.  Generalizing past 5 mm would
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
    intruder_contact_at_kappa(
        intruder,
        bounds,
        interference_m,
        cavity_offset_m,
        INSERTION_CONTACT_KAPPA,
    )
}

/// [`intruder_contact_at`] with the penalty stiffness supplied rather
/// than read from [`INSERTION_CONTACT_KAPPA`].
///
/// Exists so the contact stiffness can be *asked a question* without
/// changing what ships, the same way
/// [`run_single_insertion_step_at_tol`] does for the solve tolerance.
/// The question is specific: the bridge derives its face-barrier `κ`
/// from a traction `σ` read off this scene, and that derivation is only
/// a derivation if `σ` is a property of the compressed wall rather than
/// of the contact stiffness being solved with. On a *stiff* contact it
/// is — the gap adjusts and the load does not. This scene's gaps sit a
/// large fraction of the way into a 1 mm band, which is the regime
/// where that stops being obvious, so it is measured rather than
/// assumed.
fn intruder_contact_at_kappa(
    intruder: &GridSdf,
    bounds: Aabb,
    interference_m: f64,
    cavity_offset_m: f64,
    contact_kappa: f64,
) -> PenaltyRigidContact {
    let intruder_solid =
        Solid::from_sdf(intruder.clone(), bounds).offset(interference_m + cavity_offset_m);
    PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![intruder_solid],
        contact_kappa,
        INSERTION_CONTACT_DHAT,
        INSERTION_CONTACT_SMOOTHING_EPS_M,
        INSERTION_CONTACT_NORMAL_AVG_K,
        INSERTION_CONTACT_NORMAL_AVG_RADIUS_M,
    )
}

/// The design traction the bridge's face barrier is sized against.
///
/// **Measured on the PRODUCT scan** (`base_mold`, 2026-09-22): 58.9 kPa on
/// the REST-area basis, read at penalty `κ = 1e5` — the stiffest arm that
/// stayed seated — at a common depth of 3.4375 mm through a 5 mm inset.
/// `κ = 1e4` reads 57.83 kPa at the same depth, so σ moves **1.0183× per
/// decade** across the seated arms: the flattest coupling of any scene tried,
/// which is what makes this close to converged rather than a loose bound.
///
/// ⛔⛔ **SUPERSEDES 117 kPa, read on #959's 3 mm Ecoflex 00-30 scenes** — the
/// synthetic sphere (117.01), with `sock_over_capsule` agreeing (119.84).
/// `base_mold` is a 5 mm inset through 17 mm of
/// DRAGON_SKIN_10A at 25 % Slacker. σ is roughly HALF, and `κ` scales with it
/// linearly, so every derived stiffness taken before this re-measurement was
/// about 2× too large → see `product_scene`.
///
/// ⚠ It is a **LOWER BOUND**, not a converged rigid traction: `κ = 1e6`
/// stalls at step 0 on both scenes, so `1e5` is the stiffest arm that solves
/// and `σ` is still moving 4.7 % across that last decade. For a *floor*
/// requirement a lower bound under-states the stiffness needed, so the
/// bracket derived below is read knowing its floor is the soft end.
///
/// ⛔ Do NOT re-read this at the shipped penalty stiffness (`κ = 1e3`): on
/// the real scan that state is THROUGH THE WALL (`min_sd` −0.373 mm with the
/// 5 % area tail at −0.042 mm), and its 6.85 kPa is not a traction on
/// anything. ⛔ And do NOT read it on the DEFORMED area basis —
/// `ContactPairReadout::tributary_area` is a deformed tributary while the
/// face barrier integrates over REST area; the two differ by 22–23 %.
const BRIDGE_DESIGN_TRACTION_PA: f64 = 58.9e3;

/// Patch non-uniformity `ρ` — the effective gap the barrier must hold, as a
/// multiple of the MINIMUM gap anywhere on the patch.
///
/// The stall condition is on the *minimum* gap while the traction is a
/// *mean*, and convexity puts the effective gap above the minimum. So the
/// floor requirement is evaluated at `ρ · step`, never at `step`: omitting
/// `ρ` makes the floor optimistic.
///
/// **Measured on the PRODUCT scan** in `[1.00, 1.11]` at `κ = 1e4…1e5` on a
/// non-penetrating seat — 1.113 (min-gap) and 1.036 (5 % tail) at `κ = 1e4`,
/// 1.010 and 1.004 at `κ = 1e5`. The upper end is taken: for a floor, the
/// larger `ρ` is the conservative one. ⛔ Supersedes 1.18, which was
/// `sock_over_capsule`.
///
/// ⛔ Read `ρ` against the area-weighted 5 % tail, never `min_sd` alone:
/// `min_sd` is ONE VERTEX, and the floor divides `|b′|` at `ρ · step`, so a
/// single bad tet would move a shipped constant. At the shipped stiffness the
/// synthetic sphere's `min_sd` ratio reads 4.92 — on a geometry with no shape
/// irregularity whatsoever — while the same pose at `κ = 1e5` reads 1.005.
const BRIDGE_PATCH_NONUNIFORMITY: f64 = 1.12;

/// Barrier band `d̂` for the bridge's face contact.
///
/// ⛔ **Not inherited from the penalty path.** `INSERTION_CONTACT_DHAT` is a
/// penalty band co-tuned with a smoothing `ε` and normal averaging, neither
/// of which the IPC face barrier has; carrying it across would be a
/// transcription, not a derivation.
///
/// From #959's candidate set (0.5 / 1.0 / 1.2 / 2.0 mm), 1.2 mm is the value
/// satisfying both standing constraints with margin:
/// - one ramp step is shorter than the band, `step < d̂` —
///   [`bridge_face_barrier_kappa`] refuses otherwise;
/// - the band stays well inside one element, `d̂ < SIM_CELL_SIZE_M / 3`.
///
/// `the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling` reports the bracket at
/// every candidate rather than asserting this one is optimal.
const BRIDGE_CONTACT_DHAT_M: f64 = 1.2e-3;

/// The face-barrier stiffness: **the ceiling**, the stiffest value that still
/// stays out of the cushioning regime.
///
/// ⛔⛔ **THIS RETURNED THE GEOMETRIC CENTRE OF `[floor, ceiling]`, AND THAT
/// WAS MEASURED WRONG ON THE PRODUCT SCENE.** The floor — *"κ such that the
/// barrier holds `ρ · step` open at traction σ"* — assumes κ *sets* the
/// standoff. In the window it was tested in (16 steps, κ 2.4e7–3.7e7) the
/// held standoff did not rise with κ: 0.2936, 0.2947, 0.2944 and 0.2766 mm
/// across a 1.5× range, at depths of 1.875–2.812 mm.
///
/// ⚠ **That is a result about that window, not about the wall.** On the same
/// scene at 32 steps the held standoff DOES follow κ at matched depth —
/// 0.317 → 0.415 mm from 6.3e7 to 1e8 at 4.375 mm, and 0.528 → 0.706 mm from
/// 1.6e8 to 4.0e8 at 4.219 mm
/// (`the_bridge_ramp_over_a_stiffness_sweep_on_the_product_scan`). What
/// differs between the two windows has not been isolated. An earlier revision
/// explained the flat window by the wall's stiffness; the same wall moves in
/// the other window, so that explanation is withdrawn.
///
/// ⭐⭐⭐ **And deriving κ FROM the increment measured worse the finer the
/// march**: with κ at the floor, held/step fell 0.94 → 0.86 → 0.81 across
/// 16/32/64 steps. κ, the step and the depth all changed together, so which
/// of them moved it is not isolated. Holding κ at the ceiling and refining the
/// schedule instead:
///
/// ```text
/// steps  step_mm  held_mm   depth     was (centre κ)
///    16   0.3125   0.3062   68.8 %    37.5 %
///    32   0.1562   0.2163   90.6 %    46.9 %
///    64   0.0781   0.2146   90.6 %    60.9 %
/// ```
///
/// ⇒ the contact-feasibility wall is GONE: 32 and 64 steps stop at the same
/// depth and on a DIFFERENT failure — an element inversion (`det F` < 0), not
/// a barrier stall.
///
/// ⇒ The floor is kept as a REPORTED diagnostic —
/// `the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling` still
/// prints it — but it no longer selects. The ceiling does, because it is the
/// stated requirement (stay out of the cushion) and by construction does not
/// depend on the increment. The product sweep CHECKS it rather than selecting
/// it: across 12 arms from 4.0e6 to 4.0e8 at 32 steps and the shipped
/// tolerance, the best depth, 4.531 mm, is reached at 4.0e7 and at the
/// shipped 4.11e7, and the next grid points either side reach less. Choosing
/// κ by the depth it buys would be a sweep wearing a derivation's clothes.
///
/// ⚠ The floor DID predict the stall on sim-soft's SEALED fixture (1e6 stalls
/// above 17.567 kPa, measured 14.255). It failed to transfer from there to
/// the product scene, which is not a refutation — record the regime a
/// derivation was validated in.
///
/// ⚠ `ramp_step_m` is still taken and still checked: an increment wider than
/// the band is a mis-specified schedule and is surfaced rather than clamped.
/// The schedule requirement is now `step < held standoff`, and the held
/// standoff must be **MEASURED** — no closed form here predicts it.
///
/// # Errors
///
/// `ramp_step_m` is not a positive finite length, is at least the whole band,
/// or the ceiling is not derivable at this `d̂`.
fn bridge_face_barrier_kappa(d_hat: f64, ramp_step_m: f64) -> Result<f64> {
    if !ramp_step_m.is_finite() || ramp_step_m <= 0.0 {
        return Err(anyhow!(
            "the ramp increment must be a positive finite length, got {ramp_step_m:?}"
        ));
    }
    if ramp_step_m >= d_hat {
        return Err(anyhow!(
            "one increment ({ramp_step_m:.6e} m) is at least the whole barrier band \
             (d̂ = {d_hat:.6e} m) — the barrier cannot see a node before it has already \
             passed through, so march in smaller steps or widen d̂"
        ));
    }
    face_barrier_kappa(d_hat, 0.5 * d_hat, BRIDGE_DESIGN_TRACTION_PA)
        .ok_or_else(|| anyhow!("the barrier ceiling is not derivable at d̂ = {d_hat:.6e} m"))
}

/// The intruder as an IPC face-barrier contact — the bridge's replacement for
/// [`intruder_contact_at_kappa`].
///
/// The rigid primitive is **unchanged**: both paths hand the contact model the
/// same `Solid::from_sdf(intruder).offset(interference + cavity_offset)`, so
/// nothing about the scene's geometry moves across the bridge. Only the
/// contact law does — and with it the two parameters, which are derived
/// ([`bridge_face_barrier_kappa`]) rather than swept.
///
/// ⚠ What is *dropped* is the penalty path's smoothing `ε` and normal
/// averaging. Those exist to make a per-vertex penalty force behave on a
/// faceted SDF; the face barrier integrates over a quadratic face with
/// quadrature weights and needs neither. `INSERTION_CONTACT_SMOOTHING_EPS_M`
/// in particular is a swept, CAVITY-SPECIFIC sweet spot — losing it is the
/// point of the exercise, not a regression.
fn intruder_ipc_contact_at(
    intruder: &GridSdf,
    bounds: Aabb,
    interference_m: f64,
    cavity_offset_m: f64,
    kappa: f64,
    d_hat: f64,
) -> IpcRigidContact {
    let intruder_solid =
        Solid::from_sdf(intruder.clone(), bounds).offset(interference_m + cavity_offset_m);
    IpcRigidContact::with_params(vec![intruder_solid], kappa, d_hat)
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
    mesh: &dyn Mesh<Yeoh>,
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
        roller_vertices: Vec::new(),
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
    run_single_insertion_step_at_tol(geometry, interference_m, INSERTION_SOLVE_TOL)
}

/// [`run_single_insertion_step`] at a caller-chosen residual tolerance.
///
/// Same solve, same contact, same boundary conditions — only
/// `SolverConfig::tol` moves (see [`insertion_solver_config_at_tol`]).
/// A tolerance the solve cannot reach returns the `Err` describing how
/// it failed, with the residual it floored at, which is the reading
/// that makes the shipped tolerance's meaning measurable rather than
/// asserted.
///
/// # Errors
///
/// As [`run_single_insertion_step`], plus: a `tol` below what this
/// scene's Newton solve can reach returns the `NewtonIterCap` or
/// `ArmijoStall` message carrying `last_r_norm`.
pub fn run_single_insertion_step_at_tol(
    geometry: InsertionGeometry,
    interference_m: f64,
    tol: f64,
) -> Result<InsertionStep> {
    if !interference_m.is_finite() {
        return Err(anyhow!(
            "insertion interference is non-finite ({interference_m})"
        ));
    }

    let InsertionGeometry {
        // Not used here: a single static step records no conformity score —
        // only the ramps aggregate one, because coverage and the peak barrier
        // are read across the engagement sweep.
        cavity_tensile_strength_pa: _,
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
    let config = insertion_solver_config_at_tol(tol);

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
        Err(SolverFailure::ValidityViolation { tet_id, .. }) => {
            return Err(anyhow!(
                "insertion solve left the validity domain at tet {tet_id} (a tet over-stretched / inverted)"
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
/// `mesh_sdf::PseudoNormalSign`'s closest-face-normal sign being
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
/// [`TriMeshDistance::new`] failures: an empty mesh, no positive-area
/// triangle to derive an internal scale from, or a face naming a
/// missing vertex.
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
/// [`ReadoutMesh::readouts`]) and pre-aggregated scalar metrics (contact
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
    /// reconstruct the per-tet readouts at *any* step on demand via
    /// [`InsertionRamp::readout_mesh`].
    pub x_final: Vec<f64>,
    /// Slice 7.3b.2 — pre-aggregated engineering scalars for this step
    /// (F-d curve ordinate, principal-stretch extrema, peak stress,
    /// mean strain-energy density). Derived once when the step
    /// converges.
    pub readout: StepReadout,
    /// Wall-clock seconds this step's Newton solve took, measured around
    /// `replay_step` alone — not the readouts built afterwards.
    ///
    /// ⚠ **A diagnostic, never an assertion.** Wall clock is contended: the
    /// one release-only timing assertion in this workspace
    /// (`adjoint_gap_across_basis_sizes`) inflates **80×** under `xtask
    /// grade`'s coverage pass. Report it, print it, size a fixture with it —
    /// do not gate on it.
    ///
    /// It exists because the cost question the bridge raises is per-STEP, not
    /// per-ramp: a ramp's total folds in the geometry build and answers
    /// nothing about the solve.
    pub wall_time_s: f64,
}

/// Scalar per-step engineering aggregates — slice 7.3b.2's per-step
/// summary of the per-tet stress / stretch field and the contact
/// reaction. Stored on each converged [`RampStep`] and assembled into
/// the [`force_displacement_curve`](InsertionResult::force_displacement_curve)
/// on [`InsertionResult`].
///
/// Per-tet detail at a specific step is derivable from
/// [`RampStep::x_final`] via [`ReadoutMesh::readouts`]; pre-aggregating
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
    /// Conformity reward for this step, computed over Γ.
    ///
    /// `None` when Γ selected no faces or had no area — a score against
    /// nothing is meaningless, and its absence marks a scene-setup defect
    /// rather than a bad design.
    ///
    /// ⚠ Read [`ConformityReadout::lq_ratio`] before trusting `peak_bound`,
    /// and [`ConformityReadout::non_finite_pressures`] before trusting any of
    /// it. `stiffness_bound` is always `NaN` — `k_min` has no source.
    pub conformity: Option<ConformityReadout>,
    /// Mean strain-energy density (J/m³) across every tet — the
    /// per-step "how strained" scalar; integrating `× tet_volume`
    /// recovers total elastic energy (volumes are in the mesh's
    /// `QualityMetrics::signed_volume`, not aggregated here).
    pub mean_strain_energy_density_j_per_m3: f64,
}

/// Per-element engineering readout at a single step — the scalars the UI heat
/// map and the per-step aggregates consume, reduced from the material state at
/// the element's Gauss points ([`GaussPointReadout`]).
///
/// ⭐ **Read where the solver evaluates the material.** Tet4 has one Gauss
/// point, so each field is that point's value. Tet10 has four, and its
/// deformation gradient varies across the element, so no single `F`
/// describes the element — each field therefore names its reduction.
///
/// Built by [`ReadoutMesh::readouts`]; per-point detail, `F` and `P`
/// included, is [`ReadoutMesh::gauss_point_readouts`]. The per-step
/// aggregates in [`StepReadout`] are reductions over these.
#[derive(Debug, Clone)]
pub struct TetReadout {
    /// Element-mean strain-energy density (J/m³): `Σ_q v_q Ψ(F_q)`, with
    /// `v_q` each point's [`volume_fraction`](GaussPointReadout::volume_fraction)
    /// — the element's stored energy per unit rest volume.
    pub energy_density_j_per_m3: f64,
    /// Peak Frobenius norm of the first-Piola stress over the Gauss points
    /// (Pa) — the scalar hotspot intensity.
    pub first_piola_frobenius_pa: f64,
    /// Smallest principal stretch at any Gauss point — the reading to hold
    /// against the Yeoh `validity.min_principal_stretch` cap.
    pub min_principal_stretch: f64,
    /// Largest principal stretch at any Gauss point — the reading to hold
    /// against the Yeoh `validity.max_principal_stretch` cap.
    pub max_principal_stretch: f64,
}

impl TetReadout {
    /// Reduce one element's Gauss points.
    ///
    /// ⚠ A NaN reading PROPAGATES. `f64::max` returns the non-NaN
    /// operand, so reducing with it would report a NaN stress as the other
    /// points' peak — and a readout exists to show that state, not hide it.
    fn over(points: &[GaussPointReadout]) -> Self {
        let peak = |a: f64, b: f64| {
            if a.is_nan() || b.is_nan() {
                f64::NAN
            } else {
                a.max(b)
            }
        };
        let least = |a: f64, b: f64| {
            if a.is_nan() || b.is_nan() {
                f64::NAN
            } else {
                a.min(b)
            }
        };
        let stretches = || {
            points
                .iter()
                .flat_map(|p| p.principal_stretches.iter().copied())
        };
        // `reduce`, not `sum`/`fold` from a seed: a single-point element then
        // reports its one point's value bit for bit.
        Self {
            energy_density_j_per_m3: points
                .iter()
                .map(|p| p.volume_fraction * p.energy_density_j_per_m3)
                .reduce(|a, b| a + b)
                .unwrap_or(f64::NAN),
            first_piola_frobenius_pa: points
                .iter()
                .map(|p| p.first_piola_frobenius_pa)
                .reduce(peak)
                .unwrap_or(f64::NAN),
            min_principal_stretch: stretches().reduce(least).unwrap_or(f64::NAN),
            max_principal_stretch: stretches().reduce(peak).unwrap_or(f64::NAN),
        }
    }
}

/// The material state at one Gauss point of one element.
#[derive(Debug, Clone)]
pub struct GaussPointReadout {
    /// The share of the element's rest volume this point integrates:
    /// `w_q |det J(ξ_q)| / Σ_p w_p |det J(ξ_p)|`. Equal across the points of
    /// a straight-edged element, and `1` on Tet4.
    pub volume_fraction: f64,
    /// Deformation gradient `F = J_curr(ξ_q) · J_rest(ξ_q)⁻¹`, where `J` is
    /// the isoparametric Jacobian `Σ_a x_a ⊗ ∇_ξN_a(ξ_q)`. On Tet4 this is
    /// the edge-vector `D_curr · D_rest⁻¹`.
    pub f: Matrix3<f64>,
    /// First Piola stress `P = Yeoh::first_piola(F)` (Pa).
    pub first_piola: Matrix3<f64>,
    /// Frobenius norm of [`first_piola`](Self::first_piola) (Pa).
    pub first_piola_frobenius_pa: f64,
    /// Strain-energy density `Ψ = Yeoh::energy(F)` (J/m³).
    pub energy_density_j_per_m3: f64,
    /// Principal stretches — the singular values of `F` from
    /// `f.svd_unordered(false, false)`, the same call the solver's
    /// principal-stretch gate makes (`check_element_validity`). NOT sorted.
    ///
    /// ⚠ On Tet10 that gate reads the element's affine corner block, not
    /// these points, so a Gauss point can sit outside a stretch cap the
    /// solver accepted.
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
/// [`ReadoutMesh::readouts`] using [`RampStep::x_final`]; pre-computing
/// it for every step would hold `n_tets × n_steps` readouts, and the UI
/// consumes one step's detail at a time anyway.
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
    /// What the per-tet readouts are evaluated over — the mesh this ramp
    /// SOLVED. Any step's readouts are
    /// `readout_mesh.readouts(&positions)` of that step's `x_final`; never
    /// rebuild them from a snapshot of the scene's Tet4 mesh (see
    /// [`ReadoutMesh`]).
    pub readout_mesh: ReadoutMesh,
}

/// The rest configuration, element connectivity and per-element materials a
/// readout is evaluated over — snapshotted by a ramp from the mesh it SOLVED.
///
/// ⛔ **Take it from the ramp; never rebuild it.** The bridge solves on a Tet10
/// enrichment of the scene's Tet4 mesh, which keeps the corner ids and appends
/// the midsides. A consumer that snapshots the Tet4 mesh instead gets
/// corner-only connectivity that indexes the Tet10 positions without
/// complaint, and reads the linear part of a quadratic field —
/// `the_readout_resolves_the_tet10_strain_at_every_gauss_point` measures that
/// gap.
#[derive(Clone)]
pub struct ReadoutMesh {
    rest_positions: Vec<Vec3>,
    elements: ReadoutElements,
    /// One per element, per [`Mesh::materials`].
    materials: Vec<Yeoh>,
}

/// Element connectivity, each element in its local node order.
#[derive(Clone)]
enum ReadoutElements {
    /// The four corners.
    Tet4(Vec<[VertexId; 4]>),
    /// The four corners, then the six midsides in
    /// [`TET10_EDGE_NODES`](sim_soft::element::TET10_EDGE_NODES) order — the
    /// order [`Tet10`]'s shape functions are written in.
    Tet10(Vec<[VertexId; 10]>),
}

impl ReadoutMesh {
    /// Snapshot a Tet4 mesh.
    fn tet4(mesh: &SdfMeshedTetMesh<Yeoh>) -> Self {
        // `TetId` is a `u32`; Phase 4 meshes stay well under `u32::MAX` per
        // the `Mesh` trait docs.
        #[allow(clippy::cast_possible_truncation)]
        let elements = (0..mesh.n_tets() as TetId)
            .map(|t| mesh.tet_vertices(t))
            .collect();
        Self {
            rest_positions: mesh.positions().to_vec(),
            elements: ReadoutElements::Tet4(elements),
            materials: mesh.materials().to_vec(),
        }
    }

    /// Snapshot a Tet10 mesh — corners AND midsides.
    ///
    /// `None` if any element does not name its midsides, which `Tet10Mesh`
    /// always does; the `Option` keeps the case a caller's error rather than
    /// a panic.
    fn tet10(mesh: &Tet10Mesh<Yeoh>) -> Option<Self> {
        // `TetId` is a `u32`; Phase 4 meshes stay well under `u32::MAX`.
        #[allow(clippy::cast_possible_truncation)]
        let elements = (0..mesh.n_tets() as TetId)
            .map(|t| {
                let c = mesh.tet_vertices(t);
                let m = mesh.tet_midside_nodes(t)?;
                Some([c[0], c[1], c[2], c[3], m[0], m[1], m[2], m[3], m[4], m[5]])
            })
            .collect::<Option<Vec<_>>>()?;
        Some(Self {
            rest_positions: mesh.positions().to_vec(),
            elements: ReadoutElements::Tet10(elements),
            materials: mesh.materials().to_vec(),
        })
    }

    /// An empty mesh — no vertices, no elements — for a test fixture that
    /// builds a ramp by hand and never reads its per-tet detail.
    #[cfg(test)]
    pub(crate) fn empty() -> Self {
        Self::at_rest(Vec::new())
    }

    /// Rest positions and no elements — for a test fixture that reads only
    /// where the solved mesh's vertices sat at rest.
    #[cfg(test)]
    pub(crate) fn at_rest(rest_positions: Vec<Vec3>) -> Self {
        Self {
            rest_positions,
            elements: ReadoutElements::Tet4(Vec::new()),
            materials: Vec::new(),
        }
    }

    /// The rest positions the readouts are evaluated against — every vertex of
    /// the solved mesh, midsides included on Tet10, so indexed exactly like a
    /// step's `x_final`.
    #[must_use]
    pub fn rest_positions(&self) -> &[Vec3] {
        &self.rest_positions
    }

    /// Number of elements — one [`TetReadout`] each.
    #[must_use]
    pub fn n_elements(&self) -> usize {
        match &self.elements {
            ReadoutElements::Tet4(e) => e.len(),
            ReadoutElements::Tet10(e) => e.len(),
        }
    }

    /// Per-element readouts at the deformed positions `curr`, indexed like
    /// the rest positions (a ramp step's `x_final`, unflattened).
    ///
    /// # Panics
    ///
    /// When `curr` is not the rest positions' length (positions from another
    /// mesh), when the element and material counts differ, or on a degenerate
    /// rest element (a `J_rest` with no inverse).
    #[must_use]
    pub fn readouts(&self, curr: &[Vec3]) -> Vec<TetReadout> {
        self.check(curr);
        let rest = &self.rest_positions;
        match &self.elements {
            ReadoutElements::Tet4(elements) => elements
                .iter()
                .zip(&self.materials)
                .map(|(nodes, m)| {
                    TetReadout::over(&gauss_point_readouts(&Tet4, nodes, rest, curr, m))
                })
                .collect(),
            ReadoutElements::Tet10(elements) => elements
                .iter()
                .zip(&self.materials)
                .map(|(nodes, m)| {
                    TetReadout::over(&gauss_point_readouts(&Tet10, nodes, rest, curr, m))
                })
                .collect(),
        }
    }

    /// The material state at each Gauss point of one element, in
    /// [`Element::gauss_points`] order — the detail [`TetReadout`] reduces.
    ///
    /// # Panics
    ///
    /// As [`readouts`](Self::readouts), and on an out-of-range `element`.
    #[must_use]
    pub fn gauss_point_readouts(&self, element: usize, curr: &[Vec3]) -> Vec<GaussPointReadout> {
        self.check(curr);
        let (rest, m) = (&self.rest_positions, &self.materials[element]);
        match &self.elements {
            ReadoutElements::Tet4(e) => {
                gauss_point_readouts(&Tet4, &e[element], rest, curr, m).to_vec()
            }
            ReadoutElements::Tet10(e) => {
                gauss_point_readouts(&Tet10, &e[element], rest, curr, m).to_vec()
            }
        }
    }

    /// ⛔ Refuse positions from another mesh. A Tet4 view of an enriched mesh
    /// indexes the enriched positions without complaint — its corner ids are a
    /// prefix of them — and reads the linear part of the field, the readout
    /// this type replaced. So a length mismatch is an error, not a partial read.
    fn check(&self, curr: &[Vec3]) {
        assert_eq!(
            curr.len(),
            self.rest_positions.len(),
            "ReadoutMesh: {} positions against a mesh of {} vertices — they come from \
             different meshes",
            curr.len(),
            self.rest_positions.len(),
        );
        assert_eq!(
            self.n_elements(),
            self.materials.len(),
            "ReadoutMesh: {} elements but {} materials (one per element, per Mesh::materials)",
            self.n_elements(),
            self.materials.len(),
        );
    }
}

/// The material state at each of one element's Gauss points.
///
/// `F` at each point is `J_curr(ξ_q) · J_rest(ξ_q)⁻¹`, from the isoparametric
/// Jacobians of the element's map off the reference tet — which holds for a
/// curved element as well as a straight one (gated on a bowed element in
/// `the_readout_resolves_the_tet10_strain_at_every_gauss_point`). On Tet4 both
/// Jacobians are the
/// edge-vector matrices, so this reproduces the edge-vector
/// `D_curr · D_rest⁻¹` readout it replaced.
///
/// # Panics
///
/// On a degenerate rest element (`J_rest` non-invertible at a Gauss point).
/// Production meshers reject those with a signed-volume gate before assembly,
/// so reaching one here is a construction-side contract violation, and it is
/// surfaced loudly rather than read as a number.
//
// `clippy::panic` is denied crate-wide; this is the upstream-invariant
// carve-out, the same one sim-soft makes for invariant-violation assertions.
#[allow(clippy::panic)]
fn gauss_point_readouts<E: Element<N, G>, const N: usize, const G: usize>(
    element: &E,
    nodes: &[VertexId; N],
    rest: &[Vec3],
    curr: &[Vec3],
    material: &Yeoh,
) -> [GaussPointReadout; G] {
    let points = element.gauss_points();
    let mut volumes = [0.0; G];
    let mut readouts: [GaussPointReadout; G] = std::array::from_fn(|q| {
        let (xi, weight) = points[q];
        let grad_xi = element.shape_gradients(xi);
        let j_rest = isoparametric_jacobian(nodes, rest, &grad_xi);
        let j_rest_inv = j_rest.try_inverse().unwrap_or_else(|| {
            panic!(
                "element rest configuration is degenerate (J_rest non-invertible) — \
                 the mesh constructor's signed-volume gate should have rejected it"
            )
        });
        volumes[q] = weight * j_rest.determinant().abs();
        gauss_point_readout(
            isoparametric_jacobian(nodes, curr, &grad_xi) * j_rest_inv,
            material,
        )
    });
    let total: f64 = volumes.iter().sum();
    for (r, v) in readouts.iter_mut().zip(volumes) {
        r.volume_fraction = v / total;
    }
    readouts
}

/// `J(ξ) = Σ_a x_a ⊗ ∇_ξN_a(ξ)` — the Jacobian of the isoparametric map from
/// the reference tet to the element's nodes at `positions`.
fn isoparametric_jacobian<const N: usize>(
    nodes: &[VertexId; N],
    positions: &[Vec3],
    grad_xi: &SMatrix<f64, N, 3>,
) -> Matrix3<f64> {
    let mut j = Matrix3::zeros();
    for (a, &v) in nodes.iter().enumerate() {
        let x = positions[v as usize];
        for d in 0..3 {
            for k in 0..3 {
                j[(d, k)] += x[d] * grad_xi[(a, k)];
            }
        }
    }
    j
}

/// The material state at one deformation gradient. `volume_fraction` is left
/// at `1` for the caller to normalise.
fn gauss_point_readout(f: Matrix3<f64>, material: &Yeoh) -> GaussPointReadout {
    let first_piola = material.first_piola(&f);
    GaussPointReadout {
        volume_fraction: 1.0,
        f,
        first_piola,
        first_piola_frobenius_pa: first_piola.norm(),
        energy_density_j_per_m3: material.energy(&f),
        principal_stretches: f.svd_unordered(false, false).singular_values,
    }
}

/// Γ — the **intended contact surface** for an insertion scene.
///
/// The conformity reward integrates over Γ and divides coverage by `|Γ|`, so
/// Γ must be the cavity wall alone. `Mesh::boundary_faces` also spans the
/// outer envelope, which can never contact; using it would cap coverage
/// structurally below 1.
///
/// # Rest membership, deformed area
///
/// ⚠ These come from different configurations, on purpose.
///
/// **Membership is evaluated on the REST positions.** Γ is a statement of
/// design intent — which surface was *meant* to contact — and the scan SDF
/// describes the undeformed geometry. Evaluating it at deformed positions
/// would ask where the material has moved to, which is a different question.
///
/// **Area is evaluated on the DEFORMED positions**, because
/// `ContactPairReadout::tributary_area` is a deformed area. Dividing deformed
/// numerator areas by a rest `|Γ|` would let a stretched sleeve score coverage
/// above 1.
pub(crate) struct GammaMask {
    face_flags: Vec<bool>,
    vertex_flags: Vec<bool>,
}

impl GammaMask {
    /// Build from the rest configuration and the scan SDF at the cavity level.
    ///
    /// # ⚠ The rim rule: a vertex counts if ANY of its faces is in Γ
    ///
    /// A vertex on the rim belongs to both cavity-wall and outer-envelope
    /// faces. This marks it in-Γ. Two alternatives were considered:
    ///
    /// - **All-faces** (a vertex counts only if *every* incident face is in Γ)
    ///   would exclude the entire rim ring, and the rim is where
    ///   `04-rim.md` says contact pressure concentrates — dropping it
    ///   discards the highest-pressure band from the very terms meant to
    ///   measure it.
    /// - **Area-weighted partial membership** is more faithful but needs a
    ///   per-vertex fractional weight the `ContactPairReadout` surface does
    ///   not carry, so it cannot be done without widening that type.
    ///
    /// Any-face is chosen because it errs toward *including* the rim, which is
    /// where `04-rim.md` says contact pressure concentrates.
    ///
    /// ⛔ **But be clear which side it inflates.** The vertex rule decides the
    /// coverage **numerator** (which readouts count); `|Γ|` is computed from
    /// **faces**. So any-face makes coverage *higher*, i.e. flattering, not
    /// conservative — an earlier version of this comment had that backwards.
    ///
    /// ⚠ It also means **coverage can exceed 1** where Γ's vertex set spills
    /// onto unflagged faces: `boundary_vertex_areas` gives a vertex a third of
    /// *every* incident face, including faces absent from `|Γ|`. Measured on
    /// the cube fixture Γ happens to be closed — its 1202 vertices' tributary
    /// areas sum to exactly `|Γ|`, so coverage lands on 1.0 — but a cavity with
    /// a real rim would spill. [`ConformityReadout::coverage_overflow`] flags
    /// it rather than letting a >1 coverage read as excellent conformity.
    ///
    /// ⚠ A choice, not a derivation; the rim band is where `04-rim.md` says all
    /// four terms are most fragile.
    pub(crate) fn build(
        rest_positions: &[Vec3],
        boundary_faces: &[[VertexId; 3]],
        n_vertices: usize,
        sdf: &dyn Sdf,
        cavity_offset_m: f64,
    ) -> Self {
        let (face_flags, _rest_area) =
            boundary_faces_on_isosurface(rest_positions, boundary_faces, sdf, cavity_offset_m);
        let mut vertex_flags = vec![false; n_vertices];
        for (face, &on) in boundary_faces.iter().zip(&face_flags) {
            if on {
                for &v in face {
                    vertex_flags[v as usize] = true;
                }
            }
        }
        Self {
            face_flags,
            vertex_flags,
        }
    }

    /// Number of boundary faces in Γ — zero means the level selected nothing,
    /// which makes every reward term meaningless and must not pass silently.
    pub(crate) fn n_faces(&self) -> usize {
        self.face_flags.iter().filter(|&&f| f).count()
    }

    /// `|Γ|` at the supplied (deformed) positions.
    fn area_at(&self, positions: &[Vec3], boundary_faces: &[[VertexId; 3]]) -> f64 {
        boundary_faces
            .iter()
            .zip(&self.face_flags)
            .filter(|(_, on)| **on)
            .map(|(&[a, b, c], _)| {
                let (va, vb, vc) = (
                    positions[a as usize],
                    positions[b as usize],
                    positions[c as usize],
                );
                0.5 * (vb - va).cross(&(vc - va)).norm()
            })
            .sum()
    }

    /// Whether a contact readout lands on Γ.
    ///
    /// A `Face` pair counts when **any** of its six P2 nodes is in Γ: the
    /// barrier is integrated over the whole face, so a face straddling the Γ
    /// boundary contributes load to it. ⚠ `insertion_sim` runs Tet4 today and
    /// emits only `Vertex` pairs; this arm exists so the Tet10 face-barrier
    /// path is not silently dropped when it lands.
    fn contains(&self, readout: &ContactPairReadout) -> bool {
        match readout.pair {
            ContactPair::Vertex { vertex_id, .. } => self
                .vertex_flags
                .get(vertex_id as usize)
                .copied()
                .unwrap_or(false),
            ContactPair::Face { nodes, .. } => nodes
                .iter()
                .any(|&v| self.vertex_flags.get(v as usize).copied().unwrap_or(false)),
            _ => false,
        }
    }

    /// The conformity reward for one step.
    ///
    /// Returns `None` when Γ is empty or has no area — a score computed
    /// against nothing would be a number with no meaning, and the caller
    /// should treat its absence as a scene-setup defect.
    pub(crate) fn conformity(
        &self,
        positions: &[Vec3],
        boundary_faces: &[[VertexId; 3]],
        readouts: &[ContactPairReadout],
        tensile_strength_pa: f64,
    ) -> Option<ConformityReadout> {
        let gamma_area = self.area_at(positions, boundary_faces);
        // `is_nan()` spelled out: a NaN area must reject, and a negated
        // partial-ord comparison hides that from the reader.
        if self.n_faces() == 0 || gamma_area.is_nan() || gamma_area <= 0.0 {
            return None;
        }
        // ⛔ A non-finite ceiling must not be scored. `p_th` derives from it,
        // so every term would go NaN, `score_with` would drop all four, and
        // the composed score would be 0.0 — which ranks ABOVE a measured poor
        // design scoring negative. `SiliconeMaterial::from_measured` produces
        // exactly this ceiling on purpose.
        if !tensile_strength_pa.is_finite() || tensile_strength_pa <= 0.0 {
            return None;
        }
        let on_gamma: Vec<ContactPairReadout> = readouts
            .iter()
            .filter(|r| self.contains(r))
            .cloned()
            .collect();
        let params = ConformityParams::from_tensile_strength(tensile_strength_pa, gamma_area);
        Some(conformity_breakdown(&on_gamma, &params))
    }
}

/// Reduce per-tet readouts + orphan-filtered contact-pair readouts to
/// the scalar [`StepReadout`] aggregates a single ramp step records.
///
/// ⚠ The `>`/`<` comparisons skip a NaN, as they always have. Because an
/// element's stretch range is NaN when any of its Gauss points reads a NaN
/// stretch ([`TetReadout`] propagates it), such an element drops out of the
/// step's stretch extrema whole — the corner readout dropped only the NaN
/// value itself.
fn aggregate_step_readout(
    per_tet: &[TetReadout],
    contact_readouts: &[ContactPairReadout],
    conformity: Option<ConformityReadout>,
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
        if t.max_principal_stretch > max_principal_stretch {
            max_principal_stretch = t.max_principal_stretch;
        }
        if t.min_principal_stretch < min_principal_stretch {
            min_principal_stretch = t.min_principal_stretch;
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
        conformity,
    }
}

/// Convert a flat `Vec<f64>` vertex-major xyz into the `Vec<Vec3>` slice
/// view of vertex positions that [`ReadoutMesh::readouts`] and
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
/// the SL.4 viewport's `"stalled at step N: <reason>"` surface reads the
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
        SolverFailure::ValidityViolation { tet_id, .. } => {
            format!("validity-domain violation at tet {tet_id} (a tet over-stretched / inverted)")
        }
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
    run_insertion_ramp_at_kappa(geometry, n_steps, INSERTION_CONTACT_KAPPA)
}

/// [`run_insertion_ramp`] with the penalty contact stiffness supplied
/// rather than read from [`INSERTION_CONTACT_KAPPA`] — see
/// [`intruder_contact_at_kappa`] for why the knob exists.
///
/// Everything else about the ramp is unchanged, including the readout
/// contact, which is rebuilt at the *same* stiffness as the solve so
/// the reported patch is the one that was actually solved.
///
/// # Errors
///
/// Identical to [`run_insertion_ramp`]: `n_steps` is zero, or
/// [`outer_skin_bc`] finds no outer-skin vertex in the pin-band.
pub fn run_insertion_ramp_at_kappa(
    geometry: InsertionGeometry,
    n_steps: usize,
    contact_kappa: f64,
) -> Result<InsertionRamp> {
    run_insertion_ramp_at_kappa_and_tol(geometry, n_steps, contact_kappa, INSERTION_SOLVE_TOL)
}

/// [`run_insertion_ramp_at_kappa`] with the residual tolerance supplied too.
///
/// Exists so the Tet4 + penalty baseline can be run at the SAME tolerance as
/// the bridge ([`run_insertion_ramp_tet10_ipc`]) without editing a shipped
/// constant. #958 measured this comparison by flipping `INSERTION_SOLVE_TOL`
/// in the source and reverting afterwards; a knob makes the same measurement
/// reproducible by anyone, and — unlike a const flip — cannot be left behind.
///
/// ⛔ This does NOT change what ships. [`run_insertion_ramp`] and
/// [`run_insertion_ramp_at_kappa`] both delegate here at
/// [`INSERTION_SOLVE_TOL`], and `the_shipped_ramp_solves_at_the_shipped_tolerance`
/// pins that.
///
/// # Errors
///
/// Identical to [`run_insertion_ramp`]: `n_steps` is zero, or
/// [`outer_skin_bc`] finds no outer-skin vertex in the pin-band.
pub fn run_insertion_ramp_at_kappa_and_tol(
    geometry: InsertionGeometry,
    n_steps: usize,
    contact_kappa: f64,
    tol: f64,
) -> Result<InsertionRamp> {
    if n_steps == 0 {
        return Err(anyhow!("insertion ramp needs at least one step"));
    }

    let InsertionGeometry {
        cavity_tensile_strength_pa,
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

    // BCs are constant across the ramp (the outer skin does not move)
    // — build once, clone per step.
    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot per-tet immutables before consuming the mesh into the
    // per-step solver clones: rest positions, the readout mesh (tet
    // connectivity + per-tet Yeoh materials), and the referenced-vertex
    // set for orphan filtering (`SdfMeshedTetMesh` retains BCC lattice
    // corners not referenced by any tet — see `referenced_vertices`
    // docs). The ramp builds per-step readouts from these without
    // needing the mesh after the loop ends.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let readout_mesh = ReadoutMesh::tet4(&mesh);
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);

    // Γ is a property of the REST configuration — which surface was meant to
    // contact — so it is built once here rather than per step. Its AREA is
    // re-measured per step on the deformed positions; see `GammaMask`.
    let gamma_faces: Vec<[VertexId; 3]> = Mesh::<Yeoh>::boundary_faces(&mesh).to_vec();
    let gamma = GammaMask::build(
        &rest_positions,
        &gamma_faces,
        Mesh::<Yeoh>::n_vertices(&mesh),
        &intruder,
        cavity_offset_m,
    );

    // Full press-fit interference = the cavity inset; the ramp seats
    // the intruder there in `n_steps` equal increments.
    let inset_m = -cavity_offset_m;

    let config = insertion_solver_config_at_tol(tol);

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
        let contact = intruder_contact_at_kappa(
            &intruder,
            bounds,
            interference_m,
            cavity_offset_m,
            contact_kappa,
        );
        let solver = CpuNewtonSolver::new(Tet4, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        // `replay_step` panics on non-convergence — catch it so the
        // ramp records `failed_at_step` + the panic's reason instead
        // of aborting.
        let solve_started = std::time::Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        let wall_time_s = solve_started.elapsed().as_secs_f64();
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
                let readout_contact = intruder_contact_at_kappa(
                    &intruder,
                    bounds,
                    interference_m,
                    cavity_offset_m,
                    contact_kappa,
                );
                let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                let contact_readouts =
                    filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                let per_tet = readout_mesh.readouts(&positions_k);
                let conformity = gamma.conformity(
                    &positions_k,
                    &gamma_faces,
                    &contact_readouts,
                    cavity_tensile_strength_pa,
                );
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts, conformity);

                steps.push(RampStep {
                    interference_m,
                    iter_count: step.iter_count,
                    final_residual_norm: step.final_residual_norm,
                    x_final: step.x_final.clone(),
                    readout: step_readout,
                    wall_time_s,
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
        let final_per_tet = readout_mesh.readouts(&final_positions);
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
        readout_mesh,
    })
}

/// Fraction of its original rest Jacobian an element may lose to a midside
/// move before [`Tet10Mesh::with_projected_midsides`] backs the move off.
///
/// The projection moves boundary midsides onto a curved surface, which bends
/// elements; pushed too far it inverts them. 0.5 keeps every incident element
/// above half its original Jacobian everywhere, which the sagitta-scale moves
/// here (~0.17 mm against a 4 mm cell) never approach — the floor is a guard,
/// not a working constraint.
const CAVITY_MIDSIDE_QUALITY_FLOOR: f64 = 0.5;

/// How far from the cavity isosurface a boundary midside may sit and still be
/// treated as a cavity midside, as a fraction of the cell size.
///
/// Boundary faces span BOTH the cavity and the outer skin. The outer skin is
/// pinned and is a whole wall thickness away (17 mm on the product scene), so
/// any generous threshold separates them; this one is 0.5 cells = 2 mm
/// against a 0.17 mm sagitta.
const CAVITY_MIDSIDE_BAND_CELLS: f64 = 0.5;

/// Put the enriched boundary midsides back ON the curved cavity surface.
///
/// ⚠ **Written as the fix for the bridge's feasibility stall; it was not.**
/// Wiring it in left the stall where it was (same 6/16 — see the ramp's
/// `THE MESH SWAP` note), and it is not wired. `Tet10Mesh::from_tet4` places
/// every midside at the straight-edge MIDPOINT, so on a curved cavity each
/// boundary midside sits under the true surface by the sagitta; this moves
/// them back onto it.
///
/// Measured on the product scan before this existed:
///
/// ```text
/// at rest      tightest corner -0.0737 mm   tightest midside -0.2439 mm
///              enrichment excess                              0.1702 mm
/// at the stall  608 corners hold 0.4621 mm   4149 midsides hold 0.2936 mm
///              deficit                                        0.1685 mm
///              one increment                                  0.3125 mm
/// ```
///
/// The corners held comfortably more than an increment; the midsides did not,
/// and the deficit matches the rest-configuration sagitta to within 1 %.
///
/// Only cavity-side boundary midsides move: the outer skin is pinned and a
/// whole wall thickness away. Corners are untouched — conforming those is the
/// mesher's job (`SdfMeshedTetMesh::with_projected_nodes`) and doing it here
/// would move the Dirichlet set.
fn conform_cavity_midsides(
    mesh: Tet10Mesh<Yeoh>,
    intruder: &GridSdf,
    bounds: Aabb,
    cavity_offset_m: f64,
    cell_size_m: f64,
) -> Tet10Mesh<Yeoh> {
    let Some(faces6) = Mesh::<Yeoh>::boundary_faces6(&mesh).map(<[[VertexId; 6]]>::to_vec) else {
        // A linear mesh has no midsides to conform.
        return mesh;
    };
    let cavity = Solid::from_sdf(intruder.clone(), bounds).offset(cavity_offset_m);
    let band_m = CAVITY_MIDSIDE_BAND_CELLS * cell_size_m;
    let positions = Mesh::<Yeoh>::positions(&mesh);

    // Nodes 3..6 of a P2 boundary face are its edge midsides.
    let mut candidates: Vec<VertexId> = faces6.iter().flat_map(|f| f[3..6].to_vec()).collect();
    candidates.sort_unstable();
    candidates.dedup();

    let moves: Vec<(VertexId, Vec3)> = candidates
        .into_iter()
        .filter_map(|v| {
            let p = positions[v as usize];
            let sd = cavity.eval(Point3::from(p));
            // Outer-skin midsides sit a wall thickness away and must not move.
            if !sd.is_finite() || sd.abs() > band_m {
                return None;
            }
            // Two Newton steps on the SDF. The gradient of a signed distance
            // field is unit, so `q -= sd · ∇` lands on the zero level for an
            // exact field; twice absorbs the trilinear interpolant's error.
            let mut q = p;
            for _ in 0..2 {
                let s = cavity.eval(Point3::from(q));
                let g = cavity.grad(Point3::from(q));
                let n = g.norm();
                if !s.is_finite() || n < 1e-12 {
                    return None;
                }
                q -= (s / n) * g;
            }
            q.iter().all(|c| c.is_finite()).then_some((v, q))
        })
        .collect();

    mesh.with_projected_midsides(&moves, CAVITY_MIDSIDE_QUALITY_FLOOR)
}

/// **THE BRIDGE** — run the insertion ramp on a Tet10 mesh through the IPC
/// face barrier, at a caller-chosen residual tolerance.
///
/// The same scene, the same boundary conditions and the same rigid intruder as
/// [`run_insertion_ramp_at_kappa`]; what changes is the triple the solve is
/// built from — `Tet4` → [`Tet10`], `SdfMeshedTetMesh` → [`Tet10Mesh`],
/// `PenaltyRigidContact` → [`IpcRigidContact`] — plus a `κ` derived from a
/// measured traction instead of a swept one.
///
/// ⭐ **`tol` is a parameter because the claim lives there.** At the shipped
/// `INSERTION_SOLVE_TOL` (1e-1 N) "converged" can mean "the first residual was
/// already under 0.1 N", and #958 measured the real scan reaching full depth
/// that way while stalling at 25 % once a converged solution is actually
/// demanded. So the bridge is not asked whether it reaches depth — it is asked
/// whether it reaches depth *with the residual driven down*.
///
/// ▶ **What this does NOT change**, deliberately: `INSERTION_SOLVE_TOL`
/// itself (a behaviour change for every consumer, and its docstring says why
/// it is 1e-1), the shipped [`run_insertion_ramp`] entry point, and the
/// geometry pipeline. The Tet4 mesh is enriched here, per call — the scene is
/// meshed exactly as it always was.
///
/// ⭐ **The per-tet readouts are the Tet10 strain**, read at the four Gauss
/// points the solver evaluates the material at, through the enriched mesh's
/// own connectivity ([`InsertionRamp::readout_mesh`]). The four corners alone
/// would give the linear part of a quadratic field —
/// `the_readout_resolves_the_tet10_strain_at_every_gauss_point` measures that
/// gap.
///
/// # Errors
///
/// - `n_steps` is zero;
/// - the derived stiffness bracket is empty ([`bridge_face_barrier_kappa`]);
/// - [`outer_skin_bc`] finds no outer-skin vertex in the pin-band.
pub fn run_insertion_ramp_tet10_ipc(
    geometry: InsertionGeometry,
    n_steps: usize,
    tol: f64,
) -> Result<InsertionRamp> {
    // `κ` depends on the increment, so it cannot be a constant here — see
    // `bridge_face_barrier_kappa`. The inset is the ramp's full travel.
    // `n_steps` is small and non-zero; the cast is exact.
    #[allow(clippy::cast_precision_loss)]
    if n_steps == 0 {
        // Checked before deriving κ: a zero schedule would otherwise surface as
        // "the barrier floor is not derivable", which names the wrong problem.
        return Err(anyhow!("insertion ramp needs at least one step"));
    }
    let ramp_step_m = -geometry.cavity_offset_m / n_steps as f64;
    let kappa = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, ramp_step_m)?;
    run_insertion_ramp_tet10_ipc_at(geometry, n_steps, tol, kappa, BRIDGE_CONTACT_DHAT_M)
}

/// [`run_insertion_ramp_tet10_ipc`] with the barrier parameters supplied
/// rather than derived.
///
/// ⭐ **This is what makes the derivation falsifiable.** `κ` is claimed to be
/// determined by a measured traction rather than swept; the only way to hold
/// that claim to account is to be able to run the neighbouring stiffnesses and
/// see what they do. The derivation's own gate re-derives the bracket, and
/// `the_bridge_ramp_over_a_stiffness_sweep` reports the ramp across it.
///
/// # Errors
///
/// - `n_steps` is zero;
/// - [`outer_skin_bc`] finds no outer-skin vertex in the pin-band.
pub fn run_insertion_ramp_tet10_ipc_at(
    geometry: InsertionGeometry,
    n_steps: usize,
    tol: f64,
    contact_kappa: f64,
    d_hat: f64,
) -> Result<InsertionRamp> {
    if n_steps == 0 {
        return Err(anyhow!("insertion ramp needs at least one step"));
    }

    let InsertionGeometry {
        cavity_tensile_strength_pa,
        mesh: tet4_mesh,
        intruder,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets: _,
        per_tet_layer: _,
    } = geometry;

    // THE MESH SWAP. Corner ids and corner positions are preserved by
    // construction (`from_tet4` appends midsides at indices >= n_corners), so
    // every id the BCs and Γ already hold keeps pointing at the same material
    // point — see
    // `the_enriched_mesh_preserves_the_tet4_corners`.
    // ⚠ NOT conformed. `conform_cavity_midsides` exists and works — it moves
    // 4116 nodes and takes the tightest rest midside −0.2439 → −0.1031 mm —
    // but wiring it in did NOT fix the stall (same 6/16, residual 3.6438e4 →
    // 3.6429e4) and made the loaded gap worse. The rest sagitta was not the
    // cause; the face barrier LOADS midsides and leaves corners at ~0, so
    // midsides sitting closer to the surface is the load distribution, not an
    // artifact. Kept unwired rather than deleted: it is the right tool for a
    // curved-surface question, just not for this one.
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4_mesh);

    let n_vertices = mesh.n_vertices();
    let n_dof = 3 * n_vertices;

    // Full press-fit interference = the cavity inset, seated in `n_steps`
    // equal increments — and the increment is what sets the barrier floor, so
    // `κ` is derived here rather than read from a constant.
    let inset_m = -cavity_offset_m;
    // `n_steps` is small and non-zero; the cast is exact.
    #[allow(clippy::cast_precision_loss)]
    let ramp_step_m = inset_m / n_steps as f64;

    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot per-tet immutables before the mesh is consumed into per-step
    // solver clones — same dance as the Tet4 ramp. The readout mesh carries
    // the MIDSIDES, so the per-tet readouts are the Tet10 strain at the Gauss
    // points rather than the linear part of it. `referenced` includes midsides
    // (`referenced_vertex_mask` walks `tet_midside_nodes` too), which is what
    // makes it usable as the face-pair filter's set.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let readout_mesh = ReadoutMesh::tet10(&mesh)
        .context("the enriched mesh must name every element's midsides")?;
    // ⚠ `referenced_vertex_mask` walks `tet_midside_nodes` as well as
    // corners, so this set CONTAINS the midsides. That is load-bearing here:
    // on the face path every loaded node IS a midside, so a set of corners
    // alone would make the filter below delete the entire contact patch —
    // silently, as a clean empty readout.
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);

    // Γ is a property of the REST configuration, built once. Its faces are the
    // corner (3-node) boundary triangles, which `from_tet4` copies verbatim.
    let gamma_faces: Vec<[VertexId; 3]> = Mesh::<Yeoh>::boundary_faces(&mesh).to_vec();
    let gamma = GammaMask::build(
        &rest_positions,
        &gamma_faces,
        n_vertices,
        &intruder,
        cavity_offset_m,
    );

    let config = insertion_solver_config_at_tol(tol);

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
    // ⚠⚠ **IPC IS AN INTERIOR-POINT METHOD** — it needs a strictly feasible
    // start (`sd > 0`), and this ramp does not have one. At interference 0 the
    // cavity surface and the intruder coincide, so the very first increment
    // hands Newton a state already through the wall. Measured on the tolerance
    // fixture: the barrier's residual at Newton iteration 0 is **4.08e6 N**
    // and the line search stalls immediately. The penalty path tolerates that
    // start; a barrier cannot.
    //
    // So the intruder is marched in from a CLEARANCE, in increments of the
    // same size, and the approach steps are solved but not recorded. From
    // there the κ FLOOR carries it: that requirement is exactly "hold
    // `ρ · step` open", so each converged step leaves more standoff than the
    // next increment consumes and every later start is feasible by
    // construction. ⭐ The floor is not decoration here — it is what makes the
    // march legal.
    //
    // ⚠ The clearance is MEASURED off this mesh, not chosen. Two independent
    // effects put rest nodes on the wrong side of the cavity isosurface before
    // anything has moved:
    //   - the MESHER's own discretisation — measured on the tolerance fixture,
    //     corner nodes reach 0.1354 mm inside;
    //   - ENRICHMENT — `from_tet4` puts midsides on straight edge CHORDS, so a
    //     boundary midside sits under the true curved surface by the sagitta
    //     (`~h²/8R`). Measured: the tightest midside is 0.2522 mm inside, an
    //     excess of 0.117 mm over the tightest corner, against a predicted
    //     sagitta of 0.118 mm at h = 4 mm, R = 17 mm.
    // Marching from a round `d̂/2` walked straight back into an infeasible
    // start (the last approach step stalled at Newton iter 4). So the worst
    // rest penetration is measured and the approach begins outside it.
    // ⛔ `fold` from 0.0 floors the result: a mesh with no node inside the
    // surface asks for no penetration allowance, only the `d̂/2` margin.
    // ⛔⛔ **OVER `referenced`, NOT over `positions()`.** `positions()` is the
    // LATTICE, not the body: `SdfMeshedTetMesh` retains BCC lattice points that
    // no tet names, and plenty of them sit deep INSIDE the cavity. Measured on
    // the tolerance fixture, the worst "penetration" over all vertices is
    // 11.5427 mm — a lattice point near the cavity centre — against 0.2522 mm
    // over the vertices the solver actually sees. Taking the former inflates
    // the approach from 5 increments to 65 and the ramp never reaches its
    // recorded steps → [[feedback_positions_is_not_the_body]].
    let cavity_isosurface = Solid::from_sdf(intruder.clone(), bounds).offset(cavity_offset_m);
    let worst_rest_penetration_m = referenced
        .iter()
        .map(|&v| cavity_isosurface.eval(Point3::from(rest_positions[v as usize])))
        .filter(|sd| sd.is_finite())
        .fold(0.0_f64, |acc, sd| acc.max(-sd));
    // ⭐ The margin is `d̂`, not `d̂/2`: the barrier is INACTIVE only where
    // `sd ≥ d̂`, so that is what makes the first solve a genuinely unloaded
    // one — a rest state in equilibrium with no contact at all, which is
    // feasible by definition rather than by a margin someone chose. At `d̂/2`
    // the first solve already sits mid-band and must balance a barrier sized
    // for the FULL-seat traction against an unloaded wall.
    let approach_clearance_m = worst_rest_penetration_m + d_hat;
    // The increment count is small, so the cast is exact. `ceil` is deliberate:
    // rounding up can only make the first approach gap larger than the
    // clearance asked for, never smaller.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n_approach = (approach_clearance_m / ramp_step_m).ceil() as usize;

    // `(interference, record)` — the approach is solved, never reported, so
    // `InsertionRamp`'s steps stay the same 0 → inset grid the Tet4 ramp
    // reports and the two are directly comparable.
    let mut schedule: Vec<(f64, bool)> = Vec::with_capacity(n_approach + n_steps);
    for i in (1..=n_approach).rev() {
        // `i` is bounded by `n_approach`; the cast is exact.
        #[allow(clippy::cast_precision_loss)]
        schedule.push((-(i as f64) * ramp_step_m, false));
    }
    for k in 0..n_steps {
        // k + 1 and n_steps are tiny — well under f64's exact-integer ceiling.
        #[allow(clippy::cast_precision_loss)]
        schedule.push(((k + 1) as f64 / n_steps as f64 * inset_m, true));
    }

    for &(interference_m, record) in &schedule {
        let contact = intruder_ipc_contact_at(
            &intruder,
            bounds,
            interference_m,
            cavity_offset_m,
            contact_kappa,
            d_hat,
        );
        let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
            CpuNewtonSolver::new(Tet10, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        let solve_started = std::time::Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        let wall_time_s = solve_started.elapsed().as_secs_f64();
        match outcome {
            Ok(step) => {
                if record {
                    let positions_k: Vec<Vec3> = positions_from_flat(&step.x_final);
                    // The solver consumed the step's `contact`; rebuild at the
                    // SAME parameters so the reported patch is the solved one.
                    let readout_contact = intruder_ipc_contact_at(
                        &intruder,
                        bounds,
                        interference_m,
                        cavity_offset_m,
                        contact_kappa,
                        d_hat,
                    );
                    let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                    // ⚠ The face path's READOUT is per-NODE
                    // (`ContactPair::Vertex` naming midsides), not per-face —
                    // `ContactPair::Face` is what the SOLVER consumes. So the
                    // shipped filter applies unchanged and its `Face` arm stays
                    // genuinely unreachable.
                    let contact_readouts =
                        filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                    let per_tet = readout_mesh.readouts(&positions_k);
                    let conformity = gamma.conformity(
                        &positions_k,
                        &gamma_faces,
                        &contact_readouts,
                        cavity_tensile_strength_pa,
                    );
                    let step_readout =
                        aggregate_step_readout(&per_tet, &contact_readouts, conformity);

                    steps.push(RampStep {
                        interference_m,
                        iter_count: step.iter_count,
                        final_residual_norm: step.final_residual_norm,
                        x_final: step.x_final.clone(),
                        readout: step_readout,
                        wall_time_s,
                    });
                }
                x_prev_flat = step.x_final; // chain the warm start
            }
            Err(payload) => {
                failed_at_step = Some(steps.len());
                let why = panic_message(&*payload);
                failure_reason = Some(if record {
                    why
                } else {
                    format!(
                        "during the feasible-start approach, at interference \
                         {:.4} mm (the barrier could not be established before \
                         the ramp began): {why}",
                        interference_m * 1e3,
                    )
                });
                break;
            }
        }
    }

    let result = steps.last().map(|last| {
        let final_positions = positions_from_flat(&last.x_final);
        let final_per_tet = readout_mesh.readouts(&final_positions);
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
        readout_mesh,
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
/// contact-pair inner loop (`PenaltyRigidContact`'s `ActivePairsFor::active_pairs`)
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
/// `PenaltyRigidContact`'s `ActivePairsFor::active_pairs`.
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

/// A CONTINUOUS tangent along a polyline, for orienting a rigid body carried
/// along it.
///
/// ⛔ **Why [`point_along_polyline_at_arc_distance`]'s tangent cannot be used
/// for this.** That one returns the tangent of the *segment* the point falls
/// in — piecewise-constant, jumping by the full turn angle at every vertex.
/// For finding a POSITION that is fine. For orienting a rigid body it is not:
/// the body snaps through the whole turn the instant the walk crosses a
/// vertex, and a point far from the tip moves a long way for it.
///
/// Measured on `base_mold`: with
/// segment tangents the per-step normal closing **saturated at ~4.1 mm no
/// matter how fine the schedule** — 4.18 mm at 256 steps, 4.13 mm at 512 —
/// because the snap is set by vertex spacing, not by step size. Ratios of
/// closing-to-arc-step reached **an order of magnitude above 1**, which is not
/// a thing a rigid translation can do. With a continuous tangent the same
/// measurement scales with step size as it must.
///
/// Vertex tangents are the normalised sum of the two incident segment
/// tangents (the ends take their single neighbour), and within a segment the
/// two vertex tangents are blended by arc fraction. That is continuous across
/// vertices by construction — at a vertex both sides evaluate to the same
/// vertex tangent — and centres each turn on the vertex it belongs to.
///
/// Returns `None` for a degenerate polyline, matching its sibling.
fn smoothed_tangent_along_polyline(
    polyline: &[Point3<f64>],
    distance_m: f64,
) -> Option<Vector3<f64>> {
    if polyline.len() < 2 {
        return None;
    }
    // Segment tangents, skipping zero-length segments.
    let seg: Vec<(f64, Vector3<f64>)> = polyline
        .windows(2)
        .filter_map(|w| {
            let v = w[1].coords - w[0].coords;
            let n = v.norm();
            (n >= f64::EPSILON).then(|| (n, v / n))
        })
        .collect();
    if seg.is_empty() {
        return None;
    }
    // Vertex tangents: the normalised sum of incident segment tangents.
    let vertex_tangent = |i: usize| -> Vector3<f64> {
        let before = seg.get(i.wrapping_sub(1)).map(|s| s.1);
        let after = seg.get(i).map(|s| s.1);
        match (before, after) {
            (Some(a), Some(b)) => {
                let sum = a + b;
                // Exactly-antiparallel neighbours cancel; fall back to the
                // outgoing segment rather than normalising a zero vector.
                if sum.norm() < 1e-12 {
                    b
                } else {
                    sum.normalize()
                }
            }
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (None, None) => Vector3::z(),
        }
    };

    let target_m = distance_m.max(0.0);
    let mut walked_m = 0.0_f64;
    for (i, &(len, _)) in seg.iter().enumerate() {
        if walked_m + len >= target_m {
            let f = ((target_m - walked_m) / len).clamp(0.0, 1.0);
            let (a, b) = (vertex_tangent(i), vertex_tangent(i + 1));
            let blended = a * (1.0 - f) + b * f;
            return Some(if blended.norm() < 1e-12 {
                seg[i].1
            } else {
                blended.normalize()
            });
        }
        walked_m += len;
    }
    // Past the end: hold the final tangent, as the sibling does.
    Some(seg[seg.len() - 1].1)
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
/// ⭐ **Rotation included (the banked D-Slide2 iter-2 followup).** The pose
/// carries the rotation that takes the REST tangent (at arc distance 0) to
/// the tangent where the tip currently sits, applied about the tip. On a
/// curving path — which a real scan always is — a translation-only
/// pose slides the insertable *along* the curve without ever turning to face
/// it, so the contact it computes is not the contact the geometry implies.
/// The tangent was already being computed here and discarded.
///
/// `R` is taken between the two tangents and the translation is
/// `tip_world − R · tip_rest`, so the tip lands on the centerline and the
/// body swings with it. At `t = 1` the walk is zero, the two tangents are the
/// same vector, `R` is the identity and the whole pose collapses to the
/// identity — the seated rest pose, unchanged from iter-1.
///
/// ⚠ `rotation_between` returns `None` for exactly-antiparallel tangents (a
/// 180° doubling-back in the polyline). That is a degenerate centerline
/// rather than a pose to guess at, so it falls back to the identity rotation
/// — the iter-1 behaviour — rather than picking an arbitrary axis.
///
/// For a degenerate polyline (< 2 points or zero arc length), returns
/// identity — defensive; sliding-ramp drivers validate polyline shape up
/// front and surface their own error.
pub(crate) fn slide_pose_at(centerline: &[Point3<f64>], t: f64) -> Isometry3<f64> {
    if centerline.len() < 2 {
        return Isometry3::identity();
    }
    let l_m = polyline_arc_length_m(centerline);
    let tip_rest = centerline[0];
    let walk_from_tip = (l_m * (1.0 - t.clamp(0.0, 1.0))).max(0.0);
    let (tip_world, _) = point_along_polyline_at_arc_distance(centerline, walk_from_tip)
        .unwrap_or((tip_rest, Vector3::z()));
    // ⛔ The CONTINUOUS tangent, not the segment one — see
    // `smoothed_tangent_along_polyline` for the measurement that forced this.
    let tangent =
        smoothed_tangent_along_polyline(centerline, walk_from_tip).unwrap_or_else(Vector3::z);
    let rest_tangent = smoothed_tangent_along_polyline(centerline, 0.0).unwrap_or_else(Vector3::z);
    let rotation = Rotation3::rotation_between(&rest_tangent, &tangent)
        .map_or_else(UnitQuaternion::identity, UnitQuaternion::from);
    let translation = tip_world.coords - rotation * tip_rest.coords;
    Isometry3::from_parts(Translation3::from(translation), rotation)
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
/// same posture as `DEFAULT_N_STEPS` (the growing-ramp counterpart
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
/// (`docs/CAVITY_INSET_STALL_BOOKMARK.md` §9-§10). ⛔ F4's warm-up was
/// REVERTED (`docs/archive/F4_FALSIFICATION_POSTMORTEM.md`):
/// [`run_sliding_insertion_ramp`] passes `interference_m = 0` at every step,
/// so its seated contact is the cavity surface — no engineered interference.
/// `interference_m = 0` reproduces the pre-F4 shrunk-scan model bit-equal — analogous to
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
    /// via [`SlideRamp::readout_mesh`].
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
    /// What the per-tet readouts are evaluated over — the mesh this ramp
    /// SOLVED. Same contract as [`InsertionRamp::readout_mesh`].
    pub readout_mesh: ReadoutMesh,
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
/// sets only the active-set interior cutoff (`2 × cavity_inset_m`) in
/// [`intruder_contact_sliding_at`]; the contact itself carries no
/// engineered interference (`interference_m = 0`).
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
        cavity_tensile_strength_pa,
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

    // BCs are constant across the ramp (the outer skin does not move
    // for sliding either; per spec §3d the outer_skin_bc primitive is
    // shared unchanged with the growing ramp).
    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot per-tet immutables before consuming the mesh into the
    // per-step solver clones — same dance as the growing ramp.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let readout_mesh = ReadoutMesh::tet4(&mesh);
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);

    // Γ is a property of the REST configuration — which surface was meant to
    // contact — so it is built once here rather than per step. Its AREA is
    // re-measured per step on the deformed positions; see `GammaMask`.
    let gamma_faces: Vec<[VertexId; 3]> = Mesh::<Yeoh>::boundary_faces(&mesh).to_vec();
    let gamma = GammaMask::build(
        &rest_positions,
        &gamma_faces,
        Mesh::<Yeoh>::n_vertices(&mesh),
        &intruder,
        cavity_offset_m,
    );
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
                let per_tet = readout_mesh.readouts(&positions_k);
                let conformity = gamma.conformity(
                    &positions_k,
                    &gamma_faces,
                    &contact_readouts,
                    cavity_tensile_strength_pa,
                );
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts, conformity);

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
        let final_per_tet = readout_mesh.readouts(&final_positions);
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
        readout_mesh,
        intruder_poses,
    })
}

/// The intruder as an IPC face-barrier contact at a slide pose.
///
/// Sliding sibling of [`intruder_ipc_contact_at`]. The rigid primitive is
/// built exactly as the penalty path builds it — the same `TransformedSdf`
/// at the same pose, offset by `cavity_offset_m` — so nothing about the
/// scene's geometry moves across the bridge; only the contact law does.
///
/// ⭐ **No interior cutoff, and none is needed.** The penalty sibling passes
/// `2 · cavity_inset_m` to keep deep-interior lattice nodes out of the active
/// set. The face barrier is built from `boundary_faces6()`, which comes from
/// tet connectivity and names only surface nodes, so the interior is excluded
/// by construction rather than by a depth heuristic.
fn intruder_ipc_contact_sliding_at(
    intruder: &GridSdf,
    bounds: Aabb,
    slide_pose: Isometry3<f64>,
    cavity_offset_m: f64,
    kappa: f64,
    d_hat: f64,
) -> IpcRigidContact {
    let transformed = TransformedSdf::new(intruder.clone(), slide_pose);
    let intruder_solid = Solid::from_sdf(transformed, bounds).offset(cavity_offset_m);
    IpcRigidContact::with_params(vec![intruder_solid], kappa, d_hat)
}

/// The worst NORMAL closing one slide increment produces, measured on the
/// rest wall without solving anything.
///
/// ⭐⭐ **This is the number the sliding bridge's `κ` has to be derived from,
/// and it is NOT the arc increment.** The growing ramp advances along the
/// contact normal, so its increment and its closing are the same quantity. A
/// sliding ramp's are different quantities — but ⛔ **how different is a
/// property of the GEOMETRY and must be measured per scene, not assumed.**
///
/// An earlier revision of this comment asserted that "most of an arc step is
/// tangential once the insertable is inside". Measured on the tolerance
/// fixture that is **false**: closing 2.4069 mm against a 2.5 mm arc
/// increment, a ratio of **0.963**. That fixture's intruder is a sphere
/// entering a hole, so its surface is near-perpendicular to the motion and
/// almost the whole step closes. The product scan, an elongated insertable,
/// closes most of each step too (`what_the_sliding_contact_reaches_on_the_product_scan`).
///
/// ⇒ the consequence is a SCHEDULE requirement: [`bridge_face_barrier_kappa`]
/// refuses unless `closing < d̂`. The derivation refuses rather than guesses
/// when the schedule is too coarse.
///
/// Measured as `max(sd_k − sd_{k+1})` over the rest wall nodes, restricted to
/// nodes that END the step within `d_hat` of the intruder — a node still far
/// outside the band closes fast and costs nothing, because the barrier cannot
/// see it yet. The restriction is what makes this the *relevant* closing
/// rather than the largest one.
///
/// ⚠ **Measured on REST positions, which makes it an UPPER bound**: in the
/// real solve the wall is pushed away as the intruder arrives, so the gap
/// closes by less than this. A floor derived from it is therefore
/// conservative, which is the direction a floor should err.
fn sliding_normal_increment_m(
    intruder: &GridSdf,
    bounds: Aabb,
    centerline: &[Point3<f64>],
    n_steps: usize,
    cavity_offset_m: f64,
    d_hat: f64,
    wall_points: &[Vec3],
) -> f64 {
    let solid_at = |t: f64| {
        Solid::from_sdf(
            TransformedSdf::new(intruder.clone(), slide_pose_at(centerline, t)),
            bounds,
        )
        .offset(cavity_offset_m)
    };
    let mut worst = 0.0_f64;
    for k in 0..n_steps {
        // Step indices are tiny; the casts are exact.
        #[allow(clippy::cast_precision_loss)]
        let (t0, t1) = (k as f64 / n_steps as f64, (k + 1) as f64 / n_steps as f64);
        let (a, b) = (solid_at(t0), solid_at(t1));
        for p in wall_points {
            let pt = Point3::from(*p);
            let sd_after = b.eval(pt);
            // Only nodes inside the OPEN band at the end of the step. Two
            // exclusions, for different reasons:
            //  - `> d_hat`: the barrier cannot see it yet, so however fast it
            //    closes costs nothing.
            //  - `<= 0`: through the REST wall by the step's END — which also
            //    drops a node that crosses it within the step. Its rest gap is not
            //    its solve gap — the barrier pushes those nodes out — so
            //    reading a "closing" off the rest configuration there measures
            //    the mesh, not the schedule.
            if !sd_after.is_finite() || sd_after > d_hat || sd_after <= 0.0 {
                continue;
            }
            let sd_before = a.eval(pt);
            if sd_before.is_finite() {
                worst = worst.max(sd_before - sd_after);
            }
        }
    }
    worst
}

/// **THE SLIDING BRIDGE** — seat the intruder along its own centerline on a
/// Tet10 mesh through the IPC face barrier, at a caller-chosen tolerance.
///
/// The travelling model, which is the one a real fit actually is: the
/// insertable starts clear of the cavity and is carried in along the
/// centerline, turning to follow it ([`slide_pose_at`]), while the wall flexes
/// around it. [`run_insertion_ramp_tet10_ipc`] is the other model — a
/// coincident intruder inflated in place — and it answers a different
/// question.
///
/// ⭐⭐ **The sliding model suits IPC better than the growing one does.** IPC
/// is an interior-point method and needs a strictly feasible start; the
/// growing ramp has none by construction (at interference 0 the cavity
/// surface and the intruder coincide) and has to be given an approach march.
/// Here the intruder genuinely begins clear of the wall, so step 0 is feasible
/// with nothing added.
///
/// `κ` is derived per ramp from the measured normal closing
/// ([`sliding_normal_increment_m`]), not from the arc increment — see that
/// function for why the two are different quantities.
///
/// # Errors
///
/// - `n_steps` is zero, or the centerline has fewer than 2 points;
/// - the normal closing reaches the barrier band `d̂` — the slide schedule is
///   too coarse;
/// - [`outer_skin_bc`] finds no outer-skin vertex in the pin-band.
pub fn run_sliding_insertion_ramp_tet10_ipc(
    geometry: InsertionGeometry,
    centerline_polyline_m: &[Point3<f64>],
    n_steps: usize,
    tol: f64,
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
        cavity_tensile_strength_pa,
        mesh: tet4_mesh,
        intruder,
        cavity_offset_m,
        outer_offset_m,
        bounds,
        cell_size_m,
        n_tets: _,
        per_tet_layer: _,
    } = geometry;

    // THE MESH SWAP — corner ids and positions are preserved, so the BCs and
    // Γ keep pointing at the same material points.
    // ⚠ NOT conformed. `conform_cavity_midsides` exists and works — it moves
    // 4116 nodes and takes the tightest rest midside −0.2439 → −0.1031 mm —
    // but wiring it in did NOT fix the stall (same 6/16, residual 3.6438e4 →
    // 3.6429e4) and made the loaded gap worse. The rest sagitta was not the
    // cause; the face barrier LOADS midsides and leaves corners at ~0, so
    // midsides sitting closer to the surface is the load distribution, not an
    // artifact. Kept unwired rather than deleted: it is the right tool for a
    // curved-surface question, just not for this one.
    let mesh = Tet10Mesh::<Yeoh>::from_tet4(&tet4_mesh);
    let n_vertices = mesh.n_vertices();
    let n_dof = 3 * n_vertices;

    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    let n_pinned = bc.pinned_vertices.len();

    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    // Corners AND midsides — the per-tet readouts are the Tet10 strain.
    let readout_mesh = ReadoutMesh::tet10(&mesh)
        .context("the enriched mesh must name every element's midsides")?;
    // ⚠ Includes midsides — every loaded node on the face path IS a midside.
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);

    let gamma_faces: Vec<[VertexId; 3]> = Mesh::<Yeoh>::boundary_faces(&mesh).to_vec();
    let gamma = GammaMask::build(
        &rest_positions,
        &gamma_faces,
        n_vertices,
        &intruder,
        cavity_offset_m,
    );

    // The wall nodes the closing is measured on: boundary nodes only, which is
    // where contact can happen at all.
    let wall_points: Vec<Vec3> = {
        let mut ids: Vec<VertexId> = gamma_faces.iter().flatten().copied().collect();
        ids.sort_unstable();
        ids.dedup();
        ids.into_iter()
            .map(|v| rest_positions[v as usize])
            .collect()
    };
    let normal_increment_m = sliding_normal_increment_m(
        &intruder,
        bounds,
        centerline_polyline_m,
        n_steps,
        cavity_offset_m,
        BRIDGE_CONTACT_DHAT_M,
        &wall_points,
    );
    let contact_kappa = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, normal_increment_m)
        .map_err(|e| {
            anyhow!(
                "the sliding bridge's stiffness is not derivable at a normal closing of \
                 {normal_increment_m:.6e} m over {n_steps} steps: {e}"
            )
        })?;

    let l_m = polyline_arc_length_m(centerline_polyline_m);
    let config = insertion_solver_config_at_tol(tol);

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
        // `(k + 1) as f64 / n_steps as f64`: tiny integers.
        #[allow(clippy::cast_precision_loss)]
        let t = (k + 1) as f64 / n_steps as f64;
        let pose = slide_pose_at(centerline_polyline_m, t);
        let contact = intruder_ipc_contact_sliding_at(
            &intruder,
            bounds,
            pose,
            cavity_offset_m,
            contact_kappa,
            BRIDGE_CONTACT_DHAT_M,
        );
        let solver: CpuNewtonSolver<Tet10, Tet10Mesh<Yeoh>, IpcRigidContact, Yeoh, 10, 4> =
            CpuNewtonSolver::new(Tet10, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.try_replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        match outcome {
            Ok(Ok(step)) => {
                let positions_k: Vec<Vec3> = positions_from_flat(&step.x_final);
                let readout_contact = intruder_ipc_contact_sliding_at(
                    &intruder,
                    bounds,
                    pose,
                    cavity_offset_m,
                    contact_kappa,
                    BRIDGE_CONTACT_DHAT_M,
                );
                let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                let contact_readouts =
                    filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                let per_tet = readout_mesh.readouts(&positions_k);
                let conformity = gamma.conformity(
                    &positions_k,
                    &gamma_faces,
                    &contact_readouts,
                    cavity_tensile_strength_pa,
                );
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts, conformity);

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
                failed_at_step = Some(k);
                failure_reason = Some(solver_failure_message(&failure));
                break;
            }
            Err(payload) => {
                failed_at_step = Some(k);
                failure_reason = Some(panic_message(&*payload));
                break;
            }
        }
    }

    let result = steps.last().map(|last| {
        let final_positions = positions_from_flat(&last.x_final);
        let final_per_tet = readout_mesh.readouts(&final_positions);
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
        readout_mesh,
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

    /// An L-bent centerline: straight up `z`, then a right-angle turn into `x`.
    ///
    /// The bend is what makes rotation observable — on
    /// [`straight_z_centerline`] the tangent never changes, so every rotation
    /// test passes vacuously there.
    fn bent_centerline() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, 0.0), // tip (rest pose)
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 2.0), // the corner
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(2.0, 0.0, 2.0), // floor / cap mouth
        ]
    }

    /// The pose TURNS to follow a curved path, and still lands the tip on it.
    ///
    /// ⭐ The property that matters for a real scan: a translation-only
    /// pose slides the insertable *along* the curve without turning to face
    /// it, so the contact it computes is not the contact the geometry implies.
    ///
    /// Three things are asserted together because any one alone can pass on a
    /// broken pose: the rotation maps the REST tangent onto the LOCAL tangent,
    /// the tip still lands on the centerline (a rotation applied about the
    /// wrong origin would move it off), and the rotation is genuinely
    /// non-identity — without that last one this test passes on the iter-1
    /// translation-only code.
    #[test]
    fn slide_pose_at_turns_to_follow_a_curved_path() {
        let centerline = bent_centerline();
        let l_m = polyline_arc_length_m(&centerline);
        let (_, rest_tangent) = point_along_polyline_at_arc_distance(&centerline, 0.0)
            .expect("the bent centerline has a rest tangent");

        // t = 0.25 walks 0.75 · L = 3.0 m from the tip — past the corner at
        // 2.0 m, so the local tangent is +x while the rest tangent is +z.
        let t = 0.25;
        let walk = l_m * (1.0 - t);
        let (tip_world, local_tangent) = point_along_polyline_at_arc_distance(&centerline, walk)
            .expect("the walked point exists");
        assert!(
            (local_tangent - Vector3::x()).norm() < 1e-12,
            "fixture check: past the corner the tangent must be +x, got {local_tangent:?}",
        );

        let pose = slide_pose_at(&centerline, t);

        // 1. The rotation carries the rest tangent onto the local tangent.
        let turned = pose.rotation * rest_tangent;
        assert!(
            (turned - local_tangent).norm() < 1e-12,
            "the pose must turn the rest tangent {rest_tangent:?} onto the local \
             tangent {local_tangent:?}, got {turned:?}",
        );

        // 2. It is genuinely a turn — without this the iter-1 translation-only
        //    implementation passes everything above.
        let angle = pose.rotation.angle();
        assert!(
            angle > 1.0,
            "a right-angle bend must produce a large rotation; got {angle:.6} rad",
        );

        // 3. The tip still lands on the centerline.
        let tip_rest = centerline[0];
        let tip_mapped = pose * tip_rest;
        assert!(
            (tip_mapped - tip_world).norm() < 1e-12,
            "the tip must land at {tip_world:?}, got {tip_mapped:?} — a rotation \
             applied about the wrong origin moves the tip off the path",
        );
    }

    /// The pose's rotation is CONTINUOUS along the path — refining the
    /// schedule must refine the per-step turn.
    ///
    /// ⛔ **This is the gate for a defect that shipped in the first revision
    /// of the rotation and was caught only by a geometric measurement.**
    /// `point_along_polyline_at_arc_distance` returns the tangent of the
    /// SEGMENT a point falls in, which is piecewise-constant. Orienting a
    /// rigid body by it makes the body snap through a vertex's entire turn the
    /// instant the walk crosses that vertex — and a point far from the tip
    /// travels a long way for it.
    ///
    /// The symptom was that the measured per-step normal closing on the
    /// product scan **stopped shrinking when the schedule was refined**,
    /// saturating near 4.1 mm from 256 steps to 512, with closing-to-arc-step
    /// ratios far above 1 — which a rigid translation cannot produce.
    ///
    /// So the property asserted here is the one that failed: halving the step
    /// must roughly halve the largest consecutive rotation delta. Under
    /// segment tangents that delta is pinned to the vertex turn angle and does
    /// not move, whatever the schedule.
    #[test]
    fn the_slide_pose_rotation_refines_with_the_schedule() {
        let centerline = bent_centerline();
        let worst_turn = |n_steps: usize| -> f64 {
            let mut worst = 0.0_f64;
            for k in 0..n_steps {
                // Step indices are tiny; the casts are exact.
                #[allow(clippy::cast_precision_loss)]
                let (t0, t1) = (k as f64 / n_steps as f64, (k + 1) as f64 / n_steps as f64);
                let a = slide_pose_at(&centerline, t0).rotation;
                let b = slide_pose_at(&centerline, t1).rotation;
                worst = worst.max(a.rotation_to(&b).angle());
            }
            worst
        };

        let coarse = worst_turn(32);
        let fine = worst_turn(128);
        assert!(
            coarse > 1e-6,
            "the bent fixture must turn at all, or this gate is vacuous",
        );
        // Four times the steps must give materially less turn per step. The
        // bound is loose (a factor of 2, not 4) because the blend spreads each
        // turn over its two incident segments rather than exactly linearly —
        // what is being excluded is a delta that does not move at all.
        assert!(
            fine < coarse / 2.0,
            "refining 32 → 128 steps must refine the per-step turn: worst turn \
             went {coarse:.6} → {fine:.6} rad. A delta that does not shrink means \
             the tangent is piecewise-constant and the body is snapping at \
             polyline vertices",
        );
    }

    /// A straight path produces NO rotation — the control for the test above.
    ///
    /// Without this, "follows the tangent" could be implemented by rotating
    /// something arbitrary, and the seated pose would stop being the identity.
    #[test]
    fn slide_pose_at_does_not_turn_on_a_straight_path() {
        let centerline = straight_z_centerline();
        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let pose = slide_pose_at(&centerline, t);
            assert!(
                pose.rotation.angle().abs() < 1e-12,
                "a straight centerline must produce no rotation at t = {t}, got {:.3e} rad",
                pose.rotation.angle(),
            );
        }
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

    /// The PRODUCT scene — `base_mold`, configured as the CF Studio project
    /// on disk actually configured it.
    ///
    /// ⛔ **Not `sock_over_capsule`.** Every scan measurement in this module
    /// before 2026-09-22 used that one, and it is the wrong scene in three
    /// ways that each move the answer:
    ///
    /// | | sock_over_capsule | **base_mold** |
    /// |---|---|---|
    /// | cavity inset | 3 mm | **5 mm** |
    /// | layers | 10 mm Ecoflex+50 % Slacker + 3 mm DS20A | **17 mm DRAGON_SKIN_10A @ 25 % Slacker** |
    /// | centerline | **straight** | **curved** |
    ///
    /// The last one is why it matters most here: a dead-straight centerline
    /// makes `slide_pose_at`'s rotation an identity by construction, so the
    /// travelling model could not be told from a translation-only one. On
    /// `base_mold` it can.
    ///
    /// Values come from `base_mold.design.toml` / `.cfproject.json`, which is
    /// a project that ran all the way to `Print` with a 334.6 g pour plan.
    ///
    /// ⚠ **The project's plug carries RIDGES** (three rings, 1.8–2.0 mm deep,
    /// plus texture, side pinch and tip relief) and `SimDesign` has no notion
    /// of them — the sim's cavity is a smooth offset. So this is the right
    /// scan, the right inset and the right stack, on a SMOOTHER cavity than
    /// the one that gets poured.
    ///
    /// Returns `None` when the scan is absent, so probes can skip cleanly.
    /// `CF_SIM_RESEARCH_PRODUCT_SCAN` overrides the path.
    fn product_scene() -> Option<(IndexedMesh, Vec<Point3<f64>>, Vec<CapPlane>, SimDesign)> {
        let scan_path = std::env::var("CF_SIM_RESEARCH_PRODUCT_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/base_mold.cleaned.stl"),
            PathBuf::from,
        );
        if !scan_path.exists() {
            eprintln!("product scan absent at {}", scan_path.display());
            return None;
        }
        // `base_mold.cleaned.stl` → `base_mold.prep.toml`.
        let prep_path = scan_path.with_extension("").with_extension("prep.toml");
        let prep_text = std::fs::read_to_string(&prep_path)
            .map_err(|e| format!("{}: {e}", prep_path.display()))
            .expect("the prep.toml beside the product scan must load");
        let centerline =
            crate::parse_centerline(&prep_text).expect("parse centerline from the prep.toml");
        let caps =
            cf_cap_planes::parse_cap_planes(&prep_text).expect("parse caps from the prep.toml");
        let scan = load_stl(&scan_path).expect("load the product scan");
        // From `base_mold.design.toml` + `.cfproject.json`.
        let design = SimDesign {
            cavity_inset_m: 0.005,
            layers: vec![layer_with_slacker(0.017, "DRAGON_SKIN_10A", 0.25)],
        };
        Some((scan, centerline, caps, design))
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
        // Iterate `cf_device_types::LAYER_MATERIALS` so a new catalog
        // entry trips this test (rather than passing because the
        // hardcoded list happens to omit the new key).
        for (key, _label, _density) in cf_device_types::LAYER_MATERIALS {
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
    /// mesh carries `Some(max)` tensile cap (asymmetric one-sided
    /// bound per H4-2-C, see
    /// `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5) rather than
    /// the legacy `(None, None)` fallback that the 3-arg constructor
    /// produces.  Pinned at `silicone.validity_max_principal_stretch`
    /// for the single ECOFLEX_00_30 layer used in the fixture so a
    /// future refactor that flips `build_insertion_geometry` back to
    /// the bounds-less 3-arg constructor (or that silently drops the
    /// max bound) trips here.
    ///
    /// The compressive bound (`min_principal_stretch`) is dropped at
    /// `MaterialField::sample_yeoh` time per H4-2-C (`det F > 0`
    /// inversion is the only remaining compressive safety net); the
    /// per-anchor `validity_min_principal_stretch` value still flows
    /// through `MaterialField`'s internal `bounds` storage so a
    /// future Option B (Phase H F-bar / mixed-u-p decorator) work
    /// can re-enable the compressive gate by flipping
    /// `sample_yeoh`'s `with_max_principal_stretch_only` call back
    /// to `with_principal_stretch_bounds`.
    #[test]
    fn build_insertion_geometry_per_tet_yeoh_carries_calibrated_tensile_cap_only() {
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
        for (tet_id, yeoh) in materials.iter().enumerate() {
            let validity = yeoh.validity();
            assert_eq!(
                validity.max_principal_stretch,
                Some(expected_max),
                "tet {tet_id} max_principal_stretch must equal ECOFLEX_00_30's calibrated 0.8·λ_break"
            );
            assert!(
                validity.min_principal_stretch.is_none(),
                "tet {tet_id} min_principal_stretch must be None under H4-2-C \
                 asymmetric one-sided bound (got {:?})",
                validity.min_principal_stretch,
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
    /// override with the `CF_SIM_RESEARCH_SPIKE_SCAN` env var). Run:
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research insertion_sim -- --ignored --nocapture
    /// ```
    ///
    /// Skips gracefully (no failure) when the fixture is absent, so the
    /// `--ignored` sweep is portable across machines.
    #[test]
    #[ignore = "needs the repo-excluded iter-1 scan fixture; run with --ignored"]
    fn sdf_bridge_spike_on_iter1_scan() {
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
    /// `#[ignore]` — same repo-excluded fixture + `CF_SIM_RESEARCH_SPIKE_SCAN`
    /// override as [`sdf_bridge_spike_on_iter1_scan`]; skips
    /// gracefully when the fixture is absent. Run:
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research insertion_sim -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "needs the repo-excluded iter-1 scan fixture; run with --ignored"]
    fn build_insertion_geometry_on_iter1_scan() {
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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

    /// One Tet4 element over the first four of `rest`, for the readout tests.
    fn unit_tet_readout_mesh(rest: Vec<Vec3>) -> ReadoutMesh {
        ReadoutMesh {
            rest_positions: rest,
            elements: ReadoutElements::Tet4(vec![[0, 1, 2, 3]]),
            materials: vec![unit_tet_material()],
        }
    }

    /// Undeformed tet ⇒ `F = I`. `Yeoh::first_piola(I)` ⇒ zero stress.
    /// `Yeoh::energy(I)` ⇒ zero strain energy. Principal stretches
    /// ⇒ `[1, 1, 1]`. The bedrock of every other F-reconstruction
    /// assertion.
    #[test]
    fn tet_readout_undeformed_is_identity() {
        let rest = unit_tet_rest();
        let curr = rest.clone();
        let readout = unit_tet_readout_mesh(rest).gauss_point_readouts(0, &curr)[0].clone();

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

    /// Pure rigid translation ⇒ `F = I`. Tests that the Jacobian
    /// construction in the readout is translation-invariant (sanity check
    /// on the `J_curr · J_rest⁻¹` formula).
    #[test]
    fn tet_readout_pure_translation_is_identity() {
        let rest = unit_tet_rest();
        let t = Vec3::new(0.5, -0.3, 1.7);
        let curr: Vec<Vec3> = rest.iter().map(|p| p + t).collect();
        let f = unit_tet_readout_mesh(rest).gauss_point_readouts(0, &curr)[0].f;
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
        let readout = unit_tet_readout_mesh(rest).gauss_point_readouts(0, &curr)[0].clone();

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

    /// A 1 mm square split into two triangles, lying on the plane x = `at_x`.
    ///
    /// ⚠ Deliberately SMALL. The Γ tolerance is the face's own mean edge
    /// length, so a unit-sized triangle has `L_e ≈ 1.14` and would swallow any
    /// realistic offset whole — an earlier version of these fixtures used unit
    /// squares and could not tell the cavity level from the zero level.
    const FACE_MM: f64 = 0.001;

    fn square_at(at_x: f64) -> (Vec<Vec3>, Vec<[VertexId; 3]>) {
        let e = FACE_MM;
        let pos = vec![
            Vec3::new(at_x, 0.0, 0.0),
            Vec3::new(at_x, e, 0.0),
            Vec3::new(at_x, e, e),
            Vec3::new(at_x, 0.0, e),
        ];
        (pos, vec![[0, 1, 2], [0, 2, 3]])
    }

    /// An SDF whose zero level is the plane `x = 0`, so the isosurface at
    /// `level = d` is the plane `x = d`. Exact, so Γ membership is decidable
    /// by hand.
    struct PlaneX;

    impl Sdf for PlaneX {
        fn eval(&self, p: nalgebra::Point3<f64>) -> f64 {
            p.x
        }
        fn grad(&self, _p: nalgebra::Point3<f64>) -> Vec3 {
            Vec3::new(1.0, 0.0, 0.0)
        }
    }

    /// Γ membership is read at the CAVITY level, not the zero level. An
    /// implementation that ignored `cavity_offset_m` would pass a zero-level
    /// test and silently select the wrong surface on every real scene.
    #[test]
    fn gamma_mask_selects_the_cavity_level_not_the_zero_level() {
        let (pos, faces) = square_at(0.010);
        let on = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 0.010);
        assert_eq!(on.n_faces(), 2, "both triangles sit on x = 0.010");

        let off = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 0.0);
        assert_eq!(off.n_faces(), 0, "nothing sits on x = 0");
    }

    /// `|Γ|` is measured on the positions handed in, not baked at build time —
    /// the step loop passes DEFORMED positions because
    /// `ContactPairReadout::tributary_area` is a deformed area.
    #[test]
    fn gamma_area_tracks_the_supplied_positions() {
        let (rest, faces) = square_at(0.010);
        let mask = GammaMask::build(&rest, &faces, rest.len(), &PlaneX, 0.010);

        let readout = |vid: VertexId| ContactPairReadout {
            pair: ContactPair::Vertex {
                vertex_id: vid,
                primitive_id: 0,
            },
            position: Vec3::zeros(),
            sd: -1e-4,
            normal: Vec3::new(1.0, 0.0, 0.0),
            force_on_soft: Vec3::new(1.0, 0.0, 0.0),
            tributary_area: 0.25,
            pressure: 1.0e5,
        };
        let readouts: Vec<ContactPairReadout> = (0..4).map(readout).collect();

        let rest_score = mask
            .conformity(&rest, &faces, &readouts, 1.379e6)
            .expect("Γ is non-empty");

        // Stretch the square 2x in z: |Γ| doubles, so coverage halves.
        let stretched: Vec<Vec3> = rest
            .iter()
            .map(|p| Vec3::new(p.x, p.y, p.z * 2.0))
            .collect();
        let stretched_score = mask
            .conformity(&stretched, &faces, &readouts, 1.379e6)
            .expect("Γ is still non-empty");

        assert!(
            stretched_score.breakdown.coverage < rest_score.breakdown.coverage * 0.6,
            "doubling |Γ| must roughly halve coverage: {} vs {}",
            stretched_score.breakdown.coverage,
            rest_score.breakdown.coverage,
        );
    }

    /// Readouts OFF Γ must be excluded from the score.
    ///
    /// ⚠ Every other Γ test here places its readouts on Γ, so a `contains`
    /// that accepted everything survived them all. This is the gate that
    /// catches it: contacts on the outer envelope — which can never be
    /// intended contact — must not inflate coverage.
    #[test]
    fn readouts_off_gamma_are_excluded_from_the_score() {
        // Two surfaces: Γ at x = 0.010, an "envelope" at x = 0.050.
        let e = FACE_MM;
        let mut pos = Vec::new();
        let mut faces = Vec::new();
        let mut base: VertexId = 0;
        for x in [0.010_f64, 0.050] {
            pos.push(Vec3::new(x, 0.0, 0.0));
            pos.push(Vec3::new(x, e, 0.0));
            pos.push(Vec3::new(x, e, e));
            faces.push([base, base + 1, base + 2]);
            base += 3;
        }
        let mask = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 0.010);
        assert_eq!(mask.n_faces(), 1, "only the x = 0.010 face is Γ");

        let readout = |vid: VertexId| ContactPairReadout {
            pair: ContactPair::Vertex {
                vertex_id: vid,
                primitive_id: 0,
            },
            position: Vec3::zeros(),
            sd: -1e-4,
            normal: Vec3::new(1.0, 0.0, 0.0),
            force_on_soft: Vec3::new(1.0, 0.0, 0.0),
            // Each readout carries a third of the Γ face's area, so the three
            // Γ vertices alone tile it exactly and coverage lands at 1.
            tributary_area: 0.5 * e * e / 3.0,
            // Well past p_th (5 % of tensile = 69 kPa) so the coverage
            // logistic is fully saturated — at 100 kPa it reads 0.99975 and a
            // 1e-6 assertion fails on correct behaviour.
            pressure: 5.0e5,
        };
        // Vertices 0..3 are on Γ; 3..6 are on the envelope.
        let all: Vec<ContactPairReadout> = (0..6).map(readout).collect();

        let score = mask
            .conformity(&pos, &faces, &all, 1.379e6)
            .expect("Γ is non-empty");

        assert!(
            (score.breakdown.coverage - 1.0).abs() < 1e-6,
            "only the three Γ readouts may count — accepting all six would \
             roughly double coverage. got {}",
            score.breakdown.coverage,
        );
    }

    /// A non-finite ceiling is refused, not scored.
    ///
    /// ⛔ `p_th` derives from tensile strength, so a `NaN` ceiling makes every
    /// term `NaN`; `score_with` drops all four and returns `0.0`, which ranks
    /// ABOVE a measured poor design scoring negative.
    /// `SiliconeMaterial::from_measured` produces exactly this ceiling on
    /// purpose, so a design using a measured inner layer would otherwise score
    /// better than a real one.
    #[test]
    fn a_non_finite_ceiling_is_refused_rather_than_scored() {
        let (pos, faces) = square_at(0.010);
        let mask = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 0.010);
        assert!(
            mask.n_faces() > 0,
            "Γ must be non-empty or this proves nothing"
        );

        let readouts = vec![ContactPairReadout {
            pair: ContactPair::Vertex {
                vertex_id: 0,
                primitive_id: 0,
            },
            position: Vec3::zeros(),
            sd: -1e-4,
            normal: Vec3::new(1.0, 0.0, 0.0),
            force_on_soft: Vec3::new(1.0, 0.0, 0.0),
            tributary_area: 1.0e-7,
            pressure: 5.0e5,
        }];

        // A finite ceiling scores.
        assert!(
            mask.conformity(&pos, &faces, &readouts, 1.379e6).is_some(),
            "a finite ceiling must produce a score",
        );
        // NaN, infinite and non-positive ceilings must not.
        for bad in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            assert!(
                mask.conformity(&pos, &faces, &readouts, bad).is_none(),
                "ceiling {bad} must be refused, not scored",
            );
        }
    }

    /// A Tet10 `Face` pair is judged by its nodes, not dropped.
    ///
    /// ⚠ `insertion_sim` runs Tet4 today and emits only `Vertex` pairs, so no
    /// other test here exercises this arm — a mutation that returned `false`
    /// for every `Face` survived them all. When the Tet10 face barrier lands
    /// (renovation item 3) that would silently discard the entire contact set
    /// and report coverage 0 on a perfectly good design.
    #[test]
    fn tet10_face_pairs_are_judged_by_their_nodes() {
        let (pos, faces) = square_at(0.010);
        let mask = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 0.010);
        assert!(
            mask.n_faces() > 0,
            "Γ must be non-empty for this to mean anything"
        );

        let face_pair = |nodes: [VertexId; 6]| ContactPairReadout {
            pair: ContactPair::Face {
                nodes,
                primitive_id: 0,
                rest_area: 1.0e-6,
            },
            position: Vec3::zeros(),
            sd: -1e-4,
            normal: Vec3::new(1.0, 0.0, 0.0),
            force_on_soft: Vec3::new(1.0, 0.0, 0.0),
            tributary_area: 1.0e-7,
            pressure: 5.0e5,
        };

        // All six nodes are Γ vertices (0..4 exist on the square) -> counted.
        let on_gamma = mask
            .conformity(&pos, &faces, &[face_pair([0, 1, 2, 0, 1, 2])], 1.379e6)
            .expect("Γ non-empty");
        assert!(
            on_gamma.breakdown.coverage > 0.0,
            "a Face pair on Γ must contribute, got coverage {}",
            on_gamma.breakdown.coverage,
        );

        // No node is a Γ vertex -> excluded, so coverage collapses to zero.
        let off_gamma = mask
            .conformity(
                &pos,
                &faces,
                &[face_pair([90, 91, 92, 93, 94, 95])],
                1.379e6,
            )
            .expect("Γ non-empty");
        assert!(
            off_gamma.breakdown.coverage.abs() < 1e-12,
            "a Face pair with no Γ node must not contribute, got {}",
            off_gamma.breakdown.coverage,
        );
    }

    /// End-to-end: a real ramp must produce a usable conformity score.
    ///
    /// `#[ignore]` because it runs a full FEM ramp (~24k tets) — too slow for
    /// the default suite, but the only test here that exercises the reward on
    /// an actual scene rather than a synthetic fixture. Run with
    /// `--ignored --nocapture` to see the numbers.
    ///
    /// ★ **This test exists because a read-through could not have found what
    /// it found.** Every other gate on this path uses hand-built pressure
    /// arrays; running it once revealed `P_TH_FRACTION_OF_TENSILE` was
    /// mis-scaled by ~25×, putting the contact threshold *inside* the
    /// operating pressure distribution so that coverage measured "fraction
    /// above 69 kPa" rather than "fraction in contact".
    ///
    /// ⚠ Gates invariants and the threshold separation, **not** exact values —
    /// solver detail and mesher version move the numbers, and pinning them
    /// would produce a test that fails on correct changes.
    #[test]
    #[ignore = "full FEM ramp; run with --ignored"]
    fn real_ramp_produces_a_usable_conformity_score() {
        let scan = small_test_cube();
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.005, "ECOFLEX_00_30")],
        };
        let geometry =
            build_insertion_geometry(&scan, &design, &[], 2_500, 0.004).expect("geometry builds");
        let tensile = geometry.cavity_tensile_strength_pa;
        assert!(
            tensile.is_finite() && tensile > 0.0,
            "the innermost layer must carry a tensile strength, got {tensile}",
        );
        let p_th = sim_soft::readout::conformity::P_TH_FRACTION_OF_TENSILE * tensile;

        let ramp = run_insertion_ramp(geometry, 3).expect("ramp runs");
        let scored: Vec<&StepReadout> = ramp
            .steps
            .iter()
            .map(|st| &st.readout)
            .filter(|r| r.conformity.is_some())
            .collect();
        assert!(
            !scored.is_empty(),
            "no step produced a conformity score — Γ empty, or the ramp \
             converged nothing ({} step(s) recorded)",
            ramp.steps.len(),
        );

        for r in &scored {
            let c = r.conformity.as_ref().expect("filtered to Some");
            println!(
                "  pairs {:4} | F {:7.3} N | unif {:+.4} cov {:.6} peak {:+.4} \
                 | p_peak {:.0}/{:.0} Pa | peak/p_th {:.1} | lq {:.3}",
                r.n_active_contact_pairs,
                r.contact_force_magnitude_n,
                c.breakdown.pressure_uniformity,
                c.breakdown.coverage,
                c.breakdown.peak_bound,
                c.p_peak_smoothed,
                c.p_peak_true,
                c.p_peak_true / p_th,
                c.lq_ratio,
            );

            assert_eq!(
                c.non_finite_pressures, 0,
                "a non-finite pressure means the readout path is mis-reporting \
                 the force distribution, not that elements are unloaded",
            );
            assert!(
                !c.coverage_overflow,
                "coverage {} exceeded |Γ| — Γ's vertex set spilled onto faces \
                 absent from the denominator. On this cube fixture Γ is closed \
                 (its 1202 vertices' tributary areas sum to exactly |Γ|), so a \
                 flag here means the Γ rule or the area accounting changed",
                c.breakdown.coverage,
            );
            assert!(
                (0.0..=1.0).contains(&c.breakdown.coverage),
                "coverage must stay in [0, 1]. got {}",
                c.breakdown.coverage,
            );
            assert!(
                c.breakdown.pressure_uniformity <= 0.0,
                "R_unif = -J_unif is a cost, so it can never be positive, got {}",
                c.breakdown.pressure_uniformity,
            );
            assert!(
                c.p_peak_smoothed <= c.p_peak_true + 1e-9,
                "the L^q max must not exceed the true max: {} vs {}",
                c.p_peak_smoothed,
                c.p_peak_true,
            );
            assert!(
                c.breakdown.stiffness_bound.is_nan(),
                "k_min has no source; the term must stay NaN",
            );

            // ★ The regression guard for the mis-scaling this test found.
            assert!(
                c.p_peak_true / p_th > 10.0,
                "p_th ({p_th:.0} Pa) must sit WELL BELOW the operating pressure \
                 (peak {:.0} Pa, ratio {:.2}). At a ratio near 1 the threshold \
                 falls inside the contact-pressure distribution and coverage \
                 stops discriminating contact from no-contact — that is the \
                 defect this test was written after.",
                c.p_peak_true,
                c.p_peak_true / p_th,
            );
        }
    }

    /// An empty Γ yields `None`, not a flattering score. A scene whose cavity
    /// level selects nothing is a setup defect, and a number computed against
    /// no surface would hide it.
    #[test]
    fn empty_gamma_reports_none_rather_than_a_score() {
        let (pos, faces) = square_at(0.010);
        let mask = GammaMask::build(&pos, &faces, pos.len(), &PlaneX, 9.0);
        assert_eq!(mask.n_faces(), 0);
        assert!(
            mask.conformity(&pos, &faces, &[], 1.379e6).is_none(),
            "no Γ must produce no score",
        );
    }

    /// `aggregate_step_readout` over an empty per-tet slice + empty
    /// contact readouts must produce finite zeros — degenerate but
    /// non-panicking.
    #[test]
    fn aggregate_step_readout_empty_is_zeroed() {
        let r = aggregate_step_readout(&[], &[], None);
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
            first_piola_frobenius_pa: frob,
            energy_density_j_per_m3: psi,
            min_principal_stretch: stretches.into_iter().fold(f64::INFINITY, f64::min),
            max_principal_stretch: stretches.into_iter().fold(f64::NEG_INFINITY, f64::max),
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
                tributary_area: 1.0,
                pressure: 3.0,
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
                tributary_area: 1.0,
                pressure: 5.0,
            },
        ];

        let r = aggregate_step_readout(&per_tet, &readouts, None);
        assert_eq!(r.n_active_contact_pairs, 2);
        assert!((r.contact_force_total_n.z - 8.0).abs() < 1e-12);
        assert!((r.contact_force_magnitude_n - 8.0).abs() < 1e-12);
        assert!((r.max_principal_stretch - 1.5).abs() < 1e-12);
        assert!((r.min_principal_stretch - 0.8).abs() < 1e-12);
        assert!((r.max_first_piola_frobenius_pa - 2.0e5).abs() < 1e-9);
        assert!((r.mean_strain_energy_density_j_per_m3 - 2.0).abs() < 1e-12);
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
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research insertion_sim -- --ignored --nocapture
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
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research diagnose_iter1 -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "7.3a diagnostic — needs the repo-excluded iter-1 scan; run with --ignored --nocapture"]
    #[allow(clippy::cast_precision_loss)] // diagnostic counters → f64 for %/grid math
    fn diagnose_iter1_scan_geometry() {
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
        let out_dir = std::env::temp_dir().join("cf_sim_research_diag");
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
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research grid_sdf_fix_spike -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "7.3a fix spike — needs the repo-excluded iter-1 scan; run with --ignored --nocapture"]
    #[allow(clippy::cast_precision_loss)] // diagnostic counters → f64 for %/grid math
    fn grid_sdf_fix_spike() {
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
    /// cargo test -p cf-sim-research --release \
    ///     --bin cf-sim-research run_insertion_ramp_on_synthetic \
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
            for s in [tr.min_principal_stretch, tr.max_principal_stretch] {
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
        let path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
    /// GUI-default 10+3 mm dual-layer stack: ECOFLEX_00_30 + 50 %
    /// Slacker INNER 10 mm + DRAGON_SKIN_20A OUTER 3 mm (soft +
    /// tacky inside, firm skin outside).  Falsification artifact
    /// for H4-2-C asymmetric one-sided bound — re-runnable
    /// regression gate that maps `cargo test` outcomes onto the
    /// user's GUI visual gate behavior under H4-2-C.
    ///
    /// **Layer order** is `innermost-first` per `SimDesign.layers`
    /// docstring.  Pre-H4-arc revision of this test (`60e649d2`)
    /// had the layers INVERTED (DS20A inner / Ecoflex outer
    /// without Slacker), which produced different convergence
    /// behavior than the GUI — the §5.6 puzzle PARTIAL
    /// RESOLUTION in `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md`.
    /// This revision aligns the test substrate with the GUI
    /// default per the user-driven §5.7 visual gate.
    ///
    /// **Pre-H4 baseline**: cavity > 5 mm fake-converged step 1
    /// into an invalid Yeoh state and panicked at step 2 (per
    /// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10) — or,
    /// post-commit `2739717e` end-of-solve check, panics honestly
    /// at step 1.
    ///
    /// **Post-H4-2-C outcome** (per bookmark §5.4-§5.7):
    /// asymmetric one-sided bound drops the compressive gate at
    /// `MaterialField::sample_yeoh` time, letting Newton iterate
    /// through deep-compression equilibria.  Cavity 5 mm clears
    /// 16/16 (full seat) + cavity 8 mm clears 12/16 (75 % seat)
    /// in the GUI visual gate; the 6 + 7 mm gap is a Newton
    /// convergence wall (genuine, not a Yeoh validity firing) —
    /// separate sub-arc (slide-step bisection / N_STEPS sweep).
    ///
    /// Run:
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release \
    ///     h4_sweep_sliding_ramp_on_iter1_scan -- --ignored --nocapture
    /// ```
    ///
    /// `#[ignore]` — needs the repo-excluded iter-1 scan + prep.toml
    /// + a release-mode 5-cavity ramp; total wall-clock ~10-15 min.
    #[test]
    #[ignore = "H4.3 sweep — needs the iter-1 scan + a release ramp; run with --ignored --nocapture"]
    fn h4_sweep_sliding_ramp_on_iter1_scan() {
        let scan_path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
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
        let prep_text = std::fs::read_to_string(&prep_path).expect("load iter-1 prep.toml");
        let centerline =
            crate::parse_centerline(&prep_text).expect("parse centerline from prep.toml");
        assert!(
            centerline.len() >= 2,
            "iter-1 prep.toml must carry a centerline polyline",
        );
        // Cap planes from the same prep.toml — the GUI loads these via
        // `cf_cap_planes::parse_cap_planes` and threads them into
        // `build_insertion_geometry`'s `cap_planes` arg.  Pre-polish
        // versions of this test passed `&[]` (no caps) which routed
        // `pinned_floor_shell` through the closed-cavity short-circuit
        // — a structurally different FEM problem than the GUI's
        // open-mouth-with-floor-pinned-at-cap topology.  iter-1's
        // prep.toml has one `[[caps.loops]]` record at z ≈ -53 mm
        // with `included = true`.
        let cap_planes =
            cf_cap_planes::parse_cap_planes(&prep_text).expect("parse cap planes from prep.toml");
        let scan = load_stl(&scan_path).expect("load the iter-1 cleaned scan");

        let cavities_mm = [3.0_f64, 5.0, 6.0, 7.0, 8.0];
        let n_steps = 16_usize;
        let cell_size_m = 0.004_f64;

        eprintln!(
            "H4.3 sweep — iter-1 sock_over_capsule.cleaned.stl, dual-layer \
             Ecoflex 00-30 + 50% Slacker INNER 10 mm + DS20A OUTER 3 mm \
             (iter-1 GUI default), n_steps = {}, centerline {} points, \
             {} cap plane(s)",
            n_steps,
            centerline.len(),
            cap_planes.len(),
        );
        eprintln!(
            "  H4-2-C asymmetric one-sided bound: only the tensile cap \
             reaches the solver gate; compressive `min_principal_stretch` \
             is dropped at `MaterialField::sample_yeoh`."
        );
        eprintln!(
            "  Ecoflex 00-30 max_principal_stretch (inner pre-Slacker): {:.2}",
            ECOFLEX_00_30.validity_max_principal_stretch,
        );
        eprintln!(
            "  DS20A max_principal_stretch (outer): {:.2}",
            DRAGON_SKIN_20A.validity_max_principal_stretch,
        );

        for &cavity_mm in &cavities_mm {
            let cavity_inset_m = cavity_mm * 1e-3;
            eprintln!("\n--- cavity = {cavity_mm:.1} mm ---");
            let design = SimDesign {
                cavity_inset_m,
                // `SimDesign.layers` is innermost-first.  Iter-1 GUI
                // default: soft + tacky inside (Ecoflex 00-30 + 50 %
                // Slacker lerped to Shore 000-20), firm skin outside
                // (DS20A).  Verified against the user-driven §5.7
                // visual-gate sweep at
                // `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md`.
                layers: vec![
                    layer_with_slacker(0.010, "ECOFLEX_00_30", 0.5),
                    layer(0.003, "DRAGON_SKIN_20A"),
                ],
            };
            let t0 = Instant::now();
            let geometry =
                match build_insertion_geometry(&scan, &design, &cap_planes, 2_500, cell_size_m) {
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
    /// cargo test -p cf-sim-research --release \
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
        // ECOFLEX_00_30's 8.00 tensile cap through H4-2-C
        // asymmetric one-sided routing (the 0.20 compressive cap
        // is sampled but dropped at
        // `MaterialField::sample_yeoh` per
        // `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5).  Keeps
        // the analytical-sphere recon experiment apples-to-apples
        // with the production GridSdf geometry.
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
            roller_vertices: Vec::new(),
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
    /// the same `cf_cap_planes` primitive feeds both the sim's
    /// `build_insertion_geometry` and cf-device-geometry's cavity
    /// iso-extraction in cf-device-design).
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
    /// `with_params`); regression to `with_params` puts the probe
    /// vertex in the active set here. Verified by disabling the
    /// cutoff: the gate fails with one pair on the probe vertex.
    ///
    /// **Scope**: this test is a wire-up gate, NOT a filter-math gate.
    /// The strict-vs-non-strict, sign-convention, and band-gate
    /// semantics of the cutoff filter itself are pinned in
    /// `sim/L0/soft/tests/penalty_interior_cutoff.rs` against a
    /// `RigidPlane` fixture with precisely-controllable sd. Here the
    /// icosphere `GridSdf` is the load-bearing surface — we're only
    /// asserting that cf-sim-research's contact-build call site
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
        // The probe must sit at a TET-REFERENCED vertex index.
        // `active_pairs` filters the BCC lattice's orphan corners out
        // of the active set, and vertex 0 of this mesh is one of them
        // — a bare `vec![probe]` (index 0) would be excluded by
        // incidence before the cutoff ever ran, and this gate would
        // pass whether or not the cutoff was wired up.
        let probe_vertex = sim_soft::referenced_vertices(&geometry.mesh)[0];
        let mut probe = geometry.mesh.positions().to_vec();
        probe[probe_vertex as usize] = Vec3::new(0.0, 0.0, 0.0);
        let pairs = contact.active_pairs(&geometry.mesh, &probe);
        // Assert about the probe vertex only — the rest of the mesh
        // sits at its own rest positions and may legitimately be in
        // contact with the intruder at this pose.
        let probe_pairs = pairs
            .iter()
            .filter(|p| {
                matches!(
                    **p,
                    sim_soft::ContactPair::Vertex { vertex_id, .. } if vertex_id == probe_vertex
                )
            })
            .count();
        assert!(
            probe_pairs == 0,
            "deep-interior probe at body center (composed sd ≈ -37 mm) must be \
             excluded by the 6 mm interior_cutoff; vertex {probe_vertex} got \
             {probe_pairs} pairs",
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
    /// cargo test -p cf-sim-research --release \
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

    // ── renovation item 4 (the bridge), step 0 ──────────────────────
    //
    // What this pipeline's "converged" means today, measured rather than
    // asserted, so the bridge has something to be compared against.
    //
    // The three `#[ignore]`d ramps all reach 16/16 to their full 3 mm
    // inset, which reads as a solved problem. It is not: at the shipped
    // `INSERTION_SOLVE_TOL` of 1e-1 N, "converged" can mean "the first
    // residual was already under the bar". Re-running those ramps at
    // 1e-6 splits them — both synthetic fixtures still reach full depth
    // (the analytical shell is *better* behaved there, 4-7 iters at
    // ~1e-7, against 13 of 16 steps taking a single iteration at 1e-1),
    // while the real iter-1 scan stalls at step 4 of 16, at 0.75 mm of
    // the 3 mm inset, with an Armijo stall at r_norm 4.1e-3.
    //
    // So the shipped tolerance costs 4x the usable depth ON THE PRODUCT
    // GEOMETRY and only iterations on the idealised ones. That is the
    // finding the bridge has to move, and neither synthetic ramp can see
    // it — which is why the fixture below is chosen for CONDITIONING
    // rather than for size.

    /// The smallest synthetic scene that exercises the same conditioning
    /// the real scan runs into, so a CI-runnable gate can carry a claim
    /// the repo-excluded scan cannot.
    ///
    /// Found by search, not taste: sweeping radius / wall / cell /
    /// interference, its neighbours converge to 1e-6 in 9-14 iterations
    /// (wall 6 mm and 5 mm at this radius, and every configuration at
    /// [`TOL_FIXTURE_SHALLOW_M`]), while this one is the one that gets
    /// hard at [`TOL_FIXTURE_DEEP_M`] — 9 258 tets against the scan's
    /// 68 087, and no licensed fixture.
    ///
    /// ⚠ **How hard it gets is platform-dependent and no gate may assert
    /// it.** On macOS/ARM the deep step Armijo-stalls at Newton iter 108
    /// with `r_norm` 2.78e-3 — the same mode and decade as the scan's
    /// 4.13e-3, which is what made this scene worth finding. On
    /// Linux/x86 the identical commit drives it past 1.44e-3 and keeps
    /// going. See `the_deep_steps_stall_boundary_is_platform_dependent`.
    ///
    /// Deterministic *within* a platform: two consecutive runs agree on
    /// iteration count, residual to four significant figures, and the
    /// stalling iteration.
    fn tolerance_fixture() -> InsertionGeometry {
        let scan = icosphere(0.020, 2);
        let design = SimDesign {
            cavity_inset_m: 0.003,
            layers: vec![layer(0.008, "ECOFLEX_00_30")],
        };
        build_insertion_geometry(&scan, &design, &[], 2_000, 0.004)
            .expect("the tolerance fixture's geometry must build")
    }

    /// Interference (m) at which [`tolerance_fixture`] solves to
    /// [`TIGHT_TOL`] on every platform tried. The gates use this depth,
    /// so what they measure is the *tolerance*, never the conditioning
    /// edge.
    const TOL_FIXTURE_SHALLOW_M: f64 = 0.0025;

    /// Interference (m), 0.3 mm deeper, at which the scene becomes hard
    /// enough that the shipped tolerance's answer crosses
    /// [`ENGINEERING_TOL`].
    ///
    /// ⚠ Whether a *tighter* request is reachable at this depth is
    /// exactly the platform-dependent part — see [`tolerance_fixture`].
    /// Nothing here asserts that it is not.
    const TOL_FIXTURE_DEEP_M: f64 = 0.0028;

    /// A residual tolerance an engineering answer would be expected to
    /// reach: two decades tighter than the `1e-1` this pipeline ships.
    ///
    /// ⚠ Deliberately *not* compared against the sim-soft Tet10 + IPC
    /// fixture's 1.33e-12. These are free-DOF residual norms in newtons
    /// on different meshes, DOF counts and load scales, so their
    /// absolute values are not comparable — an earlier revision of this
    /// docstring made that comparison and got the decade count wrong by
    /// seven in the process.
    const ENGINEERING_TOL: f64 = 1e-3;

    /// A residual tolerance the solver demonstrably reaches on this
    /// scene at [`TOL_FIXTURE_SHALLOW_M`], on both macOS/ARM and
    /// Linux/x86 CI.
    const TIGHT_TOL: f64 = 1e-6;

    /// **Renovation item 4 step 0 — the insertion solve's reported
    /// convergence is bounded by its tolerance, not by its residual.**
    ///
    /// Two readings on one scene:
    ///
    /// 1. asked for [`TIGHT_TOL`] the solve reaches 2.286e-7; asked for
    ///    the shipped [`INSERTION_SOLVE_TOL`] at the same depth it
    ///    returns 5.339e-2 — **a 2.3e5x gap reported as the same
    ///    "converged"**;
    /// 2. 0.3 mm deeper, what the shipped tolerance accepts crosses
    ///    [`ENGINEERING_TOL`] outright.
    ///
    /// ⚠ **Reading 1 is generic and reading 2 is not, and the difference
    /// matters.** *Any* Newton solve asked for two tolerances leaves a
    /// gap; the ratio alone says nothing about this pipeline. What is
    /// specific is where the shipped constant sits — five decades above
    /// what this scene supports — and reading 2, which is a statement
    /// about this geometry at this depth.
    ///
    /// The gap is asserted as a **ratio, not a threshold**: where Newton
    /// lands on the first iterate under a bar is platform-dependent, the
    /// size of what it leaves on the table is not. An earlier revision
    /// asserted that the deep step *could not* reach [`ENGINEERING_TOL`]
    /// at all, which held on macOS/ARM and failed on Linux/x86 CI.
    ///
    /// ⛔ **This gate is not expected to move when the bridge lands, and
    /// an earlier revision claiming otherwise was wrong.** It pins a
    /// property of [`INSERTION_SOLVE_TOL`] against achievable precision,
    /// and the bridge does not change that constant — a *better*
    /// conditioned solver would if anything reach further below
    /// [`TIGHT_TOL`] and make the ratio **larger**. The quantity the
    /// bridge must actually improve is the deepest interference solvable
    /// at a tight tolerance, which is the platform-dependent one, so it
    /// is **reported by a diagnostic on a named platform, not gated**.
    #[test]
    fn the_insertion_solves_convergence_is_bounded_by_its_tolerance() {
        assert_eq!(
            tolerance_fixture().n_tets,
            9258,
            "the fixture's tet count is part of what makes these readings reproducible",
        );

        // Both solves return only when their residual is under the
        // tolerance they were given (`newton.rs`'s convergence test), so
        // the `expect`s below carry the claim that each is REACHABLE —
        // asserting `residual < tol` afterwards would restate the
        // solver's own postcondition and could not fail.
        let tight =
            run_single_insertion_step_at_tol(tolerance_fixture(), TOL_FIXTURE_SHALLOW_M, TIGHT_TOL)
                .expect("the fixture must solve to TIGHT_TOL at the shallow depth");
        let loose = run_single_insertion_step_at_tol(
            tolerance_fixture(),
            TOL_FIXTURE_SHALLOW_M,
            INSERTION_SOLVE_TOL,
        )
        .expect("the shipped tolerance reports this step converged");

        // Measured 2.3e5x on macOS/ARM (5.339e-2 against 2.286e-7). The
        // bar is two decades, so the reading has three decades of room
        // to move without the claim moving.
        let gap = loose.final_residual_norm / tight.final_residual_norm;
        assert!(
            gap > 100.0,
            "the shipped tolerance is only meaningfully looser if it leaves a gap: \
             {:.3e} against {:.3e} is {gap:.1e}x",
            loose.final_residual_norm,
            tight.final_residual_norm,
        );

        let deep = run_single_insertion_step_at_tol(
            tolerance_fixture(),
            TOL_FIXTURE_DEEP_M,
            INSERTION_SOLVE_TOL,
        )
        .expect("the shipped tolerance reports the deep step converged too");
        assert!(
            deep.final_residual_norm > ENGINEERING_TOL,
            "this gate is vacuous unless the shipped tolerance accepts a residual \
             above {ENGINEERING_TOL:.0e} somewhere on this scene; got {:.3e}",
            deep.final_residual_norm,
        );
    }

    /// The shipped entry point solves at the shipped tolerance.
    ///
    /// [`run_single_insertion_step`] now delegates to
    /// [`run_single_insertion_step_at_tol`], which introduces a seam:
    /// *which* tolerance the shipped path passes. Nothing else pins it —
    /// changing that argument to `1e-6` passed the entire 107-test suite
    /// before this gate existed, so a future edit could silently retune
    /// every consumer of this tool.
    ///
    /// Compares the two calls on identical inputs. Every field has to
    /// agree, including the converged positions: the solve is
    /// deterministic within a platform, so a differing tolerance shows up
    /// as a differing iteration count and residual.
    #[test]
    fn the_shipped_entry_point_solves_at_the_shipped_tolerance() {
        let shipped = run_single_insertion_step(tolerance_fixture(), TOL_FIXTURE_SHALLOW_M)
            .expect("the shipped entry point must solve the shallow step");
        let explicit = run_single_insertion_step_at_tol(
            tolerance_fixture(),
            TOL_FIXTURE_SHALLOW_M,
            INSERTION_SOLVE_TOL,
        )
        .expect("the explicit call at the shipped tolerance must solve it too");

        assert_eq!(
            shipped.iter_count, explicit.iter_count,
            "the shipped entry point took {} iterations where the shipped tolerance takes {}",
            shipped.iter_count, explicit.iter_count,
        );
        assert!(
            (shipped.final_residual_norm - explicit.final_residual_norm).abs() < f64::EPSILON,
            "residual {:.6e} from the shipped entry point against {:.6e} at the shipped tolerance",
            shipped.final_residual_norm,
            explicit.final_residual_norm,
        );
        assert_eq!(
            shipped.x_final, explicit.x_final,
            "the shipped entry point must return the same converged positions",
        );
    }

    /// **Where the deep step stops being solvable is platform-dependent,
    /// so nothing may gate on it.**
    ///
    /// Recorded because it cost a red CI run and would otherwise be
    /// rediscovered. At [`TOL_FIXTURE_DEEP_M`], asked for
    /// [`ENGINEERING_TOL`]:
    ///
    /// - macOS/ARM (local): Armijo stall at Newton iter 108, `r_norm`
    ///   2.78e-3 — the same failure mode and residual decade as the real
    ///   iter-1 scan's 4.13e-3;
    /// - Linux/x86 (CI): no stall — the residual passes 1.44e-3 by iter
    ///   39 and keeps falling.
    ///
    /// Same commit, same inputs. The `faer` LU fallback fires on both (a
    /// non-SPD tangent at a handful of recurring pivots), and which side
    /// of the Armijo edge that lands on is decided by arithmetic this
    /// test cannot pin. ⚠ **It also means the real scan's stall at
    /// `tol` = 1e-6 is a single-platform measurement** and should be read
    /// as one.
    ///
    /// ▶ This is also where the bridge's payoff gets measured: the
    /// deepest interference solvable at a tight tolerance is the quantity
    /// Tet10 + the IPC face barrier has to improve, and it is reported
    /// here on a named platform rather than gated.
    ///
    /// `#[ignore]` — a diagnostic, not a gate; it asserts nothing. Run:
    ///
    /// ```text
    /// cargo test -p cf-sim-research \
    ///     the_deep_steps_stall_boundary_is_platform_dependent -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "diagnostic — prints where this platform's stall boundary falls"]
    fn the_deep_steps_stall_boundary_is_platform_dependent() {
        for tol in [INSERTION_SOLVE_TOL, 1e-2, ENGINEERING_TOL, 1e-4, TIGHT_TOL] {
            let outcome =
                run_single_insertion_step_at_tol(tolerance_fixture(), TOL_FIXTURE_DEEP_M, tol);
            match outcome {
                Ok(step) => eprintln!(
                    "tol {tol:.0e}: converged in {} iters at {:.3e}",
                    step.iter_count, step.final_residual_norm,
                ),
                Err(e) => eprintln!("tol {tol:.0e}: {e}"),
            }
        }
    }

    /// **The ramp reaches whatever cavity inset it is asked for, across
    /// the whole product slider range** — which is what "the depth
    /// envelope is at ceiling" rests on.
    ///
    /// The three shipped ramps all run at a 3 mm inset, and reaching a
    /// requested 3 mm says nothing about 8 mm.
    /// `cf_device_types::CAVITY_INSET_SLIDER_MAX_M` is **8 mm**, so the
    /// claim needed the rest of the range. Measured 2026-09-22 on the
    /// synthetic icosphere, release:
    ///
    /// | inset | tets | steps | wall clock |
    /// |---|---|---|---|
    /// | 3 mm | 45 654 | 16/16 | 12 s |
    /// | 4 mm | 45 156 | 16/16 | 39 s |
    /// | 5 mm | 42 981 | 16/16 | 55 s |
    /// | 6 mm | 38 736 | 16/16 | 43 s |
    /// | 7 mm | 37 884 | 16/16 | 42 s |
    /// | 8 mm | 35 670 | 16/16 | 39 s |
    ///
    /// ⇒ no depth headroom for the bridge to win anywhere in the design
    /// range, and [`INSERTION_CONTACT_SMOOTHING_EPS_M`]'s recorded
    /// "stalls at cavity 6 mm" is pre-pre-smooth history.
    ///
    /// `#[ignore]` — six release-mode 16-step ramps, ~4 minutes. Run:
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release \
    ///     the_ramp_converges_across_the_whole_cavity_slider_range -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "diagnostic — six release-mode ramps, ~4 minutes"]
    fn the_ramp_converges_across_the_whole_cavity_slider_range() {
        for inset_m in [0.003_f64, 0.004, 0.005, 0.006, 0.007, 0.008] {
            let scan = icosphere(0.040, 3);
            let design = SimDesign {
                cavity_inset_m: inset_m,
                layers: vec![layer(0.010, "ECOFLEX_00_30")],
            };
            let Ok(geometry) = build_insertion_geometry(&scan, &design, &[], 2_000, 0.004) else {
                eprintln!("inset {:.1} mm: geometry failed to build", inset_m * 1e3);
                continue;
            };
            let n_tets = geometry.n_tets;
            match run_insertion_ramp(geometry, 16) {
                Ok(ramp) => {
                    let deepest = ramp.steps.last().map_or(0.0, |s| s.interference_m);
                    eprintln!(
                        "inset {:.1} mm: {n_tets} tets, {}/16 steps, deepest {:.2} mm{}",
                        inset_m * 1e3,
                        ramp.steps.len(),
                        deepest * 1e3,
                        ramp.failed_at_step.map_or_else(
                            || " — all converged".to_owned(),
                            |k| format!(
                                " — STALL at step {k}: {}",
                                ramp.failure_reason.as_deref().unwrap_or("<no reason>")
                            ),
                        ),
                    );
                }
                Err(e) => eprintln!("inset {:.1} mm: ramp failed: {e}", inset_m * 1e3),
            }
        }
    }

    /// The tolerance knob added for the gates above changes the tolerance
    /// and nothing else.
    ///
    /// [`insertion_solver_config`] delegates to
    /// [`insertion_solver_config_at_tol`], so every other field — `dt`,
    /// the Newton cap, the gated LM preset — has to survive the
    /// delegation. Debug-equality covers fields a future `SolverConfig`
    /// adds, which a hand-written field list would silently miss.
    #[test]
    fn the_tolerance_knob_changes_only_the_tolerance() {
        let shipped = insertion_solver_config();
        let delegated = insertion_solver_config_at_tol(INSERTION_SOLVE_TOL);
        assert_eq!(
            format!("{shipped:?}"),
            format!("{delegated:?}"),
            "the shipped config must be exactly the delegated one at the shipped tolerance",
        );

        let tightened = insertion_solver_config_at_tol(ENGINEERING_TOL);
        assert!(
            (tightened.tol - ENGINEERING_TOL).abs() < f64::EPSILON,
            "the knob must set the tolerance it was given",
        );
        // `SolverConfig` is `#[non_exhaustive]`, so restore by mutation
        // rather than struct-update syntax; it is `Copy`.
        let mut restored = tightened;
        restored.tol = shipped.tol;
        assert_eq!(
            format!("{restored:?}"),
            format!("{shipped:?}"),
            "restoring only `tol` must reproduce the shipped config",
        );
    }

    use sim_soft::{boundary_vertex_areas, face_barrier_kappa};

    // ─────────────────────────────────────────────────────────────────
    // THE BRIDGE — the two quantities a derived face-barrier κ needs,
    // measured for THIS scene rather than inherited.
    //
    // `sim_soft::contact::barrier` derives κ from two numbers:
    // `κ = σ / |b'(standoff)|`, where σ is the traction the compressed
    // wall pushes back with and the standoff is the gap the barrier
    // must hold open at the TIGHTEST point of the patch — one ramp
    // increment times a patch-non-uniformity factor ρ. Both were
    // measured on `tet10_yeoh_ipc_convergence`'s fixture (σ ≈ 30.4 kPa,
    // ρ = 1.30) and NEITHER transfers here: that fixture is a perfect
    // sphere, whose gap is uniform by symmetry, marching a 0.1 mm
    // increment through a 1.2 mm band. This scene is a scan-derived
    // cavity closing on a 0.1875 mm increment.
    //
    // ⚠ These probes read a PENALTY solve. What transfers to the IPC
    // path is the traction and the gap DISTRIBUTION — both set by the
    // elastic wall and the geometry — not the barrier arithmetic, which
    // is a different function on each path (see `barrier`'s module docs
    // on the three non-comparable κ).
    // ─────────────────────────────────────────────────────────────────

    /// Area-weighted summary of one converged contact patch — the
    /// reduction the bridge's κ derivation reads.
    ///
    /// Every field is computed over the pairs with a **well-defined
    /// pressure** (`tributary_area > 0`); [`n_dropped`](Self::n_dropped)
    /// counts the rest rather than letting them vanish, because a patch
    /// that is mostly zero-area pairs is a degenerate readout and not a
    /// low-traction one.
    #[derive(Debug, Clone, Copy)]
    struct PatchStats {
        /// Pairs contributing to every other field.
        n_pairs: usize,
        /// Pairs excluded for a non-positive tributary area (the
        /// `NaN`-pressure sentinel case). Reported, never assumed zero.
        n_dropped: usize,
        /// Σ tributary area over the contributing pairs (m²) — the
        /// deformed patch area the traction is spread over.
        area_m2: f64,
        /// `|Σ f|` (N) — the vector sum, which is what the shipped F-d
        /// curve plots. On an enveloping patch the radial components
        /// cancel and this reads far under the load actually carried.
        force_vector_sum_n: f64,
        /// `Σ |f|` (N) — the magnitude sum, which does not cancel.
        force_magnitude_sum_n: f64,
        /// Area-weighted mean traction on the **deformed** patch,
        /// `Σ|f| / Σa_def` (Pa).
        ///
        /// Identically the area-weighted mean of the per-pair
        /// `pressure`, since `pressure = |f| / a`. That is one reading,
        /// not two: on the penalty path the reported force *is* the
        /// barrier evaluation, so the cavity cross-check
        /// `the_force_free_traction_agrees_with_an_independent_reading_on_the_cavity`
        /// performs on the IPC fixture has no independent second source
        /// here.
        ///
        /// ⛔ **This is NOT the σ the κ derivation consumes** — see
        /// [`traction_rest_pa`](Self::traction_rest_pa).
        traction_pa: f64,
        /// Σ **rest** tributary area over the contributing pairs (m²).
        area_rest_m2: f64,
        /// Area-weighted mean traction on the **rest** patch,
        /// `Σ|f| / Σa_rest` (Pa) — **this is σ for the derivation**.
        ///
        /// ⛔⛔ The two bases are not interchangeable and sim-soft says
        /// so at the source: `ContactPairReadout::tributary_area` is the
        /// **deformed** tributary, and its own docs note that *"the
        /// surface-integrated barrier is weighted by the face's rest
        /// area, so the barrier weight and this pressure tributary are
        /// deliberately different measures"*. The face barrier's energy
        /// is `A_rest · Σ ŵ κ b`, so `face_barrier_kappa` inverts a
        /// **rest**-normalised traction, and
        /// `tet10_yeoh_ipc_convergence`'s own independent reading
        /// accumulates `flat_area += *rest_area`.
        ///
        /// Measured, the two differ by **22–23 %** at full depth
        /// (117.01 against 94.98 kPa on the sphere), the patch having
        /// stretched — a bias in a number that sets a shipped constant,
        /// not a rounding detail.
        ///
        /// ⚠ An earlier revision of this line said "~12–13 %" and
        /// called it measured. It was an ESTIMATE, extrapolated from
        /// the patch area growing across the ramp, written before the
        /// rest area was read — and it was low by nearly half.
        traction_rest_pa: f64,
        /// Contributing pairs whose rest area could not be attributed —
        /// a pair that is not a single vertex (a P2 face pair), or whose
        /// vertex is outside the rest-area table. Reported rather than
        /// folded into zero, because a patch that is mostly
        /// unattributed makes `traction_rest_pa` meaningless.
        n_unattributed: usize,
        /// Tightest signed distance on the patch (m). **Negative means
        /// the penalty solve is interpenetrating**, and then ρ below is
        /// not a gap ratio at all.
        min_sd_m: f64,
        /// Area-weighted mean signed distance (m).
        mean_sd_m: f64,
        /// Loosest signed distance among active pairs (m) — bounded
        /// above by `d̂` because the producer only emits active pairs.
        max_sd_m: f64,
        /// The gap below which [`PATCH_TAIL_FRACTION`] of the patch
        /// AREA lies (m) — the outlier-resistant stand-in for
        /// [`min_sd_m`](Self::min_sd_m).
        ///
        /// ⛔ `min_sd` is one vertex. A ρ built on it is a ratio whose
        /// denominator can be set by a single bad tet on a scan-derived
        /// patch, and the κ floor divides by `|b'|` at `ρ · step` — so
        /// an outlier there moves a shipped constant. This is the same
        /// quantity read off the tail of the area distribution instead.
        p05_sd_m: f64,
    }

    /// Area fraction defining [`PatchStats::p05_sd_m`].
    ///
    /// 5 % is a stated choice, not a derived one: small enough that it
    /// still reads the tight tail rather than the bulk, large enough
    /// that it takes more than a handful of pairs to set it. The probes
    /// print `min` and this side by side precisely so the choice is
    /// visible rather than load-bearing in silence.
    const PATCH_TAIL_FRACTION: f64 = 0.05;

    impl PatchStats {
        /// `Σ|f| / |Σf|` — how much of the carried load the vector sum
        /// cancels away. `1.0` on a patch pushing one direction; large
        /// on an enveloping one.
        fn cancellation(self) -> f64 {
            self.force_magnitude_sum_n / self.force_vector_sum_n
        }

        /// `mean_sd / min_sd` — **ρ**, the patch non-uniformity, as a
        /// ratio of measured gaps.
        ///
        /// ⚠ Not identical to the fixture's ρ, which is
        /// `face_barrier_standoff(κ, d̂, σ) / min_sd`: an *inverted
        /// barrier* standing in for the representative gap. The two
        /// agree only where the traction–gap map is near-linear across
        /// the patch's gap spread. This form is the model-free one and
        /// is the one that transfers off the penalty path.
        fn rho_gap(self) -> f64 {
            self.mean_sd_m / self.min_sd_m
        }

        /// [`rho_gap`](Self::rho_gap) against the area tail instead of
        /// the single tightest pair.
        ///
        /// The two agree when the patch's tight end is a *region*; they
        /// diverge when it is an *outlier*, and the size of the
        /// divergence is the whole reason both are reported.
        fn rho_tail(self) -> f64 {
            self.mean_sd_m / self.p05_sd_m
        }
    }

    /// One hand-built [`ContactPairReadout`], for the fixtures that
    /// pin [`patch_stats`]'s arithmetic against closed-form answers.
    ///
    /// Mirrors sim-soft's own `pressure` convention exactly, including
    /// the `NaN` sentinel at zero area — a fixture that wrote `0.0`
    /// there would be testing against a patch the solver never
    /// produces.
    fn pair_readout(vertex_id: VertexId, sd: f64, area: f64, f: Vec3) -> ContactPairReadout {
        ContactPairReadout {
            pair: ContactPair::Vertex {
                vertex_id,
                primitive_id: 0,
            },
            position: Vec3::zeros(),
            sd,
            normal: if f.norm() > 0.0 { f.normalize() } else { f },
            force_on_soft: f,
            tributary_area: area,
            pressure: if area > 0.0 {
                f.norm() / area
            } else {
                f64::NAN
            },
        }
    }

    /// Reduce a step's orphan-filtered active-pair readouts to
    /// [`PatchStats`]. `None` when no pair has a well-defined pressure
    /// — there is no patch to summarize, which is a scene defect rather
    /// than a zero-traction reading.
    fn patch_stats(readouts: &[ContactPairReadout], rest_areas: &[f64]) -> Option<PatchStats> {
        let mut n_pairs = 0_usize;
        let mut n_dropped = 0_usize;
        let mut area_m2 = 0.0_f64;
        let mut force_magnitude_sum_n = 0.0_f64;
        let mut force_sum = Vec3::zeros();
        let mut area_sd = 0.0_f64;
        let mut min_sd_m = f64::INFINITY;
        let mut max_sd_m = f64::NEG_INFINITY;
        let mut by_gap: Vec<(f64, f64)> = Vec::with_capacity(readouts.len());
        let mut area_rest_m2 = 0.0_f64;
        let mut n_unattributed = 0_usize;
        for r in readouts {
            if r.tributary_area <= 0.0 || !r.tributary_area.is_finite() || !r.pressure.is_finite() {
                n_dropped += 1;
                continue;
            }
            n_pairs += 1;
            area_m2 += r.tributary_area;
            force_magnitude_sum_n += r.force_on_soft.norm();
            force_sum += r.force_on_soft;
            area_sd += r.tributary_area * r.sd;
            min_sd_m = min_sd_m.min(r.sd);
            max_sd_m = max_sd_m.max(r.sd);
            by_gap.push((r.sd, r.tributary_area));
            // `ContactPair` is `#[non_exhaustive]`; a P2 face pair has
            // six nodes and no single rest tributary, so it is counted
            // rather than silently contributing zero.
            match r.pair {
                ContactPair::Vertex { vertex_id, .. } => match rest_areas.get(vertex_id as usize) {
                    Some(&a) if a > 0.0 => area_rest_m2 += a,
                    _ => n_unattributed += 1,
                },
                _ => n_unattributed += 1,
            }
        }
        if n_pairs == 0 || area_m2 <= 0.0 {
            return None;
        }
        // Area-weighted tail: walk the patch from its tightest pair
        // outward and stop where `PATCH_TAIL_FRACTION` of the area is
        // behind. `total_cmp` orders the gaps without a `NaN` branch —
        // a `NaN` gap cannot reach here, since its pressure would have
        // been non-finite and the pair dropped above.
        by_gap.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        let cut = PATCH_TAIL_FRACTION * area_m2;
        let mut cumulative = 0.0_f64;
        let mut p05_sd_m = min_sd_m;
        for (sd, area) in by_gap {
            cumulative += area;
            p05_sd_m = sd;
            if cumulative >= cut {
                break;
            }
        }
        Some(PatchStats {
            n_pairs,
            n_dropped,
            area_m2,
            force_vector_sum_n: force_sum.norm(),
            force_magnitude_sum_n,
            traction_pa: force_magnitude_sum_n / area_m2,
            area_rest_m2,
            traction_rest_pa: if area_rest_m2 > 0.0 {
                force_magnitude_sum_n / area_rest_m2
            } else {
                f64::NAN
            },
            n_unattributed,
            min_sd_m,
            mean_sd_m: area_sd / area_m2,
            max_sd_m,
            p05_sd_m,
        })
    }

    /// What `tet10_yeoh_ipc_convergence`'s own probe measures for its
    /// σ-vs-κ coupling — **measured by running it, not recalled**.
    ///
    /// ```text
    /// cargo test -p sim-soft --release --test tet10_yeoh_ipc_convergence \
    ///     is_the_contact_traction_a_property_of_the_scene_or_of_kappa \
    ///     -- --ignored --nocapture
    /// ```
    ///
    /// 2026-09-22, macOS/ARM, 123 s: *"at one compression, kappa spans
    /// 100x and sigma spans 1.532x (14.255 -> 21.839 kPa)"*, over
    /// `κ ∈ {1e6, 1e7, 1e8}` at a common plane height, all three arms
    /// non-penetrating (`min_sd` 0.098 / 0.493 / 0.895 mm).
    ///
    /// ⛔ **An earlier revision of this module quoted `1.003×` here.
    /// That number is in no source file and was never measured** — it
    /// was a plausible value that hardened into a citation, and it made
    /// this scene's coupling look an order of magnitude worse than the
    /// fixture's when the comparison is actually the other way round.
    /// A figure attributed to another test is a claim about that test:
    /// run it.
    const FIXTURE_SIGMA_SPAN: f64 = 1.532;

    /// Decades of `κ` the [`FIXTURE_SIGMA_SPAN`] was measured over.
    /// Quoted so this scene's span can be put on the same footing —
    /// comparing a 3-decade span against a 2-decade one is the mistake
    /// that produced the inverted conclusion above.
    const FIXTURE_SIGMA_SPAN_DECADES: f64 = 2.0;

    /// Candidate contact bands (m) the derivation is reported across.
    ///
    /// `d̂` is a free choice on the bridge, and the floor
    /// `σ / |b'(ρ·step)|` is undefined once `ρ·step ≥ d̂` — the barrier
    /// cannot hold open a gap wider than the band it acts over. Showing
    /// the derivation at several bands is what makes that boundary
    /// visible instead of a single number that happens to exist. The
    /// shipped penalty band is 1.0 mm ([`INSERTION_CONTACT_DHAT`]); the
    /// IPC fixture's is 1.2 mm.
    const BRIDGE_DHAT_CANDIDATES_M: [f64; 4] = [0.5e-3, 1.0e-3, 1.2e-3, 2.0e-3];

    /// Print the patch summary for one converged step.
    fn print_patch_row(label: &str, interference_m: f64, stats: PatchStats) {
        eprintln!(
            "  {label:<9} {:>7.3} {:>6} {:>5} {:>10.2} {:>9.3} {:>9.3} {:>7.2} {:>9.2} \
             {:>9.4} {:>9.4} {:>7.3} {:>9.4} {:>8.3} {:>10}",
            interference_m * 1e3,
            stats.n_pairs,
            stats.n_dropped,
            stats.area_m2 * 1e6,
            stats.force_magnitude_sum_n,
            stats.force_vector_sum_n,
            stats.cancellation(),
            stats.traction_pa * 1e-3,
            stats.min_sd_m * 1e3,
            stats.mean_sd_m * 1e3,
            stats.rho_gap(),
            stats.p05_sd_m * 1e3,
            stats.rho_tail(),
            if stats.min_sd_m > 0.0 {
                ""
            } else {
                "⛔ THROUGH"
            },
        );
    }

    /// The column header matching [`print_patch_row`].
    fn print_patch_header() {
        eprintln!(
            "  {:<9} {:>7} {:>6} {:>5} {:>10} {:>9} {:>9} {:>7} {:>9} {:>9} {:>9} {:>7} \
             {:>9} {:>8} {:>10}",
            "",
            "d/mm",
            "pairs",
            "drop",
            "A/mm2",
            "sum|f|/N",
            "|sum f|/N",
            "cancel",
            "sigma/kPa",
            "min_sd/mm",
            "mean_sd",
            "rho",
            "p05_sd/mm",
            "rho_tail",
            "seated",
        );
    }

    /// Run a straight-in insertion ramp and report the contact patch at
    /// every converged step. Returns the deepest converged step's stats.
    ///
    /// Rebuilds the contact primitive at each step's interference to
    /// read the patch back out of `x_final` — the same double-build
    /// `run_insertion_ramp` itself uses, for the same reason
    /// (`PenaltyRigidContact` is not `Clone`).
    ///
    /// ⛔ **NO always-on gate covers this function**, and one line of it
    /// is load-bearing for every number the probes report: the readout
    /// contact is rebuilt at `contact_kappa`, *not* at
    /// [`INSERTION_CONTACT_KAPPA`]. If that regressed to the shipped
    /// const, every stiffness arm would report its patch at 1e3 while
    /// claiming its own κ, the sweep would flatten, and nothing would
    /// fail — the gates that do exist
    /// ([`the_contact_stiffness_knob_reaches_both_the_solve_and_the_readout`])
    /// exercise `intruder_contact_at_kappa` directly and never come
    /// through here. Stated rather than fixed because a gate over this
    /// function means running a ramp per stiffness, which is what makes
    /// the probes `#[ignore]`d in the first place.
    fn patch_over_ramp(
        label: &str,
        geometry: InsertionGeometry,
        n_steps: usize,
        contact_kappa: f64,
        verbose: bool,
    ) -> Vec<(f64, PatchStats)> {
        let intruder = geometry.intruder.clone();
        let bounds = geometry.bounds;
        let cavity_offset_m = geometry.cavity_offset_m;
        let mesh = geometry.mesh.clone();
        let referenced: Vec<VertexId> = referenced_vertices(&geometry.mesh);
        let n_tets = geometry.n_tets;
        // The REST tributary per vertex, read before the ramp deforms
        // anything. `boundary_vertex_areas` is the same barycentric
        // lumping sim-soft uses for the deformed tributary, evaluated on
        // rest positions instead — so the two bases differ only by the
        // deformation, which is exactly the quantity in question.
        let rest_areas = boundary_vertex_areas(
            Mesh::<Yeoh>::positions(&geometry.mesh),
            Mesh::<Yeoh>::boundary_faces(&geometry.mesh),
        );

        let t0 = Instant::now();
        let ramp = match run_insertion_ramp_at_kappa(geometry, n_steps, contact_kappa) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("  {label}: ramp FAILED to start: {e}");
                return Vec::new();
            }
        };
        eprintln!(
            "  {label}: {} tets, {}/{} steps converged in {:.1}s",
            n_tets,
            ramp.steps.len(),
            n_steps,
            t0.elapsed().as_secs_f64(),
        );
        if let Some(k) = ramp.failed_at_step {
            eprintln!(
                "  {label}: STALL at step {k}: {}",
                ramp.failure_reason.as_deref().unwrap_or("<no reason>"),
            );
        }

        if verbose {
            print_patch_header();
        }
        let mut out = Vec::with_capacity(ramp.steps.len());
        for step in &ramp.steps {
            let positions: Vec<Vec3> = positions_from_flat(&step.x_final);
            // Read the patch back at the SAME stiffness it was solved
            // at — a readout contact built from the shipped const would
            // report a patch nobody solved.
            let readout_contact = intruder_contact_at_kappa(
                &intruder,
                bounds,
                step.interference_m,
                cavity_offset_m,
                contact_kappa,
            );
            let raw = readout_contact.per_pair_readout(&mesh, &positions);
            let filtered = filter_pair_readouts_to_referenced(raw, &referenced);
            match patch_stats(&filtered, &rest_areas) {
                Some(stats) => {
                    if verbose {
                        print_patch_row(label, step.interference_m, stats);
                    }
                    out.push((step.interference_m, stats));
                }
                None => eprintln!(
                    "  {label:<9} {:>7.3}  no pair with a well-defined pressure \
                     ({} raw active pairs)",
                    step.interference_m * 1e3,
                    filtered.len(),
                ),
            }
        }
        // ⭐ The RANGE, printed — because it is the range that gets
        // quoted, and a reader who has to fold the column themselves
        // will quote the endpoints instead. Neither series is monotone
        // in depth on either fixture, so endpoints are not the range.
        let cancel_range = out
            .iter()
            .map(|(_, s)| s.cancellation())
            .fold(None, |acc: Option<(f64, f64)>, c| {
                Some(acc.map_or((c, c), |(l, h)| (l.min(c), h.max(c))))
            });
        if let (true, Some((c_lo, c_hi))) = (verbose, cancel_range) {
            eprintln!(
                "  {label}: sum|f|/|sum f| over the converged steps spans \
                 {c_lo:.1}..{c_hi:.1}x (min..max, NOT first..last — the series is \
                 not monotone in depth), first {:.1}x, last {:.1}x",
                out.first().map_or(f64::NAN, |(_, s)| s.cancellation()),
                out.last().map_or(f64::NAN, |(_, s)| s.cancellation()),
            );
        }
        out
    }

    /// Report the derived face-barrier bounds this scene's σ and ρ
    /// imply, across the candidate bands.
    ///
    /// `floor = σ / |b'(ρ · step)|` — hold one ramp increment open at
    /// the tightest point of the patch, or the next increment starts
    /// infeasible. `ceiling = σ / |b'(d̂/2)|` — a stated requirement,
    /// that the standoff bias stay in the lower half of the band.
    fn report_derived_face_kappa(label: &str, stats: PatchStats, ramp_step_m: f64) {
        eprintln!(
            "  {label} derivation — sigma(rest) = {:.2} kPa [sigma(deformed) {:.2} kPa, \
             {} unattributed], ramp step = {:.4} mm",
            stats.traction_rest_pa * 1e-3,
            stats.traction_pa * 1e-3,
            stats.n_unattributed,
            ramp_step_m * 1e3,
        );
        // ⚠ The floor below substitutes a ρ this module DEFINES
        // DIFFERENTLY from the one the formula was derived with. Said
        // here, at the point of substitution, and not only in
        // `rho_gap`'s docstring — a caveat that lives away from the
        // arithmetic it qualifies is a caveat nobody reads.
        eprintln!(
            "  ⚠ the floor's ρ was derived as `face_barrier_standoff(κ, d̂, σ)/min_sd`, a \
             BARRIER-INVERTED ratio. Both ρ below are GAP ratios (mean/min, mean/tail), \
             which coincide with it only where the traction-gap map is near-linear \
             across the patch's spread. They are the forms that survive leaving the \
             penalty path; the substitution is not an identity.",
        );
        if !stats.min_sd_m.is_finite() || stats.min_sd_m <= 0.0 {
            eprintln!(
                "  ⛔ min_sd = {:.4} mm is not positive — the penalty solve is \
                 interpenetrating here, so rho is not a gap ratio and everything below \
                 is arithmetic on a meaningless number.",
                stats.min_sd_m * 1e3,
            );
        }
        // BOTH definitions, because they are not close on a patch with
        // a tight outlier and the floor is derived from whichever one
        // is used. Reporting one would be choosing silently.
        for (rho_name, rho) in [
            ("rho(min) ", stats.rho_gap()),
            ("rho(tail)", stats.rho_tail()),
        ] {
            let standoff = ramp_step_m * rho;
            eprintln!(
                "  {label} at {rho_name} = {rho:.3} ⇒ required standoff {:.4} mm",
                standoff * 1e3,
            );
            eprintln!(
                "  {:>9} {:>14} {:>14} {:>10} {:>14}",
                "d_hat/mm", "floor", "ceiling", "decades", "1e7 inside?",
            );
            for d_hat in BRIDGE_DHAT_CANDIDATES_M {
                let floor = face_barrier_kappa(d_hat, standoff, stats.traction_rest_pa);
                let ceiling = face_barrier_kappa(d_hat, 0.5 * d_hat, stats.traction_rest_pa);
                match (floor, ceiling) {
                    (Some(f), Some(c)) => {
                        let decades = if f > 0.0 && c > f {
                            format!("{:.2}", (c / f).log10())
                        } else {
                            "EMPTY".to_owned()
                        };
                        let holds_1e7 = if (f..=c).contains(&1.0e7) {
                            "yes"
                        } else {
                            "no"
                        };
                        eprintln!(
                            "  {:>9.3} {:>14.4e} {:>14.4e} {:>10} {:>14}",
                            d_hat * 1e3,
                            f,
                            c,
                            decades,
                            holds_1e7,
                        );
                    }
                    _ => eprintln!(
                        "  {:>9.3} {:>14} {:>14} {:>10} {:>14}",
                        d_hat * 1e3,
                        if floor.is_none() { "undefined" } else { "-" },
                        if ceiling.is_none() { "undefined" } else { "-" },
                        "-",
                        "-",
                    ),
                }
            }
        }
    }

    /// The patch summary is area-weighted, and a pair with no surface
    /// patch contributes to NOTHING but the drop count.
    ///
    /// [`PatchStats`] is the reduction every number the bridge's κ
    /// derivation rests on comes out of, and it runs only inside
    /// `#[ignore]`d probes — so without this gate its arithmetic is
    /// exercised in no CI job at all. The fixture is hand-built so
    /// every field has a closed-form answer: three contributing pairs
    /// with areas 2 : 6 : 2 µm² carrying 4 : 6 : 5 N, plus one
    /// zero-area pair carrying 100 N that must not be seen.
    ///
    /// The zero-area pair is the load-bearing half. `pressure` is
    /// `NaN` there by sim-soft's sentinel convention, and a summary
    /// that summed its force anyway would report a traction 7.7× too
    /// high while every other field still looked plausible.
    #[test]
    fn the_patch_summary_is_area_weighted_and_drops_undefined_pressures() {
        let readouts = vec![
            pair_readout(0, 0.2e-3, 2.0e-6, Vec3::new(0.0, 0.0, -4.0)),
            pair_readout(1, 0.6e-3, 6.0e-6, Vec3::new(0.0, 0.0, -6.0)),
            // Not parallel to the others, so the vector sum genuinely
            // cancels and `cancellation` is not trivially 1.
            pair_readout(2, 0.4e-3, 2.0e-6, Vec3::new(3.0, 0.0, 4.0)),
            pair_readout(3, 0.01e-3, 0.0, Vec3::new(0.0, 0.0, -100.0)),
        ];

        // Rest tributaries deliberately SMALLER than the deformed ones
        // (1.5 + 5.0 + 1.5 = 8 µm² against 10 µm²), because a stretched
        // patch is the case that makes the two bases disagree — and the
        // fourth entry belongs to the dropped pair, so it must not count.
        let rest_areas = [1.5e-6, 5.0e-6, 1.5e-6, 9.9e-6];
        let stats =
            patch_stats(&readouts, &rest_areas).expect("the fixture patch has well-defined pairs");
        assert_eq!(stats.n_pairs, 3, "the zero-area pair must not contribute");
        assert_eq!(
            stats.n_dropped, 1,
            "the zero-area pair must be counted, not silently skipped"
        );

        let close = |got: f64, want: f64, what: &str| {
            assert!(
                (got - want).abs() <= 1e-9 * want.abs().max(1.0),
                "{what}: got {got:e}, want {want:e}",
            );
        };
        close(
            stats.area_m2,
            10.0e-6,
            "area is the sum of the three tributaries",
        );
        close(stats.force_magnitude_sum_n, 15.0, "sum of |f| is 4 + 6 + 5");
        // Σf = (3, 0, -6) ⇒ |Σf| = 3√5, so the cancellation is exactly √5.
        close(
            stats.force_vector_sum_n,
            45.0_f64.sqrt(),
            "|Σf| over the three pairs",
        );
        close(
            stats.cancellation(),
            5.0_f64.sqrt(),
            "cancellation is Σ|f| / |Σf|",
        );
        close(stats.traction_pa, 1.5e6, "σ is Σ|f| / Σa = 15 N / 10 mm²");
        close(
            stats.min_sd_m,
            0.2e-3,
            "the tightest gap is the first pair's",
        );
        close(
            stats.max_sd_m,
            0.6e-3,
            "the loosest gap is the second pair's",
        );
        // (2·0.2 + 6·0.6 + 2·0.4) / 10 = 0.48 mm — NOT the unweighted
        // mean of 0.4 mm, which is what makes this an area weighting.
        close(stats.mean_sd_m, 0.48e-3, "the mean gap is area-weighted");
        close(stats.rho_gap(), 2.4, "ρ is mean_sd / min_sd");
        // The tightest pair carries 2 of 10 mm² — 20 % of the area, so
        // the 5 % tail lands inside it and the two ρ agree here. The
        // fixture below is the one where they do not.
        close(
            stats.p05_sd_m,
            0.2e-3,
            "the 5 % tail of this patch is its tightest pair",
        );
        close(
            stats.rho_tail(),
            2.4,
            "ρ_tail matches ρ when the tight end is not an outlier",
        );
        // The rest basis: 15 N over 8 µm², NOT over 10 µm².
        close(
            stats.area_rest_m2,
            8.0e-6,
            "the dropped pair's rest area must not count",
        );
        close(
            stats.traction_rest_pa,
            1.875e6,
            "σ_rest is Σ|f| / Σa_rest = 15 N / 8 mm²",
        );
        assert_eq!(
            stats.n_unattributed, 0,
            "every contributing pair has a rest area here"
        );
        assert!(
            stats.traction_rest_pa > stats.traction_pa,
            "a patch that stretched carries MORE traction per unit REST area, so the \
             rest basis must read higher — {:e} vs {:e}",
            stats.traction_rest_pa,
            stats.traction_pa,
        );

        // No rest table at all: the rest basis is undefined, and says so
        // rather than reporting a plausible zero-area traction.
        let unattributed = patch_stats(&readouts, &[]).expect("the deformed basis still resolves");
        assert_eq!(
            unattributed.n_unattributed, 3,
            "every contributing pair must be counted as unattributed",
        );
        assert!(
            unattributed.traction_rest_pa.is_nan(),
            "σ_rest with no rest area is undefined, not zero",
        );

        // A patch of nothing but undefined pressures is a scene defect,
        // and must not summarize as a zero-traction patch.
        let degenerate = vec![pair_readout(3, 0.01e-3, 0.0, Vec3::new(0.0, 0.0, -100.0))];
        assert!(
            patch_stats(&degenerate, &rest_areas).is_none(),
            "a patch whose every pair has no surface area has no traction to report",
        );
    }

    /// **σ and ρ on the synthetic sphere** — the comparison arm, and
    /// the one whose ρ is known in advance to be about discretisation
    /// rather than shape.
    ///
    /// A sphere's gap is uniform BY SYMMETRY, so whatever ρ this
    /// reports is what the BCC lattice and the SDF grid contribute. The
    /// scan probe's ρ minus this one is the part attributable to shape
    /// irregularity — which is the quantity
    /// `tet10_yeoh_ipc_convergence`'s `PATCH_NONUNIFORMITY = 1.30`
    /// could not measure, since its fixture is a sphere too.
    ///
    /// Asserts nothing. The numbers are platform-dependent in the same
    /// way the stall boundary is, and a probe that reports is worth
    /// more than a gate that pins a machine.
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release --bin cf-sim-research \
    ///     the_bridges_design_traction_and_patch_nonuniformity_on_the_synthetic_sphere \
    ///     -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "release-mode 16-step ramp — measurement, asserts nothing; run with --ignored --nocapture"]
    fn the_bridges_design_traction_and_patch_nonuniformity_on_the_synthetic_sphere() {
        let cavity_inset_m = 0.003;
        let n_steps = 16_usize;
        let scan = icosphere(0.040, 3);
        let design = SimDesign {
            cavity_inset_m,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        let geometry = build_insertion_geometry(&scan, &design, &[], 2_000, 0.004)
            .expect("synthetic-sphere geometry should build");
        eprintln!(
            "SYNTHETIC SPHERE — icosphere(40 mm, 3), single ECOFLEX_00_30 10 mm wall, \
             cavity {:.1} mm, cell 4 mm, {n_steps} steps",
            cavity_inset_m * 1e3,
        );
        let steps = patch_over_ramp("sphere", geometry, n_steps, INSERTION_CONTACT_KAPPA, true);
        let Some(&(_, stats)) = steps.last() else {
            eprintln!("  no converged step produced a well-defined patch — nothing to report");
            return;
        };
        // 16 steps over the inset is the marching schedule this scene
        // actually runs, and the floor is a statement about ONE of its
        // increments.
        #[allow(clippy::cast_precision_loss)]
        let ramp_step_m = cavity_inset_m / n_steps as f64;
        report_derived_face_kappa("sphere", stats, ramp_step_m);
        report_required_marching_increment("sphere", stats, cavity_inset_m, INSERTION_CONTACT_DHAT);
    }

    /// **σ and ρ on the real scan** — the two numbers the bridge cannot
    /// derive a face-barrier κ without, on the geometry it will
    /// actually run.
    ///
    /// ⭐ **Two scenes, because the recon's depth table is about one of
    /// them and the product is the other.** Both are
    /// `sock_over_capsule.cleaned.stl` at cavity 3 mm, cell 4 mm, on
    /// the straight-in [`run_insertion_ramp`] the bridge targets:
    ///
    /// - `1layer` — a single ECOFLEX_00_30 10 mm layer, **no cap
    ///   planes**. This is [`run_insertion_ramp_on_iter1_scan`]'s
    ///   scene, and the one
    ///   `docs/INSERTION_SIM_TET10_RENOVATION_RECON.md` §8's
    ///   `16/16 @ 3.00 mm` row was read off (68 087 tets). No cap
    ///   planes routes `pinned_floor_shell` through the closed-cavity
    ///   short-circuit.
    /// - `gui-dflt` — ECOFLEX_00_30 + 50 % Slacker 10 mm INNER and
    ///   DRAGON_SKIN_20A 3 mm OUTER with the `prep.toml`'s cap plane,
    ///   matching [`h4_sweep_sliding_ramp_on_iter1_scan`]'s substrate
    ///   and the GUI: an open mouth with a pinned floor (72 935 tets).
    ///
    /// They do not reach the same depth, so a σ quoted without its
    /// scene is a σ quoted at an unstated depth.
    ///
    /// ⛔ **Fails rather than skips when the fixture is missing.** The
    /// other scan probes in this module return early with a `skip:`
    /// line, which makes an `--ignored` sweep report success for a run
    /// that measured nothing. This one is run deliberately, so not
    /// finding the scan is a failure of the run, not a portability
    /// accommodation. Point it elsewhere with
    /// `CF_SIM_RESEARCH_SPIKE_SCAN`.
    ///
    /// Asserts nothing beyond the fixture being present — same
    /// platform-dependence caveat as the sphere arm.
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release --bin cf-sim-research \
    ///     the_bridges_design_traction_and_patch_nonuniformity_on_the_real_scan \
    ///     -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "needs the repo-excluded iter-1 scan + a release ramp; run with --ignored --nocapture"]
    fn the_bridges_design_traction_and_patch_nonuniformity_on_the_real_scan() {
        let scan_path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        assert!(
            scan_path.exists(),
            "the iter-1 scan fixture is not at {} — this probe measures the REAL \
             geometry and has nothing to report without it; set \
             CF_SIM_RESEARCH_SPIKE_SCAN to point it elsewhere",
            scan_path.display(),
        );
        let prep_path = scan_path.with_extension("").with_extension("prep.toml");
        let prep_text = std::fs::read_to_string(&prep_path)
            .map_err(|e| format!("{}: {e}", prep_path.display()))
            .expect("the prep.toml beside the scan must load — it carries the cap planes");
        let cap_planes =
            cf_cap_planes::parse_cap_planes(&prep_text).expect("parse cap planes from prep.toml");
        assert!(
            !cap_planes.is_empty(),
            "iter-1's prep.toml carries one cap-plane loop; an empty list would route \
             `pinned_floor_shell` through the CLOSED-cavity short-circuit and measure a \
             structurally different problem than the GUI's open-mouth topology",
        );
        let scan = load_stl(&scan_path).expect("load the iter-1 cleaned scan");

        let cavity_inset_m = 0.003;
        let n_steps = 16_usize;
        eprintln!(
            "REAL SCAN — {} ({} faces), cavity {:.1} mm, cell 4 mm, {n_steps} steps",
            scan_path.display(),
            scan.faces.len(),
            cavity_inset_m * 1e3,
        );

        // TWO scenes, because the recon's depth table is about the
        // first and the product is the second, and they are not the
        // same problem. `run_insertion_ramp_on_iter1_scan` — the test
        // §8's `16/16 @ 3.00 mm` row was read off — is a single
        // Ecoflex layer with NO cap planes, which routes
        // `pinned_floor_shell` through the closed-cavity short-circuit.
        // The GUI default is the dual-layer stack with the prep.toml's
        // cap plane, i.e. an open mouth with a pinned floor.
        let scenes: [(&str, Vec<SimLayer>, bool); 2] = [
            ("1layer", vec![layer(0.010, "ECOFLEX_00_30")], false),
            (
                "gui-dflt",
                // Innermost-first, per `SimDesign.layers`.
                vec![
                    layer_with_slacker(0.010, "ECOFLEX_00_30", 0.5),
                    layer(0.003, "DRAGON_SKIN_20A"),
                ],
                true,
            ),
        ];
        for (label, layers, with_caps) in scenes {
            let caps: &[CapPlane] = if with_caps { &cap_planes } else { &[] };
            eprintln!(
                "\n--- {label}: {} layer(s), {} cap plane(s) ---",
                layers.len(),
                caps.len(),
            );
            let design = SimDesign {
                layers,
                cavity_inset_m,
            };
            let geometry = match build_insertion_geometry(&scan, &design, caps, 2_500, 0.004) {
                Ok(g) => g,
                Err(e) => {
                    eprintln!("  {label}: geometry FAILED to build: {e}");
                    continue;
                }
            };
            let steps = patch_over_ramp(label, geometry, n_steps, INSERTION_CONTACT_KAPPA, true);
            let Some(&(depth_m, stats)) = steps.last() else {
                eprintln!("  {label}: no converged step produced a well-defined patch");
                continue;
            };
            eprintln!(
                "  {label}: deepest converged depth {:.4} mm of {:.4} mm ({:.0} %)",
                depth_m * 1e3,
                cavity_inset_m * 1e3,
                100.0 * depth_m / cavity_inset_m,
            );
            #[allow(clippy::cast_precision_loss)]
            let ramp_step_m = cavity_inset_m / n_steps as f64;
            report_derived_face_kappa(label, stats, ramp_step_m);
            report_required_marching_increment(
                label,
                stats,
                cavity_inset_m,
                INSERTION_CONTACT_DHAT,
            );
        }
    }

    /// How finely the ramp would have to march for the derived κ
    /// interval to be non-empty at a given band.
    ///
    /// The floor `σ / |b'(ρ·step)|` is the only one of the two bounds
    /// that moves with the marching schedule, so when the interval is
    /// empty the schedule is a lever on it and the band is the other.
    /// Holds `σ` and `ρ` fixed while varying the increment, which is
    /// first-order right — the converged state at a given depth is a
    /// property of that depth, not of how many increments reached it —
    /// and is exactly wrong if a finer march converges to a *different*
    /// state, which is itself worth finding out.
    fn report_required_marching_increment(
        label: &str,
        stats: PatchStats,
        cavity_inset_m: f64,
        d_hat: f64,
    ) {
        let Some(ceiling) = face_barrier_kappa(d_hat, 0.5 * d_hat, stats.traction_rest_pa) else {
            eprintln!("  {label}: no ceiling at d_hat = {:.3} mm", d_hat * 1e3);
            return;
        };
        eprintln!(
            "  {label} marching schedule at d_hat = {:.3} mm (ceiling {ceiling:.4e}, \
             rho {:.3} held fixed)",
            d_hat * 1e3,
            stats.rho_gap(),
        );
        eprintln!(
            "  {:>8} {:>10} {:>12} {:>14} {:>12}",
            "n_steps", "step/mm", "standoff/mm", "floor", "interval",
        );
        for n_steps in [16_u32, 32, 64, 128, 256, 512] {
            let step_m = cavity_inset_m / f64::from(n_steps);
            let standoff = step_m * stats.rho_gap();
            let floor = face_barrier_kappa(d_hat, standoff, stats.traction_rest_pa);
            let verdict = match floor {
                None => "undefined".to_owned(),
                Some(f) if f > ceiling => "EMPTY".to_owned(),
                Some(f) => format!("{:.2} decades", (ceiling / f).log10()),
            };
            eprintln!(
                "  {:>8} {:>10.4} {:>12.4} {:>14} {:>12}",
                n_steps,
                step_m * 1e3,
                standoff * 1e3,
                floor.map_or_else(|| "undefined".to_owned(), |f| format!("{f:.4e}")),
                verdict,
            );
        }
    }

    /// **Is σ a property of the compressed wall, or of the contact
    /// stiffness that compressed it?**
    ///
    /// This is the assumption the whole κ derivation rests on, and
    /// `tet10_yeoh_ipc_convergence` states it in as many words: *"`κ =
    /// σ / |b'(d)|` is only a derivation if `σ` — the traction the
    /// compressed plate pushes back with — is a property of the
    /// material and the compression, not of the barrier stiffness being
    /// solved for."* That fixture measured it and found σ spanning
    /// 1.003× while κ spanned decades.
    ///
    /// ⚠ **It has no right to hold here, and that is why it is
    /// measured.** On a stiff contact the gap adjusts and the load does
    /// not. The test of "stiff enough" is within one model and is the
    /// gap itself: this scene's area-weighted mean gap runs from a sixth
    /// to three quarters of the way into its 1 mm band as the ramp
    /// deepens, which is the regime where the equilibrium position *is*
    /// set by the stiffness.
    ///
    /// ⛔ It is **not** a comparison of the two `κ`. The penalty `κ`
    /// is `N/m` and the face `κ` is `Pa/m` — they differ by an area, and
    /// [`sim_soft::contact::barrier`]'s module docs call comparing their
    /// magnitudes a category error rather than a calibration
    /// observation.
    ///
    /// ⭐ Compares at the deepest depth **every** arm reached, not at
    /// each arm's own deepest. A stiffer contact may stall earlier, and
    /// comparing σ at two different depths would be reading the ramp's
    /// depth dependence as a stiffness dependence.
    ///
    /// Asserts nothing about the *outcome* — it is the size of the span
    /// that is the finding, and it is platform-dependent in the same way
    /// the stall boundary is. The one assert it carries protects the
    /// comparison: that the arms are read at the same depth.
    ///
    /// ```text
    /// cargo test -p cf-sim-research --release --bin cf-sim-research \
    ///     the_design_traction_is_measured_against_the_stiffness_that_produced_it \
    ///     -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "release-mode ramp per stiffness — measurement, asserts nothing; run with --ignored --nocapture"]
    fn the_design_traction_is_measured_against_the_stiffness_that_produced_it() {
        let cavity_inset_m = 0.003;
        let n_steps = 16_usize;
        let design = SimDesign {
            cavity_inset_m,
            layers: vec![layer(0.010, "ECOFLEX_00_30")],
        };
        eprintln!(
            "SIGMA vs PENALTY STIFFNESS — synthetic sphere, cavity {:.1} mm, {n_steps} steps. \
             Shipped kappa is {:.1e}.",
            cavity_inset_m * 1e3,
            INSERTION_CONTACT_KAPPA,
        );

        report_sigma_vs_stiffness("sphere", cavity_inset_m, n_steps, || {
            let scan = icosphere(0.040, 3);
            build_insertion_geometry(&scan, &design, &[], 2_000, 0.004)
                .expect("synthetic-sphere geometry should build")
        });

        // The scan arm, when the fixture is here. The sphere arm runs
        // without it, so a missing fixture is not a failure — but it is
        // said out loud, because the scene that matters is this one.
        let scan_path = std::env::var("CF_SIM_RESEARCH_SPIKE_SCAN").map_or_else(
            |_| PathBuf::from("/Users/jonhillesheim/scans/sock_over_capsule.cleaned.stl"),
            PathBuf::from,
        );
        if scan_path.exists() {
            let scan = load_stl(&scan_path).expect("load the iter-1 cleaned scan");
            let scan_design = SimDesign {
                cavity_inset_m,
                layers: vec![layer(0.010, "ECOFLEX_00_30")],
            };
            report_sigma_vs_stiffness("1layer", cavity_inset_m, n_steps, || {
                build_insertion_geometry(&scan, &scan_design, &[], 2_500, 0.004)
                    .expect("iter-1 single-layer geometry should build")
            });
        } else {
            eprintln!(
                "\n⛔ NOT MEASURED — the scan arm did not run: no fixture at {}. \
                 The sphere result below is an idealised geometry, and the scene the \
                 bridge runs is the one that is missing. Set CF_SIM_RESEARCH_SPIKE_SCAN.",
                scan_path.display(),
            );
        }
    }

    /// Sweep the penalty stiffness on one scene and report σ, the gap
    /// distribution and ρ at the deepest depth every arm reached.
    ///
    /// Rebuilds the geometry per arm because the ramp consumes it.
    fn report_sigma_vs_stiffness(
        scene: &str,
        cavity_inset_m: f64,
        n_steps: usize,
        build: impl Fn() -> InsertionGeometry,
    ) {
        // One decade either side of the shipped 1e3, plus two above it,
        // because the question is where σ STOPS moving and that is the
        // stiff end. 1e2 and 1e6 are there to bracket: the first is
        // expected to penetrate grossly and the second to stall, and an
        // arm that does neither would mean the bracket is too narrow.
        const KAPPAS: [f64; 5] = [1.0e2, 1.0e3, 1.0e4, 1.0e5, 1.0e6];
        eprintln!("\n═══ {scene} ═══");
        let mut arms: Vec<(f64, Vec<(f64, PatchStats)>)> = Vec::new();
        for contact_kappa in KAPPAS {
            eprintln!("\n--- {scene}, penalty kappa = {contact_kappa:.1e} ---");
            let label = format!("k={contact_kappa:.0e}");
            let steps = patch_over_ramp(&label, build(), n_steps, contact_kappa, false);
            arms.push((contact_kappa, steps));
        }

        // An arm that converged NOTHING is a result, not a reason to
        // drop the comparison: a stiffness the solve cannot reach says
        // the stiff limit is not available on this path. Take the
        // common depth over the arms that got somewhere, and give the
        // rest a row saying so.
        let stalled: Vec<f64> = arms
            .iter()
            .filter(|(_, s)| s.is_empty())
            .map(|(k, _)| *k)
            .collect();
        if !stalled.is_empty() {
            eprintln!(
                "\n  ⛔ {scene}: {} of {} stiffnesses converged NO step at all \
                 ({}) — the stiff end of this sweep is not reachable on this path, so \
                 the limit below is the stiffest one that solved, not the rigid one.",
                stalled.len(),
                arms.len(),
                stalled
                    .iter()
                    .map(|k| format!("{k:.0e}"))
                    .collect::<Vec<_>>()
                    .join(", "),
            );
        }
        let Some(common) = arms.iter().map(|(_, s)| s.len()).filter(|&n| n > 0).min() else {
            eprintln!("  {scene}: no stiffness converged a step — nothing to compare");
            return;
        };
        let depth_m = arms
            .iter()
            .find(|(_, s)| !s.is_empty())
            .map_or(0.0, |(_, s)| s[common - 1].0);
        eprintln!(
            "\n  {scene}: deepest depth every arm reached: step {common}/{n_steps} = {:.4} mm",
            depth_m * 1e3,
        );
        eprintln!(
            "  {:>10} {:>8} {:>11} {:>11} {:>11} {:>9} {:>9} {:>11} {:>5}",
            "kappa",
            "steps",
            "sig_r/kPa",
            "min_sd/mm",
            "p05_sd/mm",
            "rho",
            "rho_tail",
            "cancel",
            "seated",
        );
        let mut sigmas = Vec::new();
        // ⛔ Tracked separately, because an arm whose patch is through
        // the intruder has no traction to contribute to a span — see the
        // non-penetrating comparison below.
        let mut sigmas_seated = Vec::new();
        for (contact_kappa, steps) in &arms {
            let Some(&(d, stats)) = steps.get(common - 1) else {
                eprintln!(
                    "  {:>10.1e} {:>8} {:>11} {:>11} {:>11} {:>9} {:>9} {:>11} {:>5}",
                    contact_kappa,
                    steps.len(),
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                    "—",
                );
                continue;
            };
            assert!(
                (d - depth_m).abs() < 1e-12,
                "the arms must be compared at the same depth; got {d:e} vs {depth_m:e}",
            );
            eprintln!(
                "  {:>10.1e} {:>8} {:>11.2} {:>11.4} {:>11.4} {:>9.3} {:>9.3} {:>11.1} {:>5}",
                contact_kappa,
                steps.len(),
                stats.traction_rest_pa * 1e-3,
                stats.min_sd_m * 1e3,
                stats.p05_sd_m * 1e3,
                stats.rho_gap(),
                stats.rho_tail(),
                stats.cancellation(),
                if stats.min_sd_m > 0.0 {
                    ""
                } else {
                    "⛔ THROUGH"
                },
            );
            sigmas.push((*contact_kappa, stats.traction_rest_pa));
            if stats.min_sd_m > 0.0 {
                sigmas_seated.push((*contact_kappa, stats.traction_rest_pa));
            }
        }
        let (lo, hi) = sigmas
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &(_, s)| {
                (l.min(s), h.max(s))
            });
        // The span is over the stiffnesses that actually reached the
        // common depth — quoting the swept range would credit the
        // comparison with arms that contributed nothing to it.
        let (k_lo, k_hi) = sigmas
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &(k, _)| {
                (l.min(k), h.max(k))
            });
        eprintln!(
            "  {scene}: sigma spans {:.4}x over kappa {k_lo:.0e}..{k_hi:.0e} \
             ({:.0} decades), ALL arms.",
            hi / lo,
            (k_hi / k_lo).log10(),
        );
        // ⭐ The comparison that means anything: per decade, and over
        // arms that are actually seated. A span that includes a patch
        // driven through the intruder is a span over a quantity that is
        // not a traction there, and a 3-decade span is not comparable
        // to a 2-decade one.
        if sigmas_seated.len() >= 2 {
            let (s_lo, s_hi) = sigmas_seated
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &(_, s)| {
                    (l.min(s), h.max(s))
                });
            let (ks_lo, ks_hi) = sigmas_seated
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &(k, _)| {
                    (l.min(k), h.max(k))
                });
            let decades = (ks_hi / ks_lo).log10();
            let per_decade = if decades > 0.0 {
                (s_hi / s_lo).powf(1.0 / decades)
            } else {
                f64::NAN
            };
            // ⛔ SCENE-LOCAL. "Seated" is a per-scene predicate, so it
            // selects a DIFFERENT arm set on each scene and the numbers
            // it produces cannot be compared between them. Printed
            // second, and labelled, because quoting it as a cross-scene
            // figure is a mistake that has already been made twice.
            eprintln!(
                "  {scene}: SCENE-LOCAL — over all {} seated arms (min_sd > 0, kappa \
                 {ks_lo:.0e}..{ks_hi:.0e}) sigma spans {:.4}x = {per_decade:.4}x per \
                 decade. ⛔ NOT comparable across scenes: `seated` picks a \
                 different arm set on each.",
                sigmas_seated.len(),
                s_hi / s_lo,
            );
        } else {
            eprintln!(
                "  {scene}: fewer than two arms were seated (min_sd > 0), so there is no \
                 comparable span — every other arm read its traction off a patch driven \
                 through the intruder.",
            );
        }
        // The stiff end is the part that transfers: as the contact
        // approaches rigid the gap stops absorbing the load and sigma
        // stops moving. Two adjacent arms that agree are the evidence
        // that a limit was reached; two that do not mean the sweep
        // stopped early and the limit is not in hand.
        if sigmas.len() < 2 {
            eprintln!(
                "  {scene}: fewer than two stiffnesses reached the common depth — there \
                 is no span to report",
            );
            return;
        }
        // The stiffest arm that SOLVED is the closest this path gets to
        // a design traction, so the derivation is worth seeing there and
        // not only at the shipped stiffness — which is the one reading
        // the sweep has just disqualified.
        let stiffest = sigmas.last().and_then(|&(k_stiff, _)| {
            arms.iter()
                .find(|(k, _)| (k - k_stiff).abs() < f64::EPSILON * k_stiff)
                .and_then(|(_, steps)| steps.get(common - 1))
                .map(|&(_, stats)| (k_stiff, stats))
        });
        if let Some((k_stiff, stats)) = stiffest {
            #[allow(clippy::cast_precision_loss)]
            let ramp_step_m = cavity_inset_m / n_steps as f64;
            report_derived_face_kappa(&format!("{scene}@k={k_stiff:.0e}"), stats, ramp_step_m);
        }
        if let [.., (k_a, s_a), (k_b, s_b)] = sigmas.as_slice() {
            // ⭐ THE CROSS-SCENE FIGURE. The two stiffest arms that
            // solved are the same κ on every scene here, so this is the
            // one number that compares between them — and against the
            // fixture, whose own value is recorded in
            // [`FIXTURE_SIGMA_SPAN`] with the command that prints it.
            eprintln!(
                "  {scene}: CROSS-SCENE — the two stiffest arms that solved \
                 ({k_a:.0e} -> {k_b:.0e}) give sigma {:.2} -> {:.2} kPa = {:.4}x per \
                 decade, against the fixture's {:.4}x per decade ({FIXTURE_SIGMA_SPAN}x \
                 over {FIXTURE_SIGMA_SPAN_DECADES} decades).",
                s_a * 1e-3,
                s_b * 1e-3,
                s_b / s_a,
                FIXTURE_SIGMA_SPAN.powf(1.0 / FIXTURE_SIGMA_SPAN_DECADES),
            );
        }
    }

    /// The shipped ramp is the delegated one at the shipped stiffness —
    /// bit for bit.
    ///
    /// [`run_insertion_ramp`] now routes through
    /// [`run_insertion_ramp_at_kappa`], which is a refactor of the
    /// path every consumer of this module runs. Without this gate the
    /// delegation could hand the shipped path a different stiffness and
    /// nothing would fail: the probes that exercise the knob are all
    /// `#[ignore]`d, and a ramp that converges at the wrong κ still
    /// converges.
    ///
    /// ⭐ Compares `x_final` for **equality**, not closeness. The
    /// delegation is supposed to be a rename, so anything at all in the
    /// last bit is a change, and a tolerance here would be a place for
    /// one to hide.
    ///
    /// ⚠ **It does require the fixture to converge a step, and that is
    /// deliberately not the stall-boundary mistake** its sibling gate
    /// was fixed for. A 2-step ramp asks this fixture for 1.5 mm in the
    /// first increment; `the_insertion_solves_convergence_is_bounded_by_its_tolerance`
    /// already solves the *same* fixture to 2.8 mm in a **single** step
    /// in CI, so this is strictly the easier problem and is well inside
    /// the convergent regime rather than near its edge. The assertion
    /// is a fixture-health precondition, not a claim about where Armijo
    /// gives out.
    #[test]
    fn the_shipped_ramp_runs_at_the_shipped_contact_stiffness() {
        let n_steps = 2;
        let shipped = run_insertion_ramp(tolerance_fixture(), n_steps)
            .expect("the shipped ramp must run on the tolerance fixture");
        let delegated =
            run_insertion_ramp_at_kappa(tolerance_fixture(), n_steps, INSERTION_CONTACT_KAPPA)
                .expect("the delegated ramp must run on the tolerance fixture");

        // Two ramps that both converged nothing would compare equal and
        // assert nothing at all.
        assert!(
            !shipped.steps.is_empty(),
            "the fixture must converge at least one step, or this gate compares two \
             empty ramps and passes on any stiffness",
        );
        assert_eq!(
            shipped.steps.len(),
            delegated.steps.len(),
            "the delegation must converge the same number of steps",
        );
        assert_eq!(
            shipped.n_pinned, delegated.n_pinned,
            "the delegation must pin the same outer skin",
        );
        for (k, (a, b)) in shipped.steps.iter().zip(&delegated.steps).enumerate() {
            assert_eq!(
                a.iter_count, b.iter_count,
                "step {k}: Newton iteration count"
            );
            assert_eq!(
                a.final_residual_norm.to_bits(),
                b.final_residual_norm.to_bits(),
                "step {k}: final residual norm",
            );
            assert_eq!(
                a.readout.n_active_contact_pairs, b.readout.n_active_contact_pairs,
                "step {k}: active contact pair count",
            );
            assert_eq!(a.x_final, b.x_final, "step {k}: converged positions");
        }
    }

    /// The stiffness knob reaches BOTH the solve and the readout.
    ///
    /// [`the_shipped_ramp_runs_at_the_shipped_contact_stiffness`] pins
    /// the delegation's *default*, and cannot see the argument being
    /// dropped: at the shipped value a
    /// [`run_insertion_ramp_at_kappa`] that ignored `contact_kappa`
    /// entirely would agree with the shipped ramp exactly. Everything
    /// that exercises a non-shipped stiffness is `#[ignore]`d, so
    /// without this gate the knob could be inert in CI and the
    /// stiffness-independence probe would be measuring one κ four
    /// times while printing four different labels.
    ///
    /// Two halves, because there are two places the argument is
    /// threaded:
    ///
    /// - **the readout** — at FIXED positions the penalty traction is
    ///   linear in κ, so tenfold κ is tenfold σ, exactly;
    /// - **the solve** — a tenfold stiffer contact must converge
    ///   somewhere else.
    #[test]
    fn the_contact_stiffness_knob_reaches_both_the_solve_and_the_readout() {
        let geometry = tolerance_fixture();
        let rest: Vec<Vec3> = geometry.mesh.positions().to_vec();
        let referenced: Vec<VertexId> = referenced_vertices(&geometry.mesh);
        let rest_areas = boundary_vertex_areas(
            Mesh::<Yeoh>::positions(&geometry.mesh),
            Mesh::<Yeoh>::boundary_faces(&geometry.mesh),
        );
        let read_at = |kappa: f64| {
            // `interference_m = 0` sits the intruder flush with the
            // cavity wall, which is where the rest patch is engaged.
            let contact = intruder_contact_at_kappa(
                &geometry.intruder,
                geometry.bounds,
                0.0,
                geometry.cavity_offset_m,
                kappa,
            );
            let raw = contact.per_pair_readout(&geometry.mesh, &rest);
            patch_stats(
                &filter_pair_readouts_to_referenced(raw, &referenced),
                &rest_areas,
            )
        };
        let base = read_at(INSERTION_CONTACT_KAPPA)
            .expect("the fixture's cavity wall must carry an active patch at rest");
        let tenfold = read_at(10.0 * INSERTION_CONTACT_KAPPA)
            .expect("the fixture's cavity wall must carry an active patch at rest");
        assert!(
            base.n_pairs > 0,
            "an empty patch would make every ratio below 0/0 and this gate vacuous",
        );
        assert_eq!(
            base.n_pairs, tenfold.n_pairs,
            "the active set is a GAP test, so it must not move with the stiffness",
        );
        let ratio = tenfold.traction_pa / base.traction_pa;
        assert!(
            (ratio - 10.0).abs() < 1.0e-9,
            "at fixed positions the penalty traction is linear in κ, so tenfold κ must \
             be tenfold σ; got {ratio}× — the readout is not using the stiffness it \
             was handed",
        );

        let shipped = run_insertion_ramp_at_kappa(tolerance_fixture(), 2, INSERTION_CONTACT_KAPPA)
            .expect("the fixture must ramp at the shipped stiffness");
        let stiffer =
            run_insertion_ramp_at_kappa(tolerance_fixture(), 2, 10.0 * INSERTION_CONTACT_KAPPA)
                .expect("the fixture must ramp at ten times the shipped stiffness");
        // ⛔ The stiffer arm is NOT required to converge. Whether a
        // tenfold stiffer contact still solves is a STALL BOUNDARY, and
        // a stall boundary is decided by arithmetic that differs
        // between platforms - asserting it here would be asserting the
        // LOCATION of an edge, which is the mistake step 0 already made
        // once and had to fix in CI.
        //
        // What is asserted instead holds on either side of that edge:
        // the two runs must DIFFER, in step count or in where the first
        // step landed. Dropping the argument makes them identical in
        // both, so the mutation still fails this.
        assert!(
            !shipped.steps.is_empty(),
            "the shipped arm must converge a step, or this half compares nothing",
        );
        let differs = shipped.steps.len() != stiffer.steps.len()
            || shipped.steps[0].x_final != stiffer.steps[0].x_final;
        assert!(
            differs,
            "a tenfold stiffer contact must change the solve, and it converged the \
             same {} step(s) to identical positions - the stiffness never reached it",
            shipped.steps.len(),
        );
    }

    /// **One tight vertex must not set ρ — and the tail must be read
    /// from the right end, by area.**
    ///
    /// The κ floor divides `|b'|` at `ρ · step`, so ρ's denominator is
    /// a number that moves a shipped constant. Built on `min_sd` that
    /// denominator is a single pair, and on a scan-derived patch a
    /// single bad tet is the expected case, not the pathological one.
    ///
    /// ⭐ **The fixture is deliberately lopsided**, because a
    /// symmetric one cannot tell three different implementations
    /// apart. 12 pairs over 50 µm²: one outlier at 0.01 mm carrying
    /// 1 µm², ten at 0.5 mm carrying 1 µm² each, and one at 0.9 mm
    /// carrying the remaining 39 µm². The 5 % cut is 2.5 µm², which
    /// lands strictly inside the 0.5 mm group, so
    ///
    /// - walking from the tight end gives **0.5 mm** — correct;
    /// - walking from the loose end gives 0.9 mm;
    /// - weighting by pair COUNT instead of area gives 0.01 mm, since
    ///   one pair of twelve already exceeds 5 %.
    ///
    /// The two ρ then come out **exactly 50× apart**, which is the size
    /// of the mistake available to a derivation that uses one while
    /// saying the other.
    #[test]
    fn the_patch_nonuniformity_on_the_tail_is_not_moved_by_a_single_tight_pair() {
        let micro = 1.0e-6;
        let force = Vec3::new(0.0, 0.0, -1.0);
        let mut readouts = vec![pair_readout(0, 0.01e-3, micro, force)];
        for i in 1..=10 {
            readouts.push(pair_readout(i, 0.5e-3, micro, force));
        }
        readouts.push(pair_readout(11, 0.9e-3, 39.0 * micro, force));

        // Rest == deformed here, which is the other case worth pinning:
        // an unstretched patch must report the same σ on both bases.
        let mut rest_areas = vec![micro; 11];
        rest_areas.push(39.0 * micro);
        let stats = patch_stats(&readouts, &rest_areas).expect("12 well-defined pairs");
        let close = |got: f64, want: f64, what: &str| {
            assert!(
                (got - want).abs() <= 1e-9 * want.abs().max(1.0),
                "{what}: got {got:e}, want {want:e}",
            );
        };
        assert_eq!(stats.n_pairs, 12, "every pair here has a surface patch");
        close(stats.area_m2, 50.0 * micro, "the areas are 1 + 10 + 39 µm²");
        close(stats.min_sd_m, 0.01e-3, "the outlier is the tightest pair");
        close(
            stats.max_sd_m,
            0.9e-3,
            "the loose pair carries most of the area",
        );
        close(
            stats.p05_sd_m,
            0.5e-3,
            "the 5 % tail walks from the TIGHT end, by AREA, past a 2 %-area outlier",
        );
        // (1·0.01 + 10·0.5 + 39·0.9) / 50 = 0.8022 mm.
        close(stats.mean_sd_m, 0.802_2e-3, "the mean gap is area-weighted");
        close(
            stats.rho_gap(),
            80.22,
            "ρ on the minimum is set by the one outlier",
        );
        close(stats.rho_tail(), 1.604_4, "ρ on the tail is not");
        close(
            stats.rho_gap() / stats.rho_tail(),
            50.0,
            "the two definitions differ by a factor here, not a percent",
        );
        close(
            stats.traction_rest_pa,
            stats.traction_pa,
            "an unstretched patch reads the same σ on both area bases",
        );
    }

    // ─── THE BRIDGE — Tet10 + the IPC face barrier ──────────────────

    /// The bridge takes the FACE barrier, not the per-vertex path.
    ///
    /// ⭐ **This is the gate the recon doc's §9 retraction exists for.** A
    /// compile probe on this exact swap once went green for the wrong reason:
    /// `SdfMeshedTetMesh<Yeoh>` *compiles* against the Tet10 types but returns
    /// `None` from `boundary_faces6()`, so contact silently takes the
    /// per-vertex path. The capability is chosen by a runtime `Option`, so the
    /// only honest probe is of the SELECTOR and of what it produces.
    ///
    /// ⚠ **Probe `active_pairs`, not `per_pair_readout`.** The first thing this
    /// gate found is that the two do not agree on pair kind *by design*: the
    /// solver consumes `ContactPair::Face`, while the face path's READOUT is
    /// per-NODE and reports `ContactPair::Vertex` naming the six face nodes
    /// (midsides carry the load, corners ~0). Asserting "the readout is all
    /// Face pairs" fails on a perfectly correct bridge — measured here, 1442 of
    /// 1442 readouts were `Vertex` on a mesh whose solver contact was entirely
    /// face-integrated.
    ///
    /// Without this, the whole bridge could land, pass, and be a no-op wearing
    /// Tet10's type parameters.
    #[test]
    fn the_bridge_selects_the_face_barrier_not_the_vertex_path() {
        use sim_soft::ActivePairsFor;

        let g = tolerance_fixture();
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);

        // Negative control: without it, an unconditional `is_some()` would pass
        // on any mesh and this gate would be measuring nothing.
        assert!(
            Mesh::<Yeoh>::boundary_faces6(&g.mesh).is_none(),
            "the shipped Tet4 mesh must NOT surface P2 faces — if it did, this \
             gate could not tell the two contact paths apart",
        );
        assert!(
            Mesh::<Yeoh>::boundary_faces6(&tet10).is_some(),
            "the enriched mesh must surface P2 faces, or IPC falls back to the \
             per-vertex path and the bridge is a no-op",
        );

        let contact = intruder_ipc_contact_at(
            &g.intruder,
            g.bounds,
            -g.cavity_offset_m,
            g.cavity_offset_m,
            1.0e7,
            BRIDGE_CONTACT_DHAT_M,
        );

        // THE SOLVER'S pairs — what actually gets scattered into the residual.
        let positions = tet10.positions().to_vec();
        let pairs = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet10, &positions);
        // EMPTY ≠ evidence: an empty set satisfies "all are faces" vacuously.
        assert!(
            !pairs.is_empty(),
            "the seated intruder must produce active pairs; an empty set would \
             satisfy the face-kind check below vacuously",
        );
        let n_vertex = pairs
            .iter()
            .filter(|p| matches!(p, ContactPair::Vertex { .. }))
            .count();
        assert_eq!(
            n_vertex,
            0,
            "every SOLVER pair on the bridge's path must be a Face pair; {} of \
             {} were Vertex pairs, which means the face barrier was not selected",
            n_vertex,
            pairs.len(),
        );

        // And the same scene on the un-enriched mesh must take the other path,
        // or "Face pairs appear" is a property of the contact model rather than
        // of the mesh swap this bridge performs.
        let tet4_positions = Mesh::<Yeoh>::positions(&g.mesh).to_vec();
        let tet4_pairs = ActivePairsFor::<Yeoh>::active_pairs(&contact, &g.mesh, &tet4_positions);
        assert!(
            !tet4_pairs.is_empty()
                && tet4_pairs
                    .iter()
                    .all(|p| matches!(p, ContactPair::Vertex { .. })),
            "the Tet4 mesh must still take the per-vertex path ({} pairs)",
            tet4_pairs.len(),
        );
    }

    /// Enrichment preserves the corner ids the rest of the pipeline holds.
    ///
    /// The BCs, Γ and the per-tet readouts are all built from `[VertexId; 4]`
    /// corner indices. If `from_tet4` renumbered corners, every one of them
    /// would keep working and silently point at a different material point —
    /// no panic, no type error, a wrong answer. So the invariant is asserted
    /// where the bridge relies on it, not only where it is implemented.
    #[test]
    fn the_enriched_mesh_preserves_the_tet4_corners() {
        let g = tolerance_fixture();
        let tet4 = &g.mesh;
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(tet4);

        assert_eq!(
            Mesh::<Yeoh>::n_tets(&tet10),
            Mesh::<Yeoh>::n_tets(tet4),
            "enrichment must not change the tet count",
        );
        assert!(
            Mesh::<Yeoh>::n_vertices(&tet10) > Mesh::<Yeoh>::n_vertices(tet4),
            "enrichment must ADD midside nodes",
        );
        assert_eq!(
            tet10.n_corners(),
            Mesh::<Yeoh>::n_vertices(tet4),
            "every Tet4 vertex must remain a corner, including BCC orphans",
        );

        // Corner connectivity, index for index.
        #[allow(clippy::cast_possible_truncation)]
        for t in 0..Mesh::<Yeoh>::n_tets(tet4) as TetId {
            assert_eq!(
                Mesh::<Yeoh>::tet_vertices(&tet10, t),
                Mesh::<Yeoh>::tet_vertices(tet4, t),
                "tet {t}'s corner ids moved under enrichment",
            );
        }
        // Corner positions, bit for bit — the BCs pick vertices by POSITION.
        let (p4, p10) = (
            Mesh::<Yeoh>::positions(tet4),
            Mesh::<Yeoh>::positions(&tet10),
        );
        assert_eq!(
            &p10[..p4.len()],
            p4,
            "corner rest positions must be bit-identical, or the outer-skin \
             pin-band selects a different vertex set",
        );
        assert_eq!(
            Mesh::<Yeoh>::boundary_faces(&tet10),
            Mesh::<Yeoh>::boundary_faces(tet4),
            "Γ is built from the corner boundary faces; they must be copied \
             verbatim",
        );
    }

    /// The midside readouts survive the orphan filter — and would not, on a
    /// corner-only set.
    ///
    /// ⛔ **The failure this pins is total and silent.** On the face path every
    /// loaded node is a MIDSIDE: the face-integrated barrier puts ~0 on the
    /// corners. The ramp passes its readouts through
    /// [`filter_pair_readouts_to_referenced`], so if the "referenced" set were
    /// ever corners-only, that call would delete the entire contact patch and
    /// return a clean, empty, non-panicking readout — conformity 0 on a
    /// perfectly good design.
    ///
    /// It holds today because `referenced_vertex_mask` walks
    /// `tet_midside_nodes` as well as `tet_vertices`. That is a property of
    /// sim-soft, relied on here, so it is measured rather than trusted — and
    /// the corner-only counterfactual is run to show the gate can see the
    /// difference.
    #[test]
    fn the_bridges_midside_readouts_survive_the_orphan_filter() {
        let g = tolerance_fixture();
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        let referenced: Vec<VertexId> = referenced_vertices(&tet10);
        let n_corners = tet10.n_corners();

        // The set must actually reach past the corners, or the counterfactual
        // below is not a counterfactual.
        assert!(
            referenced.iter().any(|&v| v as usize >= n_corners),
            "the referenced set must include midside nodes",
        );
        // And the fixture must carry orphans, or "drops zero" is vacuous.
        let n_orphans = Mesh::<Yeoh>::n_vertices(&tet10) - referenced.len();
        assert!(
            n_orphans > 0,
            "this fixture must carry orphan lattice points, or the filter is \
             measuring nothing",
        );

        let contact = intruder_ipc_contact_at(
            &g.intruder,
            g.bounds,
            -g.cavity_offset_m,
            g.cavity_offset_m,
            1.0e7,
            BRIDGE_CONTACT_DHAT_M,
        );
        let positions = tet10.positions().to_vec();
        let raw = contact.per_pair_readout(&tet10, &positions);
        assert!(!raw.is_empty(), "the seated intruder must produce pairs");
        let n_raw = raw.len();
        let n_midside = raw
            .iter()
            .filter(|r| match r.pair {
                ContactPair::Vertex { vertex_id, .. } => vertex_id as usize >= n_corners,
                _ => false,
            })
            .count();
        assert!(
            n_midside > 0,
            "the face-consistent readout must name midside nodes — they are the \
             ones carrying the load",
        );

        let kept = filter_pair_readouts_to_referenced(raw.clone(), &referenced);
        assert_eq!(
            kept.len(),
            n_raw,
            "the orphan filter must drop ZERO of {n_raw} readouts; it dropped {}",
            n_raw - kept.len(),
        );

        // The counterfactual: a corners-only set silently deletes the patch.
        let corners_only: Vec<VertexId> = referenced
            .iter()
            .copied()
            .filter(|&v| (v as usize) < n_corners)
            .collect();
        let starved = filter_pair_readouts_to_referenced(raw, &corners_only);
        assert!(
            starved.len() < n_raw,
            "a corners-only set must visibly starve the readout, or this gate \
             cannot tell a correct filter from a destructive one",
        );
    }

    /// `κ` is the CEILING, and the floor is only reported.
    ///
    /// ⛔ An earlier revision asserted κ was the geometric centre of
    /// `[floor, ceiling]`. That selector was measured wrong on the product scene —
    /// see `bridge_face_barrier_kappa`. The floor derives κ from the
    /// increment, and on the product scene held/step fell 0.94 → 0.86 → 0.81
    /// across 16/32/64 steps with κ at the floor (κ, step and depth changed
    /// together).
    ///
    /// What this pins now is the requirement that survived: the ceiling, which
    /// describes something κ genuinely controls — how far into the cushioning
    /// regime the barrier sits.
    #[test]
    fn the_bridges_stiffness_is_the_ceiling_and_the_floor_is_only_reported() {
        // The product inset (`base_mold`), not sock's 3 mm.
        let inset_m = 0.005;
        let n_steps = 16.0;
        let step = inset_m / n_steps;

        let ceiling = face_barrier_kappa(
            BRIDGE_CONTACT_DHAT_M,
            0.5 * BRIDGE_CONTACT_DHAT_M,
            BRIDGE_DESIGN_TRACTION_PA,
        )
        .expect("the ceiling must be derivable");
        let kappa = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, step)
            .expect("κ must derive on the shipped ramp schedule");
        assert!(
            (kappa - ceiling).abs() <= 1e-9 * ceiling,
            "κ must BE the ceiling {ceiling:.6e}, got {kappa:.6e}",
        );

        // The floor is still computable and still worth printing — it is just
        // no longer a selector.
        let floor = face_barrier_kappa(
            BRIDGE_CONTACT_DHAT_M,
            BRIDGE_PATCH_NONUNIFORMITY * step,
            BRIDGE_DESIGN_TRACTION_PA,
        );
        println!(
            "d_hat {:.2} mm · step {:.4} mm · floor {} · ceiling {ceiling:.4e} · shipped {kappa:.4e}",
            BRIDGE_CONTACT_DHAT_M * 1e3,
            step * 1e3,
            floor.map_or_else(|| "(none)".to_string(), |f| format!("{f:.4e}")),
        );

        // ⚠ κ must NOT depend on the schedule any more. That independence is
        // the whole fix: it is what lets a finer march shrink the step under a
        // standoff that stays put.
        for finer in [32.0_f64, 64.0, 128.0] {
            let k = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, inset_m / finer)
                .expect("κ must derive at finer schedules too");
            assert!(
                (k - kappa).abs() <= 1e-9 * kappa,
                "κ must not move with the schedule: {finer} steps gave {k:.6e}, \
                 16 steps gave {kappa:.6e}",
            );
        }

        // A mis-specified schedule must still be REFUSED, not clamped.
        assert!(
            bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, BRIDGE_CONTACT_DHAT_M).is_err(),
            "an increment as wide as the band must surface an error",
        );
        assert!(
            bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, 0.0).is_err(),
            "a zero increment must surface an error",
        );
    }

    /// ⛔ Positions from another mesh are refused, not read — the length check
    /// `ReadoutMesh` makes because a Tet4 view of an enriched mesh would
    /// otherwise index its positions without complaint.
    #[test]
    #[should_panic(expected = "they come from different meshes")]
    fn a_readout_mesh_refuses_positions_from_another_mesh() {
        let rest = unit_tet_rest();
        // One node more than the mesh has: the shape of a Tet10 state read
        // through a Tet4 mesh.
        let mut curr = rest.clone();
        curr.push(Vec3::new(0.5, 0.0, 0.0));
        let _ = unit_tet_readout_mesh(rest).readouts(&curr);
    }

    /// ⭐ The readout IS the Tet10 strain, at every Gauss point — and the
    /// corner readout it replaced is not.
    ///
    /// A quadratic displacement, `x ↦ x + (x², 0, 0) / 4`, is applied to one
    /// reference element. Tet10 reproduces a quadratic field exactly, so `F` at
    /// each Gauss point must be the analytic gradient there,
    /// `diag(1 + x/2, 1, 1)` — a referent that owes nothing to the code under
    /// test. The four corners alone see only the linear part and must miss it;
    /// the affine control shows that the gap measures element order, not a bug
    /// in this test.
    ///
    /// It also reads a CURVED element (two midsides moved off their chords)
    /// under an affine field: `F` must still be `A` at every point, and each
    /// point must be weighted by its own `|det J_rest|`, as the solver weights a
    /// curved element.
    #[test]
    fn the_readout_resolves_the_tet10_strain_at_every_gauss_point() {
        use sim_soft::element::TET10_EDGE_NODES;
        let corners = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let mut rest: Vec<Vec3> = corners.to_vec();
        for (a, b) in TET10_EDGE_NODES {
            rest.push(0.5 * (corners[a] + corners[b]));
        }
        let tet10 = ReadoutMesh {
            rest_positions: rest.clone(),
            elements: ReadoutElements::Tet10(vec![[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]]),
            materials: vec![unit_tet_material()],
        };
        let corners_only = ReadoutMesh {
            rest_positions: rest.clone(),
            elements: ReadoutElements::Tet4(vec![[0, 1, 2, 3]]),
            materials: vec![unit_tet_material()],
        };

        let warp = |p: &Vec3| Vec3::new(p.x + 0.25 * p.x * p.x, p.y, p.z);
        let curr: Vec<Vec3> = rest.iter().map(warp).collect();
        let points = tet10.gauss_point_readouts(0, &curr);
        assert_eq!(points.len(), 4, "Tet10 is read at its four Gauss points");
        let corner_f = corners_only.gauss_point_readouts(0, &curr)[0].f;

        let mut worst_corner_gap = 0.0_f64;
        for (q, (point, (xi, _))) in points.iter().zip(Tet10.gauss_points()).enumerate() {
            // The rest map of the reference tet is the identity, so a Gauss
            // point's physical x IS its ξ.
            let analytic = Matrix3::from_diagonal(&Vec3::new(1.0 + 0.5 * xi.x, 1.0, 1.0));
            let err = (point.f - analytic).norm();
            assert!(
                err < 1e-12,
                "Gauss point {q}: F must be the analytic gradient (off by {err:.3e})",
            );
            assert!(
                (point.volume_fraction - 0.25).abs() < 1e-15,
                "a straight element splits its volume evenly (point {q}: {})",
                point.volume_fraction,
            );
            worst_corner_gap = worst_corner_gap.max((corner_f - analytic).norm() / analytic.norm());
        }
        assert!(
            worst_corner_gap > 1e-2,
            "the corners must miss the quadratic part — they matched to \
             {worst_corner_gap:.3e}, so this field no longer discriminates",
        );

        // The summary is a reduction of exactly those points.
        let summary = &tet10.readouts(&curr)[0];
        let mean: f64 = points
            .iter()
            .map(|p| 0.25 * p.energy_density_j_per_m3)
            .sum();
        assert!(
            (summary.energy_density_j_per_m3 - mean).abs() <= 1e-12 * mean.abs(),
            "element energy must be the volume-weighted mean ({} vs {mean})",
            summary.energy_density_j_per_m3,
        );
        let peak = points
            .iter()
            .map(|p| p.first_piola_frobenius_pa)
            .fold(0.0, f64::max);
        assert_eq!(summary.first_piola_frobenius_pa, peak, "peak stress");
        let stretches = || {
            points
                .iter()
                .flat_map(|p| p.principal_stretches.iter().copied())
        };
        assert_eq!(
            summary.max_principal_stretch,
            stretches().fold(f64::NEG_INFINITY, f64::max)
        );
        assert_eq!(
            summary.min_principal_stretch,
            stretches().fold(f64::INFINITY, f64::min)
        );

        // Control: on an AFFINE field every point and the corners agree.
        let a = Matrix3::new(1.3, 0.2, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.1);
        let curr_affine: Vec<Vec3> = rest.iter().map(|p| a * p).collect();
        for point in tet10.gauss_point_readouts(0, &curr_affine) {
            assert!(
                (point.f - a).norm() < 1e-12,
                "an affine field has one F everywhere; a Gauss point read {}",
                point.f,
            );
        }
        let corner_affine = corners_only.gauss_point_readouts(0, &curr_affine)[0].f;
        assert!(
            (corner_affine - a).norm() < 1e-12,
            "the corners read it too"
        );

        // A CURVED element under the same affine field.
        let mut bowed = rest.clone();
        bowed[4] += Vec3::new(0.0, -0.06, -0.04);
        bowed[9] += Vec3::new(0.05, 0.03, 0.03);
        let curved = ReadoutMesh {
            rest_positions: bowed.clone(),
            elements: ReadoutElements::Tet10(vec![[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]]),
            materials: vec![unit_tet_material()],
        };
        let curr_curved: Vec<Vec3> = bowed.iter().map(|p| a * p).collect();
        let dets = Tet10.rest_jacobian_dets(&SMatrix::<f64, 10, 3>::from_fn(|i, k| bowed[i][k]));
        let total: f64 = dets.iter().map(|d| d.abs()).sum();
        assert!(
            dets.iter()
                .any(|d| (d.abs() - 0.25 * total).abs() > 1e-3 * total),
            "the bowed element must weigh its points unequally, or the weight check is vacuous",
        );
        for (q, (point, det)) in curved
            .gauss_point_readouts(0, &curr_curved)
            .iter()
            .zip(dets)
            .enumerate()
        {
            assert!(
                (point.f - a).norm() < 1e-12,
                "curved element, point {q}: an affine field must read A, got {}",
                point.f,
            );
            assert!(
                (point.volume_fraction - det.abs() / total).abs() < 1e-15,
                "curved element, point {q}: weight {} is not |det J| / Σ = {}",
                point.volume_fraction,
                det.abs() / total,
            );
        }
    }

    /// ⭐ `ReadoutMesh::tet10` takes each element's midsides from the mesh, in
    /// the order `Tet10`'s shape functions expect — checked on every element
    /// of a real enriched mesh.
    ///
    /// The single-element gate above builds its connectivity by hand, so it
    /// cannot see the constructor get a midside's slot wrong. Here the
    /// tolerance fixture is enriched exactly as the bridge enriches it, a
    /// quadratic field is laid over every node, and `F` at every Gauss point
    /// of every element must be that field's analytic gradient at the point's
    /// physical location. A misplaced midside reads the wrong node's
    /// displacement and fails at once.
    ///
    /// It also pins that enrichment keeps every element's id, which the UI's
    /// Tet4-indexed heat-map lookup depends on.
    #[test]
    fn the_tet10_readout_mesh_places_every_elements_midsides() {
        let mesh4 = tolerance_fixture().mesh;
        let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&mesh4);
        let readout_mesh =
            ReadoutMesh::tet10(&mesh10).expect("an enriched mesh names its midsides");
        assert_eq!(readout_mesh.n_elements(), mesh10.n_tets());
        assert!(
            mesh10.n_tets() > 0,
            "an empty mesh would pass everything below"
        );

        // ⛔ Element `t` of the enriched mesh IS element `t` of the Tet4 mesh.
        // The UI indexes its heat-map centroids and its layer map by the Tet4
        // id and reads them against these readouts, so a reordering would
        // colour every element with another one's stress.
        let reordered = (0_u32..)
            .take(mesh4.n_tets())
            .filter(|&t| mesh10.tet_vertices(t) != mesh4.tet_vertices(t))
            .count();
        assert_eq!(
            reordered, 0,
            "enrichment must keep every element's id and corners"
        );

        // A quadratic in x sized to the mesh so F stays in [0.5, 1.5]:
        // u_x = k (x − x0)², F_xx = 1 + 2k (x − x0).
        let rest = mesh10.positions();
        let (lo, hi) = rest
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p.x), hi.max(p.x))
            });
        let x0 = 0.5 * (lo + hi);
        let k = 0.25 / (0.5 * (hi - lo));
        let curr: Vec<Vec3> = rest
            .iter()
            .map(|p| Vec3::new(p.x + k * (p.x - x0) * (p.x - x0), p.y, p.z))
            .collect();

        let mut worst = 0.0_f64;
        for (e, t) in (0..readout_mesh.n_elements()).zip(0_u32..) {
            // The point's physical x from the CORNERS alone — `from_tet4`
            // leaves every element straight, so the map is affine, and this
            // referent cannot share a midside-ordering bug with the readout.
            let [c0, c1, c2, c3] = mesh10.tet_vertices(t).map(|v| rest[v as usize].x);
            for (point, (xi, _)) in readout_mesh
                .gauss_point_readouts(e, &curr)
                .iter()
                .zip(Tet10.gauss_points())
            {
                let x = c0 * (1.0 - xi.x - xi.y - xi.z) + c1 * xi.x + c2 * xi.y + c3 * xi.z;
                let analytic =
                    Matrix3::from_diagonal(&Vec3::new(1.0 + 2.0 * k * (x - x0), 1.0, 1.0));
                worst = worst.max((point.f - analytic).norm());
            }
        }
        assert!(
            worst < 1e-9,
            "every Gauss point of every element must read the analytic gradient; \
             the worst was off by {worst:.3e}",
        );
    }

    /// What the corner readout got wrong on real bridge ramps — the size of
    /// the change the per-Gauss-point readout makes to the heat map.
    ///
    /// Both readouts are taken from the SAME converged state, the last step
    /// each ramp reaches: the per-Gauss-point one through the ramp's own
    /// [`InsertionRamp::readout_mesh`], and the corner one through a Tet4
    /// snapshot of the scene, which is what the UI read before. Only the
    /// readout differs, never the solve.
    ///
    /// It also runs the UI's outer-skin detection against both rest-position
    /// sources — the Tet4 snapshot the UI used to pass, which it now refuses,
    /// and the solved mesh — and counts the vertices no element names, which
    /// never move and so read as outer skin too.
    ///
    /// ⛔ Asserts nothing. It reports on whatever scenes are present, and the
    /// product scan is repo-excluded.
    #[test]
    #[ignore = "release-mode bridge ramps on two scenes; run with --ignored --nocapture"]
    fn what_the_corner_readout_missed_on_the_bridge() {
        let mut scenes: Vec<(&str, InsertionGeometry, usize)> =
            vec![("tolerance_fixture", tolerance_fixture(), 16)];
        if let Some((scan, _centerline, caps, design)) = product_scene() {
            let g = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004)
                .expect("the product geometry must build");
            // The product's best recorded operating point is 32 steps.
            scenes.push(("base_mold (product)", g, 32));
        }
        for (name, g, n_steps) in scenes {
            let corners = ReadoutMesh::tet4(&g.mesh);
            let ramp = run_insertion_ramp_tet10_ipc(g, n_steps, INSERTION_SOLVE_TOL)
                .expect("the bridge ramp must build");
            let Some(last) = ramp.steps.last() else {
                println!("\n══ {name}: no step converged ══");
                continue;
            };
            let pos = positions_from_flat(&last.x_final);
            let gp = ramp.readout_mesh.readouts(&pos);
            // The corner view reads the corner PREFIX of the enriched positions —
            // exactly what the UI's Tet4 snapshot used to read.
            let cr = corners.readouts(&pos[..corners.rest_positions.len()]);
            println!(
                "\n══ {name} · {}/{n_steps} steps · tol {INSERTION_SOLVE_TOL:e} · depth {:.3} mm \
                 · {} elements ══",
                ramp.steps.len(),
                last.interference_m * 1e3,
                gp.len(),
            );

            let peak = |r: &[TetReadout], f: fn(&TetReadout) -> f64| {
                r.iter().map(f).fold(f64::NEG_INFINITY, f64::max)
            };
            let least = |r: &[TetReadout], f: fn(&TetReadout) -> f64| {
                r.iter().map(f).fold(f64::INFINITY, f64::min)
            };
            // `len()` is an element count, far under f64's exact-integer
            // ceiling.
            #[allow(clippy::cast_precision_loss)]
            let mean = |r: &[TetReadout]| {
                r.iter().map(|t| t.energy_density_j_per_m3).sum::<f64>() / r.len() as f64
            };
            println!(
                "{:<26} {:>14} {:>14} {:>9}",
                "", "corner", "per-GP", "ratio"
            );
            for (label, c, g) in [
                (
                    "peak ‖P‖ (kPa)",
                    peak(&cr, |t| t.first_piola_frobenius_pa) * 1e-3,
                    peak(&gp, |t| t.first_piola_frobenius_pa) * 1e-3,
                ),
                (
                    "max stretch",
                    peak(&cr, |t| t.max_principal_stretch),
                    peak(&gp, |t| t.max_principal_stretch),
                ),
                (
                    "min stretch",
                    least(&cr, |t| t.min_principal_stretch),
                    least(&gp, |t| t.min_principal_stretch),
                ),
                ("mean Ψ (J/m³)", mean(&cr), mean(&gp)),
            ] {
                println!("{label:<26} {c:>14.5} {g:>14.5} {:>9.4}", g / c);
            }

            // Per element: how far the corner stress sits from the per-GP
            // peak, as a fraction of it — over the elements carrying at least
            // 1 % of the scene's per-GP peak, so an element at rounding-level
            // stress does not count as "off".
            let floor_pa = 0.01 * peak(&gp, |t| t.first_piola_frobenius_pa);
            let mut rel: Vec<f64> = cr
                .iter()
                .zip(&gp)
                .filter(|(_, g)| g.first_piola_frobenius_pa >= floor_pa)
                .map(|(c, g)| {
                    (c.first_piola_frobenius_pa - g.first_piola_frobenius_pa).abs()
                        / g.first_piola_frobenius_pa
                })
                .collect();
            rel.sort_by(f64::total_cmp);
            let q = |f: f64| {
                // A quantile index into a non-empty sorted vector.
                #[allow(
                    clippy::cast_possible_truncation,
                    clippy::cast_sign_loss,
                    clippy::cast_precision_loss
                )]
                let i = ((rel.len() - 1) as f64 * f).round() as usize;
                rel[i]
            };
            // Counts are far under f64's exact-integer ceiling.
            #[allow(clippy::cast_precision_loss)]
            let over_10pct = rel.iter().filter(|&&r| r > 0.10).count() as f64 / rel.len() as f64;
            println!(
                "per-element |Δ‖P‖| / per-GP ‖P‖, over the {} of {} elements at ≥ 1 % of the \
                 per-GP peak ({:.2} kPa):",
                rel.len(),
                gp.len(),
                floor_pa * 1e-3,
            );
            println!(
                "  median {:.4}  p90 {:.4}  p99 {:.4}  max {:.4}  · {:.2} % of them off by > 10 %",
                q(0.5),
                q(0.9),
                q(0.99),
                q(1.0),
                over_10pct * 100.0,
            );
            let argmax = |r: &[TetReadout]| {
                r.iter()
                    .enumerate()
                    .max_by(|a, b| {
                        a.1.first_piola_frobenius_pa
                            .total_cmp(&b.1.first_piola_frobenius_pa)
                    })
                    .map(|(i, _)| i)
            };
            println!(
                "hotspot element: corner {:?} · per-GP {:?}",
                argmax(&cr),
                argmax(&gp),
            );

            // The UI's outer-skin detection, from each rest-position source.
            let detected = |rest: &[Vec3]| {
                crate::insertion_sim_ui::detect_outer_skin_vertices(rest, &last.x_final)
                    .map_or_else(|e| format!("refused ({e})"), |s| s.len().to_string())
            };
            let mut named = vec![false; ramp.readout_mesh.rest_positions().len()];
            if let ReadoutElements::Tet10(elements) = &ramp.readout_mesh.elements {
                for e in elements {
                    for &v in e {
                        named[v as usize] = true;
                    }
                }
            }
            println!(
                "outer skin detected: from the Tet4 snapshot {} · from the solved mesh {} \
                 (Dirichlet-pinned {}; vertices no element names {})",
                detected(&corners.rest_positions),
                detected(ramp.readout_mesh.rest_positions()),
                ramp.n_pinned,
                named.iter().filter(|&&n| !n).count(),
            );
        }
    }

    /// The UI's own pipeline, end to end, with the bridge ticked and unticked —
    /// on the product scan at the panel's defaults.
    ///
    /// Everything else here calls the ramps and the readout functions
    /// directly. This drives `run_sim_pipeline`, the path the Simulate button
    /// runs. Both runs use the growing model (ticking the bridge forces it), the
    /// same corner mesh and the same outer-skin rule, so read the outer-face and
    /// cavity-face counts of one against the other.
    ///
    /// Per layer it also reports what the heat-map frame fix changed: at the
    /// last step, how many of the deformed view's drawn vertices take a
    /// different colour read at their rest positions than at their moved ones
    /// (the lookup before the fix), and the largest move among all drawn
    /// vertices.
    ///
    /// ⛔ Asserts nothing; the scan is repo-excluded.
    #[test]
    #[ignore = "needs the product scan + two release pipeline runs; run with --ignored --nocapture"]
    fn the_ui_pipeline_runs_the_bridge_end_to_end_on_the_product_scan() {
        use cf_device_types::SimMode;
        let Some((scan, centerline, caps, design)) = product_scene() else {
            return;
        };
        let cached =
            cf_device_geometry::sdf_layers::build_cached_scan_sdf(&scan, &caps, 0.005, 0.043)
                .expect("the product's cached SDF must build");
        for use_bridge in [true, false] {
            let started = std::time::Instant::now();
            let run = crate::insertion_sim_ui::run_sim_pipeline(
                scan.clone(),
                design.clone(),
                caps.clone(),
                cached.clone(),
                centerline.clone(),
                SimMode::GrowingIntruder,
                crate::insertion_sim_ui::DEFAULT_N_STEPS,
                use_bridge,
            );
            match run {
                Err(e) => println!("bridge {use_bridge}: ERROR {e:?}"),
                Ok(o) => {
                    let non_finite = o.per_step_scalar_fields.last().map_or(0, |last| {
                        last[0]
                            .iter()
                            .chain(&last[1])
                            .filter(|v| !v.is_finite())
                            .count()
                    });
                    println!(
                        "bridge {use_bridge}: {} steps · outer faces per layer {:?} · cavity faces {} \
                         · {} elements · non-finite last-step scalars {non_finite} · {:.0} s",
                        o.per_step_scalar_fields.len(),
                        o.per_layer_outer_faces
                            .iter()
                            .map(Vec::len)
                            .collect::<Vec<_>>(),
                        o.cavity_boundary_faces.len(),
                        o.tet_centroids.len(),
                        started.elapsed().as_secs_f64(),
                    );
                    let last = o.step_count().saturating_sub(1);
                    let rest = o.readout_mesh().rest_positions();
                    for layer in 0..o.per_layer_outer_faces.len() {
                        let Some(mesh) = o.deformed_layer_slab_mesh_at(layer, last) else {
                            continue;
                        };
                        let colour = |vertices| {
                            crate::insertion_sim_ui::project_layer_heat_map(
                                &o,
                                layer,
                                cf_device_types::ScalarMode::EnergyDensity,
                                last,
                                vertices,
                            )
                        };
                        let (Some(at_rest), Some(at_moved)) = (
                            colour(crate::insertion_sim_ui::LayerVertices::SimMesh),
                            colour(crate::insertion_sim_ui::LayerVertices::Rest(&mesh.vertices)),
                        ) else {
                            continue;
                        };
                        let mut drawn: Vec<usize> =
                            mesh.faces.iter().flatten().map(|&v| v as usize).collect();
                        drawn.sort_unstable();
                        drawn.dedup();
                        let changed = drawn.iter().filter(|&&v| at_rest[v] != at_moved[v]).count();
                        let max_move_m = drawn
                            .iter()
                            .map(|&v| (mesh.vertices[v].coords - rest[v]).norm())
                            .fold(0.0_f64, f64::max);
                        println!(
                            "    layer {layer}: the frame fix changes the colour of {changed} of \
                             {} drawn vertices · largest move among them {:.3} mm",
                            drawn.len(),
                            max_move_m * 1e3,
                        );
                    }
                }
            }
        }
    }

    /// ⭐⭐⭐ **THE DISCRIMINATING EXPERIMENT** — the bridge against the penalty
    /// baseline, across a ladder of residual tolerances.
    ///
    /// #958 measured that on `sock_over_capsule` (the scan every measurement
    /// before 2026-09-22 used) the full-depth result is bought with the
    /// tolerance: at the shipped `1e-1` that scan reaches
    /// 16/16, and at `1e-6` it stalls at step 4 — usable depth falls 4×. That,
    /// not depth and not friction, is what the bridge exists to fix, so this is
    /// the measurement that decides whether it did.
    ///
    /// ⚠ **A ladder, not a single tight run, because BOTH paths have a
    /// conditioning floor.** The penalty path's is #958's, above. The bridge's
    /// is `the_bridge_ramp_over_a_stiffness_sweep`: on `tolerance_fixture`,
    /// asked for `1e-6`, no `κ` from 1e6 to 1e9 completes the feasible-start
    /// approach — while asked for the shipped `1e-1`, every grid `κ` from
    /// 2.5e7 to 2.5e8 seats fully. A single tight run reports "both fail" and
    /// hides the only quantity that separates them, which is how much residual
    /// each path can be asked for before depth collapses.
    ///
    /// ⭐ **The payoff quantity is not depth alone** (#959): a ramp can reach
    /// full depth BY PENETRATING — the single-layer scan reaches 16/16 with
    /// `min_sd` −0.373 mm and its 5 % area tail at −0.042 mm, a region through
    /// the wall rather than an outlier. So all three are reported together:
    /// **depth, with `min_sd > 0` AND the 5 % area tail > 0**, at each
    /// tolerance. A depth reached through the wall is not a seat.
    ///
    /// ⛔ **Asserts nothing.** Where a ramp stalls is platform-dependent
    /// (#958's first push went red on exactly that), so this reports on a named
    /// platform and never gates.
    #[test]
    #[ignore = "release-mode ramps on both arms across a tolerance ladder; run with --ignored --nocapture"]
    fn the_bridge_against_the_penalty_baseline_across_a_tolerance_ladder() {
        const N_STEPS: usize = 16;
        /// Residual tolerances asked for, loosest first. `1e-1` is what ships.
        const TOLERANCES: [f64; 4] = [1e-1, 1e-2, 1e-3, 1e-4];

        // The sphere is where #959 read σ before it was re-measured on
        // `base_mold`. It stays as a larger synthetic scene beside the small
        // conditioning stand-in — not as the source of the shipped σ.
        fn sphere_scene() -> InsertionGeometry {
            let design = SimDesign {
                cavity_inset_m: 0.003,
                layers: vec![layer(0.010, "ECOFLEX_00_30")],
            };
            build_insertion_geometry(&icosphere(0.040, 3), &design, &[], 2_000, 0.004)
                .expect("the synthetic-sphere geometry must build")
        }

        for (scene, build) in [
            (
                "tol-fixture",
                tolerance_fixture as fn() -> InsertionGeometry,
            ),
            ("sphere-40mm", sphere_scene as fn() -> InsertionGeometry),
        ] {
            println!("\n══ {scene} ══");
            println!(
                "tol        arm                 steps  depth_mm  resid      pairs  \
                 min_sd_mm  tail5_mm  sigma_kPa"
            );
            for tol in TOLERANCES {
                // ── Tet4 + penalty, the baseline, at this tolerance.
                let g = build();
                let mesh4 = g.mesh.clone();
                let intruder = g.intruder.clone();
                let bounds = g.bounds;
                let cavity_offset_m = g.cavity_offset_m;
                let referenced4: Vec<VertexId> = referenced_vertices(&mesh4);
                let rest_areas4 = boundary_vertex_areas(
                    Mesh::<Yeoh>::positions(&mesh4),
                    Mesh::<Yeoh>::boundary_faces(&mesh4),
                );
                let base =
                    run_insertion_ramp_at_kappa_and_tol(g, N_STEPS, INSERTION_CONTACT_KAPPA, tol)
                        .expect("the penalty baseline ramp must build");
                let base_stats = base.steps.last().map(|last| {
                    let pos = positions_from_flat(&last.x_final);
                    let contact = intruder_contact_at_kappa(
                        &intruder,
                        bounds,
                        last.interference_m,
                        cavity_offset_m,
                        INSERTION_CONTACT_KAPPA,
                    );
                    let raw = contact.per_pair_readout(&mesh4, &pos);
                    let readouts = filter_pair_readouts_to_referenced(raw, &referenced4);
                    (
                        last.interference_m,
                        last.final_residual_norm,
                        patch_stats(&readouts, &rest_areas4),
                    )
                });
                let (d, r, st) = base_stats.unwrap_or((0.0, f64::NAN, None));
                print_arm_row(
                    &format!("{tol:.0e}"),
                    "tet4+penalty",
                    base.steps.len(),
                    N_STEPS,
                    d,
                    r,
                    st,
                );

                // ── Tet10 + IPC, the bridge, at the same tolerance.
                let g = build();
                let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
                let referenced10: Vec<VertexId> = referenced_vertices(&mesh10);
                let rest_areas10 = boundary_vertex_areas(
                    Mesh::<Yeoh>::positions(&mesh10),
                    Mesh::<Yeoh>::boundary_faces(&mesh10),
                );
                let inset_m = -g.cavity_offset_m;
                // `N_STEPS` is small; the cast is exact.
                #[allow(clippy::cast_precision_loss)]
                let kappa =
                    bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, inset_m / N_STEPS as f64)
                        .expect("the bridge's stiffness bracket must be non-empty");
                let bridge = run_insertion_ramp_tet10_ipc(g, N_STEPS, tol)
                    .expect("the bridge ramp must build");
                let bridge_stats = bridge.steps.last().map(|last| {
                    let pos = positions_from_flat(&last.x_final);
                    let contact = intruder_ipc_contact_at(
                        &intruder,
                        bounds,
                        last.interference_m,
                        cavity_offset_m,
                        kappa,
                        BRIDGE_CONTACT_DHAT_M,
                    );
                    let raw = contact.per_pair_readout(&mesh10, &pos);
                    let readouts = filter_pair_readouts_to_referenced(raw, &referenced10);
                    (
                        last.interference_m,
                        last.final_residual_norm,
                        patch_stats(&readouts, &rest_areas10),
                    )
                });
                let (d, r, st) = bridge_stats.unwrap_or((0.0, f64::NAN, None));
                print_arm_row(
                    &format!("{tol:.0e}"),
                    "tet10+ipc",
                    bridge.steps.len(),
                    N_STEPS,
                    d,
                    r,
                    st,
                );
                if let Some(reason) = bridge
                    .failure_reason
                    .as_deref()
                    .filter(|r| r.contains("approach"))
                {
                    println!("            (bridge never began: {reason})");
                }
            }
        }

        println!(
            "\n\u{26a0} \u{3c3} is on the DEFORMED area basis. The REST basis is not \
             comparable across these arms: on the face path the loaded nodes are \
             MIDSIDES, which lie on no 3-node boundary face and so carry no \
             `boundary_vertex_areas` rest tributary."
        );
    }

    /// One row of
    /// [`the_bridge_against_the_penalty_baseline_across_a_tolerance_ladder`].
    fn print_arm_row(
        scene: &str,
        arm: &str,
        converged: usize,
        n_steps: usize,
        interference_m: f64,
        residual: f64,
        stats: Option<PatchStats>,
    ) {
        let steps = format!("{converged}/{n_steps}");
        stats.map_or_else(
            || {
                println!(
                    "{scene:<20} {arm:<18} {steps:>6}  {:>8.3}  {residual:>9.2e}  \
                     (no contact patch)",
                    interference_m * 1e3,
                );
            },
            |s| {
                println!(
                    "{scene:<20} {arm:<18} {steps:>6}  {:>8.3}  {residual:>9.2e}  \
                     {:>5}  {:>9.4}  {:>8.4}  {:>9.2}",
                    interference_m * 1e3,
                    s.n_pairs,
                    s.min_sd_m * 1e3,
                    s.p05_sd_m * 1e3,
                    s.traction_pa * 1e-3,
                );
            },
        );
    }

    /// How far the enriched midsides sit under the curved cavity surface.
    ///
    /// `Tet10Mesh::from_tet4` places every midside at the straight-edge
    /// MIDPOINT. On a curved cavity that puts each boundary midside under the
    /// true surface by the sagitta (~`h²/8R`), so the barrier sees a gap that
    /// is not the geometry's gap — and the tightest node on the patch is an
    /// artifact of enrichment rather than of the scene.
    ///
    /// This measures the offset for corners and midsides separately, against
    /// the same cavity isosurface the intruder is built from. Corners are the
    /// control: they come straight from the Tet4 mesh, so whatever bias they
    /// show is the mesher's, and only the EXCESS on midsides is enrichment's.
    ///
    /// ⛔ Asserts nothing — it is a diagnostic.
    #[test]
    #[ignore = "diagnostic; run with --ignored --nocapture"]
    fn the_enriched_midsides_sit_under_the_curved_cavity_surface() {
        let g = tolerance_fixture();
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        let n_corners = tet10.n_corners();
        let cavity = Solid::from_sdf(g.intruder.clone(), g.bounds).offset(g.cavity_offset_m);
        let positions = Mesh::<Yeoh>::positions(&tet10);

        // Only nodes ON the cavity surface matter; take everything within one
        // barrier band of it so the sample is the patch the contact will see.
        let mut corner_sd: Vec<f64> = Vec::new();
        let mut midside_sd: Vec<f64> = Vec::new();
        for (vid, p) in positions.iter().enumerate() {
            let sd = cavity.eval(Point3::from(*p));
            if sd.abs() > BRIDGE_CONTACT_DHAT_M {
                continue;
            }
            if vid < n_corners {
                corner_sd.push(sd);
            } else {
                midside_sd.push(sd);
            }
        }

        let summarise = |label: &str, v: &mut Vec<f64>| {
            if v.is_empty() {
                println!("{label:<10} (none within one band)");
                return f64::NAN;
            }
            v.sort_unstable_by(f64::total_cmp);
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            println!(
                "{label:<10} n {:>5}   min {:>9.4} mm   mean {:>9.4} mm   max {:>9.4} mm",
                v.len(),
                v[0] * 1e3,
                mean * 1e3,
                v[v.len() - 1] * 1e3,
            );
            mean
        };

        println!("\nsigned distance to the cavity isosurface, at REST:");
        let c_mean = summarise("corners", &mut corner_sd);
        let m_mean = summarise("midsides", &mut midside_sd);
        println!(
            "\nenrichment's excess (midside mean - corner mean): {:.4} mm",
            (m_mean - c_mean) * 1e3,
        );
        println!(
            "for scale: one ramp increment at 16 steps is {:.4} mm, and d_hat/2 is {:.4} mm",
            -g.cavity_offset_m / 16.0 * 1e3,
            0.5 * BRIDGE_CONTACT_DHAT_M * 1e3,
        );

        // ⛔ What the ramp's approach schedule is actually derived from — over
        // EVERY vertex, which is the population the ramp itself scans.
        let worst_all = positions
            .iter()
            .map(|p| cavity.eval(Point3::from(*p)))
            .filter(|sd| sd.is_finite())
            .fold(0.0_f64, |acc, sd| acc.max(-sd));
        let referenced: BTreeSet<VertexId> = referenced_vertices(&tet10).into_iter().collect();
        let worst_referenced = positions
            .iter()
            .enumerate()
            .filter(|(vid, _)| u32::try_from(*vid).is_ok_and(|v| referenced.contains(&v)))
            .map(|(_, p)| cavity.eval(Point3::from(*p)))
            .filter(|sd| sd.is_finite())
            .fold(0.0_f64, |acc, sd| acc.max(-sd));
        let step = -g.cavity_offset_m / 16.0;
        println!(
            "\nworst rest penetration over ALL vertices        {:>9.4} mm               => {:>5} approach steps",
            worst_all * 1e3,
            ((worst_all + 0.5 * BRIDGE_CONTACT_DHAT_M) / step).ceil(),
        );
        println!(
            "worst rest penetration over REFERENCED vertices {:>9.4} mm               => {:>5} approach steps",
            worst_referenced * 1e3,
            ((worst_referenced + 0.5 * BRIDGE_CONTACT_DHAT_M) / step).ceil(),
        );
    }

    /// The bridge ramp across a stiffness sweep, at the tight AND the shipped
    /// tolerance.
    ///
    /// The derivation claims `κ` is determined rather than swept. That claim is
    /// only worth something if the neighbouring stiffnesses can be run, so this
    /// runs them: a log grid over 1e6–1e9, five per decade, plus the bracket's
    /// floor and the shipped value.
    ///
    /// ⚠ **Both tolerances, because they ask different questions.** At
    /// [`TIGHT_TOL`] the question is whether any `κ` gets past the conditioning
    /// floor; at [`INSERTION_SOLVE_TOL`] — what ships — it is which `κ` seats.
    /// A sweep at one tolerance answers only its own. A seat is depth with
    /// `min_sd` AND the 5 % tail above zero, never depth alone (#959).
    ///
    /// The shipped value IS the ceiling since the κ fix, so there is no
    /// separate CEILING arm. That the two coincide is gated in
    /// `the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling`.
    ///
    /// ⛔ Asserts nothing — where a ramp stalls is platform-dependent.
    #[test]
    #[ignore = "release-mode ramps across a stiffness sweep at two tolerances; run with --ignored --nocapture"]
    fn the_bridge_ramp_over_a_stiffness_sweep() {
        const N_STEPS: usize = 16;
        // ⛔ The schedule must come from the fixture this sweep actually runs.
        // A hardcoded product inset derives a FLOOR for a march that never
        // happens here — the approach steps at `-cavity_offset_m / N_STEPS`,
        // and [`tolerance_fixture`] is a 3 mm inset, not the product's 5 mm.
        // `N_STEPS` is small; the cast is exact.
        #[allow(clippy::cast_precision_loss)]
        let step = -tolerance_fixture().cavity_offset_m / N_STEPS as f64;
        let shipped = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, step)
            .expect("the bracket must be non-empty");
        let floor = face_barrier_kappa(
            BRIDGE_CONTACT_DHAT_M,
            BRIDGE_PATCH_NONUNIFORMITY * step,
            BRIDGE_DESIGN_TRACTION_PA,
        )
        .expect("floor");

        // Five per decade over 1e6..=1e9, then the two named values, in order.
        let mut arms: Vec<(f64, &str)> = (0..=15)
            .map(|i| (10f64.powf(6.0 + f64::from(i) / 5.0), ""))
            .collect();
        arms.extend([(floor, "FLOOR"), (shipped, "SHIPPED = CEILING")]);
        arms.sort_by(|a, b| a.0.total_cmp(&b.0));

        println!(
            "\nfloor {floor:.4e}, shipped {shipped:.4e}, d_hat {BRIDGE_CONTACT_DHAT_M:.2e} m, \
             step {:.4} mm",
            step * 1e3,
        );
        for tol in [TIGHT_TOL, INSERTION_SOLVE_TOL] {
            println!("\n══ asked for {tol:e} ══");
            println!(
                "{:<20} {:<18} {:>6}  {:>8}  {:>9}  {:>5}  {:>9}  {:>8}  {:>9}",
                "kappa",
                "label",
                "steps",
                "depth_mm",
                "resid",
                "pairs",
                "min_sd_mm",
                "tail5_mm",
                "sigma_kPa",
            );
            for &(kappa, label) in &arms {
                let kappa_label = format!("{kappa:.3e}");
                let g = tolerance_fixture();
                let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
                let referenced: Vec<VertexId> = referenced_vertices(&mesh10);
                let rest_areas = boundary_vertex_areas(
                    Mesh::<Yeoh>::positions(&mesh10),
                    Mesh::<Yeoh>::boundary_faces(&mesh10),
                );
                let intruder = g.intruder.clone();
                let bounds = g.bounds;
                let cavity_offset_m = g.cavity_offset_m;
                let ramp =
                    run_insertion_ramp_tet10_ipc_at(g, N_STEPS, tol, kappa, BRIDGE_CONTACT_DHAT_M)
                        .expect("the bridge ramp must build");
                if let Some(last) = ramp.steps.last() {
                    let pos = positions_from_flat(&last.x_final);
                    let contact = intruder_ipc_contact_at(
                        &intruder,
                        bounds,
                        last.interference_m,
                        cavity_offset_m,
                        kappa,
                        BRIDGE_CONTACT_DHAT_M,
                    );
                    let raw = contact.per_pair_readout(&mesh10, &pos);
                    let readouts = filter_pair_readouts_to_referenced(raw, &referenced);
                    print_arm_row(
                        &kappa_label,
                        label,
                        ramp.steps.len(),
                        N_STEPS,
                        last.interference_m,
                        last.final_residual_norm,
                        patch_stats(&readouts, &rest_areas),
                    );
                } else {
                    let steps = format!("0/{N_STEPS}");
                    println!("{kappa_label:<20} {label:<18} {steps:>6}  (no step converged)");
                }
                if let Some(k) = ramp.failed_at_step {
                    // The solver's closing hint is the same on every row; drop it.
                    let why = ramp.failure_reason.as_deref().unwrap_or("?");
                    let why = why
                        .split_once(" Likely causes")
                        .map_or(why, |(head, _)| head);
                    println!("    stalled at recorded step {k}: {why}");
                }
            }
        }
    }

    /// The bridge across a stiffness sweep **on the product scan** — the
    /// margin the shipped `κ` has where it actually runs.
    ///
    /// [`the_bridge_ramp_over_a_stiffness_sweep`] measured the band that seats
    /// at the shipped tolerance on [`tolerance_fixture`], a 3 mm Ecoflex inset,
    /// with the shipped value inside it. That band belongs to that fixture.
    /// This runs a subset of the same grid points on `base_mold` at its best recorded
    /// operating point — 32 steps, [`INSERTION_SOLVE_TOL`] — so the product's
    /// margin is measured rather than inherited.
    ///
    /// ⚠ **Read the stop, not just the depth.** The recon records the shipped
    /// `κ` stopping at 4.531 mm of the 5 mm inset on element inversion, so no
    /// arm is expected to reach full depth. An arm that stops on a contact
    /// stall and one that stops on inversion are different answers, and a
    /// seat still needs `min_sd` AND the 5 % tail above zero (#959).
    ///
    /// ⛔ The scan is repo-excluded, so nothing here can gate; it asserts
    /// nothing.
    #[test]
    #[ignore = "needs the product scan + ~12 release ramps; run with --ignored --nocapture"]
    fn the_bridge_ramp_over_a_stiffness_sweep_on_the_product_scan() {
        const N_STEPS: usize = 32;
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        let build = || {
            build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004)
                .expect("the product geometry must build")
        };
        // The schedule comes from the geometry the ramp runs, exactly as the
        // ramp derives it — not from the design's nominal inset.
        // `N_STEPS` is small; the cast is exact.
        #[allow(clippy::cast_precision_loss)]
        let step = -build().cavity_offset_m / N_STEPS as f64;
        let shipped = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, step)
            .expect("the bracket must be non-empty");

        // Five per decade from 10^6.6 to 10^8.6 — a subset of the fixture
        // sweep's grid, so the κ values the two share compare directly.
        let mut arms: Vec<(f64, &str)> = (3..=13)
            .map(|i| (10f64.powf(6.0 + f64::from(i) / 5.0), ""))
            .collect();
        arms.push((shipped, "SHIPPED"));
        arms.sort_by(|a, b| a.0.total_cmp(&b.0));

        println!(
            "\n══ base_mold · inset {:.1} mm · {N_STEPS} steps of {:.4} mm · asked for \
             {INSERTION_SOLVE_TOL:e} · shipped {shipped:.4e} ══",
            design.cavity_inset_m * 1e3,
            step * 1e3,
        );
        println!(
            "{:<20} {:<18} {:>6}  {:>8}  {:>9}  {:>5}  {:>9}  {:>8}  {:>9}",
            "kappa",
            "label",
            "steps",
            "depth_mm",
            "resid",
            "pairs",
            "min_sd_mm",
            "tail5_mm",
            "sigma_kPa",
        );
        for &(kappa, label) in &arms {
            let kappa_label = format!("{kappa:.3e}");
            let g = build();
            let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
            let referenced: Vec<VertexId> = referenced_vertices(&mesh10);
            let rest_areas = boundary_vertex_areas(
                Mesh::<Yeoh>::positions(&mesh10),
                Mesh::<Yeoh>::boundary_faces(&mesh10),
            );
            let intruder = g.intruder.clone();
            let bounds = g.bounds;
            let cavity_offset_m = g.cavity_offset_m;
            let ramp = run_insertion_ramp_tet10_ipc_at(
                g,
                N_STEPS,
                INSERTION_SOLVE_TOL,
                kappa,
                BRIDGE_CONTACT_DHAT_M,
            )
            .expect("the bridge ramp must build");
            if let Some(last) = ramp.steps.last() {
                let pos = positions_from_flat(&last.x_final);
                let contact = intruder_ipc_contact_at(
                    &intruder,
                    bounds,
                    last.interference_m,
                    cavity_offset_m,
                    kappa,
                    BRIDGE_CONTACT_DHAT_M,
                );
                let raw = contact.per_pair_readout(&mesh10, &pos);
                let readouts = filter_pair_readouts_to_referenced(raw, &referenced);
                print_arm_row(
                    &kappa_label,
                    label,
                    ramp.steps.len(),
                    N_STEPS,
                    last.interference_m,
                    last.final_residual_norm,
                    patch_stats(&readouts, &rest_areas),
                );
            } else {
                let steps = format!("0/{N_STEPS}");
                println!("{kappa_label:<20} {label:<18} {steps:>6}  (no step converged)");
            }
            if let Some(k) = ramp.failed_at_step {
                // The solver's closing hint is the same on every row; drop it.
                let why = ramp.failure_reason.as_deref().unwrap_or("?");
                let why = why
                    .split_once(" Likely causes")
                    .map_or(why, |(head, _)| head);
                println!("    stopped at recorded step {k}: {why}");
            }
        }
    }

    /// The product scene as the sliding probes read it: the geometry, its
    /// centerline, and the nodes of the enriched mesh's boundary faces —
    /// collected exactly as [`run_sliding_insertion_ramp_tet10_ipc`] collects
    /// its wall points. `boundary_faces` are corner triangles, so these are
    /// corner nodes only; the midsides the face barrier loads are not in it.
    fn sliding_product_scene() -> Option<(
        InsertionGeometry,
        Vec<Point3<f64>>,
        Vec<VertexId>,
        Vec<Vec3>,
    )> {
        let (scan, centerline, caps, design) = product_scene()?;
        let g = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004)
            .expect("the product geometry must build");
        let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        let rest = Mesh::<Yeoh>::positions(&mesh10);
        let mut ids: Vec<VertexId> = Mesh::<Yeoh>::boundary_faces(&mesh10)
            .iter()
            .flatten()
            .copied()
            .collect();
        ids.sort_unstable();
        ids.dedup();
        let boundary = ids.iter().map(|&v| rest[v as usize]).collect();
        Some((g, centerline, ids, boundary))
    }

    /// What the SLIDING bridge's contact reaches on the product scan — no solve.
    ///
    /// [`run_sliding_insertion_ramp_tet10_ipc`] had no caller, so it had never
    /// run. Its contact is the moving scan offset by `cavity_offset_m` alone
    /// ([`intruder_ipc_contact_sliding_at`]), and at `t = 1` the pose is the
    /// identity ([`slide_pose_at`]) — so at full seat the contact is the
    /// cavity's own surface. The growing bridge instead ends at an offset of
    /// `interference + cavity_offset = 0`, the bare scan. This measures what
    /// that difference does on this scene, for both offsets, over `N_POSES`
    /// poses:
    /// - **room** — the signed distance from the contact to the UNDEFORMED
    ///   cavity-wall nodes; negative is room the silicone must make by
    ///   stretching. The most negative over the whole travel, and the minimum
    ///   and median at `t = 1`. For the most negative it reports the pose and
    ///   where the node sits: its arc distance from the SEATED tip
    ///   (`centerline[0]`), and its distance off the centerline.
    /// - **closing** — the ramp's own schedule measure,
    ///   [`sliding_normal_increment_m`], on five schedules, then one step count
    ///   at a time up to the first the ramp accepts. Beside it, the nodes that
    ///   measure skips because they cross the rest wall within one step: how
    ///   many start the step beyond `d̂`, where the barrier cannot see them, and
    ///   the largest starting gap.
    ///
    /// ⛔ The scan is repo-excluded, so nothing here can gate; it asserts
    /// nothing.
    #[test]
    #[ignore = "needs the product scan; run with --ignored --nocapture"]
    fn what_the_sliding_contact_reaches_on_the_product_scan() {
        const N_POSES: usize = 64;
        let Some((g, centerline, _, boundary)) = sliding_product_scene() else {
            return;
        };
        // The cavity wall — not the outer skin 17 mm out: boundary nodes within
        // one cell of the rest cavity surface.
        let cavity = Solid::from_sdf(g.intruder.clone(), g.bounds).offset(g.cavity_offset_m);
        let wall: Vec<Vec3> = boundary
            .iter()
            .copied()
            .filter(|p| cavity.eval(Point3::from(*p)).abs() < g.cell_size_m)
            .collect();
        let arc_m = polyline_arc_length_m(&centerline);
        println!(
            "\n══ base_mold · inset {:.1} mm · arc {:.2} mm · {} boundary corner nodes, {} on \
             the cavity wall · d_hat {:.2} mm ══",
            -g.cavity_offset_m * 1e3,
            arc_m * 1e3,
            boundary.len(),
            wall.len(),
            BRIDGE_CONTACT_DHAT_M * 1e3,
        );
        // Arc distance from the SEATED tip (`centerline[0]`) of the closest
        // centerline point, and the
        // distance to it.
        let along_centerline = |p: Vec3| {
            let (mut best_arc, mut best_d, mut walked) = (0.0, f64::INFINITY, 0.0);
            for seg in centerline.windows(2) {
                let (a, b) = (seg[0].coords, seg[1].coords);
                let len = (b - a).norm();
                let s = if len > 0.0 {
                    ((p - a).dot(&(b - a)) / (len * len)).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let d = (p - (a + (b - a) * s)).norm();
                if d < best_d {
                    (best_arc, best_d) = (walked + s * len, d);
                }
                walked += len;
            }
            (best_arc, best_d)
        };

        let contact_at = |t: f64, offset_m: f64| {
            Solid::from_sdf(
                TransformedSdf::new(g.intruder.clone(), slide_pose_at(&centerline, t)),
                g.bounds,
            )
            .offset(offset_m)
        };
        let offsets = [
            ("as written (cavity_offset)", g.cavity_offset_m),
            ("bare scan (offset 0)", 0.0),
        ];

        println!(
            "\n── contact vs the UNDEFORMED cavity wall (mm; negative = room the wall must make) ──"
        );
        println!(
            "{:<28} {:>14} {:>10} {:>11} {:>12} {:>14}",
            "contact",
            "min over travel",
            "min at t=1",
            "median t=1",
            "nodes <0 t=1",
            "ever < -d_hat",
        );
        for &(label, offset_m) in &offsets {
            let mut deepest = (f64::INFINITY, 0.0, Vec3::zeros());
            let mut ever = vec![f64::INFINITY; wall.len()];
            for k in 1..=N_POSES {
                // `k` and `N_POSES` are tiny; the casts are exact.
                #[allow(clippy::cast_precision_loss)]
                let t = k as f64 / N_POSES as f64;
                let solid = contact_at(t, offset_m);
                for (p, lowest) in wall.iter().zip(ever.iter_mut()) {
                    let sd = solid.eval(Point3::from(*p));
                    *lowest = lowest.min(sd);
                    if sd < deepest.0 {
                        deepest = (sd, t, *p);
                    }
                }
            }
            let seat = contact_at(1.0, offset_m);
            let mut at_seat: Vec<f64> = wall.iter().map(|p| seat.eval(Point3::from(*p))).collect();
            at_seat.sort_by(f64::total_cmp);
            let n_inside = at_seat.iter().filter(|&&sd| sd < 0.0).count();
            let n_ever = ever
                .iter()
                .filter(|&&sd| sd < -BRIDGE_CONTACT_DHAT_M)
                .count();
            println!(
                "{label:<28} {:>14.4} {:>10.4} {:>11.4} {:>7}/{:<5} {:>9}/{:<5}",
                deepest.0 * 1e3,
                at_seat.first().copied().unwrap_or(f64::NAN) * 1e3,
                at_seat[at_seat.len() / 2] * 1e3,
                n_inside,
                wall.len(),
                n_ever,
                wall.len(),
            );
            let (arc_from_tip, off_axis) = along_centerline(deepest.2);
            println!(
                "{:<28} most room at t = {:.4} (moving tip {:.2} mm in) · node {:.2} mm from \
                 the seated tip along the centerline, {:.2} mm off it",
                "",
                deepest.1,
                deepest.1 * arc_m * 1e3,
                arc_from_tip * 1e3,
                off_axis * 1e3,
            );
        }

        println!(
            "\n── worst normal closing per step (mm) · κ derivable = closing < d_hat {:.2} mm ──",
            BRIDGE_CONTACT_DHAT_M * 1e3,
        );
        println!(
            "{:<28} {:>5} {:>9} {:>9} {:>11} {:>16} {:>13}",
            "contact",
            "steps",
            "arc step",
            "closing",
            "κ derivable",
            "crossers > d_hat",
            "worst start",
        );
        // The ramp's closing measure, and what it cannot see: nodes that cross
        // the rest wall within one step. How many of those START the step
        // beyond `d̂`, where the barrier cannot see them, and the largest start.
        let closing = |n: usize, offset_m: f64| {
            sliding_normal_increment_m(
                &g.intruder,
                g.bounds,
                &centerline,
                n,
                offset_m,
                BRIDGE_CONTACT_DHAT_M,
                &boundary,
            )
        };
        let crossers = |n: usize, offset_m: f64| {
            let (mut beyond, mut worst_start) = (0_usize, 0.0_f64);
            for k in 0..n {
                // Step indices are tiny; the casts are exact.
                #[allow(clippy::cast_precision_loss)]
                let (t0, t1) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
                let (a, b) = (contact_at(t0, offset_m), contact_at(t1, offset_m));
                for p in &boundary {
                    let pt = Point3::from(*p);
                    let (sd_before, sd_after) = (a.eval(pt), b.eval(pt));
                    if sd_before.is_finite()
                        && sd_after.is_finite()
                        && sd_before > 0.0
                        && sd_after <= 0.0
                    {
                        worst_start = worst_start.max(sd_before);
                        if sd_before > BRIDGE_CONTACT_DHAT_M {
                            beyond += 1;
                        }
                    }
                }
            }
            (beyond, worst_start)
        };
        let derivable = |c: f64| bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, c).is_ok();
        for &(label, offset_m) in &offsets {
            let grid = [16, 32, 64, 128, 256];
            let mut accepted = Vec::new();
            for n in grid {
                let c = closing(n, offset_m);
                let (beyond, worst_start) = crossers(n, offset_m);
                // `n` is tiny; the cast is exact.
                #[allow(clippy::cast_precision_loss)]
                let arc_step = arc_m / n as f64;
                if derivable(c) {
                    accepted.push(n);
                }
                println!(
                    "{label:<28} {n:>5} {:>9.4} {:>9.4} {:>11} {beyond:>16} {:>13.4}",
                    arc_step * 1e3,
                    c * 1e3,
                    if derivable(c) { "yes" } else { "no" },
                    worst_start * 1e3,
                );
            }
            // Between the last refused grid schedule and the first accepted one,
            // one step count at a time: the closing is not monotone in `n`.
            let Some(&hi) = accepted.first() else {
                continue;
            };
            let lo = grid.iter().copied().filter(|&n| n < hi).max().unwrap_or(0) + 1;
            if let Some(n) = (lo..=hi).find(|&n| derivable(closing(n, offset_m))) {
                let (beyond, worst_start) = crossers(n, offset_m);
                println!(
                    "{label:<28} first accepted between {lo} and {hi}: {n} steps · closing {:.4} mm \
                     · crossers > d_hat {beyond} · worst start {:.4} mm",
                    closing(n, offset_m) * 1e3,
                    worst_start * 1e3,
                );
            }
        }
    }

    /// The SLIDING bridge as written, run on the product scan.
    ///
    /// Companion to `what_the_sliding_contact_reaches_on_the_product_scan`,
    /// which is why this is not the product answer: as written the contact ends
    /// at the cavity surface, with none of the inset's interference. What this
    /// does answer is whether the travelling face-barrier solve runs on this
    /// scene at all. It runs the first of 16, 32, 64, 128 and 256 steps that the
    /// ramp's own derivation accepts — chosen before building, from the same
    /// closing measure, so the geometry is built once — then reports every
    /// converged step and, at the last, the smallest gap from the deformed
    /// boundary's CORNER nodes to the contact. Midsides are not checked.
    ///
    /// ⛔ The scan is repo-excluded, so nothing here can gate; it asserts
    /// nothing.
    #[test]
    #[ignore = "needs the product scan + one long sliding ramp; run with --ignored --nocapture"]
    fn the_sliding_bridge_as_written_on_the_product_scan() {
        let Some((g, centerline, boundary_ids, boundary)) = sliding_product_scene() else {
            return;
        };
        let (intruder, bounds, cavity_offset_m) = (g.intruder.clone(), g.bounds, g.cavity_offset_m);
        let accepted = [16, 32, 64, 128, 256].into_iter().find(|&n| {
            let closing = sliding_normal_increment_m(
                &intruder,
                bounds,
                &centerline,
                n,
                cavity_offset_m,
                BRIDGE_CONTACT_DHAT_M,
                &boundary,
            );
            let ok = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, closing).is_ok();
            if !ok {
                println!(
                    "{n:>4} steps: closing {:.4} mm — the ramp would refuse",
                    closing * 1e3
                );
            }
            ok
        });
        let Some(n) = accepted else {
            println!("no schedule up to 256 steps is accepted");
            return;
        };
        let started = Instant::now();
        let ramp = run_sliding_insertion_ramp_tet10_ipc(g, &centerline, n, INSERTION_SOLVE_TOL)
            .expect("the schedule was chosen by the ramp's own closing measure");
        println!(
            "{n:>4} steps: {}/{n} converged in {:.0} s · {} pinned",
            ramp.steps.len(),
            started.elapsed().as_secs_f64(),
            ramp.n_pinned,
        );
        println!(
            "     {:>6} {:>8} {:>5} {:>9} {:>9} {:>8} {:>8} {:>9}",
            "t", "arc_mm", "iters", "resid", "force_N", "min_str", "max_str", "pairs",
        );
        for s in &ramp.steps {
            println!(
                "     {:>6.4} {:>8.2} {:>5} {:>9.2e} {:>9.3} {:>8.4} {:>8.4} {:>9}",
                s.slide_fraction_t,
                s.arc_length_s_m * 1e3,
                s.iter_count,
                s.final_residual_norm,
                s.readout.contact_force_magnitude_n,
                s.readout.min_principal_stretch,
                s.readout.max_principal_stretch,
                s.readout.n_active_contact_pairs,
            );
        }
        if let (Some(last), Some(&pose)) = (ramp.steps.last(), ramp.intruder_poses.last()) {
            let solid = Solid::from_sdf(TransformedSdf::new(intruder, pose), bounds)
                .offset(cavity_offset_m);
            let deformed = positions_from_flat(&last.x_final);
            let sds: Vec<f64> = boundary_ids
                .iter()
                .map(|&v| solid.eval(Point3::from(deformed[v as usize])))
                .collect();
            let min_sd = sds.iter().copied().fold(f64::INFINITY, f64::min);
            let n_through = sds.iter().filter(|&&sd| sd < 0.0).count();
            println!(
                "     last converged step: min gap to the contact {:.4} mm · {n_through} \
                 boundary corner nodes through it (midsides not checked)",
                min_sd * 1e3,
            );
        }
        if let Some(k) = ramp.failed_at_step {
            let why = ramp.failure_reason.as_deref().unwrap_or("?");
            let why = why
                .split_once(" Likely causes")
                .map_or(why, |(head, _)| head);
            println!("     stopped at recorded step {k}: {why}");
        }
    }

    /// The shipped ramp entry points solve at the shipped tolerance.
    ///
    /// The inertness half of the two-gate rule for
    /// [`run_insertion_ramp_at_kappa_and_tol`]: adding a knob must not move
    /// what ships. Every reported column of every step is compared, not just
    /// the step count — a delegation that changed the answer while keeping the
    /// shape would pass a count check.
    #[test]
    fn the_shipped_ramp_solves_at_the_shipped_tolerance() {
        let n_steps = 2;
        let shipped =
            run_insertion_ramp_at_kappa(tolerance_fixture(), n_steps, INSERTION_CONTACT_KAPPA)
                .expect("the shipped ramp must run on the tolerance fixture");
        let delegated = run_insertion_ramp_at_kappa_and_tol(
            tolerance_fixture(),
            n_steps,
            INSERTION_CONTACT_KAPPA,
            INSERTION_SOLVE_TOL,
        )
        .expect("the delegated ramp must run on the tolerance fixture");

        assert!(
            !shipped.steps.is_empty(),
            "the fixture must converge at least one step, or this gate compares \
             two empty ramps",
        );
        assert_eq!(
            shipped.steps.len(),
            delegated.steps.len(),
            "the shipped tolerance must produce the same number of steps",
        );
        for (a, b) in shipped.steps.iter().zip(delegated.steps.iter()) {
            assert!(
                (a.interference_m - b.interference_m).abs() < f64::EPSILON,
                "interference moved: {} vs {}",
                a.interference_m,
                b.interference_m,
            );
            assert_eq!(a.iter_count, b.iter_count, "iteration count moved");
            assert!(
                (a.final_residual_norm - b.final_residual_norm).abs() < f64::EPSILON,
                "residual moved: {:e} vs {:e}",
                a.final_residual_norm,
                b.final_residual_norm,
            );
            assert_eq!(a.x_final, b.x_final, "the converged pose moved");
        }
        assert_eq!(
            shipped.failed_at_step, delegated.failed_at_step,
            "the stall point moved",
        );
    }

    /// The ramp's tolerance argument reaches the solve.
    ///
    /// The other half of the two-gate rule, and the half a delegation test
    /// cannot see: `the_shipped_ramp_solves_at_the_shipped_tolerance` would
    /// pass just as happily if `tol` were accepted and dropped on the floor
    /// → [[feedback_a_knob_needs_two_gates]].
    ///
    /// ⚠ Asserts that asking for a much tighter residual CHANGES the ramp, not
    /// where it changes it. Which step a tight request stalls on is
    /// platform-dependent and must never be pinned.
    #[test]
    fn the_ramp_tolerance_knob_reaches_the_solve() {
        let n_steps = 2;
        let loose = run_insertion_ramp_at_kappa_and_tol(
            tolerance_fixture(),
            n_steps,
            INSERTION_CONTACT_KAPPA,
            INSERTION_SOLVE_TOL,
        )
        .expect("the loose ramp must run");
        let tight = run_insertion_ramp_at_kappa_and_tol(
            tolerance_fixture(),
            n_steps,
            INSERTION_CONTACT_KAPPA,
            TIGHT_TOL,
        )
        .expect("the tight ramp must run");

        assert!(
            !loose.steps.is_empty(),
            "the loose ramp must converge something to compare against",
        );
        // Five decades of tolerance must show up somewhere: either the tight
        // request costs iterations / reaches a smaller residual, or it stalls
        // earlier. Any of those proves the argument reached the solver; none of
        // them pins WHERE.
        let moved = tight.steps.len() != loose.steps.len()
            || loose.steps.iter().zip(tight.steps.iter()).any(|(l, t)| {
                t.iter_count != l.iter_count
                    || (t.final_residual_norm - l.final_residual_norm).abs() > f64::EPSILON
            });
        assert!(
            moved,
            "asking for {TIGHT_TOL:e} instead of {INSERTION_SOLVE_TOL:e} changed \
             nothing about the ramp — the tolerance argument is not reaching the \
             solve",
        );
    }

    /// What each candidate band yields — ceiling (shipped) and floor (reported).
    ///
    /// `d̂` is a free choice on the bridge. The ceiling `σ / |b′(d̂/2)|` always
    /// exists; the floor `σ / |b′(ρ·step)|` does not, and where it fails to is
    /// worth seeing, because it is the boundary an earlier revision of this
    /// derivation ran into and mistook for a physical limit.
    #[test]
    fn the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling() {
        let inset_m = 0.005;
        let step = inset_m / 16.0;
        let mut n_with_floor = 0;

        println!("\nd_hat_mm   floor          ceiling (SHIPPED)   floor vs ceiling");
        for d_hat in BRIDGE_DHAT_CANDIDATES_M {
            let ceiling = face_barrier_kappa(d_hat, 0.5 * d_hat, BRIDGE_DESIGN_TRACTION_PA)
                .expect("the ceiling is derivable at every candidate band");
            let floor = face_barrier_kappa(
                d_hat,
                BRIDGE_PATCH_NONUNIFORMITY * step,
                BRIDGE_DESIGN_TRACTION_PA,
            );
            // ⚠ The interesting boundary is NOT whether the floor exists — at
            // the shipped schedule all four bands have one. It is whether the
            // floor sits BELOW the ceiling. At d̂ = 0.5 mm it does not
            // (3.4386e8 against 9.8730e7): an INVERTED bracket, which the old
            // selector reported as "empty" and treated as a physical limit.
            let sane = floor.is_some_and(|f| f < ceiling);
            if sane {
                n_with_floor += 1;
            }
            println!(
                "{:>8.2}   {:<14}   {ceiling:.4e}          {}",
                d_hat * 1e3,
                floor.map_or_else(|| "(none)".to_string(), |f| format!("{f:.4e}")),
                if sane { "below" } else { "INVERTED" },
            );
            // Whatever the floor does, the shipped value is the ceiling.
            let shipped = bridge_face_barrier_kappa(d_hat, step)
                .expect("κ derives wherever the increment fits the band");
            assert!(
                (shipped - ceiling).abs() <= 1e-9 * ceiling,
                "band {d_hat}: shipped κ must be the ceiling",
            );
        }

        // The boundary must be VISIBLE in this candidate set — some band has
        // no floor at the shipped schedule, some has one. Without both, the
        // table shows nothing about where the old selector broke down.
        assert!(
            n_with_floor > 0 && n_with_floor < BRIDGE_DHAT_CANDIDATES_M.len(),
            "the candidate set must straddle the inversion boundary \
             ({n_with_floor} of {} have a floor BELOW the ceiling)",
            BRIDGE_DHAT_CANDIDATES_M.len(),
        );
    }

    /// ⭐⭐⭐ The bridge against the penalty baseline **on the product scan**.
    ///
    /// `base_mold` as the CF Studio project configures it — 5 mm cavity inset,
    /// one 17 mm DRAGON_SKIN_10A layer at 25 % Slacker. See [`product_scene`]
    /// for why this and not `sock_over_capsule`.
    ///
    /// ⛔ The scan is repo-excluded, so nothing here can ever gate. It reports
    /// on a named platform.
    #[test]
    #[ignore = "needs the product scan + release ramps on both arms; run with --ignored --nocapture"]
    fn the_bridge_against_the_penalty_baseline_on_the_product_scan() {
        const TOLERANCES: [f64; 2] = [1e-1, 1e-2];
        // ⚠ BOTH schedules for BOTH arms. At 16 steps the bridge still hits
        // its contact stall (68.8 %) and needs 32 to clear it — comparing the
        // bridge at 32 against penalty at 16 would vary the method and the
        // schedule at once.
        const SCHEDULES: [usize; 2] = [16, 32];

        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        println!(
            "\n══ base_mold · inset {:.1} mm · {} layer(s) ══",
            design.cavity_inset_m * 1e3,
            design.layers.len(),
        );
        println!(
            "tol        arm                 steps  depth_mm  resid      pairs  \
             min_sd_mm  tail5_mm  sigma_kPa"
        );
        let build = || build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004);

        for n_steps in SCHEDULES {
            let n_steps_local = n_steps;
            println!("\n── {n_steps_local} steps ──");
            for tol in TOLERANCES {
                let Ok(g) = build() else {
                    println!("  geometry FAILED to build");
                    break;
                };
                let mesh4 = g.mesh.clone();
                let intruder = g.intruder.clone();
                let bounds = g.bounds;
                let cavity_offset_m = g.cavity_offset_m;
                let referenced4: Vec<VertexId> = referenced_vertices(&mesh4);
                let rest4 = boundary_vertex_areas(
                    Mesh::<Yeoh>::positions(&mesh4),
                    Mesh::<Yeoh>::boundary_faces(&mesh4),
                );
                let base = run_insertion_ramp_at_kappa_and_tol(
                    g,
                    n_steps_local,
                    INSERTION_CONTACT_KAPPA,
                    tol,
                )
                .expect("the penalty baseline ramp must build");
                let (d, r, st) = base
                    .steps
                    .last()
                    .map(|last| {
                        let pos = positions_from_flat(&last.x_final);
                        let c = intruder_contact_at_kappa(
                            &intruder,
                            bounds,
                            last.interference_m,
                            cavity_offset_m,
                            INSERTION_CONTACT_KAPPA,
                        );
                        let raw = c.per_pair_readout(&mesh4, &pos);
                        (
                            last.interference_m,
                            last.final_residual_norm,
                            patch_stats(
                                &filter_pair_readouts_to_referenced(raw, &referenced4),
                                &rest4,
                            ),
                        )
                    })
                    .unwrap_or((0.0, f64::NAN, None));
                print_arm_row(
                    &format!("{tol:.0e}"),
                    "tet4+penalty",
                    base.steps.len(),
                    n_steps_local,
                    d,
                    r,
                    st,
                );

                let Ok(g) = build() else { break };
                let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
                let referenced10: Vec<VertexId> = referenced_vertices(&mesh10);
                let rest10 = boundary_vertex_areas(
                    Mesh::<Yeoh>::positions(&mesh10),
                    Mesh::<Yeoh>::boundary_faces(&mesh10),
                );
                // `n_steps_local` is small; the cast is exact.
                #[allow(clippy::cast_precision_loss)]
                let kappa = bridge_face_barrier_kappa(
                    BRIDGE_CONTACT_DHAT_M,
                    -g.cavity_offset_m / n_steps_local as f64,
                )
                .expect("the bridge's stiffness bracket must be non-empty");
                let bridge = run_insertion_ramp_tet10_ipc(g, n_steps_local, tol)
                    .expect("the bridge ramp must build");
                let (d, r, st) = bridge
                    .steps
                    .last()
                    .map(|last| {
                        let pos = positions_from_flat(&last.x_final);
                        let c = intruder_ipc_contact_at(
                            &intruder,
                            bounds,
                            last.interference_m,
                            cavity_offset_m,
                            kappa,
                            BRIDGE_CONTACT_DHAT_M,
                        );
                        let raw = c.per_pair_readout(&mesh10, &pos);
                        (
                            last.interference_m,
                            last.final_residual_norm,
                            patch_stats(
                                &filter_pair_readouts_to_referenced(raw, &referenced10),
                                &rest10,
                            ),
                        )
                    })
                    .unwrap_or((0.0, f64::NAN, None));
                print_arm_row(
                    &format!("{tol:.0e}"),
                    "tet10+ipc",
                    bridge.steps.len(),
                    n_steps_local,
                    d,
                    r,
                    st,
                );
                if let Some(reason) = bridge.failure_reason.as_deref() {
                    println!("            (bridge: {reason})");
                }
            }
        }
    }

    /// What a bridge step COSTS — per-step wall clock, both arms.
    ///
    /// The question the bridge raises is per-STEP, not per-ramp: a ramp's
    /// total folds in a geometry build that can dominate it on a 70 k-tet
    /// scan and answers nothing about the solve. `RampStep::wall_time_s` is
    /// measured around `replay_step` alone.
    ///
    /// ⛔ **Asserts nothing** — wall clock is contended, and the one
    /// release-only timing assertion in this workspace inflates 80× under
    /// `xtask grade`'s coverage pass. Run it on an idle machine and read it as
    /// an order of magnitude, not a number.
    ///
    /// ⚠ The bridge's figure EXCLUDES its approach steps, which are solved and
    /// not recorded. Those are real cost the baseline does not pay, so the
    /// approach count is reported beside the per-step time.
    #[test]
    #[ignore = "release-mode ramps for timing; run with --ignored --nocapture on an idle machine"]
    fn what_a_bridge_step_costs() {
        const N_STEPS: usize = 16;

        fn sphere_scene() -> InsertionGeometry {
            let design = SimDesign {
                cavity_inset_m: 0.003,
                layers: vec![layer(0.010, "ECOFLEX_00_30")],
            };
            build_insertion_geometry(&icosphere(0.040, 3), &design, &[], 2_000, 0.004)
                .expect("the synthetic-sphere geometry must build")
        }

        let mut scenes: Vec<(String, Box<dyn Fn() -> InsertionGeometry>)> = vec![
            (
                "tol-fixture".into(),
                Box::new(tolerance_fixture) as Box<dyn Fn() -> InsertionGeometry>,
            ),
            ("sphere-40mm".into(), Box::new(sphere_scene)),
        ];

        // The product mesh, when the repo-excluded scan is on this machine.
        if let Some((scan, _centerline, caps, design)) = product_scene() {
            scenes.push((
                "base_mold (product)".into(),
                Box::new(move || {
                    build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004)
                        .expect("the product geometry must build")
                }),
            ));
        }

        for (label, build) in scenes {
            println!("\n══ {label} ══");

            // Counts are far below f64's exact-integer ceiling.
            #[allow(clippy::cast_precision_loss)]
            let ratio = |a: usize, b: usize| a as f64 / b as f64;
            let built = std::time::Instant::now();
            let g = build();
            let build_s = built.elapsed().as_secs_f64();
            let tet4_verts = Mesh::<Yeoh>::n_vertices(&g.mesh);
            let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
            let tet10_verts = Mesh::<Yeoh>::n_vertices(&tet10);
            println!(
                "geometry build {build_s:>7.2} s · {} tets · vertices {tet4_verts} (Tet4) \
                 → {tet10_verts} (Tet10, ×{:.2}) · DOF {} → {}",
                g.n_tets,
                ratio(tet10_verts, tet4_verts),
                3 * tet4_verts,
                3 * tet10_verts,
            );

            let report = |arm: &str, ramp: &InsertionRamp, extra: String| {
                let mut t: Vec<f64> = ramp.steps.iter().map(|s| s.wall_time_s).collect();
                if t.is_empty() {
                    println!("{arm:<14} no converged step{extra}");
                    return;
                }
                let total: f64 = t.iter().sum();
                // Step count is tiny; the cast is exact.
                #[allow(clippy::cast_precision_loss)]
                let mean = total / t.len() as f64;
                t.sort_unstable_by(f64::total_cmp);
                println!(
                    "{arm:<14} {:>2} steps · per step min {:>6.2} s · median {:>6.2} s · \
                     max {:>6.2} s · mean {:>6.2} s · ramp {:>7.1} s{extra}",
                    t.len(),
                    t[0],
                    t[t.len() / 2],
                    t[t.len() - 1],
                    mean,
                    total,
                );
            };

            let base = run_insertion_ramp_at_kappa_and_tol(
                build(),
                N_STEPS,
                INSERTION_CONTACT_KAPPA,
                INSERTION_SOLVE_TOL,
            )
            .expect("baseline ramp builds");
            report("tet4+penalty", &base, String::new());

            let bridge = run_insertion_ramp_tet10_ipc(build(), N_STEPS, INSERTION_SOLVE_TOL)
                .expect("bridge ramp builds");
            report("tet10+ipc", &bridge, String::new());
        }
    }

    // ─── THE SLIDING BRIDGE ─────────────────────────────────────────

    /// A straight centerline through the tolerance fixture, tip-first.
    ///
    /// `centerline[0]` is the TIP and `.last()` the FLOOR, per
    /// `slide_pose_at`'s ordering convention.
    fn fixture_centerline() -> Vec<Point3<f64>> {
        (0..=8)
            .map(|i| {
                // Loop index is tiny; the cast is exact.
                #[allow(clippy::cast_precision_loss)]
                Point3::new(0.0, 0.0, f64::from(i) * 0.005)
            })
            .collect()
    }

    /// The measured CLOSING is what decides the slide schedule.
    ///
    /// ⛔ **An earlier revision of this gate asserted the normal closing is
    /// much smaller than the arc increment. Measured, that is FALSE here** —
    /// 2.4069 mm against 2.5 mm, a ratio of 0.963 — because this fixture's
    /// intruder is a sphere entering a hole, so its surface is
    /// near-perpendicular to the motion and nearly the whole step closes. The
    /// ratio is a property of the scene's geometry, so this gate asserts only
    /// what is invariant and REPORTS the rest.
    ///
    /// Invariant: the closing is positive (something entered the band) and
    /// cannot exceed the total motion. Consequence, which is the useful part:
    /// on this scene the schedule is far too coarse to bracket a stiffness, so
    /// the derivation must REFUSE — and that refusal is asserted, because a
    /// derivation that silently returned a number for an impossible
    /// requirement is the failure this whole approach exists to avoid.
    #[test]
    fn the_sliding_closing_decides_the_schedule() {
        let g = tolerance_fixture();
        let centerline = fixture_centerline();
        let n_steps = 16;
        let arc_m = polyline_arc_length_m(&centerline);
        // `n_steps` is tiny; the cast is exact.
        let arc_increment_m = arc_m / f64::from(u32::try_from(n_steps).expect("tiny"));

        let faces: Vec<[VertexId; 3]> = Mesh::<Yeoh>::boundary_faces(&g.mesh).to_vec();
        let rest = Mesh::<Yeoh>::positions(&g.mesh);
        let mut ids: Vec<VertexId> = faces.iter().flatten().copied().collect();
        ids.sort_unstable();
        ids.dedup();
        let wall: Vec<Vec3> = ids.into_iter().map(|v| rest[v as usize]).collect();
        assert!(!wall.is_empty(), "the fixture must have boundary nodes");

        let normal_m = sliding_normal_increment_m(
            &g.intruder,
            g.bounds,
            &centerline,
            n_steps,
            g.cavity_offset_m,
            BRIDGE_CONTACT_DHAT_M,
            &wall,
        );
        println!(
            "arc L {:.4} mm · arc increment {:.4} mm · normal closing {:.4} mm · \
             ratio {:.3}",
            arc_m * 1e3,
            arc_increment_m * 1e3,
            normal_m * 1e3,
            normal_m / arc_increment_m,
        );

        assert!(
            normal_m > 0.0,
            "the closing must be positive — a zero would mean no node ever \
             entered the band and every number below would be vacuous",
        );
        assert!(
            normal_m <= arc_increment_m * (1.0 + 1e-9),
            "the normal closing {normal_m:.6e} m cannot exceed the total motion \
             {arc_increment_m:.6e} m",
        );
        // The consequence, and the half that can fail: this schedule cannot
        // bracket a stiffness, and the derivation must say so rather than
        // return a number.
        assert!(
            bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, normal_m).is_err(),
            "a closing of {normal_m:.6e} m against a {:.6e} m band must leave NO \
             bracket — if this starts passing, the fixture or the band changed \
             and the schedule guidance below is stale",
            BRIDGE_CONTACT_DHAT_M,
        );
        // And a schedule fine enough DOES bracket — without this the gate only
        // ever sees the refusal and could not tell a broken derivation from a
        // correctly-refusing one.
        let fine_closing_m = 0.5 * (0.5 * BRIDGE_CONTACT_DHAT_M) / BRIDGE_PATCH_NONUNIFORMITY;
        assert!(
            bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, fine_closing_m).is_ok(),
            "a closing of half the allowance must bracket a stiffness",
        );
    }

    /// The sliding bridge takes the FACE barrier, not the per-vertex path.
    ///
    /// Sibling of `the_bridge_selects_the_face_barrier_not_the_vertex_path`,
    /// and it exists for the same reason: the capability is chosen by a
    /// runtime `Option`, so a mesh that merely *compiles* against the Tet10
    /// types silently takes the per-vertex path. Probes `active_pairs` — what
    /// the solver scatters — with the Tet4 mesh as negative control.
    #[test]
    fn the_sliding_bridge_selects_the_face_barrier_not_the_vertex_path() {
        use sim_soft::ActivePairsFor;

        let g = tolerance_fixture();
        let centerline = fixture_centerline();
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        // t = 1 is the seated pose, where contact certainly exists.
        let contact = intruder_ipc_contact_sliding_at(
            &g.intruder,
            g.bounds,
            slide_pose_at(&centerline, 1.0),
            g.cavity_offset_m,
            1.0e7,
            BRIDGE_CONTACT_DHAT_M,
        );

        let positions = tet10.positions().to_vec();
        let pairs = ActivePairsFor::<Yeoh>::active_pairs(&contact, &tet10, &positions);
        assert!(
            !pairs.is_empty(),
            "the seated pose must produce active pairs, or the face-kind check \
             below passes vacuously",
        );
        let n_vertex = pairs
            .iter()
            .filter(|p| matches!(p, ContactPair::Vertex { .. }))
            .count();
        assert_eq!(
            n_vertex,
            0,
            "every solver pair on the sliding bridge must be a Face pair; {} of \
             {} were Vertex pairs",
            n_vertex,
            pairs.len(),
        );

        let tet4_positions = Mesh::<Yeoh>::positions(&g.mesh).to_vec();
        let tet4_pairs = ActivePairsFor::<Yeoh>::active_pairs(&contact, &g.mesh, &tet4_positions);
        assert!(
            !tet4_pairs.is_empty()
                && tet4_pairs
                    .iter()
                    .all(|p| matches!(p, ContactPair::Vertex { .. })),
            "the Tet4 mesh must still take the per-vertex path ({} pairs)",
            tet4_pairs.len(),
        );
    }

    /// ⭐⭐⭐ **σ and ρ on the PRODUCT scan** — the numbers the bridge's `κ`
    /// is actually derived from.
    ///
    /// ⛔ **The 117 kPa [`BRIDGE_DESIGN_TRACTION_PA`] used to hold was read on
    /// #959's 3 mm Ecoflex 00-30 scenes** (the synthetic sphere, with
    /// `sock_over_capsule` agreeing). `base_mold`
    /// is a **5 mm** inset through **17 mm of DRAGON_SKIN_10A at 25 %
    /// Slacker** — a substantially stiffer wall pressed further — so there is
    /// no reason for σ to carry across, and κ scales with it linearly. This is
    /// the re-measurement, and the constant now carries what it returned.
    ///
    /// Same method as #959, so the two are comparable: sweep the penalty
    /// stiffness, read the area-weighted mean traction on the REST area basis
    /// at the deepest depth every arm reached, and report the gap
    /// distribution beside it so a reading taken off a penetrating state is
    /// visible as such.
    ///
    /// ⚠ **Read σ only off a NON-penetrating arm.** The shipped κ = 1e3 reads
    /// it off a state through the wall on every scene tried so far.
    ///
    /// ⛔ Asserts nothing — it is a measurement, and the scan is repo-excluded
    /// so it could never gate.
    #[test]
    #[ignore = "needs the product scan + 5 release ramps; run with --ignored --nocapture"]
    fn the_design_traction_and_patch_nonuniformity_on_the_product_scan() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        let cavity_inset_m = design.cavity_inset_m;
        let n_steps = 16_usize;
        eprintln!(
            "PRODUCT SCAN base_mold — cavity {:.1} mm, {} layer(s), cell 4 mm, {n_steps} steps",
            cavity_inset_m * 1e3,
            design.layers.len(),
        );
        for l in &design.layers {
            eprintln!(
                "  layer: {:.1} mm {} @ slacker {:.2}",
                l.thickness_m * 1e3,
                l.anchor_key,
                l.slacker_fraction,
            );
        }
        report_sigma_vs_stiffness("base_mold", cavity_inset_m, n_steps, || {
            build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004)
                .expect("the product geometry must build")
        });
    }

    /// Does marching FINER fix the bridge's feasibility stall, as the
    /// derivation predicts?
    ///
    /// ⭐⭐ **This tests a prediction rather than tuning a constant.** On the
    /// product scan at 16 steps the bridge stalls at 6/16 with an
    /// infeasible-start signature (Armijo at Newton iteration 0, `r_norm`
    /// 3.6e4): the last converged step held `min_sd` 0.2936 mm and the next
    /// increment was 0.3125 mm. The increment outran the standoff.
    ///
    /// The κ floor is a MARCHING-SCHEME number — "hold `ρ · step` open" — so
    /// the derivation's own answer to that is a finer march, not a stiffer
    /// barrier. Halving the step halves what must be held AND widens the
    /// bracket (the floor falls; the ceiling does not move). If depth does not
    /// improve with step count, the floor is not the binding constraint and
    /// the ρ substitution is wrong in a way finer marching cannot fix.
    ///
    /// ⛔ Reaching for a stiffer κ first would be sweeping wearing a
    /// derivation's clothes — the exact thing this approach exists to avoid.
    ///
    /// ⛔ Asserts nothing; stall points are platform-dependent.
    #[test]
    #[ignore = "needs the product scan + release ramps at several schedules; run with --ignored --nocapture"]
    fn does_marching_finer_fix_the_bridges_feasibility_stall() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        let inset_m = design.cavity_inset_m;
        println!(
            "\nbase_mold · inset {:.1} mm · the floor must hold rho*step open\n",
            inset_m * 1e3,
        );
        println!("steps   step_mm   kappa        held_min_sd  depth_mm   of_inset  outcome");
        for n_steps in [16_usize, 32, 64] {
            let step_m = inset_m / f64::from(u32::try_from(n_steps).expect("fits"));
            let kappa = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, step_m)
                .expect("the bracket must be non-empty at these schedules");
            let Ok(g) = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004) else {
                println!("geometry FAILED to build");
                return;
            };
            let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
            let referenced: Vec<VertexId> = referenced_vertices(&mesh10);
            let rest10 = boundary_vertex_areas(
                Mesh::<Yeoh>::positions(&mesh10),
                Mesh::<Yeoh>::boundary_faces(&mesh10),
            );
            let intruder = g.intruder.clone();
            let bounds = g.bounds;
            let cavity_offset_m = g.cavity_offset_m;

            let ramp = run_insertion_ramp_tet10_ipc(g, n_steps, INSERTION_SOLVE_TOL)
                .expect("the bridge ramp must build");
            let (depth_m, min_sd_mm) = ramp.steps.last().map_or((0.0, f64::NAN), |last| {
                let pos = positions_from_flat(&last.x_final);
                let c = intruder_ipc_contact_at(
                    &intruder,
                    bounds,
                    last.interference_m,
                    cavity_offset_m,
                    kappa,
                    BRIDGE_CONTACT_DHAT_M,
                );
                let raw = c.per_pair_readout(&mesh10, &pos);
                let stats = patch_stats(
                    &filter_pair_readouts_to_referenced(raw, &referenced),
                    &rest10,
                );
                (
                    last.interference_m,
                    stats.map_or(f64::NAN, |s| s.min_sd_m * 1e3),
                )
            });
            let outcome = if ramp.steps.len() == n_steps {
                "COMPLETE".to_string()
            } else {
                format!("stalled {}/{n_steps}", ramp.steps.len())
            };
            println!(
                "{n_steps:>5}   {:>7.4}   {kappa:.4e}   {min_sd_mm:>10.4}   {:>7.3}   {:>6.1}%  {outcome}",
                step_m * 1e3,
                depth_m * 1e3,
                100.0 * depth_m / inset_m,
            );
        }
    }

    /// Which `ρ` makes the floor's PROMISE actually hold?
    ///
    /// ⭐⭐⭐ **The floor promises a standoff and does not deliver it.**
    /// Measured on the product scan, the barrier holds ~0.8× of the `ρ · step`
    /// the derivation asked for, at every schedule tried:
    ///
    /// ```text
    /// steps  required ρ·step   held     ratio   depth
    ///    16       0.3500 mm  0.2936 mm  0.84    37.5 %
    ///    32       0.1750 mm  0.1347 mm  0.77    46.9 %
    ///    64       0.0875 mm  0.0635 mm  0.73    60.9 %
    /// ```
    ///
    /// The shortfall did not shrink with the step (above). Why is not isolated —
    /// κ, the step and the depth all changed together.
    ///
    /// ⇒ The hypothesis this tests: `ρ` is the lever, and this measures which
    /// value keeps the promise.
    /// ⚠ **This is calibrating an approximation against its own definition,
    /// not tuning.** `ρ` means the barrier-inverted ratio
    /// `face_barrier_standoff(κ, d̂, σ)/min_sd`; what #959 substituted was a
    /// GAP ratio (`mean_sd/min_sd`), and that report said in its own output
    /// that the two "coincide only where the traction-gap map is near-linear —
    /// the substitution is not an identity". sim-soft's fixture used **1.30**
    /// from the barrier-inverted form; the gap ratio gave 1.11–1.18.
    ///
    /// The number to read off is the smallest `ρ` whose held standoff EXCEEDS
    /// one increment, since that is the feasibility condition the ramp
    /// actually needs.
    ///
    /// ⛔ Asserts nothing; stall points are platform-dependent.
    #[test]
    #[ignore = "needs the product scan + release ramps per ρ; run with --ignored --nocapture"]
    fn which_patch_nonuniformity_keeps_the_floors_promise() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        const N_STEPS: usize = 16;
        let inset_m = design.cavity_inset_m;
        let step_m = inset_m / f64::from(u32::try_from(N_STEPS).expect("fits"));
        println!(
            "\nbase_mold · inset {:.1} mm · {N_STEPS} steps · increment {:.4} mm",
            inset_m * 1e3,
            step_m * 1e3,
        );
        println!("\n   rho   required_mm   kappa        held_mm   held/step   depth_mm   outcome");
        for rho in [1.12_f64, 1.30, 1.50, 1.75] {
            let required_m = rho * step_m;
            let Some(floor) =
                face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, required_m, BRIDGE_DESIGN_TRACTION_PA)
            else {
                println!("{rho:>6.2}   (required standoff is outside the band)");
                continue;
            };
            let Some(ceiling) = face_barrier_kappa(
                BRIDGE_CONTACT_DHAT_M,
                0.5 * BRIDGE_CONTACT_DHAT_M,
                BRIDGE_DESIGN_TRACTION_PA,
            ) else {
                continue;
            };
            if floor >= ceiling {
                println!("{rho:>6.2}   {:>9.4}   (bracket EMPTY)", required_m * 1e3);
                continue;
            }
            let kappa = (floor * ceiling).sqrt();

            let Ok(g) = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004) else {
                println!("geometry FAILED to build");
                return;
            };
            let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
            let referenced: Vec<VertexId> = referenced_vertices(&mesh10);
            let rest10 = boundary_vertex_areas(
                Mesh::<Yeoh>::positions(&mesh10),
                Mesh::<Yeoh>::boundary_faces(&mesh10),
            );
            let intruder = g.intruder.clone();
            let bounds = g.bounds;
            let cavity_offset_m = g.cavity_offset_m;

            let ramp = run_insertion_ramp_tet10_ipc_at(
                g,
                N_STEPS,
                INSERTION_SOLVE_TOL,
                kappa,
                BRIDGE_CONTACT_DHAT_M,
            )
            .expect("the bridge ramp must build");
            let (depth_m, held_mm) = ramp.steps.last().map_or((0.0, f64::NAN), |last| {
                let pos = positions_from_flat(&last.x_final);
                let c = intruder_ipc_contact_at(
                    &intruder,
                    bounds,
                    last.interference_m,
                    cavity_offset_m,
                    kappa,
                    BRIDGE_CONTACT_DHAT_M,
                );
                let raw = c.per_pair_readout(&mesh10, &pos);
                let stats = patch_stats(
                    &filter_pair_readouts_to_referenced(raw, &referenced),
                    &rest10,
                );
                (
                    last.interference_m,
                    stats.map_or(f64::NAN, |s| s.min_sd_m * 1e3),
                )
            });
            let outcome = if ramp.steps.len() == N_STEPS {
                "COMPLETE".to_string()
            } else {
                format!("stalled {}/{N_STEPS}", ramp.steps.len())
            };
            println!(
                "{rho:>6.2}   {:>9.4}   {kappa:.4e}   {held_mm:>7.4}   {:>9.3}   {:>7.3}   {outcome}",
                required_m * 1e3,
                held_mm / (step_m * 1e3),
                depth_m * 1e3,
            );
        }
        println!(
            "\n⚠ feasibility needs held/step > 1. The gap-ratio ρ in use is {:.2}.",
            BRIDGE_PATCH_NONUNIFORMITY,
        );
    }

    /// WHAT is the node that limits the bridge's march?
    ///
    /// ⭐⭐⭐ The stall is a feasibility failure on `min_sd`, and `min_sd` was
    /// measured flat at ~0.294 mm across a 1.5× range of κ at 16 steps — a
    /// result about that window (see `bridge_face_barrier_kappa`). This asks
    /// whether a geometric offset accounts for it.
    ///
    /// The suspect is enrichment. `Tet10Mesh::from_tet4` puts every midside at
    /// the straight-edge MIDPOINT, so on a curved cavity a boundary midside
    /// sits under the true surface by the sagitta — measured at **0.117 mm**
    /// on the tolerance fixture (tightest corner −0.1354 mm, tightest midside
    /// −0.2522 mm). The hypothesis this probe tested: the limiting node is a
    /// midside that was never on the surface. Conforming the midsides later
    /// left the stall where it was (see `conform_cavity_midsides`).
    ///
    /// Reports the gap distribution split by node kind, at REST (free) and at
    /// the last converged step of a real ramp (the state that actually
    /// stalls). ⛔ Asserts nothing — a diagnostic.
    #[test]
    #[ignore = "needs the product scan + a release ramp; run with --ignored --nocapture"]
    fn what_node_limits_the_bridges_march() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        const N_STEPS: usize = 16;
        let Ok(g) = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004) else {
            println!("geometry FAILED to build");
            return;
        };
        let tet10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        let n_corners = tet10.n_corners();
        let referenced: Vec<VertexId> = referenced_vertices(&tet10);
        let intruder = g.intruder.clone();
        let bounds = g.bounds;
        let cavity_offset_m = g.cavity_offset_m;
        let inset_m = -cavity_offset_m;
        let step_m = inset_m / f64::from(u32::try_from(N_STEPS).expect("fits"));
        let kappa = bridge_face_barrier_kappa(BRIDGE_CONTACT_DHAT_M, step_m)
            .expect("bracket must be non-empty");

        // ── REST: how far under the cavity surface does enrichment put things?
        let cavity = Solid::from_sdf(intruder.clone(), bounds).offset(cavity_offset_m);
        let positions = Mesh::<Yeoh>::positions(&tet10);
        let (mut corner_worst, mut mid_worst) = (f64::INFINITY, f64::INFINITY);
        for &v in &referenced {
            let sd = cavity.eval(Point3::from(positions[v as usize]));
            if !sd.is_finite() || sd.abs() > BRIDGE_CONTACT_DHAT_M {
                continue;
            }
            if (v as usize) < n_corners {
                corner_worst = corner_worst.min(sd);
            } else {
                mid_worst = mid_worst.min(sd);
            }
        }
        println!(
            "\nAT REST, within one band of the cavity surface:\n  \
             tightest CORNER {:>8.4} mm · tightest MIDSIDE {:>8.4} mm · \
             enrichment excess {:>7.4} mm",
            corner_worst * 1e3,
            mid_worst * 1e3,
            (corner_worst - mid_worst) * 1e3,
        );

        // ── CONVERGED: which kind of node carries the limiting gap?
        let ramp = run_insertion_ramp_tet10_ipc(g, N_STEPS, INSERTION_SOLVE_TOL)
            .expect("the bridge ramp must build");
        let Some(last) = ramp.steps.last() else {
            println!("no converged step");
            return;
        };
        let pos = positions_from_flat(&last.x_final);
        let contact = intruder_ipc_contact_at(
            &intruder,
            bounds,
            last.interference_m,
            cavity_offset_m,
            kappa,
            BRIDGE_CONTACT_DHAT_M,
        );
        let readouts =
            filter_pair_readouts_to_referenced(contact.per_pair_readout(&tet10, &pos), &referenced);

        let (mut c_min, mut m_min) = (f64::INFINITY, f64::INFINITY);
        let (mut n_c, mut n_m) = (0_usize, 0_usize);
        for r in &readouts {
            if !r.tributary_area.is_finite() || r.tributary_area <= 0.0 {
                continue;
            }
            if let ContactPair::Vertex { vertex_id, .. } = r.pair {
                if (vertex_id as usize) < n_corners {
                    n_c += 1;
                    c_min = c_min.min(r.sd);
                } else {
                    n_m += 1;
                    m_min = m_min.min(r.sd);
                }
            }
        }
        println!(
            "\nAT THE LAST CONVERGED STEP ({:.3} mm, {}/{N_STEPS}):\n  \
             load-bearing CORNERS {n_c:>5}, tightest {:>8.4} mm\n  \
             load-bearing MIDSIDES {n_m:>4}, tightest {:>8.4} mm\n  \
             one increment is {:.4} mm",
            last.interference_m * 1e3,
            ramp.steps.len(),
            c_min * 1e3,
            m_min * 1e3,
            step_m * 1e3,
        );
        println!(
            "\n⇒ the limiting node is a {}",
            if m_min < c_min { "MIDSIDE" } else { "CORNER" },
        );
    }

    /// Did `conform_cavity_midsides` actually MOVE anything?
    ///
    /// ⚠ Written because the conform step was wired in and the solve barely
    /// changed — residual 3.6438e4 → 3.6429e4, four significant figures of
    /// agreement. That is what a no-op looks like, and assuming a helper ran
    /// because it was called is the failure this checks for.
    ///
    /// ⛔ Asserts nothing — a diagnostic.
    #[test]
    #[ignore = "needs the product scan; run with --ignored --nocapture"]
    fn did_conforming_the_midsides_move_anything() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        let Ok(g) = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004) else {
            println!("geometry FAILED to build");
            return;
        };
        let plain = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
        let n_corners = plain.n_corners();
        let before: Vec<Vec3> = Mesh::<Yeoh>::positions(&plain).to_vec();
        let conformed = conform_cavity_midsides(
            Tet10Mesh::<Yeoh>::from_tet4(&g.mesh),
            &g.intruder,
            g.bounds,
            g.cavity_offset_m,
            g.cell_size_m,
        );
        let after: Vec<Vec3> = Mesh::<Yeoh>::positions(&conformed).to_vec();
        assert_eq!(
            before.len(),
            after.len(),
            "conforming must not change count"
        );

        let mut moved = 0_usize;
        let mut max_move = 0.0_f64;
        let mut sum_move = 0.0_f64;
        for (b, a) in before.iter().zip(after.iter()) {
            let d = (a - b).norm();
            if d > 1e-12 {
                moved += 1;
                sum_move += d;
                max_move = max_move.max(d);
            }
        }
        println!(
            "\nmidsides moved: {moved} of {} total nodes ({} corners)\n  \
             max move {:.4} mm · mean move {:.4} mm",
            before.len(),
            n_corners,
            max_move * 1e3,
            if moved == 0 {
                0.0
            } else {
                sum_move / f64::from(u32::try_from(moved).expect("fits")) * 1e3
            },
        );

        // How many were even CANDIDATES — on a P2 boundary face at all?
        let faces6 = Mesh::<Yeoh>::boundary_faces6(&plain).expect("Tet10 surfaces P2 faces");
        let mut cand: Vec<VertexId> = faces6.iter().flat_map(|f| f[3..6].to_vec()).collect();
        cand.sort_unstable();
        cand.dedup();
        let cavity = Solid::from_sdf(g.intruder.clone(), g.bounds).offset(g.cavity_offset_m);
        let band_m = CAVITY_MIDSIDE_BAND_CELLS * g.cell_size_m;
        let in_band = cand
            .iter()
            .filter(|&&v| {
                let sd = cavity.eval(Point3::from(before[v as usize]));
                sd.is_finite() && sd.abs() <= band_m
            })
            .count();
        println!(
            "  boundary-face midsides: {} · within the {:.1} mm cavity band: {in_band}",
            cand.len(),
            band_m * 1e3,
        );

        // The quantity the fix exists to move.
        let tightest = |m: &[Vec3]| -> f64 {
            cand.iter()
                .map(|&v| cavity.eval(Point3::from(m[v as usize])))
                .filter(|s| s.is_finite())
                .fold(f64::INFINITY, f64::min)
        };
        println!(
            "  tightest boundary midside vs the cavity surface: {:.4} mm → {:.4} mm",
            tightest(&before) * 1e3,
            tightest(&after) * 1e3,
        );
    }

    /// ⭐⭐⭐ Does DECOUPLING `κ` from the schedule fix the stall?
    ///
    /// The floor derives `κ` from the increment, so refining the march LOWERS
    /// `κ` — and measured, held/step fell 0.94 → 0.86 → 0.81 across 16/32/64
    /// steps even as depth improved (κ, step and depth changed together).
    ///
    /// But in that window the held standoff did not follow `κ` (0.294 mm
    /// across a 1.5× range) — a result about that window, not about the wall;
    /// see `bridge_face_barrier_kappa`.
    ///
    /// ⇒ **The hypothesis: hold `κ` at the CEILING and refine the
    /// schedule.** The ceiling is the stated requirement (stay out of the
    /// cushion) and does not move with the increment, on the expectation that
    /// the held standoff then stays put while the step shrinks under it.
    ///
    /// Prediction: at `κ` = ceiling, 32 steps (0.156 mm) should clear a held
    /// standoff of ~0.29 mm and march past the 6/16 wall.
    ///
    /// Outcome (recorded in `bridge_face_barrier_kappa`): depth rose to 90.6 %
    /// at 32 steps — but the held standoff did NOT stay put (0.3062 → 0.2163 mm
    /// from 16 to 32 steps).
    ///
    /// ⛔ Asserts nothing; stall points are platform-dependent.
    #[test]
    #[ignore = "needs the product scan + release ramps per schedule; run with --ignored --nocapture"]
    fn does_decoupling_kappa_from_the_schedule_fix_the_stall() {
        let Some((scan, _centerline, caps, design)) = product_scene() else {
            return;
        };
        let inset_m = design.cavity_inset_m;
        let ceiling = face_barrier_kappa(
            BRIDGE_CONTACT_DHAT_M,
            0.5 * BRIDGE_CONTACT_DHAT_M,
            BRIDGE_DESIGN_TRACTION_PA,
        )
        .expect("the ceiling is always derivable");
        println!(
            "\nbase_mold · inset {:.1} mm · kappa HELD at the ceiling {ceiling:.4e} \
             (d_hat {:.2} mm)\n",
            inset_m * 1e3,
            BRIDGE_CONTACT_DHAT_M * 1e3,
        );
        println!("steps   step_mm   held_mm   depth_mm   of_inset   outcome");
        for n_steps in [16_usize, 32, 64] {
            let step_m = inset_m / f64::from(u32::try_from(n_steps).expect("fits"));
            let Ok(g) = build_insertion_geometry(&scan, &design, &caps, 2_500, 0.004) else {
                return;
            };
            let mesh10 = Tet10Mesh::<Yeoh>::from_tet4(&g.mesh);
            let referenced: Vec<VertexId> = referenced_vertices(&mesh10);
            let rest10 = boundary_vertex_areas(
                Mesh::<Yeoh>::positions(&mesh10),
                Mesh::<Yeoh>::boundary_faces(&mesh10),
            );
            let (intruder, bounds, cavity_offset_m) =
                (g.intruder.clone(), g.bounds, g.cavity_offset_m);
            let ramp = run_insertion_ramp_tet10_ipc_at(
                g,
                n_steps,
                INSERTION_SOLVE_TOL,
                ceiling,
                BRIDGE_CONTACT_DHAT_M,
            )
            .expect("the bridge ramp must build");
            let (depth_m, held_mm) = ramp.steps.last().map_or((0.0, f64::NAN), |last| {
                let pos = positions_from_flat(&last.x_final);
                let c = intruder_ipc_contact_at(
                    &intruder,
                    bounds,
                    last.interference_m,
                    cavity_offset_m,
                    ceiling,
                    BRIDGE_CONTACT_DHAT_M,
                );
                let raw = c.per_pair_readout(&mesh10, &pos);
                let st = patch_stats(
                    &filter_pair_readouts_to_referenced(raw, &referenced),
                    &rest10,
                );
                (
                    last.interference_m,
                    st.map_or(f64::NAN, |s| s.min_sd_m * 1e3),
                )
            });
            println!(
                "{n_steps:>5}   {:>7.4}   {held_mm:>7.4}   {:>7.3}   {:>7.1}%   {}",
                step_m * 1e3,
                depth_m * 1e3,
                100.0 * depth_m / inset_m,
                if ramp.steps.len() == n_steps {
                    "COMPLETE".to_string()
                } else {
                    format!("stalled {}/{n_steps}", ramp.steps.len())
                },
            );
        }
    }
}
