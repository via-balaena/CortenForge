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
//! - **7.0** seeded the module with the SDF-bridge *spike*,
//!   [`run_sdf_bridge_spike`]: a measurement harness that proved
//!   Route A end-to-end and characterized the decimation/timing
//!   tradeoff. `SignedDistanceField::distance` is brute-force
//!   O(faces) and the mesher samples the SDF at every BCC lattice
//!   vertex, so the raw 3.34 M-face scan must be decimated — the
//!   spike found a low target (~1.5–3k faces) is best.
//! - **7.1** adds [`build_insertion_geometry`]: the real builder that
//!   turns a [`SimDesign`] (cavity inset + layer stack) into the
//!   device-wall [`SdfMeshedTetMesh`] with per-tet Yeoh materials,
//!   plus the rigid intruder SDF.
//! - **7.2** adds [`run_single_insertion_step`]: one static FEM solve
//!   (`CpuNewtonSolver` + `PenaltyRigidContact`) that presses the
//!   intruder a chosen interference into the cavity and returns the
//!   converged deformed positions. One step only — the quasi-static
//!   ramp is 7.3.
//!
//! The UI lands at 7.4. The module is `#[cfg(test)]`-gated until 7.4
//! wires it into the app; for now it is exercised by `#[ignore]`d
//! integration tests — the iter-1 scan for the SDF-bridge spike + the
//! geometry builder, a synthetic icosphere for the solve (the real
//! scan's single-step solve does not converge — see the slice-7
//! memo's "7.2 real-scan finding"; hardening it is 7.3's battle).

use std::collections::{BTreeSet, VecDeque};
use std::time::Instant;

use anyhow::{Context, Result, anyhow};
use cf_design::{Aabb, SdfGrid, Solid};
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::{SignedDistanceField, unsigned_distance};
use mesh_types::IndexedMesh;
use meshopt::simplify_sloppy_decoder;
use nalgebra::{Point3, Vector3};
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10,
    ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50,
};
use sim_soft::{
    Aabb3, BoundaryConditions, ConstantField, CpuNewtonSolver, Field, LayeredScalarField,
    MaterialField, Mesh, MeshingHints, PenaltyRigidContact, Sdf, SdfMeshedTetMesh,
    SiliconeMaterial, Solver, SolverConfig, Tet4, Vec3, VertexId, Yeoh, pick_vertices_by_predicate,
    referenced_vertices,
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
fn decimate_for_sdf(scan: &IndexedMesh, target_faces: usize) -> IndexedMesh {
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
    /// Wall-clock time to build the [`SignedDistanceField`].
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
/// Pipeline: [`decimate_for_sdf`] → [`SignedDistanceField`] →
/// [`Solid::from_sdf`] → `outer.subtract(scan)` body (`outer =
/// scan.offset(wall_thickness_m)`) → [`SdfMeshedTetMesh::from_sdf`] at
/// `cell_size_m`. Geometry only — no materials (skeleton-default
/// Neo-Hookean), no intruder, no solve.
///
/// # Errors
///
/// Propagates [`SignedDistanceField::new`] (empty mesh) and
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
    let sdf = SignedDistanceField::new(decimated)
        .context("build SignedDistanceField from the decimated scan")?;
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
/// insertion sim: a radial thickness + a base-silicone anchor key.
///
/// The decimated form of `main.rs`'s `LayerSpec` — drops `visible` (a
/// viewport concern) and `slacker_fraction`. **Slacker note:** the
/// per-layer Slacker ratio (slice 6.5) softens the *effective* Shore
/// hardness, which would shift the Yeoh `(μ, C₂, λ)` the sim uses;
/// 7.1 sims the *base* anchor only. Folding Slacker into the sim
/// modulus is scheduled for **7.4** — that is when `SimDesign` is
/// built from the real `LayersState` (which carries
/// `slacker_fraction`), so the resolution via
/// `SiliconeMaterial::from_effective_shore` rides the same app-state
/// conversion. `SimLayer` grows a `slacker_fraction` field then.
#[derive(Debug, Clone)]
pub struct SimLayer {
    /// Radial thickness (meters). Innermost-first ordering, same as
    /// `LayerSpec`.
    pub thickness_m: f64,
    /// Smooth-On base-silicone anchor key — one of the eight in
    /// `main.rs`'s `LAYER_MATERIALS` catalog.
    pub anchor_key: String,
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

/// Per-scan-SDF offsets (meters) of the *internal* layer boundaries —
/// one per adjacent layer pair, so `N` layers yield `N - 1`
/// thresholds.
///
/// Boundary `i` (between layer `i` and layer `i + 1`) is layer `i`'s
/// outer surface, at `sum(thickness[0..=i]) - cavity_inset_m`.
/// Strictly increasing (thicknesses are positive), as
/// [`LayeredScalarField::new`] requires. Empty for a single-layer
/// design — the caller uses a [`ConstantField`] instead.
fn layer_boundary_thresholds(design: &SimDesign) -> Vec<f64> {
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

    // Per-layer Yeoh parameters, innermost-first.
    let materials = design
        .layers
        .iter()
        .map(|layer| silicone_for_anchor(&layer.anchor_key))
        .collect::<Result<Vec<SiliconeMaterial>>>()?;
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

    // Three `LayeredScalarField`s (or `ConstantField`s) over the same
    // scan-distance partition — one per Yeoh parameter — mirroring the
    // row-23 `build_material_field` precedent.
    let material_field = MaterialField::from_yeoh_fields(
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
    );

    // Route-A device wall: outer skin minus cavity void. `Solid::offset`
    // takes signed distances — `cavity_offset_m` is negative (the
    // cavity is inset *inside* the scan).
    let cavity = Solid::from_sdf(scan_sdf.clone(), bounds).offset(cavity_offset_m);
    let outer = Solid::from_sdf(scan_sdf.clone(), bounds).offset(outer_offset_m);
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
/// value. 7.3's ramp will rebuild the geometry per step; 7.2 is one
/// step only.
///
/// # Errors
///
/// - `interference_m` is non-finite;
/// - no outer-skin vertex lands in the Dirichlet pin-band (the wall
///   has nothing to react against — typically `cell_size_m` too
///   coarse for the wall, or a degenerate geometry).
///
/// # Panics
///
/// `replay_step` panics if the Newton solve fails to converge within
/// [`MAX_NEWTON_ITER`] — non-convergence returns no partial data. 7.2
/// keeps to interferences that converge; graceful "failed at step N"
/// reporting is 7.3 (the quasi-static ramp), where the contact-
/// robustness envelope (≤ 8 mm Yeoh) is exercised in earnest.
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

    // Dirichlet BCs: pin the outer-skin vertices — those within half a
    // BCC cell of the outer envelope `scan.offset(outer_offset_m)` —
    // filtered to solver-referenced vertices (BCC orphans are not in
    // any tet). The intruder, not a loaded BC, drives the deformation.
    let referenced: BTreeSet<VertexId> = referenced_vertices(&mesh).into_iter().collect();
    let outer_envelope = Solid::from_sdf(intruder.clone(), bounds).offset(outer_offset_m);
    let band_tol = 0.5 * cell_size_m;
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| {
        outer_envelope.eval(Point3::from(*p)).abs() < band_tol
    })
    .into_iter()
    .filter(|v| referenced.contains(v))
    .collect();
    if pinned.is_empty() {
        return Err(anyhow!(
            "no outer-skin vertex landed in the {band_tol:.4} m Dirichlet band — \
             the device wall has nothing pinned to react against (cell_size_m too \
             coarse for the wall, or a degenerate geometry)"
        ));
    }
    let n_pinned = pinned.len();
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        loaded_vertices: Vec::new(),
    };

    // The rigid intruder at the requested interference: penetration
    // `d` past the cavity wall is the scan SDF offset by
    // `d + cavity_offset_m` (cavity_offset_m = -inset, so
    // `d = inset` reproduces the bare scan).
    let intruder_solid = Solid::from_sdf(intruder, bounds).offset(interference_m + cavity_offset_m);
    let contact = PenaltyRigidContact::new(vec![intruder_solid]);

    // Static solve: `dt = 1.0` collapses inertia (rows 21–25
    // quasi-static precedent); the Yeoh path gets the row-23 iter cap.
    let mut config = SolverConfig::skeleton();
    config.dt = STATIC_DT;
    config.max_newton_iter = MAX_NEWTON_ITER;

    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    // Empty θ — the insertion solve carries no differentiable
    // parameters (the intruder is kinematic, fixed at construction).
    let empty_theta: [f64; 0] = [];
    let theta = Tensor::from_slice(&empty_theta, &[0]);

    let solver = CpuNewtonSolver::new(Tet4, mesh, contact, config, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, config.dt);

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

/// Per-lattice-point label from [`build_grid_sdf`]'s flood fill.
#[derive(Clone, Copy, PartialEq)]
enum Region {
    /// Flood-reached from a bbox corner — air around the scan.
    Outside,
    /// Non-wall, not flood-reached — the scan's solid interior.
    Inside,
    /// Within `wall_threshold_m` of the surface — the flood cannot
    /// pass through it; its sign is assigned by nearest-labelled
    /// neighbour expansion.
    Wall,
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

/// The up-to-six 6-connected lattice neighbours of flat index `i` in a
/// `w × h × d` grid (`x` fastest, then `y`, then `z`).
fn neighbours6(i: usize, w: usize, h: usize, d: usize) -> [Option<usize>; 6] {
    let x = i % w;
    let y = (i / w) % h;
    let z = i / (w * h);
    [
        (x > 0).then(|| i - 1),
        (x + 1 < w).then(|| i + 1),
        (y > 0).then(|| i - w),
        (y + 1 < h).then(|| i + w),
        (z > 0).then(|| i - w * h),
        (z + 1 < d).then(|| i + w * h),
    ]
}

/// Build a flood-fill-signed [`GridSdf`] of `scan` over `bbox`.
///
/// Pipeline: sample [`mesh_sdf::unsigned_distance`] at every lattice
/// point (topology-blind — the sloppy-decimation damage that wrecks
/// `mesh_sdf`'s *signed* query does not touch the *unsigned* one) →
/// mark "wall" points within `wall_threshold_m` of the surface →
/// flood "outside" 6-connected from the eight bbox corners through
/// non-wall points → non-wall, not-reached points are the interior →
/// expand the Outside/Inside labels into the wall band by multi-
/// source BFS → signed value = `±unsigned`.
///
/// `wall_threshold_m` must be `≥ 0.5 * grid_cell_m` so the wall band
/// is 6-connectivity-watertight (a surface crossing between adjacent
/// lattice points always lands one of them within half a cell). The
/// 7.3a fix spike sweeps `grid_cell_m`; `0.75 * grid_cell_m` is the
/// recommended threshold (safe margin without over-thickening the
/// band).
///
/// # Errors
///
/// Returns an error if all eight bbox corners are wall points (the
/// bbox margin is too small, or the grid too coarse, to seed the
/// outside flood).
pub fn build_grid_sdf(
    scan: &IndexedMesh,
    bbox: Aabb,
    grid_cell_m: f64,
    wall_threshold_m: f64,
) -> Result<(GridSdf, GridSdfReport)> {
    let t = Instant::now();

    // Lattice dimensions — sample points inclusive of both bbox ends.
    let span = bbox.max - bbox.min;
    // `span` components are non-negative and `grid_cell_m > 0`, so the
    // ceil is a finite non-negative integer; +1 for the inclusive end.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let dim = |s: f64| (s / grid_cell_m).ceil().max(1.0) as usize + 1;
    let (w, h, d) = (dim(span.x), dim(span.y), dim(span.z));
    let n = w * h * d;
    let origin = bbox.min;
    let flat = |x: usize, y: usize, z: usize| z * w * h + y * w + x;

    // 1. Unsigned distance at every lattice point.
    #[allow(clippy::cast_precision_loss)] // lattice index → world coord
    let world = |x: usize, y: usize, z: usize| {
        Point3::new(
            origin.x + x as f64 * grid_cell_m,
            origin.y + y as f64 * grid_cell_m,
            origin.z + z as f64 * grid_cell_m,
        )
    };
    let mut unsigned = vec![0.0_f64; n];
    for z in 0..d {
        for y in 0..h {
            for x in 0..w {
                unsigned[flat(x, y, z)] = unsigned_distance(world(x, y, z), scan);
            }
        }
    }

    // 2. Wall points — within `wall_threshold_m` of the surface. The
    //    flood cannot pass through these, so it cannot leak across
    //    the surface.
    let wall: Vec<bool> = unsigned.iter().map(|&u| u < wall_threshold_m).collect();
    let n_wall = wall.iter().filter(|&&b| b).count();

    // 3. Flood "outside" 6-connected from the eight bbox corners.
    let mut region: Vec<Region> = wall
        .iter()
        .map(|&is_wall| {
            if is_wall {
                Region::Wall
            } else {
                Region::Inside
            }
        })
        .collect();
    let corners = [
        flat(0, 0, 0),
        flat(w - 1, 0, 0),
        flat(0, h - 1, 0),
        flat(w - 1, h - 1, 0),
        flat(0, 0, d - 1),
        flat(w - 1, 0, d - 1),
        flat(0, h - 1, d - 1),
        flat(w - 1, h - 1, d - 1),
    ];
    let mut flood: VecDeque<usize> = VecDeque::new();
    for &c in &corners {
        if region[c] == Region::Inside {
            region[c] = Region::Outside;
            flood.push_back(c);
        }
    }
    if flood.is_empty() {
        return Err(anyhow!(
            "grid SDF flood-fill: all eight bbox corners are within wall_threshold_m \
             ({wall_threshold_m} m) of the scan — bbox margin too small or grid too coarse"
        ));
    }
    while let Some(i) = flood.pop_front() {
        for j in neighbours6(i, w, h, d).into_iter().flatten() {
            if region[j] == Region::Inside {
                region[j] = Region::Outside;
                flood.push_back(j);
            }
        }
    }

    // 4. Expand the Outside/Inside labels into the wall band: multi-
    //    source BFS from every already-labelled (non-wall) point —
    //    each wall point takes the label of the nearest one. Wall-band
    //    `|value|` is sub-threshold, so a tie there barely matters.
    let mut expand: VecDeque<usize> = (0..n).filter(|&i| region[i] != Region::Wall).collect();
    while let Some(i) = expand.pop_front() {
        let label = region[i];
        for j in neighbours6(i, w, h, d).into_iter().flatten() {
            if region[j] == Region::Wall {
                region[j] = label;
                expand.push_back(j);
            }
        }
    }
    // Any wall point with no path to a non-wall point (an isolated
    // pocket inside the band) — default to Inside.
    for r in &mut region {
        if *r == Region::Wall {
            *r = Region::Inside;
        }
    }

    // 5. Inside-region connected-component count — the flood-fill
    //    health metric.
    let mut seen = vec![false; n];
    let mut inside_components = 0;
    for start in 0..n {
        if region[start] != Region::Inside || seen[start] {
            continue;
        }
        inside_components += 1;
        seen[start] = true;
        let mut comp: VecDeque<usize> = VecDeque::from([start]);
        while let Some(i) = comp.pop_front() {
            for j in neighbours6(i, w, h, d).into_iter().flatten() {
                if region[j] == Region::Inside && !seen[j] {
                    seen[j] = true;
                    comp.push_back(j);
                }
            }
        }
    }

    // 6. Signed values — magnitude from the (reliable) unsigned
    //    distance, sign from the (topological) flood-fill label.
    let signed: Vec<f64> = region
        .iter()
        .zip(&unsigned)
        .map(|(&r, &u)| if r == Region::Outside { u } else { -u })
        .collect();
    let n_outside = region.iter().filter(|&&r| r == Region::Outside).count();
    let n_inside = n - n_outside;

    let grid = SdfGrid::new(signed, w, h, d, grid_cell_m, origin);
    let report = GridSdfReport {
        dims: [w, h, d],
        grid_cell_m,
        n_outside,
        n_inside,
        n_wall,
        inside_components,
        build_ms: elapsed_ms(t),
    };
    Ok((GridSdf { grid }, report))
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

    /// One [`SimLayer`] — test sugar.
    fn layer(thickness_m: f64, anchor: &str) -> SimLayer {
        SimLayer {
            thickness_m,
            anchor_key: anchor.to_string(),
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

    #[test]
    fn build_insertion_geometry_rejects_degenerate_designs() {
        // Degenerate inputs are caught up front as `Err` — never a
        // panic deep in `LayeredScalarField::new`. The validation runs
        // before the scan is decimated, so a stub cube is enough and
        // the test stays fast.
        let scan = unit_cube();
        let build = |design: &SimDesign| build_insertion_geometry(&scan, design, 2_500, 0.004);

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
        let g1 = build_insertion_geometry(&scan, &single, sdf_target_faces, cell_size_m)
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
        let g3 = build_insertion_geometry(&scan, &triple, sdf_target_faces, cell_size_m)
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
        let geometry = build_insertion_geometry(&scan, &design, 2_000, 0.004)
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
        let geometry = build_insertion_geometry(&scan, &design, 2_500, 0.004)
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
        let sdf = SignedDistanceField::new(decimated.clone()).expect("decimated scan SDF");
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
        match build_insertion_geometry(&raw, &design_10mm, 2_500, 0.004) {
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
        match build_insertion_geometry(&raw, &design_5mm, 2_500, 0.004) {
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
        let legacy = SignedDistanceField::new(decimated.clone()).expect("decimated scan SDF");

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
}
