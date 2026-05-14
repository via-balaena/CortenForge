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
//!
//! Still no solve — the Newton solve + quasi-static ramp land at
//! 7.2–7.3, the UI at 7.4. The module is `#[cfg(test)]`-gated until
//! 7.4 wires it into the app; for now it is exercised by `#[ignore]`d
//! integration tests against the iter-1 scan.

use std::time::Instant;

use anyhow::{Context, Result, anyhow};
use cf_design::{Aabb, Solid};
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::SignedDistanceField;
use mesh_types::IndexedMesh;
use meshopt::simplify_sloppy_decoder;
use nalgebra::{Point3, Vector3};
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10,
    ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50,
};
use sim_soft::{
    Aabb3, ConstantField, Field, LayeredScalarField, MaterialField, Mesh, MeshingHints,
    SdfMeshedTetMesh, SiliconeMaterial, Vec3, Yeoh,
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
    scan_sdf: &SignedDistanceField,
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
pub struct InsertionGeometry {
    /// Device-wall tet mesh with per-tet Yeoh materials sampled from
    /// the layer stack. The solver (7.2) consumes this by value.
    pub mesh: SdfMeshedTetMesh<Yeoh>,
    /// The scan-derived rigid intruder, as the SDF the penalty
    /// contact primitive will consume at 7.2. Built here so 7.1's
    /// geometry pass is self-contained; unused until the solve lands.
    pub intruder: SignedDistanceField,
    /// Scan-SDF offset (m) of the cavity surface — `-cavity_inset_m`.
    pub cavity_offset_m: f64,
    /// Scan-SDF offset (m) of the outer skin —
    /// `total_thickness - cavity_inset_m`.
    pub outer_offset_m: f64,
    /// Tet count of `mesh` — surfaced for the 7.4 UI readout + tests.
    pub n_tets: usize,
}

/// Build the Route-A insertion-sim geometry: the device-wall tet mesh
/// with per-tet Yeoh materials, plus the rigid intruder SDF.
///
/// Geometry (mirrors the sim-soft rows 21–25 layered-sleeve path):
/// decimate the scan → [`SignedDistanceField`] → cavity =
/// `scan.offset(-inset)`, outer skin = `scan.offset(total - inset)`,
/// `body = outer.subtract(cavity)`. Materials: each layer's base
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
/// - a layer names an anchor key outside the catalog;
/// - [`SignedDistanceField::new`] fails (empty decimated scan);
/// - `SdfMeshedTetMesh::from_sdf_yeoh` fails (empty mesh — e.g. a
///   degenerate design whose cavity has collapsed — or a non-finite
///   SDF sample).
pub fn build_insertion_geometry(
    scan: &IndexedMesh,
    design: &SimDesign,
    sdf_target_faces: usize,
    cell_size_m: f64,
) -> Result<InsertionGeometry> {
    if design.layers.is_empty() {
        return Err(anyhow!("insertion-sim design has no layers"));
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
    let scan_sdf = SignedDistanceField::new(decimated)
        .context("build SignedDistanceField from the decimated scan")?;

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
        n_tets,
    })
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level; the test
    // module opts out, same posture as `main.rs`'s test module.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use mesh_io::load_stl;

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
        // The rigid intruder is the decimated scan SDF — confirm it
        // wraps a non-empty mesh (it drives the 7.2 press-fit ramp).
        assert!(
            !g1.intruder.mesh().faces.is_empty(),
            "intruder SDF must wrap a non-empty mesh",
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
}
