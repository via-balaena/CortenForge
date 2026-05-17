//! `insertion_sim_ui` — slice 7.4 UI + ECS glue for the Insertion Sim
//! panel.
//!
//! Wraps the `#[cfg(test)]`-gated-through-7.3b.2 [`insertion_sim`] module
//! into a Bevy plugin that lets the user click "Simulate Insertion" in
//! the right-side panel, runs the ramp off the main thread via
//! `AsyncComputeTaskPool`, then surfaces per-step + per-layer outputs
//! (F-d curve plot, per-step table, per-layer aggregates) and a
//! heat-map mode that recolors the existing per-layer surface shells by
//! per-vertex Ψ or ‖P‖ projected from the nearest in-layer tet's
//! converged readout.
//!
//! Slice-7 ladder context: structural arc closed at 7.3b.2 — the
//! Insertion-Sim panel here is the engineering payoff of slices 7.0
//! through 7.3b.2 reaching the user. The bookmark + slice-7-plan memo
//! both name "per-layer max stretch/stress" as the canonical output;
//! per-vertex heat-map projection (Option C, picked over per-layer-flat
//! and full-per-tet-mesh) gives ~1 cm spatial detail on the existing
//! smooth proxy shells without a new mesh-render path.

use std::collections::BTreeMap;
use std::fmt;

use anyhow::Result;
use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use sim_soft::{Mesh as SimMesh, TetId, VertexId, Yeoh};

use cf_cap_planes::CapPlane;

use crate::insertion_sim::{
    InsertionRamp, RampStep, SimDesign, SimLayer, StepReadout, TetReadout,
    build_insertion_geometry, compute_tet_readouts, run_insertion_ramp,
};
use crate::sdf_layers::CapPlanes;
use crate::{CavityState, LAYER_SURFACE_PALETTE, LayersState, ScanMesh};

// ── tuned defaults ──────────────────────────────────────────────────

/// SDF-source decimation target — pinned from the slice 7.0 spike
/// finding that BCC tet count + quality are governed by `cell_size_m`,
/// not the SDF face count, so a LOW resolution is preferred. Combined
/// with `SIM_CELL_SIZE_M` (4 mm) this is the empirically-best
/// operating point per `docs/INSERTION_SIM_STATE.md`'s closing update.
const SIM_SDF_TARGET_FACES: usize = 2_500;

/// BCC lattice cell size (m) — 4 mm matches the rows 21–25 contact-
/// robustness envelope + the slice-7 ladder regression fixtures.
/// Slice 7.3d retuned the GridSdf Gaussian σ to 1.0 cell at this
/// spacing; deviating from 4 mm would invalidate the σ tuning.
const SIM_CELL_SIZE_M: f64 = 0.004;

/// Default ramp step count — 16 matches both regression fixtures and
/// is the slowest empirically-stable warm-start increment at the full
/// 3 mm seating (`0.1875` mm per step). Halving to 8 risks regressing
/// the iter-1 envelope; the panel does not expose `n_steps` as a
/// knob to keep the validated configuration the only-served default.
const DEFAULT_N_STEPS: usize = 16;

// ── public types ────────────────────────────────────────────────────

/// Which scalar field drives the heat-map gradient — `Ψ` (J/m³) or
/// `‖P‖_F` (Pa). The Insertion Sim pipeline pre-computes vertex colors
/// for BOTH modes, so the user can toggle between them without
/// re-running the ramp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScalarMode {
    /// Strain-energy density `Ψ`. Default — what "how strained is this
    /// region" intuitively reads as.
    #[default]
    EnergyDensity,
    /// First-Piola stress Frobenius norm `‖P‖_F`. The peak-hotspot
    /// scalar; cleaner for finding stress concentrations.
    StressFrobenius,
}

impl ScalarMode {
    /// Index into each step's `[Vec<f64>; 2]` in
    /// [`InsertionSimOutputs::per_step_scalar_fields`] +
    /// [`InsertionSimOutputs::scalar_min_max`].
    pub fn buffer_index(self) -> usize {
        match self {
            Self::EnergyDensity => 0,
            Self::StressFrobenius => 1,
        }
    }

    /// Short user-facing label.
    fn label(self) -> &'static str {
        match self {
            Self::EnergyDensity => "Ψ (J/m³)",
            Self::StressFrobenius => "‖P‖ (Pa)",
        }
    }
}

/// Per-layer reduction of the final-step per-tet readout — load-
/// bearing for the panel's "compare two designs" workflow + the heat-
/// map projection's per-layer in-layer-tet sub-search.
#[derive(Debug, Clone)]
pub struct LayerAggregate {
    /// Index into the user's [`LayersState`], innermost-first (matches
    /// [`crate::insertion_sim::InsertionGeometry::per_tet_layer`]).
    pub layer_index: usize,
    /// Tets in this layer's partition.
    pub n_tets: usize,
    /// Mean strain-energy density (J/m³) across this layer's tets.
    pub mean_energy_density_j_per_m3: f64,
    /// Peak first-Piola Frobenius norm (Pa) across this layer's tets
    /// — the layer's stress-hotspot magnitude.
    pub max_first_piola_frobenius_pa: f64,
    /// Peak principal stretch across this layer's tets (any tet, any
    /// SVD singular value). Watches Yeoh's `max_principal_stretch`
    /// validity cap.
    pub max_principal_stretch: f64,
    /// Lowest principal stretch — mirrors `max` for Yeoh's
    /// `min_principal_stretch` floor.
    pub min_principal_stretch: f64,
}

/// Async-task outputs: the raw ramp + the panel's derived readouts +
/// the per-tet substrate that [`project_layer_heat_map`] needs to
/// re-project scalar fields onto each layer's SDF-extracted MC mesh
/// at render time (slice 9 sub-leaf 7).
pub struct InsertionSimOutputs {
    /// The raw ramp — for the F-d plot + per-step table.
    pub ramp: InsertionRamp,
    /// Per-layer reductions of `ramp.result.final_per_tet`.
    pub per_layer: Vec<LayerAggregate>,
    /// Rest-frame centroid of every tet in the insertion-sim mesh.
    /// Indexed by tet id (same indexing as `ramp.result.final_per_tet`
    /// + [`per_tet_layer`]).
    ///
    /// Used by [`project_layer_heat_map`] to find the nearest tet
    /// IN-LAYER for each MC-mesh vertex.
    ///
    /// [`per_tet_layer`]: InsertionSimOutputs::per_tet_layer
    pub tet_centroids: Vec<Vector3<f64>>,
    /// Layer index for each tet (0..n_layers). Indexed by tet id.
    /// Tets bucketed to the same layer the GUI's `LayersState`
    /// surfaces; used by [`project_layer_heat_map`] to filter the
    /// nearest-tet search to in-layer candidates only.
    pub per_tet_layer: Vec<usize>,
    /// Per-step per-tet scalar fields — outer index is converged-step
    /// index (matches `ramp.steps`), inner is
    /// `[Energy J/m³, Stress ‖P‖ Pa]` per [`ScalarMode::buffer_index`],
    /// each then per tet id. Slice S1: populated for every converged
    /// step so the panel's playback slider can re-color the heat map
    /// at any seating depth without re-running the ramp.
    pub per_step_scalar_fields: Vec<[Vec<f64>; 2]>,
    /// `[Energy, Stress]` (min, max) **across all converged steps**.
    /// Normalizes the gradient consistently as the user scrubs through
    /// playback — step 0 reads as "cold" and step N-1 as "hot" without
    /// the saturation jumping mid-scrub. Also reported alongside the
    /// heat map so the user can read absolute scale.
    pub scalar_min_max: [(f64, f64); 2],
    /// Slice S2 — BCC analysis-mesh boundary triangles, indexed into
    /// the per-step `x_final` vertex arrays. Snapshotted once at
    /// pipeline time so the panel's "Show deformed" cavity render can
    /// build a per-step `IndexedMesh` by pairing these triangles with
    /// `ramp.steps[displayed_step].x_final` — the same vertex layout
    /// the ramp solves on, just with displaced coordinates. Covers the
    /// full mesh boundary (both the inner cavity surface and the
    /// pinned outer skin); the outer skin is Dirichlet-pinned in the
    /// solve so it stays at its rest position, the cavity surface is
    /// what visibly bulges as the intruder seats.
    pub bcc_boundary_faces: Vec<[u32; 3]>,
    /// Slice S3 — per-layer OUTWARD face triangles for the deformed
    /// layer-shells render. Outer index is layer (innermost-first,
    /// matches `per_tet_layer`); inner triangle indices reference the
    /// same vertex layout as `x_final` for any step. Built once at
    /// pipeline time from the BCC tet adjacency: a face's "outward
    /// face of layer i" if it sits between layer i and layer i+1
    /// (inter-layer interface) OR it's on the global outer skin AND
    /// its incident tet is in the OUTERMOST layer (no further layer
    /// outward to claim it). Cavity-side global-boundary faces are
    /// excluded — the cavity render owns the cavity surface, not the
    /// layer-shells render.
    pub per_layer_outer_faces: Vec<Vec<[u32; 3]>>,
}

impl InsertionSimOutputs {
    /// Per-tet scalar fields at the requested converged step (slice S1
    /// playback). Clamps `step` to the last converged index — callers
    /// passing the panel's `displayed_step` get the final step's data
    /// when the field is at or past the converged tail.
    #[must_use]
    pub fn scalar_fields_at(&self, step: usize) -> &[Vec<f64>; 2] {
        let last = self.per_step_scalar_fields.len().saturating_sub(1);
        &self.per_step_scalar_fields[step.min(last)]
    }

    /// Slice S2 — build a deformed `IndexedMesh` for the cavity render
    /// at the requested converged step by pairing the BCC analysis
    /// mesh's snapshotted boundary triangles with the step's
    /// `x_final` displaced positions. Returns `None` if `step` is
    /// out-of-range (the ramp converged fewer steps than the caller
    /// expects, or the run has no steps at all).
    ///
    /// Cost: one allocation per call (~`n_vertices` `Point3` + a
    /// per-call clone of `bcc_boundary_faces`). Cheap enough to be
    /// rebuilt every time the panel slider moves; the system that
    /// consumes this is gated by `CavityMeshKey` change-detection so
    /// it only fires on actual input changes.
    #[must_use]
    pub fn deformed_boundary_mesh_at(&self, step: usize) -> Option<IndexedMesh> {
        let ramp_step = self.ramp.steps.get(step)?;
        let positions: Vec<Point3<f64>> = ramp_step
            .x_final
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let mut mesh = IndexedMesh::new();
        mesh.vertices = positions;
        mesh.faces = self.bcc_boundary_faces.clone();
        Some(mesh)
    }

    /// Slice S3 — build a deformed layer-shell `IndexedMesh` at the
    /// requested step for layer `layer_idx`. Pairs the layer's
    /// outer-face triangles with the step's `x_final`. Returns `None`
    /// if `layer_idx` exceeds the sim's layer count, the layer has no
    /// outer faces (e.g., degenerate single-cell layer), or `step` is
    /// out of the converged range.
    ///
    /// The mesh still carries every BCC vertex in its `.vertices`
    /// slot (only the layer's outer-face triangles reference a subset)
    /// — the unreferenced vertices are silently ignored by the bevy
    /// mesh build's `Mesh::compute_smooth_normals` consumer and the
    /// per-GPU-asset waste is small at iter-1 (≤ a few hundred KB).
    #[must_use]
    pub fn deformed_layer_mesh_at(&self, layer_idx: usize, step: usize) -> Option<IndexedMesh> {
        let faces = self.per_layer_outer_faces.get(layer_idx)?;
        if faces.is_empty() {
            return None;
        }
        let ramp_step = self.ramp.steps.get(step)?;
        let positions: Vec<Point3<f64>> = ramp_step
            .x_final
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let mut mesh = IndexedMesh::new();
        mesh.vertices = positions;
        mesh.faces = faces.clone();
        Some(mesh)
    }
}

impl fmt::Debug for InsertionSimOutputs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Same `dbg!`-footgun rationale as `InsertionGeometry` /
        // `InsertionRamp`: the per-tet centroid + scalar buffers are
        // large and would dominate any debug print.
        write!(
            f,
            "InsertionSimOutputs {{ n_steps_converged: {}, n_layers: {}, \
             n_tets: {}, ranges: {:?} }}",
            self.per_step_scalar_fields.len(),
            self.per_layer.len(),
            self.tet_centroids.len(),
            self.scalar_min_max,
        )
    }
}

/// Bevy resource owning the Insertion-Sim panel state.
///
/// Stale-data policy: any change to `CavityState` or `LayersState`
/// invalidates `last_run` (via [`invalidate_on_geometry_change`]). The
/// heat-map toggle is also auto-cleared; users re-run the sim after
/// editing the design.
#[derive(Resource)]
pub struct InsertionSimState {
    /// Most-recent completed run, or `None` if the panel has not been
    /// run yet or `[`invalidate_on_geometry_change`]` cleared it.
    pub last_run: Option<InsertionSimOutputs>,
    /// `Some(task)` between "Simulate" click and task completion;
    /// `None` otherwise. Mutually exclusive with a new click —
    /// [`kick_off_simulation`] no-ops when this is `Some`.
    pub pending: Option<Task<core::result::Result<InsertionSimOutputs, String>>>,
    /// User-clicked-Simulate signal, consumed once by
    /// [`kick_off_simulation`]. The button writes this; the system
    /// resets it on consumption.
    pub request_simulate: bool,
    /// Error message from the most recent failed run / launch, if any.
    /// Cleared on the next successful Simulate click.
    pub last_error: Option<String>,
    /// Number of ramp steps the next Simulate run will request.
    /// Always [`DEFAULT_N_STEPS`] today — not user-tunable.
    pub n_steps: usize,
    /// Whether the per-vertex heat-map mode is on. When `true` and
    /// `last_run.is_some()`, the layer-mesh update path writes
    /// vertex-color attributes from `last_run`. When `false`, layers
    /// render with their palette base colors (existing behavior).
    pub heat_map_on: bool,
    /// Which scalar to color by when `heat_map_on`. Toggle is free —
    /// both buffers are pre-computed by the async task.
    pub scalar_mode: ScalarMode,
    /// Monotonic counter incremented every time `last_run` changes
    /// in a way that should re-skin the layer meshes — i.e. a new
    /// run completed, or [`invalidate_on_geometry_change`] cleared
    /// the previous run. Read by `main.rs`'s `update_layer_meshes`
    /// (a `Local<>` snapshot) to detect "the per-tet color buffers
    /// changed" without relying on Bevy's `Res::is_changed()`, which
    /// fires every frame anyway because the panel + sim systems
    /// hold `ResMut<Self>` (the same change-detection footgun the
    /// [`invalidate_on_geometry_change`] snapshot-and-compare guards
    /// against). Wraps on overflow; the snapshot only needs strict
    /// inequality across consecutive frames.
    pub last_run_generation: u64,
    /// Slice S1 — index into `last_run.per_step_scalar_fields` for the
    /// heat-map playback slider. Set to `last_step_index` by
    /// [`poll_simulation_task`] when a run completes (default-shows the
    /// fully-seated state, matching pre-S1 behavior); reset to 0 by
    /// [`invalidate_on_geometry_change`]. Read by `main.rs`'s
    /// `LayerMeshKey` so a step change triggers a layer-mesh rebuild;
    /// also drives the viewport `Step N/M` badge.
    pub displayed_step: usize,
    /// Slice S2 — when `true`, the cavity entity renders the deformed
    /// BCC analysis-mesh boundary at `displayed_step` instead of the
    /// rest-frame SDF-extracted cavity surface. Set ON automatically
    /// when a run completes ("squish" view is the more interesting
    /// default when sim data is available); cleared on invalidation.
    /// Read by `main.rs::update_cavity_mesh` via `CavityMeshKey`.
    pub show_deformed: bool,
}

impl Default for InsertionSimState {
    fn default() -> Self {
        Self {
            last_run: None,
            pending: None,
            request_simulate: false,
            last_error: None,
            n_steps: DEFAULT_N_STEPS,
            heat_map_on: false,
            scalar_mode: ScalarMode::default(),
            last_run_generation: 0,
            displayed_step: 0,
            show_deformed: false,
        }
    }
}

/// Plugin that wires the Insertion-Sim panel into the app.
/// Installed by `main.rs` alongside the other panel plugins.
pub struct InsertionSimPlugin;

impl Plugin for InsertionSimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<InsertionSimState>()
            .add_systems(
                Update,
                (
                    invalidate_on_geometry_change,
                    kick_off_simulation,
                    poll_simulation_task,
                )
                    .chain(),
            )
            .add_systems(EguiPrimaryContextPass, render_step_badge);
    }
}

// ── ECS systems ─────────────────────────────────────────────────────

/// When the user edits the cavity or layer stack, the cached run is
/// stale — drop it + turn off the heat map so the viewport falls back
/// to the palette base colors. The user re-clicks Simulate after the
/// edit.
///
/// Slice-9 follow-up: this used to gate on
/// `cavity.is_changed() || layers.is_changed()`, but Bevy's
/// change-detection marks a `ResMut<T>` as changed on every `deref_mut`
/// access — not only on actual mutations. `device_design_panel` holds
/// `ResMut<CavityState>` + `ResMut<LayersState>` for the sliders, so
/// those resources were getting flagged changed every frame the panel
/// rendered, which cleared `last_run` immediately after every sim
/// completion (the user saw the F-d plot for at most one frame, then
/// it vanished). The fix here is the documented Bevy escape hatch:
/// **snapshot-and-compare** via a `Local<>`. We cache the last-seen
/// values, compare to current, and clear `last_run` only on a real
/// value change. This is the same posture
/// `bevy::ecs::change_detection`'s docs recommend for "I want
/// actual-mutation detection, not deref-mut detection."
#[allow(clippy::needless_pass_by_value)]
pub fn invalidate_on_geometry_change(
    cavity: Res<CavityState>,
    layers: Res<LayersState>,
    mut state: ResMut<InsertionSimState>,
    mut last_seen: Local<Option<(CavityState, LayersState)>>,
) {
    // `cavity` is `Copy`; `layers` carries a `Vec<LayerSpec>` so the
    // clone is genuine (≤ LAYER_COUNT_MAX = 6 items — negligible).
    let current_cavity: CavityState = *cavity;
    let current_layers: LayersState = layers.clone();
    let changed = match &*last_seen {
        Some((prev_cavity, prev_layers)) => {
            *prev_cavity != current_cavity || *prev_layers != current_layers
        }
        // First tick — no prior snapshot. Don't clear; this avoids a
        // spurious invalidation on the very first frame before
        // anything had a chance to run.
        None => false,
    };
    *last_seen = Some((current_cavity, current_layers));
    if changed && state.last_run.is_some() {
        state.last_run = None;
        state.heat_map_on = false;
        state.last_run_generation = state.last_run_generation.wrapping_add(1);
        state.displayed_step = 0;
        state.show_deformed = false;
    }
}

/// Spawn the async ramp when the user clicks Simulate. Idempotent on
/// `request_simulate` (resets the flag); no-ops while a previous task
/// is still pending. Clones the cleaned scan into the task (a
/// one-time chunky alloc — ~MB-scale — dwarfed by the 10-20 s ramp
/// itself).
#[allow(clippy::needless_pass_by_value)]
pub fn kick_off_simulation(
    scan: Option<Res<ScanMesh>>,
    cavity: Res<CavityState>,
    layers: Res<LayersState>,
    cap_planes: Res<CapPlanes>,
    mut state: ResMut<InsertionSimState>,
) {
    if !state.request_simulate {
        return;
    }
    state.request_simulate = false;
    if state.pending.is_some() {
        return;
    }
    let Some(scan) = scan else {
        state.last_error = Some("scan not loaded".into());
        return;
    };

    let scan_clone: IndexedMesh = scan.0.clone();
    let design = build_sim_design(&cavity, &layers);
    // Snapshot the cap planes for the async task — the resource may
    // change while the simulation runs (a new scan load) and the task
    // must consume the value as-it-was-at-kickoff.
    let cap_planes_clone: Vec<CapPlane> = cap_planes.planes.clone();
    let n_steps = state.n_steps;

    state.last_error = None;
    let pool = AsyncComputeTaskPool::get();
    let task = pool.spawn(async move {
        run_sim_pipeline(scan_clone, design, cap_planes_clone, n_steps)
            .map_err(|e| format!("{e:?}"))
    });
    state.pending = Some(task);
}

/// Each frame, non-blocking poll-once on the pending task; when ready,
/// move outputs into `last_run` (or `last_error`).
#[allow(clippy::needless_pass_by_value)]
pub fn poll_simulation_task(mut state: ResMut<InsertionSimState>) {
    let Some(mut task) = state.pending.take() else {
        return;
    };
    if let Some(result) = future::block_on(future::poll_once(&mut task)) {
        match result {
            Ok(outputs) => {
                // Slice S1 — default the playback slider to the
                // fully-seated final step so the post-completion view
                // matches pre-S1 behavior (the user's first sight of
                // the heat map is at full insertion).
                state.displayed_step = outputs.per_step_scalar_fields.len().saturating_sub(1);
                // Slice S2 — auto-enable the deformed-cavity view so
                // the user sees the squish immediately. The rest
                // cavity view stays one click away.
                state.show_deformed = true;
                state.last_run = Some(outputs);
                state.last_error = None;
                // Bump the generation so `update_layer_meshes` (in
                // `main.rs`) sees the new color buffers without
                // relying on Bevy's `Res::is_changed()` (which fires
                // every frame anyway — see `last_run_generation` doc).
                state.last_run_generation = state.last_run_generation.wrapping_add(1);
            }
            Err(e) => {
                state.last_error = Some(e);
            }
        }
    } else {
        // Still pending — put the task back for next frame.
        state.pending = Some(task);
    }
}

// ── input snapshots ─────────────────────────────────────────────────

/// Translate the panel's `(CavityState, LayersState)` into the
/// insertion-sim's [`SimDesign`]. Layer order is innermost-first
/// (matches both APIs).
fn build_sim_design(cavity: &CavityState, layers: &LayersState) -> SimDesign {
    let layers = layers
        .layers
        .iter()
        .map(|l| SimLayer {
            thickness_m: l.thickness_m,
            anchor_key: l.material_anchor_key.to_string(),
            // Slice 7.5: thread the per-layer Slacker fraction
            // through to the sim. The UI's `resolve_slacker_fraction`
            // already snaps off-curve values to the nearest TB
            // tabulated point (or `0.0` when the anchor's
            // `slacker::Support` flips to `NotRecommended` /
            // `NoData`), so what reaches the sim is always either
            // `0.0` or a known curve point.
            slacker_fraction: l.slacker_fraction,
        })
        .collect();
    SimDesign {
        cavity_inset_m: cavity.inset_m,
        layers,
    }
}

// ── async pipeline ──────────────────────────────────────────────────

/// The end-to-end sim pipeline that runs inside the async task: build
/// geometry → snapshot per-tet immutables → run ramp → compute
/// per-layer aggregates + the tet-level substrate the render-time
/// heat-map projection consumes. Pure compute; no Bevy / egui access
/// (the task pool runs off main thread).
fn run_sim_pipeline(
    scan: IndexedMesh,
    design: SimDesign,
    cap_planes: Vec<CapPlane>,
    n_steps: usize,
) -> Result<InsertionSimOutputs> {
    let geometry = build_insertion_geometry(
        &scan,
        &design,
        &cap_planes,
        SIM_SDF_TARGET_FACES,
        SIM_CELL_SIZE_M,
    )?;

    let n_tets = geometry.n_tets;
    let n_layers = design.layers.len();
    let per_tet_layer = geometry.per_tet_layer.clone();
    let rest_positions: Vec<Vector3<f64>> = geometry.mesh.positions().to_vec();
    // Slice S1 — snapshot the tet connectivity + per-tet materials
    // BEFORE `run_insertion_ramp` consumes `geometry`, so we can
    // recompute per-tet readouts at every converged step's
    // `x_final` for the playback slider. Same construction the ramp
    // itself uses internally (`insertion_sim.rs` ≈ line 1869); doing
    // it twice is cheap (≤ a few MB of clones) vs. the alternative
    // of pre-baking 16 × n_tets × 184 B readouts inside the ramp.
    // `t as TetId` (u32) — Phase 4 BCC meshes stay under `u32::MAX`.
    #[allow(clippy::cast_possible_truncation)]
    let tets: Vec<[VertexId; 4]> = (0..n_tets as TetId)
        .map(|t| geometry.mesh.tet_vertices(t))
        .collect();
    let materials: Vec<Yeoh> = geometry.mesh.materials().to_vec();
    // Slice S2 — snapshot the BCC analysis-mesh boundary triangles
    // BEFORE `run_insertion_ramp` consumes `geometry`. These index
    // into the same vertex array `step.x_final` carries (the ramp
    // doesn't add or remove vertices, only moves them), so a deformed
    // cavity render at any displayed step is `boundary_faces` paired
    // with that step's `x_final`. Cost: ~3 × n_boundary_faces × 4 B
    // — a few hundred KB at iter-1 size.
    let bcc_boundary_faces: Vec<[u32; 3]> = geometry.mesh.boundary_faces().to_vec();
    // Rest-frame tet centroids — the projection's nearest-tet lookup
    // operates on these. Precomputing here once avoids re-deriving
    // them at every heat-map render.
    let tet_centroids: Vec<Vector3<f64>> = tets
        .iter()
        .map(|v| {
            (rest_positions[v[0] as usize]
                + rest_positions[v[1] as usize]
                + rest_positions[v[2] as usize]
                + rest_positions[v[3] as usize])
                * 0.25
        })
        .collect();

    let ramp = run_insertion_ramp(geometry, n_steps)?;

    let result = ramp.result.as_ref().ok_or_else(|| {
        anyhow::anyhow!(
            "ramp failed at step 0 — no converged step. {}",
            ramp.failure_reason.as_deref().unwrap_or("(no reason)"),
        )
    })?;

    let per_layer = aggregate_per_layer(&result.final_per_tet, &per_tet_layer, n_layers);

    // Slice S1 — per-step scalar fields drive the playback slider.
    // The final step reuses `result.final_per_tet` (already computed
    // by the ramp); intermediate steps rerun `compute_tet_readouts`
    // from their own `x_final` against the snapshotted rest geometry +
    // tets + materials. Cost: ~15 × n_tets × per-tet F + Yeoh evals
    // — a few seconds at iter-1 size, dwarfed by the ramp itself.
    let last_step_idx = ramp.steps.len().saturating_sub(1);
    let per_step_scalar_fields: Vec<[Vec<f64>; 2]> = ramp
        .steps
        .iter()
        .enumerate()
        .map(|(k, step)| {
            let per_tet_owned;
            let per_tet: &[TetReadout] = if k == last_step_idx {
                &result.final_per_tet
            } else {
                let positions_k: Vec<Vector3<f64>> = step
                    .x_final
                    .chunks_exact(3)
                    .map(|c| Vector3::new(c[0], c[1], c[2]))
                    .collect();
                per_tet_owned =
                    compute_tet_readouts(&rest_positions, &positions_k, &tets, &materials);
                &per_tet_owned
            };
            let energy: Vec<f64> = per_tet.iter().map(|t| t.energy_density_j_per_m3).collect();
            let stress: Vec<f64> = per_tet.iter().map(|t| t.first_piola_frobenius_pa).collect();
            [energy, stress]
        })
        .collect();

    // Global (across all steps) min/max so the gradient stays
    // calibrated as the user scrubs through the playback — step 0
    // reads as cold and step N-1 as hot without per-step rescaling
    // jitter. The S1 spec calls this the natural "watch heat build
    // up" affordance over per-step renormalization.
    let energy_min_max = global_min_max_across(&per_step_scalar_fields, 0);
    let stress_min_max = global_min_max_across(&per_step_scalar_fields, 1);

    // Slice S3 — build per-layer outer-face triangle lists for the
    // deformed layer-shells render. Requires the outer-skin pinned
    // vertex set; detected from the final converged step's `x_final`
    // (zero displacement = Dirichlet-pinned). Done AFTER the ramp so
    // both `tets` snapshot and the final step's `x_final` are
    // available without re-running `outer_skin_bc`.
    let outer_skin_vertices = detect_outer_skin_vertices(&rest_positions, &ramp.final_x);
    let per_layer_outer_faces =
        build_per_layer_outer_faces(&tets, &per_tet_layer, &outer_skin_vertices, n_layers);

    Ok(InsertionSimOutputs {
        ramp,
        per_layer,
        tet_centroids,
        per_tet_layer,
        per_step_scalar_fields,
        scalar_min_max: [energy_min_max, stress_min_max],
        bcc_boundary_faces,
        per_layer_outer_faces,
    })
}

// ── per-layer aggregation ───────────────────────────────────────────

/// Reduce per-tet final readouts to per-layer scalar aggregates,
/// indexed by user layer order (innermost-first, length `n_layers`).
/// A layer with no tets (degenerate partition — should not happen in
/// production) gets `n_tets = 0` and zeros for the metrics.
pub fn aggregate_per_layer(
    per_tet: &[TetReadout],
    per_tet_layer: &[usize],
    n_layers: usize,
) -> Vec<LayerAggregate> {
    assert_eq!(
        per_tet.len(),
        per_tet_layer.len(),
        "aggregate_per_layer: per_tet.len() = {} must match per_tet_layer.len() = {}",
        per_tet.len(),
        per_tet_layer.len(),
    );
    let mut buckets: Vec<LayerAggregate> = (0..n_layers)
        .map(|i| LayerAggregate {
            layer_index: i,
            n_tets: 0,
            mean_energy_density_j_per_m3: 0.0,
            max_first_piola_frobenius_pa: 0.0,
            max_principal_stretch: f64::NEG_INFINITY,
            min_principal_stretch: f64::INFINITY,
        })
        .collect();
    let mut energy_sums = vec![0.0_f64; n_layers];
    for (t, readout) in per_tet.iter().enumerate() {
        let li = per_tet_layer[t];
        if li >= n_layers {
            continue; // defensive
        }
        let b = &mut buckets[li];
        b.n_tets += 1;
        energy_sums[li] += readout.energy_density_j_per_m3;
        if readout.first_piola_frobenius_pa > b.max_first_piola_frobenius_pa {
            b.max_first_piola_frobenius_pa = readout.first_piola_frobenius_pa;
        }
        for &s in readout.principal_stretches.iter() {
            if s > b.max_principal_stretch {
                b.max_principal_stretch = s;
            }
            if s < b.min_principal_stretch {
                b.min_principal_stretch = s;
            }
        }
    }
    for (b, sum) in buckets.iter_mut().zip(energy_sums.iter()) {
        if b.n_tets > 0 {
            // `n_tets` bounded by mesh tet count, well under f64 exact-int ceiling.
            #[allow(clippy::cast_precision_loss)]
            let n = b.n_tets as f64;
            b.mean_energy_density_j_per_m3 = sum / n;
        } else {
            b.max_principal_stretch = 0.0;
            b.min_principal_stretch = 0.0;
        }
    }
    buckets
}

// ── heat-map projection ─────────────────────────────────────────────

/// Global `(min, max)` over a per-tet scalar field. Used to normalize
/// the gradient — finite values only. Empty / all-non-finite collapses
/// to `(0, 0)`.
fn global_min_max(values: &[f64]) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &v in values {
        if v.is_finite() {
            if v < lo {
                lo = v;
            }
            if v > hi {
                hi = v;
            }
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        (0.0, 0.0)
    } else {
        (lo, hi)
    }
}

/// Slice S3 — partition the BCC analysis mesh's triangles into
/// per-layer OUTWARD faces. Two kinds of triangle qualify as layer i's
/// outer face:
///
/// 1. **Inter-layer interface.** A triangle shared by exactly two
///    tets `t_a`, `t_b` with `per_tet_layer[t_a] == i` and
///    `per_tet_layer[t_b] == i + 1` (or vice-versa): the triangle is
///    layer `min`'s outer face (and layer `max`'s inner face, which
///    we don't render — the outer-face of the inner layer is enough
///    to depict the boundary).
/// 2. **Outermost global-boundary face.** A triangle on the global
///    BCC boundary (only one incident tet `t`) with
///    `per_tet_layer[t] == n_layers - 1` AND all 3 vertices in
///    `outer_skin_vertices`: layer N-1's outer skin. Cavity-side
///    global-boundary triangles (layer 0, vertices not pinned) are
///    excluded — the cavity render owns them.
///
/// Winding follows the first tet to register the face (any of the 4
/// vertex-triples extracted in canonical order); the per-layer-shell
/// material is double-sided so winding doesn't affect visibility.
///
/// Cost: O(n_tets × 4 × log n_faces) ≈ 5 M ops at iter-1 (~73 k tets);
/// runs once at sim completion inside the async pipeline.
fn build_per_layer_outer_faces(
    tets: &[[VertexId; 4]],
    per_tet_layer: &[usize],
    outer_skin_vertices: &std::collections::BTreeSet<VertexId>,
    n_layers: usize,
) -> Vec<Vec<[u32; 3]>> {
    if n_layers == 0 {
        return Vec::new();
    }
    // Sorted-key → (first-encountered-face winding, list of incident tet ids).
    let mut face_map: BTreeMap<[u32; 3], ([u32; 3], Vec<usize>)> = BTreeMap::new();
    for (t, verts) in tets.iter().enumerate() {
        let faces = [
            [verts[1], verts[2], verts[3]],
            [verts[0], verts[2], verts[3]],
            [verts[0], verts[1], verts[3]],
            [verts[0], verts[1], verts[2]],
        ];
        for f in faces {
            let mut sorted = f;
            sorted.sort_unstable();
            face_map
                .entry(sorted)
                .or_insert_with(|| (f, Vec::new()))
                .1
                .push(t);
        }
    }
    let mut per_layer: Vec<Vec<[u32; 3]>> = vec![Vec::new(); n_layers];
    let outermost = n_layers - 1;
    for (_sorted, (face, tet_ids)) in face_map {
        match tet_ids.as_slice() {
            [t_a, t_b] => {
                let l_a = per_tet_layer[*t_a];
                let l_b = per_tet_layer[*t_b];
                if l_a == l_b {
                    continue; // interior to a single layer
                }
                let lower = l_a.min(l_b);
                if lower < n_layers {
                    per_layer[lower].push(face);
                }
            }
            [t] => {
                if per_tet_layer[*t] != outermost {
                    continue; // inner cavity surface — not a layer's outer
                }
                if !face.iter().all(|v| outer_skin_vertices.contains(v)) {
                    continue; // cap-plane side face or other non-skin boundary
                }
                per_layer[outermost].push(face);
            }
            _ => {
                // 0 or >2 incident tets in a tet mesh: malformed; skip
                // defensively. Production BCC meshes are 2-manifold so
                // this branch should be unreachable.
            }
        }
    }
    per_layer
}

/// Slice S3 — detect the outer-skin pinned vertex set from the final
/// converged step's `x_final`. Outer-skin vertices are Dirichlet-
/// pinned (zero displacement); other boundary vertices move under
/// intruder contact. The strict-equality check is robust because
/// pinned vertices are written verbatim from rest by the solver (no
/// arithmetic applied), so even a sub-µm cavity displacement
/// distinguishes them from outer-skin verts.
///
/// Falls back to an empty set when no step converged or
/// `result.final_per_tet` is absent — callers must tolerate an empty
/// pinned set (no outer-skin face passes the membership check, so
/// layer N-1 ends up with an empty outer face list, which
/// `deformed_layer_mesh_at` reports as `None` → falls through to the
/// rest-frame SDF iso).
fn detect_outer_skin_vertices(
    rest_positions: &[Vector3<f64>],
    final_x: &[f64],
) -> std::collections::BTreeSet<VertexId> {
    if final_x.len() != 3 * rest_positions.len() {
        return std::collections::BTreeSet::new();
    }
    let mut pinned = std::collections::BTreeSet::new();
    for (v_idx, rest) in rest_positions.iter().enumerate() {
        let offset = 3 * v_idx;
        if final_x[offset] == rest.x
            && final_x[offset + 1] == rest.y
            && final_x[offset + 2] == rest.z
        {
            // `v_idx` is bounded by `n_vertices` (Phase 4 ≤ u32::MAX).
            #[allow(clippy::cast_possible_truncation)]
            pinned.insert(v_idx as VertexId);
        }
    }
    pinned
}

/// Slice S1 — `(min, max)` across every step's slot-`slot_idx` per-tet
/// scalar field. Used to calibrate the heat-map gradient consistently
/// across the playback slider so a scrub doesn't jump the color scale.
/// Defaults to `(0, 0)` on empty input (matches [`global_min_max`]).
fn global_min_max_across(per_step_fields: &[[Vec<f64>; 2]], slot_idx: usize) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for fields in per_step_fields {
        let (step_lo, step_hi) = global_min_max(&fields[slot_idx]);
        if step_lo < lo {
            lo = step_lo;
        }
        if step_hi > hi {
            hi = step_hi;
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        (0.0, 0.0)
    } else {
        (lo, hi)
    }
}

/// Project the sim's per-tet scalar field onto an SDF-extracted
/// layer mesh: for each MC vertex, find the nearest in-layer tet
/// centroid, sample
/// [`InsertionSimOutputs::scalar_fields_at`]`(step)` for the
/// requested mode, encode RGBA via [`scalar_to_rgba`].
///
/// `mc_vertices` should be the per-layer surface's MC vertex
/// positions in physics-frame meters (the same ones
/// `build_bevy_mesh_from_indexed_with_colors` consumes; cf.
/// `update_layer_meshes` in main.rs).
///
/// Returns `None` when:
/// - `layer_idx` exceeds the sim's layer count (sim was run with
///   fewer layers than the GUI now shows — the heat map for the
///   extra layers has no data),
/// - or the requested layer has no tets in its partition (degenerate
///   geometry that the GUI surfaces elsewhere).
///
/// Otherwise returns one RGBA per `mc_vertices` entry.
///
/// Cost: O(`mc_vertices.len()` × `n_tets_in_layer`). On iter-1 with
/// ~3 k MC verts per layer × ~5 k tets per layer ≈ 15 M ops per
/// rebuild; release-mode cost ~10 ms per layer. Re-walks per
/// `update_layer_meshes` rebuild (which fires on slider change OR
/// sim completion). The per-layer-vertex → nearest-tet cache the
/// spec calls out is a future optimization if scalar-mode toggle
/// ever feels laggy — drop in a `HashMap<(layer_idx, vertex_idx),
/// tet_idx>` keyed on the same `LayerMeshKey` that gates the
/// rebuild.
#[must_use]
pub(crate) fn project_layer_heat_map(
    outputs: &InsertionSimOutputs,
    layer_idx: usize,
    scalar_mode: ScalarMode,
    step: usize,
    mc_vertices: &[Point3<f64>],
) -> Option<Vec<[f32; 4]>> {
    let n_layers = outputs
        .per_tet_layer
        .iter()
        .copied()
        .max()
        .map(|m| m + 1)
        .unwrap_or(0);
    if layer_idx >= n_layers {
        return None;
    }
    let layer_tets: Vec<usize> = outputs
        .per_tet_layer
        .iter()
        .enumerate()
        .filter_map(|(t, &l)| (l == layer_idx).then_some(t))
        .collect();
    if layer_tets.is_empty() {
        return None;
    }
    let mode = scalar_mode.buffer_index();
    let scalar_field = &outputs.scalar_fields_at(step)[mode];
    let min_max = outputs.scalar_min_max[mode];
    let centroids = &outputs.tet_centroids;

    let colors: Vec<[f32; 4]> = mc_vertices
        .iter()
        .map(|v| {
            let mut best_t = layer_tets[0];
            let mut best_d2 = (centroids[best_t] - v.coords).norm_squared();
            for &t in &layer_tets[1..] {
                let d2 = (centroids[t] - v.coords).norm_squared();
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_t = t;
                }
            }
            scalar_to_rgba(scalar_field[best_t], min_max)
        })
        .collect();
    Some(colors)
}

/// Map a scalar to an RGBA color in `[0, 1]` via a three-stop gradient
/// (cold blue → warm yellow → hot red). Engineering-readable and
/// avoids the matplotlib-style perceptually-uniform colormap's external
/// dep. Out-of-range or non-finite values clamp to the endpoints.
fn scalar_to_rgba(value: f64, (lo, hi): (f64, f64)) -> [f32; 4] {
    if !value.is_finite() {
        return [0.5, 0.5, 0.5, 1.0]; // grey for NaN/inf
    }
    let span = hi - lo;
    let t = if span > 0.0 {
        ((value - lo) / span).clamp(0.0, 1.0)
    } else {
        0.5
    };
    // Three-stop gradient: t ∈ [0, 0.5] = blue → yellow;
    // t ∈ [0.5, 1.0] = yellow → red. Pinned colors:
    //   cold = (0.10, 0.30, 0.85)  — sky/cobalt
    //   mid  = (0.95, 0.85, 0.15)  — sodium-vapor yellow
    //   hot  = (0.85, 0.10, 0.10)  — fire red
    let cold = [0.10_f32, 0.30, 0.85];
    let mid = [0.95_f32, 0.85, 0.15];
    let hot = [0.85_f32, 0.10, 0.10];
    let lerp = |a: [f32; 3], b: [f32; 3], s: f32| -> [f32; 3] {
        [
            a[0] + (b[0] - a[0]) * s,
            a[1] + (b[1] - a[1]) * s,
            a[2] + (b[2] - a[2]) * s,
        ]
    };
    // `t` is in [0, 1] (clamped above); the `as f32` cast can't lose
    // significant precision for that range, and 0.5/1.0 are exact in f32.
    #[allow(clippy::cast_possible_truncation)]
    let t32 = t as f32;
    let rgb = if t32 < 0.5 {
        lerp(cold, mid, t32 * 2.0)
    } else {
        lerp(mid, hot, (t32 - 0.5) * 2.0)
    };
    [rgb[0], rgb[1], rgb[2], 1.0]
}

// ── panel rendering ─────────────────────────────────────────────────

/// Render the Insertion-Sim section into the parent `device_design_panel`.
/// Reads + mutates `InsertionSimState`; surfaces F-d plot, per-step
/// table, per-layer table, and heat-map controls.
pub fn render_insertion_sim_section(
    ui: &mut egui::Ui,
    state: &mut InsertionSimState,
    scan_loaded: bool,
) {
    egui::CollapsingHeader::new("Insertion Sim")
        .default_open(true)
        .show(ui, |ui| {
            let busy = state.pending.is_some();
            let can_simulate = scan_loaded && !busy;
            ui.add_enabled_ui(can_simulate, |ui| {
                let label = if busy {
                    "Simulating…"
                } else {
                    "▶  Simulate Insertion"
                };
                if ui.button(label).clicked() {
                    state.request_simulate = true;
                }
            });
            ui.label(format!(
                "{} steps · 4 mm cell · Yeoh material · tol=1e-1 · κ=1e3",
                state.n_steps
            ));

            if let Some(err) = &state.last_error {
                ui.colored_label(
                    egui::Color32::from_rgb(225, 90, 80),
                    format!("Error: {err}"),
                );
            }

            let Some(run) = &state.last_run else {
                if !scan_loaded {
                    ui.label("(load a scan first)");
                }
                return;
            };

            ui.separator();
            render_convergence_summary(ui, &run.ramp, state.n_steps);

            render_force_displacement_plot(ui, &run.ramp);
            render_step_table(ui, &run.ramp);
            render_layer_table(ui, &run.per_layer);

            ui.separator();
            render_playback_slider(ui, &run.ramp, &mut state.displayed_step);
            ui.checkbox(&mut state.show_deformed, "Show deformed cavity (vs rest)");
            ui.checkbox(&mut state.heat_map_on, "Heat map");
            ui.add_enabled_ui(state.heat_map_on, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Scalar:");
                    ui.radio_value(
                        &mut state.scalar_mode,
                        ScalarMode::EnergyDensity,
                        ScalarMode::EnergyDensity.label(),
                    );
                    ui.radio_value(
                        &mut state.scalar_mode,
                        ScalarMode::StressFrobenius,
                        ScalarMode::StressFrobenius.label(),
                    );
                });
                let (lo, hi) = run.scalar_min_max[state.scalar_mode.buffer_index()];
                // `..` (Rust range idiom) rather than `→` (U+2192) —
                // egui's default proportional font in bevy_egui 0.39
                // doesn't carry U+2192 and the workshop user sees a
                // missing-glyph tofu box. `..` is universally
                // supported and reads naturally as a range.
                ui.label(format!("range: {lo:.3e} .. {hi:.3e}"));
            });
        });
}

/// Slice S1 — per-step playback slider. Shown only when the ramp
/// converged at least 2 steps (a 1-step run has nothing to scrub).
/// Range is `[0, last_converged_step]`; the headline label reads
/// `Step N / 16 — d = X.XX mm` so the scrub state is legible without
/// looking at the viewport badge.
fn render_playback_slider(ui: &mut egui::Ui, ramp: &InsertionRamp, displayed_step: &mut usize) {
    let n_steps = ramp.steps.len();
    if n_steps <= 1 {
        return;
    }
    let last = n_steps - 1;
    if *displayed_step > last {
        *displayed_step = last;
    }
    let interference_mm = ramp.steps[*displayed_step].interference_m * 1e3;
    ui.label(format!(
        "Step {} / {n_steps} — d = {interference_mm:.2} mm",
        *displayed_step + 1,
    ));
    ui.add(
        egui::Slider::new(displayed_step, 0..=last)
            .integer()
            .show_value(false),
    );
}

/// Slice S1 — viewport `Step N/M` badge. Anchored top-left so the
/// workshop user sees the current playback frame without glancing at
/// the side panel. Only shown when a converged run with ≥ 2 steps is
/// loaded.
#[allow(clippy::needless_pass_by_value)]
pub fn render_step_badge(
    mut contexts: EguiContexts,
    state: Res<InsertionSimState>,
) -> bevy::ecs::error::Result {
    let Some(run) = state.last_run.as_ref() else {
        return Ok(());
    };
    let n_steps = run.ramp.steps.len();
    if n_steps <= 1 {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    let current = state.displayed_step.min(n_steps - 1) + 1;
    egui::Area::new(egui::Id::new("insertion_sim_step_badge"))
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(16.0, 16.0))
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.label(egui::RichText::new(format!("Step {current} / {n_steps}")).monospace());
            });
        });
    Ok(())
}

/// One-line convergence summary above the F-d plot. On full
/// convergence (`failed_at_step.is_none()`) this is a plain
/// neutral-color line; on partial convergence (a stall) it surfaces
/// the **seated depth in mm** (not just the step index — the
/// workshop user reads mm fluently and step indices not so much) in
/// amber, plus a collapsing "Stall details" expander carrying the
/// solver's `failure_reason` string for the engineering-curious.
///
/// Replaces the original `"stalled at step {k}"` readout — visual-
/// pass finding 6 on the slice-7-9 PR: that phrasing read like a
/// problem the user had to solve, but it's the slice-7 plan's
/// documented "partial-but-honest" Fork-B behavior. The new copy
/// reframes it: "seated to X of Y mm" tells the user what depth
/// the sim *did* reach (which IS engineering-actionable data), not
/// what depth it failed at.
fn render_convergence_summary(ui: &mut egui::Ui, ramp: &InsertionRamp, n_steps: usize) {
    // The ramp's per-step `interference_m` is evenly spaced from
    // `cavity_inset_m / n_steps` (step 0) to `cavity_inset_m` (the
    // would-be step `n_steps - 1`). Reconstruct `cavity_inset_m`
    // from the first step's interference; this works because the
    // ramp is consumed by the time we render (`InsertionGeometry`
    // is gone) but `steps[0]` is guaranteed non-empty here — the
    // upstream `run_sim_pipeline` errors with "no converged step"
    // when `steps.is_empty()`, so by the time we have a
    // `last_run`, `steps.len() >= 1`.
    let target_mm = ramp
        .steps
        .first()
        // `n_steps as f64` is bounded by `DEFAULT_N_STEPS = 16`,
        // far under `f64`'s exact-integer ceiling.
        .map_or(0.0, |s| {
            #[allow(clippy::cast_precision_loss)]
            let n = n_steps as f64;
            s.interference_m * n * 1e3
        });
    let seated_mm = ramp.steps.last().map_or(0.0, |s| s.interference_m * 1e3);
    match ramp.failed_at_step {
        None => {
            ui.label(format!(
                "Converged: {} of {} steps — seated to {target_mm:.2} mm (full depth)",
                ramp.steps.len(),
                n_steps,
            ));
        }
        Some(_) => {
            ui.colored_label(
                egui::Color32::from_rgb(225, 180, 90),
                format!(
                    "Converged: {} of {} steps — seated to {seated_mm:.2} of {target_mm:.2} mm",
                    ramp.steps.len(),
                    n_steps,
                ),
            );
            if let Some(reason) = &ramp.failure_reason {
                egui::CollapsingHeader::new("Stall details")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label(
                            "The Insertion Sim is a relative-comparison aid (Fork B); \
                             the F-d curve through the converged depth is real \
                             engineering data. The stall is the solver-robustness \
                             envelope at this configuration — typically deeper \
                             seating needs a thicker wall or a smaller cavity inset.",
                        );
                        ui.label("Solver reason:");
                        ui.monospace(reason);
                    });
            }
        }
    }
}

/// Hand-rolled F-d plot via `egui::Painter`. Avoids pulling in
/// `egui_plot` for a single small line plot. Axes: x = interference
/// (mm), y = contact force magnitude (N). 16 dots (one per converged
/// step) connected by a sky-blue polyline; axis labels render at the
/// top-left, bottom-left, and bottom-right of the plot rect.
fn render_force_displacement_plot(ui: &mut egui::Ui, ramp: &InsertionRamp) {
    if ramp.steps.is_empty() {
        return;
    }
    let curve: Vec<(f32, f32)> = ramp
        .steps
        .iter()
        .map(|s| {
            // f64 → f32 for screen coords: mm + N values are tiny.
            #[allow(clippy::cast_possible_truncation)]
            let x = (s.interference_m * 1e3) as f32;
            #[allow(clippy::cast_possible_truncation)]
            let y = s.readout.contact_force_magnitude_n as f32;
            (x, y)
        })
        .collect();
    let x_max = curve.iter().map(|(x, _)| *x).fold(0.0_f32, f32::max);
    let y_max = curve.iter().map(|(_, y)| *y).fold(0.0_f32, f32::max);
    if x_max <= 0.0 || y_max <= 0.0 {
        return; // degenerate — nothing to plot
    }

    let width = ui.available_width().min(280.0);
    let height = 110.0_f32;
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, egui::Color32::from_gray(28));

    // Left pad fits the Y-axis label `{y_max:.1} N` — "10.0 N" is
    // ~38 px at 9 pt monospace; "100.0 N" is ~46. Bumped from the
    // original 28 (which clipped "10.0 N" → ".0 N" — visual-pass
    // finding 2 on the slice-7-9 PR review). Bottom pad stays at
    // `pad` for the x-axis label.
    let pad_left = 44.0_f32;
    let pad_bottom = 28.0_f32;
    let plot_rect = egui::Rect::from_min_size(
        rect.min + egui::vec2(pad_left, 8.0),
        egui::vec2(rect.width() - pad_left - 8.0, rect.height() - pad_bottom),
    );
    // axes
    painter.line_segment(
        [plot_rect.left_bottom(), plot_rect.right_bottom()],
        egui::Stroke::new(1.0, egui::Color32::from_gray(120)),
    );
    painter.line_segment(
        [plot_rect.left_bottom(), plot_rect.left_top()],
        egui::Stroke::new(1.0, egui::Color32::from_gray(120)),
    );
    // axis labels
    let label_color = egui::Color32::from_gray(180);
    let font = egui::FontId::monospace(9.0);
    painter.text(
        plot_rect.left_bottom() + egui::vec2(-4.0, -4.0),
        egui::Align2::RIGHT_BOTTOM,
        "0",
        font.clone(),
        label_color,
    );
    painter.text(
        plot_rect.left_top() + egui::vec2(-4.0, 0.0),
        egui::Align2::RIGHT_TOP,
        format!("{y_max:.1} N"),
        font.clone(),
        label_color,
    );
    painter.text(
        plot_rect.right_bottom() + egui::vec2(0.0, 2.0),
        egui::Align2::RIGHT_TOP,
        format!("{x_max:.1} mm"),
        font,
        label_color,
    );

    // plot line
    let to_screen = |x: f32, y: f32| -> egui::Pos2 {
        let nx = x / x_max;
        let ny = y / y_max;
        egui::pos2(
            plot_rect.left() + nx * plot_rect.width(),
            plot_rect.bottom() - ny * plot_rect.height(),
        )
    };
    let stroke = egui::Stroke::new(1.5, egui::Color32::from_rgb(110, 200, 240));
    for pair in curve.windows(2) {
        let a = to_screen(pair[0].0, pair[0].1);
        let b = to_screen(pair[1].0, pair[1].1);
        painter.line_segment([a, b], stroke);
    }
    // sample dots
    let dot_color = egui::Color32::from_rgb(220, 230, 240);
    for (x, y) in &curve {
        painter.circle_filled(to_screen(*x, *y), 1.8, dot_color);
    }
}

/// Per-step scalar table — one row per converged step. Compact; uses
/// `egui::Grid` for column alignment.
fn render_step_table(ui: &mut egui::Ui, ramp: &InsertionRamp) {
    if ramp.steps.is_empty() {
        return;
    }
    egui::CollapsingHeader::new(format!("Per-step ({} rows)", ramp.steps.len()))
        .default_open(false)
        .show(ui, |ui| {
            egui::Grid::new("insertion_sim_step_table")
                .num_columns(6)
                .spacing([8.0, 2.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("d");
                    ui.label("iter");
                    ui.label("res");
                    ui.label("F");
                    ui.label("λ");
                    ui.label("‖P‖");
                    ui.end_row();
                    for step in &ramp.steps {
                        render_step_row(ui, step);
                    }
                });
        });
}

fn render_step_row(ui: &mut egui::Ui, step: &RampStep) {
    let r: &StepReadout = &step.readout;
    ui.label(format!("{:.2}", step.interference_m * 1e3));
    ui.label(format!("{}", step.iter_count));
    ui.label(format!("{:.1e}", step.final_residual_norm));
    ui.label(format!("{:.2}", r.contact_force_magnitude_n));
    ui.label(format!(
        "{:.2}…{:.2}",
        r.min_principal_stretch, r.max_principal_stretch
    ));
    ui.label(format!("{:.1e}", r.max_first_piola_frobenius_pa));
    ui.end_row();
}

/// Per-layer aggregates table — one row per layer, innermost-first.
fn render_layer_table(ui: &mut egui::Ui, per_layer: &[LayerAggregate]) {
    if per_layer.is_empty() {
        return;
    }
    egui::CollapsingHeader::new("Per-layer (mean Ψ · peak ‖P‖ · λ range)")
        .default_open(true)
        .show(ui, |ui| {
            egui::Grid::new("insertion_sim_layer_table")
                .num_columns(5)
                .spacing([8.0, 2.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Layer");
                    ui.label("Tets");
                    ui.label("mean Ψ");
                    ui.label("max ‖P‖");
                    ui.label("λ range");
                    ui.end_row();
                    for a in per_layer {
                        let (r, g, b) =
                            LAYER_SURFACE_PALETTE[a.layer_index % LAYER_SURFACE_PALETTE.len()];
                        let swatch = egui::Color32::from_rgb(
                            (r * 255.0) as u8,
                            (g * 255.0) as u8,
                            (b * 255.0) as u8,
                        );
                        ui.horizontal(|ui| {
                            let _ = ui
                                .allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                            ui.painter().circle_filled(
                                ui.min_rect().left_center() + egui::vec2(5.0, 0.0),
                                4.0,
                                swatch,
                            );
                            ui.label(format!("{}", a.layer_index));
                        });
                        ui.label(format!("{}", a.n_tets));
                        ui.label(format!("{:.2e}", a.mean_energy_density_j_per_m3));
                        ui.label(format!("{:.2e}", a.max_first_piola_frobenius_pa));
                        ui.label(format!(
                            "{:.2}…{:.2}",
                            a.min_principal_stretch, a.max_principal_stretch
                        ));
                        ui.end_row();
                    }
                });
        });
}

// ── tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::insertion_sim::TetReadout;

    fn mk_readout(stretches: [f64; 3], frob: f64, psi: f64) -> TetReadout {
        TetReadout {
            f: Matrix3::<f64>::identity(),
            first_piola: Matrix3::<f64>::identity(),
            first_piola_frobenius_pa: frob,
            energy_density_j_per_m3: psi,
            principal_stretches: Vector3::new(stretches[0], stretches[1], stretches[2]),
        }
    }

    /// `aggregate_per_layer` buckets tets by layer index, sums energy
    /// (averaged per layer), tracks per-layer max ‖P‖ + λ extrema.
    #[test]
    fn aggregate_per_layer_buckets_by_layer_index() {
        let per_tet = vec![
            mk_readout([1.2, 1.0, 0.9], 1.0e5, 2.0),
            mk_readout([1.1, 1.0, 1.0], 0.5e5, 3.0),
            mk_readout([1.4, 0.7, 1.0], 3.0e5, 7.0),
            mk_readout([1.0, 1.0, 1.0], 0.1e5, 0.0),
        ];
        let per_tet_layer = vec![0, 0, 1, 1];
        let per_layer = aggregate_per_layer(&per_tet, &per_tet_layer, 2);
        assert_eq!(per_layer.len(), 2);
        // Layer 0: tets 0+1
        assert_eq!(per_layer[0].n_tets, 2);
        assert!((per_layer[0].mean_energy_density_j_per_m3 - 2.5).abs() < 1e-12);
        assert!((per_layer[0].max_first_piola_frobenius_pa - 1.0e5).abs() < 1e-9);
        assert!((per_layer[0].max_principal_stretch - 1.2).abs() < 1e-12);
        assert!((per_layer[0].min_principal_stretch - 0.9).abs() < 1e-12);
        // Layer 1: tets 2+3
        assert_eq!(per_layer[1].n_tets, 2);
        assert!((per_layer[1].mean_energy_density_j_per_m3 - 3.5).abs() < 1e-12);
        assert!((per_layer[1].max_first_piola_frobenius_pa - 3.0e5).abs() < 1e-9);
        assert!((per_layer[1].max_principal_stretch - 1.4).abs() < 1e-12);
        assert!((per_layer[1].min_principal_stretch - 0.7).abs() < 1e-12);
    }

    /// Empty layer partition collapses to zeros — defensive degenerate case.
    #[test]
    fn aggregate_per_layer_empty_layer_is_zeroed() {
        let per_tet = vec![mk_readout([1.0, 1.0, 1.0], 1.0e3, 1.0)];
        let per_tet_layer = vec![0];
        let per_layer = aggregate_per_layer(&per_tet, &per_tet_layer, 2);
        assert_eq!(per_layer[1].n_tets, 0);
        assert_eq!(per_layer[1].mean_energy_density_j_per_m3, 0.0);
        assert_eq!(per_layer[1].max_first_piola_frobenius_pa, 0.0);
        assert_eq!(per_layer[1].max_principal_stretch, 0.0);
        assert_eq!(per_layer[1].min_principal_stretch, 0.0);
    }

    /// `scalar_to_rgba` clamps out-of-range and degenerate inputs.
    #[test]
    fn scalar_to_rgba_clamps_and_handles_degenerate() {
        // Below range
        let cold = scalar_to_rgba(0.0, (1.0, 5.0));
        assert!(
            cold[2] > cold[0],
            "below range should clamp to cold (B > R)"
        );
        // Above range
        let hot = scalar_to_rgba(10.0, (1.0, 5.0));
        assert!(hot[0] > hot[2], "above range should clamp to hot (R > B)");
        // Degenerate range
        let degenerate = scalar_to_rgba(3.0, (3.0, 3.0));
        assert!(
            (degenerate[0] - 0.95).abs() < 0.5,
            "degenerate range → midpoint color"
        );
        // NaN
        let nan = scalar_to_rgba(f64::NAN, (0.0, 1.0));
        assert_eq!(nan, [0.5, 0.5, 0.5, 1.0]);
    }

    /// `global_min_max` ignores non-finite values + degenerates to
    /// `(0, 0)` on empty / all-non-finite inputs.
    #[test]
    fn global_min_max_handles_edge_cases() {
        assert_eq!(global_min_max(&[]), (0.0, 0.0));
        assert_eq!(global_min_max(&[f64::NAN, f64::INFINITY]), (0.0, 0.0));
        assert_eq!(global_min_max(&[1.0, 2.0, f64::NAN, 0.5]), (0.5, 2.0));
    }

    /// `ScalarMode::buffer_index` partitions into 2 distinct slots —
    /// the `per_layer_vertex_colors` array length cap.
    #[test]
    fn scalar_mode_buffer_indices_cover_array() {
        let indices: Vec<usize> = [ScalarMode::EnergyDensity, ScalarMode::StressFrobenius]
            .iter()
            .map(|m| m.buffer_index())
            .collect();
        assert_eq!(indices, vec![0, 1]);
    }

    /// Slice S3 — `build_per_layer_outer_faces` assigns inter-layer
    /// interface triangles to the LOWER layer's outer face + claims
    /// outermost global-boundary triangles for the OUTERMOST layer.
    /// 2-tet fixture: tet 0 in layer 0, tet 1 in layer 1, sharing
    /// face [v0, v1, v2]. Layer 0's outer should be that shared face;
    /// layer 1's outer should be the OUTERMOST-skin face (the face
    /// opposite v_shared in tet 1, with all 3 verts pinned).
    #[test]
    fn build_per_layer_outer_faces_assigns_interface_and_outermost_skin() {
        use std::collections::BTreeSet;
        // Tet 0 = [0, 1, 2, 3] in layer 0; tet 1 = [1, 2, 3, 4] in
        // layer 1. They share the face {1, 2, 3} (the "interface").
        // Tet 1's other faces are {1, 2, 4}, {1, 3, 4}, {2, 3, 4}; if
        // their vertices include 4 (the outermost vertex), they are
        // candidates for layer 1's outer skin.
        let tets: Vec<[VertexId; 4]> = vec![[0, 1, 2, 3], [1, 2, 3, 4]];
        let per_tet_layer: Vec<usize> = vec![0, 1];
        // Vertex 4 is the only "outermost" vertex; pin it (plus its
        // 3 neighbors to make a 3-vertex face all-pinned).
        let pinned: BTreeSet<VertexId> = [1, 2, 3, 4].into_iter().collect();

        let per_layer = build_per_layer_outer_faces(&tets, &per_tet_layer, &pinned, 2);

        // Layer 0's outer face is the shared interface (sorted: 1, 2, 3).
        assert_eq!(per_layer[0].len(), 1, "layer 0 should have 1 outer face");
        let mut layer0_face = per_layer[0][0];
        layer0_face.sort_unstable();
        assert_eq!(layer0_face, [1, 2, 3]);

        // Layer 1's outer skin faces — every tet-1 face EXCEPT the
        // shared interface (which is interior to the two-tet pair):
        // {1, 2, 4}, {1, 3, 4}, {2, 3, 4}. All 3 candidates have
        // every vertex pinned per the fixture, so all 3 land in
        // layer 1's outer.
        assert_eq!(
            per_layer[1].len(),
            3,
            "layer 1 should pick up 3 outer-skin faces"
        );
    }

    /// Slice S3 — `detect_outer_skin_vertices` flags vertices whose
    /// final-step `x_final` matches rest exactly (Dirichlet-pinned)
    /// and skips ones that moved (cavity-side, displaced by intruder
    /// contact).
    #[test]
    fn detect_outer_skin_vertices_flags_only_zero_displacement() {
        let rest = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        // v0: pinned (matches rest). v1: moved. v2: pinned.
        let final_x = vec![
            0.0, 0.0, 0.0, // v0 pinned
            1.001, 0.0, 0.0, // v1 displaced
            0.0, 1.0, 0.0, // v2 pinned
        ];
        let pinned = detect_outer_skin_vertices(&rest, &final_x);
        assert!(pinned.contains(&0));
        assert!(!pinned.contains(&1));
        assert!(pinned.contains(&2));
    }

    /// Slice S2 — `deformed_boundary_mesh_at` pairs the snapshotted
    /// BCC boundary triangles with the requested step's `x_final`,
    /// producing an `IndexedMesh` ready to feed `build_bevy_mesh`. OOB
    /// step indices return `None` (no clamp — the caller should
    /// not silently shift onto a different step).
    #[test]
    fn deformed_boundary_mesh_at_pairs_step_positions_with_snapshot_faces() {
        // Two converged steps, 3 vertices, 1 boundary face.
        let outputs = InsertionSimOutputs {
            ramp: InsertionRamp {
                steps: vec![
                    RampStep {
                        interference_m: 0.001,
                        iter_count: 1,
                        final_residual_norm: 0.0,
                        x_final: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        readout: StepReadout {
                            n_active_contact_pairs: 0,
                            contact_force_total_n: Vector3::zeros(),
                            contact_force_magnitude_n: 0.0,
                            max_principal_stretch: 1.0,
                            min_principal_stretch: 1.0,
                            max_first_piola_frobenius_pa: 0.0,
                            mean_strain_energy_density_j_per_m3: 0.0,
                        },
                    },
                    RampStep {
                        interference_m: 0.002,
                        iter_count: 1,
                        final_residual_norm: 0.0,
                        // Step 2: vertex 1 has moved +0.5 in x; the
                        // deformed mesh should pick THIS step's x_final
                        // when asked for step 1.
                        x_final: vec![0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0],
                        readout: StepReadout {
                            n_active_contact_pairs: 0,
                            contact_force_total_n: Vector3::zeros(),
                            contact_force_magnitude_n: 0.0,
                            max_principal_stretch: 1.0,
                            min_principal_stretch: 1.0,
                            max_first_piola_frobenius_pa: 0.0,
                            mean_strain_energy_density_j_per_m3: 0.0,
                        },
                    },
                ],
                failed_at_step: None,
                failure_reason: None,
                final_x: vec![0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0],
                n_pinned: 0,
                result: None,
            },
            per_layer: Vec::new(),
            tet_centroids: Vec::new(),
            per_tet_layer: Vec::new(),
            per_step_scalar_fields: vec![[vec![], vec![]], [vec![], vec![]]],
            scalar_min_max: [(0.0, 0.0), (0.0, 0.0)],
            bcc_boundary_faces: vec![[0, 1, 2]],
            per_layer_outer_faces: Vec::new(),
        };
        let step0 = outputs.deformed_boundary_mesh_at(0).expect("step 0");
        assert_eq!(step0.faces, vec![[0, 1, 2]]);
        assert_eq!(step0.vertices.len(), 3);
        assert_eq!(step0.vertices[1], nalgebra::Point3::new(1.0, 0.0, 0.0));
        let step1 = outputs.deformed_boundary_mesh_at(1).expect("step 1");
        // Step 1 vertex 1 moved to (1.5, 0, 0) — the deformation.
        assert_eq!(step1.vertices[1], nalgebra::Point3::new(1.5, 0.0, 0.0));
        // OOB → None (no clamp; spec calls out fail-closed here).
        assert!(outputs.deformed_boundary_mesh_at(2).is_none());
    }

    /// Slice S1 — `scalar_fields_at` clamps OOB step indices to the
    /// last converged step (matches the "panel slider past the end"
    /// case + `displayed_step` default-zero before a run completes).
    #[test]
    fn scalar_fields_at_clamps_out_of_range_to_last_step() {
        let outputs = InsertionSimOutputs {
            ramp: InsertionRamp {
                steps: Vec::new(),
                failed_at_step: None,
                failure_reason: None,
                final_x: Vec::new(),
                n_pinned: 0,
                result: None,
            },
            per_layer: Vec::new(),
            tet_centroids: Vec::new(),
            per_tet_layer: Vec::new(),
            // Three steps with distinct energy fields so we can tell
            // which step the accessor returns.
            per_step_scalar_fields: vec![
                [vec![1.0], vec![10.0]],
                [vec![2.0], vec![20.0]],
                [vec![3.0], vec![30.0]],
            ],
            scalar_min_max: [(1.0, 3.0), (10.0, 30.0)],
            bcc_boundary_faces: Vec::new(),
            per_layer_outer_faces: Vec::new(),
        };
        assert_eq!(outputs.scalar_fields_at(0)[0], vec![1.0]);
        assert_eq!(outputs.scalar_fields_at(1)[0], vec![2.0]);
        assert_eq!(outputs.scalar_fields_at(2)[0], vec![3.0]);
        // Past the last converged step → clamp to last (3 steps → idx 2).
        assert_eq!(outputs.scalar_fields_at(99)[0], vec![3.0]);
        // Stress slot picks the same step index.
        assert_eq!(outputs.scalar_fields_at(0)[1], vec![10.0]);
        assert_eq!(outputs.scalar_fields_at(99)[1], vec![30.0]);
    }

    /// Slice S1 — `global_min_max_across` unions per-step ranges so the
    /// heat-map gradient stays calibrated across a playback scrub.
    #[test]
    fn global_min_max_across_unions_step_ranges() {
        let per_step: Vec<[Vec<f64>; 2]> = vec![
            // Step 0: energy [1, 5], stress [10, 50]
            [vec![1.0, 5.0], vec![10.0, 50.0]],
            // Step 1: energy [3, 7], stress [20, 30]
            [vec![3.0, 7.0], vec![20.0, 30.0]],
            // Step 2: energy [-2, 0.5], stress [5, 80]
            [vec![-2.0, 0.5], vec![5.0, 80.0]],
        ];
        assert_eq!(global_min_max_across(&per_step, 0), (-2.0, 7.0));
        assert_eq!(global_min_max_across(&per_step, 1), (5.0, 80.0));
        // Empty input — degenerate, collapses to (0, 0).
        assert_eq!(global_min_max_across(&[], 0), (0.0, 0.0));
    }

    /// `LAYER_COUNT_MAX` (from main.rs) caps the user-facing layer
    /// count; assert the bucket-index extraction tolerates that
    /// upper bound (no silent OOB).
    #[test]
    fn aggregate_per_layer_at_layer_count_max() {
        let n = crate::LAYER_COUNT_MAX;
        let per_tet: Vec<TetReadout> = (0..n)
            .map(|i| mk_readout([1.0, 1.0, 1.0], 1.0e3 * (i as f64 + 1.0), 1.0))
            .collect();
        let per_tet_layer: Vec<usize> = (0..n).collect();
        let per_layer = aggregate_per_layer(&per_tet, &per_tet_layer, n);
        assert_eq!(per_layer.len(), n);
        for (i, a) in per_layer.iter().enumerate() {
            assert_eq!(a.n_tets, 1);
            assert!((a.max_first_piola_frobenius_pa - 1.0e3 * (i as f64 + 1.0)).abs() < 1e-9);
        }
    }
}
