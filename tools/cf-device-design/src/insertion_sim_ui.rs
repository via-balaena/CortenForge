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

use std::fmt;

use anyhow::Result;
use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use bevy_egui::egui;
use mesh_types::IndexedMesh;

use crate::insertion_sim::{
    InsertionRamp, RampStep, SimDesign, SimLayer, StepReadout, TetReadout,
    build_insertion_geometry, run_insertion_ramp,
};
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
    /// Index into [`InsertionSimOutputs::per_layer_vertex_colors`].
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

/// Async-task outputs: the raw ramp + the panel's derived readouts.
/// `per_layer_vertex_colors` is `[Energy, Stress]` (matches
/// [`ScalarMode::buffer_index`]); each inner `Vec<Vec<[f32; 4]>>` is
/// indexed `[layer_index][proxy_vertex_index]`, length matches the
/// envelope proxy's vertex count for the layer's displaced shell.
pub struct InsertionSimOutputs {
    /// The raw ramp — for the F-d plot + per-step table.
    pub ramp: InsertionRamp,
    /// Per-layer reductions of `ramp.result.final_per_tet`.
    pub per_layer: Vec<LayerAggregate>,
    /// `[Energy, Stress]` → layer-indexed → proxy-vertex-indexed
    /// RGBA in `[0, 1]` colors, ready for Bevy's
    /// `Mesh::ATTRIBUTE_COLOR`.
    ///
    /// Temporarily orphaned (slice-9 sub-leaf 4): the per-layer mesh
    /// path swapped from per-vertex proxy displacement to
    /// SDF-extracted MC topology, so the prior `proxy.vertices.len()`
    /// → `[layer_index][vertex_index]` indexing no longer matches
    /// any per-layer MC mesh. Sub-leaf 7 re-projects per-tet scalars
    /// onto each layer's MC vertex set via closest-point lookup and
    /// re-wires the heat-map material path.
    #[allow(dead_code)] // re-wired in sub-leaf 7
    pub per_layer_vertex_colors: [Vec<Vec<[f32; 4]>>; 2],
    /// `[Energy, Stress]` global (min, max) used to normalize the
    /// gradient — also reported alongside the heat map so the user
    /// can read absolute scale.
    pub scalar_min_max: [(f64, f64); 2],
}

impl fmt::Debug for InsertionSimOutputs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Same `dbg!`-footgun rationale as `InsertionGeometry` /
        // `InsertionRamp`: the per-layer color buffers are large and
        // would dominate any debug print.
        write!(
            f,
            "InsertionSimOutputs {{ n_steps_converged: {}, n_layers: {}, ranges: {:?} }}",
            self.ramp.steps.len(),
            self.per_layer.len(),
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
        }
    }
}

/// Plugin that wires the Insertion-Sim panel into the app.
/// Installed by `main.rs` alongside the other panel plugins.
pub struct InsertionSimPlugin;

impl Plugin for InsertionSimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<InsertionSimState>().add_systems(
            Update,
            (
                invalidate_on_geometry_change,
                kick_off_simulation,
                poll_simulation_task,
            )
                .chain(),
        );
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
    let n_steps = state.n_steps;

    state.last_error = None;
    let pool = AsyncComputeTaskPool::get();
    let task = pool.spawn(async move {
        run_sim_pipeline(scan_clone, design, n_steps).map_err(|e| format!("{e:?}"))
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
/// per-layer aggregates. Pure compute; no Bevy / egui access (the
/// task pool runs off main thread).
///
/// Slice 9 sub-leaf 6: per-vertex heat-map projection is offline
/// here — the prior path keyed projection on the now-retired
/// `EnvelopeProxyMesh.vertices`; sub-leaf 7 re-projects per-tet
/// scalars onto each layer's SDF-extracted MC mesh and re-fills
/// the per-layer vertex-color buffers.
fn run_sim_pipeline(
    scan: IndexedMesh,
    design: SimDesign,
    n_steps: usize,
) -> Result<InsertionSimOutputs> {
    let geometry = build_insertion_geometry(&scan, &design, SIM_SDF_TARGET_FACES, SIM_CELL_SIZE_M)?;

    let n_layers = design.layers.len();
    let per_tet_layer = geometry.per_tet_layer.clone();

    let ramp = run_insertion_ramp(geometry, n_steps)?;

    let result = ramp.result.as_ref().ok_or_else(|| {
        anyhow::anyhow!(
            "ramp failed at step 0 — no converged step. {}",
            ramp.failure_reason.as_deref().unwrap_or("(no reason)"),
        )
    })?;

    let per_layer = aggregate_per_layer(&result.final_per_tet, &per_tet_layer, n_layers);

    // Two scalar fields (energy + stress norm) per tet — reported
    // through `scalar_min_max` so the panel can show absolute scale
    // even with the heat-map projection offline.
    let energy_field: Vec<f64> = result
        .final_per_tet
        .iter()
        .map(|t| t.energy_density_j_per_m3)
        .collect();
    let stress_field: Vec<f64> = result
        .final_per_tet
        .iter()
        .map(|t| t.first_piola_frobenius_pa)
        .collect();
    let energy_min_max = global_min_max(&energy_field);
    let stress_min_max = global_min_max(&stress_field);

    // Sub-leaf 6: heat-map color buffers empty (one Vec per layer,
    // each Vec empty). Sub-leaf 7 fills them via per-MC-vertex
    // nearest-tet lookup.
    let empty_layer_buffers: Vec<Vec<[f32; 4]>> = (0..n_layers).map(|_| Vec::new()).collect();
    Ok(InsertionSimOutputs {
        ramp,
        per_layer,
        per_layer_vertex_colors: [empty_layer_buffers.clone(), empty_layer_buffers],
        scalar_min_max: [energy_min_max, stress_min_max],
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

/// Map a scalar to an RGBA color in `[0, 1]` via a three-stop gradient
//
// Slice 9 sub-leaf 6: heat-map projection retired (was the only
// caller); sub-leaf 7 re-projects per-tet scalars onto each layer's
// SDF-extracted MC mesh and re-introduces a (different) caller.
// Worth keeping inline pending sub-leaf 7 rather than churning the
// gradient stops + sign-by-sign clamp through deletion + reintro.
#[allow(dead_code)] // re-wired in sub-leaf 7
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
