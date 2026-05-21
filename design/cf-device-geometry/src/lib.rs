//! Shared device-side geometric primitives.
//!
//! Lifted out of `tools/cf-device-design/src/{sdf_layers.rs,
//! clip_plane.rs,main.rs}` per
//! `docs/archive/SIM_DECOUPLE_PHASE_3_RECON.md` §2.5.b-d. The CAD binary
//! (`cf-device-design`) and the future sim-research binary
//! (`cf-sim-research`, Phase 3+) both depend on this crate so they
//! share one cached scan SDF + per-iso marching-cubes extraction
//! path, one set of cavity-render helpers, and one clip-plane
//! visualization stack.
//!
//! This crate is **compute + rendering primitives**, not types-only
//! (that's `cf-device-types`). The split: `cf-device-types` carries
//! pure data (CavityState / LayersState / ScalarMode / catalog /
//! palette), `cf-device-geometry` carries the meshy compute +
//! Bevy-side rendering plumbing (cached SDF + iso extraction + the
//! clip-plane ExtendedMaterial + the rest-frame cavity spawner).
//!
//! ## Module map
//!
//! - [`sdf_layers`] — the cached scan SDF + per-iso marching-cubes
//!   extraction (`CachedScanSdf`, `CapPlanes`, `build_cached_scan_sdf`,
//!   `extract_layer_surface`, `marching_cubes_at_iso`,
//!   `sample_sdf_into_cached_template`, plus the
//!   `SDF_SOURCE_TARGET_FACES` / `LAYER_PREVIEW_CELL_SIZE_M` /
//!   `LAYER_GRID_MARGIN_M` tuning constants).
//! - [`clip_plane`] — the centerline-anchored clip plane (material,
//!   plugin, state resource, egui section, math + WGSL shader).
//! - [`bevy_mesh`] — `IndexedMesh → bevy::Mesh` adapters
//!   (`build_bevy_mesh_from_indexed` + the `_with_colors` heat-map
//!   variant).
//! - [`cavity`] — the rest-frame cavity entity (`spawn_cavity_mesh`,
//!   `CavityEntity` marker, `CAVITY_COLOR`).
//!
//! ## Stage of population
//!
//! - **Phase 2.5.b** shipped the empty crate skeleton.
//! - **Phase 2.5.d** populated [`clip_plane`].
//! - **Phase 2.5.c** populated [`sdf_layers`] + [`bevy_mesh`] +
//!   [`cavity`].

pub mod bevy_mesh;
pub mod cavity;
pub mod clip_plane;
pub mod sdf_layers;
