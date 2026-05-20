//! Shared device-side geometric primitives.
//!
//! Lifted out of `tools/cf-device-design/src/{sdf_layers.rs,
//! clip_plane.rs,main.rs}` per
//! `docs/SIM_DECOUPLE_PHASE_3_RECON.md` §2.5.b-d. The CAD binary
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
//! clip-plane ExtendedMaterial).
//!
//! ## Stage of population
//!
//! Phase 2.5.b ships this crate as an empty skeleton — the lift
//! happens in subsequent sub-leaves:
//!
//! - **Phase 2.5.c** populates `sdf` (the lifted `sdf_layers`) +
//!   shared cavity-render helpers (`spawn_cavity_mesh`,
//!   `build_bevy_mesh_from_indexed`, `build_bevy_mesh_from_indexed_with_colors`,
//!   `CavityEntity`, `CAVITY_COLOR`).
//! - **Phase 2.5.d** populates `clip_plane` (the full module
//!   including `render_clip_plane_section`), adds `bevy_pbr` +
//!   `bevy_render` to the Bevy feature set, and provides a
//!   clipping slider to both binaries out-of-the-box.
//!
//! Until then, this crate exposes no public surface — the
//! migration-safety invariant (workspace build green after every
//! sub-leaf) is held by leaving the consumers untouched in 2.5.b
//! and only flipping their imports as the symbols become available
//! in 2.5.c and 2.5.d.
