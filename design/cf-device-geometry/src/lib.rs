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
//! - **Phase 2.5.b** shipped the empty crate skeleton.
//! - **Phase 2.5.d** populates [`clip_plane`] (the full module
//!   including [`clip_plane::render_clip_plane_section`]) — both
//!   binaries get a clipping slider out-of-the-box. Lift order swapped
//!   relative to the recon's alphabetic order per the recon's
//!   §2.5.c authorization: clip_plane is self-contained, so doing it
//!   first means 2.5.c can include `spawn_cavity_mesh` in a single
//!   commit (its `Assets<ClipPlaneMaterial>` parameter resolves
//!   against the lifted material at lift time).
//! - **Phase 2.5.c** (still pending at module-doc revision time)
//!   populates `sdf` (the lifted `sdf_layers`) + shared cavity-render
//!   helpers (`spawn_cavity_mesh`, `build_bevy_mesh_from_indexed*`,
//!   `CavityEntity`, `CAVITY_COLOR`).

pub mod clip_plane;
