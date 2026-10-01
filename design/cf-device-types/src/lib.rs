//! Shared device-design domain types.
//!
//! This crate is **types only**. No FEM, no rendering, no UI. Bevy
//! enters only through `#[derive(Resource)]`, behind the off-by-default
//! `bevy` feature.
//!
//! The five submodules are organized by topic:
//!
//! - [`scan`] — scan-side resources: `ScanMesh`, `ScanFilePath`,
//!   `ScanMeshVisible`, `ScanInfo`, plus the `Centerline` polyline
//!   from `cf-scan-prep`'s `.prep.toml`.
//! - [`design`] — user-dialed design state: `CavityState`,
//!   `LayerSpec`, `LayersState`, plus the silicone catalog
//!   ([`LAYER_MATERIALS`]) and the default constants.
//! - [`slacker`] — Smooth-On Slacker™ TB curve data (`Support`,
//!   `Point`, `ShoreHardness`, `ShoreScale`, `Tack`) and
//!   `resolve_slacker_fraction` — the canonical "snap an arbitrary
//!   fraction to the curve, or fall back to the native 0.0" function.
//! - [`sim`] — the sim-side projection of `(CavityState,
//!   LayersState)` into `SimDesign` / `SimLayer`, plus the per-run
//!   UI enums (`ScalarMode`, `SimMode`) and the `SlackerResolution`
//!   enum describing how a layer's Slacker-softened material was
//!   resolved.
//! - [`design_toml`] — `.design.toml` Save/Open schema +
//!   load/save/validate/apply helpers.

pub mod design;
pub mod design_toml;
pub mod scan;
pub mod sim;
pub mod slacker;

pub use design::{
    CAVITY_DEFAULT_INSET_M, CAVITY_INSET_SLIDER_MAX_M, CavityState, LAYER_COUNT_MAX,
    LAYER_MATERIALS, LAYER_SURFACE_PALETTE, LayerSpec, LayersState, material_density,
};
pub use scan::{Centerline, ScanFilePath, ScanInfo, ScanMesh, ScanMeshVisible};
pub use sim::{ScalarMode, SimDesign, SimLayer, SimMode, SlackerResolution};
pub use slacker::resolve_slacker_fraction;
