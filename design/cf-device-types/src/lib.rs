//! Shared device-design domain types.
//!
//! Lifted out of `tools/cf-device-design/src/main.rs` per
//! `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` §3 (A1 Phase 1). The CAD
//! binary (`cf-device-design`) and the future sim-research binary
//! (`cf-sim-research`, Phase 2+) both depend on this crate so they
//! describe a layered-silicone device — the cavity inset, the
//! ordered layer stack, the scan resources, the centerline, and the
//! sim-design projection of those — the same way.
//!
//! This crate is **types only**. No FEM, no rendering, no UI. Bevy
//! enters only through `#[derive(Resource)]` on the resources the
//! Bevy binaries need to share.
//!
//! The four submodules are organized by topic:
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
//!   LayersState)` into the insertion-sim's `SimDesign` /
//!   `SimLayer`, plus the per-run UI enums (`ScalarMode`,
//!   `SimMode`) and the `SlackerResolution` enum describing how
//!   `effective_silicone_for_layer` resolved a layer.

pub mod design;
pub mod scan;
pub mod sim;
pub mod slacker;

pub use design::{
    CAVITY_DEFAULT_INSET_M, CavityState, LAYER_COUNT_MAX, LAYER_MATERIALS, LayerSpec, LayersState,
    material_density,
};
pub use scan::{Centerline, ScanFilePath, ScanInfo, ScanMesh, ScanMeshVisible};
pub use sim::{ScalarMode, SimDesign, SimLayer, SimMode, SlackerResolution};
pub use slacker::resolve_slacker_fraction;
