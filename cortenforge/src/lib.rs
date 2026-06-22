//! # CortenForge SDK
//!
//! The single, stable public surface of the CortenForge SDK. Applications
//! depend on this one crate and reach the SDK through it — `cortenforge::cf_cast`,
//! `cortenforge::mesh_io`, and so on — so the SDK's internal crate structure can
//! evolve behind one contract. See `MISSION.md` and the app-vs-SDK boundary.
//!
//! This is a **facade**: each item below re-exports a constituent SDK crate
//! verbatim. It adds no logic of its own.
//!
//! ## Mesh
//! - [`mesh_types`] — core mesh types (`IndexedMesh`, `Vertex`, `Aabb`).
//! - [`mesh_io`] — mesh I/O (STL/OBJ/PLY/3MF).
//! - [`mesh_repair`] — mesh repair / welding.
//!
//! ## Design & geometry
//! - [`cf_design`] — implicit-surface design kernel.
//! - [`cf_cap_planes`] — cap-plane parsing.
//! - [`cf_device_types`] — shared device-design domain types.
//!
//! ## Scan → fabrication
//! - [`cf_scan_prep_core`] — headless scan-prep (repair / cap / centerline / trim).
//! - [`cf_cast`] — multi-material mold generation.
//! - [`cf_cast_cli`] — the scan→cast bridge (`run_with_config`, `CastConfig`).

pub use cf_cap_planes;
pub use cf_cast;
pub use cf_cast_cli;
pub use cf_design;
pub use cf_device_types;
pub use cf_scan_prep_core;
pub use mesh_io;
pub use mesh_repair;
pub use mesh_types;
