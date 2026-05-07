//! Orbit camera surface — re-exported from [`cf_bevy_common::camera`].
//!
//! sim-bevy is one of three Bevy consumers (alongside `cf-viewer` and
//! `sim-bevy-soft`); the `OrbitCamera` controller and `OrbitCameraPlugin`
//! live in `cf-bevy-common` so all three share one implementation. This
//! module re-exports them so consumers using the historical
//! `sim_bevy::camera::*` path continue to resolve.
//!
//! New code should prefer [`crate::prelude`], which re-exports both
//! types alongside the rest of sim-bevy's public surface.

pub use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
