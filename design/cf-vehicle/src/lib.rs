//! Vehicle-scale parametric specs, and the load cases they derive.
//!
//! Every part on a vehicle is sized against a number that came from
//! somewhere. This crate is that somewhere. A [`TrikeSpec`] is the
//! wheelbase, track, wheel sizes, steering geometry and a **lumped mass
//! budget** — and from those alone it derives the axle-load split,
//! per-wheel static and cornering loads, the quasi-static rollover
//! threshold and the steering trail.
//!
//! ★ **The point is that nothing here is a chosen number.** Before this
//! crate the wheel arc sized its spokes against a 70 % rear bias that its
//! own plan labelled *"ESTIMATE, not measured"*. A rear share is not a
//! thing you estimate — it is `x_cg / wheelbase`, which is arithmetic over
//! a mass budget. The budget's individual masses are still estimates, but
//! each is now one named line that one weighing corrects, instead of a
//! single fraction nobody can check.
//!
//! # Coordinates
//!
//! One convention, used everywhere in this crate:
//!
//! - **`x`** — metres **aft of the front tyre's contact patch**. The front
//!   contact patch is the origin, so `x = 0` is the front wheel and
//!   `x = wheelbase_m` is the rear axle's contact line.
//! - **`z`** — metres **above the ground plane**.
//! - **`y`** — lateral. ⚠ Not modelled. This crate assumes the mass budget
//!   is **symmetric about the centreline**, which for a single seated
//!   rider it very nearly is, and which [`CorneringLoads`] depends on.
//!
//! # The design rule this crate exists to state
//!
//! ★★★ **A drift trike must slide before it tips.** Two independent
//! numbers decide that, and neither is a preference:
//!
//! - the **friction limit** — how much lateral acceleration the rear tyres
//!   can generate before they break away, which is `µ`, a property of the
//!   compound;
//! - the **rollover threshold** — how much lateral acceleration the
//!   geometry survives before the inner rear wheel lifts, which is track,
//!   wheelbase and centre-of-gravity height.
//!
//! Whichever is lower is what actually happens in a corner. Choosing a
//! tyre hardness is therefore a **stability decision**, not a feel one, and
//! [`CorneringLoads::slides_before_it_tips`] is the check.
//!
//! ⚠ The rollover threshold here is **quasi-static**. A kerb strike, a
//! pothole or a sharp steering input can tip a vehicle that clears the
//! static number comfortably; a static margin is necessary, not sufficient.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

pub mod analysis;
pub mod spec;

pub use analysis::{CorneringLoads, StaticLoads, SteeringGeometry};
pub use spec::{MassItem, TrikeSpec};

/// Standard gravity, m/s².
///
/// The CGPM-defined conventional value, not a local one. Every weight in
/// this crate is a mass times this constant.
pub const GRAVITY_M_S2: f64 = 9.806_65;
