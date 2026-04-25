//! Canonical test-model fixtures.
//!
//! Pure-Rust [`Model`] factories for the small set of canonical mechanical
//! systems exercised across the L0-pure crates' test suites
//! (`sim-thermostat`, `sim-ml-chassis`, `sim-opt`). These fixtures replace
//! per-test ad-hoc MJCF XML strings + `sim_mjcf::load_model` so those
//! crates can drop their `sim-mjcf` dev-dep — keeping their dev-graphs
//! free of the `image`/`zip`/`mesh-io` chain. See L0 plan §3.2.
//!
//! Gated behind the `test-fixtures` feature (or unconditionally available
//! under `cfg(test)` for sim-core's own internal tests). Downstream crates
//! opt in via `sim-core = { workspace = true, features = ["test-fixtures"] }`
//! in `[dev-dependencies]`.
//!
//! # Organization
//!
//! - [`particles`] — single-DOF slide-axis particle presets (SHO, Brownian
//!   ratchet, stochastic resonance, bistable, single slide, Ising pair).
//! - [`pendulums`] — single-hinge pendulum variants (with/without sensor,
//!   ctrllimited motor, mocap target body, tip site, bench-timestep).
//! - [`arms`] — multi-link reaching arms (2-DOF planar, 6-DOF planar,
//!   6-DOF + obstacle).
//! - [`chains`] — multi-element chains (bistable chain of N, hinge chain
//!   of N).
//! - [`bodies`] — free bodies, hinge chains with explicit inertials, and
//!   miscellaneous single-purpose fixtures (cart-pole, builder-test
//!   minimal).
//!
//! # Fidelity contract
//!
//! Each fixture produces a `Model` with the same structural counts
//! (`nbody`, `njnt`, `nq`, `nv`, `nu`, `ngeom`, `nsite`, `nsensor`,
//! `nmocap`) and the same body/joint/dof name strings as the MJCF it
//! replaces. Inline `#[test]`s verify these invariants. Numeric values
//! (mass, inertia, joint stiffness/damping, actuator gains) match the
//! MJCF's explicit values; for fields the MJCF leaves implicit (e.g.
//! geom-derived inertia under `inertiafromgeom="auto"`), fixtures use
//! plausible thin-rod/uniform-sphere approximations. Per-call-site
//! migration in plan §8 step 7 may reveal divergences worth tightening.

pub mod arms;
pub mod bodies;
pub(crate) mod builders;
pub mod chains;
pub mod particles;
pub mod pendulums;

pub use arms::{reaching_2dof, reaching_6dof, reaching_6dof_obstacle};
pub use bodies::{builder_test_minimal, cart_pole, free_body_diag, hinge_chain_2dof_inertial};
pub use chains::{bistable_chain, hinge_chain};
pub use particles::{
    bistable_1dof, ising_pair, ratchet, sho_1d, single_slide, stochastic_resonance,
};
pub use pendulums::{
    pendulum_basic, pendulum_bench, pendulum_clamped, pendulum_mocap, pendulum_with_angle_sensor,
    pendulum_with_tip_site,
};
