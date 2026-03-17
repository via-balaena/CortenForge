//! Mechanism assembly types.
//!
//! A mechanism is a multi-part assembly connected by joints — the atomic unit
//! of design in cf-design. This module provides the foundational types:
//!
//! - [`Part`] — named solid body with material and flex zones
//! - [`Material`] / [`ManufacturingProcess`] — physical properties
//! - [`JointDef`] / [`JointKind`] — joint connections between parts
//! - [`TendonDef`] / [`TendonWaypoint`] — tendon routing and channel subtraction
//! - [`ActuatorDef`] / [`ActuatorKind`] — tendon actuation
//! - [`PrintProfile`] — manufacturing clearance constraints
//! - [`DesignWarning`] — validation diagnostic
//! - [`FlexZone`] / [`Plane`] — compliant mechanism support
//!
//! The `Mechanism` builder (Session 9) and MJCF generation (Session 10) build
//! on these types.

pub mod actuator;
pub mod joint;
pub mod material;
pub mod part;
pub mod print;
pub mod tendon;
pub mod validate;

pub use actuator::{ActuatorDef, ActuatorKind};
pub use joint::{JointDef, JointKind};
pub use material::{ManufacturingProcess, Material};
pub use part::{FlexZone, Part, Plane};
pub use print::PrintProfile;
pub use tendon::{TendonDef, TendonWaypoint};
pub use validate::DesignWarning;
