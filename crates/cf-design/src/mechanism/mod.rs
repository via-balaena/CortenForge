//! Mechanism assembly types.
//!
//! A mechanism is a multi-part assembly connected by joints — the atomic unit
//! of design in cf-design. Build one using [`Mechanism::builder`]:
//!
//! - [`Mechanism`] — validated assembly with tendon channels applied
//! - [`MechanismBuilder`] — chainable builder for constructing mechanisms
//! - [`Part`] — named solid body with material and flex zones
//! - [`Material`] / [`ManufacturingProcess`] — physical properties
//! - [`JointDef`] / [`JointKind`] — joint connections between parts
//! - [`TendonDef`] / [`TendonWaypoint`] — tendon routing and channel subtraction
//! - [`ActuatorDef`] / [`ActuatorKind`] — tendon actuation
//! - [`PrintProfile`] — manufacturing clearance constraints
//! - [`MechanismError`] — structural validation error
//! - [`DesignWarning`] — manufacturing validation diagnostic
//! - [`FlexZone`] / [`Plane`] — compliant mechanism support

pub mod actuator;
pub mod builder;
pub mod flex_split;
pub mod joint;
pub mod mass;
pub mod material;
mod mjcf;
pub mod part;
pub mod print;
mod shapes;
mod stl;
pub mod templates;
pub mod tendon;
pub mod validate;

pub use actuator::{ActuatorDef, ActuatorKind};
pub use builder::{Mechanism, MechanismBuilder, MechanismError};
pub use flex_split::{CrossSection, FlexJoint, SplitResult, measure_cross_section, split_part};
pub use joint::{JointDef, JointKind};
pub use mass::MassProperties;
pub use material::{ManufacturingProcess, Material};
pub use part::{FlexZone, Part, Plane};
pub use print::PrintProfile;
pub use shapes::ShapeMode;
pub use tendon::{TendonDef, TendonWaypoint};
pub use validate::DesignWarning;

#[cfg(test)]
mod integration;
