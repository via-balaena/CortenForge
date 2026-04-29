//! Implicit surface design kernel for CortenForge.
//!
//! `cf-design` enables code-first geometry design using implicit surfaces
//! (signed distance fields and f-rep). An AI or human writes Rust code that
//! defines geometry; that geometry IS the simulation collider and the 3D print
//! artifact. No export step. No approximation gap.
//!
//! # Quick start
//!
//! ```rust
//! use cf_design::Solid;
//! use nalgebra::{Point3, Vector3};
//!
//! // Create a sphere with radius 5
//! let ball = Solid::sphere(5.0);
//!
//! // Query the field at a point
//! assert!(ball.evaluate(&Point3::origin()) < 0.0);     // inside
//! assert!(ball.evaluate(&Point3::new(5.0, 0.0, 0.0)).abs() < 1e-10); // surface
//! assert!(ball.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0);  // outside
//! ```
//!
//! # Layer 0
//!
//! Zero Bevy, zero GPU, zero framework dependencies. Pure `nalgebra` +
//! `cf-geometry`.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod adaptive_dc;
mod bounds;
mod dual_contouring;
mod evaluate;
mod evaluate_batch;
pub(crate) mod field_node;
mod gradient;
mod interval;
pub mod mechanism;
mod mesher;
pub mod optim;
mod param;
mod param_gradient;
mod simplify;
mod solid;

pub use mechanism::templates;
pub use mechanism::{
    ActuatorDef, ActuatorKind, CrossSection, DesignWarning, FlexJoint, FlexZone, JointDef,
    JointKind, ManufacturingProcess, MassProperties, Material, Mechanism, MechanismBuilder,
    MechanismError, Part, Plane, PrintProfile, ShapeMode, SplitResult, TendonDef, TendonWaypoint,
    split_part,
};
pub use param::{ParamRef, ParamStore};
pub use solid::{InfillKind, ShapeHint, Solid};

// Re-export geometry types that appear in our public API
// (Solid::bounds() → Aabb, Solid::sdf_grid_at() → SdfGrid, Solid::mesh() → AttributedMesh)
pub use cf_geometry::{Aabb, IndexedMesh, SdfGrid};
pub use mesh_types::AttributedMesh;
