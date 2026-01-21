//! Deformable body simulation for soft bodies, cloth, and ropes.
//!
//! This crate provides comprehensive deformable simulation using Extended Position-Based
//! Dynamics (XPBD), offering:
//!
//! - **1D deformables**: Ropes, cables, capsule chains for cable simulation
//! - **2D deformables**: Cloth, membranes, triangle shells
//! - **3D deformables**: Soft bodies, volumetric meshes (tetrahedral)
//!
//! # Physics Model
//!
//! The simulation uses XPBD (Extended Position-Based Dynamics), which provides:
//!
//! - **Stability**: Unconditionally stable for any time step
//! - **Compliance**: Physical stiffness parameters (not iteration-dependent)
//! - **Robustness**: Handles large deformations and thin shells
//!
//! ## XPBD Algorithm
//!
//! ```text
//! For each time step:
//!   1. Predict positions: x* = x + v*dt + gravity*dt²
//!   2. For each solver iteration:
//!      a. Solve distance constraints (stretch)
//!      b. Solve bending constraints
//!      c. Solve volume constraints (3D only)
//!      d. Solve collision constraints
//!   3. Update velocities: v = (x - x_prev) / dt
//!   4. Apply damping
//! ```
//!
//! # Deformable Types
//!
//! ## 1D: Capsule Chains (Ropes/Cables)
//!
//! Chains of particles connected by distance constraints:
//!
//! ```text
//! ●───●───●───●───●
//! 0   1   2   3   4
//! ```
//!
//! Use cases: Ropes, cables, tendons, hair strands
//!
//! ## 2D: Triangle Shells (Cloth)
//!
//! Triangle mesh with distance and bending constraints:
//!
//! ```text
//!   ●───●───●
//!   |\  |\  |
//!   | \ | \ |
//!   ●───●───●
//! ```
//!
//! Use cases: Cloth, membranes, tissue surfaces
//!
//! ## 3D: Tetrahedral Meshes (Soft Bodies)
//!
//! Volumetric mesh with distance, bending, and volume constraints:
//!
//! ```text
//!       ●
//!      /|\
//!     / | \
//!    ●──●──●
//!     \ | /
//!      \|/
//!       ●
//! ```
//!
//! Use cases: Soft tissue, gelatin, rubber, foam
//!
//! # Material Model
//!
//! Materials define physical properties:
//!
//! - **Young's modulus (E)**: Stiffness (Pa)
//! - **Poisson's ratio (ν)**: Volume preservation (0-0.5)
//! - **Density (ρ)**: Mass per volume (kg/m³)
//! - **Damping**: Energy dissipation
//!
//! # Quick Start
//!
//! ```
//! use sim_deformable::{
//!     CapsuleChain, CapsuleChainConfig, Material, XpbdSolver,
//!     DeformableBody, SolverConfig,
//! };
//! use nalgebra::Point3;
//!
//! // Create a rope with 10 segments
//! let mut rope = CapsuleChain::new(
//!     "rope",
//!     Point3::new(0.0, 0.0, 2.0),  // Start position
//!     Point3::new(5.0, 0.0, 2.0),  // End position
//!     10,                           // Number of segments
//!     CapsuleChainConfig::rope(0.01), // 1cm radius rope
//! );
//!
//! // Pin the first particle (fixed to world)
//! rope.pin_vertex(0);
//!
//! // Create solver
//! let mut solver = XpbdSolver::new(SolverConfig::default());
//!
//! // Simulate
//! let gravity = nalgebra::Vector3::new(0.0, 0.0, -9.81);
//! solver.step(&mut rope, gravity, 1.0 / 60.0);
//! ```
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops for reinforcement learning
//! - Hardware control systems
//! - Analysis and optimization tools
//! - Integration with other physics engines

#![doc(html_root_url = "https://docs.rs/sim-deformable/0.1.0")]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![warn(missing_docs)]
// Allow precision loss when converting indices to f64 - these are small values
#![allow(clippy::cast_precision_loss)]
// Allow sign loss for array indices - we validate bounds
#![allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
// Allow long functions for complex algorithms
#![allow(clippy::too_many_lines)]
// Allow unused self for trait implementations
#![allow(clippy::unused_self)]
// Allow needless pass by ref mut for trait compliance
#![allow(clippy::needless_pass_by_ref_mut)]
// Allow pass by value for small Copy types
#![allow(clippy::needless_pass_by_value)]
// Test-related lints - these are style preferences
#![cfg_attr(test, allow(clippy::uninlined_format_args, clippy::float_cmp))]

pub mod capsule_chain;
pub mod cloth;
pub mod constraints;
pub mod error;
pub mod material;
pub mod mesh;
pub mod soft_body;
pub mod solver;
pub mod types;

// Re-export main types at crate root
pub use capsule_chain::{CapsuleChain, CapsuleChainConfig};
pub use cloth::{Cloth, ClothConfig};
pub use constraints::{
    BendingConstraint, Constraint, ConstraintType, DistanceConstraint, VolumeConstraint,
};
pub use error::DeformableError;
pub use material::{Material, MaterialPreset};
pub use mesh::{DeformableMesh, Edge, Tetrahedron, Triangle};
pub use soft_body::{SoftBody, SoftBodyConfig};
pub use solver::{SolverConfig, XpbdSolver};
pub use types::{DeformableId, Vertex, VertexFlags};

/// Trait for deformable bodies that can be simulated.
///
/// This trait provides a common interface for different deformable body types,
/// allowing them to be used with the XPBD solver.
pub trait DeformableBody {
    /// Get the deformable body's unique identifier.
    fn id(&self) -> DeformableId;

    /// Get the name of this deformable body.
    fn name(&self) -> &str;

    /// Get the number of vertices (particles).
    fn num_vertices(&self) -> usize;

    /// Get the vertex positions.
    fn positions(&self) -> &[nalgebra::Point3<f64>];

    /// Get mutable access to vertex positions.
    fn positions_mut(&mut self) -> &mut [nalgebra::Point3<f64>];

    /// Get the vertex velocities.
    fn velocities(&self) -> &[nalgebra::Vector3<f64>];

    /// Get mutable access to vertex velocities.
    fn velocities_mut(&mut self) -> &mut [nalgebra::Vector3<f64>];

    /// Get the vertex inverse masses (0 for pinned vertices).
    fn inverse_masses(&self) -> &[f64];

    /// Get the vertex flags (pinned, etc.).
    fn vertex_flags(&self) -> &[VertexFlags];

    /// Get mutable access to vertex flags.
    fn vertex_flags_mut(&mut self) -> &mut [VertexFlags];

    /// Get the constraints for this deformable body.
    fn constraints(&self) -> &[Constraint];

    /// Get the material properties.
    fn material(&self) -> &Material;

    /// Pin a vertex (make it immovable).
    fn pin_vertex(&mut self, index: usize);

    /// Unpin a vertex.
    fn unpin_vertex(&mut self, index: usize);

    /// Check if a vertex is pinned.
    fn is_pinned(&self, index: usize) -> bool;

    /// Apply an external force to a vertex.
    fn apply_force(&mut self, index: usize, force: nalgebra::Vector3<f64>);

    /// Get accumulated external forces.
    fn external_forces(&self) -> &[nalgebra::Vector3<f64>];

    /// Clear accumulated external forces.
    fn clear_forces(&mut self);

    /// Get the total mass of the deformable body.
    fn total_mass(&self) -> f64;

    /// Get the center of mass position.
    fn center_of_mass(&self) -> nalgebra::Point3<f64>;

    /// Get the bounding box of the deformable body.
    fn bounding_box(&self) -> (nalgebra::Point3<f64>, nalgebra::Point3<f64>);
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_capsule_chain_basic() {
        let rope = CapsuleChain::new(
            "test_rope",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        assert_eq!(rope.num_vertices(), 5);
        assert_eq!(rope.name(), "test_rope");

        // Check positions are evenly distributed
        let positions = rope.positions();
        assert_relative_eq!(positions[0].x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(positions[4].x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_solver_preserves_pinned() {
        let mut rope = CapsuleChain::new(
            "test_rope",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        rope.pin_vertex(0);
        let start_pos = rope.positions()[0];

        let mut solver = XpbdSolver::new(SolverConfig::default());
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        // Simulate several steps
        for _ in 0..10 {
            solver.step(&mut rope, gravity, 1.0 / 60.0);
        }

        // Pinned vertex should not have moved
        assert_relative_eq!(
            rope.positions()[0].coords,
            start_pos.coords,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_material_presets() {
        let rubber = Material::preset(MaterialPreset::Rubber);
        let tendon = Material::preset(MaterialPreset::Tendon);

        // Tendon should be much stiffer than rubber
        assert!(tendon.youngs_modulus > rubber.youngs_modulus);
    }
}
