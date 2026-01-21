//! Core types for deformable body simulation.
//!
//! This module provides fundamental types for representing deformable bodies:
//!
//! - [`DeformableId`] - Unique identifier for deformable bodies
//! - [`Vertex`] - A particle in the deformable mesh
//! - [`VertexFlags`] - Flags for vertex state (pinned, etc.)

use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unique identifier for a deformable body in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DeformableId(pub u64);

impl DeformableId {
    /// Create a new deformable body ID.
    #[must_use]
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Get the raw ID value.
    #[must_use]
    pub const fn raw(self) -> u64 {
        self.0
    }
}

impl From<u64> for DeformableId {
    fn from(id: u64) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for DeformableId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Deformable({})", self.0)
    }
}

bitflags::bitflags! {
    /// Flags for vertex state and behavior.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct VertexFlags: u32 {
        /// Vertex is pinned (immovable).
        const PINNED = 0b0000_0001;
        /// Vertex is on the boundary of the mesh.
        const BOUNDARY = 0b0000_0010;
        /// Vertex is part of a collision.
        const COLLIDING = 0b0000_0100;
        /// Vertex has been modified this frame.
        const DIRTY = 0b0000_1000;
    }
}

/// A vertex (particle) in a deformable mesh.
///
/// Each vertex has a position, velocity, and mass. Vertices are connected
/// by constraints to form the deformable structure.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Vertex {
    /// Position in world coordinates.
    pub position: Point3<f64>,
    /// Velocity in world coordinates.
    pub velocity: Vector3<f64>,
    /// Previous position (for velocity update).
    pub prev_position: Point3<f64>,
    /// Mass of the vertex in kg.
    pub mass: f64,
    /// Inverse mass (0 for pinned/infinite mass vertices).
    pub inv_mass: f64,
    /// Accumulated external force for this frame.
    pub force: Vector3<f64>,
    /// Vertex flags (pinned, boundary, etc.).
    pub flags: VertexFlags,
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            velocity: Vector3::zeros(),
            prev_position: Point3::origin(),
            mass: 1.0,
            inv_mass: 1.0,
            force: Vector3::zeros(),
            flags: VertexFlags::empty(),
        }
    }
}

impl Vertex {
    /// Create a new vertex at the given position with the given mass.
    #[must_use]
    pub fn new(position: Point3<f64>, mass: f64) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        Self {
            position,
            velocity: Vector3::zeros(),
            prev_position: position,
            mass,
            inv_mass,
            force: Vector3::zeros(),
            flags: VertexFlags::empty(),
        }
    }

    /// Create a pinned vertex (infinite mass, immovable).
    #[must_use]
    pub fn pinned(position: Point3<f64>) -> Self {
        Self {
            position,
            velocity: Vector3::zeros(),
            prev_position: position,
            mass: f64::INFINITY,
            inv_mass: 0.0,
            force: Vector3::zeros(),
            flags: VertexFlags::PINNED,
        }
    }

    /// Pin this vertex (make it immovable).
    pub fn pin(&mut self) {
        self.flags.insert(VertexFlags::PINNED);
        self.inv_mass = 0.0;
    }

    /// Unpin this vertex.
    pub fn unpin(&mut self) {
        self.flags.remove(VertexFlags::PINNED);
        if self.mass > 0.0 && self.mass.is_finite() {
            self.inv_mass = 1.0 / self.mass;
        }
    }

    /// Check if this vertex is pinned.
    #[must_use]
    pub const fn is_pinned(&self) -> bool {
        self.flags.contains(VertexFlags::PINNED)
    }

    /// Check if this vertex is on the boundary.
    #[must_use]
    pub const fn is_boundary(&self) -> bool {
        self.flags.contains(VertexFlags::BOUNDARY)
    }

    /// Apply an external force to this vertex.
    pub fn apply_force(&mut self, force: Vector3<f64>) {
        self.force += force;
    }

    /// Clear accumulated external forces.
    pub fn clear_force(&mut self) {
        self.force = Vector3::zeros();
    }

    /// Get the kinetic energy of this vertex.
    #[must_use]
    pub fn kinetic_energy(&self) -> f64 {
        0.5 * self.mass * self.velocity.norm_squared()
    }
}

/// ID generator for deformable bodies.
///
/// Thread-safe counter for generating unique IDs.
pub struct DeformableIdGenerator {
    next_id: std::sync::atomic::AtomicU64,
}

impl Default for DeformableIdGenerator {
    fn default() -> Self {
        Self::new()
    }
}

impl DeformableIdGenerator {
    /// Create a new ID generator starting at 0.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            next_id: std::sync::atomic::AtomicU64::new(0),
        }
    }

    /// Generate the next unique ID.
    pub fn next(&self) -> DeformableId {
        DeformableId(
            self.next_id
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        )
    }
}

/// Global ID generator for deformable bodies.
static ID_GENERATOR: DeformableIdGenerator = DeformableIdGenerator::new();

/// Generate a new unique deformable body ID.
pub fn next_deformable_id() -> DeformableId {
    ID_GENERATOR.next()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deformable_id() {
        let id = DeformableId::new(42);
        assert_eq!(id.raw(), 42);
        assert_eq!(id.to_string(), "Deformable(42)");

        let id2: DeformableId = 42.into();
        assert_eq!(id, id2);
    }

    #[test]
    fn test_vertex_new() {
        let v = Vertex::new(Point3::new(1.0, 2.0, 3.0), 2.0);
        assert_eq!(v.position.x, 1.0);
        assert_eq!(v.mass, 2.0);
        assert!((v.inv_mass - 0.5).abs() < 1e-10);
        assert!(!v.is_pinned());
    }

    #[test]
    fn test_vertex_pinned() {
        let v = Vertex::pinned(Point3::origin());
        assert!(v.is_pinned());
        assert_eq!(v.inv_mass, 0.0);
    }

    #[test]
    fn test_vertex_pin_unpin() {
        let mut v = Vertex::new(Point3::origin(), 1.0);
        assert!(!v.is_pinned());

        v.pin();
        assert!(v.is_pinned());
        assert_eq!(v.inv_mass, 0.0);

        v.unpin();
        assert!(!v.is_pinned());
        assert_eq!(v.inv_mass, 1.0);
    }

    #[test]
    fn test_vertex_force() {
        let mut v = Vertex::new(Point3::origin(), 1.0);

        v.apply_force(Vector3::new(1.0, 0.0, 0.0));
        v.apply_force(Vector3::new(0.0, 2.0, 0.0));

        assert_eq!(v.force.x, 1.0);
        assert_eq!(v.force.y, 2.0);

        v.clear_force();
        assert_eq!(v.force.norm(), 0.0);
    }

    #[test]
    fn test_id_generator() {
        let generator = DeformableIdGenerator::new();
        let id1 = generator.next();
        let id2 = generator.next();

        assert_eq!(id1.raw(), 0);
        assert_eq!(id2.raw(), 1);
    }

    #[test]
    fn test_vertex_flags() {
        let mut flags = VertexFlags::empty();
        assert!(!flags.contains(VertexFlags::PINNED));

        flags.insert(VertexFlags::PINNED | VertexFlags::BOUNDARY);
        assert!(flags.contains(VertexFlags::PINNED));
        assert!(flags.contains(VertexFlags::BOUNDARY));

        flags.remove(VertexFlags::PINNED);
        assert!(!flags.contains(VertexFlags::PINNED));
        assert!(flags.contains(VertexFlags::BOUNDARY));
    }
}
