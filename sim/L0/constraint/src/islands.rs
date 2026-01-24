//! Constraint islands for performance optimization.
//!
//! Constraint islands are groups of bodies connected by constraints that can be
//! solved independently. This module provides automatic island detection using
//! a union-find algorithm, enabling:
//!
//! - **Parallel solving**: Independent islands can be solved concurrently
//! - **Sleeping optimization**: Static islands can skip computation entirely
//! - **Reduced matrix size**: Smaller systems per island = faster solves
//!
//! # Algorithm
//!
//! Island detection uses a union-find (disjoint-set) data structure:
//!
//! 1. Each body starts in its own island
//! 2. For each constraint, union the islands of connected bodies
//! 3. Extract final island membership with path compression
//!
//! Time complexity: O(n × α(n)) ≈ O(n) where α is the inverse Ackermann function.
//!
//! # Example
//!
//! ```
//! use sim_constraint::{ConstraintIslands, RevoluteJoint, Joint};
//! use sim_types::BodyId;
//! use nalgebra::Vector3;
//!
//! // Create two disconnected chains
//! let joints = vec![
//!     RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
//!     RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
//!     // Gap - bodies 3 and 4 form a separate island
//!     RevoluteJoint::new(BodyId::new(3), BodyId::new(4), Vector3::z()),
//! ];
//!
//! let islands = ConstraintIslands::build(&joints);
//!
//! assert_eq!(islands.num_islands(), 2);
//! // Island 0: bodies 0, 1, 2
//! // Island 1: bodies 3, 4
//! ```

use sim_types::BodyId;
use std::collections::HashMap;

use crate::Joint;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A single constraint island containing connected bodies and their constraints.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Island {
    /// Bodies in this island, sorted by ID for determinism.
    pub bodies: Vec<BodyId>,

    /// Indices of constraints (joints) belonging to this island.
    /// These are indices into the original joints slice.
    pub constraint_indices: Vec<usize>,

    /// Whether this island is potentially sleeping (all bodies static or at rest).
    /// This is a hint that can be updated externally.
    pub is_static: bool,
}

impl Island {
    /// Create a new empty island.
    #[must_use]
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            constraint_indices: Vec::new(),
            is_static: false,
        }
    }

    /// Number of bodies in this island.
    #[must_use]
    pub fn num_bodies(&self) -> usize {
        self.bodies.len()
    }

    /// Number of constraints in this island.
    #[must_use]
    pub fn num_constraints(&self) -> usize {
        self.constraint_indices.len()
    }

    /// Check if this island contains a specific body.
    #[must_use]
    pub fn contains_body(&self, body: BodyId) -> bool {
        self.bodies.binary_search(&body).is_ok()
    }
}

impl Default for Island {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of constraint island detection.
///
/// This structure groups bodies and constraints into independent islands
/// that can be solved separately for improved performance.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConstraintIslands {
    /// The detected islands.
    islands: Vec<Island>,

    /// Mapping from body ID to island index.
    body_to_island: HashMap<BodyId, usize>,

    /// Total number of bodies across all islands.
    total_bodies: usize,

    /// Total number of constraints across all islands.
    total_constraints: usize,
}

impl ConstraintIslands {
    /// Build constraint islands from a set of joints.
    ///
    /// This uses union-find to efficiently group connected bodies.
    #[must_use]
    pub fn build<J: Joint>(joints: &[J]) -> Self {
        if joints.is_empty() {
            return Self::empty();
        }

        // Collect all unique bodies
        let mut body_ids: Vec<BodyId> = Vec::new();
        for joint in joints {
            if !body_ids.contains(&joint.parent()) {
                body_ids.push(joint.parent());
            }
            if !body_ids.contains(&joint.child()) {
                body_ids.push(joint.child());
            }
        }

        // Create body ID to index mapping
        let body_to_index: HashMap<BodyId, usize> = body_ids
            .iter()
            .enumerate()
            .map(|(i, &id)| (id, i))
            .collect();

        let num_bodies = body_ids.len();

        // Initialize union-find
        let mut uf = UnionFind::new(num_bodies);

        // Union bodies connected by constraints
        for joint in joints {
            if let (Some(&p_idx), Some(&c_idx)) = (
                body_to_index.get(&joint.parent()),
                body_to_index.get(&joint.child()),
            ) {
                uf.union(p_idx, c_idx);
            }
        }

        // Group bodies by root (island)
        let mut root_to_island: HashMap<usize, usize> = HashMap::new();
        let mut islands: Vec<Island> = Vec::new();
        let mut body_to_island: HashMap<BodyId, usize> = HashMap::new();

        for (body_idx, &body_id) in body_ids.iter().enumerate() {
            let root = uf.find(body_idx);
            let island_idx = if let Some(&idx) = root_to_island.get(&root) {
                idx
            } else {
                let idx = islands.len();
                islands.push(Island::new());
                root_to_island.insert(root, idx);
                idx
            };

            islands[island_idx].bodies.push(body_id);
            body_to_island.insert(body_id, island_idx);
        }

        // Assign constraints to islands
        for (joint_idx, joint) in joints.iter().enumerate() {
            // All bodies in the same constraint belong to the same island
            if let Some(&island_idx) = body_to_island.get(&joint.parent()) {
                islands[island_idx].constraint_indices.push(joint_idx);
            }
        }

        // Sort bodies within each island for determinism
        for island in &mut islands {
            island.bodies.sort();
        }

        let total_constraints = joints.len();

        Self {
            islands,
            body_to_island,
            total_bodies: num_bodies,
            total_constraints,
        }
    }

    /// Build islands with static body information.
    ///
    /// Islands where all bodies are static will be marked as static.
    #[must_use]
    pub fn build_with_static_info<J, F>(joints: &[J], is_body_static: F) -> Self
    where
        J: Joint,
        F: Fn(BodyId) -> bool,
    {
        let mut islands = Self::build(joints);

        // Mark islands as static if all bodies are static
        for island in &mut islands.islands {
            island.is_static = island.bodies.iter().all(|&id| is_body_static(id));
        }

        islands
    }

    /// Create an empty islands structure.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            islands: Vec::new(),
            body_to_island: HashMap::new(),
            total_bodies: 0,
            total_constraints: 0,
        }
    }

    /// Get the number of islands.
    #[must_use]
    pub fn num_islands(&self) -> usize {
        self.islands.len()
    }

    /// Get the total number of bodies.
    #[must_use]
    pub fn total_bodies(&self) -> usize {
        self.total_bodies
    }

    /// Get the total number of constraints.
    #[must_use]
    pub fn total_constraints(&self) -> usize {
        self.total_constraints
    }

    /// Get all islands.
    #[must_use]
    pub fn islands(&self) -> &[Island] {
        &self.islands
    }

    /// Get a specific island by index.
    #[must_use]
    pub fn get_island(&self, index: usize) -> Option<&Island> {
        self.islands.get(index)
    }

    /// Get the island index for a body.
    #[must_use]
    pub fn island_for_body(&self, body: BodyId) -> Option<usize> {
        self.body_to_island.get(&body).copied()
    }

    /// Check if two bodies are in the same island.
    #[must_use]
    pub fn same_island(&self, body_a: BodyId, body_b: BodyId) -> bool {
        match (
            self.body_to_island.get(&body_a),
            self.body_to_island.get(&body_b),
        ) {
            (Some(&a), Some(&b)) => a == b,
            _ => false,
        }
    }

    /// Get the number of non-static islands.
    #[must_use]
    pub fn num_active_islands(&self) -> usize {
        self.islands.iter().filter(|i| !i.is_static).count()
    }

    /// Iterate over non-static islands only.
    pub fn active_islands(&self) -> impl Iterator<Item = (usize, &Island)> {
        self.islands
            .iter()
            .enumerate()
            .filter(|(_, i)| !i.is_static)
    }

    /// Get statistics about island distribution.
    #[must_use]
    pub fn statistics(&self) -> IslandStatistics {
        if self.islands.is_empty() {
            return IslandStatistics::default();
        }

        let body_counts: Vec<usize> = self.islands.iter().map(Island::num_bodies).collect();
        let constraint_counts: Vec<usize> =
            self.islands.iter().map(Island::num_constraints).collect();

        let max_bodies = body_counts.iter().copied().max().unwrap_or(0);
        let max_constraints = constraint_counts.iter().copied().max().unwrap_or(0);
        #[allow(clippy::cast_precision_loss)]
        let avg_bodies = self.total_bodies as f64 / self.islands.len() as f64;
        #[allow(clippy::cast_precision_loss)]
        let avg_constraints = self.total_constraints as f64 / self.islands.len() as f64;

        IslandStatistics {
            num_islands: self.islands.len(),
            num_static_islands: self.islands.iter().filter(|i| i.is_static).count(),
            max_bodies_per_island: max_bodies,
            max_constraints_per_island: max_constraints,
            avg_bodies_per_island: avg_bodies,
            avg_constraints_per_island: avg_constraints,
        }
    }
}

/// Statistics about constraint island distribution.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IslandStatistics {
    /// Total number of islands.
    pub num_islands: usize,
    /// Number of static (sleeping) islands.
    pub num_static_islands: usize,
    /// Maximum bodies in any island.
    pub max_bodies_per_island: usize,
    /// Maximum constraints in any island.
    pub max_constraints_per_island: usize,
    /// Average bodies per island.
    pub avg_bodies_per_island: f64,
    /// Average constraints per island.
    pub avg_constraints_per_island: f64,
}

/// Union-Find (Disjoint-Set Union) data structure.
///
/// Used for efficient island detection with near-constant time operations.
struct UnionFind {
    /// Parent pointers (index of parent, or self if root).
    parent: Vec<usize>,
    /// Rank for union by rank optimization.
    rank: Vec<usize>,
}

impl UnionFind {
    /// Create a new union-find with n elements.
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    /// Find the root of the set containing element x, with path compression.
    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    /// Union the sets containing x and y, using union by rank.
    fn union(&mut self, x: usize, y: usize) {
        let root_x = self.find(x);
        let root_y = self.find(y);

        if root_x == root_y {
            return;
        }

        // Union by rank
        match self.rank[root_x].cmp(&self.rank[root_y]) {
            std::cmp::Ordering::Less => {
                self.parent[root_x] = root_y;
            }
            std::cmp::Ordering::Greater => {
                self.parent[root_y] = root_x;
            }
            std::cmp::Ordering::Equal => {
                self.parent[root_y] = root_x;
                self.rank[root_x] += 1;
            }
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use crate::RevoluteJoint;
    use nalgebra::Vector3;

    #[test]
    fn test_single_island_chain() {
        // Chain: 0 -- 1 -- 2
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 1);
        assert_eq!(islands.total_bodies(), 3);
        assert_eq!(islands.total_constraints(), 2);

        let island = islands.get_island(0).expect("Should have island 0");
        assert_eq!(island.num_bodies(), 3);
        assert_eq!(island.num_constraints(), 2);
    }

    #[test]
    fn test_two_separate_islands() {
        // Island 1: 0 -- 1
        // Island 2: 2 -- 3
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(2), BodyId::new(3), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 2);
        assert_eq!(islands.total_bodies(), 4);
        assert_eq!(islands.total_constraints(), 2);

        // Bodies 0 and 1 should be in same island
        assert!(islands.same_island(BodyId::new(0), BodyId::new(1)));
        // Bodies 2 and 3 should be in same island
        assert!(islands.same_island(BodyId::new(2), BodyId::new(3)));
        // Bodies 0 and 2 should be in different islands
        assert!(!islands.same_island(BodyId::new(0), BodyId::new(2)));
    }

    #[test]
    fn test_branching_structure() {
        // Tree structure:
        //     0
        //    / \
        //   1   2
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(0), BodyId::new(2), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 1);
        assert_eq!(islands.total_bodies(), 3);

        // All three bodies should be in the same island
        assert!(islands.same_island(BodyId::new(0), BodyId::new(1)));
        assert!(islands.same_island(BodyId::new(0), BodyId::new(2)));
        assert!(islands.same_island(BodyId::new(1), BodyId::new(2)));
    }

    #[test]
    fn test_empty_joints() {
        let joints: Vec<RevoluteJoint> = vec![];
        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 0);
        assert_eq!(islands.total_bodies(), 0);
        assert_eq!(islands.total_constraints(), 0);
    }

    #[test]
    fn test_cycle_structure() {
        // Cycle: 0 -- 1 -- 2 -- 0
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
            RevoluteJoint::new(BodyId::new(2), BodyId::new(0), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 1);
        assert_eq!(islands.total_bodies(), 3);
        assert_eq!(islands.total_constraints(), 3);
    }

    #[test]
    fn test_static_island_detection() {
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(2), BodyId::new(3), Vector3::z()),
        ];

        // Bodies 0 and 1 are dynamic, bodies 2 and 3 are static
        let islands =
            ConstraintIslands::build_with_static_info(&joints, |id| id.0 == 2 || id.0 == 3);

        assert_eq!(islands.num_islands(), 2);
        assert_eq!(islands.num_active_islands(), 1);

        // One island should be static
        let static_count = islands.islands().iter().filter(|i| i.is_static).count();
        assert_eq!(static_count, 1);
    }

    #[test]
    fn test_island_statistics() {
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
            RevoluteJoint::new(BodyId::new(3), BodyId::new(4), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);
        let stats = islands.statistics();

        assert_eq!(stats.num_islands, 2);
        assert_eq!(stats.max_bodies_per_island, 3); // Island with bodies 0,1,2
        assert_eq!(stats.max_constraints_per_island, 2);
    }

    #[test]
    fn test_island_body_lookup() {
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(2), BodyId::new(3), Vector3::z()),
        ];

        let islands = ConstraintIslands::build(&joints);

        // Bodies should map to their respective islands
        let island_0 = islands.island_for_body(BodyId::new(0));
        let island_1 = islands.island_for_body(BodyId::new(1));
        let island_2 = islands.island_for_body(BodyId::new(2));
        let island_3 = islands.island_for_body(BodyId::new(3));

        assert!(island_0.is_some());
        assert!(island_1.is_some());
        assert!(island_2.is_some());
        assert!(island_3.is_some());

        assert_eq!(island_0, island_1);
        assert_eq!(island_2, island_3);
        assert_ne!(island_0, island_2);

        // Unknown body should return None
        assert!(islands.island_for_body(BodyId::new(99)).is_none());
    }

    #[test]
    fn test_union_find_path_compression() {
        let mut uf = UnionFind::new(5);

        // Create chain: 0 -> 1 -> 2 -> 3 -> 4
        uf.union(0, 1);
        uf.union(1, 2);
        uf.union(2, 3);
        uf.union(3, 4);

        // All should have same root
        let root = uf.find(0);
        assert_eq!(uf.find(1), root);
        assert_eq!(uf.find(2), root);
        assert_eq!(uf.find(3), root);
        assert_eq!(uf.find(4), root);
    }

    #[test]
    fn test_large_island_count() {
        // Create many separate pairs
        let joints: Vec<RevoluteJoint> = (0..50)
            .map(|i| RevoluteJoint::new(BodyId::new(i * 2), BodyId::new(i * 2 + 1), Vector3::z()))
            .collect();

        let islands = ConstraintIslands::build(&joints);

        assert_eq!(islands.num_islands(), 50);
        assert_eq!(islands.total_bodies(), 100);
        assert_eq!(islands.total_constraints(), 50);

        // Each island should have exactly 2 bodies and 1 constraint
        for island in islands.islands() {
            assert_eq!(island.num_bodies(), 2);
            assert_eq!(island.num_constraints(), 1);
        }
    }

    #[test]
    fn test_active_islands_iterator() {
        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(2), BodyId::new(3), Vector3::z()),
            RevoluteJoint::new(BodyId::new(4), BodyId::new(5), Vector3::z()),
        ];

        // Make middle island static
        let islands =
            ConstraintIslands::build_with_static_info(&joints, |id| id.0 == 2 || id.0 == 3);

        assert_eq!(islands.active_islands().count(), 2);
    }
}
