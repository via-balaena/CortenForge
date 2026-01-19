//! Neighbor generation for voxel-based pathfinding.
//!
//! This module provides utilities for generating valid neighboring voxels
//! during pathfinding, with support for constraint checking.
//!
//! # Example
//!
//! ```
//! use route_pathfind::neighbors::NeighborGenerator;
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! let obstacles: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let generator = NeighborGenerator::new(&obstacles)
//!     .with_diagonal(true);
//!
//! let coord = VoxelCoord::new(5, 5, 5);
//! let neighbors: Vec<_> = generator.neighbors(coord).collect();
//! assert_eq!(neighbors.len(), 26);  // All 26 neighbors (empty grid)
//! ```

use cf_spatial::{VoxelCoord, VoxelGrid};

use crate::heuristics::move_cost;

/// Generator for valid neighboring voxels during pathfinding.
///
/// Handles connectivity (6 vs 26 neighbors) and obstacle checking.
///
/// # Example
///
/// ```
/// use route_pathfind::neighbors::NeighborGenerator;
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let mut grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
/// grid.set(VoxelCoord::new(6, 5, 5), true);  // Obstacle
///
/// let generator = NeighborGenerator::new(&grid).with_diagonal(false);
///
/// let coord = VoxelCoord::new(5, 5, 5);
/// let neighbors: Vec<_> = generator.neighbors(coord).collect();
///
/// // Should have 5 neighbors (6 - 1 blocked)
/// assert_eq!(neighbors.len(), 5);
/// ```
pub struct NeighborGenerator<'a, T> {
    /// The obstacle grid.
    grid: &'a VoxelGrid<T>,
    /// Whether to allow diagonal movement.
    allow_diagonal: bool,
    /// Clearance radius (in voxels) - not yet implemented
    #[allow(dead_code)]
    clearance: i32,
}

impl<T> NeighborGenerator<'_, T> {
    /// Creates a new neighbor generator with the given obstacle grid.
    ///
    /// Defaults to 26-connectivity (diagonal allowed).
    #[must_use]
    pub const fn new(grid: &VoxelGrid<T>) -> NeighborGenerator<'_, T> {
        NeighborGenerator {
            grid,
            allow_diagonal: true,
            clearance: 0,
        }
    }

    /// Sets whether diagonal movement is allowed.
    ///
    /// - `true`: 26-connectivity (all 26 neighbors)
    /// - `false`: 6-connectivity (face neighbors only)
    #[must_use]
    pub const fn with_diagonal(mut self, allow: bool) -> Self {
        self.allow_diagonal = allow;
        self
    }

    /// Sets the clearance radius in voxels.
    ///
    /// When clearance > 0, neighbors are only valid if all voxels
    /// within the clearance radius are free.
    ///
    /// Note: Clearance checking is not yet implemented.
    #[must_use]
    pub const fn with_clearance(mut self, voxels: i32) -> Self {
        self.clearance = voxels;
        self
    }

    /// Returns the number of potential neighbors based on connectivity.
    #[must_use]
    pub const fn neighbor_count(&self) -> usize {
        if self.allow_diagonal { 26 } else { 6 }
    }
}

impl<T: Default> NeighborGenerator<'_, T> {
    /// Checks if a voxel coordinate is blocked (occupied).
    ///
    /// Returns `true` if the voxel contains an obstacle.
    #[must_use]
    pub fn is_blocked(&self, coord: VoxelCoord) -> bool {
        self.grid.contains(coord)
    }

    /// Checks if a voxel coordinate is free (not occupied).
    #[must_use]
    pub fn is_free(&self, coord: VoxelCoord) -> bool {
        !self.is_blocked(coord)
    }

    /// Returns an iterator over valid (unblocked) neighbors of a coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::neighbors::NeighborGenerator;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let generator = NeighborGenerator::new(&grid).with_diagonal(false);
    ///
    /// let neighbors: Vec<_> = generator
    ///     .neighbors(VoxelCoord::new(0, 0, 0))
    ///     .collect();
    /// assert_eq!(neighbors.len(), 6);
    /// ```
    pub fn neighbors(&self, coord: VoxelCoord) -> impl Iterator<Item = VoxelCoord> + '_ {
        let candidates: Box<dyn Iterator<Item = VoxelCoord>> = if self.allow_diagonal {
            Box::new(coord.all_neighbors().into_iter())
        } else {
            Box::new(coord.face_neighbors().into_iter())
        };

        candidates.filter(|&n| self.is_free(n))
    }

    /// Returns an iterator over valid neighbors with their move costs.
    ///
    /// This is the primary interface for A* pathfinding.
    ///
    /// # Example
    ///
    /// ```
    /// use route_pathfind::neighbors::NeighborGenerator;
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let generator = NeighborGenerator::new(&grid).with_diagonal(false);
    ///
    /// let successors: Vec<_> = generator
    ///     .successors(VoxelCoord::new(0, 0, 0))
    ///     .collect();
    ///
    /// // All 6 face neighbors with cost 1.0
    /// assert_eq!(successors.len(), 6);
    /// for (_, cost) in &successors {
    ///     assert!((*cost - 1.0).abs() < 1e-10);
    /// }
    /// ```
    pub fn successors(&self, coord: VoxelCoord) -> impl Iterator<Item = (VoxelCoord, f64)> + '_ {
        self.neighbors(coord).map(move |n| (n, move_cost(coord, n)))
    }
}

/// Computes neighbors and costs for use with the `pathfinding` crate.
///
/// This function is designed to be used directly with `pathfinding::prelude::astar`.
///
/// # Example
///
/// ```
/// use route_pathfind::neighbors::successors_for_grid;
/// use cf_spatial::{VoxelCoord, VoxelGrid};
///
/// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
///
/// let coord = VoxelCoord::new(5, 5, 5);
/// let successors = successors_for_grid(&coord, &grid, true);
///
/// assert_eq!(successors.len(), 26);
/// ```
#[must_use]
pub fn successors_for_grid<T: Default>(
    coord: &VoxelCoord,
    grid: &VoxelGrid<T>,
    allow_diagonal: bool,
) -> Vec<(VoxelCoord, u64)> {
    let generator = NeighborGenerator::new(grid).with_diagonal(allow_diagonal);

    generator
        .successors(*coord)
        .map(|(n, cost)| {
            // Convert f64 cost to u64 by scaling (1000x for precision)
            // This matches how pathfinding crate typically handles costs
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let scaled_cost = (cost * 1000.0).round() as u64;
            (n, scaled_cost)
        })
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn empty_grid() -> VoxelGrid<bool> {
        VoxelGrid::new(1.0)
    }

    fn grid_with_obstacle(coord: VoxelCoord) -> VoxelGrid<bool> {
        let mut grid = VoxelGrid::new(1.0);
        grid.set(coord, true);
        grid
    }

    #[test]
    fn test_neighbor_generator_diagonal() {
        let grid = empty_grid();
        let generator = NeighborGenerator::new(&grid).with_diagonal(true);

        let neighbors: Vec<_> = generator.neighbors(VoxelCoord::new(5, 5, 5)).collect();
        assert_eq!(neighbors.len(), 26);
    }

    #[test]
    fn test_neighbor_generator_no_diagonal() {
        let grid = empty_grid();
        let generator = NeighborGenerator::new(&grid).with_diagonal(false);

        let neighbors: Vec<_> = generator.neighbors(VoxelCoord::new(5, 5, 5)).collect();
        assert_eq!(neighbors.len(), 6);
    }

    #[test]
    fn test_neighbor_generator_blocked() {
        let grid = grid_with_obstacle(VoxelCoord::new(6, 5, 5));
        let generator = NeighborGenerator::new(&grid).with_diagonal(false);

        let neighbors: Vec<_> = generator.neighbors(VoxelCoord::new(5, 5, 5)).collect();
        assert_eq!(neighbors.len(), 5); // 6 - 1 blocked

        // Verify blocked neighbor is not included
        assert!(!neighbors.contains(&VoxelCoord::new(6, 5, 5)));
    }

    #[test]
    fn test_neighbor_generator_blocked_diagonal() {
        let grid = grid_with_obstacle(VoxelCoord::new(6, 6, 6));
        let generator = NeighborGenerator::new(&grid).with_diagonal(true);

        let neighbors: Vec<_> = generator.neighbors(VoxelCoord::new(5, 5, 5)).collect();
        assert_eq!(neighbors.len(), 25); // 26 - 1 blocked

        // Verify blocked neighbor is not included
        assert!(!neighbors.contains(&VoxelCoord::new(6, 6, 6)));
    }

    #[test]
    fn test_is_blocked() {
        let grid = grid_with_obstacle(VoxelCoord::new(5, 5, 5));
        let generator = NeighborGenerator::new(&grid);

        assert!(generator.is_blocked(VoxelCoord::new(5, 5, 5)));
        assert!(!generator.is_blocked(VoxelCoord::new(0, 0, 0)));
    }

    #[test]
    fn test_is_free() {
        let grid = grid_with_obstacle(VoxelCoord::new(5, 5, 5));
        let generator = NeighborGenerator::new(&grid);

        assert!(!generator.is_free(VoxelCoord::new(5, 5, 5)));
        assert!(generator.is_free(VoxelCoord::new(0, 0, 0)));
    }

    #[test]
    fn test_successors_face_cost() {
        let grid = empty_grid();
        let generator = NeighborGenerator::new(&grid).with_diagonal(false);

        let successors: Vec<_> = generator.successors(VoxelCoord::new(0, 0, 0)).collect();

        // All face neighbors should have cost 1.0
        for (_, cost) in &successors {
            assert_relative_eq!(*cost, 1.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_successors_diagonal_costs() {
        let grid = empty_grid();
        let generator = NeighborGenerator::new(&grid).with_diagonal(true);

        let coord = VoxelCoord::new(0, 0, 0);
        let successors: Vec<_> = generator.successors(coord).collect();

        // Count different cost types
        let face_count = successors
            .iter()
            .filter(|(_, c)| (*c - 1.0).abs() < 1e-10)
            .count();
        let edge_count = successors
            .iter()
            .filter(|(_, c)| (*c - std::f64::consts::SQRT_2).abs() < 1e-10)
            .count();
        let corner_count = successors
            .iter()
            .filter(|(_, c)| (*c - 3.0_f64.sqrt()).abs() < 1e-10)
            .count();

        assert_eq!(face_count, 6); // 6 face neighbors
        assert_eq!(edge_count, 12); // 12 edge neighbors
        assert_eq!(corner_count, 8); // 8 corner neighbors
    }

    #[test]
    fn test_neighbor_count() {
        let grid = empty_grid();

        let gen_diag = NeighborGenerator::new(&grid).with_diagonal(true);
        assert_eq!(gen_diag.neighbor_count(), 26);

        let gen_no_diag = NeighborGenerator::new(&grid).with_diagonal(false);
        assert_eq!(gen_no_diag.neighbor_count(), 6);
    }

    #[test]
    fn test_successors_for_grid() {
        let grid = empty_grid();

        let coord = VoxelCoord::new(5, 5, 5);
        let successors = successors_for_grid(&coord, &grid, true);

        assert_eq!(successors.len(), 26);

        // Check that costs are scaled properly
        // Face neighbors should have cost 1000 (1.0 * 1000)
        let face_neighbor = VoxelCoord::new(6, 5, 5);
        let face_entry = successors.iter().find(|(n, _)| *n == face_neighbor);
        assert!(face_entry.is_some());
        assert_eq!(face_entry.unwrap().1, 1000);
    }

    #[test]
    fn test_successors_for_grid_blocked() {
        let grid = grid_with_obstacle(VoxelCoord::new(6, 5, 5));

        let coord = VoxelCoord::new(5, 5, 5);
        let successors = successors_for_grid(&coord, &grid, false);

        assert_eq!(successors.len(), 5); // 6 - 1 blocked

        // Verify blocked neighbor is not included
        let blocked = VoxelCoord::new(6, 5, 5);
        assert!(!successors.iter().any(|(n, _)| *n == blocked));
    }

    #[test]
    fn test_clearance_setting() {
        let grid = empty_grid();
        let generator = NeighborGenerator::new(&grid).with_clearance(2);

        // Clearance is stored but not yet used in filtering
        // This test just verifies the builder pattern works
        let neighbors: Vec<_> = generator.neighbors(VoxelCoord::new(5, 5, 5)).collect();
        assert!(!neighbors.is_empty());
    }
}
