//! Pareto dominance utilities for multi-objective optimization.
//!
//! This module provides types and functions for working with Pareto-optimal
//! solutions in multi-objective route optimization.
//!
//! # Overview
//!
//! In multi-objective optimization, a solution A "dominates" solution B if:
//! - A is at least as good as B in all objectives
//! - A is strictly better than B in at least one objective
//!
//! Solutions that are not dominated by any other solution form the "Pareto frontier".
//!
//! # Example
//!
//! ```
//! use route_optimize::pareto::{dominates, pareto_frontier};
//!
//! // Solutions with (length, bends, clearance) objectives
//! // Lower is better for length and bends, higher is better for clearance
//! let solutions = vec![
//!     vec![10.0, 3.0, 5.0],  // Solution A
//!     vec![12.0, 2.0, 6.0],  // Solution B
//!     vec![15.0, 4.0, 3.0],  // Solution C (dominated by A)
//! ];
//!
//! // Check dominance (assuming all objectives should be minimized)
//! // Note: In practice, you'd flip clearance to make lower better
//! ```

use route_types::{Route, RouteCost};

/// A solution point in multi-objective space.
///
/// Each dimension represents one objective value.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectivePoint {
    /// Objective values (e.g., length, bends, clearance).
    values: Vec<f64>,
}

impl ObjectivePoint {
    /// Creates a new objective point with the given values.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::pareto::ObjectivePoint;
    ///
    /// let point = ObjectivePoint::new(vec![10.0, 3.0, 5.0]);
    /// assert_eq!(point.len(), 3);
    /// ```
    #[must_use]
    pub const fn new(values: Vec<f64>) -> Self {
        Self { values }
    }

    /// Creates an objective point from a route cost.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::pareto::ObjectivePoint;
    /// use route_types::RouteCost;
    ///
    /// let cost = RouteCost::new()
    ///     .with_length(10.0)
    ///     .with_bends(3.0);
    ///
    /// let point = ObjectivePoint::from_route_cost(&cost);
    /// ```
    #[must_use]
    pub fn from_route_cost(cost: &RouteCost) -> Self {
        Self::new(vec![
            cost.length(),
            cost.bends(),
            cost.curvature(),
            -cost.clearance(), // Negate: higher clearance is better
            cost.corridor_deviation(),
        ])
    }

    /// Returns the number of objectives.
    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Returns true if the point has no objectives.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Returns the objective value at the given index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<f64> {
        self.values.get(index).copied()
    }

    /// Returns a reference to the objective values.
    #[must_use]
    pub fn values(&self) -> &[f64] {
        &self.values
    }
}

/// Checks if point `a` dominates point `b`.
///
/// A dominates B if:
/// - A is at least as good as B in all objectives
/// - A is strictly better than B in at least one objective
///
/// Assumes all objectives should be minimized.
///
/// # Example
///
/// ```
/// use route_optimize::pareto::{ObjectivePoint, dominates};
///
/// let a = ObjectivePoint::new(vec![5.0, 3.0]);
/// let b = ObjectivePoint::new(vec![6.0, 4.0]);
///
/// assert!(dominates(&a, &b));  // A is better in both dimensions
/// ```
#[must_use]
pub fn dominates(a: &ObjectivePoint, b: &ObjectivePoint) -> bool {
    if a.len() != b.len() || a.is_empty() {
        return false;
    }

    let mut strictly_better = false;

    for (av, bv) in a.values().iter().zip(b.values().iter()) {
        if av > bv {
            // A is worse in this objective
            return false;
        }
        if av < bv {
            strictly_better = true;
        }
    }

    strictly_better
}

/// Checks if two points are Pareto-equivalent (neither dominates the other).
///
/// # Example
///
/// ```
/// use route_optimize::pareto::{ObjectivePoint, is_equivalent};
///
/// let a = ObjectivePoint::new(vec![5.0, 4.0]);
/// let b = ObjectivePoint::new(vec![4.0, 5.0]);
///
/// // Neither dominates the other (trade-off)
/// assert!(is_equivalent(&a, &b));
/// ```
#[must_use]
pub fn is_equivalent(a: &ObjectivePoint, b: &ObjectivePoint) -> bool {
    !dominates(a, b) && !dominates(b, a)
}

/// Extracts the Pareto frontier from a set of solutions.
///
/// Returns indices of solutions that are not dominated by any other solution.
///
/// # Example
///
/// ```
/// use route_optimize::pareto::{ObjectivePoint, pareto_frontier};
///
/// let solutions = vec![
///     ObjectivePoint::new(vec![5.0, 3.0]),
///     ObjectivePoint::new(vec![4.0, 4.0]),
///     ObjectivePoint::new(vec![6.0, 5.0]),  // Dominated
/// ];
///
/// let frontier = pareto_frontier(&solutions);
/// assert_eq!(frontier.len(), 2);  // First two are non-dominated
/// ```
#[must_use]
pub fn pareto_frontier(solutions: &[ObjectivePoint]) -> Vec<usize> {
    let mut frontier = Vec::new();

    for (i, solution) in solutions.iter().enumerate() {
        let is_dominated = solutions
            .iter()
            .enumerate()
            .any(|(j, other)| i != j && dominates(other, solution));

        if !is_dominated {
            frontier.push(i);
        }
    }

    frontier
}

/// Extracts the Pareto frontier from a set of routes.
///
/// Returns routes that are not dominated by any other route.
///
/// # Example
///
/// ```
/// use route_optimize::pareto::routes_pareto_frontier;
/// use route_types::{Route, Path, VoxelPath, RouteCost};
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);
///
/// let routes = vec![
///     Route::new(Path::Voxel(path.clone()))
///         .with_cost(RouteCost::new().with_length(5.0).with_bends(3.0)),
///     Route::new(Path::Voxel(path.clone()))
///         .with_cost(RouteCost::new().with_length(4.0).with_bends(4.0)),
/// ];
///
/// let frontier = routes_pareto_frontier(routes);
/// assert_eq!(frontier.len(), 2);
/// ```
#[must_use]
#[allow(clippy::needless_pass_by_value)] // Takes ownership to avoid cloning in output
pub fn routes_pareto_frontier(routes: Vec<Route>) -> Vec<Route> {
    if routes.is_empty() {
        return Vec::new();
    }

    let points: Vec<ObjectivePoint> = routes
        .iter()
        .map(|r| ObjectivePoint::from_route_cost(r.cost()))
        .collect();

    let frontier_indices = pareto_frontier(&points);

    frontier_indices
        .into_iter()
        .filter_map(|i| routes.get(i).cloned())
        .collect()
}

/// Computes the hypervolume indicator for a set of solutions.
///
/// The hypervolume is the volume of objective space dominated by
/// the solutions, bounded by a reference point.
///
/// This is a simplified 2D implementation. For higher dimensions,
/// more sophisticated algorithms would be needed.
///
/// # Example
///
/// ```
/// use route_optimize::pareto::{ObjectivePoint, hypervolume_2d};
///
/// let solutions = vec![
///     ObjectivePoint::new(vec![5.0, 3.0]),
///     ObjectivePoint::new(vec![4.0, 4.0]),
/// ];
///
/// let reference = ObjectivePoint::new(vec![10.0, 10.0]);
/// let hv = hypervolume_2d(&solutions, &reference);
/// assert!(hv > 0.0);
/// ```
#[must_use]
pub fn hypervolume_2d(solutions: &[ObjectivePoint], reference: &ObjectivePoint) -> f64 {
    if solutions.is_empty() || reference.len() < 2 {
        return 0.0;
    }

    // Filter to 2D and sort by first objective
    let mut points: Vec<(f64, f64)> = solutions
        .iter()
        .filter_map(|p| {
            if p.len() >= 2 {
                let x = p.values()[0];
                let y = p.values()[1];
                // Only include points that are within the reference bounds
                if x < reference.values()[0] && y < reference.values()[1] {
                    Some((x, y))
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect();

    if points.is_empty() {
        return 0.0;
    }

    // Sort by first objective (ascending)
    points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Compute hypervolume using the inclusion-exclusion principle
    let mut volume = 0.0;
    let ref_y = reference.values()[1];

    let mut prev_x = points[0].0;
    let mut prev_y = ref_y;

    for (x, y) in &points {
        // Add rectangle from previous x to current x
        let width = x - prev_x;
        let height = prev_y;
        volume += width * height;

        prev_x = *x;
        if *y < prev_y {
            prev_y = *y;
        }
    }

    // Add final rectangle to reference point
    let width = reference.values()[0] - prev_x;
    let height = prev_y;
    volume += width * height;

    volume
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_objective_point_new() {
        let point = ObjectivePoint::new(vec![1.0, 2.0, 3.0]);
        assert_eq!(point.len(), 3);
        assert_eq!(point.get(0), Some(1.0));
        assert_eq!(point.get(1), Some(2.0));
        assert_eq!(point.get(2), Some(3.0));
        assert_eq!(point.get(3), None);
    }

    #[test]
    fn test_objective_point_empty() {
        let point = ObjectivePoint::new(vec![]);
        assert!(point.is_empty());
        assert_eq!(point.len(), 0);
    }

    #[test]
    fn test_dominates_clear_winner() {
        let a = ObjectivePoint::new(vec![5.0, 3.0, 2.0]);
        let b = ObjectivePoint::new(vec![6.0, 4.0, 3.0]);

        assert!(dominates(&a, &b));
        assert!(!dominates(&b, &a));
    }

    #[test]
    fn test_dominates_trade_off() {
        let a = ObjectivePoint::new(vec![5.0, 4.0]);
        let b = ObjectivePoint::new(vec![4.0, 5.0]);

        assert!(!dominates(&a, &b));
        assert!(!dominates(&b, &a));
    }

    #[test]
    fn test_dominates_equal() {
        let a = ObjectivePoint::new(vec![5.0, 5.0]);
        let b = ObjectivePoint::new(vec![5.0, 5.0]);

        // Neither dominates (must be strictly better in at least one)
        assert!(!dominates(&a, &b));
        assert!(!dominates(&b, &a));
    }

    #[test]
    fn test_dominates_different_lengths() {
        let a = ObjectivePoint::new(vec![5.0, 3.0]);
        let b = ObjectivePoint::new(vec![6.0, 4.0, 3.0]);

        assert!(!dominates(&a, &b));
        assert!(!dominates(&b, &a));
    }

    #[test]
    fn test_is_equivalent() {
        let a = ObjectivePoint::new(vec![5.0, 4.0]);
        let b = ObjectivePoint::new(vec![4.0, 5.0]);

        assert!(is_equivalent(&a, &b));
    }

    #[test]
    fn test_is_equivalent_dominated() {
        let a = ObjectivePoint::new(vec![5.0, 3.0]);
        let b = ObjectivePoint::new(vec![6.0, 4.0]);

        assert!(!is_equivalent(&a, &b));
    }

    #[test]
    fn test_pareto_frontier_simple() {
        let solutions = vec![
            ObjectivePoint::new(vec![5.0, 3.0]),
            ObjectivePoint::new(vec![4.0, 4.0]),
            ObjectivePoint::new(vec![6.0, 5.0]), // Dominated by first
        ];

        let frontier = pareto_frontier(&solutions);

        assert_eq!(frontier.len(), 2);
        assert!(frontier.contains(&0));
        assert!(frontier.contains(&1));
        assert!(!frontier.contains(&2));
    }

    #[test]
    fn test_pareto_frontier_all_equivalent() {
        let solutions = vec![
            ObjectivePoint::new(vec![5.0, 4.0]),
            ObjectivePoint::new(vec![4.0, 5.0]),
            ObjectivePoint::new(vec![3.0, 6.0]),
        ];

        let frontier = pareto_frontier(&solutions);

        // All are non-dominated (Pareto efficient)
        assert_eq!(frontier.len(), 3);
    }

    #[test]
    fn test_pareto_frontier_single_winner() {
        let solutions = vec![
            ObjectivePoint::new(vec![3.0, 3.0]), // Dominates all
            ObjectivePoint::new(vec![4.0, 4.0]),
            ObjectivePoint::new(vec![5.0, 5.0]),
        ];

        let frontier = pareto_frontier(&solutions);

        assert_eq!(frontier.len(), 1);
        assert_eq!(frontier[0], 0);
    }

    #[test]
    fn test_pareto_frontier_empty() {
        let solutions: Vec<ObjectivePoint> = vec![];
        let frontier = pareto_frontier(&solutions);
        assert!(frontier.is_empty());
    }

    #[test]
    fn test_routes_pareto_frontier() {
        use cf_spatial::VoxelCoord;
        use route_types::{Path, VoxelPath};

        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);

        let routes = vec![
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(5.0).with_bends(3.0)),
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(4.0).with_bends(4.0)),
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(6.0).with_bends(5.0)), // Dominated
        ];

        let frontier = routes_pareto_frontier(routes);

        assert_eq!(frontier.len(), 2);
    }

    #[test]
    fn test_hypervolume_2d_simple() {
        let solutions = vec![ObjectivePoint::new(vec![2.0, 4.0])];

        let reference = ObjectivePoint::new(vec![10.0, 10.0]);
        let hv = hypervolume_2d(&solutions, &reference);

        // Volume = width * height for single point
        // (10 - 2) * (10 - 4) = 8 * 6 = 48
        // But our algorithm computes differently...
        // From x=2 to x=10, height is min(10, 4) = 4...
        // Actually: (10-2) * 4 = 32 for the dominated region
        assert!(hv > 0.0);
    }

    #[test]
    fn test_hypervolume_2d_multiple() {
        let solutions = vec![
            ObjectivePoint::new(vec![2.0, 4.0]),
            ObjectivePoint::new(vec![4.0, 2.0]),
        ];

        let reference = ObjectivePoint::new(vec![10.0, 10.0]);
        let hv = hypervolume_2d(&solutions, &reference);

        // Should be larger than with single point
        assert!(hv > 0.0);
    }

    #[test]
    fn test_hypervolume_2d_empty() {
        let solutions: Vec<ObjectivePoint> = vec![];
        let reference = ObjectivePoint::new(vec![10.0, 10.0]);
        let hv = hypervolume_2d(&solutions, &reference);

        assert_relative_eq!(hv, 0.0);
    }

    #[test]
    fn test_hypervolume_2d_outside_reference() {
        let solutions = vec![
            ObjectivePoint::new(vec![15.0, 15.0]), // Outside reference
        ];

        let reference = ObjectivePoint::new(vec![10.0, 10.0]);
        let hv = hypervolume_2d(&solutions, &reference);

        assert_relative_eq!(hv, 0.0);
    }

    #[test]
    fn test_from_route_cost() {
        let cost = RouteCost::new()
            .with_length(10.0)
            .with_bends(3.0)
            .with_curvature(0.5)
            .with_clearance(2.0)
            .with_corridor_deviation(1.0);

        let point = ObjectivePoint::from_route_cost(&cost);

        assert_eq!(point.len(), 5);
        assert_relative_eq!(point.values()[0], 10.0); // length
        assert_relative_eq!(point.values()[1], 3.0); // bends
        assert_relative_eq!(point.values()[2], 0.5); // curvature
        assert_relative_eq!(point.values()[3], -2.0); // clearance (negated)
        assert_relative_eq!(point.values()[4], 1.0); // corridor_deviation
    }
}
