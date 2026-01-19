//! Multi-objective route optimization.
//!
//! This module provides algorithms for optimizing routes across multiple
//! objectives simultaneously, generating Pareto-optimal solutions.
//!
//! # Overview
//!
//! Route optimization typically involves trade-offs between multiple objectives:
//! - **Length**: Shorter paths are generally preferred
//! - **Clearance**: Distance from obstacles
//! - **Bends**: Number of direction changes
//! - **Curvature**: Smoothness of the path
//!
//! Rather than combining these into a single weighted sum, multi-objective
//! optimization generates a set of Pareto-optimal solutions representing
//! different trade-offs.
//!
//! # Example
//!
//! ```
//! use route_optimize::multi_objective::MultiObjectiveOptimizer;
//! use route_types::{VoxelPath, Route, Path};
//! use cf_spatial::{VoxelCoord, VoxelGrid};
//!
//! // Create an optimizer
//! let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
//! let optimizer = MultiObjectiveOptimizer::new(&grid);
//!
//! // Optimize a route
//! let path = VoxelPath::new(vec![
//!     VoxelCoord::new(0, 0, 0),
//!     VoxelCoord::new(5, 0, 0),
//! ]);
//! let route = Route::new(Path::Voxel(path));
//!
//! let pareto_set = optimizer.optimize(&route);
//! ```

use cf_spatial::VoxelGrid;
use rand::Rng;
use route_types::{CostWeights, Path, Route, RouteCost};

use crate::clearance::ClearanceOptimizer;
use crate::curvature::{count_bends, max_curvature};
use crate::pareto::routes_pareto_frontier;
use crate::shorten::PathShortener;

/// Multi-objective optimizer for routes.
///
/// Generates a set of Pareto-optimal routes by exploring different
/// combinations of optimization strategies.
pub struct MultiObjectiveOptimizer<'a, T> {
    /// The obstacle grid.
    grid: &'a VoxelGrid<T>,
    /// Number of weight samples to explore.
    num_samples: usize,
}

impl<'a, T: Default> MultiObjectiveOptimizer<'a, T> {
    /// Creates a new multi-objective optimizer.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::multi_objective::MultiObjectiveOptimizer;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = MultiObjectiveOptimizer::new(&grid);
    /// ```
    #[must_use]
    pub const fn new(grid: &'a VoxelGrid<T>) -> Self {
        Self {
            grid,
            num_samples: 10,
        }
    }

    /// Sets the number of weight samples to explore.
    ///
    /// More samples produce a denser approximation of the Pareto frontier.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::multi_objective::MultiObjectiveOptimizer;
    /// use cf_spatial::VoxelGrid;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = MultiObjectiveOptimizer::new(&grid).with_samples(20);
    /// ```
    #[must_use]
    pub const fn with_samples(mut self, samples: usize) -> Self {
        self.num_samples = samples;
        self
    }

    /// Computes the objective values for a route.
    fn compute_objectives(&self, route: &Route) -> RouteCost {
        let path = route.path();

        let length = path.length();
        #[allow(clippy::cast_precision_loss)]
        let bends = match path {
            Path::Voxel(vp) => count_bends(vp, 0.1) as f64,
            Path::Continuous(_) => 0.0, // TODO: implement for continuous
        };
        let curvature = match path {
            Path::Voxel(vp) => max_curvature(vp),
            Path::Continuous(_) => 0.0,
        };
        let clearance = match path {
            Path::Voxel(vp) => {
                let optimizer = ClearanceOptimizer::new(self.grid);
                -optimizer.minimum_clearance(vp) // Negate so lower is better
            }
            Path::Continuous(_) => 0.0,
        };

        RouteCost::new()
            .with_length(length)
            .with_bends(bends)
            .with_curvature(curvature)
            .with_clearance(clearance)
    }

    /// Optimizes a route with a specific weight configuration.
    fn optimize_with_weights(&self, route: &Route, weights: &CostWeights) -> Route {
        let path = route.path();

        // Apply different optimizations based on weights
        let optimized_path = match path {
            Path::Voxel(vp) => {
                let mut current = vp.clone();

                // If length is prioritized, shorten the path
                if weights.length() > 0.5 {
                    let shortener = PathShortener::new(self.grid);
                    current = shortener.shorten(&current);
                }

                // If clearance is prioritized, optimize for clearance
                if weights.clearance() > 0.5 {
                    let optimizer = ClearanceOptimizer::new(self.grid);
                    current = optimizer.optimize(&current, 3);
                }

                Path::Voxel(current)
            }
            Path::Continuous(cp) => Path::Continuous(cp.clone()),
        };

        let cost = self.compute_objectives(&Route::new(optimized_path.clone()));

        Route::new(optimized_path)
            .with_cost(cost)
            .with_stats(route.stats().clone())
            .with_constraints_satisfied(route.constraints_satisfied())
    }

    /// Optimizes a route to produce a Pareto-optimal set.
    ///
    /// Explores different weight combinations and returns non-dominated solutions.
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::multi_objective::MultiObjectiveOptimizer;
    /// use route_types::{VoxelPath, Route, Path};
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = MultiObjectiveOptimizer::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(1, 0, 0),
    ///     VoxelCoord::new(2, 0, 0),
    ///     VoxelCoord::new(3, 0, 0),
    /// ]);
    /// let route = Route::new(Path::Voxel(path));
    ///
    /// let pareto_set = optimizer.optimize(&route);
    /// assert!(!pareto_set.is_empty());
    /// ```
    #[must_use]
    pub fn optimize(&self, route: &Route) -> Vec<Route> {
        let mut candidates = Vec::new();

        // Generate weight configurations
        let weight_configs = self.generate_weight_configs();

        // Optimize with each weight configuration
        for weights in weight_configs {
            let optimized = self.optimize_with_weights(route, &weights);
            candidates.push(optimized);
        }

        // Also include the original route
        let original_cost = self.compute_objectives(route);
        candidates.push(
            Route::new(route.path().clone())
                .with_cost(original_cost)
                .with_stats(route.stats().clone())
                .with_constraints_satisfied(route.constraints_satisfied()),
        );

        // Extract Pareto frontier
        routes_pareto_frontier(candidates)
    }

    /// Generates a set of weight configurations to explore.
    fn generate_weight_configs(&self) -> Vec<CostWeights> {
        let mut configs = Vec::new();

        // Uniform sampling in the weight simplex
        for i in 0..self.num_samples {
            #[allow(clippy::cast_precision_loss)]
            let t = (i as f64) / (self.num_samples.max(1) as f64);

            // Simple linear interpolation between extremes
            let length_weight = t;
            let clearance_weight = 1.0 - t;

            configs.push(
                CostWeights::default()
                    .with_length(length_weight)
                    .with_clearance(clearance_weight)
                    .with_bends(0.5)
                    .with_curvature(0.3),
            );
        }

        // Add some extreme configurations
        configs.push(CostWeights::default().with_length(1.0));
        configs.push(CostWeights::default().with_clearance(1.0));
        configs.push(CostWeights::default().with_bends(1.0));

        configs
    }

    /// Optimizes using random sampling in the weight space.
    ///
    /// # Arguments
    ///
    /// * `route` - The route to optimize
    /// * `samples` - Number of random samples
    /// * `rng` - Random number generator
    ///
    /// # Example
    ///
    /// ```
    /// use route_optimize::multi_objective::MultiObjectiveOptimizer;
    /// use route_types::{VoxelPath, Route, Path};
    /// use cf_spatial::{VoxelCoord, VoxelGrid};
    /// use rand::SeedableRng;
    /// use rand::rngs::StdRng;
    ///
    /// let grid: VoxelGrid<bool> = VoxelGrid::new(1.0);
    /// let optimizer = MultiObjectiveOptimizer::new(&grid);
    ///
    /// let path = VoxelPath::new(vec![
    ///     VoxelCoord::new(0, 0, 0),
    ///     VoxelCoord::new(5, 0, 0),
    /// ]);
    /// let route = Route::new(Path::Voxel(path));
    ///
    /// let mut rng = StdRng::seed_from_u64(42);
    /// let pareto_set = optimizer.optimize_random(&route, 10, &mut rng);
    /// ```
    #[must_use]
    pub fn optimize_random<R: Rng>(
        &self,
        route: &Route,
        samples: usize,
        rng: &mut R,
    ) -> Vec<Route> {
        let mut candidates = Vec::new();

        for _ in 0..samples {
            let weights = CostWeights::default()
                .with_length(rng.r#gen())
                .with_clearance(rng.r#gen())
                .with_bends(rng.r#gen())
                .with_curvature(rng.r#gen());

            let optimized = self.optimize_with_weights(route, &weights);
            candidates.push(optimized);
        }

        // Include original
        let original_cost = self.compute_objectives(route);
        candidates.push(
            Route::new(route.path().clone())
                .with_cost(original_cost)
                .with_stats(route.stats().clone())
                .with_constraints_satisfied(route.constraints_satisfied()),
        );

        routes_pareto_frontier(candidates)
    }
}

/// Evaluates multiple routes and returns the Pareto-optimal subset.
///
/// # Example
///
/// ```
/// use route_optimize::multi_objective::filter_pareto;
/// use route_types::{VoxelPath, Route, Path, RouteCost};
/// use cf_spatial::VoxelCoord;
///
/// let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);
///
/// let routes = vec![
///     Route::new(Path::Voxel(path.clone()))
///         .with_cost(RouteCost::new().with_length(5.0).with_bends(3.0)),
///     Route::new(Path::Voxel(path.clone()))
///         .with_cost(RouteCost::new().with_length(4.0).with_bends(4.0)),
///     Route::new(Path::Voxel(path.clone()))
///         .with_cost(RouteCost::new().with_length(6.0).with_bends(5.0)),
/// ];
///
/// let pareto = filter_pareto(routes);
/// assert!(pareto.len() <= 3);
/// ```
#[must_use]
pub fn filter_pareto(routes: Vec<Route>) -> Vec<Route> {
    routes_pareto_frontier(routes)
}

/// Selects the best route from a Pareto set using weighted selection.
///
/// # Arguments
///
/// * `routes` - Set of Pareto-optimal routes
/// * `weights` - Weights for combining objectives
///
/// # Returns
///
/// The route with the lowest weighted cost.
///
/// # Example
///
/// ```
/// use route_optimize::multi_objective::select_best;
/// use route_types::{VoxelPath, Route, Path, RouteCost, CostWeights};
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
/// let weights = CostWeights::default().with_length(1.0);
/// let best = select_best(&routes, &weights);
///
/// assert!(best.is_some());
/// ```
#[must_use]
pub fn select_best(routes: &[Route], weights: &CostWeights) -> Option<Route> {
    routes
        .iter()
        .min_by(|a, b| {
            let cost_a = a.cost().compute_weighted(weights);
            let cost_b = b.cost().compute_weighted(weights);
            cost_a
                .partial_cmp(&cost_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .cloned()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use cf_spatial::VoxelCoord;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use route_types::VoxelPath;

    fn empty_grid() -> VoxelGrid<bool> {
        VoxelGrid::new(1.0)
    }

    fn simple_route() -> Route {
        let path = VoxelPath::new(vec![
            VoxelCoord::new(0, 0, 0),
            VoxelCoord::new(1, 0, 0),
            VoxelCoord::new(2, 0, 0),
            VoxelCoord::new(3, 0, 0),
        ]);
        Route::new(Path::Voxel(path))
    }

    #[test]
    fn test_multi_objective_optimizer_new() {
        let grid = empty_grid();
        let optimizer = MultiObjectiveOptimizer::new(&grid);

        let route = simple_route();
        let pareto = optimizer.optimize(&route);

        assert!(!pareto.is_empty());
    }

    #[test]
    fn test_multi_objective_optimizer_with_samples() {
        let grid = empty_grid();
        let optimizer = MultiObjectiveOptimizer::new(&grid).with_samples(5);

        let route = simple_route();
        let pareto = optimizer.optimize(&route);

        assert!(!pareto.is_empty());
    }

    #[test]
    fn test_compute_objectives() {
        let grid = empty_grid();
        let optimizer = MultiObjectiveOptimizer::new(&grid);

        let route = simple_route();
        let cost = optimizer.compute_objectives(&route);

        assert!(cost.length() > 0.0);
    }

    #[test]
    fn test_optimize_random() {
        let grid = empty_grid();
        let optimizer = MultiObjectiveOptimizer::new(&grid);

        let route = simple_route();
        let mut rng = StdRng::seed_from_u64(42);

        let pareto = optimizer.optimize_random(&route, 5, &mut rng);

        assert!(!pareto.is_empty());
    }

    #[test]
    fn test_filter_pareto() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);

        let routes = vec![
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(5.0).with_bends(3.0)),
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(4.0).with_bends(4.0)),
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(6.0).with_bends(5.0)),
        ];

        let pareto = filter_pareto(routes);

        // Third route is dominated
        assert_eq!(pareto.len(), 2);
    }

    #[test]
    fn test_filter_pareto_empty() {
        let routes: Vec<Route> = vec![];
        let pareto = filter_pareto(routes);
        assert!(pareto.is_empty());
    }

    #[test]
    fn test_select_best() {
        let path = VoxelPath::new(vec![VoxelCoord::new(0, 0, 0)]);

        let routes = vec![
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(5.0).with_bends(3.0)),
            Route::new(Path::Voxel(path.clone()))
                .with_cost(RouteCost::new().with_length(4.0).with_bends(4.0)),
        ];

        // Prioritize length
        let weights = CostWeights::default().with_length(1.0).with_bends(0.0);
        let best = select_best(&routes, &weights);

        assert!(best.is_some());
        let best_route = best.unwrap();
        assert!(best_route.cost().length() <= 5.0);
    }

    #[test]
    fn test_select_best_empty() {
        let routes: Vec<Route> = vec![];
        let weights = CostWeights::default();
        let best = select_best(&routes, &weights);

        assert!(best.is_none());
    }

    #[test]
    fn test_generate_weight_configs() {
        let grid = empty_grid();
        let optimizer = MultiObjectiveOptimizer::new(&grid).with_samples(5);

        let configs = optimizer.generate_weight_configs();

        // Should have samples + extremes
        assert!(configs.len() >= 5);
    }
}
