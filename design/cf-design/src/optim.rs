//! Finite-difference gradient descent for design parameter optimization.
//!
//! [`minimize_fd`] optimizes scalar design variables in a [`ParamStore`]
//! by evaluating a user-provided objective closure with centered finite
//! differences. The optimizer is simulator-agnostic — the closure captures
//! whatever pipeline is needed (meshing, MJCF generation, simulation).
//!
//! # Gradient chain
//!
//! For design-through-physics optimization, the end-to-end gradient is:
//!
//! ```text
//! ∂J/∂θ where θ → SDF(θ) → mesh → Model → simulate → x_T → J(x_T)
//! ```
//!
//! The middle of this chain (mesh → Model) is not analytically
//! differentiable, so we use finite differences over the full pipeline.
//! Session 25's analytic `∂f/∂θ` enables future hybrid approaches
//! (analytic mass/inertia gradients + FD for the sim boundary).

use crate::param::ParamStore;

/// Configuration for finite-difference gradient descent.
#[derive(Debug, Clone)]
pub struct OptimConfig {
    /// Maximum number of optimization iterations.
    pub max_iters: usize,
    /// Learning rate (step size for parameter updates).
    pub learning_rate: f64,
    /// Perturbation magnitude for centered finite differences.
    pub fd_eps: f64,
    /// Convergence tolerance: stop when gradient norm < `grad_tol`.
    pub grad_tol: f64,
}

impl Default for OptimConfig {
    fn default() -> Self {
        Self {
            max_iters: 100,
            learning_rate: 0.01,
            fd_eps: 1e-4,
            grad_tol: 1e-6,
        }
    }
}

/// Result of an optimization run.
#[derive(Debug, Clone)]
pub struct OptimResult {
    /// Final objective value (evaluated at the returned parameters).
    pub objective: f64,
    /// Number of gradient iterations completed.
    pub iterations: usize,
    /// Final parameter values: `(name, value)`.
    pub params: Vec<(String, f64)>,
    /// Objective value at each iteration's starting parameters.
    pub history: Vec<f64>,
    /// `true` if gradient norm fell below [`OptimConfig::grad_tol`].
    pub converged: bool,
}

/// Minimize `objective()` over the parameters in `store` using centered
/// finite-difference gradient descent.
///
/// The closure must capture all state needed to evaluate the design
/// (mechanism, simulation setup, etc.) and return a scalar cost to
/// minimize. The optimizer perturbs parameters via [`ParamStore::set`];
/// the closure reads updated values through the shared store.
///
/// For **maximization** problems, negate the objective in the closure.
///
/// # Algorithm
///
/// Each iteration:
/// 1. Evaluate `objective()` at current parameters.
/// 2. For each parameter θ\_i, compute centered FD gradient:
///    `∂J/∂θ_i ≈ [J(θ_i + ε) − J(θ_i − ε)] / (2ε)`.
/// 3. Update: `θ_i ← θ_i − lr × ∂J/∂θ_i`.
/// 4. Stop if `‖∇J‖ < grad_tol`.
///
/// # Example
///
/// ```
/// use cf_design::{ParamStore, Solid, optim::{minimize_fd, OptimConfig}};
/// use nalgebra::Point3;
///
/// let store = ParamStore::new();
/// let r = store.add("radius", 5.0);
/// let solid = Solid::sphere_p(r);
///
/// // Minimize field value at a fixed point (drives radius outward).
/// let result = minimize_fd(&store, || {
///     solid.evaluate(&Point3::new(6.0, 0.0, 0.0))
/// }, &OptimConfig {
///     max_iters: 10,
///     learning_rate: 0.5,
///     fd_eps: 1e-4,
///     grad_tol: 1e-8,
/// });
///
/// assert!(result.objective < result.history[0]);
/// ```
pub fn minimize_fd(
    store: &ParamStore,
    mut objective: impl FnMut() -> f64,
    config: &OptimConfig,
) -> OptimResult {
    let names = store.names();
    let n = names.len();

    // Edge case: no parameters to optimize.
    if n == 0 {
        let obj = objective();
        return OptimResult {
            objective: obj,
            iterations: 0,
            params: vec![],
            history: vec![obj],
            converged: true,
        };
    }

    let mut history = Vec::with_capacity(config.max_iters + 1);
    let mut converged = false;
    let mut iterations = 0;
    let mut grad = vec![0.0; n];

    for _ in 0..config.max_iters {
        let j0 = objective();
        history.push(j0);

        // Centered finite-difference gradient.
        for (i, name) in names.iter().enumerate() {
            let orig = store.get(name).unwrap_or(0.0);

            store.set(name, orig + config.fd_eps);
            let j_plus = objective();

            store.set(name, orig - config.fd_eps);
            let j_minus = objective();

            store.set(name, orig); // restore
            grad[i] = (j_plus - j_minus) / (2.0 * config.fd_eps);
        }

        let grad_norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();
        iterations += 1;

        if grad_norm < config.grad_tol {
            converged = true;
            break;
        }

        // Gradient descent step.
        for (i, name) in names.iter().enumerate() {
            let orig = store.get(name).unwrap_or(0.0);
            store.set(name, config.learning_rate.mul_add(-grad[i], orig));
        }
    }

    // Evaluate at final parameters for a consistent result.
    let final_obj = objective();

    let params = names
        .iter()
        .map(|name| (name.clone(), store.get(name).unwrap_or(0.0)))
        .collect();

    OptimResult {
        objective: final_obj,
        iterations,
        params,
        history,
        converged,
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::Solid;

    #[test]
    fn minimize_sphere_radius() {
        let store = ParamStore::new();
        let r = store.add("radius", 3.0);
        let solid = Solid::sphere_p(r);

        // f(6,0,0) = |6| - radius = 6 - radius.
        // Minimizing → radius increases (toward 6).
        let result = minimize_fd(
            &store,
            || solid.evaluate(&Point3::new(6.0, 0.0, 0.0)),
            &OptimConfig {
                max_iters: 20,
                learning_rate: 0.5,
                fd_eps: 1e-4,
                grad_tol: 1e-6,
            },
        );

        assert!(
            result.objective < result.history[0],
            "objective should decrease: {:.4} vs {:.4}",
            result.objective,
            result.history[0],
        );
        // Radius should have increased from 3.0.
        let final_r = store.get("radius").unwrap_or(0.0);
        assert!(
            final_r > 3.5,
            "radius should increase toward 6, got {final_r:.2}"
        );
    }

    #[test]
    fn no_params_returns_immediately() {
        let store = ParamStore::new();
        let result = minimize_fd(&store, || 42.0, &OptimConfig::default());
        assert_eq!(result.iterations, 0);
        assert!((result.objective - 42.0).abs() < f64::EPSILON);
        assert!(result.converged);
    }

    #[test]
    fn converges_on_flat_objective() {
        let store = ParamStore::new();
        let _r = store.add("x", 1.0);

        // Constant objective → gradient = 0 → converges immediately.
        let result = minimize_fd(
            &store,
            || 5.0,
            &OptimConfig {
                max_iters: 10,
                learning_rate: 0.1,
                fd_eps: 1e-4,
                grad_tol: 1e-6,
            },
        );

        assert!(result.converged, "should converge on constant objective");
        assert_eq!(result.iterations, 1);
    }
}
