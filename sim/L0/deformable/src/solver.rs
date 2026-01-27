//! XPBD (Extended Position-Based Dynamics) solver for deformable bodies.
//!
//! This module implements the core simulation loop for deformable bodies using
//! the XPBD algorithm, which provides stable, physically-accurate simulation.
//!
//! # Algorithm Overview
//!
//! ```text
//! For each time step:
//!   1. Reset Lagrange multipliers
//!   2. Apply external forces: v += (f/m) * dt
//!   3. Predict positions: x* = x + v * dt
//!   4. For each solver iteration:
//!      a. Solve all constraints
//!   5. Update velocities: v = (x - x_prev) / dt
//!   6. Apply damping
//! ```
//!
//! # Configuration
//!
//! The solver can be configured with:
//!
//! - Number of iterations (more = more accurate, slower)
//! - Damping (velocity-based energy dissipation)
//! - Substeps (divide time step for stability)

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::DeformableBody;
use crate::constraints::Constraint;

/// Configuration for the XPBD solver.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SolverConfig {
    /// Number of constraint solving iterations per substep.
    /// More iterations = more accurate, but slower.
    /// Typical range: 4-20.
    pub num_iterations: u32,

    /// Number of substeps per time step.
    /// More substeps = more stable for stiff materials.
    /// Typical range: 1-4.
    pub num_substeps: u32,

    /// Global damping coefficient for velocity.
    /// 0 = no damping, 1 = full damping.
    /// Typical range: 0.01-0.1.
    pub damping: f64,

    /// Whether to use sleeping (skip simulation for stationary bodies).
    pub enable_sleeping: bool,

    /// Velocity threshold for sleeping.
    pub sleep_threshold: f64,

    /// Maximum velocity clamp (prevents explosion).
    /// Set to `f64::INFINITY` to disable.
    pub max_velocity: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            num_iterations: 10,
            num_substeps: 1,
            damping: 0.01,
            enable_sleeping: false,
            sleep_threshold: 0.001,
            max_velocity: 100.0,
        }
    }
}

impl SolverConfig {
    /// Create a config optimized for real-time simulation.
    #[must_use]
    pub const fn realtime() -> Self {
        Self {
            num_iterations: 4,
            num_substeps: 1,
            damping: 0.02,
            enable_sleeping: true,
            sleep_threshold: 0.01,
            max_velocity: 50.0,
        }
    }

    /// Create a config optimized for accuracy.
    #[must_use]
    pub const fn accurate() -> Self {
        Self {
            num_iterations: 20,
            num_substeps: 4,
            damping: 0.001,
            enable_sleeping: false,
            sleep_threshold: 0.0001,
            max_velocity: 200.0,
        }
    }

    /// Create a config for soft, squishy materials.
    #[must_use]
    pub const fn soft() -> Self {
        Self {
            num_iterations: 8,
            num_substeps: 2,
            damping: 0.05,
            enable_sleeping: false,
            sleep_threshold: 0.001,
            max_velocity: 100.0,
        }
    }

    /// Create a config for stiff materials.
    #[must_use]
    pub const fn stiff() -> Self {
        Self {
            num_iterations: 20,
            num_substeps: 4,
            damping: 0.01,
            enable_sleeping: false,
            sleep_threshold: 0.001,
            max_velocity: 100.0,
        }
    }
}

/// XPBD solver for deformable body simulation.
#[derive(Debug, Clone)]
pub struct XpbdSolver {
    /// Solver configuration.
    config: SolverConfig,
    /// Statistics from the last solve.
    stats: SolverStats,
}

/// Statistics from a solver step.
#[derive(Debug, Clone, Copy, Default)]
pub struct SolverStats {
    /// Total number of constraint iterations performed.
    pub total_iterations: u32,
    /// Maximum constraint error after solving.
    pub max_error: f64,
    /// Average constraint error after solving.
    pub avg_error: f64,
    /// Number of constraints solved.
    pub num_constraints: usize,
    /// Total kinetic energy of the system.
    pub kinetic_energy: f64,
}

impl Default for XpbdSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

impl XpbdSolver {
    /// Create a new XPBD solver with the given configuration.
    #[must_use]
    pub fn new(config: SolverConfig) -> Self {
        Self {
            config,
            stats: SolverStats::default(),
        }
    }

    /// Get the solver configuration.
    #[must_use]
    pub const fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Set the solver configuration.
    pub const fn set_config(&mut self, config: SolverConfig) {
        self.config = config;
    }

    /// Get statistics from the last solve.
    #[must_use]
    pub const fn stats(&self) -> &SolverStats {
        &self.stats
    }

    /// Perform one simulation step.
    ///
    /// # Arguments
    ///
    /// * `body` - The deformable body to simulate
    /// * `gravity` - Gravity vector (m/s²)
    /// * `dt` - Time step in seconds
    pub fn step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64) {
        // Guard against zero timestep
        let dt = dt.max(1e-10);
        let substep_dt = dt / f64::from(self.config.num_substeps);

        for _ in 0..self.config.num_substeps {
            self.substep(body, gravity, substep_dt);
        }
    }

    /// Perform one substep of the simulation.
    fn substep(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64) {
        let num_vertices = body.num_vertices();
        if num_vertices == 0 {
            return;
        }

        // Collect data we need
        let inv_masses: Vec<f64> = body.inverse_masses().to_vec();
        let external_forces: Vec<Vector3<f64>> = body.external_forces().to_vec();

        // Store previous positions for velocity update
        let prev_positions: Vec<_> = body.positions().to_vec();

        // 1. Apply external forces and gravity (predict velocities)
        {
            let velocities = body.velocities_mut();
            for i in 0..num_vertices {
                if inv_masses[i] > 0.0 {
                    // Apply gravity
                    velocities[i] += gravity * dt;

                    // Apply external forces: v += (F/m) * dt = F * inv_mass * dt
                    velocities[i] += external_forces[i] * inv_masses[i] * dt;
                }
            }
        }

        // 2. Predict positions: x* = x + v * dt
        {
            // Copy velocities first to avoid borrow issues
            let velocities: Vec<_> = body.velocities().to_vec();
            let positions = body.positions_mut();
            for i in 0..num_vertices {
                if inv_masses[i] > 0.0 {
                    positions[i] += velocities[i] * dt;
                }
            }
        }

        // 3. Solve constraints
        let (max_error, avg_error) =
            self.solve_constraints(body, dt, self.config.num_iterations, &inv_masses);

        // 4. Update velocities from position changes
        {
            // Copy positions first to avoid borrow issues
            let positions: Vec<_> = body.positions().to_vec();
            let velocities = body.velocities_mut();

            for i in 0..num_vertices {
                if inv_masses[i] > 0.0 {
                    velocities[i] = (positions[i] - prev_positions[i]) / dt;

                    // Apply damping
                    velocities[i] *= 1.0 - self.config.damping;

                    // Clamp velocity
                    let speed = velocities[i].norm();
                    if speed > self.config.max_velocity {
                        velocities[i] *= self.config.max_velocity / speed;
                    }
                }
            }
        }

        // Clear external forces for next step
        body.clear_forces();

        // Update statistics
        let kinetic_energy = self.compute_kinetic_energy(body);
        self.stats = SolverStats {
            total_iterations: self.config.num_iterations,
            max_error,
            avg_error,
            num_constraints: body.constraints().len(),
            kinetic_energy,
        };
    }

    /// Solve all constraints for the given number of iterations.
    ///
    /// Returns (`max_error`, `avg_error`).
    fn solve_constraints(
        &mut self,
        body: &mut dyn DeformableBody,
        dt: f64,
        num_iterations: u32,
        inv_masses: &[f64],
    ) -> (f64, f64) {
        // Clone constraints so we can iterate while modifying positions
        let constraints: Vec<_> = body.constraints().to_vec();

        let mut max_error: f64 = 0.0;
        let mut total_error: f64 = 0.0;
        let mut error_count: usize = 0;

        for _ in 0..num_iterations {
            max_error = 0.0;
            total_error = 0.0;
            error_count = 0;

            let positions = body.positions_mut();

            for constraint in &constraints {
                let error = match constraint {
                    Constraint::Distance(c) => {
                        let mut c = *c;
                        c.solve(positions, inv_masses, dt)
                    }
                    Constraint::Bending(c) => {
                        let mut c = c.clone();
                        c.solve(positions, inv_masses, dt)
                    }
                    Constraint::Volume(c) => {
                        let mut c = *c;
                        c.solve(positions, inv_masses, dt)
                    }
                    Constraint::FlexEdge(c) => {
                        let mut c = c.clone();
                        c.solve(positions, inv_masses, dt)
                    }
                };

                max_error = max_error.max(error);
                total_error += error;
                error_count += 1;
            }
        }

        let avg_error = if error_count > 0 {
            total_error / error_count as f64
        } else {
            0.0
        };

        (max_error, avg_error)
    }

    /// Compute the total kinetic energy of the body.
    fn compute_kinetic_energy(&self, body: &dyn DeformableBody) -> f64 {
        let velocities = body.velocities();
        let inv_masses = body.inverse_masses();

        velocities
            .iter()
            .zip(inv_masses.iter())
            .map(|(v, &inv_m)| {
                if inv_m > 0.0 {
                    0.5 * v.norm_squared() / inv_m
                } else {
                    0.0
                }
            })
            .sum()
    }
}

/// A simple deformable body implementation for testing.
///
/// This provides a basic implementation of the `DeformableBody` trait
/// that can be used for testing the solver.
#[derive(Debug)]
pub struct SimpleDeformable {
    id: crate::types::DeformableId,
    name: String,
    positions: Vec<nalgebra::Point3<f64>>,
    velocities: Vec<Vector3<f64>>,
    inv_masses: Vec<f64>,
    vertex_flags: Vec<crate::types::VertexFlags>,
    constraints: Vec<Constraint>,
    material: crate::material::Material,
    external_forces: Vec<Vector3<f64>>,
}

impl SimpleDeformable {
    /// Create a new simple deformable body.
    #[must_use]
    pub fn new(
        name: &str,
        positions: Vec<nalgebra::Point3<f64>>,
        masses: Vec<f64>,
        constraints: Vec<Constraint>,
    ) -> Self {
        let n = positions.len();
        let inv_masses: Vec<f64> = masses
            .iter()
            .map(|&m| if m > 0.0 { 1.0 / m } else { 0.0 })
            .collect();

        Self {
            id: crate::types::next_deformable_id(),
            name: name.to_string(),
            positions,
            velocities: vec![Vector3::zeros(); n],
            inv_masses,
            vertex_flags: vec![crate::types::VertexFlags::empty(); n],
            constraints,
            material: crate::material::Material::default(),
            external_forces: vec![Vector3::zeros(); n],
        }
    }
}

impl DeformableBody for SimpleDeformable {
    fn id(&self) -> crate::types::DeformableId {
        self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn num_vertices(&self) -> usize {
        self.positions.len()
    }

    fn positions(&self) -> &[nalgebra::Point3<f64>] {
        &self.positions
    }

    fn positions_mut(&mut self) -> &mut [nalgebra::Point3<f64>] {
        &mut self.positions
    }

    fn velocities(&self) -> &[Vector3<f64>] {
        &self.velocities
    }

    fn velocities_mut(&mut self) -> &mut [Vector3<f64>] {
        &mut self.velocities
    }

    fn inverse_masses(&self) -> &[f64] {
        &self.inv_masses
    }

    fn vertex_flags(&self) -> &[crate::types::VertexFlags] {
        &self.vertex_flags
    }

    fn vertex_flags_mut(&mut self) -> &mut [crate::types::VertexFlags] {
        &mut self.vertex_flags
    }

    fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    fn material(&self) -> &crate::material::Material {
        &self.material
    }

    fn pin_vertex(&mut self, index: usize) {
        if index < self.inv_masses.len() {
            self.inv_masses[index] = 0.0;
            self.vertex_flags[index].insert(crate::types::VertexFlags::PINNED);
        }
    }

    fn unpin_vertex(&mut self, index: usize) {
        if index < self.inv_masses.len() {
            // Can't recover original mass, set to 1.0
            self.inv_masses[index] = 1.0;
            self.vertex_flags[index].remove(crate::types::VertexFlags::PINNED);
        }
    }

    fn is_pinned(&self, index: usize) -> bool {
        if index < self.vertex_flags.len() {
            self.vertex_flags[index].contains(crate::types::VertexFlags::PINNED)
        } else {
            false
        }
    }

    fn apply_force(&mut self, index: usize, force: Vector3<f64>) {
        if index < self.external_forces.len() {
            self.external_forces[index] += force;
        }
    }

    fn external_forces(&self) -> &[Vector3<f64>] {
        &self.external_forces
    }

    fn clear_forces(&mut self) {
        for f in &mut self.external_forces {
            *f = Vector3::zeros();
        }
    }

    fn total_mass(&self) -> f64 {
        self.inv_masses
            .iter()
            .map(|&inv_m| if inv_m > 0.0 { 1.0 / inv_m } else { 0.0 })
            .sum()
    }

    fn center_of_mass(&self) -> nalgebra::Point3<f64> {
        let total_mass = self.total_mass();
        if total_mass <= 0.0 {
            return nalgebra::Point3::origin();
        }

        let weighted_sum: Vector3<f64> = self
            .positions
            .iter()
            .zip(self.inv_masses.iter())
            .map(|(p, &inv_m)| {
                let m = if inv_m > 0.0 { 1.0 / inv_m } else { 0.0 };
                p.coords * m
            })
            .sum();

        nalgebra::Point3::from(weighted_sum / total_mass)
    }

    fn bounding_box(&self) -> (nalgebra::Point3<f64>, nalgebra::Point3<f64>) {
        if self.positions.is_empty() {
            return (nalgebra::Point3::origin(), nalgebra::Point3::origin());
        }

        let mut min = self.positions[0];
        let mut max = self.positions[0];

        for p in &self.positions {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        (min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::DistanceConstraint;
    use nalgebra::Point3;

    #[test]
    fn test_solver_config_default() {
        let config = SolverConfig::default();
        assert!(config.num_iterations > 0);
        assert!(config.damping >= 0.0 && config.damping <= 1.0);
    }

    #[test]
    fn test_solver_config_presets() {
        let realtime = SolverConfig::realtime();
        let accurate = SolverConfig::accurate();

        assert!(realtime.num_iterations < accurate.num_iterations);
        assert!(realtime.num_substeps <= accurate.num_substeps);
    }

    #[test]
    fn test_simple_deformable() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let masses = vec![1.0, 1.0];
        let constraints = vec![Constraint::Distance(DistanceConstraint::new(
            0, 1, 1.0, 0.0,
        ))];

        let body = SimpleDeformable::new("test", positions, masses, constraints);

        assert_eq!(body.num_vertices(), 2);
        assert_eq!(body.constraints().len(), 1);
    }

    #[test]
    fn test_solver_gravity() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0)];
        let masses = vec![1.0];
        let constraints = vec![];

        let mut body = SimpleDeformable::new("test", positions, masses, constraints);
        let mut solver = XpbdSolver::default();

        let gravity = Vector3::new(0.0, 0.0, -9.81);
        solver.step(&mut body, gravity, 1.0 / 60.0);

        // Body should have moved down
        assert!(body.positions()[0].z < 0.0);
    }

    #[test]
    fn test_solver_pinned_vertex() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let masses = vec![1.0, 1.0];
        let constraints = vec![Constraint::Distance(DistanceConstraint::new(
            0, 1, 1.0, 0.0,
        ))];

        let mut body = SimpleDeformable::new("test", positions, masses, constraints);
        body.pin_vertex(0);

        let mut solver = XpbdSolver::default();
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        let initial_pos = body.positions()[0];

        for _ in 0..10 {
            solver.step(&mut body, gravity, 1.0 / 60.0);
        }

        // Pinned vertex should not have moved
        assert!((body.positions()[0].coords - initial_pos.coords).norm() < 1e-10);
    }

    #[test]
    fn test_solver_distance_constraint() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Stretched beyond rest length
        ];
        let masses = vec![1.0, 1.0];
        let constraints = vec![Constraint::Distance(DistanceConstraint::new(
            0, 1, 1.0, 0.0,
        ))];

        let mut body = SimpleDeformable::new("test", positions, masses, constraints);
        let mut solver = XpbdSolver::new(SolverConfig {
            num_iterations: 20,
            damping: 0.0,
            ..SolverConfig::default()
        });

        let gravity = Vector3::zeros();

        for _ in 0..10 {
            solver.step(&mut body, gravity, 1.0 / 60.0);
        }

        // Distance should be closer to rest length
        let distance = (body.positions()[1] - body.positions()[0]).norm();
        assert!(
            (distance - 1.0).abs() < 0.5,
            "Distance {} should be close to 1.0",
            distance
        );
    }

    #[test]
    fn test_solver_stats() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let masses = vec![1.0, 1.0];
        let constraints = vec![Constraint::Distance(DistanceConstraint::new(
            0, 1, 1.0, 0.0,
        ))];

        let mut body = SimpleDeformable::new("test", positions, masses, constraints);
        let mut solver = XpbdSolver::default();

        solver.step(&mut body, Vector3::zeros(), 1.0 / 60.0);

        let stats = solver.stats();
        assert_eq!(stats.num_constraints, 1);
        assert_eq!(stats.total_iterations, solver.config.num_iterations);
    }
}
