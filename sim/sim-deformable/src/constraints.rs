//! Constraints for deformable body simulation.
//!
//! This module provides XPBD (Extended Position-Based Dynamics) constraints
//! for simulating deformable bodies:
//!
//! - [`DistanceConstraint`] - Maintains distance between two particles
//! - [`BendingConstraint`] - Maintains angle between connected edges
//! - [`VolumeConstraint`] - Preserves volume of tetrahedra
//!
//! # XPBD Constraint Solving
//!
//! Each constraint follows the XPBD formulation:
//!
//! ```text
//! Δλ = (-C - α̃ λ) / (∇C · w · ∇C^T + α̃)
//! Δx = w · ∇C^T · Δλ
//! ```
//!
//! Where:
//! - `C` is the constraint function
//! - `α̃ = α / h²` is the time-scaled compliance
//! - `λ` is the Lagrange multiplier
//! - `w` are inverse masses
//! - `∇C` is the constraint gradient

use nalgebra::Point3;
use smallvec::SmallVec;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Type of constraint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ConstraintType {
    /// Distance constraint between two particles.
    Distance,
    /// Bending constraint between three or four particles.
    Bending,
    /// Volume preservation constraint for tetrahedra.
    Volume,
    /// Collision constraint.
    Collision,
    /// Flex edge constraint for flexible body edges.
    FlexEdge,
}

/// A constraint that can be solved using XPBD.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Constraint {
    /// Distance constraint.
    Distance(DistanceConstraint),
    /// Bending constraint.
    Bending(BendingConstraint),
    /// Volume constraint.
    Volume(VolumeConstraint),
    /// Flex edge constraint.
    FlexEdge(FlexEdgeConstraint),
}

impl Constraint {
    /// Get the type of this constraint.
    #[must_use]
    pub const fn constraint_type(&self) -> ConstraintType {
        match self {
            Self::Distance(_) => ConstraintType::Distance,
            Self::Bending(_) => ConstraintType::Bending,
            Self::Volume(_) => ConstraintType::Volume,
            Self::FlexEdge(_) => ConstraintType::FlexEdge,
        }
    }

    /// Get the vertex indices involved in this constraint.
    #[must_use]
    pub fn vertices(&self) -> SmallVec<[usize; 4]> {
        match self {
            Self::Distance(c) => {
                let mut v = SmallVec::new();
                v.push(c.v0);
                v.push(c.v1);
                v
            }
            Self::Bending(c) => c.vertices.clone(),
            Self::Volume(c) => SmallVec::from_slice(&c.vertices),
            Self::FlexEdge(c) => SmallVec::from_slice(&c.vertices),
        }
    }

    /// Get the compliance of this constraint.
    #[must_use]
    pub const fn compliance(&self) -> f64 {
        match self {
            Self::Distance(c) => c.compliance,
            Self::Bending(c) => c.compliance,
            Self::Volume(c) => c.compliance,
            Self::FlexEdge(c) => c.stretch_compliance,
        }
    }

    /// Solve this constraint using XPBD.
    ///
    /// # Arguments
    ///
    /// * `positions` - Mutable slice of vertex positions
    /// * `inv_masses` - Slice of inverse masses
    /// * `dt` - Time step
    ///
    /// # Returns
    ///
    /// The constraint error after solving.
    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        match self {
            Self::Distance(c) => c.solve(positions, inv_masses, dt),
            Self::Bending(c) => c.solve(positions, inv_masses, dt),
            Self::Volume(c) => c.solve(positions, inv_masses, dt),
            Self::FlexEdge(c) => c.solve(positions, inv_masses, dt),
        }
    }
}

/// Distance constraint between two particles.
///
/// Maintains the distance between two vertices at the rest length.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DistanceConstraint {
    /// Index of the first vertex.
    pub v0: usize,
    /// Index of the second vertex.
    pub v1: usize,
    /// Rest length (target distance).
    pub rest_length: f64,
    /// Compliance (inverse stiffness).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier.
    lambda: f64,
}

impl DistanceConstraint {
    /// Create a new distance constraint.
    ///
    /// # Arguments
    ///
    /// * `v0` - Index of first vertex
    /// * `v1` - Index of second vertex
    /// * `rest_length` - Target distance
    /// * `compliance` - Inverse stiffness (0 = rigid)
    #[must_use]
    pub const fn new(v0: usize, v1: usize, rest_length: f64, compliance: f64) -> Self {
        Self {
            v0,
            v1,
            rest_length,
            compliance,
            lambda: 0.0,
        }
    }

    /// Create a distance constraint from current positions.
    #[must_use]
    pub fn from_positions(
        v0: usize,
        v1: usize,
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let rest_length = (positions[v1] - positions[v0]).norm();
        Self::new(v0, v1, rest_length, compliance)
    }

    /// Reset the Lagrange multiplier (call at start of time step).
    pub const fn reset(&mut self) {
        self.lambda = 0.0;
    }

    /// Compute the constraint value C = |x1 - x0| - L.
    #[must_use]
    pub fn evaluate(&self, positions: &[Point3<f64>]) -> f64 {
        let diff = positions[self.v1] - positions[self.v0];
        diff.norm() - self.rest_length
    }

    /// Solve this constraint using XPBD.
    ///
    /// Returns the constraint error magnitude.
    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let w0 = inv_masses[self.v0];
        let w1 = inv_masses[self.v1];
        let w_sum = w0 + w1;

        // Both particles are pinned
        if w_sum < 1e-10 {
            return 0.0;
        }

        let diff = positions[self.v1] - positions[self.v0];
        let distance = diff.norm();

        // Avoid division by zero for degenerate case
        if distance < 1e-10 {
            return 0.0;
        }

        // Constraint value: C = |x1 - x0| - rest_length
        let c = distance - self.rest_length;

        // Gradient: ∇C = n (normalized direction)
        let n = diff / distance;

        // XPBD: α̃ = α / h²
        let alpha_tilde = self.compliance / (dt * dt);

        // Δλ = (-C - α̃ λ) / (w_sum + α̃)
        let delta_lambda = alpha_tilde.mul_add(-self.lambda, -c) / (w_sum + alpha_tilde);

        // Update positions
        positions[self.v0] -= n * (w0 * delta_lambda);
        positions[self.v1] += n * (w1 * delta_lambda);

        // Update Lagrange multiplier
        self.lambda += delta_lambda;

        c.abs()
    }
}

/// Bending constraint for maintaining angles between edges.
///
/// For 1D (chains): Uses 3 consecutive particles.
/// For 2D (cloth): Uses 4 particles (two triangles sharing an edge).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BendingConstraint {
    /// Vertex indices.
    /// For chains: [v0, v1, v2] where v1 is the hinge.
    /// For cloth: [v0, v1, v2, v3] where (v0, v1) is shared edge.
    pub vertices: SmallVec<[usize; 4]>,
    /// Rest angle (radians) or rest dihedral angle.
    pub rest_angle: f64,
    /// Compliance (inverse stiffness).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier.
    lambda: f64,
}

impl BendingConstraint {
    /// Create a bending constraint for a 1D chain (3 particles).
    ///
    /// The constraint maintains the angle at v1 between edges (v0-v1) and (v1-v2).
    #[must_use]
    pub fn chain(v0: usize, v1: usize, v2: usize, rest_angle: f64, compliance: f64) -> Self {
        Self {
            vertices: SmallVec::from_buf([v0, v1, v2, 0])[..3].into(),
            rest_angle,
            compliance,
            lambda: 0.0,
        }
    }

    /// Create a bending constraint for a 1D chain from current positions.
    #[must_use]
    pub fn chain_from_positions(
        v0: usize,
        v1: usize,
        v2: usize,
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let e1 = positions[v0] - positions[v1];
        let e2 = positions[v2] - positions[v1];

        let rest_angle = e1.angle(&e2);

        Self::chain(v0, v1, v2, rest_angle, compliance)
    }

    /// Create a dihedral bending constraint for cloth (4 particles).
    ///
    /// Vertices v0 and v1 are on the shared edge.
    /// Vertices v2 and v3 are the opposite vertices of the two triangles.
    #[must_use]
    pub fn dihedral(
        v0: usize,
        v1: usize,
        v2: usize,
        v3: usize,
        rest_angle: f64,
        compliance: f64,
    ) -> Self {
        Self {
            vertices: SmallVec::from_buf([v0, v1, v2, v3]),
            rest_angle,
            compliance,
            lambda: 0.0,
        }
    }

    /// Create a dihedral bending constraint from current positions.
    #[must_use]
    pub fn dihedral_from_positions(
        v0: usize,
        v1: usize,
        v2: usize,
        v3: usize,
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let rest_angle = Self::compute_dihedral_angle(
            &positions[v0],
            &positions[v1],
            &positions[v2],
            &positions[v3],
        );
        Self::dihedral(v0, v1, v2, v3, rest_angle, compliance)
    }

    /// Compute the dihedral angle between two triangles sharing edge (p0, p1).
    fn compute_dihedral_angle(
        p0: &Point3<f64>,
        p1: &Point3<f64>,
        p2: &Point3<f64>,
        p3: &Point3<f64>,
    ) -> f64 {
        // Edge vector
        let e = p1 - p0;
        let e_len = e.norm();
        if e_len < 1e-10 {
            return 0.0;
        }
        let e_norm = e / e_len;

        // Vectors from edge to opposite vertices
        let v2 = p2 - p0;
        let v3 = p3 - p0;

        // Normals to triangles
        let n1 = e.cross(&v2);
        let n2 = v3.cross(&e);

        let n1_len = n1.norm();
        let n2_len = n2.norm();

        if n1_len < 1e-10 || n2_len < 1e-10 {
            return 0.0;
        }

        let n1 = n1 / n1_len;
        let n2 = n2 / n2_len;

        // Dihedral angle from normals
        let cos_angle = n1.dot(&n2).clamp(-1.0, 1.0);
        let sin_angle = n1.cross(&n2).dot(&e_norm);

        sin_angle.atan2(cos_angle)
    }

    /// Reset the Lagrange multiplier.
    pub const fn reset(&mut self) {
        self.lambda = 0.0;
    }

    /// Solve this constraint using XPBD.
    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        if self.vertices.len() == 3 {
            self.solve_chain(positions, inv_masses, dt)
        } else {
            self.solve_dihedral(positions, inv_masses, dt)
        }
    }

    /// Solve chain bending constraint (3 particles).
    ///
    /// Uses a distance-based bending constraint which is more stable than
    /// the angle-based formulation, especially near straight configurations.
    /// The constraint maintains the distance between v0 and v2.
    fn solve_chain(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];

        let w0 = inv_masses[v0];
        // Note: Middle vertex (v1) is not directly involved in distance-based bending
        let w2 = inv_masses[v2];

        let p0 = positions[v0];
        let p1 = positions[v1];
        let p2 = positions[v2];

        // For chain bending, we use a distance constraint between v0 and v2
        // The rest distance is computed from the rest angle and the edge lengths
        let e1 = p0 - p1;
        let e2 = p2 - p1;
        let l1 = e1.norm();
        let l2 = e2.norm();

        if l1 < 1e-10 || l2 < 1e-10 {
            return 0.0;
        }

        // Rest distance from law of cosines: c² = a² + b² - 2ab*cos(θ)
        let rest_distance = (2.0 * l1 * l2)
            .mul_add(-self.rest_angle.cos(), l1.mul_add(l1, l2 * l2))
            .sqrt();

        // Current distance between v0 and v2
        let diff = p2 - p0;
        let current_distance = diff.norm();

        if current_distance < 1e-10 {
            return 0.0;
        }

        // Constraint: C = |p2 - p0| - rest_distance
        let c = current_distance - rest_distance;

        if c.abs() < 1e-10 {
            return 0.0;
        }

        // Gradient: direction from v0 to v2, normalized
        let n = diff / current_distance;

        // For the distance constraint between v0 and v2:
        // grad_v0 = -n, grad_v2 = +n
        // The middle vertex v1 is not directly involved in the constraint
        // but we apply a small correction to maintain the chain structure

        let w_sum = w0 + w2;

        if w_sum < 1e-10 {
            return 0.0;
        }

        // XPBD update
        let alpha_tilde = self.compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda, -c) / (w_sum + alpha_tilde);

        // Apply corrections only to v0 and v2
        positions[v0] -= n * (w0 * delta_lambda);
        positions[v2] += n * (w2 * delta_lambda);

        self.lambda += delta_lambda;

        c.abs()
    }

    /// Solve dihedral bending constraint (4 particles).
    fn solve_dihedral(
        &mut self,
        positions: &mut [Point3<f64>],
        inv_masses: &[f64],
        dt: f64,
    ) -> f64 {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        let v3 = self.vertices[3];

        let w0 = inv_masses[v0];
        let w1 = inv_masses[v1];
        let w2 = inv_masses[v2];
        let w3 = inv_masses[v3];

        let p0 = positions[v0];
        let p1 = positions[v1];
        let p2 = positions[v2];
        let p3 = positions[v3];

        // Current dihedral angle
        let current_angle = Self::compute_dihedral_angle(&p0, &p1, &p2, &p3);

        // Constraint: C = angle - rest_angle
        let c = current_angle - self.rest_angle;

        if c.abs() < 1e-10 {
            return 0.0;
        }

        // Edge and normals
        let e = p1 - p0;
        let e_len = e.norm();
        if e_len < 1e-10 {
            return 0.0;
        }

        let n1 = e.cross(&(p2 - p0));
        let n2 = (p3 - p0).cross(&e);

        let n1_len = n1.norm();
        let n2_len = n2.norm();

        if n1_len < 1e-10 || n2_len < 1e-10 {
            return 0.0;
        }

        let n1_norm = n1 / n1_len;
        let n2_norm = n2 / n2_len;

        // Gradients (simplified)
        let grad0 = (n1_norm + n2_norm) * 0.5;
        let grad1 = -(n1_norm + n2_norm) * 0.5;
        let grad2 = -n1_norm;
        let grad3 = -n2_norm;

        let w_sum = w3.mul_add(
            grad3.norm_squared(),
            w2.mul_add(
                grad2.norm_squared(),
                w0.mul_add(grad0.norm_squared(), w1 * grad1.norm_squared()),
            ),
        );

        if w_sum < 1e-10 {
            return 0.0;
        }

        // XPBD update
        let alpha_tilde = self.compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda, -c) / (w_sum + alpha_tilde);

        positions[v0] += grad0 * (w0 * delta_lambda);
        positions[v1] += grad1 * (w1 * delta_lambda);
        positions[v2] += grad2 * (w2 * delta_lambda);
        positions[v3] += grad3 * (w3 * delta_lambda);

        self.lambda += delta_lambda;

        c.abs()
    }
}

/// Volume preservation constraint for tetrahedra.
///
/// Maintains the volume of a tetrahedron at its rest volume.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VolumeConstraint {
    /// Indices of the four vertices.
    pub vertices: [usize; 4],
    /// Rest volume.
    pub rest_volume: f64,
    /// Compliance (inverse stiffness).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier.
    lambda: f64,
}

impl VolumeConstraint {
    /// Create a new volume constraint.
    #[must_use]
    pub const fn new(vertices: [usize; 4], rest_volume: f64, compliance: f64) -> Self {
        Self {
            vertices,
            rest_volume,
            compliance,
            lambda: 0.0,
        }
    }

    /// Create a volume constraint from current positions.
    #[must_use]
    pub fn from_positions(
        vertices: [usize; 4],
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let rest_volume = Self::compute_volume(&vertices, positions);
        Self::new(vertices, rest_volume, compliance)
    }

    /// Compute the signed volume of a tetrahedron.
    fn compute_volume(vertices: &[usize; 4], positions: &[Point3<f64>]) -> f64 {
        let p0 = positions[vertices[0]];
        let p1 = positions[vertices[1]];
        let p2 = positions[vertices[2]];
        let p3 = positions[vertices[3]];

        let e1 = p1 - p0;
        let e2 = p2 - p0;
        let e3 = p3 - p0;

        e1.cross(&e2).dot(&e3) / 6.0
    }

    /// Reset the Lagrange multiplier.
    pub const fn reset(&mut self) {
        self.lambda = 0.0;
    }

    /// Solve this constraint using XPBD.
    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        let v3 = self.vertices[3];

        let w0 = inv_masses[v0];
        let w1 = inv_masses[v1];
        let w2 = inv_masses[v2];
        let w3 = inv_masses[v3];

        let p0 = positions[v0];
        let p1 = positions[v1];
        let p2 = positions[v2];
        let p3 = positions[v3];

        // Current volume
        let current_volume = Self::compute_volume(&self.vertices, positions);

        // Constraint: C = V - V_rest
        let c = current_volume - self.rest_volume;

        if c.abs() < 1e-10 {
            return 0.0;
        }

        // Gradients: ∂V/∂pᵢ
        // V = (p1-p0) · ((p2-p0) × (p3-p0)) / 6
        let e1 = p1 - p0;
        let e2 = p2 - p0;
        let e3 = p3 - p0;

        let grad1 = e2.cross(&e3) / 6.0;
        let grad2 = e3.cross(&e1) / 6.0;
        let grad3 = e1.cross(&e2) / 6.0;
        let grad0 = -(grad1 + grad2 + grad3);

        // Weighted sum
        let w_sum = w3.mul_add(
            grad3.norm_squared(),
            w2.mul_add(
                grad2.norm_squared(),
                w0.mul_add(grad0.norm_squared(), w1 * grad1.norm_squared()),
            ),
        );

        if w_sum < 1e-10 {
            return 0.0;
        }

        // XPBD update
        let alpha_tilde = self.compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda, -c) / (w_sum + alpha_tilde);

        positions[v0] += grad0 * (w0 * delta_lambda);
        positions[v1] += grad1 * (w1 * delta_lambda);
        positions[v2] += grad2 * (w2 * delta_lambda);
        positions[v3] += grad3 * (w3 * delta_lambda);

        self.lambda += delta_lambda;

        c.abs()
    }
}

// ============================================================================
// Flex Edge Constraint
// ============================================================================

/// Edge constraint type for flex bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FlexEdgeType {
    /// Stretch-only constraint (inextensible).
    Stretch,
    /// Shear constraint (maintains angle).
    Shear,
    /// Combined stretch and shear.
    StretchShear,
    /// Twist constraint (torsion resistance).
    Twist,
}

/// A flex edge constraint for flexible body edges.
///
/// Flex edge constraints model the mechanical behavior of flexible bodies
/// like cables, ropes, tubes, and thin rods. They combine:
///
/// - **Stretch resistance**: Maintains edge length
/// - **Shear resistance**: Maintains angle between consecutive edges
/// - **Twist resistance**: Resists torsional deformation
///
/// # Flex Compatibility
///
/// This constraint is designed to be compatible with MuJoCo-style flex edge
/// constraints, which are used to model flexible bodies composed of
/// connected capsules or spheres.
///
/// # Constraint Formulation
///
/// For stretch: `C_stretch = |p1 - p0| - L_rest`
///
/// For shear (3 particles): `C_shear = (e1 · e2) / (|e1| |e2|) - cos(θ_rest)`
///
/// For twist: `C_twist = angle(frame1, frame2) - θ_rest`
///
/// # Example
///
/// ```ignore
/// use sim_deformable::constraints::{FlexEdgeConstraint, FlexEdgeType};
/// use nalgebra::Point3;
///
/// let positions = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ];
///
/// // Create a stretch-shear constraint for a flexible rod
/// let constraint = FlexEdgeConstraint::stretch_shear_from_positions(
///     [0, 1, 2],
///     &positions,
///     0.001,  // stretch compliance
///     0.01,   // shear compliance
/// );
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FlexEdgeConstraint {
    /// Vertex indices (2 for stretch, 3 for shear/twist).
    pub vertices: [usize; 3],

    /// Number of active vertices (2 or 3).
    num_vertices: usize,

    /// Edge type.
    edge_type: FlexEdgeType,

    /// Rest length for stretch constraint.
    pub rest_length: f64,

    /// Rest angle for shear constraint (radians).
    pub rest_angle: f64,

    /// Rest twist angle for twist constraint (radians).
    pub rest_twist: f64,

    /// Compliance for stretch (inverse stiffness).
    pub stretch_compliance: f64,

    /// Compliance for shear.
    pub shear_compliance: f64,

    /// Compliance for twist.
    pub twist_compliance: f64,

    /// Damping coefficient.
    pub damping: f64,

    /// Accumulated Lagrange multiplier for stretch.
    lambda_stretch: f64,

    /// Accumulated Lagrange multiplier for shear.
    lambda_shear: f64,

    /// Accumulated Lagrange multiplier for twist.
    lambda_twist: f64,
}

impl FlexEdgeConstraint {
    /// Create a stretch-only constraint.
    #[must_use]
    pub const fn stretch(v0: usize, v1: usize, rest_length: f64, compliance: f64) -> Self {
        Self {
            vertices: [v0, v1, 0],
            num_vertices: 2,
            edge_type: FlexEdgeType::Stretch,
            rest_length,
            rest_angle: 0.0,
            rest_twist: 0.0,
            stretch_compliance: compliance,
            shear_compliance: 0.0,
            twist_compliance: 0.0,
            damping: 0.0,
            lambda_stretch: 0.0,
            lambda_shear: 0.0,
            lambda_twist: 0.0,
        }
    }

    /// Create a stretch constraint from current positions.
    #[must_use]
    pub fn stretch_from_positions(
        v0: usize,
        v1: usize,
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let rest_length = (positions[v1] - positions[v0]).norm();
        Self::stretch(v0, v1, rest_length, compliance)
    }

    /// Create a shear-only constraint (3 particles).
    #[must_use]
    pub const fn shear(vertices: [usize; 3], rest_angle: f64, compliance: f64) -> Self {
        Self {
            vertices,
            num_vertices: 3,
            edge_type: FlexEdgeType::Shear,
            rest_length: 0.0,
            rest_angle,
            rest_twist: 0.0,
            stretch_compliance: 0.0,
            shear_compliance: compliance,
            twist_compliance: 0.0,
            damping: 0.0,
            lambda_stretch: 0.0,
            lambda_shear: 0.0,
            lambda_twist: 0.0,
        }
    }

    /// Create a shear constraint from current positions.
    #[must_use]
    pub fn shear_from_positions(
        vertices: [usize; 3],
        positions: &[Point3<f64>],
        compliance: f64,
    ) -> Self {
        let rest_angle = Self::compute_angle(
            &positions[vertices[0]],
            &positions[vertices[1]],
            &positions[vertices[2]],
        );
        Self::shear(vertices, rest_angle, compliance)
    }

    /// Create a combined stretch-shear constraint.
    #[must_use]
    pub const fn stretch_shear(
        vertices: [usize; 3],
        rest_length: f64,
        rest_angle: f64,
        stretch_compliance: f64,
        shear_compliance: f64,
    ) -> Self {
        Self {
            vertices,
            num_vertices: 3,
            edge_type: FlexEdgeType::StretchShear,
            rest_length,
            rest_angle,
            rest_twist: 0.0,
            stretch_compliance,
            shear_compliance,
            twist_compliance: 0.0,
            damping: 0.0,
            lambda_stretch: 0.0,
            lambda_shear: 0.0,
            lambda_twist: 0.0,
        }
    }

    /// Create a stretch-shear constraint from current positions.
    #[must_use]
    pub fn stretch_shear_from_positions(
        vertices: [usize; 3],
        positions: &[Point3<f64>],
        stretch_compliance: f64,
        shear_compliance: f64,
    ) -> Self {
        let rest_length = (positions[vertices[1]] - positions[vertices[0]]).norm();
        let rest_angle = Self::compute_angle(
            &positions[vertices[0]],
            &positions[vertices[1]],
            &positions[vertices[2]],
        );
        Self::stretch_shear(
            vertices,
            rest_length,
            rest_angle,
            stretch_compliance,
            shear_compliance,
        )
    }

    /// Create a twist constraint from current positions.
    #[must_use]
    pub const fn twist(vertices: [usize; 3], rest_twist: f64, compliance: f64) -> Self {
        Self {
            vertices,
            num_vertices: 3,
            edge_type: FlexEdgeType::Twist,
            rest_length: 0.0,
            rest_angle: 0.0,
            rest_twist,
            stretch_compliance: 0.0,
            shear_compliance: 0.0,
            twist_compliance: compliance,
            damping: 0.0,
            lambda_stretch: 0.0,
            lambda_shear: 0.0,
            lambda_twist: 0.0,
        }
    }

    /// Set damping coefficient.
    #[must_use]
    pub const fn with_damping(mut self, damping: f64) -> Self {
        self.damping = if damping > 0.0 { damping } else { 0.0 };
        self
    }

    /// Get the edge type.
    #[must_use]
    pub const fn edge_type(&self) -> FlexEdgeType {
        self.edge_type
    }

    /// Get the number of vertices.
    #[must_use]
    pub const fn num_vertices(&self) -> usize {
        self.num_vertices
    }

    /// Compute angle at p1 between edges (p0-p1) and (p1-p2).
    fn compute_angle(p0: &Point3<f64>, p1: &Point3<f64>, p2: &Point3<f64>) -> f64 {
        let e1 = p0 - p1;
        let e2 = p2 - p1;

        let len1 = e1.norm();
        let len2 = e2.norm();

        if len1 < 1e-10 || len2 < 1e-10 {
            return std::f64::consts::PI;
        }

        let cos_angle = (e1.dot(&e2) / (len1 * len2)).clamp(-1.0, 1.0);
        cos_angle.acos()
    }

    /// Reset all Lagrange multipliers (call at start of time step).
    pub const fn reset(&mut self) {
        self.lambda_stretch = 0.0;
        self.lambda_shear = 0.0;
        self.lambda_twist = 0.0;
    }

    /// Solve this constraint using XPBD.
    ///
    /// Returns the total constraint error magnitude.
    pub fn solve(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let mut total_error = 0.0;

        match self.edge_type {
            FlexEdgeType::Stretch => {
                total_error += self.solve_stretch(positions, inv_masses, dt);
            }
            FlexEdgeType::Shear => {
                total_error += self.solve_shear(positions, inv_masses, dt);
            }
            FlexEdgeType::StretchShear => {
                total_error += self.solve_stretch(positions, inv_masses, dt);
                total_error += self.solve_shear(positions, inv_masses, dt);
            }
            FlexEdgeType::Twist => {
                total_error += self.solve_twist(positions, inv_masses, dt);
            }
        }

        total_error
    }

    /// Solve stretch constraint.
    fn solve_stretch(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];

        let w0 = inv_masses[v0];
        let w1 = inv_masses[v1];
        let w_sum = w0 + w1;

        if w_sum < 1e-10 {
            return 0.0;
        }

        let diff = positions[v1] - positions[v0];
        let distance = diff.norm();

        if distance < 1e-10 {
            return 0.0;
        }

        // Constraint: C = distance - rest_length
        let c = distance - self.rest_length;

        // Gradient: normalized direction
        let n = diff / distance;

        // XPBD update
        let alpha_tilde = self.stretch_compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda_stretch, -c) / (w_sum + alpha_tilde);

        // Apply damping
        let damping_factor = 1.0 / self.damping.mul_add(dt, 1.0);

        positions[v0] -= n * (w0 * delta_lambda * damping_factor);
        positions[v1] += n * (w1 * delta_lambda * damping_factor);

        self.lambda_stretch += delta_lambda;

        c.abs()
    }

    /// Solve shear constraint.
    fn solve_shear(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        if self.num_vertices < 3 {
            return 0.0;
        }

        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];

        let w0 = inv_masses[v0];
        let w2 = inv_masses[v2];

        let p0 = positions[v0];
        let p1 = positions[v1];
        let p2 = positions[v2];

        // Edges from hinge vertex
        let e1 = p0 - p1;
        let e2 = p2 - p1;

        let len1 = e1.norm();
        let len2 = e2.norm();

        if len1 < 1e-10 || len2 < 1e-10 {
            return 0.0;
        }

        // Current angle
        let cos_angle = (e1.dot(&e2) / (len1 * len2)).clamp(-1.0, 1.0);
        let current_angle = cos_angle.acos();

        // Constraint: C = angle - rest_angle
        let c = current_angle - self.rest_angle;

        if c.abs() < 1e-10 {
            return 0.0;
        }

        // Use distance-based formulation (more stable)
        // Rest distance from law of cosines
        let rest_distance = (2.0 * len1 * len2)
            .mul_add(-self.rest_angle.cos(), len1.mul_add(len1, len2 * len2))
            .sqrt();

        let current_distance = (p2 - p0).norm();

        if current_distance < 1e-10 {
            return 0.0;
        }

        let c_dist = current_distance - rest_distance;
        let n = (p2 - p0) / current_distance;

        let w_sum = w0 + w2;

        if w_sum < 1e-10 {
            return 0.0;
        }

        // XPBD update
        let alpha_tilde = self.shear_compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda_shear, -c_dist) / (w_sum + alpha_tilde);

        // Apply damping
        let damping_factor = 1.0 / self.damping.mul_add(dt, 1.0);

        positions[v0] -= n * (w0 * delta_lambda * damping_factor);
        positions[v2] += n * (w2 * delta_lambda * damping_factor);

        self.lambda_shear += delta_lambda;

        c.abs()
    }

    /// Solve twist constraint.
    fn solve_twist(&mut self, positions: &mut [Point3<f64>], inv_masses: &[f64], dt: f64) -> f64 {
        if self.num_vertices < 3 {
            return 0.0;
        }

        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];

        let w1 = inv_masses[v1];

        let p0 = positions[v0];
        let p1 = positions[v1];
        let p2 = positions[v2];

        // Edge direction (axis of twist)
        let edge = p2 - p0;
        let edge_len = edge.norm();

        if edge_len < 1e-10 {
            return 0.0;
        }

        let axis = edge / edge_len;

        // Vector from midpoint to v1 (perpendicular component determines twist)
        let midpoint = (p0.coords + p2.coords) / 2.0;
        let to_v1 = p1 - Point3::from(midpoint);

        // Project onto plane perpendicular to axis
        let projected = to_v1 - axis * to_v1.dot(&axis);
        let projected_len = projected.norm();

        if projected_len < 1e-10 {
            return 0.0;
        }

        // Current twist angle (simplified - uses projection magnitude)
        // For a proper twist constraint, we'd track material frames
        let c = projected_len - self.rest_twist.abs();

        if c.abs() < 1e-10 {
            return 0.0;
        }

        // Gradient direction (perpendicular to axis)
        let grad = projected / projected_len;

        // Only middle vertex moves for twist
        if w1 < 1e-10 {
            return 0.0;
        }

        // XPBD update
        let alpha_tilde = self.twist_compliance / (dt * dt);
        let delta_lambda = alpha_tilde.mul_add(-self.lambda_twist, -c) / (w1 + alpha_tilde);

        positions[v1] += grad * (w1 * delta_lambda);

        self.lambda_twist += delta_lambda;

        c.abs()
    }

    /// Evaluate stretch error without solving.
    #[must_use]
    pub fn evaluate_stretch(&self, positions: &[Point3<f64>]) -> f64 {
        let diff = positions[self.vertices[1]] - positions[self.vertices[0]];
        diff.norm() - self.rest_length
    }

    /// Evaluate shear error without solving.
    #[must_use]
    pub fn evaluate_shear(&self, positions: &[Point3<f64>]) -> f64 {
        if self.num_vertices < 3 {
            return 0.0;
        }

        let current_angle = Self::compute_angle(
            &positions[self.vertices[0]],
            &positions[self.vertices[1]],
            &positions[self.vertices[2]],
        );

        current_angle - self.rest_angle
    }
}

impl Default for FlexEdgeConstraint {
    fn default() -> Self {
        Self::stretch(0, 1, 1.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance_constraint() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Stretched
        ];
        let inv_masses = vec![1.0, 1.0];

        let mut constraint = DistanceConstraint::new(0, 1, 1.0, 0.0); // Rest length 1.0, rigid

        // Evaluate constraint
        let error_before = constraint.evaluate(&positions);
        assert!((error_before - 1.0).abs() < 1e-10); // Stretched by 1.0

        // Solve
        let dt = 1.0 / 60.0;
        constraint.solve(&mut positions, &inv_masses, dt);

        // Should be closer to rest length
        let error_after = constraint.evaluate(&positions);
        assert!(error_after.abs() < error_before.abs());
    }

    #[test]
    fn test_distance_constraint_pinned() {
        let mut positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)];
        let inv_masses = vec![0.0, 1.0]; // First vertex pinned

        let mut constraint = DistanceConstraint::new(0, 1, 1.0, 0.0);

        let dt = 1.0 / 60.0;
        constraint.solve(&mut positions, &inv_masses, dt);

        // Pinned vertex should not move
        assert!((positions[0].x - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_distance_constraint_from_positions() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 4.0, 0.0)];

        let constraint = DistanceConstraint::from_positions(0, 1, &positions, 0.001);

        // Rest length should be 5 (3-4-5 triangle)
        assert!((constraint.rest_length - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_bending_constraint_chain() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.5, 1.0, 0.0), // Bent
        ];
        let inv_masses = vec![1.0, 1.0, 1.0];

        // Rest angle is 180 degrees (straight line)
        let mut constraint = BendingConstraint::chain(0, 1, 2, std::f64::consts::PI, 0.001);

        let dt = 1.0 / 60.0;
        constraint.solve(&mut positions, &inv_masses, dt);

        // Should move toward straight line
        // (detailed behavior depends on the constraint implementation)
    }

    #[test]
    fn test_volume_constraint() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let inv_masses = vec![1.0, 1.0, 1.0, 1.0];

        // Create constraint with current volume as rest volume
        let vertices = [0, 1, 2, 3];
        let rest_volume = VolumeConstraint::compute_volume(&vertices, &positions);

        assert!((rest_volume - 1.0 / 6.0).abs() < 1e-10);

        // Scale the tetrahedron (change volume)
        positions[3] = Point3::new(0.0, 0.0, 2.0); // Double z

        let mut constraint = VolumeConstraint::new(vertices, rest_volume, 0.0);

        let dt = 1.0 / 60.0;
        let _error = constraint.solve(&mut positions, &inv_masses, dt);

        // Should reduce volume error
        let new_volume = VolumeConstraint::compute_volume(&vertices, &positions);
        assert!(
            (new_volume - rest_volume).abs() < (2.0 / 6.0 - rest_volume).abs(),
            "Volume should be closer to rest volume"
        );
    }

    #[test]
    fn test_constraint_enum() {
        let dist = Constraint::Distance(DistanceConstraint::new(0, 1, 1.0, 0.0));
        assert_eq!(dist.constraint_type(), ConstraintType::Distance);
        assert_eq!(dist.vertices().as_slice(), &[0, 1]);

        let bend =
            Constraint::Bending(BendingConstraint::chain(0, 1, 2, std::f64::consts::PI, 0.0));
        assert_eq!(bend.constraint_type(), ConstraintType::Bending);

        let vol = Constraint::Volume(VolumeConstraint::new([0, 1, 2, 3], 1.0, 0.0));
        assert_eq!(vol.constraint_type(), ConstraintType::Volume);

        let flex = Constraint::FlexEdge(FlexEdgeConstraint::stretch(0, 1, 1.0, 0.0));
        assert_eq!(flex.constraint_type(), ConstraintType::FlexEdge);
    }

    // ========================================================================
    // Flex Edge Constraint Tests
    // ========================================================================

    #[test]
    fn test_flex_edge_stretch() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Stretched
        ];
        let inv_masses = vec![1.0, 1.0];

        let mut constraint = FlexEdgeConstraint::stretch(0, 1, 1.0, 0.0);

        let error_before = constraint.evaluate_stretch(&positions);
        assert!((error_before - 1.0).abs() < 1e-10);

        let dt = 1.0 / 60.0;
        constraint.solve(&mut positions, &inv_masses, dt);

        let error_after = constraint.evaluate_stretch(&positions);
        assert!(error_after.abs() < error_before.abs());
    }

    #[test]
    fn test_flex_edge_stretch_from_positions() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 4.0, 0.0)];

        let constraint = FlexEdgeConstraint::stretch_from_positions(0, 1, &positions, 0.001);

        assert!((constraint.rest_length - 5.0).abs() < 1e-10);
        assert_eq!(constraint.edge_type(), FlexEdgeType::Stretch);
    }

    #[test]
    fn test_flex_edge_shear() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.5, 1.0, 0.0), // Bent
        ];
        let inv_masses = vec![1.0, 1.0, 1.0];

        // Rest angle is 180 degrees (straight)
        let mut constraint = FlexEdgeConstraint::shear([0, 1, 2], std::f64::consts::PI, 0.001);

        let error_before = constraint.evaluate_shear(&positions).abs();
        assert!(error_before > 0.0);

        let dt = 1.0 / 60.0;
        constraint.solve(&mut positions, &inv_masses, dt);

        // Error should decrease
        let error_after = constraint.evaluate_shear(&positions).abs();
        assert!(error_after < error_before);
    }

    #[test]
    fn test_flex_edge_shear_from_positions() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Straight line
        ];

        let constraint = FlexEdgeConstraint::shear_from_positions([0, 1, 2], &positions, 0.001);

        // Rest angle should be PI (180 degrees, straight)
        assert!((constraint.rest_angle - std::f64::consts::PI).abs() < 1e-10);
        assert_eq!(constraint.edge_type(), FlexEdgeType::Shear);
    }

    #[test]
    fn test_flex_edge_stretch_shear() {
        let mut positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Stretched
            Point3::new(3.5, 1.0, 0.0), // Bent
        ];
        let inv_masses = vec![1.0, 1.0, 1.0];

        let mut constraint = FlexEdgeConstraint::stretch_shear(
            [0, 1, 2],
            1.0,                  // rest length
            std::f64::consts::PI, // rest angle (straight)
            0.0,                  // rigid stretch
            0.001,                // soft shear
        );

        let dt = 1.0 / 60.0;
        let error = constraint.solve(&mut positions, &inv_masses, dt);

        // Should have corrected some error
        assert!(error > 0.0);
        assert_eq!(constraint.edge_type(), FlexEdgeType::StretchShear);
    }

    #[test]
    fn test_flex_edge_stretch_shear_from_positions() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let constraint =
            FlexEdgeConstraint::stretch_shear_from_positions([0, 1, 2], &positions, 0.001, 0.001);

        assert!((constraint.rest_length - 1.0).abs() < 1e-10);
        assert!((constraint.rest_angle - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn test_flex_edge_with_damping() {
        let constraint = FlexEdgeConstraint::stretch(0, 1, 1.0, 0.0).with_damping(0.5);

        assert!((constraint.damping - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_flex_edge_reset() {
        let mut constraint =
            FlexEdgeConstraint::stretch_shear([0, 1, 2], 1.0, std::f64::consts::PI, 0.0, 0.0);

        // Simulate some solving to accumulate lambda
        constraint.lambda_stretch = 1.0;
        constraint.lambda_shear = 2.0;

        constraint.reset();

        assert!((constraint.lambda_stretch).abs() < 1e-10);
        assert!((constraint.lambda_shear).abs() < 1e-10);
    }

    #[test]
    fn test_flex_edge_num_vertices() {
        let stretch = FlexEdgeConstraint::stretch(0, 1, 1.0, 0.0);
        assert_eq!(stretch.num_vertices(), 2);

        let shear = FlexEdgeConstraint::shear([0, 1, 2], std::f64::consts::PI, 0.0);
        assert_eq!(shear.num_vertices(), 3);
    }

    #[test]
    fn test_flex_edge_default() {
        let constraint = FlexEdgeConstraint::default();
        assert_eq!(constraint.edge_type(), FlexEdgeType::Stretch);
        assert!((constraint.rest_length - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_flex_edge_twist() {
        // Positions are defined but not used - the twist constraint
        // just needs vertex indices for this basic test
        let _positions = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0), // Middle vertex
            Point3::new(0.0, 0.0, 2.0),
        ];

        let constraint = FlexEdgeConstraint::twist([0, 1, 2], 0.0, 0.001);

        assert_eq!(constraint.edge_type(), FlexEdgeType::Twist);
        assert_eq!(constraint.num_vertices(), 3);
    }

    #[test]
    fn test_flex_edge_pinned_vertices() {
        let mut positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)];
        let inv_masses = vec![0.0, 0.0]; // Both pinned

        let mut constraint = FlexEdgeConstraint::stretch(0, 1, 1.0, 0.0);

        let dt = 1.0 / 60.0;
        let error = constraint.solve(&mut positions, &inv_masses, dt);

        // No movement should happen
        assert!((positions[0].x - 0.0).abs() < 1e-10);
        assert!((positions[1].x - 2.0).abs() < 1e-10);
        assert!((error).abs() < 1e-10); // Error is 0 due to early return
    }
}
