//! Constraint types for mesh morphing.
//!
//! Constraints define how source points should be moved to target positions
//! during deformation. They can be:
//! - Point-to-point mappings (source → target)
//! - Displacement-based (source + vector)
//! - Weighted (to control influence)

use nalgebra::{Point3, Vector3};

/// A constraint that defines a point mapping for mesh deformation.
///
/// Constraints specify that a source point should be moved to a target position.
/// Multiple constraints work together to define the overall deformation field.
///
/// # Examples
///
/// ```
/// use mesh_morph::Constraint;
/// use nalgebra::{Point3, Vector3};
///
/// // Point-to-point constraint
/// let c1 = Constraint::point(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
/// );
///
/// // Displacement-based constraint
/// let c2 = Constraint::displacement(
///     Point3::new(0.0, 0.0, 0.0),
///     Vector3::new(1.0, 0.0, 0.0),
/// );
///
/// // Weighted constraint
/// let c3 = Constraint::weighted(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     0.5, // Half influence
/// );
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint {
    /// The original position.
    pub source: Point3<f64>,
    /// The target position.
    pub target: Point3<f64>,
    /// The influence weight (default 1.0).
    pub weight: f64,
}

impl Constraint {
    /// Creates a point-to-point constraint with unit weight.
    ///
    /// # Arguments
    ///
    /// * `source` - The original position
    /// * `target` - The desired target position
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::Point3;
    ///
    /// let constraint = Constraint::point(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 2.0, 3.0),
    /// );
    ///
    /// assert_eq!(constraint.weight, 1.0);
    /// ```
    #[must_use]
    pub const fn point(source: Point3<f64>, target: Point3<f64>) -> Self {
        Self {
            source,
            target,
            weight: 1.0,
        }
    }

    /// Creates a weighted point-to-point constraint.
    ///
    /// Higher weights give the constraint more influence on nearby vertices.
    /// A weight of 0.0 effectively disables the constraint.
    ///
    /// # Arguments
    ///
    /// * `source` - The original position
    /// * `target` - The desired target position
    /// * `weight` - The influence weight (typically in range 0.0 to 1.0)
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::Point3;
    ///
    /// let constraint = Constraint::weighted(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     0.5,
    /// );
    ///
    /// assert_eq!(constraint.weight, 0.5);
    /// ```
    #[must_use]
    pub const fn weighted(source: Point3<f64>, target: Point3<f64>, weight: f64) -> Self {
        Self {
            source,
            target,
            weight,
        }
    }

    /// Creates a constraint from a source point and displacement vector.
    ///
    /// The target position is computed as `source + displacement`.
    ///
    /// # Arguments
    ///
    /// * `source` - The original position
    /// * `displacement` - The displacement vector to apply
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let constraint = Constraint::displacement(
    ///     Point3::new(1.0, 2.0, 3.0),
    ///     Vector3::new(0.0, 0.0, 1.0),
    /// );
    ///
    /// assert_eq!(constraint.target, Point3::new(1.0, 2.0, 4.0));
    /// ```
    #[must_use]
    pub fn displacement(source: Point3<f64>, displacement: Vector3<f64>) -> Self {
        Self {
            source,
            target: source + displacement,
            weight: 1.0,
        }
    }

    /// Creates a constraint from vertex indices in a mesh.
    ///
    /// This is a convenience method for creating constraints based on
    /// mesh vertex indices rather than explicit coordinates.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The source mesh
    /// * `source_index` - Index of the source vertex
    /// * `target` - The target position
    ///
    /// # Returns
    ///
    /// The constraint, or `None` if the index is out of bounds.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let constraint = Constraint::from_vertex_index(
    ///     &mesh,
    ///     0,
    ///     Point3::new(1.0, 0.0, 0.0),
    /// );
    ///
    /// assert!(constraint.is_some());
    /// ```
    #[must_use]
    pub fn from_vertex_index(
        mesh: &mesh_types::IndexedMesh,
        source_index: usize,
        target: Point3<f64>,
    ) -> Option<Self> {
        mesh.vertices.get(source_index).map(|v| Self {
            source: v.position,
            target,
            weight: 1.0,
        })
    }

    /// Returns the displacement vector from source to target.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let constraint = Constraint::point(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 2.0, 3.0),
    /// );
    ///
    /// let disp = constraint.displacement_vector();
    /// assert_eq!(disp, Vector3::new(1.0, 2.0, 3.0));
    /// ```
    #[must_use]
    pub fn displacement_vector(&self) -> Vector3<f64> {
        self.target - self.source
    }

    /// Returns the distance from source to target.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::Point3;
    ///
    /// let constraint = Constraint::point(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(3.0, 4.0, 0.0),
    /// );
    ///
    /// assert!((constraint.distance() - 5.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn distance(&self) -> f64 {
        (self.target - self.source).norm()
    }

    /// Returns whether this is an identity constraint (source equals target).
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::Constraint;
    /// use nalgebra::Point3;
    ///
    /// let identity = Constraint::point(
    ///     Point3::new(1.0, 2.0, 3.0),
    ///     Point3::new(1.0, 2.0, 3.0),
    /// );
    /// assert!(identity.is_identity(1e-10));
    ///
    /// let non_identity = Constraint::point(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    /// );
    /// assert!(!non_identity.is_identity(1e-10));
    /// ```
    #[must_use]
    pub fn is_identity(&self, tolerance: f64) -> bool {
        self.distance() < tolerance
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;

    #[test]
    fn test_point_constraint() {
        let c = Constraint::point(Point3::origin(), Point3::new(1.0, 2.0, 3.0));
        assert_eq!(c.source, Point3::origin());
        assert_eq!(c.target, Point3::new(1.0, 2.0, 3.0));
        assert!((c.weight - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_weighted_constraint() {
        let c = Constraint::weighted(Point3::origin(), Point3::new(1.0, 0.0, 0.0), 0.5);
        assert!((c.weight - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_displacement_constraint() {
        let c = Constraint::displacement(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 5.0));
        assert_eq!(c.source, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(c.target, Point3::new(1.0, 2.0, 8.0));
    }

    #[test]
    fn test_displacement_vector() {
        let c = Constraint::point(Point3::origin(), Point3::new(3.0, 4.0, 0.0));
        let disp = c.displacement_vector();
        assert_eq!(disp, Vector3::new(3.0, 4.0, 0.0));
    }

    #[test]
    fn test_distance() {
        let c = Constraint::point(Point3::origin(), Point3::new(3.0, 4.0, 0.0));
        assert!((c.distance() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_is_identity() {
        let identity = Constraint::point(Point3::new(1.0, 2.0, 3.0), Point3::new(1.0, 2.0, 3.0));
        assert!(identity.is_identity(1e-10));

        let non_identity = Constraint::point(Point3::origin(), Point3::new(0.001, 0.0, 0.0));
        assert!(!non_identity.is_identity(1e-10));
        assert!(non_identity.is_identity(0.01));
    }

    #[test]
    fn test_from_vertex_index() {
        let mut mesh = mesh_types::IndexedMesh::new();
        mesh.vertices
            .push(mesh_types::Vertex::from_coords(1.0, 2.0, 3.0));

        let c = Constraint::from_vertex_index(&mesh, 0, Point3::new(4.0, 5.0, 6.0));
        assert!(c.is_some());
        let c = c.unwrap();
        assert_eq!(c.source, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(c.target, Point3::new(4.0, 5.0, 6.0));

        // Out of bounds
        let c = Constraint::from_vertex_index(&mesh, 10, Point3::origin());
        assert!(c.is_none());
    }
}
