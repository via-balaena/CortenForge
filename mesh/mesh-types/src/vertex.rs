//! Vertex types and attributes.

use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// RGB color with 8-bit components.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VertexColor {
    /// Red component (0-255).
    pub r: u8,
    /// Green component (0-255).
    pub g: u8,
    /// Blue component (0-255).
    pub b: u8,
}

impl VertexColor {
    /// Create a new color from RGB components.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::VertexColor;
    ///
    /// let red = VertexColor::new(255, 0, 0);
    /// assert_eq!(red.r, 255);
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Create a color from floating point values in [0, 1] range.
    ///
    /// Values are clamped to the valid range.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::VertexColor;
    ///
    /// let color = VertexColor::from_float(1.0, 0.5, 0.0);
    /// assert_eq!(color.r, 255);
    /// assert_eq!(color.g, 127);
    /// assert_eq!(color.b, 0);
    /// ```
    #[inline]
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    // Truncation and sign loss are safe: values are clamped to [0.0, 1.0] before * 255.0
    pub fn from_float(r: f32, g: f32, b: f32) -> Self {
        Self {
            r: (r.clamp(0.0, 1.0) * 255.0) as u8,
            g: (g.clamp(0.0, 1.0) * 255.0) as u8,
            b: (b.clamp(0.0, 1.0) * 255.0) as u8,
        }
    }

    /// Convert to floating point values in [0, 1] range.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::VertexColor;
    ///
    /// let color = VertexColor::new(255, 128, 0);
    /// let (r, g, b) = color.to_float();
    /// assert!((r - 1.0).abs() < 0.01);
    /// assert!((g - 0.5).abs() < 0.01);
    /// assert!((b - 0.0).abs() < 0.01);
    /// ```
    #[inline]
    #[must_use]
    pub fn to_float(self) -> (f32, f32, f32) {
        (
            f32::from(self.r) / 255.0,
            f32::from(self.g) / 255.0,
            f32::from(self.b) / 255.0,
        )
    }

    /// Black color (0, 0, 0).
    pub const BLACK: Self = Self::new(0, 0, 0);

    /// White color (255, 255, 255).
    pub const WHITE: Self = Self::new(255, 255, 255);

    /// Red color (255, 0, 0).
    pub const RED: Self = Self::new(255, 0, 0);

    /// Green color (0, 255, 0).
    pub const GREEN: Self = Self::new(0, 255, 0);

    /// Blue color (0, 0, 255).
    pub const BLUE: Self = Self::new(0, 0, 255);
}

impl Default for VertexColor {
    fn default() -> Self {
        Self::WHITE
    }
}

/// Optional attributes that can be attached to a vertex.
///
/// These attributes are computed or assigned by various mesh operations:
/// - `normal`: Computed from adjacent faces (area-weighted average)
/// - `color`: From file format or visualization
/// - `zone_id`: Assigned by zone classification algorithms
/// - `clearance_mm`: Distance to nearest obstacle or boundary
/// - `offset`: Per-vertex offset for variable shell thickness
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VertexAttributes {
    /// Unit normal vector, computed from adjacent faces.
    pub normal: Option<Vector3<f64>>,

    /// Vertex color (RGB).
    pub color: Option<VertexColor>,

    /// Zone identifier (e.g., anatomical region, material zone).
    pub zone_id: Option<u32>,

    /// Clearance distance in millimeters.
    pub clearance_mm: Option<f32>,

    /// Offset distance for variable shell thickness.
    /// Positive = outward expansion, negative = compression.
    pub offset: Option<f32>,

    /// Texture coordinates (U, V).
    pub uv: Option<(f32, f32)>,
}

impl VertexAttributes {
    /// Create empty attributes with no values set.
    #[inline]
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            normal: None,
            color: None,
            zone_id: None,
            clearance_mm: None,
            offset: None,
            uv: None,
        }
    }

    /// Create attributes with just a normal.
    #[inline]
    #[must_use]
    pub const fn with_normal(normal: Vector3<f64>) -> Self {
        Self {
            normal: Some(normal),
            color: None,
            zone_id: None,
            clearance_mm: None,
            offset: None,
            uv: None,
        }
    }

    /// Create attributes with just a color.
    #[inline]
    #[must_use]
    pub const fn with_color(color: VertexColor) -> Self {
        Self {
            normal: None,
            color: Some(color),
            zone_id: None,
            clearance_mm: None,
            offset: None,
            uv: None,
        }
    }

    /// Check if any attributes are set.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.normal.is_none()
            && self.color.is_none()
            && self.zone_id.is_none()
            && self.clearance_mm.is_none()
            && self.offset.is_none()
            && self.uv.is_none()
    }
}

/// A vertex in 3D space with optional attributes.
///
/// The position is stored as a `Point3<f64>` for high precision.
/// Attributes are optional and stored separately to minimize memory
/// usage when not needed.
///
/// # Example
///
/// ```
/// use mesh_types::{Vertex, Point3};
///
/// // Create a vertex with just position
/// let v1 = Vertex::new(Point3::new(1.0, 2.0, 3.0));
///
/// // Create from raw coordinates
/// let v2 = Vertex::from_coords(1.0, 2.0, 3.0);
///
/// assert_eq!(v1.position, v2.position);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Vertex {
    /// 3D position.
    pub position: Point3<f64>,

    /// Optional attributes (normal, color, zone, etc.).
    pub attributes: VertexAttributes,
}

impl Vertex {
    /// Create a new vertex with only position set.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Vertex, Point3};
    ///
    /// let v = Vertex::new(Point3::new(1.0, 2.0, 3.0));
    /// assert_eq!(v.position.x, 1.0);
    /// assert!(v.attributes.is_empty());
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(position: Point3<f64>) -> Self {
        Self {
            position,
            attributes: VertexAttributes::empty(),
        }
    }

    /// Create a vertex from raw coordinates.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::Vertex;
    ///
    /// let v = Vertex::from_coords(1.0, 2.0, 3.0);
    /// assert_eq!(v.position.x, 1.0);
    /// assert_eq!(v.position.y, 2.0);
    /// assert_eq!(v.position.z, 3.0);
    /// ```
    #[inline]
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // Point3::new is not const in nalgebra
    pub fn from_coords(x: f64, y: f64, z: f64) -> Self {
        Self::new(Point3::new(x, y, z))
    }

    /// Create a vertex with position and normal.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Vertex, Point3, Vector3};
    ///
    /// let v = Vertex::with_normal(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Vector3::new(0.0, 0.0, 1.0)
    /// );
    /// assert!(v.attributes.normal.is_some());
    /// ```
    #[inline]
    #[must_use]
    pub const fn with_normal(position: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            position,
            attributes: VertexAttributes::with_normal(normal),
        }
    }

    /// Create a vertex with position and color.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Vertex, VertexColor, Point3};
    ///
    /// let v = Vertex::with_color(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     VertexColor::RED
    /// );
    /// assert_eq!(v.attributes.color, Some(VertexColor::RED));
    /// ```
    #[inline]
    #[must_use]
    pub const fn with_color(position: Point3<f64>, color: VertexColor) -> Self {
        Self {
            position,
            attributes: VertexAttributes::with_color(color),
        }
    }

    /// Get the normal if set.
    #[inline]
    #[must_use]
    pub const fn normal(&self) -> Option<Vector3<f64>> {
        self.attributes.normal
    }

    /// Get the color if set.
    #[inline]
    #[must_use]
    pub const fn color(&self) -> Option<VertexColor> {
        self.attributes.color
    }

    /// Get the zone ID if set.
    #[inline]
    #[must_use]
    pub const fn zone_id(&self) -> Option<u32> {
        self.attributes.zone_id
    }
}

impl From<Point3<f64>> for Vertex {
    fn from(position: Point3<f64>) -> Self {
        Self::new(position)
    }
}

impl From<[f64; 3]> for Vertex {
    fn from([x, y, z]: [f64; 3]) -> Self {
        Self::from_coords(x, y, z)
    }
}

impl From<(f64, f64, f64)> for Vertex {
    fn from((x, y, z): (f64, f64, f64)) -> Self {
        Self::from_coords(x, y, z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_from_coords() {
        let v = Vertex::from_coords(1.0, 2.0, 3.0);
        assert!((v.position.x - 1.0).abs() < f64::EPSILON);
        assert!((v.position.y - 2.0).abs() < f64::EPSILON);
        assert!((v.position.z - 3.0).abs() < f64::EPSILON);
        assert!(v.attributes.is_empty());
    }

    #[test]
    fn vertex_with_normal() {
        let v = Vertex::with_normal(Point3::origin(), Vector3::z());
        assert!(v.normal().is_some());
        let n = v.normal();
        assert!(n.is_some());
        let n = n.map(|n| (n.x, n.y, n.z));
        assert_eq!(n, Some((0.0, 0.0, 1.0)));
    }

    #[test]
    fn vertex_from_tuple() {
        let v: Vertex = (1.0, 2.0, 3.0).into();
        assert!((v.position.x - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn vertex_from_array() {
        let v: Vertex = [1.0, 2.0, 3.0].into();
        assert!((v.position.x - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn color_from_float() {
        let c = VertexColor::from_float(1.0, 0.5, 0.0);
        assert_eq!(c.r, 255);
        assert!((c.g as i32 - 127).abs() <= 1);
        assert_eq!(c.b, 0);
    }

    #[test]
    fn color_to_float() {
        let c = VertexColor::new(255, 128, 0);
        let (r, g, b) = c.to_float();
        assert!((r - 1.0).abs() < 0.01);
        assert!((g - 0.502).abs() < 0.01);
        assert!(b.abs() < 0.01);
    }

    #[test]
    fn color_clamps_values() {
        let c = VertexColor::from_float(2.0, -1.0, 0.5);
        assert_eq!(c.r, 255);
        assert_eq!(c.g, 0);
        assert!((c.b as i32 - 127).abs() <= 1);
    }

    #[test]
    fn attributes_is_empty() {
        let empty = VertexAttributes::empty();
        assert!(empty.is_empty());

        let with_normal = VertexAttributes::with_normal(Vector3::z());
        assert!(!with_normal.is_empty());
    }
}
