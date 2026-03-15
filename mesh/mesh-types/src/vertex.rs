//! Vertex color type.

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn color_from_float() {
        let c = VertexColor::from_float(1.0, 0.5, 0.0);
        assert_eq!(c.r, 255);
        assert!((i32::from(c.g) - 127).abs() <= 1);
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
        assert!((i32::from(c.b) - 127).abs() <= 1);
    }
}
