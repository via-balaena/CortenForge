//! Region types for printability analysis.
//!
//! Defines structures for thin wall regions, overhang regions,
//! and support regions detected during validation.

use mesh_types::Point3;

/// Information about a thin wall region.
#[derive(Debug, Clone)]
pub struct ThinWallRegion {
    /// Center of the thin region.
    pub center: Point3<f64>,
    /// Minimum thickness found in mm.
    pub thickness: f64,
    /// Approximate area of the thin region in mm squared.
    pub area: f64,
    /// Face indices in this region.
    pub faces: Vec<u32>,
}

impl ThinWallRegion {
    /// Create a new thin wall region.
    #[must_use]
    pub fn new(center: Point3<f64>, thickness: f64) -> Self {
        Self {
            center,
            thickness,
            area: 0.0,
            faces: Vec::new(),
        }
    }

    /// Set the area of the region.
    #[must_use]
    pub fn with_area(mut self, area: f64) -> Self {
        self.area = area;
        self
    }

    /// Set the affected faces.
    #[must_use]
    pub fn with_faces(mut self, faces: Vec<u32>) -> Self {
        self.faces = faces;
        self
    }
}

/// Information about an overhang region.
#[derive(Debug, Clone)]
pub struct OverhangRegion {
    /// Center of the overhang region.
    pub center: Point3<f64>,
    /// Maximum overhang angle in degrees.
    pub angle: f64,
    /// Approximate area needing support in mm squared.
    pub area: f64,
    /// Face indices in this region.
    pub faces: Vec<u32>,
}

impl OverhangRegion {
    /// Create a new overhang region.
    #[must_use]
    pub fn new(center: Point3<f64>, angle: f64, area: f64) -> Self {
        Self {
            center,
            angle,
            area,
            faces: Vec::new(),
        }
    }

    /// Set the affected faces.
    #[must_use]
    pub fn with_faces(mut self, faces: Vec<u32>) -> Self {
        self.faces = faces;
        self
    }
}

/// Information about a region that needs support.
#[derive(Debug, Clone)]
pub struct SupportRegion {
    /// Center of the support region.
    pub center: Point3<f64>,
    /// Estimated support volume in mm cubed.
    pub volume: f64,
    /// Maximum height that needs support in mm.
    pub max_height: f64,
    /// Face indices needing support.
    pub faces: Vec<u32>,
}

impl SupportRegion {
    /// Create a new support region.
    #[must_use]
    pub fn new(center: Point3<f64>, volume: f64, max_height: f64) -> Self {
        Self {
            center,
            volume,
            max_height,
            faces: Vec::new(),
        }
    }

    /// Set the affected faces.
    #[must_use]
    pub fn with_faces(mut self, faces: Vec<u32>) -> Self {
        self.faces = faces;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thin_wall_region() {
        let region = ThinWallRegion::new(Point3::new(10.0, 20.0, 30.0), 0.5)
            .with_area(100.0)
            .with_faces(vec![1, 2, 3]);

        assert!((region.thickness - 0.5).abs() < f64::EPSILON);
        assert!((region.area - 100.0).abs() < f64::EPSILON);
        assert_eq!(region.faces.len(), 3);
    }

    #[test]
    fn test_overhang_region() {
        let region = OverhangRegion::new(Point3::new(5.0, 5.0, 5.0), 60.0, 50.0)
            .with_faces(vec![10, 20]);

        assert!((region.angle - 60.0).abs() < f64::EPSILON);
        assert!((region.area - 50.0).abs() < f64::EPSILON);
        assert_eq!(region.faces.len(), 2);
    }

    #[test]
    fn test_support_region() {
        let region = SupportRegion::new(Point3::new(0.0, 0.0, 10.0), 500.0, 10.0)
            .with_faces(vec![1, 2, 3, 4, 5]);

        assert!((region.volume - 500.0).abs() < f64::EPSILON);
        assert!((region.max_height - 10.0).abs() < f64::EPSILON);
        assert_eq!(region.faces.len(), 5);
    }
}
