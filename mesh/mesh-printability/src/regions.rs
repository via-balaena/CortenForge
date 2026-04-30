//! Region types for printability analysis.
//!
//! Defines structures for thin wall, overhang, long-bridge, and
//! support regions detected during validation.

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
    pub const fn new(center: Point3<f64>, thickness: f64) -> Self {
        Self {
            center,
            thickness,
            area: 0.0,
            faces: Vec::new(),
        }
    }

    /// Set the area of the region.
    #[must_use]
    pub const fn with_area(mut self, area: f64) -> Self {
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
    pub const fn new(center: Point3<f64>, angle: f64, area: f64) -> Self {
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

/// Information about a long-bridge region (§6.2 Gap G).
///
/// A long bridge is a connected component of near-horizontal,
/// downward-facing faces (within `ANGLE_TOL_HORIZONTAL` of `-up`) whose
/// bbox max-extent in the plane perpendicular to `up` exceeds
/// `config.max_bridge_span`. FDM/SLA-relevant; SLS/MJF skip silently.
///
/// Reported `start` and `end` are the back-projected endpoints of the
/// cluster's longest bbox axis in the plane perpendicular to `up`,
/// lifted back to 3D at the cluster centroid's elevation along `up`.
/// `span` is the bbox max-extent (the larger of the two perpendicular-
/// plane axis extents). `edges` are the cluster's boundary edges
/// (`(min, max)`-normalized vertex-index pairs shared with non-candidate
/// faces).
#[derive(Debug, Clone)]
pub struct LongBridgeRegion {
    /// Back-projected start of the bridge along the longest bbox axis.
    pub start: Point3<f64>,
    /// Back-projected end of the bridge along the longest bbox axis.
    pub end: Point3<f64>,
    /// Bridge span in mm (max of the two perpendicular-plane bbox extents).
    pub span: f64,
    /// Boundary edges shared with non-candidate faces, `(min, max)`-normalized.
    pub edges: Vec<(u32, u32)>,
    /// Face indices in this cluster.
    pub faces: Vec<u32>,
}

impl LongBridgeRegion {
    /// Create a new long-bridge region.
    #[must_use]
    pub const fn new(start: Point3<f64>, end: Point3<f64>, span: f64) -> Self {
        Self {
            start,
            end,
            span,
            edges: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Set the boundary edges.
    #[must_use]
    pub fn with_edges(mut self, edges: Vec<(u32, u32)>) -> Self {
        self.edges = edges;
        self
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
    pub const fn new(center: Point3<f64>, volume: f64, max_height: f64) -> Self {
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
        let region =
            OverhangRegion::new(Point3::new(5.0, 5.0, 5.0), 60.0, 50.0).with_faces(vec![10, 20]);

        assert!((region.angle - 60.0).abs() < f64::EPSILON);
        assert!((region.area - 50.0).abs() < f64::EPSILON);
        assert_eq!(region.faces.len(), 2);
    }

    #[test]
    fn test_long_bridge_region() {
        let region = LongBridgeRegion::new(
            Point3::new(0.0, 5.0, 10.0),
            Point3::new(20.0, 5.0, 10.0),
            20.0,
        )
        .with_edges(vec![(0, 1), (2, 3)])
        .with_faces(vec![1, 2, 3, 4]);

        assert!((region.span - 20.0).abs() < f64::EPSILON);
        assert!((region.start.x - 0.0).abs() < f64::EPSILON);
        assert!((region.end.x - 20.0).abs() < f64::EPSILON);
        assert_eq!(region.edges.len(), 2);
        assert_eq!(region.faces.len(), 4);
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
