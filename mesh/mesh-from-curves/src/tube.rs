//! Tube mesh generation from curves.
//!
//! Creates cylindrical tubes around polyline curves.

use mesh_types::{IndexedMesh, Vertex};
use nalgebra::Point3;

use crate::error::{CurveError, CurveResult};
use crate::frame::parallel_transport_frames;

/// Configuration for tube generation.
#[derive(Debug, Clone)]
pub struct TubeConfig {
    /// Radius of the tube.
    pub radius: f64,
    /// Number of segments around the circumference.
    pub segments: usize,
    /// Whether to cap the ends of the tube.
    pub capped: bool,
}

impl Default for TubeConfig {
    fn default() -> Self {
        Self {
            radius: 1.0,
            segments: 16,
            capped: true,
        }
    }
}

impl TubeConfig {
    /// Create a tube config with the given radius.
    #[must_use]
    pub fn with_radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }

    /// Create a tube config with the given number of circumferential segments.
    #[must_use]
    pub fn with_segments(mut self, segments: usize) -> Self {
        self.segments = segments;
        self
    }

    /// Create a tube config without end caps.
    #[must_use]
    pub fn uncapped(mut self) -> Self {
        self.capped = false;
        self
    }
}

/// Generate a tube mesh around a polyline curve.
///
/// # Arguments
///
/// * `points` - Points defining the center line of the tube
/// * `config` - Tube configuration
///
/// # Returns
///
/// A triangle mesh representing the tube.
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 2 points are provided
/// - Radius is not positive
/// - Fewer than 3 segments are requested
///
/// # Example
///
/// ```
/// use mesh_from_curves::{tube_from_polyline, TubeConfig};
/// use nalgebra::Point3;
///
/// let points = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(0.0, 0.0, 10.0),
/// ];
///
/// let config = TubeConfig::default().with_radius(0.5);
/// let mesh = tube_from_polyline(&points, &config).unwrap();
///
/// assert!(!mesh.vertices.is_empty());
/// assert!(!mesh.faces.is_empty());
/// ```
pub fn tube_from_polyline(points: &[Point3<f64>], config: &TubeConfig) -> CurveResult<IndexedMesh> {
    // Validate inputs
    if points.len() < 2 {
        return Err(CurveError::TooFewPoints {
            min: 2,
            actual: points.len(),
        });
    }

    if config.radius <= 0.0 || !config.radius.is_finite() {
        return Err(CurveError::InvalidRadius(config.radius));
    }

    if config.segments < 3 {
        return Err(CurveError::TooFewSegments {
            min: 3,
            actual: config.segments,
        });
    }

    // Compute frames along the curve
    let frames = parallel_transport_frames(points);

    if frames.is_empty() {
        return Err(CurveError::TooFewPoints {
            min: 2,
            actual: points.len(),
        });
    }

    let mut mesh = IndexedMesh::new();
    let n_rings = points.len();
    let n_segs = config.segments;

    // Generate vertices
    for (ring_idx, (point, frame)) in points.iter().zip(frames.iter()).enumerate() {
        for seg_idx in 0..n_segs {
            let angle = 2.0 * std::f64::consts::PI * (seg_idx as f64) / (n_segs as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            let offset =
                frame.normal * cos_a * config.radius + frame.binormal * sin_a * config.radius;
            let pos = Point3::from(point.coords + offset);

            // Normal points outward
            let normal = (frame.normal * cos_a + frame.binormal * sin_a)
                .try_normalize(f64::EPSILON)
                .unwrap_or(frame.normal);

            mesh.vertices.push(Vertex::with_normal(pos, normal));

            // Create faces (except for last ring)
            if ring_idx < n_rings - 1 {
                let curr = (ring_idx * n_segs + seg_idx) as u32;
                let next_seg = (ring_idx * n_segs + (seg_idx + 1) % n_segs) as u32;
                let next_ring = ((ring_idx + 1) * n_segs + seg_idx) as u32;
                let next_both = ((ring_idx + 1) * n_segs + (seg_idx + 1) % n_segs) as u32;

                // Two triangles for this quad
                mesh.faces.push([curr, next_ring, next_seg]);
                mesh.faces.push([next_seg, next_ring, next_both]);
            }
        }
    }

    // Add end caps if requested
    if config.capped && n_rings >= 2 {
        add_cap(&mut mesh, &frames[0], points[0], n_segs, 0, true);
        let last_ring_start = (n_rings - 1) * n_segs;
        add_cap(
            &mut mesh,
            &frames[n_rings - 1],
            points[n_rings - 1],
            n_segs,
            last_ring_start,
            false,
        );
    }

    Ok(mesh)
}

/// Add an end cap to the tube.
fn add_cap(
    mesh: &mut IndexedMesh,
    frame: &crate::frame::Frame,
    center: Point3<f64>,
    n_segs: usize,
    ring_start: usize,
    is_start: bool,
) {
    // Add center vertex
    let normal = if is_start {
        -frame.tangent
    } else {
        frame.tangent
    };
    let center_idx = mesh.vertices.len() as u32;
    mesh.vertices.push(Vertex::with_normal(center, normal));

    // Create fan triangles
    for seg_idx in 0..n_segs {
        let curr = (ring_start + seg_idx) as u32;
        let next = (ring_start + (seg_idx + 1) % n_segs) as u32;

        if is_start {
            // Wind counterclockwise when looking from outside
            mesh.faces.push([center_idx, next, curr]);
        } else {
            mesh.faces.push([center_idx, curr, next]);
        }
    }
}

/// Generate a tube mesh with a variable radius along the curve.
///
/// # Arguments
///
/// * `points` - Points defining the center line
/// * `radii` - Radius at each point (must have same length as points)
/// * `segments` - Number of circumferential segments
/// * `capped` - Whether to cap the ends
///
/// # Returns
///
/// A triangle mesh representing the variable-radius tube.
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 2 points are provided
/// - Points and radii have different lengths
/// - Any radius is not positive
/// - Fewer than 3 segments are requested
pub fn tube_variable_radius(
    points: &[Point3<f64>],
    radii: &[f64],
    segments: usize,
    capped: bool,
) -> CurveResult<IndexedMesh> {
    if points.len() < 2 {
        return Err(CurveError::TooFewPoints {
            min: 2,
            actual: points.len(),
        });
    }

    if points.len() != radii.len() {
        return Err(CurveError::TooFewPoints {
            min: points.len(),
            actual: radii.len(),
        });
    }

    if segments < 3 {
        return Err(CurveError::TooFewSegments {
            min: 3,
            actual: segments,
        });
    }

    for (i, &r) in radii.iter().enumerate() {
        if r <= 0.0 || !r.is_finite() {
            return Err(CurveError::InvalidRadius(r));
        }
        // Check first radius for reporting
        if i == 0 && r <= 0.0 {
            return Err(CurveError::InvalidRadius(r));
        }
    }

    let frames = parallel_transport_frames(points);
    if frames.is_empty() {
        return Err(CurveError::TooFewPoints {
            min: 2,
            actual: points.len(),
        });
    }

    let mut mesh = IndexedMesh::new();
    let n_rings = points.len();
    let n_segs = segments;

    // Generate vertices
    for (ring_idx, ((point, frame), &radius)) in points
        .iter()
        .zip(frames.iter())
        .zip(radii.iter())
        .enumerate()
    {
        for seg_idx in 0..n_segs {
            let angle = 2.0 * std::f64::consts::PI * (seg_idx as f64) / (n_segs as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            let offset = frame.normal * cos_a * radius + frame.binormal * sin_a * radius;
            let pos = Point3::from(point.coords + offset);

            let normal = (frame.normal * cos_a + frame.binormal * sin_a)
                .try_normalize(f64::EPSILON)
                .unwrap_or(frame.normal);

            mesh.vertices.push(Vertex::with_normal(pos, normal));

            if ring_idx < n_rings - 1 {
                let curr = (ring_idx * n_segs + seg_idx) as u32;
                let next_seg = (ring_idx * n_segs + (seg_idx + 1) % n_segs) as u32;
                let next_ring = ((ring_idx + 1) * n_segs + seg_idx) as u32;
                let next_both = ((ring_idx + 1) * n_segs + (seg_idx + 1) % n_segs) as u32;

                mesh.faces.push([curr, next_ring, next_seg]);
                mesh.faces.push([next_seg, next_ring, next_both]);
            }
        }
    }

    if capped && n_rings >= 2 {
        add_cap(&mut mesh, &frames[0], points[0], n_segs, 0, true);
        let last_ring_start = (n_rings - 1) * n_segs;
        add_cap(
            &mut mesh,
            &frames[n_rings - 1],
            points[n_rings - 1],
            n_segs,
            last_ring_start,
            false,
        );
    }

    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_line() -> Vec<Point3<f64>> {
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 10.0)]
    }

    fn multi_segment() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(0.0, 0.0, 10.0),
        ]
    }

    #[test]
    fn tube_config_default() {
        let config = TubeConfig::default();
        assert!(config.radius > 0.0);
        assert!(config.segments >= 3);
        assert!(config.capped);
    }

    #[test]
    fn tube_config_builders() {
        let config = TubeConfig::default()
            .with_radius(2.0)
            .with_segments(32)
            .uncapped();

        assert!((config.radius - 2.0).abs() < f64::EPSILON);
        assert_eq!(config.segments, 32);
        assert!(!config.capped);
    }

    #[test]
    fn tube_straight_line() {
        let points = straight_line();
        let config = TubeConfig::default().with_radius(1.0).with_segments(8);

        let result = tube_from_polyline(&points, &config);
        assert!(result.is_ok());

        let mesh = result.expect("mesh");
        // 2 rings * 8 segments + 2 center vertices for caps
        assert_eq!(mesh.vertices.len(), 2 * 8 + 2);
        // (2-1) rings * 8 segments * 2 triangles per quad + 2 caps * 8 triangles
        // 1 ring of quads (16 tris) + 2 caps (16 tris) = 32
        assert_eq!(mesh.faces.len(), 32);
    }

    #[test]
    fn tube_multi_segment() {
        let points = multi_segment();
        let config = TubeConfig::default().with_radius(0.5).with_segments(6);

        let result = tube_from_polyline(&points, &config);
        assert!(result.is_ok());

        let mesh = result.expect("mesh");
        // 3 rings * 6 segments + 2 center vertices for caps
        assert_eq!(mesh.vertices.len(), 3 * 6 + 2);
    }

    #[test]
    fn tube_uncapped() {
        let points = straight_line();
        let config = TubeConfig::default().with_segments(8).uncapped();

        let result = tube_from_polyline(&points, &config);
        assert!(result.is_ok());

        let mesh = result.expect("mesh");
        // 2 rings * 8 segments, no cap vertices
        assert_eq!(mesh.vertices.len(), 2 * 8);
        // Just side faces, no caps: 1 ring of quads (16 tris)
        assert_eq!(mesh.faces.len(), 16);
    }

    #[test]
    fn tube_too_few_points() {
        let points = vec![Point3::origin()];
        let result = tube_from_polyline(&points, &TubeConfig::default());
        assert!(result.is_err());
    }

    #[test]
    fn tube_invalid_radius() {
        let points = straight_line();
        let config = TubeConfig::default().with_radius(-1.0);
        let result = tube_from_polyline(&points, &config);
        assert!(result.is_err());
    }

    #[test]
    fn tube_too_few_segments() {
        let points = straight_line();
        let config = TubeConfig::default().with_segments(2);
        let result = tube_from_polyline(&points, &config);
        assert!(result.is_err());
    }

    #[test]
    fn tube_variable_radius_basic() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(0.0, 0.0, 10.0),
        ];
        let radii = vec![1.0, 2.0, 1.0];

        let result = tube_variable_radius(&points, &radii, 8, true);
        assert!(result.is_ok());

        let mesh = result.expect("mesh");
        assert!(!mesh.vertices.is_empty());
    }

    #[test]
    fn tube_variable_radius_mismatched() {
        let points = vec![Point3::origin(), Point3::new(0.0, 0.0, 1.0)];
        let radii = vec![1.0, 2.0, 3.0]; // Too many

        let result = tube_variable_radius(&points, &radii, 8, true);
        assert!(result.is_err());
    }

    #[test]
    fn tube_variable_radius_invalid() {
        let points = vec![Point3::origin(), Point3::new(0.0, 0.0, 1.0)];
        let radii = vec![1.0, -1.0]; // Negative radius

        let result = tube_variable_radius(&points, &radii, 8, true);
        assert!(result.is_err());
    }
}
