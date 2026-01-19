//! Parametric curve types for CAD and geometric modeling.
//!
//! This crate provides a comprehensive set of curve primitives commonly used in
//! computer-aided design (CAD), computer graphics, and geometric modeling:
//!
//! - [`Polyline`] - Piecewise linear curves (connected line segments)
//! - [`CubicBezier`] - Single cubic Bézier curve segment
//! - [`BezierSpline`] - Multiple Bézier segments with configurable continuity
//! - [`BSpline`] - B-spline curves with arbitrary degree
//! - [`Nurbs`] - Non-uniform rational B-splines (NURBS)
//! - [`Arc`] - Circular arc segments
//! - [`Circle`] - Full circles
//! - [`Helix`] - Helical curves
//!
//! # Core Traits
//!
//! All curve types implement the [`Curve`] trait, which provides:
//!
//! - **Evaluation**: Position, tangent, normal, binormal at parameter `t ∈ [0, 1]`
//! - **Arc length**: Total length and arc-length parameterization
//! - **Framing**: Rotation-minimizing frames (parallel transport)
//! - **Derivatives**: First and second derivatives for curvature analysis
//!
//! For curves with variable cross-section, the [`TubularCurve`] trait extends
//! [`Curve`] with radius queries.
//!
//! # Example
//!
//! ```
//! use curve_types::{Curve, CubicBezier};
//! use nalgebra::Point3;
//!
//! // Create a cubic Bézier curve
//! let curve = CubicBezier::new(
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 2.0, 0.0),
//!     Point3::new(3.0, 2.0, 0.0),
//!     Point3::new(4.0, 0.0, 0.0),
//! );
//!
//! // Evaluate at midpoint
//! let mid = curve.point_at(0.5);
//! let tangent = curve.tangent_at(0.5);
//!
//! // Get arc length
//! let length = curve.arc_length();
//! ```
//!
//! # Coordinate System
//!
//! This crate uses a **right-handed coordinate system** consistent with the
//! CortenForge ecosystem:
//!
//! - X: width (left/right)
//! - Y: depth (front/back)
//! - Z: height (up/down)
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Embedded systems
//! - Other game engines
//! - Python bindings
//!
//! # Feature Flags
//!
//! - `serde`: Enable serialization/deserialization for all types
//!
//! # Integration with Other Crates
//!
//! - **mesh-from-curves**: Generate tube meshes from any `Curve` implementation
//! - **route-types**: `ContinuousPath` can be converted to/from curves
//! - **truck** (optional): Import curves from STEP files

#![doc(html_root_url = "https://docs.rs/curve-types/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::int_plus_one,
    clippy::suspicious_operation_groupings,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::suboptimal_flops,
    clippy::while_float,
    clippy::missing_const_for_fn,
    clippy::cast_lossless,
    clippy::doc_markdown,
    clippy::redundant_closure_for_method_calls,
    clippy::module_name_repetitions,
    clippy::needless_pass_by_value,
    clippy::option_if_let_else,
    clippy::items_after_statements,
    clippy::uninlined_format_args,
    clippy::match_wildcard_for_single_variants,
    clippy::manual_midpoint,
    clippy::same_item_push,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::needless_range_loop,
    clippy::use_self,
    clippy::let_and_return,
    clippy::imprecise_flops,
    clippy::return_self_not_must_use,
    clippy::bool_to_int_with_if
)]

mod arc;
mod bezier;
mod bspline;
mod error;
mod frame;
mod helix;
mod nurbs;
mod ops;
mod polyline;
mod traits;

// Re-export core types
pub use arc::{Arc, Circle};
pub use bezier::{BezierSpline, Continuity, CubicBezier, QuadraticBezier};
pub use bspline::BSpline;
pub use error::CurveError;
pub use frame::{
    Frame, frenet_serret_frames, parallel_transport_frames, parallel_transport_frames_at,
};
pub use helix::Helix;
pub use nurbs::Nurbs;
pub use ops::{
    CurveOps, closest_point, extract, intersections, join, join_strict, reverse, split_at,
    subdivide,
};
pub use polyline::{Polyline, TubularPolyline};
pub use traits::{Curve, Curve2D, TubularCurve};

// Re-export nalgebra types for convenience
pub use nalgebra::{Point2, Point3, Vector2, Vector3};

/// Result type for curve operations.
pub type Result<T> = std::result::Result<T, CurveError>;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Test that all curve types implement the Curve trait consistently.
    #[test]
    fn test_curve_trait_consistency() {
        // Polyline
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);
        verify_curve_basics(&polyline);

        // Cubic Bézier
        let bezier = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        );
        verify_curve_basics(&bezier);

        // Arc
        let arc = Arc::from_center_radius_angles(
            Point3::new(0.0, 0.0, 0.0),
            1.0,
            0.0,
            std::f64::consts::PI,
            Vector3::z(),
        );
        verify_curve_basics(&arc);

        // Circle
        let circle = Circle::new(Point3::origin(), 1.0, Vector3::z());
        verify_curve_basics(&circle);
    }

    fn verify_curve_basics<C: Curve>(curve: &C) {
        // Endpoints
        let start = curve.point_at(0.0);
        let end = curve.point_at(1.0);
        assert!(start.coords.norm() < 1000.0); // Sanity check
        assert!(end.coords.norm() < 1000.0);

        // Tangent is unit vector
        let tangent = curve.tangent_at(0.5);
        assert_relative_eq!(tangent.norm(), 1.0, epsilon = 1e-10);

        // Arc length is positive
        let length = curve.arc_length();
        assert!(length > 0.0);

        // Sample points
        let samples = curve.sample_uniform(10);
        assert_eq!(samples.len(), 10);
    }

    /// Test arc-length parameterization.
    #[test]
    fn test_arc_length_parameterization() {
        let curve = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        );

        let total = curve.arc_length();

        // Walking by arc length should give roughly uniform spacing
        let t_half = curve.arc_to_t(total / 2.0);
        let p_half = curve.point_at(t_half);

        // For a straight-ish curve, midpoint by arc should be near geometric mid
        assert_relative_eq!(p_half.x, 1.5, epsilon = 0.1);
    }

    /// Test frame computation.
    #[test]
    fn test_frame_computation() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ]);

        let frames = parallel_transport_frames(&curve, 5);
        assert_eq!(frames.len(), 5);

        // All frames should be orthonormal
        for frame in &frames {
            assert_relative_eq!(frame.tangent.norm(), 1.0, epsilon = 1e-10);
            assert_relative_eq!(frame.normal.norm(), 1.0, epsilon = 1e-10);
            assert_relative_eq!(frame.binormal.norm(), 1.0, epsilon = 1e-10);
            assert_relative_eq!(frame.tangent.dot(&frame.normal), 0.0, epsilon = 1e-10);
        }
    }

    /// Test curve operations.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_curve_operations() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);

        // Reverse
        let reversed = reverse(&curve);
        assert_relative_eq!(
            reversed.point_at(0.0).coords,
            curve.point_at(1.0).coords,
            epsilon = 1e-10
        );

        // Split
        let (left, right) = split_at(&curve, 0.5).unwrap();
        assert_relative_eq!(
            left.point_at(1.0).coords,
            right.point_at(0.0).coords,
            epsilon = 1e-10
        );
    }
}
