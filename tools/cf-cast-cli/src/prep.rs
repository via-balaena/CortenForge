//! `.prep.toml` centerline parsing.
//!
//! cf-scan-prep writes `.prep.toml` provenance with an optional
//! `[centerline]` block — see `tools/cf-scan-prep/src/main.rs::PrepToml`.
//! The bridge consumes the `[centerline].points_m` polyline (post-bake
//! world-frame meters) directly as the [`cf_cast::Ribbon`] input.
//!
//! Pure (no I/O); the caller reads the `.prep.toml` text and passes
//! it here.

use anyhow::Result;
use nalgebra::Point3;
use serde::Deserialize;

/// Subset of cf-scan-prep's `.prep.toml` schema that the bridge reads.
///
/// Only the `[centerline]` block matters for cast generation; every
/// other block (`[scan_prep]` / `[simplify]` / `[transform]` /
/// `[caps]` / `[centerline_trim]` / `[output]`) is provenance already
/// baked into the cleaned STL by the time cf-cast-cli runs. Marking
/// unknown fields permitted (no `deny_unknown_fields`) keeps us
/// forward-compatible with future cf-scan-prep schema additions —
/// and made the v1.0-completion `[reorient]/[recenter]` →
/// `[transform]` rename + `[clip]` removal (cf-scan-prep CSP.1 +
/// CSP.4d, 2026-05-15) downstream-safe.
#[derive(Debug, Clone, Deserialize)]
struct PrepTomlSubset {
    centerline: Option<CenterlineBlock>,
}

/// `.prep.toml`'s `[centerline]` block — polyline in post-bake
/// world-frame meters.
#[derive(Debug, Clone, Deserialize)]
struct CenterlineBlock {
    /// Polyline points as `[x, y, z]` triplets in meters. cf-scan-prep
    /// writes this from `cap.centerline_polyline` after applying the
    /// scan's Reorient + Recenter transforms (see
    /// `tools/cf-scan-prep/src/main.rs::build_prep_toml_string`).
    points_m: Vec<[f64; 3]>,
}

/// Parse `.prep.toml` text and return the centerline polyline.
///
/// Returns an empty `Vec` if the `[centerline]` block is absent OR
/// present-but-empty. The caller is responsible for validating
/// non-emptiness — that's a load-bearing precondition for
/// [`cf_cast::Ribbon::new`].
///
/// # Errors
///
/// Returns the `toml` parser error for malformed input.
pub fn parse_centerline_from_prep_toml(text: &str) -> Result<Vec<Point3<f64>>> {
    let subset: PrepTomlSubset = toml::from_str(text)?;
    let points = subset
        .centerline
        .map(|c| c.points_m)
        .unwrap_or_default()
        .into_iter()
        .map(|[x, y, z]| Point3::new(x, y, z))
        .collect();
    Ok(points)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn parses_centerline_block() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
tool_version = "0.1.0"
generated_at = "2026-05-13T20:00:00Z"
stl_units_at_load = "mm"

[centerline]
points_m = [
    [-0.04, 0.0, 0.0],
    [0.0, 0.0, 0.012],
    [0.04, 0.0, 0.0],
]
algorithm = "cross_section_centroids"
"#;
        let pts = parse_centerline_from_prep_toml(text).unwrap();
        assert_eq!(pts.len(), 3);
        assert!((pts[0].x - -0.04).abs() < 1e-12);
        assert!((pts[1].z - 0.012).abs() < 1e-12);
        assert!((pts[2].x - 0.04).abs() < 1e-12);
    }

    #[test]
    fn missing_centerline_block_returns_empty() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
tool_version = "0.1.0"
generated_at = "2026-05-13T20:00:00Z"
stl_units_at_load = "mm"
"#;
        let pts = parse_centerline_from_prep_toml(text).unwrap();
        assert!(pts.is_empty());
    }

    #[test]
    fn empty_points_block_returns_empty() {
        let text = r#"
[centerline]
points_m = []
algorithm = "cross_section_centroids"
"#;
        let pts = parse_centerline_from_prep_toml(text).unwrap();
        assert!(pts.is_empty());
    }

    #[test]
    fn extra_fields_tolerated_forward_compat() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
some_future_field = 42

[centerline]
points_m = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
algorithm = "cross_section_centroids"
another_future_field = "foo"
"#;
        let pts = parse_centerline_from_prep_toml(text).unwrap();
        assert_eq!(pts.len(), 2);
    }
}
