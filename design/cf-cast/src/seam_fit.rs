//! Apex-anchored best-fit FLAT seam plane for organic/curved parts.
//!
//! Item (A) of `docs/CF_CAST_ORGANIC_PARTS_RECON.md` §4.1 (C-A2 refined).
//!
//! The mold seam stays a single FLAT plane (so the two cup halves print
//! mating-face-down and seal), but on a part whose dome LEANS off the shaft
//! axis the naive flat seam ([`crate::ribbon::Ribbon::with_planar_seam`], which
//! flattens the ribbon binormal to HORIZONTAL and positions it at the centerline
//! midpoint) can't follow the lean: it bisects the shaft but skims the dome,
//! leaving one cup-half a thin sliver over the dome (workshop cf-view 2026-05-29).
//!
//! [`best_fit_planar_seam`] instead fits the plane to the actual body:
//! 1. **Anchor through the cap-centroid → apex axis.** The apex is the body
//!    vertex farthest from the cap plane (the dome tip). A plane that CONTAINS
//!    this base→apex line passes through the dome tip by construction and tilts
//!    with the part's lean — so it bisects every cross-section along the length,
//!    not just the middle.
//! 2. **Rotate about that axis to the most-even split.** The dome/glans is
//!    asymmetric, so the azimuth matters; the sweep minimises the mean
//!    per-height-band area imbalance (the direct anti-sliver objective), not
//!    just the overall split.
//!
//! The returned `(point, normal)` feeds [`crate::ribbon::Ribbon::with_planar_seam_at`];
//! the cup-wall half-space cut + flange both read it via
//! [`crate::ribbon::Ribbon::seam_plane_reference`], so one call retargets both.
//!
//! Validated by the S0 seam-fit spike on the real `3quartachub` scan (recon
//! §4.1): the fitted plane is symmetric in every height band (no sliver) where
//! the shipped binormal seam skewed 2:1 at the dome.

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Unit, Vector3};

/// Number of azimuth samples over the half-circle `[0, π)` (a normal and its
/// negation describe the same plane, so the half-circle covers every distinct
/// orientation). 180 → 1° resolution.
const AZIMUTH_STEPS: u32 = 180;

/// Number of height bands along the cap→apex axis used to score the split. The
/// objective is the mean per-band area imbalance, so each band (including the
/// small dome bands) is protected against slivering.
const HEIGHT_BANDS: usize = 12;

/// `HEIGHT_BANDS` as `f64` (separate literal so the band index needs no
/// `usize → f64` cast).
const HEIGHT_BANDS_F: f64 = 12.0;

/// Fraction of total area below which a band is ignored when scoring (drops the
/// near-empty bands at the very tip / very base so MC-noise slivers there don't
/// dominate the objective).
const MIN_BAND_AREA_FRACTION: f64 = 0.01;

/// One triangle reduced to what the azimuth sweep needs: its centroid
/// projected onto the search frame `(e1, e2)`, its area, and its height band.
/// `side = p1·cosθ + p2·sinθ ≥ 0` selects the `+normal` half cheaply.
struct BandTri {
    p1: f64,
    p2: f64,
    area: f64,
    band: usize,
}

/// Fit a flat seam plane to `mesh`, anchored through the cap-centroid → apex
/// axis and rotated to the most-even (least-slivering) body split.
///
/// `cap_centroid` + `cap_normal` come from the scan's `.prep.toml [caps]`
/// (the cap-plane fit); `cap_normal` points OUTWARD (away from the body), so the
/// body extends along `-cap_normal`. Returns `(anchor, normal)` where `anchor`
/// is the cap centroid (the apex lies on the plane by construction) and `normal`
/// is the unit seam-plane normal (the mold opens along `±normal`).
///
/// Returns `None` for a degenerate input (empty mesh, apex coincident with the
/// cap centroid, or zero total area) — the caller falls back to the binormal
/// [`crate::ribbon::Ribbon::with_planar_seam`].
#[must_use]
pub fn best_fit_planar_seam(
    mesh: &IndexedMesh,
    cap_centroid: Point3<f64>,
    cap_normal: Vector3<f64>,
) -> Option<(Point3<f64>, Unit<Vector3<f64>>)> {
    let up = Unit::try_new(-cap_normal, 1.0e-9)?;

    // Apex = vertex farthest from the cap plane into the body.
    let apex = mesh.vertices.iter().copied().max_by(|lhs, rhs| {
        let hl = (lhs - cap_centroid).dot(&up);
        let hr = (rhs - cap_centroid).dot(&up);
        hl.partial_cmp(&hr).unwrap_or(std::cmp::Ordering::Equal)
    })?;
    let axis = Unit::try_new(apex - cap_centroid, 1.0e-9)?;
    let h_apex = (apex - cap_centroid).dot(&axis);
    if h_apex <= 0.0 {
        return None;
    }

    // Orthonormal frame spanning the seam-normal search circle (⟂ axis).
    let seed = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let e1 = Unit::new_normalize(seed - axis.scale(axis.dot(&seed)));
    let e2 = Unit::new_normalize(axis.cross(&e1));

    let mut tris: Vec<BandTri> = Vec::with_capacity(mesh.faces.len());
    let mut band_total = [0.0_f64; HEIGHT_BANDS];
    for face in &mesh.faces {
        let (Some(v0), Some(v1), Some(v2)) = (
            mesh.vertices.get(face[0] as usize),
            mesh.vertices.get(face[1] as usize),
            mesh.vertices.get(face[2] as usize),
        ) else {
            continue;
        };
        let centroid = Point3::from((v0.coords + v1.coords + v2.coords) / 3.0);
        let area = 0.5 * (v1 - v0).cross(&(v2 - v0)).norm();
        if area <= 0.0 {
            continue;
        }
        let rel = centroid - cap_centroid;
        // Band the centroid by its fractional height along the cap→apex axis.
        let frac = (rel.dot(&axis) / h_apex).clamp(0.0, 0.999_999);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let band = (frac * HEIGHT_BANDS_F) as usize;
        band_total[band] += area;
        tris.push(BandTri {
            p1: rel.dot(&e1),
            p2: rel.dot(&e2),
            area,
            band,
        });
    }
    let total: f64 = band_total.iter().sum();
    if total <= 0.0 {
        return None;
    }
    let min_band_area = total * MIN_BAND_AREA_FRACTION;

    // Sweep the azimuth; pick the normal minimising the mean per-band imbalance.
    let mut best_err = f64::MAX;
    let mut best_normal = e1.into_inner();
    for step in 0..AZIMUTH_STEPS {
        let theta = std::f64::consts::PI * f64::from(step) / f64::from(AZIMUTH_STEPS);
        let (cos, sin) = (theta.cos(), theta.sin());
        let mut band_plus = [0.0_f64; HEIGHT_BANDS];
        for tri in &tris {
            if tri.p1.mul_add(cos, tri.p2 * sin) >= 0.0 {
                band_plus[tri.band] += tri.area;
            }
        }
        let mut err_sum = 0.0;
        let mut counted = 0u32;
        for band in 0..HEIGHT_BANDS {
            if band_total[band] >= min_band_area {
                err_sum += (band_plus[band] / band_total[band] - 0.5).abs();
                counted += 1;
            }
        }
        if counted == 0 {
            continue;
        }
        let err = err_sum / f64::from(counted);
        if err < best_err {
            best_err = err;
            best_normal = e1.into_inner() * cos + e2.into_inner() * sin;
        }
    }

    Some((cap_centroid, Unit::new_normalize(best_normal)))
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::similar_names
    )]

    use super::*;

    /// Build a leaning circular tube (side facets only — `best_fit` needs just
    /// vertices + triangle centroids/areas). The axis runs from `(0,0,0)` to
    /// `apex`; `apex` leans in +Y so a horizontal-normal plane can't follow it.
    fn leaning_tube(apex: Point3<f64>, radius: f64, rings: u32, segs: u32) -> IndexedMesh {
        let axis = Unit::new_normalize(apex.coords);
        let seed = Vector3::x();
        let radial_u = Unit::new_normalize(seed - axis.scale(axis.dot(&seed)));
        let radial_v = Unit::new_normalize(axis.cross(&radial_u));
        let mut vertices = Vec::new();
        for ring in 0..=rings {
            let frac = f64::from(ring) / f64::from(rings);
            let center = apex.coords * frac;
            for seg in 0..segs {
                let angle = std::f64::consts::TAU * f64::from(seg) / f64::from(segs);
                let offset = (radial_u.into_inner() * angle.cos()
                    + radial_v.into_inner() * angle.sin())
                    * radius;
                vertices.push(Point3::from(center + offset));
            }
        }
        let mut faces = Vec::new();
        for ring in 0..rings {
            for seg in 0..segs {
                let seg2 = (seg + 1) % segs;
                let lo0 = ring * segs + seg;
                let lo1 = ring * segs + seg2;
                let hi0 = (ring + 1) * segs + seg;
                let hi1 = (ring + 1) * segs + seg2;
                faces.push([lo0, lo1, hi0]);
                faces.push([lo1, hi1, hi0]);
            }
        }
        IndexedMesh { vertices, faces }
    }

    fn area_split_fraction(
        mesh: &IndexedMesh,
        anchor: Point3<f64>,
        normal: Unit<Vector3<f64>>,
    ) -> f64 {
        let (mut plus, mut total) = (0.0_f64, 0.0_f64);
        for face in &mesh.faces {
            let v0 = mesh.vertices[face[0] as usize];
            let v1 = mesh.vertices[face[1] as usize];
            let v2 = mesh.vertices[face[2] as usize];
            let centroid = Point3::from((v0.coords + v1.coords + v2.coords) / 3.0);
            let area = 0.5 * (v1 - v0).cross(&(v2 - v0)).norm();
            total += area;
            if (centroid - anchor).dot(&normal) >= 0.0 {
                plus += area;
            }
        }
        plus / total
    }

    #[test]
    fn fitted_plane_contains_the_cap_to_apex_axis() {
        // Tube leaning in +Y; cap at the origin pointing down (-Z).
        let apex = Point3::new(0.0, 0.030, 0.130);
        let mesh = leaning_tube(apex, 0.012, 24, 48);
        let cap_c = Point3::new(0.0, 0.0, 0.0);
        let cap_n = Vector3::new(0.0, 0.0, -1.0);
        let (anchor, normal) = best_fit_planar_seam(&mesh, cap_c, cap_n).expect("fit");

        // The plane must CONTAIN the cap→apex axis (so it passes through the
        // dome tip) — i.e. the normal is perpendicular to that axis.
        let axis = (apex - cap_c).normalize();
        assert!(
            normal.into_inner().dot(&axis).abs() < 1.0e-6,
            "seam normal must be ⟂ the cap→apex axis (plane contains the apex); \
             got n·axis = {}",
            normal.into_inner().dot(&axis),
        );
        // Anchor is the cap centroid; the apex lies on the plane.
        assert!((anchor - cap_c).norm() < 1.0e-9);
        assert!(
            (apex - anchor).dot(&normal).abs() < 1.0e-9,
            "apex must lie on the fitted plane",
        );
    }

    #[test]
    fn fitted_plane_splits_a_round_tube_evenly() {
        let apex = Point3::new(0.0, 0.030, 0.130);
        let mesh = leaning_tube(apex, 0.012, 24, 48);
        let cap_c = Point3::new(0.0, 0.0, 0.0);
        let cap_n = Vector3::new(0.0, 0.0, -1.0);
        let (anchor, normal) = best_fit_planar_seam(&mesh, cap_c, cap_n).expect("fit");

        let frac = area_split_fraction(&mesh, anchor, normal);
        assert!(
            (frac - 0.5).abs() < 0.05,
            "round tube should split ~50/50; got {:.1}% on the +normal side",
            frac * 100.0,
        );
    }

    #[test]
    fn degenerate_inputs_return_none() {
        let empty = IndexedMesh {
            vertices: Vec::new(),
            faces: Vec::new(),
        };
        assert!(
            best_fit_planar_seam(&empty, Point3::origin(), Vector3::new(0.0, 0.0, -1.0)).is_none()
        );
    }
}
