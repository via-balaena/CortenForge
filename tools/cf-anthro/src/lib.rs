//! cf-anthro — anatomical landmark detection on a cleaned body-part scan.
//!
//! S2 of the musculoskeletal-builder arc (scan → bones/tendons, Mission #4).
//! Heuristics-first (no ML): the knee reads as a cross-sectional-area **minimum**
//! along the limb axis, flanked by the thigh and calf girth **maxima**. From that
//! we derive the joint-line height, epicondyle (medio-lateral) width, segment
//! lengths, and girths. See `docs/msk_builder/03_phases/g1_knee_kinematics`.
//!
//! Input convention: the scan's long axis is **+z** (real scans are PCA-aligned
//! first via `cf-scan-prep-core::auto_pca_in_place`; the synthetic legs in
//! [`synthetic`] are built that way). Detection is orientation-agnostic about
//! which end is the hip — it labels the thicker end as proximal.
//!
//! **What's validated (and what isn't).** This is the v1 detector, validated on
//! synthetic legs with known ground truth — which proves the *numerics* (the
//! area-minimum search, parabola refine, robustness to ~1 mm noise) but NOT the
//! anatomical premise that "cross-sectional-area minimum = knee joint line"
//! (the generator places the knee *at* the area minimum, so that part is
//! circular). That premise is validated only when real leg scans + an
//! independent landmark ground truth arrive.
//!
//! **Real-scan readiness (v2 — not handled yet).** Spelled out so they aren't
//! silent surprises:
//! - **Multi-contour slices.** `mesh-measure`'s `cross_section`/`area_at_height`
//!   blend multiple contours into one value, so a slice that cuts the limb in
//!   2+ loops (bent/flexed knee, fibular head, an un-cleaned stray island)
//!   corrupts the area profile. We *tripwire* on it ([`DetectError::
//!   MultiContourSection`]) at the knee section, but the profile sweep itself
//!   isn't guarded — the real fix is per-contour measures in mesh-measure.
//! - **PCA / pose.** Detection assumes the limb axis is +z; a tilted or curved
//!   (varus/valgus/flexed) limb breaks horizontal-slice area. v2 must align via
//!   PCA and slice perpendicular to the centerline.
//! - **Epicondyle axis.** Width is the wider of the section's *world* x/y bbox;
//!   correct only when the medio-lateral axis is x/y. v2 should use the section's
//!   principal (caliper) axis.
//! - **Maxima robustness.** The thigh/calf maxima are the two largest by area; a
//!   patella/vastus bulge could outrank the calf on a real scan. v2 should select
//!   by prominence + an anatomical prior (knee in the middle third).

pub mod markers;
pub mod synthetic;

use mesh_measure::{area_at_height, circumference_at_height, cross_section};
use mesh_types::{IndexedMesh, Point3, Vector3};

/// Detected anatomical landmarks (all lengths in meters, in the scan frame).
#[derive(Debug, Clone)]
pub struct Landmarks {
    /// Axial height of the knee joint line (the area minimum).
    pub knee_z: f64,
    /// Centroid of the knee cross-section.
    pub knee_point: Point3<f64>,
    /// Medio-lateral width at the knee (wider horizontal extent of the section).
    pub epicondyle_width_m: f64,
    /// The two medio-lateral extreme points of the knee cross-section.
    pub epicondyle_a: Point3<f64>,
    pub epicondyle_b: Point3<f64>,
    /// Knee → proximal (hip) end axial length.
    pub thigh_length_m: f64,
    /// Distal (ankle) end → knee axial length.
    pub shank_length_m: f64,
    /// Circumference at the knee / thigh-max / calf-max.
    pub knee_girth_m: f64,
    pub thigh_girth_m: f64,
    pub calf_girth_m: f64,
    pub thigh_z: f64,
    pub calf_z: f64,
}

/// Number of axial samples for the area profile.
const N_SAMPLES: usize = 240;

/// Why landmark detection couldn't produce a result (degrade gracefully rather
/// than panic — a scan pipeline must not crash on a bad input).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DetectError {
    /// Mesh has no vertices.
    EmptyMesh,
    /// Fewer than two girth maxima (thigh + calf) in the area profile.
    TooFewGirthMaxima(usize),
    /// No area minimum between the two maxima.
    NoKneeMinimum,
    /// A horizontal slice cut the limb in more than one loop. `area_at_height`
    /// silently blends multiple contours (a mesh-measure limitation), so the
    /// area profile would be corrupted — bail rather than trust it. This never
    /// happens on a clean single-limb scan; it is the headline v2 real-scan item
    /// (see the crate-level "Real-scan readiness" notes).
    MultiContourSection(usize),
}

impl std::fmt::Display for DetectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "landmark detection failed: {self:?}")
    }
}
impl std::error::Error for DetectError {}

/// Detect knee + derived landmarks on a z-aligned limb scan.
pub fn detect_landmarks(mesh: &IndexedMesh) -> Result<Landmarks, DetectError> {
    if mesh.vertices.is_empty() {
        return Err(DetectError::EmptyMesh);
    }
    let (zmin, zmax) = z_range(mesh);
    let len = zmax - zmin;
    // Trim the ends: the flat end-caps degenerate the cross-section area.
    let (lo, hi) = (zmin + 0.04 * len, zmax - 0.04 * len);

    let zs: Vec<f64> = (0..N_SAMPLES)
        .map(|i| lo + (hi - lo) * (i as f64) / ((N_SAMPLES - 1) as f64))
        .collect();
    let raw: Vec<f64> = zs.iter().map(|&z| area_at_height(mesh, z)).collect();
    // Light smoothing (window ~ a few mm) for noise robustness without smearing
    // the knee feature. The thigh/calf maxima are broad, so this is plenty.
    let area = smooth(&raw, 2);

    // The two most prominent interior maxima are the thigh and calf bulges.
    let mut maxima = local_maxima(&area, 6);
    maxima.sort_by(|&a, &b| area[b].total_cmp(&area[a]));
    if maxima.len() < 2 {
        return Err(DetectError::TooFewGirthMaxima(maxima.len()));
    }
    let (m1, m2) = (maxima[0], maxima[1]); // m1 = larger (thigh side)
    let (lo_i, hi_i) = (m1.min(m2), m1.max(m2));

    // Knee = the area minimum strictly between the two maxima, parabola-refined
    // to sub-sample precision (the knee is a smooth minimum).
    let knee_i = (lo_i + 1..hi_i)
        .min_by(|&a, &b| area[a].total_cmp(&area[b]))
        .ok_or(DetectError::NoKneeMinimum)?;
    let knee_z = parabolic_min_z(&zs, &area, knee_i);

    let sec = cross_section(mesh, Point3::new(0.0, 0.0, knee_z), Vector3::z());
    if sec.contour_count > 1 {
        return Err(DetectError::MultiContourSection(sec.contour_count));
    }
    let (mn, mx) = sec.bounds;
    let (wx, wy) = (mx.x - mn.x, mx.y - mn.y);
    let epicondyle_width_m = wx.max(wy);
    let c = sec.centroid;
    // Endpoints along whichever horizontal axis is wider (the M-L axis).
    let (epicondyle_a, epicondyle_b) = if wx >= wy {
        (
            Point3::new(mn.x, c.y, knee_z),
            Point3::new(mx.x, c.y, knee_z),
        )
    } else {
        (
            Point3::new(c.x, mn.y, knee_z),
            Point3::new(c.x, mx.y, knee_z),
        )
    };

    // The thicker maximum is the thigh (proximal); its end is the hip.
    let thigh_z = zs[m1];
    let calf_z = zs[m2];
    let hip_z = if (zmax - thigh_z).abs() < (thigh_z - zmin).abs() {
        zmax
    } else {
        zmin
    };
    let ankle_z = if hip_z == zmax { zmin } else { zmax };

    Ok(Landmarks {
        knee_z,
        knee_point: sec.centroid,
        epicondyle_width_m,
        epicondyle_a,
        epicondyle_b,
        thigh_length_m: (hip_z - knee_z).abs(),
        shank_length_m: (knee_z - ankle_z).abs(),
        knee_girth_m: circumference_at_height(mesh, knee_z),
        thigh_girth_m: circumference_at_height(mesh, thigh_z),
        calf_girth_m: circumference_at_height(mesh, calf_z),
        thigh_z,
        calf_z,
    })
}

impl Landmarks {
    /// Serialize as a `landmarks.toml` provenance string.
    pub fn to_toml_string(&self) -> String {
        format!(
            "[landmarks]\n\
             knee_z_m = {:.6}\n\
             knee_point_m = [{:.6}, {:.6}, {:.6}]\n\
             epicondyle_width_m = {:.6}\n\
             thigh_length_m = {:.6}\n\
             shank_length_m = {:.6}\n\
             knee_girth_m = {:.6}\n\
             thigh_girth_m = {:.6}\n\
             calf_girth_m = {:.6}\n",
            self.knee_z,
            self.knee_point.x,
            self.knee_point.y,
            self.knee_point.z,
            self.epicondyle_width_m,
            self.thigh_length_m,
            self.shank_length_m,
            self.knee_girth_m,
            self.thigh_girth_m,
            self.calf_girth_m,
        )
    }
}

fn z_range(mesh: &IndexedMesh) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for v in &mesh.vertices {
        lo = lo.min(v.z);
        hi = hi.max(v.z);
    }
    (lo, hi)
}

/// Symmetric moving average (window radius `w`), endpoints clamped.
fn smooth(a: &[f64], w: usize) -> Vec<f64> {
    (0..a.len())
        .map(|i| {
            let lo = i.saturating_sub(w);
            let hi = (i + w + 1).min(a.len());
            a[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
        })
        .collect()
}

/// Sub-sample z of a local minimum at index `i`, by fitting a parabola through
/// the three bracketing samples (uniform spacing assumed).
fn parabolic_min_z(zs: &[f64], a: &[f64], i: usize) -> f64 {
    if i == 0 || i + 1 >= a.len() {
        return zs[i];
    }
    let (y0, y1, y2) = (a[i - 1], a[i], a[i + 1]);
    let denom = y0 - 2.0 * y1 + y2;
    if denom.abs() < 1e-15 {
        return zs[i];
    }
    // Clamp to the bracket: if the argmin sat at a sub-range boundary the triple
    // may not bracket the min and the vertex could fall outside ±½ sample.
    let delta = (0.5 * (y0 - y2) / denom).clamp(-0.5, 0.5);
    zs[i] + delta * (zs[i + 1] - zs[i])
}

/// Indices that are a strict maximum over the window `±w` (interior only).
fn local_maxima(a: &[f64], w: usize) -> Vec<usize> {
    (w..a.len().saturating_sub(w))
        .filter(|&i| (i - w..=i + w).all(|j| j == i || a[j] < a[i] || (a[j] == a[i] && j < i)))
        .collect()
}
