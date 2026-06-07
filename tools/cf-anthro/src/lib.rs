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
//! **Multi-contour slices — handled (v1).** `mesh-measure::cross_section` now
//! measures each loop separately and exposes them; this detector tracks the
//! **largest** contour at every height, so a stray noise island is ignored
//! rather than blended into the area profile. A genuinely ambiguous slice (two
//! comparable loops — a flexed joint) still bails ([`DetectError::
//! MultiContourSection`]) rather than guess.
//!
//! **Real-scan readiness (v2 — not handled yet).** Spelled out so they aren't
//! silent surprises:
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

use mesh_measure::{Contour, cross_section};
use mesh_types::{IndexedMesh, Point3, Vector3};

/// The limb (largest) contour of a horizontal slice at height `z` — ignores
/// small stray islands (scan noise) so the area/girth profile tracks the limb.
fn limb_contour(mesh: &IndexedMesh, z: f64) -> Option<Contour> {
    cross_section(mesh, Point3::new(0.0, 0.0, z), Vector3::z())
        .largest_contour()
        .cloned()
}

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
    /// The knee slice has two contours of comparable size — a genuinely
    /// ambiguous cut (a flexed joint, or two limb segments in one plane), not a
    /// small noise island (those are handled by taking the largest contour).
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
    // Limb (largest-contour) area along the axis — robust to stray noise
    // islands. Assumes the limb is the largest loop at every height (true for an
    // extended limb; a blob bigger than the limb at some z would mislead the
    // profile — a v2 real-scan concern, not on the validated synthetic legs).
    let raw: Vec<f64> = zs
        .iter()
        .map(|&z| limb_contour(mesh, z).map(|c| c.area).unwrap_or(0.0))
        .collect();
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

    // Knee = the area minimum between the two maxima — found robustly (the minimum
    // is flat, so a bare argmin is walked by profile ripple); see `robust_min_z`.
    if hi_i <= lo_i + 1 {
        return Err(DetectError::NoKneeMinimum);
    }
    let knee_z = robust_min_z(&zs, &area, lo_i, hi_i);

    let sec = cross_section(mesh, Point3::new(0.0, 0.0, knee_z), Vector3::z());
    // Take the limb (largest) contour; only bail if a SECOND contour rivals it
    // (a genuinely ambiguous slice — a flexed joint or two limb segments — not a
    // small noise island, which we ignore).
    let mut areas: Vec<f64> = sec.contours.iter().map(|c| c.area).collect();
    areas.sort_by(|a, b| b.total_cmp(a));
    if areas.is_empty() || areas[0] <= 0.0 {
        // Degenerate knee slice (no/zero-area loop) — don't measure garbage.
        return Err(DetectError::NoKneeMinimum);
    }
    if areas.len() >= 2 && areas[1] > 0.3 * areas[0] {
        return Err(DetectError::MultiContourSection(sec.contour_count));
    }
    let knee_c = sec.largest_contour().ok_or(DetectError::NoKneeMinimum)?;
    let (mn, mx) = knee_c.bounds;
    let (wx, wy) = (mx.x - mn.x, mx.y - mn.y);
    let epicondyle_width_m = wx.max(wy);
    let c = knee_c.centroid;
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

    let girth = |z: f64| limb_contour(mesh, z).map(|c| c.perimeter).unwrap_or(0.0);
    Ok(Landmarks {
        knee_z,
        knee_point: c,
        epicondyle_width_m,
        epicondyle_a,
        epicondyle_b,
        thigh_length_m: (hip_z - knee_z).abs(),
        shank_length_m: (knee_z - ankle_z).abs(),
        knee_girth_m: knee_c.perimeter,
        thigh_girth_m: girth(thigh_z),
        calf_girth_m: girth(calf_z),
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

/// Robust sub-sample z of the knee within the valley `[lo, hi]` (the two bracketing
/// girth maxima).
///
/// The knee minimum is FLAT, so a bare argmin is fragile — any periodic ripple in
/// the area profile (mesh-tessellation aliasing, scan noise) walks it across the
/// flat bottom (a regular mesh with `n_rings == N_SAMPLES` once shifted it ~27 mm).
/// Frequency separation fixes this: the true knee is a *broad* (low-frequency) bowl,
/// the ripple/spurious troughs are *high-frequency*. So we **locate** the bowl on a
/// heavy low-pass (which the ripple can't survive) and then **refine** the unbiased
/// sub-mm minimum locally on the lightly-smoothed profile. Locating and measuring
/// are split deliberately: a single wide filter is ripple-immune but biases the
/// asymmetric bowl's apparent min by ~8 mm, while a single narrow one is accurate
/// but ripple-fragile — neither alone is both.
fn robust_min_z(zs: &[f64], a: &[f64], lo: usize, hi: usize) -> f64 {
    // Two stages, so detection is BOTH robust and accurate:
    //  1. LOCATE the broad bowl on a heavy low-pass. The grid-aliasing ripple has a
    //     period of ~10-12 samples; a box window wider than that pushes it past the
    //     filter's first null, so it can't create a spurious minimum. (The low-pass
    //     biases the bowl's apparent min by ~8 mm via the asymmetric shoulders — so
    //     we only trust it to LOCATE, not to measure.)
    //  2. REFINE on the lightly-smoothed profile, but search only a small window
    //     around the located bowl, so ripple troughs elsewhere can't win. This
    //     recovers the unbiased sub-mm minimum.
    // Both widths tuned empirically (see tests/resolution_robustness.rs).
    const LOCATE_SMOOTH: usize = 12;
    const REFINE_HALF: usize = 3;

    let lp = smooth(a, LOCATE_SMOOTH);
    let k_broad = (lo + 1..hi)
        .min_by(|&i, &j| lp[i].total_cmp(&lp[j]))
        .unwrap_or(lo + 1);

    let r0 = k_broad.saturating_sub(REFINE_HALF).max(lo + 1);
    let r1 = (k_broad + REFINE_HALF + 1).min(hi);
    let k = (r0..r1)
        .min_by(|&i, &j| a[i].total_cmp(&a[j]))
        .unwrap_or(k_broad);
    parabolic_min_z(zs, a, k)
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

#[cfg(test)]
mod robust_min_tests {
    use super::robust_min_z;

    // zs at 1 cm spacing, so index i sits at z = 0.01·i.
    fn zs(n: usize) -> Vec<f64> {
        (0..n).map(|i| i as f64 * 0.01).collect()
    }

    #[test]
    fn finds_clean_bowl_vertex() {
        let z = zs(120);
        // A broad bowl with its vertex at index 60.5 (between samples).
        let a: Vec<f64> = (0..120)
            .map(|i| {
                let x = i as f64 - 60.5;
                1.0 + 0.0008 * x * x
            })
            .collect();
        let knee = robust_min_z(&z, &a, 10, 110);
        assert!((knee - 0.605).abs() < 0.005, "vertex off: {knee}");
    }

    #[test]
    fn rejects_spurious_ripple_trough() {
        let z = zs(120);
        // A PROMINENT bowl (vertex at 60) — as a real area minimum is, relative to
        // ripple — plus a NARROW (high-frequency) spurious trough at index 42 that
        // dips below the true bowl bottom. A bare argmin picks 42; the robust finder
        // must smooth the narrow trough away and return the broad bowl.
        let mut a: Vec<f64> = (0..120)
            .map(|i| {
                let x = i as f64 - 60.0;
                0.005 * x * x // bowl bottom = 0 at index 60
            })
            .collect();
        a[41] = 0.2;
        a[42] = -0.1; // below the bowl bottom (0.0)
        a[43] = 0.2;
        let bare_argmin = (10..110).min_by(|&i, &j| a[i].total_cmp(&a[j])).unwrap();
        assert_eq!(
            bare_argmin, 42,
            "test setup: the notch should beat the bowl"
        );

        // The robust finder must ignore the narrow notch and return the broad bowl.
        let knee = robust_min_z(&z, &a, 10, 110);
        assert!(
            (knee - 0.60).abs() < 0.02,
            "fooled by the spurious trough: {knee}"
        );
    }
}
