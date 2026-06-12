//! Parametric interior-canal features composed **on top of** the
//! existing scan-derived layer-0 plug.
//!
//! # Architecture (Canal Interior arc, Candidate A)
//!
//! The workshop user owns baseline canal tightness through the existing
//! layer/inset machinery (`cavity_inset_m`). This module does **not**
//! set girth. Instead it adds *stimulation + grip features* — annular
//! grip rings, a frenulum-biased D-section pinch, frenulum-gated texture
//! ribs, and a terminal suction bulb — as a spatially-varying offset of
//! the base plug's own signed-distance field:
//!
//! ```text
//! new_d(p) = base_plug.eval(p) + D(s, θ)
//! ```
//!
//! where `D > 0` pushes the plug surface **inward** (a tighter channel /
//! a protruding grip ridge in the cured silicone) and `D < 0` bulges it
//! **outward** (the terminal suction relief pocket). Because `D` rides
//! on the base plug's distance field, every feature follows the
//! arbitrary scanned surface — no assumption that the cavity is a tube
//! and no scan-girth extraction. This is the "additive features only —
//! rings make it tighter, never looser" model the workshop locked
//! 2026-05-28.
//!
//! The features are parameterized in a **near-straight frame** built
//! from the centerline endpoints: `s` is the normalized position along
//! the demolding axis (0 at the mouth end, 1 at the tip end) and `θ` is
//! the azimuth measured from `frenulum_dir`. iter-1 assumes a
//! near-straight centerline; a curved-centerline straightened-frame
//! mapping is a later upgrade.
//!
//! Mirrors the custom-SDF pattern of [`crate::flange`]'s `FlangeSdf`:
//! implement [`cf_design::Sdf`], compose distance terms, and bridge via
//! [`cf_design::Solid::from_sdf`].

use cf_design::{Aabb, Sdf, Solid};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::error::{CastError, CastTarget};

/// One annular grip ring — a smooth inward pinch of the channel.
///
/// Centered at axial fraction `center_frac`, `depth_m` deep, spanning
/// ±`half_width_frac` of the canal length. Produces a protruding grip
/// ridge in the cured silicone.
#[derive(Debug, Clone, Copy)]
pub struct RingSpec {
    /// Axial position as a fraction of canal length (0 = mouth, 1 = tip).
    pub center_frac: f64,
    /// Inward pinch depth in meters (peak of the ring).
    pub depth_m: f64,
    /// Half-width of the ring's axial support, as a fraction of length.
    pub half_width_frac: f64,
}

/// Resolved canal-feature parameters. Defaults come from
/// [`CanalSpec::iter1`]; the cf-cast-cli `[canal]` config overrides
/// individual fields.
#[derive(Debug, Clone)]
pub struct CanalSpec {
    /// Annular grip rings.
    pub rings: Vec<RingSpec>,
    /// Frenulum-gated texture rib amplitude (meters). 0 disables texture.
    pub texture_amp_m: f64,
    /// Texture rib pitch along the axis (meters).
    pub texture_pitch_m: f64,
    /// Axial fraction range `(start, end)` over which texture applies.
    pub texture_zone: (f64, f64),
    /// Frenulum-side D-section pinch depth (meters). The frenulum wall is
    /// pinched inward by this much over `dsection_zone`, fading to the
    /// dorsal side via `max(0, cos θ)`. 0 disables asymmetry.
    pub dsection_depth_m: f64,
    /// Axial fraction range `(start, end)` over which the D-section
    /// pinch applies.
    pub dsection_zone: (f64, f64),
    /// Terminal suction-bulb OUTWARD bulge depth (meters). Bulges the
    /// plug outward over `[suction_start_frac, 1.0]` → an open relief
    /// pocket in the silicone for pneumatic pull. 0 disables the bulb.
    /// **This is the one feature that loosens the channel and thins the
    /// wall toward the housing** — gated by the canal-⊂-body wall check.
    pub suction_bulge_m: f64,
    /// Axial fraction at which the suction bulb begins (runs to 1.0).
    pub suction_start_frac: f64,
    /// Frenulum direction in the cast world frame (the asymmetry axis).
    /// Projected perpendicular to the canal axis and normalized at frame
    /// construction. `θ = 0` points along this.
    pub frenulum_dir: Vector3<f64>,
    /// Marching-cubes cell size for the layer-0 plug only (meters). The
    /// canal features (ribs ~1.5 mm) need finer cells than the 3 mm
    /// default; cups stay coarse. See the S0 probe finding.
    pub plug_mesh_cell_size_m: f64,
}

impl CanalSpec {
    /// iter-1 default canal: a corona-catch entry ring + two stimulation
    /// rings, a frenulum D-section pinch + frenulum-gated ribs over the
    /// mid-canal, and a shallow terminal suction bulb. Conservative
    /// depths — the physical pull-out test on the first real cast bounds
    /// how aggressive these can grow.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            rings: vec![
                // Corona-catch entry ring near the mouth.
                RingSpec {
                    center_frac: 0.10,
                    depth_m: 0.003,
                    half_width_frac: 0.04,
                },
                // Two secondary stimulation rings in the mid-canal.
                RingSpec {
                    center_frac: 0.40,
                    depth_m: 0.002,
                    half_width_frac: 0.04,
                },
                RingSpec {
                    center_frac: 0.55,
                    depth_m: 0.002,
                    half_width_frac: 0.04,
                },
            ],
            texture_amp_m: 0.0015,
            texture_pitch_m: 0.008,
            texture_zone: (0.20, 0.65),
            dsection_depth_m: 0.0015,
            dsection_zone: (0.18, 0.70),
            suction_bulge_m: 0.003,
            suction_start_frac: 0.90,
            frenulum_dir: Vector3::new(0.0, 1.0, 0.0),
            plug_mesh_cell_size_m: 0.0005,
        }
    }

    /// Largest inward depth the feature field can reach at any point —
    /// the sum of the deepest ring, the texture amplitude, and the
    /// D-section pinch (all of which can stack on the frenulum side).
    /// Used to size the MC bounds inward margin.
    #[must_use]
    pub fn max_inward_depth_m(&self) -> f64 {
        let max_ring = self.rings.iter().map(|r| r.depth_m).fold(0.0_f64, f64::max);
        max_ring + self.texture_amp_m + self.dsection_depth_m
    }
}

/// Near-straight canal frame: maps a world point to `(axial_fraction,
/// axial_distance, cos θ)`. Built from the centerline endpoints and the
/// plug's projected axial extent.
#[derive(Debug, Clone)]
pub struct CanalFrame {
    origin: Point3<f64>,
    axis: Vector3<f64>,
    frenulum_u: Vector3<f64>,
    s_min: f64,
    s_max: f64,
}

impl CanalFrame {
    /// Build the frame from the centerline polyline + frenulum direction
    /// + the plug bounds (to fix the axial extent `[s_min, s_max]`).
    ///
    /// `frac = 0` is the **mouth** (insertion opening) and `frac = 1`
    /// the deep end — the zone model (entry ring near 0, suction bulb
    /// near 1) depends on this. For this device the mouth is the capped
    /// (cut-base) end, so `mouth_anchor` (the cap-plane centroid) orients
    /// the frame: the centerline endpoint NEAREST the anchor becomes the
    /// origin (`frac = 0`). Without an anchor the first centerline point
    /// is the mouth. **This matters** — cf-scan-prep's centerline can run
    /// either direction (on the iter-1 clone it runs glans → floor), so
    /// keying off the cap is what keeps the suction bulb at the deep
    /// glans end instead of piling onto the flat floor cap.
    ///
    /// `frenulum_dir` is projected perpendicular to the axis (fallback
    /// `+X` if parallel).
    #[must_use]
    pub fn new(
        centerline: &[Point3<f64>],
        frenulum_dir: Vector3<f64>,
        plug_bounds: Aabb,
        mouth_anchor: Option<Point3<f64>>,
    ) -> Self {
        let first = centerline.first().copied().unwrap_or_else(Point3::origin);
        let last = centerline.last().copied().unwrap_or(first);
        // Orient frac 0 = mouth. With a cap-centroid anchor, the mouth is
        // whichever centerline endpoint sits nearer the cap.
        let (origin, deep) = match mouth_anchor {
            Some(m) if (last - m).norm() < (first - m).norm() => (last, first),
            _ => (first, last),
        };
        let raw_axis = deep - origin;
        let axis = if raw_axis.norm() > 1.0e-9 {
            raw_axis.normalize()
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };

        // Frenulum direction perpendicular to the axis.
        let projected = frenulum_dir - axis * frenulum_dir.dot(&axis);
        let frenulum_u = if projected.norm() > 1.0e-9 {
            projected.normalize()
        } else {
            // frenulum_dir parallel to axis — pick any perpendicular.
            let trial = if axis.x.abs() < 0.9 {
                Vector3::new(1.0, 0.0, 0.0)
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            };
            (trial - axis * trial.dot(&axis)).normalize()
        };

        // Axial extent: project all 8 plug-bounds corners onto the axis.
        let (lo, hi) = (plug_bounds.min, plug_bounds.max);
        let mut s_min = f64::MAX;
        let mut s_max = f64::MIN;
        for &x in &[lo.x, hi.x] {
            for &y in &[lo.y, hi.y] {
                for &z in &[lo.z, hi.z] {
                    let s = (Point3::new(x, y, z) - origin).dot(&axis);
                    s_min = s_min.min(s);
                    s_max = s_max.max(s);
                }
            }
        }

        Self {
            origin,
            axis,
            frenulum_u,
            s_min,
            s_max,
        }
    }

    /// Project `p` to `(axial_fraction ∈ [0,1] (clamped), axial_distance
    /// (meters), cos θ ∈ [-1, 1])`. `cos θ = +1` on the frenulum side.
    #[must_use]
    pub fn project(&self, p: Point3<f64>) -> (f64, f64, f64) {
        let rel = p - self.origin;
        let s = rel.dot(&self.axis);
        let span = (self.s_max - self.s_min).max(1.0e-9);
        let frac = ((s - self.s_min) / span).clamp(0.0, 1.0);
        let radial = rel - self.axis * s;
        let r = radial.norm();
        let cos_theta = if r > 1.0e-9 {
            (radial.dot(&self.frenulum_u) / r).clamp(-1.0, 1.0)
        } else {
            0.0
        };
        (frac, s, cos_theta)
    }
}

/// Smooth compact pulse: raised cosine, 1.0 at `center`, fading to 0 at
/// `|x - center| = half_width`, exactly 0 beyond. `half_width <= 0`
/// returns 0.
fn raised_cosine_pulse(x: f64, center: f64, half_width: f64) -> f64 {
    if half_width <= 0.0 {
        return 0.0;
    }
    let d = (x - center).abs();
    if d >= half_width {
        0.0
    } else {
        0.5 * (1.0 + (std::f64::consts::PI * d / half_width).cos())
    }
}

/// Smooth window over `[a, b]` with raised-cosine tapers at both ends,
/// so a zone-gated feature blends to 0 at the zone boundaries instead of
/// leaving a hard ridge. Returns 0 outside `[a, b]`, 1 in the interior,
/// taper width = 20% of the span at each end.
fn smooth_window(x: f64, a: f64, b: f64) -> f64 {
    if b <= a || x <= a || x >= b {
        return 0.0;
    }
    let span = b - a;
    let taper = span * 0.2;
    let from_a = x - a;
    let from_b = b - x;
    let edge = from_a.min(from_b);
    if edge >= taper {
        1.0
    } else {
        0.5 * (1.0 - (std::f64::consts::PI * edge / taper).cos())
    }
}

/// The canal feature field `D(s, θ)` (meters): positive = inward
/// (tighter), negative = outward (suction bulb). Pure function of the
/// projected coordinates so it's unit-testable without a base plug.
#[must_use]
fn inset_field(spec: &CanalSpec, frac: f64, axial_dist: f64, cos_theta: f64) -> f64 {
    let mut d = 0.0;

    // Grip rings — axisymmetric inward pinches.
    for ring in &spec.rings {
        d += ring.depth_m * raised_cosine_pulse(frac, ring.center_frac, ring.half_width_frac);
    }

    // Frenulum side weight: full on the frenulum wall, fading dorsally.
    let w = cos_theta.max(0.0);

    // Frenulum D-section pinch — constant inward bias on the frenulum
    // side over the stimulation zone.
    if spec.dsection_depth_m > 0.0 {
        d += spec.dsection_depth_m
            * w
            * smooth_window(frac, spec.dsection_zone.0, spec.dsection_zone.1);
    }

    // Frenulum-gated texture ribs — additive (≥0) so they only tighten.
    if spec.texture_amp_m > 0.0 && spec.texture_pitch_m > 1.0e-9 {
        let rib =
            0.5 * (1.0 + (2.0 * std::f64::consts::PI * axial_dist / spec.texture_pitch_m).sin());
        d += spec.texture_amp_m
            * rib
            * w
            * smooth_window(frac, spec.texture_zone.0, spec.texture_zone.1);
    }

    // Terminal suction bulb — outward bulge (negative D) over the tip.
    if spec.suction_bulge_m > 0.0 && spec.suction_start_frac < 1.0 {
        // Half-width pulse centered in the suction zone so the bulge
        // tapers to 0 at both the neck and the very tip.
        let center = 0.5 * (spec.suction_start_frac + 1.0);
        let half_width = 0.5 * (1.0 - spec.suction_start_frac);
        d -= spec.suction_bulge_m * raised_cosine_pulse(frac, center, half_width);
    }

    d
}

/// Custom SDF: the base plug's signed distance plus the canal feature
/// field. Wraps the base plug [`Solid`] (cloned) and evaluates it
/// per-point — the same per-point cost the mesher already pays for the
/// plug, plus the cheap analytic feature field.
#[derive(Debug, Clone)]
struct CanalFeatureSdf {
    base: Solid,
    frame: CanalFrame,
    spec: CanalSpec,
}

impl Sdf for CanalFeatureSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let base_d = self.base.evaluate(&p);
        let (frac, axial_dist, cos_theta) = self.frame.project(p);
        base_d + inset_field(&self.spec, frac, axial_dist, cos_theta)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // Unused: `Solid::from_sdf` bridges via finite differences. Same
        // rationale as `FlangeSdf`.
        self.frame.axis
    }
}

/// Compose the canal features onto the base layer-0 plug, returning the
/// modified plug [`Solid`].
///
/// `base_plug` is the scan-derived `pinned_floor_shell` plug (unchanged
/// baseline girth). `centerline` + `frenulum_dir` (via `spec`) build the
/// feature frame. `mouth_anchor` (the cap-plane centroid, when the scan
/// has a cap) orients the frame so `frac = 0` is the mouth — see
/// [`CanalFrame::new`]; pass `None` only for a capless scan. `bounds`
/// constrains MC evaluation: pass the base plug's bounds — the features
/// only displace the surface by at most `spec.max_inward_depth_m()`
/// inward and `spec.suction_bulge_m` outward, both small relative to the
/// plug, so the base bounds (with the mesher's own padding) already
/// contain the modified surface, except the outward suction bulge which
/// we expand for.
#[must_use]
pub fn build_canal_plug(
    base_plug: &Solid,
    centerline: &[Point3<f64>],
    mouth_anchor: Option<Point3<f64>>,
    spec: &CanalSpec,
) -> Solid {
    let frame_bounds = base_plug
        .bounds()
        .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::new(0.1, 0.1, 0.1)));
    build_canal_plug_framed(base_plug, centerline, mouth_anchor, spec, frame_bounds)
}

/// Like [`build_canal_plug`] but with an EXPLICIT `frame_bounds`.
///
/// `frame_bounds` sets the canal frame's axial-span normalization
/// (`frac = 0..1`), decoupled from `base_plug`'s own bounds (which still set
/// the marching-cubes grid extent).
///
/// This is what lets the texture ride EVERY offset consistently. The plug and
/// each (progressively larger) layer body are offsets of the same scan, but
/// have *different* AABBs. If each framed off its own bounds (as the plain
/// [`build_canal_plug`] does), a ring at a given `center_frac` would normalize
/// to a **different world-axial position on every body**, so the rings would
/// drift apart and the inter-layer wall would NOT be constant. Framing the
/// plug and all bodies off the SAME `frame_bounds` makes
/// `inset_field(frac, axial, cosθ)` identical at each world point across the
/// surfaces, so every surface shifts by the same amount and the wall stays
/// constant. Pass the (innermost) plug's bounds as the shared basis. See the
/// cross-body wall test in `cf-cast-cli`.
#[must_use]
pub fn build_canal_plug_framed(
    base_plug: &Solid,
    centerline: &[Point3<f64>],
    mouth_anchor: Option<Point3<f64>>,
    spec: &CanalSpec,
    frame_bounds: Aabb,
) -> Solid {
    let frame = CanalFrame::new(centerline, spec.frenulum_dir, frame_bounds, mouth_anchor);

    // The MC grid still spans THIS body's own bounds (so the whole body is
    // meshed), expanded outward by the suction bulge so MC sees the bulged
    // tip; inward features stay inside the base bounds.
    let base_bounds = base_plug
        .bounds()
        .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::new(0.1, 0.1, 0.1)));
    let pad = spec.suction_bulge_m + 0.001;
    let bounds = Aabb::new(
        Point3::new(
            base_bounds.min.x - pad,
            base_bounds.min.y - pad,
            base_bounds.min.z - pad,
        ),
        Point3::new(
            base_bounds.max.x + pad,
            base_bounds.max.y + pad,
            base_bounds.max.z + pad,
        ),
    );

    Solid::from_sdf(
        CanalFeatureSdf {
            base: base_plug.clone(),
            frame,
            spec: spec.clone(),
        },
        bounds,
    )
}

/// Debris-drop ceiling for [`filter_plug_debris`].
///
/// Max fraction of the main body's face count that a single detached
/// component may have and still be treated as droppable marching-cubes
/// debris. Above this, [`filter_plug_debris`] errors instead of
/// deleting — a large detachment is a real geometry tear to surface,
/// not a floating speck to hide. The worst stray fragment observed on
/// the noisy sock test scan was 0.47% of the main body; 2% leaves
/// headroom while still catching any substantial detachment.
pub const CANAL_DEBRIS_MAX_DROP_FRACTION: f64 = 0.02;

/// Drop floating marching-cubes debris from a canal plug mesh, keeping
/// the single connected plug body.
///
/// **Guarded — cannot hide real geometry.** It logs exactly what it
/// drops (fragment count + faces), and if any detached component
/// exceeds [`CANAL_DEBRIS_MAX_DROP_FRACTION`] of the main body it
/// **errors** ([`CastError::CanalPlugDetachedComponent`]) and drops
/// nothing — because a large detached chunk is a genuine defect (a torn
/// floor cap or feature region), not the SDF→MC sign-noise specks this
/// filter exists to remove.
///
/// No-op (mesh untouched, `Ok`) when the plug is already a single
/// connected component — the common case for a clean scan.
///
/// # Errors
///
/// [`CastError::CanalPlugDetachedComponent`] when a detached component
/// is too large to be debris.
// Face counts are well within f64's exact-integer range (< 2^52), so the
// usize→f64 ratio casts below lose no precision in practice.
#[allow(clippy::cast_precision_loss)]
pub fn filter_plug_debris(mesh: &mut IndexedMesh, target: CastTarget) -> Result<(), CastError> {
    use mesh_repair::components::{find_connected_components, keep_largest_component};

    let analysis = find_connected_components(mesh);
    if analysis.component_count <= 1 {
        return Ok(());
    }

    let main_faces = analysis.largest_component_size;
    // `components` is sorted largest-first, so [0] is the body we keep
    // and [1..] are the candidates to drop.
    let largest_detached = analysis
        .components
        .iter()
        .skip(1)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let detached_fraction = if main_faces > 0 {
        largest_detached as f64 / main_faces as f64
    } else {
        1.0
    };

    if detached_fraction > CANAL_DEBRIS_MAX_DROP_FRACTION {
        return Err(CastError::CanalPlugDetachedComponent {
            target,
            detached_faces: largest_detached,
            detached_fraction: detached_fraction * 100.0,
            main_faces,
        });
    }

    let dropped_components = analysis.component_count - 1;
    let dropped_faces: usize = analysis.components.iter().skip(1).map(Vec::len).sum();
    let removed = keep_largest_component(mesh);
    eprintln!(
        "[cf-cast] {target} canal debris filter: dropped {dropped_components} floating fragment(s) \
         ({dropped_faces} faces, largest {largest_detached} = {:.2}% of main; all \u{2264} {:.1}% \
         threshold) — kept main body {main_faces} faces ({removed} removed)",
        detached_fraction * 100.0,
        CANAL_DEBRIS_MAX_DROP_FRACTION * 100.0,
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation
    )]

    use super::*;
    use approx::assert_relative_eq;

    fn straight_centerline() -> Vec<Point3<f64>> {
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.12)]
    }

    fn unit_bounds() -> Aabb {
        Aabb::new(
            Point3::new(-0.02, -0.02, 0.0),
            Point3::new(0.02, 0.02, 0.12),
        )
    }

    #[test]
    fn frame_fraction_runs_mouth_to_tip_along_axis() {
        let frame = CanalFrame::new(
            &straight_centerline(),
            Vector3::new(0.0, 1.0, 0.0),
            unit_bounds(),
            None,
        );
        let (frac_lo, _, _) = frame.project(Point3::new(0.0, 0.0, 0.0));
        let (frac_hi, _, _) = frame.project(Point3::new(0.0, 0.0, 0.12));
        assert!(
            frac_lo < 0.01,
            "z=0 should be near fraction 0, got {frac_lo}"
        );
        assert!(
            frac_hi > 0.99,
            "z=L should be near fraction 1, got {frac_hi}"
        );
    }

    #[test]
    fn frame_anchors_frac_zero_at_cap_mouth_when_centerline_runs_deep_to_mouth() {
        // Regression for the iter-1 clone bug: cf-scan-prep's centerline
        // ran glans → floor, so first=deep, last=mouth(cap). With the cap
        // centroid as the mouth anchor, frac 0 must land at the cap end.
        let cl = vec![Point3::new(0.0, 0.0, 0.12), Point3::new(0.0, 0.0, 0.0)];
        let cap_centroid = Some(Point3::new(0.0, 0.0, 0.0));
        let frame = CanalFrame::new(
            &cl,
            Vector3::new(0.0, 1.0, 0.0),
            unit_bounds(),
            cap_centroid,
        );
        let (frac_mouth, _, _) = frame.project(Point3::new(0.0, 0.0, 0.0));
        let (frac_deep, _, _) = frame.project(Point3::new(0.0, 0.0, 0.12));
        assert!(
            frac_mouth < 0.01,
            "cap/mouth end must be frac 0, got {frac_mouth}"
        );
        assert!(frac_deep > 0.99, "deep end must be frac 1, got {frac_deep}");
    }

    #[test]
    fn frame_cos_theta_is_plus_one_on_frenulum_side() {
        let frame = CanalFrame::new(
            &straight_centerline(),
            Vector3::new(0.0, 1.0, 0.0),
            unit_bounds(),
            None,
        );
        // Point offset in +Y (the frenulum direction) → cos θ ≈ 1.
        let (_, _, cos_fren) = frame.project(Point3::new(0.0, 0.01, 0.06));
        assert_relative_eq!(cos_fren, 1.0, epsilon = 1.0e-9);
        // Point offset in -Y (dorsal) → cos θ ≈ -1.
        let (_, _, cos_dorsal) = frame.project(Point3::new(0.0, -0.01, 0.06));
        assert_relative_eq!(cos_dorsal, -1.0, epsilon = 1.0e-9);
    }

    #[test]
    fn ring_produces_positive_inward_depth_at_center() {
        let spec = CanalSpec::iter1();
        let ring = spec.rings[0];
        // At the ring center, axisymmetric → positive regardless of θ.
        let d = inset_field(&spec, ring.center_frac, 0.0, -1.0);
        assert!(
            d >= ring.depth_m * 0.99,
            "ring center should reach ~depth, got {d}"
        );
    }

    #[test]
    fn dsection_pinches_frenulum_side_only() {
        let mut spec = CanalSpec::iter1();
        // Isolate the D-section: kill rings + texture so only the pinch shows.
        spec.rings.clear();
        spec.texture_amp_m = 0.0;
        let mid = 0.5 * (spec.dsection_zone.0 + spec.dsection_zone.1);
        let d_fren = inset_field(&spec, mid, 0.0, 1.0);
        let d_dorsal = inset_field(&spec, mid, 0.0, -1.0);
        assert!(
            d_fren > 0.0,
            "frenulum side should pinch inward, got {d_fren}"
        );
        assert_relative_eq!(d_dorsal, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn suction_bulb_bulges_outward_near_tip() {
        let spec = CanalSpec::iter1();
        let center = 0.5 * (spec.suction_start_frac + 1.0);
        let d = inset_field(&spec, center, 0.0, 0.0);
        assert!(
            d < 0.0,
            "suction zone should bulge outward (negative D), got {d}"
        );
    }

    #[test]
    fn inset_field_is_zero_in_a_clear_gap() {
        let mut spec = CanalSpec::iter1();
        spec.rings.clear(); // remove rings so the gap is genuinely clear
        // Fraction 0.80 is past the texture/d-section zones and before
        // the suction zone (0.90) → no feature.
        let d = inset_field(&spec, 0.80, 0.0, 1.0);
        assert_relative_eq!(d, 0.0, epsilon = 1.0e-12);
    }

    /// Build a connected fan of `n_tris` triangles (all sharing a center
    /// vertex) centered near `cx`. A single connected component.
    fn fan(n_tris: usize, cx: f64) -> (Vec<Point3<f64>>, Vec<[u32; 3]>) {
        let mut v = vec![Point3::new(cx, 0.0, 0.0)];
        for i in 0..=n_tris {
            let a = i as f64 * 0.1;
            v.push(Point3::new(cx + a.cos(), a.sin(), 0.0));
        }
        let f = (0..n_tris)
            .map(|i| [0u32, (1 + i) as u32, (2 + i) as u32])
            .collect();
        (v, f)
    }

    /// Combine two fans into one mesh as two disconnected components
    /// (second fan's indices offset; vertices spatially far apart).
    fn two_component_mesh(main_tris: usize, stray_tris: usize) -> IndexedMesh {
        let (mut verts, mut faces) = fan(main_tris, 0.0);
        let (sverts, sfaces) = fan(stray_tris, 100.0);
        let base = verts.len() as u32;
        verts.extend(sverts);
        faces.extend(
            sfaces
                .into_iter()
                .map(|t| [t[0] + base, t[1] + base, t[2] + base]),
        );
        let mut m = IndexedMesh::new();
        m.vertices = verts;
        m.faces = faces;
        m
    }

    const T: CastTarget = CastTarget::Plug {
        layer_index: Some(0),
    };

    #[test]
    fn filter_plug_debris_is_noop_on_single_component() {
        let (v, f) = fan(10, 0.0);
        let mut mesh = IndexedMesh::new();
        mesh.vertices = v;
        mesh.faces = f;
        let before = mesh.faces.len();
        filter_plug_debris(&mut mesh, T).unwrap();
        assert_eq!(
            mesh.faces.len(),
            before,
            "clean single body must be untouched"
        );
    }

    #[test]
    fn filter_plug_debris_drops_small_floating_fragment() {
        // 1 stray tri / 100 main = 1% < 2% threshold → dropped.
        let mut mesh = two_component_mesh(100, 1);
        filter_plug_debris(&mut mesh, T).unwrap();
        assert_eq!(
            mesh.faces.len(),
            100,
            "small debris fragment should be dropped"
        );
    }

    #[test]
    fn filter_plug_debris_errors_on_substantial_detachment() {
        // 10 stray tris / 100 main = 10% > 2% threshold → error, drop nothing.
        let mut mesh = two_component_mesh(100, 10);
        let before = mesh.faces.len();
        let err = filter_plug_debris(&mut mesh, T).unwrap_err();
        assert!(
            matches!(err, CastError::CanalPlugDetachedComponent { .. }),
            "substantial detachment must error, got {err:?}"
        );
        assert_eq!(
            mesh.faces.len(),
            before,
            "errored filter must not mutate the mesh"
        );
    }

    #[test]
    fn feature_sdf_equals_base_plus_field() {
        // Base = a sphere; the canal SDF must be base distance + field.
        let base = Solid::sphere(0.02);
        let spec = CanalSpec::iter1();
        let plug = build_canal_plug(&base, &straight_centerline(), None, &spec);
        // Evaluate at a point and confirm it differs from the bare sphere
        // distance by exactly the feature field there.
        let p = Point3::new(0.0, 0.018, 0.048); // frenulum side, in texture zone
        let frame = CanalFrame::new(
            &straight_centerline(),
            spec.frenulum_dir,
            base.bounds().unwrap(),
            None,
        );
        let (frac, axial, cos_theta) = frame.project(p);
        let expected = base.evaluate(&p) + inset_field(&spec, frac, axial, cos_theta);
        assert_relative_eq!(plug.evaluate(&p), expected, epsilon = 1.0e-9);
    }

    /// The wall-preservation invariant (the headline of the scan-surface
    /// texture unification): when the plug and a LARGER layer body are framed
    /// off the SAME span, the canal displacement they each receive is identical
    /// at every world point — so both surfaces shift together and the
    /// inter-layer wall stays constant. A rings-only spec makes the
    /// displacement a pure function of the axial fraction, so any drift shows
    /// up directly.
    ///
    /// Also asserts the discriminating case: framing the body off its OWN
    /// (different) bounds — the plain `build_canal_plug` — moves the ring to a
    /// different world position, which is the bug `build_canal_plug_framed`
    /// fixes.
    #[test]
    fn shared_frame_keeps_the_ring_displacement_equal_across_offset_bodies() {
        // Plug + a larger body, both domed-tip (asymmetric: the dome grows more
        // than the flat floor under an offset — exactly the real plug/shell
        // geometry), sharing one axis.
        let plug = Solid::cylinder(0.010, 0.040)
            .union(Solid::sphere(0.010).translate(Vector3::new(0.0, 0.0, 0.040)));
        let body = Solid::cylinder(0.018, 0.048)
            .union(Solid::sphere(0.018).translate(Vector3::new(0.0, 0.0, 0.048)));
        let centerline = vec![Point3::new(0.0, 0.0, -0.07), Point3::new(0.0, 0.0, 0.07)];

        // Rings only → displacement = f(frac) alone (axisymmetric).
        let mut spec = CanalSpec::iter1();
        spec.texture_amp_m = 0.0;
        spec.dsection_depth_m = 0.0;
        spec.suction_bulge_m = 0.0;

        let shared = plug.bounds().unwrap();
        let plug_tex = build_canal_plug_framed(&plug, &centerline, None, &spec, shared);
        let body_shared = build_canal_plug_framed(&body, &centerline, None, &spec, shared);
        // The bug: body framed off its OWN bounds.
        let body_perbody = build_canal_plug(&body, &centerline, None, &spec);

        // Displacement = textured.eval - base.eval = the canal inset field.
        let plug_disp = |z: f64| {
            let p = Point3::new(0.004, 0.0, z);
            plug_tex.evaluate(&p) - plug.evaluate(&p)
        };
        let body_shared_disp = |z: f64| {
            let p = Point3::new(0.004, 0.0, z);
            body_shared.evaluate(&p) - body.evaluate(&p)
        };
        let body_perbody_disp = |z: f64| {
            let p = Point3::new(0.004, 0.0, z);
            body_perbody.evaluate(&p) - body.evaluate(&p)
        };

        let mut max_shared_drift = 0.0_f64;
        let mut max_perbody_drift = 0.0_f64;
        for i in 0..=120 {
            let z = -0.02 + 0.07 * f64::from(i) / 120.0;
            max_shared_drift = max_shared_drift.max((plug_disp(z) - body_shared_disp(z)).abs());
            max_perbody_drift = max_perbody_drift.max((plug_disp(z) - body_perbody_disp(z)).abs());
        }

        // Fix: the shared frame makes the displacement IDENTICAL → wall constant.
        assert!(
            max_shared_drift < 1.0e-9,
            "shared-frame ring displacement must match the plug everywhere; drift {max_shared_drift}"
        );
        // The test discriminates: per-body framing genuinely drifts the ring
        // (this is the wall defect the fix removes).
        assert!(
            max_perbody_drift > 5.0e-4,
            "per-body framing should drift the ring (the bug); drift {max_perbody_drift}"
        );
    }
}
