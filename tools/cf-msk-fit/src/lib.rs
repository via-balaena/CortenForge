//! cf-msk-fit — the scan as a parameter source, plus the render-overlay placement.
//!
//! Two consumers of the scan landmarks live here:
//! - [`ScanSource`] — the scan as a [`cf_msk_lib::ParamSource`] for the
//!   `realize → emit` builder path. As of A3 it derives a **per-segment
//!   anisotropic** scale (femur from thigh length, tibia from shank length — see
//!   its docs); the shank no longer inherits the femur scale.
//! - [`Fitter`] / [`place_knee`] — the render overlay (cf-osim's gait2392 knee,
//!   validated against real OpenSim, posed in the scan frame for visualization).
//!   This path still uses a single **similarity transform** (rotate OpenSim → scan
//!   frame, scale, translate) pinning the knee center to the detected knee.
//!
//! Approximations (called out honestly):
//! - The [`Fitter`] overlay uses a single euclidean scale (thigh / OpenSim-femur
//!   length); the per-segment anisotropic morph lives on the [`ScanSource`] →
//!   `realize` path. Girth (transverse) on `ScanSource` stays at the template
//!   default until the generator supplies an anthropometric reference (A3-PR3).
//! - The [`Fitter`] transform is built at full knee extension (θ=0).
//! - Medio-lateral / antero-posterior orientation follows gait2392's frame
//!   convention; if a real scan's M-L axis differs, it gets re-pinned at v2.

use cf_anthro::Landmarks;
use cf_msk_lib::{BodyParams, Model, ParamSource};
use cf_osim::oracle::{Kinematics, Pose};
use nalgebra::{Matrix3, Point3, Vector3};

pub mod scorecard;

/// The scan as a **parameter source** — `cf-anthro` landmarks → [`BodyParams`]
/// that morph the template (the parametric-builder-first seam). This is the scan
/// path demoted from "the pipeline" to one [`ParamSource`] alongside
/// `cf_msk_lib::CanonicalSource`; the same `realize → emit` path consumes both.
///
/// As of A3 this is a **per-segment anisotropic** scale: the femur's axial scale
/// is `thigh_length / template_femur_axial` and the tibia's is `shank_length /
/// template_tibia_axial` — the scan's `shank_length_m` finally drives the tibia
/// independently, now that the ankle (`talus_r`) gives the tibia a defined length
/// (this was the documented v1 block). Girth (transverse) stays at the template
/// default here: a girth→scale derivation needs an anthropometric *reference*
/// girth, which arrives with the percentile generator (A3-PR3). v1 still captures
/// only the scan's **size** (per-segment scale), not its pose/orientation (the
/// rigid placement [`Fitter::pose`] does that for the render overlay).
pub struct ScanSource {
    landmarks: Landmarks,
}

impl ScanSource {
    pub fn new(landmarks: Landmarks) -> Self {
        Self { landmarks }
    }
}

impl ParamSource for ScanSource {
    fn params(&self, template: &Model) -> BodyParams {
        // Precondition guard, enforced in ALL builds (not debug-only): degenerate
        // landmark lengths (a failed/empty scan) would otherwise produce a
        // NaN/zero/negative scale that silently corrupts the emitted model.
        assert!(
            self.landmarks.thigh_length_m > 1e-6 && self.landmarks.shank_length_m > 1e-6,
            "degenerate thigh ({}) or shank ({}) length",
            self.landmarks.thigh_length_m,
            self.landmarks.shank_length_m,
        );
        BodyParams::from_lengths(
            template,
            self.landmarks.thigh_length_m,
            self.landmarks.shank_length_m,
        )
    }
}

/// A bone as a segment from its proximal to distal end (scan frame, meters).
#[derive(Debug, Clone, Copy)]
pub struct Bone {
    pub proximal: Point3<f64>,
    pub distal: Point3<f64>,
}

/// A muscle/tendon path placed in the scan frame.
#[derive(Debug, Clone)]
pub struct PlacedMuscle {
    pub name: String,
    /// The active path at this pose — normally ≥2 points (one tube segment).
    /// A muscle whose conditional points all deactivate at an extreme angle
    /// could yield <2; downstream rendering/length code handles that gracefully.
    pub polyline: Vec<Point3<f64>>,
}

/// The OpenSim knee placed in the scan frame.
#[derive(Debug, Clone)]
pub struct Placement {
    /// The fixed, femur-side joint center (the detected knee landmark). Under
    /// flexion the tibia's proximal end tracks the COUPLED joint and separates
    /// from this by the rollback — so `knee == tibia.proximal` only at extension.
    pub knee: Point3<f64>,
    /// Femur: knee (distal) → hip (proximal), scaled to the thigh length. Fixed
    /// across flexion.
    pub femur: Bone,
    /// Tibia: a rigid bone of the shank length. Its proximal (knee) end tracks
    /// the coupled joint center, so it slides off `knee` as the knee flexes (the
    /// rollback); distal is the ankle end.
    pub tibia: Bone,
    pub muscles: Vec<PlacedMuscle>,
    /// The uniform scale applied (thigh length / OpenSim femur length).
    pub scale: f64,
}

/// A fitted knee: the similarity transform (scan ← OpenSim, anchored at full
/// extension) plus the geometry to pose the skeleton at any flexion angle.
pub struct Fitter<'a> {
    model: &'a Model,
    kin: Kinematics<'a>,
    r: Matrix3<f64>,
    scale: f64,
    knee_osim: Point3<f64>,
    kp: Point3<f64>,
    thigh: f64,
    /// Tibia distal point in the tibia body frame (so the bone rotates rigidly
    /// with the coupled knee as it flexes).
    tibia_distal_loc: Vector3<f64>,
}

impl<'a> Fitter<'a> {
    /// Build the placement transform from the model + landmarks (at θ=0).
    pub fn new(model: &'a Model, lm: &Landmarks) -> Self {
        let kin = Kinematics::new(model);
        let neutral = Pose::new();
        let at0 = |body: &str, loc: Vector3<f64>| kin.body_pose(body, &neutral) * Point3::from(loc);

        let knee_osim = at0("tibia_r", Vector3::zeros());
        let hip_osim = at0("femur_r", Vector3::zeros());
        let femur_vec = hip_osim - knee_osim;
        let femur_len = femur_vec.norm();
        debug_assert!(
            femur_len > 1e-6 && lm.thigh_length_m > 1e-6,
            "degenerate femur ({femur_len}) or thigh length ({}) — can't place",
            lm.thigh_length_m
        );
        let scale = lm.thigh_length_m / femur_len;

        // Rotation: femur axis → scan +z (proximal); gait2392 lateral (+z) → +x.
        let w = femur_vec / femur_len;
        let lat = Vector3::z();
        let proj = lat - lat.dot(&w) * w;
        let x_dir = if proj.norm() < 1e-9 {
            Vector3::x()
        } else {
            proj.normalize()
        };
        let y_dir = w.cross(&x_dir);
        let r = Matrix3::new(
            x_dir.x, x_dir.y, x_dir.z, // row 0 → scan x
            y_dir.x, y_dir.y, y_dir.z, // row 1 → scan y
            w.x, w.y, w.z, // row 2 → scan z (femur axis)
        );

        Fitter {
            model,
            kin,
            r,
            scale,
            knee_osim,
            kp: lm.knee_point,
            thigh: lm.thigh_length_m,
            // Tibia long axis is −y in the tibia frame; length scan-shank/scale.
            tibia_distal_loc: Vector3::new(0.0, -lm.shank_length_m / scale, 0.0),
        }
    }

    /// The uniform model→scan scale (thigh length / OpenSim femur length).
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// Pose the skeleton at knee flexion angle `theta` (rad; 0 = extension,
    /// negative = flexion in gait2392). The femur is fixed; the tibia rotates
    /// about the knee with the coupled joint, and the muscle points (incl. the
    /// patella moving point) re-evaluate at `theta`.
    pub fn pose(&self, theta: f64) -> Placement {
        // Knee flexion only; the hip stays at its neutral default (absent → 0).
        let q = Pose::from([("knee_angle_r".to_string(), theta)]);
        let world =
            |body: &str, loc: Vector3<f64>| self.kin.body_pose(body, &q) * Point3::from(loc);
        let xform = |q: Point3<f64>| self.kp + self.scale * (self.r * (q - self.knee_osim));

        let femur = Bone {
            proximal: self.kp + self.thigh * Vector3::z(),
            distal: self.kp,
        };
        // BOTH tibia ends from the SAME θ-posed (rigid) tibia, so the bone keeps
        // its length. Its knee end tracks the COUPLED joint center, which the
        // gait2392 knee translates under flexion (the rollback we validated in
        // S0) — so the tibia separates slightly from the fixed femur knee end:
        // that small gap IS the coupled translation, not a bug.
        let tibia = Bone {
            proximal: xform(world("tibia_r", Vector3::zeros())),
            distal: xform(world("tibia_r", self.tibia_distal_loc)),
        };
        let muscles = self
            .model
            .muscles
            .iter()
            .map(|m| PlacedMuscle {
                name: m.name.clone(),
                polyline: m
                    .path
                    .iter()
                    .filter(|pp| pp.active(theta))
                    .map(|pp| xform(world(&pp.body, pp.location_at(theta))))
                    .collect(),
            })
            .collect();

        Placement {
            knee: self.kp,
            femur,
            tibia,
            muscles,
            scale: self.scale,
        }
    }
}

/// Place the OpenSim knee model onto the detected landmarks at full extension.
pub fn place_knee(model: &Model, lm: &Landmarks) -> Placement {
    Fitter::new(model, lm).pose(0.0)
}
