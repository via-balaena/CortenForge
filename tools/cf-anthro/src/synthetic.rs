//! Synthetic leg-scan generator with known ground truth.
//!
//! Real leg scans aren't on hand yet (recon O3), so we build watertight tapered
//! limbs whose knee location, epicondyle width, and segment lengths we control
//! exactly — letting the S2 detector be developed and validated cleanly before
//! real scans slot in. The radius profile captures the essential signal the
//! detector keys on: a cross-sectional-area minimum at the knee, flanked by the
//! thigh and calf girth maxima. Cross-sections are elliptical (M-L wider than
//! A-P), so the epicondyle (medio-lateral) width is a genuine, detectable
//! quantity, not just the diameter.
//!
//! Note the validation this enables is numerics-only: the knee is *placed at*
//! the area minimum here and the detector *finds* the area minimum, so a passing
//! test does not confirm the anatomical premise (area-min ⇒ joint line) — that
//! needs real scans + independent landmark truth.

use mesh_types::{IndexedMesh, Point3};
use std::f64::consts::TAU;

/// One axial control station: position `z` (m) with medio-lateral and
/// antero-posterior radii (m).
#[derive(Clone, Copy)]
pub struct Station {
    pub z: f64,
    pub r_ml: f64,
    pub r_ap: f64,
}

/// A synthetic limb: ordered control stations ankle → … → hip.
pub struct LegSpec {
    pub stations: Vec<Station>,
    /// Index into `stations` that is the knee (the area minimum).
    pub knee_idx: usize,
}

/// The ground truth a detector should recover.
#[derive(Debug, Clone, Copy)]
pub struct GroundTruth {
    pub knee_z: f64,
    /// Medio-lateral width at the knee (m) = 2 · r_ml(knee).
    pub epicondyle_width_m: f64,
    /// Knee → hip (proximal end) axial length (m).
    pub thigh_length_m: f64,
    /// Ankle (distal end) → knee axial length (m).
    pub shank_length_m: f64,
}

impl LegSpec {
    /// A default anatomically-plausible right leg, ankle at z=0 → hip at z≈0.88 m.
    pub fn default_leg() -> Self {
        Self::from_radii(
            &[
                // (z, r_ml, r_ap)
                (0.00, 0.040, 0.036), // ankle
                (0.28, 0.060, 0.052), // calf bulge (max)
                (0.43, 0.050, 0.044), // KNEE (min) — wider M-L (epicondyles)
                (0.66, 0.088, 0.082), // mid-thigh (max)
                (0.88, 0.078, 0.074), // hip cut (slight taper)
            ],
            2,
        )
    }

    fn from_radii(rows: &[(f64, f64, f64)], knee_idx: usize) -> Self {
        let stations = rows
            .iter()
            .map(|&(z, r_ml, r_ap)| Station { z, r_ml, r_ap })
            .collect();
        LegSpec { stations, knee_idx }
    }

    /// Scale every radius by `s` and stretch the axis by `len` — for generating
    /// distinct validation subjects.
    pub fn scaled(&self, radius_scale: f64, length_scale: f64) -> Self {
        let stations = self
            .stations
            .iter()
            .map(|s| Station {
                z: s.z * length_scale,
                r_ml: s.r_ml * radius_scale,
                r_ap: s.r_ap * radius_scale,
            })
            .collect();
        LegSpec {
            stations,
            knee_idx: self.knee_idx,
        }
    }

    pub fn ground_truth(&self) -> GroundTruth {
        let knee = self.stations[self.knee_idx];
        let hip_z = self.stations.last().unwrap().z;
        let ankle_z = self.stations.first().unwrap().z;
        GroundTruth {
            knee_z: knee.z,
            epicondyle_width_m: 2.0 * knee.r_ml,
            thigh_length_m: hip_z - knee.z,
            shank_length_m: knee.z - ankle_z,
        }
    }

    fn radii_at(&self, z: f64) -> (f64, f64) {
        let s = &self.stations;
        if z <= s[0].z {
            return (s[0].r_ml, s[0].r_ap);
        }
        if z >= s[s.len() - 1].z {
            let l = s[s.len() - 1];
            return (l.r_ml, l.r_ap);
        }
        let i = s.iter().rposition(|st| st.z <= z).unwrap();
        let (a, b) = (s[i], s[i + 1]);
        // Smoothstep blend (zero slope at the knots) — a real knee narrows
        // smoothly, so the area minimum at the knee is a smooth min, not a kink.
        let t = (z - a.z) / (b.z - a.z);
        let ts = t * t * (3.0 - 2.0 * t);
        (
            a.r_ml + ts * (b.r_ml - a.r_ml),
            a.r_ap + ts * (b.r_ap - a.r_ap),
        )
    }

    /// Build a watertight elliptical tube mesh (+ end caps) and its ground truth.
    pub fn build(&self, n_rings: usize, n_radial: usize) -> (IndexedMesh, GroundTruth) {
        assert!(n_rings >= 2 && n_radial >= 3);
        let (z0, z1) = (self.stations[0].z, self.stations.last().unwrap().z);
        let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(n_rings * n_radial + 2);

        for i in 0..n_rings {
            let z = z0 + (z1 - z0) * (i as f64) / ((n_rings - 1) as f64);
            let (r_ml, r_ap) = self.radii_at(z);
            for j in 0..n_radial {
                let a = TAU * (j as f64) / (n_radial as f64);
                vertices.push(Point3::new(r_ml * a.cos(), r_ap * a.sin(), z));
            }
        }
        let cb = vertices.len();
        vertices.push(Point3::new(0.0, 0.0, z0)); // bottom cap center
        let ct = vertices.len();
        vertices.push(Point3::new(0.0, 0.0, z1)); // top cap center

        let idx = |i: usize, j: usize| (i * n_radial + j % n_radial) as u32;
        let mut faces: Vec<[u32; 3]> = Vec::new();
        // Side walls (consistent winding).
        for i in 0..n_rings - 1 {
            for j in 0..n_radial {
                let (v00, v01) = (idx(i, j), idx(i, j + 1));
                let (v10, v11) = (idx(i + 1, j), idx(i + 1, j + 1));
                faces.push([v00, v01, v11]);
                faces.push([v00, v11, v10]);
            }
        }
        // End caps (fans to the centers).
        for j in 0..n_radial {
            faces.push([cb as u32, idx(0, j + 1), idx(0, j)]);
            faces.push([ct as u32, idx(n_rings - 1, j), idx(n_rings - 1, j + 1)]);
        }

        (IndexedMesh { vertices, faces }, self.ground_truth())
    }
}
