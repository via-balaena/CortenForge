//! Live preview meshing for the wizard's "Shape your piece" step.
//!
//! The faithful preview is the **real plug**: the cleaned-scan flood-fill SDF
//! offset inward by the cavity inset, textured with the *same* `cf_cast`
//! canal field the cast composes. [`PlugPreview`] caches the slow part (the
//! flood-fill SDF + the prep centerline / mouth anchor) so each inset / ridge
//! edit only re-offsets + re-textures + re-meshes at a coarse cell size — fast
//! enough to update live as the user drags a control.
//!
//! When no cleaned scan is loaded (or it can't be parsed), the frontend falls
//! back to [`proxy_preview_mesh`]: a flat-floor plug proxy textured with the
//! same field. The real part is always cast at full detail by the pipeline.

use std::path::Path;

use cf_cap_planes::{CapPlane, parse_cap_planes};
use cf_cast::{CanalSpec, RingSpec, preview_textured_plug, preview_textured_solid};
use cf_cast_cli::{SharedScanSdf, load_scan_sdf, parse_centerline_from_prep_toml};
use cf_design::{Aabb, Solid};
use cf_studio_core::{RidgeOptions, RidgeRing};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::error::{EngineError, Result};

/// Coarse preview cell size + flood-fill resolution (meters). ~2 mm meshes a
/// small plug in well under a second, so the slider stays interactive.
const PREVIEW_CELL_M: f64 = 0.002;

/// Outward margin (meters) padded onto the scan AABB for the SDF flood-fill
/// bounds + the marching-cubes domain, covering the canal field's outward
/// tip-relief / suction bulge so it isn't clipped.
const PREVIEW_BULGE_MARGIN_M: f64 = 0.006;

/// Proxy-fallback dimensions: a ~24 mm-diameter, 80 mm flat-floor plug.
const PROXY_RADIUS_M: f64 = 0.012;
const PROXY_HALF_HEIGHT_M: f64 = 0.040;

/// A cached real-scan plug preview. Holds the cleaned-scan flood-fill SDF,
/// the prep centerline, and the mouth anchor (cap-plane centroid) — all built
/// once by [`PlugPreview::load`] — so [`PlugPreview::mesh`] can re-offset +
/// re-texture + re-mesh on every edit without re-running the flood-fill.
pub struct PlugPreview {
    scan: SharedScanSdf,
    centerline: Vec<Point3<f64>>,
    mouth_anchor: Option<Point3<f64>>,
    bounds: Aabb,
}

impl PlugPreview {
    /// Build the cache from the cleaned scan STL + its `.prep.toml`. The
    /// flood-fill SDF build is the cost (hundreds of ms); run this once per
    /// page entry, off the UI thread if possible.
    ///
    /// # Errors
    /// [`EngineError::Preview`] if the prep can't be read / parsed or the
    /// scan SDF can't be built.
    pub fn load(cleaned_stl: &Path, prep_toml: &Path) -> Result<Self> {
        let prep_text = std::fs::read_to_string(prep_toml)
            .map_err(|e| EngineError::Preview(format!("read prep {}: {e}", prep_toml.display())))?;
        let centerline = parse_centerline_from_prep_toml(&prep_text)
            .map_err(|e| EngineError::Preview(format!("parse centerline: {e:#}")))?;
        let cap_planes = parse_cap_planes(&prep_text)
            .map_err(|e| EngineError::Preview(format!("parse cap planes: {e:#}")))?;
        // Anchor the canal frame's `frac = 0` at the mouth (first cap-plane
        // centroid), exactly as the cast does.
        let mouth_anchor = cap_planes
            .first()
            .map(CapPlane::as_tuple)
            .map(|(centroid, _normal)| centroid);
        let loaded = load_scan_sdf(cleaned_stl, PREVIEW_BULGE_MARGIN_M, PREVIEW_CELL_M)
            .map_err(|e| EngineError::Preview(format!("build scan SDF: {e:#}")))?;
        let bounds = loaded.aabb.expanded(PREVIEW_BULGE_MARGIN_M);
        Ok(Self {
            scan: loaded.sdf,
            centerline,
            mouth_anchor,
            bounds,
        })
    }

    /// Mesh the shaped plug: the scan offset inward by `cavity_inset_m`,
    /// textured with `ridges`, at the coarse preview resolution. The ridge
    /// field is the same one the cast composes, so what the user tunes here
    /// is what the cast cuts.
    #[must_use]
    pub fn mesh(&self, ridges: &RidgeOptions, cavity_inset_m: f64) -> IndexedMesh {
        let base = Solid::from_sdf(self.scan.clone(), self.bounds).offset(-cavity_inset_m);
        let spec = canal_spec_from(ridges);
        preview_textured_solid(
            &base,
            &self.centerline,
            self.mouth_anchor,
            &spec,
            self.bounds,
            PREVIEW_CELL_M,
        )
    }
}

/// The dependency-free fallback when no cleaned scan is loaded: the flat-floor
/// plug *proxy* textured with `ridges` (same field, representative shape). A
/// disabled `ridges` renders the smooth proxy.
#[must_use]
pub fn proxy_preview_mesh(ridges: &RidgeOptions) -> IndexedMesh {
    preview_textured_plug(
        &canal_spec_from(ridges),
        PROXY_RADIUS_M,
        PROXY_HALF_HEIGHT_M,
        PREVIEW_CELL_M,
    )
}

/// Map the owned [`RidgeOptions`] onto a `cf_cast::CanalSpec` — the one
/// unified field the cast applies to the plug + every shell (rings + texture +
/// side pinch + tip relief + orientation). A disabled `ridges` yields a
/// feature-free spec (the smooth piece).
fn canal_spec_from(o: &RidgeOptions) -> CanalSpec {
    let mut spec = CanalSpec::iter1();
    if !o.enabled {
        spec.rings.clear();
        spec.texture_amp_m = 0.0;
        spec.dsection_depth_m = 0.0;
        spec.suction_bulge_m = 0.0;
        return spec;
    }
    spec.rings = o.rings.iter().map(ring_spec).collect();
    spec.texture_amp_m = o.texture_depth_m;
    spec.texture_pitch_m = o.texture_spacing_m;
    spec.dsection_depth_m = o.side_pinch_depth_m;
    spec.suction_bulge_m = o.tip_relief_depth_m;
    let theta = o.orientation_deg.to_radians();
    spec.frenulum_dir = Vector3::new(theta.sin(), theta.cos(), 0.0);
    spec
}

fn ring_spec(r: &RidgeRing) -> RingSpec {
    RingSpec {
        center_frac: r.position_frac,
        depth_m: r.depth_m,
        half_width_frac: r.half_width_frac,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proxy_preview_meshes_on_and_off() {
        assert!(
            !proxy_preview_mesh(&RidgeOptions::default())
                .vertices
                .is_empty()
        );
        assert!(
            !proxy_preview_mesh(&RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            })
            .vertices
            .is_empty()
        );
    }
}
