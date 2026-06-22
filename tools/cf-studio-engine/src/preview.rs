//! Live preview meshing for the wizard's "Shape your piece" step.
//!
//! The preview is a close approximation of the **real plug**: the cleaned-scan
//! flood-fill SDF offset inward by the cavity inset, textured with the *same*
//! `cortenforge::cf_cast` canal field (resolved through the cast's canonical mapping, so the
//! ridge values match exactly). [`PlugPreview`] caches the slow part (the
//! flood-fill SDF + the prep centerline / mouth anchor) so each inset / ridge
//! edit only re-offsets + re-textures + re-meshes at a coarse cell size — fast
//! enough to update live as the user drags a control.
//!
//! It is an *approximation*, not a pixel-exact twin: the preview offsets the
//! scan with a plain `Solid::offset` over padded scan bounds, whereas the cast
//! builds the plug with `cortenforge::cf_design::pinned_floor_shell` (cap-pinned floor) over
//! its own eval bounds. So the floor/cap and the frac-keyed ring positions can
//! drift by a small amount from the final cast. The ridge *field* and its
//! parameters are exact; the base plug shape is representative.
//!
//! When no cleaned scan is loaded (or it can't be parsed), the frontend falls
//! back to [`proxy_preview_mesh`]: a flat-floor plug proxy textured with the
//! same field. The real part is always cast at full detail by the pipeline.

use std::path::Path;

use cf_studio_core::RidgeOptions;
use cortenforge::cf_cap_planes::{CapPlane, parse_cap_planes};
use cortenforge::cf_cast::{CanalSpec, preview_textured_plug, preview_textured_solid};
use cortenforge::cf_cast_cli::{
    SharedScanSdf, load_scan_sdf, parse_centerline_from_prep_toml, resolve_canal_spec,
};
use cortenforge::cf_design::{Aabb, Solid};
use cortenforge::mesh_types::IndexedMesh;
use nalgebra::Point3;

use crate::error::{EngineError, Result};
use crate::mold::canal_config_from_ridges;

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

/// Map the owned [`RidgeOptions`] onto a `cortenforge::cf_cast::CanalSpec` — the one
/// unified field the cast applies to the plug + every shell (rings + texture +
/// side pinch + tip relief + orientation).
///
/// Resolves through the EXACT path the cast uses
/// (`canal_config_from_ridges` → `cortenforge::cf_cast_cli::resolve_canal_spec`), so the
/// preview's spec is identical to the cast's by construction — no second
/// hand-maintained mapping to drift. A disabled `ridges` yields a feature-free
/// spec (the smooth piece) — the cast simply skips the canal entirely in that
/// case, so a smooth preview matches a smooth plug.
fn canal_spec_from(o: &RidgeOptions) -> CanalSpec {
    if !o.enabled {
        let mut spec = CanalSpec::iter1();
        spec.rings.clear();
        spec.texture_amp_m = 0.0;
        spec.dsection_depth_m = 0.0;
        spec.suction_bulge_m = 0.0;
        return spec;
    }
    resolve_canal_spec(&canal_config_from_ridges(o))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enabled_ridges_resolve_to_the_cast_spec_fields() {
        // The preview spec is composed through the cast's canonical resolver,
        // so the wizard's values must arrive in the CanalSpec the cast cuts.
        let o = RidgeOptions {
            enabled: true,
            texture_depth_m: 0.0012,
            texture_spacing_m: 0.007,
            side_pinch_depth_m: 0.0011,
            tip_relief_depth_m: 0.0025,
            orientation_deg: 90.0,
            ..RidgeOptions::default()
        };
        let spec = canal_spec_from(&o);
        assert!(!spec.rings.is_empty(), "rings carried through");
        assert!((spec.texture_amp_m - 0.0012).abs() < 1e-12);
        assert!((spec.texture_pitch_m - 0.007).abs() < 1e-12);
        assert!((spec.dsection_depth_m - 0.0011).abs() < 1e-12);
        assert!((spec.suction_bulge_m - 0.0025).abs() < 1e-12);
        // 90° → frenulum_dir = [sin90, cos90, 0] = [1, 0, 0].
        assert!((spec.frenulum_dir.x - 1.0).abs() < 1e-9);
        assert!(spec.frenulum_dir.y.abs() < 1e-9);
    }

    #[test]
    fn disabled_ridges_resolve_to_a_smooth_spec() {
        let spec = canal_spec_from(&RidgeOptions::default());
        assert!(spec.rings.is_empty(), "no rings when off");
        assert!(spec.texture_amp_m.abs() < 1e-12);
        assert!(spec.dsection_depth_m.abs() < 1e-12);
        assert!(spec.suction_bulge_m.abs() < 1e-12);
    }

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
