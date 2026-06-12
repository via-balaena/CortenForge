//! Live texture-preview meshing for the wizard's Texture step.
//!
//! Maps the owned [`RidgeOptions`] (interior canal) / [`ShellRidgeOptions`]
//! (exterior shell rings) onto a `cf_cast::CanalSpec` and renders a coarse
//! textured capsule via [`cf_cast::preview_textured_capsule`]. The proxy is a
//! fixed representative tube — it shows the *pattern* the knobs produce, fast
//! enough to update live; the real part is cast at full detail.

use cf_cast::{CanalSpec, RingSpec, preview_textured_capsule};
use cf_studio_core::{RidgeOptions, ShellRidgeOptions};
use mesh_types::IndexedMesh;
use nalgebra::Vector3;

/// Representative proxy dimensions (meters) + the coarse preview cell size.
/// A ~24 mm-diameter, 80 mm tube reads clearly and meshes in milliseconds.
const PREVIEW_RADIUS_M: f64 = 0.012;
const PREVIEW_HALF_HEIGHT_M: f64 = 0.040;
const PREVIEW_CELL_M: f64 = 0.0015;

/// Which texture the preview is currently rendering (so the UI can label it).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewShowing {
    /// The interior canal (on the plug).
    Interior,
    /// The exterior / inter-layer shell rings.
    Exterior,
    /// Neither enabled — a smooth proxy.
    Smooth,
}

/// Build the live preview mesh for the current texture settings. Interior takes
/// priority when both are on (it's the richer feature); otherwise the exterior
/// shell, else a smooth proxy. Returns the mesh + what it's showing.
#[must_use]
pub fn texture_preview_mesh(
    interior: &RidgeOptions,
    exterior: &ShellRidgeOptions,
) -> (IndexedMesh, PreviewShowing) {
    let (spec, showing) = if interior.enabled {
        (interior_spec(interior), PreviewShowing::Interior)
    } else if exterior.enabled {
        (exterior_spec(exterior), PreviewShowing::Exterior)
    } else {
        (smooth_spec(), PreviewShowing::Smooth)
    };
    let mesh = preview_textured_capsule(
        &spec,
        PREVIEW_RADIUS_M,
        PREVIEW_HALF_HEIGHT_M,
        PREVIEW_CELL_M,
    );
    (mesh, showing)
}

/// Interior canal spec from [`RidgeOptions`] — the same field mapping the cast
/// uses (rings + texture + side pinch + tip relief + orientation).
fn interior_spec(o: &RidgeOptions) -> CanalSpec {
    let mut spec = CanalSpec::iter1();
    spec.rings = o.rings.iter().map(ring_spec).collect();
    spec.texture_amp_m = o.texture_depth_m;
    spec.texture_pitch_m = o.texture_spacing_m;
    spec.dsection_depth_m = o.side_pinch_depth_m;
    spec.suction_bulge_m = o.tip_relief_depth_m;
    let theta = o.orientation_deg.to_radians();
    spec.frenulum_dir = Vector3::new(theta.sin(), theta.cos(), 0.0);
    spec
}

/// Exterior shell spec from [`ShellRidgeOptions`] — axisymmetric rings only
/// (the one-sided canal features are zeroed, matching the cast).
fn exterior_spec(o: &ShellRidgeOptions) -> CanalSpec {
    let mut spec = CanalSpec::iter1();
    spec.rings = o.rings.iter().map(ring_spec).collect();
    spec.texture_amp_m = 0.0;
    spec.dsection_depth_m = 0.0;
    spec.suction_bulge_m = 0.0;
    spec
}

/// A feature-free spec → the smooth proxy.
fn smooth_spec() -> CanalSpec {
    let mut spec = CanalSpec::iter1();
    spec.rings.clear();
    spec.texture_amp_m = 0.0;
    spec.dsection_depth_m = 0.0;
    spec.suction_bulge_m = 0.0;
    spec
}

fn ring_spec(r: &cf_studio_core::RidgeRing) -> RingSpec {
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
    fn picks_interior_then_exterior_then_smooth() {
        let on_int = RidgeOptions {
            enabled: true,
            ..RidgeOptions::default()
        };
        let on_ext = ShellRidgeOptions {
            enabled: true,
            ..ShellRidgeOptions::default()
        };
        assert_eq!(
            texture_preview_mesh(&on_int, &on_ext).1,
            PreviewShowing::Interior,
            "interior wins when both on"
        );
        assert_eq!(
            texture_preview_mesh(&RidgeOptions::default(), &on_ext).1,
            PreviewShowing::Exterior
        );
        assert_eq!(
            texture_preview_mesh(&RidgeOptions::default(), &ShellRidgeOptions::default()).1,
            PreviewShowing::Smooth
        );
    }

    #[test]
    fn every_mode_produces_a_mesh() {
        let (mesh, _) =
            texture_preview_mesh(&RidgeOptions::default(), &ShellRidgeOptions::default());
        assert!(!mesh.vertices.is_empty());
    }
}
