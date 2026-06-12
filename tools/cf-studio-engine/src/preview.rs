//! Live texture-preview meshing for the wizard's two Texture steps.
//!
//! Maps the owned [`RidgeOptions`] (interior canal) / [`ShellRidgeOptions`]
//! (exterior shell rings) onto a `cf_cast::CanalSpec` and renders a coarse
//! textured proxy. Both use the same flat-floor, domed-top plug shape (the
//! shell is cast on the plug, so its outside is flat-bottomed too) — the
//! interior preview shows ridges inside the channel, the exterior on the outer
//! surface. Each updates live; the real part is cast at full detail.

use cf_cast::{CanalSpec, RingSpec, preview_textured_plug};
use cf_studio_core::{RidgeOptions, ShellRidgeOptions};
use mesh_types::IndexedMesh;
use nalgebra::Vector3;

/// Representative proxy dimensions (meters) + the coarse preview cell size.
/// A ~24 mm-diameter, 80 mm proxy reads clearly and meshes in milliseconds.
const PREVIEW_RADIUS_M: f64 = 0.012;
const PREVIEW_HALF_HEIGHT_M: f64 = 0.040;
const PREVIEW_CELL_M: f64 = 0.0015;

/// The interior-ridge preview: the canal field on a **plug** proxy (flat floor,
/// domed top). A disabled `interior` renders the smooth plug.
#[must_use]
pub fn interior_preview_mesh(interior: &RidgeOptions) -> IndexedMesh {
    let spec = if interior.enabled {
        interior_spec(interior)
    } else {
        smooth_spec()
    };
    preview_textured_plug(
        &spec,
        PREVIEW_RADIUS_M,
        PREVIEW_HALF_HEIGHT_M,
        PREVIEW_CELL_M,
    )
}

/// The exterior-ridge preview: the axisymmetric rings on the **shell's outer
/// surface**. Uses the same flat-floor plug shape as the interior preview —
/// the shell is cast on the plug, so its outside is flat-bottomed + domed too;
/// the rings just sit on the outer surface. A disabled `exterior` renders the
/// smooth shell.
#[must_use]
pub fn exterior_preview_mesh(exterior: &ShellRidgeOptions) -> IndexedMesh {
    let spec = if exterior.enabled {
        exterior_spec(exterior)
    } else {
        smooth_spec()
    };
    preview_textured_plug(
        &spec,
        PREVIEW_RADIUS_M,
        PREVIEW_HALF_HEIGHT_M,
        PREVIEW_CELL_M,
    )
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
    fn interior_and_exterior_previews_produce_meshes() {
        // Both on + off cases mesh (smooth proxy when off).
        assert!(
            !interior_preview_mesh(&RidgeOptions::default())
                .vertices
                .is_empty()
        );
        assert!(
            !interior_preview_mesh(&RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            })
            .vertices
            .is_empty()
        );
        assert!(
            !exterior_preview_mesh(&ShellRidgeOptions::default())
                .vertices
                .is_empty()
        );
        assert!(
            !exterior_preview_mesh(&ShellRidgeOptions {
                enabled: true,
                ..ShellRidgeOptions::default()
            })
            .vertices
            .is_empty()
        );
    }
}
