//! Gizmo overlays that ride the flexing vertebra: the two ligament lines (with
//! attachment-site markers) and the engaged facet-contact points.

use bevy::prelude::*;
use nalgebra::{Unit, UnitQuaternion};
use sim_bevy::convert::vec3_from_point;

use crate::panel::SceneToggles;
use crate::replay::{Flexion, display_rotation, pose_about_pivot};
use crate::scene::Ligament;

const LIGAMENT_COLOR: Color = Color::srgb(0.86, 0.74, 0.42); // fibrous tan
const FACET_COLOR: Color = Color::srgb(0.90, 0.28, 0.22); // articular-contact red (engaged)

// Overlay glyph radii, in native millimetres.
const SITE_RADIUS: f32 = 1.5; // ligament attachment marker
const FACET_RADIUS: f32 = 0.8; // facet near-contact marker

/// The lightweight overlay data the per-frame systems + panel read for the
/// whole session (the bone meshes are gone by then).
#[derive(Resource)]
pub(crate) struct Overlays {
    pub(crate) ligaments: Vec<Ligament>,
    pub(crate) warnings: Vec<String>,
}

/// Draw the ligament lines (ALL, ISP) as tan polylines with site markers. The
/// superior (L4) attachment rides the flexing vertebra; the inferior (L5) is fixed.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn draw_ligaments(
    mut gizmos: Gizmos,
    overlays: Res<Overlays>,
    toggles: Res<SceneToggles>,
    flexion: Res<Flexion>,
) {
    if !toggles.ligaments {
        return;
    }
    let rot = display_rotation(&flexion);
    let pivot = flexion.traj.pivot;
    for lig in &overlays.ligaments {
        let superior = pose_about_pivot(&lig.superior, &rot, pivot); // rides L4
        let (a, b) = (vec3_from_point(&lig.inferior), vec3_from_point(&superior));
        gizmos.line(a, b, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(a), SITE_RADIUS, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(b), SITE_RADIUS, LIGAMENT_COLOR);
    }
}

/// Draw the coupled solve's real engaged facet contact points — where the bones actually
/// touch — as red spheres. Sourced from `flexion.facet_frame` (the shared engaged-frame gate
/// `flexion_update` computed, so the drawn markers always match the panel count): nothing at
/// an ambiguous transition or in flexion. The frame's points are re-posed by the residual
/// rotation to the interpolated body angle so they stay glued to the articulation.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn draw_facets(mut gizmos: Gizmos, toggles: Res<SceneToggles>, flexion: Res<Flexion>) {
    if !toggles.facets {
        return;
    }
    let Some(lo) = flexion.facet_frame else {
        return;
    };
    let pivot = flexion.traj.pivot;
    // Residual rotation from the source frame's angle to the interpolated body angle.
    let delta = flexion.true_theta - flexion.traj.frames[lo].theta;
    let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(flexion.traj.axis), delta);
    for p in &flexion.traj.frames[lo].facet_points {
        let posed = pose_about_pivot(p, &rot, pivot);
        gizmos.sphere(
            Isometry3d::from_translation(vec3_from_point(&posed)),
            FACET_RADIUS * 1.4,
            FACET_COLOR,
        );
    }
}
