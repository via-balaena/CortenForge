//! The coupled-ramp replay: one shared cursor advances the captured
//! moment-driven sweep, driving L4's rotation about the disc-centre pivot and
//! the disc's FEM deformation in exact lockstep — the real force-driven motion,
//! not a kinematic exaggeration.

use bevy::prelude::*;
use bevy::time::Real;
use cf_fsu_geometry::MeshOracle;
use cf_fsu_model::CoupledTrajectory;
use cf_viewer::UpAxis;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_point};
use sim_bevy_soft::mesh::apply_soft_positions;

use crate::render::{DiscMesh, FlexedL4};

/// Playback speed in captured-frames per second; interpolation keeps it smooth.
/// The motion is the REAL coupled ROM (flexion ~6°, extension ~4.5° stopping on the
/// facets) — force-driven and ROM-limited, so no exaggeration is applied: L4 rotates by
/// the solved equilibrium angle and the disc deforms by the real FEM field at that angle
/// (interpolated between incrementally-solved captured frames, not extrapolated). The
/// panel reports the applied moment, the true angle, and the
/// facet engagement so the "bones stop on the facets" is legible.
const PLAYBACK_FPS: f32 = 6.0;

/// The captured coupled ramp + its live replay state. One shared cursor drives the
/// disc deformation AND L4's rotation from the solved equilibria, so they stay in exact
/// lockstep — the real force-driven motion, not a kinematic exaggeration.
#[derive(Resource)]
pub(crate) struct Flexion {
    pub(crate) traj: CoupledTrajectory,
    /// The clean STL disc's rest vertex positions (native mm) — the render surface.
    pub(crate) disc_rest: Vec<Point3<f64>>,
    /// For each `disc_rest` vertex, the nearest tet nodes in `traj.rest_nodes_native` with
    /// inverse-distance-squared weights (see `scene::weighted_tet_nodes`), so the disc's real FEM
    /// displacement is skinned smoothly (C⁰) onto the clean surface each frame.
    pub(crate) disc_weights: Vec<Vec<(usize, f64)>>,
    /// L4/L5 oracles — each frame, a deformed disc vertex that lands inside a bone is projected
    /// back onto its surface (the FEM annulus bulges freely and would pierce the compression-side
    /// vertebra; the bone stops it). See [`crate::scene::FsuScene::o4`].
    pub(crate) o4: MeshOracle,
    pub(crate) o5: MeshOracle,
    /// Auto-advancing playback (vs. paused / hand-scrubbed).
    pub(crate) playing: bool,
    /// Continuous frame position in `[0, frames-1]`; interpolated for smoothness.
    pub(crate) cursor: f32,
    /// Ping-pong direction: `+1` flexing forward, `-1` returning.
    pub(crate) direction: f32,
    /// Interpolated flexion angle at the cursor (rad) — the solved equilibrium angle.
    /// Written each frame; read by the overlays + panel.
    pub(crate) true_theta: f64,
    /// Interpolated applied moment at the cursor (N·m, +flexion). Panel readout.
    pub(crate) applied: f64,
    /// The captured frame whose facet contacts to show, or `None` when the cursor is not
    /// squarely inside the engaged region (open, or an ambiguous transition). Computed ONCE
    /// per frame (via `engaged_facet_frame`) and read by BOTH the panel count and
    /// `draw_facets`, so they can never disagree.
    pub(crate) facet_frame: Option<usize>,
    /// Engaged facet-contact count for the panel — `facet_frame`'s point count (0 if `None`).
    pub(crate) n_facet: usize,
}

/// The captured frame to source facet contacts from at `cursor`: the lower bracketing
/// frame, but only when BOTH bracketing frames are engaged (so nothing shows at an
/// ambiguous inter-frame transition — the pose interpolates but contacts are per-frame).
/// The single definition shared by the panel readout and `draw_facets`.
fn engaged_facet_frame(traj: &CoupledTrajectory, cursor: f32) -> Option<usize> {
    let n = traj.frames.len();
    if n == 0 {
        return None;
    }
    let lo = (cursor.floor().clamp(0.0, (n - 1) as f32)) as usize;
    let hi = (lo + 1).min(n - 1);
    (!traj.frames[lo].facet_points.is_empty() && !traj.frames[hi].facet_points.is_empty())
        .then_some(lo)
}

/// Advance the replay cursor and drive BOTH the disc deformation and L4's rotation from
/// it, in lockstep. The motion is the solved coupled equilibrium: L4 rotates by the true
/// angle `θ` about the disc-centre pivot, and the disc surface is displaced by the FEM
/// field (`rest + (deformed − rest)`) at that angle — no exaggeration.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn flexion_update(
    time: Res<Time<Real>>,
    mut flexion: ResMut<Flexion>,
    mut meshes: ResMut<Assets<Mesh>>,
    // Reused across frames (cleared + refilled in place) to avoid a per-frame heap alloc.
    mut flat: Local<Vec<f64>>,
    // The cursor last rendered — lets us skip the whole rebuild while idle.
    mut last_cursor: Local<Option<f32>>,
    q_disc: Query<&Mesh3d, With<DiscMesh>>,
    mut q_l4: Query<&mut Transform, With<FlexedL4>>,
) {
    let n = flexion.traj.frames.len();
    if n == 0 {
        return;
    }
    let max = (n - 1) as f32;

    // Advance the ping-pong cursor while playing (reflecting at both ends).
    if flexion.playing && n > 1 {
        let mut c = flexion.cursor + flexion.direction * PLAYBACK_FPS * time.delta_secs();
        let mut dir = flexion.direction;
        while c < 0.0 || c > max {
            if c < 0.0 {
                c = -c;
                dir = 1.0;
            } else {
                c = 2.0 * max - c;
                dir = -1.0;
            }
        }
        flexion.cursor = c;
        flexion.direction = dir;
    }

    // Skip the (expensive) disc-mesh rebuild + normal recompute + GPU upload when the
    // pose is unchanged since last frame — e.g. paused with no scrub.
    if *last_cursor == Some(flexion.cursor) {
        return;
    }
    *last_cursor = Some(flexion.cursor);

    // Bracketing frames + fraction for interpolation.
    let c = flexion.cursor.clamp(0.0, max);
    let lo = c.floor() as usize;
    let hi = (lo + 1).min(n - 1);
    let frac = f64::from(c - lo as f32);
    let pivot = flexion.traj.pivot;
    let axis = flexion.traj.axis;

    // Build the deformed disc surface into the reused scratch buffer: each clean-surface vertex
    // is displaced by the disc's REAL FEM displacement (`deformed − rest`), skinned smoothly
    // (inverse-distance-squared blend of the nearest tet nodes, interpolated between frames). The
    // deformation is a genuine incremental solve, so no carrier or extrapolation is needed — but
    // the FEM bonds to boxes, not the bones, so its annulus can bulge INTO the compression-side
    // vertebra; a final projection pushes any vertex inside a bone back onto that surface (the
    // bone stops the bulge). `flat` (a system Local) is independent of `flexion`, so filling it
    // while borrowing the trajectory is fine.
    flat.clear();
    let (true_theta, applied) = {
        let traj = &flexion.traj;
        let (fa, fb) = (&traj.frames[lo], &traj.frames[hi]);
        let theta = fa.theta + (fb.theta - fa.theta) * frac;
        let applied = fa.applied + (fb.applied - fa.applied) * frac;
        // L4 rotates by θ this frame (project into its rotated frame); L5 is fixed.
        let l4_rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), theta);
        for (dr, weights) in flexion.disc_rest.iter().zip(&flexion.disc_weights) {
            let mut disp = Vector3::zeros();
            for &(j, w) in weights {
                let a = fa.deformed_nodes_native[j];
                let b = fb.deformed_nodes_native[j];
                disp += ((a + (b - a) * frac) - traj.rest_nodes_native[j]) * w;
            }
            let mut p = dr + disp;
            // Non-penetration: project out of rotated-L4 (un-rotate → nearest surface → re-rotate)
            // or fixed-L5, so the disc never pierces a bone.
            let l4_local = pivot + l4_rot.inverse() * (p - pivot);
            if flexion.o4.evaluate(l4_local) < 0.0 {
                p = pivot + l4_rot * (flexion.o4.closest_point(l4_local) - pivot);
            }
            if flexion.o5.evaluate(p) < 0.0 {
                p = flexion.o5.closest_point(p);
            }
            flat.push(p.x);
            flat.push(p.y);
            flat.push(p.z);
        }
        (theta, applied)
    };
    flexion.true_theta = true_theta;
    flexion.applied = applied;
    // Compute the engaged facet frame ONCE (the both-bracketing-frames-engaged gate); the
    // panel count and `draw_facets` both read this, so they can never disagree.
    flexion.facet_frame = engaged_facet_frame(&flexion.traj, flexion.cursor);
    flexion.n_facet = flexion
        .facet_frame
        .map_or(0, |lo| flexion.traj.frames[lo].facet_points.len());

    // Rewrite the disc surface (Z-up→Y-up swap + smooth-normal recompute).
    if let Ok(handle) = q_disc.single() {
        if let Some(mesh) = meshes.get_mut(&handle.0) {
            apply_soft_positions(mesh, &flat, UpAxis::PlusZ);
        }
    }

    // Rotate L4 by the true angle about the pivot. The Y/Z swap is a reflection, so
    // `quat_from_unit_quaternion` gives the swap-consistent rotation R; rotating about
    // pivot p is `R` with translation `p − R·p`.
    let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), true_theta);
    let r_bevy = quat_from_unit_quaternion(&rot);
    let p_bevy = vec3_from_point(&pivot);
    if let Ok(mut tf) = q_l4.single_mut() {
        tf.rotation = r_bevy;
        tf.translation = p_bevy - r_bevy * p_bevy;
    }
}

/// The physics-space rotation that maps a rest point to its displayed pose:
/// `pivot + R(axis, θ)·(p − pivot)`. Shared by the overlay systems so L4-attached sites
/// track the vertebra exactly.
pub(crate) fn display_rotation(flexion: &Flexion) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Unit::new_normalize(flexion.traj.axis), flexion.true_theta)
}

/// Rotate a native-mm point about the flexion `pivot` by `rot` — the `pivot + R·(p−pivot)`
/// idiom the L4-riding overlays share.
pub(crate) fn pose_about_pivot(
    p: &Point3<f64>,
    rot: &UnitQuaternion<f64>,
    pivot: Point3<f64>,
) -> Point3<f64> {
    pivot + rot * (p - pivot)
}
