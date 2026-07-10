//! `cf-spine-viewer` — native Bevy viewer for anatomical spine models.
//!
//! **Coupled contact FSU scene (this rung):** renders the literature-validated
//! L4–L5 Functional Spinal Unit (geometry ladder rung 7) as a *coupled, moment-driven
//! simulation* — the two real vertebrae with a per-tissue bone material, the
//! intervertebral disc as a clean deforming surface, and the field-derived ligament
//! lines + facet contacts that ride the superior vertebra. A headless capture
//! (`cf_fsu_model::CoupledFsu::capture_ramp`) sweeps an applied pure moment from
//! extension to flexion and solves the coupled equilibrium (disc bushing + ligament
//! tendons + oriented facet contact) at each level; the Bevy loop then replays it —
//! L5 fixed, L4 rotating to its solved equilibrium angle about the disc-centre pivot,
//! the disc deforming in lockstep — with an egui timeline. The motion is **force-driven
//! and ROM-limited**: L4 stops on the facets in extension (the facets glow when engaged)
//! and the disc/ligaments limit flexion, so the bones no longer interpenetrate.
//!
//! ## Disc rendering
//!
//! The FEM tet mesh is too fragmented to draw as a coherent disc (the BCC
//! isosurface-stuffing mesher shatters the thin lens), so the viewer renders the
//! clean STL surface (endplate bands conformed onto the real bone via
//! `CoupledFsu::conformed_disc_surface`) and displaces each of its vertices by the FEM
//! displacement field — sampled from the nearest tet nodes, inverse-distance-squared weighted (`scene::weighted_tet_nodes`).
//! Clean geometry from the STL, real deformation from the physics.
//!
//! ## Honesty
//!
//! The rigid-body ROM is the real solved equilibrium (no exaggeration). The disc's
//! deformation at each ROM angle is a genuine incremental FEM solve: the bonded disc is
//! warm-started through sub-degree steps to the target angle and the real deformed tet
//! nodes are read back (`CoupledFsu::capture_ramp`). There is NO
//! extrapolation — the "sub-degree convergence wall" was a from-rest-jump artifact, and a
//! fine monotone sweep reaches the full ±ROM cleanly. (The equilibrium *angle* itself
//! still comes from the linear `k_disc` bushing rung 7 validated.) The panel reports the
//! applied moment, the solved angle, and the facet engagement.
//!
//! ## Module map
//!
//! The headless scene assembly + capture live in [`scene`] (Bevy-free); this file is the
//! thin Bevy driver that composes the concern modules — [`render`] (tissue entities),
//! [`replay`] (the moment-ramp playback), [`overlays`] (ligament/facet gizmos), [`panel`]
//! (the egui side panel), and [`input`] (pointer arbitration + quit) — into one app. The
//! visual pass is user-side (this session cannot see GUI output). Run:
//!
//! ```text
//! cargo run -p cf-spine-viewer -- --dir <dir-with-the-3-STLs>
//! # or explicit paths:
//! cargo run -p cf-spine-viewer -- --l4 L4.stl --l5 L5.stl --disc DISC.stl
//! ```
//!
//! The BodyParts3D meshes (FMA13075 = L4, FMA13076 = L5, FMA16036 = disc)
//! are CC BY-SA — session-local, never committed.
//!
//! `--disc` accepts any watertight disc STL in the same native-mm frame — in
//! particular a disc *painted and lofted* in the `paint-faces` tool
//! (`sim-bevy-soft`), which prints this exact command on export. That closes the
//! loop: hand-paint the two endplate patches, loft the bushing, and watch it
//! deform between the real vertebrae here.

mod cli;
mod input;
mod overlays;
mod panel;
mod render;
mod replay;
mod scene;

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_bevy_common::prelude::{OrbitCameraPlugin, orbit_camera_input};
use cf_viewer::{UpAxis, setup_camera_and_lighting};

use clap::Parser;
use cli::Cli;
use input::{block_orbit_input_when_over_egui, exit_on_esc};
use overlays::{Overlays, draw_facets, draw_ligaments};
use panel::{SceneToggles, apply_visibility, scene_panel};
use render::{SceneMeshes, setup_scene};
use replay::{Flexion, flexion_update};
use scene::FsuScene;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let (l4, l5, disc) = cli::resolve_paths(&cli)?;
    let fsu = scene::build(&l4, &l5, &disc)?;
    run_app(fsu);
    Ok(())
}

/// Wire the assembled FSU scene into a Bevy app: inject the scene data as
/// resources, then compose the concern modules' systems into the schedule.
fn run_app(fsu: FsuScene) {
    let FsuScene {
        l4,
        l5,
        disc_surface,
        disc_node_weights,
        o4,
        o5,
        flexion,
        ligaments,
        aabb,
        warnings,
    } = fsu;
    let disc_rest = disc_surface.vertices.clone();
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-spine-viewer — L4–L5 FSU (coupled contact sim)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SceneMeshes {
            l4,
            l5,
            disc_surface,
        })
        .insert_resource(Overlays {
            ligaments,
            aabb,
            warnings,
        })
        .insert_resource(Flexion {
            // Start at the neutral (middle) frame so the FSU opens at rest and
            // eases into flexion rather than snapping to a full tilt on frame 1.
            cursor: (flexion.frames.len().saturating_sub(1)) as f32 / 2.0,
            traj: flexion,
            disc_rest,
            disc_weights: disc_node_weights,
            o4,
            o5,
            playing: true,
            direction: 1.0,
            true_theta: 0.0,
            applied: 0.0,
            facet_frame: None,
            n_facet: 0,
        })
        .insert_resource(SceneToggles::default())
        .add_systems(Startup, (setup_scene, setup_camera))
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                flexion_update,
                apply_visibility,
                draw_ligaments.after(flexion_update),
                draw_facets.after(flexion_update),
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, scene_panel)
        .run();
}

/// Spawn the orbit camera + lighting, framed on the combined scene AABB. Kept
/// separate from the tissue spawn ([`render::setup_scene`]) so the camera
/// persists independently of the scene entities.
fn setup_camera(mut commands: Commands, overlays: Res<Overlays>) {
    // Native mm — frame directly on the combined AABB. `setup_camera_and_lighting`
    // swaps the target into Bevy Y-up, matching the swapped meshes.
    setup_camera_and_lighting(&mut commands, &overlays.aabb, UpAxis::PlusZ);
}
