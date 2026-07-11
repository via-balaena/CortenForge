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
//! thin Bevy driver that composes the concern modules — [`cli`] (STL path resolution),
//! [`render`] (tissue entities), [`replay`] (the moment-ramp playback), [`overlays`]
//! (ligament/facet gizmos), [`panel`] (the egui side panel), [`input`] (pointer
//! arbitration), [`state`] (the mode machine), and [`solve`] (the on-demand background
//! FSU assembly) — into one app.
//!
//! The app opens in `Design`: it loads the two vertebrae and lets you **paint the
//! two endplate patches** on them (the [`cf_mesh_paint`] brush). `S` lofts the
//! patches into the disc and dispatches the ~90 s coupled-FSU solve to a
//! background thread (`Solving`), then replays it (`Simulate`). The camera
//! persists; the paint bodies (Design) and the bones+disc (Simulate) are
//! mode-scoped. The visual pass is user-side (this session cannot see GUI output).
//! Run:
//!
//! ```text
//! cargo run -p cf-spine-viewer -- --dir <dir-with-the-2-vertebra-STLs>
//! # or explicit paths:
//! cargo run -p cf-spine-viewer -- --l4 L4.stl --l5 L5.stl
//! ```
//!
//! The BodyParts3D meshes (FMA13075 = L4, FMA13076 = L5) are CC BY-SA —
//! session-local, never committed. The disc is no longer an input: it is
//! hand-painted and lofted in the Design state, closing the loop end-to-end.

mod cli;
mod input;
mod overlays;
mod panel;
mod render;
mod replay;
mod scene;
mod solve;
mod state;

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_bevy_common::prelude::update_orbit_camera;
use cf_fsu_geometry::load;
use cf_mesh_paint::prelude::{MeshPaintPlugin, orbit_when_not_painting};
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::{Aabb, IndexedMesh};

use clap::Parser;
use cli::Cli;
use input::arbitrate_pointer_over_egui;
use overlays::{draw_facets, draw_ligaments};
use panel::{SceneToggles, apply_visibility, scene_panel};
use render::{
    SourceMeshes, despawn_paint_bodies, despawn_sim_scene, spawn_bones, spawn_paint_bodies,
    update_paint_visibility,
};
use replay::flexion_update;
use solve::{SolveError, SolveTask, poll_solve, start_solve};
use state::{StudioState, design_panel, quit_on_esc, simulate_back, solving_panel};

fn main() -> Result<()> {
    let cli = Cli::parse();
    let (l4_path, l5_path) = cli::resolve_paths(&cli)?;
    // Load the two vertebrae up front; the disc is painted + lofted in Design,
    // and the FSU is assembled on demand (S) on a background thread.
    let (l4, l5) = (load(&l4_path)?, load(&l5_path)?);
    run_app(l4, l5);
    Ok(())
}

/// Wire the app: keep the two vertebra meshes resident, spawn the camera + the
/// Design paint bodies at startup, and compose the mode-scoped systems. The sim
/// resources (`Flexion`, `Overlays`) are inserted by [`solve::poll_solve`] when a
/// solve completes — not up front.
fn run_app(l4: IndexedMesh, l5: IndexedMesh) {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-spine-viewer — L4–L5 FSU (paint → solve → replay)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(MeshPaintPlugin::default())
        .init_state::<StudioState>()
        .init_resource::<SolveTask>()
        .init_resource::<SolveError>()
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SourceMeshes { l4, l5 })
        .insert_resource(SceneToggles::default())
        // Camera persists. The Design paint bodies (startup + on leaving Simulate)
        // and the Simulate bones+disc (on entering / leaving Simulate) are
        // mode-scoped — but a drawn mesh always exists where quit is reachable
        // (paint bodies in Design/Solving, bones+disc in Simulate), and the swaps
        // happen in one `StateTransition` flush, so no frame is empty (the macOS
        // wgpu quit-deadlock guard).
        .add_systems(Startup, (spawn_paint_bodies, setup_camera))
        .add_systems(OnEnter(StudioState::Simulate), (despawn_paint_bodies, spawn_bones))
        .add_systems(OnExit(StudioState::Simulate), (despawn_sim_scene, spawn_paint_bodies))
        // Always-on: egui pointer arbitration (suppresses orbit + paint over the
        // panel), then the paint-aware orbit camera.
        .add_systems(
            Update,
            (
                arbitrate_pointer_over_egui
                    .before(orbit_when_not_painting)
                    .before(cf_mesh_paint::brush::hover_ray),
                orbit_when_not_painting,
                update_orbit_camera.after(orbit_when_not_painting),
            ),
        )
        // Per-mode input + the solve poll.
        .add_systems(
            Update,
            (
                update_paint_visibility.run_if(in_state(StudioState::Design)),
                start_solve.run_if(in_state(StudioState::Design)),
                quit_on_esc
                    .run_if(in_state(StudioState::Design).or(in_state(StudioState::Solving))),
                poll_solve.run_if(in_state(StudioState::Solving)),
                simulate_back.run_if(in_state(StudioState::Simulate)),
            ),
        )
        // The replay + overlays run only in Simulate.
        .add_systems(
            Update,
            (
                flexion_update,
                apply_visibility,
                draw_ligaments.after(flexion_update),
                draw_facets.after(flexion_update),
            )
                .run_if(in_state(StudioState::Simulate)),
        )
        .add_systems(
            EguiPrimaryContextPass,
            (
                design_panel.run_if(in_state(StudioState::Design)),
                solving_panel.run_if(in_state(StudioState::Solving)),
                scene_panel.run_if(in_state(StudioState::Simulate)),
            ),
        )
        .run();
}

/// Spawn the orbit camera + lighting, framed on the combined L4/L5 AABB (the FSU
/// bounds; the disc sits between the bodies). Persists across modes.
fn setup_camera(mut commands: Commands, sources: Res<SourceMeshes>) {
    let bounds = Aabb::from_points(sources.l4.vertices.iter().chain(sources.l5.vertices.iter()));
    setup_camera_and_lighting(&mut commands, &bounds, UpAxis::PlusZ);
}
