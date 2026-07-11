//! `cf-spine-studio` — native Bevy Design↔Simulate studio for anatomical spine models.
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
//! two endplate patches** on them (the [`cf_mesh_paint`] brush). `Enter` lofts the
//! patches into the disc and dispatches the ~5 s **build** (tet-mesh + `k_disc` probe +
//! fragmentation guards) to a background thread (`Building`), then shows the conformed
//! disc (`Preview`) — so a bad painting is caught in seconds. `S` in Preview dispatches
//! only the ~85 s moment-ramp **capture** (`Solving`) on the held build, then replays it
//! (`Simulate`); `Esc`/`D` from Preview or Simulate returns to Design to repaint. The
//! camera persists; the paint bodies (Design → Solving), the Preview disc, and the
//! bones+disc (Simulate) are mode-scoped. The visual pass is user-side (this session
//! cannot see GUI output).
//! Run:
//!
//! ```text
//! cargo run -p cf-spine-studio -- --dir <dir-with-the-2-vertebra-STLs>
//! # or explicit paths:
//! cargo run -p cf-spine-studio -- --l4 L4.stl --l5 L5.stl
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
    PaintStash, SourceMeshes, despawn_paint_bodies, despawn_preview_disc, despawn_sim_scene,
    show_paint_bodies, spawn_bones, spawn_paint_bodies, spawn_preview_disc, stash_paint,
    update_paint_visibility,
};
use replay::flexion_update;
use solve::{
    BuildTask, HeldBuildSlot, SolveError, SolveTask, discard_pending_work, poll_build, poll_solve,
    start_build, start_capture,
};
use state::{
    StudioState, back_to_design, building_panel, design_panel, preview_panel, quit_on_esc,
    solving_panel,
};

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
                title: "cf-spine-studio — L4–L5 FSU (paint → build → simulate)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(MeshPaintPlugin::default())
        .init_state::<StudioState>()
        .init_resource::<BuildTask>()
        .init_resource::<SolveTask>()
        .init_resource::<SolveError>()
        // The built-but-not-captured FSU is held here between `Enter` (build) and `S`
        // (capture). NonSend because it wraps a non-`Sync` `CoupledFsu` (see `solve`).
        .init_non_send_resource::<HeldBuildSlot>()
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SourceMeshes { l4, l5 })
        .insert_resource(SceneToggles::default())
        // Painted selections survive the Simulate round-trip via this stash.
        .init_resource::<PaintStash>()
        // Camera persists. The Design paint bodies (startup + on leaving Simulate), the
        // Preview disc (on entering Preview), and the Simulate bones+disc (on entering /
        // leaving Simulate) are mode-scoped — but a drawn mesh always exists where quit is
        // reachable (paint bodies in Design/Building/Preview/Solving, bones+disc in
        // Simulate), and the swaps happen in one `StateTransition` flush, so no frame is
        // empty (the macOS wgpu quit-deadlock guard).
        .add_systems(Startup, (spawn_paint_bodies, setup_camera))
        // Returning to Design: drop the Preview disc + any held build / in-flight task
        // (repaint from scratch).
        .add_systems(
            OnEnter(StudioState::Design),
            (despawn_preview_disc, discard_pending_work),
        )
        // The build succeeded: show the conformed disc + both (un-hidden) vertebrae.
        .add_systems(
            OnEnter(StudioState::Preview),
            (spawn_preview_disc, show_paint_bodies),
        )
        // Stash the painting before the paint bodies are torn down, so `spawn_paint_bodies`
        // can restore it on the way back (OnExit).
        .add_systems(
            OnEnter(StudioState::Simulate),
            (
                stash_paint.before(despawn_paint_bodies),
                despawn_paint_bodies,
                spawn_bones,
                despawn_preview_disc,
            ),
        )
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
        // Per-mode input + the build/capture polls.
        .add_systems(
            Update,
            (
                update_paint_visibility.run_if(in_state(StudioState::Design)),
                start_build.run_if(in_state(StudioState::Design)),
                poll_build.run_if(in_state(StudioState::Building)),
                start_capture.run_if(in_state(StudioState::Preview)),
                poll_solve.run_if(in_state(StudioState::Solving)),
                back_to_design
                    .run_if(in_state(StudioState::Preview).or(in_state(StudioState::Simulate))),
                quit_on_esc.run_if(
                    in_state(StudioState::Design)
                        .or(in_state(StudioState::Building))
                        .or(in_state(StudioState::Solving)),
                ),
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
                building_panel.run_if(in_state(StudioState::Building)),
                preview_panel.run_if(in_state(StudioState::Preview)),
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
