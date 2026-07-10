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
//! The app opens in `Design`, loads L4/L5 + a `--disc` (a stand-in until the paint
//! front-end lands next rung), and on `S` dispatches the ~90 s coupled-FSU solve to a
//! background thread (`Solving`) before replaying it (`Simulate`). The bones + camera
//! persist across modes; only the disc + the replay are mode-scoped. The visual pass is
//! user-side (this session cannot see GUI output). Run:
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
mod solve;
mod state;

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_bevy_common::prelude::{OrbitCameraPlugin, orbit_camera_input};
use cf_fsu_geometry::load;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_types::{Aabb, IndexedMesh};

use clap::Parser;
use cli::Cli;
use input::block_orbit_input_when_over_egui;
use overlays::{draw_facets, draw_ligaments};
use panel::{SceneToggles, apply_visibility, scene_panel};
use render::{SourceMeshes, leave_simulate, spawn_bones};
use replay::flexion_update;
use solve::{SolveError, SolveTask, poll_solve, start_solve};
use state::{StudioState, design_panel, quit_on_esc, simulate_back, solving_panel};

fn main() -> Result<()> {
    let cli = Cli::parse();
    let (l4_path, l5_path, disc_path) = cli::resolve_paths(&cli)?;
    // Load the three meshes into memory up front; the FSU is NOT assembled here —
    // the solve runs on demand (S) from these, on a background thread.
    let (l4, l5, disc) = (load(&l4_path)?, load(&l5_path)?, load(&disc_path)?);
    run_app(l4, l5, disc);
    Ok(())
}

/// Wire the app: keep the three source meshes resident, spawn the persistent
/// bones + camera at startup, and compose the mode-scoped systems. The sim
/// resources (`Flexion`, `Overlays`) are inserted by [`solve::poll_solve`] when a
/// solve completes — not up front.
fn run_app(l4: IndexedMesh, l5: IndexedMesh, disc: IndexedMesh) {
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
        .init_state::<StudioState>()
        .init_resource::<SolveTask>()
        .init_resource::<SolveError>()
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SourceMeshes { l4, l5, disc })
        .insert_resource(SceneToggles::default())
        // Bones + camera persist across modes (so the 3D world is never empty);
        // the disc is spawned by the solve poll and torn down on leaving Simulate.
        .add_systems(Startup, (spawn_bones, setup_camera))
        .add_systems(OnExit(StudioState::Simulate), leave_simulate)
        // Always-on: pointer arbitration.
        .add_systems(
            Update,
            block_orbit_input_when_over_egui.before(orbit_camera_input),
        )
        // Per-mode input + the solve poll.
        .add_systems(
            Update,
            (
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
