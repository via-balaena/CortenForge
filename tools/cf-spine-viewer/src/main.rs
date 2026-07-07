//! `cf-spine-viewer` — native Bevy viewer for anatomical spine models.
//!
//! **Static FSU scene (this rung):** renders the assembled, literature-
//! validated L4–L5 Functional Spinal Unit (geometry ladder rung 7) as a
//! proper anatomical scene — the two real vertebrae with a per-tissue bone
//! material, the intervertebral disc, the field-derived ligament lines
//! (anterior longitudinal + interspinous), and the facet near-contact
//! points — all in native BodyParts3D millimetres, orbitable.
//!
//! It turns the rung-7 geometry (previously only numbers in test output)
//! into something inspectable, and forces the reusable pieces every later
//! anatomical viz needs (multi-tissue scene, ligament/contact overlays).
//! No deformation this rung — that (flexion playback) is next.
//!
//! The headless scene assembly lives in [`scene`] (Bevy-free); this file is
//! the thin Bevy/egui driver. The visual pass is user-side (this session
//! cannot see GUI output). Run:
//!
//! ```text
//! cargo run -p cf-spine-viewer -- --dir <dir-with-the-3-STLs>
//! # or explicit paths:
//! cargo run -p cf-spine-viewer -- --l4 L4.stl --l5 L5.stl --disc DISC.stl
//! ```
//!
//! The BodyParts3D meshes (FMA13075 = L4, FMA13076 = L5, FMA16036 = disc)
//! are CC BY-SA — session-local, never committed.

mod scene;

use std::path::PathBuf;

use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::{OrbitCameraPlugin, orbit_camera_input};
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use clap::Parser;
use mesh_types::{Aabb, AttributedMesh, IndexedMesh};
use nalgebra::Point3;
use scene::{FsuScene, Ligament};
use sim_bevy::convert::vec3_from_point;
use sim_bevy::mesh::triangle_mesh_from_attributed;

// ── Per-tissue palette (a serious anatomical read, not toy colors). ──
const BONE_COLOR: Color = Color::srgb(0.90, 0.88, 0.82); // ivory cortical bone
const DISC_COLOR: Color = Color::srgba(0.20, 0.62, 0.68, 0.55); // translucent teal cartilage
const LIGAMENT_COLOR: Color = Color::srgb(0.86, 0.74, 0.42); // fibrous tan
const FACET_COLOR: Color = Color::srgb(0.90, 0.28, 0.22); // articular-contact red

// Overlay glyph radii, in native millimetres.
const SITE_RADIUS: f32 = 1.5; // ligament attachment marker
const FACET_RADIUS: f32 = 0.8; // facet near-contact marker

/// Render the assembled static L4–L5 FSU in a native Bevy window.
#[derive(Parser)]
#[command(name = "cf-spine-viewer")]
struct Cli {
    /// Directory holding the three BodyParts3D STLs, named by FMA id
    /// (FMA13075 = L4, FMA13076 = L5, FMA16036 = disc). Native mm.
    #[arg(long)]
    dir: Option<PathBuf>,
    /// Explicit L4 STL path (overrides `--dir`).
    #[arg(long)]
    l4: Option<PathBuf>,
    /// Explicit L5 STL path (overrides `--dir`).
    #[arg(long)]
    l5: Option<PathBuf>,
    /// Explicit disc STL path (overrides `--dir`).
    #[arg(long)]
    disc: Option<PathBuf>,
}

/// Which tissue an entity is, so the visibility panel can toggle it.
#[derive(Component, Clone, Copy)]
enum TissuePart {
    L4,
    L5,
    Disc,
}

/// The tissue meshes — consumed once at startup to build the GPU mesh assets,
/// then removed (freed) so a second full copy of the geometry does not sit
/// resident for the whole session.
#[derive(Resource)]
struct SceneMeshes {
    l4: IndexedMesh,
    l5: IndexedMesh,
    disc: IndexedMesh,
}

/// The lightweight overlay data the per-frame systems + panel read for the
/// whole session (the meshes are gone by then).
#[derive(Resource)]
struct Overlays {
    ligaments: Vec<Ligament>,
    facet_contacts: Vec<Point3<f64>>,
    aabb: Aabb,
    warnings: Vec<String>,
}

/// Per-tissue visibility flags, driven by the egui panel.
#[derive(Resource)]
struct SceneToggles {
    l4: bool,
    l5: bool,
    disc: bool,
    ligaments: bool,
    facets: bool,
}

impl Default for SceneToggles {
    fn default() -> Self {
        Self {
            l4: true,
            l5: true,
            disc: true,
            ligaments: true,
            facets: true,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let (l4, l5, disc) = resolve_paths(&cli)?;
    let fsu = scene::build(&l4, &l5, &disc)?;
    run_app(fsu);
    Ok(())
}

/// Resolve the three STL paths from `--dir` (by FMA id) or explicit flags.
fn resolve_paths(cli: &Cli) -> Result<(PathBuf, PathBuf, PathBuf)> {
    let from_dir = |id: &str| cli.dir.as_ref().map(|d| d.join(format!("FMA{id}.stl")));
    let l4 = cli
        .l4
        .clone()
        .or_else(|| from_dir("13075"))
        .context("provide --l4 <path> or --dir <dir with FMA13075.stl>")?;
    let l5 = cli
        .l5
        .clone()
        .or_else(|| from_dir("13076"))
        .context("provide --l5 <path> or --dir <dir with FMA13076.stl>")?;
    let disc = cli
        .disc
        .clone()
        .or_else(|| from_dir("16036"))
        .context("provide --disc <path> or --dir <dir with FMA16036.stl>")?;
    Ok((l4, l5, disc))
}

fn run_app(fsu: FsuScene) {
    let FsuScene {
        l4,
        l5,
        disc,
        ligaments,
        facet_contacts,
        aabb,
        warnings,
    } = fsu;
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-spine-viewer — L4–L5 FSU (static)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SceneMeshes { l4, l5, disc })
        .insert_resource(Overlays {
            ligaments,
            facet_contacts,
            aabb,
            warnings,
        })
        .insert_resource(SceneToggles::default())
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                apply_visibility,
                draw_ligaments,
                draw_facets,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, scene_panel)
        .run();
}

/// Build a SMOOTH-shaded Bevy mesh from an indexed surface: area-weighted
/// per-vertex normals (`AttributedMesh::compute_normals`) rendered by the
/// shared attributed converter (f64→f32 + Z-up→Y-up handled internally).
fn smooth_mesh(indexed: &IndexedMesh) -> Mesh {
    let mut attributed = AttributedMesh::from(indexed.clone());
    attributed.compute_normals();
    triangle_mesh_from_attributed(&attributed)
}

/// Spawn one tissue surface with its material + a visibility-toggle marker.
///
/// A deliberate generalization of `sim_bevy::mesh::spawn_design_mesh` (same
/// double-sided / `cull_mode: None` StandardMaterial base) — that helper takes
/// no alpha mode, so it can't render the translucent disc, and attaches no
/// toggle marker. Rather than split into two spawn paths, this is the single
/// per-tissue spawner (opaque bone + translucent disc + `TissuePart`).
fn spawn_tissue(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    indexed: &IndexedMesh,
    base_color: Color,
    part: TissuePart,
) {
    let translucent = base_color.alpha() < 1.0;
    let material = StandardMaterial {
        base_color,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        alpha_mode: if translucent {
            AlphaMode::Blend
        } else {
            AlphaMode::Opaque
        },
        ..default()
    };
    commands.spawn((
        Mesh3d(meshes.add(smooth_mesh(indexed))),
        MeshMaterial3d(materials.add(material)),
        Transform::default(),
        Visibility::Visible,
        part,
    ));
}

/// Spawn the three tissue meshes + frame the orbit camera on the whole FSU,
/// then drop the CPU meshes (they now live as GPU mesh assets).
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene_meshes: Res<SceneMeshes>,
    overlays: Res<Overlays>,
) {
    spawn_tissue(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l4,
        BONE_COLOR,
        TissuePart::L4,
    );
    spawn_tissue(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l5,
        BONE_COLOR,
        TissuePart::L5,
    );
    spawn_tissue(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.disc,
        DISC_COLOR,
        TissuePart::Disc,
    );

    // Native mm — the parts are already >1 unit, so no render-scale lift is
    // needed; frame directly on the combined AABB. `setup_camera_and_lighting`
    // (via `OrbitCamera::framing_for_aabb`) swaps the target into Bevy Y-up,
    // matching the swapped meshes.
    setup_camera_and_lighting(&mut commands, &overlays.aabb, UpAxis::PlusZ);

    // The meshes are now GPU assets; free the CPU-side copies for the session.
    commands.remove_resource::<SceneMeshes>();
}

/// Toggle each tissue mesh's visibility from the panel flags.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn apply_visibility(toggles: Res<SceneToggles>, mut query: Query<(&TissuePart, &mut Visibility)>) {
    for (part, mut vis) in &mut query {
        let show = match part {
            TissuePart::L4 => toggles.l4,
            TissuePart::L5 => toggles.l5,
            TissuePart::Disc => toggles.disc,
        };
        *vis = if show {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Draw the ligament lines (ALL, ISP) as tan polylines with site markers.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_ligaments(mut gizmos: Gizmos, overlays: Res<Overlays>, toggles: Res<SceneToggles>) {
    if !toggles.ligaments {
        return;
    }
    for lig in &overlays.ligaments {
        let (a, b) = (
            vec3_from_point(&lig.inferior),
            vec3_from_point(&lig.superior),
        );
        gizmos.line(a, b, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(a), SITE_RADIUS, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(b), SITE_RADIUS, LIGAMENT_COLOR);
    }
}

/// Draw the facet near-contact points as small red spheres.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_facets(mut gizmos: Gizmos, overlays: Res<Overlays>, toggles: Res<SceneToggles>) {
    if !toggles.facets {
        return;
    }
    for p in &overlays.facet_contacts {
        gizmos.sphere(
            Isometry3d::from_translation(vec3_from_point(p)),
            FACET_RADIUS,
            FACET_COLOR,
        );
    }
}

/// Right-side panel: per-tissue visibility toggles + any co-registration
/// warnings (so a misaligned/mismatched trio is never shown as a valid FSU).
fn scene_panel(
    mut contexts: EguiContexts,
    mut toggles: ResMut<SceneToggles>,
    overlays: Res<Overlays>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("fsu-panel")
        .resizable(false)
        .default_width(230.0)
        .show(ctx, |ui| {
            ui.heading("L4–L5 FSU");
            ui.label("geometry ladder rung 7 (static)");
            ui.separator();
            ui.label("Show:");
            ui.checkbox(&mut toggles.l4, "L4 vertebra");
            ui.checkbox(&mut toggles.l5, "L5 vertebra");
            ui.checkbox(&mut toggles.disc, "Intervertebral disc");
            ui.checkbox(&mut toggles.ligaments, "Ligaments");
            ui.checkbox(&mut toggles.facets, "Facet near-contacts");
            ui.separator();
            // Derive the ligament readout from what was actually built — a
            // degenerate mesh can drop one, and the panel must not claim it.
            let names: Vec<&str> = overlays.ligaments.iter().map(|l| l.name).collect();
            ui.label(format!(
                "{} ligaments ({})",
                overlays.ligaments.len(),
                names.join(", ")
            ));
            ui.label(format!(
                "{} facet near-contact points",
                overlays.facet_contacts.len()
            ));
            ui.label("LMB orbit · RMB pan · scroll zoom · Esc quit");
            if !overlays.warnings.is_empty() {
                ui.separator();
                for w in &overlays.warnings {
                    ui.colored_label(egui::Color32::from_rgb(230, 140, 60), format!("⚠ {w}"));
                }
            }
        });
    Ok(())
}

/// Suppress orbit-camera input while the pointer is over the egui panel.
fn block_orbit_input_when_over_egui(
    mut contexts: EguiContexts,
    mut motion: ResMut<bevy::input::mouse::AccumulatedMouseMotion>,
    mut scroll: ResMut<bevy::input::mouse::AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    if ctx.wants_pointer_input() || ctx.is_pointer_over_area() {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}

/// Esc quits the window (mirrors the other `tools/` viewers).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
