//! `cf-spine-viewer` — native Bevy viewer for anatomical spine models.
//!
//! **Flexion-deformation FSU scene (this rung):** renders the literature-
//! validated L4–L5 Functional Spinal Unit (geometry ladder rung 7) *flexing*
//! — the two real vertebrae with a per-tissue bone material, the
//! intervertebral disc as its real deforming FEM tet-boundary surface, and
//! the field-derived ligament lines + facet near-contact points that ride the
//! superior vertebra. A headless capture (`cf_fsu_model::capture_flexion`)
//! solves a sub-degree flexion sweep of the bonded disc once at startup; the
//! Bevy loop then replays it — L5 fixed, L4 rotating about the disc-centre
//! pivot, the disc deforming in lockstep — with an egui timeline and a
//! labelled "deformation ×N" exaggeration slider.
//!
//! ## Honesty
//!
//! The bonded disc converges only at **sub-degree** strains (~0.86° before the
//! boundary tets leave their SPD region — rung 7), so the real deformation is
//! sub-millimetre. The ×N slider exaggerates it (and L4's rotation, coherently)
//! for legibility; the panel always reports the **true** physical angle. The
//! disc is the real `cell`-resolution tet boundary — visibly faceted, not the
//! smooth STL shell of the static rung.
//!
//! The headless scene assembly + capture live in [`scene`] (Bevy-free); this
//! file is the thin Bevy/egui driver. The visual pass is user-side (this
//! session cannot see GUI output). Run:
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
use bevy::time::Real;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::{OrbitCameraPlugin, orbit_camera_input};
use cf_fsu_model::FlexionTrajectory;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use clap::Parser;
use mesh_types::{Aabb, AttributedMesh, IndexedMesh};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use scene::{FsuScene, Ligament};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_point};
use sim_bevy::mesh::triangle_mesh_from_attributed;
use sim_bevy_soft::mesh::{apply_soft_positions, build_soft_mesh};

// ── Per-tissue palette (a serious anatomical read, not toy colors). ──
const BONE_COLOR: Color = Color::srgb(0.90, 0.88, 0.82); // ivory cortical bone
const DISC_COLOR: Color = Color::srgb(0.20, 0.62, 0.68); // teal cartilage (opaque — see disc material)
const LIGAMENT_COLOR: Color = Color::srgb(0.86, 0.74, 0.42); // fibrous tan
const FACET_COLOR: Color = Color::srgb(0.90, 0.28, 0.22); // articular-contact red

// Overlay glyph radii, in native millimetres.
const SITE_RADIUS: f32 = 1.5; // ligament attachment marker
const FACET_RADIUS: f32 = 0.8; // facet near-contact marker

// ── Flexion replay ──
/// Default deformation exaggeration. The real flexion is sub-degree (≤0.86°) and
/// the disc's deformation is sub-mm, so we exaggerate for legibility — but only
/// modestly: the disc is ~6 mm thick with coarse ~3 mm tets, and scaling node
/// displacements past ~half the disc thickness inverts those tets (shatters the
/// surface), while scaling L4's rigid rotation past the real ROM drives it through
/// L5 (there is no contact in this kinematic replay). ×4 ≈ 3.4° apparent keeps
/// both well inside the safe range; the panel always reports the TRUE angle.
const DEFORM_SCALE_DEFAULT: f32 = 4.0;
/// Slider ceiling — ×6 ≈ 5.2°, about the segment's physiological flexion ROM
/// (rung 7 measured ~6°); beyond this the thin coarse disc starts to self-cross.
const DEFORM_SCALE_MAX: f32 = 6.0;
/// Playback speed in captured-frames per second; interpolation keeps it smooth.
const PLAYBACK_FPS: f32 = 6.0;

/// Render the assembled L4–L5 FSU flexing in a native Bevy window.
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

/// Marks the superior (L4) vertebra — the one flexion rotates about the pivot.
#[derive(Component)]
struct FlexedL4;

/// Marks the deforming disc soft-mesh entity (its positions are rewritten each frame).
#[derive(Component)]
struct DiscMesh;

/// The bone meshes — consumed once at startup to build the GPU assets, then
/// removed (freed) so a second copy of the geometry does not sit resident.
#[derive(Resource)]
struct SceneMeshes {
    l4: IndexedMesh,
    l5: IndexedMesh,
}

/// The lightweight overlay data the per-frame systems + panel read for the
/// whole session (the bone meshes are gone by then).
#[derive(Resource)]
struct Overlays {
    ligaments: Vec<Ligament>,
    facet_contacts: Vec<Point3<f64>>,
    aabb: Aabb,
    warnings: Vec<String>,
}

/// The captured flexion sweep + its live replay state. One shared cursor drives
/// the disc deformation AND L4's rotation, so they stay in exact lockstep.
#[derive(Resource)]
struct Flexion {
    traj: FlexionTrajectory,
    /// Auto-advancing playback (vs. paused / hand-scrubbed).
    playing: bool,
    /// Continuous frame position in `[0, frames-1]`; interpolated for smoothness.
    cursor: f32,
    /// Ping-pong direction: `+1` flexing forward, `-1` returning.
    direction: f32,
    /// Deformation (and coherent rotation) exaggeration factor `N`.
    deform_scale: f32,
    /// Interpolated TRUE flexion angle at the cursor (rad) — the physical angle,
    /// before exaggeration. Written each frame; read by the overlays + panel.
    true_theta: f64,
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
        flexion,
        ligaments,
        facet_contacts,
        aabb,
        warnings,
    } = fsu;
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-spine-viewer — L4–L5 FSU (flexion)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SceneMeshes { l4, l5 })
        .insert_resource(Overlays {
            ligaments,
            facet_contacts,
            aabb,
            warnings,
        })
        .insert_resource(Flexion {
            // Start at the neutral (middle) frame so the FSU opens at rest and
            // eases into flexion rather than snapping to a full tilt on frame 1.
            cursor: (flexion.frames.len().saturating_sub(1)) as f32 / 2.0,
            traj: flexion,
            playing: true,
            direction: 1.0,
            deform_scale: DEFORM_SCALE_DEFAULT,
            true_theta: 0.0,
        })
        .insert_resource(SceneToggles::default())
        .add_systems(Startup, setup_scene)
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

/// Build a SMOOTH-shaded Bevy mesh from an indexed surface: area-weighted
/// per-vertex normals (`AttributedMesh::compute_normals`) rendered by the
/// shared attributed converter (f64→f32 + Z-up→Y-up handled internally).
fn smooth_mesh(indexed: &IndexedMesh) -> Mesh {
    let mut attributed = AttributedMesh::from(indexed.clone());
    attributed.compute_normals();
    triangle_mesh_from_attributed(&attributed)
}

/// Spawn one bone (vertebra) surface with its material + a visibility-toggle
/// marker, returning the entity so the caller can tag the flexing one.
fn spawn_bone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    indexed: &IndexedMesh,
    part: TissuePart,
) -> Entity {
    let material = StandardMaterial {
        base_color: BONE_COLOR,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        ..default()
    };
    commands
        .spawn((
            Mesh3d(meshes.add(smooth_mesh(indexed))),
            MeshMaterial3d(materials.add(material)),
            Transform::default(),
            Visibility::Visible,
            part,
        ))
        .id()
}

/// Spawn the two vertebrae (L4 flexes, L5 fixed) + the deforming disc soft-mesh,
/// frame the orbit camera, then drop the CPU bone meshes.
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene_meshes: Res<SceneMeshes>,
    overlays: Res<Overlays>,
    flexion: Res<Flexion>,
) {
    // L5 (inferior) — fixed. L4 (superior) — tagged as the flexing body.
    spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l5,
        TissuePart::L5,
    );
    let l4 = spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l4,
        TissuePart::L4,
    );
    commands.entity(l4).insert(FlexedL4);

    // Disc — the real deforming tet-boundary surface (native mm, Z-up→Y-up swap
    // in `build_soft_mesh`). Built at the rest configuration; `flexion_update`
    // rewrites its positions each frame. Translucent teal, double-sided.
    let rest: Vec<Vector3<f64>> = flexion
        .traj
        .rest_nodes_native
        .iter()
        .map(|p| p.coords)
        .collect();
    let disc_mesh = build_soft_mesh(&rest, &flexion.traj.boundary_faces, UpAxis::PlusZ);
    // OPAQUE: the disc is a CLOSED tet-boundary solid. Alpha blending it (as the
    // rung-1 STL shell did) makes Bevy blend every triangle without depth sorting,
    // which shreds the closed volume into translucent confetti. Opaque renders it as
    // the clean solid it is; the gap it fills reads fine without see-through.
    let disc_material = StandardMaterial {
        base_color: DISC_COLOR,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        alpha_mode: AlphaMode::Opaque,
        ..default()
    };
    commands.spawn((
        Mesh3d(meshes.add(disc_mesh)),
        MeshMaterial3d(materials.add(disc_material)),
        Transform::default(),
        Visibility::Visible,
        TissuePart::Disc,
        DiscMesh,
    ));

    // Native mm — frame directly on the combined AABB. `setup_camera_and_lighting`
    // swaps the target into Bevy Y-up, matching the swapped meshes.
    setup_camera_and_lighting(&mut commands, &overlays.aabb, UpAxis::PlusZ);

    // The bone meshes are now GPU assets; free the CPU-side copies for the session.
    commands.remove_resource::<SceneMeshes>();
}

/// Advance the replay cursor and drive BOTH the disc deformation and L4's
/// rotation from it, in lockstep. Deformation is exaggerated by `deform_scale`
/// (`rest + N·(deformed − rest)`) and L4 rotates by the coherently-scaled angle
/// (`N·θ` about the disc-centre pivot), so the disc top stays glued to L4.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn flexion_update(
    time: Res<Time<Real>>,
    mut flexion: ResMut<Flexion>,
    mut meshes: ResMut<Assets<Mesh>>,
    // Reused across frames (cleared + refilled in place) to avoid a per-frame heap alloc.
    mut flat: Local<Vec<f64>>,
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

    // Bracketing frames + fraction for interpolation.
    let c = flexion.cursor.clamp(0.0, max);
    let lo = c.floor() as usize;
    let hi = (lo + 1).min(n - 1);
    let frac = f64::from(c - lo as f32);
    let scale = f64::from(flexion.deform_scale);
    let pivot = flexion.traj.pivot;
    let axis = flexion.traj.axis;

    // Build the exaggerated deformed frame into the reused scratch buffer + the
    // interpolated true angle, in a scoped borrow so we can write back `true_theta`.
    // `flat` (a system Local) is independent of `flexion`, so filling it while borrowing
    // the trajectory is fine.
    flat.clear();
    let true_theta = {
        let traj = &flexion.traj;
        let (fa, fb) = (&traj.frames[lo], &traj.frames[hi]);
        for (i, r) in traj.rest_nodes_native.iter().enumerate() {
            let a = fa.deformed_nodes_native[i];
            let b = fb.deformed_nodes_native[i];
            let interp = a + (b - a) * frac; // interpolated deformed position
            let p = r + (interp - r) * scale; // exaggerated about the rest position
            flat.push(p.x);
            flat.push(p.y);
            flat.push(p.z);
        }
        fa.theta + (fb.theta - fa.theta) * frac
    };
    flexion.true_theta = true_theta;

    // Rewrite the disc surface (Z-up→Y-up swap + smooth-normal recompute).
    if let Ok(handle) = q_disc.single() {
        if let Some(mesh) = meshes.get_mut(&handle.0) {
            apply_soft_positions(mesh, &flat, UpAxis::PlusZ);
        }
    }

    // Rotate L4 by the exaggerated angle about the pivot. The Y/Z swap is a
    // reflection, so `quat_from_unit_quaternion` gives the swap-consistent
    // rotation R; rotating about pivot p is `R` with translation `p − R·p`.
    let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), true_theta * scale);
    let r_bevy = quat_from_unit_quaternion(&rot);
    let p_bevy = vec3_from_point(&pivot);
    if let Ok(mut tf) = q_l4.single_mut() {
        tf.rotation = r_bevy;
        tf.translation = p_bevy - r_bevy * p_bevy;
    }
}

/// The physics-space rotation that maps a rest point to its displayed (exaggerated)
/// pose: `pivot + R(axis, N·θ)·(p − pivot)`. Shared by the overlay systems so
/// L4-attached sites track the vertebra exactly.
fn display_rotation(flexion: &Flexion) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(flexion.traj.axis),
        flexion.true_theta * f64::from(flexion.deform_scale),
    )
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

/// Draw the ligament lines (ALL, ISP) as tan polylines with site markers. The
/// superior (L4) attachment rides the flexing vertebra; the inferior (L5) is fixed.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_ligaments(
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
        let superior = pivot + rot * (lig.superior - pivot); // rides L4
        let (a, b) = (vec3_from_point(&lig.inferior), vec3_from_point(&superior));
        gizmos.line(a, b, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(a), SITE_RADIUS, LIGAMENT_COLOR);
        gizmos.sphere(Isometry3d::from_translation(b), SITE_RADIUS, LIGAMENT_COLOR);
    }
}

/// Draw the facet near-contact points as small red spheres, riding the superior
/// articular process (rotated with L4 — an approximation of the shifting contact).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_facets(
    mut gizmos: Gizmos,
    overlays: Res<Overlays>,
    toggles: Res<SceneToggles>,
    flexion: Res<Flexion>,
) {
    if !toggles.facets {
        return;
    }
    let rot = display_rotation(&flexion);
    let pivot = flexion.traj.pivot;
    for p in &overlays.facet_contacts {
        let posed = pivot + rot * (p - pivot);
        gizmos.sphere(
            Isometry3d::from_translation(vec3_from_point(&posed)),
            FACET_RADIUS,
            FACET_COLOR,
        );
    }
}

/// Right-side panel: flexion replay controls + per-tissue visibility toggles +
/// any co-registration warnings (so a misaligned trio is never shown as valid).
fn scene_panel(
    mut contexts: EguiContexts,
    mut toggles: ResMut<SceneToggles>,
    mut flexion: ResMut<Flexion>,
    overlays: Res<Overlays>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("fsu-panel")
        .resizable(false)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.heading("L4–L5 FSU");
            ui.label("geometry ladder rung 7 (flexion)");
            ui.separator();

            // ── Flexion replay ──
            ui.label("Flexion replay:");
            let n = flexion.traj.frames.len();
            let max = (n.saturating_sub(1)) as f32;
            let play_label = if flexion.playing {
                "⏸ Pause"
            } else {
                "▶ Play"
            };
            if ui.button(play_label).clicked() {
                flexion.playing = !flexion.playing;
            }
            // Scrubbing the timeline pauses auto-play.
            let mut cursor = flexion.cursor;
            if ui
                .add(egui::Slider::new(&mut cursor, 0.0..=max).text("frame"))
                .changed()
            {
                flexion.cursor = cursor;
                flexion.playing = false;
            }
            let mut scale = flexion.deform_scale;
            if ui
                .add(egui::Slider::new(&mut scale, 1.0..=DEFORM_SCALE_MAX).text("deformation ×N"))
                .changed()
            {
                flexion.deform_scale = scale;
            }
            let true_deg = flexion.true_theta.to_degrees();
            let shown_deg = true_deg * f64::from(flexion.deform_scale);
            ui.label(format!("true flexion: {true_deg:+.3}°"));
            ui.label(format!(
                "shown ×{:.0} = {shown_deg:+.1}° (exaggerated)",
                flexion.deform_scale
            ));
            ui.separator();

            ui.label("Show:");
            ui.checkbox(&mut toggles.l4, "L4 vertebra");
            ui.checkbox(&mut toggles.l5, "L5 vertebra");
            ui.checkbox(&mut toggles.disc, "Intervertebral disc (FEM)");
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
