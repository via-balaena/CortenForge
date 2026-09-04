//! The 3D view behind the wizard: camera, light, and the body on show.
//!
//! ⚠ The startup placeholder is **step 1's empty state**, not a render-safety
//! guard. An earlier note in this workspace claimed a drawn mesh must exist
//! wherever quit is reachable, to avoid a macOS quit deadlock. That attribution
//! was measured and is wrong: the deadlock is Bevy's pipelined-rendering
//! teardown and fires with a mesh on screen. `main.rs` carries the real fix. Do
//! not re-derive a scene constraint from it.

use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, PrimaryEguiContext};
use cf_bevy_common::camera::OrbitCamera;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use mesh_types::Bounded;

use crate::scan::{SCAN_UP_AXIS, ScanEdit, ViewUpdate};

/// The body on show — the startup placeholder, then the scan that replaces it.
#[derive(Component)]
pub(crate) struct SceneBody;

/// Muted clay, readable against the light background without competing with
/// the panel for attention.
const BODY_COLOR: Color = Color::srgb(0.72, 0.70, 0.66);

/// The centerline overlay's cyan, carried over from the pre-port line shader.
const CENTERLINE_COLOR: Color = Color::srgb(0.05, 0.85, 0.95);

/// Spawn the camera, a key light, and the placeholder body.
pub(crate) fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        OrbitCamera {
            distance: 4.0,
            elevation: 0.35,
            ..OrbitCamera::default()
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        SceneBody,
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(materials.add(body_material())),
    ));

    // ⚠⚠ egui's context lives on THIS camera, never on the 3D one, and the
    // separation is load-bearing rather than tidy.
    //
    // `bevy_egui` reads its context camera's viewport as egui's screen rect. Put
    // the context on the 3D camera and `fit_viewport_to_free_space` — which sets
    // that viewport from egui's *available* rect — closes a loop: egui lays out
    // inside the strip it was just confined to, the available rect collapses to
    // zero width, the zero guard resets the viewport to the whole window, and it
    // oscillates every frame. Measured on `main`: screen 1280 → 600 → 0, panels
    // bunched mid-window over a full-window 3D view, visibly strobing.
    //
    // `order: 1` puts egui on top of the scene, so tooltips and combo popups may
    // cross into the 3D strip; `ClearColorConfig::None` leaves the clear to the
    // 3D camera, whose `LoadOp` covers the whole target before its viewport
    // scissors the drawing.
    commands.spawn((
        Camera2d,
        Camera {
            order: 1,
            clear_color: ClearColorConfig::None,
            ..default()
        },
        PrimaryEguiContext,
    ));
}

/// Show the scan as it now stands: replace the body, and frame the camera on it
/// only when the scan itself is new.
///
/// ⚠ The camera is the whole reason this branches. A step-2 edit changes the
/// mesh a dozen times over, and re-framing on each would snap the view back to
/// the front every time the user welded or trimmed — so the pre-port code
/// rebuilt the buffers "PRESERVING the orbit angle". Only a freshly loaded scan
/// earns a camera move.
pub(crate) fn show_scan(
    mut commands: Commands,
    scan: Res<ScanEdit>,
    bodies: Query<Entity, With<SceneBody>>,
    mut cameras: Query<&mut OrbitCamera>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let reframe = match scan.view() {
        // The edit left nothing renderable; the last good view stands.
        ViewUpdate::Hold => return,
        ViewUpdate::Reframe => true,
        ViewUpdate::Remesh => false,
    };
    let Some(active) = scan.active() else { return };
    let mesh = active.display();

    for body in &bodies {
        commands.entity(body).despawn();
    }
    commands.spawn((
        SceneBody,
        Mesh3d(meshes.add(triangle_mesh_flat_shaded(mesh, None, SCAN_UP_AXIS))),
        MeshMaterial3d(materials.add(body_material())),
        active.scale().transform(),
    ));

    if reframe {
        let framing = active.scale().framing_aabb(&mesh.aabb());
        for mut camera in &mut cameras {
            *camera = OrbitCamera::framing_for_aabb(&framing, SCAN_UP_AXIS);
        }
    }
}

/// Draw the centerline Find floor traced, over the body.
///
/// The line runs *inside* the scan, so it is only visible because the gizmo
/// group is configured to ignore depth (`plugin.rs`) — the pre-port viewer had
/// its own line pipeline at `depth_compare: Always` for the same reason. Cyan,
/// as it was there.
///
/// Immediate mode, so this runs every frame. That is why the points come from
/// the scan's cache rather than the engine — see [`crate::scan::ActiveScan`].
pub(crate) fn draw_centerline(scan: Res<ScanEdit>, mut gizmos: Gizmos) {
    let Some(active) = scan.active() else { return };
    let points = active.centerline();
    if points.len() < 2 {
        return;
    }
    gizmos.linestrip(points.iter().copied(), CENTERLINE_COLOR);
}

/// The clay surface, for both the placeholder and the scan.
///
/// `double_sided` + no culling because a raw scan's winding is not to be
/// trusted: a back-facing triangle would otherwise read as a hole in the body.
fn body_material() -> StandardMaterial {
    StandardMaterial {
        base_color: BODY_COLOR,
        perceptual_roughness: 0.85,
        double_sided: true,
        cull_mode: None,
        ..default()
    }
}

/// Keep the 3D view inside the region egui leaves uncovered.
///
/// The panels cover [`crate::panel::PANEL_WIDTH`] at any window size. Without
/// this the camera renders the whole window, so the body — framed on the
/// *window's* centre — sits partly behind the right panel. Bevy's viewport is
/// physical pixels; egui's rect is logical points, hence the scale factor.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // Rect → whole pixels.
pub(crate) fn fit_viewport_to_free_space(
    mut contexts: EguiContexts,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut cameras: Query<&mut Camera, With<Camera3d>>,
) -> bevy::ecs::error::Result {
    let free = contexts.ctx_mut()?.available_rect();
    let Ok(window) = windows.single() else {
        return Ok(());
    };
    let scale = window.scale_factor();
    let whole_px = |points: f32| (points * scale).max(0.0) as u32;

    let bounds = UVec2::new(window.physical_width(), window.physical_height());
    let position = UVec2::new(whole_px(free.min.x), whole_px(free.min.y)).min(bounds);
    let size = UVec2::new(whole_px(free.width()), whole_px(free.height()))
        .min(bounds.saturating_sub(position));

    for mut camera in &mut cameras {
        // A zero-area viewport is not renderable; fall back to the full window.
        camera.viewport = (size.x > 0 && size.y > 0).then(|| Viewport {
            physical_position: position,
            physical_size: size,
            ..default()
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use bevy::asset::AssetPlugin;

    use super::*;
    use crate::scan::ActiveScan;

    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    /// A headless app with `show_scan` wired and a camera to aim.
    fn headless() -> App {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()))
            .init_asset::<Mesh>()
            .init_asset::<StandardMaterial>()
            .init_resource::<ScanEdit>()
            .add_systems(Update, show_scan);
        app.world_mut()
            .spawn((Camera3d::default(), OrbitCamera::default()));
        app
    }

    fn a_scan(tag: &str) -> Option<ActiveScan> {
        let path = std::env::temp_dir().join(format!(
            "cf-studio-gui-scene-{tag}-{}.stl",
            std::process::id()
        ));
        std::fs::write(&path, ONE_TRIANGLE_STL).ok()?;
        let loaded = ActiveScan::load(&path).ok();
        let _ = std::fs::remove_file(&path);
        loaded
    }

    /// Where the camera is aimed and how far back it sits — the two fields
    /// `framing_for_aabb` sets, and so the two that say whether it moved.
    fn aim_of(app: &mut App) -> Option<(Vec3, f32)> {
        app.world_mut()
            .query::<&OrbitCamera>()
            .iter(app.world())
            .next()
            .map(|c| (c.target, c.distance))
    }

    /// ★★★ The invariant #874 violated, and the reason the app strobed: egui's
    /// context must NOT live on the camera whose viewport we resize.
    ///
    /// `bevy_egui` reads its context camera's viewport as egui's screen rect, so
    /// with the context on `Camera3d` — the default, since it attaches to the
    /// first camera created — `fit_viewport_to_free_space` fed egui a rect
    /// derived from egui's own output. Measured before the fix: screen width
    /// 1280 -> 600 -> 0, oscillating every frame.
    ///
    /// This asserts the shape rather than the symptom, because the symptom needs
    /// a real window: exactly one 3D camera, exactly one context holder, and they
    /// are different entities.
    #[test]
    fn egui_does_not_share_a_camera_with_the_scene() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()))
            .init_asset::<Mesh>()
            .init_asset::<StandardMaterial>()
            .add_systems(Startup, setup_scene);
        app.update();

        let world = app.world_mut();
        let scene: Vec<Entity> = world
            .query_filtered::<Entity, With<Camera3d>>()
            .iter(world)
            .collect();
        let egui: Vec<Entity> = world
            .query_filtered::<Entity, With<PrimaryEguiContext>>()
            .iter(world)
            .collect();

        assert_eq!(scene.len(), 1, "one camera renders the scan");
        assert_eq!(egui.len(), 1, "and one holds egui's context");
        assert_ne!(
            scene[0], egui[0],
            "egui must not sit on the camera whose viewport is resized"
        );

        // ⚠ Both of these were live mutants: dropping either field still
        // compiles, still passes the assertions above, and still ruins the
        // window. `order` back to 0 leaves the two cameras' sequence undefined;
        // `clear_color` back to the default makes the UI camera clear the whole
        // target AFTER the scene drew into it, erasing the 3D view outright.
        let (ui, body) = (
            app.world().get::<Camera>(egui[0]),
            app.world().get::<Camera>(scene[0]),
        );
        assert!(
            ui.is_some() && body.is_some(),
            "both the scene camera and the context camera carry a Camera"
        );
        if let (Some(ui), Some(body)) = (ui, body) {
            // ⚠ The relationship, not the literal 1. "egui draws after the
            // scene" is the invariant; pinning the magic number would still pass
            // if someone pushed the 3D camera past it.
            assert!(
                ui.order > body.order,
                "egui must render after the scene, not before it: ui={} body={}",
                ui.order,
                body.order
            );
            assert!(
                matches!(ui.clear_color, ClearColorConfig::None),
                "the 3D camera owns the clear; a second one would wipe the scene"
            );
        }
    }

    /// ★★★ The invariant the whole intent split exists to protect: a NEW scan
    /// frames the camera, an EDIT must not touch it. Inverting these two arms
    /// would snap the view back to the front on every weld and trim — and the
    /// producing side's tests cannot see it, because they only assert which
    /// `ViewUpdate` was recorded, never what the viewport did with it.
    #[test]
    fn a_load_frames_the_camera_and_an_edit_leaves_it_alone() {
        let loaded = a_scan("frame");
        assert!(loaded.is_some(), "the fixture must load");
        let Some(scan) = loaded else { return };
        let mut app = headless();
        let start = aim_of(&mut app);
        assert!(start.is_some(), "the fixture must have a camera");

        app.world_mut().resource_mut::<ScanEdit>().set(scan);
        app.update();
        let framed = aim_of(&mut app);
        // ⚠ `is_some` first. `assert_ne!` alone would be satisfied by the camera
        // having been despawned, and the equality below would then hold
        // vacuously — the whole test would pass while nothing was aimed at all.
        assert!(framed.is_some(), "the camera must survive the re-mesh");
        assert_ne!(framed, start, "a new scan must frame the camera on it");

        app.world_mut()
            .resource_mut::<ScanEdit>()
            .edit(cf_studio_engine::EditSession::weld);
        app.update();

        assert_eq!(
            aim_of(&mut app),
            framed,
            "an edit re-meshes; it must never move the camera"
        );
    }
}
