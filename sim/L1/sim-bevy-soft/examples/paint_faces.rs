//! Disc region-painting tool (`paint-faces`) — paint two endplate patches,
//! loft a disc between them, export it to drive the physics.
//!
//! Loads the two vertebrae (`$CF_L4_STL`, `$CF_L5_STL`) and lets you **paint /
//! erase a region of faces on each** with a round brush drawn on the surface.
//! This is the region-selection front-end for the disc loft: a human paints the
//! two endplate patches (the automatic rule can't select real endplates
//! cleanly), which then loft into the disc (**Enter**).
//!
//! Picking contract: `cf_bevy_common::triangle_mesh_flat_shaded` emits three
//! vertices per face **in face order**, so a mesh ray hit's `triangle_index` is
//! the source `IndexedMesh` face id; the brush paints every face whose centroid
//! is within the brush radius of the hit face (optionally normal-filtered).
//!
//! Controls (also shown in the on-screen HUD): **left-drag** orbits; **scroll**
//! zooms; **right-drag** pans. **Shift + left-drag** paints/erases on the active
//! body; **Tab** switches active body; **E** paint/erase; **N** normal filter;
//! **`-` / `=`** filter tolerance; **`[` / `]`** brush size; **C** clears the
//! active body. **Enter** lofts the disc; **S** exports it to an STL.
//!
//! **Paint → simulate loop:** paint both endplates, **Enter** to loft, **S** to
//! export the disc (STL, to `$CF_DISC_OUT` or a temp path). On export the console
//! prints the exact command to run the **anatomical FSU** — the painted disc as
//! the deforming soft bushing between the *real* L4/L5 vertebrae, with the
//! field-derived ligaments and facet contacts, solved as one moment-driven
//! equilibrium (`cf-spine-viewer`, reusing the same vertebra STLs this tool
//! loaded). A simpler alternative bonds the disc between two abstract plates
//! (`real-disc-bonded`, via `$CF_DISC_STL`). Same validated recipe either way:
//! the painted disc tet-meshed, bonded, and flexing.
//!
//! The STLs are BodyParts3D (CC BY-SA, **not committed**). Point `$CF_L4_STL` /
//! `$CF_L5_STL` at the L4 / L5 STLs (FMA13075 / 13076).
//!
//! Run: `CF_L4_STL=… CF_L5_STL=… cargo run --release -p sim-bevy-soft --example paint-faces`

#![allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#![allow(clippy::expect_used)] // an example may expect on its own required asset/env setup.
#![allow(clippy::too_many_arguments)] // Bevy systems legitimately take many resource params.

use std::collections::HashSet;
use std::path::PathBuf;

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::mesh::VertexAttributeValues;
use bevy::picking::mesh_picking::ray_cast::{MeshRayCast, MeshRayCastSettings};
use bevy::prelude::*;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use cf_bevy_common::prelude::{OrbitCamera, update_orbit_camera};
use cf_fsu_geometry::load_from_env;
use cf_fsu_geometry::loft::{
    WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch, is_watertight,
};
use cf_geometry::IndexedMesh;
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_io::save_stl;
use mesh_types::Aabb;

/// Base (unpainted) face colour — pale bone ivory.
const BASE: [f32; 4] = [0.80, 0.78, 0.72, 1.0];
/// Highlight colour for a painted face.
const HIGHLIGHT: [f32; 4] = [0.90, 0.30, 0.20, 1.0];
/// Initial brush radius, in millimetres (native mesh units).
const BRUSH_INIT: f64 = 4.0;
/// Brush radius bounds (mm).
const BRUSH_MIN: f64 = 0.5;
const BRUSH_MAX: f64 = 30.0;

/// A paintable body: its render mesh, per-face centroids + unit normals (native
/// mm) for the brush queries, and the set of painted face ids.
#[derive(Component)]
struct PaintBody {
    name: &'static str,
    source: IndexedMesh,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    centroids: Vec<[f64; 3]>,
    normals: Vec<[f64; 3]>,
    painted: HashSet<usize>,
}

/// Whether the lofted disc is currently shown (review mode). In review both
/// vertebrae are translucent and the disc is visible; otherwise only the active
/// body is shown, opaque, for painting.
#[derive(Resource, Default)]
struct ShowDisc(bool);

/// The spawned disc entity (replaced each time the disc is re-lofted).
#[derive(Resource, Default)]
struct DiscEntity(Option<Entity>);

/// The last lofted disc mesh (native mm), kept so `S` can export it to an STL
/// the physics consumers (`real-disc-bonded`) load via `$CF_DISC_STL`.
#[derive(Resource, Default)]
struct LoftedDisc(Option<IndexedMesh>);

/// The two bodies and which one the brush acts on (cycled with `Tab`).
#[derive(Resource)]
struct Bodies {
    entities: Vec<Entity>,
    active: usize,
}

impl Bodies {
    fn active_entity(&self) -> Entity {
        self.entities[self.active]
    }
}

/// Restrict the brush to faces whose normal is within `max_angle_deg` of the
/// face under the cursor — so painting a flat endplate doesn't spill onto the
/// steep lateral walls. Toggle with `N`, widen / tighten with `-` / `=`.
#[derive(Resource)]
struct NormalFilter {
    enabled: bool,
    max_angle_deg: f64,
}

/// The surface point under the cursor this frame (world space) and the active
/// body's face there — set by [`hover_ray`], read by the brush ring and stroke.
#[derive(Resource, Default)]
struct Hover {
    point: Option<Vec3>,
    normal: Vec3,
    face: Option<usize>,
}

/// The current brush radius (mm).
#[derive(Resource)]
struct BrushRadius(f64);

/// Whether the brush paints or erases.
#[derive(Resource, Clone, Copy, PartialEq, Eq)]
enum BrushMode {
    Paint,
    Erase,
}

/// Marks the HUD status-text node (the dynamic half).
#[derive(Component)]
struct HudText;

/// One paint/erase stroke: the faces whose selection state it flipped, so it
/// can be undone. `mode` records whether the stroke painted or erased them.
struct Stroke {
    body: Entity,
    mode: BrushMode,
    faces: Vec<usize>,
}

/// Undo stack of finished strokes.
#[derive(Resource, Default)]
struct History(Vec<Stroke>);

/// The stroke currently being drawn (accumulates while Shift + left is held).
#[derive(Resource, Default)]
struct ActiveStroke(Option<Stroke>);

/// Static controls reference, one per row, shown in the HUD sidebar.
const CONTROLS: &str = "CONTROLS\n\
     orbit        left-drag\n\
     pan          right-drag\n\
     zoom         scroll\n\
     paint        Shift + left-drag\n\
     Tab          switch body\n\
     E            paint / erase\n\
     N            normal filter\n\
     - / =        tolerance\n\
     [ / ]        brush size\n\
     Ctrl + Z     undo stroke\n\
     C            clear body\n\
     Enter        loft disc\n\
     S            export disc (STL)\n\
     Tab          (in review) back";

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<Hover>()
        .init_resource::<History>()
        .init_resource::<ActiveStroke>()
        .init_resource::<ShowDisc>()
        .init_resource::<DiscEntity>()
        .init_resource::<LoftedDisc>()
        .insert_resource(BrushRadius(BRUSH_INIT))
        .insert_resource(BrushMode::Paint)
        .insert_resource(NormalFilter {
            enabled: true,
            max_angle_deg: 35.0,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, (camera_input, update_orbit_camera).chain())
        .add_systems(
            Update,
            (hover_ray, apply_brush, finalize_stroke, draw_brush).chain(),
        )
        .add_systems(
            Update,
            (
                adjust_brush,
                toggle_mode,
                toggle_filter,
                switch_body,
                clear_selection,
                undo_stroke,
                loft_disc,
                export_disc,
            ),
        )
        .add_systems(Update, (update_hud, update_display))
        .run();
}

/// Per-face centroids and unit normals in native coordinates.
fn face_geometry(mesh: &IndexedMesh) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut centroids = Vec::with_capacity(mesh.faces.len());
    let mut normals = Vec::with_capacity(mesh.faces.len());
    for &[a, b, c] in &mesh.faces {
        let (a, b, c) = (
            mesh.vertices[a as usize],
            mesh.vertices[b as usize],
            mesh.vertices[c as usize],
        );
        centroids.push([
            (a.x + b.x + c.x) / 3.0,
            (a.y + b.y + c.y) / 3.0,
            (a.z + b.z + c.z) / 3.0,
        ]);
        let n = (b - a).cross(&(c - a));
        let len = n.norm();
        normals.push(if len > 1e-12 {
            [n.x / len, n.y / len, n.z / len]
        } else {
            [0.0, 0.0, 1.0]
        });
    }
    (centroids, normals)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let l4 = load_from_env("CF_L4_STL").expect("set $CF_L4_STL (superior vertebra, FMA13075)");
    let l5 = load_from_env("CF_L5_STL").expect("set $CF_L5_STL (inferior vertebra, FMA13076)");

    let mut entities = Vec::new();
    for (mesh, name) in [(&l4, "L4"), (&l5, "L5")] {
        let (centroids, normals) = face_geometry(mesh);
        let seed = vec![BASE; mesh.vertices.len()];
        let handle = meshes.add(triangle_mesh_flat_shaded(mesh, Some(&seed), UpAxis::PlusZ));
        let material = materials.add(StandardMaterial {
            base_color: Color::WHITE, // per-vertex face colours show through
            perceptual_roughness: 0.85,
            double_sided: true,
            cull_mode: None,
            ..default()
        });
        let entity = commands
            .spawn((
                Mesh3d(handle.clone()),
                MeshMaterial3d(material.clone()),
                Transform::default(),
                PaintBody {
                    name,
                    source: mesh.clone(),
                    mesh: handle,
                    material,
                    centroids,
                    normals,
                    painted: HashSet::new(),
                },
            ))
            .id();
        entities.push(entity);
    }
    commands.insert_resource(Bodies {
        entities,
        active: 0,
    });

    // Frame both vertebrae.
    let bounds = Aabb::from_points(l4.vertices.iter().chain(l5.vertices.iter()));
    setup_camera_and_lighting(&mut commands, &bounds, UpAxis::PlusZ);

    // HUD sidebar (full-height, left edge): a title, the static controls list,
    // and the live status block, each item on its own row.
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                height: Val::Percent(100.0),
                width: Val::Px(226.0),
                padding: UiRect::all(Val::Px(16.0)),
                row_gap: Val::Px(16.0),
                flex_direction: FlexDirection::Column,
                ..default()
            },
            BackgroundColor(Color::srgba(0.05, 0.05, 0.07, 0.88)),
        ))
        .with_children(|panel| {
            panel.spawn((
                Text::new("PAINT  FACES"),
                TextFont {
                    font_size: 17.0,
                    ..default()
                },
                TextColor(Color::srgb(0.96, 0.72, 0.40)),
            ));
            panel.spawn((
                Text::new(CONTROLS),
                TextFont {
                    font_size: 12.5,
                    ..default()
                },
                TextColor(Color::srgb(0.60, 0.62, 0.68)),
            ));
            panel.spawn((
                Text::new(""),
                TextFont {
                    font_size: 13.0,
                    ..default()
                },
                TextColor(Color::srgb(0.95, 0.95, 0.97)),
                HudText,
            ));
        });
}

/// Usual controls: orbit on left-drag (unless Shift is held for painting), pan
/// on right-drag, zoom on scroll.
fn camera_input(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut cameras: Query<&mut OrbitCamera>,
) {
    for mut camera in &mut cameras {
        if mouse.pressed(MouseButton::Left) && !painting(&keys) {
            camera.orbit(motion.delta);
        }
        if mouse.pressed(MouseButton::Right) {
            camera.pan(motion.delta);
        }
        if scroll.delta.y.abs() > 1e-3 {
            camera.zoom(scroll.delta.y);
        }
    }
}

/// Ray-cast the cursor every frame, recording the hit only when the **active**
/// body is the front-most surface — so the brush ring and stroke stay on the
/// body you are painting.
fn hover_ray(
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    bodies: Res<Bodies>,
    mut ray_cast: MeshRayCast,
    mut hover: ResMut<Hover>,
) {
    hover.point = None;
    hover.face = None;
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
        return;
    };
    let active = bodies.active_entity();
    if let Some((entity, hit)) = ray_cast
        .cast_ray(ray, &MeshRayCastSettings::default())
        .first()
    {
        if *entity == active {
            hover.point = Some(hit.point);
            hover.normal = hit.normal;
            hover.face = hit.triangle_index;
        }
    }
}

/// While Shift + left is held, paint (or erase) faces on the active body within
/// the brush radius (and, if enabled, the normal-similarity tolerance).
fn apply_brush(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<BrushMode>,
    radius: Res<BrushRadius>,
    filter: Res<NormalFilter>,
    hover: Res<Hover>,
    bodies: Res<Bodies>,
    mut stroke: ResMut<ActiveStroke>,
    mut q_bodies: Query<&mut PaintBody>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !painting(&keys) || !mouse.pressed(MouseButton::Left) {
        return;
    }
    let Some(face) = hover.face else {
        return;
    };
    let active_entity = bodies.active_entity();
    let Ok(mut body) = q_bodies.get_mut(active_entity) else {
        return;
    };
    let PaintBody {
        mesh: handle,
        centroids,
        normals,
        painted,
        ..
    } = body.as_mut();

    let (Some(&centre), Some(&reference)) = (centroids.get(face), normals.get(face)) else {
        return;
    };
    let Some(mesh) = meshes.get_mut(&*handle) else {
        return;
    };
    let Some(VertexAttributeValues::Float32x4(colours)) = mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    else {
        return;
    };

    // Begin (or continue) the current stroke, so it can be undone as a unit.
    let stroke = stroke.0.get_or_insert_with(|| Stroke {
        body: active_entity,
        mode: *mode,
        faces: Vec::new(),
    });

    let r2 = radius.0 * radius.0;
    let cos_min = filter.max_angle_deg.to_radians().cos();
    for (f, (c, n)) in centroids.iter().zip(normals.iter()).enumerate() {
        let d = [c[0] - centre[0], c[1] - centre[1], c[2] - centre[2]];
        if d[0].mul_add(d[0], d[1].mul_add(d[1], d[2] * d[2])) > r2 {
            continue;
        }
        if filter.enabled {
            let dot = n[0].mul_add(
                reference[0],
                n[1].mul_add(reference[1], n[2] * reference[2]),
            );
            if dot < cos_min {
                continue; // normal too different from the face under the cursor
            }
        }
        let changed = match stroke.mode {
            BrushMode::Paint => {
                let inserted = painted.insert(f);
                if inserted {
                    recolour(colours, f, HIGHLIGHT);
                }
                inserted
            }
            BrushMode::Erase => {
                let removed = painted.remove(&f);
                if removed {
                    recolour(colours, f, BASE);
                }
                removed
            }
        };
        if changed {
            stroke.faces.push(f);
        }
    }
}

/// Close the current stroke once the brush is released, pushing it (if it
/// changed anything) onto the undo history.
fn finalize_stroke(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut stroke: ResMut<ActiveStroke>,
    mut history: ResMut<History>,
) {
    if stroke.0.is_none() {
        return;
    }
    if painting(&keys) && mouse.pressed(MouseButton::Left) {
        return; // still drawing
    }
    if let Some(done) = stroke.0.take() {
        if !done.faces.is_empty() {
            history.0.push(done);
        }
    }
}

/// Undo the last stroke with `Ctrl`/`Cmd + Z`: re-flip the faces it changed and
/// make that body active so the change is visible.
fn undo_stroke(
    keys: Res<ButtonInput<KeyCode>>,
    mut history: ResMut<History>,
    mut bodies: ResMut<Bodies>,
    mut q_bodies: Query<&mut PaintBody>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !undo_pressed(&keys) {
        return;
    }
    let Some(stroke) = history.0.pop() else {
        return;
    };
    if let Some(index) = bodies.entities.iter().position(|&e| e == stroke.body) {
        bodies.active = index;
    }
    let Ok(mut body) = q_bodies.get_mut(stroke.body) else {
        return;
    };
    let PaintBody {
        mesh: handle,
        painted,
        ..
    } = body.as_mut();
    let Some(mesh) = meshes.get_mut(&*handle) else {
        return;
    };
    let Some(VertexAttributeValues::Float32x4(colours)) = mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    else {
        return;
    };
    for &f in &stroke.faces {
        match stroke.mode {
            BrushMode::Paint => {
                painted.remove(&f);
                recolour(colours, f, BASE);
            }
            BrushMode::Erase => {
                painted.insert(f);
                recolour(colours, f, HIGHLIGHT);
            }
        }
    }
}

/// Draw the brush as a ring on the surface, coloured by mode.
fn draw_brush(
    mut gizmos: Gizmos,
    hover: Res<Hover>,
    radius: Res<BrushRadius>,
    mode: Res<BrushMode>,
) {
    let Some(point) = hover.point else {
        return;
    };
    let colour = match *mode {
        BrushMode::Paint => Color::srgb(0.95, 0.35, 0.25),
        BrushMode::Erase => Color::srgb(0.30, 0.80, 0.90),
    };
    let rotation = Quat::from_rotation_arc(Vec3::Z, hover.normal.normalize_or_zero());
    gizmos.circle(Isometry3d::new(point, rotation), radius.0 as f32, colour);
}

/// Shrink / grow the brush with `[` / `]`.
fn adjust_brush(keys: Res<ButtonInput<KeyCode>>, mut radius: ResMut<BrushRadius>) {
    if keys.just_pressed(KeyCode::BracketLeft) {
        radius.0 = (radius.0 * 0.8).max(BRUSH_MIN);
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        radius.0 = (radius.0 * 1.25).min(BRUSH_MAX);
    }
}

/// Toggle paint / erase with `E`.
fn toggle_mode(keys: Res<ButtonInput<KeyCode>>, mut mode: ResMut<BrushMode>) {
    if keys.just_pressed(KeyCode::KeyE) {
        *mode = match *mode {
            BrushMode::Paint => BrushMode::Erase,
            BrushMode::Erase => BrushMode::Paint,
        };
    }
}

/// Toggle the normal-similarity filter with `N`; widen / tighten it with
/// `-` / `=`.
fn toggle_filter(keys: Res<ButtonInput<KeyCode>>, mut filter: ResMut<NormalFilter>) {
    if keys.just_pressed(KeyCode::KeyN) {
        filter.enabled = !filter.enabled;
    }
    if keys.just_pressed(KeyCode::Minus) {
        filter.max_angle_deg = (filter.max_angle_deg + 5.0).min(90.0);
    }
    if keys.just_pressed(KeyCode::Equal) {
        filter.max_angle_deg = (filter.max_angle_deg - 5.0).max(5.0);
    }
}

/// Cycle the active body with `Tab` (also returns from disc review to painting).
fn switch_body(
    keys: Res<ButtonInput<KeyCode>>,
    mut bodies: ResMut<Bodies>,
    mut show: ResMut<ShowDisc>,
) {
    if keys.just_pressed(KeyCode::Tab) {
        if show.0 {
            show.0 = false; // leave review without advancing the body
        } else {
            bodies.active = (bodies.active + 1) % bodies.entities.len();
        }
    }
}

/// Clear the active body's selection with `C`.
fn clear_selection(
    keys: Res<ButtonInput<KeyCode>>,
    bodies: Res<Bodies>,
    mut q_bodies: Query<&mut PaintBody>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !keys.just_pressed(KeyCode::KeyC) {
        return;
    }
    let Ok(mut body) = q_bodies.get_mut(bodies.active_entity()) else {
        return;
    };
    if body.painted.is_empty() {
        return;
    }
    if let Some(mesh) = meshes.get_mut(&body.mesh) {
        if let Some(VertexAttributeValues::Float32x4(colours)) =
            mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
        {
            for &f in &body.painted {
                recolour(colours, f, BASE);
            }
        }
    }
    body.painted.clear();
}

/// Refresh the on-screen HUD: controls + live status.
fn update_hud(
    bodies: Res<Bodies>,
    mode: Res<BrushMode>,
    radius: Res<BrushRadius>,
    filter: Res<NormalFilter>,
    q_bodies: Query<&PaintBody>,
    mut q_text: Query<&mut Text, With<HudText>>,
) {
    let Ok(mut text) = q_text.single_mut() else {
        return;
    };
    let active = bodies.active_entity();
    let active_name = q_bodies.get(active).map_or("?", |b| b.name);
    let mode = match *mode {
        BrushMode::Paint => "PAINT",
        BrushMode::Erase => "ERASE",
    };
    let filter_status = if filter.enabled {
        format!("on ({:.0} deg)", filter.max_angle_deg)
    } else {
        "off".to_string()
    };
    let mut counts = String::new();
    for &e in &bodies.entities {
        if let Ok(b) = q_bodies.get(e) {
            counts.push_str(&format!("  {:<11}{}\n", b.name, b.painted.len()));
        }
    }

    text.0 = format!(
        "STATUS\n\
         active       {active_name}\n\
         mode         {mode}\n\
         brush        {:.1} mm\n\
         filter       {filter_status}\n\
         \n\
         painted\n\
         {counts}",
        radius.0,
    );
}

/// Loft the two painted patches into the disc with `Enter`, spawn it, and enter
/// review (both vertebrae translucent, disc visible).
fn loft_disc(
    keys: Res<ButtonInput<KeyCode>>,
    q_bodies: Query<&PaintBody>,
    mut show: ResMut<ShowDisc>,
    mut disc_entity: ResMut<DiscEntity>,
    mut lofted: ResMut<LoftedDisc>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !keys.just_pressed(KeyCode::Enter) {
        return;
    }

    // Extract + prepare each painted patch, keyed by body name. finalize_patch
    // keeps the largest component and seals interior holes, leaving one outer rim.
    let mut l4 = None;
    let mut l5 = None;
    for body in &q_bodies {
        if body.painted.len() < 3 {
            continue;
        }
        let faces: Vec<usize> = body.painted.iter().copied().collect();
        let patch = finalize_patch(&extract_patch(&body.source, &faces));
        match body.name {
            "L4" => l4 = Some(patch),
            "L5" => l5 = Some(patch),
            _ => {}
        }
    }
    let (Some(l4), Some(l5)) = (l4, l5) else {
        println!("loft: paint a region on BOTH L4 and L5 first");
        return;
    };

    // L4's bone normal points down toward the disc, so flip it to match L5's.
    // Arc-length correspondence distributes evenly around these convex painted
    // rims, avoiding the fan/spike a greedy shortest-diagonal can drift into.
    let top = flip_patch(&l4);
    let bushing = assemble_bushing(&top, &l5, 1, WallCorrespondence::ArcLength);
    let disc = bushing.mesh;
    if !is_watertight(&disc) {
        println!(
            "loft: the painted patches didn't form a watertight disc — paint a \
             single connected region on each body and try again"
        );
        return;
    }
    println!(
        "lofted disc: {} verts / {} faces (press S to export for physics)",
        disc.vertices.len(),
        disc.faces.len()
    );
    lofted.0 = Some(disc.clone());

    let render = triangle_mesh_flat_shaded(&disc, None, UpAxis::PlusZ);
    let handle = meshes.add(render);
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.62, 0.68),
        perceptual_roughness: 0.6,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    if let Some(old) = disc_entity.0.take() {
        commands.entity(old).despawn();
    }
    disc_entity.0 = Some(
        commands
            .spawn((
                Mesh3d(handle),
                MeshMaterial3d(material),
                Transform::default(),
            ))
            .id(),
    );
    show.0 = true;
}

/// Export the last lofted disc to an STL with `S` and print the exact commands
/// to simulate it — closing the paint → simulate loop. The path is
/// `$CF_DISC_OUT`, or `painted_disc.stl` in the temp dir.
///
/// The primary finale is the **anatomical FSU** (`cf-spine-viewer`): the painted
/// disc becomes the deforming soft bushing between the *real* L4/L5 vertebrae,
/// with the field-derived ligaments and facet contacts, solved as one
/// moment-driven equilibrium ([`cf_fsu_model::CoupledFsu`]). It reuses the same
/// vertebra STLs the tool was loaded with (`$CF_L4_STL` / `$CF_L5_STL`), so the
/// printed command is a single copy-paste. A simpler alternative bonds the disc
/// between two abstract plates (`real-disc-bonded`).
fn export_disc(keys: Res<ButtonInput<KeyCode>>, lofted: Res<LoftedDisc>) {
    if !keys.just_pressed(KeyCode::KeyS) {
        return;
    }
    let Some(disc) = &lofted.0 else {
        println!("export: loft a disc (Enter) first");
        return;
    };
    let path = std::env::var("CF_DISC_OUT").map_or_else(
        |_| std::env::temp_dir().join("painted_disc.stl"),
        PathBuf::from,
    );
    if let Err(e) = save_stl(disc, &path, true) {
        println!("export failed: {e}");
        return;
    }
    let p = path.display();
    println!("exported disc -> {p}");
    // The anatomical finale: the painted disc as the bushing between the REAL
    // vertebrae. Reuse the vertebra paths the tool was loaded with.
    match (std::env::var("CF_L4_STL"), std::env::var("CF_L5_STL")) {
        (Ok(l4), Ok(l5)) => println!(
            "  anatomical FSU (disc between the real L4/L5, + ligaments + facets):\n\
             \x20   cargo run --release -p cf-spine-viewer -- --l4 {l4} --l5 {l5} --disc {p}"
        ),
        _ => println!(
            "  anatomical FSU: cargo run --release -p cf-spine-viewer -- --l4 <L4.stl> --l5 <L5.stl> --disc {p}"
        ),
    }
    println!(
        "  or between two plates: CF_DISC_STL={p} cargo run --release -p sim-bevy-soft --example real-disc-bonded"
    );
}

/// Update what is shown: in review (disc lofted) both vertebrae are translucent
/// and the disc is visible; otherwise only the active body is shown, opaque.
fn update_display(
    bodies: Res<Bodies>,
    show: Res<ShowDisc>,
    disc_entity: Res<DiscEntity>,
    q_bodies: Query<&PaintBody>,
    mut q_vis: Query<&mut Visibility>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !bodies.is_changed() && !show.is_changed() && !disc_entity.is_changed() {
        return;
    }
    for (i, &e) in bodies.entities.iter().enumerate() {
        if let Ok(mut visibility) = q_vis.get_mut(e) {
            *visibility = if show.0 || i == bodies.active {
                Visibility::Visible
            } else {
                Visibility::Hidden
            };
        }
        if let Ok(body) = q_bodies.get(e) {
            if let Some(material) = materials.get_mut(&body.material) {
                if show.0 {
                    material.base_color = Color::srgba(0.87, 0.84, 0.78, 0.30);
                    material.alpha_mode = AlphaMode::Blend;
                } else {
                    material.base_color = Color::WHITE;
                    material.alpha_mode = AlphaMode::Opaque;
                }
            }
        }
    }
    if let Some(disc) = disc_entity.0 {
        if let Ok(mut visibility) = q_vis.get_mut(disc) {
            *visibility = if show.0 {
                Visibility::Visible
            } else {
                Visibility::Hidden
            };
        }
    }
}

/// Whether a Shift key (the paint modifier) is held.
fn painting(keys: &ButtonInput<KeyCode>) -> bool {
    keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight)
}

/// Whether the undo chord (`Ctrl`/`Cmd + Z`) was just pressed.
fn undo_pressed(keys: &ButtonInput<KeyCode>) -> bool {
    let modifier = keys.pressed(KeyCode::ControlLeft)
        || keys.pressed(KeyCode::ControlRight)
        || keys.pressed(KeyCode::SuperLeft)
        || keys.pressed(KeyCode::SuperRight);
    modifier && keys.just_pressed(KeyCode::KeyZ)
}

/// Set the three emitted vertices of face `f` (`3f, 3f+1, 3f+2`) to `colour`.
fn recolour(colours: &mut [[f32; 4]], f: usize, colour: [f32; 4]) {
    let base = f * 3;
    for k in 0..3 {
        if let Some(slot) = colours.get_mut(base + k) {
            *slot = colour;
        }
    }
}
