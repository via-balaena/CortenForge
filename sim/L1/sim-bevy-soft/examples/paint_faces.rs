//! Disc region-painting tool (`paint-faces`) — paint two endplate patches,
//! loft a disc between them, export it to drive the physics.
//!
//! Loads the two vertebrae (`$CF_L4_STL`, `$CF_L5_STL`) and lets you **paint /
//! erase a region of faces on each** with the [`cf_mesh_paint`] brush. This is
//! the anatomical composition of that generic primitive: a human paints the two
//! endplate patches (the automatic rule can't select real endplates cleanly),
//! which then loft into the disc (**Enter**). The brush, undo, filter, and
//! active-body switching all come from [`MeshPaintPlugin`]; this example adds
//! only the spine-specific glue (vertebra loading, loft, export, disc review).
//!
//! **Paint → simulate loop:** paint both endplates, **Enter** to loft, **S** to
//! export the disc (STL, to `$CF_DISC_OUT` or a temp path). On export the console
//! prints the exact command to run the **anatomical FSU** — the painted disc as
//! the deforming soft bushing between the *real* L4/L5 vertebrae, with the
//! field-derived ligaments and facet contacts, solved as one moment-driven
//! equilibrium (`cf-spine-studio`, reusing the same vertebra STLs this tool
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

use std::path::PathBuf;

use bevy::prelude::*;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use cf_bevy_common::prelude::update_orbit_camera;
use cf_fsu_geometry::load_from_env;
use cf_geometry::IndexedMesh;
use cf_mesh_paint::prelude::*;
use cf_mesh_paint::{Brush, BrushMode, NormalFilter};
use cf_viewer::{UpAxis, setup_camera_and_lighting};
use mesh_io::save_stl;
use mesh_loft::{
    WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch, is_watertight,
};
use mesh_types::Aabb;

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

/// Marks the HUD status-text node (the dynamic half).
#[derive(Component)]
struct HudText;

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
     Esc          (in review) back";

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // The brush, undo, filter, and active-body switching. Its default
        // config matches this tool's original constants (ivory base, red
        // highlight, 4 mm brush, 35° filter).
        .add_plugins(MeshPaintPlugin::default())
        .init_resource::<ShowDisc>()
        .init_resource::<DiscEntity>()
        .init_resource::<LoftedDisc>()
        .add_systems(Startup, setup)
        // Paint-aware orbit: left-drag orbits unless Shift is held for painting.
        .add_systems(
            Update,
            (orbit_when_not_painting, update_orbit_camera).chain(),
        )
        .add_systems(
            Update,
            (loft_disc, export_disc, leave_review, update_hud, update_display),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let l4 = load_from_env("CF_L4_STL").expect("set $CF_L4_STL (superior vertebra, FMA13075)");
    let l5 = load_from_env("CF_L5_STL").expect("set $CF_L5_STL (inferior vertebra, FMA13076)");

    // Seed each render mesh with the same base colour the brush restores on
    // erase (the plugin's default config).
    let base = MeshPaintConfig::default().base_color;

    let mut entities = Vec::new();
    for (mesh, name) in [(&l4, "L4"), (&l5, "L5")] {
        let handle = meshes.add(paint_render_mesh(mesh, UpAxis::PlusZ, base));
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
                MeshMaterial3d(material),
                Transform::default(),
                PaintBody::new(name, mesh.clone(), handle),
            ))
            .id();
        entities.push(entity);
    }
    commands.insert_resource(PaintTargets::new(entities));

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

/// Refresh the on-screen HUD: controls + live status, read from the plugin's
/// public paint-state resources.
fn update_hud(
    targets: Res<PaintTargets>,
    mode: Res<BrushMode>,
    brush: Res<Brush>,
    filter: Res<NormalFilter>,
    q_bodies: Query<&PaintBody>,
    mut q_text: Query<&mut Text, With<HudText>>,
) {
    let Ok(mut text) = q_text.single_mut() else {
        return;
    };
    let active_name = targets
        .active_entity()
        .and_then(|e| q_bodies.get(e).ok())
        .map_or("?", PaintBody::name);
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
    for &e in targets.entities() {
        if let Ok(b) = q_bodies.get(e) {
            counts.push_str(&format!("  {:<11}{}\n", b.name(), b.painted_count()));
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
        brush.radius,
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
        if body.painted_count() < 3 {
            continue;
        }
        let faces: Vec<usize> = body.painted().iter().copied().collect();
        let patch = finalize_patch(&extract_patch(body.source(), &faces));
        match body.name() {
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
    // `finalize_patch` prevents the disqualifying failures (a hole in a cap, a
    // disconnected shell). A few open wall-seam edges can remain and are fine —
    // the disc still renders, exports, and tet-meshes — so this is an
    // informational note, not a gate.
    if !is_watertight(&disc) {
        println!(
            "loft: disc has a minor wall seam (not fully watertight) — still tet-meshable, proceeding"
        );
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

/// Export the last lofted disc to an STL with `S` and print a command to bond it
/// between two abstract plates (`real-disc-bonded`). The path is `$CF_DISC_OUT`,
/// or `painted_disc.stl` in the temp dir.
///
/// The full **anatomical FSU** is its own app now, `cf-spine-studio`: it paints
/// the two endplate patches on the *real* L4/L5 vertebrae in-window and lofts +
/// solves the coupled FSU (disc bushing + field-derived ligaments + facet
/// contacts) as one moment-driven equilibrium ([`cf_fsu_model::CoupledFsu`]) — so
/// it paints its own disc and does not take an exported STL. If the vertebra STLs
/// are known (`$CF_L4_STL` / `$CF_L5_STL`), the printed command launches it.
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
    // What the exported disc feeds: bond it between two abstract plates.
    println!(
        "  bond between two plates: CF_DISC_STL={p} cargo run --release -p sim-bevy-soft --example real-disc-bonded"
    );
    // The full anatomical FSU is the cf-spine-studio app — it paints its OWN disc
    // on the real vertebrae, so it doesn't consume the exported STL. Reuse the
    // vertebra paths the tool was loaded with.
    match (std::env::var("CF_L4_STL"), std::env::var("CF_L5_STL")) {
        (Ok(l4), Ok(l5)) => println!(
            "  anatomical FSU (paint + solve on the real L4/L5, + ligaments + facets):\n\
             \x20   cargo run --release -p cf-spine-studio -- --l4 {l4} --l5 {l5}"
        ),
        _ => println!(
            "  anatomical FSU: cargo run --release -p cf-spine-studio -- --l4 <L4.stl> --l5 <L5.stl>"
        ),
    }
}

/// Leave disc review with `Esc`, returning to painting (the active body opaque).
/// `Tab` is the plugin's active-body switch, so review-exit uses its own key.
fn leave_review(keys: Res<ButtonInput<KeyCode>>, mut show: ResMut<ShowDisc>) {
    if show.0 && keys.just_pressed(KeyCode::Escape) {
        show.0 = false;
    }
}

/// Update what is shown: in review (disc lofted) both vertebrae are translucent
/// and the disc is visible; otherwise only the active body is shown, opaque.
fn update_display(
    targets: Res<PaintTargets>,
    show: Res<ShowDisc>,
    disc_entity: Res<DiscEntity>,
    q_mat: Query<&MeshMaterial3d<StandardMaterial>>,
    mut q_vis: Query<&mut Visibility>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !targets.is_changed() && !show.is_changed() && !disc_entity.is_changed() {
        return;
    }
    for (i, &e) in targets.entities().iter().enumerate() {
        if let Ok(mut visibility) = q_vis.get_mut(e) {
            *visibility = if show.0 || i == targets.active_index() {
                Visibility::Visible
            } else {
                Visibility::Hidden
            };
        }
        if let Ok(mat) = q_mat.get(e) {
            if let Some(material) = materials.get_mut(&mat.0) {
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
