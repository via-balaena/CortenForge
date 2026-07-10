//! The brush: its state resources and the per-frame paint systems.
//!
//! [`hover_ray`] ray-casts the cursor onto the active body, [`apply_brush`]
//! paints/erases the faces the brush covers while `Shift + left` is held, and
//! [`draw_brush`] draws the brush ring on the surface. `E` / `N` / `-` `=` /
//! `[` `]` toggle mode, toggle the normal filter, tune its tolerance, and
//! resize the brush.

use bevy::mesh::VertexAttributeValues;
use bevy::picking::mesh_picking::ray_cast::{MeshRayCast, MeshRayCastSettings};
use bevy::prelude::*;

use crate::body::{PaintBody, PaintTargets, recolor};
use crate::input::painting;
use crate::stroke::{ActiveStroke, Stroke};

/// The paint / erase base colors a body's faces are driven to. Seeded from
/// [`MeshPaintConfig`](crate::MeshPaintConfig); `base` must match the color
/// [`paint_render_mesh`](crate::paint_render_mesh) seeded the mesh with.
#[derive(Resource, Clone, Copy, Debug)]
pub struct PaintColors {
    /// Unpainted face color (what erase restores).
    pub base: [f32; 4],
    /// Painted face color.
    pub highlight: [f32; 4],
}

/// The brush radius (native mesh units) and its bounds.
#[derive(Resource, Clone, Copy, Debug)]
pub struct Brush {
    /// Current radius.
    pub radius: f64,
    /// Lower clamp (`[` shrinks toward it).
    pub min: f64,
    /// Upper clamp (`]` grows toward it).
    pub max: f64,
}

/// Whether the brush paints or erases (toggled with `E`).
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug)]
pub enum BrushMode {
    /// Add faces to the selection.
    Paint,
    /// Remove faces from the selection.
    Erase,
}

/// Restrict the brush to faces whose normal is within `max_angle_deg` of the
/// face under the cursor — so painting a flat region doesn't spill onto steep
/// adjacent walls. Toggle with `N`, widen / tighten with `-` / `=`.
#[derive(Resource, Clone, Copy, Debug)]
pub struct NormalFilter {
    /// Whether the filter is active.
    pub enabled: bool,
    /// Maximum normal deviation, in degrees.
    pub max_angle_deg: f64,
}

/// The surface point under the cursor this frame (world space), its normal, and
/// the active body's face there — set by [`hover_ray`], read by the brush ring
/// and the stroke.
#[derive(Resource, Default, Debug)]
pub struct Hover {
    /// World-space hit point, or `None` when the cursor is off the active body.
    pub point: Option<Vec3>,
    /// Hit-face normal (world space).
    pub normal: Vec3,
    /// Hit face id on the active body, or `None`.
    pub face: Option<usize>,
}

/// When set, the brush is suppressed: [`hover_ray`] clears the hover so neither
/// the ring draws nor [`apply_brush`] paints. A consumer sets this while the
/// pointer is over its own UI (e.g. an egui panel) so a `Shift`-click on the
/// panel doesn't paint the mesh behind it — the plugin is UI-agnostic, so this
/// is the seam a consumer uses to arbitrate the pointer.
#[derive(Resource, Default)]
pub struct PaintingBlocked(pub bool);

/// Brush-ring color while painting.
const RING_PAINT: Color = Color::srgb(0.95, 0.35, 0.25);
/// Brush-ring color while erasing.
const RING_ERASE: Color = Color::srgb(0.30, 0.80, 0.90);

/// Ray-cast the cursor every frame, recording the hit only when the **active**
/// body is the front-most surface — so the brush ring and stroke stay on the
/// body being painted.
pub fn hover_ray(
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    targets: Res<PaintTargets>,
    blocked: Res<PaintingBlocked>,
    mut ray_cast: MeshRayCast,
    mut hover: ResMut<Hover>,
) {
    hover.point = None;
    hover.face = None;
    if blocked.0 {
        return;
    }
    let Some(active) = targets.active_entity() else {
        return;
    };
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

/// While `Shift + left` is held, paint (or erase) the faces the brush covers on
/// the active body, within the brush radius and — if enabled — the
/// normal-similarity tolerance.
// A Bevy system takes each resource/query as a distinct parameter by design;
// the paint step legitimately reads input + brush state + colors + the target.
#[allow(clippy::too_many_arguments)]
pub fn apply_brush(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<BrushMode>,
    brush: Res<Brush>,
    filter: Res<NormalFilter>,
    colors: Res<PaintColors>,
    hover: Res<Hover>,
    targets: Res<PaintTargets>,
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
    let Some(active_entity) = targets.active_entity() else {
        return;
    };
    let Ok(mut body) = q_bodies.get_mut(active_entity) else {
        return;
    };
    let PaintBody {
        mesh: handle,
        field,
        painted,
        ..
    } = body.as_mut();

    let Some(mesh) = meshes.get_mut(&*handle) else {
        return;
    };
    let Some(VertexAttributeValues::Float32x4(face_colors)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    else {
        return;
    };

    // Begin (or continue) the current stroke, so it can be undone as a unit.
    let stroke = stroke.0.get_or_insert_with(|| Stroke {
        body: active_entity,
        mode: *mode,
        faces: Vec::new(),
    });

    let normal_filter = filter.enabled.then_some(filter.max_angle_deg);
    for f in field.brush(face, brush.radius, normal_filter) {
        let changed = match stroke.mode {
            BrushMode::Paint => {
                let inserted = painted.insert(f);
                if inserted {
                    recolor(face_colors, f, colors.highlight);
                }
                inserted
            }
            BrushMode::Erase => {
                let removed = painted.remove(&f);
                if removed {
                    recolor(face_colors, f, colors.base);
                }
                removed
            }
        };
        if changed {
            stroke.faces.push(f);
        }
    }
}

/// Draw the brush as a ring on the surface, colored by mode.
pub fn draw_brush(mut gizmos: Gizmos, hover: Res<Hover>, brush: Res<Brush>, mode: Res<BrushMode>) {
    let Some(point) = hover.point else {
        return;
    };
    let color = match *mode {
        BrushMode::Paint => RING_PAINT,
        BrushMode::Erase => RING_ERASE,
    };
    let rotation = Quat::from_rotation_arc(Vec3::Z, hover.normal.normalize_or_zero());
    gizmos.circle(Isometry3d::new(point, rotation), brush.radius as f32, color);
}

/// Shrink / grow the brush with `[` / `]`.
pub fn adjust_brush(keys: Res<ButtonInput<KeyCode>>, mut brush: ResMut<Brush>) {
    if keys.just_pressed(KeyCode::BracketLeft) {
        brush.radius = (brush.radius * 0.8).max(brush.min);
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        brush.radius = (brush.radius * 1.25).min(brush.max);
    }
}

/// Toggle paint / erase with `E`.
pub fn toggle_mode(keys: Res<ButtonInput<KeyCode>>, mut mode: ResMut<BrushMode>) {
    if keys.just_pressed(KeyCode::KeyE) {
        *mode = match *mode {
            BrushMode::Paint => BrushMode::Erase,
            BrushMode::Erase => BrushMode::Paint,
        };
    }
}

/// Toggle the normal-similarity filter with `N`; widen / tighten it with
/// `-` / `=`.
pub fn toggle_filter(keys: Res<ButtonInput<KeyCode>>, mut filter: ResMut<NormalFilter>) {
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
