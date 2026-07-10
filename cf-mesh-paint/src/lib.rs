//! Reusable Bevy brush-painting plugin over the [`mesh_select`] primitive.
//!
//! [`MeshPaintPlugin`] adds the interaction glue — a round surface brush that
//! paints / erases face regions on one or more meshes, with a normal-similarity
//! filter, per-drag undo, and per-body active selection — on top of the
//! `mesh-select` face-id query. It is the front-end for any workflow that needs
//! a human to select a region of a mesh: disc-endplate patches to loft, tendon
//! attachment sites, subtractive bore footprints, scan trimming.
//!
//! Like its sibling `cf-bevy-common`, this crate is **UI-free**: it owns the
//! paint *state* and *interaction*, not a HUD. A consumer reads the public
//! state resources ([`Brush`], [`BrushMode`], [`NormalFilter`],
//! [`PaintTargets`]) and each body's [`PaintBody::painted_count`] to render its
//! own HUD (the workspace convention is egui — see the `paint-mesh` example).
//!
//! # Using it
//!
//! 1. Add [`MeshPaintPlugin`] (tune defaults via [`MeshPaintConfig`]).
//! 2. For each paintable mesh, build its render mesh with [`paint_render_mesh`]
//!    (this preserves the picking contract), spawn `(Mesh3d, your material,
//!    Transform, PaintBody::new(..))`, and collect the entity.
//! 3. `insert_resource(PaintTargets::new(entities))`.
//! 4. Wire a camera that yields to the brush — either your own (checking
//!    [`painting`]) or the provided [`orbit_when_not_painting`] chained before
//!    `update_orbit_camera`.
//! 5. Read the selection back out by querying `&PaintBody`
//!    ([`PaintBody::painted`] + [`PaintBody::source`]).
//!
//! # Controls
//!
//! `Shift + left-drag` paints / erases the active body; `Tab` cycles the active
//! body; `E` toggles paint / erase; `N` toggles the normal filter, `-` / `=`
//! widen / tighten it; `[` / `]` resize the brush; `Ctrl`/`Cmd + Z` undoes the
//! last stroke; `C` clears the active body.

pub mod body;
pub mod brush;
pub mod input;
pub mod stroke;

use bevy::prelude::*;

pub use body::{PaintBody, PaintTargets, paint_render_mesh};
pub use brush::{Brush, BrushMode, Hover, NormalFilter, PaintColors, PaintingBlocked};
pub use input::{orbit_when_not_painting, painting, undo_pressed};
pub use stroke::History;

// Internal-only: the plugin owns the in-progress stroke; a consumer has no use
// for it, so it is not part of the public surface (unlike `History`, which a
// HUD can read).
use stroke::ActiveStroke;

/// Configurable defaults for [`MeshPaintPlugin`]: the paint / base colors, the
/// brush radius and its bounds, and the normal filter's initial state. The
/// `base_color` here must match the color you pass to [`paint_render_mesh`].
#[derive(Clone, Copy, Debug)]
pub struct MeshPaintConfig {
    /// Unpainted face color (also the [`paint_render_mesh`] seed).
    pub base_color: [f32; 4],
    /// Painted face color.
    pub highlight_color: [f32; 4],
    /// Initial brush radius (native mesh units).
    pub brush_init: f64,
    /// Minimum brush radius.
    pub brush_min: f64,
    /// Maximum brush radius.
    pub brush_max: f64,
    /// Whether the normal-similarity filter starts enabled.
    pub filter_enabled: bool,
    /// Initial normal-filter tolerance (degrees).
    pub filter_max_angle_deg: f64,
}

impl Default for MeshPaintConfig {
    fn default() -> Self {
        Self {
            base_color: [0.80, 0.78, 0.72, 1.0],
            highlight_color: [0.90, 0.30, 0.20, 1.0],
            brush_init: 4.0,
            brush_min: 0.5,
            brush_max: 30.0,
            filter_enabled: true,
            filter_max_angle_deg: 35.0,
        }
    }
}

/// The brush-painting plugin: wires the paint state resources (from `config`)
/// and the per-frame interaction systems. Add it, register paintable bodies via
/// [`PaintTargets`], and read the selection back out from each [`PaintBody`].
///
/// It does **not** wire a camera — pair it with `cf_bevy_common`'s orbit camera
/// (using [`orbit_when_not_painting`] so the brush and orbit don't fight) — nor
/// a HUD, which a consumer renders from the public state resources.
#[derive(Default)]
pub struct MeshPaintPlugin {
    /// Initial brush / color / filter defaults.
    pub config: MeshPaintConfig,
}

impl MeshPaintPlugin {
    /// Construct the plugin with an explicit configuration.
    #[must_use]
    pub fn new(config: MeshPaintConfig) -> Self {
        Self { config }
    }
}

impl Plugin for MeshPaintPlugin {
    fn build(&self, app: &mut App) {
        let c = self.config;
        app.init_resource::<PaintTargets>()
            .init_resource::<Hover>()
            .init_resource::<History>()
            .init_resource::<ActiveStroke>()
            .init_resource::<PaintingBlocked>()
            .insert_resource(PaintColors {
                base: c.base_color,
                highlight: c.highlight_color,
            })
            .insert_resource(Brush {
                radius: c.brush_init,
                min: c.brush_min,
                max: c.brush_max,
            })
            .insert_resource(BrushMode::Paint)
            .insert_resource(NormalFilter {
                enabled: c.filter_enabled,
                max_angle_deg: c.filter_max_angle_deg,
            })
            // The stroke pipeline is ordered: hover sets the target face, the
            // brush flips faces, finalize closes the stroke, the ring draws it.
            .add_systems(
                Update,
                (
                    brush::hover_ray,
                    brush::apply_brush,
                    stroke::finalize_stroke,
                    brush::draw_brush,
                )
                    .chain(),
            )
            // Order-independent controls.
            .add_systems(
                Update,
                (
                    brush::adjust_brush,
                    brush::toggle_mode,
                    brush::toggle_filter,
                    input::switch_body,
                    input::clear_selection,
                    stroke::undo_stroke,
                ),
            );
    }
}

/// Convenience re-exports: `use cf_mesh_paint::prelude::*;` brings in the types
/// a consumer wires with — the plugin, its config, the body + registry types,
/// the render-mesh builder, and the paint-aware camera helper.
pub mod prelude {
    pub use crate::body::{PaintBody, PaintTargets, paint_render_mesh};
    pub use crate::brush::PaintingBlocked;
    pub use crate::input::{orbit_when_not_painting, painting};
    pub use crate::{Brush, BrushMode, MeshPaintConfig, MeshPaintPlugin, NormalFilter};
}

#[cfg(test)]
mod system_tests {
    //! Headless drives of the interaction systems through a raw `World`: the
    //! brush actually paints/erases, undo reverts, the keyboard controls flip
    //! state, and the paint-aware camera orbits. `run_system_once` runs one
    //! system in isolation (the plugin chains `hover_ray` ahead of
    //! `apply_brush`, so we set `Hover` by hand and run the brush alone).

    use bevy::ecs::system::RunSystemOnce;
    use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
    use bevy::prelude::*;
    use cf_bevy_common::axis::UpAxis;
    use cf_bevy_common::prelude::OrbitCamera;
    use cf_geometry::IndexedMesh;

    use crate::body::{PaintBody, PaintTargets, paint_render_mesh};
    use crate::brush::{
        Brush, BrushMode, Hover, NormalFilter, PaintColors, PaintingBlocked, adjust_brush,
        apply_brush, hover_ray, toggle_filter, toggle_mode,
    };
    use crate::input::{clear_selection, orbit_when_not_painting, switch_body};
    use crate::stroke::{ActiveStroke, History, finalize_stroke, undo_stroke};

    const BASE: [f32; 4] = [0.80, 0.78, 0.72, 1.0];
    const HL: [f32; 4] = [0.90, 0.30, 0.20, 1.0];

    /// Two triangles sharing an edge — face ids 0 and 1, nearly coplanar so a
    /// wide brush covers both.
    fn two_tris() -> IndexedMesh {
        IndexedMesh::from_raw(
            &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0],
            &[0, 1, 2, 1, 3, 2],
        )
    }

    fn pressed<T: Copy + Eq + std::hash::Hash + Send + Sync + 'static>(
        buttons: &[T],
    ) -> ButtonInput<T> {
        let mut input = ButtonInput::default();
        for &b in buttons {
            input.press(b);
        }
        input
    }

    /// A world with one painted-ready body under the cursor (`Hover` face 0), a
    /// wide brush, and `Shift + left` held — ready to run `apply_brush`.
    fn paint_world() -> (World, Entity) {
        let mut world = World::new();
        let source = two_tris();
        let mut meshes = Assets::<Mesh>::default();
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, BASE));
        world.insert_resource(meshes);
        let entity = world.spawn(PaintBody::new("A", source, handle)).id();
        world.insert_resource(PaintTargets::new(vec![entity]));
        world.insert_resource(PaintColors {
            base: BASE,
            highlight: HL,
        });
        world.insert_resource(Brush {
            radius: 100.0, // covers both faces of the unit mesh
            min: 0.5,
            max: 30.0,
        });
        world.insert_resource(BrushMode::Paint);
        world.insert_resource(NormalFilter {
            enabled: false,
            max_angle_deg: 90.0,
        });
        world.insert_resource(Hover {
            point: Some(Vec3::ZERO),
            normal: Vec3::Z,
            face: Some(0),
        });
        world.insert_resource(ActiveStroke::default());
        world.insert_resource(History::default());
        world.insert_resource(PaintingBlocked(false));
        world.insert_resource(pressed(&[MouseButton::Left]));
        world.insert_resource(pressed(&[KeyCode::ShiftLeft]));
        (world, entity)
    }

    fn painted_count(world: &World, e: Entity) -> usize {
        world
            .get::<PaintBody>(e)
            .map_or(0, PaintBody::painted_count)
    }

    #[test]
    fn apply_brush_paints_covered_faces() {
        let (mut world, e) = paint_world();
        assert!(world.run_system_once(apply_brush).is_ok());
        assert_eq!(painted_count(&world, e), 2);
    }

    #[test]
    fn apply_brush_without_shift_does_nothing() {
        let (mut world, e) = paint_world();
        world.insert_resource(pressed::<KeyCode>(&[])); // no Shift
        assert!(world.run_system_once(apply_brush).is_ok());
        assert_eq!(painted_count(&world, e), 0);
    }

    #[test]
    fn erase_removes_painted_faces() {
        let (mut world, e) = paint_world();
        assert!(world.run_system_once(apply_brush).is_ok());
        assert_eq!(painted_count(&world, e), 2);
        // Switch to erase, keep the same hover, run again → back to empty.
        world.insert_resource(BrushMode::Erase);
        world.insert_resource(ActiveStroke::default());
        assert!(world.run_system_once(apply_brush).is_ok());
        assert_eq!(painted_count(&world, e), 0);
    }

    #[test]
    fn finalize_then_undo_reverts_a_stroke() {
        let (mut world, e) = paint_world();
        assert!(world.run_system_once(apply_brush).is_ok());
        // Release the brush → the stroke closes onto the history.
        world.insert_resource(pressed::<MouseButton>(&[]));
        world.insert_resource(pressed::<KeyCode>(&[]));
        assert!(world.run_system_once(finalize_stroke).is_ok());
        assert_eq!(world.resource::<History>().len(), 1);
        // Ctrl+Z → the two painted faces revert.
        world.insert_resource(pressed(&[KeyCode::ControlLeft, KeyCode::KeyZ]));
        assert!(world.run_system_once(undo_stroke).is_ok());
        assert_eq!(painted_count(&world, e), 0);
        assert_eq!(world.resource::<History>().len(), 0);
    }

    #[test]
    fn clear_selection_empties_the_active_body() {
        let (mut world, e) = paint_world();
        assert!(world.run_system_once(apply_brush).is_ok());
        assert_eq!(painted_count(&world, e), 2);
        world.insert_resource(pressed(&[KeyCode::KeyC]));
        assert!(world.run_system_once(clear_selection).is_ok());
        assert_eq!(painted_count(&world, e), 0);
    }

    #[test]
    fn blocked_hover_ray_clears_the_hover() {
        let (mut world, _e) = paint_world();
        world.insert_resource(PaintingBlocked(true));
        assert!(world.run_system_once(hover_ray).is_ok());
        assert_eq!(world.resource::<Hover>().face, None);
    }

    #[test]
    fn toggle_mode_flips_paint_and_erase() {
        let mut world = World::new();
        world.insert_resource(BrushMode::Paint);
        world.insert_resource(pressed(&[KeyCode::KeyE]));
        assert!(world.run_system_once(toggle_mode).is_ok());
        assert_eq!(*world.resource::<BrushMode>(), BrushMode::Erase);
    }

    #[test]
    fn toggle_filter_and_tolerance() {
        let mut world = World::new();
        world.insert_resource(NormalFilter {
            enabled: true,
            max_angle_deg: 35.0,
        });
        world.insert_resource(pressed(&[KeyCode::KeyN, KeyCode::Minus]));
        assert!(world.run_system_once(toggle_filter).is_ok());
        let f = world.resource::<NormalFilter>();
        assert!(!f.enabled);
        assert_eq!(f.max_angle_deg, 40.0); // widened by 5°
    }

    #[test]
    fn adjust_brush_shrinks_and_clamps() {
        let mut world = World::new();
        world.insert_resource(Brush {
            radius: 0.6,
            min: 0.5,
            max: 30.0,
        });
        world.insert_resource(pressed(&[KeyCode::BracketLeft]));
        assert!(world.run_system_once(adjust_brush).is_ok());
        // 0.6 * 0.8 = 0.48 → clamped up to the 0.5 minimum.
        assert_eq!(world.resource::<Brush>().radius, 0.5);
    }

    #[test]
    fn switch_body_cycles_the_active_target() {
        let mut world = World::new();
        let (a, b) = (world.spawn_empty().id(), world.spawn_empty().id());
        world.insert_resource(PaintTargets::new(vec![a, b]));
        world.insert_resource(pressed(&[KeyCode::Tab]));
        assert!(world.run_system_once(switch_body).is_ok());
        assert_eq!(world.resource::<PaintTargets>().active_entity(), Some(b));
    }

    #[test]
    fn orbit_when_not_painting_orbits_without_shift() {
        let mut world = World::new();
        let cam = world.spawn(OrbitCamera::default()).id();
        world.insert_resource(pressed(&[MouseButton::Left]));
        world.insert_resource(pressed::<KeyCode>(&[])); // no Shift → orbit
        world.insert_resource(AccumulatedMouseMotion {
            delta: Vec2::new(10.0, 0.0),
        });
        world.insert_resource(AccumulatedMouseScroll::default());
        let before = world.get::<OrbitCamera>(cam).map(|c| c.azimuth);
        assert!(world.run_system_once(orbit_when_not_painting).is_ok());
        let after = world.get::<OrbitCamera>(cam).map(|c| c.azimuth);
        assert_ne!(before, after);
    }

    #[test]
    fn orbit_suppressed_while_painting() {
        let mut world = World::new();
        let cam = world.spawn(OrbitCamera::default()).id();
        world.insert_resource(pressed(&[MouseButton::Left]));
        world.insert_resource(pressed(&[KeyCode::ShiftLeft])); // Shift → no orbit
        world.insert_resource(AccumulatedMouseMotion {
            delta: Vec2::new(10.0, 0.0),
        });
        world.insert_resource(AccumulatedMouseScroll::default());
        let before = world.get::<OrbitCamera>(cam).map(|c| c.azimuth);
        assert!(world.run_system_once(orbit_when_not_painting).is_ok());
        let after = world.get::<OrbitCamera>(cam).map(|c| c.azimuth);
        assert_eq!(before, after);
    }
}
