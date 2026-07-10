//! Stroke history — grouping a drag into one undoable unit.
//!
//! [`apply_brush`](crate::brush::apply_brush) accumulates the faces it flips
//! into the [`ActiveStroke`]; [`finalize_stroke`] closes it onto the [`History`]
//! when the brush is released, and [`undo_stroke`] re-flips the last stroke's
//! faces on `Ctrl`/`Cmd + Z`.

use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;

use crate::body::{PaintBody, PaintTargets, recolor};
use crate::brush::{BrushMode, PaintColors};

/// One paint/erase stroke: the faces whose selection state it flipped (so it
/// can be undone) and which body they belong to. `mode` records whether the
/// stroke painted or erased them.
pub(crate) struct Stroke {
    pub(crate) body: Entity,
    pub(crate) mode: BrushMode,
    pub(crate) faces: Vec<usize>,
}

/// Undo stack of finished strokes.
#[derive(Resource, Default)]
pub struct History(pub(crate) Vec<Stroke>);

impl History {
    /// How many strokes can still be undone.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether there is nothing to undo.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The stroke currently being drawn (accumulates while `Shift + left` is held).
#[derive(Resource, Default)]
pub struct ActiveStroke(pub(crate) Option<Stroke>);

/// Close the current stroke once the brush is released, pushing it (if it
/// changed anything) onto the undo history.
pub fn finalize_stroke(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut stroke: ResMut<ActiveStroke>,
    mut history: ResMut<History>,
) {
    if stroke.0.is_none() {
        return;
    }
    if crate::input::painting(&keys) && mouse.pressed(MouseButton::Left) {
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
pub fn undo_stroke(
    keys: Res<ButtonInput<KeyCode>>,
    colors: Res<PaintColors>,
    mut history: ResMut<History>,
    mut targets: ResMut<PaintTargets>,
    mut q_bodies: Query<&mut PaintBody>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !crate::input::undo_pressed(&keys) {
        return;
    }
    let Some(stroke) = history.0.pop() else {
        return;
    };
    targets.set_active(stroke.body);
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
    let Some(VertexAttributeValues::Float32x4(face_colors)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
    else {
        return;
    };
    for &f in &stroke.faces {
        match stroke.mode {
            BrushMode::Paint => {
                painted.remove(&f);
                recolor(face_colors, f, colors.base);
            }
            BrushMode::Erase => {
                painted.insert(f);
                recolor(face_colors, f, colors.highlight);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_history_reports_empty() {
        let h = History::default();
        assert!(h.is_empty());
        assert_eq!(h.len(), 0);
    }

    #[test]
    fn history_len_counts_pushed_strokes() {
        let mut world = World::new();
        let mut h = History::default();
        h.0.push(Stroke {
            body: world.spawn_empty().id(),
            mode: BrushMode::Paint,
            faces: vec![0, 1],
        });
        assert!(!h.is_empty());
        assert_eq!(h.len(), 1);
    }
}
