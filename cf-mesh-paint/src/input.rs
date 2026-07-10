//! Input helpers and the non-brush interaction systems: the paint modifier and
//! undo chord, active-body cycling, clearing, and an optional paint-aware
//! orbit-camera system.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;
use cf_bevy_common::prelude::OrbitCamera;

use crate::body::{PaintBody, PaintTargets, recolour};
use crate::brush::PaintColors;

/// Whether a `Shift` key (the paint modifier) is held. Public so a consumer's
/// own camera system can suppress orbiting while the user paints.
#[must_use]
pub fn painting(keys: &ButtonInput<KeyCode>) -> bool {
    keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight)
}

/// Whether the undo chord (`Ctrl`/`Cmd + Z`) was just pressed.
#[must_use]
pub fn undo_pressed(keys: &ButtonInput<KeyCode>) -> bool {
    let modifier = keys.pressed(KeyCode::ControlLeft)
        || keys.pressed(KeyCode::ControlRight)
        || keys.pressed(KeyCode::SuperLeft)
        || keys.pressed(KeyCode::SuperRight);
    modifier && keys.just_pressed(KeyCode::KeyZ)
}

/// Cycle the active paintable body with `Tab`.
pub fn switch_body(keys: Res<ButtonInput<KeyCode>>, mut targets: ResMut<PaintTargets>) {
    if keys.just_pressed(KeyCode::Tab) {
        targets.cycle();
    }
}

/// Clear the active body's selection with `C`.
pub fn clear_selection(
    keys: Res<ButtonInput<KeyCode>>,
    colors: Res<PaintColors>,
    targets: Res<PaintTargets>,
    mut q_bodies: Query<&mut PaintBody>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !keys.just_pressed(KeyCode::KeyC) {
        return;
    }
    let Some(active) = targets.active_entity() else {
        return;
    };
    let Ok(mut body) = q_bodies.get_mut(active) else {
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
                recolour(colours, f, colors.base);
            }
        }
    }
    body.painted.clear();
}

/// A drop-in orbit-camera input system that yields to the brush: it orbits on
/// left-drag **unless** `Shift` is held (the paint modifier), and always pans on
/// right-drag and zooms on scroll. Wire it in place of
/// [`orbit_camera_input`](cf_bevy_common::prelude::orbit_camera_input), chained
/// before `update_orbit_camera`, when the same window both paints and orbits.
pub fn orbit_when_not_painting(
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

#[cfg(test)]
mod tests {
    use super::*;

    fn keys(pressed: &[KeyCode], just: &[KeyCode]) -> ButtonInput<KeyCode> {
        let mut input = ButtonInput::default();
        for &k in pressed {
            input.press(k);
        }
        // `press` records both pressed + just_pressed; clear then re-mark the
        // ones that should read as held-but-not-just-pressed.
        for &k in pressed {
            if !just.contains(&k) {
                input.clear_just_pressed(k);
            }
        }
        input
    }

    #[test]
    fn painting_true_only_when_shift_held() {
        assert!(painting(&keys(&[KeyCode::ShiftLeft], &[])));
        assert!(painting(&keys(&[KeyCode::ShiftRight], &[])));
        assert!(!painting(&keys(&[KeyCode::KeyA], &[])));
    }

    #[test]
    fn undo_needs_modifier_and_just_pressed_z() {
        // Ctrl held + Z just pressed → undo.
        assert!(undo_pressed(&keys(
            &[KeyCode::ControlLeft, KeyCode::KeyZ],
            &[KeyCode::KeyZ]
        )));
        // Z just pressed without a modifier → no undo.
        assert!(!undo_pressed(&keys(&[KeyCode::KeyZ], &[KeyCode::KeyZ])));
        // Ctrl held but Z only held (not just pressed) → no undo.
        assert!(!undo_pressed(&keys(
            &[KeyCode::ControlLeft, KeyCode::KeyZ],
            &[]
        )));
    }
}
