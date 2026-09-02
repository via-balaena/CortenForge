//! Pointer arbitration between egui and the 3D view.
//!
//! While the pointer is over a panel, the orbit camera is suppressed by zeroing
//! the accumulated motion/scroll, so dragging a slider does not also spin the
//! body behind it. Six copies of this rule exist across the workspace's Bevy
//! tools; promoting it into `cf-viewer` is deliberately deferred.

use bevy::prelude::*;
use bevy_egui::EguiContexts;

/// Zero the orbit camera's input while the pointer is over egui.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn arbitrate_pointer_over_egui(
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
