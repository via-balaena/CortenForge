//! Pointer arbitration between egui and the 3D view. While the pointer is over
//! an egui panel it suppresses both the orbit camera (zero the accumulated
//! motion/scroll) and the paint brush (set [`PaintingBlocked`]), so a drag on the
//! panel neither orbits nor paints the vertebra behind it. The brush is also blocked
//! outside Design, so a Shift+drag while inspecting the Preview disc (a side panel, so
//! the 3D view is exposed) can't silently repaint the held build's bodies.

use bevy::prelude::*;
use bevy_egui::EguiContexts;
use cf_mesh_paint::prelude::PaintingBlocked;

use crate::state::StudioState;

/// Suppress orbit input while the pointer is over the egui panel, and the paint brush
/// both there and in any non-Design state (painting only ever happens in Design).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn arbitrate_pointer_over_egui(
    mut contexts: EguiContexts,
    state: Res<State<StudioState>>,
    mut blocked: ResMut<PaintingBlocked>,
    mut motion: ResMut<bevy::input::mouse::AccumulatedMouseMotion>,
    mut scroll: ResMut<bevy::input::mouse::AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let over_panel = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
    blocked.0 = over_panel || *state.get() != StudioState::Design;
    if over_panel {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}
