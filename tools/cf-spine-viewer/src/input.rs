//! Pointer arbitration between egui and the 3D view. While the pointer is over
//! an egui panel it suppresses both the orbit camera (zero the accumulated
//! motion/scroll) and the paint brush (set [`PaintingBlocked`]), so a drag on the
//! panel neither orbits nor paints the vertebra behind it.

use bevy::prelude::*;
use bevy_egui::EguiContexts;
use cf_mesh_paint::prelude::PaintingBlocked;

/// Suppress orbit + paint input while the pointer is over the egui panel.
pub(crate) fn arbitrate_pointer_over_egui(
    mut contexts: EguiContexts,
    mut blocked: ResMut<PaintingBlocked>,
    mut motion: ResMut<bevy::input::mouse::AccumulatedMouseMotion>,
    mut scroll: ResMut<bevy::input::mouse::AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let over_panel = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
    blocked.0 = over_panel;
    if over_panel {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}
