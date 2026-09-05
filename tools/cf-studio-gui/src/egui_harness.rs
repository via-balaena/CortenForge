//! Driving a screen as the Bevy system it really is.
//!
//! `bevy_egui`'s plugin needs a window and a render device, so a system taking
//! `EguiContexts` looks untestable — it panics before it draws anything. But
//! all the plugin does around the system is open and close an egui pass, so
//! [`begin`] and [`end`] do that by hand and the system becomes drivable: read
//! back what it painted, and click it.
//!
//! ⚠ This is the only thing that catches a screen deleted wholesale. Drawing
//! leaves nothing in the ECS to observe, so replacing a whole screen system
//! with a no-op passes every other kind of test in this crate.

#![allow(clippy::expect_used)]

use bevy::prelude::*;
use bevy_egui::{EguiContext, EguiUserTextures, PrimaryEguiContext, egui};

/// What one frame painted, and where.
#[derive(Resource, Default)]
pub(crate) struct Painted(pub(crate) Vec<(String, egui::Rect)>);

/// The click to deliver on the next frame, if any.
#[derive(Resource, Default)]
pub(crate) struct Click(pub(crate) Option<egui::Pos2>);

/// `main.rs`'s opening resolution, so screens lay out at the size the app
/// really runs at.
const SCREEN: egui::Vec2 = egui::Vec2::new(1280.0, 900.0);

/// Open the pass the plugin would have, delivering any click [`click_on`] left.
pub(crate) fn begin(mut contexts: Query<&mut EguiContext>, click: Res<Click>) {
    let Some(mut context) = contexts.iter_mut().next() else {
        return;
    };
    let mut events = Vec::new();
    if let Some(pos) = click.0 {
        events.push(egui::Event::PointerMoved(pos));
        for pressed in [true, false] {
            events.push(egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: egui::Modifiers::NONE,
            });
        }
    }
    context.get_mut().begin_pass(egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, SCREEN)),
        events,
        ..Default::default()
    });
}

/// Close it, and record every piece of text the frame painted.
pub(crate) fn end(mut contexts: Query<&mut EguiContext>, mut painted: ResMut<Painted>) {
    let Some(mut context) = contexts.iter_mut().next() else {
        return;
    };
    painted.0.clear();
    for shape in &context.get_mut().end_pass().shapes {
        if let egui::epaint::Shape::Text(text) = &shape.shape {
            painted.0.push((
                text.galley.text().to_owned(),
                egui::Rect::from_min_size(text.pos, text.galley.size()),
            ));
        }
    }
}

/// An app holding the pass's own state, for a screen's systems to be chained
/// between [`begin`] and [`end`]. Whatever else the screen needs is the
/// caller's to add.
pub(crate) fn app() -> App {
    let mut app = App::new();
    app.init_resource::<EguiUserTextures>()
        .init_resource::<Painted>()
        .init_resource::<Click>();
    app.world_mut().spawn(PrimaryEguiContext);
    app
}

/// Everything the last frame painted, text only.
pub(crate) fn painted_texts(app: &App) -> Vec<String> {
    app.world()
        .resource::<Painted>()
        .0
        .iter()
        .map(|(text, _)| text.clone())
        .collect()
}

/// Click the middle of whatever was painted starting with `text`, next frame.
///
/// ⚠ Two frames, and that is the point: egui places a widget from the previous
/// pass, so a click can only land where the frame before it drew.
pub(crate) fn click_on(app: &mut App, text: &str) {
    let painted = &app.world().resource::<Painted>().0;
    let at = painted
        .iter()
        .find(|(shown, _)| shown.starts_with(text))
        .map(|(_, rect)| rect.center());
    assert!(at.is_some(), "nothing painted {text:?}: {painted:?}");
    app.world_mut().resource_mut::<Click>().0 = at;
    app.update();
    app.world_mut().resource_mut::<Click>().0 = None;
}
