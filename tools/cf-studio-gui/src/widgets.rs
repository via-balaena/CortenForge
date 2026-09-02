//! Small egui helpers shared by the screens.
//!
//! ⚠ Two egui defaults bite this app specifically, and both are wrapped here so
//! no call site has to remember them:
//!
//! - `ui.label()` does **not** wrap. Most text on these screens is
//!   multi-paragraph prose from `format_*`, so it must go through
//!   [`wrapped_label`] or it runs off the panel.
//! - panels do **not** scroll unless asked. Screens are laid out inside an
//!   explicit `ScrollArea`.

use bevy_egui::egui;

/// Cool green used for the "this succeeded, here's what happened" cards.
pub(crate) const GOOD_FILL: egui::Color32 = egui::Color32::from_rgb(0xee, 0xf3, 0xee);
/// Text on [`GOOD_FILL`].
pub(crate) const GOOD_TEXT: egui::Color32 = egui::Color32::from_rgb(0x2e, 0x4d, 0x2e);
/// The active-instruction blue of the pour assistant.
pub(crate) const ACTIVE_TEXT: egui::Color32 = egui::Color32::from_rgb(0x15, 0x65, 0xc0);
/// Error text — also the expired-pot-life red.
pub(crate) const ERROR_TEXT: egui::Color32 = egui::Color32::from_rgb(0xc6, 0x28, 0x28);
/// Warning amber: under five minutes of working time left.
pub(crate) const WARN_TEXT: egui::Color32 = egui::Color32::from_rgb(0xe6, 0x51, 0x00);

/// A label that wraps. Use this for anything longer than a few words.
pub(crate) fn wrapped_label(ui: &mut egui::Ui, text: impl Into<String>) {
    ui.add(egui::Label::new(text.into()).wrap());
}

/// A wrapping, coloured label.
pub(crate) fn wrapped_colored(ui: &mut egui::Ui, color: egui::Color32, text: impl Into<String>) {
    ui.add(egui::Label::new(egui::RichText::new(text.into()).color(color)).wrap());
}

/// A rounded, filled card — the panel-within-a-panel the summaries sit in.
pub(crate) fn card(ui: &mut egui::Ui, fill: egui::Color32, add: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(fill)
        .corner_radius(10.0)
        .inner_margin(12.0)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui);
        });
}
