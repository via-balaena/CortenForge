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
use cf_studio_gui::StepBoxState;

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
/// The ridge editor's card — warmer than the result cards, as the pre-port
/// screen had it.
pub(crate) const RIDGE_FILL: egui::Color32 = egui::Color32::from_rgb(0xf4, 0xf1, 0xea);
/// The notes on [`RIDGE_FILL`]: what ridges cost, and what is not on screen yet.
pub(crate) const RIDGE_NOTE_TEXT: egui::Color32 = egui::Color32::from_rgb(0x6b, 0x5f, 0x44);

/// A label that wraps. Use this for anything longer than a few words.
pub(crate) fn wrapped_label(ui: &mut egui::Ui, text: impl Into<String>) {
    ui.add(egui::Label::new(text.into()).wrap());
}

/// A wrapping, coloured label.
pub(crate) fn wrapped_colored(ui: &mut egui::Ui, color: egui::Color32, text: impl Into<String>) {
    ui.add(egui::Label::new(egui::RichText::new(text.into()).color(color)).wrap());
}

/// The green a finished cleanup section's heading turns.
///
/// ⚠ Not [`GOOD_TEXT`]. The pre-port screen used a brighter green here than in
/// its result cards, because this one carries no fill behind it.
pub(crate) const DONE_TEXT: egui::Color32 = egui::Color32::from_rgb(0x2e, 0x7d, 0x32);
/// A cleanup section heading before it is done.
pub(crate) const HEADING_TEXT: egui::Color32 = egui::Color32::from_rgb(0x1a, 0x1a, 0x1a);
/// The plain-language hint under a section heading.
pub(crate) const HINT_TEXT: egui::Color32 = egui::Color32::from_rgb(0x5f, 0x5f, 0x5f);
/// The working-mesh stats line above the cleanup controls.
pub(crate) const STATS_TEXT: egui::Color32 = egui::Color32::from_rgb(0x55, 0x55, 0x55);
/// The small labels between controls — "from tip", "mm above cut".
pub(crate) const CONTROL_TEXT: egui::Color32 = egui::Color32::from_rgb(0x33, 0x33, 0x33);

/// A section heading's point size.
const HEADING_SIZE: f32 = 15.0;
/// A hint's point size.
const HINT_SIZE: f32 = 13.0;
/// The stepper's text field.
const FIELD_WIDTH: f32 = 56.0;
/// Between a heading, its hint, and its controls.
const SECTION_SPACING: f32 = 4.0;
/// Between a field grid's columns and rows.
const GRID_SPACING: f32 = 6.0;

/// Centred text, wrapping to the available width.
///
/// ⚠ **Both halves are needed, and each is silently wrong alone.**
///
/// - The layout job's `halign` centres the *lines within the galley*, but
///   `Label` allocates a rect of `galley.size()` — the text's natural width, not
///   the wrap width — and anchors the galley at that rect's centre. In a
///   left-aligned top-down `Ui` the rect sits at the left edge, so short text
///   centres inside its own narrow box and still renders hard left. Only a
///   paragraph long enough to wrap to full width looks centred by accident.
/// - `vertical_centered` centres that rect, but leaves every line inside it
///   left-aligned, so a wrapped paragraph comes out ragged-right in the middle
///   of the panel.
///
/// Together the rect is centred and so are the lines in it.
pub(crate) fn centered_wrapped(
    ui: &mut egui::Ui,
    size: f32,
    color: egui::Color32,
    text: impl Into<String>,
) {
    let mut job = egui::text::LayoutJob::simple(
        text.into(),
        egui::FontId::proportional(size),
        color,
        ui.available_width(),
    );
    job.halign = egui::Align::Center;
    ui.vertical_centered(|ui| ui.add(egui::Label::new(job)));
}

/// One numbered sub-step of step 2's cleanup flow: a sequence heading
/// ("First — …", "Then — …"), a one-line hint, then its controls.
///
/// Laid out top to bottom so the page reads as an obvious order rather than a
/// flat toolbar — the reason the pre-port screen had this as a component at all.
/// `done` appends a tick and turns the heading green.
pub(crate) fn cleanup_section(
    ui: &mut egui::Ui,
    heading: &str,
    hint: &str,
    done: bool,
    add: impl FnOnce(&mut egui::Ui),
) {
    let (text, color) = if done {
        (format!("{heading}   ✔"), DONE_TEXT)
    } else {
        (heading.to_string(), HEADING_TEXT)
    };
    // ⚠ Not `RichText::strong()`. egui resolves `strong` to a *colour*, and only
    // when no explicit colour is set — so beside `.color()` it is dead, and it
    // was never going to give the pre-port `font-weight: 700` either, because
    // the bundled proportional family has no bold face. Size and colour are the
    // weight this heading gets.
    centered_wrapped(ui, HEADING_SIZE, color, text);
    ui.add_space(SECTION_SPACING);
    if !hint.is_empty() {
        centered_wrapped(ui, HINT_SIZE, HINT_TEXT, hint);
        ui.add_space(SECTION_SPACING);
    }
    add(ui);
}

/// A numeric stepper: `−`, a text field, `+`.
///
/// ⚠ Deliberately not a `DragValue` or a spinner. Both step on the mouse wheel,
/// which fights the scrolling panel this sits in — the pre-port widget was hand-
/// built to avoid exactly that. Typing does not commit; Enter, the buttons and
/// focus-out do. [`StepBoxState`] holds all four of those rules.
pub(crate) fn step_box(
    ui: &mut egui::Ui,
    state: &mut StepBoxState,
    (min, max): (i32, i32),
    step: i32,
    enabled: bool,
) {
    // ⚠ One `horizontal` group, not three loose widgets, or a wrapping parent is
    // free to break BETWEEN them and orphan the `+` under the field.
    ui.horizontal(|ui| {
        // The committed value is read when an action button is clicked, not here:
        // nothing in step 2 re-meshes on a field edit, so the commit results go
        // unused on purpose.
        if ui
            .add_enabled(enabled && state.value() > min, egui::Button::new("−"))
            .clicked()
        {
            let _ = state.step(-step, min, max);
        }
        let field = ui.add_enabled(
            enabled,
            egui::TextEdit::singleline(state.text_mut())
                .desired_width(FIELD_WIDTH)
                .horizontal_align(egui::Align::Center),
        );
        if field.changed() {
            state.on_typed();
        }
        // Enter also drops focus on a single-line field, so this is both commit
        // paths at once.
        if field.lost_focus() {
            let _ = state.commit(min, max);
        }
        if ui
            .add_enabled(enabled && state.value() < max, egui::Button::new("+"))
            .clicked()
        {
            let _ = state.step(step, min, max);
        }
    });
}

/// A label / field / unit grid for a stacked control block.
///
/// ⚠ A `Grid`, not a stack of `ui.horizontal` rows: stacked fields have to line
/// up with each other, and the labels beside them differ in width.
///
/// `columns` is how many cells a full row has. egui sizes the last column from
/// it, so a width-filling widget there gets the rest of the row.
pub(crate) fn field_grid(
    ui: &mut egui::Ui,
    id: &str,
    columns: usize,
    add: impl FnOnce(&mut egui::Ui),
) {
    egui::Grid::new(id)
        .num_columns(columns)
        .spacing([GRID_SPACING, GRID_SPACING])
        .show(ui, add);
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
