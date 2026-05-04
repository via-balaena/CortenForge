//! egui-driven dropdown panel for scalar + colormap selection.
//!
//! UI library: `bevy_egui = "0.39.1"` (depends on `bevy ^0.18.0`; the
//! workspace has no Bevy UI primitive precedent — see
//! `docs/VIEWER_DESIGN.md` iter-2 still-open #3 lock).
//!
//! ## State
//!
//! - [`Selection`] — `Resource` carrying the active `scalar_index` (into
//!   `ViewerInput::scalar_names`) and a [`ColormapOverride`]. Mutated by
//!   [`scalar_and_colormap_panel`]; read by the geometry-spawn system in
//!   `main.rs`.
//! - [`GeometryEntity`] — `Component` marker on every entity spawned by
//!   the geometry-spawn system. The despawn step in that system queries
//!   for this marker so re-spawn is bounded to the geometry.
//!
//! ## Re-render mechanism (per `docs/VIEWER_DESIGN.md` head-architect call B)
//!
//! When [`Selection`] mutates, the geometry-spawn system in `main.rs`
//! despawns all [`GeometryEntity`] entities and rebuilds them with the
//! new selection. This is the simple-and-brute-force path (`feedback_baby_steps`);
//! the alternative — mutating `StandardMaterial::base_color` in-place —
//! becomes worth doing only if a future consumer pushes past ~10K verts.
//!
//! ## CLI override
//!
//! `--scalar=<name>` / `--colormap=<kind>` (commit 6) seed [`Selection`]
//! before the UI sees it; the dropdowns then act as runtime overrides.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::ViewerInput;

/// Marker for entities owned by the geometry-spawn system.
///
/// Despawned in bulk when [`Selection`] changes so the spawn step can
/// rebuild from scratch with the new scalar / colormap choice.
#[derive(Component)]
pub struct GeometryEntity;

/// CLI-or-UI override for the auto-detected colormap kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ColormapOverride {
    /// Use [`crate::colormap::detect`] on the data — Q5 default.
    #[default]
    Auto,
    /// Force divergent (`coolwarm`-style bwr).
    Divergent,
    /// Force sequential (`viridis`).
    Sequential,
    /// Force categorical (`tab10`, indexed by `value mod 10`).
    Categorical,
}

impl ColormapOverride {
    /// All variants in dropdown order.
    pub const ALL: [Self; 4] = [
        Self::Auto,
        Self::Divergent,
        Self::Sequential,
        Self::Categorical,
    ];

    /// Human-readable label for the dropdown row.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Divergent => "divergent",
            Self::Sequential => "sequential",
            Self::Categorical => "categorical",
        }
    }
}

/// Active selection — which scalar to color by and which colormap.
#[derive(Resource, Debug, Default, Clone, PartialEq, Eq)]
pub struct Selection {
    /// Index into `ViewerInput::scalar_names`. `0` is the Q4 alphabetical
    /// first-pick. For empty `scalar_names` the index is unused; the
    /// geometry-spawn system gates on `scalar_names.is_empty()`.
    pub scalar_index: usize,
    /// User override for the colormap kind. `Auto` runs Q5 detection.
    pub colormap_override: ColormapOverride,
}

impl Selection {
    /// Clamp `scalar_index` against `scalar_count`, never producing an
    /// out-of-range index. For empty inputs it returns `0` (the geometry
    /// system independently checks emptiness before indexing).
    #[must_use]
    pub fn clamped_scalar_index(&self, scalar_count: usize) -> usize {
        if scalar_count == 0 {
            0
        } else {
            self.scalar_index.min(scalar_count - 1)
        }
    }
}

/// egui side panel: scalar dropdown + colormap dropdown. Runs in
/// [`bevy_egui::EguiPrimaryContextPass`] (the schedule egui lays out
/// against the primary window).
///
/// Mutates [`Selection`] only when a different choice is clicked, so
/// `Selection::is_changed()` stays clean unless the user actually moved.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn scalar_and_colormap_panel(
    mut contexts: EguiContexts,
    input: Res<ViewerInput>,
    mut selection: ResMut<Selection>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::left("cf-view-controls")
        .resizable(false)
        .default_width(220.0)
        .show(ctx, |ui| {
            ui.heading("cf-view");
            ui.separator();

            ui.label("Scalar");
            if input.scalar_names.is_empty() {
                ui.label(egui::RichText::new("(none — geometry only)").italics());
            } else {
                let current = selection.clamped_scalar_index(input.scalar_names.len());
                let current_name = input.scalar_names[current].as_str();
                egui::ComboBox::from_id_salt("scalar")
                    .selected_text(current_name)
                    .show_ui(ui, |ui| {
                        for (i, name) in input.scalar_names.iter().enumerate() {
                            if ui.selectable_label(i == current, name).clicked()
                                && selection.scalar_index != i
                            {
                                selection.scalar_index = i;
                            }
                        }
                    });
            }

            ui.add_space(8.0);
            ui.label("Colormap");
            let current = selection.colormap_override;
            egui::ComboBox::from_id_salt("colormap")
                .selected_text(current.label())
                .show_ui(ui, |ui| {
                    for kind in ColormapOverride::ALL {
                        if ui.selectable_label(kind == current, kind.label()).clicked()
                            && selection.colormap_override != kind
                        {
                            selection.colormap_override = kind;
                        }
                    }
                });

            ui.add_space(8.0);
            ui.separator();
            ui.label(
                egui::RichText::new("LMB orbit · RMB pan · scroll zoom · Esc exit")
                    .small()
                    .italics(),
            );
        });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selection_default_is_index_zero_and_auto() {
        let s = Selection::default();
        assert_eq!(s.scalar_index, 0);
        assert_eq!(s.colormap_override, ColormapOverride::Auto);
    }

    #[test]
    fn clamped_scalar_index_caps_at_last_valid() {
        let s = Selection {
            scalar_index: 99,
            ..Selection::default()
        };
        assert_eq!(s.clamped_scalar_index(3), 2);
    }

    #[test]
    fn clamped_scalar_index_returns_zero_for_empty() {
        let s = Selection {
            scalar_index: 99,
            ..Selection::default()
        };
        assert_eq!(s.clamped_scalar_index(0), 0);
    }

    #[test]
    fn clamped_scalar_index_passes_through_when_in_range() {
        let s = Selection {
            scalar_index: 1,
            ..Selection::default()
        };
        assert_eq!(s.clamped_scalar_index(3), 1);
    }

    #[test]
    fn colormap_override_all_lists_every_variant_once() {
        let labels: Vec<&str> = ColormapOverride::ALL.iter().map(|k| k.label()).collect();
        assert_eq!(
            labels,
            vec!["auto", "divergent", "sequential", "categorical"]
        );
    }

    #[test]
    fn colormap_override_default_is_auto() {
        assert_eq!(ColormapOverride::default(), ColormapOverride::Auto);
    }
}
