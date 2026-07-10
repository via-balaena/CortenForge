//! The right-side egui panel — flexion replay controls, per-tissue visibility
//! toggles, and co-registration warnings — plus the system that applies those
//! toggles to the tissue entities' visibility.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::overlays::Overlays;
use crate::render::TissuePart;
use crate::replay::Flexion;

/// Per-tissue visibility flags, driven by the egui panel.
#[derive(Resource)]
pub(crate) struct SceneToggles {
    pub(crate) l4: bool,
    pub(crate) l5: bool,
    pub(crate) disc: bool,
    pub(crate) ligaments: bool,
    pub(crate) facets: bool,
}

impl Default for SceneToggles {
    fn default() -> Self {
        Self {
            l4: true,
            l5: true,
            disc: true,
            ligaments: true,
            facets: true,
        }
    }
}

/// Toggle each tissue mesh's visibility from the panel flags.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn apply_visibility(
    toggles: Res<SceneToggles>,
    mut query: Query<(&TissuePart, &mut Visibility)>,
) {
    for (part, mut vis) in &mut query {
        let show = match part {
            TissuePart::L4 => toggles.l4,
            TissuePart::L5 => toggles.l5,
            TissuePart::Disc => toggles.disc,
        };
        *vis = if show {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Right-side panel: flexion replay controls + per-tissue visibility toggles +
/// any co-registration warnings (so a misaligned trio is never shown as valid).
pub(crate) fn scene_panel(
    mut contexts: EguiContexts,
    mut toggles: ResMut<SceneToggles>,
    mut flexion: ResMut<Flexion>,
    overlays: Res<Overlays>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("fsu-panel")
        .resizable(false)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.heading("L4–L5 FSU");
            ui.label("coupled contact sim (moment-driven)");
            ui.separator();

            // ── Coupled replay ──
            ui.label("Moment-driven replay:");
            let n = flexion.traj.frames.len();
            let max = (n.saturating_sub(1)) as f32;
            let play_label = if flexion.playing {
                "⏸ Pause"
            } else {
                "▶ Play"
            };
            if ui.button(play_label).clicked() {
                flexion.playing = !flexion.playing;
            }
            // Scrubbing the timeline pauses auto-play.
            let mut cursor = flexion.cursor;
            if ui
                .add(egui::Slider::new(&mut cursor, 0.0..=max).text("frame"))
                .changed()
            {
                flexion.cursor = cursor;
                flexion.playing = false;
            }
            // Force-driven readouts: applied moment → solved equilibrium angle, and the
            // facet engagement (the bones stopping in extension).
            let angle_deg = flexion.true_theta.to_degrees();
            let phase = if angle_deg >= 0.0 {
                "flexion"
            } else {
                "extension"
            };
            ui.label(format!("applied moment: {:+.2} N·m", flexion.applied));
            ui.label(format!("equilibrium angle: {angle_deg:+.2}° ({phase})"));
            if flexion.n_facet > 0 {
                ui.colored_label(
                    egui::Color32::from_rgb(230, 80, 60),
                    format!(
                        "● facets ENGAGED — {} contacts (bones stop)",
                        flexion.n_facet
                    ),
                );
            } else {
                ui.label("○ facets open (no contact)");
            }
            ui.separator();

            ui.label("Show:");
            ui.checkbox(&mut toggles.l4, "L4 vertebra");
            ui.checkbox(&mut toggles.l5, "L5 vertebra");
            ui.checkbox(&mut toggles.disc, "Intervertebral disc (FEM)");
            ui.checkbox(&mut toggles.ligaments, "Ligaments");
            ui.checkbox(&mut toggles.facets, "Facet contacts (extension)");
            ui.separator();

            // Derive the ligament readout from what was actually built — a
            // degenerate mesh can drop one, and the panel must not claim it.
            let names: Vec<&str> = overlays.ligaments.iter().map(|l| l.name).collect();
            ui.label(format!(
                "{} ligaments ({})",
                overlays.ligaments.len(),
                names.join(", ")
            ));
            ui.label("red spheres = engaged facet contacts (extension)");
            ui.label("LMB orbit · RMB pan · scroll zoom · Esc quit");
            if !overlays.warnings.is_empty() {
                ui.separator();
                for w in &overlays.warnings {
                    ui.colored_label(egui::Color32::from_rgb(230, 140, 60), format!("⚠ {w}"));
                }
            }
        });
    Ok(())
}
