//! Step 4's field state: the silicone layers built outward off the shaped
//! piece.

use bevy::prelude::*;
use cf_studio_core::{DesignDraft, LayerDraft};
use cf_studio_gui::{LayerStack, apply_design_draft};

use crate::state::Studio;

/// The step-4 screen's field state, which outlives any one frame.
#[derive(Resource, Default)]
pub(crate) struct DesignControls {
    /// The silicone layers, innermost first.
    pub(crate) layers: LayerStack,
}

/// Commit the layer stack as the project's design.
///
/// ⚠ Does not advance, where [`crate::shape::commit_plug`] does — the pre-port
/// screen left you on step 4 with Next → newly enabled. "Use this design" sits
/// between two buttons ("+ Add layer", "…or load a file") that are reasonable
/// things to reach for after seeing the design took, and paging away from them
/// would be the surprise.
pub(crate) fn commit_design(layers: Vec<LayerDraft>, studio: &mut Studio) {
    // The cavity inset belongs to "Shape your piece" — the layer stack builds
    // outward off the plug that step shaped, and carries a copy so the design
    // is self-contained for the cast engine.
    let cavity_inset_m = studio
        .project
        .plug()
        .map_or(0.0, |plug| plug.cavity_inset_m);
    let outcome = apply_design_draft(
        &mut studio.project,
        DesignDraft {
            cavity_inset_m,
            layers,
        },
    );
    studio.message = Some(outcome);
}
