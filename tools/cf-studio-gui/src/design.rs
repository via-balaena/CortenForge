//! Step 4's field state: the silicone layers built outward off the shaped
//! piece.

use bevy::prelude::*;
use cf_studio_core::LayerDraft;
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
/// screen left you on step 4 with Next → newly enabled, beside two buttons you
/// might reasonably reach for next.
pub(crate) fn commit_design(layers: Vec<LayerDraft>, studio: &mut Studio) {
    studio.message = Some(apply_design_draft(&mut studio.project, layers));
}
