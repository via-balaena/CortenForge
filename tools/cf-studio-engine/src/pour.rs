//! Assembling the structured pour plan — the data the Step-6 pour
//! assistant renders (per layer: material, mass, mix ratio, pot-life
//! timer, cure wait). This is structured data, not the rendered
//! `procedure.md` markdown: the GUI needs the *numbers* to drive a
//! countdown timer and a check-off list.
//!
//! Inputs come from the cast run (the per-layer poured masses) zipped
//! with the design (anchor keys + Slacker). The mix ratio / pot life /
//! cure time come from `cf-cast`'s cure-protocol table, the friendly
//! display name from the silicone catalog.

use cf_cast::cure::lookup as cure_lookup;
use cf_cast_cli::display_name_for_anchor;
use cf_studio_core::{PourPlan, PourStep};

use crate::error::{EngineError, Result};

/// One layer's inputs to [`build_pour_plan`]: the silicone anchor key,
/// the poured mass (grams, from the cast run), and the Slacker fraction
/// (`None` when the layer uses no Slacker).
#[derive(Debug, Clone, PartialEq)]
pub struct LayerPour {
    /// Silicone catalog key, e.g. `"ECOFLEX_00_30"`.
    pub anchor_key: String,
    /// Poured mass for this layer, in grams.
    pub mass_g: f64,
    /// Slacker fraction, if any.
    pub slacker_fraction: Option<f64>,
}

/// Assemble the ordered [`PourPlan`] (innermost layer first) the Step-6
/// assistant renders. Each layer's mix ratio / pot life / cure time come
/// from `cf-cast`'s cure-protocol table; the display name from the catalog.
///
/// # Errors
/// [`EngineError::PourDataUnavailable`] if a layer's silicone has no
/// cure-protocol entry — the assistant can't guide a pour it has no
/// timing for, so we surface that rather than emit a plan with blanks.
pub fn build_pour_plan(layers: &[LayerPour]) -> Result<PourPlan> {
    let mut steps = Vec::with_capacity(layers.len());
    for (layer_index, layer) in layers.iter().enumerate() {
        let cure =
            cure_lookup(&layer.anchor_key).ok_or_else(|| EngineError::PourDataUnavailable {
                key: layer.anchor_key.clone(),
            })?;
        // Fall back to the raw key if the catalog has no friendly name
        // (shouldn't happen for catalog materials, but never blank).
        let material_display_name = display_name_for_anchor(&layer.anchor_key)
            .map(ToString::to_string)
            .unwrap_or_else(|| layer.anchor_key.clone());
        steps.push(PourStep {
            layer_index,
            material_display_name,
            mass_g: layer.mass_g,
            mix_ratio_a_to_b: cure.mix_ratio_a_to_b.to_string(),
            pot_life_minutes: cure.pot_life_minutes,
            cure_time_hours: cure.cure_time_hours,
            slacker_fraction: layer.slacker_fraction,
        });
    }
    Ok(PourPlan { steps })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use super::*;

    /// The base_mold 17.5 / 7.5 / 5 mm stack, with masses from a run.
    fn base_mold_layers() -> Vec<LayerPour> {
        vec![
            LayerPour {
                anchor_key: "ECOFLEX_00_30".to_string(),
                mass_g: 369.0,
                slacker_fraction: Some(0.25),
            },
            LayerPour {
                anchor_key: "DRAGON_SKIN_10A".to_string(),
                mass_g: 260.5,
                slacker_fraction: None,
            },
            LayerPour {
                anchor_key: "DRAGON_SKIN_20A".to_string(),
                mass_g: 213.4,
                slacker_fraction: None,
            },
        ]
    }

    #[test]
    fn builds_a_step_per_layer_in_order() {
        let plan = build_pour_plan(&base_mold_layers()).unwrap();
        assert_eq!(plan.steps.len(), 3);
        for (i, step) in plan.steps.iter().enumerate() {
            assert_eq!(step.layer_index, i, "steps must be in pour order");
            // Every layer must carry real, non-blank pour guidance.
            assert!(!step.material_display_name.is_empty());
            assert!(!step.mix_ratio_a_to_b.is_empty());
            assert!(step.pot_life_minutes > 0, "pot life must be known");
            assert!(step.cure_time_hours > 0.0, "cure time must be known");
        }
    }

    #[test]
    fn passes_through_mass_and_slacker() {
        let plan = build_pour_plan(&base_mold_layers()).unwrap();
        assert_eq!(plan.steps[0].mass_g, 369.0);
        assert_eq!(plan.steps[0].slacker_fraction, Some(0.25));
        assert_eq!(plan.steps[1].mass_g, 260.5);
        assert_eq!(plan.steps[1].slacker_fraction, None);
    }

    #[test]
    fn material_without_cure_data_is_rejected() {
        let layers = vec![LayerPour {
            anchor_key: "NOT_A_SILICONE".to_string(),
            mass_g: 100.0,
            slacker_fraction: None,
        }];
        let err = build_pour_plan(&layers).unwrap_err();
        assert!(
            matches!(&err, EngineError::PourDataUnavailable { key } if key == "NOT_A_SILICONE"),
            "got: {err:?}"
        );
    }

    #[test]
    fn empty_input_yields_empty_plan() {
        let plan = build_pour_plan(&[]).unwrap();
        assert!(plan.steps.is_empty());
    }
}
