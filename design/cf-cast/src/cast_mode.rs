//! The casting model: how the per-layer silicone shells are produced.
//! See `docs/CF_CAST_BONDED_INPLACE_TEXTURE_RECON.md`.
//!
//! Both modes mesh the same per-layer **cup** halves; they differ only in the
//! **plugs**. The mode maps to a [`PartSelection`] over the existing
//! [`CastSpec::export_selected`](crate::CastSpec::export_selected) path, so
//! `Detachable` is exactly today's full cast and `Bonded` simply drops the
//! redundant plugs — no separate meshing path.

use crate::part_selection::{PartId, PartSelection};

/// How a multi-layer cast is assembled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CastMode {
    /// **Detachable** (SDK default). Every layer gets its own plug
    /// (`plug_layer_N`); each layer is cast independently and the cured shells
    /// **nest** + come apart for cleaning/replacement. This is the historical
    /// `export_molds_v2` output (`2L` cups + `L` plugs).
    #[default]
    Detachable,
    /// **Bonded cast-in-place** (Cendrillon default). Only **one** plug
    /// (`plug_layer_0`) is needed: the cured layer N *is* the plug for layer
    /// N+1. Each layer is poured onto the previous cured one so the silicone
    /// bonds as it cures — one growing, bonded part. Output: `2L` cups + 1 plug.
    ///
    /// The per-layer plugs above layer 0 are redundant (`plug_layer_{N>0}` is
    /// just `layers[N-1].body`, the outer geometry of the previous shell), so
    /// bonded mode drops them.
    Bonded,
}

impl CastMode {
    /// The [`PartSelection`] this mode implies for a `layer_count`-layer cast.
    ///
    /// `Detachable` selects everything (→ the validated full export);
    /// `Bonded` selects everything **except** the plugs above layer 0.
    ///
    /// `layer_count <= 1` makes the two modes identical (one layer ⇒ one plug
    /// either way), and the bonded exclusion is empty.
    #[must_use]
    pub fn part_selection(self, layer_count: usize) -> PartSelection {
        match self {
            Self::Detachable => PartSelection::all(),
            Self::Bonded => {
                PartSelection::all_except((1..layer_count).map(|i| PartId::Plug { layer_index: i }))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ribbon::PieceSide;

    #[test]
    fn detachable_selects_everything() {
        let sel = CastMode::Detachable.part_selection(3);
        assert!(sel.is_all(), "detachable → the full-cast path");
        assert!(sel.includes(PartId::Plug { layer_index: 2 }));
    }

    #[test]
    fn bonded_keeps_only_plug_zero() {
        let sel = CastMode::Bonded.part_selection(3);
        assert!(!sel.is_all());
        assert!(sel.includes(PartId::Plug { layer_index: 0 }), "plug 0 kept");
        assert!(
            !sel.includes(PartId::Plug { layer_index: 1 }),
            "plug 1 dropped"
        );
        assert!(
            !sel.includes(PartId::Plug { layer_index: 2 }),
            "plug 2 dropped"
        );
        // Cups + accessories untouched.
        assert!(sel.includes(PartId::Cup {
            layer_index: 2,
            side: PieceSide::Positive
        }));
        assert!(sel.includes(PartId::Platform));
    }

    #[test]
    fn default_is_detachable() {
        assert_eq!(CastMode::default(), CastMode::Detachable);
    }

    #[test]
    fn single_layer_modes_coincide() {
        // One layer ⇒ one plug either way; the bonded exclusion is empty so it
        // selects everything (though not via the `All` variant).
        let bonded = CastMode::Bonded.part_selection(1);
        assert!(bonded.includes(PartId::Plug { layer_index: 0 }));
    }
}
