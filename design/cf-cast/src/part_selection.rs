//! Per-piece selection for [`CastSpec::export_selected`](crate::CastSpec::export_selected).
//!
//! A full cast meshes every cup half + plug for every layer plus the
//! shared accessories — minutes of marching cubes. When a workshop user
//! only needs to re-print one piece (a single layer-0 plug, say), a
//! [`PartSelection`] tells the export to mesh + write *only* the chosen
//! [`PartId`]s and skip the rest.
//!
//! [`PartSelection::all`] is the identity: it selects every part, so the
//! selected-export path reproduces the full cast. The unselected pieces
//! are simply never meshed (the time saving), and the pour plan is still
//! computed for every layer (cheap volume integration, no marching cubes)
//! so downstream pour instructions stay complete.

use crate::ribbon::PieceSide;

/// A single addressable cast output. Mirrors the emitted-artifact set
/// one-to-one (and the internal `CastTarget`), so a selection maps
/// directly onto the per-piece meshing the export does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartId {
    /// One of a layer's two mold cup halves.
    Cup {
        /// Index into `CastSpec::layers` (innermost-first).
        layer_index: usize,
        /// Which ribbon side this half occupies.
        side: PieceSide,
    },
    /// A layer's inner-cavity plug (`plug_layer_{layer_index}.stl`).
    Plug {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
    },
    /// The shared workshop support platform (`platform.stl`).
    Platform,
    /// The shared workshop pour funnel (`funnel.stl`).
    Funnel,
    /// A layer's gasket mold (`gasket_mold_layer_{layer_index}.stl`).
    GasketMold {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
    },
    /// The shared printable dowel array (`dowel.stl`).
    Dowel,
}

/// Internal selection mode.
#[derive(Debug, Clone)]
enum Sel {
    /// Everything (reproduces a full cast).
    All,
    /// Exactly the listed parts.
    Only(Vec<PartId>),
    /// Everything except the listed parts (e.g. bonded mode drops the
    /// redundant per-layer plugs).
    AllExcept(Vec<PartId>),
}

/// Which parts a selected export should mesh + write.
///
/// [`PartSelection::all`] is the everything-set (reproduces a full cast);
/// [`PartSelection::from_ids`] selects exactly a list; [`PartSelection::all_except`]
/// selects everything but a list. Consumed by
/// [`CastSpec::export_selected`](crate::CastSpec::export_selected).
#[derive(Debug, Clone)]
pub struct PartSelection {
    sel: Sel,
}

impl PartSelection {
    /// Select every part — the selected-export path then reproduces a
    /// full [`export_molds_v2`](crate::CastSpec::export_molds_v2).
    #[must_use]
    pub const fn all() -> Self {
        Self { sel: Sel::All }
    }

    /// Select exactly the listed parts (deduplication is unnecessary —
    /// [`Self::includes`] is a membership test, so repeats are harmless).
    #[must_use]
    pub fn from_ids(ids: impl IntoIterator<Item = PartId>) -> Self {
        Self {
            sel: Sel::Only(ids.into_iter().collect()),
        }
    }

    /// Select **everything except** the listed parts. Used by bonded casting
    /// ([`CastMode::Bonded`](crate::CastMode)) to drop every plug above
    /// layer 0 without enumerating the rest of the cast.
    #[must_use]
    pub fn all_except(ids: impl IntoIterator<Item = PartId>) -> Self {
        Self {
            sel: Sel::AllExcept(ids.into_iter().collect()),
        }
    }

    /// `true` when this is the everything-set (`all()`) — the only selection
    /// that routes to the validated full `export_molds_v2`.
    #[must_use]
    pub const fn is_all(&self) -> bool {
        matches!(self.sel, Sel::All)
    }

    /// Whether `id` should be generated under this selection.
    #[must_use]
    pub fn includes(&self, id: PartId) -> bool {
        match &self.sel {
            Sel::All => true,
            Sel::Only(list) => list.contains(&id),
            Sel::AllExcept(list) => !list.contains(&id),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_includes_everything() {
        let sel = PartSelection::all();
        assert!(sel.is_all());
        assert!(sel.includes(PartId::Plug { layer_index: 0 }));
        assert!(sel.includes(PartId::Cup {
            layer_index: 3,
            side: PieceSide::Positive
        }));
        assert!(sel.includes(PartId::Dowel));
    }

    #[test]
    fn from_ids_selects_only_listed() {
        let sel = PartSelection::from_ids([PartId::Plug { layer_index: 0 }]);
        assert!(!sel.is_all());
        assert!(sel.includes(PartId::Plug { layer_index: 0 }));
        assert!(!sel.includes(PartId::Plug { layer_index: 1 }));
        assert!(!sel.includes(PartId::Cup {
            layer_index: 0,
            side: PieceSide::Negative
        }));
        assert!(!sel.includes(PartId::Platform));
    }

    #[test]
    fn cup_sides_are_distinct() {
        let sel = PartSelection::from_ids([PartId::Cup {
            layer_index: 0,
            side: PieceSide::Negative,
        }]);
        assert!(sel.includes(PartId::Cup {
            layer_index: 0,
            side: PieceSide::Negative
        }));
        assert!(!sel.includes(PartId::Cup {
            layer_index: 0,
            side: PieceSide::Positive
        }));
    }

    #[test]
    fn all_except_includes_everything_but_the_listed() {
        // Bonded shape: drop plugs 1 + 2, keep everything else.
        let sel = PartSelection::all_except([
            PartId::Plug { layer_index: 1 },
            PartId::Plug { layer_index: 2 },
        ]);
        assert!(!sel.is_all(), "an exclusion is not the everything-set");
        assert!(sel.includes(PartId::Plug { layer_index: 0 }), "plug 0 kept");
        assert!(
            !sel.includes(PartId::Plug { layer_index: 1 }),
            "plug 1 dropped"
        );
        assert!(
            !sel.includes(PartId::Plug { layer_index: 2 }),
            "plug 2 dropped"
        );
        assert!(sel.includes(PartId::Cup {
            layer_index: 1,
            side: PieceSide::Negative
        }));
        assert!(sel.includes(PartId::Platform));
        assert!(sel.includes(PartId::Dowel));
    }
}
