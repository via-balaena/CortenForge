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

/// Which parts a selected export should mesh + write.
///
/// [`PartSelection::all`] is the everything-set (reproduces a full cast);
/// otherwise only the listed [`PartId`]s are generated. Consumed by
/// [`CastSpec::export_selected`](crate::CastSpec::export_selected).
#[derive(Debug, Clone)]
pub struct PartSelection {
    /// `None` ⇒ select everything; `Some(list)` ⇒ select exactly `list`.
    selected: Option<Vec<PartId>>,
}

impl PartSelection {
    /// Select every part — the selected-export path then reproduces a
    /// full [`export_molds_v2`](crate::CastSpec::export_molds_v2).
    #[must_use]
    pub const fn all() -> Self {
        Self { selected: None }
    }

    /// Select exactly the listed parts (deduplication is unnecessary —
    /// [`Self::includes`] is a membership test, so repeats are harmless).
    #[must_use]
    pub fn from_ids(ids: impl IntoIterator<Item = PartId>) -> Self {
        Self {
            selected: Some(ids.into_iter().collect()),
        }
    }

    /// `true` when this is the everything-set (`all()`).
    #[must_use]
    pub const fn is_all(&self) -> bool {
        self.selected.is_none()
    }

    /// Whether `id` should be generated under this selection.
    #[must_use]
    pub fn includes(&self, id: PartId) -> bool {
        self.selected.as_ref().is_none_or(|list| list.contains(&id))
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
}
