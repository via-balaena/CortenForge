//! Solid cores the cup grows INTO the pour cavity.
//!
//! The cup wall is a shell that tracks the layer body, so wherever the body
//! carries a through-void the shell fills it and a solid core stands inside the
//! cavity. The wheel has done this since 2026-09-12: its bore column is
//! subtracted from the cast body, and the mold grows a locating pin through it
//! for free. Spokes added more.
//!
//! ⚠ **This exists because the bench sheet condemned them.** The cf-view
//! checklist ends with "do NOT proceed to print", and two of its bullets
//! declare raised material a regression — so every wheel cast told the workshop
//! that its own locating pin was a defect. A core is correct geometry, and the
//! sheet has to be able to say so.

/// Whether the cup grows solid cores into the pour cavity.
///
/// Follows the crate's `FlangeKind` / `GasketKind` / `SpokeKind` shape, and
/// like them it is read off the [`crate::Ribbon`] rather than passed in as a
/// caller's boolean — `procedure::seam_face_features` builds ONE list that
/// every seam-face bullet filters, precisely so two bullets cannot disagree
/// about what is there.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum CupCoreKind {
    /// The cup is a plain shell around the body; the cavity is empty.
    #[default]
    None,
    /// The cup fills through-voids in the body. The payload is what the
    /// bencher should expect to SEE, in their words.
    ///
    /// ⚠ **Generate this from the geometry that creates the cores — never type
    /// it.** A hand-written "six spoke cores" outlives the day someone changes
    /// the spoke count, and it is read inside a checklist whose failure
    /// instruction is "do NOT proceed to print".
    Present(String),
}

impl CupCoreKind {
    /// The bencher-facing description for [`Self::Present`], `None` otherwise.
    #[must_use]
    pub fn description(&self) -> Option<&str> {
        match self {
            Self::None => None,
            Self::Present(text) => Some(text),
        }
    }
}
