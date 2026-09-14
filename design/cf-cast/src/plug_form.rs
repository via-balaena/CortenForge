//! What the plug LOOKS LIKE to a bencher holding it.
//!
//! The cf-view checklist ends with "do NOT proceed to print", and its plug
//! block asserts a positive shape: *"Dome end is smooth and closed."* That is
//! true of the sock-over-capsule paradigm every cast came from, and false of
//! the first subject that was not one. A wheel's plug is the rim — a bored,
//! dimpled, sometimes slotted disc with no dome anywhere on it — so the sheet
//! condemned a correct rim on a line the bencher is told not to print past.
//!
//! ⚠ **This is not [`crate::CupCoreKind`], and must not be folded into it.**
//! `cup_cores` describes through-voids in the layer BODY. A cast can carry
//! those while its plug is a solid capsule, so reading cores as permission for
//! plug openings would excuse exactly the cast that should be flagged.
//! Cavity topology and plug shape are independent axes.

/// The plug's outward form, as the bench sheet should describe it.
///
/// Follows [`crate::CupCoreKind`]'s shape — a default that renders what every
/// sheet has always rendered, and a [`Self::Described`] arm carrying prose for
/// subjects the capsule vocabulary does not fit.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum PlugFormKind {
    /// A dome-ended capsule: flat cap-plane face at one end, smooth closed
    /// dome at the other. The shape the cast pipeline was built around.
    #[default]
    DomedCapsule,
    /// The plug is some other shape. The payload is what the bencher should
    /// expect to SEE, in their words — a noun phrase, article and all, that
    /// reads after "The plug is ".
    ///
    /// ⚠ **Generate this from the geometry that makes the shape — never type
    /// it.** A hand-written "six slots" outlives the day someone changes the
    /// spoke count, inside a checklist whose failure instruction is "do NOT
    /// proceed to print".
    Described(String),
}

impl PlugFormKind {
    /// The bencher-facing description for [`Self::Described`], `None` for a
    /// domed capsule.
    #[must_use]
    pub fn description(&self) -> Option<&str> {
        match self {
            Self::DomedCapsule => None,
            Self::Described(text) => Some(text),
        }
    }
}
