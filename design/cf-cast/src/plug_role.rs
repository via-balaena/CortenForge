//! What the printed plug becomes once the pour cures.
//!
//! Every cast in this crate before 2026-09-13 retrieved its plug: it is
//! tooling, pulled out of the cured layer and reused. An **overmold** is the
//! same cast with the opposite answer — the pour is cast *onto* the plug and
//! the plug stays inside the finished part.
//!
//! The geometry is identical either way, so this changes the procedure sheet
//! and nothing else. Two of the sentences it moves are the ones that would
//! destroy an overmolded part — where mold release goes, and what demold does
//! with the plug; the rest describe what you are left holding.

/// What the printed plug becomes once the pour cures.
///
/// Only `plug_layer_0` can ever be an [`Insert`](Self::Insert) — the plugs
/// above it are printed positives of the layer below and come out under every
/// role.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlugRole {
    /// **Tooling** (default, and every cast this crate shipped before the
    /// wheel). The plug is pulled out of the cured layer and reused, so it
    /// gets mold release like the cup halves do.
    #[default]
    Tooling,
    /// **Insert** — an overmold. The plug is part of the finished product and
    /// stays inside the cured layer. It gets no mold release: release is what
    /// lets a cast let go, and this one is not meant to.
    Insert,
}
