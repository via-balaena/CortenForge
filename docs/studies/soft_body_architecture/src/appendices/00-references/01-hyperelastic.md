# Hyperelasticity papers

Hyperelastic constitutive laws are `sim-soft`'s default material model per the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md). This leaf indexes the foundational constitutive-law references plus the silicone curve-fit studies the [material database](../02-material-db.md) cites.

## Pass 1 note

Unlike the other four reference leaves, Parts 1 and 2's prose mentions hyperelasticity author names (Ogden, Mooney, Rivlin, Holzapfel, Gasser, Marechal, Liao) in running text without linking them to anchors. No Pass-1 chapter currently has an `appendices/00-references/01-hyperelastic.md#lastname-year` inline link that must resolve. This file is scaffolding for the Pass 3 pass that will promote those in-text mentions to anchor-linked citations and populate the entries.

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate:

- Mooney 1940 and Rivlin 1948 — the original Mooney-Rivlin formulation, cited in [Part 2 Ch 04](../../20-materials/04-hyperelastic.md).
- Ogden 1972 — the principal-stretch-based Ogden family, cited in the same chapter and in [Part 1 Ch 04](../../10-physical/04-material-data.md) for the silicone curve fits.
- Holzapfel, Gasser & Ogden 2000 — the HGO fiber-reinforced anisotropic hyperelastic law [Part 2 Ch 06](../../20-materials/06-anisotropic.md) commits to as the default directional-material law.
- Bonet & Wood (textbook) — the standard reference for nonlinear-continuum-mechanics derivations Part 2 cites implicitly when deriving first-Piola expressions and tangent-stiffness forms.
- Marechal et al. — an Ecoflex 00-30 hyperelastic curve-fit study referenced by Part 1 Ch 04 for the stiffest-end-of-the-range Ecoflex calibration.
- Liao et al. — an Ecoflex 00-50 hyperelastic curve-fit study referenced in the same chapter.
- A Dragon Skin curve-fit study — Part 1 Ch 04's Dragon Skin data trace will be anchored here once the Pass 3 source check settles which published fit the book relies on.

Each entry will carry a 1–2 line description in Pass 3. Anchoring them now is premature per the "extra entries are Pass 3 judgment" rule — the book does not yet reference them by anchor, so committing to first-author naming before the Pass 3 source fetch is a self-consistency risk.
