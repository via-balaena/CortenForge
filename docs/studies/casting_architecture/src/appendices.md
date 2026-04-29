# References and glossary

What this part covers (when authored to depth):

## References

To be populated. Candidate categories mirroring the soft-body book's appendices structure:

- **Mold design papers / textbooks.** Boothroyd-Dewhurst design-for-manufacturing classics. Solidworks Mold Tools / NX Mold Wizard documentation as practitioner references.
- **Parting-surface generation papers.** Silhouette extraction, convex decomposition, free-form parting. Active research; CG / CAD venues.
- **Casting / molding process literature.** Smooth-On technical bulletins, silicone manufacturer cure-chemistry references, vacuum-degassing best practices.
- **Demoldability analysis papers.** Geometric algorithms for undercut detection, demolding-direction analysis, draft-angle synthesis.
- **Manufacturing-aware optimization papers.** Topology optimization with manufacturability constraints (additive and subtractive), differentiable manufacturing models.

## Material database

A cast-side companion to the soft-body book's [material data appendix](../../soft_body_architecture/src/10-physical/04-material-data.md). Same Ecoflex / Dragon Skin / polyurethane families, but with the cast-side properties:

- Mix ratio (parts A : parts B by mass).
- Pot life / working time at 20 °C.
- Demold time at 20 °C, with oven post-cure deltas.
- Shrinkage (linear, % of mold dimension).
- Viscosity (cP, freshly mixed, before cure onset).
- Inhibition susceptibility (which contaminants kill cure).
- Adhesion compatibility matrix (which materials bond to which without primer).

The database structure ideally lives co-located with the soft-body material database — same datasheet, two consuming domains.

## Glossary

To be populated. Candidate terms:

- **Cure** — the chemical reaction transforming liquid prepolymer into solid elastomer.
- **Demold** — separating the cured cast from the mold.
- **Draft** — the angle between mold-side surfaces and the demolding direction; required to be positive for clean release.
- **Flash** — thin material that escapes the mold cavity at the parting line; trimmed off post-demold.
- **Inhibition** — a contaminant preventing cure; a common silicone failure mode.
- **Mold-in-mold** — using a cured cast as a feature of the next shot's mold cavity.
- **Pot life** — the working time between mixing and the prepolymer becoming too viscous to pour cleanly.
- **Riser** — a reservoir of extra cast material that feeds shrinkage during cure.
- **Shot** — one pour-cure cycle; multi-shot parts have multiple, in sequence.
- **Undercut** — a surface region whose normal opposes the demolding direction.
- **Vent** — a small channel in the mold that lets displaced air escape during pour.

## Notation

Aligned with the soft-body book's notation conventions where applicable. Casting-specific notation introduced as needed during the depth pass.
