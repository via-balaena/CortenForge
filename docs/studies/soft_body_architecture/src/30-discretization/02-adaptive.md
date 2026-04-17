# Adaptive refinement

Adaptive refinement is the taxonomy — three ways to increase local mesh fidelity where the solver needs it: **h-refinement** (subdivide tets into smaller tets), **p-refinement** (upgrade local elements to higher shape-function order), and **contact-driven refinement** (refine based on contact proximity rather than stress feedback). This chapter names the taxonomy and points at the sub-chapters that cover each. The specific `sim-soft` commitment for h-refinement — stress-gradient-triggered red-green subdivision per Bey 1995 — was made in [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md) and is not repeated here.

| Section | What it covers |
|---|---|
| [Stress-driven h-refinement](02-adaptive/00-h-refinement.md) | Subdivide tets where stress gradient across neighbors exceeds a threshold; see [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md) for the `sim-soft` commitment and implementation |
| [p-refinement](02-adaptive/01-p-refinement.md) | Upgrade specific elements from Tet4 to Tet10 (one order higher shape functions). Complementary to h-refinement, not an alternative; Phase H's Tet10-in-contact-band pattern is a manual p-refinement. Fully adaptive p-refinement is a post-Phase-I concern (per Ch 00 Claim 4) |
| [Contact-driven refinement](02-adaptive/02-contact-driven.md) | Refine tets near active IPC contact pairs regardless of stress magnitude. Rejected as default (refines too aggressively on glancing contact, not enough on internal stress concentrations); could be available as a debug flag |

Two claims.

**h-refinement and p-refinement are complementary, not alternatives.** h-refinement addresses the *mesh-resolution* half of the [rim-deformation failure](../10-physical/02-what-goes-wrong/04-rim.md); p-refinement addresses the *shape-function-order* half. Both are useful, on different problems. The canonical problem benefits from h-refinement in bulk stress-concentration zones and p-refinement (via Tet10) in the contact band — together, not either-or. Phase H ships both; [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md) specifies the h-refinement path, [Ch 00 Claim 2 of this Part](00-element-choice.md) specifies the p-refinement path as Tet10-in-band.

**Contact-driven refinement is rejected as the primary trigger.** [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md) lays out the reasoning: refining based on contact-pair proximity alone misses internal stress concentrations (e.g., the stress riser at a probe's diameter step that is not at a contact pair) and over-refines on glancing contact where stress is low. Stress-gradient subsumes both cases. Contact-driven is available as a diagnostic flag for debugging IPC behavior, not as a production-default trigger.
