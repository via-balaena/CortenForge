# Working Principles — Sharpen the Axe

> Extracted from `MASTER_PLAN.md` §0 during the 2026-04-09 doc-tree refactor.

This initiative is run on the *sharpen-the-axe* discipline. **We do not
rush.** The goal is not to ship fast; it is to ship work we are proud of,
on a foundation that doesn't have to be redone.

The house-building analogy: best plans, best materials, best subcontractors.
Pride, vision, wisdom. Apply these principles before any tactical decision
on this branch:

- **Recon depth before commitment.** Take an extra round of investigation
  rather than commit to a scheme on intuition. When a question has a
  literature answer (constrained Langevin, BAOAB error analysis, Landauer
  bound, stochastic resonance), read the literature first. The cost of an
  extra recon round is hours; the cost of building on a wrong foundation
  is months.
- **Validation must pass with margin.** Every phase has a validation gate
  in The Gap. The gate must pass with *margin*, not at threshold. If a
  test passes by 1% on a metric where the theory predicts 0%, that is a
  finding to investigate, not a green light.
- **Two schemes, then choose.** When proposing an algorithm, propose two
  approaches and compare honestly before committing. Single-option
  proposals are sketches, not decisions.
- **Document the *why*, not just the *what*.** Every meaningful decision
  goes in the Recon Log with its reasoning. Future-us — or a beginner
  reading the code months from now — should be able to reconstruct *why*
  we chose what we chose without having to re-derive it.
- **No batching.** One phase fully shipped, validated, and reviewed before
  the next opens. No batching of validation runs across phases. No
  batching of recon items across phases. One thing at a time, with
  attention.
- **Foundations over scaffolding.** When a shortcut creates technical debt,
  pay the debt now. The A-grade bar is not negotiable on this line. If a
  fix is foundational, take it even if it's breaking — bad foundations
  compound across phases more than anywhere else in the project.
- **Stop and ask when uncertain.** If a recon item turns up something
  surprising or contradictory, stop the implementation cadence and surface
  it. Surprises are information; ignoring them costs more than addressing
  them.

These principles apply with strength to *every* phase of this initiative,
including the early ones. Phase 1 is not a "warm-up" — it is the
foundation, and a sloppy Phase 1 will undermine every phase above it.
