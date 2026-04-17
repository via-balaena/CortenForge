# How this study is produced

This chapter names the process the book is written under, because the process is part of the thesis. A soft-body study written in the received review-paper form would be a taxonomy: list the solvers, list the papers, list the open problems, stop. That is not this book. This book is an engineering study written to be *executed* — every decision in Parts 1–12 is a decision the book expects to make, or to surface (unresolved, named) as a research question. The process below is what makes that execution credible.

## Three passes

The book is written in three passes, not one.

**Pass 1 — Spine.** All top-level chapter pages — the entries directly under a `# Part X` header in `SUMMARY.md` — are written so the whole book is readable at depth 0. Parent pages get a table of their children plus one or two claims. Depth-0 leaves (chapters with no further nesting, e.g. [Part 1 Ch 03 thesis](../10-physical/03-thesis.md)) get fully written, because there is nothing to trickle to. Batched by Part; checkpoint between Parts.

**Pass 2 — Branches.** All remaining parent pages at depth 1 or 2, fleshed to overview detail. The grid is wide enough that some of these sub-chapters carry their own weight — Part 2 Ch 04's hyperelastic family, for instance, is a chapter of its own with three model sub-chapters under it.

**Pass 3 — Leaves.** The ~200 leaf pages at full depth. Code sketches, equations, references all become concrete. By the time Pass 3 begins, the spine and branches have already been pressure-tested by Pass 1 and Pass 2 reviews.

Checkpoint between passes. After Pass 1, the spine is read as a connected whole and reshuffled if needed before descending. After Pass 2, the branch list is reread and sub-chapter order is adjusted before leaves lock it in. Cheap to reorganize now, expensive after leaves are written.

The reason for the three-pass shape is that soft-body simulation is deep enough — the SOTA survey covers nine non-trivial systems, the material models alone fill a graduate course, IPC and differentiable adjoints each take a paper of their own to state cleanly — that a single-pass writing order either rushes the overview or loses the narrative. The three-pass shape lets the spine get pressure-tested before the leaves commit to it.

## Collaboration mode

The book is written by Jon Hillesheim with Claude (Anthropic) as a collaborator. Claude proposes architecture, drafts prose, runs the mdbook build, and implements the writing conventions locked in at the start of each pass. Jon drives the overall thesis, adjudicates contested design decisions, and vetoes anything that feels wrong. Where a chapter makes a judgment call that could have gone the other way, the chapter names the call explicitly rather than presenting one side as inevitable.

This is not an automation story. A book written by an LLM from a prompt that says *write a soft-body simulation textbook* would hallucinate its citations, smooth over the places where the literature disagrees, and hedge every claim that has not yet been independently settled. The collaboration mode here is the opposite: every specific number is traced to a source, every contested choice is named as contested, and the book flags its own uncertainty rather than hiding it in consensus voice.

## No hallucinated citations

Claude does not cite papers from memory. Every paper named in the book (see the [reference list](../appendices/00-references.md)) is a paper that was fetched, whose abstract or relevant section was read, and in some cases traced into for a specific claim. Where a numeric claim is drawn from a paper, that claim is cited to its source and the source is fetched as of the date the chapter is written. Where a claim is Jon's or Claude's opinion rather than a cited finding, the chapter marks it as such.

This is a ground rule, not a nice-to-have. A book whose citations drift is strictly worse than the primary literature; a book whose citations hold up becomes a navigable index into that literature. The referencing convention — inline name-year links into the appendix, per Pass 1 conventions — is designed so broken references are visible, not hidden.

## No timeline

There is no timeline. The study exists because Jon wants to build a world-class soft-body stack, and the design work — the recon, the SOTA reading, the architectural pressure-testing — is itself the point. Chapters are written until they say what they need to say and then they stop. Parts are reorganized when the spine reveals the current order is wrong. The book ships when it is done, not by a date.

This is explicit because the absence of a timeline is load-bearing for the rest of the process. A study on a deadline would settle the three-pass question by skipping Pass 1 and writing leaves. A study without a deadline can afford the spine. The quality of the final result is bounded by the quality of the spine, which is bounded by whether the spine was allowed to be reorganized before the leaves were committed to.

## Grading

Each chapter is graded before it is considered finished. The criteria translate CortenForge's shipped-code bar ("A-grade or it doesn't ship") to prose: does the chapter make a thesis, does the thesis have evidence, does the evidence support the thesis, is the prose compact, are the cross-references accurate, is the code realistic. Chapters that do not meet the bar are rewritten, not landed with a `TODO — tighten`. The grade applies per chapter, not per Part.

## What this process excludes

- **Exhaustive derivations of every equation.** Derivations are cited and the book trusts the reader to follow the citation when it matters. Re-deriving known results in 300-line appendix sections inflates the book without improving it.
- **A beginner's intro to FEM.** The [reading guide](04-reading-guide.md) points at external resources for readers who need one. The book assumes working familiarity with continuum mechanics, FEM, and the basic ML autograd machinery.
- **Benchmarks of `sim-soft` against the Ch 02 solvers.** Those are [Part 12 Ch 02 — first-working milestone](../120-roadmap/02-first-working.md) work, executed after the code exists. Writing benchmarks against an unimplemented stack would be fiction.

The book is honest about what it is: a ground-up design study for a soft-body stack that does not yet exist, written by its designers.
