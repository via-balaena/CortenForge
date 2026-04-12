# Review log — Chapter 00: Why this study exists

## Factual pass

**Status:** Self-exempt. No code-dependent claims in this chapter.

**Rationale:** Chapter 00 is pure context-setting. It summarizes the
session that led to the study's creation, restates three findings that
are developed in detail in later chapters (Parts 1 and 2), and outlines
the execution plan that Part 4 will spell out. Every concrete claim in
this chapter is either (a) a forward reference to a later chapter where
it gets its own factual pass, or (b) a piece of historical context about
the session itself.

Specifically:

- **"Each crate landed in roughly a day or two"** — historical claim
  about the original `sim-ml-bridge` and `sim-thermostat` development.
  Not verifiable from the current code, but also not load-bearing — it's
  framing for why this study exists. If the claim is wrong about "a day
  or two" specifically, the framing is still right.
- **"Henderson et al. 2018"** — external citation, pointing at a real
  paper (*Deep Reinforcement Learning That Matters*, AAAI 2018). Load-
  bearing for Part 2 chapter 20, where it will be cited with a proper
  reference and the specific finding (same-algorithm seed variance flips
  rankings) will be verified against the paper.
- **The three findings** — each is summarized here and then developed in
  detail in its respective chapter. Verification happens in those
  chapters (11, 12 for finding 1; 23, 24 for finding 2; 21, 22 for
  finding 3), not here.
- **"Three PRs"** — forward reference to Part 4 chapters 40-43. Those
  chapters get their own factual/design pass.

No file:line citations appear in chapter 00. No function signatures.
No behavioral claims that depend on reading the source. Therefore the
factual pass is self-exempt.

## Thinking pass

**Status:** Author self-review only. Human review pending (user reading
on receipt of phase 1 deliverables).

**Self-review notes:**

1. **Tone check.** The chapter uses phrases like "it is supposed to be
   longer" and "it is a cost-benefit calculation" which are slightly
   more editorial than strictly descriptive. This is deliberate — the
   chapter is the motivating statement for the whole study, and bland
   neutrality would undersell the reason to spend time here. The tone
   is informed by the user's explicit framing of the study as "a study,
   not a fix." If the user pushes back on tone, this is the first lever
   to adjust.
2. **Honesty about my own role.** The chapter uses third-person ("the
   reviewer," "the author") deliberately to avoid sounding self-
   congratulatory about the review pass. The reviewer is me (Claude).
   The construction spec was my earlier draft. I edited both. The
   chapter does not disclose this directly because it would derail the
   argument; the methodology chapter (01) makes the AI-review pattern
   explicit.
3. **The "third finding" framing.** I explicitly call out that finding
   3 only surfaced when I thought harder. This is honest — it did
   emerge late — but a reader might read it as false modesty. I left
   it because the alternative (presenting three findings as
   simultaneous discoveries) would be a lie.
4. **The "years of physics-aware ML research" claim.** This is
   load-bearing for the "why bother with a study" argument. It's
   rooted in the user's stated research direction (the thermo-RL
   loop vision, per MEMORY.md). If that direction changes, the
   chapter's motivation weakens. Flagged as a sensitivity point but
   not worth softening unless the direction actually shifts.
5. **"If you are reading this book looking for a fast answer..."** —
   the closing paragraph is somewhat didactic. Considered removing
   it. Kept because it sets expectations for the reader's mode of
   attention. Second-round candidate if user finds it preachy.

**Open questions:**

- Does the tone match what the user wants for the rest of the book?
  (User is reviewing now.)
- Is the "three findings" narrative the right organizing structure
  for Part 0, or should the findings be foreshadowed differently?
- Chapter 01 (methodology) and chapter 02 (history) are still
  sequential front matter. Does the reader experience improve if 02
  comes before 01? Open for adjustment.

## Second round

**Triggered:** No (pending first-round human review).

## Status

Drafted, self-reviewed, uncommitted at time of writing. Awaiting human
review of tone and structure before the rest of Part 0 is drafted in
the same pattern.
