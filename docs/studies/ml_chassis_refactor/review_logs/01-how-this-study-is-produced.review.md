# Review log — Chapter 01: How this study is produced

## Factual pass

**Status:** Self-exempt. No code-dependent claims in this chapter.

**Rationale:** Chapter 01 is a process description. It defines the
review protocol the study uses — fact-check pass, thinking pass, review
logs, human-review triggers. None of these claims are about the
codebase; they are about how the study itself is produced. The only
verifiable claim would be "the protocol described here is what I
actually follow," and that is verifiable post-hoc by inspecting the
review logs of subsequent chapters.

Specifically:

- **"Every chapter gets both [passes]"** — a commitment, not a fact.
  Verifiable by checking future chapters' review logs.
- **"A focused sub-agent with no prior context"** — a description of
  the Agent-tool pattern. Technically a claim about Claude Code's
  tooling; accurate to my understanding of how the Agent tool works.
- **"Henderson et al. 2018"** — not cited in chapter 01, only
  foreshadowed in 00.
- **"The highest-stakes chapters get human review on top of that"** —
  an intent, not a fact. The list of high-stakes chapters (14, 15, 23,
  24, 40-43) is a forward commitment.

No factual pass needed.

## Thinking pass

**Status:** Author self-review only. Human review pending.

**Self-review notes:**

1. **The self-reference problem.** Chapter 01 defines the review
   protocol. The review protocol says every chapter should be
   verified. But chapter 01 itself cannot be verified against the
   protocol it defines — that would be circular. I resolved this by
   making chapters 00 and 01 explicitly self-exempt, documented in
   their review logs. A cold reader might object to "the protocol
   exempts itself"; the defense is that the protocol is a commitment
   about future behavior, not a factual claim about current code.
2. **Protocol vs. reality risk.** The chapter describes an ideal
   protocol. If in practice I skip a pass, cut corners on the
   thinking pass, or fail to produce a review log for a chapter, the
   book's trustworthiness claim is broken. The mitigation is that the
   review logs are themselves committed alongside chapters, so
   skipping one is visible. If a future reader sees a chapter without
   a review log, they should assume the chapter was not reviewed and
   treat its claims with commensurate skepticism.
3. **The "what this protocol is not" section.** I added two
   disclaimers at the end: it's not a substitute for running the
   code, and it's not a substitute for humility about what we don't
   know. Both feel important — without them, the chapter overclaims.
   With them, the chapter is honest about its limits. Keep.
4. **The sub-agent section at the end.** Somewhat tool-specific — it
   references Claude Code's `Agent` tool by name. A reader unfamiliar
   with Claude Code might find this jarring. Kept because the
   protocol is literally implemented using that tool and obscuring it
   would be dishonest. If the user wants this section toned down or
   moved to an appendix, easy second-round fix.
5. **Length.** Chapter 01 is ~1200 words, which is longer than I'd
   originally budgeted for a methodology chapter. The length is
   justified by the number of moving parts (two passes, review logs,
   human review triggers, what-this-is-not) each deserving explicit
   treatment. Shorter = easier to skim but risks the reader not
   understanding a commitment the rest of the book relies on. Kept
   at full length; candidate for trimming if user pushes back.

**Open questions:**

- Should the review log files themselves be listed in `SUMMARY.md`
  (visible as book pages) or kept separate (as they currently are,
  under `review_logs/`)? Current instinct: keep separate, so the
  book reads cleanly and the logs are verifiable by people who want
  to audit the trail. Ask user if uncertain.
- Does the sub-agent section belong in chapter 01, or should it move
  to an appendix ("How the AI-assisted workflow works")? Leaning
  toward keeping it in 01 because it's load-bearing for the
  methodology claim, but open to moving it.

## Second round

**Triggered:** No (pending first-round human review).

## Status

Drafted, self-reviewed, uncommitted at time of writing. Awaiting human
review of tone and structure. Any tone changes to chapter 00 will
likely propagate here for consistency.
