# How this study is produced

The findings and design decisions in this book are the kind of thing
that is easy to get wrong. "I read the code and here is what I saw"
is reliable when the reader is fresh and careful; it is unreliable
when the reader has been staring at the same files for hours and has
started to normalize things. The study is full of the second kind of
reading. The review protocol described in this chapter exists to
push the reliability back up.

The protocol has two parts: a factual pass that verifies every
concrete claim, and a thinking pass that challenges the reasoning.
Every chapter gets both. Some chapters get both twice. The highest-
stakes chapters get human review on top of that.

## Part 1 — The factual pass

Every chapter in this book makes concrete claims: file paths, line
numbers, function signatures, behavioral assertions about what the
code does in specific circumstances, numerical citations ("the
constant is defined at line 57"), cross-references to other parts
of the study. All of these are falsifiable by reading the source.
All of them are verified against the source before the chapter is
committed.

The mechanical procedure is:

1. After a chapter is drafted, every falsifiable claim is extracted
   into a verification checklist — literally a bulleted list of
   "at `file.rs:line`, the function `foo` takes `(a: T, b: U)` and
   returns `Result<V, W>`" style assertions.
2. The checklist and the chapter go to a verifier — a focused
   sub-agent with no prior context on this work — with a specific
   instruction: "for each item in the checklist, check it against
   the current source and report any discrepancy."
3. The verifier's output is a table: `claim → verified / discrepancy`,
   with specifics for any item that did not match.
4. Discrepancies get fixed in the chapter before commit. If a
   discrepancy is substantive — the finding was actually wrong, not
   just the line number — the chapter's *argument* may need to
   change, in which case the chapter goes back through the thinking
   pass.
5. The verifier's output is preserved as a small per-chapter review
   log (see "the review log" below).

This pass is cheap. It catches line-number drift, signature
mistakes, stale citations, and the small-but-embarrassing errors
that erode reader trust. It is mandatory for every chapter, no
exceptions.

## Part 2 — The thinking pass

Factual accuracy is not enough. A chapter can be factually correct
and still make a bad argument — by omitting alternatives, by
glossing over tradeoffs, by assuming a premise that does not hold,
by confidently concluding something the evidence does not actually
support. The thinking pass exists to push back on the reasoning.

For every chapter that makes an argument — which is most of them —
a cold reader looks at it with a specific brief:

- What does this chapter assume? Is any assumption unstated? Is any
  unstated assumption controversial?
- What alternatives did the chapter not consider? Why? Would
  considering them change the conclusion?
- Where does the chapter say "obviously" or "clearly"? What work
  is that word doing?
- Does the conclusion actually follow from the evidence presented,
  or is there a gap?
- Is there a simpler explanation that the chapter does not engage
  with?
- What would falsify the conclusion? Does the chapter acknowledge
  that?

The cold reader is either a sub-agent with no prior context on
this work (for low-stakes chapters — inventory, history, context)
or the original author after a deliberate context break (for
high-stakes chapters — the design decisions in Parts 1, 2, and 4).
On the highest-stakes chapters — the core design calls for
`PassiveComponent`, `Competition`, the rematch protocol — the
thinking pass is run twice, once by the original author and once
by a fresh sub-agent, and any disagreements between the two are
reconciled explicitly in the chapter.

The thinking pass is where the real quality comes from. It is
slower than the factual pass, harder to automate, and the single
most valuable piece of this protocol.

## When a second round is triggered

The default is one round of each pass per chapter. A second round
triggers if the first round finds substantive issues — not nits,
not typos, not off-by-one line numbers, but real problems with the
argument or with the verification. Examples:

- The factual pass finds that a cited function has a different
  signature than the chapter claims, *and* the chapter's reasoning
  depended on the wrong signature. Not just a citation fix.
- The thinking pass finds that an alternative the chapter dismissed
  in one sentence is actually the right answer. The chapter's
  conclusion was wrong.
- A finding in one chapter contradicts a finding in another chapter,
  and both need to be reconciled.

When a second round triggers, the fix is made and then the same
pass is run again on the revised chapter. Round 3 is possible but
signals that the chapter probably needs to be rethought from
scratch, not iterated on further.

## The review log

Every chapter has a companion review log — a short markdown file
that records:

- What claims were in the verification checklist
- What the verifier found (verified vs discrepancies)
- What fixes were applied
- Whether a thinking pass was run, by whom
- What the thinking pass surfaced
- Whether a second round was triggered, and why
- Any chapter-specific notes about confidence, remaining unknowns,
  or open questions the chapter intentionally did not resolve

The review log is committed alongside the chapter. It exists so
that a future reader can answer the question "how do I know this
chapter is trustworthy?" without having to re-read the chapter and
independently re-verify everything. The review log is the *trail*
of verification, not the verification itself — but the trail is
what makes the verification durable.

## Where human review enters

Some decisions in this study are important enough that the
author's self-review and an agent's cold read are not sufficient.
These are flagged explicitly in the relevant chapters, and the
human owner of the project is asked to do a pass. The flagged
chapters are, at minimum:

- Part 1 chapter on per-env RNG seed derivation
- Part 1 chapter on the `PassiveComponent` signature change
- Part 2 chapter on the `Competition` v2 API shape
- Part 2 chapter on the rematch protocol (budget unit, seed count,
  statistical gate)
- Part 4 execution plan, particularly PR boundaries

For these chapters, the author pauses after the self-review and
agent review, explicitly surfaces "this needs your eyes," and
waits for a response before committing. For all other chapters,
the two-pass protocol is sufficient and the author proceeds
without blocking on external review.

## What this protocol is not

It is not a substitute for *running the code*. The study records
design decisions and findings, but the design decisions are
eventually implemented in real code, and the implementation is
tested and graded through the existing `cargo xtask grade`
toolchain. A chapter can be fact-checked, thinking-passed, and
human-reviewed, and still be wrong in a way that only shows up
when the code is built. The protocol reduces the surface area for
that kind of failure; it does not eliminate it. The code ships on
test results, not on chapter reviews.

It is also not a substitute for *humility about what we do not
know*. Several chapters in this study end with a section titled
"what remains uncertain" or "open questions." These are
deliberate — they mark the places where the investigation hit a
limit and the authors are naming it rather than papering over it.
A chapter that pretends to have resolved every question is a less
trustworthy chapter, not a more trustworthy one.

The goal of the protocol is to make the chapters as reliable as a
careful review process can make them, while being clear about the
remaining uncertainty. That is the honest standard.

## Preserving pre-registration anchors across squash merges

Some chapters in this study anchor reproducibility to individual
commit hashes on the feature branch — Ch 51 §2.5 and Ch 53 §4
both tell the reader "check the ordering with `git log --reverse`
over these files," and the pre-registration claim is backed by
the chronological fact that the pre-commit landed before the
code commit it constrains. Six sections across Chs 51-53 make
that kind of verifiability claim explicitly.

The repo's merge convention is squash-merge: every PR collapses
to a single commit on main. That is fine for routine feature
work, where pre-merge history is a convenience rather than an
artifact. It is not fine for pre-registration chapters.
Squash-merging collapses `cde92f8c`, `086c04c8`, `ee6ccdbb`, and
every other hash referenced by those chapters into one merge
commit whose log entry reads "study PR #N". The individual
hashes still exist in git's object store as long as some ref
points to them, but if the only ref is the feature branch and
the branch is deleted after merge, they eventually get
garbage-collected and the pre-registration claims become
unverifiable retroactively.

The fix is a pre-squash tag. Before merging a PR that contains
pre-registration content, tag the feature branch HEAD, push the
tag to origin, and then squash-merge normally. Tags are
permanent refs, so the referenced hashes stay alive indefinitely
and the chapter's `git log --reverse` claim remains exercisable
against the tag.

**Trigger.** Not every PR needs this. Most feature work has no
pre-registration content, and collapsing its history is exactly
what squash-merge is for. The mechanical trigger is: does the
PR's diff add or modify study chapters that reference specific
commit hashes or that use `git log --reverse` verifiability
language? Self-check with `grep -rn "git log --reverse"
docs/studies/*/src/` over the PR's changes, plus a scan for
short-hash references in any added or modified chapter. If
either hit lands inside the PR's diff, the PR is a pre-squash-tag
PR.

**Procedure.**

1. Wait for CI to go green on the feature branch.
2. `git tag <branch-name>-pre-squash <branch-name>` at HEAD.
3. `git push origin <branch-name>-pre-squash`.
4. Squash-merge via the GitHub UI as normal.
5. Do not delete the tag, ever. Pre-squash tags are load-bearing
   refs; a cleanup pass that deletes them silently unverifies
   every chapter that references a hash they anchor.

The `<branch-name>-pre-squash` naming is deliberately mechanical
so a future reader who finds a hash in a chapter can derive the
tag name from the branch name without guessing. If a branch
happens to get merged more than once (rare, possible under
rebase-and-reopen workflows), append a short suffix like
`-pre-squash-2`.

## A note on the sub-agents

The "sub-agent" pattern referenced above uses Claude Code's
`Agent` tool to spawn focused readers with their own context
window. Each sub-agent gets a tightly scoped prompt — "read these
specific files, answer these specific questions, report back" —
and returns a structured report. The review protocol's agent-run
passes all follow this shape.

Why it matters: the sub-agents have no context contamination from
the chapter author. They see the claim and the source and make
their own judgment. When an agent's verdict disagrees with the
chapter, the disagreement is informative — either the agent is
wrong (and the chapter can explain why) or the chapter is wrong
(and gets fixed). Either way, the process produces a verifiable
trail rather than a single author's assertion.

This is not a magic quality multiplier. Agents can miss things,
especially subtle behavioral claims or claims about code that
requires running to understand. But for the bulk of the
factual-pass work — "does this function have this signature, at
this line, returning this type" — agents are reliable and cheap,
and they free the author's attention for the harder work of
getting the thinking right.
