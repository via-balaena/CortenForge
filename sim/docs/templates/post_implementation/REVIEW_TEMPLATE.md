# {SPEC_NAME} — Post-Implementation Review

**Spec:** `{path to SPEC_*.md}`
**Implementation session(s):** {session number(s) from SESSION_PLAN.md}
**Reviewer:** {human / AI agent}
**Date:** {YYYY-MM-DD}

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| {behavior 1} | {what MuJoCo does} | {what we did before — from spec} | {what we do now — cite file:line} | {Yes / Partial / No} |
| {behavior 2} | ... | ... | ... | ... |

**Unclosed gaps:**
{List any behaviors where the gap is not fully closed. Each needs an action.}

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

**How to grade each section:**

- Read the spec section (algorithm, file path, MuJoCo equivalent, design
  decision, before/after code).
- Read the actual implementation.
- Compare. Grade honestly.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. {section title from spec}

**Grade:** {Pass / Weak / Deviated / Missing}

**Spec says:**
{1-2 sentence summary of what the spec requires}

**Implementation does:**
{1-2 sentence summary of what the code actually does. Cite file:line.}

**Gaps (if any):**
{What's different. Be specific — name the exact behavior gap.}

**Action:** {None / Fix now / Track as future work}

### S2. {section title}

**Grade:**
**Spec says:**
**Implementation does:**
**Gaps (if any):**
**Action:**

{Continue S3, S4, ... for every spec section.}

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | {brief description} | {test function name(s)} | {Pass / Fail / No test} | {any caveats — e.g., "tolerance loosened from 1e-12 to 1e-6"} |
| AC2 | {brief description} | {test function name(s)} | {Pass / Fail / No test} | |
| AC{N} | {code review AC} | — (code review) | {Pass / Fail} | {what was reviewed} |

**Missing or failing ACs:**
{List any ACs that don't have passing tests or that failed code review.
Each needs an action: fix now, or track with rationale.}

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | {description from spec's test plan} | {Yes / No / Partial} | {test function name or "—"} | |
| T2 | ... | ... | ... | ... |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| {edge case from spec} | {why it matters — from spec} | {Yes / No} | {test name or "—"} | {if No, why and what action} |

**Missing tests:**
{List any planned tests or edge cases that weren't implemented.
Each needs an action: write now, or track with rationale.}

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| {from spec's Behavioral Changes table} | {Yes / No / Different than expected} | {if different, explain} |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| {file from spec} | {Yes / No} | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| {file} | {reason} |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| {test from spec's Existing Test Impact table} | {what spec predicted} | {what actually happened} | {Yes / No} |

**Unexpected regressions:**
{Any test breakage the spec did not predict. Root-cause each one.}

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| {field/concept} | {porting rule from spec} | {Yes / No / Partial} | {if No/Partial, describe the gap} |

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

**What counts as weak:**

- `TODO` / `FIXME` / `HACK` comments in new code
- Hardcoded values that should come from the spec or MuJoCo
- Loose tolerances (e.g., 1e-3 where MuJoCo conformance demands 1e-10)
- Missing edge-case guards the spec calls for
- Placeholder error handling (e.g., `unwrap()` in non-test code, empty
  `Err(_) => {}` that swallows information)
- Functionality that "passes tests" but uses a different algorithm than
  the spec prescribes
- Dead code or commented-out code from debugging

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| W1 | {file:line} | {what's weak and why} | {High / Medium / Low} | {Fix now / Acceptable with rationale} |
| W2 | ... | ... | ... | ... |

**Severity guide:**
- **High** — Conformance risk. MuJoCo would produce different results.
  Fix before shipping.
- **Medium** — Code quality issue. Correct behavior but fragile, unclear,
  or not matching spec's prescribed approach. Fix if time permits, else
  track.
- **Low** — Style or minor robustness. No conformance risk. Track if not
  fixing now.

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation that's out of scope. **The goal:
nothing deferred is untracked.**

For each item, verify it appears in at least one of:
- `sim/docs/todo/` (future_work file or spec_fleshout)
- `sim/docs/ROADMAP_V1.md`
- The umbrella spec's Out of Scope section (if applicable)

If it's not tracked anywhere, it's a review finding — add tracking now.

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| {out-of-scope item 1} | Out of Scope, bullet 1 | {file path or "NOT TRACKED"} | {DT-xxx / §xxx / —} | {Yes / No — needs tracking} |
| {out-of-scope item 2} | ... | ... | ... | ... |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| {discovered item 1} | {what prompted the discovery} | {file path or "NOT TRACKED"} | {DT-xxx / —} | {Yes / No} |
| {discovered item 2} | ... | ... | ... | ... |

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| {gap description} | {deviation taken} | {Yes — Rev N / No — needs update} | |

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
{paste the summary line from cargo test, e.g.:}
{test result: ok. 2187 passed; 0 failed; 0 ignored}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings — list them}
**Fmt:** {clean / issues}

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | {All closed / N unclosed} |
| Spec section compliance | 2 | {All Pass / N sections need attention} |
| Acceptance criteria | 3 | {All pass / N failing or missing} |
| Test plan completeness | 4 | {All written / N missing tests} |
| Blast radius accuracy | 5 | {All predicted / N surprises} |
| Convention fidelity | 6 | {All correct / N mismatches} |
| Weak items | 7 | {None / N items — M fixed, K tracked} |
| Deferred work tracking | 8 | {All tracked / N items need tracking} |
| Test health | 9 | {Green / Issues} |

**Overall:** {Ship / Fix then ship / Needs rework}

**Items to fix before shipping:**
1. {item — from any section above}
2. ...

**Items tracked for future work:**
1. {item — tracking ID}
2. ...

---

## How to Use This Template

1. **Copy** this file into the spec's directory. Name it
   `{SPEC_NAME}_REVIEW.md` (e.g., `SPEC_A_REVIEW.md`).

2. **Fill in sections 1-9** by reading the spec and implementation
   side-by-side. Be thorough — the whole point is to catch things now
   rather than rediscovering them months later.

3. **Section 7 (Weak Items) is the most important section for code
   quality.** Don't rush it. Read the actual implementation line by line
   against the spec. A "Pass" in section 2 means the algorithm is right;
   section 7 asks whether the *code* is right.

4. **Section 8 (Deferred Work) is the most important section for project
   health.** If deferred items aren't tracked, they're forgotten. Check
   every "out of scope" bullet and every "we'll handle that later" from
   the implementation session.

5. **Section 5 (Blast Radius) is the best section for process learning.**
   Compare what the spec predicted against what actually happened. Patterns
   in the surprises (unexpected file changes, unpredicted test breakage)
   reveal blind spots in the spec process itself.

6. **Present section 10 (Verdict) to the user.** If there are items to fix
   before shipping, fix them in the same session. If there are untracked
   deferred items, add tracking now.

7. **After fixing all "fix before shipping" items,** update the relevant
   rows in sections 2/3/7 to reflect the fixes, and re-run the verdict.
