# Next Session: Multi-Contact Collision Recon

## Context

We're building a solver comparison example (PGS vs CG vs Newton on a stacked-box scene). Stress testing revealed that the solver example can't work yet because of a foundational gap in the collision system: **box-plane returns 1 contact (lowest corner), but MuJoCo returns up to 4 (all penetrating corners)**. With 1 contact, boxes tip over and stacks collapse.

We attempted a quick fix (iterate all 8 vertices, return penetrating ones) — it worked for box-plane (4 contacts, box_a stable), but:
- **Noslip postprocessor broke**: slip went from 0.015 to 0.88 (worse, not better) on a box-on-tilted-plane test. 2 tests failed.
- **CG warmstart threshold shifted**: avg iterations went from <80 to 92.6. 1 test failed.
- **Box-box still single contact**: stacked box_b still topples because box-box uses GJK (1 contact). MuJoCo uses SAT + face clipping (~750 LOC) for up to 8 contacts.

We reverted the change. The codebase is clean on branch `feature/integrator-examples`.

## What We Don't Know

The multi-contact change touched collision detection but broke downstream constraint processing. Before fixing collision, we need to understand the full constraint pipeline's assumptions:

### 1. Single-contact-per-pair assumptions
Map every place in the constraint pipeline that assumes at most 1 contact per geom pair:
- `sim/L0/core/src/constraint/mod.rs` — constraint row generation from contacts
- `sim/L0/core/src/constraint/solver/pgs.rs` — does PGS assume row grouping?
- `sim/L0/core/src/constraint/solver/newton.rs` — Hessian assembly
- `sim/L0/core/src/constraint/solver/cg.rs` — preconditioner
- `sim/L0/core/src/constraint/noslip.rs` (or wherever noslip lives) — friction cone processing
- Warmstart key generation — does it handle multiple contacts from same pair?

### 2. Noslip postprocessor structural analysis
The noslip test uses `cone="elliptic"` with a box on a 3-degree tilted plane. With 4 contacts:
- Does noslip iterate over friction rows grouped by contact, or flat across all rows?
- Does it assume a specific relationship between normal and tangent rows?
- Could 4 overlapping contacts from the same face create conflicting friction demands?
- Check MuJoCo's noslip implementation for comparison (`mj_solveNoslip` in engine_solver.c)

### 3. Contact row layout for multi-contact pairs
When one geom pair produces N contacts, each with condim=3:
- How are the constraint rows ordered? (all normals first, then tangents? or per-contact: normal, tan1, tan2?)
- Does the solver exploit this ordering?
- Does warmstart key generation produce unique keys for each contact in a pair?

### 4. Box-box collision gap
- `collide_box_box` in `pair_cylinder.rs` — what does it actually return? Is it GJK/EPA single contact, or does it already have SAT?
- The function returns `Vec<Contact>` (line 392) — does it already return multiple contacts?
- If so, why does box_b still topple? Maybe the SAT is single-contact?
- What would it take to implement MuJoCo's full `mjc_BoxBox` (SAT + Sutherland-Hodgman face clipping)?

### 5. Minimum viable scope
Given all the above, what is the smallest correct change that:
- Enables stable box-plane stacking (4 contacts per face)
- Doesn't break noslip (or fixes noslip to handle multi-contact)
- Doesn't break warmstart
- Passes all existing tests

## Key Files

| File | Role |
|------|------|
| `sim/L0/core/src/collision/plane.rs` | Box-plane collision (currently single contact) |
| `sim/L0/core/src/collision/narrow.rs` | Narrowphase dispatch |
| `sim/L0/core/src/collision/pair_cylinder.rs:392` | Box-box collision |
| `sim/L0/core/src/constraint/mod.rs` | Constraint row generation + solver dispatch |
| `sim/L0/core/src/constraint/solver/newton.rs` | Newton solver (89% fallback with 1 contact) |
| `sim/L0/core/src/constraint/solver/pgs.rs` | PGS solver |
| `sim/L0/core/src/constraint/solver/cg.rs` | CG solver |
| `sim/L0/core/src/constraint/impedance.rs` | efc_D computation (Hessian conditioning) |
| noslip postprocessor (find location) | Broke with multi-contact |
| `sim/L0/tests/integration/noslip.rs` | Failing test (box on tilted plane) |
| `sim/L0/tests/integration/newton_solver.rs:730` | Failing test (same scene) |
| `sim/L0/tests/integration/cg_solver.rs:415` | CG warmstart threshold |
| `sim/docs/todo/archived/future_work_1.md` | Multi-contact warmstart docs |
| `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C.md` | Collision completeness roadmap |

## MuJoCo Reference

- `mjc_PlaneBox` in `engine_collision_primitive.c` — iterates 8 vertices, filters by `ldist > 0`, returns up to 4 contacts
- `mjc_BoxBox` in `engine_collision_box.c` — full SAT + Sutherland-Hodgman, ~750 LOC, returns up to 50 contacts
- `mj_solveNoslip` in `engine_solver.c` — noslip postprocessor (verify it handles multi-contact)
- Contact row layout: MuJoCo groups rows per contact (normal, tan1, tan2 for condim=3)

## What's Already Done

- MJCF energy flag cleanup (14 examples) — done, passing clippy
- Integrators spec — done, implemented
- Solvers spec draft — at `examples/fundamentals/sim-cpu/solvers/SOLVERS_SPEC.md`
- Solvers spec needs update after this recon (scene choice, thresholds, etc.)

## Deliverable

A written analysis (not code) answering questions 1-5 above, with a clear recommendation on scope and ordering for the multi-contact fix. The analysis should cite exact line numbers and explain the constraint pipeline's assumptions so we can make the change confidently.
