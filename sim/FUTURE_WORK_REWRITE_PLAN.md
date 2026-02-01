# FUTURE_WORK.md Rewrite Plan

## Goal
Rewrite `sim/FUTURE_WORK.md` from 7 items (491 lines) into a comprehensive spec covering all verified gaps, with consistent structure, acceptance criteria, explicit priority rationale, and a dependency graph.

## Execution Steps

Each step writes one section to `sim/FUTURE_WORK.md`. Verify code references before writing.

### Step 1: Scaffold + Priority Framework + Dependency Graph
- [x] Write document header, priority framework (3 axes: RL Impact, Correctness, Effort), summary table, and ASCII dependency DAG.
- [x] No code references needed — pure structure.

### Step 2: Group A — Solver & Linear Algebra (Items 1–3)
- [x] **Item 1: In-Place Cholesky** — existing #1, add acceptance criteria.
- [x] **Item 2: Sparse L^T D L** — existing #7 renumbered, add acceptance criteria.
- [x] **Item 3: CG Contact Solver** — existing #2, add prerequisites note (#1/#2 benefit but not hard deps), flag fallback-on-failure testing concern, add acceptance criteria.
- [x] Verify all file:line references against current source before writing.

### Step 3: Group B — Pipeline Integration (Items 4–6)
- [x] **Item 4: Tendon Pipeline** — NEW full spec: `mj_fwd_tendon()`, `mj_fwd_tendon_vel()`, tendon passive forces, tendon actuation wiring. Based on verified Model fields (`tendon_stiffness`, `tendon_damping`, etc.) and Data scaffolds (`ten_length`, `ten_velocity`, `ten_force`, `ten_J`).
- [x] **Item 5: Muscle Pipeline** — NEW full spec: activation dynamics, Hill-type force model, `act` array update, depends on #4.
- [x] **Item 6: Sensor Completion** — NEW full spec with table of all 10 stubbed/dropped sensors.
- [x] Verify all file:line references against current source before writing.

### Step 4: Group C — Integrator & Dynamics (Items 7–8)
- [x] **Item 7: Integrator Rename** — split from existing #3 part 1. Rename `Implicit` → `ImplicitSpringDamper`, clarify this is semantic only, foundation for future non-diagonal coupling.
- [x] **Item 8: True RK4 Integration** — split from existing #3 part 2. Multi-stage RK4 with actual `k1..k4` evaluations. Add acceptance criteria.
- [x] Verify all file:line references against current source before writing.

### Step 5: Group D — Deformable Body (Item 9)
- [x] **Item 9: Deformable Body Integration** — existing #4, add substep iteration spec to address XPBD/contact ordering problem, add acceptance criteria.
- [x] Verify all file:line references against current source before writing.

### Step 6: Group E — Scaling & Performance (Items 10–11)
- [x] **Item 10: Batched Simulation** — existing #5, add explicit single-model design constraint note, add acceptance criteria.
- [x] **Item 11: GPU Acceleration** — existing #6, kept sparse (blocked on #10).
- [x] Verify all file:line references against current source before writing.

### Step 7: Cleanup Tasks + Completed Work
- [x] **C1: Disambiguate integrators.rs** — decide: consolidate trait system with pipeline or remove dead code.
- [x] **C2: Correct stale MUJOCO_GAP_ANALYSIS.md** — update or mark superseded.
- [x] **Completed Work (Reference)** — preserved verbatim from previous FUTURE_WORK.md.
- [x] Verify completed work section is still accurate (commit hashes confirmed).

### Step 8: Final Review
- [ ] Read the full rewritten `sim/FUTURE_WORK.md` end-to-end.
- [ ] Verify all file:line references one more time.
- [ ] Check for internal consistency (numbering, cross-references, dependency graph matches items).

## Per-Item Template (standardized)

Every item gets this structure:
```
### N. Title
**Status:** Not started | **Effort:** S/M/L/XL | **Prerequisites:** #N, #M or None

#### Current State
What exists today, with `file:line` references.

#### Objective
One sentence.

#### Specification
Algorithms, signatures, data flow.

#### Acceptance Criteria
1. Binary, testable criterion.
2. ...

#### Files
- `path/to/file.rs` — create | modify
```

## Files Modified
- `sim/FUTURE_WORK.md` — complete rewrite (only production file changed)
- `sim/FUTURE_WORK_REWRITE_PLAN.md` — this plan file (delete when done)
