# Phase 12 — Conformance Test Suite: Session Plan

15 sessions, each self-contained. The umbrella spec
(`PHASE12_UMBRELLA.md`) coordinates across context boundaries.

Phase 12 is the gate phase — it validates CortenForge against MuJoCo
3.4.0 at every pipeline stage. It builds a four-layer conformance test
suite (§45) and expands golden-file coverage to all 25 disable/enable
flags (DT-97). Failures found here drive fix iterations back into
Phases 1–11 until green.

The four layers from §45:
- **Layer A** — Self-consistency (no MuJoCo dependency): forward/inverse
  equivalence, island/monolithic solver equivalence, determinism,
  integrator energy ordering, sparse/dense mass matrix equivalence
- **Layer B** — Per-stage reference: compare each pipeline stage output
  (FK, CRBA, RNE, passive, collision, constraint, actuation, sensors,
  tendons, integration) against MuJoCo 3.4.0 reference data
- **Layer C** — Trajectory comparison: multi-step trajectory tests with
  per-step per-field diagnostics to isolate which stage first diverges
- **Layer D** — Property/invariant tests (no MuJoCo dependency): momentum
  conservation, energy conservation, quaternion normalization, contact
  force feasibility, mass matrix properties

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Design Decisions

### Why Layers A+D are a T1 session, not a full spec cycle

The existing integration test suite already has **1,216 tests across
57 modules**, including:
- Forward/inverse round-trip (`inverse_dynamics.rs`)
- Energy conservation (`rk4_integration.rs`)
- Determinism (`batch_sim.rs` — serial vs parallel identity)
- FK/CRBA/RNE analytical validation (`validation.rs` — 9 tests)
- Contact feasibility patterns (`collision_test_utils.rs`)

A full 5-session rubric→spec→implement→review cycle for Layer A+D
would over-engineer ~15-20 targeted gap-fill tests. Instead, Session 3
audits existing coverage, identifies gaps, and writes systematic
conformance versions in `mujoco_conformance/`. The existing integration
tests are ad-hoc and feature-specific; the conformance suite is
systematic and complete. Some intentional overlap is expected.

### Why Layers B and C are separate specs

Layer B (per-stage reference) and Layer C (trajectory comparison) were
originally grouped into one spec. Stress testing revealed:
- **Layer B alone covers 10+ pipeline stages × multiple models × Python
  reference generation** — easily the heaviest deliverable in Phase 12
- **Layer C has distinct challenges**: chaos-aware tolerances, step-limited
  comparison windows, divergence diagnostics
- **Different infrastructure**: Layer B needs per-stage snapshots (single
  forward() call); Layer C needs multi-step trajectory capture
- Splitting gives each spec a focused rubric and right-sized sessions

### Known-failure strategy

Phase 12 tests will discover conformance failures — that's the point.
Strategy for tests that fail due to known CortenForge gaps:
1. Write the test with the correct MuJoCo-matching assertion
2. Mark with `#[ignore]` and a tracking comment: `// CONFORMANCE GAP: <description> — see DT-XXX or Phase N`
3. `cargo test -p sim-conformance-tests` passes (ignored tests skip)
4. `cargo test -p sim-conformance-tests -- --ignored` runs only
   known-failing tests for monitoring progress
5. As fixes land in Phases 1–11, un-ignore the test and verify it passes
6. Session 15 (Gate Triage) compiles the full `#[ignore]` inventory

This gives clean CI signal while preserving the correct test as the
standard. Tests are never weakened to pass — tolerances and assertions
reflect MuJoCo ground truth.

### Tolerance philosophy

Per-stage tolerances based on algorithmic properties:

| Stage | Tolerance | Rationale |
|-------|-----------|-----------|
| FK (xpos, xquat, xmat) | 1e-12 | Exact arithmetic modulo FP accumulation through kinematic tree |
| CRBA (mass matrix M) | 1e-12 | Same — CRBA is exact given FK |
| RNE (qfrc_bias) | 1e-10 | Accumulated FP in tree traversal (gravity, Coriolis) |
| Passive (qfrc_passive) | 1e-10 | Spring/damper forces — direct computation |
| Collision (contacts) | STRUCTURAL | Contact enumeration order may differ; compare sets, not arrays. Depths/normals: 1e-6 |
| Constraint (efc_force) | 1e-4 | Iterative solver convergence — different paths, same basin |
| Actuation (qfrc_actuator) | 1e-10 | Direct computation from ctrl/act |
| Sensors (sensordata) | 1e-8 | Depends on upstream stage tolerance |
| Tendons (ten_length) | 1e-10 | Geometric computation |
| Integration (qpos, qvel) | 1e-8 single step | Accumulates across steps — see Layer C |
| Trajectory (Layer C) | Growing | Step-aware: `tol(step) = base_tol * (1 + step * growth_rate)`. Non-contact models get tighter base. Contact models get wider base + faster growth due to chaotic sensitivity. |

These are starting points — the rubric and spec sessions will refine them
against actual MuJoCo C source analysis.

### Reference data generation infrastructure

Session 4 builds the reference generation pipeline as a dedicated T1:
- `gen_conformance_reference.py` — single script generating all reference data
- Pinned to `mujoco==3.4.0` (hard requirement for reproducibility)
- Canonical conformance models checked into `assets/golden/conformance/`
- Reference data as **individual `.npy` files** (one field per file) — no
  `.npz` archives. This avoids ZIP decompression in Rust and lets the
  existing `parse_npy()` utility handle all reference data with zero new
  parsing infrastructure.
- Reference files checked into repo (no MuJoCo build dependency for Rust)
- Documented regeneration workflow: `uv pip install mujoco==3.4.0 numpy && uv run ...`

This is a separate T1 session because it's infrastructure that both
Spec A (Layer B) and Spec B (Layer C) depend on, and it has its own
failure modes (Python environment, mujoco version, model design).

### Iteration protocol

Phase 12 is designed to be iterative:
1. **Write tests** (Sessions 2-14): Write correct tests, mark failures with `#[ignore]`
2. **Gate triage** (Session 15): Compile `#[ignore]` inventory, assign each failure
   to the responsible phase/DT, prioritize by pipeline order (FK → dynamics →
   constraint → sensor → integration)
3. **Fix iteration** (separate sessions, not Phase 12): Fix the root cause in the
   responsible phase. Each fix session ends with un-ignoring the conformance test.
4. **Re-run**: After each fix batch, run `cargo test -p sim-conformance-tests -- --ignored`
   to check remaining failures
5. **Gate passes** when `cargo test -p sim-conformance-tests -- --ignored` has zero
   failures (all tests un-ignored and passing)

---

## Task Assignment

Every Phase 12 task is assigned to exactly one deliverable:

| Task | Deliverable | Status | Rationale |
|------|-------------|--------|-----------|
| DT-97 | T1 session (Session 2) | | Golden flag expansion — extend gen script to all 25 flags + enrich canonical model |
| §45 Layer A | T1 session (Session 3) | | Self-consistency tests — audit existing 1,216 tests, write systematic gap-fills |
| §45 Layer D | T1 session (Session 3) | | Property/invariant tests — grouped with Layer A (both MuJoCo-independent) |
| §45 Ref Gen | T1 session (Session 4) | | Reference data infrastructure — canonical models + Python gen script + .npy files |
| §45 Layer B | Spec A | | Per-stage reference tests — systematic MuJoCo 3.4.0 comparison at each pipeline stage |
| §45 Layer C | Spec B | | Trajectory comparison — multi-step golden-file validation with divergence diagnostics |

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: DT-97 golden flags)            <- independent
    |
    +-- Session 3 (T1: Layer A+D audit + gap-fill)     <- independent
    |
    +-- Session 4 (T1: Reference gen infrastructure)   <- independent
    |       |
    |       v (hard dep: Spec A needs reference .npy files)
    |
    +-- Sessions 5-9 (Spec A: Layer B per-stage ref)   <- depends on Session 4
    |       |
    |       v (soft dep: Spec B reuses Layer B infrastructure)
    |
    +-- Sessions 10-14 (Spec B: Layer C trajectory)    <- soft dep on Spec A
    |
    +-- Session 15 (Gate Triage)                        <- depends on all above
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| Session 4 (Ref Gen) -> Spec A (Layer B) | **Hard** | Spec A's per-stage tests load reference `.npy` files generated by Session 4's Python script. Without reference data, Layer B tests have nothing to compare against. |
| DT-97 (Session 2) -> Spec B (Layer C) | **Soft** | DT-97 establishes the `.npy` comparison pattern and `parse_npy()` utility. Spec B reuses this for trajectory golden files. Spec B can build its own if needed. |
| Spec A (Layer B) -> Spec B (Layer C) | **Soft** | Spec B can reuse Spec A's per-stage comparison infrastructure, tolerance constants, and diagnostic patterns. Spec B can stand alone if needed. |
| All sessions -> Session 15 (Gate) | **Hard** | Gate triage requires all tests to be written (even if `#[ignore]`d) to compile the failure inventory. |

### Parallelism

Sessions 2, 3, and 4 are fully independent and can proceed in any order
(or parallel). Spec A must wait for Session 4. Spec B ideally follows
Spec A but is not blocked. Session 15 must be last.

---

## Session 1: Phase 12 Umbrella

- [x] Complete

```
Phase 12 Conformance Test Suite -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 12 section)
2. sim/docs/todo/future_work_11.md (§45 — four-layer conformance test suite)
3. sim/docs/todo/future_work_10j.md (DT-97 — golden file generation)
4. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SESSION_PLAN.md
   (THIS FILE — read fully, especially Design Decisions section)
5. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
   (structural template — multi-spec pipeline pass)
6. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
   (structural template — T1 session + multi-spec)
7. sim/L0/tests/mujoco_conformance/mod.rs (current placeholder — 7 lines)
8. sim/L0/tests/integration/mod.rs (existing 57-module test organization)
9. sim/L0/tests/integration/golden_flags.rs (existing golden-file pattern)
10. sim/L0/tests/integration/inverse_dynamics.rs (existing forward/inverse round-trip)
11. sim/L0/tests/integration/rk4_integration.rs (existing energy conservation)
12. sim/L0/tests/scripts/gen_flag_golden.py (existing gen script)
13. sim/L0/tests/assets/golden/flags/ (existing golden data — 2 files)

Phase 12 is the gate phase. It validates CortenForge against MuJoCo
3.4.0 at every pipeline stage. Tasks: §45 (four-layer conformance
test suite) and DT-97 (golden file expansion). Failures found here
drive fix iterations back into Phases 1–11.

Write PHASE12_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Existing infrastructure summary (CRITICAL — acknowledge the 1,216
  existing tests and explain what Phase 12 adds on top):
  - What exists: 57 integration test modules, golden_flags.rs (1 flag),
    gen_flag_golden.py, validation.rs (9 FK/CRBA/RNE tests), derivatives.rs
    (149+ FD tests), spatial_tendons.rs (18 MuJoCo ref tests),
    inverse_dynamics.rs (round-trip), rk4_integration.rs (energy conservation),
    collision_test_utils.rs (tolerance hierarchy + assertion macros)
  - What Phase 12 adds: SYSTEMATIC per-stage comparison (not ad-hoc),
    trajectory-level validation, complete golden-file coverage,
    organized conformance binary separate from integration tests
  - Relationship: integration tests = per-feature regression;
    conformance tests = systematic MuJoCo-matching validation.
    Intentional overlap expected and acceptable.
- Known-failure strategy (from SESSION_PLAN.md Design Decisions):
  #[ignore] with tracking comments, CI-clean by default,
  --ignored runs known-failing tests for monitoring
- Sub-spec scope statements:
  - Spec A: Layer B (per-stage MuJoCo reference comparison)
  - Spec B: Layer C (trajectory comparison + divergence diagnostics)
- T1 scope:
  - Session 2: DT-97 (golden flag expansion to all 25 flags)
  - Session 3: Layers A+D (audit existing coverage + systematic gap-fill)
  - Session 4: Reference data infrastructure (canonical models + Python script)
- Dependency Graph (Session 4 → Spec A hard dep)
- File Ownership Matrix:
  - Session 2: golden_flags.rs, gen_flag_golden.py, assets/golden/flags/
  - Session 3: mujoco_conformance/layer_a.rs, layer_d.rs
  - Session 4: gen_conformance_reference.py, assets/golden/conformance/,
    conformance model XML files
  - Spec A: mujoco_conformance/layer_b.rs (or per-stage submodules)
  - Spec B: mujoco_conformance/layer_c.rs
  - Shared: mujoco_conformance/common.rs (parse_npy, tolerance constants)
- Test Model Registry:
  - DT-97 canonical model (flag_golden_test.xml — enriched)
  - Conformance canonical models (purpose-built, small, deterministic):
    (a) pendulum (hinge, no contacts)
    (b) double pendulum (chain dynamics)
    (c) contact scenario (sphere on plane + wall)
    (d) actuated system (motor + position servo)
    (e) tendon model (spatial + fixed tendons)
    (f) sensor-rich model (position/velocity/acceleration sensors)
    (g) flex body (deformable, if Phase 10 complete)
    (h) equality constraints (weld + joint + tendon)
    Selection criteria: exercises specific pipeline stages, deterministic,
    small enough for fast CI, complex enough to catch real bugs
- Reference Data Strategy:
  - Python script pinned to mujoco==3.4.0, uses uv
  - Individual .npy files ONLY (one field per file, NO .npz archives).
    This avoids ZIP parsing in Rust — existing parse_npy() handles everything.
  - Checked into repo — no MuJoCo build dependency for Rust tests
  - Regeneration: documented, reproducible, idempotent
  - Per-stage data: single forward() call, capture intermediate fields
  - Trajectory data: N steps, capture qpos/qvel/qacc per step
  - Canonical models designed so relevant stages produce NON-TRIVIAL
    output at qpos0 (contacts exist at initial state, tendons non-zero, etc.)
- Tolerance Strategy (from SESSION_PLAN.md Design Decisions table):
  - Per-stage tolerances with algorithmic justification
  - Collision: STRUCTURAL comparison (sets, not ordered arrays)
  - Trajectory: step-aware growing tolerance
  - All tolerances are starting points — rubric/spec refine them
- Iteration Protocol (from SESSION_PLAN.md Design Decisions):
  - Write tests → gate triage → fix in source phase → un-ignore → re-run
  - Gate passes when --ignored has zero failures
- Phase-Level Acceptance Criteria (PH12-AC1 through PH12-AC8+):
  - PH12-AC1: All 25 flag golden-file tests pass or skip gracefully
  - PH12-AC2: Layer A self-consistency tests pass without MuJoCo
  - PH12-AC3: Layer D invariant tests pass without MuJoCo
  - PH12-AC4: Layer B per-stage reference tests exist for all 10+ stages
  - PH12-AC5: Layer C trajectory tests cover contact and non-contact
  - PH12-AC6: cargo test -p sim-conformance-tests passes (ignoring known gaps)
  - PH12-AC7: No existing integration tests broken or moved
  - PH12-AC8: Gate triage inventory compiled with phase assignments
- Out of Scope:
  - GPU tests, URDF conformance, performance benchmarks
  - MuJoCo build dependency for Rust tests
  - Fixing conformance failures (that's the fix iteration, not Phase 12)
  - Flag combination tests (single-flag isolation only for DT-97)
  - Moving or modifying existing integration tests

Key differences from Phase 11 umbrella:
- Phase 12 writes tests, not production code
- Designed to be iterative — tests reveal failures for earlier phases
- Infrastructure-heavy: Python reference gen, golden files, model curation
- Two test binaries: mujoco_conformance (populated by Phase 12) and
  integration (existing 57 modules, untouched)
- Known-failure strategy via #[ignore] — unique to Phase 12
- Gate triage session (Session 15) — no equivalent in other phases
- Layers A+D are T1 (leveraging existing 1,216 tests), not full specs

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 — DT-97 Golden Flag Expansion

- [x] Complete

```
Phase 12 Conformance Test Suite -- implement T1 item (golden flag expansion).

Read these first:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
   (Known-failure strategy, Test Model Registry sections)
2. sim/docs/todo/future_work_10j.md (DT-97)
3. sim/L0/tests/scripts/gen_flag_golden.py (existing script — generates 2 files)
4. sim/L0/tests/integration/golden_flags.rs (existing test — 1 flag test)
5. sim/L0/tests/assets/golden/flags/flag_golden_test.xml (existing canonical model)
6. sim/L0/tests/assets/golden/flags/README.md
7. sim/L0/core/src/types/enums.rs (lines 615-674 — all 25 flag constants)

**DT-97 — Golden File Expansion to All 25 Flags:**

Two deliverables:

**D1: Enrich the canonical model.**
The current `flag_golden_test.xml` is a 1-DOF hinge with spring/damper/
motor/floor contact. Many flags won't produce observable qacc differences
with this model. Enrich it to exercise all flag-gated subsystems:
- Equality constraint (weld or joint equality — for DISABLE_EQUALITY)
- Fixed tendon with spring (tendon pipeline — passive force via DISABLE_SPRING, friction via DISABLE_FRICTIONLOSS)
- Joint limits that are active at qpos0 (for DISABLE_LIMIT)
- At least one sensor (for DISABLE_SENSOR / ENABLE_SENSOR)
- Contact-producing geometry with friction (for DISABLE_CONTACT,
  DISABLE_FRICTIONLOSS, DISABLE_FILTERPARENT, DISABLE_REFSAFE,
  DISABLE_MIDPHASE, DISABLE_NATIVECCD)
- Passive spring + damper on joint (for DISABLE_SPRING, DISABLE_DAMPER,
  DISABLE_EULERDAMP)
- Motor actuator with ctrlrange (for DISABLE_ACTUATION, DISABLE_CLAMPCTRL)
- Gravity (for DISABLE_GRAVITY)
- Multiple bodies for island discovery (for DISABLE_ISLAND)
- Enable energy flag compatibility (for ENABLE_ENERGY)
Keep the model small — aim for nv <= 6, fast to simulate.
After enrichment, regenerate ALL golden files (baseline + all flags).

**D2: Expand gen script + Rust tests to all 25 flags.**
1. Extend `gen_flag_golden.py`:
   - Loop over all 19 disableflags + 6 enableflags
   - For disableflags: set one flag, step 10 times, capture qacc
   - For enableflags: set one flag, step 10 times, capture qacc
   - Naming: `disable_{name}_qacc.npy`, `enable_{name}_qacc.npy`
   - Generate baseline (no flags) as `baseline_qacc.npy`
   - Total: 26 .npy files (1 baseline + 25 flags)
2. Run: `uv pip install mujoco==3.4.0 numpy && uv run sim/L0/tests/scripts/gen_flag_golden.py`
3. Extend `golden_flags.rs`:
   - Create `parse_npy()` in `mujoco_conformance/common.rs` for use by
     all Phase 12 conformance tests. NOTE: `mujoco_conformance` and
     `integration` are separate `[[test]]` binaries and CANNOT share
     code via module imports. The existing `parse_npy()` in
     `golden_flags.rs` stays where it is — ~50 lines of duplication
     is intentional and correct. Do NOT try to share code across
     test binaries.
   - Use a macro or helper function to generate one test per flag
   - Each test: load model, set flag, step 10 times, compare qacc
   - Tolerance: 1e-8 absolute (same as existing)
   - Graceful skip if .npy file missing (existing pattern)
   - Tests that fail due to CortenForge gaps: mark `#[ignore]` with
     tracking comment per known-failure strategy
4. Verify all 25 flag constants are publicly exported from sim-core
5. Update README.md with full regeneration instructions + flag list

Run `cargo test -p sim-conformance-tests` after completion. Verify
the golden_flags tests pass (or are #[ignore]d with tracking comments).
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: T1 — Layer A+D Coverage Audit + Gap-Fill

- [x] Complete

```
Phase 12 Conformance Test Suite -- implement T1 item (Layer A+D systematic tests).

Read these first:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
   (Known-failure strategy, Existing Infrastructure sections)
2. sim/L0/tests/integration/mod.rs (all 57 module declarations — scan for
   existing Layer A/D coverage)
3. sim/L0/tests/integration/inverse_dynamics.rs (forward/inverse round-trip)
4. sim/L0/tests/integration/rk4_integration.rs (energy conservation)
5. sim/L0/tests/integration/batch_sim.rs (determinism)
6. sim/L0/tests/integration/validation.rs (FK/CRBA/RNE analytical)
7. sim/L0/tests/integration/collision_test_utils.rs (tolerance hierarchy)
8. sim/L0/tests/mujoco_conformance/mod.rs (placeholder to populate)

**Layer A+D Systematic Conformance Tests:**

This session audits existing test coverage and writes targeted gap-fills
to create a systematic Layer A (self-consistency) + Layer D (property/
invariant) conformance test suite in `mujoco_conformance/`.

**Phase 1: Audit (first half of session)**
Read the existing integration test modules that relate to Layer A/D:
- inverse_dynamics.rs — forward/inverse round-trip
- rk4_integration.rs — energy conservation, integrator convergence
- batch_sim.rs — determinism (serial vs parallel)
- validation.rs — FK/CRBA/RNE analytical verification
- unified_solvers.rs, newton_solver.rs — solver equivalence patterns
- Any other modules with self-consistency or invariant tests

For each Layer A/D test category below, note whether it's:
- COVERED: existing integration test fully validates this
- PARTIAL: existing test touches this but not systematically
- MISSING: no existing test

**Layer A categories:**
1. Forward/inverse equivalence (multiple models, multiple joint types)
2. Island/monolithic solver equivalence (contact + constraint model)
3. Determinism (same input → bit-identical output, multiple runs)
4. Integrator energy ordering (implicit < semi-implicit < Euler drift)
5. Sparse/dense mass matrix equivalence (factored LDL^T vs dense CRBA)

**Layer D categories:**
6. Momentum conservation (free-flying, no gravity, no contacts)
7. Energy conservation (conservative system, implicit integrator)
8. Quaternion normalization (free/ball joints, N steps, |q|=1)
9. Contact force feasibility (normal >= 0, tangential <= mu * normal)
10. Mass matrix SPD (symmetric, positive definite, diagonal > 0)

**Phase 2: Gap-fill (second half of session)**
Create `mujoco_conformance/layer_a.rs` and `mujoco_conformance/layer_d.rs`.
Write systematic tests for PARTIAL and MISSING categories.
For COVERED categories, write a brief conformance version that exercises
the same property with a conformance-focused model (inline MJCF).

Guidelines:
- Use inline MJCF models (no external file dependency)
- Use `approx::assert_relative_eq!` with documented tolerances
- Each test function named `test_layer_{a|d}_{category}_{variant}`
- No MuJoCo reference data — these are internal validation tests
- Tests must pass. If a test reveals a CortenForge bug, mark #[ignore]
  with tracking comment per known-failure strategy.
- Update `mujoco_conformance/mod.rs` to include both modules

Import `parse_npy` and tolerance constants from `common.rs` if Session 2
created it; otherwise create `common.rs` with shared utilities.

Run `cargo test -p sim-conformance-tests --test mujoco_conformance`
after completion. All non-ignored tests must pass.
MuJoCo conformance is the cardinal goal.
```

---

## Session 4: T1 — Reference Data Generation Infrastructure

- [ ] Complete

```
Phase 12 Conformance Test Suite -- implement T1 item (reference data generation).

Read these first:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
   (Reference Data Strategy, Test Model Registry, Tolerance Strategy sections)
2. sim/L0/tests/scripts/gen_flag_golden.py (existing script — pattern to follow)
3. sim/L0/tests/assets/golden/flags/ (existing golden data structure)
4. sim/L0/core/src/types/data.rs (Data struct fields organized by pipeline stage)

**Reference Data Generation Infrastructure:**

Three deliverables:

**D1: Design and create canonical conformance models.**
Create purpose-built MJCF models in `assets/golden/conformance/models/`.
Each model is small, deterministic, and exercises specific pipeline stages:

(a) `pendulum.xml` — single hinge joint, no contacts, gravity.
    Exercises: FK, CRBA, RNE, passive (spring/damper), integration.
    Ideal for: tight-tolerance per-stage comparison. nv=1.

(b) `double_pendulum.xml` — two hinge joints in chain.
    Exercises: FK chain propagation, CRBA off-diagonal, Coriolis terms.
    nv=2.

(c) `contact_scenario.xml` — sphere(s) on plane with wall contact.
    Exercises: collision detection, constraint assembly, solver.
    Include friction for condim=3 contacts. nv=6 (free joint).

(d) `actuated_system.xml` — motor + position servo on hinge.
    Exercises: actuator force computation, ctrl→qfrc_actuator path.
    Include activation dynamics (dyntype=integrator). nv=1, na=1.

(e) `tendon_model.xml` — fixed + spatial tendons with spring.
    Exercises: tendon length, velocity, Jacobian, passive tendon forces.
    nv=2-3.

(f) `sensor_model.xml` — body with position/velocity/acceleration sensors.
    Exercises: sensordata evaluation at all three stages.
    nv=1-2, nsensor=6+.

(g) `equality_model.xml` — weld + joint equality constraints.
    Exercises: equality constraint assembly, solver interaction.
    nv=2-3.

(h) `composite_model.xml` — combines contacts + actuators + sensors +
    tendons. Exercises: full pipeline interaction. nv=3-6.
    Used primarily for Layer C trajectory comparison.

Keep each model minimal. Include XML comments documenting which pipeline
stages each model exercises and why it was designed this way.

**CRITICAL: Design models so relevant pipeline stages produce non-trivial
output at qpos0 (the initial state).** The per-stage reference tests
call `forward()` once without stepping — if a stage produces zeros or
trivial output at qpos0, the test is worthless.
- Contact models: sphere must be resting ON or slightly overlapping the
  plane at qpos0 (not floating above it). Use `pos="0 0 0.1"` with
  sphere radius 0.1 so contact exists at initial state.
- Tendon models: joints at non-zero qpos0 so tendons have non-zero
  length and non-zero passive force.
- Actuated models: set non-zero `ctrl` before calling `forward()` in
  the gen script (and document the ctrl value for the Rust test).
- Sensor models: use position-stage sensors that produce non-zero
  output from FK alone (e.g., framepos of a body not at origin).

**D2: Write `gen_conformance_reference.py`.**
Python script that generates per-stage reference data from MuJoCo 3.4.0:

For each canonical model:
1. Load model with `mujoco.MjModel.from_xml_path()`
2. Create data with `mujoco.MjData(model)`
3. For actuated models: set `data.ctrl[:]` to documented values
4. Call `mujoco.mj_forward(model, data)` (single forward pass)
5. Capture per-stage outputs as INDIVIDUAL `.npy` files (one field per
   file). Do NOT use `.npz` — it requires ZIP decompression in Rust,
   which is unnecessary complexity. The existing `parse_npy()` handles
   all individual `.npy` files with zero new infrastructure.

   Per-stage output files:
   - `{model}_fk_xpos.npy`: xpos (nbody×3)
   - `{model}_fk_xquat.npy`: xquat (nbody×4)
   - `{model}_fk_xipos.npy`: xipos (nbody×3)
   - `{model}_crba_qM.npy`: full mass matrix (nv×nv)
   - `{model}_rne_qfrc_bias.npy`: qfrc_bias (nv)
   - `{model}_passive_qfrc_passive.npy`: qfrc_passive (nv)
   - `{model}_actuator_qfrc_actuator.npy`: qfrc_actuator (nv)
   - `{model}_actuator_force.npy`: actuator_force (nu)
   - `{model}_sensor_sensordata.npy`: sensordata (nsensordata)
   - `{model}_tendon_length.npy`: ten_length (ntendon)
   - `{model}_tendon_velocity.npy`: ten_velocity (ntendon)
   - `{model}_contact_pos.npy`: contact positions (ncon×3)
   - `{model}_contact_normal.npy`: contact frame row 0 (ncon×3)
   - `{model}_contact_depth.npy`: contact depths (ncon)
   - `{model}_contact_geom_pairs.npy`: geom1/geom2 (ncon×2, int)
   - `{model}_constraint_efc_J.npy`: efc_J (nefc×nv)
   - `{model}_constraint_efc_b.npy`: efc_b (nefc)
   - `{model}_constraint_efc_force.npy`: efc_force (nefc)
6. For trajectory models: step N times (default 100), capture per-step:
   - `{model}_trajectory_qpos.npy`: qpos (N×nq)
   - `{model}_trajectory_qvel.npy`: qvel (N×nv)
   - `{model}_trajectory_qacc.npy`: qacc (N×nv)

Output to `assets/golden/conformance/reference/`.
Include metadata file (`reference_metadata.json`) with:
- mujoco version used, generation timestamp, model checksums
- Per-file field names and shapes for downstream test consumption
- ctrl values used for actuated models

Script requirements:
- `mujoco==3.4.0` (hard pin — assert version at script start)
- `numpy` for .npy serialization (NO .npz — individual files only)
- Idempotent (same input → same output, safe to re-run)
- Run with: `uv pip install mujoco==3.4.0 numpy && uv run sim/L0/tests/scripts/gen_conformance_reference.py`

**D3: Generate all reference data and check into repo.**
Run the script, verify outputs, commit reference data.
Write `assets/golden/conformance/README.md` with:
- What each file contains
- How to regenerate
- MuJoCo version pinning requirement
- File size inventory

Do NOT write Rust tests — that's Spec A and Spec B's job.

MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A Rubric (Layer B — Per-Stage Reference)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
3. sim/L0/tests/assets/golden/conformance/ (reference data from Session 4)
4. sim/L0/core/src/types/data.rs (Data struct — per-stage output fields)

Spec A covers §45 Layer B: per-stage reference tests. For each pipeline
stage, compare CortenForge output against MuJoCo 3.4.0 reference data
generated by Session 4.

**Pipeline stages to cover (10):**
1. FK: xpos, xquat, xipos, xmat, geom_xpos, geom_xmat, site_xpos
2. CRBA: qM (dense mass matrix, nv×nv)
3. RNE: qfrc_bias (Coriolis + centrifugal + gravity)
4. Passive: qfrc_passive (spring + damper + fluid)
5. Collision: contact positions, normals, depths, geom pair IDs
6. Constraint assembly: efc_J, efc_b, efc_pos, efc_margin
7. Constraint solver: efc_force, qacc, qfrc_constraint
8. Actuation: qfrc_actuator, actuator_force, act, act_dot
9. Sensors: sensordata (position/velocity/acceleration stage sensors)
10. Tendons: ten_length, ten_velocity, ten_force

**Key areas for the rubric to validate:**
- Per-stage tolerance selection with algorithmic justification
  (from SESSION_PLAN.md Tolerance Philosophy table)
- Collision comparison strategy: STRUCTURAL comparison of contact sets
  (same geom pairs, same depths within tolerance) rather than element-wise
  array comparison (contact enumeration order may differ)
- Test organization: one test per (model, stage) pair, or grouped?
- Reference data loading: .npy parsing (individual files, no .npz), shape validation
- Diagnostic output on failure: model name, stage, field name, index,
  expected vs actual, tolerance — enough to identify root cause
- Coverage matrix: which models × which stages = which tests
- Known-failure strategy: #[ignore] with tracking for failed comparisons

Follow the rubric workflow exactly:
- Phase 1: Read CortenForge pipeline stages and Data struct fields
- Phase 2: Build SPEC_A_RUBRIC.md
- Phase 3: Grade P1 honestly
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/
MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec A Spec (Layer B — Per-Stage Reference)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A_RUBRIC.md
4. sim/L0/tests/assets/golden/conformance/ (reference data + models)
5. sim/L0/tests/mujoco_conformance/common.rs (shared utilities from Sessions 2-3)

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section: map each pipeline stage to MuJoCo C functions
  and Data struct fields
- S1: Common infrastructure — reference data loading (reuse parse_npy
  from common.rs for individual .npy files), tolerance constant registry,
  diagnostic assertion helpers that print model/stage/field/index on failure
- S2: FK reference tests — for each canonical model with FK-relevant
  geometry: load reference xpos/xquat/xipos, run forward(), compare.
  Tolerance: 1e-12. Models: pendulum, double_pendulum, contact_scenario.
- S3: CRBA reference tests — load reference qM, run forward(), extract
  dense mass matrix, compare element-wise. Tolerance: 1e-12.
  Models: pendulum, double_pendulum.
- S4: RNE reference tests — load reference qfrc_bias, run forward(),
  compare. Tolerance: 1e-10. Models: pendulum, double_pendulum.
- S5: Passive force reference tests — load reference qfrc_passive,
  compare. Tolerance: 1e-10. Models: pendulum (with spring/damper).
- S6: Collision reference tests — STRUCTURAL comparison. Load reference
  contact data, run forward(), match contact sets by geom pair. For
  matched contacts, compare depth (1e-6) and normal (1e-6). Report
  unmatched contacts. Models: contact_scenario.
- S7: Constraint reference tests — load reference efc_J, efc_b,
  efc_force. Compare constraint Jacobian rows (structural match first,
  then element-wise). Tolerance: efc_J 1e-8, efc_force 1e-4.
  Models: contact_scenario, equality_model.
- S8: Actuation reference tests — load reference qfrc_actuator,
  actuator_force, act. Compare. Tolerance: 1e-10.
  Models: actuated_system.
- S9: Sensor reference tests — load reference sensordata, compare.
  Tolerance: 1e-8. Models: sensor_model.
- S10: Tendon reference tests — load reference ten_length, ten_velocity.
  Compare. Tolerance: 1e-10. Models: tendon_model.
- For each section: test naming convention, failure diagnostic format,
  #[ignore] policy for known gaps
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/
MuJoCo conformance is the cardinal goal.
```

---

## Session 7: Spec A Implementation (Layer B — Per-Stage Reference)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section
(S1, S2, ..., S10). Verify each AC as you go. The spec is the source
of truth — if you discover a gap, stop and update the spec first.

**This session covers 10 pipeline stages — it is the most likely session
to overflow context.** If context runs out, commit completed stages and
continue in a follow-up session with the same prompt. Priority order
(simpler stages first, harder stages last):
  FK → CRBA → RNE → passive → actuation → tendon → sensor →
  collision (structural matching) → constraint (row matching) →
  integration

All tests go into `sim/L0/tests/mujoco_conformance/layer_b.rs` (or
per-stage submodules under `mujoco_conformance/layer_b/` if the file
gets too large). Update `mujoco_conformance/mod.rs`.

**Critical implementation notes:**
- Load reference .npy files from assets/golden/conformance/reference/
  (individual files, one field per file — no .npz)
- Use the parse_npy utility from common.rs (same utility for all files)
- For collision (S6): implement set-based contact matching, not array
  element-wise comparison. Match by geom pair, then compare depth/normal.
- For constraint (S7): match constraint rows by type + associated
  body/joint/tendon before comparing Jacobian values.
- When a comparison fails: if the failure is a genuine CortenForge gap
  (not a test bug), mark the test #[ignore] with a tracking comment
  identifying the likely source phase and DT number. Document the
  failure in a comment: expected value, actual value, magnitude of gap.
- Do NOT weaken tolerances to make tests pass. The reference data is
  ground truth.

Run `cargo test -p sim-conformance-tests --test mujoco_conformance`
after each section. Passing + ignored tests are both acceptable.
MuJoCo conformance is the cardinal goal.
```

---

## Session 8: Spec A Review — Create Document

- [ ] Complete

```
Phase 12 Conformance Test Suite -- create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A.md

Copy the review template into this directory as SPEC_A_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A_REVIEW.md
```

---

## Session 9: Spec A Review — Execute

- [ ] Complete

```
Phase 12 Conformance Test Suite -- execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A.md
3. The implementation files listed in SPEC_A.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test (or is correctly #[ignore]d), check every planned test
was written, compare blast radius predictions against reality, audit
convention notes, scan for weak test implementations.

**Phase 12 specific review checks:**
- Every per-stage test loads reference data from the correct .npy file
- Tolerances match the spec (not weakened to pass)
- Collision tests use structural comparison (set matching), not array ordering
- Constraint tests match rows by type before comparing values
- Diagnostic output on failure includes: model, stage, field, index,
  expected, actual, tolerance
- #[ignore]d tests have tracking comments identifying source phase/DT
- No existing integration tests broken
- All reference .npy files are consumed by at least one test

Compile a preliminary #[ignore] inventory: list each ignored test with
the failure description and likely source phase. This feeds into
Session 15 (Gate Triage).

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) and the preliminary #[ignore]
inventory to the user when done.
```

---

## Session 10: Spec B Rubric (Layer C — Trajectory Comparison)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A_REVIEW.md
   (preliminary #[ignore] inventory — understand current conformance gaps)

Spec B covers §45 Layer C: trajectory comparison tests. Multi-step
simulation comparison with per-step per-field diagnostics to isolate
which stage first diverges between CortenForge and MuJoCo 3.4.0.

**Layer C test categories:**
1. Non-contact trajectory — pendulum, double pendulum. Should match
   tightly (no chaotic contact switching). 100+ steps.
2. Contact trajectory — sphere on plane, objects colliding. Contact
   events introduce divergence sensitivity. 50-100 steps with wider
   tolerance + faster growth.
3. Actuated trajectory — motor-driven system with ctrl input sequence.
   Tests actuator→dynamics→integration chain. 100 steps.
4. Composite trajectory — full-pipeline model (contacts + actuators +
   sensors + tendons). The integrative test. 100 steps.

**Key areas for the rubric to validate:**
- Chaos-aware tolerance strategy: step-dependent tolerance growth.
  Non-contact: `base_tol * (1 + step * 0.01)`. Contact: `base_tol *
  (1 + step * 0.1)` or even exponential. The rubric should validate
  that tolerance growth is justified, not just "make tests pass."
- Divergence diagnostic quality: when trajectory diverges, identify:
  (a) first step where any field exceeds tolerance
  (b) which field (qpos, qvel, qacc, sensordata)
  (c) which DOF/sensor index
  (d) magnitude of divergence
  This diagnostic maps trajectory failures back to per-stage root causes.
- Step-limited comparison: define maximum step count per model based on
  expected Lyapunov divergence rate. Don't compare 1000 steps of a
  chaotic contact system — it won't match and the diagnostic is useless.
- Per-step field comparison: qpos, qvel, qacc at minimum. sensordata
  and energy as secondary fields. Contact count as a structural check
  (same number of contacts at each step ± tolerance).
- Reference data format: individual trajectory .npy files from Session 4's
  Python script (e.g., `{model}_trajectory_qpos.npy` shape N×nq,
  `{model}_trajectory_qacc.npy` shape N×nv). No .npz — same parse_npy().
- Ctrl input sequences: for actuated models, the ctrl input at each
  step must be identical (zero, constant, or scripted). Document the
  ctrl sequence in the test and in the Python gen script.

Follow the rubric workflow exactly:
- Phase 1: Study trajectory comparison challenges (chaos, contact switching)
- Phase 2: Build SPEC_B_RUBRIC.md
- Phase 3: Grade P1 honestly
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/
MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec B Spec (Layer C — Trajectory Comparison)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B_RUBRIC.md
4. sim/L0/tests/assets/golden/conformance/ (trajectory reference data)
5. sim/L0/tests/mujoco_conformance/common.rs (shared utilities)
6. sim/L0/tests/mujoco_conformance/layer_b.rs (Layer B infrastructure to reuse)

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section: trajectory comparison methodology,
  MuJoCo C pipeline stage ordering, expected sources of divergence
- S1: Trajectory comparison infrastructure — `TrajectoryComparison`
  struct/helper that:
  - Loads reference trajectory .npy files (qpos, qvel, qacc per step)
  - Steps CortenForge the same number of times
  - Compares per-step with step-aware tolerance
  - On first divergence: captures diagnostic (step, field, DOF, expected,
    actual, magnitude) and either panics (for clean tests) or continues
    (for full diagnostic sweep)
  - Reports summary: total steps matched, first divergent step, worst-case
    field, accumulated max error
- S2: Non-contact trajectory tests — pendulum, double_pendulum.
  100 steps, tight tolerance (base 1e-8, growth 0.01 per step).
  These should match very well — divergence here indicates a
  fundamental FK/dynamics/integration bug.
- S3: Contact trajectory tests — contact_scenario. 50-100 steps,
  wider tolerance (base 1e-6, growth 0.05 per step). Contact events
  introduce divergence sensitivity. Compare step count where contacts
  activate (structural check). Document expected divergence behavior.
- S4: Actuated trajectory tests — actuated_system. 100 steps with
  scripted ctrl sequence. Tests the ctrl → activation → force →
  dynamics → integration chain. Tolerance: base 1e-8, growth 0.02
  per step. IMPORTANT: ctrl sequences must be exactly reproducible
  cross-language — use constants (0.0, 1.0, -0.5), step functions,
  or integer-valued sequences only. Do NOT use transcendental
  functions (sin, cos, exp) — Python and Rust produce subtly
  different results, causing phantom divergence.
- S5: Composite trajectory tests — composite_model (full pipeline).
  100 steps. Most integrative test. Tolerance tuned per model.
  This is the "does the whole thing work" test.
- S6: Diagnostic reporting — structured output on failure that maps
  trajectory divergence back to pipeline stages. Use Layer B's
  per-stage comparison as a secondary diagnostic: if trajectory diverges
  at step N, run per-stage comparison at step N's state to identify
  which stage caused the divergence.
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/
MuJoCo conformance is the cardinal goal.
```

---

## Session 12: Spec B Implementation (Layer C — Trajectory Comparison)

- [ ] Complete

```
Phase 12 Conformance Test Suite -- implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section
(S1, S2, ..., S6). Verify each AC as you go.

All tests go into `sim/L0/tests/mujoco_conformance/layer_c.rs`.
Update `mujoco_conformance/mod.rs`.

**Critical implementation notes:**
- Reuse parse_npy from common.rs (all reference data is individual .npy files)
- Reuse tolerance constants from common.rs
- The trajectory comparison infrastructure (S1) should be a reusable
  function/struct, not duplicated per test
- For contact trajectory tests (S3): contact event timing differences
  are expected. A contact activating at step 47 vs step 48 causes
  cascade divergence — the test should detect this and report "contact
  event timing mismatch at step 47" rather than just "qacc mismatch."
- For actuated tests (S4): ctrl inputs must match EXACTLY between
  the Python gen script and the Rust test. Define ctrl sequences as
  constants shared between gen script documentation and test code.
- When trajectory diverges: #[ignore] the test with a tracking comment
  noting the first divergent step and likely cause. Do NOT reduce step
  count or widen tolerance to make it pass.
- Run Layer B per-stage comparison at the divergent step's state as a
  diagnostic — this cross-references Layer C failures with Layer B data.

Run `cargo test -p sim-conformance-tests --test mujoco_conformance`
after each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 13: Spec B Review — Create Document

- [ ] Complete

```
Phase 12 Conformance Test Suite -- create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B.md

Copy the review template into this directory as SPEC_B_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B_REVIEW.md
```

---

## Session 14: Spec B Review — Execute

- [ ] Complete

```
Phase 12 Conformance Test Suite -- execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B.md
3. The implementation files listed in SPEC_B.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test (or is correctly #[ignore]d), check every planned test
was written, compare blast radius predictions against reality, audit
convention notes, scan for weak test implementations.

**Phase 12 specific review checks:**
- Trajectory comparison infrastructure is reusable (not copy-pasted per test)
- Step-aware tolerances are justified and documented (not arbitrarily widened)
- Contact trajectory tests detect contact event timing mismatches
- Ctrl sequences match between Python gen script and Rust tests
- Diagnostic output identifies first divergent step/field/DOF
- #[ignore]d tests have tracking comments with first divergent step
  and likely source phase/DT
- No existing integration tests broken

Compile the Spec B #[ignore] inventory and merge with Spec A's
preliminary inventory from Session 9.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) and the merged #[ignore]
inventory to the user when done.
```

---

## Session 15: Gate Triage

- [ ] Complete

```
Phase 12 Conformance Test Suite -- gate triage.

Read these:
1. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/PHASE12_UMBRELLA.md
   (Phase-Level Acceptance Criteria)
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_A_REVIEW.md
   (Spec A #[ignore] inventory)
3. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/SPEC_B_REVIEW.md
   (Spec B #[ignore] inventory)
4. sim/docs/ROADMAP_V1.md (Phases 1-11 — for failure assignment)
5. All mujoco_conformance/ test files (scan for #[ignore] annotations)

**This is the final Phase 12 session.** It does not write code — it
produces the Gate Assessment document that drives the fix iteration.

**D1: Verify Phase-Level Acceptance Criteria.**
Check each PH12-AC against reality:
- PH12-AC1: All 25 flag golden-file tests pass or skip gracefully
- PH12-AC2: Layer A self-consistency tests pass without MuJoCo
- PH12-AC3: Layer D invariant tests pass without MuJoCo
- PH12-AC4: Layer B per-stage reference tests exist for all 10+ stages
- PH12-AC5: Layer C trajectory tests cover contact and non-contact
- PH12-AC6: `cargo test -p sim-conformance-tests` passes (ignoring known gaps)
- PH12-AC7: No existing integration tests broken or moved
- PH12-AC8: Gate triage inventory compiled with phase assignments

**D2: Compile complete #[ignore] inventory.**
Scan all mujoco_conformance/ files for #[ignore] annotations. For each:
- Test name and location
- Layer (A/B/C/D)
- Failure description (what diverges, by how much)
- First divergent step (for Layer C tests)
- Pipeline stage where divergence originates
- Likely source phase (1-11) and DT number if applicable
- Severity: CRITICAL (FK/dynamics wrong), HIGH (constraint/solver gap),
  MEDIUM (sensor/tendon secondary), LOW (edge case)

**D3: Prioritize fix backlog.**
Order the #[ignore] inventory by:
1. Pipeline stage order: FK → CRBA → RNE → passive → collision →
   constraint → actuation → sensor → tendon → integration
   (upstream fixes may resolve downstream failures)
2. Severity within stage
3. Number of downstream tests affected

**D4: Write GATE_ASSESSMENT.md.**
- Summary: N total conformance tests, M passing, K ignored (failing)
- Per-layer breakdown: Layer A (N/M pass), Layer B, Layer C, Layer D
- Prioritized fix backlog with phase assignments
- Estimated fix effort per item (S/M/L based on Phase 8-11 precedents)
- Recommended fix iteration plan: which items to fix first, expected
  cascade effects (fixing FK may resolve 5 downstream failures)

Write to: sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/GATE_ASSESSMENT.md

Present the Gate Assessment to the user when done. This document defines
what stands between CortenForge and v1.0.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 12 Umbrella | Done | fdb514e |
| 2 | T1: DT-97 (golden flag expansion + model enrichment) | Done | 0fbdb7c |
| 3 | T1: Layer A+D (coverage audit + systematic gap-fill) | Done | 30972f3 |
| 4 | T1: Reference data gen infrastructure (models + Python script) | | |
| 5 | Spec A rubric (Layer B — per-stage reference) | | |
| 6 | Spec A spec | | |
| 7 | Spec A implementation | | |
| 8 | Spec A review — create document | | |
| 9 | Spec A review — execute + preliminary #[ignore] inventory | | |
| 10 | Spec B rubric (Layer C — trajectory comparison) | | |
| 11 | Spec B spec | | |
| 12 | Spec B implementation | | |
| 13 | Spec B review — create document | | |
| 14 | Spec B review — execute + merged #[ignore] inventory | | |
| 15 | Gate Triage — GATE_ASSESSMENT.md | | |
