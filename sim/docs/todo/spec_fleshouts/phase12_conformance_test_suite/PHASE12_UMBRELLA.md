# Phase 12 — Conformance Test Suite: Umbrella Spec

**Status:** Stress-tested
**Phase:** Roadmap Phase 12
**Tasks:** 2 (§45, DT-97)
**Deliverables:** 3 T1 sessions + 2 sub-specs (Spec A, Spec B)
**Test baseline:** 3,600+ domain tests (post-Phase 11)

---

## Scope

Phase 12 is the gate phase. It validates CortenForge against MuJoCo 3.4.0 at
every pipeline stage. **The goal is MuJoCo conformance** — after Phase 12,
CortenForge has a four-layer conformance test suite (§45) that systematically
compares every pipeline stage output against MuJoCo reference data, and
golden-file coverage for all 25 disable/enable flags (DT-97). Failures found
here drive fix iterations back into Phases 1–11 until green.

Phase 12 writes tests, not production code. It produces no new simulation
features — it validates the features built in Phases 1–11.

Phase 12 builds on substantial existing test infrastructure (57 integration
test modules, 1,216+ tests):

| Existing (Phases 1–11) | Phase 12 adds |
|------------------------|---------------|
| 57 integration test modules across all pipeline stages | Systematic per-stage MuJoCo reference comparison (Layer B) |
| `golden_flags.rs` — 1 flag golden-file test (DISABLE_GRAVITY) | All 25 flags with enriched canonical model (DT-97) |
| `gen_flag_golden.py` — generates 2 `.npy` files | `gen_conformance_reference.py` — generates all reference data |
| `validation.rs` — 9 FK/CRBA/RNE analytical tests | Layer B: MuJoCo-verified expected values at every stage |
| `derivatives.rs` — 149+ FD conformance tests | Trajectory-level validation with divergence diagnostics (Layer C) |
| `spatial_tendons.rs` — 18 MuJoCo reference tests | Complete tendon stage reference comparison |
| `inverse_dynamics.rs` — forward/inverse round-trip | Layer A: systematic self-consistency audit + gap-fill |
| `rk4_integration.rs` — energy conservation | Layer D: systematic property/invariant audit + gap-fill |
| `collision_test_utils.rs` — tolerance hierarchy + assertion macros | Structural contact comparison (set-based, order-independent) |
| Ad-hoc per-feature regression tests | Organized four-layer conformance binary |

**Relationship between integration tests and conformance tests:**
Integration tests are per-feature regression tests — they validate specific
features as they are built. Conformance tests are systematic MuJoCo-matching
validation — they compare every pipeline stage against reference data from
MuJoCo 3.4.0. Intentional overlap is expected and acceptable. Phase 12 does
not move, modify, or break any existing integration tests.

**Two test binaries (separate `[[test]]` targets in Cargo.toml):**
- `integration/` — existing 57 modules, untouched by Phase 12
- `mujoco_conformance/` — populated by Phase 12 (Layers A, B, C, D)

These are separate compilation units. Code CANNOT be shared across test
binaries via module imports. Shared utilities (e.g., `parse_npy()`, tolerance
constants) must be duplicated — ~50 lines of intentional duplication per
binary is the correct approach. Do NOT attempt to create a shared test
utility crate or extract common code across binaries.

> **Conformance mandate for all sessions:** Every conformance test must use
> expected values derived from MuJoCo's actual output — not from hand-
> calculation or intuition. The MuJoCo C source code is the single source of
> truth. When the MuJoCo docs and the C source disagree, the C source wins.

---

## Task Assignment

Every Phase 12 task is assigned to exactly one deliverable. No orphans, no
overlaps.

| Task | Description | Deliverable | Status | Rationale |
|------|-------------|-------------|--------|-----------|
| DT-97 | Golden file expansion to all 25 disable/enable flags | **T1** (Session 2) | Pending | Extend `gen_flag_golden.py` to all 25 flags + enrich canonical model to exercise every flag. Established pattern from §41 AC18. |
| §45 Layer A+D | Self-consistency + property/invariant tests | **T1** (Session 3) | Pending | Audit existing 1,216 tests, write systematic conformance versions for gaps. Leverages existing coverage — not worth a full spec cycle. |
| §45 Ref Gen | Reference data generation infrastructure | **T1** (Session 4) | Pending | Canonical conformance models + Python gen script + `.npy` reference files. Infrastructure that Spec A and Spec B depend on. |
| §45 Layer B | Per-stage MuJoCo reference comparison | **Spec A** (Sessions 5–9) | Pending | Systematic comparison at each pipeline stage — the heaviest deliverable. Full rubric → spec → implement → review cycle. |
| §45 Layer C | Trajectory comparison + divergence diagnostics | **Spec B** (Sessions 10–14) | Pending | Multi-step trajectory validation with per-step per-field diagnostics. Distinct challenges from Layer B. |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo pipeline stages it validates and
produce acceptance criteria with MuJoCo-verified expected values.

**Spec A — Per-Stage Reference Comparison** (§45 Layer B):
Systematic comparison of each pipeline stage output against MuJoCo 3.4.0
reference data generated by `gen_conformance_reference.py`. Covers 10+
pipeline stages: FK (`xpos`, `xquat`, `xmat`), CRBA (`M`), RNE
(`qfrc_bias`), passive forces (`qfrc_passive`), collision (contact
enumeration, depths, normals), constraint solving (`efc_force`), actuation
(`qfrc_actuator`), sensors (`sensordata`), tendons (`ten_length`,
`ten_velocity`), and integration (`qpos`, `qvel` after single step).

Each stage test loads a canonical conformance model, loads the corresponding
MuJoCo reference `.npy` file, runs CortenForge's pipeline to that stage, and
compares at algorithm-appropriate tolerances. Tests that fail due to known
CortenForge gaps are marked `#[ignore]` with tracking comments.

Primary files: `mujoco_conformance/layer_b.rs` (or per-stage submodules),
`assets/golden/conformance/` reference data.

**Spec B — Trajectory Comparison** (§45 Layer C):
Multi-step trajectory comparison with per-step per-field diagnostics to
isolate which pipeline stage first diverges from MuJoCo. Covers contact and
non-contact scenarios with step-aware growing tolerances.

Trajectory tests load a canonical model, load reference trajectory `.npy`
files (qpos/qvel/qacc per step for N steps), run CortenForge for N steps, and
compare at each step with tolerances that grow as `tol(step) = base_tol *
(1 + step * growth_rate)`. When a divergence is detected, diagnostics report
which field and step first exceeded tolerance. Contact scenarios use wider
base tolerance and faster growth due to chaotic sensitivity.

Primary files: `mujoco_conformance/layer_c.rs`,
`assets/golden/conformance/` trajectory reference data.

### T1 scope

**Session 2 — DT-97 (Golden Flag Expansion):**
Enrich the canonical model (`flag_golden_test.xml`) so every flag has
observable effect, extend `gen_flag_golden.py` to generate `.npy` files for
all 25 flags, and extend `golden_flags.rs` with one test per flag. Each test
compares CortenForge's output against the golden file at `1e-8` tolerance.
Flags whose pipeline stages are not yet implemented get `#[ignore]` tests.

**Session 3 — Layers A+D (Coverage Audit + Gap-Fill):**
Audit existing integration tests against Layer A (self-consistency) and
Layer D (property/invariant) requirements from §45. Identify systematic
gaps. Write conformance versions in `mujoco_conformance/layer_a.rs` and
`mujoco_conformance/layer_d.rs`. Expected gap-fills: sparse/dense mass
matrix equivalence, island/monolithic solver equivalence, integrator energy
ordering, contact force feasibility, mass matrix positive definiteness.

**Session 4 — Reference Data Infrastructure:**
Build the complete reference data generation pipeline:
- Design and implement canonical conformance models (see Test Model Registry)
- Write `gen_conformance_reference.py` pinned to `mujoco==3.4.0`
- Generate and check in all reference `.npy` files
- Document regeneration workflow

This is a hard dependency for Spec A — Layer B tests cannot exist without
reference data to compare against.

---

## Existing Infrastructure Summary

Phase 12 builds on, but does not modify, substantial test infrastructure
from Phases 1–11.

### Integration test suite (57 modules, 1,216+ tests)

Organized in `sim/L0/tests/integration/mod.rs`. Key modules relevant to
Phase 12:

| Module | Tests | Relevance |
|--------|-------|-----------|
| `validation.rs` | 9 | FK/CRBA/RNE analytical validation — Layer B overlap |
| `derivatives.rs` | 149+ | FD conformance tests — Layer A/B overlap |
| `golden_flags.rs` | 1 | Flag golden-file test — DT-97 foundation |
| `spatial_tendons.rs` | 18 | MuJoCo reference tendon tests — Layer B overlap |
| `inverse_dynamics.rs` | 2 | Forward/inverse round-trip — Layer A overlap |
| `rk4_integration.rs` | 4+ | Energy conservation, convergence order — Layer D overlap |
| `collision_*.rs` | 50+ | Collision detection tests — Layer B/D overlap |
| `sleeping.rs` | 20+ | Sleep/wake tests — relevant to ENABLE_SLEEP flag |
| `flex_*.rs` | 20+ | Flex body tests — Layer B overlap if Phase 10 complete |
| `sensors.rs`, `sensor_phase6*.rs` | 30+ | Sensor evaluation tests — Layer B overlap |
| `collision_test_utils.rs` | — | Tolerance hierarchy + assertion macros — Layer B/D utilities |

### Golden-file infrastructure

- `gen_flag_golden.py` — Python script generating `.npy` golden files from
  MuJoCo 3.4.0. Currently generates 2 files (baseline + DISABLE_GRAVITY).
- `golden_flags.rs` — Rust test loading `.npy` and comparing against
  CortenForge output. Includes `parse_npy()` for NumPy v1.0 format.
- `assets/golden/flags/` — golden data directory with `flag_golden_test.xml`
  canonical model.

### What Phase 12 adds on top

The existing 1,216 tests are **per-feature regression tests** — each validates
a specific feature in isolation. Phase 12 adds:

1. **Systematic per-stage comparison** — not ad-hoc. Every pipeline stage
   gets a dedicated test comparing against MuJoCo reference data.
2. **Trajectory-level validation** — multi-step comparison with divergence
   diagnostics. No existing test does this.
3. **Complete golden-file coverage** — all 25 flags, not just 1.
4. **Organized conformance binary** — `mujoco_conformance/` is a separate
   test target with four layers, not scattered across 57 feature modules.
5. **Known-failure tracking** — `#[ignore]` strategy for systematic gap
   inventory, enabling iterative fix cycles.

---

## Known-Failure Strategy

Phase 12 tests will discover conformance failures — that's the point.
Strategy for tests that fail due to known CortenForge gaps:

1. **Write the test with the correct MuJoCo-matching assertion.** Never
   weaken tolerances or assertions to make a test pass. The test encodes
   the ground truth.
2. **Mark with `#[ignore]` and a tracking comment:**
   ```rust
   #[test]
   #[ignore] // CONFORMANCE GAP: <description> — see DT-XXX or Phase N
   fn layer_b_constraint_efc_force() {
       // ... correct test with correct assertion
   }
   ```
3. **`cargo test -p sim-conformance-tests` passes** — ignored tests skip
   by default, so CI stays green.
4. **`cargo test -p sim-conformance-tests -- --ignored` runs known-failing
   tests** for monitoring progress during fix iterations.
5. **As fixes land in Phases 1–11**, un-ignore the test and verify it passes.
6. **Session 15 (Gate Triage)** compiles the full `#[ignore]` inventory
   with phase assignments and priority ordering.

Gate passes when `cargo test -p sim-conformance-tests -- --ignored` has
zero failures (all tests un-ignored and passing).

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

| From → To | Type | Specific dependency |
|-----------|------|---------------------|
| Session 4 (Ref Gen) → Spec A (Layer B) | **Hard** | Spec A's per-stage tests load reference `.npy` files generated by Session 4's Python script. Without reference data, Layer B tests have nothing to compare against. |
| DT-97 (Session 2) → Spec B (Layer C) | **Soft** | DT-97 establishes the `.npy` comparison pattern and `parse_npy()` utility. Spec B reuses this for trajectory golden files. Spec B can build its own if needed. |
| Spec A (Layer B) → Spec B (Layer C) | **Soft** | Spec B can reuse Spec A's per-stage comparison infrastructure, tolerance constants, and diagnostic patterns. Spec B can stand alone if needed. |
| All sessions → Session 15 (Gate) | **Hard** | Gate triage requires all tests to be written (even if `#[ignore]`d) to compile the failure inventory. |

### Parallelism

Sessions 2, 3, and 4 are fully independent and can proceed in any order (or
parallel). Spec A must wait for Session 4. Spec B ideally follows Spec A but
is not blocked. Session 15 must be last.

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/tests/mujoco_conformance/mod.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (Session 2) | Replace placeholder, add `mod common;` declaration | Common module registered |
| 2 | T1 (Session 3) | Add `mod layer_a;` and `mod layer_d;` declarations | Layer A+D modules registered |
| 3 | Spec A | Add `mod layer_b;` (or per-stage submodule declarations) | Layer B modules registered |
| 4 | Spec B | Add `mod layer_c;` declaration | All four layers registered |

**Conflict risk: None.** Additive `mod` declarations. Each deliverable adds
its own module without touching others.

### `sim/L0/tests/mujoco_conformance/common.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (Session 2) | Create with `parse_npy()` (duplicated from golden_flags.rs, not shared — separate test binary), tolerance constants | Basic shared utilities for conformance binary |
| 2 | T1 (Session 3) | Add model loading helpers, Layer A/D assertion utilities | Self-consistency/invariant test infra available |
| 3 | Spec A | Add per-stage comparison helpers, structural contact comparison, reference data path resolution | Layer B comparison infra available |
| 4 | Spec B | Add trajectory comparison helpers, step-aware tolerance computation, divergence diagnostic reporting | Layer C comparison infra available |

**Conflict risk: Low.** Each deliverable adds to the shared module. Session 2
establishes the foundation; subsequent sessions extend it with their
respective comparison patterns.

### `sim/L0/tests/assets/golden/conformance/`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (Session 4) | Create `models/` with canonical XMLs, create `reference/` with per-stage + trajectory `.npy` files, create `reference_metadata.json` | All reference data for Layers B+C available |

**Conflict risk: None.** Session 4 generates all reference data in one pass.
Spec A and Spec B only read from this directory.

### Per-deliverable owned files (single-owner)

| Deliverable | Owned files |
|-------------|------------|
| T1 (Session 2) | `integration/golden_flags.rs` (extend), `scripts/gen_flag_golden.py` (extend), `assets/golden/flags/*.npy` (add 23 files), `assets/golden/flags/flag_golden_test.xml` (enrich) |
| T1 (Session 3) | `mujoco_conformance/layer_a.rs`, `mujoco_conformance/layer_d.rs` |
| T1 (Session 4) | `scripts/gen_conformance_reference.py`, `assets/golden/conformance/models/*.xml` (canonical models), `assets/golden/conformance/reference/*.npy` (reference data), `assets/golden/conformance/reference_metadata.json` |
| Spec A | `mujoco_conformance/layer_b.rs` (or per-stage submodules) |
| Spec B | `mujoco_conformance/layer_c.rs` |

---

## Test Model Registry

Canonical conformance models — purpose-built, small, deterministic. Each
model exercises specific pipeline stages and produces non-trivial output at
`qpos0` (contacts exist at initial state, tendons non-zero, etc.).

### DT-97 canonical model

**`flag_golden_test.xml`** (existing, to be enriched in Session 2):
Currently a minimal model (1 DOF, spring/damper/motor/floor). Must be
enriched to exercise every disable/enable flag. Session 2 adds:
- Equality constraint (for DISABLE_EQUALITY)
- Joint limit (for DISABLE_LIMIT)
- Tendon with friction (for DISABLE_FRICTIONLOSS)
- Multiple bodies for collision filtering (for DISABLE_FILTERPARENT)
- Sensor (for DISABLE_SENSOR)
- Actuator with clamped ctrl (for DISABLE_CLAMPCTRL)
- Energy tracking (for ENABLE_ENERGY)
- Forward/inverse comparison (for ENABLE_FWDINV)

All additions must keep the model small and deterministic.

### Conformance canonical models

Session 4 designs and implements these. Selection criteria: exercises specific
pipeline stages, deterministic, small enough for fast CI, complex enough to
catch real bugs, non-trivial output at `qpos0`.

| ID | Model | Pipeline stages exercised | Key properties |
|----|-------|--------------------------|----------------|
| (a) | Pendulum | FK, CRBA, RNE, passive (spring/damper), integration | Single hinge joint, no contacts. Conservative system for energy tests. |
| (b) | Double pendulum | FK chain, CRBA (off-diagonal M), RNE (Coriolis) | 2-DOF chain. Non-trivial mass matrix. Chaotic for trajectory divergence. |
| (c) | Contact scenario | FK, collision, constraint solving | Sphere on plane + wall contact. Contacts exist at qpos0. Tests collision enumeration + solver. |
| (d) | Actuated system | Actuation, FK, integration | Motor + position servo. Non-zero ctrl at initial state. Tests actuator force computation. |
| (e) | Tendon model | Tendons (spatial + fixed), passive tendon forces | Spatial tendon with wrapping + fixed tendon. Non-zero tendon length at qpos0. |
| (f) | Sensor-rich model | All sensor stages (position/velocity/acceleration) | Position, velocity, acceleration, force sensors. Tests sensordata computation. |
| (g) | Flex body | Flex pipeline (if Phase 10 complete) | Deformable body with flex elements. Tests flex-specific pipeline stages. |
| (h) | Equality constraints | Equality constraint assembly + solving | Weld + joint + tendon equality constraints. Tests constraint formulation. |
| (i) | Composite model | Full pipeline interaction | Contacts + actuators + sensors + tendons combined. Primary Layer C trajectory test model. nv=3–6. |

Models are checked into `assets/golden/conformance/models/` as MJCF XML files.

---

## Reference Data Strategy

### Generation infrastructure

- **Script:** `gen_conformance_reference.py` — single Python script generating
  all conformance reference data.
- **MuJoCo version:** Pinned to `mujoco==3.4.0` (hard requirement). Script
  verifies version at startup and aborts on mismatch.
- **Python environment:** Uses `uv` exclusively. Documented installation:
  ```
  uv pip install mujoco==3.4.0 numpy
  uv run sim/L0/tests/scripts/gen_conformance_reference.py
  ```
- **Output format:** Individual `.npy` files ONLY (one field per file). NO
  `.npz` archives. This avoids ZIP parsing in Rust — the existing `parse_npy()`
  utility in `golden_flags.rs` handles all reference data with zero new
  parsing infrastructure.
- **Storage:** Checked into repo under `assets/golden/conformance/`. No MuJoCo
  build dependency for Rust tests.
- **Regeneration:** Documented, reproducible, idempotent. Running the script
  twice produces bit-identical output.
- **Metadata:** `reference_metadata.json` checked in alongside reference data.
  Contains: MuJoCo version used, generation timestamp, per-model checksums,
  per-file field names and shapes, ctrl values used for actuated models.
  Downstream Rust tests can optionally load this for shape validation.

### Per-stage reference data (Layer B)

Single `forward()` call on each canonical model. Capture intermediate fields
at each pipeline stage:

| Stage | Fields captured | File naming |
|-------|----------------|-------------|
| FK | `xpos`, `xquat`, `xipos` | `{model}_fk_xpos.npy`, `{model}_fk_xquat.npy`, `{model}_fk_xipos.npy` |
| CRBA | `qM` (mass matrix, nv×nv) | `{model}_crba_qM.npy` |
| RNE | `qfrc_bias` (nv) | `{model}_rne_qfrc_bias.npy` |
| Passive | `qfrc_passive` (nv) | `{model}_passive_qfrc_passive.npy` |
| Collision | geom pairs, depths, normals, positions | `{model}_contact_geom_pairs.npy`, `{model}_contact_depth.npy`, `{model}_contact_normal.npy`, `{model}_contact_pos.npy` |
| Constraint | `efc_J`, `efc_b`, `efc_force` | `{model}_constraint_efc_J.npy`, `{model}_constraint_efc_b.npy`, `{model}_constraint_efc_force.npy` |
| Actuation | `qfrc_actuator`, `actuator_force` | `{model}_actuator_qfrc_actuator.npy`, `{model}_actuator_force.npy` |
| Sensors | `sensordata` | `{model}_sensor_sensordata.npy` |
| Tendons | `ten_length`, `ten_velocity` | `{model}_tendon_length.npy`, `{model}_tendon_velocity.npy` |
| Integration | `qpos`, `qvel` (after 1 step) | `{model}_integration_qpos.npy`, `{model}_integration_qvel.npy` |

All per-stage files go to `assets/golden/conformance/reference/`.

### Trajectory reference data (Layer C)

N steps on each canonical model. Capture per-step state:

| Field | Shape | File naming |
|-------|-------|-------------|
| `qpos` | `(N, nq)` | `{model}_trajectory_qpos.npy` |
| `qvel` | `(N, nv)` | `{model}_trajectory_qvel.npy` |
| `qacc` | `(N, nv)` | `{model}_trajectory_qacc.npy` |

All trajectory files go to `assets/golden/conformance/reference/`.

Step count `N` is model-dependent — chosen to be long enough to reveal
divergence but short enough for fast CI (typically 50–200 steps).

---

## Tolerance Strategy

Per-stage tolerances based on algorithmic properties. These are starting
points — the rubric and spec sessions refine them against actual MuJoCo C
source analysis.

| Stage | Tolerance | Rationale |
|-------|-----------|-----------|
| FK (`xpos`, `xquat`, `xmat`) | `1e-12` | Exact arithmetic modulo FP accumulation through kinematic tree |
| CRBA (mass matrix `M`) | `1e-12` | Same — CRBA is exact given FK |
| RNE (`qfrc_bias`) | `1e-10` | Accumulated FP in tree traversal (gravity, Coriolis) |
| Passive (`qfrc_passive`) | `1e-10` | Spring/damper forces — direct computation |
| Collision (contacts) | STRUCTURAL | Contact enumeration order may differ; compare sets, not arrays. Depths/normals: `1e-6` |
| Constraint (`efc_force`) | `1e-4` | Iterative solver convergence — different paths, same basin |
| Actuation (`qfrc_actuator`) | `1e-10` | Direct computation from ctrl/act |
| Sensors (`sensordata`) | `1e-8` | Depends on upstream stage tolerance |
| Tendons (`ten_length`) | `1e-10` | Geometric computation |
| Integration (`qpos`, `qvel`) | `1e-8` (single step) | Accumulates across steps — see Layer C |
| Trajectory (Layer C) | Growing | `tol(step) = base_tol * (1 + step * growth_rate)`. Non-contact: tighter base. Contact: wider base + faster growth (chaotic sensitivity). |

### Collision comparison

Collision contacts require structural comparison, not element-wise array
comparison. Contact pairs may be enumerated in different order by CortenForge
vs MuJoCo. The comparison strategy:
1. Sort contacts by geom pair IDs (canonical ordering)
2. Match contacts by geom pair
3. Compare depths, normals, and positions per matched contact
4. Report unmatched contacts as failures

---

## Iteration Protocol

Phase 12 is designed to be iterative:

1. **Write tests** (Sessions 2–14): Write correct tests, mark failures with
   `#[ignore]`.
2. **Gate triage** (Session 15): Compile `#[ignore]` inventory, assign each
   failure to the responsible phase/DT, prioritize by pipeline order (FK →
   CRBA → RNE → passive → collision → constraint → actuation → sensors →
   tendons → integration).
3. **Fix iteration** (separate sessions, not Phase 12): Fix the root cause in
   the responsible phase. Each fix session ends with un-ignoring the
   conformance test.
4. **Re-run**: After each fix batch, run
   `cargo test -p sim-conformance-tests -- --ignored` to check remaining
   failures.
5. **Gate passes** when `cargo test -p sim-conformance-tests -- --ignored` has
   zero failures (all tests un-ignored and passing).

Pipeline ordering ensures downstream fixes don't mask upstream bugs — fix FK
before CRBA before RNE before dynamics.

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 12 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge has a comprehensive conformance test suite that
systematically validates every pipeline stage against MuJoCo 3.4.0.**

### PH12-AC1: All 25 flag golden-file tests exist
Every disable/enable flag has a test in `golden_flags.rs`. Tests either pass
or are marked `#[ignore]` with a tracking comment. The enriched canonical
model exercises every flag.

### PH12-AC2: Layer A self-consistency tests pass
Self-consistency tests (forward/inverse equivalence, island/monolithic solver
equivalence, determinism, integrator energy ordering, sparse/dense mass matrix
equivalence) pass without MuJoCo dependency.

### PH12-AC3: Layer D invariant tests pass
Property/invariant tests (momentum conservation, energy conservation,
quaternion normalization, contact force feasibility, mass matrix positive
definiteness) pass without MuJoCo dependency.

### PH12-AC4: Layer B per-stage reference tests exist for all stages
Per-stage reference comparison tests exist for all 10+ pipeline stages (FK,
CRBA, RNE, passive, collision, constraint, actuation, sensors, tendons,
integration). Each test loads a reference `.npy` file from MuJoCo 3.4.0 and
compares at algorithm-appropriate tolerances.

### PH12-AC5: Layer C trajectory tests cover contact and non-contact
Trajectory comparison tests exist for at least one contact scenario and one
non-contact scenario. Tests use step-aware growing tolerances and report
which field and step first diverges.

### PH12-AC6: `cargo test -p sim-conformance-tests` passes
All non-ignored conformance tests pass. CI stays green. Known gaps are
`#[ignore]`d with tracking comments, not weakened assertions.

### PH12-AC7: No existing integration tests broken or moved
All 57 existing integration test modules pass unchanged. Phase 12 adds to the
test suite — it does not modify or reorganize existing tests.

### PH12-AC8: Gate triage inventory compiled
Session 15 produces `GATE_ASSESSMENT.md` with:
- Complete `#[ignore]` inventory
- Each failure assigned to a responsible phase/DT
- Priority ordering by pipeline stage
- Fix effort estimates

---

## Shared Convention Registry

Conventions decided once here. All sessions reference this section instead of
inventing their own.

### 1. Reference data file format

All reference data uses NumPy v1.0 `.npy` format:
- Magic: `\x93NUMPY`
- Version: `1.0`
- Dtype: `<f8` (little-endian float64)
- Order: C-contiguous
- Individual files only — **never `.npz` archives**

Parsed in Rust by `parse_npy()`. The existing implementation in
`golden_flags.rs` (integration binary) stays in place. A duplicate
implementation is created in `mujoco_conformance/common.rs` (conformance
binary) by Session 2. ~50 lines of intentional duplication — the two test
binaries cannot share code via module imports.

### 2. Golden file path convention

| Data type | Base directory | Path pattern |
|-----------|---------------|-------------|
| Flag golden files | `assets/golden/flags/` | `{flag_name}_qacc.npy` |
| Per-stage reference | `assets/golden/conformance/reference/` | `{model}_{stage}_{field}.npy` |
| Trajectory reference | `assets/golden/conformance/reference/` | `{model}_trajectory_{field}.npy` |
| Canonical models | `assets/golden/conformance/models/` | `{model}.xml` |
| Metadata | `assets/golden/conformance/` | `reference_metadata.json` |

### 3. Test naming convention

```rust
// Layer A: self-consistency
fn layer_a_{property}() { ... }
// e.g., layer_a_forward_inverse_roundtrip, layer_a_determinism

// Layer B: per-stage reference
fn layer_b_{stage}_{model}() { ... }
// e.g., layer_b_fk_pendulum, layer_b_crba_double_pendulum

// Layer C: trajectory
fn layer_c_trajectory_{model}() { ... }
// e.g., layer_c_trajectory_pendulum, layer_c_trajectory_contact

// Layer D: property/invariant
fn layer_d_{property}() { ... }
// e.g., layer_d_energy_conservation, layer_d_quaternion_normalization
```

### 4. `#[ignore]` comment format

```rust
#[ignore] // CONFORMANCE GAP: <brief description> — see DT-XXX or Phase N
```

Always include:
- `CONFORMANCE GAP:` prefix (grep-able)
- Brief description of the failure
- Reference to the responsible DT or Phase

### 5. Tolerance constant naming

```rust
// In mujoco_conformance/common.rs
pub const TOL_FK: f64 = 1e-12;
pub const TOL_CRBA: f64 = 1e-12;
pub const TOL_RNE: f64 = 1e-10;
pub const TOL_PASSIVE: f64 = 1e-10;
pub const TOL_COLLISION_DEPTH: f64 = 1e-6;
pub const TOL_CONSTRAINT: f64 = 1e-4;
pub const TOL_ACTUATION: f64 = 1e-10;
pub const TOL_SENSOR: f64 = 1e-8;
pub const TOL_TENDON: f64 = 1e-10;
pub const TOL_INTEGRATION: f64 = 1e-8;
pub const TOL_FLAG_GOLDEN: f64 = 1e-8;
```

### 6. Python script conventions

- Shebang: `#!/usr/bin/env python3`
- MuJoCo version check at startup: `assert mujoco.__version__ == "3.4.0"`
- Use `numpy.save()` for individual `.npy` files (never `numpy.savez()`)
- Print progress to stdout: `print(f"Generated {path}")`
- All paths relative to script location
- Idempotent — safe to re-run

### 7. Model loading convention

```rust
// In conformance tests, always use:
let (model, mut data) = load_conformance_model("pendulum");
// which resolves to assets/golden/conformance/models/pendulum.xml
```

Helper function in `common.rs` handles path resolution and model loading.

---

## Cross-Spec Blast Radius

### Behavioral interactions between deliverables

| Interaction | Analysis |
|-------------|----------|
| DT-97 (Session 2) + Layer A+D (Session 3) | **Independent.** DT-97 works in `integration/golden_flags.rs` and `assets/golden/flags/`. Session 3 works in `mujoco_conformance/layer_a.rs` and `layer_d.rs`. No file overlap. |
| DT-97 (Session 2) + Ref Gen (Session 4) | **Complementary.** Both generate `.npy` files but for different purposes (flags vs per-stage reference). DT-97 extends `gen_flag_golden.py`; Session 4 creates `gen_conformance_reference.py`. Separate scripts, separate output directories. |
| Ref Gen (Session 4) + Spec A (Layer B) | **Producer-consumer.** Session 4 generates reference data; Spec A consumes it. No conflict — clear ownership boundary at `.npy` files. |
| Spec A (Layer B) + Spec B (Layer C) | **Complementary.** Spec A validates per-stage snapshots (single forward call). Spec B validates trajectories (multi-step). Both use `common.rs` utilities — Spec B adds to what Spec A established. |
| Layer A+D (Session 3) + Spec A (Layer B) | **Independent.** Layer A/D tests are MuJoCo-independent (self-consistency, invariants). Layer B tests compare against MuJoCo reference data. Different test methodologies, different files. |
| All sessions + Session 15 (Gate) | **Read-only.** Gate triage reads all `#[ignore]` annotations but modifies nothing. |

### Existing test impact

| Test area | Touched by | Conflict risk |
|-----------|-----------|---------------|
| `integration/golden_flags.rs` | Session 2 (extend) | **None.** Adds tests, no modifications to existing test. |
| `integration/*.rs` (other modules) | None | **None.** Phase 12 does not modify existing integration tests. |
| `mujoco_conformance/mod.rs` | Sessions 3, Spec A, Spec B (all add `mod` declarations) | **None.** Additive changes. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1 (Session 2: DT-97) | 24 (one per remaining flag) | +24 |
| T1 (Session 3: Layer A+D) | 10–20 (gap-fill tests) | +10–20 |
| T1 (Session 4: Ref Gen) | 0 (infrastructure only) | +0 |
| Spec A (Layer B) | 30–50 (10+ stages × multiple models) | +30–50 |
| Spec B (Layer C) | 6–10 (trajectory tests per model) | +6–10 |
| **Total** | **70–104** | **+70–104** |

---

## Out of Scope

Explicitly excluded from Phase 12. Each exclusion states its impact.

- **Fixing conformance failures** — Phase 12 writes tests and inventories
  failures. Fixing root causes happens in separate fix iterations back in
  Phases 1–11. *Impact: none — the test is the deliverable, not the fix.*

- **GPU tests** — CortenForge does not have GPU pipeline. *Impact: none.*

- **URDF conformance** — URDF loading is tested separately in `sim-urdf`.
  Phase 12 focuses on MJCF pipeline conformance. *Impact: minimal — URDF
  coverage is adequate.*

- **Performance benchmarks** — Phase 12 validates correctness, not speed.
  *Impact: none.*

- **MuJoCo build dependency for Rust tests** — All reference data is
  pre-generated and checked into the repo. Rust tests never invoke MuJoCo.
  *Impact: none — by design.*

- **Flag combination tests** — DT-97 tests single-flag isolation only. Testing
  flag interactions (e.g., DISABLE_GRAVITY + DISABLE_CONTACT) is combinatorial
  and deferred. *Impact: low — single-flag tests catch most bugs.*

- **Moving or modifying existing integration tests** — Phase 12 adds a new
  conformance binary. Existing tests stay exactly where they are. *Impact:
  none — PH12-AC7 enforces this.*

- **Analytical reference values** — All reference values come from MuJoCo
  3.4.0 output, not from hand-calculation. *Impact: positive — eliminates
  human error in expected values.*

- **Non-MuJoCo reference implementations** — Only MuJoCo 3.4.0 is the
  reference. No comparison against Bullet, Drake, or other simulators.
  *Impact: none — single authoritative source.*
