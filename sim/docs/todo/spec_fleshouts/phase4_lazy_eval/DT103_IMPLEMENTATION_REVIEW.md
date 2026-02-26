# DT-103 — Implementation Review

**Commit:** `29501df`
**Spec:** `DT103_SPATIAL_TRANSPORT.md` (Rev 5)
**Spec rubric:** `DT103_RUBRIC.md` (10 criteria — grades the spec, not the code)

This document grades the **implementation** against the spec. 6 criteria, all
must be A for ship.

---

## I1. Spec Fidelity

> Every function body, consumer rewrite, and import change matches the spec.

| Grade | Bar |
|-------|-----|
| **A** | All 5 new functions match spec code (modulo rustfmt). Wrapper matches. All 6 sensor arms match spec sketches. Import and re-export changes match spec §Blast Radius. Deviations documented and justified. |
| **B** | Functions correct. Minor deviations that don't affect semantics. |
| **C** | Functions diverge from spec without justification. |

### Checklist

- [x] `transport_motion` matches §New Functions 1
- [x] `transport_force` matches §New Functions 2
- [x] `object_velocity` matches §New Functions 3
- [x] `object_acceleration` matches §New Functions 4
- [x] `object_force` matches §New Functions 5
- [x] `object_velocity_local` wrapper matches §New Functions 6
- [x] Accelerometer arm matches §Consumer Rewrites
- [x] FrameLinAcc arm matches §Consumer Rewrites
- [x] Force arm matches §Consumer Rewrites
- [x] Torque arm matches §Consumer Rewrites
- [x] Velocimeter arm matches §Consumer Rewrites
- [x] FrameLinVel arm matches §Consumer Rewrites
- [x] `dynamics/mod.rs` re-exports match §Blast Radius (see §Deviations)
- [x] `sensor/acceleration.rs` import matches §Blast Radius
- [x] `sensor/velocity.rs` import matches §Blast Radius

### Justified Deviations from Spec

**Visibility.** Spec uses `pub(crate) fn` on all new functions. Implementation
uses `pub fn` because `spatial` is a `pub(crate) mod` — clippy
(`redundant_pub_crate`) correctly flags `pub(crate) fn` inside a private module.
Effective visibility is identical: crate-internal only. `#[must_use]` added on
public functions per clippy `must_use_candidate`.

**Re-exports.** Spec shows a single `pub(crate) use spatial::{...}` line with
all functions. Implementation splits into `pub use` (for `object_*` functions)
and `pub(crate) use` (for internal-only `compute_body_spatial_inertia`,
`shift_spatial_inertia`). The `pub use` makes the helpers accessible from
integration tests. `transport_motion` / `transport_force` remain un-exported
per spec.

### Status: ✓ PASS

All 15 checklist items verified. Every function body matches the spec modulo
rustfmt whitespace and the two justified deviations above (visibility, re-export
split). All 6 sensor arms match the spec sketches exactly: Accelerometer and
FrameLinAcc call `object_acceleration`, Force and Torque call `object_force`,
Velocimeter and FrameLinVel call `object_velocity`. Import lines in
`acceleration.rs` (line 18) and `velocity.rs` (line 16) match spec §Blast Radius.

---

## I2. Acceptance Criteria

> All 12 ACs pass. Each AC verified by test or code review.

| Grade | Bar |
|-------|-----|
| **A** | All 12 ACs pass. Behavioral ACs have tests with correct tolerances. Structural ACs verified by inspection or grep. |
| **B** | 11/12 pass; one minor gap with documented rationale. |
| **C** | ≤10 pass. |

### AC Traceability

| AC | Description | Method | Status |
|----|-------------|--------|--------|
| AC1 | `transport_motion` zero offset | T1 (unit) | ✓ pass (exact equality) |
| AC2 | `transport_motion` nonzero offset | T2 (unit) | ✓ pass (tol 1e-15) |
| AC3 | `transport_force` nonzero offset | T3 (unit) | ✓ pass (tol 1e-15) |
| AC4 | `object_velocity` ≡ `object_velocity_local` | T5, T14 | ✓ pass (tol 1e-15) |
| AC5 | `object_acceleration` Coriolis | T7, T8 | ✓ pass (T7: 1e-10, T8: 1e-8) |
| AC6 | Phase 4 sensor regression (39 tests) | T11 (suite) | ✓ pass (commit 29501df) |
| AC7 | Full sim domain regression (2,148+) | T12 (suite) | ✓ pass (commit 29501df: 2,148/0/20) |
| AC8 | `object_velocity_local` backward compat | T5, T14 | ✓ pass (bit-identical) |
| AC9 | Convention encoded once | `grep xpos\[body_id\]` in sensor/ → 0 hits | ✓ 0 hits confirmed |
| AC10 | `transport_force` force invariance | T4 (unit), T9 | ✓ pass (exact equality, 3 offsets) |
| AC11 | World body (body_id=0) valid output | T13 | ✓ pass (finite, vel=0, acc=gravity) |
| AC12 | No new `Data` fields / `EXPECTED_SIZE` unchanged | Code review | ✓ `types/data.rs` NOT modified |

### Status: ✓ 12/12 PASS

---

## I3. Test Coverage

> T1–T14 all exist, pass, and cover the right thing.

| Grade | Bar |
|-------|-----|
| **A** | All 14 tests exist and pass. Unit tests (T1–T4) in `spatial.rs`. Integration tests (T5–T10, T13–T14) in `spatial_transport.rs`. Regression suites (T11, T12) verified via full domain run. Tolerances match spec. |
| **B** | Tests pass but one or two are weaker than spec (wrong tolerance, missing edge). |
| **C** | Tests exist but some fail or are missing. |

### Test Inventory

| Test | Location | Type | Status |
|------|----------|------|--------|
| T1: `t01_transport_motion_zero_offset` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | ✓ pass |
| T2: `t02_transport_motion_cross_product` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | ✓ pass |
| T3: `t03_transport_force_moment_arm` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | ✓ pass |
| T4: `t04_transport_force_invariance` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | ✓ pass |
| T5: `t05_object_velocity_matches_local` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T6: `t06_object_velocity_world_frame` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T7: `t07_object_acceleration_static_gravity` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T8: `t08_object_acceleration_centripetal` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T9: `t09_object_force_translation_invariance` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T10: `t10_object_force_torque_transport` | `tests/integration/spatial_transport.rs` | Integration | ✓ pass |
| T11: Phase 4 regression (39 tests) | `tests/integration/sensors_phase4.rs` | Regression | ✓ pass (commit 29501df) |
| T12: Full sim domain (2,148+ tests) | domain-wide | Regression | ✓ pass (2,148/0/20 at commit 29501df) |
| T13: `t13_world_body` | `tests/integration/spatial_transport.rs` | Edge case | ✓ pass |
| T14: `t14_object_velocity_local_backward_compat` | `tests/integration/spatial_transport.rs` | Edge case | ✓ pass |

**Spec deviation — test placement.** Spec says "Write tests T1–T14 in
`spatial_transport.rs`." T1–T4 are placed in `spatial.rs` as `#[cfg(test)]`
unit tests instead, because `transport_motion` / `transport_force` are not
re-exported and are inaccessible from the integration test crate. This is the
standard Rust pattern for testing private functions.

### Status: ✓ 14/14 PASS

All tests verified by running:
- `cargo test -p sim-core -- t01 t02 t03 t04` → 4 passed
- `cargo test -p sim-conformance-tests -- spatial_transport` → 8 passed
- T11/T12 verified at commit 29501df (2,148 pass / 0 fail / 20 ignored)

---

## I4. Blast Radius

> Only predicted files changed. No unintended side effects.

| Grade | Bar |
|-------|-----|
| **A** | File set matches spec §Blast Radius (with justified additions). No unexpected files touched. `EXPECTED_SIZE` unchanged. `forward/passive.rs` and `derivatives.rs` compile without changes. Clippy clean. |
| **B** | Correct files modified. One minor extra change (e.g., fixing a pre-existing warning). |
| **C** | Unexpected files modified or behavioral changes in untouched code. |

### File Checklist

| File | Spec Prediction | Actual | Notes |
|------|-----------------|--------|-------|
| `dynamics/spatial.rs` | +~65, −~15 | +201, −24 | Larger: includes T1–T4 unit tests (~65 lines) + `#[must_use]` + rustfmt expansion |
| `dynamics/mod.rs` | +3 | +2, −3 | Split into two `use` lines (pub + pub(crate)) |
| `sensor/acceleration.rs` | −~55, +~12 | +21, −84 | Net removal larger than estimated (good) |
| `sensor/velocity.rs` | −~30, +~12 | +22, −56 | Net removal larger than estimated (good) |
| `tests/integration/spatial_transport.rs` | +~200 (new) | +373 (new) | Larger: thorough test models + assertions |
| `tests/integration/mod.rs` | (not in spec) | +8 | Module registration — required, minor |
| `forward/passive.rs` | NOT modified | NOT modified | ✓ |
| `derivatives.rs` | NOT modified | NOT modified | ✓ |
| `types/data.rs` | NOT modified | NOT modified | ✓ |

### Status: ✓ PASS

All predicted files modified. All predicted non-modifications confirmed.
One minor addition (`tests/integration/mod.rs` +8 lines) is structurally
required for module registration and does not affect behavior. Net line delta
in sensor files is larger removal than estimated (more duplication eliminated
than predicted).

---

## I5. Convention Safety

> The `xpos[body_id]` reference-point convention is encoded once in the helpers,
> not inlined in consumers. This is the central safety goal of the refactor.

| Grade | Bar |
|-------|-----|
| **A** | `grep xpos\[body_id\]` in `sensor/` returns 0 transport-related hits. All sensor transport goes through `object_velocity` / `object_acceleration` / `object_force`. Convention documented in helper doc comments. |
| **B** | One residual inline reference with justification. |
| **C** | Multiple residual inline transport patterns. |

### Status: ✓ PASS

`grep -n 'xpos\[body_id\]' sim/L0/core/src/sensor/` → **0 hits**.

All 6 refactored sensor arms call through the helpers:
- Accelerometer → `object_acceleration(data, body_id, &site_pos, Some(&site_mat))`
- FrameLinAcc → `object_acceleration(data, body_id, &obj_pos, None)`
- Force → `object_force(data, body_id, &site_pos, Some(&site_mat))`
- Torque → `object_force(data, body_id, &site_pos, Some(&site_mat))`
- Velocimeter → `object_velocity(data, body_id, &site_pos, Some(&site_mat))`
- FrameLinVel → `object_velocity(data, body_id, &site_pos, None)`

The `xpos[body_id]` convention is encapsulated inside `object_velocity` (line 275),
`object_acceleration` (line 308), and `object_force` (line 344) of `spatial.rs`.
Each helper's doc comment explicitly states "Convention: reads ... at `xpos[body_id]`."

---

## I6. Quality Gate

> Standard quality checks pass.

| Grade | Bar |
|-------|-----|
| **A** | `cargo clippy -- -D warnings` clean. `cargo fmt --all -- --check` clean. Full sim domain tests (2,148+) pass with zero failures. No new `unsafe`. |
| **B** | Clean after minor fix (e.g., unused import from development). |
| **C** | Warnings or test failures remain. |

### Status: ✓ PASS

- `cargo clippy -p sim-core -p sim-conformance-tests -- -D warnings` → **clean** (zero warnings)
- `cargo fmt --all -- --check` → **clean** (only nightly-feature warnings from `rustfmt.toml`, no formatting diff)
- Full sim domain: **2,148 pass / 0 fail / 20 ignored** (at commit 29501df)
- No new `unsafe` blocks: `spatial.rs` contains zero `unsafe`, sensor files contain zero `unsafe`

---

## Scoring Summary

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| I1. Spec Fidelity | **A** | All 15 checklist items verified. Two justified deviations (visibility, re-export split). |
| I2. Acceptance Criteria | **A** | 12/12 ACs pass. All behavioral ACs have tests with correct tolerances. |
| I3. Test Coverage | **A** | 14/14 tests exist and pass (4 unit + 8 integration + 2 regression suites). |
| I4. Blast Radius | **A** | Exact file set matches spec. No unexpected modifications. `EXPECTED_SIZE` unchanged. |
| I5. Convention Safety | **A** | 0 `xpos[body_id]` hits in sensor/. All transport through helpers. Convention documented. |
| I6. Quality Gate | **A** | Clippy clean, fmt clean, 2,148/0/20 domain tests, zero `unsafe`. |

**Ship criteria:** All A. **Result:** 6/6 at A.
