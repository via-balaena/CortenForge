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

- [ ] `transport_motion` matches §New Functions 1
- [ ] `transport_force` matches §New Functions 2
- [ ] `object_velocity` matches §New Functions 3
- [ ] `object_acceleration` matches §New Functions 4
- [ ] `object_force` matches §New Functions 5
- [ ] `object_velocity_local` wrapper matches §New Functions 6
- [ ] Accelerometer arm matches §Consumer Rewrites
- [ ] FrameLinAcc arm matches §Consumer Rewrites
- [ ] Force arm matches §Consumer Rewrites
- [ ] Torque arm matches §Consumer Rewrites
- [ ] Velocimeter arm matches §Consumer Rewrites
- [ ] FrameLinVel arm matches §Consumer Rewrites
- [ ] `dynamics/mod.rs` re-exports match §Blast Radius (see §Deviations)
- [ ] `sensor/acceleration.rs` import matches §Blast Radius
- [ ] `sensor/velocity.rs` import matches §Blast Radius

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
| AC1 | `transport_motion` zero offset | T1 (unit) | |
| AC2 | `transport_motion` nonzero offset | T2 (unit) | |
| AC3 | `transport_force` nonzero offset | T3 (unit) | |
| AC4 | `object_velocity` ≡ `object_velocity_local` | T5, T14 | |
| AC5 | `object_acceleration` Coriolis | T7, T8 | |
| AC6 | Phase 4 sensor regression (39 tests) | T11 (suite) | |
| AC7 | Full sim domain regression (2,148+) | T12 (suite) | |
| AC8 | `object_velocity_local` backward compat | T5, T14 | |
| AC9 | Convention encoded once | `grep xpos\[body_id\]` in sensor/ → 0 hits | |
| AC10 | `transport_force` force invariance | T4 (unit), T9 | |
| AC11 | World body (body_id=0) valid output | T13 | |
| AC12 | No new `Data` fields / `EXPECTED_SIZE` unchanged | Code review | |

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
| T1: `t01_transport_motion_zero_offset` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | |
| T2: `t02_transport_motion_cross_product` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | |
| T3: `t03_transport_force_moment_arm` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | |
| T4: `t04_transport_force_invariance` | `dynamics/spatial.rs` `#[cfg(test)]` | Unit | |
| T5: `t05_object_velocity_matches_local` | `tests/integration/spatial_transport.rs` | Integration | |
| T6: `t06_object_velocity_world_frame` | `tests/integration/spatial_transport.rs` | Integration | |
| T7: `t07_object_acceleration_static_gravity` | `tests/integration/spatial_transport.rs` | Integration | |
| T8: `t08_object_acceleration_centripetal` | `tests/integration/spatial_transport.rs` | Integration | |
| T9: `t09_object_force_translation_invariance` | `tests/integration/spatial_transport.rs` | Integration | |
| T10: `t10_object_force_torque_transport` | `tests/integration/spatial_transport.rs` | Integration | |
| T11: Phase 4 regression (39 tests) | `tests/integration/sensors_phase4.rs` | Regression | |
| T12: Full sim domain (2,148+ tests) | domain-wide | Regression | |
| T13: `t13_world_body` | `tests/integration/spatial_transport.rs` | Edge case | |
| T14: `t14_object_velocity_local_backward_compat` | `tests/integration/spatial_transport.rs` | Edge case | |

**Spec deviation — test placement.** Spec says "Write tests T1–T14 in
`spatial_transport.rs`." T1–T4 are placed in `spatial.rs` as `#[cfg(test)]`
unit tests instead, because `transport_motion` / `transport_force` are not
re-exported and are inaccessible from the integration test crate. This is the
standard Rust pattern for testing private functions.

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

---

## I5. Convention Safety

> The `xpos[body_id]` reference-point convention is encoded once in the helpers,
> not inlined in consumers. This is the central safety goal of the refactor.

| Grade | Bar |
|-------|-----|
| **A** | `grep xpos\[body_id\]` in `sensor/` returns 0 transport-related hits. All sensor transport goes through `object_velocity` / `object_acceleration` / `object_force`. Convention documented in helper doc comments. |
| **B** | One residual inline reference with justification. |
| **C** | Multiple residual inline transport patterns. |

---

## I6. Quality Gate

> Standard quality checks pass.

| Grade | Bar |
|-------|-----|
| **A** | `cargo clippy -- -D warnings` clean. `cargo fmt --all -- --check` clean. Full sim domain tests (2,148+) pass with zero failures. No new `unsafe`. |
| **B** | Clean after minor fix (e.g., unused import from development). |
| **C** | Warnings or test failures remain. |

---

## Scoring Summary

| Criterion | Grade | Evidence |
|-----------|:-----:|----------|
| I1. Spec Fidelity | | |
| I2. Acceptance Criteria | | |
| I3. Test Coverage | | |
| I4. Blast Radius | | |
| I5. Convention Safety | | |
| I6. Quality Gate | | |

**Ship criteria:** All A. **Result:** ___/6 at A.
