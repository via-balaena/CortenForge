# Phase 4 Branch Audit — Spec vs Implementation

**Status:** Complete
**Branch:** `phase_4_core_data_fields`
**Result:** 39 new tests, all 2,141 tests pass, 0 clippy warnings.

---

## Part 1: Inventory

| Spec | Implementation Commit | ACs | Test Plan Items |
|------|-----------------------|-----|-----------------|
| CVEL_REFERENCE_POINT_FIXES.md | `444046e` | 13 | 10 |
| S56_SUBTREE_VEL_SPEC.md | `503ac6d` | 13 | 15 |
| PHASE4_LAZY_EVAL_SPEC.md (umbrella) | `8e8f5f7` | 13 | 12 |
| SENSOR_CACC_CFRC_REFACTOR.md (4A.6) | `16cfcb3` | 9 | 14 |
| **Totals** | | **48** | **51** |

After deduplication (umbrella ACs that map to sub-spec ACs): **41 unique ACs**.

---

## Part 2: Code Review ACs (5 unique — no runtime test needed)

| AC | Claim | Status |
|----|-------|--------|
| S56-AC7 | `mj_subtree_vel()` is O(n), 3 passes, no parent-chain walks | ✓ `forward/velocity.rs` — 3 loops over `0..nbody` |
| S56-AC12 | `SubtreeCom` sensor reads `data.subtree_com[objid]` directly | ✓ `sensor/position.rs:159-163` (also tested: `b13`) |
| S56-AC13 | `compute_subtree_*` helpers deleted, no references remain | ✓ `sensor/derived.rs` deleted entirely |
| 4A.6-AC8 | `compute_body_acceleration`, `compute_body_angular_acceleration`, `compute_site_force_torque` deleted; `sensor/derived.rs` module deleted | ✓ Module gone, `mod derived;` removed |
| 4A.6-AC9 | No new `Data` fields; `EXPECTED_SIZE` unchanged at 4104 | ✓ |

Note: Umbrella-AC4 (O(n) complexity) is a duplicate of S56-AC7 — counted once above.

---

## Part 3: Umbrella AC Deduplication

The umbrella spec's 13 ACs collapse as follows:

| Umbrella AC | Maps to | Covered by |
|-------------|---------|------------|
| AC1 (`subtree_linvel` persistent) | S56-AC1, S56-AC2 | `b01`, `b02` |
| AC2 (`subtree_angmom` persistent) | S56-AC4, S56-AC5 | `b03`, `b04`, `b05` |
| AC3 (sensor regression) | S56-AC6 | `b01`–`b05` (exact values) |
| AC4 (O(n) complexity) | S56-AC7 | Code review ✓ |
| AC5 (lazy subtree gate) | S56-AC8 | `b06` |
| **AC6** (lazy rnepost gate) | **unique** | `c01` |
| **AC7** (once per step, rnepost) | **unique** | `c04` |
| AC8 (once per step, subtreevel) | S56-AC9 | `b07` |
| **AC9** (flag clearing) | **unique** | `c03` |
| AC10 (reset clears fields) | S56-AC11 | `b09` |
| **AC11** (inverse dynamics) | **unique** | `c02` |
| **AC12** (DISABLE_SENSOR) | **unique** ^† | `d11`, `a10`, `b11` |
| **AC13** (sleep interaction) | **unique** ^† | `d12`, `a09`, `b12` |

**6 unique umbrella ACs**: AC6, AC7, AC9, AC11 → Section C tests;
AC12, AC13 → cross-cutting edge case tests in Sections A/B/D.

^† Sub-specs have DISABLE_SENSOR and sleep as test plan items (T-numbers)
but not as numbered acceptance criteria. These two umbrella ACs are the
sole authority for these requirements.

---

## Part 4: AC Traceability Matrix

### 4A. CVEL Reference Point Fixes (13 ACs)

| AC | Description | Test | Tol | Status |
|----|-------------|------|-----|--------|
| B1-AC1 | Velocimeter site offset: tangential velocity | `a01_velocimeter_site_offset` | 1e-10 | ✓ |
| B1-AC2 | Velocimeter on body (no offset): unchanged | `a02_velocimeter_body_origin` | 1e-10 | ✓ |
| B1-AC3 | Velocimeter off-axis: `\|v\|` = ω×r | `a03_velocimeter_off_axis_magnitude` | 1e-10 | ✓ |
| B2-AC1 | FrameLinVel site offset: world frame | `a04_framelinvel_site_offset` | 1e-10 | ✓ |
| B2-AC2 | FrameLinVel at body origin: zero | `a05_framelinvel_body_origin` | 1e-10 | ✓ |
| B3-AC1 | Subtree momentum: xipos-xpos shift correct | `b02_subtree_linvel_chain_average` | 1e-8 | ✓ indirect ^1 |
| B3-AC2 | Subtree momentum: COM at origin unchanged | `b01_subtree_linvel_free_body` | 1e-10 | ✓ |
| B3-AC3 | SubtreeLinVel sensor reads corrected value | `b01`, `b02` | 1e-8 | ✓ |
| B4-AC1 | Subtree angmom: orbital term uses corrected v_com | `b05_subtree_angmom_world_total` | 1e-6 | ✓ indirect ^2 |
| B4-AC2 | Spin angmom: reference-point independent | `b03_subtree_angmom_spin` | 1e-4 | ✓ |
| B4-AC3 | SubtreeAngMom sensor: orbital term present | `b05_subtree_angmom_world_total` | 1e-6 | ✓ |
| B5-AC1 | KE consistency: 0.5·qvel^T·M·qvel | `a06_kinetic_energy_consistency` | 1e-10 | ✓ |
| B5-AC2 | KE: COM at origin unchanged | `a06_kinetic_energy_consistency` | 1e-10 | ✓ ^3 |

^1 `mj_subtree_vel` Phase 1 shifts from xpos to xipos for all bodies. `b02` would
give wrong mass-weighted average without this shift. Not a dedicated
`<inertial pos>` model — the fix is tested through the replacement algorithm.

^2 `b05` verifies total orbital angular momentum `[0,0,2]` for two opposing bodies.
This value depends on the corrected v_com in Phase 3 of `mj_subtree_vel`.

^3 Same test model (sphere at origin, COM = body origin) covers the "no offset"
baseline case.

### 4B. §56 Persistent Subtree Fields (13 ACs)

| AC | Description | Test | Tol | Status |
|----|-------------|------|-----|--------|
| AC1 | Fields are `Vec<Vector3<f64>>` of length `nbody` | `b01` (no panic, correct dims) | — | ✓ |
| AC2 | Free body: `subtree_linvel[1]=[1,2,3]` | `b01_subtree_linvel_free_body` | 1e-10 | ✓ |
| AC3 | 3-body chain: mass-weighted average | `b02_subtree_linvel_chain_average` | 1e-8 | ✓ |
| AC4 | Spin: `I_zz·ω_z = 6.0` | `b03_subtree_angmom_spin` | 1e-4 | ✓ |
| AC5 | Orbital angular momentum | `b04_subtree_angmom_orbital`, `b05_subtree_angmom_world_total` | 1e-6 | ✓ |
| AC6 | Sensor regression (exact values) | `b01`–`b05` (tight tolerances) | ≤1e-6 | ✓ |
| AC7 | O(n) complexity (code review) | Part 2 | — | ✓ |
| AC8 | Lazy: not triggered without subtree sensors | `b06_lazy_no_subtree_sensors` | — | ✓ |
| AC9 | Lazy: single trigger for multiple sensors | `b07_lazy_single_trigger_multiple_sensors` | — | ✓ |
| AC10 | Flag cleared each step at velocity start | `b08_flag_cleared_each_step` | — | ✓ |
| AC11 | Reset clears fields and flag | `b09_reset_clears_subtree` | — | ✓ |
| AC12 | `SubtreeCom` reads persistent field (code review) | Part 2 + `b13_subtreecom_reads_persistent` | exact | ✓ |
| AC13 | Dead code removed (code review) | Part 2 | — | ✓ |

### 4C. Umbrella Unique ACs (6 ACs)

| AC | Description | Test | Status |
|----|-------------|------|--------|
| AC6 | No acc sensors → `flg_rnepost` false, cacc zeroed | `c01_no_acc_sensors_flag_false` | ✓ |
| AC7 | Multiple acc sensors → single `mj_body_accumulators` call | `c04_multiple_acc_sensors_single_trigger` | ✓ |
| AC9 | `flg_rnepost` cleared between steps | `c03_flag_cleared_between_steps` | ✓ |
| AC11 | `inverse()` sets `flg_rnepost`, subsequent sensors skip | `c02_inverse_sets_flg_rnepost` | ✓ |
| AC12 | DISABLE_SENSOR: neither lazy gate triggered, fields stale | `d11`, `a10`, `b11` (3 tests across sections) | ✓ |
| AC13 | Sleep: lazy gates orthogonal, compute for all bodies | `d12`, `a09`, `b12` (3 tests across sections) | ✓ |

### 4D. 4A.6 Acc-Stage Sensor Values (9 ACs)

| AC | Description | Test | Tol | Status |
|----|-------------|------|-----|--------|
| AC1 | Accelerometer static: `[0, 0, +9.81]` | `d01_accelerometer_static_gravity` | 1e-10 | ✓ |
| AC2 | Accelerometer centripetal: `[-50, 0, 0]` | `d03_accelerometer_centripetal` | 1e-4 | ✓ |
| AC3 | FrameLinAcc includes gravity (conformance fix) | `d04_framelinacc_includes_gravity` | 1e-10 | ✓ |
| AC4 | FrameAngAcc hinge: `[0, qacc, 0]` | `d05_frameangacc_hinge` | 1e-10 | ✓ |
| AC5 | Force sensor static: `[0, 0, +19.62]` | `d06_force_sensor_static` | 1e-6 | ✓ |
| AC6 | Torque spatial shift: `τ_site = τ_origin - r × f` | `d07_torque_sensor_moment_arm` | 1e-6 | ✓ |
| AC7 | Force/Torque 3-link chain: monotonic propagation | `d08_force_torque_chain_regression` | 10% | ✓ |
| AC8 | Dead code removed (code review) | Part 2 | — | ✓ |
| AC9 | No new Data fields (code review) | Part 2 | — | ✓ |

---

## Part 5: Edge Case Coverage

Cross-cutting edge cases from all 4 specs, each tested in multiple sections:

| Edge case | Spec refs | Tests |
|-----------|-----------|-------|
| World body sensor | CVEL-T7, 4A.6-T11 | `a07_vel_sensor_world_body_zero`, `d09_acc_sensor_world_body_finite` |
| Zero-mass body | CVEL-T8, S56-T13, 4A.6-T12 | `a08_vel_sensor_zero_mass_no_nan`, `b10_subtree_zero_mass_no_nan`, `d10_acc_sensor_zero_mass_no_nan` |
| DISABLE_SENSOR | CVEL-T10, S56-T14, 4A.6-T13, Umbrella-AC12 | `a10_vel_sensor_disable_sensor`, `b11_subtree_disable_sensor`, `d11_disable_sensor_acc_stage` |
| Sleep interaction | CVEL-T9, S56-T15, 4A.6-T14, Umbrella-AC13 | `a09_vel_sensor_sleeping_frozen`, `b12_subtree_sleep_computes_all`, `d12_acc_sensor_sleep_computes_all` |
| Free-fall accelerometer | 4A.6-T1 | `d02_accelerometer_free_fall` |

---

## Part 6: Test Manifest

**File:** `sim/L0/tests/integration/sensors_phase4.rs`
**Registered in:** `sim/L0/tests/integration/mod.rs`
**Commit:** `9f98755`

### Section D: 4A.6 Acc-Stage Sensor Values (12 tests)

| # | Test | Covers |
|---|------|--------|
| 1 | `d01_accelerometer_static_gravity` | AC1, T2 |
| 2 | `d02_accelerometer_free_fall` | T1 |
| 3 | `d03_accelerometer_centripetal` | AC2, T3 |
| 4 | `d04_framelinacc_includes_gravity` | **AC3** (conformance fix), T4 |
| 5 | `d05_frameangacc_hinge` | AC4, T5 |
| 6 | `d06_force_sensor_static` | AC5, T6 |
| 7 | `d07_torque_sensor_moment_arm` | AC6, T8 |
| 8 | `d08_force_torque_chain_regression` | AC7, T7, T9 |
| 9 | `d09_acc_sensor_world_body_finite` | T11 |
| 10 | `d10_acc_sensor_zero_mass_no_nan` | T12 |
| 11 | `d11_disable_sensor_acc_stage` | T13, Umbrella-AC12 |
| 12 | `d12_acc_sensor_sleep_computes_all` | T14, Umbrella-AC13 |

### Section A: CVEL Reference Point Fixes (10 tests)

| # | Test | Covers |
|---|------|--------|
| 1 | `a01_velocimeter_site_offset` | B1-AC1, T1 |
| 2 | `a02_velocimeter_body_origin` | B1-AC2 |
| 3 | `a03_velocimeter_off_axis_magnitude` | B1-AC3 |
| 4 | `a04_framelinvel_site_offset` | B2-AC1, T2 |
| 5 | `a05_framelinvel_body_origin` | B2-AC2 |
| 6 | `a06_kinetic_energy_consistency` | B5-AC1, B5-AC2, T5 |
| 7 | `a07_vel_sensor_world_body_zero` | T7 |
| 8 | `a08_vel_sensor_zero_mass_no_nan` | T8 |
| 9 | `a09_vel_sensor_sleeping_frozen` | T9, Umbrella-AC13 |
| 10 | `a10_vel_sensor_disable_sensor` | T10, Umbrella-AC12 |

### Section B: §56 Persistent Subtree Fields (13 tests)

| # | Test | Covers |
|---|------|--------|
| 1 | `b01_subtree_linvel_free_body` | AC2, B3-AC2, T1 |
| 2 | `b02_subtree_linvel_chain_average` | AC3, B3-AC1, B3-AC3, T2 |
| 3 | `b03_subtree_angmom_spin` | AC4, B4-AC2, T3 |
| 4 | `b04_subtree_angmom_orbital` | AC5, T4 |
| 5 | `b05_subtree_angmom_world_total` | AC5, B4-AC1, B4-AC3, T12 |
| 6 | `b06_lazy_no_subtree_sensors` | AC8, T7 |
| 7 | `b07_lazy_single_trigger_multiple_sensors` | AC9, T8 |
| 8 | `b08_flag_cleared_each_step` | AC10 |
| 9 | `b09_reset_clears_subtree` | AC11, T10 |
| 10 | `b10_subtree_zero_mass_no_nan` | T13 |
| 11 | `b11_subtree_disable_sensor` | T14, Umbrella-AC12 |
| 12 | `b12_subtree_sleep_computes_all` | T15, Umbrella-AC13 |
| 13 | `b13_subtreecom_reads_persistent` | AC12, T11 |

### Section C: flg_rnepost Lazy Gate (4 tests)

| # | Test | Covers |
|---|------|--------|
| 1 | `c01_no_acc_sensors_flag_false` | Umbrella-AC6 |
| 2 | `c02_inverse_sets_flg_rnepost` | Umbrella-AC11 |
| 3 | `c03_flag_cleared_between_steps` | Umbrella-AC9 |
| 4 | `c04_multiple_acc_sensors_single_trigger` | Umbrella-AC7 |

---

## Part 7: Coverage Summary

| Category | Count | Status |
|----------|-------|--------|
| Code review ACs | 5 | ✓ all verified |
| Runtime ACs with direct test | 30 | ✓ all pass |
| Runtime ACs with indirect test | 6 ^4 | ✓ all pass |
| Edge case test items | 14 | ✓ all pass |
| **Total unique ACs** | **41** | **41/41 covered** |
| **Total new tests** | **39** | **39/39 pass** |

Breakdown: 13 CVEL + 13 S56 + 6 umbrella-unique + 9 4A.6 = 41.
Of these, 5 are code-review-only, 6 are indirect, 30 are direct.

^4 Indirect: B3-AC1, B3-AC3, B4-AC1 (tested through `mj_subtree_vel` replacement,
not a dedicated `<inertial pos>` model); S56-AC1 (field existence, not panicking);
S56-AC6 (regression by exact values, not snapshot comparison); B5-AC2 (same model
covers both cases).

---

## Part 8: Verification

```
$ cargo test -p sim-conformance-tests -- sensors_phase4
test result: ok. 39 passed; 0 failed; 0 ignored

$ cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
    -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
    -p sim-types -p sim-simd
test result: ok. 2,141 passed; 0 failed

$ cargo clippy -- -D warnings
0 warnings
```

Audit complete.
