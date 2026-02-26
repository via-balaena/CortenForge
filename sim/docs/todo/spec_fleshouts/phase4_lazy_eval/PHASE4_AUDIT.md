# Phase 4 Branch Audit — Spec vs Implementation

**Status:** Draft
**Branch:** `phase_4_core_data_fields` (7 commits, 4 implementation)
**Scope:** Verify all 48 ACs and 51 test plan items across 4 specs.

---

## Inventory

| Spec | Implementation Commit | ACs | Tests |
|------|-----------------------|-----|-------|
| CVEL_REFERENCE_POINT_FIXES.md | `444046e` | 13 | 10 |
| S56_SUBTREE_VEL_SPEC.md | `503ac6d` | 13 | 15 |
| PHASE4_LAZY_EVAL_SPEC.md (4A.1–4A.8) | `8e8f5f7` | 13 | 12 |
| SENSOR_CACC_CFRC_REFACTOR.md (4A.6) | `16cfcb3` | 9 | 14 |
| **Totals** | | **48** | **51** |

After deduplication (umbrella ACs that overlap sub-spec ACs): **41 unique ACs**.

---

## Part 1: Code Review ACs (6 items — verify by reading source)

| AC | Claim | Status |
|----|-------|--------|
| S56-AC7 | `mj_subtree_vel()` is O(n), 3 passes, no parent-chain walks | ✓ `forward/velocity.rs` — 3 loops over `0..nbody` |
| S56-AC12 | `SubtreeCom` sensor reads `data.subtree_com[objid]` directly | ✓ `sensor/position.rs:159-163` |
| S56-AC13 | `compute_subtree_*` helpers deleted, no references remain | ✓ `sensor/derived.rs` deleted entirely |
| Umbrella-AC4 | = S56-AC7 | ✓ (same) |
| 4A.6-AC8 | `compute_body_acceleration`, `compute_body_angular_acceleration`, `compute_site_force_torque` deleted; `sensor/derived.rs` module deleted | ✓ Module gone, `mod derived;` removed |
| 4A.6-AC9 | No new `Data` fields; `EXPECTED_SIZE` unchanged at 4104 | ✓ |

---

## Part 2: Umbrella AC Deduplication

The umbrella spec's 13 ACs collapse into sub-spec ACs already counted:

| Umbrella AC | Covered by |
|-------------|------------|
| AC1 (`subtree_linvel` persistent) | S56-AC1, S56-AC2 |
| AC2 (`subtree_angmom` persistent) | S56-AC4, S56-AC5 |
| AC3 (sensor regression) | S56-AC6, 4A.6-AC7 |
| AC4 (O(n) complexity) | S56-AC7 (code review ✓) |
| AC5 (lazy subtree gate) | S56-AC8 |
| AC6 (lazy rnepost gate) | Lazy gate tests below |
| AC7 (once per step, rnepost) | Lazy gate tests below |
| AC8 (once per step, subtreevel) | S56-AC9 |
| AC9 (flag clearing) | S56-AC10, lazy gate tests |
| AC10 (reset clears fields) | S56-AC11 |
| AC11 (inverse dynamics) | Lazy gate tests below |
| AC12 (DISABLE_SENSOR) | Edge case tests below |
| AC13 (sleep interaction) | Edge case tests below |

**4 unique umbrella ACs remain** (AC6, AC7, AC9, AC11) requiring their own tests.

---

## Part 3: Gap Analysis — Existing Test Coverage

### Current coverage (2,102 tests pass)

| Existing test | What it covers | Tolerance |
|---------------|---------------|-----------|
| `test_accelerometer_at_rest_reads_gravity` | Accel reads +g when static | ±2.0 (!) |
| `test_accelerometer_in_free_fall_reads_zero` | Accel reads ~0 in free fall | < 1e-6 |
| `test_force_sensor_at_rest_in_gravity` | Force sensor reads nonzero | > 0.1 (!) |
| `test_torque_sensor_at_rest_in_gravity` | Torque sensor reads nonneg | ≥ 0.0 (!) |
| `test_subtree_angmom_spinning` | SubtreeAngMom nonzero when spinning | ≠ 0 (no exact value) |
| `body_accumulators.rs` (8 tests) | cacc, cfrc_int, cfrc_ext, reset, sleep | Exact values, tight |
| `ac7_sensor_disable` | DISABLE_SENSOR stale sensordata | Correct behavior |

**Problem:** The accelerometer/force/torque tests have tolerances so loose they
couldn't catch a sign error or a missing gravity term.

### Coverage summary

| Spec | ACs with 0 coverage | ACs with loose coverage | ACs fully covered |
|------|---------------------|------------------------|-------------------|
| CVEL fixes | **13** | 0 | 0 |
| §56 subtree | **10** | 1 (AC4 — nonzero check only) | 2 (code review) |
| Lazy rnepost | **4** | 0 | 0 |
| 4A.6 sensors | **5** | 2 (AC1, AC5 — loose) | 2 (code review) |
| **Total** | **32** | **3** | **4** |

---

## Part 4: New Tests Required

All tests go in a new integration test file:

**File:** `sim/L0/tests/integration/sensors_phase4.rs`
**Register:** add `pub mod sensors_phase4;` to `sim/L0/tests/integration/mod.rs`

### Section D: Acc-Stage Sensor Values (4A.6) — **Highest Priority**

These verify the most significant implementation changes (behavior changes,
new read paths through `cacc`/`cfrc_int`).

| Test | Model | Expected | Tol | Covers |
|------|-------|----------|-----|--------|
| `accelerometer_static_exact` | Free body on plane, mass=1 kg, gravity `(0,0,-9.81)`, site at body origin | `[0.0, 0.0, +9.81]` | 1e-10 | AC1, T2 |
| `accelerometer_centripetal_site_offset` | Free body, no gravity, ω=`[0,0,10]`, site at `[0.5,0,0]` | `[-50.0, 0.0, 0.0]` | 1e-8 | AC2, T3 |
| `accelerometer_free_fall_zero` | Free body, gravity, no contacts | `[0, 0, 0]` | 1e-6 | T1 |
| `frame_lin_acc_includes_gravity` | Free body on plane, gravity | `[0.0, 0.0, +9.81]` | 1e-10 | **AC3** (conformance fix), T4 |
| `frame_ang_acc_hinge` | Y-axis hinge, qacc=5.0 | `[0.0, 5.0, 0.0]` | 1e-10 | AC4, T5 |
| `force_sensor_single_body` | Free body on plane, mass=2 kg, gravity | `[0.0, 0.0, +19.62]` | 1e-6 | AC5, T6 |
| `torque_sensor_moment_arm` | xfrc_applied=`[0,0,0,0,0,100]`, no gravity, site at `[0.5,0,0]` | `[0.0, -50.0, 0.0]` | 1e-6 | AC6, T8 |
| `force_torque_3link_regression` | 3-link chain (hinge, mass=1, L=0.5), gravity, force+torque on link1 | Match analytical | 1e-6 | AC7, T7, T9 |
| `world_body_sensor` | Accelerometer on world body | `[0, 0, +9.81]`, finite | — | T11 |
| `zero_mass_body_force_torque` | Body mass≈0, force+torque sensors | Near-zero, no NaN | — | T12 |
| `disable_sensor_rnepost_not_triggered` | DISABLE_SENSOR, accelerometer model | `flg_rnepost == false`, stale data | — | T13 |
| `sleep_awake_body_triggers_gate` | Body A sleeping, B awake with accel | `flg_rnepost == true`, cacc[A] populated | — | T14 |

### Section A: CVEL Reference Point Fixes — **High Priority**

| Test | Model | Expected | Tol | Covers |
|------|-------|----------|-----|--------|
| `velocimeter_site_offset` | Hinge z-axis, site at `[0,0,0.5]`, qvel=1.0 | `[-0.5, 0.0, 0.0]` | 1e-10 | B1-AC1, T1 |
| `velocimeter_body_no_offset` | Same model, velocimeter on body | `[0.0, 0.0, 0.0]` | 1e-10 | B1-AC2 |
| `velocimeter_oblique_offset` | Hinge z-axis, site at `[0.3,0.4,0]`, qvel=2.0 | `[-0.8, 0.6, 0.0]`, `|v|=1.0` | 1e-8 | B1-AC3 |
| `frame_lin_vel_site_offset` | Hinge z-axis, site at `[0,0,0.5]`, qvel=1.0 | `[-0.5, 0.0, 0.0]` world | 1e-10 | B2-AC1, T2 |
| `frame_lin_vel_body_no_offset` | Same, FrameLinVel on body | `[0.0, 0.0, 0.0]` | 1e-10 | B2-AC2 |
| `subtree_linvel_com_offset` | `<inertial pos="0.1 0 0"/>`, hinge z, qvel=1.0 | `[0.0, 0.1, 0.0]` | 1e-10 | B3-AC1, B3-AC3, T3 |
| `subtree_linvel_no_com_offset` | `<inertial pos="0 0 0"/>`, same model | `[0.0, 0.0, 0.0]` | 1e-10 | B3-AC2 |
| `subtree_angmom_com_offset` | Same COM-offset model | Orbital term present | 1e-8 | B4-AC1, B4-AC3, T4 |
| `spin_angmom_reference_independent` | Symmetric body, hinge, no COM offset | Unchanged pre/post | 1e-12 | B4-AC2 |
| `kinetic_energy_com_offset` | COM-offset model, hinge z, qvel=1.0 | `KE == 0.5 * qvel^T * M * qvel` | 1e-10 | B5-AC1, T5 |
| `kinetic_energy_no_com_offset` | No COM offset | Same formula | 1e-10 | B5-AC2 |
| `regression_com_at_origin` | Simple model, all sensors | Values unchanged | 1e-12 | T6 |
| `world_body_velocimeter` | Velocimeter on world body | Zero, no panic | — | T7 |
| `zero_mass_momentum` | Body mass=0 | No NaN | — | T8 |
| `sleeping_body_velocimeter` | Sleeping body, site offset | Zero | — | T9 |

### Section B: §56 Persistent Subtree Fields — **Medium-High Priority**

| Test | Model | Expected | Tol | Covers |
|------|-------|----------|-----|--------|
| `free_body_linvel` | Free body, mass=1, qvel linvel=`[1,2,3]` | `subtree_linvel[1]=[1,2,3]`, `[0]=[1,2,3]` | 1e-12 | AC2, T1 |
| `chain_mass_weighted_avg` | 3-body, masses `[2,1,1]`, vels `[[1,0,0],[4,0,0],[0,0,0]]` | `subtree_linvel[root]=[1.5,0,0]` | 1e-10 | AC3, T2 |
| `spin_angmom_exact` | Hinge, I_zz=2.0, qvel=3.0 | `subtree_angmom[1]=[0,0,6.0]` | 1e-10 | AC4, T3 |
| `orbital_angmom` | Two bodies, known orbital config | Analytical L | 1e-8 | AC5, T4 |
| `lazy_no_trigger` | Model with only accel sensors | `flg_subtreevel == false`, fields zeroed | — | AC8, T7 |
| `lazy_single_trigger` | Model with 3 SubtreeLinVel sensors | `flg_subtreevel == true`, all populated | — | AC9, T8 |
| `flag_cleared_each_step` | Step twice, check flag lifecycle | Cleared at velocity start | — | AC10 |
| `reset_clears_fields` | Step, reset, check | Fields zeroed, flag false | — | AC11, T10 |
| `subtree_com_direct_read` | SubtreeCom sensor | Matches `data.subtree_com[body]` | exact | T11 |
| `world_body_total_angmom` | Multi-body, known L | `subtree_angmom[0]` = total | 1e-8 | T12 |
| `zero_mass_in_chain` | Middle body mass=0 | No NaN/panic | — | T13 |
| `disable_sensor_flag_false` | DISABLE_SENSOR | `flg_subtreevel == false` | — | T14 |
| `sleep_all_bodies_computed` | Awake + sleeping, SubtreeLinVel on awake | Sleeping body's fields populated | — | T15 |

### Section C: flg_rnepost Lazy Gate — **Medium Priority**

| Test | Model | Expected | Covers |
|------|-------|----------|--------|
| `no_acc_sensors_flag_false` | Only position sensors | `flg_rnepost == false`, cacc zeroed | Umbrella-AC6 |
| `multi_acc_sensors_consistent` | 3 accelerometers | `flg_rnepost == true`, all read gravity | Umbrella-AC7 |
| `flag_cleared_between_steps` | Accelerometer model, step 2× | Flag cleared then re-set each step | Umbrella-AC9 |
| `inverse_sets_flag` | Forward (no acc sensors), then inverse() | `flg_rnepost == true`, cacc populated | Umbrella-AC11 |

---

## Part 5: Execution Order

1. **Step 0:** Confirm baseline (existing 2,102 tests pass, code review ACs ✓)
2. **Step 1:** Write Section D (acc-stage sensors) — highest risk, largest behavior changes
3. **Step 2:** Write Section A (CVEL fixes) — physics correctness, zero coverage today
4. **Step 3:** Write Section B (§56 subtree) — some partial coverage exists
5. **Step 4:** Write Section C (lazy flags) — indirect coverage exists
6. **Step 5:** Run full suite, clippy, fix any failures
7. **Step 6:** Update this audit with pass/fail for every AC

---

## Part 6: Verification

After all tests pass:

```bash
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
cargo clippy -- -D warnings
```

All tests green, zero clippy warnings → audit complete.
