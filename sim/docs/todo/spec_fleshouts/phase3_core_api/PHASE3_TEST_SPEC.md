# Phase 3 — Missing Test Specifications

Tests needed to bring Phase 3 from B to A grade. Each test specifies the
exact setup, action, and assertion so implementation is mechanical.

---

## T1. S51: cfrc_ext with Contact Forces (G5)

**File:** `sim/L0/tests/integration/body_accumulators.rs`

### Setup
- MJCF: Sphere (mass=1, geom=sphere r=0.1) on a plane (body 0 has plane geom)
- Free joint on sphere body
- Gravity enabled, sphere resting on plane

### Action
```rust
data.forward(&model)?;
```

### Assertions
1. `data.ncon >= 1` — at least one contact generated
2. `data.cfrc_ext[sphere_body_id]` has nonzero linear component (contact normal force)
3. `cfrc_ext[sphere_body_id][5]` > 0 (upward contact force ≈ mg)
4. `cfrc_ext[0]` (world body) has equal-and-opposite force (Newton's 3rd law)

---

## T2. S51: Body Accumulators with Sleep (G6)

**File:** `sim/L0/tests/integration/body_accumulators.rs`

### Setup
- MJCF: Single body with hinge joint, ENABLE_SLEEP set
- Run enough steps to put the body to sleep (zero velocity, damping)
- Verify `data.body_sleep_state[1] == SleepState::Asleep`

### Action
```rust
data.forward(&model)?;
```

### Assertions
1. `cacc[1]` is still computed (sleeping bodies still have gravity)
2. `cfrc_int[1]` reflects static equilibrium forces
3. Body accumulators are physically consistent even when sleeping

---

## T3. S51: Multi-Body cfrc_int Propagation (G7)

**File:** `sim/L0/tests/integration/body_accumulators.rs`

### Setup
- MJCF: 3-link serial chain (world → body1 → body2 → body3), hinge joints
- Equal masses (1 kg each), equal link lengths (0.5 m)
- Gravity enabled, zero velocity (static configuration)

### Action
```rust
data.forward(&model)?;
```

### Assertions
1. `cfrc_int[3]` supports body3's weight only (1g downward)
2. `cfrc_int[2]` supports body2 + body3 weight (2g downward)
3. `cfrc_int[1]` supports all three bodies (3g downward)
4. `cfrc_int[0]` = sum of all children (total weight)
5. At each joint: `|cfrc_int[child]| <= |cfrc_int[parent]|` (monotonic)

---

## T4. DT-79: cb_sensor Invocation (G17)

**File:** `sim/L0/tests/integration/callbacks.rs`

### Setup
- MJCF with 3 User sensors: one Position-stage, one Velocity-stage, one Acceleration-stage
- Callback that records `(sensor_id, stage)` into a shared `Arc<Mutex<Vec<...>>>`

### Action
```rust
model.set_sensor_callback(|_model, data, sensor_id, stage| {
    // Write a known value to sensordata to prove invocation
    let adr = model.sensor_adr[sensor_id];
    data.sensordata[adr] = match stage {
        SensorStage::Pos => 100.0,
        SensorStage::Vel => 200.0,
        SensorStage::Acc => 300.0,
    };
});
data.forward(&model)?;
```

### Assertions
1. Position-stage sensor data == 100.0
2. Velocity-stage sensor data == 200.0
3. Acceleration-stage sensor data == 300.0

---

## T5. DT-79: Actuator Callback Invocation (G18)

**File:** `sim/L0/tests/integration/callbacks.rs`

### Setup
- MJCF with one actuator using `dyntype="user"`, `gaintype="user"`, `biastype="user"`
- Register all three actuator callbacks

### Action
```rust
model.set_act_dyn_callback(|_m, _d, _id| 1.5);   // act_dot = 1.5
model.set_act_gain_callback(|_m, _d, _id| 2.0);   // gain = 2.0
model.set_act_bias_callback(|_m, _d, _id| -0.5);  // bias = -0.5
data.ctrl[0] = 1.0;
data.step(&model)?;
```

### Assertions
1. `data.act_dot[0]` == 1.5 (dynamics callback return value)
2. Actuator force == `gain * ctrl + bias` = `2.0 * 1.0 + (-0.5)` = 1.5
3. With no callbacks set, `act_dot` == 0.0 and gain/bias == 0.0 (safe defaults)

---

## T6. S53: RK4 + step2() Warning (G12)

**File:** `sim/L0/tests/integration/split_step.rs`

### Setup
- Model with `integrator = RungeKutta4`

### Action
```rust
data.step1(&model)?;
data.step2(&model)?;
```

### Assertions
1. No panic (step2 should work, just with Euler)
2. Results differ from `step()` (which uses actual RK4)
3. A `tracing::warn!` is emitted (verify via tracing test subscriber)

---

## Implementation Order (T1–T6)

1. **T1** (cfrc_ext contacts) — validates the most complex code path in S51
2. **T3** (multi-body cfrc_int) — validates propagation correctness
3. **T4** (cb_sensor) — validates 3-stage callback dispatch
4. **T5** (actuator callbacks) — validates user-defined actuator pipeline
5. **T2** (sleep accumulators) — interaction test
6. **T6** (RK4 guard) — requires impl change first

---

# Phase 3 — Gap Tests (T7–T12)

Tests identified during post-implementation review. Close remaining coverage
gaps from the Phase 3 audit (G2, G8, G22–G24) and strengthen T1/T6.

## Vetting Rubric

Each test was vetted against 5 criteria before implementation:

| # | Criterion | Question |
|---|-----------|----------|
| V1 | Code exists | Does the code path under test actually exist today? |
| V2 | Assertion correct | Are the expected values physically/mathematically justified? |
| V3 | API feasible | Can the setup be built with current model/data fields? |
| V4 | Non-redundant | Does it add coverage not already present? |
| V5 | Deterministic | Free of race conditions, timing sensitivity, or flaky tolerances? |

### Rubric Results

| Test | V1 | V2 | V3 | V4 | V5 | Verdict |
|------|----|----|----|----|----|----|
| T7 | FAIL | — | — | — | — | **Killed** |
| T8 | Pass | Pass | Partial | Pass | Pass | **Killed** |
| T9 | Pass | Pass | Pass | Pass | Pass | **Ship** |
| T10 | Pass | FAIL | — | — | — | **Killed** |
| T11 | Pass | Pass | Pass | Pass | Pass | **Ship** |
| T12 | — | — | — | — | — | **Deferred** |

---

## T7. ~~T1 gap: Newton's 3rd Law on World Body~~ — KILLED

**Reason (V1 FAIL):** `mj_body_accumulators()` at `acceleration.rs:509-512`
explicitly skips `body_id == 0` when accumulating contact forces into cfrc_ext.
The world body's cfrc_ext stays at its xfrc_applied value (zero). This is
**by design** — MuJoCo does the same. The spec's assertion is physically
intuitive but not how the data structure works.

---

## T8. ~~T6 gap: tracing::warn Capture~~ — KILLED

**Reason (V3 PARTIAL → not worth it):** `tracing-test` is not a dev-dependency
anywhere in the workspace. Adding it just to assert one warning message is not
justified. T6 already tests the behavioral consequence (Euler fallback
divergence). The warning at `forward/mod.rs:145-148` is verified by code
inspection.

---

## T9. S52: Inverse Dynamics Identity — SHIP

**File:** `sim/L0/tests/integration/inverse_dynamics.rs` (new)

### Context
No test verifies `qfrc_inverse` correctness after a forward+inverse pass.
Audit gap G22 (constraint forces) and G23 (fwdinv format).

### Setup
- MJCF: 2-link pendulum with hinge joints, gravity enabled
- Apply known `qfrc_applied` and `ctrl` values
- Run `forward()` to compute qacc

### Action
```rust
data.forward(&model)?;
// Now qacc is known. Inverse dynamics should recover the generalized forces.
```

### Assertions
1. `qfrc_inverse` is nonzero (nontrivial configuration)
2. Inverse dynamics identity: for each DOF i,
   `qfrc_inverse[i] ≈ M*qacc + bias - qfrc_passive - qfrc_constraint`
   (the actual formula in `inverse.rs`)
3. If ENABLE_FWDINV is set, `qfrc_inverse` is populated; if not, it is zero

### Implementation notes
- `Data::inverse()` is at `inverse.rs:33-46`
- Formula: `qfrc_inverse = M*qacc + qfrc_bias - qfrc_passive - qfrc_constraint`
- ENABLE_FWDINV flag: `model.enableflags |= ENABLE_FWDINV` (u32 constant)
- `compare_fwd_inv()` in `forward/mod.rs:443-474` runs inverse when flag is set

---

## T10. ~~Reset Completeness~~ — KILLED

**Reason (V2 FAIL):** `Data::reset()` already zeroes every Phase 3 field
(`sensordata`, `act_dot`, `cacc`, `cfrc_int`, `cfrc_ext`, `qfrc_inverse`).
Plus a compile-time `data_reset_field_inventory` test forces updates to
`reset()` whenever a field is added to Data. The existing
`body_accumulators_zeroed_after_reset` test covers the accumulator subset.
No additional coverage needed.

---

## T11. DT-79: cb_control Skipped When DISABLE_ACTUATION Set (G24) — SHIP

**File:** `sim/L0/tests/integration/callbacks.rs`

### Context
Audit gap G24 — `cb_control` should be gated by the DISABLE_ACTUATION flag.
When the flag is set, the control callback must not fire.

### Setup
- MJCF with one motor actuator
- Register `cb_control` that sets `ctrl[0] = 99.0` and records invocation
  via `Arc<AtomicBool>`
- Set `model.disableflags |= DISABLE_ACTUATION`

### Action
```rust
data.forward(&model)?;
```

### Assertions
1. The `AtomicBool` is `false` — callback was never invoked
2. `data.ctrl[0]` is 0.0 (untouched)
3. `data.qfrc_actuator[0]` is 0.0 (no actuation)

### Implementation notes
- Flag: `DISABLE_ACTUATION` constant (`1 << 11`) in `types/enums.rs`
- Field: `model.disableflags: u32` (no underscore)
- Gate exists in `forward_core()` at `forward/mod.rs:276-283`
- Note: `step1()` does NOT gate cb_control on DISABLE_ACTUATION — this test
  uses `forward()` which routes through `forward_core()`

---

## T12. Numerical MuJoCo Cross-Validation — DEFERRED

**File:** `sim/L0/sim-conformance-tests/` (when infrastructure exists)

### Context
T1–T11 test physics invariants and API contracts. This test tier would compare
exact numerical output against MuJoCo C for the same MJCF models.

### Status
**Deferred.** Requires MuJoCo C bindings or golden-file infrastructure. Tracked
here for completeness — not part of the current implementation pass.

### When to implement
- When `sim-conformance-tests` gains golden-file comparison infrastructure
- When MuJoCo C bindings (via `mujoco-sys` or FFI) are available for generating
  reference data

---

## Implementation Order (T7–T12)

1. ~~**T7**~~ — killed (V1 fail: world body cfrc_ext intentionally unpopulated)
2. ~~**T10**~~ — killed (V2 fail: already covered by compile-time guard)
3. **T9** (inverse dynamics identity) — new file, closes biggest untested feature
4. **T11** (DISABLE_ACTUATION gate) — closes audit gap G24
5. ~~**T8**~~ — killed (not worth a new dependency; T6 covers the behavior)
6. **T12** — deferred (needs MuJoCo C bindings)
