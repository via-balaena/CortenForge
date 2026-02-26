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

## Implementation Order

1. **T1** (cfrc_ext contacts) — validates the most complex code path in S51
2. **T3** (multi-body cfrc_int) — validates propagation correctness
3. **T4** (cb_sensor) — validates 3-stage callback dispatch
4. **T5** (actuator callbacks) — validates user-defined actuator pipeline
5. **T2** (sleep accumulators) — interaction test
6. **T6** (RK4 guard) — requires impl change first
