# Example API Fixes Spec

> **Status**: Draft
> **Origin**: Friction discovered building sim-ml visual examples (Phase 1 spaces + Phase 2 sim-env)
> **Scope**: sim-core, sim-ml-bridge, sim-bevy
> **Delete after**: All fixes implemented and validated

## Overview

Seven issues surfaced while building 7 examples across two phases. Each
required a workaround that future examples (and users) would repeat.
Fixing them now prevents compounding debt.

---

## Fix A: SimEnv closures → Send + Sync

**Crate**: sim-ml-bridge
**File**: `sim/L0/ml-bridge/src/env.rs`

### Problem

SimEnv stores closures as `Box<dyn Fn(...)>` (no Send/Sync bounds):

```rust
reward_fn:    Box<dyn Fn(&Model, &Data) -> f64>,
done_fn:      Box<dyn Fn(&Model, &Data) -> bool>,
truncated_fn: Box<dyn Fn(&Model, &Data) -> bool>,
on_reset_fn:  Option<Box<dyn FnMut(&Model, &mut Data)>>,
```

Bevy Resources require `Send + Sync + 'static`. Every Bevy integration
must use `NonSendMut` and `insert_non_send_resource()` — a single-
threaded workaround that blocks parallel system scheduling.

### Fix

Add `Send + Sync` bounds to all closure types:

```rust
reward_fn:    Box<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
done_fn:      Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
truncated_fn: Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
on_reset_fn:  Option<Box<dyn FnMut(&Model, &mut Data) + Send + Sync>>,
```

Update builder methods to require the bounds:

```rust
pub fn reward(mut self, f: impl Fn(&Model, &Data) -> f64 + Send + Sync + 'static) -> Self
pub fn done(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self
pub fn truncated(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self
pub fn on_reset(mut self, f: impl FnMut(&Model, &mut Data) + Send + Sync + 'static) -> Self
```

### Breaking changes

Closures that capture `Rc`, `Cell`, or other non-Send types will no
longer compile. This is rare — the stress test's `Rc<Cell<usize>>`
counter in check 4 is the only instance in our codebase. That test
should switch to `Arc<AtomicUsize>`.

VecEnv likely has the same issue — apply the same fix there.

### Validation

- `cargo test -p sim-ml-bridge` — all 111 tests pass
- Episode-loop example: replace `NonSendMut<SimEnvRes>` with
  `ResMut<SimEnvRes>` and `insert_non_send_resource` with
  `insert_resource`. Must compile and run.
- Stress test check 4: rewrite with `Arc<AtomicUsize>`, must pass.

---

## Fix B: PhysicsAccumulator field accessibility

**Crate**: sim-bevy
**File**: `sim/L1/bevy/src/model_data.rs`

### Problem

`PhysicsAccumulator(f64)` has a private inner field. External code
(anything outside sim-bevy) can't read or write it. Episode-loop had
to duplicate it as a local `StepAccumulator(f64)`.

### Fix

Make the field public (matches PhysicsModel and PhysicsData pattern):

```rust
#[derive(Resource)]
pub struct PhysicsAccumulator(pub f64);
```

### Breaking changes

None. Field was inaccessible before — no external code depends on
the current (private) state.

### Validation

- Episode-loop example: replace local `StepAccumulator` with
  `PhysicsAccumulator`. Must compile and run.
- `cargo test -p sim-bevy` — existing tests still pass.

---

## Fix C: SimEnv.step() should call forward()

**Crate**: sim-ml-bridge (SimEnv), documented in sim-core
**Files**: `sim/L0/ml-bridge/src/env.rs`, `sim/L0/core/src/types/data.rs`

### Problem

`Data::step()` integrates (qpos, qvel, time) but does NOT recompute
kinematics (geom_xpos, geom_xmat, sensordata, xpos, xquat). Users
must call `data.forward(&model)` separately.

`step_physics_realtime` in sim-bevy knows this and calls `forward()`
once after all steps in a frame. But SimEnv.step() does NOT — it runs
sub-steps, evaluates reward/done/truncated, and returns without ever
calling `forward()`.

This means:
- `env.data().geom_xpos` is stale after `env.step()`
- `env.data().sensordata` is stale after `env.step()`
- Rendering code must manually call `data_mut().forward(&model)`

Episode-loop discovered this the hard way. Any user doing
`env.step()` + render will have the same bug.

### Fix

Add a single `forward()` call at the end of `SimEnv::step()`, after
the sub-stepping loop and before evaluating reward/done/truncated:

```rust
fn step(&mut self, action: &Tensor) -> Result<StepResult, StepError> {
    self.act_space.apply(action, &mut self.data, &self.model);

    for _ in 0..self.sub_steps {
        self.data.step(&self.model)?;
        if (self.done_fn)(&self.model, &self.data) {
            break;
        }
    }

    // NEW: recompute kinematics so geom poses, sensors, etc. are fresh.
    self.data.forward(&self.model)?;

    let reward = (self.reward_fn)(&self.model, &self.data);
    let done = (self.done_fn)(&self.model, &self.data);
    let truncated = (self.truncated_fn)(&self.model, &self.data);
    let observation = self.obs_space.extract(&self.data);

    Ok(StepResult { observation, reward, done, truncated })
}
```

**Why here and not in Data::step()?** `Data::step()` is the low-level
integrator. Calling `forward()` inside it would add overhead for users
who step many times and only need kinematics once (like
`step_physics_realtime`). SimEnv is the right boundary — it owns the
sub-stepping loop and knows when stepping is complete.

### Breaking changes

- SimEnv.step() now returns `Result<StepResult, StepError>` where the
  error can also come from `forward()`. The existing signature already
  returns `Result<..., StepError>`, and `forward()` returns the same
  error type, so no signature change is needed.
- Reward/done/truncated closures that read derived quantities (sensors,
  geom poses) will now get correct values. Previously they got stale
  data — this is a correctness fix, not a behavioral change.
- Episode-loop: remove the manual `data_mut().forward()` call.

### Validation

- `cargo test -p sim-ml-bridge` — all tests pass.
- Stress test check 5 (on_reset + forward): should still pass (reset
  already calls forward).
- Stress test check 10 (observe consistency): should still pass.
- Episode-loop: remove manual forward() call, rendering must still
  work correctly.
- New test: after `env.step()`, verify `data().sensordata` matches
  `data().qpos` (sensors are recomputed).

---

## Fix D: Bevy Resource derives for ML bridge types

**Crate**: sim-ml-bridge
**Files**: `sim/L0/ml-bridge/src/space.rs`

### Problem

Every Bevy example wraps `ObservationSpace` and `ActionSpace` in
newtype structs with manual `Deref` impls:

```rust
#[derive(Resource)]
struct ObsSpace(ObservationSpace);
impl std::ops::Deref for ObsSpace { ... }
```

This 8-line pattern is duplicated in all 4 visual examples.

### Fix

`ObservationSpace` and `ActionSpace` are pure data (no closures) —
they're already `Send + Sync + 'static`. Add an optional `bevy`
feature that derives `Resource`:

```toml
# sim/L0/ml-bridge/Cargo.toml
[features]
bevy = ["dep:bevy_ecs"]

[dependencies]
bevy_ecs = { version = "0.18", optional = true }
```

```rust
// space.rs
#[cfg_attr(feature = "bevy", derive(bevy_ecs::prelude::Resource))]
pub struct ObservationSpace { ... }

#[cfg_attr(feature = "bevy", derive(bevy_ecs::prelude::Resource))]
pub struct ActionSpace { ... }
```

Also derive `Resource` on `SimEnv` behind the same feature (requires
Fix A first — SimEnv must be Send + Sync).

### Breaking changes

None — feature is opt-in. Examples enable it and drop their newtypes.

### Validation

- Examples compile without the newtype wrappers when `bevy` feature
  is enabled.
- `cargo test -p sim-ml-bridge` — tests pass with and without
  the feature.

---

## Fix E: ObservationSpace introspection

**Crate**: sim-ml-bridge
**File**: `sim/L0/ml-bridge/src/space.rs`

### Problem

obs-rich manually reconstructs the extractor layout to label HUD
segments. It hardcodes `(name, start_idx, end_idx)` tuples and
sensor→obs index mappings. If the space changes, the HUD silently
breaks.

### Fix

Add a `segments()` method that returns the extractor layout:

```rust
pub struct ObsSegment {
    pub label: String,   // e.g. "qpos(0..2)", "sensor(cart_pos)", "energy"
    pub offset: usize,   // start index in the flat obs vector
    pub dim: usize,      // number of elements
}

impl ObservationSpace {
    pub fn segments(&self) -> &[ObsSegment] { ... }
}
```

The builder already knows this information — it just needs to be
stored and exposed.

### Breaking changes

None — additive API.

### Validation

- obs-rich: replace manual `ExtractorLayout` with
  `obs_space.segments()`. HUD must display the same labels.
- Unit test: build a space with 5 extractors, verify segments()
  returns correct labels, offsets, and dims.

---

## Fix F: ValidationHarness wall-clock mode

**Crate**: sim-bevy
**File**: `sim/L1/bevy/src/examples.rs`

### Problem

ValidationHarness reads `data.time` for its `report_at()` and
`print_every()` triggers. For episodic environments, `data.time`
resets to 0 on each episode. The harness never reaches the report
time.

Episode-loop had to skip the harness entirely and implement custom
wall-clock timing.

### Fix

Add a `wall_clock()` mode that reads from Bevy's `Time` resource
instead of `data.time`:

```rust
impl ValidationHarness {
    /// Use wall-clock time (Bevy's Time resource) instead of sim time
    /// for report_at and print_every triggers. Use this for episodic
    /// environments where sim time resets per episode.
    pub fn wall_clock(mut self) -> Self {
        self.use_wall_clock = true;
        self
    }
}
```

In `validation_system`, branch on the flag:

```rust
let current_time = if harness.use_wall_clock {
    time.elapsed_secs_f64()
} else {
    data.time
};
```

### Breaking changes

None — new method, default behavior unchanged.

### Validation

- Episode-loop: switch to `ValidationHarness::new().wall_clock()
  .report_at(15.0)`. Remove custom wall-clock diagnostics. Report
  must trigger at 15s wall clock.
- Existing sim-cpu examples: unchanged behavior (still use sim time).

---

## Fix G: Rendering sync without full Data clone

**Crate**: sim-ml-bridge or sim-bevy
**File**: new method on SimEnv or utility in sim-bevy

### Problem

Pattern B copies the entire `Data` struct every frame for rendering:

```rust
physics_data.0 = env.0.data().clone();
```

Data has 50+ fields including large DVector/Vec allocations. For a
simple pendulum this is negligible, but for a 100-DOF robot with
contacts it becomes a real cost — every Vec is a fresh allocation +
memcpy per frame.

### Fix

Add a `sync_rendering` method that copies only the fields needed by
`sync_geom_transforms`, `sync_site_transforms`, and the HUD:

```rust
// In sim-bevy or as a standalone utility
pub fn sync_env_to_physics_data(env: &SimEnv, dst: &mut Data) {
    let src = env.data();

    // Required for sync_geom_transforms:
    dst.geom_xpos.clone_from(&src.geom_xpos);
    dst.geom_xmat.clone_from(&src.geom_xmat);

    // Required for sync_site_transforms:
    dst.site_xpos.clone_from(&src.site_xpos);
    dst.site_xmat.clone_from(&src.site_xmat);

    // Required for HUD / validation:
    dst.time = src.time;
    dst.qpos.clone_from(&src.qpos);
    dst.qvel.clone_from(&src.qvel);
    dst.ctrl.clone_from(&src.ctrl);
    dst.sensordata.clone_from(&src.sensordata);
}
```

Note: `Vec::clone_from()` reuses the existing allocation when lengths
match, so this is zero-alloc in steady state.

### Breaking changes

None — additive API. Existing `Data::clone()` still works.

### Validation

- Episode-loop: replace `physics_data.0 = env.0.data().clone()` with
  `sync_env_to_physics_data(&env.0, &mut physics_data.0)`. Rendering
  and HUD must still work correctly.
- Benchmark: compare frame time with full clone vs selective sync on
  a model with 20+ geoms.

---

## Implementation Order

Fixes have dependencies:

```
A (Send+Sync closures)
├── D (bevy Resource derives — requires A for SimEnv)
│
B (PhysicsAccumulator pub)
│
C (forward() in SimEnv.step())
│
E (ObservationSpace segments)
│
F (ValidationHarness wall_clock)
│
G (selective rendering sync)
```

Recommended order: **A → C → B → D → E → F → G**

A first because D depends on it. C next because it affects how all
future examples call step(). B is a one-liner. D, E, F, G are
independent and can be done in any order.

---

## Post-Fix Example Updates

After all fixes, update the affected examples:

| Example | Changes needed |
|---------|---------------|
| spaces/stress-test | Check 4: `Rc<Cell>` → `Arc<AtomicUsize>` (Fix A) |
| spaces/obs-extract | Drop ObsSpace newtype (Fix D) |
| spaces/act-inject | Drop ObsSpace + ActSpace newtypes (Fix D) |
| spaces/obs-rich | Drop ObsSpace newtype, replace manual layout with `segments()` (Fix D, E) |
| spaces/act-clamping | Drop ActSpace newtype (Fix D) |
| sim-env/stress-test | No changes needed |
| sim-env/episode-loop | Drop NonSendMut, drop manual forward(), drop StepAccumulator, use ValidationHarness.wall_clock(), use selective sync (Fixes A, B, C, D, F, G) |
