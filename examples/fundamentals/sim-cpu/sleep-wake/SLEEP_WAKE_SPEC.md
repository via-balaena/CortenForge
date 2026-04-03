# Sleep/Wake Examples — Specification

**Layer:** 6 (Island-Based Deactivation)
**Engine module:** `sim-core/src/island/` (1,415 LOC: `mod.rs` 665 + `sleep.rs` 750)
**Test coverage:** `sim-conformance-tests/integration/sleeping.rs` (5,093 LOC, T1–T106)

---

## Overview

The sleep system groups bodies into **constraint islands** and deactivates
islands whose velocities fall below a threshold. Sleeping bodies skip
integration and collision narrowphase — essential for large scenes with many
resting objects. Each visual example demonstrates exactly **one concept**.

### Key engine concepts

| Concept | Type / Field | Description |
|---------|-------------|-------------|
| `SleepPolicy` | enum | `Auto`, `AutoAllowed`, `AutoNever`, `Never`, `Allowed`, `Init` |
| `SleepState` | enum | `Static` (world), `Asleep`, `Awake` |
| `ENABLE_SLEEP` | flag (bit 5) | Must be set for sleep to activate |
| `sleep_tolerance` | `f64` (default 1e-4) | Velocity threshold: `|qvel[dof]| <= tol * dof_length[dof]` |
| `MIN_AWAKE` | const = 10 | Minimum awake timesteps before sleep-eligible |
| `tree_asleep` | `Vec<i32>` | `< 0` = awake (countdown), `>= 0` = asleep (cycle link) |
| `body_sleep_state` | `Vec<SleepState>` | Per-body query cache |
| `nisland` | `usize` | Number of constraint islands this step |
| `tree_island` | `Vec<i32>` | Island assignment per tree (-1 = singleton) |
| `nbody_awake` | `usize` | Count of awake bodies |

### Sleep lifecycle

```
Drop → Collide → Settle → Countdown (10 steps) → Sleep
                                                    ↓
                              Contact/Force → Wake → Countdown → Sleep
```

### Visual convention (all visual examples)

Bodies are **color-coded by sleep state** each frame:
- **Orange/warm** = Awake
- **Steel blue** = Asleep
- **Dark grey** = Static (ground plane)

This makes sleep transitions immediately visible without reading the console.

### MJCF requirements (shared)

- `<flag sleep="enable"/>` — required, sleep is off by default
- Euler integrator — RK4 is incompatible with sleep (auto-disabled with warning)
- No actuators on sleeping bodies — actuated trees get `AutoNever` policy
- Appropriate `sleep_tolerance` per example (default 1e-4 is fine for most)

---

## Example 1: `sleep-settle/` — Velocity Threshold and Sleep Transition

### Concept
Bodies go to sleep when their velocity drops below the threshold for long
enough. This is the fundamental sleep mechanism.

### One-liner
Five boxes drop from staggered heights and turn blue one by one as they
settle and sleep.

### What it demonstrates
- Velocity threshold: bodies sleep when all DOF velocities drop below
  `sleep_tolerance * dof_length`
- Countdown timer: `MIN_AWAKE = 10` consecutive sub-threshold steps required
- `nbody_awake` counts down as boxes settle at different times

### Scene

5 boxes (mass 1 kg each) dropped from heights 0.4–0.8 m (staggered so they
land at different times). Spacing is wide — no contact between boxes. Each is
an independent singleton. No impulse, no disturbance. Just gravity → settle
→ sleep.

### MJCF sketch

```xml
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box_0" pos="-2 0 0.4"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="box_1" pos="-1 0 0.5"><freejoint/>
      <geom .../>  </body>
    <body name="box_2" pos="0 0 0.6"><freejoint/> ...  </body>
    <body name="box_3" pos="1 0 0.7"><freejoint/> ...  </body>
    <body name="box_4" pos="2 0 0.8"><freejoint/> ...  </body>
  </worldbody>
</mujoco>
```

### HUD

```
Sleep Settle
────────────────────
  time           3.20 s
  awake          2 / 5
  ────────────────────
  box_0          Asleep
  box_1          Asleep
  box_2          Asleep
  box_3          Awake
  box_4          Awake
```

### Validation harness

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| All awake at start | `nbody_awake == 6` (5 + world) at t=0.1 | exact |
| All asleep after settle | `nbody_awake == 1` (world only) by t=6.0 | exact |
| Sleeping qvel = 0 | `|qvel[dof]| == 0.0` for all sleeping DOFs at t=6.5 | exact (bitwise) |
| Sleeping qacc = 0 | `|qacc[dof]| == 0.0` for all sleeping DOFs at t=6.5 | exact (bitwise) |

### Camera

- Target: (0, 0, 0.3)
- Distance: 6.0
- Azimuth: 0° (side view)
- Elevation: 25°

---

## Example 2: `wake-on-contact/` — Contact-Triggered Reactivation

### Concept
Contact between an awake body and a sleeping body wakes the sleeper.

### One-liner
A sleeping box is struck by a falling ball — the contact wakes the box.

### What it demonstrates
- `sleep="init"` policy: box starts asleep from timestep 0 (no settling wait)
- Wake-on-contact mechanism: `mj_wake_collision()` detects awake-vs-sleeping
  contact and wakes the sleeping tree
- Init-sleep freeze: box does not move under gravity while asleep

### Scene

A single box (mass 2 kg) resting on a ground plane with `sleep="init"` — it
starts asleep (blue). A sphere (mass 0.5 kg) is positioned 2.0 m above. The
sphere free-falls under gravity and strikes the box at approximately t = 0.64 s.
The box turns orange on impact.

### MJCF sketch

```xml
<mujoco>
  <option timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 0.15" sleep="init">
      <freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="2"/>
    </body>
    <body name="ball" pos="0 0 2.0">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="0.5"/>
    </body>
  </worldbody>
</mujoco>
```

### HUD

```
Wake-on-Contact
────────────────────
  time           0.70 s
  box            Awake    ← woke on contact
  ball           Awake
  contacts       2
  ────────────────────
  impact time    0.64 s
```

### Validation harness

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Box starts asleep | `sleep_state(box) == Asleep` at t=0.0 | exact |
| Ball starts awake | `sleep_state(ball) == Awake` at t=0.0 | exact |
| Box frozen while asleep | box z-position == 0.15 at t=0.3 (no gravity drift) | 1e-12 |
| Box wakes on impact | `sleep_state(box) == Awake` at t=0.7 | exact |
| Both re-sleep | `nbody_awake == 1` by t=14.0 | exact |

### Camera

- Target: (0, 0, 0.5)
- Distance: 4.0
- Azimuth: 30°
- Elevation: 20°

---

## Example 3: `island-groups/` — Independent Constraint Islands

### Concept
Constraint islands are independent — disturbing one doesn't affect the other.

### One-liner
Two separated stacks form independent islands; disturbing one leaves the
other asleep.

### What it demonstrates
- Island discovery: DFS flood-fill groups coupled bodies into islands
- Island independence: each island's sleep state is evaluated atomically
- Selective wake: applying force to one island wakes only that island

### Scene

Two stacks of 3 boxes each, placed 3.0 m apart on a ground plane. Stack A at
x = -1.5, Stack B at x = +1.5. While settling (awake), each stack forms its
own constraint island via inter-box contacts. Once all boxes sleep, islands
disappear (`nisland == 0` — sleeping trees are excluded from island discovery).

At **t = 7.0 s**, apply a lateral `xfrc_applied` to the top box of Stack A.
Stack A's island wakes (turns orange); Stack B remains asleep (blue).

### MJCF sketch

```xml
<mujoco>
  <option timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1"/>
    <!-- Stack A -->
    <body name="a1" pos="-1.5 0 0.15"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
    <body name="a2" pos="-1.5 0 0.45"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
    <body name="a3" pos="-1.5 0 0.75"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
    <!-- Stack B -->
    <body name="b1" pos="1.5 0 0.15"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
    <body name="b2" pos="1.5 0 0.45"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
    <body name="b3" pos="1.5 0 0.75"><freejoint/>
      <geom type="box" size="0.15 0.15 0.15" mass="1"/></body>
  </worldbody>
</mujoco>
```

### HUD

```
Island Groups
────────────────────
  time           7.50 s
  islands        1
  awake          3 / 6
  ────────────────────
  Stack A
    a1           Awake
    a2           Awake
    a3           Awake    <- impulse
  Stack B
    b1           Asleep
    b2           Asleep
    b3           Asleep
```

### Validation harness

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| All asleep after settle | `nbody_awake == 1` by t=6.0 | exact |
| Stack A wakes after impulse | all 3 Stack A bodies `Awake` at t=7.1 | exact |
| Stack B stays asleep | all 3 Stack B bodies `Asleep` at t=7.1 | exact |
| Island count pre-disturb | `nisland == 0` when all asleep | exact |
| Stack A re-sleeps | Stack A bodies `Asleep` by t=14.0 | exact |

### Camera

- Target: (0, 0, 0.4)
- Distance: 6.0
- Azimuth: 20°
- Elevation: 25°

---

## Example 4: `stress-test/` — Headless Validation (IMPLEMENTED)

### One-liner
18 automated checks covering sleep threshold, wake triggers, island
decomposition, countdown mechanics, and policy flags.

### What it demonstrates
Comprehensive headless validation of the sleep/wake subsystem. No window,
no visuals — pure physics verification. Each check targets a specific
mechanism in the 1,415-LOC island module.

### Test matrix

| # | Check | Setup | Condition | Tolerance |
|---|-------|-------|-----------|-----------|
| 1 | Sleep after threshold | 1 box on plane, settle | `sleep_state == Asleep` after settle | exact |
| 2 | Sleeping qvel = 0 | sleeping box | `qvel[dof_adr..dof_adr+6] == 0.0` bitwise | exact |
| 3 | Sleeping qacc = 0 | sleeping box | `qacc[dof_adr..dof_adr+6] == 0.0` bitwise | exact |
| 4 | Sleeping qfrc_applied = 0 | sleeping box | `qfrc_applied[dof_adr..dof_adr+6]` zeroed by sleep | exact |
| 5 | Countdown duration | 1 box, count steps from sub-threshold to sleep | steps >= MIN_AWAKE (10) | exact |
| 6 | Wake on contact | sleeping box + dropped ball | box wakes on first contact step | exact |
| 7 | Wake on equality | 2 boxes, weld constraint, one sleeping one awake | sleeping box wakes | exact |
| 8 | Wake on xfrc_applied | sleeping box, apply force | box wakes | exact |
| 9 | Wake cascade | 3 boxes in a row, poke end box | middle box wakes via contact chain | exact |
| 10 | No island for singletons | 2 isolated boxes on plane | both `tree_island == -1` (singletons, no shared island) | exact |
| 11 | Shared island | 2 stacked boxes | `tree_island[a] == tree_island[b]` and >= 0 | exact |
| 12 | Island count | 2 stacks of 2 boxes each | `nisland == 2` | exact |
| 13 | Selective wake | 2 stacks, poke one | only poked stack awake | exact |
| 14 | Sleep countdown resets | box nearly asleep, velocity spike | countdown restarts from `-(1+MIN_AWAKE)` | exact |
| 15 | Sleep policy Never | box with `sleep="never"` | never sleeps regardless of velocity | exact |
| 16 | ENABLE_SLEEP disabled | clear the flag | no bodies ever sleep | exact |
| 17 | Narrowphase skip | 2 sleeping boxes side by side | zero contacts between them | exact |
| 18 | nbody_awake bookkeeping | various states | `nbody_awake` matches count of non-`Asleep` bodies (i.e. `Awake` + `Static`) | exact |

### Run

```sh
cargo run -p example-sleep-wake-stress-test --release
```

---

## Directory structure

```
examples/fundamentals/sim-cpu/sleep-wake/
├── SLEEP_WAKE_SPEC.md          # this file
├── README.md                   # group overview (museum-plaque style)
├── sleep-settle/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── wake-on-contact/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── island-groups/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── stress-test/                # IMPLEMENTED
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

### Package names

| Example | Package name |
|---------|-------------|
| sleep-settle | `example-sleep-wake-settle` |
| wake-on-contact | `example-sleep-wake-contact` |
| island-groups | `example-sleep-wake-islands` |
| stress-test | `example-sleep-wake-stress-test` |

---

## Implementation order

1. **stress-test** — DONE. 18/18 PASS.
2. **sleep-settle** — pure sleep mechanism, simplest visual
3. **wake-on-contact** — single wake trigger
4. **island-groups** — island independence

---

## Visual implementation notes

### Sleep-state color coding

Each visual example needs a system that reads `data.body_sleep_state[body_id]`
each frame and swaps the entity's material:

```rust
/// Component tagging each geom entity with its owning body index.
/// Attached via `spawn_model_geoms_with` callback during setup.
#[derive(Component)]
struct GeomBodyId(usize);

fn update_sleep_colors(
    data: Res<PhysicsData>,
    mats: Res<SleepMaterials>,          // Resource holding awake/asleep handles
    mut query: Query<(&GeomBodyId, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    for (body_id, mut mat_handle) in &mut query {
        match data.0.body_sleep_state[body_id.0] {
            SleepState::Awake  => *mat_handle = MeshMaterial3d(mats.awake.clone()),
            SleepState::Asleep => *mat_handle = MeshMaterial3d(mats.asleep.clone()),
            SleepState::Static => {} // ground stays grey
        }
    }
}
```

Each geom entity already gets a `ModelGeomIndex(geom_id)` component from the
spawner. We use `spawn_model_geoms_with` to additionally tag each entity with
`GeomBodyId(model.geom_body[geom_id])` via the callback:

```rust
spawn_model_geoms_with(
    &mut commands, &mut meshes, &mut materials, &model, &data, &overrides,
    |cmd, geom_id, _name| { cmd.insert(GeomBodyId(model.geom_body[geom_id])); },
);
```

### Color palette

| State | Color | MetalPreset |
|-------|-------|-------------|
| Awake | warm orange `(0.95, 0.45, 0.15)` | `Anodized(Color::srgb(0.95, 0.45, 0.15))` |
| Asleep | steel blue `(0.35, 0.50, 0.70)` | `Anodized(Color::srgb(0.35, 0.50, 0.70))` |
| Static | dark grey `(0.25, 0.25, 0.25)` | `CastIron` |
