# sim-bevy Examples API Spec

**Status:** Approved — Changes 1–4 DONE, Changes 5–6 pending
**Date:** 2026-03-25
**Scope:** sim-bevy API improvements to support all 12 Track 1 example domains

## Problem

We have 8 examples across 3 joint domains (hinge, slide, ball). They work, but
~39% of each example is copy-paste boilerplate. Before building 9 more domains
(sensors, actuators, muscles, solvers, integrators, equality-constraints,
contact-tuning, inverse-dynamics, energy-momentum, urdf-loading), we need to
fix the infrastructure so every future example builds on good APIs.

### Issues (ranked by frequency)

1. **Material override hack** — every example defines a custom `MaterialOverrides`
   resource, a one-shot `apply_materials` system in Update that runs once then
   deletes itself. ~30 lines repeated 8 times.

2. **Raw array indexing** — `data.qpos[model.jnt_qpos_adr[0] + k]` everywhere.
   Error-prone, opaque to beginners, no name-based access.

3. **Validation boilerplate** — every example defines its own `Validation`
   resource, `Default` impl, and `diagnostics()` system. Only the tracker
   types and tolerances differ.

4. **No text HUD** — sensors, solvers, integrators, energy-momentum all need
   scalar readouts on screen. Currently everything goes to `println!`.

5. **Sensor viz is dead code** — 5 visual types in a kitchen-sink struct, zero
   consumers. Needs full redesign.

6. **No multi-scene support** — solvers and integrators examples need 3–4
   parallel physics scenes. No infrastructure for this.

## Design Principles

- **Transparent, not magic.** Examples should show the Bevy ECS architecture
  clearly. No opaque builders that hide the system graph.
- **One concept per API.** Each helper does one thing. Compose them, don't nest.
- **Beginners can read it.** A student seeing Bevy + physics for the first time
  should understand the data flow.
- **Zero regression.** Existing examples must migrate cleanly.

---

## Change 1: Material Overrides at Spawn Time

**Stress test result: PASS — no changes from v1.**

### Current (bad)

```rust
// In setup():
spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);
let steel = materials.add(MetalPreset::PolishedSteel.material());
commands.insert_resource(MaterialOverrides { rod: steel, ... });

// Separate system in Update:
fn apply_materials(mut commands: Commands, model: Option<Res<PhysicsModel>>,
    overrides: Option<Res<MaterialOverrides>>,
    mut query: Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>) {
    let (Some(model), Some(ovr)) = (model, overrides) else { return };
    override_geom_materials_by_name(&model, &[("rod", ovr.rod.clone()), ...], &mut query);
    commands.remove_resource::<MaterialOverrides>();
}
```

**30 lines per example. Runs in Update but is logically Startup.**

### Proposed

Add a `material_overrides` parameter to `spawn_model_geoms`:

```rust
/// Material override: geom name → material handle.
pub type GeomMaterialOverride<'a> = (&'a str, Handle<StandardMaterial>);

pub fn spawn_model_geoms(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: &Model,
    data: &Data,
    overrides: &[GeomMaterialOverride<'_>],
)
```

Inside `spawn_model_geoms`, build a `HashMap<usize, Handle<StandardMaterial>>`
from `overrides` using `model.geom_name_to_id` (already exists, O(1) lookup).
For each geom, check the map before falling back to the MJCF-rgba default.

**Usage:**

```rust
let steel = materials.add(MetalPreset::PolishedSteel.material());
let iron = materials.add(MetalPreset::CastIron.material());

spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data, &[
    ("rod", steel),
    ("bracket", iron),
]);
```

**For trail attachment and other per-entity customization**, add a callback
variant:

```rust
pub fn spawn_model_geoms_with<F>(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: &Model,
    data: &Data,
    overrides: &[GeomMaterialOverride<'_>],
    per_entity: F,
) where F: FnMut(&mut EntityCommands, usize, &str)
```

The callback receives `(entity_commands, geom_id, geom_name)` for each
spawned geom, letting examples attach `TrailGizmo` or custom components
without a separate system.

### Verified

- `model.geom_name_to_id: HashMap<String, usize>` exists and is populated
  during MJCF building.
- The callback pattern is already proven in production by
  `override_geom_materials_by_name_with`.
- 27 call sites total (26 examples + 1 test), all internal. Mechanical
  migration: add `&[]` to each call.

### Migration

- Add `overrides` parameter to `spawn_model_geoms` (breaking change).
  Existing callers add `&[]` for no overrides.
- Delete `MaterialOverrides` resources and `apply_materials` systems from all
  8 examples.
- Remove `apply_materials` from Update system chains.
- Net savings: ~30 lines per example × 8 = ~240 lines removed.

---

## Change 2: Named Accessors for Data

**Stress test result: PASS — with one correction. Methods live on `Data`
taking `&Model`, not `Model` taking `&Data`. Precedent: `data.qld_diag(&model, i)`.
The state lives on Data, the metadata on Model.**

### Current (bad)

```rust
let adr = model.jnt_qpos_adr[0];
let (w, x, y, z) = (data.qpos[adr], data.qpos[adr+1], data.qpos[adr+2], data.qpos[adr+3]);
```

### Proposed

Add accessor methods to `Data` (in sim-core) and name-lookup methods to
`Model`:

```rust
impl Model {
    // ── Name lookups (delegate to existing *_name_to_id HashMaps) ────

    /// Look up joint ID by name. Returns None if not found.
    pub fn joint_id(&self, name: &str) -> Option<usize>;

    /// Look up sensor ID by name. Returns None if not found.
    pub fn sensor_id(&self, name: &str) -> Option<usize>;

    /// Look up actuator ID by name. Returns None if not found.
    pub fn actuator_id(&self, name: &str) -> Option<usize>;

    /// Look up body ID by name. Returns None if not found.
    pub fn body_id(&self, name: &str) -> Option<usize>;

    /// Look up geom ID by name. Returns None if not found.
    pub fn geom_id(&self, name: &str) -> Option<usize>;
}

impl Data {
    // ── Joint state accessors ───────────────────────────────────────

    /// Joint qpos slice (length: 1 for hinge/slide, 4 for ball, 7 for free).
    /// Uses `model.jnt_qpos_adr` + `model.jnt_type[id].nq()`.
    pub fn joint_qpos(&self, model: &Model, joint_id: usize) -> &[f64];
    pub fn joint_qpos_mut(&mut self, model: &Model, joint_id: usize) -> &mut [f64];

    /// Joint qvel slice (length: 1 for hinge/slide, 3 for ball, 6 for free).
    /// Uses `model.jnt_dof_adr` + `model.jnt_type[id].nv()`.
    pub fn joint_qvel(&self, model: &Model, joint_id: usize) -> &[f64];
    pub fn joint_qvel_mut(&mut self, model: &Model, joint_id: usize) -> &mut [f64];

    // ── Sensor state accessor ───────────────────────────────────────

    /// Sensor data slice (length = `model.sensor_dim[sensor_id]`).
    /// Uses `model.sensor_adr` + `model.sensor_dim`.
    pub fn sensor_data(&self, model: &Model, sensor_id: usize) -> &[f64];
}
```

**Usage:**

```rust
// Before:
let adr = model.jnt_qpos_adr[0];
let angle = data.qpos[adr];

// After:
let angle = data.joint_qpos(&model, 0)[0];

// By name:
let jid = model.joint_id("elbow").unwrap();
let angle = data.joint_qpos(&model, jid)[0];

// Sensor by name:
let sid = model.sensor_id("gyro").unwrap();
let gyro = data.sensor_data(&model, sid); // &[f64] of length 3
```

### Verified

- All 5 `*_name_to_id` HashMaps exist on Model (plus site, tendon, eq, mesh,
  hfield — 10 total).
- `MjJointType::nq()` and `::nv()` already exist as `const fn` methods.
- `Model::joint_qpos0()` already uses the exact same pattern (slice from
  `jnt_qpos_adr` + `jnt_type.nq()`).
- `sensor_dim` is precomputed at model build time and stored in
  `model.sensor_dim[i]`. Fixed per sensor type (1/3/4/6), except User/Plugin
  which are set explicitly.
- Zero-cost: inline slice operations, no allocation.

---

## Change 3: Validation Harness

**Stress test result: REDESIGN — trait-based approach is not viable.
The 6 tracker types have incompatible `sample()` signatures. Use an
enum-based harness with closures for input extraction.**

**Implementation: DONE — Phase A.3 complete.**
Harness: `sim-bevy/src/examples.rs` (`ValidationHarness`, `validation_system`).
All 8 sim-cpu examples migrated. MaterialOverrides + apply_materials deleted.
Additions beyond spec: `skip_until()` builder for constraint-settling transients,
harness-level `display()` for consolidated per-second console output,
`reported()` accessor for sensor-accuracy co-reporting (spherical-pendulum).

### Current (bad)

Every example defines:
```rust
#[derive(Resource)]
struct Validation {
    period: PeriodTracker,
    energy: EnergyConservationTracker,
    reported: bool,
}
impl Default for Validation { ... }
fn diagnostics(model: Res<PhysicsModel>, data: Res<PhysicsData>,
    mut timer: ResMut<DiagTimer>, mut val: ResMut<Validation>) { ... }
```

~50 lines per example, identical structure, only tracker types differ.

### Why trait objects don't work

The 6 tracker types have fundamentally different `sample()` signatures:

| Tracker | sample() args |
|---------|--------------|
| `PeriodTracker` | `(value: f64, time: f64)` |
| `EnergyConservationTracker` | `(energy: f64)` |
| `EnergyMonotonicityTracker` | `(energy: f64)` |
| `LimitTracker` | `(value: f64, lo: f64, hi: f64)` |
| `QuaternionNormTracker` | `(w: f64, x: f64, y: f64, z: f64)` |
| `EquilibriumTracker` | `(value: f64, time: f64)` |

`check()` signatures also diverge (some take `expected + threshold_pct`,
others just `threshold`). Cannot unify behind a single trait.

### Proposed: Enum harness with closures

```rust
/// A validation entry: tracker + how to feed it + how to check it.
pub struct ValidationEntry {
    name: &'static str,
    tracker: TrackerKind,
    display: Option<fn(&Model, &Data) -> String>,
}

/// Enum wrapping the 6 tracker types. No trait objects, no allocation.
pub enum TrackerKind {
    Period {
        tracker: PeriodTracker,
        /// Extract (value, time) from simulation state.
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
    },
    EnergyConservation {
        tracker: EnergyConservationTracker,
        tolerance_pct: f64,
    },
    EnergyMonotonicity {
        tracker: EnergyMonotonicityTracker,
        tolerance_j: f64,
    },
    Limit {
        tracker: LimitTracker,
        /// Extract (value, lo, hi) from simulation state.
        sampler: fn(&Model, &Data) -> (f64, f64, f64),
        tolerance: f64,
    },
    QuatNorm {
        tracker: QuaternionNormTracker,
        /// Extract (w, x, y, z) from simulation state.
        sampler: fn(&Model, &Data) -> (f64, f64, f64, f64),
        tolerance: f64,
    },
    Equilibrium {
        tracker: EquilibriumTracker,
        /// Extract (value, time) from simulation state.
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance: f64,
    },
}
```

The harness owns timing and reporting logic (the actual shared boilerplate):

```rust
/// Resource: collects validation entries, handles timing, prints reports.
#[derive(Resource)]
pub struct ValidationHarness {
    entries: Vec<ValidationEntry>,
    report_time: f64,
    print_interval: f64,
    reported: bool,
    last_print: f64,
}

impl ValidationHarness {
    pub fn new() -> Self;
    pub fn report_at(mut self, time: f64) -> Self;
    pub fn print_every(mut self, interval: f64) -> Self;

    // ── Convenience builders (one per tracker kind) ─────────────────

    /// Track oscillation period. `sampler` returns (signal_value, sim_time).
    pub fn track_period(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
    ) -> Self;

    /// Track energy conservation (reads data.energy_kinetic + energy_potential).
    pub fn track_energy(mut self, tolerance_pct: f64) -> Self;

    /// Track energy monotonic decrease (damped systems).
    pub fn track_energy_monotonic(mut self, tolerance_j: f64) -> Self;

    /// Track joint limit enforcement. `sampler` returns (value, lo, hi).
    pub fn track_limit(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64, f64),
        tolerance: f64,
    ) -> Self;

    /// Track quaternion norm. `sampler` returns (w, x, y, z).
    pub fn track_quat_norm(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64, f64, f64),
        tolerance: f64,
    ) -> Self;

    /// Track equilibrium convergence. `sampler` returns (value, sim_time).
    pub fn track_equilibrium(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance: f64,
    ) -> Self;
}
```

**System:**

```rust
/// Bevy system: sample all trackers, print periodic diagnostics, print report.
pub fn validation_system(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut harness: ResMut<ValidationHarness>,
) {
    let time = data.time;

    // Sample each tracker via its closure
    for entry in &mut harness.entries {
        match &mut entry.tracker {
            TrackerKind::Period { tracker, sampler, .. } => {
                let (val, t) = sampler(&model, &data);
                tracker.sample(val, t);
            }
            TrackerKind::EnergyConservation { tracker, .. } => {
                tracker.sample(data.energy_kinetic + data.energy_potential);
            }
            // ... etc for each variant
        }
    }

    // Periodic console output
    if time - harness.last_print >= harness.print_interval {
        harness.last_print = time;
        // Print display_line for each entry
    }

    // Final report
    if time >= harness.report_time && !harness.reported {
        harness.reported = true;
        // Collect Check results, call print_report()
    }
}
```

**Usage in examples:**

```rust
fn main() {
    App::new()
        // ...
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .track_energy(0.5)
                .track_period("θ", |_m, d| (d.joint_qpos(_m, 0)[0], d.time), analytical_T(), 2.0)
                .track_limit("slide", |m, d| {
                    let pos = d.joint_qpos(m, 0)[0];
                    let (lo, hi) = m.jnt_range[0];
                    (pos, lo, hi)
                }, 0.001)
        )
        .add_systems(PostUpdate, validation_system)
        // ...
}
```

**Note on `track_energy`**: Energy trackers always read
`data.energy_kinetic + data.energy_potential` — no sampler closure needed.
This is hardcoded in the harness (not configurable).

### Why closures, not trait objects

- `fn(&Model, &Data) -> T` is a function pointer — zero-size, no allocation,
  no `Box<dyn>`, no lifetime issues.
- Each tracker variant knows its exact input shape at compile time.
- Enum dispatch is a single match — no vtable indirection.
- The harness is not extensible by third parties (no plugin ecosystem). All 6
  tracker types are known at compile time. Enum is the right tool.

---

## Change 4: Text HUD

**Stress test result: PASS — with corrections. Must use Node-based Bevy UI
(not Text2d). Requires adding `bevy_ui` and `bevy_text` features to
sim-bevy's Cargo.toml.**

### Problem

9 of 12 domains need scalar readouts on screen:
- sensors: sensor values table
- actuators: ctrl/act/force
- muscles: activation, FL/FV
- solvers: iterations, cost, residual
- integrators: energy per integrator
- equality-constraints: violation magnitudes
- contact-tuning: friction, forces
- inverse-dynamics: torque values
- energy-momentum: KE, PE, total, momentum

Currently all go to `println!`. On-screen text is essential for visual
examples.

### Proposed

A lightweight HUD resource in sim-bevy's `examples` module:

```rust
/// A single line of HUD text.
pub struct HudLine {
    pub label: String,
    pub value: String,
}

/// Resource: list of HUD lines to display this frame.
#[derive(Resource, Default)]
pub struct PhysicsHud {
    lines: Vec<HudLine>,
}

impl PhysicsHud {
    /// Clear all lines (call at start of each frame).
    pub fn clear(&mut self);

    /// Add a labeled scalar value.
    pub fn scalar(&mut self, label: &str, value: f64, fmt: &str);

    /// Add a labeled 3D vector.
    pub fn vec3(&mut self, label: &str, v: &[f64], fmt: &str);

    /// Add a labeled quaternion.
    pub fn quat(&mut self, label: &str, q: &[f64]);

    /// Add a section header.
    pub fn section(&mut self, title: &str);

    /// Add a raw pre-formatted line.
    pub fn raw(&mut self, text: String);
}
```

**Rendering system:**

```rust
/// Marker component for the HUD text entity.
#[derive(Component)]
pub struct HudText;

/// Bevy system: render PhysicsHud as Node-based UI text in the top-left corner.
pub fn render_physics_hud(
    hud: Res<PhysicsHud>,
    mut query: Query<&mut Text, With<HudText>>,
) {
    // Join all HudLines into a single string, update the Text component.
}
```

**Spawn helper** (called once in Startup):

```rust
/// Spawn the HUD overlay: a dark semi-transparent panel with monospace text.
pub fn spawn_physics_hud(commands: &mut Commands) {
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            padding: UiRect::all(Val::Px(8.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
        ZIndex(999),
    )).with_children(|parent| {
        parent.spawn((
            HudText,
            Text::new(""),
            TextFont { font_size: 14.0, ..default() },
            TextColor(Color::srgb(0.0, 1.0, 0.0)),
        ));
    });
}
```

**Usage:**

```rust
fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Sensors");
    let sid = model.sensor_id("gyro").unwrap();
    let g = data.sensor_data(&model, sid);
    hud.vec3("Gyro", g, "{:+.3}");
    hud.scalar("Clock", data.time, "{:.2}s");
}
```

### Cargo.toml Change Required

```toml
# sim/L1/bevy/Cargo.toml — add these features:
bevy = { workspace = true, default-features = false, features = [
    # ... existing features ...
    "bevy_ui",
    "bevy_text",
] }
```

### Implementation Notes

- **Node-based UI**, not Text2d. Text2d is world-space — wrong for a HUD.
  Node-based UI is screen-space, handles window resizing, and is the
  standard Bevy approach for overlays.
- Bevy 0.18 includes a default embedded font (Roboto Flex). No custom font
  file needed.
- Toggle: `ViewerConfig::show_hud` (default true for examples).
- The HUD is an examples utility, not a core sim-bevy feature. It lives in
  `examples.rs` alongside `DiagTimer` and `spawn_example_camera`.

---

## Change 5: Sensor Visualization Redesign

**Stress test result: PASS — with two clarifications on arrow rotation
and dual-object field names.**

### Current (dead code)

```rust
pub enum SensorVisualType { Imu, ForceTorque, Touch, Rangefinder, Magnetometer }

pub struct SensorVisualData {
    pub sensor_type: SensorVisualType,
    pub position: Point3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub force: Option<Vector3<f64>>,    // only for ForceTorque
    pub torque: Option<Vector3<f64>>,   // only for ForceTorque
    pub is_active: bool,                // only for Touch
    pub ray: Option<(Vector3<f64>, f64)>, // only for Rangefinder
    pub magnetic_field: Option<Vector3<f64>>, // only for Magnetometer
}
```

Problems: kitchen-sink struct, 5 types for 38 sensors, manual push model,
zero consumers.

### Proposed

Replace with auto-discovery from `Model` + proper enum:

```rust
/// What to draw for a sensor reading.
pub enum SensorGizmo {
    /// RGB coordinate frame at site (Accelerometer, Gyro, BallQuat).
    Frame {
        position: Vector3<f64>,
        orientation: Matrix3<f64>,
    },
    /// Arrow from site in a direction (Force, Torque, Velocimeter, etc.).
    /// `direction` is in **world frame** (pre-rotated by the update system).
    Arrow {
        origin: Vector3<f64>,
        direction: Vector3<f64>,
        color: Color,
    },
    /// Scalar highlight (Touch — sphere that glows when active).
    Highlight {
        position: Vector3<f64>,
        intensity: f64,  // 0.0 = inactive, 1.0 = max
    },
    /// Ray with hit point (Rangefinder).
    Ray {
        origin: Vector3<f64>,
        direction: Vector3<f64>,
        distance: f64,
    },
    /// Distance line between two points (GeomDist, GeomFromTo).
    DistanceLine {
        from: Vector3<f64>,
        to: Vector3<f64>,
        distance: f64,
    },
    /// No gizmo — scalar sensors displayed only on HUD.
    HudOnly,
}

/// Auto-populated sensor visualization resource.
#[derive(Resource, Default)]
pub struct SensorVisualization {
    entries: Vec<SensorVizEntry>,
}

pub struct SensorVizEntry {
    pub sensor_id: usize,
    pub name: String,
    pub sensor_type: MjSensorType,
    pub gizmo: SensorGizmo,
    pub value: Vec<f64>,  // raw sensor data this frame
}
```

**Auto-update system:**

```rust
/// System: reads model.sensor_* metadata + data.sensordata, populates
/// SensorVisualization with world-frame gizmos.
pub fn update_sensor_visualization(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut viz: ResMut<SensorVisualization>,
) {
    viz.entries.clear();
    for sensor_id in 0..model.nsensor {
        let value = data.sensor_data(&model, sensor_id).to_vec();
        let gizmo = sensor_type_to_gizmo(&model, &data, sensor_id, &value);
        viz.entries.push(SensorVizEntry {
            sensor_id,
            name: model.sensor_name[sensor_id].clone().unwrap_or_default(),
            sensor_type: model.sensor_type[sensor_id],
            gizmo,
            value,
        });
    }
}
```

### Arrow rotation: local-frame vs world-frame sensors

The `sensor_type_to_gizmo` function must handle two cases for vector sensors:

**Local-frame sensors** (Accelerometer, Gyro, Velocimeter, Force, Torque,
Magnetometer): The sensor reading is in the **site's local frame**. To get
the world-frame arrow direction, rotate by `site_xmat`:

```rust
let site_id = model.sensor_objid[sensor_id];
let site_mat = &data.site_xmat[site_id];
let local_vec = Vector3::new(value[0], value[1], value[2]);
let world_vec = site_mat * local_vec;
```

**World-frame sensors** (FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc
when `sensor_reftype == MjObjectType::None`): The reading is already in
world coordinates. Do NOT rotate:

```rust
let world_vec = Vector3::new(value[0], value[1], value[2]);
```

**Relative-frame sensors** (Frame* when `sensor_reftype != None`): The
reading is in the reference object's frame. Rotate by the reference frame's
rotation matrix (inverse):

```rust
let ref_mat = get_object_xmat(&data, model.sensor_reftype[i], model.sensor_refid[i]);
let world_vec = ref_mat * local_vec;
```

### Dual-object sensors (GeomDist, GeomNormal, GeomFromTo)

These use `sensor_objtype/objid` for the first object and
`sensor_reftype/refid` for the second object (reusing the reference fields,
NOT separate `obj1id`/`obj2id` fields). The contact points from the
`geom_distance` computation are already in world coordinates — use directly
for the `DistanceLine` gizmo.

### Sensor type → gizmo mapping

| Sensor Type | Gizmo | Frame handling |
|-------------|-------|----------------|
| Accelerometer, Gyro | Frame | Local → rotate by site_xmat |
| Force, Torque | Arrow | Local → rotate by site_xmat |
| Velocimeter | Arrow | Local → rotate by site_xmat |
| Magnetometer | Arrow | Local → rotate by site_xmat |
| FrameLinVel, FrameAngVel | Arrow | World if no ref, else rotate by ref_xmat |
| FrameLinAcc, FrameAngAcc | Arrow | World if no ref, else rotate by ref_xmat |
| Touch | Highlight | Intensity = sensor value (force magnitude) |
| Rangefinder | Ray | Along site Z-axis (local), rotate by site_xmat |
| GeomDist | DistanceLine | World coords from geom_distance |
| GeomFromTo | DistanceLine | World coords (6D: from + to points) |
| GeomNormal | Arrow | World coords from geom_distance |
| BallQuat | Frame | Quaternion → rotation matrix |
| FramePos | Highlight | World position |
| FrameQuat | Frame | Quaternion → rotation matrix |
| FrameXAxis, FrameYAxis, FrameZAxis | Arrow | World direction |
| SubtreeCom | Highlight | World position |
| JointPos, JointVel, Clock, etc. | HudOnly | Scalar → text only |
| ActuatorPos, ActuatorVel, ActuatorFrc | HudOnly | Scalar → text only |
| TendonPos, TendonVel | HudOnly | Scalar → text only |
| JointLimitFrc, TendonLimitFrc | HudOnly | Scalar → text only |
| JointActuatorFrc | HudOnly | Scalar → text only |
| SubtreeLinVel, SubtreeAngMom | Arrow | World vector at subtree COM |

### Migration

- Delete `SensorVisualType`, old `SensorVisualData`, old `SensorVisualization`.
- Replace `draw_sensors` with `draw_sensor_gizmos`.
- No external consumers to migrate (zero users of old API).

---

## Change 6: Multi-Scene Support

**Stress test result: REDESIGN — two blockers found.**

**Blocker 1:** A shared accumulator with scenes at different timesteps gives
the faster scene more steps per frame. For comparison examples (solvers,
integrators), this silently invalidates the comparison.

**Blocker 2:** The existing `sync_geom_transforms` queries
`Query<(&ModelGeomIndex, &mut Transform)>` and reads from the global
`Res<PhysicsData>`. A multi-scene sync needs
`Query<(&PhysicsSceneId, &ModelGeomIndex, &mut Transform)>` reading from
per-scene data. These query signatures are incompatible.

### Proposed: Separate Plugin

Multi-scene is a **separate code path**, not an extension of single-scene.
Users explicitly choose one or the other. Only 2 of 12 domains need it
(solvers, integrators).

```rust
/// A physics scene: model + data + its own time accumulator.
pub struct PhysicsScene {
    pub model: Model,
    pub data: Data,
    pub label: String,
    accumulator: f64,
}

/// Resource holding multiple physics scenes.
#[derive(Resource, Default)]
pub struct PhysicsScenes {
    scenes: Vec<PhysicsScene>,
}

impl PhysicsScenes {
    pub fn add(&mut self, label: impl Into<String>, model: Model, data: Data) -> usize;
    pub fn get(&self, id: usize) -> Option<&PhysicsScene>;
    pub fn get_mut(&mut self, id: usize) -> Option<&mut PhysicsScene>;
    pub fn len(&self) -> usize;
    pub fn iter(&self) -> impl Iterator<Item = &PhysicsScene>;
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut PhysicsScene>;
}
```

**Tag component:**

```rust
/// Identifies which physics scene an entity belongs to.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PhysicsSceneId(pub usize);
```

**Lockstep stepping** (for comparison scenes):

```rust
/// Step all scenes in lockstep: each scene advances by the same wall-clock
/// time, regardless of its internal timestep. This ensures fair comparison.
pub fn step_scenes_lockstep(
    mut scenes: ResMut<PhysicsScenes>,
    time: Res<Time>,
) {
    let wall_dt = time.delta_secs_f64();
    for scene in scenes.iter_mut() {
        scene.accumulator += wall_dt;
        let dt = scene.model.timestep;
        let mut steps = 0;
        while scene.accumulator >= dt && steps < 200 {
            let _ = scene.data.step(&scene.model);
            scene.accumulator -= dt;
            steps += 1;
        }
        if steps > 0 {
            let _ = scene.data.forward(&scene.model);
        }
    }
}
```

Each scene has its **own accumulator**. A scene with `dt=0.001` steps 5x
per frame compared to `dt=0.005`, but both advance the same wall-clock
duration. For comparison scenes (solvers), use the **same timestep** across
all scenes so they take the same number of steps.

**Multi-scene sync:**

```rust
/// Sync geom transforms for multi-scene entities.
/// Reads from PhysicsScenes (not Res<PhysicsData>).
pub fn sync_scene_geom_transforms(
    scenes: Res<PhysicsScenes>,
    mut geoms: Query<(&PhysicsSceneId, &ModelGeomIndex, &mut Transform)>,
) {
    for (scene_id, geom_idx, mut transform) in &mut geoms {
        if let Some(scene) = scenes.get(scene_id.0) {
            let idx = geom_idx.0;
            if idx < scene.data.geom_xpos.len() {
                transform.translation = vec3_from_vector(&scene.data.geom_xpos[idx]);
                transform.rotation = quat_from_physics_matrix(&scene.data.geom_xmat[idx]);
            }
        }
    }
}
```

**Spawning with offset:**

```rust
/// Spawn geoms for one scene, offset in world space for side-by-side layout.
/// Tags each entity with PhysicsSceneId.
pub fn spawn_scene_geoms(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scene_id: usize,
    scene: &PhysicsScene,
    offset: Vec3,
    overrides: &[GeomMaterialOverride<'_>],
)
```

**Plugin:**

```rust
/// Plugin for multi-scene physics. Mutually exclusive with the single-scene
/// PhysicsModel/PhysicsData resource pattern.
pub struct MultiScenePlugin;

impl Plugin for MultiScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, step_scenes_lockstep
            .run_if(resource_exists::<PhysicsScenes>))
        .add_systems(PostUpdate, sync_scene_geom_transforms
            .run_if(resource_exists::<PhysicsScenes>));
    }
}
```

**Usage (solvers example):**

```rust
fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin { ... }))
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(MultiScenePlugin)  // NOT ModelDataPlugin
        .add_systems(Startup, setup)
        .add_systems(PostUpdate, diagnostics)
        .run();
}

fn setup(mut commands: Commands, ...) {
    let mut scenes = PhysicsScenes::default();
    let spacing = 2.0;

    for (i, solver) in [SolverType::PGS, SolverType::CG, SolverType::Newton].iter().enumerate() {
        let mut model = load_model(MJCF).unwrap();
        model.solver_type = *solver;
        model.enableflags |= ENABLE_ENERGY;
        let data = model.make_data();
        let id = scenes.add(format!("{solver:?}"), model, data);
        let offset = Vec3::new(i as f32 * spacing, 0.0, 0.0);
        spawn_scene_geoms(&mut commands, &mut meshes, &mut materials,
            id, &scenes.get(id).unwrap(), offset, &[...]);
    }

    commands.insert_resource(scenes);
    spawn_example_camera(&mut commands, ...);
}
```

### Backward Compatibility

Single-scene examples (10 of 12 domains) continue using `PhysicsModel` /
`PhysicsData` / `sync_geom_transforms` unchanged. Multi-scene is a separate
plugin for solvers and integrators only. No migration needed for existing
examples.

---

## Domain Requirements Matrix

| Domain | C1 | C2 | C3 | C4 | C5 | C6 |
|--------|----|----|----|----|----|----|
| | Materials | Accessors | Validation | HUD | Sensor Viz | Multi-Scene |
| hinge-joint (existing) | Yes | Yes | Yes | — | — | — |
| slide-joint (existing) | Yes | Yes | Yes | — | — | — |
| ball-joint (existing) | Yes | Yes | Yes | — | — | — |
| **sensors/** | Yes | **sensor_data()** | Yes | **Yes** | **Yes** | — |
| **actuators/** | Yes | **actuator_id()** | Yes | **Yes** | — | — |
| **muscles/** | Yes | Yes | Yes | **Yes** | — | — |
| **solvers/** | Yes | — | Yes | **Yes** | — | **Yes** |
| **integrators/** | Yes | — | Yes | **Yes** | — | **Yes** |
| **equality-constraints/** | Yes | Yes | Yes | **Yes** | — | — |
| **contact-tuning/** | Yes | Yes | Yes | **Yes** | — | — |
| **inverse-dynamics/** | Yes | **joint_qpos_mut()** | Yes | **Yes** | — | — |
| **energy-momentum/** | Yes | — | Yes | **Yes** | — | — |
| **urdf-loading/** | Yes | Yes | Yes | **Yes** | — | — |

Every domain uses Changes 1–3. This confirms they're foundational.

---

## Implementation Order

### Phase A: Foundation (Changes 1–3)

These affect every example. Do them first, migrate existing examples.

1. **Change 2: Named accessors** — DONE (commit `00d490a`)
   - Model: joint_id(), sensor_id(), actuator_id(), body_id(), geom_id(),
     site_id(), tendon_id()
   - Data: joint_qpos(), joint_qpos_mut(), joint_qvel(), joint_qvel_mut(),
     sensor_data()
   - 13 integration tests, all passing

2. **Change 1: Material overrides at spawn** — DONE (commit `00d490a`)
   - spawn_model_geoms() takes &[GeomMaterialOverride] overrides parameter
   - spawn_model_geoms_with() callback variant added
   - All 27 call sites migrated, 1954 tests pass
   - MaterialOverrides + apply_materials deleted from all 8 examples (in Change 3)

3. **Change 3: Validation harness** — DONE (2026-03-25)
   - ValidationHarness + validation_system in sim-bevy/src/examples.rs
   - 6 tracker types (Period, EnergyConservation, EnergyMonotonicity, Limit, QuatNorm, Equilibrium)
   - Extensions: skip_until(), display(), reported()
   - All 8 pre-sensor examples migrated, MaterialOverrides + apply_materials deleted

### Phase B: Visualization (Changes 4–5)

Needed before sensors example.

4. **Change 4: Text HUD** — **DONE** (2026-03-25)
   - Added bevy_ui + bevy_text features to sim-bevy Cargo.toml
   - PhysicsHud resource (scalar/vec3/quat/section/raw), HudText component,
     render_physics_hud system, spawn_physics_hud helper
   - 8 unit tests (formatting, clear, composition)
   - All 9 sensor examples updated with per-example update_hud systems
   - ViewerConfig::show_hud toggle (default true, hides HUD node when false)

5. **Change 5: Sensor viz redesign** — NEXT
   - Delete old dead code (SensorVisualType, SensorVisualData)
   - New SensorVisualization with auto-discovery + draw_sensor_gizmos

### Phase C: Multi-Scene (Change 6)

Needed before solvers/integrators examples.

6. **Change 6: Multi-scene** — pending
   - New MultiScenePlugin alongside existing single-scene pattern

---

## What This Spec Does NOT Cover

- **Muscle/tendon visualization** — the existing `MuscleVisualization` and
  `TendonVisualization` resources use the same manual-push pattern as the
  old sensor viz. They should get the same redesign treatment, but that's
  for the muscles example spec.

- **Contact visualization** — the `CachedContacts` system needs work but
  that's for the contact-tuning example spec.

- **spring_coil axis** — minor issue (hardcoded to +X). Can fix ad hoc.

- **Coordinate system documentation** — important but orthogonal.

---

## Test Plan

### Change 1 (Materials)
- Unit test: `spawn_model_geoms` with overrides produces entities with
  correct material handles (not MJCF-rgba defaults).
- Unit test: empty overrides `&[]` preserves existing behavior.
- Integration: all 27 call sites compile. All 8 existing examples render
  identically.

### Change 2 (Accessors)
- Unit test: `data.joint_qpos(&model, id)` returns correct slice for each
  joint type (hinge → 1, ball → 4, free → 7, slide → 1).
- Unit test: `data.joint_qvel(&model, id)` returns correct slice.
- Unit test: `data.sensor_data(&model, id)` returns correct slice matching
  `sensor_adr`/`sensor_dim`.
- Unit test: `model.joint_id("nonexistent")` returns `None`.
- Unit test: mutable variants allow writing and changes persist.
- Integration: existing conformance tests pass unchanged.

### Change 3 (Validation)
- Unit test: `ValidationHarness` with `track_energy` reports pass/fail
  correctly given known energy values.
- Unit test: `track_period` with known sinusoidal data detects correct period.
- Integration: one existing example migrated to harness produces identical
  console output (report text matches).

### Change 4 (HUD)
- Visual test: HUD renders monospace text in top-left corner with dark
  background.
- Unit test: `PhysicsHud::scalar` / `vec3` / `section` produce correct
  formatted strings.
- Visual test: HUD respects `ViewerConfig::show_hud` toggle.

### Change 5 (Sensor Viz)
- Unit test: `sensor_type_to_gizmo` maps all 38 sensor types to a gizmo
  variant (no panics, no unhandled types).
- Unit test: local-frame sensors (Gyro) produce rotated world-frame arrows.
- Unit test: world-frame sensors (FrameLinVel, no ref) produce unrotated arrows.
- Integration: model with 5+ sensors auto-populates `SensorVisualization`.
- Visual test: sensor gizmos render at correct world positions.

### Change 6 (Multi-Scene)
- Unit test: `PhysicsScenes::add` returns sequential IDs starting at 0.
- Unit test: `step_scenes_lockstep` with identical scenes produces identical
  `data.time` after N frames.
- Integration: 3-scene setup spawns 3× the geom entities, each tagged with
  correct `PhysicsSceneId`.
- Visual test: scenes render side-by-side with correct offsets.
- Visual test: all 3 scenes animate independently.
