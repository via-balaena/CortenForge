# Plugin/Extension System (§66) — Spec

**Status:** Draft
**Phase:** Roadmap Phase 13 — Remaining Core
**Effort:** L
**MuJoCo ref:** `mjpPlugin` in `mjplugin.h`, `engine_forward.c`, `engine_sensor.c`,
`engine_passive.c`, `engine_collision_sdf.c`, `engine_plugin.cc`
**MuJoCo version:** 3.4.0
**Test baseline:** 2,297+ sim domain tests
**Prerequisites:**
- Spec A landed (Session 2, commit `099299e`)
- Spec B landed (Session 6, commit `6f056ec`)
- Spec C landed (Session 10, commit `78f9462`)
- No code-level dependency — plugin system is additive (new files + hooks)

**Independence:** This spec is independent of Specs A–C at code level. Spec C
(composite) adds `<composite>` parsing; Spec D adds `<extension>`/`<plugin>`
parsing. No shared files conflict.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for models using plugins. The
> MuJoCo C source code is the single source of truth. When in doubt, read
> the C source and match its behavior exactly.
>
> **Extension note:** The Rust trait-based registration mechanism replaces
> MuJoCo's C `dlopen`/`LoadLibrary` dynamic loading. This is a necessary
> platform adaptation (Rust is statically compiled), not a conformance
> deviation. Plugin *behavior* (dispatch ordering, state management,
> capability gating) matches MuJoCo exactly.

---

## Scope Adjustment

| Umbrella claim | MuJoCo 3.4.0 reality | Action |
|----------------|---------------------|--------|
| 4 plugin types | Confirmed: ACTUATOR, SENSOR, PASSIVE, SDF (bitfield) | In scope |
| Dynamic library loading | C `dlopen`/`LoadLibrary` — no Rust equivalent | Replace with `PluginRegistry` struct + explicit registration |
| Resource providers + decoders | Separate from physics plugins | Out of scope (DT-171) |
| Plugin visualization | `mjv_updateScene` callback — L0 has no viz | Out of scope (L1/Bevy) |
| SDF collision dispatch | Complex collision pipeline integration | Trait defined; collision dispatch deferred (DT-170) |

**Final scope:**
1. `Plugin` trait + capability types (new `plugin.rs` module)
2. `PluginRegistry` for name-based registration and lookup
3. Model plugin fields (dimensions, per-object IDs, state/attr storage)
4. Data plugin fields (mutable state, plugin-managed data)
5. MJCF types for `<extension>` and per-element `<plugin>`
6. MJCF parsing of `<extension>` declarations + `<plugin>` on elements
7. Builder plugin resolution (MJCF → Model instances)
8. Forward pipeline dispatch hooks (actuation, passive, sensor, advance)
9. Plugin lifecycle (init in `make_data`, reset, advance)
10. `MjSensorType::Plugin` and `MjObjectType::Plugin` enum variants

---

## Problem Statement

**Conformance gap** — MuJoCo implements a plugin/extension system (`§66`) that
allows users to register custom actuator, sensor, passive force, and SDF
plugins via `<extension>`/`<plugin>` MJCF elements and a C API. CortenForge
has zero plugin infrastructure: no trait definitions, no MJCF parsing for
`<extension>`, no dispatch hooks in the forward pipeline, and no Model/Data
fields for plugin state. MJCF models containing `<extension>` elements
currently hit the `_ => skip_element()` fallback silently — this means valid
MuJoCo models with plugins are silently broken.

---

## MuJoCo Reference

### Plugin Capability Flags (`mjplugin.h:90–95`)

```c
typedef enum mjtPluginCapabilityBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,
  mjPLUGIN_SENSOR   = 1<<1,
  mjPLUGIN_PASSIVE  = 1<<2,
  mjPLUGIN_SDF      = 1<<3,
} mjtPluginCapabilityBit;
```

A single plugin can have multiple capabilities (bitfield OR). The `compute()`
callback receives the active capability bit so it knows which role it's
fulfilling on each call.

### Plugin Struct (`mjplugin.h:97–156`)

`mjpPlugin` has these callback function pointers:

| Callback | Required? | Signature (simplified) | Purpose |
|----------|-----------|----------------------|---------|
| `nstate` | Yes | `(m, instance) → int` | Number of state values for this instance |
| `nsensordata` | Sensor only | `(m, instance, sensor_id) → int` | Dimension of sensor output |
| `init` | Yes | `(m, d, instance) → int` | Initialize plugin data; 0=success, -1=failure |
| `destroy` | Optional | `(d, instance)` | Free plugin data |
| `copy` | Optional | `(dest, m, src, instance)` | Deep copy plugin data |
| `reset` | Yes | `(m, plugin_state, plugin_data, instance)` | Reset state |
| `compute` | Yes | `(m, d, instance, capability_bit)` | Main computation |
| `advance` | Optional | `(m, d, instance)` | Time integration |
| `actuator_act_dot` | Optional | `(m, d, instance)` | Actuator activation derivative |
| `sdf_distance` | SDF only | `(point, d, instance) → f64` | Signed distance |
| `sdf_gradient` | SDF only | `(gradient, point, d, instance)` | Distance gradient |
| `sdf_staticdistance` | SDF only | `(point, attributes) → f64` | Compile-time distance |
| `sdf_attribute` | SDF only | `(attribute, name, value)` | Parse config into numeric attributes |
| `sdf_aabb` | SDF only | `(aabb, attributes)` | Bounding box |

Additional fields: `name` (unique string), `nattribute`/`attributes` (config
schema), `capabilityflags` (bitfield), `needstage` (sensor stage).

### Plugin Registration (`engine_plugin.cc`)

Global registry: `mjp_registerPlugin(plugin)` adds to a thread-safe global
table. `mjp_getPlugin(name, &slot)` looks up by name. `mjp_getPluginAtSlot(slot)`
looks up by slot index. Plugins are globally unique by name.

### Model Fields (`mjmodel.h`)

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `nplugin` | int | 1 | Number of plugin instances |
| `npluginattr` | int | 1 | Total chars in all config attributes |
| `npluginstate` | int | 1 | Total mjtNums in state vector |
| `body_plugin` | int* | nbody | Per-body plugin ID; -1 = none |
| `geom_plugin` | int* | ngeom | Per-geom plugin ID; -1 = none |
| `actuator_plugin` | int* | nu | Per-actuator plugin ID; -1 = none |
| `sensor_plugin` | int* | nsensor | Per-sensor plugin ID; -1 = none |
| `plugin` | int* | nplugin | Global plugin slot per instance |
| `plugin_stateadr` | int* | nplugin | Address in state array |
| `plugin_statenum` | int* | nplugin | Number of states per instance |
| `plugin_attr` | char* | npluginattr | Flattened config attributes |
| `plugin_attradr` | int* | nplugin | Per-instance attribute address |
| `name_pluginadr` | int* | nplugin | Plugin instance name pointers |

### Data Fields (`mjdata.h`)

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `plugin_state` | mjtNum* | npluginstate | Mutable plugin state |
| `plugin` | int* | nplugin | Copy of m->plugin |
| `plugin_data` | uintptr_t* | nplugin | Plugin-managed data pointers |

### Forward Pipeline Dispatch (`engine_forward.c`, `engine_sensor.c`, `engine_passive.c`)

**Actuator dispatch** (`mj_fwdActuation()` in `engine_forward.c`):
Two loops over `m->nplugin`:
1. **act_dot**: After builtin activation dynamics — calls
   `plugin->actuator_act_dot(m, d, instance)` for plugins with
   `ACTUATOR` capability and non-null `actuator_act_dot`.
2. **force**: After builtin gain/bias — calls
   `plugin->compute(m, d, instance, mjPLUGIN_ACTUATOR)` for plugins with
   `ACTUATOR` capability.

```c
// Pattern (both loops identical structure):
if (m->nplugin) {
  const int nslot = mjp_pluginCount();
  for (int i = 0; i < m->nplugin; i++) {
    const int slot = m->plugin[i];
    const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
    if (plugin->capabilityflags & mjPLUGIN_ACTUATOR) {
      plugin->compute(m, d, i, mjPLUGIN_ACTUATOR);
    }
  }
}
```

**Passive dispatch** (`mj_passive()` in `engine_passive.c`):
After builtin passive forces and user callback (`mjcb_passive`):
```c
if (m->nplugin) {
  for (int i = 0; i < m->nplugin; i++) {
    if (plugin->capabilityflags & mjPLUGIN_PASSIVE) {
      plugin->compute(m, d, i, mjPLUGIN_PASSIVE);
    }
  }
}
```

**Sensor dispatch** (`engine_sensor.c`):
`compute_plugin_sensors(m, d, stage)` is called at 3 stages:
- After `mj_sensorPos()` → `stage = mjSTAGE_POS`
- After `mj_sensorVel()` → `stage = mjSTAGE_VEL`
- After `mj_sensorAcc()` → `stage = mjSTAGE_ACC`

Stage matching logic:
```c
int matches_stage = (plugin->needstage == stage) ||
                    (stage == mjSTAGE_POS && plugin->needstage == mjSTAGE_NONE);
```
A plugin with `needstage=NONE` fires at `POS` stage (earliest available).

After compute, cutoff is applied to all sensors attached to the plugin:
```c
for (int j = 0; j < m->nsensor; j++) {
  if (m->sensor_type[j] == mjSENS_PLUGIN &&
      m->sensor_plugin[j] == i &&
      m->sensor_needstage[j] == stage) {
    apply_cutoff(m, j, d->sensordata + m->sensor_adr[j]);
  }
}
```

**Advance** (`mj_advance()` in `engine_forward.c`):
After position/velocity/activation integration:
```c
if (m->nplugin) {
  for (int i = 0; i < m->nplugin; ++i) {
    if (plugin->advance) {
      plugin->advance(m, d, i);
    }
  }
}
```

### MJCF Schema

**Extension declaration (top-level):**
```xml
<extension>
  <plugin plugin="plugin.name">         <!-- declares plugin type -->
    <instance name="inst1">             <!-- named instance -->
      <config key="k" value="v"/>       <!-- configuration -->
    </instance>
  </plugin>
</extension>
```

**Plugin on elements:**
```xml
<body>
  <plugin plugin="name" instance="inst1">
    <config key="k" value="v"/>
  </plugin>
</body>
```

Attributes: `plugin` (required — plugin type name), `instance` (optional —
references a named instance from `<extension>`). Child `<config>` elements:
`key` (required), `value` (required).

### Edge Cases

1. **Zero plugins** (`nplugin == 0`): All dispatch loops skip immediately.
   No overhead for models without plugins.
2. **Plugin with no state** (`nstate() → 0`): Valid. `plugin_statenum[i] = 0`,
   no state slice allocated. `plugin_stateadr[i]` still set (next instance's address).
3. **Multi-capability plugin**: Single plugin instance with e.g.
   `ACTUATOR | PASSIVE`. Gets `compute()` called twice per step — once with
   `mjPLUGIN_ACTUATOR` during actuation, once with `mjPLUGIN_PASSIVE` during
   passive. The `capability_bit` argument distinguishes which call.
4. **Sensor `needstage=NONE`**: Maps to `mjSTAGE_POS` (fires at earliest stage).
5. **Invalid plugin name in MJCF**: MuJoCo errors during compilation. We
   return `MjcfError` during builder resolution.
6. **Duplicate config keys**: MuJoCo errors on `<config>` with duplicate keys
   within the same `<instance>` or `<plugin>` element.
7. **Plugin without `<extension>` declaration**: An element can reference a
   plugin by name directly (inline plugin, no `<extension>` needed). The
   plugin must still be registered in the global registry.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Plugin registration | Global C registry via `mjp_registerPlugin()` | **Not implemented** — no registry |
| MJCF `<extension>` parsing | Parsed by `xml_native_reader.cc` | **Silently skipped** (unknown element fallback) |
| Per-element `<plugin>` sub-element | Parsed on body/geom/actuator/sensor | **Not implemented** — no plugin sub-element support |
| Model plugin fields | `nplugin`, `body_plugin`, etc. on `mjModel` | **Not implemented** — no fields on Model |
| Data plugin state | `plugin_state`, `plugin_data` on `mjData` | **Not implemented** — no fields on Data |
| Actuator dispatch | `compute()` called in `mj_fwdActuation()` | **No hook** |
| Passive dispatch | `compute()` called in `mj_passive()` | **No hook** (user callback exists, no plugin dispatch) |
| Sensor dispatch | `compute_plugin_sensors()` at 3 stages | **No hook** |
| Advance | `advance()` called in `mj_advance()` | **No hook** |
| `mjSENS_PLUGIN` sensor type | Enum variant `mjSENS_PLUGIN` | **Missing** — no `MjSensorType::Plugin` |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Plugin callbacks | C function pointers (`void (*compute)(...)`) | Rust trait methods (`fn compute(&self, ...)`) | Trait method per C callback. `&self` replaces implicit global state. |
| Plugin registry | Global thread-safe table (`engine_plugin.cc`) | `PluginRegistry` struct on `Model` (or passed at build time) | Use `HashMap<String, Arc<dyn Plugin>>` instead of global table |
| Instance ID | `int instance` (0-based index into `plugin[]` array) | `usize` (0-based) | Direct port — no translation |
| Capability flags | `int capabilityflags` (C bitfield) | `PluginCapabilities` (bitflags crate or manual bitfield) | Direct port — same bit values |
| Plugin state | `mjtNum* plugin_state` (raw pointer + address/num arrays) | `Vec<f64>` + `plugin_stateadr`/`plugin_statenum` (safe slices) | `&mut data.plugin_state[adr..adr+num]` for state access |
| Plugin data | `uintptr_t* plugin_data` (opaque C pointer) | `Vec<Option<Box<dyn Any + Send + Sync>>>` | Type-erased owned data per instance |
| Sensor type | `mjSENS_PLUGIN` enum value | `MjSensorType::Plugin` | Add variant, `dim() → 0` (variable, set by nsensordata) |
| Config attributes | `char* plugin_attr` (null-terminated, concatenated) | `Vec<String>` (flattened) + address array | Concatenate key=value pairs, store addresses |
| Stage matching | `plugin->needstage == stage \|\| (stage==POS && needstage==NONE)` | Same logic in Rust | Direct port |

---

## Architecture Decisions

### AD-1: Single `Plugin` Trait vs. Per-Capability Traits

**Problem:** MuJoCo uses a single `mjpPlugin` struct with all callback
pointers. Rust could use one trait or separate traits per capability.

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Single `Plugin` trait with all methods | Matches MuJoCo 1:1, multi-capability natural, simpler registry | Implementors see methods they don't need (mitigated by defaults) |
| 2 | Per-capability traits (`ActuatorPlugin`, `SensorPlugin`, etc.) | Type-safe, implementors only see relevant methods | Multi-capability needs wrapper type, complex registry (4 trait objects per plugin) |

**Chosen:** Option 1 — Single `Plugin` trait with default implementations
for all optional methods. This matches MuJoCo's design exactly,
multi-capability plugins are natural (set bits + implement methods),
and the registry is simple (`Arc<dyn Plugin>`). Unused methods have
no-op defaults — implementors override only what they need.

### AD-2: Plugin Registry Location

**Problem:** MuJoCo uses a global registry. Rust doesn't favor global
mutable state.

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | `PluginRegistry` struct passed to builder | No global state, testable, thread-safe | Must be passed through build pipeline |
| 2 | Global `lazy_static` registry | Matches MuJoCo pattern | Global mutable state, testing difficulties |

**Chosen:** Option 1 — `PluginRegistry` struct passed to the builder at
model construction time. Clean, testable, no global state. The builder
resolves plugin names against the registry during compilation.

### AD-3: Plugin State Access Pattern

**Problem:** MuJoCo passes raw `mjtNum*` pointer + instance index.
Rust needs safe access.

**Chosen:** Plugin state accessed via `&mut [f64]` slices. The dispatch
code computes `let state = &mut data.plugin_state[adr..adr+num]` and
passes it to the plugin method. This is zero-cost (slice is a pointer+length)
and safe.

---

## Specification

### S1. Plugin Trait and Types

**File:** `sim/L0/core/src/plugin.rs` (new file)
**MuJoCo equivalent:** `mjpPlugin` struct in `mjplugin.h:97–156`
**Design decision:** AD-1 (single trait). AD-3 (safe state slices).

Create a new module defining the core plugin infrastructure:

```rust
use std::any::Any;
use std::collections::HashMap;
use std::sync::Arc;

use crate::types::data::Data;
use crate::types::model::Model;

/// Plugin capability flags (matches mjtPluginCapabilityBit).
///
/// A plugin can have multiple capabilities (bitfield OR).
/// The `compute()` callback receives the active capability so
/// it knows which role it's fulfilling.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PluginCapabilityBit {
    Actuator = 1 << 0,
    Sensor   = 1 << 1,
    Passive  = 1 << 2,
    Sdf      = 1 << 3,
}

/// Combined plugin capabilities (bitfield).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PluginCapabilities(pub u32);

impl PluginCapabilities {
    pub const NONE: Self = Self(0);

    pub fn contains(self, bit: PluginCapabilityBit) -> bool {
        self.0 & (bit as u32) != 0
    }

    pub fn with(self, bit: PluginCapabilityBit) -> Self {
        Self(self.0 | bit as u32)
    }
}

impl std::ops::BitOr<PluginCapabilityBit> for PluginCapabilities {
    type Output = Self;
    fn bitor(self, rhs: PluginCapabilityBit) -> Self {
        Self(self.0 | rhs as u32)
    }
}

impl From<PluginCapabilityBit> for PluginCapabilities {
    fn from(bit: PluginCapabilityBit) -> Self {
        Self(bit as u32)
    }
}

/// Computation stage for sensor plugins (matches mjtStage).
///
/// Default is `Pos` — most sensor plugins compute at position stage.
/// `None` is NOT the default; it means "no specific stage needed" and
/// is dispatched at the Pos stage (earliest available), matching MuJoCo's
/// `needstage == mjSTAGE_NONE` → fires at `mjSTAGE_POS` convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PluginStage {
    /// No specific stage needed — dispatched at Pos stage (earliest).
    None = 0,
    /// Position-dependent computation (default).
    #[default]
    Pos = 1,
    /// Velocity-dependent computation.
    Vel = 2,
    /// Acceleration-dependent computation.
    Acc = 3,
}

/// A simulation plugin (matches mjpPlugin).
///
/// Implement this trait to create custom actuators, sensors, passive forces,
/// or SDF collision geometries. Register instances with `PluginRegistry`
/// before building a model.
///
/// # Capabilities
///
/// Return the appropriate capability flags from `capabilities()`. Only
/// methods matching the declared capabilities will be called:
/// - `ACTUATOR`: `actuator_act_dot()` + `compute()` called in `mj_fwd_actuation()`
/// - `SENSOR`: `compute()` called at the stage returned by `need_stage()`
/// - `PASSIVE`: `compute()` called in `mj_fwd_passive()`
/// - `SDF`: `sdf_distance()` / `sdf_gradient()` called during collision
///
/// # Thread Safety
///
/// Plugins must be `Send + Sync` for multi-threaded simulation support.
pub trait Plugin: Send + Sync {
    /// Globally unique name (e.g., "mujoco.elasticity.cable").
    fn name(&self) -> &str;

    /// Plugin capabilities (bitfield of PluginCapabilityBit).
    fn capabilities(&self) -> PluginCapabilities;

    /// Configuration attributes this plugin accepts.
    /// Used for validation during model compilation.
    fn attributes(&self) -> &[&str] { &[] }

    /// Sensor computation stage (only relevant for SENSOR plugins).
    /// Default: Acc.
    fn need_stage(&self) -> PluginStage { PluginStage::Acc }

    /// Number of mjtNum state values for this instance.
    /// Called during model compilation to allocate `plugin_state`.
    fn nstate(&self, model: &Model, instance: usize) -> usize { 0 }

    /// Dimension of sensor output (SENSOR plugins only).
    /// Called during model compilation to set `sensor_dim`.
    fn nsensordata(&self, model: &Model, instance: usize, sensor_id: usize) -> usize { 0 }

    /// Initialize plugin data when Data is created.
    /// Called from `make_data()`. Return Err to abort.
    fn init(&self, model: &Model, data: &mut Data, instance: usize) -> Result<(), String> {
        Ok(())
    }

    /// Reset plugin state to initial values.
    fn reset(&self, _model: &Model, _state: &mut [f64], _instance: usize) {}

    /// Main computation callback.
    /// `capability` indicates which role is being computed (ACTUATOR/SENSOR/PASSIVE/SDF).
    fn compute(
        &self,
        _model: &Model,
        _data: &mut Data,
        _instance: usize,
        _capability: PluginCapabilityBit,
    ) {
    }

    /// Time integration of plugin state (optional).
    /// Called after position/velocity/activation integration.
    fn advance(&self, _model: &Model, _data: &mut Data, _instance: usize) {}

    /// Actuator activation derivative (ACTUATOR plugins, optional).
    /// Called before `compute()` during actuation.
    fn actuator_act_dot(&self, _model: &Model, _data: &mut Data, _instance: usize) {}

    /// SDF signed distance (SDF plugins).
    fn sdf_distance(&self, _point: &[f64; 3], _data: &Data, _instance: usize) -> f64 { 0.0 }

    /// SDF gradient of distance (SDF plugins).
    fn sdf_gradient(
        &self,
        _gradient: &mut [f64; 3],
        _point: &[f64; 3],
        _data: &Data,
        _instance: usize,
    ) {
    }
}

/// Registry for plugin type registration.
///
/// Plugins are registered by name before model compilation.
/// The builder resolves MJCF `<plugin>` references against this registry.
#[derive(Debug, Default, Clone)]
pub struct PluginRegistry {
    plugins: HashMap<String, Arc<dyn Plugin>>,
}

impl PluginRegistry {
    /// Create an empty registry.
    pub fn new() -> Self {
        Self {
            plugins: HashMap::new(),
        }
    }

    /// Register a plugin. Panics if a plugin with the same name already exists.
    pub fn register(&mut self, plugin: Arc<dyn Plugin>) {
        let name = plugin.name().to_string();
        if self.plugins.contains_key(&name) {
            panic!("plugin already registered: {name}");
        }
        self.plugins.insert(name, plugin);
    }

    /// Look up a plugin by name.
    pub fn get(&self, name: &str) -> Option<&Arc<dyn Plugin>> {
        self.plugins.get(name)
    }

    /// Number of registered plugin types.
    pub fn count(&self) -> usize {
        self.plugins.len()
    }
}

// Arc<dyn Plugin> needs Debug for PluginRegistry derive
impl std::fmt::Debug for dyn Plugin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Plugin(\"{}\")", self.name())
    }
}
```

### S2. Model Plugin Fields

**File:** `sim/L0/core/src/types/model.rs`
**MuJoCo equivalent:** Plugin fields in `mjmodel.h:710–1229`
**Design decision:** Fields mirror MuJoCo's arrays. Plugin trait objects
stored as `Arc<dyn Plugin>` for runtime dispatch.

Add to `Model` struct:

```rust
// ==================== Plugin Instances (§66) ====================
/// Number of plugin instances.
pub nplugin: usize,
/// Total plugin state values across all instances.
pub npluginstate: usize,

/// Per-body plugin instance ID; None = no plugin. Length: nbody.
pub body_plugin: Vec<Option<usize>>,
/// Per-geom plugin instance ID; None = no plugin. Length: ngeom.
pub geom_plugin: Vec<Option<usize>>,
/// Per-actuator plugin instance ID; None = not a plugin actuator. Length: nu.
pub actuator_plugin: Vec<Option<usize>>,
/// Per-sensor plugin instance ID; None = not a plugin sensor. Length: nsensor.
pub sensor_plugin: Vec<Option<usize>>,

/// Plugin trait objects for runtime dispatch. Length: nplugin.
/// Each entry is the registered Plugin that this instance references.
pub plugin_objects: Vec<Arc<dyn Plugin>>,
/// Plugin need_stage per instance (for sensor dispatch). Length: nplugin.
pub plugin_needstage: Vec<PluginStage>,
/// Plugin capabilities per instance (cached). Length: nplugin.
pub plugin_capabilities: Vec<PluginCapabilities>,

/// Address in `Data.plugin_state` for each instance. Length: nplugin.
pub plugin_stateadr: Vec<usize>,
/// Number of state values per instance. Length: nplugin.
pub plugin_statenum: Vec<usize>,

/// Flattened plugin config attributes (concatenated key=value strings).
pub plugin_attr: Vec<String>,
/// Address into `plugin_attr` per instance. Length: nplugin.
pub plugin_attradr: Vec<usize>,
/// Number of attributes per instance. Length: nplugin.
pub plugin_attrnum: Vec<usize>,

/// Plugin instance names (for MJCF `instance` attribute lookup). Length: nplugin.
pub plugin_name: Vec<Option<String>>,
```

Add use declaration at top of model.rs:
```rust
use crate::plugin::{Plugin, PluginCapabilities, PluginStage};
use std::sync::Arc;
```

Initialize in model construction (all empty/zero for models without plugins):
```rust
nplugin: 0,
npluginstate: 0,
body_plugin: vec![None; nbody],
geom_plugin: vec![None; ngeom],
actuator_plugin: vec![None; nu],
sensor_plugin: vec![None; nsensor],
plugin_objects: Vec::new(),
plugin_needstage: Vec::new(),
plugin_capabilities: Vec::new(),
plugin_stateadr: Vec::new(),
plugin_statenum: Vec::new(),
plugin_attr: Vec::new(),
plugin_attradr: Vec::new(),
plugin_attrnum: Vec::new(),
plugin_name: Vec::new(),
```

### S3. Data Plugin Fields

**File:** `sim/L0/core/src/types/data.rs`
**MuJoCo equivalent:** Plugin fields in `mjdata.h:253–280`

Add to `Data` struct:

```rust
// ==================== Plugin State (§66) ====================
/// Mutable plugin state vector (length `npluginstate`).
/// Each plugin instance owns a contiguous slice at
/// `plugin_state[stateadr..stateadr+statenum]`.
pub plugin_state: Vec<f64>,
/// Plugin-managed data (type-erased). Length: nplugin.
/// Plugins can store arbitrary data here via `init()`.
pub plugin_data: Vec<Option<Box<dyn std::any::Any + Send + Sync>>>,
```

Initialize in `make_data()`:
```rust
plugin_state: vec![0.0; model.npluginstate],
plugin_data: (0..model.nplugin).map(|_| None).collect(),
```

### S4. Enum Variants

**File:** `sim/L0/core/src/types/enums.rs`
**MuJoCo equivalent:** `mjSENS_PLUGIN` in `mjmodel.h:366`, `mjOBJ_PLUGIN` in `mjmodel.h:285`

Add `Plugin` variant to `MjSensorType`:
```rust
/// Plugin-controlled sensor (dimension set by plugin nsensordata).
/// MuJoCo: mjSENS_PLUGIN.
Plugin,
```

In `MjSensorType::dim()`, add to the `0` arm:
```rust
Self::User | Self::Plugin => 0, // Variable, set explicitly
```

In `MjSensorType::data_kind()`: no change needed. The existing `_ =>` catch-all
maps unknown variants to `MjSensorDataKind::Real`, which is correct for plugin
sensors (they produce `f64` values written to `sensordata`). Do NOT add a
`MjSensorDataKind::Plugin` variant — plugin sensors are just `Real` data
produced by plugin code instead of builtin code.

Add `Plugin` variant to `MjObjectType`:
```rust
/// Plugin instance.
Plugin,
```

### S5. MJCF Types

**File:** `sim/L0/mjcf/src/types.rs`
**MuJoCo equivalent:** `mjsPlugin` in `mjspec.h:210–216`

Add new types:

```rust
/// Plugin configuration (key-value pair from <config> element).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfPluginConfig {
    pub key: String,
    pub value: String,
}

/// Plugin instance declaration (from <instance> under <extension>/<plugin>).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfPluginInstance {
    /// Instance name (required, referenced by elements).
    pub name: String,
    /// Configuration key-value pairs.
    pub config: Vec<MjcfPluginConfig>,
}

/// Plugin type declaration (from <plugin> under <extension>).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfExtensionPlugin {
    /// Plugin type name (e.g., "mujoco.elasticity.cable").
    pub plugin: String,
    /// Named instances.
    pub instances: Vec<MjcfPluginInstance>,
}

/// Extension element (top-level <extension>).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfExtension {
    /// Plugin type declarations.
    pub plugins: Vec<MjcfExtensionPlugin>,
}

/// Plugin reference on an element (body, geom, actuator, sensor).
/// Parsed from <plugin> child element on a body/geom, or from
/// plugin/instance attributes on actuator/sensor.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfPluginRef {
    /// Plugin type name.
    pub plugin: String,
    /// Reference to a named instance (from <extension>).
    pub instance: Option<String>,
    /// Inline configuration (when no instance reference).
    pub config: Vec<MjcfPluginConfig>,
    /// Whether this plugin reference is active.
    pub active: bool,
}
```

Add to `MjcfModel`:
```rust
/// Extension declarations (plugin types + instances).
pub extensions: Vec<MjcfExtension>,
```

Add to `MjcfBody`:
```rust
/// Plugin reference (passive force plugin on this body).
pub plugin: Option<MjcfPluginRef>,
```

Add to `MjcfActuator` and `MjcfSensor` (if not already present):
```rust
/// Plugin reference (plugin-controlled actuator/sensor).
pub plugin: Option<MjcfPluginRef>,
```

### S6. MJCF Parsing

**File:** `sim/L0/mjcf/src/parser.rs`
**MuJoCo equivalent:** `xml_native_reader.cc` — `ReadPluginConfigs()`, extension parsing
**Design decision:** Follow existing parser patterns (quick_xml, `get_attribute`).

**6a. Top-level `<extension>` parsing:**

Add to `parse_mujoco()` dispatch (after existing elements, before `_ => skip_element`):
```rust
b"extension" => {
    let ext = parse_extension(reader)?;
    model.extensions.push(ext);
}
```

New function:
```rust
/// Parse `<extension>` element containing plugin declarations.
fn parse_extension<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfExtension> {
    let mut ext = MjcfExtension::default();
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_name = get_attribute(e, "plugin")
                    .ok_or_else(|| MjcfError::MissingAttribute {
                        element: "plugin".into(),
                        attribute: "plugin".into(),
                    })?;
                let mut ep = MjcfExtensionPlugin {
                    plugin: plugin_name,
                    instances: Vec::new(),
                };
                parse_extension_plugin(reader, &mut ep)?;
                ext.plugins.push(ep);
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"extension" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in extension".into()))
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(ext)
}

/// Parse children of <plugin> inside <extension>.
fn parse_extension_plugin<R: BufRead>(
    reader: &mut Reader<R>,
    ep: &mut MjcfExtensionPlugin,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"instance" => {
                let name = get_attribute(e, "name")
                    .ok_or_else(|| MjcfError::MissingAttribute {
                        element: "instance".into(),
                        attribute: "name".into(),
                    })?;
                let config = parse_plugin_config(reader, b"instance")?;
                ep.instances.push(MjcfPluginInstance { name, config });
            }
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"instance" => {
                let name = get_attribute(e, "name")
                    .ok_or_else(|| MjcfError::MissingAttribute {
                        element: "instance".into(),
                        attribute: "name".into(),
                    })?;
                ep.instances.push(MjcfPluginInstance {
                    name,
                    config: Vec::new(),
                });
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"plugin" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in plugin".into()))
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(())
}

/// Parse <config key="..." value="..."/> children inside a parent element.
fn parse_plugin_config<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
) -> Result<Vec<MjcfPluginConfig>> {
    let mut configs = Vec::new();
    let mut seen_keys = std::collections::HashSet::new();
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"config" => {
                let key = get_attribute(e, "key")
                    .ok_or_else(|| MjcfError::MissingAttribute {
                        element: "config".into(),
                        attribute: "key".into(),
                    })?;
                if !seen_keys.insert(key.clone()) {
                    return Err(MjcfError::Validation(format!(
                        "duplicate config key: {key}"
                    )));
                }
                let value = get_attribute(e, "value")
                    .ok_or_else(|| MjcfError::MissingAttribute {
                        element: "config".into(),
                        attribute: "value".into(),
                    })?;
                configs.push(MjcfPluginConfig { key, value });
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == parent_tag => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(format!(
                    "unexpected EOF in {}",
                    String::from_utf8_lossy(parent_tag)
                )))
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(configs)
}
```

**6b. Per-element `<plugin>` sub-element parsing:**

Add `<plugin>` child handling to `parse_body()`:
```rust
b"plugin" => {
    let plugin_ref = parse_plugin_ref(reader, e)?;
    body.plugin = Some(plugin_ref);
}
```

Add `<plugin>` child handling to `parse_geom()`:
```rust
b"plugin" => {
    let plugin_ref = parse_plugin_ref(reader, e)?;
    geom.plugin = Some(plugin_ref);
}
```

Add to `MjcfGeom` struct in `types.rs`:
```rust
/// Plugin reference (SDF plugin on this geom).
pub plugin: Option<MjcfPluginRef>,
```

New function:
```rust
/// Parse a <plugin> sub-element on a body/geom/etc.
fn parse_plugin_ref<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfPluginRef> {
    let plugin = get_attribute(start, "plugin")
        .ok_or_else(|| MjcfError::MissingAttribute {
            element: "plugin".into(),
            attribute: "plugin".into(),
        })?;
    let instance = get_attribute_opt(start, "instance");
    let config = parse_plugin_config(reader, b"plugin")?;

    // Validation: can't have both instance reference and inline config
    if instance.is_some() && !config.is_empty() {
        return Err(MjcfError::Validation(
            "plugin configuration attributes cannot be used in an element \
             that references a predefined plugin instance"
                .into(),
        ));
    }

    Ok(MjcfPluginRef {
        plugin,
        instance,
        config,
        active: true,
    })
}
```

### S7. Builder Plugin Resolution

**File:** `sim/L0/mjcf/src/builder/plugin.rs` (new file)
**MuJoCo equivalent:** `mjCModel::CopyPlugins()` in `user_model.cc`
**Design decision:** AD-2 (registry passed to builder).

The builder resolves MJCF plugin references against the `PluginRegistry`:

```rust
use std::collections::HashMap;
use std::sync::Arc;

use sim_core::plugin::{Plugin, PluginRegistry};
use crate::types::{MjcfExtension, MjcfPluginRef};

/// Resolved plugin instance for model construction.
#[derive(Debug)]
pub(crate) struct ResolvedPlugin {
    pub plugin: Arc<dyn Plugin>,
    pub instance_name: Option<String>,
    pub config: HashMap<String, String>,
}
```

In the builder's main `build()` function (`sim/L0/mjcf/src/builder/mod.rs`),
add `registry: &PluginRegistry` parameter and call plugin resolution after
existing model construction:

```rust
/// Build a Model from parsed MJCF.
///
/// `registry` provides plugin type lookups for models that use `<extension>`.
/// Pass `&PluginRegistry::new()` for models without plugins.
pub fn build(mjcf: &MjcfModel, registry: &PluginRegistry) -> Result<Model, BuildError> {
    // ... existing build logic ...

    // Resolve plugins (after all elements are built)
    resolve_and_assign_plugins(&mjcf, &mut model, registry)?;

    Ok(model)
}
```

The `resolve_and_assign_plugins` function:

```rust
/// Resolve all MJCF plugin references and populate Model plugin fields.
pub(crate) fn resolve_and_assign_plugins(
    mjcf: &MjcfModel,
    model: &mut Model,
    registry: &PluginRegistry,
) -> Result<(), String> {
    // 1. Resolve named instances from <extension> declarations
    let mut resolved: Vec<ResolvedPlugin> = Vec::new();
    let mut instance_map: HashMap<String, usize> = HashMap::new();

    for ext in &mjcf.extensions {
        for ep in &ext.plugins {
            let plugin = registry.get(&ep.plugin).ok_or_else(|| {
                format!("unknown plugin: {}", ep.plugin)
            })?;
            for inst in &ep.instances {
                if instance_map.contains_key(&inst.name) {
                    return Err(format!("duplicate plugin instance name: {}", inst.name));
                }
                let config: HashMap<String, String> = inst
                    .config
                    .iter()
                    .map(|c| (c.key.clone(), c.value.clone()))
                    .collect();
                let idx = resolved.len();
                resolved.push(ResolvedPlugin {
                    plugin: Arc::clone(plugin),
                    instance_name: Some(inst.name.clone()),
                    config,
                });
                instance_map.insert(inst.name.clone(), idx);
            }
        }
    }

    // 2. Helper: resolve a plugin ref, creating anonymous instance if inline
    let mut resolve_ref = |pref: &MjcfPluginRef| -> Result<usize, String> {
        if let Some(ref inst_name) = pref.instance {
            instance_map.get(inst_name).copied().ok_or_else(|| {
                format!("unknown plugin instance: {inst_name}")
            })
        } else {
            let plugin = registry.get(&pref.plugin).ok_or_else(|| {
                format!("unknown plugin: {}", pref.plugin)
            })?;
            let config = pref.config.iter()
                .map(|c| (c.key.clone(), c.value.clone()))
                .collect();
            let idx = resolved.len();
            resolved.push(ResolvedPlugin {
                plugin: Arc::clone(plugin),
                instance_name: None,
                config,
            });
            Ok(idx)
        }
    };

    // Assign per-body plugin IDs
    // (body_plugin is already initialized to vec![None; nbody])
    // Body plugins are resolved during body tree walk in the main builder.
    // Here we handle top-level assignment for bodies that have plugin refs.

    // Assign per-geom plugin IDs (SDF plugins on geoms)
    for (i, geom) in mjcf.geoms_flat().iter().enumerate() {
        if let Some(ref pref) = geom.plugin {
            let idx = resolve_ref(pref)?;
            model.geom_plugin[i] = Some(idx);
        }
    }

    // Assign per-actuator plugin IDs
    for (i, act) in mjcf.actuators.iter().enumerate() {
        if let Some(ref pref) = act.plugin {
            let idx = resolve_ref(pref)?;
            model.actuator_plugin[i] = Some(idx);
        }
    }

    // Assign per-sensor plugin IDs
    for (i, sen) in mjcf.sensors.iter().enumerate() {
        if let Some(ref pref) = sen.plugin {
            let idx = resolve_ref(pref)?;
            model.sensor_plugin[i] = Some(idx);
        }
    }

    // Populate model plugin arrays
    model.nplugin = resolved.len();
    let mut stateadr = 0;
    for (i, r) in resolved.iter().enumerate() {
        let nstate = r.plugin.nstate(model, i);
        model.plugin_objects.push(Arc::clone(&r.plugin));
        model.plugin_capabilities.push(r.plugin.capabilities());
        model.plugin_needstage.push(r.plugin.need_stage());
        model.plugin_stateadr.push(stateadr);
        model.plugin_statenum.push(nstate);
        model.plugin_name.push(r.instance_name.clone());

        // Flatten config attributes
        let attr_adr = model.plugin_attr.len();
        model.plugin_attradr.push(attr_adr);
        let attr_count = r.config.len();
        for (k, v) in &r.config {
            model.plugin_attr.push(format!("{k}={v}"));
        }
        model.plugin_attrnum.push(attr_count);

        stateadr += nstate;
    }
    model.npluginstate = stateadr;

    // For sensor plugins, query nsensordata to set sensor dimension
    for i in 0..model.nsensor {
        if let Some(plugin_id) = model.sensor_plugin[i] {
            let dim = model.plugin_objects[plugin_id].nsensordata(model, plugin_id, i);
            model.sensor_dim[i] = dim;
        }
    }

    Ok(())
}
```

### S8. Forward Pipeline Dispatch Hooks

**MuJoCo equivalent:** Plugin dispatch in `engine_forward.c`, `engine_sensor.c`,
`engine_passive.c`
**Design decision:** Zero-plugin fast path (skip when `nplugin == 0`).

**8a. Actuator dispatch** — `sim/L0/core/src/forward/actuation.rs`

Insert at end of `mj_fwd_actuation()`, after builtin force accumulation.
**Two separate loops** — MuJoCo runs ALL plugins' `actuator_act_dot` before
ANY plugin's `compute`. This ordering matters when plugins interact (plugin B's
act_dot should see state before any plugin's compute modifies it).

```rust
// §66: Plugin actuator dispatch — Phase 1: activation derivatives
// All act_dot calls run before any compute calls (MuJoCo ordering).
if model.nplugin > 0 {
    for i in 0..model.nplugin {
        if model.plugin_capabilities[i].contains(PluginCapabilityBit::Actuator) {
            model.plugin_objects[i].actuator_act_dot(model, data, i);
        }
    }
}

// §66: Plugin actuator dispatch — Phase 2: force computation
if model.nplugin > 0 {
    for i in 0..model.nplugin {
        if model.plugin_capabilities[i].contains(PluginCapabilityBit::Actuator) {
            model.plugin_objects[i].compute(model, data, i, PluginCapabilityBit::Actuator);
        }
    }
}
```

**8b. Passive dispatch** — `sim/L0/core/src/forward/passive.rs`

Insert after user callback (`cb_passive`), before closing brace:

```rust
// §66: Plugin passive force dispatch
if model.nplugin > 0 {
    for i in 0..model.nplugin {
        if model.plugin_capabilities[i].contains(PluginCapabilityBit::Passive) {
            model.plugin_objects[i].compute(model, data, i, PluginCapabilityBit::Passive);
        }
    }
}
```

**8c. Sensor dispatch** — `sim/L0/core/src/sensor/mod.rs` (or position/velocity/acceleration.rs)

Add dispatch function:
```rust
/// Dispatch plugin sensor computation at the given stage.
/// MuJoCo: compute_plugin_sensors() in engine_sensor.c.
fn compute_plugin_sensors(model: &Model, data: &mut Data, stage: PluginStage) {
    if model.nplugin == 0 {
        return;
    }
    for i in 0..model.nplugin {
        if !model.plugin_capabilities[i].contains(PluginCapabilityBit::Sensor) {
            continue;
        }
        // Stage matching: needstage == stage, OR stage is Pos and needstage is None
        let matches = model.plugin_needstage[i] == stage
            || (stage == PluginStage::Pos && model.plugin_needstage[i] == PluginStage::None);
        if !matches {
            continue;
        }
        model.plugin_objects[i].compute(model, data, i, PluginCapabilityBit::Sensor);

        // Apply cutoff to sensors attached to this plugin AT THIS STAGE.
        // MuJoCo checks sensor_needstage[j] == stage to avoid applying cutoff
        // at the wrong stage when a plugin serves sensors at different stages.
        for j in 0..model.nsensor {
            if model.sensor_type[j] == MjSensorType::Plugin
                && model.sensor_plugin[j] == Some(i)
                && model.sensor_needstage[j] == stage
            {
                apply_sensor_cutoff(model, data, j);
            }
        }
    }
}
```

Call from each sensor stage function:
- End of `mj_sensor_pos()`: `compute_plugin_sensors(model, data, PluginStage::Pos);`
- End of `mj_sensor_vel()`: `compute_plugin_sensors(model, data, PluginStage::Vel);`
- End of `mj_sensor_acc()`: `compute_plugin_sensors(model, data, PluginStage::Acc);`

**8d. Advance** — `sim/L0/core/src/integrate/mod.rs`

Insert at end of `integrate()`, after position/velocity updates:

```rust
// §66: Plugin state advance
if model.nplugin > 0 {
    for i in 0..model.nplugin {
        model.plugin_objects[i].advance(model, data, i);
    }
}
```

### S9. Plugin Lifecycle in make_data

**File:** `sim/L0/core/src/types/model_init.rs`
**MuJoCo equivalent:** `mj_makeData()` plugin init loop

In `make_data()`, after plugin state allocation, call `init()` on each plugin:

```rust
// §66: Initialize plugin instances
for i in 0..self.nplugin {
    if let Err(e) = self.plugin_objects[i].init(self, &mut data, i) {
        panic!("plugin init failed for instance {i}: {e}");
    }
}
```

### S10. Plugin Reset Dispatch

**File:** `sim/L0/core/src/types/model_init.rs` (or wherever `reset_data()` lives)
**MuJoCo equivalent:** `mj_resetData()` plugin reset loop

When resetting Data to initial state, call `reset()` on each plugin:

```rust
// §66: Reset plugin state
for i in 0..self.nplugin {
    let adr = self.plugin_stateadr[i];
    let num = self.plugin_statenum[i];
    let state = &mut data.plugin_state[adr..adr + num];
    self.plugin_objects[i].reset(self, state, i);
}
```

This goes in the `reset_data()` or equivalent function, after zeroing
`data.plugin_state` and before returning. Matches MuJoCo's `mj_resetData()`
which calls `plugin->reset(m, plugin_state, plugin_data, instance)` for
each plugin instance.

### S11. Module Registration

**File:** `sim/L0/core/src/lib.rs`

Add `pub mod plugin;` to expose the plugin module.

---

## Acceptance Criteria

### AC1: Plugin trait is defined and usable *(code review)*
**Assert:** `plugin.rs` exports `Plugin` trait, `PluginCapabilities`,
`PluginCapabilityBit`, `PluginStage`, `PluginRegistry`. All types are
`Send + Sync`. `Plugin` trait has default implementations for all optional
methods.

### AC2: Plugin registry works *(runtime test)*
**Given:** Create a `PluginRegistry`, register a test plugin named "test.plugin".
**After:** Call `registry.get("test.plugin")`.
**Assert:** Returns `Some` with correct plugin. `registry.get("nonexistent")`
returns `None`. `registry.count()` == 1.

### AC3: MJCF `<extension>` parses correctly *(runtime test)*
**Given:** MJCF with:
```xml
<extension>
  <plugin plugin="test.actuator">
    <instance name="act1">
      <config key="gain" value="100"/>
    </instance>
  </plugin>
</extension>
```
**After:** Parse MJCF.
**Assert:** `model.extensions[0].plugins[0].plugin` == "test.actuator",
`instances[0].name` == "act1", `instances[0].config[0].key` == "gain",
`instances[0].config[0].value` == "100".

### AC4: Duplicate config key rejected *(runtime test)*
**Given:** MJCF with two `<config>` elements having the same key.
**After:** Parse MJCF.
**Assert:** Returns `MjcfError::Validation` mentioning "duplicate config key".

### AC5: Plugin on body parses *(runtime test)*
**Given:** MJCF with `<body>` containing `<plugin plugin="test.passive"/>`.
**After:** Parse MJCF.
**Assert:** `body.plugin` is `Some` with `plugin == "test.passive"`, `active == true`.

### AC6: Model plugin fields populated *(runtime test)*
**Given:** Build a model with 1 registered actuator plugin, MJCF referencing it
on one actuator.
**After:** Build model.
**Assert:** `model.nplugin == 1`, `model.actuator_plugin[0] == Some(0)`,
`model.plugin_capabilities[0].contains(Actuator)`.
**Field:** `Model.nplugin`, `Model.actuator_plugin`, `Model.plugin_capabilities`

### AC7: Data plugin state allocated *(runtime test)*
**Given:** Build model with plugin that returns `nstate=3`.
**After:** `model.make_data()`.
**Assert:** `data.plugin_state.len() == 3`, `model.plugin_stateadr[0] == 0`,
`model.plugin_statenum[0] == 3`.
**Field:** `Data.plugin_state`

### AC8: Actuator plugin compute called *(runtime test)*
**Given:** Register an actuator plugin that sets `data.qfrc_actuator[0] = 42.0`
in its `compute()`. MJCF with one hinge joint + plugin actuator.
**After:** `data.step()`.
**Assert:** `data.qfrc_actuator[0]` reflects plugin's contribution (42.0 ± tolerance
depending on integration).
**Field:** `Data.qfrc_actuator`

### AC9: Passive plugin compute called *(runtime test)*
**Given:** Register a passive plugin that sets `data.qfrc_passive[0] += 10.0`
in its `compute()`. MJCF with one hinge joint + body with passive plugin.
**After:** `data.forward()`.
**Assert:** `data.qfrc_passive[0]` includes plugin's 10.0 contribution.
**Field:** `Data.qfrc_passive`

### AC10: Sensor plugin compute called at correct stage *(runtime test)*
**Given:** Register a sensor plugin with `need_stage=Vel` that writes `99.0`
to `data.sensordata[adr]`. MJCF with plugin sensor.
**After:** `data.forward()`.
**Assert:** `data.sensordata[sensor_adr] == 99.0`.
**Field:** `Data.sensordata`

### AC11: Zero plugins — no regression *(runtime test)*
**Given:** Any existing test model (no plugins).
**After:** Build + step.
**Assert:** All 2,297+ existing tests pass unchanged. No performance regression
from zero-plugin fast paths.

### AC12: Plugin advance called during integration *(runtime test)*
**Given:** Register plugin with `nstate=1`. In `advance()`, set
`data.plugin_state[adr] += 1.0`.
**After:** `data.step()`.
**Assert:** `data.plugin_state[0] == 1.0` (advance was called once).
**Field:** `Data.plugin_state`

### AC13: MjSensorType::Plugin enum variant exists *(code review)*
**Assert:** `MjSensorType::Plugin` variant exists. `dim()` returns 0 (variable).
All exhaustive match sites handle the new variant.

### AC14: Unknown plugin name rejected *(runtime test)*
**Given:** MJCF referencing `plugin="nonexistent.plugin"`, empty registry.
**After:** Attempt to build model.
**Assert:** Returns error mentioning "unknown plugin".

### AC15: Plugin reset called during data reset *(runtime test)*
**Given:** Register plugin with `nstate=2`. In `reset()`, set `state[0] = 1.0, state[1] = 2.0`.
**After:** Build model, `make_data()`, modify `plugin_state`, then `reset_data()`.
**Assert:** `data.plugin_state[0] == 1.0`, `data.plugin_state[1] == 2.0` (reset restored values).
**Field:** `Data.plugin_state`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (trait defined) | — | Code review (manual) |
| AC2 (registry) | T1 | Direct |
| AC3 (extension parse) | T2 | Direct |
| AC4 (dup config key) | T3 | Edge case |
| AC5 (body plugin parse) | T4 | Direct |
| AC6 (model fields) | T5 | Direct |
| AC7 (data state alloc) | T6 | Direct |
| AC8 (actuator dispatch) | T7 | Direct + integration |
| AC9 (passive dispatch) | T8 | Direct + integration |
| AC10 (sensor dispatch) | T9 | Direct + integration |
| AC11 (no regression) | T10 | Regression |
| AC12 (advance) | T11 | Direct |
| AC13 (enum variant) | — | Code review (manual) |
| AC14 (unknown plugin) | T12 | Edge case |
| AC3 (extension parse) | T13 | Supplementary (empty config) |
| AC6 (model fields) | T14 | Supplementary (multi-instance) |
| AC8 + AC9 (multi-cap) | T15 | Integration |
| AC15 (reset) | T16 | Direct |

---

## Test Plan

### T1: Plugin registry registration and lookup → AC2
Register a test plugin. Verify `get()` returns it by name. Verify `get()`
returns `None` for unknown names. Verify `count()`.

### T2: Parse `<extension>` with instances and config → AC3
Parse MJCF with nested `<extension>/<plugin>/<instance>/<config>` structure.
Assert all fields parsed correctly (plugin name, instance name, config
key-value pairs).

### T3: Duplicate config key rejected → AC4
Parse MJCF with `<config key="x" value="1"/><config key="x" value="2"/>`.
Assert `MjcfError::Validation` returned with "duplicate config key" message.

### T4: Plugin sub-element on body → AC5
Parse MJCF with `<body><plugin plugin="test.passive"/></body>`.
Assert `body.plugin` is populated with correct plugin name.

### T5: Model plugin fields populated after build → AC6
Create a `PluginRegistry` with a test actuator plugin. Build a model from
MJCF that references it. Assert `model.nplugin`, `model.actuator_plugin`,
`model.plugin_capabilities` are correct.

### T6: Data plugin state allocation → AC7
Register plugin with `nstate=5`. Build model + `make_data()`. Assert
`data.plugin_state.len() == 5`. Assert `model.plugin_stateadr[0] == 0`,
`model.plugin_statenum[0] == 5`.

### T7: Actuator plugin dispatch end-to-end → AC8
Register actuator plugin whose `compute()` adds a force to `qfrc_actuator`.
Build model with one hinge + plugin actuator. Step once. Verify
`qfrc_actuator` reflects the plugin's force.

### T8: Passive plugin dispatch end-to-end → AC9
Register passive plugin whose `compute()` adds to `qfrc_passive`. Build
model with body + passive plugin. Forward once. Verify `qfrc_passive`
includes plugin contribution.

### T9: Sensor plugin stage dispatch → AC10
Register sensor plugin with `need_stage=Vel`. Build model with plugin sensor.
Forward once. Verify `sensordata` at sensor address contains plugin's output.

### T10: Zero-plugin regression → AC11
Run existing sim domain test suite. All 2,297+ tests pass unchanged.

### T11: Plugin advance in integration → AC12
Register plugin with `nstate=1`, `advance()` increments state. Build + step.
Verify `plugin_state[0] == 1.0`.

### T12: Unknown plugin name error → AC14
Build model with MJCF referencing unregistered plugin name. Assert build
returns error with "unknown plugin" message.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Zero plugins | Most models won't use plugins; dispatch must be zero-cost | T10 | AC11 |
| Plugin with nstate=0 | Valid: stateless plugins exist | T5 variant | AC6 |
| Multi-capability plugin | ACTUATOR\|PASSIVE gets compute() twice per step | T7+T8 combined | AC8, AC9 |
| Sensor needstage=None | Maps to POS stage (MuJoCo convention) | T9 variant | AC10 |
| Duplicate config key | MuJoCo rejects this | T3 | AC4 |
| Unknown plugin name | Must error, not silently skip | T12 | AC14 |
| Instance reference + inline config | MuJoCo rejects this combination | T3 variant | AC4 |
| Plugin on body (passive) | Body-level plugin for passive forces | T8 | AC9 |

### T16: Plugin reset restores state → AC15
Register plugin with `nstate=2`, `reset()` writes `[1.0, 2.0]`. Build model,
`make_data()`, overwrite `plugin_state` with `[99.0, 99.0]`, then
`reset_data()`. Assert `plugin_state == [1.0, 2.0]`.

### T15: Multi-capability plugin dispatched for both capabilities → AC8, AC9
Register a plugin with `capabilities = ACTUATOR | PASSIVE`. Build model with
body (passive plugin) + actuator (same plugin instance). Step once. Verify
that `compute()` was called with both `PluginCapabilityBit::Actuator` and
`PluginCapabilityBit::Passive` (use an `AtomicU32` counter in the test plugin
to track call count and capability bits).

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T13 (plugin with empty config) | Plugin with no `<config>` children | Boundary: empty config is valid |
| T14 (multiple plugin instances) | Model with 3+ plugin instances of same type | Verifies instance ID assignment and state address computation |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `<extension>` parsing | Silently skipped | Parsed into `MjcfExtension` | Toward MuJoCo | Models with `<extension>` elements | None — transparent |
| `MjSensorType` enum | No `Plugin` variant | `Plugin` variant added | Toward MuJoCo | Exhaustive match sites | Add arm |
| `MjObjectType` enum | No `Plugin` variant | `Plugin` variant added | Toward MuJoCo | Exhaustive match sites | Add arm |
| Model struct | No plugin fields | Plugin fields added | Toward MuJoCo | Model consumers | Fields default to empty/zero |
| Data struct | No plugin fields | Plugin state/data added | Toward MuJoCo | Data consumers | Fields default to empty/zero |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/plugin.rs` | New module — trait, types, registry | +200 |
| `sim/L0/core/src/lib.rs` | Add `pub mod plugin;` | +1 |
| `sim/L0/core/src/types/model.rs` | Add plugin fields to Model | +30 |
| `sim/L0/core/src/types/model_init.rs` | Init plugin fields in make_data + reset dispatch | +25 |
| `sim/L0/core/src/types/data.rs` | Add plugin_state, plugin_data | +10 |
| `sim/L0/core/src/types/enums.rs` | Add Plugin variants to MjSensorType, MjObjectType | +10 |
| `sim/L0/core/src/forward/actuation.rs` | Plugin actuator dispatch hook | +10 |
| `sim/L0/core/src/forward/passive.rs` | Plugin passive dispatch hook | +10 |
| `sim/L0/core/src/sensor/mod.rs` | compute_plugin_sensors() + call sites | +30 |
| `sim/L0/core/src/integrate/mod.rs` | Plugin advance hook | +8 |
| `sim/L0/mjcf/src/types.rs` | MjcfExtension, MjcfPluginRef, etc. | +60 |
| `sim/L0/mjcf/src/parser.rs` | parse_extension(), parse_plugin_ref(), parse_geom plugin, etc. | +130 |
| `sim/L0/mjcf/src/builder/plugin.rs` | New — plugin resolution | +80 |
| `sim/L0/mjcf/src/builder/mod.rs` | Call plugin resolution in build pipeline | +10 |
| Tests | New test file(s) | +280 |
| **Total** | | **~910** |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All 2,297+ existing tests | Various | Pass (unchanged) | Plugin system is purely additive; no existing behavior modified |
| `MjSensorType` exhaustive matches | See below | Compile error until `Plugin` arm added | New enum variant |
| `MjObjectType` exhaustive matches | See below | Compile error until `Plugin` arm added | New enum variant |

**Exhaustive match sites requiring `Plugin` arm:**

| File | Line | Match | Action |
|------|------|-------|--------|
| `types/enums.rs:440` | `MjSensorType::dim()` | Exhaustive (no catch-all) | Add `Self::Plugin => 0` |
| `types/enums.rs:490` | `MjSensorType::data_kind()` | Has `_ =>` catch-all | No change needed — falls to `Real` |
| `sensor/position.rs` | `mj_sensor_pos()` match | Has `_ => {}` catch-all | No change — Plugin skipped (dispatched separately) |
| `sensor/velocity.rs` | `mj_sensor_vel()` match | Has `_ => {}` catch-all | No change — Plugin skipped |
| `sensor/acceleration.rs` | `mj_sensor_acc()` match | Has `_ => {}` catch-all | No change — Plugin skipped |
| `sensor/postprocess.rs` | Cutoff/postprocess | Has `_ =>` catch-all | No change — Plugin sensors get cutoff via `compute_plugin_sensors` |
| `mjcf/src/builder/sensor.rs` | Sensor builder | Match on sensor type | Add `Plugin` handling |
| `sensor/mod.rs` | `sensor_body_id()` match on `MjObjectType` | Has `_ =>` patterns | No change — Plugin falls through |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `forward/mod.rs` step/forward | Pipeline orchestration | Dispatch hooks go in sub-functions, not orchestrator |
| `constraint/` | Constraint solver | Plugins don't affect constraints |
| `collision/` | Collision detection | SDF dispatch deferred (DT-170) |

---

## Execution Order

1. **S1** (plugin.rs — traits + types + registry) → standalone, no deps
2. **S4** (enum variants) → add `Plugin` variants, fix all match sites
3. **S2** (Model fields) → depends on S1 types
4. **S3** (Data fields) → depends on S2 (model references)
5. **S5** (MJCF types) → standalone MJCF-side
6. **S6** (MJCF parsing) → depends on S5 types
7. **S7** (builder plugin resolution) → depends on S1, S2, S5, S6
8. **S8** (pipeline dispatch hooks) → depends on S1, S2, S3
9. **S9** (lifecycle in make_data) → depends on S1, S2, S3
10. **S10** (reset dispatch) → depends on S1, S2, S3
11. **S11** (module registration) → depends on S1

After each section, run `cargo test -p sim-core -p sim-mjcf` to verify
no regressions.

---

## Out of Scope

- **SDF collision dispatch** (DT-170) — Trait defined but collision pipeline
  integration requires `mjc_SDF()` equivalent. Conformance impact: models
  with SDF plugins won't have functional collision. Acceptable for v1.0
  (no known MuJoCo test models require SDF plugins for conformance).
- **Resource providers** (DT-171) — `mjpResourceProvider` handles file I/O,
  not physics. No conformance impact.
- **Decoders** (DT-171) — `mjpDecoder` handles file format decoding.
  No conformance impact.
- **Plugin visualization** — `mjv_updateScene` callback. L0 has no viz;
  L1/Bevy concern. No conformance impact.
- **Dynamic library loading** — `mj_loadPluginLibrary()`. Replaced by
  Rust trait registration. No conformance impact (same plugin behavior,
  different registration mechanism).
- **Plugin copy/destroy callbacks** — Optional lifecycle for deep-copying
  Data with plugin state. Deferred until Data cloning is needed. No
  conformance impact for basic usage.
- **Data Clone with plugins** — `plugin_data: Vec<Option<Box<dyn Any + Send + Sync>>>`
  does not implement `Clone`. If `Data` currently derives `Clone`, the impl
  must be made manual (skip or shallow-clone `plugin_data`). Alternatively,
  implement `copy` callback on `Plugin` trait and use it for deep cloning.
  Deferred — tracked with copy/destroy above.
