# Plugin/Extension System (§66) — Spec Quality Rubric

Grades the Spec D spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** P1 (MuJoCo Reference Fidelity)
is the most important criterion — grade it first and hardest.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

| Umbrella claim | MuJoCo 3.4.0 reality | Action |
|----------------|---------------------|--------|
| 4 plugin types: actuator, sensor, passive, SDF | Confirmed — `mjtPluginCapabilityBit` has exactly these 4 flags. A single plugin can have multiple capabilities (bitfield). | In scope: all 4 |
| Dynamic library loading (`mj_loadPluginLibrary`) | MuJoCo uses dlopen/LoadLibrary for C shared libs. Rust doesn't have this pattern — trait objects are the idiomatic equivalent. | Replace with Rust trait-based registration. Dynamic loading out of scope (post-v1.0 `libloading` crate if needed). |
| `mjPLUGIN_LIB_INIT` constructor macro | C-specific linker attribute for auto-registration. No Rust equivalent needed — users register plugins explicitly via API. | Drop — Rust-idiomatic registration replaces this |
| Resource providers and decoders | `mjpResourceProvider` and `mjpDecoder` are separate from the 4 plugin types. They handle file I/O and format decoding. | Out of scope — not physics plugins, deferred to post-v1.0 |
| Plugin visualization (`visualize` callback) | Optional callback for mjv_updateScene. CortenForge has no visualization layer in L0. | Out of scope — L1/Bevy concern |
| SDF collision integration | Full SDF requires `sdf_distance`, `sdf_gradient`, `sdf_staticdistance`, `sdf_aabb` callbacks integrated into collision detection pipeline. Complex. | In scope for trait definition and registration. Collision dispatch deferred to DT-171 (requires collision pipeline integration). |

**Final scope:**
1. Single `Plugin` trait (AD-1) with capability-gated methods — covers all 4 plugin types (including SDF; collision dispatch deferred DT-171)
2. `PluginRegistry`: build-time registration, name-based lookup
3. Model fields: `nplugin`, per-object plugin IDs, plugin state/attr storage
4. Data fields: `plugin_state` mutable state vector, `plugin_data` type-erased data
5. MJCF types: `MjcfExtension`, `MjcfPluginRef`, etc.
6. MJCF parsing: `<extension>` declarations + `<plugin>` sub-elements on bodies/geoms/actuators/sensors
7. Builder plugin resolution: MJCF → Model instances (registry passed to builder API)
8. Pipeline dispatch: actuator hooks (two-loop) in `mj_fwd_actuation`, passive hooks in `mj_fwd_passive`, sensor hooks at 3 stages, advance after integration
9. Plugin state lifecycle: init (make_data), reset (reset_data), advance (integration)
10. `MjSensorType::Plugin` and `MjObjectType::Plugin` enum variants

---

## Empirical Ground Truth

### EGT-1: MuJoCo plugin capability flags (mjplugin.h)

```c
typedef enum mjtPluginCapabilityBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,  // actuator forces
  mjPLUGIN_SENSOR   = 1<<1,  // sensor measurements
  mjPLUGIN_PASSIVE  = 1<<2,  // passive forces
  mjPLUGIN_SDF      = 1<<3,  // signed distance fields
} mjtPluginCapabilityBit;
```

A single plugin can have multiple capabilities (bitfield OR).

### EGT-2: mjpPlugin struct (mjplugin.h:97–156)

```c
struct mjpPlugin_ {
  const char* name;               // globally unique name
  int nattribute;                 // number of config attributes
  const char* const* attributes;  // attribute names
  int capabilityflags;            // bitfield of mjtPluginCapabilityBit
  int needstage;                  // sensor computation stage (mjtStage)
  // Required callbacks:
  int (*nstate)(const mjModel* m, int instance);
  int (*nsensordata)(const mjModel* m, int instance, int sensor_id);
  int (*init)(const mjModel* m, mjData* d, int instance);
  void (*reset)(const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance);
  void (*compute)(const mjModel* m, mjData* d, int instance, int capability_bit);
  // Optional callbacks:
  void (*destroy)(mjData* d, int instance);
  void (*copy)(mjData* dest, const mjModel* m, const mjData* src, int instance);
  void (*advance)(const mjModel* m, mjData* d, int instance);
  void (*actuator_act_dot)(const mjModel* m, mjData* d, int instance);
  // SDF-specific:
  mjtNum (*sdf_distance)(const mjtNum point[3], const mjData* d, int instance);
  void (*sdf_gradient)(mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance);
  mjtNum (*sdf_staticdistance)(const mjtNum point[3], const mjtNum* attributes);
  void (*sdf_attribute)(mjtNum attribute[], const char* name[], const char* value[]);
  void (*sdf_aabb)(mjtNum aabb[6], const mjtNum* attributes);
};
```

### EGT-3: Model plugin fields (mjmodel.h)

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `nplugin` | `int` | 1 | Number of plugin instances |
| `npluginattr` | `int` | 1 | Total chars in all plugin config attributes |
| `npluginstate` | `int` | 1 | Total mjtNums in plugin state vector |
| `body_plugin` | `int*` | nbody | Plugin instance ID per body; -1 = not in use |
| `geom_plugin` | `int*` | ngeom | Plugin instance ID per geom; -1 = not in use |
| `actuator_plugin` | `int*` | nu | Plugin instance ID per actuator; -1 = not a plugin |
| `sensor_plugin` | `int*` | nsensor | Plugin instance ID per sensor; -1 = not a plugin |
| `plugin` | `int*` | nplugin | Globally registered plugin slot number |
| `plugin_stateadr` | `int*` | nplugin | Address in plugin state array |
| `plugin_statenum` | `int*` | nplugin | Number of states per plugin instance |
| `plugin_attr` | `char*` | npluginattr | Flattened config attributes |
| `plugin_attradr` | `int*` | nplugin | Address to each instance's config attrib |
| `name_pluginadr` | `int*` | nplugin | Plugin instance name pointers |

### EGT-4: Data plugin fields (mjdata.h)

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `nplugin` | `int` | 1 | Copy of m->nplugin |
| `plugin_state` | `mjtNum*` | npluginstate | Mutable plugin state |
| `plugin` | `int*` | nplugin | Copy of m->plugin (for deletion) |
| `plugin_data` | `uintptr_t*` | nplugin | Plugin-managed data pointers |

### EGT-5: Forward pipeline dispatch points (engine_forward.c, engine_sensor.c, engine_passive.c)

| Plugin type | Pipeline function | When | Callback |
|-------------|------------------|------|----------|
| ACTUATOR (act_dot) | `mj_fwdActuation()` | After builtin actuator dynamics | `plugin->actuator_act_dot(m, d, instance)` |
| ACTUATOR (force) | `mj_fwdActuation()` | After gain/bias for builtins | `plugin->compute(m, d, instance, mjPLUGIN_ACTUATOR)` |
| PASSIVE | `mj_passive()` | After builtin passive + user callback | `plugin->compute(m, d, instance, mjPLUGIN_PASSIVE)` |
| SENSOR (pos) | `mj_sensorPos()` | After builtin pos sensors | `compute_plugin_sensors(m, d, mjSTAGE_POS)` |
| SENSOR (vel) | `mj_sensorVel()` | After builtin vel sensors | `compute_plugin_sensors(m, d, mjSTAGE_VEL)` |
| SENSOR (acc) | `mj_sensorAcc()` | After builtin acc sensors | `compute_plugin_sensors(m, d, mjSTAGE_ACC)` |
| SDF | `mjc_SDF()` collision | During collision detection | `plugin->sdf_distance/gradient(...)` |
| ALL (advance) | `mj_advance()` | After position/velocity/activation update | `plugin->advance(m, d, instance)` |

### EGT-6: MJCF schema for plugins

**Extension declaration (top-level):**
```xml
<extension>
  <plugin plugin="mujoco.elasticity.cable">
    <instance name="cable1">
      <config key="stiffness" value="1e-3"/>
    </instance>
  </plugin>
</extension>
```

**Plugin usage on elements:**
```xml
<body>
  <plugin plugin="mujoco.elasticity.cable" instance="cable1">
    <config key="param1" value="val1"/>
  </plugin>
</body>

<geom type="sdf">
  <plugin plugin="mujoco.sdf.torus"/>
</geom>

<sensor type="plugin" plugin="mujoco.sensor.touch_grid"/>

<general joint="j1" gainprm="100" plugin="mujoco.pid"/>
```

Attributes: `plugin` (plugin name), `instance` (named instance reference).
Child `<config>` elements: key-value configuration pairs.

### EGT-7: Codebase integration points

| File | Line | What | Action |
|------|------|------|--------|
| `sim/L0/mjcf/src/parser.rs:63` | `parse_mujoco()` | Top-level dispatch — no `extension` handler | Add `b"extension"` branch |
| `sim/L0/mjcf/src/types.rs:4087` | `MjcfModel` struct | No plugin fields | Add `extensions: Vec<MjcfExtension>` |
| `sim/L0/core/src/types/model.rs:44` | `Model` struct | No plugin fields | Add nplugin, per-object plugin IDs, plugin state/attr arrays |
| `sim/L0/core/src/types/data.rs:34` | `Data` struct | No plugin fields | Add plugin_state, plugin_data |
| `sim/L0/core/src/types/enums.rs:343` | `MjSensorType` | No `Plugin` variant | Add `Plugin` variant |
| `sim/L0/core/src/types/enums.rs:563` | `MjObjectType` | No `Plugin` variant | Add `Plugin` variant |
| `sim/L0/core/src/forward/actuation.rs` | `mj_fwd_actuation()` | No plugin dispatch | Add actuator plugin hooks |
| `sim/L0/core/src/forward/passive.rs` | `mj_fwd_passive()` | No plugin dispatch | Add passive plugin hooks |
| `sim/L0/core/src/sensor/mod.rs` | `mj_sensor_pos/vel/acc()` | No plugin dispatch | Add sensor plugin dispatch at each stage |
| `sim/L0/core/src/forward/mod.rs` | `mj_advance()` / integration | No plugin advance | Add plugin state advance hook |
| `sim/L0/core/src/types/model_init.rs` | `make_data()` | No plugin init | Add plugin init loop |
| `sim/L0/mjcf/src/builder/mod.rs` | Builder pipeline | No plugin resolution | Add plugin resolution step |

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites `mjpPlugin` struct (`mjplugin.h:97–156`) with every callback's signature and nullability. Cites `engine_forward.c` actuator plugin dispatch (two callbacks: `actuator_act_dot` then `compute`), `engine_passive.c` passive dispatch, `engine_sensor.c` `compute_plugin_sensors()` with stage-matching logic (including `mjSTAGE_NONE` → `mjSTAGE_POS` rule), `engine_collision_sdf.c` SDF dispatch. Cites MJCF schema: `<extension>` → `<plugin>` → `<instance>` → `<config>` hierarchy with exact attributes (`plugin`, `instance`, `name`, `key`, `value`). Cites model fields (`nplugin`, `body_plugin`, `geom_plugin`, `actuator_plugin`, `sensor_plugin`, `plugin[]`, `plugin_state*`, `plugin_attr*`) with types and sizes from `mjmodel.h`. Edge cases: zero plugins, plugin with no state, plugin with multiple capabilities, sensor plugin with `needstage=NONE`. |
| **A** | MuJoCo behavior described correctly. Minor gaps in callback nullability or stage-matching edge cases. |
| **B** | High-level description correct but missing specifics (e.g., doesn't distinguish `actuator_act_dot` from `compute`). |
| **C** | Partially correct. Plugin dispatch logic assumed rather than verified. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. Plugin registration,
> MJCF parsing, model compilation, and runtime dispatch are all specified
> in implementable Rust.

| Grade | Bar |
|-------|-----|
| **A+** | Every trait method signature is written in real Rust. Plugin registry lookup algorithm specified (name → slot). MJCF parsing for `<extension>` + `<plugin>` child elements fully specified with attribute names and error handling. Model compilation plugin resolution (name → instance, instance → slot, state address computation) fully specified. Builder API shows how registry is passed in. Forward pipeline dispatch loops with capability-flag gating specified for all 4 types + advance. Actuator dispatch uses two separate loops (act_dot for all, then compute for all — matching MuJoCo ordering). Plugin state init/reset/advance lifecycle complete — reset dispatch point specified (in `reset_data()`). |
| **A** | Algorithm complete. One or two minor details left implicit (e.g., exact error message wording). |
| **B** | Algorithm structure clear but some steps hand-waved (e.g., "resolve plugin" without specifying how). |
| **C** | Skeleton only. |

### P3. Convention Awareness

> MuJoCo C function pointers → Rust trait objects. C global registry →
> Rust registry struct. C `int instance` → Rust `usize`. All conventions
> mapped explicitly.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table maps: C function pointers → Rust trait methods, C global plugin table → Rust `PluginRegistry` struct, C `int instance` → Rust `usize` (direct port — both are plain integer indices), C `capabilityflags` bitfield → Rust `PluginCapabilities(u32)`, C `plugin_state` raw pointer → Rust `&mut [f64]` slice, C `plugin_data` (uintptr_t) → Rust `Box<dyn Any + Send + Sync>`, C `needstage` → Rust `PluginStage` enum. Each porting rule verified to preserve semantics. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | C code pasted without Rust adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Plugin system ACs test
> registration, parsing, dispatch, and lifecycle — not numerical physics
> output (plugins are user-defined).

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete input (MJCF model or programmatic setup), (2) exact expected behavior (callback invoked with correct arguments, state vector initialized to correct size, parsing produces correct config map), (3) what to check. At least one end-to-end AC: register a test plugin, load MJCF that uses it, step the simulation, verify plugin's compute() was called with correct arguments. Code-review ACs for trait safety (`Send + Sync`), no panics on zero plugins, no regressions. |
| **A** | ACs testable. Some lack exact expected values for callback arguments. |
| **B** | ACs directionally correct but vague. |
| **C** | Aspirational statements. |

### P5. Test Plan Coverage

> Tests cover: plugin registration, MJCF parsing, model compilation,
> forward dispatch at all stages, state lifecycle, edge cases.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: zero plugins (model without any plugin), plugin with no state (`nstate=0`), plugin with multiple capabilities, sensor plugin `needstage=NONE`, invalid plugin name in MJCF, plugin config with duplicate keys, model with plugins on body+geom+actuator+sensor simultaneously. At least one end-to-end test per plugin type (actuator, sensor, passive). Negative test: model without plugins is unaffected. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering, and interactions with Specs A–C stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Each section states what it requires. Prerequisites: Specs A–C landed (commit hashes). No circular dependencies. Spec D is independent of Specs A–C at code level (new files, not modifications to constraint solver). |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> New module — blast radius should be small. Main risk is enum variant
> additions and forward pipeline modifications.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. New files identified (`plugin.rs`, MJCF types). Modified files identified (forward pipeline, sensor dispatch, Model/Data structs, enums). Existing test impact: all 2,297+ tests pass unchanged (plugin system is additive). Enum variant additions (`MjSensorType::Plugin`, `MjObjectType::Plugin`) identified with all exhaustive match sites listed. Model/Data struct changes: new fields only, no modifications to existing fields. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions. Shared concepts (plugin instance, plugin slot, capability
> flags) use identical terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "plugin instance" vs "plugin slot" vs "plugin type" consistently distinguished. Cross-references accurate. File paths match between Specification and Files Affected. AC numbers match traceability matrix. Edge case lists consistent across MuJoCo Reference and Test Plan. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. API Design

> Plugin traits are idiomatic Rust, safe, extensible, and match MuJoCo's
> capability model.

| Grade | Bar |
|-------|-----|
| **A+** | Single `Plugin` trait (AD-1) with default implementations for all optional methods — each method maps to a specific MuJoCo callback with equivalent semantics. Multi-capability natural via bitfield (no wrapper types needed). Trait is `Send + Sync` for multi-threaded simulation. Registration API is ergonomic (`PluginRegistry::register()`). Plugin configuration is type-safe (parsed from MJCF key-value pairs into `HashMap<String,String>`). State access is safe (`&mut [f64]` slices, not raw pointers). |
| **A** | Traits well-designed. Minor ergonomic issues. |
| **B** | Traits work but miss some MuJoCo callbacks or have unsafe gaps. |
| **C** | Trait design incomplete or fundamentally wrong. |

### P10. Pipeline Integration

> Plugin dispatch happens at exactly the right point in the forward pipeline,
> matching MuJoCo's ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Spec names the exact CortenForge function and line range where each dispatch hook is inserted. Ordering matches MuJoCo: actuator dispatch uses **two separate loops** (ALL plugins' act_dot first, then ALL plugins' compute — NOT interleaved), passive compute (in `mj_fwd_passive`), sensor compute at 3 stages (in `mj_sensor_pos/vel/acc`) with cutoff filtered by `sensor_needstage == stage`, advance (in integration). Each hook is gated by capability flag check. Dispatch loop iterates `nplugin` instances, not per-object — matching MuJoCo's pattern. Zero-plugin fast path (skip dispatch entirely when `nplugin == 0`). |
| **A** | Dispatch points correct. Minor ordering detail left implicit. |
| **B** | Dispatch points mostly correct but one or two in wrong position. |
| **C** | Dispatch points incorrect or missing. |

---

## Rubric Self-Audit

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions (`engine_forward.c`, `engine_sensor.c`, `compute_plugin_sensors()`), struct fields (`nplugin`, `body_plugin`, `plugin_state`), and callback names (`actuator_act_dot`, `compute`, `advance`). Two independent reviewers would agree on grades.

- [x] **Non-overlap:** P1 grades MuJoCo reference accuracy. P2 grades algorithm completeness (Rust code). P3 grades convention translation (C→Rust). P9 grades API ergonomics (Rust-idiomatic). P10 grades pipeline ordering (where hooks fire). Each gap maps to exactly one criterion. Boundary: P1 asks "did we correctly understand MuJoCo?"; P10 asks "did we put the hooks in the right place in CortenForge?"

- [x] **Completeness:** All dimensions covered: MuJoCo fidelity (P1), algorithm (P2), conventions (P3), testability (P4, P5), dependencies (P6), risk (P7), consistency (P8), API design (P9), pipeline integration (P10). No gap could exist that isn't covered by at least one criterion.

- [x] **Gradeability:** P1 → MuJoCo Reference section. P2 → Specification sections. P3 → Convention Notes. P4 → Acceptance Criteria. P5 → Test Plan + Edge Case Inventory. P6 → Prerequisites + Execution Order. P7 → Risk & Blast Radius. P8 → all sections cross-checked. P9 → trait definitions in Specification. P10 → pipeline dispatch sections in Specification.

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C function names, header files, and dispatch patterns. P4 and P5 reference MuJoCo-derived dispatch behavior.

- [x] **Empirical grounding:** EGT-1 through EGT-7 filled in with verified MuJoCo header data, dispatch patterns from fetched source code, and CortenForge codebase integration points.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors, Convention Notes |
| P2 | Specification (S1–SN) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections* |
| P9 | Trait definitions in Specification, Architecture Decisions |
| P10 | Pipeline dispatch sections in Specification |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | A+ | Cites `mjpPlugin` (mjplugin.h:97–156) with all 14 callbacks (including `sdf_attribute`), signatures, and nullability. Cites `mjtPluginCapabilityBit` (4 flags). Cites model fields (13 arrays incl. `name_pluginadr`, with types/sizes from mjmodel.h). Cites dispatch from engine_forward.c (actuator two-phase), engine_sensor.c (stage matching with C snippet), engine_passive.c. MJCF schema documented. 7 edge cases. Convention table with 9 entries + porting rules. |
| P2. Algorithm Completeness | A+ | S1: Plugin trait with 13 method signatures in real Rust. PluginRegistry with HashMap. S6: 4 parser functions in real Rust (parse_extension, parse_extension_plugin, parse_plugin_config, parse_plugin_ref) + geom parsing. S7: Full resolve_and_assign_plugins with builder API signature change, per-element (body/geom/actuator/sensor) ID assignment, state address computation, nsensordata query. S8: Dispatch hooks — actuator uses two separate loops (act_dot then compute), sensor cutoff filtered by needstage. S9: init lifecycle. S10: reset dispatch in reset_data(). |
| P3. Convention Awareness | A+ | Convention table maps 9 C→Rust translations: function pointers→traits, global registry→PluginRegistry struct, int instance→usize (plain index), capabilityflags→PluginCapabilities(u32), plugin_state→Vec<f64> slices, plugin_data→Box<dyn Any + Send + Sync>, sensor type→enum variant, config→HashMap<String,String>, stage matching→same logic with PluginStage enum. Each has explicit porting rule. |
| P4. Acceptance Criteria Rigor | A+ | 15 ACs. All runtime ACs have three-part structure (input/assert/field). End-to-end ACs (AC8–AC10, AC12, AC15) test actual dispatch with concrete plugin behavior. Code review ACs (AC1, AC13) specify structural properties. AC14 tests error path. AC15 tests reset lifecycle. |
| P5. Test Plan Coverage | A+ | 16 tests + traceability matrix (all tests mapped including supplementary). Edge case inventory: 8 cases. End-to-end tests for all 3 dispatch-capable types (actuator T7, passive T8, sensor T9). Zero-plugin regression (T10). Multi-capability (T15). Negative (T12). Reset lifecycle (T16). Supplementary justified (T13, T14). |
| P6. Dependency Clarity | A+ | Prerequisites with commit hashes (099299e, 6f056ec, 78f9462). 11-step execution order (S1–S11) with explicit dependencies. Independence from Specs A–C stated with rationale. |
| P7. Blast Radius & Risk | A+ | 15 files listed with per-file change description and line estimates (~910 total). Exhaustive match sites enumerated (8 sites with file:line and required action). Non-modification sites listed (3). Behavioral changes table (5 entries). All 2,297+ tests pass unchanged. |
| P8. Internal Consistency | A+ | Terminology uniform: "plugin instance" (indexed), "plugin type" (registered), "capability" (bitfield). AC numbers match traceability matrix (15 ACs → 16 tests). File paths consistent between Specification sections and Files Affected. Edge cases consistent between MuJoCo Reference (7) and Test Plan (8 — superset). |
| P9. API Design | A+ | Single Plugin trait (AD-1) with default implementations — each method maps to an mjpPlugin callback. Send + Sync. Multi-capability natural via PluginCapabilities bitfield. PluginRegistry with HashMap (AD-2), passed to builder via API. Safe state access via &mut [f64] slices (AD-3). Config as HashMap<String,String>. plugin_data as Box<dyn Any + Send + Sync> (Data Clone impact noted in Out of Scope). |
| P10. Pipeline Integration | A+ | 4 dispatch points specified with exact function names and capability gating: actuation (two separate loops: ALL act_dot then ALL compute — matching MuJoCo ordering), passive (after cb_passive), sensor (3 stages with stage matching including None→Pos rule, cutoff filtered by sensor_needstage == stage), advance (after integration). Zero-plugin fast path on all. |

**Overall: A+ (Rev 2 — 24 gaps found and closed: 4 in Rev 1, 20 in Rev 2 stress test + verification)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| G1 | P2 | S7 builder resolution incomplete — showed skeleton but not full per-element plugin ID assignment and state address computation | Initial grading | Added complete `resolve_and_assign_plugins()` function with per-actuator/sensor ID assignment, state address loop, config flattening, and nsensordata query | Rev 1 |
| G2 | P5 | No dedicated test for multi-capability plugin (listed in edge cases but no test) | Initial grading | Added T15: multi-capability plugin dispatched for both ACTUATOR and PASSIVE, verified via AtomicU32 counter | Rev 1 |
| G3 | P6 | Missing commit hashes for Specs A–C prerequisites | Initial grading | Added commit hashes: 099299e (Spec A), 6f056ec (Spec B), 78f9462 (Spec C) | Rev 1 |
| G4 | P7 | Missing specific exhaustive match site line numbers for MjSensorType/MjObjectType enum variants | Initial grading | Added table of 8 exhaustive match sites with file, line, match type, and required action | Rev 1 |
| G5 | P2, P10 | S8a combined act_dot + compute in single loop; MuJoCo uses TWO separate loops (all act_dot first, then all compute) | Stress test | Split S8a into two separate loops with explanatory comment on ordering semantics | Rev 2 |
| G6 | P2, P10 | S8c sensor cutoff loop missing `sensor_needstage[j] == stage` check | Stress test | Added stage filter to cutoff loop matching MuJoCo's `compute_plugin_sensors()` | Rev 2 |
| G7 | P2 | Plugin `reset()` trait method defined but no dispatch point specified (MuJoCo calls it in `mj_resetData()`) | Stress test | Added S10 (reset dispatch in `reset_data()`), AC15, T16 | Rev 2 |
| G8 | P8 | Rubric Final Scope item 1 said per-capability traits (`ActuatorPlugin`, etc.) but spec chose single `Plugin` trait (AD-1) | Stress test | Updated rubric scope to match AD-1 single-trait decision | Rev 2 |
| G9 | P9 | Rubric P9 A+ bar said "one trait per plugin capability" contradicting spec's AD-1 | Stress test | Rewrote P9 bar to match single-trait design with capability-gated methods | Rev 2 |
| G10 | P3 | Rubric P3 bar said `PluginInstanceId(usize)` newtype; spec uses plain `usize` | Stress test | Updated P3 bar to `usize` (direct port matching MuJoCo's plain `int`) | Rev 2 |
| G11 | P8 | S7 had dead `resolve_plugins()` (5-param signature) alongside `resolve_and_assign_plugins()` which called it with 2 params | Stress test | Removed dead function; inlined extension resolution into `resolve_and_assign_plugins()` | Rev 2 |
| G12 | P8 | S4 ambiguous about `MjSensorDataKind::Plugin` — said "add if needed (or map to existing)" | Stress test | Committed: no new variant needed; `_ =>` catch-all maps to `Real` which is correct | Rev 2 |
| G13 | P2 | S8d advance code used `self` instead of `data` (wrong variable name) | Stress test | Fixed to `data` | Rev 2 |
| G14 | P2, P8 | Geom plugin parsing missing from S6 and S7 despite rubric scope and EGT-6 including geoms | Stress test | Added geom parsing in S6b, `MjcfGeom.plugin` field in S5, geom resolution loop in S7 | Rev 2 |
| G15 | P2 | Builder `build()` API signature change (adding `&PluginRegistry` param) not shown | Stress test | Added explicit builder signature change in S7 with doc comment | Rev 2 |
| G16 | P7 | Data Clone affected by `Box<dyn Any>` in `plugin_data` — not addressed | Stress test | Added to Out of Scope with explanation (deferred with copy/destroy callbacks) | Rev 2 |
| G17 | P5 | Supplementary tests T13/T14 not in AC→Test traceability matrix | Stress test | Added T13 → AC3, T14 → AC6 mappings to matrix | Rev 2 |
| G18 | P8 | PluginStage `#[default]` on `Pos` with misleading comment on `None` variant | Stress test | Added enum-level doc comment clarifying `None` vs `Pos` default semantics | Rev 2 |
| G19 | P8 | Rubric P8 scorecard said "14 ACs → 15 tests" — should be 15 ACs → 16 tests after Rev 2 additions | Verification | Updated scorecard to "15 ACs → 16 tests" | Rev 2 |
| G20 | P6 | Rubric P6 scorecard said "10-step execution order" — spec has 11 steps after S10 (reset) added | Verification | Updated to "11-step" | Rev 2 |
| G21 | P8 | Rubric scope said SDF collision deferred to DT-170; spec uses DT-171 (DT-170 = resource providers) | Verification | Fixed rubric scope to DT-171 | Rev 2 |
| G22 | P1 | Spec MuJoCo Reference callback table missing `sdf_attribute` (had 13/14 callbacks) | Verification | Added `sdf_attribute` row to spec callback table | Rev 2 |
| G23 | P1 | Spec MuJoCo Reference model fields missing `name_pluginadr` (had 12/13 fields) | Verification | Added `name_pluginadr` row to spec model fields table | Rev 2 |
| G24 | P8 | Rubric Final Scope had 8 items; spec Final Scope had 10 items — misaligned | Verification | Rewrote rubric Final Scope to match spec's 10-item structure | Rev 2 |
