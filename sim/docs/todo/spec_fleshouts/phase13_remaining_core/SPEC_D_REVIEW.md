# Plugin/Extension System (§66) — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D.md`
**Implementation session(s):** Session 15
**Reviewer:** AI agent
**Date:** 2026-03-11

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Plugin registration | Global C registry via `mjp_registerPlugin()` | **Not implemented** — no registry | | |
| MJCF `<extension>` parsing | Parsed by `xml_native_reader.cc` | **Silently skipped** (unknown element fallback) | | |
| Per-element `<plugin>` sub-element | Parsed on body/geom/actuator/sensor | **Not implemented** — no plugin sub-element support | | |
| Model plugin fields | `nplugin`, `body_plugin`, etc. on `mjModel` | **Not implemented** — no fields on Model | | |
| Data plugin state | `plugin_state`, `plugin_data` on `mjData` | **Not implemented** — no fields on Data | | |
| Actuator dispatch | `compute()` called in `mj_fwdActuation()` | **No hook** | | |
| Passive dispatch | `compute()` called in `mj_passive()` | **No hook** (user callback exists, no plugin dispatch) | | |
| Sensor dispatch | `compute_plugin_sensors()` at 3 stages | **No hook** | | |
| Advance | `advance()` called in `mj_advance()` | **No hook** | | |
| `mjSENS_PLUGIN` sensor type | Enum variant `mjSENS_PLUGIN` | **Missing** — no `MjSensorType::Plugin` | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. Plugin Trait and Types

**Grade:**

**Spec says:**
Create `plugin.rs` with `Plugin` trait (single-trait design, AD-1), `PluginCapabilities`
bitfield, `PluginCapabilityBit` enum, `PluginStage` enum, `PluginRegistry` struct.
All types `Send + Sync`. Default implementations for all optional trait methods.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Model Plugin Fields

**Grade:**

**Spec says:**
Add 15+ plugin fields to `Model` struct: `nplugin`, `npluginstate`, `body_plugin`,
`geom_plugin`, `actuator_plugin`, `sensor_plugin`, `plugin_objects`, `plugin_needstage`,
`plugin_capabilities`, `plugin_stateadr`, `plugin_statenum`, `plugin_attr`,
`plugin_attradr`, `plugin_attrnum`, `plugin_name`. Initialize all to empty/zero.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Data Plugin Fields

**Grade:**

**Spec says:**
Add `plugin_state: Vec<f64>` (length `npluginstate`) and
`plugin_data: Vec<Option<Box<dyn Any + Send + Sync>>>` (length `nplugin`) to `Data`.
Initialize in `make_data()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Enum Variants

**Grade:**

**Spec says:**
Add `Plugin` variant to `MjSensorType` (with `dim() → 0`) and `MjObjectType`.
Handle all exhaustive match sites. `data_kind()` falls through to `Real` via
existing catch-all.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. MJCF Types

**Grade:**

**Spec says:**
Add `MjcfPluginConfig`, `MjcfPluginInstance`, `MjcfExtensionPlugin`, `MjcfExtension`,
`MjcfPluginRef` to `sim-mjcf/src/types.rs`. Add `extensions: Vec<MjcfExtension>` to
`MjcfModel`. Add `plugin: Option<MjcfPluginRef>` to `MjcfBody`, `MjcfActuator`,
`MjcfSensor`, `MjcfGeom`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. MJCF Parsing

**Grade:**

**Spec says:**
Parse `<extension>` top-level element with `parse_extension()`. Parse `<plugin>/<instance>/<config>`
hierarchy. Parse per-element `<plugin>` sub-elements on body/geom via `parse_plugin_ref()`.
Validate: duplicate config keys rejected. Validate: instance ref + inline config rejected.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Builder Plugin Resolution

**Grade:**

**Spec says:**
New `builder/plugin.rs`. `resolve_and_assign_plugins()` resolves MJCF plugin references
against `PluginRegistry`. Creates `ResolvedPlugin` instances. Populates Model plugin
arrays (nplugin, plugin_objects, stateadr/num, capabilities, attr). Sets sensor_dim
for plugin sensors via `nsensordata()`. Builder `build()` takes `registry: &PluginRegistry`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Forward Pipeline Dispatch Hooks

**Grade:**

**Spec says:**
Four dispatch points: (8a) actuator — two separate loops (act_dot then compute) in
`mj_fwd_actuation()`; (8b) passive — after user callback in `mj_passive()`; (8c) sensor —
`compute_plugin_sensors()` at Pos/Vel/Acc stages with stage matching (needstage==stage
OR stage==Pos && needstage==None), plus cutoff application; (8d) advance — at end of
`integrate()`. All guarded by `nplugin > 0` fast path.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. Plugin Lifecycle in make_data

**Grade:**

**Spec says:**
Call `plugin.init(model, data, i)` for each plugin instance in `make_data()`. Panic on
init failure.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S10. Plugin Reset Dispatch

**Grade:**

**Spec says:**
In `reset_data()`, call `plugin.reset(model, state_slice, i)` for each plugin instance.
State slice is `data.plugin_state[adr..adr+num]`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S11. Module Registration

**Grade:**

**Spec says:**
Add `pub mod plugin;` to `sim/L0/core/src/lib.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Plugin trait defined and usable (code review) | — (code review) | | |
| AC2 | Plugin registry works | T1 | | |
| AC3 | MJCF `<extension>` parses correctly | T2, T13 | | |
| AC4 | Duplicate config key rejected | T3 | | |
| AC5 | Plugin on body parses | T4 | | |
| AC6 | Model plugin fields populated | T5, T14 | | |
| AC7 | Data plugin state allocated | T6 | | |
| AC8 | Actuator plugin compute called | T7, T15 | | |
| AC9 | Passive plugin compute called | T8, T15 | | |
| AC10 | Sensor plugin stage dispatch | T9 | | |
| AC11 | Zero plugins — no regression | T10 | | |
| AC12 | Plugin advance called during integration | T11 | | |
| AC13 | MjSensorType::Plugin enum variant (code review) | — (code review) | | |
| AC14 | Unknown plugin name rejected | T12 | | |
| AC15 | Plugin reset called during data reset | T16 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Plugin registry registration and lookup | | | |
| T2 | Parse `<extension>` with instances and config | | | |
| T3 | Duplicate config key rejected | | | |
| T4 | Plugin sub-element on body | | | |
| T5 | Model plugin fields populated after build | | | |
| T6 | Data plugin state allocation | | | |
| T7 | Actuator plugin dispatch end-to-end | | | |
| T8 | Passive plugin dispatch end-to-end | | | |
| T9 | Sensor plugin stage dispatch | | | |
| T10 | Zero-plugin regression | | | |
| T11 | Plugin advance in integration | | | |
| T12 | Unknown plugin name error | | | |
| T13 | Plugin with empty config (supplementary) | | | |
| T14 | Multiple plugin instances (supplementary) | | | |
| T15 | Multi-capability plugin dispatched for both capabilities | | | |
| T16 | Plugin reset restores state | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T13 | Plugin with empty config — no `<config>` children is valid | | Boundary: empty config |
| T14 | Multiple plugin instances (3+) of same type — verifies instance ID assignment and state address computation | | Multi-instance |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Zero plugins | Most models won't use plugins; dispatch must be zero-cost | | | |
| Plugin with nstate=0 | Valid: stateless plugins exist | | | |
| Multi-capability plugin | ACTUATOR\|PASSIVE gets compute() twice per step | | | |
| Sensor needstage=None | Maps to POS stage (MuJoCo convention) | | | |
| Duplicate config key | MuJoCo rejects this | | | |
| Unknown plugin name | Must error, not silently skip | | | |
| Instance reference + inline config | MuJoCo rejects this combination | | | |
| Plugin on body (passive) | Body-level plugin for passive forces | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `<extension>` parsing: silently skipped → parsed into `MjcfExtension` | | |
| `MjSensorType` enum: no `Plugin` variant → `Plugin` variant added | | |
| `MjObjectType` enum: no `Plugin` variant → `Plugin` variant added | | |
| Model struct: no plugin fields → plugin fields added | | |
| Data struct: no plugin fields → plugin state/data added | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/plugin.rs` (new) | | |
| `sim/L0/core/src/lib.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` | | |
| `sim/L0/core/src/types/data.rs` | | |
| `sim/L0/core/src/types/enums.rs` | | |
| `sim/L0/core/src/forward/actuation.rs` | | |
| `sim/L0/core/src/forward/passive.rs` | | |
| `sim/L0/core/src/sensor/mod.rs` | | |
| `sim/L0/core/src/integrate/mod.rs` | | |
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/builder/plugin.rs` (new) | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All 2,297+ existing tests | Pass (unchanged) — plugin system purely additive | | |
| `MjSensorType` exhaustive matches | Compile error until `Plugin` arm added | | |
| `MjObjectType` exhaustive matches | Compile error until `Plugin` arm added | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Plugin callbacks | C function pointers → Rust trait methods (`fn compute(&self, ...)`) | | |
| Plugin registry | Global thread-safe table → `PluginRegistry` struct (no global state) | | |
| Instance ID | `int instance` (0-based) → `usize` (0-based) | | |
| Capability flags | `int capabilityflags` (C bitfield) → `PluginCapabilities` (manual bitfield) | | |
| Plugin state | `mjtNum*` raw pointer → `Vec<f64>` + safe slices `&mut [f64]` | | |
| Plugin data | `uintptr_t*` (opaque C pointer) → `Vec<Option<Box<dyn Any + Send + Sync>>>` | | |
| Sensor type | `mjSENS_PLUGIN` → `MjSensorType::Plugin`, `dim() → 0` (variable) | | |
| Config attributes | `char*` concatenated → `Vec<String>` + address arrays | | |
| Stage matching | `needstage == stage \|\| (stage==POS && needstage==NONE)` → same Rust logic | | |

---

## 6b. Architecture Decision Compliance

The spec defines 3 architecture decisions. Verify implementation honored each.

| AD | Decision | Spec Says | Implementation Follows? | Notes |
|----|----------|-----------|------------------------|-------|
| AD-1 | Single `Plugin` trait vs per-capability traits | Single `Plugin` trait with default implementations for all optional methods. Multi-capability natural, simple registry (`Arc<dyn Plugin>`). | | |
| AD-2 | Plugin registry location | `PluginRegistry` struct passed to builder at model construction — no global state. Builder resolves plugin names against registry during compilation. | | |
| AD-3 | Plugin state access pattern | Safe `&mut [f64]` slices via `data.plugin_state[adr..adr+num]`. Zero-cost (pointer+length). | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution. Check for: TODO/FIXME/HACK comments,
hardcoded values, loose tolerances, missing edge-case guards, placeholder error
handling (unwrap() in non-test code), algorithm deviations, dead code.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| SDF collision dispatch | Out of Scope, bullet 1 | | DT-171 | |
| Resource providers | Out of Scope, bullet 2 | | DT-170 | |
| Decoders | Out of Scope, bullet 3 | | DT-170 | |
| Plugin visualization | Out of Scope, bullet 4 | | — | |
| Dynamic library loading | Out of Scope, bullet 5 | | — | |
| Plugin copy/destroy callbacks | Out of Scope, bullet 6 | | — | |
| Data Clone with plugins | Out of Scope, bullet 7 | | — | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution — check Session 15 commit notes
for any items discovered during implementation.}

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution.}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings}
**Fmt:** {clean / issues}

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:**

**Items fixed during review:**
{To be filled during review execution.}

**Items to fix before shipping:**
{To be filled during review execution.}

**Items tracked for future work:**
{To be filled during review execution.}
