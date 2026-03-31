# Keyframe Name→Index Lookup Spec

**Date:** 2026-03-31
**Status:** Implemented — 10/10 AC pass
**Scope:** Add `Keyframe` to the `name2id`/`id2name` infrastructure

## Problem

Every named element type in sim-core (Body, Joint, Geom, Site, Tendon,
Actuator, Sensor, Mesh, Hfield, Equality) supports O(1) name→index lookup via
`Model::name2id(ElementType::Foo, "name")` and a convenience method
`model.foo_id("name")`. Keyframes are the only named element missing from this
infrastructure.

Current workaround: `model.keyframes.iter().position(|k| k.name == "home")` —
O(n) linear scan, no integration with the unified `name2id`/`id2name` API.

MuJoCo equivalent: `mj_name2id(m, mjOBJ_KEY, "name")`.

## Changes

### 1. `ElementType` enum (`enums.rs`)

Add variant:

```rust
/// Keyframe elements (indexed by keyframe_id).
Keyframe,
```

### 2. `Model` struct (`model.rs`)

Add field in the `Name↔Index Lookup (§59)` section:

```rust
/// Keyframe name → index. Populated from `keyframes[i].name`.
pub keyframe_name_to_id: HashMap<String, usize>,
```

### 3. `Model::name2id()` (`model.rs`)

Add match arm:

```rust
ElementType::Keyframe => self.keyframe_name_to_id.get(name).copied(),
```

### 4. `Model::id2name()` (`model.rs`)

Add match arm. Unlike other elements that use separate `*_name: Vec<Option<String>>`
arrays, keyframe names live on the `Keyframe` struct directly. Return `None` for
empty-string names (unnamed keyframes) to match the convention that unnamed
elements return `None`:

```rust
ElementType::Keyframe => self
    .keyframes
    .get(id)
    .map(|k| k.name.as_str())
    .filter(|n| !n.is_empty()),
```

### 5. Convenience method (`model.rs`)

Add in the "Convenience Name Lookups" section:

```rust
/// Look up keyframe ID by name. Returns `None` if not found.
#[inline]
#[must_use]
pub fn keyframe_id(&self, name: &str) -> Option<usize> {
    self.keyframe_name_to_id.get(name).copied()
}
```

### 6. `Model::empty()` (`model_init.rs`)

Add field initialization alongside the other 10 `*_name_to_id` maps:

```rust
keyframe_name_to_id: HashMap::new(),
```

This is the factory method used by all test helpers (`n_link_pendulum`,
`free_body`, etc.). Without this, test code won't compile.

### 7. Builder struct literal (`build.rs`)

In `ModelBuilder::build()` → `assemble_model()`, initialize the field:

```rust
keyframe_name_to_id: HashMap::new(), // Populated post-build in model_from_mjcf()
```

### 8. MJCF builder population (`mod.rs`)

After resolving keyframes (line ~339), populate the map. Only insert non-empty
names (unnamed keyframes are not discoverable by name):

```rust
model.keyframe_name_to_id = model
    .keyframes
    .iter()
    .enumerate()
    .filter(|(_, kf)| !kf.name.is_empty())
    .map(|(i, kf)| (kf.name.clone(), i))
    .collect();
```

### 9. URDF builder

No changes needed. The URDF loader (`load_urdf_model`) converts URDF → MJCF XML
and then calls `sim_mjcf::load_model()`, so it flows through the MJCF builder
path. Since URDF has no keyframe concept, the resolved keyframes list is empty,
and the map will be empty.

## Edge cases

| Case | Behavior |
|---|---|
| Unnamed keyframe (`name=""` or omitted) | Not inserted into map. `id2name` returns `None`. |
| Duplicate names | Last-wins (HashMap `.collect()` overwrites earlier entries). This matches the existing codebase convention — all non-validated element types (geom, site, tendon, sensor, equality) use the same last-wins semantics via `.insert()`. MuJoCo uses first-wins (`mj_name2id` returns the lowest index), but duplicate keyframe names are a user error in both engines. No validation added — consistent with 7 of 10 existing element types. |
| Empty model (no keyframes) | Map is empty. All lookups return `None`. |
| URDF-loaded model | Map is empty. All lookups return `None`. |

## Acceptance criteria

### Parsing & lookup

1. **AC01** — `name2id(Keyframe, "home")` returns `Some(0)` for a model with keyframe named "home" at index 0.
2. **AC02** — `name2id(Keyframe, "nonexistent")` returns `None`.
3. **AC03** — `keyframe_id("home")` convenience method returns same result as `name2id(Keyframe, "home")`.
4. **AC04** — `id2name(Keyframe, 0)` returns `Some("home")` for a named keyframe.
5. **AC05** — `id2name(Keyframe, 0)` returns `None` for an unnamed keyframe (empty string name).
6. **AC06** — `id2name(Keyframe, 999)` returns `None` (out of bounds).

### Multiple keyframes

7. **AC07** — Model with 3 named keyframes: all three resolve correctly via `name2id`.
8. **AC08** — Model with mix of named and unnamed keyframes: named ones resolve, unnamed ones are absent from map.

### Integration with reset

9. **AC09** — Round-trip: `name2id` → `reset_to_keyframe` → verify state matches expected keyframe. Proves the index returned by name lookup is correct for state restoration.

### Builder

10. **AC10** — URDF-loaded model: `keyframe_name_to_id` is empty, `name2id(Keyframe, _)` always returns `None`.

## Files touched

| File | Change |
|---|---|
| `sim/L0/core/src/types/enums.rs` | +2 lines (variant + doc comment) |
| `sim/L0/core/src/types/model.rs` | +8 lines (field, name2id arm, id2name arm, convenience method) |
| `sim/L0/core/src/types/model_init.rs` | +1 line (HashMap::new() in Model::empty()) |
| `sim/L0/mjcf/src/builder/build.rs` | +1 line (field init in assemble_model struct literal) |
| `sim/L0/mjcf/src/builder/mod.rs` | +6 lines (populate map after keyframe resolution) |
| `sim/L0/tests/integration/keyframes.rs` | +~80 lines (AC01–AC10 tests) |

**Total: ~105 lines.** Adding a variant to `ElementType` is a
compile-guided breakage — exactly 2 match arms in `model.rs` (`name2id` and
`id2name`) need the new `Keyframe` arm, which this spec already covers. No
downstream crates outside sim-core match on `ElementType` exhaustively.
