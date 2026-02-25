# DT-16 + DT-90: Flex Parsing Conformance Fixes

**Status:** Draft
**Phase:** Roadmap Phase 1 — Correctness bugs (last 2 remaining)
**Tier:** T1 (mechanical — no iterative design needed)

---

## Overview

Two flex-related MJCF parsing bugs that cause data loss or non-conformant behavior.
Both are in the parser → builder → model pipeline, both are T1 mechanical fixes.

---

## DT-16: Flex `density` attribute location wrong

### Problem

`parse_flex_attrs()` parses `density` from the top-level `<flex>` / `<flexcomp>`
element attributes (`parser.rs:2653-2657`). MuJoCo has **no** `density` attribute
on `<flex>`, `<flexcomp>`, or any flex child element.

```xml
<!-- What our parser accepts (WRONG — not in MuJoCo spec): -->
<flexcomp density="1000.0" dim="2" type="grid" count="5 5 1"/>

<!-- MuJoCo conformant (mass, not density): -->
<flexcomp mass="1.5" dim="2" type="grid" count="5 5 1"/>
```

### Current behavior

- `density` parsed from `<flex>`/`<flexcomp>` element attributes (non-conformant)
- Used as fallback in `compute_vertex_masses()` when `mass` is `None`
- Comments already say `// Non-standard extension` and reference `DEFERRED(#27E)`

### What MuJoCo does

MuJoCo sets flex mass via `<flexcomp mass="...">` (total mass distributed
uniformly across vertices). There is no density-based mass lumping. Our `mass`
attribute parsing already works correctly (`parser.rs:2650-2651`).

### Fix

**Remove the non-conformant `density` parsing from `parse_flex_attrs()`.**

The field stays on `MjcfFlex` (for internal use / future extensions), but XML
parsing must not accept it on `<flex>` or `<flexcomp>` elements.

#### Files to change

| File | Change |
|------|--------|
| `sim/L0/mjcf/src/parser.rs:2653-2657` | Delete `density` parsing from `parse_flex_attrs()` |

#### Code change

```rust
// DELETE these lines (parser.rs:2653-2657):
    // Non-standard extension: density on <flex> for element-based mass lumping.
    // Fallback when mass attr is not present.
    if let Some(s) = get_attribute_opt(e, "density") {
        flex.density = s.parse().unwrap_or(1000.0);
    }
```

The default `density: 1000.0` on `MjcfFlex` remains unchanged — it's just no
longer overridable from XML. The `mass` path (`compute_vertex_masses()` uniform
distribution) is the conformant API.

#### Tests

1. **Regression test:** Parse a `<flexcomp>` with `density="500"` — verify the
   attribute is silently ignored (density stays at default 1000.0).
2. **Positive test:** Parse `<flexcomp mass="1.5">` — verify vertex masses are
   uniform `1.5 / n_vertices`.

---

## DT-90: `flex_friction` scalar → `Vector3<f64>`

### Problem

Flex friction is stored as `f64` (scalar) but MuJoCo's `<contact friction="...">`
accepts 3 values: sliding, torsional, rolling. The parser (`parser.rs:2673-2682`)
explicitly takes only the first value and discards torsional/rolling:

```rust
// Current (WRONG) — parser.rs:2673-2682:
if let Some(s) = get_attribute_opt(e, "friction") {
    // MuJoCo friction is real(3): "slide torsion rolling".
    // MjcfFlex.friction is f64 (scalar = sliding component only).
    // Parse first whitespace-separated value ...
    flex.friction = s
        .split_whitespace()
        .next()
        .and_then(|t| t.parse().ok())
        .unwrap_or(1.0);
}
```

This flows through to the contact parameter combination in `collision/mod.rs:172-211`,
where the scalar is replicated across all 5 friction slots — torsional and rolling
friction data from the MJCF is lost.

### What MuJoCo does

`<flex><contact friction="1.0 0.005 0.0001"/>` → 3 values stored per flex:
- `[0]` = sliding (mapped to friction slots 0, 1)
- `[1]` = torsional (mapped to friction slot 2)
- `[2]` = rolling (mapped to friction slots 3, 4)

Same pattern as `geom_friction: Vec<Vector3<f64>>`.

### Fix

Change `flex_friction` from `Vec<f64>` to `Vec<Vector3<f64>>` end-to-end.

#### Files to change

| File | Line(s) | Change |
|------|---------|--------|
| `sim/L0/mjcf/src/types.rs:3472-3473` | `friction: f64` → `friction: Vector3<f64>` |
| `sim/L0/mjcf/src/types.rs:3541` | Default `friction: 1.0` → `friction: Vector3::new(1.0, 0.005, 0.0001)` |
| `sim/L0/mjcf/src/parser.rs:2673-2682` | Parse up to 3 values, fill MuJoCo defaults for missing components |
| `sim/L0/mjcf/src/builder/mod.rs:642` | `flex_friction: Vec<f64>` → `Vec<Vector3<f64>>` |
| `sim/L0/core/src/types/model.rs:328` | `flex_friction: Vec<f64>` → `Vec<Vector3<f64>>` |
| `sim/L0/core/src/collision/mod.rs:172,178` | `[f, f, f, f, f]` → `[f.x, f.x, f.y, f.z, f.z]` |
| `sim/L0/core/src/collision/mod.rs:203-211` | `ff.max(gf.x)` etc → `ff.x.max(gf.x)`, `ff.y.max(gf.y)`, `ff.z.max(gf.z)` |

#### Parser change

```rust
// AFTER (parser.rs — parse_flex_contact_attrs):
if let Some(s) = get_attribute_opt(e, "friction") {
    let parts: Vec<f64> = s.split_whitespace()
        .filter_map(|t| t.parse().ok())
        .collect();
    flex.friction = Vector3::new(
        parts.first().copied().unwrap_or(1.0),
        parts.get(1).copied().unwrap_or(0.005),
        parts.get(2).copied().unwrap_or(0.0001),
    );
}
```

`parse_vector3()` can't be used here — it errors on <3 values, and
`parse_flex_contact_attrs` doesn't return `Result`. MuJoCo accepts 1-3 values,
filling defaults for missing components: `1.0 0.005 0.0001` (sliding,
torsional, rolling).

This matches how geom friction defaults work in MuJoCo.

#### Collision change

```rust
// AFTER (collision/mod.rs — flex-priority path):
let f = model.flex_friction[flex_id];
return (
    model.flex_condim[flex_id],
    gap,
    model.flex_solref[flex_id],
    model.flex_solimp[flex_id],
    [f.x, f.x, f.y, f.z, f.z],  // same pattern as geom
);

// AFTER (collision/mod.rs — equal-priority path):
let ff = model.flex_friction[flex_id];
let gf = model.geom_friction[geom_idx];
let fri = [
    ff.x.max(gf.x),
    ff.x.max(gf.x),
    ff.y.max(gf.y),
    ff.z.max(gf.z),
    ff.z.max(gf.z),
];
```

#### Default value

MuJoCo default friction for all contact-capable elements: `1.0 0.005 0.0001`
(sliding, torsional, rolling). Verify this is correct for flex specifically.

#### Tests

1. **Parse test:** `<flex><contact friction="0.8 0.01 0.002"/>` → verify all 3
   components stored correctly.
2. **Parse test (1 value):** `<flex><contact friction="0.5"/>` → verify sliding
   component set, torsional/rolling at defaults.
3. **Contact param test (flex priority):** Verify 5-element friction array has
   correct mapping `[slide, slide, torsion, roll, roll]`.
4. **Contact param test (equal priority):** Verify element-wise max with geom
   friction uses per-component comparison, not scalar.

---

## Implementation Order

1. **DT-90 first** — the friction type change is isolated and won't affect DT-16.
2. **DT-16 second** — removing density parsing is a 4-line deletion.
3. **Tests** — add parser + collision tests for both.
4. **Verify** — `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics`

## Risk

Both changes are **breaking** for users who:
- DT-16: Use `density="..."` on `<flex>` or `<flexcomp>` (non-standard, should use `mass`)
- DT-90: None (additive — preserves existing sliding friction, adds torsional/rolling)

DT-16 is intentionally breaking: removing a non-conformant extension to match MuJoCo.

## Verification

Run the 11-crate baseline after implementation:
```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics \
  -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf \
  -p sim-types -p sim-simd
```
Expected: 2,007+ passed, 0 failed.
