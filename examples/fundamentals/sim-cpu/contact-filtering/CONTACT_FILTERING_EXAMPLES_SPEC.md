# Contact Filtering Examples — Spec

## Overview

Contact filtering controls which geometry pairs can collide. The engine has
four mechanisms:

1. **Bitmask filtering** — `contype`/`conaffinity` on each geom.
   Rule: contact if `(g1.contype & g2.conaffinity) != 0 || (g2.contype & g1.conaffinity) != 0`.
2. **Parent-child auto-exclusion** — Bodies connected by a joint don't
   self-collide. World body (0) is exempt (ground planes always collide).
3. **Explicit exclusion** — `<exclude body1="a" body2="b"/>` suppresses all
   contact between two bodies.
4. **Explicit pair override** — `<pair geom1="a" geom2="b" .../>` bypasses
   bitmask filtering and applies custom solver parameters.

Defaults: `contype=1`, `conaffinity=1` (everything collides with everything).

## Directory layout

```
contact-filtering/
├── CONTACT_FILTERING_EXAMPLES_SPEC.md   (this file)
├── README.md
├── stress-test/          # headless — 12 checks, implement FIRST
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── bitmask/              # visual — contype/conaffinity bitmask demo
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── exclude-pairs/        # visual — <exclude> body-pair suppression
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── ghost-layers/         # visual — collision layers via bitmasks
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

## Implementation order

1. **stress-test** (headless, no Bevy) — validates all engine invariants
2. **bitmask** (visual) — one concept: contype/conaffinity bitmask rule
3. **exclude-pairs** (visual) — one concept: `<exclude>` body-pair suppression
4. **ghost-layers** (visual) — one concept: collision layers via bitmasks

---

## 1. `stress-test/` — Headless Validation

**Concept:** Exhaustive engine-level verification of all four filtering
mechanisms. No window, no Bevy — pure sim-core assertions.

**Package:** `example-contact-filtering-stress-test`

**Dependencies:** `sim-core`, `sim-mjcf`, `nalgebra`

### MJCF Models

**Model A** — Bitmask basics (checks 1–3):
Two spheres + ground plane. Sphere "collider" has `contype=1 conaffinity=1`
(default). Sphere "ghost" has `contype=0 conaffinity=0`. Both drop under
gravity. Ghost falls through the ground; collider rests on it.

**Model A2** — OR-rule verification (check 4):
One sphere + ground plane. Sphere has `contype=0 conaffinity=1`. Ground is
default (`contype=1 conaffinity=1`). Rule: `(0 & 1) || (1 & 1)` = true.
Sphere rests on ground despite `contype=0` — proves the OR semantics
(one matching side is enough).

**Model B** — Selective bitmask (checks 5–6):
Three spheres + ground plane.
- "sphere_a": `contype=1 conaffinity=3` (layer 0+1 type, layer 0+1 affinity)
- "sphere_b": `contype=2 conaffinity=2` (layer 1 type, layer 1 affinity)
- "sphere_c": `contype=4 conaffinity=4` (layer 2 type, layer 2 affinity)

A–B should collide (A.contype=1 & B.conaffinity=2 = 0, but B.contype=2 &
A.conaffinity=3 = 2 ≠ 0). B–C should NOT collide (2&4=0 and 4&2=0).

**Model C** — Parent-child auto-exclusion (checks 7–8):
A non-world parent body (attached to worldbody via freejoint) with a hinge
child. Parent geom and child geom overlap spatially. They should NOT collide
(auto-excluded because `body_parent[child] = parent` and both are non-zero).
A ground plane on the world body (body 0) SHOULD still collide with the child
(world body is exempt from parent-child filter: the `body1 != 0 && body2 != 0`
guard means body-0 geoms always pass through to the bitmask check).

**Model D** — Explicit exclusion (checks 9–10):
Two free bodies "box_a" and "box_b" placed overlapping. `<exclude body1="box_a"
body2="box_b"/>`. Despite overlap, `ncon` between them should be 0. A third
body "box_c" at the same position with no exclusion DOES collide with "box_a".

**Model E** — Explicit pair override (checks 11–12):
Two geoms "left" and "right" with `contype=1 conaffinity=0` (would normally
NOT collide by bitmask). An explicit `<pair geom1="left" geom2="right"
condim="1" friction="0"/>` forces them to collide with custom parameters.
Validates that: (a) contact is generated despite failing bitmask, (b) the
contact uses `condim=1` (frictionless) from the pair override.

### Checks (12)

| #  | Name | Model | What it verifies |
|----|------|-------|-----------------|
| 1  | contype=0 falls through ground | A | Ghost sphere z < -1 after 500 steps (no ground contact). |
| 2  | Default bitmask collides | A | Collider sphere rests on ground (z ≈ sphere radius). |
| 3  | ncon=0 for ghost | A | No contacts involving ghost geom after settling. |
| 4  | OR rule — contype=0 still collides if conaffinity matches | A2 | Sphere with `contype=0 conaffinity=1` rests on ground: `(0&1)\|\|(1&1)` = true. |
| 5  | Cross-layer bitmask collides | B | A–B contact count > 0 when placed touching. |
| 6  | Disjoint layers no contact | B | B–C contact count = 0 when placed touching. |
| 7  | Parent-child auto-excluded | C | Overlapping parent-child geoms produce ncon=0 between them. |
| 8  | World body exempt from auto-exclusion | C | Ground plane (body 0) collides with child body. |
| 9  | Exclude suppresses contact | D | box_a ↔ box_b ncon=0 despite overlap. |
| 10 | Exclude is bidirectional | D | Same result regardless of body1/body2 ordering in `<exclude>`. |
| 11 | Pair bypasses bitmask | E | Contact generated between geoms that fail bitmask. |
| 12 | Pair overrides condim | E | Contact dim matches pair-specified condim=1. |

### main() structure

```
fn check(name, pass, detail) -> bool          // shared helper
fn check_1_ghost_falls_through() -> (u32, u32)
fn check_2_default_collides() -> (u32, u32)
...
fn check_12_pair_condim() -> (u32, u32)
fn main()  // run all, print report, exit(1) on failure
```

---

## 2. `bitmask/` — Contype/Conaffinity Bitmask Demo

**Concept:** Bitmask filtering rule. Four spheres with different
contype/conaffinity values fall onto a ground plane.

**Package:** `example-contact-filtering-bitmask`

**Dependencies:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`, `nalgebra`

### Scene

Ground plane + four spheres dropped from height 1.0, spaced along X:

| Sphere | Color | contype | conaffinity | Behavior |
|--------|-------|---------|-------------|----------|
| A "solid" | Blue | 1 | 1 | Collides with ground, rests on surface |
| B "one-way" | Green | 1 | 0 | Falls through — type matches nothing's affinity=0, 0 matches nothing |
| C "receiver" | Orange | 0 | 1 | Falls through — contype=0, and ground's contype=1 & C's affinity=1 → wait, let me recalculate. Ground: ct=1,ca=1. C: ct=0,ca=1. Rule: (1&1)≠0 → YES, collides. Need to pick values more carefully. |

Let me recalculate to get 4 distinct behaviors:

Ground plane: contype=1, conaffinity=1 (default).

| Sphere | contype | conaffinity | Ground collision? | Why |
|--------|---------|-------------|-------------------|-----|
| A | 1 | 1 | YES | (1&1)∨(1&1) = true |
| B | 2 | 2 | NO | (2&1)∨(1&2) = 0∨0 = false |
| C | 0 | 0 | NO | (0&1)∨(1&0) = 0∨0 = false |
| D | 1 | 0 | YES | (1&1)∨(1&0) = 1∨0 = true |

So A and D rest on ground; B and C fall through.

For sphere-sphere: place A and D close enough to collide.
A(ct=1,ca=1) vs D(ct=1,ca=0): (1&0)∨(1&1) = 0∨1 = true → they collide.

| Sphere | Color | contype | conaffinity | Rests on ground? | Collides with A? |
|--------|-------|---------|-------------|------------------|-----------------|
| A "full" | Steel blue | 1 | 1 | Yes | — |
| B "layer-2" | Green | 2 | 2 | No (falls through) | No |
| C "disabled" | Red (translucent) | 0 | 0 | No (falls through) | No |
| D "type-only" | Gold | 1 | 0 | Yes | Yes |

### HUD

- Label each sphere with name + contype/conaffinity values
- Show "COLLIDING" / "FALLING" status per sphere
- Show active contact count

### Validation (in-window)

After 2s of sim time:
- A.z ≈ sphere_radius (resting)
- B.z < -1.0 (fell through)
- C.z < -1.0 (fell through)
- D.z ≈ sphere_radius (resting)

---

## 3. `exclude-pairs/` — Explicit Body-Pair Exclusion

**Concept:** `<exclude body1="a" body2="b"/>` suppresses all contact between
two bodies, regardless of bitmask.

**Package:** `example-contact-filtering-exclude-pairs`

**Dependencies:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`, `nalgebra`

### Scene

Three boxes dropped side by side onto a ground plane:

| Body | Color | Excluded with? | Behavior |
|------|-------|---------------|----------|
| "box_a" | Blue | box_b | Passes through box_b, rests on ground |
| "box_b" | Red (translucent) | box_a | Passes through box_a, rests on ground |
| "box_c" | Gold | (none) | Collides normally with everything |

- box_a and box_b start at the same X position but slightly different heights
  so one falls through the other. box_c is offset to the side.
- `<exclude body1="box_a" body2="box_b"/>` in MJCF.
- All three have default contype=1, conaffinity=1. Without the exclude,
  box_a and box_b would collide. The exclude overrides this.

### HUD

- Label each box
- Show "EXCLUDED" relationship between box_a ↔ box_b
- Show per-pair contact count (a↔b = 0, a↔ground > 0, c↔ground > 0)

### Validation (in-window)

After 2s:
- box_a and box_b both rest on ground (z ≈ box half-size)
- box_c rests on ground
- No contacts between box_a and box_b at any point

---

## 4. `ghost-layers/` — Collision Layers via Bitmasks

**Concept:** Using bitmasks to create "physics layers" — solid objects
collide with each other, sensor/ghost objects pass through each other but
interact with solid objects.

**Package:** `example-contact-filtering-ghost-layers`

**Dependencies:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`, `nalgebra`

### Scene

Two layers:

| Layer | contype | conaffinity | Color style |
|-------|---------|-------------|-------------|
| Solid (bit 0) | 1 | 1 | Opaque metals |
| Ghost (bit 1) | 2 | 3 | Translucent |

Ghost `conaffinity=3` (bits 0+1) means ghosts collide with both solid AND
ghost objects. But solid `conaffinity=1` (bit 0 only) means solids only
collide with other solids. Let's verify:

- Solid A vs Solid B: (1&1)∨(1&1) = true ✓
- Solid vs Ghost: (1&3)∨(2&1) = 1∨2 = true ✓
- Ghost vs Ghost: (2&3)∨(2&3) = 2∨2 = true — but we want ghosts to pass
  through each other.

Recalculate. We want:
- Solid ↔ Solid: collide
- Solid ↔ Ghost: collide (ghost detects solid)
- Ghost ↔ Ghost: NO collision

| Layer | contype | conaffinity |
|-------|---------|-------------|
| Solid | 1 | 3 | 
| Ghost | 2 | 1 |

- Solid vs Solid: (1&3)∨(1&3) = 1∨1 = true ✓
- Solid vs Ghost: (1&1)∨(2&3) = 1∨2 = true ✓
- Ghost vs Ghost: (2&1)∨(2&1) = 0∨0 = false ✓

Objects:
- 2 solid boxes (blue, gold) — stack on each other on the ground
- 2 ghost spheres (translucent green) — pass through each other, but rest
  on top of the solid stack

### HUD

- Label each object with layer assignment
- Show which pairs are colliding vs passing through
- Active contact count

### Validation (in-window)

After 3s:
- Solid boxes stacked on ground
- Ghost spheres resting on solid surface (not passing through solids)
- Ghost spheres at different X positions but same Z (not stacked — they
  passed through each other)

---

## Naming conventions

All packages follow: `example-contact-filtering-{name}`

All examples follow the established patterns from free-joint/, keyframes/,
mocap-bodies/:
- Inline MJCF as `const` strings
- Stress test: headless, `check()` helper, `(u32, u32)` return, exit(1) on failure
- Visual: Bevy app with `OrbitCameraPlugin`, `PhysicsHud`, `ValidationHarness`
