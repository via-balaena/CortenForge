# Composites Examples Spec

**Domain:** `examples/fundamentals/sim-cpu/composites/`
**Engine feature:** `<composite type="cable">` — rigid-body chain with ball joints
**Depends on:** sim-core (physics), sim-mjcf (parsing + cable expansion), sim-bevy (rendering)

---

## Background

The composite system expands a single `<composite type="cable">` MJCF element
into a chain of N-1 bodies connected by ball joints, with auto-generated
geoms, sites, and contact exclusions. Cables are rigid-body chains — not
deformable — useful for ropes, cables, tethers, and flexible tubes. The
engine implements discrete Bishop frame propagation for segment orientation,
4 curve shapes (line, cos, sin, zero) for initial geometry, 3 initial modes
(ball, free, none), and 3 geom types (capsule, cylinder, box).

Implementation: `sim/L0/mjcf/src/builder/composite.rs` (696 lines),
12 integration tests (T1-T12), 18 acceptance criteria — all pass.

---

## Directory Layout

```
composites/
├── README.md                        # Domain overview (museum plaque)
├── hanging-cable/
│   ├── Cargo.toml                   # example-composite-hanging-cable
│   ├── README.md                    # Museum plaque
│   └── src/
│       └── main.rs                  # Visual — pinned cable under gravity
├── cable-catenary/
│   ├── Cargo.toml                   # example-composite-cable-catenary
│   ├── README.md                    # Museum plaque
│   └── src/
│       └── main.rs                  # Visual — both-ends-fixed passive sag
├── cable-loaded/
│   ├── Cargo.toml                   # example-composite-cable-loaded
│   ├── README.md                    # Museum plaque
│   └── src/
│       └── main.rs                  # Visual — midpoint load on fixed cable
└── stress-test/
    ├── Cargo.toml                   # example-composite-stress-test
    └── src/
        └── main.rs                  # Headless — 8 checks
```

---

## Example 1: `hanging-cable` — Pendulum Chain Under Gravity

### One concept

A cable pinned at one end hangs under gravity. Resolution comparison.

### What you see

Three cables (red, green, blue) pinned at equally-spaced anchor points along
a horizontal bar. Each starts as a straight line and sags into a natural
hanging curve. The 5-segment cable looks angular; the 20-segment cable looks
smooth and rope-like. All three settle to approximately the same tip height,
confirming convergence.

### MJCF Model (one per cable)

```xml
<mujoco model="hanging-cable">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <body name="anchor_5" pos="-0.4 0 1">
      <composite type="cable" prefix="C5" count="6 1 1"
                 initial="none" curve="l 0 0" size="0.8">
        <joint kind="main" damping="0.05"/>
        <geom type="capsule" size="0.01" rgba="0.9 0.2 0.2 1"
              density="500"/>
      </composite>
    </body>
    <!-- Repeat for count=11 (green) and count=21 (blue) at different x offsets -->
  </worldbody>
</mujoco>
```

Key parameters:
- `initial="none"` — first body is rigidly attached to the anchor (pinned)
- `damping=0.05` — ball joint damping for settling
- `density=500` — gives each segment realistic mass
- `curve="l 0 0"` — straight line initial shape (along local x)
- Three cables: count=6 (5 segments), count=11 (10 segments), count=21
  (20 segments)

Note: `count` in `<composite>` is the number of *vertices*. The engine
generates `count - 1` bodies. So count=6 -> 5 bodies, count=11 -> 10,
count=21 -> 20.

### HUD

```
Hanging Cable — Resolution Comparison
  t           4.320
  5-seg  tip  -0.213, 0.000, 0.214
  10-seg tip  -0.209, 0.000, 0.208
  20-seg tip  -0.207, 0.000, 0.205
```

Display tip position (xpos of last body) for each cable.

### Validation (report at t=15s, print every 2s)

| Check | Condition | Threshold |
|-------|-----------|-----------|
| 5-seg tip below anchor | tip_z < anchor_z | — |
| 10-seg tip below anchor | tip_z < anchor_z | — |
| 20-seg tip below anchor | tip_z < anchor_z | — |
| Convergence: 10 vs 20 | \|tip_10 - tip_20\| < \|tip_5 - tip_20\| | — |
| All cables settled | max angular velocity < 0.01 rad/s | 0.01 |

### Camera

Orbit camera at distance ~2.5, looking at the center of the three cables.
Slight elevation (~20°) so you can see the sag shape clearly.

---

## Example 2: `cable-catenary` — Both-Ends-Fixed Catenary

### One concept

A cable pinned at both ends sags under its own weight into a catenary shape.

### What you see

A single gold cable spanning a gap between two fixed pylons (gray boxes). The
cable starts as a straight horizontal line and sags into a smooth catenary
curve under gravity. 15 segments for a clean appearance. The pylons frame
the scene and make the fixed endpoints obvious.

### MJCF Model (sketch)

```xml
<mujoco model="cable-catenary">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>

  <worldbody>
    <!-- Left pylon -->
    <body name="pylon_L" pos="-0.5 0 0.8">
      <geom type="box" size="0.02 0.02 0.4" rgba="0.5 0.5 0.5 1"/>

      <composite type="cable" prefix="A" count="16 1 1"
                 initial="none" curve="l 0 0" size="1.0" offset="0 0 0.4">
        <joint kind="main" damping="0.1"/>
        <geom type="capsule" size="0.008" rgba="0.9 0.75 0.2 1"
              density="800"/>
      </composite>
    </body>

    <!-- Right pylon -->
    <body name="pylon_R" pos="0.5 0 0.8">
      <geom type="box" size="0.02 0.02 0.4" rgba="0.5 0.5 0.5 1"/>
    </body>
  </worldbody>

  <equality>
    <!-- Pin cable's far end to right pylon -->
    <connect body1="AB_last" anchor1="..." body2="pylon_R" anchor2="0 0 0.4"/>
  </equality>
</mujoco>
```

Note: The `connect` constraint's `anchor1` must be in the last cable body's
local frame. The S_last site is at `[edge_length, 0, 0]` in body-local
coordinates — use that as anchor1. Verify during implementation.

### HUD

```
Cable Catenary — Both-Ends-Fixed
  t           6.430
  sag depth   0.082
  span        0.998
```

Sag depth = anchor_z - min(body_z over cable bodies).
Span = |last_body_x - first_body_x|.

### Validation (report at t=15s, print every 2s)

| Check | Condition | Threshold |
|-------|-----------|-----------|
| Cable sags below anchors | min_z < anchor_z | — |
| Cable settled | max angular velocity < 0.01 | 0.01 |
| Span preserved | \|span - 1.0\| < 0.05 | 5% |
| Symmetric sag | \|sag_left_half - sag_right_half\| < 0.01 | 0.01 |

### Camera

Orbit camera at distance ~2.0, side view (azimuth ~0°) showing the cable
in profile. Slight elevation (~15°) to see the sag shape.

---

## Example 3: `cable-loaded` — Midpoint Load on Fixed Cable

### One concept

An external downward force applied at a cable's midpoint increases deflection.

### What you see

A single cyan cable spanning two pylons (same geometry as cable-catenary).
A visible weight or force indicator at the midpoint pulls the cable downward.
The cable sags significantly more than a passive catenary would — the
midpoint dips well below the rest of the curve, creating a V-like shape
rather than a smooth catenary.

### MJCF Model (sketch)

```xml
<mujoco model="cable-loaded">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>

  <worldbody>
    <!-- Left pylon -->
    <body name="pylon_L" pos="-0.5 0 0.8">
      <geom type="box" size="0.02 0.02 0.4" rgba="0.5 0.5 0.5 1"/>

      <composite type="cable" prefix="A" count="16 1 1"
                 initial="none" curve="l 0 0" size="1.0" offset="0 0 0.4">
        <joint kind="main" damping="0.1"/>
        <geom type="capsule" size="0.008" rgba="0.2 0.8 0.9 1"
              density="800"/>
      </composite>
    </body>

    <!-- Right pylon -->
    <body name="pylon_R" pos="0.5 0 0.8">
      <geom type="box" size="0.02 0.02 0.4" rgba="0.5 0.5 0.5 1"/>
    </body>
  </worldbody>

  <equality>
    <connect body1="AB_last" anchor1="..." body2="pylon_R" anchor2="0 0 0.4"/>
  </equality>
</mujoco>
```

**Midpoint load approach:** Ball joints have rotational DOF only — no
translational DOF to apply a linear force directly via a motor. Options:
- Use `xfrc_applied` directly on the midpoint body (body index of `AB_8`)
  to apply a constant downward force. Simplest, no extra MJCF elements.
- Add a small weight body at the midpoint with a free or slide joint.

The implementation phase will choose the cleanest approach. The key visual:
the cable sags significantly more at the midpoint than a passive catenary.

### HUD

```
Cable Loaded — Midpoint Force
  t           6.430
  sag depth   0.154
  load (N)    5.0
  mid z       0.646
```

### Validation (report at t=15s, print every 2s)

| Check | Condition | Threshold |
|-------|-----------|-----------|
| Cable sags below anchors | min_z < anchor_z | — |
| Midpoint is lowest point | mid_z < all other body z | — |
| Cable settled | max angular velocity < 0.01 | 0.01 |
| Sag exceeds passive catenary | sag_loaded > 0.05 | — |

### Camera

Same framing as cable-catenary — orbit camera at distance ~2.0, side view.

---

## Example 4: `stress-test` — Headless Validation (8 checks)

### Checks

Each check is a self-contained function that loads its own model, simulates,
and returns (pass_count, total_count).

#### Check 1: Body count matches specification

Load a cable with `count="11 1 1"`. Verify `model.nbody == 11` (world + 10
cable bodies). Cable with N vertices generates N-1 bodies.

```
model.nbody == count[0]   (world body + count-1 cable bodies = count)
```

#### Check 2: Joint count and types

Load a cable with `count="8 1 1"` and `initial="ball"` (default). Verify
`model.njnt == 7` (one ball joint per cable body). Verify all joints are
`MjJointType::Ball`.

#### Check 3: Contact exclusions prevent adjacent self-collision

Load a cable with `count="6 1 1"`. Verify `model.contact_excludes.len() == 4`
(N-2 pairs for N-1 bodies, i.e. 5-1=4). Step for 2 seconds. Verify that no
contact is detected between adjacent cable bodies (contact pairs should not
include any adjacent cable body pair).

#### Check 4: Cable hangs below anchor

Load a cable with `initial="none"` (pinned at top), `count="11 1 1"`, offset
at z=1. Step for 5 seconds with damping. Verify every cable body's z-position
is below the anchor z-position (1.0). The cable must hang downward under
gravity.

#### Check 5: Cable end converges with more segments

Create three cables with count=6, 11, 21 (same total length, same offset,
same damping). Step each for 10 seconds. Record tip position (last body xpos).
Verify `|tip_11 - tip_21| < |tip_6 - tip_21|` — more segments = closer to
the converged solution.

#### Check 6: Cable preserves total length

Load a cable with `count="11 1 1"`, `size="1.0"`. Step for 5 seconds. Sum
the Euclidean distances between consecutive cable body positions. Verify the
total is within 5% of the original size (1.0). Rigid-body chains preserve
segment lengths exactly (they're rigid links), so this should hold tightly.

#### Check 7: Cosine curve produces smooth initial shape

Load a cable with `curve="s c 0"` (sin in x, cos in y, zero in z) and
`count="21 1 1"`, `size="1 0.3 2"`. Before stepping (t=0), read body
positions. Verify:
- y-positions follow a cosine envelope (not all zero)
- The curve is smooth: no consecutive-segment angle > 30 degrees

This exercises the `CompositeShape::Cos` and `CompositeShape::Sin` vertex
generation paths.

#### Check 8: Multiple cables don't interfere

Load two cables with different prefixes ("A" and "B"), different offsets.
Step for 3 seconds. Verify cable A's body positions are independent of cable
B's — specifically, compare cable A's final state when loaded alone vs. when
loaded with cable B. Positions should match within floating-point tolerance
(1e-10), confirming no cross-contamination.

---

## Naming and Packaging

| Directory | Package name | Type |
|-----------|-------------|------|
| `composites/hanging-cable/` | `example-composite-hanging-cable` | visual |
| `composites/cable-catenary/` | `example-composite-cable-catenary` | visual |
| `composites/cable-loaded/` | `example-composite-cable-loaded` | visual |
| `composites/stress-test/` | `example-composite-stress-test` | headless |

All four packages must be registered in the workspace `Cargo.toml` members
list.

---

## Implementation Order

1. **stress-test** — headless, no rendering dependencies, validates engine
   behavior. If any check fails, we discover engine gaps before investing in
   visual polish.
2. **hanging-cable** — simplest visual, one concept (pinned chain + gravity).
3. **cable-catenary** — both-ends-fixed via equality connect constraint.
4. **cable-loaded** — builds on cable-catenary, adds midpoint force.

---

## Resolved Design Decisions

1. **cable-catenary / cable-loaded anchor1:** The `connect` constraint's
   `anchor1` must be in the last cable body's local frame. The S_last site
   is at `[edge_length, 0, 0]` where edge_length = size / (count-1). For a
   16-vertex cable with `size="1.0"`, anchor1 = `[0.0667, 0, 0]`. Verified:
   composite expansion runs before equality processing in the builder
   pipeline, so body names like `AB_last` are available for constraint
   references.

2. **cable-loaded midpoint force:** Use `xfrc_applied` directly on the
   midpoint body. `data.xfrc_applied[body_id]` is a `SpatialVector` with
   layout `[torque_x, torque_y, torque_z, force_x, force_y, force_z]`.
   Downward force: `data.xfrc_applied[mid_body][5] = -F`. Persists across
   steps (clear manually if needed). No extra MJCF elements required.
   Verified: dedicated integration tests exist in
   `sim/L0/tests/integration/xfrc_applied.rs`.

3. **Stress check 3 contact inspection:** Use
   `data.contacts.iter().take(data.ncon)` to iterate active contacts.
   Each contact has `.bodies(&model) -> (usize, usize)` to get body IDs.
   Verify excluded pairs don't appear via `model.contact_excludes` HashSet
   (canonical key: `(min(b1,b2), max(b1,b2))`). Verified: API exists and
   is used in existing tests.
