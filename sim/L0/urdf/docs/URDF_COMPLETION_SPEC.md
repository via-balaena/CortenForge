# sim-urdf v1.0 Completion Spec

**Status:** Draft — awaiting review
**Date:** 2026-03-30
**Goal:** Close all gaps between what sim-urdf parses and what it converts

## Current State

sim-urdf parses URDF XML and converts it to MJCF XML, which is then compiled
to a `sim_core::Model` via `sim_mjcf::load_model()`. 32/32 tests pass.

The crate parses several features that it silently drops during conversion.
The engine already supports all of these features — they just need to be wired
through the converter.

## Fixes

### Fix 1: Joint friction → `frictionloss`

**Gap:** `UrdfJointDynamics.friction` is parsed but never written to MJCF.
MuJoCo's equivalent is the `frictionloss` joint attribute.

**Change:** In `converter.rs::convert_joint()`, add:

```rust
if dynamics.friction > 0.0 {
    write!(attrs, r#" frictionloss="{}""#, dynamics.friction).ok();
}
```

**Test:** URDF with `<dynamics friction="0.5"/>` → MJCF contains
`frictionloss="0.5"`. Stepping the model produces velocity-independent
resistive torque.

**Risk:** None. Additive change.

---

### Fix 2: Mesh file support

**Gap:** `UrdfGeometry::Mesh` is parsed but silently dropped in
`convert_geom()` (DT-177). sim-mjcf already handles `<asset><mesh>` elements
with full collision support (convex hull via Quickhull, BVH for triangle mesh).

**Change:**

1. Add an `assets` buffer to `Converter` that collects `<mesh>` declarations.
2. In `convert_geom()`, when encountering `UrdfGeometry::Mesh`:
   - Generate a unique mesh asset name (e.g., `"{link_name}_{geom_name}"` or
     `"{link_name}_mesh_{index}"`).
   - Append `<mesh name="..." file="..." scale="..."/>` to the assets buffer.
   - Emit `<geom type="mesh" mesh="{name}" .../>` instead of returning early.
3. In `convert()`, write `<asset>...</asset>` block before `<worldbody>` if
   any mesh assets were collected.
4. Handle URDF `scale` attribute → MJCF `scale` attribute (URDF is xyz,
   MJCF is xyz — direct passthrough).

**Path resolution:** URDF uses `package://` URIs and relative paths. The
converter should:
- Strip `package://` prefix (sim-mjcf's `strippath="true"` handles the rest)
- Pass the filename through as-is — actual file resolution is sim-mjcf's
  responsibility via `meshdir`/`assetdir` compiler settings
- Document that callers must set up asset directories appropriately

**MJCF output example:**
```xml
<asset>
    <mesh name="link1_visual" file="meshes/link1.stl" scale="0.001 0.001 0.001"/>
</asset>
<worldbody>
    <body name="link1">
        <geom type="mesh" mesh="link1_visual"/>
    </body>
</worldbody>
```

**Test:**
- URDF with `<mesh filename="box.stl"/>` → MJCF contains `<mesh>` asset and
  `type="mesh"` geom.
- Round-trip: load_urdf_model with a URDF referencing a real STL file (embed a
  tiny cube STL in the test) → model has mesh geom with correct collision.

**Risk:** Medium. File path resolution across platforms needs care. The
`package://` stripping is a heuristic — ROS package resolution is out of scope.

---

### Fix 3: Mimic joints → equality constraints

**Gap:** URDF `<mimic>` elements are not parsed at all. sim-core already has
`EqualityType::Joint` with polynomial coupling, and sim-mjcf parses
`<equality><joint>` elements with `polycoef`.

URDF mimic semantics: `position = multiplier * leader_position + offset`
MuJoCo equivalent: `<joint joint1="follower" joint2="leader" polycoef="offset multiplier 0 0 0"/>`

**Changes:**

1. **Parser** (`parser.rs`): Parse `<mimic>` child element inside `<joint>`:
   ```xml
   <mimic joint="leader_joint" multiplier="2.0" offset="0.1"/>
   ```
   Add to `UrdfJoint`:
   ```rust
   pub mimic: Option<UrdfMimic>,
   ```
   New type:
   ```rust
   pub struct UrdfMimic {
       pub joint: String,      // leader joint name
       pub multiplier: f64,    // default 1.0
       pub offset: f64,        // default 0.0
   }
   ```

2. **Converter** (`converter.rs`): Collect mimic constraints during tree walk.
   After `</worldbody>`, emit:
   ```xml
   <equality>
       <joint joint1="follower" joint2="leader" polycoef="offset multiplier 0 0 0"/>
   </equality>
   ```

3. **Types** (`types.rs`): Add `UrdfMimic` struct.

**Test:**
- URDF with mimic joint → MJCF contains `<equality><joint>` with correct
  polycoef.
- Two hinges with mimic (ratio=2): driving joint1 to 0.5 rad causes joint2
  to reach 1.0 rad (within solver tolerance).

**Risk:** Low. Clean mapping to existing engine feature.

---

### Fix 4: Planar joints → 2 slides + 1 hinge

**Gap:** URDF `planar` joints are approximated as a single hinge. A planar
joint allows translation in 2 axes of the plane + rotation about the plane
normal. This should be 2 slide joints + 1 hinge joint (3 DOF).

**Change:** In `convert_joint()`, when `joint_type == Planar`:

1. Determine the plane from the joint axis (axis = plane normal).
2. Compute two orthogonal in-plane axes (e.g., if normal = Z, use X and Y).
3. Emit three joints:
   ```xml
   <joint name="{name}_slide_x" type="slide" axis="{in_plane_x}"/>
   <joint name="{name}_slide_y" type="slide" axis="{in_plane_y}"/>
   <joint name="{name}_hinge" type="hinge" axis="{normal}"/>
   ```

**Test:** URDF with planar joint → model has 3 DOF (nv increases by 3).
Body can translate in-plane and rotate about the normal.

**Risk:** Low. Changes naming convention (single joint → 3 joints). Any
downstream code referencing planar joint by name would need to adapt, but
since planar joints are extremely rare in practice and the current
approximation is wrong, this is the right fix.

---

### Fix 5: Effort and velocity limits

**Gap:** `UrdfJointLimit.effort` and `UrdfJointLimit.velocity` are parsed but
not mapped to MJCF.

**Analysis:**
- **Effort** → MuJoCo doesn't have a joint-level force limit. Effort limits
  are enforced on actuators (`forcerange` or `ctrlrange`). Since URDF joints
  don't carry actuators (actuators are a separate concern), and the converter
  doesn't auto-generate actuators, there's no clean place to map this.
- **Velocity** → MuJoCo doesn't have a joint-level velocity limit. Some
  converters approximate this with high damping, but that changes dynamics.

**Decision:** Document as intentional limitation. These are actuator-level
concerns that belong in the user's MJCF augmentation (actuators, sensors, etc.
can be added to the generated MJCF before compilation).

**Change:** Add a doc comment to `UrdfJointLimit` explaining the limitation.
No converter changes.

---

## Documented Limitations (no changes needed)

These are architectural boundaries, not bugs:

| Limitation | Reason |
|------------|--------|
| No kinematic loops | MuJoCo only supports tree structures |
| No Gazebo extensions | Out of scope — Gazebo-specific |
| No `<safety_controller>` | No MuJoCo equivalent |
| No `<calibration>` | No MuJoCo equivalent |
| No `<transmission>` (ROS) | ROS-specific, not part of URDF spec |
| No material rendering | Layer 0 crate — no visual pipeline |

---

## Implementation Order

1. **Fix 1** — friction (5 min, one line)
2. **Fix 3** — mimic joints (1 hour — parser + converter + types + tests)
3. **Fix 4** — planar joints (30 min — converter change + tests)
4. **Fix 2** — mesh files (2-3 hours — converter + path handling + tests)
5. **Fix 5** — documentation only

Fixes 1, 3, 4 are low-risk and self-contained. Fix 2 has the most surface
area (path resolution, asset block generation) but the engine already does
the heavy lifting.

---

## Examples (after fixes)

One concept per example, each isolating a single URDF feature:

```
fundamentals/
  sim-cpu/
    urdf-loading/
      revolute/         # Revolute joint: URDF loads, pendulum swings, matches MJCF
      prismatic/        # Prismatic joint: slider oscillates, matches MJCF
      continuous/       # Continuous (unlimited revolute): free spinning wheel
      fixed/            # Fixed joint: fused bodies, verify body count
      mimic/            # Mimic joint: leader-follower with 2:1 ratio
      geometry/         # All 3 primitives (box, sphere, cylinder) in one arm
      inertia/          # Diagonal vs full inertia: precession difference
      damping-friction/ # Joint damping + frictionloss: decay rate comparison
      error-handling/   # Invalid URDF: clear error messages, no panics
      stress-test/      # Headless: all checks in one pass
```

### Example Descriptions

#### 1. `revolute/` — Revolute Joint Pendulum
Single hinge pendulum defined in URDF. Swings under gravity. Prints joint
angle and energy. Verifies period matches analytical `T = 2pi*sqrt(L/g)`.
**Concept:** Basic URDF → Model pipeline with the most common joint type.

#### 2. `prismatic/` — Prismatic Joint Slider
Mass on a horizontal spring (stiffness via joint stiffness). Oscillates.
Verifies period matches `T = 2pi*sqrt(m/k)`.
**Concept:** Slide joint conversion, axis mapping.

#### 3. `continuous/` — Continuous Joint Wheel
Spinning wheel with no limits. Apply constant torque via ctrl, verify
angular acceleration `alpha = tau/I`.
**Concept:** Unlimited revolute (no `limited` attribute in MJCF).

#### 4. `fixed/` — Fixed Joint Fusion
Three links: base → fixed → child → revolute → grandchild. Verify that
`fusestatic="true"` merges the fixed-joint link into its parent. Model
has fewer bodies than URDF links.
**Concept:** Fixed joints disappear in MJCF, body tree is compacted.

#### 5. `mimic/` — Mimic Joint Coupling
Two revolute joints. Joint2 mimics joint1 with multiplier=2, offset=0.1.
Drive joint1 sinusoidally, verify joint2 tracks at 2x + 0.1 within solver
tolerance.
**Concept:** `<mimic>` → `<equality><joint>` conversion.

#### 6. `geometry/` — Primitive Geometry Shapes
One arm with box (upper arm), cylinder (forearm), sphere (hand). Verify
all three appear in model with correct sizes (box half-extents, cylinder
half-length).
**Concept:** Geometry size conversion between URDF and MJCF conventions.

#### 7. `inertia/` — Inertia Tensor Handling
Two free-floating bodies: one with diagonal inertia (uniform sphere), one
with off-diagonal inertia (asymmetric object). Release with angular
velocity. Diagonal body has symmetric precession; off-diagonal body
precesses asymmetrically.
**Concept:** `diaginertia` vs `fullinertia` conversion paths.

#### 8. `damping-friction/` — Joint Dynamics
Three identical pendulums with different dynamics:
(A) damping=0 friction=0 (no loss),
(B) damping=0.5 friction=0 (velocity-dependent),
(C) damping=0 friction=0.5 (velocity-independent frictionloss).
Compare energy decay curves.
**Concept:** Both dynamics parameters propagate correctly to MJCF.

#### 9. `error-handling/` — Error Messages
Headless. Feeds several invalid URDFs and verifies each produces the
expected error variant (not a panic):
- Missing `<robot>` → MissingElement
- Unknown joint type → UnknownJointType
- Undefined link reference → UndefinedLink
- Kinematic loop → KinematicLoop (or NoRootLink)
- Duplicate link names → DuplicateName
**Concept:** The crate fails gracefully with actionable messages.

#### 10. `stress-test/` — Headless Validation
Runs all checks in one headless binary (~20-25 checks):
- Structural equivalence (URDF vs hand-written MJCF)
- Dynamic equivalence (qpos/qvel match after 1s)
- Each joint type loads correctly
- Geometry sizes correct
- Inertia propagates
- Limits and damping propagate
- Mimic constraint tracks
- Friction loss active
- Error variants correct
**Concept:** Regression gate for the full URDF pipeline.
