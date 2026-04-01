# Mocap Bodies Examples Spec

**Status:** Draft
**Parent:** `examples/COVERAGE_SPEC.md` Track 1B, subdirectory 3
**Date:** 2026-03-31

## Overview

Mocap (motion capture) bodies are world-attached kinematic bodies whose pose
is set directly via `data.mocap_pos` and `data.mocap_quat` rather than
computed by the integrator. They are the mechanism for user input, teleop,
animation playback, and VR controllers.

Key properties:
- Must be direct children of worldbody (no parent body other than world)
- Cannot have joints (pose is set externally, not computed)
- Participate in collision (contacts affect dynamic bodies, not mocap)
- Not affected by gravity, contact forces, or constraint forces
- Keyframe reset restores `mocap_pos` and `mocap_quat`

## Implementation Order

Build the stress test first — it validates all engine-level invariants
headlessly. Only after the stress test passes do we build visual examples.
Each visual example demonstrates exactly one concept to keep debugging simple.

```
1. stress-test     (headless, 12 checks — engine validation)
2. drag-target     (visual — soft weld tracking, translational mocap_pos)
3. push-object     (visual — mocap contact interaction, translational sweep)
4. spin-fling      (visual — orientation-driven mocap_quat, rotational contact)
```

## Examples

### 1. `stress-test` — Headless Validation (12 checks)

Pure sim-core + sim-mjcf, no Bevy. Validates all mocap invariants that the
visual examples rely on.

**MJCF models (inline constants):**

- **Model A** — Single mocap body + dynamic ball, gravity, contact-capable.
  Used by checks 1–6.
- **Model B** — Two independent mocap bodies + dynamic body. Used by check 7.
- **Model C** — Mocap body with keyframe storing mpos/mquat. Used by check 8.
- **Model D** — Mocap body + free-floating box connected by soft weld.
  Used by checks 9–10.
- **Model E** — Mocap body with zero-mass geom. Used by check 11.
- **Model F** — Mocap body with a child body (hinge joint on child).
  Used by check 12.

**Checks:**

| # | Name | Model | What it verifies |
|---|------|-------|-----------------|
| 1 | Position tracks mocap_pos | A | After setting `mocap_pos` and calling `forward()`, `xpos[mocap_body]` matches exactly (epsilon 1e-12). |
| 2 | Quaternion tracks mocap_quat | A | After setting `mocap_quat` to a non-identity rotation and calling `forward()`, `xquat[mocap_body]` matches exactly. |
| 3 | World-child invariant | A | `body_parent[mocap_body] == 0` (always parented to world). |
| 4 | Contact generation | A | Move mocap wall close to dynamic ball, step — `ncon > 0` (contacts generated). |
| 5 | Immune to gravity | A | Step 1000 times under gravity — mocap body `xpos` unchanged (epsilon 1e-12). |
| 6 | Immune to contact forces | A | After contact with dynamic ball, mocap body `xpos` unchanged (epsilon 1e-12). |
| 7 | Multiple mocap independent | B | Move only mocap A — verify mocap B unchanged, mocap A at new position. |
| 8 | Keyframe restores mocap state | C | Set mocap to arbitrary pose, reset to keyframe — verify mocap_pos and mocap_quat match keyframe mpos/mquat. |
| 9 | Weld-to-mocap: force generated | D | Displace mocap from follower, call `forward()` — `qfrc_constraint` has nonzero entries for follower DOFs. |
| 10 | Weld-to-mocap: follower tracks | D | Displace mocap, step 3000 — follower moves toward mocap (separation < 50% of initial displacement). |
| 11 | Zero-mass FK override | E | Mocap body with `mass="0"` geom. Set mocap_pos, forward() — xpos matches exactly. (cinert is pure multiplication, no division-by-mass risk.) |
| 12 | Child body follows mocap parent | F | Move mocap parent, forward() — child body xpos has shifted by the same amount as parent (within joint offset). |

**Pattern:** Same as `free-joint/stress-test` — `check()` helper, per-check
functions returning `(passed, total)`, main tallies and exits nonzero on
failure.

**Dependencies:** `sim-core`, `sim-mjcf`, `nalgebra`.

---

### 2. `drag-target` — Soft Weld Tracking (Visual)

**One concept:** Mocap body drives a dynamic body through a compliant weld
constraint.

A translucent green mocap sphere moves on a sinusoidal path (scripted in
the physics step). A solid box is connected to the mocap body via a weld
constraint with compliance (`solref="0.02 1.0"`). The box follows the target
with visible spring-like lag.

**MJCF model:**
- Mocap body: small sphere, semi-transparent green
- Dynamic body: box with free joint, solid color
- Ground plane for visual reference
- Weld constraint: `body1="follower" body2="target" solref="0.02 1.0"`
- Zero gravity (focus on weld tracking, not falling)

**Validation checks (inline, 2 checks):**
1. Mocap body position matches scripted path exactly each frame
2. Follower body stays within 0.3 m of target (weld keeps it close)

**What the user sees:** Green ghost sphere gliding on a sine wave; a box
chasing it with smooth, springy lag.

---

### 3. `push-object` — Mocap Contact Interaction (Visual)

**One concept:** Mocap body geometry generates one-way contacts with dynamic
bodies.

A mocap paddle (capsule) sweeps linearly across the scene. A dynamic ball
sits on a ground plane in the paddle's path. When the paddle reaches the
ball, contact forces push the ball; the paddle is unaffected.

**MJCF model:**
- Mocap body: horizontal capsule (the paddle), red
- Dynamic body: sphere with free joint, blue
- Ground plane (for the ball to rest on)
- Standard gravity

**Validation checks (inline, 2 checks):**
1. Ball has moved in the sweep direction after paddle passes
2. Mocap paddle position matches scripted path exactly (unaffected by contact)

**What the user sees:** Red paddle slides through the scene and flicks a
blue ball across the ground.

---

### 4. `spin-fling` — Orientation-Driven Mocap Interaction (Visual)

**One concept:** Mocap body orientation (`mocap_quat`) drives rotational
contact interaction with dynamic bodies.

A mocap turntable (flat cylinder) spins via scripted `mocap_quat` updates.
A small ball sits on top of the platform. Friction transmits the rotation
to the ball, which eventually slides off the edge and flies away.

**MJCF model:**
- Mocap body: flat cylinder (turntable), pos at moderate height, orange
- Dynamic body: small sphere with free joint resting on the platform, blue
- Ground plane below (ball lands on it after being flung)
- Standard gravity (ball needs to rest on platform, then fall after flung)
- Moderate friction between platform and ball (enough to grip, not infinite)

**Validation checks (inline, 2 checks):**
1. Mocap turntable orientation matches scripted rotation each frame
2. Ball has nonzero horizontal velocity after a few seconds (friction drove it)

**What the user sees:** An orange disc spinning up; a blue ball gripping
the surface, sliding outward, then flying off the edge.

## Directory Layout

```
mocap-bodies/
  MOCAP_EXAMPLES_SPEC.md
  stress-test/
    Cargo.toml
    src/main.rs
  drag-target/
    Cargo.toml
    src/main.rs
  push-object/
    Cargo.toml
    src/main.rs
  spin-fling/
    Cargo.toml
    src/main.rs
```
