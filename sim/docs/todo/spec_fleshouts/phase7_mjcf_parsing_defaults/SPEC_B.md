# Spec B — Joint Physics: Spec

**Status:** Draft
**Phase:** Roadmap Phase 7 — MJCF Parsing & Defaults Gaps
**Effort:** M
**MuJoCo ref:** `mj_passive()` in `engine_passive.c`, `mj_energyPos()` in
`engine_core_smooth.c`, `mj_instantiateLimit()` in `engine_core_constraint.c`
**MuJoCo version:** 3.x (C source on GitHub, `google-deepmind/mujoco` main branch)
**Test baseline:** 1,900+ sim domain tests (post-Phase 6)
**Prerequisites:**
- §64 (spring force + energy): Spec A `qpos_spring` array (landed in
  `01ae59f` — `qpos_spring` already exists on `Model` at `model.rs:750` and
  is populated by `builder/joint.rs:164-196`)
- §64a (jnt_margin): No external prerequisites — adds a new field from scratch

**Independence:** This spec is independent of Spec A and Spec C per the
umbrella dependency graph. Shared files: `parser.rs` (Spec A touches defaults
section ~line 600; Spec B touches joint attrs section ~line 1662 — different
sections, no conflict), `types.rs` (Spec A adds `MjcfEqualityDefaults`; Spec B
adds `margin` to `MjcfJoint` and `MjcfJointDefaults` — different structs),
`model.rs` (Spec A may add fields; Spec B adds `jnt_margin` — no overlap).

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the features described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

The rubric (SPEC_B_RUBRIC.md) performed empirical verification against MuJoCo
C source and discovered two scope changes from the umbrella:

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| §60: `springinertia` — parse, store, add to CRBA diagonal | **Does not exist in MuJoCo.** No `springinertia` field in `mjmodel.h`, no such attribute in `<joint>` XML reference, zero GitHub search results across `google-deepmind/mujoco`. The umbrella conflated `dof_armature` with a nonexistent spring-inertia coupling. | **DROP** — cannot implement conformance for nonexistent behavior |
| §64: Ball/free spring energy only (energy.rs stub) | `future_work_15.md` §64 covers both spring **force** and spring **energy**. Ball/free spring force is also missing (`visit_multi_dof_joint` in `passive.rs:818-835` does damping only, no spring). Force and energy use identical quaternion math (`subquat`). Energy without force is physically inconsistent. | **Expand** to include both spring force (`passive.rs`) and energy (`energy.rs`) |

**Final scope:**

1. **§64:** Ball/free joint spring force (`passive.rs`) + spring energy
   (`energy.rs`) via quaternion geodesic distance using `subquat()` equivalent
   of MuJoCo's `mju_subQuat`. Includes fixing 3 pre-existing conformance gaps
   in `energy.rs` (missing `DISABLE_SPRING` gate, missing sleep filter on
   spring section, wrong stiffness guard).
2. **§64a:** `jnt_margin` — parse `margin` from `<joint>` and
   `<default><joint>`, store as `jnt_margin`, replace 6 activation checks +
   3 `finalize_row!` margin arguments in `assembly.rs`.

---

## Problem Statement

**Conformance gap — ball/free spring force and energy.** MuJoCo computes
spring forces for ball and free joints in `mj_passive()` (`engine_passive.c`)
using quaternion geodesic distance via `mju_subQuat()`, writing to
`d->qfrc_spring`. It computes spring potential energy for the same joints in
`mj_energyPos()` (`engine_core_smooth.c`) using the same quaternion math.
CortenForge has an explicit stub in `energy.rs:50-53` ("Ball/Free joint springs
would use quaternion distance — Not commonly used, skip for now") and
`visit_multi_dof_joint()` in `passive.rs:818-835` only computes damping, not
spring force. Any model with ball or free joint stiffness will produce wrong
`qfrc_spring` (zero instead of `-k * subquat(q, q_spring)`) and wrong
`energy_potential` (missing spring contribution).

Additionally, `energy.rs` has 3 pre-existing conformance gaps affecting ALL
joint types: (1) no `DISABLE_SPRING` gate on the spring section, (2) no sleep
filter on the spring section, (3) stiffness guard uses `> 0.0` instead of
MuJoCo's `== 0`.

**Conformance gap — joint limit margin.** MuJoCo activates joint limit
constraints when `dist < m->jnt_margin[i]` in `mj_instantiateLimit()`
(`engine_core_constraint.c`), allowing soft pre-activation when `margin > 0`.
The margin value is also stored on the constraint row (`efc_margin`) for use
by the impedance and reference acceleration computations. CortenForge hardcodes
`< 0.0` in 6 locations in `assembly.rs` (3 counting + 3 assembly) and passes
`0.0` as the margin to `finalize_row!` in 3 locations. This prevents joint
limits from pre-activating and produces wrong impedance values when margin
should be nonzero.

---

## MuJoCo Reference

### Ball/free spring force — `mj_passive()` / `mj_springdamper()`

**Source:** `engine_passive.c`, function `mj_passive()` or `mj_springdamper()`
(exact function name varies by MuJoCo version; the logic is the same).

**Ball joint spring force:**

```c
mji_copy4(quat, d->qpos+padr);
mji_normalize4(quat);
mji_subQuat(dif, quat, m->qpos_spring + padr);
d->qfrc_spring[dadr+0] = -stiffness * dif[0];
d->qfrc_spring[dadr+1] = -stiffness * dif[1];
d->qfrc_spring[dadr+2] = -stiffness * dif[2];
```

**Free joint spring force (translational, then falls through to ball):**

```c
d->qfrc_spring[dadr+0] = -stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
d->qfrc_spring[dadr+1] = -stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
d->qfrc_spring[dadr+2] = -stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);
padr += 3;
dadr += 3;
// falls through to ball case for rotational DOFs
```

**Algorithm:**
1. Gate on `DISABLE_SPRING` — skip if set
2. Gate on sleep — skip if body is sleeping
3. For each joint with stiffness != 0:
   - **Free joint:** 3 translational DOFs use Euclidean difference
     (`-k * (pos - pos_spring)`), then `padr += 3; dadr += 3;` to advance
     past position to quaternion, then fall through to ball case
   - **Ball joint:** normalize `qpos` quaternion, compute `subQuat(qpos,
     qpos_spring)` to get 3D axis-angle vector, force = `-k * dif[0..3]`
4. Single stiffness value used for both translational and rotational DOFs
   of a free joint

**Addressing:** Force written to `dadr` (DOF-indexed, 6 for free, 3 for ball).
Position read from `padr` (qpos-indexed, 7 for free, 4 for ball).

### Ball/free spring energy — `mj_energyPos()`

**Source:** `engine_core_smooth.c` (or `engine_sensor.c` depending on version),
function `mj_energyPos()`.

```c
case mjJNT_FREE:
  // Translational: E = 0.5 * k * ||pos - pos_spring||²
  mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
  d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);
  padr += 3;
  mjFALLTHROUGH;

case mjJNT_BALL:
  // Rotational: E = 0.5 * k * ||subQuat(q, q_spring)||²
  mju_copy4(quat, d->qpos+padr);
  mju_normalize4(quat);
  mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
  d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);
  break;
```

**Algorithm:**
1. Gate on `!mjDISABLED(mjDSBL_SPRING)` — skip entire spring section if set
2. MuJoCo's `mj_energyPos()` has a two-phase structure:
   (a) gravity loop — runs unconditionally over ALL bodies (no sleep check)
   (b) spring/tendon loop — applies sleep filter (skips sleeping bodies)
3. Guard: `if (stiffness == 0) continue;` — exact zero check. MuJoCo
   processes negative stiffness (though physically meaningless).
4. Free joint: translational energy `0.5 * k * ||pos - pos_spring||²`, then
   `padr += 3` and fall through to ball case for rotational energy
5. Ball joint: `0.5 * k * ||subQuat(q, q_spring)||²` = `0.5 * k * θ²`

**Dead normalization in energy path:** MuJoCo calls `mju_normalize4(quat)` but
then passes `d->qpos+padr` (not `quat`) to `mju_subQuat`. This is
conformance-neutral because `mji_quat2Vel`'s `atan2(sin_half, w)` is
scale-invariant — both paths produce identical axis-angle results.

### `mju_subQuat()` and `mji_quat2Vel()` chain

**Source:** `engine_util_spatial.c`.

```c
void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]) {
  mjtNum qneg[4], qdif[4];
  mji_negQuat(qneg, qb);       // conjugate: [w, -x, -y, -z]
  mji_mulQuat(qdif, qneg, qa); // relative rotation: conj(qb) * qa
  mji_quat2Vel(res, qdif, 1);  // axis-angle vector with dt=1
}
```

`mji_quat2Vel(res, qdif, dt=1)`:
```c
axis = normalize(xyz_components);  // sin_half = ||xyz||
angle = 2 * atan2(sin_half, w);
if (angle > PI) angle -= 2*PI;    // shortest-path wrapping
result = axis * angle;             // 3D axis-angle vector
```

**Mathematical derivation:**
- `subQuat(qa, qb)` = `quat2Vel(conj(qb) * qa, dt=1)` = axis-angle vector
  where `||vec|| = θ` (rotation angle in radians)
- Energy: `0.5 * k * θ²` = `0.5 * k * dot(dif, dif)` because `||dif|| = θ`
- Force: `-k * dif` (3D vector, DOF-indexed)
- Wrapping at `θ > π`: `angle -= 2π` ensures `θ ∈ (-π, π]` (shortest path,
  handles quaternion double-cover of SO(3))
- Edge cases: identity → zero vector; near-π → wrapping gives correct sign;
  exact-π → axis ambiguous but `||dif|| = π`

**CortenForge equivalent:** `subquat()` in `tendon/spatial.rs:441` implements
identical logic: `dq = qb.conjugate() * qa` → extract `(i, j, k)` → normalize
→ `angle = 2 * atan2(sin_half, w)` → shortest-path wrap → `axis * angle`.

**Argument order verification:** `subquat(qa, qb)` in CortenForge computes
`conj(qb) * qa` — the rotation FROM `qb` TO `qa`. MuJoCo's `mju_subQuat(res,
qa, qb)` computes the same: `conj(qb) * qa`. So `subquat(current, reference)`
matches `mju_subQuat(res, current, reference)`. An argument swap would produce
wrong-sign forces.

**Quaternion normalization strategy:** CortenForge's `subquat()` requires
`UnitQuaternion` arguments (type-system enforced), so BOTH the force path
(S1, `passive.rs`) and the energy path (S2, `energy.rs`) must normalize when
constructing from raw `data.qpos` f64 values — use
`UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))`. MuJoCo's force
path normalizes before `mji_subQuat` (`mji_normalize4(quat)`). MuJoCo's
energy path has dead normalization code — `mju_normalize4(quat)` is called
but the result is unused; `d->qpos+padr` is passed to `mju_subQuat` instead.
This discrepancy is **conformance-neutral** because `mji_quat2Vel`'s
`atan2(sin_half, w)` is scale-invariant — both normalized and unnormalized
quaternions produce identical axis-angle results. CortenForge normalizes in
both paths (via `new_normalize`), which is technically more defensive than
MuJoCo's energy path but produces identical numerical output.

### `mj_instantiateLimit()` — margin handling

**Source:** `engine_core_constraint.c`, function `mj_instantiateLimit()`.

```c
margin = m->jnt_margin[i];

// Hinge/Slide: bilateral (both limits checked)
for (int side=-1; side <= 1; side+=2) {
  dist = side * (m->jnt_range[2*i+(side+1)/2] - value);
  if (dist < margin) {
    mj_addConstraint(m, d, jac, &dist, &margin, 0,
                     1, mjCNSTR_LIMIT_JOINT, i, ...);
  }
}

// Ball: unilateral
dist = mju_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;
if (dist < margin) {
  mj_addConstraint(m, d, jac, &dist, &margin, 0,
                   1, mjCNSTR_LIMIT_JOINT, i, ...);
}
```

**Margin data flow:**
```
m->jnt_margin[i]  →  margin (local variable)
                          |
                          +→ activation test: dist < margin
                          |
                          +→ mj_addConstraint(..., &margin, ...)
                                  |
                                  v
                              efc_margin[nefc] = margin
                                  |
                                  +→ compute_impedance(solimp, |pos - margin|)
                                  +→ compute_aref(k, b, imp, pos, margin, vel)
```

**Dual purpose:** (1) activation threshold — constraint created when `dist <
margin`; (2) stored on constraint row for impedance/aref computation.

**Default:** `margin = 0.0` (MuJoCo MJCF default). When margin=0, the
condition `dist < 0` matches CortenForge's current hardcoded behavior — this
is the backward-compatibility guarantee.

### Pre-existing conformance gaps in `energy.rs`

Three gaps affecting ALL joint types (hinge, slide, ball, free):

1. **Missing `DISABLE_SPRING` gate.** MuJoCo gates the entire spring energy
   section on `!mjDISABLED(mjDSBL_SPRING)`. CortenForge enters the spring
   loop unconditionally (`energy.rs:38`).

2. **Missing sleep filter on spring section.** MuJoCo's `mj_energyPos()` has
   a two-phase structure: (a) gravity loop runs unconditionally over ALL bodies
   (no sleep check), then (b) a sleep filter variable is computed and applied
   to the spring/tendon loops only. CortenForge's `energy.rs:38` iterates
   joints with no sleep check. **Important:** the sleep filter must NOT be
   added to the gravity section — MuJoCo computes gravitational PE for all
   bodies regardless of sleep state.

3. **`stiffness > 0.0` vs `stiffness == 0`.** CortenForge's guard
   (`energy.rs:40`) skips joints with zero OR negative stiffness. MuJoCo only
   skips exact zero (`if (stiffness == 0) continue;`). MuJoCo would process
   negative stiffness.

### `implicit_mode` gate

The `PassiveForceVisitor` has `implicit_mode: bool` (`passive.rs:773`) set to
`true` when `model.integrator == Integrator::ImplicitSpringDamper`.

The hinge/slide spring force is gated on `if !self.implicit_mode`
(`passive.rs:800`). The implicit integrator handles spring forces internally
via `implicit_stiffness` and `implicit_springref`, so explicit spring forces
are suppressed.

Ball/free spring force must be gated identically. However,
`compute_implicit_params()` sets `implicit_stiffness = 0.0` for ball/free
joints (quaternion springs are too nonlinear for the diagonal implicit
approximation). Combined with the `!implicit_mode` gate, ball/free spring
forces are fully suppressed in `ImplicitSpringDamper` mode — neither explicit
(gated out) nor implicit (stiffness=0). This matches MuJoCo behavior and is
documented as expected, not a gap.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Ball spring force | `-k * subQuat(q, q_spring)` in `mj_passive()` | **Missing** — `visit_multi_dof_joint` does damping only (`passive.rs:818-835`) |
| Free spring force (trans) | `-k * (pos - pos_spring)` for 3 translational DOFs | **Missing** — same function |
| Free spring force (rot) | Falls through to ball case after `padr += 3; dadr += 3` | **Missing** — same function |
| Ball spring energy | `0.5 * k * ||subQuat(q, q_spring)||²` in `mj_energyPos()` | **Stub** — returns 0 (`energy.rs:50-53`) |
| Free spring energy (trans) | `0.5 * k * ||pos - pos_spring||²` | **Stub** — returns 0 |
| Free spring energy (rot) | Falls through to ball, same stiffness | **Stub** — returns 0 |
| `DISABLE_SPRING` gate on energy | Gates entire spring section | **Missing** — spring section always runs (`energy.rs:38`) |
| Sleep filter on spring energy | Applied to spring/tendon section only (gravity unconditional) | **Missing** — no sleep check in energy (`energy.rs:38`) |
| Stiffness guard (energy) | `== 0` (exact zero; negative stiffness processed) | **Wrong** — `> 0.0` skips negative stiffness (`energy.rs:40`) |
| Joint limit activation | `dist < m->jnt_margin[i]` | **Hardcoded** — `dist < 0.0` at 6 sites (`assembly.rs:109,113,128,439,460,492`) |
| Margin on constraint row | `efc_margin = margin` via `mj_addConstraint` | **Hardcoded** — `0.0` at 3 `finalize_row!` calls (`assembly.rs:448,469,508`) |
| `implicit_mode` gate (spring) | Spring force suppressed in implicit mode | **Correct for hinge/slide** — needs matching for ball/free |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Quaternion type | `mjtNum[4]` raw array | `UnitQuaternion<f64>` (nalgebra) | Use `UnitQuaternion::new_normalize()` when constructing from raw `qpos` values; `subquat()` in `tendon/spatial.rs:441` bridges the gap |
| `subquat` args | `mju_subQuat(res, current, reference)` = `conj(ref) * cur` | `subquat(current, reference)` = `conj(ref) * cur` | Direct port — argument semantics match. `subquat(&qa, &qb)` ↔ `mju_subQuat(res, qa, qb)` |
| qpos indexing | `m->jnt_qposadr[j]` | `model.jnt_qpos_adr[jnt_id]` | Direct port — `qpos_adr` maps to `jnt_qposadr` |
| DOF indexing | `m->jnt_dofadr[j]` | `model.jnt_dof_adr[jnt_id]` | Direct port — `dof_adr` maps to `jnt_dofadr` |
| `jnt_range` | Flat `jnt_range[2*i+0/1]` | `(f64, f64)` tuple: `jnt_range[i].0` / `.1` | Use `.0` for lower, `.1` for upper |
| `qfrc_spring` | `d->qfrc_spring` flat array | `DVector<f64>` indexed by DOF | `data.qfrc_spring[dof_adr + i]` |
| Sleep check | `body_awake` + `mjS_AWAKE` bit | `body_sleep_state[body_id] == SleepState::Asleep` | `model.jnt_body[jnt_id]` → `data.body_sleep_state[body_id]` |
| `finalize_row!` margin | `mj_addConstraint(..., &margin, ...)` | 4th arg of `finalize_row!` | Direct port — pass `margin` as 4th arg |
| `qpos_spring` (ball/free) | Per-joint reference config, size nq; ball: 4 quaternion values; free: 7 pos+quat values. Populated from `qpos0` (body initial pose), NOT from XML `springref` (scalar, ignored for ball/free). | `model.qpos_spring: Vec<f64>` indexed by `qpos_adr`. Already populated correctly by `builder/joint.rs:186-196`. | Direct port — `model.qpos_spring[qpos_adr..qpos_adr+nq]` |
| `JointContext` fields | N/A | `ctx.jnt_id`, `ctx.jnt_type`, `ctx.dof_adr`, `ctx.qpos_adr`, `ctx.nv`, `ctx.nq` | Branch on `ctx.jnt_type` within `visit_multi_dof_joint` |
| Energy accumulation | `d->energy[0]` (potential energy scalar) | `potential` local variable → `data.energy_potential` | Direct port |

---

## Specification

### S1. Ball/free spring force in `passive.rs`

**File:** `sim/L0/core/src/forward/passive.rs` (lines 818-835)
**MuJoCo equivalent:** Ball/free case in `mj_passive()` / `mj_springdamper()`
in `engine_passive.c`
**Design decision:** Add spring force computation to the existing
`visit_multi_dof_joint()` method. This keeps the visitor pattern consistent
with `visit_1dof_joint()` which already handles spring + damper. The method
must branch on `ctx.jnt_type` because free joints need 3 translational
Euclidean DOFs + 3 rotational quaternion DOFs, while ball joints need only 3
rotational DOFs. The `JointContext` carries `jnt_type: MjJointType` so this
branching is possible.

**Before** (current code):
```rust
fn visit_multi_dof_joint(&mut self, ctx: JointContext) {
    if self.is_joint_sleeping(&ctx) {
        return;
    }
    for i in 0..ctx.nv {
        let dof_idx = ctx.dof_adr + i;
        // S4.7b: Damper gated on DISABLE_DAMPER.
        if !self.implicit_mode && self.has_damper {
            let dof_damping = self.model.dof_damping[dof_idx];
            let qvel = self.data.qvel[dof_idx];
            self.data.qfrc_damper[dof_idx] -= dof_damping * qvel;
        }
    }
}
```

**After** (new implementation):
```rust
fn visit_multi_dof_joint(&mut self, ctx: JointContext) {
    if self.is_joint_sleeping(&ctx) {
        return;
    }

    // Spring force: gated on DISABLE_SPRING and implicit_mode.
    // Ball/free spring forces are fully suppressed in ImplicitSpringDamper
    // mode — matching MuJoCo (quaternion springs too nonlinear for diagonal
    // implicit approximation; implicit_stiffness=0.0 for ball/free).
    if !self.implicit_mode && self.has_spring {
        let stiffness = self.model.jnt_stiffness[ctx.jnt_id];
        if stiffness != 0.0 {
            match ctx.jnt_type {
                MjJointType::Free => {
                    // Translational: -k * (pos - pos_spring) for 3 DOFs
                    for i in 0..3 {
                        let q = self.data.qpos[ctx.qpos_adr + i];
                        let q_spring = self.model.qpos_spring[ctx.qpos_adr + i];
                        self.data.qfrc_spring[ctx.dof_adr + i] -=
                            stiffness * (q - q_spring);
                    }
                    // Rotational: falls through to ball case
                    // quat starts at qpos_adr + 3, rotational DOFs at dof_adr + 3
                    let quat_adr = ctx.qpos_adr + 3;
                    let rot_dof_adr = ctx.dof_adr + 3;
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        self.data.qpos[quat_adr],
                        self.data.qpos[quat_adr + 1],
                        self.data.qpos[quat_adr + 2],
                        self.data.qpos[quat_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        self.model.qpos_spring[quat_adr],
                        self.model.qpos_spring[quat_adr + 1],
                        self.model.qpos_spring[quat_adr + 2],
                        self.model.qpos_spring[quat_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    self.data.qfrc_spring[rot_dof_adr] -= stiffness * dif[0];
                    self.data.qfrc_spring[rot_dof_adr + 1] -= stiffness * dif[1];
                    self.data.qfrc_spring[rot_dof_adr + 2] -= stiffness * dif[2];
                }
                MjJointType::Ball => {
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        self.data.qpos[ctx.qpos_adr],
                        self.data.qpos[ctx.qpos_adr + 1],
                        self.data.qpos[ctx.qpos_adr + 2],
                        self.data.qpos[ctx.qpos_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        self.model.qpos_spring[ctx.qpos_adr],
                        self.model.qpos_spring[ctx.qpos_adr + 1],
                        self.model.qpos_spring[ctx.qpos_adr + 2],
                        self.model.qpos_spring[ctx.qpos_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    self.data.qfrc_spring[ctx.dof_adr] -= stiffness * dif[0];
                    self.data.qfrc_spring[ctx.dof_adr + 1] -= stiffness * dif[1];
                    self.data.qfrc_spring[ctx.dof_adr + 2] -= stiffness * dif[2];
                }
                _ => {} // Hinge/Slide handled by visit_1dof_joint
            }
        }
    }

    // Damper (existing code, unchanged)
    for i in 0..ctx.nv {
        let dof_idx = ctx.dof_adr + i;
        if !self.implicit_mode && self.has_damper {
            let dof_damping = self.model.dof_damping[dof_idx];
            let qvel = self.data.qvel[dof_idx];
            self.data.qfrc_damper[dof_idx] -= dof_damping * qvel;
        }
    }
}
```

**Required imports:** Add `use crate::tendon::subquat;` and
`use nalgebra::{Quaternion, UnitQuaternion};` to `passive.rs`.

### S2. Ball/free spring energy + pre-existing fixes in `energy.rs`

**File:** `sim/L0/core/src/energy.rs` (lines 7-58)
**MuJoCo equivalent:** `mj_energyPos()` in `engine_core_smooth.c`
**Design decision:** Fix all 3 pre-existing gaps in the same function for all
joint types (not just ball/free), then add ball/free energy computation. This
avoids leaving known conformance bugs for hinge/slide while fixing ball/free.
The 3 fixes are: (1) add `DISABLE_SPRING` gate around entire spring section,
(2) add sleep filter on spring section only (NOT gravity section), (3) change
stiffness guard from `> 0.0` to `== 0.0` continue.

**Before** (current code):
```rust
use crate::types::{DISABLE_GRAVITY, Data, MjJointType, Model};
use nalgebra::Vector3;

pub(crate) fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        Vector3::zeros()
    } else {
        model.gravity
    };

    // Gravitational potential energy (unconditional — no sleep filter)
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        potential -= mass * grav.dot(&com);
    }

    // Spring potential energy
    for jnt_id in 0..model.njnt {
        let stiffness = model.jnt_stiffness[jnt_id];
        if stiffness > 0.0 {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let springref = model.qpos_spring[qpos_adr];
                    let displacement = q - springref;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free joint springs would use quaternion distance
                    // Not commonly used, skip for now
                }
            }
        }
    }

    data.energy_potential = potential;
}
```

**After** (new implementation):
```rust
use crate::tendon::subquat;
use crate::types::{
    DISABLE_GRAVITY, DISABLE_SPRING, Data, ENABLE_SLEEP, MjJointType, Model, SleepState,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

pub(crate) fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        Vector3::zeros()
    } else {
        model.gravity
    };

    // Phase 1: Gravitational potential energy — unconditional (no sleep filter).
    // MuJoCo computes gravity PE for ALL bodies regardless of sleep state.
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        potential -= mass * grav.dot(&com);
    }

    // Phase 2: Spring potential energy — gated on DISABLE_SPRING + sleep filter.
    if model.disableflags & DISABLE_SPRING == 0 {
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        for jnt_id in 0..model.njnt {
            let stiffness = model.jnt_stiffness[jnt_id];
            // MuJoCo: `if (stiffness == 0) continue;` — exact zero check.
            // Processes negative stiffness (though physically meaningless).
            if stiffness == 0.0 {
                continue;
            }

            // Sleep filter: skip sleeping bodies' springs.
            if sleep_enabled {
                let body_id = model.jnt_body[jnt_id];
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }

            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let springref = model.qpos_spring[qpos_adr];
                    let displacement = q - springref;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Free => {
                    // Translational: E = 0.5 * k * ||pos - pos_spring||²
                    let mut trans_sq = 0.0;
                    for i in 0..3 {
                        let d = data.qpos[qpos_adr + i]
                            - model.qpos_spring[qpos_adr + i];
                        trans_sq += d * d;
                    }
                    potential += 0.5 * stiffness * trans_sq;

                    // Falls through to ball case for rotational energy.
                    let quat_adr = qpos_adr + 3;
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        data.qpos[quat_adr],
                        data.qpos[quat_adr + 1],
                        data.qpos[quat_adr + 2],
                        data.qpos[quat_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        model.qpos_spring[quat_adr],
                        model.qpos_spring[quat_adr + 1],
                        model.qpos_spring[quat_adr + 2],
                        model.qpos_spring[quat_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    potential += 0.5 * stiffness * dif.dot(&dif);
                }
                MjJointType::Ball => {
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        model.qpos_spring[qpos_adr],
                        model.qpos_spring[qpos_adr + 1],
                        model.qpos_spring[qpos_adr + 2],
                        model.qpos_spring[qpos_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    potential += 0.5 * stiffness * dif.dot(&dif);
                }
            }
        }
    }

    data.energy_potential = potential;
}
```

### S3. Parse `margin` attribute for `<joint>` and `<default><joint>`

**File:** `sim/L0/mjcf/src/parser.rs` (~line 1662, `parse_joint_attrs()`)
and (~line 600, `parse_joint_defaults()`)
**MuJoCo equivalent:** `margin` attribute on `<joint>` element
**Design decision:** Follow the existing pattern for scalar joint attributes
(e.g., `damping`, `stiffness`). Add `margin` field to both `MjcfJoint` and
`MjcfJointDefaults` as `Option<f64>`, parse with `parse_float_attr(e,
"margin")`.

**After** (additions to `parse_joint_attrs`):
```rust
// In parse_joint_attrs(), after existing float attrs:
joint.margin = parse_float_attr(e, "margin");
```

**After** (additions to `parse_joint_defaults`):
```rust
// In parse_joint_defaults(), after existing float attrs:
defaults.margin = parse_float_attr(e, "margin");
```

### S4. Add `margin` field to MJCF types

**File:** `sim/L0/mjcf/src/types.rs` (MjcfJoint and MjcfJointDefaults structs)
**MuJoCo equivalent:** `margin` attribute on `<joint>` element
**Design decision:** Add `margin: Option<f64>` to both structs, following the
pattern of `damping`, `stiffness`, etc.

**After** (addition to `MjcfJointDefaults`):
```rust
pub struct MjcfJointDefaults {
    // ... existing fields ...
    /// Joint limit activation margin.
    pub margin: Option<f64>,
}
```

**After** (addition to `MjcfJoint`):
```rust
pub struct MjcfJoint {
    // ... existing fields ...
    /// Joint limit activation margin.
    pub margin: Option<f64>,
}
```

Also update `MjcfJoint::default()` to include `margin: None`.

### S5. Wire `margin` through defaults cascade

**File:** `sim/L0/mjcf/src/defaults.rs` (~line 190, `apply_to_joint()`)
**MuJoCo equivalent:** Default class resolution for `margin` in
`mjCJoint` / `mjXReader::Default()`
**Design decision:** Follow the existing `if result.X.is_none() {
result.X = defaults.X; }` pattern used for all other joint attributes.

**After** (addition to `apply_to_joint`):
```rust
// In apply_to_joint(), after existing field cascades:
if result.margin.is_none() {
    result.margin = defaults.margin;
}
```

### S6. Add `jnt_margin` to Model and wire through builder

**File:** `sim/L0/core/src/types/model.rs` (~line 199),
`sim/L0/core/src/types/model_init.rs` (~line 100),
`sim/L0/mjcf/src/builder/joint.rs` (~line 160)
**MuJoCo equivalent:** `m->jnt_margin` array in `mjModel`
**Design decision:** Add `jnt_margin: Vec<f64>` to Model following the
existing joint field pattern. Default is `0.0` per MuJoCo. Wire through
the builder with `joint.margin.unwrap_or(0.0)`.

**After** (addition to `model.rs`, in Joints section after `jnt_actgravcomp`):
```rust
/// Joint limit activation margin. Constraint activated when dist < margin.
/// MuJoCo ref: `m->jnt_margin[i]` in `mj_instantiateLimit()`.
pub jnt_margin: Vec<f64>,
```

**After** (addition to `model_init.rs`, in Joints init section):
```rust
jnt_margin: vec![],
```

**After** (addition to `builder/joint.rs`, near other joint field pushes):
```rust
self.jnt_margin.push(joint.margin.unwrap_or(0.0));
```

The `ModelBuilder` struct declaration in `sim/L0/mjcf/src/builder/mod.rs:383`
needs a `jnt_margin: Vec<f64>` field. The field initialization (`jnt_margin:
vec![]`) goes in `ModelBuilder::new()` in `sim/L0/mjcf/src/builder/init.rs`.
The finalize/build method needs `model.jnt_margin = self.jnt_margin;`.

### S7. Replace hardcoded activation checks + `finalize_row!` margin args

**File:** `sim/L0/core/src/constraint/assembly.rs`
**MuJoCo equivalent:** `mj_instantiateLimit()` in `engine_core_constraint.c`
**Design decision:** Read `model.jnt_margin[jnt_id]` once per joint, use it
for all activation checks on that joint (both limits for hinge/slide, one
limit for ball). Pass the same margin value to `finalize_row!` as the 4th arg.

**Counting–assembly consistency invariant:** Both phases MUST use identical
activation conditions. If counting uses `< margin` but assembly uses `< 0.0`
(or vice versa), the row count won't match allocation, causing a panic.

All 9 modification sites, paired by counting↔assembly:

**Counting phase (Phase 1):**

| Site | Line | Current | After |
|------|------|---------|-------|
| C1: Hinge/Slide lower | :109 | `q - limit_min < 0.0` | `q - limit_min < margin` |
| C2: Hinge/Slide upper | :113 | `limit_max - q < 0.0` | `limit_max - q < margin` |
| C3: Ball | :128 | `dist < 0.0` | `dist < margin` |

**Assembly phase (Phase 3):**

| Site | Line | Current | After |
|------|------|---------|-------|
| A1: Hinge/Slide lower (activation) | :439 | `dist_lower < 0.0` | `dist_lower < margin` |
| A2: Hinge/Slide upper (activation) | :460 | `dist_upper < 0.0` | `dist_upper < margin` |
| A3: Ball (activation) | :492 | `dist < 0.0` | `dist < margin` |
| F1: Hinge/Slide lower `finalize_row!` | :448 | `0.0` (4th arg) | `margin` |
| F2: Hinge/Slide upper `finalize_row!` | :469 | `0.0` (4th arg) | `margin` |
| F3: Ball `finalize_row!` | :508 | `0.0` (4th arg) | `margin` |

**Before** (counting, hinge/slide):
```rust
if q - limit_min < 0.0 {
    nefc += 1;
}
if limit_max - q < 0.0 {
    nefc += 1;
}
```

**After** (counting, hinge/slide):
```rust
let margin = model.jnt_margin[jnt_id];
if q - limit_min < margin {
    nefc += 1;
}
if limit_max - q < margin {
    nefc += 1;
}
```

**Before** (counting, ball):
```rust
if dist < 0.0 {
    nefc += 1;
}
```

**After** (counting, ball):
```rust
let margin = model.jnt_margin[jnt_id];
if dist < margin {
    nefc += 1;
}
```

**Before** (assembly, hinge/slide lower):
```rust
let dist_lower = q - limit_min;
if dist_lower < 0.0 {
    data.efc_J[(row, dof_adr)] = 1.0;
    finalize_row!(sr, si, dist_lower, 0.0, qdot, 0.0,
                  ConstraintType::LimitJoint, 1, jnt_id, [0.0; 5]);
}
```

**After** (assembly, hinge/slide lower):
```rust
let margin = model.jnt_margin[jnt_id];
let dist_lower = q - limit_min;
if dist_lower < margin {
    data.efc_J[(row, dof_adr)] = 1.0;
    finalize_row!(sr, si, dist_lower, margin, qdot, 0.0,
                  ConstraintType::LimitJoint, 1, jnt_id, [0.0; 5]);
}
```

**After** (assembly, hinge/slide upper):
```rust
let dist_upper = limit_max - q;
if dist_upper < margin {
    data.efc_J[(row, dof_adr)] = -1.0;
    finalize_row!(sr, si, dist_upper, margin, -qdot, 0.0,
                  ConstraintType::LimitJoint, 1, jnt_id, [0.0; 5]);
}
```

**After** (assembly, ball):
```rust
let margin = model.jnt_margin[jnt_id];
// ... existing ball quaternion / axis-angle code ...
if dist < margin {
    // ... existing Jacobian code ...
    finalize_row!(model.jnt_solref[jnt_id], model.jnt_solimp[jnt_id],
                  dist, margin, vel, 0.0,
                  ConstraintType::LimitJoint, 1, jnt_id, [0.0; 5]);
}
```

**Non-modification sites:** Tendon limit activation checks (`assembly.rs:540,
552`) also use `< 0.0` but are NOT modified — tendon margins are a separate
feature (`tendon_margin`, not part of §64a).

---

## Acceptance Criteria

### AC1: Ball joint spring force — 90° rotation *(runtime test, analytically derived)*
**Given:** Single ball joint, `stiffness=2.0`, qpos = 90° rotation about X
from identity reference (`qpos_spring = [1,0,0,0]`).
**After:** `forward()` (which calls `mj_passive()`)
**Assert:** `qfrc_spring[dof_adr]` = `-2.0 * π/2` ≈ `-3.14159`, components
[1] and [2] ≈ 0.0 (rotation is purely about X axis). Tolerance: ± 1e-10.
**Field:** `Data.qfrc_spring`

### AC2: Free joint spring force — translational + rotational *(runtime test, analytically derived)*
**Given:** Single free joint, `stiffness=1.0`, qpos displaced by [1,0,0] in
position and 90° about Z in orientation from reference.
**After:** `forward()`
**Assert:** Translational: `qfrc_spring[0..3]` = `[-1.0, 0.0, 0.0]`.
Rotational: `qfrc_spring[3..6]` — component [5] (Z axis) ≈ `-π/2` ≈
`-1.5708`, components [3] and [4] ≈ 0.0. Tolerance: ± 1e-10.
**Field:** `Data.qfrc_spring`

### AC3: Ball joint spring energy — 90° rotation *(runtime test, analytically derived)*
**Given:** Same model as AC1, with `ENABLE_ENERGY` set.
**After:** `forward()`
**Assert:** Spring contribution to `energy_potential` = `0.5 * 2.0 * (π/2)²`
≈ `2.4674` ± 1e-4.
**Field:** `Data.energy_potential` (compare with/without stiffness to isolate
spring contribution from gravity)

### AC4: Free joint spring energy — translational + rotational *(runtime test, analytically derived)*
**Given:** Same model as AC2, with `ENABLE_ENERGY` set.
**After:** `forward()`
**Assert:** Spring energy = `0.5 * 1.0 * (1² + 0² + 0²)` (translational) +
`0.5 * 1.0 * (π/2)²` (rotational) = `0.5 + 1.2337` ≈ `1.7337` ± 1e-4.
**Field:** `Data.energy_potential`

### AC5: Zero stiffness — no spring force or energy *(runtime test, analytically derived)*
**Given:** Ball joint with `stiffness=0.0`, qpos = 90° rotation about X from
identity reference (`qpos_spring = [1,0,0,0]`).
**After:** `forward()`
**Assert:** `qfrc_spring[dof_adr..dof_adr+3]` = `[0.0, 0.0, 0.0]`.
Spring energy contribution = 0.
**Field:** `Data.qfrc_spring`, `Data.energy_potential`

### AC6: `DISABLE_SPRING` suppresses spring force AND energy for ALL joint types *(runtime test, analytically derived)*
**Given:** Model with 4 joints (hinge stiffness=1.0 displaced 1 rad, slide
stiffness=1.0 displaced 1m, ball stiffness=1.0 displaced 90° about X, free
stiffness=1.0 displaced [1,0,0] + 90° about Z), `DISABLE_SPRING` flag set,
`ENABLE_ENERGY` set.
**After:** `forward()`
**Assert:** `qfrc_spring` is all zeros (all DOFs). `energy_potential` equals
gravitational PE only — compute expected gravitational PE from the body masses
and positions, assert spring contribution = 0 by comparing with a reference
run that has `stiffness=0` for all joints (same gravity). Tolerance: exact
match (spring suppression is discrete, not approximate).
**Field:** `Data.qfrc_spring`, `Data.energy_potential`

### AC7: `implicit_mode` suppresses ball/free spring force *(runtime test)*
**Given:** Ball joint with `stiffness=2.0`, integrator =
`ImplicitSpringDamper`, qpos = 90° rotation about X from identity reference
(`qpos_spring = [1,0,0,0]`).
**After:** `forward()`
**Assert:** `qfrc_spring[dof_adr..dof_adr+3]` = `[0.0, 0.0, 0.0]` (spring
force fully suppressed — neither explicit nor implicit for ball/free).
**Field:** `Data.qfrc_spring`

### AC8: `margin=0` regression — identical behavior to current code *(runtime test)*
**Given:** Hinge joint with limits `[-1.0, 1.0]`, `margin=0.0` (default),
`qpos=-1.5` (violates lower limit).
**After:** `assemble_unified_constraints()`
**Assert:** `nefc`, `efc_pos`, `efc_margin`, `efc_aref` values identical to
current code (before this change). `efc_margin[row]` = 0.0.
**Field:** `Data.nefc`, `Data.efc_margin`, `Data.efc_pos`

### AC9: `margin > 0` pre-activation *(runtime test, analytically derived)*
**Given:** Hinge joint with limits `[-1.0, 1.0]`, `margin=0.1`, `qpos=-0.95`
(inside limits but within margin of lower limit: dist = -0.95 - (-1.0) =
0.05 < 0.1).
**After:** `assemble_unified_constraints()`
**Assert:** `nefc >= 1` (constraint pre-activated). `efc_margin[row]` = 0.1.
`efc_pos[row]` = 0.05.
**Field:** `Data.nefc`, `Data.efc_margin`, `Data.efc_pos`

### AC10: Ball joint with margin *(runtime test, analytically derived)*
**Given:** Ball joint with limits `[0.0, 1.0]` (max angle 1.0 rad),
`margin=0.05`. Set qpos to quaternion representing 0.97 rad rotation about Z:
`[cos(0.485), 0, 0, sin(0.485)]`. Ball limit angle = 0.97 rad, limit = 1.0,
dist = 1.0 - 0.97 = 0.03 < 0.05 (within margin zone).
**After:** `assemble_unified_constraints()`
**Assert:** `nefc >= 1` (constraint pre-activated). `efc_margin[row]` = 0.05.
`efc_pos[row]` ≈ 0.03 ± 1e-6.
**Field:** `Data.nefc`, `Data.efc_margin`, `Data.efc_pos`

### AC11: `DISABLE_LIMIT` ignores margin *(runtime test)*
**Given:** Hinge joint with limits `[-1.0, 1.0]`, `margin=0.5`, `qpos=0.0`
(within margin of both limits), `DISABLE_LIMIT` flag set.
**After:** `assemble_unified_constraints()`
**Assert:** `nefc` = 0 (no limit constraints regardless of margin).
**Field:** `Data.nefc`

### AC12: Sleep filter on spring energy *(runtime test, analytically derived)*
**Given:** Model with 1 body (mass=1.0) at height z=1.0, hinge joint with
`stiffness=2.0`, `qpos=0.5` (displaced from `springref=0.0`). Body set to
`SleepState::Asleep`. `ENABLE_ENERGY` and `ENABLE_SLEEP` set. Gravity =
`[0, 0, -9.81]`.
**After:** `forward()`
**Assert:** `energy_potential` ≈ `1.0 * 9.81 * 1.0` = 9.81 (gravitational PE
only — sleeping body's spring energy `0.5 * 2.0 * 0.25 = 0.25` is excluded).
A reference run with same model but `SleepState::Awake` should give
`energy_potential` ≈ 9.81 + 0.25 = 10.06. Tolerance: ± 1e-6.
**Field:** `Data.energy_potential`

### AC13: Negative stiffness produces force/energy *(runtime test, analytically derived)*
**Given:** Hinge joint with `stiffness=-1.0`, `qpos=1.0`, `springref=0.0`.
**After:** `forward()`
**Assert:** `qfrc_spring[dof_adr]` = `-(-1.0) * (1.0 - 0.0)` = `1.0` (force
is computed, not skipped). Spring energy = `0.5 * (-1.0) * 1.0²` = `-0.5`
(negative, physically meaningless but MuJoCo computes it).
**Field:** `Data.qfrc_spring`, `Data.energy_potential`

### AC14: `margin` parses from MJCF and defaults *(runtime test)*
**Given:** MJCF with `<default><joint margin="0.05"/></default>` and a joint
that inherits defaults.
**After:** Parse + build
**Assert:** `model.jnt_margin[jnt_id]` = 0.05.
**Field:** `Model.jnt_margin`

### AC15: No new `unsafe` blocks *(code review)*
All new code is safe Rust. No `unsafe` blocks added.

### AC16: §60 not implemented *(code review)*
No `springinertia` field, attribute, or CRBA modification exists in the
codebase. Verified by grep.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (ball spring force 90°) | T1 | Direct |
| AC2 (free spring force trans+rot) | T2 | Direct |
| AC3 (ball spring energy 90°) | T3 | Direct |
| AC4 (free spring energy trans+rot) | T4 | Direct |
| AC5 (zero stiffness) | T5 | Edge case |
| AC6 (DISABLE_SPRING all types) | T6 | Direct |
| AC7 (implicit_mode ball/free) | T7 | Edge case |
| AC8 (margin=0 regression) | T8 | Regression |
| AC9 (margin>0 pre-activation) | T9 | Direct |
| AC10 (ball margin) | T10 | Direct |
| AC11 (DISABLE_LIMIT + margin) | T11 | Edge case |
| AC12 (sleep filter spring energy) | T12 | Direct |
| AC13 (negative stiffness) | T13 | Edge case |
| AC14 (margin parse + defaults) | T14 | Direct |
| AC15 (no unsafe) | — | Code review (manual) |
| AC16 (§60 not implemented) | — | Code review (manual) |

---

## Test Plan

### T1: Ball joint spring force — 90° rotation → AC1
Build model: 1 body with ball joint, `stiffness=2.0`. Set `qpos` to 90°
rotation about X: `[cos(π/4), sin(π/4), 0, 0]` = `[0.7071, 0.7071, 0, 0]`.
Reference: identity `[1, 0, 0, 0]` (default `qpos_spring` from `qpos0`).
Run `forward()`. Assert `qfrc_spring[dof_adr]` ≈ `-π` (≈ -3.14159),
`qfrc_spring[dof_adr+1]` ≈ 0, `qfrc_spring[dof_adr+2]` ≈ 0. Tolerance 1e-10.
Expected value: analytically derived from `subquat([cos45, sin45, 0, 0],
[1,0,0,0])` = `[π/2, 0, 0]`, force = `-2.0 * [π/2, 0, 0]`.

### T2: Free joint spring force — translational + rotational → AC2
Build model: 1 body with free joint, `stiffness=1.0`. Set `qpos` to
`[1, 0, 0, cos(π/4), 0, 0, sin(π/4)]` (displaced 1m in X, rotated 90° about
Z). Reference: `qpos_spring = [0, 0, 0, 1, 0, 0, 0]` (origin, identity).
Run `forward()`. Assert translational force `[-1.0, 0.0, 0.0]` at
`dof_adr..+3`. Assert rotational force — Z component ≈ `-π/2` at
`dof_adr+5`, X/Y ≈ 0. Tolerance 1e-10.

### T3: Ball joint spring energy — 90° rotation → AC3
Same model as T1, with `ENABLE_ENERGY`. Run `forward()`. Measure
`energy_potential`. Compare with same model at zero stiffness to isolate
spring contribution. Assert spring energy ≈ `0.5 * 2.0 * (π/2)²` ≈ 2.4674.
Tolerance 1e-4. Comment: "Analytically derived from `0.5 * k * θ²` where
`θ = π/2` (90° rotation)."

### T4: Free joint spring energy — translational + rotational → AC4
Same model as T2, with `ENABLE_ENERGY`. Run `forward()`. Isolate spring
energy contribution. Assert ≈ `0.5 * 1.0 * 1.0` (trans) + `0.5 * 1.0 *
(π/2)²` (rot) ≈ 1.7337. Tolerance 1e-4.

### T5: Zero stiffness — no force or energy → AC5
Ball joint, `stiffness=0.0`, displaced 90° from reference. Run `forward()`.
Assert `qfrc_spring` all zeros for that joint's DOFs. Assert spring energy
contribution = 0.

### T6: `DISABLE_SPRING` suppresses all 4 joint types → AC6
Model with 4 joints: hinge (stiffness=1.0, displaced 1 rad), slide
(stiffness=1.0, displaced 1m), ball (stiffness=1.0, displaced 90° about X),
free (stiffness=1.0, displaced [1,0,0] + 90° about Z). Set `DISABLE_SPRING`
flag and `ENABLE_ENERGY`. Run `forward()`. Assert `qfrc_spring` is all zeros
across all DOFs (1 hinge + 1 slide + 3 ball + 6 free = 11 DOFs). Assert
`energy_potential` = gravitational PE only — compare with reference run using
`stiffness=0` for all joints (same gravity, same positions). Tests all 4
joint types explicitly.

### T7: `implicit_mode` suppresses ball/free spring force → AC7
Ball joint with `stiffness=2.0`, integrator = `ImplicitSpringDamper`, 90°
from reference. Run `forward()`. Assert `qfrc_spring` is all zeros for ball
DOFs. Document: "Ball/free spring forces are fully suppressed in implicit
mode — matching MuJoCo."

### T8: `margin=0` regression → AC8
Hinge joint, limits `[-1.0, 1.0]`, `margin=0.0` (default). Set `qpos=-1.5`
(violates lower limit). Run `assemble_unified_constraints()`. Assert `nefc`
includes this constraint. Assert `efc_margin[row]` = 0.0. Assert `efc_pos`
matches current behavior.

### T9: `margin > 0` pre-activation — hinge → AC9
Hinge joint, limits `[-1.0, 1.0]`, `margin=0.1`. Set `qpos=-0.95` (inside
limit but within margin zone: dist = 0.05 < 0.1). Run assembly. Assert
`nefc >= 1`. Assert `efc_margin[row]` = 0.1. Assert `efc_pos[row]` = 0.05.

### T10: Ball joint with margin → AC10
Ball joint with limits `[0.0, 1.0]` (max angle 1 rad), `margin=0.05`. Set
angle to 0.97 rad (within limit but dist = 1.0 - 0.97 = 0.03 < 0.05). Run
assembly. Assert `nefc >= 1` (pre-activated). Assert `efc_margin[row]` = 0.05.
Assert `efc_pos[row]` ≈ 0.03 ± 1e-6.

### T11: `DISABLE_LIMIT` ignores margin → AC11
Hinge with large `margin=0.5`, limits `[-1.0, 1.0]`, `qpos=0.0` (within
margin of both limits). Set `DISABLE_LIMIT` flag. Run assembly. Assert
`nefc` = 0 (no limit constraints).

### T12: Sleep filter on spring energy → AC12
Build model: 1 body (mass=1.0) at height z=1.0, hinge with `stiffness=2.0`,
`qpos=0.5`, `springref=0.0`. Set `body_sleep_state[body_id] =
SleepState::Asleep`. Set `ENABLE_ENERGY` + `ENABLE_SLEEP`. Run `forward()`.
Assert `energy_potential` ≈ 9.81 (gravitational PE only — spring energy
`0.5 * 2.0 * 0.25 = 0.25` excluded because body is sleeping). Compare with
same model at `SleepState::Awake`: `energy_potential` ≈ 10.06. Tolerance
1e-6. Verifies: sleeping body's springs skipped, gravitational PE
unconditional.

### T13: Negative stiffness → force and energy computed → AC13
Hinge with `stiffness=-1.0`, `qpos=1.0`, `springref=0.0`, `ENABLE_ENERGY`.
Run `forward()`. Assert `qfrc_spring[dof_adr]` = `1.0` (not zero — negative
stiffness IS processed). Assert spring energy ≈ `-0.5`.

### T14: Margin parsed from MJCF defaults → AC14
MJCF string: `<default><joint margin="0.05"/></default>`, body with joint
inheriting defaults. Parse and build. Assert `model.jnt_margin[0]` = 0.05.
Also test explicit override: joint with `margin="0.1"` overrides default of
0.05.

### T15: Near-180° rotation edge case → AC1 (supplementary)
Ball joint, `stiffness=1.0`, rotation of 179° about Y from reference. Run
`forward()`. Assert `qfrc_spring` magnitude ≈ `179° * π/180` ≈ 3.1241 (not
wrapped to negative). Tolerance 1e-6. Tests wrapping boundary.

### T16: Identity quaternion — zero force/energy → AC5 (supplementary)
Ball joint with `stiffness=5.0`, `qpos` = identity = reference. Run
`forward()`. Assert `qfrc_spring` = `[0, 0, 0]`. Assert spring energy = 0.

### T17: Multi-joint model — free + hinge chain → AC2 (supplementary)
Model: body1 with free joint (stiffness=1), body2 child with hinge
(stiffness=2). Displace both from reference. Run `forward()`. Assert
`qfrc_spring` has correct values for both joints — verifies indexing doesn't
cross-contaminate.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Zero stiffness | Must produce zero force/energy (exact `== 0` guard) | T5 | AC5 |
| Identity quaternion reference | Zero rotation → zero force/energy | T16 | AC5 |
| 90° rotation | Standard test case for quaternion math correctness | T1, T3 | AC1, AC3 |
| Near-180° rotation | Wrapping boundary in `subquat` — tests shortest-path | T15 | AC1 |
| Free joint: trans + rot combined | Verifies translational uses Euclidean, rotational uses quaternion; correct DOF/qpos indexing | T2, T4 | AC2, AC4 |
| `DISABLE_SPRING` flag | Spring force AND energy suppressed for ALL joint types | T6 | AC6 |
| `margin=0` regression | Must produce identical output to current code | T8 | AC8 |
| `margin > 0` pre-activation | Constraint created before limit violation | T9 | AC9 |
| Ball joint with margin | Ball limit uses margin correctly | T10 | AC10 |
| `DISABLE_LIMIT` ignores margin | No constraints regardless of margin value | T11 | AC11 |
| Sleep filter on spring energy | Sleeping bodies excluded from spring energy but NOT gravitational energy | T12 | AC12 |
| `implicit_mode` suppresses ball/free spring | Total suppression — neither explicit nor implicit | T7 | AC7 |
| Free joint qpos_spring layout | Trans reference at `qpos_adr..+3`, quat at `qpos_adr+3..+7` | T2, T17 | AC2 |
| Negative stiffness | Force/energy IS computed (matches MuJoCo `== 0` guard) | T13 | AC13 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T15 (near-180°) | Wrapping boundary in `subquat` | Catches sign-flip bugs at the quaternion double-cover boundary |
| T16 (identity quat) | Zero-rotation edge case | Catches division-by-zero in `subquat` small-angle path |
| T17 (multi-joint) | Cross-joint indexing | Catches `qpos_adr`/`dof_adr` indexing bugs that only appear with multiple joints |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Ball/free spring force | `qfrc_spring` = 0 for ball/free joints | `qfrc_spring` = `-k * subquat(q, q_spring)` for ball, `-k * (pos - pos_spring)` + rotational for free | Toward MuJoCo | Models with ball/free joint stiffness | None — transparent (stiffness=0 was the common case) |
| Ball/free spring energy | `energy_potential` missing spring contribution for ball/free | Includes `0.5 * k * θ²` (ball) and `0.5 * k * (||d||² + θ²)` (free) | Toward MuJoCo | Same models, when `ENABLE_ENERGY` set | None — more correct values |
| `DISABLE_SPRING` gate on energy (ALL types) | Spring energy always computed even when `DISABLE_SPRING` set | Spring energy = 0 when `DISABLE_SPRING` set | Toward MuJoCo | Any model using `DISABLE_SPRING` flag | None — fixes a bug |
| Sleep filter on spring energy (ALL types) | Sleeping bodies' springs contribute to `energy_potential` | Sleeping bodies' springs excluded from `energy_potential` | Toward MuJoCo | Models with sleep enabled + spring joints | None — fixes a bug |
| Stiffness guard (ALL types) | `stiffness > 0.0` (skips negative) | `stiffness == 0.0` (processes negative) | Toward MuJoCo | Models with negative stiffness (rare/none) | None — no practical impact |
| Joint limit activation (margin=0) | `dist < 0.0` | `dist < 0.0` (identical — margin defaults to 0) | Already conformant | None | None — backward compatible |
| Joint limit activation (margin>0) | Not possible | `dist < margin` (pre-activation) | Toward MuJoCo | Models using `margin` attribute (new capability) | None — new feature |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/forward/passive.rs` | S1: Add ball/free spring force in `visit_multi_dof_joint()`, add `subquat` import | ~+50 |
| `sim/L0/core/src/energy.rs` | S2: Ball/free energy + DISABLE_SPRING gate + sleep filter + stiffness guard fix | ~+40 / -10 |
| `sim/L0/mjcf/src/parser.rs` | S3: Add `margin` to `parse_joint_attrs()` and `parse_joint_defaults()` | ~+2 |
| `sim/L0/mjcf/src/types.rs` | S4: Add `margin: Option<f64>` to `MjcfJoint` and `MjcfJointDefaults` | ~+4 |
| `sim/L0/mjcf/src/defaults.rs` | S5: Add `margin` to `apply_to_joint()` cascade | ~+3 |
| `sim/L0/core/src/types/model.rs` | S6: Add `jnt_margin: Vec<f64>` | ~+3 |
| `sim/L0/core/src/types/model_init.rs` | S6: Init `jnt_margin: vec![]` | ~+1 |
| `sim/L0/mjcf/src/builder/joint.rs` | S6: Push `jnt_margin` value | ~+2 |
| `sim/L0/mjcf/src/builder/mod.rs` | S6: Add `jnt_margin: Vec<f64>` field to `ModelBuilder` struct + finalize line | ~+2 |
| `sim/L0/mjcf/src/builder/init.rs` | S6: Init `jnt_margin: vec![]` in `ModelBuilder::new()` | ~+1 |
| `sim/L0/core/src/constraint/assembly.rs` | S7: Replace 6 activation checks + 3 `finalize_row!` margin args | ~+12 / -9 |
| `sim/L0/core/tests/` (or inline) | T1-T17: New tests | ~+400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_model_data_energy` | `lib.rs:316` | Pass (unchanged) | Uses zero gravity + kinetic energy; no spring stiffness set |
| `test_ball_limit_axis_angle_*` | `impedance.rs:511-580` | Pass (unchanged) | Tests the `ball_limit_axis_angle()` utility, not assembly |
| Constraint assembly tests | `assembly.rs` tests (if any) | Pass (unchanged) | Default `jnt_margin` = 0.0, so `< margin` = `< 0.0` — identical behavior |
| Passive force tests | `passive.rs` tests (if any) | Pass (unchanged) | Existing tests use hinge/slide joints; ball/free spring force is new code |
| `test_hill_passive_fl_curve` | `muscle.rs:2228` | Pass (unchanged) | Tests muscle-specific passive force, not joint springs |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `assembly.rs:148` | Tendon lower limit: `length - limit_min < 0.0` | Tendon margins are a separate feature — not part of §64a |
| `assembly.rs:152` | Tendon upper limit: `limit_max - length < 0.0` | Same — tendon limits, not joint limits |
| `assembly.rs:540` | Tendon limit assembly lower | Same |
| `assembly.rs:552` | Tendon limit assembly upper | Same |
| `passive.rs:800-814` | `visit_1dof_joint` spring/damper | Already correct for hinge/slide — not modified by this spec |
| `energy.rs` gravity loop | `for body_id in 1..model.nbody` | MuJoCo gravity loop is unconditional — must NOT add sleep filter here |

### Data Staleness

No `EXPECTED_SIZE` constants are affected. `jnt_margin` is a new `Vec<f64>`
on `Model`, and all other changes modify runtime behavior (energy computation,
constraint activation), not model structure counts.

---

## Execution Order

1. **S3, S4, S5, S6** (parse `margin`, add types, defaults cascade, model
   field) → verify with T14 (margin parse + defaults test). These are
   prerequisite for S7 but independent of S1/S2.

2. **S7** (replace activation checks + `finalize_row!` margin args) → verify
   with T8 (margin=0 regression), T9 (margin>0 pre-activation), T10 (ball
   margin), T11 (DISABLE_LIMIT).

3. **S1** (ball/free spring force) → verify with T1 (ball force), T2 (free
   force), T5 (zero stiffness), T7 (implicit_mode), T15 (near-180°), T16
   (identity), T17 (multi-joint).

4. **S2** (ball/free spring energy + pre-existing fixes) → verify with T3
   (ball energy), T4 (free energy), T6 (DISABLE_SPRING), T12 (sleep filter),
   T13 (negative stiffness).

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p
sim-conformance-tests` after each section group.

---

## Out of Scope

- **§60 (`springinertia`)** — Does not exist in MuJoCo (verified: EGT-1 in
  rubric). Zero GitHub search results, no `mjmodel.h` field, no XML attribute.
  Umbrella should note §60 as invalid. *Conformance impact: none — feature
  doesn't exist.*

- **Tendon limit margins** — Tendon limits (`assembly.rs:140-155, 524-570`)
  also hardcode `< 0.0` but are a separate feature. *Conformance impact:
  minor — tendon margins are uncommon. Tracked as future work.*

- **`implicit_stiffness` for ball/free joints** — MuJoCo's implicit integrator
  doesn't handle quaternion springs (sets `implicit_stiffness=0`). CortenForge
  matches this. No changes needed. *Conformance impact: none.*

- **`qpos_spring` runtime updates** — If `qpos_spring` needs updating during
  simulation (e.g., `setconst()`), that's a separate task. Currently
  `qpos_spring` is set at build time and static. *Conformance impact: minor
  for models that call `mj_setConst()` at runtime.*

- **DT-17 (Global `<option o_margin>`)** — Per-geom margin override. Not
  related to joint margin. *Conformance impact: minor.*
