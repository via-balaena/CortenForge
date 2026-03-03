# Spec B — Joint Physics: Spec Quality Rubric

Grades the Spec B spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

The umbrella assigned three tasks to Spec B. Empirical verification against
MuJoCo C source revealed one fabricated feature and one scope expansion:

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| §60: `springinertia` — parse from `<joint>`, store, add `springinertia * stiffness[dof]` to CRBA diagonal in `mj_crb()` | **Does not exist in MuJoCo.** Verified: `mjmodel.h` has no `dof_springinertia` or `jnt_springinertia` field. `mj_crb()` in `engine_core_smooth.c` initializes diagonal with `dof_armature[i]` only — no spring-inertia coupling. `<joint>` XML reference has no `springinertia` attribute. Full GitHub search for "springinertia" across `google-deepmind/mujoco` = 0 results. The umbrella's claim conflated `dof_armature` (which exists) with a nonexistent spring-inertia coupling. | **DROP** — not a MuJoCo feature; cannot implement conformance for a nonexistent behavior |
| §64: Ball/free spring potential energy only (energy.rs stub) | `future_work_15.md` §64 covers both spring **force** (`passive.rs`) and spring **energy** (`energy.rs`). The umbrella narrowed to energy only. However: (1) ball/free spring force is also missing (`visit_multi_dof_joint` in `passive.rs:818-835` does damping only, no spring); (2) force and energy use identical quaternion math (`mju_subQuat`); (3) energy without force is physically inconsistent — `qfrc_spring` would remain zero while `energy_potential` reports nonzero spring energy. | **Expand** to include both spring force (`passive.rs`) and energy (`energy.rs`). Same quaternion math, same `qpos_spring` reference, same `DISABLE_SPRING` gate. |
| §64a: `jnt_margin` — parse, store, wire into 6 activation checks + 3 `finalize_row!` margin args | Confirmed. 6 hardcoded `< 0.0` checks in `assembly.rs` (counting: lines 109, 113, 128; assembly: lines 439, 460, 492). 3 `finalize_row!` calls pass `0.0` as margin (lines 448, 469, 508). MuJoCo uses `dist < m->jnt_margin[i]` and passes `&margin` to `mj_addConstraint`. | **In scope — confirmed** |

**Final scope:**

1. **§64:** Ball/free joint spring force (`passive.rs`) + spring energy
   (`energy.rs`) via quaternion geodesic distance using `mju_subQuat`
   equivalent
2. **§64a:** `jnt_margin` — parse `margin` from `<joint>` and
   `<default><joint>`, store as `jnt_margin`, replace 6 activation
   checks + 3 `finalize_row!` margin arguments

§60 is dropped. Umbrella and future work documents should note §60 as
invalid (feature does not exist in MuJoCo).

---

## Empirical Ground Truth

All findings verified against MuJoCo C source on GitHub
(`google-deepmind/mujoco`, `main` branch). No local MuJoCo binary run
for this rubric — numerical expectations will be established in the spec
itself from the verified C source formulas.

### EGT-1: §60 (`springinertia`) Does Not Exist in MuJoCo

Verified across four authoritative sources:

1. **`mjmodel.h`:** No `dof_springinertia`, `jnt_springinertia`, or any
   `springinertia` field. The only spring-related DOF field is
   `dof_armature` (`mjtNum* dof_armature; // dof armature inertia/mass`).
2. **`engine_core_smooth.c` (`mj_crb()`):** Diagonal initialized with
   `M[Madr_ij] = dof_armature[i]`. No spring-stiffness coupling anywhere
   in the function.
3. **MuJoCo XML reference (`XMLreference.rst`):** `<joint>` attributes
   include `armature`, `stiffness`, `springref`, `margin`, `damping`,
   `frictionloss` — no `springinertia`.
4. **Full GitHub search:** Zero results for "springinertia" across the
   entire `google-deepmind/mujoco` repository.

The umbrella's claim that `M[dof,dof] += springinertia * stiffness[dof]`
is added in `mj_crb()` is incorrect. The diagonal augmentation is
`dof_armature` only.

### EGT-2: `mj_energyPos()` Spring Energy for Ball/Free Joints

Source: `engine_core_smooth.c` (or `engine_sensor.c` — file location
varies by MuJoCo version), function `mj_energyPos()`.

```c
case mjJNT_FREE:
  // Translational spring energy: 0.5 * k * ||pos - pos_spring||²
  mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
  d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);
  padr += 3;
  mjFALLTHROUGH;

case mjJNT_BALL:
  // Rotational spring energy: 0.5 * k * ||subQuat(q, q_spring)||²
  mju_copy4(quat, d->qpos+padr);
  mju_normalize4(quat);
  mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
  d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);
  break;
```

Key behaviors:

| Behavior | Detail |
|----------|--------|
| Free translational | `E = 0.5 * k * \|\|pos - pos_spring\|\|²` (3D Euclidean) |
| Free rotational | Falls through (`mjFALLTHROUGH`) to ball case, same stiffness |
| Ball rotational | `E = 0.5 * k * \|\|subQuat(q, q_spring)\|\|²` = `0.5 * k * θ²` |
| Zero stiffness guard | `if (stiffness == 0) continue;` — exact zero check |
| Disable flag | Gated on `mjDSBL_SPRING` |
| Sleep filter | Skips sleeping bodies |
| qpos_spring reference | Per-joint reference configuration, size `nq` |

### EGT-3: `mju_subQuat()` Implementation

Source: `engine_util_spatial.c`.

```c
void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]) {
  mjtNum qneg[4], qdif[4];
  mji_negQuat(qneg, qb);       // conjugate: [w, -x, -y, -z]
  mji_mulQuat(qdif, qneg, qa); // relative rotation: conj(qb) * qa
  mji_quat2Vel(res, qdif, 1);  // axis-angle vector with dt=1
}
```

`mji_quat2Vel` with `dt=1`:

```c
axis = normalize(xyz_components);  // returns sin_half = ||xyz||
angle = 2 * atan2(sin_half, w);
if (angle > PI) angle -= 2*PI;    // shortest-path wrapping
result = axis * angle;             // 3D axis-angle vector
```

The returned 3D vector has magnitude = rotation angle (radians) and
direction = rotation axis. Therefore `dot(dif, dif) = θ²`, and the
energy formula `0.5 * k * dot(dif, dif) = 0.5 * k * θ²`.

The wrapping `if (angle > PI) angle -= 2*PI` ensures shortest-path
rotation — handles the quaternion double-cover of SO(3).

**CortenForge equivalent:** `subquat()` in `tendon/spatial.rs:441`
uses `UnitQuaternion` from nalgebra but implements identical logic:
conjugate → multiply → axis-angle conversion with PI wrapping. Can be
reused for §64.

### EGT-4: Ball/Free Spring Force in MuJoCo (`engine_passive.c`)

Source: `mj_springdamper()` or `mj_passive()` in `engine_passive.c`.

**Ball joint:**
```c
mji_copy4(quat, d->qpos+padr);
mji_normalize4(quat);
mji_subQuat(dif, quat, m->qpos_spring + padr);
d->qfrc_spring[dadr+0] = -stiffness * dif[0];
d->qfrc_spring[dadr+1] = -stiffness * dif[1];
d->qfrc_spring[dadr+2] = -stiffness * dif[2];
```

**Free joint (translational, then falls through to ball):**
```c
d->qfrc_spring[dadr+0] = -stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
d->qfrc_spring[dadr+1] = -stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
d->qfrc_spring[dadr+2] = -stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);
padr += 3;
dadr += 3;
// falls through to ball case for rotational DOFs
```

Key behaviors:

| Behavior | Detail |
|----------|--------|
| Addressing | Force written to `dadr` (DOF-indexed); position read from `padr` (qpos-indexed) |
| Free: 6 DOFs, 7 qpos | 3 translational DOFs + 3 rotational DOFs; 3 pos + 4 quat in qpos |
| Ball: 3 DOFs, 4 qpos | 3 rotational DOFs; 4 quat in qpos |
| Same stiffness | Free joint uses single `jnt_stiffness[j]` for both translational and rotational |
| Disable flag | Gated on `DISABLE_SPRING` (same as hinge/slide) |
| Sleep filter | Skips sleeping bodies |

**CortenForge gap:** `visit_multi_dof_joint()` in `passive.rs:818-835`
only computes damping. No spring force for ball/free joints. The
`has_spring` field and `DISABLE_SPRING` gating infrastructure already
exist — only the force computation is missing.

### EGT-5: `mj_instantiateLimit()` Margin Handling

Source: `engine_core_constraint.c`, function `mj_instantiateLimit()`.

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
m->jnt_margin[i]
    |
    v
margin (local variable)
    |
    +---> activation test: if (dist < margin)
    |
    +---> mj_addConstraint(..., &margin, ...)
              |
              v
          d->efc_margin[nefc] = margin
              |
              +---> compute_impedance(solimp, |pos - margin|)
              +---> compute_aref(k, b, imp, pos, margin, vel)
```

Margin serves dual purpose:
1. **Activation threshold:** constraint created when `dist < margin`
2. **Stored on constraint row:** available to solver for impedance and
   reference acceleration computation

**Default value:** `margin = 0.0` (MuJoCo MJCF default). When `margin=0`,
the condition `dist < 0` matches CortenForge's current hardcoded behavior.

**Global disable:** `mjDSBL_LIMIT` disables all limit constraints (skips
the entire function). CortenForge already checks `DISABLE_LIMIT`.

### EGT-7: Pre-existing Conformance Gaps in `energy.rs`

CortenForge's `mj_energy_pos()` has three pre-existing conformance gaps
relative to MuJoCo's `mj_energyPos()`, discovered during stress testing:

1. **Missing `DISABLE_SPRING` gate.** MuJoCo gates the entire spring
   energy section on `!mjDISABLED(mjDSBL_SPRING)`. CortenForge enters the
   spring loop unconditionally (energy.rs:38). This means spring energy is
   always computed even when `DISABLE_SPRING` is set.

2. **Missing sleep filter on spring section.** MuJoCo's `mj_energyPos()`
   has a two-phase structure: (a) gravity loop runs unconditionally over
   ALL bodies (no sleep check), then (b) `sleep_filter` variable is
   computed and applied to the spring/tendon loops only. CortenForge
   iterates directly over joints with no sleep check (energy.rs:38,
   joint-based loop). **Important:** the sleep filter must NOT be added
   to the gravity section — MuJoCo computes gravitational PE for all
   bodies regardless of sleep state.

3. **`stiffness > 0.0` vs `stiffness == 0`.** CortenForge's guard
   (energy.rs:40) skips joints with zero OR negative stiffness. MuJoCo
   only skips exact zero (`if (stiffness == 0) continue;`). MuJoCo would
   process negative stiffness (though physically meaningless).

These gaps affect ALL joint types (hinge/slide/ball/free), not just the
ball/free cases being added. The spec must decide: fix all three for all
joint types (recommended — conformance fix), or note as deferred.

### EGT-8: `implicit_mode` Gate in `passive.rs`

The `PassiveForceVisitor` has an `implicit_mode: bool` field
(passive.rs:773) set to `true` when `model.integrator ==
Integrator::ImplicitSpringDamper` (passive.rs:385).

The hinge/slide spring force is inside `if !self.implicit_mode { ... }`
(passive.rs:800). The implicit integrator handles spring forces internally
via `implicit_stiffness` and `implicit_springref` arrays, so explicit
spring forces are suppressed to avoid double-counting.

The ball/free spring force added by §64 must be gated identically. The
existing `visit_multi_dof_joint` already gates damping on
`!self.implicit_mode` (passive.rs:829).

**Consequence:** `compute_implicit_params()` in `model_init.rs` sets
`implicit_stiffness = 0.0` and `implicit_springref = 0.0` for ball/free
joints (MuJoCo's implicit integrator doesn't handle quaternion springs).
Combined with the `!implicit_mode` gate, ball/free spring forces are
completely suppressed in `ImplicitSpringDamper` mode — neither explicit
(gated out) nor implicit (stiffness=0). This matches MuJoCo: quaternion
spring dynamics are too nonlinear for the diagonal implicit
approximation. The spec should document this as expected behavior, not
a gap.

### EGT-6: CortenForge Codebase Context

#### Files Spec B will touch

| File | Location | Current state | Spec B change |
|------|----------|---------------|---------------|
| `sim/L0/core/src/energy.rs` | :38-56 | Ball/Free stub (:50-53); missing `DISABLE_SPRING` gate (:38); missing sleep filter; `stiffness > 0.0` guard (:40) | §64: implement ball/free energy + fix pre-existing gaps (DISABLE_SPRING gate, spring-only sleep filter, stiffness `== 0` guard) for ALL joint types |
| `sim/L0/core/src/forward/passive.rs` | :818-835 | `visit_multi_dof_joint`: damping only, no spring | §64: add ball/free spring force |
| `sim/L0/core/src/constraint/assembly.rs` | :109 | `q - limit_min < 0.0` (counting, lower) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :113 | `limit_max - q < 0.0` (counting, upper) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :128 | `dist < 0.0` (counting, ball) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :439 | `dist_lower < 0.0` (assembly, lower) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :460 | `dist_upper < 0.0` (assembly, upper) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :492 | `dist < 0.0` (assembly, ball) | §64a: `< margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :448 | `finalize_row!(..., 0.0, ...)` margin arg | §64a: pass `margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :469 | `finalize_row!(..., 0.0, ...)` margin arg | §64a: pass `margin` |
| `sim/L0/core/src/constraint/assembly.rs` | :508 | `finalize_row!(..., 0.0, ...)` margin arg | §64a: pass `margin` |
| `sim/L0/mjcf/src/parser.rs` | :1662-1733 | `parse_joint_attrs()`: no `margin` parsed | §64a: add `margin` |
| `sim/L0/mjcf/src/parser.rs` | ~:600-620 | `parse_joint_defaults()`: no `margin` | §64a: add `margin` |
| `sim/L0/mjcf/src/types.rs` | :584-614 | `MjcfJointDefaults`: no `margin` field | §64a: add `margin: Option<f64>` |
| `sim/L0/mjcf/src/types.rs` | MjcfJoint section | `MjcfJoint`: no `margin` field | §64a: add `margin: Option<f64>` |
| `sim/L0/core/src/types/model.rs` | :163-205 | Joint fields: no `jnt_margin` | §64a: add `jnt_margin: Vec<f64>` |
| `sim/L0/core/src/types/model_init.rs` | :83-100 | Joint init block | §64a: init `jnt_margin: vec![]` |
| `sim/L0/mjcf/src/builder/joint.rs` | ~:160 | Joint builder | §64a: wire `margin` → `jnt_margin` |
| `sim/L0/mjcf/src/defaults.rs` | ~:169 | `apply_to_joint()` | §64a: add `margin` cascade |

#### Existing utilities to reuse

| Utility | File:Line | Use |
|---------|-----------|-----|
| `subquat()` | `tendon/spatial.rs:441` | §64: quaternion difference → axis-angle 3-vector (matches `mju_subQuat`) |
| `ball_limit_axis_angle()` | `constraint/impedance.rs:228` | Reference: quaternion → axis-angle for ball limits (similar math) |
| `normalize_quat4()` | `constraint/impedance.rs` | §64: quaternion normalization |
| `DISABLE_SPRING` | `types/enums.rs:630` | §64: spring force/energy gate |
| `DISABLE_LIMIT` | `types/enums.rs` | §64a: limit constraint gate (already used) |
| `has_spring` | `forward/passive.rs:776` | §64: spring force gate (already wired) |
| `finalize_row!` | `constraint/assembly.rs:203` | §64a: margin parameter (4th positional arg) |

#### Counting–assembly consistency invariant

The constraint assembly in `assembly.rs` has a critical invariant: the
counting phase (Phase 1, lines 58-170) must produce the **exact same**
number of rows as the assembly phase (Phase 3, lines 199+). If counting
uses `< margin` but assembly uses `< 0.0` (or vice versa), the row count
won't match allocation, causing a panic or buffer overflow. The spec must
enumerate all 6 activation checks and ensure counting and assembly use
identical conditions.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mj_energyPos()` (spring energy switch for FREE fallthrough + BALL), `mj_springdamper()`/`mj_passive()` (spring force for FREE translational + BALL rotational), `mju_subQuat()` (quaternion difference), `mji_quat2Vel()` (axis-angle with wrapping), `mj_instantiateLimit()` (margin activation + `mj_addConstraint` storage). Edge cases explicitly addressed: zero stiffness (exact `== 0` check in MuJoCo — NOT `> 0.0`; MuJoCo processes negative stiffness), identity quaternion (zero angle/force/energy), near-π rotation (wrapping), `mjDSBL_SPRING` / `mjDSBL_LIMIT` flags, sleep filter (MuJoCo's `mj_energyPos` applies sleep filter to spring/tendon section ONLY — gravity loop is unconditional; CortenForge's `energy.rs` currently lacks both `DISABLE_SPRING` gate and spring sleep filter), free joint fallthrough (`padr += 3` then BALL case), same stiffness for translational + rotational, `implicit_mode` gate for spring force in `passive.rs` (hinge/slide gated on `!implicit_mode`; ball/free must match). DOF vs qpos addressing difference documented (free: 6 DOF / 7 qpos; ball: 3 DOF / 4 qpos). Force path normalizes quaternion before `subQuat`; energy path normalizes but passes original (scale-invariant — results identical). C code snippets for non-obvious behavior (quaternion operations, fallthrough). Per EGT-1, spec explicitly states §60 is dropped because `springinertia` does not exist in MuJoCo. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics (e.g., "compute quaternion difference" without specifying `subQuat` → axis-angle → energy formula chain). |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or assumed from docs. |

**P1 boundary with P9 (Quaternion Geometric Correctness):** P1 grades
whether the spec *got the MuJoCo reference right* (correct function,
correct behavior). P9 grades whether the *mathematical derivation* in the
spec is rigorous (correct formula, correct equivalence proof).

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | §64 force: Rust pseudocode for `visit_multi_dof_joint` (or `visit_ball`/`visit_free` directly) ball/free spring force — **must branch on `ctx.jnt_type`** because free joints need 3 translational Euclidean DOFs (`-k * (pos - pos_spring)`) + 3 rotational quaternion DOFs (via `subquat`), while ball joints need only 3 rotational DOFs. Pseudocode covers: `qpos_spring` indexing (free: `qpos_adr` for pos, `qpos_adr+3` for quat; ball: `qpos_adr` for quat), `subquat` call, force assignment to `qfrc_spring[dof_adr..dof_adr+nv]`, `DISABLE_SPRING` gate, sleep check, **`implicit_mode` gate** (must match `visit_1dof_joint` pattern: spring force inside `if !self.implicit_mode`; note: `implicit_stiffness=0.0` for ball/free joints, so spring forces are fully suppressed in implicit mode — same as MuJoCo, which doesn't handle quaternion springs implicitly). §64 energy: Rust pseudocode for ball/free match arms in `mj_energy_pos` — including free translational (3D dot product), fallthrough to ball (axis-angle squared norm), `stiffness == 0` guard (exact zero check, not `> 0.0`). **Pre-existing conformance gaps in `energy.rs` addressed:** (1) missing `DISABLE_SPRING` gate on entire spring section, (2) missing sleep filter on spring section only (MuJoCo's gravity loop is unconditional — sleep filter applies AFTER gravity, to springs/tendons only), (3) `stiffness > 0.0` guard should be `stiffness == 0.0` continue (match MuJoCo). §64a margin: exact list of all 9 modification sites (6 activation checks + 3 `finalize_row!` args) with before/after code for each. An implementer can type it in without reading MuJoCo source. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses CortenForge conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present covering: (1) `UnitQuaternion<f64>` (nalgebra) vs raw `[f64; 4]` (MuJoCo) — `subquat()` in `tendon/spatial.rs` already bridges this (re-exported `pub(crate)` via `tendon/mod.rs`); (2) qpos indexing via `model.jnt_qpos_adr[jnt_id]` vs MuJoCo's `m->jnt_qposadr[j]`; (3) DOF indexing via `model.jnt_dof_adr[jnt_id]`; (4) `jnt_range` as `(f64, f64)` tuple vs MuJoCo's flat `jnt_range[2*i+0/1]`; (5) `qfrc_spring` as `DVector<f64>` indexed by dof vs MuJoCo's `d->qfrc_spring`; (6) sleep check via `body_sleep_state` + `SleepState::Asleep` vs MuJoCo's `body_awake` + `mjS_AWAKE`; (7) `finalize_row!` macro 4th arg = margin vs MuJoCo's `mj_addConstraint(..., &margin, ...)`. (8) **`qpos_spring` for ball/free joints:** builder copies `qpos0` (initial body pose/orientation) into `qpos_spring`, NOT from XML `springref` (which is a scalar and ignored for ball/free). Ball: 4-value quaternion `[w,x,y,z]` at `qpos_adr`. Free: 7-value pose `[x,y,z,w,qx,qy,qz]` at `qpos_adr`. (9) `JointContext` carries `jnt_type: MjJointType`, `qpos_adr`, `dof_adr`, `nv`, `nq` — implementer can branch on `ctx.jnt_type` within `visit_multi_dof_joint`. Each porting rule verified to preserve numerical equivalence. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete input (joint type, stiffness value, qpos configuration, margin value), (2) exact expected value or tolerance (e.g., "ball joint at 90° from reference with `stiffness=2.0`: energy = `0.5 * 2.0 * (π/2)² ≈ 2.467`; force magnitude = `2.0 * π/2 ≈ 3.14159`"), (3) field to check (`energy_potential`, `qfrc_spring[dof]`, `nefc`, `efc_margin[row]`). §64a regression: `margin=0` must produce identical `nefc` and `efc_*` values to current code (before/after comparison). Code-review ACs labeled as such. At least one AC per feature has expected values derived from MuJoCo's formulas with concrete numbers. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory covering: (1) zero stiffness → no force/energy, (2) identity quaternion reference → zero force/energy, (3) 90° rotation from reference, (4) near-180° rotation (wrapping boundary), (5) free joint: translational + rotational combined (verify translational uses Euclidean, rotational uses quaternion), (6) `DISABLE_SPRING` → zero spring force AND energy (tests both `passive.rs` and `energy.rs`; tests ALL joint types, not just ball/free), (7) `margin=0` regression (identical to current behavior), (8) `margin > 0` pre-activation zone, (9) ball joint with margin, (10) `DISABLE_LIMIT` → no limit constraints regardless of margin, (11) sleep filter → sleeping bodies excluded from spring energy but NOT gravitational energy, (12) `implicit_mode` → ball/free spring force skipped AND not compensated implicitly (total suppression — verify `qfrc_spring` is zero for ball/free in implicit mode), (13) free joint qpos_spring layout: translational reference at `qpos_adr..+3`, quaternion reference at `qpos_adr+3..+7`, (14) negative stiffness → force/energy IS computed (matching MuJoCo's `== 0` guard — validates the guard change from `> 0.0`). Negative tests: spring disabled → energy zero; limit disabled → margin irrelevant; implicit mode → no explicit spring force. At least one MuJoCo conformance test per feature (test comment states expected value source). At least one multi-joint model test (e.g., free + hinge chain) to catch indexing bugs. Supplementary tests justified. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | §64 prerequisite on Spec A's `qpos_spring` array explicitly stated with commit hash for already-landed work. §64a has no prerequisites. Execution order unambiguous: §64 force before §64 energy (energy reuses same quaternion math, but force changes `qfrc_spring` which energy doesn't read — so order is flexible). §64a independent of §64. Cross-spec interaction with Spec A's `qpos_spring` documented (§64 reads `qpos_spring`; Spec A populated it). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Every file touched, every behavior that changes, and every existing test
> that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (per EGT-6 table). Behavioral changes: (1) §64 — `energy_potential` changes for models with ball/free joint stiffness (moves *toward* MuJoCo conformance); `qfrc_spring` changes for same models (moves *toward* conformance); (2) **Pre-existing energy.rs fixes affect ALL joint types** — (a) `DISABLE_SPRING` gate: any model using `disableflags & DISABLE_SPRING` will now see `energy_potential` drop to gravitational-only (currently spring energy is always computed even when disabled), (b) sleep filter: sleeping bodies' springs will no longer contribute to `energy_potential` (currently all springs always contribute), (c) `stiffness == 0` guard: no behavioral change in practice (negative stiffness is not physically meaningful); (3) §64a `margin=0` — no behavioral change (same activation condition); `margin > 0` — new behavior (earlier activation). Existing test impact: names specific tests that exercise energy/passive/assembly paths. States whether breakage is expected. If existing test values change (e.g., total energy for a model with a ball joint spring), the new values are stated and verified as more conformant. Data staleness: no `EXPECTED_SIZE` constants affected. Backward-compat: `margin=0` default ensures all existing models unchanged. Domain test suites: `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-conformance-tests`. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "margin" (not "jnt_margin" for the MJCF attribute, "jnt_margin" for the model field), "spring energy" / "spring force" (not interchanged), "quaternion geodesic distance" (used consistently for the energy formula). File paths in Specification sections match Files Affected. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. All 9 assembly.rs modification sites enumerated consistently between P2 algorithm and P7 blast radius. Consumer counts match between sections. |
| **A** | Consistent. One or two minor inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Quaternion Geometric Correctness *(domain-specific)*

> The quaternion difference → axis-angle → energy/force derivation is
> mathematically rigorous and produces numerically identical results to
> MuJoCo's `mju_subQuat` → `mji_quat2Vel` chain.

| Grade | Bar |
|-------|-----|
| **A+** | Complete derivation: `subQuat(qa, qb)` = `quat2Vel(conj(qb) * qa, dt=1)` = axis-angle vector where `\|\|vec\|\| = θ` (rotation angle). Energy: `0.5 * k * θ²` = `0.5 * k * dot(dif, dif)`. Force: `-k * dif` (3D, DOF-indexed). Equivalence with CortenForge's `subquat()` in `tendon/spatial.rs` proven or referenced — **argument order explicitly verified:** `subquat(current, reference)` matches MuJoCo's `mju_subQuat(res, current, reference)` convention (both compute `conj(reference) * current`); an argument swap produces wrong-sign forces. Wrapping behavior at `θ > π` explained (handles quaternion double-cover). Edge case: identity → zero vector; near-π → wrapping gives `θ ∈ (-π, π]`; exact-π → axis undefined but `\|\|dif\|\| = π`. Quaternion normalization strategy documented: CortenForge's `subquat()` requires `UnitQuaternion` arguments, so BOTH force and energy paths must normalize when constructing from raw `qpos` f64 values (use `UnitQuaternion::new_normalize()`). MuJoCo's force path normalizes before `mji_subQuat`; MuJoCo's energy path has dead normalization code (`mju_normalize4(quat)` result is unused — `d->qpos+padr` passed instead). This discrepancy is conformance-neutral because `quat2Vel`'s `atan2(sin_half, w)` is scale-invariant — both paths produce identical axis-angle results regardless of quaternion magnitude. The relationship between `qpos` indexing (4 values for ball) and `dof` indexing (3 values for ball) is explicit in the force assignment. Free joint: `padr` advances by 3 (past position) to reach quaternion, `dadr` advances by 3 (past translational DOFs) to reach rotational DOFs. |
| **A** | Derivation correct. Minor gaps in edge-case math. |
| **B** | Formula stated but not derived from `subQuat`. |
| **C** | Formula wrong or uses incorrect quaternion operation. |

**P9 boundary with P1:** P1 grades whether the spec identified the
*correct* MuJoCo functions and behavior. P9 grades whether the
*mathematical derivation* mapping those functions to Rust is rigorous.

### P10. Call-Site Completeness *(domain-specific)*

> For §64a, every activation check and `finalize_row!` margin argument in
> `assembly.rs` is enumerated and modified consistently. Missing even one
> site is a correctness bug (counting–assembly mismatch → panic).

| Grade | Bar |
|-------|-----|
| **A+** | Exhaustive enumeration of all 9 modification sites in `assembly.rs` (6 activation checks + 3 `finalize_row!` margin args), with file:line reference for each. Counting sites (3) and assembly sites (3) explicitly paired — each counting check has its corresponding assembly check identified. The counting–assembly consistency invariant is stated: both phases must use identical activation conditions or the allocated row count won't match assembly output. `finalize_row!` margin arg identified for each of the 3 joint-limit call sites (lower hinge/slide, upper hinge/slide, ball). No tendon-limit sites modified (tendon margins are a separate feature). The spec states that `margin` is read once per joint (`model.jnt_margin[jnt_id]`) and used for all checks on that joint (both limits for hinge/slide, one limit for ball). |
| **A** | All sites enumerated. Counting–assembly pairing left implicit. |
| **B** | Most sites listed but 1-2 missing or inconsistent. |
| **C** | Sites not enumerated — "replace all hardcoded `< 0.0` checks." |

**P10 boundary with P2:** P2 grades algorithmic completeness (the
formulas and logic). P10 grades exhaustive enumeration of *where* those
formulas apply — every call site, every consumer, no silent skips.

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions, line numbers,
      and edge cases. P1 names 5 MuJoCo functions and 10+ edge cases
      (including pre-existing gaps). P5 names 14 edge cases. P10 names
      9 specific modification sites. Two independent reviewers would
      assign the same grade.

- [x] **Non-overlap:** P1 vs P9 boundary explained (MuJoCo reference vs
      mathematical derivation). P2 vs P10 boundary explained (algorithm
      logic vs call-site enumeration). P9 vs P10 don't overlap (P9 is
      quaternion math correctness; P10 is margin call-site completeness).

- [x] **Completeness:** 10 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6),
      blast radius (P7), consistency (P8), quaternion math (P9), call-site
      enumeration (P10). Stress test Rev 2: `implicit_mode` gate (R5),
      `DISABLE_SPRING` pre-existing gap (R6), sleep filter gap (R7),
      `subquat` argument order (R8), stiffness guard (R9). Stress test
      Rev 3: joint-type branching in P2 (R10), pre-existing fix blast
      radius in P7 (R11), sleep filter scope in EGT-7/P1 (R12),
      `qpos_spring` ball/free convention in P3 (R13), `implicit_mode`
      suppression consequence in EGT-8 (R14). Stress test Rev 4:
      EGT-6 energy.rs change description (R15), P9 energy-path
      normalization (R16) — all captured. Also verified correct:
      `quat2Vel` wrapping, `subquat` argument order, `qpos_spring`
      normalization guarantees, EGT-2 C code accuracy. Stress test
      Rev 5 (convergence): self-audit edge case count (R17), negative
      stiffness edge case (R18). Two cosmetic observations noted but
      not fixed (EGT section order, P9 "undefined" vs "ambiguous" at π).

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors. P2 →
      Specification sections. P3 → Convention Notes. P4 → Acceptance
      Criteria. P5 → Test Plan + Traceability. P6 → Prerequisites +
      Execution Order. P7 → Risk & Blast Radius. P8 → cross-cutting. P9 →
      MuJoCo Reference quaternion derivation + Specification quaternion
      code. P10 → Specification §64a modification site list.

- [x] **Conformance primacy:** P1 is tailored with 5 specific MuJoCo C
      functions, 10+ edge cases, and file references. P4 requires MuJoCo-
      derived expected values. P5 requires conformance tests. P9 requires
      `subQuat` equivalence proof with argument order verification. The
      rubric cannot produce an A+ spec that diverges from MuJoCo.

- [x] **Empirical grounding:** EGT-1 through EGT-8 verified against MuJoCo
      C source and CortenForge codebase. Every A+ bar that references
      MuJoCo behavior has a corresponding EGT entry. §60 dropped based on
      EGT-1 (does not exist). Pre-existing gaps in energy.rs documented
      in EGT-7. Implicit mode gate documented in EGT-8. No criterion bar
      written from assumptions.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1: §64 force, S2: §64 energy, S3: §64a margin) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | MuJoCo Reference (quaternion derivation), Specification (§64 quaternion code) |
| P10 | Specification (§64a call-site enumeration), Risk & Blast Radius (assembly.rs sites) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Quaternion Geometric Correctness | | |
| P10. Call-Site Completeness | | |

**Overall: (Rev 5)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | §60 (`springinertia`) listed in umbrella but does not exist in MuJoCo | EGT-1 (GitHub search, mjmodel.h, mj_crb source, XML reference) | Dropped from Spec B scope | Rubric Rev 1 |
| R2 | Scope | §64 narrowed to energy only in umbrella, but force is also missing and uses identical math | EGT-4 (passive.rs:818 has no spring force for ball/free) | Expanded to include both force and energy | Rubric Rev 1 |
| R3 | P10 | Tendon limit `< 0.0` checks (assembly.rs:148, 152, 540, 563) could be confused with joint limit checks | Codebase research (EGT-6) | P10 A+ bar explicitly states "No tendon-limit sites modified" | Rubric Rev 1 |
| R4 | P2 | Counting–assembly consistency invariant not captured in algorithm criteria | Codebase research (EGT-6, counting–assembly note) | Added to P10 as explicit requirement | Rubric Rev 1 |
| R5 | P2 | `implicit_mode` gate missing — hinge/slide spring force gated on `!implicit_mode` (passive.rs:800); ball/free spring force must match but P2 bar didn't mention it | Stress test (passive.rs:800 vs :829 pattern) | Added `implicit_mode` to P2 A+ bar and P5 edge case inventory | Rubric Rev 2 |
| R6 | P1/P2 | `DISABLE_SPRING` pre-existing gap in `energy.rs` — MuJoCo gates entire spring energy section; CortenForge has no such gate. Rubric mentioned the flag but not the existing omission | Stress test (energy.rs:38 vs MuJoCo `!mjDISABLED(mjDSBL_SPRING)`) | Added EGT-7, added to P1 and P2 A+ bars, added to P5 edge case inventory | Rubric Rev 2 |
| R7 | P1/P2 | Sleep filter pre-existing gap in `energy.rs` — MuJoCo iterates bodies with sleep check; CortenForge iterates joints without | Stress test (energy.rs:38 body-vs-joint loop) | Added EGT-7, added to P1 A+ bar, added to P5 edge case inventory | Rubric Rev 2 |
| R8 | P9 | `subquat` argument order not explicitly verified — swapped args produce wrong-sign forces | Stress test (subquat convention analysis) | Added argument order verification to P9 A+ bar | Rubric Rev 2 |
| R9 | P1/P2 | `stiffness > 0.0` guard in energy.rs differs from MuJoCo's `== 0` — negative stiffness handled differently | Stress test (energy.rs:40 vs MuJoCo) | Added to P1 A+ bar (explicit "NOT `> 0.0`"), added to P2 A+ bar, added EGT-7 | Rubric Rev 2 |
| R10 | P2 | `visit_multi_dof_joint` pseudocode must branch on `ctx.jnt_type` — free joints need 3 translational Euclidean DOFs + 3 rotational quaternion DOFs, ball needs only 3 rotational. Without branching, free joint translational spring would be computed with quaternion math (wrong). `JointContext` carries `jnt_type: MjJointType` so branching is possible. | Stress test 2 (JointContext struct analysis, visitor dispatch pattern) | Added explicit branching requirement to P2 A+ bar, added `JointContext` fields to P3 | Rubric Rev 3 |
| R11 | P7 | Pre-existing energy.rs fixes (DISABLE_SPRING, sleep filter, stiffness guard) affect ALL joint types (hinge/slide/ball/free), not just ball/free. P7 said "models with ball/free joint stiffness" — underspecified. | Stress test 2 (EGT-7 scope analysis) | P7 A+ bar expanded to enumerate behavioral changes for all joint types | Rubric Rev 3 |
| R12 | P1/EGT-7 | Sleep filter in MuJoCo's `mj_energyPos()` applies ONLY to spring/tendon section. Gravity loop is unconditional. "Missing sleep filter" without scope qualifier could cause implementer to add sleep filter to gravity section (non-conformant). | Stress test 2 (MuJoCo `engine_sensor.c` source verification) | EGT-7 updated with two-phase structure. P1/P2 bars clarified "spring sleep filter". | Rubric Rev 3 |
| R13 | P3 | `qpos_spring` for ball/free joints is populated from `qpos0` (initial body pose), NOT from XML `springref` (scalar, ignored for ball/free). This convention was missing from P3 table. | Stress test 2 (builder/joint.rs:186/195 analysis) | Added as convention (8) in P3 A+ bar | Rubric Rev 3 |
| R14 | EGT-8 | `implicit_mode` gate + `implicit_stiffness=0.0` for ball/free (model_init.rs) = total suppression of ball/free spring forces in `ImplicitSpringDamper` mode. This is correct MuJoCo behavior (quaternion springs too nonlinear for diagonal implicit approximation) but was undocumented. | Stress test 2 (model_init.rs `compute_implicit_params()` analysis) | Added consequence note to EGT-8. P2 bar updated to document as expected behavior. | Rubric Rev 3 |
| R15 | P8/EGT-6 | EGT-6 files table said energy.rs change is "§64: implement ball/free energy" but omitted pre-existing fixes (DISABLE_SPRING gate, sleep filter, stiffness guard) that P2/P7 require. Internal consistency violation — P7 says these changes happen but EGT-6 didn't list them. | Stress test 3 (P8 cross-section consistency check) | EGT-6 energy.rs entry updated to include pre-existing fix scope for ALL joint types | Rubric Rev 4 |
| R16 | P9 | P9 said "force path should normalize before `subquat`" but didn't address energy path. CortenForge's `subquat()` requires `UnitQuaternion` type, so BOTH paths must normalize (type-system requirement). MuJoCo's energy path has dead normalization code (`mju_normalize4(quat)` result unused — passes raw `d->qpos`). Conformance-neutral (`quat2Vel`'s `atan2` is scale-invariant). | Stress test 3 (MuJoCo `engine_sensor.c` source verification of exact arguments to `mju_subQuat`) | P9 A+ bar updated with energy-path normalization note and scale-invariance explanation | Rubric Rev 4 |
| R17 | Self-audit | Self-audit said "P5 names 12 edge cases" but P5 now has 14 (after Rev 3 added (13) and Rev 5 added (14)). Stale count. | Stress test 4 (convergence — word-by-word cross-reference check) | Updated self-audit count to 14 | Rubric Rev 5 |
| R18 | P5 | Missing negative stiffness edge case. P2/EGT-7 require changing stiffness guard from `> 0.0` to `== 0`, but P5 only tested boundary (zero stiffness). Negative stiffness → force/energy IS computed (matching MuJoCo) was not in the edge case inventory. | Stress test 4 (convergence — stiffness guard boundary analysis) | Added edge case (14) to P5 A+ bar | Rubric Rev 5 |
