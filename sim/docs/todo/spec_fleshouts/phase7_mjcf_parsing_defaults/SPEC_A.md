# Spec A — Defaults Completeness: Spec

**Status:** Draft
**Phase:** Roadmap Phase 7 — MJCF Parsing & Defaults Gaps
**Effort:** M
**MuJoCo ref:** `OneEquality()` in `xml_native_reader.cc`, `CopyTree()` in
`user_model.cc`, `mj_springdamper()` in `engine_passive.c`, `Default()` and
`OneActuator()` in `xml_native_reader.cc`
**MuJoCo version:** 3.x (main branch, 2026-03)
**Test baseline:** 1,900+ sim domain tests (post-Phase 6)
**Prerequisites:**
- Phase 6 complete (commit `cf76731`)
- Phase 7 Sessions 1–3 complete (umbrella, T1, rubric)

**Independence:** This spec is independent of Spec B and Spec C per the
umbrella dependency graph. Shared files: `parser.rs`, `types.rs`, `model.rs`,
`model_init.rs` — Spec A touches different structs/sections than Spec B and
Spec C. The one overlap (`margin` on joints) is owned by Spec B per umbrella
§ File Ownership Matrix.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| DT-2: No `MjcfEqualityDefaults` exists | Confirmed — no struct, no parse, no cascade, 5 hardcoded fallback sites in `builder/equality.rs` | **In scope** — full implementation needed |
| DT-11: Joint `range` not defaultable | **Already implemented** — `range` field on `MjcfJointDefaults` (types.rs:606), parsed in `parse_joint_defaults()` (parser.rs:611), cascaded in `apply_to_joint()` (defaults.rs:169), applied via resolver in `builder/joint.rs:265` | **Drop** — no implementation needed; verify in review |
| DT-13: `qpos_spring` not implemented | Confirmed — no `qpos_spring` array on Model; `jnt_springref` (per-joint scalar) used directly; cannot represent quaternion reference for ball/free | **In scope** — full implementation needed |
| DT-14: Actuator type-specific defaults missing | Confirmed — parser match (parser.rs:522) handles `actuator|motor|position|velocity|general` but **misses** `cylinder|muscle|adhesion|damper|intvelocity`; `MjcfActuatorDefaults` lacks cylinder/muscle/adhesion-specific fields | **In scope** — parser + struct + cascade additions needed |

**Final scope:**

1. **DT-2:** Create `MjcfEqualityDefaults`, parse `<default><equality>`,
   cascade `active`/`solref`/`solimp` to all 5 equality constraint types
2. **DT-13:** Add `qpos_spring: Vec<f64>` to Model (size `nq`), initialize
   from `qpos0` for ball/free and from `springref` for hinge/slide, update
   consumers (`passive.rs`, `energy.rs`, `build.rs` tendon sentinel)
3. **DT-14:** Add missing shortcut names to `parse_default()` match,
   add cylinder/muscle/adhesion-specific fields to `MjcfActuatorDefaults`,
   extend cascade

DT-11 is dropped from the spec (already implemented). The review phase
(Session 7) will verify it passes.

---

## Problem Statement

Three distinct conformance gaps in the defaults and model data pipeline:

**DT-2 — Equality constraint defaults:** MuJoCo supports `<default><equality
active="..." solref="..." solimp="..."/>` which applies uniformly to all 5
equality constraint types (connect, weld, joint, tendon, distance).
CortenForge has no `MjcfEqualityDefaults` struct, no equality field on
`MjcfDefault`, no parsing of `<equality>` inside `<default>`, and all 5
equality builder sites use hardcoded `DEFAULT_SOLREF`/`DEFAULT_SOLIMP`
fallbacks. Any MJCF model that sets equality solver parameters via default
classes will get the wrong solver behavior.

**DT-13 — `qpos_spring` array:** MuJoCo stores a full `qpos_spring` array
(size `nq`) alongside `qpos0`. For hinge/slide joints, `qpos_spring` holds
the scalar `springref` value. For ball/free joints, `qpos_spring` holds the
reference quaternion/pose copied from `qpos0`. This array is used at runtime
by `mj_springdamper()` (spring forces) and at compile time by `setSpring()`
(tendon sentinel resolution). CortenForge uses `jnt_springref` (a per-joint
scalar indexed by `jnt_id`) which cannot represent quaternion references for
ball/free joints. The tendon sentinel code at `build.rs:491` uses `qpos0`
instead of `qpos_spring`, producing divergent tendon spring lengths when
`springref != 0`.

**DT-14 — Actuator type-specific defaults:** MuJoCo's `Default()` parser
dispatches `<cylinder>`, `<muscle>`, `<adhesion>`, `<damper>`,
`<intvelocity>` shortcut elements inside `<default>` to `OneActuator()`,
which reads type-specific attributes and stores them on the single
`mjsActuator` default struct. CortenForge's `parse_default()` only handles
`b"actuator"|b"motor"|b"position"|b"velocity"|b"general"` — the other 5
shortcut names silently fall through to `_ => skip_element()`. Additionally,
`MjcfActuatorDefaults` lacks fields for `area`, `diameter`, `bias`,
`timeconst`, muscle params, and `gain`.

---

## MuJoCo Reference

### DT-2: Equality Defaults

#### `mjs_defaultEquality()` in `user_init.c` (~line 259)

```c
void mjs_defaultEquality(mjsEquality* equality) {
  memset(equality, 0, sizeof(mjsEquality));
  equality->type = mjEQ_CONNECT;
  equality->active = 1;
  mj_defaultSolRefImp(equality->solref, equality->solimp);
  equality->data[1] = 1;
  equality->data[10] = 1;  // torque:force ratio
}
```

#### `mj_defaultSolRefImp()` in `engine_io.c`

```c
void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp) {
  if (solref) {
    solref[0] = 0.02;   // timeconst
    solref[1] = 1;       // dampratio
  }
  if (solimp) {
    solimp[0] = 0.9;    // dmin
    solimp[1] = 0.95;   // dmax
    solimp[2] = 0.001;  // width
    solimp[3] = 0.5;    // midpoint
    solimp[4] = 2;       // power
  }
}
```

#### `OneEquality()` in `xml_native_reader.cc` (~line 5083)

Two parsing contexts controlled by the `readingdefaults` flag:

1. **Inside `if (!readingdefaults)`** — parses `name`, object references
   (body1/body2, site1/site2), type-specific data (anchor, relpose,
   polycoef, torquescale). **Skipped** when parsing `<default><equality>`.

2. **After the if-block (unconditional)** — parses `active`, `solref`,
   `solimp`. **Always parsed**, including in defaults context.

```c
// ---- applies in BOTH regular and default contexts ----
if (MapValue(elem, "active", &n, bool_map, 2)) {
    equality->active = (n == 1);
}
ReadAttr(elem, "solref", mjNREF, equality->solref, text, false, false);
ReadAttr(elem, "solimp", mjNIMP, equality->solimp, text, false, false);
```

**Key constraint:** `<default><equality>` can set only 3 attributes:
`active` (bool), `solref` (`[f64; 2]`), `solimp` (`[f64; 5]`).

#### Defaults cascade mechanism

1. `mjs_defaultEquality()` — built-in defaults (zeroed struct + active=1 +
   solref/solimp from `mj_defaultSolRefImp`)
2. `Default()` function — creates `mjCDef` for each `<default class="X">`,
   calls `OneEquality()` with `readingdefaults=true`
3. Concrete element creation — `mjCEquality(model, def)` constructor applies
   defaults from class, then `OneEquality()` with `readingdefaults=false`
   parses element-specific attributes (which override class defaults)
4. Nested defaults inherit: child class starts with parent class values

**The `<default>` element name is `equality`** (not per-type like `connect`
or `weld`). One equality default per class applies to ALL 5 constraint types.

#### `bool_map` in `xml_util.cc`

MuJoCo's `MapValue()` uses `FindKey()` with a `bool_map` that only accepts
`"true"`/`"false"`. Passing `"0"` or `"1"` produces an XML parse error. The
current CortenForge parser uses `!= "false"` for `active` on equality
structs (parser.rs:2331/2382/2423/2464/2505), while 27 other boolean
attributes use `== "true"`. When touching these sites for the `Option<bool>`
migration, standardize to `== "true"`.

### DT-13: `qpos_spring`

#### `CopyTree()` in `user_model.cc` (~line 3465)

Initializes `qpos_spring` per joint type during model compilation:

```c
switch (pj->type) {
  case mjJNT_FREE:
    mjuu_copyvec(m->qpos0+qposadr, pb->pos, 3);
    mjuu_copyvec(m->qpos0+qposadr+3, pb->quat, 4);
    mjuu_copyvec(m->qpos_spring+qposadr, m->qpos0+qposadr, 7);
    break;

  case mjJNT_BALL:
    m->qpos0[qposadr] = 1;
    m->qpos0[qposadr+1] = 0;
    m->qpos0[qposadr+2] = 0;
    m->qpos0[qposadr+3] = 0;
    mjuu_copyvec(m->qpos_spring+qposadr, m->qpos0+qposadr, 4);
    break;

  case mjJNT_SLIDE:
  case mjJNT_HINGE:
    m->qpos0[qposadr] = (mjtNum)pj->ref;
    m->qpos_spring[qposadr] = (mjtNum)pj->springref;
    break;

  default:
    throw mjCError(pj, "unknown joint type");
}
```

**Summary:**

| Joint type | `qpos_spring` content | Source |
|------------|----------------------|--------|
| HINGE | 1 scalar: `springref` value | `pj->springref` (MJCF `springref` attribute) |
| SLIDE | 1 scalar: `springref` value | `pj->springref` |
| BALL | 4 values: identity quat `[1,0,0,0]` | Copied from `qpos0` |
| FREE | 7 values: body pos (3) + body quat (4) | Copied from `qpos0` |

**Ball/Free ignore the `springref` MJCF attribute** — their spring reference
is always the initial configuration copied from `qpos0`.

#### `mj_springdamper()` in `engine_passive.c` (~line 118)

Uses `qpos_spring` for spring force computation:

```c
switch ((mjtJoint) m->jnt_type[j]) {
case mjJNT_FREE:
  // translational spring: F = -k * (qpos - qpos_spring)
  d->qfrc_spring[dadr+0] = -stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
  d->qfrc_spring[dadr+1] = -stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
  d->qfrc_spring[dadr+2] = -stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);
  dadr += 3;
  padr += 3;
  mjFALLTHROUGH;

case mjJNT_BALL:
  {
    mjtNum dif[3], quat[4];
    mji_copy4(quat, d->qpos+padr);
    mju_normalize4(quat);
    mji_subQuat(dif, quat, m->qpos_spring + padr);
    d->qfrc_spring[dadr+0] = -stiffness*dif[0];
    d->qfrc_spring[dadr+1] = -stiffness*dif[1];
    d->qfrc_spring[dadr+2] = -stiffness*dif[2];
  }
  break;

case mjJNT_SLIDE:
case mjJNT_HINGE:
  d->qfrc_spring[dadr] = -stiffness*(d->qpos[padr] - m->qpos_spring[padr]);
  break;
}
```

**Note:** The FREE case falls through to BALL for rotational DOFs. The
`padr` (qpos address) and `dadr` (dof address) differ for ball (`nq=4,
nv=3`) and free (`nq=7, nv=6`) joints.

#### `setSpring()` in `engine_setconst.c` (~line 1231)

Uses `qpos_spring` for tendon sentinel resolution:

```c
static void setSpring(mjModel* m, mjData* d) {
  mju_copy(d->qpos, m->qpos_spring, m->nq);
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_tendon(m, d);
  mj_transmission(m, d);

  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_lengthspring[2*i] == -1 && m->tendon_lengthspring[2*i+1] == -1) {
      m->tendon_lengthspring[2*i] = m->tendon_lengthspring[2*i+1] = d->ten_length[i];
    }
  }
}
```

CortenForge currently uses `qpos0` (build.rs:486) instead of `qpos_spring`
for tendon sentinel resolution. When `springref != 0` for a joint that a
tendon wraps, this produces a different spring rest length.

#### `mjCJoint::Compile()` in `user_objects.cc` (~line 2771)

Hinge joints convert `springref` from degrees to radians when the compiler
uses degrees:

```c
if (type == mjJNT_HINGE && compiler->degree) {
  ref *= mjPI/180.0;
  springref *= mjPI/180.0;
}
```

CortenForge already handles this conversion in `builder/joint.rs:118–123`.

### DT-14: Actuator Type-Specific Defaults

#### `Default()` in `xml_native_reader.cc` (~line 2884)

The `Default()` function dispatches all actuator shortcut names to
`OneActuator()`:

```
"general", "motor", "position", "velocity", "damper",
"intvelocity", "cylinder", "muscle", "adhesion"
```

All dispatch to the **same** `OneActuator(elem, def->actuator)` function.
MuJoCo stores a **single** `mjsActuator*` per default class — there is no
per-type segregation in the defaults struct.

If `<default>` contains both `<motor>` and `<position>`, the last one
**overwrites** the first (full struct replacement at parse level). At the
**merge level** (class inheritance), merge is per-field via child-or-parent.

#### `OneActuator()` in `xml_native_reader.cc` (~line 4035)

When parsing shortcut elements, `OneActuator()`:
1. Reads common attributes (name, class, gear, ctrlrange, etc.)
2. Reads shortcut-specific attributes as local variables
3. Calls `mjs_setTo*()` to expand the shortcut into gainprm/biasprm/dynprm

**Critical MuJoCo design:** Shortcut attributes (`area`, `diameter`, `bias`,
`timeconst`, `gain`, muscle params) are **local variables** in
`OneActuator()` that read from/write to `gainprm[]`/`biasprm[]`/`dynprm[]`.
They are not real fields on `mjsActuator`.

CortenForge stores these as real fields on `MjcfActuator` (for ergonomic
parsing). This is a valid design choice — the spec must add matching fields
to `MjcfActuatorDefaults` and cascade them correctly.

#### `mjs_setTo*()` Expansion Functions in `user_api.cc` (~lines 780–920)

| Function | gaintype | biastype | dyntype | gainprm | biasprm | dynprm |
|----------|----------|----------|---------|---------|---------|--------|
| `mjs_setToMotor` | FIXED | NONE | NONE | `[1,0,..]` | `[0,..]` | `[1,0,0,..]` |
| `mjs_setToPosition` | FIXED | AFFINE | NONE* | `[kp,0,..]` | `[0,-kp,-kv,..]` | `[tc,0,..]` |
| `mjs_setToVelocity` | FIXED | AFFINE | NONE | `[kv,0,..]` | `[0,0,-kv,..]` | `[1,0,0,..]` |
| `mjs_setToDamper` | AFFINE | NONE | NONE | `[0,0,-kv,..]` | `[0,..]` | `[1,0,0,..]` |
| `mjs_setToIntVelocity` | FIXED | AFFINE | INTEGRATOR | `[kv,0,..]` | `[0,0,-kv,..]` | `[1,0,0,..]` |
| `mjs_setToCylinder` | FIXED | AFFINE | FILTER | `[area,0,..]` | `bias[0..3]` | `[tc,0,..]` |
| `mjs_setToMuscle` | MUSCLE | MUSCLE | MUSCLE | `range+force+scale+lmin+..` | `=gainprm` | `[tc_act,tc_deact,..]` |
| `mjs_setToAdhesion` | FIXED | NONE | NONE | `[gain,0,..]` | `[0,..]` | `[1,0,0,..]` |

*Position dyntype is NONE by default; if `timeconst > 0`, it becomes
FILTEREXACT.

**Muscle sentinel detection** in `mjs_setToMuscle()`: Uses equality checks
against global default values as sentinels:
- `if (actuator->gainprm[0] == 1) actuator->gainprm[0] = 0.75;`
- `if (actuator->dynprm[0] == 1) actuator->dynprm[0] = 0.01;`

This means explicitly setting `gainprm[0]=1` is indistinguishable from "not
set" and gets overwritten by muscle defaults. This only manifests for
`<general dyntype="muscle" gainprm="1">`, not for `<muscle>` (which builds
from ergonomic fields like `range.0`, not from `gainprm`).

**Scope clarification:** CortenForge defers shortcut expansion to build time
(builder/actuator.rs) rather than expanding at parse time like MuJoCo. For
defaults, this means: when `<default><cylinder area="0.01"/>` is parsed,
CortenForge stores `area=0.01` on `MjcfActuatorDefaults` (new field). When a
concrete `<cylinder class="...">` inherits the default, the cascade applies
`area=0.01`. The builder then expands `area` into `gainprm[0]`. This
deferred expansion produces identical results because the shortcut expansion
in `builder/actuator.rs` (lines 374–392) already correctly maps area →
gainprm[0].

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Equality defaults cascade | `<default><equality solref="...">` applies to all 5 types via `OneEquality()` with `readingdefaults=true` | **Not implemented** — 5 hardcoded fallback sites, no defaults struct, class field ignored |
| `active` in equality defaults | Parsed as `bool` via `MapValue()+bool_map`, inheritable via class | `active: bool` (not `Option<bool>`) — cannot distinguish "not set" from "default true" |
| `qpos_spring` for hinge/slide | `m->qpos_spring[padr] = springref` | Uses `jnt_springref[jnt_id]` — numerically equivalent for hinge/slide but different index space |
| `qpos_spring` for ball | `[1,0,0,0]` copied from `qpos0` | **Not implemented** — no `qpos_spring` array exists |
| `qpos_spring` for free | `pos[3]+quat[4]` copied from `qpos0` | **Not implemented** |
| Tendon sentinel resolution | Evaluates at `qpos_spring` configuration | Evaluates at `qpos0` — **divergent** when `springref != 0` |
| `implicit_springref` init | Reads from `qpos_spring` per-DOF | Reads from `jnt_springref` per-joint |
| Actuator shortcut defaults | `cylinder|muscle|adhesion|damper|intvelocity` → `OneActuator()` | Missing from `parse_default()` match — silently dropped |
| Type-specific default fields | Stored on single `mjsActuator` per class | `MjcfActuatorDefaults` lacks `area`, `diameter`, `bias`, `timeconst`, muscle params, `gain` |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `qpos_spring` index space | `m->qpos_spring[padr]` where `padr = jnt_qposadr[j]` | `model.qpos_spring[qpos_adr]` where `qpos_adr = jnt_qpos_adr[jnt_id]` | Direct port — same index space. Use `jnt_qpos_adr` (not `jnt_dof_adr`) |
| `jnt_springref` vs `qpos_spring` | No `jnt_springref` on `mjModel` — only `qpos_spring` | Keep `jnt_springref` as builder intermediate; add `qpos_spring` on Model | Runtime code reads `qpos_spring`; `jnt_springref` stays for builder use only |
| `solref` size | `[mjNREF]` = `[2]` | `[f64; 2]` | Direct port |
| `solimp` size | `[mjNIMP]` = `[5]` | `[f64; 5]` | Direct port |
| Equality default struct | Single `mjsEquality` per class, applies to all 5 types | Single `MjcfEqualityDefaults` per class — same pattern | Direct port |
| Actuator default struct | Single `mjsActuator` per class, all shortcuts write to same struct | Single `MjcfActuatorDefaults` per class — same pattern | Direct port |
| Shortcut expansion timing | Parse time (`mjs_setTo*()` called in `OneActuator()`) | Build time (`builder/actuator.rs` switch) | Deferred expansion produces identical results because same mapping logic |
| `springref` default | `0.0` (from `memset` zero in `mjs_defaultJoint()`; NOT `NaN` or sentinel) | `Option<f64>` resolved via `unwrap_or(0.0)` | Direct port — `0.0` is the default when no `springref` attribute specified |
| `bool_map` | Only `"true"`/`"false"` — `"0"`/`"1"` produces parse error | Mix of `== "true"` and `!= "false"` patterns | Standardize to `== "true"` when touching equality active sites |

---

## Architecture Decisions

### AD-1: `apply_to_equality()` — single method for all 5 types

**Problem:** MuJoCo uses one `<equality>` default per class (not per-type
like `<connect>` or `<weld>`). Should we have one `apply_to_equality()` that
works on any of the 5 equality types, or 5 separate apply methods?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Single generic `apply_to_equality()` taking `(class, &mut active, &mut solref, &mut solimp)` | Matches MuJoCo's single-default pattern; DRY | Requires extracting fields from each equality type |
| 2 | Five per-type methods (`apply_to_connect()`, `apply_to_weld()`, etc.) | Type-safe per variant | 5× boilerplate; all identical except field access |

**Chosen:** Option 1 — Single generic method. All 5 types share the same 3
defaultable fields (`active`, `solref`, `solimp`). The caller extracts these
fields, passes them as mutable references, and the cascade applies uniformly.

### AD-2: `jnt_springref` co-existence with `qpos_spring`

**Problem:** Adding `qpos_spring` to Model means two overlapping data paths
for spring reference. Should `jnt_springref` be removed?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Keep `jnt_springref` as builder-intermediate, add `qpos_spring` on Model | No breaking changes to ~33 consumer sites outside runtime | Two representations of spring ref in the system |
| 2 | Remove `jnt_springref` entirely, use only `qpos_spring` | Single source of truth | ~33 consumer sites in tests/factories must change; `implicit_springref` logic gets complex |

**Chosen:** Option 1 — Keep `jnt_springref` as builder intermediate. The
runtime code (`passive.rs`, `energy.rs`) migrates to `qpos_spring`. The
builder uses `jnt_springref` internally during joint processing to accumulate
per-joint springref values, then expands them into `qpos_spring` at
finalization. The 18 test model factory sites that push `jnt_springref` also
need to push `qpos_spring` entries — but this is mechanical and confined to
test setup.

### AD-3: Actuator defaults cascade — deferred expansion invariant

**Problem:** MuJoCo expands shortcuts at parse time (in defaults context).
CortenForge defers expansion to build time. Does this cause a conformance
issue for defaults?

**Analysis:** No. When `<default><cylinder area="0.01"/>` is parsed:
- MuJoCo: `OneActuator()` → `mjs_setToCylinder()` → `gainprm[0]=0.01`
  stored on the default struct
- CortenForge: `parse_actuator_defaults()` → `area=Some(0.01)` stored on
  `MjcfActuatorDefaults`

When a concrete `<cylinder class="X">` is processed:
- MuJoCo: Element starts from default struct's `gainprm[0]=0.01`, then
  attribute overrides
- CortenForge: `apply_to_actuator()` applies `area=0.01` from defaults to
  element's `area` field, then builder expands `area` → `gainprm[0]`

The end result is identical: `gainprm[0]=0.01` in the compiled model.

**Chosen:** Deferred expansion is conformance-safe. Add type-specific fields
to `MjcfActuatorDefaults` and cascade them in `apply_to_actuator()`.

---

## Specification

### S1. DT-2 — Equality Defaults: Struct and Parse

**Depends on:** None (independent)
**File:** `sim/L0/mjcf/src/types.rs` (line 556) and `sim/L0/mjcf/src/parser.rs`
(lines 515–558)
**MuJoCo equivalent:** `mjs_defaultEquality()` in `user_init.c`,
`OneEquality()` in `xml_native_reader.cc`
**Design decision:** Follow existing defaults struct pattern
(`MjcfJointDefaults`, `MjcfActuatorDefaults`). All fields are `Option<T>`
to support "not set" semantics for cascade.

#### S1.1 Create `MjcfEqualityDefaults` struct

**File:** `sim/L0/mjcf/src/types.rs` — insert after `MjcfPairDefaults`

```rust
/// Defaults for equality constraints (connect, weld, joint, tendon, distance).
/// MuJoCo uses a single `<equality>` default per class — not per-type.
/// Only 3 fields are defaultable: active, solref, solimp.
/// MuJoCo ref: `OneEquality()` in `xml_native_reader.cc` (defaults context).
#[derive(Debug, Clone, Default)]
pub struct MjcfEqualityDefaults {
    /// Whether the constraint is initially active.
    pub active: Option<bool>,
    /// Solver reference parameters [timeconst, dampratio].
    pub solref: Option<[f64; 2]>,
    /// Solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub solimp: Option<[f64; 5]>,
}
```

#### S1.2 Add `equality` field to `MjcfDefault`

**File:** `sim/L0/mjcf/src/types.rs` — `MjcfDefault` struct (~line 556)

**Before:**
```rust
pub struct MjcfDefault {
    pub class: String,
    pub parent_class: Option<String>,
    pub joint: Option<MjcfJointDefaults>,
    pub geom: Option<MjcfGeomDefaults>,
    pub actuator: Option<MjcfActuatorDefaults>,
    pub tendon: Option<MjcfTendonDefaults>,
    pub sensor: Option<MjcfSensorDefaults>,
    pub mesh: Option<MjcfMeshDefaults>,
    pub site: Option<MjcfSiteDefaults>,
    pub pair: Option<MjcfPairDefaults>,
}
```

**After:**
```rust
pub struct MjcfDefault {
    pub class: String,
    pub parent_class: Option<String>,
    pub joint: Option<MjcfJointDefaults>,
    pub geom: Option<MjcfGeomDefaults>,
    pub actuator: Option<MjcfActuatorDefaults>,
    pub tendon: Option<MjcfTendonDefaults>,
    pub sensor: Option<MjcfSensorDefaults>,
    pub mesh: Option<MjcfMeshDefaults>,
    pub site: Option<MjcfSiteDefaults>,
    pub pair: Option<MjcfPairDefaults>,
    pub equality: Option<MjcfEqualityDefaults>,
}
```

#### S1.3 Change `active` to `Option<bool>` on all 5 equality structs

**Files:** `sim/L0/mjcf/src/types.rs` — `MjcfConnect` (line 1555),
`MjcfWeld` (line 1647), `MjcfJointEquality` (line 1746), `MjcfDistance`
(line 1858), `MjcfTendonEquality` (line 1949)

On each struct, change:
```rust
pub active: bool,
```
to:
```rust
pub active: Option<bool>,
```

And update `Default::default()` implementations to use `None` instead of
`true`.

#### S1.4 Parse `<equality>` in `parse_default()`

**File:** `sim/L0/mjcf/src/parser.rs` — `parse_default()` (lines 515, 548)

Add `b"equality"` to both match blocks (Start and Empty):

```rust
b"equality" => {
    default.equality = Some(parse_equality_defaults(e)?);
}
```

#### S1.5 Create `parse_equality_defaults()` function

**File:** `sim/L0/mjcf/src/parser.rs` — new function

```rust
fn parse_equality_defaults(e: &BytesStart) -> Result<MjcfEqualityDefaults> {
    let mut defaults = MjcfEqualityDefaults::default();

    if let Some(v) = get_attribute_opt(e, "active") {
        defaults.active = Some(v == "true");
    }
    if let Some(v) = get_attribute_opt(e, "solref") {
        defaults.solref = Some(parse_array_2(&v)?);
    }
    if let Some(v) = get_attribute_opt(e, "solimp") {
        defaults.solimp = Some(parse_array_5(&v)?);
    }

    Ok(defaults)
}
```

#### S1.6 Update `active` parsing on all 5 equality parsers

**File:** `sim/L0/mjcf/src/parser.rs` — lines 2331, 2382, 2423, 2464, 2505

Each equality parser currently has:
```rust
active: get_attribute_opt(start, "active").map_or(true, |v| v != "false"),
```

Change to (matching `Option<bool>` type + standardized `== "true"`):
```rust
active: get_attribute_opt(start, "active").map(|v| v == "true"),
```

#### S1.7 Export `MjcfEqualityDefaults` in public API

**File:** `sim/L0/mjcf/src/lib.rs` — add to the `pub use types::{...}` list
(~line 186–193).

### S2. DT-2 — Equality Defaults: Merge and Cascade

**Depends on:** S1 (struct definition)
**File:** `sim/L0/mjcf/src/defaults.rs`
**MuJoCo equivalent:** Default class inheritance (nested `<default>` elements)
**Design decision:** Follow existing merge/apply pattern. Per AD-1, single
generic `apply_equality_defaults()` taking mutable field references.

#### S2.1 Add `merge_equality_defaults()`

**File:** `sim/L0/mjcf/src/defaults.rs` — new function (alongside existing
merge functions)

```rust
fn merge_equality_defaults(
    parent: Option<&MjcfEqualityDefaults>,
    child: Option<&MjcfEqualityDefaults>,
) -> Option<MjcfEqualityDefaults> {
    match (parent, child) {
        (None, None) => None,
        (Some(p), None) => Some(p.clone()),
        (None, Some(c)) => Some(c.clone()),
        (Some(p), Some(c)) => Some(MjcfEqualityDefaults {
            active: c.active.or(p.active),
            solref: c.solref.or(p.solref),
            solimp: c.solimp.or(p.solimp),
        }),
    }
}
```

#### S2.2 Add `equality` to `merge_defaults()`

**File:** `sim/L0/mjcf/src/defaults.rs` — `merge_defaults()` (~line 664)

Add to the `MjcfDefault` constructor:
```rust
equality: Self::merge_equality_defaults(
    parent.equality.as_ref(),
    child.equality.as_ref(),
),
```

#### S2.3 Add `equality_defaults()` accessor

**File:** `sim/L0/mjcf/src/defaults.rs` — alongside existing accessors

```rust
/// Look up equality defaults for the given class name.
pub fn equality_defaults(&self, class: Option<&str>) -> Option<&MjcfEqualityDefaults> {
    let class = class.unwrap_or("");
    self.resolved_defaults
        .get(class)
        .and_then(|d| d.equality.as_ref())
}
```

#### S2.4 Add `apply_equality_defaults()` method

**File:** `sim/L0/mjcf/src/defaults.rs` — new method on `DefaultResolver`

```rust
/// Apply equality defaults to the given mutable fields.
/// Generic across all 5 equality types — they share the same 3 defaultable fields.
pub fn apply_equality_defaults(
    &self,
    class: Option<&str>,
    active: &mut Option<bool>,
    solref: &mut Option<[f64; 2]>,
    solimp: &mut Option<[f64; 5]>,
) {
    if let Some(defaults) = self.equality_defaults(class) {
        if active.is_none() {
            *active = defaults.active;
        }
        if solref.is_none() {
            *solref = defaults.solref;
        }
        if solimp.is_none() {
            *solimp = defaults.solimp;
        }
    }
}
```

### S3. DT-2 — Equality Defaults: Builder Integration

**Depends on:** S1, S2 (struct + cascade methods)
**File:** `sim/L0/mjcf/src/builder/mod.rs` (~line 294) and
`sim/L0/mjcf/src/builder/equality.rs` (lines 73–280)
**MuJoCo equivalent:** `mjCEquality` constructor applies defaults, then
compile copies to `mjModel.eq_*` arrays
**Design decision:** Call `apply_equality_defaults()` on each constraint
before processing, following the actuator pattern at `builder/mod.rs:290`.
Replace 5 hardcoded `unwrap_or(DEFAULT_SOLREF/SOLIMP)` sites with
`.unwrap_or(DEFAULT_SOLREF)` (which now reads post-cascade values).

#### S3.1 Apply equality defaults in builder pipeline

**File:** `sim/L0/mjcf/src/builder/mod.rs` — before
`process_equality_constraints()` call (~line 295)

**Chosen approach:** Pass the resolver into `process_equality_constraints()`
and apply defaults inline at each match arm, following the pattern of
actuator processing (`builder.resolver.apply_to_actuator()` at line 290).

**Before:**
```rust
builder.process_equality_constraints(&mjcf.equality)?;
```

**After:**
```rust
// Extract resolver reference before the mutable method call to avoid
// borrow conflict (&builder.resolver while &mut self).
let resolver = &builder.resolver;
builder.process_equality_constraints(&mjcf.equality, resolver)?;
```

**Note:** Depending on the borrow checker, this may require cloning the
resolved defaults or restructuring `process_equality_constraints()` to take
the resolver as a separate non-self parameter. The exact pattern will be
determined during implementation — the key requirement is that the resolver
is available inside the method.

Inside `process_equality_constraints()` (builder/equality.rs), at the top
of each match arm (Connect, Weld, Joint, Distance, Tendon), apply defaults
before reading solref/solimp/active:

```rust
// Example for Connect arm:
let mut active = connect.active;
let mut solref = connect.solref;
let mut solimp = connect.solimp;
resolver.apply_equality_defaults(connect.class.as_deref(), &mut active, &mut solref, &mut solimp);
// Then use active/solref/solimp instead of connect.active/connect.solref/connect.solimp
```

This pattern repeats for all 5 equality types. The resolver reference is
passed as a parameter to `process_equality_constraints()`, not stored on the
builder (avoiding borrow checker issues).

#### S3.2 Replace hardcoded fallbacks in `builder/equality.rs`

**File:** `sim/L0/mjcf/src/builder/equality.rs`

At each of the 5 sites (Connect:73–76, Weld:136–137, Joint:184–187,
Distance:~236, Tendon:~279), the code currently does:

```rust
self.eq_solimp.push(connect.solimp.unwrap_or(DEFAULT_SOLIMP));
self.eq_solref.push(connect.solref.unwrap_or(DEFAULT_SOLREF));
```

After S3.1 applies defaults, the `solimp` and `solref` fields are already
cascaded. The `unwrap_or(DEFAULT_SOLREF/SOLIMP)` fallback is still needed
for constraints with no class AND no explicit attribute — and in that case,
the hardcoded default IS the correct fallback (matching
`mj_defaultSolRefImp()`). So these sites remain unchanged structurally, but
now the `Option` values carry cascaded defaults, not just raw parse values.

For `active`:

```rust
// Before (bool, always has a value):
self.eq_active.push(connect.active);

// After (Option<bool>, needs fallback to MuJoCo default true):
self.eq_active.push(connect.active.unwrap_or(true));
```

### S4. DT-13 — `qpos_spring`: Model Array

**Depends on:** None (independent of DT-2)
**File:** `sim/L0/core/src/types/model.rs` (~line 186) and
`sim/L0/core/src/types/model_init.rs` (~line 93)
**MuJoCo equivalent:** `m->qpos_spring` in `mjmodel.h` (~line 1240)
**Design decision:** Add `qpos_spring: Vec<f64>` sized `nq` (same as
`qpos0`). Per AD-2, keep `jnt_springref` as builder intermediate.

#### S4.1 Add field to Model

**File:** `sim/L0/core/src/types/model.rs`

```rust
/// Spring reference position in generalized coordinates (size nq).
/// For hinge/slide: scalar springref value.
/// For ball: quaternion [w,x,y,z] copied from qpos0.
/// For free: 7D [pos_x,y,z, quat_w,x,y,z] copied from qpos0.
/// MuJoCo ref: m->qpos_spring in mjModel (engine_passive.c: mj_springdamper).
pub qpos_spring: Vec<f64>,
```

#### S4.2 Init in `model_init.rs`

**File:** `sim/L0/core/src/types/model_init.rs` — alongside other empty vec
inits

```rust
qpos_spring: vec![],
```

### S5. DT-13 — `qpos_spring`: Builder Population

**Depends on:** S4 (field declaration on Model)
**File:** `sim/L0/mjcf/src/builder/mod.rs`, `sim/L0/mjcf/src/builder/joint.rs`,
`sim/L0/mjcf/src/builder/flex.rs`, `sim/L0/mjcf/src/builder/build.rs`,
`sim/L0/mjcf/src/builder/init.rs`
**MuJoCo equivalent:** `CopyTree()` in `user_model.cc`
**Design decision:** Add `qpos_spring_values: Vec<f64>` accumulator to
`ModelBuilder`, following the same pattern as `qpos0_values` (~line 583).
Population parallels `qpos0` population in `builder/joint.rs:163–196`.

#### S5.1 Add accumulator to `ModelBuilder`

**File:** `sim/L0/mjcf/src/builder/mod.rs` — alongside `qpos0_values`
(~line 583)

```rust
pub(crate) qpos_spring_values: Vec<f64>,
```

**File:** `sim/L0/mjcf/src/builder/init.rs` — alongside `qpos0_values`
init (~line 170)

```rust
qpos_spring_values: vec![],
```

#### S5.2 Populate `qpos_spring` during joint building

**File:** `sim/L0/mjcf/src/builder/joint.rs` — after qpos0 population
(lines 163–196)

For each joint type, push matching `qpos_spring` entries immediately after
the `qpos0` entries:

```rust
match jnt_type {
    MjJointType::Hinge => {
        // qpos0
        self.qpos0_values.push(ref_val);
        // qpos_spring: uses springref (already in radians if converted)
        self.qpos_spring_values.push(spring_ref_converted);
    }
    MjJointType::Slide => {
        self.qpos0_values.push(joint.ref_pos.unwrap_or(0.0));
        self.qpos_spring_values.push(spring_ref);
    }
    MjJointType::Ball => {
        self.qpos0_values.extend_from_slice(&[1.0, 0.0, 0.0, 0.0]);
        // Ball: qpos_spring = qpos0 (identity quaternion)
        self.qpos_spring_values.extend_from_slice(&[1.0, 0.0, 0.0, 0.0]);
    }
    MjJointType::Free => {
        let q = world_quat.into_inner();
        let qpos0_free = [world_pos.x, world_pos.y, world_pos.z,
                          q.w, q.i, q.j, q.k];
        self.qpos0_values.extend_from_slice(&qpos0_free);
        // Free: qpos_spring = qpos0 (initial body pose)
        self.qpos_spring_values.extend_from_slice(&qpos0_free);
    }
}
```

Note: `spring_ref` is the scalar value from `joint.spring_ref.unwrap_or(0.0)`
and `spring_ref_converted` is the degree-converted value (matching existing
logic at line 118–123). Ball/free joints ignore `springref` and copy `qpos0`
— matching MuJoCo's `CopyTree()`.

#### S5.3 Populate `qpos_spring` for flex vertex slide joints

**File:** `sim/L0/mjcf/src/builder/flex.rs` — after `jnt_springref.push(0.0)`
(~line 197)

```rust
self.jnt_springref.push(0.0);
self.qpos_spring_values.push(0.0); // Slide joint springref = 0.0
```

Each flex vertex body creates 3 slide joints (x, y, z axes). Each needs a
corresponding `qpos_spring` scalar entry of `0.0`.

#### S5.4 Convert accumulator to Model field in `build.rs`

**File:** `sim/L0/mjcf/src/builder/build.rs` — alongside `qpos0` conversion
(~line 347)

```rust
model.qpos_spring = self.qpos_spring_values.clone();
```

#### S5.5 Update tendon sentinel resolution

**File:** `sim/L0/mjcf/src/builder/build.rs` — tendon sentinel code
(~lines 477–498)

**Before:**
```rust
if dof_adr < model.qpos0.len() {
    length += coef * model.qpos0[dof_adr];
}
```

**After:**
```rust
if dof_adr < model.qpos_spring.len() {
    length += coef * model.qpos_spring[dof_adr];
}
```

**Index space note:** The current code uses `wrap_objid[w]` which is a DOF
address. For hinge/slide joints, `dof_adr == qpos_adr` (both scalar), so
`qpos_spring[dof_adr]` works correctly. For ball/free joints where
`dof_adr != qpos_adr`, the fixed-tendon wrapping only supports hinge/slide
joints (spatial tendons use a different path), so the index space is safe.

### S6. DT-13 — `qpos_spring`: Consumer Migration

**Depends on:** S5 (field populated during build)
**File:** `sim/L0/core/src/forward/passive.rs` (~line 804),
`sim/L0/core/src/energy.rs` (~line 46),
`sim/L0/core/src/types/model_init.rs` (~line 823)
**MuJoCo equivalent:** `mj_springdamper()` in `engine_passive.c`,
`mj_energyPos()` in `engine_core_smooth.c`
**Design decision:** Migrate runtime consumers from `jnt_springref[jnt_id]`
to `qpos_spring[qpos_adr]`. The change is numerically invisible for
hinge/slide (values are identical), but structurally correct.

#### S6.1 Update `passive.rs` spring force

**File:** `sim/L0/core/src/forward/passive.rs` — `visit_1dof_joint()` (~line
804)

**Before:**
```rust
let springref = self.model.jnt_springref[jnt_id];
let q = self.data.qpos[qpos_adr];
self.data.qfrc_spring[dof_adr] -= stiffness * (q - springref);
```

**After:**
```rust
let springref = self.model.qpos_spring[qpos_adr];
let q = self.data.qpos[qpos_adr];
self.data.qfrc_spring[dof_adr] -= stiffness * (q - springref);
```

Ball/free spring force computation is Spec B §64 scope — not changed here.

#### S6.2 Update `energy.rs` spring energy

**File:** `sim/L0/core/src/energy.rs` (~line 46)

**Before:**
```rust
let springref = model.jnt_springref[jnt_id];
```

**After:**
```rust
let springref = model.qpos_spring[qpos_adr];
```

Ball/free spring energy computation is Spec B §64 scope — the stub remains.

#### S6.3 Update `model_init.rs` `implicit_springref`

**File:** `sim/L0/core/src/types/model_init.rs` — `compute_implicit_params()`
(~line 823)

**Before:**
```rust
MjJointType::Hinge | MjJointType::Slide => {
    self.implicit_springref[dof_adr] = self.jnt_springref[jnt_id];
}
```

**After:**
```rust
MjJointType::Hinge | MjJointType::Slide => {
    self.implicit_springref[dof_adr] = self.qpos_spring[self.jnt_qpos_adr[jnt_id]];
}
```

**Index space mapping:** `implicit_springref` is sized `nv` (velocity DOFs),
`qpos_spring` is sized `nq`. For hinge/slide, `nv_jnt == nq_jnt == 1`, so
`dof_adr == qpos_adr` — the mapping is trivial. For ball/free joints, the
existing code already sets `implicit_springref[dof_adr+i] = 0.0` for each
DOF, which remains unchanged.

### S7. DT-13 — `qpos_spring`: Test Model Factory Updates

**Depends on:** S4 (field added to Model)
**File:** `sim/L0/core/src/types/model_factories.rs` (lines 89, 190, 278),
`sim/L0/core/src/sensor/mod.rs` (lines 105, 1020, 1137),
`sim/L0/core/src/jacobian.rs` (7 sites),
`sim/L0/constraint/src/jacobian.rs` (2 sites),
`sim/L0/core/src/forward/muscle.rs` (6 sites)
**Design decision:** Mechanical update — each site that pushes
`jnt_springref` also populates `qpos_spring`. For hinge/slide test models,
`qpos_spring` gets the same scalar value. For free-body factories,
`qpos_spring` gets the same 7D values as `qpos0`.

~18 sites total. Each follows the pattern:

```rust
// Existing:
model.jnt_springref.push(0.0);

// Add alongside:
model.qpos_spring.push(0.0);  // For hinge/slide
// OR for free body:
model.qpos_spring.extend_from_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]); // pos + identity quat
```

### S8. DT-14 — Actuator Defaults: Parser Extension

**Depends on:** None (independent of DT-2, DT-13)
**File:** `sim/L0/mjcf/src/parser.rs` — `parse_default()` (lines 515, 548)
and `parse_actuator_defaults()` (~line 739)
**MuJoCo equivalent:** `Default()` in `xml_native_reader.cc`
**Design decision:** Add missing shortcut names to the match arm. All
dispatch to the same `parse_actuator_defaults()` — matching MuJoCo's single
`mjsActuator` per default class.

#### S8.1 Extend `parse_default()` match arms

**File:** `sim/L0/mjcf/src/parser.rs` — both Start and Empty matches

**Before:**
```rust
b"actuator" | b"motor" | b"position" | b"velocity" | b"general" => {
    default.actuator = Some(parse_actuator_defaults(e)?);
}
```

**After:**
```rust
b"actuator" | b"motor" | b"position" | b"velocity" | b"general"
| b"cylinder" | b"muscle" | b"adhesion" | b"damper" | b"intvelocity" => {
    default.actuator = Some(parse_actuator_defaults(e)?);
}
```

#### S8.2 Add type-specific fields to `MjcfActuatorDefaults`

**File:** `sim/L0/mjcf/src/types.rs` — `MjcfActuatorDefaults` struct
(~line 682–730)

Add these fields after the existing comment about missing fields:

```rust
// Cylinder-specific defaults
pub area: Option<f64>,
pub diameter: Option<f64>,
pub timeconst: Option<f64>,
pub bias: Option<[f64; 3]>,

// Muscle-specific defaults
pub muscle_timeconst: Option<(f64, f64)>,
pub range: Option<(f64, f64)>,
pub force: Option<f64>,
pub scale: Option<f64>,
pub lmin: Option<f64>,
pub lmax: Option<f64>,
pub vmax: Option<f64>,
pub fpmax: Option<f64>,
pub fvmax: Option<f64>,

// Adhesion-specific defaults
pub gain: Option<f64>,
```

Remove the `#todo` comment that documents these fields as missing.

#### S8.3 Parse type-specific attributes in `parse_actuator_defaults()`

**File:** `sim/L0/mjcf/src/parser.rs` — `parse_actuator_defaults()` (~line
739)

Add parsing for each new field:

```rust
if let Some(v) = get_attribute_opt(e, "area") {
    defaults.area = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "diameter") {
    defaults.diameter = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "timeconst") {
    // Parse as single value or tuple depending on context
    let parts: Vec<f64> = parse_float_list(&v)?;
    if parts.len() == 1 {
        defaults.timeconst = Some(parts[0]);
    } else if parts.len() == 2 {
        defaults.muscle_timeconst = Some((parts[0], parts[1]));
    }
}
if let Some(v) = get_attribute_opt(e, "bias") {
    defaults.bias = Some(parse_array_3(&v)?);
}
if let Some(v) = get_attribute_opt(e, "range") {
    defaults.range = Some(parse_pair(&v)?);
}
if let Some(v) = get_attribute_opt(e, "force") {
    defaults.force = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "scale") {
    defaults.scale = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "lmin") {
    defaults.lmin = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "lmax") {
    defaults.lmax = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "vmax") {
    defaults.vmax = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "fpmax") {
    defaults.fpmax = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "fvmax") {
    defaults.fvmax = Some(v.parse::<f64>()?);
}
if let Some(v) = get_attribute_opt(e, "gain") {
    defaults.gain = Some(v.parse::<f64>()?);
}
```

**`timeconst` ambiguity:** In MuJoCo, `timeconst` for cylinders is a single
float, but for muscles it's a 2-element tuple `(act, deact)`. The MuJoCo
XML reference resolves this: `timeconst` on `<cylinder>` is `real(1)`,
`timeconst` on `<muscle>` is `real(2)`. Since `parse_actuator_defaults()`
is called for all shortcut types, parse by length: 1 value → `timeconst`
(cylinder), 2 values → `muscle_timeconst`.

### S9. DT-14 — Actuator Defaults: Merge and Cascade

**Depends on:** S8 (parser + struct fields)
**File:** `sim/L0/mjcf/src/defaults.rs` — `merge_actuator_defaults()`
(~line 752) and `apply_to_actuator()` (~line 294)
**MuJoCo equivalent:** Default class inheritance for actuator attributes
**Design decision:** Extend existing merge and apply functions with new fields.
Each new field is `Option<T>` so `c.field.or(p.field)` merge is correct.

#### S9.1 Extend `merge_actuator_defaults()`

**File:** `sim/L0/mjcf/src/defaults.rs` — inside the `(Some(p), Some(c))`
arm (~line 762)

Add after existing fields:

```rust
area: c.area.or(p.area),
diameter: c.diameter.or(p.diameter),
timeconst: c.timeconst.or(p.timeconst),
bias: c.bias.or(p.bias),
muscle_timeconst: c.muscle_timeconst.or(p.muscle_timeconst),
range: c.range.or(p.range),
force: c.force.or(p.force),
scale: c.scale.or(p.scale),
lmin: c.lmin.or(p.lmin),
lmax: c.lmax.or(p.lmax),
vmax: c.vmax.or(p.vmax),
fpmax: c.fpmax.or(p.fpmax),
fvmax: c.fvmax.or(p.fvmax),
gain: c.gain.or(p.gain),
```

#### S9.2 Extend `apply_to_actuator()`

**File:** `sim/L0/mjcf/src/defaults.rs` — inside `apply_to_actuator()`
(~line 294)

The challenge: on `MjcfActuator`, type-specific fields use non-Option types
with sentinel defaults (`area: f64` default `1.0`, `bias: [f64; 3]` default
`[0,0,0]`). We need sentinel-value detection to know if the element-level
field was explicitly set.

For fields where the default value IS a valid user value (e.g., `area=1.0`,
`gain=1.0`), sentinel detection is inherently unreliable. MuJoCo has the
same issue — `mjs_setToMuscle()` uses `gainprm[0]==1` as a sentinel.

**Approach:** Use the same pattern as existing `kp` handling
(defaults.rs:341). Check if the element value equals the MjcfActuator
default, and if so, apply the class default:

```rust
// Cylinder-specific
if (result.area - 1.0).abs() < 1e-10 {
    if let Some(area) = defaults.area { result.area = area; }
}
if result.diameter.is_none() {
    result.diameter = defaults.diameter;
}
if result.timeconst.is_none() {
    result.timeconst = defaults.timeconst;
}
if result.bias == [0.0, 0.0, 0.0] {
    if let Some(bias) = defaults.bias { result.bias = bias; }
}

// Muscle-specific
if (result.muscle_timeconst.0 - 0.01).abs() < 1e-10
    && (result.muscle_timeconst.1 - 0.04).abs() < 1e-10 {
    if let Some(tc) = defaults.muscle_timeconst { result.muscle_timeconst = tc; }
}
if (result.range.0 - 0.75).abs() < 1e-10
    && (result.range.1 - 1.05).abs() < 1e-10 {
    if let Some(r) = defaults.range { result.range = r; }
}
if (result.force - (-1.0)).abs() < 1e-10 {
    if let Some(f) = defaults.force { result.force = f; }
}
if (result.scale - 200.0).abs() < 1e-10 {
    if let Some(s) = defaults.scale { result.scale = s; }
}
if (result.lmin - 0.5).abs() < 1e-10 {
    if let Some(v) = defaults.lmin { result.lmin = v; }
}
if (result.lmax - 1.6).abs() < 1e-10 {
    if let Some(v) = defaults.lmax { result.lmax = v; }
}
if (result.vmax - 1.5).abs() < 1e-10 {
    if let Some(v) = defaults.vmax { result.vmax = v; }
}
if (result.fpmax - 1.3).abs() < 1e-10 {
    if let Some(v) = defaults.fpmax { result.fpmax = v; }
}
if (result.fvmax - 1.2).abs() < 1e-10 {
    if let Some(v) = defaults.fvmax { result.fvmax = v; }
}

// Adhesion-specific
if (result.gain - 1.0).abs() < 1e-10 {
    if let Some(g) = defaults.gain { result.gain = g; }
}
```

**Per-field wrapping strategy:**

| Defaults field | `MjcfActuator` type | Default value | Strategy | Detection |
|---------------|--------------------|-----------|---------|----|
| `area` | `f64` | `1.0` | Sentinel | `(val - 1.0).abs() < 1e-10` |
| `diameter` | `Option<f64>` | `None` | Option | `is_none()` |
| `timeconst` | `Option<f64>` | `None` | Option | `is_none()` |
| `bias` | `[f64; 3]` | `[0,0,0]` | Sentinel | `== [0.0, 0.0, 0.0]` |
| `muscle_timeconst` | `(f64, f64)` | `(0.01, 0.04)` | Sentinel | `(val.0 - 0.01).abs() < 1e-10 && ...` |
| `range` | `(f64, f64)` | `(0.75, 1.05)` | Sentinel | `(val.0 - 0.75).abs() < 1e-10 && ...` |
| `force` | `f64` | `-1.0` | Sentinel | `(val - (-1.0)).abs() < 1e-10` |
| `scale` | `f64` | `200.0` | Sentinel | `(val - 200.0).abs() < 1e-10` |
| `lmin` | `f64` | `0.5` | Sentinel | `(val - 0.5).abs() < 1e-10` |
| `lmax` | `f64` | `1.6` | Sentinel | `(val - 1.6).abs() < 1e-10` |
| `vmax` | `f64` | `1.5` | Sentinel | `(val - 1.5).abs() < 1e-10` |
| `fpmax` | `f64` | `1.3` | Sentinel | `(val - 1.3).abs() < 1e-10` |
| `fvmax` | `f64` | `1.2` | Sentinel | `(val - 1.2).abs() < 1e-10` |
| `gain` | `f64` | `1.0` | Sentinel | `(val - 1.0).abs() < 1e-10` |

Fields with `Option<T>` type use Option-based cascade (correct, no ambiguity).
Fields with non-Option types use sentinel detection (matches MuJoCo behavior).

**Known limitation:** Sentinel detection is inherently imperfect. If a user
explicitly sets `area="1.0"` (the default), the defaults cascade will
overwrite it with the class default (if different). MuJoCo has the same
behavior — it's a design quirk, not a bug. The spec documents this as a
known divergence for the rare case where a user explicitly sets a field to
its default value while also having a class default that differs.

---

## Acceptance Criteria

### AC1: Equality defaults — solref cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><equality solref="0.05 0.8"/></default>` and
a `<connect class="" body1="b1" body2="b2" anchor="0 0 0"/>`
**After:** Load model via `load_model()`
**Assert:** `model.eq_solref[0] == [0.05, 0.8]`
**Field:** `Model.eq_solref`

### AC2: Equality defaults — solimp cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><equality solimp="0.8 0.9 0.01 0.4 3.0"/></default>`
and a `<weld class="" body1="b1"/>`
**After:** Load model
**Assert:** `model.eq_solimp[0] == [0.8, 0.9, 0.01, 0.4, 3.0]`
**Field:** `Model.eq_solimp`

### AC3: Equality defaults — active cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><equality active="false"/></default>` and a
`<connect class="" body1="b1" body2="b2" anchor="0 0 0"/>` (no explicit
`active` attribute)
**After:** Load model
**Assert:** `model.eq_active[0] == false`
**Field:** `Model.eq_active`

### AC4: Equality defaults — no class → hardcoded default *(runtime test, analytically derived)*

**Given:** MJCF with a `<connect body1="b1" body2="b2" anchor="0 0 0"/>`
(no class, no explicit solref)
**After:** Load model
**Assert:** `model.eq_solref[0] == [0.02, 1.0]` (MuJoCo built-in default)
**Field:** `Model.eq_solref`

### AC5: `qpos_spring` — hinge joint with springref *(runtime test, analytically derived)*

**Given:** MJCF with one hinge joint, `springref="0.5"` (radians)
**After:** Load model
**Assert:** `model.qpos_spring[qpos_adr] == 0.5`
**Field:** `Model.qpos_spring`

### AC6: `qpos_spring` — ball joint *(runtime test, analytically derived)*

**Given:** MJCF with one ball joint (default qpos0 = identity quaternion)
**After:** Load model
**Assert:** `model.qpos_spring[qpos_adr..qpos_adr+4] == [1.0, 0.0, 0.0, 0.0]`
**Field:** `Model.qpos_spring`

### AC7: `qpos_spring` — free joint *(runtime test, analytically derived)*

**Given:** MJCF with one free joint body at position `[1, 2, 3]` and
quaternion `[0.707107, 0.707107, 0, 0]` (90° rotation about X)
**After:** Load model
**Assert:** `model.qpos_spring[0..7] == [1.0, 2.0, 3.0, 0.707107, 0.707107, 0.0, 0.0]`
(non-identity quaternion proves the 7D copy is from actual `qpos0`, not hardcoded)
**Field:** `Model.qpos_spring`

### AC8: `qpos_spring` — hinge/slide spring force regression *(runtime test, analytically derived)*

**Given:** One hinge joint, stiffness=100.0, springref=0.0, qpos[0]=0.5
**After:** `mj_passive()` / spring force computation
**Assert:** `qfrc_spring[0] == -100.0 * (0.5 - 0.0) == -50.0 ± 1e-12`
(identical to pre-Spec A because `qpos_spring[0] == springref == 0.0`)
**Field:** `Data.qfrc_spring`

### AC9: Actuator defaults — cylinder area cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><cylinder area="0.01"/></default>` and
`<actuator><cylinder class="" joint="j1"/></actuator>`
**After:** Load model
**Assert:** `model.actuator_gainprm[0][0] == 0.01` (area → gainprm[0])
**Field:** `Model.actuator_gainprm`

### AC10: Actuator defaults — muscle params cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><muscle range="0.5 1.2"/></default>` and
`<actuator><muscle class="" joint="j1"/></actuator>`
**After:** Load model
**Assert:** `model.actuator_gainprm[0][0] == 0.5` and
`model.actuator_gainprm[0][1] == 1.2`
**Field:** `Model.actuator_gainprm`

### AC11: Tendon sentinel resolution uses `qpos_spring` *(runtime test, analytically derived)*

**Given:** MJCF with a fixed tendon wrapping a single hinge joint with
`springref="0.3"` (radians), `ref="0.0"`, coefficient `coef="1.0"`,
`springlength="-1 -1"` (sentinel)
**After:** Load model
**Assert:** `model.tendon_lengthspring[0] == [0.3, 0.3]` (spring rest length
resolved at the `qpos_spring` configuration, where the hinge is at 0.3 rad).
`model.tendon_length0[0] == 0.0` (length at `qpos0`, where the hinge is at
0.0 rad). These differ because `qpos_spring[0] = springref = 0.3` while
`qpos0[0] = ref = 0.0`. The tendon length is `coef * qpos = 1.0 * 0.3 = 0.3`.
**Field:** `Model.tendon_lengthspring`, `Model.tendon_length0`

### AC12: No regression *(runtime test)*

**Given:** Full sim domain test suite
**After:** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`
**Assert:** All pre-existing tests pass (test factory updates are mechanical,
not behavioral)
**Field:** All test assertions

### AC13: Adhesion defaults — gain cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><adhesion gain="0.5"/></default>` and
`<actuator><adhesion class="" body="b1"/></actuator>`
**After:** Load model
**Assert:** `model.actuator_gainprm[0][0] == 0.5` (gain → gainprm[0])
**Field:** `Model.actuator_gainprm`

### AC14: Damper defaults — kv cascade *(runtime test, analytically derived)*

**Given:** MJCF with `<default><damper kv="5.0"/></default>` and
`<actuator><damper class="" joint="j1"/></actuator>`
**After:** Load model
**Assert:** `model.actuator_gainprm[0][2] == -5.0` (damper: gainprm[2] = -kv)
**Field:** `Model.actuator_gainprm`

### AC15: Ball `qpos_spring` copies from `qpos0`, not hardcoded *(code review)*

Verify in `builder/joint.rs` that ball joint `qpos_spring` population copies
from the same source as `qpos0` (currently `[1.0, 0.0, 0.0, 0.0]`) rather
than hardcoding the identity quaternion. Structural verification: the code
must use the same values as the `qpos0` push, not independent constants.

### AC16: `MjcfEqualityDefaults` exported *(code review)*

`MjcfEqualityDefaults` is exported in `sim/L0/mjcf/src/lib.rs` public API,
following the pattern of other defaults types.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (eq solref cascade) | T1 | Direct |
| AC2 (eq solimp cascade) | T1 | Direct (combined model) |
| AC3 (eq active cascade) | T2 | Direct |
| AC4 (eq no class → default) | T3 | Edge case |
| AC5 (qpos_spring hinge) | T4 | Direct |
| AC6 (qpos_spring ball) | T5 | Direct |
| AC7 (qpos_spring free) | T6 | Direct |
| AC8 (spring force regression) | T7 | Regression + analytical |
| AC9 (actuator cylinder defaults) | T8 | Direct |
| AC10 (actuator muscle defaults) | T9 | Direct |
| AC11 (tendon sentinel) | T10 | Direct |
| AC12 (no regression) | Existing suite | Regression |
| AC13 (adhesion defaults) | T14 | Direct |
| AC14 (damper defaults) | T15 | Direct |
| AC15 (ball qpos_spring structural) | — | Code review (manual) |
| AC16 (export) | — | Code review (manual) |

---

## Test Plan

### T1: Equality defaults — solref/solimp cascade → AC1, AC2

MJCF model with `<default><equality solref="0.05 0.8" solimp="0.8 0.9 0.01 0.4 3.0"/></default>`
containing connect and weld constraints (no explicit solref/solimp). Assert
`model.eq_solref[0] == [0.05, 0.8]` and `model.eq_solimp[0] == [0.8, 0.9, 0.01, 0.4, 3.0]`
for connect; same values at index 1 for weld. Analytically derived — values are
directly from the defaults attribute.

### T2: Equality defaults — active=false cascade → AC3

MJCF model with `<default><equality active="false"/></default>` containing a
connect constraint with no explicit `active`. Assert `eq_active[0] == false`.

### T3: Equality defaults — no class → MuJoCo built-in → AC4

MJCF model with a connect constraint (no class, no explicit solref/solimp).
Assert `eq_solref[0] == [0.02, 1.0]` and `eq_solimp[0] == [0.9, 0.95, 0.001, 0.5, 2.0]`.
Also verifies that `<default><equality>` does NOT cascade `data`, `anchor`, or
`polycoef` attributes — these are type-specific and excluded from the defaults
context (guarded by `readingdefaults` in MuJoCo). The `parse_equality_defaults()`
function only parses `active`/`solref`/`solimp`, structurally preventing leakage.

### T4: `qpos_spring` — hinge with explicit springref → AC5

MJCF model with hinge joint, `springref="0.5"`. Assert
`model.qpos_spring[qpos_adr] == 0.5`.

### T5: `qpos_spring` — ball joint identity quaternion → AC6

MJCF model with ball joint. Assert
`model.qpos_spring[qpos_adr..+4] == [1.0, 0.0, 0.0, 0.0]`.

### T6: `qpos_spring` — free joint body pose → AC7

MJCF model with free joint body at `pos="1 2 3" quat="0.707107 0.707107 0 0"`.
Assert `model.qpos_spring[0..7] == [1.0, 2.0, 3.0, 0.707107, 0.707107, 0.0, 0.0]`.
Non-identity quaternion proves the 7D copy is from actual `qpos0`, not hardcoded.

### T7: Spring force regression — hinge/slide unchanged → AC8

Hinge joint, stiffness=100.0, springref=0.0, qpos[0]=0.5. Assert
`qfrc_spring[0] == -50.0 ± 1e-12`. Values are bit-identical before and after
the `jnt_springref → qpos_spring` migration because
`qpos_spring[0] == springref == 0.0`.

### T8: Cylinder area defaults cascade → AC9

MJCF model with `<default><cylinder area="0.01"/></default>` and a cylinder
actuator inheriting the default. Assert `actuator_gainprm[0][0] == 0.01`.

### T9: Muscle range defaults cascade → AC10

MJCF model with `<default><muscle range="0.5 1.2"/></default>` and a muscle
actuator inheriting the default. Assert gainprm[0]=0.5, gainprm[1]=1.2.

### T10: Tendon sentinel uses `qpos_spring` → AC11

MJCF model with fixed tendon wrapping a hinge joint with `springref="0.3"`,
`springlength="-1"`. Assert `model.tendon_lengthspring[0] == [0.3, 0.3] ± 1e-12`
and `model.tendon_length0[0] == 0.0 ± 1e-12`. This differs from using `qpos0`
because `qpos_spring[0] = springref = 0.3` while `qpos0[0] = ref = 0.0`.

### T11: Equality defaults — nested class inheritance → (supplementary)

MJCF with `<default class="parent"><equality solref="0.05 0.8"/><default class="child"><equality solimp="0.8 0.9 0.01 0.4 3.0"/></default></default>`.
Assert child-class constraint has `eq_solref == [0.05, 0.8]` (inherited from
parent) and `eq_solimp == [0.8, 0.9, 0.01, 0.4, 3.0]` (child's own).

### T12: Multiple actuator shortcuts in one default → (supplementary)

MJCF with `<default><motor/><cylinder area="0.02"/></default>`. Assert
`defaults.actuator.area == Some(0.02)` (cylinder's value, not motor's default).
Matches MuJoCo's "last wins" semantics (struct replacement at parse level).

### T13: Mixed joint types — `qpos_spring` alignment → (supplementary)

MJCF with hinge (springref=0.5) + slide (springref=0.1) + ball + free (pos=1,2,3)
joints in one model. Assert `qpos_spring[0] == 0.5` (hinge), `qpos_spring[1] == 0.1`
(slide), `qpos_spring[2..6] == [1.0, 0.0, 0.0, 0.0]` (ball),
`qpos_spring[6..13] == [1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]` (free).
Verifies `qpos_spring` entries align correctly with `qpos0` by `jnt_qpos_adr`.

### T14: Adhesion gain defaults cascade → AC13

MJCF model with `<default><adhesion gain="0.5"/></default>` and an adhesion
actuator inheriting the default. Assert `actuator_gainprm[0][0] == 0.5`.

### T15: Damper kv defaults cascade → AC14

MJCF model with `<default><damper kv="5.0"/></default>` and a damper
actuator inheriting the default. Assert `actuator_gainprm[0][2] == -5.0`
(damper expansion: `gainprm[2] = -kv`).

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Equality constraint with NO class, no explicit solref | Must get hardcoded default `[0.02, 1.0]` | T3 | AC4 |
| Nested class inheritance for equality | Child overrides solimp, inherits parent solref | T11 | AC1 |
| `qpos_spring` for ball (identity quaternion from qpos0) | Ball ignores `springref`; must copy qpos0 | T5 | AC6 |
| `qpos_spring` for free with non-identity quat | Free copies full 7D pose from qpos0 (non-identity quat proves actual copy) | T6 | AC7 |
| Hinge springref=0 → `qpos_spring[adr]==0` same as `qpos0[adr]` | Regression: values numerically identical to before | T7 | AC8 |
| Multiple actuator shortcuts in one `<default>` | Last wins (struct replacement at parse level) | T12 | AC9 |
| Tendon sentinel with springref≠0 | Only test that proves qpos0→qpos_spring matters | T10 | AC11 |
| `<default><equality>` does NOT cascade data/anchor/polycoef | These are type-specific, not in defaults context | T3 | AC4 |
| `active` defaults cascade (Option<bool> migration) | Previous `bool` type prevented cascade | T2 | AC3 |
| Mixed joint types qpos_spring alignment | Different nq per joint type must align | T13 | AC5-AC7 |
| Element-level override beats class default | `<connect solref="0.1 0.5" class="X">` should use 0.1/0.5 not X's default | T1 (variant) | AC1 |
| `<default><cylinder area="0.01"/>` then `<cylinder class="...">` | Cylinder-specific defaults cascade | T8 | AC9 |
| `<default><adhesion gain="0.5"/>` then `<adhesion class="...">` | Adhesion-specific defaults cascade | T14 | AC13 |
| `<default><damper kv="5.0"/>` then `<damper class="...">` | Damper-specific defaults cascade; expansion maps kv→gainprm[2]=-kv | T15 | AC14 |
| Ball `qpos_spring` copies from `qpos0` structurally | Prevents hardcoded identity quat diverging from actual qpos0 in future | — | AC15 (code review) |
| Muscle sentinel detection (`gainprm[0]==1` → 0.75 overwrite) | MuJoCo quirk only manifests for `<general dyntype="muscle">` path; documented as known divergence — see Out of Scope | — | — (deferred) |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T11 (nested class) | Nested defaults class inheritance for equality | Verifies merge logic, not just apply logic |
| T12 (multiple shortcuts) | Last-wins semantics for actuator shortcut defaults | Conformance edge case: MuJoCo overwrites previous shortcut |
| T13 (mixed joints) | qpos_spring array alignment across joint types | Integration test — ensures builder correctly tracks nq offsets |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Equality constraints inherit from default classes | Hardcoded `[0.02, 1.0]`/`[0.9, 0.95, ...]` fallback | Class defaults cascade, then hardcoded fallback | Toward MuJoCo | Builder equality processing | None — transparent when no `<default><equality>` exists |
| `active` type on equality structs | `bool` (default `true`) | `Option<bool>` (default `None`) | Toward MuJoCo | All code reading `active` | `.unwrap_or(true)` at use sites |
| Spring force reads `qpos_spring` | Reads `jnt_springref[jnt_id]` | Reads `qpos_spring[qpos_adr]` | Toward MuJoCo | `passive.rs`, `energy.rs` | Numerically identical for hinge/slide |
| Tendon sentinel resolution | Uses `qpos0[dof_adr]` | Uses `qpos_spring[dof_adr]` | Toward MuJoCo | `build.rs` tendon code | Different results only when springref≠0 |
| `implicit_springref` source | `jnt_springref[jnt_id]` | `qpos_spring[qpos_adr]` | Toward MuJoCo | `model_init.rs` | Numerically identical for hinge/slide |
| Actuator defaults parse `cylinder|muscle|...` | Silently dropped | Parsed into `MjcfActuatorDefaults` | Toward MuJoCo | Parser | None — previously no-op |
| Actuator type-specific defaults cascade | Not cascaded | Cascaded via `apply_to_actuator()` | Toward MuJoCo | `defaults.rs` | None — new code path |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | Create `MjcfEqualityDefaults`; add to `MjcfDefault`; change `active` to `Option<bool>` on 5 equality structs; add 14 fields to `MjcfActuatorDefaults` | ~80 |
| `sim/L0/mjcf/src/parser.rs` | Add `b"equality"` + 5 shortcut names to `parse_default()` match; create `parse_equality_defaults()`; add type-specific attrs to `parse_actuator_defaults()`; update 5 equality `active` parsers | ~60 |
| `sim/L0/mjcf/src/defaults.rs` | Add `merge_equality_defaults()`; add `equality_defaults()` accessor; add `apply_equality_defaults()`; extend `merge_defaults()`; extend `merge_actuator_defaults()` (14 fields); extend `apply_to_actuator()` (14 fields) | ~100 |
| `sim/L0/mjcf/src/builder/equality.rs` | Update `active` handling from `bool` to `Option<bool>` at 5 sites | ~10 |
| `sim/L0/mjcf/src/builder/mod.rs` | Add equality defaults resolution before `process_equality_constraints()`; add `qpos_spring_values` accumulator | ~15 |
| `sim/L0/mjcf/src/builder/joint.rs` | Add `qpos_spring_values` population per joint type | ~20 |
| `sim/L0/mjcf/src/builder/flex.rs` | Add `qpos_spring_values.push(0.0)` for flex vertex joints | ~3 |
| `sim/L0/mjcf/src/builder/build.rs` | Convert `qpos_spring_values` to Model; update tendon sentinel | ~10 |
| `sim/L0/mjcf/src/builder/init.rs` | Init `qpos_spring_values: vec![]` | ~1 |
| `sim/L0/mjcf/src/lib.rs` | Export `MjcfEqualityDefaults` | ~1 |
| `sim/L0/core/src/types/model.rs` | Add `qpos_spring: Vec<f64>` | ~4 |
| `sim/L0/core/src/types/model_init.rs` | Init `qpos_spring`; update `implicit_springref` source | ~4 |
| `sim/L0/core/src/forward/passive.rs` | Change `jnt_springref[jnt_id]` → `qpos_spring[qpos_adr]` | ~2 |
| `sim/L0/core/src/energy.rs` | Change `jnt_springref[jnt_id]` → `qpos_spring[qpos_adr]` | ~2 |
| `sim/L0/core/src/types/model_factories.rs` | Add `qpos_spring` entries (3 factories) | ~12 |
| Various test files | Add `qpos_spring` entries (~15 more sites) | ~30 |
| Test file(s) for Spec A | New test module | ~200 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_equality_connect` | `builder/equality.rs` tests | Pass — values unchanged | No class defaults in test models |
| `test_equality_weld` | `builder/equality.rs` tests | Pass — values unchanged | Same |
| Spring force tests | `forward/passive.rs` tests | Pass — values unchanged | hinge/slide: `qpos_spring[adr] == jnt_springref[jnt_id]` when both are 0.0 |
| Energy tests | `energy.rs` tests | Pass — values unchanged | Same reasoning |
| Tendon tests | `build.rs` tendon tests | Pass — values unchanged | Test models use springref=0 |
| Model factory tests | `model_factories.rs` tests | Compile error → fix | Must add `qpos_spring` field |
| Jacobian tests | `jacobian.rs` tests | Compile error → fix | Must add `qpos_spring` to test models |
| Muscle tests | `forward/muscle.rs` tests | Compile error → fix | Must add `qpos_spring` to test models |
| Sensor tests | `sensor/mod.rs` tests | Compile error → fix | Must add `qpos_spring` to test models |
| Constraint tests | `constraint/jacobian.rs` tests | Compile error → fix | Must add `qpos_spring` to test models |

---

## Execution Order

1. **S1 + S2** (DT-2: equality defaults struct + parse + merge + cascade)
   → verify: parse a model with `<default><equality>`, inspect parsed model
2. **S3** (DT-2: builder integration) → verify: AC1–AC4 pass
3. **S4 + S5 + S7** (DT-13: model array + builder population + test factory
   updates) → verify: model compiles, existing tests pass after factory
   updates
4. **S6** (DT-13: consumer migration) → verify: AC5–AC8 pass, AC11 pass
5. **S8 + S9** (DT-14: parser extension + merge/cascade) → verify: AC9–AC10
   pass
6. Run full domain test suite → verify AC12

S1–S3 (DT-2) and S4–S7 (DT-13) are independent and can be done in either
order. S8–S9 (DT-14) are independent of both. The ordering above groups by
task for clarity.

---

## Out of Scope

- **DT-11 (Joint `range` default)** — Already implemented. Verified in rubric
  EGT-4. Will be re-verified during review (Session 7). *Conformance impact:
  none — already conformant.*

- **Ball/free spring force computation** — The `mji_subQuat` quaternion
  spring force path is Spec B §64 scope. Spec A provides `qpos_spring` data
  but does not implement the runtime force computation for ball/free.
  *Conformance impact: ball/free spring forces remain unimplemented (explicit
  stub in `passive.rs`). Spec B will consume `qpos_spring` for this.*

- **Ball/free spring energy computation** — Same as above. The stub in
  `energy.rs` (lines 50–53) remains until Spec B §64. *Conformance impact:
  spring potential energy for ball/free joints remains zero.*

- **`IntVelocity` enum variant** — The `MjcfActuatorType` enum (types.rs:2292)
  does not have an `IntVelocity` variant. Adding it is NOT in Spec A scope
  (DT-14 is about defaults, not actuator type completeness). The defaults
  parse arm `b"intvelocity"` dispatches to `parse_actuator_defaults()` which
  is type-agnostic. If a concrete `<intvelocity>` element is encountered in
  the actuator section, it would need an enum variant — but that is a
  separate task. *Conformance impact: `<intvelocity>` defaults parse
  correctly; concrete `<intvelocity>` elements not yet supported.*

- **Muscle sentinel detection for `<general>` path** — MuJoCo's
  `mjs_setToMuscle()` checks `gainprm[0]==1` as a sentinel. This only
  manifests for `<general dyntype="muscle" gainprm="1">`, which is rare.
  Replicating this quirk in the defaults cascade is not worth the complexity
  for v1.0. *Conformance impact: minimal — affects only `<general>` with
  explicit muscle dyntype and gainprm=1.*

- **Migration of `MjcfActuator` type-specific fields from non-Option to
  Option** — This would eliminate sentinel detection in `apply_to_actuator()`.
  Deferred as code quality improvement (DT-15 adjacent). *Conformance impact:
  none — sentinel detection matches MuJoCo's behavior.*
