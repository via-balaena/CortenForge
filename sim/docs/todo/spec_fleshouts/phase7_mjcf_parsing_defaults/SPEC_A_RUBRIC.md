# Spec A — Defaults Completeness: Spec Quality Rubric

Grades the Spec A spec on 10 criteria. Target: A+ on every criterion
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

Spec A covers DT-2, DT-11, DT-13, and DT-14 per the umbrella. Empirical
verification of the CortenForge codebase revealed:

| Umbrella claim | CortenForge reality | Action |
|----------------|---------------------|--------|
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

## Empirical Ground Truth

### MuJoCo behavioral verification

All findings verified against MuJoCo 3.x C source on GitHub
(`google-deepmind/mujoco`). No local MuJoCo binary run for this rubric —
numerical expectations will be established in the spec itself by running
MuJoCo models or deriving from verified C source constants.

### Codebase context

#### CC-1: Files Spec A will touch

| File | Change | Lines affected |
|------|--------|---------------|
| `sim/L0/mjcf/src/types.rs` | Create `MjcfEqualityDefaults`; add `equality` to `MjcfDefault` (line 556); add cylinder/muscle/adhesion fields to `MjcfActuatorDefaults` (after line 726) | 556–577, 682–730 |
| `sim/L0/mjcf/src/parser.rs` | Add `b"equality"` to `parse_default()` match (line 515/548); add `b"cylinder"|b"muscle"|b"adhesion"|b"damper"|b"intvelocity"` to actuator shortcut match (line 522/555); create `parse_equality_defaults()` function; add cylinder/muscle/adhesion attrs to `parse_actuator_defaults()` | 515–579, 739+ |
| `sim/L0/mjcf/src/defaults.rs` | Add `equality_defaults()` accessor; add `apply_to_equality()` cascade; add `merge_equality_defaults()`; add equality to `merge_defaults()` (line 664); extend `merge_actuator_defaults()` with new fields (line 752); extend `apply_to_actuator()` with new fields (line 294) | 47–121, 294–411, 664–785 |
| `sim/L0/mjcf/src/builder/equality.rs` | Replace 5 hardcoded `unwrap_or(DEFAULT_SOLREF/DEFAULT_SOLIMP)` sites with defaults-cascaded values | 73–76, 136–137, 184–187, 236, 279 |
| `sim/L0/core/src/types/model.rs` | Add `qpos_spring: Vec<f64>` | ~186 |
| `sim/L0/core/src/types/model_init.rs` | Init `qpos_spring` as empty vec | ~93 |
| `sim/L0/mjcf/src/builder/build.rs` | Initialize `qpos_spring` after qpos0 is set; update tendon sentinel resolution to use `qpos_spring` instead of `qpos0` (line 491) | ~480–497 |
| `sim/L0/mjcf/src/builder/mod.rs` | Add `qpos_spring_values: Vec<f64>` accumulator to `ModelBuilder` (alongside existing `qpos0_values` at ~583) | ~430, ~583 |
| `sim/L0/mjcf/src/builder/joint.rs` | Populate `qpos_spring` entries during joint building | ~119, ~163–196 |
| `sim/L0/mjcf/src/builder/flex.rs` | Populate `qpos_spring` entries for flex vertex slide joints | ~197 |
| `sim/L0/core/src/forward/passive.rs` | Change `jnt_springref[jnt_id]` reads to `qpos_spring[qpos_adr]` for hinge/slide spring force | ~804 |
| `sim/L0/core/src/energy.rs` | Change `jnt_springref[jnt_id]` reads to `qpos_spring[qpos_adr]` for hinge/slide spring energy | ~46 |
| `sim/L0/mjcf/src/builder/actuator.rs` | Add `IntVelocity` arm to exhaustive matches on `MjcfActuatorType` (dyntype match at ~188, gaintype/biastype/gainprm/biasprm/dynprm match at ~321) — compile error without this | ~188, ~321 |
| `sim/L0/mjcf/src/builder/init.rs` | Init `qpos_spring_values: vec![]` on `ModelBuilder` (alongside `qpos0_values: vec![]` at ~170) — compile error without this | ~170 |
| `sim/L0/mjcf/src/lib.rs` | Export `MjcfEqualityDefaults` in public API (all other defaults types are exported at ~186–193) | ~186–193 |

**Note:** `builder/mod.rs` (listed above for `qpos_spring_values`) also needs
updating at ~295 to apply equality defaults before `process_equality_constraints()`
— currently the call at line 295 passes raw constraints without cascade.

`model_init.rs` (listed above for empty vec init) also has a second touch point:
`compute_implicit_params()` at line 823 reads `jnt_springref[jnt_id]` into
`implicit_springref[dof_adr]` — may need migration to `qpos_spring` (see CC-5).

#### CC-2: Exhaustive match sites in `parse_default()`

The `parse_default()` function (parser.rs:511–579) has two match blocks —
one for `Event::Start` (line 515) and one for `Event::Empty` (line 548).
Both must be updated identically. The `_ => skip_element()` and `_ => {}`
catch-all arms silently skip unknown element names.

Currently handled: `joint`, `geom`, `actuator|motor|position|velocity|general`,
`tendon`, `sensor`, `mesh`, `site`, `pair`, `default` (nested).

Missing: `equality`, `cylinder`, `muscle`, `adhesion`, `damper`, `intvelocity`.

#### CC-3: Hardcoded equality default sites in builder

Five equality constraint types in `builder/equality.rs` use hardcoded
`DEFAULT_SOLREF`/`DEFAULT_SOLIMP`:

| Type | Lines | Pattern |
|------|-------|---------|
| Connect | 73–76 | `connect.solimp.unwrap_or(DEFAULT_SOLIMP)` / `connect.solref.unwrap_or(DEFAULT_SOLREF)` |
| Weld | 136–137 | `weld.solimp.unwrap_or(DEFAULT_SOLIMP)` / `weld.solref.unwrap_or(DEFAULT_SOLREF)` |
| Joint | 184–187 | `joint_eq.solimp.unwrap_or(DEFAULT_SOLIMP)` / `joint_eq.solref.unwrap_or(DEFAULT_SOLREF)` |
| Distance | ~236 | Same pattern |
| Tendon | ~279 | Same pattern |

After Spec A, these should cascade: element value → class default → hardcoded default.

**Architectural decision required:** All 5 equality constraint structs have a
`class: Option<String>` field (MjcfConnect:1540, MjcfWeld:1632,
MjcfJointEquality:1732, MjcfDistance:1846, MjcfTendonEquality:1936), but the
builder **ignores** this field entirely. The cascade path must be: (a) parser
stores `class` on each constraint element, (b) builder calls
`resolver.apply_to_equality()` (new method) before reading solref/solimp,
following the same pattern as `resolver.apply_to_joint()`. The spec must
decide whether `apply_to_equality()` takes a single method for all 5 types
(since MuJoCo uses one `<equality>` default, not per-type defaults) or
whether each type gets its own cascade call.

#### CC-4: `merge_defaults()` and `merge_actuator_defaults()` match sites

`merge_defaults()` (defaults.rs:664) constructs `MjcfDefault` with explicit
field list. Adding `equality` requires a new field in the constructor and a
new `merge_equality_defaults()` function.

`merge_actuator_defaults()` (defaults.rs:752–785) enumerates every
`MjcfActuatorDefaults` field. New fields must be added to both the struct and
this merge function.

#### CC-5: `jnt_springref` consumer sites

Consumers that read `jnt_springref` and may need updating for `qpos_spring`:

| File | Line | Usage |
|------|------|-------|
| `forward/passive.rs` | 804 | Spring force: `springref = model.jnt_springref[jnt_id]` |
| `energy.rs` | 46 | Spring energy: `springref = model.jnt_springref[jnt_id]` |
| `types/model_init.rs` | 823 | `implicit_springref[dof_adr] = jnt_springref[jnt_id]` |
| `builder/build.rs` | 491 | Tendon sentinel: uses `qpos0` (should use `qpos_spring`) |
| `builder/flex.rs` | 197 | Flex vertices create slide joints: `self.jnt_springref.push(0.0)` — also needs `qpos_spring` population |

Note: `implicit_springref` in `model_init.rs` copies from `jnt_springref` for
implicit integrator use. This may need to read from `qpos_spring` instead,
but only for hinge/slide (it's already 0.0 for ball/free DOFs).

Note: `builder/flex.rs` creates 3 slide joints per flex vertex body (line 181–199).
Each pushes `jnt_springref = 0.0`. These joints also need `qpos_spring` entries
(scalar `0.0` per slide joint, matching `qpos0` for flex vertices).

#### CC-6: Test model factory sites that push `jnt_springref`

These manually-constructed Model instances (NOT via the MJCF builder) push
`jnt_springref` and will need corresponding `qpos_spring` population when
the field is added to Model:

| File | Line | Context |
|------|------|---------|
| `types/model_factories.rs` | 89 | `create_chain_model()` — hinge chain factory |
| `types/model_factories.rs` | 190 | `create_single_body_model()` — single body factory |
| `types/model_factories.rs` | 278 | `create_free_body_model()` — free body factory |
| `sensor/mod.rs` | 105 | Test setup in sensor module |
| `sensor/mod.rs` | 1020 | Test setup in sensor module |
| `sensor/mod.rs` | 1137 | Test setup in sensor module |
| `jacobian.rs` | 551, 713, 828, 940, 1175, 1307, 1457 | 7 test model constructions |
| `constraint/jacobian.rs` | 389, 477 | 2 test model constructions |
| `forward/muscle.rs` | 812, 1068, 1387, 1731, 1895, 2038 | 6 test model constructions |

**Total: ~18 manual model construction sites** that will need `qpos_spring`
added. These are all test code — the blast radius is confined to test setup,
not production code paths.

### EGT-1: Equality default fields in MuJoCo

**Source:** `OneEquality()` in `xml_native_reader.cc` line ~2097;
`mjs_defaultEquality()` in `user_init.c` line ~301.

MuJoCo equality defaults parse exactly **3 fields** in the defaults context
(when `readingdefaults == true`):

| Field | Type | Default | Parsed in defaults |
|-------|------|---------|-------------------|
| `active` | bool (int) | `1` (true) | Yes |
| `solref` | `[f64; mjNREF]` (2) | `[0.02, 1.0]` | Yes |
| `solimp` | `[f64; mjNIMP]` (5) | `[0.9, 0.95, 0.001, 0.5, 2.0]` | Yes |
| `type` | enum | `mjEQ_CONNECT` | Set from element name but irrelevant — `<default>` uses `<equality>` name |
| `data[*]` | f64 array | various | **No** — body/joint/geom targets only in non-defaults context |

The `<default><equality>` element name resolves to the generic `equality`
tag in the Default() dispatcher, not to a specific equality type (connect,
weld, etc.). The default applies uniformly to ALL equality constraint types
in the class.

Default values come from `mj_defaultSolRefImp()` in `engine_init.c`:
- `solref = [0.02, 1.0]`
- `solimp = [0.9, 0.95, 0.001, 0.5, 2.0]`

**CortenForge `active` parsing — code quality note (NOT a conformance bug):**
All 5 equality parsers use `active != "false"` (parser.rs:2331/2382/2423/2464/
2505), while 27 other boolean attributes use `== "true"`. MuJoCo's `bool_map`
only accepts `"true"`/`"false"` (not `"0"`/`"1"`) — passing `active="0"` to
MuJoCo produces an XML parse error, so no conformant MJCF file will contain it.
The `!= "false"` pattern is inconsistent with the rest of the parser but not a
conformance bug. The spec should standardize to `== "true"` for consistency
when touching these sites for the `Option<bool>` migration.

**CortenForge `active` type gap:** On all 5 equality constraint structs,
`active` is `bool` (not `Option<bool>`), defaulting to `true`. This prevents
defaults inheritance — if a `<default><equality active="false"/>` sets
`active=false`, a concrete constraint without `active` cannot inherit it
because the struct already has `active: true` from `Default::default()`. The
spec must change `active` to `Option<bool>` on equality structs to support
cascade, following the same pattern as `solref`/`solimp`.

### EGT-2: `qpos_spring` initialization and runtime

**Source:** `CopyTree()` in `user_model.cc` line ~2835; `mj_springdamper()`
in `engine_passive.c` line ~118; `setSpring()` in `engine_setconst.c`
line ~1152.

`qpos_spring` is a model array of size `nq` (same as `qpos0`), indexed by
`jnt_qposadr`:

| Joint type | `qpos_spring` initialization | Size at offset |
|------------|------------------------------|---------------|
| FREE | Copy from `qpos0[qposadr..qposadr+7]` | 7 (pos[3] + quat[4]) |
| BALL | Copy from `qpos0[qposadr..qposadr+4]` | 4 (quat[4]) |
| HINGE | `(mjtNum)pj->springref` | 1 |
| SLIDE | `(mjtNum)pj->springref` | 1 |

**Default value of `springref`:** `0.0` (from `memset` in `mjs_defaultJoint()`).

**Runtime usage in `mj_springdamper()` (engine_passive.c:118):**
- HINGE/SLIDE: `qfrc_spring[dadr] = -stiffness * (qpos[padr] - qpos_spring[padr])`
- BALL: `mji_subQuat(dif, quat, qpos_spring + padr)` → `qfrc_spring[dadr+i] = -stiffness * dif[i]`
- FREE: translational spring (3 DOF) + fall-through to BALL (3 rotational DOF)

**Key insight:** `qpos_spring` is indexed by `qposadr`, not by `jnt_id`.
CortenForge's `jnt_springref` is indexed by `jnt_id` and is scalar — it
cannot represent the quaternion reference needed for ball/free spring forces.

**Scope boundary:** Spec A creates the `qpos_spring` array with correct
initialization. Ball/free spring FORCE computation (the `mji_subQuat` path)
is Spec B §64 territory. But Spec A must provide the data that Spec B reads.

**`setSpring()` in `engine_setconst.c`:** Evaluates forward kinematics at
the `qpos_spring` configuration to compute tendon spring lengths when the
sentinel `[-1, -1]` is present. CortenForge currently uses `qpos0` for this
(build.rs:491 divergence note). Spec A should update this to use `qpos_spring`.

**Index space warning for tendon sentinel:** The current code at build.rs:483–486
indexes `qpos0` by `wrap_objid[w]` (which is a DOF address, velocity space).
For hinge/slide joints, `dof_adr == qpos_adr` (both are scalar), so this works.
For ball joints (`nq=4, nv=3`) or free joints (`nq=7, nv=6`), `dof_adr !=
qpos_adr`. If the tendon sentinel switch to `qpos_spring` preserves the
`dof_adr` index, it will be WRONG for ball/free tendons. The spec must either
(a) switch to `qpos_adr` indexing, or (b) document that tendon sentinel
resolution only supports hinge/slide joints (matching current behavior).

### EGT-3: Actuator type-specific defaults in MuJoCo

**Source:** `Default()` in `xml_native_reader.cc` line ~2884;
`OneActuator()` in `xml_native_reader.cc`.

**Critical design fact:** MuJoCo stores a **single** `mjsActuator*` per
default class. ALL shortcut element names (`general`, `motor`, `position`,
`velocity`, `damper`, `intvelocity`, `cylinder`, `muscle`, `adhesion`)
dispatch to the **same** `OneActuator(elem, def->actuator)`. This means:

1. There is NO per-type segregation in the default struct
2. If `<default>` contains both `<motor>` and `<position>`, the last one
   **overwrites** the first (they share one `mjsActuator` spec). **Scope
   clarification:** This "last wins" is a **parse-level** full replacement
   within a single `<default>` XML element (CortenForge: `default.actuator =
   Some(parse_actuator_defaults(e))` replaces the entire struct). At the
   **merge-level** (class inheritance), the merge is per-field via
   `c.field.or(p.field)`, so a child class overrides individual fields, not
   the whole struct.
3. Type-specific attributes (like `area` for cylinder, `force` for muscle)
   are stored in the SAME `mjsActuator` struct alongside common attributes

**Critical MuJoCo detail — shortcut expansion at parse time:** When MuJoCo
parses `<default><cylinder area="0.01"/>`, `OneActuator()` is called which:
(a) reads `area` as a local variable (defaulting to `actuator->gainprm[0]`),
(b) reads the `area="0.01"` attribute from XML, and (c) calls
`mjs_setToCylinder(actuator, timeconst, bias, area, diameter)` which sets
`gainprm[0]=0.01`, `dyntype=DYN_FILTER`, `gaintype=GAIN_FIXED`,
`biastype=BIAS_AFFINE` **on the default struct's actuator immediately**. This
means the default is fully "expanded" at parse time. CortenForge defers
expansion to build time — the spec must address this design tension and prove
the deferred expansion produces identical results.

**Shortcut attributes are NOT real fields on `mjsActuator`:** `area`,
`diameter`, `bias`, `timeconst`, `gain`, and muscle params are **local
variables** in `OneActuator()` that read from / write to `gainprm[]`,
`biasprm[]`, `dynprm[]`:

| Shortcut attribute | Stored in `mjsActuator` |
|-------------------|------------------------|
| `area` (cylinder) | `gainprm[0]` |
| `diameter` (cylinder) | `gainprm[0]` (computed: π/4 × d²) |
| `bias` (cylinder) | `biasprm[0]` |
| `timeconst` (cylinder) | `dynprm[0]` |
| `kp` (position) | `gainprm[0]` + `biasprm[1]` (negated) |
| `kv` (position/velocity) | `biasprm[2]` (negated) or `gainprm[0]` |
| `gain` (adhesion) | `gainprm[0]` |
| Muscle fields | `gainprm[0..8]`, `biasprm[0..8]`, `dynprm[0..2]` |

CortenForge stores these as real fields on `MjcfActuator` (for ergonomic
parsing), which is a valid design choice. But the spec must document this
MuJoCo → CortenForge convention difference explicitly.

**Muscle sentinel detection (conformance-critical):** `mjs_setToMuscle()`
in `user_api.cc` uses **equality checks against global default values** as
sentinels: `if (actuator->gainprm[0] == 1) actuator->gainprm[0] = 0.75;`,
`if (actuator->dynprm[0] == 1) actuator->dynprm[0] = 0.01;`, etc. This
means explicitly setting `gainprm[0]=1` is indistinguishable from "not set"
and will be overwritten by muscle defaults. CortenForge must replicate this
behavior for conformance, even though it's arguably a MuJoCo design quirk.

**CortenForge gap analysis:**

Parser match at `parse_default()` (parser.rs:522):
```
b"actuator" | b"motor" | b"position" | b"velocity" | b"general"
```
**Missing:** `b"cylinder"`, `b"muscle"`, `b"adhesion"`, `b"damper"`,
`b"intvelocity"`. These fall through to `_ => skip_element()` / `_ => {}`
and are silently ignored. Additionally, `IntVelocity` is missing from the
`MjcfActuatorType` enum entirely (types.rs:2292) — not just the parser match.

`MjcfActuatorDefaults` (types.rs:682–730):
**Present:** ctrlrange, forcerange, gear, kp, kv, dampratio, ctrllimited,
forcelimited, gaintype, biastype, dyntype, gainprm, biasprm, dynprm, group,
actlimited, actrange, actearly, lengthrange, nsample, interp, delay.
**Missing:** area, diameter, timeconst, bias, muscle_timeconst, range, force,
scale, lmin, lmax, vmax, fpmax, fvmax, gain (CortenForge-specific ergonomic
fields — in MuJoCo these map to gainprm/biasprm/dynprm arrays).

These fields exist on `MjcfActuator` (types.rs:2419–2453) but not on
`MjcfActuatorDefaults`. They need to be added to the defaults struct,
parser, merge function, and apply function.

**Non-Option type challenge:** On `MjcfActuator`, these fields use non-Option
types with sentinel/default values:

| Field | Type on `MjcfActuator` | Default value | Option wrapping needed |
|-------|----------------------|---------------|----------------------|
| `area` | `f64` | `1.0` | Yes — `1.0` is both valid and default |
| `bias` | `[f64; 3]` | `[0, 0, 0]` | Yes — `[0,0,0]` is both valid and default |
| `muscle_timeconst` | `(f64, f64)` | `(0.01, 0.04)` | Yes |
| `range` | `(f64, f64)` | `(0.75, 1.05)` | Yes |
| `force` | `f64` | `-1.0` | Yes — `-1.0` is sentinel for auto-compute |
| `scale` | `f64` | `200.0` | Yes |
| `lmin` | `f64` | `0.5` | Yes |
| `lmax` | `f64` | `1.6` | Yes |
| `vmax` | `f64` | `1.5` | Yes |
| `fpmax` | `f64` | `1.3` | Yes |
| `fvmax` | `f64` | `1.2` | Yes |
| `gain` | `f64` | `1.0` | Yes — `1.0` is both valid and default |

The defaults struct must use `Option<T>` for all of these (to distinguish
"not set in defaults" from "set to its default value"). The `apply_to_actuator()`
function must check `is_none()` or use sentinel detection for non-Option
element fields. This is the same design pattern as the existing `kp` sentinel
at defaults.rs:341, which the codebase already flags as problematic (comment
at defaults.rs:298–301).

### EGT-4: DT-11 (Joint `range`) already implemented

**Verification trail:**

1. **Struct:** `MjcfJointDefaults.range: Option<(f64, f64)>` — types.rs:606 ✓
2. **Parse:** `parse_joint_defaults()` reads `range` attribute — parser.rs:611 ✓
3. **Cascade:** `apply_to_joint()` cascades `range` — defaults.rs:169 ✓
4. **Merge:** `merge_joint_defaults()` merges `range` — defaults.rs:702 ✓
5. **Builder:** `process_joint()` calls `resolver.apply_to_joint()` — builder/joint.rs:265 ✓

Full parse → store → merge → cascade → builder pipeline is complete.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and line range: `OneEquality()` in `xml_native_reader.cc`, `mjs_defaultEquality()` in `user_init.c`, `mj_defaultSolRefImp()` in `engine_init.c`, `CopyTree()` in `user_model.cc` (qpos_spring init), `mj_springdamper()` in `engine_passive.c` (qpos_spring usage), `setSpring()` in `engine_setconst.c` (tendon sentinel resolution), `Default()` in `xml_native_reader.cc` (actuator dispatch), `OneActuator()` in `xml_native_reader.cc` (shortcut expansion), `mjs_setToCylinder()`/`mjs_setToMuscle()` in `user_api.cc` (shortcut expansion functions). Edge cases addressed: (1) equality defaults apply uniformly to all 5 constraint types, not per-type, (2) `qpos_spring` for ball is identity quaternion from `qpos0`, not from `springref`, (3) `qpos_spring` for free is 7D from `qpos0`, (4) multiple actuator shortcuts in one `<default>` — last one wins, (5) `springref` default is `0.0` not `NaN`, (6) `active` is a parseable equality default (not just solref/solimp), (7) muscle sentinel detection — `mjs_setToMuscle()` treats `gainprm[0]==1` as "not set" and overwrites with `0.75`; spec must address this conformance quirk, (8) shortcut attributes (`area`, `diameter`, `bias`, `timeconst`, `gain`) are local variables in `OneActuator()` mapping to `gainprm[]`/`biasprm[]`/`dynprm[]` — not real fields on `mjsActuator`, (9) MuJoCo `bool_map` only accepts `"true"`/`"false"` (via `MapValue()` + `FindKey()` in `xml_util.cc`) — NOT `"0"`/`"1"` (those produce an XML parse error); spec must document this convention, (10) for each NEW actuator shortcut type (`cylinder`, `muscle`, `adhesion`, `damper`, `intvelocity`), the spec must cite the MuJoCo `mjs_setTo*()` function and specify the exact `gaintype`/`biastype`/`dyntype`/`gainprm`/`biasprm`/`dynprm` expansion values — not just "add to parser match" (e.g., `intvelocity`: `gaintype=FIXED`, `biastype=AFFINE`, `dyntype=INTEGRATOR`, `gainprm[0]=kv`, `biasprm[1]=-kv`). C code snippets included for `qpos_spring` initialization switch statement and `mj_springdamper()` per-joint-type dispatch. |
| **A** | Functions cited correctly. Minor edge-case gaps (e.g., missing ball/free qpos_spring initialization detail). |
| **B** | Functions cited but behavior described from docs not C source, or edge cases omitted. |
| **C** | MuJoCo behavior assumed or hand-waved. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps.

| Grade | Bar |
|-------|-----|
| **A+** | For DT-2: exact struct definition, parse logic, cascade method (`apply_to_equality()`), and builder integration code written out. For DT-13: `qpos_spring` initialization algorithm specified per joint type with qpos_adr indexing. Tendon sentinel resolution update. Runtime consumer updates (passive.rs, energy.rs). For DT-14: parser match extension, new defaults fields, merge function extension, apply function extension — all fields enumerated. An implementer can type it all without reading MuJoCo source. |
| **A** | Algorithm is complete. One or two minor details left implicit. |
| **B** | Algorithm structure clear but some cascade steps hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses MuJoCo → CortenForge convention translations.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table covers: (1) `qpos_spring` indexed by `qposadr` (MuJoCo) → same in CortenForge, NOT by `jnt_id`, (2) equality defaults use same struct pattern as existing defaults (not a new pattern), (3) actuator shortcut dispatch writes to single `MjcfActuatorDefaults` (matching MuJoCo's single `mjsActuator`), (4) `jnt_springref` (CortenForge, per-joint scalar) vs `qpos_spring` (MuJoCo, per-qpos vector) naming and indexing difference, (5) `solimp` is `[f64; 5]` in CortenForge (matching `mjNIMP=5`), `solref` is `[f64; 2]` (matching `mjNREF=2`). |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others risk silent mismatch. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete MJCF model, (2) expected value from MuJoCo or derivation, (3) field to check. DT-2 AC: equality constraint with `<default><equality solref="0.05 0.8"/>` → `eq_solref[0] == [0.05, 0.8]`. DT-13 AC: hinge joint with `springref="0.5"` → `model.qpos_spring[qpos_adr] == 0.5`; ball joint → `model.qpos_spring[qpos_adr..+4] == [1,0,0,0]`. DT-14 AC: `<default><cylinder area="0.01"/>` → actuator with class inherits `area=0.01`. Code-review ACs explicitly labeled. At least one MuJoCo-verified expected value per task. |
| **A** | ACs testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: (1) equality defaults with nested class inheritance (parent sets solref, child overrides solimp), (2) equality constraint with NO class → root default applies, (3) `qpos_spring` for ball joint (quaternion values — must come from `qpos0`, NOT from scalar `springref`; **scope limitation**: ball joints always start at identity quaternion `[1,0,0,0]` in MJCF, so this test cannot distinguish "copy from qpos0" vs "hardcode identity" within Spec A — the invariant is verified structurally at code review, not numerically; full verification deferred to Spec B when ball spring forces consume the quaternion), (4) `qpos_spring` for free joint with non-identity initial quaternion (7D values must copy actual `qpos0`, not identity — this IS numerically testable because free joint qpos0 includes world position/orientation), (5) `qpos_spring` for hinge with explicit springref vs default springref=0, (6) multiple actuator shortcuts in one `<default>` class (last wins), (7) `<default><cylinder area="0.01"/>` followed by `<cylinder class="...">` inherits area, (8) `<default><muscle>` with muscle-specific params cascades correctly, (9) muscle sentinel detection: in MuJoCo, `mjs_setToMuscle()` checks `gainprm[0]==1` (the `mjsActuator` default) and overwrites with `0.75`. **Target clarification:** CortenForge's `<muscle>` shortcut builds `gainprm` from `actuator.range.0` (not from `gainprm` directly), so this sentinel only manifests for `<general dyntype="muscle" gainprm="1">` → spec must clarify whether to replicate this MuJoCo quirk or document it as a known divergence for the `<general>` path, (10) equality `active` defaults cascade: `<default><equality active="false"/>` with concrete constraint having no `active` attr → must inherit `false`, (11) **tendon sentinel with non-zero springref**: tendon with `springlength="-1"` (sentinel) wrapping joints with `springref != 0` — resolved length must use `qpos_spring` not `qpos0` (this is the ONLY test that proves the `qpos0→qpos_spring` switch matters), (12) mixed joint types model (hinge+slide+ball+free in one model) — `qpos_spring` entries correctly aligned with `qpos0` by `jnt_qpos_adr`. Negative test: `<default><equality>` does NOT cascade data/anchor/polycoef attributes. At least one MuJoCo conformance test per task. At least one multi-element model test. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. DT-2, DT-13, DT-14 sections can be implemented in any order (no data dependencies between them). Each section states which files it modifies and which prior sections (if any) it depends on. Cross-spec interaction documented: Spec B §64 will consume `qpos_spring` for ball/free spring energy — Spec A provides the data, Spec B provides the computation. No circular dependencies. |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior change, every test impact.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (see CC-1). Behavioral changes: (1) equality constraints now inherit from default classes — moves TOWARD MuJoCo conformance, (2) `qpos_spring` replaces `jnt_springref` as spring reference source — moves TOWARD conformance (though hinge/slide values are numerically identical when springref=0), (3) tendon sentinel resolution uses `qpos_spring` instead of `qpos0` — moves TOWARD conformance (identical when springref=0). Existing test impact: passive force tests read `jnt_springref` — values unchanged for hinge/slide but source field changes. Energy tests use `jnt_springref` — same. `implicit_springref` init may need update. Data staleness guards: `merge_defaults()` explicit field constructor, `merge_actuator_defaults()` explicit field constructor — both must be updated. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform (e.g., `qpos_spring` always means the model array, `springref` always means the MJCF attribute). Cross-references accurate. File lists match between Specification and Files Affected. AC numbers match Test Plan traceability. Edge cases listed in MuJoCo Reference appear in Test Plan. Consumer counts consistent. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Default Cascade Completeness

> For each task (DT-2, DT-14), the full 5-stage default cascade pipeline is
> complete: **parse** → **store** (struct field) → **merge** (inheritance) →
> **apply** (cascade to element) → **builder** (consume cascaded value).
> Missing any stage is a silent conformance bug — the attribute parses but
> doesn't affect the compiled model.

*Boundary with P2:* P2 grades whether the algorithm is specified unambiguously.
P9 grades whether the cascade pipeline is complete end-to-end — every new
default field traverses all 5 stages, and no stage is accidentally skipped.

| Grade | Bar |
|-------|-----|
| **A+** | For DT-2: all 5 stages explicitly specified for each equality default field (`active`, `solref`, `solimp`). Parse function creates `MjcfEqualityDefaults` → field stored on struct → `merge_equality_defaults()` handles inheritance → `apply_to_equality()` cascades to element → builder reads cascaded values instead of hardcoded constants. DT-2 cascade architecture specified: how does `apply_to_equality()` look up the class? (equality types have `class: Option<String>` on their element structs — builder currently ignores it). **Integration call site required:** the spec must specify where `apply_to_equality()` is called in the builder pipeline — currently `builder/mod.rs:295` calls `process_equality_constraints()` without applying defaults first; the spec must add the cascade call at this site, following the same pattern as `builder.resolver.apply_to_actuator(actuator)` at line 290. **`active` type migration:** the spec must change `active` from `bool` to `Option<bool>` on all 5 equality structs (currently `bool` with default `true` prevents cascade inheritance). When touching these parse sites, standardize from `!= "false"` to `== "true"` for consistency with the other 27 boolean attributes in the parser (note: MuJoCo's `bool_map` only accepts `"true"`/`"false"`, not `"0"`/`"1"`, so this is a code quality fix, not a conformance fix). For DT-14: all 5 stages for each new actuator default field (area, diameter, timeconst, bias, muscle_timeconst, range, force, scale, lmin, lmax, vmax, fpmax, fvmax, gain). Each stage names the exact function and the exact field. No field is parsed but not merged, or merged but not applied. **Type wrapping design decision addressed:** On `MjcfActuator`, type-specific fields use non-Option types with sentinel defaults (`area: f64` default 1.0, `bias: [f64; 3]` default [0,0,0], `muscle_timeconst: (f64, f64)` default (0.01, 0.04)). On `MjcfActuatorDefaults`, these must use `Option<T>` to distinguish "not set" from "set to default value" — same sentinel-vs-Option problem as existing `kp` (defaults.rs:341). The spec must state the wrapping strategy for each field. |
| **A** | Pipeline stages present for most fields. One or two fields skip a stage. |
| **B** | Pipeline described at high level but individual fields not traced through all 5 stages. |
| **C** | Pipeline not analyzed. |

### P10. Data Lifecycle Correctness

> For DT-13 (`qpos_spring`), the model array has a specific initialization
> lifecycle that differs by joint type. The spec must get the initialization,
> population, and consumer-update lifecycle correct.

*Boundary with P1:* P1 grades whether the MuJoCo reference for `qpos_spring`
is accurately described. P10 grades whether the CortenForge data lifecycle
(allocation, initialization, population during build, consumer migration) is
specified completely and correctly — the Rust-side engineering.

| Grade | Bar |
|-------|-----|
| **A+** | Lifecycle specified: (1) `model.rs` — field declaration with doc comment, (2) `model_init.rs` — empty vec init, (3) `ModelBuilder` — `qpos_spring_values: Vec<f64>` accumulator field (following same pattern as existing `qpos0_values: Vec<f64>` at mod.rs:~583), (4) `builder/joint.rs` — per-joint population using `self.nq` offset and joint-type switch (hinge/slide: springref value, ball: copy qpos0 quaternion, free: copy qpos0 7D). **Note:** Ball/free quaternion/7D population is entirely NEW logic — no existing code to modify, only new code to write alongside the existing hinge/slide path, (5) `builder/flex.rs` — flex vertex slide joints also populate `qpos_spring` (scalar 0.0 per slide joint), (6) `build.rs` — converts accumulator to `DVector` (like qpos0 at line 347); uses `qpos_spring` for tendon sentinel resolution instead of `qpos0`, (7) `passive.rs` — spring force reads `qpos_spring[qpos_adr]` instead of `jnt_springref[jnt_id]`, (8) `energy.rs` — spring energy reads `qpos_spring[qpos_adr]`, (9) `model_init.rs` — `implicit_springref` init reads from `qpos_spring`. **Indexing note:** `implicit_springref` is sized `nv` (velocity DOFs, line 812), while `qpos_spring` is sized `nq` (position coords). Step 9 requires `jnt_qpos_adr`/`jnt_dof_adr` mapping to convert between these index spaces — the spec must specify this mapping explicitly. Allocation order: `qpos_spring` must be sized `nq` and populated AFTER `qpos0` is set. `jnt_springref` co-existence explicitly addressed: MuJoCo has no `jnt_springref` on `mjModel` (only `qpos_spring`); spec must state whether `jnt_springref` is kept (builder intermediate + backward compat) or removed, and list migration path for ~33 consumer sites. |
| **A** | Lifecycle mostly complete. One consumer site missed. |
| **B** | Lifecycle described but missing builder population or consumer migration details. |
| **C** | Lifecycle not analyzed. |

---

## Rubric Self-Audit

- [x] **Specificity:** Every A+ bar names specific functions (`OneEquality()`,
      `CopyTree()`, `merge_actuator_defaults()`, `mjs_setToCylinder()`,
      `mjs_setToMuscle()`, `mjs_setToIntVelocity()`), specific file:line
      ranges (parser.rs:522, defaults.rs:752, builder/actuator.rs:188/321,
      builder/mod.rs:295), and specific field names. Rev 6: P1 has 10
      edge cases (+per-shortcut expansion values); P5 has 12 edge cases
      with scope limitations documented (ball qpos_spring unfalsifiable);
      P9 requires integration call site; EGT-2 warns about dof_adr vs
      qpos_adr index spaces. Two independent reviewers would agree on
      the grade.

- [x] **Non-overlap:** P1 vs P9: P1 grades MuJoCo reference accuracy; P9
      grades cascade pipeline completeness in CortenForge code. P1 vs P10:
      P1 grades MuJoCo qpos_spring behavior; P10 grades CortenForge lifecycle
      engineering. P2 vs P9: P2 grades algorithm unambiguity; P9 grades that
      no pipeline stage is skipped. Each criterion has a unique grading angle.

- [x] **Completeness:** 10 criteria cover: MuJoCo reference (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), cascade pipeline (P9), data lifecycle
      (P10). The three tasks (DT-2, DT-13, DT-14) each have at least one
      domain criterion addressing their unique challenge (DT-2/DT-14: cascade
      pipeline; DT-13: data lifecycle).

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors. P2 →
      Specification sections. P3 → Convention Notes. P4 → Acceptance Criteria.
      P5 → Test Plan + Traceability. P6 → Prerequisites + Execution Order.
      P7 → Blast Radius + Files Affected. P8 → cross-cutting. P9 →
      Specification (cascade stages per field). P10 → Specification (qpos_spring
      lifecycle).

- [x] **Conformance primacy:** P1 is tailored with 10 specific MuJoCo C
      functions, 10 edge cases (Rev 6: +per-shortcut expansion values),
      and C code snippet requirements. P4 requires MuJoCo-verified expected
      values. P5 requires 12 edge-case tests with scope limitations
      honestly documented (ball qpos_spring unfalsifiable within Spec A).
      P9 requires integration call site specification. Adversarial review
      (round 5) found no unmitigated exploits that would allow a conformance
      divergence to pass all criteria.

- [x] **Empirical grounding:** EGT-1 through EGT-4 filled in with verified
      C source citations and CortenForge codebase context. CC-1 through CC-6
      enumerate every file, function, and match site. Rev 6 adds: tendon
      sentinel index space warning (dof_adr vs qpos_adr) in EGT-2; per-
      shortcut expansion values in P1; integration call site in P9; ball
      qpos_spring scope limitation in P5; muscle sentinel target clarification.
      Adversarial review found remaining exploits are either (a) scope
      boundary issues honestly documented, or (b) guarded by Rust compiler.
      No criterion bar was written from header-file assumptions.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1: DT-2, S2: DT-13, S3: DT-14) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification (all 5 cascade stages per DT-2/DT-14 field) |
| P10 | Specification (qpos_spring lifecycle: declare → init → populate → consume) |

---

## Scorecard

This scorecard grades **the rubric itself**, not the spec. Grade as "N/A"
since the spec has not been written yet — this section will be used when
grading the spec in Session 4.

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | — | *Awaiting spec* |
| P2. Algorithm Completeness | — | *Awaiting spec* |
| P3. Convention Awareness | — | *Awaiting spec* |
| P4. Acceptance Criteria Rigor | — | *Awaiting spec* |
| P5. Test Plan Coverage | — | *Awaiting spec* |
| P6. Dependency Clarity | — | *Awaiting spec* |
| P7. Blast Radius & Risk | — | *Awaiting spec* |
| P8. Internal Consistency | — | *Awaiting spec* |
| P9. Default Cascade Completeness | — | *Awaiting spec* |
| P10. Data Lifecycle Correctness | — | *Awaiting spec* |

**Overall: Rubric Rev 6 — quintuple stress-tested (including adversarial review), converged**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | DT-11 listed in umbrella but already fully implemented in CortenForge | Codebase exploration (EGT-4) | Dropped from spec scope; verify in review phase | Rubric Rev 1 |
| R2 | P1 | Need `active` field in equality defaults — not just solref/solimp | MuJoCo C source (`OneEquality()` parses `active` unconditionally) | Added `active` to EGT-1 field table and P1 edge case list | Rubric Rev 1 |
| R3 | P9 | Initial draft had no domain criterion for cascade pipeline completeness | Template Step 5 (signal: "Multiple call sites must be updated") | Added P9 (Default Cascade Completeness) | Rubric Rev 1 |
| R4 | P10 | Initial draft had no criterion for qpos_spring data lifecycle | Template Step 5 (signal: "New fields need init/reset/populate lifecycle") | Added P10 (Data Lifecycle Correctness) | Rubric Rev 1 |
| R5 | CC-2 | Parser `parse_default()` has TWO match blocks (Start + Empty) that must both be updated | Codebase read (parser.rs:515 vs 548) | Documented in CC-2 as dual-match-site risk | Rubric Rev 1 |
| R6 | EGT-3 | Multiple actuator shortcuts in one `<default>` — last one wins (overwrite, not merge) | MuJoCo C source (`Default()` calls `OneActuator()` on same `def->actuator`) | Added as edge case in EGT-3 and P5 | Rubric Rev 1 |
| R7 | CC-5 | `implicit_springref` in model_init.rs:823 copies from `jnt_springref` — may need qpos_spring update | Grep for `jnt_springref` consumers | Added to CC-5 consumer table and P10 lifecycle | Rubric Rev 1 |
| R8 | P7 | Tendon sentinel resolution in build.rs:491 uses `qpos0` — should use `qpos_spring` per MuJoCo's `setSpring()` | MuJoCo C source (EGT-2) | Added to P7 blast radius bar and P10 lifecycle | Rubric Rev 1 |
| R9 | P10 | `jnt_springref` co-existence: MuJoCo has NO `jnt_springref` on `mjModel` — only `qpos_spring`. CortenForge has `jnt_springref` with ~33 consumer sites (3 production reads + ~21 test factory pushes + ~5 test assertions + ~4 builder writes). Spec must explicitly decide migration strategy (keep both, or deprecate `jnt_springref` to builder-only) | Grep for `jnt_springref` (CC-5) + MuJoCo `mjModel` field list | Added to P10 bar: spec must address `jnt_springref` co-existence and migration plan | Rubric Rev 1 (count corrected Rev 5) |
| R10 | P5 | Equality active defaults: need test that `active=false` in `<default><equality>` cascades to constraints — not just solref/solimp | EGT-1 (active is a parseable default field) | Added to P5 edge case (10) as explicit `active` cascade test | Rubric Rev 1 (made explicit Rev 5) |
| R11 | P9 | Equality class lookup: all 5 equality types have `class: Option<String>` (MjcfConnect:1540, MjcfWeld:1632, etc.) but builder ignores it entirely. Spec must specify how `apply_to_equality()` resolves class and which resolver method it calls. | Stress-test: builder/equality.rs field audit | Added to CC-3 as architectural decision; added to P9 A+ bar | Rubric Rev 2 |
| R12 | P10 | Flex joint qpos_spring: `builder/flex.rs:197` creates slide joints that push `jnt_springref = 0.0`. These also need `qpos_spring` population. | Stress-test: flex.rs code read | Added flex.rs to CC-1 files table, CC-5 consumer table, and P10 A+ bar | Rubric Rev 2 |
| R13 | P9 | Non-Option type fields: `MjcfActuator` uses `f64`/`[f64; 3]`/`(f64, f64)` for type-specific attrs (area, bias, muscle_timeconst, etc.) with sentinel defaults. `MjcfActuatorDefaults` must use `Option<T>` wrappers. Apply function needs sentinel detection or element field migration to Option. | Stress-test: types.rs:2481–2538 default values audit | Added non-Option field table to EGT-3; added wrapping requirement to P9 A+ bar | Rubric Rev 2 |
| R14 | P10 | qpos_spring accumulator: ModelBuilder uses `qpos0_values: Vec<f64>` accumulator (mod.rs:~583) → `DVector` in build.rs:347. `qpos_spring` needs analogous `qpos_spring_values: Vec<f64>` on ModelBuilder. | Stress-test: builder/mod.rs struct audit | Added to CC-1 (mod.rs entry) and P10 A+ bar (item 3) | Rubric Rev 2 |
| R15 | P7 | ~18 manual Model construction sites in test code push `jnt_springref` and will need `qpos_spring` — model_factories.rs (3), sensor/mod.rs (3), jacobian.rs (7), constraint/jacobian.rs (2), forward/muscle.rs (6) | Stress-test: grep for `jnt_springref.push` | Added CC-6 documenting all 18 sites; P7 blast radius includes test factory impact | Rubric Rev 2 |
| R16 | P3 | Shortcut attributes are NOT real fields on `mjsActuator`: `area`, `diameter`, `bias`, `timeconst`, `gain`, and muscle params are **local variables** in `OneActuator()` that map to `gainprm[]`/`biasprm[]`/`dynprm[]`. CortenForge stores them as real fields on `MjcfActuator` (ergonomic design choice). Spec must document this convention difference explicitly. | Stress-test round 2: MuJoCo C source `OneActuator()` | Added mapping table to EGT-3; P3 convention table must include this | Rubric Rev 3 |
| R17 | P1 | MuJoCo expands actuator shortcuts at parse time: `mjs_setToCylinder()` sets `dyntype`/`gaintype`/`biastype` on the default struct immediately. CortenForge defers expansion to build time. Spec must prove deferred expansion produces identical results or explicitly adopt parse-time expansion. | Stress-test round 2: MuJoCo C source `mjs_setToCylinder()` | Added design tension to EGT-3; P1 must require spec to address this | Rubric Rev 3 |
| R18 | P1, P5 | Muscle sentinel detection: `mjs_setToMuscle()` uses equality checks against global defaults as sentinels (`gainprm[0]==1` → replace with `0.75`). Explicitly setting `gainprm[0]=1` is indistinguishable from "not set". CortenForge must replicate this for conformance. | Stress-test round 2: MuJoCo C source `mjs_setToMuscle()` in `user_api.cc` | Added to EGT-3 conformance note; P1 edge case and P5 test added | Rubric Rev 3 |
| R19 | P1 | ~~**FALSE ALARM (corrected Rev 5):**~~ Originally claimed `active="0"` was a conformance bug. Verification against MuJoCo C source (`bool_map` in `xml_native_reader.cc` via `MapValue()` + `FindKey()`) shows MuJoCo only accepts `"true"`/`"false"` — `"0"`/`"1"` produce an XML parse error. The `!= "false"` pattern is a **code quality inconsistency** (27 other boolean attrs use `== "true"`) but NOT a conformance bug. | Stress-test round 3 (initially); corrected by round 4 MuJoCo `bool_map` verification | Downgraded from conformance-critical to code quality note in EGT-1; P1 edge case (9) corrected; P9 bar updated | Rubric Rev 4 → corrected Rev 5 |
| R20 | P9 | **`active` type on equality structs is `bool` not `Option<bool>`:** All 5 equality constraint structs use `active: bool` (default `true`). Cannot represent "unspecified" → prevents defaults cascade. Must change to `Option<bool>` for DT-2. | Stress-test round 3: types.rs equality struct audit | Added to EGT-1 as type gap; added to P9 A+ bar as migration requirement | Rubric Rev 4 |
| R21 | P9, EGT-3 | **`IntVelocity` missing from `MjcfActuatorType` enum** (types.rs:2292): Not just missing from parser match — the type variant itself is absent. Spec must add the variant. | Stress-test round 3: P3 convention enum audit | Added to EGT-3 gap analysis | Rubric Rev 4 |
| R22 | EGT-3 | **"Last shortcut wins" scope clarification:** Parse-level is full replacement (`default.actuator = Some(...)`) within a single `<default>` element. Merge-level is per-field `Option::or()` (child overrides individual fields, not whole struct). | Stress-test round 3: P9 cascade audit of merge semantics | Added clarification to EGT-3 point (2) | Rubric Rev 4 |
| R23 | P10 | **`implicit_springref` is sized `nv` (velocity DOFs), `qpos_spring` is sized `nq` (position coords).** Step 9 of lifecycle requires `jnt_qpos_adr`/`jnt_dof_adr` mapping between index spaces. | Stress-test round 3: model_init.rs:812 sizing audit | Added indexing note to P10 A+ bar step 9 | Rubric Rev 4 |
| R24 | P10 | **Ball/Free qpos_spring population is entirely NEW logic** — no existing code to modify in builder/joint.rs for quaternion/7D expansion. Existing `jnt_springref.push()` only pushes a scalar. | Stress-test round 3: builder/joint.rs:118-123 audit | Added note to P10 A+ bar step 4 | Rubric Rev 4 |
| R25 | CC-1 | **`builder/actuator.rs` missing from file table:** Exhaustive match on `MjcfActuatorType` at lines ~188 (dyntype) and ~321 (gaintype/biastype/gainprm/biasprm/dynprm) will fail to compile when `IntVelocity` variant is added. | Stress-test round 4: CC-1 completeness audit | Added to CC-1 file table | Rubric Rev 5 |
| R26 | CC-1 | **`builder/init.rs` missing from file table:** Must initialize `qpos_spring_values: vec![]` on `ModelBuilder` — compile error without it. | Stress-test round 4: CC-1 completeness audit | Added to CC-1 file table | Rubric Rev 5 |
| R27 | CC-1 | **`builder/mod.rs` entry incomplete:** CC-1 only mentions `qpos_spring_values` accumulator. Also needs update at line ~295 to apply equality defaults before `process_equality_constraints()`. | Stress-test round 4: CC-1 completeness audit | Added note below CC-1 table | Rubric Rev 5 |
| R28 | CC-1 | **`lib.rs` missing from file table:** Must export `MjcfEqualityDefaults` in public API to match all other defaults type exports at lines ~186–193. | Stress-test round 4: CC-1 completeness audit | Added to CC-1 file table | Rubric Rev 5 |
| R29 | P8 | **`jnt_springref` consumer count mismatch:** R9 originally said "35+" but CC-5+CC-6 document ~23 sites. Actual count ~33 (including test assertions and builder writes). | Stress-test round 4: P8 internal consistency audit | R9 count corrected to "~33" with breakdown | Rubric Rev 5 |
| R30 | P5 | **`active` cascade had no explicit numbered test in P5.** R10 said "implicitly via P1 edge case (6)" but P8 bar requires explicit correspondence. | Stress-test round 4: P8 cross-reference audit | Added P5 edge case (10): explicit `active` defaults cascade test | Rubric Rev 5 |
| R31 | P5 | **Tendon sentinel with non-zero springref — missing test.** This is the ONLY test that proves the `qpos0→qpos_spring` switch matters. Without it, the implementation could use either source and pass all tests. | Stress-test round 4: P5 edge case audit | Added P5 edge case (11): tendon sentinel resolution with `springref != 0` | Rubric Rev 5 |
| R32 | P5 | **Mixed joint types model — missing integration test.** Model with hinge+slide+ball+free verifying `qpos_spring` entries correctly aligned with `qpos0` by `jnt_qpos_adr`. | Stress-test round 4: P5 edge case audit | Added P5 edge case (12) | Rubric Rev 5 |
| R33 | P5 | **Ball joint ignores scalar springref for qpos_spring** — must use `qpos0` quaternion instead. Not explicitly tested. | Stress-test round 4: P5 edge case audit | Folded into P5 edge case (3) with explicit "NOT from scalar springref" wording | Rubric Rev 5 |
| R34 | P5 | **Free joint non-identity initial quaternion** — `qpos_spring` must copy actual rotated `qpos0`, not identity. | Stress-test round 4: P5 edge case audit | Folded into P5 edge case (4) with explicit "non-identity" wording | Rubric Rev 5 |
| R35 | P5 | **Ball qpos_spring unfalsifiable in Spec A scope:** Ball joints always have identity quaternion `qpos0=[1,0,0,0]` in MJCF — cannot numerically distinguish "copy from qpos0" vs "hardcode identity". Full verification deferred to Spec B. | Adversarial review round 5: scope boundary exploit | Added scope limitation note to P5 edge case (3); test is structural (code review), not numerical | Rubric Rev 6 |
| R36 | P9 | **`apply_to_equality()` call site not graded:** A perfect cascade implementation that is never invoked by the builder. P9 grades pipeline stages but not the integration call site at `builder/mod.rs:295`. | Adversarial review round 5: V11 | Added explicit integration call site requirement to P9 A+ bar | Rubric Rev 6 |
| R37 | EGT-2 | **Tendon sentinel index space confusion:** `build.rs:483-486` indexes by `dof_adr` (velocity space). For hinge/slide, `dof_adr==qpos_adr`. For ball/free, `dof_adr!=qpos_adr`. Switching to `qpos_spring` with wrong index is silently incorrect. | Adversarial review round 5: V13 | Added index space warning to EGT-2 | Rubric Rev 6 |
| R38 | P1 | **`IntVelocity` expansion values not graded:** P1 required "add to parser match" but not the exact `gaintype`/`biastype`/`dyntype`/`gainprm`/`biasprm` values for each new shortcut. | Adversarial review round 5: V14 | Added P1 edge case (10): per-shortcut expansion values required | Rubric Rev 6 |
| R39 | P5 | **Muscle sentinel target ambiguity:** Edge case (9) cited `<muscle>` shortcut but CortenForge's muscle path uses `actuator.range`, not `gainprm`. Sentinel only manifests for `<general dyntype="muscle">`. | Adversarial review round 5: P5 testability audit | Clarified edge case (9) target to `<general dyntype="muscle">` path | Rubric Rev 6 |
| R40 | P6 | **"Any order" claim is semantically correct but merge-conflict prone:** DT-2 and DT-14 touch adjacent lines in `parse_default()` and `merge_defaults()`. No data dependency, but textual conflict risk if implemented on separate branches simultaneously. | Adversarial review round 5: V20 | Noted as implementation caveat, not a dependency | Rubric Rev 6 |
