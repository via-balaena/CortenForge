# Spec C — Parsing Breadth: Spec

**Status:** Draft
**Phase:** Roadmap Phase 7 — MJCF Parsing & Defaults Gaps
**Effort:** M
**MuJoCo ref:** Element user data in `mjmodel.h` lines 809–1222; `nuser_*`
auto-sizing in `user_model.cc`; `<size>` parsing in `xml_native.cc`
**MuJoCo version:** 3.5.0 (C source and Python bindings verification)
**Test baseline:** 2,100+ sim domain tests (post-Phase 7 Spec B)
**Prerequisites:**
- Phase 7 Umbrella (`b025795`)
- Phase 7 T1: DT-85 flex contact attrs (`cf76731`)
- Phase 7 Spec A: Defaults Completeness (`01ae59f`)
- Phase 7 Spec B: Joint Physics (`3f70616`)
- Phase 7 Spec C Rubric (`0290ced`)

**Independence:** This spec is independent of Specs A and B per the umbrella
dependency graph. Shared files: `parser.rs` (different sections — Spec C
touches body/geom/joint/site/tendon/actuator user parsing and `<size>`
parsing; Specs A/B touch defaults cascade and joint attrs), `types.rs`
(Spec C adds user fields to 6 structs and 4 flexcomp fields; Specs A/B
touch different structs), `model.rs` (all additions are new fields — no
conflicts).

> **Conformance mandate (split):** This spec has a split conformance posture:
> 1. **§55 (Per-element user data)** — MUST produce numerically identical
>    results to MuJoCo 3.5.0. The MuJoCo C source code and Python bindings
>    are the source of truth. All expected values verified empirically.
> 2. **DT-88 (Flexcomp attributes)** — is parse-and-store only.
>    `<flexcomp>` is NOT recognized by MuJoCo 3.5.0's XML parser (verified
>    empirically — produces "Schema violation: unrecognized element"). The
>    dm_control `schema.xml` and MuJoCo documentation are the source of
>    truth for this task.

---

## Scope Adjustment

Empirical verification against MuJoCo 3.5.0 revealed critical facts that
refine the umbrella's original scope:

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| §55: 7 element types for user data | Confirmed: body, geom, joint, site, tendon, actuator, sensor. MuJoCo also supports `cam_user`/`nuser_cam` (camera). | **Camera out of scope** per umbrella — track as DT-126 |
| §55: `Vec<Vec<f64>>` storage | MuJoCo uses flat `nuser_* × count` stride storage. `Vec<Vec<f64>>` is conformance-equivalent when all inner vecs have uniform length `nuser_*`. | **In scope — conformance-equivalent** |
| §55: Default class inheritance | MuJoCo supports user defaults on 5 of 7 types: joint, geom, site, tendon, actuator. Body and sensor do NOT support `<default><body>` or `<default><sensor>` (schema violation). CortenForge already has `MjcfSensorDefaults.user` — do not modify. | **5 defaults structs need user field** |
| DT-88: `<flexcomp>` attributes | `<flexcomp>` NOT recognized by MuJoCo 3.5.0 binary. dm_control schema includes it. | **Documentation fidelity only** |
| DT-88: `inertiabox` type | dm_control schema says `float` (scalar). MuJoCo docs page says `real(3)`. | **Use dm_control schema** — `f64` scalar |
| DT-88: `file` loading | Umbrella says parse+store path only. Actual loading deferred. | **Parse + store path only** |

**Final scope:**

1. **§55:** Per-element user data arrays — parse `user` attribute from 7
   element types (body, geom, joint, site, tendon, actuator, sensor), parse
   `nuser_*` from `<size>` element, store `*_user` arrays and `nuser_*`
   fields on Model, wire through builder. Default class inheritance for 5
   element types (joint, geom, site, tendon, actuator). Full MuJoCo 3.5.0
   conformance. Sensor: parse already exists, defaults already exist — wire
   to Model only.
2. **DT-88:** Flexcomp attributes — parse `inertiabox` (f64), `scale`
   (Vec3), `quat` (Vec4), `file` (String) from `<flexcomp>`. Store on
   `MjcfFlex`. Apply scale and quat transforms to generated vertex
   positions during flexcomp expansion. Documentation-fidelity only.

---

## Problem Statement

**Conformance gap — §55:** MuJoCo supports per-element custom user data
arrays (`body_user`, `geom_user`, `jnt_user`, `site_user`, `tendon_user`,
`actuator_user`, `sensor_user`) with auto-sizing, explicit sizing via
`<size nuser_*="N">`, zero-padding, default class inheritance, and
validation. CortenForge has zero implementation for 6 of 7 element types.
Sensor user data is parsed from MJCF but never wired to Model — the parsed
`Vec<f64>` is discarded during building. No `<size>` element parsing
exists (falls through to `skip_element` at `parser.rs:132`). No `*_user`
or `nuser_*` fields exist on the Model struct.

**Conformance gap — DT-88:** MuJoCo's `<flexcomp>` element (documented in
MuJoCo docs and dm_control schema) supports `inertiabox`, `scale`, `quat`,
and `file` attributes. CortenForge's flexcomp parser (`parse_flex_attrs` at
`parser.rs:2772`) does not parse any of these. Generated vertices from
`generate_grid`/`generate_box_mesh` are not transformed by scale or
rotation.

---

## MuJoCo Reference

### User Data Storage Layout (mjmodel.h)

MuJoCo stores user data as flat arrays with stride `nuser_*`:

| Field | mjmodel.h | Shape | Stride |
|-------|-----------|-------|--------|
| `body_user` | line 809 | `nbody × nuser_body` | `nuser_body` |
| `jnt_user` | line 845 | `njnt × nuser_jnt` | `nuser_jnt` |
| `geom_user` | line 894 | `ngeom × nuser_geom` | `nuser_geom` |
| `site_user` | line 906 | `nsite × nuser_site` | `nuser_site` |
| `cam_user` | line 925 | `ncam × nuser_cam` | `nuser_cam` (**out of scope**) |
| `tendon_user` | line 1167 | `ntendon × nuser_tendon` | `nuser_tendon` |
| `actuator_user` | line 1202 | `nu × nuser_actuator` | `nuser_actuator` |
| `sensor_user` | line 1222 | `nsensor × nuser_sensor` | `nuser_sensor` |

Each `nuser_*` field is an `int` in `mjModel` (lines 743–750).

### Auto-Sizing Algorithm

When no explicit `<size nuser_*="N">` is specified (or `nuser_*=-1`),
MuJoCo auto-computes `nuser_*` as the **maximum resolved user array
length** across ALL instantiated elements of that type:

```
nuser_* = max over all instantiated elements of type T:
            len(effective_user_data(element))
```

**Demand-driven, not declaration-driven:** A default class with
`user="1 2 3 4 5"` contributes to auto-sizing ONLY if at least one
element actually inherits from that class. If all elements override with
explicit `user=` or no element references the class, the default's length
is ignored.

When NO element of a type has user data AND no `nuser_*` is set:
`nuser_* = 0`, arrays exist but with zero width.

### Explicit nuser_* via `<size>`

When `<size nuser_body="5"/>` is set:
- `nuser_body = 5` (overrides auto-sizing)
- All body user arrays are padded/truncated to length 5
- **Validation:** If any element's effective user data length > 5, MuJoCo
  raises an error: `"user has more values than nuser_body in body"`

Validation details:
- Validation operates on **effective** (post-defaults-cascade) user data
- A default with 5 values is legal even if `nuser_*=3`, as long as every
  element that inherits it overrides with ≤ 3 values
- Only the first validation error is reported (XML document order)

### nuser_* Parsing Rules

- Type: strict integer. `"2.5"`, `"3.7"`, `"1.0"`, `"1e2"` all rejected
  with "problem reading attribute"
- Range: `>= -1`. `-1` = auto-size sentinel. `-2`, `-100` rejected with
  "nuser_* must be >= -1"
- Maximum: `< 2147483647` (INT_MAX). Value 2147483647 itself rejected.
  Implies `i32` storage with reserved sentinel.
- Multiple `<size>` elements merge with last-writer-wins per attribute

### Zero-Padding Behavior

Elements with user data shorter than `nuser_*` are zero-padded:
- `user="1.5 2.5"` with `nuser_body=5` → `[1.5, 2.5, 0.0, 0.0, 0.0]`
- Elements with no `user` attribute → `[0.0, 0.0, ..., 0.0]` (nuser_* zeros)
- World body → always `[0.0, ..., 0.0]` (implicit, not an XML element)

### Default Class Inheritance for User Data

**Supported on 5 element types:** joint, geom, site, tendon, actuator.
**NOT supported:** body (`<default><body>` → schema violation), sensor
(`<default><sensor>` → schema violation in MuJoCo, but CortenForge already
supports it — do not modify).

Inheritance rules:
1. **Full replacement, not partial merge:** Element `user="10"` with
   default `user="100 200 300"` produces `[10, 0, 0]`, NOT `[10, 200, 300]`.
   The element's user data completely replaces the default; remaining
   positions are zero-padded to `nuser_*`.
2. **`user=""` = attribute not specified:** Treated identically to omitting
   `user` — defaults cascade still applies. `user=""` with default
   `user="1 2 3"` → element gets `[1, 2, 3]`.
3. **`user="0"` IS data:** `user="0"` produces `[0.0]` (1-element array,
   nuser=1). `user=""` produces absence (nuser=0 or inherits default).
4. **Transitive inheritance:** Class A sets `joint user="1 2 3"`, class B
   (child of A) sets no user, class C (child of B) sets no user → element
   using class C gets `[1, 2, 3]`.
5. **`childclass` propagation:** `childclass="foo"` on a body propagates
   to all descendant elements. Nearest-ancestor rule: inner `childclass`
   overrides outer. Explicit `class=` on an element overrides `childclass`.
   `class="main"` explicitly selects root default.
6. **`childclass` does NOT shadow root default:** If class "foo" defines
   NO user and root default defines user, elements under
   `childclass="foo"` still inherit root's user through class chain.
7. **Actuator subtype defaults share storage:** Within a single default
   class, `<motor>`, `<position>`, `<velocity>`, `<cylinder>`, `<muscle>`,
   `<general>` are aliases for the SAME underlying default. Last-write-wins
   in XML order. To differentiate subtypes, use separate default classes.

### User Data Value Parsing

- Whitespace-only separators (space, tab, XML entities `&#9;`/`&#10;`)
- Commas, semicolons, pipes rejected
- Scientific notation accepted (`1e5`, `-2.5e-3`)
- Hex notation accepted via strtod (`0xFF` → 255.0, `0x1p10` → 1024.0)
- NaN/Inf accepted (case-insensitive) with warning at process exit
- Integer-written values stored as float64
- Leading zeros stripped silently (`007` → `7.0`)

### Element Types That Do NOT Accept `user`

Exhaustively verified: `<inertial>`, `<light>`, `<freejoint>`, `<pair>`,
`<exclude>`, `<numeric>`, `<tuple>`, `<text>`, `<key>`, `<material>`,
`<texture>`, `<mesh>`, `<hfield>`, `<flex>`, `<connect>`, `<weld>`,
equality `<joint>`, equality `<tendon>`, `<frame>` (parser accepts but
data discarded — compile-time element only). `<freejoint>` has a reduced
attribute set; use `<joint type="free" user="..."/>` instead.

### User Data Physics Inertness

Empirically verified: two identical models — one with user data on all
element types, one without — produce bit-identical `qpos` and `qvel`
after 100 simulation steps. User data has zero runtime physics effect.

### Flexcomp Attributes (dm_control schema)

From dm_control `schema.xml` (element definition starts at line 1163):

| Attribute | Type | Default | Schema Line | Description |
|-----------|------|---------|-------------|-------------|
| `inertiabox` | `float` (scalar) | 0 | 1173 | Vertex inertia: `mass/n × 2×ib²/3` per vertex |
| `scale` | `float[3]` | `[1, 1, 1]` | 1169 | Non-uniform scale applied to generated vertices |
| `quat` | `float[4]` | `[1, 0, 0, 0]` | 1183 | Rotation quaternion (w,x,y,z) applied to generated vertices |
| `file` | `string` | `""` | 1174 | Path to mesh/grid data file |

Transform application order: scale first (component-wise multiply), then
quaternion rotation. `<flexcomp>` is NOT in the MuJoCo 3.5.0 binary —
cannot verify empirically (produces "Schema violation: unrecognized
element"). However, the `<flex>` element (not `<flexcomp>`) IS confirmed
present and functional in MuJoCo 3.5.0 — it accepts visual/geometry
attributes (`radius`, `rgba`, `material`, `flatskin`, `group`,
`texcoord`) but requires the `body` attribute. Only `<flexcomp>` is absent
from 3.5.0; `<flex>` itself works.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Body user data parsing | `<body user="1 2 3">` → `body_user[id]` = `[1, 2, 3, ...]` zero-padded to `nuser_body` | **Missing** — no `user` attribute parsed in `parse_body_attrs` (parser.rs:1607) |
| Geom user data parsing | `<geom user="...">` → `geom_user[id]` | **Missing** — `parse_geom_attrs` (parser.rs:1769) |
| Joint user data parsing | `<joint user="...">` → `jnt_user[id]` | **Missing** — `parse_joint_attrs` (parser.rs:1663) |
| Site user data parsing | `<site user="...">` → `site_user[id]` | **Missing** — `parse_site_attrs` (parser.rs:1933) |
| Tendon user data parsing | `<tendon user="...">` → `tendon_user[id]` | **Missing** — `parse_tendon_attrs` (parser.rs:3411) |
| Actuator user data parsing | `<actuator user="...">` → `actuator_user[id]` | **Missing** — `parse_actuator_attrs` (parser.rs:2121) |
| Sensor user data parsing | `<sensor_type user="...">` → `sensor_user[id]` | **Parsed** in `parse_sensor_attrs` (parser.rs:3630) but **not wired** to Model |
| `<size>` element parsing | `<size nuser_body="5" .../>` → sets nuser_* fields | **Missing** — `<size>` falls to `skip_element` (parser.rs:132) |
| nuser_* auto-sizing | Max resolved user length across type | **Missing** — no nuser_* fields on Model |
| Zero-padding | Elements with shorter user data padded to nuser_* length | **Missing** |
| Validation | Error if effective user length > explicit nuser_* | **Missing** |
| Default class inheritance for user | 5 types: joint, geom, site, tendon, actuator | **Missing** for 5 types. Sensor defaults user already exists (defaults.rs:956) |
| Model *_user arrays | `Vec<Vec<f64>>` per element type | **Missing** — no *_user fields on Model |
| Flexcomp inertiabox | Scalar on `<flexcomp>` | **Missing** — `parse_flex_attrs` (parser.rs:2772) |
| Flexcomp scale | Vec3 applied to generated vertices | **Missing** |
| Flexcomp quat | Rotation quaternion applied after scale | **Missing** |
| Flexcomp file | Path string for mesh/grid data | **Missing** |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| User data storage | Flat `mjtNum*` with stride `nuser_*`: `element_user[id * nuser_* + idx]` | `Vec<Vec<f64>>`: `element_user[id][idx]` — each inner vec has length `nuser_*` | Stride access `[id * nuser + k]` → indexed access `[id][k]`. All inner vecs must be uniform length for conformance equivalence. |
| `nuser_*` type | `int` (C `int` = 32-bit signed) | `i32` | Direct port — same type |
| `nuser_*` compiled default | `≥ 0` after compilation (auto-sized or explicit) | `i32` field, initialized to `-1` on MjcfModel, resolved to `≥ 0` during build | Builder resolves `-1` → auto-size; positive → use as-is |
| User data field naming | `body_user`, `jnt_user`, `geom_user`, `site_user`, `tendon_user`, `actuator_user`, `sensor_user` | Same names on Model struct | Direct port — no translation needed |
| `nuser_*` field naming | `nuser_body`, `nuser_jnt`, `nuser_geom`, `nuser_site`, `nuser_tendon`, `nuser_actuator`, `nuser_sensor` | Same names on Model struct | Direct port — no translation needed |
| Quaternion (flexcomp) | `[w, x, y, z]` | `UnitQuaternion` (nalgebra, `[w, x, y, z]` internally) | Construct via `UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))` |
| Element indexing (user arrays) | body_user[body_id], geom_user[geom_id], etc. — same index as the element's position in model arrays | Same — `body_user[body_id]` etc. | Direct port — array position = element ID |
| Element ordering | Depth-first pre-order for bodies; encounter order for geoms/joints/sites; XML document order for sensors/tendons/actuators | Same — builder already processes in this order | Direct port — no translation needed |

---

## Architecture Decisions

### AD-1: `<size>` Data Storage Location

**Problem:** `<size>` is a top-level MJCF element that contains `nuser_*`
attributes. Where should parsed `nuser_*` values be stored before they
reach the Model?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Add `nuser_*` fields to `MjcfModel` | Simple, direct. MjcfModel is the MJCF IR — `<size>` data belongs there. | Slightly mixes model metadata with element data. |
| 2 | Create a new `MjcfSize` struct on `MjcfModel` | Clean separation. Mirrors XML structure. | Over-engineers for 7 integer fields. |
| 3 | Store directly on `MjcfOption` | No new struct. | Semantically wrong — `<size>` is not `<option>`. |

**Chosen:** Option 1 — Add 7 `nuser_*` fields (type `i32`, default `-1`)
directly to `MjcfModel`. The `<size>` element is simple enough that a
dedicated struct is unnecessary. The fields sit alongside other model-level
metadata. Builder reads them during user data wiring and resolves `-1` to
the auto-sized value.

### AD-2: User Data Auto-Sizing and Validation Location

**Problem:** Auto-sizing (computing `nuser_*` from max user array length)
and validation (error if user length > explicit nuser_*) must happen
somewhere in the pipeline. Where?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | In the builder, after defaults cascade, before Model assembly | Has access to all resolved user arrays. Matches MuJoCo's compilation phase. | Slightly complex — needs two passes (compute nuser, then pad). |
| 2 | In the parser, during attribute parsing | Early detection. | No access to resolved defaults — can't auto-size correctly. |
| 3 | As a post-build validation step | Clean separation. | Too late — Model arrays already allocated. |

**Chosen:** Option 1 — Builder handles auto-sizing and validation. After
defaults cascade resolves all user arrays, the builder: (a) computes
`nuser_*` as max length if auto-sizing, (b) validates all lengths ≤
`nuser_*` if explicit, (c) pads/truncates all arrays to `nuser_*` length.
This mirrors MuJoCo's compilation phase where validation happens after
defaults application.

### AD-3: User Data Defaults Integration Pattern

**Problem:** 5 element types need `user: Option<Vec<f64>>` added to their
defaults structs plus integration into the existing merge and apply
functions.

**Chosen:** Follow the sensor pattern exactly. For each of the 5 types:
- Add `user: Option<Vec<f64>>` to the defaults struct
- Add merge logic: `user: c.user.clone().or_else(|| p.user.clone())`
- Add apply logic: `if result.user.is_empty() { use default }`
- Add parse logic: parse `user` attribute in `parse_*_defaults()`

This is the lightest-touch approach — extends existing pattern, no new
abstractions.

---

## Specification

### Element Type Matrix

Every element type with its specific pipeline stages:

| Type | MuJoCo C Field | MJCF Struct | Parser Fn | Defaults Struct | Defaults Parse Fn | Apply Fn | Builder Fn | Model Field | nuser Field |
|------|----------------|-------------|-----------|-----------------|-------------------|----------|------------|-------------|-------------|
| body | `body_user` / `nuser_body` | `MjcfBody` | `parse_body_attrs` | N/A (no body defaults in MuJoCo) | N/A | N/A | `process_body` | `body_user` | `nuser_body` |
| geom | `geom_user` / `nuser_geom` | `MjcfGeom` | `parse_geom_attrs` | `MjcfGeomDefaults` | `parse_geom_defaults` | `apply_to_geom` | `process_geom` | `geom_user` | `nuser_geom` |
| joint | `jnt_user` / `nuser_jnt` | `MjcfJoint` | `parse_joint_attrs` | `MjcfJointDefaults` | `parse_joint_defaults` | `apply_to_joint` | `process_joint` | `jnt_user` | `nuser_jnt` |
| site | `site_user` / `nuser_site` | `MjcfSite` | `parse_site_attrs` | `MjcfSiteDefaults` | `parse_site_defaults` | `apply_to_site` | `process_site` | `site_user` | `nuser_site` |
| tendon | `tendon_user` / `nuser_tendon` | `MjcfTendon` | `parse_tendon_attrs` | `MjcfTendonDefaults` | `parse_tendon_defaults` | `apply_to_tendon` | `process_tendons` | `tendon_user` | `nuser_tendon` |
| actuator | `actuator_user` / `nuser_actuator` | `MjcfActuator` | `parse_actuator_attrs` | `MjcfActuatorDefaults` | `parse_actuator_defaults` | `apply_to_actuator` | `process_actuator` | `actuator_user` | `nuser_actuator` |
| sensor | `sensor_user` / `nuser_sensor` | `MjcfSensor` | `parse_sensor_attrs` | `MjcfSensorDefaults` | `parse_sensor_defaults` | `apply_to_sensor` | `process_sensors` | `sensor_user` | `nuser_sensor` |

**Lifecycle stages per type:**
- (1) Parse `user` attribute in parser → (2) Store on MJCF type struct →
  (3) Apply defaults cascade in defaults.rs → (4) Wire through builder to
  Model field → (5) Initialize Model field in model_init.rs

| Type | Stage 1 (parse) | Stage 2 (struct) | Stage 3 (defaults) | Stage 4 (builder wire) | Stage 5 (model init) |
|------|-----------------|------------------|--------------------|-----------------------|---------------------|
| body | **Add** | **Add** | N/A (no defaults) | **Add** | **Add** |
| geom | **Add** | **Add** | **Add** | **Add** | **Add** |
| joint | **Add** | **Add** | **Add** | **Add** | **Add** |
| site | **Add** | **Add** | **Add** | **Add** | **Add** |
| tendon | **Add** | **Add** | **Add** | **Add** | **Add** |
| actuator | **Add** | **Add** | **Add** | **Add** | **Add** |
| sensor | Exists | Exists | Exists | **Add** (missing) | **Add** |

### S1. `<size>` Element Parsing + MjcfModel nuser_* Fields

**File:** `sim/L0/mjcf/src/types.rs` (MjcfModel struct, ~line 3807)
**MuJoCo equivalent:** `<size>` element parsing in `xml_native.cc`,
`nuser_*` fields in `mjModel` (mjmodel.h lines 743–750)
**Design decision:** Add 7 `nuser_*` fields directly to `MjcfModel`
(AD-1). Default to `-1` (auto-size sentinel). Parser sets them from
`<size>` attributes.

**Add to MjcfModel struct:**
```rust
/// Per-element user data sizes from <size> element.
/// -1 = auto-size (default), >= 0 = explicit size.
pub nuser_body: i32,
pub nuser_jnt: i32,
pub nuser_geom: i32,
pub nuser_site: i32,
pub nuser_tendon: i32,
pub nuser_actuator: i32,
pub nuser_sensor: i32,
```

All default to `-1` in `MjcfModel::default()`.

**File:** `sim/L0/mjcf/src/parser.rs` (~line 67, main parse loop)
**Design decision:** Handle `<size>` as both `Event::Start` (with
`skip_element` after parsing attrs) and `Event::Empty` (self-closing).
Multiple `<size>` elements merge with last-writer-wins per attribute,
matching MuJoCo behavior.

**Add `<size>` handling to the main parse loop:**

In the `Event::Start` match arm (~line 71), add before the `_ =>` fallthrough:
```rust
b"size" => {
    parse_size_attrs(e, &mut model);
    skip_element(reader, &elem_name)?;
}
```

In the `Event::Empty` match arm (~line 135), add:
```rust
} else if e.name().as_ref() == b"size" {
    parse_size_attrs(e, &mut model);
}
```

**New function `parse_size_attrs`:**
```rust
/// Parse `<size>` element attributes into MjcfModel nuser_* fields.
/// Multiple <size> elements merge with last-writer-wins per attribute.
fn parse_size_attrs(e: &BytesStart, model: &mut MjcfModel) {
    if let Some(val) = parse_int_attr(e, "nuser_body") {
        model.nuser_body = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_jnt") {
        model.nuser_jnt = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_geom") {
        model.nuser_geom = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_site") {
        model.nuser_site = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_tendon") {
        model.nuser_tendon = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_actuator") {
        model.nuser_actuator = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_sensor") {
        model.nuser_sensor = val;
    }
}
```

### S2. MJCF Type Structs — Add `user` Field to 6 Element Types

**File:** `sim/L0/mjcf/src/types.rs`
**MuJoCo equivalent:** `user` attribute on body/geom/joint/site/tendon/
actuator elements.
**Design decision:** Use `Vec<f64>` (not `Option<Vec<f64>>`) for the
element-level user field, matching the existing `MjcfSensor.user` pattern.
Empty vec means "no user data specified." The distinction between
"not specified" and "explicit empty" (`user=""`) is handled by the parser:
`user=""` is treated as not-specified (empty vec), matching MuJoCo behavior
where `user=""` ≡ omitting the attribute.

**Add to each struct:**

```rust
// MjcfBody (types.rs ~line 2217):
/// Per-element user data from user="..." attribute.
pub user: Vec<f64>,

// MjcfGeom (types.rs ~line 1170):
pub user: Vec<f64>,

// MjcfJoint (types.rs ~line 1407):
pub user: Vec<f64>,

// MjcfSite (types.rs ~line 1530):
pub user: Vec<f64>,

// MjcfTendon (types.rs ~line 2750):
pub user: Vec<f64>,

// MjcfActuator (types.rs ~line 2400):
pub user: Vec<f64>,
```

All default to `Vec::new()` via the existing `Default` impls (which use
`Vec::new()` for vec fields).

### S3. Parser — Add `user` Attribute Parsing to 6 Element Types

**File:** `sim/L0/mjcf/src/parser.rs`
**MuJoCo equivalent:** `user` attribute parsing on element types
**Design decision:** Use the same pattern as existing sensor user parsing
(`parse_sensor_attrs` line 3630). Parse as space-separated float array.
`user=""` or `user="   "` (whitespace only) produces empty vec — treated
as "not specified" for defaults cascade purposes.

**Add to each parse function (identical pattern):**

```rust
// In parse_body_attrs (~line 1607):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        body.user = parts;
    }
}

// In parse_geom_attrs (~line 1769):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        geom.user = parts;
    }
}

// In parse_joint_attrs (~line 1663):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        joint.user = parts;
    }
}

// In parse_site_attrs (~line 1933):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        site.user = parts;
    }
}

// In parse_tendon_attrs (~line 3411):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        tendon.user = parts;
    }
}

// In parse_actuator_attrs (~line 2121):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        actuator.user = parts;
    }
}
```

Note: `parse_float_array` already handles whitespace-separated floats.
For `user=""`, `parse_float_array` returns `Ok(vec![])` (empty), which
leaves the field at its default empty vec — correct MuJoCo behavior.

### S4. Defaults — Add `user` to 5 Defaults Structs + Merge + Apply

**File:** `sim/L0/mjcf/src/types.rs` (defaults structs)
**File:** `sim/L0/mjcf/src/parser.rs` (defaults parsing)
**File:** `sim/L0/mjcf/src/defaults.rs` (merge + apply)
**MuJoCo equivalent:** Default class user attribute inheritance
**Design decision:** Follow the exact sensor defaults pattern (AD-3).
Body and sensor already handled — body has no defaults in MuJoCo,
sensor defaults already exist.

**Step 4a: Add `user: Option<Vec<f64>>` to 5 defaults structs in types.rs:**

```rust
// MjcfJointDefaults (~line 584): add field
pub user: Option<Vec<f64>>,

// MjcfGeomDefaults (~line 626): add field
pub user: Option<Vec<f64>>,

// MjcfSiteDefaults (~line 951): add field
pub user: Option<Vec<f64>>,

// MjcfTendonDefaults (~line 771): add field
pub user: Option<Vec<f64>>,

// MjcfActuatorDefaults (~line 686): add field
pub user: Option<Vec<f64>>,
```

All default to `None` via `#[derive(Default)]`.

**Step 4b: Parse `user` in 5 defaults parse functions in parser.rs:**

Add to each `parse_*_defaults` function (identical pattern):
```rust
// In parse_joint_defaults (~line 597):
if let Some(user) = get_attribute_opt(e, "user") {
    if let Ok(parts) = parse_float_array(&user) {
        if !parts.is_empty() {
            defaults.user = Some(parts);
        }
    }
}

// Same pattern in: parse_geom_defaults, parse_site_defaults,
// parse_tendon_defaults, parse_actuator_defaults
```

Note: Only set `Some(...)` when parsing produces non-empty array. This
ensures `user=""` in defaults does not set `Some(vec![])` — it should
remain `None` to allow parent defaults to cascade through.

**Step 4c: Add merge logic in defaults.rs (5 merge functions):**

Add to each `merge_*_defaults` function, in the `(Some(p), Some(c))` arm:
```rust
// merge_joint_defaults:
user: c.user.clone().or_else(|| p.user.clone()),

// Same pattern in: merge_geom_defaults, merge_site_defaults,
// merge_tendon_defaults, merge_actuator_defaults
```

This gives child-wins-if-present, parent-as-fallback semantics —
implementing transitive inheritance.

**Step 4d: Add apply logic in defaults.rs (5 apply functions):**

Add to each `apply_to_*` function:
```rust
// apply_to_joint:
if result.user.is_empty() {
    if let Some(ref user) = defaults.user {
        result.user.clone_from(user);
    }
}

// Same pattern in: apply_to_geom, apply_to_site,
// apply_to_tendon, apply_to_actuator
```

This matches the existing `apply_to_sensor` pattern exactly (defaults.rs
line 660): if element has no user data (empty vec), apply default.

### S5. Model Struct — Add `*_user` and `nuser_*` Fields

**File:** `sim/L0/core/src/types/model.rs` (~line 44)
**File:** `sim/L0/core/src/types/model_init.rs`
**MuJoCo equivalent:** `mjModel` user data fields (mjmodel.h lines 743–750,
809–1222)
**Design decision:** Use `Vec<Vec<f64>>` for user data arrays (umbrella
convention §5). Use `i32` for `nuser_*` fields to match MuJoCo's int type.

**Add to Model struct (grouped with existing element arrays):**

```rust
// After body arrays (~line 162):
/// Per-body user data. Each inner vec has length nuser_body.
pub body_user: Vec<Vec<f64>>,

// After joint arrays (~line 209):
/// Per-joint user data. Each inner vec has length nuser_jnt.
pub jnt_user: Vec<Vec<f64>>,

// After geom arrays (~line 308):
/// Per-geom user data. Each inner vec has length nuser_geom.
pub geom_user: Vec<Vec<f64>>,

// After site arrays (~line 503):
/// Per-site user data. Each inner vec has length nuser_site.
pub site_user: Vec<Vec<f64>>,

// After tendon arrays (~line 703):
/// Per-tendon user data. Each inner vec has length nuser_tendon.
pub tendon_user: Vec<Vec<f64>>,

// After actuator arrays (~line 648):
/// Per-actuator user data. Each inner vec has length nuser_actuator.
pub actuator_user: Vec<Vec<f64>>,

// After sensor arrays (~line 554):
/// Per-sensor user data. Each inner vec has length nuser_sensor.
pub sensor_user: Vec<Vec<f64>>,

// With dimension fields:
/// Number of user data values per body (-1 before compilation).
pub nuser_body: i32,
pub nuser_jnt: i32,
pub nuser_geom: i32,
pub nuser_site: i32,
pub nuser_tendon: i32,
pub nuser_actuator: i32,
pub nuser_sensor: i32,
```

**model_init.rs — initialize all new fields:**

```rust
// In Model::empty():
body_user: vec![],
jnt_user: vec![],
geom_user: vec![],
site_user: vec![],
tendon_user: vec![],
actuator_user: vec![],
sensor_user: vec![],
nuser_body: 0,
nuser_jnt: 0,
nuser_geom: 0,
nuser_site: 0,
nuser_tendon: 0,
nuser_actuator: 0,
nuser_sensor: 0,
```

Note: Model `nuser_*` fields initialize to `0` (not `-1`), because after
compilation they always hold the resolved value `≥ 0`.

### S6. Builder Wiring — Auto-Size, Validate, Pad, Store

**File:** `sim/L0/mjcf/src/builder/mod.rs` (main build pipeline)
**File:** `sim/L0/mjcf/src/builder/body.rs` (body processing)
**File:** `sim/L0/mjcf/src/builder/geom.rs` (geom/site processing)
**File:** `sim/L0/mjcf/src/builder/joint.rs` (joint processing)
**File:** `sim/L0/mjcf/src/builder/tendon.rs` (tendon processing)
**File:** `sim/L0/mjcf/src/builder/actuator.rs` (actuator processing)
**File:** `sim/L0/mjcf/src/builder/sensor.rs` (sensor processing)
**File:** `sim/L0/mjcf/src/builder/init.rs` (builder initialization)
**MuJoCo equivalent:** Compilation phase user data handling in
`user_model.cc`
**Design decision:** Follow AD-2. The builder accumulates raw user arrays
during element processing, then a post-processing step (after all elements
are processed, before `build()`) computes nuser_*, validates, and pads.

**Step 6a: Add raw user accumulation arrays to ModelBuilder (init.rs):**

```rust
// In ModelBuilder fields:
pub body_user_raw: Vec<Vec<f64>>,
pub geom_user_raw: Vec<Vec<f64>>,
pub jnt_user_raw: Vec<Vec<f64>>,
pub site_user_raw: Vec<Vec<f64>>,
pub tendon_user_raw: Vec<Vec<f64>>,
pub actuator_user_raw: Vec<Vec<f64>>,
pub sensor_user_raw: Vec<Vec<f64>>,
```

Initialize as `vec![]`. World body (body 0) gets `vec![]` pushed in
`ModelBuilder::new()` (world body never has user data).

**Step 6b: Push user data in each builder function:**

In each `process_*` function, after the existing array pushes, add:
```rust
// process_body (body.rs): after body arrays
self.body_user_raw.push(body.user.clone());

// process_geom (geom.rs): after geom arrays
self.geom_user_raw.push(geom.user.clone());

// process_joint (joint.rs): after joint arrays
self.jnt_user_raw.push(joint.user.clone());

// process_site (geom.rs): after site arrays
self.site_user_raw.push(site.user.clone());

// process_tendons (tendon.rs): in the per-tendon loop
self.tendon_user_raw.push(tendon.user.clone());

// process_actuator (actuator.rs): after actuator arrays
self.actuator_user_raw.push(actuator.user.clone());

// process_sensors (sensor.rs): in the per-sensor loop
self.sensor_user_raw.push(mjcf_sensor.user.clone());
```

**Step 6c: Post-processing — resolve nuser, validate, pad (mod.rs):**

Add a new function called after all elements are processed but before
`build()`:

```rust
/// Resolve nuser_* values, validate user data lengths, and pad arrays.
fn finalize_user_data(&mut self, mjcf: &MjcfModel) -> Result<()> {
    self.finalize_user_type(
        "body", mjcf.nuser_body, &mut self.body_user_raw,
    )?;
    self.finalize_user_type(
        "jnt", mjcf.nuser_jnt, &mut self.jnt_user_raw,
    )?;
    self.finalize_user_type(
        "geom", mjcf.nuser_geom, &mut self.geom_user_raw,
    )?;
    self.finalize_user_type(
        "site", mjcf.nuser_site, &mut self.site_user_raw,
    )?;
    self.finalize_user_type(
        "tendon", mjcf.nuser_tendon, &mut self.tendon_user_raw,
    )?;
    self.finalize_user_type(
        "actuator", mjcf.nuser_actuator, &mut self.actuator_user_raw,
    )?;
    self.finalize_user_type(
        "sensor", mjcf.nuser_sensor, &mut self.sensor_user_raw,
    )?;
    Ok(())
}

/// For a single element type: resolve nuser, validate, pad.
fn finalize_user_type(
    &mut self,
    type_name: &str,
    explicit_nuser: i32,
    raw: &mut Vec<Vec<f64>>,
) -> Result<()> {
    let nuser = if explicit_nuser >= 0 {
        // Explicit: validate all lengths
        let nuser = explicit_nuser as usize;
        for (i, user) in raw.iter().enumerate() {
            if user.len() > nuser {
                return Err(MjcfError::Validation(format!(
                    "user has more values than nuser_{} in {}",
                    type_name, type_name
                )));
            }
        }
        nuser
    } else {
        // Auto-size: max length across all elements
        raw.iter().map(|u| u.len()).max().unwrap_or(0)
    };

    // Pad all arrays to nuser length
    for user in raw.iter_mut() {
        user.resize(nuser, 0.0);
    }

    nuser  // Store on model via returned value
}
```

The exact function signatures will be adjusted during implementation to
handle ownership and return the computed `nuser` values. The algorithm is:
1. If `explicit_nuser >= 0`: use it, validate all lengths ≤ nuser
2. If `explicit_nuser == -1`: compute max length across all raw arrays
3. Pad all arrays to the resolved nuser length with zeros

**Step 6d: Transfer to Model in `build()` (build.rs):**

In the `build()` function, transfer the finalized user data:
```rust
model.body_user = self.body_user_raw;
model.geom_user = self.geom_user_raw;
model.jnt_user = self.jnt_user_raw;
model.site_user = self.site_user_raw;
model.tendon_user = self.tendon_user_raw;
model.actuator_user = self.actuator_user_raw;
model.sensor_user = self.sensor_user_raw;
model.nuser_body = computed_nuser_body as i32;
// ... etc for all 7 types
```

### S7. Flexcomp Attributes — Parse, Store, Transform

**File:** `sim/L0/mjcf/src/types.rs` (MjcfFlex struct, ~line 3669)
**File:** `sim/L0/mjcf/src/parser.rs` (parse_flex_attrs ~line 2772,
parse_flexcomp ~line 2902, parse_flexcomp_empty ~line 2986,
generate_grid ~line 3021, generate_box_mesh ~line 3050)
**MuJoCo equivalent:** `mjCFlexcomp` in `user_objects.cc`
**Design decision:** Store flexcomp attributes on MjcfFlex (existing struct)
rather than creating a separate MjcfFlexcomp struct. Apply scale and quat
transforms after vertex generation in `parse_flexcomp` and
`parse_flexcomp_empty`. This keeps the transformation local to the
flexcomp parsing code.

**Step 7a: Add fields to MjcfFlex (types.rs):**

```rust
// MjcfFlex struct (~line 3669):
/// Flexcomp inertia box size (scalar). Used for vertex inertia calculation.
pub inertiabox: f64,   // default 0.0
/// Flexcomp non-uniform scale applied to generated vertices.
pub flexcomp_scale: Option<Vector3<f64>>,
/// Flexcomp rotation quaternion applied to generated vertices (after scale).
pub flexcomp_quat: Option<UnitQuaternion<f64>>,
/// Flexcomp mesh/grid file path.
pub flexcomp_file: Option<String>,
```

Note: `flexcomp_scale` and `flexcomp_quat` use `Option` because they are
only meaningful for `<flexcomp>` elements, not `<flex>`. When `None`, no
transform is applied. Field names prefixed with `flexcomp_` to avoid
collision with MjcfFlex's own potential scale/quat fields.

**Step 7b: Parse attributes in parse_flex_attrs (parser.rs ~line 2772):**

Add to `parse_flex_attrs`:
```rust
if let Some(s) = get_attribute_opt(e, "inertiabox") {
    flex.inertiabox = s.parse().unwrap_or(0.0);
}
if let Some(s) = get_attribute_opt(e, "scale") {
    let vals: Vec<f64> = s.split_whitespace()
        .filter_map(|t| t.parse().ok()).collect();
    if vals.len() >= 3 {
        flex.flexcomp_scale = Some(Vector3::new(vals[0], vals[1], vals[2]));
    }
}
if let Some(s) = get_attribute_opt(e, "quat") {
    let vals: Vec<f64> = s.split_whitespace()
        .filter_map(|t| t.parse().ok()).collect();
    if vals.len() >= 4 {
        let q = Quaternion::new(vals[0], vals[1], vals[2], vals[3]);
        flex.flexcomp_quat = Some(UnitQuaternion::from_quaternion(q));
    }
}
if let Some(s) = get_attribute_opt(e, "file") {
    flex.flexcomp_file = Some(s);
}
```

**Step 7c: Apply transforms after vertex generation:**

In `parse_flexcomp` (~line 2915, after the `match comp_type` block) and
in `parse_flexcomp_empty` (~line 2999, after the `match comp_type` block),
add:

```rust
// Apply scale and rotation to generated vertices
apply_flexcomp_transforms(&mut flex);
```

New function:
```rust
/// Apply flexcomp scale and quat transforms to generated vertices.
/// Order: scale first (component-wise), then quaternion rotation.
fn apply_flexcomp_transforms(flex: &mut MjcfFlex) {
    // Scale (component-wise multiply)
    if let Some(scale) = flex.flexcomp_scale {
        for v in &mut flex.vertices {
            v.x *= scale.x;
            v.y *= scale.y;
            v.z *= scale.z;
        }
    }
    // Rotate (quaternion)
    if let Some(ref quat) = flex.flexcomp_quat {
        for v in &mut flex.vertices {
            *v = quat.transform_vector(v);
        }
    }
}
```

---

## Acceptance Criteria

### AC1: `<size>` nuser_* parsing *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_body="5" nuser_geom="3"/>`
**After:** Parse to MjcfModel
**Assert:** `mjcf_model.nuser_body == 5`, `mjcf_model.nuser_geom == 3`,
all other `nuser_*` fields remain `-1`
**Field:** `MjcfModel.nuser_body`, `MjcfModel.nuser_geom`

### AC2: User data auto-sizing *(runtime test — MuJoCo-verified)*
**Given:** MJCF with bodies: b1 `user="1 2 3"`, b2 `user="4 5"`, no
explicit nuser_body in `<size>`
**After:** Build Model
**Assert:** `model.nuser_body == 3` (max length), `model.body_user[0]`
= `[0, 0, 0]` (world), `model.body_user[1]` = `[1, 2, 3]`,
`model.body_user[2]` = `[4, 5, 0]` (zero-padded)
**Field:** `Model.nuser_body`, `Model.body_user`

### AC3: Explicit nuser_* with padding *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_geom="5"/>`, geom g1 `user="1.5 2.5"`,
geom g2 no user attribute
**After:** Build Model
**Assert:** `model.nuser_geom == 5`, `model.geom_user[g1_id]` =
`[1.5, 2.5, 0, 0, 0]`, `model.geom_user[g2_id]` = `[0, 0, 0, 0, 0]`
**Field:** `Model.nuser_geom`, `Model.geom_user`

### AC4: Too-long user data error *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_geom="2"/>`, geom `user="10 20 30 40"`
**After:** Attempt to build Model
**Assert:** Error containing "user has more values than nuser_geom"
**Field:** Build result (error)

### AC5: Default class inheritance *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<default><joint user="100 200"/></default>`, joint
j1 with no user attribute, joint j2 with `user="300"`
**After:** Build Model
**Assert:** `model.jnt_user[j1_id]` = `[100, 200]` (from default),
`model.jnt_user[j2_id]` = `[300, 0]` (override, zero-padded to nuser_jnt=2)
**Field:** `Model.jnt_user`, `Model.nuser_jnt`

### AC6: No-user-data model *(runtime test — MuJoCo-verified)*
**Given:** MJCF with no user attributes on any element, no `<size>` element
**After:** Build Model
**Assert:** All `nuser_*` == 0. All `*_user` arrays exist but inner vecs
are empty (length 0).
**Field:** All `Model.nuser_*`, `Model.*_user`

### AC7: World body user data *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<body user="1 2 3">` (one child body with user data)
**After:** Build Model
**Assert:** `model.body_user[0]` = `[0, 0, 0]` (world body — always
zeros), `model.body_user[1]` = `[1, 2, 3]`
**Field:** `Model.body_user`

### AC8: Sensor user data wired to Model *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<sensor><jointpos joint="j1" user="5 10"/></sensor>`
**After:** Build Model
**Assert:** `model.sensor_user[0]` = `[5, 10]`, `model.nuser_sensor == 2`
**Field:** `Model.sensor_user`, `Model.nuser_sensor`

### AC9: `user=""` inherits default *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<default><geom user="5 6 7"/></default>`, geom with
`user=""` (empty string attribute)
**After:** Build Model
**Assert:** Geom gets default `[5, 6, 7]` (user="" = not specified,
defaults apply)
**Field:** `Model.geom_user`

### AC10: `user="0"` is data, not absence *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<default><geom user="5 6 7"/></default>`, geom with
`user="0"` (explicit zero)
**After:** Build Model
**Assert:** Geom gets `[0]` (NOT `[5, 6, 7]`). `nuser_geom = 3` (max
of default's 3 and override's 1). Geom is zero-padded to `[0, 0, 0]`.
**Field:** `Model.geom_user`, `Model.nuser_geom`

### AC11: Actuator subtype defaults last-write-wins *(runtime test — MuJoCo-verified)*
**Given:** MJCF with default class containing `<general user="100"/>` then
`<motor user="200"/>`, plus a motor actuator with no user attribute
**After:** Build Model
**Assert:** Actuator gets `[200]` (last-write-wins within default class)
**Field:** `Model.actuator_user`

### AC12: Transitive default inheritance *(runtime test — MuJoCo-verified)*
**Given:** MJCF with 3-level default hierarchy: class A sets
`joint user="1 2 3"`, class B (child of A) sets NO joint user, class C
(child of B) sets NO joint user. Joint uses class C.
**After:** Build Model
**Assert:** Joint gets `[1, 2, 3]` (transitive inheritance from A through
B and C)
**Field:** `Model.jnt_user`

### AC13: All 7 element types store user data *(runtime test — MuJoCo-verified)*
**Given:** MJCF with user data on all 7 element types (body, geom, joint,
site, tendon, actuator, sensor)
**After:** Build Model
**Assert:** All 7 `*_user` arrays populated correctly, all 7 `nuser_*`
fields > 0
**Field:** All `Model.*_user` and `Model.nuser_*`

### AC14: Too-long default inherited → error *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_jnt="2"/>`, default class with
`joint user="1 2 3"` (3 values > nuser_jnt=2), joint j1 inherits default
(no explicit user)
**After:** Attempt to build Model
**Assert:** Error — inherited user data (3 values) exceeds nuser_jnt (2)
**Field:** Build result (error)

### AC15: Explicit override avoids too-long default *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_jnt="2"/>`, default class with
`joint user="1 2 3"` (3 values), joint j1 with explicit `user="10 20"`
(2 values ≤ nuser)
**After:** Build Model
**Assert:** Success. `model.jnt_user[j1_id]` = `[10, 20]`
**Field:** `Model.jnt_user`

### AC16: Flexcomp scale and quat transform *(runtime test — analytically derived)*
**Given:** `<flexcomp type="grid" count="2 2" spacing="1.0"
scale="2 3 1" quat="0.7071068 0 0 0.7071068"/>` (90° rotation about Z)
**After:** Parse to MjcfFlex
**Assert:** Original vertices `(0,0,0), (1,0,0), (0,1,0), (1,1,0)` →
after scale `(0,0,0), (2,0,0), (0,3,0), (2,3,0)` → after 90° Z rotation
`(0,0,0), (0,2,0), (-3,0,0), (-3,2,0)` (± 1e-10 tolerance)
**Field:** `MjcfFlex.vertices`

### AC17: Flexcomp inertiabox and file parsed *(runtime test — analytically derived)*
**Given:** `<flexcomp type="grid" count="2 2" spacing="1.0"
inertiabox="0.5" file="mesh.obj"/>`
**After:** Parse to MjcfFlex
**Assert:** `flex.inertiabox == 0.5`, `flex.flexcomp_file == Some("mesh.obj")`
**Field:** `MjcfFlex.inertiabox`, `MjcfFlex.flexcomp_file`

### AC18: Multiple `<size>` elements merge *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_body="3" nuser_geom="2"/>` followed by
`<size nuser_jnt="5"/>`
**After:** Parse to MjcfModel
**Assert:** `nuser_body=3, nuser_geom=2, nuser_jnt=5`, others `-1`
**Field:** `MjcfModel.nuser_*`

### AC19: User data physics inertness *(runtime test — MuJoCo-verified)*
**Given:** Two models — identical except one has user data on all elements
**After:** Run one simulation step on each
**Assert:** `qpos` and `qvel` are bit-identical between the two models
**Field:** `Data.qpos`, `Data.qvel`

### AC20: `nuser_*=-1` triggers auto-sizing *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_body="-1"/>` (explicit auto-size),
body with `user="1 2 3"`
**After:** Build Model
**Assert:** `model.nuser_body == 3` (auto-sized from max user length)
**Field:** `Model.nuser_body`

### AC21: `childclass` propagation with nearest-ancestor rule *(runtime test — MuJoCo-verified)*
**Given:** MJCF with class "A" defining `geom user="1 2"`, class "B"
defining `geom user="10 20 30"`. Body1 has `childclass="A"`, child Body2
has `childclass="B"`. Geom in Body1 (outside Body2), geom in Body2.
**After:** Build Model
**Assert:** Geom in Body1 gets `[1, 2, 0]` (from class A, padded to
nuser_geom=3). Geom in Body2 gets `[10, 20, 30]` (from class B —
nearest ancestor wins).
**Field:** `Model.geom_user`

### AC22: `childclass` does not shadow root default *(runtime test — MuJoCo-verified)*
**Given:** MJCF with root default `joint user="1 2 3"`, class "myclass"
(child of root) defines NO joint user. Body with `childclass="myclass"`,
joint j1 with no explicit class or user.
**After:** Build Model
**Assert:** `jnt_user[j1]` = `[1, 2, 3]` (class "myclass" inherits from
root through class chain; childclass selects a class, does not block
inheritance)
**Field:** `Model.jnt_user`

### AC23: `nuser_*` strict integer validation *(runtime test — MuJoCo-verified)*
**Given:** MJCF with `<size nuser_geom="2.5"/>`
**After:** Attempt to parse
**Assert:** `nuser_geom` remains at default `-1` (float value not parsed
as integer). Note: `parse_int_attr` returns `None` for non-integer values.

**Given:** MJCF with `<size nuser_geom="-2"/>`
**After:** Attempt to build Model
**Assert:** Error — `nuser_geom` must be >= -1.
**Field:** Parse/build result

### AC24: `<freejoint>` does not accept `user` *(code review)*
**Assert:** CortenForge's `<freejoint>` parser does not parse a `user`
attribute. `<freejoint>` has a reduced attribute set — user data on free
joints requires `<joint type="free" user="..."/>` instead. The
`<freejoint>` parser at its current location in parser.rs should not be
modified to add user parsing.

### AC25: Existing tests pass *(code review)*
**Assert:** All 2,100+ existing sim domain tests continue to pass with
zero regressions. No existing test behavior changes.

### AC26: Hex notation in user data is a known deviation *(code review)*
**Assert:** CortenForge's user data parsing uses Rust's `f64::parse`,
which does NOT support C-style hex notation (`0xFF` → 255.0) or hex float
notation (`0x1p10` → 1024.0). MuJoCo accepts these via `strtod`.
**Deviation:** `user="0xFF"` succeeds in MuJoCo (→ `[255.0]`) but will
fail in CortenForge. This is a known conformance deviation documented in
Out of Scope. No special handling is implemented — standard decimal float
notation covers all practical use cases.
**Field:** N/A — documents a parsing behavior gap, not a structural field.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (size parsing) | T1 | Direct |
| AC2 (auto-sizing) | T2 | Direct |
| AC3 (explicit nuser + padding) | T3 | Direct |
| AC4 (too-long error) | T4 | Direct |
| AC5 (default inheritance) | T5 | Direct |
| AC6 (no-user-data) | T6 | Direct |
| AC7 (world body zeros) | T2 | Edge case (covered by auto-sizing test) |
| AC8 (sensor wiring) | T7 | Direct |
| AC9 (user="" inherits) | T8 | Edge case |
| AC10 (user="0" is data) | T8 | Edge case |
| AC11 (actuator last-write-wins) | T9 | Edge case |
| AC12 (transitive inheritance) | T10 | Direct |
| AC13 (all 7 types) | T11 | Integration |
| AC14 (too-long inherited) | T12 | Edge case |
| AC15 (override avoids too-long) | T12 | Edge case |
| AC16 (flexcomp transform) | T13 | Direct |
| AC17 (flexcomp inertiabox/file) | T14 | Direct |
| AC18 (multiple size merge) | T1 | Edge case (covered in size parsing test) |
| AC19 (physics inertness) | T15 | Direct |
| AC20 (nuser=-1 auto-size) | T2 | Edge case (covered by auto-sizing test) |
| AC21 (childclass nearest-ancestor) | T16 | Direct |
| AC22 (childclass no shadow root) | T17 | Direct |
| AC23 (nuser_* integer validation) | T18 | Direct |
| AC24 (freejoint no user) | — | Code review (manual) |
| AC25 (existing tests pass) | — | Code review (manual) |
| AC26 (hex notation deviation) | — | Code review (manual) |

---

## Test Plan

### T1: `<size>` element parsing → AC1, AC18
**Model:** `<mujoco><size nuser_body="5" nuser_geom="3"/>
<size nuser_jnt="2"/><worldbody/></mujoco>`
**Assert:** Parse MjcfModel: `nuser_body=5, nuser_geom=3, nuser_jnt=2`,
others `-1`. Second `<size>` merges (last-writer-wins).
**MuJoCo 3.5.0 verified:** Yes — multiple `<size>` elements merge
correctly.

### T2: Auto-sizing with world body and padding → AC2, AC7, AC20
**Model:** Body b1 with `user="1 2 3"`, body b2 with `user="4 5"`. No
explicit nuser_body.
**Assert:** `nuser_body=3`, `body_user[0]=[0,0,0]` (world),
`body_user[1]=[1,2,3]`, `body_user[2]=[4,5,0]`.
**Additional sub-test:** Same model with `<size nuser_body="-1"/>` →
identical result (explicit `-1` = auto-size).
**MuJoCo 3.5.0 verified:** Yes (EGT-1).

### T3: Explicit nuser_* with zero-padding → AC3
**Model:** `<size nuser_geom="5"/>`, geom g1 `user="1.5 2.5"`, geom g2
no user.
**Assert:** `nuser_geom=5`, `geom_user[g1]=[1.5,2.5,0,0,0]`,
`geom_user[g2]=[0,0,0,0,0]`.
**MuJoCo 3.5.0 verified:** Yes (EGT-1).

### T4: Too-long user data validation → AC4
**Model:** `<size nuser_geom="2"/>`, geom `user="10 20 30 40"` (4 > 2).
**Assert:** Build error containing "user has more values than nuser_geom".
**MuJoCo 3.5.0 verified:** Yes (EGT-2).

### T5: Default class inheritance with full replacement → AC5
**Model:** Default class with `joint user="100 200"`. Joint j1 no user
attr, joint j2 `user="300"`.
**Assert:** `jnt_user[j1]=[100,200]` (inherited), `jnt_user[j2]=[300,0]`
(override, padded to nuser_jnt=2). Full replacement — not partial merge.
**MuJoCo 3.5.0 verified:** Yes (EGT-3).

### T6: No-user-data model → AC6
**Model:** Simple model with a body, joint, geom — no user attributes
anywhere, no `<size>` element.
**Assert:** All `nuser_*=0`. All `*_user` arrays populated but each inner
vec is empty (length 0).
**MuJoCo 3.5.0 verified:** Yes (EGT-1).

### T7: Sensor user data wired to Model → AC8
**Model:** Joint j1 + `<sensor><jointpos joint="j1" user="5 10"/></sensor>`
**Assert:** `sensor_user[0]=[5,10]`, `nuser_sensor=2`.
**MuJoCo 3.5.0 verified:** Yes.

### T8: user="" and user="0" semantics → AC9, AC10
**Model:** Default `<geom user="5 6 7"/>`. Geom g1 with `user=""`, geom
g2 with `user="0"`.
**Assert:** g1 gets default `[5,6,7]` (empty string = not specified);
g2 gets `[0,0,0]` (user="0" is data, overrides default, padded to
nuser_geom=3).
**MuJoCo 3.5.0 verified:** Yes (EGT-3a, EGT-3c).

### T9: Actuator subtype defaults last-write-wins → AC11
**Model:** Default class with `<general user="100"/>` then
`<motor user="200"/>`. Motor actuator with no user.
**Assert:** Actuator gets `[200]`. The `<motor>` overwrites `<general>`
within the same default class.
**MuJoCo 3.5.0 verified:** Yes (EGT-3b).

### T10: Transitive 3-level default inheritance → AC12
**Model:** Class A: `joint user="1 2 3"`. Class B (child of A): no joint
user. Class C (child of B): no joint user. Joint using class C.
**Assert:** Joint gets `[1, 2, 3]` from class A through B and C.
**MuJoCo 3.5.0 verified:** Yes (EGT-3d).

### T11: All 7 element types comprehensive → AC13
**Model:** Single model with user data on body, geom, joint, site, tendon,
actuator, sensor — all with different lengths and values.
**Assert:** All 7 `*_user` arrays populated, all 7 `nuser_*` > 0. Each
type's user data matches the MJCF input.
**MuJoCo 3.5.0 verified:** Yes (EGT-3f golden model 1).

### T12: Too-long inherited default vs explicit override → AC14, AC15
**Model:** `<size nuser_jnt="2"/>`, default `joint user="1 2 3"` (3 vals).
Sub-test A: Joint j1 inherits (no user) → error (3 > 2).
Sub-test B: Joint j1 with `user="10 20"` → success (override is 2 ≤ 2).
**MuJoCo 3.5.0 verified:** Yes (EGT-3a, EGT-3e).

### T13: Flexcomp scale + quat transform → AC16
**Model:** `<flexcomp type="grid" count="2 2" spacing="1.0"
scale="2 3 1" quat="0.7071068 0 0 0.7071068"/>`
**Assert:** Vertices after transform: `(0,0,0), (0,2,0), (-3,0,0),
(-3,2,0)` (± 1e-10). Scale applied first (component-wise), then rotation.
**Analytically derived.** (No MuJoCo binary to verify.)

### T14: Flexcomp inertiabox and file → AC17
**Model:** `<flexcomp type="grid" count="2 2" spacing="1.0"
inertiabox="0.5" file="mesh.obj"/>`
**Assert:** `flex.inertiabox == 0.5`, `flex.flexcomp_file == Some("mesh.obj")`.
**Analytically derived.**

### T15: User data physics inertness → AC19
**Model:** Two identical models with a hinge joint + motor. Model A has
user data on body, geom, joint, sensor. Model B has none. Run 10 steps
with `ctrl[0] = 1.0`.
**Assert:** `qpos` and `qvel` are bit-identical between A and B.
**MuJoCo 3.5.0 verified:** Yes (EGT-3c).

### T16: `childclass` propagation with nearest-ancestor rule → AC21
**Model:** Class "A" with `geom user="1 2"`. Class "B" with
`geom user="10 20 30"`. Body1 `childclass="A"` with geom g1, child Body2
`childclass="B"` with geom g2.
**Assert:** `geom_user[g1]=[1,2,0]` (from A, padded to nuser=3),
`geom_user[g2]=[10,20,30]` (from B, nearest ancestor).
**MuJoCo 3.5.0 verified:** Yes (EGT-3b/3c).

### T17: `childclass` does not shadow root default → AC22
**Model:** Root default `joint user="1 2 3"`. Class "myclass" (child of
root) defines NO joint user. Body with `childclass="myclass"`, joint j1.
**Assert:** `jnt_user[j1]=[1,2,3]` (inherited from root through class chain).
**MuJoCo 3.5.0 verified:** Yes (EGT-3d).

### T18: `nuser_*` validation edge cases → AC23
**Sub-test A:** `<size nuser_geom="-2"/>` → build error (nuser must be >= -1).
**Sub-test B:** `<size nuser_geom="0"/>` with `<geom user="1 2 3"/>` →
error (user data length 3 > nuser 0).
**Sub-test C:** `<size nuser_geom="0"/>` with no user data → success,
nuser_geom=0, geom_user arrays have zero-width inner vecs.
**MuJoCo 3.5.0 verified:** Yes (EGT-3b).

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| World body user data (always zeros) | Body 0 is implicit — can't have user attr. Must be zeros. | T2 | AC7 |
| No-user-data model (nuser=0) | Zero-width arrays must still be correct shape | T6 | AC6 |
| Too-long user array error | Validation prevents memory overrun | T4 | AC4 |
| Default inheritance with no override | Element omits `user` entirely — inherits default class user data unchanged | T5 (j1) | AC5 |
| Default inheritance with shorter override | Full replacement, not partial merge | T5 | AC5 |
| Single-value user (e.g., `user="42"`) | Must produce 1-element array, not error | T5 (j2 `user="300"`) | AC5 |
| `user=""` with defaults (inherits) | Empty string = not specified, defaults cascade applies | T8 | AC9 |
| `user="0"` is data, not absence | Semantic distinction: 0.0 is data, nuser=1 not 0 | T8 | AC10 |
| `<size nuser_*="0">` explicit zero | Zero-width arrays are legal when no user data exists | T18 (sub-test C) | AC23 |
| `<size nuser_*="0">` with user data | Must error — length > 0 exceeds explicit nuser=0 | T18 (sub-test B) | AC23 |
| `nuser_*=-2` rejected | Only -1 is valid negative value | T18 (sub-test A) | AC23 |
| Actuator subtype last-write-wins | MuJoCo aliases `<motor>`/`<general>` etc. within default class | T9 | AC11 |
| Transitive 3-level inheritance | Chain A→B→C must propagate through non-overriding intermediates | T10 | AC12 |
| `childclass` propagation nearest-ancestor | Inner childclass overrides outer for user data | T16 | AC21 |
| `childclass` does not shadow root default | Class selects, does not block parent inheritance | T17 | AC22 |
| `class="main"` references root default | Escape from childclass scope | T17 (variant) | AC22 |
| Too-long inherited default | Validation on effective (post-cascade) data | T12 | AC14 |
| Explicit override avoids too-long default | Override shields from default length check | T12 | AC15 |
| Multiple `<size>` elements merge | Last-writer-wins per attribute | T1 | AC18 |
| nuser_*=-1 explicit auto-size | Sentinel triggers same behavior as omitting | T2 | AC20 |
| `<freejoint>` does not accept `user` | Reduced attribute set — use `<joint type="free">` | — (code review) | AC24 |
| User data physics inertness | Must not affect simulation output | T15 | AC19 |
| Depth-first element ordering in `*_user` arrays | body_user[0] is world, then DFS pre-order | T2, T11 | AC2, AC13 |
| Multiple elements with different user lengths | Auto-sizing picks max, shorter arrays padded | T2 | AC2 |
| Auto-sizing from defaults only (no inline user) | Default class user length drives nuser_* when elements inherit | T5 | AC5 |
| NaN/Inf in user data | MuJoCo accepts with warning — Rust f64 parsing handles NaN/Inf | — (out of scope for error matching; Rust accepts these) | — |
| Hex notation (`0xFF`) in user data | MuJoCo accepts via strtod; Rust f64::parse does not | — (known deviation, see Out of Scope) | AC26 |
| Whitespace variants (tabs, newlines) | MuJoCo accepts standard XML whitespace separators | — (handled by XML parser, no special code needed) | — |
| Commas/semicolons rejected as separators | Only whitespace separators valid | — (handled by `parse_float_array` whitespace split) | — |
| `nuser_*` with float value rejected | `nuser_geom="2.5"` must not parse as integer — strict integer validation | T18 | AC23 |
| Override is full replacement not partial merge | Element `user="1"` replaces default `user="10 20 30"` entirely — result is `[1]`, not `[1, 20, 30]` | T5 | AC5 |
| Denormal float values | MuJoCo rejects some; Rust may accept | — (known deviation, see Out of Scope) | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T8 (user="" / user="0") | Parser semantics for empty vs zero | Critical semantic distinction — prevents defaults cascade bugs |
| T15 (physics inertness) | Runtime behavior with user data | Guarantees user data is truly read-only from physics perspective |
| T18 (nuser validation) | Boundary conditions for nuser_* values | Prevents invalid nuser values from corrupting Model state |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `<size>` element parsing | Silently skipped (`skip_element`) | Parsed for `nuser_*` attributes | Toward MuJoCo | No downstream consumers — new data | None — transparent |
| Body/geom/joint/site/tendon/actuator `user` attribute | Silently ignored | Parsed and stored on MJCF type structs | Toward MuJoCo | No downstream consumers — new data | None — transparent |
| Model `*_user` arrays | Don't exist | 7 new `Vec<Vec<f64>>` arrays on Model | Toward MuJoCo | No downstream consumers — new data | None — transparent |
| Model `nuser_*` fields | Don't exist | 7 new `i32` fields on Model | Toward MuJoCo | No downstream consumers — new data | None — transparent |
| Sensor user data | Parsed but discarded in builder | Parsed AND wired to `Model.sensor_user` | Toward MuJoCo | No downstream consumers | None — was already parsed, now stored |
| 5 defaults structs | No `user` field | `user: Option<Vec<f64>>` added | Toward MuJoCo | Existing defaults tests — minor additions only | None — additive |
| Flexcomp vertices | No scale/quat transform | Scale then rotate applied | Toward MuJoCo docs | Flexcomp tests with scale/quat | Update expected values if tests use scale/quat (unlikely — none exist currently) |
| Too-long user data | Silently accepted (no user data parsing) | Error on build | Toward MuJoCo | Models with user data where len > nuser_* | Fix model or increase nuser_* |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | Add `user: Vec<f64>` to 6 structs, `user: Option<Vec<f64>>` to 5 defaults structs, `nuser_*` to MjcfModel, 4 fields to MjcfFlex | ~+40 |
| `sim/L0/mjcf/src/parser.rs` | `<size>` parsing, `user` in 6 parse functions + 5 defaults parse functions, flexcomp 4 attrs, `apply_flexcomp_transforms` | ~+80 |
| `sim/L0/mjcf/src/defaults.rs` | Add `user` merge logic to 5 merge functions, apply logic to 5 apply functions | ~+30 |
| `sim/L0/core/src/types/model.rs` | Add 7 `*_user` arrays + 7 `nuser_*` fields | ~+20 |
| `sim/L0/core/src/types/model_init.rs` | Initialize 14 new fields | ~+14 |
| `sim/L0/mjcf/src/builder/init.rs` | Add 7 `*_user_raw` arrays + world body init | ~+10 |
| `sim/L0/mjcf/src/builder/body.rs` | Push `body_user_raw` | ~+2 |
| `sim/L0/mjcf/src/builder/geom.rs` | Push `geom_user_raw` and `site_user_raw` | ~+4 |
| `sim/L0/mjcf/src/builder/joint.rs` | Push `jnt_user_raw` | ~+2 |
| `sim/L0/mjcf/src/builder/tendon.rs` | Push `tendon_user_raw` | ~+2 |
| `sim/L0/mjcf/src/builder/actuator.rs` | Push `actuator_user_raw` | ~+2 |
| `sim/L0/mjcf/src/builder/sensor.rs` | Push `sensor_user_raw` | ~+2 |
| `sim/L0/mjcf/src/builder/mod.rs` | `finalize_user_data()`, call in pipeline | ~+60 |
| `sim/L0/mjcf/src/builder/build.rs` | Transfer user arrays to Model in `build()` | ~+20 |
| Test files (new) | 15 tests across sim-mjcf and sim-conformance-tests | ~+400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All flexcomp tests | `sim/L0/mjcf/src/parser.rs` tests | Pass (unchanged) | No existing tests use scale/quat/inertiabox/file on flexcomp |
| All sensor defaults tests | `defaults.rs` tests | Pass (unchanged) | Sensor user defaults code path not modified |
| All joint/geom/site/tendon/actuator defaults tests | `defaults.rs` tests | Pass (unchanged) | New `user` field defaults to `None` — no change to existing merge/apply behavior |
| All body/geom/joint parsing tests | `parser.rs` tests | Pass (unchanged) | New `user` field defaults to empty vec — no change to existing parse results |
| Model serialization tests | Various | May need update if Model serialization includes new fields | Add user fields to expected serialized output |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `defaults.rs:641` (`apply_to_sensor`) | Applies sensor defaults including user | Already handles user correctly — do not modify |
| `defaults.rs:945` (`merge_sensor_defaults`) | Merges sensor defaults including user | Already handles user correctly — do not modify |
| `parser.rs:3630` (`parse_sensor_attrs` user) | Parses sensor user attribute | Already implemented — do not modify |
| `parser.rs:962` (`parse_sensor_defaults` user) | Parses sensor defaults user | Already implemented — do not modify |
| `types.rs:3178` (`MjcfSensor.user`) | Sensor user field | Already exists — do not modify |
| `types.rs:815` (`MjcfSensorDefaults.user`) | Sensor defaults user field | Already exists — do not modify |

---

## Execution Order

1. **S1 (`<size>` parsing + MjcfModel nuser_*)** → Foundation. Must land
   first because S6 reads nuser_* values from MjcfModel. Verify: parse
   test with multiple `<size>` elements.

2. **S2 (MJCF type structs)** → Independent of S1. Can parallelize but
   ordered here for clarity. Verify: struct compilation.

3. **S3 (Parser user attribute)** → Depends on S2 (writes to struct
   fields added in S2). Verify: parse round-trip tests.

4. **S4 (Defaults)** → Depends on S2 (reads/writes struct fields) and S3
   (parse functions for defaults). Verify: defaults merge + apply tests.

5. **S5 (Model fields)** → Independent of S1–S4. Can parallelize. Verify:
   Model compilation + model_init.

6. **S6 (Builder wiring)** → Depends on ALL previous sections. Builder
   reads parsed user data (S2–S4), nuser_* (S1), and writes to Model
   (S5). This is the integration section. Verify: full pipeline tests
   (parse → defaults → build → check Model fields).

7. **S7 (Flexcomp)** → Independent of S1–S6 (different code paths). Verify:
   flexcomp parse + transform tests.

After each section lands, run:
```
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests
```

---

## Out of Scope

- **Camera user data** (`cam_user`/`nuser_cam`) — Umbrella lists 7 types;
  camera is the 8th. Track as DT-126 if needed. *Conformance impact:
  minor — camera user data rarely used.*

- **Flexcomp pos/axisangle/xyaxis/zaxis/euler/origin** — Umbrella lists
  only 4 attributes (inertiabox, scale, quat, file). These additional
  flexcomp orientation alternatives are tracked as future work.
  *Conformance impact: minor — alternative orientations are convenience.*

- **Flexcomp file loading** — DT-88 parses and stores the `file` path.
  Actual mesh/grid file loading is a separate task. *Conformance impact:
  the path is stored; loading deferred.*

- **nuserdata** (global runtime scratch array) — Completely separate from
  nuser_* per-element user data. Controlled by `<size nuserdata="N">`.
  Not part of §55. *Conformance impact: none for element user data.*

- **Hex notation in user data** — MuJoCo accepts `0xFF` via strtod.
  Rust's `f64::parse` does not support hex. This is a known deviation.
  Standard float notation covers all practical use cases. Track if needed.
  *Conformance impact: negligible — hex in user data is extremely rare.*

- **Denormal/subnormal float rejection** — MuJoCo rejects some subnormal
  values. Rust's parser may accept them. Minor deviation. *Conformance
  impact: negligible — subnormal user data values have no practical use.*

- **`nuser_*` range validation** — MuJoCo validates `>= -1` and
  `< INT_MAX`. Implementation should validate but exact error messages
  are not required to match MuJoCo's. *Conformance impact: none — valid
  models will work identically.*

- **User data mutability at runtime** — MuJoCo allows modifying `*_user`
  arrays after compilation. CortenForge Model fields are inherently
  mutable in Rust. No additional work needed.
