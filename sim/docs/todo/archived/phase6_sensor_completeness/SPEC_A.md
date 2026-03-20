# Phase 6 Spec A — objtype Parsing + XBody, Touch Multi-Geom, Geom Acc Sensors

**Status:** Draft
**Phase:** Roadmap Phase 6 — Sensor Completeness
**Effort:** M
**MuJoCo ref:** `get_xpos_xmat()`, `get_xquat()`, `mj_sensorAcc()` in `engine_sensor.c`;
`mj_objectAcceleration()` in `engine_sensor.c`; `mj_contactForce()` in `engine_support.c`
**MuJoCo version:** 3.5.0
**Prerequisites:**
- Phase 5 Actuator Completeness (landed in `3acd01a`)
- DT-103 spatial transport utilities (landed — `object_acceleration()` at `spatial.rs:286–323`)
- Test baseline: 2,238+ domain tests post-Phase 5

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Problem Statement

Three conformance gaps in CortenForge's sensor pipeline:

**DT-62 — Frame sensor `objtype` attribute not parsed.** MuJoCo's MJCF schema
allows `<framepos objtype="body" objname="b1"/>` to explicitly specify whether a
frame sensor reads from a body, xbody, geom, site, or camera. CortenForge's parser
(`parser.rs:3463–3468`) ignores the `objtype` attribute entirely and uses a
site→body→geom name-guessing heuristic in the builder (`builder/sensor.rs:166–186`).
This produces wrong results when the same name exists across multiple object types
or when the user wants a specific object type (e.g., `body` COM frame vs `xbody`
joint frame — a 0.15m error for bodies with offset COM per EGT-1 in the rubric).
Additionally, the parser's `objname` fallback chain (`parser.rs:3463–3468`) does
not try the `geom=` attribute, so `<framepos geom="g1"/>` is silently lost. The
parser also conflates `reftype` and `refname` into a single field at
`parser.rs:3470`, which must be separated for Spec B.

**Conformance gap** — MuJoCo resolves frame sensors via `sensor_objtype[i]` and
`sensor_objid[i]` in `get_xpos_xmat()` / `get_xquat()`; CortenForge guesses.

**DT-64 — Multi-geom touch sensor aggregation.** MuJoCo's touch sensor
(`mj_sensorAcc()` in `engine_sensor.c`) resolves `site_bodyid[objid]` at runtime
and iterates ALL contacts, checking body membership via `geom_bodyid[con->geom[k]]`.
CortenForge's builder (`builder/sensor.rs:140–163`) resolves site→body→**first geom
on body** and stores `(MjObjectType::Geom, first_geom_id)`. Evaluation
(`acceleration.rs:143–181`) then checks `c.geom1 == objid || c.geom2 == objid`
against that single geom ID — contacts on other geoms are silently dropped.

**Correctness bug** — multi-geom bodies produce wrong touch sensor values.

**DT-102 — Geom-attached FrameLinAcc/FrameAngAcc.** MuJoCo's `mj_sensorAcc()`
passes any object type to `mj_objectAcceleration()`, including `mjOBJ_GEOM`.
CortenForge's acceleration arms (`acceleration.rs:207–220`, `222–238`) only match
`Site` and `Body`; geom falls through to the `_ => zeros` catch-all.

**Conformance gap** — geom-attached acceleration sensors return zeros instead of
computed values.

---

## MuJoCo Reference

### DT-62: Frame sensor object type resolution

**Source:** `engine_sensor.c`, `get_xpos_xmat()` helper (called from
`mj_sensorPos()` for FramePos/FrameQuat/FrameAxis sensors).

`get_xpos_xmat()` dispatches on `m->sensor_objtype[i]` to select position and
rotation data sources:

| `sensor_objtype[i]` | MuJoCo enum | Position source | Rotation source |
|---------------------|-------------|----------------|-----------------|
| `mjOBJ_XBODY` (2) | XBody | `d->xpos + 3*id` (joint frame origin) | `d->xmat + 9*id` |
| `mjOBJ_BODY` (1) | Body | `d->xipos + 3*id` (inertial/COM frame) | `d->ximat + 9*id` |
| `mjOBJ_GEOM` (5) | Geom | `d->geom_xpos + 3*id` | `d->geom_xmat + 9*id` |
| `mjOBJ_SITE` (6) | Site | `d->site_xpos + 3*id` | `d->site_xmat + 9*id` |
| `mjOBJ_CAMERA` (7) | Camera | `d->cam_xpos + 3*id` | `d->cam_xmat + 9*id` |

`get_xquat()` (called for FrameQuat) dispatches similarly:

| `sensor_objtype[i]` | Quaternion computation |
|---------------------|----------------------|
| `mjOBJ_XBODY` | `d->xquat[id]` (direct copy) |
| `mjOBJ_BODY` | `mju_mulQuat(d->xquat[body], m->body_iquat[body])` |
| `mjOBJ_GEOM` | `mju_mulQuat(d->xquat[geom_bodyid], m->geom_quat[geom])` |
| `mjOBJ_SITE` | `mju_mulQuat(d->xquat[site_bodyid], m->site_quat[site])` |
| `mjOBJ_CAMERA` | `mju_mulQuat(d->xquat[cam_bodyid], m->cam_quat[cam])` |

**Empirical verification (MuJoCo 3.5.0, from rubric EGT-1):** Body with capsule
geom (mass=1, COM offset 0.15m from joint):
```
framepos objtype="body"  → xipos = [0.15, 0.0, 1.0]  (COM position)
framepos objtype="xbody" → xpos  = [0.0,  0.0, 1.0]  (joint frame origin)
Difference = [-0.15, 0.0, 0.0]
```

**MJCF attribute inference:** When the `objtype` attribute is omitted, MuJoCo
infers the object type from which MJCF attribute was used:
- `site="name"` → `mjOBJ_SITE`
- `body="name"` → `mjOBJ_XBODY` (NOT `mjOBJ_BODY`)
- `geom="name"` → `mjOBJ_GEOM`
- `objname="name"` → type determined by sensor type default

**CortenForge current state:**
- `MjObjectType` enum (`enums.rs:494–512`) has `Body`, `Joint`, `Geom`, `Site`,
  `Actuator`, `Tendon`, `None`. Missing: `XBody`, `Camera`.
- Current `Body` uses `xpos`/`xmat` (joint frame), which is MuJoCo's `mjOBJ_XBODY`.
- Parser (`parser.rs:3463–3468`) collapses `site=`, `body=`, `geom=` (missing!),
  `objname=` into a single `objname` field, losing provenance.
- Builder (`builder/sensor.rs:166–186`) uses heuristic: try site → body → geom
  name lookup.

### DT-64: Touch sensor multi-geom aggregation

**Source:** `engine_sensor.c`, `mj_sensorAcc()`, `mjSENS_TOUCH` case.

```c
case mjSENS_TOUCH:
  bodyid = m->site_bodyid[objid];   // objid = site ID, resolve to body
  sensordata[0] = 0;

  for (int j=0; j < d->ncon; j++) {
    con = d->contact + j;
    int conbody[2];
    for (int k=0; k < 2; k++) {
      conbody[k] = (con->geom[k] >= 0) ? m->geom_bodyid[con->geom[k]] : -1;
    }

    if (con->efc_address >= 0 &&
        (bodyid == conbody[0] || bodyid == conbody[1])) {
      mj_contactForce(m, d, j, conforce);
      if (conforce[0] <= 0) continue;

      mju_scl3(conray, con->frame, conforce[0]);
      mju_normalize3(conray);
      if (bodyid == conbody[1]) mju_scl3(conray, conray, -1);

      if (mju_rayGeom(d->site_xpos+3*objid, d->site_xmat+9*objid,
                      m->site_size+3*objid, con->pos, conray,
                      m->site_type[objid], NULL) >= 0) {
        sensordata[0] += conforce[0];
      }
    }
  }
  break;
```

**MuJoCo touch sensor has 4 algorithm layers:**

1. **Body-level contact iteration:** `bodyid = site_bodyid[objid]` then iterate
   `d->ncon` contacts, checking `geom_bodyid[con->geom[k]] == bodyid`. ALL geoms
   on the body contribute.

2. **Contact force computation via `mj_contactForce()`:** Reconstructs the
   physical normal force from the pyramidal/elliptic/frictionless constraint force
   basis. This is NOT equivalent to summing `efc_force` rows. For pyramidal contacts
   with `dim=3`, `mj_contactForce()` produces ~33% higher normal force than raw
   `sum(efc_force)` (EGT-2a: 9.808 vs 7.356).

3. **Ray direction construction:** `conray = normalize(con->frame * conforce[0])`
   (contact normal scaled by normal force, normalized), with sign flip when
   `bodyid == conbody[1]`.

4. **Ray-geom intersection filter via `mju_rayGeom()`:** Contact contributes only
   if a ray from the contact point along `conray` intersects the sensor site's
   geometric volume. This filters contacts outside the site's spatial extent.

**CortenForge current state:**
- Builder (`builder/sensor.rs:140–163`): resolves site→body→**first geom** via
  `geom_body.iter().position(|&b| b == body_id)`. Stores `(MjObjectType::Geom,
  first_geom_id)`.
- Evaluation (`acceleration.rs:143–181`): checks `c.geom1 == objid || c.geom2 ==
  objid` against single geom ID. No `mj_contactForce()` — reads `efc_force`
  directly. No ray-geom intersection filter.

### DT-102: Geom-attached acceleration sensors

**Source:** `engine_sensor.c`, `mj_sensorAcc()`, `mjSENS_FRAMELINACC` /
`mjSENS_FRAMEANGACC` cases.

```c
case mjSENS_FRAMELINACC:
case mjSENS_FRAMEANGACC:
  mj_objectAcceleration(m, d, objtype, objid, tmp, 0);
  if (type == mjSENS_FRAMELINACC) {
    mju_copy3(sensordata, tmp+3);   // linear = elements [3..6]
  } else {
    mju_copy3(sensordata, tmp);     // angular = elements [0..3]
  }
  break;
```

`mj_objectAcceleration()` handles `mjOBJ_GEOM`: resolves `body_id =
m->geom_bodyid[objid]`, target position `d->geom_xpos[objid]`, applies spatial
transport from body origin to geom position, adds Coriolis correction.

**Important:** FrameLinAcc/FrameAngAcc **ignore refid** — no reference-frame
transform in the acceleration case. Confirmed in C source.

**CortenForge current state:** `acceleration.rs:207–238` — FrameLinAcc and
FrameAngAcc match only `Site` and `Body`. The `_` catch-all writes zeros.
`object_acceleration()` at `spatial.rs:286–323` already handles the full
computation (spatial transport + Coriolis) — we just need to pass the correct
geom position and body ID.

### Edge cases

- **World body (`body_id == 0`):** `cacc[0]` is zero (world body has no parent).
  Sensors attached to world body geoms/sites correctly produce gravity-compensated
  values only if `mj_body_accumulators` zeroes `cacc[0]`.
- **Sleeping bodies:** `mj_sensorAcc()` skips sensors on sleeping bodies (sleep
  state check before evaluation). CortenForge already implements this at
  `acceleration.rs:52–58`.
- **`mjDSBL_SENSOR` flag:** Early return — sensordata is NOT zeroed. CortenForge
  already implements this at `acceleration.rs:31–33`.
- **`flg_rnepost` lazy gate:** Triggers `mj_body_accumulators` for FrameLinAcc/
  FrameAngAcc. CortenForge already implements this at `acceleration.rs:60–79`.
  Geom acc sensors need the same gate — already covered since they use the
  FrameLinAcc/FrameAngAcc match arms.
- **`efc_address >= 0` guard:** MuJoCo skips contacts without active constraints.
  CortenForge's contact iteration already requires `ConstraintType::Contact*`
  match, which inherently only reaches active contacts.
- **Bodyless geom:** In valid MJCF, every geom has a parent body. If `geom_body[objid]`
  is somehow invalid, bounds check prevents out-of-bounds access.
- **Zero-mass body:** MuJoCo sets `xipos`/`ximat` to the joint frame origin for
  massless bodies. CortenForge must verify this behavior is consistent.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Frame sensor objtype dispatch | Explicit `sensor_objtype[i]` set from MJCF `objtype` attribute, with `get_xpos_xmat()` / `get_xquat()` dispatch. 5 object types: `mjOBJ_XBODY`, `mjOBJ_BODY`, `mjOBJ_GEOM`, `mjOBJ_SITE`, `mjOBJ_CAMERA`. | No `objtype` attribute parsed. Builder guesses from name: site→body→geom heuristic (`builder/sensor.rs:166–186`). Only 3 types matched in evaluation. |
| `body` vs `xbody` distinction | `mjOBJ_BODY` reads `xipos`/`ximat` (COM/inertial frame); `mjOBJ_XBODY` reads `xpos`/`xmat` (joint frame). 0.15m difference for offset COM. | Only `MjObjectType::Body`, which reads `xpos`/`xmat` = MuJoCo's `mjOBJ_XBODY`. No inertial/COM frame option. |
| FrameQuat per objtype | `get_xquat()` computes per-type quaternion: `BODY` = `mulQuat(xquat, body_iquat)`, `GEOM` = `mulQuat(xquat[geom_bodyid], geom_quat)`, `SITE` = `mulQuat(xquat[site_bodyid], site_quat)`. | Site and Geom compute from rotation matrix via `from_matrix_unchecked`. Body reads `xquat` directly (= MuJoCo's XBODY behavior). |
| Touch sensor scope | Body-level: `site_bodyid[objid]` → iterate all contacts → `geom_bodyid[con->geom[k]] == bodyid`. ALL geoms contribute. | Single-geom: stores `(Geom, first_geom_id)`. Only contacts on first geom counted. |
| Touch force computation | `mj_contactForce()` reconstructs physical normal force from constraint basis. 33% higher than raw `efc_force` sum for pyramidal. | Reads `efc_force` directly. Sums all facet forces for pyramidal. Numerically wrong for pyramidal contacts. |
| Touch ray-geom filter | `mju_rayGeom()` filters contacts outside sensor site's volume. | No filter — all contacts on the geom summed. |
| Geom-attached FrameLinAcc | `mj_objectAcceleration(m, d, mjOBJ_GEOM, objid, tmp, 0)` → full spatial transport + Coriolis at geom position. | `_ => zeros`. Geom not matched. |
| Geom-attached FrameAngAcc | `mj_objectAcceleration(m, d, mjOBJ_GEOM, objid, tmp, 0)` → angular from body's `cacc[0..3]`. | `_ => zeros`. Geom not matched. |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `mjOBJ_BODY` (1) | Inertial/COM frame: reads `xipos`, `ximat` | `MjObjectType::Body` reads `xpos`, `xmat` (= MuJoCo's XBODY) | **After this spec:** add `MjObjectType::XBody` for joint frame. Rename behavior: current `Body` match arms that read `xpos`/`xmat` are semantically `XBody`. New `Body` arms should read `xipos`/`ximat` when `MjObjectType::Body` is explicitly specified via `objtype="body"`. See DECISION 1. |
| `mjOBJ_XBODY` (2) | Joint frame: reads `xpos`, `xmat` | Not in enum. Current `Body` behavior matches. | **After this spec:** `MjObjectType::XBody` = current `Body` data paths. When MJCF says `body="name"` (without `objtype`), default to `XBody` to match MuJoCo inference. |
| `sensor_objtype` array | Index into `mjtObj` enum | `Vec<MjObjectType>` in `Model` | Direct port — same semantics. |
| `sensor_objid` array | Integer ID into type-specific array | `Vec<usize>` in `Model` | Direct port — same semantics. |
| `SpatialVector` layout | `tmp[0..3]` = angular, `tmp[3..6]` = linear | `cacc[body_id]` is `[f64; 6]`, `[0..3]` = angular, `[3..6]` = linear | Direct port — no translation needed. |
| `mj_contactForce()` | Reconstructs physical normal force from pyramidal/elliptic/frictionless basis | No equivalent — reads `efc_force` directly | **Deferred** — see DECISION 3. For frictionless: `efc_force[0]` IS normal force. For elliptic: first row IS normal force. For pyramidal: reconstruction needed (33% error without it). |
| Contact `con->frame[0..3]` | First 3 floats of a 3×3 rotation matrix = contact normal direction | `Contact.normal` is a separate `Vector3<f64>` field; `Contact.frame` is `[Vector3<f64>; 2]` containing tangent vectors only | Use `contact.normal` wherever MuJoCo reads `con->frame[0..3]` for the normal direction. |
| `con->geom[0]`/`con->geom[1]` | Two geom IDs per contact | `Contact.geom1`/`Contact.geom2` (`contact_types.rs:48–50`) | Direct port — `c.geom1` = `con->geom[0]`, `c.geom2` = `con->geom[1]`. |
| `model.geom_bodyid` | `m->geom_bodyid[geom_id]` → body owning the geom | `model.geom_body[geom_id]` (`model.rs:253`) | Use `model.geom_body` wherever MuJoCo uses `m->geom_bodyid`. |
| `model.site_bodyid` | `m->site_bodyid[site_id]` → body owning the site | `model.site_body[site_id]` (`model.rs:474`) | Use `model.site_body` wherever MuJoCo uses `m->site_bodyid`. |
| Contact normal direction | MuJoCo `con->frame[0..3]` points from geom2 to geom1 (by convention — the first row of the contact frame is the normal). | `Contact.normal` — verify points in same direction. Touch sensor sign-flip logic depends on this. | Direct port if directions match. If CortenForge normal points geom1→geom2, the sign-flip condition reverses (`bodyid == conbody[0]` instead of `conbody[1]`). Tests will catch this. |
| Contact iteration pattern | MuJoCo iterates `d->contact[j]` (j=0..ncon), checks `con->efc_address >= 0`. | CortenForge iterates `data.efc_type[ei]` forward, uses `data.efc_id[ei]` → `data.contacts[ci]`. Structurally inverted. | Touch rewrite must use CortenForge's iteration pattern (iterate `data.contacts[ci]` and check `data.efc_*` for force data), or iterate `data.contacts` directly and look up EFC data per contact. See S2 for implementation. |

---

## Specification

### Architectural Decisions

**DECISION 1: `XBody` variant — add now.**

Add `MjObjectType::XBody` to the enum. Rationale: the `body` vs `xbody`
distinction produces a real 0.15m conformance error for offset-COM bodies (EGT-1).
Without `XBody`, we cannot distinguish `objtype="body"` from `objtype="xbody"` in
the builder. Current `MjObjectType::Body` reads `xpos`/`xmat` = MuJoCo's XBODY.

Implementation: add `XBody` variant. Audit all match sites (EGT-5 from rubric):
- `sensor_body_id()` at `mod.rs:27`: add `XBody => Some(objid)` (same as Body).
- `acceleration.rs` FrameLinAcc (line 211): `XBody` arm reads `(objid, data.xpos[objid])` (same as current Body).
- `acceleration.rs` FrameAngAcc (line 228): `XBody` arm reads `objid` (same as current Body).
- `position.rs` FramePos (line 109): current Body arm reads `data.xpos` → this IS XBody. Add new Body arm reading `data.xipos`.
- `position.rs` FrameQuat (line 120): current Body arm reads `data.xquat` directly → this IS XBody. Add new Body arm computing `mulQuat(xquat, body_iquat)`.
- `position.rs` FrameAxis (line 143): current Body arm reads `data.xmat` → this IS XBody. Add new Body arm reading `data.ximat`.
- `velocity.rs` FrameLinVel (line 134): current Body arm reads `data.xpos` → this IS XBody. Add new Body arm reading `data.xipos`.
- `velocity.rs` FrameAngVel (line 150): angular velocity is body-level (reference-point-independent). `XBody` and `Body` read same `cvel`. No change needed — just add arm.
- `velocity.rs` Velocimeter (line 116): current Body arm reads `data.xpos`, `data.xmat` → this IS XBody. Add new Body arm reading `data.xipos`, `data.ximat`.

Default inference: when MJCF uses `body="name"` without `objtype` attribute, the
builder stores `XBody` (matching MuJoCo's inference: `body=` → `mjOBJ_XBODY`).

**DECISION 2: `Camera` variant — defer.**

Defer `MjObjectType::Camera` to a new DT-ID (DT-120). Rationale: CortenForge does
not yet have `cam_xpos`, `cam_xmat`, or `cam_quat` fields in `Data`. Adding camera
support requires populating these during forward kinematics. Frame sensors attached
to cameras are rare in RL/robotics models. If `objtype="camera"` is encountered,
the builder emits a warning and falls back to the current resolution heuristic.

**DECISION 3: `mj_contactForce()` — defer with quantitative impact.**

Defer full `mj_contactForce()` implementation to DT-118. Rationale: the
reconstruction algorithm depends on contact model (pyramidal vs elliptic vs
frictionless), contact dimension, and friction model — moderate complexity.

**Impact analysis:**
- **Frictionless contacts** (`dim=1`): `efc_force[0]` IS the normal force.
  CortenForge is **correct** for frictionless contacts.
- **Elliptic contacts** (`dim=3` or `dim=4`): first EFC row IS the normal force.
  CortenForge is **correct** for elliptic contacts.
- **Pyramidal contacts** (`dim=3`): `efc_force` rows are facet projections onto
  pyramid edges. Summing them produces ~75% of the true normal force
  (EGT-2a: 7.356 vs 9.808 = ratio 0.75). CortenForge is **numerically wrong**
  for pyramidal contacts. The default contact model in MuJoCo is **pyramidal**,
  so this affects most models.

The spec will implement body-level contact iteration (layer 1) but continue
reading `efc_force` directly (layers 2–4 deferred). This fixes the multi-geom
aggregation bug while leaving the pyramidal force magnitude error for DT-118.

**DECISION 4: Ray-geom intersection filter — defer.**

Defer to DT-119. Impact: without the filter, touch sensors over-report for small
sensor sites on large bodies (all contacts on the body contribute regardless of
spatial proximity to the site). For typical models where the sensor site covers
the full body, the difference is zero.

### S1. Parser + types: `objtype` attribute, `geom=` attribute, `reftype`/`refname` separation (DT-62)

**File:** `sim/L0/mjcf/src/parser.rs`, lines 3452–3487
**File:** `sim/L0/mjcf/src/types.rs`, lines 3069–3101
**MuJoCo equivalent:** MJCF schema — `objtype` attribute on frame sensors,
`reftype`/`refname` as separate attributes
**Design decision:** Minimal parser changes to store the raw string values. No
semantic validation in the parser — that's the builder's job. The parser stores
`objtype`, `geom=`, and separated `reftype`/`refname` as raw strings. This follows
the existing pattern where the parser is a thin XML→struct mapper.

**S1.1 — Add `objtype` and `reftype` fields to `MjcfSensor` struct.**

**File:** `sim/L0/mjcf/src/types.rs`

**After** (add to `MjcfSensor` struct):
```rust
pub struct MjcfSensor {
    pub name: String,
    pub class: Option<String>,
    pub sensor_type: MjcfSensorType,
    pub objname: Option<String>,
    /// Explicit object type string from MJCF `objtype` attribute.
    /// E.g., "site", "body", "xbody", "geom", "camera".
    pub objtype: Option<String>,
    /// Reference object type string from MJCF `reftype` attribute.
    pub reftype: Option<String>,
    /// Reference object name from MJCF `refname` attribute.
    pub refname: Option<String>,
    pub noise: f64,
    pub cutoff: f64,
    pub user: Vec<f64>,
}
```

Update the `Default` impl to include `objtype: None` and `reftype: None`.

**S1.2 — Parse `objtype`, `geom=`, and separate `reftype`/`refname` in `parse_sensor_attrs()`.**

**File:** `sim/L0/mjcf/src/parser.rs`

**Before** (current code at lines 3462–3470):
```rust
    // Different sensor types use different attribute names for their target
    sensor.objname = get_attribute_opt(e, "joint")
        .or_else(|| get_attribute_opt(e, "site"))
        .or_else(|| get_attribute_opt(e, "body"))
        .or_else(|| get_attribute_opt(e, "tendon"))
        .or_else(|| get_attribute_opt(e, "actuator"))
        .or_else(|| get_attribute_opt(e, "objname"));

    sensor.refname = get_attribute_opt(e, "reftype").or_else(|| get_attribute_opt(e, "refname"));
```

**After:**
```rust
    // Different sensor types use different attribute names for their target
    sensor.objname = get_attribute_opt(e, "joint")
        .or_else(|| get_attribute_opt(e, "site"))
        .or_else(|| get_attribute_opt(e, "body"))
        .or_else(|| get_attribute_opt(e, "geom"))
        .or_else(|| get_attribute_opt(e, "tendon"))
        .or_else(|| get_attribute_opt(e, "actuator"))
        .or_else(|| get_attribute_opt(e, "objname"));

    // Explicit object type (frame sensors only — builder validates)
    sensor.objtype = get_attribute_opt(e, "objtype");

    // Separate reftype and refname (previously conflated)
    sensor.reftype = get_attribute_opt(e, "reftype");
    sensor.refname = get_attribute_opt(e, "refname");
```

Changes:
1. Added `get_attribute_opt(e, "geom")` to the `objname` chain (between `body` and `tendon`).
2. Added `sensor.objtype = get_attribute_opt(e, "objtype")`.
3. Separated `reftype` and `refname` into distinct fields (was conflated).

**S1.3 — Preserve attribute provenance for default inference.**

The current parser collapses `site=`, `body=`, `geom=`, `objname=` into a single
`objname` field, losing provenance. MuJoCo infers `sensor_objtype` from which
attribute name was used when `objtype` is omitted:
- `site="name"` → `mjOBJ_SITE`
- `body="name"` → `mjOBJ_XBODY` (not BODY)
- `geom="name"` → `mjOBJ_GEOM`

Rather than restructuring the parser, we infer provenance in the builder:
when `objtype` is `None`, the builder tries `site_name_to_id.get(name)` first
(→ Site), then `body_name_to_id.get(name)` (→ XBody), then
`geom_name_to_id.get(name)` (→ Geom). This replicates the current heuristic but
now produces the correct `MjObjectType` variant. This works because MJCF names
are unique within each element type, and the priority order (site→body→geom)
matches MuJoCo's inference for the common case.

**Limitation:** If `objtype` is omitted AND the object name exists in multiple
name maps, the heuristic picks the first match (site > body > geom). This
matches the current behavior and is safe for valid MJCF (which specifies which
attribute name to use). For explicit `objtype`, the builder uses the exact mapping
(see S3).

### Field Existence Verification

Every model/data field referenced by this spec. All fields verified to exist;
no new fields need to be added.

| Field | Type | File:Line | Used By |
|-------|------|-----------|---------|
| `data.xpos` | `Vec<Vector3<f64>>` | `data.rs:95` | S4.1 (XBody FramePos), S5.1 (XBody FrameLinAcc), S4.4 (XBody FrameLinVel) |
| `data.xquat` | `Vec<UnitQuaternion<f64>>` | `data.rs:97` | S4.2 (XBody FrameQuat, Body FrameQuat via mulQuat) |
| `data.xmat` | `Vec<Matrix3<f64>>` | `data.rs:99` | S4.3 (XBody FrameAxis), S4.6 (XBody Velocimeter) |
| `data.xipos` | `Vec<Vector3<f64>>` | `data.rs:101` | S4.1 (Body FramePos), S5.1 (Body FrameLinAcc), S4.4 (Body FrameLinVel) |
| `data.ximat` | `Vec<Matrix3<f64>>` | `data.rs:103` | S4.3 (Body FrameAxis), S4.6 (Body Velocimeter) |
| `data.geom_xpos` | `Vec<Vector3<f64>>` | `data.rs:107` | S5.1 (Geom FrameLinAcc) |
| `data.geom_xmat` | `Vec<Matrix3<f64>>` | `data.rs:109` | (not directly used — Geom frame axis already handled in position.rs) |
| `data.site_xpos` | `Vec<Vector3<f64>>` | `data.rs:113` | S5.1 (Site FrameLinAcc — existing) |
| `data.site_xmat` | `Vec<Matrix3<f64>>` | `data.rs:115` | Existing sensor arms |
| `data.cacc` | `Vec<[f64; 6]>` | `data.rs:393` (approx) | S5.2 (FrameAngAcc angular components) |
| `data.cvel` | `Vec<[f64; 6]>` | `data.rs:391` (approx) | S4.5 (FrameAngVel) |
| `model.body_iquat` | `Vec<UnitQuaternion<f64>>` | `model.rs:135` | S4.2 (Body FrameQuat: `xquat * body_iquat`) |
| `model.geom_body` | `Vec<usize>` | `model.rs:253` | S2.2 (touch body match), S5.1/S5.2 (Geom→body resolution) |
| `model.site_body` | `Vec<usize>` | `model.rs:474` | S2.2 (touch site→body), existing sensor arms |
| `model.geom_quat` | `Vec<UnitQuaternion<f64>>` | `model.rs:257` | (Geom FrameQuat — existing in position.rs) |
| `model.site_quat` | `Vec<UnitQuaternion<f64>>` | `model.rs:480` | (Site FrameQuat — existing in position.rs) |
| `model.nsite` | `usize` | `model.rs:57` | Bounds checks in evaluation |
| `model.ngeom` | `usize` | `model.rs:59` | Bounds checks in evaluation |
| `model.nbody` | `usize` | `model.rs:44` | Bounds checks in evaluation |

### `objtype` Attribute Dispatch Table

Which sensor types honor vs ignore the `objtype` attribute in the builder:

| Sensor Type(s) | `objtype` Honored? | Rationale |
|---------------|-------------------|-----------|
| FramePos, FrameQuat, FrameXAxis, FrameYAxis, FrameZAxis, FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc | **Yes** | Frame sensors are polymorphic — can attach to site, body, xbody, geom, camera. `objtype` selects which. |
| Touch | **No** — always resolves via `site=` → `(Site, site_id)` | MuJoCo touch sensors are always site-based (`sensor_objtype = mjOBJ_SITE`). The `objtype` attribute is not meaningful for touch. |
| Accelerometer, Gyro, Velocimeter, Force, Torque, Magnetometer, Rangefinder | **No** — always resolves via `site=` → `(Site, site_id)` | These are site-attached sensors in MuJoCo. `objtype` has no effect. |
| JointPos, JointVel, BallQuat, BallAngVel, JointLimitFrc | **No** — always resolves via `joint=` → `(Joint, joint_id)` | Joint-attached sensors. |
| TendonPos, TendonVel, TendonLimitFrc | **No** — always resolves via `tendon=` → `(Tendon, tendon_id)` | Tendon-attached sensors. |
| ActuatorPos, ActuatorVel, ActuatorFrc | **No** — always resolves via `actuator=` → `(Actuator, actuator_id)` | Actuator-attached sensors. |
| SubtreeCom, SubtreeLinVel, SubtreeAngMom | **No** — always resolves via `body=` → `(Body, body_id)` | Body-attached subtree sensors. |
| User | **No** — always returns `(None, 0)` | User sensors have no object. |

**Pipeline trace for ignored `objtype`:** `<touch site="s1" objtype="geom"/>` →
parser stores `objname = "s1"`, `objtype = Some("geom")` → builder calls
`resolve_sensor_object(Touch, Some("s1"), Some("geom"))` → Touch arm ignores
`objtype_str`, resolves `site_name_to_id.get("s1")` → returns `(Site, site_id)`.
The `objtype="geom"` attribute is silently ignored. This matches MuJoCo's behavior
where `objtype` is only relevant for frame sensors.

### Attribute Provenance Analysis

When `objtype` is omitted for frame sensors, the builder infers the object type
from name lookup priority (site → body → geom). This matches MuJoCo's inference
because:

1. **MJCF schema enforces attribute exclusivity.** A valid `<framepos>` element
   uses exactly one of `site=`, `body=`, `geom=`, or `objname=`. It cannot
   specify both `site=` and `body=` on the same element.

2. **Name uniqueness within element type.** MJCF requires unique names within
   each element type (all sites have unique names, all bodies have unique names,
   etc.). Cross-type name collisions are allowed but the MJCF attribute name
   disambiguates (MuJoCo reads `site=` → Site, `body=` → XBody).

3. **Parser fallback chain matches MJCF attribute priority.** The parser at
   `parser.rs:3463–3468` tries `joint` → `site` → `body` → `geom` → `tendon` →
   `actuator` → `objname`. For frame sensors, only `site`, `body`, `geom` are
   relevant. The builder's heuristic then tries `site_name_to_id` → `body_name_to_id`
   → `geom_name_to_id`, producing Site / XBody / Geom respectively.

4. **Equivalence proof for valid MJCF:** If the MJCF element has `site="s1"`, the
   parser stores `objname = "s1"`. The builder looks up `"s1"` in `site_name_to_id`
   → match → returns `(Site, site_id)`. This matches MuJoCo's inference
   (`site=` → `mjOBJ_SITE`). The same holds for `body=` → `body_name_to_id` →
   `(XBody, body_id)` and `geom=` → `geom_name_to_id` → `(Geom, geom_id)`.

5. **Edge case — cross-type name collision:** If a site and body share the name
   `"x"`, and the MJCF element has `body="x"`, the parser stores `objname = "x"`.
   The builder finds `"x"` in `site_name_to_id` first → returns `(Site, site_id)`.
   MuJoCo would return `(XBody, body_id)`. This is a divergence. However, this
   edge case requires explicit `objtype` to resolve correctly, and valid MJCF
   models with explicit `objtype` bypass the heuristic entirely. Without `objtype`,
   MuJoCo's inference depends on the attribute name (`body=`), which our parser
   loses. **Impact:** Rare — cross-type name collisions in MJCF are uncommon and
   are an MJCF anti-pattern. The fix (preserving attribute provenance) is deferred;
   using explicit `objtype` resolves all ambiguity. Tracked as a known limitation,
   not a DT-ID (too minor for v1.0).

### S2. Touch sensor keying change (DT-64)

**File:** `sim/L0/mjcf/src/builder/sensor.rs`, lines 140–163
**File:** `sim/L0/core/src/sensor/acceleration.rs`, lines 143–181
**MuJoCo equivalent:** `mj_sensorAcc()` `mjSENS_TOUCH` case in `engine_sensor.c`
**Design decision:** Store `(MjObjectType::Site, site_id)` in the model (matching
MuJoCo). Resolve site→body at evaluation time via `model.site_body[objid]`. This
follows MuJoCo's exact data flow and allows the same sensor to track body changes
if the site is re-parented.

**S2.1 — Builder: change touch sensor resolution.**

**File:** `sim/L0/mjcf/src/builder/sensor.rs`

**Before** (lines 140–163):
```rust
MjSensorType::Touch => {
    let site_id = *self.site_name_to_id.get(name)
        .ok_or_else(|| /* error */)?;
    let body_id = *self.site_body.get(site_id)
        .ok_or_else(|| /* error */)?;
    let geom_id = self.geom_body
        .iter()
        .position(|&b| b == body_id)
        .unwrap_or(usize::MAX);
    Ok((MjObjectType::Geom, geom_id))
}
```

**After:**
```rust
MjSensorType::Touch => {
    let site_id = *self.site_name_to_id.get(name)
        .ok_or_else(|| ModelConversionError::NameNotFound {
            name: name.to_string(),
            element_type: "site".to_string(),
        })?;
    Ok((MjObjectType::Site, site_id))
}
```

Touch sensor now stores `(Site, site_id)` — matching MuJoCo's model where
`sensor_objtype = mjOBJ_SITE` and `sensor_objid = site_id`.

**S2.2 — Evaluation: rewrite touch sensor to body-level contact iteration.**

**File:** `sim/L0/core/src/sensor/acceleration.rs`

**Before** (lines 143–181):
```rust
MjSensorType::Touch => {
    let mut total_force = 0.0;
    let nefc = data.efc_type.len();
    let mut ei = 0;
    while ei < nefc {
        let dim = data.efc_dim[ei];
        if matches!(data.efc_type[ei], ConstraintType::ContactElliptic
            | ConstraintType::ContactFrictionless | ConstraintType::ContactPyramidal)
        {
            let ci = data.efc_id[ei];
            if ci < data.contacts.len() {
                let c = &data.contacts[ci];
                if c.geom1 == objid || c.geom2 == objid {
                    if data.efc_type[ei] == ConstraintType::ContactPyramidal {
                        for k in 0..dim { total_force += data.efc_force[ei + k]; }
                    } else {
                        total_force += data.efc_force[ei];
                    }
                }
            }
        ei += dim;
        } else { ei += 1; }
    }
    sensor_write(&mut data.sensordata, adr, 0, total_force);
}
```

**After:**
```rust
MjSensorType::Touch => {
    // MuJoCo: objid = site_id, resolve to body at runtime
    let body_id = if objid < model.nsite {
        model.site_body[objid]
    } else {
        sensor_write(&mut data.sensordata, adr, 0, 0.0);
        continue;
    };

    let mut total_force = 0.0;
    let nefc = data.efc_type.len();
    let mut ei = 0;
    while ei < nefc {
        let dim = data.efc_dim[ei];
        if matches!(
            data.efc_type[ei],
            ConstraintType::ContactElliptic
                | ConstraintType::ContactFrictionless
                | ConstraintType::ContactPyramidal
        ) {
            let ci = data.efc_id[ei];
            if ci < data.contacts.len() {
                let c = &data.contacts[ci];
                // Body-level match: check if either contact geom belongs to
                // the sensor's body (MuJoCo: geom_bodyid[con->geom[k]] == bodyid)
                let geom1_body = if c.geom1 < model.ngeom {
                    model.geom_body[c.geom1]
                } else {
                    usize::MAX
                };
                let geom2_body = if c.geom2 < model.ngeom {
                    model.geom_body[c.geom2]
                } else {
                    usize::MAX
                };
                if body_id == geom1_body || body_id == geom2_body {
                    // Read normal force from efc_force.
                    // NOTE: This reads efc_force directly, NOT via mj_contactForce().
                    // For frictionless/elliptic contacts, efc_force[0] IS the normal
                    // force. For pyramidal contacts, this sums facet projections —
                    // ~75% of the true normal force (DT-118 tracks full conformance).
                    if data.efc_type[ei] == ConstraintType::ContactPyramidal {
                        for k in 0..dim {
                            total_force += data.efc_force[ei + k];
                        }
                    } else {
                        total_force += data.efc_force[ei];
                    }
                }
            }
            ei += dim;
        } else {
            ei += 1;
        }
    }
    sensor_write(&mut data.sensordata, adr, 0, total_force);
}
```

**`objid` audit:** In the old code, `objid` was a geom_id (from builder resolution).
In the new code, `objid` is a site_id. Every use of `objid` in the Touch arm:
- Line `let body_id = model.site_body[objid]` — correct, reads site's parent body.
- No line compares `objid` against `c.geom1`/`c.geom2` — that comparison is now
  `body_id == geom1_body || body_id == geom2_body`.

### S3. Builder: explicit objtype dispatch for frame sensors + `XBody` enum (DT-62)

**File:** `sim/L0/core/src/types/enums.rs`, lines 494–512
**File:** `sim/L0/mjcf/src/builder/sensor.rs`, lines 166–186
**File:** `sim/L0/core/src/sensor/mod.rs`, lines 21–53
**MuJoCo equivalent:** `get_xpos_xmat()` dispatch in `engine_sensor.c`
**Design decision:** Add `XBody` to `MjObjectType`. Rewrite frame sensor
resolution in the builder to use explicit `objtype` when available, falling back to
the site→body→geom heuristic (with corrected object types) when omitted.

**S3.1 — Add `XBody` to `MjObjectType` enum.**

**File:** `sim/L0/core/src/types/enums.rs`

**After:**
```rust
pub enum MjObjectType {
    #[default]
    None,
    Body,    // MuJoCo mjOBJ_BODY (1) — inertial/COM frame (xipos/ximat)
    XBody,   // MuJoCo mjOBJ_XBODY (2) — joint frame (xpos/xmat)
    Joint,
    Geom,
    Site,
    Actuator,
    Tendon,
}
```

**S3.2 — Update `sensor_body_id()` for `XBody`.**

**File:** `sim/L0/core/src/sensor/mod.rs`

Add `XBody` arm:
```rust
match model.sensor_objtype[sensor_id] {
    MjObjectType::Body | MjObjectType::XBody => Some(objid),
    MjObjectType::Joint => { /* unchanged */ }
    MjObjectType::Geom => { /* unchanged */ }
    MjObjectType::Site => { /* unchanged */ }
    MjObjectType::Tendon | MjObjectType::Actuator | MjObjectType::None => None,
}
```

**S3.3 — Rewrite frame sensor resolution in builder.**

**File:** `sim/L0/mjcf/src/builder/sensor.rs`

**Before** (lines 166–186 — frame sensor block):
```rust
MjSensorType::FramePos | MjSensorType::FrameQuat | /* ... */ => {
    if let Some(&id) = self.site_name_to_id.get(name) {
        Ok((MjObjectType::Site, id))
    } else if let Some(&id) = self.body_name_to_id.get(name) {
        Ok((MjObjectType::Body, id))
    } else if let Some(&id) = self.geom_name_to_id.get(name) {
        Ok((MjObjectType::Geom, id))
    } else {
        Err(/* ... */)
    }
}
```

**After:**
```rust
MjSensorType::FramePos
| MjSensorType::FrameQuat
| MjSensorType::FrameXAxis
| MjSensorType::FrameYAxis
| MjSensorType::FrameZAxis
| MjSensorType::FrameLinVel
| MjSensorType::FrameAngVel
| MjSensorType::FrameLinAcc
| MjSensorType::FrameAngAcc => {
    // Note: `objtype_str` comes from the MjcfSensor struct, threaded
    // through process_sensors(). See S1.
    if let Some(objtype_str) = objtype_str {
        // Explicit objtype attribute — use it
        match objtype_str {
            "site" => {
                let id = *self.site_name_to_id.get(name)
                    .ok_or_else(|| /* NameNotFound */)?;
                Ok((MjObjectType::Site, id))
            }
            "body" => {
                let id = *self.body_name_to_id.get(name)
                    .ok_or_else(|| /* NameNotFound */)?;
                Ok((MjObjectType::Body, id))
            }
            "xbody" => {
                let id = *self.body_name_to_id.get(name)
                    .ok_or_else(|| /* NameNotFound */)?;
                Ok((MjObjectType::XBody, id))
            }
            "geom" => {
                let id = *self.geom_name_to_id.get(name)
                    .ok_or_else(|| /* NameNotFound */)?;
                Ok((MjObjectType::Geom, id))
            }
            "camera" => {
                // DT-120: Camera deferred. Warn and fall through.
                warn!("objtype='camera' not yet supported (DT-120); \
                       falling back to name heuristic for sensor '{}'",
                       sensor_name);
                // Fall through to heuristic below
                self.resolve_frame_sensor_by_name(name)
            }
            other => {
                Err(ModelConversionError::InvalidAttribute {
                    attribute: "objtype".to_string(),
                    value: other.to_string(),
                })
            }
        }
    } else {
        // No explicit objtype — infer from name lookup
        // Priority: site → body (as XBody) → geom
        // MuJoCo: body= attribute → mjOBJ_XBODY (joint frame)
        self.resolve_frame_sensor_by_name(name)
    }
}
```

**Helper:**
```rust
fn resolve_frame_sensor_by_name(
    &self,
    name: &str,
) -> Result<(MjObjectType, usize), ModelConversionError> {
    if let Some(&id) = self.site_name_to_id.get(name) {
        Ok((MjObjectType::Site, id))
    } else if let Some(&id) = self.body_name_to_id.get(name) {
        // body= without objtype → XBody (MuJoCo: mjOBJ_XBODY)
        Ok((MjObjectType::XBody, id))
    } else if let Some(&id) = self.geom_name_to_id.get(name) {
        Ok((MjObjectType::Geom, id))
    } else {
        Err(ModelConversionError::NameNotFound {
            name: name.to_string(),
            element_type: "site/body/geom".to_string(),
        })
    }
}
```

**S3.4 — Thread `objtype` through `process_sensors()`.**

**File:** `sim/L0/mjcf/src/builder/sensor.rs`

Modify `process_sensors()` to pass `mjcf_sensor.objtype.as_deref()` to
`resolve_sensor_object()`. Update `resolve_sensor_object()` signature:

```rust
fn resolve_sensor_object(
    &self,
    sensor_type: MjSensorType,
    objname: Option<&str>,
    objtype_str: Option<&str>,  // NEW: explicit objtype from MJCF
) -> Result<(MjObjectType, usize), ModelConversionError>
```

Non-frame-sensor arms ignore `objtype_str`.

### S4. Evaluation: `XBody` + `Body` arms in frame sensors

**File:** `sim/L0/core/src/sensor/position.rs`
**File:** `sim/L0/core/src/sensor/velocity.rs`
**File:** `sim/L0/core/src/sensor/acceleration.rs`
**MuJoCo equivalent:** `get_xpos_xmat()`, `get_xquat()` in `engine_sensor.c`
**Design decision:** Add `XBody` and `Body` dispatch arms to every frame sensor
evaluation site. Current `Body` match arms read `xpos`/`xmat` (= MuJoCo XBODY).
New `Body` arms read `xipos`/`ximat` (= MuJoCo BODY).

**S4.1 — FramePos (position.rs):**

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => data.xpos[objid],
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => data.xpos[objid],
MjObjectType::Body if objid < model.nbody => data.xipos[objid],
```

**S4.2 — FrameQuat (position.rs):**

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => data.xquat[objid],
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => data.xquat[objid],
MjObjectType::Body if objid < model.nbody => {
    // MuJoCo: mulQuat(xquat[body], body_iquat[body])
    data.xquat[objid] * model.body_iquat[objid]
}
```

Note: `UnitQuaternion * UnitQuaternion` in nalgebra performs quaternion
multiplication, matching MuJoCo's `mju_mulQuat()`.

**S4.3 — FrameAxis (position.rs):**

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => data.xmat[objid],
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => data.xmat[objid],
MjObjectType::Body if objid < model.nbody => data.ximat[objid],
```

**S4.4 — FrameLinVel (velocity.rs):**

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => (objid, data.xpos[objid]),
MjObjectType::Body if objid < model.nbody => (objid, data.xipos[objid]),
```

For `Body` (inertial frame), the velocity is computed at the COM position
`xipos`, matching MuJoCo's behavior where `mj_objectVelocity()` uses the
position returned by `get_xpos_xmat()`.

**S4.5 — FrameAngVel (velocity.rs):**

Angular velocity is body-level (reference-point-independent for rigid bodies).

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => Vector3::new(
    data.cvel[objid][0], data.cvel[objid][1], data.cvel[objid][2],
),
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => Vector3::new(
    data.cvel[objid][0], data.cvel[objid][1], data.cvel[objid][2],
),
MjObjectType::Body if objid < model.nbody => Vector3::new(
    data.cvel[objid][0], data.cvel[objid][1], data.cvel[objid][2],
),
```

Both read the same `cvel` since angular velocity doesn't depend on reference point.

**S4.6 — Velocimeter (velocity.rs):**

**Before:**
```rust
MjObjectType::Body if objid < model.nbody => {
    (objid, data.xpos[objid], data.xmat[objid])
}
```

**After:**
```rust
MjObjectType::XBody if objid < model.nbody => {
    (objid, data.xpos[objid], data.xmat[objid])
}
MjObjectType::Body if objid < model.nbody => {
    (objid, data.xipos[objid], data.ximat[objid])
}
```

### S5. Evaluation: geom-attached FrameLinAcc/FrameAngAcc (DT-102)

**File:** `sim/L0/core/src/sensor/acceleration.rs`, lines 207–238
**MuJoCo equivalent:** `mj_sensorAcc()` `mjSENS_FRAMELINACC`/`mjSENS_FRAMEANGACC`
in `engine_sensor.c`, calling `mj_objectAcceleration(m, d, mjOBJ_GEOM, objid, ...)`
**Design decision:** Add `Geom` arm to FrameLinAcc and FrameAngAcc. Use existing
`object_acceleration()` from `spatial.rs:286–323` for FrameLinAcc (includes spatial
transport + Coriolis). For FrameAngAcc, read `cacc[body_id]` angular components
directly — angular acceleration is reference-point-independent, so spatial transport
is unnecessary.

**S5.1 — FrameLinAcc: add Geom arm.**

**Before:**
```rust
MjSensorType::FrameLinAcc => {
    let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };
    let (_alpha, a_lin) = object_acceleration(data, body_id, &obj_pos, None);
    sensor_write3(&mut data.sensordata, adr, &a_lin);
}
```

**After:**
```rust
MjSensorType::FrameLinAcc => {
    let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::XBody if objid < model.nbody => (objid, data.xpos[objid]),
        MjObjectType::Body if objid < model.nbody => (objid, data.xipos[objid]),
        MjObjectType::Geom if objid < model.ngeom => {
            (model.geom_body[objid], data.geom_xpos[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };
    let (_alpha, a_lin) = object_acceleration(data, body_id, &obj_pos, None);
    sensor_write3(&mut data.sensordata, adr, &a_lin);
}
```

**S5.2 — FrameAngAcc: add Geom + XBody arms.**

**Before:**
```rust
MjSensorType::FrameAngAcc => {
    let body_id = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => model.site_body[objid],
        MjObjectType::Body if objid < model.nbody => objid,
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };
    let cacc = data.cacc[body_id];
    let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
    sensor_write3(&mut data.sensordata, adr, &alpha);
}
```

**After:**
```rust
MjSensorType::FrameAngAcc => {
    let body_id = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => model.site_body[objid],
        MjObjectType::XBody if objid < model.nbody => objid,
        MjObjectType::Body if objid < model.nbody => objid,
        MjObjectType::Geom if objid < model.ngeom => model.geom_body[objid],
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };
    // Angular acceleration: read directly from cacc[body_id] angular components.
    // Angular acceleration is reference-point-independent — no spatial transport
    // needed. This matches MuJoCo: mj_objectAcceleration() returns tmp[0..3] for
    // angular, which is cacc[body_id][0..3] regardless of target position.
    let cacc = data.cacc[body_id];
    let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
    sensor_write3(&mut data.sensordata, adr, &alpha);
}
```

Note: FrameAngAcc reads `cacc` angular components directly, NOT via
`object_acceleration()`. This is intentional — `object_acceleration()` computes
spatial transport (affecting linear only) and Coriolis (affecting linear only).
Angular acceleration is the same at all points on a rigid body. Calling
`object_acceleration()` and discarding the linear result would waste computation
and introduce floating-point drift.

---

## Acceptance Criteria

### AC1: Parser stores `objtype` attribute *(runtime test)*
**Given:** MJCF with `<framepos name="fp1" objtype="geom" objname="g1"/>`
**After:** Parse MJCF
**Assert:** `MjcfSensor.objtype == Some("geom".to_string())`
**Field:** `MjcfSensor.objtype`

### AC2: Parser separates `reftype` and `refname` *(runtime test)*
**Given:** MJCF with `<framepos name="fp1" site="s1" reftype="body" refname="b1"/>`
**After:** Parse MJCF
**Assert:** `MjcfSensor.reftype == Some("body")` AND `MjcfSensor.refname == Some("b1")`
**Field:** `MjcfSensor.reftype`, `MjcfSensor.refname`

### AC3: Parser handles `geom=` attribute *(runtime test)*
**Given:** MJCF with `<framepos name="fp1" geom="g1"/>`
**After:** Parse MJCF
**Assert:** `MjcfSensor.objname == Some("g1")`
**Field:** `MjcfSensor.objname`

### AC4: Builder resolves explicit `objtype="geom"` to `MjObjectType::Geom` *(runtime test)*
**Given:** MJCF with `<framepos name="fp1" objtype="geom" objname="g1"/>` and a geom named `g1`
**After:** Build model
**Assert:** `model.sensor_objtype[0] == MjObjectType::Geom` AND
`model.sensor_objid[0] == geom_id_of_g1`
**Field:** `Model.sensor_objtype`, `Model.sensor_objid`

### AC5: Builder resolves `objtype="body"` → `MjObjectType::Body` and `objtype="xbody"` → `MjObjectType::XBody` *(runtime test)*
**Given:** MJCF with:
- `<framepos name="fp_body" objtype="body" objname="b1"/>`
- `<framepos name="fp_xbody" objtype="xbody" objname="b1"/>`
**After:** Build model
**Assert:** `model.sensor_objtype[0] == MjObjectType::Body` AND
`model.sensor_objtype[1] == MjObjectType::XBody`
**Field:** `Model.sensor_objtype`

### AC6: Body vs XBody position distinction *(runtime test — MuJoCo-verified)*
**Given:** Body with capsule geom (mass=1, half-length=0.15, COM offset 0.15m),
`<framepos name="fp_body" objtype="body" objname="b1"/>` and
`<framepos name="fp_xbody" objtype="xbody" objname="b1"/>`
**After:** `forward()`
**Assert:** `sensordata[fp_body]` reads `xipos` = `[0.15, 0.0, 1.0]` ± 1e-10;
`sensordata[fp_xbody]` reads `xpos` = `[0.0, 0.0, 1.0]` ± 1e-10
**Field:** `Data.sensordata`
**Source:** MuJoCo 3.5.0, EGT-1 (rubric)

### AC7: Body FrameQuat uses `mulQuat(xquat, body_iquat)` *(runtime test)*
**Given:** Body with asymmetric geom (body_iquat ≠ identity), `<framequat objtype="body" objname="b1"/>`
**After:** `forward()`
**Assert:** `sensordata` quaternion = `xquat[b1] * body_iquat[b1]` ± 1e-10
**Field:** `Data.sensordata`

### AC8: Default inference: `body="name"` without `objtype` → XBody *(runtime test)*
**Given:** MJCF with `<framepos body="b1"/>` (no `objtype` attribute)
**After:** Build model
**Assert:** `model.sensor_objtype[0] == MjObjectType::XBody`
**Field:** `Model.sensor_objtype`

### AC9: Touch sensor multi-geom aggregation *(runtime test — MuJoCo-verified)*
**Given:** Body with 3 sphere geoms (mass=1 each), touch sensor attached via site.
Contacts injected on all 3 geoms (elliptic, normal force = 9.81 each).
**After:** `mj_sensor_acc()`
**Assert:** `sensordata[touch] ≈ 29.43` (total force from all 3 geoms, ± 1e-2).
**Field:** `Data.sensordata`
**Source:** MuJoCo 3.5.0, EGT-2 (rubric). Note: exact value depends on contact
model; this AC uses elliptic contacts where `efc_force[0]` IS the normal force.

### AC10: Touch sensor ignores contacts on different bodies *(runtime test)*
**Given:** Two bodies, touch sensor on body 1. Inject contacts on body 1 and body 2.
**After:** `mj_sensor_acc()`
**Assert:** Touch sensor value only includes forces from body 1's geoms.
**Field:** `Data.sensordata`

### AC11: Touch sensor handles body with zero geoms *(runtime test)*
**Given:** Touch sensor on a site whose body has no geoms.
**After:** `mj_sensor_acc()`
**Assert:** `sensordata[touch] == 0.0` (no contacts possible).
**Field:** `Data.sensordata`

### AC12: Geom-attached FrameLinAcc *(runtime test — MuJoCo-verified)*
**Given:** `<framelinacc name="fla" objtype="geom" objname="g1"/>`, static body
under gravity (g = [0, 0, -9.81]).
**After:** `forward()`
**Assert:** `sensordata[fla] ≈ [0, 0, 9.81]` ± 1e-6
**Field:** `Data.sensordata`
**Source:** MuJoCo 3.5.0, EGT-3

### AC13: Geom-attached FrameAngAcc *(runtime test)*
**Given:** `<frameangacc name="faa" objtype="geom" objname="g1"/>`, static body.
**After:** `forward()`
**Assert:** `sensordata[faa] ≈ [0, 0, 0]` ± 1e-10 (no angular acceleration at rest)
**Field:** `Data.sensordata`

### AC14: Geom-attached FrameLinAcc with offset geom *(runtime test)*
**Given:** Geom at position offset from body origin (geom_xpos ≠ xpos),
rotating body (ω = 10 rad/s around z-axis), geom at [0.5, 0, 0].
**After:** `forward()`
**Assert:** FrameLinAcc includes centripetal component: `a_x ≈ -50.0` (ω²r = 100×0.5).
Verifies spatial transport from body origin to geom position.
**Field:** `Data.sensordata`

### AC15: `MjObjectType::XBody` in enum *(code review)*
`MjObjectType` enum has `XBody` variant. `sensor_body_id()` handles `XBody`.
All frame sensor evaluation arms handle both `Body` and `XBody` with correct
data sources.

### AC16: Existing sensor tests pass *(regression test)*
All 2,238+ domain tests pass unchanged. Touch sensor tests with single-geom bodies
produce unchanged values (behavior is the same — the single geom's body matches).

### AC17: Builder compiles and passes for existing MJCF models *(regression test)*
Models that use `<touch site="..."/>` build successfully with
`sensor_objtype == Site` (was Geom). Models that use frame sensors without
`objtype` attribute build with same object resolution results (heuristic produces
same output for non-ambiguous names, with Body→XBody rename).

### AC18: `objtype` ignored for non-frame sensors *(runtime test)*
**Given:** `<touch site="s1" objtype="geom"/>` (explicit `objtype` on a Touch sensor)
**After:** Build model
**Assert:** `model.sensor_objtype[0] == MjObjectType::Site` (NOT `Geom`).
Touch sensor ignores the `objtype` attribute.
**Field:** `Model.sensor_objtype`

### AC19: Body FramePos on zero-mass body *(runtime test)*
**Given:** Body with no geoms (zero mass), `<framepos objtype="body" objname="b1"/>`.
MuJoCo sets `xipos`/`ximat` to joint frame origin for massless bodies.
**After:** `forward()`
**Assert:** `sensordata[fp]` = `xipos[b1]` ≈ `xpos[b1]` (COM frame = joint frame
for massless bodies). Verifies `xipos` is populated even for zero-mass bodies.
**Field:** `Data.sensordata`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (objtype parsed) | T1 | Direct |
| AC2 (reftype/refname separated) | T2 | Direct |
| AC3 (geom= attribute) | T3 | Direct |
| AC4 (builder geom objtype) | T4 | Direct |
| AC5 (body/xbody builder) | T5 | Direct |
| AC6 (body/xbody position) | T6 | Direct (MuJoCo-verified) |
| AC7 (body FrameQuat) | T7 | Direct |
| AC8 (default inference) | T8 | Direct |
| AC9 (touch multi-geom) | T9 | Direct (MuJoCo-verified) |
| AC10 (touch wrong body) | T10 | Edge case |
| AC11 (touch zero geoms) | T11 | Edge case |
| AC12 (geom FrameLinAcc) | T12 | Direct (MuJoCo-verified) |
| AC13 (geom FrameAngAcc) | T13 | Direct |
| AC14 (geom LinAcc offset) | T14 | Edge case |
| AC15 (XBody in enum) | — | Code review (manual) |
| AC16 (regression) | T15 | Regression |
| AC17 (builder regression) | T16 | Regression |
| AC18 (objtype ignored for Touch) | T17 | Negative case |
| AC19 (zero-mass body FramePos) | T18 | Edge case |

---

## Test Plan

### T1: Parser `objtype` attribute → AC1
Parse `<framepos name="fp1" objtype="geom" objname="g1"/>`. Assert
`sensor.objtype == Some("geom")`. Parse without `objtype` → `sensor.objtype == None`.

### T2: Parser `reftype`/`refname` separation → AC2
Parse `<framepos site="s1" reftype="body" refname="b1"/>`. Assert
`sensor.reftype == Some("body")` AND `sensor.refname == Some("b1")`.
Parse with only `refname` → `sensor.reftype == None`, `sensor.refname == Some("b1")`.
Parse with only `reftype` → `sensor.reftype == Some("body")`, `sensor.refname == None`.

### T3: Parser `geom=` attribute → AC3
Parse `<framepos geom="g1"/>`. Assert `sensor.objname == Some("g1")`.

### T4: Builder explicit `objtype="geom"` → AC4
Build model with `<framepos objtype="geom" objname="g1"/>` and a geom named `g1`.
Assert `model.sensor_objtype[0] == Geom`, `model.sensor_objid[0] == geom_id`.

### T5: Builder `body`/`xbody` distinction → AC5
Build model with both `objtype="body"` and `objtype="xbody"` frame sensors on same
body. Assert `model.sensor_objtype[0] == Body`, `model.sensor_objtype[1] == XBody`.

### T6: MuJoCo conformance — body vs xbody position → AC6
**Model:** Body at pos="0 0 1" with capsule geom (half-length 0.15, aligned
along x-axis, mass=1). Two `<framepos>` sensors: one `objtype="body"`, one
`objtype="xbody"`.
**Expected:** `sensordata[body]` = `xipos = [0.15, 0.0, 1.0]` (COM position);
`sensordata[xbody]` = `xpos = [0.0, 0.0, 1.0]` (joint frame origin).
**Tolerance:** 1e-10
**Source:** MuJoCo 3.5.0, EGT-1. Analytically: COM offset = capsule half-length
= 0.15m from joint origin.

### T7: Body FrameQuat uses `mulQuat` → AC7
**Model:** Body with asymmetric geom producing non-identity `body_iquat`. One
`<framequat objtype="body" objname="b1"/>`.
**Expected:** `sensordata = xquat[b1] * body_iquat[b1]` (computed analytically).
**Tolerance:** 1e-10

### T8: Default inference `body=` → XBody → AC8
Build model with `<framepos body="b1"/>` (no `objtype`). Assert
`model.sensor_objtype[0] == XBody`.

### T9: MuJoCo conformance — touch multi-geom → AC9
**Model:** Body with 3 sphere geoms, site-based touch sensor. Manually inject
3 elliptic contacts (one per geom), each with `efc_force[0] = 9.81`.
**Expected:** `sensordata[touch] ≈ 29.43` (3 × 9.81).
**Tolerance:** 1e-2
**Source:** MuJoCo 3.5.0, EGT-2 (adjusted for elliptic — direct `efc_force` read).
**Comment in test:** "MuJoCo 3.5.0 produces 29.4237 for pyramidal contacts using
`mj_contactForce()`; this test uses elliptic contacts where `efc_force[0]` IS the
normal force (no `mj_contactForce` needed). DT-118 tracks full pyramidal conformance."

### T10: Touch — wrong body filtered → AC10
**Model:** Two bodies (b1 with touch sensor, b2 without). Inject contact
with `geom1` on b2, `geom2` on b2. Assert touch sensor = 0.
Inject contact with `geom1` on b1, `geom2` on b2. Assert touch sensor > 0.

### T11: Touch — body with zero geoms → AC11
**Model:** Body with no geoms (has a site). Touch sensor on that site.
**Expected:** `sensordata[touch] == 0.0`. No contacts possible without geoms.

### T12: MuJoCo conformance — geom-attached FrameLinAcc → AC12
**Model:** Static body under gravity, `<framelinacc objtype="geom" objname="g1"/>`.
**Expected:** `sensordata ≈ [0, 0, 9.81]` (gravity-compensated).
**Tolerance:** 1e-6
**Source:** MuJoCo 3.5.0, EGT-3.

### T13: Geom-attached FrameAngAcc → AC13
**Model:** Static body, `<frameangacc objtype="geom" objname="g1"/>`.
**Expected:** `sensordata ≈ [0, 0, 0]` (no angular acceleration at rest).
**Tolerance:** 1e-10

### T14: Geom-attached FrameLinAcc with centripetal → AC14
**Model:** Body with hinge joint (axis z), geom at `pos="0.5 0 0"`.
Set `qvel[0] = 10.0`. `<framelinacc objtype="geom" objname="g1"/>`.
**Expected:** `sensordata[0] ≈ -50.0` (ω²r = 100 × 0.5, centripetal toward axis).
**Tolerance:** 1e-6
**Source:** Analytically derived. Matches existing T8 in
`sim/L0/tests/integration/spatial_transport.rs:154–186`.

### T15: Regression — existing sensor tests pass → AC16
Run full domain test suite. Verify 0 failures.

### T16: Builder regression — touch sensor builds as Site → AC17
Load an existing MJCF model fixture that uses `<touch site="s1"/>`. Verify build
succeeds with `sensor_objtype == Site`. Verify `sensor_adr` and sensor count
unchanged.

### T17: Negative case — `objtype` ignored for Touch → AC18
Build model with `<touch site="s1" objtype="geom"/>`. Assert
`model.sensor_objtype[0] == Site` (NOT `Geom`). The `objtype` attribute is
silently ignored for non-frame sensors. This tests the pipeline trace described
in the "objtype Attribute Dispatch Table" section.

### T18: Edge case — Body FramePos on zero-mass body → AC19
**Model:** Body with no geoms (has a site for anchoring, mass = 0).
`<framepos objtype="body" objname="b1"/>`.
**Expected:** `sensordata` = `xipos[b1]`. For zero-mass bodies, MuJoCo sets
`xipos`/`ximat` to the joint frame origin (same as `xpos`/`xmat`). Verifies
that the `Body` (inertial frame) arm reads `xipos` correctly even when the
body has zero mass.
**Tolerance:** 1e-10

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| World body (`body_id == 0`) | `cacc[0]` is zero; sensors on world body sites produce correct values. | T12 (world body variant if needed) | AC12 |
| Zero-mass body | `xipos`/`ximat` set to joint frame for massless bodies. Body-attached frame sensor reads inertial frame. | T18 | AC19 |
| Body with no geoms | Touch sensor on body with only sites → 0.0 (no contacts possible). | T11 | AC11 |
| Sleeping body | Touch sensor skipped (existing sleep check). | Existing tests | AC16 |
| `mjDSBL_SENSOR` flag | Early return, stale sensordata preserved. | Existing tests | AC16 |
| Multi-geom body, contacts on all geoms | Touch sensor aggregates forces from all geoms. | T9 | AC9 |
| Contact on different body | Touch sensor excludes contacts from other bodies. | T10 | AC10 |
| Geom at offset position from body | FrameLinAcc includes spatial transport + Coriolis. | T14 | AC14 |
| `objtype` omitted | Builder falls back to name heuristic, `body=` → XBody. | T8 | AC8 |
| `objtype` on non-frame sensor | `objtype` silently ignored for Touch, Accelerometer, etc. Builder resolves via type-specific attribute. | T17 | AC18 |
| `objtype="camera"` (unsupported) | Warning emitted, falls back to heuristic. | — (DT-120) | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T16 (builder regression) | Existing MJCF fixtures build correctly | Guards against builder changes breaking existing models |
| T17 (objtype ignored for Touch) | Non-frame sensor objtype silencing | Verifies pipeline dispatch table correctness — objtype only honored for frame sensors |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Touch sensor objtype | `(Geom, first_geom_id)` | `(Site, site_id)` | Toward MuJoCo | `acceleration.rs` touch arm, `sensor_body_id()` | Transparent — evaluation resolves site→body at runtime. Downstream code using `sensor_objtype == Geom` for touch must now check `Site`. |
| Touch sensor scope | Single-geom contacts only | All geoms on body contribute | Toward MuJoCo | Touch sensor values change for multi-geom bodies | Values increase (more contacts counted). Single-geom bodies unchanged. |
| Frame sensor `body=` heuristic | `MjObjectType::Body` | `MjObjectType::XBody` | Toward MuJoCo | Frame sensor evaluation — same data paths (`xpos`/`xmat`) | Transparent rename. Evaluation produces identical values since XBody reads same arrays as old Body. |
| Frame sensor `objtype="body"` explicit | Not possible (attribute ignored) | `MjObjectType::Body` → reads `xipos`/`ximat` | Toward MuJoCo | New functionality — no existing code affected | New feature. |
| Geom acc sensors | Return zeros | Return computed acceleration | Toward MuJoCo | `sensordata` values change from 0 to non-zero for geom-attached acc sensors | Values go from wrong (zeros) to correct. |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | Add `objtype`, `reftype` fields to `MjcfSensor`; update `Default` | +6 / ~3 modified |
| `sim/L0/mjcf/src/parser.rs` | Add `geom=` to chain; parse `objtype`; separate `reftype`/`refname` | +4 / ~2 modified |
| `sim/L0/mjcf/src/builder/sensor.rs` | Rewrite touch resolution; rewrite frame sensor resolution with explicit objtype dispatch; add `resolve_frame_sensor_by_name()` helper; thread `objtype` parameter | +60 / -20 / ~15 modified |
| `sim/L0/core/src/types/enums.rs` | Add `XBody` variant to `MjObjectType` | +2 |
| `sim/L0/core/src/sensor/mod.rs` | Add `XBody` arm to `sensor_body_id()` | +1 / ~1 modified |
| `sim/L0/core/src/sensor/position.rs` | Add `XBody` + `Body` arms to FramePos, FrameQuat, FrameAxis | +12 / ~3 modified |
| `sim/L0/core/src/sensor/velocity.rs` | Add `XBody` + `Body` arms to FrameLinVel, FrameAngVel, Velocimeter | +12 / ~3 modified |
| `sim/L0/core/src/sensor/acceleration.rs` | Add `Geom` + `XBody` + `Body` arms to FrameLinAcc/FrameAngAcc; rewrite Touch | +50 / -25 / ~10 modified |
| `sim/L0/core/src/sensor/acceleration.rs` tests | Update existing touch tests (objtype Geom → Site) | ~5 modified |
| `sim/L0/tests/integration/mjcf_sensors.rs` | Update `sensor_objtype == Body` assertions to `XBody` for `body=` tests | ~3 modified |
| New test file(s) | T1–T18 | +330 |

### Sentinel Audit

Resolution paths in `resolve_sensor_object()` produce `(MjObjectType, usize)`.
Every path must either return a valid ID or an `Err`. Sentinels (`usize::MAX`,
`0` for invalid) must never reach evaluation code.

| Resolution Path | Before | After | Sentinel Risk |
|----------------|--------|-------|---------------|
| Touch → site→body→first geom | `unwrap_or(usize::MAX)` at builder line 161 — `usize::MAX` stored as `sensor_objid` when body has no geoms. Evaluation reads `data.contacts[ci].geom1 == usize::MAX` — no match, touch = 0. **Silently wrong** (should be 0, and is, but via sentinel coincidence). | Returns `(Site, site_id)` — no sentinel. Site ID always valid (came from `site_name_to_id`). Evaluation bounds-checks `objid < model.nsite` before access. | **Eliminated.** |
| Frame sensor → name heuristic | Returns `Err(NameNotFound)` if name not in any map. No sentinel. | Same — `Err(NameNotFound)`. No sentinel. | **None.** |
| Frame sensor → explicit objtype | N/A (new code) | Returns `Err(NameNotFound)` if name not in correct type map. Returns `Err(InvalidAttribute)` for unknown objtype string. No sentinel. | **None.** |
| Joint/Tendon/Actuator/Site sensors | Returns `Err` on missing name. No sentinel. | Unchanged. | **None.** |
| Subtree sensors | Returns `Err` on missing body name. No sentinel. | Unchanged. | **None.** |
| User sensors | Returns `(None, 0)`. Evaluation skips User sensors at this stage. | Unchanged. | **None** — `0` is unused because User evaluation is a no-op. |

All evaluation match arms have bounds checks (`objid < model.nsite`,
`objid < model.ngeom`, `objid < model.nbody`). If a sentinel value somehow
reached evaluation, the bounds check would fail and the `_ =>` catch-all
would write zeros. The spec eliminates the only sentinel-producing path
(touch `unwrap_or(usize::MAX)`).

### Downstream Consumer Audit

All crates that read `Model.sensor_objtype` or `Model.sensor_objid`, searched
across the full workspace via `rg sensor_objtype` and `rg sensor_objid`:

| Crate | File | Usage | Impact |
|-------|------|-------|--------|
| **sim-core** | `sensor/mod.rs:27` | `sensor_body_id()` — sleep filtering dispatch | **Needs update:** add `XBody` arm (S3.2). |
| **sim-core** | `sensor/position.rs:108,119,141` | Frame sensor evaluation dispatch | **Needs update:** add `XBody`/`Body` arms (S4). |
| **sim-core** | `sensor/velocity.rs:87,113,132,148` | Frame sensor + site sensor dispatch | **Needs update:** add `XBody`/`Body` arms (S4). |
| **sim-core** | `sensor/acceleration.rs:83,103,125,208,226` | Acc sensor + frame acc dispatch | **Needs update:** add `Geom`/`XBody`/`Body` arms (S5), rewrite Touch (S2). |
| **sim-mjcf** | `builder/sensor.rs:42–43` | Populates arrays | **Needs update:** S2, S3 changes. |
| **sim-mjcf** | `builder/build.rs:234–235` | Transfers to Model | **No change.** |
| **sim-mjcf** | `builder/init.rs:221–222` | Initializes empty vecs | **No change.** |
| **sim-core** | `types/model.rs:500,502` | Field declarations | **No change.** |
| **sim-core** | `types/model_init.rs:224–225` | Default init | **No change.** |
| **sim-core** (integration tests) | `tests/integration/mjcf_sensors.rs:40,71,78,165,204` | Sensor wiring assertions | **May need update:** tests asserting `sensor_objtype == Body` for frame sensors with `body=` attribute should now assert `XBody`. |
| **sim-core** (integration tests) | `tests/integration/sensors_phase4.rs:1701` | Body ID extraction | **Pass** — reads `sensor_objid[0]`, not type-specific. |
| **sim-bevy** (L1) | No references found | N/A | **No impact.** |
| **sim-sensor** | No references found | N/A | **No impact.** |
| **sim-conformance-tests** | No references to `sensor_objtype`/`sensor_objid` | N/A | **No impact.** |

All consumers are within sim-core and sim-mjcf (L0). No downstream L1 or test
crate changes needed beyond updating integration test assertions for Body→XBody
rename.

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_touch_sensor_no_contact` | `sensor/mod.rs:188` | **Needs update** — change `MjObjectType::Geom` → `MjObjectType::Site` in test setup | Touch now stores Site, not Geom |
| `test_touch_sensor_with_contact` | `sensor/mod.rs:206` | **Needs update** — change geom ID to site ID, update contact check logic | Touch objid is now site_id; contact matching uses body-level check |
| `test_touch_sensor_*` | `sim/L0/sensor/src/touch.rs:476–669` | **Pass (unchanged)** — standalone `sim-sensor` crate tests don't use the pipeline | These test the `sim-sensor` crate's own touch API, not the pipeline |
| `t07_object_acceleration_static_gravity` | `integration/spatial_transport.rs:112` | **Pass (unchanged)** — tests `object_acceleration()` directly, not sensor pipeline | Function unchanged |
| `t08_object_acceleration_centripetal` | `integration/spatial_transport.rs:154` | **Pass (unchanged)** — tests `object_acceleration()` directly | Function unchanged |
| Existing `sim-conformance-tests` | Various | **Pass** — no frame sensor uses `objtype` attribute in existing test models | Heuristic produces same results for unambiguous names; XBody reads same arrays as old Body |
| `mjcf_sensors.rs:71` | `integration/mjcf_sensors.rs` | **Needs update** — assertion `sensor_objtype[0] == Body` must change to `XBody` | Frame sensor with `site=` resolves to Site (unchanged); with `body=` resolves to XBody (was Body). Tests using `site=` are unaffected. Tests using `body=` need assertion update. |

---

## Execution Order

1. **S1** (Parser + types) first because S3 depends on `objtype` being available
   in `MjcfSensor` → verify parser tests (T1, T2, T3)

2. **S3.1** (Add `XBody` to `MjObjectType`) second because S2, S4, S5 depend on
   the enum variant existing → verify compilation

3. **S3.2** (`sensor_body_id()` update) + **S3.3, S3.4** (Builder rewrite) → verify
   builder tests (T4, T5, T8)

4. **S2** (Touch sensor rewrite) can proceed after S3.1 (needs `MjObjectType::XBody`
   to exist for compilation, though touch doesn't use it). Touch builder change
   (S2.1) is independent; evaluation rewrite (S2.2) is independent → verify
   touch tests (T9, T10, T11)

5. **S4** (Frame sensor evaluation `XBody`/`Body` arms) after S3 (builder must
   produce `XBody`/`Body` values) → verify conformance tests (T6, T7)

6. **S5** (Geom acc sensors) after S3 (builder must resolve `objtype="geom"` for
   frame sensors) → verify acc tests (T12, T13, T14)

7. **T15, T16** (Regression) — run full domain test suite after all sections land.

---

## Out of Scope

- **DT-120** (`MjObjectType::Camera`) — deferred. No `cam_xpos`/`cam_xmat`/`cam_quat`
  arrays in `Data` yet. Conformance impact: none for typical RL/robotics models;
  camera frame sensors are rare.

- **DT-118** (`mj_contactForce()` equivalent) — deferred. Touch sensor reads
  `efc_force` directly. Conformance impact: 33% error for pyramidal contacts
  (EGT-2a). Correct for frictionless and elliptic contacts.

- **DT-119** (Ray-geom intersection filter for touch) — deferred. Touch sensor
  sums all contacts on the body without spatial filtering. Conformance impact:
  over-reports for small sensor sites on large bodies.

- **DT-63** (Frame sensor `reftype`/`refid` — relative-frame measurements) —
  Spec B scope. Parser separation of `reftype`/`refname` (S1) is prerequisite.
  Builder wiring and evaluation transforms are Spec B.

- **Sensor noise application** — runtime noise intentionally not applied (RL
  training parity). Not a conformance gap (noise is stochastic).
