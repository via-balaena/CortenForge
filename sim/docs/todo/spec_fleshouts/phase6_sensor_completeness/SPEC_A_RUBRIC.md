# Phase 6 Spec A — Spec Quality Rubric

Grades the Spec A spec (DT-62 objtype parsing, DT-64 multi-geom touch
aggregation, DT-102 geom-attached FrameLinAcc/FrameAngAcc) on 10 criteria.
Target: A+ on every criterion before implementation begins. A+ means "an
implementer could build this without asking a single clarifying question —
and the result would produce numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Empirical Ground Truth

All values verified against MuJoCo 3.5.0 via Python bindings. Scripts
available for reproduction during implementation.

### EGT-1: `body` vs `xbody` objtype distinction

MuJoCo `engine_sensor.c` — `get_xpos_xmat()` helper resolves frame sensor
position/rotation by `sensor_objtype[i]`:

| objtype | MuJoCo enum | Position source | Rotation source |
|---------|-------------|----------------|-----------------|
| `xbody` | `mjOBJ_XBODY` (2) | `d->xpos + 3*id` (joint frame) | `d->xmat + 9*id` |
| `body` | `mjOBJ_BODY` (1) | `d->xipos + 3*id` (inertial/COM frame) | `d->ximat + 9*id` |
| `geom` | `mjOBJ_GEOM` (5) | `d->geom_xpos + 3*id` | `d->geom_xmat + 9*id` |
| `site` | `mjOBJ_SITE` (6) | `d->site_xpos + 3*id` | `d->site_xmat + 9*id` |
| `camera` | `mjOBJ_CAMERA` (7) | `d->cam_xpos + 3*id` | `d->cam_xmat + 9*id` |

**Empirical verification (MuJoCo 3.5.0):** Body with capsule geom (mass=1,
COM offset 0.15m from joint) produces:

```
framepos objtype="body"  → xipos = [0.15, 0.0, 1.0]  (COM position)
framepos objtype="xbody" → xpos  = [0.0,  0.0, 1.0]  (joint frame origin)
Difference = [-0.15, 0.0, 0.0]
```

This is a **real, measurable** distinction. CortenForge currently has only
`MjObjectType::Body` — all frame sensors attached to a body use `xpos`/`xmat`
(joint frame), which corresponds to MuJoCo's `mjOBJ_XBODY`, not `mjOBJ_BODY`.
The spec must either: (a) add `XBody` to `MjObjectType` and implement both,
or (b) document the deviation and its impact.

**CortenForge `MjObjectType` enum** (at `enums.rs:496–512`): Has `None`,
`Body`, `Joint`, `Geom`, `Site`, `Actuator`, `Tendon`. Missing: `XBody`,
`Camera`. Current `Body` uses `xpos`/`xmat` (line `acceleration.rs:212`,
`position.rs` frame sensor arms), which maps to MuJoCo's `mjOBJ_XBODY`.

Frame quaternion resolution (`get_xquat()` in `engine_sensor.c`):

| objtype | Quaternion computation |
|---------|----------------------|
| `mjOBJ_XBODY` | `d->xquat[id]` (direct copy) |
| `mjOBJ_BODY` | `mulQuat(d->xquat[body], m->body_iquat[body])` |
| `mjOBJ_GEOM` | `mulQuat(d->xquat[geom_bodyid], m->geom_quat[geom])` |
| `mjOBJ_SITE` | `mulQuat(d->xquat[site_bodyid], m->site_quat[site])` |
| `mjOBJ_CAMERA` | `mulQuat(d->xquat[cam_bodyid], m->cam_quat[cam])` |

### EGT-2: Touch sensor multi-geom aggregation and `mj_contactForce()`

MuJoCo `engine_sensor.c` — `mjSENS_TOUCH` case in `mj_sensorAcc()`:

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

**Key MuJoCo behaviors:**
1. Touch sensor `objid` is a **site ID** (`sensor_objtype = mjOBJ_SITE`,
   `sensor_objid = site_id`).
2. Body resolved at runtime: `bodyid = m->site_bodyid[objid]`.
3. Contact iteration checks **body membership** (`geom_bodyid[con->geom[k]]`),
   not geom identity. ALL geoms on the body contribute.
4. **`mj_contactForce()` is NOT equivalent to summing `efc_force` rows.**
   (See EGT-2a below.)
5. **Ray-geom intersection filter**: Contact only contributes if a ray from
   the contact point along the (scaled, oriented) contact normal intersects
   the sensor site's geometric volume via `mju_rayGeom()`, using the site's
   type (sphere, capsule, box), position, orientation, and size.
6. Sign flip: if `bodyid == conbody[1]`, ray direction is negated.

#### EGT-2a: `mj_contactForce()` vs raw `efc_force` — critical conformance gap

**Empirical verification (MuJoCo 3.5.0, pyramidal friction cone=0):**

Body with 3 spheres (mass=1 each) resting on ground plane with touch sensor:

```
Contact 0 (floor↔g1): mj_contactForce()[0] = 9.807906, sum(efc_force) = 7.355929
Contact 1 (floor↔g2): mj_contactForce()[0] = 9.807906, sum(efc_force) = 7.355929
Contact 2 (floor↔g3): mj_contactForce()[0] = 9.807906, sum(efc_force) = 7.355929

Touch sensor value = 29.4237 ≈ 3 × 9.81 (all 3 geoms contribute)
```

**The 33% discrepancy** (`9.808` vs `7.356`) occurs because `mj_contactForce()`
reconstructs the physical normal force from pyramidal facet forces. For
pyramidal contacts with `dim=3` (1 normal + 2 friction), the facet forces are
projections onto pyramid edges — the normal force `= sqrt(sum(f_i²))` or a
scaling transform, NOT `sum(f_i)`. CortenForge's current touch sensor reads
`efc_force` directly and sums all facets — this produces **numerically wrong**
touch values for pyramidal contacts.

**CortenForge current behavior** (`acceleration.rs:143–181`,
`builder/sensor.rs:138–163`):
- Builder resolves site → body → **first geom on body** via
  `geom_body.iter().position(|&b| b == body_id)` (line 157–161). Returns
  `(MjObjectType::Geom, first_geom_id)`.
- Evaluation checks `c.geom1 == objid || c.geom2 == objid` against the single
  geom ID. Multi-geom bodies lose contacts on non-first geoms.
- No `mj_contactForce()` equivalent — reads `efc_force` directly. For pyramidal
  contacts, sums all facet forces; for elliptic/frictionless, reads first row.
  **This is numerically wrong** per EGT-2a.
- No ray-geom intersection filter — all contacts involving the geom are summed
  regardless of whether they fall within the sensor site's volume.

### EGT-3: Geom-attached FrameLinAcc/FrameAngAcc

MuJoCo `engine_sensor.c` — `mjSENS_FRAMELINACC`/`mjSENS_FRAMEANGACC`:

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

`mj_objectAcceleration()` handles any object type including `mjOBJ_GEOM`.
For geom objects, it resolves `body_id = m->geom_bodyid[objid]`, uses
`d->geom_xpos[objid]` as the target position, applies spatial motion
transport from body origin to geom position, and adds Coriolis correction.

CortenForge's `object_acceleration()` (`spatial.rs:297–323`) already
implements the full spatial transport + Coriolis logic. It takes `(data,
body_id, target_pos, local_rot)`. For geom objects, the caller just needs
`body_id = model.geom_body[objid]` and `target_pos = data.geom_xpos[objid]`.

**Empirical verification (MuJoCo 3.5.0):**

```
fla_geom (objtype=geom):  [1.21e-17, -1.98e-17, 9.808]
fla_site (objtype=site):  [1.21e-17, -1.98e-17, 9.808]
fla_body (objtype=body):  [1.21e-17, -1.98e-17, 9.808]
fla_xbody (objtype=xbody): [1.21e-17, -1.98e-17, 9.808]
faa_geom (objtype=geom):  [3.02e-16, -1.02e-17, 7.16e-17]
```

All four objtype variants produce identical FrameLinAcc for this symmetric
model (COM coincides with joint frame). The spec must verify with an
asymmetric model where geom position ≠ body origin to confirm spatial
transport correctness.

**Current CortenForge** (`acceleration.rs:207–238`):
- FrameLinAcc (line 208): matches Site and Body only. Geom falls through to
  `_` → writes zeros.
- FrameAngAcc (line 226): matches Site and Body only. Geom falls through to
  `_` → writes zeros.
- Both use `_` catch-all, so adding `MjObjectType::Geom` arm is additive.

**Important MuJoCo detail:** FrameLinAcc/FrameAngAcc **ignore refid** (no
reference-frame transform for acceleration sensors). This is confirmed in
the C source — the acceleration case has no `refid` check unlike position
and velocity cases.

### EGT-4: Analytical verification data

All empirical values have derivation chains from physics first principles.

**V1 — Touch sensor total force (EGT-2):**
Three geoms, each mass = 1 kg. Gravity = 9.81 m/s². Static equilibrium ⇒
total normal force = 3 × 1 × 9.81 = **29.43 N**. MuJoCo 3.5.0 reports
29.4237 (within solver tolerance of analytical value).

**V2 — `mj_contactForce()` normal force per contact (EGT-2a):**
Each geom: F_normal = mg = 1 × 9.81 = **9.81 N**. MuJoCo
`mj_contactForce()[0]` = 9.808 (within solver tolerance). Raw
`sum(efc_force)` = 7.356. Discrepancy factor: 9.808/7.356 = **1.333 ≈ 4/3**.
This arises from the pyramidal friction cone basis: for `dim=3` (1 normal +
2 tangent), the 4 pyramid edges project onto the normal axis with factor
`1/sqrt(1 + mu²)` per edge. The reconstruction `mj_contactForce()` inverts
this projection to recover the physical normal force; raw `efc_force` sums
the projected facet forces without inversion.

**V3 — body vs xbody position (EGT-1):**
Capsule geom, half-length = 0.15 m, aligned along x-axis, attached at body
origin. Uniform density ⇒ COM at geometric center ⇒ COM offset =
(0.15, 0, 0) from joint frame origin. MuJoCo: `xipos = [0.15, 0, 1]`
(COM frame), `xpos = [0, 0, 1]` (joint frame). Difference = COM offset.

**V4 — Geom-attached FrameLinAcc static (EGT-3):**
Static equilibrium, gravity = [0, 0, -9.81]. Body acceleration compensates
gravity: `cacc[linear] = [0, 0, 9.81]` (in world frame). Spatial transport
from body origin to geom position adds zero (ω = 0, α = 0 in static case).
Result: `FrameLinAcc = [0, 0, 9.81]`. MuJoCo reports `[1.2e-17, -2.0e-17,
9.808]` (gravity-compensated, within solver tolerance).

### EGT-5: Exhaustive match-site inventory

Every `match` on `MjObjectType` or `MjSensorType` in the sim crates that
Spec A touches or that new `MjObjectType` variants (`XBody`, `Camera`) would
affect.

#### Table 1: Exhaustive matches (compilation fails if enum gains variants)

| File | Line | Context | Current Arms | Required Change |
|------|------|---------|-------------|-----------------|
| `sensor/mod.rs` | 27 | `sensor_body_id()` | `Body`, `Joint`, `Geom`, `Site`, `Tendon\|Actuator\|None` | **Add `XBody` arm** (→ `Some(objid)`, same as Body); **add `Camera` arm** (→ camera's body or `None`). Without these, **compilation fails**. |

#### Table 2: Wildcard matches — semantically incorrect for new variants

These compile but produce **wrong results** for `XBody`/`Geom` because the
`_` catch-all returns zeros or identity.

| File | Line | Context | Current Arms | Required Change |
|------|------|---------|-------------|-----------------|
| `acceleration.rs` | 208 | FrameLinAcc | `Site`, `Body`, `_ → zeros` | **Add `Geom` arm** (DT-102): `model.geom_body[objid]`, `data.geom_xpos[objid]` |
| `acceleration.rs` | 226 | FrameAngAcc | `Site`, `Body`, `_ → zeros` | **Add `Geom` arm** (DT-102): `model.geom_body[objid]` |
| `acceleration.rs` | 143 | Touch eval | `Geom`, `_ → skip` | **Rewrite to Site-based** (DT-64): `model.site_body[objid]` → body-level contact iteration |
| `position.rs` | 108 | FramePos | `Site`, `Body`, `Geom`, `_ → zeros` | Add `XBody` arm: read `data.xipos` (not `xpos`) |
| `position.rs` | 119 | FrameQuat | `Site`, `Body`, `Geom`, `_ → identity` | Add `XBody` arm: read `mulQuat(xquat, body_iquat)` |
| `position.rs` | 141 | FrameAxis | `Site`, `Body`, `Geom`, `_ → identity` | Add `XBody` arm: read `data.ximat` |
| `velocity.rs` | 113 | Velocimeter | `Site`, `Body`, `_ → zeros` | Add `XBody` arm for correct reference point |
| `velocity.rs` | 132 | FrameLinVel | `Site`, `Body`, `_ → zeros` | Add `XBody` arm |
| `velocity.rs` | 148 | FrameAngVel | `Site`, `Body`, `_ → zeros` | Add `XBody` arm |

#### Table 3: Wildcard matches — no change needed

These either use types that don't receive new variants, or the `_` catch-all
is semantically correct (e.g., Rangefinder is always Site-attached).

| File | Line | Context | Reason |
|------|------|---------|--------|
| `position.rs` | 170 | Rangefinder | Equality check `== Site` only; always Site-attached |
| `position.rs` | 269 | Magnetometer | Wildcard; always Site/Body |
| `velocity.rs` | 87 | Gyro | Wildcard; always Site/Body |
| `acceleration.rs` | 83 | Accelerometer | Wildcard; always Site/Body |
| `acceleration.rs` | 103 | Force | Wildcard; always Site/Body |
| `acceleration.rs` | 125 | Torque | Wildcard; always Site/Body |
| `postprocess.rs` | 60 | Cutoff clamping | Type-agnostic for non-Touch/Rangefinder |

#### Table 4: `MjSensorType` matches — no change needed

No new `MjSensorType` variants proposed in Spec A. All exhaustive matches
(`enums.rs:427` dim(), `builder/sensor.rs:82/209/248`) are unaffected.

### EGT-6: Architectural decisions

Spec A must make explicit design decisions for these open questions. The
rubric does not prescribe the answer — it requires the spec to **choose
one, justify it, and document the conformance impact**.

**DECISION 1: `XBody` variant scope**

Should Spec A add `MjObjectType::XBody` now, or defer?

| Option | Trade-off |
|--------|-----------|
| (a) Add `XBody` now | Correct body/xbody distinction immediately. Touches 10+ match sites across position.rs, velocity.rs, acceleration.rs (Table 2). Larger blast radius but fixes a real conformance gap (EGT-1: 0.15m error for offset-COM bodies). |
| (b) Defer `XBody` to Spec B/C | Smaller Spec A scope. Current `Body` = MuJoCo's `XBody` (joint frame), which is what MJCF `body=` attribute defaults to. Spec A's parser stores `objtype` string, ready for later dispatch. **Risk:** frame sensors attached via `<framepos objtype="body" objname="b1"/>` will read `xpos` (wrong — should read `xipos`). |
| (c) Rename current `Body` → `XBody`, add new `Body` | Semantically cleanest: CortenForge names match MuJoCo names. **Risk:** breaks every existing `MjObjectType::Body` consumer. |

**DECISION 2: `Camera` variant scope**

Should Spec A add `MjObjectType::Camera`, or defer?

| Option | Trade-off |
|--------|-----------|
| (a) Add `Camera` now | Full MuJoCo objtype parity. Requires `model.cam_xpos`/`cam_xmat`/`cam_quat` arrays (may not exist yet). |
| (b) Defer with DT-ID | Cameras are rarely used as sensor objects. Defer without conformance risk for typical models. |

**DECISION 3: `mj_contactForce()` implementation scope**

Should Spec A implement `mj_contactForce()` equivalent for touch sensor, or
defer?

| Option | Trade-off |
|--------|-----------|
| (a) Implement now | Full touch conformance. Requires understanding pyramidal/elliptic/frictionless cone basis transforms. **Complex** — the reconstruction depends on `cone` flag, contact `dim`, and friction model. |
| (b) Defer with DT-ID + impact analysis | Touch sensor works for frictionless/elliptic contacts (where `efc_force[0]` IS the normal force). **33% error** for pyramidal contacts (EGT-2a). Spec must document which contact models are affected and quantify the deviation. |
| (c) Implement frictionless + elliptic, defer pyramidal | Partial conformance: correct for 2 of 3 contact models. Pyramidal deferred with quantified error. |

**DECISION 4: Ray-geom intersection filter**

Should Spec A implement `mju_rayGeom()` for touch sensor, or defer?

| Option | Trade-off |
|--------|-----------|
| (a) Implement now | Full touch conformance layer 4. Requires ray-geometry intersection for sphere/capsule/box/cylinder/ellipsoid site types. **Moderate complexity.** |
| (b) Defer with DT-ID + impact analysis | Touch sensor sums ALL contacts on the body, not just those whose normal ray intersects the sensor site volume. Over-reports for small sites on large bodies. **Impact depends on site size relative to body.** |

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest. If P1 is not A+, do not
> proceed to grading other criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file and exact behavior. **DT-62:** `get_xpos_xmat()` and `get_xquat()` helper dispatch tables cited with all 5 object types (`mjOBJ_XBODY`, `mjOBJ_BODY`, `mjOBJ_GEOM`, `mjOBJ_SITE`, `mjOBJ_CAMERA`) and their exact data sources — `xpos`/`xmat` for XBODY (joint frame), `xipos`/`ximat` for BODY (inertial/COM frame), `geom_xpos`/`geom_xmat`, `site_xpos`/`site_xmat`, `cam_xpos`/`cam_xmat`. Quaternion resolution for each type explicit (BODY uses `mulQuat(xquat, body_iquat)`, GEOM uses `mulQuat(xquat[geom_bodyid], geom_quat)`, etc.). `mjOBJ_BODY` vs `mjOBJ_XBODY` empirical difference documented (EGT-1 reference values: `xipos=[0.15,0,1]` vs `xpos=[0,0,1]` for 0.15m COM offset). **DT-64:** Touch sensor body-level aggregation described with exact C code flow: `site_bodyid[objid]` → iterate `ncon` contacts → `geom_bodyid[con->geom[k]]` body match → `mj_contactForce(m, d, j, conforce)` → `conforce[0]` normal force check → ray direction from `con->frame` scaled by `conforce[0]` + normalized → sign flip for `conbody[1]` → `mju_rayGeom()` intersection with site type/size/pose. Spec documents that `mj_contactForce()` is NOT equivalent to summing `efc_force` rows for pyramidal contacts (EGT-2a: 33% discrepancy verified empirically). `con->efc_address >= 0` guard documented (skips contacts without active constraints). **DT-102:** `mj_objectAcceleration(m, d, objtype, objid, tmp, 0)` for `mjOBJ_GEOM` — body resolution via `geom_bodyid`, position at `geom_xpos`, spatial transport + Coriolis. No refid for acceleration sensors (explicitly stated — no `refid` check in C source unlike position/velocity cases). Edge cases addressed: world body (`body_id == 0`), bodyless geom, site on world body, zero-mass body, sleeping bodies (`mj_sleepState` check), `mjDSBL_SENSOR` flag (early return), `flg_rnepost` lazy gate (triggers `mj_rnePostConstraint` for cacc/cfrc-dependent sensors). |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage (e.g., `mjOBJ_CAMERA` omitted, ray-geom intersection details incomplete, `mj_contactForce` vs `efc_force` difference not analyzed). |
| **B** | Correct at high level, but missing specifics — e.g., touch aggregation described as "body-level" without `mj_contactForce()` distinction or ray-geom filter, or objtype described without `mjOBJ_XBODY` vs `mjOBJ_BODY` empirical verification. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood, assumed, or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | **DT-62:** (1) Parser change: `get_attribute_opt(e, "objtype")` → `MjcfSensor.objtype: Option<String>` field. Separate `reftype`/`refname` conflation at `parser.rs:3470` (currently `get_attribute_opt(e, "reftype").or_else(|| get_attribute_opt(e, "refname"))` conflates two attributes into one field). (2) Builder `resolve_sensor_object()` rewrite for frame sensors (lines 165–186): replace site→body→geom heuristic with explicit objtype dispatch — string-to-enum mapping from MJCF `objtype` attribute (`"site"`→`MjObjectType::Site`, `"body"`→`MjObjectType::Body`, `"geom"`→`MjObjectType::Geom`, `"xbody"`→`MjObjectType::XBody`, `"camera"`→`MjObjectType::Camera`; or explicit deferral of XBody/Camera with DT-IDs). Default/fallback when `objtype` omitted specified (MuJoCo infers from which attribute name was used: `site=`→Site, `body=`→XBody, `objname=`→depends on sensor type). **DT-64:** (1) Builder touch resolution (lines 138–163): change from `(MjObjectType::Geom, first_geom_id)` to `(MjObjectType::Site, site_id)`. (2) Evaluation rewrite (lines 143–181): `objid` is now site_id; body resolution at runtime via `model.site_body[objid]`; contact iteration with `model.geom_body[c.geom1] == body_id \|\| model.geom_body[c.geom2] == body_id`; `mj_contactForce()` equivalent or explicit analysis of current `efc_force` approach vs MuJoCo's (with EGT-2a data); ray-geom intersection (implement or defer with DT-ID + impact analysis). Every loop, guard, and formula written in Rust-like pseudocode. **DT-102:** (1) FrameLinAcc Geom arm (after line 211): `MjObjectType::Geom if objid < model.ngeom => (model.geom_body[objid], data.geom_xpos[objid])` — calls existing `object_acceleration(data, body_id, &obj_pos, None)`. (2) FrameAngAcc Geom arm (after line 228): `MjObjectType::Geom if objid < model.ngeom => model.geom_body[objid]`. FrameAngAcc MUST read `cacc[body_id]` angular components directly (matching MuJoCo), NOT call `object_acceleration()` and discard the linear part — spatial transport only affects linear components; calling it for angular wastes computation and introduces floating-point drift. **Field existence:** For every model/data field referenced (e.g., `model.body_iquat`, `data.xipos`, `data.ximat`, `model.site_body`), the spec verifies the field exists in CortenForge's `Model`/`Data` structs (citing `model.rs`/`data.rs` line number) or specifies its addition with type, default, and population site. An implementer can type all three in without reading MuJoCo source. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit (e.g., bounds check pattern, `efc_address >= 0` guard). |
| **B** | Algorithm structure is clear but some steps are hand-waved or deferred without justification. |
| **C** | Skeleton only — "implement this somehow." |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present with every porting rule cell filled. Rules include: (1) `mjOBJ_BODY` uses `xipos`/`ximat` (inertial frame) — CortenForge's current `MjObjectType::Body` uses `xpos`/`xmat` (joint frame), which is MuJoCo's `mjOBJ_XBODY`; spec maps the correct CortenForge fields for each MuJoCo object type. (2) `MjObjectType` enum variants vs MuJoCo `mjtObj` values — missing `XBody` (2) and `Camera` (7); spec states which to add and which to defer. (3) `SpatialVector` layout `[angular; linear]` matches MuJoCo's `tmp[0..3]` = angular, `tmp[3..6]` = linear (confirmed consistent — no porting needed). (4) Touch sensor: `mj_contactForce()` vs current `efc_force` direct access — conformance difference documented with empirical data (EGT-2a: 33% discrepancy for pyramidal contacts); spec either implements equivalent or proves numerical equivalence for non-pyramidal contacts with explicit deviation scope. (5) `sensor_objtype`/`sensor_objid` naming matches MuJoCo model arrays (no porting needed). (6) Contact struct fields: `c.geom1`/`c.geom2` (CortenForge `contact_types.rs:59–60`) vs MuJoCo `con->geom[0]`/`con->geom[1]` — semantically identical. (7) `model.geom_body` (CortenForge, `model.rs:253`) vs `m->geom_bodyid` (MuJoCo) — same semantics. (8) Contact frame layout: MuJoCo's `con->frame[0..3]` is the contact normal (first row of a 3x3 rotation matrix stored as 9 floats). CortenForge's `Contact.frame` is `[Vector3<f64>; 2]` containing tangent vectors only; the normal is in `Contact.normal` separately. Any algorithm reading MuJoCo's `con->frame` for the normal direction must use `contact.normal` in CortenForge. (9) Contact normal direction: MuJoCo's `con->frame[0..3]` vs CortenForge's `Contact.normal` — spec documents whether both point in the same direction (geom1→geom2 or geom2→geom1). If conventions differ, the touch sensor sign-flip logic (`if bodyid == conbody[1]`) must be adjusted. Spec includes a concrete test verifying normal direction for a known contact pair. (10) Contact iteration pattern: MuJoCo iterates `d->contact[j]` (j=0..ncon) and checks `con->efc_address >= 0` to identify active contacts. CortenForge iterates `data.efc_type[ei]` forward and uses `data.efc_id[ei]` to map to `data.contacts[ci]`. The data flow is structurally inverted. The spec must translate the MuJoCo algorithm to CortenForge's iteration pattern, or justify adding MuJoCo-style iteration. Each rule verified to preserve numerical equivalence or explicitly documents where it diverges. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete MJCF model, (2) exact expected value or tolerance from MuJoCo's output, (3) what sensordata field to check. **DT-62:** AC with MJCF `<framepos objtype="geom" objname="g1"/>` → verify `model.sensor_objtype[i] == MjObjectType::Geom` and `model.sensor_objid[i] == geom_id` and `sensordata == data.geom_xpos[geom_id]`. AC with body having offset COM: `<framepos objtype="body" objname="b1"/>` → verify position reads `xipos` (= `[0.15, 0.0, 1.0]` from EGT-1) vs `<framepos objtype="xbody" objname="b1"/>` → `xpos` (= `[0.0, 0.0, 1.0]`). Both values MuJoCo-verified. **DT-64:** AC with body having 3 geoms, contacts on all geoms → verify `sensordata[0] ≈ 29.42` (MuJoCo 3.5.0, ±1e-4, from EGT-2). Negative AC: geom on different body → force NOT included. **DT-102:** AC with geom-attached `<framelinacc objtype="geom" objname="g1"/>` → verify acceleration at geom position matches MuJoCo output (EGT-3 values ± 1e-10). Code-review ACs (parser has `objtype` field, builder match arms present) explicitly labeled as structural. At least one AC per task has expected values verified against MuJoCo (stated as "MuJoCo 3.5.0 produces X; we assert X ± tolerance"). |
| **A** | ACs are testable. Some lack exact numerical expectations or MuJoCo-verified values. |
| **B** | ACs are directionally correct but vague ("output should change"), or values are manually calculated without MuJoCo verification. |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test, every test maps to ≥1 AC or is justified as supplementary. Explicit edge case inventory: world body (`body_id == 0` — body COM at origin, sensor reads zeros/identity), multi-geom body with contacts on all geoms (DT-64 regression), body with zero geoms (no contacts — touch sensor = 0), sleeping body with touch sensor (`SleepState::Asleep` → sensor skipped), geom-attached acceleration on world body, `mjDSBL_SENSOR` flag (all sensors return stale values), frame sensor with `objtype="body"` on body with offset COM (must read `xipos`, not `xpos`), frame sensor with `objtype="body"` on zero-mass body (verify `xipos`/`ximat` populated — MuJoCo sets to joint frame origin for massless bodies), `objtype` omitted (must fall back to heuristic or infer from attribute name). Negative cases tested: contact on wrong body → touch sensor unaffected; geom on different body → not aggregated; `objtype="geom"` on Touch sensor → error or fallback (touch only supports site). **Build-time regression:** at least one test loads an existing MJCF model that uses touch sensors (from conformance test suite or existing fixtures) and verifies it builds successfully with the same sensor count and sensor addresses as before the change. At least one MuJoCo-vs-CortenForge conformance test per task (DT-62, DT-64, DT-102) — test comment states MuJoCo version 3.5.0, expected value source, and tolerance. At least one test uses a non-trivial model (multi-body, multi-geom, offset COM) to catch bugs that only appear in non-symmetric configurations. |
| **A** | Good coverage. Minor edge-case gaps. Conformance tests present but not for all code paths. |
| **B** | Happy path covered. Edge cases and negative cases sparse. No explicit MuJoCo conformance tests. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated. No circular dependencies or implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous: S1 DT-62 (parser + builder objtype) → S2 DT-64 (touch rewrite — depends on S1's parser refactoring that separates `reftype`/`refname`) → S3 DT-102 (geom acc — depends on S1's explicit objtype dispatch to verify Geom arms work correctly). Each section states prerequisites. Cross-spec interactions: Spec B (reftype/refid) depends on Spec A's parser change separating `reftype` from `refname` at `parser.rs:3470` — spec states the handoff state. Spec C (new sensor types) uses Spec A's explicit objtype infrastructure in `resolve_sensor_object()` — spec states the expected API. No circular dependencies. Section ordering within the spec is explicit with rationale for each dependency. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced — implementer must infer. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break. Surprises are spec bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description: **sim-mjcf:** `parser.rs` lines 3453–3487 (add `objtype` attribute parsing, separate `reftype`/`refname` at line 3470), `types.rs` lines 3069–3101 (`MjcfSensor` struct: add `objtype: Option<String>` field, separate `reftype` field), `builder/sensor.rs` lines 68–204 (rewrite `resolve_sensor_object()` for frame sensors + touch keying change), lines 40–44 (`process_sensors()`: wire `objtype`). **sim-core:** `acceleration.rs` lines 143–181 (touch evaluation rewrite), lines 207–220 (FrameLinAcc: add Geom arm), lines 222–238 (FrameAngAcc: add Geom arm), lines 64–71 (lazy gate: add Geom to cacc-trigger list if needed), `enums.rs` lines 496–512 (`MjObjectType`: add XBody/Camera if scoped in), `sensor/mod.rs` lines 25–56 (`sensor_body_id`: add XBody/Camera arms). Behavioral changes: (1) touch sensor now body-level (moves toward MuJoCo conformance — current single-geom keying produces wrong results for multi-geom bodies), (2) frame sensor objtype now explicit (moves toward conformance — heuristic guessing produces wrong results when names collide), (3) geom acc sensors now produce non-zero values (moves toward conformance — currently return zeros). Existing test impact: names specific test suites — `sim-sensor` tests, `sim-conformance-tests`, Phase 4 position-stage sensor tests. States whether breakage is expected (touch sensor value changes for multi-geom models = expected, toward conformance) or unexpected (single-geom touch should be unchanged = regression guard). Existing test baseline: 2,238+ domain tests post-Phase 5. Exhaustive match sites that will error if `MjObjectType` gains variants: `sensor_body_id()` at `mod.rs:25`, `acceleration.rs` matches at lines 83/103/125/207/226 (all use `_` catch-all — additive, no breakage), `builder/sensor.rs:82` (exhaustive on `MjSensorType` — no new types in Spec A, no breakage). **Sentinel audit:** spec audits all resolution paths in `resolve_sensor_object()` for sentinel values (`usize::MAX`, `0` for invalid IDs) and confirms that evaluation code either bounds-checks before array access or proves sentinels cannot reach evaluation. **Downstream consumers:** spec lists all crates that read `Model.sensor_objtype` or `Model.sensor_objid` (search across workspace). For each consumer, spec states whether the behavioral change affects it and whether existing tests cover the change. At minimum: `sim-bevy` (L1 ECS integration), `sim-sensor` (test crate), `sim-conformance-tests`. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. Section references are accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "objtype" always means the MJCF attribute (string like `"geom"`), "object type" is the `MjObjectType` Rust enum, "sensor object type" is the `model.sensor_objtype[i]` array entry. File paths match between Specification sections and Files Affected table. AC numbers match between AC section and Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. Touch sensor keying convention consistent: if spec says `(MjObjectType::Site, site_id)` in one section, all other sections use the same — no stale references to `(MjObjectType::Geom, first_geom_id)`. Consumer/caller counts consistent across sections. `MjObjectType::Body` semantics consistent: if spec says it maps to `xpos`/`xmat` (joint frame), no section contradicts this by reading `xipos`. Field names spelled identically everywhere (`geom_body` not sometimes `geom_bodyid`, `site_body` not sometimes `site_bodyid`). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Touch Sensor Conformance Depth

> Evaluates whether the touch sensor rewrite achieves full MuJoCo conformance,
> not just the "multi-geom" fix. MuJoCo's touch sensor has four layers:
> (1) body-level contact iteration, (2) contact force computation via
> `mj_contactForce()`, (3) ray direction construction with sign flip,
> (4) ray-geom intersection filtering via `mju_rayGeom()`. A spec that only
> fixes layer 1 while ignoring layers 2–4 produces a partial conformance fix
> that diverges numerically (EGT-2a: 33% error for pyramidal contacts).

**Boundary with P1:** P1 grades whether the spec *documents* MuJoCo's touch
sensor behavior correctly. P9 grades whether the spec *designs* a
CortenForge implementation that reproduces all four layers of that behavior,
including explicit decisions about which layers to implement now vs defer.

| Grade | Bar |
|-------|-----|
| **A+** | All four layers addressed: (1) Body-level contact iteration with `model.geom_body[c.geom1] == body_id` matching — fully specified. **objid audit:** spec explicitly lists every use of `objid` in the Touch evaluation arm (`acceleration.rs:143–181`), confirms no code path compares `objid` (now a site_id) against geom indices (`c.geom1`/`c.geom2`), and states what `objid` means at each line (site_id, not geom_id). Any line that previously compared `objid` against geom fields is rewritten to compare body IDs. (2) Contact force computation — spec either implements `mj_contactForce()` equivalent that reconstructs normal force from pyramidal/elliptic/frictionless `efc_force` rows (requires understanding the contact model basis transform), or explicitly defers with DT-ID + quantitative impact analysis citing EGT-2a (33% error for pyramidal), and states which contact models produce correct results without it (frictionless: `efc_force[0]` IS the normal force; elliptic: first row IS normal force; pyramidal: facet→normal reconstruction needed). If implementing `mj_contactForce()`, the spec includes numerical edge-case analysis: (a) `mu = 0` (frictionless — bypass reconstruction, use first `efc_force` row), (b) near-zero `mu` (reconstruction matrix condition), (c) `dim = 1` vs `dim = 3` vs `dim = 4` special casing. AC includes a test with near-zero friction verifying no `NaN`/`Inf`. (3) Ray direction construction — `conray = normalize(contact.normal * conforce[0])` (using CortenForge's `Contact.normal`, NOT `contact.frame[0]` which is a tangent vector — see P3 rules 8-9), sign flip when `body_id == conbody[1]` — spec either implements or defers with impact analysis (without this, touch sensor double-counts contacts whose normal points away from the sensor site). (4) Ray-geom intersection — spec either implements `mju_rayGeom()` equivalent with site type/size/pose, or explicitly defers with DT-ID, conformance impact analysis (e.g., "without ray-geom filter, a touch sensor on a small site attached to a large body sums forces from contacts outside the site volume — MuJoCo would report zero for those contacts"), and names specific test cases that will produce different results without it. Deferred layers have concrete "what's different" analysis with MuJoCo-verified numerical examples. |
| **A** | All four layers acknowledged. One deferred with DT-ID but missing quantitative impact analysis. |
| **B** | Only body-level fix addressed. `mj_contactForce()` difference not mentioned or dismissed without analysis. |
| **C** | Touch sensor treated as simple multi-geom fix without awareness of the full MuJoCo algorithm. |

### P10. Parser-Builder-Evaluation Pipeline Integrity

> Evaluates whether the spec correctly threads the new `objtype` attribute
> through the full pipeline: MJCF parsing → `MjcfSensor` struct → builder
> resolution → model arrays → evaluation dispatch. A break at any stage
> silently loses the attribute. This is the most common source of Phase 5
> bugs (DT-9, DT-103).

**Boundary with P2:** P2 grades whether each stage's algorithm is complete.
P10 grades whether the stages *connect* — that the output of one stage
matches the input expectations of the next.

| Grade | Bar |
|-------|-----|
| **A+** | Full pipeline trace for `objtype`: (1) parser reads `get_attribute_opt(e, "objtype")` → `MjcfSensor.objtype: Option<String>`, (2) builder `resolve_sensor_object()` maps string to `MjObjectType` enum via explicit match (`"site"→Site`, `"body"→Body`, `"geom"→Geom`, `"xbody"→XBody`), validates against allowed types per sensor type (e.g., touch only allows Site; frame sensors allow Site/Body/XBody/Geom/Camera), resolves `objname` against the correct name map for the chosen type, stores result in `self.sensor_objtype.push(objtype)` + `self.sensor_objid.push(objid)`, (3) evaluation reads `model.sensor_objtype[sensor_id]` and dispatches to correct data arrays. **objtype-on-non-frame-sensors:** Spec explicitly states which sensor types honor the `objtype` attribute and which ignore it. For sensor types that ignore `objtype` (touch, accelerometer, gyro, velocimeter, force, torque, rangefinder, magnetometer), the builder ignores the parsed `objtype` value. Pipeline trace includes the "objtype ignored" path (e.g., `<touch site="s1" objtype="geom"/>` → builder ignores `objtype`, resolves via `site=` attribute → `(Site, site_id)`), not just the "objtype honored" path. Pipeline trace for touch keying change: (1) parser reads `site=` attribute → `objname`, (2) builder resolves to `(MjObjectType::Site, site_id)` (changed from `(MjObjectType::Geom, first_geom_id)`), (3) evaluation reads `model.sensor_objtype[sensor_id] == Site`, resolves `model.site_body[objid]` at runtime to get body_id, then iterates contacts. Each handoff has explicit field names on both sides. **Attribute provenance:** When `objtype` is omitted, the spec documents how the implementation determines the default object type. If inference depends on which MJCF attribute name was used (`site=` vs `body=` vs `geom=`), the spec either (a) adds a field to `MjcfSensor` preserving the attribute name, or (b) proves the current parser's try-site-first-then-body-then-geom heuristic at `parser.rs:3463–3468` produces identical results to MuJoCo's attribute-name-based inference for all valid MJCF inputs. **Note:** the current parser collapses `site=`, `body=`, `geom=`, and `objname=` into a single `objname` field, losing provenance — the spec must address this. Pipeline trace for `reftype`/`refname` separation: currently conflated at `parser.rs:3470`; after Spec A, `reftype` stored in new `MjcfSensor.reftype: Option<String>` field, `refname` in existing `MjcfSensor.refname` field. Builder wiring deferred to Spec B but field separation is Spec A's responsibility. |
| **A** | Pipeline is complete. One handoff detail left implicit. |
| **B** | Pipeline described at high level but individual handoffs not traced with field names. |
| **C** | Pipeline not discussed — individual stages treated in isolation. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names exact MuJoCo C functions
      (`get_xpos_xmat()`, `get_xquat()`, `mj_contactForce()`, `mju_rayGeom()`,
      `mj_objectAcceleration()`), exact CortenForge file:line references
      (`parser.rs:3470`, `builder/sensor.rs:138–163`, `acceleration.rs:143–181`,
      `acceleration.rs:207–220`, `acceleration.rs:226–238`, `spatial.rs:297`,
      `enums.rs:496–512`, `sensor/mod.rs:25–56`, `contact_types.rs:59–60`,
      `model.rs:253`), and specific edge cases (world body, `mjOBJ_XBODY`
      vs `mjOBJ_BODY` with empirical values, sleeping bodies, `mjDSBL_SENSOR`).
      Two reviewers could independently determine whether the spec cites
      `get_xquat()` with all 5 object types and quaternion formulas.
      EGT section includes analytical derivations (V1-V4), structured
      match-site tables (28 sites across 4 tables), and architectural
      decision points (4 decisions with enumerated options).

- [x] **Non-overlap:** P1 vs P9: P1 grades whether the spec *documents*
      MuJoCo's touch sensor correctly (all 4 layers cited from C source);
      P9 grades whether the spec *designs* a conformant implementation
      (including deferral decisions with quantitative impact). P2 vs P10:
      P2 grades algorithm completeness per stage (parser logic, builder
      logic, evaluation logic); P10 grades cross-stage data flow (does the
      output of parser.rs reach the input of evaluation.rs without loss?).
      P1 vs P10: P1 grades MuJoCo accuracy; P10 grades pipeline integrity.
      P3 vs P9: P3 grades convention translation (`efc_force` vs
      `mj_contactForce` documented); P9 grades whether the *implementation
      design* handles the difference. Each pair evaluates the same content
      from a different angle.

- [x] **Completeness:** 10 criteria cover: MuJoCo accuracy (P1), algorithms
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), touch conformance depth (P9), pipeline
      threading (P10). Could a spec be A+ on all 10 but still have a gap?
      No — P9 catches the touch sensor depth issue (4-layer algorithm,
      `mj_contactForce` 33% discrepancy) that P1/P2 alone would miss, and
      P10 catches pipeline breaks that per-stage criteria miss. EGT-5
      (match-site inventory) catches blast-radius misses that P7 prose alone
      would miss. EGT-6 (architectural decisions) forces the spec to make
      explicit deferral choices rather than ignoring scope questions.

- [x] **Gradeability:** Each criterion maps to specific spec sections:
      P1→MuJoCo Reference + Key Behaviors, P2→Specification (S1-S3),
      P3→Convention Notes, P4→Acceptance Criteria, P5→Test Plan + Traceability,
      P6→Prerequisites + Execution Order, P7→Risk & Blast Radius,
      P8→cross-cutting, P9→Touch sensor specification section + MuJoCo
      Reference (touch section), P10→Specification sections (all three
      tasks) + Files Affected. EGT sections (EGT-1 through EGT-6)
      provide reference data for grading P1/P4/P7/P9 — a grader can
      check spec claims against EGT tables without re-running MuJoCo.

- [x] **Conformance primacy:** P1 is tailored with 5 specific MuJoCo C
      functions, 5 object types, 3 data source tables, and explicit edge
      cases. P4 requires MuJoCo-verified expected values from EGT-1/2/3
      with analytical derivations from EGT-4 (V1-V4). P5 requires
      MuJoCo-vs-CortenForge conformance tests with version and tolerance
      stated. P9 evaluates touch sensor conformance at all 4 algorithm
      layers with empirical discrepancy data. EGT-6 requires architectural
      decisions to include conformance impact analysis. The rubric cannot
      produce an A+ spec that diverges from MuJoCo without explicit,
      justified, tracked deviations.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1 DT-62, S2 DT-64, S3 DT-102) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | S2 (Touch sensor specification), MuJoCo Reference (touch section) |
| P10 | S1 (objtype pipeline), S2 (touch keying change), Files Affected |

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
| P9. Touch Sensor Conformance Depth | | |
| P10. Parser-Builder-Evaluation Pipeline Integrity | | |

**Overall: —**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Initial draft lacked `mjOBJ_XBODY` vs `mjOBJ_BODY` distinction. MuJoCo uses inertial frame (`xipos`/`ximat`) for `mjOBJ_BODY` and joint frame (`xpos`/`xmat`) for `mjOBJ_XBODY`. CortenForge `MjObjectType::Body` uses `xpos`/`xmat` (= MuJoCo's XBODY, not BODY). | Rubric self-audit: reading `get_xpos_xmat()` C source | Added to P1 A+ bar and Empirical Ground Truth with empirical values from MuJoCo 3.5.0 (EGT-1). | Rev 1 |
| R2 | P9 | Initial draft treated touch sensor as simple "multi-geom" fix with 3 layers. Reading MuJoCo C source + running empirical tests revealed 4 layers (body iteration → contact force → ray direction → ray-geom intersection) and that layer 2 (`mj_contactForce`) produces a 33% discrepancy vs raw `efc_force` for pyramidal contacts (EGT-2a). | Rubric self-audit: MuJoCo 3.5.0 empirical testing | Added P9 with 4-layer analysis. Reclassified from 3 layers to 4. Added EGT-2a with empirical values. | Rev 2 |
| R3 | P1 | Initial draft lacked `mjOBJ_CAMERA` in objtype dispatch. MuJoCo supports Camera as frame sensor object type. CortenForge `MjObjectType` lacks Camera variant. | Rubric self-audit: reading `get_xpos_xmat()` switch cases | Added Camera to P1 A+ bar objtype list. | Rev 1 |
| R4 | P3 | Need explicit comparison between CortenForge's current `efc_force` direct access for touch sensors vs MuJoCo's `mj_contactForce()`. Empirical testing confirmed 33% discrepancy for pyramidal contacts. | Rubric self-audit + MuJoCo 3.5.0 empirical testing | Added to P3 A+ bar item (4): must document `efc_force` vs `mj_contactForce()` equivalence with EGT-2a data. Also added to P9 layer 2. | Rev 2 |
| R5 | P10 | Pipeline integrity for `objtype` not covered by P1-P8. Phase 5 had multiple bugs (DT-9, DT-103) from attributes getting lost between parser → builder → model → evaluation stages. | Rubric self-audit: reviewing Phase 5 lessons | Added P10 (Parser-Builder-Evaluation Pipeline Integrity). | Rev 1 |
| R6 | P1 | Acceleration sensors ignore refid in MuJoCo (confirmed from C source — no refid check in FRAMELINACC/FRAMEANGACC case). Important for Spec B coordination. | Rubric self-audit: reading acceleration sensor C code | Added to Empirical Ground Truth and P1 A+ bar. | Rev 1 |
| R7 | P7 | Initial draft lacked specific match site line numbers. Phase 5 rubrics cite exact line numbers for every match site. CortenForge audit found: `sensor_body_id()` at `mod.rs:25–56` (exhaustive on MjObjectType), `acceleration.rs` FrameLinAcc at line 207 and FrameAngAcc at line 226 (both `_` catch-all), `builder/sensor.rs:82` (exhaustive on MjSensorType), `parser.rs:3470` (reftype/refname conflation site). | Codebase audit agent | Added all line references to P7 A+ bar and P2 A+ bar. | Rev 2 |
| R8 | P2 | Initial draft lacked line-level codebase references in algorithm descriptions. Phase 5 rubrics reference exact file:line for every code change. | Comparison with Phase 5 Spec A/B rubrics | Added exact line references throughout P2 A+ bar: `parser.rs:3470`, `builder/sensor.rs:138–163`, `builder/sensor.rs:165–186`, `acceleration.rs:143–181`, `acceleration.rs:207–220`, `acceleration.rs:226–238`. | Rev 2 |
| R9 | P4 | Initial draft lacked MuJoCo-verified reference values. Phase 5 rubrics require at least one AC per feature with values from running MuJoCo. | Comparison with Phase 5 Spec A rubric | Added EGT reference values to P4 A+ bar: EGT-1 body/xbody distinction (`[0.15,0,1]` vs `[0,0,1]`), EGT-2 touch value (`29.42`), EGT-3 geom acceleration (`9.808`). | Rev 2 |
| R10 | P5 | Initial draft lacked "non-trivial model" requirement. Phase 5 rubric P5 requires at least one test using multi-body/multi-joint model. | Comparison with Phase 5 Spec A rubric (R10) | Added to P5 A+ bar: "At least one test uses a non-trivial model (multi-body, multi-geom, offset COM)." | Rev 2 |
| R11 | P9 | Ray direction construction is a distinct algorithmic step MuJoCo performs between `mj_contactForce()` and `mju_rayGeom()`: `conray = normalize(con.frame * conforce[0])` with sign flip for `conbody[1]`. This was lumped with ray-geom in initial draft but is a separate conformance dimension (without it, double-counting occurs). | Careful re-reading of touch sensor C code | Split from 3 layers to 4 layers: (1) body iteration, (2) contact force, (3) ray direction + sign flip, (4) ray-geom intersection. | Rev 2 |
| R12 | P10 | `reftype`/`refname` separation pipeline trace missing. Spec A separates the parser fields (Contract 3 in umbrella) but the rubric didn't require P10 to verify this handoff. | Cross-reference with PHASE6_UMBRELLA.md Contract 3 | Added to P10 A+ bar: pipeline trace for `reftype`/`refname` separation (parser field split → Spec B handoff). | Rev 2 |
| R13 | P2 | Default/fallback behavior when `objtype` attribute is omitted was not in P2 bar. MuJoCo infers objtype from which attribute name was used (`site=`→Site, `body=`→XBody for frame sensors, `objname=`→depends). CortenForge must implement matching inference. | MuJoCo MJCF schema analysis | Added to P2 A+ bar: default/fallback inference logic when `objtype` omitted. | Rev 2 |
| R14 | P3 | CortenForge `contact_types.rs` field names and `model.rs` field names not mapped to MuJoCo equivalents. Phase 5 rubrics map every field. | Comparison with Phase 5 Spec B rubric P3 (8 convention items) | Added P3 items (6) and (7): `c.geom1`/`c.geom2` mapping and `model.geom_body` mapping. | Rev 2 |
| R15 | P4 | Empirical values stated without analytical derivation chains. Phase 5 Spec C rubric has V1-V7 with step-by-step math from first principles to expected value. Without derivations, the rubric can't distinguish "MuJoCo happens to output X" from "X is physically correct." | Structural comparison with Phase 5 Spec C rubric | Added EGT-4: Analytical verification data with derivations V1-V4 (touch total force, `mj_contactForce` discrepancy factor 4/3 from pyramidal basis, body/xbody COM offset, geom acceleration static case). | Rev 3 |
| R16 | P7 | Match sites discussed in P7 prose but not structured as lookup tables. Phase 5 Spec C rubric has 4 structured tables (exhaustive, wildcard, no-change, MjSensorType) with File/Line/Context/Required-Change columns. An implementer needs these as a checklist. | Structural comparison with Phase 5 Spec C rubric | Added EGT-5: Exhaustive match-site inventory with 4 tables (1 exhaustive compilation-breaker, 9 wildcard-but-wrong, 7 wildcard-no-change, MjSensorType summary). Full codebase audit with 28 match sites catalogued. | Rev 3 |
| R17 | P2, P9 | Architectural decisions implicit in rubric (add XBody? implement `mj_contactForce` now or defer? ray-geom filter?) but not structured as explicit choice points with trade-off analysis. Phase 5 Spec C rubric has "DECISION NEEDED" flags with enumerated options. | Structural comparison with Phase 5 Spec C rubric | Added EGT-6: Architectural decisions with 4 decision points (XBody scope, Camera scope, `mj_contactForce` scope, ray-geom filter scope), each with 2-3 options and trade-off analysis. Rubric requires spec to choose and justify, not prescribing the answer. | Rev 3 |
| R18 | P9 | Touch keying change from `(Geom, geom_id)` to `(Site, site_id)` creates type-confusion risk: `objid` changes meaning but all code paths use `usize`, so compiler won't catch site_id compared against geom indices. A spec could pass P9 by describing body-level iteration correctly while residual code still compares `objid` against `c.geom1`/`c.geom2`. | "A+ but still broken" thought experiment | Added to P9 A+ bar: explicit `objid` audit requirement — spec must list every use of `objid` in touch eval arm and confirm no code path compares it against geom indices. | Rev 4 (stress test) |
| R19 | P3 | Contact frame field layout differs between MuJoCo (3x3 matrix, normal in first row = `con->frame[0..3]`) and CortenForge (`Contact.normal` separate, `Contact.frame` = 2 tangent vectors). Transcribing MuJoCo's `mju_scl3(conray, con->frame, conforce[0])` as `contact.frame[0] * conforce[0]` uses a tangent vector instead of the normal. Also: normal direction convention may differ, requiring sign-flip logic adjustment. | "A+ but still broken" thought experiment | Added P3 rules (8), (9), (10): contact frame layout, normal direction convention, contact iteration pattern inversion. | Rev 4 (stress test) |
| R20 | P10 | Parser loses MJCF attribute provenance. Current parser at `parser.rs:3463–3468` collapses `site=`, `body=`, `geom=`, `objname=` into a single `objname` field. When `objtype` is omitted, MuJoCo infers type from which attribute name was used, but CortenForge's parser has already lost this information. A spec could document MuJoCo's inference logic perfectly (A+ P1) while the parser can't implement it (broken P10). | "A+ but still broken" thought experiment | Added to P10 A+ bar: attribute provenance requirement — spec must either add a field preserving which MJCF attribute was used, or prove the existing heuristic produces identical results to MuJoCo's inference. | Rev 4 (stress test) |
| R21 | P10 | `objtype` parsed for ALL sensor types by the parser, but only meaningful for frame sensors in MuJoCo. A spec could pass P10 by tracing the frame-sensor `objtype` path while `<touch site="s1" objtype="geom"/>` bypasses the DT-64 fix by routing through the `objtype` dispatch instead of the touch resolution path. | "A+ but still broken" thought experiment | Added to P10 A+ bar: "objtype-on-non-frame-sensors" path — spec must state which sensor types honor `objtype` and which ignore it, including pipeline trace for the "ignored" case. | Rev 4 (stress test) |
| R22 | P2 | Spec references model/data fields (`body_iquat`, `xipos`, `ximat`) that may not exist in CortenForge's structs. A spec could cite the correct MuJoCo formula (A+ P1) and write complete pseudocode (A+ P2) but reference fields that don't exist, causing compilation failure. | "A+ but still broken" thought experiment | Added to P2 A+ bar: field existence requirement — for every model/data field referenced, spec verifies it exists in CortenForge's structs (citing line number) or specifies its addition. | Rev 4 (stress test) |
| R23 | P5 | No build-time regression test required. Touch keying change could cause existing MJCF models to fail at build time (e.g., if site name lookup fails where geom heuristic previously succeeded). Spec could pass all P5 edge cases (runtime behavior) while models break at construction time. | "A+ but still broken" thought experiment | Added to P5 A+ bar: build-time regression test — at least one test loads an existing MJCF with touch sensors and verifies it builds with same sensor count and addresses. | Rev 4 (stress test) |
| R24 | P7 | No downstream consumer audit. Touch keying change affects `Model.sensor_objtype` semantics. Any crate reading `sensor_objtype == Geom` for touch sensors (e.g., `sim-bevy` L1 ECS integration) would silently break. All criteria focus on L0 crates. | "A+ but still broken" thought experiment | Added to P7 A+ bar: downstream consumer search across workspace for `sensor_objtype`/`sensor_objid` readers, with per-consumer impact assessment. | Rev 4 (stress test) |
| R25 | P7 | Sentinel values (`usize::MAX`) in resolution paths could cause panics. `builder/sensor.rs:161` uses `unwrap_or(usize::MAX)`. If any resolution path returns this sentinel, evaluation code indexes out of bounds. | "A+ but still broken" thought experiment | Added to P7 A+ bar: sentinel audit for all `resolve_sensor_object()` paths. | Rev 4 (stress test) |
| R26 | P9 | If spec implements `mj_contactForce()`, no criterion checked numerical stability. Pyramidal reconstruction involves `1/mu` divisions. For near-zero friction (`mu → 0`), this produces `NaN`/`Inf`. MuJoCo special-cases `dim=1` (frictionless). | "A+ but still broken" thought experiment | Added to P9 A+ bar: if implementing `mj_contactForce()`, require numerical edge-case analysis for `mu=0`, near-zero `mu`, and dim-specific special casing. AC must include near-zero friction test. | Rev 4 (stress test) |
| R27 | P3 | Contact iteration pattern structurally inverted between MuJoCo and CortenForge. MuJoCo iterates `d->contact[j]` forward, CortenForge iterates `efc_type[ei]` and maps via `efc_id[ei]`. Transcribing MuJoCo's algorithm literally would not work. | "A+ but still broken" thought experiment | Added to P3 A+ bar as rule (10): contact iteration pattern translation requirement. | Rev 4 (stress test) |
