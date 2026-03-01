# Phase 6 Spec C — Spec Quality Rubric

Grades the Spec C spec (§62 missing sensor types: `Clock`, `JointActuatorFrc`,
`GeomDist`, `GeomNormal`, `GeomFromTo`) on 10 criteria. Target: A+ on every
criterion before implementation begins. A+ means "an implementer could build
this without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment from Umbrella

The umbrella spec (§62) listed 6 sensor types: `clock`, `jointactuatorfrc`,
`camprojection`, `geomdist`, `geompoint`, `geomnormal`. Empirical verification
against MuJoCo 3.5.0 revealed scope corrections:

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| `geompoint` | **Does not exist.** `<geompoint>` produces "unrecognized element" error. | **Drop.** Replace with `geomfromto` (MJCF element `fromto`), which does exist (dim=6, `mjSENS_GEOMFROMTO`). |
| `camprojection` | Exists (`mjSENS_CAMPROJECTION`, dim=2, objtype=Site, reftype=Camera). Requires `cam_xpos`, `cam_xmat`, `cam_resolution`, `cam_fovy`, `cam_intrinsic`, `cam_sensorsize`. | **Defer.** CortenForge has zero camera infrastructure — no `MjObjectType::Camera`, no `cam_xpos`/`cam_xmat`/`cam_resolution`/`cam_fovy`/`cam_intrinsic`/`cam_sensorsize` fields in Model/Data. Implementing `CamProjection` requires first building the camera subsystem, which is outside Phase 6 scope. Track as DT-120 (camera deferral — per `future_work_15.md:287` and `index.md:114`). |

**Also not in scope:** `InsideSite` (`mjSENS_INSIDESITE`) exists in MuJoCo 3.5.0
but was not listed in the umbrella spec. Requires geometric containment testing.
Track as DT-121 (DT-118 = contactForce, DT-119 = ray-geom filter, DT-120 =
camera — all taken per `future_work_15.md`).

**Spec C final scope: 5 sensor types**
1. `Clock` (MJCF `clock`) — 1D, position stage
2. `JointActuatorFrc` (MJCF `jointactuatorfrc`) — 1D, acceleration stage
3. `GeomDist` (MJCF `distance`) — 1D, position stage
4. `GeomNormal` (MJCF `normal`) — 3D, position stage
5. `GeomFromTo` (MJCF `fromto`) — 6D, position stage

---

## Empirical Ground Truth

All values verified against MuJoCo 3.5.0 via Python bindings (`mujoco`
package). Scripts reproducible.

### EGT-1: Clock sensor

MuJoCo `engine_sensor.c` — `mj_computeSensorPos()`:
```c
case mjSENS_CLOCK:
  sensordata[0] = d->time;
  break;
```

| Property | Value |
|----------|-------|
| Enum value | `mjSENS_CLOCK` (45) |
| Dim | 1 |
| Datatype | `mjDATATYPE_REAL` (0) |
| Needstage | `mjSTAGE_POS` (1) |
| Objtype | `mjOBJ_UNKNOWN` (0) — no object attachment |
| Objid | -1 |
| Reftype | `mjOBJ_UNKNOWN` (0) — no reference |
| Refid | -1 |

**Timing behavior (empirical):**
- `mj_forward(t=0)` → sensor=0.0
- `mj_step` advances time *after* `mj_forward`, so sensor always reads the
  time at the *start* of the step, lagging by one timestep after `mj_step`.
- This is expected: sensors are evaluated inside `mj_forward`, before
  time is incremented.

**MJCF attributes:** `name`, `nsample`, `interp`, `delay`, `interval`,
`cutoff`, `noise`, `user`. No object name attributes.

### EGT-2: JointActuatorFrc sensor

MuJoCo `engine_sensor.c` — `mj_computeSensorAcc()`:
```c
case mjSENS_JOINTACTFRC:
  sensordata[0] = d->qfrc_actuator[m->jnt_dofadr[objid]];
  break;
```

| Property | Value |
|----------|-------|
| Enum value | `mjSENS_JOINTACTFRC` (16) |
| Dim | 1 |
| Datatype | `mjDATATYPE_REAL` (0) |
| Needstage | `mjSTAGE_ACC` (3) |
| Objtype | `mjOBJ_JOINT` (3) |
| Reftype | `mjOBJ_UNKNOWN` (0) |

**Semantic difference from `ActuatorFrc`:** `ActuatorFrc` reads
`data.actuator_force[actuator_id]` — the force of a *single* actuator.
`JointActuatorFrc` reads `data.qfrc_actuator[dof_adr]` — the *net* actuator
force at a joint DOF (sum of all actuators acting on that joint, through their
gears).

**Empirical verification:**
```
ctrl = [3.0, 4.0], gear = [1, 2] (two motors on same joint)
qfrc_actuator = 11.0 (3*1 + 4*2)
jointactuatorfrc sensor = 11.0 ✓
actuatorfrc a1 = 3.0, actuatorfrc a2 = 4.0 ✓
```

**Zero-actuator case:** When no actuators act on a joint,
`qfrc_actuator[dof_adr]` stays at its reset value of 0.0 (zeroed during
`Data::reset()`). The sensor reads 0.0. This is a valid edge case for testing.

**Joint type restriction:** MuJoCo's compiler enforces `joint must be slide or
hinge in sensor` — ball and free joints are rejected at parse time. This matches
`JointPos`/`JointVel` which also only support 1-DOF joints. The restriction is
compile-time, not runtime.

**MJCF attributes:** `name`, `joint`, `nsample`, `interp`, `delay`,
`interval`, `cutoff`, `noise`, `user`.

### EGT-3: GeomDist / GeomNormal / GeomFromTo sensors

MuJoCo `engine_sensor.c` — `mj_computeSensorPos()`.
Three sensor types share a single case block. All call `mj_geomDistance()`
to compute the minimum signed distance between geom pairs.

```c
case mjSENS_GEOMDIST:
case mjSENS_GEOMNORMAL:
case mjSENS_GEOMFROMTO:
{
    mjtNum cutoff = m->sensor_cutoff[i];
    mjtNum dist = cutoff;          // initialize to cutoff
    mjtNum fromto[6] = {0};

    // Resolve geom lists — body ⇒ iterate body's geoms; geom ⇒ single
    int n1, id1;
    if (objtype == mjOBJ_BODY) {
        n1 = m->body_geomnum[objid];
        id1 = m->body_geomadr[objid];
    } else {
        n1 = 1; id1 = objid;
    }
    int n2, id2;
    if (reftype == mjOBJ_BODY) {
        n2 = m->body_geomnum[refid];
        id2 = m->body_geomadr[refid];
    } else {
        n2 = 1; id2 = refid;
    }

    // All-pairs minimum distance
    for (int geom1=id1; geom1 < id1+n1; geom1++) {
        for (int geom2=id2; geom2 < id2+n2; geom2++) {
            mjtNum fromto_new[6] = {0};
            mjtNum dist_new = mj_geomDistance(m, d, geom1, geom2, cutoff, fromto_new);
            if (dist_new < dist) {
                dist = dist_new;
                mju_copy(fromto, fromto_new, 6);
            }
        }
    }

    // Output
    if (type == mjSENS_GEOMDIST)   sensordata[0] = dist;
    else if (type == mjSENS_GEOMNORMAL) {
        mjtNum normal[3] = {fromto[3]-fromto[0], fromto[4]-fromto[1], fromto[5]-fromto[2]};
        if (normal[0] || normal[1] || normal[2]) mju_normalize3(normal);
        mju_copy3(sensordata, normal);
    }
    else mju_copy(sensordata, fromto, 6);  // GEOMFROMTO
}
```

**Metadata table:**

| Type | Enum | Dim | Datatype | Stage | Objtype | Reftype |
|------|------|-----|----------|-------|---------|---------|
| `GeomDist` | `mjSENS_GEOMDIST` (39) | 1 | `mjDATATYPE_REAL` (0) | POS | Geom or Body | Geom or Body |
| `GeomNormal` | `mjSENS_GEOMNORMAL` (40) | 3 | `mjDATATYPE_AXIS` (2)* | POS | Geom or Body | Geom or Body |
| `GeomFromTo` | `mjSENS_GEOMFROMTO` (41) | 6 | `mjDATATYPE_REAL` (0) | POS | Geom or Body | Geom or Body |

*`GeomNormal` uses `mjDATATYPE_AXIS` (2), NOT `mjDATATYPE_QUATERNION` (3).
MuJoCo's `apply_cutoff()` only clamps REAL and POSITIVE datatypes — AXIS
and QUATERNION are implicitly skipped (no matching branch in the `if/else if`
chain). Additionally, MuJoCo's compiler normally rejects `cutoff` on
AXIS/QUATERNION sensors (`"cutoff applied to axis or quaternion datatype"`),
but `GeomNormal` is **explicitly exempted** from this compiler check
(`type != mjSENS_GEOMNORMAL` in the condition at `user_objects.cc`).
`GeomDist` and `GeomFromTo` have REAL datatype, so the AXIS/QUATERNION
check never triggers for them. All three geom distance sensors use
`sensor_cutoff` as a search radius for `mj_geomDistance()`, not for
postprocess clamping.

**Architectural note:** CortenForge does NOT have a `mjDATATYPE` equivalent
enum — `MjSensorDataType` (`enums.rs:480–492`) stores the pipeline STAGE
(`Position`/`Velocity`/`Acceleration`), not the data kind
(`REAL`/`POSITIVE`/`AXIS`/`QUATERNION`). Therefore the AXIS-based postprocess
exemption mechanism cannot be replicated via datatype check. Both GeomNormal
and GeomFromTo must be exempted via **explicit sensor type matching** in
`postprocess.rs` (see AD-3, option a).

MuJoCo's `apply_cutoff()` postprocess logic (simplified from
`engine_sensor.c:64–89` — local variables expanded for clarity):
```c
static void apply_cutoff(const mjModel* m, int i, mjtNum* data) {
  mjtNum cutoff = m->sensor_cutoff[i];
  if (cutoff <= 0) return;

  mjtSensor type = (mjtSensor)m->sensor_type[i];
  if (type == mjSENS_CONTACT || type == mjSENS_GEOMFROMTO) return;

  int dim = m->sensor_dim[i];
  for (int j = 0; j < dim; j++) {
    if (m->sensor_datatype[i] == mjDATATYPE_REAL)
      data[j] = mju_clip(data[j], -cutoff, cutoff);
    else if (m->sensor_datatype[i] == mjDATATYPE_POSITIVE)
      data[j] = mju_min(cutoff, data[j]);
    // AXIS, QUATERNION: no branch → implicitly skipped
  }
}
```

**MJCF element names differ from enum names:**
- `distance` (not `geomdist`) — attributes: `geom1`, `geom2`, `body1`, `body2`, `cutoff` (schema says required, runtime defaults to 0 — see below)
- `normal` (not `geomnormal`) — attributes: `geom1`, `geom2`, `body1`, `body2`, `cutoff` (schema says required, runtime defaults to 0 — see below)
- `fromto` (not `geomfromto`) — attributes: `geom1`, `geom2`, `body1`, `body2`, `cutoff` (schema says required, runtime defaults to 0 — see below)

**Cutoff defaults to 0 for all sensors:** Although the dm_control bundled
`schema.xml` marks `cutoff` as `required="true"` for `<distance>`, `<normal>`,
and `<fromto>`, MuJoCo 3.5.0's actual parser accepts omitted cutoff and defaults
to 0.0. Empirically verified: `<distance geom1="g1" geom2="g2"/>` (no cutoff)
compiles without error and stores `cutoff=0.0`. CortenForge must NOT enforce
`required` — doing so would reject valid MJCF files.

**Object resolution rules:**
- **Strict XOR on each side:** exactly one of `{geom1, body1}` and exactly one of `{geom2, body2}` must be specified. Specifying both on the same side (e.g., `geom1` + `body1`) is a parse error: `"exactly one of (geom1, body1) must be specified"`. Specifying neither is also a parse error: `"exactly one of (geom2, body2) must be specified"`.
- If `geom1`/`geom2` specified → objtype/reftype=`mjOBJ_GEOM`, single geom per side
- If `body1`/`body2` specified → objtype/reftype=`mjOBJ_BODY`, iterate all body's **direct** geoms only (uses `body_geomadr`/`body_geomnum`, does NOT traverse subtree/child bodies)
- **Mixed combinations supported:** `geom1` + `body2` (objtype=GEOM, reftype=BODY) and `body1` + `geom2` (objtype=BODY, reftype=GEOM) are both valid. All 4 combinations (geom×geom, geom×body, body×geom, body×body) work.
- Same-body geoms: two geoms on the same body CAN be distance-measured (no restriction). Self-distance (geom1==geom2) is rejected by compiler.

**FromTo output format:** `[from_x, from_y, from_z, to_x, to_y, to_z]` — nearest surface point on geom1 (or minimum-distance geom from body1), nearest surface point on geom2 (or minimum-distance geom from body2). Swapping `geom1`↔`geom2` swaps from↔to. When geoms penetrate, the surface points go **inside** the opposite geom's volume (from point can be past to point along the contact axis).

**Beyond-cutoff behavior:** When actual distance > cutoff (geoms too far apart), the evaluation loop never updates — `fromto` stays at initialization `{0}` and `dist` stays at cutoff. Therefore: GeomDist returns cutoff value, GeomNormal returns `[0,0,0]` (zero vector), GeomFromTo returns `[0,0,0,0,0,0]`. This is a critical asymmetry: distance clamps to cutoff; normal and fromto zero out entirely.

**Negative cutoff:** MuJoCo compiler rejects negative cutoff values at compile time: `"negative cutoff in sensor"`. Not a runtime case.

**Noise:** The `noise` attribute is stored in `sensor_noise` but **not applied** during `mj_forward`. The forward pass is fully deterministic. Noise is metadata for downstream/user-side injection.

**Cutoff is universal:** Cutoff is NOT restricted to geom distance sensors. Clock with `cutoff=5` gets clamped at 5.0 by postprocess. JointActuatorFrc with `cutoff=5` clamps to `[-5, +5]`. The `apply_cutoff()` function runs on ALL sensors with `cutoff > 0` — it is a generic postprocessing step, not per-sensor-type logic. (GeomFromTo and GeomNormal are still exempt via their specific exemption mechanisms.)

**Per-element clamping:** For multi-dimensional sensors (dim > 1), cutoff clamping is applied **independently to each scalar element**, NOT as a vector magnitude clamp. Verified: gyro with `qvel=[2, 0.5, -3]` and `cutoff=1.0` → `[1.0, 0.5, -1.0]` (each element clamped to `[-1, 1]` independently).

**Compiler cutoff rejection for AXIS/QUATERNION:** MuJoCo's compiler rejects `cutoff` on sensors with AXIS or QUATERNION datatypes: `"cutoff applied to axis or quaternion datatype in sensor"`. Only `GeomNormal` needs an explicit exemption from this check — the compiler condition is `(datatype == AXIS && type != mjSENS_GEOMNORMAL)`, so `GeomNormal` (AXIS datatype) with cutoff compiles without error. `GeomDist` and `GeomFromTo` both have `mjDATATYPE_REAL`, so the AXIS/QUATERNION check never triggers for them — they are unaffected, not exempted.

**Geom type support:** `mj_geomDistance()` works with ALL geom types — sphere, box, capsule, cylinder, ellipsoid, and plane. Verified empirically: box-sphere (dist=1.2), capsule-capsule (dist=0.8), cylinder-sphere (dist=1.5), plane-sphere (dist=0.9), ellipsoid-sphere (dist=1.6), rotated box-box (dist=0.793). The rotated box-box case (involving sqrt(2)) confirms GJK/MPR is used, not analytic shortcuts.

**Cutoff semantics (critical):**
- `cutoff=0` (default): dist is initialized to 0. `mj_geomDistance()` still
  computes. For **non-penetrating** geoms, `dist_new >= 0`, so `dist_new < dist`
  (i.e., `>= 0 < 0`) is false → dist stays 0 → sensor returns 0.0. For
  **penetrating** geoms, `dist_new < 0`, so `dist_new < 0` is true → dist
  updates to the negative penetration depth → sensor returns the negative value.
  **Summary:** cutoff=0 suppresses positive distances but NOT penetration.
  Empirically verified: non-overlapping spheres with cutoff=0 → 0.0;
  overlapping spheres with cutoff=0 → -0.7.
- `cutoff>0`: distance is initialized to cutoff; `mj_geomDistance` uses cutoff
  as the search radius. Output is `min(actual_dist, cutoff)`.
- Postprocess cutoff clamping also applies to `GeomDist` (REAL type), but
  `GeomFromTo` is exempted. `GeomNormal` is exempted via AXIS datatype (implicit skip).

**Empirical verification (MuJoCo 3.5.0):**
```
Two spheres: g1 at (0,0,0) r=0.1, g2 at (1,0,0) r=0.2
  distance cutoff=10:  0.700000 ✓ (1.0 - 0.1 - 0.2)
  normal cutoff=10:    [1, 0, 0] ✓ (g1→g2 direction)
  fromto cutoff=10:    [0.1, 0, 0, 0.8, 0, 0] ✓ (surface points)

Overlapping spheres: g1 at (0,0,0) r=0.5, g2 at (0.3,0,0) r=0.5
  distance cutoff=10:  -0.700000 ✓ (signed — negative = penetration)

Multi-geom body: b1 has g1a(0,0,0 r=0.1), g1b(0.3,0,0 r=0.1); b2 has g2(1,0,0 r=0.1)
  distance body1/body2 cutoff=10: 0.500000 ✓ (min of all pairs: g1b↔g2)

Cutoff=0 with non-overlapping geoms:
  distance cutoff=0:   0.0 (positive distances suppressed)
  normal cutoff=0:     [0, 0, 0] (no fromto computed)
  fromto cutoff=0:     [0, 0, 0, 0, 0, 0] (no fromto computed)

Cutoff=0 with OVERLAPPING (penetrating) geoms:
  distance cutoff=0:   -0.700000 (penetration still reported!)
  normal cutoff=0:     [-1, 0, 0] (computed from penetration)
  fromto cutoff=0:     [0.5, 0, 0, -0.2, 0, 0] (penetration contact points)

Cutoff clamping:
  distance cutoff=0.5: 0.5 (clamped to cutoff)

Self-distance:
  geom1==geom2: MuJoCo compiler rejects ("1st body/geom must be different from 2nd body/geom")

Body with zero geoms:
  body1 has 0 geoms, cutoff=10: 10.0 (loop body never executes, dist stays at cutoff init)

Coincident geoms (same position, different IDs, same size 0.5):
  distance cutoff=10:  -1.000000 (full penetration = -2*radius)
  normal cutoff=10:    [-1, 0, 0] (arbitrary direction from collision system)
  fromto cutoff=10:    [0.5, 0, 0, -0.5, 0, 0]

Negative distance + postprocess cutoff:
  actual dist=-0.7, cutoff=0.5: sensor=-0.500000 (postprocess clamps to [-cutoff, cutoff])
  → postprocess clamp(-0.5, 0.5) applies to GeomDist (REAL type)

Beyond-cutoff (geoms too far apart):
  g1 at (0,0,0) r=0.1, g2 at (10,0,0) r=0.1, cutoff=1:
  distance: 1.0 (clamped to cutoff)
  normal:   [0, 0, 0] (zeroed — no contact within cutoff)
  fromto:   [0, 0, 0, 0, 0, 0] (zeroed — no contact within cutoff)

FromTo ordering:
  ft(g1→g2): [0.1, 0, 0, 0.8, 0, 0] (from=g1 surface, to=g2 surface)
  ft(g2→g1): [0.8, 0, 0, 0.1, 0, 0] (from=g2 surface, to=g1 surface — swapped)

Penetrating fromto:
  g1 r=0.5 at (0,0,0), g2 r=0.5 at (0.3,0,0):
  fromto: [0.5, 0, 0, -0.2, 0, 0] (from point INSIDE g2, to point INSIDE g1)

Body-pair subtree test:
  b1 at (0,0,0) with direct geom g1_parent + child body b1_child at (1.5,0,0) with g1_child
  b2 at (2,0,0) with geom g2:
  d_body_pair = 1.8 (matches g1_parent↔g2 = 1.8, NOT g1_child↔g2 = 0.3)
  → body-pair uses DIRECT geoms only (body_geomnum), NOT subtree

Negative cutoff:
  cutoff=-1: REJECTED ("negative cutoff in sensor")

Noise:
  noise=0.1 on distance/normal/fromto: all runs return identical values
  → noise is stored in model but NOT applied during mj_forward

Object resolution validation:
  geom1 + body2: ACCEPTED (objtype=GEOM(5), reftype=BODY(1), distance=1.8)
  body1 + geom2: ACCEPTED (objtype=BODY(1), reftype=GEOM(5), distance=1.8)
  geom1 + body1 (both on same side): REJECTED ("exactly one of (geom1, body1) must be specified")
  no geom/body: REJECTED ("exactly one of (geom1, body1) must be specified")
  only geom1, no geom2/body2: REJECTED ("exactly one of (geom2, body2) must be specified")
  same-body geoms (g1+g2 on same body): ACCEPTED (distance=0.8)

Cutoff on non-geom sensors:
  Clock cutoff=5, timestep=1: clamped at 5.0 after t=5 ✓
  JointActuatorFrc cutoff=5, ctrl=10: sensor=5.0 (qfrc_actuator=10.0, clamped) ✓
  JointActuatorFrc cutoff=5, ctrl=-10: sensor=-5.0 (symmetric clamping) ✓

Geom type support:
  box-sphere: distance=1.2 ✓ (2.0 - 0.5 - 0.3)
  capsule-capsule: distance=0.8 ✓ (1.0 - 0.1 - 0.1)
  cylinder-sphere: distance=1.5 ✓ (2.0 - 0.3 - 0.2)
  plane-sphere: distance=0.9 ✓ (1.0 - 0.1)
  ellipsoid-sphere: distance=1.6 ✓ (2.0 - 0.3 - 0.1)
  rotated box-box: distance=0.793 ✓ (GJK/MPR, involves sqrt(2))
```

### EGT-4: `mj_geomDistance()` dependency

`mj_geomDistance()` is defined in MuJoCo's `engine_support.c`.
It computes signed distance between two geoms using the collision/SDF system.
CortenForge has GJK/EPA infrastructure in `sim/L0/core/src/gjk_epa.rs` and
collision shapes in `collision_shape.rs`, plus SDF support in `sdf.rs`. The
geom distance sensors need a Rust equivalent of `mj_geomDistance()` that:
1. Takes two geom IDs and a cutoff
2. Returns signed distance and fromto points
3. Uses the existing collision infrastructure

### EGT-5: CamProjection — deferred (no camera infrastructure)

`CamProjection` requires these Model/Data fields that don't exist:
- `MjObjectType::Camera` variant
- `Model::cam_xpos`, `cam_xmat`, `cam_resolution`, `cam_fovy`, `cam_intrinsic`, `cam_sensorsize`
- `Data::cam_xpos`, `Data::cam_xmat`

Building camera infrastructure is a separate feature. `CamProjection` is
deferred to DT-120 (post-Phase 6). This is conformance-neutral: camera
projection sensors are uncommon in RL/robotics models.

### EGT-6: Analytical verification data

Physics derivations for all empirical values in EGT-1 through EGT-3. These
provide independent mathematical confirmation that the MuJoCo-observed values
are correct.

**V1: Clock sensor value derivation**

Clock reads `data.time`. After `mj_forward()` at t=0, time has not been
incremented → sensor = 0.0. After N steps with `timestep = dt`:
- `mj_step` calls `mj_forward` (evaluates sensors at current time) then
  increments `d->time += dt`.
- After step 1: sensor was evaluated at t=0 → reads 0.0, then time becomes dt.
- After step 2: sensor was evaluated at t=dt → reads dt, then time becomes 2*dt.
- After step N: sensor reads (N-1)*dt, time is N*dt.
- **Formula:** `sensor_value_after_N_steps = (N-1) * timestep`.
- EGT-1 confirmation: `mj_forward(t=0)` → 0.0 ✓

**V2: JointActuatorFrc derivation (two actuators, same joint)**

`qfrc_actuator[dof]` accumulates actuator forces mapped to generalized
coordinates. For motor actuators with `dyntype=none` (no activation dynamics)
and `gaintype=fixed` (gain_prm[0]=1 default):
- `actuator_force[i] = gain * ctrl[i]` (for position-less muscle model this
  is just `ctrl` times `gain`, which is 1.0 for default gain)
- `qfrc_actuator[dof] = Σ_i (actuator_force[i] * gear[i])` for all actuators
  acting on the joint at `dof`.
- With ctrl=[3.0, 4.0], gear=[1.0, 2.0]:
  `qfrc_actuator = 3.0*1.0 + 4.0*2.0 = 3.0 + 8.0 = 11.0`
- EGT-2 confirmation: 11.0 ✓

**V3: GeomDist sphere-sphere derivation**

Signed distance between two spheres:
- `dist = |c2 - c1| - r1 - r2` where c1, c2 are centers, r1, r2 are radii.
- g1 at origin r=0.1, g2 at (1,0,0) r=0.2:
  `dist = 1.0 - 0.1 - 0.2 = 0.7`
- EGT-3 confirmation: 0.700000 ✓

**V4: GeomNormal sphere-sphere derivation**

Normal is the unit vector from nearest point on geom1 to nearest point on geom2:
- `normal = normalize(fromto[3:6] - fromto[0:3])`
- For spheres aligned along x-axis: `normal = normalize((0.8-0.1, 0-0, 0-0)) = normalize((0.7, 0, 0)) = (1, 0, 0)`
- EGT-3 confirmation: [1, 0, 0] ✓

**V5: GeomFromTo sphere-sphere derivation**

FromTo returns 6 values: [from_x, from_y, from_z, to_x, to_y, to_z].
- "from" = closest point on geom1 surface to geom2: `c1 + r1 * normal = (0,0,0) + 0.1 * (1,0,0) = (0.1, 0, 0)`
- "to" = closest point on geom2 surface to geom1: `c2 - r2 * normal = (1,0,0) - 0.2 * (1,0,0) = (0.8, 0, 0)`
- `fromto = [0.1, 0, 0, 0.8, 0, 0]`
- EGT-3 confirmation: [0.1, 0, 0, 0.8, 0, 0] ✓

**V6: Penetration distance derivation**

For overlapping spheres:
- g1 at origin r=0.5, g2 at (0.3,0,0) r=0.5:
  `dist = |0.3 - 0| - 0.5 - 0.5 = 0.3 - 1.0 = -0.7`
- Negative = penetration depth.
- EGT-3 confirmation: -0.700000 ✓

**V7: Cutoff=0 suppression derivation**

The C code initializes `dist = cutoff`. For cutoff=0, dist starts at 0.
- `mj_geomDistance()` returns `min(actual_dist, cutoff)` for non-penetrating
  geoms. With cutoff=0 and actual_dist=0.7: returns 0.0.
- The `if (dist_new < dist)` guard: `0.0 < 0` is false → dist stays 0. ✓
- Penetrating: actual_dist=-0.7 < 0 → `mj_geomDistance()` returns -0.7
  (exact distance for penetration). `if (-0.7 < 0)` is true → dist updates. ✓
- EGT-3 confirmation: non-overlapping cutoff=0 → 0.0 ✓; overlapping cutoff=0 → -0.7 ✓

**V8: Multi-geom body distance derivation**

b1 has g1a(0,0,0 r=0.1) and g1b(0.3,0,0 r=0.1); b2 has g2(1,0,0 r=0.1). Cutoff=10 (both distances well below cutoff, so `mj_geomDistance()` returns exact values).
- g1a↔g2: `|1.0 - 0| - 0.1 - 0.1 = 0.8`
- g1b↔g2: `|1.0 - 0.3| - 0.1 - 0.1 = 0.7 - 0.2 = 0.5`
- Body-pair distance = `min(0.8, 0.5) = 0.5` ✓
- EGT-3 confirmation: 0.500000 ✓

**V9: Penetrating fromto and normal derivation**

g1 at origin r=0.5, g2 at (0.3,0,0) r=0.5:
- Center-to-center direction: `normalize((0.3,0,0)) = (1,0,0)`
- from (g1 surface toward g2): `c1 + r1 * dir = (0,0,0) + 0.5*(1,0,0) = (0.5, 0, 0)`
- to (g2 surface toward g1): `c2 - r2 * dir = (0.3,0,0) - 0.5*(1,0,0) = (-0.2, 0, 0)`
- fromto = `[0.5, 0, 0, -0.2, 0, 0]` ✓ — "from" is inside g2, "to" is inside g1
- Normal: `normalize(to - from) = normalize((-0.2 - 0.5, 0, 0)) = normalize((-0.7, 0, 0)) = (-1, 0, 0)`
- The sign flip (negative x) is correct: when penetrating, the surface-to-surface vector reverses relative to center-to-center.
- EGT-3 confirmation: fromto=[0.5, 0, 0, -0.2, 0, 0] ✓; normal=[-1, 0, 0] ✓

**V10: Postprocess cutoff clamping derivation**

Actual distance = -0.7 (penetrating), cutoff = 0.5:
- `apply_cutoff()` runs because cutoff > 0
- GeomDist has `mjDATATYPE_REAL` (0) → enters the `clamp` branch
- `clamp(-0.7, -cutoff, cutoff) = clamp(-0.7, -0.5, 0.5) = -0.5`
- EGT-3 confirmation: -0.500000 ✓

**V11: Positive-distance cutoff clamping derivation**

Two spheres separated by 0.7, cutoff = 0.5:
- `mj_geomDistance()` returns `min(0.7, 0.5) = 0.5` (caps at cutoff when
  `actual_dist >= cutoff`).
- Eval-stage: `dist = cutoff = 0.5`. `dist_new = 0.5`. `0.5 < 0.5` is false
  → dist stays 0.5.
- Postprocess: `clamp(0.5, -0.5, 0.5) = 0.5` (already within range, no change).
- Result: 0.5 ✓
- EGT-3 confirmation: 0.5 ✓

### EGT-7: CortenForge field-name mapping

Exact CortenForge field names and locations for all fields referenced in
the MuJoCo C source excerpts above.

**Sensor-specific read fields:**

| MuJoCo C field | CortenForge field | File | Line | Type |
|---------------|-------------------|------|------|------|
| `d->time` | `data.time` | `core/src/types/data.rs` | 499 | `f64` |
| `d->qfrc_actuator[...]` | `data.qfrc_actuator[...]` | `core/src/types/data.rs` | 51 | `DVector<f64>` |
| `m->jnt_dofadr[...]` | `model.jnt_dof_adr[...]` | `core/src/types/model.rs` | 171 | `Vec<usize>` |
| `m->body_geomadr[...]` | `model.body_geom_adr[...]` | `core/src/types/model.rs` | 123 | `Vec<usize>` |
| `m->body_geomnum[...]` | `model.body_geom_num[...]` | `core/src/types/model.rs` | 125 | `Vec<usize>` |
| `m->geom_bodyid[...]` | `model.geom_body[...]` | `core/src/types/model.rs` | 253 | `Vec<usize>` |
| `m->sensor_cutoff[i]` | `model.sensor_cutoff[sensor_id]` | `core/src/types/model.rs` | 514 | `Vec<f64>` |
| `m->sensor_objtype[i]` | `model.sensor_objtype[sensor_id]` | `core/src/types/model.rs` | 500 | `Vec<MjObjectType>` |
| `m->sensor_objid[i]` | `model.sensor_objid[sensor_id]` | `core/src/types/model.rs` | 502 | `Vec<usize>` |
| `m->sensor_reftype[i]` | `model.sensor_reftype[sensor_id]` | `core/src/types/model.rs` | 504 | `Vec<MjObjectType>` |
| `m->sensor_refid[i]` | `model.sensor_refid[sensor_id]` | `core/src/types/model.rs` | 506 | `Vec<usize>` |

**Infrastructure fields (used by ALL sensor evaluation, not sensor-type-specific):**

| MuJoCo C field | CortenForge field | File | Line | Type |
|---------------|-------------------|------|------|------|
| `m->nsensor` | `model.nsensor` | `core/src/types/model.rs` | 492 | `usize` |
| `m->nsensordata` | `model.nsensordata` | `core/src/types/model.rs` | 494 | `usize` |
| `m->sensor_type[i]` | `model.sensor_type[sensor_id]` | `core/src/types/model.rs` | 496 | `Vec<MjSensorType>` |
| `m->sensor_datatype[i]` | `model.sensor_datatype[sensor_id]` | `core/src/types/model.rs` | 498 | `Vec<MjSensorDataType>` * |
| `m->sensor_adr[i]` | `model.sensor_adr[sensor_id]` | `core/src/types/model.rs` | 508 | `Vec<usize>` |
| `m->sensor_dim[i]` | `model.sensor_dim[sensor_id]` | `core/src/types/model.rs` | 510 | `Vec<usize>` |
| `m->sensor_noise[i]` | `model.sensor_noise[sensor_id]` | `core/src/types/model.rs` | 512 | `Vec<f64>` |
| `d->sensordata` | `data.sensordata` | `core/src/types/data.rs` | 398 | `DVector<f64>` |

*`sensor_datatype` stores pipeline stage (`Position`/`Velocity`/`Acceleration`),
NOT MuJoCo's data kind (`REAL`/`POSITIVE`/`AXIS`/`QUATERNION`). See EGT-3
architectural note.

**Critical naming differences:** CortenForge uses snake_case with underscores
between words (`jnt_dof_adr`, `body_geom_adr`, `body_geom_num`) whereas MuJoCo
concatenates (`jnt_dofadr`, `body_geomadr`, `body_geomnum`). The spec MUST use
CortenForge names, not MuJoCo names, in all Rust pseudocode and algorithm
sections.

### EGT-8: Exhaustive match-site inventory

Every code location that must be modified when adding new `MjSensorType` or
`MjcfSensorType` variants. Organized by crate and exhaustiveness.

**Table 1: sim-core enums (compiler-enforced)**

| File | Line range | Function/Match | Exhaustive? | Required change |
|------|-----------|----------------|-------------|-----------------|
| `core/src/types/enums.rs` | 342–421 | `MjSensorType` enum definition | N/A (enum) | Add 5 variants: `Clock`, `JointActuatorFrc`, `GeomDist`, `GeomNormal`, `GeomFromTo` before `User` |
| `core/src/types/enums.rs` | 427–462 | `MjSensorType::dim()` | Yes (exhaustive match) | Add 5 arms: Clock→1, JointActuatorFrc→1, GeomDist→1, GeomNormal→3, GeomFromTo→6 |

**Table 2: sim-mjcf parser & builder (mixed)**

| File | Line range | Function/Match | Exhaustive? | Required change |
|------|-----------|----------------|-------------|-----------------|
| `mjcf/src/types.rs` | 2863–2938 | `MjcfSensorType` enum definition | N/A (enum) | Add 5 variants: `Clock`, `Jointactuatorfrc`, `Distance`, `Normal`, `Fromto` |
| `mjcf/src/types.rs` | 2942–2977 | `MjcfSensorType::from_str()` | **No** (wildcard `_ => None`) | Add 5 arms: `"clock"`, `"jointactuatorfrc"`, `"distance"`, `"normal"`, `"fromto"` — **compiler will NOT catch omissions** |
| `mjcf/src/types.rs` | 2982–3016 | `MjcfSensorType::as_str()` | Yes (exhaustive) | Add 5 arms returning `"clock"`, `"jointactuatorfrc"`, `"distance"`, `"normal"`, `"fromto"` |
| `mjcf/src/types.rs` | 3022–3060 | `MjcfSensorType::dim()` | Yes (exhaustive) | Add 5 arms: Clock→1, Jointactuatorfrc→1, Distance→1, Normal→3, Fromto→6 |
| `mjcf/src/builder/sensor.rs` | 93–252 | `resolve_sensor_object()` | Yes (exhaustive on `MjSensorType`) | Add arms for all 5 types. Clock→`(None, 0)` (early return before objname check, like User). JointActuatorFrc→joint lookup (with hinge/slide validation — MuJoCo rejects ball/free). GeomDist/Normal/FromTo→dual-object resolution (NEW pattern). |
| `mjcf/src/builder/sensor.rs` | 366–400 | `convert_sensor_type()` | Yes (exhaustive on `MjcfSensorType`) | Add 5 mappings. |
| `mjcf/src/builder/sensor.rs` | 404–444 | `sensor_datatype()` | Yes (exhaustive on `MjSensorType`) | Add 5 arms: Clock→Position, JointActuatorFrc→Acceleration, GeomDist/Normal/FromTo→Position. |
| `mjcf/src/types.rs` | 3069–3091 | `MjcfSensor` struct definition | N/A (struct) | Add 4 fields: `geom1: Option<String>`, `geom2: Option<String>`, `body1: Option<String>`, `body2: Option<String>`. Required for Distance/Normal/Fromto dual-object attributes. |
| `mjcf/src/types.rs` | 3093 | `MjcfSensor` `Default` impl | N/A (impl) | Add default values for the 4 new fields (`None` for all). Must stay in sync with struct definition above. |
| `mjcf/src/parser.rs` | 3453–3493 | `parse_sensor_attrs()` | N/A (no match) | Add conditional parsing of `geom1`/`geom2`/`body1`/`body2` attributes when `sensor_type` is `Distance \| Normal \| Fromto`. Must also validate strict XOR (exactly one of geom1/body1, exactly one of geom2/body2). Current `objname` or_else chain (line 3463) does NOT handle these attributes — they use different attribute names (`geom1` not `geom`, `body1` not `body`). |

**Table 3: sim-core sensor evaluation (wildcard — DANGER)**

| File | Line range | Function/Match | Exhaustive? | Required change |
|------|-----------|----------------|-------------|-----------------|
| `core/src/sensor/position.rs` | 99–423 | `mj_sensor_pos()` main match | **No** (`_ => {}` wildcard) | Add 2 arms: (1) Clock (`sensor_write(sensordata, adr, 0, data.time)`), (2) `GeomDist \| GeomNormal \| GeomFromTo` combined arm with internal type dispatch (mirrors MuJoCo's fall-through case block and the existing `FrameXAxis \| FrameYAxis \| FrameZAxis` pattern at line 213). Shared computation: body-vs-geom dispatch, all-pairs loop, `geom_distance()` call. Per-type output: GeomDist→scalar, GeomNormal→normalize fromto diff, GeomFromTo→copy fromto. **Compiler will NOT catch omission — sensor silently produces 0.** |
| `core/src/sensor/velocity.rs` | 77–316 | `mj_sensor_vel()` main match | **No** (`_ => {}` wildcard) | No change needed (none of the 5 new types are velocity-stage). |
| `core/src/sensor/acceleration.rs` | 64–78 | Lazy gate match | **No** (`_ => {}` wildcard) | No change needed — JointActuatorFrc reads `qfrc_actuator` directly, does NOT need body accumulators. Falls through to `_ => {}` correctly. |
| `core/src/sensor/acceleration.rs` | 81–278 | `mj_sensor_acc()` main match | **No** (`_ => {}` wildcard) | Add 1 arm: JointActuatorFrc. **Compiler will NOT catch omission — sensor silently produces 0.** |

**Table 4: sim-core postprocess**

| File | Line range | Function/Match | Exhaustive? | Required change |
|------|-----------|----------------|-------------|-----------------|
| `core/src/sensor/postprocess.rs` | 55–68 | Cutoff section in `mj_sensor_postprocess()` | **No** (`_ =>` wildcard at line 66) | Add skip guard for `GeomFromTo` and `GeomNormal` **before** the per-element loop (between lines 55–57), matching MuJoCo's early-return pattern. Both need explicit type match since CortenForge's `MjSensorDataType` stores stage not data kind. See AD-3 option (a). |

**Table 5: sim-mjcf compiler (wildcard — DANGER)**

| File | Line range | Function/Match | Exhaustive? | Required change |
|------|-----------|----------------|-------------|-----------------|
| `mjcf/src/builder/compiler.rs` | 85–103 | `apply_fusestatic()` sensor body protection | **No** (`_ => {}` wildcard) | Add `Distance \| Normal \| Fromto` arms. **Important:** existing arms read `sensor.objname` (line 98), but geom distance sensors store body names in `body1`/`body2` fields — NOT `objname`. The `MjcfSensor` struct must first be extended with these fields (AD-4/R26 prerequisite). The fusestatic arms must read `sensor.body1` and `sensor.body2` (the new fields), not `sensor.objname`. Only `body1`/`body2` need protection (geoms are not fused). If only `geom1`+`geom2` are specified (no body refs), no fusestatic protection needed. **Compiler will NOT catch omission — sensor silently breaks at load time.** |

**Summary:** 18 match sites across 9 files. 6 are compiler-enforced
(exhaustive matches). 2 are enum definitions (compiler-enforced by
construction). 7 use wildcards — these are the danger zones where
missing arms produce silent zero-output or load-time bugs. 3 are
struct/function definitions (N/A for exhaustiveness — require manual
field additions and parse logic).

### EGT-9: Architectural decisions

Decisions the spec must make explicitly. Each lists enumerated options with
trade-offs.

**AD-1: `geom_distance()` implementation approach**

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Implement in Spec C** | Full `geom_distance()` using GJK/EPA from `gjk_epa.rs`. | Complete but large scope — may delay Spec C. GJK/EPA gives unsigned distance; signed distance requires additional penetration depth logic. |
| **(b) Stub + interface contract** | Define the `fn geom_distance(model, data, geom1, geom2, cutoff) -> (f64, [f64; 6])` signature and behavior contract. Implementation deferred to a separate task. Sensor evaluation calls the stub. | Unblocks sensor implementation but sensors produce placeholder output until stub is filled. |
| **(c) Thin wrapper over existing collision** | Use existing `gjk_epa.rs` + `collision_shape.rs` to compose the function. No new collision algorithms. | Pragmatic if existing infrastructure is sufficient. Must verify sphere-sphere, box-sphere, etc. coverage. **Caveat:** existing GJK/EPA provides intersection testing and penetration depth (EPA), but does NOT provide separation distance for non-overlapping shapes or closest surface points. Option (c) requires extending `gjk_epa.rs` with a GJK closest-point query, or implementing analytic distance functions per shape pair. |

**Recommended:** (c) if existing GJK/EPA covers needed geom pairs, (b) otherwise.
**Important infrastructure gap:** `gjk_epa.rs` currently supports intersection + penetration depth only. Unsigned separation distance (the GJK closest-point query for non-overlapping shapes) and closest surface point computation are NOT implemented. The spec must address this gap explicitly — either by extending `gjk_epa.rs` or by using analytic formulas for common shape pairs.

**AD-2: `sensor_write` helpers for 6D output**

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Add `sensor_write6()`** | New helper in `postprocess.rs` alongside `sensor_write3()` and `sensor_write4()`. Takes `adr` and 6 values. | Clean API symmetry. Used only by GeomFromTo (1 caller). |
| **(b) Manual 6x `sensor_write()` calls** | Six individual `sensor_write(sensordata, adr, i, val)` calls in the GeomFromTo arm. | No new code in postprocess.rs. Slightly verbose but explicit. |
| **(c) Slice-based `sensor_write_n()`** | Generic `sensor_write_n(sensordata, adr, &[f64])` that writes N values. | Replaces all `sensor_write3/4/6` with one function. Larger refactor scope. |

**Recommended:** (a) — matches existing convention (`sensor_write`, `sensor_write3`, `sensor_write4` already exist at `postprocess.rs:12,21,29`).

**AD-3: GeomFromTo + GeomNormal postprocess cutoff exemption**

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Explicit type match** | Add a skip guard for `GeomFromTo` and `GeomNormal` **before** the per-element loop in `postprocess.rs` (between lines 55–57), matching MuJoCo's early-return pattern (`if (type == mjSENS_GEOMFROMTO) return;` which exits before the `for j` loop). Use `if matches!(sensor_type, GeomFromTo \| GeomNormal) { continue; }` or equivalent. Do NOT add inside the match at line 60, which is for choosing clamping strategy. | Most readable. MuJoCo exempts GeomFromTo by type, GeomNormal implicitly via AXIS datatype (only REAL/POSITIVE get clamped). |
| **(b) Datatype-based exemption** | Add a data-kind enum (`MjSensorDataKind`: Real/Positive/Axis/Quaternion) separate from the pipeline-stage `MjSensorDataType`. Skip cutoff for Axis/Quaternion kinds. GeomFromTo still needs explicit exemption (REAL datatype). | Architecturally mirrors MuJoCo. Large scope increase — adds a new enum and populates it for all sensors. |
| **(c) Hybrid** | GeomNormal exempted by explicit type match. GeomFromTo exempted by explicit type match. Both in a single match arm. | Simplest. Both exemptions use the same mechanism. No new types needed. |

**Recommended:** (a) — CortenForge's `MjSensorDataType` stores pipeline stage
(`Position`/`Velocity`/`Acceleration`), NOT data kind. Adding a separate data-kind
enum (option b) is architecturally clean but overkill for two exemptions. Explicit
type matching achieves identical behavior with zero new infrastructure. GeomNormal's
AXIS-based implicit skip in MuJoCo is an implementation detail; the *effect* is
"no cutoff clamping", which an explicit match achieves identically.

**AD-4: Dual-object resolution in `resolve_sensor_object()`**

| Option | Description | Trade-off |
|--------|-------------|-----------|
| **(a) Expand `resolve_sensor_object()` to return `(objtype, objid, reftype, refid)`** | Change return type for dual-object sensors. Other sensors return `(objtype, objid, None, -1)`. | Requires changing the function signature, affecting all callers. |
| **(b) Separate `resolve_sensor_ref_object()` function** | Leave `resolve_sensor_object()` unchanged. Add a parallel function for reftype/refid resolution. Builder calls both for geom sensors. | No signature change. Two lookups in builder for dual-object types. |
| **(c) Handle in parser/builder layer** | Parse `geom1`/`body1`/`geom2`/`body2` attributes in the sensor parser and store as separate fields in `MjcfSensor`. Builder maps both to `(objtype, objid)` and `(reftype, refid)`. | Cleanest separation: parser handles MJCF attributes, builder maps to Model arrays. `resolve_sensor_object()` only handles obj side. |

**Recommended:** (c) — the dual-object pattern is fundamentally a parser concern
(4 MJCF attributes → 2 object references). The builder already stores
`sensor_reftype`/`sensor_refid` arrays (`model.rs:504–506`) and has
`resolve_reference_object()` (`builder/sensor.rs:277–361`) — just populate them.
**Note:** `MjcfSensor` (at `types.rs:3069–3091`) currently has `objname`/`objtype`
and `reftype`/`refname` fields, but does NOT have `geom1`/`geom2`/`body1`/`body2`
fields. The parser must add these fields. The builder must then map:
`geom1`/`body1` → `objname`+`objtype` (resolved via `resolve_sensor_object()`)
and `geom2`/`body2` → `refname`+`reftype` (resolved via `resolve_reference_object()`
at `builder/sensor.rs:277–361`, which already handles `"body"` and `"geom"` strings).

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on every other criterion and still be worthless if P1 is
> wrong — because an incorrect MuJoCo reference means every algorithm, every
> AC, and every test is verifying the wrong behavior. **Grade P1 first and
> grade it hardest.** If P1 is not A+, do not proceed to grading other
> criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right. An error here propagates to every
> algorithm, every AC, and every test.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file, line range, and exact behavior. For **Clock**: `mj_computeSensorPos()` case `mjSENS_CLOCK`, `d->time` read, position-stage assignment, objtype=UNKNOWN, objid=-1. For **JointActuatorFrc**: `mj_computeSensorAcc()` case `mjSENS_JOINTACTFRC`, `d->qfrc_actuator[m->jnt_dofadr[objid]]` formula, difference from `ActuatorFrc` (`actuator_force[actuator_id]` — single actuator — vs. `qfrc_actuator[dof_adr]` — net force at joint DOF). For **GeomDist/GeomNormal/GeomFromTo**: shared case block in `mj_computeSensorPos()`, `mj_geomDistance()` call, body-vs-geom objtype/reftype dispatch, cutoff=0 ⇒ returns 0 behavior, fromto→normal normalization, zero-vector guard (`if (normal[0] \|\| normal[1] \|\| normal[2])`). Edge cases addressed: cutoff=0 default semantics (**cutoff=0 suppresses positive distances but NOT penetration** — overlapping geoms still report negative dist, normal, and fromto), signed negative distance for penetration, postprocess clamps GeomDist to `[-cutoff, cutoff]` (penetration -0.7 with cutoff=0.5 → -0.5), zero-length normal vector (non-overlapping with cutoff=0 → output stays zero), `GeomFromTo` cutoff exemption in postprocess (`if (type == mjSENS_GEOMFROMTO) return;`), `GeomNormal` cutoff exemption (via explicit type match — MuJoCo uses AXIS datatype to skip, but CortenForge lacks a datatype enum; see EGT-3 architectural note), body multi-geom iteration (`body_geom_num`/`body_geom_adr` dispatch — **direct geoms only**, NOT subtree), body with zero geoms (returns cutoff — loop never executes), self-distance rejected by MuJoCo compiler (`1st body/geom must be different from 2nd`), coincident geoms (full penetration = -2r), beyond-cutoff asymmetry (distance→cutoff value, normal→`[0,0,0]`, fromto→`[0,0,0,0,0,0]`), fromto output ordering (from=geom1 surface, to=geom2 surface; swapping geom order swaps from/to), negative cutoff rejected by compiler (`negative cutoff in sensor`), noise is metadata-only (not applied during `mj_forward`), strict XOR validation (`exactly one of (geom1, body1)` and `exactly one of (geom2, body2)` — specifying both or neither is a parse error), mixed objtype/reftype supported (geom1+body2 and body1+geom2 both valid), same-body geoms allowed (two geoms on same body can be distance-measured), cutoff is universal (Clock/JointActuatorFrc also get postprocess clamping — cutoff=5 on Clock clamps at 5.0), `mj_geomDistance()` supports all geom types (sphere, box, capsule, cylinder, ellipsoid, plane — verified empirically). MJCF element names stated (`distance`/`normal`/`fromto` — NOT `geomdist`/`geomnormal`/`geomfromto`). **JointActuatorFrc** restricted to hinge/slide joints by MuJoCo compiler (`joint must be slide or hinge in sensor`). |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage (e.g., missing the cutoff=0 ⇒ 0 behavior). |
| **B** | Correct at high level, but missing specifics — e.g., "uses collision distance" without specifying `mj_geomDistance()` parameters, or element name mismatch. |
| **C** | Partially correct. Cutoff semantics wrong or geom/body dispatch missing. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable. The algorithm must
> reproduce MuJoCo's behavior exactly.

| Grade | Bar |
|-------|-----|
| **A+** | For **Clock**: trivial — `sensor_write(sensordata, adr, 0, data.time)` (reads `data.time` at `data.rs:499`). For **JointActuatorFrc**: `sensor_write(sensordata, adr, 0, data.qfrc_actuator[model.jnt_dof_adr[objid]])` — index chain fully specified with CortenForge field names (`jnt_dof_adr` at `model.rs:171`, `qfrc_actuator` at `data.rs:51`). For **GeomDist/Normal/FromTo**: complete Rust pseudocode for body-vs-geom dispatch using `model.body_geom_num[bodyid]` (`model.rs:125`) and `model.body_geom_adr[bodyid]` (`model.rs:123`), all-pairs loop, `geom_distance()` call with signature specified, distance→normal conversion (`fromto[3..6] - fromto[0..3]`, normalize, zero guard), fromto direct copy. The `geom_distance()` helper is either (a) specified with full algorithm or (b) scoped as a prerequisite with explicit interface contract (see AD-1). No "verify against source" notes. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but `geom_distance()` internals hand-waved or deferred without interface contract. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo (reference points, SpatialVector layout, field naming, etc.) and
> provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present covering: (1) MJCF element names (`"distance"` → `MjcfSensorType::Distance`, `"normal"` → `Normal`, `"fromto"` → `Fromto`, `"clock"` → `Clock`, `"jointactuatorfrc"` → `Jointactuatorfrc`); (2) `MjSensorType` variant naming (`GeomDist`, `GeomNormal`, `GeomFromTo`, `Clock`, `JointActuatorFrc`); (3) `MjObjectType` mappings: `MjObjectType::None` ↔ `mjOBJ_UNKNOWN` (0) — naming differs; `MjObjectType::Body` ↔ `mjOBJ_BODY`, `MjObjectType::Geom` ↔ `mjOBJ_GEOM`; (4) **CortenForge field names differ from MuJoCo**: `body_geom_num` (not `body_geomnum`, `model.rs:125`), `body_geom_adr` (not `body_geomadr`, `model.rs:123`), `jnt_dof_adr` (not `jnt_dofadr`, `model.rs:171`); (5) `MjSensorDataType` mapping: CortenForge uses `Position`/`Velocity`/`Acceleration` vs MuJoCo's `mjSTAGE_POS`/`VEL`/`ACC`; (6) `qfrc_actuator` field in `data.rs:51` (`DVector<f64>`); (7) `data.time` at `data.rs:499` (`f64`); (8) sensor_write helpers for 1D/3D/6D output (`postprocess.rs:12,21,29`, new `sensor_write6` needed — see AD-2); (9) **nalgebra types**: existing sensor code uses `nalgebra::Vector3<f64>` (not raw `[f64; 3]`) — `sensor_write3` takes `&Vector3<f64>`, proposed `sensor_write6` and `geom_distance()` return type must specify nalgebra vs raw array convention. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly."

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete input model/state, (2) exact expected value or tolerance — ideally derived from MuJoCo's output, (3) what field/sensor to check. For **Clock**: AC with `timestep=0.01`, 3 steps, expected sensordata values per step (from EGT-1; analytical: V1). For **JointActuatorFrc**: AC with two actuators on same joint, `ctrl=[3,4]`, `gear=[1,2]`, expected=11.0 (from EGT-2; analytical: V2) — this AC implicitly validates correct acceleration-stage placement since `qfrc_actuator` is only populated during the acceleration pipeline; incorrect position-stage placement would read zero-initialized data. For **GeomDist**: AC with two spheres, cutoff=10, expected=0.7 (V3); AC with cutoff=0, expected=0.0 (non-overlapping); AC with overlapping geoms, expected=-0.7 (V6). For **GeomNormal**: AC with sphere pair, expected=[1,0,0] (V4). For **GeomFromTo**: AC with sphere pair, expected=[0.1,0,0,0.8,0,0] (V5). Edge-case ACs with analytical backing from V7–V11: cutoff=0 suppression (V7), multi-geom body distance (V8), penetrating normal/fromto (V9), postprocess clamping (V10), positive-distance cutoff capping (V11). Code-review ACs explicitly labeled. Stage-correctness validation: at least one AC per sensor type must produce a non-trivial expected value that would be 0.0 if evaluated in the wrong pipeline stage (wildcard catch-all). |
| **A** | ACs are testable. Some lack MuJoCo-verified values. |
| **B** | ACs are directionally correct but vague ("should return distance"). |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test. Negative cases (feature NOT triggered) are tested.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory covering: cutoff=0 with non-penetrating geoms (sensor returns 0), cutoff=0 with penetrating geoms (**sensor still returns negative distance, normal, and fromto**), signed negative distance (penetration), postprocess clamps GeomDist to `[-cutoff, cutoff]` (e.g., -0.7 with cutoff=0.5 → -0.5), zero-length normal vector (non-overlapping with cutoff=0 → zero), coincident geoms (distance = -2r, arbitrary normal), body multi-geom iteration (N×M pairs), body with zero geoms (returns cutoff init value), self-distance rejection by compiler, `GeomFromTo` cutoff exemption in postprocess (verified: values not clamped), `GeomNormal` cutoff exemption via explicit type match, clock sensor reads `data.time`, `JointActuatorFrc` with no actuators on joint (expect 0), `JointActuatorFrc` with multiple actuators (net force), `JointActuatorFrc` parser rejects ball/free joints, negative cutoff rejected by compiler. Beyond-cutoff asymmetry: distance returns cutoff, normal returns `[0,0,0]`, fromto returns `[0,0,0,0,0,0]`. FromTo ordering: from=geom1 surface, to=geom2 surface (swap geom order → swap from/to). Body-pair uses direct geoms only (not subtree). Mixed objtype/reftype (geom1+body2 and body1+geom2). Same-body geoms (two geoms on same body). Strict XOR validation (both or neither on one side → parse error). Cutoff is universal (Clock with cutoff=5 clamps at 5.0, JointActuatorFrc with cutoff=5 clamps to [-5,5]). Noise is metadata-only (not applied during `mj_forward` — verify determinism). All geom types supported (box, capsule, cylinder, ellipsoid, plane — not just spheres). Clock multi-step time progression: after N steps, sensor reads `(N-1)*timestep` (V1 formula verifies one-timestep-lag). Penetrating normal sign flip: normal reverses relative to center-to-center direction when geoms overlap (V9). Penetrating fromto: surface points go inside opposite geom volume (V9). Negative cases: sensor disabled (`mjDSBL_SENSOR`), sleeping body sensor skip (infrastructure-level behaviors inherited from existing sensor framework). At least one MuJoCo conformance test per sensor type with MuJoCo-derived expected value (from EGT-1/2/3 with analytical backing V1–V11). Parser tests for new element names (`distance`, `normal`, `fromto`, `clock`, `jointactuatorfrc`). |
| **A** | Good coverage. Minor edge-case gaps. Conformance tests present. |
| **B** | Happy path covered. Edge cases sparse. No conformance tests. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated. No circular dependencies or implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Each section states what it requires from prior sections/specs. Dependencies on Spec A (objtype parsing infrastructure) called out with commit hash. `geom_distance()` prerequisite clearly stated: either (a) implemented as part of this spec or (b) scoped as a prerequisite with a stub/interface (see AD-1). If AD-1 option (c) is chosen, spec must verify that existing GJK/EPA infrastructure (`gjk_epa.rs`) supports unsigned separation distance (not just intersection/penetration) and closest-point queries, or specify what additions are needed. Parser→builder→evaluation ordering explicit. Cross-spec interactions with Spec B (reftype — these sensors don't use reftype in the frame-sensor sense, but `GeomDist`/`Normal`/`FromTo` use reftype/refid for the second geom/body) documented. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break. Surprises are spec bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description matching EGT-8 inventory. Files include: `enums.rs` (5 new `MjSensorType` variants + `dim()` arms, `enums.rs:342–462`), `types.rs` (5 new `MjcfSensorType` variants + `from_str()`/`as_str()`/`dim()` arms, `types.rs:2863–3060`), parser (new element names in sensor dispatch — note `from_str()` at `types.rs:2942` uses wildcard, **compiler won't catch missing arms**), `builder/sensor.rs` (`convert_sensor_type()` at line 366, `sensor_datatype()` at line 404, `resolve_sensor_object()` at line 93 — all exhaustive, compiler enforces), `position.rs` (2 new evaluation arms in `mj_sensor_pos()` at line 99 — **wildcard match, compiler won't catch**), `acceleration.rs` (1 new arm: JointActuatorFrc in `mj_sensor_acc()` at line 81 — **wildcard match, compiler won't catch**), `postprocess.rs` (GeomFromTo + GeomNormal cutoff exemption — skip guard between lines 55–57, before the per-element loop), `builder/compiler.rs` (`apply_fusestatic()` at line 85 — **wildcard match, must add body1/body2 protection for geom distance sensors**). Staleness guards: 6 exhaustive matches (compiler enforced) + 2 enum definitions + 7 wildcard matches (manual audit required) + 3 struct/function definitions (manual field additions). Existing test impact: none expected — all new code paths, no modification of existing arms. Domain test baseline: 2,238+ tests. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. Section references are accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "GeomDist" everywhere (not mixing "distance sensor" and "geomdist sensor"). MJCF element names clearly distinguished from enum variant names. File paths match between Specification and Files Affected. AC numbers match between AC section and Traceability Matrix. Edge case lists consistent across MuJoCo Reference and Test Plan. Sensor type counts are consistent (5 new types everywhere). Dim values consistent (1/1/1/3/6). Stage assignments consistent (4 position + 1 acceleration). **V-derivation consistency:** AC expected values are consistent with V-derivation (V1–V11) expected values — every AC that cites a specific numeric result has a matching V-derivation that derives the same result analytically. **Terminology hazards enumerated:** spec explicitly warns about MuJoCo/CortenForge naming confusions — `MjSensorDataType` (stage) vs `mjDATATYPE` (data kind), `MjObjectType::None` vs `mjOBJ_UNKNOWN`, CortenForge snake_case vs MuJoCo concatenated names. Cross-references EGT-8 match-site inventory for file/function consistency. |
| **A** | Consistent. One or two minor terminology inconsistencies. V-derivation values match ACs with at most one discrepancy. Terminology hazards mostly documented. |
| **B** | Some sections use different names for the same concept. V-derivation consistency not verified. No terminology hazard warnings. |
| **C** | Contradictions between sections. AC values conflict with V-derivations. |

### P9. Dual Object Resolution *(domain-specific)*

> GeomDist/GeomNormal/GeomFromTo sensors require resolution of *two* objects
> (obj and ref), each of which can be either a geom or a body. This is a new
> pattern — existing sensors resolve at most one object. The spec must handle
> this dual-object resolution correctly.

Boundary with P1: P1 grades whether the MuJoCo behavior is *described*
correctly. P9 grades whether the *dual-object resolution pattern* is
specified completely enough to implement without ambiguity — including
parser attribute handling (`geom1`/`geom2`/`body1`/`body2`), builder
resolution to `(objtype, objid, reftype, refid)`, and evaluation dispatch.

| Grade | Bar |
|-------|-----|
| **A+** | Parser specifies all 4 attributes (`geom1`, `geom2`, `body1`, `body2`) with **strict XOR validation**: exactly one of `{geom1, body1}` and exactly one of `{geom2, body2}` required — specifying both or neither on one side is a parse error. Builder resolution maps `geom1`→`(Geom, id)` and `body1`→`(Body, id)` for obj; `geom2`→`(Geom, id)` and `body2`→`(Body, id)` for ref. **Body/XBody distinction:** `body1`/`body2` resolve to `MjObjectType::Body` (NOT `MjObjectType::XBody`) — MuJoCo uses `mjOBJ_BODY` (not `mjOBJ_XBODY`) for geom distance sensors. This matters because `MjObjectType::Body` and `MjObjectType::XBody` appear as separate arms in existing sensor dispatch code (e.g., `resolve_sensor_object()`) and use different lookup paths — `Body` accesses `body_geom_adr`/`body_geom_num` for geom iteration while `XBody` would route to frame-position logic. Using the wrong type would silently hit the wrong dispatch arm. **All 4 combinations** valid: geom×geom (1×1), geom×body (1×N), body×geom (N×1), body×body (N×M) — mixed objtype/reftype is supported (e.g., `geom1` + `body2`). Body geom iteration uses **direct geoms only** (NOT subtree/child bodies) via `model.body_geom_num[bodyid]` (`model.rs:125`) and `model.body_geom_adr[bodyid]` (`model.rs:123`). Minimum-distance pair's normal and fromto are used for body-pair sensors. Tie-breaking: first-found pair wins (strict `<` comparison — iteration order is outer loop obj geoms, inner loop ref geoms). Architectural decision on dual-object resolution approach documented (see AD-4). |
| **A** | All combinations handled. Minor detail gaps. |
| **B** | Only geom×geom or body×body handled. Mixed cases missing. |
| **C** | Dual resolution not addressed. |

### P10. `geom_distance()` Interface Contract *(domain-specific)*

> The three geometry sensors depend on a `geom_distance()` function that
> doesn't yet exist. The spec must either fully specify this function or
> define a clear interface contract that another implementation can satisfy.

Boundary with P2: P2 grades whether the *sensor evaluation* algorithm is
complete. P10 grades whether the *geom distance helper* — the key
dependency — is adequately specified or contracted.

| Grade | Bar |
|-------|-----|
| **A+** | `geom_distance()` has a complete Rust signature (`fn geom_distance(model: &Model, data: &Data, geom1: usize, geom2: usize, cutoff: f64) -> (f64, [f64; 6])` or equivalent — **note:** `[f64; 6]` for fromto is acceptable here since `sensor_write6` does not yet exist; coordinate with P3 item (9) on whether to use nalgebra types for the return value). Behavior contract: returns `(signed_distance, fromto_points)` where `fromto_points = [from_x, from_y, from_z, to_x, to_y, to_z]` ("from" = nearest surface point on `geom1`, "to" = nearest surface point on `geom2`). **Return value semantics:** For penetrating geoms (`actual_dist < 0`), returns the exact signed distance and populated fromto. For non-penetrating geoms where `actual_dist < cutoff`, returns the exact distance and populated fromto. For non-penetrating geoms where `actual_dist >= cutoff`, returns `cutoff` and leaves fromto unpopulated (zeroed). Empirically verified: `mj_geomDistance()` with cutoff=0.5 and actual distance 0.7 returns 0.5 (not 0.7). The cutoff=0 suppression behavior (positive distances returning 0) is a consequence of this return-value-cap behavior combined with the caller's evaluation loop pattern (`dist = cutoff; if (dist_new < dist)`). Specifies which existing CortenForge collision primitives are used (GJK/EPA from `gjk_epa.rs`, collision shapes from `collision_shape.rs`). **Infrastructure gap cross-reference:** existing GJK/EPA lacks unsigned separation distance — see AD-1 important infrastructure gap note; spec must address this gap per P6 A+ bar. Handles sphere-sphere, box-sphere, etc. via existing `geom_to_collision_shape()`. Architectural decision on implementation approach documented (see AD-1). |
| **A** | Interface contract is clear. Implementation strategy identified. Minor details deferred. |
| **B** | Signature specified but behavior contract incomplete. |
| **C** | "Use collision system" without specifics. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`mj_computeSensorPos`, `mj_computeSensorAcc`, `mj_geomDistance`),
      specific fields with CortenForge names and file:line references
      (`data.qfrc_actuator` at `data.rs:51`, `model.jnt_dof_adr` at
      `model.rs:171`, `model.body_geom_num` at `model.rs:125`,
      `model.body_geom_adr` at `model.rs:123`, `data.time` at `data.rs:499`),
      specific edge cases (cutoff=0, signed negative distance, zero-length
      normal, body multi-geom, GeomFromTo cutoff exemption), and specific
      MJCF element names (`distance`/`normal`/`fromto`). Two reviewers would
      agree on grade by pointing to spec lines.

- [x] **Non-overlap:** P1 grades MuJoCo behavior description accuracy. P2
      grades algorithm implementability. P3 grades convention translation.
      P9 grades dual-object resolution pattern completeness. P10 grades
      `geom_distance()` interface contract. P1↔P9 boundary: P1 checks if
      MuJoCo's body/geom dispatch is *described* correctly; P9 checks if the
      *parser↔builder↔evaluation* chain for dual objects is fully specified.
      P2↔P10 boundary: P2 checks if the sensor evaluation loop is complete;
      P10 checks if the called helper has a sufficient contract.
      P4↔P8 boundary: P4 checks if each AC *cites* a specific value traceable
      to EGT or V-derivations; P8 checks if the cited values are *mutually
      consistent* across all sections (AC section, Test Plan, V-derivations).
      P3↔P8 boundary: P3 checks if convention differences are *documented*
      in the Convention Notes section; P8 checks if conventions are *applied
      consistently* across all sections.

- [x] **Completeness:** 10 criteria cover: MuJoCo fidelity (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), dual-object resolution (P9), geom
      distance contract (P10). The only dimension not covered is performance,
      which is explicitly out of scope for Phase 6.

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors. P2 →
      Specification sections. P3 → Convention Notes. P4 → Acceptance Criteria.
      P5 → Test Plan + Traceability Matrix. P6 → Prerequisites + Execution
      Order. P7 → Files Affected + Blast Radius (cross-ref with EGT-8). P8 →
      Cross-cutting. P9 → Parser + Builder + Evaluation sections for geom
      sensors. P10 → `geom_distance()` specification or interface contract
      section.

- [x] **Conformance primacy:** P1 is tailored with `mj_computeSensorPos`,
      `mj_computeSensorAcc`, `mj_geomDistance`, `mjSENS_CLOCK`,
      `mjSENS_JOINTACTFRC`, `mjSENS_GEOMDIST`/`GEOMNORMAL`/`GEOMFROMTO`.
      P4 requires MuJoCo-derived expected values (from EGT-1/2/3) with
      analytical backing (V1–V11 from EGT-6). P5 requires MuJoCo conformance
      tests. P10 requires collision behavior matching MuJoCo's
      `mj_geomDistance`.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Files Affected — cross-ref with EGT-8 inventory) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Parser section, Builder section, Evaluation section (geom sensors) |
| P10 | `geom_distance()` helper specification or interface contract |

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
| P9. Dual Object Resolution | | |
| P10. `geom_distance()` Contract | | |

**Overall:**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella listed `geompoint` — does not exist in MuJoCo 3.5.0 | Rubric self-audit (empirical verification) | Replaced with `geomfromto` (MJCF `fromto`). Documented in Scope Adjustment table. | Rev 1 |
| R2 | Scope | Umbrella listed `camprojection` — requires camera infrastructure | Rubric self-audit (codebase grep) | Deferred to DT-120 (not DT-118, which is contactForce). Documented in Scope Adjustment and EGT-5. | Rev 1 |
| R3 | P1 | MJCF element names differ from enum names: `distance`/`normal`/`fromto` | Rubric self-audit (empirical verification) | Added explicit element name mapping requirement to P1 A+ bar. | Rev 1 |
| R4 | P1 | GeomNormal uses `mjDATATYPE_AXIS` (2) despite being 3D — NOT `mjDATATYPE_QUATERNION` (3) as originally claimed. AXIS means only REAL/POSITIVE get cutoff clamping; AXIS implicitly skips. | Rubric self-audit (empirical verification), corrected Rev 6 | Documented in EGT-3, added to P1 A+ bar as cutoff exemption edge case. | Rev 1, **corrected Rev 6** |
| R5 | P1 | Cutoff=0 default ⇒ distance sensors return 0 (non-obvious). **Refined by R85:** cutoff=0 is the MuJoCo default when omitted; schema marks `required="true"` but runtime does not enforce. | Rubric self-audit (empirical verification) | Added as explicit edge case in P1 and P5 A+ bars. | Rev 1 |
| R6 | P9 | Dual-object resolution is a new pattern needing its own criterion | Rubric self-audit (completeness check) | Added P9 criterion for dual-object resolution. | Rev 1 |
| R7 | P10 | `geom_distance()` doesn't exist in CortenForge | Rubric self-audit (codebase grep) | Added P10 criterion for interface contract. | Rev 1 |
| R8 | P7 | `GeomFromTo` cutoff exemption in postprocess needs mention | Rubric self-audit (C source review) | Added to P7 A+ bar (postprocess.rs changes). | Rev 1 |
| R9 | P3 | CortenForge field names differ from MuJoCo: `jnt_dof_adr` not `jnt_dofadr`, `body_geom_adr` not `body_geomadr`, `body_geom_num` not `body_geomnum` | Quality review (codebase grep, `model.rs:171,123,125`) | Added EGT-7 field-mapping table. Updated P2, P3, P9 A+ bars to use correct CortenForge names with file:line refs. | Rev 2 |
| R10 | All | Scorecard was pre-filled with A+ — rubric grades the *spec*, not itself | Quality review (comparison with Spec A/B rubrics) | Cleared scorecard to blank (to be filled when grading the spec). | Rev 2 |
| R11 | P2, P7 | Missing CortenForge codebase file:line references in criterion bars | Quality review (comparison with Spec A/B rubrics) | Added `data.rs:499`, `data.rs:51`, `model.rs:171`, `model.rs:123`, `model.rs:125`, `postprocess.rs:12,21,29`, `enums.rs:342–462`, `types.rs:2863–3060` references throughout P2, P3, P7 bars. | Rev 2 |
| R12 | All | Missing analytical verification data (Spec A has V1–V4 derivations, Spec B has V1–V3) | Quality review (comparison with Spec A rubric EGT-4) | Added EGT-6 with V1–V6 physics derivations for all empirical values. | Rev 2 |
| R13 | P7 | Missing exhaustive match-site inventory (Spec A has 28 sites in 4 tables, Spec B has 3 tables). **Superseded by R65 (Rev 10), R84 (Rev 12):** count grew from 14→15→18 match sites across 5 tables as compiler.rs, MjcfSensor struct, and parse_sensor_attrs() were identified. | Quality review (comparison with Spec A rubric EGT-5) | Added EGT-8 with 14 match sites across 4 tables, marking exhaustiveness and wildcard danger zones. | Rev 2 |
| R14 | P2, P10 | Missing architectural decisions (Spec A has 4 decisions, Spec B has 2) | Quality review (comparison with Spec A rubric EGT-6) | Added EGT-9 with 4 architectural decisions: AD-1 through AD-4. | Rev 2 |
| R15 | P7 | Wildcard matches in position.rs:99, acceleration.rs:81, and postprocess.rs:60 silently produce wrong output if new sensor type arms are omitted — not flagged as danger zones. **Superseded by R65 (Rev 10), R79 (Rev 11), R84 (Rev 12):** wildcard count is 7 (added compiler.rs:85); postprocess skip guard location corrected to lines 55–57 (before loop, not at line 60); 3 struct/function sites added. | Quality review (match-site audit) | EGT-8 explicitly marks 6 wildcard sites as **DANGER** with bold warnings. P7 A+ bar now references "8 exhaustive + 6 wildcard" split. | Rev 2 |
| R16 | P1 | "A+ rubric, broken spec" scenario: a spec could describe the MuJoCo C source perfectly but use wrong CortenForge field names (e.g., `jnt_dofadr` instead of `jnt_dof_adr`), causing compilation errors or wrong field reads | Stress-test (adversarial audit) | P3 A+ bar now mandates CortenForge field names with file:line references. EGT-7 provides complete mapping table. | Rev 2 |
| R17 | P9 | "A+ rubric, broken spec" scenario: spec could correctly describe body×geom dispatch but omit `body_geom_adr`/`body_geom_num` field names, leaving implementer to guess the CortenForge equivalent of MuJoCo's `body_geomadr` | Stress-test (adversarial audit) | P9 A+ bar now explicitly requires `model.body_geom_num[bodyid]` (`model.rs:125`) and `model.body_geom_adr[bodyid]` (`model.rs:123`). | Rev 2 |
| R18 | P5 | "A+ rubric, broken spec" scenario: test plan could require "conformance tests" but not specify what tolerance to use, leaving implementer to guess between exact equality, 1e-10, 1e-6 etc. | Stress-test (adversarial audit) | P4 A+ bar requires "exact expected value or tolerance". EGT-6 V1–V6 derivations provide exact analytical values matching empirical data, enabling exact-equality tests (no tolerance needed for integer-precision results like 11.0 or geometry with rational coordinates). | Rev 2 |
| R19 | P7 | "A+ rubric, broken spec" scenario: spec could list all files but miss `from_str()` at `types.rs:2942` which uses wildcard — new MJCF element names silently ignored at parse time | Stress-test (adversarial audit) | EGT-8 Table 2 marks `from_str()` as **wildcard, compiler will NOT catch omissions** with bold warning. P7 A+ bar references the exhaustive/wildcard split. | Rev 2 |
| R20 | P2 | "A+ rubric, broken spec" scenario: algorithm section could specify evaluation code for position.rs but forget to add JointActuatorFrc to acceleration.rs — wildcard catch-all silently returns 0 | Stress-test (adversarial audit) | EGT-8 Table 3 explicitly marks `mj_sensor_acc()` at `acceleration.rs:81` as wildcard danger zone. P2 A+ bar references the index chain with file:line for JointActuatorFrc. | Rev 2 |
| R21 | P1 | `mjDATATYPE_QUATERNION` numeric value cited as (2) — actual MuJoCo value is (3). `mjDATATYPE_AXIS` is (2). **This fix was itself wrong** — it changed the GeomNormal datatype claim from AXIS(2) to QUATERNION(3), but GeomNormal's actual datatype IS AXIS(2), as confirmed empirically in Rev 6 (R34). The Rev 3 "fix" made the metadata table MORE wrong by applying the correct QUATERNION value to the wrong sensor. | Factual audit (mjmodel.h verification) | Rev 3: Changed (2)→(3). **Rev 6: Reverted — GeomNormal is AXIS(2), not QUATERNION(3).** | Rev 3, **corrected Rev 6** |
| R22 | P3 | `sensor_write4` line reference cited as `postprocess.rs:28` — actual function signature is at line 29 (line 28 is the `#[inline]` attribute) | Factual audit (codebase line verification) | Fixed all references: `postprocess.rs:12,21,28` → `postprocess.rs:12,21,29`. | Rev 3 |
| R23 | P1 | **CRITICAL:** cutoff=0 semantics were WRONG. Rubric claimed "cutoff=0 means no computation, returns 0." Empirical test: overlapping spheres with cutoff=0 → distance=-0.700000, normal=[-1,0,0], fromto=[0.5,0,0,-0.2,0,0]. cutoff=0 only suppresses POSITIVE distances (non-penetrating). Penetration is still detected and reported. | Stress-test (empirical re-verification) | Rewrote EGT-3 cutoff semantics section and empirical data. Updated P1 A+ bar. Added cutoff=0+penetration edge case to P5 A+ bar. | Rev 4 |
| R24 | P1 | JointActuatorFrc joint type restriction: MuJoCo's compiler rejects ball and free joints with `ValueError: joint must be slide or hinge in sensor`. Rubric did not mention this restriction. | Stress-test (empirical verification via `uv run python3`) | Added restriction note to EGT-2. Updated P1 A+ bar. Added parser rejection test to P5 edge case inventory. Updated EGT-8 resolve_sensor_object entry. | Rev 4 |
| R25 | Scope | `InsideSite` (`mjSENS_INSIDESITE`) exists in MuJoCo 3.5.0 but is not tracked in umbrella spec, Spec C scope, or any DT-ID. Requires geometric containment testing. **Resolved by R74 (Rev 11):** assigned DT-121. | Stress-test (mjmodel.h enum audit) | Added note to Scope Adjustment section. Needs DT-ID assignment (verify ID availability). | Rev 4 |
| R26 | P9 | AD-4 stated "The builder already stores `sensor_reftype`/`sensor_refid` arrays — just populate them." Verified TRUE (`model.rs:504–506`, `builder/sensor.rs:47–52`). However, `MjcfSensor` struct (`types.rs:3069–3091`) does NOT have `geom1`/`geom2`/`body1`/`body2` fields — parser extension required. | Stress-test (codebase verification) | Updated AD-4 recommendation with note about missing parser fields and file:line references. | Rev 4 |
| R27 | P9 | Clock sensor needs early return in `resolve_sensor_object()` before the `objname.ok_or_else(...)` check at line 89, same pattern as User (line 85–87). Without this, Clock with no objname will error with "sensor missing object name." | Stress-test (code path analysis) | Updated EGT-8 Table 2 entry for resolve_sensor_object to note Clock early return requirement. | Rev 4 |
| R28 | P1, P3 | **CRITICAL:** `MjSensorDataType` ≠ `mjDATATYPE`. CortenForge's `MjSensorDataType` (`enums.rs:480–492`) stores pipeline STAGE (`Position`/`Velocity`/`Acceleration`), NOT data kind (`REAL`/`POSITIVE`/`AXIS`/`QUATERNION`). The rubric's references to "QUATERNION datatype cutoff exemption" implied CortenForge has a data-kind mechanism — it doesn't. GeomNormal must be exempted via explicit sensor type matching, not a datatype check. | Stress-test (architectural audit) | Rewrote EGT-3 QUATERNION note with architectural clarification. Updated P1 A+ bar to say "explicit type match" not "QUATERNION datatype exemption". AD-3 option (a) was already correct. | Rev 5 |
| R29 | P1 | Self-distance (geom1==geom2) rejected by MuJoCo compiler: `"1st body/geom must be different from 2nd body/geom"`. Not previously documented as edge case. | Stress-test (empirical) | Added to EGT-3 empirical data and P1/P5 edge case lists. | Rev 5 |
| R30 | P1 | Body with zero geoms: N=0 → all-pairs loop body never executes → dist stays at cutoff initialization value → sensor returns cutoff (e.g., 10.0 with cutoff=10). Not previously documented. | Stress-test (empirical) | Added to EGT-3 empirical data and P1/P5 edge case lists. | Rev 5 |
| R31 | P1 | Coincident geoms (same position, different IDs): distance = -(2*radius), normal = arbitrary direction from collision system, fromto = surface points. Not previously documented. | Stress-test (empirical) | Added to EGT-3 empirical data. | Rev 5 |
| R32 | P1 | Postprocess DOES clamp GeomDist negative values: penetration -0.7 with cutoff=0.5 → sensor reads -0.5 (clamped by `clamp(-cutoff, cutoff)` in `postprocess.rs:66`). This is a second layer of clamping beyond the eval-stage cutoff initialization. | Stress-test (empirical) | Added to EGT-3 empirical data and P1/P5 edge case lists. | Rev 5 |
| R33 | P7 | `sensor_cutoff` line number was `—` in EGT-7 table. | Stress-test (codebase grep) | Fixed: `sensor_cutoff` at `model.rs:514`. | Rev 5 |
| R34 | P1 | **CRITICAL:** GeomNormal datatype is `mjDATATYPE_AXIS` (2), NOT `mjDATATYPE_QUATERNION` (3). Empirically verified: `m.sensor_datatype[sensor_id]` returns 2 for GeomNormal. The Rev 3 "fix" (R21) that changed the metadata table from (2) to (3) was based on faulty research — the agent checked the enum ordering for QUATERNION but applied it to the wrong sensor. GeomNormal has always been AXIS. | Stress-test round 3 (empirical `m.sensor_datatype[]` verification) | Fixed metadata table: `mjDATATYPE_QUATERNION (3)*` → `mjDATATYPE_AXIS (2)*`. Rewrote EGT-3 architectural note with actual `apply_cutoff()` C source showing AXIS implicit skip. Updated all QUATERNION→AXIS references throughout (lines 233, 428, 463–471, R4, R21). | Rev 6 |
| R35 | P1 | MJCF element names `<distance>`, `<normal>`, `<fromto>` could collide with other XML elements sharing the same names. Investigation confirmed parser is SAFE — `parser.rs:3417–3450` dispatches on element name contextually within the `<sensor>` section; `<distance>` under `<equality>` (line 2209) is a separate code path. | Stress-test round 3 (parser collision analysis) | No change needed — parser is context-aware. Documented as verified-safe. | Rev 6 |
| R36 | P3 | `sensor_dim` population mechanism: stored as `Vec<usize>` in Model, populated from `MjSensorType::dim()` method (exhaustive match at `enums.rs:427–462`) via `process_sensors()` in `builder/sensor.rs:35/54`. Not a Model field that needs manual population — computed from the type. | Stress-test round 3 (codebase trace) | No rubric change needed. Verified dim() exhaustive match is already in EGT-8 Table 1. | Rev 6 |
| R37 | P3 | `sensor_objid` stored as `Vec<usize>` — cannot store -1. Clock maps to objid=0, which is safe because `sensor_objtype` is `MjObjectType::None` and evaluation code checks objtype before using objid. User sensor also uses early returns. | Stress-test round 3 (type analysis) | No rubric change needed. Storage mechanism verified compatible with Clock's no-object semantics. | Rev 6 |
| R38 | P1 | MuJoCo's `apply_cutoff()` function in `engine_sensor.c` — full source verified. Explicit handling: `CONTACT` type → special path, `GEOMFROMTO` → early return, then `if (type==REAL) clamp` `else if (type==POSITIVE) max(0,min)`. AXIS and QUATERNION have NO matching branch → implicitly skip (no clamping applied). This is the actual exemption mechanism, not an explicit "exempt AXIS" check. | Stress-test round 3 (C source verification) | Rewrote EGT-3 note to include actual `apply_cutoff()` C source excerpt and explain the implicit-skip mechanism. | Rev 6 |
| R39 | P7 | AD-3 options referenced QUATERNION datatype for GeomNormal exemption throughout all three options. Since CortenForge's `MjSensorDataType` stores pipeline stage (not data kind), and GeomNormal's actual datatype is AXIS (not QUATERNION), all three options needed rewriting. Option (b) rewritten to propose a new `MjSensorDataKind` enum. Option (c) simplified to pure explicit type match. | Stress-test round 3 (consistency audit) | Rewrote all three AD-3 options and recommendation to reflect AXIS datatype and `MjSensorDataType` ≠ `mjtDataType` architectural fact (R28). | Rev 6 |
| R40 | P10 | **MEDIUM:** P10's A+ bar misattributed cutoff=0 suppression to `geom_distance()` itself ("If cutoff ≤ 0, returns 0 with zero fromto"). In MuJoCo, `mj_geomDistance()` always computes the actual signed distance — cutoff is a search radius hint. The suppression of positive distances when cutoff=0 is an emergent property of the **caller's** evaluation loop pattern (`dist = cutoff; if (dist_new < dist)`). A spec following P10 literally would bake caller logic into the helper, deviating from MuJoCo's architecture. **Corrected by R82 (Rev 12):** "always computes actual signed distance" is itself wrong — `mj_geomDistance()` returns `min(actual_dist, cutoff)` for non-penetrating geoms, i.e., the return value IS capped at cutoff. | Stress-test round 4 (P-criteria consistency audit) | Rewrote P10 A+ bar: `geom_distance()` always computes actual signed distance; cutoff=0 suppression belongs to evaluation loop, not helper. | Rev 7 |
| R41 | P1 | Beyond-cutoff asymmetry: when actual distance > cutoff, GeomDist returns the cutoff value, but GeomNormal returns `[0,0,0]` and GeomFromTo returns `[0,0,0,0,0,0]`. The rubric documented distance=cutoff but not the zeroing of normal/fromto. Empirically verified: geoms 10 apart, cutoff=1 → dist=1.0, normal=[0,0,0], fromto=[0,0,0,0,0,0]. | Stress-test round 4 (empirical) | Added to EGT-3 (new "Beyond-cutoff behavior" section), P1 A+ bar, P5 edge case inventory, and empirical data. | Rev 7 |
| R42 | P1 | GeomFromTo output ordering: `[from_x, from_y, from_z, to_x, to_y, to_z]` where "from" = nearest surface point on geom1 and "to" = nearest surface point on geom2. Swapping `geom1`↔`geom2` swaps from↔to. Empirically verified: `ft(g1→g2)=[0.1,0,0, 0.8,0,0]`, `ft(g2→g1)=[0.8,0,0, 0.1,0,0]`. Not previously documented. | Stress-test round 4 (empirical) | Added to EGT-3 (object resolution notes and empirical data). | Rev 7 |
| R43 | P1 | Penetrating fromto: surface points go **inside** the opposite geom's volume. `g1 r=0.5 at origin, g2 r=0.5 at (0.3,0,0)` → `fromto=[0.5, 0, 0, -0.2, 0, 0]` — from point (0.5) is inside g2, to point (-0.2) is inside g1. Points do NOT swap or clamp at boundaries. | Stress-test round 4 (empirical) | Added to EGT-3 (object resolution notes and empirical data). | Rev 7 |
| R44 | P1 | Negative cutoff: MuJoCo compiler rejects at compile time with `"negative cutoff in sensor"`. Not a runtime case. Previously undocumented. | Stress-test round 4 (empirical) | Added to EGT-3 and P1/P5 edge case lists. | Rev 7 |
| R45 | P1, P3 | Noise is metadata-only: `sensor_noise` is stored in the model but **not applied** during `mj_forward`. The forward pass is fully deterministic. Verified: 5 consecutive `mj_forward` calls with `noise=0.1` all return identical values. This means CortenForge only needs to store the noise value, not apply it. | Stress-test round 4 (empirical) | Added to EGT-3 and P1/P5 notes. | Rev 7 |
| R46 | P1, P9 | **Body-pair uses direct geoms only, NOT subtree.** `body_geomadr`/`body_geomnum` enumerate only the body's own geoms — child body geoms are excluded. Empirically verified: b1 with direct geom at (0,0,0) + child body at (1.5,0,0) with geom → body-pair distance matches direct geom only (1.8), not child geom (0.3). Also: tie-breaking uses first-found (strict `<` comparison), minimum-distance pair's normal/fromto are used. | Stress-test round 4 (empirical) | Updated EGT-3 object resolution, P1 A+ bar ("direct geoms only"), P9 A+ bar (tie-breaking, first-found semantics). | Rev 7 |
| R47 | P1 | P1's A+ bar text used MuJoCo C field names (`body_geomnum`/`body_geomadr`) instead of CortenForge names (`body_geom_num`/`body_geom_adr`). While P3 catches this in specs, the rubric itself should model correct naming. | Stress-test round 4 (P-criteria consistency audit) | Fixed P1 A+ bar to use CortenForge field names. | Rev 7 |
| R48 | P1, P9 | **CRITICAL:** Rubric claimed "body1/body2 takes precedence if both specified." Empirical test: specifying BOTH `geom1` and `body1` on the same side produces parse error `"exactly one of (geom1, body1) must be specified"`. There is NO precedence — it is strict XOR. Specifying neither is also an error. | Stress-test round 5 (empirical) | Rewrote EGT-3 object resolution rules: strict XOR validation, no precedence. Updated P1 A+ bar and P9 A+ bar. | Rev 8 |
| R49 | P1, P9 | Mixed objtype/reftype (geom1+body2 and body1+geom2) are both valid — all 4 combinations supported. Empirically verified: `geom1+body2` → objtype=GEOM(5), reftype=BODY(1), distance=1.8. Not previously documented as a supported combination. | Stress-test round 5 (empirical) | Added to EGT-3, P1 A+ bar, P9 A+ bar, P5 edge case inventory. | Rev 8 |
| R50 | P1, P5 | Cutoff is **universal** — not restricted to geom distance sensors. Clock with cutoff=5 gets clamped at 5.0 by postprocess. JointActuatorFrc with cutoff=5 clamps to [-5,+5]. `apply_cutoff()` is a generic postprocessing step for ALL sensors with cutoff>0. | Stress-test round 5 (empirical) | Added to EGT-3, P1 A+ bar, P5 edge case inventory. | Rev 8 |
| R51 | P1 | `mj_geomDistance()` supports ALL geom types — sphere, box, capsule, cylinder, ellipsoid, and plane. Verified empirically: box-sphere (1.2), capsule-capsule (0.8), cylinder-sphere (1.5), plane-sphere (0.9), ellipsoid-sphere (1.6), rotated box-box (0.793 — confirms GJK/MPR, not analytic). | Stress-test round 5 (empirical) | Added to EGT-3 and EGT-4. | Rev 8 |
| R52 | P1, P5 | Same-body geoms (two different geoms on the same body) CAN be distance-measured — no restriction. Self-distance (geom1==geom2) is still rejected by compiler. | Stress-test round 5 (empirical) | Added to EGT-3 object resolution rules and P5 edge case inventory. | Rev 8 |
| R53 | EGT-8 | Match site file count was "6 files" — actual count is **7 distinct files** (`enums.rs`, `types.rs`, `builder/sensor.rs`, `position.rs`, `velocity.rs`, `acceleration.rs`, `postprocess.rs`). **Superseded by R65 (Rev 10, +compiler.rs→8 files), R90 (Rev 13, +parser.rs→9 files).** | Stress-test round 5 (count audit) | Fixed EGT-8 summary: "6 files" → "7 files". | Rev 8 |
| R54 | P3 | `MjObjectType::None` ↔ `mjOBJ_UNKNOWN` naming difference missing from P3 convention table. The table listed `MjObjectType::Body` and `Geom` but not `None`. Clock uses `objtype=None` (CortenForge) = `mjOBJ_UNKNOWN` (MuJoCo). | Stress-test round 5 (naming audit) | Added `MjObjectType::None` ↔ `mjOBJ_UNKNOWN` (0) mapping to P3 A+ bar. | Rev 8 |
| R55 | P1 | EGT-6 coverage gap: 4 empirical values had no analytical derivations — multi-geom body distance (V8), penetrating normal sign flip (V9), postprocess cutoff clamping (V10), positive-distance cutoff clamping (V11). All derivations are mathematically straightforward but test distinct code paths. | Stress-test round 5 (EGT-6 audit) | Added V8–V11 derivations to EGT-6. | Rev 8 |
| R56 | P3, P7 | Variant naming conventions verified against codebase: `MjcfSensorType` uses flat capitalization (`Jointactuatorfrc` matching `Jointlimitfrc`, `Actuatorfrc`). `MjSensorType` uses PascalCase (`JointActuatorFrc` matching `JointLimitFrc`, `ActuatorFrc`). Rubric's P3 claims are consistent. `sensor_adr` computation is a cumulative dim sum — dim=6 works with zero builder changes. | Stress-test round 5 (codebase verification) | No rubric change needed. Naming conventions and sensor_adr computation verified correct. | Rev 8 |
| R57 | P1, P5 | Cutoff clamping is **per-element**, not vector magnitude. Empirically verified: gyro `[2, 0.5, -3]` with `cutoff=1.0` → `[1.0, 0.5, -1.0]` (each component independently clamped). This matters for multi-dim sensors like GeomFromTo (dim=6) — each coordinate clamped individually, NOT the 6D vector magnitude. | Stress-test round 6 (empirical) | Added per-element clamping note to EGT-3. | Rev 9 |
| R58 | P1, P3 | MuJoCo compiler rejects cutoff on `mjDATATYPE_AXIS` and `mjDATATYPE_QUATERNION` sensors. Only `GeomNormal` (AXIS datatype) needs an explicit exemption — the compiler check at `user_objects.cc:mjCSensor::Compile()` is `(datatype == AXIS && type != mjSENS_GEOMNORMAL)`. `GeomDist`/`GeomFromTo` have REAL datatype so the check never triggers. Compile-time rejection, not runtime skip. | Stress-test round 6 (C source verification) | Added compiler cutoff rejection note to EGT-3. **Rev 10: corrected file/function reference (was `engine_sensor.c:checkaliassensor()` which does not exist).** | Rev 9, **corrected Rev 11** |
| R59 | EGT-7 | **MEDIUM:** EGT-7 model fields table was missing `sensor_objtype` (`model.rs:500`), `sensor_objid` (`model.rs:502`), `sensor_reftype` (`model.rs:504`), `sensor_refid` (`model.rs:506`). These are needed by GeomDist/GeomNormal/GeomFromTo for dual-object resolution (objtype+reftype encode which combination of geom/body is specified on each side). | Stress-test round 6 (completeness audit) | Added all 4 fields with file:line references to EGT-7. | Rev 9 |
| R60 | EGT-8 | Table 3 said "Add 4 arms" to `mj_sensor_pos()` — ambiguous whether 4 separate match arms or 2 arms with combined patterns. Since GeomDist/GeomNormal/GeomFromTo share identical object resolution code (dual-object → `geom_distance()` → write), they should be a single combined match arm `GeomDist \| GeomNormal \| GeomFromTo`, mirroring the `FrameXAxis \| FrameYAxis \| FrameZAxis` pattern at `position.rs:213`. Clock is a trivial separate arm. | Stress-test round 6 (implementation pattern analysis) | Changed "Add 4 arms" → "Add 2 arms: (1) Clock, (2) GeomDist\|GeomNormal\|GeomFromTo combined arm" with rationale. | Rev 9 |
| R61 | P4 | End-to-end verification: all 5 sensor types (Clock, JointActuatorFrc, GeomDist, GeomNormal, GeomFromTo) coexist correctly in a single model. `nsensordata=15` for 8 sensors (1+1+1+1+3+6+1+1), `sensor_adr` spacing correct, all values numerically correct. Clock cutoff clamping also verified end-to-end. | Stress-test round 6 (empirical integration test) | No rubric change needed. End-to-end coexistence verified. | Rev 9 |
| R62 | P7, P3 | Self-audit of all 28+ cited file:line references in rubric: ALL verified accurate against current codebase. No stale line numbers found. `enums.rs:342–421` (MjSensorType), `enums.rs:427–462` (dim()), `enums.rs:480–492` (MjSensorDataType), `model.rs:500–514` (sensor fields), `postprocess.rs:12,21,29,60` (write/cutoff), `position.rs:213` (combined arm) — all correct. | Stress-test round 6 (reference accuracy audit) | No rubric change needed. All file:line references verified current. | Rev 9 |
| R63 | AD-4 | AD-4 correctly delegates XOR validation to spec writers via EGT-3 and P1/P9 A+ bars rather than prescribing implementation. The rubric establishes the REQUIREMENT (strict XOR) and the PRECEDENT (builder/sensor.rs pattern), letting the spec decide the exact validation approach. | Stress-test round 6 (architectural decision audit) | No rubric change needed. Delegation strategy verified sound. | Rev 9 |
| R64 | Scorecard | Rubric scorecard section verified BLANK — no pre-filled grades. This is correct: the scorecard is filled during spec review, not rubric authoring. All 10 criteria (P1–P10) have A+ bars defined but no grades assigned. | Stress-test round 6 (scorecard audit) | No rubric change needed. Scorecard correctly blank. | Rev 9 |
| R65 | EGT-8, P7 | **CRITICAL:** Missing match site — `apply_fusestatic()` at `builder/compiler.rs:85–103` matches on `MjcfSensorType` with `_ => {}` wildcard. Geom distance sensors with `body1`/`body2` attributes reference body names that must be protected from fusestatic optimization, otherwise the referenced body could be silently fused away, invalidating the sensor. This is a 15th match site in an 8th file, not covered by the previous "14 sites across 7 files" inventory. | Stress-test round 7 (match-site search beyond EGT-8) | Added EGT-8 Table 5 for `compiler.rs`. Updated EGT-8 summary: "15 match sites across 8 files" (6 exhaustive + 2 enum defs + 7 wildcard). Updated P7 A+ bar to include `compiler.rs`. | Rev 10 |
| R66 | EGT-3 | **MEDIUM:** `apply_cutoff()` C excerpt (lines 200–209) was simplified pseudocode presented without labeling — missing `for (int j=0; j<dim; j++)` loop, variable declarations, and replacing MuJoCo's actual comments with interpretive ones. The missing loop conceals the per-element iteration structure. | Stress-test round 7 (C source audit) | Rewrote excerpt with actual MuJoCo structure: loop, variable declarations, explicit "AXIS/QUATERNION: no branch" comment. Labeled as "simplified from `engine_sensor.c:64–89`". | Rev 10 |
| R67 | Scope | **MEDIUM (pre-existing bug):** CortenForge `postprocess.rs:62` treats `Rangefinder` as POSITIVE datatype (`min(cutoff)`) but MuJoCo assigns `mjDATATYPE_REAL` to Rangefinder (bilateral `clamp(-cutoff, cutoff)`). Not introduced by this rubric, but rubric discusses postprocess cutoff in detail without flagging this conformance bug. Tracked for post-Spec C fix. | Stress-test round 7 (postprocess audit against MuJoCo user_objects.cc) | Not a Spec C rubric fix — flagged as pre-existing conformance bug for separate tracking. | Rev 10 |
| R68 | Self-audit | **MEDIUM:** Self-audit "Conformance primacy" checklist item said "V1–V6 from EGT-6" but EGT-6 contains V1–V11 (V7–V11 added in Rev 8). Stale reference. | Stress-test round 7 (self-audit audit) | Fixed: "V1–V6" → "V1–V11". | Rev 10 |
| R69 | P1, P5 | **MEDIUM:** P5 A+ bar required testing "JointActuatorFrc with no actuators on joint (expect 0)" but P1 never described this edge case, and EGT-2 had no empirical data. Test requirement with no ground truth. The expected behavior is 0.0 — `qfrc_actuator[dof_adr]` stays at reset value (zeroed in `Data::reset()` at `data.rs:866`). | Stress-test round 7 (P1↔P5 cross-reference) | Added zero-actuator case to EGT-2 with derivation from `Data::reset()`. | Rev 10 |
| R70 | EGT-8, P7 | **MEDIUM:** Match-site counts were inflated — "14 sites, 8 exhaustive" counted 2 enum definitions as "exhaustive match sites" despite being marked "N/A (enum)" in the tables. Correct count (before R65 addition): 12 match sites (6 exhaustive + 6 wildcard) + 2 enum defs. After R65: 15 sites total (6 exhaustive + 2 enum defs + 7 wildcard). | Stress-test round 7 (count audit) | Fixed EGT-8 summary and P7 A+ bar to reflect accurate categorization. | Rev 10 |
| R71 | EGT-3 | **MEDIUM:** Line 235 said all three geom distance sensors are "explicitly exempted" from the compiler AXIS/QUATERNION cutoff check. Only `GeomNormal` has an actual exemption (`type != mjSENS_GEOMNORMAL` in the compiler condition). `GeomDist` and `GeomFromTo` have `mjDATATYPE_REAL` so the AXIS/QUATERNION check never triggers — they are unaffected, not exempted. | Stress-test round 7 (compiler check analysis) | Rewrote to clarify: only GeomNormal has an explicit exemption; GeomDist/GeomFromTo are unaffected due to REAL datatype. | Rev 10 |
| R72 | EGT-8, P7 | **CRITICAL:** Table 5 fusestatic description said "Add match arms to protect body1/body2" but the existing `apply_fusestatic()` reads `sensor.objname` (line 98), which is a DIFFERENT field from the `body1`/`body2` attributes used by geom distance sensors. `MjcfSensor` lacks `body1`/`body2` fields entirely (AD-4/R26 prerequisite). Adding match arms that read `objname` would silently fail to protect any bodies. Even the current parser's `objname` chain reads `"body"` (singular), not `"body1"`. | Stress-test round 8 (fusestatic implementation analysis) | Rewrote Table 5 description: must read `sensor.body1`/`sensor.body2` (new fields per AD-4), NOT `sensor.objname`. Added cross-reference to AD-4/R26. | Rev 11 |
| R73 | Scope | **CRITICAL:** DT-118 ID conflict — rubric assigned DT-118 to CamProjection deferral, but DT-118 is already `mj_contactForce()` per `future_work_15.md:310` and `index.md:114`. The correct camera deferral DT-ID is DT-120 (per `future_work_15.md:287`). | Stress-test round 8 (DT-ID cross-reference) | Changed DT-118 → DT-120 in Scope Adjustment, EGT-5, and R2 gap log entry. | Rev 11 |
| R74 | Scope | **CRITICAL:** DT-119 ID conflict — rubric assigned DT-119 to InsideSite with hedging language "verify ID availability" but never resolved. DT-119 is already Ray-Geom Intersection Filter per `future_work_15.md:336`. | Stress-test round 8 (DT-ID cross-reference) | Changed to DT-121. Verified DT-118=contactForce, DT-119=ray-geom, DT-120=camera — all taken. | Rev 11 |
| R75 | R58 | **CRITICAL:** R58 gap log cited `engine_sensor.c:checkaliassensor()` — this function does not exist in MuJoCo. The actual compiler cutoff check for AXIS/QUATERNION is in `user_objects.cc:mjCSensor::Compile()` at lines 7929–7932. The EGT-3 body text (line 248) was already correct; only the gap log entry had the wrong provenance. | Stress-test round 8 (function existence verification) | Corrected R58 to cite `user_objects.cc:mjCSensor::Compile()`. | Rev 11 |
| R76 | EGT-3 | **MEDIUM:** C source line reference "lines 854–918" for the GeomDist/GeomNormal/GeomFromTo case block was wrong — actual location in `engine_sensor.c` is approximately lines 666–720. Line numbers drift between MuJoCo versions. | Stress-test round 8 (line number verification) | Removed specific line numbers from EGT-3 heading (version-dependent). | Rev 11 |
| R77 | EGT-4 | **MEDIUM:** Said `mj_geomDistance()` is defined in `engine_collision_sdf.c (or similar)`. Actual location is `engine_support.c`. `engine_collision_sdf.c` contains a different file-scoped `geomDistance()` helper. | Stress-test round 8 (file location verification) | Fixed: `engine_collision_sdf.c (or similar)` → `engine_support.c`. | Rev 11 |
| R78 | P1, P5 | **MEDIUM:** MJCF schema marks `cutoff` as `required="true"` for `<distance>`, `<normal>`, `<fromto>` but optional for `<clock>` and `<jointactuatorfrc>`. The rubric discussed cutoff=0 default behavior but never noted this required/optional distinction. Parser must enforce required cutoff for geom distance sensors. **Corrected by R85 (Rev 12):** MuJoCo runtime does NOT enforce `required` — cutoff defaults to 0 when omitted. | Stress-test round 8 (schema.xml verification) | Added cutoff `required` note to EGT-3 MJCF element description. Added to P1/P5 edge case scope. | Rev 11 |
| R79 | AD-3, EGT-8 | **MEDIUM:** AD-3 said "add to cutoff dispatch at postprocess.rs:60" — line 60 is inside the per-element `for i in 0..dim` loop. MuJoCo's `apply_cutoff()` exits BEFORE the loop (`if (type == mjSENS_GEOMFROMTO) return;`). The skip guard should be between lines 55–57 (before the loop), not at line 60 (inside the match). | Stress-test round 8 (postprocess code structure analysis) | Rewrote AD-3 option (a) and EGT-8 Table 4 to specify skip before loop, not inside match. | Rev 11 |
| R80 | P6 | **MEDIUM:** P6 A+ bar says "Dependencies on Spec A (objtype parsing infrastructure)" without enumerating specific Spec A features: `resolve_sensor_object()` (builder/sensor.rs:78), `MjObjectType::{Body, Geom}` variants, `sensor_objtype`/`sensor_objid` arrays. An implementer cannot determine Spec A readiness without reading Spec A. | Stress-test round 8 (P6 specificity audit) | Not fixed in rubric body (P6 A+ bar describes what the SPEC should contain, not what the rubric should contain). Documented for spec writer awareness. | Rev 11 |
| R81 | EGT-3 | **LOW:** Line 191 still said "geom distance sensors are exempted" (plural, all three) despite R71 fixing only line 248. The metadata table note at line 191 was the same imprecise phrasing. | Stress-test round 8 (consistency audit) | Rewrote line 191 to specify only GeomNormal has an explicit exemption; GeomDist/GeomFromTo have REAL datatype and are unaffected. | Rev 11 |
| R82 | P10 | **CRITICAL:** P10 A+ bar claimed `mj_geomDistance()` "always computes actual signed distance." Empirically wrong: for non-penetrating geoms, it returns `min(actual_dist, cutoff)` — the return value is capped at cutoff. For penetrating geoms (negative distance), it returns exact distance. This is a return-value cap, not "always actual." | Stress-test round 9 (mj_geomDistance return value analysis) | Rewrote P10 A+ bar with empirically correct return-value-cap contract. | Rev 12 |
| R83 | EGT-6 | **CRITICAL:** V7 derivation had correct final result but wrong intermediate reasoning — stated `mj_geomDistance()` returns actual distance 0.7, then applies cutoff. Actual: function returns `min(0.7, 0.5) = 0.5` directly; postprocess sees 0.5. V11 had same issue. | Stress-test round 9 (derivation intermediate step audit) | Corrected V7 and V11 intermediate reasoning to show `min(actual, cutoff)` return semantics. Final results unchanged. | Rev 12 |
| R84 | EGT-8 | **CRITICAL:** Table 2 missing 3 match sites: `MjcfSensor` struct (`types.rs:3069`), `parse_sensor_attrs()` (`parser.rs:3453`), and `Default` impl (`types.rs:3093`). These are critical modification points for AD-4 dual-object field additions and geom1/body1/geom2/body2 attribute parsing. | Stress-test round 9 (parser dispatch analysis) | Added 3 entries to Table 2. Updated EGT-8 summary (15→18) and P7 A+ bar staleness counts. | Rev 12 |
| R85 | P1, P5 | **MEDIUM:** R78 (Rev 11) added note that parser must enforce `required` cutoff for geom distance sensors. Empirically wrong: MuJoCo 3.5.0 accepts `<distance>` with no cutoff attribute — defaults to 0. Schema `required="true"` is not enforced at runtime. Enforcing it in CortenForge would reject valid MJCF files. | Stress-test round 9 (empirical cutoff enforcement test) | Removed cutoff `required` enforcement note. Added empirical evidence that MuJoCo accepts omitted cutoff. Updated R5 annotation. | Rev 12 |
| R86 | P7 | **MEDIUM:** P7 A+ bar still said "cutoff exemption at line 60" despite R79 (Rev 11) correcting AD-3 and Table 4 to "between lines 55–57." Internal inconsistency. | Stress-test round 9 (P8 self-consistency audit) | Fixed P7 A+ bar: "at line 60" → "between lines 55–57, before the per-element loop". | Rev 12 |
| R87 | EGT-7 | **CRITICAL:** EGT-7 listed only sensor-specific read fields but omitted 8 infrastructure fields that EVERY new sensor type requires: `sensor_type` (`model.rs:496`), `sensor_datatype` (`model.rs:498`), `sensor_adr` (`model.rs:508`), `sensor_dim` (`model.rs:510`), `sensor_noise` (`model.rs:512`), `nsensor` (`model.rs:492`), `nsensordata` (`model.rs:494`), `sensordata` (`data.rs:398`). Without these, new sensor arms compile but evaluation never dispatches to them. | Stress-test round 9 (EGT-7 completeness audit) | Added 8 infrastructure fields in new sub-table within EGT-7. | Rev 12 |
| R88 | R5, R13, R15 | **LOW:** Historical gap log entries R5, R13, R15 were superseded by later findings but lacked correction annotations. R4 and R21 set the precedent of "**corrected by Rnn**" annotations for superseded entries. | Stress-test round 9 (gap log consistency audit) | Added "**Refined by**" / "**Superseded by**" annotations to R5, R13, R15 with cross-references. | Rev 12 |
| R89 | EGT-7 | **LOW:** `geom_body` type column was `—` (em dash) instead of `Vec<usize>`. Every other field in EGT-7 has its type documented; the dash was inconsistent. Verified at `model.rs:253`: `pub geom_body: Vec<usize>`. | Stress-test round 10 (EGT-7 field verification) | Fixed type column: `—` → `Vec<usize>`. | Rev 13 |
| R90 | EGT-8 | **MEDIUM:** Summary said "8 files" but 9 distinct files are listed across the 5 tables: `enums.rs`, `types.rs`, `builder/sensor.rs`, `parser.rs`, `position.rs`, `velocity.rs`, `acceleration.rs`, `postprocess.rs`, `builder/compiler.rs`. `parser.rs` was counted as part of the `types.rs` file group. | Stress-test round 10 (count consistency audit) | Fixed: "8 files" → "9 files". | Rev 13 |
| R91 | EGT-8 | **MEDIUM:** Summary said "18 match sites" but only 17 visible rows existed. The `Default` impl (types.rs:3093) was embedded as a note in the `MjcfSensor struct` row rather than having its own row. The "3 struct/function definitions" category counted it as separate, creating a row/count mismatch. | Stress-test round 10 (count consistency audit) | Split `Default` impl into its own Table 2 row. 18 rows now match 18 claimed sites. | Rev 13 |
| R92 | EGT-6 | **LOW:** V8 derivation did not state the cutoff value. EGT-3 shows cutoff=10 for this scenario. Since the `min(actual, cutoff)` return-value contract depends on cutoff, omitting it leaves ambiguity about whether `mj_geomDistance()` returns exact values. | Stress-test round 10 (derivation completeness audit) | Added "Cutoff=10 (both distances well below cutoff)" to V8 setup. | Rev 13 |
| R93 | EGT-6 | **LOW:** V11 said "caps at cutoff for non-penetrating geoms beyond search radius." The phrase "beyond search radius" is imprecise — the cutoff IS the search radius, and the cap applies when `actual_dist >= cutoff` (inclusive of equality). | Stress-test round 10 (derivation precision audit) | Rewrote: "caps at cutoff when `actual_dist >= cutoff`". | Rev 13 |
| R94 | R25, R78 | **MEDIUM:** R25 (Rev 4) resolution still said "Needs DT-ID assignment" but R74 (Rev 11) resolved this to DT-121. R78 (Rev 11) added cutoff `required` enforcement note but R85 (Rev 12) contradicted it. Neither had backward correction annotations. | Stress-test round 10 (gap log annotation audit) | Added "**Resolved by R74**" to R25 and "**Corrected by R85**" to R78. | Rev 13 |
| R95 | Scope | **MEDIUM:** DT-121 (InsideSite) is referenced in the rubric but is not registered in any canonical tracking file (`future_work_*.md`, `index.md`). Risk of ID re-assignment. | Stress-test round 10 (DT-ID registration audit) | Documented — requires registration in canonical tracking files outside rubric scope. | Rev 13 |
| R96 | EGT-6 | **LOW:** V4 (GeomNormal) references fromto values (0.1 and 0.8) derived in V5 (GeomFromTo), but V4 is presented before V5. Not a mathematical error, but a pedagogical ordering issue. | Stress-test round 10 (derivation ordering audit) | No fix applied — reordering V5 before V4 would break the conventional Dist→Normal→FromTo ordering. Noted for spec writer awareness. | Rev 13 |
| R97 | P5 | **CRITICAL:** P5 A+ bar said "with analytical backing V1–V7" but V8–V11 exist in EGT-6 since Rev 8 (R55). V8 (multi-geom body distance), V9 (penetrating normal/fromto), V10 (postprocess clamping), V11 (positive cutoff capping) all provide analytically-verified expected values not covered by the stale "V1–V7" range. R68 (Rev 10) fixed the self-audit's "V1–V6" → "V1–V11" but never propagated the fix to P5's A+ bar. | Stress-test round 11 (P5 vs EGT-6 coverage audit) | Fixed: "V1–V7" → "V1–V11" in P5 A+ bar. | Rev 14 |
| R98 | P5 | **MEDIUM:** Clock multi-step time progression (V1 formula: sensor reads `(N-1)*timestep` after N steps) not explicitly required as a test. P5 only required "clock sensor reads `data.time`" which could be satisfied by a t=0 check alone, missing the one-timestep-lag behavior. | Stress-test round 11 (P5 edge case completeness) | Added "Clock multi-step time progression" to P5 A+ bar edge case inventory. | Rev 14 |
| R99 | P5 | **MEDIUM:** Penetrating normal sign flip (V9: normal reverses from `[1,0,0]` to `[-1,0,0]` when geoms overlap) and penetrating fromto (V9: surface points inside opposite geom volume) not explicitly required as tests. V9 derivation exists with full analytical backing but P5 did not reference it. | Stress-test round 11 (P5 edge case completeness) | Added "Penetrating normal sign flip (V9)" and "Penetrating fromto (V9)" to P5 A+ bar. | Rev 14 |
| R100 | P5 | **MEDIUM:** `mjDSBL_SENSOR` and sleeping body sensor skip required by P5 as edge cases but have no EGT-backed empirical data. These are infrastructure-level behaviors inherited from the existing sensor framework, not Spec C-specific. | Stress-test round 11 (P5 evidence grounding audit) | Added "(infrastructure-level behaviors inherited from existing sensor framework)" annotation. | Rev 14 |
| R101 | AD-4 | **MEDIUM:** AD-4's recommended option (c) said "just populate them" for the reftype/refid mapping without specifying how `geom2`/`body2` fields map to `resolve_reference_object()`'s `reftype_str`/`refname` parameters. An implementer must decide whether to translate at parser or builder level. | Stress-test round 11 (AD-4 completeness audit) | Added explicit mapping chain: `geom2`/`body2` → `refname`+`reftype` via `resolve_reference_object()`. | Rev 14 |
| R102 | EGT-8 | **LOW:** All 18 exhaustiveness claims verified correct by reading every source function. 6 exhaustive matches confirmed: no wildcards. 7 wildcard matches confirmed: all have `_ => {}` or `_ => None`. 3 struct/function sites confirmed: no match involved. | Stress-test round 11 (exhaustiveness verification) | No rubric change needed. EGT-8 exhaustiveness inventory is accurate and trustworthy. | Rev 14 |
| R103 | AD-1, P6 | **CRITICAL:** AD-1 option (c) recommends "thin wrapper over existing collision" using `gjk_epa.rs`, but existing GJK/EPA only provides intersection testing and penetration depth (EPA). It does NOT provide unsigned separation distance for non-overlapping shapes or closest surface points. An implementer choosing option (c) would discover mid-implementation that `gjk_epa.rs` needs a GJK closest-point query extension, or analytic distance functions per shape pair. This is exactly the kind of dependency surprise P6 should prevent. | Stress-test round 12 (GJK/EPA capability audit) | Added caveat to AD-1 option (c) about separation distance gap. Added infrastructure gap note below AD-1. Updated P6 A+ bar to require spec to verify GJK/EPA separation distance capability. | Rev 15 |
| R104 | P4 | **MEDIUM:** P4 A+ bar did not require any AC that validates correct pipeline stage assignment. A stage misassignment (e.g., Clock arm in `velocity.rs` instead of `position.rs`, or JointActuatorFrc arm in `position.rs` instead of `acceleration.rs`) causes silent zero output via wildcard catch-all — and could pass zero-valued ACs. The JointActuatorFrc two-actuator AC (expected=11.0) implicitly validates stage correctness (since `qfrc_actuator` is zero before the acceleration pipeline), but this was not stated. | Stress-test round 12 (P4 bug-catching audit) | Added stage-correctness validation requirement to P4 A+ bar. Added explanatory note to JointActuatorFrc AC about implicit stage validation. | Rev 15 |
| R105 | P3 | **MEDIUM:** P3 A+ bar listed 8 convention items but omitted nalgebra types. Existing sensor code consistently uses `nalgebra::Vector3<f64>` (not raw `[f64; 3]`). `sensor_write3` takes `&Vector3<f64>`. The proposed `sensor_write6` and `geom_distance()` return type must specify whether they use nalgebra types or raw arrays. | Stress-test round 12 (P3 convention completeness audit) | Added item (9) to P3 A+ bar: nalgebra types convention. | Rev 15 |
| R106 | R53 | **LOW:** R53 (Rev 8) fixed file count "6 files" → "7 files" but was superseded by R65 (Rev 10, +compiler.rs→8) and R90 (Rev 13, +parser.rs→9) without a backward annotation. | Stress-test round 12 (gap log annotation audit) | Added "**Superseded by R65, R90**" annotation to R53. | Rev 15 |
| R107 | EGT-1, EGT-2 | **LOW:** Both sections verified thorough and accurate against MuJoCo source. All metadata values (enum numbers, dim, datatype, needstage, objtype) confirmed correct. C source excerpts match MuJoCo. Empirical data mathematically consistent. Indexing chain `qfrc_actuator[jnt_dof_adr[objid]]` verified. Hinge/slide restriction confirmed in compiler source. | Stress-test round 12 (EGT-1/EGT-2 deep audit) | No rubric change needed. Both sections are accurate and complete. | Rev 15 |
| R108 | All | **LOW:** End-to-end sweep confirmed all prior fixes (R82–R102) were faithfully applied to the rubric body. No stale "V1–V7", "at line 60", "8 files", "DT-118/119", "QUATERNION", "engine_collision_sdf.c", "checkaliassensor()", or cutoff enforcement text remains in the rubric body. Tonal consistency is excellent across all 15 revisions. | Stress-test round 12 (end-to-end integrity sweep) | No rubric change needed. All prior fixes confirmed applied. | Rev 15 |
| R109 | AD-1 | **LOW:** Stress-test round 13 Agent 1 independently verified R103's GJK/EPA claim by reading `gjk_epa.rs`. Confirmed: `gjk_epa_contact()` returns `None` for non-intersecting shapes, `GjkResult` has no distance field, `support()` / `gjk_intersection()` / `gjk_query()` / `epa_query()` — none compute unsigned separation distance. R103's caveat is fully accurate. | Stress-test round 13 (GJK/EPA source verification) | No rubric change needed. R103 claim confirmed. | Rev 16 |
| R110 | P8 | **CRITICAL:** P8 A+ bar did not require V-derivation consistency — that V1–V11 expected values match AC expected values. This cross-cutting consistency check is essential: a spec could have internally consistent ACs that contradict the analytically-derived V-derivations, or vice versa. | Stress-test round 13 (P8 completeness audit) | Added V-derivation consistency requirement and terminology hazard enumeration to P8 A+ bar. | Rev 16 |
| R111 | P9 | **MEDIUM:** P9 A+ bar correctly used `MjObjectType::Body` but did not warn about `Body`/`XBody` confusion. CortenForge has both `MjObjectType::Body` and `MjObjectType::XBody` which use different position/rotation arrays. Geom distance sensors use `Body` (not `XBody`), but an implementer could easily confuse them — existing sensor dispatch code has separate arms for both. Adversarial bug-catching test confirmed this was only PARTIALLY caught by the rubric. | Stress-test round 13 (adversarial bug-catching) | Added Body/XBody distinction warning to P9 A+ bar. | Rev 16 |
| R112 | P10 | **MEDIUM:** P10 A+ bar signature used raw `[f64; 6]` for fromto return type but P3 item (9) (added in Rev 15) requires specifying nalgebra vs raw array convention. The spec must coordinate these two requirements. | Stress-test round 13 (P3↔P10 consistency audit) | Added nalgebra coordination note to P10 A+ bar. | Rev 16 |
| R113 | P10 | **MEDIUM:** P10 A+ bar referenced "existing CortenForge collision primitives" without cross-referencing the AD-1 infrastructure gap (R103). An implementer following P10 could assume existing primitives are sufficient when they are not. | Stress-test round 13 (P10↔AD-1 cross-reference audit) | Added GJK gap cross-reference to P10 A+ bar. | Rev 16 |
| R114 | P1, P5 | **MEDIUM (usability):** P1 and P5 A+ bars are monolithic ~1500-character table cells. A cold-read implementer or spec writer would find it difficult to audit completeness against these text walls. Restructuring into numbered checklists would improve usability. However, markdown table cells with line breaks are fragile and the current format is technically complete. Documented for spec writer awareness — the SPEC itself should use checklists, but the rubric's table format is acceptable. | Stress-test round 13 (cold-read implementer simulation) | No rubric change applied. Noted as usability awareness item for spec writers — spec should decompose P1/P5 requirements into auditable checklists. | Rev 16 |
| R115 | EGT-8 | **MEDIUM (usability):** EGT-8 exhaustiveness column documents what IS (exhaustive vs wildcard) but does not prescribe what the SPEC should DO about wildcards. A spec writer reading EGT-8 knows the danger zones but has no actionable guidance on mitigation (e.g., should the spec require `#[deny(unreachable_patterns)]`, code-review ACs, integration tests?). Documented for spec writer awareness. | Stress-test round 13 (cold-read implementer simulation) | No rubric change applied. Noted as awareness item — the spec should prescribe wildcard mitigation strategy per P7's blast radius analysis. | Rev 16 |
| R116 | P8 | **MEDIUM:** P8 did not require the spec to enumerate terminology hazards — known naming confusions between MuJoCo and CortenForge that an implementer might trip on. Added enumeration requirement to P8 A+ bar. | Stress-test round 13 (P8 completeness audit) | Added terminology hazard enumeration to P8 A+ bar. | Rev 16 |
| R117 | P8, P4 | **MEDIUM:** Rev 16 expanded P8 A+ bar with V-derivation consistency and terminology hazard enumeration, but the P8↔P4 grading boundary was not stated. P4 already requires ACs to cite V-derivations (V1–V6 by name); P8 now requires those citations to be numerically consistent. The self-audit Non-overlap checklist discussed P1↔P9 and P2↔P10 but was silent on P4↔P8 and P3↔P8. | Stress-test round 14 (P8 expanded bar consistency) | Added P4↔P8 and P3↔P8 boundary statements to self-audit Non-overlap checklist. | Rev 17 |
| R118 | P8 | **MEDIUM:** P8 A/B/C grade bars were not updated when the A+ bar was expanded in Rev 16. The A/B/C bars still described only terminology consistency; they did not mention V-derivation consistency failures or missing terminology hazard sections. A spec failing item 9 or 10 of the P8 A+ bar had no matching lower-grade description. | Stress-test round 14 (P8 A/B/C bar alignment) | Updated P8 A/B/C bars to incorporate V-derivation and terminology hazard dimensions. | Rev 17 |
| R119 | P4 | **MEDIUM:** P4 A+ bar explicitly cited V1–V6 by name for happy-path ACs but omitted V7–V11 (edge-case derivations added in Rev 8). P8's V-derivation consistency requirement meant a spec could pass P4 without V7–V11 ACs but fail P8 for the same omission. | Stress-test round 14 (P4↔P8 coverage alignment) | Added V7–V11 edge-case AC references to P4 A+ bar. | Rev 17 |
| R120 | All | **LOW:** All 30+ file:line references in the rubric verified correct against current codebase. Zero stale line numbers found across 11 source files (`model.rs`, `data.rs`, `enums.rs`, `postprocess.rs`, `position.rs`, `velocity.rs`, `acceleration.rs`, `builder/sensor.rs`, `builder/compiler.rs`, `types.rs`, `parser.rs`). | Stress-test round 14 (line number freshness audit) | No rubric change needed. All file:line references are current. | Rev 17 |
| R121 | R40 | **CRITICAL:** R40 (Rev 7) stated `mj_geomDistance()` "always computes actual signed distance" — a principle that R82 (Rev 12) explicitly declared "Empirically wrong." R40 had no backward correction annotation, violating the convention set by R4, R21, R25, R78. A reader studying R40 in isolation would learn a false principle. | Stress-test round 14 (gap log integrity audit) | Added "**Corrected by R82 (Rev 12)**" annotation to R40. | Rev 17 |
| R122 | R58 | **MEDIUM:** R58 Revision column said "corrected Rev 10" but the correcting entry R75 is at Rev 11 (R72–R81). Rev 10 contains R65–R71. The corrected-Rev tag was off by one. | Stress-test round 14 (gap log integrity audit) | Fixed: "corrected Rev 10" → "corrected Rev 11". | Rev 17 |
| R123 | P9 | **LOW:** P9's Rev 16 Body/XBody warning stated the distinction matters because of "different position/rotation arrays (`xipos`/`ximat` vs `xpos`/`xmat`)." While true in general, geom distance sensors never access position arrays during evaluation — they only use `body_geom_adr`/`body_geom_num`. The real risk is hitting the wrong dispatch arm in `resolve_sensor_object()`. | Stress-test round 14 (spec-writer simulation) | Rewrote P9 Body/XBody justification to focus on dispatch routing, not position arrays. | Rev 17 |
| R124 | R110, R116 | **LOW:** R110 and R116 both modify P8 A+ bar in Rev 16 without cross-referencing each other. R110's resolution claims credit for both V-derivation AND terminology hazard additions, while R116 covers terminology hazard independently — a double-attribution. | Stress-test round 14 (gap log integrity audit) | Documented. Minor gap log redundancy — no rubric body change needed. | Rev 17 |
