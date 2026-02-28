# Spec B — Transmission Types + Slider-Crank

**Status:** Implemented
**Phase:** Roadmap Phase 5 — Actuator Completeness
**Effort:** M
**MuJoCo ref:** `mj_transmission()` in `engine_core_smooth.c`;
`mj_jacPointAxis()` and `mj_jacSite()` in `engine_core_util.c`
**MuJoCo version:** 3.2.6 (google-deepmind/mujoco `main` branch)
**Prerequisites:**
- Spec A (acc0, dampratio, lengthrange) landed in `a1cbbba`
- T1-b (§63) `dynprm` resize landed in `d4db634`

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the features described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Problem Statement

Two MuJoCo conformance gaps in actuator transmission types:

1. **DT-8 (Transmission types `jointinparent`, `cranksite`/`slidersite`):**
   MuJoCo supports 6 transmission types in `mj_transmission()` at
   `engine_core_smooth.c`. CortenForge implements 4 (`Joint`, `Tendon`,
   `Site`, `Body`). The `mjTRN_JOINTINPARENT` and `mjTRN_SLIDERCRANK` types
   are missing. Any MJCF model using `<general jointinparent="...">` or
   `<general cranksite="..." slidersite="...">` cannot be loaded.

2. **§61 (Slider-crank runtime computation):** The slider-crank transmission
   is a non-trivial 3D mechanism that computes actuator length from a
   crank-slider geometric linkage, with chain-rule Jacobian-based moment arm
   computation. This requires a new helper function (`mj_jac_point_axis`)
   that CortenForge does not yet have, plus a new standalone dispatch
   function for the slider-crank geometry, derivative, and Jacobian pipeline.

Both are **conformance gaps** — MuJoCo implements these in `mj_transmission()`
at `engine_core_smooth.c`; CortenForge does not implement them at all.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here.

### `mj_transmission()` — `engine_core_smooth.c`

The function computes `actuator_length` and `actuator_moment` for all
actuators. It dispatches on `m->actuator_trntype[i]` via a `switch`. The two
cases relevant to this spec:

#### Case: `mjTRN_JOINT` / `mjTRN_JOINTINPARENT` (combined case label)

```c
case mjTRN_JOINT:                   // joint
case mjTRN_JOINTINPARENT:           // joint, force in parent frame
```

The two cases share a single code block via C fall-through. Three sub-paths
based on joint type:

**Sub-path 1: Hinge/Slide (nv == 1) — scalar gear**

```c
if (m->jnt_type[id] == mjJNT_SLIDE || m->jnt_type[id] == mjJNT_HINGE) {
    rownnz[i] = 1;
    colind[adr] = m->jnt_dofadr[id];
    length[i] = d->qpos[m->jnt_qposadr[id]] * gear[0];
    moment[adr] = gear[0];
}
```

For hinge and slide joints, `mjTRN_JOINT` and `mjTRN_JOINTINPARENT` produce
**identical behavior** — no `trntype` check occurs in this path. The
transmission is purely scalar: `length = qpos * gear[0]`, `moment = gear[0]`.

**Sub-path 2: Ball joint (nv == 3) — 3D wrench gear**

```c
else if (m->jnt_type[id] == mjJNT_BALL) {
    mjtNum axis[3], quat[4];
    mji_copy4(quat, d->qpos + m->jnt_qposadr[id]);
    mju_normalize4(quat);
    mji_quat2Vel(axis, quat, 1);

    mjtNum gearAxis[3];
    if (m->actuator_trntype[i] == mjTRN_JOINT) {
        mji_copy3(gearAxis, gear);
    } else {
        mju_negQuat(quat, quat);
        mji_rotVecQuat(gearAxis, gear, quat);
    }
    length[i] = mju_dot3(axis, gearAxis);
    mji_copy3(moment + adr, gearAxis);
    rownnz[i] = 3;
}
```

For ball joints, `mjTRN_JOINTINPARENT` differs: it rotates the gear vector
by the inverse quaternion to express the force in the parent frame. **This
sub-path is OUT OF SCOPE** — see Out of Scope section for rationale.

**Sub-path 3: Free joint (nv == 6) — 6D wrench gear**

Also OUT OF SCOPE — same rationale as ball joints.

#### Case: `mjTRN_SLIDERCRANK` — the entirety

```c
case mjTRN_SLIDERCRANK:
{
    // Site IDs: id = cranksite (trnid[2*i]), idslider = slidersite (trnid[2*i+1])
    int idslider = m->actuator_trnid[2*i+1];
    mjtNum rod = m->actuator_cranklength[i];

    // Slider axis: z-column (third column) of slider site rotation matrix
    mjtNum axis[3] = {d->site_xmat[9*idslider + 2],
                      d->site_xmat[9*idslider + 5],
                      d->site_xmat[9*idslider + 8]};

    // Vector from slider site to crank site
    mjtNum vec[3];
    mju_sub3(vec, d->site_xpos + 3*id, d->site_xpos + 3*idslider);

    // Length and determinant
    mjtNum av = mju_dot3(vec, axis);
    mjtNum sdet, det = av*av + rod*rod - mju_dot3(vec, vec);
    int ok = 1;
    if (det <= 0) {
        ok = 0;
        sdet = 0;
        length[i] = av;
    } else {
        sdet = mju_sqrt(det);
        length[i] = av - sdet;
    }

    // Derivatives of length w.r.t. vec and axis
    mjtNum dlda[3], dldv[3];
    if (ok) {
        mju_scl3(dldv, axis, 1 - av/sdet);
        mju_scl3(dlda, vec, 1/sdet);       // temp in dlda
        mji_addTo3(dldv, dlda);             // dldv = axis*(1-av/sdet) + vec/sdet

        mju_scl3(dlda, vec, 1 - av/sdet);  // dlda = vec*(1-av/sdet)
    } else {
        mji_copy3(dlda, vec);               // dlda = vec
        mji_copy3(dldv, axis);              // dldv = axis
    }

    // Jacobians: point+axis at slider site, translational at crank site
    mj_jacPointAxis(m, d, jacS, jacA, d->site_xpos + 3*idslider,
                    axis, m->site_bodyid[idslider]);
    mj_jacSite(m, d, jac, 0, id);    // crank: translational only, jacr=NULL
    mju_subFrom(jac, jacS, 3*nv);    // jac = J_crank - J_slider_point

    // Chain rule: moment[j] = sum_k(dlda[k]*jacA[k*nv+j] + dldv[k]*jac[k*nv+j])
    moment_row = mjSTACKALLOC(d, nv, mjtNum);
    mju_zero(moment_row, nv);
    for (int j = 0; j < nv; j++) {
        for (int k = 0; k < 3; k++) {
            moment_row[j] += dlda[k]*jacA[k*nv+j] + dldv[k]*jac[k*nv+j];
        }
    }

    // Gear scaling (AFTER derivatives/Jacobians/moment)
    length[i] *= gear[0];

    // Compress to sparse; moment *= gear[0] during storage
    nnz = 0;
    for (int j = 0; j < nv; j++) {
        if (moment_row[j]) {
            moment[adr + nnz] = moment_row[j] * gear[0];
            colind[adr + nnz] = j;
            nnz++;
        }
    }
    rownnz[i] = nnz;
}
break;
```

**Key observations:**
- `trnid[2*i]` = crank site ID (`id` in outer scope), `trnid[2*i+1]` = slider site ID
- `cranklength` = rod length connecting crank to slider pin
- Slider axis = z-column of slider site's rotation matrix (row-major indices 2, 5, 8)
- Length formula: `length = av - sqrt(det)` where `det = av² + r² - v·v`
- Degenerate case (`det <= 0`): silent degradation — `length = av`, `dlda = vec`, `dldv = axis`. No warning issued.
- MuJoCo reuses `dlda` as a temp buffer when computing `dldv`, then overwrites `dlda`
- `mj_jacSite` on crank site: translational only (rotational Jacobian pointer = NULL/0)
- `mj_jacPointAxis` on slider site: returns both translational (`jacS`) and axis (`jacA`) Jacobians
- Jacobian subtraction: `jac = J_crank_trans - J_slider_trans`
- Gear scaling: `length *= gear[0]` after all computation; `moment *= gear[0]` during sparse storage. Gear is baked into the moment vector, so `velocity = moment.dot(&qvel)` requires no additional gear factor.

### `mj_jacPointAxis()` — `engine_core_util.c`

Computes both the translational point Jacobian and the axis Jacobian at a
given world point for a given body.

```c
void mj_jacPointAxis(const mjModel* m, mjData* d,
                     mjtNum* jacPoint, mjtNum* jacAxis,
                     const mjtNum point[3], const mjtNum axis[3], int body) {
    int nv = m->nv;
    mj_markStack(d);
    mjtNum* jacp = (jacPoint ? jacPoint : mjSTACKALLOC(d, 3*nv, mjtNum));
    mjtNum* jacr = mjSTACKALLOC(d, 3*nv, mjtNum);
    mj_jac(m, d, jacp, jacr, point, body);

    if (jacAxis) {
        for (int i = 0; i < nv; i++) {
            jacAxis[     i] = jacr[  nv+i]*axis[2] - jacr[2*nv+i]*axis[1];
            jacAxis[  nv+i] = jacr[2*nv+i]*axis[0] - jacr[     i]*axis[2];
            jacAxis[2*nv+i] = jacr[     i]*axis[1] - jacr[  nv+i]*axis[0];
        }
    }
    mj_freeStack(d);
}
```

Algorithm:
1. Call `mj_jac()` to get full-body Jacobian `(jacp, jacr)` at `point` for `body`
2. For each DOF column `i`, compute the axis Jacobian as: `jacAxis_col_i = cross(jacr_col_i, axis)`

The cross product order is `cross(jacr_col, axis)`, NOT `cross(axis, jacr_col)`.

### `mj_jacSite()` — `engine_core_util.c`

Thin wrapper around `mj_jac()`:

```c
void mj_jacSite(const mjModel* m, const mjData* d,
                mjtNum* jacp, mjtNum* jacr, int site) {
    mj_jac(m, d, jacp, jacr, d->site_xpos + 3*site, m->site_bodyid[site]);
}
```

Passes the site's world position and parent body ID to `mj_jac()`.

### MuJoCo Compiler Validation

From `user_objects.cc`:
- `cranklength > 0` is validated at compile time (`mjCError` if non-positive)
- `slidersite` is required when `cranksite` is specified
- `trnid[0]` = cranksite ID, `trnid[1]` = slidersite ID (confirmed from compiler assignment order)

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| `mjTRN_SLIDERCRANK` transmission | Full implementation: length, derivatives, Jacobian composition, moment via chain rule | **Not implemented** — no enum variant, no parsing, no runtime |
| `mjTRN_JOINTINPARENT` for hinge/slide | Identical to `mjTRN_JOINT` — scalar `gear[0]` | **Not implemented** — no enum variant, no parsing |
| `mjTRN_JOINTINPARENT` for ball/free | Gear rotation via inverse quaternion | **Not implemented** — also pre-existing gap in `mjTRN_JOINT` (`nv == 1` guard skips ball/free) |
| `mj_jacPointAxis` helper | `engine_core_util.c` — `cross(jacr_col, axis)` per DOF | **Not implemented** — `jacobian.rs` has `mj_jac` and `mj_jac_site` only |
| Slider-crank `det <= 0` singularity | Silent degradation: `length = av`, `dlda = vec`, `dldv = axis` | N/A |
| Slider-crank gear scaling | `length *= gear[0]` after derivatives; `moment *= gear[0]` during storage | N/A |
| `cranklength > 0` validation | Compiler error (`mjCError`) | N/A |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `actuator_trnid` indexing | Flat array: `[2*i]` / `[2*i+1]` | `Vec<[usize; 2]>`: `[i][0]` / `[i][1]` | Use `model.actuator_trnid[i][0]` wherever MuJoCo uses `m->actuator_trnid[2*i]` |
| `actuator_trnid` slot semantics (SliderCrank) | `trnid[0]` = cranksite ID, `trnid[1]` = slidersite ID (from `user_objects.cc`) | Same: `[i][0]` = cranksite, `[i][1]` = slidersite | Direct port — same semantic assignment |
| `actuator_gear` | Flat: `gear + 6*i`, only `gear[0]` used for SliderCrank | `Vec<[f64; 6]>`: `[i][0]` | Use `model.actuator_gear[i][0]` wherever MuJoCo uses `m->actuator_gear[6*i]` |
| `site_xmat` z-column | Row-major `9*id + k` indexing: z-col at indices `[2]`, `[5]`, `[8]` | `nalgebra::Matrix3` column access: `site_xmat[id].column(2)` | Use `data.site_xmat[id].column(2)` to extract z-axis |
| `actuator_moment` | Sparse CSR: `moment[rowadr[i] + j]`, `rownnz`, `colind` | Dense: `Vec<DVector<f64>>` — `actuator_moment[i][dof]` | Store directly into dense vector; no sparse compression needed |
| Jacobian layout | Row-major `3*nv`: row `k` at `[k*nv..k*nv+nv]` | `nalgebra::DMatrix` (column-major): `matrix[(row, col)]` | Use `matrix[(k, j)]` wherever MuJoCo uses `jac[k*nv + j]` |
| Quaternion convention | `(w, x, y, z)` | `nalgebra::UnitQuaternion` — `(w, x, y, z)` (same) | Direct port — no translation needed |
| Function naming | C camelCase: `mj_jacPointAxis`, `mj_jacSite`, `mj_transmission` | Rust snake_case: `mj_jac_point_axis`, `mj_jac_site`, `mj_transmission_slidercrank` | Spec uses C names for MuJoCo reference, Rust names for implementation |

---

## Specification

### S1. Enum variants — `enums.rs`

**File:** `core/src/types/enums.rs`
**MuJoCo equivalent:** `mjtTrn` enum values `mjTRN_SLIDERCRANK`, `mjTRN_JOINTINPARENT`
**Design decision:** Append two variants after `Body`, matching the umbrella
spec convention (§2). No `User` variant exists on `ActuatorTransmission`, so
new variants are simply appended. The `#[derive(Default)]` remains on `Joint`.

**After:**
```rust
/// Actuator transmission type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ActuatorTransmission {
    /// Direct joint actuation.
    #[default]
    Joint,
    /// Tendon actuation.
    Tendon,
    /// Site-based actuation.
    Site,
    /// Body (adhesion) actuation.
    Body,
    /// Slider-crank mechanism: crank site + slider site + rod.
    /// MuJoCo: `mjTRN_SLIDERCRANK`.
    SliderCrank,
    /// Joint transmission with force in parent frame.
    /// For hinge/slide joints, identical to `Joint`.
    /// MuJoCo: `mjTRN_JOINTINPARENT`.
    JointInParent,
}
```

### S2. Model field + init — `model.rs`, `model_init.rs`

**File:** `core/src/types/model.rs`
**MuJoCo equivalent:** `m->actuator_cranklength` array in `mjModel`
**Design decision:** Single new field `actuator_cranklength: Vec<f64>`.
SliderCrank actuators store the rod length here; all other types store `0.0`
(unused). This mirrors MuJoCo's flat array where only SliderCrank entries are
nonzero.

**model.rs — add after `actuator_actearly`:**
```rust
/// Crank rod length for slider-crank transmissions.
/// Only meaningful for `ActuatorTransmission::SliderCrank`.
/// MuJoCo reference: `m->actuator_cranklength[i]`.
pub actuator_cranklength: Vec<f64>,
```

**model_init.rs — add in `Model::empty()` actuator block:**
```rust
actuator_cranklength: vec![],
```

### S3. `mj_jac_point_axis` helper — `jacobian.rs`

**File:** `core/src/jacobian.rs`
**MuJoCo equivalent:** `mj_jacPointAxis()` in `engine_core_util.c`
**Design decision:** Returns `(jacp, jacr_axis)` as `(DMatrix<f64>, DMatrix<f64>)`,
both 3×nv. Calls `mj_jac()` internally, then computes the axis Jacobian
column-by-column via `cross(jacr_col, axis)`. This matches MuJoCo's algorithm
exactly. We return both matrices (not writing into caller-provided buffers)
to follow the existing `mj_jac` / `mj_jac_site` return-value pattern.

**After (new function, below `mj_jac_site`):**
```rust
/// Compute the translational point Jacobian and the axis Jacobian at a
/// world-frame point for a given body.
///
/// The axis Jacobian column `i` is `cross(jacr_col_i, axis)`, where `jacr`
/// is the rotational body Jacobian at `point`.
///
/// MuJoCo equivalent: `mj_jacPointAxis()` in `engine_core_util.c`.
///
/// Returns `(jac_point, jac_axis)` — both 3×nv dense matrices.
#[must_use]
pub fn mj_jac_point_axis(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
    axis: &Vector3<f64>,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let (jacp, jacr) = mj_jac(model, data, body_id, point);
    let mut jac_axis = DMatrix::zeros(3, model.nv);

    for i in 0..model.nv {
        // jacAxis_col_i = cross(jacr_col_i, axis)
        // MuJoCo: jacAxis[i] = jacr[nv+i]*axis[2] - jacr[2*nv+i]*axis[1]
        //         jacAxis[nv+i] = jacr[2*nv+i]*axis[0] - jacr[i]*axis[2]
        //         jacAxis[2*nv+i] = jacr[i]*axis[1] - jacr[nv+i]*axis[0]
        let r0 = jacr[(0, i)];
        let r1 = jacr[(1, i)];
        let r2 = jacr[(2, i)];
        jac_axis[(0, i)] = r1 * axis.z - r2 * axis.y;
        jac_axis[(1, i)] = r2 * axis.x - r0 * axis.z;
        jac_axis[(2, i)] = r0 * axis.y - r1 * axis.x;
    }

    (jacp, jac_axis)
}
```

### S4. MJCF parsing — `types.rs`, `parser.rs`, `builder/actuator.rs`

**Files:** `mjcf/src/types.rs`, `mjcf/src/parser.rs`, `mjcf/src/builder/actuator.rs`
**MuJoCo equivalent:** Actuator element parsing in `user_model.cc` / `user_objects.cc`
**Design decision:** Four new attributes parsed: `jointinparent` (site name),
`cranksite` (site name), `slidersite` (site name), `cranklength` (f64).
Match MuJoCo attribute names exactly (umbrella spec §4). `jointinparent`
maps to `ActuatorTransmission::JointInParent` with the same `trnid`
semantics as `joint`. `cranksite` + `slidersite` map to
`ActuatorTransmission::SliderCrank` with `trnid[0]` = cranksite,
`trnid[1]` = slidersite. Builder validates `cranklength > 0` (matching
MuJoCo compiler `mjCError`) and `slidersite` present when `cranksite`
specified.

**types.rs — add to `MjcfActuator` struct:**
```rust
/// Target joint name for parent-frame transmission.
/// Maps to `ActuatorTransmission::JointInParent`.
/// For hinge/slide joints, behaviorally identical to `joint`.
pub jointinparent: Option<String>,
/// Crank site name for slider-crank transmission.
/// When specified, `slidersite` must also be present.
pub cranksite: Option<String>,
/// Slider site name for slider-crank transmission.
/// Required when `cranksite` is specified.
pub slidersite: Option<String>,
/// Rod length connecting crank pin to slider pin (meters).
/// Required positive when slider-crank is used.
/// MuJoCo validates `cranklength > 0` at compile time.
pub cranklength: Option<f64>,
```

**parser.rs — parse the 4 new attributes in actuator element parsing:**
```rust
// In the actuator attribute parsing block:
"jointinparent" => actuator.jointinparent = Some(value.to_string()),
"cranksite" => actuator.cranksite = Some(value.to_string()),
"slidersite" => actuator.slidersite = Some(value.to_string()),
"cranklength" => actuator.cranklength = Some(value.parse::<f64>().map_err(/* ... */)?),
```

**builder/actuator.rs — transmission resolution, add before the `else` error:**

```rust
} else if let Some(ref joint_name) = actuator.jointinparent {
    let jnt_id =
        self.joint_name_to_id
            .get(joint_name)
            .ok_or_else(|| ModelConversionError {
                message: format!(
                    "Actuator '{}' references unknown joint: {joint_name}",
                    actuator.name
                ),
            })?;
    (ActuatorTransmission::JointInParent, [*jnt_id, usize::MAX])
} else if let Some(ref crank_name) = actuator.cranksite {
    let slider_name = actuator.slidersite.as_ref().ok_or_else(|| {
        ModelConversionError {
            message: format!(
                "Actuator '{}': cranksite specified without slidersite",
                actuator.name
            ),
        }
    })?;
    let crank_id = *self
        .site_name_to_id
        .get(crank_name.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!(
                "Actuator '{}': cranksite '{}' not found",
                actuator.name, crank_name
            ),
        })?;
    let slider_id = *self
        .site_name_to_id
        .get(slider_name.as_str())
        .ok_or_else(|| ModelConversionError {
            message: format!(
                "Actuator '{}': slidersite '{}' not found",
                actuator.name, slider_name
            ),
        })?;
    let rod = actuator.cranklength.unwrap_or(0.0);
    if rod <= 0.0 {
        return Err(ModelConversionError {
            message: format!(
                "Actuator '{}': cranklength must be positive, got {}",
                actuator.name, rod
            ),
        });
    }
    (ActuatorTransmission::SliderCrank, [crank_id, slider_id])
}
```

**builder/actuator.rs — push `cranklength` after existing pushes:**
```rust
// After self.actuator_actearly.push(...)
self.actuator_cranklength.push(
    actuator.cranklength.unwrap_or(0.0),
);
```

### S5. SliderCrank transmission dispatch — `actuation.rs`

**File:** `core/src/forward/actuation.rs`
**MuJoCo equivalent:** `mjTRN_SLIDERCRANK` case in `mj_transmission()`,
`engine_core_smooth.c`
**Design decision:** New standalone function `mj_transmission_slidercrank()`
following the existing pattern of `mj_transmission_site()` and
`mj_transmission_body()`. Called from the same dispatch point in
`forward_core()`. Computes length, derivatives, Jacobian composition, and
moment in a single pass per actuator. We compute dldv and dlda independently
(not reusing dlda as temp buffer) for clarity — the numerical result is
identical.

**After (new function + dispatch):**
```rust
use crate::jacobian::{mj_jac_point_axis, mj_jac_site};

/// Compute actuator length and moment for slider-crank transmissions.
///
/// For each SliderCrank-transmission actuator:
/// 1. Extract slider axis (z-column of slider site rotation matrix)
/// 2. Compute vec = cranksite_pos - slidersite_pos
/// 3. Compute length = av - sqrt(det), with degenerate fallback
/// 4. Compute derivatives dlda, dldv via chain rule
/// 5. Compute Jacobians: mj_jac_site on crank, mj_jac_point_axis on slider
/// 6. Compose moment via chain rule
/// 7. Scale length and moment by gear[0]
///
/// Must run after `mj_fwd_position` (needs site_xpos, site_xmat, FK results).
/// MuJoCo equivalent: `mjTRN_SLIDERCRANK` case in `mj_transmission()`.
pub fn mj_transmission_slidercrank(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] != ActuatorTransmission::SliderCrank {
            continue;
        }

        let crank_id = model.actuator_trnid[i][0];   // trnid[0] = cranksite
        let slider_id = model.actuator_trnid[i][1];  // trnid[1] = slidersite
        let rod = model.actuator_cranklength[i];
        let gear0 = model.actuator_gear[i][0];

        // Slider axis: z-column of slider site rotation matrix
        let axis = data.site_xmat[slider_id].column(2).into_owned();

        // Vector from slider site to crank site
        let vec3 = data.site_xpos[crank_id] - data.site_xpos[slider_id];

        // Length and determinant
        let av = vec3.dot(&axis);
        let det = av * av + rod * rod - vec3.dot(&vec3);

        let (length, dlda, dldv);
        if det <= 0.0 {
            // Degenerate: silent degradation (matches MuJoCo)
            length = av;
            dlda = vec3;
            dldv = axis.clone_owned();
        } else {
            let sdet = det.sqrt();
            length = av - sdet;

            let factor = 1.0 - av / sdet;
            // dldv = axis * (1 - av/sdet) + vec / sdet
            dldv = axis.scale(factor) + vec3.scale(1.0 / sdet);
            // dlda = vec * (1 - av/sdet)
            dlda = vec3.scale(factor);
        };

        // Scale length by gear
        data.actuator_length[i] = length * gear0;

        // Jacobians:
        // mj_jac_point_axis on slider site → (jac_slider_point, jac_axis)
        let slider_body = model.site_body[slider_id];
        let (jac_slider_point, jac_axis) = mj_jac_point_axis(
            model,
            data,
            slider_body,
            &data.site_xpos[slider_id],
            &axis,
        );

        // mj_jac_site on crank site → (jac_crank_point, _)
        // Translational only — rotational Jacobian not used
        let (jac_crank_point, _) = mj_jac_site(model, data, crank_id);

        // jac_vec = J_crank_point - J_slider_point
        let jac_vec = &jac_crank_point - &jac_slider_point;

        // Chain rule: moment[j] = sum_k(dlda[k]*jacA[k,j] + dldv[k]*jac_vec[k,j])
        let moment = &mut data.actuator_moment[i];
        for j in 0..model.nv {
            let mut m = 0.0;
            for k in 0..3 {
                m += dlda[k] * jac_axis[(k, j)] + dldv[k] * jac_vec[(k, j)];
            }
            // Gear scaling baked into moment
            moment[j] = m * gear0;
        }
    }
}
```

**Dispatch — add call in `forward_core()` alongside existing transmission dispatches:**
```rust
// After mj_transmission_site(model, data):
mj_transmission_slidercrank(model, data);
```

### S6. JointInParent + SliderCrank pipeline arms — `actuation.rs`

**File:** `core/src/forward/actuation.rs`
**MuJoCo equivalent:** `mjTRN_JOINTINPARENT` combined case in `mj_transmission()`;
`mjTRN_SLIDERCRANK` velocity/force paths in `mj_transmission()`
**Design decision:** For hinge/slide joints (nv == 1), `JointInParent` is
behaviorally identical to `Joint`. We widen the existing `Joint` match arms.
For `SliderCrank`, velocity and force use the cached moment vector (same
pattern as `Site`/`Body`), so we widen those arms. This covers all three
pipeline stages (position, velocity, force) for both new types.

**mj_actuator_length — widen Joint arm + widen Site|Body arm:**

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

**Before:**
```rust
ActuatorTransmission::Site => {
    // Length already set by mj_transmission_site (position stage).
    // Velocity from cached moment:
    data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
}
ActuatorTransmission::Body => {
    // Length already set to 0 by mj_transmission_body (position stage).
    // Velocity from cached moment (same as Site):
    data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
}
```
→
```rust
ActuatorTransmission::Site | ActuatorTransmission::SliderCrank => {
    // Length already set by mj_transmission_site / mj_transmission_slidercrank.
    // Velocity from cached moment:
    data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
}
ActuatorTransmission::Body => {
    // Length already set to 0 by mj_transmission_body (position stage).
    // Velocity from cached moment (same as Site):
    data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
}
```

**mj_fwd_actuation Phase 3 — widen Joint arm + widen Site|Body arm:**

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

**Before:**
```rust
ActuatorTransmission::Site | ActuatorTransmission::Body => {
    // Use cached moment vector from transmission function.
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```
→
```rust
ActuatorTransmission::Site | ActuatorTransmission::Body | ActuatorTransmission::SliderCrank => {
    // Use cached moment vector from transmission function.
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```

### S7. Match arm updates — 8 exhaustive match sites

Adding two new `ActuatorTransmission` variants breaks every exhaustive
`match` on the enum. There are 8 production match sites across 7 files
(7 sim-core, 1 sim-mjcf). Each needs explicit arms for `SliderCrank`
and `JointInParent`.

**S7a. `muscle.rs` line ~160 — `compute_actuator_params` Phase 1 lengthrange:**

`JointInParent` → widen the `Joint` arm (same semantics: scalar joint length).
`SliderCrank` → add arm alongside `Site | Body`: no-op (configuration-dependent
length, leave at `(0, 0)`).

**Before:**
```rust
ActuatorTransmission::Site | ActuatorTransmission::Body => {
    // Site: configuration-dependent (full FK required).
    // Body: no length concept. Both: no-op, leave at (0, 0).
}
```

**After:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

```rust
ActuatorTransmission::Site | ActuatorTransmission::Body | ActuatorTransmission::SliderCrank => {
    // Site/SliderCrank: configuration-dependent (full FK required).
    // Body: no length concept. All: no-op, leave at (0, 0).
}
```

**S7b. `muscle.rs` line ~344 — `build_actuator_moment`:**

`JointInParent` → widen the `Joint` arm (same moment: scalar gear at dof_adr).
`SliderCrank` → add new arm that computes moment using the slider-crank
geometry at `qpos0`. This mirrors the runtime dispatch but operates on the
build-time FK data.

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

**Add SliderCrank arm (after Body). Requires adding import at top of `muscle.rs`:**
```rust
use crate::jacobian::mj_jac_point_axis;
```

```rust
ActuatorTransmission::SliderCrank => {
    let crank_id = model.actuator_trnid[actuator_idx][0];
    let slider_id = model.actuator_trnid[actuator_idx][1];
    let rod = model.actuator_cranklength[actuator_idx];

    let axis: Vector3<f64> = data.site_xmat[slider_id].column(2).into_owned();
    let vec3 = data.site_xpos[crank_id] - data.site_xpos[slider_id];
    let av = vec3.dot(&axis);
    let det = av * av + rod * rod - vec3.dot(&vec3);

    let (dlda, dldv);
    if det <= 0.0 {
        dlda = vec3;
        dldv = axis.clone_owned();
    } else {
        let sdet = det.sqrt();
        let factor = 1.0 - av / sdet;
        dldv = axis.scale(factor) + vec3.scale(1.0 / sdet);
        dlda = vec3.scale(factor);
    }

    let slider_body = model.site_body[slider_id];
    let (jac_slider_point, jac_axis) = mj_jac_point_axis(
        model, data, slider_body, &data.site_xpos[slider_id], &axis,
    );
    let (jac_crank_point, _) = mj_jac_site(model, data, crank_id);
    let jac_vec = &jac_crank_point - &jac_slider_point;

    for j in 0..model.nv {
        let mut m = 0.0;
        for k in 0..3 {
            m += dlda[k] * jac_axis[(k, j)] + dldv[k] * jac_vec[(k, j)];
        }
        j_vec[j] = m * gear;
    }
}
```

**S7c. `muscle.rs` line ~509 — `mj_set_length_range` Step 3 uselimit:**

**CRITICAL:** This match site has a dangerous catch-all `_ => {}` that
silently skips new transmission types. Must be replaced with explicit arms.

**Before:**
```rust
_ => {} // Site/Body: no limits to copy
```

**After:**
```rust
ActuatorTransmission::Site
| ActuatorTransmission::Body
| ActuatorTransmission::SliderCrank => {
    // Site/SliderCrank: configuration-dependent, no static limits to copy.
    // Body: no length concept. Skip.
}
ActuatorTransmission::JointInParent => {
    let jid = model.actuator_trnid[i][0];
    if jid < model.njnt && model.jnt_limited[jid] {
        let (jlo, jhi) = model.jnt_range[jid];
        model.actuator_lengthrange[i] = scale(jlo, jhi);
        continue;
    }
}
```

**S7d. `derivatives.rs` line ~563 — `mjd_actuator_vel`:**

`JointInParent` → widen the `Joint` arm.
`SliderCrank` → add arm alongside `Site | Body` (uses cached moment vector).

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

```rust
ActuatorTransmission::Site | ActuatorTransmission::Body => {
```
→
```rust
ActuatorTransmission::Site | ActuatorTransmission::Body | ActuatorTransmission::SliderCrank => {
```

**S7e. `sensor/position.rs` line ~282 — `mj_sensor_pos` ActuatorPos:**

`JointInParent` → widen the `Joint` arm.
`SliderCrank` → add arm alongside `Site | Body` (reads cached `actuator_length`).

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

```rust
ActuatorTransmission::Site | ActuatorTransmission::Body => {
```
→
```rust
ActuatorTransmission::Site | ActuatorTransmission::Body | ActuatorTransmission::SliderCrank => {
```

**S7f. `builder/build.rs` line ~565 — sleep policy resolution:**

`JointInParent` → widen the `Joint` arm (same: look up joint body → tree).
`SliderCrank` → new arm: mark BOTH crank site and slider site trees as
`AutoNever`. MuJoCo's runtime checks both site bodies for wakefulness.

**Before:**
```rust
ActuatorTransmission::Joint => {
```
→
```rust
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
```

**Add SliderCrank arm (before the `if let Some(bid) = body_id` line):**
```rust
ActuatorTransmission::SliderCrank => {
    // Mark both crank and slider site trees as AutoNever
    let crank_id = trnid[0];
    let slider_id = trnid[1];
    for &sid in &[crank_id, slider_id] {
        if sid < model.nsite {
            let bid = model.site_body[sid];
            if bid > 0 {
                let tree = model.body_treeid[bid];
                if tree < model.ntree {
                    model.tree_sleep_policy[tree] = SleepPolicy::AutoNever;
                }
            }
        }
    }
    None // body_id not used below — trees already marked
}
```

---

## Acceptance Criteria

### AC1: SliderCrank length — colinear geometry *(runtime test, analytically derived)*
**Given:** Model with single hinge joint (Y-axis), body at origin. Crank site
on body at (0, 0, 1.0), slider site on world body at (0, 0, 0) with rotation
= I₃. `cranklength = 0.5`, `gear = [2.0]`. `qpos = [0.0]`.
**After:** `mj_transmission_slidercrank()`
**Assert:** `actuator_length[0]` = `(1.0 - 0.5) * 2.0` = `1.0 ± 1e-12`
**Field:** `Data.actuator_length`
**Derivation:** `vec = (0,0,1)-(0,0,0) = (0,0,1)`, `axis = (0,0,1)`,
`av = 1.0`, `det = 1 + 0.25 - 1 = 0.25`, `sdet = 0.5`,
`length = 1.0 - 0.5 = 0.5`, scaled by gear: `0.5 * 2.0 = 1.0`.

### AC2: SliderCrank moment — offset geometry *(runtime test, analytically derived)*
**Given:** Model with single hinge joint (Y-axis) at body origin. Body at
origin. Crank site at body-local (0.5, 0, 0.5). Slider site on world body
at (0, 0, 0) with rotation = I₃. `cranklength = 0.6`, `gear = [1.0]`.
`qpos = [0.0]`.
**After:** `mj_transmission_slidercrank()`
**Assert:** `actuator_moment[0][dof_0]` ≈ 0.25378 ± 1e-4.
**Field:** `Data.actuator_moment`
**Derivation (full worked example):**
1. `vec = (0.5, 0, 0.5) - (0, 0, 0) = (0.5, 0, 0.5)`, `axis = (0, 0, 1)`
2. `av = 0.5`, `v·v = 0.5`, `det = 0.25 + 0.36 - 0.5 = 0.11`, `sdet = √0.11 ≈ 0.33166`
3. `length = 0.5 - 0.33166 ≈ 0.16834`
4. `factor = 1 - 0.5/0.33166 = -0.50755`
5. `dldv = (0,0,1)·(-0.50755) + (0.5,0,0.5)/0.33166 = (1.50755, 0, 1.0)`
6. `dlda = (0.5,0,0.5)·(-0.50755) = (-0.25378, 0, -0.25378)`
7. Hinge about Y at origin, crank at (0.5, 0, 0.5):
   `jacp_col = (0,1,0) × (0.5,0,0.5) = (0.5, 0, -0.5)`, `jacr_col = (0,1,0)`
8. Slider on world body: all Jacobians = 0. `jac_axis = 0`, `jac_vec = jacp_crank - 0 = (0.5, 0, -0.5)`
9. `moment[0] = dlda·jacA_col + dldv·jac_vec_col = 0 + (1.50755·0.5 + 0 + 1.0·(-0.5))
   = 0.75378 - 0.5 = 0.25378`
10. With `gear = 1.0`: `moment[0] = 0.25378`

### AC3: SliderCrank velocity from cached moment *(runtime test, analytically derived)*
**Given:** Same model as AC2. `qvel = [1.0]` (hinge velocity).
**After:** `mj_actuator_length()` (velocity stage)
**Assert:** `actuator_velocity[0]` = `actuator_moment[0].dot(&qvel)` ≈ 0.25378 ± 1e-4
(no additional gear factor — gear is baked into moment).
**Field:** `Data.actuator_velocity`

### AC4: SliderCrank force application *(runtime test, analytically derived)*
**Given:** Model with SliderCrank motor, `ctrl = [1.0]`, `gainprm[0] = 1.0`,
`gear = [1.0]`.
**After:** `mj_step()`
**Assert:** `qfrc_actuator` receives force via cached moment: `qfrc_actuator[dof]
+= moment[dof] * force`. Value is nonzero.
**Field:** `Data.qfrc_actuator`

### AC5: SliderCrank degenerate case *(runtime test, analytically derived)*
**Given:** Slider site at (0, 0, 0) with axis = (0, 0, 1). Crank site at
(1, 0, 0). `cranklength = 0.5`. `gear = [1.0]`.
**After:** `mj_transmission_slidercrank()`
**Assert:** `det = 0 + 0.25 - 1.0 = -0.75 ≤ 0`. Degenerate path:
`actuator_length[0]` = `av * gear = 0.0 * 1.0 = 0.0 ± 1e-12`. No panic.
**Field:** `Data.actuator_length`
**Derivation:** `vec = (1,0,0)`, `av = dot((1,0,0), (0,0,1)) = 0`.

### AC6: JointInParent hinge == Joint identity *(runtime test, analytically derived)*
**Given:** Two identical models, one with `joint="hinge"` (Joint type) and
one with `jointinparent="hinge"` (JointInParent type). Same gear, same
`ctrl`. Hinge joint.
**After:** `mj_step()`
**Assert:** `actuator_length`, `actuator_velocity`, `qfrc_actuator` are
identical between the two models (bitwise, ± 0.0).
**Field:** `Data.actuator_length`, `Data.actuator_velocity`, `Data.qfrc_actuator`

### AC7: JointInParent slide == Joint identity *(runtime test, analytically derived)*
**Given:** Same as AC6 but with a slide joint.
**After:** `mj_step()`
**Assert:** All actuator fields identical between Joint and JointInParent.
**Field:** Same as AC6.

### AC8: MJCF parsing — `jointinparent` round-trip *(runtime test)*
**Given:** MJCF with `<general jointinparent="hinge_joint" gear="5"/>`.
**After:** `load_model()`
**Assert:** `model.actuator_trntype[0]` = `ActuatorTransmission::JointInParent`,
`model.actuator_trnid[0][0]` = joint index for `"hinge_joint"`,
`model.actuator_gear[0][0]` = `5.0`.
**Field:** `Model.actuator_trntype`, `Model.actuator_trnid`, `Model.actuator_gear`

### AC9: MJCF parsing — SliderCrank round-trip *(runtime test)*
**Given:** MJCF with `<general cranksite="crank" slidersite="slider" cranklength="0.5" gear="2"/>`.
**After:** `load_model()`
**Assert:** `model.actuator_trntype[0]` = `ActuatorTransmission::SliderCrank`,
`model.actuator_trnid[0][0]` = site index for `"crank"`,
`model.actuator_trnid[0][1]` = site index for `"slider"`,
`model.actuator_cranklength[0]` = `0.5`,
`model.actuator_gear[0][0]` = `2.0`.
**Field:** `Model.actuator_trntype`, `Model.actuator_trnid`, `Model.actuator_cranklength`

### AC10: MJCF error — cranksite without slidersite *(runtime test)*
**Given:** MJCF with `<general cranksite="crank" cranklength="0.5"/>` (missing `slidersite`).
**After:** `load_model()`
**Assert:** Returns `Err` containing `"slidersite"`.
**Field:** Error message.

### AC11: MJCF error — non-positive cranklength *(runtime test)*
**Given:** MJCF with `<general cranksite="crank" slidersite="slider" cranklength="0"/>`.
**After:** `load_model()`
**Assert:** Returns `Err` containing `"cranklength"`.
**Field:** Error message.

### AC12: Exhaustive match completeness *(code review)*
All 8 exhaustive match sites on `ActuatorTransmission` have explicit arms for
`SliderCrank` and `JointInParent`. No wildcard `_` catch-all remains at
`muscle.rs` line 509. Verified by: compilation succeeds (Rust exhaustive
match guarantee), plus manual review of the 8 sites listed in S7.

### AC13: `mj_jac_point_axis` correctness *(code review + runtime test)*
The helper produces numerically identical results to computing
`cross(jacr_col, axis)` manually for a known 1-DOF hinge model.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (SliderCrank length) | T1 | Direct |
| AC2 (SliderCrank moment) | T1, T2 | Direct |
| AC3 (SliderCrank velocity) | T2 | Direct |
| AC4 (SliderCrank force) | T3 | Direct |
| AC5 (SliderCrank degenerate) | T4 | Edge case |
| AC6 (JointInParent hinge == Joint) | T5 | Direct |
| AC7 (JointInParent slide == Joint) | T6 | Direct |
| AC8 (MJCF jointinparent) | T7 | Direct |
| AC9 (MJCF SliderCrank) | T8 | Direct |
| AC10 (MJCF error: missing slidersite) | T9 | Direct |
| AC11 (MJCF error: non-positive cranklength) | T10 | Direct |
| AC12 (match completeness) | — | Code review (manual) |
| AC13 (jac_point_axis) | T11 | Direct |

---

## Test Plan

All tests go in `sim/L0/tests/integration/actuator_phase5.rs` (extending
the existing Phase 5 test file) and/or a new
`sim/L0/tests/integration/transmission_specb.rs`.

### T1: SliderCrank length + moment — colinear geometry → AC1, AC2

**Model:** Single hinge joint (Y-axis) at origin. Body at (0, 0, 0). Crank
site at body-local (0, 0, 1.0). Slider site on world body at (0, 0, 0)
with default rotation (axis = z). `cranklength = 0.5`, `gear = [2.0]`.

```xml
<mujoco>
  <worldbody>
    <site name="slider" pos="0 0 0"/>
    <body pos="0 0 0">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <site name="crank" pos="0 0 1"/>
    </body>
  </worldbody>
  <actuator>
    <general cranksite="crank" slidersite="slider" cranklength="0.5" gear="2"/>
  </actuator>
</mujoco>
```

**Assert at qpos=0:**
- `actuator_length[0]` = 1.0 ± 1e-10
- `actuator_moment[0]` has nonzero entry at the hinge DOF

**Expected value derivation (analytical):**
`vec = (0,0,1)`, `axis = (0,0,1)`, `av = 1.0`, `det = 0.25`, `sdet = 0.5`,
`length = 0.5`, `length * gear = 1.0`.

### T2: SliderCrank moment + velocity — offset geometry → AC2, AC3

**Model:** Single hinge joint (Y-axis) at origin. Crank site at body-local
(0.5, 0, 0.5). Slider site on world body at (0, 0, 0). `cranklength = 0.6`,
`gear = [1.0]`.

```xml
<mujoco>
  <worldbody>
    <site name="slider" pos="0 0 0"/>
    <body pos="0 0 0">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <site name="crank" pos="0.5 0 0.5"/>
    </body>
  </worldbody>
  <actuator>
    <general cranksite="crank" slidersite="slider" cranklength="0.6" gear="1"/>
  </actuator>
</mujoco>
```

**Assert at qpos=0:**
- `actuator_length[0]` ≈ 0.16834 ± 1e-4
- `actuator_moment[0][dof_0]` ≈ 0.25378 ± 1e-4

**Assert with qvel=[1.0]:**
- `actuator_velocity[0]` = `actuator_moment[0].dot(&qvel)` ≈ 0.25378 ± 1e-4
  (no additional gear factor — gear baked into moment)

### T3: SliderCrank end-to-end force application → AC4

**Model:** Same as T1 but with `gear = [1.0]`, `gainprm = [1.0]` (motor-like).
Set `ctrl = [1.0]`.
**After:** `mj_step()`
**Assert:** `qfrc_actuator[dof_0]` is nonzero. Direction and magnitude
consistent with moment arm and unit force.

### T4: SliderCrank degenerate (det ≤ 0) — no panic → AC5

**Model:** Slider site at (0, 0, 0), crank site at (1, 0, 0) (orthogonal
to slider axis). `cranklength = 0.5`, `gear = [1.0]`.

```xml
<mujoco>
  <worldbody>
    <site name="slider" pos="0 0 0"/>
    <body pos="1 0 0">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <site name="crank" pos="0 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <general cranksite="crank" slidersite="slider" cranklength="0.5" gear="1"/>
  </actuator>
</mujoco>
```

**Assert:** `actuator_length[0]` = 0.0 ± 1e-10 (degenerate path: `av = 0`,
`length = av`). No panic, no NaN.

### T5: JointInParent hinge identity → AC6

**Model A:** `<motor joint="hinge" gear="5"/>`
**Model B:** `<general jointinparent="hinge" gear="5"/>`
Same body, joint, inertia, `ctrl = [1.0]`.

**After:** `mj_step()` on both.
**Assert:** `data_a.actuator_length[0] == data_b.actuator_length[0]`,
`data_a.actuator_velocity[0] == data_b.actuator_velocity[0]`,
`data_a.qfrc_actuator == data_b.qfrc_actuator` (bitwise equal).

### T6: JointInParent slide identity → AC7

Same as T5 but with `type="slide"`.

### T7: MJCF parsing — jointinparent → AC8

```xml
<general name="a" jointinparent="j" gear="5"/>
```

**Assert:** `model.actuator_trntype[0] == JointInParent`,
`model.actuator_trnid[0][0]` = joint index of `"j"`,
`model.actuator_gear[0][0]` = 5.0.

### T8: MJCF parsing — SliderCrank → AC9

```xml
<general name="a" cranksite="c" slidersite="s" cranklength="0.5" gear="2"/>
```

**Assert:** `model.actuator_trntype[0] == SliderCrank`,
`model.actuator_trnid[0][0]` = site index of `"c"`,
`model.actuator_trnid[0][1]` = site index of `"s"`,
`model.actuator_cranklength[0]` = 0.5,
`model.actuator_gear[0][0]` = 2.0.

### T9: MJCF error — cranksite without slidersite → AC10

```xml
<general cranksite="c" cranklength="0.5"/>
```

**Assert:** `load_model()` returns Err containing `"slidersite"`.

### T10: MJCF error — non-positive cranklength → AC11

```xml
<general cranksite="c" slidersite="s" cranklength="0"/>
```

**Assert:** `load_model()` returns Err containing `"cranklength"`.

### T11: `mj_jac_point_axis` correctness → AC13

**Model:** Single hinge joint (Y-axis). Body at origin.
**Setup:** Call `mj_jac_point_axis` at a known point with known axis.
**Assert:** The returned axis Jacobian column equals
`cross(jacr_col, axis)` computed manually from the rotational Jacobian.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| SliderCrank `det ≤ 0` (crank orthogonal to slider axis) | Singularity: sqrt of negative number would panic without guard | T4 | AC5 |
| JointInParent hinge == Joint identity | Behavioral equivalence must hold exactly | T5 | AC6 |
| JointInParent slide == Joint identity | Same for slide joints | T6 | AC7 |
| `cranklength = 0` (non-positive) | MuJoCo compiler rejects this; we must too | T10 | AC11 |
| Missing `slidersite` with `cranksite` present | MuJoCo requires both; partial spec is invalid | T9 | AC10 |
| SliderCrank with gear != 1 | Gear scaling affects length and moment differently | T1 (gear=2) | AC1, AC2 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T2 (velocity) | Velocity = moment.dot(qvel) with no gear factor | Verifies gear-baked-into-moment convention — easy to double-scale |
| T11 (jac_point_axis) | Standalone helper correctness | New function not covered by any AC except AC13; needs dedicated unit test |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| 2 new `ActuatorTransmission` variants | 4 variants: Joint, Tendon, Site, Body | 6 variants: + SliderCrank, JointInParent | Toward MuJoCo | All exhaustive `match` sites on `ActuatorTransmission` | Add arms — compile errors guide |
| New `mj_jac_point_axis` function | Did not exist | Public helper in `jacobian.rs` | Toward MuJoCo | None — additive only | None — transparent addition |
| New `actuator_cranklength` field | Did not exist | New `Vec<f64>` in Model | Toward MuJoCo | Model builders / serializers | Add field to any code constructing Model |
| `_ => {}` catch-all at muscle.rs:509 | Silently skips unknown variants | Explicit arms for all 6 variants | Correctness improvement | `mj_set_length_range` callers | None — behavioral fix |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/types/enums.rs` | +2 enum variants | +8 |
| `core/src/types/model.rs` | +1 field `actuator_cranklength` | +4 |
| `core/src/types/model_init.rs` | +1 init line | +1 |
| `core/src/jacobian.rs` | +`mj_jac_point_axis` function | +30 |
| `core/src/forward/actuation.rs` | +`mj_transmission_slidercrank` + widen 2 Joint arms + add SliderCrank arms | +80 |
| `core/src/forward/muscle.rs` | +arms in 3 match sites (lines ~160, ~344, ~509); replace `_ => {}` catch-all | +40 |
| `core/src/derivatives.rs` | +widen 2 match arms (line ~563) | +4 |
| `core/src/sensor/position.rs` | +widen 2 match arms (line ~282) | +4 |
| `mjcf/src/types.rs` | +4 struct fields on `MjcfActuator` | +12 |
| `mjcf/src/parser.rs` | +4 attribute parsing cases | +8 |
| `mjcf/src/builder/actuator.rs` | +transmission resolution for JointInParent + SliderCrank | +45 |
| `mjcf/src/builder/build.rs` | +2 match arms in sleep policy (line ~565) | +15 |
| `tests/integration/transmission_specb.rs` | New test file | +250 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| Phase 4 regression suite (39 tests) | `conformance/phase4.rs` | Pass (unchanged) | No existing code paths modified — extension only |
| Spec A tests (T7-T12) | `actuator_phase5.rs` | Pass (unchanged) | New variants added to `compute_actuator_params` match arms; existing behavior for Joint/Tendon/Site/Body untouched |
| `test_site_actuator` | `builder/actuator.rs` | Pass (unchanged) | Site transmission parsing path not modified |
| `test_dampratio_mjcf_roundtrip` | `actuator_phase5.rs` | Pass (unchanged) | dampratio for Joint transmission unaffected |
| `test_acc0_motor_via_mjcf` | `actuator_phase5.rs` | Pass (unchanged) | Motor acc0 computation unaffected |
| Full sim domain baseline (~2,148+ tests) | Various | All pass | Extension-only change; no existing behavior altered |

---

## Execution Order

1. **S1 (enums.rs)** — Add `SliderCrank`, `JointInParent` variants.
   This intentionally breaks compilation at all exhaustive match sites.
   → Verify: `cargo build` fails with expected match errors at 8 sites.

2. **S2 (model.rs, model_init.rs)** — Add `actuator_cranklength` field.
   → Verify: compilation still fails (match errors) but no new errors from field.

3. **S3 (jacobian.rs)** — Add `mj_jac_point_axis` helper.
   → Verify: new function compiles, unit test T11 passes.

4. **S4 (types.rs, parser.rs, builder/actuator.rs)** — MJCF parsing.
   → Verify: T7, T8, T9, T10 pass. `cargo build` for sim-mjcf succeeds.

5. **S7 (all match arm updates)** — Fix all 8 exhaustive match sites.
   → Verify: `cargo build` succeeds (all match errors resolved).
   **Critical:** S7c replaces the `_ => {}` catch-all at muscle.rs:509.

6. **S6 (actuation.rs)** — Widen Joint arms for JointInParent.
   → Verify: T5, T6 pass (identity with Joint).

7. **S5 (actuation.rs)** — Add `mj_transmission_slidercrank`.
   → Verify: T1, T2, T3, T4 pass.

8. **Final gate:** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-constraint -p sim-muscle -p sim-tendon -p sim-sensor -p sim-urdf -p sim-types -p sim-simd` — all tests pass including new ones. `cargo xtask check` passes.

---

## Out of Scope

- **Ball/free joint transmission (both Joint and JointInParent):** MuJoCo's
  `mj_transmission()` has full ball (3-DOF) and free (6-DOF) joint handling
  with quaternion gear rotation for `mjTRN_JOINTINPARENT`. This is a
  **pre-existing gap** in the base `Joint` type: `mj_actuator_length()` at
  `actuation.rs:252` has an `if nv == 1` guard that silently skips ball/free
  joints for ALL joint transmissions (not just `JointInParent`). Fixing this
  for `JointInParent` alone would be inconsistent — it would mean
  `JointInParent` supports ball/free while `Joint` does not. The correct fix
  is to implement ball/free joint transmission for BOTH types together in a
  future spec. Conformance impact: gap acceptable for v1.0 — ball/free joint
  actuation is uncommon in standard models.

- **`mjTRN_TENDON` and `mjTRN_SITE` improvements:** Already implemented and
  conformant. No changes needed.

- **Sparse moment compression:** MuJoCo stores `actuator_moment` in sparse
  CSR. CortenForge uses dense `Vec<DVector<f64>>`. Converting to sparse is a
  performance optimization, not a conformance issue — dense and sparse produce
  identical numerical results. Deferred to a performance phase.
