# Future Work 10j — Deferred Item Tracker: Group 9 — Misc Pipeline & API

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 9 — Misc Pipeline & API (16 items)

**Spec approach:** ~~DT-74/75 need individual specs (T3 — Jacobian correctness bugs,
need formula derivation)~~ **DONE** — both specced and implemented.
DT-79/82/83 each need individual specs (T3 — API design
or data layout architecture). DT-77 shares a "Length-Range Estimation" spec with
DT-59 (T2). The rest (DT-76/80/81/84/91/92) implement directly (T1). Totals:
6 T1, 1 T2, 3 T3 remaining (2 T3 + 1 T2 done).

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| ~~DT-74~~ | §4 | ~~`compute_body_jacobian_at_point()` incomplete — only x-component, `#[allow(dead_code)]`~~ **DONE** — canonical `mj_jac` API landed, dead code deleted, 14 tests | Medium | T3 |
| ~~DT-75~~ | §4 | ~~`add_body_jacobian` free joint bug — world-frame unit vectors instead of body-frame `R*e_i`~~ **DONE** — body-frame axes fix in 3 locations, 6 tests | Medium | T3 |
| DT-76 | §8 | Pre-allocated `efc_lambda_saved` for RK4 — avoid `efc_lambda.clone()` per step | Low | T1 |
| DT-77 | §5 | Length-range auto-estimation for site-transmission muscle actuators (no-op stub) | Low | T2 |
| ~~DT-78~~ | §4 | ~~`actuator_lengthrange` for unlimited spatial tendons — wrap-array DOF lookup wrong~~ **DONE** — spatial tendon guard skips DOF lookup, logs warning; landed in §4 step 5 (spatial tendon impl) | Low | T2 |
| ~~DT-79~~ | §14 | ~~User callbacks `mjcb_*` Rust equivalents — closures vs trait objects, thread safety~~ **Done** — `Callback<F>` Arc wrapper, cb_passive/cb_control/cb_contactfilter, 5 tests | Medium | T3 |
| DT-80 | §14 | Mocap body + equality weld constraint integration testing | Low | T1 |
| DT-81 | §14 | `key_userdata` support — no `userdata` concept in CortenForge | Low | T1 |
| DT-82 | §9 | SoA layout across environments for cache locality — deferred to GPU work | Low | T3 |
| DT-83 | §9 | Multi-model batching — different robots in same batch (per-env dimensions) | Low | T3 |
| DT-84 | §32 | `mju_encodePyramid` utility not implemented — API compatibility only | Low | T1 |
| DT-91 | §2 | Warmstart `Vec<f64>` → `SmallVec<[f64; 6]>` — avoid heap allocation in warmstart vectors | Low | T1 |
| DT-92 | §9 | Parallel reset for `BatchSim` — sequential O(nq+nv+nu+na) reset deferred | Low | T1 |
| ~~DT-93~~ | §41 | ~~Auto-reset on NaN/divergence~~ **Subsumed by §41 S8** | Medium | T2 |
| DT-96 | §41 | Lazy energy evaluation (`flg_energypos`/`flg_energyvel`) — MuJoCo avoids redundant energy recomputation when a plugin or sensor already triggered it. Only matters once plugins or energy-dependent sensors exist. | Low | T1 |
| DT-97 | §41 | Golden file generation for per-flag trajectory conformance (AC18). Generate `.npy` reference data from MuJoCo Python for all 25 flags. Required before v1.0 if not completed during §41 implementation. | Medium | T2 |
| ~~DT-98~~ | §41 | ~~Remove `passive` backward-compatibility shim~~ **Retired** — `passive` dropped entirely in §41 S2a (pre-v1.0, no users to break). Silently ignored by parser, matching MuJoCo 3.3.6+ behavior. | — | — |

---

## DT-74 Spec: Canonical `mj_jac` — Full Body Jacobian API

### Context

`compute_body_jacobian_at_point()` (now deleted; was in `jacobian.rs`) is dead code with
fundamental correctness bugs. It was intended to compute the translational
Jacobian at a world-frame point on a body, but:

1. **Returns 1×nv instead of 3×nv** — only stores `.x` component of each column
   (Hinge stores `j_col.x`, Slide stores `axis.x`, Ball stores `j_col.x`,
   Free linear hardcodes `(1.0, 0.0, 0.0)` — all are the `.x`-only bug)
2. **Free joint angular DOFs have two bugs:**
   - Uses **world-frame unit vectors** (`e_x, e_y, e_z`) instead of body-frame
     `R*e_i` (MuJoCo's `cdof` convention)
   - Computes lever arm from **raw `data.qpos`** instead of FK-computed
     `data.xpos[jnt_body]` — reads `qpos[qpos_adr..+3]` directly as the
     position, which only coincides with `xpos` when the body has no parent
     transform. The correct source is `data.xpos[jnt_body]` (as `mj_jac_site`
     and `mj_jac_point` use).
3. **Free joint linear DOFs hardcode `(1,0,0)`** — same `.x`-only bug as item 1
   (stores only the x-component of the 3×3 identity)
4. **Marked `#[allow(dead_code)]`** — zero callers

Meanwhile, the codebase already has **correct** implementations:
- `mj_jac_site` (line 9817): `(3×nv, 3×nv)` — correct `R*e_i` for ball/free
- `mj_jac_point` (line 9895): `6×nv` combined — correct `R*e_i`
- `mj_jac_body_com` (line 9968): thin wrapper on `mj_jac_point`
- `mj_jac_geom` (line 9973): thin wrapper on `mj_jac_point`
- `accumulate_point_jacobian` (line 9746): scalar projection — correct `R*e_i`

But MuJoCo's **core** function `mj_jac(m, d, jacp, jacr, point, body)` has no
direct equivalent. The existing functions are specialized variants, and the
chain-walk logic is duplicated across `mj_jac_site` (~70 lines) and
`mj_jac_point` (~70 lines).

### Scope & Dependencies

**In scope:** Canonical `mj_jac` function, wrapper refactor, dead-code removal,
comprehensive tests.

**Out of scope:** The `add_body_jacobian` closure inside `compute_contact_jacobian`
has a known free-joint bug (world-frame unit vectors instead of body-frame
`R*e_i`) — that is **DT-75**, a separate spec. This work does not touch
contact Jacobian code.

**Not refactored (intentional):** `accumulate_point_jacobian` (line 9746) and
`mj_apply_ft` (line 9986) both contain chain-walks that compute `J^T * vec`
without materializing the full Jacobian. These are kept separate for performance:
they avoid allocating two `3×nv` matrices when only a scalar or 1×nv projection
is needed. They are correct and tested independently.

### MuJoCo Reference

MuJoCo's `mj_jac` (`engine_core_util.c`):
- **Signature**: `mj_jac(m, d, jacp, jacr, point, body)` → two 3×nv matrices
- **`jacp`** (3×nv): translational Jacobian — velocity of `point` per unit DOF
- **`jacr`** (3×nv): rotational Jacobian — angular velocity per unit DOF
- Both nullable (caller can request only one)

#### Per-joint-type formulas

MuJoCo uses precomputed `cdof`; we compute inline (equivalent result).

| Joint | `jacp` column(s) | `jacr` column(s) | Lever arm `r` |
|-------|------------------|------------------|---------------|
| **Hinge** (1 DOF) | `axis × r` | `axis` | `point - anchor` |
| **Slide** (1 DOF) | `axis` | `0` | n/a |
| **Ball** (3 DOFs, i∈{0,1,2}) | `(R·eᵢ) × r` | `R·eᵢ` | `point - anchor` |
| **Free** lin (DOFs 0–2, i∈{0,1,2}) | `eᵢ` (world identity) | `0` | n/a |
| **Free** ang (DOFs 3–5, i∈{0,1,2}) | `(R·eᵢ) × r` | `R·eᵢ` | `point - xpos[body]` |

Where:
- `axis = xquat[jnt_body] * jnt_axis` (joint axis rotated to world frame)
- `anchor = xpos[jnt_body] + xquat[jnt_body] * jnt_pos` (joint anchor in world frame)
- `R = xquat[jnt_body].to_rotation_matrix()` (body orientation)
- `eᵢ = Vector3::ith(i, 1.0)` (i-th standard basis vector)

**Critical detail — free joint lever arm:** For hinge, slide, and ball joints,
`r = point - anchor` where `anchor = xpos[body] + R * jnt_pos`. For free
joints, `r = point - xpos[body]` directly — there is no `jnt_pos` offset
because free joints are always at the body origin (`jnt_pos = [0,0,0]` by
MuJoCo convention). The code must use `xpos[body]`, not the general anchor
formula, to avoid depending on this convention silently.

#### MuJoCo convenience wrappers

All are thin delegation to `mj_jac`:
- `mj_jacBody(body)` → `mj_jac(point=xpos[body], body)`
- `mj_jacBodyCom(body)` → `mj_jac(point=xipos[body], body)`
- `mj_jacSite(site)` → `mj_jac(point=site_xpos[site], body=site_body[site])`
- `mj_jacGeom(geom)` → `mj_jac(point=geom_xpos[geom], body=geom_body[geom])`

### Plan

#### Step 1: Add `mj_jac` as the canonical Jacobian function

Add a new public function `mj_jac` that is the single source of truth for the
body-chain Jacobian walk. Signature:

```rust
/// Compute the body-chain Jacobian at a world-frame point on a body.
///
/// Returns `(jacp, jacr)` where:
/// - `jacp` (3×nv): translational Jacobian — `v_point = jacp · qvel`
/// - `jacr` (3×nv): rotational Jacobian — `ω = jacr · qvel`
///
/// Per-joint-type columns:
///
/// | Joint     | jacp column       | jacr column |
/// |-----------|-------------------|-------------|
/// | Hinge     | axis × r          | axis        |
/// | Slide     | axis              | 0           |
/// | Ball(i)   | (R·eᵢ) × r       | R·eᵢ       |
/// | Free(0–2) | eᵢ (identity)     | 0           |
/// | Free(3–5) | (R·eᵢ) × r       | R·eᵢ       |
///
/// Returns all-zeros for `body_id == 0` (world body).
///
/// Equivalent to MuJoCo's `mj_jac()` in `engine_core_util.c`.
#[must_use]
pub fn mj_jac(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
) -> (DMatrix<f64>, DMatrix<f64>)  // (jacp 3×nv, jacr 3×nv)
```

Implementation: identical chain-walk to `mj_jac_site` (line 9817–9886), which
is already correct for all joint types. The only change is parameterizing by
`(body_id, point)` instead of `site_id`.

**File:** `sim/L0/core/src/jacobian.rs`.

#### Step 2: Refactor existing functions as thin wrappers

Rewrite the existing functions to delegate to `mj_jac`:

- **`mj_jac_site`** → calls `mj_jac(model, data, site_body[site_id], &site_xpos[site_id])`.
  **Return type unchanged:** `(DMatrix<f64>, DMatrix<f64>)` = `(jac_trans 3×nv, jac_rot 3×nv)`.
  All 4 existing production callers (`mj_fwd_actuation` method on Model ×2,
  `mj_fwd_actuation` standalone fn ×2) continue to destructure `(jac_t, jac_r)`
  with zero signature changes. (Also called by 2 tests in `jac_site_tests` and
  1 integration test in `fluid_derivatives.rs` — all unaffected.)
  **Remove `#[doc(hidden)]`** from `mj_jac_site`: after this refactor it is a
  first-class public wrapper (equivalent to MuJoCo's `mj_jacSite`), not an
  internal implementation detail.
- **`mj_jac_body`** → **new**, calls `mj_jac(model, data, body_id, &xpos[body_id])`, returns `(jacp, jacr)` — equivalent to MuJoCo's `mj_jacBody`.
  **Zero callers today** — added for MuJoCo API completeness. No existing code
  calls `mj_jac_point(model, data, body_id, &data.xpos[body_id])` with body
  origin as the point, so no migration is needed.
- **`mj_jac_point`** (6×nv) → calls `mj_jac`, combines into 6×nv (rows 0–2 = angular, rows 3–5 = linear)
- **`mj_jac_body_com`** → keeps delegating to `mj_jac_point(model, data, body_id, &xipos[body_id])`.
  No direct `mj_jac` call — transitive delegation via `mj_jac_point` which
  itself calls `mj_jac` and stacks into 6×nv. Body unchanged from current code.
  **Stays `pub(crate)`** — only called by `derivatives.rs` (inertia-box
  derivatives); not part of the user-facing API.
- **`mj_jac_geom`** → keeps delegating to `mj_jac_point(model, data, geom_body[geom_id], &geom_xpos[geom_id])`.
  Same transitive delegation pattern. Body unchanged from current code.
  **Stays `pub(crate)`** — only called by `derivatives.rs` (ellipsoid
  derivatives); not part of the user-facing API.

This eliminates the duplicated ~70-line chain-walk in `mj_jac_point` (lines
9895–9965) and ensures all paths share one implementation.

**Complete MuJoCo API parity:**

| MuJoCo | CortenForge | Returns |
|--------|-------------|---------|
| `mj_jac` | `mj_jac` | `(jacp 3×nv, jacr 3×nv)` |
| `mj_jacBody` | `mj_jac_body` | `(jacp 3×nv, jacr 3×nv)` |
| `mj_jacBodyCom` | `mj_jac_body_com` | `DMatrix 6×nv` |
| `mj_jacSite` | `mj_jac_site` | `(jacp 3×nv, jacr 3×nv)` |
| `mj_jacGeom` | `mj_jac_geom` | `DMatrix 6×nv` |

Note: `mj_jac_body_com` and `mj_jac_geom` retain their existing 6×nv return
type (rows 0–2 = angular, rows 3–5 = linear) because their callers
(`mjd_smooth_vel` derivatives, inertia-box derivatives, ellipsoid derivatives)
consume 6×nv. Changing them to return `(3×nv, 3×nv)` would churn callers for no
benefit.

**`#[doc(hidden)]` changes:**
- **Remove `#[doc(hidden)]` from `mj_jac_site`** (line 9815): it becomes a
  first-class public wrapper, equivalent to MuJoCo's `mj_jacSite`.
- **Keep `#[doc(hidden)]` on `mj_jac_point`** (line 9893): after this refactor,
  `mj_jac` becomes the canonical public API for `(3×nv, 3×nv)` output, and
  `mj_jac_point` is the 6×nv spatial-Jacobian variant used by internal
  derivative code (`mj_jac_body_com`, `mj_jac_geom`, `mjd_smooth_vel`). It is
  a layout convenience for internal consumers, not a user-facing API. External
  callers should use `mj_jac` (or `mj_jac_body`, `mj_jac_site`) and stack
  rows themselves if they need 6×nv.

**`#[must_use]` policy:**
All functions that return Jacobian matrices get `#[must_use]` — silently
discarding an allocated matrix is always a bug. This applies to:
- `mj_jac` (Step 1 signature already shows it)
- `mj_jac_body`, `mj_jac_site` (public wrappers)
- `mj_jac_point`, `mj_jac_body_com`, `mj_jac_geom` (`pub(crate)` wrappers)

#### Step 3: Delete `compute_body_jacobian_at_point`

Remove the broken dead-code function (lines 14177–14259). It has zero callers
and its functionality is superseded by `mj_jac`.

#### Step 4: Export from the crate public API

Add `mj_jac` and `mj_jac_body` to the re-exports in `sim/L0/core/src/lib.rs`
alongside the existing `mj_jac_point` and `mj_jac_site` exports (line 167–168).

#### Step 5: Tests

All tests in `sim-core`. Test module: `mj_jac_tests` (new), adjacent to existing
`jac_site_tests`.

**5a. Unit tests — `mj_jac` correctness per joint type:**

For each joint type (Hinge, Slide, Ball, Free), build a minimal single-joint
model at a non-trivial qpos, call `mj_jac`, and verify both `jacp` and `jacr`:

| Test | Assertion |
|------|-----------|
| `mj_jac_hinge_basic` | `jacp[:,0] = axis × r`, `jacr[:,0] = axis` |
| `mj_jac_slide_basic` | `jacp[:,0] = axis`, `jacr[:,0] = 0` |
| `mj_jac_ball_body_frame_axes` | `jacp[:,i] = (R·eᵢ) × r`, `jacr[:,i] = R·eᵢ` for i∈{0,1,2} |
| `mj_jac_free_translation` | `jacp[:,0:3] = I₃`, `jacr[:,0:3] = 0` |
| `mj_jac_free_rotation_body_frame` | `jacp[:,3+i] = (R·eᵢ) × r`, `jacr[:,3+i] = R·eᵢ` — **must use non-identity orientation** to distinguish body-frame from world-frame |
| `mj_jac_world_body_returns_zeros` | body_id=0 → both matrices all zeros, dimensions 3×nv |

**5b. Unit tests — wrapper consistency:**

| Test | Assertion |
|------|-----------|
| `mj_jac_site_delegates_to_mj_jac` | `mj_jac_site(site)` == `mj_jac(site_body, site_xpos)` |
| `mj_jac_body_delegates_to_mj_jac` | `mj_jac_body(body)` == `mj_jac(body, xpos[body])` |
| `mj_jac_point_combines_correctly` | `mj_jac_point` rows 0–2 == `jacr`, rows 3–5 == `jacp` (from `mj_jac` on same body/point). Note: `t32_jac_point_matches_jac_site` in `fluid_derivatives.rs` already cross-validates this for a hinge chain — this new test supplements it with an `mj_jac`-based assertion and extends coverage to ball/free joints. |
| `mj_jac_body_com_consistent` | `mj_jac_body_com(body)` == `mj_jac_point(body, xipos[body])` |
| `mj_jac_geom_consistent` | `mj_jac_geom(geom)` == `mj_jac_point(geom_body, geom_xpos)` |

**5c. Regression — existing `jac_site_tests` pass unchanged:**

The existing tests at line 24296 (`jac_site_agrees_with_accumulate_hinge`,
`jac_site_slide_joint`) must continue to pass, confirming the refactor is
behavior-preserving.

**5d. Multi-joint chain test:**

Build a 3-body chain (free → hinge → hinge) with non-trivial joint angles.
Verify `mj_jac` at a point on body 3 produces `jacp` (3×8) and `jacr` (3×8)
with correct columns for all 8 DOFs (6 free + 2 hinge). Cross-check `jacp`
against finite-difference (see 5e procedure).

**5e. Finite-difference validation (gold standard):**

For a free-joint + ball-joint chain at random non-identity orientation:

1. Run `mj_fwd_position` at baseline `qpos₀`, record `point₀`
2. For each DOF `d` in `0..nv`:
   - Clone `qpos₀` into `qpos_pert`
   - **Apply velocity perturbation via `mj_integrate_pos_explicit`**: set
     `qvel = 0` except `qvel[d] = 1.0`, integrate for `dt = epsilon = 1e-7`.
     This correctly handles quaternion DOFs (ball `nq=4,nv=3`; free `nq=7,nv=6`)
     — naive `qpos[i] += epsilon` is **wrong** for quaternion components.
   - Run `mj_fwd_position` with `qpos_pert`
   - `jacp_fd[:, d] = (point_pert - point₀) / epsilon`
   - `jacr_fd[:, d] = rotation_log(R_pert · R₀⁻¹) / epsilon`
     where `rotation_log` extracts the axis-angle vector from the relative
     rotation quaternion. **Which body's rotation:** use `data.xquat[body_id]`
     for the body that owns the point — `jacr` gives angular velocity of
     **that specific body**, not intermediate bodies in the chain.
     **Small-angle approximation:** for `epsilon = 1e-7`, the relative
     quaternion `q_rel = q_pert · q₀⁻¹` is near-identity, so
     `rotation_log(q_rel) ≈ 2 * [q_rel.i, q_rel.j, q_rel.k]` (nalgebra's
     `[w, i, j, k]` quaternion layout). This avoids `acos` numerical issues.
3. Assert `jacp ≈ jacp_fd` within `1e-5` relative tolerance
4. Assert `jacr ≈ jacr_fd` within `1e-5` relative tolerance

This validates both `jacp` and `jacr` and catches any frame convention errors
(body-frame vs world-frame) that analytical unit tests might miss.

**Why `mj_integrate_pos_explicit`:** The codebase already exports this function
(lib.rs:165). It maps a velocity-space perturbation to a valid `qpos`
perturbation, handling quaternion renormalization. Without it, FD for ball/free
joints would corrupt the quaternion and produce garbage FK results.

**5f. `jacr` analytical cross-check for hinge chain:**

For a 2-hinge chain, verify `jacr` analytically: each hinge's rotational
Jacobian column should be its world-frame axis vector, and `jacr · qvel` should
equal the total angular velocity. This is a simpler cross-check that doesn't
require the rotation-log machinery of 5e.

### Deliverables

| # | Deliverable |
|---|-------------|
| D1 | `mj_jac` function with doc-comment containing formula table |
| D2 | `mj_jac_body` new wrapper |
| D3 | `mj_jac_site` refactored as thin wrapper, `#[doc(hidden)]` removed |
| D4 | `mj_jac_point` refactored as thin wrapper (keeps `#[doc(hidden)]`) |
| D5 | `mj_jac_body_com` unchanged (stays `pub(crate)`, transitive delegation) |
| D6 | `mj_jac_geom` unchanged (stays `pub(crate)`, transitive delegation) |
| D7 | `compute_body_jacobian_at_point` deleted |
| D8 | `lib.rs` exports updated (`mj_jac`, `mj_jac_body` added) |
| D9 | Tests 5a–5f |

### Files Modified

| File | Change |
|------|--------|
| `sim/L0/core/src/jacobian.rs` | Add `mj_jac` (D1), add `mj_jac_body` (D2), refactor `mj_jac_site`/`mj_jac_point`/`mj_jac_body_com`/`mj_jac_geom` as wrappers (D3–D6), delete `compute_body_jacobian_at_point` (D7), add `mj_jac_tests` module (D9) |
| `sim/L0/core/src/lib.rs` | Export `mj_jac`, `mj_jac_body` (D8) |

### Verification

```bash
cargo test -p sim-core -- jac          # all Jacobian tests (old + new)
cargo test -p sim-core                 # full crate regression
cargo clippy -p sim-core -- -D warnings
cargo fmt --all -- --check
```

### Implementation Status: DONE (2026-02-22)

All 9 deliverables (D1–D9) implemented. Summary:

- **`mj_jac`** added as canonical body-chain Jacobian (single source of truth)
- **`mj_jac_body`** added (MuJoCo `mj_jacBody` equivalent)
- **`mj_jac_site`** refactored to thin wrapper, `#[doc(hidden)]` removed
- **`mj_jac_point`** refactored to thin wrapper (keeps `#[doc(hidden)]`)
- **`#[must_use]`** added to `mj_jac_body_com` and `mj_jac_geom`
- **`compute_body_jacobian_at_point`** deleted (88 lines of broken dead code)
- **`lib.rs`** exports updated (`mj_jac`, `mj_jac_body`)
- **14 new tests** in `mj_jac_tests` module: 6 per-joint-type analytical, 5 wrapper
  consistency, 1 multi-joint chain (free→hinge→hinge, 8 DOFs + FD), 1 free+ball FD
  validation (jacp + jacr), 1 jacr hinge cross-check (jacr·qvel = ω)
- All 396 sim-core tests pass, clippy clean, fmt clean

---

## DT-75 Spec: Contact Jacobian Free-Joint Bug — Body-Frame Axes Fix

### Context

The `add_body_jacobian` closure (used inside both `compute_contact_jacobian` and
`compute_flex_contact_jacobian`) and the standalone `add_angular_jacobian`
function have three bugs in their free-joint handling:

**Bug 1 — World-frame angular axes in translational projection:** Free joint
angular DOFs (3–5) use world-frame unit vectors (`e_x, e_y, e_z`) instead of
body-frame rotated axes (`R·eᵢ`). This produces incorrect contact Jacobian
columns whenever the free body has a non-identity orientation.

Buggy code (`compute_contact_jacobian`, line 14377–14382):
```rust
let ex = Vector3::x();   // WRONG: world-frame
let ey = Vector3::y();
let ez = Vector3::z();
j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
```

Correct formula (matching `mj_jac` at line 9886–9893 and `accumulate_point_jacobian`
at line 9797–9801):
```rust
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let omega = rot * Vector3::ith(i, 1.0);  // body-frame axis
    j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
}
```

MuJoCo's convention: angular DOFs of free joints use `cdof[dof+3..dof+6]` which
are the columns of the body orientation matrix `R`. The angular velocity
`ω = R·ω_local`, so the Jacobian columns must be `R·eᵢ`, not `eᵢ`. Ball joints
in the same closures are already correct (they use `rot * Vector3::ith(i, 1.0)`).

**Bug 2 — Lever arm from `qpos` instead of `xpos`:** The lever arm `r` is
computed from `data.qpos[qpos_adr..+3]` (raw generalized coordinates) instead
of `data.xpos[jnt_body]` (FK-computed world position). For rootless free bodies,
`qpos[0..3]` coincides with `xpos[body]`. But in principle this is wrong — it
reads the position *before* FK updates it. The correct source is
`data.xpos[jnt_body]`, as used by `mj_jac` (line 9886), `accumulate_point_jacobian`
(line 9795), and `mj_apply_ft` (line 10022).

Buggy code (line 14370–14374):
```rust
let jpos = Vector3::new(
    data.qpos[model.jnt_qpos_adr[jnt_id]],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
);
```

Correct: `let jpos = data.xpos[jnt_body];`

**Bug 3 — `add_angular_jacobian` world-frame axes in rotational projection:**
The standalone `add_angular_jacobian` function (line 14436, free branch at
14475–14480) is called by `compute_contact_jacobian` for torsional friction
(row 3, lines 14414–14415) and rolling friction (rows 4–5, lines 14421–14425).
It has the same world-frame bug for free joint angular DOFs:

```rust
MjJointType::Free => {
    j[(row, dof_adr + 3)] += sign * direction.x;  // WRONG: world-frame
    j[(row, dof_adr + 4)] += sign * direction.y;
    j[(row, dof_adr + 5)] += sign * direction.z;
}
```

The current code computes `direction · eᵢ` (extracting the x/y/z component of
`direction`), which is the dot product with world-frame basis vectors. The
correct formula is `direction · (R·eᵢ)` — the rotational Jacobian column for
a free-joint angular DOF `i` is `R·eᵢ`, and we project it along `direction`.
This matches the Ball joint case directly above it (line 14467–14473), which
already correctly uses `rot * Vector3::ith(i, 1.0)`.

**Formula comparison (old vs new):**

| Location | DOF | Old (Buggy) | New (Correct) | Reference |
|----------|-----|-------------|---------------|-----------|
| `add_body_jacobian` angular col `i` (translational) | 3+i | `d · (eᵢ × r)` where `r = contact.pos - qpos[0:3]` | `d · ((R·eᵢ) × r)` where `r = contact.pos - xpos[body]` | `mj_jac` line 9886–9892 (`jacp` column) |
| `add_angular_jacobian` angular col `i` (rotational) | 3+i | `d · eᵢ` (= `direction[i]`) | `d · (R·eᵢ)` | `mj_jac` line 9892 (`jacr` column) |

**Affected functions (3 locations, 2 functions):**

| Location | Function | Lines | Bugs |
|----------|----------|-------|------|
| 1 | `add_body_jacobian` closure in `compute_flex_contact_jacobian` | 14235–14251 | Bug 1 + Bug 2 |
| 2 | `add_body_jacobian` closure in `compute_contact_jacobian` | 14363–14383 | Bug 1 + Bug 2 |
| 3 | `add_angular_jacobian` standalone function | 14475–14480 | Bug 3 (no lever arm — pure angular) |

**Impact:** Incorrect contact constraint Jacobian for any contact involving a
free-joint body at non-identity orientation. This corrupts:
- Normal force direction (row 0): wrong velocity mapping → wrong constraint
  violation computation → incorrect normal forces
- Friction forces (rows 1–2): wrong tangential velocity mapping → friction
  forces applied along wrong DOFs
- Torsional friction (row 3): wrong angular velocity projection
- Rolling friction (rows 4–5): wrong angular velocity projection

For bodies near identity orientation the error is small (world ≈ body frame),
which is why this bug has gone undetected — most test cases start at or near
identity orientation.

### Scope & Dependencies

**In scope:** Fix the three buggy free-joint code paths, add targeted tests.

**Out of scope:**
- Refactoring `add_body_jacobian` closures to delegate to `mj_jac` — this
  would be cleaner but is a larger refactor that changes the projected-row
  computation pattern (`direction.dot(&j_col)` vs materializing full 3×nv
  matrices). The closure computes `d^T · J_col` directly (scalar per DOF),
  which is more efficient than materializing two 3×nv matrices and then
  projecting. This optimization is worth keeping. A future unification could
  add a `mj_jac_projected` variant, but that is out of scope here.
- Hinge, Slide, and Ball joint handling in these closures — already correct.
- The `compute_contact_normal_jacobian` function (line 10795) — it delegates
  to `accumulate_point_jacobian`, which is already correct.
- Flex-rigid contact testing — programmatic construction of flex bodies (flex
  vertex DOFs, `flexvert_dofadr`, flex element arrays) is substantially more
  complex than rigid-body model construction and warrants its own test
  infrastructure work. The rigid-body `add_body_jacobian` closure in
  `compute_flex_contact_jacobian` (Location 1) has identical code to
  Location 2, so the rigid-body tests validate the fix for both.

**Dependencies:** None. DT-74 (canonical `mj_jac`) is complete and provides the
reference implementation, but this fix is a standalone correction to the contact
Jacobian closures.

### MuJoCo Reference

MuJoCo's contact Jacobian uses `cdof` (precomputed motion subspace columns).
For a free joint body:
- `cdof[dof+0..+3]` = `[0,0,0, 1,0,0]`, `[0,0,0, 0,1,0]`, `[0,0,0, 0,0,1]`
  (world-frame translational identity — linear DOFs)
- `cdof[dof+3..+6]` = `[R·e₀; 0]`, `[R·e₁; 0]`, `[R·e₂; 0]`
  (body-frame rotated axes — angular DOFs)

The translational Jacobian column for angular DOF `i` is `(R·eᵢ) × r` where
`r = point - xpos[body]`. The rotational Jacobian column is `R·eᵢ`.

This is identical to the ball-joint formula (which is already correct in the
contact Jacobian closures), except the lever arm uses `xpos[body]` directly
(no `jnt_pos` offset, since free joints always have `jnt_pos = [0,0,0]`).

### Plan

#### Step 1: Fix `add_body_jacobian` closure in `compute_contact_jacobian`

Replace the free-joint block (lines 14363–14383) with the correct body-frame
formula:

```rust
MjJointType::Free => {
    // Linear DOFs (0–2): world-frame identity — unchanged, already correct.
    j[(row, dof_adr)] += sign * direction.x;
    j[(row, dof_adr + 1)] += sign * direction.y;
    j[(row, dof_adr + 2)] += sign * direction.z;

    // Angular DOFs (3–5): body-frame axes R·eᵢ, lever arm from xpos.
    let jpos = data.xpos[jnt_body];
    let r = contact.pos - jpos;
    let rot = data.xquat[jnt_body].to_rotation_matrix();
    for i in 0..3 {
        let omega = rot * Vector3::ith(i, 1.0);
        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
    }
}
```

**File:** `sim/L0/core/src/constraint/jacobian.rs`.

#### Step 2: Fix `add_body_jacobian` closure in `compute_flex_contact_jacobian`

Replace the free-joint block (lines 14235–14251) with the identical body-frame
formula. The code is structurally identical to Step 1 — same closure signature,
same `contact.pos` for lever arm:

```rust
MjJointType::Free => {
    j[(row, dof_adr)] += sign * direction.x;
    j[(row, dof_adr + 1)] += sign * direction.y;
    j[(row, dof_adr + 2)] += sign * direction.z;

    let jpos = data.xpos[jnt_body];
    let r = contact.pos - jpos;
    let rot = data.xquat[jnt_body].to_rotation_matrix();
    for i in 0..3 {
        let omega = rot * Vector3::ith(i, 1.0);
        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
    }
}
```

**File:** `sim/L0/core/src/constraint/jacobian.rs`.

#### Step 3: Fix `add_angular_jacobian` standalone function

Replace the free-joint case (lines 14475–14480) with body-frame axes:

```rust
MjJointType::Free => {
    // Angular DOFs (3–5): body-frame axes, matching Ball case above.
    let rot = data.xquat[jnt_body].to_rotation_matrix();
    for i in 0..3 {
        let omega = rot * Vector3::ith(i, 1.0);
        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega);
    }
}
```

Note: no lever arm (`r`) here — `add_angular_jacobian` computes the *rotational*
Jacobian projection, not the translational one. The rotational Jacobian column
for a free-joint angular DOF is simply `R·eᵢ` (same as the `jacr` column in
`mj_jac`).

**File:** `sim/L0/core/src/constraint/jacobian.rs`.

#### Step 4: Remove stale comment in `accumulate_point_jacobian`

Lines 9792–9794 have a comment noting the bug:
```rust
// Rotational DOFs: body-frame axes (R*e_i), matching MuJoCo's
// cdof convention. NOTE: the existing `add_body_jacobian` uses
// world-frame unit vectors — that is a pre-existing bug.
```

Replace with a clean comment (the bug is now fixed):
```rust
// Rotational DOFs: body-frame axes (R*e_i), matching MuJoCo's
// cdof convention.
```

#### Step 5: Tests

All tests in `sim-core`. Test module: `contact_jac_free_joint_tests` (new),
placed in `constraint/jacobian.rs` as a `#[cfg(test)]` submodule alongside the
existing `mj_jac_tests` module.

**Visibility note:** `compute_contact_jacobian` and
`add_angular_jacobian` are module-private functions (plain `fn`,
no `pub`). Tests in `#[cfg(test)]` submodules *within* `constraint/jacobian.rs`
can call them directly — Rust's visibility rules grant child modules access to
all items in their parent module, regardless of visibility modifier. This is
the same pattern used by the existing `mj_jac_tests` module.

**Test strategy — direct contact construction:** All tests construct `Contact`
structs directly via `Contact::new(...)` or `Contact::with_condim(...)` rather
than relying on collision detection to generate contacts. This isolates the
Jacobian computation under test from collision detection geometry, margin, and
gap logic. The contact position, normal, and frame are set to known values,
making assertions deterministic.

**`Contact::new` returns `dim=3`** (when `friction > 0.0`) — sufficient for
tests 5a/5b/5e/5f which only need normal + tangent rows. Tests requiring
torsional/rolling rows (5c, 5d) use `Contact::with_condim(..., condim: 6, ...)`
to get a 6-row Jacobian. Note: `with_condim` takes `condim: i32`.

**Test model helper — `make_free_body_contact_model`:**

The existing `make_single_joint_model` helper (line 24365) does **not** populate
`model.geom_body` or `model.ngeom` (it sets `body_geom_num = [0, 0]`). Since
`compute_contact_jacobian` resolves body IDs via
`model.geom_body[contact.geom1]` and `model.geom_body[contact.geom2]`
(line 14309–14310), calling it without `geom_body` would panic.

**What `compute_contact_jacobian` actually accesses on Model (geom-related):**
only `geom_body` — it does **not** read `geom_pos`, `geom_quat`, `geom_type`,
or `geom_size`. The helper therefore only needs `geom_body`, `ngeom`,
`body_geom_adr`, and `body_geom_num`.

Create a new test helper `make_free_body_contact_model` that extends the
free-joint model with the minimal geom fields needed for contact tests:

```rust
fn make_free_body_contact_model() -> (Model, Data) {
    let (mut model, mut data) = make_single_joint_model(
        MjJointType::Free,
        Vector3::z(),     // axis (unused for free joints)
        Vector3::zeros(), // body_pos
        0.0,              // qpos_val
    );
    // Only geom_body is read by compute_contact_jacobian (line 14309–14310).
    // geom_pos/quat/type/size are not accessed — omitted intentionally.
    model.ngeom = 2;
    model.geom_body = vec![0, 1];  // geom 0 → world, geom 1 → free body
    model.body_geom_adr = vec![0, 1];
    model.body_geom_num = vec![1, 1];

    // IMPORTANT: make_single_joint_model sets qpos0 = zeros(7), giving a
    // degenerate zero quaternion [0,0,0,0]. Fix to identity quat [1,0,0,0]
    // and re-run FK so xquat[1] is valid. Without this, xquat[1] is computed
    // from UnitQuaternion::from_quaternion(Quaternion::new(0,0,0,0)) — undefined.
    data.qpos[3] = 1.0; // w component of identity quaternion
    mj_fwd_position(&model, &mut data);

    (model, data)
}
```

**Note on `make_single_joint_model` and free joints:** `make_single_joint_model`
sets `qpos0 = DVector::zeros(nq)` (line 24427) without initializing the
quaternion `w`-component for free/ball joints. The DT-74 test
`mj_jac_free_translation` (line 24578) gets away with this because it only
checks linear DOF columns. Contact tests exercise angular DOFs and require
valid orientation — so the helper must fix the quaternion before FK.

For tests that need a non-identity orientation, the caller writes a non-identity
quaternion into `data.qpos[3..7]` and re-runs `mj_fwd_position`. Example (45°
about Z):
```rust
let (mut model, mut data) = make_free_body_contact_model();
let angle = std::f64::consts::FRAC_PI_4; // 45°
let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
data.qpos[3] = q.w; data.qpos[4] = q.i; data.qpos[5] = q.j; data.qpos[6] = q.k;
mj_fwd_position(&model, &mut data);
```

**Two-free-body helper — `make_two_free_body_contact_model`:**

For test 5d, create a model with `nbody=3`, `njnt=2`, `nv=12`, `nq=14`,
`ngeom=3` (world geom + geom per free body). Each free body is a child of the
world body (no kinematic chain between them). This requires a separate helper
because `make_single_joint_model` only supports one joint.

**Completeness requirement:** `Model::make_data()` reads `self.qLD_nnz`
(computed by `compute_qld_csr_metadata()`) and `self.qpos0` for initial qpos.
`mj_fwd_position` reads `body_parent`, `body_pos`, `body_quat`, joint fields,
and dimensions. The helper must set **all** fields that `make_data()` and
`mj_fwd_position` access — following the pattern of the DT-74 manual model
construction at line 24598–24676. Missing fields cause panics.

```rust
fn make_two_free_body_contact_model() -> (Model, Data) {
    let mut model = Model::empty();

    // --- Topology: 3 bodies (world + 2 free), no kinematic chain ---
    model.nbody = 3;
    model.body_parent = vec![0, 0, 0];  // both free bodies are children of world
    model.body_rootid = vec![0, 1, 2];
    model.body_jnt_adr = vec![0, 0, 1]; // world has no joints
    model.body_jnt_num = vec![0, 1, 1];
    model.body_dof_adr = vec![0, 0, 6];
    model.body_dof_num = vec![0, 6, 6];
    model.body_pos = vec![Vector3::zeros(); 3];
    model.body_quat = vec![UnitQuaternion::identity(); 3];
    model.body_ipos = vec![Vector3::zeros(); 3];
    model.body_iquat = vec![UnitQuaternion::identity(); 3];
    model.body_mass = vec![0.0, 1.0, 1.0];
    model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01),
                              Vector3::new(0.01, 0.01, 0.01)];
    model.body_name = vec![Some("world".into()), Some("b1".into()), Some("b2".into())];
    model.body_subtreemass = vec![2.0, 1.0, 1.0];

    // --- 2 free joints ---
    model.njnt = 2;
    model.nq = 14;  // 7 + 7
    model.nv = 12;  // 6 + 6
    model.jnt_type = vec![MjJointType::Free; 2];
    model.jnt_body = vec![1, 2];
    model.jnt_qpos_adr = vec![0, 7];
    model.jnt_dof_adr = vec![0, 6];
    model.jnt_axis = vec![Vector3::z(); 2];   // unused for free joints
    model.jnt_pos = vec![Vector3::zeros(); 2]; // free joints: always origin
    model.jnt_limited = vec![false; 2];
    model.jnt_range = vec![(0.0, 0.0); 2];
    model.jnt_stiffness = vec![0.0; 2];
    model.jnt_springref = vec![0.0; 2];
    model.jnt_damping = vec![0.0; 2];
    model.jnt_armature = vec![0.0; 2];
    model.jnt_solref = vec![[0.02, 1.0]; 2];
    model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
    model.jnt_name = vec![Some("j1".into()), Some("j2".into())];

    // --- DOF metadata (12 DOFs: 6 per free joint) ---
    model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2];
    model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1];
    model.dof_parent = vec![None; 12];
    model.dof_armature = vec![0.0; 12];
    model.dof_damping = vec![0.0; 12];
    model.dof_frictionloss = vec![0.0; 12];

    // --- No sites needed for contact tests ---
    model.nsite = 0;
    model.site_body = vec![];
    model.site_pos = vec![];
    model.site_quat = vec![];
    model.site_type = vec![];
    model.site_size = vec![];
    model.site_name = vec![];

    // --- Geoms: only geom_body is read by compute_contact_jacobian ---
    model.ngeom = 3;
    model.geom_body = vec![0, 1, 2];
    model.body_geom_adr = vec![0, 1, 2];
    model.body_geom_num = vec![1, 1, 1];

    // --- qpos0: identity quaternions for both free joints ---
    // qpos layout: [x1,y1,z1, w1,i1,j1,k1, x2,y2,z2, w2,i2,j2,k2]
    model.qpos0 = DVector::zeros(14);
    model.qpos0[3] = 1.0;   // body 1: identity quaternion w-component
    model.qpos0[10] = 1.0;  // body 2: identity quaternion w-component
    model.timestep = 0.001;

    // --- Ancestor metadata (required by compute_qld_csr_metadata → make_data) ---
    model.body_ancestor_joints = vec![vec![]; 3];
    model.body_ancestor_mask = vec![vec![]; 3];
    model.compute_ancestors();
    model.compute_qld_csr_metadata();

    // --- Data: caller sets orientations and calls mj_fwd_position ---
    let data = model.make_data();
    (model, data)
}
```

**5a. Unit test — `contact_jac_free_body_frame_vs_world_frame`:**

Build a free-joint body at 45° about Z. Construct a `Contact` directly:

```rust
let contact = Contact::new(
    Vector3::new(0.3, 0.2, 0.1),    // pos: arbitrary point near body
    Vector3::new(0.0, 0.0, 1.0),    // normal: +Z
    0.01,                             // depth
    0,                                // geom1 (world geom)
    1,                                // geom2 (free body geom)
    0.5,                              // friction → dim=3
);
let j = compute_contact_jacobian(&model, &data, &contact);
// j is 3×6 (dim=3, nv=6)
```

1. Compute `R = data.xquat[1].to_rotation_matrix()` and `r = contact.pos - data.xpos[1]`
2. For each angular DOF `i` in 0..3:
   - Expected: `normal.dot(&(R * Vector3::ith(i, 1.0)).cross(&r))`
   - Actual: `j[(0, 3 + i)]`
3. Assert within `1e-12`

This test **must fail** with world-frame axes and **pass** with body-frame axes.
At 45° about Z, `R·e₀ = [cos45, sin45, 0]` ≠ `e₀ = [1, 0, 0]`, so the
angular DOF columns will differ measurably (not just floating-point noise).

**5b. Cross-check — `contact_jac_all_rows_match_mj_jac_projection`:**

For the same model as 5a (free body at 45° about Z), construct a `dim=3`
contact. Verify **all three translational rows** (normal + 2 tangents) match
the projected body Jacobians from `mj_jac`. The contact Jacobian computes
**relative** velocity: `J_contact = J_body2 - J_body1`, so the cross-check must
account for both bodies:

```rust
let body1 = model.geom_body[contact.geom1];  // world (body 0)
let body2 = model.geom_body[contact.geom2];  // free body

let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
let (jacp1, _) = mj_jac(&model, &data, body1, &contact.pos);
// jacp1 is all-zeros for world body (body 0)

let directions = [contact.normal, contact.frame[0], contact.frame[1]];
for (row, dir) in directions.iter().enumerate() {
    for dof in 0..nv {
        let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
        let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
        let expected = dir.dot(&(col2 - col1));
        assert_abs_diff_eq!(j[(row, dof)], expected, epsilon = 1e-12);
    }
}
```

This validates that **every element** of the `dim=3` contact Jacobian (all rows,
all DOFs — linear and angular) matches the `mj_jac` projection. Since DT-74's
FD tests already validate `mj_jac` itself against finite differences
(`mj_jac_free_ball_chain_fd`, line 24917), this transitively validates the
contact Jacobian against FD without repeating the FD machinery.

**5c. Torsional/rolling — `angular_jac_free_body_frame`:**

Build a free-joint body at non-identity orientation. Construct a `condim=6`
contact:

```rust
let contact = Contact::with_condim(
    Vector3::new(0.3, 0.2, 0.1),    // pos
    Vector3::new(0.0, 0.0, 1.0),    // normal
    0.01,                             // depth
    0,                                // geom1 (world)
    1,                                // geom2 (free body)
    0.5,                              // sliding friction
    0.01,                             // torsional friction
    0.005,                            // rolling friction
    6_i32,                            // condim = 6 (all 6 rows)
    DEFAULT_SOLREF,
    DEFAULT_SOLIMP,
);
let j = compute_contact_jacobian(&model, &data, &contact);
// j is 6×6 (dim=6, nv=6)
```

Cross-check **all 6 rows** against `mj_jac`'s relative `jacp` and `jacr`.
Use the same `col2 - col1` pattern as tests 5b/5d for consistency — even
though `body1 = 0` (world) produces all-zero Jacobians, the relative formula
makes the test self-documenting and correct for any future generalization:

```rust
let body1 = model.geom_body[contact.geom1];  // world (body 0)
let body2 = model.geom_body[contact.geom2];  // free body

let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);
let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);
// jacp1/jacr1 are all-zeros for world body (body 0)

// Rows 0–2 (normal + tangents): relative translational Jacobian projection
let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
for (row, dir) in trans_dirs.iter().enumerate() {
    for dof in 0..nv {
        let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
        let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
        let expected = dir.dot(&(col2 - col1));
        assert_abs_diff_eq!(j[(row, dof)], expected, epsilon = 1e-12);
    }
}

// Row 3 (torsional): relative rotational Jacobian projected along normal
// Rows 4–5 (rolling): relative rotational Jacobian projected along tangents
let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
for (i, dir) in rot_dirs.iter().enumerate() {
    let row = 3 + i;
    for dof in 0..nv {
        let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
        let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
        let expected = dir.dot(&(col2 - col1));
        assert_abs_diff_eq!(j[(row, dof)], expected, epsilon = 1e-12);
    }
}
```

This validates all 6 constraint rows for all DOFs (including linear DOFs 0–2,
where angular rows should be zero, and angular DOFs 3–5 where both translational
and rotational rows have non-trivial contributions). The relative Jacobian
formula is consistent with `compute_contact_jacobian`'s sign convention
(`body2: +1, body1: -1`) and matches the pattern used in tests 5b and 5d.

**5d. Two-body contact — `contact_jac_two_free_bodies`:**

Build a model with two free-joint bodies (body 1 and body 2), each at a
different non-identity orientation (e.g., body 1 at 45° about Z, body 2 at 30°
about X). The caller must write quaternions into `data.qpos` and call
`mj_fwd_position` after `make_two_free_body_contact_model` returns:

```rust
let (model, mut data) = make_two_free_body_contact_model();
// Body 1: 45° about Z
let q1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
data.qpos[3] = q1.w; data.qpos[4] = q1.i; data.qpos[5] = q1.j; data.qpos[6] = q1.k;
// Body 2: 30° about X
let q2 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_6);
data.qpos[10] = q2.w; data.qpos[11] = q2.i; data.qpos[12] = q2.j; data.qpos[13] = q2.k;
mj_fwd_position(&model, &mut data);
```

Construct a `condim=6` contact between them (geom1 on body 1, geom2
on body 2). Verify all 6 rows against `mj_jac`-based relative Jacobian:

```rust
let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);
let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);

// Rows 0–2: relative translational Jacobian
let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
for (row, dir) in trans_dirs.iter().enumerate() {
    for dof in 0..nv {
        let col2 = Vector3::new(jacp2[(0,dof)], jacp2[(1,dof)], jacp2[(2,dof)]);
        let col1 = Vector3::new(jacp1[(0,dof)], jacp1[(1,dof)], jacp1[(2,dof)]);
        let expected = dir.dot(&(col2 - col1));
        assert_abs_diff_eq!(j[(row, dof)], expected, epsilon = 1e-12);
    }
}

// Rows 3–5: relative rotational Jacobian
let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
for (i, dir) in rot_dirs.iter().enumerate() {
    let row = 3 + i;
    for dof in 0..nv {
        let col2 = Vector3::new(jacr2[(0,dof)], jacr2[(1,dof)], jacr2[(2,dof)]);
        let col1 = Vector3::new(jacr1[(0,dof)], jacr1[(1,dof)], jacr1[(2,dof)]);
        let expected = dir.dot(&(col2 - col1));
        assert_abs_diff_eq!(j[(row, dof)], expected, epsilon = 1e-12);
    }
}
```

This is the strongest test because it exercises both sides of the relative
Jacobian with non-trivial body-frame rotations on each, catching any sign or
body-ID confusion. The model has `nv = 12` (6 per free body), `nq = 14`
(7 per free body), and each body's angular DOFs produce distinct `R·eᵢ` columns.
With body 1's DOFs at columns 0–5 and body 2's at columns 6–11, the relative
Jacobian has body 1 contributing with sign `-1` and body 2 with sign `+1` —
the test verifies this sign convention is correct across all 12 DOFs.

**Sign convention origin:** `compute_contact_jacobian` calls
`add_body_jacobian(..., body2, 1.0)` and `add_body_jacobian(..., body1, -1.0)`
(lines 14399–14400), so the Jacobian encodes relative velocity `v₂ - v₁`.
The `add_angular_jacobian` calls at lines 14414–14425 use the same `+1`/`-1`
pattern for torsional/rolling rows. The sign comment block at lines 14390–14396
documents the physical interpretation: positive normal velocity = separating.

**5e. Regression — identity orientation — `contact_jac_free_identity_unchanged`:**

Use `make_free_body_contact_model` directly — it returns identity orientation
(`qpos = [0,0,0, 1,0,0,0]`) with FK already run. Verify that the contact
Jacobian columns for angular DOFs match both the old world-frame formula and
the new body-frame formula (they coincide at identity). Specifically:

```rust
let rot = data.xquat[1].to_rotation_matrix();
// At identity: rot == I₃, so R·eᵢ == eᵢ
for i in 0..3 {
    assert_abs_diff_eq!(rot * Vector3::ith(i, 1.0), Vector3::ith(i, 1.0), epsilon = 1e-15);
}
```

Then verify the contact Jacobian matches both the hand-computed world-frame
values (which were "accidentally correct" before) and the `mj_jac` projection.
This confirms the fix doesn't break the identity case.

**5f. `condim=1` (frictionless) — `contact_jac_free_frictionless`:**

Use the same non-identity orientation setup as tests 5a–5c (45° about Z) to
make the body-frame assertion meaningful. Construct a `condim=1` contact (no
friction, `dim=1`):
```rust
let contact = Contact::with_condim(
    Vector3::new(0.3, 0.2, 0.1), Vector3::z(), 0.01,
    0, 1, 0.0, 0.0, 0.0, 1_i32, DEFAULT_SOLREF, DEFAULT_SOLIMP,
);
let j = compute_contact_jacobian(&model, &data, &contact);
// j is 1×6 (dim=1, nv=6)
```

Verify: `j` has exactly 1 row (the normal row), and its angular DOF columns
match the body-frame `mj_jac` projection (same pattern as 5b but only row 0).
This ensures the fix works for the minimal contact dimension and that the code
path doesn't index out of bounds when `dim=1` (no tangent, torsional, or
rolling rows).

### Deliverables

| # | Deliverable |
|---|-------------|
| D1 | Fix `add_body_jacobian` closure in `compute_contact_jacobian` — body-frame axes + xpos lever arm |
| D2 | Fix `add_body_jacobian` closure in `compute_flex_contact_jacobian` — same fix |
| D3 | Fix `add_angular_jacobian` function — body-frame axes |
| D4 | Remove stale bug comment in `accumulate_point_jacobian` (lines 9792–9794) |
| D5 | `make_free_body_contact_model` + `make_two_free_body_contact_model` test helpers |
| D6 | Tests 5a–5f (6 tests) |

### Files Modified

| File | Change |
|------|--------|
| `sim/L0/core/src/constraint/jacobian.rs` | Fix free-joint angular DOFs in `compute_contact_jacobian` (D1), `compute_flex_contact_jacobian` (D2), `add_angular_jacobian` (D3), clean stale comment (D4), add `contact_jac_free_joint_tests` module with helpers and 6 tests (D5, D6) |

### Verification

```bash
cargo test -p sim-core -- contact_jac    # new contact Jacobian tests
cargo test -p sim-core -- jac            # all Jacobian tests (old + new)
cargo test -p sim-core                   # full crate regression
cargo clippy -p sim-core -- -D warnings
cargo fmt --all -- --check
```

### Implementation Status: DONE (2026-02-22)

All 6 deliverables (D1–D6) implemented. Summary:

- **D1–D2:** `add_body_jacobian` closures in `compute_contact_jacobian` (line 14359)
  and `compute_flex_contact_jacobian` (line 14234) fixed — world-frame
  `Vector3::x()/y()/z()` replaced with body-frame `rot * Vector3::ith(i, 1.0)`,
  `qpos`-based lever arm replaced with `data.xpos[jnt_body]`
- **D3:** `add_angular_jacobian` (line 14465) fixed — `direction.x/y/z`
  (world-frame dot products) replaced with `direction.dot(&(rot * Vector3::ith(i, 1.0)))`
- **D4:** Stale "pre-existing bug" comment in `accumulate_point_jacobian` removed
- **D5:** `make_free_body_contact_model` and `make_two_free_body_contact_model`
  test helpers added in `contact_jac_free_joint_tests` module
- **D6:** 6 tests: body-frame analytical (5a), all-rows mj_jac cross-check (5b),
  condim=6 torsional+rolling (5c), two-body 6×12 (5d), identity regression (5e),
  frictionless condim=1 (5f)
- All 402 sim-core tests pass, clippy clean, fmt clean
