# Future Work 10j — Deferred Item Tracker: Group 9 — Misc Pipeline & API

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 9 — Misc Pipeline & API (13 items)

**Spec approach:** DT-74/75 need individual specs (T3 — Jacobian correctness bugs,
need formula derivation). DT-79/82/83 each need individual specs (T3 — API design
or data layout architecture). DT-77/78 share a "Length-Range Estimation" spec with
DT-59 (T2). The rest (DT-76/80/81/84/91/92) implement directly (T1). Totals:
6 T1, 2 T2, 5 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-74 | §4 | `compute_body_jacobian_at_point()` incomplete — only x-component, `#[allow(dead_code)]` | Medium | T3 |
| DT-75 | §4 | `add_body_jacobian` free joint bug — world-frame unit vectors instead of body-frame `R*e_i` | Medium | T3 |
| DT-76 | §8 | Pre-allocated `efc_lambda_saved` for RK4 — avoid `efc_lambda.clone()` per step | Low | T1 |
| DT-77 | §5 | Length-range auto-estimation for site-transmission muscle actuators (no-op stub) | Low | T2 |
| DT-78 | §4 | `actuator_lengthrange` for unlimited spatial tendons — wrap-array DOF lookup wrong | Low | T2 |
| DT-79 | §14 | User callbacks `mjcb_*` Rust equivalents — closures vs trait objects, thread safety | Medium | T3 |
| DT-80 | §14 | Mocap body + equality weld constraint integration testing | Low | T1 |
| DT-81 | §14 | `key_userdata` support — no `userdata` concept in CortenForge | Low | T1 |
| DT-82 | §9 | SoA layout across environments for cache locality — deferred to GPU work | Low | T3 |
| DT-83 | §9 | Multi-model batching — different robots in same batch (per-env dimensions) | Low | T3 |
| DT-84 | §32 | `mju_encodePyramid` utility not implemented — API compatibility only | Low | T1 |
| DT-91 | §2 | Warmstart `Vec<f64>` → `SmallVec<[f64; 6]>` — avoid heap allocation in warmstart vectors | Low | T1 |
| DT-92 | §9 | Parallel reset for `BatchSim` — sequential O(nq+nv+nu+na) reset deferred | Low | T1 |

---

## DT-74 Spec: Canonical `mj_jac` — Full Body Jacobian API

### Context

`compute_body_jacobian_at_point()` (mujoco_pipeline.rs:14182) is dead code with
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

**File:** `sim/L0/core/src/mujoco_pipeline.rs`, near line 9817.

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
| `sim/L0/core/src/mujoco_pipeline.rs` | Add `mj_jac` (D1), add `mj_jac_body` (D2), refactor `mj_jac_site`/`mj_jac_point`/`mj_jac_body_com`/`mj_jac_geom` as wrappers (D3–D6), delete `compute_body_jacobian_at_point` (D7), add `mj_jac_tests` module (D9) |
| `sim/L0/core/src/lib.rs` | Export `mj_jac`, `mj_jac_body` (D8) |

### Verification

```bash
cargo test -p sim-core -- jac          # all Jacobian tests (old + new)
cargo test -p sim-core                 # full crate regression
cargo clippy -p sim-core -- -D warnings
cargo fmt --all -- --check
```
