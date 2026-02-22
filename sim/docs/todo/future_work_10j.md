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

## DT-74 Spec: Fix `compute_body_jacobian_at_point()` — Full Body Jacobian API

### Context

`compute_body_jacobian_at_point()` (mujoco_pipeline.rs:14182) is dead code with
fundamental correctness bugs. It was intended to compute the translational
Jacobian at a world-frame point on a body, but:

1. **Returns 1×nv instead of 3×nv** — only stores `.x` component of each column
2. **Free joint angular DOFs use world-frame unit vectors** (`e_x, e_y, e_z`)
   instead of body-frame `R*e_i` (MuJoCo's `cdof` convention)
3. **Free joint linear DOFs hardcode `(1,0,0)`** instead of 3×3 identity
4. **Marked `#[allow(dead_code)]`** — zero callers

Meanwhile, the codebase already has **correct** implementations:
- `mj_jac_site` (line 9817): `(3×nv, 3×nv)` — correct R*e_i for ball/free
- `mj_jac_point` (line 9895): `6×nv` combined — correct R*e_i
- `mj_jac_body_com` (line 9968): thin wrapper on `mj_jac_point`
- `mj_jac_geom` (line 9973): thin wrapper on `mj_jac_point`
- `accumulate_point_jacobian` (line 9746): scalar projection — correct R*e_i

But MuJoCo's **core** function `mj_jac(m, d, jacp, jacr, point, body)` has no
direct equivalent. The existing functions are specialized variants.

### MuJoCo Reference

MuJoCo's `mj_jac` (engine_core_util.c):
- **Signature**: `mj_jac(m, d, jacp, jacr, point, body)` → two 3×nv matrices
- **`jacp`** (3×nv): translational Jacobian — velocity of `point` per unit DOF
- **`jacr`** (3×nv): rotational Jacobian — angular velocity per unit DOF
- Both nullable (caller can request only one)

Per-joint-type formulas (using precomputed `cdof`, but we compute inline):

| Joint | `jacp` column | `jacr` column |
|-------|--------------|--------------|
| **Hinge** | `axis × r` | `axis` |
| **Slide** | `axis` | `0` |
| **Ball** (3 DOFs) | `(R*e_i) × r` | `R*e_i` |
| **Free** lin (DOFs 0-2) | `e_i` (world identity) | `0` |
| **Free** ang (DOFs 3-5) | `(R*e_i) × r` | `R*e_i` |

Where `axis = R_body * jnt_axis`, `r = point - joint_anchor`, `R = xquat[body].to_rotation_matrix()`.

All variant functions are thin wrappers:
- `mj_jacBody(body)` → `mj_jac(point=xpos[body], body)`
- `mj_jacBodyCom(body)` → `mj_jac(point=xipos[body], body)`
- `mj_jacSite(site)` → `mj_jac(point=site_xpos[site], body=site_body[site])`
- `mj_jacGeom(geom)` → `mj_jac(point=geom_xpos[geom], body=geom_body[geom])`

### Plan

#### Step 1: Add `mj_jac` as the canonical Jacobian function

Add a new public function `mj_jac` that is the single source of truth for the
body-chain Jacobian walk. Signature:

```rust
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

- **`mj_jac_site`** → calls `mj_jac(model, data, site_body[site_id], &site_xpos[site_id])`
- **`mj_jac_point`** (6×nv) → calls `mj_jac`, combines into 6×nv (rows 0-2 = angular, 3-5 = linear)
- **`mj_jac_body_com`** → calls `mj_jac(model, data, body_id, &xipos[body_id])`, converts to 6×nv
- **`mj_jac_geom`** → calls `mj_jac(model, data, geom_body[geom_id], &geom_xpos[geom_id])`, converts to 6×nv

This eliminates the duplicated 80-line chain-walk in `mj_jac_point` (lines
9895–9965) and ensures all paths share one implementation.

#### Step 3: Delete `compute_body_jacobian_at_point`

Remove the broken dead-code function (lines 14177–14259). It has zero callers
and its functionality is superseded by `mj_jac`.

#### Step 4: Export `mj_jac` from the crate public API

Add `mj_jac` to the re-exports in `sim/L0/core/src/lib.rs` alongside the
existing `mj_jac_point` and `mj_jac_site` exports (line 167-168).

#### Step 5: Tests

All tests in `sim-core` and `sim-conformance-tests`.

**5a. Unit tests — `mj_jac` correctness per joint type:**

For each joint type (Hinge, Slide, Ball, Free), build a minimal single-joint
model at a non-trivial qpos, call `mj_jac`, and verify:

| Test | Assertion |
|------|-----------|
| `mj_jac_hinge_basic` | `jacp[:,0] = axis × r`, `jacr[:,0] = axis` |
| `mj_jac_slide_basic` | `jacp[:,0] = axis`, `jacr[:,0] = 0` |
| `mj_jac_ball_body_frame_axes` | `jacp[:,i] = (R*e_i) × r`, `jacr[:,i] = R*e_i` for i in {0,1,2} |
| `mj_jac_free_translation` | `jacp[:,0:3] = I_3`, `jacr[:,0:3] = 0` |
| `mj_jac_free_rotation_body_frame` | `jacp[:,3+i] = (R*e_i) × r`, `jacr[:,3+i] = R*e_i` — **explicitly verify NOT world-frame** by testing at non-identity orientation |
| `mj_jac_world_body_returns_zeros` | body_id=0 → both matrices all zeros |

**5b. Unit tests — wrapper consistency:**

| Test | Assertion |
|------|-----------|
| `mj_jac_site_delegates_to_mj_jac` | `mj_jac_site` output == `mj_jac(site_body, site_pos)` |
| `mj_jac_point_combines_correctly` | `mj_jac_point` rows 0-2 == `jacr`, rows 3-5 == `jacp` |
| `mj_jac_body_com_consistent` | `mj_jac_body_com(body)` == `mj_jac_point(body, xipos[body])` |

**5c. Regression — existing `jac_site_tests` pass unchanged:**

The existing tests at line 24296 (`jac_site_agrees_with_accumulate_hinge`,
`jac_site_slide_joint`) must continue to pass, confirming the refactor is
behavior-preserving.

**5d. Multi-joint chain test:**

Build a 3-body chain (free → hinge → hinge) with non-trivial angles. Verify
`mj_jac` at a point on body 3 produces a 3×nv matrix with correct columns for
all 8 DOFs (6 free + 2 hinge). Cross-check translational Jacobian against
finite-difference: perturb each `qpos` by epsilon, re-run FK, measure
`delta_point / epsilon ≈ jacp[:,dof]`.

**5e. Finite-difference validation (gold standard):**

For a free-joint + ball-joint chain at random non-identity orientation:
- Perturb each DOF by `epsilon = 1e-7`
- Re-run `mj_fwd_position`
- Compute `(point_new - point_old) / epsilon`
- Assert `≈ jacp[:, dof]` within `1e-5` relative tolerance

This catches any frame convention errors that unit tests might miss.

### Files Modified

| File | Change |
|------|--------|
| `sim/L0/core/src/mujoco_pipeline.rs` | Add `mj_jac`, refactor `mj_jac_site`/`mj_jac_point`/`mj_jac_body_com`/`mj_jac_geom` as wrappers, delete `compute_body_jacobian_at_point`, add tests |
| `sim/L0/core/src/lib.rs` | Export `mj_jac` |

### Verification

```bash
cargo test -p sim-core -- jac       # all Jacobian tests
cargo test -p sim-core              # full crate regression
cargo clippy -p sim-core -- -D warnings
```
