# Spec A — acc0, dampratio, and length-range estimation

**Status:** Draft
**Phase:** Roadmap Phase 5 — Actuator Completeness
**Effort:** L
**MuJoCo ref:** `set0()`, `mj_setM0()`, `mj_setLengthRange()`, `evalAct()` in
`engine_setconst.c`
**MuJoCo version:** 3.2.6 (google-deepmind/mujoco `main` branch)
**Prerequisites:**
- T1-b (§63) `dynprm` resize landed in `d4db634`
- T1-a (DT-6) `actearly` verification landed in `dc12b8b`

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the features described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth.

---

## Problem Statement

Four MuJoCo conformance gaps in actuator parameter computation:

1. **DT-57 (acc0 for non-muscle actuators):** MuJoCo computes `acc0 =
   ||M^{-1} J||` for **every** actuator in `set0()` at
   `engine_setconst.c`. CortenForge only computes `acc0` for
   `ActuatorDynamics::Muscle` actuators. All non-muscle actuators have
   `acc0 = 0.0`, which is incorrect — MuJoCo computes a nonzero value used
   for gain normalization and dampratio.

2. **DT-56 (dampratio for position actuators):** MuJoCo's `set0()` converts
   positive `biasprm[2]` to negative damping for position-like actuators
   using `dof_M0` (CRB diagonal mass) and the actuator transmission. This
   enables the `dampratio` MJCF attribute (alternative to explicit `kv`):
   `kv = dampratio * 2 * sqrt(kp * reflected_inertia)`. CortenForge does
   not implement this conversion — models using `dampratio` produce zero
   damping.

3. **DT-59 (lengthrange for unlimited slide joints):** MuJoCo's
   `mj_setLengthRange()` falls through to a simulation-based estimation
   (`evalAct`) when a joint is unlimited. CortenForge only copies from
   joint/tendon limits, leaving unlimited joints with `lengthrange = (0, 0)`.

4. **DT-77 (site-transmission lengthrange):** Site transmissions have no
   joint/tendon limits to copy from. MuJoCo always uses simulation-based
   estimation for sites. CortenForge leaves site-transmission actuators with
   `lengthrange = (0, 0)`.

All four are **conformance gaps** — MuJoCo implements these; we don't.

---

## MuJoCo Reference

> **This is the most important section of the spec.**

### `set0()` in `engine_setconst.c` (approx. lines 350–500)

`set0()` is called by `mj_setConst()` and runs at `qpos0` after FK, CRBA,
and transmission. It computes three actuator quantities:

#### acc0 computation (DT-57)

```c
// engine_setconst.c — set0()
for (int i=0; i < m->nu; i++) {
    mju_sparse2dense(moment, d->actuator_moment, 1, nv,
                     d->moment_rownnz + i,
                     d->moment_rowadr + i, d->moment_colind);
    mj_solveM(m, d, tmp, moment, 1);
    m->actuator_acc0[i] = mju_norm(tmp, nv);
}
```

For **every** actuator (not just muscles):
1. Extract the dense moment vector `J_i` (row `i` of actuator moment matrix —
   the transmission Jacobian `d(actuator_length_i)/d(qvel)`).
2. Solve `M * x = J_i` using pre-factored `L^T D L`.
3. `acc0[i] = ||x||_2 = ||M^{-1} J_i||_2`.

Physical meaning: the Euclidean norm of the joint-space acceleration produced
by a unit force through actuator `i`'s transmission. Measures how "effective"
a unit actuator force is given the system's inertia and the actuator's
geometric coupling.

**Edge cases:**
- `nv == 0`: `acc0 = 0` for all actuators (norm of empty vector).
- Zero transmission (moment is all zeros): `x = 0`, `acc0 = 0`.
- Body transmission at `qpos0`: moment is all zeros (no contacts), `acc0 = 0`.

#### dampratio-to-damping conversion (DT-56)

```c
// engine_setconst.c — set0(), after acc0 loop
for (int i=0; i < m->nu; i++) {
    mjtNum* biasprm = m->actuator_biasprm + i*mjNBIAS;
    mjtNum* gainprm = m->actuator_gainprm + i*mjNGAIN;

    // not a position-like actuator: skip
    if (gainprm[0] != -biasprm[1]) {
        continue;
    }

    // damping is 0 or negative (interpreted as regular "kv"): skip
    if (biasprm[2] <= 0) {
        continue;
    }

    // interpret biasprm[2] > 0 as dampratio
    int rownnz = d->moment_rownnz[i];
    int rowadr = d->moment_rowadr[i];
    mjtNum* transmission = d->actuator_moment + rowadr;
    mjtNum mass = 0;
    for (int j=0; j < rownnz; j++) {
        mjtNum trn = mju_abs(transmission[j]);
        mjtNum trn2 = trn*trn;
        if (trn2 > mjMINVAL) {
            int dof = d->moment_colind[rowadr + j];
            mass += m->dof_M0[dof] / trn2;
        }
    }

    mjtNum damping = biasprm[2] * 2 * mju_sqrt(gainprm[0] * mass);
    biasprm[2] = -damping;
}
```

Algorithm:
1. **Position-actuator fingerprint:** `gainprm[0] != -biasprm[1]` → skip.
   MuJoCo uses **exact floating-point `!=`** (not approximate). This is correct
   because both `gainprm[0]` and `biasprm[1]` are set from the same `kp` value
   during model compilation — `gainprm[0] = kp` and `biasprm[1] = -kp` — so
   the bit patterns are exactly `kp` and `-(kp)`, making `gainprm[0] == -biasprm[1]`
   an exact `true` for any position actuator. Non-position actuators have
   independently-set gain/bias that won't match exactly. **Our port must use
   exact `!=` comparison, not approximate.** For a position actuator,
   `gainprm[0] = kp` and `biasprm = [0, -kp, -kv]`, so this checks `kp == kp`.
2. **dampratio vs explicit kv:** `biasprm[2] <= 0` means already-negative kv
   (explicit damping). Only `biasprm[2] > 0` is interpreted as a damping ratio.
3. **Reflected inertia:** For each non-zero element in the sparse moment row:
   `mass += dof_M0[dof] / trn_j^2`. This is the effective mass seen by the
   actuator, inversely scaled by the transmission ratio squared.
4. **Critical damping formula:** `damping = dampratio * 2 * sqrt(kp * mass)`.
   For `dampratio = 1.0`, this gives exactly critical damping.
5. **Sign flip:** `biasprm[2] = -damping` (bias force convention:
   `bias = b0 + b1*length + b2*velocity`, damping opposes velocity).

**Edge cases:**
- `gainprm[0] != -biasprm[1]`: not position-like → skip. This covers motor,
  velocity, damper, cylinder, muscle, and general actuators with non-matching
  gain/bias.
- `biasprm[2] <= 0`: already explicit kv → skip. No mutation.
- `biasprm[2] == 0`: zero damping → skip (not positive).
- Zero transmission (`trn2 <= mjMINVAL`): that DOF is excluded from reflected
  inertia sum. If all DOFs are zero, `mass = 0`, `damping = 0`,
  `biasprm[2] = 0` (effectively no damping).
- Multi-DOF transmissions (e.g., tendon across multiple joints): reflected
  inertia sums contributions from each DOF, weighted by inverse-square
  transmission.

#### `dof_M0` computation — `mj_setM0()` in `engine_setconst.c` (approx. lines 300–340)

```c
static void mj_setM0(mjModel* m, mjData* d) {
    mjtNum buf[6];
    mjtNum* crb = d->crb;

    // copy cinert into crb
    mju_copy(crb, d->cinert, 10*m->nbody);

    // backward pass: accumulate composite inertias
    for (int i = m->nbody-1; i > 0; i--) {
        if (m->body_parentid[i] > 0) {
            mju_addTo(crb + 10*m->body_parentid[i], crb + 10*i, 10);
        }
    }

    // dof_M0 = armature + cdof . (crb * cdof)
    for (int i = 0; i < m->nv; i++) {
        mju_mulInertVec(buf, crb + 10*m->dof_bodyid[i], d->cdof + 6*i);
        m->dof_M0[i] = m->dof_armature[i] + mju_dot(d->cdof + 6*i, buf, 6);
    }
}
```

`dof_M0[i]` is the **diagonal element** of the joint-space mass matrix at
`qpos0`, computed via the Composite Rigid Body (CRB) algorithm. Numerically
equivalent to `qM[(i,i)]` from our CRBA, but MuJoCo computes it separately
using a compact 10-element spatial inertia representation (not the full 6×6
matrix).

**Equivalence:** After CRBA at `qpos0`, `dof_M0[i] == qM[(i,i)]`. We can
read the diagonal directly from `qM` instead of duplicating the CRB pass.

**Edge cases:**
- `nv == 0`: empty `dof_M0` array.
- World body (`body_parentid[i] == 0`): not accumulated to parent
  (guard: `body_parentid[i] > 0`).

### `mj_setLengthRange()` in `engine_setconst.c` (approx. lines 100–250)

```c
int mj_setLengthRange(mjModel* m, mjData* d, int index,
                      const mjLROpt* opt, char* error, int error_sz);
```

Returns 1 on success, 0 on error (convergence failure).

**Step 1: Mode filtering**
```c
int ismuscle = (m->actuator_gaintype[index] == mjGAIN_MUSCLE ||
                m->actuator_biastype[index] == mjBIAS_MUSCLE);
int isuser = (m->actuator_gaintype[index] == mjGAIN_USER ||
              m->actuator_biastype[index] == mjBIAS_USER);
if ((opt->mode == mjLRMODE_NONE) ||
    (opt->mode == mjLRMODE_MUSCLE && !ismuscle) ||
    (opt->mode == mjLRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
    return 1;  // skip
}
```

**Step 2: Use existing range**
```c
if (opt->useexisting &&
    m->actuator_lengthrange[2*index] < m->actuator_lengthrange[2*index+1]) {
    return 1;
}
```

**Step 3: Copy from joint/tendon limits**
```c
if (opt->uselimit) {
    if (trntype == mjTRN_JOINT || trntype == mjTRN_JOINTINPARENT) {
        if (m->jnt_limited[threadid]) {
            m->actuator_lengthrange[2*index]   = m->jnt_range[2*threadid];
            m->actuator_lengthrange[2*index+1] = m->jnt_range[2*threadid+1];
            return 1;
        }
    }
    if (trntype == mjTRN_TENDON) {
        if (m->tendon_limited[threadid]) {
            m->actuator_lengthrange[2*index]   = m->tendon_range[2*threadid];
            m->actuator_lengthrange[2*index+1] = m->tendon_range[2*threadid+1];
            return 1;
        }
    }
}
```

For unlimited joints/tendons: falls through to Step 4.
For site transmissions: not in Step 3 at all → always Step 4.

**Step 4: Simulation-based estimation**

Two runs (`side = 0` for min, `side = 1` for max):
```c
for (side = 0; side < 2; side++) {
    mj_resetData(m, d);  // init at qpos0
    int updated = 0;
    while (d->time < opt->inttotal) {
        mjtNum len = evalAct(m, d, index, side, opt);
        if (d->time == 0) return 0;  // instability
        if (d->time > opt->inttotal - opt->interval) {
            if (len < lmin[side] || !updated) lmin[side] = len;
            if (len > lmax[side] || !updated) lmax[side] = len;
            updated = 1;
        }
    }
    lengthrange[side] = (side == 0 ? lmin[side] : lmax[side]);
}
```

**Step 5: Convergence check**
```c
mjtNum dif = lengthrange[1] - lengthrange[0];
if (dif <= 0) return 0;         // invalid range
if (lmax[0]-lmin[0] > opt->tolrange * dif) return 0;  // didn't converge
if (lmax[1]-lmin[1] > opt->tolrange * dif) return 0;  // didn't converge
```

### `evalAct()` in `engine_setconst.c` (approx. lines 50–100)

Inner simulation step for length-range estimation:

```c
static mjtNum evalAct(const mjModel* m, mjData* d, int index, int side,
                      const mjLROpt* opt) {
    int nv = m->nv;
    mjtNum nrm;

    // 1. Velocity damping
    for (int i = 0; i < nv; i++)
        d->qvel[i] *= mju_exp(-m->opt.timestep / mju_max(0.01, opt->timeconst));

    // 2. Forward pass (full pipeline: FK, collision, gravity, passive, actuation)
    mj_step1(m, d);

    // 3. Build dense moment vector for actuator i
    mju_sparse2dense(moment, d->actuator_moment + d->moment_rowadr[index], ...);

    // 4. Compute applied force
    mj_solveM(m, d, d->qfrc_applied, moment, 1);  // tmp = M^{-1} * moment
    nrm = mju_norm(d->qfrc_applied, nv);
    // Scale: force = ±accel * moment / ||M^{-1} * moment||
    mju_scl(d->qfrc_applied, moment, (2*side-1)*opt->accel/mju_max(mjMINVAL, nrm), nv);

    // 5. Force capping
    if (opt->maxforce > 0) {
        nrm = mju_norm(d->qfrc_applied, nv);
        if (nrm > opt->maxforce)
            mju_scl(d->qfrc_applied, d->qfrc_applied, opt->maxforce/nrm, nv);
    }

    // 6. Integration step
    mj_step2(m, d);

    return d->actuator_length[index];
}
```

Key details:
- **Velocity damping:** `qvel *= exp(-dt / max(0.01, timeconst))` prevents
  runaway oscillation.
- **Force direction:** `moment` (the transmission Jacobian) — scaled to produce
  target acceleration magnitude `accel`.
- **Force magnitude normalization:** `accel / ||M^{-1} J||` ensures the
  *acceleration* magnitude is constant regardless of the actuator's mechanical
  advantage.
- **Force environment — gravity, contacts, passive forces ARE active:**
  MuJoCo's `evalAct` calls the full `mj_step1`/`mj_step2` pipeline with NO
  modifications to the model. Specifically:
  - `mj_setLengthRange()` does NOT modify `m->opt.gravity`, does NOT set
    `mjDSBL_GRAVITY`, does NOT disable collision, does NOT zero passive forces.
  - `mj_step1` calls `mj_fwdPosition` (FK + collision detection) and
    `mj_fwdVelocity` (which calls `mj_rne` → gravity is propagated into
    `d->qfrc_bias` via `cacc[world] = -gravity`).
  - `mj_step2` calls `mj_fwdAcceleration` which sums:
    `qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator`.
    Gravity (in `qfrc_bias`), contacts (in `qfrc_constraint`), and passive
    forces (in `qfrc_passive`) are all active.
  - `qfrc_actuator` is zero because `mj_step1` calls `mj_fwd_actuation` which
    zeros it (no ctrl input during LR estimation — `mj_resetData` zeros ctrl).
  - The applied force (`qfrc_applied`, set by evalAct) overwhelms gravity by
    design: default `accel=20 m/s²` ≈ 2× Earth gravity (9.81 m/s²). The
    velocity damping (`timeconst=1.0`) dissipates kinetic energy so the system
    settles rather than oscillating.
  - **For conformance, our implementation must also run with full gravity,
    contacts, and passive forces active.** The applied force is large enough
    to overwhelm them, matching MuJoCo's design.
- **Both sides:** `side=0` pushes toward minimum length (negative force),
  `side=1` toward maximum.

**Compilation pipeline context:**
- `set0()` is called by `mj_setConst()` → `mj_setConst()` is called during
  model compilation after FK and CRBA at `qpos0`. All `qM`, `qLD`, and
  `actuator_moment` data is available.
- `mj_setLengthRange()` is a separate public API, called after `mj_setConst()`
  during model compilation. It is NOT called from within `mj_setConst`. In
  MuJoCo's `compiler.cc`, the call sequence is approximately:
  `mj_setConst(m, d)` → `mj_setLengthRange(m, d, i, &opt, ...)` for each
  actuator that needs it.
- Both run at compile time on `qpos0` — not during forward simulation.

### `mjLROpt` defaults

| Field | Default | Type | Description |
|-------|---------|------|-------------|
| `mode` | `mjLRMODE_MUSCLE` (1) | enum | Which actuators to compute for. Values: NONE=0, MUSCLE=1, MUSCLEUSER=2, ALL=3. |
| `useexisting` | `true` | bool | Skip if range already set |
| `uselimit` | `true` | bool | Copy from joint/tendon limits if limited |
| `accel` | `20.0` | f64 | Target acceleration magnitude |
| `maxforce` | `0.0` | f64 | Force cap (0 = unlimited) |
| `timeconst` | `1.0` | f64 | Velocity damping time constant (s) |
| `timestep` | `0.01` | f64 | Internal simulation timestep (s) |
| `inttotal` | `10.0` | f64 | Total simulation time (s) |
| `interval` | `2.0` | f64 | Measurement interval (last N s) |
| `tolrange` | `0.05` | f64 | Convergence tolerance (fraction of range) |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| acc0 for non-muscle actuators | Computed for all `i < m->nu` in `set0()` | Only for `ActuatorDynamics::Muscle`; all others = 0.0 |
| dampratio → damping conversion | `set0()`: positive `biasprm[2]` → `-2*dampratio*sqrt(kp*mass)` for position-like actuators | Not implemented; positive `biasprm[2]` remains positive (wrong sign, wrong magnitude) |
| `dof_M0` diagonal mass | Computed by `mj_setM0()` via CRB at qpos0 | Does not exist as field; equivalent `qM[(i,i)]` available after CRBA |
| lengthrange for unlimited joints | Simulation-based estimation via `evalAct()` | Not implemented; unlimited joints → `lengthrange = (0, 0)` |
| lengthrange for site transmissions | Simulation-based estimation via `evalAct()` | Not implemented; sites → `lengthrange = (0, 0)` |
| `mjLROpt` / `<lengthrange>` XML element | Configurable options for LR estimation | Not parsed |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `actuator_moment` storage | Sparse CSR: `moment_rownnz`, `moment_rowadr`, `moment_colind` | Dense: `Vec<DVector<f64>>` (one `DVector` per actuator) | `data.actuator_moment` is populated by `mj_fwd_actuation` (acceleration stage), NOT by `step1` alone. For acc0 and dampratio computation (which runs at model compile time, not runtime), we construct moment vectors manually from transmission type — same approach as existing `compute_muscle_params`. The `build_actuator_moment` helper encapsulates this. |
| `dof_M0` | Separate `m->dof_M0` field (10-element CRB representation) | Use `data.qM[(i,i)]` diagonal after CRBA | Read `qM` diagonal instead of separate CRB pass. Numerically equivalent at qpos0. |
| `biasprm`/`gainprm` indexing | `m->actuator_biasprm + i*mjNBIAS` (flat array, `mjNBIAS=9`) | `model.actuator_biasprm[i]` (Vec of `[f64; 9]`) | Direct index: `model.actuator_biasprm[i][k]` replaces `biasprm[k]` |
| `mjMINVAL` | `1e-15` (minimum value guard) | Use `1e-15_f64` constant or `f64::EPSILON` equivalent | Use `1e-15` for `trn2 > mjMINVAL` guard to match MuJoCo |
| `actuator_lengthrange` storage | Flat `[2*i]`/`[2*i+1]` | `Vec<(f64, f64)>` tuple | `.0` = min, `.1` = max |
| `mj_solveM(m, d, x, rhs, 1)` | Sparse solve `M x = rhs` | `mj_solve_sparse(rowadr, rownnz, colind, qLD_data, qLD_diag_inv, &mut x)` where `x` starts as `rhs` | Copy `rhs` into `x`, then call `mj_solve_sparse` in-place |
| `step1` / `step2` pipeline | `mj_step1` = pos+vel stages + control callback; `mj_step2` = acc + integration | `data.step1(model)` / `data.step2(model)` | Direct port. For evalAct, run with full gravity/contacts/passive — MuJoCo does NOT disable these. |
| LR mode enum | `mjLRMODE_NONE=0, mjLRMODE_MUSCLE=1, mjLRMODE_MUSCLEUSER=2, mjLRMODE_ALL=3` | New `LengthRangeMode` enum | Direct mapping |

---

## Specification

### S1. Extend `acc0` computation to all actuators (DT-57)

**File:** `sim/L0/core/src/forward/muscle.rs`
**MuJoCo equivalent:** `set0()` in `engine_setconst.c`, acc0 loop
**Design decision:** Remove the `ActuatorDynamics::Muscle` guard from the acc0
loop. The existing acc0 infrastructure (FK → CRBA → sparse solve → norm) is
correct and complete for all transmission types. Only the muscle-only filter
needs removal. The function is renamed to `compute_actuator_params()` to
reflect its broader scope (per Contract 2 in the umbrella).

**Before** (current code, `muscle.rs:131–143`):
```rust
pub fn compute_muscle_params(&mut self) {
    if self.nu == 0 {
        return;
    }

    let has_muscles =
        (0..self.nu).any(|i| self.actuator_dyntype[i] == ActuatorDynamics::Muscle);
    if !has_muscles {
        return;
    }
    // ... Phase 1 (lengthrange) + Phase 2 (acc0) + Phase 3 (F0)
    // All gated by: if self.actuator_dyntype[i] != ActuatorDynamics::Muscle { continue; }
```

**After:**
```rust
/// Compute actuator parameters: acc0, dampratio, lengthrange, and F0.
///
/// For each actuator:
///   1. Computes `actuator_lengthrange` from joint/tendon limits or simulation.
///   2. Runs a forward pass at `qpos0` to get M, then computes
///      `acc0 = ||M^{-1} * moment||` for every actuator.
///   3. Converts `dampratio` to damping for position-like actuators.
///   4. Resolves muscle `F0` (gainprm[2]) when `force < 0`: `F0 = scale / acc0`.
///
/// Must be called after `compute_ancestors()` and `compute_implicit_params()`,
/// and after all tendon/actuator fields are populated.
pub fn compute_actuator_params(&mut self) {
    if self.nu == 0 {
        return;
    }

    // --- Phase 1: Compute actuator_lengthrange from limits ---
    // (existing muscle-only lengthrange code, extended to all actuators
    //  that need it — see S4 for simulation-based extension)
    for i in 0..self.nu {
        // lengthrange from limits: applies to all actuator types, not just muscles
        let gear = self.actuator_gear[i][0];
        let scale_range = |lo: f64, hi: f64| -> (f64, f64) {
            let a = gear * lo;
            let b = gear * hi;
            (a.min(b), a.max(b))
        };
        match self.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = self.actuator_trnid[i][0];
                if jid < self.njnt && self.jnt_limited[jid] {
                    let (lo, hi) = self.jnt_range[jid];
                    self.actuator_lengthrange[i] = scale_range(lo, hi);
                }
            }
            ActuatorTransmission::Tendon => {
                let tid = self.actuator_trnid[i][0];
                if tid < self.ntendon && self.tendon_limited[tid] {
                    let (lo, hi) = self.tendon_range[tid];
                    self.actuator_lengthrange[i] = scale_range(lo, hi);
                } else if tid < self.ntendon {
                    // Unlimited tendon: existing fixed-tendon estimation
                    // (unchanged from current code)
                    if self.tendon_type[tid] == TendonType::Fixed {
                        let adr = self.tendon_adr[tid];
                        let num = self.tendon_num[tid];
                        let (mut lmin, mut lmax) = (0.0, 0.0);
                        for w in adr..(adr + num) {
                            let dof = self.wrap_objid[w];
                            let coef = self.wrap_prm[w];
                            if let Some(jid) =
                                (0..self.njnt).find(|&j| self.jnt_dof_adr[j] == dof)
                            {
                                let (qlo, qhi) = self.jnt_range[jid];
                                if coef >= 0.0 {
                                    lmin += coef * qlo;
                                    lmax += coef * qhi;
                                } else {
                                    lmin += coef * qhi;
                                    lmax += coef * qlo;
                                }
                            }
                        }
                        self.actuator_lengthrange[i] = scale_range(lmin, lmax);
                    }
                    // Spatial tendons and unlimited: handled by S4 simulation
                }
            }
            ActuatorTransmission::Site | ActuatorTransmission::Body => {
                // Site: handled by S4 simulation-based estimation
                // Body: no length concept at qpos0
            }
        }
    }

    // --- Phase 2: Forward pass at qpos0 for acc0 (ALL actuators) ---
    let mut data = self.make_data();
    mj_fwd_position(self, &mut data);
    mj_crba(self, &mut data);

    for i in 0..self.nu {
        // NO muscle-only guard — compute acc0 for every actuator
        let gear = self.actuator_gear[i][0];
        let mut j_vec = DVector::zeros(self.nv);

        // Build moment vector J (same logic as current code, all transmission types)
        match self.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = self.actuator_trnid[i][0];
                if jid < self.njnt {
                    let dof_adr = self.jnt_dof_adr[jid];
                    j_vec[dof_adr] = gear;
                }
            }
            ActuatorTransmission::Tendon => {
                // (existing tendon moment code — unchanged)
                let tid = self.actuator_trnid[i][0];
                if tid < self.ntendon {
                    match self.tendon_type[tid] {
                        TendonType::Fixed => {
                            let adr = self.tendon_adr[tid];
                            let num = self.tendon_num[tid];
                            for w in adr..(adr + num) {
                                let dof_adr = self.wrap_objid[w];
                                let coef = self.wrap_prm[w];
                                if dof_adr < self.nv {
                                    j_vec[dof_adr] = gear * coef;
                                }
                            }
                        }
                        TendonType::Spatial => {
                            for dof in 0..self.nv {
                                j_vec[dof] = gear * data.ten_J[tid][dof];
                            }
                        }
                    }
                }
            }
            ActuatorTransmission::Site => {
                // (existing site moment code — unchanged)
                let sid = self.actuator_trnid[i][0];
                let refid = self.actuator_trnid[i][1];
                let (jac_t, jac_r) = mj_jac_site(self, &data, sid);
                let full_gear = self.actuator_gear[i];

                if refid == usize::MAX {
                    let wrench_t = data.site_xmat[sid]
                        * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                    let wrench_r = data.site_xmat[sid]
                        * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                    for dof in 0..self.nv {
                        j_vec[dof] = jac_t.column(dof).dot(&wrench_t)
                            + jac_r.column(dof).dot(&wrench_r);
                    }
                } else {
                    // (existing difference-Jacobian + ancestor-zeroing code)
                    // ... unchanged ...
                }
            }
            ActuatorTransmission::Body => {
                // At qpos0: no contacts → J is zero. No-op.
            }
        }

        // Solve M * x = J
        let mut x = j_vec;
        let (rowadr, rownnz, colind) = self.qld_csr();
        mj_solve_sparse(
            rowadr, rownnz, colind,
            &data.qLD_data, &data.qLD_diag_inv,
            &mut x,
        );

        // acc0 = ||M^{-1} J||_2
        self.actuator_acc0[i] = x.norm().max(1e-10);
    }

    // --- Phase 3: dampratio conversion (S3) ---
    // (see S3 below)

    // --- Phase 4: Resolve muscle F0 ---
    for i in 0..self.nu {
        if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
            continue;
        }
        if self.actuator_gainprm[i][2] < 0.0 {
            self.actuator_gainprm[i][2] =
                self.actuator_gainprm[i][3] / self.actuator_acc0[i];
            self.actuator_biasprm[i][2] = self.actuator_gainprm[i][2];
        }
    }
}
```

The key change is the removal of two guards:
1. The early return `if !has_muscles { return; }` — models with no muscles now
   still compute acc0 for all actuators.
2. The `if dyntype != Muscle { continue; }` guard in the acc0 loop — every
   actuator gets acc0.

### S2. Rename callers: `compute_muscle_params` → `compute_actuator_params`

**Files:**
- `sim/L0/mjcf/src/builder/mod.rs` (or wherever `compute_muscle_params` is called)
- `sim/L0/core/src/forward/muscle.rs` (the function itself)
- Any test helpers that call it

**MuJoCo equivalent:** N/A (naming only)
**Design decision:** The function now processes all actuator types, not just
muscles. The rename reflects the broader scope and prevents future confusion.

Find every call site of `compute_muscle_params()` and rename to
`compute_actuator_params()`. This is a mechanical search-and-replace.
Known call sites:
- `sim/L0/mjcf/src/builder/mod.rs` (model compilation)
- `sim/L0/core/src/forward/muscle.rs:486` (`build_muscle_model_joint` test helper)
- `sim/L0/core/src/forward/muscle.rs:898` (`test_f0_explicit_not_overridden`)

### S3. dampratio-to-damping conversion (DT-56)

**File:** `sim/L0/core/src/forward/muscle.rs` (inside `compute_actuator_params`)
**MuJoCo equivalent:** `set0()` dampratio loop in `engine_setconst.c`
**Design decision:** Insert the dampratio loop between the acc0 computation and
the F0 resolution. It uses `qM` diagonal (equivalent to `dof_M0`) and the
dense actuator moment vectors already computed. No separate `dof_M0` model
field needed — we read `data.qM[(i,i)]` directly.

**Why no separate `dof_M0` field:** MuJoCo stores `dof_M0` separately because
its compact 10-element CRB representation is cheaper than its full mass matrix.
Our CRBA already builds the full `qM` (dense `DMatrix<f64>`) and the diagonal
is equivalent at `qpos0`. Introducing a separate field would duplicate data
without numerical benefit. If performance becomes a concern (hot path, many
actuators), a `dof_M0` cache can be added later without changing the algorithm.

**After** (inserted into `compute_actuator_params`, after the acc0 loop):
```rust
    // --- Phase 3: dampratio → damping for position-like actuators ---
    const MJ_MINVAL: f64 = 1e-15;

    for i in 0..self.nu {
        let gainprm = &self.actuator_gainprm[i];
        let biasprm = &mut self.actuator_biasprm[i];

        // Position-actuator fingerprint: gainprm[0] == -biasprm[1]
        // For position actuator: gainprm[0] = kp, biasprm[1] = -kp
        // MuJoCo uses exact != comparison (both values derive from the same kp
        // parse, so bit patterns match exactly for position actuators).
        if gainprm[0] != -biasprm[1] {
            continue;
        }

        // Only positive biasprm[2] is interpreted as dampratio
        // Zero or negative means explicit kv (already correct sign)
        if biasprm[2] <= 0.0 {
            continue;
        }

        // Compute reflected inertia from actuator moment and qM diagonal
        let j_vec = &data.actuator_moment[i];
        let mut mass = 0.0;
        for dof in 0..self.nv {
            let trn = j_vec[dof].abs();
            let trn2 = trn * trn;
            if trn2 > MJ_MINVAL {
                // dof_M0[dof] equivalent: diagonal of mass matrix at qpos0
                let dof_m0 = data.qM[(dof, dof)];
                mass += dof_m0 / trn2;
            }
        }

        // Critical damping formula: kv = dampratio * 2 * sqrt(kp * mass)
        let kp = gainprm[0];
        let dampratio = biasprm[2];
        let damping = dampratio * 2.0 * (kp * mass).sqrt();

        // Store as negative (bias convention: b2 * velocity opposes motion)
        biasprm[2] = -damping;
    }
```

**Dependency on `data`:** The dampratio loop reads `data.actuator_moment[i]`
and `data.qM[(dof, dof)]`. Both are available after the Phase 2 forward pass.
However, `data.actuator_moment` is populated by `mj_transmission()` during
`step1()`, not by `mj_fwd_position()` alone. We need to ensure the moment
vectors are available.

**Resolution:** The current acc0 code constructs moment vectors manually
(building `j_vec` from transmission type). For the dampratio loop, we reuse
the same manually-constructed `j_vec` approach rather than relying on
`data.actuator_moment`. This means the dampratio loop needs to either:
(a) store the `j_vec` from the acc0 loop for reuse, or
(b) reconstruct it.

Option (a) is preferred — store `j_vec` per actuator during the acc0 loop
and reuse in the dampratio loop:

```rust
    // Phase 2: acc0 + store moment vectors for Phase 3
    let mut moment_vecs: Vec<DVector<f64>> = Vec::with_capacity(self.nu);

    for i in 0..self.nu {
        let mut j_vec = DVector::zeros(self.nv);
        // ... (existing moment construction code) ...

        // Solve M * x = J for acc0
        let mut x = j_vec.clone();
        let (rowadr, rownnz, colind) = self.qld_csr();
        mj_solve_sparse(rowadr, rownnz, colind,
            &data.qLD_data, &data.qLD_diag_inv, &mut x);
        self.actuator_acc0[i] = x.norm().max(1e-10);

        moment_vecs.push(j_vec);
    }

    // Phase 3: dampratio
    for i in 0..self.nu {
        // ... (dampratio conversion using moment_vecs[i] instead of data.actuator_moment[i])
        let j_vec = &moment_vecs[i];
        // ...
    }
```

### S3b. Parse `dampratio` MJCF attribute for position actuators

**File:** `sim/L0/mjcf/src/builder/actuator.rs`
**MuJoCo equivalent:** Position actuator attribute parsing in `user_model.cc`
**Design decision:** MuJoCo's position actuator accepts `dampratio` as an
alternative to `kv`. When `dampratio` is specified, it's stored as a positive
`biasprm[2]` value. The `set0()` function then converts it to negative damping
at compile time. Currently our builder always stores `-kv` in `biasprm[2]`
(where `kv` defaults to `0.0`). To support `dampratio`, we store the positive
dampratio value when the user specifies `dampratio` instead of `kv`.

**After** (in `process_actuator`, position arm):
```rust
MjcfActuatorType::Position => {
    let kp = actuator.kp;
    let kv = actuator.kv;
    let dampratio = actuator.dampratio; // new: Option<f64>

    let mut gp = [0.0; 9];
    gp[0] = kp;

    let mut bp = [0.0; 9];
    bp[1] = -kp;

    // dampratio vs explicit kv:
    // If dampratio is specified (positive), store as positive biasprm[2].
    // compute_actuator_params() will convert it to -damping at compile time.
    // If kv is specified, store as -kv (explicit damping).
    // If neither, biasprm[2] = 0.0 (no damping).
    if let Some(dr) = dampratio {
        bp[2] = dr; // positive → dampratio (converted later)
    } else {
        bp[2] = -kv.unwrap_or(0.0); // negative → explicit kv
    }

    // ... rest unchanged
}
```

Also requires adding `dampratio: Option<f64>` to the `MjcfActuator` struct
in `sim/L0/mjcf/src/types.rs` and parsing it from the `<position>` element.

### S4. Simulation-based length-range estimation (DT-59, DT-77)

**File:** `sim/L0/core/src/forward/muscle.rs` (new function)
**MuJoCo equivalent:** `mj_setLengthRange()` + `evalAct()` in
`engine_setconst.c`
**Design decision:** Implement a simplified version of `mj_setLengthRange`
that supports the critical path: unlimited joints and site transmissions.
The full `mjLROpt` configurability is implemented but with MuJoCo defaults.
The `<lengthrange>` XML element parsing is deferred to a future spec (it's
rare in practice).

#### S4a. `LengthRangeOpt` struct

**File:** `sim/L0/core/src/types/model.rs` (or a new `lengthrange.rs`)

```rust
/// Options for simulation-based actuator length-range estimation.
/// Matches MuJoCo's `mjLROpt` struct with identical defaults.
#[derive(Debug, Clone)]
pub struct LengthRangeOpt {
    /// Which actuators to compute for.
    pub mode: LengthRangeMode,
    /// Skip if range already set (lo < hi).
    pub useexisting: bool,
    /// Copy from joint/tendon limits when available.
    pub uselimit: bool,
    /// Target acceleration magnitude for the applied force.
    pub accel: f64,
    /// Force cap (0 = unlimited).
    pub maxforce: f64,
    /// Velocity damping time constant (seconds).
    pub timeconst: f64,
    /// Internal simulation timestep (seconds).
    pub timestep: f64,
    /// Total simulation time (seconds).
    pub inttotal: f64,
    /// Measurement interval — last N seconds used for convergence.
    pub interval: f64,
    /// Convergence tolerance (fraction of total range).
    pub tolrange: f64,
}

impl Default for LengthRangeOpt {
    fn default() -> Self {
        Self {
            mode: LengthRangeMode::Muscle,
            useexisting: true,
            uselimit: true,
            accel: 20.0,
            maxforce: 0.0,
            timeconst: 1.0,
            timestep: 0.01,
            inttotal: 10.0,
            interval: 2.0,
            tolrange: 0.05,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LengthRangeMode {
    /// Disabled entirely.
    None,
    /// Compute only for muscle actuators (default).
    #[default]
    Muscle,
    /// Compute for muscle and user-defined actuators.
    MuscleUser,
    /// Compute for all actuators.
    All,
}
```

#### S4b. `mj_set_length_range()` function

**File:** `sim/L0/core/src/forward/muscle.rs`

```rust
/// Estimate actuator length range via simulation.
///
/// For each actuator that passes mode filtering:
/// 1. If `useexisting` and range already set, skip.
/// 2. If `uselimit` and joint/tendon is limited, copy limits.
/// 3. Otherwise, run two-sided simulation to find min/max length.
///
/// Returns Ok(()) on success, Err with actuator index on convergence failure.
fn mj_set_length_range(
    model: &mut Model,
    opt: &LengthRangeOpt,
) -> Result<(), LengthRangeError> {
    for i in 0..model.nu {
        // Step 1: Mode filtering
        let is_muscle = model.actuator_gaintype[i] == GainType::Muscle
            || model.actuator_biastype[i] == BiasType::Muscle;
        let is_user = model.actuator_gaintype[i] == GainType::User
            || model.actuator_biastype[i] == BiasType::User;

        let skip = match opt.mode {
            LengthRangeMode::None => true,
            LengthRangeMode::Muscle => !is_muscle,
            LengthRangeMode::MuscleUser => !is_muscle && !is_user,
            LengthRangeMode::All => false,
        };
        if skip {
            continue;
        }

        // Step 2: Use existing range
        let (lo, hi) = model.actuator_lengthrange[i];
        if opt.useexisting && lo < hi {
            continue;
        }

        // Step 3: Copy from limits
        if opt.uselimit {
            let gear = model.actuator_gear[i][0];
            let scale = |a: f64, b: f64| -> (f64, f64) {
                let x = gear * a;
                let y = gear * b;
                (x.min(y), x.max(y))
            };

            match model.actuator_trntype[i] {
                ActuatorTransmission::Joint => {
                    let jid = model.actuator_trnid[i][0];
                    if jid < model.njnt && model.jnt_limited[jid] {
                        let (jlo, jhi) = model.jnt_range[jid];
                        model.actuator_lengthrange[i] = scale(jlo, jhi);
                        continue;
                    }
                }
                ActuatorTransmission::Tendon => {
                    let tid = model.actuator_trnid[i][0];
                    if tid < model.ntendon && model.tendon_limited[tid] {
                        let (tlo, thi) = model.tendon_range[tid];
                        model.actuator_lengthrange[i] = scale(tlo, thi);
                        continue;
                    }
                }
                _ => {} // Site/Body: no limits to copy
            }
        }

        // Step 4: Simulation-based estimation
        let range = eval_length_range(model, i, opt)?;
        model.actuator_lengthrange[i] = range;
    }
    Ok(())
}
```

#### S4c. `eval_length_range()` — inner simulation

```rust
/// Run two-sided simulation to estimate actuator length range.
///
/// Side 0: apply negative force → find minimum length.
/// Side 1: apply positive force → find maximum length.
///
/// Runs the full step1/step2 pipeline with gravity, contacts, and passive
/// forces active — matching MuJoCo's evalAct behavior exactly. MuJoCo does
/// NOT disable gravity or contacts during length-range estimation. The large
/// applied force (accel=20 ≈ 2× gravity) overwhelms other forces by design.
///
/// The model is cloned to set the LR-specific timestep without mutating the
/// original. Gravity and contacts remain at their model-configured values.
fn eval_length_range(
    model: &Model,
    actuator_idx: usize,
    opt: &LengthRangeOpt,
) -> Result<(f64, f64), LengthRangeError> {
    let nv = model.nv;

    // Clone model to set LR-specific timestep.
    // IMPORTANT: Do NOT zero gravity or disable contacts — MuJoCo keeps them
    // active. The applied force overwhelms gravity by design (accel >> g).
    let mut lr_model = model.clone();
    lr_model.timestep = opt.timestep;

    let mut lengthrange = [0.0f64; 2];
    let mut lmin_sides = [f64::MAX; 2];
    let mut lmax_sides = [f64::MIN; 2];

    for side in 0..2 {
        let mut data = lr_model.make_data(); // init at qpos0
        let sign = if side == 0 { -1.0 } else { 1.0 };

        let mut updated = false;
        let total_steps = (opt.inttotal / opt.timestep).ceil() as usize;
        let measure_start = opt.inttotal - opt.interval;

        for step_idx in 0..total_steps {
            let time = step_idx as f64 * opt.timestep;

            // 1. Velocity damping: qvel *= exp(-dt / max(0.01, timeconst))
            let decay = (-opt.timestep / opt.timeconst.max(0.01)).exp();
            for dof in 0..nv {
                data.qvel[dof] *= decay;
            }

            // 2. Full forward pass (FK, collision, CRBA, gravity, passive, actuation)
            data.step1(&lr_model).expect("step1 failed in LR estimation");

            // 3. Build moment vector and compute applied force
            let j_vec = build_actuator_moment(&lr_model, &data, actuator_idx);

            let mut x = j_vec.clone();
            let (rowadr, rownnz, colind) = lr_model.qld_csr();
            mj_solve_sparse(
                rowadr, rownnz, colind,
                &data.qLD_data, &data.qLD_diag_inv,
                &mut x,
            );
            let nrm = x.norm().max(1e-15);

            // qfrc_applied = sign * accel * moment / ||M^{-1} moment||
            for dof in 0..nv {
                data.qfrc_applied[dof] = sign * opt.accel * j_vec[dof] / nrm;
            }

            // 4. Force capping
            if opt.maxforce > 0.0 {
                let fnrm = data.qfrc_applied.norm();
                if fnrm > opt.maxforce {
                    let scale = opt.maxforce / fnrm;
                    for dof in 0..nv {
                        data.qfrc_applied[dof] *= scale;
                    }
                }
            }

            // 5. Acceleration + integration (step2)
            data.step2(&lr_model).expect("step2 failed in LR estimation");

            // 6. Read actuator length (populated by step1's transmission stage)
            let len = data.actuator_length[actuator_idx];

            // 7. Track min/max during measurement interval
            if time >= measure_start {
                if len < lmin_sides[side] || !updated {
                    lmin_sides[side] = len;
                }
                if len > lmax_sides[side] || !updated {
                    lmax_sides[side] = len;
                }
                updated = true;
            }
        }

        lengthrange[side] = if side == 0 { lmin_sides[side] } else { lmax_sides[side] };
    }

    // Step 5: Convergence check
    let dif = lengthrange[1] - lengthrange[0];
    if dif <= 0.0 {
        return Err(LengthRangeError::InvalidRange { actuator: actuator_idx });
    }
    // Per-side convergence: oscillation during measurement window must be
    // within tolrange fraction of total range
    for side in 0..2 {
        let oscillation = lmax_sides[side] - lmin_sides[side];
        if oscillation > opt.tolrange * dif {
            return Err(LengthRangeError::ConvergenceFailed { actuator: actuator_idx });
        }
    }

    Ok((lengthrange[0], lengthrange[1]))
}

/// Build the actuator moment vector (transmission Jacobian) for a single actuator.
/// Extracted as a helper to share between acc0 and lengthrange computation.
fn build_actuator_moment(
    model: &Model,
    data: &Data,
    actuator_idx: usize,
) -> DVector<f64> {
    let gear = model.actuator_gear[actuator_idx][0];
    let mut j_vec = DVector::zeros(model.nv);

    match model.actuator_trntype[actuator_idx] {
        ActuatorTransmission::Joint => {
            let jid = model.actuator_trnid[actuator_idx][0];
            if jid < model.njnt {
                let dof_adr = model.jnt_dof_adr[jid];
                j_vec[dof_adr] = gear;
            }
        }
        ActuatorTransmission::Tendon => {
            let tid = model.actuator_trnid[actuator_idx][0];
            if tid < model.ntendon {
                match model.tendon_type[tid] {
                    TendonType::Fixed => {
                        let adr = model.tendon_adr[tid];
                        let num = model.tendon_num[tid];
                        for w in adr..(adr + num) {
                            let dof_adr = model.wrap_objid[w];
                            let coef = model.wrap_prm[w];
                            if dof_adr < model.nv {
                                j_vec[dof_adr] = gear * coef;
                            }
                        }
                    }
                    TendonType::Spatial => {
                        for dof in 0..model.nv {
                            j_vec[dof] = gear * data.ten_J[tid][dof];
                        }
                    }
                }
            }
        }
        ActuatorTransmission::Site => {
            let sid = model.actuator_trnid[actuator_idx][0];
            let refid = model.actuator_trnid[actuator_idx][1];
            let (jac_t, jac_r) = mj_jac_site(model, data, sid);
            let full_gear = model.actuator_gear[actuator_idx];

            if refid == usize::MAX {
                let wrench_t = data.site_xmat[sid]
                    * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                let wrench_r = data.site_xmat[sid]
                    * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                for dof in 0..model.nv {
                    j_vec[dof] = jac_t.column(dof).dot(&wrench_t)
                        + jac_r.column(dof).dot(&wrench_r);
                }
            } else {
                // Difference Jacobian with common-ancestor zeroing
                // (existing code from compute_muscle_params — extract into helper)
                let (ref_jac_t, ref_jac_r) = mj_jac_site(model, data, refid);
                let mut diff_t = &jac_t - &ref_jac_t;
                let mut diff_r = &jac_r - &ref_jac_r;

                // Zero common-ancestor DOF columns
                let b0 = model.site_body[sid];
                let b1 = model.site_body[refid];
                let bca = find_common_ancestor(model, b0, b1);
                zero_ancestor_dofs(model, &mut diff_t, &mut diff_r, bca);

                let wrench_t = data.site_xmat[refid]
                    * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                let wrench_r = data.site_xmat[refid]
                    * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                for dof in 0..model.nv {
                    j_vec[dof] = diff_t.column(dof).dot(&wrench_t)
                        + diff_r.column(dof).dot(&wrench_r);
                }
            }
        }
        ActuatorTransmission::Body => {
            // No contacts at qpos0 → zero moment
        }
    }
    j_vec
}
```

#### S4d. Actuator length in the simulation loop

The actuator length is read from `data.actuator_length[actuator_idx]`, which
is populated by `step1()`'s transmission stage (`mj_actuator_length` in the
velocity stage). This handles all transmission types correctly (joint, tendon,
site, body) including multi-DOF joints. No separate `compute_actuator_length`
helper is needed.

#### S4e. `LengthRangeError` type

```rust
#[derive(Debug)]
pub enum LengthRangeError {
    /// Length range has zero or negative extent (max <= min).
    InvalidRange { actuator: usize },
    /// Simulation did not converge within tolerance.
    ConvergenceFailed { actuator: usize },
}
```

#### S4f. Integration into `compute_actuator_params`

The simulation-based lengthrange is called as Phase 1b, after the limit-based
Phase 1 and before the Phase 2 forward pass:

```rust
pub fn compute_actuator_params(&mut self) {
    if self.nu == 0 {
        return;
    }

    // Phase 1a: Copy lengthrange from limits (existing code)
    // ... (as shown in S1)

    // Phase 1b: Simulation-based lengthrange for actuators that need it
    let lr_opt = LengthRangeOpt::default();
    if let Err(e) = mj_set_length_range(self, &lr_opt) {
        eprintln!("Warning: lengthrange estimation failed: {:?}", e);
        // Non-fatal: actuators with failed estimation keep (0, 0)
    }

    // Phase 2: Forward pass at qpos0 + acc0 for all actuators
    // ... (as shown in S1)

    // Phase 3: dampratio conversion
    // ... (as shown in S3)

    // Phase 4: Resolve muscle F0
    // ... (as shown in S1)
}
```

**Important note on Phase 1 restructuring:** The limit-based lengthrange
(Phase 1a) is now handled inside `mj_set_length_range` via the `uselimit`
path (Step 3). This means Phase 1a in `compute_actuator_params` can be
removed — all lengthrange computation is centralized in `mj_set_length_range`.
However, `mj_set_length_range` applies mode filtering (only muscle by default),
so non-muscle actuators with limited joints would not get their lengthrange
set. Since the current code sets lengthrange for ALL actuators from limits
(not just muscles), we should either:
(a) Set `mode = LengthRangeMode::All` for the Phase 1a equivalent, or
(b) Keep Phase 1a for the limit-based path (all actuators) and only use
    `mj_set_length_range` for the simulation fallback.

**Resolution:** Option (b) is correct. MuJoCo's `mj_setLengthRange` has mode
filtering because it's a public API callable with different options. In our
internal `compute_actuator_params`, the limit-based path runs unconditionally
for all actuators (matching MuJoCo's compile-time behavior), and the simulation
path runs per the `LengthRangeOpt` mode (defaulting to muscle-only, matching
MuJoCo's default).

---

## Acceptance Criteria

> **MuJoCo verification requirement:** At least AC1 (acc0), AC4 (dampratio),
> and AC8 (lengthrange) must have expected values verified by running the same
> model in MuJoCo and reading its output. During implementation, create
> equivalent MJCF models, load them in MuJoCo (Python bindings), and record the
> exact values. Update the AC expected values from "analytically derived" to
> "MuJoCo 3.2.6 verified" with the actual MuJoCo output. This is a Phase 7
> (implementation) deliverable — the spec states the analytical expectations
> now, and implementation must confirm them against MuJoCo before the AC is
> considered passing.

### AC1: acc0 for motor actuator on single hinge *(runtime test, analytically derived — verify against MuJoCo)*
**Given:** Model with one body (mass=1.0, inertia=(0.1, 0.1, 0.1)), one hinge
joint about Y axis, one motor actuator (DynType::None, gear=2.0).
**After:** `compute_actuator_params()`
**Assert:** `actuator_acc0[0]` = 20.0 ± 1e-6 (= |gear|/I_yy = 2.0/0.1)
**Field:** `Model.actuator_acc0`

### AC2: acc0 for position actuator *(runtime test, analytically derived)*
**Given:** Same model as AC1 but with a position actuator (kp=100, kv=10,
gear=1.0).
**After:** `compute_actuator_params()`
**Assert:** `actuator_acc0[0]` = 10.0 ± 1e-6 (= |gear|/I_yy = 1.0/0.1)
**Field:** `Model.actuator_acc0`

### AC3: acc0 unchanged for muscle actuator *(runtime test, regression)*
**Given:** Existing `build_muscle_model_joint(2.0)` test model.
**After:** `compute_actuator_params()`
**Assert:** `actuator_acc0[0]` = 20.0 ± 0.2 (same as before rename).
**Field:** `Model.actuator_acc0`

### AC4: dampratio conversion for position actuator *(runtime test, analytically derived — verify against MuJoCo)*
**Given:** Model with one hinge (I_yy=0.1), one position actuator with
kp=100.0, dampratio=1.0, gear=1.0. Builder stores `biasprm[2] = 1.0`
(positive = dampratio).
**After:** `compute_actuator_params()`
**Assert:** `actuator_biasprm[0][2]` = `-2.0 * sqrt(100.0 * 0.1)` =
`-2.0 * sqrt(10.0)` ≈ -6.3246 ± 1e-4.
Reflected inertia: `dof_M0[0] / gear^2 = I_yy / 1.0 = 0.1`.
`kv = 1.0 * 2 * sqrt(100 * 0.1) = 2 * sqrt(10) ≈ 6.3246`.
Stored as `-6.3246`.
**Field:** `Model.actuator_biasprm`

### AC5: dampratio skipped for motor actuator *(runtime test, negative case)*
**Given:** Motor actuator (gaintype=Fixed, biastype=None, gainprm[0]=1.0,
biasprm[1]=0.0). `gainprm[0] != -biasprm[1]` → not position-like.
**After:** `compute_actuator_params()`
**Assert:** `actuator_biasprm[0][2]` = 0.0 (unchanged).
**Field:** `Model.actuator_biasprm`

### AC6: dampratio skipped for explicit kv *(runtime test, negative case)*
**Given:** Position actuator with kp=100.0, explicit kv=10.0 (stored as
`biasprm[2] = -10.0`).
**After:** `compute_actuator_params()`
**Assert:** `actuator_biasprm[0][2]` = -10.0 (unchanged — negative means
explicit, not dampratio).
**Field:** `Model.actuator_biasprm`

### AC7: lengthrange from limits unchanged *(runtime test, regression)*
**Given:** Muscle actuator on limited hinge joint (range=[-1.0, 1.0], gear=2.0).
**After:** `compute_actuator_params()`
**Assert:** `actuator_lengthrange[0]` = (-2.0, 2.0) (= gear * joint range).
**Field:** `Model.actuator_lengthrange`

### AC8: lengthrange via simulation for unlimited slide *(runtime test, analytically bounded — verify against MuJoCo)*
**Given:** Muscle actuator on unlimited slide joint (jnt_limited=false),
gear=1.0, body mass=1.0. `LengthRangeOpt::default()` (accel=20, inttotal=10s,
timestep=0.01s). Gravity active (MuJoCo does NOT disable it during LR).
**After:** `compute_actuator_params()`
**Assert:** `actuator_lengthrange[0].0 < -0.1` and
`actuator_lengthrange[0].1 > 0.1` (simulation finds a nonzero range).
Approximate bound: with constant acceleration `a = accel * gear / mass = 20`,
after `inttotal = 10s` with damping, displacement ≈ `a * timeconst^2` ≈ 20m.
The exact values depend on the damped simulation dynamics, but the range
must be symmetric about 0 (symmetric model) and span at least 1.0 total.
**Field:** `Model.actuator_lengthrange`

### AC9: lengthrange via simulation for site transmission *(runtime test, analytically bounded)*
**Given:** Muscle actuator with site transmission, two sites on adjacent bodies
separated by a hinge joint (range unlimited). Sites at distance 0.5 from
joint axis.
**After:** `compute_actuator_params()`
**Assert:** `actuator_lengthrange[0].0 < actuator_lengthrange[0].1`
(simulation finds a valid range). For a hinge with arm=0.5, the site
separation varies as `2 * 0.5 * sin(theta/2)` — range should be
approximately `[-1.0, 1.0]` (full rotation) or a subset depending on
simulation convergence.
**Field:** `Model.actuator_lengthrange`

### AC10: function renamed *(code review)*
**Verify:** `compute_muscle_params` does not exist anywhere in the codebase.
`compute_actuator_params` is called from the same call sites.

### AC11: existing tests pass *(runtime test, regression)*
**After:** Full domain test suite.
**Assert:** All 2,148+ domain tests pass. Zero failures.

### AC12: `dampratio` MJCF attribute parsed *(runtime test)*
**Given:** MJCF XML with `<position joint="j" kp="100" dampratio="1.0"/>`.
**After:** Model build.
**Assert:** `actuator_biasprm[0][2] > 0.0` before `compute_actuator_params()`
(stored as positive dampratio). After: negative (converted to damping).
**Field:** `Model.actuator_biasprm`

### AC13: lengthrange mode filtering *(runtime test, negative case)*
**Given:** Motor actuator (non-muscle) on unlimited slide joint.
`LengthRangeOpt::default()` (mode=Muscle).
**After:** `compute_actuator_params()`
**Assert:** `actuator_lengthrange[0]` = (0.0, 0.0) — unchanged, because
mode=Muscle skips non-muscle actuators.
**Field:** `Model.actuator_lengthrange`

### AC14: multi-body acc0 + dampratio *(runtime test, MuJoCo-verified)*
**Given:** 3-body chain (base → link1 → link2), two hinge joints, link1
mass=2.0 I_yy=0.2, link2 mass=1.0 I_yy=0.1. Actuator (a): position on joint
1, kp=200, dampratio=1.0, gear=1.0. Actuator (b): motor on joint 2, gear=3.0.
**After:** `compute_actuator_params()`
**Assert:** `actuator_acc0[0]`, `actuator_acc0[1]`, and `actuator_biasprm[0][2]`
match MuJoCo's output for the same model within ±1e-6. (Exact expected values
to be filled in during implementation after running MuJoCo.)
**Field:** `Model.actuator_acc0`, `Model.actuator_biasprm`
**Purpose:** Non-trivial kinematic structure with mass-matrix coupling.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (acc0 motor) | T1 | Direct |
| AC2 (acc0 position) | T2 | Direct |
| AC3 (acc0 muscle regression) | T3 | Regression |
| AC4 (dampratio conversion) | T4 | Direct |
| AC5 (dampratio skip motor) | T5 | Negative |
| AC6 (dampratio skip explicit kv) | T6 | Negative |
| AC7 (lengthrange limits) | T7 | Regression |
| AC8 (lengthrange unlimited slide) | T8 | Direct |
| AC9 (lengthrange site) | T9 | Direct |
| AC10 (rename) | — | Code review (manual) |
| AC11 (existing tests) | T10 | Regression |
| AC12 (dampratio parse) | T11 | Direct |
| AC13 (LR mode filtering) | T12 | Negative |
| AC14 (multi-body conformance) | T13 | MuJoCo conformance |

---

## Test Plan

### T1: acc0 for motor actuator → AC1
Single hinge, motor actuator, gear=2.0, body inertia I_yy=0.1.
Expected: `acc0 = 20.0 ± 1e-6`. Analytically derived: `||M^{-1}J|| = |gear|/I_yy`.

### T2: acc0 for position actuator → AC2
Single hinge, position actuator (kp=100, kv=10), gear=1.0, I_yy=0.1.
Expected: `acc0 = 10.0 ± 1e-6`. Position actuator uses same acc0 formula as motor.

### T3: acc0 muscle regression → AC3
Use existing `build_muscle_model_joint(2.0)`. Verify acc0 unchanged after rename.
Expected: `acc0 = 20.0 ± 0.2` (matches existing `test_acc0_single_hinge`).

### T4: dampratio conversion → AC4
Single hinge, position actuator, kp=100.0, dampratio=1.0, gear=1.0, I_yy=0.1.
Expected: `biasprm[2] = -2*sqrt(100*0.1) ≈ -6.3246 ± 1e-4`.
Analytically derived: `reflected_mass = I_yy / gear^2 = 0.1`,
`kv = dampratio * 2 * sqrt(kp * mass) = 1.0 * 2 * sqrt(10) ≈ 6.3246`.

### T5: dampratio skip for motor → AC5
Motor actuator: `gainprm[0]=1.0, biasprm[1]=0.0`. Fingerprint fails
(`1.0 != -0.0`). biasprm[2] should remain 0.0.

### T6: dampratio skip for explicit kv → AC6
Position actuator with explicit `kv=10.0` → `biasprm[2] = -10.0`.
Negative → already explicit, skip. biasprm[2] should remain -10.0.

### T7: lengthrange limits regression → AC7
Reuse existing muscle model test. Limited hinge range=[-1,1], gear=2.0.
Expected: lengthrange = (-2.0, 2.0). Unchanged from current behavior.

### T8: lengthrange unlimited slide simulation → AC8
Muscle actuator on unlimited slide joint. Run `compute_actuator_params`.
Assert lengthrange[0] < 0 and lengthrange[1] > 0 (nonzero range found).
Note: exact values depend on simulation; test asserts structure not exact values.

### T9: lengthrange site transmission simulation → AC9
Muscle actuator with site transmission on a hinge joint.
Assert lengthrange forms valid range (lo < hi).

### T10: existing tests pass → AC11
Run `cargo test -p sim-core -p sim-mjcf -p sim-muscle -p sim-sensor`.
All tests pass.

### T11: dampratio MJCF round-trip → AC12
Build model from MJCF with `<position dampratio="1.0" kp="100"/>`.
Assert biasprm[2] is positive before `compute_actuator_params`, negative after.

### T12: lengthrange mode filtering → AC13
Motor actuator (non-muscle) on unlimited slide joint. Default LR mode=Muscle.
Assert lengthrange remains (0.0, 0.0) — simulation not run for non-muscle.

### T13: multi-body acc0 + dampratio conformance → AC14 *(MuJoCo-verified)*
**Model:** 3-body chain (base → link1 → link2), two hinge joints (Y-axis),
link1 mass=2.0 I_yy=0.2, link2 mass=1.0 I_yy=0.1. Two actuators:
(a) position actuator on joint 1 with kp=200, dampratio=1.0, gear=1.0.
(b) motor actuator on joint 2 with gear=3.0.
**Verify against MuJoCo:** Load equivalent MJCF in MuJoCo Python, read
`m.actuator_acc0[0]`, `m.actuator_acc0[1]`, and `m.actuator_biasprm[0][2]`
after `mj_setConst`. Assert our values match within ±1e-6.
**Purpose:** Tests that acc0 and dampratio work correctly with non-diagonal
mass matrix coupling between joints. This is the minimum complexity model
that exercises the multi-DOF reflected inertia accumulation.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `nv == 0` (no DOFs) | acc0 loop should produce 0 for all actuators | Supplementary S1 | — |
| Zero transmission (body at qpos0) | Moment is all zeros → acc0 = 0, dampratio mass = 0 | T5 (motor with biastype::None) | AC5 |
| `gainprm[0] != -biasprm[1]` (not position-like) | Dampratio must NOT fire for motor/velocity/damper | T5 | AC5 |
| `biasprm[2] <= 0` (explicit kv) | Dampratio must NOT convert already-negative kv | T6 | AC6 |
| `biasprm[2] == 0` (zero damping) | Not positive → skip (no conversion to −0) | T6 variant | AC6 |
| Multi-DOF transmission (multi-body chain) | Reflected inertia and acc0 with mass-matrix coupling | T13 | AC14 |
| Unlimited hinge joint | Falls through limit-based path to simulation | T8 | AC8 |
| Site transmission | No limit path at all → always simulation | T9 | AC9 |
| Convergence failure | Simulation doesn't converge → warning, keep (0,0) | Supplementary S2 | — |
| LR mode=Muscle filtering | Non-muscle actuators must NOT get simulation-based LR | T12 | AC13 |
| Multi-DOF tendon + dampratio | Reflected inertia sums across DOFs (exotic but correct) | Future (not in v1.0 test set) | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| S1: acc0 with nv=0 | Zero-DOF model produces acc0=0 | Guards against panic on empty qM |
| S2: lengthrange convergence failure | Body transmission → simulation produces (0,0) range | Verifies graceful error handling |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| acc0 for non-muscle actuators | acc0 = 0.0 for all non-muscle actuators | acc0 computed via `\|\|M^{-1}J\|\|` for ALL actuators | Toward MuJoCo | Any code reading `actuator_acc0` for non-muscle actuators (currently: none) | None — transparent change |
| `compute_muscle_params` → `compute_actuator_params` rename | Function named for muscles only | Named for all actuators | N/A (naming) | All call sites | Search-and-replace |
| dampratio conversion | Positive `biasprm[2]` remains positive (wrong) | Positive `biasprm[2]` converted to negative damping | Toward MuJoCo | Models using `dampratio` attribute (currently: none parsed) | None — new feature |
| lengthrange for unlimited joints | lengthrange = (0, 0) | Estimated via simulation | Toward MuJoCo | Muscle actuators on unlimited joints (currently: warned) | None — fills gap |
| lengthrange for site transmissions | lengthrange = (0, 0) | Estimated via simulation | Toward MuJoCo | Muscle actuators with site transmission | None — fills gap |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/forward/muscle.rs` | Rename function, remove muscle-only guards, add dampratio loop, add lengthrange simulation, extract `build_actuator_moment` helper | ~+200 / -30 |
| `sim/L0/core/src/types/model.rs` | Add `LengthRangeOpt`, `LengthRangeMode`, `LengthRangeError` types | ~+60 |
| `sim/L0/core/src/types/model_init.rs` | No change (acc0 already initialized as empty vec) | 0 |
| `sim/L0/mjcf/src/builder/actuator.rs` | Parse `dampratio` attribute, store as positive biasprm[2] | ~+15 |
| `sim/L0/mjcf/src/types.rs` | Add `dampratio: Option<f64>` to MjcfActuator | ~+3 |
| `sim/L0/mjcf/src/builder/mod.rs` | Rename `compute_muscle_params` → `compute_actuator_params` call | ~1 |
| Test files (new) | New tests T1–T11, S1–S2 | ~+300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_fl_curve_shape` | `muscle.rs` | Pass (unchanged) | Curve functions not modified |
| `test_fv_curve_shape` | `muscle.rs` | Pass (unchanged) | Curve functions not modified |
| `test_fp_curve_shape` | `muscle.rs` | Pass (unchanged) | Curve functions not modified |
| `test_activation_dynamics_ramp_up` | `muscle.rs` | Pass (unchanged) | Activation dynamics not modified |
| `test_activation_dynamics_ramp_down` | `muscle.rs` | Pass (unchanged) | Activation dynamics not modified |
| `test_activation_asymmetry` | `muscle.rs` | Pass (unchanged) | Activation dynamics not modified |
| `test_muscle_force_at_optimal_length` | `muscle.rs` | Pass (unchanged) | Calls renamed function via model builder |
| `test_muscle_act_num_is_one` | `muscle.rs` | Pass (unchanged) | act_num unchanged |
| `test_motor_actuator_unchanged` | `muscle.rs` | Pass (unchanged) | Motor force computation unchanged; acc0 now computed but not used by motor gain |
| `test_control_clamping` | `muscle.rs` | Pass (unchanged) | Clamping logic unchanged |
| `test_force_clamping` | `muscle.rs` | Pass (unchanged) | Force clamping unchanged |
| `test_muscle_params_transferred` | `muscle.rs` | Pass (unchanged) | dynprm/gainprm values unchanged |
| `test_acc0_single_hinge` | `muscle.rs` | Pass (unchanged) | acc0 for muscles unchanged |
| `test_f0_auto_computation` | `muscle.rs` | Pass (unchanged) | F0 auto-compute unchanged |
| `test_f0_explicit_not_overridden` | `muscle.rs` | **Call site rename** | Must update `compute_muscle_params()` → `compute_actuator_params()` |
| `test_rk4_activation_single_step` | `muscle.rs` | Pass (unchanged) | RK4 integration unchanged |
| `test_sigmoid_boundaries` | `muscle.rs` | Pass (unchanged) | Sigmoid not modified |
| Phase 4 regression suite (39 tests) | conformance/ | Pass (unchanged) | Phase 5 does not modify Phase 4 code paths |

---

## Execution Order

1. **S2 (rename)** first — mechanical rename of `compute_muscle_params` →
   `compute_actuator_params` across all call sites and tests. Verify all
   existing tests pass after rename. → run domain tests
2. **S1 (acc0 extension)** — remove muscle-only guards. Verify existing
   muscle acc0 tests still pass, add new motor/position acc0 tests. → run
   domain tests
3. **S3 + S3b (dampratio)** — add dampratio conversion loop + MJCF parsing.
   Requires acc0 to be computed for all actuators (S1). → run domain tests
4. **S4 (lengthrange simulation)** — add simulation infrastructure. Independent
   of S3 but benefits from the `build_actuator_moment` extraction in S1.
   → run domain tests

---

## Performance Characterization

The `mj_set_length_range` simulation (S4) is the only performance-relevant
addition. The other features (acc0, dampratio) are O(nu × nv) sparse operations
that are negligible at compile time.

**S4 simulation cost:**
- Per-actuator: 1 `model.clone()` + 2 sides × `inttotal/timestep` integration
  steps = 2 × 1,000 = 2,000 `step1`/`step2` calls (with default opts).
- Total: O(nu_lr × 2,000) where `nu_lr` = number of actuators that fall through
  to the simulation path (i.e., muscle actuators with unlimited joints or site
  transmissions). With default mode=Muscle, non-muscle actuators are skipped.
- For a humanoid with ~20 muscle actuators, all with limited joints: 0 simulation
  actuators (all handled by `uselimit`). Cost: zero.
- For a humanoid with 20 muscle actuators on unlimited joints: 20 × 2,000 =
  40,000 integration steps at compile time. Each step involves FK + CRBA +
  collision + sparse solve. Estimated ~1–5 seconds total on modern hardware.
- **Model clone:** One `model.clone()` per `eval_length_range` call (per
  actuator that reaches Step 4). The clone only changes `timestep`; a future
  optimization could reuse a single clone across actuators by resetting
  `data` instead of creating fresh data. Not required for v1.0.

**Verdict:** Acceptable for models with <100 simulation-path actuators. The
common case (limited joints) incurs zero simulation cost. For large models
with many unlimited muscle actuators, the cost is bounded by ~5 seconds
compile time and can be optimized later via clone reuse or parallelism.

---

## Out of Scope

- **`<lengthrange>` XML element parsing** — The `LengthRangeOpt` struct is
  defined with MuJoCo defaults, but parsing from `<compiler><lengthrange ...>`
  is deferred. Conformance impact: minimal — defaults match MuJoCo.
- **`dof_M0` as a separate model field** — We use `qM[(i,i)]` diagonal instead.
  Numerically equivalent. If performance requires it, can add later.
- **Spec B transmission types (SliderCrank, JointInParent)** — If Spec B lands
  first, S1's moment construction and S4's simulation need arms for the new
  variants. Handled by Spec B's own implementation.
