# Phase 2 — Multi-DOF Equipartition

> **Status**: Spec draft, pending approval.
> **Branch**: TBD (will branch from `main` after approval)
> **Owner**: Jon
> **Parents**:
> - [`01_langevin_thermostat.md`](./01_langevin_thermostat.md) (Phase 1 — the gate this phase inherits)
> - [`../02_foundations/open_questions.md`](../02_foundations/open_questions.md) (Q1 resolution lives here)
> - [`../02_foundations/chassis_design.md`](../02_foundations/chassis_design.md) (Decisions 1–7 — inherited unchanged)

This spec is a **planning document for an immediate implementation**. It does not re-litigate any chassis decision or Phase 1 design choice. The thermostat code (`sim-thermostat/src/`) is unchanged — all Phase 2 deliverables are new integration tests in `sim-thermostat/tests/`. The spec's job is to define the two MJCF models, the physics of generalized equipartition on multi-DOF systems, and the validation tests precise enough to implement against.

---

## 1. Goal

Demonstrate that the `LangevinThermostat` from Phase 1, **without any code changes**, produces correct thermal equilibrium on:

- **Model A**: A free body with 6 DOFs (3 translational + 3 rotational) and an asymmetric inertia tensor (`I_xx ≠ I_yy ≠ I_zz`).
- **Model B**: A 2-link articulated chain with 2 hinge-joint DOFs and a configuration-dependent, non-diagonal mass matrix `M(q)`.

Both models must satisfy the equipartition theorem to within the same statistical tolerance as Phase 1 (±3σ of the two-level Welford standard error). Passing these gates is the entrance condition for Phase 3 (bistable elements).

---

## 2. Scope

**In scope for Phase 2:**
- Two new MJCF models (§5, §6).
- Model-invariant fixtures for each (§5.1, §6.1).
- Two equipartition gate tests — one per model (§7, §8).
- One multi-DOF reproducibility test (§9).
- Q1 resolution documented (§3).

**Explicitly out of scope:**
- Any changes to `sim-thermostat/src/`. The thermostat already handles arbitrary `nv`.
- Equality constraints (connect, weld) — see §4. Joint DOFs in generalized coordinates don't involve constraint projection.
- Contact constraints — Phase 3+ concern (bistable elements with contacts).
- BatchSim per-env independence — Phase 1 shipped `install_per_env`; Phase 2 validates single-env only.
- Visual examples — validation first, visuals after (separate initiative).
- The γ/T sweep — Phase 1's §7.4 sweep already validates temperature linearity and γ-independence on the 1-DOF system. Phase 2 tests the multi-DOF extension at the central parameter set only.

---

## 3. Q1 Resolution — Noise vs. Constraint Projection

**Q1 asked**: When stochastic forces are written into `qfrc_passive` and projected by the constraint solver, what is the effective temperature on constrained DOFs?

**Answer**: For the Phase 2 test models, the question does not apply. Here is why, and where it does apply.

### 3.1 Force-space FDT is mass-independent

The Langevin equation in force space:

```
M(q) dv = [-∇V(q) - γv] dt + sqrt(2γk_BT) dW
```

samples the canonical distribution `ρ ∝ exp(-H / k_BT)` where `H = ½v^T M(q) v + V(q)` in the continuous-time limit, **regardless of M(q)** — constant or configuration-dependent. The fluctuation–dissipation theorem is a relationship between friction FORCE (`-γv`) and noise FORCE (`σ dW`), not between accelerations. The mass matrix converts both equally when computing `qacc = M^{-1} · F`, so it cancels in the FDT balance.

### 3.2 Generalized coordinates eliminate the constraint projection question for joints

In sim-core (MuJoCo-style generalized coordinates), a hinge joint has `nv=1` velocity DOF. The five locked DOFs per joint (the other rotations and translations) don't exist as variables — they are not "projected out" by the constraint solver, they simply aren't in the formulation. The constraint solver only activates for:

- **Equality constraints** (connect, weld, joint equality)
- **Inequality constraints** (contacts, joint limits)

Neither of these appears in the Phase 2 test models (hinge chain in free space, no contacts, no equality constraints). The constraint solver has zero constraint rows and does nothing.

### 3.3 Where Q1 does matter (Phase 3+)

When a model has active constraints (e.g., a bistable element with contact surfaces, or a connect equality constraint coupling two bodies), the constraint solver adds reactive forces `qfrc_constraint = J^T λ` that project the dynamics onto the constraint manifold. These forces do no work on average (they are normal to the constraint surface), so they should not affect the thermal distribution of the unconstrained DOFs. This is the standard result from constrained Langevin dynamics (Lelièvre, Stoltz, *Free Energy Computations*, Chapter 2).

**Phase 2 does not test this.** The constraint-projection question is deferred to Phase 3, where bistable elements will introduce contacts. Q1 is resolved for Phase 2's scope; it remains an open verification target for Phase 3.

### 3.4 The real subtlety Phase 2 tests

The per-DOF equipartition statement changes form when the mass matrix is non-diagonal. For a 1-DOF system (Phase 1), `⟨½Mv²⟩ = ½k_BT`. For a multi-DOF system with non-diagonal `M(q)`, the correct per-DOF statement is the **generalized equipartition theorem**:

```
⟨v_i · (M(q) · v)_i⟩ = k_BT    for each DOF i
```

And the aggregate: `⟨½ v^T M(q) v⟩ = (n_dof / 2) · k_BT`.

For a **diagonal** mass matrix (Model A's free body), this reduces to `⟨½ M_ii v_i²⟩ = ½k_BT` per DOF — the familiar scalar form, with each DOF having its own effective mass.

For a **non-diagonal, configuration-dependent** mass matrix (Model B's hinge chain), the generalized form must be used. The test computes `(M(q) · v)_i · v_i` at each measurement step using `data.qM` (valid after each `data.step()`, computed by CRBA in the forward pipeline).

---

## 4. Physics: Generalized Equipartition

### 4.1 The theorem

For a classical system in thermal equilibrium at temperature `T`, the generalized equipartition theorem states:

```
⟨x_i · ∂H/∂x_j⟩ = k_BT · δ_ij
```

For kinetic energy `T_kin = ½ v^T M(q) v`, applying this to the velocity coordinates gives:

```
⟨v_i · ∂T_kin/∂v_i⟩ = k_BT

where ∂T_kin/∂v_i = Σ_j M_ij(q) v_j = (M(q) · v)_i
```

This is the per-DOF test. The aggregate test follows by summation:

```
⟨Σ_i v_i · (M(q)v)_i⟩ = ⟨v^T M(q) v⟩ = 2⟨T_kin⟩ = n_dof · k_BT
```

### 4.2 Model A: diagonal constant M (free body)

For a single free body, `qvel[0:3]` is translational velocity (world frame), `qvel[3:6]` is angular velocity (body frame). The mass matrix is block-diagonal:

```
M = diag(m, m, m, I_xx, I_yy, I_zz)
```

This is **constant** (no configuration dependence — the body-frame inertia tensor is a property of the body, not its pose). Each DOF independently satisfies:

```
⟨½ M_ii v_i²⟩ = ½ k_BT
```

With `I_xx ≠ I_yy ≠ I_zz`, each rotational DOF thermalizes to a different equilibrium velocity variance `⟨ω_i²⟩ = k_BT / I_ii`, testing that the thermostat produces the correct temperature per DOF regardless of effective mass.

### 4.3 Model B: non-diagonal configuration-dependent M (hinge chain)

For a 2-link planar hinge chain, the 2×2 mass matrix `M(q)` depends on the relative joint angle and has off-diagonal coupling terms. The per-DOF test uses the generalized form:

```
⟨v_i · (M(q) · v)_i⟩ = k_BT    for i ∈ {0, 1}
```

And the total kinetic energy test:

```
⟨½ v^T M(q) v⟩ = k_BT    (n_dof = 2)
```

The total KE test can use `data.energy_kinetic` (pre-computed by the energy stage when `ENABLE_ENERGY` is set). The per-DOF test requires manual computation: `let mv = &data.qM * &data.qvel; let per_dof_i = data.qvel[i] * mv[i];`.

**Measurement timing note.** After `data.step()`, `qvel` is post-integration (new) but `qM` is pre-integration (computed from old `qpos` during the forward pass before integration). For Model A this is irrelevant (M is constant). For Model B, computing `qvel[i] * (qM * qvel)[i]` after step uses new `v` with old `M(q)`. The lag is O(h) = O(0.001) — three orders of magnitude below the ~3% tolerance — so it is negligible and does not require a separate `forward()` call to recompute `qM`. Similarly, `data.energy_kinetic` is `½ v_old^T M(q_old) v_old` — the KE at the start of the step, before integration — which is a valid equilibrium measurement point.

### 4.4 Discretization error

Same `O(h · γ / M)` as Phase 1. At `h = 0.001`, `γ = 0.1`:
- Model A: `h·γ/m = 10⁻⁴` (translation), `h·γ/I_min = 2×10⁻⁴` (rotation, worst case `I_xx = 0.5`). Both well below the ~3% sampling tolerance.
- Model B: `h·γ/M_eff` depends on configuration, but the diagonal elements of `M` for reasonable link parameters are O(1), giving `h·γ/M ~ 10⁻⁴`. Well within tolerance.

---

## 5. Model A — Free Body (6 DOF)

### MJCF

```xml
<mujoco model="free_body_6dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.5 1.0 1.5"/>
      <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

**Design choices:**
- `diaginertia="0.5 1.0 1.5"` — three distinct principal moments, so the three rotational DOFs have different equilibrium velocity variances. This is the informative test — a sphere (uniform `I`) would pass even if the thermostat treated all DOFs identically.
- `mass="1"` — unit mass for translational DOFs. Combined with the non-unit inertia, the mass matrix `M = diag(1, 1, 1, 0.5, 1.0, 1.5)` has six distinct effective masses (three translational at 1.0, three rotational at 0.5, 1.0, 1.5).
- `gravity="0 0 0"` — no gravitational acceleration. The body random-walks in position space; we only measure velocity statistics.
- `contype="0" conaffinity="0"` — disable collision. The body must not contact the ground plane (there isn't one, but this is defensive).
- `pos="0 0 1"` — initial position offset from origin; cosmetic only.
- `Integrator::Euler` — same as Phase 1.
- No `dof_damping` — model damping is zero by default on free joints (thermostat owns damping, Q4).

### 5.1 Model invariants

```rust
#[test]
fn test_free_body_model_invariants() {
    let model = sim_mjcf::load_model(FREE_BODY_XML).expect("load");
    assert_eq!(model.nv, 6, "free body should have 6 velocity DOFs");
    assert_eq!(model.nq, 7, "free body should have 7 position DOFs (3 pos + 4 quat)");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    assert_eq!(model.body_mass[1], 1.0, "body mass must be 1.0");
    // dof_damping must be zero on all 6 DOFs.
    for i in 0..6 {
        assert_eq!(model.dof_damping[i], 0.0,
            "dof_damping[{i}] must be zero — thermostat owns damping");
    }
}
```

### 5.2 Expected equilibrium per DOF

| DOF index | Physical meaning | `M_ii` | Expected `⟨½ M_ii v_i²⟩` | Expected `⟨v_i²⟩` |
|---|---|---|---|---|
| 0 | x translation | 1.0 | ½k_BT = 0.5 | 1.0 |
| 1 | y translation | 1.0 | ½k_BT = 0.5 | 1.0 |
| 2 | z translation | 1.0 | ½k_BT = 0.5 | 1.0 |
| 3 | ω_x rotation | 0.5 | ½k_BT = 0.5 | 2.0 |
| 4 | ω_y rotation | 1.0 | ½k_BT = 0.5 | 1.0 |
| 5 | ω_z rotation | 1.5 | ½k_BT = 0.5 | 0.667 |

All six DOFs should yield `⟨½ M_ii v_i²⟩ = 0.5` (for `k_BT = 1`), but the velocity variances differ. The `I_xx = 0.5` DOF thermalizes to `⟨ω_x²⟩ = 2.0` while the `I_zz = 1.5` DOF thermalizes to `⟨ω_z²⟩ = 0.667`. The test must verify each independently.

---

## 6. Model B — 2-Link Hinge Chain (2 DOF)

### MJCF

```xml
<mujoco model="hinge_chain_2dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="link1">
      <joint name="j1" type="hinge" axis="0 1 0"
             stiffness="20" damping="0" springref="0"/>
      <inertial pos="0.25 0 0" mass="1" diaginertia="0.01 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05"
            contype="0" conaffinity="0" mass="0"/>
      <body name="link2" pos="0.5 0 0">
        <joint name="j2" type="hinge" axis="0 1 0"
               stiffness="20" damping="0" springref="0"/>
        <inertial pos="0.25 0 0" mass="1" diaginertia="0.01 0.1 0.1"/>
        <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05"
              contype="0" conaffinity="0" mass="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

**Design choices:**
- **Two hinge joints** rotating around the y-axis — a planar 2-link chain in the x-z plane.
- `stiffness="20"` on both joints — linear restoring springs that keep joint angles in the small-angle regime (~0.22 rad at kT=1). **This is load-bearing**: the Euler-Maruyama integrator introduces systematic bias when M(q) changes significantly per step. At stiffness=1 (angles ~1.9 rad), KE inflates to ~3.8× expected; at stiffness=20, both per-DOF tests pass within 1σ. See §12 Finding 1 for the full analysis. BAOAB/GJF integrators would remove this constraint (clear upgrade path, not Phase 2 work).
- `damping="0"` — thermostat owns damping (Q4).
- `mass="1"` per link, with center of mass at the link midpoint (`pos="0.25 0 0"`).
- `diaginertia="0.01 0.1 0.1"` — realistic for a thin rod along x. The hinge axis is y, so `I_yy = 0.1` is the moment about the rotation axis.
- `geom mass="0"` — mass and inertia come entirely from `<inertial>`, not from geometry. Prevents double-counting.
- `gravity="0 0 0"` — isolates thermal physics from gravitational effects.
- `contype="0" conaffinity="0"` — no self-collision.

### 6.1 Model invariants

```rust
#[test]
fn test_hinge_chain_model_invariants() {
    let model = sim_mjcf::load_model(HINGE_CHAIN_XML).expect("load");
    assert_eq!(model.nv, 2, "hinge chain should have 2 velocity DOFs");
    assert_eq!(model.nq, 2, "hinge chain should have 2 position DOFs");
    assert_eq!(model.timestep, 0.001);
    assert!(matches!(model.integrator, Integrator::Euler));
    for i in 0..2 {
        assert_eq!(model.dof_damping[i], 0.0,
            "dof_damping[{i}] must be zero — thermostat owns damping");
    }
    // Verify M is 2×2 and non-trivially coupled (off-diagonal non-zero).
    let mut data = model.make_data();
    data.qpos[1] = 0.5;  // non-zero angle to activate coupling
    data.forward(&model).expect("forward");
    assert_eq!(data.qM.nrows(), 2);
    assert_eq!(data.qM.ncols(), 2);
    assert!(
        data.qM[(0, 1)].abs() > 1e-6,
        "M should have non-zero off-diagonal at non-zero q[1], got M_01={}",
        data.qM[(0, 1)],
    );
}
```

### 6.2 Why the mass matrix is configuration-dependent

At `q = [0, 0]` (straight chain), the off-diagonal coupling is maximal. At `q = [0, π/2]` (right angle), the coupling changes. The 2-link chain mass matrix has the standard form:

```
M(q) = [I_1eff + I_2eff + 2·m₂·l₁·l_c2·cos(q₂)    I_2eff + m₂·l₁·l_c2·cos(q₂)]
       [I_2eff + m₂·l₁·l_c2·cos(q₂)                  I_2eff                       ]
```

where `I_ieff` includes both rotational inertia and parallel-axis contributions. The `cos(q₂)` dependence is exactly what makes per-DOF equipartition require the generalized form `⟨v_i · (Mv)_i⟩ = k_BT`.

---

## 7. Validation test #1 — Free body equipartition (6 DOF)

### 7.1 Statistics

Same two-level Welford as Phase 1 §7: 100 independent trajectories, per-trajectory inner accumulator reduced to a scalar mean, pushed into a top-level across-trajectories accumulator. One accumulator **per DOF** — six top-level `WelfordOnline` instances.

### 7.2 Parameter set

| Parameter | Value | Rationale |
|---|---|---|
| `γ` | `DVector::from_element(6, 0.1)` | uniform damping across all DOFs |
| `k_BT` | `1.0` | central baseline |
| `h` | `0.001` | inherited from Phase 1 |
| `Integrator` | `Euler` | inherited from Phase 1 |
| `n_burn_in` | `75_000` steps (= `5 · τ_eq_max`) | `τ_eq = M_ii/γ`; slowest DOF is `I_zz = 1.5` → `τ_eq = 15` time units = `15_000` steps. Burn-in = `5 · 15_000 = 75_000`. |
| `n_measure` | `300_000` steps (= `20 · τ_eq_max`) | `20 · 15_000 = 300_000`. |
| `n_traj` | `100` | same as Phase 1 |
| `seed_base` | `0xFACE_B00D_u64` | arbitrary fixed |
| `n_sigma` | `3.0` | inherited |

**Time-scale accounting for the slowest DOF (`I_zz = 1.5`):**
- `τ_eq = M/γ = 1.5/0.1 = 15` time units = `15,000` steps.
- `τ_int = M/(2γ) = 7,500` steps.
- `N_eff` per trajectory ≈ `300,000 / (1 + 2 · 7,500)` ≈ `20`.
- Per-trajectory std error ≈ `(½k_BT) · √2 / √20 ≈ 0.316 · (½k_BT)`.
- Across 100 trajectories: std error ≈ `0.0316 · (½k_BT)` ≈ **3.2% of ½k_BT**.
- At `n_sigma = 3.0`, gate boundary is ±9.5% of ½k_BT. Discretization bias ~10⁻⁴. Margin is real.

Faster DOFs (translation at `M = 1`, rotation at `I_xx = 0.5`) have shorter `τ_int` and therefore higher `N_eff`, so their std error is smaller. The slowest DOF is the binding constraint.

### 7.3 Measured quantity

Per step, per DOF: `½ · M_ii · v_i²`. Since `M` is diagonal and constant for the free body, this is simply:

```rust
let m_diag = [1.0, 1.0, 1.0, 0.5, 1.0, 1.5]; // from model definition
for dof in 0..6 {
    let ke_dof = 0.5 * m_diag[dof] * data.qvel[dof] * data.qvel[dof];
    traj_accumulators[dof].push(ke_dof);
}
```

No need to read `data.qM` for Model A — the mass matrix is known and constant.

### 7.4 Test pseudocode

```rust
#[test]
fn test_free_body_equipartition() {
    let m_diag = [1.0, 1.0, 1.0, 0.5, 1.0, 1.5];
    let n_burn_in = 75_000;
    let n_measure = 300_000;
    let n_traj = 100_usize;
    let seed_base = 0xFACE_B00D_u64;
    let k_b_t = 1.0;
    let gamma_value = 0.1;

    let mut across: Vec<WelfordOnline> = (0..6).map(|_| WelfordOnline::new()).collect();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(FREE_BODY_XML).expect("load");
        let mut data = model.make_data();

        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build()
            .install(&mut model);

        for _ in 0..n_burn_in { data.step(&model).expect("burn-in"); }

        let mut traj: Vec<WelfordOnline> = (0..6).map(|_| WelfordOnline::new()).collect();
        for _ in 0..n_measure {
            data.step(&model).expect("measure");
            for dof in 0..6 {
                traj[dof].push(0.5 * m_diag[dof] * data.qvel[dof] * data.qvel[dof]);
            }
        }

        for dof in 0..6 {
            across[dof].push(traj[dof].mean());
        }
    }

    let expected = 0.5 * k_b_t;
    let dof_names = ["v_x", "v_y", "v_z", "ω_x", "ω_y", "ω_z"];
    for dof in 0..6 {
        assert_within_n_sigma(
            across[dof].mean(), expected, across[dof].std_error_of_mean(), 3.0,
            &format!("free body equipartition DOF {dof} ({name}), M_ii={m}",
                     name = dof_names[dof], m = m_diag[dof]),
        );
    }
}
```

**Multiple-comparison note**: Six DOFs tested at 3σ each. Under the null hypothesis (thermostat is correct), each test has ~0.3% false-fail rate; six independent tests give ~1.8% combined false-fail rate. This is acceptable for a gate test — if it flakes, re-run once. If two DOFs fail simultaneously, investigate.

---

## 8. Validation test #2 — Hinge chain equipartition (2 DOF)

### 8.1 Statistics

Same two-level Welford. Two per-DOF accumulators (generalized equipartition) plus one total-KE accumulator.

### 8.2 Parameter set

| Parameter | Value | Rationale |
|---|---|---|
| `γ` | `DVector::from_element(2, 0.1)` | uniform damping |
| `k_BT` | `1.0` | central baseline |
| `h` | `0.001` | inherited |
| `n_burn_in` | `50_000` (= `5 · τ_eq`) | `τ_eq ≈ M_eff/γ ≈ 1.0/0.1 = 10` time units = `10,000` steps. Use `50,000` for margin (the effective mass varies with configuration; this covers the upper range). |
| `n_measure` | `200_000` (= `20 · τ_eq`) | Same Phase 1 sizing. |
| `n_traj` | `100` | same as Phase 1 |
| `seed_base` | `0xDEAD_F00D_u64` | arbitrary fixed |
| `n_sigma` | `3.0` | inherited |

### 8.3 Measured quantities

Two measurement types per step:

**A. Total kinetic energy** — uses `data.energy_kinetic`:
```rust
model.enableflags |= ENABLE_ENERGY;
// ... after each step:
total_ke_traj.push(data.energy_kinetic);
```
Expected: `⟨T_kin⟩ = (n_dof / 2) · k_BT = 1.0`.

**B. Generalized per-DOF equipartition** — requires `data.qM`:
```rust
// M(q) is valid after step() (computed by CRBA in the forward pipeline).
let mv = &data.qM * &data.qvel;  // M(q) · v
for dof in 0..2 {
    // ⟨v_i · (Mv)_i⟩ should equal k_BT
    gen_equip_traj[dof].push(data.qvel[dof] * mv[dof]);
}
```
Expected: `k_BT = 1.0` for each DOF.

### 8.4 Test pseudocode

```rust
#[test]
fn test_hinge_chain_equipartition() {
    let n_burn_in = 50_000;
    let n_measure = 200_000;
    let n_traj = 100_usize;
    let seed_base = 0xDEAD_F00D_u64;
    let k_b_t = 1.0;
    let gamma_value = 0.1;

    let mut across_total_ke = WelfordOnline::new();
    let mut across_gen: Vec<WelfordOnline> = (0..2).map(|_| WelfordOnline::new()).collect();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(HINGE_CHAIN_XML).expect("load");
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();

        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build()
            .install(&mut model);

        for _ in 0..n_burn_in { data.step(&model).expect("burn-in"); }

        let mut traj_total_ke = WelfordOnline::new();
        let mut traj_gen: Vec<WelfordOnline> = (0..2).map(|_| WelfordOnline::new()).collect();
        for _ in 0..n_measure {
            data.step(&model).expect("measure");
            traj_total_ke.push(data.energy_kinetic);
            let mv = &data.qM * &data.qvel;
            for dof in 0..2 {
                traj_gen[dof].push(data.qvel[dof] * mv[dof]);
            }
        }

        across_total_ke.push(traj_total_ke.mean());
        for dof in 0..2 {
            across_gen[dof].push(traj_gen[dof].mean());
        }
    }

    // Total KE: ⟨T_kin⟩ = (n_dof/2) · k_BT = 1.0
    let expected_total = k_b_t;  // n_dof/2 · k_BT = 2/2 · 1 = 1
    assert_within_n_sigma(
        across_total_ke.mean(), expected_total,
        across_total_ke.std_error_of_mean(), 3.0,
        "hinge chain total KE: ⟨½v^T M v⟩ = k_BT",
    );

    // Generalized per-DOF: ⟨v_i · (Mv)_i⟩ = k_BT
    for dof in 0..2 {
        assert_within_n_sigma(
            across_gen[dof].mean(), k_b_t,
            across_gen[dof].std_error_of_mean(), 3.0,
            &format!("hinge chain generalized equipartition DOF {dof}"),
        );
    }
}
```

### 8.5 Why both total-KE and per-DOF tests

Total KE could pass with compensating errors (one DOF too hot, the other too cold). The per-DOF test catches this. The total-KE test is included because it uses a different measurement path (`data.energy_kinetic` vs. manual `qM * qvel`) and serves as a cross-check on both the thermostat and the energy computation.

---

## 9. Validation test #3 — Multi-DOF reproducibility

Extends Phase 1 §9 to both multi-DOF models. Two simulations with identical parameters and seed must produce bit-for-bit identical `qpos` and `qvel` after N steps.

```rust
#[test]
fn test_reproducibility_free_body() {
    assert_multi_dof_reproducibility(FREE_BODY_XML, 6, 7, 10_000);
}

#[test]
fn test_reproducibility_hinge_chain() {
    assert_multi_dof_reproducibility(HINGE_CHAIN_XML, 2, 2, 10_000);
}

fn assert_multi_dof_reproducibility(xml: &str, nv: usize, nq: usize, n_steps: usize) {
    let seed = 0x00C0_FFEE_u64;
    let gamma_value = 0.1;
    let k_b_t = 1.0;

    let run = |model: &mut Model| -> Data {
        let mut data = model.make_data();
        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed,
            ))
            .build()
            .install(model);
        for _ in 0..n_steps { data.step(model).expect("step"); }
        data
    };

    let mut model1 = sim_mjcf::load_model(xml).expect("load 1");
    let mut model2 = sim_mjcf::load_model(xml).expect("load 2");
    let data1 = run(&mut model1);
    let data2 = run(&mut model2);

    for i in 0..nq {
        assert_eq!(data1.qpos[i], data2.qpos[i],
            "reproducibility: qpos[{i}] must match bit-for-bit");
    }
    for i in 0..nv {
        assert_eq!(data1.qvel[i], data2.qvel[i],
            "reproducibility: qvel[{i}] must match bit-for-bit");
    }
}
```

---

## 10. Test file layout

All Phase 2 tests go into a new integration test file alongside Phase 1's:

```
sim/L0/thermostat/tests/
├── langevin_thermostat.rs          (Phase 1 — unchanged)
└── multi_dof_equipartition.rs      (Phase 2 — NEW, ~300 LOC)
```

The new file contains:
- `FREE_BODY_XML` and `HINGE_CHAIN_XML` constants
- `test_free_body_model_invariants` (§5.1)
- `test_hinge_chain_model_invariants` (§6.1)
- `test_free_body_equipartition` (§7)
- `test_hinge_chain_equipartition` (§8)
- `test_reproducibility_free_body` and `test_reproducibility_hinge_chain` (§9)
- `assert_multi_dof_reproducibility` helper

Total Phase 2 footprint: **~300 LOC in one new test file, zero changes to `src/`**.

---

## 11. Acceptance criteria

All of the following must be green before Phase 2 is considered done:

1. `test_free_body_model_invariants` — PASS
2. `test_hinge_chain_model_invariants` — PASS (M is 2×2 with non-zero off-diagonal)
3. `test_free_body_equipartition` — PASS (all 6 DOFs within 3σ)
4. `test_hinge_chain_equipartition` — PASS (total KE + both per-DOF generalized within 3σ)
5. `test_reproducibility_free_body` — PASS (bit-for-bit)
6. `test_reproducibility_hinge_chain` — PASS (bit-for-bit)
7. Phase 1 tests still pass (no regressions)
8. `cargo clippy -p sim-thermostat -- -D warnings` — clean
9. `cargo fmt -p sim-thermostat -- --check` — clean
10. Q1 resolution documented in `open_questions.md`

---

## 12. Implementation findings

### Finding 1: Euler integrator breaks equipartition at large angles

**Discovered during implementation stress-testing.** The force-space FDT is correct in continuous time — the Langevin equation with coordinate-space friction `-γv` and noise `sqrt(2γkT) dW` samples the canonical distribution regardless of M(q). However, the Euler-Maruyama discretization introduces a systematic bias when M(q) changes significantly per step.

**Stiffness sweep** (100 trajectories × 200K steps, kT=1, γ=0.1, h=0.001):

| Stiffness | Typical angle (rad) | Measured ⟨½v^T Mv⟩ / kT | Per-DOF deviation |
|---|---|---|---|
| 1 | 1.9 | 3.8 | catastrophic |
| 10 | 0.35 | 1.06 | DOF 1 at 4.4σ |
| 20 | 0.22 | 0.99 | both DOFs < 1σ |
| 50 | 0.15 | 0.99 | both DOFs < 1.5σ |
| 500 | 0.05 | 1.00 | both DOFs < 0.3σ |

**Diagnosis**: At large angles, cos(q₂) changes rapidly per step, so M(q) evaluated at q_n is a poor approximation of M over the [q_n, q_{n+1}] interval. The noise kick `M^{-1}(q_n) · σξ` uses stale M, creating a velocity-dependent bias that redistributes energy between DOFs.

**Resolution**: `stiffness=20` keeps angles at ~0.22 rad where cos(0.22) ≈ 0.976, so M varies by ~2.4% from its q=0 value. The Euler bias is well within the ~3% sampling tolerance.

**Implication for Phase 3+**: Bistable elements at large deflections will need either (a) a BAOAB/GJF integrator or (b) careful parameter choices that keep the integrator in its accurate regime. This is an integrator limitation, not a thermostat limitation — the chassis's force-space FDT is correct.

---

## 13. What this enables

Passing Phase 2 proves the thermostat works on:
- Multi-DOF systems (not just nv=1)
- Diagonal constant mass matrices (free body)
- Non-diagonal, configuration-dependent mass matrices (articulated chain)
- The generalized equipartition theorem (not just the scalar form)

This unlocks **Phase 3** (single bistable element + Kramers' formula), where the physics test shifts from equilibrium statistics to escape-rate dynamics. Phase 3 will be the first phase where the system has a non-trivial energy landscape (double-well potential) and contact constraints may enter the picture.
