# Actuator dynamics — recon + PR1

*Keystone deepening. Written 2026-06-15. The second dynamic-coverage leaf (after joint
damping): the coupled gradient through a MuJoCo `<actuator>` — the on-ramp to the powered
exo (you cannot drive an exo without actuators). PR1 = the direct-torque MOTOR + its
joint control gradient.*

## 1. The physics (sim-core)

A MuJoCo actuator drives the joint with `qfrc_actuator = moment^T · force`,
`force = gain·ctrl + bias`. For the affine force law, `gain` and `bias` are affine in the
transmission length/velocity. A **motor** is the simplest: `force = gain·ctrl`, `gain = 1`
(`gainprm = [1,0,0]`), `bias = 0`, so `qfrc_actuator = gear·moment·ctrl` (the gear folds
into `moment`). Under the default Euler integrator the actuator force is **explicit** in
`f_ext` (NOT in `M_impl` — `compute_implicit_params` only folds passive joint damping), so:

- The **control channel** `∂qfrc_actuator/∂ctrl` is the new gradient piece. It reaches the
  next velocity through the SAME implicit factor as the contact wrench:
  `∂qvel'/∂ctrl = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl` (`M_impl = M + Δt·D`, the damping leaf).
- Actuator **state-feedback** (position/velocity servos: `∂force/∂qpos`, `∂force/∂qvel`)
  is explicit → it flows through the unloaded transition `A`, which the FD
  `loaded_state_jacobian` captures **once the scratch steps replicate `ctrl`** (they don't
  today). That's the PR2 piece.

## 2. S0 spike (THROWAWAY) — carry column CONFIRMED

On a motor-driven (gear 3) damped hinge: `∂qfrc_actuator/∂ctrl = 3.0` (= gear, gain 1);
the carry column `Δt·M_impl⁻¹·∂qfrc/∂ctrl` matches the full-step FD to **6.0e-11**; the
single-hinge closed form `Δt·gear/(M + Δt·D)` matches to **1.3e-10**.

## 3. PR1 — the single-hinge motor control gradient

- `actuator_velocity_column` → `Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl` (`nv × nu`).
  `∂qfrc_actuator/∂ctrl` is a forward-only central FD of `qfrc_actuator` — EXACT for the
  affine-in-`ctrl` force law (no truncation), and robust: the persisted
  `data.actuator_moment` is **transient (reads 0 after `forward`)**, but `qfrc_actuator`
  is the reliable output. (★ sim-core observation: `actuator_moment` is used to assemble
  `qfrc_actuator`/`actuator_velocity` during forward but not persisted afterward.)
- `RigidStateCarryVjp` gains an optional control channel: `s' = J_state·s + G·w + G_act·u`,
  `G_act = [Δt·G_act_vel; G_act_vel]` (the same integrator tangent `D` as `G_pos`). `None`
  on the material/passive path ⇒ two parents, byte-identical.
- `coupled_trajectory_actuator_gradient(controls)` (tape) + `coupled_trajectory_actuated_z`
  (the control-aware FD oracle). Each `u_k` is a tape parameter feeding the carry's third
  parent; the reverse pass gets BOTH the direct joint drive AND the indirect coupled path.

**Scope.** Single-hinge MOTOR (state-independent force, config-independent `M`), so the
constant actuator force does not perturb `J_state` (the analytic single-hinge carry stays
exact and the FD `J_state` needs no `ctrl` replication). Flat normal; joint damping
supported (via `M_impl`). Gate: `actuator_control_gradient_matches_fd` — `∂tip_z_N/∂u_k`
vs the full-coupled FD, machine-exact (rel ~1e-11); `motor_control_moves_the_tip`
(materiality).

## 4. Follow-ons
- **State-feedback servos** (position/velocity): the control channel is identical; the
  state channel `∂qfrc/∂(qpos,qvel)` rides the FD `J_state` once `scratch_state_step`
  replicates `ctrl`. The PD joint controller an exo actually uses.
- **Chains** (`nv > 1`): the constant actuator force perturbs `J_state` through `∂M⁻¹/∂q`,
  so the scratch must replicate `ctrl` there too.
- **Muscles / activation dynamics** (`act` state, nonlinear gain) — the heavier nonlinear
  actuator.
- **Joint actuator + design on one tape** (the full co-design gradient through an actuator).
