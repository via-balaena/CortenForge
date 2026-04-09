# 2026-04-09 (part 2) — Forward step + integrator code recon

> Extracted from `MASTER_PLAN.md` §7 part 02 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Read `forward/mod.rs`, `integrate/mod.rs`, `integrate/euler.rs`,
  and traced `implicit_damping` usage across the crate to resolve Q2 and
  tighten Q4 before drafting the Phase 1 spec.
- **Pipeline structure**: `step()` → `forward()` → `integrate()`. The forward
  pipeline is split into `forward_pos_vel()` (FK, collision, velocity FK,
  energy, sensors) and `forward_acc()` (actuation, RNE, passive, constraint
  solve, qacc). `cb_control` fires at the boundary between them.
- **Documented force-injection API**: `step1()` runs `forward_pos_vel()` +
  `cb_control`; `step2()` runs `forward_acc()` + `integrate()`. The user is
  *expected* to write into `ctrl`, `qfrc_applied`, or `xfrc_applied` between
  step1 and step2. The doc comment on `step1` literally calls this out as
  the RL force-injection point. The thermostat is just another consumer of
  this hook. RK4 is excluded by design (multi-stage substeps recompute
  `forward()` internally) — fine, since Langevin + RK4 is mathematically
  problematic anyway.
- **Force flow trace**: `qfrc_applied[i] += …` → folded into `qfrc_smooth` in
  `constraint/mod.rs:78-87` (`qfrc_smooth = qfrc_applied + qfrc_actuator +
  qfrc_passive − qfrc_bias`, plus `xfrc_applied` projection) → constraint
  solver computes `qfrc_constraint` → `mj_fwd_acceleration` computes `qacc`
  with `M_hat = M − h·∂f/∂v` for implicit integrators → `integrate()` does
  `qvel += qacc * h` (or the Eulerdamp `(M + h·D)·qacc_new = rhs` solve when
  `implicit_damping > 0`). Position integration `qpos += qvel * h` then runs
  in `mj_integrate_pos` with proper SO(3) handling for ball/free joints, and
  `mj_normalize_quat` cleans up quaternion drift.
- **`implicit_damping` is model-owned state**: Populated from `jnt_damping`
  at model init (`model_init.rs:911`). Consumed by Eulerdamp
  (`integrate/mod.rs:85,94,136`), the constraint solver
  (`constraint/mod.rs:149,221`), implicit-fast acceleration
  (`forward/acceleration.rs:103`), and the derivatives engine
  (`derivatives/hybrid.rs:78`). Mutating it per step would couple thermostat
  damping to model damping in ways that break separation of concerns.
- **Free wake-up bonus**: `island/sleep.rs:132-150,522-539` already inspects
  both `qfrc_applied` and `xfrc_applied` for wake detection. **A sleeping
  body receiving thermal noise wakes up automatically.** No thermostat-
  specific logic needed for islanded simulations. Correctness win for free.
- **Q2 RESOLVED**: Use `qfrc_applied` injection between step1/step2. No
  BAOAB, no integrator bypass. Integrator-agnostic across Euler /
  ImplicitFast / Implicit.
- **Q4 RESOLVED (option a)**: Thermostat owns its own `γ_thermostat[i]` and
  writes both `−γ·qvel` and the FDT noise into `qfrc_applied`. Model damping
  stays untouched.
- **Phase 1 algorithm sharpened**: Explicit Langevin (Euler-Maruyama). For
  `M=1, k_spring=1, γ=0.1, k_B·T=1, h=0.001`: natural frequency `ω=1`,
  `h·ω=0.001`, damping ratio `ζ=γ/(2√(kM))=0.05` (underdamped, healthy
  mixing), discretization temperature error `O(h·γ/M) ≈ 10⁻⁴` — well below
  the `~10⁻²` sampling-error tolerance for 10⁵ samples. Equipartition test
  should pass with margin.
- **Did NOT resolve**: Q1 (constraint projection — irrelevant for 1-DOF
  Phase 1, gates Phase 2), Q3 (`thrml-rs` existence — gates Phase 6),
  Q5 (cf-design end-to-end differentiability — gates Phase 5), Q6 (Phase 7
  reward signal — defer until Phase 6 results inform the choice).
- **Phase-1-blocking recon items still open** (small, listed in the post-
  recon report): PRNG conventions, `qfrc_applied` lifecycle / auto-clearing,
  existing usage patterns for writing into `qfrc_applied`, `model.timestep`
  variability, statistical-test infrastructure conventions, where the
  thermostat module should live in the crate, public mutability of
  `Data::qfrc_applied`.
- **Next action**: Answer the Phase-1-blocking recon items (small focused
  recon round), *then* draft the Phase 1 spec.

