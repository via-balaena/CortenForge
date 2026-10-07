> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# ledger-L32 — bodies with ≥ 2 joints: the velocity product

Researcher section, 2026-10-06. Repo source @ 3520544e (see the repo-state note at the end). Every number below comes from a script in `SCRATCH/rigid_spec/l32_multijoint/` (scripts in `scripts/`, outputs in `out/`). MuJoCo oracle: `SCRATCH/rigid_oracle_350/venv` (mujoco 3.5.0); source citations are from `SCRATCH/mj350src/mujoco` (tag 3.5.0, 881544c).

## 1. The fixture is the subject

`scripts/compare.py` loads the same XML in both engines, sets the same qpos/qvel (qvel = [1.0, 0.5, −0.3, 0.2, 0.1, 0.4, …]), and compares the model first. For hinge+hinge (`xml/hinge_hinge.xml`, the other researcher's fixture), the following are identical (difference 0): nq, nv, nbody, njnt, `jnt_pos`, `jnt_axis`, `body_pos`, `body_quat`, `body_ipos`, `body_mass`, timestep, gravity, qpos, qvel, `body_parent` and `jnt_body`. The body inertia tensor (`R(iquat)·diag(inertia)·Rᵀ`) differs by 2.8e-17. The same holds for every fixture used here (`out/compare_orig.txt`). The divergence does not come from the fixture.

## 2. The first quantity that differs

The comparison is stage by stage. MuJoCo's com-frame quantities are moved to our reference point `xpos[b]`: motion vectors use `lin + ang × (xpos − c)` and force vectors use `ang + (c − xpos) × lin`. MuJoCo runs `mj_kinematics`, `mj_comPos`, `mj_comVel` and `mj_rne(flg_acc=0)` separately.

| stage | hinge+hinge, qpos0 (main) |
|---|---|
| xpos, xquat, xanchor, xaxis, xipos, subtree_com | 0 |
| cdof (our `joint_motion_subspace` formula) | 0 |
| cinert (6×6 at xpos) | 1.4e-17 |
| cvel | 0 |
| **bias acceleration** (ours `cacc_bias`; MuJoCo Σ `cdof_dot·qvel` down the chain) | **0.500** |
| cfrc_bias | 0.075 |
| qfrc_bias | 0.0588 (MuJoCo 0.6116 / −1.5891, ours 0.5528 / −1.5962) |

The other fixtures, at qpos0 and at a nonzero qpos, with and without gravity, show the same pattern: slide+hinge, hinge+slide, hinge→ball, slide→ball and 3 slides + 3 hinges ("six"). In each one the first differing quantity is the bias acceleration (0.30 to 0.50). The controls are the same joints split across two bodies, with a 1e-9 kg intermediate body because MuJoCo refuses a massless one. They agree at every stage (`out/compare_orig.txt`).

One other difference is unrelated. For a ball joint, ours stores `xaxis = 0` (`forward/position.rs:112`), while MuJoCo stores the rotated `jnt_axis`. This does not feed the dynamics in these runs: `ball_only` and `hinge_ball_split` agree with MuJoCo to ≤ 9e-14 after 500 steps. I did not trace who else reads it.

## 3. Cause

**MuJoCo** (`engine_core_smooth.c`, `mj_comVel` :2277): for each dof, cdof_dot is computed from the body velocity **accumulated so far in this body**. `cvel` starts at the parent velocity (:2290). Each joint first computes `cdofdot_j = cvel × cdof_j` (hinge/slide :2331; ball :2315–2317, all three dofs using the velocity before the ball) and then adds its own `cdof·qvel` to `cvel` (:2335, :2321). `mj_rne` uses `cdof_dot·qvel` (:2451), and so does `mj_rnePostConstraint` (:2661). The velocity derivative does the same, with a running `Dcvel` (`engine_derivative.c` `mjd_comVel_vel_dense` :321, scalar joints :371–372).

**Ours** uses the transported **parent** velocity for every joint of the body:
- `sim/L0/core/src/dynamics/rne.rs:233-234`: `let v_parent_at_body = transport_motion_spatial(data.cvel[parent_id], &r); a_bias += spatial_cross_motion(v_parent_at_body, v_joint);`, inside the per-joint loop at :208.

This drops `Σ_{k<j} (S_k q̇_k) ×_m (S_j q̇_j)` for each body. On hinge+hinge, `ω₀ × ω₁ = (1,0,0)·1.0 × (0,1,0)·0.5` has magnitude 0.5, which equals the measured cacc difference. The comment at `rne.rs:221-223` justifies the parent velocity by saying "(S·qdot) ×_m (S·qdot) = 0". That holds only when the body has one joint.

The same pattern appears in three more places:
- `forward/acceleration.rs:603-604` (`mj_body_accumulators`, :400). This feeds `cacc`, `cfrc_int` and the acceleration sensors. On main, `cacc` differs from MuJoCo's `mj_rnePostConstraint` by 0.5 (hinge+hinge) and 1.5 (slide+hinge) (`out/compare_post.txt`).
- `derivatives/hybrid.rs:443-444` (`mjd_rne_vel`, :388): `dvt`/`vt` are taken once per body and used at :506 and :513.
- `derivatives/hybrid.rs:1278` (`mjd_rne_pos` operating point, :1098). The derivative block at :1820-1990 has no term for the same-body cross product.
- `sim/L0/gpu/src/shaders/rne.wgsl:236-254`: the same, looped per dof.

## 4. The fix and what it measured

The change touches 3 files and adds 87 lines (`out/fix.diff`, applied in `SCRATCH/rigid_spec/l32_multijoint/core_fix`):

```rust
// rne.rs (and the same in acceleration.rs and the mjd_rne_pos operating point)
let mut v_partial = transport_motion_spatial(data.cvel[parent_id], &r);
for jnt_id in jnt_start..jnt_end {
    // … v_joint = S·qvel …
    a_bias += spatial_cross_motion(v_partial, v_joint);
    v_partial += v_joint;
    // … free-joint correction unchanged …
}
```

- In `mjd_rne_vel`, `vt` and `dvt` become mutable and advance after each joint (`vt += v_joint`; `dvt[:, dof] += S[:, d]`). This mirrors `mjd_comVel_vel_dense`.
- In `mjd_rne_pos`, a new block before the free-joint correction differentiates `Σ_{k<j} v_k ×_m v_j`. It uses the file's existing convention (`∂v_i/∂q_m = s_ang_m ×_m v_i` for every ancestor DOF, and for a same-body DOF only for joints at or after its own joint).
- Rewriting the old comments (rne.rs:221-232, acceleration.rs:591-596, hybrid.rs:437-442, 1243-1252, 1820-1826, and `types/data.rs:159`) is **not** done in the scratch copy.

**Measured agreement with MuJoCo 3.5.0** (`out/dyn_cmp_final.txt`, `out/dyn_integrators_full.txt`, `out/compare_fix_final.txt`):
- **Stages:** every stage agrees to ≤ 1e-12 for all fixtures (the ball xaxis difference remains).
- **Euler, max |Δqvel|** over qpos0 and nonzero qpos:

  | fixture | after 1 step | after 500 steps |
  |---|---|---|
  | hinge+hinge | 0 | ≤ 2.2e-15 |
  | slide+hinge | 0 | ≤ 8.9e-15 |
  | hinge+slide | 0 | ≤ 8.3e-17 |
  | six | ≤ 1.7e-16 | ≤ 2.4e-14 |
  | hinge→ball | ≤ 4.7e-16 | ≤ 5.4e-14 |
  | slide→ball | ≤ 6.7e-16 | ≤ 1.2e-13 |

  The ball cases are not at ~1e-14. Single-joint ball bodies, which already agree on main, sit at the same level: `ball_only` 4.6e-14, `hinge_ball_split` 9.1e-14, the hinge→ball chain 1.1e-13. On main the same multi-joint cases were off by 2.3e-4 to 3.0e-3 after 1 step.
- **Other integrators:** Implicit, ImplicitFast and RK4 on hinge+hinge, slide+hinge, hinge→ball and six are ≤ 7.1e-14 after 500 steps. Implicit only agrees once `mjd_rne_vel` is fixed. With the forward fix alone it was off by 1.6e-5 after 1 step.
- **Body accumulators and inverse:** after the fix, `cacc` agrees to ≤ 2.1e-13 (ball) / 1.4e-14, `cfrc_int` to ≤ 2.3e-15, and `qfrc_inverse` to ≤ 1e-15.
- **Single-joint bodies stay bit-identical.** This is by construction: for the first joint, `v_partial` is the same expression as before. It was also measured:
  - The probe's qpos/qvel bits match for the split controls, `ball_only` and the chain after 1 and 500 steps.
  - Corpus harness (copy of `rigid_plan_tests/harness`, 100 steps, all qpos/qvel/act/time bits): **1,387 of 1,387** deterministic single-joint-per-body docs are identical between main and the fix. So are 14 of the 15 repo XML files; the other one is `knee_ref.xml`, which has a 3-hinge body.
- **Analytic vs FD derivatives** (`validate_analytical_vs_fd`, max |FD − analytic| per column block, `out/deriv_*.txt`):
  - Forward fix alone: velocity columns are off by up to 8e-3 and position columns by up to 3.7e-2.
  - Full fix: ≤ 1.4e-10 / ≤ 5.6e-10 on every fixture except two.
  - `hinge_slide` (6.8e-2) and `Model::multi_joint_body` (1.1e-1) were already off on main (7.2e-2 and 1.09e-1). That gap predates this fix and is separate; see open question 2.
- **Lib tests:** `cargo test --lib` in the scratch copy passes 720/720, the same as an unmodified copy. Clippy (repo lint set, `--all-targets -D warnings`, default features) is clean. rustfmt `--check` on the 3 files is clean.

## 5. Tests and examples whose numbers change

**Corpus** (`out/corpus_classify.txt`, by our loader). 91 docs have a body with ≥ 2 joints.
- 73 of them have only Slide×3 (flex-vertex) bodies, where `[0;a] ×_m [0;b] = 0`. 26 of those are deterministic and bit-identical. 47 are **nondeterministic run to run** in both main and fix (3 runs each), so the harness cannot compare them.
- The other 18 docs match the other researcher's 18. The trajectory changes in **6** of them:
  - `sim/L0/tests/integration/validation.rs:487`, `:571`, `:627`
  - `derivatives.rs:1899`
  - `sim/L0/mjcf/src/builder/actuator.rs:795`
  - `examples/fundamentals/sim-ml/ML_COMPETITION_SPEC.md:920` (a markdown block that nothing runs)
- The remaining 12 are unchanged:
  - 10 have a zero cross term (coaxial hinges, or slide+slide).
  - 2 (`builder/joint.rs:436`, `builder/build.rs:1192`) have a nonzero cross term, but isotropic inertia centred on the joints. That matches the other researcher's "sphere centred shows no difference".

**Suites run, main vs fix** (each in a scratch copy, `-j 4`):

| suite | main | fix |
|---|---|---|
| sim-conformance-tests `integration` | 1338 pass / 27 ignored | identical |
| sim-conformance-tests `mujoco_conformance` | 83 pass | identical |
| sim-conformance-tests ignored `golden_flags` | 25 fail | same values in both |
| sim-mjcf lib + `forward_conformance` | 386 | identical |
| cf-mjcf-emit tests (`knee_ref.xml` has a 3-hinge body; whether the models its dynamics tests emit do: not checked) | 18 pass | identical |

**No test I ran pins today's wrong values.** With the forward-only fix, `derivatives::test_pos_deriv_multi_joint_body` (`integration/derivatives.rs:1935`) **fails** (relative error 2.6e-2 > 1e-4). So the forward and derivative changes must land together.

**sim-gpu, not run.** T15c (`sim/L0/gpu/src/pipeline/tests.rs:1189`, GPU vs CPU at tolerance 1e-3, `Model::multi_joint_body`) will fail wherever a GPU adapter exists unless `rne.wgsl` changes too. The CPU change on that fixture is 4.8e-2 (`out/`, `l32-probe mjb`: main [2.3397, 5.3507, 0.2974] → fix [2.2959, 5.3922, 0.3453]).

## 6. Tests to add (each fails on main)

1. **Invariant.** The same two joints on one body, and split across two bodies with a 1e-9 kg intermediate body, must give the same `qacc` to 1e-6:

   | | hinge+hinge | slide+hinge | hinge→ball |
   |---|---|---|---|
   | main | 0.117 | 1.50 | 0.569 |
   | fix | 4.1e-10 | 2.4e-8 | 4.6e-8 |

2. **MuJoCo 3.5.0 golden.** `xml/hinge_hinge_g0.xml` at qpos0 with qvel (1, 0.5) gives `qfrc_bias = [0.12111494293556116, −0.11756763963282571]`. Main gives `[0.06233, −0.12466]`; the fix matches to ≤ 1e-16. Add the same check for slide+hinge and hinge→ball.
3. **Derivatives.** `validate_analytical_vs_fd` on hinge+hinge, hinge→ball and six, absolute ≤ 1e-9. This does not fail on main; it guards the forward/derivative coupling.
4. **Implicit integrator vs MuJoCo** on hinge+hinge: 1.6e-5 apart after 1 step with the forward fix alone, 0 with the full fix.

## 7. Open questions

1. **GPU in the same commit?** Options: change `rne.wgsl` now, or ship the CPU fix and let T15c fail on GPU machines. I recommend changing it now, stepping the partial velocity per **joint**, not per dof, so the three ball dofs share the pre-ball velocity as at `:2315-2317`. If that is wrong (for example, if the GPU path is meant to lag), T15c goes red on any GPU run. I have not implemented or run this.
2. **Pre-existing position-derivative gap**: `hinge_slide` and `multi_joint_body` disagree with FD on main. Both fixtures have a hinge followed by a slide on one body. Every fixture without that pattern agrees to ≤ 5.6e-10. The mechanism is **not isolated**; it needs its own ledger row.
3. **In Rigid or its own PR**: this is Jon's call (ledger). The change is one commit: core forward, accumulators, both derivatives, plus the GPU shader if (1) is accepted.

**Commit:** `fix(sim-core): the velocity product of a multi-joint body uses the earlier joints' velocity (MuJoCo mj_comVel)`. It carries tests 1–4, the comment rewrites and (1). It depends on no other area. The ball `xaxis` difference and the harness nondeterminism are separate findings and are not addressed by this commit.

## What this method cannot see

- I did not run the suites for sim-gpu, sim-urdf, sim-thermostat, cf-design or cf-design-tests, cf-osim, cf-msk-fit, cf-codesign, sim-rl/ml-chassis/opt, L1 crates, examples (validate-examples), doctests, or the licensed gates.
- MJCF generated at runtime (`format!` templates, cf-design mechanisms) is outside the static corpus. cf-design builds multi-joint bodies (`model_builder.rs:612-623`). The one test I found that uses one (`:2137`) checks only qM and kinetic energy.
- Sleep was not enabled in any run.
- The 47 nondeterministic Slide×3 docs.

**Repo state.** Before: `main` @ 3520544e, `git status --short` empty. After: `git status --short` is still empty, but HEAD is `ea11c8d6` on branch `fix/rigid-sim-core-mjcf`. The reflog shows that branch checked out at 20:03 with two commits at 20:05 and 20:09. I did not make them (I ran no git write command). They touch only `sim/docs/todo/spec_fleshouts/rigid_0_10/*.md` (`git diff --name-only 3520544e HEAD`), so the source I built against equals 3520544e.
