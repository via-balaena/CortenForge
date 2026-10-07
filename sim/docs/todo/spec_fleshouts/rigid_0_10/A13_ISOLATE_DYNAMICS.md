> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# Three measured divergences from MuJoCo 3.5.0: connect/weld impedance (ledger-L39a), implicit-integrator qacc (ledger-L38a), flex edge rows (ledger-L39c)

Researcher section, 2026-10-06. Repo source at HEAD `139f7b25` (branch `fix/rigid-sim-core-mjcf`; `git diff --name-only 3520544e HEAD` touches only `sim/docs/todo/spec_fleshouts/rigid_0_10/*.md`, so the source equals `3520544e`). Evidence is in `SCRATCH/rigid_spec/isolate_dynamics/` (written `ID/` below): `scripts/`, `xml/`, `out/`, `probe/` (a Rust probe), `harness/` (a corpus harness), `validators/` (copies of five example validators). Oracle: `mujoco==3.5.0` (`SCRATCH/rigid_oracle_350/venv`). MuJoCo citations are from `SCRATCH/mj350src/mujoco` (tag 3.5.0, 881544c).

**Method** (as `l32_multijoint.md`):
1. Confirm that the fixture is identical in both engines.
2. Find the first intermediate quantity that differs.
3. Cite the code on both sides.
4. Prototype the fix in a scratch copy of sim-core (`ID/core_fix`, diff in `ID/out/prototype_core.diff`).
5. Show agreement, show that unaffected models stay bit-identical, and list the flips.

The prototype puts each fix behind an environment switch, so one build serves both main and the fix: `ISO_IMP=1` (item 1), `ISO_FIX=2` (item 2; `ISO_FIX=1` is a warmstart-only variant), `ISO_FLEX=1|2|3` (item 3). The switches are scratch-only. With every switch unset, the code paths are main's:
- The cable trajectories are byte-identical to the repo crates (`ISO_FIX=0` vs `orig`, 3 docs × 500 steps).
- On every one of the 1,430 corpus docs whose result does not vary from run to run, the harness's `traj_fp` equals the prior main run's `SCRATCH/rigid_plan_tests/emb_1.jsonl`. All 40 docs where the two differ are among the 48 docs that vary run to run (`ID/out/corpus_cmp.txt`).

| | first differing quantity | cause (ours ↔ MuJoCo) | prototype result |
|---|---|---|---|
| L39a connect | `efc_imp` (impedance) | ours: one impedance per row, `constraint/assembly.rs:83-84`. MuJoCo: one per constraint, from the norm of all its rows, `engine_core_constraint.c:1386-1391`, `:1476-1484` | cables 4.8e-5 → ≤ 1.9e-14 qpos over 500 steps; applies to connect **and weld**, every integrator and solver |
| L38a accelerometer | the stored `qacc` | ours stores the implicit solve's acceleration in `data.qacc` (`forward/acceleration.rs:316-317`, `:364-365`). MuJoCo keeps the explicit `qacc` and solves `M − h·D` into a stack local (`engine_forward.c:1134`, `:1348`) | accelerometer 1.77e-1 → 8.9e-16 (delay_history's own case); A and B derivatives bit-identical |
| L39c flex edges | `nefc`, then `efc_diagApprox` | ours: a row for every edge, diagApprox `MJ_MINVAL` (`equality_assembly.rs:77-136`). MuJoCo: rows only for a flex named by a `flex` equality, rigid edges skipped, diagApprox `flexedge_invweight0` (`engine_core_constraint.c:616-643`, `:1153-1165`) | pinned 3×3 grid 3.5e-2 → ≤ 5.4e-16 |

---

## 1. ledger-L39a: connect constraints (and welds)

### 1.1 Fixture
MuJoCo's own composite expansion is written out as explicit XML at 17 digits (`ID/scripts/explicit.py`, from `flex_composite/comp/{234f1d7a,67374b97,f529edbd}.xml` → `ID/xml/cable_{loaded,catenary,connect_stab}.xml`). Both engines then load plain bodies, and our composite `curve` bug (M8) is out of the picture.

**One fixture defect was found and corrected.** MuJoCo's writer emits `solref="0.002"` (one value). Ours reads that as the default (0.02); MuJoCo reads it as (0.002, 1) (`ID/out/cable_connect_stab_*_f0.json`, `eq_solref` Δ 0.018). The XMLs were rewritten to `solref="0.002 1"`. This parser defect is handed off (§5).

After that correction, a qpos0 comparison by `ID/scripts/cmp.py` finds the following (`ID/out/fixture_identity.txt`):
- **Equal**: nq, nv, bodies, `jnt_*`, `dof_damping`, `eq_obj*id`, `eq_solref`, `eq_solimp`, and `eq_data[0..10]`.
- **Below 1e-12 relative**: masses, inertia tensors, `qM`, `xpos`, `efc_J`.
- **`eq_data[10]`**: ours 0, MuJoCo 1. This is weld's torquescale slot, which a connect never reads (`engine_core_constraint.c:431-434` reads `data[0..6]`).
- **`body_ipos`** of a body with no mass and no geoms.
- **`dof_invweight0`**: 8e-12 relative.
- **`cacc`**: ours computes it lazily, so it is not filled after `forward()` unless a sensor needs it.

The same holds for the minimal fixtures.

### 1.2 First differing quantity
- Over 500 steps the gaps are 8.88e-9 / 4.83e-5 / 4.83e-5 qpos (stab / loaded / catenary). This reproduces the flex_composite researcher's numbers (`ID/out/cable_*_mj_traj.jsonl`).
- The gap is **1.1e-16 after step 1 and 5.7e-9 after step 2**.
- Comparing the forward pass at the start of step 2 (`ID/out/cable_loaded_{ours,mj}_s2.json`):
  - `xpos` ≤ 4.4e-16, `efc_J` 7.2e-16, `efc_pos` 6.7e-16, `efc_vel` 9.9e-17;
  - **`efc_imp` 4.0e-5**, then `efc_R` 1.2e-3, `efc_aref` 1.4e-5, `efc_force` 2.6e-4.
- At qpos0 the violation is 0, so `efc_imp` is equal there, and step 1 agrees.

**Measured, not the cause:** the warmstart of item 2. Prototype `ISO_FIX=1` (MuJoCo's explicit warmstart) and `ISO_FIX=2` (explicit `qacc` in Data) leave the cable gaps at 8.88e-9 / 4.83e-5 (`ID/out/cable_*_fix{1,2}_traj.jsonl`).

### 1.3 Cause
- **MuJoCo** (`engine_core_constraint.c`):
  - `getposdim` (`:1366`) sets `pos = ‖efc_pos[i..i+3]‖` for a connect (`:1388-1390`) and `‖efc_pos[i..i+6]‖` for a weld (`:1385-1387`).
  - `mj_makeImpedance` (`:1465`) computes one `imp` from it (`:1479`) and writes the same `imp` into `R` and `KBIP` of all `dim` rows (`:1482-1519`). Each row keeps its own `diagApprox`. It skips the rest of the constraint (`:1524`).
  - `aref` uses each row's own `pos` (`mj_referenceConstraint`, `:2375-2386`).
- **Ours**:
  - `finalize_constraint_row` (`constraint/assembly.rs:53`) computes `violation = |pos − margin|` and the impedance **per row** (`:83-84`).
  - `assemble_equality_rows` calls it per row with that row's `pos` (`constraint/equality_assembly.rs:31-74`, call at `:55`).
  - A row with no violation gets `imp(0) = d0`, while MuJoCo gives it `imp(‖pos‖)`.
- **Checked at one state** (`xml/conn_pend_Euler_Newton.xml`, hinge θ = 0.002):
  - `efc_pos` = (4.0e-7, 0, −6.0e-4) in both engines.
  - MuJoCo `efc_imp` = 0.9340000586665481 on all three rows.
  - Main gives (0.9000000159999627, 0.9, 0.934000047999936).
  - The prototype gives MuJoCo's value bit-for-bit, and the same `qacc` −4.870860884148606 (`ID/out/conn_pend_Euler_Newton_{ours,fix,mj}_st.json`).

### 1.4 Connect-general? Yes, and weld too
These are plain-body fixtures with no cable and contacts off (`ID/out/minimal_connect_weld.txt`, `minimal_connect_free2.txt`, `weld_torque.txt`, `rk4_implicit_eq.txt`). Max |Δqpos| vs MuJoCo over 500 steps:

| fixture | main | `ISO_IMP=1` |
|---|---|---|
| hinge pendulum + free box joined by a connect; Euler and implicitfast × Newton | 3.5e-4 | ≤ 2.8e-15 |
| same, Euler PGS | 3.5e-4 | 1.1e-15 |
| same, implicitfast PGS | 3.5e-4 | 1.3e-7 (2.6e-15 once item 2's explicit warmstart is added) |
| same, RK4 Newton | 3.2e-4 | 1.9e-15 |
| same, CG (both integrators) | 3.5e-4 | 1.9e-7 (separate, §1.8) |
| pendulum tip connected to world; Newton, PGS, CG | 3.1e-12 | ≤ 4.0e-13 |
| free box welded to world; Newton, PGS, both integrators | 3.2e-4 | ≤ 1.7e-15 |
| hinge + offset box, weld with rotational violation, `solimp="0.8 0.95 0.01 0.4 3"`; Euler, implicit | 9.9e-3 | ≤ 1.8e-15 |
| the 3 cable docs (implicitfast, Newton) | 8.9e-9 … 4.8e-5 | ≤ 1.9e-14 qpos / 7.9e-13 qvel |

The cable residual is the floor that flex_composite measured for these docs with the equality removed (≤ 2.3e-15 qpos), times a few. That was not checked further.

### 1.5 Now → Target → Change
- **Now:** one impedance per row (`assembly.rs:83-84`).
- **Target:** MuJoCo's `getposdim`. For a connect (3 rows) and a weld (6 rows), every row's impedance uses `‖pos‖` over the constraint's rows. All other row kinds are unchanged:
  - joint, tendon, distance and flex edges have dim 1;
  - contacts already share the normal row's impedance through the elliptic post-fix at `contact_assembly.rs:123-170`, which was not examined here.
- **Change** (crate-private, no public signature):
  ```rust
  // constraint/assembly.rs — the impedance argument becomes explicit
  pub fn finalize_constraint_row(/* …existing 16 args… */, imp_violation: f64)
  //   imp = compute_impedance(solimp, imp_violation);
  //   every other call site passes (pos - margin).abs()
  // constraint/equality_assembly.rs, per equality:
  let imp_violation = match model.eq_type[eq_id] {
      EqualityType::Connect | EqualityType::Weld => rows.pos.iter().map(|p| p * p).sum::<f64>().sqrt(),
      _ => rows.pos[r].abs(),
  };
  ```
  The prototype added a sibling `finalize_constraint_row_imp` instead (`ID/out/prototype_core.diff`); either shape gives the same arithmetic. `efc_pos` and `aref` stay per row, as in MuJoCo.
  - The GPU needs no change: `sim/L0/gpu/src/shaders/assemble.wgsl` assembles contact rows only. Grepping it for equality, connect, weld and flex finds nothing.
  - The doc comment of `finalize_constraint_row` (`assembly.rs:43-49`) gains the new argument; `compute_impedance` (`impedance.rs:32-43`) is unchanged.

### 1.6 Unaffected models stay bit-identical
Corpus harness (`ID/harness`, 100 steps, fingerprints of qpos/qvel/act/time, all sensordata of every step, and `qacc` after a final `forward()`). It was run with no switch 5 times, which flags 48 docs as varying run to run (47 flex, 1 RK4), and once per switch (`ID/out/corpus_*.jsonl`, `corpus_cmp.txt`, `corpus_detail.txt`):
- **0** of the docs that do not vary run to run and have no `<connect>`/`<weld>` change, on any fingerprint.
- **45 of the 55** deterministic connect/weld docs change trajectory bits. The other 10 are unchanged; why was not examined (listed in `corpus_detail.txt`).

### 1.7 Tests and examples that flip
All of the following were run in scratch copies against the prototype; source line numbers refer to the repo.
- **Unchanged:** sim-conformance `integration` (1,338 pass, 27 ignored), `mujoco_conformance` (83), sim-core lib (720), sim-mjcf lib (385), `forward_conformance` (1), and the 24 ignored golden-flag tests (same failure messages per test, `ID/out/golden_*.txt`). **No test I ran pins today's values.**
- **Validators:**
  - `example-equality-stress-test` stays rc 0 with all PASS, but its printed numbers change. For example, "Pivot 2 attached: max err 7.85 → 7.84 mm" and "Energy growth −3.42 → −3.41 %" (`ID/out/val_equality_stress_{base,IMP}.out`).
  - `example-mocap-bodies-stress-test` and `example-sleep-wake-stress-test`: stdout byte-identical.
- **Corpus docs whose bits change (45)** are listed with their sources in `corpus_detail.txt`. They include:
  - `equality_constraints.rs` ×14 (e.g. `:56`, `:109`, `:148`);
  - `newton_solver.rs:380/1395/1513`;
  - `sleeping.rs:2498/2615`;
  - `implicit_integration.rs:1202`;
  - `mujoco_conformance/layer_a.rs:323`;
  - the equality-constraints examples (stress-test ×8, connect-to-world, connect-body-to-body, weld-to-world, weld-body-to-body);
  - mocap `drag-target` and `stress-test`;
  - sleep-wake `stress-test`;
  - composites `cable-loaded:43` and `cable-catenary:58`;
  - `sim/L1/bevy/examples/crank_slider.rs:42`.
- **Not run:** the Bevy demos.

### 1.8 Tests to add (each fails on main)
1. `connect_impedance_is_shared_across_rows`: `conn_pend_Euler_Newton.xml` at qpos = 0.002. Assert that all 3 `efc_imp` equal MuJoCo 3.5.0's 0.9340000586665481 (to 1e-15), and that `qacc[0]` = −4.870860884148606 (1e-12). **Main fails**, with imp (0.90000002, 0.9, 0.93400005); the prototype passes.
2. `weld_matches_mujoco_3_5_0`: `weld_torque_Newton.xml`, 500 steps, against MuJoCo 3.5.0 golden qpos at 1e-12. Main is 9.9e-3 off; the prototype is 1.8e-15 off.
3. Promote the cable check: `implicit_integration.rs:1202`'s doc vs MuJoCo's explicit expansion at 1e-12. Main is 8.9e-9 off; the prototype with item 2 is 1.9e-14 off. This needs M8's curve keywords, or MuJoCo's explicit XML as the fixture.

### 1.9 Found alongside
- **CG differs separately; not isolated.** Main and the prototype both differ from MuJoCo after step 1 on `weld_free_Euler_CG` (qvel 7.0e-6). With `iterations="1000" tolerance="1e-15"` the 500-step gap falls from 6.1e-5 to 1.8e-8 (`ID/out/weld_free_Euler_CGtight_*`). MuJoCo stops after 4 iterations at step 2 (`solver_niter`); our iteration count was not measured. What differs between the two CG implementations has not been isolated.
- **The weld `torquescale` attribute is not implemented.** `git grep -i torquescale -- sim/L0` finds 0 files; the control `polycoef` finds 14. MuJoCo scales the rotational rows by it (`engine_core_constraint.c:482`, `:510`, `:530`) and reads it at `xml_native_reader.cc:2192`. Under decision 19 it must be refused or implemented (parser/M15).
- **A one-value `solref` silently becomes the default** (§1.1). This belongs to mjcf-S2/A4 (attribute lengths).

---

## 2. ledger-L38a: implicit and implicitfast store the implicit acceleration in `data.qacc`

### 2.1 Fixture and first differing quantity
`ID/xml/acc_hinge_{Euler,implicitfast,implicit,RK4}.xml`: a damped hinge pendulum with accelerometer, framelinacc, frameangacc and jointvel sensors. It is forwarded at qpos = 0.3, qvel = 1.5 (`ID/out/acc_hinge_*_st.json`):
- Equal: qpos, qvel, `xpos`. Within 1.1e-16: `qM`, `qfrc_bias`, `qfrc_passive`, `qfrc_smooth`. `qacc_smooth` within 7.6e-15.
- **First differing quantity: `qacc`.** Ours −2.2220866570183677, MuJoCo −2.907859844926252, a gap of 0.686. That gap carries into `cacc` and `sensordata`, also 0.686.

With the prototype (`ISO_FIX=2`), `qacc`, `cacc` and `sensordata` all agree to ≤ 7.6e-15. So the sensor's own computation and the state it reads already agree; only the `qacc` it is given differs.

On delay_history's own case (`SCRATCH/rigid_spec/delay_history/golden_acc.json`, 20 steps with its ctrl sequence; `ID/out/dh_*`), the undelayed accelerometer gap:

| | Euler | implicitfast | implicit | RK4 |
|---|---|---|---|---|
| main | 8.9e-16 | **1.77e-1** | **1.77e-1** | 1.8e-15 |
| `ISO_FIX=2` | 8.9e-16 | 8.9e-16 | 8.9e-16 | 1.8e-15 |

Over 200 steps on `acc_hinge` the sensordata gap goes from up to 10.7 (main) to ≤ 2.1e-14, while qpos and qvel are ≤ 4.4e-16 in both (`ID/out/acc_hinge.txt`). Trajectories never differed: both engines integrate with the implicit acceleration.

### 2.2 Cause
- **MuJoCo:**
  - `mj_forwardSkip` runs `mj_fwdAcceleration` and then `mj_fwdConstraint` (`engine_forward.c:1404-1405`). `mj_fwdConstraint` leaves the explicit `qacc` in `d->qacc`: `qacc_smooth` when `nefc = 0` (`:747-750`), the solver's result otherwise.
  - `mj_sensorAcc` then reads it (`:1408`), through `mj_rnePostConstraint`'s `d->qacc` (`engine_core_smooth.c:2663`).
  - The implicit solve happens later, in the integrator. `mj_implicitSkip` (`engine_forward.c:1128`) allocates `qacc` as a stack local (`:1134`), solves `(M − h·qDeriv)` from `qfrc_smooth + qfrc_constraint` (`:1141-1146`, `:1165-1188`), and passes it to `mj_advance` (`:1348`).
  - `mj_advance` saves **`d->qacc`** (the explicit one) as the next warmstart (`:939`).
- **Ours:**
  - `forward_acc` always runs `mj_fwd_acceleration` for Implicit/ImplicitFast (`forward/mod.rs:565-571`).
  - That function writes `(M − h·D)⁻¹(qfrc_smooth + qfrc_constraint)` into `data.qacc` (`forward/acceleration.rs:316-317` implicitfast, `:364-365` implicit) **before** the acceleration sensors run (`forward/mod.rs:581-582`).
  - `mj_body_accumulators` reads it (`acceleration.rs:587`).
  - `step`/`step2` save it as the warmstart (`forward/mod.rs:264`, `:204`).
  - `integrate` adds `qacc·h` (`integrate/mod.rs:156-165`).
- delay_history §8 F1 named this mechanism by reading. The prototype above isolates it by measurement.

### 2.3 A second effect of the same cause: the warmstart
For a constrained model under an implicit integrator, the warmstart is the implicit acceleration in ours and the explicit one in MuJoCo.
- Newton reached the same answer from either start on these fixtures.
- PGS did not: `conn_free2_implicitfast_PGS` with item 1 fixed is 1.3e-7 off; adding the explicit warmstart (`ISO_FIX=1` or `2`) brings it to 2.6e-15 (§1.4).
- In the corpus, `ISO_FIX=2` changed the trajectory bits of **4** deterministic docs, all implicit with constraint rows: the 3 cable-connect docs and `flex_unified.rs:242`.

### 2.4 Now → Target → Change
- **Target:** `data.qacc` after `forward()` is the explicit acceleration for every integrator, as in MuJoCo. That is the solver's result when Newton solved it, else `M⁻¹(qfrc_smooth + qfrc_constraint)`. The implicit acceleration feeds only the velocity update and the analytic derivatives. Hence MuJoCo's acceleration sensors, `cacc`, `cfrc_int`, `qfrc_inverse`, `solver_fwdinv` and warmstart.
- **Change** (recommended shape, option B of §6 Q2):
  ```rust
  // types/data.rs — crate-private, sized nv, zeroed by make_data and reset, cloned with Data
  /// The acceleration Implicit/ImplicitFast advance qvel with: (M − h·∂f/∂v)⁻¹ (qfrc_smooth + qfrc_constraint).
  /// MuJoCo keeps it in a stack local of mj_implicitSkip (engine_forward.c:1134).
  pub(crate) qacc_implicit: DVector<f64>,
  ```
  - `forward_acc`: run `mj_fwd_acceleration_explicit` when `!newton_solved`, for every integrator except ISD. Then, for Implicit/ImplicitFast, the existing implicit solve writes into `qacc_implicit` instead of `qacc` (`acceleration.rs:316-317`, `:364-365`). The Cholesky/LU failure still surfaces from `forward()`.
  - `integrate` (`integrate/mod.rs:156-165`) uses `qacc_implicit`.
  - The analytic derivatives must read `qacc_implicit` where they meant the integrator's acceleration. All three sites are needed; measured below.
    - `derivatives/integration.rs:140-142` and `:167-169` (ball/free post-step ω).
    - `derivatives/hybrid.rs:2513` (full-implicit Coriolis term `T = rne_vel(qacc)`).
    - `hybrid.rs:2745-2752` / `:2788` (`qacc_transition`, today `scratch.qacc` after a nominal step).
  - Comments: `forward/mod.rs:557-564`; the doc comments of `mj_fwd_acceleration_implicitfast` and `_implicit_full` (`acceleration.rs:266-279`, `:322-335`); the `qacc` field doc; `integrate/mod.rs:36-38`.
- **Derivatives with all three sites routed** (`ID/out/deriv_bits.txt`; `mjd_transition` hybrid at a state, 10 fixtures, implicit and implicitfast, with free and ball joints):
  - `A` and `B` are **bit-identical** to main.
  - `C` changes only where an accelerometer is present, because the sensor itself changes.
  - Before the routing, `integration/derivatives.rs:1615` (`test_pos_deriv_transition_implicit_fast`, 2.5e-2) and two sim-core harness tests (`transition_matrix_harness::analytic_transition_matches_fd_{under_stiffness,implicit_coriolis_under_damping}`, up to 4.0e-3) failed. That is how the 3 sites were found.

### 2.5 Unaffected models and flips
- **Corpus (`ISO_FIX=2`):** only Implicit/ImplicitFast docs change.
  - `qacc` fingerprint: 43 of 51 deterministic implicit docs. The 8 unchanged include the two integrator examples' docs; not examined further.
  - Trajectory: 4 (§2.3). Sensordata: 0 (no implicit corpus doc has an acceleration-stage sensor).
  - Euler and RK4: 0 changes.
- **Suites with the routing in place:** integration 1,338, conformance 83, sim-core lib 720, sim-mjcf 385 + 1: **no flips**. Validators `example-integrator-stress-test` and `example-composite-stress-test`: stdout identical.
- **Downstream readers of `data.qacc`, by grep:**
  - `sim/L0/ml-chassis/src/space.rs:78` (`Qacc` observation; for an implicit-integrator env it becomes the explicit acceleration);
  - `sim/L0/thermostat/tests/langevin_thermostat.rs:65`;
  - `tools/cf-mjcf-emit/tests/{forward_dynamics_gate.rs:198,215, coupled_knee_equality.rs:131}`;
  - `sim/L1/bevy/examples/diag_pendulum.rs:75`;
  - `sim/L0/mjcf/tests/forward_conformance.rs:215`.

  In none of them did I find an implicit integrator: the MJCF fixtures set none and cf-mjcf-emit emits none. Not run. `sim-gpu` has no implicit integrator (grep).

### 2.6 Tests to add (each fails on main)
1. `implicit_accelerometer_matches_mujoco_3_5_0`: the `golden_acc.json` damped-hinge case under implicitfast and implicit, undelayed accelerometer to 1e-12. Main is 1.77e-1 off.
2. `forward_qacc_is_explicit_under_implicit`: `acc_hinge_implicitfast.xml` at (0.3, 1.5). `qacc` must equal MuJoCo's −2.907859844926252 (1e-12) and must equal `qacc_smooth` (no constraints). Main gives −2.22209.
3. `implicit_warmstart_is_explicit`: `conn_free2_implicitfast_PGS.xml`, 500 steps against MuJoCo golden at 1e-12. It fails on main, and also with item 1 alone (1.3e-7).
4. Keep the three derivative tests above as the guard for the routing. They fail if any of the 3 sites is missed.

---

## 3. ledger-L39c: flex edge constraint rows

### 3.1 MuJoCo 3.5.0's rule
- **Edge rows exist only for a flex named by an `mjEQ_FLEX` equality.** It comes from either:
  - `<equality><flex flex="name"/>` (`xml_native_reader.cc:678`), or
  - `<flexcomp><edge equality="true"/>`, which creates that equality (`user_flexcomp.cc:643-649`, parsed at `xml_native_reader.cc:2769-2775`; map `false/true/vert` at `:933-937`).
- `flex_edgeequality` records it (`user_model.cc:3432-3446`).
- **One row per non-rigid edge** (`engine_core_constraint.c:616-643`; rigid edges skipped at `:620-624`).
- The rows are of type `mjCNSTR_EQUALITY` with `efc_id` = the equality's id. Their `solref` and `solimp` are the equality's (`getsolparam`, `:1301-1304`); flexcomp `<edge solref solimp>` sets them (`xml_native_reader.cc:2772-2773`).
- **`diagApprox = flexedge_invweight0[e]`** (`:1153-1165`), computed in `mj_setConst` (`engine_setconst.c:716-750`):
  - 0 for a rigid edge;
  - `(1/m₁ + 1/m₂)/2` when both vertex bodies are simple;
  - `J·M⁻¹·Jᵀ` otherwise.
- `equality="vert"` (`mjEQ_FLEXVERT`) gives 2 rows per vertex (`:647-670`).
- Edge stiffness and damping are passive forces, separate from these rows (`engine_passive.c:413-443`).

### 3.2 Ours
- **Rows:** every edge of every flex gets a `ConstraintType::FlexEdge` row. That covers counting (`constraint/assembly.rs:151`) and assembly (`constraint/equality_assembly.rs:77-136`), and rigid edges are included.
- **Row parameters:**
  - diagApprox is `MJ_MINVAL` (`:100`, `:134`; `impedance.rs:481`);
  - solref and solimp are the flex's **contact** `solref`/`solimp` (`equality_assembly.rs:90`, `:124`; `mjcf/src/builder/flex.rs:131`, `:148-149`; `compute_edge_solref` returns `flex.solref` at `:753-755`).
- **Parser:**
  - `<equality><flex>` is skipped silently: `_ => skip_element` at `mjcf/src/parser/equality.rs:62-63`, and `_ => {}` at `:98` for the self-closing form;
  - `<edge>` reads only stiffness and damping (`parser/deformable.rs:372-379`), so `equality`, `solref` and `solimp` are ignored;
  - sim-core has no vertex-equality machinery (`git grep -E 'flexvert_J|flexvert_length' -- sim/L0`: 0 files; control `flexedge_J`: 11).

### 3.3 Row counts, measured
Synthetic 3×3 dim-2 grids. MuJoCo reads the body-level `flexcomp`; ours reads the same `flexcomp` inside `<deformable>`, because a body-level `flexcomp` loads as **nflex = 0** in ours. Both give nv = 27 (18 with 3 pins), 16 edges and 2 rigid edges with pins (`ID/xml/flex{A..E}*.xml`, `ID/out` counts):

| doc | MuJoCo nefc (flex rows) | ours nefc |
|---|---|---|
| no equality | **0** | 16 |
| `<edge equality="true"/>` | 16 | 16 (attribute ignored) |
| equality + 3 pins | **14** (2 rigid skipped) | 16 |
| pins, no equality | **0** | 16 |
| `<equality><flex flex="cloth"/>` | 16 | 16 (element skipped) |

All 80 corpus docs with `<flex`/`<flexcomp>` are refused by MuJoCo as written: density ×71, unknown element ×5, unknown body ×2, mass ×2 (`ID/out/flex_count_mj.jsonl`). Grep finds **0** in-tree docs that request an edge equality: no `equality=` on any `<edge>`, no `<flex flex=`, no `<flexvert`. So under MuJoCo's rule none of the 75 flex docs that ours loads would have edge rows.

### 3.4 Dynamics on a matched fixture, and the prototype
The fixture is `ID/xml/flexG_ours.xml` and `flexG_mj.xml`. For the edge sets to match, MuJoCo's grid is rotated 90° (its triangulation runs the other diagonal) and shifted. Matched by rest position, the 16 edges and the rigid flags are identical sets. Comparison is by world vertex position (`ID/scripts/flexcmp.py`):

| ours | MuJoCo | max gap over 500 steps |
|---|---|---|
| main (16 rows) | edge equality, 3 pins (14 rows) | 3.5e-2 |
| `ISO_FLEX=1`: skip rigid edges, diagApprox per MuJoCo | same | **≤ 3.6e-16** |
| `ISO_FLEX=3`: rigid rows kept, diagApprox per MuJoCo | same | ≤ 5.4e-16 (rows 16 vs 14) |
| main | no equality (0 rows) | 4.89 |
| `ISO_FLEX=2`: no rows | same | **8.9e-16** |

So the dynamics gap comes from diagApprox. In ours an edge row is nearly hard: R ≈ 1e-15 against MuJoCo's 1.11 (`ID/out/flexB_*_f0.json`). The rigid rows change `nefc` but are inert here, because their J is 0.

Repeated runs of `ISO_FLEX=1` give 2.6e-16 and 3.6e-16. Our flex edge order varies from process to process (ledger-L27), so the 1e-16 level is not reproducible bit for bit.

### 3.5 Target and Change
- **Target:** the rule decides it. MuJoCo does what the file asks: no equality, no rows. So:
  - rows only for a flex named by an active flex equality;
  - one row per non-rigid edge;
  - the equality's `solref`/`solimp`;
  - diagApprox `flexedge_invweight0`.
- **sim-core** (breaking):
  ```rust
  pub enum EqualityType { /* … */ Flex }   // eq_obj1id = flex id
  pub flexedge_invweight0: Vec<f64>,       // Model; computed in Model::compute_invweight0 (types/model_init.rs:1010) / K6's recompute_derived
  pub flex_edgeequality: Vec<bool>,        // Model; gates mj_flex_edge's Jacobian skip (dynamics/flex.rs:37-41, today keyed on flex_edge_solref == [0,0])
  // removed: Model::flex_edge_solref, Model::flex_edge_solimp (only sim-mjcf writes them, only sim-core reads them)
  // ConstraintType::FlexEdge → see §6 Q3
  ```
  - `EqualityType` is not `#[non_exhaustive]` (`types/enums.rs:331-332`). Its downstream users are `cf-design` `model_builder.rs:244` (constructs it, no exhaustive `match` found by grep) and `cf-mjcf-emit`.
- **sim-mjcf:**
  - parse `<equality><flex flex=…/>`;
  - parse `<flexcomp><edge equality solref solimp>` (`true` creates the equality);
  - refuse `equality="vert"` and `<equality><flexvert>` as a stated limitation;
  - delete `compute_edge_solref`.
- **Docs:** each in-tree flex doc whose test means to hold edges needs `<equality><flex flex="…"/>`. That requires named flexes (§6 Q4).

### 3.6 Flips (measured with the switches, not with the MJCF change)
- **`ISO_FLEX=2`** (MuJoCo's rule applied to today's docs, none of which request edges) fails 3 integration tests:
  - `flex_unified::ac5_edge_constraint_stiffness` (`:514`, "frc_x=0");
  - `flex_unified::ac20_bending_stability_clamp` (`:1134`, "vertex exploded");
  - `flex_flex_collision::t09_full_forward_step_no_panic` (`:517`, "must produce non-zero qfrc_constraint").

  Corpus: 28 deterministic docs change (`flex_unified.rs` ×24, `parser/tests.rs:2034/2054/2078/2102`), plus 47 docs that vary run to run and cannot be compared. Every other suite and the five validators are unchanged.
- **`ISO_FLEX=1`** (every flex treated as requesting edges, MuJoCo's row parameters; roughly the state after rewriting the docs): **0 test failures**. 26 deterministic corpus docs change trajectory bits; 5 change `nefc` (rigid edges).

### 3.7 Tests to add (each fails on main)
1. `flex_rows_only_with_edge_equality`: a 3×3 grid with no equality has `nefc == 0`; with an equality and 3 pins, `nefc == 14`. Main gives 16 and 16.
2. `flex_edge_rows_match_mujoco_3_5_0`: the pinned grid, 500 steps, vertex positions vs MuJoCo golden at 1e-12. Main is 3.5e-2 off. As a fixture this needs either our flexcomp generator to match MuJoCo's (§5: spacing, pos, vertex order, diagonal) or an explicit vertex/element `<flex>` (A4-Q1).

---

## 4. Commit list and dependencies

| # | commit | contents | depends / interacts with RIGID_SPEC §7 |
|---|---|---|---|
| D1 | `fix(sim-core): a connect's and a weld's impedance come from the norm of their violation (MuJoCo getposdim)` | §1.5; tests §1.8 #1–2; impedance doc comments | None. It changes multi-body equality trajectories, so it belongs in the core series, **before** the MJCF harness baseline (§7: "taken at the head of the core series"). M14 (connect `site1`/`site2`) inherits the rule unchanged. |
| D2 | `fix(sim-core): implicit integrators keep the explicit qacc; the implicit solve feeds the integrator (MuJoCo mj_implicitSkip)` | §2.4: field, `forward_acc`, `integrate`, the 3 derivative sites, comments; tests §2.6 | **K3** edits the same three places (`forward/mod.rs:565-571`, `acceleration.rs`, `integrate/mod.rs:155-182`) and the same `hybrid.rs` comments (`:2742-2745`, `:2780-2786`). Land after K3 and document both forward→integrate carriers together (`scratch_v_new` for ISD, `qacc_implicit` here). **K1:** option B keeps `integrate()` returning `()`, as K1 assumes; option A would not (§6 Q2). **K2/K9:** `C` changes for acceleration sensors under implicit, so their sensor-derivative goldens must be generated after D2. **Delay (M25 / DH-*):** delay_history excluded the accelerometer from its implicitfast goldens because of this gap; after D2 they can include it. D2 must precede M25, or M25's golden must exclude it. **Sleep re-forward fix:** if that work builds an `mj_advance(qacc)` helper (MuJoCo `engine_forward.c:833`), `qacc_implicit` becomes its argument for implicit integrators. **K7** also edits `forward_acc` (textual only). |
| D3 | `feat: flex edge constraints only through a flex equality, as MuJoCo` | §3.5 sim-core + sim-mjcf + doc rewrites; tests §3.7 | After **M3** (flex edge order, ledger-L27), so the 47 run-to-run flex docs become comparable. With **K6** (`flexedge_invweight0` is a derived field). Coordinate with **F1–F3** (flex_composite) and **A4-Q1** (vertex=/element= rewrites of the same 81 docs), and before **M15** (otherwise the schema pre-pass first refuses `<edge equality>` / `<equality><flex>` as MuJoCo-valid but unimplemented). |

D1 and D2 are independent of each other; both are needed for the cable docs to reach ≤ 1.9e-14 with an iterative solver.

## 5. Handed to other areas (measured unless marked)
| finding | referent | owner |
|---|---|---|
| a one-value `solref="0.002"` becomes the default (0.02); MuJoCo reads (0.002, 1) | `ID/out/cable_connect_stab_*_f0.json` | parser (mjcf-S2, attribute lengths) |
| weld `torquescale` not implemented | git grep 0 files (control 14); MuJoCo `engine_core_constraint.c:482-530` | parser / decision 19 |
| a body-level `<flexcomp>` loads as nothing (nflex = 0); MuJoCo builds it | `flexA.xml` count | parser (A4-Q3, M15) |
| `<deformable><flexcomp spacing="0.1 0.1 0.1">` gives spacing 0.02 (the value is parsed as one f64, `parser/deformable.rs:392-394`); `pos` is not applied (`:497-510` apply only scale and quat) | `flexB_*_f0.json` body_pos | parser |
| our grid generator orders vertices x-fastest and splits cells along the other diagonal from MuJoCo's | `flexG` edge-set check | flex |
| CG differs from MuJoCo after one step; most of it is where each engine stops iterating; not isolated | §1.9 | core / solver |

## 6. Open questions
1. **D1, D2, D3 in Rigid?** The ledger already places L38a in Rigid with the delay work, and L39 in Rigid with "(a) must be isolated or ledgered".
   - **Recommendation:** D1 and D2 in Rigid. Each is one contained commit with 0 test flips measured. D3 in Rigid only if the flex doc rewrites (F2/F3, A4-Q1) land too, because it rewrites the same docs; otherwise put it in the 0.10 ledger as DONE-or-DROPPED.
   - If wrong: D3 alone flips 3 tests and 28+ corpus docs unless the rewrites ride along.
2. **D2's shape.**
   - **A:** move the implicit solve into `integrate()` (MuJoCo's place). Then `integrate()` must return `Result` (CholeskyFailed), contradicting K1, and the hybrid derivatives must factor `M − h·D` themselves.
   - **B:** keep the solve in the acceleration stage, writing a crate-private `qacc_implicit` (prototyped).
   - **Recommendation: B.** It is measured: A and B are bit-identical to main, and 0 tests flip.
   - If wrong: a caller that edits forces between `forward()` and `integrate()` gets the stale implicit acceleration under B, as today; under A it would not.
3. **`ConstraintType::FlexEdge`.**
   - **Option 1:** keep it as our label for flex-equality rows.
   - **Option 2:** use `Equality` with `efc_id` = the equality id, as MuJoCo.
   - **Recommendation: option 2.** It is MuJoCo's shape, the change is breaking anyway, and the users are only sim-core and 2 tests (`flex_unified.rs:2364`, `phase13_diagnostic.rs:83`).
   - If wrong: the solver behaves identically (measured: the same trajectories with the `FlexEdge` label); only the API differs.
4. **Rewriting the in-tree flex docs.**
   - **Option 1:** add `<equality><flex flex=…/>` (named flexes) wherever a test means edges to hold.
   - **Option 2:** let them lose their rows.
   - **Recommendation: option 1** for `ac5`, `ac20`, `t09` and the docs whose assertions need edges. Which of the 75 those are was not determined; only 3 tests fail without rows.
   - If wrong: under option 2, tests like `ac20` ("vertex exploded") show that our bending without edge rows is unstable on that fixture.
5. **`equality="vert"` / `flexvert`.** Recommend refusing it as a stated limitation: no flexvert machinery exists (grep). 0 in-tree uses.

## 7. What this method cannot see
- **Not run:**
  - sim-gpu (T15c is not affected by reading: assemble.wgsl has no equality rows, and sim-gpu has no implicit integrator);
  - sim-urdf and sim-thermostat test suites (only built, as dependencies);
  - cf-design and cf-design-tests (they build `connect` linkages: `mechanism/model_builder.rs:244`, `mjcf.rs:486-501`);
  - cf-codesign, sim-rl, sim-opt, ml-chassis, L1 crates, the Bevy demos, doctests, `grade`, clippy on the prototype, licensed gates.
- **Validators:** only five were run, as copies (equality, mocap, sleep-wake, integrators and composites stress-tests).
- **Not covered:** runtime-generated MJCF (`format!` templates), and the 48 docs that vary run to run.
- **The prototype is env-switched scratch code**, not the commit shape. Per-commit greenness in §7's order was not run.
- **Item 3's MJCF side** (parsing `<equality><flex>` and `<edge equality>`) was not prototyped. Its row count and dynamics were measured with the sim-core switches only.
- **Items 1 and 2 on contacts** (`contact_assembly.rs` impedance handling) were not examined.

**Repo state.** Before: HEAD `139f7b25`, `git status --short` empty. After: HEAD `139f7b25`, `git status --short` empty. `MUJOCO_LOG.TXT` in the repo predates this session (19:22). All my MuJoCo runs used `ID/` as the working directory.
