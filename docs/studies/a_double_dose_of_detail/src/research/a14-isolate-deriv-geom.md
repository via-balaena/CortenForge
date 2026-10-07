> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# ledger-L36a, ledger-L39b, ledger-L36b: position derivatives, `fromto` z direction, ball/free `xaxis`

Researcher section, 2026-10-06. Repo source = `main` @ 3520544e (checkout `fix/rigid-sim-core-mjcf` @ 139f7b25; `git diff --name-only 3520544e HEAD` lists only `sim/docs/todo/spec_fleshouts/rigid_0_10/*.md`). Every number comes from a script or binary in `SCRATCH/rigid_spec/isolate_derivgeom/` (`DG/` below): sources in `scripts/`, `probe/src/main.rs`, `harness/src/main.rs`; outputs in `out/`. MuJoCo oracle: `SCRATCH/rigid_oracle_350/venv` (`mujoco.__version__` printed `3.5.0`); source citations: `SCRATCH/mj350src/mujoco` (tag 3.5.0, 881544c).

**Variants.** Three builds are compared throughout:
- `orig`: the repo crates.
- `l32`: scratch copies (`DG/core_l32`, `DG/mjcf_l32`) with the L32 fix applied (`SCRATCH/rigid_spec/l32_multijoint/out/fix.diff`). After patching, `diff -r` against the L32 researcher's `core_fix/src` was empty.
- `fix`: `l32` plus the three prototypes below (`DG/core_fix`, `DG/mjcf_fix`). The per-item diffs are taken against `l32`: `out/l36a_deriv.diff` (+118 / −12), `out/l36b_xaxis.diff` (+15 / −2), `out/l39b_fromto.diff` (+42 / −20).

---

## 1. ledger-L36a: analytic position derivatives vs finite differences

### 1.1 The fixture is the subject

There is no analytic position derivative in MuJoCo 3.5.0 to compare against:
- `engine_derivative.c` defines only velocity derivatives (`mjd_comVel_vel_dense` :321, `mjd_rne_vel_dense` :385, `mjd_rne_vel` :596, `mjd_smooth_vel` :1792).
- Position columns come from finite differences only (`engine_derivative_fd.c`: `mjd_stepFD` :295, `mjd_transitionFD` :542).

So the reference is FD. It is valid only if our FD equals MuJoCo's FD, so I checked that first.

`scripts/mj_trans.py` runs MuJoCo's `mjd_transitionFD` (eps 1e-6, centered). The probe's `trans` mode runs ours. Both use the same XML, qpos and qvel. With `l32`, our FD `A` equals MuJoCo's to ≤ 5.6e-11 (velocity columns ≤ 1.1e-10) on every fixture without a ball (`out/trans_base.txt`, `out/trans_fix.txt`, `out/trans_extra.txt`).

The factory `Model::multi_joint_body()` cannot be loaded by MuJoCo. I transcribed it into `xml/F2_mjb.xml` and checked the transcription against the factory: our hybrid `A` matrices for the two agree to ≤ 8.3e-17, and the FD `A` matrices to 5.6e-11.

### 1.2 Fixtures and the first differing quantity

`probe rnepos` runs `mjd_smooth_pos` (which is `−∂(M·a + qfrc_bias)/∂q` here: no springs, no actuators). It compares the result with a centered FD of `qM(q)·a + qfrc_bias(q,v)` at fixed `a = qacc`. It does the same for the intermediates `deriv_Dcvel_pos` against FD of `cvel`, `deriv_Dcacc_pos` (Part B) against FD of `cacc_bias`, and `deriv_Dcfrc_pos` (Part B) against FD of the accumulated `cfrc_bias`.

Results (`out/rnepos_base.txt`, `out/rnepos_fix1.txt`; worst body):

| fixture | pattern | total `l32` | `Dcvel` `l32` | total `fix` |
|---|---|---|---|---|
| F1 `hinge_slide` (the L32 researcher's) | root body: hinge, then slide | 3.25 | **0.88** | 1.2e-9 |
| F2 `multi_joint_body` (factory and XML) | root body: offset hinge → slide → offset hinge | 1.56 | **0.62** | 6.9e-10 / 1.9e-9 |
| F4 | root body: hinge, then a hinge with `pos="0 0 .3"` | 0.318 | **0.11** | 3.7e-10 |
| F5 | **one joint per body**: a child hinge with `pos≠0` under a moving parent | 0.219 | **0.030** | 1.2e-9 |
| F6 | chain; child body: hinge, then slide | 2.28 | **0.39** | 1.1e-9 |
| F3 (control) | F1 split across two bodies | 8.0e-10 | 2.7e-11 | 8.0e-10 |
| F7 (control) | root body: slide, then hinge | 2.1e-10 | 0 | 2.1e-10 |

The first differing quantity is `deriv_Dcvel_pos` of the affected body, in one column each:

- **F1, slide column.** Analytic 0; FD `(−0.47943, 0, −0.87758)` in the linear rows. This equals `ω_hinge × ∂xpos/∂q_slide = (0,1,0)·1.0 × (0.87758, 0, −0.47943)`, where `∂xpos/∂q_slide` is the FD of `xpos` from the same run.
- **F5, the child's own hinge column.** FD − analytic = `(0, 0.03041, 0.00941)`. This equals `ω_parent × ∂xpos_child/∂q_own = (1,0,0) × (−0.20368, 0.00941, −0.03041)`.

### 1.3 Cause

In `mjd_rne_pos`, every body-b quantity is expressed at the body origin `xpos[b]`. A body's **own** DOF k moves that origin by `dof_lin[k] = ∂xpos[b]/∂q_k`:
- a slide: its axis;
- a hinge: `â×(xpos−anchor)`, nonzero when the anchor is off the origin;
- zero for ball and free-angular DOFs: their subspace is `[R; 0]` at the origin (`joint_visitor.rs:175`).

The code models only rotation by own DOFs, and moving the origin only for zero-axis DOFs. Two families of terms are missing (`sim/L0/core/src/derivatives/hybrid.rs` @ 3520544e):

- **A. Earlier joints on the same body.** A DOF k does not rotate the joints applied before it, but it moves the origin they are referenced at. So `∂S_j/∂q_k = [0; ω(S_j) × dof_lin[k]]` for j < k. The code says the opposite:
  - `:1436-1440`: "only affects the subspaces of joints applied at-or-after k … NOT the earlier ones". The same rule is applied at `:1441-1456` (Dcvel), `:1576-1595` (Part A, Term D), `:1961-1966` (Part B, `vt ×_m ∂v_J`), and the same-body cross-term block that K13 adds (`fix.diff`, `dv(i)` = 0 for `i < first`).
  - The projection derivatives skip later same-body DOFs outright: `:1775-1777` and `:2197-2199` (`if anc == bid && kid > jid { continue; }`).
  - All of these sites also skip zero-axis DOFs, so a slide never contributes.
- **B. Quantities not attached to the body.** The transported parent velocity `vt`, the parent's transported acceleration, and the force lever back to the parent all move with the origin. The code adds `ω_p × dof_lin[k]` (and `α_p ×`, and `dof_lin × f`) only where `is_translational(k)` holds. That predicate is `dof_axis = 0 && dof_lin ≠ 0` (`:1142-1143`; its premise, at `:1136-1141`, is that a hinge's lever "is a pure lever", so "only genuine prismatic DOFs contribute"). It is used at `:1385` (Dcvel), `:1541` (Part A forward), `:1727` (Part A backward), `:1874` (Part B forward) and `:2155` (Part B backward).
  - An offset-anchor hinge on a non-root body is therefore missing all five terms.
  - So is a hinge whose origin was moved by a later same-body slide (F6).

**Why the in-tree harness did not see it.** `transition_matrix_harness.rs` claims "every case in the matrix is machine-exact" (`:86-89`). Its cases come from `test_fixtures/conformance.rs`:
- `multi_joint_body` there is two hinges at `jnt_pos = 0` (`:304-318`), so neither family A nor B is present.
- `hinge_offset_pivot` is a **root** body (`:147-163`): `ω_p = 0`, so family B vanishes.
- No case has a slide after a rotating joint.

### 1.4 Fix and measured agreement

Prototype: `out/l36a_deriv.diff`, applied on top of the L32 fix, one file.

- `is_translational` becomes `dof_lin[k] ≠ 0` (renamed `moves_origin`). This fixes family B at all five sites. For slides and free-linear DOFs the predicate is unchanged; for ball and free-angular DOFs `dof_lin = 0`.
- A helper `shift_dv(v, k) = [0; ω(v) × dof_lin[k]]` is added for family A. It is applied:
  - in the Dcvel same-body block,
  - in Part A Term D (with the `S·qacc` prefix),
  - in Part B's `vt ×_m ∂v_J` block,
  - inside K13's cross-term `dv(i)` (for `i < first`; translational k is no longer skipped there),
  - in both projection derivatives, replacing the `continue` with the shift term.

Measured agreement:
- **rne level.** Every fixture is ≤ 1.9e-9 in total and ≤ 1.1e-10 in each intermediate (`out/rnepos_fix1.txt`).
- **Transition vs MuJoCo `mjd_transitionFD`** (`out/trans_fix.txt`, `out/trans_extra.txt`), hybrid position columns:

| fixture | `orig` | `l32` | `fix` |
|---|---|---|---|
| F1 | 6.78e-2 | 6.78e-2 | 7.9e-11 |
| F2 | 1.12e-1 | 1.12e-1 | 4.9e-11 |
| F4 | 3.73e-2 | 3.15e-2 | 2.4e-11 |
| F5 | 8.6e-4 | 8.6e-4 | 4.9e-11 |
| F6 | 4.08e-2 | 4.08e-2 | 3.8e-11 |
| F9: 3-level chain, every hinge offset, a leaf hinge→slide | 1.36e-1 | 1.36e-1 | 4.3e-11 |

- **Factory `Model::multi_joint_body()`, ours only.** Hybrid vs FD: 1.12e-1 (`l32`) → 5.6e-11 (`fix`).
- **The harness metric** (`max_relative_error(A_hy, A_fd, floor 1e-3)`, threshold 1e-5, `out/trans_rel.txt`): `l32` gives 1.0, 1.0, 0.27, 0.243 and 0.98 on F1, F2, F4, F5 and F6; `fix` gives ≤ 4.2e-8.
- **ImplicitFast and ImplicitSpringDamper** on F1, F5 and F9 (`out/trans_integrators.txt`): `l32` 8.6e-4 to 0.136, `fix` ≤ 2.1e-9. Full Implicit uses FD position columns (`hybrid.rs:2709-2722`) and is unaffected.
- **Two cases that already agreed** stay agreed:
  - F8, a child body with an offset hinge then a ball: hybrid vs FD 1.13e-3 → 4.0e-10.
  - F10, a free child of an offset hinge: 2.0e-10 → 1.6e-10.

**Bit-identity elsewhere** (`probe scan` with `SCAN_ALL=1`, `out/scan_all.tsv`). This compares the hybrid `A` of `l32` and `fix` at a perturbed state for every loadable corpus doc (1,456 of 1,584; 128 fail to load, step or differentiate in both variants).
- 1,401 docs are bit-identical.
- 1 doc differs because it has the pattern: `sim/L0/tests/integration/validation.rs:571`, a body with a hinge then a slide. It moves from 1.3e-3 off FD to 3.9e-11.
- 9 docs differ deterministically by ≤ 6.1e-10. All 9 contain `fromto` geoms; that is the L39b change in the same build (§2).
- 44 docs differ, but the same 44 also differ between **two loads of `l32`** in one process (`SCAN_CTRL`, `out/scan_ctrl.tsv`). One of them has `fromto` geoms. All 44 ids are in `determinism_verification/flex_docs.txt`, so this is the flex-ordering nondeterminism of ledger-L27, not this change.
- 1 flex doc (`flex_unified.rs:169`) differed by 5.0e-7 in 2 of 3 `l32`-vs-`fix` runs and was bit-identical in the third. In 3 `l32`-vs-`l32` runs it was identical. My change does not touch flex, so I count it with the class above; this was not proven further.

The five groups add up to 1,456.

### 1.5 Corpus reach and flips

Pattern count (XML parse, `scripts/count_joints.py`; the L32 researcher's per-body table `l32_multijoint/out/corpus_multi.tsv`):
- **0** joints with a nonzero `pos` in the 1,584 static docs.
- **1** body with a rotating joint before a slide (`validation.rs:571`).
- That test does not call derivatives (`git grep` for derivative calls lists only `derivatives.rs`, `implicit_integration.rs`, `param.rs`, `hybrid.rs`, `transition_matrix_harness.rs`, `sim/L1/coupling/src/articulated.rs` and `examples/fundamentals/sim-cpu/derivatives/*`).

Suites run, `l32` vs `fix`, each in a scratch copy (`out/suite_*.txt`, `*.names`). These runs include all three prototypes:

| suite | result |
|---|---|
| sim-core `--lib` | 720 / 720 in both; per-test outcomes identical |
| sim-mjcf lib + `tests/` | 385 + 1 + 1 (3 ignored) in both |
| sim-conformance-tests `integration` | 1338 pass / 27 ignored in both; per-test outcomes identical (1,448 names, `diff` empty) |
| sim-conformance-tests `mujoco_conformance` | 83 in both |
| validators `derivatives/stress-test`, `equality-constraints/stress-test`, `sensors-advanced/stress-test` (scratch copies, `out/ex_*`) | all checks pass in both; the derivatives validator's output is byte-identical |

No in-tree test pins the wrong values.

### 1.6 Tests to add (each measured to fail at the K13 parent state, `l32`)

1. **Harness cases**, `max_relative_error < 1e-5`:
   - `hinge_then_slide_root` (F1): `l32` 1.0
   - `offset_hinge_child` (F5): 0.243
   - `hinge_then_offset_hinge` (F4): 0.27
   - `chain3_offset` (F9): 1.05 (Euler)

   All four are ≤ 4.2e-8 after the fix (F9: 3.6e-8).
2. **sim-conformance-tests `derivatives.rs`**: MJCF hinge→slide (F1) and offset child hinge (F5). Hybrid vs FD position columns, absolute ≤ 1e-9: `l32` 6.8e-2 and 8.6e-4, `fix` 7.9e-11 and 4.2e-11.
3. **Factory `multi_joint_body`** at the probe state (qpos `(0.4, 0.15, −0.3)`, qvel `(0.8, −0.5, 0.6)`): `l32` 1.12e-1, `fix` 5.6e-11. This is the fixture GPU T15c uses.

### 1.7 Downstream

No signature changes. Values change for models with the pattern. On the corpus, every other doc is bit-identical or falls in one of the groups explained in §1.4. The affected functions:
- `mjd_smooth_pos`;
- `mjd_transition` / `Data::transition_derivatives` (analytic path: Euler, ImplicitFast, ISD);
- `mass_directional_derivative` (its Part A difference).

Consumers: `sim/L1/coupling/src/articulated.rs:424,546` (`transition_derivatives`) and `:611` (`mass_directional_derivative`). Coupling's in-tree MJCF is in the static corpus, which has no pattern doc other than `validation.rs:571`; coupling's own suites were not run. MuJoCo has nothing to match here, so there is no parity text.

---

## 2. ledger-L39b: geom `fromto` z direction

### 2.1 Fixture and the first differing quantity

`xml/G1_fromto.xml` has 40 geoms:
- types: capsule, cylinder, box, ellipsoid;
- directions: ±x, +y, ±z, three general, and two within 1e-9 of ±z;
- placed on a body with `euler="30 20 10"`.

Results (`scripts/cmp_geoms.py`, `out/geoms_G1.txt`, `out/geoms_G2.txt`):
- `geom_pos` and the capsule/cylinder `geom_size` agree (≤ 1.4e-17).
- **`geom_quat` differs for all 40.** `geom_xmat`'s z column equals **minus** MuJoCo's (≤ 4.4e-16), and the x and y columns differ by up to 1.6.
- Box and ellipsoid `geom_size` also differ. That is mjcf-S8 ("box/ellipsoid fromto", A4 §8.4, M14) and is not part of this item.

### 2.2 Cause

- **MuJoCo.** `mjCGeom::Compile`, `user_objects.cc:3693-3698`, sets `vec = from − to` and normalizes it. `:3715` calls `mjuu_z2quat(quat, vec)`. That function (`user_util.cc:395-408`) builds the shortest-arc rotation taking +z onto `vec`, with the x axis as fallback when `|z×vec| < 1e-10`. Sites use the same rule (`:3953`, `:3971`).
- **Ours.** `compute_fromto_pose`, `sim/L0/mjcf/src/builder/geom.rs:825`, sets `axis = to − from`. `:829-849` rotates z onto that axis, using a rotation of π about x when it is anti-parallel. So our +z points from→to and MuJoCo's points to→from.
- Cable composites inherit the rule: both engines set `fromto = (0,0,0, length,0,0)` (`builder/composite.rs:507-508`; MuJoCo `user_composite.cc:395-398`).

### 2.3 Fix and measured agreement

Prototype: `out/l39b_fromto.diff`. `compute_fromto_pose` calls a port of `mjuu_z2quat(from − to)`, including `mjuu_normvec`'s two thresholds (`user_util.cc:149-165`, mjEPS = 1e-14 at `user_util.h:29`).

- **G1:** `geom_quat` is **bit-equal** to MuJoCo's for **40/40** (main 0/40). `geom_xmat` is ≤ 5.6e-16.
- **Test literals.** For capsule `fromto="0.05 -0.02 0.1 0.05 -0.02 0.5"` (+z):
  - MuJoCo gives `[6.123233995736766e-17, 1, 0, 0]`
  - main gives `[1, 0, 0, 0]`
  - `fix` is equal to MuJoCo.

  For the general direction `capsule_g1`:
  - MuJoCo gives `[0.9445804731606344, 0.10381123721622432, −0.31143371164867284, 0]`
  - main gives `[0.3283, −0.2987, 0.8961, 0]`
  - `fix` is bit-equal to MuJoCo.
- **Cables.** These need M8's curve map and the flex/composite section's f32 vertices, which I added env-switched in `DG/cable_*` from `flex_composite/fix`. With them, capsule and cylinder `geom_quat` vs MuJoCo is **1.4 → 0.0** on the corpus doc `263f733b2df7d3fe` and on `xml/C3_cable_sine.xml` and `xml/C4_cable_line.xml` (`out/cable_quat.txt`).

### 2.4 What is observable

- **Geom frames and frame sensors.** `geom_quat`, `geom_xmat`, and frame sensors with `objtype="geom"` change. For `xml/D1_capsule_drop.xml` (framequat + framezaxis), main is 1.85 off MuJoCo after one step and `fix` is 2.2e-16 (`out/dyn_fromto.txt`).
- **Capsule and cylinder dynamics change only at rounding level.**
  - Body inertia is bit-identical across `orig` and `fix` (`xml/G3_fromto_capcyl.xml`, `out/geoms_G3_inertia.txt`).
  - D1 (capsule-plane), D2 (cylinder-plane), D3 (two parallel capsules) and D4 (a cylinder standing on its end, 3 contacts): `orig` vs `fix` is ≤ 1.1e-13 qvel after 500 steps, with the same contact counts. Distance to MuJoCo is unchanged by the fix: 1.2e-4, 7.4e-9, 2.3e-13 and 7.2e-14.
- **Boxes: the frame matters once sizes are right.** This was measured in MuJoCo alone (`scripts/mj_box_frame.py`, `out/mj_box_frame.txt`). The same square-section box was placed two ways, via `fromto` and via explicit pos/quat equal to *our* frame, with identical size and inertia. After 200 steps qpos differs by 0.405 (4 vs 2 contacts), and it comes to rest 0.244 apart.
  - Today our box `fromto` size is `(r, half, 0)` (M14), so the orientation also changes inertia (8.9e-4 in G1).
  - Ellipsoid sizes are `(r, r, h)` after M14, which is symmetric about z. Its dynamics under the flip were not measured with M14's sizes.
- **Corpus** (`harness`, 2 runs per variant, `out/corpus_cmp.txt`):
  - `fromto` usage is 315 capsule geoms and 1 cylinder in 183 docs. No box, ellipsoid or site uses `fromto` (`scripts/count_fromto.py`).
  - `geom_quat` changes in **200** docs (this count includes composites).
  - Trajectories change in **21** of them after 100 steps. In 20 the change is ≤ 8.4e-14. One is chaotic: `examples/fundamentals/sim-cpu/equality-constraints/stress-test/src/main.rs:388`, two free bodies joined by `connect` constraints. Its inertia differs by 1 ulp, and the gap grows from 1.4e-17 (1 step) to 2.8e-4 (100) and 7.8e-2 (1000).
  - That example is a **validator**, and its 18/18 checks still pass. Two printed lines change: `angvel` at t=2 s goes 0.067 → 0.016, and energy growth −3.42% → −3.43% (`out/ex_equality-constraints-stress-test_*.txt`).
  - Sensordata changes ≤ 6.7e-18 in 3 example docs (`sensors-advanced/stress-test/src/main.rs:54` and `:294`, `force-torque/src/main.rs:50`). That validator passes; one printed `-0.0000` flips sign.

### 2.5 Interactions and downstream

- **M14** (mjcf-S8, `compute_fromto_pose(fromto, size, geom_type)`) already rewrites this function for box and ellipsoid sizes. L39b belongs in M14, so that a box's size and frame match MuJoCo together.
- **M8** (curve keywords): cable frames match only with M8, L39b and f32 vertices together.
- **Frame expansion** transforms the endpoints before `compute_fromto_pose` (`builder/frame.rs:164-174`), so it is unaffected.
- **Inertia** goes through `resolve_geom_fromto` → `compute_fromto_pose` (`mass.rs:169`), so it follows automatically.
- No signature change.
- Consumers of `geom_quat`/`geom_xmat`: collision; frame sensors with `objtype="geom"`; sim-gpu (it reads `Model`, so GPU vs CPU stays consistent; not run); sim-bevy rendering of symmetric capsules (the Bevy examples use `fromto` capsules, e.g. `sim/L1/bevy/examples/2dof_arm.rs:58`; not run).

### 2.6 Tests to add (fail on main; measured)

- Capsule +z and general-direction `geom_quat` bits (the literals in §2.3).
- A box `fromto` frame equal to MuJoCo's, with M14's sizes.
- A `framequat objtype="geom"` sensor on D1 vs a MuJoCo 3.5.0 golden: main 1.85, `fix` 2.2e-16.

---

## 3. ledger-L36b: ball `xaxis` (and free `xaxis`, slide `xanchor`)

### 3.1 Now vs MuJoCo (measured, `out/jnts_main.txt`)

| joint | ours (`forward/position.rs`) | MuJoCo 3.5.0 |
|---|---|---|
| ball | `xaxis = 0`: never written (`:100-115`, comment `:112`; zero from `model_init.rs:543`) | `xaxis = R_before·jnt_axis` (`engine_core_smooth.c:115-116`, assigned `:154-156`); `jnt_axis` is forced to (0,0,1) for ball and free (`user_objects.cc:2946-2950`) |
| free | `xaxis = 0` (`:116-131`) | `xaxis = jnt_axis`, unrotated (`engine_core_smooth.c:75-77`) |
| slide | `xanchor = pos` (`:96`) | `xanchor = xpos + R·jnt_pos` (`:118-120`) |
| ball, `jnt_axis` in Model | MJCF `axis` kept: `axis="1 0 0"` → (1,0,0) (`builder/joint.rs:73-83`) | (0,0,1) |

Measured differences:
- X1 (tilted ball): xaxis 0 vs `(0.34202, −0.469846, 0.813798)`.
- X3 (free): 0 vs (0,0,1).
- X5 (slide with `pos`): xanchor differs by 0.22.
- X7: `jnt_axis` (1,0,0) vs (0,0,1).
- MuJoCo loads `type="ball"`/`"free"` with `axis="0 0 0"` (stored as (0,0,1)) and refuses hinge and slide with `axis too small` (`out/mj_zero_axis.txt`).

### 3.2 Readers

Every `xaxis` read in the repo (`git grep xaxis`, excluding `.md`) sits in a `Hinge` or `Slide` arm:
- `constraint/equality.rs:477,484,550`
- `constraint/jacobian.rs:94,101,274,281,387`
- `dynamics/rne.rs:90,97`
- `forward/velocity.rs:85,95,102`
- `jacobian/mod.rs:61,71,247,253`
- `joint_visitor.rs:154,170`
- `tendon/spatial.rs:381,387`

The pass-through sites (`constraint/mod.rs:103`, `rne.rs:377`, `acceleration.rs:139`, `actuation.rs:217,230`, `passive.rs:133,275`, `jacobian/mod.rs:411,516`, `tendon/spatial.rs:104-300`) reach those same arms.

`xanchor` reads are hinge-only, plus the free-body branch in `hybrid.rs` (`−axis×(xpos[parent]−xanchor[jid])`, ancestor `jid`), which multiplies by `dof_axis = 0` for a slide.

Outside sim-core:
- sim-gpu keeps its own `part_axis` (`fk.wgsl:184-246`); its tests read `data.xaxis` for hinge and slide only (`conformance_tests.rs:111,115`, `tests.rs:429`).
- No sensor reads `xaxis`. `framexaxis` reads `xmat`.

**Measured:** 613 corpus docs see `xaxis` change. By construction these are the docs with a ball or free joint: only those joints' `xaxis` changes, and the slide `xanchor` change shows up in 0 docs, since no static doc gives a slide a `pos`. In the 569 of them that have no `fromto` geom and are deterministic, qpos, qvel, act, time and sensordata after 100 steps are **bit-identical** (`out/corpus_cmp.txt`). The suites in §1.5 are identical. **What is observable is the public `Data.xaxis` / `Data.xanchor` field only**, plus `Model.jnt_axis` for a ball given an `axis`. 0 static docs give a ball or free joint an `axis` (the `jnt_axis` column of `out/corpus_cmp.txt` changed in 0 docs).

### 3.3 Fix (cheap) and agreement

Prototype: `out/l36b_xaxis.diff`.
- **Ball:** `xaxis = quat_before · jnt_axis`.
- **Free:** `xaxis = jnt_axis`.
- **Slide:** `xanchor = pos + quat·jnt_pos`.
- **Builder:** forces `jnt_axis = (0,0,1)` for ball and free.

After the fix (`out/jnts_fix.txt`): ball, free, hinge→ball and slide-anchor agree with MuJoCo to ≤ 1.1e-16. The ball `xanchor` is deliberately unchanged; see §4.1.

The `Data.xaxis` doc (`types/data.rs:113-116`, "zero for ball/free") and the comment at `position.rs:108-112` must change.

**Tests to add** (each fails on main; measured): X1 ball `xaxis` vs the MuJoCo literal; X3 free `xaxis == (0,0,1)`; X5 slide `xanchor`; X7 `jnt_axis == (0,0,1)`; `<joint type="ball" axis="0 0 0"/>` loads.

**Downstream:**
- Data field values only.
- `design/cf-design/src/mechanism/model_builder.rs:563` pushes its own `joint.axis()` for every joint type into a factory Model. A cf-design ball would get `xaxis = R·(its axis)`. Whether cf-design builds ball joints with an axis other than z: not checked.

---

## 4. Found alongside (not in the three items; measured, not fixed)

1. **Ball `pos` is ignored by our forward kinematics.**
   - MuJoCo rotates a ball about `xanchor = xpos + R·jnt_pos` (`engine_core_smooth.c:118-120,143-146`). Ours rotates about the body origin (`position.rs:100-115`), and the ball subspace is `[R; 0]` at the origin.
   - `xml/X2_ball_offset.xml`: xpos differs 6.2e-2 at a nonzero qpos; after 100 steps qpos differs 0.143 and qvel 19 (`out/dyn_ref_ballpos.txt`).
   - 0 static in-tree docs set a joint `pos` (`scripts/count_joints.py`).
   - Under the parity rule this is either "implement" or "refuse as a stated limitation". Neither is in this section.
   - Changing only the ball `xanchor` field would make it disagree with our own rotation centre, which the free-child branch in `hybrid.rs` reads. So I left it.
2. **`ref` (`qpos0`) is not subtracted in forward kinematics.**
   - MuJoCo uses `qpos − qpos0` for hinge and slide (`engine_core_smooth.c:125`, `:137`). Ours uses `qpos` (`position.rs:62`, `:93`). The sleep code has its own forward-kinematics copy (`island/sleep.rs:225-260`), not checked for this.
   - `xml/X6_hinge_ref.xml` (`ref="30"`): `xquat` differs 0.26 at qpos0; after 100 steps qpos differs 0.246 (`out/jnts_main.txt`, `out/dyn_ref_ballpos.txt`).
   - **14 hinges and 1 slide in 10 static docs** use `ref`: `tendon_springlength.rs:83,112,141`, `joint-limits/stress-test/src/main.rs:28,71,115,194`, `joint-limits/hinge-limits/src/main.rs:46`, `tendons/spatial-path/src/main.rs:41`, `tendons/tendon-limits/src/main.rs:44`.
   - This lines up with the lengthrange section's unexplained `muscle_qpos0_ref` gap (2.9e-2, `lengthrange.md:206`). Not re-run here.
3. **Ball tangent block of the transition.**
   - Our `A` position-row × position-column block for ball DOFs differs from MuJoCo's `mjd_transitionFD`: `ball_only` 9.9e-4, `hinge_ball` 4.8e-4, F8 2.8e-4 (`out/trans_ball_block.txt`).
   - The velocity rows agree (≤ 6.8e-10). The position-row × velocity-column block is ≤ 2.0e-6. Dynamics agree (F8: 3.6e-15 qpos after 100 steps, `out/dyn/F8_*`). F8 shows the same gap in `orig`, `l32` and `fix` (`out/trans_extra.txt`).
   - Cause not isolated.
4. **44 corpus docs (all in the flex list) give a different hybrid `A` on two loads in one process** (§1.4). This adds to ledger-L27's evidence: derivative outputs inherit the build nondeterminism.
5. **Our loader accepts a non-top-level free joint**; MuJoCo refuses it (F10, "free joint can only be used on top level"). This is mjcf-H3's territory (M16), noted only.

---

## 5. Commit list and dependencies

| # | commit | rows | depends | carries |
|---|---|---|---|---|
| after K13 | `fix(sim-core): analytic position derivatives follow the origin a body's own joints move` | ledger-L36a | K13 (it extends the cross-term block K13 adds) | the 4 harness cases + 2 MJCF derivative tests (§1.6); the comment rewrites at `hybrid.rs:1136-1141`, `:1436-1440`, `:1576-1578`, `:1773-1777`, `:2195-2199` |
| core, any position | `fix(sim-core): ball and free joints store MuJoCo's xaxis; a slide's xanchor includes its pos` | ledger-L36b (core) | — | X1/X3/X5 tests; the `data.rs:113-116` doc |
| M17 | add: ball and free `jnt_axis` forced to (0,0,1) before the "axis too small" check, so M17 must not refuse `ball axis="0 0 0"` | ledger-L36b (mjcf) | — | the X7 test and the zero-axis ball test |
| M14 | add: `compute_fromto_pose` orients with `mjuu_z2quat(from − to)` | ledger-L39b | — (cables: M8 + the flex/composite f32-vertex item) | §2.6 tests; harness expected-change list: `geom_quat` in 200 docs, trajectory in 21 (20 at ≤ 8.4e-14, plus `equality-constraints/stress-test:388`) |

The L36a commit could fold into K13. I recommend keeping it separate: family B also hits **single-joint** bodies (F5), which is a different defect from K13's.

## 6. Open questions

1. **Where the new harness cases go.** `dynamics_conformance_matrix()` is shared with sim-gpu's GPU-vs-CPU suite (`transition_matrix_harness.rs:30-38`). Options: (a) add them to the shared matrix; (b) add a CPU-only list in the harness.
   - I recommend (b). GPU forward kinematics and RNE for hinge→slide bodies and offset anchors were not run here, and K13 already leaves `rne.wgsl` open.
   - If that is wrong, the GPU suite simply lacks these four models until someone adds them; nothing goes red.
2. **Ball `pos ≠ 0` (§4.1).** Options: refuse (stated limitation) in Rigid, or implement. Implementing touches forward kinematics, the ball subspace, velocity, Jacobians, derivatives and `fk.wgsl`.
   - I recommend refusing in Rigid, as a ledger row to implement, because it has 0 static in-tree uses.
   - If that is wrong, a user's offset ball stops loading instead of silently mis-simulating. cf-design builds Model directly (`model_builder.rs` pushes `jnt_pos`) and bypasses an MJCF refusal: not checked.
3. **`ref`/`qpos0` (§4.2).** It is not my item, and it needs a ledger row and Jon's call on Rigid vs later. The parity rule says match MuJoCo. If it lands, 10 docs' trajectories change, including the `joint-limits` validators.

## 7. What this method cannot see

- **Suites not run:** sim-gpu (no adapter run), sim-urdf, sim-thermostat (built as dev-deps only), cf-design, cf-osim, sim/L1 (coupling included), the Bevy demos (including the 6 non-validator derivative demos), doctests, licensed gates, `--features mjb`, `--no-default-features`.
- **Runtime-generated MJCF** (the 157 `format!` templates, cf-design) is outside the corpus. So are the 33 docs nondeterministic run to run in either variant and the 106 that fail to load in both.
- **Box and ellipsoid `fromto` with M14's sizes in *our* engine.** The box effect was measured in MuJoCo only; the ellipsoid was not measured.
- **The L36a fix is checked against FD on 10 fixtures, 3 integrators and the corpus.** It is not derived for joint types the parity rules refuse (ball then slide, nested free). Sleep was never enabled.
- **Clippy** (repo lint set, `--all-targets -D warnings`) is clean on all four scratch crates. I made it fail once with a planted `f64 as f32` in `geom.rs`, then restored the file (`cmp` confirmed) and it was clean again. rustfmt `--check` is clean on the 4 changed files.

**Repo state.** Before: HEAD 139f7b25 on `fix/rigid-sim-core-mjcf`, `git status --short` empty (`out/repo_state_before.txt`). After: identical (re-checked at the end of the session). Scratch `target/` directories were deleted at the end.
