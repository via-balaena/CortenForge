> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# Parity census: sim-core/sim-mjcf vs MuJoCo 3.5.0 over the in-tree corpus

Researcher section, 2026-10-06. Scratch: `SCRATCH/rigid_spec/parity_census/` (written `PC/` below). Repo read-only. At start, `git status --short` was empty and HEAD was `139f7b25`. At the end, status was empty and HEAD was `e48267f8`. That commit, made by the coordinator at 21:39 before any build here, adds only `sim/docs/todo/spec_fleshouts/rigid_0_10/A7–A14*.md`: `git diff --stat 139f7b25 HEAD -- sim/L0 design mesh` is empty. So every census run used the code of `main` @ `3520544e`.

## 0. What was run

| step | how | output |
|---|---|---|
| corpus | 1,584 static docs. Ours loads 1,478 (`rigid_plan_tests/emb_1.jsonl`), MuJoCo 3.5.0 loads 1,226 (`rigid_oracle_350/corpus350.jsonl`), **both load 1,214**. 11 of the 71 hash-order-nondeterministic docs (ledger-L27, mask from `determinism_verification/scripts/varying.py` over `emb_{1..10}`) are among the 1,214; they are run and marked `nondet` | `PC/docs_both.txt`, `PC/nondet71.txt` |
| our side | Scratch Cargo project with path deps on the repo crates (`PC/harness`). One more variant with feature `fix` builds against scratch copies (`PC/core_fix`, `PC/mjcf_fix`; with L42 also `PC/core_fix_b`, `PC/geom_fix_b`, `PC/harness_b`) | `PC/out/<variant>.jsonl` |
| MuJoCo side | `PC/scripts/mj_census.py` under `rigid_oracle_350/venv` (mujoco 3.5.0) | `PC/out/mj.jsonl` |
| batching | ≤ 100 docs per process. Each batch ran under `timeout 700` and an RSS watchdog (2.5 GB, 100 ms poll, `PC/scripts/run_side.sh`). No batch was killed (`*.batchlog`) | |
| compare | `PC/scripts/compare.py`, then `classify.py`, `pass2.py`, `clusters.py`, `labels.py` (`PC/scripts/pipeline.sh` chains them) | `PC/out/cmp_*`, `cls_*`, `clu_*`, `labels_table.txt` |

**Initial state** (both sides): qpos = qpos0; qvel_i = 0.1·(1 + i mod 3) ("e1"); ctrl = lo + 0.625·(hi − lo) for a limited actuator, else 0; act = 0.

**Per doc**:
- `forward` on a clone at t = 0;
- 100 `step`s on a separate Data, recording qpos/qvel/act/time after every step;
- `forward` on a clone after step 1 and after step 100.

The forward dumps run on clones, so they cannot touch the trajectory.

**Order of quantities.** The first differing quantity is taken in MuJoCo's pipeline order (`engine_forward.c:1370-1408`, `mj_fwdPosition` `:166` makeM, `:170/:196` collision, `:204` makeConstraint):
- xpos/xquat, qM (full if nv ≤ 120, else the diagonal);
- contacts with |dist| > 1e-12, as multisets keyed by the object pair: count, pairs, dist, pos, normal, tangent frame;
- nefc and efc_type counts (FlexEdge counted as Equality; MuJoCo's two friction types merged);
- qfrc_bias, qfrc_passive, qfrc_actuator, qacc_smooth, qfrc_constraint, qacc, sensordata;
- zero-distance contacts (`con_zero`), compared separately and placed last.

**Pass 2.** Some docs agree at t0 but their trajectory first differs at step k. For those, both sides re-run k−1 steps and `forward`. The first quantity that differs there, while the state still agrees, localizes the cause (`pass2_*.txt`).

**Metric.** max|ours − MuJoCo| / max(1, max|MuJoCo|) per quantity. A quantity "differs" when this exceeds **1e-9**.

**Why 1e-9.** The per-doc maximum over all continuous quantities at t0 (same-model docs, `PC/out/cmp_fixB_all.jsonl`) is:
- ≤ 1e-14 for 928 docs;
- 1e-13…1e-11 for 4;
- none in [1e-10, 1e-8);
- ≥ 1e-8 for 90.

The trajectory maximum over 100 steps has no doc in [1e-10, 1e-9). 1e-9 therefore sits in an empty decade in both. 14 docs with trajectory drift in [1e-13, 1e-10) count as **agree**.

### The comparison was made to lie first (`PC/lie/`)

All four lies used doc `32be1503…`, with the MuJoCo side perturbed:

| lie | flagged |
|---|---|
| `body_mass[1]·1.01` before the model dump | `model differs: body_mass` (9.9e-3) |
| same, after the dump | `t0: qM` (9.9e-3) |
| gravity_z − 0.1 after the dump | `t0: qfrc_bias` |
| geom size + 1e-3 after the dump | `t0: con_dist` (1e-3) |

**The first attempt failed.** A post-compile mass edit did not reach MuJoCo's `qM`, because MuJoCo caches `dof_M0` for simple dofs (`engine_core_smooth.c:1866-1867`). The lie needed `mj_setConst`.

**Default excitation unchanged.** After adding the second excitation, the default-excitation output was checked byte-identical on 40 docs, for both binaries.

### Model fields compared only where they act (each with its reader)

| field | compared where | reader / reason |
|---|---|---|
| `dynprm` | stateful actuators only | MuJoCo skips actuators with no act, `engine_forward.c:327-330` |
| `gainprm` | Fixed `[0]`, Affine `[0..3]` | `:417-424` |
| `biasprm` | None: nothing; Affine `[0..3]` | `:462-467` |
| weld `eq_data[10]` | MuJoCo's value vs 1 | ours has no torquescale: `constraint/equality.rs:110-116` reads `data[0..10]`. MuJoCo reads it at `engine_core_constraint.c:482` |
| `sensor_objid` | only where MuJoCo's is ≥ 0 | |
| joint range/margin/solref/solimp | limited joints only | |
| dof solref/solimp | dofs with frictionloss > 0 | |
| geom size | the components the type uses | |
| geom contact parameters | colliding geoms only | |
| `jnt_axis` | hinge/slide | |
| `jnt_pos` | not free | |
| `body_ipos` | massive bodies | |
| inertia | the tensor R·diag·Rᵀ | |
| quaternions | up to sign | |
| lengthrange/acc0 | muscle actuators | |
| ctrl/force range | as (−∞, ∞) when unlimited | |

Before these rules, 298 docs differed in the model. After them, 192.

---

## 1. Counts (1,214 docs both load)

| | main (`orig`) | all prototypes (`fixB_all`) |
|---|---|---|
| model differs (loader) | 192 | 192 |
| same model, **state** differs within 100 steps | 134 | 101 |
| same model, state agrees, an intermediate differs (**API only**) | 93 | 71 |
| agree | 795 | 850 |

The dynamics of the 192 model-differs docs were run as well, for information. On main:
- **147** of the 166 `fromto` docs agree;
- 14 diverge in state;
- 5 differ in API only (`PC/out/cmp_orig.jsonl`). With every prototype applied: 153, 10, 3.

## 2. Cluster table (main; label = the cause left once the prototypes are applied)

Examples are corpus doc id → source `file:line` (`corpus/manifest.jsonl`). The complete per-doc list is `PC/out/labels.json`.

### Known (259 docs, plus 55 absorbed by prototypes)

| n | label | where it shows | examples |
|---|---|---|---|
| 166 | ledger-L45b `fromto` (capsule z reversed, measured R·ẑ = −R'·ẑ) | model `geom_quat` | `004befaf` examples/…/tendons/fixed-coupling/src/main.rs:42; `01c95f37` sim/L1/bevy/examples/coupled_pendulums.rs:46 |
| 30 | ledger-L41, API only: MuJoCo lists dist = 0 contacts, ours drops them | `con_zero` | `0e9ce360` sleeping.rs:2498; `0ebba016` collision_primitives.rs:1580 |
| 25 | ledger-L44b, implicit `qacc` stored (API only; state already agrees) | t0 `qacc` | fluid_derivatives.rs:267, :215 |
| 24 | ledger-L44a, connect/weld impedance | step 2 `qfrc_constraint` | equality-constraints stress-test main.rs:343; phase7_spec_a.rs:336 |
| 20 | sleep rows (sleep_parity SP-*: the init-asleep forward, timing) | t0 `qfrc_bias`/`qfrc_passive`, step 10–11 | fluid_derivatives.rs:2050, :2725 |
| 14 | ledger-L41: rows for contacts with dist ≥ includemargin (dist 0, roundoff ±1e-16, and box–plane corners at dist **+0.35**) | t0 `nefc`/`ncon`, step 3/7 | newton_solver.rs:1395; contact-filtering stress-test main.rs:72 |
| 6 | plane-only static body gets mass 1.0 (mjcf_validation S7) | model `body_mass` | contact-tuning stress-test main.rs:452 |
| 5 | ledger-L32, multi-joint bias force | t0/step 2 `qfrc_bias` | validation.rs:571, :487 |
| 5 | model fields that follow other model differences (composite, mesh inertia, fromto, rotated inertia; 2 `nondet`) | model `meaninertia` | spatial-path main.rs:41; composite.rs:25 |
| 4 | ledger-L45, FK ignores `ref` | t0 `xquat`/`xpos` | tendon_springlength.rs:83, :141 |
| 2 | ledger-L44, CG | steps 31, 90 | cg_solver.rs:561; noslip.rs:201 |
| 2 | A3-O5, cylinder `bias` | | parser/tests.rs:1402 |
| 2 | A5-Q6, muscle actlimited | | actuator_phase5.rs:226 |
| 1 each | ledger-L42 (absorbed) · ledger-L43c · ledger-L34 (sensor delay) · A5-Q10 · free joint `limited` (A5 §3.4) · mjcf-S4 · A4 partial array (`polycoef="0.5"`) · sleep off under RK4 (sleep_parity Q2) · mesh re-centring (`user_mesh.cc:1676-1686`; dynamics agree) | | |

The model differences in **any** position (not only the first) also include:
- `actuator_lengthrange` in 8 docs (ledger-L37);
- `actlimited` in 9.

### NEW (98 docs; each is still there with every prototype applied)

| n | label | measured | examples |
|---|---|---|---|
| 28 (20 state) | **contact tangent frame** | normal and position equal, t1 differs. Ours `types/contact_types.rs:335-369`; MuJoCo `mju_makeFrame`, `engine_util_spatial.c:508-534`, plus the plane–capsule axis hint at `engine_collision_primitive.c:81,84`. **Probe** `ISO_FRAME=1` (MuJoCo's default rule, in `PC/core_fix_b`): 13 of 28 → agree, 0 regressions. 6 of the rest are plane–capsule (hint not probed). 9 move to a later quantity | collision_edge_cases.rs:698; contact-filtering stress-test main.rs:171 |
| 18 | degenerate geometry | 11 concentric sphere pairs (normal arbitrary); 2 crossing capsules; 1 parallel capsule–cylinder; sphere centre at a box centre (dist ours −0.05, MuJoCo −0.15); 3 sphere centres on a cylinder axis (ours emits **no** contact, MuJoCo one) | spatial_tendons.rs:1224; equality_constraints.rs:1293 |
| 9 | Newton solve | same contacts, `qfrc_constraint` 1e-7…1e-4 relative. With `iterations=1000 tolerance=1e-15` on both sides, 4 agree and the t0 gap of the other 5 shrinks 5–1600× (`PC/out/cmp_tight.jsonl`). What differs in the iterations is not isolated | collision_plane.rs:1076, :1317 |
| 8 (API) | accelerometer / framelinacc sensors | Under e1 (a free joint's v ∥ ω) 7 agree at t0 and differ after 100 steps; 1 differs at t0. Under excitation e2 (qvel_i = 0.1·(1 + i mod 5)·(−1)^i) sensors differ at t0 in 12 docs: accelerometer ×9, of which 2 agreed under e1; framelinacc ×1; force ×1, which agreed under e1; framequat ×1. Both sides add ω×v (`engine_core_util.c:836-838`, `dynamics/spatial.rs:325-329`). Not isolated | body_accumulators.rs:370; sensors-advanced stress-test main.rs:81 |
| 8 | contact position offset by \|dist\| along the normal | box–sphere ×4, cylinder–sphere ×4. Same symptom as L42's box–capsule (`pair_cylinder.rs:349`), which the L42 prototype fixes only for that pair | collision_edge_cases.rs:251; collision_primitives.rs:978 |
| 6 | late onset (steps 36–88) | elliptic cone / noslip / PGS: box contacts appear one step apart after a drift below 1e-9 (pass 2: state diff ≤ 5.8e-10). Unchanged under the tight tolerance. Not isolated | noslip.rs:695; collision_primitives.rs:802 |
| 5 (API) | equality rows between bodies with no dofs | ours 3/6 rows, MuJoCo 0 | builder/equality.rs:551, :480 |
| 2 | contact count, non-degenerate | parallel capsule–capsule: MuJoCo 2, ours 1. Box–capsule: ours 2, MuJoCo 1, after the L42 fix | collision_primitives.rs:473, :1363 |
| 2 | `<pair solreffriction>` with an elliptic cone | Contacts are equal and `qfrc_constraint` differs. For `0.1 0`, MuJoCo warns "solreffriction values should have the same sign, replacing with default" when stepped (measured). `0.08 0.6` gets no warning and differs too | unified_solvers.rs:1104, :1166 |
| 2 | RK4 + mocap | t0 and the forward at onset agree, the step differs (weld: step 1; contact: step 43) | equality_constraints.rs:1232; mocap-bodies/tilt-drop main.rs:39 |
| 2 | GJK/EPA capsule–cylinder | normal differs 3.5e-6 / 9.5e-3, dist 9.8e-8 / 1.0e-5 (the second is a deep, parallel case) | collision_primitives.rs:1091; collision_performance.rs:292 |
| 2 (model) | `discardvisual` | MuJoCo keeps the discarded geom's mass (11.43 vs 4.19) | builder/compiler.rs:254, :636 |
| 1 each | `framequat objtype="body"` (sensor_phase6.rs:220) · implicitfast + ellipsoid fluid differs after step 1 (fluid_derivatives.rs:224) · mesh–plane contact points (collision_plane.rs:1493, **nondet**) · weld `relpose` stored differently, quaternion not normalised (equality_constraints.rs:800) · ellipsoid `shellinertia` ~1 % (mesh_inertia_modes.rs:196) · `fusestatic` rotated child ipos, which side is right not established (builder/compiler.rs:586) | | |

**Out of scope (7).** Two or three hinges with the default axis on one body give a singular M. Both engines go NaN at step 1 (actuator_phase5.rs:739); only the post-NaN arrays differ.

## 3. What the prototypes absorb (main → variant, `PC/out/clu_*`)

| prototype | effect | regressions (agree → differ) |
|---|---|---|
| L32 (`l32_multijoint/out/fix.diff`) | 5 docs → agree | 0 |
| + L44a (`ISO_IMP=1`) | +24 docs → agree; 3 more move to an L41 label | 0 |
| + L44b (`ISO_FIX=2`) | 25 API docs → agree; 1 moves to "implicitfast + fluid" | 0 |
| + L44c (`ISO_FLEX=2`) | 0. **No flex doc is loaded by both engines** | 0 |
| + L42 (`isolate_fallthrough/out/fix.diff`) | 1 → agree; 2 move from position to frame | 0 |
| all | 55 docs → agree (795 → 850) | 0 |

## 4. Open questions (not settled by the fixed decisions)

1. **`fusestatic` with a rotated child.** Which ipos is right was not established: ours (1, 0, 0), MuJoCo (0, 1, 0). The parity rule says match unless MuJoCo is wrong; a test would decide.
2. **Degenerate pairs** (concentric spheres, crossing capsules). MuJoCo's choice is arbitrary but defined. Under the parity rule: match it. No alternative proposed.

## 5. What the census cannot see

- **The 370 docs either engine refuses.** That includes **every flex doc**, so ledger-L44c, L39 and L27-flex are invisible here.
- **Runtime-generated MJCF**: the 157 `format!` templates, cf-design, sim-urdf, and others.
- **Longer horizons.** Only 100 steps.
- **Other states and inputs.** Two excitations: e1 hides velocity-coupled sensor terms, and e2 moved 12 docs from agree to differ and 3 the other way. Not covered:
  - randomised states;
  - ctrl other than one constant (unlimited actuators get 0, which hides delay: 3 delay docs agree);
  - act ≠ 0;
  - xfrc/qfrc_applied (ledger-L40);
  - keyframes, callbacks, plugins.
- **Other entry points:** derivatives (L36/L45a), `step1`/`step2` (L38b), BatchSim.
- **Drift below 1e-9:** 14 docs.
- **A second cause behind the first.** The first differing quantity can mask one; 9 frame docs and 3 L44a docs revealed one only after a fix was applied.
- **Mechanisms.** Labels are rules derived from drill-downs. Only "frame" (13/28) and "Newton" (tight tolerance) were tested by an executable probe.
