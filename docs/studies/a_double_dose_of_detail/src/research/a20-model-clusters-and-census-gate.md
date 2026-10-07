> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Round 3 — the census's model clusters, flex, and the permanent census gate

Researcher section, 2026-10-06. Repo read-only throughout: `git status --short` empty and HEAD `fbc0ad54` at start and at end; the code is `main` @ `3520544e` (the branch adds only `.md` under `sim/docs/todo/spec_fleshouts/`). Scratch: `SCRATCH/rigid_spec/r3_model_and_gate/` — `model/` (written `R3M/`, Part 1A), `flex/` (Part 1B), `gate/` (Part 2), `scripts/`. MuJoCo oracle `SCRATCH/rigid_oracle_350/venv` (3.5.0); citations from `SCRATCH/mj350src/mujoco` (3.5.0, 881544c). Parts 1B and 2 were researched by two forks of this researcher in parallel; their text is kept, with their own referents. No `target/` directory is left; nothing is running.

## Summary

**Part 1A, model clusters (192 docs).** Every cluster was isolated to a first differing field with both sides cited, and every one but three (mesh re-centring, sleep under RK4, the cylinder-`bias` refusal) was prototyped (env-gated scratch copies; streaming-harness bit-identity per toggle).
- With every prototype on (the census's core prototypes + the model ones here + A9/A10/A11's, and A14's `fromto` rule through A10's port of it), the census goes from **795 → 1,026 agree** (e1) and **785 → 1,017** (e2), with **0 docs that agree on main ceasing to agree**. Of the 192: **171 agree**, 13 move to dynamics clusters, 8 stay model-different (§1A.1, §1A.4, §1A.5).
- Settled by measurement:
  - **plane-only mass** — MuJoCo's plane volume is 0;
  - **`discardvisual`** — MuJoCo's documented rule: inertia frozen with the visual geoms;
  - **ellipsoid shell** — match MuJoCo's ε-shell, recommended;
  - **weld `relpose`** — position kept, quaternion normalised;
  - **`meaninertia`** — purely derived, no fix of its own;
  - **FK `ref`** and **spatial spring length at `qpos_spring`** — two sim-core model-init causes behind the "derived" docs;
  - **muscle F0 kept at −1** — sim-core; also fixes `<general gaintype="muscle">`, measured off by sign on main.
- **`fusestatic`: MuJoCo is the wrong side.** Its fused model does not equal its unfused model (`qacc` off by 2.6–11.3 on four fixtures), against its own documentation. The cause, by reading and confirmed by the numbers: `AccumulateInertia` composes the frames in the wrong order. Main is wrong too, in three other ways (`fromto` geoms not moved, the child's `<inertial>` dropped, a parent's `<inertial>` never receives the child's mass). The prototype's fused model equals its unfused model to ≤ 1.1e-14 with an exact eigensolver, and to the eig3 inaccuracy otherwise.
- **Found outside the census** (measured; no corpus doc): `<inertial>` orientation alternatives silently ignored, and sim-urdf emits them; sim-urdf writes URDF `rpy` as intrinsic `xyz`; a spatial tendon over a sphere wrap reads `tendonvel` 0 at the initial forward.

**Part 1B, flex.** MuJoCo 3.5.0 loads 0 of the 80 flex docs; each first error hides another (`density` → `<vertex>` children → required `body`). The gap the appendices miss: MuJoCo's `<flex body=…>` names existing bodies as vertices, ours creates them. So A4-Q1's "same meaning, mechanical rewrite" does not hold. With our meaning written out as explicit bodies, at most 72 docs become comparable, and under e2 they still differ; one cause no appendix lists is `<elasticity damping>`, which means a different thing in each engine. New panic: flex + any `gravcomp` body → `rne.rs:357`.

**Part 2, the gate.** Prototyped in Rust in scratch against golden JSON from a pinned-3.5.0 generator; it reproduces the census class for 1,214/1,214 docs on main and on the census prototypes.
- Golden: e1 + e2; dumps at steps 0, 1, 100; 19 state checkpoints; 8.1 MB git pack.
- Corpus: committed, append-only, 742 KiB.
- Tolerance: 1e-9, re-justified per doc.
- Ratchet: per-doc class; improvements fail until blessed; regressions are never auto-blessed.
- Runtime: ~2 s in the release test shard 1.
- Floors measured: main **783**, census prototypes **837**, **every prototype here 1,012** (passes once blessed; main against that file fails with every flip listed).
- The gate also caught a commit-ordering hazard: the self-closing-`<body/>` fix (M9) without the connect-anchor refusal turns 5 both-refused docs into lenient loads.

## Open for Jon (only what the fixed decisions do not settle)

1. **Massless body's inertial frame** (§1A.3.1).
   - (a) Parity: copy MuJoCo's parent-frame pose into `ipos`/`iquat`.
   - (b) Keep the body frame, as today, and list it as a deviation, with a test.
   - Recommend (b).
   - If wrong: `xipos`/`ximat` of massless bodies differ from MuJoCo by the body's own offset. No census verdict changes either way (measured).
2. **`fusestatic` deviation** (§1A.3.5). Recommend keeping the documented invariant (fused == unfused) and listing MuJoCo's frame-order defect.
   - If wrong, i.e. parity with the defect: fused URDF/MJCF models get the wrong CoM/inertia exactly as MuJoCo does.
   - Ours has to be rewritten either way (main is wrong in three other ways).
3. **Ellipsoid `shellinertia`** (§1A.3.3). Recommend parity with MuJoCo's ε-shell.
   - If wrong: about 1 % of inertia on ellipsoid shells; 1 corpus doc; `t8` tightened either way.
4. **Mesh re-centring has no owner** (§1A.2, 3 docs).
   - MuJoCo moves the geom frame to the mesh CoM/principal frame.
   - A10's MH1–MH5 do not include it.
   - Options: implement it in A10's series, or list it as a representation divergence. Trajectories agree on all 3; `qM` differs in 2, from the 5e-8 mass.
5. **Flex: MuJoCo's `body` meaning or ours** (§1B.4). Without a decision, flex stays outside the census and the gate. Also open:
   - `node=` (same attribute, different meaning);
   - `<elasticity damping>` semantics;
   - an empty `element`.
6. **Gate** (Part 2):
   - 19 checkpoints (8.1 MB) or every step (28.3 MB);
   - pin classes, not labels;
   - append-only snapshot.

   Recommended: 19 / classes / append-only.

## Dependencies on other areas

- ledger-L45 FK (owner not in my brief). My `R3_FKREF` probe is only an isolation: derivatives and `sim-gpu` are not done. R3-P2 needs it.
- A10 MH2/MH3 (eig3, selection, orientation alternatives); A14/M14 (fromto); A9 M8 + C1 (curve, f32 cable vertices); A11 LR-a (lengthrange); A5-Q6, A5 M18; A4 M6/M9/M14 (connect anchor); A3-O5; A8 SP-9. Each was measured here to flip its census docs (§1A.2), except A8 and A3-O5.
- Gate (Part 2) is Rigid-physics' first commit; every later commit in both PRs re-blesses `verdicts.tsv`.

---

## Part 1A — the census's model-differs docs (192)

### 1A.0 Method, variants, and the fixture check

- **Fixture.** The 192 docs are those whose `model_diffs` is non-empty in `PC/out/cmp_orig.jsonl` (PC = `SCRATCH/rigid_spec/parity_census`). MuJoCo's side is the census's own `PC/out/mj.jsonl` (mujoco 3.5.0, e1 excitation) and `PC/out/mj_e2.jsonl` (e2). Our side is a copy of the census harness (`R3M/harness`, R3M = `SCRATCH/rigid_spec/r3_model_and_gate/model`), same `model_json`/`data_json`, same init.
- **Confirmed the copy is the subject.**
  - `orig` (repo crates) reproduces the census's per-field tally on the 192 exactly (`geom_quat` 171, `body_I` 14, `body_mass` 11, … `R3M/m_orig.jsonl`, `scripts/mcmp.py`).
  - The prototype build with **no** env toggle is byte-identical to `orig` in model JSON on 191/192 docs; the one exception, `9b0854df`, is in the 71-doc nondeterminism mask (ledger-L27).
  - The combined variant `all` with the census toggles `ISO_IMP=1 ISO_FIX=2 ISO_FLEX=2` reproduces the census's `fixB_all` classes exactly (850 / 192 / 101 / 71, 0 docs differ, `R3M/cls_coreB.jsonl`).
- **Prototypes.** Every change below is env-gated in scratch copies, so one build gives every variant:
  - `R3M/mjcf_fix` = repo sim-mjcf + A10's `MIH` patch (mjcf hunks only) + A9's composite hunks (`FC_CURVE`, `FC_F32VERT`) + mine (`R3_*`).
  - `R3M/mjcf_r3c` = `mjcf_fix` + A11's lengthrange port (mjcf part) + `R3_S4`, `R3_FREELIM`, `R3_PREFIX`, `R3_MUSCLEACT`; `R3M/core_r3` = census `core_fix_b` + A11 (core part) + `R3_FKREF`, `R3_SPRINGLEN`, `R3_F0`.
  - Full diffs: `R3M/out/proto_mjcf_all.diff` (vs repo), `R3M/out/proto_core_on_census.diff` (vs `core_fix_b`).
- **Bit-identity.** Streaming fingerprint harness only (`R3M/h2_{main,fix,all2}` = copies of `determinism_verification/harness/src/bin/harness2.rs`), 1,478 docs ours loads, 100 steps, mask = the 71 nondeterministic docs (two `main` runs vary in 59, all inside the mask). Each toggle alone against its own baseline (`scripts/h2diff.py`).
- **Agreement metric.** The census's (`compare.py`, 1e-9 relative, model fields only where they act).

### 1A.1 Result

| | e1 | e2 |
|---|---|---|
| census `main` (`orig`) | 795 agree / 192 model / 134 state / 93 API | 785 / 192 / 142 / 95 |
| census prototypes (`fixB_all`) | 850 / 192 / 101 / 71 | 841 / 192 / 109 / 72 |
| **+ every prototype here** (`cls_every3.jsonl`, env in `R3M/every3.env`) | **1,026 / 8 / 111 / 69** | **1,017 / 8 / 119 / 70** |
| docs agreeing on `main` that stop agreeing | 0 | 0 |

Of the 192 model-differs docs: **171 agree** in every quantity, **13 move to a dynamics cluster** (§1A.4), **8 still differ in the model** (§1A.5).

### 1A.2 Clusters, cause, owner, measured flip

"Flips" = the doc's model agrees with MuJoCo once the toggle is on (`R3M/m_*.jsonl`, `cls_every*.jsonl`). PR: **P** = Rigid-physics, **L** = Rigid-loading.

| cluster (n) | first differing field | cause (ours ↔ MuJoCo 3.5.0) | fix | PR | measured |
|---|---|---|---|---|---|
| `fromto` (166) | `geom_quat` | A14 / ledger-L45b | A14's `mjuu_z2quat(from−to)` (`MIH_FROMTO`) | L (M14) | 156 of 166 flip alone; the other 10 (7 muscle, 1 tendon length, 2 rotated frame) flip with the rows below |
| fromto under a rotated `<frame>` (2: `4eed75d1`, `eef83d84`, both `builder/frame.rs` tests) | `geom_quat` (0.71) | ours transforms the endpoints, then aligns z (`builder/frame.rs:164-174`); MuJoCo resolves fromto in the frame (`user_objects.cc:3685-3715`) and then composes the frame (`:3833`) — same axis, other twist | §1A.3.6 | L (M14) | 2/2 flip (`R3_FRAMEFROMTO`) |
| plane-only body mass (6) | `body_mass` 1.0 vs 0 | §1A.3.1 | §1A.3.1 | L | 6/6 flip (`R3_PLANE`) |
| `discardvisual` (2) | `body_mass` | §1A.3.2 | §1A.3.2 | L | 2/2 flip (`R3_DISCARD`) |
| ellipsoid `shellinertia` (1) | `body_I` 1.3 % | §1A.3.3 | §1A.3.3 | L | 1/1 flips (`R3_SHELL`) |
| weld `relpose` (1, `496f2186`) | `eq_data[3..6]` | §1A.3.4 | §1A.3.4 | L | model flips (`R3_RELPOSE`); the doc then differs from step 33 (§1A.4) |
| `fusestatic` (2: `68a35ea3`, `a6fb60bd`) | `body_ipos`, `body_I` | **MuJoCo is wrong**, §1A.3.5 | keep ours' right answer; fix ours' own three bugs | L | stays different by decision; trajectories agree on both (z hinge at the origin); `a6fb60bd`'s `qM` differs |
| derived `meaninertia` (5) | `meaninertia` | no own cause: mean of `qM` diagonal at `qpos0` in both (`engine_setconst.c:1056-1063`; ours `types/model_init.rs:1206-1231`). It follows: FK ignoring `ref` (`251c5904`), composite `curve` (`263f733b`), f32 mesh vertices (`7b6eff02`, `9b0854df`), fusestatic (`a6fb60bd`) | none of its own | — | flips with its cause in 2/5 (`251c5904` with `R3_FKREF`; `263f733b` with A9's `FC_CURVE`+`FC_F32VERT`); 3 stay with theirs |
| FK ignores `ref` (2 model docs here: `251c5904`, `9a9380fc`; plus 4 L45ref API docs and 4 fromto docs whose next quantity was `xquat`/`xpos`) | `tendon_length0`, `meaninertia` | ours rotates hinges by `qpos` (`forward/position.rs:62`), slides by `qpos` (`:93`); MuJoCo by `qpos − qpos0` (`engine_core_smooth.c:137`, `:125`). Isolated: with `ref` deleted from both docs, both models agree (`R3M/out_fix_ref.jsonl`) | ledger-L45 (the FK half) | P | `R3_FKREF`: `tendon_length0`, `meaninertia` agree; 10 docs flip in all (§1A.3.7) |
| spatial tendon spring length (same 2 docs) | `tendon_lengthspring` | ours resolves the `[-1,-1]` sentinel at `qpos0` (`tendon/mod.rs:69-74`); MuJoCo at `qpos_spring` (`engine_setconst.c:1069-1084`, `setSpring`). Fixed tendons already use `qpos_spring` (`builder/build.rs:665-680`) | §1A.3.7 | P | flips with `R3_FKREF`+`R3_SPRINGLEN` (2/2) |
| muscle (9: A5-Q6 ×2 + 7 `fromto` docs) | `actuator_actlimited`, `gainprm[2]`/`biasprm[2]`, `lengthrange` | three causes: forced `actlimited` (A5-Q6); `lengthrange` (A11); F0 stored instead of −1 (§1A.3.8; A11 open item 4, `A11_LENGTHRANGE.md:206`) | A5-Q6 + A11 LR-a + §1A.3.8 | L + P | 9/9 flip with `R3_MUSCLEACT`+A11+`R3_F0` |
| rotated geom inertia (1, `636fc5d5`) | `body_I` | ledger-L43c, A10 MH3 | A10 (`MIH_EIG3`) | L | flips |
| composite `curve` (1, `263f733b`) | `body_pos` … | A4 M8 + A9 C1 (f32 cable vertices) | A9's prototype | L | flips |
| partial `polycoef` (1, `5eb35113`) | `eq_data` | MuJoCo reads ≤ 5 values over the default `0 1 0 0 0` (`xml_native_reader.cc:2199`, `user_init.c:306`); ours zero-fills | A4 attribute reader (M6) | L | flips (`R3_PREFIX`) |
| self-closing `<body/>` (1, `94e0077d`) | `nbody` | mjcf-S4, A4 §8.2 | A4 M9 | L | flips (`R3_S4`, worldbody arm only) |
| free joint `limited` (1, `bc9d11e0`) | `jnt_limited` | A5 §3.4 | A5 M18 | L | flips (`R3_FREELIM`) |
| sleep under RK4 (1, `d22fcd34`) | `enableflags` | A8 SP-9 (`builder/build.rs:963-969`) | A8 | L (+P runtime) | not prototyped here |
| cylinder `bias` (2: `8e77cf7b`, `f755f979`) | `biasprm` | A3-O5 | refuse (stricter) | L | becomes a refusal; leaves the both-load set (example `actuators/cylinder/src/main.rs:46` needs rewriting) |
| mesh frame + f32 vertices (3: `cb2e58ac`, `7b6eff02`, `9b0854df`) | `geom_pos`, `geom_quat`, mass 5e-8 | MuJoCo re-centres a mesh at its CoM/principal frame and moves the geom frame (`user_mesh.cc:1676-1686`, `user_objects.cc:3749-3754`); embedded vertices are f32 in MuJoCo (A10 §2.3) | **no commit owns the re-centring** (A10's MH1–MH5 do not) | L | not prototyped; trajectories agree on all 3 (`cmp_every3.jsonl`: ≤ 4e-10); `qM` differs in `7b6eff02`, `9b0854df` (the 5e-8 mass) |

### 1A.3 Items (the ones no appendix owned)

#### 1A.3.1 Plane (and hfield) geom mass — sim-mjcf, Rigid-loading

- **Now.** `compute_geom_mass` gives a plane volume 0.001 (`builder/geom.rs:405`, `_ =>` arm), so mass = density × 0.001 = 1.0; `compute_geom_inertia` gives 0.001 diagonal (`:531`). An explicit `mass` is returned as is (`:360-362`). Hfield falls in the same arm.
- **Target.** MuJoCo 3.5.0: `GetVolume` returns 0 for a plane (`user_objects.cc:3115-3187`, default arm), hfield = box of the geom size (`:3175-3179`, size from the hfield asset `:3761-3764`); `SetInertia` zero for a plane (`:3404-3406`); an explicit `mass` on a geom whose volume ≤ mjEPS leaves `mass_ = 0` (`:3784-3790`); geoms with `mass_ ≤ mjEPS` are not selected (`:2166-2170`).
- **Measured** (`R3M/mj_massless.py`): a plane-only body and a plane with `mass="3"` both get mass 0, inertia 0 in MuJoCo.
- **Change.** `compute_geom_mass`/`compute_geom_inertia` (`builder/geom.rs`): plane → 0; hfield → box of `(sx, sy, 0.25·sz + 0.5·base)`; explicit mass with volume ≤ 1e-14 → 0. The geom selection (`mass > 1e-14`, group in `inertiagrouprange`) is already A10 MH3's.
- **Flips** (`h2_plane` vs `h2_main_1`): `model_fp` 8 docs (the 6 census docs + `unified_solvers.rs:1894`, `phase7_spec_a.rs:465`, both MuJoCo-refused), `traj_fp` 0. No hfield geom sits on a non-world body in the corpus (6 hfield docs, all world).
- **Test** (fails on main): `plane_geom_has_no_mass` — a tilted body with only a plane, and a plane with `mass="3"`: `body_mass == 0`, inertia 0 (main: 1.0 and 3.0).
- **Open (Jon): the massless body's inertial frame.** MuJoCo copies the body's *parent-frame* pose into the *body-frame* inertial pose when nothing gives an inertia (`user_objects.cc:2443-2446`). Measured: body at `pos="1 0 0" euler="0 30 0"` → `ipos (1,0,0)`, `xipos (1.866, 0, −0.5)`; `framepos objtype="body"` on a massless child reads `(1.854, 1.354, 1)` against its `xpos (1.5, 1, 1)` (`R3M/mj_massless.py`). Ours: `ipos 0`, `iquat` identity. The census cannot see it: with the copy on (`R3_MASSLESS_IPOS`) or off, every class is identical (`cls_every3` vs `cls_e3nm`); 32 docs change `model_fp`, `traj_fp` changes only in the 3 `mass="1e-20"` docs that A5 M19 refuses anyway (`fluid_forces.rs:119,185`, `fluid_derivatives.rs:1393`). It is observable through `xipos`/`ximat`, body-objtype frame sensors and `xfrc_applied` on a massless body. Options: (a) copy (parity); (b) keep ours, list it as a deviation ("MuJoCo puts the parent-frame pose in the body-frame slot; its own comment says 'copy body frame into inertial'"), with the test `framepos objtype="body"` == `objtype="xbody"` for a massless body. **Recommend (b).** If wrong: `xipos` of massless bodies differs from MuJoCo by the body's own offset; no dynamics differ in the corpus (measured above).

#### 1A.3.2 `discardvisual` keeps the discarded geoms' inertia — sim-mjcf, Rigid-loading

- **Now.** `apply_discardvisual` deletes `contype=conaffinity=0` geoms from the parsed tree before the builder computes inertia (`builder/compiler.rs:15-54`, `retain` at `:27-31`), so their mass is lost. Protected geoms: any sensor `objname` (`:19-24`).
- **Target.** MuJoCo computes the inertia with the visual geoms, freezes it as an explicit inertial, then deletes them (`user_model.cc:1987-2023`, after `bodies_[i]->Compile()` at `:4936-4938`); documented (`XMLreference.rst:815-817`: "an explicit inertial element is added to the body"). A geom is visual when `!contype && !conaffinity` (`user_objects.cc:3672`) unless a pair (`:5675-5676`), a tendon wrap (`:6396`), a sensor (`:7110`, `:7139`) or a tuple (`:7918`) references it.
- **Change.** The pre-pass marks instead of deleting (`MjcfGeom` gains a crate-private flag, or the pass returns a set); `process_body` computes inertia over all geoms and emits only unmarked ones (`builder/body.rs:258-262`, and the world loop `:33`); meshes used only by discarded geoms stay loaded for inertia and are dropped from the Model after it (MuJoCo deletes them, `user_model.cc:2022`). The protection list becomes MuJoCo's (pairs, tendon wraps, sensors, tuples).
- **Flips** (`h2_discard`): `model_fp` 2 (the census docs), `traj_fp` 0 (both bodies are static). Not measured: runtime MJCF — **sim-urdf emits visual geoms with `contype="0" conaffinity="0"` and `discardvisual="true"`** (`urdf/src/converter.rs:177`, `:300`, `:503`, `:532`). Measured (`R3M/xml/noinert.urdf`, a link with a visual box and no `<inertial>`): main 0.5236 kg = MuJoCo's URDF reader 0.5236 kg; with `R3_DISCARD` alone 16.52 kg, because the visual box now counts. MuJoCo's URDF reader creates no visual geom when `discardvisual` is set (`xml_urdf.cc:317-319`). ⇒ in the same commit sim-urdf stops emitting visual geoms (matching MuJoCo's URDF reader); check `urdf-loading/*` validators.
- **Test** (fails on main): the two census docs as literals — `5f4d4961`: mass 11.427019678657274, inertia 0.058447362588641846 (main 4.18879); `b6ad1797`: mass 8.377580409572783, ipos (0.5, 0, 0) (main 4.18879, 0).

#### 1A.3.3 Ellipsoid `shellinertia` — sim-mjcf, Rigid-loading

- **Now.** A 64×128 Gauss–Legendre integral of a uniform surface density (`builder/geom.rs:714-760`). Mass = density × Thomsen area in both engines.
- **Target.** MuJoCo: expanded ellipsoid (+1e-6 on each semi-axis) minus the solid one (`user_objects.cc:3323-3352`).
- **Measured.** Ours equals an independent 256/512-node uniform-density integral to ≤ 2e-15 (`R3M/shell_exact.py`); MuJoCo's values (180751.03343200684, 140683.07421875, 81026.3416671753) are 1.1 % / 1.6 % / 0.8 % from it. Why its ε-shell lands there was not isolated.
- **Call.** Parity: the documented contract is only "mass concentrated on the surface" (`XMLreference.rst`, `shellinertia`); MuJoCo's own comment calls it an approximation; the other four shell formulas already agree (read: `geom.rs:585-711` vs `user_objects.cc:3209-3322, 3357-3402`). If wrong: ellipsoid shell inertias move ~1 % away from the uniform-density value; 1 corpus doc.
- **Change.** `compute_geom_shell_inertia`'s ellipsoid arm becomes MuJoCo's formula; `shell_ellipsoid_inertia` and `gauss_legendre_mapped` go if unused.
- **Flips.** `h2_shell`: 1 `model_fp`, 0 `traj_fp`. `mesh_inertia_modes.rs:195` `t8` already pins MuJoCo's three values at 2 % tolerance — tighten to 1e-9 relative (fails on main: 1.1 %).

#### 1A.3.4 Weld `relpose` — sim-mjcf, Rigid-loading

- **Now.** With `relpose`, ours packs only the quaternion, unnormalised, and computes `eq_data[3..6]` from the anchor at the build pose (`builder/equality.rs:166-198`); a zero quaternion is packed as zero.
- **Target.** MuJoCo reads `relpose` into `data[3..10]` (`xml_native_reader.cc:2155`), zeroes an absent anchor (`:2183-2185`); `mj_setConst` keeps a nonzero quaternion (normalised) with the user's position, and computes both from `qpos0` only when the quaternion is zero (`engine_setconst.c:810-830`).
- **Change.** `process_weld`: quaternion part non-zero → `data[3..6] = relpose[0..3]`, `data[6..10] = normalize(relpose[3..7])`; zero → today's computed path.
- **Flips.** `h2_relpose`: 1 doc, model and trajectory (`equality_constraints.rs:800`). In the census it then agrees in the model and differs from step 33 (`cls_every3`: `onset@33`, not isolated). `builder/equality.rs:521` `weld_explicit_relpose_packed_verbatim` stays green (identity quaternion, zero position).
- **Test** (fails on main): `496f2186`'s literal → `eq_data[0][3..6] == 0`, `[6..10] == (0.7071067811865475, 0, 0.7071067811865475, 0)` (main: `(0.5, 0, 0)`, `0.707`).

#### 1A.3.5 `fusestatic` — which side is right (established) — sim-mjcf, Rigid-loading

- **The test that decides** is MuJoCo's documented invariant: "the new model has identical kinematics and dynamics as the original" (`XMLreference.rst:841-843`). Fused vs the same file with `fusestatic="false"`, `qacc` at t0 (`R3M/xml/{fs,nofs}_*.xml`, `out_*_fuse.jsonl`, `tool_*.jsonl`):

  | fixture | MuJoCo fused − unfused | main fused − MuJoCo unfused | prototype fused − unfused |
  |---|---|---|---|
  | child with explicit `<inertial>` (`fs_inertial`) | 2.88 | 23.9 | 1.2e-7 |
  | child with a `fromto` capsule (`fs_fromto`) | 11.28 | 26.08 | 2.9e-6 |
  | parent with explicit `<inertial>` (`fs_parentinertial`) | 2.63 | 37.6 | 5.9e-8 |
  | sim-urdf output, fixed tool on a moving arm (`fs_tool`; `pos` added to the root `<inertial>`, which sim-urdf omits) | 5.22 vs 1.62 | 0 vs 1.62 | 0 (both 1.62454881, 2.2e-4 from MuJoCo: §1A.3.9) |
  | the 2 corpus docs (z hinge at the origin) | 0 | 0 | 0 |

- **MuJoCo's two defects**, by reading and matching the numbers: `AccumulateInertia` composes `ipose ∘ bodypose` instead of its own comment's `bodypose ∘ ipose` (`user_objects.cc:2249`: `mjuu_frameaccum(other_ipos, other_iquat, other->pos, other->quat)`, with `mjuu_frameaccum` = outer frame first, `user_util.cc:469-479`), and uses the child's un-rotated `iquat` (`:2266`). Hand check: `a6fb60bd` gives MuJoCo `ipos (0.5,0,0)` = child pos (1,0,0) + un-rotated geom offset (0.5,0,0), mass-weighted; the geom actually sits at (1, 0.5, 0) in both engines.
- **Main's three defects**, measured: (a) a fused child's `fromto` geom stays in child coordinates (`fs_fromto`: geom at (0.15, 0, 0.1), MuJoCo and prototype (0.3061, 0.3419, 0.0396)) — `fuse_static_body` moves `pos`/`quat` but not `fromto` (`builder/compiler.rs:176-205`); (b) the child's explicit `<inertial>` is dropped (geoms move, `body.inertial` does not; `fs_tool` loses the 0.5 kg tool → `qacc` 0); (c) a parent with an explicit `<inertial>` never receives the child's mass.
- **The prototype's residual is eig3.** With an exact eigendecomposition in place of `mjuu_eig3` the fused − unfused gap is ≤ 1.1e-14 on all three (`R3_FUSE_EXACT`); with eig3 it is 5.9e-8…2.9e-6, the eig3 inaccuracy A10 §2.3 item 2 measured (cosine stop).
- **Change.** `fuse_static_body` resolves `fromto` in the child frame first, records each fused child as (its pose in the parent, its own inertia source: `<inertial>` or its geoms, and its own fused list); the builder accumulates each child's compiled inertia into the parent's (`AccumulateInertia` with `bodypose ∘ ipose` and the rotated `iquat`, skipped for the world and below mjMINVAL — `user_model.cc:4268-4270`). Moved geoms are excluded from the parent's own geom inertia. Prototype: `R3_FUSE` in `builder/compiler.rs` and `builder/body.rs::r3_body_inertia`.
- **Deviation entry.** "MuJoCo's `fusestatic` accumulates the fused child's inertia in the wrong frame order; ours keeps the documented invariant." Test: fused `qacc` == unfused `qacc` to 1e-5 relative on the four fixtures (eig3 bound; fails on main by 23.9–37.6). The 2 corpus docs are marked as this deviation in the gate (trajectories agree; `a6fb60bd`'s `qM` differs).
- **Flips.** `h2_fuse`: `model_fp` 1 (`a6fb60bd`: eig3 now diagonalises the accumulated tensor), `traj_fp` 0; the corpus has 10 `fusestatic` docs, 7 of them both-load. Downstream: sim-urdf always sets `fusestatic="true"` (`converter.rs:177`); every URDF with a fixed child of a moving link changes mass (`fs_tool`); not measured on the in-tree URDF examples (runtime MJCF).

#### 1A.3.6 `fromto` under a rotated `<frame>` — sim-mjcf, Rigid-loading (with M14)

- **Change.** `builder/frame.rs:164-174`: resolve the geom's `fromto` to pos/quat/size in the frame's coordinates (the M14 port), then `frame_accum_child` like every other geom. A geom whose radius comes from a default class must resolve after defaults (the prototype only handles an explicit `size`).
- **Flips.** `h2_framefromto`: `model_fp` 2 (the 2 docs), `traj_fp` 0. **Test:** `4eed75d1`'s literal → `geom_quat` ±(4.33e-17, 0.7071067811865476, 0.7071067811865475, 4.33e-17) (main (6.1e-17, 1, 0, 0)).

#### 1A.3.7 Forward kinematics at `qpos − qpos0`, and the spring length at `qpos_spring` — sim-core, Rigid-physics

- **FK** is ledger-L45's open half; prototyped here only far enough to isolate the model clusters (`core_r3/src/forward/position.rs`, hinge and slide). `h2a_R3_FKREF` vs `h2a_none`: `model_fp` 9, `traj_fp` 8 — `tendon_springlength.rs:83,112,141`; `joint-limits/stress-test/src/main.rs:28,115,194` (**a validator, CI-run**); `joint-limits/hinge-limits/src/main.rs:46`; `tendons/spatial-path:41`; `tendons/tendon-limits:44`. Census: +10 docs agree, 0 regress. The full fix (derivatives, `sim-gpu` shader) is not prototyped.
- **Spring length.** `compute_spatial_tendon_length0` (`tendon/mod.rs:55-77`) resolves the sentinel with a second FK at `qpos_spring`. `h2a_R3_SPRINGLEN`: `model_fp` 2, `traj_fp` 1 (`spatial-path:41`). Test: `tendon-limits` doc → `tendon_lengthspring == 0.7071067811865476` (with FK fixed and this not: 0.632455532033676).

#### 1A.3.8 Muscle F0 stays −1 in the Model — sim-core, Rigid-physics

- **Now.** `compute_actuator_params` Phase 4 overwrites `gainprm[2]`/`biasprm[2]` with `scale/acc0` only when `dyntype` is a muscle (`forward/fiber.rs:223-239`); readers take `prm[2]` (`forward/actuation.rs:585`, `:650`; `derivatives/hybrid.rs:153`, `:985`).
- **Target.** MuJoCo keeps `force = −1` in the Model and resolves `scale / max(mjMINVAL, acc0)` on every call, for any muscle gain/bias (`engine_util_misc.c:665-667`, `:708-710`).
- **Measured.** `<general gaintype="muscle" biastype="muscle" dyntype="none" … lengthrange="-1 1">` (`R3M/xml/gen_muscle.xml`): MuJoCo `qfrc_actuator` −6.482484378125003; main +0.69; prototype (`R3_F0` + A11's explicit `lengthrange`) −6.482484378125003. On the 6 muscle-shortcut docs `traj_fp` is bit-identical (`h2a_R3_F0`: 6 `model_fp`, 0 `traj_fp`).
- **Change.** Phase 4 deleted; one helper `muscle_f0(prm, acc0)` at the four MuJoCo-muscle readers (Hill/Millard extensions keep their own resolution). This also removes one of A1's "consumed inputs" that make `recompute_derived()` non-idempotent (A1 table, `fiber.rs:233-237`).
- **Downstream.** Any reader of `actuator_gainprm[i][2]` for a muscle: `git grep` before implementing (sim-gpu, cf-design, cf-osim not checked).

#### 1A.3.9 Found outside the census (measured, no corpus doc)

- **`<inertial>` orientation alternatives are silently ignored.** `parse_inertial_attrs` reads `pos quat mass diaginertia fullinertia` only (`parser/body.rs:455-470`); MuJoCo resolves `euler/axisangle/xyaxes/zaxis` (`user_objects.cc:2419-2424`) and refuses them with `fullinertia` (`:2404-2406`). **sim-urdf writes `<inertial euler=…>` for every rotated URDF inertial frame** (`urdf/src/converter.rs:456-458`). `fs_tool`: the tool's `iquat` is identity in ours, (0.98877, 0, 0.14944, 0) in MuJoCo; the remaining 2.2e-4 `qacc` gap of the prototype is this. 0 corpus docs, 0 in-tree URDFs with a rotated inertial (one parser test, `urdf/src/parser.rs:569`). Implement in Rigid-loading (A4's schema rule would otherwise refuse sim-urdf's output).
- **sim-urdf converts URDF `rpy` as intrinsic `xyz`.** It emits `eulerseq="xyz"` (`converter.rs:177`); URDF rpy is Rz·Ry·Rx. Measured: MuJoCo's URDF reader gives body_quat (0.79496, 0.23500, −0.20083, 0.52200) for `rpy="0.2 -0.6 1.1"`, MuJoCo on sim-urdf's MJCF (0.82580, −0.07238, −0.30053, 0.47170); with `eulerseq="XYZ"` they agree to 1e-8 (`R3M/xml/rpy*.xml`). The inertia tensor then still differs by 3.4e-3 (not isolated). 1 in-tree URDF string has a multi-axis rpy (a parser test). Hand-off: sim-urdf.
- **Spatial tendon velocity over a sphere wrap reads 0 at the initial forward** (`11976cd3`, `tendons/sphere-wrap/src/main.rs:43`): ours `tendonvel` 0, MuJoCo 0.007071067811865478; after one step both non-zero. On `main` too (hidden behind the `fromto` model difference). Hand-off: tendon/sensor dynamics.

### 1A.4 The 13 docs that move from "model" to a dynamics cluster

First differing quantity with every prototype on (`cmp_every3.jsonl`); these join the dynamics researchers' clusters:

| doc | now | cluster |
|---|---|---|
| `02eb5ff4`, `38a06052`, `11beabb1` (contact-tuning) | t0 `ncon` (ours 12, MuJoCo 6: extra box–plane contacts at dist +0.0152) | ledger-L41 |
| `b6af1a7e` (equality stress-test :206) | t0 `ncon` (capsule–capsule ours 1, MuJoCo 2) | NEW-PAIRCOUNT |
| `a4b1e607`, `b86fda11` | `con_frame` | NEW-FRAME |
| `50190505`, `6343fad7`, `f59e50fc` (`sleeping.rs`) | step 10 | sleep |
| `11976cd3` | t0 `sensordata` (`tendonvel`) | new, §1A.3.9 |
| `496f2186` (relpose), `49c63d4b` (ML spec md), `5456113f` | steps 33 / 84 / 87 | late onset, not isolated |

### 1A.5 The 8 still model-different

`68a35ea3`, `a6fb60bd` (fusestatic, MuJoCo wrong — deviation); `cb2e58ac`, `7b6eff02`, `9b0854df` (mesh frame + f32, unowned); `8e77cf7b`, `f755f979` (cylinder `bias` → refusal, A3-O5); `d22fcd34` (sleep under RK4, A8 SP-9).

### 1A.6 Commits this part adds (each carries its tests and flips)

| # | PR | commit | rows | after |
|---|---|---|---|---|
| R3-P1 | P | `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo` | §1A.3.8, A11 item 4 | A11 LR-a if that lands in P; else alone |
| R3-P2 | P | `fix(sim-core): spatial tendon spring length at qpos_spring` | §1A.3.7 | ledger-L45 FK commit (the test needs both) |
| R3-L1 | L | `fix(sim-mjcf): plane and hfield geoms have MuJoCo's volume; zero-volume mass is zero` | §1A.3.1 | A10 MH3 (selection) |
| R3-L2 | L | `fix(sim-mjcf): discardvisual keeps the discarded geoms' inertia; sim-urdf emits no visual geoms` | §1A.3.2 | M11 (A3 k01: resolved contype) |
| R3-L3 | L | `fix(sim-mjcf): ellipsoid shell inertia as MuJoCo` | §1A.3.3 | — |
| R3-L4 | L | `fix(sim-mjcf): weld relpose position kept and quaternion normalised` | §1A.3.4 | M1 |
| R3-L5 | L | `fix(sim-mjcf): fusestatic accumulates body inertias, moves fromto geoms` (body: the deviation) | §1A.3.5 | M11 (A3 k02), A10 MH2 (eig3) |
| R3-L6 | L | `fix(sim-mjcf): <inertial> orientation alternatives` | §1A.3.9 | M6 (reader) |
| — | L | fromto in a rotated frame: **into M14** | §1A.3.6 | M14 |

### 1A.7 What this part cannot see

- Runtime MJCF: sim-urdf (two findings above came from it), cf-design, templates. Not run: validators (`urdf-loading/*`, `joint-limits/stress-test` under the FK change), sim-gpu, cf-osim, downstream suites.
- Prototypes are env-gated scratch code, not the PR code; clippy/fmt not run on them.
- `R3_S4` covers the worldbody arm only; `R3_FRAMEFROMTO` needs an explicit `size`; `R3_FUSE` ignores `boundmass`/`boundinertia` on the fused child (MuJoCo clamps the child at its compile, `user_objects.cc:2449-2453`, before fusing).
- The 3 mesh docs and the sleep-RK4 doc were not prototyped.
- Model fields the census does not compare (e.g. `xipos` of massless bodies, mesh vertices, `actuator_acc0` of non-muscles) can still differ.

---

## Part 1B — flex: why the census cannot see it, and what makes it comparable

*(Researched by a fork of this researcher; `FX/` = `SCRATCH/rigid_spec/r3_model_and_gate/flex/`.)*

Fork section, 2026-10-06. Repo read-only: `git status --short` was empty and HEAD was `fbc0ad54` at both start and end. Code = `main` @ `3520544e` (that commit only adds spec docs).
- Scratch: `SCRATCH/rigid_spec/r3_model_and_gate/flex/` (written `FX/`). Scripts are in `FX/scripts/`.
- Per-doc list: **`FX/flex_docs.tsv`** (80 rows: form, dims, pins, our status, MuJoCo's error at each rewrite stage, the census class per variant, source `file:line`).
- Oracle: `rigid_oracle_350/venv` (mujoco 3.5.0). Citations: `mj350src/mujoco` (3.5.0).
- Our side: a copy of the census harness built against the repo crates (`FX/harness`). Prototype runs used the census copies `core_fix_b`/`mjcf_fix_b` (`FX/harness_b`). Both `target/` dirs are deleted.

### 1B.1 Which docs are flex (counted two ways)

- **By XML** (`grep_flex_or_flexcomp.txt`): **80** of the 1,584 docs contain `<flex` or `<flexcomp`.
  - 77 contain `<flex>` and 3 contain `<flexcomp>`. All 80 sit inside `<deformable>`.
  - Composites make no flex in either engine:
    - The corpus has 24 composite docs, and every one is `type="cable"`.
    - MuJoCo 3.5.0 builds only the cable type, as bodies (`user_composite.cc:202-236`). Every other type is an error ("deprecated").
    - Ours does the same (`builder/composite.rs:38-79`).
- **By loaded model.** Our loader was run over all 1,584 docs (`ours_t0.jsonl`): 1,478 ok and 106 err, the same split as `emb_1.jsonl`.
  - **79** docs have `nflex > 0`.
  - The two counts agree: every doc with `nflex > 0` is in the grep set.
  - The 80th grep doc, `3e109bc2` (`flex_unified.rs:1353`), is one our loader refuses ("node references undefined body").
  - A13:261 says "75 flex docs that ours loads". This run measures 79. The difference is not reconciled.
- **MuJoCo 3.5.0 loads 0 of the 80** (`corpus350.jsonl`, re-run in `mj_S0.jsonl`).

### 1B.2 Why each engine refuses: the first error, and what it hides

The docs were rewritten in cumulative stages by `FX/scripts/convert.py` and loaded in MuJoCo after each stage (`mj_S{0..5}.jsonl`). Our loader loads 79 of the 80 docs as written.

| stage | rewrite (spec source) | MuJoCo 3.5.0 first error (count) |
|---|---|---|
| S0 | as written | `flex@density` 71 · `<flexcomp>` element 3 · unknown body '0' 2 · `flex@mass` 2 · `<vertex>` element 2 |
| S1 | drop dead attributes: flex/flexcomp `density`, flex `selfcollide`, edge `solref` (A4:468,474; RIGID_SPEC §3 "flex density") | `<vertex>` 65 · `<element>` 4 · `<flexcomp>` 3 · `bending_model` 3 · unknown body 2 · `mass` 2 · `<pin>` 1 |
| S2 | **A4-Q1 literally**: `<vertex pos>`/`<element data>` children → `vertex=`/`element=` | **`body` required 51** · `<pin>` 19 · `<flexcomp>` 3 · `bending_model` 3 · unknown body 2 · `mass` 2 |
| S3 | drop our extensions `<pin>`, `mass`, `bending_model` (A4:456-458) | **`body` required 75** · `<flexcomp>` 3 · unknown body 2 |
| S4 | **our meaning written in MuJoCo's form** (no appendix specifies this; §4) | **OK 72** · flexcomp `name` 2 / `spacing` 1 · unknown body 2 · "elem size must be multiple of (dim+1)" 1 · `element` required 1 · "trilinear interpolation cannot do self-collision" 1 |
| S5 | S4 + A9 2b `elastic2d="bend"` where thickness > 0 (A9 §2.4) + A13 D3 `<equality><flex flex=…/>` | **OK 66** · "pins are not supported for bending" 6 (`user_mesh.cc:4083-4086`) · the same 8 as S4 |

**Each first error hides at least one more** (S0 → S3): `density` hides the children, the children hide the missing `body`, and so on.

**`body` is required by MuJoCo's reader** (`xml_native_reader.cc:1478`; `element` too, `:1488`). The compiler then treats the listed bodies *as* the vertices (`user_mesh.cc:4117-4146`): it creates no bodies and no dofs.

**Ours does the opposite.** Our `<flex>` creates one new body per vertex, with 3 slide joints, or none if the vertex is pinned (`builder/flex.rs:231-400`). The vertex mass is lumped from a **hard-coded density 1000**, because the `density` attribute is never parsed (`flex.rs:639-708`, `types.rs:4086`). The lumping also applies a 0.001 floor and, for dim 2, multiplies by the thickness, whose default is −1, so it falls to the floor.

**Ours at main on the rewritten docs** (`ours_S2.jsonl`, `ours_S5.jsonl`):
- S2: 79 load. **72 of them load with `nflexvert = 0`**, because `vertex=`/`element=` are ignored (A6:213).
- S5: **80/80 load, all with `nflexvert = 0`.** The flex is silently empty.

### 1B.3 Clusters

| cluster | n | ours @ main | MuJoCo 3.5.0 (first → hidden) | what makes it comparable | measured after rewrite |
|---|---|---|---|---|---|
| C1 `<vertex>`/`<element>` children | 62 (14 with pins) | loads | `density` (58) / `mass` (2) / `<vertex>` (2) → children → `body` required (→ `<pin>` 14) | RIGID_SPEC §3 density removal + A4-Q1 + **explicit vertex bodies (§4, in no appendix)** + A9 F-commit `elastic2d` + A13 D3 equality | MuJoCo loads **62/62** (S5) |
| C2 children + pins + bending | 6 | loads | as C1, then "pins are not supported for bending" | A9 Q3 lenient deviation (ours keeps bending with pins). Comparable only **without** bending (S4 + equality) | MuJoCo loads **6/6** without `elastic2d` |
| C3 `node=` | 5 | 4 load (one 3-slide body per node body, `flex.rs:36-70`); `3e109bc2` refused | `density` → `<element>` → `body` required | **Not comparable in MuJoCo's spelling.** MuJoCo's `node` means trilinear interpolation (`user_mesh.cc:4105, 4148-4154`; `:4126-4128` refuses self-collision). Same attribute, different meaning. Needs a refusal, or a rewrite to explicit bodies | rewritten as child bodies: **4/4 load**. `3e109bc2`: both refuse |
| C4 `vertex=`/`element=` with `body="0"` (`builder/mod.rs:1345,1379`) | 2 | loads an **empty** flex (attributes ignored) | unknown body '0' (`user_mesh.cc:4088`) | A4-Q1 implementation + a named body in the test | not measured (needs the attribute form in ours) |
| C5 `<deformable><flexcomp>` (`build.rs:1447,1489`, `flex_unified.rs:185`) | 3 | loads (our grid generator) | unknown element (MuJoCo's flexcomp is body-level, `xml_native_reader.cc:312-327`). Moved there: `name` required ×2, `spacing` needs 3 values ×1 | A4-Q3 keeps ours under `<deformable>` ⇒ **never MuJoCo-loadable as written**. A body-level flexcomp also triangulates differently (A13:264) | 0/3 |
| C6 partial element `data="0"`, dim 1 (`deformable_friction_dt25.rs:22`) | 1 | loads; the partial element is dropped silently (`parser/deformable.rs:118-130`) | → "elem size must be multiple of (dim+1)" (`user_mesh.cc:4114-4116`) | RIGID_SPEC §3 A6-Q2 refusal ⇒ both refuse | — |
| C7 empty `element` (`flex_unified.rs:2578`) | 1 | loads a 1-vertex flex | `element` required / "elem is empty" (`user_mesh.cc:4111-4113`) | **no spec row**: A6-Q2 covers length ≠ dim+1, not empty | — |

So the census can compare at most **72 docs** (C1 + C2 + 4 of C3), and only after rewrites, one of which (§4) the spec does not contain.

### 1B.4 The gap: A4-Q1 is not "the same meaning"

A4-Q1 says "implement MuJoCo's `vertex=`/`element=` attributes … same meaning, mechanical rewrite". That is not what was measured:
- MuJoCo's `<flex>` **names existing bodies as its vertices**. With `vertex=`, the vertices sit in those bodies' frames, and `body` must list 1 or nvert names (`user_mesh.cc:4134-4146`).
- Ours **creates** the vertex bodies and their dofs.

A doc in MuJoCo's form therefore needs the vertex bodies written out. That is stage S4: one world body per vertex at the vertex position, 3 slides unless pinned, `<inertial>` with our lumped mass and `diaginertia 1e-15`.
- The inertia value is needed because MuJoCo refuses a moving body whose inertia is below `mjMINVAL` (`user_model.cc:4993-4997`, `:5310-5317`).
- Slide-only bodies never read rotational inertia. That is by reading; the census compare tolerates the 1e-15.

For our loader to read those docs the way MuJoCo does, it must take MuJoCo's meaning of `body`: no auto-created bodies, vertex mass from the bodies.
- Without that change, ours loads the MuJoCo form as an empty flex. That was measured (S5 80/80, `nflexvert = 0`).
- This is a spec decision. It is not in A4, A9 or A13. If it is not made, flex stays outside the census and the gate.

### 1B.5 What the census sees once comparable (ours = original doc, MuJoCo = S5, or S4 + equality for C2)

- Comparison: `parity_census/scripts/compare.py` at 1e-9 over the 72 docs.
- **35 of 66 docs differ between two of our own runs** (ledger-L27, `nondet_r1r2.txt`). The class counts were identical across those two runs.
- First differing quantity per variant (`flex_docs.tsv` columns 12–15; the C2 rows are e2 in every column):

| variant | agree | qfrc_passive | qfrc_constraint | ncon | nefc | model |
|---|---|---|---|---|---|---|
| main, census excitation e1 | **16** | 24 | 3 | 13 | 11 | 5 |
| main, e2 | **0** | 24 | 19 | 13 | 11 | 5 |
| A13 prototype `ISO_FLEX=1` (census `core_fix_b`), e2 | **23** | 30 | 0 | 13 | 1 | 5 |

The "model" column is 4 node docs (body numbering) and 1 `fromto` doc (A14 / ledger-L45b).

**What each column traces to:**

- **Before any of this: two model differences in all 72 docs.** `FX/scripts/normalize.py` removes them before comparing.
  - (a) MuJoCo's `mjEQ_FLEX` equality appears in `neq`, while ours has rows but no equality. This goes away with A13 D3.
  - (b) **Ours' `body_gravcomp` and `jnt_actgravcomp` are shorter than `nbody` and `njnt`.** Flex vertex bodies push neither (`flex.rs:285-400` vs `body.rs:220`).
  - **NEW bug, measured:** a flex doc plus any `gravcomp` body **panics** in `forward`/`step` at `dynamics/rne.rs:357` ("index out of bounds: the len is 2 but the index is 2", `FX/xml/flex_gravcomp.xml`). MuJoCo loads such a doc. **Hand-off: sim-mjcf flex builder.**
- **The e1 "agree" is an artefact of the excitation.** e1 sets `qvel_i = 0.1·(1 + i mod 3)`. With 3 slide dofs per vertex, every free vertex gets the same velocity (0.1, 0.2, 0.3), i.e. a rigid translation. Under e2, all 16 docs differ at t0 in `qfrc_constraint`.
- **Edge rows (ledger-L39c, A13 §3).** On main, `qfrc_constraint` is off from diagApprox; with `ISO_FLEX=1` it is gone. `nefc` on main comes from rows that ours makes for rigid edges between pinned vertices. 10 of the 11 such docs have pins, and with `ISO_FLEX=1` 1 is left.
- **NEW: `<elasticity damping>` (25 docs; all 24 qfrc_passive docs on main, plus `e8c7216a`, which moved there from `nefc`, have it).**
  - Ours applies `flex_damping` as **absolute per-vertex velocity damping**, `−damping·qvel` (`forward/passive.rs:474-490`).
  - MuJoCo uses it only as Rayleigh damping, inside bending (`engine_passive.c:265`) and inside stiffness elements (`:305`, `:330`, with `kD = damping/timestep`). A flex whose stiffness is zero skips that block, its dim-1 flexes included (`:271`).
  - **Measured:** with `damping` removed on both sides (`FX/nodamp/`, `ISO_FLEX=1`, e2), 15 of 25 agree, 9 move to `qfrc_constraint` (not isolated) and 1 to the trajectory.
  - No appendix has a row for this.
- **C2 (pins + bending).** With `ISO_FLEX=1`, 5 differ in `qfrc_passive`, because ours bends with pins and MuJoCo cannot. 1 agrees.
- **`ncon` (13, all nondeterministic).** The contact representations differ:
  - MuJoCo's flex–geom and flex–flex contacts are per **element** (`engine_collision_driver.c:402-425`; `contact.elem`).
  - Ours treats each vertex as a sphere against geoms (`collision/pairs.rs:17-22`) and stores flex–flex contacts by vertex (`flex_narrow.rs:345-346`).
  - Counts differ: 4 vs 3, 50 vs 31, 4 vs 6. Not isolated further.
  - Separately, `30bf91f8` (`flex_flex_collision.rs:590`): ours makes 7 self-contacts, MuJoCo 0. Not isolated.

### 1B.6 What `compare.py` and the gate would need for flex

1. **An excitation that moves vertices relative to each other.** e2 is enough: it turned 16 false "agree" into differ.
2. **A mapping from MuJoCo's flex equality to ours' FlexEdge rows**, or A13 D3 landed first.
3. **Contact keys.** `mj_census.py:118-125` records `geom`/`flex`/`vert` and not `elem`. Ours' `Contact` has `flex_vertex`/`flex_vertex2` and no element id.
   - Element contacts cannot be matched against vertex contacts, so `con_key` (compare.py) cannot pair them.
   - Until ours makes element contacts, a flex doc can only be compared by `ncon` and the aggregate forces.
4. **Model fields.** `cmp_model` compares only `flex_dim/vertnum/edgenum/elemnum` and `flexvert_bodyid`. It does not compare:
   - the `flex_edge` order or flaps (ledger-L27, A6 M3, A9 2a);
   - `flexedge_length0` or `flexedge_rigid`;
   - `flex_damping` or `flex_stiffness`.
5. **Whether the gate needs (3)–(4) is for Part 2.** The vertex slide dofs make vertex positions part of `qpos`/`qvel`, so with (1) the trajectory already sees flex dynamics. (3)–(4) only matter for localizing the first difference.
6. **The precondition is §4.** No flex doc enters a both-engines corpus until the loader decision is made and the 75 docs are rewritten.

### 1B.7 Spec items this adds (none is settled by the appendices)

- **§4: vertex bodies.** Either ours adopts MuJoCo's `body`/`vertex` meaning and the docs get explicit bodies, or the `<flex>` form stays our extension and flex stays outside the census.
- **C3 `node`.** Same attribute, different meaning. The parity rule's "silently does something other than the file asked" applies, so: refuse, or implement trilinear interpolation.
- **`<elasticity damping>` semantics** (§5): 25 docs.
- **Short `body_gravcomp`/`jnt_actgravcomp` and the `rne.rs:357` panic** (§5).
- **C7: empty `element`** has no refusal row.
- **Flex contacts:** element-based in MuJoCo, vertex-based in ours.

### 1B.8 What this cannot see

- **Every comparison is ours(original doc) vs MuJoCo(my rewrite).** S4 is a hand port of `create_flex_vertex_body`/`compute_vertex_masses`. Ours never loaded a rewritten doc with MuJoCo's meaning, because that is not implemented.
- **The prototype is not a complete flex fix.** It is the census's `core_fix_b`, which also carries L32/L42/L44 and has the edge rows only. A6 M3 (edge order) and A9's flaps/`elastic2d`/tet prototypes were **not** in that build. With them, classes may move. Not measured.
- **Nondeterminism.** 35 of 66 docs are nondeterministic in ours, and one run's classes are shown (r1 and r2 gave the same counts).
- **Not covered:** the 7 flex `format!` templates (A4:570) and runtime-generated flex.
- **Horizon and depth.** Only the t0 first quantity plus 100 steps.
- **Mechanisms not isolated:** the `ncon`/self-contact mechanisms, and the 9 docs left at `qfrc_constraint` after damping is removed.

---

## Part 2 — the permanent parity-census gate (design, measured, prototyped)

*(Researched by a fork of this researcher; `GT/` = `SCRATCH/rigid_spec/r3_model_and_gate/gate/`.)*

Researcher fork, 2026-10-06. Repo read-only: at start `git status --short` empty, HEAD `fbc0ad54`; at end the same (§9). Code under test = `main` @ `3520544e` (the branch adds only `.md` under `sim/docs/todo/spec_fleshouts/`). Scratch `SCRATCH/rigid_spec/r3_model_and_gate/gate/` (written `GT/`): scripts `GT/scripts/`, generator `GT/gen/`, Rust prototype `GT/proto/` (lib `cmp.rs` = port of the census `compare.py`, `record.rs` = the census harness, `lib.rs` = verdict + ratchet, `tests/census_gate.rs` = the test), outputs `GT/out/`. MuJoCo = `SCRATCH/rigid_oracle_350/venv` (3.5.0). The machine was shared with other builds throughout (load average 2.7–5.8): every time below gives wall and user CPU.

### 2.0 What the prototype is, and that it is the census

- **Golden generator** `GT/gen/gen_census_golden.py` (85 lines): the census's own `mj_census.py` `model_json`/`data_json` imported unchanged, plus excitation e2, checkpoints, refused docs, non-finite numbers as strings (`serde_json` has no `Infinity`). One JSON per doc + `meta.json` (versions, platform, rules; no timestamp).
- **Fixture check**: its e1 output equals the census's `out/mj.jsonl` for **1,214/1,214** docs, and its e2 equals `out/mj_e2.jsonl` for **1,214/1,214** (exact `==` after the string encoding).
- **Port check**: the Rust comparison reproduces the census's class (`classify.py`: model / state / api / agree) for **1,214/1,214** docs on `main` and **1,214/1,214** on the census prototypes (`cls_orig.jsonl`, `cls_fixB_all.jsonl`); at census rules it counts **795** and **850**, A16's numbers. The prototype build = `core_fix_b`/`mjcf_fix_b`/`geom_fix_b` with `ISO_IMP=1 ISO_FIX=2 ISO_FLEX=2` (reproducing 850 is what confirms those were the `fixB_all` toggles; they were not recorded).

### 2.1 (a) Golden data per doc

**Recommendation:** status (`ok`/`refused`, no message); the model fields as the census compares them; per excitation **e1 and e2**: `forward` dumps at **steps 0, 1, 100**, and qpos/qvel/act/time at **19 checkpoints** (1–10, 20, 30 … 100); horizon 100. 21.2 MB of JSON in 1,585 files, **8.1 MB as a git pack** (repo pack today: 49.79 MiB, `git count-objects -vH`).

**What each component buys** (agree count under the gate's rule; `GT/out` from the census `cmp_*` files and the prototype):

| golden | agree, main | agree, prototypes | git pack |
|---|---|---|---|
| e1, dump 0 only | 811 | 866 | — |
| e1, dumps 0, 100 (A16's rule) | 795 | 850 | — |
| e1, dumps 0, 1, 100 | 793 | 847 | 4.8 MB |
| e1+e2, dump 0 | 792 | 847 | 6.3 MB |
| e1+e2, dumps 0, 100 | 783 | 838 | 7.1 MB |
| **e1+e2, dumps 0, 1, 100** (prototyped) | **783** | **837** | **8.1 MB** |

- e2 adds 10 differing docs on main (793 → 783) and 10 on the prototypes (847 → 837). The dump at step 1 adds 2 docs on main under e1 alone (`con_frame`, `con_pos`); with e2 in, it adds 0 on main and 1 on the prototypes (838 → 837). Dropping it saves 0.96 MB of pack.
- What the dumps at 1/100 catch that t0 + trajectory do not (main, e1): accelerometer sensors ×7, the 7 both-NaN docs (`qfrc_passive`), sensor delay ×1, contact frame/position ×3.

**Horizon** (main, e1, 134 state-differing docs, first differing step): ≤ 1: 61; 2–10: 40; 11–30: 19; **31–100: 14** (`GT/scripts/horizon.py`). A 30-step horizon loses those 14 onsets. Beyond 100: not measured.

**Checkpoints vs every step** (every doc whose counts match, model-differing docs included; `GT/scripts/checkpoints2.py`):

| | state-differing docs | missed by {1,2,5,10,20,50,100} | missed by 1–10 + every 10th | missed by all 100 |
|---|---|---|---|---|
| main e1 | 158 | 1 | 0 | 0 |
| prototypes e1 | 121 | 2 | **1** | 0 |
| main / prototypes e2 | 165 / 128 | 0 | 0 | 0 |

The two transients, per-step magnitudes printed: `49c63d4b…` (prototypes) differs at step 84 only (2.1e-9, the step both engines blow up to ~1e13 and auto-reset; from step 85 they agree to 2e-16); `5456113f…` differs 1.0e-8 at step 87, decaying to 7.3e-10 by step 100. The checkpoint set therefore misses a difference that heals before the next checkpoint; all 100 steps would cost **28.3 MB** of pack (57 % of the repo's).

**Format and size** (git pack = a scratch repo with only those files, `git gc --aggressive`):

| e1+e2, dumps 0/1/100 | raw | tar.gz -9 | tar.xz -9 | git pack |
|---|---|---|---|---|
| every step, shortest-repr JSON | 58.9 MB | 15.4 MB | 7.3 MB | 28.3 MB |
| every step, 13 significant digits | 45.1 MB | 12.2 MB | 5.6 MB | 22.5 MB |
| 19 checkpoints, repr JSON | 21.2 MB | — | — | **8.1 MB** |
| 7 checkpoints, repr JSON | 15.8 MB | 2.3 MB | 1.2 MB | 5.1 MB |

- e1 alone, every step: JSON 31.3 MB → gz 8.0 / xz 3.8 / zstd-19 4.1 MB; the same numbers as raw f64 15.9 MB → gz 5.8 / xz 3.2 / zstd 3.6 MB. Binary saves little after compression; `.npz` per doc was **larger** on disk than JSON (60.3 vs 33.0 MB `du`, per-array zip overhead). Git compresses each blob, so committing plain JSON costs the pack size above, not the raw size.
- Of the e1 raw bytes, the trajectory is 22.7 MB of 31.1 (`GT/out/sizes_e1.txt`); the model 4.3 MB.
- **Regeneration is byte-stable**: three runs gave identical files — after dropping MuJoCo's error text. With the text, doc `67ed2610…` named `'a.stl'` in 4 of 6 runs and `'b.stl'` in 2 (the RIGID_SPEC §8 "message varies" case). So golden records status only.
- **MuJoCo cost**: 1,584 docs × 2 excitations × 100 steps = 3.4–4.7 s wall (3.1–4.4 s user), peak 56 MB.
- **Existing golden** (for coexistence): 3.4.0, `.npy` v1.0 per field (`gen_conformance_reference.py:184` refuses other versions; `gen_flag_golden.py:75`), 552 KB + 416 KB + 16 KB under `sim/L0/tests/assets/golden/`; read by a hand-written parser (`mujoco_conformance/common.rs:12`, duplicated at `integration/golden_flags.rs:35`); no LFS (`.gitattributes` covers only `xtask/hooks/*`). `sim-conformance-tests` has no `serde_json` today (its `Cargo.toml`); the workspace does (`Cargo.toml:613`). Per-field `.npy` would mean ~30k files for this corpus, so JSON + `serde_json` (dev-dependency) is the proposal.
- **3.5.0 alongside 3.4.0**: a separate directory and script, each pinned. Measured: golden from 3.4.0 instead changes 28 verdicts on main — 23 docs 3.4.0 refuses (sensor/actuator history attributes; 17 agree at 3.5.0), 5 dynamics labels; no other agree verdict moves (agree 766 vs 783).
- Side note: `sim/docs/MUJOCO_CONFORMANCE.md:316` says the flag golden is MuJoCo 3.5.0; its generator pins 3.4.0 (`gen_flag_golden.py:75`, `reference_metadata.json`).

**Regeneration script** (repo form of `GT/gen/gen_census_golden.py`): `sim/L0/tests/scripts/gen_census_golden.py` with a PEP 723 header `dependencies = ["mujoco==3.5.0", "numpy"]`, run as `uv run sim/L0/tests/scripts/gen_census_golden.py <docs> <golden>`; it refuses any other MuJoCo version (as `gen_conformance_reference.py:184`). The `uv run` form was **not run** here (the scratch venv was used).

### 2.2 (b) Where the corpus lives

**Recommendation: commit the extracted docs as a snapshot**, append-only.

- Size: 1,584 docs = 860,582 bytes, **742 KiB** as a pack; the manifest (doc id → source `file:line`) 446 KB raw.
- Doc ids are `sha256(text)[:16]` (`rigid_plan_tests/extract_mjcf.py:196`), so a file name is its content address.
- **Extract at test time** would need a Rust port of the extractor's Rust-literal lexer, and the golden would still be keyed by doc hash and generated offline. Every in-tree MJCF edit would then leave a doc with no golden: the test either fails (MuJoCo + Python needed on that commit) or skips (coverage silently lost). Measured exposure: **13 of 105** commits on `main` in the last 30 days (60 of 395 in 90) touched one of the 326 files that hold MJCF — an upper bound on doc changes (`git log -- <files>`).
- **Drift check** (prototyped, `GT/drift/`): re-running the extractor at HEAD `fbc0ad54` takes 1.9 s and finds **0 added, 0 removed** against the snapshot. Made to report a difference once: snapshot minus one doc → 1 added. Proposed as an on-demand script, or a non-blocking scheduled job, **not** a PR gate.
- **Refresh**: append-only (re-extract, add new ids, regenerate golden in ~4 s, bless). Old docs stay as fixtures, so a refresh never lowers the floor. The commits that rewrite in-tree MJCF (A4-Q1's 81 flex docs, flex `density`) refresh in the same commit, so their new docs are covered.
- **What a snapshot cannot see**: MJCF added or edited after the last refresh.

### 2.3 (c) Tolerance and the first-differing rule

**1e-9** on max|ours − MuJoCo| / max(1, max|MuJoCo|), kept. A16's justification ("an empty decade") was **per quantity at t0 and for the e1 trajectory**. It does not hold for the dumps at 1/100 or for e2: on main, the dump at step 100 has 2 docs in [1e-10, 1e-9) and 4 in [1e-9, 1e-8) for some quantity (`GT/out/decades.txt`). What decides agree vs differ is each doc's **maximum over everything compared** (dumps, trajectory, e1+e2). That has a gap at **[3.2e-10, 1e-8)** on both main and the prototypes (`GT/out/docmax.txt`).

Docs whose maximum lies within 10× of the tolerance, either side:

| tolerance | main | prototypes |
|---|---|---|
| 1e-10 | 6 | 7 |
| **1e-9** | **2** | **2** |
| 1e-8 | 7 | 5 |
| 1e-7 | 12 | 10 |
| 1e-6 | 7 | 6 |

Moving the tolerance ×3 changes 0 verdicts (looser) and 2 (tighter; 1 agree → `e1:t0:qacc`). 14 docs' trajectories drift below 1e-9 and count as agree (A16).

**Per-quantity rules** = A16 §0's table, ported unchanged: model fields only where they act; quaternions up to sign; contacts with |dist| > 1e-12 as multisets keyed by object pair (greedy nearest position); `con_zero` separate and last; efc counts with FlexEdge→Equality and the two friction types merged; NaN at the same position equal.

**First differing quantity** (verdict string), in this order:
1. load status;
2. the first model field, in `compare.py`'s order;
3. e1, then e2: the earliest event in time — at step k a differing **state** comes before the `forward` dump; inside a dump, MuJoCo's pipeline order.

A model-differing doc whose counts agree also carries its dynamics verdict (`model:geom_quat;dyn:agree`). On main **158 of the 192** model-differing docs agree in the dynamics; this pins them.

### 2.4 (d) Ratchet mechanics

**File** `verdicts.tsv`: `doc ⟶ verdict ⟶ note`, header `# agree_floor N`; 45 KB.

**Pinned per doc:** the **class**, not the label. The class is one of:
- `agree`;
- `model:<field>;dyn-agree|dyn-differs`;
- `dyn` (both load, the dynamics differ);
- `ours-refused`, `mj-refuses`, `both-refuse`.

**Labels** (`e1:state@7`, `t0:con_frame`) are recorded and printed when they change, but do not fail. Why: moving every initial qvel by n ulps, with the expected file fixed, gave (`GT/out/ulp_*.txt`):

| ulps | agree regressions | class shifts | label shifts |
|---|---|---|---|
| 1 | 0 | 0 | 0 |
| 16 | 0 | 0 | 2 (`26747e04` `state@70` → `t0:qfrc_constraint`; `31833131` `state@7` → `state@6`) |
| 1024 | 0 | 0 | 1 |
| 2²⁰ | 16 | 0 | — |

So pinning labels would make the test fail on last-bit noise. That is a proxy for cross-platform differences; Linux was not measured.

**Rules** (`GT/proto/src/lib.rs` `run_gate`). Transitions are ranked within MuJoCo's fixed status:
- golden ok: agree > model+dyn-agree > dyn-differing > ours-refused;
- golden refused: both-refuse > mj-refuses.

Failures:
- **improvement → FAIL until blessed.** `CENSUS_BLESS=1` rewrites the file and raises the floor. Reason: the ratchet protects only what is recorded. Measured: main run against the prototypes' blessed file reports **61 regressions**. If improvements passed with a message instead, those docs would still read "differs" in the file, and the same regression would pass silently.
- **regression → FAIL; bless never writes it.** A deliberate one (a stricter refusal of a MuJoCo-loading doc) is a hand edit: the row's verdict, a `divergence=<ID>` note, and the floor line. All three are visible in review.
- `divergence=<ID>` must name an ID present in the Intentional Divergences table, which needs an ID column added (`MUJOCO_CONFORMANCE.md:325` has none). A divergence doc that starts to **agree** fails (the deliberate deviation was lost).
- `ours-refused` on a MuJoCo-loading doc must carry `divergence=` or `known=<row>`.
- `nondet=` skips a doc. **None is needed today**: the 71 hash-order docs gave the same verdict in **60/60** processes (10 full + 50 targeted runs).

**Made to fail, and every lie caught** (`GT/out/`):

| case | result |
|---|---|
| main, blessed file | agree 783 = floor, pass (release and test profile) |
| prototypes vs main's file | 61 improvements (54 → agree, 7 model → dyn-agree), 6 label shifts → FAIL; bless → floor **837**, pass |
| main vs the 837 file | 61 regressions → FAIL (cargo test: `FAILED. 0 passed; 1 failed`) |
| our `body_mass[1]` ×(1+1e-6) | 877 regressions |
| ×(1+5e-10) (below the model tolerance) | 15 regressions, via the dynamics |
| one golden `body_mass[1]` ×1.01 | exactly that doc regresses |
| dangling divergence ID / divergence doc agreeing / 12 `ours-refused` without notes | 1 / 1 / 12 failures |

### 2.5 (e) CI time and placement

- **Test execution, 1,584 docs (1,226 compared):**
  - release: 1.83–1.88 s wall (1.77–1.79 s user);
  - the repo's test profile (opt-level 2 + debug assertions, `Cargo.toml:914-918`): 2.09–2.10 s (1.99–2.01 s user);
  - peak RSS 31–36 MB.
  - Our side alone: 1.66 s (release). It includes the JSON formatting and parsing the prototype does.
- Coverage-instrumented (local `xtask grade`): not measured.
- **Placement:** a module `layer_e_census.rs` in the existing `mujoco_conformance` target (`sim/L0/tests/Cargo.toml` `[[test]]`). It adds no new binary.
  - That crate runs only in `tests-release` **shard 1** (`quality-gate.yml:721`, `cargo nextest run --release $PKGS` `:814`). Shard 1 is the lightest leg, re-measured at 5m46s (`:699`). The crate is deliberately absent from `tests-debug` (`:468`).
  - The test is not `cfg_attr(debug_assertions, ignore)`, so the release-only trap does not apply. Locally, `cargo test -p sim-conformance-tests --test mujoco_conformance` runs it in about 2 s.
  - PR scoping: assets under `sim/L0/tests/` map to `sim-conformance-tests` by directory (`xtask/src/affected.rs:104`), and sim-core/sim-mjcf changes reach it through reverse dependencies.
  - Adding `serde_json` changes `Cargo.lock`, which forces a full run (`affected.rs` doc, full-fallback list) once.

### 2.6 (f) Deliberately different docs and load status

- Load status is part of the verdict, so Rigid-loading's progress is ratcheted too. Main: `mj-refuses` 264, `both-refuse` 94, `ours-refused` 12 (these match the design file's 264 / 94 / 12 at 3.5.0).
- Notes: `divergence=<ID>` for:
  - a stricter refusal (`ours-refused`);
  - a lenient load (`mj-refuses`, e.g. MuJoCo's 2-body cable);
  - our fix where MuJoCo is wrong (pinned non-agree).
- `known=<row>` marks a bug with a ledger/triage row. The census's `labels.json` gives one for every non-agree both-load doc, so the first file can carry A16's clusters.
- Main's 12 `ours-refused` docs map as:
  - mesh vertex-only ×4 (mjcf-D1);
  - connect to an unknown `body1` ×2;
  - non-finite keyframe/springlength ×3 (the non-finite decision → divergence);
  - springlength −0.5 ×1;
  - `fluidcoef` with 2 values ×1;
  - `<inertial>` in `<frame>` ×1 (decision 5 → divergence).

### 2.7 Commit placement

- **Rigid-physics, first commit:** `test(sim-conformance): MuJoCo 3.5.0 parity census gate`.
  - Contents: snapshot + manifest; golden + `meta.json`; both scripts; `layer_e_census.rs`; the `serde_json` dev-dependency; `verdicts.tsv` blessed at `main`, floor **783**, with `known=`/`divergence=` notes.
  - Depends on nothing. It compiles and passes at `main` (measured, prototype).
- **Every later commit** whose verdicts change carries its re-blessed file. Its improvements and class shifts are its flip list — the A6 per-commit protocol, executable for the docs both engines load. With the census's prototypes alone the file reaches **837**; the NEW-cluster fixes come on top of that.
- **Rigid-loading:**
  - load-status moves (`mj-refuses` → `both-refuse` is an improvement);
  - model-differing docs → agree (fromto etc.);
  - a new refusal of a MuJoCo-loading doc is a hand-edited regression with its divergence ID;
  - the commits that rewrite in-tree MJCF also append-refresh the snapshot.

### 2.8 What the gate cannot see

A16 §5 applies unchanged: refused docs beyond their status, flex, runtime-generated MJCF, horizons past 100, other states and inputs, derivatives, `step1`/`step2`, BatchSim, the first cause masking a second. In addition:

1. **Transients between checkpoints**: 1 measured doc (`49c63d4b…`, step 84 only); all 100 steps closes it at +20 MB of pack.
2. **Labels of non-agree docs** are informational: a differing doc can start differing earlier, or in another quantity, without failing.
3. **Cross-platform**: golden from macOS arm64 MuJoCo against our side on Linux x86-64 in CI — **not measured**. The proxy is the ulp table in §4.
4. **MuJoCo error kinds**: status only.
5. **Model fields outside the census list**: sites, cameras, mesh data, flex beyond counts, keyframes, tendon wrap geometry.
6. **The snapshot's staleness** between refreshes (§2).
7. **The trimmed dump set**: if the dump at step 1 is dropped, the 1 prototype doc caught only there.

### 2.9 Repo state, cleanup

At start: `git status --short` empty, HEAD `fbc0ad542ab9daa036fed3a561bed95af90918f3`. At end: empty, HEAD the same. No file was written in the repo. Scratch `target/` dirs deleted; large format experiments deleted. `GT/golden` (ck19, 21 MB), `GT/corpus/docs`, the scripts, the prototype sources and `GT/out/*.tsv`/`*.txt` are kept for re-running.

### 2.10 Addendum (this researcher): the gate prototype on every Part 1 prototype

The fork's prototype (`gate/proto`) copied to `R3M/gateproto` with one more variant, `r3` = `core_r3` + `mjcf_r3c` (Part 1A), run with the env in `R3M/every3.env`, golden `gate/golden`, divergence table `gate/out/fake_divergences.md`:

| run | result |
|---|---|
| `r3` against the census-prototype file (floor 837) | 176 improvements + 5 **regressions** (`both-refuse` → `mj-refuses`) → FAIL |
| the 5 | `parser/tests.rs:1237,1276,1305,1324,1378`: self-closing `<body/>`s that `R3_S4` now builds, while MuJoCo refuses each for a connect without `anchor` |
| `r3` without `R3_S4`, blessed | **floor 1,012**, 0 regressions, pass (`R3M/gate_out/exp_r3b.tsv`) |
| main (`orig`) against that file | agree 783 < 1,012, every flip listed as a regression → FAIL |
| class moves main → `r3` | model → agree 170, dyn → agree 59, `mj-refuses` → `both-refuse` 21 (A11's 4 lengthrange refusals + A9's 17 curve-keyword refusals), model → dyn 13 |

**What the 5 show.** M9 (self-closing elements, A4 §8.2) must not land before the connect-without-`anchor` refusal (A4 "G" list, MuJoCo `xml_native_reader.cc` connect rule). Otherwise the gate correctly fails on 5 new lenient loads. A4 §8.2 noted the change "to a different error (connect rule)"; the gate turns that note into an ordering constraint.

## Commits from this section (both PRs; each compiles and carries its tests and flips)

| # | PR | commit | after |
|---|---|---|---|
| G1 | Physics, **first** | `test(sim-conformance): MuJoCo 3.5.0 parity census gate` (Part 2 §7; floor 783) | — |
| R3-P1 | Physics | `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo` | G1; with or after A11 LR-a |
| R3-P2 | Physics | `fix(sim-core): spatial tendon spring length at qpos_spring` | ledger-L45 FK commit |
| R3-L1…L6 + M14 note | Loading | Part 1A §1A.6 | as listed there |
| — | Loading | M9 (A4) **after** the connect-anchor refusal | gate §2.10 |

Every commit after G1 carries its re-blessed `verdicts.tsv`. The improvements in that file are its census flip list.

## What this section cannot see (all parts)

- Everything in A16 §5: runtime MJCF, horizons past 100 steps, other states and inputs, derivatives, `step1`/`step2`, BatchSim, and a first cause masking a second.
- **Part 1A**: the prototypes are env-gated scratch code, and clippy/fmt were not run on them. Not run: validators, sim-gpu, sim-urdf's in-tree examples, downstream suites. Not prototyped: the 3 mesh-frame docs, sleep under RK4, and the full ledger-L45 FK fix.
- **Part 1B**: every flex comparison is ours (original doc) against MuJoCo (a hand rewrite); the A6 M3 and A9 flex prototypes were not in that build.
- **Part 2**: transients between checkpoints; label changes of non-agree docs; cross-platform golden (macOS golden, Linux CI) not measured; the snapshot's staleness between refreshes.
