> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid round 3 — sensors and kinematics: accelerometer, `ref`/ball `pos`, ball transition block, framequat, RK4+mocap, implicitfast+fluid, sleep under RK4

Researcher section, 2026-10-06. Repo read-only at HEAD `fbc0ad54` (branch `fix/rigid-sim-core-mjcf`; its source equals `main` @ `3520544e`, the branch adds only spec docs). Evidence is in `SCRATCH/rigid_spec/r3_sensors_kinematics/` (written `R3/` below): `xml/` fixtures, `scripts/` (MuJoCo side, comparators), `probe/` (Rust probe), `out/` (every number below), `census/` (copy of the parity-census harness), `fpharness/` (streaming fingerprint harness), `sleepws/` + `sleepprobe/` + `sleepharness/` (copies of A8's sleep prototype workspace and probes), `validators/`, `tests/`. MuJoCo oracle: `mujoco==3.5.0` (`SCRATCH/rigid_oracle_350/venv`); citations from `SCRATCH/mj350src/mujoco` (tag 3.5.0, 881544c), paths under `src/`.

**Base.** As asked: `R3/core` = the census's `parity_census/core_fix` = main + A12's multi-joint fix (`l32_multijoint/out/fix.diff`) + A13's prototype with its switches. Every "base" run sets `ISO_IMP=1 ISO_FIX=2` (A13's connect/weld impedance and explicit `qacc`). My changes sit behind further env switches in the same copy (`R3/out/prototype_core.diff`, +322/−21, 11 files): `R3_ACC`, `R3_X7`, `R3_X6`, `R3_REF`, `R3_BALLA`, `R3_IFLUID`. Item 5 is prototyped in a copy of A8's sleep workspace (`R3/sleepws`, switch `R3_RK4SLEEP`, `R3/out/prototype_rk4sleep.diff`). The switches are scratch-only; with all unset the code is the base.

**Method** (as `l32_multijoint.md`): same fixture in both engines; first differing intermediate quantity; both sides cited; scratch prototype; agreement measured; bit-identity elsewhere with the streaming fingerprint harness (`R3/fpharness`, derived from `determinism_verification/harness/src/bin/harness2.rs`: model `{:#?}` streamed into a hash, never materialised; qpos/qvel/act/time bits after 100 steps; every step's sensordata bits; `qacc`+`cacc`+`cfrc_int` bits after a final `forward()`+`inverse()`; t0 sensordata; with `R3_DERIV=1` also the FD and hybrid `A`,`B` bits) masked by two base runs plus `parity_census/nondet71.txt`; census re-run on the 1,214 both-load docs under both excitations (`R3/out/cmp_*.jsonl`, `R3/scripts/cmpsum.py`).

## Summary

| cluster (census doc ids in `labels.json`) | first differing quantity | cause | fix (switch) | measured after |
|---|---|---|---|---|
| NEW-ACCEL (8; +2 more under e2: `29661ba2` accelerometer, `72212e25` force/torque) | `cacc` linear part of a free body (= ω×v exactly) | ours omits the free joint's ω×v term in `mj_body_accumulators`; MuJoCo's `mj_comVel` gives it through the rotational `cdof_dot` | add the term (`R3_ACC`) | 7 of 8 + both e2 docs → agree; 0 regressions; fixture sensors ≤ 3.6e-14 over 200 steps |
| NEW-ACCEL `809fc2ad` (static body) | `sensordata` (9.81) | MuJoCo returns 0 for any object on a world-welded body (`engine_core_util.c:823-827`) | **recommend keep ours (+g)**, lenient deviation with a test; parity variant `R3_X6` measured | `R3_X6`: agrees; flips 3 sim-core lib tests only through an underived `body_weldid` |
| found alongside (0 census docs) | `cfrc_ext` | ours references contact/xfrc torques at `xipos` and omits connect/weld forces; MuJoCo includes them (`engine_core_smooth.c:2503-2650`) | `R3_X7` | force/torque sensors 2.8 → 6.8e-13 (connect), 3.44 → 1.35e-13 (contact), 0.30 → 1.2e-15 (xfrc) |
| L45ref (4 census docs; 10 in-tree docs use `ref`) | `xquat`/`xpos` at t0 | FK uses `qpos`, MuJoCo `qpos − qpos0` (`engine_core_smooth.c:125,137`) | `R3_REF` | 4 L45ref docs + 5 more (dynamics) → agree; X6 fixture 0.246 → 0 |
| ball joint `pos` (0 docs) | `xpos` | ours rotates a ball about the body origin; MuJoCo about its anchor (`:118-120,143-146`) | `R3_REF` (FK + velocity + subspace) | offset ball 0.526 → 1.1e-15 qpos (100 steps); non-root offset ball 0.457 → 5.6e-16 |
| A11's unexplained `muscle_qpos0_ref` residual | `actuator_force` via FK | the same `ref` defect | `R3_REF` on A11's own port | 6.65e-2 → 4.4e-16 qpos (100 steps) |
| found alongside (`251c5904`, spatial-path) | `qfrc_passive` | spatial-tendon `lengthspring` sentinel resolved at `qpos0`; MuJoCo at `qpos_spring` (`engine_setconst.c:1069-1082`) | `R3_REF` | 0.253 → 5.6e-17 qpos |
| ball/free position block of `A` (A14 §4.3) | FD position rows | ours differences next-state tangents taken at `qpos_t`; MuJoCo differences the perturbed next states directly (`engine_derivative_fd.c:55-65`); plus a `1e-10` clip in our quaternion log | `R3_BALLA` | 7.9e-4 → ≤ 2.8e-10 (FD and hybrid vs MuJoCo FD; ball, hinge→ball, free, offset ball) |
| NEW-FRAMEQUAT (1) | `body_iquat` (model) | principal-axis order (sensor formula already MuJoCo's) | none here: A10's MH2 (`eig3`) | with MuJoCo's iquat/inertia: 0.557 → 2.3e-15; A10's prototype gives MuJoCo's bits for this doc |
| NEW-RK4MOCAP (2) | contacts | **not RK4, not mocap**: box–box contacts of near-coincident boxes, and sphere–box contact position offset by \|dist\| | none here: collision area | Euler and static-body variants show the same first quantity; RK4+mocap with contacts off agrees to 5.6e-16 |
| NEW-IFLUID (1) | `qvel` after step 1 | MuJoCo builds `M − hD` only inside M's sparsity pattern (simple DOFs: diagonal only; `engine_forward.c:1186`, `engine_io.c:943-944`); ours uses dense D | `R3_IFLUID` (also full implicit: tree pattern) | 1.33e-5 → 5.6e-16; cross-branch tendon damping 5.8e-2 → 4.4e-16 (implicit and implicitfast) |
| SLEEP under RK4 | `tree_asleep` | **two MuJoCo defects** (isolated by intervention on MuJoCo itself): stale stage-4 poses on the slept tree, and stale `qacc` of sleeping DOFs moving the tree inside RK4 stages | lenient deviation: position re-forward on the sleep step + frozen stages (`R3_RK4SLEEP`) | stays asleep (1 transition vs MuJoCo's 584); equals MuJoCo-with-both-interventions to 2.3e-18 over 3000 steps |

Combined (`all` = `R3_ACC R3_X7 R3_REF R3_BALLA R3_IFLUID`): census e1 agree 847 → 859, e2 839 → 853, 0 docs move from agree to differ (`R3/out/cmp_all*.jsonl`). Suites: 0 test flips (sim-core lib 720, sim-mjcf lib 385, integration 1,338 + 27 ignored, mujoco_conformance 83); 16 validators all rc 0. Details per item below.

---

## 1. Accelerometer / framelinacc / force (NEW-ACCEL)

### 1.1 Fixture and first differing quantity
`R3/xml/acc_free.xml`: a free box (rotated, site offset) with a hinged child, a static body, and every acceleration-stage sensor type on each (accelerometer, framelinacc on site/body/xbody, frameangacc, force, torque). State: qpos0, qvel = e2-like `(0.1,−0.2,0.3,−0.4,0.5,−0.1,0.7)`.
- Equal: qpos, qvel, xpos (2.1e-17), xquat, cvel (1.1e-16), qM, qfrc_bias, **qacc (2.0e-14)** (`R3/out/acc_free_base_s0.json` vs `acc_free_mj_s0.json`; MuJoCo's com-referenced `cvel/cacc/cfrc_*` transported to `xpos` in `scripts/mj_probe.py`).
- **First differing: `cacc` of the free body, linear part.** ours − MuJoCo = (0.092872, 0.193274, 0.097892) = ω_world × v_world (0.09287182, 0.19327386, 0.09789197) to 1e-14. The hinged child carries the same linear offset (propagated). The force sensor on the free body reads (0.33, 0.18, 0.13) in ours, exactly 0 in MuJoCo (no joint force on a free body).
- Under e1 a free joint starts with v = ω component-wise and these docs' free bodies start unrotated, so ω×v = 0 at t0; the census saw them agree at t0 and differ after one step. Under e2 the census t0 gap was 0.13, the largest component of ω×v for e2's free-joint qvel (0.13, 0.11, 0.03).

### 1.2 Cause
- **MuJoCo** `mj_comVel` (`engine_core_smooth.c:2301-2321`): free joint → translational `cdof_dot = 0` (`:2303`), `cvel += cdof·v` (`:2306-2307`), then falls through to the ball case, whose rotational `cdof_dot = cvel × cdof` (`:2315-2317`) is taken with the translational velocity already in `cvel`. `mj_rnePostConstraint` (`:2653-2672`) sums `cdof_dot·qvel` into `cacc`, so `cacc` holds the spatial acceleration a_O − ω×v_O. `mj_objectAcceleration` (`engine_core_util.c:829-838`) then adds ω×v back (Coriolis) for the classical acceleration.
- **Ours** `mj_body_accumulators` (`sim/L0/core/src/forward/acceleration.rs:400`, per-joint loop `:586-605`): `cacc = parent + S·qacc + v_parent ×_m S·qvel`; for a free root body the cross term is 0, so `cacc` holds the classical a_O. `object_acceleration` (`dynamics/spatial.rs:312-337`, `:329`) then adds ω×v again → double-counted. The RNE has the missing term (`dynamics/rne.rs:244-262`, "Free joint correction"); the accumulators never got it. The sensors themselves (`sensor/acceleration.rs:81-99`, `:212-229`) are already MuJoCo's formulas.

### 1.3 What else `mj_rnePostConstraint` does that ours does not (found alongside; 0 census docs)
Measured on `R3/xml/force_connect.xml`, `force_contact_sph.xml`, and `force_contact.xml` + `R3P_XFRC`:
1. **Connect/weld constraint forces are missing from `cfrc_ext`.** MuJoCo adds `efc_force[i..i+3]` (+ weld torque `[i+3..i+6]`) at body1's point and subtracts at body2's (`engine_core_smooth.c:2562-2622`). Ours adds contacts only (`acceleration.rs:406-529`). Force sensors on a connected pendulum: 2.825 off.
2. **Contact torques are referenced at `xipos`** (`acceleration.rs:523`, `r = cp − xipos`) while `cfrc_int = cinert·cacc + … − cfrc_ext` is at `xpos`. Torque sensor, frictionless sphere with offset COM: 3.44 off.
3. **`xfrc_applied` is copied without shifting its torque from the COM to `xpos`** (`:402-404`; MuJoCo transforms it, `:2503-2517`). Torque sensor: 0.30 off.
4. **World body `cfrc_int[0]`**: MuJoCo adds each root's `cfrc_int` (referenced at that root's subtree COM) into body 0 without transport (`:2676-2679`), so its torque mixes reference points; ours transports. 12.4 apart on `acc_free` (`R3/out/acc_free_acc_s0.json`). Visible only through `data.cfrc_int[0]` or a torque sensor on a world site. **Not changed** (Q1.2).

### 1.4 The static body (`809fc2ad`, `parser/tests.rs:1784`)
MuJoCo's `mj_objectAcceleration` returns 0 for any object whose body is welded to the world (`engine_core_util.c:823-827`), a copy of `mj_objectVelocity`'s quick return (`:766-770`; there the transform would give the same 0 — MuJoCo's `cvel` of the welded table in `acc_free` is exactly 0). MuJoCo documents the accelerometer as "the linear acceleration of the site (including gravity)" (`doc/XMLreference.rst:6348-6350`). Measured (`R3/xml/static_vs_hinge.xml`, two identical boxes, one welded, one on a vertical-axis hinge at rest):

| | welded site | same site on a hinge at rest |
|---|---|---|
| MuJoCo accelerometer | (0, 0, 0) | (−4.905, 0, 8.4957) |
| ours | (−4.905, 0, 8.4957) | (−4.905, 0, 8.4957) |
| MuJoCo framelinacc | (0, 0, 0) | (0, 0, 9.81) |

A site that does not move reads two different proper accelerations in MuJoCo depending on whether a zero-velocity joint exists. Recommendation in Q1.1.

### 1.5 Now → Target → Change
- **Now:** as §1.2–1.3.
- **Target:** MuJoCo 3.5.0 `mj_rnePostConstraint` for `cacc`, `cfrc_ext`, `cfrc_int` (bodies ≥ 1); sensors unchanged; static-body readings kept as ours (deviation, Q1.1); `cfrc_int[0]` kept transported (Q1.2).
- **Change** (`forward/acceleration.rs`, crate-private, no signature change):
  - in the per-joint loop, for `MjJointType::Free`: `acc.linear −= ω_world × v_world` (same expression as `rne.rs:244-262`); better, one shared helper used by `rne.rs` and the accumulators so the two cannot drift again;
  - Step 1: shift `xfrc_applied` torque by `(xipos − xpos) × f`; contact torque lever `cp − xpos`; add connect/weld rows (MuJoCo's body points: connect `eq_data[0..3]` on body1, `[3..6]` on body2; weld `[3..6]` on body1, `[0..3]` on body2 — the same points our Jacobians use, `constraint/equality.rs:63-64,113-114`);
  - doc of `Data::cfrc_ext` (`types/data.rs:652-654`) says "referenced at the body origin `xpos`, like `cacc` and `cfrc_int`".
  - Interaction: the second-round decision moves `xfrc_applied` to MuJoCo's [force, torque] order with a new type; `cfrc_ext` stays [torque; force] (MuJoCo's own `cfrc_*` order), so Step 1 swaps the halves. Whichever lands second edits that line.

### 1.6 Measured
- Fixture `acc_free`, 200 steps (`R3/out/af_*.jsonl`): every sensor on moving bodies ≤ 3.6e-14 (base up to 3.4); qpos/qvel unchanged (1.1e-15, both). Static sensors 9.81 (ours) / 0 (with `R3_X6`).
- Census: e1 7 NEW-ACCEL docs → agree, `af3775ea` loses its sensor difference (its `con_zero` label remains); e2 the same plus `29661ba2`, `72212e25`; 0 regressions (`cmpsum.py cmp_base.jsonl cmp_acc.jsonl`, `cmp_base_e2.jsonl cmp_acc_e2.jsonl`). `R3_X6` adds `809fc2ad`. `R3_X7`: no census change (no census doc has a force/torque sensor in those configurations).
- Bit identity (`fp_acc` and `fp_accx7` vs two base runs, masked): **0** trajectory changes; `sensordata` changes in exactly 10 docs, all free bodies with acceleration-stage sensors: 7 of the 8 NEW-ACCEL docs, `29661ba2`, `72212e25`, `af3775ea` (`out/fp_acc.jsonl.changed.json`); `cacc/cfrc_int` bits in 506 docs (`R3_ACC`; 505 have a free joint, the other is a `composite initial="free"`) and 520 with `R3_X7`. `R3_X6` changes sensordata in 2 docs (`809fc2ad`, and `f2cd0104` = `sensors_phase4.rs:502`, a framelinacc on a world site that MuJoCo refuses to load).

### 1.7 Tests to add (each fails on main; measured on main with `probe orig`)
1. `accelerometer_free_body_matches_mujoco_3_5_0`: `acc_free.xml` (or a free box alone) at qvel `(0.1,−0.2,0.3,−0.4,0.5,−0.1)`, 1 forward + 100 steps, accelerometer/framelinacc/force/torque vs MuJoCo 3.5.0 golden at 1e-12. Main: 2.0 (accelerometer), 3.4 (force).
2. `force_sensor_includes_connect_force`: `force_connect.xml` vs golden, 1e-10. Main/base 2.825.
3. `torque_sensor_contact_lever_at_origin`: `force_contact_sph.xml` (frictionless sphere, offset COM, qpos `(−0.06, 0.05)`) vs golden, 1e-10. Base 3.44.
4. `xfrc_torque_moved_to_origin`: `force_contact.xml` with `xfrc_applied` on body 1, torque sensor vs golden. Base 0.30.
5. `static_site_reads_gravity` (the deviation test, Q1.1): `static_vs_hinge.xml`: accelerometer on the welded site equals the one on the hinged-at-rest site bit for bit, and equals R(site)ᵀ·(0,0,9.81). Passes on main and after; fails under MuJoCo's rule.

### 1.8 Flips
None in sim-core lib (720), sim-mjcf (385), integration (1,338), mujoco_conformance (83) (`R3/out/suites/{base,all}_*.out`). Validators: 16 run (`R3/out/val/`), all rc 0; `sensors_advanced`, `inverse_dynamics`, `free_joint` stdout byte-identical. With `R3_X6` 3 sim-core lib tests panic (`sensor_tests::test_accelerometer_at_rest_reads_gravity`, `_in_free_fall_reads_zero`, `test_multiple_sensors_coexist`): index out of bounds on `body_weldid`, which factory-built models leave at length 1 — not a semantic flip, but it means parity would need K6's derived `body_weldid` first.

### 1.9 Downstream
`data.cacc`, `cfrc_int`, `cfrc_ext` readers outside sim-core (`git grep`): `examples/fundamentals/sim-cpu/inverse-dynamics/stress-test/src/main.rs:575` (checks `cfrc_ext` reflects `xfrc_applied`; stdout unchanged, it applies at a body whose `xipos = xpos`), `examples/integration/full-pipeline/src/main.rs:179,274`, `examples/integration/sim-informed-design/src/main.rs:80,228` (read `cfrc_ext`; not run). sim-gpu has no accumulators (its `body_cacc` is the RNE bias).

---

## 2. Forward kinematics: `ref`/`qpos0`, ball `pos`, `xaxis`/`xanchor` (L45ref) — and A11's residual

### 2.1 Fixtures and first differing quantity
- `X6_hinge_ref.xml` (A14's, `ref="30"`): `xquat` at qpos0; trajectory 0.246 qpos after 100 steps (base and main).
- `X2_ball_offset.xml` (A14's, ball `pos=".2"` on a rotated body): `xpos` at t0; 0.526 qpos after 100 steps.
- `chain_offball.xml` (new): hinge root, child with an offset ball, rotated: 0.457.

### 2.2 Cause (both sides)
MuJoCo `mj_kinematics1` (`engine_core_smooth.c:40-185`): for each joint `xaxis = R·jnt_axis` (`:116`), `xanchor = xpos + R·jnt_pos` (`:119-120`); slide `xpos += xaxis·(qpos − qpos0)` (`:125`); hinge `qloc = axisAngle(jnt_axis, qpos − qpos0)` (`:137`); ball `qloc = normalize(qpos)`; then `xquat ·= qloc` and `xpos = xanchor − R_after·jnt_pos` (`:143-146`, the off-centre correction, ball and hinge alike); free `xaxis = jnt_axis` (`:77`).
Ours `mj_fwd_position` (`forward/position.rs`): hinge angle `qpos` (`:62`), slide `qpos` (`:93`) with `xanchor = pos` (`:96`), ball rotates about the origin and never writes `xaxis` (`:100-115`), free `xaxis` unwritten (`:116-131`). The sleep code's FK copy has the same hinge/slide/ball formulas (`island/sleep.rs:236,250,254-262`). The ball's velocity (`forward/velocity.rs:107-118`) and motion subspace (`joint_visitor.rs:175-190`) also assume the origin, while every Jacobian already uses the anchor `xpos + xquat·jnt_pos` (`jacobian/mod.rs:77-89`, `constraint/jacobian.rs:105-108`, `constraint/equality.rs:488`, `tendon/spatial.rs:391`, `dynamics/rne.rs:101`) — so today an offset ball is internally inconsistent (hybrid vs FD 0.98 on X2, `R3/out/tr_X2_ball_offset_*`).

### 2.3 Change (`R3_REF`)
- `forward/position.rs`: hinge `angle = qpos − qpos0`; slide `disp = qpos − qpos0`, `xanchor = pos + R·jnt_pos`; ball `xaxis = R_before·jnt_axis`, `xanchor = pos + R_before·jnt_pos`, rotate, `pos = xanchor − R_after·jnt_pos`; free `xaxis = jnt_axis`. With `qpos0 = 0` and `jnt_pos = 0` these are the same floating-point operations as today (bit-identity measured, §2.5). This absorbs A14's L36b core commit (ball/free `xaxis`, slide `xanchor`).
- `forward/velocity.rs` ball arm: `lin += ω_world × (xpos − xanchor)`; `joint_visitor.rs` ball subspace: linear rows `(R e_i) × (xpos − xanchor)` (both guarded by `jnt_pos ≠ 0` in the prototype to keep zero-offset balls bit-identical).
- The FK copy in `island/sleep.rs` gets the same formulas, or — better — is replaced by the one FK (A8 keeps `sync_tree_fk` only for the init-sleep path); one per-body pose function used by both removes the duplicate.
- Builder (Rigid-loading, A14's M17 addition): ball and free `jnt_axis` forced to (0,0,1), so a ball's `xaxis` matches MuJoCo when the MJCF gives an `axis`.
- **sim-gpu** `shaders/fk.wgsl:214,246` uses `qpos[qa]` for hinge angle and slide displacement: subtract `qpos0`. `JointModelGpu.axis[3]` is written 0.0 today (`pipeline/model_buffers.rs:142`), so it can carry `qpos0` without a layout change. An offset ball needs the same three changes on the GPU (FK, velocity, RNE subspace) or a GPU-side refusal (Q2.1). Not written or run.
- **Spatial tendon `lengthspring`** (`tendon/mod.rs:62-74`): the `[-1,-1]` sentinel is resolved from FK at `qpos0`; MuJoCo resolves it at `qpos_spring` (`engine_setconst.c:1069-1082`). Fixed tendons already use `qpos_spring` (`sim-mjcf builder/build.rs:666-680`). Change: run the second FK at `qpos_spring`. It only differs when `ref ≠ springref`; found through `251c5904`.

### 2.4 Measured agreement
| fixture | base | `R3_REF` |
|---|---|---|
| `X6_hinge_ref` 100 steps | 0.246 qpos | 0 qpos, 8.9e-16 qvel |
| `X2_ball_offset` 100 steps | 0.526 | 1.1e-15 |
| `chain_offball` 100 steps | 0.457 | 5.6e-16 |
| spatial-path `251c5904` 100 steps | 0.253 | 5.6e-17 (with the lengthspring change) |
| `X6_hinge_ref` transition A vs MuJoCo FD | 1.6e-2 | FD 0 (bit-equal), hybrid 4.6e-11 |
| `X2_ball_offset` A (with `R3_BALLA`) | 0.91 | FD ≤ 2.8e-10, hybrid ≤ 2.0e-10 |
| `chain_offball` A (with `R3_BALLA`) | 1.52 | FD ≤ 1.7e-10; hybrid 2.8e-2 alone, ≤ 1.9e-10 with A14's L36a applied (`R3/core_l36a`, `out/tr_chain_offball_o2.json`) |

**A11's residual is this defect.** `lengthrange/cases2/muscle_qpos0_ref.xml` (hinge `ref="20"`, muscle): on A11's own port (`lengthrange/core_lr`, copied to `R3/lr_core` with my FK hunk) 100-step qpos 6.65e-2 → 4.4e-16 with `R3_REF`; the port's lengthrange equals MuJoCo's to 1e-13 in both. On my base (no LR port) with MuJoCo's lengthrange forced in (`R3P_LR`), the same 6.65e-2 → 3.3e-16. (A11 recorded 2.9e-2 for this case; the two numbers were not reconciled.)

### 2.5 Bit identity, census, flips
- Fingerprints (`fpd_ref` vs two base runs): **9** docs change, all with a nonzero `ref` (model fields, 9; trajectory, 8; derivatives, 8; sensors, 5) — `tendon_springlength.rs:83,112,141`, `joint-limits/stress-test:28,115,194`, `joint-limits/hinge-limits:46`, `tendons/spatial-path:41`, `tendons/tendon-limits:44`. The 10th nonzero-`ref` doc, `41bb4969` (`joint-limits/stress-test:71`, a slide along x under gravity along z), changes only `xpos`, which no fingerprint covers; that is what the census flagged, and it now agrees. Every other deterministic doc is bit-identical on every fingerprint, derivative bits included. 0 static docs set a joint `pos` (A14 `count_joints.py`), so the ball change is invisible in the corpus.
- Census e1/e2: L45ref 4 docs → agree; 5 docs whose only remaining difference is now the `fromto` model field (L45b) and DERIVED `251c5904` agree in dynamics; 0 regressions.
- Suites: 0 flips. Validators: `joint_limits` rc 0, 5 printed values change (e.g. "Hinge limit activates: angle = 42.98° → 45.02°", "peak limit_frc 54.10 → 56.11", `R3/out/val/joint_limits_*.out`). `hinge-limits`, `spatial-path`, `tendon-limits` are demos (compile-checked, not run by CI).

### 2.6 Tests to add (fail on main; measured)
1. `hinge_ref_fk_matches_mujoco`: `X6_hinge_ref.xml`: `xquat` at qpos0 equals the body's MJCF orientation (identity); 100-step qpos vs MuJoCo golden 1e-12. Main 0.246.
2. `slide_ref_fk`: a slide with `ref` (none exists in-tree; MuJoCo literal at qpos0).
3. `offset_ball_matches_mujoco`: `X2_ball_offset.xml` and `chain_offball.xml`, 100 steps vs golden 1e-12; transition A vs MuJoCo FD 1e-8 (needs L36a for the chain case). Main 0.526 / 0.457.
4. A14's X1/X3/X5 `xaxis`/`xanchor` literals (they belong to this commit now).
5. `spatial_lengthspring_at_springref`: `251c5904`-shaped doc, `tendon_lengthspring` equals MuJoCo's; main 0.253 qpos after 100 steps.
6. Promote `muscle_qpos0_ref` into A11's LR test list (after LR-a).

### 2.7 Downstream
`qpos0` producers outside sim-mjcf: cf-design sets hinge/slide `qpos0 = 0` (`design/cf-design/src/mechanism/model_builder.rs:662-681`), so its FK is unchanged; factories set 0. Readers of `xaxis`/`xanchor` for ball/free: none (A14 §3.2). sim-gpu needs the shader change in the same commit (not run). sim-bevy renders from `xpos/xquat` (not run).

---

## 3. Ball/free position block of the transition matrix (A14 §4.3)

### 3.1 First differing quantity
A14 measured our **FD** `A` vs MuJoCo's `mjd_transitionFD` (so the dynamics are not the cause): pos-rows × pos-cols 9.9e-4, pos-rows × vel-cols 2.0e-6, velocity rows ≤ 6.8e-10. Re-measured (`R3/out/tr_*`, eps 1e-6 centred, our probe `trans` vs `scripts/mj_trans.py`): ball_only 7.89e-4, hinge_ball 7.68e-4, F8 7.81e-4, **free (X3) 8.00e-4** — free joints have it too.

### 3.2 Cause
- **MuJoCo** `mjd_stepFD` perturbs a position column with `mj_integratePos(qpos, e_i, ±eps)` (`engine_derivative_fd.c:479-503`) and differences the two **perturbed next states directly**: `stateDiff(next_minus, next_plus, 2eps)` (`:513-515`) → `mj_differentiatePos(m, ds, h, s1, s2)` (`:55-65`; `engine_support.c:608-638`) → `mju_subQuat` = `quat2Vel(q₁⁻¹q₂)` (`engine_util_spatial.c:117-141`). The output tangent lives at the next state.
- **Ours** `mjd_transition_fd` (`derivatives/fd.rs:106,154,181`) stores each next state as a tangent relative to **`qpos_0`, the pre-step state** (`extract_state`, `:378-397`, `mj_differentiate_pos(qpos_ref = qpos_0 → next)`), then subtracts those tangents (`:189,199,267`). For a quaternion the two conventions differ by the SO(3) Jacobian of log(q₀⁻¹q_next) — I + ½[ωh]× + …, i.e. ~½|ω|h ≈ 8e-4 at |ω|≈0.8 rad/s, h = 0.002. The hybrid path matches our FD on purpose: `mjd_quat_integrate` (`derivatives/integration.rs:20-77`) returns J_l⁻¹(θ) and h·I "for the FD tangent convention" (`:51-75`), although its header documents MuJoCo's exp(−[θ]×) and h·J_r(θ) (`:10-17`).
- A second, smaller defect shows once the outputs are differenced directly: `mj_differentiate_pos` returns exactly 0 for any rotation with sin(θ/2) ≤ 1e-10 (`jacobian/position.rs:88`, `:127`). MuJoCo's `mju_quat2Vel` has no such clip (floor mjMINVAL in `mju_normalize3`) and wraps angles above π (`:123-125`); ours does not wrap. Cross-coupling entries of an FD column are rotations of ~1e-12 rad: with the convention fixed but the clip kept, `hinge_ball` pos-rows × vel-cols went 1.5e-6 → 1.0e-5 (measured) before the clip was removed.

### 3.3 Change (`R3_BALLA`)
- `fd.rs`: position rows of every `A` and `B` column = `mj_differentiate_pos(q_minus → q_plus)/(2eps)` (centred) or `(q_0next → q_plus)/eps` (forward) and the clamped one-sided variants; velocity/act rows unchanged. (`extract_state`'s position half becomes unused.)
- `integration.rs::mjd_quat_integrate`: return `(exp(−[θ]×), h·J_r(θ))` — MuJoCo's convention (output tangent at q_new: q·exp(ξ)·exp(θ) = q_new·exp(Ad_{exp(−θ)}ξ)). This is a **public function whose values change** (exported, `lib.rs:284`); its body's comments `:51-75` are rewritten.
- `jacobian/position.rs::mj_differentiate_pos`: port `mju_quat2Vel` exactly (norm floor 1e-15 with axis x, angle wrap > π). Public; values change only for rotations < 2e-10 rad and for |angle| > π.

### 3.4 Measured
| fixture | FD vs MuJoCo FD, all blocks | hybrid vs MuJoCo FD | hybrid vs our FD |
|---|---|---|---|
| ball_only | 7.89e-4 → 1.67e-10 | 7.89e-4 → 9.4e-11 | 1.5e-10 → 1.0e-10 |
| hinge_ball | 7.68e-4 → 2.2e-10 | 7.68e-4 → 2.1e-10 | 2.7e-10 → 2.7e-10 |
| X3_free | 8.00e-4 → 7.8e-11 | 8.00e-4 → 8.3e-11 | 1.5e-10 → 1.2e-10 |
| F8 (offset hinge → ball) | 7.81e-4 → 2.8e-10 | velocity rows 4.03e-4 both (A14's L36a family); with L36a ≤ 4.9e-10 | |

- Bit identity (`fpd_balla`, derivative fingerprints at the state after 100 steps): trajectories, sensors, models 0 changes; derivative bits change in 683 docs: 601 with a ball/free joint (the intended change) and 82 hinge/slide-only docs, where the FD now subtracts outputs directly instead of differences of differences: max |ΔA_fd| 2.3e-13, |ΔA_hybrid| 1.1e-16 (all 82 measured at qpos0, e1).
- Census: no change (the census has no derivative step). Suites: 0 flips. Validator `derivatives` rc 0; one printed value: "Ball: hybrid vs FD A … max rel err = 4.94e-8 → 9.02e-8".

### 3.5 Tests to add
1. `ball_transition_matches_mujoco_fd`: ball_only, hinge_ball, X3_free at the probe states (`R3/out/tr_*_mj.json` as goldens), FD and hybrid `A` vs MuJoCo 3.5.0 at 1e-8. Main 7.9e-4 / 7.7e-4 / 8.0e-4.
2. `differentiate_pos_tiny_rotation`: q₂ = q₁·exp(1e-12·e_x): result 1e-12 (main 0). And a w < 0 pair → the short way round.
3. `quat_integrate_jacobians`: `mjd_quat_integrate` vs FD of q·exp(ξ)·exp(θ) in the q_new tangent (1e-9).

### 3.6 Downstream
- sim-coupling uses `transition_derivatives` only for single hinges and hinge chains (`sim/L1/coupling/src/articulated.rs:410-424`, `:546`), with `DerivativeConfig::default()` (`use_analytical: true`, `derivatives/mod.rs:195`); on the corpus's hinge/slide-only docs the change is ≤ 1.1e-16 hybrid and ≤ 2.3e-13 FD (coupling's own suites not run). It calls `mj_differentiate_pos` in its own tangent FD (`:300`), which already uses MuJoCo's convention and gains the exact tiny-rotation values. `docs/keystone/quaternion_joints_recon.md:116-119` ("do NOT reconcile") and `vjp.rs:127` (its own `right_jacobian_so3` because sim-core's had "its own convention") become stale. Not run.
- `examples/fundamentals/sim-cpu/derivatives/stress-test/src/main.rs:849` comment refers to the old convention.

---

## 4. framequat body (1), RK4 + mocap (2), implicitfast + fluid (1)

### 4.1 framequat `objtype="body"` (`f0b36671`, `sensor_phase6.rs:220`) — hand-off to A10 MH2
Both engines compute `xquat ⊗ body_iquat` (`engine_sensor.c:129-131`; ours `sensor/position.rs:173-176`). The model differs: ours `body_inertia (0.0582, 0.0015, 0.0582)`, `iquat (0.788, 0, 0, 0.615)`; MuJoCo `(0.0582, 0.0582, 0.0015)`, `(0.557, 0.557, 0.435, 0.435)` — same tensor, different principal-axis order (`R3/out/fq_*.json`). With MuJoCo's inertia/iquat forced in (`R3P_BODY`), framequat agrees 0.557 → 2.3e-15 over 100 steps (`out/fqb_*.jsonl`). A10's eig3 prototype gives MuJoCo's values for this doc bit for bit (`mesh_inertia_hull/out/corpus_parity.jsonl`, k=1475). **No sensor change**; the census doc agrees once Rigid-loading's MH2 lands.

### 4.2 RK4 + mocap (`70b20f04` `equality_constraints.rs:1232`; `e4fd2386` `mocap-bodies/tilt-drop:39`) — not RK4, not mocap; hand-off to collision
- `70b20f04` (weld between a free box and a mocap box in the same place): at MuJoCo's RK4 stage-1 state our plain `forward()` already differs (qacc 2.3e-2, `out/st1_*`). The same doc under Euler, and with the mocap body made static, gives the same first differing quantity one step later: **box–box contacts** (con_dist 1.6e-9 relative, contact order permuted, MuJoCo's normal tilted (1.79e-4, 0, 1) with the moving box, ours (0,0,1); `out/rk4m_cmp.jsonl`). With the boxes' contacts disabled, RK4 + mocap + weld agrees to 5.6e-16 qpos over 100 steps (`rk4_mocap_weld_nocon`).
- `e4fd2386` (sphere on a mocap platform): Euler variant first differs at step 42, `con_pos`: our contact point is offset by exactly |dist| (2.3372e-4) along the normal, and the geom order is swapped (MuJoCo sphere-first) — the census's NEW-POS cluster (`out/tilt_cmp.jsonl`).
- Recommendation: relabel both docs into the collision clusters (box–box near-coincident; NEW-POS). Nothing to change for RK4 or mocap.

### 4.3 implicitfast + ellipsoid fluid (`d58bb68a`, `fluid_derivatives.rs:224`)
- **First differing quantity:** `qvel` after step 1, while qacc, `qDeriv` (≤ 4.3e-19) and M agree. MuJoCo's qvel₁ matches neither our (M − hD)⁻¹ prediction nor explicit Euler (`out/ifd_*`). `m.M_rownnz = [1,1,1,1,1,1]`, `nC = 6`: MuJoCo stores this body's M **diagonal-only**.
- **Cause.** MuJoCo implicitfast gathers qDeriv into `qH` through `mapD2M` ("symmetric to lower", `engine_io.c:943-944`; `engine_forward.c:1186-1189`), i.e. only the entries of M's *reduced* sparsity pattern: ancestors, and for "simple" DOFs (`body_simple`, `user_model.cc:2750-2860`; `dof_simplenum`, `:3942-3962`) the diagonal only. Full implicit uses D's pattern = tree ancestors/descendants (`qLU`). MuJoCo documents the restriction ("we restrict D to have the same sparsity pattern as M … will exclude damping in tendons which connect bodies that are on different branches", `doc/computation/index.rst:544-546`). Ours uses the dense symmetrized D for implicitfast (`forward/acceleration.rs:292-306`) and the dense D for implicit (`:336-353`).
- **Change (`R3_IFLUID`).** Mask D when forming `M − hD` (not in `data.qDeriv`, which MuJoCo keeps unmasked and the hybrid derivatives re-read): implicitfast keeps lower entries (i ≥ j) with j an ancestor of i and DOF i not simple, mirrored; implicit keeps ancestor-or-descendant pairs. The prototype computes `body_simple` on the fly (`r3_dof_simple`); the commit should make it a derived `Model` field (MuJoCo's `dof_simplenum`) in K6's `recompute_derived`.
- **Measured.**
  | fixture | base | `R3_IFLUID` |
  |---|---|---|
  | `ifluid` (= `d58bb68a`) 100 steps | 1.33e-5 qpos | 5.6e-16 |
  | `tendon_xbranch_implicit` (fixed tendon with damping across branches) | 5.81e-2 | 4.4e-16 |
  | `tendon_xbranch_implicitfast` | 5.82e-2 | 2.2e-16 |
  | `tendon_xbranch_Euler` (control) | 1.1e-16 | 1.1e-16 |
  | `fluid_chain_implicitfast` (non-simple bodies) | 3.4e-15 | 3.4e-15 |
  Hybrid vs our FD: unchanged for the tendon cases (≤ 9e-11); implicitfast+fluid stays ~6e-4 before and after (the hybrid drops ∂D/∂v; pre-existing, MuJoCo has no analytic counterpart).
- **Simple-DOF flags vs MuJoCo** over the 1,214 both-load docs: 1,206 equal, 8 differ (`out/simple_*.tsv`), all docs whose `body_ipos/iquat` differ from MuJoCo's (mesh re-centring, principal-axis order, L43c); none of the 8 uses implicitfast. So the flags depend on A10's MH2 and mesh-frame work.
- Bit identity (`fpd_ifluid`): trajectories change in 2 docs only, both implicitfast + fluid (`fluid_derivatives.rs:224`, `:298` — the latter 2.5e-12 → 6.3e-16 vs MuJoCo, under the census threshold); derivative bits in those + `:2768`. Census: `d58bb68a` → agree, 0 regressions. Suites 0 flips; `t15_implicitfast_b_symmetric` and `t17_implicitfast_mujoco_conformance` still pass because `qDeriv` itself is not masked.
- **Tests:** `implicitfast_fluid_matches_mujoco_3_5_0` (`ifluid.xml`, 100 steps, golden 1e-12; main 1.33e-5); `implicit_cross_branch_tendon_damping` (`tendon_xbranch_{implicit,implicitfast}.xml`, golden 1e-12; main 5.8e-2).

---

## 5. Sleep under RK4: why MuJoCo wakes the tree, and a fix with tests

### 5.1 What MuJoCo does (measured by intervening in MuJoCo itself)
`R3/scripts/mj_rk4_wake.py` steps `sleep_parity/models/box_rest_RK4.xml` (3,000 steps) and, after every step on which the tree is asleep, optionally (a) overwrites the stored poses of the tree's bodies with FK(qpos) from a scratch `MjData`, (b) zeros `qacc_smooth` of its DOFs:

| intervention | transitions | first | steps with stored pose ≠ FK(qpos) |
|---|---|---|---|
| none | 584 | 83, 84, 93, 94 | 292 |
| (a) poses | 100 | 83, 84, 142, 143 | — |
| (b) qacc_smooth | 584 | 83, 84, 93, 94 | — |
| (a)+(b) | **1** | 83 | — |

So two mechanisms wake the tree, and together they are the whole cause on this fixture:
1. **Stale poses.** `mj_RungeKutta` leaves the position-stage quantities of its last stage (state X[0] + h·k₃) in `mjData`, resets qpos/qvel/act/time to the step's start (`engine_forward.c:1114-1118`) and calls `mj_advance` (`:1121`), whose sleep re-forward skips the position stage (`mj_forwardSkip(m, d, mjSTAGE_POS, 0)`, `:899-905`). The slept tree keeps its stage-4 pose while its qpos stays at the start, and the next `mj_kinematics1` sees a mismatch and marks the tree to wake (`engine_core_smooth.c:163-179`). Measured: stored `xpos_z` 0.09989117 vs FK 0.09989108 (A8 `py/rk4_wake.py`). Under Euler and implicit the slept tree's qpos is the one its pose was computed from (the step's own forward); measured: 0 pose violations on 10 Euler/implicit fixtures (below).
2. **Stale `qacc` drives the stages.** A sleeping DOF's `qacc` is its last awake `qacc_smooth` (A8 SP-7; measured up to 2.7e-3 here). RK4 builds every stage state for all DOFs (`mj_integratePos(m, X[i], dX, h)`, `mju_addToScl(X[i]+nq, dX+nv, h, nv+na)`, `:1086-1090`), so the sleeping tree gets a nonzero stage velocity, and the stage `mj_forwardSkip` (`:1100`) runs `mj_wake`, whose `treeCanSleep(…, 0)` requires every qvel byte zero (`engine_sleep.c:125-152`, `:150`).

**Is it a defect?** By MuJoCo's own rules yes, twice: its wake test assumes a sleeping body's stored pose equals FK(qpos) (`:163-179`; docs: a tree wakes when its qpos is changed), and `mj_advance` itself never moves a sleeping tree (velocity update over awake DOFs, positions of awake bodies, `:907-918`) while the RK4 stages do. A third consequence is visible on awake trees: the sleep-step re-forward computes velocity/acceleration quantities at stage-4 positions with start-of-step velocities. `R3/xml/rk4_two.xml` (the resting box plus an independent pendulum with `sleep="never"`): on the box's sleep step the pendulum's velocimeter/framelinvel/accelerometer differ by **0.116** from MuJoCo's own run of the same model with the box's sleep disabled, although the pendulum's trajectory is identical (0.0).

### 5.2 Fix (prototype on A8's S3 workspace, `R3_RK4SLEEP`, `R3/out/prototype_rk4sleep.diff`)
In `integrate/rk4.rs` of A8's prototype (A8 already moved sleep into the RK4 advance):
1. On a step where `mj_sleep` puts trees to sleep, re-forward from the **position** stage (`forward_skip_unchecked(model, MjStage::None, false)`) instead of skipping it, because qpos/qvel/time were just reset to the step's start and every position-stage quantity is stage-4's.
2. In the stage loop, set `dX_vel = dX_acc = 0` for DOFs of sleeping trees, so their stage states equal the step's start bit for bit (MuJoCo's `mj_advance` already treats them as frozen).
Euler and implicit paths are untouched.

### 5.3 Measured
- `box_rest_RK4`, 3,000 steps (`out/rk4sleep_{0,1}.jsonl`): A8's parity prototype 584 transitions, 292 pose violations; with the fix **1 transition (step 83, as MuJoCo) and 0 pose violations**.
- **Ours-with-fix equals MuJoCo-with-both-interventions**: 0 sleep-state disagreements over 3,000 steps, qpos ≤ 9.8e-20, qvel ≤ 2.3e-18, sensors ≤ 8.7e-19 (`out/rk4w_mj_both.jsonl`). A8's parity prototype vs unmodified MuJoCo: 0 disagreements, 1.3e-13. Same on `chain_RK4` (hinged chain, 6,000 steps): MuJoCo 22 transitions; ours-with-fix 1 (step 5,897), equal to MuJoCo (a)+(b) at 2.2e-16.
- `rk4_two`: the pendulum's sensors on the sleep step equal our own no-sleep run **bit for bit** and MuJoCo's no-sleep run to 7.1e-15; A8's parity variant reproduces MuJoCo's 0.116 to 1.1e-14.
- Non-RK4 sleep fixtures (A8's `box_rest`, `box_rest_implicit`, `box_rest_implicitfast`, `swake`, `sstack`, `eqpair`, `limit`, `chain2`, `rbox`, `act_sleep`): traces byte-identical with the switch on and off; pose violations 0 in both; transitions equal A8's MuJoCo table.
- Corpus, A8's harness with sleep forced on every doc (`R3/sleepharness`, `out/sleepcorpus_*.jsonl`, 3 runs): only **1** doc changes outside the run-to-run mask (`mocap-bodies/drag-target:41`, RK4, forced sleep: 18 transitions → 1, first sleep at 114 in both); the 3 other diffs are `nondet71` flex docs. The one in-tree RK4 doc that enables sleep (`d22fcd34`, `sleeping.rs:116`, a ball falling with no floor) never sleeps in 3,000 steps in either engine and agrees at 2.8e-14 with or without the fix.
- A8's prototype integration suite: per-test outcomes identical with the switch on and off (1,321 pass / 17 fail — A8's own listed flips, `out/sleepws_integ_*.out`); its sleep-wake validator output byte-identical.

### 5.4 Target, deviation, tests
- **Target:** under RK4 a sleeping tree stays asleep until a wake rule fires, and the sleep step's re-forward evaluates the step-start state. **Deviation: lenient** (MuJoCo wakes a tree nothing touched; we keep it asleep), listed in the divergences table with the tests below. A8's Q2 option (a) "parity" is replaced; its sleep timing parity is unchanged.
- **Tests** (each fails on A8's S3 = the parent; RK4 sleep is disabled by the builder on main, so they fail on main too):
  1. `rk4_sleeping_tree_stays_asleep`: `box_rest_RK4.xml`, 3,000 steps: exactly one transition, at step 83 (MuJoCo's first sleep step). Parent: 584.
  2. `sleeping_pose_equals_fk_of_qpos` (all integrators): after every step, for every sleeping body, stored `xpos/xquat` == FK(qpos) bitwise (FK on a clone with sleep disabled). Parent: violated 292 times under RK4; 0 under Euler/implicit (so the test is integrator-generic).
  3. `rk4_sleep_step_does_not_disturb_awake_sensors`: `rk4_two.xml`: on the box's sleep step, the pendulum's sensordata equals the same model with the box `sleep="never"`, bitwise. Parent: 0.116.
  4. `rk4_sleep_matches_mujoco_without_its_wake_defects`: optional golden from `mj_rk4_wake.py both` (3,000 steps at 1e-12); it pins that the deviation is exactly the two defects and nothing else.

---

## 6. Commit list (in order; per-commit greenness was not run — only the full stack)

All are **Rigid-physics** (sim-core, plus sim-gpu where noted) unless marked.

| # | commit | contents | depends on |
|---|---|---|---|
| R1 | `fix(sim-core): body accumulators match mj_rnePostConstraint` | §1.5 (free-joint term via a helper shared with `rne.rs`; `cfrc_ext` at `xpos`; connect/weld forces); `Data::cfrc_ext` doc; tests §1.7 1–5 | K13 (edits the same loop; the free term must sit after K13's `v_partial`); the xfrc type change (swap order in Step 1); Q1.1 decides test 5 |
| R2 | `fix(sim-core): forward kinematics as mj_kinematics1 (qpos − qpos0, off-centre ball, xaxis/xanchor)` | §2.3 FK, velocity, subspace, sleep FK copy, sim-gpu `fk.wgsl` (+ ball on GPU or GPU refusal, Q2.1); takes over A14's L36b core commit | A14's L36a for non-root offset-ball derivatives (measured 2.8e-2 without it), so L36a first or same series; A8 S1–S4 rewrite `island/sleep.rs` (textual) |
| R2b | `fix(sim-core): spatial tendon spring length resolved at qpos_spring` | §2.3 last bullet, test §2.6 5 | none (K6 lists `compute_spatial_tendon_length0` in `recompute_derived`) |
| R3 | `fix(sim-core): implicit integrators restrict D to M's sparsity (MuJoCo qH/qLU)` | §4.3; derived `dof_simplenum` (or `body_simple`) | K6 (derived field), A13 D2 (same functions), D2's `qacc_implicit`; simple flags agree with MuJoCo only after A10 MH2 + mesh re-centring (Rigid-loading) |
| R4 | `fix(sim-core): transition derivatives use MuJoCo's tangent convention for quaternion joints` | §3.3 (`fd.rs`, `mjd_quat_integrate`, `mj_differentiate_pos`); doc updates incl. `docs/keystone/quaternion_joints_recon.md` | K8/K9 edit `fd.rs` (textual); A14 L36a for F8-type hybrid agreement |
| R5 | `fix(sim-core): RK4 keeps a sleeping tree asleep` (body: lenient deviation) | §5.2, tests §5.4 | A8 S1–S3 (sleep in the advance), K5 (RK4 tail) |
| — | Rigid-loading | ball/free `jnt_axis = (0,0,1)` (with A14's M17 addition) | — |
| — | docs (M26 / K14) | divergences table: static-body accelerometer (if Q1.1 = keep), `cfrc_int[0]` reference (if Q1.2 = keep), RK4 sleep (lenient) | — |

Hand-offs: NEW-FRAMEQUAT → A10 MH2 (Rigid-loading); NEW-RK4MOCAP → the collision area (box–box near-coincident contacts; NEW-POS). The census gate's ratchet needs a way to mark listed deviations (`809fc2ad` under Q1.1 = keep).

## 7. Open questions (not settled by the fixed decisions)

**Q1.1 Static-body acceleration.** (a) Keep ours (+g; lenient deviation, test §1.7 5); (b) MuJoCo's quick return (0; needs `body_weldid` derived for every producer, K6). *Recommend (a)*: MuJoCo documents the accelerometer as including gravity, and its own value for the same stationary site flips with the presence of a zero-velocity joint (§1.4). If wrong: `809fc2ad` stays a listed census difference and users porting from MuJoCo see +g where MuJoCo shows 0 on welded bodies (and mocap bodies, which MuJoCo also treats as static).

**Q1.2 World-body `cfrc_int[0]`.** (a) Keep ours (transported to the world origin), list it; (b) copy MuJoCo's untransported sum. *Recommend (a)*: MuJoCo's torque mixes reference points (`engine_core_smooth.c:2676-2679` adds root wrenches referenced at different subtree COMs). If wrong: a torque sensor on a world site differs from MuJoCo (not measured on any in-tree doc; 12.4 on `acc_free`'s `cfrc_int[0]`). Test for (a): torque on a world site equals Σ child wrenches moved to the site.

**Q2.1 Offset ball on the GPU.** (a) Implement in `fk.wgsl` + the GPU velocity/RNE subspace; (b) sim-gpu refuses a model with a ball `jnt_pos ≠ 0`. *Recommend (b) in Rigid* (0 in-tree uses; nothing GPU was run here), CPU implemented as measured. If wrong: GPU users with offset balls get a refusal instead of support.

**Q3.1 Quaternion `A` convention is a public behaviour change.** `mjd_transition`/`transition_derivatives` position rows and `mjd_quat_integrate` change for ball/free joints. *Recommend* MuJoCo's convention (it is also the one that composes along a trajectory, and sim-coupling's own FD already uses it). If wrong: callers that relied on the old q₀-tangent rows (none found by grep outside sim-core) see ~½|ω|h changes.

**Q5.1 Frozen-stage snapshot.** A tree woken *during* an RK4 stage (e.g. by a stage contact) — keep it frozen for the rest of the step (snapshot of awake DOFs at the step's start) or let later stages move it with the earlier stages' stale `F[j]`? The prototype re-reads the live flags each stage. *Recommend the snapshot*: no stale accelerations enter the integration. Not measured (no fixture wakes a tree mid-step).

## 8. What this method cannot see
- **Not run:** sim-gpu (no adapter run; the `fk.wgsl` change is not written), sim-urdf/sim-thermostat suites (built only), cf-design, sim-coupling, sim-rl/opt/ml-chassis, L1 crates, Bevy demos, doctests (the scratch rename breaks them in both variants), `grade`, clippy on the prototype, licensed gates, `--features mjb`, `--no-default-features`.
- **Excitations:** census e1/e2 and my fixtures' states only. The census still cannot see derivatives (§3 measured by fixtures + derivative fingerprints), flex docs, or runtime MJCF.
- **Per-commit greenness** in §6's order: not run; the switches were measured singly and all together.
- **The RK4 sleep fix** was prototyped on A8's prototype (not on my base) and measured on 3 RK4 fixtures + the forced-sleep corpus; trees woken mid-step (Q5.1) and RK4 with multi-tree islands were not exercised.
- **Simple-DOF flags** were checked against MuJoCo on the corpus (1,206/1,214); the tendon-armature demotion rule (`user_model.cc:3925-3940`) has no counterpart because our Model has no tendon armature.
- `cfrc_ext` changes (§1.3) have no census doc; their tests rest on four fixtures.
- One base integration run failed `collision_performance::scaling_collision_bodies` (a timing ratio, 14.3× > 10×, under load); it passed in the two variant runs.

**Repo state.** Before: HEAD `fbc0ad54` on `fix/rigid-sim-core-mjcf`, `git status --short` empty. After: HEAD `fbc0ad54`, `git status --short` empty; the repo's `MUJOCO_LOG.TXT` predates this session (19:22). Every MuJoCo run used `R3/out` as its working directory. Scratch `target/` directories and the copied test assets were deleted; nothing left running.
