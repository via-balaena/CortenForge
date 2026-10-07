> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# r3 — constraint and solver divergences from MuJoCo 3.5.0 (Newton, CG, late onset, solreffriction, dof-less equalities, weld relpose/torquescale)

Researcher section, round 3, 2026-10-06. Repo read-only at HEAD `fbc0ad54` (branch `fix/rigid-sim-core-mjcf`; source = `main` @ `3520544e`, only spec docs differ). Scratch: `SCRATCH/rigid_spec/r3_constraint_solver/` (written `R3/` below). Oracle: `mujoco==3.5.0` (`SCRATCH/rigid_oracle_350/venv`, arm64); MuJoCo citations are `SCRATCH/mj350src/mujoco` (tag 3.5.0). Doc ids are from `SCRATCH/rigid_spec/parity_census/out/labels.json` (`fixB_all`).

**Prototype base.** `R3/core` is a copy of the census's `core_fix_b` (L32 fix, the A13 prototype behind `ISO_IMP`/`ISO_FIX`/`ISO_FLEX`, the L42 fix, the `ISO_FRAME` probe); `R3/mjcf` is the repo's sim-mjcf pointed at it. Census runs use `ISO_IMP=1 ISO_FIX=2 ISO_FLEX=2` (what A16 calls "all prototypes"); test-suite runs use `ISO_IMP=1 ISO_FIX=2` (A13 measured `ISO_FLEX=2` alone failing 3 flex tests). Every fix below sits behind a scratch env switch in that copy, so one build serves base and fix (`R3/out/prototype_r3_core.diff`, 801 added lines, 649 of them the port in `constraint/solver/primal_mj.rs`; `R3/out/prototype_r3_mjcf.diff`):

| switch | what | item |
|---|---|---|
| `R3_HESS=3` | incremental Hessian uses MuJoCo's `mju_cholUpdate` for both add and remove (minimal fix, old solver otherwise) | 1 |
| `R3_PRIMAL=1` | Newton and CG replaced by a port of MuJoCo's `mj_solPrimal` (see §4) | 1, 2 |
| `R3_ISLAND=1` | with the port: one solve per constraint island, MuJoCo's partition and scale | 1, 2 |
| `R3_PLANEBOX=1` | box–plane emits only corners within margin (MuJoCo's per-corner test) | 3 |
| `R3_EMPTY=1` | a non-contact constraint whose Jacobian rows are all zero adds no rows | 4b |
| `R3_SOLREF=1` | MuJoCo's `getsolparam` format checks for `solref` and `solreffriction` | 4a |
| `R3_ELLR=1`, `R3_ELLMU=1` | elliptic friction rows: MuJoCo's R rule (impratio, friction ratios) and regularised cone μ | 4a |
| `R3_DIAGADJ=1` | `efc_diagApprox` back-computed from the final R, as MuJoCo | API |
| `R3_MEANI=1` | PGS and noslip scale with the model's `stat_meaninertia` | — |
| `R3_RELPOSE=1` (sim-mjcf), `R3_TQ=1` (sim-core) | weld relpose position kept and quaternion normalised; weld torquescale | 4c, 4d |

## 0. Method and tools

Per the brief: confirm the fixture, find the first differing quantity, cite both sides, prototype in scratch, measure agreement, bit-identity elsewhere, flips.

| tool | what it does |
|---|---|
| `R3/probe` (`r3-probe`, `r3traj`) + `R3/scripts/mjprobe.py`, `mjtraj.py` | the census initial state (qpos0; qvel_i = 0.1(1+i mod 3); ctrl rule); `nprior` steps; optional `iterations`/`tolerance` override; forward on a clone; dumps solver statistics, efc arrays, contacts |
| `R3/scripts/samein.py` | **same input**: MuJoCo's exact state after k steps (qpos, qvel, qacc_warmstart, by `repr`) is loaded into ours, then both forward. Removes trajectory drift from any comparison |
| `pcmp.py`, `concmp.py`, `stepgap.py`, `trajcmp.py` | quantity-by-quantity, contact-by-contact, step-by-step comparisons |
| `mjsens_solver.py`, `sensitivity.py`, `mjneval.py` | MuJoCo-only: how much MuJoCo's own output moves under a 1-ulp change of one state entry |
| `R3/harness` | the census harness (A16) built against `R3/` crates; compared with the census's MuJoCo files `PC/out/mj.jsonl` / `mj_e2.jsonl` by A16's `compare.py`; verdicts by `R3/scripts/verdict.py` |
| `R3/bitfp` | **streaming** fingerprint harness: copy of `determinism_verification/harness/src/bin/harness2.rs` plus a per-step hash of qpos/qvel/act/sensordata/qacc bits; all 1,584 corpus docs in batches of 100 under the 2.5 GB RSS watchdog. Base run 10 times: 70 docs vary; apart from the relpose doc (an intended change), every other model-fingerprint change in any variant is `aa20ff29`, which is in the census's `nondet71.txt` |
| `R3/tests` | `sim-conformance-tests` (`integration`, `mujoco_conformance`, and `--ignored`) against `R3/` copies; sim-core lib and sim-mjcf lib tests run in the copies |
| `R3/validators` | the 20 validator examples whose deps are only sim-core/sim-mjcf/nalgebra, copied as binaries |

**The tools were made to report a difference before being trusted.** The census harness copy reproduced A16's first-differing steps at base (e.g. 406a1991 step 1, 443824ba step 90, 3810989d step 14, `R3/out/cmp_base.jsonl`). The bit harness reports 18 changed docs for `R3_PLANEBOX` and 1 for `R3_RELPOSE` (`R3/out/bitcmp_summary.txt`), so it can see a change. My verdict classifier gives 847 agree at base where A16's gives 850 for the same comparison; I did not reconcile the 3 docs and compare variants only against my own base.

## 1. Results at a glance

| cluster (docs) | first differing quantity | cause | measured after fix |
|---|---|---|---|
| Newton, identical contacts (9) | solver iteration 2: improvement 40.4 vs 136.1, gradient 22.9 vs 0.354 (406a1991, t0) | the incremental Hessian "downdate" adds the row back instead of removing it (`hessian.rs:643-658`) ↔ `HessianIncremental`, `engine_solver.c:1746-1808` with `mju_cholUpdate(…, flg_plus=0)`, `engine_util_solve.c:96-136` | 6 of 9 agree (≤ 8.1e-15 over 100 steps); 3 move to other clusters (§2.5) |
| CG after one step (2 census + A13 fixtures) | iteration count and stop point (weld CG: ours 8, MuJoCo 6) | a different algorithm: AND vs OR termination, PR β forced 0 once, PGS fallback, qacc recomputed from forces, different line search (§3) | 443824ba agrees (1.4e-8 → 1.1e-15); conn CG fixtures 6.9e-7 → 1.2e-14; the remaining gaps are at MuJoCo's own 1-ulp sensitivity (§3.3) |
| late onset (6) | at the onset step, with MuJoCo's exact state: **contact count** ours 4, MuJoCo 1 | box–plane emits all 4 bottom corners when the deepest is within margin (`collision/plane.rs:79-147`) ↔ per-corner `if (dist + ldist > margin …) continue`, `engine_collision_primitive.c:219-222` | 5 of 6 agree; 28695e44 moves to a box–box normal difference (§5) |
| `<pair solreffriction>`, elliptic (2) | `efc_aref` (d81df1fa) / `efc_R` of torsional and rolling rows (2cb92700) | two causes: no `solreffriction` sign check ↔ `engine_core_constraint.c:1340-1343`; elliptic friction rows copy the normal row's R ↔ `:1531-1548` | both trajectories agree (≤ 1.0e-15); both keep an API difference (`con_zero`, ledger-L41) |
| equality rows between dof-less bodies (5, API) | `nefc` 3/6 vs 0 | no empty-Jacobian guard ↔ `mj_addConstraint`, `engine_core_constraint.c:248-265`, `:306-309` | 5 of 5 agree; 0 trajectory bits change in the corpus |
| weld `relpose` (1, model) | `eq_data[3..6]` (0.5 vs 0) and unnormalised quaternion | builder recomputes the relpose position (`builder/equality.rs:183-197`) ↔ `engine_setconst.c:813-818` | model agrees; trajectory agrees to 5.6e-16 until the boxes first collide (step 33, box–box normal, §8) |
| weld `torquescale` (0 corpus docs) | — (unparsed) | not implemented ↔ `engine_core_constraint.c:482, :510, :530` | 5 fixtures, 500 steps: 2.8e-2…2.3 → ≤ 8.7e-15 |

**Census totals** (1,214 docs both engines load; `R3/out/census_summary.txt`), my base → every fix:

| | agree | state | API | model |
|---|---|---|---|---|
| excitation e1, base | 847 | 101 | 74 | 192 |
| e1, every fix in this section except plane–box | 859 | 91 | 73 | 191 |
| e1, every fix incl. plane–box (§5) | **867** | 84 | 72 | 191 |
| e2, base | 839 | 109 | 74 | 192 |
| e2, every fix except plane–box | 857 | 90 | 76 | 191 |
| e2, every fix | **864** | 84 | 75 | 191 |

No doc goes from agree to differ. Docs whose gap to MuJoCo grew by a decade or more: 6369216c (NEW-CONVEX; 6.7e-10 → 5.5e-8; it already differs at t0 in contact distance, A16) and, under e2 with plane–box only, the three copies of the two-box solver scene (aeb76329, e0554f47, faa6b270; 0.008 → 0.17 qpos; they differ at t0 in `nefc` because of ledger-L41's zero-distance contacts, §5.4).

**Also measured: the 24 `#[ignore]`d golden-flag tests now pass.** `sim/L0/tests/integration/golden_flags.rs` compares `qacc` with MuJoCo 3.4.0 golden data at 1e-8. At base 25 ignored tests fail (`R3/out/tests_ign_T0.log`); with `R3_HESS=3` alone, or with the port, 26 of 27 pass and only `collision_primitives::sphere_stack_dynamic` still fails (`tests_ign_T2.log`). `docs/KNOWN_GAPS.md:16-46` ("Gap 1", status open) says reaching 1e-8 "likely requires bit-level matching of MuJoCo's constraint solver … may be aspirational". That explanation is overturned by the measurement. What it supported: the 24 `ignore` reason strings (`golden_flags.rs:197`, `:218-355`), Gap 1's "not scheduled" status, and the planning appendices' treatment of these tests as an unchanging residual (A6 a7, A12 §5, A13 §1.7).

---

## 2. Item 1 — Newton with identical contacts (9 docs)

### 2.1 Fixture and first differing quantity
406a1991 (`collision_plane.rs:1076`), at t0, same input: `efc_J` ≤ 2.2e-16, `efc_aref` ≤ 1.1e-13, R and D equal (`R3/out/newton_406a1991.txt`); A16 found the contacts equal. MuJoCo's stats (island 0) and ours:

| iteration | MuJoCo improvement / gradient / nactive | base | `R3_HESS=3` | port |
|---|---|---|---|---|
| 1 | 850320.7085227252 / 21.3738 / 8 | identical | identical | identical |
| 2 | **136.11566162669902** / 0.35361 / 7 | 40.364 / 22.91 / 8 | 136.11566162669902 / 0.35361 / 7 | same |
| 3 | 0.09305058493257877 / 2.6e-15 / 7 | 19.64 / 7.73 / 8 … 100 iterations, then PGS | 0.09305058493257877 / 3.1e-13 | same |

Base Newton returns `MaxIterationsExceeded` here and the step is solved by PGS (`R3_DEBUG` output; the stats MuJoCo prints for it are PGS's).

### 2.2 Cause
- **Ours.** `hessian_incremental` (`sim/L0/core/src/constraint/solver/hessian.rs:620-662`) removes a row that left the quadratic state by negating `v = √D·J` and calling `cholesky_rank1_update` (`:646-657`). Since (−v)(−v)ᵀ = v·vᵀ, this **adds** the row again. Measured on random SPD matrices (`R3/scripts/chol_check.py`): the negated update reproduces H + vvᵀ to 3.6e-15 and misses H − vvᵀ by 0.14–0.83. The unused `cholesky_rank1_downdate` (`linalg.rs:160-213`, `#[allow(dead_code)]`) is also wrong: it misses H − vvᵀ by 0.016–0.37.
- **MuJoCo.** `HessianIncremental` calls `mju_cholUpdate(L, vec, nv, flag_update)` with `flag_update = 0` for a removal (`engine_solver.c:1761-1786`) and refactorises from scratch when the rank drops (`:1789-1795`). `mju_cholUpdate` handles both signs (`engine_util_solve.c:96-136`).
- **Consequences.** With a wrong Hessian, Newton takes many iterations or none converge; `MaxIterationsExceeded` then falls back to PGS (`constraint/mod.rs:425-433`). MuJoCo has no fallback (`engine_forward.c:739-813`). Over 100 steps the fallback fired once in 406a1991 and once in db7b2a4b, at t0. With `iterations=1` on 406a1991, MuJoCo returns Newton's first iterate; base and `R3_HESS=3` return a 1-iteration PGS solve, 170 off; the port is 2.8e-13 off.
- **Why the census's tight tolerance shrank the gap** (A16: at `tolerance=1e-15`, 4 agree and 5 shrink 5–1600×): with a wrong Hessian, Newton still descends, so a smaller tolerance buys more iterations toward the same optimum. Not measured further.

### 2.3 Fix and measured agreement
`R3_HESS=3` (port of `mju_cholUpdate`, `R3/core/src/linalg.rs` `chol_update_mj`) alone: 3810989d, 406a1991, 9b5189ae, cd84b523, db7b2a4b, dcfa08ab agree with MuJoCo over 100 steps at ≤ 8.1e-15 (`census_summary.txt`, `v0 → v1`; under e2 too). The port (§4) gives the same 6.

### 2.4 Bit-identity
`R3_HESS=3`: 156 of 1,408 deterministic corpus docs change trajectory bits; all 156 are in the port's 263 (`comm`, `R3/out/hess_changed.txt`). By their `solver` attribute the changed docs are Newton docs (default or explicit); none is a PGS doc.

### 2.5 The other 3 Newton docs
- **a0ff89f4** (`collision_edge_cases.rs:526`, thin box on a plane, zero gravity): after the fix it first differs at step 3. With MuJoCo's state after step 2, ours emits 4 box–plane contacts (3 at dist +1.5e-5 … +3.7e-4, margin 0), MuJoCo 1 (`late_onset_contacts.txt`). That is §5's cause; with `R3_PLANEBOX=1` it agrees (5.6e-16).
- **c31a6ac0** (`collision_plane.rs:459`, tilted capsule on a plane): differs at step 1 in the contact tangent frame (2.8e-4). That is NEW-FRAME's plane–capsule axis hint (`engine_collision_primitive.c:81,84`), which `ISO_FRAME=1` does not cover (A16). Handed off.
- **87d99b20** (`sim/L1/bevy/examples/collision_shapes.rs:31`, 8 free bodies): after the fix t0 agrees and the trajectory first differs at step 29. Measured: at step 1 the states differ by 4.6e-16 (qvel); given our state, MuJoCo's collision reproduces our contacts exactly (`samestate.py`, 0 difference); and in MuJoCo itself a **1-ulp change of one qpos entry (the wide cylinder's quaternion) moves its plane contacts by 1.0e-12** (`87d99b20_sensitivity.txt`). No formulation difference was found; the measured facts are a roundoff-level state difference and a contact that amplifies such differences by 10⁴ in MuJoCo too. Islands do not change it: with `R3_ISLAND=1` our island 0 statistics equal MuJoCo's to 10 digits and the t0 qacc gap is 4.7e-12 (vs 4.3e-12 monolithic).

---

## 3. Item 2 — CG differs after one step (ledger-L44)

### 3.1 First differing quantity
A13's `weld_free_Euler_CG.xml`, t0, same input: MuJoCo stops after 6 iterations, ours after 8; qacc 2.2e-6 apart (`R3/out/cg_weld.txt`). `conn_free2_Euler_CG.xml`: MuJoCo 4, ours 2; 6.4e-12. 443824ba (`noslip.rs:201`, CG + elliptic sphere): first differs at step 90.

### 3.2 Causes (each cited; their individual shares were not separated — the port changes all at once)
| ours | MuJoCo 3.5.0 |
|---|---|
| terminates when `improvement < tol && gradient < tol` (`cg.rs:294`) | `improvement < tol \|\| gradient < tol` (`engine_solver.c:1920`) |
| β forced to 0 after the first line search (`cg.rs:256-258`); β = 0 when the denominator < MINVAL (`:262`) | Polak–Ribière every iteration, denominator `max(MINVAL, …)`, β < 0 → 0 (`:1927-1933`) |
| a pre-loop exit when the scaled gradient is already small (`cg.rs:130-135`; Newton `newton.rs:141-147`) | none: the line search always runs (`:1874-1883`) |
| not converged → PGS (`constraint/mod.rs:436-440`) | none |
| after CG, `qacc` is recomputed as M⁻¹(qfrc_smooth + qfrc_constraint) because `newton_solved` is false (`forward/mod.rs:569`) | the primal iterate is `d->qacc` |
| line search: doubling bracket, best-\|derivative\| refinement, no cost (`primal.rs:399-553`, `primal_eval` `:256-397`) | Newton steps from p1, three-candidate bracket with costs (`PrimalSearch` `:1328-1512`, `updateBracket` `:1298-1325`, `PrimalEval` `:1160-1295`) |
| `ls_iterations.max(20)` loops per phase (`cg.rs:139`, `newton.rs:151`) | `ls_iterations` bounds the count of every evaluation (`:1293`, `:1422`, `:1458`) |
| scale 1/trace(M(q)) per step (`constraint/mod.rs:67-78`, `cg.rs:42`) | island: 1/Σ diag of the island's M (`:1864-1870`); monolithic: 1/(m->stat.meaninertia·max(1,nv)) (`:1863`) |
| warmstart taken when `cost_warmstart < cost_smooth` (`cg.rs:79`, `newton.rs:93`) | smooth taken when `cost_warmstart > cost_smooth`, ties keep the warmstart (`engine_forward.c:665`) |

### 3.3 Measured with the port
- `conn_free2_Euler_CG`, `conn_free2_implicitfast_CG`, `conn_pend_Euler_CG`: 500 steps ≤ 1.2e-14 qpos (base 6.9e-7, 6.9e-7, 3.6e-13); iteration counts equal MuJoCo's in the four steps printed.
- 443824ba: agrees over 100 steps (1.1e-15; base 1.4e-8).
- `weld_free_Euler_CG`: iterations now 6 = 6. Per-iteration iterate gap with identical input: 1.2e-15, 1.8e-15, 2.7e-12, 1.06e-8, 1.1e-4, 3.6e-6 (iterations 1–6, `cg_weld.txt`). MuJoCo's own response to a 1-ulp qpos change on this fixture is 6.8e-8 (scaled qacc); our final gap is 1.2e-7 scaled, the same order. The iteration statistics agree to 9 digits through iteration 3, so no formulation difference was found; one source of arithmetic differences is §3.4.
- 60c23753 (`cg_solver.rs:561`, frictionless pyramid, 4 identical rows, D = 4.8e10): first differs at step 32 (base 31). In the forward after step 30, with identical input, the first line search takes **4 evaluations in ours and 51 in MuJoCo**; MuJoCo's count stays 50–51 under every 1-ulp perturbation of qpos and qvel there. In the forward after step 31, a 1-ulp qpos change moves MuJoCo's own qacc by 6.9e-4 (scaled) (`fma_evidence.txt`).

### 3.4 Fused multiply-add in the oracle binary (measured)
- The arm64 slice of `libmujoco.3.5.0.dylib` contains 8,901 `fmadd`/`fmsub` instructions, 18 of them in `PrimalEval`, 6 in `mju_dot`, 2 in `mju_cholUpdate` (`otool -arch arm64 -tv`). The x86_64 slice of the same wheel contains 0 (`vfmadd*`, measured in this session). Rust does not contract `a*b + c`.
- Test: replacing the cost and `deriv[0]` expressions at the end of the port's `PrimalEval` with `mul_add` (`R3_FMA=1`) changes our evaluation count on 60c23753 (forward after step 30) from 4 to **51**, MuJoCo's count. The trajectory still first differs at step 32. A Python replica of `PrimalSearch` on MuJoCo's own arrays gives `d0 = 0` exactly with unfused arithmetic (4 evaluations) and `d0 = 5.2e8` with an exact fused multiply-add (`lsreplica_fma.py`).
- Consequence: bit-level parity with this oracle on Jon's arm64 machine is not reachable in general without emulating the C compiler's contraction choices; and golden data generated on arm64 and on x86_64 differ at roundoff level. Neither consequence is something this section fixes (open question Q6, hand-off to the census gate).

---

## 4. The change for items 1–2: a port of `mj_solPrimal`

**Recommended shape** (prototype `R3/core/src/constraint/solver/primal_mj.rs`, dense). One function serves Newton and CG, as in MuJoCo:
```rust
// constraint/solver/primal.rs (crate-private), replacing newton_solve, cg_solve_unified,
// primal_prepare/primal_eval/primal_search and hessian_incremental's dense path
pub(crate) fn fwd_constraint_primal(model: &Model, data: &mut Data, m_eff: &DMatrix<f64>,
                                    qfrc_smooth: &DVector<f64>, newton: bool);
```
- **Warmstart** as `engine_forward.c:611-683` (ties keep the warmstart).
- **Islands** (if Q1 is accepted): partition from the efc rows (each row joins the trees of its nonzero Jacobian columns, as `engine_island.c` `treeNext`/`treeFirst`/`findEdges` `:144-370`); dofs outside every island get `qacc_smooth` (`engine_forward.c:670-676`); scale 1/Σ diag(M_island). Islands are used when not `DISABLE_ISLAND`, `noslip_iterations == 0` and the model has tree data (`ntree > 0`); otherwise monolithic with scale 1/(`model.stat_meaninertia`·max(1,nv)). The tree guard is needed today: without it the prototype panicked loading a muscle doc whose lengthrange simulation runs on a Model with empty `dof_treeid` (f7d1a8a1, `parser/tests.rs:1551`; ledger-L31's tree gap).
- **Loop** `mj_solPrimal` `:1811-1969`; constraint update `mj_constraintUpdate_impl` (`engine_core_constraint.c:2394-2585`, including the cone Hessian on the cone's first row); Hessian `MakeHessian`/`FactorizeHessian` with the diagonal clamped at `mjMINVAL` instead of failing (`engine_util_solve.c:33-62`), `HessianIncremental` with `mju_cholUpdate(±)`, `HessianCone` keyed by row; line search `PrimalSearch`/`updateBracket`/`PrimalEval` with costs; `deriv[1] ≤ 0 → mjMINVAL`.
- **No PGS fallback**; `NewtonResult` (crate-private) deleted.
- **CG keeps its iterate** as `data.qacc` (the forward path must not recompute it).
- **`solver_niter = 0` and `solver_stat` cleared when `nefc == 0`** (`engine_forward.c:747-751`; ours returns early at `constraint/mod.rs:399-402` and keeps the previous step's count: measured on 28695e44 after step 3, ours 2, MuJoCo 0).
- **Sparse** (`nv ≥ 60`, MuJoCo `mj_isSparse` `engine_core_util.c:32-39`; ours `nv > NV_SPARSE_THRESHOLD = 60`, `hessian.rs:19`): keep `SparseHessian`, refactorising instead of incremental updates (mathematically `FactorizeHessian(recompute)`); the threshold becomes `≥ 60`. **No census doc has nv > 60**, so this path is unmeasured.
- **ImplicitSpringDamper** (our extension, MuJoCo refuses it): Newton keeps `m_eff = M_impl` as today; CG keeps `qM` and today's implicit velocity path. 0 corpus docs use ISD with CG (grep); not measured.

**Public changes** (all in sim-core):
```rust
pub struct SolverStat {          // MuJoCo mjSolverStat
    pub improvement: f64,
    pub gradient: f64,
    pub lineslope: f64,          // |slope| at the accepted step × scale/‖search‖ (was: slope before the search)
    pub nactive: usize,          // rows not Satisfied (was: Quadratic or Cone)
    pub nchange: usize,
    pub neval: usize,            // renamed from nline; counts every line-search evaluation
    pub nupdate: usize,          // new: Cholesky updates this iteration
}
// Data: solver_niter = Σ over islands; solver_stat concatenated in island order (Q3)
// Data::newton_solved: removed from the public API (Q2)
```

**Measured with the port** (`R3_PRIMAL=1 R3_ISLAND=1`): §2.3 and §3.3; the A13 equality fixtures stay ≤ 4.8e-15 over 500 steps (11 fixtures; `conn_pend_Euler_Newton` 3.9e-13 → 3.5e-17); on `cone_idx.xml` (elliptic, a limited hinge with frictionloss plus a box contact, two islands) the iteration counts of the first four steps equal MuJoCo's [7, 6, 4, 5] only with islands (monolithic: [6, 5, 3, 4]), and the trajectory agrees for 238 steps (6.5e-10 at 300; same-input qacc gap at the onset 2.3e-13, MuJoCo's own 1-ulp sensitivity there 1.7e-13).

**Islands alone** (`R3_ISLAND` on top of the port): 0 census verdicts change (`v2 → v3`); 50 corpus docs change bits relative to the monolithic port.

---

## 5. Item 3 — late onset under elliptic cones / noslip (6 docs)

### 5.1 First differing quantity
With MuJoCo's exact state at the step before onset (`samein.py`), **the contact count** differs: ours 4 box–plane contacts, MuJoCo 1. Ours: one penetrating corner (same distance as MuJoCo's to ≤ 1e-16) plus three corners above the plane at +3.0e-5 … +6.4e-2 with margin 0 (`late_onset_contacts.txt`; 26747e04 at 64, 28695e44 at 40, 38afc806 at 80, 703108e1 at 87, 830708de at 40, 92209052 at 35). At that step the states of five docs agree to ≤ 8.9e-16 (with the port), so the first touchdown already differs; 38afc806 (PGS + noslip) has drifted by 8.9e-12 qpos / 1.2e-10 qvel by then, and the source of that drift was not isolated.

### 5.2 Cause
- **Ours** `collision/plane.rs:79-147`: "if the DEEPEST bottom corner is within margin, emit ALL bottom-face corners" (`:88`, `:130`), justified in its comment by noslip stability with a reference to `MULTI_CONTACT_ANALYSIS.md Appendix A` (not checked).
- **MuJoCo** `mjc_PlaneBox`, `engine_collision_primitive.c:196-239`: each corner is skipped when `dist + ldist > margin || ldist > 0` (`:219-222`).
- This is ledger-L41's "box–plane corners at positive distance" (A16 §2, 14 docs).

### 5.3 Prototype and measurement (`R3_PLANEBOX=1`)
26747e04, 38afc806, 703108e1, 830708de, 92209052 agree over 100 steps (≤ 9.4e-12 qpos, 1.2e-10 qvel); so do a0ff89f4 (§2.5) and the L41 docs 1a42c10c and 74908604 (`v3 → v4`). Bits: 18 corpus docs change (all box-on-plane; listed in `bitcmp_summary.txt`). Tests and the 20 validators: no failure; `contact-tuning/stress-test` prints different numbers (low-μ displacement 7.1098 → 7.1115 m, override-box velocity 4.788 → 4.942), all checks still PASS.

### 5.4 What remains
- **28695e44** (two stacked boxes): first differs at step 57. With MuJoCo's state, the box–box contact normals differ by 3.0e-3 and the tangent frames by 1.0 from step 1 on (`concmp.py`, step 55–56). Box–box collision: handed off.
- **e2 excitation**: the three two-box docs (aeb76329, e0554f47, faa6b270) get further from MuJoCo with `R3_PLANEBOX` (0.008 → 0.17 qpos). They already differ at t0 in `nefc`: MuJoCo keeps the zero-distance box–box contacts, ours drops them (ledger-L41's other half). The plane–box rule should therefore land with or after L41's zero-distance fix, and these docs be re-measured then.
- **Owner**: collision (ledger-L41) in Rigid-physics. This section only isolates and prototypes it.

---

## 6. Item 4a — `<pair solreffriction>` with an elliptic cone (2 docs)

Two independent causes.

### 6.1 d81df1fa (`unified_solvers.rs:1166`, `solreffriction="0.1 0.0"`)
- First differing quantity at the first contact (step 31), same input: friction rows' `efc_aref` ours 0, MuJoCo −22.104 / 8.424.
- **MuJoCo** `getsolparam` (`engine_core_constraint.c:1290-1361`): a solreffriction whose two values do not share a sign is replaced by (0,0) with the warning "solreffriction values should have the same sign, replacing with default" (`:1340-1343`; seen on stdout); (0,0) means the friction rows use `solref`. The same check replaces a mixed-format `solref` by (0.02, 1) with "mixed solref format" (`:1334-1337`).
- **Ours**: no check (`contact_assembly.rs:76-77`; `compute_kbip` `impedance.rs:120-152`); `0.1 0.0` gives B = 0.
- **Fix** (`R3_SOLREF=1`): both checks in assembly, with `log::warn!` (sim-core's warning sink). d81df1fa: trajectory agrees (1.0e-15; base 9.9e-2). A mixed `solreflimit="0.05 -2"` fixture (`R3/xml/mixed_solref_limit.xml`): 6.1e-2 → 8.7e-19 over 300 steps.
- **Bits**: 1 corpus doc changes (d81df1fa). The static corpus has 1 mixed-sign value; `git grep` finds one more in a `format!` template: the derivatives validator's `solref="{} 1.0"` with `-stiffness` (`examples/fundamentals/sim-cpu/derivatives/stress-test/src/main.rs:178`), which flips (§11).

### 6.2 2cb92700 (`unified_solvers.rs:1104`, condim 6, `solreffriction="0.08 0.6"`)
- Same input at step 31: `efc_R` of rows 3–5 (torsional, rolling) ours 0.1037, MuJoCo 4146 / 103654 / 103654; `efc_force` of the sliding rows −4.76 vs −1.54.
- **MuJoCo** `mj_makeImpedance` (`:1465-1579`): for an elliptic contact R[1] = R[0]/impratio, R[j+1] = R[1]·f₀²/f_j² (`:1531-1548`), cone μ = f₀·√(R[1]/R[0]) (`:1537`); D = 1/R; then every row's diagApprox = R·imp/(1−imp) (`:1575-1578`).
- **Ours** (`contact_assembly.rs:139-151`) copies the normal row's imp, diagApprox, R and D into every friction row; the classifier uses f₀ as the cone μ (`pgs.rs:613`, whose comment records the impratio gap).
- **Fix** (`R3_ELLR=1 R3_ELLMU=1`; `R3_DIAGADJ=1` for the diagApprox field). 2cb92700: trajectory agrees (1.0e-15; base 7.2e-2). `R3/xml/ell_imp_{3,6}_{Newton,CG,PGS}.xml` (impratio 10): ≤ 3.0e-10 over 300 steps for Newton and PGS at condim 3 and 6 and for CG at condim 3 (base, where run: 1.2e-4 … 5.5e-2); `ell_imp_6_CG` 3.5e-7 (same-input per-step gap 2.0e-13, MuJoCo's own 1-ulp sensitivity 7.7e-14; the growth over steps was not isolated).
- **Bits**: `R3_ELLR` changes 2 corpus docs (2cb92700; and 92209052, isotropic friction 0.3: measured, `r·0.3·0.3/(0.3·0.3) ≠ r` for 30,539 of 100,000 random r, and for none with 0.5 or 1.0); `R3_ELLMU` 0; `R3_DIAGADJ` 0 (no solver reads `efc_diagApprox`; MuJoCo's only other reader is `engine_print.c:1521`). With `R3_DIAGADJ`, `efc_diagApprox` equals MuJoCo's on a0ff89f4, 406a1991 (pyramidal) and 60c23753 (base differs by 1.0, 9.3e-3, 1.0).

---

## 7. Item 4b — equality rows between bodies with no dofs (5 docs)

- **MuJoCo's rule** (cited): `mj_addConstraint` (`engine_core_constraint.c:238-327`) drops any non-contact constraint whose rows are all zero: dense, it scans `size·nv` Jacobian entries (`:248-265`) and returns without adding rows (`:306-309`); sparse, `NV == 0` returns early (`:279-284`). It is not a body-dof test; a weld between two bodies with no dofs is one case of an all-zero Jacobian.
- **Ours**: `assemble_equality_rows` emits 3 or 6 rows regardless (`equality_assembly.rs:31-74`). The rows have J = 0, so no force reaches the bodies.
- **Fix** (`R3_EMPTY=1`): skip the equality when every Jacobian entry is zero; truncate the pre-counted arrays (the counting pass `assembly.rs:136-150` still over-counts, as MuJoCo's dense count `mj_addConstraintCount` `:1635-1641`). The spec should apply it to every non-contact row kind (tendon friction and tendon limits can have zero Jacobians too; flex edges are A13 D3's).
- **Measured**: 0ee0d13e, 10d41092, 27a5746e, b125875a, f85c405a agree (nefc 0, as MuJoCo). **0 corpus docs change trajectory bits**; tests and validators unchanged.

---

## 8. Items 4c, 4d — weld `relpose` and `torquescale`

### 8.1 relpose (496f2186, `equality_constraints.rs:800`)
- MuJoCo reads `relpose` into `eq_data[3..10]` (`xml_native_reader.cc:2155`); in `mj_setConst`, if the quaternion part is not all zero it keeps the user's position, **normalises the quaternion** and skips the qpos0 computation (`engine_setconst.c:813-818`); otherwise both parts come from qpos0 (`:820-829`).
- Ours always computes `eq_data[3..6]` from qpos0 and copies the quaternion verbatim (`sim/L0/mjcf/src/builder/equality.rs:166-197`); an all-zero relpose quaternion is used as given.
- Fix (`R3_RELPOSE=1`, sim-mjcf, Rigid-loading): MuJoCo's rule. 496f2186's model now agrees (`eq_data` as MuJoCo's); the trajectory agrees to ≤ 5.6e-16 qpos / 5.3e-15 qvel for 32 steps; at step 33 the two boxes collide and, with MuJoCo's state, the box–box normal differs by 1.1e-3 (box–box, handed off). The quaternion's runtime use was already normalised (`UnitQuaternion::from_quaternion`, `constraint/equality.rs:115-117`).

### 8.2 torquescale — implement (cheap)
- MuJoCo: element attribute only (not in `<default><equality>`, `xml_native_reader.cc:179`), read into `eq_data[10]` (`:2192`), default 1 (`user_init.c:307`); the rotational error and the rotational Jacobian rows are multiplied by it (`engine_core_constraint.c:482`, `:510`, `:530`). No range check found.
- Prototype: parser + `MjcfWeld::torquescale: Option<f64>` + builder `eq_data[10] = torquescale.unwrap_or(1.0)`; `extract_weld_jacobian` (`constraint/equality.rs:107-215`) scales `rot_error` and rows 3–5.
- Measured over 500 steps (`R3/xml/weld_ts_{0,0.5,2}.xml`, `weld_free_ts_0.3.xml`, `weld_ts_relpose.xml`): base 2.8e-2 … 2.3 qpos off; fix ≤ 8.7e-15. **0 corpus docs change bits** (none sets torquescale; the scaling is skipped at 1.0).
- **Ordering hazard**: today's builder writes `eq_data[10] = 0` (A16 §0). The core change reading it must land in the same commit as the builder writing 1.0, or every MJCF weld loses its rotational rows. No code outside the MJCF builder constructs a weld (`git grep EqualityType::Weld`). A `.mjb` saved by 0.9 carries 0 there.

---

## 9. Further findings in this area

| finding | referent | disposition |
|---|---|---|
| `hessian_cone` finds a contact's first row with `data.efc_type[id]` (the contact index used as a row index), `hessian.rs:689-692`; with non-contact rows first, cones get no Hessian term | `R3/xml/cone_idx.xml`: base 37/23/25/25 Newton iterations vs MuJoCo 7/6/4/5; the index fix alone 13/7/6/6 | subsumed by the port |
| PGS and noslip scale with the per-step trace (`pgs.rs:249`, `noslip.rs:200`); MuJoCo with `m->stat.meaninertia` (`engine_solver.c:326`, `:550`) | `R3_MEANI=1`: 0 of 1,408 corpus docs change bits; two scratch fixtures unchanged | no failing test found (Q5) |
| `Data::stat_meaninertia` is ours only (per-step trace/nv, `constraint/mod.rs:67-78`) | MuJoCo has `m->stat.meaninertia` only | Q5 |
| validator `solvers/stress-test` prints CG average iterations 31.1 (base) → 1.1 (port); MuJoCo on the same scene: 2.67 (max 31), Newton 1.04 | `R3/xml/solvers_stress_*.xml`, MuJoCo loop over 2,500 steps | the scene's box contacts also differ (§5.4); not chased |

---

## 10. Tests to add (each fails at the parent; values measured here)

| test | input | at the parent | after |
|---|---|---|---|
| un-ignore the 24 `golden_flags` tests | `golden_flags.rs` (MuJoCo 3.4.0 golden `qacc`, 1e-8) | 24 fail (first: `golden_baseline`, step 0, dof 0, diff 1.98e-3) | pass (also with `R3_HESS=3` alone) |
| `chol_update_minus_removes_row` (unit) | random SPD 2…6, `v` | negated update misses H−vvᵀ by 0.14–0.83 | ≤ 1e-14 |
| `newton_matches_mujoco_3_5_0_iterates` | 406a1991 at t0: niter 3, improvements 850320.7085227252 / 136.11566162669902 / 0.09305058493257877 (1e-9 rel), qacc 1e-12 | 100 iterations then PGS | match |
| `newton_has_no_pgs_fallback` | 406a1991, `iterations=1`: qacc = MuJoCo's first iterate | 170 off | 2.8e-13 |
| `cg_matches_mujoco_3_5_0` | `conn_free2_Euler_CG`, 500 steps, 1e-12; per-step iteration count = MuJoCo's | 6.9e-7; 2 vs 4 iterations | 1.2e-14; equal |
| `cg_keeps_its_iterate` | 443824ba, 100 steps, 1e-12 | 1.4e-8 | 1.1e-15 |
| `islands_iterate_as_mujoco` (if Q1) | `cone_idx.xml`: iterations [7,6,4,5]; 200 steps 1e-12 | [37,23,25,25] | equal; 238 steps ≤ 1e-12 |
| `solver_niter_cleared_without_rows` | 28695e44 after step 3 (no rows) | 2 | 0 |
| `empty_equality_adds_no_rows` | 0ee0d13e: nefc 0 | 6 | 0 |
| `mixed_solreffriction_uses_solref` | d81df1fa, 100 steps, 1e-12 | 9.9e-2 | 1.0e-15 |
| `mixed_solref_uses_default` | `mixed_solref_limit.xml`, 300 steps | 6.1e-2 | 8.7e-19 |
| `elliptic_friction_rows_as_mujoco` | 2cb92700 100 steps; `ell_imp_6_Newton` 300 steps, 1e-9 | 7.2e-2; 5.5e-2 | 1.0e-15; 3.0e-10 |
| `diag_approx_as_mujoco` | a0ff89f4 t0 `efc_diagApprox` | differs by 1.0 | 0 |
| `weld_torquescale_matches_mujoco` | `weld_ts_0.5.xml`, 500 steps, 1e-12 | 8.1e-2 | 1.0e-15 |
| `weld_relpose_as_mujoco` (Rigid-loading) | 496f2186: `eq_data` equal; 30 steps 1e-12 | eq_data[3] 0.5 vs 0 | equal; 5.6e-16 |
| plane–box per corner (collision owner) | 830708de, 100 steps | 4.4e-3 | 1.4e-13 |

Golden values come from MuJoCo 3.5.0 (`mjprobe.py`/`mjtraj.py`); fixtures in `R3/xml/` and A13's `ID/xml/`.

## 11. Tests and examples that flip

All suites run in scratch copies (`R3/out/tests_T*.log`, `lib_*`, `val/`). Base: integration 1,338 pass / 27 ignored, conformance 83, sim-core lib 720, sim-mjcf lib 385.

| flip | where | why | kind |
|---|---|---|---|
| `flex_unified::spec_a_t4_simulation_regression_bit_identical` | `sim/L0/tests/integration/flex_unified.rs:2269` | pins `qvel[2]` bits; the port changes it by 1 ulp (−0.09807351723758387 vs −0.0980735172375839) | CI test; re-capture or compare to a tolerance (no MuJoCo reference: MuJoCo refuses the flex doc) |
| `unified_solvers::test_s31_one_component_zero_still_activates` | `unified_solvers.rs:1164-1204` | asserts friction rows use the mixed `[0.1, 0.0]`; MuJoCo replaces it | CI test; rewrite to MuJoCo's rule (the doc becomes `mixed_solreffriction_uses_solref`) |
| derivatives validator check 30 "Contact stiffness affects A" | `examples/fundamentals/sim-cpu/derivatives/stress-test/src/main.rs:165-186`, `:926-955` | its `solref="-500 1.0"` is MuJoCo-mixed → default for both stiffnesses → A identical | **validator (CI)**: change the fixture to direct format (`"-k -b"`) in the same commit |
| 24 `golden_flags` tests | `golden_flags.rs` | now pass | ignored → un-ignore; update `docs/KNOWN_GAPS.md` Gap 1 |
| validators with changed numbers, all PASS | `solvers/stress-test` (CG iterations), `contact-tuning/stress-test` (§5.3), `equality-constraints/stress-test` (`solver_niter` 1 → 2: sum over 2 islands) | | validators |

Unchanged: `mujoco_conformance` 83; sim-core lib 720 and sim-mjcf lib 385 with every switch. With every switch, 16 of the 20 validators print identical stdout; contact-tuning, equality-constraints and solvers print different numbers and pass; derivatives fails check 30 (above).

## 12. Downstream
- `Data::newton_solved` readers: `sim/L0/core/src/test_fixtures/conformance.rs:827`, `:937` (asserts no fallback before using CPU Newton as the GPU oracle; consumed by `sim/L0/gpu/src/pipeline/contact_conformance_tests.rs:392`), `integrate/mod.rs:168`, `forward/mod.rs:569`, examples `solvers/newton/src/main.rs:128,206,244`, `solvers/stress-test/src/main.rs:135`.
- `solver_niter` / `SolverStat` readers: examples `solvers/{cg,pgs,newton,stress-test,comparison-visual}`, `equality-constraints/stress-test:298`; `SolverStat` re-exported at `sim/L0/core/src/lib.rs:233`.
- Files outside sim-core mentioning `stat_meaninertia` (Model's or Data's, not separated): `sim/L0/tests/integration/newton_solver.rs`, `sim/L0/mjcf/src/builder/build.rs`, `design/cf-design/src/mechanism/model_builder.rs`, `design/cf-design-tests/tests/sdf_sphere_diagnostic.rs` (grep; uses not read).
- `efc_diagApprox` readers in tests: `deformable_friction_dt25.rs:286`, `diagapprox_bodyweight.rs:443-500`, `phase13_diagnostic.rs:112,227`; all pass with `R3_DIAGADJ`.
- `eq_data` writers: only the MJCF builder writes welds; cf-design writes connects (`model_builder.rs:239-248`, slot 10 unread by a connect).
- sim-gpu's own solver (`shaders/newton_solve.wgsl`, fixed backtracking line search) is unchanged; its CPU oracle changes.

## 13. Commit list (Rigid-physics unless marked)

| # | commit | contents | flips it carries | expected corpus bit changes |
|---|---|---|---|---|
| P1 | `fix(sim-core): Newton and CG run MuJoCo's primal solver` | §4 (port, no fallback, termination, line search, Hessian ±update, cone by row, scale, `solver_niter` reset, CG iterate kept, `SolverStat` shape); un-ignore the 24 golden-flag tests; `KNOWN_GAPS.md` Gap 1 | `spec_a_t4` re-capture; GPU-oracle assertion on `newton_solved` | 263 docs, every one a Newton or CG doc |
| P2 | `feat(sim-core): constraint islands solved separately, as MuJoCo` (Q1) | MuJoCo's efc-based partition; island scale | `equality-constraints` validator prints `solver_niter` 2 | 50 docs relative to P1 |
| P3 | `fix(sim-core): a constraint with an all-zero Jacobian adds no rows (MuJoCo mj_addConstraint)` | §7 for every non-contact row kind | — | 0 |
| P4 | `fix(sim-core): solref and solreffriction format checks as MuJoCo getsolparam` | §6.1 | `test_s31…` rewrite; derivatives validator fixture | 1 (d81df1fa) |
| P5 | `fix(sim-core): elliptic friction rows take MuJoCo's impratio and friction-ratio R; diagApprox from R` | §6.2 | — | 2 |
| P6 | `feat: weld torquescale` | core scaling + builder default `eq_data[10] = 1.0` (one cross-layer commit, §8.2) | — | 0 |
| L1 (Rigid-loading) | `fix(sim-mjcf): weld relpose as MuJoCo; parse torquescale` | §8.1; the `torquescale` attribute (schema allowlist, M15) | — | 1 (496f2186) |
| (collision) | plane–box per corner, with ledger-L41's zero-distance contacts | §5 | — | 18 |

Each row was measured as an env switch on one prototype, not as a commit series; per-commit greenness in this order was not run.

## 14. Dependencies
- **A13 D2** (explicit `qacc` under implicit integrators) gates on `newton_solved` in `forward_acc` (`forward/mod.rs:578-595` in the prototype); P1 changes what that flag means (CG too). Land D2 first and replace the flag in P1, or merge.
- **K3** (core-L6, ISD `forward()`) edits the same `forward_acc` branch.
- **K6** (`recompute_derived`, trees in sim-core): P2 needs tree data on every Model; until then P2 needs its `ntree > 0` guard.
- **Sleep area**: P2's partition comes from efc rows (MuJoCo); our `mj_island` builds from raw contacts/equalities/limits and omits friction rows (`island/mod.rs:34-150`) and is what sleep uses. One partition or two is the sleep area's call.
- **A13 D3** (flex rows): P3's empty rule also drops zero-Jacobian flex edge rows; MuJoCo skips rigid edges explicitly too (`engine_core_constraint.c:620-624`).
- **Collision / ledger-L41**: §5's fix; 28695e44 and 496f2186 (box–box normals) and c31a6ac0 (plane–capsule frame) need the collision and frame owners.
- **Census gate**: baseline after P1–P6; §3.4 decides which platform's goldens it pins.

## 15. Open questions (not settled by the fixed decisions)

**Q1 — islands in Rigid?** Options: (a) P2 in Rigid; (b) monolithic port only. *Recommend (a)*: it is MuJoCo's default path (`engine_forward.c:763-797`), and only with it do per-step iteration counts match (`cone_idx`, 87d99b20's island 0 statistics). If wrong: measured, (b) changes 0 census verdicts relative to (a), so the cost of deferring is iteration counts and roundoff-level differences on multi-island scenes.

**Q2 — `Data::newton_solved`.** Options: delete; make crate-private as "qacc came from a primal solve"; keep pub. *Recommend crate-private*: with no fallback it is always true under Newton, and MuJoCo has no such field. If wrong: a user reading it gets a compile error instead of a constant.

**Q3 — statistics shape.** Options: MuJoCo's per-island arrays (`solver_niter[mjNISLAND]`, stats at `island·mjNSOLVER + iter`); or one `usize` sum plus concatenated stats. *Recommend the sum plus concatenation, with MuJoCo's `SolverStat` fields* (§4). If wrong: a user cannot split statistics per island without a later additive field.

**Q4 — mixed-sign `solref`/`solreffriction`.** MuJoCo replaces with a warning each step (not silent). Options: parity (replace + `log::warn!`, also for code-built models); refuse in the loader (stricter). *Recommend parity*, since MuJoCo warns. If wrong (refusal wanted): add a loader check; the runtime rule stays for code-built models.

**Q5 — PGS/noslip scale and `Data::stat_meaninertia`.** MuJoCo uses the model constant (`engine_solver.c:326`, `:550`, `:1863`). No corpus doc or fixture changes (0 of 1,408). *Recommend*: P1 uses the model constant for the primal monolithic path (as measured); leave PGS/noslip and the Data field until a fixture fails (a rule with no failing test is not added). If wrong: PGS/noslip termination can differ on configuration-dependent trace(M); not observed.

**Q6 — fused multiply-add.** Options: emulate `mul_add` where the arm64 build contracts; accept roundoff differences. *Recommend accept*: the contraction pattern is a compiler choice (the x86_64 slice has none), and emulating it ties our arithmetic to one build of the oracle. If wrong: amplified docs (87d99b20, 60c23753, the weld CG fixture) stay outside 1e-9 agreement; they need the gate's classification either way.

**Q7 — the solver port vs patches.** Options: P1 as the port; or patches (downdate fix, cone index, remove fallback, CG termination/β, keep our line search). *Recommend the port*: the patches leave two line-search algorithms that do not match MuJoCo's evaluation sequence, and `R3_HESS=3` alone fixes 6 Newton docs but not the CG doc (`v1` vs `v2`). If wrong: the patch route was measured only for the Hessian part (`R3_HESS=3`: 6 docs, 24 golden tests); its CG part was not measured separately.

## 16. Handed to other areas
| finding | referent | owner |
|---|---|---|
| plane–box per-corner rule; it must land with L41's zero-distance contacts (§5.4) | `R3_PLANEBOX`, `late_onset_contacts.txt` | collision / ledger-L41 |
| box–box normal (3.0e-3, 1.1e-3) and frame differences | 28695e44 step 56, 496f2186 step 33 (`concmp.py`) | collision |
| plane–capsule frame hint | c31a6ac0 step 1 | NEW-FRAME |
| oracle FMA on arm64, none on x86_64; goldens are platform-dependent; amplified docs need a classification (e.g. MuJoCo's own 1-ulp sensitivity, `mjsens_solver.py`) | §3.4 | census gate |
| `docs/KNOWN_GAPS.md` Gap 1 is overturned | §1 | docs (K14/M26) |
| island partition source (efc rows vs raw sources) | §14 | sleep |

## 17. What this method cannot see
- **The sparse path** (nv ≥ 60): no census doc has nv > 60; the prototype is dense only.
- **Not run**: sim-gpu (its CPU oracle changes; GPU tests compare at a tolerance), cf-design / cf-design-tests (connect linkages), sim-thermostat tests, sim-urdf tests, sim-mjcf `tests/forward_conformance.rs`, ml-chassis / sim-rl / sim-opt, L1 crates, doctests, `grade`, clippy, licensed gates; validators that need sim-urdf, sim-ml-chassis, cf-*, mesh-* (20 of the validators were run).
- **x86_64 oracle**: Rosetta is not installed, so whether x86_64 MuJoCo agrees bit-closer with ours is not measured (the downloaded x86_64 interpreter was removed).
- **Flex**: MuJoCo refuses every corpus flex doc; the port's effect on flex docs is measured only as bit changes and the one bit-pinned test.
- **Runtime-generated MJCF** (157 `format!` templates): only grepped for `solref` (1 hit).
- **Horizons and states**: 100 census steps, two excitations; scratch fixtures 300–500 steps.
- **Causes behind the first**: 6 docs moved to another cluster after a fix (§2.5, §5.4, §8.1); more may sit behind the remaining ones.
- **The individual shares** of the CG causes in §3.2 were not separated.
- **ISD + CG**: 0 corpus docs; the recommended behaviour is unmeasured.

**Repo state.** Before and after: HEAD `fbc0ad542ab9daa036fed3a561bed95af90918f3`, `git status --short` empty. All builds used scratch `target/` dirs (deleted at the end, with the 1.5 M-line arm64 disassembly; its counts are reproduced by `otool -arch arm64 -tv …/libmujoco.3.5.0.dylib | grep -c 'fmadd\|fmsub'`); all MuJoCo runs used `R3/out` as working directory. One `uv venv` attempt downloaded an x86_64 interpreter into `~/.local/share/uv/python/`; it failed to run (no Rosetta) and was deleted.
