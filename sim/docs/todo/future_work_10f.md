# Future Work 10f — Deferred Item Tracker: Group 5 — Derivatives & Analytical Methods

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 5 — Derivatives & Analytical Methods (11 items)

**Spec approach:** DT-45/46/50 each need individual specs (T3 — research-grade).
DT-47/51/52/54 share a "Derivative Extensions" spec (T2). DT-49/53 share an
"FD Performance" spec (T2). DT-48 joins the "Sparse Storage" spec with DT-36/37/44
(T2). DT-55 implements directly (T1). Totals: 1 T1, 7 T2, 3 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-45 | §12 | Full position-analytical derivatives (`dFK/dq`, `dM/dq`) — massive complexity, deferred | Low | T3 |
| DT-46 | §12 | Contact-analytical derivatives — implicit function theorem through PGS/CG | Low | T3 |
| DT-47 | §12 | Sensor derivatives (C, D matrices) — `TransitionMatrices` reserves `Option` fields | Low | T2 |
| DT-48 | §12 | Sparse derivative storage — all matrices dense, follow-up for nv > 100 | Low | T2 |
| DT-49 | §12 | Parallel FD computation — each perturbation column requires sequential `step()` | Low | T2 |
| DT-50 | §12 | Automatic differentiation — dual numbers / enzyme, no scalar type genericity changes | Low | T3 |
| ~~DT-51~~ | §12 | ~~`mjd_inverseFD` — inverse dynamics derivatives~~ **DONE** (Phase 11 Session 3, `c68d9cb`) | Low | T2 |
| ~~DT-52~~ | §12 | ~~`mjd_subQuat` — quaternion subtraction Jacobians~~ **DONE** (Phase 11 Session 2, `9feebfe`) | Low | T2 |
| ~~DT-53~~ | §12 | ~~Skip-stage optimization (`mj_forwardSkip`)~~ **DONE** (Phase 11 Session 3, `c68d9cb`) | Medium | T2 |
| ~~DT-54~~ | §12 | ~~Muscle actuator velocity derivatives~~ **DONE** (Phase 11 Session 2, `9feebfe`) | Low | T2 |
| DT-55 | §13 | `skipfactor` / factorization reuse across implicit steps | Low | T1 |
