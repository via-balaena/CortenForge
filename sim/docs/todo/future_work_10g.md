# Future Work 10g — Deferred Item Tracker: Group 6 — Actuator & Dynamics

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 6 — Actuator & Dynamics (9 items)

**Spec approach:** DT-58 needs an individual spec (T3 — new dynamics variant).
DT-56/57 share an "acc0/dampratio" spec (T2). DT-59 joins the cross-file
"Length-Range Estimation" spec with DT-77/78 (T2). DT-60/61 implement directly
(T1). DT-107 shares the runtime interpolation spec with Spec C (T2).
DT-108/110 implement directly (T1). Totals: 4 T1, 4 T2, 1 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-56 | §12 | `dampratio` for position actuators — requires `acc0` at compile time | Low | T2 |
| DT-57 | §12 | `acc0` computation for non-muscle actuators — extend `compute_muscle_params()` | Low | T2 |
| DT-58 | §5 | sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant | Low | T3 |
| DT-59 | §5 | Bisection-based lengthrange for unlimited slide joints (`mj_setLengthRange`) | Low | T2 |
| ~~DT-60~~ | §35 | ~~`jnt_actgravcomp` routing to `qfrc_actuator` — currently all goes to `qfrc_passive`~~ **Subsumed by §41 S4.2a** | Medium | T1 |
| ~~DT-61~~ | §35 | ~~`DISABLE_GRAVITY` flag not defined — only `gravity.norm() == 0.0` check exists~~ **Subsumed by §41 S4.2** | Low | T1 |
| DT-107 | Spec D | Runtime interpolation logic — `mj_forward` reads history buffer to compute delayed ctrl, `mj_step` writes to history buffer as circular buffer. Model/data structures exist (Spec D), runtime application missing. | Medium | T2 |
| DT-108 | Spec D | `dyntype` enum gating interpolation eligibility — MuJoCo restricts which `ActuatorDynamics` variants may use the history buffer. Currently no gate; any actuator with `nsample > 0` gets a history buffer regardless of dyntype. | Low | T1 |
| DT-110 | Spec D | `actuator_plugin` model array — per-actuator plugin ID (`int[nu]`, -1 sentinel). Depends on §66 (plugin/extension system). | Low | T1 |
