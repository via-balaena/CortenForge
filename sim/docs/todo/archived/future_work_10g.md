# Future Work 10g — Deferred Item Tracker: Group 6 — Actuator & Dynamics

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 6 — Actuator & Dynamics (15 items)

**Spec approach:** DT-58 needs an individual spec (T3 — new dynamics variant).
DT-56/57 share an "acc0/dampratio" spec (T2). DT-59 joins the cross-file
"Length-Range Estimation" spec with DT-77/78 (T2). DT-60/61 implement directly
(T1). DT-107 shares the runtime interpolation spec with Spec C (T2).
DT-108/110 implement directly (T1). Totals: 4 T1, 4 T2, 1 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| ~~DT-56~~ | §12 | ~~`dampratio` for position actuators — requires `acc0` at compile time~~ **DONE** — Phase 5 Spec A (Session 3, commit `a1cbbba`). | Low | T2 |
| ~~DT-57~~ | §12 | ~~`acc0` computation for non-muscle actuators — extend `compute_muscle_params()`~~ **DONE** — Phase 5 Spec A (Session 3, commit `a1cbbba`). Renamed to `compute_actuator_params()`. | Low | T2 |
| ~~DT-58~~ | §5 | ~~sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant~~ **DONE** — Phase 5 Spec C (Session 17, commit `c64bab1`). Rigid tendon only; deferred sub-items below. | Low | T3 |
| DT-111 | Spec C | HillMuscle compliant tendon mode — requires persistent fiber state (`act_num ≥ 2` or separate state array) and custom integration. Currently only rigid tendon supported. | Low | T3 |
| DT-112 | Spec C | HillMuscle named MJCF attributes (`optlen`, `slacklen`, `pennation`) — convenience UX over raw `gainprm` indices | Low | T1 |
| DT-113 | Spec C | `<hillmuscle>` shortcut element — analogous to `<muscle>`, no MuJoCo precedent | Low | T1 |
| DT-114 | Spec C | HillMuscle variable pennation angle — `α = asin(w / L_fiber)` as function of fiber length, replacing fixed `gainprm[7]` | Low | T2 |
| DT-115 | Spec C | HillMuscle configurable curve parameters — Gaussian FL widths, FV curvature, FP shape via extended `gainprm`/`biasprm` | Low | T2 |
| DT-116 | Spec C | Per-actuator `GainType::User` / `BiasType::User` callback infrastructure — depends on §66 (plugin system) | Low | T3 |
| ~~DT-59~~ | §5 | ~~Bisection-based lengthrange for unlimited slide joints (`mj_setLengthRange`)~~ **DONE** — Phase 5 Spec A (Session 3, commit `a1cbbba`). Simulation-based LR estimation. | Low | T2 |
| ~~DT-60~~ | §35 | ~~`jnt_actgravcomp` routing to `qfrc_actuator` — currently all goes to `qfrc_passive`~~ **Subsumed by §41 S4.2a** | Medium | T1 |
| ~~DT-61~~ | §35 | ~~`DISABLE_GRAVITY` flag not defined — only `gravity.norm() == 0.0` check exists~~ **Subsumed by §41 S4.2** | Low | T1 |
| DT-107 | Spec D | Runtime interpolation logic — `mj_forward` reads history buffer to compute delayed ctrl, `mj_step` writes to history buffer as circular buffer. Covers both actuators (Phase 5 Spec D) and sensors (Phase 6 Spec D): model/data structures exist for both, runtime application missing for both. Includes sensor history pre-population in `reset_data()`. | Medium | T2 |
| DT-108 | Spec D | `dyntype` enum gating interpolation eligibility — MuJoCo restricts which `ActuatorDynamics` variants may use the history buffer. Currently no gate; any actuator with `nsample > 0` gets a history buffer regardless of dyntype. Sensor-side: no gating needed (sensors always eligible). | Low | T1 |
| DT-110 | Spec D | `actuator_plugin` model array — per-actuator plugin ID (`int[nu]`, -1 sentinel). Depends on §66 (plugin/extension system). | Low | T1 |
