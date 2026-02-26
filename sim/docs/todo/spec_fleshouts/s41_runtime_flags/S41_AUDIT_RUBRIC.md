# S41 Audit Plan — Grading Rubric

Grades the audit plan itself (not the implementation). Each criterion is
scored A/B/C/D/F. The plan ships at A across all criteria.

---

## R1. Spec Traceability

> Every spec section, sub-section, and code-level requirement maps to at
> least one audit checkbox. No orphan spec items.

| Grade | Bar |
|-------|-----|
| **A** | 1:1 mapping from every S-section and sub-section (S1, S2a–S2e, S3, S3b, S4.1–S4.17, S5.1–S5.6, S6, S7a–S7d, S8a–S8g, S9a–S9f, S10a–S10d) to audit checkboxes. Cross-reference table exists. |
| **B** | All major S-sections covered but some sub-sections (e.g., S4.7d, S8d, S3b) missing or merged into parent without distinct checks. |
| **C** | Major sections covered, multiple sub-sections missing. |
| **D** | Only top-level coverage (S1–S10 as blocks). |
| **F** | Significant spec sections have no audit coverage. |

**Current gaps to check:**
- Is every S sub-section (S2a, S2b, S2c, S2d, S2e, S3b, S4.2a, S4.7a–S4.7e,
  S4.7-prereq, S7a–S7d, S8a–S8g, S9a–S9f, S10a–S10d) individually
  traceable to one or more checkboxes?
- Does the plan have a spec-section → checkbox cross-reference index?

---

## R2. Specificity

> Each checkbox specifies exactly what to verify — file, function,
> condition, expected value — such that two independent auditors would
> reach the same pass/fail conclusion.

| Grade | Bar |
|-------|-----|
| **A** | Every checkbox names the file, function (or struct/field), and the exact condition or value to verify. No subjective checks ("looks correct", "properly implemented"). |
| **B** | Most checks are specific; a few use vague language ("verify guard exists") without stating the exact condition. |
| **C** | Mix of specific and vague checks. |
| **D** | Mostly "verify X is implemented" without expected values/conditions. |
| **F** | Checklist is a list of section names, not verifiable items. |

**Current gaps to check:**
- Do all "guard" checks specify the exact boolean expression?
- Do field checks specify the exact type, default value, and initialization site?
- Are interaction checks (e.g., gravity × actuation × gravcomp) expressed
  as truth tables or matrices with expected outcomes per cell?

---

## R3. Init-Then-Guard Coverage

> The spec's "init-then-guard" pattern (unconditional initialization
> before conditional guards) is the #1 source of subtle bugs. The audit
> must verify BOTH the init AND the guard at every site.

| Grade | Bar |
|-------|-----|
| **A** | Every init-then-guard site has TWO separate checkboxes: one for the unconditional init (what is zeroed/cleared, in what order), one for the guard (exact condition). The ordering constraint (init BEFORE guard) is explicitly checked. |
| **B** | Init-then-guard sites identified, but some only check the guard (not the init) or don't verify ordering. |
| **C** | Some init-then-guard sites missing entirely. |
| **D** | Pattern not systematically audited. |
| **F** | Not mentioned. |

**Init-then-guard sites from spec (exhaustive):**
1. `mj_collision()` — zeroes `ncon`, clears contacts BEFORE flag guard
2. `mj_fwd_actuation()` — zeroes `actuator_force` BEFORE `nu==0 || DISABLE_ACTUATION`
3. `mj_passive()` — zeroes all 5 force vectors BEFORE spring+damper guard
4. `mj_fwd_constraint()` — zeroes `qfrc_constraint` BEFORE constraint guard
5. Sensor functions — do NOT zero sensordata (intentional exception)

---

## R4. Interaction / Compound Guard Coverage

> Many flags interact — compound guards, hierarchical gating, orthogonal
> mechanisms. The audit must verify these interactions, not just
> individual flags in isolation.

| Grade | Bar |
|-------|-----|
| **A** | Every compound guard and interaction from the spec has a dedicated check with the full boolean expression. Interaction matrices (truth tables) exist for multi-flag interactions. All hierarchical gating chains are traced end-to-end. |
| **B** | Major interactions covered, but some compound guards only check one branch. |
| **C** | Individual flags checked, interactions partially covered. |
| **D** | Only individual flags checked. |
| **F** | Interactions not considered. |

**Compound guards from spec (exhaustive):**
1. `DISABLE_CONTACT || DISABLE_CONSTRAINT || nbodyflex < 2` (S4.1)
2. `DISABLE_SPRING && DISABLE_DAMPER` → passive early return (S4.7a)
3. `!DISABLE_EULERDAMP && !DISABLE_DAMPER` → implicit damping (S4.14)
4. `!DISABLE_REFSAFE && solref[0] > 0.0` → solref clamp (S4.15)
5. `disabled(ACTUATION) || actuator_disabled(model, i)` → per-actuator (S4.8/S7d)
6. `!DISABLE_ISLAND && nisland > 0 && noslip == 0 && (CG || Newton)` (S4.13b)
7. `ngravcomp > 0 && !DISABLE_GRAVITY && gravity.norm() >= MIN_VAL` (S4.2)

**Hierarchical gating chains:**
1. `DISABLE_CONSTRAINT` → skips collision AND assembly → `nefc=0` → solver no-op (S4.1 + S4.3)
2. `DISABLE_SPRING + DISABLE_DAMPER` → skips gravcomp, fluid, contactPassive (S4.7a)
3. `DISABLE_ACTUATION` → skips force AND activation integration (S4.8 Sites 1+3)
4. `DISABLE_GRAVITY` → blocks both passive-side AND actuation-side gravcomp (S4.2 + S4.2a)

**Interaction matrices needed:**
1. Gravity × Actuation × Gravcomp routing (S4.2a — 4 combinations)
2. DISABLE_ACTUATION × per-group disable (S7d/AC41 — 3 states)
3. DISABLE_CONTACT × spring/damper × contactPassive (S4.7d/AC40)

---

## R5. Acceptance Criteria Mapping

> Every AC (AC1–AC48) maps to both (a) a test-existence check and
> (b) the specific assertion the test must make.

| Grade | Bar |
|-------|-----|
| **A** | Every AC has: test function name (or "missing"), the key assertion(s) the test must make, and traceability to the spec section it verifies. AC47 (deferred) is explicitly marked. |
| **B** | All ACs listed with assertions, but test function names not verified against actual code. |
| **C** | ACs listed but assertions are vague ("test exists"). |
| **D** | AC table is a copy of spec AC text without audit-specific content. |
| **F** | ACs not mapped. |

---

## R6. Behavioral Change / Regression Tracking

> §41 introduces multiple behavioral changes. Each must be explicitly
> listed with: what changed, what breaks, what was migrated.

| Grade | Bar |
|-------|-----|
| **A** | Every behavioral change from the spec's Risk section is a separate audit item with: (1) the change, (2) affected code/tests, (3) verification that migration was done. |
| **B** | Most changes covered, one or two missing. |
| **C** | Changes listed but affected code not traced. |
| **D** | "Check for regressions" without specifics. |
| **F** | Not addressed. |

**Behavioral changes from spec (exhaustive):**
1. `passive` field removed from `MjcfFlag` (S2a)
2. `island` default `false → true` (S2e)
3. Energy gated behind `ENABLE_ENERGY` — default 0.0 (S5.1)
4. `mj_check_acc` threshold: `is_finite()` → `is_bad()` (adds >1e10 bound) (S8g)
5. `step()` no longer returns `Err` for NaN — auto-resets internally (S8g)
6. `StepError` loses 3 variants (S8g)
7. Collision guard: `ngeom >= 2` → `nbodyflex < 2` (S4.1)
8. Flex bending: sum-then-clamp → independent per-component clamp (S4.7b Site 5)
9. `qfrc_passive` refactored to aggregation of 4 sub-arrays (S4.7-prereq)

---

## R7. Edge Case & Boundary Conditions

> The spec calls out specific edge cases. Each must have an audit check.

| Grade | Bar |
|-------|-----|
| **A** | Every spec-identified edge case has a dedicated checkbox. Boundary values (nv=0, nu=0, nq≠nv, group<0, group>30, solref[0]≤0) are explicitly checked. |
| **B** | Most edge cases covered, a few boundary conditions missing. |
| **C** | Some edge cases, no systematic boundary coverage. |
| **D** | Only happy-path checks. |
| **F** | Edge cases not considered. |

**Edge cases from spec:**
1. `nv == 0` in `mj_passive()` — AC48
2. `nu == 0` in `mj_fwd_actuation()` — part of S4.8 guard
3. `nq != nv` for quaternion joints — why `mj_check_pos` isn't sleep-filtered
4. `actuator_group < 0` or `> 30` — never disabled (S7c)
5. `solref[0] <= 0.0` — no refsafe clamping (negative solref has different semantics)
6. `nbodyflex < 2` — no collision possible
7. Post-reset `mj_forward()` failure in `mj_check_acc` — `tracing::error!`, no propagation
8. Model whose `qpos0` produces singular mass matrix — pathological but handled
9. Sleeping DOF with NaN — not detected by sleep-filtered check (AC30)
10. `qacc_warmstart` saved even when `DISABLE_WARMSTART` — consumption gated, not population

---

## R8. Completeness of File Inventory

> The spec lists ~30 files. The audit must verify changes in every file.

| Grade | Bar |
|-------|-----|
| **A** | Every file from the spec's Files table has at least one audit check. New files verified to exist. Modified files verified for each stated change. |
| **B** | Most files covered, a few missing. |
| **C** | Only primary files checked (enums.rs, flags.rs, builder, passive.rs). |
| **D** | File inventory not audited. |
| **F** | Not addressed. |

**Files from spec (verify each has audit coverage):**
- `types/enums.rs` — constants + StepError pruning
- `types/flags.rs` (new) — helpers
- `types/validation.rs` (new) — is_bad, MAX_VAL, MIN_VAL
- `types/warning.rs` (new) — Warning enum, WarningStat, mj_warning
- `mjcf/src/types.rs` — MjcfFlag changes
- `mjcf/src/parser.rs` — attribute parsing
- `mjcf/src/builder/mod.rs` — apply_flags
- `types/data.rs` — new fields + divergence_detected
- `types/model.rs` — new fields
- `types/model_init.rs` — field initialization
- `forward/mod.rs` — energy guards, step pipeline
- `forward/actuation.rs` — actuation guards, gravcomp routing, ctrl validation
- `forward/velocity.rs` — optional actuator_velocity split
- `sensor/position.rs`, `sensor/velocity.rs`, `sensor/acceleration.rs` — sensor guards
- `dynamics/rne.rs` — gravity guard in mj_rne + mj_gravcomp
- `energy.rs` — gravity guard + energy enable guard
- `constraint/assembly.rs` — contact/equality/frictionloss/limit guards
- `forward/passive.rs` — spring/damper refactor, gravcomp routing, fluid (§40c)
- `constraint/mod.rs` — constraint guard, warmstart
- `collision/mod.rs` — collision guard, filterparent, midphase, override broadphase
- `collision/mesh_collide.rs` — use_bvh parameter
- `constraint/impedance.rs` — refsafe guard
- `integrate/mod.rs` — per-actuator disable in Euler
- `integrate/rk4.rs` — per-actuator disable in RK4
- `constraint/contact_params.rs` (new) — assign_* helpers
- `reset.rs` (new) — mj_reset_data
- `forward/check.rs` — refactored check functions
- `forward/acceleration.rs` — eulerdamp guard
- `island/mod.rs` — island compound guard
- `lib.rs` — re-exports
- `tests/integration/runtime_flags.rs` — AC tests
- `tests/integration/golden_flags.rs` — golden conformance
- `tests/scripts/gen_flag_golden.py` — generation script
- `tests/assets/golden/flags/` — golden data + test MJCF

---

## R9. Audit Methodology

> The plan specifies HOW to verify, not just WHAT. Methodology is
> reproducible by any qualified auditor.

| Grade | Bar |
|-------|-----|
| **A** | Each phase has execution instructions: what tools to use (grep, read, test run), what order, how to record findings. Discrepancy classification defined (bug vs drift vs gap). Parallel execution strategy specified. |
| **B** | Execution strategy exists but methodology is implicit ("read the file and check"). |
| **C** | Checklist only, no methodology. |
| **D** | No structure beyond a flat list. |
| **F** | Not a usable audit document. |

---

## Scoring Summary

| Criterion | Current Grade | Target | Notes |
|-----------|:---:|:---:|-------|
| R1. Spec Traceability | **A** | A | All sub-sections mapped, Appendix A cross-ref index present |
| R2. Specificity | **A** | A | All guards have exact boolean expressions, all fields have types/defaults |
| R3. Init-Then-Guard | **A** | A | All 5 sites have separate init + guard checkboxes with ordering |
| R4. Interactions | **A** | A | 7 compound guards, 4 hierarchical chains, 3 interaction matrices |
| R5. AC Mapping | **A** | A | All 48 ACs mapped with test names, AC32/AC34 gaps documented |
| R6. Behavioral Changes | **A** | A | All 9 changes with affected code + migration checks |
| R7. Edge Cases | **A** | A | All 10 edge cases with dedicated checkboxes |
| R8. File Inventory | **A** | A | All 35 files with expected changes |
| R9. Methodology | **A** | A | Discrepancy classification, execution DAG, tool instructions |

**Ship criteria:** All A. ✓ Verified 2026-02-26.
