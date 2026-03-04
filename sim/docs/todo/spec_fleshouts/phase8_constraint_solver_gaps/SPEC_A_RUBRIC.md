# Spec A — Solver Param & Margin Completeness: Spec Quality Rubric

Grades the Spec A spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

**Spec A scope:** Two "finish items" that complete the constraint parameter
lifecycle for existing but partially-wired features:
- **DT-23**: Per-DOF friction loss solver params — verify multi-DOF fan-out,
  add non-default test coverage, verify tendon friction solref end-to-end.
- **DT-33**: Tendon `margin` attribute — add `tendon_margin` model field,
  wire builder, replace 4 hardcoded `< 0.0` checks in assembly.rs with
  margin-aware checks.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Spec A is part of the Phase 8 umbrella. Empirical verification confirms
the umbrella's scope is accurate with minor refinements.

| Umbrella claim | MuJoCo reality | Action |
|----------------|----------------|--------|
| DT-23: "fields may partially exist" | Full pipeline exists: `dof_solref`/`dof_solimp` in Model (model.rs:233,236), parsing (parser.rs:669-678), defaults cascade (defaults.rs:209-213), builder fan-out (builder/joint.rs:160-163 inside `for i in 0..nv`), assembly wiring (assembly.rs:391-392). | **In scope** as verification + test coverage only. No code changes expected for DT-23 core pipeline. |
| DT-23: "verify multi-DOF fan-out" | Builder loops `for i in 0..nv` and pushes `dof_solref` per-DOF (joint.rs:160-163). MuJoCo does same: `for (int j1=0; j1<pj->nv(); j1++) { mjuu_copyvec(m->dof_solref+mjNREF*dofadr, ...) }`. | **In scope**: add test confirming ball (3 DOFs) and free (6 DOFs) all get parent joint's `solreffriction`. |
| DT-23: "verify tendon friction solref end-to-end" | `tendon_solref_fri`/`tendon_solimp_fri` in Model (model.rs:700-702), parsed (parser.rs:3649-3658), defaults cascade (defaults.rs:638-642), builder (tendon.rs:45-48), assembly (assembly.rs:418-419). Pipeline complete. | **In scope** as verification. Add test with non-default tendon `solreffriction`. |
| DT-33: "parsing + defaults exist, model field missing" | Confirmed: `MjcfTendon.margin` parsed (parser.rs:3646), `MjcfTendonDefaults.margin` cascaded (defaults.rs:644-645). `tendon_margin` does NOT exist in Model. 4 hardcoded `< 0.0` at assembly.rs:150,154,562,586. | **In scope**: add Model field, wire builder, fix 4+2 sites in assembly.rs. |
| DT-33: "4 hardcoded sites" | Actually 4 hardcoded `< 0.0` checks (150,154,562,586) PLUS 2 hardcoded `0.0` margin args to `finalize_row!` (573,597). Total: 6 sites. | **In scope**: umbrella undercounted by 2 finalize_row margin args. |

**Final scope:**
1. DT-23: Verify per-DOF friction solver param pipeline end-to-end (parsing →
   defaults → builder → model → assembly). Add tests for multi-DOF fan-out
   (ball, free) and non-default values. Verify tendon friction solref
   end-to-end.
2. DT-33: Add `tendon_margin: Vec<f64>` to Model. Wire in builder. Replace
   4 hardcoded `< 0.0` with `< model.tendon_margin[t]` and 2 hardcoded
   `0.0` finalize_row margin args with `model.tendon_margin[t]`.

---

## Empirical Ground Truth

### EGT-1: MuJoCo tendon limit uses `tendon_margin` with `dist < margin` check

**MuJoCo C source:** `mj_instantiateLimit()` in `engine_core_constraint.c`
(lines ~1197-1229).

```c
// Tendon limits
for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_limited[i]) {
      value = d->ten_length[i];
      margin = m->tendon_margin[i];

      for (int side=-1; side <= 1; side+=2) {
        dist = side * (m->tendon_range[2*i+(side+1)/2] - value);

        if (dist < margin) {
          // ... create constraint row with &dist and &margin passed to
          // mj_addConstraint()
        }
      }
    }
}
```

**Key behavioral facts:**
- `margin = m->tendon_margin[i]` — per-tendon, resolved at compile time
- Default value: `0.0` (field default in `mjmodel.h`)
- Activation check: `dist < margin` (NOT `dist < 0.0`)
- Both `dist` and `margin` are passed to `mj_addConstraint()`, which stores
  them in `efc_pos` and `efc_margin` respectively
- When margin > 0: constraint activates BEFORE the limit is violated, creating
  a pre-activation zone where the impedance ramps up smoothly
- Side convention: `side=-1` → lower limit (`dist = length - limit_min`),
  `side=+1` → upper limit (`dist = limit_max - length`)
- Jacobian: `jac = -side * ten_J` (same sign convention as joint limits)

### EGT-2: MuJoCo DOF friction loss reads `dof_solref`/`dof_solimp` directly

**MuJoCo C source:** `getsolparam()` in `engine_core_constraint.c`
(lines ~1430-1440).

```c
case mjCNSTR_FRICTION_DOF:
    mju_copy(solref, m->dof_solref + mjNREF*id, mjNREF);   // direct read
    mju_copy(solimp, m->dof_solimp + mjNIMP*id, mjNIMP);   // no fallback
    break;
```

**Key behavioral facts:**
- No runtime fallback chain — all defaults resolved at compile time
- Direct array read from per-DOF model fields
- `mj_instantiateFriction()` passes `0, 0` for pos/margin (friction loss has
  no positional activation — it's always active when `frictionloss > 0`)

### EGT-3: MuJoCo compile-time default chain for DOF friction solver params

**MuJoCo MJCF compiler** (CopyTree in `user_model.cc`):

```
mj_defaultSolRefImp()        → [0.02, 1.0] / [0.9, 0.95, 0.001, 0.5, 2.0]
<default><joint>             → overrides if solreffriction set
<joint solreffriction="..."> → overrides if set
CopyTree(): for each DOF of joint:
    m->dof_solref[dof_adr] = final_value  (per-DOF copy)
```

**Multi-DOF fan-out:**
```c
for (int j1 = 0; j1 < pj->nv(); j1++) {
    mjuu_copyvec(m->dof_solref + mjNREF*dofadr, pj->solref_friction, mjNREF);
    mjuu_copyvec(m->dof_solimp + mjNIMP*dofadr, pj->solimp_friction, mjNIMP);
    dofadr++;
}
```

Ball joint: 3 DOFs all get identical values from parent joint's
`solreffriction`. Free joint: 6 DOFs all get identical values.

### EGT-4: Both counting and instantiation phases need margin-aware checks

**MuJoCo C source:** `mj_nl()` counting function in `engine_core_constraint.c`
uses the same `dist < margin` check as `mj_instantiateLimit()`:

```c
// mj_nl() tendon limit counting:
margin = m->tendon_margin[i];
for (int side=-1; side <= 1; side+=2) {
    dist = side * (m->tendon_range[2*i+(side+1)/2] - value);
    if (dist < margin) {
        nl += mj_addConstraintCount(m, 1, m->ten_J_rownnz[i]);
    }
}
```

**CortenForge assembly.rs** Phase 1 (counting):
- Line 150: `if length - limit_min < 0.0` — hardcoded, should be `< margin`
- Line 154: `if limit_max - length < 0.0` — hardcoded, should be `< margin`

**CortenForge assembly.rs** Phase 3e (instantiation):
- Line 562: `if dist_lower < 0.0` — hardcoded, should be `< margin`
- Line 586: `if dist_upper < 0.0` — hardcoded, should be `< margin`

**finalize_row! margin args** (instantiation):
- Line 573: `0.0` → should be `model.tendon_margin[t]`
- Line 597: `0.0` → should be `model.tendon_margin[t]`

The counting phase and instantiation phase MUST agree on which rows are
emitted. If the counting phase uses `< 0.0` but instantiation uses
`< margin`, the pre-allocated buffer size will be wrong when margin > 0.
MuJoCo uses the identical `dist < margin` check in both `mj_nl()` and
`mj_instantiateLimit()`.

**Reference pattern:** Joint limits (Phase 7 Spec B) correctly use
`model.jnt_margin[jnt_id]` in both counting (lines 111, 115) and
instantiation (lines 447, 510, 456, 478) phases.

### EGT-5: CortenForge DOF friction fan-out is already correct

**builder/joint.rs:144-163** (inside `for i in 0..nv` loop):
```rust
self.dof_solref
    .push(joint.solreffriction.unwrap_or(DEFAULT_SOLREF));
self.dof_solimp
    .push(joint.solimpfriction.unwrap_or(DEFAULT_SOLIMP));
```

This pushes per-DOF for multi-DOF joints, matching MuJoCo's CopyTree
behavior (EGT-3). Verified: the loop iterates `0..nv` where `nv` is the
joint's DOF count (1 for hinge/slide, 3 for ball, 6 for free).

### EGT-6: CortenForge tendon margin parsed but not wired to Model

- **Parsed**: `parser.rs:3646` — `tendon.margin = parse_float_attr(e, "margin")`
- **Defaults cascade**: `defaults.rs:644-645` — margin cascaded in
  `apply_to_tendon()`
- **Model field**: Does NOT exist — `tendon_margin` not in model.rs
- **Builder**: `tendon.rs` does NOT push `tendon_margin` (skipped between
  `tendon_frictionloss` at line 43 and `tendon_solref_fri` at line 45)
- **Assembly**: 4 checks hardcode `< 0.0` instead of `< margin`

Gap: parse → defaults → **[missing: builder push → build.rs transfer → model → assembly]**

**Six separate locations must be updated for the field to work:**
1. `model.rs` — declare `tendon_margin: Vec<f64>` field on Model struct
2. `model_init.rs` — initialize `tendon_margin: vec![]` in `Model::empty()`
3. `builder/mod.rs` — declare `pub(crate) tendon_margin: Vec<f64>` on the
   ModelBuilder struct (~line 623, after `tendon_solimp_lim`)
4. `builder/init.rs` — initialize `tendon_margin: vec![]` in
   `ModelBuilder::new()` (~line 60 area, where `jnt_margin: vec![]` is)
5. `builder/tendon.rs` — push `tendon.margin.unwrap_or(0.0)` (~line 44)
6. `builder/build.rs` — transfer `tendon_margin: self.tendon_margin` to
   Model struct construction (~line 300-324)

Forgetting ANY ONE of these 6 locations creates a silent bug or compile
failure. Note: Model derives `Clone` and `Debug` automatically
(`#[derive(Debug, Clone)]` at model.rs:42), so no manual trait impls need
updating. ModelBuilder has no derives that would auto-include the field —
it must be explicitly declared and initialized.

### EGT-7: CortenForge tendon friction solver params fully wired

- **Model**: `tendon_solref_fri` (model.rs:700), `tendon_solimp_fri`
  (model.rs:702)
- **Builder**: `tendon.rs:45-48` — `tendon.solreffriction.unwrap_or(DEFAULT_SOLREF)`
- **Assembly**: `assembly.rs:418-419` — `model.tendon_solref_fri[t]`,
  `model.tendon_solimp_fri[t]`
- **Parsing**: `parser.rs:3649-3658`
- **Defaults**: `defaults.rs:638-642`

Full pipeline is complete. DT-23 should verify with tests.

### EGT-8: Downstream impedance/aref formulas correctly use margin

**Stress-test finding:** The `finalize_row!` macro passes `margin_val` to
two downstream functions:

1. **`compute_impedance()`** (impedance.rs:48-107): Uses
   `violation = (pos - margin).abs()` then `x = violation / width`. Matches
   MuJoCo's `getimpedance()` formula: `x = (pos - margin) / solimp[2]`.

2. **`compute_aref()`** (impedance.rs:196-202): Uses
   `aref = -B * vel - K * imp * (pos - margin)`. Matches MuJoCo's aref.

3. **`compute_kbip()`** (impedance.rs:144-192): Computes `(K, B)` from
   `solref`/`solimp` — independent of margin. No changes needed.

**Implication:** Wiring the correct `tendon_margin[t]` value through
`finalize_row!` will automatically produce correct impedance ramp-up and
reference acceleration for tendon limits. No changes needed in impedance.rs.

**Runtime validation (EGT-2 supplement):** `compute_impedance()` clamps
solimp values (impedance.rs:63-71) matching MuJoCo's `getsolparam()` runtime
validation. `compute_kbip()` checks `DISABLE_REFSAFE` and clamps
`solref[0] >= 2*timestep` (impedance.rs:155-176). Both are already correct —
no changes needed for Spec A.

### Codebase Context

| Item | File | Line(s) | Status |
|------|------|---------|--------|
| Tendon limit counting (hardcoded `< 0.0`) | assembly.rs | 150, 154 | **Fix** → `< margin` |
| Tendon limit instantiation (hardcoded `< 0.0`) | assembly.rs | 562, 586 | **Fix** → `< margin` |
| Tendon limit finalize_row margin (hardcoded `0.0`) | assembly.rs | 573, 597 | **Fix** → `tendon_margin[t]` |
| `finalize_row!` macro definition | assembly.rs | 205-258 | **Read** — accepts `$margin:expr` |
| `jnt_margin` field (pattern reference) | model.rs | 215 | **Pattern** for `tendon_margin` |
| `jnt_margin` counting usage (pattern) | assembly.rs | 109, 111, 115, 130 | **Pattern** for counting phase |
| `jnt_margin` instantiation usage (pattern) | assembly.rs | 447, 456, 465, 478, 487, 510, 512, 528 | **Pattern** for instantiation phase |
| `dof_solref`/`dof_solimp` fields | model.rs | 233, 236 | **Verify** existing |
| DOF friction fan-out | builder/joint.rs | 144, 160-163 | **Verify** in `for i in 0..nv` loop |
| DOF friction assembly wiring | assembly.rs | 391-392 | **Verify** `model.dof_solref[dof_idx]` |
| `tendon_solref_fri`/`tendon_solimp_fri` | model.rs | 700, 702 | **Verify** existing |
| Tendon friction assembly wiring | assembly.rs | 418-419 | **Verify** existing |
| ModelBuilder struct (missing margin field) | builder/mod.rs | ~623 | **Fix** — add `pub(crate) tendon_margin: Vec<f64>` |
| ModelBuilder::new() init (missing margin) | builder/init.rs | ~60 | **Fix** — add `tendon_margin: vec![]` (pattern: `jnt_margin: vec![]` at same line) |
| Tendon builder push (missing margin) | builder/tendon.rs | ~44 | **Fix** — add `self.tendon_margin.push(...)` |
| Builder→Model transfer (missing margin) | builder/build.rs | ~300-324 | **Fix** — add `tendon_margin: self.tendon_margin` |
| Model init (no `tendon_margin`) | model_init.rs | ~102 | **Fix** — add init |
| Model struct derives Clone/Debug | model.rs | 42 | **Safe** — no manual impl, field auto-included |
| `efc_margin` downstream consumers | data.rs, assembly.rs | 323, 217 | **Write-only** in production — solvers don't read it directly; margin affects behavior via impedance/aref |
| All 15 `finalize_row!` calls audited | assembly.rs | various | **Verified** — only tendon limits (573, 597) pass wrong margin; all other types correct |
| Margin tests T8-T11 (joint only) | assembly.rs | 904-1057 | **Pattern** for tendon margin tests |
| Friction loss integration tests | unified_solvers.rs | 80, 83, 95, 108, 147 | **Verify** friction param coverage |
| `MjcfTendon.margin` (parsed) | types.rs | 2808 | **Exists** ✓ |
| `MjcfTendonDefaults.margin` (cascaded) | defaults.rs | 644-645 | **Exists** ✓ |

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest. If P1 is not A+, do not
> proceed to grading other criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites all of: (1) `mj_instantiateLimit()` in `engine_core_constraint.c` — tendon limit section with `margin = m->tendon_margin[i]` and `dist < margin` check, both lower/upper side convention, Jacobian sign `-side * ten_J`, margin passed to `mj_addConstraint()` for impedance; (2) `mj_instantiateFriction()` in `engine_core_constraint.c` — DOF loop and tendon loop, pos=0, margin=0 for friction rows; (3) `getsolparam()` in `engine_core_constraint.c` — `mjCNSTR_FRICTION_DOF` reads `m->dof_solref[mjNREF*id]` directly, `mjCNSTR_FRICTION_TENDON` reads `m->tendon_solref_fri[mjNREF*id]`, `mjCNSTR_LIMIT_TENDON` reads `m->tendon_solref_lim[mjNREF*id]`; (4) CopyTree per-DOF fan-out for multi-DOF joints (ball=3, free=6); (5) `mj_defaultSolRefImp()` default values `[0.02, 1.0]` / `[0.9, 0.95, 0.001, 0.5, 2.0]`. Edge cases addressed: tendon margin=0 (degenerates to `< 0.0`), negative margin (shrinks activation zone), free joint DOF fan-out (6 identical entries), world body tendon attachment. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics or description based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood, assumed, or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | For DT-33: exact Rust code for all 6 assembly.rs modification sites (4 activation checks + 2 finalize_row margin args). Model field declaration with type, default, and doc comment matching `jnt_margin` pattern. Builder push line with `unwrap_or(0.0)`. model_init.rs initialization. For DT-23: verification checklist with exact file:line for each pipeline stage (parse → defaults → builder → model → assembly) and the specific assertion to make at each stage. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved or deferred. |
| **C** | Skeleton only — "implement this somehow." |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo and provides the correct translation.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table present covering: (1) MuJoCo `tendon_margin[ntendon]` flat array → CortenForge `tendon_margin: Vec<f64>` (identical semantics); (2) MuJoCo `tendon_range[2*i+...]` → CortenForge `tendon_range[t]: (f64, f64)` tuple (side convention mapped); (3) MuJoCo `dof_solref[mjNREF*id]` flat array → CortenForge `dof_solref[dof_idx]: [f64; 2]` (indexed by DOF, same semantics); (4) MuJoCo CopyTree fan-out → CortenForge builder `for i in 0..nv` loop. Naming convention: `tendon_margin` matches `jnt_margin` pattern (both use `{element}_margin`). |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly."

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has three-part structure: (1) concrete model configuration (joint type, limit range, margin value, frictionloss value, solreffriction value), (2) exact expected value or behavior (constraint row count, margin field value, solref values at specific DOF indices), (3) what to check (`data.efc_margin[row]`, `model.dof_solref[dof_idx]`, row count). DT-33 ACs include: margin=0 regression (identical to pre-change), margin>0 pre-activation (row emitted when `dist < margin` but `dist > 0`), finalize_row receives correct margin value. DT-23 ACs include: ball joint (3 DOFs all identical solref), free joint (6 DOFs), defaults cascade (class override), non-default dynamics difference. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague ("output should change"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test. Negative cases tested.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory includes: tendon margin=0 (regression — produces identical behavior to pre-change `< 0.0`), margin>0 pre-activation (row emitted when dist is positive but < margin), margin<0 (valid — shrinks activation zone, constraint only fires when limit violated by more than |margin|), large margin overlap (margin > (limit_max - limit_min)/2 → both lower and upper limits simultaneously active, producing 2 constraint rows), disabled tendon limits + margin (no rows emitted), ball joint with 3 identical dof_solref entries, free joint with 6, defaults cascade override of solreffriction, non-default solreffriction produces different constraint dynamics. Negative cases: `tendon_limited=false` ignores margin, `DISABLE_LIMIT` ignores margin, `frictionloss=0` emits no friction row regardless of solreffriction. At least one test uses multi-tendon model to catch indexing bugs. Pattern reference: T8-T11 in assembly.rs (lines 904-1057) for joint margin tests — tendon margin tests should mirror this structure. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous with section numbering (S1, S2, ...). Each section states what it requires from prior sections. DT-33 depends on DT-23 verification being complete (both touch assembly.rs but different sections — friction loss at lines 379-430 vs tendon limits at lines 545-607). Prerequisites include: Phase 7 Spec B jnt_margin pattern (already landed), Phase 8 umbrella (commit `4574a03`), DT-32 naming conformance (commit `9f9cf9f`). No circular dependencies. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. DT-33 behavioral change: tendon limits with margin>0 will now activate earlier than before — this moves **toward** MuJoCo conformance. DT-23: no behavioral change (verification only + new tests). Existing test impact: T8-T11 joint margin tests (assembly.rs:904-1057) should be unaffected (joint limits, not tendon limits). Friction loss integration tests (unified_solvers.rs) should be unaffected (use default solref values). No data staleness guards (no `EXPECTED_SIZE` constants in affected code). Crates affected: sim-core (model, assembly), sim-mjcf (builder). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "tendon margin" (not "tendon limit margin" or "activation margin"). Field name `tendon_margin` used identically everywhere. File paths in Specification match Files Affected. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. Site counts: "6 sites" (4 activation checks + 2 finalize_row args) consistent between MuJoCo Reference, Algorithm, and Test Plan sections. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Pipeline Lifecycle Completeness

> Both DT-23 and DT-33 follow the parse → defaults cascade → builder →
> model → assembly pipeline. Every stage must be addressed (verified or
> implemented) with no gaps.

| Grade | Bar |
|-------|-----|
| **A+** | For each item (DT-23, DT-33), a pipeline table lists every stage with: (1) file:line, (2) current state (exists/missing), (3) action (verify/implement/fix), (4) expected value at that stage. DT-23 pipeline: all 5 stages verified with test assertions at builder output and assembly consumption. DT-33 pipeline: parse ✓, defaults ✓, ModelBuilder struct field (implement in builder/mod.rs:~623), ModelBuilder init (implement in builder/init.rs:~60), builder push (implement in builder/tendon.rs:~44), build.rs transfer (implement at ~line 300-324), model field (implement in model.rs), model_init (implement in model_init.rs), assembly (fix 6 sites). Six locations must each be addressed for the field to reach assembly.rs — forgetting any one creates a silent bug or compile failure. No stage left with "assumed working" — every stage has an explicit verification action or implementation step. Pattern reference: `jnt_margin` pipeline lifecycle (Phase 7 Spec B §64a) — `jnt_margin` touched the same 6 locations. |
| **A** | Pipeline is addressed end-to-end. Minor stage left without explicit verification. |
| **B** | Some stages hand-waved or "assumed to work." |
| **C** | Pipeline not discussed as a lifecycle. |

**Boundary with P2:** P2 grades whether the *algorithm* at each stage is
specified unambiguously. P9 grades whether the spec *identifies and
addresses all stages* of the pipeline — completeness, not correctness.

### P10. Cross-Subsystem Parity

> DT-33 (tendon margin) must replicate the jnt_margin pattern exactly. Any
> structural difference from the joint margin implementation must be
> explicitly justified.

| Grade | Bar |
|-------|-----|
| **A+** | Side-by-side comparison table showing: (1) `jnt_margin` model field declaration vs `tendon_margin` declaration (both `Vec<f64>`, both default `0.0`), (2) builder push (`jnt_margin` at builder/joint.rs vs `tendon_margin` at builder/tendon.rs), (3) counting phase usage pattern (assembly.rs joint lines 111,115 vs tendon lines 150,154), (4) instantiation usage pattern (assembly.rs joint lines 447,456,478,510 vs tendon lines 562,573,586,597), (5) test structure (T8-T11 joint margin tests vs corresponding tendon margin tests). Any structural difference from jnt_margin is explicitly justified (e.g., tendon limits have no ball/free joint type dispatch — just lower/upper sides). |
| **A** | Pattern replicated correctly. Minor structural differences not called out. |
| **B** | Pattern followed loosely. Some parity gaps. |
| **C** | No parity analysis. |

**Boundary with P1:** P1 grades whether the MuJoCo behavior is correctly
described. P10 grades whether the CortenForge implementation pattern is
replicated consistently from the joint margin subsystem.

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific file:line references,
      MuJoCo C function names, and concrete counts (6 sites, 4 activation
      checks, 2 finalize_row args, 3/6 DOFs for ball/free). Two independent
      reviewers would agree on the grade by checking these specific items.

- [x] **Non-overlap:** P1 (MuJoCo reference) vs P10 (cross-subsystem parity)
      boundary: P1 grades MuJoCo behavioral correctness, P10 grades
      CortenForge pattern replication from jnt_margin. P2 (algorithm
      correctness) vs P9 (pipeline completeness) boundary: P2 grades whether
      each step is specified, P9 grades whether all stages are covered.

- [x] **Completeness:** 10 criteria cover: MuJoCo conformance (P1),
      algorithm (P2), conventions (P3), ACs (P4), tests (P5), dependencies
      (P6), blast radius (P7), consistency (P8), pipeline lifecycle (P9),
      cross-subsystem parity (P10). Both DT-23 and DT-33 are addressed in
      every criterion. The "finish item" nature of both tasks (verify +
      complete vs greenfield) is reflected in the bars.

- [x] **Gradeability:** P1→MuJoCo Reference; P2→Specification sections;
      P3→Convention Notes; P4→Acceptance Criteria; P5→Test Plan; P6→
      Prerequisites/Execution Order; P7→Files Affected/Blast Radius; P8→
      cross-cutting; P9→Pipeline tables; P10→Parity comparison table.

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C
      functions (`mj_instantiateLimit`, `mj_instantiateFriction`,
      `getsolparam`, `mj_defaultSolRefImp`, CopyTree fan-out). P4 requires
      MuJoCo-verified expected values. P5 requires at least one MuJoCo
      conformance test per code path.

- [x] **Empirical grounding:** EGT-1 through EGT-8 filled in with verified
      MuJoCo C source excerpts and CortenForge codebase line references.
      Every P1 A+ bar item has a corresponding EGT entry:
      `mj_instantiateLimit` → EGT-1, `getsolparam` → EGT-2, CopyTree →
      EGT-3, counting phase → EGT-4, fan-out → EGT-5, tendon margin gap →
      EGT-6, tendon friction params → EGT-7, downstream impedance/aref →
      EGT-8. Stress test (Rev 2) added EGT-8 and expanded EGT-6 with
      build.rs transfer stage and Model derive confirmation.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) — algorithm at each stage |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Pipeline Lifecycle tables (DT-23 and DT-33 separate tables) |
| P10 | Cross-Subsystem Parity table (jnt_margin vs tendon_margin comparison) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Pipeline Lifecycle Completeness | | |
| P10. Cross-Subsystem Parity | | |

**Overall: — (Rev 4, post stress test round 3)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope Adjustment | Umbrella says "4 hardcoded sites" but there are actually 6 (4 activation checks + 2 finalize_row margin args) | Rubric self-audit (EGT-4) | Documented in Scope Adjustment table; P2 A+ bar requires all 6 sites | Rev 1 |
| R2 | P9 | Pipeline missing build.rs transfer stage — builder push and Model construction are separate locations; forgetting transfer is a silent bug | Stress test (codebase audit) | Added build.rs:~300-324 to EGT-6, Codebase Context table, and P9 A+ bar. Pipeline now has 7 stages: parse → defaults → builder push → build.rs transfer → model field → model_init → assembly | Rev 2 |
| R3 | P9 | Missing confirmation that Model derives Clone/Debug (no manual impl risk) | Stress test (codebase audit) | Added to EGT-6 and Codebase Context table: `#[derive(Debug, Clone)]` at model.rs:42, no manual trait impls | Rev 2 |
| R4 | P1/P7 | Missing confirmation that downstream impedance/aref formulas already correctly use margin — no solver changes needed | Stress test (MuJoCo ref audit) | Added EGT-8: `compute_impedance()`, `compute_aref()`, `compute_kbip()` all correctly use/ignore margin. Wiring correct margin values flows through automatically | Rev 2 |
| R5 | P7 | No serialization breaking risk documented | Stress test (codebase audit) | Model has no serde derives — adding field has zero serialization impact. Not worth a dedicated EGT entry; noted in gap log | Rev 2 |
| R6 | P9 | ModelBuilder struct definition in builder/mod.rs:~623 not mentioned — field must be declared on struct AND pushed in tendon.rs AND transferred in build.rs (5 locations total, not 4) | Stress test round 2 (codebase audit) | Updated EGT-6 to list 5 locations. Updated P9 A+ bar to require ModelBuilder struct field. Updated Codebase Context table. | Rev 3 |
| R7 | P5 | Large margin overlap edge case missing — when margin > (limit_max - limit_min)/2, both lower and upper limits fire simultaneously, producing 2 rows. MuJoCo handles this without guard. | Stress test round 2 (MuJoCo edge case analysis) | Added to P5 A+ bar edge case inventory: "large margin overlap (margin > (limit_max - limit_min)/2 → both lower and upper limits simultaneously active, producing 2 constraint rows)" | Rev 3 |
| R8 | P7 | `efc_margin` is write-only in production code — solvers don't read it directly. Margin affects behavior through impedance/aref only. Missing from blast radius analysis. | Stress test round 2 (efc_margin consumer audit) | Added to Codebase Context table. Confirms no solver-side risk from changing tendon margin values. | Rev 3 |
| R9 | P2 | All 15 finalize_row! calls not audited — only tendon limits were known to be wrong; round 2 confirmed all other constraint types pass correct margin | Stress test round 2 (complete finalize_row audit) | Added to Codebase Context table: "All 15 finalize_row! calls audited — only tendon limits (573, 597) pass wrong margin" | Rev 3 |
| R10 | P9 | `builder/init.rs` ModelBuilder::new() is a separate file from `builder/mod.rs` struct declaration — initialization at ~line 60 is location #6 (not covered by mod.rs struct field addition) | Stress test round 3 (codebase audit) | Updated EGT-6 to 6 locations. Updated P9 A+ bar. Pattern confirmed: `jnt_margin: vec![]` at init.rs:60 | Rev 4 |
| R11 | — (out of scope) | `compute_kbip()` does not validate mixed-sign solref `(solref[0] > 0) ^ (solref[1] > 0)` — MuJoCo's `getsolparam()` warns and replaces with default. Gap affects ALL constraint types, not specific to Spec A. | Stress test round 3 (MuJoCo edge case) | Out of scope for Spec A (param wiring, not validation). Noted for future DT item. | — |
