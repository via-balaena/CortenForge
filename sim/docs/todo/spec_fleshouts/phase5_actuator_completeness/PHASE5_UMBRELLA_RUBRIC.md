# Phase 5 — Actuator Completeness: Umbrella Spec Rubric

Grades the Phase 5 umbrella spec on 8 criteria. Target: A+ on every criterion
before sub-specs are written. A+ means "a Claude session starting fresh could
write all 4 sub-specs without asking a single coordination question."

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).

---

## Conformance Context

**MuJoCo conformance is the cardinal goal of Phase 5 and of CortenForge as a
whole.** Every sub-spec exists to close a specific gap between "what MuJoCo does"
and "what CortenForge does." The umbrella's job is to coordinate these specs so
they collectively achieve actuator conformance without conflicts. While this
rubric grades coordination quality (not MuJoCo fidelity — that's graded by each
sub-spec's P1 criterion), the umbrella must establish the conformance mandate
that every sub-spec inherits: cite the MuJoCo C source, verify expected values
against MuJoCo, match MuJoCo's behavior exactly.

---

## Why Custom Criteria

The standard P1–P8 criteria grade *individual task specs* — MuJoCo reference
fidelity, algorithm completeness, etc. An umbrella spec doesn't describe
algorithms or MuJoCo behavior; its job is to **coordinate multiple specs that
share files, APIs, and conventions** so they don't conflict — while ensuring
every sub-spec holds to the conformance standard.

The 8 criteria below grade the umbrella's coordination quality. The sub-specs
will each have their own P1–P8 rubrics for technical content, with P1 (MuJoCo
Reference Fidelity) as the cardinal criterion.

---

## Criteria

### U1. Scope & Boundary Clarity

> Every Phase 5 task is assigned to exactly one spec or T1 item. No task is
> orphaned. No task is claimed by two specs. The boundary between specs is
> unambiguous — for any change an implementer might make, they can point to
> exactly one spec that owns it. Each task maps to a specific MuJoCo
> conformance gap being closed.

| Grade | Bar |
|-------|-----|
| **A+** | All 10 Phase 5 tasks (DT-6, DT-8, DT-9, DT-56, DT-57, DT-58, DT-59, DT-77, §61, §63) explicitly assigned. Assignment table present. For each spec, a 1–2 sentence scope statement makes the boundary unambiguous and cites the MuJoCo C functions being ported. No "this could be in Spec A or Spec B" situations. Conformance mandate stated for sub-spec authors. |
| **A** | All tasks assigned. Minor boundary ambiguity between two specs. |
| **B** | Most tasks assigned. One or two tasks unclear on which spec owns them. |
| **C** | Tasks listed but not clearly assigned to specs. |

### U2. Dependency Graph & Ordering

> The implementation order is a valid topological sort of the dependency DAG.
> No spec starts before its prerequisites land. T1 items are sequenced before
> the specs that depend on their changes. The ordering rationale is explicit
> (not just "do A then B" but "A before B because B reads the `acc0` field
> that A populates").

| Grade | Bar |
|-------|-----|
| **A+** | Dependency DAG shown (text or ASCII). Every edge justified with the specific code/data dependency. T1 prerequisites (DT-6, §63) sequenced first with explicit rationale. No circular dependencies. Parallelizable specs identified. A reader can derive the exact `git` branch/PR ordering from this section alone. |
| **A** | Order is correct and justified. Minor parallelization opportunity missed. |
| **B** | Order listed but rationale is vague ("do this first"). |
| **C** | No ordering discussion, or ordering has a dependency violation. |

### U3. File Ownership Matrix

> For every file touched by 2+ specs, the umbrella assigns ownership per
> phase: who touches it first, what they change, and what the file looks like
> when the next spec picks it up. This prevents merge conflicts and stale-API
> surprises.

| Grade | Bar |
|-------|-----|
| **A+** | Matrix present: File × Spec, with change description per cell. Every multi-touch file (identified from the overlap analysis: `enums.rs`, `model.rs`, `model_init.rs`, `builder/actuator.rs`, `forward/actuation.rs`, `forward/muscle.rs`) has a clear ownership sequence. For each handoff, the "state after Spec X" is described so Spec Y knows what it inherits. |
| **A** | Matrix present. Most multi-touch files covered. One handoff underspecified. |
| **B** | Some files listed as shared, but no ownership sequence. |
| **C** | No multi-touch file analysis. |

### U4. API Contract Definitions

> When Spec X changes a function signature, struct, or enum that Spec Y
> depends on, the umbrella defines the contract: the new signature, new
> fields, or new variants. Sub-specs are written against these contracts,
> not against the current (pre-Phase 5) codebase.

| Grade | Bar |
|-------|-----|
| **A+** | Every cross-spec API dependency identified. Contracts specified: (1) `compute_muscle_params()` signature after Spec A — so Spec C knows what to integrate with, (2) `ActuatorTransmission` enum after Spec B — so slider-crank runtime code targets the right variant, (3) `ActuatorDynamics` enum after Spec C — so no variant ID collision with existing code, (4) `dynprm` array type after §63 — so all specs use `[f64; 10]` not `[f64; 3]`. Contracts include Rust type signatures, not just prose. |
| **A** | Major contracts defined. One cross-spec dependency missing an explicit contract. |
| **B** | Some contracts noted, but missing Rust signatures — sub-spec authors must guess. |
| **C** | No cross-spec API analysis. |

### U5. Shared Convention Registry

> Naming patterns, array sizing conventions, enum variant ordering, and
> default value conventions are defined once in the umbrella. Sub-specs
> reference these conventions instead of inventing their own. Conventions
> match MuJoCo where possible; deviations are justified.

| Grade | Bar |
|-------|-----|
| **A+** | Registry covers: (1) new model array naming pattern (`actuator_{name}` following existing convention, with specific names for each new field), (2) enum variant ordering convention (append after existing variants, alphabetical within new additions, or other rule), (3) default values for new arrays (zero, `None`, sentinel — with rationale), (4) MJCF attribute naming convention (match MuJoCo exactly vs adapt to Rust idiom — with decision per attribute). |
| **A** | Major conventions defined. One category missing or underspecified. |
| **B** | Some conventions noted, but inconsistent or incomplete. |
| **C** | No shared conventions — each sub-spec would invent its own. |

### U6. Cross-Spec Blast Radius

> The umbrella analyzes existing tests that are affected by multiple specs
> and ensures no conflict. Also identifies cases where one spec's behavioral
> change could invalidate another spec's assumptions or expected values.

| Grade | Bar |
|-------|-----|
| **A+** | Cross-spec test impact analyzed: which existing tests are touched by 2+ specs, and in what order the expected-value updates happen. Behavioral interactions identified (e.g., does §63's `dynprm` resize change any runtime values that Spec A's `acc0` computation reads? Does DT-6's `actearly` ordering affect Spec C's Hill muscle activation timing?). For each interaction: "no conflict because X" or "conflict resolved by ordering Y." Current test baseline stated (2,148+ domain tests). |
| **A** | Major interactions analyzed. One cross-spec test conflict unaddressed. |
| **B** | Individual spec blast radii listed, but no cross-spec analysis. |
| **C** | No blast radius discussion. |

### U7. Phase-Level Acceptance Criteria

> The umbrella defines what "Phase 5 complete" means at the aggregate level —
> not per-task ACs (those live in sub-specs), but the phase-level gate that
> says all actuator work is done and integrated. The ultimate gate is
> conformance: CortenForge matches MuJoCo for every actuator feature in scope.

| Grade | Bar |
|-------|-----|
| **A+** | Phase-level ACs defined: (1) all 10 tasks ship-complete (sub-spec ACs met + T1 items verified), (2) aggregate test count increase stated, (3) no regression in existing 2,148+ domain tests, (4) `cargo xtask check` passes, (5) actuator-related conformance coverage statement (which MuJoCo actuator features are now covered vs before Phase 5), (6) conformance verification mandate — each sub-spec has at least one AC with MuJoCo-verified expected values. A reader can determine "is Phase 5 done?" from this section alone. |
| **A** | Phase-level completion criteria present. Minor gap in coverage or conformance mandate. |
| **B** | "All sub-specs done" is the only criterion — no aggregate or conformance verification. |
| **C** | No phase-level acceptance criteria. |

### U8. Internal Consistency

> No contradictions within the umbrella. Task counts match between sections.
> File lists match between the ownership matrix and the blast radius analysis.
> Spec names are consistent throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Task count (10) matches between scope table and assignment table. File list in ownership matrix matches blast radius section. Spec names/labels are identical everywhere (no "Spec A" in one place and "acc0 spec" in another without linking them). Dependency edges in the DAG match the ordering section. Convention registry items are referenced in the correct sub-spec scope descriptions. |
| **A** | Consistent. One minor naming inconsistency. |
| **B** | Some sections use different labels for the same spec or different task counts. |
| **C** | Contradictions between sections. |

---

## Rubric Self-Audit

- [ ] **Specificity:** Every A+ bar names the specific Phase 5 tasks, files,
      and specs — not generic "all tasks assigned."
- [ ] **Non-overlap:** U1 (scope) and U3 (file ownership) are distinct:
      U1 grades task-to-spec assignment, U3 grades file-level coordination.
      U4 (API contracts) and U5 (conventions) are distinct: U4 grades
      cross-spec function signatures, U5 grades naming/sizing/ordering rules.
- [ ] **Completeness:** Could the umbrella be A+ on all 8 criteria but still
      leave a sub-spec author confused about coordination? If yes, a criterion
      is missing.
- [ ] **Gradeability:** Each criterion maps to specific umbrella sections.
- [ ] **Conformance primacy:** The umbrella establishes that MuJoCo conformance
      is the cardinal goal. Sub-spec scope statements reference MuJoCo C source.
      Phase-level ACs include conformance verification. Could a sub-spec author
      read this umbrella and produce a sub-spec that doesn't prioritize
      conformance? If yes, the umbrella is missing something.

### Criterion → Umbrella Spec Section Mapping

| Criterion | Section(s) to Grade |
|-----------|---------------------|
| U1 | Scope, Task Assignment Table, Sub-Spec Scope Statements |
| U2 | Dependency Graph, Implementation Order, T1 Prerequisites |
| U3 | File Ownership Matrix |
| U4 | API Contracts |
| U5 | Shared Conventions |
| U6 | Cross-Spec Blast Radius, Test Impact |
| U7 | Phase-Level Acceptance Criteria |
| U8 | *Cross-cutting — all sections checked for mutual consistency* |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| U1. Scope & Boundary Clarity | **A+** | All 10 tasks assigned in Task Assignment table (10 rows, Deliverable + Rationale columns). 4 sub-spec scope statements with boundary clarifications (e.g., `cranksite`/`slidersite` are SliderCrank attributes, not separate types). Each scope statement cites MuJoCo C functions. Conformance mandate in Scope section. |
| U2. Dependency Graph & Ordering | **A+** | ASCII DAG with step labels. 3-step implementation order table with rationale. 5 dependency edges with specific code references. Parallelizable specs identified (A, B, D in Step 2). Within-step ordering recommendation (Spec A before B) justified by Contract 3. |
| U3. File Ownership Matrix | **A+** | 7 files analyzed (enums.rs, model.rs, model_init.rs, builder/actuator.rs, actuation.rs, muscle.rs, sensor/mod.rs). Each has Order/Deliverable/Change/State-After table. Highest-risk file (muscle.rs) flagged. Conflict assessments per file. |
| U4. API Contract Definitions | **A+** | 4 contracts with Rust signatures. Contract 1: dynprm before/after signature. Contract 2: compute_muscle_params() expected changes + what Spec C needs. Contract 3: landing-order resolution for enum overlap. Contract 4: defensive enum contract. |
| U5. Shared Convention Registry | **A+** | 5 conventions: array naming (table with types/defaults/spec), enum ordering (code examples with clarification that Transmission has no User variant), default values (rules), MJCF naming (exact MuJoCo matches stated), dynprm post-resize (annotated layout). Registry intro states conventions match MuJoCo where possible. |
| U6. Cross-Spec Blast Radius | **A+** | 5 behavioral interactions analyzed with verdicts. Cross-spec test impact table (5 areas). Test count estimates per deliverable (22–34 total). actearly + Hill muscle interaction flagged for Spec C attention. |
| U7. Phase-Level Acceptance Criteria | **A+** | 5 ACs (PH5-AC1 through AC5). AC1 requires MuJoCo-verified values. AC4 requires conformance tests per sub-spec. AC5 includes 8-row before/after conformance coverage table. Phase completion determinable from this section alone. Overarching conformance criterion stated. |
| U8. Internal Consistency | **A+** | Task count (10) matches between Scope and Assignment Table. Spec names consistent throughout (Spec A/B/C/D, T1-a/T1-b). DAG step labels match Implementation Order table. Dependency edges match DAG arrows. File lists in Ownership Matrix match Blast Radius section. Enum ordering in Conventions matches Contract 3/4. |

**Overall: A+**

**Revision 4** — Graded after closing G1–G6 + conformance primacy pass across all sections.

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| G1 | U8 | Step numbering inconsistency: DAG showed T1-a/T1-b in one box but table had them as separate steps 1/2 | Initial grading | Merged into Step 1 (parallel). Renumbered Steps 1→3. Added step labels to DAG. | Rev 2 |
| G2 | U5 | Enum ordering note said "before any future User variant" for `ActuatorTransmission`, but that enum has no `User` variant | Initial grading | Added clarification: "ActuatorTransmission has no User variant — new variants simply appended. ActuatorDynamics has User last — new variants go before it." | Rev 2 |
| G3 | U4 | Contract 3 said "both are in Step 3" — stale reference from pre-renumbering, should be "Step 2" | Fresh-eyes review | Fixed step reference to "Step 2". | Rev 3 |
| G4 | U1/U3 | Spec A scope said "All changes are in muscle.rs and builder/actuator.rs" but file ownership said "No parsing changes" for Spec A in builder/actuator.rs — contradiction. `dampratio` is an MJCF attribute needing parsing. | Fresh-eyes review | Updated scope statement and file ownership entry. Spec A parses `dampratio` in builder/actuator.rs. | Rev 3 |
| G5 | U3 | builder/actuator.rs conflict note only mentioned Specs B and D, but Spec A also adds parsing there | Fresh-eyes review | Updated conflict note to include Specs A, B, and D. | Rev 3 |
| G6 | U6 | `muscle.rs` blast radius said "15 tests" — actual count is 17 tests | Fresh-eyes review | Updated to "17 tests". | Rev 3 |
