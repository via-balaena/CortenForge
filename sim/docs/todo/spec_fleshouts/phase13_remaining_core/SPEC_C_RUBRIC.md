# Composite Body Generation (§46) — Spec Quality Rubric

Grades the Spec C spec on 9 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** P1 (MuJoCo Reference Fidelity)
is the most important criterion — grade it first and hardest.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

The umbrella SESSION_PLAN.md assumed 7 composite types matching older MuJoCo
documentation. MuJoCo 3.4.0 empirical verification reveals a dramatically
different reality:

| Umbrella claim | MuJoCo 3.4.0 reality | Action |
|----------------|---------------------|--------|
| 7 composite types: grid, rope, cable, cloth, box, cylinder, ellipsoid | Only `cable` is non-deprecated. `particle`, `grid`, `rope`, `loop`, `cloth` return deprecation errors with specific replacement messages. `box`, `cylinder`, `ellipsoid` were **never** composite types — they are `flexcomp` types. | Reduce scope to `cable` + deprecation errors |
| Sessions 10+11 split: 3 types part 1, 4 types part 2 | Only `cable` needs implementation; 5 deprecated types need error messages | Sessions 10+11 can be merged into single session |
| `<composite>` generates tendons | Cable type does NOT generate tendons (tendons were only for deprecated `cloth` type) | Drop tendon generation |
| Skin generation for visual deformation | Skin only for box geom cables; complex bicubic interpolation; rendering-only | Defer — no physics impact |
| Plugin support (`mujoco.elasticity.cable`) | Requires plugin system (Spec D) | Defer to post-Spec-D |

**Final scope:**
1. Parse `<composite>` element with all attributes and child elements
2. Cable expansion: body chain, geoms, ball joints, boundary sites, contact excludes, custom text
3. Curve-based vertex generation (LINE, COS, SIN, ZERO shapes)
4. Discrete Bishop frame propagation (`mjuu_updateFrame` equivalent)
5. Deprecation errors for non-cable types matching MuJoCo's exact messages
6. Pipeline integration: expand composites after frame expansion in builder

---

## Empirical Ground Truth

### EGT-1: Cable body generation (MuJoCo 3.4.0)

`count=N` creates `N-1` cable bodies in a linear parent chain.
Each body is a child of the previous one. First body is a child of
the enclosing body (worldbody or parent).

**Naming convention:** `{prefix}B_first`, `{prefix}B_1`, ...,
`{prefix}B_{N-3}`, `{prefix}B_last`.

| count | bodies | names |
|-------|--------|-------|
| 2 | 1 | B_first |
| 4 | 3 | B_first, B_1, B_last |
| 5 | 4 | B_first, B_1, B_2, B_last |
| 6 | 5 | B_first, B_1, B_2, B_3, B_last |

**First body position:** `offset + uservert[0]` (world frame).
**Subsequent body positions:** `[edge_length, 0, 0]` relative to parent
(always along local x-axis after frame rotation).

### EGT-2: Joint, geom, site, exclude generation

**Joints:** `N-1` ball joints (one per body), named `{prefix}J_first`,
`{prefix}J_1`, ..., `{prefix}J_last`. Three `initial` modes:
- `"ball"` (default): all bodies get ball joints
- `"none"`: first body has NO joint (pinned); rest get ball joints
- `"free"`: first body gets free joint (6 DOF, damping=0); rest get ball joints

**Geoms:** `N-1` geoms (one per body), named `{prefix}G0`, `{prefix}G1`, ....
Must be capsule, cylinder, or box. For capsule/cylinder: uses `fromto`
placement `[0,0,0] -> [edge_length,0,0]`. For box: uses pos/size placement.

**Sites:** Exactly 2: `{prefix}S_first` on first body at origin,
`{prefix}S_last` on last body at `[edge_length, 0, 0]`.

**Excludes:** `N-2` contact exclude pairs for adjacent bodies.

**Custom text:** `composite_{prefix}` with data `rope_{prefix}`.

### EGT-3: Deprecated type error messages (from C source)

| Type | Error message |
|------|--------------|
| `particle` | `The "particle" composite type is deprecated. Please use "replicate" instead.` |
| `grid` | `The "grid" composite type is deprecated. Please use "flex" instead.` |
| `rope` | `The "rope" composite type is deprecated. Please use "cable" instead.` |
| `loop` | `The "loop" composite type is deprecated. Please use "flexcomp" instead.` |
| `cloth` | `The "cloth" composite type is deprecated. Please use "shell" instead.` |

### EGT-4: Codebase integration points

| File | Line | What | Action |
|------|------|------|--------|
| `sim/L0/mjcf/src/parser.rs` | 1587+ | `parse_worldbody()` — skips unknown elements | Add `<composite>` handling |
| `sim/L0/mjcf/src/parser.rs` | 1645+ | `parse_body()` — skips unknown elements | Add `<composite>` handling |
| `sim/L0/mjcf/src/types.rs` | 2336 | `MjcfBody` struct | Add `composites: Vec<MjcfComposite>` field |
| `sim/L0/mjcf/src/types.rs` | 3956 | `MjcfModel` struct | (no change — excludes go to `contact.excludes`) |
| `sim/L0/mjcf/src/builder/mod.rs` | 238 | After `expand_frames()` | Add `expand_composites()` call |
| `sim/L0/mjcf/src/builder/frame.rs` | — | Pattern to follow | `expand_composites()` follows same tree-walk pattern |
| `sim/L0/mjcf/src/lib.rs` | — | Module exports | Add `composite` module, update docs |

### EGT-5: Moving frame computation (from `mjuu_updateFrame`)

Discrete Bishop frame propagation along a polyline:
- **First segment:** Initialize frame from `cross(tangent, tnext)` (binormal),
  then `cross(binormal, tangent)` (normal). Returns quaternion + edge length.
- **Subsequent segments:** Rotate previous normal about vertex binormal
  (`cross(tprev, tangent)`) by `atan2(||cross||, dot(tprev, tangent))`.
  This is rotation-minimizing parallel transport.
- **Output:** `frame2quat(tangent, normal, binormal)` → quaternion.

Source: `mjuu_updateFrame()` in `user_util.cc`.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does for `<composite>` — exact
> function names, field names, calling conventions, and edge cases.

| Grade | Bar |
|-------|-----|
| **A+** | Spec cites `mjCComposite::Make()` (dispatch + validation, `user_composite.cc:131-239`), `MakeCable()` (cable entry, `:243-313`), `AddCableBody()` (per-segment body creation, `:317-445`), and `mjuu_updateFrame()` (`user_util.cc`) with source files, line ranges, and exact behavior. Vertex generation formula for all 4 curve shapes (LINE/COS/SIN/ZERO) cited with C code. Body naming convention (B_first/B_{n}/B_last) explicit. Three `initial` modes documented with exact joint behavior for each. Edge cases addressed: deprecated types (all 5 with exact error messages per EGT-3), count validation (count[0]>=2, dim==1), geom type validation (capsule/cylinder/box only), uservert vs count mutual exclusion. |
| **A** | Functions and behavior correct from C source. Minor edge case gaps. |
| **B** | High-level description correct but missing specifics (e.g., naming convention, frame computation). |
| **C** | MuJoCo behavior assumed from docs, not verified against C source. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. Vertex generation,
> frame propagation, and body chain creation are line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Complete Rust code for: (1) curve vertex generation with all 4 shapes, (2) discrete Bishop frame propagation matching `mjuu_updateFrame`, (3) body chain creation with position/quaternion computation matching `AddCableBody`, (4) geom placement (fromto for capsule/cylinder, pos/size for box), (5) joint type selection per `initial` mode, (6) exclude pair generation, (7) deprecation error dispatch. No "see MuJoCo source" gaps. |
| **A** | Algorithm complete. One or two minor details left implicit. |
| **B** | Algorithm structure clear but some steps hand-waved (e.g., frame propagation). |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses MuJoCo→CortenForge convention differences
> for naming, coordinate systems, and data structures.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table present with porting rules for: body naming (`B_first` etc.), geom naming (`G{n}` 0-indexed), joint naming, site naming, quaternion convention (w,x,y,z), exclude representation (body names → `MjcfContactExclude`), custom text mapping. All porting rules verified to preserve correct behavior. |
| **A** | Major conventions documented. Minor mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values
> from MuJoCo 3.4.0 empirical verification (EGT-1, EGT-2).

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete MJCF input, (2) exact expected values from MuJoCo 3.4.0 (body count, names, positions, joint types, geom properties), (3) field to check. At least one AC per major feature (cable generation, deprecation, initial modes, naming). Code-review ACs labeled with specific structural properties. Copy-pasteable into test functions. |
| **A** | ACs testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague. |
| **C** | Aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover cable generation, deprecated types, initial modes, edge cases,
> and naming conventions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory includes: count=2 (minimum), deprecated types (all 5), initial modes (ball/none/free), prefix naming, nested body composite, multiple composites, invalid geom type, count validation errors. At least one MuJoCo conformance test (same model, verify body tree matches MuJoCo's output). Supplementary tests justified. |
| **A** | Good coverage. Minor edge case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering, and pipeline integration are explicit.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Parser changes (S1-S2) before expansion logic (S3) before pipeline integration (S4). Prerequisites stated: frame expansion must precede composite expansion. Cross-spec interactions: Spec D (plugin) explicitly noted as out-of-scope dependency. Skin deferred explicitly. |
| **A** | Order clear. Minor interactions implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched and every existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. `parser.rs` changes scoped to `parse_body()`/`parse_worldbody()` with no effect on existing element parsing. `types.rs` changes add new types (no modification to existing types). `builder/mod.rs` adds one function call. New `builder/composite.rs` is self-contained. Existing test impact: none (composite was previously silently skipped; new behavior adds support without changing existing paths). Test baseline stated. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Terminology uniform.

| Grade | Bar |
|-------|-----|
| **A+** | Naming conventions identical across MuJoCo Reference, Specification, ACs, and Test Plan. Body count formulas consistent (N-1 bodies, N-2 excludes). File paths match between Specification and Files Affected. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. |
| **A** | Consistent. One or two minor inconsistencies. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Expansion Architecture

> Composite expansion happens at the correct pipeline stage, produces valid
> MjcfBody structures, and integrates cleanly with the existing builder flow.

| Grade | Bar |
|-------|-----|
| **A+** | Spec defines: (1) pipeline stage (after frame expansion, before discardvisual/fusestatic) with rationale matching MuJoCo's compiler order, (2) tree manipulation strategy (composites on MjcfBody replaced with generated children), (3) exclude pairs added to MjcfModel.contact.excludes, (4) custom text added to model. Body tree structure after expansion is a valid MjcfBody tree that the builder can process without modification. Pattern follows `expand_frames()` architecture. Error handling for deprecated types prevents expansion. |
| **A** | Architecture correct. Minor integration details implicit. |
| **B** | Architecture described but pipeline stage unclear. |
| **C** | No architecture discussion. |

---

## Rubric Self-Audit

- [x] **Specificity:** Every A+ bar names specific functions (`Make()`,
      `MakeCable()`, `AddCableBody()`, `updateFrame()`), specific data
      (body naming convention, count formula), and specific edge cases
      (deprecated types, initial modes, count validation).

- [x] **Non-overlap:** P1 grades MuJoCo reference accuracy. P2 grades
      algorithm completeness in Rust. P3 grades convention translation.
      P9 grades pipeline integration. Each gap maps to exactly one criterion.
      Boundary: P1 asks "did we understand MuJoCo correctly?"; P2 asks
      "did we specify the implementation completely?"; P9 asks "does it
      fit our architecture?"

- [x] **Completeness:** 9 criteria cover: MuJoCo fidelity (P1), algorithm
      (P2), conventions (P3), ACs (P4), tests (P5), ordering (P6), risk
      (P7), consistency (P8), and architecture (P9). No meaningful gap
      dimension is uncovered.

- [x] **Gradeability:** P1 → MuJoCo Reference section. P2 → Specification
      sections. P3 → Convention Notes. P4 → Acceptance Criteria. P5 → Test
      Plan. P6 → Prerequisites + Execution Order. P7 → Risk & Blast Radius.
      P8 → cross-cutting. P9 → Architecture Decisions + pipeline integration
      in Specification.

- [x] **Conformance primacy:** P1 is tailored with 4 specific MuJoCo C
      functions, 5 deprecation error messages, and 6 edge cases. P4
      requires MuJoCo-verified expected values from EGT-1/EGT-2. P5
      requires at least one MuJoCo conformance test.

- [x] **Empirical grounding:** EGT-1 through EGT-5 filled with verified
      MuJoCo 3.4.0 values and codebase integration points.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors, Convention Notes |
| P2 | Specification (S1, S2, S3, S4) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius |
| P8 | *Cross-cutting — all sections* |
| P9 | Architecture Decisions, S4 (pipeline integration) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | A+ | All 4 MuJoCo functions cited with file:line ranges. Vertex generation formula for 4 shapes with C snippets. Naming convention table (B_first/B_{n}/B_last). 3 initial modes documented. 5 deprecation messages cited verbatim. Edge cases: count validation, geom type, uservert/count exclusion, dimensionality, size validation. Make()-level validations all documented AND enforced in code. |
| P2. Algorithm Completeness | A+ | S3 has complete Rust for vertex gen, frame computation, body chain, geom placement, joint selection, exclude gen. All functions return `Result<>`. `validate_cable()` checks Make()-level (uservert/count exclusion, size, initial value) + MakeCable()-level (dim, count, geom type). Child element parsing fully specified with field-by-field mapping. `build_cable_geom` maps all 16 template fields explicitly. Site generation uses two separate `if` checks (matching MuJoCo), not `if/else if`. No elided code. |
| P3. Convention Awareness | A+ | Convention table present with porting rules for quaternion, body pos, geom fromto, exclude mapping, joint/geom defaults. All rules verified. |
| P4. Acceptance Criteria Rigor | A+ | 18 ACs, all with concrete MJCF input, exact expected values from MuJoCo 3.4.0, fields to check. Covers generation, initial modes, deprecation, naming, excludes, sites, geom placement, nesting, minimum count (with dual-site edge case), invalid geom, uservert cable, uservert/count exclusion, count validation errors, invalid initial. |
| P5. Test Plan Coverage | A+ | AC→Test matrix present. 12 tests for 18 ACs. Edge case inventory with 14 cases (up from 8). T11 covers uservert code path. T12 covers all validation error paths (5 sub-cases). Supplementary tests justified. |
| P6. Dependency Clarity | A+ | Execution order S1→S2→S3→S4 with rationale. Prerequisites stated. Spec D cross-reference explicit. |
| P7. Blast Radius & Risk | A+ | Complete file list. Existing tests unaffected. Non-modification sites listed. Behavioral changes table present. |
| P8. Internal Consistency | A+ | Naming consistent. File paths match. AC numbers match matrix. Exclude formula explicit ("count[0]-2"). Custom text scope unambiguous (out of scope, DT-166). Error variant consistently `composite_error()` throughout. All Make()-level validations documented in MuJoCo Reference ARE enforced in validate_cable(). Frame-composite interaction qualified (supported for composites inside bodies, not direct frame children). |
| P9. Expansion Architecture | A+ | Pipeline stage explicit with rationale. AD-1 with alternatives and qualified frame interaction. Tree manipulation described. Excludes returned to caller. Follows expand_frames() pattern. |

**Overall: A+ (Rev 2) — all 16 gaps closed**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| G1 | P2 | `expand_composites_recursive` deprecation handling shows comments, not error code. Function returns `Vec<MjcfContactExclude>` but needs `Result<Vec<MjcfContactExclude>>` to propagate errors. | Initial grading | Changed function signatures to return `Result<>`. Added `Err(MjcfError::composite_error(...))` for each deprecated type. | Rev 1 |
| G2 | P8 | "N-2 exclude pairs" uses undefined N. Inconsistent: N is count[0] in some places, num_bodies in others. | Initial grading | Changed all occurrences to explicit formulas: "count[0]-2 exclude pairs" or "(num_bodies-1) excludes". | Rev 1 |
| G3 | P2 | `make_cable()` uses `.expect("cable requires <geom> child")` — panics on missing geom instead of returning error. | Initial grading | Changed to return `Err(MjcfError::...)` with descriptive message. Added geom type validation before chain generation. | Rev 1 |
| G4 | P2 | S3 code missing dim/count validation. MuJoCo validates `dim == 1` (count[1]==count[2]==1), count[0]>=2, and geom type. Not shown in S3 Rust code. | Initial grading | Added `validate_cable()` function in S3 that checks dim, count, and geom type before generating. | Rev 1 |
| G5 | P5 | No test for cylinder geom type cable. Only capsule tested. Box and cylinder are valid per C source. | Initial grading | Added T10 for cylinder geom cable. | Rev 1 |
| G6 | P8 | Custom text scope ambiguous — MuJoCo Reference says cable adds `composite_{prefix}` text, but Convention Notes says "Defer (DT-166) or add minimal support". Out of Scope section also mentions DT-166. | Initial grading | Moved custom text explicitly to Out of Scope with rationale: metadata-only, no physics impact, requires `<custom><text>` infrastructure. Removed ambiguous "or add minimal support" from Convention Notes. | Rev 1 |
| G7 | P2 | Site generation uses `if/else if` for first/last — count=2 body is both first AND last but only gets `S_first`. MuJoCo uses two separate `if` checks (`AddCableBody:436-442`), so count=2 body gets BOTH sites. Conformance-breaking bug. | Stress test | Changed to two separate `if is_first` / `if is_last` checks with `let mut sites = Vec::new()`. | Rev 2 |
| G8 | P2 | S2 child element parsing elided: `// ... parse children loop ...`. Implementer cannot determine how to parse `<joint kind="main">` vs `<geom>` children, what attributes to extract, or how to map to `CompositeJoint`/`CompositeGeom`. | Stress test | Replaced with complete parsing code: `<joint kind="main">` with all 7 attributes, `<geom>` with all 16 attributes, `<site>`/`<skin>`/`<plugin>` skip. Uses existing parser helper style. | Rev 2 |
| G9 | P2 | `build_cable_geom()` shows 5 of 16 template fields, then `// ... copy all template fields ...`. Not implementable without reading struct definition. | Stress test | Expanded to explicitly map all 16 `CompositeGeom` fields to `MjcfGeom` fields. No elided code. | Rev 2 |
| G10 | P2 | Missing Make()-level validations: (a) uservert/count mutual exclusion (`Make:147-153`), (b) size validation (`Make:142-144`, `dot(size,size) >= mjMINVAL`), (c) `initial` attribute value validation. MuJoCo Reference documents these but code doesn't enforce. | Stress test | Added all three to `validate_cable()`: uservert/count exclusion + uservert length%3, size squared check, initial value match. Also added effective_count0 computation for uservert case. | Rev 2 |
| G11 | P4 | AC13 (count=2) doesn't verify site behavior. MuJoCo puts both S_first and S_last on the single body. | Stress test | Updated AC13 assert to include `nsite == 2` with explanation of dual-site edge case. | Rev 2 |
| G12 | P4 | No AC for uservert-based cables — distinct code path in `generate_vertices()` never tested. | Stress test | Added AC15 (uservert cable with 3 vertices). | Rev 2 |
| G13 | P4, P5 | No AC or test for validation error cases: count=1, multi-dim count, uservert/count conflict, invalid initial value, zero size. | Stress test | Added AC16 (uservert/count exclusion), AC17 (count errors), AC18 (invalid initial). Added T12 with 5 sub-cases covering all error paths. | Rev 2 |
| G14 | P5 | No test for uservert cable — `generate_vertices()` uservert branch never exercised. | Stress test | Added T11 for uservert cable. | Rev 2 |
| G15 | P8 | MuJoCo Reference documents Make()-level validations (lines 83-90) but `validate_cable()` doesn't enforce them — contradiction between documented MuJoCo behavior and specified implementation. | Stress test | Resolved by G10: all documented validations now enforced in code. | Rev 2 |
| G16 | P8, P9 | AD-1 claims composites inside frames work but doesn't distinguish "composite inside a body inside a frame" (works) from "composite as direct frame child" (unsupported). Untested claim. | Stress test | Qualified AD-1 with explicit frame interaction note. `<composite>` parsing added to `parse_body()`/`parse_worldbody()` only — not frame parser. | Rev 2 |
