# Phase 7 — MJCF Parsing & Defaults Gaps: Umbrella Spec

**Status:** Draft
**Phase:** Roadmap Phase 7
**Tasks:** 11 (DT-2, DT-3, DT-11, DT-13, DT-14, DT-85, DT-88, §55, §60, §64, §64a)
**Deliverables:** 2 T1 items + 3 sub-specs (Spec A, B, C)
**Test baseline:** 1,900+ domain tests (post-Phase 6)

---

## Scope

Phase 7 closes all MJCF parsing gaps, defaults cascade gaps, and joint physics
edge-case gaps in the v1.0 roadmap. **The goal is MuJoCo conformance** — after
Phase 7, CortenForge correctly parses and applies default classes for equality
constraints, joint attributes, and actuator type-specific parameters; computes
joint spring energy for all joint types; wires `springinertia` into the CRBA
diagonal; activates joint limit constraints with the correct margin; parses
per-element user data arrays; and loads hfield elevation from PNG files.

Unlike Phases 5 and 6 (single-domain depth passes on actuators and sensors
respectively), Phase 7 is a **heterogeneous breadth pass** across three
unrelated subsystems: defaults cascade (Spec A), joint physics (Spec B), and
parsing completeness (Spec C). Tasks are grouped by shared design decisions,
not by subsystem. The three sub-specs and the T1 session are fully
independent — no cross-spec dependencies exist.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 7 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Rationale |
|------|-------------|-------------|-----------|
| DT-2 | Equality constraint defaults — `solref`/`solimp` in defaults structs, `apply_to_equality()` cascade | **Spec A** | Defaults cascade — shares design pattern with DT-11/13/14 |
| DT-11 | Joint `range` as defaultable attribute in `MjcfJointDefaults` | **Spec A** | Defaults cascade — joint attribute default resolution |
| DT-13 | `qpos_spring` — distinct from `qpos0`, parse + store + wire default cascade | **Spec A** | Defaults cascade — joint attribute + model field |
| DT-14 | Actuator type-specific defaults — cylinder area/timeconst, muscle params defaultable per actuator type | **Spec A** | Defaults cascade — actuator default class resolution |
| §60 | `springinertia` — joint inertia-spring coupling in CRBA diagonal | **Spec B** | Joint physics — mass matrix modification |
| §64 | Ball/free joint spring potential energy (quaternion geodesic) | **Spec B** | Joint physics — energy computation |
| §64a | `jnt_margin` — joint limit activation margin and constraint row margin | **Spec B** | Joint physics — constraint activation |
| §55 | Per-element `*_user` custom data arrays from MJCF | **Spec C** | Parsing breadth — new model arrays, no runtime effect |
| DT-88 | `<flexcomp>` attributes: `inertiabox`, `scale`, `quat`, `file` | **Spec C** | Parsing breadth — flexcomp attribute completeness |
| DT-3 | File-based hfield loading from PNG | **T1** | Mechanical — parent spec (§6a) defines the "what" |
| DT-85 | Flex `<contact>` runtime attributes: `internal`, `activelayers`, `vertcollide`, `passive` | **T1** | Mechanical — 4 fields, parse + store + wire |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — Defaults Completeness** (DT-2, DT-11, DT-13, DT-14):
Completes the default-class cascade for element types that currently lack full
defaults support. Four distinct gaps:

- **DT-2 (Equality defaults):** No `MjcfEqualityDefaults` struct exists.
  Equality constraints (`connect`, `weld`, `joint`, `tendon`, `distance`) have
  no default-class inheritance for `solref`/`solimp`. Create the struct, parse
  equality defaults from `<default><equality>`, and apply them during builder
  compilation. MuJoCo reference: `mjCEquality` default resolution in
  `user_model.cc` (`mjCDef::Equality()`), and `mjCModel::CopyBack()` for
  equality parameter application.

- **DT-11 (Joint `range` default):** `range` is present on `MjcfJointDefaults`
  already but the session plan says it's not defaultable. Verify current state:
  if `range` is parsed in `parse_joint_defaults()` but not applied via the
  cascade, wire it. MuJoCo reference: `mjCJoint` default resolution in
  `user_model.cc`.

- **DT-13 (`qpos_spring`):** MuJoCo has `springref` (parsed as `springref` in
  MJCF) and a separate `qpos_spring` model array that defaults to `qpos0` but
  can differ when `springref` is explicitly set. Currently CortenForge uses
  `jnt_springref` for the MJCF attribute but has no separate `qpos_spring`
  model array — the spring reference position used in `mj_passive()` reads
  directly from `jnt_springref`. Need to verify whether this is conformant or
  whether a separate `qpos_spring` array (initialized from `qpos0` + overridden
  by `springref`) is needed. MuJoCo reference: `m->qpos_spring` in
  `engine_core_smooth.c` (`mj_passive()`), initialization in
  `engine_setconst.c`.

- **DT-14 (Actuator type-specific defaults):** MuJoCo allows defaults to be
  scoped to specific actuator types — e.g., `<default><muscle
  timeconst="..."/>` sets `timeconst` only for muscle actuators, not for
  position actuators. The current `MjcfActuatorDefaults` struct stores fields
  that apply to all actuator types uniformly. Need to determine whether
  CortenForge already handles type-specific defaults via the existing
  actuator shortcut elements (`<position>`, `<velocity>`, `<motor>`,
  `<muscle>`, `<cylinder>`) or whether additional per-type default structs are
  needed. MuJoCo reference: `mjCActuator` default resolution in
  `user_model.cc`, per-type defaults in `mjXReader::Default()`.

**Spec B — Joint Physics** (§60, §64, §64a):
Three joint physics features that are completely unimplemented:

- **§60 (`springinertia`):** Not parsed, not stored, not enforced. Parse
  `springinertia` from `<joint>` and `<default><joint>`, store on Model, and
  add `springinertia * stiffness[dof]` to the CRBA diagonal (`M[dof,dof]`).
  This couples spring stiffness with apparent inertia for implicit integrator
  stability. MuJoCo reference: `mj_crb()` in `engine_core_smooth.c` — the
  diagonal augmentation happens after the standard composite rigid body
  algorithm completes.

- **§64 (Ball/free spring energy):** Explicit stub in `energy.rs` (lines
  50–53): "Ball/Free joint springs would use quaternion distance — Not commonly
  used, skip for now." The spring potential energy for ball joints uses
  quaternion geodesic distance: `E = 0.5 * k * theta^2` where
  `theta = 2 * arccos(|q_current . q_ref|)`. Free joints combine translational
  spring energy with the ball joint rotational formula. MuJoCo reference:
  `mj_energyPos()` in `engine_core_smooth.c`.

- **§64a (`jnt_margin`):** MuJoCo activates joint limit constraints when
  `dist < jnt_margin[i]`, allowing soft pre-activation. CortenForge hardcodes
  `< 0.0` in 6 locations (3 counting + 3 assembly in `assembly.rs`). The
  `finalize_row!` macro also receives `0.0` as the margin argument instead of
  the joint's margin. Parse `margin` from `<joint>` and `<default><joint>`,
  store as `jnt_margin`, replace all 6 hardcoded checks, and pass margin to
  `finalize_row!`. MuJoCo reference: `mj_instantiateLimit()` in
  `engine_core_constraint.c`.

**Spec C — Parsing Breadth** (§55, DT-88):
Two unrelated parsing completeness tasks grouped by their "parse + store, no
runtime effect" pattern:

- **§55 (Per-element `*_user` data):** Zero references in codebase. Parse
  `user` attribute from `<body>`, `<geom>`, `<joint>`, `<tendon>`,
  `<actuator>`, `<sensor>`, `<site>` elements. Parse `nuser_body`,
  `nuser_geom`, `nuser_jnt`, `nuser_tendon`, `nuser_actuator`, `nuser_sensor`
  from `<size>`. Store as `body_user: Vec<Vec<f64>>` etc. on Model. User data
  is read-only from the physics perspective. MuJoCo reference: element parsing
  in `user_model.cc`, `nuser_*` fields in `mjModel` (`mjmodel.h`).

- **DT-88 (`<flexcomp>` attributes):** Four deferred `<flexcomp>` attributes
  not yet parsed: `inertiabox` (bool), `scale` (Vec3), `quat` (Vec4), `file`
  (string path). Currently `<flexcomp>` is parsed and expanded into `MjcfFlex`
  directly (no separate `MjcfFlexcomp` struct) — the parser at line 2789
  handles generation but these four attributes are skipped. Parse and store
  them on the generated `MjcfFlex` or in intermediate builder state. MuJoCo
  reference: `mjCFlexcomp` in `user_objects.cc`.

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: DT-3, DT-85)         <-- independent
    |
    +-- Sessions 3-7 (Spec A: Defaults)      <-- independent
    |
    +-- Sessions 8-12 (Spec B: Joint)        <-- independent
    |
    +-- Sessions 13-17 (Spec C: Parsing)     <-- independent
```

**Flat dependency graph.** No cross-spec dependencies. All three specs and the
T1 session can be executed in any order. This is fundamentally different from
Phases 5 and 6, which had dependency edges between sub-specs.

### Why no cross-spec dependencies?

| Spec pair | Interaction analysis |
|-----------|---------------------|
| Spec A (defaults) vs Spec B (joint physics) | Spec A adds `margin` to `MjcfJointDefaults` and `qpos_spring` to Model. Spec B adds `springinertia` and `jnt_margin` to Model and reads them at runtime. These touch different code paths: Spec A is builder/defaults, Spec B is runtime/physics. The one overlap — `jnt_margin` parsing — is in Spec B's scope, not Spec A's. `margin` as a defaultable attribute could be in Spec A, but since §64a's primary scope is the runtime constraint activation (Spec B), the full parse-store-wire pipeline stays in Spec B. |
| Spec A (defaults) vs Spec C (parsing) | No shared data. Spec A modifies defaults structs and application. Spec C adds new model arrays (`*_user`, flexcomp attrs). Different parser sections, different model fields. |
| Spec B (joint physics) vs Spec C (parsing) | No shared data. Spec B modifies joint model fields and runtime physics. Spec C adds user data and flexcomp model fields. |
| T1 vs any spec | T1 adds hfield PNG loading and flex contact attributes. No overlap with defaults, joint physics, or user data. |

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/mjcf/src/parser.rs`

| Order | Deliverable | Section | Change | State after |
|-------|-------------|---------|--------|-------------|
| any | T1 | hfield (~line 1266) | Add `file` attribute parsing to `parse_hfield_attrs()` | hfield loadable from file or inline |
| any | T1 | flex contact (~line 2705) | Add `internal`, `activelayers`, `vertcollide`, `passive` to `parse_flex_contact_attrs()` | 4 new flex contact attrs parsed |
| any | Spec A | defaults (~line 589) | Add `parse_equality_defaults()`, add `margin` to `parse_joint_defaults()` | Equality defaults parsed; joint margin defaultable |
| any | Spec B | joint attrs (~line 1555) | Add `springinertia`, `margin` to `parse_joint_attrs()` | 2 new joint attrs parsed |
| any | Spec C | many sections | Add `user` attr parsing to body/geom/joint/tendon/actuator/sensor/site parsers; add flexcomp attrs | user attrs parsed on 7 element types |

**Conflict risk: Low.** Each deliverable modifies a different section of the
parser. The parser is ~4000+ lines and sections are spatially separated.

### `sim/L0/mjcf/src/types.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | T1 | Add `internal`, `activelayers`, `vertcollide`, `passive` to `MjcfFlex` contact fields | 4 new flex contact fields |
| any | Spec A | Create `MjcfEqualityDefaults` struct; add `equality` field to `MjcfDefault`; add `margin` to `MjcfJointDefaults` | New defaults struct + 2 field additions |
| any | Spec C | Add `user: Option<Vec<f64>>` to element structs (body/geom/joint/tendon/actuator/sensor/site); add flexcomp attrs | user fields on 7+ structs |

**Conflict risk: Low.** Different structs, different fields.

### `sim/L0/core/src/types/model.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | T1 | Add `flex_internal`, `flex_activelayers`, `flex_vertcollide`, `flex_passive` arrays | 4 new flex arrays |
| any | Spec A | Add `qpos_spring: Vec<f64>` (if needed after MuJoCo C source analysis) | 1 potential new field |
| any | Spec B | Add `jnt_springinertia: Vec<f64>`, `jnt_margin: Vec<f64>` | 2 new joint fields |
| any | Spec C | Add `body_user`, `geom_user`, `jnt_user`, `tendon_user`, `actuator_user`, `sensor_user`, `site_user` arrays; add `nuser_*` fields; add flexcomp attrs | 7+ user arrays + nuser fields |

**Conflict risk: Low.** All additions are new fields — no modification of
existing fields.

### `sim/L0/core/src/types/model_init.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | T1 | Init flex contact arrays | 4 new defaults |
| any | Spec A | Init `qpos_spring` (if added) | 1 potential default |
| any | Spec B | Init `jnt_springinertia`, `jnt_margin` | 2 new defaults |
| any | Spec C | Init `*_user` arrays and `nuser_*` fields | 7+ new defaults |

**Conflict risk: None.** Append-only pattern, same as `model.rs`.

### `sim/L0/mjcf/src/builder/` (joint building)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| any | Spec A | Wire joint defaults cascade for `range` (if not already wired) and `margin` | Defaults applied |
| any | Spec B | Wire `springinertia` and `margin` through builder to model fields | New fields populated |

**Conflict risk: Low.** Spec A wires defaults application (the cascade). Spec B
wires the new model fields through the builder. Both touch the joint builder
but add code in different sections — Spec A in the defaults-application path,
Spec B in the field-population path.

**Note on `margin`:** Both Spec A and Spec B have reason to touch `margin`.
Spec A might add it to `MjcfJointDefaults` for the cascade, while Spec B
parses it in `parse_joint_attrs()` and stores it as `jnt_margin`. Resolution:
**Spec B owns `margin` end-to-end** — parsing, defaults, model storage, and
runtime wiring. This keeps the full §64a pipeline in one spec. Spec A's
DT-11 scope covers `range` as a defaultable attribute, not `margin`.

---

## API Contracts

No cross-spec API dependencies exist in Phase 7. Each spec produces
independent additions to the codebase:

- **Spec A** adds defaults structs and cascade logic — no other spec calls
  these functions.
- **Spec B** adds model fields and runtime physics — no other spec reads
  `jnt_margin` or `jnt_springinertia`.
- **Spec C** adds model arrays — no other spec reads `*_user` data.
- **T1** adds model arrays and parsing — no other spec reads flex contact
  attributes or hfield file data.

This is the key structural difference from Phases 5 and 6, which had API
contracts between sub-specs (e.g., Spec A's objtype parsing consumed by
Spec B's reftype resolution). Phase 7's flat dependency graph means each
sub-spec is self-contained.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. New model array naming

All new model arrays follow the existing `{element}_{name}` pattern:

| Field | Type | Default | Deliverable |
|-------|------|---------|-------------|
| `qpos_spring` | `Vec<f64>` | Copy of `qpos0` | Spec A (if needed) |
| `jnt_springinertia` | `Vec<f64>` | `0.0` | Spec B |
| `jnt_margin` | `Vec<f64>` | `0.0` | Spec B |
| `body_user` | `Vec<Vec<f64>>` | `vec![]` (empty per-body) | Spec C |
| `geom_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `jnt_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `tendon_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `actuator_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `sensor_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `site_user` | `Vec<Vec<f64>>` | `vec![]` | Spec C |
| `nuser_body` | `i32` | `-1` | Spec C |
| `nuser_geom` | `i32` | `-1` | Spec C |
| `nuser_jnt` | `i32` | `-1` | Spec C |
| `nuser_tendon` | `i32` | `-1` | Spec C |
| `nuser_actuator` | `i32` | `-1` | Spec C |
| `nuser_sensor` | `i32` | `-1` | Spec C |
| `flex_internal` | `Vec<bool>` | `true` | T1 |
| `flex_activelayers` | `Vec<i32>` | `0` | T1 |
| `flex_vertcollide` | `Vec<bool>` | `false` | T1 |
| `flex_passive` | `Vec<bool>` | `true` | T1 |

**`nuser_*` default convention:** MuJoCo uses `-1` as the default for
`nuser_*` fields (meaning "use the length of the `user` attribute"). When
`nuser_* >= 0`, the user array is padded with zeros or truncated to that
length. Sub-spec C must match this behavior exactly.

### 2. MJCF attribute naming

Match MuJoCo attribute names exactly:
- `springinertia` (not `spring_inertia`)
- `margin` (not `jnt_margin` in MJCF — stored as `jnt_margin` in Rust model)
- `user` (space-separated float list)
- `nuser_body`, `nuser_geom`, etc. (on `<size>` element)
- `inertiabox`, `scale`, `quat`, `file` (on `<flexcomp>`)
- `internal`, `activelayers`, `vertcollide`, `passive` (on `<flex><contact>`)

Rust model field names use the `{element}_{attr}` snake_case pattern.

### 3. Default values for new fields

- Numeric fields default to `0.0` (not `f64::NAN` or sentinels), unless MuJoCo
  uses a different default (e.g., `nuser_*` defaults to `-1`).
- Boolean fields default to their MuJoCo default (e.g., `internal` defaults to
  `true`, `passive` defaults to `true`, `vertcollide` defaults to `false`).
- `Option<T>` fields default to `None`.
- New arrays initialized in `model_init.rs` as empty `vec![]`.

### 4. Equality defaults struct pattern

Spec A creates `MjcfEqualityDefaults` following the pattern of existing
defaults structs (`MjcfJointDefaults`, `MjcfActuatorDefaults`, etc.):

```rust
#[derive(Debug, Clone, Default)]
pub struct MjcfEqualityDefaults {
    pub solref: Option<[f64; 2]>,
    pub solimp: Option<[f64; 5]>,
    // Additional fields as needed per MuJoCo C source analysis
}
```

The struct is added to `MjcfDefault` alongside existing defaults structs.
Application follows the existing `apply_defaults()` pattern in the builder.

### 5. User data storage pattern

User data arrays use `Vec<Vec<f64>>` (not flat `Vec<f64>`) so each element's
user data can have variable length. When `nuser_*` is set, all arrays are
padded/truncated to that fixed length. When `nuser_*` is `-1`, each element
stores exactly the floats specified in its `user` attribute (or empty if no
`user` attribute).

This matches MuJoCo's flat storage (`nuser_* * count` floats) conceptually
but uses Rust's `Vec<Vec<f64>>` for clarity. A future optimization (DT-117
adjacent) could flatten to `Vec<f64>` with stride.

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| Spec A defaults + Spec B `jnt_margin` | **No conflict.** Spec A handles `range` as a defaultable joint attribute. Spec B handles `margin` end-to-end (parse, default, store, runtime). Different attributes, different code paths. |
| Spec A `qpos_spring` + Spec B spring energy | **Potential alignment.** Spec A may add `qpos_spring` to Model. Spec B's §64 uses the spring reference quaternion for ball/free energy. If `qpos_spring` replaces `jnt_springref` as the source of truth for spring reference, Spec B's energy computation should read from `qpos_spring`. **Resolution:** Spec A's DT-13 defines the `qpos_spring` semantics. Spec B should read from whatever field the spring reference is stored in after Spec A lands, or from `jnt_springref` if Spec A hasn't landed yet. Since the specs are independent, each documents its source field and the review phase catches any mismatch. |
| Spec B `springinertia` + Spec A joint defaults | **No conflict.** `springinertia` is not a defaultable attribute in the current codebase. If MuJoCo supports `springinertia` in `<default><joint>`, Spec B handles it. |
| Spec C `*_user` + anything | **No conflict.** User data has no runtime physics effect. No other spec reads user data. |
| T1 flex contact + Spec C flexcomp | **No conflict.** T1 modifies `<flex><contact>` attributes. Spec C modifies `<flexcomp>` element-level attributes. Different parser sections, different model fields. |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| Joint parsing tests | Spec A (defaults), Spec B (new attrs) | **Low.** Different attributes being tested. |
| Constraint assembly tests | Spec B only | **None.** Single-owner. |
| Energy computation tests | Spec B only | **None.** Single-owner. |
| CRBA tests | Spec B only | **None.** Single-owner. |
| Flex parsing tests | T1 (contact attrs), Spec C (flexcomp attrs) | **Low.** Different parser sections. |
| Phase 6 regression suite (1,900+ tests) | None | **None.** Phase 7 does not modify any sensor code paths. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1-a (DT-3: hfield PNG) | 2–4 (file load, grayscale conversion, missing file error) | +2–4 |
| T1-b (DT-85: flex contact attrs) | 2–3 (parse + round-trip, builder wire) | +2–3 |
| Spec A | 8–12 (equality defaults cascade, joint range default, qpos_spring, actuator type-specific defaults) | +8–12 |
| Spec B | 8–12 (springinertia CRBA, ball/free spring energy, jnt_margin activation at 3 joint types) | +8–12 |
| Spec C | 6–10 (user data parse + pad/truncate per element type, flexcomp attrs) | +6–10 |
| **Total** | **26–41** | **+26–41** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 7 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's parsing, defaults, and joint physics behavior is
numerically identical to MuJoCo's for every feature implemented in Phase 7.**

### PH7-AC1: All 11 tasks ship-complete
Every task in the assignment table has landed and its sub-spec ACs (or T1
verification criteria) are met. Every sub-spec AC that asserts a numerical
value has that value verified against MuJoCo's actual output.

### PH7-AC2: No regression in existing test suite
All 1,900+ domain tests from the post-Phase 6 baseline pass. Zero test
failures attributable to Phase 7 changes.

### PH7-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH7-AC4: Aggregate test growth
Domain test count increases by at least 26 (lower bound of per-spec estimates).
At least one MuJoCo conformance test (expected value from running MuJoCo) per
sub-spec.

### PH7-AC5: Feature coverage
After Phase 7, the following MuJoCo features are covered:

| Feature | Pre-Phase 7 | Post-Phase 7 |
|---------|------------|-------------|
| Equality defaults | No default-class inheritance | `solref`/`solimp` via `<default><equality>` |
| Joint `range` default | Parsed in defaults struct, verify cascade | Confirmed defaultable via cascade |
| `qpos_spring` | Uses `jnt_springref` directly | Separate `qpos_spring` array (if MuJoCo requires it) |
| Actuator type defaults | Uniform defaults for all types | Type-specific defaults matching MuJoCo |
| `springinertia` | Not parsed/stored/enforced | Parsed + CRBA diagonal: `M[dof,dof] += springinertia * stiffness` |
| Ball/free spring energy | Stub (returns 0) | `0.5 * k * theta^2` via quaternion geodesic |
| `jnt_margin` | Hardcoded `< 0.0` (6 sites) | `< jnt_margin[i]` + margin in `finalize_row!` |
| Per-element `*_user` data | Not parsed/stored | 7 element types + `nuser_*` sizing |
| `<flexcomp>` attributes | 4 attrs skipped | `inertiabox`, `scale`, `quat`, `file` parsed |
| Hfield from file | Inline `elevation` only | PNG file loading via `image` crate |
| Flex contact runtime attrs | Not parsed | `internal`, `activelayers`, `vertcollide`, `passive` |

---

## Out of Scope

Explicitly excluded from Phase 7. Each exclusion states its conformance impact.

- **DT-1** (Mesh defaults `apply_to_mesh()`) — Low priority; tracked in
  Post-v1.0 Low-Priority MuJoCo Compat. *Conformance impact: minimal — mesh
  defaults are rarely used in standard models. Root-only mesh scale defaults
  are a convenience, not a correctness gap.*

- **DT-5** (`gaintype/biastype/dyntype="user"` callback types) — Requires
  plugin system (§66). *Conformance impact: minimal — user callback types
  are not used in standard MuJoCo models.*

- **DT-10** (Deferred `<compiler>` attributes: `fitaabb`, `usethread`, etc.)
  — Low priority; no physics effect. *Conformance impact: none for numerical
  conformance — these are build-time optimizations.*

- **DT-12** (Programmatic enforcement that `worldbody.childclass = None`)
  — Validation-only; no physics effect. *Conformance impact: none.*

- **DT-15** (Sentinel-value detection to `Option<T>` migration) — Code
  quality improvement, not conformance. *Conformance impact: none.*

- **DT-17** (Global `<option o_margin>` override) — Per-geom margin is the
  correct foundation; global override is a convenience. *Conformance impact:
  minor — affects models that use `o_margin` in `<option>`, which is uncommon.*

- **DT-87** (Shared-body flex vertices) — Complex flex feature; tracked in
  Post-v1.0 Flexcomp Completeness. *Conformance impact: affects models with
  multi-vertex-per-body flex topology, which is uncommon.*

- **DT-86** (`elastic2d` keyword) — Flex elasticity model selection; tracked
  in Post-v1.0. *Conformance impact: minor — affects 2D flex models only.*

- **Flex `<contact>` runtime wiring beyond parse+store** — DT-85 (T1) parses
  and stores the 4 flex contact attributes. Runtime behavior changes (e.g.,
  `internal` affecting which flex contacts are generated) are part of the
  broader flex pipeline (Phase 10). *Conformance impact: the parsed values are
  available; runtime gating is deferred.*

- **`<flexcomp>` mesh file loading** — DT-88 parses the `file` attribute and
  stores the path. Actual mesh/grid file loading for flexcomp is a separate
  task. *Conformance impact: the attribute is parsed; loading deferred.*

- **`qpos_spring` runtime changes** — If DT-13 discovers that `qpos_spring`
  requires changes to `mj_passive()` or `mj_energyPos()`, those runtime changes
  are in DT-13's scope (Spec A). However, the Spring Reference quaternion
  computation for ball/free joints (§64 energy) uses the same reference data —
  Spec B handles that independently using `jnt_springref`.
