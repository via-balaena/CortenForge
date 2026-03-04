# Spec A — Solver Param & Margin Completeness

**Status:** Draft
**Phase:** Roadmap Phase 8 — Constraint & Solver Gaps
**Effort:** S (DT-23 verification + tests) + M (DT-33 field wiring + assembly fixes)
**MuJoCo ref:** `mj_instantiateLimit()` in `engine_core_constraint.c`, lines ~1197–1229;
`mj_instantiateFriction()` in `engine_core_constraint.c`, lines ~1240–1280;
`getsolparam()` in `engine_core_constraint.c`, lines ~1430–1440
**MuJoCo version:** 3.2.x (C source read for rubric EGT-1 through EGT-8)
**Test baseline:** 1,900+ sim domain tests (post-Phase 7)
**Prerequisites:**
- Phase 7 Spec B `jnt_margin` pattern (landed — direct reference for DT-33)
- Phase 8 umbrella (commit `4574a03`)
- DT-32 naming conformance (commit `9f9cf9f`)
- DT-28 + DT-39 T1 items (commit `22c7c3d`)

**Independence:** This spec is independent of Spec B (QCQP Cone Projection) and
DT-25 (verification pass) per the umbrella dependency graph. Shared file:
`assembly.rs` — Spec A modifies tendon limit sections (lines 142–157, 545–607);
Spec B modifies solver projection (solver/*.rs); no overlap.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Spec A is part of the Phase 8 umbrella. Empirical verification confirms the
umbrella's scope is accurate with minor refinements.

| Umbrella claim | MuJoCo reality | Action |
|----------------|----------------|--------|
| DT-23: "fields may partially exist" | Full pipeline exists: `dof_solref`/`dof_solimp` in Model (model.rs:233,236), parsing (parser.rs:669–678), defaults cascade (defaults.rs:209–213), builder fan-out (builder/joint.rs:160–163 inside `for i in 0..nv`), assembly wiring (assembly.rs:391–392). | **In scope** as verification + test coverage only. No code changes expected. |
| DT-23: "verify multi-DOF fan-out" | Builder loops `for i in 0..nv` and pushes `dof_solref` per-DOF (joint.rs:160–163). MuJoCo does same in CopyTree: `for (int j1=0; j1<pj->nv(); j1++) { mjuu_copyvec(m->dof_solref+mjNREF*dofadr, ...) }`. | **In scope**: add tests confirming ball (3 DOFs) and free (6 DOFs) all get parent joint's `solreffriction`. |
| DT-23: "verify tendon friction solref end-to-end" | `tendon_solref_fri`/`tendon_solimp_fri` in Model (model.rs:700–702), parsed (parser.rs:3649–3658), defaults cascade (defaults.rs:638–642), builder (tendon.rs:45–48), assembly (assembly.rs:418–419). Pipeline complete. | **In scope** as verification. Add test with non-default tendon `solreffriction`. |
| DT-33: "parsing + defaults exist, model field missing" | Confirmed: `MjcfTendon.margin` parsed (parser.rs:3646), `MjcfTendonDefaults.margin` cascaded (defaults.rs:644–645). `tendon_margin` does NOT exist in Model. 4 hardcoded `< 0.0` at assembly.rs:150,154,562,586. | **In scope**: add Model field, wire builder, fix 6 sites in assembly.rs. |
| DT-33: "4 hardcoded sites" | Actually 4 hardcoded `< 0.0` checks (150, 154, 562, 586) PLUS 2 hardcoded `0.0` margin args to `finalize_row!` (573, 597). Total: 6 sites. | **In scope**: umbrella undercounted by 2 `finalize_row!` margin args. |

**Final scope:**
1. DT-23: Verify per-DOF friction solver param pipeline end-to-end (parsing →
   defaults → builder → model → assembly). Add tests for multi-DOF fan-out
   (ball, free) and non-default values. Verify tendon friction solref
   end-to-end.
2. DT-33: Add `tendon_margin: Vec<f64>` to Model. Wire in builder. Replace
   4 hardcoded `< 0.0` with `< model.tendon_margin[t]` and 2 hardcoded
   `0.0` finalize_row margin args with `model.tendon_margin[t]`.

---

## Problem Statement

**DT-33 (Conformance gap — tendon margin):** MuJoCo activates tendon limit
constraints when `dist < margin` (`mj_instantiateLimit()` in
`engine_core_constraint.c`), where `margin = m->tendon_margin[i]` is a
per-tendon compile-time field defaulting to `0.0`. CortenForge parses the
`margin` attribute from `<tendon>` elements and cascades it through defaults,
but the pipeline is broken: no `tendon_margin` field exists in Model, no
builder wiring populates it, and 6 sites in assembly.rs use hardcoded `0.0`
instead of the parsed margin value. When `margin > 0`, CortenForge fails to
pre-activate tendon limit constraints, producing different constraint counts
and impedance behavior than MuJoCo.

**DT-23 (Verification gap — DOF friction solver params):** MuJoCo copies
`solreffriction`/`solimpfriction` to every DOF of a joint (including all 3
DOFs of a ball joint and all 6 DOFs of a free joint) via CopyTree in
`user_model.cc`. The CortenForge pipeline appears complete — parsing, defaults
cascade, builder fan-out, model fields, and assembly wiring all exist — but
no tests verify the multi-DOF fan-out or exercise non-default
`solreffriction` values through the constraint solver. The tendon friction
solver params (`tendon_solref_fri`/`tendon_solimp_fri`) are also fully wired
but untested with non-default values.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here. If this section is wrong, everything is wrong.

### Tendon limit activation with margin

**Source:** `mj_instantiateLimit()` in `engine_core_constraint.c`, lines ~1197–1229.

MuJoCo activates tendon limit constraints using a per-tendon margin:

```c
// Tendon limits (inside mj_instantiateLimit)
for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_limited[i]) {
      value = d->ten_length[i];
      margin = m->tendon_margin[i];

      for (int side=-1; side <= 1; side+=2) {
        dist = side * (m->tendon_range[2*i+(side+1)/2] - value);

        if (dist < margin) {
          // Create constraint row:
          // - pos = dist (negative = limit violated, positive = within margin zone)
          // - margin passed to mj_addConstraint() → stored in efc_margin
          // - Jacobian = -side * ten_J (pushes length toward limit)
          // - vel = -side * ten_velocity
          mj_addConstraint(m, d, &jac, dist, margin, ...);
        }
      }
    }
}
```

**Side convention:**
- `side = -1` → lower limit: `dist = -(limit_min - length) = length - limit_min`
  - When `dist < 0`: length is below minimum (violated)
  - When `0 < dist < margin`: length is above minimum but within margin (pre-activated)
  - Jacobian: `+ten_J` (pushes length up toward limit)
- `side = +1` → upper limit: `dist = +(limit_max - length) = limit_max - length`
  - When `dist < 0`: length exceeds maximum (violated)
  - When `0 < dist < margin`: length is below maximum but within margin (pre-activated)
  - Jacobian: `-ten_J` (pushes length down toward limit)

**Counting phase:** `mj_nl()` in `engine_core_constraint.c` uses the identical
`dist < margin` check:

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

**Critical invariant:** Counting phase and instantiation phase MUST use
identical activation conditions. A mismatch causes buffer overrun or
under-allocation.

**Default value:** `tendon_margin[i]` defaults to `0.0` (`mjmodel.h` field
default). When `margin = 0.0`, `dist < 0.0` is the activation condition —
identical to the pre-change hardcoded behavior.

**Negative margin:** Valid in MuJoCo. When `margin < 0`, the activation zone
shrinks: the constraint only fires when the limit is violated by more than
`|margin|`. This is useful for "dead zone" around limits.

**Downstream impedance:** `mj_addConstraint()` stores `dist` as `efc_pos` and
`margin` as `efc_margin`. The impedance function `getimpedance()` uses
`x = (pos - margin) / width` (matching CortenForge's `compute_impedance()`
formula: `violation = (pos - margin).abs()`, `x = violation / width`). The
reference acceleration `aref` uses `K * imp * (pos - margin)`. Wiring the
correct margin value through `finalize_row!` produces correct impedance
ramp-up and reference acceleration with no changes needed in impedance.rs.

### DOF friction loss solver parameters

**Source:** `getsolparam()` in `engine_core_constraint.c`, lines ~1430–1440.

```c
case mjCNSTR_FRICTION_DOF:
    mju_copy(solref, m->dof_solref + mjNREF*id, mjNREF);   // direct read
    mju_copy(solimp, m->dof_solimp + mjNIMP*id, mjNIMP);   // no fallback
    break;
```

No runtime fallback chain — all defaults are resolved at compile time. The
`dof_solref`/`dof_solimp` arrays are read directly by DOF index.

**Friction constraint instantiation:** `mj_instantiateFriction()` in
`engine_core_constraint.c` handles both DOF friction and tendon friction in
two consecutive loops. Both loops pass `pos=0, margin=0` (friction loss has
no positional activation — it is always active when `frictionloss > 0`):

```c
// DOF friction loop (mj_instantiateFriction):
for (int i=0; i < m->nv; i++) {
    if (m->dof_frictionloss[i] > 0) {
        // pos=0, margin=0, frictionloss=m->dof_frictionloss[i]
        mj_addConstraint(m, d, ..., 0, 0, m->dof_frictionloss[i], ...);
    }
}
// Tendon friction loop (mj_instantiateFriction):
for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_frictionloss[i] > 0) {
        // pos=0, margin=0, frictionloss=m->tendon_frictionloss[i]
        mj_addConstraint(m, d, ..., 0, 0, m->tendon_frictionloss[i], ...);
    }
}
```

### Tendon friction loss solver parameters

**Source:** `getsolparam()` case `mjCNSTR_FRICTION_TENDON`:

```c
case mjCNSTR_FRICTION_TENDON:
    mju_copy(solref, m->tendon_solref_friction + mjNREF*id, mjNREF);
    mju_copy(solimp, m->tendon_solimp_friction + mjNIMP*id, mjNIMP);
    break;
```

Same direct-read pattern as DOF friction. Per-tendon solver params resolved
at compile time.

### Tendon limit solver parameters

**Source:** `getsolparam()` case `mjCNSTR_LIMIT_TENDON`:

```c
case mjCNSTR_LIMIT_TENDON:
    mju_copy(solref, m->tendon_solref_limit + mjNREF*id, mjNREF);
    mju_copy(solimp, m->tendon_solimp_limit + mjNIMP*id, mjNIMP);
    break;
```

Tendon limit solver params are read per-tendon directly from model fields.
CortenForge already wires these correctly at assembly.rs:553–554
(`model.tendon_solref_lim[t]`, `model.tendon_solimp_lim[t]`). No changes
needed — verified in DT-32 (commit `9f9cf9f`).

### Compile-time default chain

**Source:** CopyTree in `user_model.cc`, `mj_defaultSolRefImp()` in
`engine_setconst.c`.

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

**Edge cases:**
- `margin = 0.0` → `dist < 0.0` (degenerates to limit-violated-only check)
- `margin < 0.0` → valid; shrinks activation zone
- Free joint → 6 DOFs, all receive same `solreffriction` values
- World body tendon attachment → tendon Jacobian has zeros for world DOFs
- `tendon_limited = false` → no limit rows regardless of margin
- `DISABLE_LIMIT` flag → no limit rows regardless of margin
- `frictionloss = 0.0` → no friction row regardless of `solreffriction`

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Tendon limit activation check | `dist < margin` where `margin = m->tendon_margin[i]` (`mj_instantiateLimit()`) | Hardcoded `dist < 0.0` at 4 sites in assembly.rs:150,154,562,586 |
| Tendon limit `finalize_row!` margin arg | `margin` passed to `mj_addConstraint()` → `efc_margin[row]` | Hardcoded `0.0` at assembly.rs:573,597 |
| `tendon_margin` model field | `tendon_margin[ntendon]` in `mjModel` (`mjmodel.h`), default `0.0` | **Missing** — field does not exist in Model |
| Counting / instantiation agreement | Identical `dist < margin` in both `mj_nl()` and `mj_instantiateLimit()` | Both use `< 0.0` (consistent but wrong) |
| DOF friction solref direct read | `m->dof_solref[mjNREF*id]` in `getsolparam()` | `model.dof_solref[dof_idx]` at assembly.rs:391 — **conformant** |
| Multi-DOF solreffriction fan-out | CopyTree loops `for j1 in 0..nv`, copies identical values | Builder loops `for i in 0..nv`, pushes identical values — **conformant** (untested) |
| Tendon friction solref read | `m->tendon_solref_friction[mjNREF*id]` in `getsolparam()` | `model.tendon_solref_fri[t]` at assembly.rs:418 — **conformant** (untested) |
| Default solref/solimp values | `[0.02, 1.0]` / `[0.9, 0.95, 0.001, 0.5, 2.0]` via `mj_defaultSolRefImp()` | `DEFAULT_SOLREF` / `DEFAULT_SOLIMP` constants — **conformant** |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `tendon_margin` | `tendon_margin[ntendon]` flat `mjtNum` array, indexed by tendon id | `tendon_margin: Vec<f64>`, indexed by tendon id | Direct port — identical semantics |
| `tendon_range` | `tendon_range[2*i+...]` flat array, side indexing via `(side+1)/2` | `tendon_range[t]: (f64, f64)` tuple, `.0` = min, `.1` = max | Use `model.tendon_range[t].0` for `tendon_range[2*i]` (lower), `.1` for `tendon_range[2*i+1]` (upper) |
| `dof_solref` | `dof_solref[mjNREF*id]` flat array (2 floats per DOF) | `dof_solref[dof_idx]: [f64; 2]`, indexed by DOF id | Direct port — `model.dof_solref[dof_idx]` for `m->dof_solref + mjNREF*id` |
| CopyTree fan-out | `for j1 in 0..nv` copies identical solref to each DOF | `for i in 0..nv` pushes identical solref to each DOF (joint.rs:160–163) | Direct port — no translation needed |
| Side convention | `side = -1` → lower, `side = +1` → upper; `dist = side * (range - value)` | Lower: `dist = length - limit_min`; Upper: `dist = limit_max - length` | Equivalent: CortenForge computes dist directly instead of using side multiplier. Both produce identical dist values. |
| Jacobian sign | `jac = -side * ten_J` | Lower: `+ten_J` (side=-1, -(-1)=+1); Upper: `-ten_J` (side=+1, -(+1)=-1) | Direct port — CortenForge's explicit signs match MuJoCo's `-side` formula |

### Cross-Subsystem Parity

DT-33 (tendon margin) replicates the `jnt_margin` pattern from Phase 7 Spec B.

| Aspect | `jnt_margin` (existing) | `tendon_margin` (this spec) | Parity |
|--------|------------------------|-----------------------------|--------|
| Model field | `jnt_margin: Vec<f64>` (model.rs:215) | `tendon_margin: Vec<f64>` | Exact match — same type, same default `0.0` |
| Model init | `jnt_margin: vec![]` (model_init.rs:103) | `tendon_margin: vec![]` | Exact match |
| Builder struct | `jnt_margin: Vec<f64>` (builder/mod.rs) | `tendon_margin: Vec<f64>` | Exact match |
| Builder init | `jnt_margin: vec![]` (builder/init.rs:60) | `tendon_margin: vec![]` | Exact match |
| Builder push | `self.jnt_margin.push(joint.margin.unwrap_or(0.0))` (joint.rs:140) | `self.tendon_margin.push(tendon.margin.unwrap_or(0.0))` | Exact match |
| Build transfer | `jnt_margin: self.jnt_margin` (build.rs) | `tendon_margin: self.tendon_margin` | Exact match |
| Counting phase | `if dist < margin` at assembly.rs:111 (lower), 115 (upper), 131 (ball) | `if dist < margin` at assembly.rs:150 (lower), 154 (upper) | Exact match — `length` replaces `q`; tendon has no ball joint dispatch |
| Instantiation phase | `if dist < margin` at assembly.rs:447 (hinge lower), 465 (hinge upper), 492 (ball) | `if dist < margin` at assembly.rs:562 (lower), 586 (upper) | Exact match — tendon has no ball joint dispatch |
| finalize_row margin | `margin` at assembly.rs:456 (hinge lower), 478 (hinge upper), 510 (ball) | `margin` at assembly.rs:573 (lower), 597 (upper) | Exact match |
| Limit types | Hinge lower/upper + Ball (3 dispatches) | Lower/Upper (2 dispatches, no joint type dispatch) | **Structural difference** — tendon limits have no joint type dispatch (single distance formula for all tendon types) |
| Test structure | T8–T11 (assembly.rs:904–1057) | T1–T6 (new, mirroring T8–T11 structure) | Pattern match |

---

## Pipeline Lifecycle

### DT-33: `tendon_margin` pipeline (parse → model → assembly)

| # | Stage | File:line | Current state | Action | Expected value at stage |
|---|-------|-----------|---------------|--------|------------------------|
| 1 | Parse | parser.rs:3646 | **Exists** ✓ | Verify | `MjcfTendon.margin: Option<f64>` populated from `<tendon margin="...">` |
| 2 | Defaults cascade | defaults.rs:644–645 | **Exists** ✓ | Verify | `apply_to_tendon()` cascades `margin` from default class |
| 3 | ModelBuilder struct | builder/mod.rs:~625 | **Missing** | Implement | `pub(crate) tendon_margin: Vec<f64>` declared on struct |
| 4 | ModelBuilder init | builder/init.rs:~193 | **Missing** | Implement | `tendon_margin: vec![]` in `ModelBuilder::new()` |
| 5 | Builder push | builder/tendon.rs:~62 | **Missing** | Implement | `self.tendon_margin.push(tendon.margin.unwrap_or(0.0))` |
| 6 | Build transfer | builder/build.rs:~323 | **Missing** | Implement | `tendon_margin: self.tendon_margin` in Model construction |
| 7 | Model field | model.rs:~696 | **Missing** | Implement | `pub tendon_margin: Vec<f64>` on Model struct |
| 8 | Model init | model_init.rs:~288 | **Missing** | Implement | `tendon_margin: vec![]` in `Model::empty()` |
| 9a | Assembly counting (lower) | assembly.rs:150 | **Wrong** (`< 0.0`) | Fix | `< margin` where `margin = model.tendon_margin[t]` |
| 9b | Assembly counting (upper) | assembly.rs:154 | **Wrong** (`< 0.0`) | Fix | `< margin` |
| 10a | Assembly instantiation (lower check) | assembly.rs:562 | **Wrong** (`< 0.0`) | Fix | `< margin` |
| 10b | Assembly instantiation (lower margin arg) | assembly.rs:573 | **Wrong** (`0.0`) | Fix | `margin` passed to `finalize_row!` |
| 10c | Assembly instantiation (upper check) | assembly.rs:586 | **Wrong** (`< 0.0`) | Fix | `< margin` |
| 10d | Assembly instantiation (upper margin arg) | assembly.rs:597 | **Wrong** (`0.0`) | Fix | `margin` passed to `finalize_row!` |

**Forgetting any one of stages 3–8 creates a silent bug or compile failure.**
Rust's exhaustive struct initialization catches missing fields in build.rs
and model_init.rs at compile time. Missing builder/mod.rs or builder/init.rs
fields are also caught by compilation. Missing builder/tendon.rs push would
compile but silently produce an undersized `tendon_margin` vec, causing a
panic at assembly time.

Pattern reference: `jnt_margin` pipeline (Phase 7 Spec B §64a) touches the
same 6 implementation locations (stages 3–8).

### DT-23: Per-DOF friction solver param pipeline (verify-only)

| File:line | Current state | Action | Expected value |
|-----------|---------------|--------|----------------|
| parser.rs:669–678 | Exists ✓ | Verify (T9) | `solreffriction` parsed from `<joint>` |
| defaults.rs:209–213 | Exists ✓ | Verify (T9) | Cascaded from `<default><joint>` |
| joint.rs:160–163 | Exists ✓ | Verify (T7, T8) | Per-DOF fan-out: ball=3, free=6 identical values |
| assembly.rs:391–392 | Exists ✓ | Verify (T10) | `dof_solref[dof_idx]` read for DOF friction row |
| assembly.rs:418–419 | Exists ✓ | Verify (T11) | `tendon_solref_fri[t]` read for tendon friction row |

---

## Specification

### S1. Add `tendon_margin` model field (DT-33)

**File:** `core/src/types/model.rs` (after `tendon_solimp_lim` at line ~695)
**MuJoCo equivalent:** `tendon_margin[ntendon]` in `mjmodel.h`
**Design decision:** Place adjacent to `tendon_solimp_lim` (the last tendon
limit-related field), matching the `jnt_margin` placement pattern (which is
adjacent to `jnt_solimp`). Type `Vec<f64>` with default `0.0` — identical to
`jnt_margin`.

**After:**
```rust
    /// Tendon limit activation margin. Constraint activated when dist < margin.
    /// Default: 0.0 (degenerates to dist < 0.0, i.e., limit-violated-only).
    /// MuJoCo ref: `m->tendon_margin[i]` in `mj_instantiateLimit()`.
    pub tendon_margin: Vec<f64>,
```

Also add initialization in `model_init.rs` (after `tendon_solimp_lim: vec![]`
at line ~287):

```rust
            tendon_margin: vec![],
```

### S2. Wire `tendon_margin` through builder pipeline (DT-33)

**File:** `mjcf/src/builder/mod.rs`, `builder/init.rs`, `builder/tendon.rs`,
`builder/build.rs`
**MuJoCo equivalent:** CopyTree `tendon_margin` population in `user_model.cc`
**Design decision:** Follow the exact same 4-file pattern as `jnt_margin`:
declare on builder struct, initialize, push from parsed value, transfer to
Model. No transformation — direct `unwrap_or(0.0)`.

**builder/mod.rs** — add field to `ModelBuilder` struct (after
`tendon_solimp_lim` at line ~625):

```rust
    pub(crate) tendon_margin: Vec<f64>,
```

**builder/init.rs** — initialize in `ModelBuilder::new()` (after
`tendon_solimp_lim: vec![]` at line ~193):

```rust
            tendon_margin: vec![],
```

**builder/tendon.rs** — push in tendon processing (after
`tendon_solimp_lim` push, before `tendon_num` push, at line ~62):

```rust
            self.tendon_margin.push(tendon.margin.unwrap_or(0.0));
```

**builder/build.rs** — transfer to Model (after `tendon_solimp_lim` at
line ~323):

```rust
            tendon_margin: self.tendon_margin,
```

### S3. Fix assembly.rs tendon limit activation and margin wiring (DT-33)

**File:** `core/src/constraint/assembly.rs`, lines 142–157 (counting) and
545–607 (instantiation)
**MuJoCo equivalent:** `mj_nl()` and `mj_instantiateLimit()` in
`engine_core_constraint.c`
**Design decision:** Read `model.tendon_margin[t]` once per tendon (matching
the joint pattern at line 109 where `margin = model.jnt_margin[jnt_id]` is
read once). Apply to all 6 sites. The counting and instantiation phases use
identical conditions, preserving the critical invariant.

**Phase 1 counting — before (lines 142–157):**
```rust
        // Tendon limits (MuJoCo convention: dist < 0 means violated)
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            // Lower tendon limit: dist = length - limit_min (negative when too short)
            if length - limit_min < 0.0 {
                nefc += 1;
            }
            // Upper tendon limit: dist = limit_max - length (negative when too long)
            if limit_max - length < 0.0 {
                nefc += 1;
            }
        }
```

**Phase 1 counting — after:**
```rust
        // Tendon limits (MuJoCo convention: dist < margin means active)
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            let margin = model.tendon_margin[t];
            // Lower tendon limit: dist = length - limit_min
            if length - limit_min < margin {
                nefc += 1;
            }
            // Upper tendon limit: dist = limit_max - length
            if limit_max - length < margin {
                nefc += 1;
            }
        }
```

**Phase 3e instantiation — before (lines 545–607):**
```rust
        // --- 3e: Tendon limits ---
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            let vel = data.ten_velocity[t];
            let sr = model.tendon_solref_lim[t];
            let si = model.tendon_solimp_lim[t];
            // ...
            let bw = model.tendon_invweight0.get(t).copied().unwrap_or(0.0);

            let dist_lower = length - limit_min;
            if dist_lower < 0.0 {
                // ... Jacobian setup ...
                finalize_row!(
                    sr, si, dist_lower,
                    0.0,       // ← hardcoded margin
                    vel, 0.0, ConstraintType::LimitTendon, 1, t, [0.0; 5], bw
                );
            }

            let dist_upper = limit_max - length;
            if dist_upper < 0.0 {
                // ... Jacobian setup ...
                finalize_row!(
                    sr, si, dist_upper,
                    0.0,       // ← hardcoded margin
                    -vel, 0.0, ConstraintType::LimitTendon, 1, t, [0.0; 5], bw
                );
            }
        }
```

**Phase 3e instantiation — after:**
```rust
        // --- 3e: Tendon limits ---
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            let vel = data.ten_velocity[t];
            let sr = model.tendon_solref_lim[t];
            let si = model.tendon_solimp_lim[t];

            let margin = model.tendon_margin[t];
            let bw = model.tendon_invweight0.get(t).copied().unwrap_or(0.0);

            // Lower tendon limit: dist = length - limit_min
            let dist_lower = length - limit_min;
            if dist_lower < margin {
                // J = +ten_J (MuJoCo convention: pushes length up)
                for col in 0..nv {
                    data.efc_J[(row, col)] = data.ten_J[t][col];
                }
                finalize_row!(
                    sr, si, dist_lower,
                    margin,
                    vel, 0.0, ConstraintType::LimitTendon, 1, t, [0.0; 5], bw
                );
            }

            // Upper tendon limit: dist = limit_max - length
            let dist_upper = limit_max - length;
            if dist_upper < margin {
                // J = -ten_J (MuJoCo convention: pushes length down)
                for col in 0..nv {
                    data.efc_J[(row, col)] = -data.ten_J[t][col];
                }
                finalize_row!(
                    sr, si, dist_upper,
                    margin,
                    -vel, 0.0, ConstraintType::LimitTendon, 1, t, [0.0; 5], bw
                );
            }
        }
```

**Six modification sites — complete inventory:**

| Site | Phase | Line (before) | Change |
|------|-------|---------------|--------|
| 1 | Counting lower | 150 | `< 0.0` → `< margin` |
| 2 | Counting upper | 154 | `< 0.0` → `< margin` |
| 3 | Instantiation lower check | 562 | `< 0.0` → `< margin` |
| 4 | Instantiation lower margin arg | 573 | `0.0` → `margin` |
| 5 | Instantiation upper check | 586 | `< 0.0` → `< margin` |
| 6 | Instantiation upper margin arg | 597 | `0.0` → `margin` |

### S4. DT-23 verification and test coverage

**File:** New tests in `core/src/constraint/assembly.rs` (test module) and
`sim-conformance-tests`
**MuJoCo equivalent:** CopyTree fan-out, `getsolparam()` cases
`mjCNSTR_FRICTION_DOF` and `mjCNSTR_FRICTION_TENDON`
**Design decision:** DT-23 is verification-only — the implementation is
already complete. All work is in the form of new tests that exercise the
existing pipeline with non-default values and multi-DOF joints. No production
code changes expected. If verification reveals a bug, the spec will be
updated before fixing.

**Verification checklist (each item becomes a test assertion):**

| File:line | Current state | Action | Expected value |
|-----------|---------------|--------|----------------|
| parser.rs:669–678 | Exists ✓ — `solreffriction` parsed from `<joint>` | Verify (T9) | `MjcfJoint.solreffriction: Option<[f64; 2]>` populated from XML |
| defaults.rs:209–213 | Exists ✓ — `solreffriction` cascaded from `<default><joint>` | Verify (T9) | DOF receives cascaded `[0.03, 0.7]` from default class |
| joint.rs:160–163 | Exists ✓ — CopyTree fan-out for ball (3 DOFs) | Verify (T7) | `model.dof_solref[dof_adr..dof_adr+3]` all equal `[0.05, 0.8]` |
| joint.rs:160–163 | Exists ✓ — CopyTree fan-out for free (6 DOFs) | Verify (T8) | `model.dof_solref[dof_adr..dof_adr+6]` all equal `[0.05, 0.8]` |
| assembly.rs:391–392 | Exists ✓ — `model.dof_solref[dof_idx]` read for DOF friction | Verify (T10) | `efc_solref[row] == [0.05, 0.8]` (non-default value) |
| assembly.rs:418–419 | Exists ✓ — `model.tendon_solref_fri[t]` read for tendon friction | Verify (T11) | `efc_solref[row] == [0.04, 0.9]` (non-default value) |

---

## Acceptance Criteria

### AC1: Tendon margin=0 regression *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `range=(-1.0, 1.0)`, `margin=0.0`,
`ten_length=-1.5` (violates lower limit by 0.5)
**After:** `assemble_unified_constraints()`
**Assert:** `nefc >= 1`, `efc_margin[0] == 0.0`, `efc_pos[0] == -0.5`
**Field:** `data.efc_margin`, `data.efc_pos`, constraint count
**Rationale:** When margin=0, behavior is identical to the pre-change
hardcoded `< 0.0`. This is the regression guard.

### AC2: Tendon margin>0 pre-activation *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `range=(-1.0, 1.0)`, `margin=0.1`,
`ten_length=-0.95` (within limit but `dist = -0.95 - (-1.0) = 0.05 < 0.1`)
**After:** `assemble_unified_constraints()`
**Assert:** `nefc >= 1` (constraint pre-activated), `efc_margin[0] == 0.1`,
`efc_pos[0] ≈ 0.05`
**Field:** `data.efc_margin`, `data.efc_pos`, constraint count

### AC3: Tendon margin in `efc_margin` field *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `margin=0.2`, limit violated
**After:** `assemble_unified_constraints()`
**Assert:** `efc_margin[0] == 0.2` (not `0.0`)
**Field:** `data.efc_margin`
**Rationale:** Confirms the margin value flows through `finalize_row!` to
`efc_margin`, which feeds `compute_impedance()` and `compute_aref()`.

### AC4: Counting and instantiation phases agree with margin>0 *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `margin=0.1`, tendon positioned to
pre-activate (dist between 0 and margin)
**After:** `assemble_unified_constraints()`
**Assert:** No panic (buffer sizes match), correct row count
**Field:** Implicit — if counting allocates N rows but instantiation produces
M ≠ N, the function panics or produces garbage.

### AC5: DISABLE_LIMIT ignores tendon margin *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `margin=0.5`, `ten_length=0.0`
(within margin of both limits), `DISABLE_LIMIT` flag set
**After:** `assemble_unified_constraints()`
**Assert:** `nefc == 0` (no limit constraints)
**Field:** Constraint count

### AC6: Negative margin shrinks activation zone *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `range=(-1.0, 1.0)`, `margin=-0.1`.
Two sub-cases:
- Case A: `ten_length=-1.05` → `dist = -1.05 - (-1.0) = -0.05`. Is `-0.05 < -0.1`? No → constraint NOT active.
- Case B: `ten_length=-1.15` → `dist = -1.15 - (-1.0) = -0.15`. Is `-0.15 < -0.1`? Yes → constraint active.
**After:** `assemble_unified_constraints()`
**Assert:** Case A: `nefc == 0` (violation too small to exceed dead zone).
Case B: `nefc >= 1` (violation exceeds `|margin|`).
**Field:** Constraint count

### AC7: Large margin overlap — both limits active *(runtime test — analytically derived)*
**Given:** Model with 1 limited tendon, `range=(-0.5, 0.5)`, `margin=0.6`,
`ten_length=0.0` (lower dist = `0.0 - (-0.5) = 0.5 < 0.6` ✓; upper dist =
`0.5 - 0.0 = 0.5 < 0.6` ✓)
**After:** `assemble_unified_constraints()`
**Assert:** `nefc >= 2` (both lower and upper limits simultaneously active)
**Field:** Constraint count, `efc_pos` values

### AC8: Ball joint 3 DOFs identical dof_solref *(runtime test — analytically derived)*
**Given:** Model with 1 ball joint, `solreffriction="0.05 0.8"`,
`frictionloss=1.0`
**After:** Model build
**Assert:** `model.dof_solref[dof_adr] == model.dof_solref[dof_adr+1] ==
model.dof_solref[dof_adr+2] == [0.05, 0.8]`
**Field:** `model.dof_solref`

### AC9: Free joint 6 DOFs identical dof_solref *(runtime test — analytically derived)*
**Given:** Model with 1 free joint, `solreffriction="0.05 0.8"`,
`frictionloss=1.0`
**After:** Model build
**Assert:** `model.dof_solref[0..6]` all equal `[0.05, 0.8]`
**Field:** `model.dof_solref`

### AC10: Defaults cascade for solreffriction *(runtime test — analytically derived)*
**Given:** `<default><joint solreffriction="0.03 0.7"/></default>`, joint
without explicit `solreffriction`
**After:** Model build
**Assert:** `model.dof_solref[0] == [0.03, 0.7]`
**Field:** `model.dof_solref`

### AC11: Non-default solreffriction in constraint rows *(runtime test — analytically derived)*
**Given:** Model with hinge joint, `frictionloss=1.0`,
`solreffriction="0.05 0.8"` (non-default)
**After:** `assemble_unified_constraints()`
**Assert:** `efc_solref[friction_row] == [0.05, 0.8]` (not default `[0.02, 1.0]`)
**Field:** `data.efc_solref`

### AC12: Tendon friction solref end-to-end *(runtime test — analytically derived)*
**Given:** Model with 1 tendon, `frictionloss=1.0`,
`solreffriction="0.04 0.9"` (non-default)
**After:** `assemble_unified_constraints()`
**Assert:** `efc_solref[tendon_friction_row] == [0.04, 0.9]`
**Field:** `data.efc_solref`

### AC13: Pipeline completeness *(code review — not a runtime test)*
All 6 locations for `tendon_margin` field are implemented:
1. `model.rs` — field declaration with doc comment
2. `model_init.rs` — `tendon_margin: vec![]`
3. `builder/mod.rs` — `pub(crate) tendon_margin: Vec<f64>`
4. `builder/init.rs` — `tendon_margin: vec![]`
5. `builder/tendon.rs` — `self.tendon_margin.push(tendon.margin.unwrap_or(0.0))`
6. `builder/build.rs` — `tendon_margin: self.tendon_margin`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (margin=0 regression) | T1 | Direct |
| AC2 (margin>0 pre-activation) | T2 | Direct |
| AC3 (efc_margin field) | T2, T3 | Direct (T2 checks efc_margin; T3 checks upper limit) |
| AC4 (counting/instantiation agreement) | T2, T5 | Direct (pre-activation triggers counting path) |
| AC5 (DISABLE_LIMIT) | T4 | Direct |
| AC6 (negative margin) | T5 | Direct |
| AC7 (large margin overlap) | T6 | Direct |
| AC8 (ball 3 DOFs) | T7 | Direct |
| AC9 (free 6 DOFs) | T8 | Direct |
| AC10 (defaults cascade) | T9 | Direct |
| AC11 (non-default solref in rows) | T10 | Direct |
| AC12 (tendon friction solref) | T11 | Direct |
| AC13 (pipeline completeness) | — | Code review (manual) |

---

## Test Plan

### T1: Tendon margin=0 regression → AC1
**Model:** 2-body, 1 hinge joint, 1 fixed tendon with `limited=true`,
`range=(-1.0, 1.0)`, `margin=0.0`. Set `ten_length = -1.5` (violates lower
limit).
**Expected:** `nefc >= 1`, `efc_margin[0] == 0.0`, `efc_pos[0] == -0.5`
(identical to pre-change behavior).
**Rationale:** Regression guard — margin=0 must produce exactly the same
results as the old hardcoded `< 0.0` check.
**Expected value source:** Analytically derived (margin=0 degenerates to
`dist < 0` which is the pre-existing behavior).

### T2: Tendon margin>0 pre-activation (lower limit) → AC2, AC3, AC4
**Model:** Same as T1 but `margin=0.1`, `ten_length = -0.95`
(lower dist = `0.05 < 0.1` → pre-activated).
**Expected:** `nefc >= 1`, `efc_margin[0] == 0.1`, `efc_pos[0] ≈ 0.05`.
**Rationale:** Core margin pre-activation behavior — the whole point of DT-33.
Without margin, `dist = 0.05 > 0.0` would NOT activate. With margin=0.1,
`dist = 0.05 < 0.1` activates.

### T3: Tendon margin>0 pre-activation (upper limit) → AC3
**Model:** Same as T1 but `margin=0.1`, `ten_length = 0.95`
(upper dist = `1.0 - 0.95 = 0.05 < 0.1` → pre-activated).
**Expected:** `nefc >= 1`, `efc_margin[0] == 0.1`, `efc_pos[0] ≈ 0.05`,
Jacobian is `-ten_J` (upper limit sign).
**Rationale:** Verifies upper limit uses the same margin-aware check.

### T4: DISABLE_LIMIT ignores tendon margin → AC5
**Model:** Same as T2 (margin=0.1, within margin zone), with `DISABLE_LIMIT`
flag set.
**Expected:** `nefc == 0`.
**Rationale:** The `!limit_disabled` guard (assembly.rs:100) wraps the entire
tendon limit section, preventing any rows regardless of margin.

### T5: Negative margin shrinks activation zone → AC6
**Model:** Tendon with `range=(-1.0, 1.0)`, `margin=-0.1`.
- Case A: `ten_length=-1.05` (`dist = -0.05`). Is `-0.05 < -0.1`? No → no row.
- Case B: `ten_length=-1.15` (`dist = -0.15`). Is `-0.15 < -0.1`? Yes → row.
**Expected:** Case A: `nefc == 0`. Case B: `nefc >= 1`.
**Rationale:** Negative margin creates a "dead zone" — constraint only fires
when violation exceeds `|margin|`.

### T6: Large margin overlap — both limits active → AC7
**Model:** Tendon with `range=(-0.5, 0.5)`, `margin=0.6`, `ten_length=0.0`.
Lower dist = `0.5 < 0.6` ✓. Upper dist = `0.5 < 0.6` ✓.
**Expected:** `nefc >= 2`, two constraint rows (one lower, one upper).
**Rationale:** When margin > `(limit_max - limit_min) / 2`, both limits
activate simultaneously. MuJoCo handles this without guard.

### T7: Ball joint 3 DOFs identical dof_solref → AC8
**Model:** 2-body, 1 ball joint with `solreffriction="0.05 0.8"`,
`frictionloss=1.0`.
**Expected:** `model.dof_solref[dof_adr]`, `model.dof_solref[dof_adr+1]`,
`model.dof_solref[dof_adr+2]` all equal `[0.05, 0.8]`.
**Rationale:** Verifies builder fan-out loop (joint.rs:160–163) correctly
copies `solreffriction` to all 3 DOFs of a ball joint.

### T8: Free joint 6 DOFs identical dof_solref → AC9
**Model:** 2-body, 1 free joint with `solreffriction="0.05 0.8"`,
`frictionloss=1.0`.
**Expected:** All 6 entries `model.dof_solref[0..6]` equal `[0.05, 0.8]`.
**Rationale:** Free joint has 6 DOFs (3 translational + 3 rotational). All
must receive the same `solreffriction` from the parent joint.

### T9: Defaults cascade for solreffriction → AC10
**Model:** MJCF with `<default><joint solreffriction="0.03 0.7"/></default>`,
hinge joint without explicit `solreffriction`.
**Expected:** `model.dof_solref[0] == [0.03, 0.7]`.
**Rationale:** Verifies the defaults cascade path (defaults.rs:209–213)
correctly overrides the global default `[0.02, 1.0]`.

### T10: Non-default solreffriction in constraint solver params → AC11
**Model:** Hinge joint with `frictionloss=1.0`,
`solreffriction="0.05 0.8"`.
**Expected:** After `assemble_unified_constraints()`,
`efc_solref[friction_row] == [0.05, 0.8]`.
**Rationale:** Verifies the full pipeline from parsed attribute → defaults →
builder → model → assembly. The constraint row receives the non-default
solver params, confirming MuJoCo-conformant behavior.

### T11: Tendon friction solref end-to-end → AC12
**Model:** Tendon with `frictionloss=1.0`,
`solreffriction="0.04 0.9"`.
**Expected:** After `assemble_unified_constraints()`,
`efc_solref[tendon_friction_row] == [0.04, 0.9]`.
**Rationale:** Verifies tendon friction solver params are wired end-to-end
(parsing → builder → model → assembly), mirroring the DOF friction
verification.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `margin = 0.0` (default) | Regression: must produce identical behavior to pre-change hardcoded `< 0.0` | T1 | AC1 |
| `margin > 0` (pre-activation) | Core feature: constraint fires before limit violation | T2, T3 | AC2, AC3 |
| `margin < 0` (dead zone) | Valid in MuJoCo: shrinks activation zone, ignores small violations | T5 | AC6 |
| Large margin overlap | Both limits fire simultaneously when margin > half range width | T6 | AC7 |
| `DISABLE_LIMIT` with margin | Limit flag overrides margin — no rows regardless of margin value | T4 | AC5 |
| `tendon_limited = false` with margin | Inactive tendon ignores margin | T1 helper (skip non-limited) | AC1 |
| Ball joint (3 DOFs) solreffriction | Builder fan-out: all 3 DOFs must get identical values | T7 | AC8 |
| Free joint (6 DOFs) solreffriction | Builder fan-out: all 6 DOFs must get identical values | T8 | AC9 |
| Defaults cascade override | Class-level `solreffriction` overrides global default | T9 | AC10 |
| `frictionloss = 0.0` | No friction row emitted regardless of `solreffriction` value | T10 helper (verify row count = 0 with fl=0) | AC11 |
| World body tendon attachment | Tendon Jacobian has zeros for world DOFs — margin activation still correct with zero Jacobian rows | T12 (multi-tendon model includes world-attached tendon) | AC1 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T12: Multi-tendon indexing | 3 tendons with different margins, verify correct margin applied to each | Catches off-by-one indexing bugs in `model.tendon_margin[t]` |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Tendon limit activation with margin>0 | Constraint activates only when limit violated (`dist < 0.0`) | Constraint activates when `dist < margin` (before violation when margin>0) | **Toward MuJoCo** — closes conformance gap | Models using `tendon margin="..."` attribute | None — transparent for margin=0 (default); MuJoCo-conformant for margin>0 |
| Tendon limit `efc_margin` values | Always `0.0` for tendon limit rows | Actual `tendon_margin[t]` value | **Toward MuJoCo** — correct impedance ramp-up | Impedance and aref computation for tendon limits | None — transparent change, produces correct physics |
| DT-23 (no behavioral change) | — | — | Already conformant | — | — |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/types/model.rs` | Add `tendon_margin: Vec<f64>` field + doc comment | +4 |
| `core/src/types/model_init.rs` | Add `tendon_margin: vec![]` initialization | +1 |
| `mjcf/src/builder/mod.rs` | Add `pub(crate) tendon_margin: Vec<f64>` to ModelBuilder struct | +1 |
| `mjcf/src/builder/init.rs` | Add `tendon_margin: vec![]` initialization | +1 |
| `mjcf/src/builder/tendon.rs` | Add `self.tendon_margin.push(tendon.margin.unwrap_or(0.0))` | +2 |
| `mjcf/src/builder/build.rs` | Add `tendon_margin: self.tendon_margin` transfer | +1 |
| `core/src/constraint/assembly.rs` | Replace 4 hardcoded `< 0.0` with `< margin`, 2 hardcoded `0.0` with `margin` | ~10 modified |
| `core/src/constraint/assembly.rs` (tests) | New test module for tendon margin (T1–T6, T12) + DT-23 tests (T7–T11) | +200–300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `t8_margin_zero_regression` | assembly.rs:904 | Pass (unchanged) | Tests joint margin, not tendon margin |
| `t9_margin_pre_activation` | assembly.rs:931 | Pass (unchanged) | Tests joint margin |
| `t10_ball_margin` | assembly.rs:955 | Pass (unchanged) | Tests ball joint limit margin |
| `t11_disable_limit_ignores_margin` | assembly.rs:1045 | Pass (unchanged) | Tests joint DISABLE_LIMIT |
| Friction loss integration tests | unified_solvers.rs:80,83,95,108,147 | Pass (unchanged) | Use default solref values; no tendon margin involved |
| Tendon length/velocity tests | sim-tendon | Pass (unchanged) | Test tendon computation, not constraint assembly |
| Phase 7 parsing/defaults tests | sim-mjcf | Pass (unchanged) | Phase 8 does not modify parsing or defaults code |

**Data staleness guards:** No `EXPECTED_SIZE` constants or static assertions
exist in the affected code paths. Adding `tendon_margin` to Model has zero
serialization impact (Model has no serde derives). The only compile-time
check is Rust's exhaustive struct initialization — which will catch any
missing field in `build.rs` or `model_init.rs` at build time.

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| assembly.rs:109 | `let margin = model.jnt_margin[jnt_id]` (joint limit counting) | Already uses `jnt_margin` — correct, different subsystem |
| assembly.rs:130 | `let margin = model.jnt_margin[jnt_id]` (ball joint counting) | Already uses `jnt_margin` — correct |
| assembly.rs:391–392 | `model.dof_solref[dof_idx]`, `model.dof_solimp[dof_idx]` (DOF friction) | Already correct — DT-23 is verification-only |
| assembly.rs:418–419 | `model.tendon_solref_fri[t]`, `model.tendon_solimp_fri[t]` (tendon friction) | Already correct — DT-23 is verification-only |
| impedance.rs:48–107 | `compute_impedance()` — uses `(pos - margin).abs()` | Already correct — margin flows through `finalize_row!` automatically |
| impedance.rs:196–202 | `compute_aref()` — uses `K * imp * (pos - margin)` | Already correct — no changes needed |
| impedance.rs:144–192 | `compute_kbip()` — independent of margin | Not affected |

---

## Execution Order

1. **S1** (model field) + **S2** (builder wiring) → verifies compilation.
   S2 depends on S1 because the builder transfers to Model fields that must
   exist. Run `cargo build -p sim-core -p sim-mjcf` to verify.

2. **S4** (DT-23 verification) → independent of S1–S3 (different assembly.rs
   sections: friction loss at lines 379–430 vs tendon limits at 545–607).
   Run S4 first as a precaution: if DT-23 verification reveals a bug in the
   existing DOF friction code, the fix might touch assembly.rs near DT-33's
   changes. Implement T7–T11 and verify AC8–AC12.

3. **S3** (assembly fixes) → requires S1 + S2 (model field must exist for
   `model.tendon_margin[t]` to compile). Run
   `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-tendon -p sim-conformance-tests`
   to verify all existing tests pass. Then implement T1–T6, T12 (tendon
   margin tests) and verify AC1–AC7.

4. **AC13** (pipeline code review) → after S1–S3. Manual verification that
   all 6 locations are wired.

After each section lands, run the domain test suite to verify no regressions:
```
cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-tendon -p sim-conformance-tests
```

---

## Out of Scope

- **`solreffriction` on contacts** (per-direction friction solver params on
  `<geom>` or `<pair>`) — separate scope from per-DOF/per-tendon friction loss
  params. Conformance impact: affects models with per-contact friction solver
  tuning, acceptable for v1.0.

- **Mixed-sign solref validation** (`(solref[0] > 0) ^ (solref[1] > 0)` →
  MuJoCo warns and replaces with default) — affects ALL constraint types, not
  specific to Spec A. Discovered in rubric stress test round 3, noted in gap
  log R11. Conformance impact: minor edge case. Tracked for future DT item.

- **Tendon limit `solref_limit`/`solimp_limit` naming** — already done in
  DT-32 (commit `9f9cf9f`). No further work needed.

- **`efc_impP` impedance derivative field** (DT-22) — API introspection only,
  not used by solvers. Deferred to post-v1.0.

- **Condim=4/6 for deformable contacts** — DT-25 verification scope, not
  Spec A.
