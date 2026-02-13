# Future Work 7 — Phase 3A: Inertia + Contact Parameter Foundation (Items #23–27)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers inertia computation and the contact parameter combination layer — the
foundation that all contact constraint assembly builds on.

---

### 23. `<compiler>` `exactmeshinertia`
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed. `MjcfCompiler` struct has many attributes but does not include
`exactmeshinertia`. Mesh inertia computation may not match MuJoCo's behavior
when this flag is set.

#### Objective

Parse `exactmeshinertia` from `<compiler>` and use it to control mesh inertia
computation method.

#### Specification

1. **MJCF parsing**: Parse `exactmeshinertia` (bool, default false) from
   `<compiler>`.
2. **Storage**: Add to `MjcfCompiler` struct.
3. **Effect on mesh inertia**:
   - When `false` (default): compute mesh inertia using bounding-box
     approximation (current behavior, if applicable).
   - When `true`: compute mesh inertia using exact triangle integration
     (sum of per-triangle inertia contributions using the divergence theorem).
4. **Algorithm (exact)**: For each triangle in the mesh, compute the contribution
   to mass, center of mass, and inertia tensor using the signed volume formula.
   This is a well-known algorithm (Mirtich 1996 / Eberly) that computes exact
   inertia for a closed triangle mesh.
5. **Scope**: Only affects `<geom type="mesh">` elements where inertia is
   auto-computed (no explicit `<inertial>`). Bodies with explicit `<inertial>`
   are unaffected.

#### Acceptance Criteria

1. `<compiler exactmeshinertia="false"/>` preserves current behavior.
2. `<compiler exactmeshinertia="true"/>` computes exact mesh inertia.
3. A cube mesh with `exactmeshinertia="true"` matches the analytical box
   inertia formula.
4. Non-mesh geoms are unaffected regardless of the flag.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse compiler flag, mesh inertia
  computation path

---

### 24. Friction Combination Rule: Geometric Mean → Element-Wise Max
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`make_contact_from_geoms()` (`mujoco_pipeline.rs:5690`) combines friction
coefficients from two contacting geoms using **geometric mean**:

```rust
let sliding = (f1.x * f2.x).sqrt();
let torsional = (f1.y * f2.y).sqrt();
let rolling = (f1.z * f2.z).sqrt();
```

The same pattern is used for deformable-rigid contacts (`mujoco_pipeline.rs:19230`).

MuJoCo uses **element-wise maximum** (when geom priorities are equal):

```c
// engine_collision_driver.c — mj_contactParam()
for (int i=0; i < 3; i++) {
    fri[i] = mju_max(friction1[i], friction2[i]);
}
```

This divergence was documented as "known non-conformance" in `future_work_2.md`
(Decision D3) and explicitly deferred: "better done as a deliberate conformance
task with before/after validation, not buried in a condim refactor."

**Impact:** Every contact between geoms with asymmetric friction values produces
wrong friction coefficients. This affects every trajectory comparison test.

#### Objective

Switch friction combination from geometric mean to element-wise max, matching
MuJoCo's `mj_contactParam()` behavior for equal-priority geoms.

#### Specification

1. **`make_contact_from_geoms()`** (`mujoco_pipeline.rs:5690`): Change three lines:
   ```rust
   let sliding = f1.x.max(f2.x);
   let torsional = f1.y.max(f2.y);
   let rolling = f1.z.max(f2.z);
   ```

2. **Deformable-rigid contacts** (`mujoco_pipeline.rs:19230`): Same change pattern.
   Deformable material has a single `friction: f64` combined with rigid geom's
   per-axis friction:
   ```rust
   let sliding = deform_friction.max(rigid_friction.x);
   let torsional = deform_friction.max(rigid_friction.y);
   let rolling = deform_friction.max(rigid_friction.z);
   ```

3. **`<contact><pair>` override**: When a `<pair>` element specifies explicit
   `friction`, it overrides the combination rule entirely — no change needed here.

4. **Geom priority**: MuJoCo uses `geom/@priority` to select which geom's
   friction wins outright (higher priority geom's friction used directly). Our
   parser does not read `priority`. This is out of scope — `priority` is item
   #25. For equal-priority geoms (the only case we handle), element-wise max is
   correct.

5. **Doc correction**: Update any documentation that claims geometric mean
   "matching MuJoCo's combination rule" — this is factually wrong. State
   element-wise max.

#### Acceptance Criteria

1. `make_contact_from_geoms()` uses `f1.max(f2)` per component.
2. Deformable-rigid contact friction uses `max()` combination.
3. Existing contact tests updated — any test that relied on geometric mean
   behavior must be re-validated. Expected: tolerance tightening, not loosening.
4. New test: two geoms with `friction="0.3"` and `friction="0.7"` produce
   contact with `sliding = 0.7` (not `sqrt(0.21) ≈ 0.458`).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `make_contact_from_geoms()`, deformable contacts
- `sim/L0/tests/integration/` — contact friction tests

---

### 25. `geom/@priority` — Contact Priority
**Status:** Not started | **Effort:** S | **Prerequisites:** None (soft dep: after #24)

#### Current State

Not parsed, not stored, not used. Zero references in codebase. Contact parameter
combination currently always uses element-wise max for friction and element-wise
min/max for solref/solimp, regardless of priority.

#### Objective

Parse `priority` from `<geom>`, store on Model, and use it in contact parameter
combination to select the higher-priority geom's parameters outright when
priorities differ.

#### Specification

1. **MJCF parsing**: Parse `priority` (int, default 0) from `<geom>`. Also parse
   from `<default>` class.
2. **Model storage**: Add `geom_priority: Vec<i32>` to `Model`.
3. **Contact parameter combination** (in `make_contact_from_geoms()` or equivalent):
   - If `priority[g1] > priority[g2]`: use g1's friction, solref, solimp, solmix.
   - If `priority[g1] < priority[g2]`: use g2's friction, solref, solimp, solmix.
   - If `priority[g1] == priority[g2]`: use the existing combination rule
     (element-wise max for friction — or weighted average after #26 solmix).
4. **Interaction with #24 (friction combination)**: Priority check happens BEFORE
   the combination rule. If priorities differ, no combination is needed. This is
   why #25 has a soft dependency on #24 — the combination rule it falls back to
   should be the correct MuJoCo rule, not the current approximation.

#### Acceptance Criteria

1. Geom with `priority="1"` vs geom with `priority="0"`: the priority-1 geom's
   friction/solver params are used verbatim.
2. Equal priority geoms use the existing combination rule.
3. Default priority (0) matches MuJoCo behavior.
4. `<contact><pair>` explicit pairs respect priority overrides (if specified).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse priority
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, contact parameter selection

---

### 26. `solmix` Attribute
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed. Current implementation uses element-wise min for solref and
element-wise max for solimp as an approximation. Documented as known divergence
in `future_work_2.md`.

#### Objective

Parse `solmix` from `<geom>` and use it for weighted averaging of solver
parameters in contact pairs.

#### Specification

1. **MJCF parsing**: Parse `solmix` (float, default 1.0) from `<geom>`. Also
   parse from `<default>` class.
2. **Model storage**: Add `geom_solmix: Vec<f64>` to `Model`.
3. **Contact parameter combination** (MuJoCo's `mj_contactParam` logic):
   - Compute mixing weight: `w1 = solmix[g1] / (solmix[g1] + solmix[g2])`,
     `w2 = 1 - w1`.
   - `solref_combined = w1 * solref[g1] + w2 * solref[g2]` (weighted average).
   - `solimp_combined = w1 * solimp[g1] + w2 * solimp[g2]` (weighted average).
   - Friction: NOT affected by solmix — friction uses element-wise max (#24) or
     priority override (#25).
4. **Edge case**: If both `solmix` values are 0, use equal weights (0.5/0.5).
5. **Interaction with #25 (priority)**: If priorities differ, the higher-priority
   geom's parameters are used directly — solmix is not consulted. Solmix only
   applies when priorities are equal.

#### Acceptance Criteria

1. Two geoms with `solmix="1"` (default) produce equal-weighted combination.
2. Geom with `solmix="2"` vs `solmix="1"`: first geom has 2/3 weight.
3. Solmix=0 edge case handled without NaN/inf.
4. `<contact><pair>` can override solref/solimp directly (bypass solmix).
5. Existing contact tests pass with default solmix=1 (regression).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse solmix
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, weighted combination in
  contact parameter assembly

---

### 27. Contact Margin/Gap Runtime Effect
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`geom_margin` and `geom_gap` are stored on `Model` as `Vec<f64>` (one per geom,
initialized to 0.0). `ContactPair` has `margin` and `gap` fields, correctly parsed
from `<contact><pair>` MJCF elements. However, margin and gap have **no runtime
effect** on contact activation or constraint assembly:

- **Narrow-phase**: All collision functions (e.g., `collide_sphere_sphere`) generate
  contacts only when `penetration > 0.0`. Margin is not checked.
  (`mujoco_pipeline.rs`, `collide_sphere_sphere`, line ~6509.)
- **Constraint assembly**: `margin` is hardcoded to `0.0` in `assemble_contact_system`
  (`mujoco_pipeline.rs`, line ~14600: `let margin = 0.0_f64;`). The `compute_aref()`
  function accepts a `margin` parameter but always receives 0.0.
- **`apply_pair_overrides`**: Applies condim, friction, solref, solimp from
  `ContactPair` but skips margin/gap. Comment at line ~5664: "margin/gap are NOT
  applied here."
- **Broad-phase**: `pair.margin` IS used in the bounding-sphere distance cull for
  explicit pairs (line ~5431). This is correct and should be preserved.
- **Geom-level parsing**: `<geom margin="..." gap="...">` attributes are NOT parsed.
  `model_builder.rs` comment: "geom-level not yet parsed, default to 0.0."
- **`efc_margin`**: The `Data` struct has an `efc_margin: Vec<f64>` field (per
  constraint row), currently always populated with 0.0.

MuJoCo's margin/gap system involves three distinct locations: contact detection
(wider activation), constraint violation computation (shifted reference), and the
`aref` stabilization formula. All three must be wired up.

#### Objective

Wire `margin` and `gap` into the full pipeline — MJCF parsing, contact detection,
and constraint assembly — so that contacts activate at the correct distance and
Baumgarte stabilization uses the correct reference point.

#### Specification

##### S1. Margin/gap combination rule (MuJoCo semantics)

MuJoCo **sums** margins and gaps from both geoms (source:
`engine_collision_driver.c`, `mj_collideGeoms`):

```
effective_margin = geom_margin[g1] + geom_margin[g2]
effective_gap    = geom_gap[g1]    + geom_gap[g2]
includemargin    = effective_margin - effective_gap
```

For explicit `<contact><pair>` contacts, the pair's `margin` and `gap` attributes
**override** the geom-summed values entirely (pair takes precedence).

The `includemargin` value is the distance threshold below which a contact generates
a constraint. It is stored on the `Contact` struct and propagated to `efc_margin`
during constraint assembly.

##### S2. Geom-level MJCF parsing

Parse `margin` and `gap` attributes from `<geom>` elements. These cascade through
the defaults system (existing infrastructure handles attribute cascading):

```xml
<geom type="sphere" size="0.1" margin="0.002" gap="0.001"/>
```

Default values: `margin = 0.0`, `gap = 0.0` (preserves current behavior when
attributes are absent).

Update `Model.geom_margin` and `Model.geom_gap` arrays, which are already
allocated (currently zeroed). Wire through `model_builder.rs` → `process_geom()`.

##### S3. Narrow-phase contact activation

Change the activation condition in all narrow-phase collision functions from:

```rust
if penetration > 0.0 { ... }  // current
```

to:

```rust
if penetration > -margin { ... }  // with margin
```

where `margin` is the effective margin computed per S1. When a contact is generated
with `penetration ∈ (-margin, 0]`, the contact's `depth` field stores the
actual geometric penetration (which is negative — surfaces are separated). The
`includemargin` field on the Contact stores the margin value.

This means the contact exists in the constraint system but may produce zero force
(if `depth > includemargin`, the constraint violation `r > 0` so no corrective
force is applied). This is MuJoCo's mechanism for "pre-contact" — smooth force
onset rather than sudden activation.

Affected functions: `collide_sphere_sphere`, `collide_sphere_plane`,
`collide_capsule_plane`, `collide_box_plane`, `collide_sphere_capsule`,
and all other `collide_*` variants. Each function must receive the effective
margin for the geom pair.

Implementation approach: pass `margin` as a parameter to each `collide_*`
function. Compute `margin` in the dispatch function (`collide_geoms` or explicit
pair loop) using S1, and thread it through.

##### S4. Contact struct: store `includemargin`

The `Contact` struct already has a `pub includemargin: bool` field (currently
always `false`). Replace this with a numeric field:

```rust
/// Distance threshold used for constraint activation.
/// `includemargin = effective_margin - effective_gap`.
/// Constraint violation: `r = depth - includemargin`.
/// Contact exists when `depth > -effective_margin` (see S3).
pub includemargin: f64,
```

Set by `make_contact_from_geoms()` or `apply_pair_overrides()`.

##### S5. `apply_pair_overrides`: apply margin/gap from ContactPair

In `apply_pair_overrides()`, set `contact.includemargin = pair.margin - pair.gap`
when the contact comes from an explicit `<contact><pair>`. Remove the existing
comment "margin/gap are NOT applied here."

For automatic (Mechanism 1) contacts, set
`contact.includemargin = (geom_margin[g1] + geom_margin[g2]) - (geom_gap[g1] + geom_gap[g2])`
in `make_contact_from_geoms()`.

##### S6. Constraint assembly: populate `efc_margin`

In `assemble_contact_system` (line ~14598), replace the hardcoded
`let margin = 0.0_f64` with:

```rust
let margin = contact.includemargin;
```

This value flows into:
1. `data.efc_margin[row] = margin` — per constraint row (normal direction only;
   friction rows get `efc_margin = 0.0`, matching MuJoCo).
2. `compute_aref(k, b, imp, pos, margin, vel)` — the existing function already
   computes `aref = -b * vel - k * imp * (pos - margin)`. With nonzero margin,
   the constraint equilibrium shifts to `pos = margin` instead of `pos = 0`.

No changes to `compute_aref()` itself — the function is already correct; it just
needs a nonzero margin input.

##### S7. Interaction with global `<option>` override

MuJoCo supports `<option o_margin="...">` which overrides all per-geom and
per-pair margins globally. This is a low-priority extension — defer unless existing
models require it. Note in code with a TODO.

#### Acceptance Criteria

1. **Margin activation**: A sphere at height 1mm above a plane with
   `geom_margin="0.002"` (each geom) generates a contact. Effective margin =
   `0.002 + 0.002 = 0.004 > 0.001` (distance). Contact `depth ≈ -0.001`
   (negative, surfaces separated), `includemargin = 0.004`, constraint
   `r = -0.001 - 0.004 = -0.005 < 0` → constraint force engages.
2. **Gap buffer**: With `margin="0.004"` and `gap="0.003"`, `includemargin = 0.001`.
   A sphere at distance 0.002 above the plane generates a contact (within margin)
   but `r = -0.002 - 0.001 = -0.003 < 0` → force engages. At distance 0.0005,
   still in contact, stronger force. Verify force magnitude scales with
   `|r|` (deeper violation → stronger force).
3. **Pair override**: An explicit `<contact><pair margin="0.01" gap="0.005">`
   uses `includemargin = 0.005`, ignoring geom-level margins.
4. **Zero margin/gap regression**: All existing tests pass unchanged (default
   margin/gap = 0.0 preserves `penetration > 0.0` activation and
   `efc_margin = 0.0` in assembly).
5. **MuJoCo reference**: Compare contact count and forces against MuJoCo 3.4.0
   for a sphere-on-plane test at 5 separation distances spanning
   `[-0.002, +0.003]` with `margin=0.004, gap=0.001`. Contact count and
   `qfrc_constraint` must match within tolerance `1e-8`.
6. **`efc_margin` populated**: After `forward()`, `data.efc_margin[normal_row]`
   equals `contact.includemargin` for every active contact.

#### Implementation Notes

**Narrow-phase function signature change.** Every `collide_*` function currently
takes `(model, geom1, geom2, pos1, pos2, size1, size2) -> Option<Contact>`. Add
a `margin: f64` parameter. The dispatch function computes the effective margin
(S1) before calling the specific collider.

**Existing broad-phase is correct.** The bounding-sphere cull at line ~5431
already adds `pair.margin` to the distance check. For automatic contacts, add
`geom_margin[g1] + geom_margin[g2]` to the broad-phase distance threshold.

**Breaking change to Contact struct.** Replacing `includemargin: bool` with
`includemargin: f64` is a breaking change. Any existing code that checks
`contact.includemargin` as a bool must be updated. Search for all uses.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — all `collide_*` functions,
  `make_contact_from_geoms()`, `apply_pair_overrides()`,
  `assemble_contact_system()` (margin wiring), broad-phase distance check
- `sim/L0/mjcf/src/parser.rs` — parse `margin`/`gap` from `<geom>` elements
- `sim/L0/mjcf/src/model_builder.rs` — wire geom-level margin/gap into
  `Model.geom_margin`/`Model.geom_gap` in `process_geom()`
- `sim/L0/mjcf/src/defaults.rs` — ensure margin/gap cascade through defaults
- `sim/L0/tests/integration/` — new test file `contact_margin_gap.rs`
