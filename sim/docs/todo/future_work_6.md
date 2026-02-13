# Future Work 6 — Phase 3A: Foundation + Core Correctness (Items #18–22)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items are ordered foundationally: `<include>` unlocks loading production models,
the conformance suite unlocks verifying everything that follows, then core contact
correctness, then solver/actuator completeness.

---

### 18. `<include>` File Support + `<compiler>` Element
**Status:** ✅ Done | **Effort:** L | **Prerequisites:** None

#### Current State

**`<include>` element:** The parser silently skips `<include>` elements (they fall
through to `skip_element()` in `parse_mujoco`). The `IncludeNotSupported` error
variant exists in `error.rs` but is never raised. Many production MJCF models
(MuJoCo Menagerie, DeepMind Control Suite) use `<include>` for modular definitions.
This blocks loading a significant fraction of real-world models.

**`<compiler>` element:** Completely skipped during parsing. This causes **silent
correctness bugs** in every model that relies on compiler defaults:

- **`angle="degree"` (MuJoCo default):** The parser currently hardcodes
  degree-to-radian conversion for `euler` attributes on bodies (line 957) and
  geoms (line 1305) in `model_builder.rs`. However, joint `range`, joint `ref`,
  and the angle component of `axisangle` receive **no conversion** — these values
  are stored as raw degrees and interpreted as radians at runtime. This silently
  corrupts joint limits for every model that does not explicitly set
  `angle="radian"`.
- **`eulerseq`:** Hardcoded to intrinsic XYZ (`euler_xyz_to_quat`). Models using
  non-default Euler sequences (e.g., `"ZYX"`, `"XYZ"` extrinsic) get wrong
  orientations.
- **`meshdir` / `texturedir` / `assetdir`:** Not parsed. Models that specify asset
  directories fail to load mesh files.
- **`autolimits`:** MuJoCo 3.0+ defaults to `autolimits="true"`, meaning `limited`
  is inferred from the presence of `range`. Our parser requires explicit `limited`
  attributes.
- **`inertiafromgeom`:** The `compute_inertia_from_geoms()` function exists but is
  not gated by this attribute — it runs unconditionally.
- **`boundmass` / `boundinertia` / `settotalmass` / `balanceinertia`:** Not
  implemented. Models relying on these produce incorrect mass properties.

These features are tightly coupled: `<include>` needs `<compiler>` path attributes,
and both are prerequisites for loading production MJCF models correctly.

#### Objective

Implement `<include>` element resolution and `<compiler>` element parsing with
MuJoCo-conformant semantics. Fix the existing silent correctness bugs in angle
unit handling.

#### Specification

##### Part A — `<compiler>` element

Add a `MjcfCompiler` struct to `types.rs` and parse it as a child of `<mujoco>`.
The struct is stored on `MjcfModel` and threaded into `ModelBuilder` to control
compilation behavior. All attributes use MuJoCo defaults.

**A1. `angle` — Angular unit specification** (Critical correctness fix)

- Type: `enum AngleUnit { Degree, Radian }`, default: `Degree`.
- During model building, when `angle == Degree`, convert **all** angle-valued
  quantities from degrees to radians:
  - Body `euler` angles (currently converted — keep working)
  - Body `axisangle` angle component (4th element only — axis stays untouched).
    **Additional bug:** body `axisangle` is parsed into `MjcfBody.axisangle` but
    `model_builder.rs:956-961` never checks it — the code only checks `euler` then
    falls back to `quat`, silently ignoring `axisangle`. Fix: add `axisangle`
    handling to the body orientation resolution chain (euler > axisangle > quat),
    then apply degree-to-radian conversion to the angle component.
  - Geom `euler` angles (currently converted — keep working). Note: `MjcfGeom`
    does not currently have an `axisangle` field — only `euler` and `quat`.
  - Joint `range` for hinge joints (currently **not** converted — **bug fix**)
  - Joint `ref` for hinge joints (currently **not** converted — **bug fix**)
  - Joint `springref` for hinge joints
- Note: `MjcfSite` currently stores only `quat` (no `euler` or `axisangle`).
  Angle conversion for sites is not needed until those orientation alternatives
  are added to the site type.
- When `angle == Radian`, **no conversion** is applied (values are already in
  radians). The current hardcoded `euler_deg * (PI / 180.0)` in `model_builder.rs`
  must be made conditional on this setting.
- Ball joint `range` **is** angle-valued (limits the total rotation angle) and
  must be converted. Ball joint `ref` is not angle-valued (it's a quaternion).
- Free joint ranges/refs are **not** angle-valued and must not be converted.
- Slide joint ranges/refs are **not** angle-valued and must not be converted.

**A2. `eulerseq` — Euler angle rotation sequence**

- Type: `String`, default: `"xyz"`, exactly 3 characters from `{x,y,z,X,Y,Z}`.
- Lowercase = intrinsic (body-fixed axes), uppercase = extrinsic (space-fixed axes).
- Replace the hardcoded `euler_xyz_to_quat()` with a generalized
  `euler_seq_to_quat(euler_rad: Vector3<f64>, seq: &str)` function that mirrors
  MuJoCo's `mju_euler2Quat` exactly:
  - Initialize an accumulator quaternion `q = identity`.
  - Iterate over each character `seq[i]` with corresponding angle `euler[i]`:
    - Build a single-axis rotation quaternion `R_i` for the axis (x/y/z) and angle.
    - If `seq[i]` is **lowercase** (intrinsic): **post-multiply** → `q = q * R_i`.
    - If `seq[i]` is **uppercase** (extrinsic): **pre-multiply** → `q = R_i * q`.
  - Return the accumulated `q`.
- This per-character rule handles all cases uniformly:
  - All-lowercase `"xyz"` → `Rx * Ry * Rz` (intrinsic XYZ, current behavior).
  - All-uppercase `"XYZ"` → `Rz * Ry * Rx` (extrinsic XYZ = intrinsic ZYX).
  - Mixed case `"XYz"` → `Ry * Rx * Rz` (extrinsic X, extrinsic Y, intrinsic z).
- Validation: reject sequences that are not exactly 3 characters or contain
  characters outside `{x,y,z,X,Y,Z}`. Return `MjcfError::InvalidAttribute`.

**A3. `meshdir` / `texturedir` / `assetdir` — Asset path resolution**

- Type: `Option<String>` for each.
- `meshdir` applies to both mesh files and height field (hfield) files.
- Path resolution algorithm (matches MuJoCo exactly):
  1. If the asset filename is an absolute path → use it directly.
  2. If the type-specific dir (`meshdir` for meshes/hfields, `texturedir` for
     textures) is set:
     - If that dir is absolute → `dir / filename`.
     - Else → `model_file_dir / dir / filename`.
  3. Else if `assetdir` is set:
     - Same logic as above but using `assetdir`.
  4. Else → `model_file_dir / filename` (current behavior, preserved).
- `meshdir` / `texturedir` take precedence over `assetdir` for their respective
  file types.
- Refactor `resolve_mesh_path()` into a generalized `resolve_asset_path()` that
  accepts a `CompilerPaths` context and an `AssetKind` enum (`Mesh`, `Texture`)
  instead of a bare `Option<&Path>`.

**A4. `autolimits` — Automatic limit inference**

- Type: `bool`, default: `true` (MuJoCo 3.0+ default).
- Applied during model building, **after** default class resolution. The inference
  operates on the fully-resolved element (element attributes + inherited defaults):
  - Joint: if the resolved joint has no explicit `limited` attribute (neither on
    the element nor from defaults), infer `limited = range.is_some()`.
  - Tendon: same logic for tendon `limited` / `range`.
  - Actuator: infer `ctrllimited` from `ctrlrange`, `forcelimited` from
    `forcerange`, `actlimited` from `actrange` — same precedence rules.
  - An explicitly specified `limited` attribute (on the element or from defaults)
    always takes precedence over inference.
- When `false`, both `limited` and `range` must be explicitly specified (current
  behavior).

**A5. `inertiafromgeom` — Inertia computation control**

- Type: `enum InertiaFromGeom { False, True, Auto }`, default: `Auto`.
- `False`: never compute inertia from geoms. Bodies without explicit `<inertial>`
  get zero mass/inertia (MuJoCo behavior).
- `True`: always compute inertia from geoms, **overriding** any explicit
  `<inertial>` element.
- `Auto`: compute from geoms only when the body lacks an explicit `<inertial>`
  child (current unconditional behavior becomes `Auto`-only).
- **Existing quirk to fix:** `compute_inertia_from_geoms()` currently returns
  mass=0.001 for bodies with no geoms (line 3512). MuJoCo gives zero mass in
  this case. The 0.001 fallback should be removed; `boundmass` (A6) is the
  correct mechanism for ensuring minimum mass.

**A6. `boundmass` / `boundinertia` — Minimum mass/inertia clamping**

- Type: `f64`, default: `0.0` (disabled).
- `boundmass`: after inertia computation, clamp every non-world body mass to
  `max(body_mass, boundmass)`.
- `boundinertia`: clamp every non-world body diagonal inertia component to
  `max(component, boundinertia)`.
- Applied after `inertiafromgeom`, before `balanceinertia`.

**A7. `balanceinertia` — Inertia matrix correction**

- Type: `bool`, default: `false`.
- When `true`, for each body: check the triangle inequality on diagonal inertia
  (A + B ≥ C for all permutations). If violated, set all three diagonal elements
  to their mean `(A + B + C) / 3`.
- Applied after `boundmass` / `boundinertia`, before `settotalmass`.

**A8. `settotalmass` — Total mass rescaling**

- Type: `f64`, default: `-1.0` (disabled).
- When positive: compute the total mass of all bodies (excluding world body),
  then scale every body's mass and inertia by `settotalmass / total_mass`.
- Applied **last**, after all other mass-affecting operations.

**A9. `strippath` — Path stripping**

- Type: `bool`, default: `false`.
- When `true`, strip directory components from all asset file references, keeping
  only the base filename. Applied before path resolution.

**A10. `discardvisual` — Visual element removal**

- Type: `bool`, default: `false`.
- When `true`, during model building discard:
  - All geoms with `contype=0` and `conaffinity=0` (visual-only) unless
    referenced by sensors/actuators.
  - All unused meshes after geom removal.
- Reduces compiled model size without affecting dynamics.

**A11. `fusestatic` — Static body fusion**

- Type: `bool`, default: `false`.
- When `true`, fuse bodies that have no joints (static bodies) into their parent.
  Transfer all geoms, sites, and child elements to the parent body. Do not fuse
  bodies referenced by equality constraints, sensors, or actuators.

**A12. `coordinate` — Coordinate frame (deprecated)**

- Type: `enum Coordinate { Local, Global }`, default: `Local`.
- `coordinate="global"` was **removed in MuJoCo 2.3.4**. Our parser should:
  - Accept `"local"` silently (no-op, matches current behavior).
  - Reject `"global"` with `MjcfError::Unsupported("coordinate='global' was
    removed in MuJoCo 2.3.4")`.
- This prevents silently misinterpreting legacy models.

**Compilation order for mass-related operations:**
1. `inertiafromgeom` (A5)
2. `balanceinertia` (A7)
3. `boundmass` / `boundinertia` (A6)
4. `settotalmass` (A8)

**Deferred attributes** (parse and store, do not implement behavior):
- `fitaabb`, `usethread`, `alignfree`, `saveinertial`, `inertiagrouprange`,
  `<lengthrange>` child element. These have no effect on simulation correctness
  and can be implemented later. Store them on `MjcfCompiler` so they are preserved
  for round-tripping.

##### Part B — `<include>` file support

**B1. Pre-parse XML expansion**

Implement `<include>` resolution as a **pre-parse XML rewriting step** before the
MJCF parser sees the document. This operates on the raw XML string/DOM, not on
parsed MJCF types.

Algorithm:
1. Parse the main MJCF file into an XML DOM (using `quick-xml`'s tree API, or a
   simple recursive descent over events that buffers elements).
2. Walk the DOM looking for `<include file="..."/>` elements at any nesting level.
3. For each `<include>`:
   a. Resolve `file` path relative to the **main model file directory** (not the
      including file's directory — this matches MuJoCo semantics).
   b. Read and parse the referenced file as XML.
   c. Verify the file has exactly one root element (the wrapper).
   d. Extract all children of that root element.
   e. Replace the `<include>` element with those children in the parent DOM.
4. After all includes are expanded, serialize the composite DOM back to an XML
   string and pass it to the existing `parse_mjcf_str()`.

**B2. Uniqueness constraint**

Each file may be included **at most once** across the entire include tree. Track
the set of resolved canonical paths. If a file appears a second time, return
`MjcfError::CircularInclude` (reuse the existing error variant, or add
`DuplicateInclude`).

**B3. Nested includes**

After inserting children from an included file, re-scan those children for further
`<include>` elements. Process recursively until no `<include>` elements remain.
The uniqueness constraint (B2) prevents infinite recursion.

**B4. Error handling**

- File not found: `MjcfError::IncludeNotSupported` → rename to
  `MjcfError::IncludeError(String)` with descriptive message.
- Invalid XML in included file: propagate `MjcfError::XmlParse`.
- Empty included file (no children after removing wrapper): error.
- Include path resolution failure: clear error with both the `file` attribute
  value and the resolved path attempted.

**B5. Integration with `<compiler>` path attributes**

- `meshdir` / `texturedir` / `assetdir` do **NOT** affect include file path
  resolution (those only affect asset paths). Include paths always resolve
  relative to the main model file directory.
- If an included file contains a `<compiler>` element, its settings are merged
  (last-writer-wins, since compiler settings are global). This happens naturally
  because include expansion occurs before parsing.

**B6. Placement flexibility**

`<include>` can appear inside any MJCF section — `<worldbody>`, `<asset>`,
`<default>`, `<actuator>`, `<tendon>`, `<sensor>`, `<contact>`, `<equality>`, or
at the top level as a direct child of `<mujoco>`. After expansion, repeated
sections (e.g., two `<worldbody>` elements) are valid and should be merged by the
parser.

##### Part C — Parser changes for section merging

After include expansion, the composite DOM may contain duplicate top-level
sections (e.g., two `<actuator>` blocks). Update `parse_mujoco()` to **merge**
rather than replace. Currently `defaults` already extends (line 76) but all
other sections overwrite. Fix:
- `actuators`: extend vec (currently overwrites)
- `sensors`: extend vec (currently overwrites)
- `tendons`: extend vec (currently overwrites)
- `defaults`: extend vec (already correct)
- `meshes` / `hfields`: extend vecs (currently overwrites)
- `worldbody`: merge body trees — append children of the second `<worldbody>` to
  the first (currently overwrites)
- `equality`: merge constraint lists (currently overwrites)
- `contact`: merge pair/exclude lists (currently overwrites)
- `keyframes`: extend vec (currently overwrites)
- `skins`: extend vec (currently overwrites)
- `option`: last-writer-wins (current behavior is correct — global setting)
- `compiler`: last-writer-wins (global setting, matches MuJoCo)

#### Acceptance Criteria

**Compiler correctness:**
1. `<compiler angle="degree"/>` (default): hinge joint `range="-90 90"` produces
   internal range `(-π/2, π/2)`. Ball joint `range="0 60"` produces `(0, π/3)`.
   Euler `"90 0 0"` produces a 90° rotation. `axisangle="0 0 1 45"` produces a
   45° rotation.
2. `<compiler angle="radian"/>`: values pass through unchanged. Joint
   `range="-1.57 1.57"` stays as `(-1.57, 1.57)`.
3. Slide and free joint ranges are never angle-converted regardless of `angle`
   setting.
4. `<compiler eulerseq="ZYX"/>` (extrinsic) produces the same quaternion as
   MuJoCo for the same Euler values.
5. `<compiler eulerseq="xyz"/>` (default, intrinsic) matches current behavior
   (regression).
6. `<compiler meshdir="assets/"/>` resolves `<mesh file="box.stl"/>` to
   `<model_dir>/assets/box.stl`.
7. `<compiler assetdir="data/"/>` with `<compiler meshdir="meshes/"/>` resolves
   meshes to `<model_dir>/meshes/` and textures to `<model_dir>/data/`.
8. `<compiler autolimits="true"/>`: joint with `range` but no `limited` attribute
   is treated as limited. Joint with no `range` is unlimited.
9. `<compiler inertiafromgeom="false"/>`: body without `<inertial>` gets zero mass.
10. `<compiler inertiafromgeom="auto"/>` (default): body with `<inertial>` uses
    explicit values; body without uses computed values.
11. `<compiler settotalmass="10"/>`: total model mass equals 10 after compilation.
12. `<compiler balanceinertia="true"/>`: body with diagonal inertia `(1, 1, 10)`
    → `(4, 4, 4)`.
13. `<compiler boundmass="0.01"/>`: body with mass 0.001 gets clamped to 0.01.
14. `<compiler coordinate="global"/>` produces `MjcfError::Unsupported`.

**Include file support:**
15. A model with `<include file="bodies.xml"/>` where `bodies.xml` contains body
    definitions loads successfully and produces the same model as inline XML.
16. Nested includes (file A includes file B which includes file C) work correctly.
17. Including the same file twice produces `MjcfError::DuplicateInclude`.
18. `<include>` inside `<asset>`, `<worldbody>`, `<actuator>`, `<default>` all work.
19. Included file with invalid XML produces clear `MjcfError::XmlParse`.
20. Include path is relative to main model file, not the including file.

**Regression safety:**
21. Models without `<include>` or `<compiler>` that use only `euler` and `quat`
    orientations (no `axisangle`) and have no hinge/ball joint
    `range`/`ref`/`springref` produce identical results to current behavior.
    Models with hinge or ball joint limits will now have those limits correctly
    converted from degrees — this is an intentional correctness fix, not a
    regression.
22. The URDF converter emits `<compiler angle="radian" eulerseq="xyz"
    fusestatic="true" discardvisual="true" strippath="true"/>` — matching
    MuJoCo's URDF-specific defaults. URDF-loaded models continue to work
    correctly.
23. All existing tests pass (angle conversion changes may require updating test
    MJCF strings that have hinge joint ranges specified in radians — these should
    add explicit `<compiler angle="radian"/>`).

#### Implementation Notes

**API surface for `<include>`:** Only `load_model_from_file()` can support
`<include>` because include resolution requires a file path for relative path
resolution and file I/O. The string-based APIs (`parse_mjcf_str()`, `load_model()`)
have no file context — `<include>` elements encountered via these APIs should
produce `MjcfError::IncludeError("includes require file-based loading; use
load_model_from_file()")`. The `<compiler>` element works in all API paths since
it requires no file I/O.

**Existing bug to fix:** The current code hardcodes `euler_deg * (PI / 180.0)` at
`model_builder.rs:957` and `:1305`. This is correct only when `angle="degree"`.
After this change, the conversion must be conditional: apply the factor only when
`compiler.angle == Degree`. When `angle == Radian`, euler values pass through
unchanged.

**Breaking change:** Adding default `angle="degree"` will cause joint range values
that were previously stored as raw degrees (but interpreted as radians) to now be
correctly converted. This means simulations that previously had "working" joint
limits were actually using wrong limits — the behavior change is a **correctness
fix**, not a regression. Existing test MJCF strings that specify hinge joint ranges
in radians should add `<compiler angle="radian"/>` to preserve their intent.

**URDF converter defaults:** MuJoCo uses different compiler defaults for
URDF-sourced models: `fusestatic="true"`, `discardvisual="true"`,
`strippath="true"`. The URDF converter at `converter.rs:129` currently emits
only `angle="radian" eulerseq="xyz"`. Once `fusestatic`/`discardvisual`/
`strippath` become functional, URDF models would silently diverge from MuJoCo
behavior without these defaults. Fix: update the converter's `<compiler>` line
to include all URDF-appropriate defaults.

**Test migration strategy:** The existing test suite embeds MJCF strings via
`load_model()` with joint ranges in radians (e.g., `range="-1.57 1.57"`). Since
`angle="degree"` is now the default, these tests must add
`<compiler angle="radian"/>` to their MJCF strings, or convert their range values
to degrees. The former is preferred — it makes the intent explicit and matches
MuJoCo convention.

#### Files
- `sim/L0/mjcf/src/types.rs` — `MjcfCompiler` struct, `AngleUnit` enum,
  `InertiaFromGeom` enum; add `compiler: MjcfCompiler` to `MjcfModel`
- `sim/L0/mjcf/src/parser.rs` — `parse_compiler()` function; add `b"compiler"`
  case to `parse_mujoco` match; section merging for duplicate elements
- `sim/L0/mjcf/src/include.rs` — **new file**: XML pre-processing for `<include>`
  expansion, path resolution, uniqueness tracking
- `sim/L0/mjcf/src/model_builder.rs` — `CompilerPaths` context replacing bare
  `base_path`; conditional angle conversion; `euler_seq_to_quat()` replacing
  `euler_xyz_to_quat()`; `autolimits` inference; `inertiafromgeom` gating;
  mass post-processing pipeline (`balanceinertia` → `boundmass`/`boundinertia` →
  `settotalmass`); `discardvisual` filtering; `fusestatic` body merging
- `sim/L0/mjcf/src/error.rs` — rename `IncludeNotSupported` to `IncludeError`;
  add `DuplicateInclude` variant
- `sim/L0/mjcf/src/lib.rs` — wire include expansion into `load_model_from_file`;
  update public API and doc comments; remove "Include files not supported"
  limitation note
- `sim/L0/urdf/src/converter.rs` — update `<compiler>` line to include
  URDF-specific defaults (`fusestatic`, `discardvisual`, `strippath`)

---

### 19. MuJoCo Conformance Test Suite
**Status:** Not started | **Effort:** XL | **Prerequisites:** None (benefits from #18)

#### Current State

Model loading is tested (MuJoCo Menagerie + DeepMind Control Suite all load
successfully). Per-feature correctness relies on ad-hoc verification: analytical
ground truth for FK/CRBA (9 tests in `validation.rs`), finite-difference
cross-checks for derivatives (30+ tests in `derivatives.rs`), and hardcoded
MuJoCo 3.4.0 reference values for spatial tendon lengths/Jacobians (18 tests in
`spatial_tendons.rs`). These are valuable but **unsystematic** — they cover
whatever the implementor tested at the time, leaving gaps between features and
no trajectory-level validation.

**What's missing:**
- No per-pipeline-stage comparison against MuJoCo (does `mj_crba` produce the
  same mass matrix? does `mj_rne` produce the same bias forces?)
- No self-consistency tests (forward/inverse equivalence, island/monolithic
  equivalence, sparse/dense equivalence)
- No multi-step trajectory comparison (error accumulation, divergence detection)
- No determinism verification (same inputs → identical outputs across runs)
- No property-based tests (energy conservation ordering across integrators)

This is foundational infrastructure: every subsequent physics item (#20–#39)
should be verified against MuJoCo ground truth via this harness. MuJoCo's own
test suite uses four complementary strategies — **unit tests** (analytical ground
truth), **self-consistency tests** (two code paths, same answer),
**cross-implementation tests** (C engine as oracle for MJX), and **property
tests** (conservation laws, ordering invariants). We adopt the same taxonomy.

#### Objective

Build a four-layer conformance test suite that:
1. Validates each pipeline stage in isolation against analytical or MuJoCo ground truth
2. Verifies internal self-consistency (forward/inverse, island/monolithic)
3. Compares multi-step trajectories against MuJoCo reference data
4. Enforces invariant properties (determinism, energy ordering, conservation)

The suite must be **debuggable** — when a trajectory diverges at step 12, you
can immediately identify which pipeline stage first diverged and on which DOF.

#### Specification

The spec is organized into four layers (A–D), each independently implementable.
Layer A requires no MuJoCo dependency. Layers B and C use pre-generated reference
data (checked into the repo). Layer D requires no MuJoCo dependency.

##### Layer A — Self-Consistency Tests (no MuJoCo dependency)

Self-consistency tests verify that different code paths in CortenForge produce
equivalent results. These are the highest-value tests because they catch bugs
without external dependencies and never need regeneration.

**Layer A model set.** These models are shared across A1–A5 (not all tests
use all models — each test specifies which subset applies):

| Model | Description | nv | Features |
|-------|-------------|-----|---------|
| `a_pendulum` | Single hinge, no damping | 1 | Minimal, no contacts |
| `a_double_pendulum` | Two-link hinge chain | 2 | Off-diagonal M, Coriolis |
| `a_spring_damper` | Hinge with stiffness + damping | 1 | Passive forces |
| `a_ball_free` | Ball joint + free joint body | 9 | Quaternion, nq≠nv |
| `a_three_link_arm` | 3-hinge chain with motor | 3 | Actuation, FK chain |
| `a_sphere_on_plane` | Sphere on ground plane | 6 | Single contact |
| `a_two_boxes` | Two boxes on plane (separate trees) | 12 | Multi-tree, islands |
| `a_ant` | 4-leg ant model, ctrl=0 | 8 | Multiple contacts, islands, 4+ geoms |
| `a_tendon_arm` | 3-link arm with tendon actuator | 3 | Tendon J^T mapping |
| `a_humanoid` | Full humanoid, ctrl=0 | 27 | Integration: everything |

**A1. Forward/inverse equivalence**

After `forward()`, verify the force balance identity. This is CortenForge's
force-balance verification, inspired by (but stronger than) MuJoCo's
`mj_compareFwdInv()` diagnostic, which compares forward vs inverse constraint
forces rather than the full residual.

The identity depends on which code path computed `qacc`:

- **Euler/RK4 (explicit path via `mj_fwd_acceleration_explicit`):**
  `M * qacc == qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias`
  Here `qfrc_passive` includes friction loss (`qfrc_frictionloss`), and the
  explicit solver uses it directly.

- **Newton solver path (lines 16679–16693):**
  `M * qacc == qfrc_applied + qfrc_actuator + (qfrc_passive - qfrc_frictionloss) + qfrc_constraint - qfrc_bias`
  Newton separates friction loss from smooth forces because it handles dry
  friction within the constraint system. `qfrc_frictionloss` is a `Data` field
  (`DVector<f64>`, populated by `mj_fwd_passive`) containing the friction-loss
  component that is also included in `qfrc_passive`.

- **Implicit integrators (ImplicitFast/Implicit/ImplicitSpringDamper):**
  These solve `(M - h·D) * qacc = rhs` with a modified mass matrix, so
  the `M * qacc == f_total` identity does **not** hold. A1 is not applicable
  to implicit integrator models. (Implicit integrator correctness is tested
  by B10 single-step comparison and the existing `implicit_integration.rs`
  AC-9/AC-10 analytical-vs-FD tests.)

The implementation must branch on `data.newton_solved` (a `bool` field on
`Data`, set to `true` when the Newton solver computes `qacc`, `false`
otherwise) because the two paths assemble the RHS differently:

```rust
fn assert_fwd_inv_consistent(model: &Model, data: &Data, eps: f64) {
    // Build the RHS that was used to solve M * qacc = rhs.
    // Newton subtracts qfrc_frictionloss from qfrc_passive (it handles
    // dry friction in the constraint system). The explicit path uses
    // qfrc_passive directly (which already includes frictionloss).
    let mut rhs = &data.qfrc_applied + &data.qfrc_actuator
        + &data.qfrc_passive
        + &data.qfrc_constraint - &data.qfrc_bias;
    if data.newton_solved {
        rhs -= &data.qfrc_frictionloss;
    }
    // M * qacc via dense multiply (not via solve — we want the residual)
    let m_qacc = &data.qM * &data.qacc;
    let residual = &m_qacc - &rhs;
    let norm = residual.amax() / data.qacc.amax().max(1e-10);
    assert!(norm < eps, "Force balance residual {norm:.2e} exceeds {eps:.2e}");
}
```

- Tolerance: `1e-10` (algebraic identity — `qacc` is computed by solving
  `M * qacc = f_total` via `mj_solve_sparse`, so the residual is factorization
  round-trip error only).
- Test at: `qpos0`, 3 random configurations (seeded deterministically with
  `qpos[i] = 0.1 * sin(i * seed)` for hinge/slide DOFs; quaternion DOFs
  use `quat = normalize([1, 0.1*sin(seed), 0.1*cos(seed), 0.05*sin(2*seed)])`
  to stay near identity while avoiding axis-aligned degeneracies), and
  3 mid-simulation states (step 10, 50, 100 of a default run with Euler
  integrator).
- Models: all Layer A models **that use Euler or RK4 integrators**. Models
  with implicit integrators are excluded (see above). All Layer A model
  MJCF defaults to Euler, so this covers the full set unless a model
  explicitly sets `integrator="implicit*"`.
- Implementation: single function `assert_fwd_inv_consistent(model, data, eps)`
  as shown above, branching on `data.newton_solved` to determine whether
  `qfrc_frictionloss` is subtracted from the RHS.

**A2. Island/monolithic solver equivalence**

For models with multiple kinematic trees and contacts, compare:
- `mj_fwd_constraint()` (monolithic solve via `DISABLE_ISLAND`, `nisland = 0`)
- `mj_fwd_constraint_islands()` (per-island solve)

Both must produce identical `qfrc_constraint` and `efc_force`.
(Note: `Data::efc_lambda` in CortenForge is the warmstart cache, not the
solved forces. The solved constraint forces are in `Data::efc_force`.)

- Tolerance: `1e-8` for PGS (iteration order may differ within islands vs.
  global), `1e-6` for CG. Newton is excluded — it always dispatches to the
  monolithic solver (`mj_fwd_constraint`) regardless of islands, so the
  comparison is vacuous (identical code path, `0` difference).
- Models: `a_two_boxes` (multi-tree), `a_ant` (single tree, multiple islands).
- Implementation: toggle `model.disableflags |= DISABLE_ISLAND` to force
  monolithic solve (sets `nisland = 0`, which triggers the global fallback
  in `mj_fwd_constraint_islands`). No new parameter or `#[cfg(test)]` gate
  needed — the flag is already public on `Model`.

**A3. Determinism (bit-identical replay)**

Step a model 100 times. Reset to initial state. Step 100 times again. Assert
bit-identical `qpos`, `qvel`, `qacc`, `qfrc_constraint`, `sensordata` at
every step. This catches uninitialized memory, HashMap iteration order leaks,
and floating-point non-determinism.

- **Reset mechanism**: use `model.make_data()` to create a fresh `Data`, not
  `data.reset(&model)`. The `reset()` method does not clear the `efc_lambda`
  warmstart cache (`HashMap<WarmstartKey, Vec<f64>>`), so stale entries from
  run 1 would leak into run 2, causing the second run to diverge from the
  first at the first contact step. `make_data()` creates a fresh empty
  HashMap and guarantees identical initial state.
- Tolerance: `0` (exact bit equality via `assert_eq!` on raw f64 bits, or
  `f64::to_bits()` comparison). Where HashMap iteration order causes
  unavoidable differences (warmstart cache), compare the *physics outputs*
  bit-identically, not the cache internals.
- Repeat with warmstart disabled to isolate warmstart-related non-determinism.
  The MJCF `<flag warmstart="disable"/>` is parsed but not currently wired
  into the solver. To disable warmstart in tests, manually clear the cache
  before each step: `data.efc_lambda.clear(); data.qacc_warmstart.fill(0.0);`.
- Models: `a_double_pendulum`, `a_sphere_on_plane`, `a_ant`, `a_humanoid`
  (from Layer A model set — covers no-contact, single-contact,
  multi-contact, and high-DOF systems).

**A4. Integrator energy ordering**

For a conservative system (spring, no damping, no contacts), verify that
absolute energy drift after N steps satisfies:
`|ΔE_RK4| < |ΔE_Implicit| < |ΔE_Euler|`.

This is a property test — no absolute tolerance, just ordering. MuJoCo's
`engine_forward_test.cc` (`EnergyConservation`) uses this exact pattern.

- Models: a variant of `a_spring_damper` with `damping="0"` in its MJCF
  (pure conservative spring), and `a_double_pendulum` (conservative two-link,
  no damping or contacts by default).
- Simulation: 1000 steps at `dt = 0.001`.
- Assert: strict inequality `|ΔE_RK4| < |ΔE_Implicit| < |ΔE_Euler|`.
- Note: `ImplicitFast` is not included in the ordering assertion because
  its relationship to `Implicit` depends on whether Coriolis terms
  contribute significant energy error for the specific model. Instead,
  assert `|ΔE_ImplicitFast| < |ΔE_Euler|` as a separate weaker check.

**A5. Sparse/dense mass matrix equivalence**

Compare `qM` (dense, from CRBA) against reconstruction from sparse `qLD`
factorization: `L^T * D * L` must equal `qM`.

- Tolerance: `1e-12` (factorization round-trip).
- Models: all Layer A models.
- Implementation: reconstruct dense M from `qLD_data`, `qLD_rowadr`,
  `qLD_rownnz`, `qLD_colind`, `qLD_diag_inv`; compare element-wise.

**A6. Sleep bit-identity**

For each pipeline function, verify that enabling sleep with all bodies awake
produces bit-identical results to the non-sleep code path. This already has
93 tests in `sleeping.rs` — the conformance suite should reference/extend
these rather than duplicate.

- Pattern: run forward() with `ENABLE_SLEEP` and all bodies awake, compare
  against forward() without `ENABLE_SLEEP`. Assert bit-identity for all
  `Data` fields.
- This test already exists (sleeping.rs Phase C tests). Reference it in the
  conformance index and extend to cover any new pipeline stages.

##### Layer B — Per-Stage Reference Tests (MuJoCo reference data, pre-generated)

Each test calls `forward()` (which runs all pipeline stages), then compares
a specific `Data` field — the one written by the target pipeline stage —
against a hardcoded MuJoCo 3.4.0 reference value. Although `forward()` runs
all stages, each Layer B test checks **only the output of its target stage**.
If B3 (RNE → `qfrc_bias`) fails but B1 (FK → `xpos`) passes, the bug is in
RNE, not FK. This is the **diagnostic backbone** — when a trajectory test
(Layer C) fails, you drill down to the per-stage test that fails.

All reference values are hardcoded as Rust constants with a `// MuJoCo 3.4.0`
annotation, matching the established pattern in `spatial_tendons.rs`. No
external reference files — constants are generated once by the Python reference
script and pasted into the test source.

Layer B models are **separate** from the Layer A model set. They are purpose-built
for per-stage testing and may overlap in concept (e.g., both layers have a double
pendulum) but are independent MJCF definitions with different names.

**B1. Forward kinematics (`mj_fwd_position`)**

For each test model at `qpos0` and 3 non-trivial configurations:
- Record `xpos[i]` and `xquat[i]` for all bodies from MuJoCo 3.4.0.
- Assert `xpos` and `xquat` match within tolerance.
- Tolerance: `1e-10` (identical algorithm — recursive FK from parent).

Models: `single_hinge`, `double_pendulum`, `ball_joint_chain`,
`free_body`, `humanoid_qpos0`.

Existing coverage: `validation.rs` has 9 FK tests with analytical ground truth.
Layer B adds MuJoCo-reference FK tests for complex models where analytical
solutions are impractical.

**B2. Mass matrix (`mj_crba`)**

For each test model at `qpos0` and 2 non-trivial configurations:
- Record the full `qM` matrix (dense, `nv × nv`) from MuJoCo 3.4.0.
- Compare element-wise.
- Tolerance: `1e-10` (identical CRBA algorithm; only difference is
  accumulation order in the backward pass, which for tree-order traversal
  is deterministic).

Also verify: `qM` is symmetric (`|M[i,j] - M[j,i]| < 1e-14`), positive
definite (all eigenvalues > 0), and armature appears on the diagonal
(`qM[dof,dof] >= armature[dof]`).

Models: `double_pendulum`, `three_link_arm`, `free_body`,
`ant_standing` (8-DOF, multi-body chain).

Existing coverage: `validation.rs` has 4 CRBA tests. Layer B extends with
larger models.

**B3. Bias forces (`mj_rne`)**

For each test model at 3 configurations with nonzero velocity:
- Record `qfrc_bias[nv]` from MuJoCo 3.4.0.
- Compare element-wise.
- Tolerance: `1e-10` (identical RNE algorithm with identical traversal order).

Verify: at zero velocity with gravity, `qfrc_bias` equals the gravity
torque from `subtree_com × gravity × subtree_mass` projected through joint
axes (already tested analytically in `validation.rs`; Layer B adds MuJoCo
cross-check).

Models: `double_pendulum`, `three_link_arm`, `humanoid_qpos0`.

**B4. Passive forces (`mj_fwd_passive`)**

For each test model at 3 configurations with nonzero velocity:
- Record `qfrc_passive[nv]` from MuJoCo 3.4.0.
- Compare element-wise.
- Tolerance: `1e-10` for spring and damper forces (`-k*(q-qref)`, `-b*qvel`)
  — identical formulas.

**Frictionloss divergence.** ⚠️ **Migration tracked as #19b/#19c in
[future_work_6_precursor_1.md](./future_work_6_precursor_1.md).** CortenForge
computes frictionloss as a tanh passive force; MuJoCo uses Huber constraint
rows. This divergence will be resolved before #19 conformance tests are written.
B4 frictionloss tests should be marked `#[ignore]` until #19b/#19c are complete.

Spring and damper passive forces match MuJoCo exactly. Also verify: at
`qpos == springref` and `qvel == 0`, `qfrc_passive == 0` (equilibrium).

Models: `spring_damper_hinge`, `tendon_passive_forces`.

Existing coverage: `passive_forces.rs` has 15+ tests. Layer B adds MuJoCo
cross-check for tendon passive forces (which go through `J^T` mapping and
are more error-prone).

**B5. Contact detection (`mj_collision`)**

For each contact scenario at a fixed configuration:
- Record contact count, contact positions, normals, and depths from MuJoCo 3.4.0.
- Compare: count must match exactly; positions/normals/depths within tolerance.
- Tolerance: `1e-6` for positions/normals (floating-point geometry), `1e-8`
  for depths (direct distance computation).

Special cases to cover:
- `contype`/`conaffinity` filtering (contact exists vs. suppressed)
- `<contact><pair>` explicit pairs bypass filters
- `<contact><exclude>` suppresses automatic pairs
- Parent-child default filtering
- Margin activation (`dist < margin` generates contact) — requires #20;
  skip this sub-test until margin is implemented
- Contact sorting order (by geom pair ID then by depth)

Models: `sphere_on_plane`, `box_on_plane`, `capsule_capsule`,
`eight_spheres_on_plane` (MuJoCo's own test: must produce exactly 8 contacts),
`pair_override`, `exclude_test`.

**B6. Constraint forces (`mj_fwd_constraint`)**

For each contact scenario:
- Record `qfrc_constraint[nv]` and `efc_force` from MuJoCo 3.4.0.
- Tolerance depends on solver:
  - Newton: `1e-8` (quadratic convergence, same formulation)
  - CG: `1e-6` (same PGD algorithm, minor step-size path differences)
  - PGS: `1e-4` (iteration order sensitive; may need relaxed tolerance
    or iteration-count matching)
- Also compare: joint limit forces, tendon limit forces, equality constraint
  forces (these use penalty, not iterative solve — tolerance `1e-10`).

Models: `box_on_plane_newton`, `sphere_stack_pgs`, `joint_limits_hinge`,
`tendon_limits`, `weld_constraint`, `connect_constraint`.

**B7. Actuator forces (`mj_fwd_actuation`)**

For each actuator scenario:
- Record `actuator_force[nu]`, `qfrc_actuator[nv]`, `act_dot[na]` from
  MuJoCo 3.4.0.
- Tolerance: `1e-10` (identical gain/bias formulas and transmission
  Jacobians).

Scenarios:
- Motor (Fixed gain): `gain * ctrl`
- Position servo (Affine bias): `kp * (ctrl - qpos) - kv * qvel`
- Velocity servo: `kv * (ctrl - qvel)`
- Damper: `-kv * |qvel|`
- Muscle (FLV curves): gain/bias from normalized length/velocity
- Filter dynamics: `act_dot = (ctrl - act) / tau`
- FilterExact: same `act_dot`, different integration
- Integrator dynamics: `act_dot = ctrl`
- Site transmission: `mj_jac_site()` → `J^T * gear * force`
- Tendon transmission: Jacobian-mapped force

Models: `motor_hinge`, `position_servo`, `velocity_servo`,
`muscle_arm`, `filter_actuator`, `site_transmission_3link`,
`tendon_actuator`.

Existing coverage: `site_transmission.rs` has a `#[ignore]` placeholder
awaiting MuJoCo reference data. Layer B provides the data.

**B8. Sensor readings (`mj_sensor_pos` / `mj_sensor_vel` / `mj_sensor_acc`)**

For each sensor scenario:
- Record `sensordata[nsensordata]` from MuJoCo 3.4.0.
- Tolerance: `1e-8` for kinematics-derived sensors (JointPos, FramePos,
  etc.), `1e-6` for force-derived sensors (Touch, Force, Torque — depend
  on constraint solve).

Sensors to cover (grouped by derivation):
- **State-read**: JointPos, JointVel, BallQuat, BallAngVel, ActuatorPos,
  ActuatorVel, TendonPos, TendonVel — tolerance `1e-10` (direct state/
  tendon read).
- **FK-derived**: FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis,
  SubtreeCom — tolerance `1e-10`.
- **Velocity-derived**: FrameLinVel, FrameAngVel, SubtreeLinVel,
  SubtreeAngMom, Velocimeter, Gyro — tolerance `1e-8`.
- **Acceleration-derived**: FrameLinAcc, FrameAngAcc, Accelerometer —
  tolerance `1e-6` (depends on constraint forces).
- **Force-derived**: ActuatorFrc, JointLimitFrc, TendonLimitFrc,
  Force, Torque, Touch — tolerance `1e-6`.
- **Geometric**: Rangefinder, Magnetometer — tolerance `1e-6`.

Models: `sensor_suite_hinge` (one model with all 32 sensor types attached
to a simple kinematic chain — maximizes coverage per model).

**B9. Tendon kinematics (`mj_fwd_tendon`)**

Existing coverage in `spatial_tendons.rs` (18 tests) already follows the
Layer B pattern with MuJoCo 3.4.0 reference values. Extend with:
- Fixed tendon `ten_length` and `ten_J` at non-default configurations.
- `ten_velocity` at nonzero `qvel`.
- Tolerance: `1e-6` for lengths/Jacobians (established pattern).

**B10. Integration step**

For each integrator at a fixed state, take one `step()` and compare the
resulting `qpos`, `qvel`, `act`:
- Euler: tolerance `1e-10` (identical semi-implicit formula).
- RK4: tolerance `1e-10` (identical Butcher tableau, identical stage order).
- ImplicitFast: tolerance `1e-8` (same D assembly, Cholesky solve may have
  different pivoting behavior in edge cases).
- Implicit: tolerance `1e-8` (LU factorization order may differ slightly).
- ImplicitSpringDamper: tolerance `1e-10` (identical diagonal formula).

Models: `spring_pendulum` (stiff spring, tests implicit stability),
`damped_hinge` (tests velocity damping), `muscle_arm` (tests activation
integration).

##### Layer C — Trajectory Comparison Tests (multi-step, MuJoCo reference data)

Multi-step trajectory tests catch error accumulation and feedback loops that
single-step tests miss. A small per-step error in contact forces feeds into
the next step's positions, which changes collision geometry, which changes
contact forces — Layer C detects this drift.

**C1. Reference data format**

Reference data is stored as JSON files in `sim/L0/tests/conformance/references/`,
generated by `generate_references.py` and **checked into the repo**. Each file
contains:

```json
{
  "mujoco_version": "3.4.0",
  "model_xml": "<mujoco>...</mujoco>",
  "timestep": 0.002,
  "n_steps": 200,
  "ctrl_sequence": [[0.1, -0.2], [0.1, -0.2], ...],
  "steps": [
    {
      "step": 0,
      "qpos": [0.0, 0.1, ...],
      "qvel": [0.0, 0.0, ...],
      "qacc": [1.2, -0.3, ...],
      "xpos": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.5], ...],
      "xquat": [[1.0, 0.0, 0.0, 0.0], ...],
      "qfrc_bias": [-9.81, ...],
      "qfrc_passive": [0.0, ...],
      "qfrc_constraint": [0.0, ...],
      "qfrc_actuator": [0.1, ...],
      "sensordata": [0.0, ...],
      "ncon": 0,
      "energy": [4.905, 0.0]
    },
    ...
  ]
}
```

The `energy` array is `[energy_potential, energy_kinetic]`, matching MuJoCo's
`d.energy[0]` (potential) and `d.energy[1]` (kinetic).

**Post-step field semantics.** Both MuJoCo's `mj_step()` and CortenForge's
`step()` call `forward()` then `integrate()` with no second `forward()` pass.
After `step()` returns, `qpos` and `qvel` reflect the **post-integration**
state (time N+1), while `xpos`, `xquat`, `qfrc_bias`, `qfrc_passive`,
`qfrc_actuator`, `qfrc_constraint`, `qacc`, `sensordata`, and `energy` all
reflect the `forward()` pass computed from the **pre-integration** state
(time N). The Python reference generator records fields after `mj_step()`
which produces exactly this semantic, and the Rust harness reads fields after
`step()` which matches. All comparisons are therefore against the same
logical state.

The reference includes per-step intermediate pipeline outputs — FK (`xpos`,
`xquat`) and all force components (`qfrc_bias`, `qfrc_passive`,
`qfrc_constraint`, `qfrc_actuator`) — so that when the trajectory diverges,
the test harness can report *which pipeline stage* first diverged, not just
that `qpos` drifted. CRBA (`qM`) is tested per-configuration in Layer B
(B2) and is not included per-step in trajectories to avoid O(nv²·N) data
bloat for high-DOF models.

File size: a 200-step trajectory for a 30-DOF model with 10 sensors is ~800 KB
JSON (~150 KB gzipped). Total for all models: < 10 MB uncompressed.

**C2. Test harness**

```rust
/// Compare a CortenForge trajectory against MuJoCo reference.
///
/// Returns `TrajectoryReport` with per-step, per-field max errors and
/// the first step/field where tolerance was exceeded.
fn compare_trajectory(
    reference: &TrajectoryReference,
    tolerance: &ToleranceConfig,
) -> TrajectoryReport;
```

The harness:
1. Parses `model_xml` via `load_model()`.
2. Creates `Data` via `model.make_data()`.
3. For each step:
   a. Sets `data.ctrl` from `ctrl_sequence[step]`.
   b. Calls `data.step(&model)`.
   c. After `step()` returns — compares all fields against reference:
      `qpos`, `qvel`, `qacc`, `xpos`, `xquat`, `qfrc_bias`,
      `qfrc_passive`, `qfrc_actuator`, `qfrc_constraint`, `sensordata`,
      `energy` (`[energy_potential, energy_kinetic]`).
      These fields persist on `Data` after `step()` — see "Post-step
      field semantics" in C1 for the precise state each field reflects.
      Including `xpos`/`xquat` (FK output) enables C5 pipeline-stage
      diagnosis: if `xpos` matches but `qfrc_bias` doesn't, the bug is
      in RNE, not FK.
   d. Compares `ncon` (contact count) at each step — must match exactly.
      A contact-count mismatch is a hard failure (broad-phase or filtering
      bug) and is reported immediately without checking subsequent fields.
4. Collects `TrajectoryReport`:
   - `max_xpos_error: f64` — max over all steps/bodies (FK diagnostic)
   - `max_qpos_error: f64` — max over all steps/DOFs
   - `max_qvel_error: f64`
   - `max_qacc_error: f64`
   - `max_force_error: f64` — max over `qfrc_bias`, `qfrc_passive`,
     `qfrc_actuator` (not `qfrc_constraint` — solver-dependent)
   - `max_constraint_error: f64` — `qfrc_constraint` separately
   - `max_sensor_error: f64`
   - `max_energy_error: f64` — max over `|E_potential - ref|`, `|E_kinetic - ref|`
   - `ncon_mismatch: Option<(step, expected, actual)>` — first contact
     count mismatch (hard failure)
   - `first_violation: Option<(step, field, dof, error)>` — diagnostic
   - `divergence_step: Option<usize>` — step where error exceeds 10×
     tolerance (indicates exponential divergence, not just drift)

**C3. Tolerance configuration**

```rust
struct ToleranceConfig {
    qpos_per_step: f64,      // per-step position tolerance (also used for xpos/xquat)
    qvel_per_step: f64,      // per-step velocity tolerance
    qacc: f64,               // acceleration tolerance
    force_smooth: f64,       // qfrc_bias + qfrc_passive + qfrc_actuator
    force_constraint: f64,   // qfrc_constraint (solver-dependent)
    sensor: f64,             // sensordata
    energy: f64,             // energy_potential + energy_kinetic vs reference
}
```

`xpos` and `xquat` (FK outputs) use `qpos_per_step` — they are smooth functions
of `qpos`, so their error is bounded by the position tolerance. No separate field
needed; the harness reuses `qpos_per_step` for FK comparison and C5 diagnosis.

Default tiers (applied per model based on contact complexity and integrator):

| Scenario                     | qpos/step | qvel/step | qacc    | force_smooth | force_constraint | sensor  |
|------------------------------|-----------|-----------|---------|--------------|------------------|---------|
| No contacts (Euler/RK4)     | 1e-10     | 1e-10     | 1e-10   | 1e-10        | 1e-10            | 1e-10   |
| No contacts (Implicit*)     | 1e-8      | 1e-8      | 1e-8    | 1e-8         | 1e-10            | 1e-8    |
| Penalty constraints          | 1e-10     | 1e-10     | 1e-10   | 1e-10        | 1e-10            | 1e-8    |
| Contact (Newton)             | 1e-8      | 1e-8      | 1e-8    | 1e-8         | 1e-6             | 1e-6    |
| Contact (CG)                | 1e-7      | 1e-7      | 1e-7    | 1e-7         | 1e-5             | 1e-5    |
| Contact (PGS)               | 1e-6      | 1e-6      | 1e-6    | 1e-6         | 1e-4             | 1e-4    |

Energy tolerance (`energy` field) matches `qpos_per_step` for each tier —
energy is a smooth function of state, so it diverges at the same rate.

(*) "Implicit*" covers ImplicitFast, Implicit, and ImplicitSpringDamper.
These integrators involve implicit linear solves (Cholesky/LU) where minor
factorization path differences produce O(1e-8) per-step state deviations
even without contacts. Models T18/T19 use these tolerances.
ImplicitSpringDamper is the simplest (diagonal formula, identical to MuJoCo
at single-step `1e-10` per B10), but over 200 steps small differences
compound, so the conservative `1e-8` tier applies to all three.

Rationale:
- MuJoCo's own tests (`engine_solver_test.cc`): Newton agrees at `1e-8` to
  `1e-15`, CG at `1e-7`, PGS at `1e-6`. Our PGS is MuJoCo-aligned (pure GS,
  no SOR), so `1e-4` for `force_constraint` accounts for constraint assembly
  accumulation-order differences that compound into the state via integration.
- Smooth forces (`qfrc_bias`, `qfrc_passive`, `qfrc_actuator`) use identical
  algorithms, but in a trajectory comparison they are computed from
  *diverged states*. At step N, if `|Δqpos| ≈ ε`, then `|Δforce_smooth| ≈
  O(ε)` because forces are smooth functions of position/velocity. Therefore
  `force_smooth` tolerance matches `qpos` tolerance for each tier. (At
  step 0 the match is `1e-10` regardless — the tolerance is a worst-case
  bound over the full trajectory.)
- Euler/RK4 are explicit integrators with identical formulas — `1e-10`
  reflects only FP accumulation order differences; states stay close so
  smooth forces stay close.
- Per-step tolerance is appropriate because trajectory drift is O(ε·N) for
  stable systems — we don't need cumulative tolerance that grows with N.

**C4. Test models (trajectory set)**

Each model is purpose-built (minimal, inline MJCF) to isolate specific
pipeline interactions. Complex real-world models (Menagerie, DM Control)
are tested for *loading* (existing tests) but not for trajectory conformance
— they conflate too many features and make diagnosis impossible.

| Model ID | Description | DOF | Contacts | Actuators | Features tested |
|----------|-------------|-----|----------|-----------|-----------------|
| `T01_free_fall` | Sphere falling under gravity | 6 | 0 | 0 | FK, RNE (gravity only), Euler integration |
| `T02_pendulum_swing` | Hinge pendulum, no damping | 1 | 0 | 0 | FK, RNE (gravity + Coriolis), energy conservation |
| `T03_double_pendulum` | Two-link chain | 2 | 0 | 0 | CRBA off-diagonal, RNE Coriolis coupling |
| `T04_spring_damper` | Hinge with stiffness + damping | 1 | 0 | 0 | Passive forces, implicit integrator stability |
| `T05_motor_tracking` | Hinge + position servo | 1 | 0 | 1 | Actuation (Affine bias), ctrl → force path |
| `T06_ball_joint_free` | Ball joint + free joint | 9 | 0 | 0 | Quaternion integration, nq≠nv handling |
| `T07_sphere_on_plane` | Sphere dropped onto plane | 6 | 1 | 0 | Contact detection, normal force, Baumgarte |
| `T08_box_on_plane` | Box resting on plane | 6 | 4 | 0 | Multi-contact, friction (condim 3) |
| `T09_ant_standing` | 4-leg ant, ctrl=0 | 8 | 4+ | 8 | Multiple islands, contacts, actuators at rest |
| `T10_tendon_arm` | 3-link arm with tendon actuator | 3 | 0 | 1 | Fixed tendon kinematics, J^T force mapping |
| `T11_muscle_pendulum` | Hinge with muscle actuator | 1 | 0 | 1 | Muscle FLV, activation dynamics, FilterExact |
| `T12_humanoid_walk` | Full humanoid, gravity-comp ctrl | 27 | 8+ | 21 | Integration test: everything together |
| `T13_joint_limits` | Hinge driven into limits | 1 | 0 | 1 | Penalty limits, solref/solimp (motor drives past limit) |
| `T14_equality_weld` | Two bodies welded | 12 | 0 | 0 | Weld constraint penalty |
| `T15_friction_slide` | Box sliding on incline | 6 | 4 | 0 | Friction forces, condim 3/4/6 comparison |
| `T16_sensor_suite` | Hinge chain with all 32 sensors | 3 | 1 | 1 | All sensor types in one model |
| `T17_spatial_tendon` | Arm with sphere-wrapped tendon | 2 | 0 | 1 | Spatial tendon wrapping + Jacobian |
| `T18_implicit_stiff` | Stiff spring (k=10000) | 1 | 0 | 0 | Implicit integrators vs Euler blowup |
| `T19_rk4_accuracy` | Conservative system, long run | 2 | 0 | 0 | RK4 vs Euler energy drift comparison |
| `T20_warmstart` | Repeated contact (box bouncing) | 6 | 0-4 | 0 | Warmstart cache effectiveness |

Control inputs: `T05` uses sinusoidal `ctrl`; `T09`/`T12` use constant
gravity-compensating `ctrl`; all others use `ctrl = 0`. Exact `ctrl_sequence`
is stored in the reference JSON.

Simulation length: 200 steps at `dt = 0.002` (0.4 seconds). This is enough
to capture multiple contact events and oscillation cycles without excessive
file size.

**C5. Divergence detection and diagnosis**

When a trajectory comparison fails, the `TrajectoryReport` provides:
1. **First violation**: step number, field name, DOF index, actual vs expected
   value, and relative error.
2. **Pipeline-stage isolation**: if `qfrc_bias` diverges but `xpos`/`xquat`
   agree, the bug is in RNE. If `qfrc_constraint` diverges but all smooth
   forces agree, the bug is in the constraint solver. The test harness prints
   a diagnostic summary:
   ```
   DIVERGENCE at step 12:
     FK (xpos):           PASS  (max error 2.1e-14)
     RNE (qfrc_bias):     PASS  (max error 4.2e-11)
     Passive (qfrc_pass): PASS  (max error 0.0)
     Actuation (qfrc_act):PASS  (max error 3.1e-11)
     Constraint (qfrc_c): FAIL  (max error 8.3e-3, DOF 4)
     Energy:              PASS  (max error 1.2e-4)
     -> Root cause: constraint solver (step 12, DOF 4)
   ```
3. **Error trend**: plot-friendly output (step, max_error_per_field) so
   divergence rate can be visualized externally.

##### Layer D — Property and Invariant Tests (no MuJoCo dependency)

Property tests verify physical invariants and mathematical properties that
must hold regardless of the specific numerical values.

**D1. Momentum conservation (contact-free)**

For a single free-floating rigid body with no external forces, no gravity,
no contacts, and no actuation:
- Total linear momentum is conserved: `m * v_com = const`.
- Total angular momentum about the world-frame COM is conserved.
- Tolerance: `1e-8` for Euler (semi-implicit Euler is not exactly symplectic
  for rotational DOF due to quaternion integration), `1e-10` for RK4.
- Model: single free body with nonzero initial linear and angular velocity,
  gravity disabled.
- Steps: 500 at `dt = 0.002`.
- Note: for multi-body free-floating systems (multiple free joints),
  momentum is conserved only if internal joint forces cancel in aggregate.
  Start with the single-body case; multi-body momentum conservation can
  be added later as a stretch test.

**D2. Energy conservation (conservative system)**

For a system with springs but no damping, no contacts, no actuation:
- `total_energy() = E_kinetic + E_potential` is approximately constant.
  (Spring potential energy is included in `energy_potential` — see
  `mj_energy_pos()` which sums gravity + spring terms.)
- Euler: energy drift < `0.01 * E_0` over 1000 steps at `dt = 0.001`.
- RK4: energy drift < `1e-6 * E_0` over 1000 steps.
- ImplicitFast/Implicit/ImplicitSpringDamper: energy drift < `0.001 * E_0`
  (dissipative — all implicit integrators introduce numerical damping).

**D3. Symmetry preservation**

For a symmetric model (e.g., two identical pendulums with symmetric initial
conditions), verify that the solution preserves symmetry:
- `|qpos[left] - qpos[right]| < 1e-12` at every step.
- Tests numerical symmetry breaking from accumulation order.

**D4. Gravity-free zero acceleration**

With gravity disabled, zero velocity, no forces: `qacc == 0` everywhere.
Tolerance: `1e-14`.

**D5. Mass matrix properties**

At every configuration tested:
- Symmetric: `|M[i,j] - M[j,i]| < 1e-14`.
- Positive definite: minimum eigenvalue > 0 (or: Cholesky succeeds).
- Armature: `M[dof,dof] >= armature[dof]` for all DOFs.
- Consistent with `qLD`: `L^T * D * L == M` (within `1e-12`).

**D6. Quaternion normalization**

After every `step()`, all quaternion-valued `qpos` entries satisfy
`| ||q|| - 1.0 | < 1e-10` (absolute value of the norm deviation).
Tests quaternion drift suppression.

**D7. Contact force feasibility**

For every active contact:
- Normal force ≥ 0 (unilateral constraint).
- Friction force within cone: `||f_tangent|| ≤ μ * f_normal`, where `μ`
  is the combined sliding friction `max(geom_friction[g1].x,
  geom_friction[g2].x)` (element-wise max, matching MuJoCo's combination
  rule in `mj_contactParam`). **Note:** CortenForge currently uses geometric
  mean — the switch to element-wise max is tracked as #19a in
  [future_work_6_precursor_1.md](./future_work_6_precursor_1.md).
- Tolerance: `1e-8` (solver should satisfy these exactly up to solver
  tolerance).

##### Part E — Reference Data Generation

**E1. Python reference generator (`generate_references.py`)**

A self-contained Python script that generates all Layer B constants and
Layer C reference JSON files. Located at
`sim/L0/tests/conformance/generate_references.py`.

Requirements:
- Python ≥ 3.10, `mujoco == 3.4.0` (exact pin — `>=` would silently
  regenerate references against a newer MuJoCo, causing tolerance
  violations that look like CortenForge bugs). The script asserts
  `mujoco.__version__ == "3.4.0"` at startup and aborts on mismatch.
- Takes no arguments — regenerates all references from embedded model XML.
- Outputs:
  - `references/*.json` — Layer C trajectory files.
  - `reference_constants.txt` — Layer B hardcoded values formatted as Rust
    constants, ready to paste into test source files. Example:
    ```
    // Model: T02_pendulum_swing, qpos0, forward() only
    // MuJoCo 3.4.0
    const T02_QFRC_BIAS_QPOS0: &[f64] = &[-4.905000000000000];
    const T02_QM_QPOS0: &[f64] = &[0.083333333333333];
    ```
- Prints MuJoCo version at startup and embeds it in all output files.
- Deterministic: running twice produces identical output (no random seeds,
  no HashMap iteration).

All model XMLs must include `<option><flag energy="true"/></option>` (or the
generator must set `m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY`
after loading). Without this, MuJoCo does not compute `d.energy` and it
remains `[0, 0]`. CortenForge always computes energy (no flag gate), so this
only affects the Python reference generator.

Generator algorithm per model:
1. **Layer B constants** (per-stage, single-configuration):
   - Load model: `m = mujoco.MjModel.from_xml_string(xml)`
   - Enable energy: `m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY`
   - Create data: `d = mujoco.MjData(m)`
   - Set configuration: `d.qpos[:] = qpos_test; d.qvel[:] = qvel_test`
   - Run forward only: `mujoco.mj_forward(m, d)` — this populates all
     intermediate fields without integration.
   - Extract per-stage outputs: `d.xpos`, `d.xquat` (FK), `d.qM` (CRBA),
     `d.qfrc_bias` (RNE), `d.qfrc_passive` (passive), `d.qfrc_actuator`
     (actuation), `d.qfrc_constraint` (constraint), `d.sensordata`
     (sensors), `d.ten_length`/`d.ten_J` (tendons).
   - Format each as a Rust `const` with `// MuJoCo 3.4.0` annotation.
2. **Layer C trajectory JSON** (multi-step):
   - Load model, create data, set initial state.
   - Loop for `N` steps: `mujoco.mj_step(m, d)` — after each step, record
     a snapshot of `d.qpos`, `d.qvel`, `d.qacc`, `d.xpos`, `d.xquat`,
     `d.qfrc_bias`, `d.qfrc_passive`, `d.qfrc_actuator`,
     `d.qfrc_constraint`, `d.sensordata`, `d.ncon`,
     `[d.energy[0], d.energy[1]]`. (`qM` is omitted from trajectories —
     it's tested per-configuration in Layer B to avoid O(nv²·N) bloat.)
   - If control input is specified, set `d.ctrl[:] = ctrl_sequence[step]`
     before each `mj_step()`.
   - Write JSON with schema matching C1: `{ "mujoco_version": "3.4.0",
     "model_xml": "...", "timestep": 0.002, "n_steps": 200,
     "ctrl_sequence": [...], "steps": [ { "step": 0, ... }, ... ] }`.
3. **Output format**: all floating-point values written with 15 decimal
   digits (`f"{val:.15e}"`) to preserve full double precision.
4. **Self-validation**: after generating all references, the script runs
   MuJoCo's own `mj_compareFwdInv()` equivalent on every model (verify
   `||M*qacc - f_total||_inf < 1e-10`). This catches model XML errors that
   would produce valid-looking but physically wrong references. Also verify
   energy conservation for conservative models (T02, T03) over the first 10
   steps — energy drift should be < `1e-6` for Euler at `dt=0.002`.

**E2. Version pinning and regeneration policy**

- References are generated against **MuJoCo 3.4.0** (same version used
  for existing spatial tendon references).
- References are **checked into the repo** under
  `sim/L0/tests/conformance/references/`. They are source-controlled
  artifacts, not generated at test time.
- When upgrading the MuJoCo reference version: re-run
  `generate_references.py` with the new version, review the diff (any
  tolerance violations indicate either a MuJoCo behavior change or a
  CortenForge bug), update the version constant, commit.
- The Python script is NOT run in CI. CI only runs the Rust tests against
  the checked-in references.

**E3. No MuJoCo build dependency**

The Rust test crate (`sim-conformance-tests`) does **not** depend on
`mujoco-sys` or any MuJoCo library. References are pre-generated and
checked in. This means:
- `cargo test -p sim-conformance-tests` works on any machine without
  MuJoCo installed.
- No `--features mujoco` gate needed. All conformance tests run by default.
- The Python generator is a *development tool*, not a build dependency.

This simplifies CI, avoids cross-platform MuJoCo installation issues, and
matches the existing pattern (hardcoded constants in `spatial_tendons.rs`).

##### Part F — Test Organization and CI

**F1. Directory structure**

```
sim/L0/tests/
├── conformance/
│   ├── mod.rs                          — test module root
│   ├── self_consistency.rs             — Layer A tests (A1–A6)
│   ├── per_stage/
│   │   ├── mod.rs
│   │   ├── fk.rs                       — B1 (forward kinematics)
│   │   ├── crba.rs                     — B2 (mass matrix)
│   │   ├── rne.rs                      — B3 (bias forces)
│   │   ├── passive.rs                  — B4 (passive forces)
│   │   ├── collision.rs                — B5 (contact detection)
│   │   ├── constraint.rs               — B6 (constraint forces)
│   │   ├── actuation.rs                — B7 (actuator forces)
│   │   ├── sensors.rs                  — B8 (sensor readings)
│   │   ├── tendons.rs                  — B9 (tendon kinematics)
│   │   └── integration.rs              — B10 (integration step)
│   ├── trajectory/
│   │   ├── mod.rs
│   │   ├── harness.rs                  — TrajectoryReport, compare_trajectory()
│   │   ├── models.rs                   — T01–T20 inline MJCF + ctrl sequences
│   │   └── tests.rs                    — C4 trajectory comparison tests
│   ├── properties/
│   │   ├── mod.rs
│   │   ├── conservation.rs             — D1 (momentum), D2 (energy)
│   │   ├── invariants.rs               — D3–D7 (symmetry, quat, etc.)
│   │   └── determinism.rs              — A3 (placed here, not in self_consistency.rs,
│   │                                      because determinism is a property test that
│   │                                      uses `step()` replay rather than comparing
│   │                                      two different code paths)
│   ├── references/
│   │   ├── T01_free_fall.json          — Layer C reference data
│   │   ├── T02_pendulum_swing.json
│   │   ├── ...
│   │   └── T20_warmstart.json
│   └── generate_references.py          — E1 Python generator
├── integration/                        — existing integration tests (unchanged)
│   ├── spatial_tendons.rs              — keep (already follows Layer B pattern)
│   ├── derivatives.rs                  — keep
│   ├── sleeping.rs                     — keep (referenced by A6)
│   └── ...
└── mujoco_conformance/
    └── mod.rs                          — existing stub (update to re-export conformance/)
```

**F2. Existing test migration**

Existing tests in `integration/` that already follow the conformance pattern
are **not** moved — they stay where they are. The conformance suite
*references* them in its index but does not duplicate them:
- `spatial_tendons.rs` tests 16–18 → Layer B (tendon kinematics, B9)
- `derivatives.rs` → already validates analytical vs FD (not cross-impl)
- `sleeping.rs` Phase C → Layer A (self-consistency, A6)
- `validation.rs` FK/CRBA tests → Layer B (augmented, not replaced)

**F3. CI integration**

All conformance tests run as part of `cargo test -p sim-conformance-tests`
in the existing `quality-gate.yml` workflow. No separate workflow needed.
No MuJoCo installation required.

The `generate_references.py` script runs only during development, on a
machine with `pip install mujoco==3.4.0`. It is not part of CI.

**F4. Test naming convention**

All conformance tests use a structured naming scheme for greppability:
```
conformance_a1_fwd_inv_double_pendulum
conformance_a3_determinism_ant
conformance_b1_fk_humanoid_qpos0
conformance_b6_constraint_newton_box_on_plane
conformance_c_trajectory_T07_sphere_on_plane
conformance_d2_energy_conservation_rk4
```

Pattern: `conformance_{layer}{number}_{description}`.

##### Part G — Incremental Rollout Plan

The four layers are independently implementable and deliver value at each
step. The recommended implementation order prioritizes immediate value:

**Phase 1 (immediate, no MuJoCo needed):**
- Layer A (self-consistency): A1, A3, A4, A5 — catches bugs today.
- Layer D (properties): D1, D2, D4, D5, D6, D7 — catches bugs today.
- Estimated: ~800 LOC tests, ~200 LOC harness utilities.

**Phase 2 (requires MuJoCo Python for one-time reference generation):**
- Part E: write `generate_references.py`.
- Layer B: per-stage tests B1–B10 with hardcoded constants.
- Estimated: ~300 LOC Python, ~1500 LOC tests.

**Phase 3 (trajectory infrastructure):**
- Layer C: harness, JSON parsing, all T01–T20 trajectory tests.
- Estimated: ~600 LOC harness, ~800 LOC tests, ~5 MB reference data.

**Phase 4 (polish):**
- Layer A: A2 (island/monolithic — uses existing `DISABLE_ISLAND` flag), A6.
- Layer D: D3 (symmetry).
- CI verification, documentation update.

Each phase is a standalone PR. Phase 1 can ship immediately. Phase 2
requires a one-time run of the Python generator on a dev machine. Phase 3
is the full trajectory infrastructure. Phase 4 is optional polish.

#### Acceptance Criteria

**Layer A — Self-Consistency (no external dependency):**
1. A1: forward/inverse equivalence passes at `1e-10` for ≥5 models (Euler/RK4
   integrators only) at ≥3 configurations each, with correct
   `qfrc_frictionloss` handling for Newton-solved vs explicit paths.
2. A2: island/monolithic equivalence for ≥2 contact models with multiple
   islands (Phase 4 — uses `DISABLE_ISLAND` flag to force monolithic path).
3. A3: bit-identical replay for ≥4 models over 100 steps.
4. A4: energy ordering `|ΔE_RK4| < |ΔE_Implicit| < |ΔE_Euler|` for ≥2
   conservative systems.
5. A5: sparse/dense mass matrix agreement at `1e-12` for ≥5 models.
6. A6: sleep bit-identity referenced from existing `sleeping.rs` Phase C
   tests (Phase 4 — extend to new pipeline stages as needed).

**Layer B — Per-Stage Reference (MuJoCo 3.4.0):**
7. B1: FK matches MuJoCo at `1e-10` for ≥5 models at ≥3 configurations.
8. B2: CRBA matches MuJoCo at `1e-10` for ≥4 models.
9. B3: RNE matches MuJoCo at `1e-10` for ≥3 models with nonzero velocity.
10. B4: passive forces match at `1e-10` for ≥3 models.
11. B5: contact detection matches (count exact, geometry `1e-6`) for ≥5
    contact scenarios.
12. B6: constraint forces match at solver-appropriate tolerance for ≥3
    solver types.
13. B7: actuator forces match at `1e-10` for ≥5 actuator types.
14. B8: all 32 sensor types match at sensor-appropriate tolerance.
15. B9: tendon kinematics extended beyond existing `spatial_tendons.rs`.
16. B10: single-step integration matches for all 5 integrator types.

**Layer C — Trajectory Comparison:**
17. ≥15 trajectory models (T01–T15 minimum) with 200-step references.
18. No-contact trajectories (T01–T06) pass at `1e-10` per-step for
    Euler/RK4, `1e-8` for implicit integrators.
19. Contact trajectories with Newton solver pass at `1e-8` per-step.
20. Contact trajectories with CG solver pass at `1e-7` per-step.
21. PGS contact trajectories pass at `1e-6` per-step.
22. Divergence diagnostics correctly identify the first failing pipeline
    stage in at least one intentionally-broken scenario (manual test).

**Layer D — Properties:**
23. Momentum conservation (D1) at `1e-8` for Euler, `1e-10` for RK4
    (single free body, 500 steps).
24. Energy conservation (D2) within bounds for all 5 integrators.
25. Zero-gravity zero-acceleration (D4) at `1e-14`.
26. Mass matrix properties (D5) verified for ≥5 models.
27. Quaternion normalization (D6) at `1e-10` after 1000 steps.
28. Contact feasibility (D7) for ≥3 contact scenarios.

**Infrastructure:**
29. `generate_references.py` runs successfully with `mujoco==3.4.0` and
    produces deterministic output.
30. All conformance tests pass in `cargo test -p sim-conformance-tests`
    without MuJoCo installed.
31. No existing test in `integration/` is broken or moved.

#### Implementation Notes

**Relative error function.** Use MuJoCo's corrected relative error formula
(from `engine_derivative_test.cc`) rather than `approx::assert_relative_eq`:
```rust
fn relative_error(a: f64, b: f64, abs_tol: f64) -> f64 {
    let diff = (a - b).abs();
    if diff <= abs_tol { return 0.0; }
    (diff - abs_tol) / (a.abs() + b.abs() + abs_tol)
}
```
This handles near-zero values gracefully (avoids division by near-zero
denominator). Use `abs_tol = 1e-12` as the floor.

**Accessing intermediate pipeline state.** Layer C wants to compare
`qfrc_bias`, `qfrc_passive`, etc. at each step. These fields are already
populated by `forward()` and persist on `Data` after `step()` returns —
they are *not* cleared between steps. So the harness can read them after
`step()`. The only subtlety is that `qacc` is computed during `forward()`
then used by `integrate()` — both before and after integration, `qacc` is
the same value. No API changes needed.

**Quaternion convention.** Both MuJoCo and CortenForge store quaternions
in `[w, x, y, z]` order. The MuJoCo Python reference generator records
`d.xquat` in this order, and the JSON stores `[w, x, y, z]` arrays.
CortenForge's `xquat` is `Vec<UnitQuaternion<f64>>` (nalgebra), which
*internally* uses `[x, y, z, w]` storage. The C2 harness must extract
components via `q.w`, `q.i`, `q.j`, `q.k` (not raw array indexing)
when comparing against reference `[w, x, y, z]` values.

**Contact ordering.** CortenForge and MuJoCo may produce contacts in
different order (different broad-phase traversal). Layer B5 must sort
contacts by geom pair ID then by position before comparison. If contact
*count* differs, that's a hard failure — no tolerance.

**Warmstart cache and determinism.** The `efc_lambda` warmstart cache uses
`HashMap<WarmstartKey, Vec<f64>>`. HashMap iteration order is
non-deterministic across runs (random seed). This does NOT affect physics
determinism because the cache is only *read* during constraint assembly
(deterministic key lookup) and *written* after solve. A3 (determinism)
must verify this claim by comparing physics outputs, not cache internals.

**Existing `spatial_tendons.rs` pattern.** The established pattern for
Layer B tests is: embed MJCF as `const`, run MuJoCo 3.4.0 on a dev
machine, hardcode reference values as `const` with `// MuJoCo 3.4.0`
annotation, use `assert_relative_eq!` with explicit `epsilon`. Layer B
follows this exact pattern — no JSON files for per-stage tests (only
Layer C trajectories use JSON, because they have too much data for inline
constants).

**`forward()` is public.** `Data::forward(&model)` is already public
(used in `validation.rs` tests). Layer B tests call `forward()` directly
(not `step()`) to inspect intermediate state without integration.

**Example test function (Layer C):**
```rust
#[test]
fn conformance_c_trajectory_T07_sphere_on_plane() {
    let reference = load_reference("references/T07_sphere_on_plane.json");
    let tolerance = ToleranceConfig::contact_newton();
    let report = compare_trajectory(&reference, &tolerance);
    assert!(
        report.passed(),
        "T07 sphere_on_plane failed:\n{}",
        report.diagnostic_summary()
    );
}
```
Each trajectory test is ~5 lines: load JSON, pick tolerance tier, compare,
assert with diagnostic output on failure.

#### Files

- `sim/L0/tests/conformance/mod.rs` — conformance module root
- `sim/L0/tests/conformance/self_consistency.rs` — Layer A (A1–A6)
- `sim/L0/tests/conformance/per_stage/*.rs` — Layer B (B1–B10, 10 files)
- `sim/L0/tests/conformance/trajectory/harness.rs` — `TrajectoryReport`,
  `compare_trajectory()`, `ToleranceConfig`
- `sim/L0/tests/conformance/trajectory/models.rs` — T01–T20 MJCF + ctrl
- `sim/L0/tests/conformance/trajectory/tests.rs` — Layer C tests
- `sim/L0/tests/conformance/properties/*.rs` — Layer D (3 files)
- `sim/L0/tests/conformance/references/*.json` — Layer C reference data
  (checked in, ~5 MB total)
- `sim/L0/tests/conformance/generate_references.py` — Python generator
- `sim/L0/tests/integration/` — **unchanged** (existing tests stay)

---

### 20. Contact Margin/Gap Runtime Effect
⚠️ **Migrated to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #20.**
Full specification is now tracked there as a prerequisite to #19.

---

### 21. Noslip Post-Processor for PGS/CG Solvers
⚠️ **Migrated to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #21.**
Full specification is now tracked there as a prerequisite to #19.

---

### 22. Body-Transmission Actuators (Adhesion)
⚠️ **Migrated to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #22.**
Full specification is now tracked there as a prerequisite to #19.
