# Future Work 6 — Phase 3A: Parser Fundamentals + Foundation (Items #18–22)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items are ordered by implementation dependency: `<include>` + `<compiler>` (#18)
unlocks production model loading. Parser fundamentals (#19–22) fix MJCF parsing
gaps that cause incorrect model compilation — these must be correct before any
runtime physics testing is meaningful.

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

### 19. `<frame>` Element Parsing
**Status:** Not started | **Effort:** S–M | **Prerequisites:** None

#### Current State

No `MjcfFrame` type, no `b"frame"` match in parser. Silently dropped by catch-all
`skip_element()`. Models using `<frame>` for coordinate system convenience (common
in complex humanoid MJCF files) silently lose frame transformations.

#### Objective

Parse `<frame>` elements within `<body>` and apply their coordinate transformations
to child elements.

#### Specification

1. **MJCF semantics**: `<frame>` defines a coordinate frame transformation within
   `<body>`. All child elements of the `<frame>` (geoms, joints, sites, other
   frames) have their positions/orientations interpreted relative to the frame's
   pose, NOT relative to the parent body directly.
2. **Implementation**: `<frame>` does NOT create a new body. It is a
   pre-processing transformation:
   - Parse `pos` and orientation (quat/euler/axisangle/xyaxes/zaxis) of the frame.
   - For each child element, compose the frame's transformation with the child's
     local transformation to get the child's pose relative to the parent body.
   - Recursively handle nested `<frame>` elements.
3. **Expansion approach**: Like `<include>`, this can be handled as a
   pre-processing step: expand `<frame>` by transforming children's coordinates,
   then remove the `<frame>` wrapper. The model builder never sees `<frame>`.
4. **Children allowed inside `<frame>`**: `<body>`, `<geom>`, `<joint>`, `<site>`,
   `<camera>`, `<light>`, `<frame>` (nested).

#### Acceptance Criteria

1. `<frame pos="1 0 0"><geom pos="0 0 0" .../></frame>` places the geom at
   body-relative position (1, 0, 0).
2. `<frame euler="0 0 90"><geom pos="1 0 0" .../></frame>` rotates the geom's
   position by 90° about Z.
3. Nested frames compose correctly.
4. Models without `<frame>` are unaffected (regression).
5. A real-world humanoid MJCF using `<frame>` loads with correct geometry placement.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — frame parsing and coordinate transformation

---

### 20. `childclass` Attribute
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed. Documented as "Not in scope" in `future_work_2.md`. `MjcfBody` has no
`childclass` field. The default class resolver already supports hierarchical
lookup — just needs the inherited class name threaded through body parsing.

#### Objective

Parse `childclass` from `<body>` and use it as the default class for all child
elements within that body subtree.

#### Specification

1. **MJCF parsing**: Parse `childclass` (string) from `<body>` elements.
2. **Class resolution**: When resolving defaults for a child element (geom, joint,
   site, etc.) within a body that has `childclass="X"`:
   - If the child element has an explicit `class="Y"`, use class Y.
   - If the child element has no explicit class, use class X (from `childclass`).
   - `childclass` is inherited by sub-bodies: if body A has `childclass="X"` and
     child body B has no `childclass`, B's children also default to class X.
   - If body B has its own `childclass="Y"`, it overrides A's for B's subtree.
3. **Existing infrastructure**: The default class resolver already supports
   class lookup by name. The change is threading the `childclass` name through
   body parsing so it's used as the fallback when no explicit `class` is specified.

#### Acceptance Criteria

1. `<body childclass="robot"><geom .../></body>` applies class "robot" defaults
   to the geom.
2. `<body childclass="robot"><geom class="special" .../></body>` uses class
   "special" (explicit overrides childclass).
3. Nested bodies inherit parent's `childclass` unless overridden.
4. Models without `childclass` are unaffected (regression).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse childclass, thread through body
  parsing as default class fallback

---

### 21. `<site>` euler/axisangle/xyaxes/zaxis Orientation
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Site parsing only supports `quat`. `parse_site_attrs()` handles only the `quat`
attribute. Model builder comments: "sites have only quat, no euler." Body and geom
elements already support all orientation formats (euler, axisangle, xyaxes, zaxis,
quat).

#### Objective

Support all MuJoCo orientation attributes on `<site>` elements, matching the
existing body/geom orientation parsing.

#### Specification

1. **Reuse existing orientation parsing**: The `parse_orientation()` helper (or
   equivalent) used for body/geom elements already handles `euler`, `axisangle`,
   `xyaxes`, `zaxis`, and `quat`. Apply the same logic to site parsing.
2. **Compiler angle convention**: Respect `compiler.angle` (degree vs radian)
   for euler and axisangle, just as body/geom parsing does.
3. **Priority**: MuJoCo's attribute priority when multiple orientation attributes
   are specified: `quat` > `axisangle` > `xyaxes` > `zaxis` > `euler`. Match
   this priority.

#### Acceptance Criteria

1. `<site euler="90 0 0"/>` produces the correct orientation quaternion.
2. `<site axisangle="0 0 1 90"/>` produces the correct orientation quaternion.
3. `<site xyaxes="0 1 0 -1 0 0"/>` produces the correct orientation quaternion.
4. `<site zaxis="0 0 1"/>` produces the correct orientation quaternion.
5. `<site quat="..."/>` continues to work (regression).
6. Sensors attached to sites with non-quat orientation produce correct readings.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — `parse_site_attrs()` extension

---

### 22. Tendon `springlength` MJCF Parsing
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Pipeline has `tendon_lengthspring` field and auto-computes it from `tendon_length0`.
But the MJCF parser does NOT parse the `springlength` attribute from XML. Zero
matches for `springlength` in `parser.rs`. The runtime field exists but can't be
set from MJCF.

#### Objective

Parse `springlength` from `<tendon>` children (`<fixed>`, `<spatial>`) and use it
to override the auto-computed spring rest length.

#### Specification

1. **MJCF parsing**: Parse `springlength` (float or 2-element array) from
   `<fixed>` and `<spatial>` tendon elements. MuJoCo supports both scalar
   (sets both min/max to the same value) and 2-element (min, max for
   length-dependent spring).
2. **Model builder**: When `springlength` is explicitly provided, store it in
   `tendon_lengthspring` instead of auto-computing from `tendon_length0`.
3. **Auto-compute fallback**: When `springlength` is NOT provided (default),
   preserve existing behavior (compute from initial tendon length at qpos0).

#### Acceptance Criteria

1. `<spatial springlength="0.5"/>` sets the spring rest length to 0.5.
2. `<spatial springlength="0.3 0.7"/>` sets min/max spring rest lengths.
3. Without `springlength`, auto-computation from qpos0 is preserved.
4. A pre-tensioned tendon (springlength ≠ initial length) produces correct
   spring forces.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse springlength attribute
